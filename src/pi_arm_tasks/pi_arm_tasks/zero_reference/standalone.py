"""启动硬件和标零服务，采集一次软件零点，再退出本次启动的服务。"""

import argparse
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from pi_arm_interfaces.msg import HardwareState
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
import xacro
import yaml

from .client import ZeroReferenceClient, zero_request
from .files import resolve_reference_files


def _wait(node, condition, children, stopped, timeout, description, *, interruptible=True):
    deadline = time.monotonic() + timeout
    while rclpy.ok():
        if interruptible and stopped.is_set():
            raise RuntimeError("zero capture interrupted before submission")
        for name, process in children:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited with code {process.returncode}; see service log")
        if condition():
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f"timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError("ROS context shut down")


def _stop_services(children):
    errors = []
    for name, process in reversed(children):
        forced = False
        for signum, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            if process.poll() is not None:
                break
            try:
                forced = signum == signal.SIGKILL
                process.send_signal(signum)
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
        # 启动期间尚未安装 ROS 信号处理器时，退出信号会直接终止子进程。
        if forced or process.returncode not in (0, -signal.SIGINT, -signal.SIGTERM):
            errors.append(f"{name}: exit={process.returncode}, forced={forced}")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-reference", action="store_true", required=True,
                        help="confirm the arm is physically at the specified reference pose")
    parser.add_argument("--reference-deg", type=float, nargs=6, default=[0.0, 100.7, -85.6, 0.0, 0.0, 0.0])
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--calibration-file", type=Path)
    parser.add_argument("--cable-kinematics-file", type=Path)
    parser.add_argument("--software-zero-file", type=Path)
    parser.add_argument("--startup-timeout", type=float, default=15.0,
                        help="seconds to wait for services and stationary feedback (default: 15)")
    args = parser.parse_args()
    children = []
    node = None
    stopped = threading.Event()
    previous_handlers = {}
    submitted = False
    result = {"success": False}
    try:
        reference = [math.radians(value) for value in args.reference_deg]
        zero_request(reference, args.confirm_reference)
        if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
            raise ValueError("startup-timeout must be finite and positive")
        socket.if_nametoindex(args.can_interface)
        description = Path(get_package_share_directory("pi_arm_description"))
        calibration, zero = resolve_reference_files(
            args.can_interface, args.calibration_file, args.software_zero_file, capture=True)
        cable = (args.cable_kinematics_file or description / "config/cable_kinematics.yaml").expanduser().resolve()
        if not cable.is_file():
            raise FileNotFoundError(cable)
        if calibration != zero and not calibration.is_file():
            raise FileNotFoundError(calibration)
        result["software_zero_file"] = str(zero)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: stopped.set())

        # 本次服务使用独立名称，客户端和反馈订阅只连接本次启动的进程。
        namespace = "/pi_arm_zero_" + uuid.uuid4().hex
        remaps = ["--ros-args", "-r", f"__ns:={namespace}"]
        for name in ("hardware_state", "tasks/set_zero_reference", "commissioning/set_zero_reference",
                     "commissioning/axis_test", "commissioning/enable_motor", "commissioning/disable_motor",
                     "commissioning/reset_motor", "commissioning/reboot_motor"):
            remaps += ["-r", f"/pi_arm/{name}:={namespace}/{name}"]
        robot_description = xacro.process_file(str(description / "urdf/pi_arm.urdf.xacro"), mappings={
            "backend": "real", "can_interface": args.can_interface,
            "calibration_file": str(calibration), "cable_kinematics_file": str(cable),
            "software_zero_file": str(zero), "allow_real_control": "false",
        }).toxml()
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("pi_arm_zero_once", cli_args=remaps, use_global_arguments=False)
        latest = [None, 0.0]

        def on_state(state):
            latest[:] = [state, time.monotonic()]

        def ready():
            state, received = latest
            return (state is not None and time.monotonic() - received < 0.5
                    and state.can_interface == args.can_interface and state.commissioning
                    and state.motor_test_permitted and client.wait_for_service(timeout_sec=0.0))

        node.create_subscription(HardwareState, "/pi_arm/hardware_state", on_state, qos_profile_sensor_data)
        client = ZeroReferenceClient(node)
        with tempfile.TemporaryDirectory(prefix="pi-arm-zero-") as folder:
            params = Path(folder) / "monitor.yaml"
            params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
                "robot_description": robot_description, "commissioning": True,
            }}}), encoding="utf-8")
            try:
                for package, executable, extra in (
                    ("pi_arm_hardware", "pi_arm_hardware_monitor_node", ["--params-file", str(params)]),
                    ("pi_arm_tasks", "pi_arm_zero_reference", []),
                ):
                    if stopped.is_set():
                        raise RuntimeError("zero capture interrupted before submission")
                    path = Path(get_package_prefix(package)) / "lib" / package / executable
                    process = subprocess.Popen([str(path), *remaps, *extra], stdout=sys.stderr,
                                               stderr=sys.stderr, start_new_session=True)
                    children.append((executable, process))
                print("等待标零服务和电机静止反馈……", file=sys.stderr, flush=True)
                _wait(node, ready, children, stopped, args.startup_timeout, "services and stationary motor feedback")
                if stopped.is_set():
                    raise RuntimeError("zero capture interrupted before submission")
                future = client.set_zero_reference(reference, reference_pose_confirmed=True)
                submitted = True
                # 发出请求后等待这一笔结果；中断不会触发重复采集。
                _wait(node, future.done, children, stopped, 6.0, "zero-reference result", interruptible=False)
                response = future.result()
                result.update(success=response.success, code=response.code, message=response.message,
                              reference_id=response.reference_id,
                              raw_protocol_positions_deg=list(response.raw_protocol_positions_deg),
                              software_output_offsets_rad=list(response.software_output_offsets_rad))
            finally:
                errors = _stop_services(children)
                result["cleanup_success"] = not errors
                if errors:
                    result["cleanup_errors"] = errors
    except Exception as error:
        result.update(success=False, message=str(error))
        if submitted:
            result["result_unknown"] = True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["success"] and result.get("cleanup_success") else 1
