#!/usr/bin/env python3
"""在独立 ROS 域启动 mock 控制链，验证标定采样 CLI 与中断停止。"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from pi_arm_interfaces.msg import RobotState, TaskState
from pi_arm_interfaces.srv import ManageMotor, StopMotion
from rclpy.signals import SignalHandlerOptions


def wait(node, condition, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise AssertionError("timed out waiting for mock calibration task")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=110)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--case", choices=("all", "success", "interrupt"), default="all")
    args = parser.parse_args()
    work = args.log_dir or Path(tempfile.mkdtemp(prefix="pi-arm-calibration-motion-"))
    work.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("calibration_motion_mock_test")
    states = []
    node.create_subscription(RobotState, "/pi_arm/state", states.append, 10)
    enable = node.create_client(ManageMotor, "/pi_arm/enable_motor")
    stop = node.create_client(StopMotion, "/pi_arm/stop_motion")
    launch = interrupted = None
    with (work / "launch.log").open("w") as log:
        try:
            launch = subprocess.Popen([
                "ros2", "launch", "pi_arm_bringup", "bringup.launch.py",
                "hardware_type:=mock", "use_rviz:=false", "use_websocket:=false",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait(node, lambda: states and states[-1].hardware.calibrated and enable.service_is_ready())
            assert states[-1].hardware.calibration_id == "MOCK", "mock fixture required"
            future = enable.call_async(ManageMotor.Request(motor_id=0))
            wait(node, future.done)
            assert future.result().success, future.result().message
            wait(node, lambda: states[-1].state == RobotState.READY)

            plan = work / "poses.json"
            if args.case in ("all", "success"):
                plan.write_text(json.dumps([
                    {"name": "P1", "joint_positions_deg": [0] * 6, "dwell_sec": 0.1},
                    {"name": "P2", "joint_positions_deg": [0.5, 0, 0, 0, 0, 0], "dwell_sec": 0.1},
                ]))
                output = work / "samples.jsonl"
                completed = subprocess.run([
                    "ros2", "run", "pi_arm_tasks", "pi_arm_calibration_motion", "--plan", str(plan),
                    "--output", str(output), "--motion-timeout-sec", "20",
                ], capture_output=True, text=True, timeout=45)
                (work / "cli.log").write_text(completed.stdout + completed.stderr)
                assert completed.returncode == 0, completed.stdout + completed.stderr
                samples = [record for record in map(json.loads, output.read_text().splitlines()) if record["type"] == "sample"]
                assert [sample["pose_name"] for sample in samples] == ["P1", "P2"]
                assert math.isclose(samples[-1]["feedback_joint_positions_rad"][0], math.radians(0.5),
                                    abs_tol=math.radians(0.2))
                assert all(sample["calibration_id"] == "MOCK" and sample["task_id"] > 0 for sample in samples)
                wait(node, lambda: states[-1].task.task_id == samples[-1]["task_id"])
                print("PASS calibration CLI: two mock poses, arrival, dwell and matching feedback", flush=True)

            if args.case in ("all", "interrupt"):
                plan.write_text(json.dumps([
                    {"name": "long", "joint_positions_deg": [10, 0, 0, 0, 0, 0], "dwell_sec": 3},
                    {"name": "later", "joint_positions_deg": [0] * 6},
                ]))
                interrupted_output = work / "interrupted.jsonl"
                previous_task = states[-1].task.task_id
                # 向任务进程发信号，避免只终止 ros2 run 包装进程。
                executable = Path(get_package_prefix("pi_arm_tasks")) / "lib/pi_arm_tasks/pi_arm_calibration_motion"
                interrupted = subprocess.Popen([
                    str(executable), "--plan", str(plan), "--output", str(interrupted_output), "--speed-deg-s", "1",
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
                wait(node, lambda: states[-1].task.task_id != previous_task and states[-1].task.status == TaskState.RUNNING)
                interrupted.send_signal(signal.SIGTERM)
                stdout, stderr = interrupted.communicate(timeout=15)
                (work / "interrupted-cli.log").write_text(stdout + stderr)
                assert interrupted.returncode != 0
                result = list(map(json.loads, interrupted_output.read_text().splitlines()))[-1]
                assert result["type"] == "result" and not result["success"] and result["stop_confirmed"] is True, result
                assert result["samples"] == 0
                wait(node, lambda: states[-1].task.status != TaskState.RUNNING and states[-1].state == RobotState.READY)
                print("PASS calibration CLI: SIGTERM confirms stop and skips subsequent poses", flush=True)
        finally:
            if interrupted is not None and interrupted.poll() is None:
                interrupted.send_signal(signal.SIGTERM)
                try:
                    interrupted.communicate(timeout=15)
                except subprocess.TimeoutExpired:
                    interrupted.kill()
                    interrupted.communicate()
            if states and states[-1].task.status == TaskState.RUNNING and stop.service_is_ready():
                future = stop.call_async(StopMotion.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if launch is not None and launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=10)
            node.destroy_node()
            rclpy.try_shutdown()
    print(f"Logs: {work}", flush=True)


if __name__ == "__main__":
    main()
