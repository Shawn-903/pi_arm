#!/usr/bin/env python3
"""Software-zero regression using the production monitor and virtual CAN only.

Source the tested install first. Requires unused vcan0; all records stay in /tmp.
"""

import argparse
import contextlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from pi_arm_interfaces.msg import HardwareState
from pi_arm_interfaces.srv import AxisTest, SetZeroReference
from rclpy.qos import qos_profile_sensor_data

from test_motion_safety_vcan import Emulator
from vcan_test_model import calibration_mapping

ROOT = Path(__file__).resolve().parents[1]


def wait(node, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if condition():
            return
    raise AssertionError("timed out waiting for software-zero test condition")


def call(node, client, request):
    future = client.call_async(request)
    wait(node, future.done)
    return future.result()


@contextlib.contextmanager
def monitor(work, params):
    executable = Path(get_package_prefix('pi_arm_hardware')) / 'lib/pi_arm_hardware/pi_arm_hardware_monitor_node'
    with (work / 'monitor.log').open('a') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params)],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            yield
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
            assert process.returncode == 0, (work / 'monitor.log').read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all', 'capture'), default='all')
    args = parser.parse_args()
    info = json.loads(subprocess.check_output(['ip', '-d', '-j', 'link', 'show', 'vcan0']))[0]
    assert info['linkinfo']['info_kind'] == 'vcan', 'virtual CAN only'
    os.environ['ROS_DOMAIN_ID'] = '84'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init()
    node = rclpy.create_node('software_zero_regression')
    samples = []
    node.create_subscription(HardwareState, '/pi_arm/hardware_state', samples.append, qos_profile_sensor_data)
    client = node.create_client(SetZeroReference, '/pi_arm/commissioning/set_zero_reference')
    axis = node.create_client(AxisTest, '/pi_arm/commissioning/axis_test')
    emulator = Emulator()
    angles = [180.0, -360.0, 540.0, -120.0, 60.0, -30.0]
    emulator.reset(angles)
    try:
        with tempfile.TemporaryDirectory(prefix='pi-arm-software-zero-') as folder:
            work = Path(folder)
            zero = work / 'references/zero.yaml'
            base = work / 'base.yaml'
            cable = ROOT / 'src/pi_arm_description/config/cable_kinematics.yaml'
            mapping, fingerprint = calibration_mapping(cable)
            data = yaml.safe_load((ROOT / 'src/pi_arm_hardware/config/pack_reference_calibration.template.yaml').read_text())
            data['calibration'].update(
                approved=True, robot_id='VCAN-ONLY', calibration_id='SOFTWARE-ZERO-VCAN-ONLY',
                packing_pose_joint_rad=[0.0] * 6, raw_output_at_packing_rad=[0.0] * 6,
                configuration_fingerprint=fingerprint,
            )
            base.write_text(yaml.safe_dump(data))
            base_before = base.read_bytes()
            description = subprocess.check_output([
                'xacro', str(ROOT / 'src/pi_arm_description/urdf/pi_arm.urdf.xacro'),
                'backend:=real', 'can_interface:=vcan0', f'cable_kinematics_file:={cable}',
                f'calibration_file:={base}', f'software_zero_file:={zero}',
            ], text=True)
            params = work / 'monitor.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                'robot_description': description, 'commissioning': True,
            }}}))
            request = SetZeroReference.Request(reference_joint_positions_rad=[0.0] * 6)
            with monitor(work, params):
                wait(node, lambda: client.service_is_ready() and samples and all(samples[-1].motor_fresh))
                assert not call(node, client, request).success
                assert not zero.exists()
                request.reference_pose_confirmed = True
                result = call(node, client, request)
                assert result.success, result.message
                saved = zero.read_bytes()
                offsets = list(result.software_output_offsets_rad)
                wait(node, lambda: list(samples[-1].software_output_offsets_rad) == offsets)
                assert max(abs(value) for value in samples[-1].positions) < 1e-8
                for index, actuator in enumerate(mapping):
                    expected = -math.radians(angles[index]) / (actuator['protocol_angle_ratio'] * actuator['motor_direction'])
                    assert abs(result.software_output_offsets_rad[index] - expected) < 1e-9
                assert list(result.raw_protocol_positions_deg) == angles
                assert not any(emulator.frames_for(code) for code in (0x19, 0x95, 0xA4, 0x88, 0x80)), 'zeroing wrote a motor command'
                print('PASS capture: confirmed pose, unchanged raw readings, zero model feedback, no motor writes', flush=True)

                with emulator.lock:
                    emulator.drop_feedback.add(3)
                assert samples[-1].motor_fresh[2]
                rejected = call(node, client, request)
                assert not rejected.success, "a cached encoder angle was accepted for a new capture"
                assert zero.read_bytes() == saved
                with emulator.lock:
                    emulator.drop_feedback.clear()
                count = len(samples)
                wait(node, lambda: len(samples) > count and all(samples[-1].motor_fresh)
                     and samples[-1].motor_ages_sec[2] < 0.15)

                zero.parent.chmod(0o500)
                try:
                    result = call(node, client, request)
                    assert not result.success, result.message
                finally:
                    zero.parent.chmod(0o700)
                assert zero.read_bytes() == saved and list(samples[-1].software_output_offsets_rad) == offsets
                print('PASS stale feedback and persistence failure leave the committed reference unchanged', flush=True)
                if args.case == 'capture':
                    return

                start = call(node, axis, AxisTest.Request(operation='start', space='motor', axis_id=1,
                                                         delta_deg=1.0, speed_deg_s=5.0))
                assert start.success, start.message
                rejected = call(node, client, request)
                assert not rejected.success and rejected.code == 2003, rejected.message
                stopped = call(node, axis, AxisTest.Request(operation='stop', test_id=start.test_id))
                assert stopped.success, stopped.message
                assert zero.read_bytes() == saved
                print('PASS active task rejects reference changes', flush=True)
            samples.clear()
            with monitor(work, params):
                wait(node, lambda: samples and list(samples[-1].software_output_offsets_rad) == offsets and all(samples[-1].motor_fresh))
                assert max(abs(value) for value in samples[-1].positions) < 1e-8
                assert zero.read_bytes() == saved and base.read_bytes() == base_before
                assert not emulator.frames_for(0x19) and not emulator.frames_for(0x95)
                print('PASS process restart reloads software zero; base calibration unchanged', flush=True)
    finally:
        emulator.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
