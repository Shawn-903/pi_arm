#!/usr/bin/env python3
"""Python/CLI integration against an isolated commissioning fixture on vcan0.

Requires pi_arm_tasks and a writable test software_zero_file. Changes only the
virtual fixture's reference. Use a dedicated ROS domain, never a physical CAN service.
"""

import json
import math
import subprocess

import rclpy
from pi_arm_interfaces.msg import HardwareState
from pi_arm_tasks.zero_reference import ZeroReferenceClient
from rclpy.qos import qos_profile_sensor_data

from test_software_zero_vcan import wait


def main():
    info = json.loads(subprocess.check_output(['ip', '-d', '-j', 'link', 'show', 'vcan0']))[0]
    assert info['linkinfo']['info_kind'] == 'vcan'
    rclpy.init()
    node = rclpy.create_node('software_zero_clients_test')
    samples = []
    node.create_subscription(HardwareState, '/pi_arm/hardware_state', samples.append, qos_profile_sensor_data)
    try:
        wait(node, lambda: samples and all(samples[-1].motor_fresh))
        assert samples[-1].commissioning and samples[-1].can_interface == 'vcan0', 'virtual maintenance service required'
        client = ZeroReferenceClient(node)
        assert client.wait_for_service()
        future = client.set_zero_reference([0.0] * 6, reference_pose_confirmed=True)
        wait(node, future.done)
        result = future.result()
        assert result.success, result.message
        wait(node, lambda: list(samples[-1].software_output_offsets_rad) == list(result.software_output_offsets_rad))
        assert max(abs(value) for value in samples[-1].positions) < 1e-8
        print('PASS Python ZeroReferenceClient: confirmed reference applied through task service', flush=True)

        command = ['ros2', 'run', 'pi_arm_tasks', 'pi_arm_zero', '--confirm-reference']
        packing = json.loads(subprocess.check_output(command, text=True, timeout=12))
        assert packing['success'], packing
        wait(node, lambda: list(samples[-1].software_output_offsets_rad) == packing['software_output_offsets_rad'])
        expected = [math.radians(v) for v in (0, 100.7, -85.6, 0, 0, 0)]
        assert len(samples[-1].positions) == len(expected)
        assert all(abs(a - b) < 1e-6 for a, b in zip(samples[-1].positions, expected))
        print('PASS CLI: default mechanical zero matches the packing pose', flush=True)

        output = subprocess.check_output([
            *command, '--reference-deg', '1', '0', '0', '0', '0', '0',
        ], text=True, timeout=12)
        cli = json.loads(output)
        assert cli['success'], cli
        wait(node, lambda: list(samples[-1].software_output_offsets_rad) == cli['software_output_offsets_rad'])
        assert abs(samples[-1].positions[0] - math.radians(1)) < 1e-8
        print('PASS CLI: degrees converted to radians; returned reference matches live feedback', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
