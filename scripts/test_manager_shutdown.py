#!/usr/bin/env python3
"""Manager shutdown regression with synthetic hardware/actions in ROS domain 98.

Source the install under test, then run with --case and --log-dir. Starts only
manager; reuses the arrival fixture and never launches a CAN driver.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_msgs.action import MoveGroup
from pi_arm_interfaces.action import MoveJ
from pi_arm_interfaces.srv import PreviewMotion
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import xacro
import yaml

from test_arrival_integration import Fixture, DirectMove, FollowJointTrajectory, NAMES, await_future


class ShutdownFixture(Fixture):
    def __init__(self, case):
        super().__init__()
        self.case = case
        self.arm_pending = threading.Event()
        self.controller_running = threading.Event()
        self.planner_running = threading.Event()
        self.planner_canceled = threading.Event()
        self.end_fixture = threading.Event()
        self.move_server.destroy()
        self.move_server = ActionServer(
            self, MoveGroup, '/move_action', execute_callback=self.plan,
            cancel_callback=self.cancel_plan, callback_group=self.group)
        self.move_client = ActionClient(self, MoveJ, '/pi_arm/movej', callback_group=self.group)

    def arm(self, request, response):
        if request.armed:
            self.arm_pending.set()
            if self.case == 'arming':
                time.sleep(.6)
        elif self.case == 'stop_failure':
            self.arm_requests.append((False, request.hold_position, request.lease_id))
            self.events.append(('arm_response', False, request.hold_position, request.lease_id, False))
            response.success = False
            response.armed = self.armed
            response.message = 'synthetic stop acknowledgement failure'
            return response
        return super().arm(request, response)

    def accept_follow(self, _request):
        self.controller_requested.set()
        if self.case == 'late_accept':
            time.sleep(.7)
        elif self.case == 'expired_accept':
            time.sleep(1.5)
        return GoalResponse.ACCEPT

    def follow(self, handle):
        self.goal = handle.request
        self.controller_running.set()
        while not handle.is_cancel_requested and not self.end_fixture.wait(.01):
            pass
        if handle.is_cancel_requested:
            self.controller_canceled.set()
            handle.canceled()
        else:
            handle.abort()
        return FollowJointTrajectory.Result(error_code=0)

    def cancel_plan(self, _handle):
        self.events.append(('planner_cancel',))
        return CancelResponse.ACCEPT

    def plan(self, handle):
        assert handle.request.planning_options.plan_only
        self.planner_running.set()
        while not handle.is_cancel_requested and not self.end_fixture.wait(.01):
            pass
        if handle.is_cancel_requested:
            self.planner_canceled.set()
            handle.canceled()
        else:
            handle.abort()
        return MoveGroup.Result()


def manager_parameters():
    desc = Path(get_package_share_directory('pi_arm_description'))
    moveit = Path(get_package_share_directory('pi_arm_moveit_config'))
    return {
        'robot_description': xacro.process_file(str(desc / 'urdf/pi_arm.urdf.xacro'), mappings={
            'backend': 'mock', 'cable_kinematics_file': str(desc / 'config/cable_kinematics.yaml')}).toxml(),
        'robot_description_semantic': (moveit / 'config/pi_arm.srdf').read_text(),
        'robot_description_kinematics': yaml.safe_load((moveit / 'config/kinematics.yaml').read_text()),
        'robot_description_planning': yaml.safe_load((moveit / 'config/joint_limits.yaml').read_text()),
        'hardware_state_topic': '/test/hardware_state',
        'follow_joint_trajectory_action': '/test/follow',
        'operation_timeout_sec': 1.0,
        'arrival_stable_time_sec': .2, 'arrival_timeout_sec': 1.0,
    }


def run(case, log_dir):
    assert os.environ.get('ROS_DOMAIN_ID') == '98', 'Run only in isolated domain 98'
    rclpy.init(domain_id=98)
    node = ShutdownFixture(case)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    process = None
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'manager.yaml'
            config.write_text(yaml.safe_dump({'pi_arm_manager': {'ros__parameters': manager_parameters()}}))
            executable = Path(get_package_prefix('pi_arm_manager')) / 'lib/pi_arm_manager/pi_arm_manager_node'
            log_path = log_dir / f'{case}.log'
            with log_path.open('w') as log:
                process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config)],
                                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            assert node.client.wait_for_server(timeout_sec=30), 'manager not ready'
            time.sleep(.4)
            goal = DirectMove.Goal(joint_names=NAMES, positions=[.1] * 6,
                                   max_velocity=math.radians(1), max_output_velocity=math.radians(1))
            if case == 'idle':
                result = None
            elif case == 'preview':
                assert node.preview.wait_for_service(timeout_sec=5)
                request = PreviewMotion.Request(mode='lin', max_linear_velocity=.01,
                                                max_linear_acceleration=.02,
                                                max_angular_velocity=math.radians(1))
                request.target.position.x = .2
                request.target.position.z = .3
                request.target.orientation.w = 1.
                result = node.preview.call_async(request)
                assert node.planner_running.wait(5), 'preview did not reach planner'
            else:
                client = node.move_client if case == 'planning' else node.client
                assert client.wait_for_server(timeout_sec=5)
                if case == 'planning':
                    goal = MoveJ.Goal(joint_names=NAMES, positions=[.1] * 6,
                                     max_velocity=math.radians(1), max_acceleration=math.radians(2))
                handle = await_future(client.send_goal_async(goal))
                assert handle.accepted, case
                result = handle.get_result_async()
                ready = {'arming': node.arm_pending, 'late_accept': node.controller_requested,
                         'expired_accept': node.controller_requested,
                         'planning': node.planner_running}.get(case, node.controller_running)
                assert ready.wait(5), f'{case}: did not reach shutdown boundary'
                if case == 'expired_accept':
                    assert await_future(result, 3).result.code == 3001, 'expected goal acceptance timeout'
                    time.sleep(.05)  # Include the failed worker's lease guard.
            started = time.monotonic()
            exit_signal = signal.SIGTERM if case in ('arming', 'planning', 'stop_failure') else signal.SIGINT
            os.kill(process.pid, exit_signal)
            if case == 'arming':
                time.sleep(.1)
                rejected = await_future(node.client.send_goal_async(goal))
                assert not rejected.accepted, 'new task accepted during shutdown'
            returncode = process.wait(timeout=10)
            elapsed = time.monotonic() - started
            assert returncode == (1 if case == 'stop_failure' else 0), (case, returncode, log_path.read_text())
            reply = await_future(result, 3) if result is not None else None
            if case == 'idle':
                assert not node.arm_requests, node.arm_requests
            elif case == 'preview':
                assert not reply.success, reply
            else:
                assert reply.status == GoalStatus.STATUS_ABORTED, reply
                expected_code = 3001 if case == 'expired_accept' else 3104
                assert not reply.result.success and reply.result.code == expected_code, reply
            releases = [request for request in node.arm_requests if not request[0]]
            if case in ('idle', 'preview'):
                assert not node.arm_requests, node.arm_requests
            elif case == 'expired_accept':
                lease = node.arm_requests[0][2]
                assert releases and all(request == (False, False, lease) for request in releases), node.arm_requests
            else:
                assert releases and all(request == (False, False, 0) for request in releases), node.arm_requests
            if case != 'stop_failure':
                assert not node.armed, 'lease survived manager exit'
            if case in ('planning', 'preview'):
                assert node.planner_canceled.wait(1), 'planner goal survived manager exit'
                assert not any(request[0] for request in node.arm_requests), node.arm_requests
                cancel = ('planner_cancel',)
            elif node.controller_requested.is_set():
                assert node.controller_canceled.wait(1), 'controller goal survived manager exit'
                cancel = ('controller_cancel',)
            else:
                cancel = None  # Shutdown can win before a controller goal is sent.
            if cancel and case != 'preview':
                lease = node.arm_requests[0][2] if case == 'expired_accept' else 0
                assert ('arm_response', False, False, lease, case != 'stop_failure') in \
                    node.events[:node.events.index(cancel)], node.events
            output = log_path.read_text()
            assert 'workers and requests drained' in output, output
            for failure in ('SEVERE WARNING', 'Segmentation fault', 'terminate called', 'context is invalid'):
                assert failure not in output, output
            print(f'PASS {case}: {exit_signal.name}, exit={returncode}, {elapsed:.2f}s, '
                  f'releases={len(releases)}', flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        node.end_fixture.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True,
                        choices=('active', 'arming', 'late_accept', 'expired_accept',
                                 'planning', 'preview', 'stop_failure', 'idle'))
    parser.add_argument('--log-dir', required=True, type=Path)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    run(args.case, args.log_dir)
