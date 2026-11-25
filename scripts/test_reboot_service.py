#!/usr/bin/env python3
"""Exercise the WebSocket reboot command with a real manager and a synthetic motor."""
import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
import websockets
import yaml
from ament_index_python.packages import get_package_prefix
from pi_arm_interfaces.srv import ManageMotor
from pi_arm_websocket.gateway import RosBridge, WebSocketGateway
from rclpy.executors import MultiThreadedExecutor

from test_arrival_integration import Fixture
from test_manager_shutdown import manager_parameters


async def check_replies(bridge, fixture, hardware_service):
    confirmed = 'fresh feedback confirmed; motor disabled'
    rejected = 'reboot feedback recovered but position reference unverified'
    calls = []
    next_request_started = threading.Event()

    def reboot(request, response):
        calls.append(request.motor_id)
        index = len(calls)
        if index == 3:
            # Deliver this reply only after the next request has reached the hardware.
            next_request_started.wait(12)
        else:
            if index == 4:
                next_request_started.set()
            time.sleep(.65)
        response.success = index != 2
        response.code = 0 if response.success else 2002
        response.message = (confirmed, rejected, 'late reply', 'next reboot confirmed')[index - 1]
        return response

    fixture.create_service(ManageMotor, hardware_service, reboot, callback_group=fixture.group)
    gateway = WebSocketGateway(bridge)
    try:
        async with websockets.serve(gateway._client, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
                expected = [(0, confirmed), (2002, rejected), (2002, 'hardware service timeout'),
                            (0, 'next reboot confirmed')]
                for msg_id, (code, message) in enumerate(expected, start=1):
                    start = time.monotonic()
                    await ws.send(json.dumps({'type': 'req', 'msg_id': msg_id,
                                              'cmd': 'reboot_motor', 'params': {'motor_id': 6}}))
                    reply = json.loads(await asyncio.wait_for(ws.recv(), 12))
                    assert (reply['type'], reply['msg_id'], reply['code']) == ('rsp', msg_id, code), reply
                    detail = reply['ms'] if code else reply['data']['message']
                    assert detail == message, reply
                    print(f'PASS {time.monotonic() - start:.3f}s  {detail}', flush=True)
        assert calls == [6, 6, 6, 6] and not fixture.arm_requests and fixture.follow_count == 0
    finally:
        next_request_started.set()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '83', 'Run only in isolated ROS domain 83'
    prefix = f'/test/reboot_{os.getpid()}'
    api = f'/pi_arm/reboot_motor:={prefix}/reboot_motor'
    hardware = f'{prefix}/hardware/reboot_motor'
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        os.environ['ROS_LOG_DIR'] = str(root / 'log')
        rclpy.init(domain_id=83, args=['--ros-args', '-r', api, '-p', 'hardware_type:=mock',
                                       '-p', f'points_file:={root}/points.json'])
        fixture = Fixture()
        bridge = RosBridge(connect_execution=False)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(fixture)
        executor.add_node(bridge)
        spin = threading.Thread(target=executor.spin)
        spin.start()
        process = None
        try:
            params = manager_parameters()
            params['operation_timeout_sec'] = 5.0
            config = root / 'manager.yaml'
            config.write_text(yaml.safe_dump({'pi_arm_manager': {'ros__parameters': params}}))
            executable = Path(get_package_prefix('pi_arm_manager')) / 'lib/pi_arm_manager/pi_arm_manager_node'
            process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config),
                                        '-r', api, '-r', f'/pi_arm/hardware/reboot_motor:={hardware}'])
            assert bridge.service_clients['reboot_motor'].wait_for_service(timeout_sec=30)
            deadline = time.monotonic() + 10
            samples = set()
            while time.monotonic() < deadline:
                state = bridge.state_snapshot()
                if state and state.state == state.READY:
                    stamp = state.hardware.stamp
                    samples.add((stamp.sec, stamp.nanosec))
                    if len(samples) >= 3:
                        break
                time.sleep(.05)
            else:
                raise AssertionError('manager did not receive stationary mock feedback')
            asyncio.run(check_replies(bridge, fixture, hardware))
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            executor.shutdown()
            spin.join()
            bridge.destroy_node()
            fixture.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
