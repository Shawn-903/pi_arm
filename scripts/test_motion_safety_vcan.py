#!/usr/bin/env python3
"""Regression tests against production RealBackend; requires an unused, UP vcan0.

Source install/setup.bash after building. This script cannot select physical CAN.
It compiles a temporary probe and uses synthetic zero-offset calibration only.
"""
import contextlib
import json
import math
from pathlib import Path
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time

import yaml
from vcan_motor_emulator import CAN_BASE, CAN_FRAME, Motor, response

ROOT = Path(__file__).resolve().parents[1]


class Emulator:
    def __init__(self):
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.bus.bind(('vcan0',))
        self.bus.settimeout(.05)
        self.lock = threading.Lock()
        self.running = True
        self.reset()
        self.thread = threading.Thread(target=self.serve)
        self.thread.start()

    def reset(self, angles=None, dynamic=False):
        with self.lock:
            self.motors = {i: Motor(enabled=True) for i in range(1, 7)}
            self.positions = {i: float(v) for i, v in enumerate(angles or [0]*6, 1)}
            self.targets = dict(self.positions)
            self.speeds = dict.fromkeys(self.positions, 0)
            self.dynamic = dynamic
            self.drop_stop_ack = set()
            self.drop_feedback = set()
            self.frames = []
            self.last = time.monotonic()

    def inject(self, motor, protocol_angle):
        with self.lock:
            self.positions[motor] = protocol_angle

    def frames_for(self, opcode):
        with self.lock:
            return [(i, data) for i, data in self.frames if data[0] == opcode]

    def assert_stopped(self, ids):
        assert {i for i, _ in self.frames_for(0x81)} == set(ids)
        with self.lock:
            first = next(n for n, (_, data) in enumerate(self.frames) if data[0] == 0x81)
            assert not any(data[0] == 0xa4 for _, data in self.frames[first+1:]), 'A4 after STOP'

    def serve(self):
        while self.running:
            try:
                raw = self.bus.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            can_id, length, data = CAN_FRAME.unpack(raw)
            motor = can_id - CAN_BASE
            if length != 8 or motor not in self.motors:
                continue
            with self.lock:
                now = time.monotonic()
                elapsed = now - self.last
                self.last = now
                for i, state in self.motors.items():
                    delta = self.targets[i] - self.positions[i]
                    if self.dynamic:
                        self.positions[i] += math.copysign(min(abs(delta), self.speeds[i]*elapsed), delta)
                    state.angle_raw = round(self.positions[i]*100)
                    state.speed_dps = (int(math.copysign(self.speeds[i], delta))
                                       if self.dynamic and abs(delta) > .005 else 0)
                self.frames.append((motor, data))
                if data[0] == 0xa4:
                    self.targets[motor] = struct.unpack('<i', data[4:])[0]/100
                    self.speeds[motor] = struct.unpack('<H', data[2:4])[0]
                    reply = response(self.motors[motor], bytes([0x9c])+bytes(7))
                else:
                    if data[0] == 0x81:
                        self.targets[motor] = self.positions[motor]
                        self.speeds[motor] = 0
                    reply = response(self.motors[motor], data)
                if ((data[0] == 0x81 and motor in self.drop_stop_ack) or
                        (data[0] in (0x92, 0x9a, 0x9c) and motor in self.drop_feedback)):
                    reply = None
            if reply is not None:
                self.bus.send(CAN_FRAME.pack(CAN_BASE+motor, 8, reply))

    def close(self):
        self.running = False
        self.thread.join()
        self.bus.close()


def readline(process):
    assert select.select([process.stdout], [], [], 5)[0], 'probe response timed out'
    line = process.stdout.readline().strip()
    assert line, f'probe exited: {process.poll()}'
    return line


def command(process, text, ok=True):
    process.stdin.write(text+'\n')
    process.stdin.flush()
    line = readline(process)
    print(line, flush=True)
    values = dict(part.split('=', 1) for part in line.split(' detail=')[0].split())
    assert values['ok'] == str(int(ok)), line
    return values, line


@contextlib.contextmanager
def backend(executable, calibration, commissioning=False):
    process = subprocess.Popen([str(executable), str(ROOT), str(calibration),
                                'commissioning' if commissioning else 'normal'],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        assert readline(process) == 'READY'
        yield process
    finally:
        if process.poll() is None:
            process.stdin.write('quit\n')
            process.stdin.flush()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise


def settle(process):
    for _ in range(40):
        values, line = command(process, 'update')
        if values['active'] == '0':
            assert 'succeeded' in line, line
            return values
    raise AssertionError('test failed to settle')


def main():
    link = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', 'vcan0']))
    assert link[0].get('linkinfo', {}).get('info_kind') == 'vcan', 'requires virtual CAN'
    with tempfile.TemporaryDirectory(prefix='pi_arm_regression_') as directory:
        work = Path(directory)
        calibration = work/'zero.yaml'
        data = yaml.safe_load((ROOT/'src/pi_arm_hardware/config/pack_reference_calibration.R02.recorded.yaml').read_text())
        data['calibration'].update(robot_id='VCAN-ONLY', calibration_id='VCAN-ONLY-ZERO',
                                  packing_pose_joint_rad=[0.0]*6, raw_output_at_packing_rad=[0.0]*6)
        calibration.write_text(yaml.safe_dump(data))
        packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
        executable = work/'probe'
        subprocess.run(['g++', '-std=c++17', str(ROOT/'src/pi_arm_hardware/test/vcan_backend_probe.cpp'),
                        *['-I'+str(ROOT/'src'/p/'include') for p in packages],
                        *['-L'+str(ROOT/'install'/p/'lib') for p in packages],
                        *['-l'+p for p in packages], '-pthread', '-o', str(executable)], check=True)
        emulator = Emulator()
        try:
            print('CASE: joint feedback limit inhibits normal control')
            outside = -6*180/math.pi*(33.94/16.25)*math.sin(math.radians(35))
            emulator.reset([0, 0, 0, outside, 0, 0])
            with backend(executable, calibration) as p:
                values, _ = command(p, 'read')
                assert values['kin'] == values['control'] == '0'
                assert values['error'] == 'JOINT_POSITION_LIMIT'
                assert float(values['q5_deg']) > 30
                assert not emulator.frames_for(0xa4)

            print('CASE: confirmed normal STOP, wrong lease and concurrent writer')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'disarm')  # Explicit Stop also covers targets from a previous process.
                emulator.assert_stopped(range(1, 7))
                assert not emulator.frames_for(0xa4)
                with emulator.lock:
                    emulator.frames.clear()
                command(p, 'write')
                command(p, 'wrong_lease', ok=False)
                assert not emulator.frames_for(0x81)
                command(p, 'race')
                emulator.assert_stopped(range(1, 7))
                command(p, 'disarm')
                emulator.assert_stopped(range(1, 7))  # Repeated Stop cannot submit a new target.

            print('CASE: missing one STOP ACK latches failure until explicit successful retry')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                emulator.drop_stop_ack = {6}
                command(p, 'disarm', ok=False)
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '1' and values['control'] == '0'
                command(p, 'arm', ok=False)
                sent = len(emulator.frames_for(0x81))
                command(p, 'read')
                assert len(emulator.frames_for(0x81)) == sent, 'read must not retry failed stop'
                emulator.drop_stop_ack = set()
                command(p, 'disarm')
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '0' and values['control'] == '1'
                command(p, 'write')
                command(p, 'disarm')

            print('CASE: fault stop starts from read without waiting 500 ms for ACK')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                emulator.drop_stop_ack = set(range(1, 7))
                emulator.inject(4, outside)
                values, _ = command(p, 'read')
                assert values['kin'] == values['control'] == '0'
                assert int(values['read_us']) < 100000
                time.sleep(.55)
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '1'
                emulator.assert_stopped(range(1, 7))
                emulator.drop_stop_ack = set()
                command(p, 'disarm')

            print('CASE: lost raw feedback revokes normal control and sends STOP')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                emulator.drop_feedback = {6}
                time.sleep(1.05)
                values, _ = command(p, 'read')
                assert values['control'] == '0'
                time.sleep(.05)
                emulator.assert_stopped(range(1, 7))
                emulator.drop_feedback = set()

            print('CASE: unrepresentable raw speed rejected; representable speed rounded down')
            emulator.reset(dynamic=True)
            with backend(executable, calibration, True) as p:
                _, line = command(p, 'motor 4 .1 .05', ok=False)
                assert 'resolution' in line and not emulator.frames_for(0xa4)
                command(p, 'motor 4 .1 .2')
                settle(p)
                frames = emulator.frames_for(0xa4)
                assert len(frames) == 1 and frames[0][0] == 4
                assert struct.unpack('<H', frames[0][1][2:4])[0]/6 <= .2
                emulator.assert_stopped([4])

            print('CASE: near-limit coupled test rejected before any A4')
            near = -6*180/math.pi*(33.94/16.25)*math.sin(math.radians(29.99))
            emulator.reset([0, 0, 0, near, 0, 0])
            with backend(executable, calibration, True) as p:
                _, line = command(p, 'joint 2 .1 .2', ok=False)
                assert 'asynchronous' in line and not emulator.frames_for(0xa4)

            print('CASE: coupled test completes with independently moving quantized motors')
            emulator.reset(dynamic=True)
            with backend(executable, calibration, True) as p:
                command(p, 'joint 2 .05 .2')
                values = settle(p)
                assert abs(float(values['q2_deg'])-.05) < .001
                assert abs(float(values['q5_deg'])) < .002
                assert {i for i, _ in emulator.frames_for(0xa4)} == {2, 3, 4, 5, 6}
                emulator.assert_stopped([2, 3, 4, 5, 6])

            print('CASE: path departure stops even while joint limits still valid')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'joint 2 .1 .2')
                emulator.inject(4, .02)  # opposite to the accepted negative protocol direction
                values, line = command(p, 'update')
                assert values['kin'] == '1' and values['active'] == '0'
                assert 'validated asynchronous path' in line
                emulator.assert_stopped([2, 3, 4, 5, 6])

            print('CASE: joint limit crossed during active test triggers STOP')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'joint 2 .1 .2')
                emulator.inject(4, outside)
                values, _ = command(p, 'update')
                assert values['kin'] == values['active'] == '0'
                assert values['error'] == 'JOINT_POSITION_LIMIT'
                emulator.assert_stopped([2, 3, 4, 5, 6])

            print('CASE: commissioning stop ACK loss prevents restart until retry')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'motor 3 .1 .2')
                emulator.drop_stop_ack = {3}
                _, line = command(p, 'stop', ok=False)
                assert 'STOP UNCONFIRMED' in line
                command(p, 'motor 3 .1 .2', ok=False)
                emulator.drop_stop_ack = set()
                command(p, 'stop')
                command(p, 'motor 3 .1 .2')
                command(p, 'stop')

            print('CASE: heartbeat loss still stops bounded test')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'motor 3 .1 .2')
                time.sleep(.85)
                values, line = command(p, 'noheartbeat')
                assert values['active'] == '0' and 'heartbeat expired' in line
                emulator.assert_stopped([3])
            print('PASS: 12 production-backend virtual CAN safety scenarios')
        finally:
            emulator.close()


if __name__ == '__main__':
    main()
