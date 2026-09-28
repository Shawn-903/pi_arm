#!/usr/bin/env python3
"""Regression tests against production RealBackend; requires an unused, UP vcan0.

Source install/setup.bash after building. This script cannot select physical CAN.
It compiles a temporary probe and uses synthetic zero-offset calibration only.
"""
import contextlib
import json
import math
import runpy
from pathlib import Path
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time

import yaml
from ament_index_python.packages import get_package_prefix
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
            self.drop_position_ack = set()
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
                    reply = bytes([0xa4]) + response(self.motors[motor], bytes([0x9c])+bytes(7))[1:]
                else:
                    if data[0] == 0x81:
                        self.targets[motor] = self.positions[motor]
                        self.speeds[motor] = 0
                    reply = response(self.motors[motor], data)
                if ((data[0] == 0x81 and motor in self.drop_stop_ack) or
                        (data[0] in (0x92, 0x9a, 0x9c) and motor in self.drop_feedback) or
                        (data[0] == 0xa4 and motor in self.drop_position_ack)):
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


def settle(process, attempts=40):
    for _ in range(attempts):
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
        data = yaml.safe_load((ROOT/'docs/calibration-history/pack_reference_calibration.R02.recorded.yaml').read_text())
        data['calibration'].update(robot_id='VCAN-ONLY', calibration_id='VCAN-ONLY-ZERO',
                                  packing_pose_joint_rad=[0.0]*6, raw_output_at_packing_rad=[0.0]*6)
        cable_file = ROOT/'src/pi_arm_description/config/cable_kinematics.yaml'
        directions = yaml.safe_load(cable_file.read_text())['cable_kinematics'].get(
            'output_directions', {f'motor_{i:02d}': 1 for i in range(1, 7)})
        signs = [directions[f'motor_{i:02d}'] for i in range(1, 7)]
        swapped = yaml.safe_load(cable_file.read_text())['cable_kinematics'].get('swap_wrist_channels', False)
        j4_motor = 5 if swapped else 4
        axes = yaml.safe_load(cable_file.read_text())['cable_kinematics']['axes']
        wrist_gains = {motor: axes[name]['joint_radius_mm'] / axes[name]['motor_radius_mm']
                       for motor, name in ((4, 'joint_05'), (5, 'joint_06'))}
        candidate = runpy.run_path(str(ROOT/'src/pi_arm_hardware/scripts/pi_arm_calibration_candidate'))
        mapping = [dict(joint_name=name, motor_id=i, protocol_angle_ratio=36 if i <= 3 else 6,
                        motor_direction=direction)
                   for i, (name, direction) in enumerate(zip(candidate['JOINT_NAMES'], [1, 1, -1, -1, -1, 1]), 1)]
        data['calibration']['configuration_fingerprint'] = candidate['configuration_fingerprint'](cable_file, mapping)
        calibration.write_text(yaml.safe_dump(data))
        packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
        executable = work/'probe'
        subprocess.run(['g++', '-std=c++17', str(ROOT/'src/pi_arm_hardware/test/vcan_backend_probe.cpp'),
                        *['-I'+str(ROOT/'src'/p/'include') for p in packages],
                        *['-L'+str(Path(get_package_prefix(p))/'lib') for p in packages],
                        *['-l'+p for p in packages], '-pthread', '-o', str(executable)], check=True)
        emulator = Emulator()
        try:
            print('CASE: normal wire speed ceiling, low-speed fallback and tolerant start projection')
            for cap in (5.0, 10.0, .2, 1.5):
                emulator.reset()
                with backend(executable, calibration) as p:
                    command(p, f'write_speed {cap}')
                    time.sleep(.1)
                    frames = emulator.frames_for(0xa4)
                    assert len(frames) == 6
                    for motor, data in frames:
                        speed = struct.unpack('<H', data[2:4])[0]
                        assert 1 <= speed <= math.floor(cap*(36 if motor <= 3 else 6)+1e-9)
                    command(p, 'disarm')

            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write_speed 10.001', ok=False)
                assert not emulator.frames_for(0xa4), 'over-limit request emitted A4'

            print('CASE: feedback allowance preserves measured angles, command bounds stay strict')
            def protocol_from_joint_degrees(q2, q3):
                coupling = 28.5 / 16.25
                original = [0, q2*36, -(51.75/16.25*q3-coupling*q2)*36,
                        -coupling*q2*6, -coupling*q2*6,
                        .5*(20.25/12.25+28.5/17.25)*q2*6]
                return [value*sign for value, sign in zip(original, signs)]

            emulator.reset(protocol_from_joint_degrees(101.7, 0))
            with backend(executable, calibration) as p:
                command(p, 'recover 5')
                time.sleep(.1)
                sent = dict(emulator.frames_for(0xa4))
                assert len(sent) == 6
                # Interpolated q2=101.6 is accepted as recovery, but the actual
                # A4 target is the standard 100.7-degree command boundary.
                assert abs(struct.unpack('<i', sent[2][4:])[0]/100/36 - 100.7) < .0003
                command(p, 'disarm')

            for axis, bound in ((2, 100.7), (3, 85.6)):
                for sign in (-1, 1):
                    for extra, expected_valid in ((1.0, '1'), (2.99, '1'), (3.01, '0')):
                        angle = sign*(bound+extra)
                        emulator.reset(protocol_from_joint_degrees(
                            angle if axis == 2 else 0, angle if axis == 3 else 0))
                        with backend(executable, calibration) as p:
                            values, _ = command(p, 'read')
                            assert values['kin'] == values['control'] == expected_valid
                            if expected_valid == '1':
                                assert values['error'] == 'JOINT_COMMAND_LIMIT_WARNING'
                                assert abs(float(values[f'q{axis}_deg'])-angle) < .002
                            else:
                                assert values['error'] in ('JOINT_POSITION_LIMIT', 'OUTPUT_POSITION_LIMIT')
                            assert not emulator.frames_for(0xa4)
                    emulator.reset()
                    with backend(executable, calibration) as p:
                        command(p, 'write')
                        deadline = time.monotonic() + 1
                        while len(emulator.frames_for(0xa4)) < 6 and time.monotonic() < deadline:
                            time.sleep(.01)
                        sent = len(emulator.frames_for(0xa4))
                        assert sent == 6
                        command(p, f'write_target {axis} {sign*(bound+1)}', ok=False)
                        assert len(emulator.frames_for(0xa4)) == sent

            print('CASE: joint feedback limit inhibits normal control')
            outside = -signs[j4_motor-1]*6*180/math.pi*(wrist_gains[j4_motor])*math.sin(math.radians(35))
            angles = [0]*6
            angles[j4_motor-1] = outside
            emulator.reset(angles)
            with backend(executable, calibration) as p:
                values, _ = command(p, 'read')
                assert values['kin'] == values['control'] == '0'
                assert values['error'] == 'JOINT_POSITION_LIMIT'
                assert float(values['q5_deg']) > 30
                assert not emulator.frames_for(0xa4)

            print('CASE: low-level release STOP, wrong lease and concurrent writer')
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
                emulator.inject(j4_motor, outside)
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
            near = -signs[j4_motor-1]*6*180/math.pi*(wrist_gains[j4_motor])*math.sin(math.radians(29.99))
            angles = [0]*6
            angles[j4_motor-1] = near
            emulator.reset(angles)
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
                emulator.inject(4, signs[3]*.02)  # opposite to accepted J2 compensation
                values, line = command(p, 'update')
                assert values['kin'] == '1' and values['active'] == '0'
                assert 'validated asynchronous path' in line
                emulator.assert_stopped([2, 3, 4, 5, 6])

            print('CASE: joint limit crossed during active test triggers STOP')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'joint 2 .1 .2')
                emulator.inject(j4_motor, outside)
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

            print('CASE: heartbeat loss stops and holds bounded test')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'motor 3 .1 .2')
                time.sleep(.85)
                values, line = command(p, 'noheartbeat')
                assert values['active'] == '0' and 'heartbeat expired' in line
                assert not emulator.frames_for(0x81), 'healthy heartbeat stop must hold'
                assert [i for i, _ in emulator.frames_for(0xa4)] == [3, 3]

            print('CASE: stop during dwell captures current position and is idempotent')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                command(p, 'hold')
                command(p, 'stop_hold')
                sent = len(emulator.frames_for(0xa4))
                assert sent == 12 and not emulator.frames_for(0x81)
                command(p, 'stop_hold')
                command(p, 'release_old')
                command(p, 'write_target 1 20')
                command(p, 'read')
                assert len(emulator.frames_for(0xa4)) == sent
                assert not emulator.frames_for(0x81), 'old cleanup released hold'

            print('CASE: Stop before enabling never sends A4 or latches a false hold fault')
            emulator.reset()
            with emulator.lock:
                for motor in emulator.motors.values():motor.enabled = False
            with backend(executable, calibration) as p:
                command(p, 'stop_hold')
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '0'
                assert not emulator.frames_for(0xa4)

            print('CASE: moving soft stop fences concurrent writes and holds measured position')
            emulator.reset(dynamic=True)
            with backend(executable, calibration) as p:
                command(p, 'write')
                command(p, 'write_target 1 10')
                time.sleep(.15)
                old_goal = emulator.targets[1]
                command(p, 'hold_race')
                held = dict(emulator.frames_for(0xa4)[-6:])
                assert abs(struct.unpack('<i', held[1][4:])[0]/100 - old_goal) > 1
                assert not emulator.frames_for(0x81)
                command(p, 'release_old')
                command(p, 'read')
                assert not emulator.frames_for(0x81)
                for motor, frame in held.items():
                    assert abs(struct.unpack('<i', frame[4:])[0]/100 - emulator.positions[motor]) <= .11
                command(p, 'disarm')  # Fault/shutdown release remains available.
                assert {i for i, _ in emulator.frames_for(0x81)} == set(range(1, 7))

            print('CASE: failed hold ACK inhibits motion; explicit retry survives read-loop races')
            emulator.reset(dynamic=True)
            with backend(executable, calibration) as p:
                command(p, 'write')
                emulator.drop_position_ack = {6}
                command(p, 'stop_hold', ok=False)
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '1' and values['control'] == '0'
                command(p, 'arm', ok=False)
                emulator.drop_position_ack = set()
                releases = len(emulator.frames_for(0x81))
                command(p, 'hold_race')
                values, _ = command(p, 'read')
                assert values['stop_failed'] == '0' and values['control'] == '1'
                assert len(emulator.frames_for(0x81)) == releases, 'stale read released newly confirmed hold'

            print('CASE: missing feedback cannot create a stop hold target')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                emulator.drop_feedback = {6}
                time.sleep(.25)
                sent = len(emulator.frames_for(0xa4))
                command(p, 'stop_hold', ok=False)
                assert len(emulator.frames_for(0xa4)) == sent
                assert emulator.frames_for(0x81)

            print('CASE: commissioning operator stop holds tested motor across subsequent reads')
            emulator.reset(dynamic=True)
            with backend(executable, calibration, True) as p:
                command(p, 'motor 4 5 1')
                command(p, 'axis_hold')
                command(p, 'read')
                command(p, 'axis_hold')
                assert not emulator.frames_for(0x81)
                assert [i for i, _ in emulator.frames_for(0xa4)] == [4, 4]

            for motor, ratio, sign in [(1, 36, 1), (4, 6, -1)]:
                print(f'CASE: CAN {motor} output step 5 deg at 1 deg/s, ratio {ratio}')
                emulator.reset(dynamic=True)
                with backend(executable, calibration, True) as p:
                    for delta, speed in [(5.01, 1), (-5.01, 1), (5, 10.01)]:
                        command(p, f'motor {motor} {delta} {speed}', ok=False)
                    # Enlarging raw motor tests does not enlarge joint requests.
                    command(p, 'joint 1 .51 .2', ok=False)
                    command(p, 'joint 1 .1 10.01', ok=False)
                    assert not emulator.frames_for(0xa4)
                    command(p, f'motor {motor} {sign*5} 1')
                    settle(p, attempts=80)  # Heartbeat stays alive during the 5-second move.
                    frames = emulator.frames_for(0xa4)
                    assert len(frames) == 1 and frames[0][0] == motor
                    data = frames[0][1]
                    assert struct.unpack('<i', data[4:])[0] == sign*5*ratio*100
                    assert struct.unpack('<H', data[2:4])[0] == ratio
                    with emulator.lock:
                        assert abs(emulator.positions[motor]/ratio-sign*5) < .001
                        assert all(abs(value) < .001 for i, value in emulator.positions.items() if i != motor)
                    emulator.assert_stopped([motor])

            # Both commissioning spaces accept the default and upper boundary.
            for space, axis in [('motor', 4), ('joint', 1)]:
                for speed in (5, 10):
                    emulator.reset(dynamic=True)
                    with backend(executable, calibration, True) as p:
                        command(p, f'{space} {axis} .1 {speed}')
                        settle(p)
                        frames = emulator.frames_for(0xa4)
                        assert frames
                        assert all(1 <= struct.unpack('<H', data[2:4])[0] <= 10*(36 if motor <= 3 else 6)
                                   for motor, data in frames)

            for sign in (1, -1):
                print(f'CASE: signed session boundary {sign*30} output deg from nonzero origin')
                origin = 60.0  # CAN 4 protocol angle, 10 output degrees.
                emulator.reset([0, 0, 0, origin, 0, 0])
                with backend(executable, calibration, True) as p:
                    for step in range(1, 7):
                        command(p, f'motor 4 {sign*5} 1')
                        emulator.inject(4, origin+sign*step*5*6)
                        settle(p)
                    sent = len(emulator.frames_for(0xa4))
                    assert sent == 6
                    _, line = command(p, f'motor 4 {sign*.01} 1', ok=False)
                    assert 'session displacement exceeds 30 deg' in line
                    assert len(emulator.frames_for(0xa4)) == sent
                    # Returning inside the envelope is allowed, without resetting its origin.
                    command(p, f'motor 4 {-sign*5} 1')
                    emulator.inject(4, origin+sign*25*6)
                    settle(p)

            print('CASE: small requested step retains a small displacement guard')
            emulator.reset()
            with backend(executable, calibration, True) as p:
                command(p, 'motor 1 .1 .2')
                emulator.inject(1, .2*36)
                values, line = command(p, 'update')
                assert values['active'] == '0' and 'unexpected output-shaft displacement' in line
                emulator.assert_stopped([1])
            print('CASE: cable directions reach A4 targets and feedback for motors 3 and 4')
            for axis in (3, 4):
                emulator.reset()
                with backend(executable, calibration) as p:
                    command(p, 'write')
                    command(p, f'write_target {axis} 1')
                    time.sleep(.1)
                    sent = dict(emulator.frames_for(0xa4))
                    motor = j4_motor if axis == 4 else axis
                    protocol = struct.unpack('<i', sent[motor][4:])[0]/100
                    base = 51.75/16.25 if axis == 3 else wrist_gains[j4_motor]*math.sin(math.radians(1))*180/math.pi
                    expected = -signs[motor-1]*base*(36 if axis == 3 else 6)
                    assert abs(protocol-expected) <= .011, (axis, protocol, expected)
                    emulator.inject(motor, protocol)
                    values, _ = command(p, 'read')
                    assert abs(float(values['q3_deg' if axis == 3 else 'q5_deg'])-1) < .002
                    command(p, 'disarm')
            print('CASE: accepted packing measurements survive direction rebind; zero target reverses')
            reference = yaml.safe_load((ROOT/'docs/calibration-history/pack_reference_calibration.R03.recorded.yaml').read_text())
            reference['calibration']['configuration_fingerprint'] = data_fingerprint = candidate['configuration_fingerprint'](cable_file, mapping)
            assert data_fingerprint
            pack_calibration = work/'packing.yaml'
            pack_calibration.write_text(yaml.safe_dump(reference))
            raw = reference['calibration']['raw_output_at_packing_rad']
            protocol_pack = [math.degrees(value)*entry['protocol_angle_ratio']*entry['motor_direction']
                             for value, entry in zip(raw, mapping)]
            emulator.reset(protocol_pack)
            with backend(executable, pack_calibration) as p:
                values, _ = command(p, 'read')
                assert values['kin'] == values['control'] == '1'
                assert abs(float(values['q2_deg'])-100.7) < .002
                assert abs(float(values['q3_deg'])+85.6) < .002
                command(p, 'write')  # q1=.01 rad, all other joints zero, virtual CAN only
                time.sleep(.1)
                sent = dict(emulator.frames_for(0xa4))
                old_pack_output = [0, 100.7, (51.75*-85.6-28.5*100.7)/16.25,
                                   28.5/16.25*100.7, 28.5/16.25*100.7,
                                   .5*(20.25/12.25+28.5/17.25)*100.7]
                for axis in (3, 4):
                    actual = struct.unpack('<i', sent[axis][4:])[0]/100
                    entry = mapping[axis-1]
                    expected = protocol_pack[axis-1]-signs[axis-1]*old_pack_output[axis-1]*entry['motor_direction']*entry['protocol_angle_ratio']
                    assert abs(actual-expected) <= .011, (axis, actual, expected)
                command(p, 'disarm')
            print('CASE: successful arrival retains A4 loop, no stale writes, explicit stop still works')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                time.sleep(.1)
                command(p, 'hold_wrong', ok=False)
                command(p, 'hold')
                assert not emulator.frames_for(0x81), 'normal completion must not stop motors'
                sent = len(emulator.frames_for(0xa4))
                command(p, 'write_target 1 2')
                assert len(emulator.frames_for(0xa4)) == sent, 'released lease allowed stale controller write'
                command(p, 'disarm')
                emulator.assert_stopped(range(1,7))

            print('CASE: teaching releases all motors; exit captures moved position, never old target')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'write')
                command(p, 'teach_on')
                assert {i for i,_ in emulator.frames_for(0x81)} == set(range(1,7))
                assert not emulator.frames_for(0x80), 'teaching must preserve multi-turn reference'
                command(p, 'arm', ok=False)
                command(p, 'enable', ok=False)
                emulator.inject(1, 36.0)  # hand drag output 1 by one degree
                time.sleep(.2)
                before = len(emulator.frames_for(0xa4))
                command(p, 'teach_off')
                held = emulator.frames_for(0xa4)[before:]
                assert len(held) == 6
                assert struct.unpack('<i', dict(held)[1][4:])[0] == 3600
                assert not emulator.frames_for(0x88), 'must not RUN the previous target'
                command(p, 'write_target 1 10')
                assert len(emulator.frames_for(0xa4)) == before+6
                values,_ = command(p, 'read')
                assert values['control'] == '1' and values['teaching'] == '0'
                command(p, 'disarm')

            print('CASE: stale feedback cannot exit teaching')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'teach_on')
                with emulator.lock: emulator.drop_feedback.add(2)
                time.sleep(1.1)
                command(p, 'teach_off', ok=False)
                assert not emulator.frames_for(0xa4)
            print('CASE: already-reached goals explicitly establish position hold')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'idle')
                command(p, 'arm')
                command(p, 'hold')
                assert len(emulator.frames_for(0xa4)) == 6
                assert not emulator.frames_for(0x81)

            print('CASE: missing position ACK cannot falsely report teaching exit success')
            emulator.reset()
            with backend(executable, calibration) as p:
                command(p, 'teach_on')
                with emulator.lock: emulator.drop_position_ack.add(4)
                command(p, 'teach_off', ok=False)
                values,_ = command(p, 'read')
                assert values['teaching'] == '1' and values['control'] == '0'
                assert {i for i,_ in emulator.frames_for(0x81)} == set(range(1,7))
                assert not emulator.frames_for(0x80), 'failed hold must preserve multi-turn reference'
            print('CASE: swapped wrist joint tests route to correct CAN motor and speed ceiling')
            for joint,motor in ((4,j4_motor),(5,4 if swapped else 5)):
                emulator.reset(dynamic=True)
                with backend(executable, calibration, True) as p:
                    command(p, f'joint {joint} .1 .2')
                    settle(p)
                    frames=emulator.frames_for(0xa4)
                    assert {i for i,_ in frames} == {motor}
                    cap=math.floor(.2*6*(wrist_gains[motor])*math.cos(math.radians(.1))+1e-9)
                    assert all(1 <= struct.unpack('<H',data[2:4])[0] <= cap for _,data in frames)
            print('PASS: production-backend virtual CAN safety and stop/hold scenarios')
        finally:
            emulator.close()


if __name__ == '__main__':
    main()
