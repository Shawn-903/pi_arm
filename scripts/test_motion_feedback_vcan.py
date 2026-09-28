#!/usr/bin/env python3
"""Motion-state regression through production RealBackend on vcan_motion only.

Source the install to test. Requires a dedicated UP virtual CAN interface named
vcan_motion. No ROS graph, physical CAN, movement, enable or zeroing commands.
"""
import json
import math
import os
from pathlib import Path
import runpy
import select
import socket
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
import yaml
from vcan_motor_emulator import CAN_BASE, CAN_FRAME, Motor, response

ROOT = Path(__file__).resolve().parents[1]
INTERFACE = 'vcan_motion'


class Feedback:
    def __init__(self):
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.bus.bind((INTERFACE,))
        self.bus.settimeout(.05)
        self.phase = ('noise', time.monotonic())
        self.running = True
        self.unexpected = []
        self.thread = threading.Thread(target=self.serve)
        self.thread.start()

    def serve(self):
        while self.running:
            try:
                frame = self.bus.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            can_id, length, request = CAN_FRAME.unpack(frame)
            motor_id = (can_id & 0x7ff) - CAN_BASE
            if length != 8 or motor_id not in range(1, 7):
                continue
            if request[0] not in (0x92, 0x9a, 0x9c):
                self.unexpected.append(request[0])
                continue
            phase, started = self.phase
            elapsed = time.monotonic() - started
            tick = int(elapsed / .02)
            motor = Motor(enabled=True)
            if phase == 'missing':
                continue
            if phase in ('noise', 'invalid'):
                motor.angle_raw = (tick + motor_id) % 3 - 1
                motor.speed_dps = 5 if tick % 23 == motor_id else 0
                if phase == 'invalid' and motor_id == 4:
                    motor.angle_raw += 100000  # Invalid wrist inverse; raw flags still valid.
            if phase == 'slow' and motor_id == 6:
                motor.angle_raw = round(.02 * elapsed * 6 * 100)
            if phase == 'oscillation' and motor_id == 6:
                motor.angle_raw = round(.02 * math.sin(2 * math.pi * elapsed / .5) * 6 * 100)
            if phase == 'wrist' and motor_id == 4:
                motor.angle_raw = round(elapsed * 6 * 100)
                motor.speed_dps = 6
            if phase == 'speed_only' and motor_id == 6:
                motor.speed_dps = 2
            reply = response(motor, request)
            self.bus.send(CAN_FRAME.pack(can_id, 8, reply))

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.bus.close()


def readline(process):
    assert select.select([process.stdout], [], [], 8)[0], 'feedback probe timed out'
    line = process.stdout.readline().strip()
    assert line, f'feedback probe exited: {process.poll()}'
    return line


def main():
    link = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', INTERFACE]))
    assert link[0].get('linkinfo', {}).get('info_kind') == 'vcan', 'requires virtual CAN'
    assert 'UP' in link[0]['flags'], 'virtual interface must be UP'
    with tempfile.TemporaryDirectory(prefix='pi-arm-motion-feedback-') as temp:
        work = Path(temp)
        cable = ROOT / 'src/pi_arm_description/config/cable_kinematics.yaml'
        candidate = runpy.run_path(str(ROOT / 'src/pi_arm_hardware/scripts/pi_arm_calibration_candidate'))
        mapping = [dict(joint_name=name, motor_id=i, protocol_angle_ratio=36 if i <= 3 else 6,
                        motor_direction=direction)
                   for i, (name, direction) in enumerate(zip(candidate['JOINT_NAMES'], [1, 1, -1, -1, -1, 1]), 1)]
        data = yaml.safe_load((ROOT / 'docs/calibration-history/pack_reference_calibration.R02.recorded.yaml').read_text())
        data['calibration'].update(robot_id='VCAN-ONLY', calibration_id='VCAN-MOTION-TEST',
                                  packing_pose_joint_rad=[0.] * 6, raw_output_at_packing_rad=[0.] * 6,
                                  configuration_fingerprint=candidate['configuration_fingerprint'](cable, mapping))
        calibration = work / 'zero.yaml'
        calibration.write_text(yaml.safe_dump(data))
        packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
        prefixes = [Path(get_package_prefix(package)) for package in packages]
        executable = work / 'probe'
        subprocess.run(['g++', '-std=c++17', str(ROOT / 'src/pi_arm_hardware/test/vcan_motion_probe.cpp'),
                        *['-I' + str(prefix / 'include') for prefix in prefixes],
                        *['-L' + str(prefix / 'lib') for prefix in prefixes],
                        *['-l' + package for package in packages], '-pthread', '-o', str(executable)], check=True)
        env = dict(os.environ, LD_LIBRARY_PATH=':'.join(str(prefix / 'lib') for prefix in prefixes)
                   + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
        feedback = Feedback()
        process = subprocess.Popen([str(executable), str(ROOT), str(calibration)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, env=env)
        try:
            assert readline(process) == 'READY'
            swapped = yaml.safe_load(cable.read_text())['cable_kinematics'].get('swap_wrist_channels', False)
            cases = [('noise', 0, 0), ('slow', 32, 32), ('oscillation', 32, 32), ('noise', 0, 0),
                     ('speed_only', 32, 32), ('wrist', 8, 16 if swapped else 8),
                     ('missing', 63, 63), ('noise', 0, 0), ('invalid', 0, 63)]
            for phase, motors, joints in cases:
                feedback.phase = (phase, time.monotonic())
                process.stdin.write('sample\n')
                process.stdin.flush()
                result = json.loads(readline(process))
                print(phase, result, flush=True)
                assert result['motor_mask'] == motors, result
                assert result['joint_mask'] == joints, result
                assert result['fresh_frames'] == (0 if phase == 'missing' else 60), result
                assert result['kinematics_frames'] == (0 if phase == 'invalid' else 60), result
                if phase == 'noise':
                    assert result['raw_speed_frames'] > 0, 'raw velocity diagnostics must be preserved'
            assert not feedback.unexpected, feedback.unexpected
            print('PASS: production feedback motion detection; polling frames only')
        finally:
            if process.poll() is None:
                process.stdin.write('quit\n')
                process.stdin.flush()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            feedback.close()


if __name__ == '__main__':
    main()
