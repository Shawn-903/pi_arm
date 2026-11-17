#!/usr/bin/env python3
"""Backend/plugin lifecycle regression. Requires an unused UP vcan0 and domain 100.

Source the candidate install, build pi_arm_hardware with BUILD_TESTING=ON, and
pass --probe build/pi_arm_hardware/vcan_lifecycle_probe. Uses synthetic feedback
and temporary reference data; physical CAN cannot be selected.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import runpy
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import xacro
import yaml

from test_motion_safety_vcan import Emulator, backend, command, readline

ROOT = Path(__file__).resolve().parents[1]


def make_calibration(work):
    candidate = runpy.run_path(str(ROOT / 'src/pi_arm_hardware/scripts/pi_arm_calibration_candidate'))
    cable = ROOT / 'src/pi_arm_description/config/cable_kinematics.yaml'
    mapping = [dict(joint_name=name, motor_id=i, protocol_angle_ratio=36 if i <= 3 else 6, motor_direction=d)
               for i, (name, d) in enumerate(zip(candidate['JOINT_NAMES'], [1, 1, -1, -1, -1, 1]), 1)]
    data = yaml.safe_load((ROOT / 'src/pi_arm_hardware/config/pack_reference_calibration.template.yaml').read_text())
    data['calibration'].update(approved=True, robot_id='VCAN-ONLY', calibration_id='LIFECYCLE-VCAN-ONLY',
                              packing_pose_joint_rad=[0.] * 6, raw_output_at_packing_rad=[0.] * 6,
                              configuration_fingerprint=candidate['configuration_fingerprint'](cable, mapping))
    path = work / 'zero.yaml'
    path.write_text(yaml.safe_dump(data))
    return path, cable


def check_backend(work, calibration, emulator):
    packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
    executable = work / 'backend-probe'
    subprocess.run(['g++', '-std=c++17', str(ROOT / 'src/pi_arm_hardware/test/vcan_backend_probe.cpp'),
                    *['-I'+str(ROOT / 'src' / p / 'include') for p in packages],
                    *['-L'+str(Path(get_package_prefix(p)) / 'lib') for p in packages],
                    *['-l'+p for p in packages], '-pthread', '-o', str(executable)], check=True)
    with backend(executable, calibration) as probe:
        command(probe, 'idle')
        command(probe, 'arm')
        command(probe, 'write_target 1 1')
        values, _ = command(probe, 'deactivate')
        assert values['control'] == '0' and values['fresh'] == '6', values
        command(probe, 'arm', ok=False)
        command(probe, 'stop_hold', ok=False)
        command(probe, 'teach_on', ok=False)
        command(probe, 'write_target 1 2')
        emulator.assert_stopped(range(1, 7))
        count = len(emulator.frames_for(0xa4))
        command(probe, 'activate')
        command(probe, 'write_target 1 2')
        time.sleep(.1)
        assert len(emulator.frames_for(0xa4)) == count, 'activation replayed an old command/lease'
        command(probe, 'arm')
        command(probe, 'write_target 1 3')
        time.sleep(.1)
        assert len(emulator.frames_for(0xa4)) == count + 6, 'fresh active lease did not execute'
    print('PASS inactive feedback cannot rearm/hold; reactivation needs a new lease', flush=True)


def interface_lease_available():
    with open('/run/lock/pi_arm_can_vcan0.lock', 'r+') as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(lease, fcntl.LOCK_UN)
        return True


def check_plugin(probe, work, calibration, cable, emulator, log_dir):
    urdf = work / 'robot.urdf'
    urdf.write_text(xacro.process_file(str(ROOT / 'src/pi_arm_description/urdf/pi_arm.urdf.xacro'),
                                     mappings={'backend': 'real', 'can_interface': 'vcan0',
                                               'allow_real_control': 'true', 'calibration_file': str(calibration),
                                               'cable_kinematics_file': str(cable)}).toxml())
    for transition in ('deactivate', 'shutdown', 'error', 'cleanup', 'shutdown_no_ack'):
        emulator.reset()
        log_path = log_dir / f'{transition}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([str(probe), str(urdf)], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=log, text=True)
        try:
            assert readline(process) == 'READY'
            assert not interface_lease_available(), 'active plugin lost its exclusive CAN lease'
            assert len(emulator.frames_for(0xa4)) == 6
            if transition == 'shutdown_no_ack':
                emulator.drop_stop_ack = {6}
            started = time.monotonic()
            values, _ = command(process, transition.replace('_no_ack', ''))
            if transition == 'deactivate':
                assert not interface_lease_available(), 'inactive diagnostics lost the CAN connection'
                command(process, 'arm', ok=False)
                command(process, 'hold', ok=False)
                values, _ = command(process, 'shutdown')
            # Keep the plugin object/process alive: destructor cleanup cannot mask a leak.
            assert process.poll() is None
            assert interface_lease_available(), f'{transition}: SocketCAN owner lease survived transition'
            assert values['can_sockets'] == '0', values
            values, _ = command(process, 'cleanup')
            assert values['can_sockets'] == '0', values
            emulator.assert_stopped(range(1, 7))
            assert time.monotonic() - started < 4, 'cleanup did not finish within the expected stop timeout'
            if transition == 'shutdown_no_ack':
                assert 'STOP UNCONFIRMED' in log_path.read_text(), log_path.read_text()
            print(f'PASS plugin {transition}: stopped, CAN socket/lease released, repeated cleanup safe', flush=True)
        finally:
            if process.poll() is None:
                process.stdin.write('quit\n')
                process.stdin.flush()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise
            assert process.returncode == 0, log_path.read_text()
            process.stdin.close()
            process.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', required=True, type=Path)
    parser.add_argument('--log-dir', required=True, type=Path)
    parser.add_argument('--case', choices=('all', 'backend', 'plugin'), default='all')
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '100', 'Use isolated ROS domain 100'
    link = json.loads(subprocess.check_output(['ip', '-d', '-j', 'link', 'show', 'vcan0']))[0]
    assert link['linkinfo']['info_kind'] == 'vcan', 'requires virtual CAN'
    args.log_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='backend-lifecycle-') as folder:
        work = Path(folder)
        calibration, cable = make_calibration(work)
        emulator = Emulator()
        try:
            if args.case in ('all', 'backend'):
                check_backend(work, calibration, emulator)
            if args.case in ('all', 'plugin'):
                check_plugin(args.probe.resolve(), work, calibration, cable, emulator, args.log_dir)
        finally:
            emulator.close()


if __name__ == '__main__':
    main()
