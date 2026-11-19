#!/usr/bin/env python3
"""Check CAN reconnect and stale-command rejection. Needs root and absent vcan0.

Creates/deletes only its own vcan0; never opens a physical CAN interface. Source
the intended ROS install before running. All calibration/probe files are temporary.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_prefix
from test_motion_safety_vcan import Emulator, backend, command
from vcan_test_model import calibration_mapping

ROOT = Path(__file__).resolve().parents[1]


def link_info():
    result = subprocess.run(['ip', '-d', '-j', 'link', 'show', 'vcan0'], capture_output=True, text=True)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None


def wait_state(probe, predicate):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        values, line = command(probe, 'read')
        if predicate(values):
            return values
        time.sleep(.1)
    raise AssertionError(line)


def main():
    assert os.geteuid() == 0, 'root is required to create/remove the test virtual interface'
    assert link_info() is None, 'refuse to modify an existing vcan0'
    owned_index = None
    emulator = None
    with tempfile.TemporaryDirectory(prefix='pi_arm_can_recovery_') as folder:
        work = Path(folder)
        work.chmod(0o755)
        cable = ROOT / 'src/pi_arm_description/config/cable_kinematics.yaml'
        _, fingerprint = calibration_mapping(cable)
        data = yaml.safe_load((ROOT / 'src/pi_arm_hardware/config/pack_reference_calibration.template.yaml').read_text())
        data['calibration'].update(approved=True, robot_id='VCAN-ONLY', calibration_id='RECONNECT-VCAN-ONLY',
                                  packing_pose_joint_rad=[0.0]*6, raw_output_at_packing_rad=[0.0]*6,
                                  configuration_fingerprint=fingerprint)
        calibration = work / 'base.yaml'
        calibration.write_text(yaml.safe_dump(data))
        packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
        executable = work / 'probe'
        subprocess.run(['g++', '-std=c++17', str(ROOT / 'src/pi_arm_hardware/test/vcan_backend_probe.cpp'),
                        *['-I'+str(ROOT / 'src' / p / 'include') for p in packages],
                        *['-L'+str(Path(get_package_prefix(p)) / 'lib') for p in packages],
                        *['-l'+p for p in packages], '-pthread', '-o', str(executable)], check=True)
        try:
            # Only virtual-link operations need root. Keep the backend's normal
            # user/lease ownership when invoked through sudo.
            with backend(executable, calibration, user=int(os.environ.get('SUDO_UID', os.getuid()))) as probe:
                values, _ = command(probe, 'read')
                assert values['driver'] == 'FAULT' and values['fresh'] == '0'
                command(probe, 'arm', ok=False)
                subprocess.run(['ip', 'link', 'add', 'dev', 'vcan0', 'type', 'vcan'], check=True)
                owned_index = link_info()['ifindex']
                subprocess.run(['ip', 'link', 'set', 'dev', 'vcan0', 'up'], check=True)
                # An existing but silent interface must keep serving diagnostics.
                wait_state(probe, lambda s: s['driver'] == 'MONITORING' and s['fresh'] == '0')
                emulator = Emulator()
                wait_state(probe, lambda s: s['fresh'] == '6' and s['control'] == '1')
                assert not emulator.frames_for(0xa4)
                print('PASS unavailable interface and late feedback recover without restarting', flush=True)

                command(probe, 'idle')
                command(probe, 'arm')
                emulator.close()
                emulator = None
                subprocess.run(['ip', 'link', 'set', 'dev', 'vcan0', 'down'], check=True)
                wait_state(probe, lambda s: s['driver'] == 'FAULT' and s['fresh'] == '0')
                subprocess.run(['ip', 'link', 'set', 'dev', 'vcan0', 'up'], check=True)
                emulator = Emulator()
                wait_state(probe, lambda s: s['fresh'] == '6' and s['control'] == '1')
                command(probe, 'write_target 1 1')
                time.sleep(.1)
                assert not emulator.frames_for(0xa4), 'an old armed lease survived reconnect'
                print('PASS link recovery invalidates an armed but unsent trajectory', flush=True)

                command(probe, 'write')
                time.sleep(.1)
                assert emulator.frames_for(0xa4)
                emulator.close()
                emulator = None
                subprocess.run(['ip', 'link', 'delete', 'dev', 'vcan0'], check=True)
                owned_index = None
                wait_state(probe, lambda s: s['driver'] == 'FAULT')
                subprocess.run(['ip', 'link', 'add', 'dev', 'vcan0', 'type', 'vcan'], check=True)
                owned_index = link_info()['ifindex']
                subprocess.run(['ip', 'link', 'set', 'dev', 'vcan0', 'up'], check=True)
                emulator = Emulator()
                recovered = wait_state(probe, lambda s: s['fresh'] == '6')
                assert recovered['control'] == '0' and recovered['stop_failed'] == '1'
                command(probe, 'write_target 1 2')
                command(probe, 'arm', ok=False)
                assert not emulator.frames_for(0xa4), 'motion resumed after interface recreation'
                assert not emulator.frames_for(0x88)
                command(probe, 'disarm')  # Explicit operator stop confirms the previous motion is stopped.
                wait_state(probe, lambda s: s['stop_failed'] == '0' and s['control'] == '1')
                assert not emulator.frames_for(0xa4)
                print('PASS recreated interface polls; old motion stays stopped until explicit acknowledgement', flush=True)
        finally:
            if emulator is not None:
                emulator.close()
            info = link_info()
            if owned_index is not None and info is not None:
                assert info['ifindex'] == owned_index and info['linkinfo']['info_kind'] == 'vcan'
                subprocess.run(['ip', 'link', 'delete', 'dev', 'vcan0'], check=True)


if __name__ == '__main__':
    main()
