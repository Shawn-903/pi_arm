#!/usr/bin/env python3
"""Rebind accepted packing references after cable signs or an explicit wrist swap.

Offline: no ROS node, CAN connection, encoder reset or motion. Explicit invocation
accepts the selected mapping for testing; it does not validate physical cable accuracy.
"""
import argparse
import copy
from datetime import datetime, timezone
from pathlib import Path
import runpy
import subprocess
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]


def validate_direction_change(old, new):
    old, new = copy.deepcopy(old), copy.deepcopy(new)
    for document in (old, new):
        signs = document['cable_kinematics'].pop('output_directions', None)
        if signs is not None:
            if set(signs) != {f'motor_{i:02d}' for i in range(1, 7)}:
                raise ValueError('output_directions must contain all six motor IDs')
            if any(type(sign) is not int or sign not in (-1, 1) for sign in signs.values()):
                raise ValueError('output_directions values must be integer +1 or -1')
    if old != new:
        raise ValueError('only output_directions may change; geometry/limits require separate calibration review')


def validate_wrist_swap(old, new, calibration):
    old, new = copy.deepcopy(old), copy.deepcopy(new)
    a, b = old['cable_kinematics'], new['cable_kinematics']
    before, after = a.pop('swap_wrist_channels', False), b.pop('swap_wrist_channels', False)
    if type(before) is not bool or type(after) is not bool or before == after:
        raise ValueError('wrist rebind requires an explicit boolean toggle')
    # Only valid for this accepted reference: both wrist coordinates are zero,
    # so F(q_pack) is unchanged and no new physical angle measurement is invented.
    if calibration['packing_pose_joint_rad'][3:5] != [0.0, 0.0]:
        raise ValueError('wrist rebind requires both accepted packing wrist angles to be zero')
    limits = b['output_limits']
    limits['motor_04'], limits['motor_05'] = limits['motor_05'], limits['motor_04']
    if old != new:
        raise ValueError('wrist rebind only permits swapping assignment and its two output envelopes')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--reference-model', type=Path,
                        help='default: reference filename with .yaml replaced by .cable.yaml')
    parser.add_argument('--model', type=Path,
                        default=ROOT/'src/pi_arm_description/config/cable_kinematics.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibration-id', required=True)
    parser.add_argument('--swap-wrist', action='store_true', help='validated wrist channel swap, retaining zero-wrist packing reference')
    args = parser.parse_args()
    old_model = args.reference_model or args.reference.with_suffix('.cable.yaml')
    snapshot = args.output.with_suffix('.cable.yaml')
    if args.output.exists() or snapshot.exists():
        raise ValueError('output/snapshot already exists; use a new revision')
    source = yaml.safe_load(args.reference.read_text())
    calibration = source['calibration']
    if calibration['approved'] is not True:
        raise ValueError('reference must already be accepted')
    if calibration['calibration_id'] == args.calibration_id:
        raise ValueError('use a new calibration ID')
    old_document, new_document = yaml.safe_load(old_model.read_text()), yaml.safe_load(args.model.read_text())
    if args.swap_wrist:
        validate_wrist_swap(old_document, new_document, calibration)
    else:
        validate_direction_change(old_document, new_document)
    helper = runpy.run_path(str(ROOT/'src/pi_arm_hardware/scripts/pi_arm_calibration_candidate'))
    with tempfile.TemporaryDirectory() as directory:
        urdf = Path(directory)/'robot.urdf'
        with urdf.open('w') as stream:
            subprocess.run(['xacro', str(ROOT/'src/pi_arm_description/urdf/pi_arm.urdf.xacro'),
                            'backend:=real', f'cable_kinematics_file:={args.model.resolve()}'],
                           stdout=stream, check=True)
        _, mapping = helper['parse_robot_description'](urdf)
    if calibration['joint_names'] != helper['JOINT_NAMES'] or calibration['motor_ids'] != [
            entry['motor_id'] for entry in mapping]:
        raise ValueError('reference actuator mapping mismatch')
    fingerprint = helper['configuration_fingerprint']
    if fingerprint(old_model, mapping) != calibration['configuration_fingerprint']:
        raise ValueError('reference model or protocol installation mapping has changed')
    calibration['calibration_id'] = args.calibration_id
    calibration['created_at_utc'] = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    calibration['configuration_fingerprint'] = fingerprint(args.model, mapping)
    # Keep the accepted packing pose AND measured ROM-relative outputs unchanged.
    # The C++ loader recomputes b = F_new(q_pack) - raw_pack at startup.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with snapshot.open('xb') as stream:
        stream.write(args.model.read_bytes())
    with args.output.open('x') as stream:
        kind = 'Wrist-channel swap' if args.swap_wrist else 'Direction-only'
        stream.write(f'# {kind} rebind of {args.reference.name}; original measurements retained.\n'
                     '# Explicitly requested mapping for onsite testing; not a new physical measurement.\n')
        yaml.safe_dump(source, stream, sort_keys=False)
    print(f'Wrote {args.output}\nModel snapshot: {snapshot}')
    print('Restart using the new calibration_file. No CAN operation was performed.')


if __name__ == '__main__':
    main()
