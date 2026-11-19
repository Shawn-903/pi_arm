#!/usr/bin/env python3
"""在 vcan0 上验证一键标零脚本的保存、超时、中断和独占冲突清理。"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import runpy
import signal
import subprocess
import tempfile
import time

import yaml

from test_motion_safety_vcan import Emulator


ROOT = Path(__file__).resolve().parents[1]
LEASE = Path("/run/lock/pi_arm_can_vcan0.lock")


def lease_available():
    with LEASE.open("a+") as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except BlockingIOError:
            return False


def run_case(work, name, command, *, interrupt=False):
    children = set()
    sent = False
    deadline = time.monotonic() + 25
    with (work / f"{name}.log").open("w") as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=log, text=True)
        try:
            while True:
                child_file = Path(f"/proc/{process.pid}/task/{process.pid}/children")
                current_children = child_file.read_text().split() if child_file.exists() else []
                children.update(current_children)
                if interrupt and not sent and len(current_children) == 2 and not lease_available():
                    process.send_signal(signal.SIGINT)
                    sent = True
                try:
                    stdout, _ = process.communicate(timeout=0.05)
                    break
                except subprocess.TimeoutExpired:
                    assert time.monotonic() < deadline, f"{name}: script did not exit"
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.communicate(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
            # 失败时也清理测试捕获的子进程，便于单独重跑该场景。
            remaining = [pid for pid in children if Path(f"/proc/{pid}").exists()]
            for pid in remaining:
                os.kill(int(pid), signal.SIGTERM)
        assert not remaining, f"{name}: child services survived exit: {remaining}"
        assert not interrupt or sent, "test did not reach service startup before interruption"
    (work / f"{name}.json").write_text(stdout)
    return process.returncode, json.loads(stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--case", choices=("all", "success", "timeout", "interrupt", "busy"), default="all")
    args = parser.parse_args()
    info = json.loads(subprocess.check_output(["ip", "-d", "-j", "link", "show", "vcan0"]))[0]
    assert info["linkinfo"]["info_kind"] == "vcan", "virtual CAN required"
    assert lease_available(), "vcan0 is already in use"
    os.environ.update(ROS_DOMAIN_ID="112", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    work = args.log_dir or Path(tempfile.mkdtemp(prefix="pi-arm-zero-once-test-"))
    work.mkdir(parents=True, exist_ok=True)
    os.environ["HOME"] = str(work.resolve() / "home")
    Path.home().mkdir(exist_ok=True)
    cable = ROOT / "src/pi_arm_description/config/cable_kinematics.yaml"
    candidate = runpy.run_path(str(ROOT / "src/pi_arm_hardware/scripts/pi_arm_calibration_candidate"))
    mapping = [dict(joint_name=name, motor_id=i, protocol_angle_ratio=36.0 if i <= 3 else 6.0, motor_direction=d)
               for i, (name, d) in enumerate(zip(candidate["JOINT_NAMES"], [1, -1, -1, -1, -1, 1]), 1)]
    data = yaml.safe_load((ROOT / "src/pi_arm_hardware/config/pack_reference_calibration.template.yaml").read_text())
    data["calibration"].update(approved=True, robot_id="VCAN-ONLY", calibration_id="ZERO-ONCE-VCAN-ONLY",
                              packing_pose_joint_rad=[0.0] * 6, raw_output_at_packing_rad=[0.0] * 6,
                              configuration_fingerprint=candidate["configuration_fingerprint"](cable, mapping))
    base = work / "base.yaml"
    base.write_text(yaml.safe_dump(data))
    base_before = base.read_bytes()
    zero = Path.home() / ".local/share/pi_arm/software-zero-vcan0.yaml"
    capture = ["bash", str(ROOT / "scripts/zero_reference.sh"), "--confirm-reference",
               "--can-interface", "vcan0", "--cable-kinematics-file", str(cable)]
    command = capture + ["--calibration-file", str(base)]
    emulator = Emulator()
    angles = [180.0, -360.0, 540.0, -120.0, 60.0, -30.0]
    emulator.reset(angles)
    try:
        if args.case in ("all", "success"):
            missing = subprocess.run(["bash", str(ROOT / "scripts/zero_reference.sh")],
                                     capture_output=True, text=True, timeout=10)
            assert missing.returncode != 0 and "--confirm-reference" in missing.stderr
            assert not emulator.frames_for(0x92), "missing confirmation started hardware polling"
            for name, pose in (("packing", [0, 100.7, -85.6, 0, 0, 0]), ("zero", [0] * 6),
                               ("known-pose", [1, 0, 0, 0, 0, 0])):
                source = command if name == "packing" else capture + ["--reference-deg", *map(str, pose)]
                code, result = run_case(work, name, source)
                assert code == 0 and result["success"] and result["cleanup_success"], result
                assert result["raw_protocol_positions_deg"] == angles
                document = yaml.safe_load(zero.read_text())
                if name == "packing":
                    assert document["calibration"] == data["calibration"]
                else:
                    assert set(document) == {"software_zero"}, "recapture retained previous packing metadata"
                saved = document["software_zero"]
                assert saved["schema_version"] == 4 and result["software_zero_file"] == str(zero)
                assert saved["reference_joint_positions_rad"] == [math.radians(value) for value in pose]
                assert lease_available(), "CAN lease survived successful exit"
            print("PASS one-command zero: default packing pose, explicit zero and custom pose", flush=True)
            zero.rename(work / "reference-before-loss.yaml")
            for name in ("lost-file", "recapture"):
                angles[0] += 72
                emulator.inject(1, angles[0])
                code, result = run_case(work, name, capture + ["--reference-deg", "1", "0", "0", "0", "0", "0"])
                assert code == 0 and result["cleanup_success"] and result["success"], result
                document = yaml.safe_load(zero.read_text())
                assert set(document) == {"software_zero"}
                assert document["software_zero"]["raw_protocol_positions_deg"] == angles
            print("PASS lost file and recapture: known pose and current encoders alone", flush=True)

            for name, invocation in (("changed-model", capture), ("changed-model-with-base", command)):
                document = yaml.safe_load(zero.read_text())
                document["software_zero"]["schema_version"] = 3
                document["software_zero"]["reference_id"] = "legacy-sample"
                document["software_zero"]["configuration_fingerprint"] = "fnv1a64:0000000000000000"
                zero.write_text(yaml.safe_dump(document))
                angles[1] += 36
                emulator.inject(2, angles[1])
                code, result = run_case(work, name, invocation)
                assert code == 0 and result["success"] and result["cleanup_success"], result
                document = yaml.safe_load(zero.read_text())
                saved = document["software_zero"]
                assert "configuration_fingerprint" not in saved and "reference_id" not in saved
                assert saved["raw_protocol_positions_deg"] == angles
                assert ("calibration" in document) == (invocation == command)
            print("PASS replaced incompatible software zero from fresh samples", flush=True)

        before = zero.read_bytes() if zero.exists() else None
        with emulator.lock:
            emulator.drop_feedback.add(3)
        if args.case in ("all", "timeout"):
            code, result = run_case(work, "timeout", command + ["--startup-timeout", "3"])
            assert code != 0 and not result["success"] and result["cleanup_success"], result
            assert "stationary motor feedback" in result["message"], result
            assert lease_available(), "CAN lease survived feedback timeout"
            print("PASS feedback timeout: exits, preserves reference and releases CAN", flush=True)
        if args.case in ("all", "interrupt"):
            code, result = run_case(work, "interrupt", command, interrupt=True)
            assert code != 0 and not result["success"] and result["cleanup_success"], result
            assert "interrupted before submission" in result["message"], result
            assert lease_available(), "CAN lease survived interruption"
            print("PASS SIGINT: exits and releases child services and CAN", flush=True)
        if args.case in ("all", "busy"):
            with LEASE.open("r+") as lease:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
                code, result = run_case(work, "busy", command)
                assert code != 0 and not result["success"] and "exited" in result["message"], result
                assert not lease_available(), "existing owner lease was changed"
            assert lease_available()
            print("PASS busy CAN: startup fails, own services exit and existing owner is preserved", flush=True)
        assert (zero.read_bytes() if zero.exists() else None) == before, "failed capture changed reference"
        assert base.read_bytes() == base_before, "base calibration changed"
        counts = {f"0x{code:02X}": len(emulator.frames_for(code)) for code in (0x19, 0x95, 0xA4, 0x88, 0x80, 0x81)}
        (work / "frame-counts.json").write_text(json.dumps(counts))
        assert not any(counts.values()), counts
        print("PASS frame capture: no ROM-zero, motion, enable, power-off or stop commands", flush=True)
    finally:
        emulator.close()
    print(f"Logs: {work}", flush=True)


if __name__ == "__main__":
    main()
