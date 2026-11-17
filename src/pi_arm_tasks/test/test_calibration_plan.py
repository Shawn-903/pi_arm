import json
import math

import pytest

from pi_arm_tasks.calibration_motion.plan import CalibrationPose, load_poses, validate_poses


def test_loads_named_poses_in_degrees_with_per_pose_dwell(tmp_path):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps([
        {"name": "P1", "joint_positions_deg": [90, 0, 0, 0, 0, 0], "dwell_sec": 0.2},
        {"name": "P2", "joint_positions_deg": [0, 0, 0, 0, 0, 0]},
    ]))
    poses = load_poses(path)
    assert [pose.name for pose in poses] == ["P1", "P2"]
    assert poses[0].joint_positions_rad[0] == pytest.approx(math.pi / 2)
    assert [pose.dwell_sec for pose in poses] == [0.2, 3.0]


@pytest.mark.parametrize("change", [
    {"joint_positions_deg": [0] * 5}, {"joint_positions_deg": [True] + [0] * 5},
    {"joint_positions_deg": [float("nan")] + [0] * 5}, {"dwell_sec": -1},
    {"dwell_sec": float("inf")}, {"dwell_s": 10},
])
def test_invalid_plan_is_rejected_before_motion(tmp_path, change):
    row = {"name": "P1", "joint_positions_deg": [0] * 6}
    row.update(change)
    path = tmp_path / "poses.json"
    path.write_text(json.dumps([row]))
    with pytest.raises(ValueError):
        load_poses(path)


def test_programmatic_poses_freeze_inputs_and_require_distinct_names():
    angles = [0.0] * 6
    pose = CalibrationPose("P1", angles, 0.0)
    angles[0] = 1.0
    assert pose.joint_positions_rad[0] == 0.0
    with pytest.raises(ValueError, match="unique"):
        validate_poses([pose, pose])
    with pytest.raises(ValueError):
        validate_poses([])
