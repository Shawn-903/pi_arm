"""读取标定姿态清单，并将文件中的度转换为任务使用的弧度。"""

from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path


JOINT_NAMES = ("joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09")


def finite_number(value, name, *, minimum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if (minimum is not None and value < minimum) or (positive and value <= 0):
        raise ValueError(f"{name} is outside its allowed range")
    return float(value)


@dataclass(frozen=True)
class CalibrationPose:
    name: str
    joint_positions_rad: tuple[float, ...]
    dwell_sec: float = 3.0

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("pose name must be a non-empty string")
        values = tuple(self.joint_positions_rad)
        if len(values) != len(JOINT_NAMES):
            raise ValueError("joint_positions_rad must contain six angles")
        values = tuple(finite_number(value, "joint angle") for value in values)
        object.__setattr__(self, "joint_positions_rad", values)
        object.__setattr__(self, "dwell_sec", finite_number(self.dwell_sec, "dwell_sec", minimum=0))


def validate_poses(poses):
    result = tuple(poses)
    if not result or any(not isinstance(pose, CalibrationPose) for pose in result):
        raise ValueError("provide a non-empty sequence of CalibrationPose objects")
    if len({pose.name for pose in result}) != len(result):
        raise ValueError("pose names must be unique within a plan")
    return result


def load_poses(path):
    rows = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(rows, list):
        raise ValueError("the pose file must contain a JSON list")
    poses = []
    for row in rows:
        if (not isinstance(row, dict) or not {"name", "joint_positions_deg"} <= row.keys()
                or row.keys() - {"name", "joint_positions_deg", "dwell_sec"}):
            raise ValueError("each pose needs name, joint_positions_deg and optional dwell_sec")
        angles = row["joint_positions_deg"]
        if not isinstance(angles, list):
            raise ValueError("joint_positions_deg must be a list")
        poses.append(CalibrationPose(
            row["name"], tuple(math.radians(finite_number(value, "joint angle")) for value in angles),
            row.get("dwell_sec", 3.0),
        ))
    return validate_poses(poses)
