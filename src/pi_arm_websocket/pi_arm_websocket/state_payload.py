"""将同一帧 ROS 状态转换为网页协议中的硬件和任务字段。"""
from __future__ import annotations

import math
from typing import Any, TYPE_CHECKING

from .rotations import RAD_TO_DEG

if TYPE_CHECKING:
    from pi_arm_interfaces.msg import RobotState


def robot_state_payload(state: RobotState | None) -> dict[str, Any]:
    """只使用传入快照，返回独立字典，避免混入另一时刻的反馈或修改原消息。"""
    if state is None:
        body: dict[str, Any] = {
            "current_joint_angles": [],
            "current_speeds": [],
            "enabled": [],
            "joint_enabled": [],
            "has_error": [],
            "in_motion": [],
            "last_update_timestamps": [],
            "motor_ids": [],
            "motor_enabled": [],
            "motor_has_error": [],
            "motor_in_motion": [],
            "motor_fresh": [],
            "motor_error_codes": [],
            "motor_ages_sec": [],
            "motor_last_update_timestamps": [],
            "calibrated": False,
            "kinematics_valid": False,
            "control_permitted": False,
            "calibration_id": "",
            "software_zero_id": "",
            "raw_protocol_positions_deg": [],
            "raw_output_positions_rad": [],
            "software_output_offsets_rad": [],
            "model_output_positions_rad": [],
            "kinematics_error": {},
            "all_enabled": False,
            "any_error": False,
            "any_in_motion": False,
            "robot_state": "DISCONNECTED",
        }
    else:
        hardware = state.hardware
        stamp = float(hardware.stamp.sec) + float(hardware.stamp.nanosec) / 1e9
        body = {
            "current_joint_angles": [v * RAD_TO_DEG for v in hardware.positions],
            "current_speeds": [v * RAD_TO_DEG for v in hardware.velocities],
            # 兼容字段 enabled 仍表示关节使能状态；原始电机状态另放 motor_enabled。
            "enabled": list(hardware.enabled),
            "joint_enabled": list(hardware.enabled),
            "has_error": list(hardware.errors),
            "in_motion": list(hardware.moving),
            "last_update_timestamps": [
                max(0.0, stamp - age) if fresh and math.isfinite(age) else 0.0
                for fresh, age in zip(hardware.fresh, hardware.ages_sec)
            ],
            "motor_ids": list(hardware.motor_ids),
            "motor_enabled": list(hardware.motor_enabled),
            "motor_has_error": list(hardware.motor_errors),
            "motor_in_motion": list(hardware.motor_moving),
            "motor_fresh": list(hardware.motor_fresh),
            "motor_error_codes": list(hardware.motor_error_codes),
            "motor_ages_sec": list(hardware.motor_ages_sec),
            "motor_last_update_timestamps": [
                max(0.0, stamp - age) if fresh and math.isfinite(age) else 0.0
                for fresh, age in zip(hardware.motor_fresh, hardware.motor_ages_sec)
            ],
            "calibrated": hardware.calibrated,
            "kinematics_valid": hardware.kinematics_valid,
            "control_permitted": hardware.control_permitted,
            "calibration_id": hardware.calibration_id,
            "software_zero_id": hardware.software_zero_id,
            "raw_protocol_positions_deg": list(hardware.raw_protocol_positions_deg),
            "raw_output_positions_rad": list(hardware.raw_output_positions_rad),
            "software_output_offsets_rad": list(hardware.software_output_offsets_rad),
            "model_output_positions_rad": list(hardware.model_output_positions_rad),
            "kinematics_error": {
                "code": hardware.kinematics_error_code,
                "axis": hardware.kinematics_error_axis,
                "value": hardware.kinematics_error_value,
                "message": hardware.kinematics_error_message,
            },
            "all_enabled": bool(hardware.motor_enabled)
            and all(hardware.motor_enabled),
            "any_error": any(hardware.errors)
            or any(hardware.error_codes)
            or any(hardware.motor_errors)
            or any(hardware.motor_error_codes),
            "any_in_motion": any(hardware.moving) or any(hardware.motor_moving),
            "robot_state": state.state_name,
            "diagnostic": hardware.diagnostic,
            "task": {
                "task_id": state.task.task_id,
                "command": state.task.command,
                "status": state.task.status,
                "progress": state.task.progress,
                "code": state.task.code,
                "message": state.task.message,
                "arrival_state": getattr(state.task, "arrival_state", ""),
                "goal_tolerance_deg": getattr(state.task, "goal_position_tolerance", 0.0) * RAD_TO_DEG,
                "joint_names": list(getattr(state.task, "joint_names", [])),
                "target_joint_angles_deg": [v * RAD_TO_DEG for v in getattr(state.task, "target_positions", [])],
                "error_joint_angles_deg": [v * RAD_TO_DEG for v in getattr(state.task, "position_errors", [])],
            },
        }
    hardware = state.hardware if state else None
    body["can_interface"] = hardware.can_interface if hardware else ""
    body["motor_test_permitted"] = bool(hardware and hardware.motor_test_permitted)
    body["joint_test_permitted"] = bool(hardware and hardware.joint_test_permitted)
    body["axis_test"] = {
        "active": bool(hardware and hardware.axis_test_active),
        "test_id": hardware.axis_test_id if hardware else 0,
        "status": hardware.axis_test_status if hardware else "state unavailable",
    }
    body["protocol_angle_ratios"] = list(hardware.protocol_angle_ratios) if hardware else []
    body["motor_directions"] = list(hardware.motor_directions) if hardware else []
    body["teaching_active"] = bool(hardware and hardware.teaching_active)
    body["motion_stop_pending"] = bool(hardware and hardware.motion_stop_pending)
    body["motion_stop_failed"] = bool(hardware and hardware.motion_stop_failed)
    body["state_available"] = state is not None
    body["joint_names"] = list(state.hardware.joint_names) if state else []
    body["hardware_connected"] = state.hardware.connected if state else False
    body["driver_mode"] = state.hardware.driver_mode if state else "UNKNOWN"
    body["joint_fresh"] = list(state.hardware.fresh) if state else []
    body["joint_ages_sec"] = list(state.hardware.ages_sec) if state else []
    return body


def json_safe(value: Any) -> Any:
    """把无效反馈中的 NaN/Infinity 转为 JSON null，避免浏览器拒绝整条状态消息。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value
