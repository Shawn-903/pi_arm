"""Named joint targets; real packing targets come from the active calibration."""
from pathlib import Path
import math

import yaml
from .motion_limits import DEFAULT_ANGULAR_SPEED_DEG_S, MAX_ANGULAR_SPEED_DEG_S

JOINT_NAMES = ["joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"]
MOCK_PACKING_DEG = [0.0, 100.7, -85.6, 0.0, 0.0, 0.0]
# Default adjustable joint speed for packing/zero pose buttons (degrees/second).
# The per-request output-shaft speed limit is enforced independently.
PRESET_JOINT_SPEED_DEG_S = DEFAULT_ANGULAR_SPEED_DEG_S


class PosePresets:
    def __init__(self, calibration_file="", hardware_type="real"):
        self.packing = None
        self.calibration_id = ""
        self.approved = False
        self.error = "未加载打包标定文件"
        if hardware_type == "mock":
            self.packing = list(MOCK_PACKING_DEG)
            self.approved = True
            self.error = ""
        elif calibration_file:
            try:
                data = yaml.safe_load(Path(calibration_file).read_text())["calibration"]
                if data["joint_names"] != JOINT_NAMES:
                    raise ValueError("标定关节顺序不匹配")
                values = data["packing_pose_joint_rad"]
                if len(values) != 6 or any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
                    raise ValueError("打包姿态必须包含六个有限角度")
                self.packing = [math.degrees(float(v)) for v in values]
                self.calibration_id = str(data["calibration_id"])
                self.approved = data["approved"] is True
                self.error = "" if self.approved else "打包标定尚未确认"
            except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
                self.error = f"打包标定读取失败：{exc}"

    def status(self, bridge, state):
        hardware = state.hardware if state is not None else None
        reasons = []
        if bridge.read_only:
            reasons.append("只读模式不提供运动")
        if bridge.commissioning:
            reasons.append("方向测试模式未启动整臂轨迹控制器，请在普通控制模式使用")
        if hardware is None:
            reasons.append("等待新鲜机械臂状态")
        else:
            if not hardware.calibrated:
                reasons.append("标定尚未确认或无效")
            if not hardware.kinematics_valid:
                reasons.append("运动学反解无效")
            if not hardware.control_permitted:
                reasons.append("硬件未授予整臂运动许可")
            if state.state_name != "READY" or state.task.status == 1:
                reasons.append("机械臂未就绪或正在运动")
            if list(hardware.joint_names) != JOINT_NAMES:
                reasons.append("关节顺序与预设姿态不匹配")
            if len(hardware.motor_fresh) != 6 or not all(hardware.motor_fresh):
                reasons.append("六轴反馈未全部就绪")
            if len(hardware.motor_enabled) != 6 or not all(hardware.motor_enabled):
                reasons.append("六轴电机未全部使能")
            if any(hardware.motor_errors) or any(hardware.motor_error_codes):
                reasons.append("电机报告故障")
            if hardware.motion_stop_pending or hardware.motion_stop_failed:
                reasons.append("停止尚未确认")
        result = {}
        for name, label, target in [("packing", "打包姿态", self.packing), ("zero", "零位姿态", [0.0]*6)]:
            denied = list(reasons)
            if name == "packing":
                if self.error:
                    denied.append(self.error)
                if hardware is not None and bridge.hardware_type == "real" and self.calibration_id != hardware.calibration_id:
                    denied.append("网页目标与硬件加载的标定 ID 不一致")
            result[name] = {"label": label, "target_joint_angles_deg": target,
                            "available": not denied, "reason": "；".join(denied),
                            "joint_speed_deg_s": PRESET_JOINT_SPEED_DEG_S,
                            "max_joint_speed_deg_s": MAX_ANGULAR_SPEED_DEG_S}
        return result
