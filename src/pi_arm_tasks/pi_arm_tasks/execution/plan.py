"""校验程序并展开为有限步骤。"""

from copy import deepcopy
import math

MOTIONS = {
    "move",
    "movej",
    "movel",
    "step_move",
    "move_to_point",
    "movejs",
    "motor_move",
    "output_move",
    "move_preset",
}
MAX_STEPS = 10000


def number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} 必须是有限数字")
    if positive and value <= 0:
        raise ValueError(f"{label} 必须大于零")
    return value


def vector(value, label, size=6):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label} 必须包含 {size} 个数字")
    return [number(v, label) for v in value]


def validate_motion(command, params):
    if not isinstance(params, dict):
        raise ValueError("运动参数必须是对象")
    if command in ("move", "movej", "movel"):
        key = "targets" if command == "move" else "target"
        vector(params.get(key), key)
        fields = {
            "move": ["max_speed"],
            "movej": ["vel", "acc"],
            "movel": ["linear_vel", "linear_acc", "angular_vel", "angular_acc"],
        }[command]
        for field in fields:
            number(params.get(field), field, positive=True)
        for field in ("max_speed", "vel", "angular_vel"):
            if field in params and params[field] > 10:
                raise ValueError("关节和角速度上限为 10 度/秒")
    elif command == "step_move":
        space = params.get("space")
        if space not in ("joint", "position", "orientation"):
            raise ValueError("步进空间无效")
        axis = params.get("axis")
        if type(axis) is not int or not 0 <= axis < (6 if space == "joint" else 3):
            raise ValueError("步进轴序号无效")
        if number(params.get("delta"), "步长") == 0:
            raise ValueError("步长不能为零")
    elif command == "move_to_point":
        if not isinstance(params.get("point"), str) or not params["point"]:
            raise ValueError("请选择点位")
        if params.get("motion") not in ("move", "movej", "movel"):
            raise ValueError("点位运动方式必须为 move/movej/movel")
    elif command == "movejs":
        targets = params.get("targets")
        if not isinstance(targets, list) or not 1 <= len(targets) <= 50000:
            raise ValueError("轨迹需要 1–50000 组目标")
        for row in targets:
            vector(row, "targets")
        if type(params.get("dt")) is not int or params["dt"] <= 0:
            raise ValueError("dt 必须为正整数毫秒")
    elif command == "output_move":
        vector(params.get("targets"), "targets")
    elif command == "motor_move":
        if type(params.get("joint_id")) is not int or not 1 <= params["joint_id"] <= 6:
            raise ValueError("joint_id 应为 1–6")
        number(params.get("target_angle"), "目标角")
        if number(params.get("max_speed", 5.0), "速度", positive=True) > 10:
            raise ValueError("关节和角速度上限为 10 度/秒")
    elif command == "move_preset":
        if params.get("preset") not in ("packing", "zero"):
            raise ValueError("预设姿态无效")
    else:
        raise ValueError("不支持的运动方式")
    if (
        "output_speed_limit" in params
        and not 1 / 6 <= number(params["output_speed_limit"], "输出轴限速") <= 10
    ):
        raise ValueError("输出轴限速范围为 1/6–10 度/秒")


def validate_program(document):
    if not isinstance(document, dict) or document.get("schema_version", 1) != 1:
        raise ValueError("程序格式版本无效")
    name = document.get("name")
    if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80:
        raise ValueError("程序名称应为 1–80 个字符")
    steps = document.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("程序至少需要一个步骤")
    expanded = []

    def append(rows, inside=False, cycle=None):
        if not isinstance(rows, list) or not rows:
            raise ValueError("循环体不能为空")
        for index, step in enumerate(rows):
            if not isinstance(step, dict):
                raise ValueError("步骤必须是对象")
            kind = step.get("type")
            if kind == "repeat":
                count = step.get("count")
                if inside or type(count) is not int or not 1 <= count <= MAX_STEPS:
                    raise ValueError("循环次数应为正整数，且仅支持单层循环")
                for iteration in range(count):
                    append(step.get("steps"), True, iteration + 1)
                continue
            if kind == "wait":
                if number(step.get("seconds"), "等待时间") < 0:
                    raise ValueError("等待时间不能为负数")
            elif kind in ("move", "movej", "movel"):
                if "point_id" not in step:
                    validate_motion(kind, step.get("params"))
                elif not isinstance(step["point_id"], str) or not step["point_id"]:
                    raise ValueError("点位编号无效")
            else:
                raise ValueError("程序仅支持运动、等待和有限循环")
            result = deepcopy(step)
            result["source_index"] = step.get("source_index", index)
            if cycle is not None:
                result["cycle"] = cycle
            expanded.append(result)
            if len(expanded) > MAX_STEPS:
                raise ValueError(f"展开后最多 {MAX_STEPS} 个步骤")

    append(steps)
    return expanded
