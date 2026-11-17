"""校验网页命令和运行模式，在协议边界换算单位，再通过 ROS bridge 分发请求。"""
from __future__ import annotations

import asyncio
from typing import Any, TYPE_CHECKING

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from pi_arm_interfaces.action import DirectMove, MoveJ, MoveJs, MoveL
from pi_arm_interfaces.srv import AxisTest, ManageMotor, StopMotion, PreviewMotion, SetTeaching, SetZeroReference
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .motion_limits import DEFAULT_ANGULAR_SPEED_DEG_S, MAX_ANGULAR_SPEED_DEG_S
from .points import dispatch_points
from .presets import PosePresets, JOINT_NAMES, PRESET_JOINT_SPEED_DEG_S
from .protocol import (
    CODE_COMMAND_FAILED, CODE_INVALID_PARAMS, CODE_STATE_UNAVAILABLE,
    POINT_COMMANDS, ProtocolError, Request, integer_param, parse_request,
    required_float, required_vector, response,
)
from .rotations import DEG_TO_RAD, RAD_TO_DEG, quaternion_from_rpy, rpy_from_quaternion

if TYPE_CHECKING:
    from .ros_bridge import RosBridge


class CommandGateway:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge = bridge

    @staticmethod
    def _params_for_log(params: dict[str, Any]) -> Any:
        """Avoid flooding logs with large movejs target arrays."""
        targets = params.get("targets")
        if isinstance(targets, list) and len(targets) > 8:
            summary = dict(params)
            summary["targets"] = f"<{len(targets)} points>"
            return summary
        return params

    async def handle(self, raw: str) -> str:
        msg_id = 0
        try:
            request = parse_request(raw)
            msg_id = request.msg_id
            self.bridge.get_logger().info(
                f"WS 请求: msg_id={msg_id} cmd={request.command} "
                f"params={self._params_for_log(request.params)}"
            )
            data = await self.dispatch(request)
            summary = ({'revision': data.get('revision'), 'point_count': len(data.get('points', []))}
                       if request.command in POINT_COMMANDS else
                       data.get('message') if request.command == 'preview_motion' else data)
            self.bridge.get_logger().info(
                f"WS 响应成功: msg_id={msg_id} cmd={request.command} "
                f"data={summary}"
            )
            return response(msg_id, 0, "ok", data)
        except ProtocolError as exc:
            self.bridge.get_logger().warning(
                f"WS 协议/业务失败: msg_id={exc.msg_id or msg_id} "
                f"code={exc.code} message={exc.message}"
            )
            return response(exc.msg_id or msg_id, exc.code, exc.message)
        except Exception as exc:  # Keep the wire protocol stable at the process boundary.
            self.bridge.get_logger().error(f"WS 未处理异常: msg_id={msg_id} error={exc}")
            return response(msg_id, CODE_COMMAND_FAILED, f"命令执行失败: {exc}")

    async def dispatch(self, request: Request) -> dict[str, Any]:
        command, params = request.command, request.params
        if command in POINT_COMMANDS:
            return await asyncio.to_thread(dispatch_points, self.bridge, command, params)
        if self.bridge.read_only:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "只读排查模式：仅查看反馈，不提供控制服务")
        if command == "set_zero_reference":
            if not self.bridge.commissioning:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, "软件标零需要 commissioning 维护模式")
            if set(params) != {"reference_joint_positions_deg", "reference_pose_confirmed"} or params.get("reference_pose_confirmed") is not True:
                raise ProtocolError(CODE_INVALID_PARAMS, "必须明确参考姿态，并确认机械臂已处于该实际姿态")
            reference = required_vector(params, "reference_joint_positions_deg", 6)
            result = await self.bridge.call_service(command, SetZeroReference.Request(
                reference_joint_positions_rad=[value * DEG_TO_RAD for value in reference],
                reference_pose_confirmed=True,
            ))
            return {"reference_id": result.reference_id, "message": result.message,
                    "raw_protocol_positions_deg": list(result.raw_protocol_positions_deg),
                    "software_output_offsets_rad": list(result.software_output_offsets_rad)}
        if command == "set_teaching":
            if self.bridge.commissioning or set(params) != {"enabled"} or type(params["enabled"]) is not bool:
                raise ProtocolError(CODE_INVALID_PARAMS, "拖拽示教需要正常控制模式和 enabled 布尔参数")
            result = await self.bridge.call_service(command, SetTeaching.Request(enabled=params["enabled"]))
            return {"cmd": command, "message": result.message}
        if command == "preview_motion":
            return await self._preview(params)
        if command == "move_preset":
            name = params.get("preset")
            if name not in ("packing", "zero"):
                raise ProtocolError(CODE_INVALID_PARAMS, "preset 必须为 packing 或 zero")
            if set(params) - {"preset", "output_speed_limit", "max_speed"}:
                raise ProtocolError(CODE_INVALID_PARAMS, "仅接受 preset、max_speed 和 output_speed_limit 参数")
            state = self.bridge.state_snapshot()
            catalog = getattr(self.bridge, "presets", PosePresets()).status(self.bridge, state)
            preset = catalog[name]
            if not preset["available"]:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, preset["reason"])
            speed = self._angular_speed(params, "max_speed", PRESET_JOINT_SPEED_DEG_S)
            task_id = await self._direct(JOINT_NAMES, preset["target_joint_angles_deg"], speed,
                                         self._output_speed(params))
            return {"cmd": command, "preset": name, "task_id": task_id,
                    "target_joint_angles_deg": preset["target_joint_angles_deg"],
                    "joint_speed_deg_s": speed}
        if command.startswith("axis_test") or (
            self.bridge.commissioning and command in {"motor_stop", "stop_motion"}
        ):
            if not self.bridge.commissioning:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, "需要 --commissioning 单轴测试模式")
            ros_request = AxisTest.Request()
            ros_request.operation = {"axis_test": "start", "axis_test_heartbeat": "heartbeat"}.get(command, "stop")
            ros_request.test_id = integer_param(params, "test_id", default=0, allow_zero=True)
            if ros_request.operation == "start":
                ros_request.space = str(params.get("space", ""))
                ros_request.axis_id = integer_param(params, "axis_id")
                ros_request.delta_deg = required_float(params, "delta_deg")
                ros_request.speed_deg_s = self._angular_speed(params, "speed_deg_s")
            result = await self.bridge.call_service("axis_test", ros_request)
            return {"cmd": command, "axis_test_id": result.test_id, "message": result.message}
        if self.bridge.commissioning and command not in {"enable_motor", "disable_motor", "reset_motor", "reboot_motor"}:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "方向测试模式请使用电机/关节小步点动")
        if command in {"movej", "movejs", "movel", "move", "motor_move", "output_move"}:
            state = self.bridge.state_snapshot()
            if state is None or not state.hardware.control_permitted:
                raise ProtocolError(
                    CODE_STATE_UNAVAILABLE,
                    "运动被禁止：标定、运动学或硬件控制许可未满足",
                    request.msg_id,
                )
        if command in {"enable_motor", "disable_motor", "reset_motor", "reboot_motor"}:
            motor_id = integer_param(params, "motor_id", allow_zero=command != "reboot_motor")
            ros_request = ManageMotor.Request()
            ros_request.motor_id = motor_id
            result = await self.bridge.call_service(command, ros_request)
            motor_ids = [motor_id] if motor_id else list(range(1, len(self._joint_names()) + 1))
            return {"cmd": command, "motor_ids": motor_ids, "message": getattr(result, "message", "ok")}
        if command == "stop_motion":
            state = self.bridge.state_snapshot()
            task_id = int(state.task.task_id) if state is not None else None
            result = await self.bridge.call_service(command, StopMotion.Request())
            return {
                "cmd": command,
                "stopped": result.message != "no active motion",
                "task_id": task_id if result.message != "no active motion" else None,
                "message": result.message,
            }
        if command == "movej":
            return await self._movej(params)
        if command == "movejs":
            return await self._movejs(params)
        if command == "movel":
            return await self._movel(params)
        if command == "move":
            targets = self._joint_vector(params, "targets")
            speed = self._angular_speed(params, "max_speed")
            task_id = await self._direct(self._joint_names(), targets, speed, self._output_speed(params))
            return {"cmd": "move", "task_id": task_id}
        if command == "output_move":
            targets = required_vector(params, "targets", 6)
            speed = self._output_speed(params)
            task_id = await self._direct(JOINT_NAMES, targets, MAX_ANGULAR_SPEED_DEG_S, speed, output_space=True)
            return {"cmd": command, "task_id": task_id, "target_output_deg": targets,
                    "output_speed_limit": speed * RAD_TO_DEG}
        if command == "motor_move":
            return await self._motor_move(params, stop=False)
        if command == "motor_stop":
            return await self._motor_move(params, stop=True)
        raise ProtocolError(CODE_INVALID_PARAMS, f"未实现命令: {command}")

    async def _preview(self, params: dict[str, Any]) -> dict[str, Any]:
        if self.bridge.commissioning:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "方向测试模式不提供规划预览")
        mode = params.get("mode")
        if mode not in {"joint", "output", "ik", "lin"}:
            raise ProtocolError(CODE_INVALID_PARAMS, "mode 必须为 joint/output/ik/lin")
        request = PreviewMotion.Request()
        request.mode = mode
        request.max_output_velocity = self._output_speed(params)
        if mode in {"joint", "output"}:
            request.positions = [v * DEG_TO_RAD for v in required_vector(params, "targets", 6)]
            request.max_velocity = (MAX_ANGULAR_SPEED_DEG_S if mode == "output" else
                self._angular_speed(params, "max_speed")) * DEG_TO_RAD
        else:
            target = required_vector(params, "target", 6)
            request.target.position.x, request.target.position.y, request.target.position.z = [v / 1000.0 for v in target[:3]]
            q = quaternion_from_rpy(*[v * DEG_TO_RAD for v in target[3:]])
            (request.target.orientation.x, request.target.orientation.y,
             request.target.orientation.z, request.target.orientation.w) = q
            if mode == "lin":
                request.max_linear_velocity = required_float(params, "linear_vel", positive=True) / 1000.0
                request.max_linear_acceleration = required_float(params, "linear_acc", positive=True) / 1000.0
                request.max_angular_velocity = self._angular_speed(params, "angular_vel") * DEG_TO_RAD
        result = await self.bridge.call_service("preview_motion", request)
        pose = result.target_pose
        rpy = rpy_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return dict(cmd="preview_motion", mode=mode, message=result.message,
                    joint_names=list(result.joint_names),
                    start_joint_angles_deg=[v * RAD_TO_DEG for v in result.start_positions],
                    target_joint_angles_deg=[v * RAD_TO_DEG for v in result.target_positions],
                    target_output_angles_deg=[v * RAD_TO_DEG for v in result.output_positions],
                    target_pose=[pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000,
                                 *[v * RAD_TO_DEG for v in rpy]],
                    points=[dict(t=p.time_from_start.sec + p.time_from_start.nanosec / 1e9,
                                 joints=[v * RAD_TO_DEG for v in p.positions],
                                 tcp=[tcp.x, tcp.y, tcp.z])
                            for p, tcp in zip(result.trajectory.points, result.tcp_path)])

    def _joint_names(self) -> list[str]:
        state = self.bridge.state_snapshot()
        if state and state.hardware.joint_names:
            return list(state.hardware.joint_names)
        return list(self.bridge.default_joint_names)

    def _joint_vector(self, params: dict[str, Any], name: str) -> list[float]:
        return required_vector(params, name, len(self._joint_names()))

    async def _movej(self, params: dict[str, Any]) -> dict[str, Any]:
        target = self._joint_vector(params, "target")
        velocity = self._angular_speed(params, "vel")
        acceleration = required_float(params, "acc", positive=True)
        goal = MoveJ.Goal()
        goal.max_output_velocity = self._output_speed(params)
        goal.joint_names = self._joint_names()
        goal.positions = [value * DEG_TO_RAD for value in target]
        goal.max_velocity = velocity * DEG_TO_RAD
        goal.max_acceleration = acceleration * DEG_TO_RAD
        task_id = await self.bridge.send_goal("movej", goal)
        return {"cmd": "movej", "task_id": task_id, "target": target, "vel": velocity, "acc": acceleration}

    async def _movejs(self, params: dict[str, Any]) -> dict[str, Any]:
        targets_value = params.get("targets")
        if not isinstance(targets_value, list) or not targets_value:
            raise ProtocolError(CODE_INVALID_PARAMS, "targets 必须是非空数组")
        if len(targets_value) > 50_000:
            raise ProtocolError(CODE_INVALID_PARAMS, "targets 不能超过 50000 组")
        dt_ms = integer_param(params, "dt")
        names = self._joint_names()
        trajectory = JointTrajectory()
        trajectory.joint_names = names
        targets: list[list[float]] = []
        for index, value in enumerate(targets_value):
            target = required_vector({"target": value}, "target", len(names))
            targets.append(target)
            point = JointTrajectoryPoint()
            point.positions = [item * DEG_TO_RAD for item in target]
            nanoseconds = (index + 1) * dt_ms * 1_000_000
            point.time_from_start = Duration(
                sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000,
            )
            trajectory.points.append(point)
        goal = MoveJs.Goal()
        goal.max_output_velocity = self._output_speed(params)
        goal.trajectory = trajectory
        task_id = await self.bridge.send_goal("movejs", goal)
        return {"cmd": "movejs", "task_id": task_id, "point_count": len(targets), "dt": dt_ms}

    async def _movel(self, params: dict[str, Any]) -> dict[str, Any]:
        target = required_vector(params, "target", 6)
        linear_velocity = required_float(params, "linear_vel", positive=True)
        linear_acceleration = required_float(params, "linear_acc", positive=True)
        angular_velocity = self._angular_speed(params, "angular_vel")
        angular_acceleration = required_float(params, "angular_acc", positive=True)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            target[0] / 1000.0,
            target[1] / 1000.0,
            target[2] / 1000.0,
        )
        qx, qy, qz, qw = quaternion_from_rpy(
            target[3] * DEG_TO_RAD, target[4] * DEG_TO_RAD, target[5] * DEG_TO_RAD
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = MoveL.Goal()
        goal.max_output_velocity = self._output_speed(params)
        goal.target = pose
        goal.max_linear_velocity = linear_velocity / 1000.0
        goal.max_linear_acceleration = linear_acceleration / 1000.0
        goal.max_angular_velocity = angular_velocity * DEG_TO_RAD
        # Angular acceleration is accepted for protocol compatibility; Pilz LIN
        # derives rotational acceleration from cartesian_limits and does not use
        # this field as an independent timing constraint.
        goal.max_angular_acceleration = angular_acceleration * DEG_TO_RAD
        task_id = await self.bridge.send_goal("movel", goal)
        return {
            "cmd": "movel",
            "task_id": task_id,
            "target": target,
            "linear_vel": linear_velocity,
            "linear_acc": linear_acceleration,
            "angular_vel": angular_velocity,
            "angular_acc": angular_acceleration,
        }

    async def _direct(
        self, names: list[str], targets_deg: list[float], speed_deg_s: float,
        output_speed: float = DEFAULT_ANGULAR_SPEED_DEG_S * DEG_TO_RAD, *, output_space: bool = False
    ) -> int:
        goal = DirectMove.Goal()
        goal.joint_names = names
        goal.positions = [value * DEG_TO_RAD for value in targets_deg]
        goal.max_velocity = speed_deg_s * DEG_TO_RAD
        goal.max_output_velocity = output_speed
        goal.output_space = output_space
        return await self.bridge.send_goal("direct", goal)

    @staticmethod
    def _angular_speed(params: dict[str, Any], name: str,
                       default: float = DEFAULT_ANGULAR_SPEED_DEG_S) -> float:
        speed = required_float({name: params.get(name, default)}, name, positive=True)
        if speed > MAX_ANGULAR_SPEED_DEG_S:
            raise ProtocolError(CODE_INVALID_PARAMS, f"{name} 不得超过 {MAX_ANGULAR_SPEED_DEG_S:g} °/s")
        return speed

    @staticmethod
    def _output_speed(params: dict[str, Any]) -> float:
        speed = CommandGateway._angular_speed(params, "output_speed_limit")
        if speed < 1.0 / 6.0:
            raise ProtocolError(CODE_INVALID_PARAMS, "输出轴限速不得低于协议分辨率 1/6 °/s")
        return speed * DEG_TO_RAD

    async def _motor_move(self, params: dict[str, Any], *, stop: bool) -> dict[str, Any]:
        joint_id = integer_param(params, "joint_id")
        names = self._joint_names()
        if joint_id > len(names):
            raise ProtocolError(CODE_INVALID_PARAMS, "joint_id 不在当前机械臂配置中")
        if stop:
            # motor_stop 将所选关节的停止操作转为整条任务的 StopMotion 请求。
            await self.bridge.call_service("stop_motion", StopMotion.Request())
            return {"cmd": "motor_stop", "joint_id": joint_id}
        speed = self._angular_speed(params, "max_speed")
        angle = required_float(params, "target_angle")
        task_id = await self._direct([names[joint_id - 1]], [angle], speed, self._output_speed(params))
        return {
            "cmd": "motor_move",
            "task_id": task_id,
            "joint_id": joint_id,
            "target_angle": angle,
            "max_speed": speed,
        }
