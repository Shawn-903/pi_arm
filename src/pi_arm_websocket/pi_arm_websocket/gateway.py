from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from itertools import count
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from pi_arm_interfaces.action import DirectMove, MoveJ, MoveJs, MoveL
from pi_arm_interfaces.msg import HardwareState, RobotState, TaskState
from pi_arm_interfaces.srv import AxisTest, ManageMotor, StopMotion, PreviewMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import websockets
from websockets.exceptions import ConnectionClosed
from .presets import PosePresets, JOINT_NAMES, PRESET_JOINT_SPEED_DEG_S
from .frames import FrameModel
from .points import PointStore, dispatch_points

from .protocol import (
    CODE_COMMAND_FAILED,
    CODE_INVALID_PARAMS,
    CODE_MOTION_BUSY,
    CODE_STATE_UNAVAILABLE,
    ProtocolError,
    POINT_COMMANDS,
    Request,
    integer_param,
    parse_request,
    required_float,
    required_vector,
    response,
)

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("pi_arm_websocket")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("hardware_type", "real")
        self.declare_parameter("read_only", False)
        self.declare_parameter("commissioning", False)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("state_timeout_sec", 1.0)
        self.declare_parameter("port", 8765)
        self.declare_parameter("state_push_interval_ms", 100)
        self.declare_parameter("max_size_bytes", 32 * 1024 * 1024)
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter(
            "joint_names",
            ["joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"],
        )
        self.hardware_type = str(self.get_parameter("hardware_type").value)
        self.declare_parameter('points_file', '')
        self.declare_parameter('frame_model_file', str(Path(get_package_share_directory('pi_arm_description')) / 'urdf/pi_arm.urdf.xacro'))
        self.points = PointStore(str(self.get_parameter('points_file').value) or str(Path.home() / '.local/share/pi_arm' / f'points-{self.hardware_type}.json'))
        self.frame_model = FrameModel(str(self.get_parameter('frame_model_file').value),
                                      list(self.get_parameter('joint_names').value))
        self.read_only = bool(self.get_parameter("read_only").value)
        self.commissioning = bool(self.get_parameter("commissioning").value)
        self.presets = PosePresets(str(self.get_parameter("calibration_file").value), self.hardware_type)
        if self.read_only and self.commissioning:
            raise ValueError("read_only and commissioning are mutually exclusive")
        self.state_timeout = float(self.get_parameter("state_timeout_sec").value)
        self._state_received_at = 0.0
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.push_interval = max(
            0.01, float(self.get_parameter("state_push_interval_ms").value) / 1000.0
        )
        self.max_size = max(1024 * 1024, int(self.get_parameter("max_size_bytes").value))
        self.timeout = float(self.get_parameter("request_timeout_sec").value)
        if not self.host or not 1 <= self.port <= 65535 or self.timeout <= 0.0:
            raise ValueError("WebSocket host、port 或 request_timeout_sec 参数非法")
        if self.hardware_type not in {"real", "mock"} or not math.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError("hardware_type 或 state_timeout_sec 参数非法")
        self.default_joint_names = list(self.get_parameter("joint_names").value)
        self._state: RobotState | None = None
        self._state_lock = threading.Lock()
        # ClientGoalHandle is unhashable in Jazzy rclpy; track with a list.
        self._active_goals: list[Any] = []
        self._goal_lock = threading.Lock()

        if self.read_only or self.commissioning:
            self.create_subscription(
                HardwareState, "/pi_arm/hardware_state", self._on_hardware_state,
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(RobotState, "/pi_arm/state", self._on_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service_clients = {} if self.read_only or self.commissioning else {
            "enable_motor": self.create_client(ManageMotor, "/pi_arm/enable_motor"),
            "disable_motor": self.create_client(ManageMotor, "/pi_arm/disable_motor"),
            "reset_motor": self.create_client(ManageMotor, "/pi_arm/reset_motor"),
            "reboot_motor": self.create_client(ManageMotor, "/pi_arm/reboot_motor"),

            "stop_motion": self.create_client(StopMotion, "/pi_arm/stop_motion"),
            "preview_motion": self.create_client(PreviewMotion, "/pi_arm/preview_motion"),
        }
        self.action_clients = {} if self.read_only or self.commissioning else {
            "movej": ActionClient(self, MoveJ, "/pi_arm/movej"),
            "movejs": ActionClient(self, MoveJs, "/pi_arm/movejs"),
            "movel": ActionClient(self, MoveL, "/pi_arm/movel"),
            "direct": ActionClient(self, DirectMove, "/pi_arm/direct_move"),
        }
        if self.commissioning:
            # No normal manager/controller commands exist in this mode.
            for client in self.service_clients.values():
                self.destroy_client(client)
            for client in self.action_clients.values():
                client.destroy()
            self.action_clients = {}
            self.service_clients = {
                name: self.create_client(ManageMotor, "/pi_arm/commissioning/" + name)
                for name in ("enable_motor", "disable_motor", "reset_motor", "reboot_motor")
            }
            self.service_clients["axis_test"] = self.create_client(AxisTest, "/pi_arm/commissioning/axis_test")

    def _on_hardware_state(self, hardware: HardwareState) -> None:
        self._on_state(monitor_state(hardware))

    def _on_state(self, message: RobotState) -> None:
        with self._state_lock:
            self._state = message
            self._state_received_at = time.monotonic()

    def state_snapshot(self) -> RobotState | None:
        with self._state_lock:
            if time.monotonic() - self._state_received_at > self.state_timeout:
                return None
            return self._state

    async def wait_future(self, future: Any, timeout: float | None = None) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        ok = await asyncio.to_thread(completed.wait, timeout or self.timeout)
        if not ok:
            self.get_logger().error(f"ROS future 超时 ({timeout or self.timeout:.1f}s)")
            raise ProtocolError(CODE_COMMAND_FAILED, "ROS 请求超时")
        exception = future.exception()
        if exception is not None:
            self.get_logger().error(f"ROS future 失败: {exception}")
            raise ProtocolError(CODE_COMMAND_FAILED, f"ROS 请求失败: {exception}")
        return future.result()

    async def call_service(self, command: str, request: Any) -> Any:
        client = self.service_clients[command]
        self.get_logger().info(f"调用服务 {command}")
        available = await asyncio.to_thread(client.wait_for_service, self.timeout)
        if not available:
            self.get_logger().error(f"服务不可用: {command}")
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "manager 服务不可用")
        result = await self.wait_future(client.call_async(request), 12.0 if command == "preview_motion" else None)
        if not result.success:
            code = int(result.code) if result.code else CODE_COMMAND_FAILED
            self.get_logger().warning(
                f"服务返回失败: cmd={command} code={code} message={result.message}"
            )
            raise ProtocolError(code, result.message)
        self.get_logger().info(f"服务成功: {command}")
        return result

    def _track_goal(self, goal_handle: Any) -> Any:
        result_future = goal_handle.get_result_async()
        with self._goal_lock:
            self._active_goals.append(goal_handle)

        def release(_: Any) -> None:
            with self._goal_lock:
                if goal_handle in self._active_goals:
                    self._active_goals.remove(goal_handle)

        result_future.add_done_callback(release)
        return result_future

    def _cancel_late_goal(self, future: Any) -> None:
        """Runs on the ROS executor even after the WebSocket request timed out."""
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                self._track_goal(handle)
                cancel_future = handle.cancel_goal_async()

                def cancellation_done(result: Any) -> None:
                    try:
                        reply = result.result()
                        if not reply.goals_canceling:
                            self.get_logger().error("迟到目标取消未被接受，请检查任务状态并停止")
                    except Exception as exc:
                        self.get_logger().error(f"迟到目标取消失败，停止未确认: {exc}")

                cancel_future.add_done_callback(cancellation_done)
                self.get_logger().warning("已请求取消超时后接受的目标，等待终态确认")
        except Exception as exc:
            self.get_logger().error(f"目标接受状态确认失败: {exc}")

    async def _cancel_accepted_goal(self, handle: Any, result_future: Any) -> bool:
        try:
            if not result_future.done():
                await self.wait_future(handle.cancel_goal_async())
            # A cancel response alone does not prove that motion has stopped.
            result = await self.wait_future(result_future)
            return result.status in (4, 5, 6)  # SUCCEEDED, CANCELED, ABORTED
        except Exception as exc:
            self.get_logger().error(f"已接受目标停止未确认: {exc}")
            return False

    async def send_goal(self, action_name: str, goal: Any) -> int:
        state = self.state_snapshot()
        if state is None:
            self.get_logger().warning(f"action {action_name} 拒绝: 状态尚未就绪")
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "机械臂状态尚未就绪")
        # Busy only while a task is actually running. Do not treat residual
        # robot_state=RUNNING (e.g. still_frames mask after FAILED) as busy.
        if state.task.status == TaskState.RUNNING:
            self.get_logger().warning(
                f"action {action_name} 拒绝: 忙 (task={state.task.command}/{state.task.task_id})"
            )
            raise ProtocolError(CODE_MOTION_BUSY, "已有运动任务正在运行")
        effective_state = state.state
        if (
            effective_state == RobotState.RUNNING
            and state.task.status != TaskState.RUNNING
        ):
            effective_state = RobotState.READY
        if effective_state != RobotState.READY:
            self.get_logger().warning(
                f"action {action_name} 拒绝: 状态={state.state_name}"
            )
            raise ProtocolError(CODE_STATE_UNAVAILABLE, f"当前状态不允许运动: {state.state_name}")
        client = self.action_clients[action_name]
        available = await asyncio.to_thread(client.wait_for_server, self.timeout)
        if not available:
            self.get_logger().error(f"action server 不可用: {action_name}")
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "manager action 不可用")
        self.get_logger().info(f"发送 action goal: {action_name}")
        feedback_ready = threading.Event()
        task_id_box: list[int] = []

        def feedback_callback(message: Any) -> None:
            if not task_id_box:
                task_id_box.append(int(message.feedback.task_id))
                feedback_ready.set()

        acceptance = client.send_goal_async(goal, feedback_callback=feedback_callback)
        try:
            goal_handle = await self.wait_future(acceptance)
        except (ProtocolError, asyncio.CancelledError):
            acceptance.add_done_callback(self._cancel_late_goal)
            raise ProtocolError(CODE_COMMAND_FAILED,
                "目标接受状态未知；已安排取消迟到目标，请检查任务状态，必要时点击停止")
        if not goal_handle.accepted:
            self.get_logger().warning(f"action goal 被 manager 拒绝: {action_name}")
            raise ProtocolError(CODE_MOTION_BUSY, "运动目标被拒绝")
        result_future = self._track_goal(goal_handle)
        received = await asyncio.to_thread(feedback_ready.wait, self.timeout)
        if not received:
            self.get_logger().error(f"action {action_name} 未在超时内返回 task_id")
            ended = await self._cancel_accepted_goal(goal_handle, result_future)
            detail = "已请求取消，目标已结束" if ended else "停止未确认，请点击停止并检查任务状态"
            raise ProtocolError(CODE_COMMAND_FAILED, f"目标已接受但任务 ID 反馈超时；{detail}")
        self.get_logger().info(f"action {action_name} 已接受: task_id={task_id_box[0]}")
        return task_id_box[0]


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
        if command == "preview_motion":
            return await self._preview(params)
        if command == "move_preset":
            name = params.get("preset")
            if name not in ("packing", "zero"):
                raise ProtocolError(CODE_INVALID_PARAMS, "preset 必须为 packing 或 zero")
            if set(params) - {"preset", "output_speed_limit"}:
                raise ProtocolError(CODE_INVALID_PARAMS, "仅接受 preset 和 output_speed_limit 参数")
            state = self.bridge.state_snapshot()
            catalog = getattr(self.bridge, "presets", PosePresets()).status(self.bridge, state)
            preset = catalog[name]
            if not preset["available"]:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, preset["reason"])
            task_id = await self._direct(JOINT_NAMES, preset["target_joint_angles_deg"], PRESET_JOINT_SPEED_DEG_S,
                                         self._output_speed(params))
            return {"cmd": command, "preset": name, "task_id": task_id,
                    "target_joint_angles_deg": preset["target_joint_angles_deg"],
                    "joint_speed_deg_s": PRESET_JOINT_SPEED_DEG_S}
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
                ros_request.speed_deg_s = required_float(params, "speed_deg_s", positive=True)
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
            }
        if command == "movej":
            return await self._movej(params)
        if command == "movejs":
            return await self._movejs(params)
        if command == "movel":
            return await self._movel(params)
        if command == "move":
            targets = self._joint_vector(params, "targets")
            speed = required_float(params, "max_speed", positive=True)
            task_id = await self._direct(self._joint_names(), targets, speed, self._output_speed(params))
            return {"cmd": "move", "task_id": task_id}
        if command == "output_move":
            targets = required_vector(params, "targets", 6)
            speed = self._output_speed(params)
            task_id = await self._direct(JOINT_NAMES, targets, 1.0, speed, output_space=True)
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
            request.max_velocity = (1.0 if mode == "output" else
                required_float(params, "max_speed", positive=True)) * DEG_TO_RAD
        else:
            target = required_vector(params, "target", 6)
            request.target.position.x, request.target.position.y, request.target.position.z = [v / 1000.0 for v in target[:3]]
            q = quaternion_from_rpy(*[v * DEG_TO_RAD for v in target[3:]])
            (request.target.orientation.x, request.target.orientation.y,
             request.target.orientation.z, request.target.orientation.w) = q
            if mode == "lin":
                request.max_linear_velocity = required_float(params, "linear_vel", positive=True) / 1000.0
                request.max_linear_acceleration = required_float(params, "linear_acc", positive=True) / 1000.0
                request.max_angular_velocity = required_float(params, "angular_vel", positive=True) * DEG_TO_RAD
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
        velocity = required_float(params, "vel", positive=True)
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
        angular_velocity = required_float(params, "angular_vel", positive=True)
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
        output_speed: float = 5.0 * DEG_TO_RAD, *, output_space: bool = False
    ) -> int:
        goal = DirectMove.Goal()
        goal.joint_names = names
        goal.positions = [value * DEG_TO_RAD for value in targets_deg]
        goal.max_velocity = speed_deg_s * DEG_TO_RAD
        goal.max_output_velocity = output_speed
        goal.output_space = output_space
        return await self.bridge.send_goal("direct", goal)

    @staticmethod
    def _output_speed(params: dict[str, Any]) -> float:
        speed = required_float({"limit": params.get("output_speed_limit", 5.0)}, "limit", positive=True)
        if speed < 1.0 / 6.0:
            raise ProtocolError(CODE_INVALID_PARAMS, "输出轴限速不得低于协议分辨率 1/6 °/s")
        return speed * DEG_TO_RAD

    async def _motor_move(self, params: dict[str, Any], *, stop: bool) -> dict[str, Any]:
        joint_id = integer_param(params, "joint_id")
        names = self._joint_names()
        if joint_id > len(names):
            raise ProtocolError(CODE_INVALID_PARAMS, "joint_id 不在当前机械臂配置中")
        if stop:
            # Stopping never creates a fresh movement goal or requires READY.
            await self.bridge.call_service("stop_motion", StopMotion.Request())
            return {"cmd": "motor_stop", "joint_id": joint_id}
        speed = required_float({"max_speed": params.get("max_speed", 1)}, "max_speed", positive=True)
        angle = required_float(params, "target_angle")
        task_id = await self._direct([names[joint_id - 1]], [angle], speed, self._output_speed(params))
        return {
            "cmd": "motor_move",
            "task_id": task_id,
            "joint_id": joint_id,
            "target_angle": angle,
            "max_speed": speed,
        }


class WebSocketGateway:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge = bridge
        self.commands = CommandGateway(bridge)
        self.clients: set[Any] = set()
        self.counter = count(1)

    async def run(self) -> None:
        self.bridge.get_logger().info(
            f"WebSocket 网关监听 ws://{self.bridge.host}:{self.bridge.port}"
        )
        async with websockets.serve(
            self._client,
            self.bridge.host,
            self.bridge.port,
            max_size=self.bridge.max_size,
            ping_interval=20,
            ping_timeout=20,
        ):
            await self._push_loop()

    def _peer(self, websocket: Any) -> str:
        remote = getattr(websocket, "remote_address", None)
        if isinstance(remote, tuple) and len(remote) >= 2:
            return f"{remote[0]}:{remote[1]}"
        return str(remote or "unknown")

    async def _client(self, websocket: Any) -> None:
        peer = self._peer(websocket)
        self.clients.add(websocket)
        self.bridge.get_logger().info(
            f"WebSocket 客户端接入: {peer} (连接数={len(self.clients)})"
        )
        owned_test = None
        try:
            async for message in websocket:
                raw = message if isinstance(message, str) else "{}"
                reply = await self.commands.handle(raw)
                data = json.loads(reply)
                if data["code"] == 0 and data["data"].get("cmd") == "axis_test":
                    owned_test = data["data"]["axis_test_id"]
                await websocket.send(reply)
        except ConnectionClosed as exc:
            self.bridge.get_logger().info(
                f"WebSocket 连接关闭: {peer} code={exc.code} reason={exc.reason!r}"
            )
        except Exception as exc:
            self.bridge.get_logger().error(f"WebSocket 客户端异常: {peer} error={exc}")
        finally:
            if owned_test is not None:
                try:
                    request = AxisTest.Request(operation="stop", test_id=owned_test)
                    await self.bridge.call_service("axis_test", request)
                except Exception as exc:
                    self.bridge.get_logger().warning(f"Disconnect stop request: {exc}")
            self.clients.discard(websocket)
            self.bridge.get_logger().info(
                f"WebSocket 客户端离开: {peer} (连接数={len(self.clients)})"
            )

    async def _push_loop(self) -> None:
        while rclpy.ok():
            await asyncio.sleep(self.bridge.push_interval)
            if not self.clients:
                continue
            payload = self._state_message()
            dead = []
            for client in tuple(self.clients):
                try:
                    await client.send(payload)
                except ConnectionClosed:
                    dead.append(client)
            for client in dead:
                self.clients.discard(client)

    def _state_message(self) -> str:
        state = self.bridge.state_snapshot()
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
                # Legacy `enabled` remains joint-level for compatibility.
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
        body["pose"] = self._pose() if state is not None and not getattr(self.bridge, 'frame_model', None) else {}
        body["hardware_type"] = self.bridge.hardware_type
        preview = getattr(self.bridge, "service_clients", {}).get("preview_motion")
        reboot = getattr(self.bridge, "service_clients", {}).get("reboot_motor")
        body["reboot_available"] = bool(reboot and reboot.service_is_ready())
        body["preview_modes"] = ["joint", "output", "ik", "lin"] if preview and preview.service_is_ready() else []
        body["read_only"] = self.bridge.read_only
        body["commissioning"] = self.bridge.commissioning
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
        body["motion_stop_pending"] = bool(hardware and hardware.motion_stop_pending)
        body["motion_stop_failed"] = bool(hardware and hardware.motion_stop_failed)
        body["state_available"] = state is not None
        body["preset_poses"] = getattr(self.bridge, "presets", PosePresets()).status(self.bridge, state)
        body["joint_names"] = list(state.hardware.joint_names) if state else []
        body["hardware_connected"] = state.hardware.connected if state else False
        body["driver_mode"] = state.hardware.driver_mode if state else "UNKNOWN"
        body["joint_fresh"] = list(state.hardware.fresh) if state else []
        body["joint_ages_sec"] = list(state.hardware.ages_sec) if state else []
        if self.bridge.read_only or self.bridge.commissioning:
            body["control_permitted"] = False
            body["pose"] = {}
        body["supported_motion_commands"] = ["move", "motor_move", "output_move", "movej", "movejs", "movel"]
        if self.bridge.read_only or self.bridge.commissioning:
            body["supported_motion_commands"] = []
        model = getattr(self.bridge, 'frame_model', None)
        body['base_frames'] = model.feedback(hardware) if model else {'valid': False, 'joints': [], 'tool': None}
        body['points_available'] = hasattr(self.bridge, 'points') and model is not None
        if model:
            tip = body['base_frames'].get('tool')
            # Use the exact same feedback snapshot as joint/base-frame display and capture.
            body['pose'] = dict(zip(('x','y','z','rx','ry','rz'), tip['position_mm'] + tip['rpy_deg'])) if tip else {}
        return json.dumps(
            json_safe({"type": "state", "msg_id": next(self.counter), **body}),
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
        )

    def _pose(self) -> dict[str, float | None]:
        try:
            transform = self.bridge.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            roll, pitch, yaw = rpy_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            return {
                "x": transform.translation.x * 1000.0,
                "y": transform.translation.y * 1000.0,
                "z": transform.translation.z * 1000.0,
                "rx": roll * RAD_TO_DEG,
                "ry": pitch * RAD_TO_DEG,
                "rz": yaw * RAD_TO_DEG,
            }
        except TransformException:
            return {"x": None, "y": None, "z": None, "rx": None, "ry": None, "rz": None}


def monitor_state(hardware: HardwareState) -> RobotState:
    """Diagnostic state without MoveIt, controllers, manager or motion services."""
    state = RobotState()
    state.hardware = hardware
    fresh = bool(hardware.motor_ids) and len(hardware.motor_fresh) == len(hardware.motor_ids) and all(hardware.motor_fresh)
    if not hardware.connected or not fresh:
        state.state, state.state_name = RobotState.DISCONNECTED, "DISCONNECTED"
    elif (not hardware.calibrated or not hardware.kinematics_valid or
          any(hardware.motor_errors) or any(hardware.motor_error_codes) or
          any(hardware.errors) or any(hardware.error_codes)):
        state.state, state.state_name = RobotState.FAULT, "FAULT"
    else:
        state.state, state.state_name = RobotState.DISABLED, "DISABLED"
    return state


def json_safe(value: Any) -> Any:
    """ROS unavailable samples may contain NaN/Infinity, which browsers reject."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, ...]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main() -> None:
    rclpy.init()
    bridge = RosBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        asyncio.run(WebSocketGateway(bridge).run())
    except KeyboardInterrupt:
        bridge.get_logger().info("WebSocket 网关收到退出信号")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
