"""连接 WebSocket 协程与 ROS 订阅、服务和动作，保存反馈快照并跟踪动作终态。"""
from __future__ import annotations

import asyncio
import math
from pathlib import Path
import threading
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from pi_arm_interfaces.action import DirectMove, MoveJ, MoveJs, MoveL
from pi_arm_interfaces.msg import HardwareState, RobotState, TaskState
from pi_arm_interfaces.srv import AxisTest, ManageMotor, StopMotion, PreviewMotion, SetTeaching, SetZeroReference
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from .frames import FrameModel
from .points import PointStore
from .presets import PosePresets
from .protocol import CODE_COMMAND_FAILED, CODE_MOTION_BUSY, CODE_STATE_UNAVAILABLE, ProtocolError


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
            "set_teaching": self.create_client(SetTeaching, "/pi_arm/set_teaching"),
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
            self.service_clients["set_zero_reference"] = self.create_client(
                SetZeroReference, "/pi_arm/tasks/set_zero_reference")

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
        # ROS 回调通知完成，等待放到工作线程，避免阻塞 WebSocket 的 asyncio 循环。
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
            # 取消应答仅表示请求被处理；还需等动作终态，才能确认跟踪中的任务已结束。
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
        # 任务是否忙以 TaskState 为准。失败后整机 RUNNING 可能因静止防抖暂时残留，
        # 不能据此一直封锁新请求；后续仍检查整机状态和后端执行条件。
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
