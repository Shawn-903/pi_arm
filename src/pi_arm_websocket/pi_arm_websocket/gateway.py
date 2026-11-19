"""WebSocket 连接、状态推送与进程入口。

命令分发交给 commands，ROS 通信交给 ros_bridge，状态字段交给 state_payload。
此处继续导出原有类和旋转换算函数，保留调用方已有的导入路径。
"""
from __future__ import annotations

import asyncio
from itertools import count
import json
import threading
import time
import uuid
from typing import Any

import rclpy
from pi_arm_interfaces.srv import AxisTest
from tf2_ros import TransformException
import websockets
from websockets.exceptions import ConnectionClosed

from .commands import CommandGateway
from .presets import PACKING_JOINTS_DEG, PosePresets
from .protocol import point_precision
from .state_payload import json_safe, robot_state_payload
from .ros_bridge import RosBridge, monitor_state
from .rotations import DEG_TO_RAD, RAD_TO_DEG, quaternion_from_rpy, rpy_from_quaternion

__all__ = [
    "CommandGateway", "RosBridge", "WebSocketGateway", "monitor_state",
    "DEG_TO_RAD", "RAD_TO_DEG", "quaternion_from_rpy", "rpy_from_quaternion", "main",
]


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
        connection_id = uuid.uuid4().hex
        pending = set()
        async def reply_to(raw):
            nonlocal owned_test
            reply = await self.commands.handle(raw, connection_id)
            data = json.loads(reply)
            if data['code'] == 0 and data['data'].get('cmd') == 'axis_test':
                owned_test = data['data']['axis_test_id']
            await websocket.send(reply)
        def replied(task):
            pending.discard(task)
            if not task.cancelled():
                error = task.exception()
                if error and not isinstance(error, ConnectionClosed):
                    self.bridge.get_logger().warning(f'响应发送失败: {error}')
        try:
            async for message in websocket:
                raw = message if isinstance(message, str) else "{}"
                # 每条响应按 msg_id 关联，停止和心跳无需等普通请求完成。
                if len(pending) >= 32:
                    await websocket.close(code=1013, reason='too many pending requests')
                    break
                task = asyncio.create_task(reply_to(raw))
                pending.add(task)
                task.add_done_callback(replied)
        except ConnectionClosed as exc:
            self.bridge.get_logger().info(
                f"WebSocket 连接关闭: {peer} code={exc.code} reason={exc.reason!r}"
            )
        except Exception as exc:
            self.bridge.get_logger().error(f"WebSocket 客户端异常: {peer} error={exc}")
        finally:
            if getattr(self.bridge, 'connect_execution', False) and not self.bridge.read_only and not self.bridge.commissioning:
                try:
                    await self.bridge.execution('connection_lost', {}, connection_id)
                except Exception as exc:
                    self.bridge.get_logger().warning(f'会话断线通知失败: {exc}')
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)
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
        body = robot_state_payload(state)
        body["pose"] = self._pose() if state is not None and not getattr(self.bridge, 'frame_model', None) else {}
        body["hardware_type"] = self.bridge.hardware_type
        body['execution'] = dict(getattr(self.bridge, 'execution_state', {}),
                                 fresh=time.monotonic()-getattr(self.bridge, 'execution_received_at', 0.) < 1.)
        model = getattr(self.bridge, 'frame_model', None)
        body['model'] = model.metadata() if model else {}
        preview = getattr(self.bridge, "service_clients", {}).get("preview_motion")
        reboot = getattr(self.bridge, "service_clients", {}).get("reboot_motor")
        body["reboot_available"] = bool(reboot and reboot.service_is_ready())
        body["preview_modes"] = ["joint", "output", "ik", "lin"] if preview and preview.service_is_ready() else []
        body["read_only"] = self.bridge.read_only
        body["commissioning"] = self.bridge.commissioning
        if self.bridge.commissioning:
            body["mechanical_zero_joints_deg"] = PACKING_JOINTS_DEG
        hardware = state.hardware if state else None
        body["teaching_available"] = not self.bridge.read_only and not self.bridge.commissioning
        body["preset_poses"] = getattr(self.bridge, "presets", PosePresets()).status(self.bridge, state)
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
            point_precision(json_safe({"type": "state", "msg_id": next(self.counter), **body})),
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
        if rclpy.ok():
            bridge.get_logger().info("WebSocket 网关收到退出信号")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
