"""WebSocket connection/state transport and process entry point.

The original class and rotation-helper imports remain available here for callers.
"""
from __future__ import annotations

import asyncio
from itertools import count
import json
import math
import threading
from typing import Any

import rclpy
from pi_arm_interfaces.srv import AxisTest
from tf2_ros import TransformException
import websockets
from websockets.exceptions import ConnectionClosed

from .commands import CommandGateway
from .presets import PosePresets
from .protocol import point_precision
from .ros_bridge import RosBridge, monitor_state
from .rotations import DEG_TO_RAD, RAD_TO_DEG, quaternion_from_rpy, rpy_from_quaternion


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
        body["teaching_active"] = bool(hardware and hardware.teaching_active)
        body["teaching_available"] = not self.bridge.read_only and not self.bridge.commissioning
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


def json_safe(value: Any) -> Any:
    """ROS unavailable samples may contain NaN/Infinity, which browsers reject."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


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
