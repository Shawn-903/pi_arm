"""程序执行服务的 ROS 适配器。"""

import asyncio
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from pi_arm_interfaces.msg import ExecutionState
from pi_arm_interfaces.srv import ControlSession, ExecutionCommand, StopMotion
from pi_arm_websocket.commands import CommandGateway
from pi_arm_websocket.frames import multiply, quaternion, pose_record
from pi_arm_websocket.protocol import Request
from pi_arm_websocket.ros_bridge import RosBridge
from pi_arm_websocket.state_payload import robot_state_payload, json_safe

from .engine import Engine, ExecutionError
from .plan import validate_motion
from .store import Store, encode


class Adapter:
    def __init__(self, bridge):
        self.bridge = bridge
        self.hardware_type = bridge.hardware_type
        self.commands = CommandGateway(bridge)

    async def lease(self, operation, token, name=""):
        result = await self.bridge.call_service(
            "control_session", ControlSession.Request(operation=operation, token=token, name=name)
        )
        return dict(token=result.token, controller_instance_id=result.controller_instance_id)

    async def check(self, instance, *, stationary=False):
        state = self.bridge.state_snapshot()
        if state is None or state.controller_instance_id != instance:
            raise ExecutionError(3104, "控制器已重启或状态已过期")
        if state.state not in (state.READY, state.RUNNING) or not state.hardware.control_permitted:
            raise ExecutionError(2001, "当前硬件状态不允许运行")
        if (
            len(state.hardware.fresh) != 6
            or not all(state.hardware.fresh)
            or len(state.hardware.motor_fresh) != 6
            or not all(state.hardware.motor_fresh)
        ):
            raise ExecutionError(2001, "六轴反馈已过期")
        if stationary and any(state.hardware.motor_moving):
            raise ExecutionError(2001, "等待期间检测到运动")

    async def stop(self):
        await self.bridge.call_service("stop_motion", StopMotion.Request())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = self.bridge.state_snapshot()
            if state:
                if state.hardware.motion_stop_failed:
                    raise ExecutionError(3006, "当前位置保持失败")
                if state.task.status != 1 and not state.hardware.motion_stop_pending:
                    return
            await asyncio.sleep(0.02)
        raise ExecutionError(3006, "停止后未收到静止终态")

    async def move(self, command, params, token):
        self.bridge.control_token = token
        result = await self.commands.dispatch(Request(0, command, params))
        return result["task_id"]

    async def result(self, task_id, instance):
        future = self.bridge.task_results[task_id]
        while not future.done():
            await self.check(instance)
            await asyncio.sleep(0.02)
        response = future.result()
        result = response.result
        deadline = time.monotonic() + 2.0
        while True:
            state = self.bridge.state_snapshot()
            if state and state.controller_instance_id != instance:
                raise ExecutionError(3104, "控制器终态无法核对")
            if state and state.task.task_id == task_id and state.task.status in (2, 3, 4):
                break
            if time.monotonic() >= deadline:
                raise ExecutionError(3104, "未收到对应任务的最终反馈")
            await asyncio.sleep(0.02)
        # Action 的取消可能携带 success=true，只有 SUCCEEDED 状态才允许推进。
        status = (
            "SUCCEEDED"
            if response.status == 4 and result.success
            else "CANCELED"
            if response.status == 5
            else "FAILED"
        )
        sample = robot_state_payload(state)
        return dict(status=status, code=int(result.code), message=result.message, sample=json_safe(sample))

    async def sample(self):
        state = self.bridge.state_snapshot()
        frames = self.bridge.frame_model.feedback(state.hardware if state else None)
        if (
            not frames["valid"]
            or state.task.status != 2
            or state.task.arrival_state != "reached"
            or any(state.hardware.motor_moving)
        ):
            raise ExecutionError(2001, "采样时反馈或到位核验无效")
        payload = robot_state_payload(state)
        target, current = payload["task"]["target_joint_angles_deg"], payload["current_joint_angles"]
        return dict(
            joint_angles_deg=current,
            pose_mm_deg=frames["tool"]["position_mm"] + frames["tool"]["rpy_deg"],
            raw_protocol_positions_deg=payload["raw_protocol_positions_deg"],
            target_joint_angles_deg=target,
            sampled_error_joint_angles_deg=[a - b for a, b in zip(target, current)],
            verification_error_joint_angles_deg=payload["task"]["error_joint_angles_deg"],
            goal_tolerance_deg=payload["task"]["goal_tolerance_deg"],
            arrival_state=state.task.arrival_state,
        )

    def point(self, identifier, catalog):
        matches = [p for p in catalog["points"] if p["id"] == identifier or p["name"] == identifier]
        if len(matches) != 1:
            raise ValueError("点位不存在或名称不唯一")
        point = matches[0]
        state = self.bridge.state_snapshot()
        if (
            point.get("hardware_type") != self.bridge.hardware_type
            or point.get("model_id") != self.bridge.frame_model.model_id
            or point.get("joint_names") != self.bridge.frame_model.names
        ):
            raise ValueError("点位的运行模式或机械臂模型不兼容")
        if state and point.get("calibration_id", "") != state.hardware.calibration_id:
            raise ValueError("点位参考信息已变化，请重新核对并保存")
        return point

    async def freeze(self, steps):
        catalog = await asyncio.to_thread(self.bridge.points.list)
        output = []
        for row in deepcopy(steps):
            if row.get("point_id"):
                point = self.point(row["point_id"], catalog)
                params = dict(row.get("params") or {})
                self.point_target(row["type"], params, point)
                row.update(params=params, point_snapshot=deepcopy(point))
            if row["type"] != "wait":
                validate_motion(row["type"], row["params"])
            output.append(row)
        return output

    @staticmethod
    def point_target(command, params, point):
        expected = "cartesian" if command == "movel" else "joint"
        if point["kind"] != expected:
            raise ValueError("点位类型与运动方式不匹配")
        params["targets" if command == "move" else "target"] = list(point["values"])
        defaults = (
            dict(max_speed=2.0)
            if command == "move"
            else dict(vel=2.0, acc=5.0)
            if command == "movej"
            else dict(linear_vel=10.0, linear_acc=20.0, angular_vel=2.0, angular_acc=5.0)
        )
        for key, value in defaults.items():
            params.setdefault(key, value)

    async def resolve(self, command, params):
        params = deepcopy(params)
        point = None
        params.pop("request_id", None)
        params.pop("session_id", None)
        if command == "wait":
            return dict(type="wait", seconds=params["seconds"])
        if command == "move_preset":
            state = self.bridge.state_snapshot()
            preset = self.bridge.presets.status(self.bridge, state)[params.pop("preset")]
            if not preset["available"]:
                raise ValueError(preset["reason"])
            command = "move"
            params["targets"] = list(preset["target_joint_angles_deg"])
            params.setdefault("max_speed", 5.0)
        if command == "motor_move":
            state = self.bridge.state_snapshot()
            frames = self.bridge.frame_model.feedback(state.hardware if state else None)
            if not frames["valid"]:
                raise ExecutionError(2001, "反馈无效，无法生成关节目标")
            target = [math.degrees(v) for v in state.hardware.positions]
            target[params.pop("joint_id") - 1] = params.pop("target_angle")
            params["targets"] = target
            params.setdefault("max_speed", 5.0)
            command = "move"
        if command == "move_to_point":
            catalog = await asyncio.to_thread(self.bridge.points.list)
            point = self.point(params.pop("point"), catalog)
            command = params.pop("motion")
            self.point_target(command, params, point)
        if command == "step_move":
            state = self.bridge.state_snapshot()
            hardware = state.hardware if state else None
            frames = self.bridge.frame_model.feedback(hardware)
            if not frames["valid"] or state.task.status == 1 or any(hardware.motor_moving):
                raise ExecutionError(2001, "步进需要新鲜、有效且静止的反馈")
            space, axis, delta = params.pop("space"), params.pop("axis"), params.pop("delta")
            if space == "joint":
                target = [math.degrees(v) for v in hardware.positions]
                target[axis] += delta
                command = "move"
                params.update(targets=target)
                params.setdefault("max_speed", 2.0)
            else:
                tool = frames["tool"]
                position, rotation = list(tool["position_mm"]), list(tool["quaternion_xyzw"])
                if space == "position":
                    position[axis] += delta
                else:
                    rpy = [0.0, 0.0, 0.0]
                    rpy[axis] = math.radians(delta)
                    rotation = multiply(quaternion(rpy), rotation)
                pose = pose_record("tool0", [v / 1000 for v in position], rotation)
                command = "movel"
                params.update(target=position + pose["rpy_deg"])
                for key, value in dict(
                    linear_vel=10.0, linear_acc=20.0, angular_vel=2.0, angular_acc=5.0
                ).items():
                    params.setdefault(key, value)
        validate_motion(command, params)
        result = dict(type=command, params=params)
        if point is not None:
            result["point_snapshot"] = deepcopy(point)
        return result


class ExecutionNode(RosBridge):
    def __init__(self, loop):
        super().__init__("pi_arm_execution", connect_execution=False)
        self.declare_parameter("execution_file", "")
        filename = str(self.get_parameter("execution_file").value) or str(
            Path.home() / ".local/share/pi_arm" / f"execution-{self.hardware_type}.sqlite3"
        )
        self.store = Store(filename)
        self.engine = Engine(Adapter(self), self.store)
        self.loop = loop
        self.create_service(
            ExecutionCommand,
            "/pi_arm/execution/command",
            self.handle_command,
            callback_group=ReentrantCallbackGroup(),
        )
        self.publisher = self.create_publisher(ExecutionState, "/pi_arm/execution/state", 10)

    def handle_command(self, request, response):
        try:
            params = json.loads(request.params_json or "{}")
            if not isinstance(params, dict):
                raise ValueError("params 必须是对象")
            result = asyncio.run_coroutine_threadsafe(
                self.engine.command(request.command, params, request.connection_id), self.loop
            ).result(timeout=30.0)
            response.success, response.data_json = True, encode(result)
        except Exception as error:
            response.success = False
            response.code = getattr(error, "code", 1004 if isinstance(error, ValueError) else 2002)
            response.message = str(error)
        return response

    async def run(self):
        while rclpy.ok():
            doc = self.engine.state()
            message = ExecutionState()
            message.stamp = self.get_clock().now().to_msg()
            for key in message.get_fields_and_field_types():
                if key in doc:
                    setattr(message, key, doc[key])
            self.publisher.publish(message)
            await asyncio.sleep(0.1)


async def serve():
    node = ExecutionNode(asyncio.get_running_loop())
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        await node.run()
    finally:
        await node.engine.close()
        executor.shutdown()
        thread.join(timeout=2)
        node.store.close()
        node.destroy_node()


def main():
    rclpy.init()
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
