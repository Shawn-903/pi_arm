"""逐点执行关节姿态、等待到位和停留，将同帧反馈写入 JSONL。"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from pi_arm_interfaces.action import DirectMove
from pi_arm_interfaces.msg import RobotState, TaskState
from pi_arm_interfaces.srv import StopMotion
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .plan import JOINT_NAMES, finite_number, validate_poses


class CalibrationMotionError(RuntimeError):
    def __init__(self, message, *, stop_confirmed=None):
        super().__init__(message)
        self.stop_confirmed = stop_confirmed


class CalibrationMotionTask(Node):
    """在调用线程中处理 ROS 消息；其他线程可通过 request_stop() 请求结束。"""

    def __init__(self, *, state_timeout_sec=1.0, service_timeout_sec=5.0, **node_kwargs):
        self.state_timeout_sec = finite_number(state_timeout_sec, "state_timeout_sec", positive=True)
        self.service_timeout_sec = finite_number(service_timeout_sec, "service_timeout_sec", positive=True)
        super().__init__("pi_arm_calibration_motion", **node_kwargs)
        self._state = None
        self._state_received = 0.0
        self._reference = None
        self._stop_requested = threading.Event()
        self._run_lock = threading.Lock()
        self._acceptance = self._goal = self._result = None
        self._attempted = False
        self._stop_unconfirmed = False
        self._task_id = 0
        self.create_subscription(RobotState, "/pi_arm/state", self._on_state, 10)
        self._move = ActionClient(self, DirectMove, "/pi_arm/direct_move")
        self._stop = self.create_client(StopMotion, "/pi_arm/stop_motion")
        self._task_executor = SingleThreadedExecutor(context=self.context)
        self._task_executor.add_node(self)

    def _on_state(self, state):
        self._state = state
        self._state_received = time.monotonic()

    def request_stop(self):
        self._stop_requested.set()

    def _check_state(self):
        if self._state is None or time.monotonic() - self._state_received > self.state_timeout_sec:
            raise CalibrationMotionError("robot state is unavailable or stale")
        state = self._state
        if state.state not in (RobotState.READY, RobotState.RUNNING):
            raise CalibrationMotionError(f"robot is not ready: {state.state_name}")
        hardware = state.hardware
        if hardware.commissioning or hardware.teaching_active or not hardware.control_permitted:
            raise CalibrationMotionError("normal control permission is required")
        if self._reference is not None and self._reference != (hardware.calibration_id, hardware.software_zero_id):
            raise CalibrationMotionError("calibration or software-zero reference changed during sampling")

    def _wait(self, condition, timeout, description, *, monitor=True):
        deadline = time.monotonic() + timeout
        while self.context.ok():
            if self._stop_requested.is_set():
                raise CalibrationMotionError("sampling interrupted")
            if monitor:
                self._check_state()
            if condition():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CalibrationMotionError(f"timed out waiting for {description}")
            self._task_executor.spin_once(timeout_sec=min(0.05, remaining))
        raise CalibrationMotionError("ROS context shut down during sampling")

    def _feedback(self, message):
        self._task_id = int(message.feedback.task_id)

    def _move_to(self, pose, joint_speed, output_speed, timeout):
        self._check_state()
        if self._state.state != RobotState.READY or self._state.task.status == TaskState.RUNNING:
            raise CalibrationMotionError("wait for the current task to finish before sampling")
        self._task_id = 0
        self._goal = self._result = None
        goal = DirectMove.Goal(
            joint_names=list(JOINT_NAMES), positions=list(pose.joint_positions_rad),
            max_velocity=joint_speed, max_output_velocity=output_speed, output_space=False,
        )
        self._acceptance = self._move.send_goal_async(goal, feedback_callback=self._feedback)
        self._attempted = True
        started = time.monotonic()
        self._wait(self._acceptance.done, self.service_timeout_sec, "goal acceptance")
        self._goal = self._acceptance.result()
        if not self._goal.accepted:
            self._attempted = False
            raise CalibrationMotionError(f"pose {pose.name} was rejected by manager")
        self._result = self._goal.get_result_async()
        self._wait(self._result.done, max(0.0, timeout - (time.monotonic() - started)), "motion result")
        result = self._result.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise CalibrationMotionError(f"pose {pose.name} failed: {result.result.message}")
        self._wait(self._arrived, self.service_timeout_sec, "verified arrival state")

    def _arrived(self):
        state = self._state
        return (self._task_id != 0 and state.task.task_id == self._task_id
                and state.task.status == TaskState.SUCCEEDED and state.task.arrival_state == "reached"
                and state.state == RobotState.READY
                and not any(state.hardware.moving) and not any(state.hardware.motor_moving))

    def _dwell(self, seconds):
        deadline = time.monotonic() + seconds

        def complete():
            if not self._arrived():
                raise CalibrationMotionError("pose changed or another task started during dwell")
            return time.monotonic() >= deadline

        self._wait(complete, seconds + self.service_timeout_sec, "stationary dwell")

    def _stop_current(self):
        """请求现有停止服务，并处理接受响应晚到的动作，等待动作终态。"""
        deadline = time.monotonic() + self.service_timeout_sec
        stop_future = cancel_future = None
        try:
            while self.context.ok() and time.monotonic() < deadline:
                if stop_future is None and self._stop.service_is_ready():
                    stop_future = self._stop.call_async(StopMotion.Request())
                if self._goal is None and self._acceptance is not None and self._acceptance.done():
                    self._goal = self._acceptance.result()
                if self._goal is not None and self._goal.accepted:
                    if self._result is None:
                        self._result = self._goal.get_result_async()
                    if not self._result.done() and cancel_future is None:
                        cancel_future = self._goal.cancel_goal_async()
                settled = self._goal is not None and not self._goal.accepted
                if self._result is not None and self._result.done():
                    settled = self._result.result().status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                    )
                if stop_future is not None and stop_future.done() and settled:
                    return bool(stop_future.result().success)
                self._task_executor.spin_once(timeout_sec=0.05)
        except Exception as error:
            self.get_logger().error(f"sampling stop failed: {error}")
        return False

    @staticmethod
    def _write(stream, record):
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()

    def _sample(self, pose, index, started):
        hardware = self._state.hardware
        joint_values = dict(zip(hardware.joint_names, hardware.positions, strict=True))
        feedback = [joint_values[name] for name in JOINT_NAMES]
        raw = list(hardware.raw_protocol_positions_deg)
        outputs = list(hardware.model_output_positions_rad)
        if (len(raw) != 6 or len(outputs) != 6 or sorted(hardware.motor_ids) != list(range(1, 7))
                or not all(math.isfinite(value) for value in feedback + raw + outputs)):
            raise CalibrationMotionError("sampling feedback is incomplete or non-finite")
        return {
            "type": "sample", "sample_index": index, "pose_name": pose.name,
            "task_id": self._task_id, "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_sec": time.monotonic() - started, "dwell_sec": pose.dwell_sec,
            "ros_stamp": {"sec": hardware.stamp.sec, "nanosec": hardware.stamp.nanosec},
            "joint_names": list(JOINT_NAMES), "target_joint_positions_rad": list(pose.joint_positions_rad),
            "feedback_joint_positions_rad": feedback, "motor_ids": list(hardware.motor_ids),
            "raw_protocol_positions_deg": raw, "model_output_positions_rad": outputs,
            "calibration_id": hardware.calibration_id, "software_zero_id": hardware.software_zero_id,
        }

    def run(self, poses, output_path, *, joint_speed_rad_s=math.radians(5),
            output_speed_rad_s=math.radians(5), motion_timeout_sec=180.0):
        """运行一组姿态，返回保存的采样点数；失败时抛出 CalibrationMotionError。"""
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("this calibration task is already running")
        try:
            if self._stop_unconfirmed:
                raise CalibrationMotionError("previous stop was not confirmed", stop_confirmed=False)
            poses = validate_poses(poses)
            joint_speed = finite_number(joint_speed_rad_s, "joint_speed_rad_s", positive=True)
            output_speed = finite_number(output_speed_rad_s, "output_speed_rad_s", positive=True)
            timeout = finite_number(motion_timeout_sec, "motion_timeout_sec", positive=True)
            path = Path(output_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._attempted = False
            self._reference = None
            started = time.monotonic()
            count = 0
            with path.open("x", encoding="utf-8") as stream:
                self._write(stream, {"type": "run", "schema_version": 1,
                                    "started_at_utc": datetime.now(timezone.utc).isoformat(),
                                    "joint_speed_rad_s": joint_speed, "output_speed_rad_s": output_speed,
                                    "motion_timeout_sec": timeout, "pose_count": len(poses),
                                    "joint_names": list(JOINT_NAMES),
                                    "poses": [{"name": pose.name, "joint_positions_rad": list(pose.joint_positions_rad),
                                               "dwell_sec": pose.dwell_sec} for pose in poses]})
                try:
                    self._wait(lambda: self._state is not None and
                               time.monotonic() - self._state_received <= self.state_timeout_sec,
                               self.service_timeout_sec, "fresh robot state", monitor=False)
                    self._check_state()
                    self._reference = (self._state.hardware.calibration_id, self._state.hardware.software_zero_id)
                    self._wait(lambda: self._move.server_is_ready() and self._stop.service_is_ready(),
                               self.service_timeout_sec, "manager interfaces")
                    for index, pose in enumerate(poses, 1):
                        self._move_to(pose, joint_speed, output_speed, timeout)
                        self._dwell(pose.dwell_sec)
                        self._write(stream, self._sample(pose, index, started))
                        count += 1
                    self._write(stream, {"type": "result", "success": True, "samples": count})
                    return count
                except (Exception, KeyboardInterrupt) as error:
                    confirmed = self._stop_current() if self._attempted else None
                    self._stop_unconfirmed = confirmed is False
                    message = str(error) or "sampling interrupted"
                    if confirmed is False:
                        message += "; stop was not confirmed"
                    try:
                        self._write(stream, {"type": "result", "success": False, "samples": count,
                                            "message": message, "stop_confirmed": confirmed})
                    except OSError as log_error:
                        message += f"; could not save final record: {log_error}"
                    raise CalibrationMotionError(message, stop_confirmed=confirmed) from error
        finally:
            self._run_lock.release()

    def close(self):
        self._task_executor.shutdown()
        self._task_executor.remove_node(self)
        self._move.destroy()
        self.destroy_node()
