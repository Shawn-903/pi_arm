"""通过实际 ROS Action/Service 验证采样编排，使用合成 manager 状态。"""

from copy import deepcopy
import json
import threading
import time

import pytest
import rclpy
from pi_arm_interfaces.action import DirectMove
from pi_arm_interfaces.msg import HardwareState, RobotState, TaskState
from pi_arm_interfaces.srv import StopMotion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from pi_arm_tasks.calibration_motion import CalibrationMotionError, CalibrationMotionTask, CalibrationPose
from pi_arm_tasks.calibration_motion.plan import JOINT_NAMES


class ManagerFixture(Node):
    def __init__(self, context):
        super().__init__("calibration_manager_fixture", context=context)
        self.mode = "success"
        self.accept_delay = 0.0
        self.stop_success = True
        self.publish_states = True
        self.stops = self.cancels = 0
        self.requests, self.arrivals = [], []
        self.active = False
        self.motion_stop = threading.Event()
        self.arrived = threading.Event()
        self.lock = threading.Lock()
        hardware = HardwareState(
            connected=True, joint_names=list(JOINT_NAMES), positions=[0.0] * 6,
            velocities=[0.0] * 6, enabled=[True] * 6, errors=[False] * 6,
            fresh=[True] * 6, moving=[False] * 6, motor_ids=list(range(1, 7)),
            motor_enabled=[True] * 6, motor_errors=[False] * 6, motor_fresh=[True] * 6,
            motor_moving=[False] * 6, calibrated=True, kinematics_valid=True, control_permitted=True,
            calibration_id="synthetic-calibration", software_output_offsets_rad=[0.0]*6,
            raw_protocol_positions_deg=[10.0] * 6, model_output_positions_rad=[0.0] * 6,
        )
        self.state = RobotState(state=RobotState.READY, state_name="READY", hardware=hardware)
        group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(RobotState, "/pi_arm/state", 10)
        self.create_timer(0.02, self.publish)
        self.create_service(StopMotion, "/pi_arm/stop_motion", self.stop, callback_group=group)
        self.server = ActionServer(
            self, DirectMove, "/pi_arm/direct_move", self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel, callback_group=group,
        )

    def publish(self):
        if self.publish_states:
            with self.lock:
                message = deepcopy(self.state)
            message.hardware.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)

    def accept(self, request):
        self.requests.append((time.monotonic(), request))
        time.sleep(self.accept_delay)
        return GoalResponse.REJECT if self.mode == "reject" else GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def stop(self, request, response):
        self.stops += 1
        if self.active:
            self.motion_stop.set()
        response.success = self.stop_success
        response.message = "held" if self.stop_success else "hold unconfirmed"
        return response

    def execute(self, handle):
        self.motion_stop.clear()
        self.active = True
        with self.lock:
            self.state.task.task_id += 1
            task_id = self.state.task.task_id
            self.state.task.status = TaskState.RUNNING
            self.state.task.arrival_state = "tracking"
            self.state.state, self.state.state_name = RobotState.RUNNING, "RUNNING"
            self.state.hardware.moving = [True] * 6
        handle.publish_feedback(DirectMove.Feedback(task_id=task_id, progress=0.5))
        if self.mode == "stale":
            self.publish_states = False
        deadline = time.monotonic() + (3.0 if self.mode in ("hang", "stale") else 0.1)
        while time.monotonic() < deadline:
            if handle.is_cancel_requested or self.motion_stop.is_set():
                with self.lock:
                    self.state.task.status = TaskState.CANCELED
                    self.state.hardware.moving = [False] * 6
                    self.state.state, self.state.state_name = RobotState.READY, "READY"
                self.active = False
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                return DirectMove.Result(success=False, code=3001, message="stopped")
            time.sleep(0.005)
        self.active = False
        with self.lock:
            self.state.hardware.moving = [False] * 6
            self.state.state, self.state.state_name = RobotState.READY, "READY"
            if self.mode == "failure":
                self.state.task.status = TaskState.FAILED
                handle.abort()
                return DirectMove.Result(success=False, code=3002, message="synthetic failure")
            self.state.hardware.positions = list(handle.request.positions)
            self.state.task.status = TaskState.SUCCEEDED
            self.state.task.arrival_state = "reached"
        self.arrivals.append(time.monotonic())
        self.arrived.set()
        handle.succeed()
        return DirectMove.Result(success=True, code=0, message="reached")


@pytest.fixture
def rig():
    context = Context()
    rclpy.init(context=context, domain_id=109, signal_handler_options=SignalHandlerOptions.NO)
    manager = ManagerFixture(context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(manager)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    task = CalibrationMotionTask(context=context, state_timeout_sec=0.3, service_timeout_sec=1.0)
    try:
        yield manager, task
    finally:
        task.close()
        manager.motion_stop.set()
        executor.shutdown()
        thread.join(timeout=5)
        manager.server.destroy()
        manager.destroy_node()
        rclpy.shutdown(context=context)


def poses(dwell=0.0):
    return [CalibrationPose("P1", [0.01] + [0.0] * 5, dwell),
            CalibrationPose("P2", [0.02] + [0.0] * 5, 0.0)]


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_sequential_arrival_dwell_and_timestamped_feedback(rig, tmp_path):
    manager, task = rig
    output = tmp_path / "samples.jsonl"
    assert task.run(poses(0.15), output) == 2
    samples = [record for record in records(output) if record["type"] == "sample"]
    assert [sample["pose_name"] for sample in samples] == ["P1", "P2"]
    assert [sample["task_id"] for sample in samples] == [1, 2]
    assert samples[1]["feedback_joint_positions_rad"][0] == pytest.approx(0.02)
    assert samples[1]["calibration_id"] == "synthetic-calibration"
    assert "software_zero_id" not in samples[1]
    assert samples[1]["ros_stamp"]["sec"] > 0
    assert manager.requests[1][0] - manager.arrivals[0] >= 0.15
    assert manager.stops == 0
    assert records(output)[-1] == {"type": "result", "success": True, "samples": 2}


def test_rejected_goal_does_not_stop_another_task(rig, tmp_path):
    manager, task = rig
    manager.mode = "reject"
    with pytest.raises(CalibrationMotionError, match="rejected"):
        task.run(poses(), tmp_path / "rejected.jsonl")
    assert len(manager.requests) == 1 and manager.stops == 0


@pytest.mark.parametrize("mode", ["failure", "stale"])
def test_failed_or_stale_motion_stops_without_sending_next_pose(rig, tmp_path, mode):
    manager, task = rig
    manager.mode = mode
    path = tmp_path / "failed.jsonl"
    with pytest.raises(CalibrationMotionError) as failure:
        task.run(poses(), path)
    assert failure.value.stop_confirmed is True
    assert len(manager.requests) == 1 and manager.stops == 1
    assert records(path)[-1]["samples"] == 0
    assert not manager.active


@pytest.mark.parametrize("during_dwell", [False, True])
def test_interruption_stops_during_motion_or_dwell(rig, tmp_path, during_dwell):
    manager, task = rig
    manager.mode = "success" if during_dwell else "hang"

    def interrupt():
        if during_dwell:
            assert manager.arrived.wait(3.0)
        else:
            deadline = time.monotonic() + 3.0
            while not manager.active and time.monotonic() < deadline:
                time.sleep(0.01)
        time.sleep(0.08)
        task.request_stop()

    worker = threading.Thread(target=interrupt)
    worker.start()
    path = tmp_path / "interrupted.jsonl"
    try:
        with pytest.raises(CalibrationMotionError, match="interrupted") as failure:
            task.run(poses(2.0), path)
    finally:
        worker.join(timeout=5)
    assert failure.value.stop_confirmed is True
    assert len(manager.requests) == 1 and manager.stops == 1
    assert records(path)[-1]["samples"] == 0


def test_late_accepted_goal_is_canceled_during_cleanup(rig, tmp_path):
    manager, task = rig
    manager.mode = "hang"
    manager.accept_delay = 1.25
    with pytest.raises(CalibrationMotionError, match="acceptance") as failure:
        task.run(poses(), tmp_path / "late.jsonl")
    assert failure.value.stop_confirmed is True
    assert manager.cancels == 1 and not manager.active
    assert len(manager.requests) == 1


def test_stop_failure_is_reported_and_blocks_reuse(rig, tmp_path):
    manager, task = rig
    manager.mode = "failure"
    manager.stop_success = False
    with pytest.raises(CalibrationMotionError) as failure:
        task.run(poses(), tmp_path / "unconfirmed.jsonl")
    assert failure.value.stop_confirmed is False
    with pytest.raises(CalibrationMotionError, match="previous stop"):
        task.run(poses(), tmp_path / "another.jsonl")
    assert len(manager.requests) == 1


def test_reference_change_during_dwell_aborts_sampling(rig, tmp_path):
    manager, task = rig

    def change_reference():
        assert manager.arrived.wait(3.0)
        time.sleep(0.08)
        with manager.lock:
            manager.state.hardware.software_output_offsets_rad = [0.01] * 6

    worker = threading.Thread(target=change_reference)
    worker.start()
    try:
        with pytest.raises(CalibrationMotionError, match="reference changed"):
            task.run(poses(2.0), tmp_path / "changed.jsonl")
    finally:
        worker.join(timeout=5)
    assert len(manager.requests) == 1 and manager.stops == 1


def test_existing_output_is_preserved_before_any_motion(rig, tmp_path):
    manager, task = rig
    path = tmp_path / "existing.jsonl"
    path.write_text("previous measurements\n")
    with pytest.raises(FileExistsError):
        task.run(poses(), path)
    assert path.read_text() == "previous measurements\n"
    assert not manager.requests


def test_stop_requested_before_run_prevents_motion(rig, tmp_path):
    manager, task = rig
    task.request_stop()
    with pytest.raises(CalibrationMotionError, match="interrupted"):
        task.run(poses(), tmp_path / "pre-stopped.jsonl")
    assert not manager.requests and manager.stops == 0
