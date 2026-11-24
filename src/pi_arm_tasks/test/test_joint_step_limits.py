import asyncio
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from ament_index_python.packages import get_package_share_directory
from pi_arm_tasks.execution.engine import ExecutionError
from pi_arm_tasks.execution.node import Adapter
from pi_arm_websocket.frames import FrameModel


@pytest.fixture
def motion():
    names = ["joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"]
    urdf = Path(get_package_share_directory("pi_arm_description")) / "urdf/pi_arm.urdf.xacro"
    model = FrameModel(urdf, names)
    feedback = SimpleNamespace(
        connected=True, control_permitted=True, kinematics_valid=True, joint_names=names, fresh=[True] * 6,
        motor_fresh=[True] * 6, motor_moving=[False] * 6,
        positions=list(map(math.radians, [1, 100.69901042, -85.60071944, 0, 0, 0])),
    )
    state = SimpleNamespace(hardware=feedback, task=SimpleNamespace(status=0))
    bridge = SimpleNamespace(hardware_type="mock", read_only=False, commissioning=False,
                             frame_model=model, state_snapshot=lambda: state, send_goal=AsyncMock(return_value=42))
    return Adapter(bridge), feedback


@pytest.mark.parametrize("command,params,speed", [
    ("step_move", dict(space="joint", axis=1, delta=-1), 2),
    ("motor_move", dict(joint_id=2, target_angle=99.69901042), 5),
])
def test_joint_command_sends_only_selected_axis(motion, command, params, speed):
    adapter, feedback = motion
    resolved = asyncio.run(adapter.resolve(command, params))
    assert resolved["type"] == "motor_move"
    assert resolved["params"] == pytest.approx(dict(joint_id=2, target_angle=99.69901042, max_speed=speed))
    feedback.positions[1] = 0.0  # A later sample cannot change an accepted step.
    task_id = asyncio.run(adapter.move(resolved["type"], resolved["params"], "test-session"))
    channel, goal = adapter.bridge.send_goal.await_args.args
    assert task_id == 42 and channel == "direct"
    assert list(goal.joint_names) == ["joint_02"]
    assert list(goal.positions) == pytest.approx([math.radians(99.69901042)])
    assert goal.max_velocity == pytest.approx(math.radians(speed)) and not goal.output_space
    assert math.degrees(feedback.positions[2]) == pytest.approx(-85.60071944)


@pytest.mark.parametrize("field,value", [("fresh", False), ("motor_moving", True)])
def test_step_requires_fresh_stationary_feedback(motion, field, value):
    adapter, feedback = motion
    getattr(feedback, field)[2] = value
    with pytest.raises(ExecutionError, match="静止"):
        asyncio.run(adapter.resolve("step_move", dict(space="joint", axis=1, delta=-1)))
    adapter.bridge.send_goal.assert_not_awaited()
