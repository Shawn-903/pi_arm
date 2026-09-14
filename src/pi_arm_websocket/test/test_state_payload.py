import itertools
import json

from pi_arm_interfaces.msg import RobotState
from pi_arm_websocket.gateway import WebSocketGateway


class _Bridge:
    def __init__(self, state):
        self._state = state

    def state_snapshot(self):
        return self._state


def test_state_payload_separates_joint_and_motor_enabled_fields():
    state = RobotState()
    state.state = RobotState.DISABLED
    state.state_name = "DISABLED"
    state.hardware.stamp.sec = 10
    state.hardware.joint_names = [
        "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"
    ]
    state.hardware.positions = [0.0] * 6
    state.hardware.velocities = [0.0] * 6
    state.hardware.enabled = [True, False, False, False, False, False]
    state.hardware.errors = [False] * 6
    state.hardware.moving = [False] * 6
    state.hardware.fresh = [True] * 6
    state.hardware.error_codes = [0] * 6
    state.hardware.ages_sec = [0.1] * 6
    state.hardware.motor_ids = [1, 2, 3, 4, 5, 6]
    state.hardware.motor_enabled = [True, False, True, True, True, True]
    state.hardware.motor_errors = [False] * 6
    state.hardware.motor_moving = [False] * 6
    state.hardware.motor_fresh = [True] * 6
    state.hardware.motor_error_codes = [0] * 6
    state.hardware.motor_ages_sec = [0.2] * 6

    gateway = object.__new__(WebSocketGateway)
    gateway.bridge = _Bridge(state)
    gateway.counter = itertools.count(1)
    gateway._pose = lambda: {"x": None, "y": None, "z": None,
                             "roll": None, "pitch": None, "yaw": None}

    payload = json.loads(gateway._state_message())
    assert payload["enabled"] == state.hardware.enabled
    assert payload["joint_enabled"] == state.hardware.enabled
    assert payload["motor_ids"] == [1, 2, 3, 4, 5, 6]
    assert payload["motor_enabled"] == [True, False, True, True, True, True]
    assert payload["motor_fresh"] == [True] * 6
    assert payload["motor_error_codes"] == [0] * 6
    assert payload["motor_ages_sec"] == [0.2] * 6
    assert payload["all_enabled"] is False
