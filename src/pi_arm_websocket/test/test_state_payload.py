import itertools
import json

from pi_arm_interfaces.msg import RobotState
from pi_arm_websocket.gateway import WebSocketGateway


class _Bridge:
    def __init__(self, state):
        self._state = state
        self.hardware_type = "mock"
        self.read_only = False
        self.commissioning = False

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
    state.hardware.calibrated = True
    state.hardware.kinematics_valid = True
    state.hardware.control_permitted = False
    state.hardware.calibration_id = "test-calibration"
    state.hardware.raw_protocol_positions_deg = [1.0] * 6
    state.hardware.raw_output_positions_rad = [0.1] * 6
    state.hardware.software_output_offsets_rad = [0.2] * 6
    state.hardware.model_output_positions_rad = [0.3] * 6
    state.hardware.kinematics_error_code = "NONE"
    state.hardware.kinematics_error_axis = 0
    state.hardware.kinematics_error_value = 0.0
    state.hardware.kinematics_error_message = ""

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
    assert payload["calibrated"] is True
    assert payload["kinematics_valid"] is True
    assert payload["control_permitted"] is False
    assert payload["calibration_id"] == "test-calibration"
    assert payload["raw_protocol_positions_deg"] == [1.0] * 6
    assert payload["model_output_positions_rad"] == [0.3] * 6
    assert payload["kinematics_error"]["code"] == "NONE"


def test_nonfinite_feedback_is_valid_browser_json():
    state = RobotState()
    state.hardware.positions = [float("nan")]
    state.hardware.motor_ages_sec = [float("inf")]
    gateway = WebSocketGateway(_Bridge(state))
    gateway._pose = lambda: {"x": float("nan")}
    payload = gateway._state_message()
    def reject_constant(value):
        raise AssertionError(value)
    data = json.loads(payload, parse_constant=reject_constant)
    assert data["current_joint_angles"] == [None]
    assert data["motor_ages_sec"] == [None]
    assert data["pose"]["x"] is None


def test_stale_ros_state_is_unavailable():
    import threading
    import time
    from pi_arm_websocket.gateway import RosBridge
    bridge = object.__new__(RosBridge)
    bridge._state_lock = threading.Lock()
    bridge._state = RobotState()
    bridge.state_timeout = 1.0
    bridge._state_received_at = time.monotonic()
    assert bridge.state_snapshot() is bridge._state
    bridge._state_received_at -= 2.0
    assert bridge.state_snapshot() is None


def test_disconnected_payload_does_not_show_cached_pose():
    gateway = WebSocketGateway(_Bridge(None))
    gateway._pose = lambda: {"x": 100.0}
    payload = json.loads(gateway._state_message())
    assert payload["control_permitted"] is False
    assert payload["pose"] == {}
