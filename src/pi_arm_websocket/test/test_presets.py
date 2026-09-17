import asyncio
import json
import math

import pytest
import yaml
from pi_arm_interfaces.msg import RobotState
from pi_arm_websocket.gateway import CommandGateway
from pi_arm_websocket.presets import JOINT_NAMES, PosePresets
from pi_arm_websocket.protocol import ProtocolError, parse_request


class Bridge:
    read_only = False
    commissioning = False
    hardware_type = "real"
    default_joint_names = JOINT_NAMES

    def __init__(self, presets):
        self.presets = presets
        self.state = RobotState()
        self.state.state = RobotState.READY
        self.state.state_name = "READY"
        h = self.state.hardware
        h.joint_names = JOINT_NAMES
        h.calibrated = h.kinematics_valid = h.control_permitted = True
        h.calibration_id = "TEST-PACK"
        h.motor_fresh = h.motor_enabled = [True]*6
        h.motor_error_codes = [0]*6
        self.goals = []

    def state_snapshot(self):
        return self.state

    async def send_goal(self, action, goal):
        self.goals.append((action, goal))
        return 42


@pytest.fixture
def presets(tmp_path):
    path = tmp_path/'packing.yaml'
    path.write_text(yaml.safe_dump({'calibration': {
        'joint_names': JOINT_NAMES, 'calibration_id': 'TEST-PACK', 'approved': True,
        'packing_pose_joint_rad': [0, .15, -.3, 0, 0, .05],
    }}))
    return PosePresets(str(path))


def dispatch(bridge, preset, **extra):
    return asyncio.run(CommandGateway(bridge).dispatch(parse_request(json.dumps({
        'cmd': 'move_preset', 'params': {'preset': preset, **extra}}))))


@pytest.mark.parametrize('preset', ['packing', 'zero'])
@pytest.mark.parametrize('configured_speed', [1.0, 3.0])
def test_presets_use_backend_joint_targets_and_slow_speed(presets, preset, configured_speed, monkeypatch):
    # Check the configured speed and units, independently of an operator's local setting.
    monkeypatch.setattr('pi_arm_websocket.presets.PRESET_JOINT_SPEED_DEG_S', configured_speed)
    monkeypatch.setattr('pi_arm_websocket.gateway.PRESET_JOINT_SPEED_DEG_S', configured_speed)
    bridge = Bridge(presets)
    result = dispatch(bridge, preset)
    assert result['task_id'] == 42
    action, goal = bridge.goals[0]
    assert action == 'direct' and goal.joint_names == JOINT_NAMES
    assert list(goal.positions) == pytest.approx([0, .15, -.3, 0, 0, .05] if preset == 'packing' else [0]*6)
    assert goal.max_velocity == pytest.approx(math.radians(configured_speed))
    assert result['joint_speed_deg_s'] == configured_speed
    assert presets.status(bridge, bridge.state)[preset]['joint_speed_deg_s'] == configured_speed


@pytest.mark.parametrize('condition', ['readonly', 'commissioning', 'unapproved', 'invalid_kinematics',
                                      'no_permission', 'stale', 'disabled', 'fault', 'stop_failed',
                                      'stop_pending', 'busy', 'wrong_order', 'no_state'])
def test_pose_gates_reject_before_action_submission(presets, condition):
    b = Bridge(presets)
    h = b.state.hardware
    if condition == 'readonly': b.read_only = True
    if condition == 'commissioning': b.commissioning = True
    if condition == 'unapproved': h.calibrated = False
    if condition == 'invalid_kinematics': h.kinematics_valid = False
    if condition == 'no_permission': h.control_permitted = False
    if condition == 'stale': h.motor_fresh = [True]*5+[False]
    if condition == 'disabled': h.motor_enabled = [True]*5+[False]
    if condition == 'fault': h.motor_error_codes = [1]+[0]*5
    if condition == 'stop_failed': h.motion_stop_failed = True
    if condition == 'stop_pending': h.motion_stop_pending = True
    if condition == 'busy': b.state.task.status = 1
    if condition == 'wrong_order': h.joint_names = list(reversed(JOINT_NAMES))
    if condition == 'no_state': b.state = None
    for preset in ('packing', 'zero'):
        with pytest.raises(ProtocolError): dispatch(b, preset)
    assert not b.goals


def test_packing_calibration_must_match_live_hardware(presets):
    b = Bridge(presets)
    b.state.hardware.calibration_id = 'DIFFERENT'
    with pytest.raises(ProtocolError, match='标定 ID'): dispatch(b, 'packing')
    assert not b.goals


def test_unknown_pose_or_client_target_override_rejected(presets):
    b = Bridge(presets)
    with pytest.raises(ProtocolError): dispatch(b, 'unknown')
    with pytest.raises(ProtocolError): dispatch(b, 'packing', targets=[0]*6)
    with pytest.raises(ProtocolError): dispatch(b, 'zero', max_speed=10)
    assert not b.goals


@pytest.mark.parametrize('data', [None, {}, {'calibration': {}}, {'calibration': {'joint_names': JOINT_NAMES,
                          'packing_pose_joint_rad': [float('nan')]*6}}])
def test_invalid_file_keeps_packing_disabled_without_crashing(tmp_path, data):
    p = tmp_path/'bad.yaml'; p.write_text(yaml.safe_dump(data))
    presets = PosePresets(str(p))
    b = Bridge(presets)
    assert not presets.status(b, b.state)['packing']['available']
    with pytest.raises(ProtocolError): dispatch(b, 'packing')


def test_unapproved_file_keeps_target_visible_but_disabled(tmp_path):
    p = tmp_path/'candidate.yaml'
    p.write_text(yaml.safe_dump({'calibration': {'joint_names': JOINT_NAMES,
        'calibration_id': 'TEST-PACK', 'approved': False, 'packing_pose_joint_rad': [0, .1, -.2, 0, 0, 0]}}))
    presets = PosePresets(str(p)); b = Bridge(presets)
    status = presets.status(b, b.state)['packing']
    assert status['target_joint_angles_deg'][1] == pytest.approx(math.degrees(.1))
    assert not status['available'] and '尚未确认' in status['reason']
