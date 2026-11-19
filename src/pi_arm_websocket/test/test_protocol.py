import asyncio
import json
import logging
from types import SimpleNamespace

import pytest

from pi_arm_interfaces.msg import RobotState
from pi_arm_websocket.gateway import CommandGateway
from pi_arm_websocket.protocol import (
    COMMANDS,
    CODE_INVALID_JSON,
    CODE_INVALID_REQUEST,
    CODE_STATE_UNAVAILABLE,
    CODE_UNSUPPORTED_COMMAND,
    ProtocolError,
    parse_request,
    response,
)


def test_parse_request_normalizes_command_and_defaults_params():
    request = parse_request('{"type":"req","msg_id":7,"cmd":" MoveJ "}')
    assert request.msg_id == 7
    assert request.command == "movej"
    assert request.params == {}


def test_run_query_preserves_snapshot_and_sample_precision():
    gateway = CommandGateway(None)
    data = dict(run=dict(steps=[dict(point_snapshot=dict(values=[1.234567891]*6))]),
                steps=[dict(sample=dict(joint_angles_deg=[2.345678912]*6))])
    async def dispatch(*args):
        return data
    gateway.dispatch = dispatch
    result = json.loads(asyncio.run(gateway.handle('{"cmd":"run_get","params":{"run_id":"r"}}')))
    assert result['code'] == 0 and result['data'] == data


def test_legacy_commands_remain_available():
    legacy = {"enable_motor", "disable_motor", "reset_motor", "reboot_motor", "move",
              "movej", "movejs", "movel", "motor_move", "motor_stop", "stop_motion",
              "set_teaching", "set_zero_reference", "preview_motion", "output_move", "move_preset",
              "axis_test", "axis_test_heartbeat", "axis_test_stop", "point_list", "point_save", "point_delete"}
    assert legacy <= COMMANDS


@pytest.mark.parametrize(
    ("payload", "code"),
    [
        ("not-json", CODE_INVALID_JSON),
        ('{"type":"state","cmd":"movej"}', CODE_INVALID_REQUEST),
        ('{"type":"req","cmd":"unknown"}', CODE_UNSUPPORTED_COMMAND),
    ],
)
def test_parse_request_reports_stable_error_codes(payload, code):
    with pytest.raises(ProtocolError) as caught:
        parse_request(payload)
    assert caught.value.code == code


def test_response_keeps_legacy_ms_field():
    value = json.loads(response(3, 0, "ok", {"accepted": True}))
    assert value == {
        "type": "rsp",
        "msg_id": 3,
        "code": 0,
        "ms": "ok",
        "data": {"accepted": True},
    }

class _BridgeWithoutControlPermission:
    commissioning = False
    read_only = False
    hardware_type = "mock"
    default_joint_names = ["joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"]

    def __init__(self):
        self.state = RobotState()
        self.state.hardware.control_permitted = False

    def state_snapshot(self):
        return self.state


def test_motion_command_is_rejected_without_control_permission():
    gateway = CommandGateway(_BridgeWithoutControlPermission())
    request = parse_request(
        '{"type":"req","msg_id":42,"cmd":"movej","params":{"target":[0,0,0,0,0,0],"vel":1,"acc":1}}'
    )
    with pytest.raises(ProtocolError) as caught:
        asyncio.run(gateway.dispatch(request))
    assert caught.value.code == CODE_STATE_UNAVAILABLE


class _CommandBridge(_BridgeWithoutControlPermission):
    def __init__(self):
        super().__init__()
        self.state.hardware.control_permitted = True
        self.goals = []
        self.services = []

    async def send_goal(self, name, goal):
        self.goals.append((name, goal))
        return 19

    async def call_service(self, name, request):
        self.services.append(name)


def test_single_joint_fractional_speed_and_task_id():
    bridge = _CommandBridge()
    result = asyncio.run(CommandGateway(bridge).dispatch(parse_request(
        '{"cmd":"motor_move","params":{"joint_id":2,"target_angle":1,"max_speed":0.5}}'
    )))
    assert result["task_id"] == 19
    name, goal = bridge.goals[0]
    assert name == "direct"
    assert goal.joint_names == ["joint_02"]
    assert goal.max_velocity == pytest.approx(0.5 * 3.141592653589793 / 180)


def test_multi_joint_direct_move_returns_task_id():
    bridge = _CommandBridge()
    result = asyncio.run(CommandGateway(bridge).dispatch(parse_request(
        '{"cmd":"move","params":{"targets":[0,0,0,0,0,0],"max_speed":1}}'
    )))
    assert result["task_id"] == 19
    assert bridge.goals[0][0] == "direct"


def test_stop_never_sends_a_new_motion_even_without_control_permission():
    bridge = _CommandBridge()
    bridge.state.hardware.control_permitted = False
    asyncio.run(CommandGateway(bridge).dispatch(parse_request(
        '{"cmd":"motor_stop","params":{"joint_id":1}}'
    )))
    assert bridge.services == ["stop_motion"]
    assert bridge.goals == []


def test_unconfirmed_stop_reports_controller_reason_on_the_wire():
    async def execution(command, params, connection):
        assert command == "execution_stop"
        return dict(run_id="run", code=3006, message="保持反馈超时", stop_confirmed=False)

    bridge = SimpleNamespace(read_only=False, commissioning=False, connect_execution=True,
                             get_logger=lambda: logging.getLogger(__name__), execution=execution,
                             service_clients={"execution": SimpleNamespace(service_is_ready=lambda: True)})
    result = json.loads(asyncio.run(CommandGateway(bridge).handle('{"msg_id":7,"cmd":"stop_motion"}')))
    assert result["msg_id"] == 7 and result["code"] == 3006
    assert result["ms"] == "保持反馈超时"
    assert result["data"]["run_id"] == "run" and result["data"]["stop_confirmed"] is False


def test_real_cartesian_request_uses_normal_action_and_output_speed_ceiling():
    bridge = _CommandBridge()
    bridge.hardware_type = "real"
    asyncio.run(CommandGateway(bridge).dispatch(parse_request(json.dumps({
        'cmd':'movel', 'params':{'target':[100,20,300,0,0,0], 'linear_vel':1,
        'linear_acc':1, 'angular_vel':1, 'angular_acc':1, 'output_speed_limit':2}}))))
    name, goal = bridge.goals[0]
    assert name == 'movel' and goal.target.header.frame_id == 'base_link'
    assert goal.target.pose.position.x == pytest.approx(.1)
    assert goal.max_output_velocity == pytest.approx(2*3.141592653589793/180)


def test_output_targets_keep_model_output_coordinates_until_manager_inverse():
    bridge = _CommandBridge()
    result = asyncio.run(CommandGateway(bridge).dispatch(parse_request(json.dumps({
        'cmd':'output_move', 'params':{'targets':[1,-2,3,-4,5,-6], 'output_speed_limit':3}}))))
    name, goal = bridge.goals[0]
    assert name == 'direct' and goal.output_space
    assert list(goal.positions) == pytest.approx([v*3.141592653589793/180 for v in [1,-2,3,-4,5,-6]])
    assert goal.max_output_velocity == pytest.approx(3*3.141592653589793/180)
    assert result['output_speed_limit'] == pytest.approx(3)


@pytest.mark.parametrize('speed', [0, -.1, .1, 10.001, float('inf'), float('nan')])
def test_invalid_output_speed_never_submits_goal(speed):
    bridge = _CommandBridge()
    with pytest.raises(ProtocolError):
        asyncio.run(CommandGateway(bridge).dispatch(parse_request(json.dumps({
            'cmd':'move', 'params':{'targets':[0]*6, 'max_speed':1, 'output_speed_limit':speed}}))))
    assert not bridge.goals


def test_commissioning_never_uses_normal_motion_actions():
    bridge = _CommandBridge()
    bridge.commissioning = True
    with pytest.raises(ProtocolError, match="方向测试模式"):
        asyncio.run(CommandGateway(bridge).dispatch(parse_request(
            '{"cmd":"motor_move","params":{"joint_id":1,"target_angle":1,"max_speed":1}}'
        )))
    assert not bridge.goals


def test_axis_test_requires_dedicated_commissioning_mode():
    with pytest.raises(ProtocolError, match="commissioning"):
        asyncio.run(CommandGateway(_CommandBridge()).dispatch(parse_request(
            '{"cmd":"axis_test","params":{"space":"motor","axis_id":1,"delta_deg":0.1,"speed_deg_s":0.2}}'
        )))


@pytest.mark.parametrize('speed', [None, 10.0, 10.001])
@pytest.mark.parametrize('command,params,key,attribute', [
    ('motor_move', {'joint_id':1,'target_angle':1}, 'max_speed', 'max_velocity'),
    ('move', {'targets':[0]*6}, 'max_speed', 'max_velocity'),
    ('movej', {'target':[0]*6,'acc':1}, 'vel', 'max_velocity'),
    ('movel', {'target':[0]*6,'linear_vel':5,'linear_acc':10,'angular_acc':2}, 'angular_vel', 'max_angular_velocity'),
    ('output_move', {'targets':[0]*6}, 'output_speed_limit', 'max_output_velocity'),
])
def test_angular_speed_defaults_and_boundary(command, params, key, attribute, speed):
    import math
    bridge = _CommandBridge()
    params = dict(params)
    if speed is not None: params[key] = speed
    request = parse_request(json.dumps({'cmd':command,'params':params}))
    if speed is not None and speed > 10:
        with pytest.raises(ProtocolError): asyncio.run(CommandGateway(bridge).dispatch(request))
        assert not bridge.goals
    else:
        asyncio.run(CommandGateway(bridge).dispatch(request))
        assert getattr(bridge.goals[0][1],attribute) == pytest.approx(math.radians(5 if speed is None else speed))
