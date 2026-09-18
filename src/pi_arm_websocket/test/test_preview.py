import asyncio
import math

import pytest
from geometry_msgs.msg import Point
from pi_arm_interfaces.srv import PreviewMotion
from trajectory_msgs.msg import JointTrajectoryPoint
from pi_arm_websocket.gateway import CommandGateway
from pi_arm_websocket.presets import JOINT_NAMES
from pi_arm_websocket.protocol import ProtocolError, Request


class Bridge:
    read_only = False
    commissioning = False

    def __init__(self):
        self.requests = []

    async def call_service(self, command, request):
        assert command == 'preview_motion'
        self.requests.append(request)
        result = PreviewMotion.Response(success=True, message='preview')
        result.joint_names = JOINT_NAMES
        result.start_positions = [0.] * 6
        result.target_positions = [math.pi / 6] * 6
        result.output_positions = [math.pi / 3] * 6
        result.target_pose.position.z = 1.
        result.target_pose.orientation.w = 1.
        p = JointTrajectoryPoint(positions=[math.pi / 6] * 6)
        p.time_from_start.sec = 2
        result.trajectory.points = [p]
        result.tcp_path = [Point(x=.1, y=.2, z=1.)]
        return result

    async def send_goal(self, *args):
        raise AssertionError('preview must never execute a goal')


def dispatch(bridge, **params):
    return asyncio.run(CommandGateway(bridge).dispatch(Request(1, 'preview_motion', params)))


@pytest.mark.parametrize('mode', ['joint', 'output'])
def test_angle_preview_units_and_no_action(mode):
    bridge = Bridge()
    data = dispatch(bridge, mode=mode, targets=[30]*6, max_speed=2, output_speed_limit=1)
    req = bridge.requests[0]
    assert list(req.positions) == pytest.approx([math.pi / 6]*6)
    assert req.max_velocity == pytest.approx(math.radians(10 if mode == 'output' else 2))
    assert req.max_output_velocity == pytest.approx(math.radians(1))
    assert data['target_joint_angles_deg'] == pytest.approx([30]*6)
    assert data['target_output_angles_deg'] == pytest.approx([60]*6)
    assert data['points'][0]['t'] == 2
    assert data['points'][0]['tcp'] == [.1, .2, 1.]
    assert data['target_pose'] == [0., 0., 1000., 0., 0., 0.]


def test_lin_units_and_quaternion():
    bridge = Bridge()
    dispatch(bridge, mode='lin', target=[100,-200,950,45,45,-45],
             linear_vel=10, linear_acc=2, angular_vel=1)
    req = bridge.requests[0]
    assert req.target.position.x == .1 and req.target.position.y == -.2
    assert req.target.position.z == .95
    assert req.max_linear_velocity == .01 and req.max_linear_acceleration == .002
    assert req.max_angular_velocity == pytest.approx(math.radians(1))
    q = req.target.orientation
    assert q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == pytest.approx(1)


@pytest.mark.parametrize('params', [dict(mode='bad'), dict(mode='output', targets=[0]*5),
    dict(mode='joint',targets=[0]*6,max_speed=0), dict(mode='ik',target=[float('nan')]*6),
    dict(mode='output',targets=[0]*6,output_speed_limit=.1)])
def test_invalid_preview_is_rejected_before_service(params):
    bridge = Bridge()
    with pytest.raises(ProtocolError): dispatch(bridge, **params)
    assert bridge.requests == []


def test_commissioning_has_no_preview_services():
    bridge = Bridge()
    bridge.commissioning = True
    with pytest.raises(ProtocolError): dispatch(bridge, mode='ik',target=[0]*6)
    assert bridge.requests == []
