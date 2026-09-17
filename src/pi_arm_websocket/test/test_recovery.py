import asyncio

import pytest
from pi_arm_interfaces.msg import RobotState
from pi_arm_interfaces.srv import ManageMotor
from pi_arm_websocket.gateway import CommandGateway
from pi_arm_websocket.protocol import ProtocolError, Request


class Bridge:
    read_only = False
    commissioning = False

    def __init__(self):
        self.calls = []

    def state_snapshot(self):
        state = RobotState()
        state.state = RobotState.FAULT
        state.hardware.kinematics_valid = False
        state.hardware.control_permitted = False
        return state

    async def call_service(self, command, request):
        self.calls.append((command, request.motor_id))
        return ManageMotor.Response(success=True, message='fresh disabled feedback confirmed')

    async def send_goal(self, *args):
        raise AssertionError('recovery must never submit a motion goal')


@pytest.mark.parametrize('commissioning', [False, True])
def test_fault_recovery_uses_management_service_without_motion_permission(commissioning):
    bridge = Bridge()
    bridge.commissioning = commissioning
    result = asyncio.run(CommandGateway(bridge).dispatch(Request(1, 'reboot_motor', {'motor_id': 2})))
    assert bridge.calls == [('reboot_motor', 2)]
    assert result['motor_ids'] == [2]
    assert result['message'] == 'fresh disabled feedback confirmed'


@pytest.mark.parametrize('motor_id', [0, -1, 1.5, True, None])
def test_reboot_rejects_all_motors_and_invalid_ids(motor_id):
    bridge = Bridge()
    with pytest.raises(ProtocolError):
        asyncio.run(CommandGateway(bridge).dispatch(Request(1, 'reboot_motor', {'motor_id': motor_id})))
    assert bridge.calls == []
