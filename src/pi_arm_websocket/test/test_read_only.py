import asyncio
import json

import pytest

from pi_arm_interfaces.msg import HardwareState, RobotState
from pi_arm_websocket.gateway import CommandGateway, WebSocketGateway, monitor_state
from pi_arm_websocket.protocol import COMMANDS, ProtocolError, Request


def fault_hardware():
    hardware = HardwareState()
    hardware.connected = True
    hardware.driver_mode = "MONITORING"
    hardware.motor_ids = [1, 2, 3, 4, 5, 6]
    hardware.motor_fresh = [True] * 6
    hardware.calibrated = True
    hardware.kinematics_valid = False
    hardware.raw_protocol_positions_deg = [0.03, -319.55, 631.27, 0.04, -0.04, 0.03]
    hardware.positions = [float("nan")] * 6
    hardware.kinematics_error_code = "OUTPUT_POSITION_LIMIT"
    hardware.kinematics_error_axis = 3
    hardware.kinematics_error_value = -8.146277666983426
    hardware.kinematics_error_message = "model output limit exceeded at motor_03"
    hardware.diagnostic = "value=-8.146277666983426 rad, limit=[-7.841,7.841] rad"
    return hardware


class ReadOnlyBridge:
    hardware_type = "real"
    commissioning = False
    read_only = True

    def state_snapshot(self):
        return monitor_state(fault_hardware())


def test_fault_still_publishes_raw_feedback_and_complete_diagnostics():
    gateway = WebSocketGateway(ReadOnlyBridge())
    gateway._pose = lambda: {}
    data = json.loads(gateway._state_message())
    assert data["state_available"] is True
    assert data["robot_state"] == "FAULT"
    assert data["current_joint_angles"] == [None] * 6
    assert data["raw_protocol_positions_deg"][1] == -319.55
    assert data["kinematics_error"]["axis"] == 3
    assert "limit=[-7.841,7.841]" in data["diagnostic"]
    assert data["read_only"] is True
    assert data["control_permitted"] is False
    assert data["supported_motion_commands"] == []


@pytest.mark.parametrize("command", sorted(COMMANDS))
def test_read_only_rejects_every_control_command_without_ros_clients(command):
    with pytest.raises(ProtocolError, match="只读排查"):
        asyncio.run(CommandGateway(ReadOnlyBridge()).dispatch(Request(1, command, {})))


def test_stale_motor_feedback_is_disconnected_not_healthy():
    hardware = fault_hardware()
    hardware.motor_fresh[1] = False
    assert monitor_state(hardware).state == RobotState.DISCONNECTED


def test_invalid_calibration_is_a_visible_fault():
    hardware = fault_hardware()
    hardware.kinematics_valid = True
    hardware.calibrated = False
    assert monitor_state(hardware).state == RobotState.FAULT
