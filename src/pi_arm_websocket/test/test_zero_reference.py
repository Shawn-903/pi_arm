import asyncio
import math
from types import SimpleNamespace

import pytest

from pi_arm_websocket.commands import CommandGateway
from pi_arm_websocket.protocol import ProtocolError, Request


class Bridge:
    read_only = False
    commissioning = True

    def __init__(self):
        self.calls = []

    async def call_service(self, command, request):
        self.calls.append((command, request))
        return SimpleNamespace(message='saved',
                               raw_protocol_positions_deg=[1.0] * 6,
                               software_output_offsets_rad=[0.1] * 6)


def dispatch(bridge, **changes):
    params = dict(reference_joint_positions_deg=[10, 0, 0, 0, 0, 0], reference_pose_confirmed=True)
    params.update(changes)
    return asyncio.run(CommandGateway(bridge).dispatch(Request(1, 'set_zero_reference', params)))


def test_zero_reference_converts_units_and_returns_saved_reference():
    bridge = Bridge()
    result = dispatch(bridge)
    assert bridge.calls[0][0] == 'set_zero_reference'
    assert bridge.calls[0][1].reference_joint_positions_rad[0] == pytest.approx(math.radians(10))
    assert bridge.calls[0][1].reference_pose_confirmed
    assert "reference_id" not in result
    assert result['raw_protocol_positions_deg'] == [1.0] * 6


@pytest.mark.parametrize('changes', [
    {'reference_pose_confirmed': False}, {'reference_pose_confirmed': 1},
    {'reference_joint_positions_deg': [0] * 5},
    {'reference_joint_positions_deg': [float('nan')] * 6},
    {'raw_protocol_positions_deg': [0] * 6},
])
def test_invalid_reference_cannot_reach_hardware(changes):
    bridge = Bridge()
    with pytest.raises(ProtocolError):
        dispatch(bridge, **changes)
    assert not bridge.calls


@pytest.mark.parametrize('mode', ['normal', 'read_only'])
def test_zero_reference_requires_maintenance_mode(mode):
    bridge = Bridge()
    bridge.commissioning = False
    bridge.read_only = mode == 'read_only'
    with pytest.raises(ProtocolError):
        dispatch(bridge)
    assert not bridge.calls
