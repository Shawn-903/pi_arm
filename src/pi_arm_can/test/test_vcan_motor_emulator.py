#!/usr/bin/env python3

import struct
import sys
import unittest
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE_ROOT / "scripts"))

from vcan_motor_emulator import Motor, response  # noqa: E402


class VcanMotorEmulatorTest(unittest.TestCase):
    def test_stop_preserves_enable_and_angle(self):
        motor = Motor(angle_raw=1234, speed_dps=3, enabled=True)
        acknowledgement = response(motor, bytes([0x81]) + bytes(7))
        self.assertEqual(acknowledgement, bytes([0x81]) + bytes(7))
        self.assertTrue(motor.enabled)
        self.assertEqual(motor.angle_raw, 1234)
        self.assertEqual(motor.speed_dps, 0)

    def test_position_command_round_trips_through_multi_turn_angle(self) -> None:
        motor = Motor()
        target_raw = -12345
        command = bytearray(8)
        command[0] = 0xA4
        command[2:4] = (100).to_bytes(2, "little", signed=False)
        command[4:8] = target_raw.to_bytes(4, "little", signed=True)

        response(motor, bytes(command))
        angle_reply = response(motor, bytes([0x92]) + bytes(7))

        self.assertIsNotNone(angle_reply)
        self.assertEqual(
            int.from_bytes(angle_reply[1:8], "little", signed=True), target_raw
        )

    def test_set_zero_ack_does_not_change_live_angle_before_power_cycle(self) -> None:
        motor = Motor(angle_raw=54321)

        acknowledgement = response(motor, bytes([0x19]) + bytes(7))

        self.assertEqual(acknowledgement, bytes([0x19]) + bytes(7))
        self.assertEqual(motor.angle_raw, 54321)

    def test_status_payload_remains_eight_bytes(self) -> None:
        motor = Motor(speed_dps=-12)
        reply = response(motor, bytes([0x9C]) + bytes(7))
        self.assertEqual(len(reply), 8)
        self.assertEqual(struct.unpack("<Bb h h H", reply)[3], -12)


if __name__ == "__main__":
    unittest.main()
