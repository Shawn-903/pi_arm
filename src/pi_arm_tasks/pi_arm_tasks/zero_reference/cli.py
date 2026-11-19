"""Capture a known physical pose as a persistent host-side zero reference."""

import argparse
import json
import math

import rclpy
from pi_arm_websocket.presets import PACKING_JOINTS_DEG

from .client import ZeroReferenceClient


def add_reference_arguments(parser):
    parser.add_argument("--reference-deg", type=float, nargs=6, default=PACKING_JOINTS_DEG,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="参考关节角（°），默认为机械零位（打包姿态）%(default)s")
    parser.add_argument("--confirm-reference", action="store_true", required=True,
                        help="确认实物已处于指定姿态并保持静止")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_reference_arguments(parser)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("pi_arm_zero_client")
    try:
        client = ZeroReferenceClient(node)
        if not client.wait_for_service():
            raise RuntimeError("zero-reference service is unavailable")
        future = client.set_zero_reference(
            [math.radians(value) for value in args.reference_deg],
            reference_pose_confirmed=args.confirm_reference,
        )
        rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
        if not future.done():
            raise RuntimeError("result unknown; inspect software_zero_id, do not retry automatically")
        result = future.result()
        print(json.dumps(dict(success=result.success, code=result.code, message=result.message,
                              reference_id=result.reference_id,
                              raw_protocol_positions_deg=list(result.raw_protocol_positions_deg),
                              software_output_offsets_rad=list(result.software_output_offsets_rad))))
        return 0 if result.success else 1
    except (RuntimeError, ValueError) as error:
        print(json.dumps(dict(success=False, message=str(error))))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
