"""Capture a known physical pose as a persistent host-side zero reference."""

import argparse
import json
import math

import rclpy

from .client import TaskClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-deg", type=float, nargs=6, default=[0.0] * 6)
    parser.add_argument("--confirm-reference", action="store_true", required=True,
                        help="confirm that the arm is physically at the specified reference pose")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("pi_arm_zero_client")
    try:
        client = TaskClient(node)
        if not client.wait_for_service():
            raise RuntimeError("pi_arm_tasks is unavailable")
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
