"""Execute calibration poses, dwell at each point and save encoder feedback."""

import argparse
import json
import math
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions

from .plan import load_poses
from .task import CalibrationMotionError, CalibrationMotionTask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="JSON pose list using joint_positions_deg")
    parser.add_argument("--output", required=True, help="new JSONL result file")
    parser.add_argument("--speed-deg-s", type=float, default=5.0)
    parser.add_argument("--output-speed-deg-s", type=float, default=5.0)
    parser.add_argument("--motion-timeout-sec", type=float, default=180.0)
    args = parser.parse_args()
    task = None
    previous_handlers = {}
    try:
        poses = load_poses(args.plan)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        task = CalibrationMotionTask()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: task.request_stop())
        count = task.run(
            poses, args.output, joint_speed_rad_s=math.radians(args.speed_deg_s),
            output_speed_rad_s=math.radians(args.output_speed_deg_s),
            motion_timeout_sec=args.motion_timeout_sec,
        )
        print(json.dumps({"success": True, "samples": count, "output": args.output}))
        return 0
    except (CalibrationMotionError, OSError, ValueError, RuntimeError) as error:
        print(json.dumps({"success": False, "message": str(error),
                          "stop_confirmed": getattr(error, "stop_confirmed", None)}))
        return 1
    finally:
        if task is not None:
            task.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        rclpy.try_shutdown()
