"""传入已核对的六关节角和末端位姿后运行。"""

import argparse
from pi_arm_sdk import Arm

parser = argparse.ArgumentParser()
parser.add_argument("host")
parser.add_argument("--joints-deg", nargs=6, type=float, required=True)
parser.add_argument("--position-mm", nargs=3, type=float, required=True)
parser.add_argument("--rpy-deg", nargs=3, type=float, required=True)
args = parser.parse_args()
with Arm(args.host) as arm:
    arm.enable()
    with arm.control("关节与直线运动"):
        print(arm.movej(args.joints_deg))
        print(arm.movel(args.position_mm, args.rpy_deg))
