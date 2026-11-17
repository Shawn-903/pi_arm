import argparse
from pi_arm_sdk import Arm

parser = argparse.ArgumentParser()
parser.add_argument("host")
parser.add_argument("a")
parser.add_argument("b")
args = parser.parse_args()
with Arm(args.host) as arm:
    arm.enable()
    with arm.control("点位往返"):
        for _ in range(3):
            arm.move_to_point(args.a)
            arm.wait(1.0)
            arm.move_to_point(args.b)
