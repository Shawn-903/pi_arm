import argparse
from pi_arm_sdk import Arm

parser = argparse.ArgumentParser()
parser.add_argument("host")
parser.add_argument("--port", type=int, default=8765)
args = parser.parse_args()
with Arm(args.host, port=args.port) as arm:
    print(arm.info)
    print(arm.get_state())
