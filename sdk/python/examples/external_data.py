"""上游设备将测量结果写为 JSON；Python 根据测量值选择已保存点位。"""

import argparse
import json
from pathlib import Path
from pi_arm_sdk import Arm

parser = argparse.ArgumentParser()
parser.add_argument("host")
parser.add_argument("measurement_file")
parser.add_argument("accepted_point")
parser.add_argument("rejected_point")
args = parser.parse_args()
with Arm(args.host) as arm:
    arm.enable()
    with arm.control("测量分流"):
        measurement = json.loads(Path(args.measurement_file).read_text(encoding="utf-8"))
        destination = args.accepted_point if measurement["accepted"] is True else args.rejected_point
        arm.move_to_point(destination)
