import argparse
from pi_arm_sdk import Arm, ArmError

parser = argparse.ArgumentParser()
parser.add_argument("host")
parser.add_argument("point")
args = parser.parse_args()
with Arm(args.host) as arm:
    arm.enable()
    try:
        with arm.control("非阻塞任务"):
            task = arm.move_to_point(args.point, wait=False)
            print(task.query())
            print(task.wait(timeout_s=60.0))
    except ArmError as error:
        print(error.code, error.task_id, error.message, error.stop_confirmed)
