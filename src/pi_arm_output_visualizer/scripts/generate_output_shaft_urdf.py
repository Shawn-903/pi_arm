#!/usr/bin/env python3

"""Generate the six-slider URDF from cable_kinematics.yaml."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


MOTOR_NAMES = tuple(f"motor_{index:02d}" for index in range(1, 7))


def load_output_limits(path: Path) -> list[tuple[str, float, float]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    limits = document["cable_kinematics"]["output_limits"]
    if limits.get("coordinate") != "reducer_output_shaft":
        raise ValueError("output_limits.coordinate must be reducer_output_shaft")
    if limits.get("unit") != "rad":
        raise ValueError("output_limits.unit must be rad")

    result = []
    for motor_name in MOTOR_NAMES:
        motor_limit = limits[motor_name]
        lower = float(motor_limit["lower_position_rad"])
        upper = float(motor_limit["upper_position_rad"])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"invalid output limit for {motor_name}")
        result.append((f"{motor_name}_output", lower, upper))
    return result


def make_urdf(limits: list[tuple[str, float, float]]) -> str:
    robot = ET.Element("robot", {"name": "pi_arm_output_shaft_gui"})
    ET.SubElement(robot, "link", {"name": "output_shaft_reference"})
    for joint_name, lower, upper in limits:
        link_name = f"{joint_name}_link"
        ET.SubElement(robot, "link", {"name": link_name})
        joint = ET.SubElement(robot, "joint", {"name": joint_name, "type": "revolute"})
        ET.SubElement(joint, "parent", {"link": "output_shaft_reference"})
        ET.SubElement(joint, "child", {"link": link_name})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": format(lower, ".17g"),
                "upper": format(upper, ".17g"),
                "effort": "1.0",
                "velocity": "100.0",
            },
        )
    return ET.tostring(robot, encoding="unicode", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("cable_kinematics_file", type=Path)
    args = parser.parse_args()
    print(make_urdf(load_output_limits(args.cable_kinematics_file)))


if __name__ == "__main__":
    main()
