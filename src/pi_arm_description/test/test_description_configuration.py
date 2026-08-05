#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "urdf" / "pi_arm.urdf.xacro"
YAML_PATH = PACKAGE_ROOT / "config" / "cable_kinematics.yaml"
XACRO_NS = {"xacro": "http://www.ros.org/wiki/xacro"}


class DescriptionConfigurationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = ET.parse(XACRO_PATH)
        self.root = self.tree.getroot()

    def test_ros2_control_has_six_expected_active_joints(self) -> None:
        ros2_control = self.root.find("ros2_control")
        self.assertIsNotNone(ros2_control)
        joint_names = [joint.attrib["name"] for joint in ros2_control.findall("joint")]
        self.assertEqual(
            joint_names,
            [
                "joint_01",
                "joint_02",
                "joint_03",
                "joint_05",
                "joint_06",
                "joint_09",
            ],
        )

        for joint in ros2_control.findall("joint"):
            parameters = {
                parameter.attrib["name"]: (parameter.text or "").strip()
                for parameter in joint.findall("param")
            }
            self.assertEqual(
                set(parameters),
                {"motor_id", "protocol_angle_ratio", "motor_direction"},
            )

    def test_hardware_references_global_cable_file(self) -> None:
        arguments = {
            argument.attrib["name"]
            for argument in self.root.findall("xacro:arg", XACRO_NS)
        }
        self.assertIn("cable_kinematics_file", arguments)
        hardware = self.root.find("ros2_control/hardware")
        parameters = {
            parameter.attrib["name"]: (parameter.text or "").strip()
            for parameter in hardware.findall("param")
        }
        self.assertEqual(
            parameters["cable_kinematics_file"],
            "$(arg cable_kinematics_file)",
        )

    def test_mechanical_mimic_relations_are_unchanged(self) -> None:
        mimic = {}
        for joint in self.root.findall("joint"):
            element = joint.find("mimic")
            if element is not None:
                mimic[joint.attrib["name"]] = (
                    element.attrib["joint"],
                    element.attrib["multiplier"],
                )
        self.assertEqual(
            mimic,
            {
                "joint_04": ("joint_03", "1.0"),
                "joint_07": ("joint_06", "1.0"),
                "joint_08": ("joint_05", "-1.0"),
            },
        )

    def test_yaml_contains_only_expected_six_axes(self) -> None:
        document = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))
        cable = document["cable_kinematics"]
        self.assertEqual(cable["model_version"], 2)
        self.assertEqual(cable["angle_unit"], "rad")
        self.assertEqual(cable["length_unit"], "mm")
        self.assertEqual(
            list(cable["axes"]),
            [
                "joint_01",
                "joint_02",
                "joint_03",
                "joint_05",
                "joint_06",
                "joint_09",
            ],
        )
        axes = cable["axes"]
        self.assertEqual(axes["joint_03"]["model"], "synchronized_pair_coupled")
        self.assertAlmostEqual(axes["joint_01"]["motor_radius_mm"], 16.25)
        self.assertAlmostEqual(axes["joint_01"]["joint_radius_mm"], 19.75)
        self.assertAlmostEqual(axes["joint_03"]["motor_radius_mm"], 16.25)
        self.assertAlmostEqual(axes["joint_03"]["joint3_radius_mm"], 28.5)
        self.assertAlmostEqual(axes["joint_03"]["joint4_radius_mm"], 23.25)
        self.assertAlmostEqual(axes["joint_03"]["joint2_guide_radius_mm"], 28.5)
        self.assertAlmostEqual(
            (
                axes["joint_03"]["joint3_radius_mm"]
                + axes["joint_03"]["joint4_radius_mm"]
            )
            / axes["joint_03"]["motor_radius_mm"],
            207.0 / 65.0,
        )
        for joint_name in ("joint_05", "joint_06"):
            self.assertAlmostEqual(
                axes[joint_name]["joint_radius_mm"]
                / axes[joint_name]["motor_radius_mm"],
                214.0 / 65.0,
            )
            self.assertAlmostEqual(
                axes[joint_name]["joint2_guide_radius_mm"]
                / axes[joint_name]["motor_radius_mm"],
                114.0 / 65.0,
            )
        axis6 = cable["axes"]["joint_09"]
        small = axis6["branch_a"]
        large = axis6["branch_b"]
        self.assertAlmostEqual(
            small["joint_radius_mm"] / small["motor_radius_mm"],
            large["joint_radius_mm"] / large["motor_radius_mm"],
        )
        self.assertAlmostEqual(
            small["joint2_guide_radius_mm"] / small["motor_radius_mm"],
            large["joint2_guide_radius_mm"] / large["motor_radius_mm"],
        )


if __name__ == "__main__":
    unittest.main()
