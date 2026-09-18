#!/usr/bin/env python3

import math
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
                "joint_04",
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
                {"motor_id", "protocol_angle_ratio", "motor_direction"} |
                ({"feedback_position_tolerance_rad"}
                 if joint.attrib["name"] in ("joint_02", "joint_04") else set()),
            )
            tolerance = float(parameters.get("feedback_position_tolerance_rad", "0"))
            self.assertAlmostEqual(tolerance, math.radians(3)
                                   if joint.attrib["name"] in ("joint_02", "joint_04") else 0)

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
                "joint_03": ("joint_04", "1.0"),
                "joint_07": ("joint_06", "1.0"),
                "joint_08": ("joint_05", "-1.0"),
            },
        )
        joints = {joint.attrib["name"]: joint for joint in self.root.findall("joint")}
        for mimic_name, (target_name, _) in mimic.items():
            mimic_limit = joints[mimic_name].find("limit")
            target_limit = joints[target_name].find("limit")
            self.assertEqual(mimic_limit.attrib["lower"], target_limit.attrib["lower"])
            self.assertEqual(mimic_limit.attrib["upper"], target_limit.attrib["upper"])

    def test_yaml_contains_only_expected_six_axes(self) -> None:
        document = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))
        cable = document["cable_kinematics"]
        self.assertEqual(cable["model_version"], 3)
        self.assertEqual(cable["angle_unit"], "rad")
        self.assertEqual(cable["length_unit"], "mm")
        self.assertEqual(
            list(cable["axes"]),
            [
                "joint_01",
                "joint_02",
                "joint_04",
                "joint_05",
                "joint_06",
                "joint_09",
            ],
        )
        axes = cable["axes"]
        self.assertEqual(axes["joint_04"]["model"], "synchronized_pair_coupled")
        self.assertAlmostEqual(axes["joint_01"]["motor_radius_mm"], 16.25)
        self.assertAlmostEqual(axes["joint_01"]["joint_radius_mm"], 19.75)
        self.assertAlmostEqual(axes["joint_04"]["motor_radius_mm"], 16.25)
        self.assertAlmostEqual(axes["joint_04"]["joint3_radius_mm"], 28.5)
        self.assertAlmostEqual(axes["joint_04"]["joint4_radius_mm"], 23.25)
        self.assertAlmostEqual(axes["joint_04"]["joint2_guide_radius_mm"], 28.5)
        self.assertAlmostEqual(
            (
                axes["joint_04"]["joint3_radius_mm"]
                + axes["joint_04"]["joint4_radius_mm"]
            )
            / axes["joint_04"]["motor_radius_mm"],
            207.0 / 65.0,
        )
        for joint_name in ("joint_05", "joint_06"):
            self.assertAlmostEqual(
                axes[joint_name]["joint_radius_mm"]
                / axes[joint_name]["motor_radius_mm"],
                33.94 / 16.25,
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
        small_coupling = (
            small["joint2_guide_radius_mm"] / small["motor_radius_mm"]
        )
        large_coupling = (
            large["joint2_guide_radius_mm"] / large["motor_radius_mm"]
        )
        self.assertNotEqual(small_coupling, large_coupling)
        self.assertLess(
            abs(small_coupling - large_coupling)
            / max(abs(small_coupling), abs(large_coupling)),
            cable["validation"]["compatibility_tolerance"],
        )

        self.assertAlmostEqual(small["joint2_guide_radius_mm"], 20.25)
        self.assertAlmostEqual(axes["joint_05"]["joint_radius_mm"], 33.94)
        self.assertAlmostEqual(axes["joint_06"]["joint_radius_mm"], 33.94)

        output_limits = cable["output_limits"]
        self.assertEqual(output_limits["coordinate"], "reducer_output_shaft")
        self.assertEqual(output_limits["unit"], "rad")
        self.assertEqual(
            [name for name in output_limits if name.startswith("motor_")],
            [f"motor_{index:02d}" for index in range(1, 7)],
        )
        expected_upper = [3.819, 1.758, 7.841, 4.892, 4.127, 6.047]
        for index, upper in enumerate(expected_upper, start=1):
            limit = output_limits[f"motor_{index:02d}"]
            self.assertAlmostEqual(limit["lower_position_rad"], -upper)
            self.assertAlmostEqual(limit["upper_position_rad"], upper)

    def test_urdf_joint_limits_match_kinematic_derivation(self) -> None:
        expected = {
            "joint_01": (-3.141592654, 3.141592654),
            "joint_02": (-math.radians(100.7), math.radians(100.7)),
            "joint_04": (-math.radians(85.6), math.radians(85.6)),
            "joint_05": (-0.523598776, 0.523598776),
            "joint_06": (-1.047197551, 1.047197551),
            "joint_09": (-3.141592654, 3.141592654),
        }
        joints = {joint.attrib["name"]: joint for joint in self.root.findall("joint")}
        for name, (lower, upper) in expected.items():
            limit = joints[name].find("limit")
            self.assertIsNotNone(limit)
            self.assertAlmostEqual(float(limit.attrib["lower"]), lower)
            self.assertAlmostEqual(float(limit.attrib["upper"]), upper)

    def test_confirmed_packing_pose_is_inside_joint_limits(self) -> None:
        # Check without a tolerance: rounded URDF bounds must not exclude the
        # operator's degree-to-radian conversion at a software boundary.
        joints = {joint.attrib["name"]: joint for joint in self.root.findall("joint")}
        pose_deg = {
            "joint_01": 0.0,
            "joint_02": 100.7,
            "joint_04": -85.6,
            "joint_05": 0.0,
            "joint_06": 0.0,
            "joint_09": 0.0,
        }
        for name, degrees in pose_deg.items():
            limit = joints[name].find("limit")
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            position = math.radians(degrees)
            self.assertLessEqual(lower, position)
            self.assertGreaterEqual(upper, position)
            self.assertLessEqual(lower, -position)
            self.assertGreaterEqual(upper, -position)

    def test_output_limits_enclose_joint_limit_image(self) -> None:
        cable = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))[
            "cable_kinematics"
        ]
        axes = cable["axes"]
        joints = {joint.attrib["name"]: joint for joint in self.root.findall("joint")}

        def max_abs_limit(name: str) -> float:
            limit = joints[name].find("limit")
            return max(abs(float(limit.attrib["lower"])), abs(float(limit.attrib["upper"])))

        q1 = max_abs_limit("joint_01")
        q2 = max_abs_limit("joint_02")
        q3 = max_abs_limit("joint_04")
        q4 = max_abs_limit("joint_05")
        q5 = max_abs_limit("joint_06")
        q6 = max_abs_limit("joint_09")
        if cable.get("swap_wrist_channels", False):
            q4, q5 = q5, q4
        coupling = axes["joint_04"]["joint2_guide_radius_mm"] / axes["joint_04"][
            "motor_radius_mm"
        ]
        branch_a = axes["joint_09"]["branch_a"]
        branch_b = axes["joint_09"]["branch_b"]
        alpha6 = 0.5 * (
            branch_a["joint_radius_mm"] / branch_a["motor_radius_mm"]
            + branch_b["joint_radius_mm"] / branch_b["motor_radius_mm"]
        )
        coupling6 = 0.5 * (
            branch_a["joint2_guide_radius_mm"] / branch_a["motor_radius_mm"]
            + branch_b["joint2_guide_radius_mm"] / branch_b["motor_radius_mm"]
        )
        theoretical = [
            axes["joint_01"]["joint_radius_mm"]
            / axes["joint_01"]["motor_radius_mm"]
            * q1,
            axes["joint_02"]["joint_radius_mm"]
            / axes["joint_02"]["motor_radius_mm"]
            * q2,
            (
                axes["joint_04"]["joint3_radius_mm"]
                + axes["joint_04"]["joint4_radius_mm"]
            )
            / axes["joint_04"]["motor_radius_mm"]
            * q3
            + coupling * q2,
            axes["joint_05"]["joint_radius_mm"]
            / axes["joint_05"]["motor_radius_mm"]
            * math.sin(q4)
            + coupling * q2,
            axes["joint_06"]["joint_radius_mm"]
            / axes["joint_06"]["motor_radius_mm"]
            * math.sin(q5)
            + coupling * q2,
            alpha6 * q6 + coupling6 * q2,
        ]
        for index, required in enumerate(theoretical, start=1):
            configured = cable["output_limits"][f"motor_{index:02d}"][
                "upper_position_rad"
            ]
            self.assertGreaterEqual(configured, required)
            self.assertLess(configured - required, 0.0011)

        # Motor 6 is rounded above the larger manufactured branch envelope too.
        branch_a_required = (
            branch_a["joint_radius_mm"] / branch_a["motor_radius_mm"] * q6
            + branch_a["joint2_guide_radius_mm"]
            / branch_a["motor_radius_mm"]
            * q2
        )
        self.assertGreaterEqual(
            cable["output_limits"]["motor_06"]["upper_position_rad"],
            branch_a_required,
        )


if __name__ == "__main__":
    unittest.main()
