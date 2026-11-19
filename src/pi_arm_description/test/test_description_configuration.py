#!/usr/bin/env python3

import math
import subprocess
import tempfile
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
        output_limits = cable["output_limits"]
        self.assertEqual(output_limits["coordinate"], "reducer_output_shaft")
        self.assertEqual(output_limits["unit"], "rad")
        self.assertEqual((YAML_PATH.parent / output_limits["joint_limits_from"]).resolve(), XACRO_PATH)

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

    def test_changing_only_j2_radius_updates_just_its_output_range(self):
        model = yaml.safe_load(YAML_PATH.read_text())
        limits = model["cable_kinematics"]["output_limits"]
        limits["joint_limits_from"] = str(XACRO_PATH)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cable.yaml"

            def ranges():
                path.write_text(yaml.safe_dump(model))
                output = subprocess.check_output(["pi_arm_output_limits", str(path)], text=True)
                return [list(map(float, line.split())) for line in output.splitlines()]

            axis = model["cable_kinematics"]["axes"]["joint_02"]
            limits.pop("motor_02", None)
            axis.update(motor_radius_mm=16.0, joint_radius_mm=16.0)
            before = ranges()
            axis["motor_radius_mm"] = 28.5
            after = ranges()
            self.assertEqual(before[1], [-1.758, 1.758])
            self.assertEqual(after[1], [-0.987, 0.987])
            self.assertEqual(before[:1] + before[2:], after[:1] + after[2:])
            limits["motor_02"] = {"lower_position_rad": -0.5, "upper_position_rad": 2.0}
            self.assertEqual(ranges()[1], [-0.5, 0.987])

    def test_invalid_source_joint_limits_are_reported(self):
        model = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory) / "arm.urdf"
            config = Path(directory) / "cable.yaml"
            model["cable_kinematics"]["output_limits"]["joint_limits_from"] = urdf.name
            config.write_text(yaml.safe_dump(model), encoding="utf-8")
            for defect in ("duplicate", "upper"):
                with self.subTest(defect=defect):
                    tree = ET.parse(XACRO_PATH)
                    joint = tree.find("joint[@name='joint_02']")
                    if defect == "duplicate":
                        tree.getroot().append(joint)
                    else:
                        joint.find("limit").set("upper", "1.5rad")
                    tree.write(urdf)
                    result = subprocess.run(["pi_arm_output_limits", str(config)],
                                            text=True, capture_output=True, check=False)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(defect, result.stderr)
                    self.assertIn("joint_02", result.stderr)

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
        branches = [axes["joint_09"]["branch_a"], axes["joint_09"]["branch_b"]]
        axis6_range = max(
            (b["joint_radius_mm"] * q6 + b["joint2_guide_radius_mm"] * q2) / b["motor_radius_mm"]
            for b in branches
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
            + axes["joint_05"]["joint2_guide_radius_mm"] / axes["joint_05"]["motor_radius_mm"] * q2,
            axes["joint_06"]["joint_radius_mm"]
            / axes["joint_06"]["motor_radius_mm"]
            * math.sin(q5)
            + axes["joint_06"]["joint2_guide_radius_mm"] / axes["joint_06"]["motor_radius_mm"] * q2,
            axis6_range,
        ]
        actual = [list(map(float, line.split())) for line in subprocess.check_output(
            ["pi_arm_output_limits", str(YAML_PATH)], text=True).splitlines()]
        self.assertEqual(len(actual), 6)
        for i, ((lower, upper), required) in enumerate(zip(actual, theoretical), start=1):
            bound = math.ceil(required * 1000) / 1000
            cap = cable["output_limits"].get(f"motor_{i:02d}", {})
            self.assertAlmostEqual(lower, max(-bound, cap.get("lower_position_rad", -bound)))
            self.assertAlmostEqual(upper, min(bound, cap.get("upper_position_rad", bound)))


if __name__ == "__main__":
    unittest.main()
