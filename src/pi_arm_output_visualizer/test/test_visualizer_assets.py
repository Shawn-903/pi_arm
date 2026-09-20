#!/usr/bin/env python3

import ast
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
KINEMATICS_YAML = (
    PACKAGE_ROOT.parent / "pi_arm_description" / "config" / "cable_kinematics.yaml"
)


class VisualizerAssetsTest(unittest.TestCase):
    def test_generated_slider_urdf_uses_yaml_output_limits(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(PACKAGE_ROOT / "scripts" / "generate_output_shaft_urdf.py"),
                str(KINEMATICS_YAML),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        root = ET.fromstring(completed.stdout)
        joints = root.findall("joint")
        self.assertEqual(
            [joint.attrib["name"] for joint in joints],
            [f"motor_{index:02d}_output" for index in range(1, 7)],
        )
        expected = [3.819, 1.758, 7.841, 4.892, 4.127, 6.047]
        for joint, upper in zip(joints, expected):
            self.assertEqual(joint.attrib["type"], "revolute")
            limit = joint.find("limit")
            self.assertIsNotNone(limit)
            self.assertAlmostEqual(float(limit.attrib["lower"]), -upper)
            self.assertAlmostEqual(float(limit.attrib["upper"]), upper)

    def test_launch_is_parseable_and_isolates_gui_description(self) -> None:
        launch_text = (
            PACKAGE_ROOT / "launch" / "output_shaft_visualizer.launch.py"
        ).read_text(encoding="utf-8")
        ast.parse(launch_text)
        self.assertIn("generate_output_shaft_urdf.py", launch_text)
        self.assertIn("gui_description", launch_text)
        self.assertIn("/pi_arm/output_shaft_robot_description", launch_text)
        self.assertIn("/pi_arm/model_output_shaft_states", launch_text)
        self.assertIn('DeclareLaunchArgument("use_gui"', launch_text)
        self.assertNotIn("controller_manager", launch_text)
        self.assertNotIn("pi_arm_can", launch_text)

    def test_rviz_opens_with_base_link_and_robot_model(self) -> None:
        rviz_text = (
            PACKAGE_ROOT / "config" / "output_shaft_visualizer.rviz"
        ).read_text(encoding="utf-8")
        self.assertIn("Fixed Frame: base_link", rviz_text)
        self.assertIn("rviz_default_plugins/RobotModel", rviz_text)
        self.assertIn("Value: /robot_description", rviz_text)


if __name__ == "__main__":
    unittest.main()
