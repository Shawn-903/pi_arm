#!/usr/bin/env python3

import ast
import unittest
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
)


class BringupLaunchSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.source = LAUNCH_PATH.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source)

    def test_control_description_topic_is_scoped_by_backend_and_interface(self) -> None:
        assignment = next(
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "controller_robot_description_topic"
                for target in node.targets
            )
        )
        text = ast.unparse(assignment.value)
        self.assertIn("PathJoinSubstitution", text)
        self.assertIn("hardware_type", text)
        self.assertIn("can_interface", text)
        self.assertIn("/pi_arm/controller_robot_description", text)

    def test_publisher_and_controller_use_the_same_dedicated_topic(self) -> None:
        remap_count = self.source.count(
            '("robot_description", controller_robot_description_topic)'
        )
        self.assertEqual(remap_count, 2)
        self.assertNotIn('("~/robot_description", "/robot_description")', self.source)


if __name__ == "__main__":
    unittest.main()
