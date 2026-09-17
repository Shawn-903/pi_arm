#!/usr/bin/env python3
"""Offline checks for direction-only reuse of accepted packing measurements."""
import copy
import unittest

from rebind_cable_directions import validate_direction_change


class DirectionRebindTest(unittest.TestCase):
    def setUp(self):
        self.old = {'cable_kinematics': {'model_version': 3, 'axes': {'radius': 16.25}}}
        self.new = copy.deepcopy(self.old)
        self.new['cable_kinematics']['output_directions'] = {
            f'motor_{i:02d}': -1 if i in (3, 4) else 1 for i in range(1, 7)}

    def test_accepts_only_direction_edit_without_mutating_inputs(self):
        saved = copy.deepcopy(self.new)
        validate_direction_change(self.old, self.new)
        self.assertEqual(self.new, saved)
        validate_direction_change(self.new, self.old)

    def test_rejects_geometry_changes(self):
        self.new['cable_kinematics']['axes']['radius'] = 17
        with self.assertRaises(ValueError):
            validate_direction_change(self.old, self.new)

    def test_rejects_bad_or_missing_directions(self):
        for invalid in (True, 0, 2, 1.0, '1'):
            self.new['cable_kinematics']['output_directions']['motor_03'] = invalid
            with self.assertRaises(ValueError):
                validate_direction_change(self.old, self.new)
        del self.new['cable_kinematics']['output_directions']['motor_03']
        with self.assertRaises(ValueError):
            validate_direction_change(self.old, self.new)


if __name__ == '__main__':
    unittest.main()
