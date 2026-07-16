#!/usr/bin/env python3

import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from utils.robot_ids import sort_robot_ids, validate_robot_ids


class RobotIdTests(unittest.TestCase):
    def test_numeric_order_does_not_sort_10_before_2(self):
        self.assertEqual(
            sort_robot_ids(["tb3_10", "tb3_2", "tb3_1"]),
            ["tb3_1", "tb3_2", "tb3_10"],
        )

    def test_sort_removes_empty_and_duplicate_values(self):
        self.assertEqual(
            sort_robot_ids(["tb3_2", "", "tb3_2", "tb3_0"]),
            ["tb3_0", "tb3_2"],
        )

    def test_validation_rejects_duplicate_runtime_names(self):
        with self.assertRaisesRegex(ValueError, "unique"):
            validate_robot_ids(["tb3_1", "tb3_1"])

    def test_validation_rejects_non_session_namespace(self):
        with self.assertRaisesRegex(ValueError, "tb3_<number>"):
            validate_robot_ids(["robot_1"])


if __name__ == "__main__":
    unittest.main()
