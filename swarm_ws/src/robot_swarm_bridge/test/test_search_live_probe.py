#!/usr/bin/env python3
"""ROS-free checks for the collaborative-search hand-off probe."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest


def _load_probe_module():
    rospy = types.ModuleType("rospy")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    stubs = {
        "rospy": rospy,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    originals = {name: sys.modules.get(name) for name in stubs}
    try:
        sys.modules.update(stubs)
        path = Path(__file__).with_name("robotswarm_search_live.py")
        spec = importlib.util.spec_from_file_location(
            "robotswarm_search_probe_under_test", path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, original in originals.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


PROBE = _load_probe_module()


class SearchResponderEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.probe = PROBE.SearchProbe.__new__(PROBE.SearchProbe)
        self.probe.approach_initial_distances = {"tb3_1": 2.0}
        self.probe.approach_latest_distances = {"tb3_1": 2.0}
        self.probe.approach_correlated_samples = {}
        self.probe.approach_moving_samples = {}
        self.probe.approach_path_length = {}
        self.discovery = {
            "finder": "tb3_0",
            "notified_robots": ["tb3_1"],
        }

    def evidence(self):
        return self.probe._responder_evidence(self.discovery)["tb3_1"]

    def test_correlated_status_alone_is_not_a_response(self):
        self.probe.approach_correlated_samples["tb3_1"] = 5

        self.assertFalse(self.evidence()["responded"])

    def test_sustained_moving_commands_are_a_response(self):
        self.probe.approach_moving_samples["tb3_1"] = 2

        self.assertTrue(self.evidence()["responded"])

    def test_post_notice_motion_is_a_response(self):
        self.probe.approach_path_length["tb3_1"] = 0.03

        self.assertTrue(self.evidence()["responded"])


if __name__ == "__main__":
    unittest.main()
