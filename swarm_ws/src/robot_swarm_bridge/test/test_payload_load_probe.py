#!/usr/bin/env python3
"""ROS-free checks for the loaded transport capacity probe."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest


class _Message:
    def __init__(self, *args, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


def _module(name, **values):
    module = types.ModuleType(name)
    for key, value in values.items():
        setattr(module, key, value)
    return module


def _load_probe():
    stubs = {
        'rosnode': _module('rosnode', get_node_names=lambda: []),
        'rospy': _module('rospy'),
        'gazebo_msgs': _module('gazebo_msgs'),
        'gazebo_msgs.msg': _module(
            'gazebo_msgs.msg', ModelState=_Message, ModelStates=_Message
        ),
        'gazebo_msgs.srv': _module(
            'gazebo_msgs.srv', DeleteModel=_Message,
            SetModelState=_Message, SpawnModel=_Message,
        ),
        'geometry_msgs': _module('geometry_msgs'),
        'geometry_msgs.msg': _module(
            'geometry_msgs.msg', Pose=_Message, Twist=_Message
        ),
        'rosgraph_msgs': _module('rosgraph_msgs'),
        'rosgraph_msgs.msg': _module(
            'rosgraph_msgs.msg', Clock=_Message
        ),
        'std_msgs': _module('std_msgs'),
        'std_msgs.msg': _module('std_msgs.msg', String=_Message),
    }
    originals = {name: sys.modules.get(name) for name in stubs}
    try:
        sys.modules.update(stubs)
        path = Path(__file__).with_name('robotswarm_payload_load_live.py')
        spec = importlib.util.spec_from_file_location(
            'robotswarm_payload_load_live_under_test', path
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


PROBE = _load_probe()


class PayloadLoadProbeTests(unittest.TestCase):
    def args(self):
        return types.SimpleNamespace(
            fleet_count=4,
            single_max_progress=0.05,
            root_only_max_progress=0.06,
            fleet_min_progress=0.20,
            minimum_gain=4.0,
            min_robot_progress=0.05,
            min_rtf=2.70,
        )

    @staticmethod
    def trial(
        count, payload_progress, robot_progress, rtf=2.95,
        connected_count=None,
    ):
        return {
            'robot_count': count,
            'payload_forward_progress_m': payload_progress,
            'robot_forward_progress_m': robot_progress,
            'real_time_factor': rtf,
            'final_connected_robot_count': (
                count if connected_count is None else connected_count
            ),
        }

    def test_layout_uses_two_balanced_push_chains(self):
        positions = PROBE.push_layout(8, -0.8, -1.6)
        self.assertEqual(8, len(positions))
        self.assertEqual(2, len({y for _, y in positions}))
        self.assertEqual(positions[0][0], positions[1][0])
        self.assertEqual(positions[2][0], positions[3][0])
        self.assertLess(positions[2][0], positions[0][0])
        self.assertEqual(len(positions), len(set(positions)))

    def test_single_robot_layout_is_centered_on_the_crate(self):
        self.assertEqual(
            [(-1.115, -1.6)],
            PROBE.push_layout(1, -0.8, -1.6),
        )

    def test_connection_report_requires_both_roots_and_rear_companions(self):
        robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
        layout = dict(zip(robots, PROBE.push_layout(4, 0.0, 0.0)))

        connected = PROBE.push_connections(robots, layout, (0.0, 0.0))
        self.assertTrue(all(item['connected'] for item in connected.values()))
        self.assertEqual('tb3_0', connected['tb3_2']['parent'])
        self.assertEqual('tb3_1', connected['tb3_3']['parent'])

        layout['tb3_2'] = (-0.60, -0.09)
        broken = PROBE.push_connections(robots, layout, (0.0, 0.0))
        self.assertFalse(broken['tb3_2']['connected'])

    def test_model_xml_uses_spawn_pose_instead_of_embedded_pose(self):
        model_xml = PROBE.read_model_xml(
            PROBE.LOADED_MODEL,
            "./model[@name='transport_object']",
        )
        self.assertTrue(model_xml.startswith('<sdf version='))
        self.assertNotIn('<pose>', model_xml)
        self.assertIn('<mass>0.75</mass>', model_xml)

    def test_capacity_gate_accepts_resistant_single_and_moving_fleet(self):
        single = self.trial(1, 0.02, {'tb3_0': 0.01})
        roots = self.trial(2, 0.03, {'tb3_0': 0.03, 'tb3_1': 0.03})
        fleet = self.trial(4, 0.24, {
            'tb3_1': 0.25, 'tb3_2': 0.25,
            'tb3_3': 0.24, 'tb3_4': 0.24,
        })

        failures, gain = PROBE.evaluate(single, roots, fleet, self.args())

        self.assertEqual([], failures)
        self.assertEqual(12.0, gain)

    def test_capacity_gate_rejects_easy_crate_and_idle_companion(self):
        single = self.trial(1, 0.10, {'tb3_0': 0.10})
        roots = self.trial(2, 0.10, {'tb3_0': 0.10, 'tb3_1': 0.10})
        fleet = self.trial(4, 0.21, {
            'tb3_1': 0.22, 'tb3_2': 0.22,
            'tb3_3': 0.00, 'tb3_4': 0.22,
        }, rtf=2.0, connected_count=3)

        failures, _gain = PROBE.evaluate(
            single, roots, fleet, self.args()
        )

        self.assertTrue(any('one robot moved' in item for item in failures))
        self.assertTrue(any('two payload roots' in item for item in failures))
        self.assertTrue(any('did not all advance' in item for item in failures))
        self.assertTrue(any('not connected' in item for item in failures))
        self.assertTrue(any('real-time factor' in item for item in failures))


if __name__ == '__main__':
    unittest.main()
