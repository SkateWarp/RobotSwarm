#!/usr/bin/env python3

import importlib.util
import json
import math
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest import mock


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_DIR / 'src'
FORMATION_SCRIPT = PACKAGE_DIR / 'scripts' / 'behaviors' / 'formation_control.py'
sys.path.insert(0, str(PACKAGE_SOURCE))


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Point(Vector3):
    pass


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Pose:
    def __init__(self, x=0.0, y=0.0):
        self.position = Point(x=x, y=y)
        self.orientation = Quaternion()


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class String:
    def __init__(self, data=''):
        self.data = data


class Bool:
    def __init__(self, data=False):
        self.data = data


class Odometry:
    def __init__(self):
        self.pose = types.SimpleNamespace(pose=Pose())


class Marker:
    CYLINDER = 0
    ADD = 0
    SPHERE = 0
    ARROW = 0
    TEXT_VIEW_FACING = 0


class MarkerArray:
    def __init__(self):
        self.markers = []


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.unregistered = False

    def publish(self, message):
        self.messages.append(message)

    def unregister(self):
        self.unregistered = True


class FakeResource:
    def __init__(self):
        self.closed = False

    def unregister(self):
        self.closed = True

    def shutdown(self):
        self.closed = True


class FakePid:
    def __init__(self):
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


def _load_formation_module():
    rospy = _module(
        'rospy',
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
    )
    geometry_msg = _module(
        'geometry_msgs.msg',
        Twist=Twist,
        Pose=Pose,
        Point=Point,
        Quaternion=Quaternion,
    )
    nav_msg = _module('nav_msgs.msg', Odometry=Odometry)
    std_msg = _module('std_msgs.msg', String=String, Bool=Bool)
    visualization_msg = _module(
        'visualization_msgs.msg',
        Marker=Marker,
        MarkerArray=MarkerArray,
    )

    geometry = _module('geometry_msgs', msg=geometry_msg)
    nav = _module('nav_msgs', msg=nav_msg)
    std = _module('std_msgs', msg=std_msg)
    visualization = _module(
        'visualization_msgs', msg=visualization_msg
    )

    obstacle_module = _module(
        'core.obstacle_avoidance',
        ObstacleAvoidance=object,
    )
    core_module = _module('core', obstacle_avoidance=obstacle_module)
    core_module.__path__ = []

    stubs = {
        'rospy': rospy,
        'geometry_msgs': geometry,
        'geometry_msgs.msg': geometry_msg,
        'nav_msgs': nav,
        'nav_msgs.msg': nav_msg,
        'std_msgs': std,
        'std_msgs.msg': std_msg,
        'visualization_msgs': visualization,
        'visualization_msgs.msg': visualization_msg,
        'core': core_module,
        'core.obstacle_avoidance': obstacle_module,
    }

    spec = importlib.util.spec_from_file_location(
        'formation_control_under_test', FORMATION_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


formation = _load_formation_module()


def make_controller():
    controller = formation.FormationController.__new__(
        formation.FormationController
    )
    controller.command_lock = threading.RLock()
    controller.lock = threading.RLock()
    controller.is_running = False
    controller.is_paused = False
    controller.emergency_stop_active = False
    controller.current_task_id = None
    controller.formation_state = formation.FormationState.IDLE

    controller.robot_ids = ['tb3_0']
    controller.robot_count = 1
    controller.robot_poses = {'tb3_0': Pose()}
    controller.robot_yaws = {'tb3_0': 0.0}
    controller.formation_offsets = [(1.0, 0.0)]
    controller.assignments = {'tb3_0': 0}
    controller.assignment_pending = False
    controller._assignment_generation = 0

    controller.formation_type = 'line'
    controller.movement_mode = formation.MovementMode.MOVING
    controller.spacing = 1.0
    controller.centroid_speed = 0.1
    controller.path_radius = 2.5
    controller.centroid_path = formation.CentroidPath.CIRCULAR
    controller.centroid_waypoints = []
    controller.centroid_x = 0.0
    controller.centroid_y = 0.0
    controller.centroid_heading = 0.0
    controller.centroid_time = 99.0
    controller.current_waypoint_idx = 3

    controller.pid_linear = {'tb3_0': FakePid()}
    controller.pid_angular = {'tb3_0': FakePid()}
    controller.cmd_vel_pubs = {'tb3_0': FakePublisher()}
    controller._odom_subs = {}
    controller.avoidance = {}
    return controller


class FormationAssignmentTests(unittest.TestCase):
    def test_start_assigns_against_circular_path_t0_pose(self):
        controller = make_controller()
        captured = {}

        def capture_assignment(
            robot_positions,
            target_positions,
            previous_slots=None,
            switch_penalty=0.0,
        ):
            captured['targets'] = tuple(target_positions)
            return [0]

        with mock.patch.object(
            formation,
            'minimum_distance_assignment',
            side_effect=capture_assignment,
        ):
            controller._start_cb(String(data=json.dumps({
                'task_id': 'task-current',
                'formation_type': 'line',
                'movement_mode': 'moving',
                'spacing': 1.0,
            })))

        self.assertAlmostEqual(controller.centroid_x, 2.5)
        self.assertAlmostEqual(controller.centroid_y, 0.0)
        self.assertAlmostEqual(controller.centroid_heading, math.pi / 2.0)
        self.assertAlmostEqual(captured['targets'][0][0], 2.5)
        self.assertAlmostEqual(captured['targets'][0][1], 1.0)

        assigned_target = captured['targets'][0]
        controller._update_centroid(1.0 / 20.0)
        first_tick_target = controller._get_world_targets()[0]
        self.assertLess(
            math.hypot(
                first_tick_target[0] - assigned_target[0],
                first_tick_target[1] - assigned_target[1],
            ),
            0.02,
        )

    def test_emergency_stop_is_not_blocked_and_rejects_stale_solution(self):
        controller = make_controller()
        solver_entered = threading.Event()
        release_solver = threading.Event()
        emergency_finished = threading.Event()

        def blocking_assignment(*args, **kwargs):
            solver_entered.set()
            release_solver.wait(2.0)
            return [0]

        start_message = String(data=json.dumps({
            'task_id': 'task-current',
            'formation_type': 'line',
            'movement_mode': 'moving',
            'spacing': 1.0,
        }))

        with mock.patch.object(
            formation,
            'minimum_distance_assignment',
            side_effect=blocking_assignment,
        ):
            start_thread = threading.Thread(
                target=controller._start_cb, args=(start_message,)
            )
            start_thread.start()
            self.assertTrue(solver_entered.wait(1.0))

            def emergency_stop():
                controller._emergency_stop_cb(Bool(data=True))
                emergency_finished.set()

            emergency_thread = threading.Thread(target=emergency_stop)
            emergency_thread.start()
            try:
                self.assertTrue(
                    emergency_finished.wait(0.5),
                    'emergency stop waited for the Hungarian solver',
                )
            finally:
                release_solver.set()
                start_thread.join(2.0)
                emergency_thread.join(2.0)

        self.assertFalse(start_thread.is_alive())
        self.assertFalse(emergency_thread.is_alive())
        self.assertFalse(controller.is_running)
        self.assertEqual(
            controller.formation_state, formation.FormationState.STOPPED
        )
        self.assertEqual(controller.assignments, {})
        self.assertFalse(controller.assignment_pending)


class FormationLifecycleTests(unittest.TestCase):
    def test_lifecycle_commands_require_the_exact_nonempty_task_id(self):
        controller = make_controller()
        controller.current_task_id = 'task-current'
        controller.is_running = True
        publisher = controller.cmd_vel_pubs['tb3_0']

        controller._pause_cb(String(data=json.dumps({
            'task_id': 'task-stale'
        })))
        self.assertFalse(controller.is_paused)
        self.assertEqual(publisher.messages, [])

        controller._pause_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))
        self.assertTrue(controller.is_paused)
        self.assertEqual(len(publisher.messages), 1)

        controller._resume_cb(String(data='{}'))
        self.assertTrue(controller.is_paused)
        controller._resume_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))
        self.assertFalse(controller.is_paused)

        publisher.messages.clear()
        controller._stop_cb(String(data=json.dumps({
            'task_id': 'task-stale'
        })))
        self.assertTrue(controller.is_running)
        self.assertEqual(publisher.messages, [])

        controller._stop_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))
        self.assertFalse(controller.is_running)
        self.assertEqual(
            controller.formation_state, formation.FormationState.STOPPED
        )
        self.assertEqual(len(publisher.messages), 1)

    def test_empty_fleet_cancels_task_and_tears_down_robot_resources(self):
        controller = make_controller()
        controller.current_task_id = 'task-current'
        controller.is_running = True
        controller.is_paused = True
        publisher = controller.cmd_vel_pubs['tb3_0']
        subscriber = FakeResource()
        avoidance = FakeResource()
        controller._odom_subs['tb3_0'] = subscriber
        controller.avoidance['tb3_0'] = avoidance

        controller._fleet_list_cb(String(data=''))

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertIsNone(controller.current_task_id)
        self.assertEqual(
            controller.formation_state, formation.FormationState.STOPPED
        )
        self.assertEqual(controller.robot_ids, [])
        self.assertEqual(controller.assignments, {})
        self.assertTrue(publisher.messages)
        self.assertTrue(publisher.unregistered)
        self.assertTrue(subscriber.closed)
        self.assertTrue(avoidance.closed)

        controller._resume_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)

    def test_start_is_rejected_while_fleet_is_empty(self):
        controller = make_controller()
        controller.robot_ids = []
        controller.robot_count = 0
        controller.robot_poses = {}
        controller.robot_yaws = {}
        controller.formation_offsets = []
        controller.assignments = {}
        controller.pid_linear = {}
        controller.pid_angular = {}
        controller.cmd_vel_pubs = {}

        controller._start_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))

        self.assertFalse(controller.is_running)
        self.assertIsNone(controller.current_task_id)
        self.assertEqual(
            controller.formation_state, formation.FormationState.IDLE
        )


if __name__ == '__main__':
    unittest.main()
