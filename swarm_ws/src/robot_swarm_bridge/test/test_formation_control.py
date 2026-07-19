#!/usr/bin/env python3

import importlib.util
import json
import math
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_DIR / 'src'
FORMATION_SCRIPT = PACKAGE_DIR / 'scripts' / 'behaviors' / 'formation_control.py'
SPAWN_CONFIG = PACKAGE_DIR / 'config' / 'arena_spawn_zones.yaml'
sys.path.insert(0, str(PACKAGE_SOURCE))

from robot_swarm_bridge.algorithms.formation import (  # noqa: E402
    find_safe_formation_plan,
    find_safe_formation_center,
    formation_targets_are_safe,
    straight_route_is_safe,
)

with SPAWN_CONFIG.open(encoding='utf-8') as config_file:
    SPAWN_SETTINGS = yaml.safe_load(config_file)


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


class ModelStates:
    def __init__(self, names=None, poses=None):
        self.name = list(names or [])
        self.pose = list(poses or [])


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


class PassThroughAvoidance:
    def update_robot_positions(self, _positions):
        pass

    def apply_avoidance(self, command):
        return command


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
    gazebo_msg = _module('gazebo_msgs.msg', ModelStates=ModelStates)
    std_msg = _module('std_msgs.msg', String=String, Bool=Bool)
    visualization_msg = _module(
        'visualization_msgs.msg',
        Marker=Marker,
        MarkerArray=MarkerArray,
    )

    geometry = _module('geometry_msgs', msg=geometry_msg)
    nav = _module('nav_msgs', msg=nav_msg)
    gazebo = _module('gazebo_msgs', msg=gazebo_msg)
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
        'gazebo_msgs': gazebo,
        'gazebo_msgs.msg': gazebo_msg,
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
    controller.odom_received_at = {'tb3_0': None}
    controller.odom_timeout_wall_s = 0.75
    controller.odom_initialization_timeout_wall_s = 5.0
    controller.task_started_at = None
    controller.stale_odometry = []
    controller.waiting_for_odometry = []
    controller.formation_offsets = [(1.0, 0.0)]
    controller.assignments = {'tb3_0': 0}
    controller.route_waypoints = {}
    controller.route_waypoint_indices = {}
    controller.route_batches = []
    controller.route_batch_index = 0
    controller.assignment_pending = False
    controller._assignment_generation = 0
    controller._slot_reached = {'tb3_0': False}
    controller._settled_duration = 0.0
    controller.position_tolerance = 0.09
    controller.position_release_tolerance = 0.14
    controller.settle_time = 0.5
    controller.arena_size = 10.0
    controller.arena_margin = 0.35
    controller.arena_profile = 'swarm_arena'
    controller.formation_obstacle_clearance = SPAWN_SETTINGS[
        'spawn_obstacle_clearance'
    ]
    controller.formation_search_step = SPAWN_SETTINGS['spawn_search_step']
    controller.formation_search_limit = SPAWN_SETTINGS[
        'formation_search_limit'
    ]
    controller.spawn_exclusion_zones = SPAWN_SETTINGS[
        'spawn_exclusion_zones'
    ]
    controller.model_poses = {}
    controller.placement_error = None

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
    controller.status_pub = FakePublisher()
    controller._odom_subs = {}
    controller.avoidance = {}
    return controller


def make_closed_loop_controller(robot_count, shape, spacing):
    """Build a ROS-free differential-drive formation simulation."""
    controller = make_controller()
    controller.robot_ids = [f'tb3_{index}' for index in range(robot_count)]
    controller.robot_count = robot_count

    columns = int(math.ceil(math.sqrt(robot_count)))
    rows = int(math.ceil(robot_count / columns))
    poses = []
    for index in range(robot_count):
        row = index // columns
        column = index % columns
        poses.append(Pose(
            x=(column - (columns - 1) / 2.0) * 0.6,
            y=(row - (rows - 1) / 2.0) * 0.6,
        ))

    controller.robot_poses = dict(zip(controller.robot_ids, poses))
    controller.robot_yaws = {
        robot_id: 0.0 for robot_id in controller.robot_ids
    }
    controller.formation_type = shape
    controller.spacing = spacing
    controller.movement_mode = formation.MovementMode.STATIC
    controller.centroid_x = sum(p.position.x for p in poses) / robot_count
    controller.centroid_y = sum(p.position.y for p in poses) / robot_count
    controller.centroid_heading = 0.0
    controller.formation_offsets = controller._compute_formation_positions(
        shape, robot_count, spacing
    )

    targets = controller._get_world_targets()
    slots = formation.minimum_distance_assignment(
        [(pose.position.x, pose.position.y) for pose in poses],
        targets,
    )
    controller.assignments = dict(zip(controller.robot_ids, slots))
    controller.assignment_pending = False
    controller._slot_reached = {
        robot_id: False for robot_id in controller.robot_ids
    }
    controller._settled_duration = 0.0

    controller.control_rate = 20.0
    controller.max_linear_vel = 0.2
    controller.max_angular_vel = 1.5
    controller.pid_linear = {
        robot_id: formation.PIDController(0.6, 0.01, 0.15, 0.2)
        for robot_id in controller.robot_ids
    }
    controller.pid_angular = {
        robot_id: formation.PIDController(1.2, 0.0, 0.1, 1.5)
        for robot_id in controller.robot_ids
    }
    controller.cmd_vel_pubs = {
        robot_id: FakePublisher() for robot_id in controller.robot_ids
    }
    controller.avoidance = {
        robot_id: PassThroughAvoidance()
        for robot_id in controller.robot_ids
    }
    controller.is_running = True
    controller._publish_status = lambda *args, **kwargs: None
    controller._publish_markers = lambda *args, **kwargs: None
    return controller, targets


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

    def test_static_assignment_uses_the_current_fleet_centroid(self):
        controller = make_controller()
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.robot_poses = {
            'tb3_0': Pose(x=2.0, y=3.0),
            'tb3_1': Pose(x=4.0, y=3.0),
        }
        controller.robot_yaws = {'tb3_0': 0.0, 'tb3_1': 0.0}
        controller.formation_offsets = [(-0.5, 0.0), (0.5, 0.0)]
        controller.assignments = {}
        controller.movement_mode = formation.MovementMode.STATIC

        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(controller._compute_and_commit_assignment(snapshot))

        self.assertAlmostEqual(controller.centroid_x, 3.0)
        self.assertAlmostEqual(controller.centroid_y, 3.0)
        self.assertEqual(
            snapshot['target_world'],
            ((2.5, 3.0), (3.5, 3.0)),
        )

    def test_moved_transport_object_changes_the_safe_static_center(self):
        controller = make_controller()
        controller.movement_mode = formation.MovementMode.STATIC
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.robot_poses = {
            'tb3_0': Pose(x=-0.6, y=0.0),
            'tb3_1': Pose(x=0.6, y=0.0),
        }
        controller.robot_yaws = {'tb3_0': 0.0, 'tb3_1': 0.0}
        controller.formation_offsets = [(-0.35, 0.0), (0.35, 0.0)]
        controller._model_states_cb(ModelStates(
            names=['transport_object'],
            poses=[Pose(x=0.0, y=0.0)],
        ))

        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(controller._compute_and_commit_assignment(snapshot))

        self.assertIsNotNone(snapshot)
        self.assertNotEqual(
            (0.0, 0.0),
            (controller.centroid_x, controller.centroid_y),
        )
        self.assertTrue(formation_targets_are_safe(
            snapshot['target_world'],
            controller.arena_size,
            controller.arena_margin,
            controller.formation_obstacle_clearance,
            controller.spawn_exclusion_zones,
            controller.arena_profile,
            controller.model_poses,
        ))

    def test_moved_payload_diamond_uses_routed_assignment(self):
        controller = make_controller()
        positions = [
            (1.37707, 0.609376),
            (1.05755, 1.49906),
            (0.227548, 1.95738),
            (-0.696883, 1.79625),
            (-1.29892, 1.05796),
            (-1.29361, 0.123469),
            (-0.677494, -0.602777),
            (0.24658, -0.749901),
            (1.06862, -0.277766),
        ]
        controller.robot_ids = [
            f'tb3_{index}' for index in range(len(positions))
        ]
        controller.robot_count = len(positions)
        controller.robot_poses = {
            robot_id: Pose(x=x, y=y)
            for robot_id, (x, y) in zip(
                controller.robot_ids, positions
            )
        }
        controller.robot_yaws = {
            robot_id: 0.0 for robot_id in controller.robot_ids
        }
        controller.formation_type = 'diamond'
        controller.spacing = 0.55
        controller.movement_mode = formation.MovementMode.STATIC
        controller.formation_offsets = (
            controller._compute_formation_positions('diamond', 9, 0.55)
        )
        controller.model_poses = {
            'transport_object': (-0.8, -1.6, 0.0)
        }

        snapshot = controller._prepare_assignment_locked()
        started_at = time.perf_counter()
        committed = controller._compute_and_commit_assignment(snapshot)
        elapsed = time.perf_counter() - started_at

        self.assertTrue(committed)
        self.assertIsNone(controller.placement_error)
        self.assertEqual(9, len(controller.assignments))
        self.assertEqual(9, len(controller.route_waypoints))
        self.assertTrue(controller.route_batches)
        self.assertLess(elapsed, 5.0)

    def test_static_route_batches_wait_for_the_active_corridor(self):
        controller, targets = make_closed_loop_controller(2, 'line', 1.0)
        first, second = controller.robot_ids
        controller.route_waypoints = {
            first: [targets[controller.assignments[first]]],
            second: [targets[controller.assignments[second]]],
        }
        controller.route_waypoint_indices = {first: 0, second: 0}
        controller.route_batches = [[first], [second]]
        controller.route_batch_index = 0

        controller._control_step(None)

        first_command = controller.cmd_vel_pubs[first].messages[-1]
        second_command = controller.cmd_vel_pubs[second].messages[-1]
        self.assertNotEqual(
            (0.0, 0.0),
            (first_command.linear.x, first_command.angular.z),
        )
        self.assertEqual(
            (0.0, 0.0),
            (second_command.linear.x, second_command.angular.z),
        )

        first_target = targets[controller.assignments[first]]
        controller.robot_poses[first].position.x = first_target[0]
        controller.robot_poses[first].position.y = first_target[1]
        controller._control_step(None)
        controller._control_step(None)

        second_command = controller.cmd_vel_pubs[second].messages[-1]
        self.assertNotEqual(
            (0.0, 0.0),
            (second_command.linear.x, second_command.angular.z),
        )

    def test_impossible_spacing_reports_a_correlated_failure(self):
        controller = make_controller()
        controller.robot_ids = [f'tb3_{index}' for index in range(10)]
        controller.robot_poses = {
            robot_id: Pose(x=0.0, y=0.0)
            for robot_id in controller.robot_ids
        }
        controller.robot_yaws = {
            robot_id: 0.0 for robot_id in controller.robot_ids
        }
        controller.formation_type = 'line'
        controller.spacing = 2.0
        controller.formation_offsets = controller._compute_formation_positions(
            'line', 10, 2.0
        )
        controller.movement_mode = formation.MovementMode.STATIC
        controller.current_task_id = 'unsafe-formation'
        controller.is_running = True

        snapshot = controller._prepare_assignment_locked()
        controller._compute_and_commit_assignment(snapshot)
        controller._control_step(None)

        self.assertIsNotNone(snapshot)
        self.assertFalse(controller.is_running)
        self.assertTrue(controller.placement_error)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('unsafe-formation', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertIn('collision-free placement', status['error'])


class FormationPlacementTests(unittest.TestCase):
    SHAPES = (
        'triangle', 'square', 'circle', 'line', 'v_formation', 'diamond',
    ) + tuple(sorted(formation.LETTERS))
    LIVE_SPACINGS = (0.35, 0.55, 0.7, 1.0)
    FRONTEND_SPACINGS = tuple(
        round(0.35 + index * 0.05, 2) for index in range(34)
    )

    def test_crossing_letters_stay_compact(self):
        controller = make_controller()
        slots = controller._compute_formation_positions('X', 10, 1.0)

        self.assertLessEqual(max(abs(x) for x, _ in slots), 2.1)
        self.assertLessEqual(max(abs(y) for _, y in slots), 3.1)
        closest = min(
            math.hypot(first[0] - second[0], first[1] - second[1])
            for index, first in enumerate(slots)
            for second in slots[index + 1:]
        )
        self.assertGreaterEqual(closest + 1e-9, 1.0)

    def test_robot_inside_clearance_buffer_can_only_route_outward(self):
        start = (1.605, -0.979)
        outward_target = (1.4356, -0.7314)
        self.assertTrue(straight_route_is_safe(
            start,
            outward_target,
            SPAWN_SETTINGS['spawn_obstacle_clearance'],
            SPAWN_SETTINGS['spawn_exclusion_zones'],
            'swarm_arena',
        ))

        # The original S slot was technically farther from the wall, but its
        # route was almost tangent. The Burger arced into the wall while it
        # turned, so this is not treated as a safe escape.
        self.assertFalse(straight_route_is_safe(
            start,
            (0.508, -1.528),
            SPAWN_SETTINGS['spawn_obstacle_clearance'],
            SPAWN_SETTINGS['spawn_exclusion_zones'],
            'swarm_arena',
        ))

        # This target is clear on the other side of the diagonal wall, but
        # reaching it would pass through the wall instead of leaving the
        # planning buffer on the same side.
        self.assertFalse(straight_route_is_safe(
            start,
            (1.2259, -1.8301),
            SPAWN_SETTINGS['spawn_obstacle_clearance'],
            SPAWN_SETTINGS['spawn_exclusion_zones'],
            'swarm_arena',
        ))

    def test_live_ten_robot_s_plan_is_safe_and_bounded(self):
        controller = make_controller()
        robot_positions = [
            (-1.4624, -0.9297),
            (-0.4629, -0.9295),
            (0.5371, -0.9295),
            (1.5369, -0.9294),
            (-1.4635, 0.0708),
            (-0.4638, 0.0709),
            (0.5361, 0.0710),
            (1.5358, 0.0711),
            (-1.4643, 1.0712),
            (-0.4646, 1.0713),
        ]
        preferred_center = (
            sum(x for x, _ in robot_positions) / len(robot_positions),
            sum(y for _, y in robot_positions) / len(robot_positions),
        )
        offsets = controller._compute_formation_positions('S', 10, 0.55)

        started_at = time.perf_counter()
        plan = find_safe_formation_plan(
            offsets,
            preferred_center,
            robot_positions,
            10.0,
            0.35,
            SPAWN_SETTINGS['spawn_obstacle_clearance'],
            SPAWN_SETTINGS['spawn_search_step'],
            SPAWN_SETTINGS['spawn_exclusion_zones'],
            'swarm_arena',
            maximum_center_candidates=SPAWN_SETTINGS[
                'formation_search_limit'
            ],
        )
        elapsed = time.perf_counter() - started_at

        self.assertIsNotNone(plan)
        self.assertLess(elapsed, 1.5)
        _, targets, assignment = plan
        self.assertTrue(all(
            straight_route_is_safe(
                robot_positions[robot_index], targets[slot_index],
                SPAWN_SETTINGS['spawn_obstacle_clearance'],
                SPAWN_SETTINGS['spawn_exclusion_zones'],
                'swarm_arena',
            )
            for robot_index, slot_index in enumerate(assignment)
        ))

    def test_route_through_diagonal_wall_is_rejected(self):
        self.assertFalse(straight_route_is_safe(
            (1.0, -1.0),
            (1.0, -2.0),
            SPAWN_SETTINGS['spawn_obstacle_clearance'],
            SPAWN_SETTINGS['spawn_exclusion_zones'],
            'swarm_arena',
        ))

    def test_clear_assignment_keeps_the_preferred_pattern_center(self):
        robots = [(-2.0, -2.0), (-2.0, -1.0)]
        offsets = [(-2.0, -2.0), (1.0, 0.0)]
        zones = [{
            'shape': 'circle', 'x': 0.0, 'y': 0.0, 'radius': 0.3,
        }]

        plan = find_safe_formation_plan(
            offsets,
            (0.0, 0.0),
            robots,
            10.0,
            0.35,
            0.2,
            0.1,
            zones,
            'swarm_arena',
        )

        self.assertIsNotNone(plan)
        center, targets, assignment = plan
        self.assertEqual(center, (0.0, 0.0))
        self.assertEqual(assignment, [1, 0])
        self.assertTrue(all(
            straight_route_is_safe(
                robots[robot_index], targets[slot_index],
                0.2, zones, 'swarm_arena',
            )
            for robot_index, slot_index in enumerate(assignment)
        ))

    def test_representative_static_plans_have_clear_routes(self):
        controller = make_controller()
        cases = (
            (5, 'square', 0.7),
            (7, 'A', 0.7),
            (8, 'V', 0.55),
            (9, 'diamond', 0.55),
            (10, 'S', 0.55),
        )

        for robot_count, shape, spacing in cases:
            with self.subTest(
                robot_count=robot_count, shape=shape, spacing=spacing
            ):
                columns = int(math.ceil(math.sqrt(robot_count)))
                rows = int(math.ceil(robot_count / columns))
                robot_positions = [
                    (
                        (index % columns - (columns - 1) / 2.0) * 0.6,
                        (index // columns - (rows - 1) / 2.0) * 0.6,
                    )
                    for index in range(robot_count)
                ]
                preferred_center = (
                    sum(x for x, _ in robot_positions) / robot_count,
                    sum(y for _, y in robot_positions) / robot_count,
                )
                offsets = controller._compute_formation_positions(
                    shape, robot_count, spacing
                )
                plan = find_safe_formation_plan(
                    offsets,
                    preferred_center,
                    robot_positions,
                    10.0,
                    0.35,
                    SPAWN_SETTINGS['spawn_obstacle_clearance'],
                    SPAWN_SETTINGS['spawn_search_step'],
                    SPAWN_SETTINGS['spawn_exclusion_zones'],
                    'swarm_arena',
                )

                self.assertIsNotNone(plan)
                _, targets, assignment = plan
                for robot_index, slot_index in enumerate(assignment):
                    self.assertTrue(straight_route_is_safe(
                        robot_positions[robot_index],
                        targets[slot_index],
                        SPAWN_SETTINGS['spawn_obstacle_clearance'],
                        SPAWN_SETTINGS['spawn_exclusion_zones'],
                        'swarm_arena',
                    ))

    def test_every_live_shape_count_and_spacing_has_a_safe_placement(self):
        controller = make_controller()
        zones = SPAWN_SETTINGS['spawn_exclusion_zones']

        for spacing in self.LIVE_SPACINGS:
            for robot_count in range(1, 11):
                for shape in self.SHAPES:
                    with self.subTest(
                        spacing=spacing,
                        robot_count=robot_count,
                        shape=shape,
                    ):
                        offsets = controller._compute_formation_positions(
                            shape, robot_count, spacing
                        )
                        center = find_safe_formation_center(
                            offsets,
                            (0.0, 0.0),
                            10.0,
                            0.35,
                            SPAWN_SETTINGS['spawn_obstacle_clearance'],
                            SPAWN_SETTINGS['spawn_search_step'],
                            zones,
                            'swarm_arena',
                        )
                        self.assertIsNotNone(center)
                        targets = [
                            (center[0] + x, center[1] + y)
                            for x, y in offsets
                        ]
                        self.assertTrue(formation_targets_are_safe(
                            targets,
                            10.0,
                            0.35,
                            SPAWN_SETTINGS['spawn_obstacle_clearance'],
                            zones,
                            'swarm_arena',
                        ))

                        if robot_count > 1:
                            closest = min(
                                math.hypot(
                                    first[0] - second[0],
                                    first[1] - second[1],
                                )
                                for index, first in enumerate(targets)
                                for second in targets[index + 1:]
                            )
                            self.assertGreaterEqual(
                                closest + 1e-9, spacing
                            )

    def test_every_frontend_case_is_safe_or_cleanly_rejected(self):
        """Cover every spacing value accepted by the primary task panel."""
        controller = make_controller()
        zones = SPAWN_SETTINGS['spawn_exclusion_zones']

        for spacing in self.FRONTEND_SPACINGS:
            for robot_count in range(1, 11):
                for shape in self.SHAPES:
                    with self.subTest(
                        spacing=spacing,
                        robot_count=robot_count,
                        shape=shape,
                    ):
                        offsets = controller._compute_formation_positions(
                            shape, robot_count, spacing
                        )
                        if robot_count > 1:
                            closest = min(
                                math.hypot(
                                    first[0] - second[0],
                                    first[1] - second[1],
                                )
                                for index, first in enumerate(offsets)
                                for second in offsets[index + 1:]
                            )
                            self.assertGreaterEqual(
                                closest + 1e-9, spacing
                            )

                        center = find_safe_formation_center(
                            offsets,
                            (0.0, 0.0),
                            10.0,
                            0.35,
                            SPAWN_SETTINGS['spawn_obstacle_clearance'],
                            SPAWN_SETTINGS['spawn_search_step'],
                            zones,
                            'swarm_arena',
                        )

                        # Large combinations cannot physically fit inside the
                        # arena. The controller reports those as failed tasks;
                        # every accepted placement still has to be safe.
                        if center is None:
                            continue
                        targets = [
                            (center[0] + x, center[1] + y)
                            for x, y in offsets
                        ]
                        self.assertTrue(formation_targets_are_safe(
                            targets,
                            10.0,
                            0.35,
                            SPAWN_SETTINGS['spawn_obstacle_clearance'],
                            zones,
                            'swarm_arena',
                        ))


class FormationConvergenceTests(unittest.TestCase):
    FAILED_LIVE_CASES = (
        (5, 'square', 0.7),
        (7, 'A', 0.7),
        (8, 'V', 0.55),
        (9, 'diamond', 0.55),
        (10, 'S', 0.55),
    )

    def test_slots_are_centered_and_keep_requested_clearance(self):
        controller = make_controller()

        for robot_count, shape, spacing in self.FAILED_LIVE_CASES:
            with self.subTest(robot_count=robot_count, shape=shape):
                slots = controller._compute_formation_positions(
                    shape, robot_count, spacing
                )
                self.assertEqual(len(slots), robot_count)
                self.assertAlmostEqual(
                    sum(x for x, _ in slots) / robot_count, 0.0, places=9
                )
                self.assertAlmostEqual(
                    sum(y for _, y in slots) / robot_count, 0.0, places=9
                )
                closest = min(
                    math.hypot(a[0] - b[0], a[1] - b[1])
                    for index, a in enumerate(slots)
                    for b in slots[index + 1:]
                )
                self.assertGreaterEqual(closest + 1e-9, spacing)

    def test_previously_failing_shapes_converge_inside_live_timeout(self):
        for robot_count, shape, spacing in self.FAILED_LIVE_CASES:
            with self.subTest(robot_count=robot_count, shape=shape):
                controller, targets = make_closed_loop_controller(
                    robot_count, shape, spacing
                )
                closest_seen = float('inf')

                # The live acceptance timeout is 75 simulated seconds.
                for _ in range(1500):
                    controller._control_step(None)
                    dt = 1.0 / controller.control_rate
                    for robot_id in controller.robot_ids:
                        command = controller.cmd_vel_pubs[
                            robot_id
                        ].messages[-1]
                        yaw = formation.normalize_angle(
                            controller.robot_yaws[robot_id]
                            + command.angular.z * dt
                        )
                        pose = controller.robot_poses[robot_id]
                        pose.position.x += (
                            command.linear.x * math.cos(yaw) * dt
                        )
                        pose.position.y += (
                            command.linear.x * math.sin(yaw) * dt
                        )
                        controller.robot_yaws[robot_id] = yaw

                    positions = [
                        (
                            controller.robot_poses[rid].position.x,
                            controller.robot_poses[rid].position.y,
                        )
                        for rid in controller.robot_ids
                    ]
                    closest_seen = min(
                        closest_seen,
                        min(
                            math.hypot(a[0] - b[0], a[1] - b[1])
                            for index, a in enumerate(positions)
                            for b in positions[index + 1:]
                        ),
                    )
                    if (
                        controller.formation_state
                        == formation.FormationState.FORMED
                    ):
                        break

                self.assertEqual(
                    controller.formation_state,
                    formation.FormationState.FORMED,
                )
                errors = [
                    math.hypot(
                        controller.robot_poses[rid].position.x
                        - targets[controller.assignments[rid]][0],
                        controller.robot_poses[rid].position.y
                        - targets[controller.assignments[rid]][1],
                    )
                    for rid in controller.robot_ids
                ]
                self.assertLessEqual(max(errors), 0.1)
                self.assertGreater(closest_seen, 0.3)


class FormationLifecycleTests(unittest.TestCase):
    def test_orderly_shutdown_zeroes_every_formation_command(self):
        controller = make_controller()
        controller.is_running = True

        controller._shutdown()

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        for publisher in controller.cmd_vel_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

    def test_stale_odometry_is_reported_after_the_start_grace_period(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_timeout_wall_s = 0.5
        controller.odom_received_at = {'tb3_0': 10.0}

        self.assertEqual(
            ['tb3_0'], controller._stale_odometry(['tb3_0'], now=10.6)
        )
        self.assertEqual(
            [], controller._stale_odometry(['tb3_0'], now=10.4)
        )

    def test_never_received_odometry_gets_a_bounded_startup_grace(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_initialization_timeout_wall_s = 5.0
        controller.odom_received_at = {'tb3_0': None}

        self.assertEqual(
            (['tb3_0'], []),
            controller._odometry_readiness(['tb3_0'], now=14.9),
        )
        self.assertEqual(
            ([], ['tb3_0']),
            controller._odometry_readiness(['tb3_0'], now=15.1),
        )

    def test_previously_live_odometry_keeps_the_strict_stale_timeout(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_timeout_wall_s = 0.75
        controller.odom_initialization_timeout_wall_s = 5.0
        controller.odom_received_at = {'tb3_0': 10.1}

        self.assertEqual(
            ([], ['tb3_0']),
            controller._odometry_readiness(['tb3_0'], now=10.9),
        )

    def test_control_waits_stopped_for_the_complete_initial_odometry_set(self):
        controller = make_controller()
        controller.is_running = True
        controller.task_started_at = 10.0
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.robot_poses = {'tb3_0': Pose(), 'tb3_1': None}
        controller.robot_yaws = {'tb3_0': 0.0, 'tb3_1': 0.0}
        controller.odom_received_at = {'tb3_0': 11.5, 'tb3_1': None}
        controller.cmd_vel_pubs['tb3_1'] = FakePublisher()

        with mock.patch.object(formation.time, 'monotonic', return_value=12.0):
            controller._control_step(None)

        self.assertTrue(controller.is_running)
        self.assertEqual(
            formation.FormationState.FORMING, controller.formation_state
        )
        for publisher in controller.cmd_vel_pubs.values():
            self.assertEqual(1, len(publisher.messages))
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)

        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('forming', status['state'])
        self.assertEqual(['tb3_1'], status['waiting_for_odometry'])
        self.assertEqual([], status['stale_odometry'])

    def test_control_fails_closed_when_initial_odometry_never_arrives(self):
        controller = make_controller()
        controller.is_running = True
        controller.task_started_at = 10.0
        controller.odom_received_at = {'tb3_0': None}

        with mock.patch.object(formation.time, 'monotonic', return_value=15.1):
            controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED, controller.formation_state
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(['tb3_0'], status['stale_odometry'])
        self.assertIn('tb3_0', status['error'])

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
