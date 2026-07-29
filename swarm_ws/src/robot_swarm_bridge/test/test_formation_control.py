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
        self.connections = 1

    def publish(self, message):
        self.messages.append(message)

    def unregister(self):
        self.unregistered = True

    def get_num_connections(self):
        return self.connections


class SafetyRequestPublisher(FakePublisher):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller

    def publish(self, message):
        super().publish(message)
        request = json.loads(message.data)
        self.controller._safety_stop_ack_cb(String(data=json.dumps({
            'request_id': request['request_id'],
            'accepted': True,
            'supervisor_latched': True,
            'supervisor_will_remain_active': True,
            'valid_request': True,
            'zero_publications_confirmed': True,
        })))


class FakeSafetyLane:
    def __init__(self, _label, _error_logger=None):
        self.available = True
        self.closed = False

    def submit(self, callback):
        if self.closed:
            return False
        callback()
        return True

    def close(self):
        self.closed = True
        self.available = False


class FlakyPublisher(FakePublisher):
    def __init__(self, failures=1):
        super().__init__()
        self.failures_remaining = failures
        self.calls = 0

    def publish(self, message):
        self.calls += 1
        if self.failures_remaining > 0:
            self.failures_remaining -= 1
            raise RuntimeError('publisher disconnected')
        super().publish(message)


class BlockingPublisher(FakePublisher):
    def __init__(self, release):
        super().__init__()
        self.entered = threading.Event()
        self.release = release

    def publish(self, message):
        self.entered.set()
        self.release.wait(1.0)
        super().publish(message)


class FirstPublishBlockingPublisher(FakePublisher):
    def __init__(self, release):
        super().__init__()
        self.entered = threading.Event()
        self.release = release
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            call_index = self.calls
            self.calls += 1
        if call_index == 0:
            self.entered.set()
            self.release.wait(1.0)
        super().publish(message)


class LateCompensationFailingPublisher(FakePublisher):
    def __init__(self, release):
        super().__init__()
        self.entered = threading.Event()
        self.release = release
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            call_index = self.calls
            self.calls += 1
        if call_index == 0:
            self.entered.set()
            self.release.wait(1.0)
        elif call_index == 2:
            raise RuntimeError('late compensation socket rejected zero')
        super().publish(message)


class LateCompensationBlockingPublisher(FakePublisher):
    def __init__(self, release_motion, release_compensation):
        super().__init__()
        self.motion_entered = threading.Event()
        self.compensation_entered = threading.Event()
        self.release_motion = release_motion
        self.release_compensation = release_compensation
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            call_index = self.calls
            self.calls += 1
        if call_index == 0:
            self.motion_entered.set()
            self.release_motion.wait(1.0)
        elif call_index == 2:
            self.compensation_entered.set()
            self.release_compensation.wait(1.0)
        super().publish(message)


class StatusReassertionBlockingPublisher(FakePublisher):
    def __init__(self, release_regular, release_reassertion):
        super().__init__()
        self.regular_entered = threading.Event()
        self.reassertion_entered = threading.Event()
        self.release_regular = release_regular
        self.release_reassertion = release_reassertion
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            call_index = self.calls
            self.calls += 1
        if call_index == 0:
            self.regular_entered.set()
            self.release_regular.wait(1.0)
        elif call_index == 2:
            self.reassertion_entered.set()
            self.release_reassertion.wait(1.0)
        super().publish(message)


class FakeResource:
    def __init__(self):
        self.closed = False

    def unregister(self):
        self.closed = True

    def shutdown(self):
        self.closed = True


class ContentionAwareRLock:
    """RLock that exposes when another thread has reached its boundary."""

    def __init__(self):
        self._lock = threading.RLock()
        self._metadata_lock = threading.Lock()
        self._owner = None
        self._depth = 0
        self.contended = threading.Event()

    def __enter__(self):
        thread_id = threading.get_ident()
        with self._metadata_lock:
            if self._owner not in (None, thread_id):
                self.contended.set()
        self._lock.acquire()
        with self._metadata_lock:
            if self._owner == thread_id:
                self._depth += 1
            else:
                self._owner = thread_id
                self._depth = 1
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        with self._metadata_lock:
            self._depth -= 1
            if self._depth == 0:
                self._owner = None
        self._lock.release()


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


class AdaptiveThreatAvoidance(PassThroughAvoidance):
    def __init__(self, repulsion_x=0.04, repulsion_y=0.0):
        self.repulsion = Point(x=repulsion_x, y=repulsion_y)

    def compute_threat_level(self):
        return 1.0

    def compute_repulsion_force(self):
        return self.repulsion


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
    controller._emergency_stop_generation = 0
    controller._latest_emergency_true_generation = 0
    controller._emergency_true_pending = set()
    controller.current_task_id = None
    controller._cancelled_task_ids = {}
    controller.formation_state = formation.FormationState.IDLE

    controller.robot_ids = ['tb3_0']
    controller.robot_count = 1
    controller.robot_poses = {'tb3_0': Pose()}
    controller.robot_yaws = {'tb3_0': 0.0}
    controller.invalid_robot_poses = ()
    controller.odom_received_at = {'tb3_0': None}
    controller.odom_confirmed_for_task = {'tb3_0': False}
    controller.odom_timeout_wall_s = 0.75
    controller.odom_initialization_timeout_wall_s = 10.0
    controller.task_started_at = None
    controller.stale_odometry = []
    controller.waiting_for_odometry = []
    controller.formation_offsets = [(1.0, 0.0)]
    controller.assignments = {'tb3_0': 0}
    controller.route_waypoints = {}
    controller.route_waypoint_indices = {}
    controller.route_batches = []
    controller.route_batch_index = 0
    controller.route_stall_timeout = 20.0
    controller.route_progress_epsilon = 0.20
    controller._route_progress_token = None
    controller._route_progress_best = {}
    controller._route_stall_duration = {}
    controller._active_route_batch_ids = ()
    controller._last_route_stall_detail = None
    controller._last_live_safety_failure = None
    controller._live_safety_failure_history = []
    controller.active_placement_plan = None
    controller.assignment_pending = False
    controller._assignment_generation = 0
    controller.live_replan_limit = 2
    controller._live_replan_attempts = 0
    controller._stop_publication_attempt = 0
    controller._last_stop_publication = None
    controller._stop_publication_lock = threading.RLock()
    controller._stop_result_lock = threading.RLock()
    controller._emergency_stop_condition = threading.Condition(
        controller._stop_result_lock
    )
    controller._command_publisher_sequence = 0
    controller._command_publisher_generations = {}
    controller._stop_publication_debts = {}
    controller._safety_fallback_lanes = {}
    controller._safety_fallback_lane_factory = FakeSafetyLane
    controller._shutdown_publisher_snapshot = ()
    controller._shutdown_started = False
    controller._shutdown_publication = None
    controller._shutdown_status_delivery = None
    controller._shutdown_supervisor_delivery = None
    controller._safety_stop_ack_lock = threading.Lock()
    controller._safety_stop_ack_waiters = {}
    controller._motion_publish_inflight = set()
    controller._shutdown_motion_inflight = frozenset()
    controller.assignment_pose_drift_tolerance = 0.02
    controller.assignment_yaw_drift_tolerance = 0.05
    controller.assignment_settle_time_wall_s = 0.5
    controller.assignment_settle_timeout_wall_s = 5.0
    controller.assignment_settle_position_tolerance = 0.01
    controller.assignment_settle_yaw_tolerance = 0.03
    controller._assignment_settle_anchor = {}
    controller._assignment_settle_since = None
    controller._assignment_settle_deadline = None
    controller._assignment_settle_odom_deadline = None
    controller._assignment_settle_ready_at = None
    controller._assignment_planning_phase = 'idle'
    controller._assignment_settle_waiting = ()
    controller._assignment_worker_stop = threading.Event()
    controller._assignment_worker_wakeup = threading.Event()
    controller._assignment_worker = None
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
    controller.invalid_model_poses = ()
    controller.invalid_avoidance_limits = ()
    controller.placement_error = None

    controller.formation_type = 'line'
    controller.movement_mode = formation.MovementMode.MOVING
    controller.spacing = 1.0
    controller.centroid_speed = 0.1
    controller.path_radius = 2.5
    controller.minimum_safe_path_radius = 0.25
    controller.orbit_sample_step = 0.04
    controller.route_tracking_margin = 0.02
    controller.route_waypoint_tolerance = 0.08
    controller.centroid_path = formation.CentroidPath.CIRCULAR
    controller.centroid_waypoints = []
    controller.centroid_x = 0.0
    controller.centroid_y = 0.0
    controller.centroid_heading = 0.0
    controller.formation_heading = 0.0
    controller.centroid_time = 99.0
    controller.current_waypoint_idx = 3
    controller.path_center_x = 0.0
    controller.path_center_y = 0.0
    controller.effective_path_radius = controller.path_radius
    controller.orbit_path_validated = False
    controller.orbit_validation_samples = 0
    controller.orbit_validation_live_models = 0
    controller.orbit_radius_adapted = False
    controller._initial_formation_acquired = False
    controller._maximum_position_error = 0.0
    controller.control_rate = 20.0
    controller.max_linear_vel = 0.22
    controller.max_angular_vel = 1.5
    controller._motion_limits_valid = True

    controller.pid_linear = {'tb3_0': FakePid()}
    controller.pid_angular = {'tb3_0': FakePid()}
    controller.cmd_vel_pubs = {'tb3_0': FakePublisher()}
    controller.status_pub = FakePublisher()
    controller.safety_stop_request_pub = SafetyRequestPublisher(controller)
    controller._register_safety_fallback_lane(
        'supervisor-request', controller.safety_stop_request_pub
    )
    controller._odom_subs = {}
    controller.avoidance = {}
    with controller.lock:
        controller._refresh_shutdown_publisher_snapshot_locked()
    return controller


def make_two_robot_controller():
    """Extend the small controller fixture without starting ROS resources."""
    controller = make_controller()
    controller.robot_ids = ['tb3_0', 'tb3_1']
    controller.robot_count = 2
    controller.robot_poses = {
        'tb3_0': Pose(x=0.0, y=0.0),
        'tb3_1': Pose(x=1.0, y=0.0),
    }
    controller.robot_yaws = {'tb3_0': 0.0, 'tb3_1': 0.0}
    controller.odom_received_at = {'tb3_0': None, 'tb3_1': None}
    controller.odom_confirmed_for_task = {
        'tb3_0': False,
        'tb3_1': False,
    }
    controller.formation_offsets = [(-0.5, 0.0), (0.5, 0.0)]
    controller.assignments = {'tb3_0': 0, 'tb3_1': 1}
    controller._slot_reached = {'tb3_0': False, 'tb3_1': False}
    controller.pid_linear['tb3_1'] = FakePid()
    controller.pid_angular['tb3_1'] = FakePid()
    second_publisher = FakePublisher()
    controller.cmd_vel_pubs['tb3_1'] = second_publisher
    controller._register_safety_fallback_lane(
        'tb3_1', second_publisher
    )
    with controller.lock:
        controller._refresh_shutdown_publisher_snapshot_locked()
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
    controller.active_placement_plan = {
        'kind': 'static',
        'arena_size': controller.arena_size,
        'arena_margin': controller.arena_margin,
        'obstacle_clearance': controller.formation_obstacle_clearance,
        'exclusion_zones': (),
        'arena_profile': controller.arena_profile,
    }
    controller.route_waypoints = {
        robot_id: [targets[controller.assignments[robot_id]]]
        for robot_id in controller.robot_ids
    }
    controller.route_waypoint_indices = {
        robot_id: 0 for robot_id in controller.robot_ids
    }
    controller._slot_reached = {
        robot_id: False for robot_id in controller.robot_ids
    }
    controller._settled_duration = 0.0

    controller.control_rate = 20.0
    controller.max_linear_vel = 0.22
    controller.max_angular_vel = 1.5
    controller.pid_linear = {
        robot_id: formation.PIDController(0.6, 0.01, 0.15, 0.22)
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


def apply_production_spawn_pattern(controller, robot_count, pattern):
    """Use the same 0.6 m base layouts and safe translation as FleetManager."""

    spawn_spacing = 0.6
    poses = []
    if pattern == 'grid':
        columns = int(math.ceil(math.sqrt(robot_count)))
        rows = int(math.ceil(robot_count / float(columns)))
        x_offset = -(columns - 1) * spawn_spacing / 2.0
        y_offset = -(rows - 1) * spawn_spacing / 2.0
        poses = [
            (
                x_offset + (index % columns) * spawn_spacing,
                y_offset + (index // columns) * spawn_spacing,
                0.0,
            )
            for index in range(robot_count)
        ]
    elif pattern == 'circle':
        radius = (
            0.0 if robot_count == 1 else
            spawn_spacing / (2.0 * math.sin(math.pi / robot_count))
        )
        poses = [
            (
                radius * math.cos(2.0 * math.pi * index / robot_count),
                radius * math.sin(2.0 * math.pi * index / robot_count),
                formation.normalize_angle(
                    2.0 * math.pi * index / robot_count + math.pi
                ),
            )
            for index in range(robot_count)
        ]
    elif pattern == 'line':
        start_x = -(robot_count - 1) * spawn_spacing / 2.0
        poses = [
            (start_x + index * spawn_spacing, 0.0, 0.0)
            for index in range(robot_count)
        ]
    else:
        raise ValueError('unsupported production spawn pattern')

    center = find_safe_formation_center(
        [(x, y) for x, y, _yaw in poses],
        (0.0, 0.0),
        controller.arena_size,
        controller.arena_margin,
        controller.formation_obstacle_clearance,
        controller.formation_search_step,
        controller.spawn_exclusion_zones,
        controller.arena_profile,
        controller.model_poses,
    )
    if center is None:
        raise AssertionError('production spawn pattern does not fit')

    for robot_id, (x, y, yaw) in zip(controller.robot_ids, poses):
        controller.robot_poses[robot_id] = Pose(
            x=center[0] + x,
            y=center[1] + y,
        )
        controller.robot_yaws[robot_id] = yaw


class FormationAssignmentTests(unittest.TestCase):
    def test_assignment_worker_waits_for_the_stationary_pose_gate(self):
        controller = make_controller()
        clock = [100.0]
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 99.9
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = clock[0]
        controller._reset_assignment_settle_locked(now=clock[0])
        gate_checked = threading.Event()
        prepare_entered = threading.Event()
        plan_finished = threading.Event()
        prepare_after_settle = (
            controller._prepare_assignment_after_settle_locked
        )

        def observe_gate(*args, **kwargs):
            result = prepare_after_settle(*args, **kwargs)
            gate_checked.set()
            return result

        def prepare_snapshot(*_args, **_kwargs):
            prepare_entered.set()
            return {'generation': controller._assignment_generation}

        def compute_snapshot(_snapshot):
            controller.assignment_pending = False
            plan_finished.set()
            return True

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker
        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ), mock.patch.object(
            controller,
            '_prepare_assignment_after_settle_locked',
            side_effect=observe_gate,
        ), mock.patch.object(
            controller,
            '_prepare_assignment_locked',
            side_effect=prepare_snapshot,
        ), mock.patch.object(
            controller,
            '_compute_and_commit_assignment',
            side_effect=compute_snapshot,
        ):
            worker.start()
            try:
                controller._request_pending_assignment()
                self.assertTrue(gate_checked.wait(1.0))
                self.assertFalse(prepare_entered.is_set())
                self.assertTrue(controller._assignment_settle_anchor)

                gate_checked.clear()
                with controller.command_lock:
                    clock[0] = 100.51
                    controller.odom_received_at['tb3_0'] = clock[0]
                controller._request_pending_assignment()
                self.assertTrue(prepare_entered.wait(1.0))
                self.assertTrue(plan_finished.wait(1.0))
            finally:
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                worker.join(1.0)

        self.assertFalse(worker.is_alive())

    def test_last_first_odometry_does_not_run_the_planner_inline(self):
        controller = make_controller()
        controller.robot_poses['tb3_0'] = None
        controller.odom_received_at['tb3_0'] = None
        controller.assignment_pending = True
        controller.task_started_at = time.monotonic() - 1.0
        planner_entered = threading.Event()
        release_planner = threading.Event()
        callback_finished = threading.Event()

        def blocking_planner(_snapshot):
            planner_entered.set()
            release_planner.wait(2.0)
            return False

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker

        with mock.patch.object(
            controller,
            '_compute_and_commit_assignment',
            side_effect=blocking_planner,
        ):
            worker.start()
            next_sample = None
            try:
                controller._odom_cb(Odometry(), 'tb3_0')
                self.assertTrue(planner_entered.wait(1.0))

                def publish_next_sample():
                    controller._odom_cb(Odometry(), 'tb3_0')
                    callback_finished.set()

                next_sample = threading.Thread(target=publish_next_sample)
                next_sample.start()
                self.assertTrue(
                    callback_finished.wait(0.25),
                    'a route solve blocked the next odometry callback',
                )
                self.assertTrue(
                    controller.odom_confirmed_for_task['tb3_0']
                )
            finally:
                release_planner.set()
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                if next_sample is not None:
                    next_sample.join(1.0)
                worker.join(1.0)

        self.assertIsNotNone(next_sample)
        self.assertFalse(next_sample.is_alive())
        self.assertFalse(worker.is_alive())

    def test_assignment_worker_coalesces_wakeups_during_a_solve(self):
        controller = make_controller()
        controller.assignment_pending = True
        first_planner_entered = threading.Event()
        release_first_planner = threading.Event()
        second_plan_finished = threading.Event()
        calls = []

        def prepare_snapshot():
            return {'generation': len(calls) + 1}

        def compute_snapshot(snapshot):
            calls.append(snapshot['generation'])
            if len(calls) == 1:
                first_planner_entered.set()
                release_first_planner.wait(2.0)
            else:
                second_plan_finished.set()
            return True

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker

        with mock.patch.object(
            controller,
            '_prepare_assignment_locked',
            side_effect=prepare_snapshot,
        ), mock.patch.object(
            controller,
            '_compute_and_commit_assignment',
            side_effect=compute_snapshot,
        ):
            worker.start()
            try:
                controller._request_pending_assignment()
                self.assertTrue(first_planner_entered.wait(1.0))
                controller._request_pending_assignment()
                controller._request_pending_assignment()
                controller._request_pending_assignment()
                release_first_planner.set()
                self.assertTrue(second_plan_finished.wait(1.0))
                time.sleep(0.05)
            finally:
                release_first_planner.set()
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual([1, 2], calls)

    def test_assignment_worker_survives_prepare_and_compute_exceptions(self):
        controller = make_controller()
        controller.assignment_pending = True
        controller.is_running = True
        prepare_failed = threading.Event()
        compute_failed = threading.Event()
        recovered = threading.Event()
        prepare_calls = []
        compute_calls = []

        def prepare_snapshot():
            prepare_calls.append(len(prepare_calls) + 1)
            controller._assignment_generation += 1
            generation = controller._assignment_generation
            if len(prepare_calls) == 1:
                prepare_failed.set()
                raise RuntimeError('broken snapshot')
            return {'generation': generation}

        def compute_snapshot(_snapshot):
            compute_calls.append(len(compute_calls) + 1)
            if len(compute_calls) == 1:
                compute_failed.set()
                raise RuntimeError('broken solver')
            controller.assignment_pending = False
            recovered.set()
            return True

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker

        with mock.patch.object(
            controller,
            '_prepare_assignment_locked',
            side_effect=prepare_snapshot,
        ), mock.patch.object(
            controller,
            '_compute_and_commit_assignment',
            side_effect=compute_snapshot,
        ):
            worker.start()
            try:
                controller._request_pending_assignment()
                self.assertTrue(prepare_failed.wait(1.0))
                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertFalse(controller.assignment_pending)
                self.assertTrue(worker.is_alive())
                deadline = time.monotonic() + 1.0
                while (
                    not controller.status_pub.messages
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.005)
                prepare_status = json.loads(
                    controller.status_pub.messages[-1].data
                )
                self.assertEqual('failed', prepare_status['state'])
                self.assertIn('snapshot preparation', prepare_status['error'])
                prepare_status_count = len(controller.status_pub.messages)

                with controller.command_lock:
                    controller.assignment_pending = True
                    controller.is_running = True
                controller._request_pending_assignment()
                self.assertTrue(compute_failed.wait(1.0))
                deadline = time.monotonic() + 1.0
                while controller.assignment_pending and time.monotonic() < deadline:
                    time.sleep(0.005)
                while (
                    len(controller.status_pub.messages) <= prepare_status_count
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.005)
                self.assertFalse(controller.assignment_pending)
                self.assertFalse(controller.is_running)
                self.assertTrue(worker.is_alive())
                compute_status = json.loads(
                    controller.status_pub.messages[-1].data
                )
                self.assertEqual('failed', compute_status['state'])
                self.assertIn('plan computation', compute_status['error'])

                with controller.command_lock:
                    controller.assignment_pending = True
                    controller.is_running = True
                controller._request_pending_assignment()
                self.assertTrue(recovered.wait(1.0))
            finally:
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual([1, 2, 3], prepare_calls)
        self.assertEqual([1, 2], compute_calls)

    def test_assignment_worker_failure_stops_all_publishers_best_effort(self):
        class RaisingPublisher:
            def __init__(self):
                self.calls = 0

            def publish(self, _message):
                self.calls += 1
                raise RuntimeError('publisher disconnected')

        controller = make_controller()
        first = FakePublisher()
        broken = RaisingPublisher()
        last = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': broken,
            'tb3_2': last,
        }
        controller.status_pub = RaisingPublisher()
        controller.assignment_pending = True
        controller.is_running = True
        prepare_entered = threading.Event()

        def broken_prepare():
            controller._assignment_generation += 1
            prepare_entered.set()
            raise RuntimeError('planner input failed')

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker
        with mock.patch.object(
            controller,
            '_prepare_assignment_locked',
            side_effect=broken_prepare,
        ):
            worker.start()
            try:
                controller._request_pending_assignment()
                self.assertTrue(prepare_entered.wait(1.0))
                deadline = time.monotonic() + 1.0
                while controller.assignment_pending and time.monotonic() < deadline:
                    time.sleep(0.005)
                self.assertFalse(controller.assignment_pending)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertTrue(worker.is_alive())
            finally:
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(1, broken.calls)
        self.assertEqual(1, controller.status_pub.calls)
        self.assertEqual(1, len(first.messages))
        self.assertEqual(1, len(last.messages))
        for command in (first.messages[-1], last.messages[-1]):
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
        self.assertIn(
            'Zero-velocity publication was not accepted for: tb3_1',
            controller.placement_error,
        )

    def test_stale_worker_exception_does_not_fail_a_new_generation(self):
        controller = make_controller()
        controller._assignment_generation = 8
        controller.is_running = True
        controller.formation_state = formation.FormationState.MOVING

        self.assertFalse(controller._fail_assignment_worker(
            RuntimeError('obsolete plan failed'),
            'plan computation',
            expected_generation=7,
        ))

        self.assertTrue(controller.is_running)
        self.assertEqual(
            formation.FormationState.MOVING,
            controller.formation_state,
        )
        self.assertIsNone(controller.placement_error)
        self.assertEqual([], controller.cmd_vel_pubs['tb3_0'].messages)

    def test_shutdown_invalidates_an_assignment_worker_result(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.assignment_pending = True
        controller.is_running = True
        settled_at = time.monotonic()
        first_sample_at = settled_at - 0.51
        controller.task_started_at = first_sample_at - 0.1
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = first_sample_at
        controller._reset_assignment_settle_locked(now=first_sample_at)
        controller._assignment_settle_readiness_locked(
            now=first_sample_at
        )
        controller.odom_received_at['tb3_0'] = settled_at
        planner_entered = threading.Event()
        release_planner = threading.Event()

        def blocking_routes(snapshot, _plan, targets):
            planner_entered.set()
            release_planner.wait(2.0)
            start = snapshot['robot_positions'][0]
            return [0], {snapshot['robot_ids'][0]: [start, targets[0]]}

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker

        with mock.patch.object(
            controller,
            '_plan_routes_to_targets',
            side_effect=blocking_routes,
        ):
            worker.start()
            shutdown = None
            try:
                controller._request_pending_assignment()
                self.assertTrue(planner_entered.wait(1.0))

                shutdown = threading.Thread(target=controller._shutdown)
                shutdown.start()
                time.sleep(0.05)
                release_planner.set()
            finally:
                release_planner.set()
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                if shutdown is not None:
                    shutdown.join(1.0)
                worker.join(1.0)

        self.assertIsNotNone(shutdown)
        self.assertFalse(shutdown.is_alive())
        self.assertFalse(worker.is_alive())
        self.assertFalse(controller.is_running)
        self.assertEqual({}, controller.assignments)
        self.assertFalse(controller.assignment_pending)
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)

    def test_shutdown_waits_past_the_old_timeout_for_the_worker(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.assignment_pending = True
        controller.is_running = True
        settled_at = time.monotonic()
        first_sample_at = settled_at - 0.51
        controller.task_started_at = first_sample_at - 0.1
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = first_sample_at
        controller._reset_assignment_settle_locked(now=first_sample_at)
        controller._assignment_settle_readiness_locked(
            now=first_sample_at
        )
        controller.odom_received_at['tb3_0'] = settled_at
        planner_entered = threading.Event()
        release_planner = threading.Event()

        def blocking_compute(_snapshot):
            planner_entered.set()
            release_planner.wait(3.0)
            return False

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker

        with mock.patch.object(
            controller,
            '_compute_and_commit_assignment',
            side_effect=blocking_compute,
        ):
            worker.start()
            shutdown = None
            try:
                controller._request_pending_assignment()
                self.assertTrue(planner_entered.wait(1.0))
                shutdown = threading.Thread(target=controller._shutdown)
                shutdown.start()
                shutdown.join(1.1)
                self.assertTrue(
                    shutdown.is_alive(),
                    'shutdown returned while the planner was still alive',
                )
                self.assertTrue(worker.is_alive())
                release_planner.set()
                shutdown.join(1.0)
            finally:
                release_planner.set()
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                if shutdown is not None:
                    shutdown.join(1.0)
                worker.join(1.0)

        self.assertIsNotNone(shutdown)
        self.assertFalse(shutdown.is_alive())
        self.assertFalse(worker.is_alive())
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)

    def test_open_moving_paths_keep_a_live_safe_placement_plan(self):
        for centroid_path in (
            formation.CentroidPath.LINEAR,
            formation.CentroidPath.WAYPOINTS,
        ):
            with self.subTest(path=centroid_path.value):
                controller = make_controller()
                controller.movement_mode = formation.MovementMode.MOVING
                controller.centroid_path = centroid_path
                controller.formation_offsets = [(0.0, 0.0)]
                if centroid_path == formation.CentroidPath.WAYPOINTS:
                    controller.centroid_waypoints = [{'x': 1.0, 'y': 0.0}]

                snapshot = controller._prepare_assignment_locked()

                self.assertIsNotNone(snapshot['placement_plan'])
                self.assertEqual(
                    centroid_path.value,
                    snapshot['placement_plan']['kind'],
                )
                self.assertTrue(
                    controller._compute_and_commit_assignment(snapshot),
                    controller.placement_error,
                )
                self.assertEqual(
                    centroid_path.value,
                    controller.active_placement_plan['kind'],
                )
                self.assertEqual(
                    {'tb3_0'}, set(controller.route_waypoints)
                )

    def test_start_plans_a_safe_orbit_after_stationary_odometry(self):
        controller = make_controller()
        controller.robot_poses['tb3_0'] = Pose(x=2.0, y=3.0)
        captured = {}

        original_routes = controller._plan_routes_to_targets

        def capture_routes(snapshot, plan, targets):
            captured['targets'] = tuple(targets)
            return original_routes(snapshot, plan, targets)

        with mock.patch.object(
            controller,
            '_plan_routes_to_targets',
            side_effect=capture_routes,
        ):
            controller._start_cb(String(data=json.dumps({
                'task_id': 'task-current',
                'formation_type': 'line',
                'movement_mode': 'moving',
                'spacing': 1.0,
            })))
            self.assertTrue(controller.assignment_pending)
            self.assertEqual(
                'waiting_for_stability',
                controller._assignment_planning_phase,
            )
            self.assertFalse(controller.orbit_path_validated)

            first_sample_at = controller.task_started_at + 0.01
            controller.odom_confirmed_for_task['tb3_0'] = True
            controller.odom_received_at['tb3_0'] = first_sample_at
            ready, timed_out, _ = (
                controller._assignment_settle_readiness_locked(
                    now=first_sample_at
                )
            )
            self.assertFalse(ready)
            self.assertFalse(timed_out)

            settled_at = (
                first_sample_at
                + controller.assignment_settle_time_wall_s
                + 0.01
            )
            controller.odom_received_at['tb3_0'] = settled_at
            ready, timed_out, _ = (
                controller._assignment_settle_readiness_locked(
                    now=settled_at
                )
            )
            self.assertTrue(ready)
            self.assertFalse(timed_out)
            controller._assignment_planning_phase = 'solving'
            snapshot = controller._prepare_assignment_locked(
                settle_gate_passed=True
            )
            self.assertTrue(
                controller._compute_and_commit_assignment(snapshot)
            )

        self.assertTrue(controller.orbit_path_validated)
        self.assertAlmostEqual(controller.centroid_heading, math.pi / 2.0)
        self.assertAlmostEqual(
            controller.centroid_x,
            controller.path_center_x + controller.effective_path_radius,
        )
        self.assertAlmostEqual(
            captured['targets'][0][0], controller.centroid_x
        )
        self.assertAlmostEqual(
            captured['targets'][0][1], controller.centroid_y
        )

        assigned_target = captured['targets'][0]
        controller._update_centroid(1.0 / 20.0)
        self.assertEqual(0.0, controller.centroid_time)
        self.assertEqual(
            assigned_target,
            controller._get_world_targets()[0],
        )

        controller._initial_formation_acquired = True
        controller._update_centroid(1.0 / 20.0)
        self.assertGreater(controller.centroid_time, 0.0)
        self.assertNotEqual(
            assigned_target,
            controller._get_world_targets()[0],
        )

        controller._maximum_position_error = 0.10
        held_time = controller.centroid_time
        controller._update_centroid(1.0 / 20.0)
        self.assertEqual(held_time, controller.centroid_time)

    def test_active_second_start_requires_a_confirmed_synchronous_stop(self):
        controller = make_controller()
        controller.current_task_id = 'first-task'
        controller.task_started_at = 50.0
        controller.is_running = True
        publisher = controller.cmd_vel_pubs['tb3_0']
        queued_after_stop = []
        queue_assignment = controller._queue_assignment_after_settle_locked

        def observe_queue():
            queued_after_stop.append(len(publisher.messages))
            return queue_assignment()

        with mock.patch.object(
            controller,
            '_queue_assignment_after_settle_locked',
            side_effect=observe_queue,
        ), mock.patch.object(
            controller,
            '_stop_all_robots',
            wraps=controller._stop_all_robots,
        ) as stop_all:
            controller._start_cb(String(data=json.dumps({
                'task_id': 'second-task',
                'formation_type': 'triangle',
            })))

        stop_all.assert_called_once_with('task_restart')
        self.assertEqual([1], queued_after_stop)
        self.assertEqual(1, len(publisher.messages))
        restart_zero = publisher.messages[0]
        self.assertEqual(0.0, restart_zero.linear.x)
        self.assertEqual(0.0, restart_zero.angular.z)
        self.assertEqual('second-task', controller.current_task_id)
        self.assertTrue(controller.is_running)
        self.assertTrue(controller.assignment_pending)
        self.assertEqual(
            'waiting_for_stability',
            controller._assignment_planning_phase,
        )

        failed_controller = make_controller()
        failed_controller.current_task_id = 'original-task'
        failed_controller.task_started_at = 75.0
        failed_controller.is_running = True
        broken_publisher = FlakyPublisher(failures=10)
        failed_controller.cmd_vel_pubs = {
            'tb3_0': broken_publisher
        }

        with mock.patch.object(
            failed_controller,
            '_queue_assignment_after_settle_locked',
        ) as queue_after_failure:
            failed_controller._start_cb(String(data=json.dumps({
                'task_id': 'must-not-activate',
                'formation_type': 'square',
            })))

        queue_after_failure.assert_not_called()
        self.assertEqual(
            'original-task', failed_controller.current_task_id
        )
        self.assertEqual(75.0, failed_controller.task_started_at)
        self.assertFalse(failed_controller.is_running)
        self.assertFalse(failed_controller.assignment_pending)
        self.assertEqual(
            formation.FormationState.FAILED,
            failed_controller.formation_state,
        )
        self.assertEqual(
            'task_restart',
            failed_controller._last_stop_publication['reason'],
        )
        self.assertFalse(
            failed_controller._last_stop_publication[
                'publication_confirmed'
            ]
        )

    def test_emergency_stop_is_not_blocked_and_rejects_stale_solution(self):
        controller = make_controller()
        solver_entered = threading.Event()
        release_solver = threading.Event()
        emergency_finished = threading.Event()

        def blocking_routes(snapshot, plan, targets):
            solver_entered.set()
            release_solver.wait(2.0)
            start = snapshot['robot_positions'][0]
            return [0], {snapshot['robot_ids'][0]: [start, targets[0]]}

        start_message = String(data=json.dumps({
            'task_id': 'task-current',
            'formation_type': 'line',
            'movement_mode': 'moving',
            'spacing': 1.0,
        }))

        with mock.patch.object(
            controller,
            '_plan_routes_to_targets',
            side_effect=blocking_routes,
        ):
            controller._start_cb(start_message)
            first_sample_at = controller.task_started_at + 0.01
            controller.odom_confirmed_for_task['tb3_0'] = True
            controller.odom_received_at['tb3_0'] = first_sample_at
            controller._assignment_settle_readiness_locked(
                now=first_sample_at
            )
            settled_at = (
                first_sample_at
                + controller.assignment_settle_time_wall_s
                + 0.01
            )
            controller.odom_received_at['tb3_0'] = settled_at
            ready, timed_out, _ = (
                controller._assignment_settle_readiness_locked(
                    now=settled_at
                )
            )
            self.assertTrue(ready)
            self.assertFalse(timed_out)
            with controller.command_lock:
                controller._assignment_planning_phase = 'solving'
                snapshot = controller._prepare_assignment_locked(
                    settle_gate_passed=True
                )

            start_thread = threading.Thread(
                target=controller._compute_and_commit_assignment,
                args=(snapshot,),
            )
            start_thread.start()
            self.assertTrue(solver_entered.wait(1.0))
            planning_status = json.loads(
                controller.status_pub.messages[-1].data
            )
            self.assertEqual('task-current', planning_status['task_id'])
            self.assertEqual('forming', planning_status['state'])
            self.assertEqual({}, planning_status['robot_assignments'])

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

    def test_control_waits_safely_while_a_replacement_plan_is_solving(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        first_snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(first_snapshot)
        )
        self.assertIsNotNone(controller.active_placement_plan)

        controller.is_running = True
        controller._initial_formation_acquired = False
        controller._publish_status = lambda *args, **kwargs: None
        controller._publish_markers = lambda *args, **kwargs: None

        # A shape or fleet update prepares under command_lock, then solves
        # outside it. Exercise the timer interleaving in that open window.
        controller.formation_offsets = [(-0.25, 0.0)]
        replacement = controller._prepare_assignment_locked(
            settle_gate_passed=True
        )
        self.assertTrue(controller.assignment_pending)
        self.assertEqual({}, controller.assignments)
        self.assertIsNone(controller.active_placement_plan)

        controller._control_loop(None)

        self.assertTrue(controller.is_running)
        self.assertIsNone(controller.placement_error)
        self.assertEqual(
            formation.FormationState.FORMING, controller.formation_state
        )
        waiting_command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertEqual(0.0, waiting_command.linear.x)
        self.assertEqual(0.0, waiting_command.angular.z)

        self.assertTrue(controller._compute_and_commit_assignment(replacement))
        self.assertTrue(controller.is_running)
        self.assertIsNone(controller.placement_error)
        self.assertEqual({'tb3_0': 0}, controller.assignments)

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
        controller.active_placement_plan = {
            'kind': 'static',
            'arena_size': controller.arena_size,
            'arena_margin': controller.arena_margin,
            'obstacle_clearance': controller.formation_obstacle_clearance,
            'exclusion_zones': controller.spawn_exclusion_zones,
            'arena_profile': controller.arena_profile,
        }

        with mock.patch.object(
            controller, '_next_route_batch_is_clear', return_value=False
        ):
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
        with mock.patch.object(
            controller, '_next_route_batch_is_clear', return_value=False
        ):
            controller._control_step(None)
            controller._control_step(None)

        second_command = controller.cmd_vel_pubs[second].messages[-1]
        self.assertNotEqual(
            (0.0, 0.0),
            (second_command.linear.x, second_command.angular.z),
        )

    def test_route_batches_move_a_future_slot_blocker_last(self):
        controller = make_controller()
        robot_ids = ('tb3_0', 'tb3_1')
        routes = {
            'tb3_0': [(-1.0, 0.0), (0.0, 0.0)],
            'tb3_1': [(0.0, -1.0), (0.0, 1.0)],
        }

        batches = controller._build_route_batches(
            robot_ids,
            ((-1.0, 0.0), (0.0, -1.0)),
            routes,
            0.30,
        )

        self.assertEqual([['tb3_1'], ['tb3_0']], batches)

    def test_next_route_batch_releases_after_the_live_crossing_clears(self):
        controller = make_controller()
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.assignments = {'tb3_0': 0, 'tb3_1': 1}
        controller.route_batches = [['tb3_0'], ['tb3_1']]
        controller.route_waypoints = {
            'tb3_0': [(1.0, 0.0)],
            'tb3_1': [(0.0, 1.0)],
        }
        controller.route_waypoint_indices = {'tb3_0': 0, 'tb3_1': 0}
        controller.active_placement_plan = {'slot_clearance': 0.30}
        targets = [(1.0, 0.0), (0.0, 1.0)]
        controller.robot_poses = {
            'tb3_0': Pose(x=-0.5, y=0.0),
            'tb3_1': Pose(x=0.0, y=-1.0),
        }

        self.assertFalse(controller._next_route_batch_is_clear(1, targets))

        controller.robot_poses['tb3_0'] = Pose(x=0.5, y=0.0)

        self.assertTrue(controller._next_route_batch_is_clear(1, targets))

    def test_early_route_release_keeps_both_batches_moving(self):
        controller, targets = make_closed_loop_controller(2, 'line', 1.0)
        first, second = controller.robot_ids
        controller.route_waypoints = {
            first: [targets[controller.assignments[first]]],
            second: [targets[controller.assignments[second]]],
        }
        controller.route_waypoint_indices = {first: 0, second: 0}
        controller.route_batches = [[first], [second]]
        controller.route_batch_index = 0
        controller.active_placement_plan = {
            'kind': 'static',
            'arena_size': controller.arena_size,
            'arena_margin': controller.arena_margin,
            'obstacle_clearance': controller.formation_obstacle_clearance,
            'exclusion_zones': controller.spawn_exclusion_zones,
            'arena_profile': controller.arena_profile,
            'slot_clearance': 0.30,
        }
        with mock.patch.object(
            controller, '_next_route_batch_is_clear', return_value=True
        ):
            controller._control_step(None)

        self.assertEqual(1, controller.route_batch_index)
        for robot_id in (first, second):
            command = controller.cmd_vel_pubs[robot_id].messages[-1]
            self.assertNotEqual(
                (0.0, 0.0),
                (command.linear.x, command.angular.z),
            )

    def test_route_batch_releases_inside_safe_hysteresis_band(self):
        controller, targets = make_closed_loop_controller(2, 'line', 1.0)
        first, second = controller.robot_ids
        controller.movement_mode = formation.MovementMode.STATIC
        controller.route_waypoints = {
            first: [targets[controller.assignments[first]]],
            second: [targets[controller.assignments[second]]],
        }
        controller.route_waypoint_indices = {first: 0, second: 0}
        controller.route_batches = [[first], [second]]
        controller.route_batch_index = 0
        controller.active_placement_plan = {
            'kind': 'static',
            'arena_size': controller.arena_size,
            'arena_margin': controller.arena_margin,
            'obstacle_clearance': controller.formation_obstacle_clearance,
            'exclusion_zones': controller.spawn_exclusion_zones,
            'arena_profile': controller.arena_profile,
        }

        first_target = targets[controller.assignments[first]]
        clearance_error = (
            controller.position_tolerance
            + controller.position_release_tolerance
        ) / 2.0
        controller.robot_poses[first].position.x = (
            first_target[0] + clearance_error
        )
        controller.robot_poses[first].position.y = first_target[1]

        controller._control_step(None)

        self.assertFalse(controller._slot_reached[first])
        self.assertEqual(1, controller.route_batch_index)
        second_command = controller.cmd_vel_pubs[second].messages[-1]
        self.assertNotEqual(
            (0.0, 0.0),
            (second_command.linear.x, second_command.angular.z),
        )

    def test_stalled_route_batch_replans_without_another_positive_batch(self):
        controller, targets = make_closed_loop_controller(2, 'line', 1.0)
        first, second = controller.robot_ids
        controller.movement_mode = formation.MovementMode.STATIC
        controller.route_stall_timeout = 0.1
        controller.route_waypoints = {
            first: [targets[controller.assignments[first]]],
            second: [targets[controller.assignments[second]]],
        }
        controller.route_waypoint_indices = {first: 0, second: 0}
        controller.route_batches = [[first], [second]]
        controller.route_batch_index = 0
        controller.active_placement_plan = {
            'kind': 'static',
            'arena_size': controller.arena_size,
            'arena_margin': controller.arena_margin,
            'obstacle_clearance': controller.formation_obstacle_clearance,
            'exclusion_zones': controller.spawn_exclusion_zones,
            'arena_profile': controller.arena_profile,
            'model_poses': {},
        }

        with mock.patch.object(
            controller,
            '_schedule_live_replan_locked',
            return_value=True,
        ) as schedule, mock.patch.object(
            controller,
            '_next_route_batch_is_clear',
            return_value=False,
        ):
            controller._control_step(None)
            controller._control_step(None)
            positive_batches = len(
                controller.cmd_vel_pubs[first].messages
            )
            controller._control_step(None)

        schedule.assert_called_once()
        detail = schedule.call_args.args[-1]
        self.assertEqual('route_stall', detail['gate'])
        self.assertEqual(first, detail['robot'])
        self.assertEqual(
            positive_batches,
            len(controller.cmd_vel_pubs[first].messages),
            'the stall edge published another positive command batch',
        )
        controller._live_replan_attempts = 1
        formation.FormationController._publish_status(
            controller, targets, 0.4
        )
        routing = json.loads(
            controller.status_pub.messages[-1].data
        )['routing']
        self.assertEqual([first], routing['active_batch'])
        self.assertEqual(1, routing['live_replan_attempts'])
        self.assertEqual(first, routing['last_stall']['robot'])
        self.assertEqual(0.2, routing['progress_threshold'])

    def test_small_route_steps_accumulate_into_useful_progress(self):
        controller = make_controller()
        controller.route_batches = [['tb3_0']]
        controller.route_waypoint_indices = {'tb3_0': 0}
        controller.route_stall_timeout = 0.25

        details = [
            controller._route_stall_detail(
                {'tb3_0'}, {'tb3_0': distance}, 0.05
            )
            for distance in (1.0, 0.92, 0.84, 0.76)
        ]

        self.assertEqual([None, None, None, None], details)
        self.assertEqual(0.0, controller._route_stall_duration['tb3_0'])
        self.assertAlmostEqual(
            0.76, controller._route_progress_best['tb3_0']
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

        snapshot = controller._prepare_assignment_locked(
            settle_gate_passed=True
        )
        controller._compute_and_commit_assignment(snapshot)
        controller._control_step(None)

        self.assertIsNotNone(snapshot)
        self.assertFalse(controller.is_running)
        self.assertTrue(controller.placement_error)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('unsafe-formation', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertIn('collision-free placement', status['error'])

    def test_impossible_moving_orbit_is_rejected_before_motion(self):
        controller = make_controller()
        controller.robot_ids = [f'tb3_{index}' for index in range(10)]
        controller.robot_poses = {
            robot_id: Pose() for robot_id in controller.robot_ids
        }
        controller.robot_yaws = {
            robot_id: 0.0 for robot_id in controller.robot_ids
        }
        controller.formation_type = 'line'
        controller.spacing = 2.0
        controller.formation_offsets = controller._compute_formation_positions(
            'line', 10, 2.0
        )
        controller.movement_mode = formation.MovementMode.MOVING

        snapshot = controller._prepare_assignment_locked()

        self.assertFalse(controller._compute_and_commit_assignment(snapshot))
        self.assertEqual({}, controller.assignments)
        self.assertFalse(controller.orbit_path_validated)
        self.assertIn('full orbit', controller.placement_error)

    def test_live_obstacle_change_during_solver_is_rejected_before_commit(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller.model_poses = {'moving_box': (4.0, 4.0, 0.0)}
        snapshot = controller._prepare_assignment_locked()
        original_routes = controller._plan_routes_to_targets

        def move_obstacle_after_planning(snapshot, plan, targets):
            routes = original_routes(snapshot, plan, targets)
            controller.model_poses = {
                'moving_box': (targets[0][0], targets[0][1], 0.0)
            }
            return routes

        with mock.patch.object(
            controller,
            '_plan_routes_to_targets',
            side_effect=move_obstacle_after_planning,
        ):
            committed = controller._compute_and_commit_assignment(snapshot)

        self.assertFalse(committed)
        self.assertEqual({}, controller.assignments)
        self.assertFalse(controller.orbit_path_validated)
        self.assertIn('live arena changed', controller.placement_error)
        self.assertEqual([], controller.cmd_vel_pubs['tb3_0'].messages)

    def test_live_pose_drift_queues_one_bounded_replan(self):
        controller = make_controller()
        controller.is_running = True
        controller.movement_mode = formation.MovementMode.STATIC
        controller.robot_poses['tb3_0'] = Pose(x=0.4355, y=0.0)
        controller.formation_offsets = [(0.0, 0.0)]
        controller.spawn_exclusion_zones = [{
            'name': 'flat_wall',
            'worlds': ['swarm_arena'],
            'shape': 'box',
            'x': -0.5,
            'y': 0.0,
            'width': 1.0,
            'height': 4.0,
        }]
        snapshot = controller._prepare_assignment_locked(
            settle_gate_passed=True
        )
        controller.robot_poses['tb3_0'] = Pose(x=0.2965, y=0.0)
        routes = {'tb3_0': [(0.4355, 0.0), (0.4355, 0.0)]}

        with mock.patch.object(
            controller,
            '_plan_routed_static_assignment',
            return_value=(
                (0.4355, 0.0), [(0.4355, 0.0)], [0], routes
            ),
        ), mock.patch.object(
            controller,
            '_placement_plan_is_live_safe',
            return_value=False,
        ):
            committed = controller._compute_and_commit_assignment(snapshot)

        self.assertFalse(committed)
        self.assertTrue(controller.is_running)
        self.assertIsNone(controller.placement_error)
        self.assertEqual(formation.FormationState.FORMING,
                         controller.formation_state)
        self.assertEqual(1, controller._live_replan_attempts)
        self.assertEqual(
            {
                'gate': 'assignment_pose_snapshot',
                'kind': 'drift',
                'robot': 'tb3_0',
                'position_drift': 0.139,
                'yaw_drift': 0.0,
                'stage': 'assignment_commit',
                'plan_attempt': 1,
            },
            controller._last_live_safety_failure,
        )
        self.assertTrue(controller.assignment_pending)
        self.assertEqual({}, controller.assignments)
        self.assertTrue(controller._assignment_worker_wakeup.is_set())
        self.assertEqual(
            'waiting_for_stability',
            controller._assignment_planning_phase,
        )
        ready, timed_out, _ = (
            controller._assignment_settle_readiness_locked()
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)

    def test_live_replan_is_not_queued_after_a_failed_stop_publication(self):
        controller = make_controller()
        controller.current_task_id = 'replan-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        first = FakePublisher()
        broken = FlakyPublisher(failures=10)
        controller.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': broken,
        }

        with mock.patch.object(
            controller, '_live_poses_can_be_replanned', return_value=True
        ):
            scheduled = controller._schedule_live_replan_locked(
                {}, {}, {}, {'gate': 'route', 'robot': 'tb3_0'}
            )

        self.assertFalse(scheduled)
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertFalse(controller._assignment_worker_wakeup.is_set())
        self.assertEqual(0, controller._live_replan_attempts)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertEqual(1, len(first.messages))
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('replan-stop-task', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertEqual(
            {
                'task_id': 'replan-stop-task',
                'attempt': 1,
                'reason': 'live_replan',
                'requested_count': 2,
                'accepted_count': 1,
                'failed_robots': ['tb3_1'],
                'unconfirmed_publishers': [{
                    'robot_id': 'tb3_1',
                    'publisher_generation': 3,
                    'task_id': 'replan-stop-task',
                    'reason': 'live_replan',
                    'first_attempt': 1,
                    'last_attempt': 1,
                }],
                'publication_confirmed': False,
            },
            status['stop_publication'],
        )

    def test_assignment_snapshot_correlates_position_and_yaw(self):
        controller = make_controller()
        snapshot = controller._prepare_assignment_locked()

        position_drift = controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (0.139, 0.0)},
            {'tb3_0': 0.0},
        )
        self.assertEqual('tb3_0', position_drift['robot'])
        self.assertEqual(0.139, position_drift['position_drift'])

        yaw_drift = controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (0.0, 0.0)},
            {'tb3_0': 0.051},
        )
        self.assertEqual(0.051, yaw_drift['yaw_drift'])

        self.assertIsNone(controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (0.01, 0.0)},
            {'tb3_0': 0.04},
        ))
        self.assertEqual('invalid', controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (float('nan'), 0.0)},
            {'tb3_0': 0.0},
        )['kind'])

    def test_assignment_snapshot_allows_only_bounded_spawn_yaw_settling(self):
        controller = make_controller()
        controller.assignment_yaw_drift_tolerance = 0.15
        snapshot = controller._prepare_assignment_locked()

        self.assertIsNone(controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (0.002, 0.0)},
            {'tb3_0': 0.149},
        ))
        drift = controller._assignment_pose_drift(
            snapshot,
            {'tb3_0': (0.002, 0.0)},
            {'tb3_0': 0.151},
        )
        self.assertEqual('tb3_0', drift['robot'])
        self.assertEqual(0.151, drift['yaw_drift'])

    def test_start_waits_for_fresh_cumulative_pose_settling(self):
        controller = make_controller()
        controller._start_cb(String(data=json.dumps({
            'task_id': 'settling-start',
            'formation_type': 'line',
        })))

        first_at = controller.task_started_at + 0.01
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=first_at)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        self.assertEqual('odometry', detail['reason'])

        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = first_at
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=first_at)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        self.assertEqual('first_sample', detail['reason'])

        controller.robot_poses['tb3_0'] = Pose(x=0.006, y=0.0)
        controller.odom_received_at['tb3_0'] = first_at + 0.20
        ready, _, _ = controller._assignment_settle_readiness_locked(
            now=first_at + 0.20
        )
        self.assertFalse(ready)

        # Each step is smaller than 1 cm, but the cumulative drift from the
        # anchor is not. The quiet window must restart here.
        controller.robot_poses['tb3_0'] = Pose(x=0.012, y=0.0)
        controller.odom_received_at['tb3_0'] = first_at + 0.40
        ready, _, detail = controller._assignment_settle_readiness_locked(
            now=first_at + 0.40
        )
        self.assertFalse(ready)
        self.assertEqual('movement', detail['reason'])
        self.assertEqual(['tb3_0'], detail['robots'])

        controller.odom_received_at['tb3_0'] = first_at + 0.70
        ready, _, _ = controller._assignment_settle_readiness_locked(
            now=first_at + 0.70
        )
        self.assertFalse(ready)

        settled_at = first_at + 0.91
        controller.odom_received_at['tb3_0'] = settled_at
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=settled_at)
        )
        self.assertTrue(ready)
        self.assertFalse(timed_out)
        self.assertEqual('stable', detail['reason'])
        self.assertEqual({}, controller.assignments)

    def test_assignment_settle_gate_normalizes_wrapped_yaw(self):
        controller = make_controller()
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 100.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.robot_yaws['tb3_0'] = math.pi - 0.01
        controller.odom_received_at['tb3_0'] = 100.1
        controller._reset_assignment_settle_locked(now=100.1)

        ready, timed_out, _ = (
            controller._assignment_settle_readiness_locked(now=100.1)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)

        controller.robot_yaws['tb3_0'] = -math.pi + 0.01
        controller.odom_received_at['tb3_0'] = 100.61
        ready, timed_out, _ = (
            controller._assignment_settle_readiness_locked(now=100.61)
        )
        self.assertTrue(ready)
        self.assertFalse(timed_out)

    def test_assignment_settle_timeout_fails_closed(self):
        controller = make_controller()
        controller.current_task_id = 'settle-timeout'
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 100.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = 100.1
        controller._reset_assignment_settle_locked(now=100.1)
        controller._assignment_settle_readiness_locked(now=100.1)

        timeout_at = (
            100.1 + controller.assignment_settle_timeout_wall_s + 0.01
        )
        controller.robot_poses['tb3_0'] = Pose(x=0.02, y=0.0)
        controller.odom_received_at['tb3_0'] = timeout_at
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=timeout_at)
        )
        self.assertFalse(ready)
        self.assertTrue(timed_out)
        self.assertTrue(
            controller._fail_assignment_settle_timeout_locked(detail)
        )

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual({}, controller.assignments)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('did not become stationary', controller.placement_error)
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(
            'assignment_pose_settle',
            status['routing']['last_live_safety_failure']['gate'],
        )
        self.assertEqual(
            'assignment_pose_settle',
            status['routing']['live_safety_failure_history'][-1]['gate'],
        )

    def test_settle_deadline_wins_over_late_stability(self):
        controller = make_controller()
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = 100.0
        controller._reset_assignment_settle_locked(now=100.0)

        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=100.0)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        self.assertEqual('first_sample', detail['reason'])

        deadline = controller._assignment_settle_deadline
        controller.odom_received_at['tb3_0'] = deadline
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=deadline)
        )

        self.assertFalse(ready)
        self.assertTrue(timed_out)
        self.assertEqual('quiet_window', detail['reason'])
        self.assertEqual(['tb3_0'], detail['robots'])

    def test_control_timer_preserves_predeadline_stability_for_late_worker(
        self,
    ):
        def seal_stability_without_running_the_worker():
            controller = make_controller()
            clock = [100.0]
            controller.current_task_id = 'timer-settle'
            controller.is_running = True
            controller.task_started_at = 99.0
            controller.movement_mode = formation.MovementMode.STATIC
            controller.formation_offsets = [(0.0, 0.0)]
            controller.odom_confirmed_for_task['tb3_0'] = True
            controller.odom_received_at['tb3_0'] = clock[0]
            controller._publish_status = lambda *args, **kwargs: None
            controller._publish_markers = lambda *args, **kwargs: None

            with mock.patch.object(
                formation.time,
                'monotonic',
                side_effect=lambda: clock[0],
            ):
                controller._queue_assignment_after_settle_locked()
                with mock.patch.object(
                    controller, '_request_pending_assignment'
                ) as wake_worker:
                    controller._control_step(None)
                    clock[0] = 100.51
                    controller.odom_received_at['tb3_0'] = clock[0]
                    controller._control_step(None)

            wake_worker.assert_called_once_with()
            self.assertEqual(
                100.51, controller._assignment_settle_ready_at
            )
            self.assertEqual(
                'waiting_for_stability',
                controller._assignment_planning_phase,
            )
            return controller

        stable_controller = seal_stability_without_running_the_worker()
        stable_controller.odom_received_at['tb3_0'] = 105.01
        with stable_controller.command_lock:
            snapshot, timed_out, detail = (
                stable_controller._prepare_assignment_after_settle_locked(
                    now=105.01
                )
            )

        self.assertIsNotNone(snapshot)
        self.assertFalse(timed_out)
        self.assertEqual('stable', detail['reason'])
        self.assertEqual(100.51, detail['ready_at'])

        moved_controller = seal_stability_without_running_the_worker()
        moved_controller.robot_poses['tb3_0'] = Pose(x=0.02, y=0.0)
        moved_controller.odom_received_at['tb3_0'] = 105.01
        with moved_controller.command_lock:
            snapshot, timed_out, detail = (
                moved_controller._prepare_assignment_after_settle_locked(
                    now=105.01
                )
            )
            failed_closed = (
                moved_controller._fail_assignment_settle_timeout_locked(
                    detail
                )
            )

        self.assertIsNone(snapshot)
        self.assertTrue(timed_out)
        self.assertEqual('movement', detail['reason'])
        self.assertEqual(['tb3_0'], detail['robots'])
        self.assertTrue(failed_closed)
        self.assertIsNone(moved_controller._assignment_settle_ready_at)
        self.assertFalse(moved_controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            moved_controller.formation_state,
        )

    def test_paused_control_loop_seals_stability_for_late_worker(self):
        controller = make_controller()
        clock = [100.0]
        controller.current_task_id = 'paused-settle'
        controller.is_running = True
        controller.is_paused = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = clock[0]

        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ):
            controller._queue_assignment_after_settle_locked()
            with mock.patch.object(
                controller, '_request_pending_assignment'
            ) as wake_worker:
                controller._control_loop(None)
                clock[0] = 100.51
                controller.odom_received_at['tb3_0'] = clock[0]
                controller._control_loop(None)

        wake_worker.assert_called_once_with()
        self.assertEqual(100.51, controller._assignment_settle_ready_at)
        self.assertEqual([], controller.cmd_vel_pubs['tb3_0'].messages)
        self.assertTrue(controller.is_paused)

        controller.odom_received_at['tb3_0'] = 105.01
        with controller.command_lock:
            snapshot, timed_out, detail = (
                controller._prepare_assignment_after_settle_locked(
                    now=105.01
                )
            )

        self.assertIsNotNone(snapshot)
        self.assertFalse(timed_out)
        self.assertEqual('stable', detail['reason'])
        self.assertEqual(100.51, detail['ready_at'])

    def test_start_gives_initial_odometry_its_own_deadline(self):
        controller = make_controller()
        controller._publish_markers = lambda *args, **kwargs: None
        clock = [0.0]
        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ):
            controller._start_cb(String(data=json.dumps({
                'task_id': 'late-first-odometry',
                'formation_type': 'line',
            })))
            self.assertIsNone(controller._assignment_settle_deadline)
            self.assertEqual(
                10.0, controller._assignment_settle_odom_deadline
            )

            clock[0] = 5.01
            controller._control_loop(None)
            self.assertTrue(controller.is_running)
            self.assertTrue(controller.assignment_pending)
            self.assertEqual(
                formation.FormationState.FORMING,
                controller.formation_state,
            )
            self.assertEqual(
                ('tb3_0',), controller._assignment_settle_waiting
            )

            clock[0] = 10.0
            controller._control_loop(None)
            self.assertTrue(controller.is_running)
            self.assertTrue(controller.assignment_pending)

            controller._odom_cb(Odometry(), 'tb3_0')
            timed_out = controller._observe_assignment_settle_locked(
                now=clock[0]
            )

        self.assertFalse(timed_out)
        self.assertTrue(
            controller.odom_confirmed_for_task['tb3_0']
        )
        self.assertEqual(10.0, controller._assignment_settle_since)
        self.assertEqual(15.0, controller._assignment_settle_deadline)
        self.assertIsNone(controller._assignment_settle_odom_deadline)

        missing_controller = make_controller()
        missing_controller._publish_markers = (
            lambda *args, **kwargs: None
        )
        missing_clock = [0.0]
        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: missing_clock[0],
        ):
            missing_controller._start_cb(String(data=json.dumps({
                'task_id': 'missing-first-odometry',
                'formation_type': 'line',
            })))
            missing_clock[0] = 10.01
            missing_controller._control_loop(None)

        self.assertFalse(missing_controller.is_running)
        self.assertFalse(missing_controller.assignment_pending)
        self.assertEqual(
            formation.FormationState.FAILED,
            missing_controller.formation_state,
        )
        self.assertIn(
            'did not become stationary',
            missing_controller.placement_error,
        )

    def test_multi_robot_gate_waits_for_all_and_restarts_for_one_mover(self):
        controller = make_two_robot_controller()
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task = {
            'tb3_0': True,
            'tb3_1': True,
        }
        controller.odom_received_at = {
            'tb3_0': 100.0,
            'tb3_1': 100.0,
        }
        controller._reset_assignment_settle_locked(now=100.0)
        controller._assignment_settle_readiness_locked(now=100.0)

        controller.odom_received_at['tb3_0'] = 100.51
        ready, timed_out, _ = (
            controller._assignment_settle_readiness_locked(now=100.51)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        self.assertEqual(
            ('tb3_1',), controller._assignment_settle_waiting
        )

        controller.robot_poses['tb3_1'] = Pose(x=1.02, y=0.0)
        controller.odom_received_at['tb3_1'] = 100.60
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=100.60)
        )
        self.assertFalse(ready)
        self.assertFalse(timed_out)
        self.assertEqual('movement', detail['reason'])
        self.assertEqual(['tb3_1'], detail['robots'])

        controller.odom_received_at = {
            'tb3_0': 101.11,
            'tb3_1': 101.11,
        }
        ready, timed_out, detail = (
            controller._assignment_settle_readiness_locked(now=101.11)
        )
        self.assertTrue(ready)
        self.assertFalse(timed_out)
        self.assertEqual('stable', detail['reason'])
        self.assertEqual((), controller._assignment_settle_waiting)

    def test_settle_check_and_snapshot_share_one_pose_lock_epoch(self):
        controller = make_controller()
        controller.lock = ContentionAwareRLock()
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = 100.0
        controller._reset_assignment_settle_locked(now=100.0)
        controller._assignment_settle_readiness_locked(now=100.0)
        controller.odom_received_at['tb3_0'] = 100.51

        snapshot_entered = threading.Event()
        release_snapshot = threading.Event()
        odometry_finished = threading.Event()
        planner_result = []
        original_prepare = controller._prepare_assignment_locked

        def blocking_prepare(*args, **kwargs):
            snapshot_entered.set()
            release_snapshot.wait(1.0)
            return original_prepare(*args, **kwargs)

        def prepare_snapshot():
            with controller.command_lock:
                planner_result.append(
                    controller._prepare_assignment_after_settle_locked(
                        now=100.51
                    )
                )

        odometry = Odometry()
        odometry.pose.pose.position.x = 0.20

        def publish_odometry():
            controller._odom_cb(odometry, 'tb3_0')
            odometry_finished.set()

        planner = threading.Thread(target=prepare_snapshot)
        odometry_thread = threading.Thread(target=publish_odometry)
        with mock.patch.object(
            formation.time, 'monotonic', return_value=100.52
        ), mock.patch.object(
            controller,
            '_prepare_assignment_locked',
            side_effect=blocking_prepare,
        ):
            planner.start()
            try:
                self.assertTrue(snapshot_entered.wait(1.0))
                odometry_thread.start()
                self.assertTrue(controller.lock.contended.wait(1.0))
                self.assertFalse(odometry_finished.is_set())
            finally:
                release_snapshot.set()
                planner.join(1.0)
                odometry_thread.join(1.0)

        self.assertFalse(planner.is_alive())
        self.assertFalse(odometry_thread.is_alive())
        snapshot, timed_out, detail = planner_result[0]
        self.assertFalse(timed_out)
        self.assertEqual('stable', detail['reason'])
        self.assertEqual(((0.0, 0.0),), snapshot['robot_positions'])
        self.assertEqual(
            0.20, controller.robot_poses['tb3_0'].position.x
        )

    def test_worker_times_out_without_an_odometry_wakeup(self):
        controller = make_controller()
        clock = [100.0]
        controller.current_task_id = 'worker-settle-timeout'
        controller.is_running = True
        controller.assignment_pending = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = clock[0]
        controller._reset_assignment_settle_locked(now=clock[0])
        controller._assignment_settle_readiness_locked(now=clock[0])
        controller._assignment_worker_wakeup.clear()
        clock[0] = controller._assignment_settle_deadline + 0.01

        timeout_handled = threading.Event()
        fail_timeout = controller._fail_assignment_settle_timeout_locked

        def observe_timeout(detail):
            result = fail_timeout(detail)
            timeout_handled.set()
            return result

        worker = threading.Thread(
            target=controller._assignment_worker_loop
        )
        controller._assignment_worker = worker
        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ), mock.patch.object(
            controller,
            '_fail_assignment_settle_timeout_locked',
            side_effect=observe_timeout,
        ), mock.patch.object(
            controller,
            '_prepare_assignment_locked',
        ) as prepare:
            worker.start()
            try:
                self.assertTrue(timeout_handled.wait(1.0))
            finally:
                controller._assignment_worker_stop.set()
                controller._assignment_worker_wakeup.set()
                worker.join(1.0)

        self.assertFalse(worker.is_alive())
        prepare.assert_not_called()
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )

    def test_second_live_replan_opens_a_new_settle_window(self):
        controller = make_controller()
        clock = [100.0]
        controller.current_task_id = 'second-live-replan'
        controller.is_running = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = clock[0]
        plan = {
            'arena_size': 10.0,
            'arena_profile': 'swarm_arena',
            'exclusion_zones': (),
            'model_poses': {},
        }
        positions = {'tb3_0': (0.0, 0.0)}
        detail = {'gate': 'assignment_pose_snapshot', 'robot': 'tb3_0'}

        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ):
            self.assertTrue(controller._schedule_live_replan_locked(
                plan, positions, {}, detail
            ))
            controller._assignment_settle_readiness_locked(now=100.0)
            controller.odom_received_at['tb3_0'] = 100.51
            snapshot, timed_out, _ = (
                controller._prepare_assignment_after_settle_locked(
                    now=100.51
                )
            )
            self.assertIsNotNone(snapshot)
            self.assertFalse(timed_out)

            clock[0] = 101.0
            controller._assignment_worker_wakeup.clear()
            self.assertTrue(controller._schedule_live_replan_locked(
                plan, positions, {}, detail
            ))
            second_deadline = controller._assignment_settle_deadline
            snapshot, timed_out, detail = (
                controller._prepare_assignment_after_settle_locked(
                    now=101.0
                )
            )

        self.assertEqual(2, controller._live_replan_attempts)
        self.assertEqual(106.0, second_deadline)
        self.assertIsNone(snapshot)
        self.assertFalse(timed_out)
        self.assertEqual('first_sample', detail['reason'])
        self.assertEqual(
            'waiting_for_stability',
            controller._assignment_planning_phase,
        )
        self.assertTrue(controller._assignment_worker_wakeup.is_set())

    def test_cancel_and_stop_clear_planning_status(self):
        controller = make_controller()
        controller.current_task_id = 'settle-cleanup'
        controller.is_running = True
        controller.task_started_at = 99.0
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = 100.0
        controller._queue_assignment_after_settle_locked()
        controller._assignment_settle_deadline = 105.0
        controller._assignment_settle_readiness_locked(now=100.0)

        with mock.patch.object(
            formation.time, 'monotonic', return_value=100.25
        ):
            controller._publish_status([], 0.0)
        planning = json.loads(
            controller.status_pub.messages[-1].data
        )['planning']
        self.assertEqual('waiting_for_stability', planning['phase'])
        self.assertFalse(planning['stability_reached'])
        self.assertEqual(0.25, planning['stationary_for'])
        self.assertEqual(0.5, planning['required_stationary_time'])
        self.assertEqual(
            ['tb3_0'], planning['waiting_for_stability']
        )

        controller.odom_received_at['tb3_0'] = 100.51
        ready, timed_out, _ = (
            controller._assignment_settle_readiness_locked(now=100.51)
        )
        self.assertTrue(ready)
        self.assertFalse(timed_out)
        with mock.patch.object(
            formation.time, 'monotonic', return_value=100.51
        ):
            controller._publish_status([], 0.0)
        stable_planning = json.loads(
            controller.status_pub.messages[-1].data
        )['planning']
        self.assertTrue(stable_planning['stability_reached'])
        self.assertEqual([], stable_planning['waiting_for_stability'])

        with controller.command_lock:
            controller._cancel_pending_assignment_locked(
                clear_assignments=True
            )
        controller._publish_status([], 0.0)
        cancelled_status = json.loads(
            controller.status_pub.messages[-1].data
        )
        self.assertNotIn('planning', cancelled_status)
        self.assertEqual({}, controller._assignment_settle_anchor)
        self.assertIsNone(controller._assignment_settle_since)
        self.assertIsNone(controller._assignment_settle_deadline)
        self.assertIsNone(controller._assignment_settle_odom_deadline)
        self.assertIsNone(controller._assignment_settle_ready_at)
        self.assertEqual('idle', controller._assignment_planning_phase)
        self.assertEqual((), controller._assignment_settle_waiting)

        with mock.patch.object(
            formation.time, 'monotonic', return_value=101.0
        ):
            controller._queue_assignment_after_settle_locked()
        controller._stop_cb(String(data=json.dumps({
            'task_id': 'settle-cleanup',
        })))
        stopped_status = json.loads(
            controller.status_pub.messages[-1].data
        )
        self.assertNotIn('planning', stopped_status)
        self.assertFalse(controller.assignment_pending)
        self.assertIsNone(controller._assignment_settle_odom_deadline)
        self.assertIsNone(controller._assignment_settle_ready_at)
        self.assertEqual('idle', controller._assignment_planning_phase)

    def test_active_replan_entry_points_never_run_the_solver_inline(self):
        for entry_point in ('shape', 'fleet', 'recompute', 'assign'):
            with self.subTest(entry_point=entry_point):
                controller = (
                    make_two_robot_controller()
                    if entry_point == 'fleet'
                    else make_controller()
                )
                controller.current_task_id = f'{entry_point}-gate'
                controller.is_running = True
                controller._assignment_worker_wakeup.clear()

                with mock.patch.object(
                    formation, 'minimum_distance_assignment'
                ) as solver:
                    if entry_point == 'shape':
                        controller._set_shape_cb(String(data='triangle'))
                    elif entry_point == 'fleet':
                        controller._fleet_list_cb(String(data='tb3_0'))
                    elif entry_point == 'recompute':
                        controller._recompute_formation()
                    else:
                        controller._assign_robots_to_positions()

                solver.assert_not_called()
                self.assertTrue(controller.assignment_pending)
                self.assertEqual({}, controller.assignments)
                self.assertEqual(
                    'waiting_for_stability',
                    controller._assignment_planning_phase,
                )
                self.assertTrue(
                    controller._assignment_worker_wakeup.is_set()
                )

    def test_active_fleet_resize_reuses_one_confirmed_stop(self):
        controller = make_two_robot_controller()
        controller.current_task_id = 'single-resize-stop'
        controller.is_running = True
        original_publishers = dict(controller.cmd_vel_pubs)

        with mock.patch.object(
            controller,
            '_stop_all_robots',
            wraps=controller._stop_all_robots,
        ) as stop_all:
            controller._fleet_list_cb(String(data='tb3_0'))

        stop_all.assert_called_once_with('fleet_change')
        self.assertEqual(['tb3_0'], controller.robot_ids)
        self.assertTrue(controller.assignment_pending)
        self.assertEqual(
            'waiting_for_stability',
            controller._assignment_planning_phase,
        )
        for robot_id, publisher in original_publishers.items():
            with self.subTest(robot_id=robot_id):
                self.assertEqual(1, len(publisher.messages))
                command = publisher.messages[0]
                self.assertEqual(0.0, command.linear.x)
                self.assertEqual(0.0, command.angular.z)

        self.assertEqual(
            'fleet_change',
            controller._last_stop_publication['reason'],
        )
        self.assertEqual(
            1, controller._last_stop_publication['attempt']
        )
        self.assertEqual(
            2, controller._last_stop_publication['requested_count']
        )
        self.assertTrue(
            controller._last_stop_publication['publication_confirmed']
        )

    def test_active_fleet_growth_and_replacement_stop_every_endpoint_once(
        self,
    ):
        class ProvisionedAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

            def __init__(self, _robot_id):
                self.closed = False

            def shutdown(self):
                self.closed = True

        cases = (
            ('grow', 'tb3_0,tb3_1', ['tb3_0', 'tb3_1']),
            ('replace', 'tb3_1', ['tb3_1']),
        )
        for label, roster, expected_ids in cases:
            with self.subTest(case=label):
                controller = make_controller()
                controller.current_task_id = f'{label}-single-stop'
                controller.is_running = True
                original_publisher = controller.cmd_vel_pubs['tb3_0']
                created_publishers = []

                def create_publisher(*_args, **_kwargs):
                    publisher = FakePublisher()
                    created_publishers.append(publisher)
                    return publisher

                with mock.patch.object(
                    formation.rospy,
                    'Publisher',
                    side_effect=create_publisher,
                    create=True,
                ) as publisher_factory, mock.patch.object(
                    formation.rospy,
                    'Subscriber',
                    side_effect=lambda *_args, **_kwargs: FakeResource(),
                    create=True,
                ), mock.patch.object(
                    formation,
                    'ObstacleAvoidance',
                    ProvisionedAvoidance,
                ), mock.patch.object(
                    controller,
                    '_stop_all_robots',
                    wraps=controller._stop_all_robots,
                ) as stop_all:
                    controller._fleet_list_cb(String(data=roster))

                publisher_factory.assert_called_once()
                self.assertEqual(1, len(created_publishers))
                self.assertEqual(2, stop_all.call_count)
                self.assertEqual(
                    mock.call('fleet_change'),
                    stop_all.call_args_list[0],
                )
                second_call = stop_all.call_args_list[1]
                self.assertEqual(
                    ('fleet_change',), second_call.args
                )
                new_snapshot = second_call.kwargs['publisher_snapshot']
                self.assertEqual(1, len(new_snapshot))
                self.assertEqual('tb3_1', new_snapshot[0]['robot_id'])
                self.assertEqual(expected_ids, controller.robot_ids)
                self.assertTrue(
                    controller._last_stop_publication[
                        'publication_confirmed'
                    ]
                )
                self.assertEqual(
                    1,
                    controller._last_stop_publication['requested_count'],
                )
                self.assertEqual(
                    'fleet_change',
                    controller._last_stop_publication['reason'],
                )
                self.assertEqual(
                    2, controller._last_stop_publication['attempt']
                )
                self.assertTrue(controller.assignment_pending)
                self.assertEqual(
                    'waiting_for_stability',
                    controller._assignment_planning_phase,
                )

                affected_publishers = [
                    original_publisher, created_publishers[0]
                ]
                for index, publisher in enumerate(affected_publishers):
                    with self.subTest(case=label, publisher=index):
                        self.assertEqual(1, len(publisher.messages))
                        command = publisher.messages[0]
                        self.assertEqual(0.0, command.linear.x)
                        self.assertEqual(0.0, command.angular.z)

    def test_growth_stops_old_fleet_before_new_publisher_setup_finishes(
        self,
    ):
        class ProvisionedAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

            def __init__(self, _robot_id):
                pass

            def shutdown(self):
                pass

        controller = make_controller()
        controller.current_task_id = 'blocked-grow-setup'
        controller.is_running = True
        old_publisher = controller.cmd_vel_pubs['tb3_0']
        publisher_setup_started = threading.Event()
        release_publisher_setup = threading.Event()
        created_publishers = []

        def blocked_publisher_setup(*_args, **_kwargs):
            publisher_setup_started.set()
            release_publisher_setup.wait(1.0)
            publisher = FakePublisher()
            created_publishers.append(publisher)
            return publisher

        roster = threading.Thread(
            target=controller._fleet_list_cb,
            args=(String(data='tb3_0,tb3_1'),),
        )
        with mock.patch.object(
            formation.rospy,
            'Publisher',
            side_effect=blocked_publisher_setup,
            create=True,
        ), mock.patch.object(
            formation.rospy,
            'Subscriber',
            side_effect=lambda *_args, **_kwargs: FakeResource(),
            create=True,
        ), mock.patch.object(
            formation,
            'ObstacleAvoidance',
            ProvisionedAvoidance,
        ), mock.patch.object(
            controller,
            '_stop_all_robots',
            wraps=controller._stop_all_robots,
        ) as stop_all:
            roster.start()
            try:
                self.assertTrue(publisher_setup_started.wait(0.5))
                self.assertEqual(1, len(old_publisher.messages))
                old_zero = old_publisher.messages[0]
                self.assertEqual(0.0, old_zero.linear.x)
                self.assertEqual(0.0, old_zero.angular.z)
                self.assertEqual(
                    1, controller._last_stop_publication['attempt']
                )
                self.assertEqual(
                    'fleet_change',
                    controller._last_stop_publication['reason'],
                )
            finally:
                release_publisher_setup.set()
                roster.join(1.0)

        self.assertFalse(roster.is_alive())
        self.assertEqual(2, stop_all.call_count)
        self.assertEqual(1, len(created_publishers))
        self.assertEqual(1, len(created_publishers[0].messages))
        new_zero = created_publishers[0].messages[0]
        self.assertEqual(0.0, new_zero.linear.x)
        self.assertEqual(0.0, new_zero.angular.z)
        self.assertEqual(
            2, controller._last_stop_publication['attempt']
        )
        self.assertEqual(
            'fleet_change',
            controller._last_stop_publication['reason'],
        )

    def test_partial_fleet_provision_failure_fails_closed(self):
        class ProvisionedAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

            def __init__(self, _robot_id):
                pass

            def shutdown(self):
                pass

        controller = make_controller()
        controller.current_task_id = 'partial-grow-failure'
        controller.is_running = True
        old_publisher = controller.cmd_vel_pubs['tb3_0']
        created_publishers = []

        def create_then_fail(*_args, **_kwargs):
            if created_publishers:
                raise RuntimeError('publisher setup failed')
            publisher = FakePublisher()
            created_publishers.append(publisher)
            return publisher

        with mock.patch.object(
            formation.rospy,
            'Publisher',
            side_effect=create_then_fail,
            create=True,
        ), mock.patch.object(
            formation.rospy,
            'Subscriber',
            side_effect=lambda *_args, **_kwargs: FakeResource(),
            create=True,
        ), mock.patch.object(
            formation,
            'ObstacleAvoidance',
            ProvisionedAvoidance,
        ):
            controller._fleet_list_cb(
                String(data='tb3_0,tb3_1,tb3_2')
            )

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual({}, controller.assignments)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('could not provision', controller.placement_error)
        self.assertEqual(['tb3_0'], controller.robot_ids)
        self.assertIn('tb3_1', controller.cmd_vel_pubs)
        self.assertEqual(2, len(old_publisher.messages))
        self.assertEqual(1, len(created_publishers[0].messages))
        self.assertEqual(
            'fleet_provision_failure',
            controller._last_stop_publication['reason'],
        )
        self.assertEqual(
            2, controller._last_stop_publication['requested_count']
        )
        self.assertTrue(
            controller._last_stop_publication['publication_confirmed']
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('failed', status['state'])
        self.assertEqual(
            'fleet_provision_failure',
            status['stop_publication']['reason'],
        )

    def test_grow_uses_roster_odometry_deadline_not_original_task_age(
        self,
    ):
        class ProvisionedAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

            def __init__(self, _robot_id):
                pass

            def shutdown(self):
                pass

        controller = make_controller()
        clock = [100.0]
        controller.current_task_id = 'old-running-task'
        controller.task_started_at = 0.0
        controller.is_running = True
        controller.formation_state = formation.FormationState.MOVING
        controller._publish_markers = lambda *args, **kwargs: None
        controller.odom_confirmed_for_task['tb3_0'] = True
        controller.odom_received_at['tb3_0'] = clock[0]

        with mock.patch.object(
            formation.time,
            'monotonic',
            side_effect=lambda: clock[0],
        ), mock.patch.object(
            formation.rospy,
            'Publisher',
            side_effect=lambda *_args, **_kwargs: FakePublisher(),
            create=True,
        ), mock.patch.object(
            formation.rospy,
            'Subscriber',
            side_effect=lambda *_args, **_kwargs: FakeResource(),
            create=True,
        ), mock.patch.object(
            formation,
            'ObstacleAvoidance',
            ProvisionedAvoidance,
        ):
            controller._fleet_list_cb(String(data='tb3_0,tb3_1'))
            self.assertEqual(
                110.0, controller._assignment_settle_odom_deadline
            )
            self.assertFalse(
                controller.odom_confirmed_for_task['tb3_1']
            )

            clock[0] = 100.01
            controller._control_loop(None)
            self.assertTrue(controller.is_running)
            self.assertEqual(
                formation.FormationState.FORMING,
                controller.formation_state,
            )
            self.assertEqual(
                ['tb3_1'], controller.waiting_for_odometry
            )
            self.assertEqual([], controller.stale_odometry)
            self.assertIsNone(controller.placement_error)

            clock[0] = 110.01
            controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )

    def test_persistent_pose_churn_exhausts_replan_budget(self):
        controller = make_controller()
        controller.is_running = True
        controller.movement_mode = formation.MovementMode.STATIC
        controller.robot_poses['tb3_0'] = Pose(x=0.4355, y=0.0)
        controller.formation_offsets = [(0.0, 0.0)]
        controller.spawn_exclusion_zones = [{
            'name': 'flat_wall',
            'worlds': ['swarm_arena'],
            'shape': 'box',
            'x': -0.5,
            'y': 0.0,
            'width': 1.0,
            'height': 4.0,
        }]

        for attempt in range(3):
            old_x = controller.robot_poses['tb3_0'].position.x
            snapshot = controller._prepare_assignment_locked(
                settle_gate_passed=True
            )
            new_x = 0.2965 if old_x > 0.40 else 0.4355
            controller.robot_poses['tb3_0'] = Pose(x=new_x, y=0.0)
            routes = {'tb3_0': [(old_x, 0.0), (0.60, 0.0)]}
            with mock.patch.object(
                controller,
                '_plan_routed_static_assignment',
                return_value=((0.60, 0.0), [(0.60, 0.0)], [0], routes),
            ):
                committed = controller._compute_and_commit_assignment(
                    snapshot
                )
            self.assertFalse(committed)
            if attempt < controller.live_replan_limit:
                self.assertTrue(controller.assignment_pending)
                self.assertIsNone(controller.placement_error)

        self.assertEqual(
            controller.live_replan_limit,
            controller._live_replan_attempts,
        )
        self.assertFalse(controller.assignment_pending)
        self.assertEqual(formation.FormationState.FAILED,
                         controller.formation_state)
        self.assertIn('live arena changed', controller.placement_error)

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('failed', status['state'])
        self.assertIn('live arena changed', status['error'])

    def test_live_replan_rejects_contact_corruption_and_moved_geometry(self):
        controller = make_controller()
        controller.is_running = True
        wall = {
            'name': 'flat_wall',
            'model': 'wall',
            'worlds': ['swarm_arena'],
            'shape': 'box',
            'x': -0.5,
            'y': 0.0,
            'width': 1.0,
            'height': 4.0,
        }
        plan = {
            'arena_size': 10.0,
            'arena_margin': 0.35,
            'arena_profile': 'swarm_arena',
            'exclusion_zones': (wall,),
            'model_poses': {'wall': (-0.5, 0.0, 0.0)},
        }
        good_models = {'wall': (-0.5, 0.0, 0.0)}

        unsafe_starts = (
            {'tb3_0': (0.10, 0.0)},
            {'tb3_0': (float('nan'), 0.0)},
        )
        for positions in unsafe_starts:
            with self.subTest(positions=positions):
                self.assertFalse(controller._live_poses_can_be_replanned(
                    plan, positions, good_models
                ))

        self.assertFalse(controller._live_poses_can_be_replanned(
            plan,
            {'tb3_0': (0.2965, 0.0)},
            {'wall': (-0.49, 0.0, 0.0)},
        ))

        controller._live_replan_attempts = controller.live_replan_limit
        self.assertFalse(controller._live_poses_can_be_replanned(
            plan, {'tb3_0': (0.2965, 0.0)}, good_models
        ))


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

    def test_2965_mm_buffer_jitter_only_allows_clearance_gain(self):
        wall = [{
            'name': 'flat_wall',
            'worlds': ['swarm_arena'],
            'shape': 'box',
            'x': -0.5,
            'y': 0.0,
            'width': 1.0,
            'height': 4.0,
        }]
        start = (0.2965, 0.0)

        self.assertTrue(straight_route_is_safe(
            start, (0.60, 0.0), 0.30, wall, 'swarm_arena'
        ))
        self.assertFalse(straight_route_is_safe(
            start, (0.2965, 1.0), 0.30, wall, 'swarm_arena'
        ))
        self.assertFalse(straight_route_is_safe(
            start, (-0.20, 0.0), 0.30, wall, 'swarm_arena'
        ))
        self.assertFalse(straight_route_is_safe(
            (float('nan'), 0.0), (0.60, 0.0),
            0.30, wall, 'swarm_arena',
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
    MOVING_MATRIX_CASES = (
        (3, 'triangle', 0.65, 'grid'),
        (5, 'square', 0.65, 'circle'),
        (7, 'A', 0.55, 'line'),
        (8, 'V', 0.55, 'grid'),
        (9, 'diamond', 0.55, 'circle'),
        (10, 'S', 0.55, 'grid'),
    )

    @staticmethod
    def _enable_pid_motion(controller):
        controller.pid_linear['tb3_0'] = formation.PIDController(
            0.6, 0.01, 0.15, 0.22
        )
        controller.pid_angular['tb3_0'] = formation.PIDController(
            1.2, 0.0, 0.1, 1.5
        )

    def _prepared_positive_entry_controller(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.robot_poses['tb3_0'] = Pose(x=-1.0, y=0.0)
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller._model_states_cb(ModelStates(
            names=['moving_box'], poses=[Pose(x=4.0, y=4.0)]
        ))
        controller.pid_linear['tb3_0'] = formation.PIDController(
            0.6, 0.01, 0.15, 0.22
        )
        controller.pid_angular['tb3_0'] = formation.PIDController(
            1.2, 0.0, 0.1, 1.5
        )
        controller._publish_status = lambda *args, **kwargs: None
        controller._publish_markers = lambda *args, **kwargs: None

        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(snapshot),
            controller.placement_error,
        )
        controller.is_running = True
        return controller

    def _controller_with_a_positive_entry_command(self):
        controller = self._prepared_positive_entry_controller()
        controller._control_loop(None)
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertGreater(command.linear.x, 0.0)
        return controller

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

    def test_moving_diamond_forms_before_its_centroid_path_starts(self):
        controller, _targets = make_closed_loop_controller(
            9, 'diamond', 0.55
        )
        controller.movement_mode = formation.MovementMode.MOVING
        controller._initial_formation_acquired = False

        snapshot = controller._prepare_assignment_locked(
            settle_gate_passed=True
        )
        self.assertTrue(controller._compute_and_commit_assignment(snapshot))
        anchored_centroid = (controller.centroid_x, controller.centroid_y)

        # At RTF 3.0 this represents the 75 wall-second live horizon.
        for _ in range(4500):
            controller._control_step(None)
            if controller._initial_formation_acquired:
                break

            self.assertEqual(
                anchored_centroid,
                (controller.centroid_x, controller.centroid_y),
            )
            dt = 1.0 / controller.control_rate
            for robot_id in controller.robot_ids:
                command = controller.cmd_vel_pubs[robot_id].messages[-1]
                yaw = formation.normalize_angle(
                    controller.robot_yaws[robot_id]
                    + command.angular.z * dt
                )
                pose = controller.robot_poses[robot_id]
                pose.position.x += command.linear.x * math.cos(yaw) * dt
                pose.position.y += command.linear.x * math.sin(yaw) * dt
                controller.robot_yaws[robot_id] = yaw

        self.assertTrue(controller._initial_formation_acquired)
        self.assertEqual(
            formation.FormationState.MOVING,
            controller.formation_state,
        )
        self.assertEqual(0.0, controller.centroid_time)

        maximum_tracking_error = 0.0
        for _ in range(1500):
            controller._control_step(None)
            dt = 1.0 / controller.control_rate
            for robot_id in controller.robot_ids:
                command = controller.cmd_vel_pubs[robot_id].messages[-1]
                yaw = formation.normalize_angle(
                    controller.robot_yaws[robot_id]
                    + command.angular.z * dt
                )
                pose = controller.robot_poses[robot_id]
                pose.position.x += command.linear.x * math.cos(yaw) * dt
                pose.position.y += command.linear.x * math.sin(yaw) * dt
                controller.robot_yaws[robot_id] = yaw

            world_targets = controller._get_world_targets()
            maximum_tracking_error = max(
                maximum_tracking_error,
                max(
                    math.hypot(
                        controller.robot_poses[robot_id].position.x
                        - world_targets[controller.assignments[robot_id]][0],
                        controller.robot_poses[robot_id].position.y
                        - world_targets[controller.assignments[robot_id]][1],
                    )
                    for robot_id in controller.robot_ids
                ),
            )
            self.assertEqual(
                formation.FormationState.MOVING,
                controller.formation_state,
            )

        self.assertLessEqual(maximum_tracking_error, 0.11)
        self.assertGreater(
            controller.centroid_time * controller.centroid_speed,
            0.5,
        )
        self.assertNotEqual(
            anchored_centroid,
            (controller.centroid_x, controller.centroid_y),
        )

    def test_complete_moving_matrix_has_a_safe_rigid_orbit_at_rtf3(self):
        horizon_steps = int(75.0 * 3.0 * 20.0)
        minimum_reserve_steps = int(15.0 * 20.0)

        for (
            robot_count, shape, spacing, spawn_pattern
        ) in self.MOVING_MATRIX_CASES:
            with self.subTest(robot_count=robot_count, shape=shape):
                controller, _targets = make_closed_loop_controller(
                    robot_count, shape, spacing
                )
                controller.movement_mode = formation.MovementMode.MOVING
                controller._initial_formation_acquired = False
                controller.model_poses = {
                    zone['model']: (
                        zone.get('x', 0.0),
                        zone.get('y', 0.0),
                        zone.get('yaw', 0.0),
                    )
                    for zone in controller.spawn_exclusion_zones
                    if zone.get('model')
                }
                apply_production_spawn_pattern(
                    controller, robot_count, spawn_pattern
                )

                snapshot = controller._prepare_assignment_locked(
                    settle_gate_passed=True
                )
                self.assertTrue(
                    controller._compute_and_commit_assignment(snapshot),
                    controller.placement_error,
                )
                self.assertTrue(controller.orbit_path_validated)
                self.assertEqual(
                    len(controller.model_poses),
                    controller.orbit_validation_live_models,
                )
                self.assertEqual(
                    set(controller.robot_ids),
                    set(controller.route_waypoints),
                )

                swept_offsets, sample_count = (
                    controller._rigid_orbit_offsets(
                        tuple(controller.formation_offsets),
                        controller.effective_path_radius,
                        controller.orbit_sample_step,
                    )
                )
                self.assertEqual(
                    sample_count, controller.orbit_validation_samples
                )
                swept_targets = [
                    (
                        controller.path_center_x + offset_x,
                        controller.path_center_y + offset_y,
                    )
                    for offset_x, offset_y in swept_offsets
                ]
                self.assertTrue(formation_targets_are_safe(
                    swept_targets,
                    controller.arena_size,
                    controller.arena_margin,
                    controller.formation_obstacle_clearance,
                    controller.spawn_exclusion_zones,
                    controller.arena_profile,
                    controller.model_poses,
                ))

                acquired_at = None
                for step in range(horizon_steps):
                    controller._control_step(None)
                    self.assertTrue(
                        controller.is_running, controller.placement_error
                    )
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
                    if (
                        acquired_at is None
                        and controller._initial_formation_acquired
                    ):
                        acquired_at = step

                self.assertIsNotNone(acquired_at)
                self.assertLessEqual(
                    acquired_at,
                    horizon_steps - minimum_reserve_steps,
                    'formation left less than 15 simulated seconds of '
                    'movement margin inside the 225-second gate',
                )
                self.assertEqual(
                    formation.FormationState.MOVING,
                    controller.formation_state,
                )
                self.assertGreater(controller.centroid_time, 0.0)
                self.assertAlmostEqual(controller.formation_heading, 0.0)
                for offset, target in zip(
                    controller.formation_offsets,
                    controller._get_world_targets(),
                ):
                    self.assertAlmostEqual(
                        target[0] - controller.centroid_x,
                        offset[0],
                        places=9,
                    )
                    self.assertAlmostEqual(
                        target[1] - controller.centroid_y,
                        offset[1],
                        places=9,
                    )

    def test_live_obstacle_on_the_next_orbit_pose_fails_closed(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller.path_center_x = 0.0
        controller.path_center_y = 0.0
        controller.effective_path_radius = 0.5
        controller.centroid_time = 0.0
        controller._set_circular_centroid_pose()
        controller.orbit_path_validated = True
        controller._initial_formation_acquired = True
        controller._maximum_position_error = 0.0

        next_phase = (
            controller.centroid_speed / controller.effective_path_radius
            / controller.control_rate
        )
        controller.model_poses = {
            'moving_box': (
                0.5 * math.cos(next_phase),
                0.5 * math.sin(next_phase),
                0.0,
            )
        }

        controller._update_centroid(1.0 / controller.control_rate)

        self.assertIn('live obstacle', controller.placement_error)
        self.assertEqual(0.0, controller.centroid_time)

    def test_open_path_target_outside_the_arena_fails_closed(self):
        for centroid_path in (
            formation.CentroidPath.LINEAR,
            formation.CentroidPath.WAYPOINTS,
        ):
            with self.subTest(path=centroid_path.value):
                controller = make_controller()
                controller.movement_mode = formation.MovementMode.MOVING
                controller.centroid_path = centroid_path
                controller.formation_offsets = [(0.0, 0.0)]
                controller.robot_poses['tb3_0'] = Pose(x=4.0, y=0.0)
                if centroid_path == formation.CentroidPath.WAYPOINTS:
                    controller.centroid_waypoints = [{'x': 5.0, 'y': 0.0}]

                snapshot = controller._prepare_assignment_locked()
                self.assertTrue(
                    controller._compute_and_commit_assignment(snapshot),
                    controller.placement_error,
                )
                controller.centroid_x = 4.649
                controller.centroid_y = 0.0
                controller.centroid_heading = 0.0
                controller._initial_formation_acquired = True
                controller._maximum_position_error = 0.0
                controller.is_running = True
                self._enable_pid_motion(controller)
                generation = controller._assignment_generation

                controller._control_step(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertIn('live obstacle', controller.placement_error)
                self.assertGreater(
                    controller._assignment_generation, generation
                )
                self.assertEqual({}, controller.assignments)
                messages = controller.cmd_vel_pubs['tb3_0'].messages
                self.assertTrue(messages)
                self.assertTrue(all(
                    message.linear.x == 0.0
                    and message.angular.z == 0.0
                    for message in messages
                ))

    def test_linear_path_rechecks_an_obstacle_before_publishing(self):
        controller = make_controller()
        controller.movement_mode = formation.MovementMode.MOVING
        controller.centroid_path = formation.CentroidPath.LINEAR
        controller.formation_offsets = [(0.0, 0.0)]
        controller.robot_poses['tb3_0'] = Pose(x=-0.5, y=0.0)
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller._model_states_cb(ModelStates(
            names=['moving_box'], poses=[Pose(x=4.0, y=4.0)]
        ))
        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(snapshot),
            controller.placement_error,
        )
        controller._initial_formation_acquired = True
        controller._maximum_position_error = 0.0
        controller.is_running = True
        self._enable_pid_motion(controller)
        obstacle_was_moved = []

        class MoveObstacleDuringCommand(PassThroughAvoidance):
            def apply_avoidance(self, command):
                target_x, target_y = controller._get_world_targets()[0]
                controller._model_states_cb(ModelStates(
                    names=['moving_box'],
                    poses=[Pose(x=target_x, y=target_y)],
                ))
                obstacle_was_moved.append(True)
                return command

        controller.avoidance['tb3_0'] = MoveObstacleDuringCommand()

        controller._control_step(None)

        self.assertTrue(obstacle_was_moved)
        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('live obstacle', controller.placement_error)
        messages = controller.cmd_vel_pubs['tb3_0'].messages
        self.assertTrue(messages)
        self.assertTrue(all(
            message.linear.x == 0.0 and message.angular.z == 0.0
            for message in messages
        ))

    def test_adaptive_deformed_target_is_validated_before_publish(self):
        controller = make_controller()
        controller.movement_mode = formation.MovementMode.ADAPTIVE
        controller.centroid_path = formation.CentroidPath.LINEAR
        controller.centroid_speed = 0.0
        controller.formation_offsets = [(0.0, 0.0)]
        controller.robot_poses['tb3_0'] = Pose(x=4.0, y=0.0)
        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(snapshot),
            controller.placement_error,
        )
        controller._initial_formation_acquired = True
        controller._maximum_position_error = 0.0
        controller.is_running = True
        self._enable_pid_motion(controller)
        controller.avoidance['tb3_0'] = AdaptiveThreatAvoidance()

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('live obstacle', controller.placement_error)
        messages = controller.cmd_vel_pubs['tb3_0'].messages
        self.assertTrue(messages)
        self.assertTrue(all(
            message.linear.x == 0.0 and message.angular.z == 0.0
            for message in messages
        ))

    def test_safe_adaptive_target_drives_status_and_markers(self):
        controller = make_controller()
        controller.movement_mode = formation.MovementMode.ADAPTIVE
        controller.centroid_path = formation.CentroidPath.LINEAR
        controller.centroid_speed = 0.0
        controller.formation_offsets = [(0.0, 0.0)]
        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(snapshot),
            controller.placement_error,
        )
        controller._initial_formation_acquired = True
        controller._maximum_position_error = 0.0
        controller.is_running = True
        self._enable_pid_motion(controller)
        controller.avoidance['tb3_0'] = AdaptiveThreatAvoidance()
        published = {}
        controller._publish_status = lambda targets, error: published.update(
            status=list(targets), error=error
        )
        controller._publish_markers = lambda targets: published.update(
            markers=list(targets)
        )

        controller._control_step(None)

        self.assertTrue(controller.is_running, controller.placement_error)
        self.assertEqual(
            formation.FormationState.DEFORMING,
            controller.formation_state,
        )
        self.assertAlmostEqual(0.99, published['status'][0][0], places=6)
        self.assertEqual(published['status'], published['markers'])
        self.assertAlmostEqual(0.0, controller._get_world_targets()[0][0])
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertGreater(command.linear.x, 0.0)

    def test_non_finite_adaptive_geometry_fails_closed(self):
        controller = make_controller()
        controller.movement_mode = formation.MovementMode.ADAPTIVE
        controller.centroid_path = formation.CentroidPath.LINEAR
        controller.centroid_speed = 0.0
        controller.formation_offsets = [(0.0, 0.0)]
        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(
            controller._compute_and_commit_assignment(snapshot),
            controller.placement_error,
        )
        controller._initial_formation_acquired = True
        controller._maximum_position_error = 0.0
        controller.is_running = True
        self._enable_pid_motion(controller)
        controller.avoidance['tb3_0'] = AdaptiveThreatAvoidance(
            repulsion_x=float('nan')
        )

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('invalid target geometry', controller.placement_error)
        messages = controller.cmd_vel_pubs['tb3_0'].messages
        self.assertTrue(messages)
        self.assertTrue(all(
            math.isfinite(message.linear.x)
            and math.isfinite(message.angular.z)
            and message.linear.x == 0.0
            and message.angular.z == 0.0
            for message in messages
        ))

    def test_obstacle_entering_an_assembly_route_only_publishes_stop(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.robot_poses['tb3_0'] = Pose(x=-1.0, y=0.0)
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller.model_poses = {'moving_box': (4.0, 4.0, 0.0)}
        controller.pid_linear['tb3_0'] = formation.PIDController(
            0.6, 0.01, 0.15, 0.2
        )
        controller.pid_angular['tb3_0'] = formation.PIDController(
            1.2, 0.0, 0.1, 1.5
        )

        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(controller._compute_and_commit_assignment(snapshot))
        route = controller.route_waypoints['tb3_0']
        next_point = route[0]
        start = controller.robot_poses['tb3_0'].position
        controller.model_poses = {
            'moving_box': (
                (start.x + next_point[0]) / 2.0,
                (start.y + next_point[1]) / 2.0,
                0.0,
            )
        }
        controller.is_running = True

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn('remaining entry route', controller.placement_error)
        messages = controller.cmd_vel_pubs['tb3_0'].messages
        self.assertTrue(messages)
        self.assertTrue(all(
            message.linear.x == 0.0 and message.angular.z == 0.0
            for message in messages
        ))

    def test_identical_or_jittered_safe_model_states_do_not_cancel_entry(self):
        controller = make_controller()
        controller.formation_offsets = [(0.0, 0.0)]
        controller.robot_poses['tb3_0'] = Pose(x=-1.0, y=0.0)
        controller.spawn_exclusion_zones = [{
            'name': 'moving_box',
            'model': 'moving_box',
            'worlds': ['swarm_arena'],
            'shape': 'circle',
            'x': 4.0,
            'y': 4.0,
            'radius': 0.10,
        }]
        controller.model_poses = {'moving_box': (4.0, 4.0, 0.0)}
        controller.pid_linear['tb3_0'] = formation.PIDController(
            0.6, 0.01, 0.15, 0.2
        )
        controller.pid_angular['tb3_0'] = formation.PIDController(
            1.2, 0.0, 0.1, 1.5
        )
        controller._publish_status = lambda *args, **kwargs: None
        controller._publish_markers = lambda *args, **kwargs: None
        snapshot = controller._prepare_assignment_locked()
        self.assertTrue(controller._compute_and_commit_assignment(snapshot))
        controller.model_poses = {'moving_box': (4.005, 3.996, 0.002)}
        controller.is_running = True

        controller._control_step(None)

        self.assertTrue(controller.is_running)
        self.assertIsNone(controller.placement_error)
        command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
        self.assertGreater(
            abs(command.linear.x) + abs(command.angular.z), 0.0
        )

    def test_truncated_model_state_cannot_hide_an_obstacle_on_the_route(self):
        duplicate_bad_pose = Pose(x=4.0, y=4.0)
        duplicate_bad_pose.orientation.z = float('nan')
        malformed_messages = (
            (
                'truncated',
                ModelStates(names=['moving_box'], poses=[]),
            ),
            (
                'malformed',
                types.SimpleNamespace(name=None, pose=[]),
            ),
            (
                'malformed_name',
                types.SimpleNamespace(name=[[]], pose=[Pose()]),
            ),
            (
                'duplicate_name',
                ModelStates(
                    names=['moving_box', 'moving_box'],
                    poses=[duplicate_bad_pose, Pose(x=4.0, y=4.0)],
                ),
            ),
        )
        for label, malformed in malformed_messages:
            with self.subTest(case=label):
                controller = self._prepared_positive_entry_controller()
                target_x, target_y = controller._get_world_targets()[0]
                controller._model_states_cb(ModelStates(
                    names=['moving_box'],
                    poses=[Pose(x=target_x, y=target_y)],
                ))
                self.assertIn('moving_box', controller.model_poses)

                controller._model_states_cb(malformed)

                self.assertEqual(
                    ('moving_box',), controller.invalid_model_poses
                )
                self.assertNotIn('moving_box', controller.model_poses)
                controller._control_loop(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                emitted = controller.cmd_vel_pubs['tb3_0'].messages
                self.assertTrue(emitted)
                self.assertTrue(all(
                    command.linear.x == 0.0
                    and command.angular.z == 0.0
                    for command in emitted
                ))

                # A later complete, finite sample clears only this model's
                # rejection and lets a future task use live geometry again.
                controller._model_states_cb(ModelStates(
                    names=['moving_box'], poses=[Pose(x=4.0, y=4.0)]
                ))
                self.assertEqual((), controller.invalid_model_poses)
                self.assertEqual(
                    (4.0, 4.0, 0.0), controller.model_poses['moving_box']
                )

    def test_non_finite_model_states_replace_motion_with_a_stop(self):
        bad_yaw_pose = Pose(x=4.0, y=4.0)
        bad_yaw_pose.orientation = Quaternion(z=float('nan'))
        infinite_quaternion_pose = Pose(x=4.0, y=4.0)
        infinite_quaternion_pose.orientation = Quaternion(z=float('inf'))
        bad_poses = (
            ('nan_x', Pose(x=float('nan'), y=4.0)),
            ('infinite_y', Pose(x=4.0, y=float('inf'))),
            ('nan_yaw', bad_yaw_pose),
            ('infinite_raw_quaternion', infinite_quaternion_pose),
        )

        for label, bad_pose in bad_poses:
            with self.subTest(case=label):
                controller = self._controller_with_a_positive_entry_command()
                previous_count = len(
                    controller.cmd_vel_pubs['tb3_0'].messages
                )

                controller._model_states_cb(ModelStates(
                    names=['moving_box'], poses=[bad_pose]
                ))
                self.assertEqual(
                    ('moving_box',), controller.invalid_model_poses
                )
                self.assertNotIn('moving_box', controller.model_poses)
                controller._control_loop(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                emitted = controller.cmd_vel_pubs['tb3_0'].messages[
                    previous_count:
                ]
                self.assertTrue(emitted)
                self.assertTrue(all(
                    command.linear.x == 0.0
                    and command.angular.z == 0.0
                    for command in emitted
                ))

    def test_non_finite_raw_odometry_quaternion_does_not_refresh_pose(self):
        controller = self._controller_with_a_positive_entry_command()
        previous_pose = controller.robot_poses['tb3_0']
        previous_yaw = controller.robot_yaws['tb3_0']
        previous_stamp = controller.odom_received_at['tb3_0']
        previous_count = len(controller.cmd_vel_pubs['tb3_0'].messages)
        message = Odometry()
        message.pose.pose.position.x = -1.0
        message.pose.pose.orientation.z = float('inf')

        controller._odom_cb(message, 'tb3_0')

        self.assertEqual(('tb3_0',), controller.invalid_robot_poses)
        self.assertIs(previous_pose, controller.robot_poses['tb3_0'])
        self.assertEqual(previous_yaw, controller.robot_yaws['tb3_0'])
        self.assertEqual(previous_stamp, controller.odom_received_at['tb3_0'])
        controller._control_loop(None)

        self.assertFalse(controller.is_running)
        emitted = controller.cmd_vel_pubs['tb3_0'].messages[previous_count:]
        self.assertTrue(emitted)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in emitted
        ))

    def test_live_safety_exception_replaces_motion_with_a_stop(self):
        controller = self._controller_with_a_positive_entry_command()
        previous_count = len(controller.cmd_vel_pubs['tb3_0'].messages)

        with mock.patch.object(
            controller,
            '_live_motion_is_safe_locked',
            side_effect=RuntimeError('malformed live geometry'),
        ):
            controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED, controller.formation_state
        )
        self.assertIn('safety validation failed', controller.placement_error)
        emitted = controller.cmd_vel_pubs['tb3_0'].messages[previous_count:]
        self.assertTrue(emitted)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in emitted
        ))

    def test_unexpected_live_data_error_replaces_motion_with_a_stop(self):
        class MalformedScanAvoidance:
            def update_robot_positions(self, _positions):
                pass

            def apply_avoidance(self, _command):
                raise ValueError('non-finite LaserScan angle metadata')

        controller = self._controller_with_a_positive_entry_command()
        controller.avoidance['tb3_0'] = MalformedScanAvoidance()
        previous_count = len(controller.cmd_vel_pubs['tb3_0'].messages)

        controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED, controller.formation_state
        )
        self.assertIn('invalid live data', controller.placement_error)
        self.assertEqual({}, controller.assignments)
        emitted = controller.cmd_vel_pubs['tb3_0'].messages[previous_count:]
        self.assertTrue(emitted)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in emitted
        ))

    def test_invalid_velocity_command_fails_closed_before_publish(self):
        cases = (
            ('nan_linear', 'linear', 'x', float('nan'), None, None),
            ('infinite_angular', 'angular', 'z', float('inf'), None, None),
            ('linear_over_limit', 'linear', 'x', 0.23, None, None),
            ('angular_over_limit', 'angular', 'z', 1.51, None, None),
            ('unsupported_linear_y', 'linear', 'y', 0.01, None, None),
            ('unsupported_linear_z', 'linear', 'z', -0.01, None, None),
            ('unsupported_angular_x', 'angular', 'x', 0.01, None, None),
            ('unsupported_angular_y', 'angular', 'y', -0.01, None, None),
            ('negative_linear_limit', 'linear', 'x', 0.0, -0.1, None),
            ('negative_angular_limit', 'angular', 'z', 0.0, None, -0.1),
            ('burger_linear_limit', 'linear', 'x', 0.23, 1.0, None),
            ('burger_angular_limit', 'angular', 'z', 2.85, None, 10.0),
        )

        for (
            label, vector, field, value, linear_limit, angular_limit
        ) in cases:
            with self.subTest(case=label):
                controller = self._controller_with_a_positive_entry_command()
                if linear_limit is not None:
                    controller.max_linear_vel = linear_limit
                if angular_limit is not None:
                    controller.max_angular_vel = angular_limit
                previous_count = len(
                    controller.cmd_vel_pubs['tb3_0'].messages
                )
                generation = controller._assignment_generation
                late_snapshot = {
                    'generation': generation,
                    'robot_ids': tuple(controller.robot_ids),
                    'robot_positions': tuple(
                        (
                            controller.robot_poses[robot_id].position.x,
                            controller.robot_poses[robot_id].position.y,
                        )
                        for robot_id in controller.robot_ids
                    ),
                    'target_world': tuple(controller._get_world_targets()),
                    'previous_slots': tuple(
                        controller.assignments.get(robot_id)
                        for robot_id in controller.robot_ids
                    ),
                    'switch_penalty': (controller.spacing * 0.35) ** 2,
                    'placement_plan': dict(
                        controller.active_placement_plan
                    ),
                }

                class InvalidCommandAvoidance(PassThroughAvoidance):
                    def apply_avoidance(self, _command):
                        command = Twist()
                        setattr(getattr(command, vector), field, value)
                        return command

                controller.avoidance['tb3_0'] = InvalidCommandAvoidance()

                controller._control_loop(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertIn(
                    'non-finite or out-of-bounds',
                    controller.placement_error,
                )
                self.assertGreater(
                    controller._assignment_generation, generation
                )
                self.assertFalse(controller.assignment_pending)
                self.assertEqual({}, controller.assignments)
                self.assertFalse(
                    controller._compute_and_commit_assignment(late_snapshot)
                )
                self.assertEqual({}, controller.assignments)
                emitted = controller.cmd_vel_pubs['tb3_0'].messages[
                    previous_count:
                ]
                self.assertTrue(emitted)
                self.assertTrue(all(
                    math.isfinite(command.linear.x)
                    and math.isfinite(command.angular.z)
                    and command.linear.x == 0.0
                    and command.angular.z == 0.0
                    for command in emitted
                ))

    def test_missing_active_plan_cannot_publish_motion(self):
        controller = self._controller_with_a_positive_entry_command()
        previous_count = len(controller.cmd_vel_pubs['tb3_0'].messages)
        generation = controller._assignment_generation
        controller.active_placement_plan = None

        controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertGreater(controller._assignment_generation, generation)
        self.assertEqual({}, controller.assignments)
        emitted = controller.cmd_vel_pubs['tb3_0'].messages[previous_count:]
        self.assertTrue(emitted)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in emitted
        ))

    def test_non_finite_odometry_after_moving_is_rejected_atomically(self):
        cases = (
            ('nan_x', float('nan'), 0.0, None),
            ('infinite_x', float('inf'), 0.0, None),
            ('nan_y', 0.0, float('nan'), None),
            ('infinite_y', 0.0, float('inf'), None),
            ('nan_yaw', 0.0, 0.0, float('nan')),
            ('infinite_yaw', 0.0, 0.0, float('inf')),
        )

        for label, x, y, yaw_override in cases:
            with self.subTest(case=label):
                controller = self._controller_with_a_positive_entry_command()
                controller._initial_formation_acquired = True
                previous_pose = controller.robot_poses['tb3_0']
                previous_yaw = controller.robot_yaws['tb3_0']
                previous_stamp = controller.odom_received_at['tb3_0']
                previous_count = len(
                    controller.cmd_vel_pubs['tb3_0'].messages
                )

                message = Odometry()
                message.pose.pose.position.x = x
                message.pose.pose.position.y = y
                if yaw_override is None:
                    controller._odom_cb(message, 'tb3_0')
                else:
                    with mock.patch.object(
                        formation,
                        'quaternion_to_yaw',
                        return_value=yaw_override,
                    ):
                        controller._odom_cb(message, 'tb3_0')

                self.assertEqual(
                    ('tb3_0',), controller.invalid_robot_poses
                )
                self.assertIs(
                    previous_pose, controller.robot_poses['tb3_0']
                )
                self.assertEqual(previous_yaw, controller.robot_yaws['tb3_0'])
                self.assertEqual(
                    previous_stamp, controller.odom_received_at['tb3_0']
                )

                controller._control_loop(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertEqual(['tb3_0'], controller.stale_odometry)
                emitted = controller.cmd_vel_pubs['tb3_0'].messages[
                    previous_count:
                ]
                self.assertTrue(emitted)
                self.assertTrue(all(
                    command.linear.x == 0.0
                    and command.angular.z == 0.0
                    for command in emitted
                ))

    def test_live_gate_rejects_non_finite_robot_geometry_after_moving(self):
        cases = (
            ('nan_x', 'x', float('nan')),
            ('infinite_x', 'x', float('inf')),
            ('nan_y', 'y', float('nan')),
            ('infinite_y', 'y', float('inf')),
            ('nan_yaw', 'yaw', float('nan')),
            ('infinite_yaw', 'yaw', float('inf')),
        )

        for label, field, value in cases:
            with self.subTest(case=label):
                controller = self._controller_with_a_positive_entry_command()
                controller._initial_formation_acquired = True
                previous_count = len(
                    controller.cmd_vel_pubs['tb3_0'].messages
                )
                if field == 'yaw':
                    controller.robot_yaws['tb3_0'] = value
                else:
                    setattr(
                        controller.robot_poses['tb3_0'].position,
                        field,
                        value,
                    )

                controller._control_loop(None)

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                self.assertIn(
                    'non-finite planar pose', controller.placement_error
                )
                emitted = controller.cmd_vel_pubs['tb3_0'].messages[
                    previous_count:
                ]
                self.assertTrue(emitted)
                self.assertTrue(all(
                    command.linear.x == 0.0
                    and command.angular.z == 0.0
                    for command in emitted
                ))

    def test_odometry_expiring_during_control_stops_before_first_twist(self):
        controller = self._prepared_positive_entry_controller()
        controller.task_started_at = 10.0
        controller.odom_received_at = {'tb3_0': 10.0}
        controller.odom_confirmed_for_task = {'tb3_0': True}
        clock = [10.0]

        def expire_during_live_geometry(*_args, **_kwargs):
            clock[0] = 11.0
            return True

        with mock.patch.object(
            controller,
            '_live_motion_is_safe_locked',
            side_effect=expire_during_live_geometry,
        ), mock.patch.object(
            formation.time, 'monotonic', side_effect=lambda: clock[0]
        ):
            controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED, controller.formation_state
        )
        self.assertEqual(['tb3_0'], controller.stale_odometry)
        self.assertIn('Odometry became stale', controller.placement_error)
        emitted = controller.cmd_vel_pubs['tb3_0'].messages
        self.assertTrue(emitted)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in emitted
        ))


class FormationLifecycleTests(unittest.TestCase):
    @staticmethod
    def wait_until(predicate, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return predicate()

    def test_obstacle_steering_uses_the_formation_velocity_envelope(self):
        class DenseSteeringAvoidance:
            def __init__(self, _robot_id):
                self.max_linear_velocity = 0.22
                self.max_angular_velocity = 2.84
                self.max_linear_acceleration = 0.47
                self.max_angular_acceleration = 2.7
                self.smoothing_alpha = 0.37

            def apply_avoidance(self, _command):
                safe = Twist()
                safe.linear.x = max(
                    -self.max_linear_velocity,
                    min(self.max_linear_velocity, 0.2),
                )
                # Dense neighbouring robots request a stronger turn than the
                # formation controller permits.
                safe.angular.z = max(
                    -self.max_angular_velocity,
                    min(self.max_angular_velocity, 2.4),
                )
                return safe

        controller = make_controller()
        with mock.patch.object(
            formation,
            'ObstacleAvoidance',
            DenseSteeringAvoidance,
        ), mock.patch.object(
            formation.rospy,
            'Publisher',
            side_effect=lambda *_args, **_kwargs: FakePublisher(),
            create=True,
        ), mock.patch.object(
            formation.rospy,
            'Subscriber',
            side_effect=lambda *_args, **_kwargs: FakeResource(),
            create=True,
        ):
            controller._add_robot('tb3_1')

        avoidance = controller.avoidance['tb3_1']
        self.assertNotIn('tb3_1', controller.invalid_avoidance_limits)
        self.assertEqual(0.22, avoidance.max_linear_velocity)
        self.assertEqual(1.5, avoidance.max_angular_velocity)
        self.assertEqual(0.47, avoidance.max_linear_acceleration)
        self.assertEqual(2.7, avoidance.max_angular_acceleration)
        self.assertEqual(0.37, avoidance.smoothing_alpha)
        safe = avoidance.apply_avoidance(Twist())
        self.assertLessEqual(abs(safe.linear.x), 0.22)
        self.assertLessEqual(abs(safe.angular.z), 1.5)

    def test_invalid_avoidance_or_owner_limits_lock_both_axes(self):
        class ConfigurableAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

        cases = (
            ('nan', 'max_angular_velocity', float('nan')),
            ('infinite', 'max_linear_velocity', float('inf')),
            ('negative', 'max_angular_velocity', -0.1),
        )
        for label, field, value in cases:
            with self.subTest(case=label):
                controller = make_controller()
                avoidance = ConfigurableAvoidance()
                setattr(avoidance, field, value)

                self.assertFalse(
                    controller._align_avoidance_motion_limits(avoidance)
                )
                self.assertEqual(0.0, avoidance.max_linear_velocity)
                self.assertEqual(0.0, avoidance.max_angular_velocity)

        controller = make_controller()
        controller._motion_limits_valid = False
        avoidance = ConfigurableAvoidance()
        self.assertFalse(
            controller._align_avoidance_motion_limits(avoidance)
        )
        self.assertEqual(0.0, avoidance.max_linear_velocity)
        self.assertEqual(0.0, avoidance.max_angular_velocity)
        self.assertIsNone(controller._validated_motion_command(Twist()))

        controller = make_controller()
        controller.is_running = True
        now = time.monotonic()
        controller.task_started_at = now - 0.1
        controller.odom_received_at = {'tb3_0': now}
        controller.odom_confirmed_for_task = {'tb3_0': True}
        controller.invalid_avoidance_limits = ('tb3_0',)

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertIn(
            'Obstacle-avoidance velocity limits were invalid',
            controller.placement_error,
        )

    def test_start_rejects_a_missing_or_invalid_task_id(self):
        controller = make_controller()

        controller._start_cb(String(data=json.dumps({
            'formation_type': 'triangle',
        })))
        controller._start_cb(String(data=json.dumps({
            'task_id': 42,
        })))

        self.assertFalse(controller.is_running)
        self.assertIsNone(controller.current_task_id)
        self.assertEqual(
            formation.FormationState.IDLE,
            controller.formation_state,
        )

    def test_stop_arriving_before_start_tombstones_the_task(self):
        controller = make_controller()
        controller.current_task_id = 'older-task'
        stop = String(data=json.dumps({'task_id': 'cancelled-task'}))

        controller._stop_cb(stop)
        controller._start_cb(String(data=json.dumps({
            'task_id': 'cancelled-task',
            'formation_type': 'line',
        })))

        self.assertIn('cancelled-task', controller._cancelled_task_ids)
        self.assertFalse(controller.is_running)
        self.assertEqual('older-task', controller.current_task_id)
        self.assertEqual(
            formation.FormationState.IDLE,
            controller.formation_state,
        )

    def test_orderly_shutdown_zeroes_every_formation_command(self):
        controller = make_controller()
        controller.is_running = True
        controller._initial_formation_acquired = True

        controller._shutdown()

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertFalse(controller._initial_formation_acquired)
        for publisher in controller.cmd_vel_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
        self.assertTrue(controller.safety_stop_request_pub.messages)
        request = json.loads(
            controller.safety_stop_request_pub.messages[-1].data
        )
        self.assertEqual('formation', request['source'])
        self.assertEqual('shutdown', request['reason'])
        self.assertIsInstance(request['request_id'], str)
        self.assertTrue(request['request_id'])
        self.assertTrue(
            controller._shutdown_supervisor_delivery[
                'acknowledgement_received'
            ]
        )
        self.assertTrue(
            controller._shutdown_supervisor_delivery[
                'publication_confirmed'
            ]
        )

    def test_supervisor_request_without_a_subscriber_is_not_confirmed(self):
        controller = make_controller()
        controller.safety_stop_request_pub = FakePublisher()
        controller.safety_stop_request_pub.connections = 0

        delivery = controller._request_supervised_shutdown_stop(
            time.monotonic() + 0.03
        )

        self.assertTrue(delivery['attempted'])
        self.assertFalse(delivery['subscriber_connected'])
        self.assertFalse(delivery['acknowledgement_received'])
        self.assertFalse(delivery['publication_confirmed'])
        self.assertIn('no ROS subscriber', delivery['error'])

    def test_supervisor_ack_for_another_request_is_ignored(self):
        controller = make_controller()
        publisher = FakePublisher()

        def publish_with_wrong_ack(_publisher, message):
            FakePublisher.publish(publisher, message)
            controller._safety_stop_ack_cb(String(data=json.dumps({
                'request_id': 'another-request',
                'accepted': True,
                'supervisor_latched': True,
            })))

        publisher.publish = types.MethodType(
            publish_with_wrong_ack, publisher
        )
        controller.safety_stop_request_pub = publisher

        delivery = controller._request_supervised_shutdown_stop(
            time.monotonic() + 0.03
        )

        self.assertTrue(delivery['publication_returned'])
        self.assertFalse(delivery['acknowledgement_received'])
        self.assertFalse(delivery['publication_confirmed'])
        self.assertIn('acknowledgement', delivery['error'])

    def test_blocked_supervisor_request_is_bounded_and_reported(self):
        controller = make_controller()
        controller.current_task_id = 'blocked-supervisor-request'
        controller.is_running = True
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release = threading.Event()
        request_publisher = BlockingPublisher(release)
        controller.safety_stop_request_pub = request_publisher

        started_at = time.monotonic()
        try:
            controller._shutdown()
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.25)
            self.assertTrue(request_publisher.entered.is_set())
            delivery = controller._shutdown_supervisor_delivery
            self.assertTrue(delivery['attempted'])
            self.assertFalse(delivery['publication_confirmed'])
            self.assertIn('deadline', delivery['error'])
            status = json.loads(controller.status_pub.messages[-1].data)
            self.assertEqual('failed', status['state'])
            self.assertFalse(
                status['supervisor_stop_request'][
                    'publication_confirmed'
                ]
            )
            self.assertIn('safety-stop request', status['error'])
            command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
        finally:
            release.set()

    def test_supervisor_request_uses_prestarted_lane_if_threads_fail(self):
        controller = make_controller()
        publisher = controller.safety_stop_request_pub
        fallback_lane = formation.SafetyPublishLane(
            'test-supervisor-request'
        )
        self.assertTrue(fallback_lane.available)
        controller._safety_fallback_lanes[id(publisher)] = fallback_lane
        original_start = threading.Thread.start
        failed_starts = []

        def fail_request_workers(worker):
            if worker.name.startswith(
                'formation-supervised-stop-request-'
            ):
                failed_starts.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        try:
            with mock.patch.object(
                formation.threading.Thread,
                'start',
                new=fail_request_workers,
            ):
                delivery = controller._request_supervised_shutdown_stop(
                    time.monotonic() + 0.2
                )
            self.assertEqual(2, len(failed_starts))
            self.assertTrue(delivery['publication_returned'])
            self.assertTrue(delivery['acknowledgement_received'])
            self.assertTrue(delivery['publication_confirmed'])
        finally:
            fallback_lane.close()

    def test_stop_failure_is_correlated_and_a_retry_remains_idempotent(self):
        controller = make_controller()
        controller.current_task_id = 'correlated-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        first = FakePublisher()
        flaky = FlakyPublisher(failures=1)
        controller.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': flaky,
        }
        command = String(data=json.dumps({
            'task_id': 'correlated-stop-task'
        }))

        controller._stop_cb(command)

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        failed_status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('correlated-stop-task', failed_status['task_id'])
        self.assertEqual('failed', failed_status['state'])
        self.assertIn(
            'Zero-velocity publication was not accepted',
            failed_status['error'],
        )
        self.assertEqual(
            {
                'task_id': 'correlated-stop-task',
                'attempt': 1,
                'reason': 'task_stop',
                'requested_count': 2,
                'accepted_count': 1,
                'failed_robots': ['tb3_1'],
                'unconfirmed_publishers': [{
                    'robot_id': 'tb3_1',
                    'publisher_generation': 3,
                    'task_id': 'correlated-stop-task',
                    'reason': 'task_stop',
                    'first_attempt': 1,
                    'last_attempt': 1,
                }],
                'publication_confirmed': False,
            },
            failed_status['stop_publication'],
        )

        controller._stop_cb(command)

        self.assertEqual(
            formation.FormationState.STOPPED,
            controller.formation_state,
        )
        recovered_status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('stopped', recovered_status['state'])
        self.assertNotIn('error', recovered_status)
        self.assertEqual(2, recovered_status['stop_publication']['attempt'])
        self.assertTrue(
            recovered_status['stop_publication']['publication_confirmed']
        )
        self.assertEqual([], recovered_status['stop_publication']['failed_robots'])
        self.assertEqual(2, len(first.messages))
        self.assertEqual(2, flaky.calls)
        self.assertEqual(1, len(flaky.messages))

    def test_pause_and_emergency_stop_fail_without_claiming_stopped(self):
        cases = (
            (
                'pause',
                'task_pause',
                lambda controller: controller._pause_cb(String(
                    data=json.dumps({'task_id': controller.current_task_id})
                )),
            ),
            (
                'emergency',
                'emergency_stop',
                lambda controller: controller._emergency_stop_cb(
                    Bool(data=True)
                ),
            ),
        )
        for label, reason, invoke in cases:
            with self.subTest(command=label):
                controller = make_controller()
                controller.current_task_id = 'lifecycle-stop-task'
                controller.is_running = True
                controller.robot_ids = ['tb3_0', 'tb3_1']
                controller.cmd_vel_pubs = {
                    'tb3_0': FakePublisher(),
                    'tb3_1': FlakyPublisher(failures=10),
                }

                invoke(controller)

                self.assertFalse(controller.is_running)
                self.assertFalse(controller.is_paused)
                self.assertEqual(
                    formation.FormationState.FAILED,
                    controller.formation_state,
                )
                status = json.loads(controller.status_pub.messages[-1].data)
                self.assertEqual('failed', status['state'])
                self.assertNotEqual('stopped', status['state'])
                self.assertEqual(reason, status['stop_publication']['reason'])
                self.assertFalse(
                    status['stop_publication']['publication_confirmed']
                )
                self.assertEqual(
                    ['tb3_1'], status['stop_publication']['failed_robots']
                )

    def test_start_does_not_erase_an_unconfirmed_previous_stop(self):
        controller = make_controller()
        controller.current_task_id = 'old-task'
        controller._last_stop_publication = {
            'task_id': 'old-task',
            'attempt': 3,
            'reason': 'task_stop',
            'requested_count': 1,
            'accepted_count': 0,
            'failed_robots': ['tb3_0'],
            'publication_confirmed': False,
        }

        controller._start_cb(String(data=json.dumps({
            'task_id': 'new-task',
            'formation_type': 'line',
        })))

        self.assertFalse(controller.is_running)
        self.assertEqual('old-task', controller.current_task_id)
        self.assertEqual(
            'old-task', controller._last_stop_publication['task_id']
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('old-task', status['task_id'])
        self.assertFalse(
            status['stop_publication']['publication_confirmed']
        )

    def test_shutdown_is_bounded_and_does_not_claim_worker_exit(self):
        controller = make_controller()
        controller.current_task_id = 'bounded-shutdown-task'
        controller.is_running = True
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.02
        worker_started = threading.Event()
        release_worker = threading.Event()

        def blocked_worker():
            worker_started.set()
            release_worker.wait(1.0)

        worker = threading.Thread(target=blocked_worker)
        controller._assignment_worker = worker
        worker.start()
        self.assertTrue(worker_started.wait(0.5))
        started_at = time.monotonic()
        try:
            controller._shutdown()
            elapsed = time.monotonic() - started_at
            self.assertLess(elapsed, 0.5)
            self.assertTrue(worker.is_alive())
            self.assertTrue(controller._assignment_worker_stop.is_set())
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
            status = json.loads(controller.status_pub.messages[-1].data)
            self.assertEqual('failed', status['state'])
            self.assertIn(
                'bounded shutdown deadline', status['error']
            )
            self.assertTrue(
                status['stop_publication']['publication_confirmed']
            )
        finally:
            release_worker.set()
            worker.join(1.0)

        self.assertFalse(worker.is_alive())

    def test_shutdown_publishes_command_socket_failure_without_hanging(self):
        controller = make_controller()
        controller.current_task_id = 'shutdown-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.cmd_vel_pubs = {
            'tb3_0': FakePublisher(),
            'tb3_1': FlakyPublisher(failures=10),
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        controller._shutdown()

        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('shutdown-stop-task', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertEqual('shutdown', status['stop_publication']['reason'])
        self.assertFalse(
            status['stop_publication']['publication_confirmed']
        )

    def test_stale_odometry_is_reported_after_the_start_grace_period(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_timeout_wall_s = 0.5
        controller.odom_received_at = {'tb3_0': 10.0}
        controller.odom_confirmed_for_task = {'tb3_0': True}

        self.assertEqual(
            ['tb3_0'], controller._stale_odometry(['tb3_0'], now=10.6)
        )
        self.assertEqual(
            [], controller._stale_odometry(['tb3_0'], now=10.4)
        )

    def test_never_received_odometry_gets_a_bounded_startup_grace(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_initialization_timeout_wall_s = 10.0
        controller.odom_received_at = {'tb3_0': None}
        controller.odom_confirmed_for_task = {'tb3_0': False}

        self.assertEqual(
            (['tb3_0'], []),
            controller._odometry_readiness(['tb3_0'], now=20.0),
        )
        self.assertEqual(
            ([], ['tb3_0']),
            controller._odometry_readiness(['tb3_0'], now=20.0001),
        )

    def test_prestart_sample_waits_for_one_sample_from_the_new_task(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_timeout_wall_s = 0.75
        controller.odom_initialization_timeout_wall_s = 10.0
        controller.odom_received_at = {'tb3_0': 9.9}
        controller.odom_confirmed_for_task = {'tb3_0': False}

        self.assertEqual(
            (['tb3_0'], []),
            controller._odometry_readiness(['tb3_0'], now=19.9),
        )

    def test_first_post_start_sample_enables_the_strict_stale_timeout(self):
        controller = make_controller()
        controller.task_started_at = 10.0
        controller.odom_timeout_wall_s = 0.75
        controller.odom_initialization_timeout_wall_s = 10.0
        controller.odom_received_at = {'tb3_0': 10.1}
        controller.odom_confirmed_for_task = {'tb3_0': True}

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
        controller.odom_confirmed_for_task = {
            'tb3_0': True, 'tb3_1': False,
        }
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
        controller.odom_confirmed_for_task = {'tb3_0': False}

        with mock.patch.object(formation.time, 'monotonic', return_value=20.0001):
            controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED, controller.formation_state
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(['tb3_0'], status['stale_odometry'])
        self.assertIn('tb3_0', status['error'])

    def test_control_failure_reports_an_unaccepted_stop_publication(self):
        controller = make_controller()
        controller.current_task_id = 'control-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_poses['tb3_1'] = Pose()
        controller.robot_yaws['tb3_1'] = 0.0
        controller.cmd_vel_pubs = {
            'tb3_0': FakePublisher(),
            'tb3_1': FlakyPublisher(failures=10),
        }
        controller.placement_error = 'Synthetic control failure.'

        controller._control_step(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('control-stop-task', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertEqual(
            'control_failure', status['stop_publication']['reason']
        )
        self.assertFalse(
            status['stop_publication']['publication_confirmed']
        )
        self.assertIn(
            'Zero-velocity publication was not accepted for: tb3_1',
            status['error'],
        )

    def test_lifecycle_commands_require_the_exact_nonempty_task_id(self):
        controller = make_controller()
        controller.current_task_id = 'task-current'
        controller.is_running = True
        controller._initial_formation_acquired = True
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
        self.assertTrue(controller._initial_formation_acquired)
        self.assertEqual(publisher.messages, [])

        controller._stop_cb(String(data=json.dumps({
            'task_id': 'task-current'
        })))
        self.assertFalse(controller.is_running)
        self.assertFalse(controller._initial_formation_acquired)
        self.assertEqual(
            controller.formation_state, formation.FormationState.STOPPED
        )
        self.assertEqual(len(publisher.messages), 1)

    def test_emergency_stop_reaches_healthy_robot_past_blocked_socket(self):
        controller = make_controller()
        controller.current_task_id = 'emergency-fanout-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.EMERGENCY_STOP_TIMEOUT = 0.05
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        started_at = time.monotonic()
        try:
            controller._emergency_stop_cb(Bool(data=True))
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.2)
            self.assertTrue(blocked.entered.is_set())
            self.assertTrue(healthy.messages)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
            self.assertIn('tb3_0', controller.placement_error)
        finally:
            release_blocked.set()

    def test_regular_stop_reaches_healthy_robot_past_blocked_socket(self):
        controller = make_controller()
        controller.current_task_id = 'regular-fanout-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.EMERGENCY_STOP_TIMEOUT = 0.05
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        try:
            started_at = time.monotonic()
            controller._stop_cb(String(data=json.dumps({
                'task_id': 'regular-fanout-task',
            })))
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.2)
            self.assertTrue(blocked.entered.is_set())
            self.assertTrue(healthy.messages)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
            self.assertIn('tb3_0', controller.placement_error)
        finally:
            release_blocked.set()

    def test_emergency_stop_preempts_a_regular_stop_waiting_on_its_lock(self):
        controller = make_controller()
        controller.current_task_id = 'regular-stop-lock-race'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.EMERGENCY_STOP_TIMEOUT = 0.4
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        regular_stop = threading.Thread(
            target=controller._stop_cb,
            args=(String(data=json.dumps({
                'task_id': 'regular-stop-lock-race',
            })),),
        )
        emergency_stop = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        regular_stop.start()
        self.assertTrue(blocked.entered.wait(0.5))
        self.assertTrue(self.wait_until(lambda: healthy.messages))
        first_zero_count = len(healthy.messages)

        try:
            emergency_stop.start()
            self.assertTrue(self.wait_until(lambda: (
                controller.emergency_stop_active
                and len(healthy.messages) > first_zero_count
            )))
            self.assertTrue(regular_stop.is_alive())
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
        finally:
            release_blocked.set()
            regular_stop.join(1.0)
            emergency_stop.join(1.0)

        self.assertFalse(regular_stop.is_alive())
        self.assertFalse(emergency_stop.is_alive())
        self.assertTrue(controller.emergency_stop_active)

    def test_start_waiting_on_pose_lock_cannot_outlive_emergency_stop(self):
        controller = make_controller()
        controller.current_task_id = 'old-formation'
        controller.EMERGENCY_STOP_TIMEOUT = 0.1
        controller.lock.acquire()
        start = threading.Thread(
            target=controller._start_cb,
            args=(String(data=json.dumps({
                'task_id': 'new-formation',
                'formation_type': 'triangle',
            })),),
        )
        emergency = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        start.start()

        start_has_command_lock = False
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if not controller.command_lock.acquire(blocking=False):
                start_has_command_lock = True
                break
            controller.command_lock.release()
            time.sleep(0.005)
        self.assertTrue(start_has_command_lock)

        try:
            emergency.start()
            self.assertTrue(self.wait_until(lambda: (
                controller.emergency_stop_active
            )))
        finally:
            controller.lock.release()
            start.join(1.0)
            emergency.join(1.0)

        self.assertFalse(start.is_alive())
        self.assertFalse(emergency.is_alive())
        self.assertTrue(controller.emergency_stop_active)
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertEqual(
            formation.FormationState.STOPPED,
            controller.formation_state,
        )
        self.assertEqual('old-formation', controller.current_task_id)

        controller._emergency_stop_cb(Bool(data=False))
        self.assertFalse(controller.emergency_stop_active)
        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.STOPPED,
            controller.formation_state,
        )

    def test_newer_emergency_edge_wins_over_an_older_queued_reset(self):
        controller = make_controller()
        controller.emergency_stop_active = True
        controller.command_lock.acquire()
        reset = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=False),),
        )
        emergency = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        reset.start()
        self.assertTrue(self.wait_until(lambda: (
            controller._emergency_stop_generation >= 1
        )))
        emergency.start()
        self.assertTrue(self.wait_until(lambda: (
            controller._emergency_stop_generation >= 2
            and controller.emergency_stop_active
        )))

        controller.command_lock.release()
        reset.join(1.0)
        emergency.join(1.0)

        self.assertFalse(reset.is_alive())
        self.assertFalse(emergency.is_alive())
        self.assertTrue(controller.emergency_stop_active)
        self.assertFalse(controller.is_running)
        self.assertEqual(
            formation.FormationState.STOPPED,
            controller.formation_state,
        )

    def test_reset_waits_for_every_preceding_emergency_callback(self):
        controller = make_controller()
        controller.current_task_id = 'formation-before-estop'
        controller.is_running = True
        controller.EMERGENCY_STOP_TIMEOUT = 0.1
        first_entered = threading.Event()
        release_first = threading.Event()
        call_lock = threading.Lock()
        call_count = [0]
        stop_from_snapshot = controller._stop_from_shutdown_snapshot

        def controlled_stop(*args, **kwargs):
            with call_lock:
                call_count[0] += 1
                current_call = call_count[0]
            if current_call == 1:
                first_entered.set()
                release_first.wait(1.0)
            return stop_from_snapshot(*args, **kwargs)

        controller._stop_from_shutdown_snapshot = controlled_stop
        first = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        second = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        reset = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=False),),
        )

        first.start()
        self.assertTrue(first_entered.wait(0.5))
        second.start()
        second.join(1.0)
        self.assertFalse(second.is_alive())
        self.assertTrue(first.is_alive())

        reset.start()
        self.assertTrue(self.wait_until(lambda: (
            controller._emergency_stop_generation >= 3
        )))
        time.sleep(0.03)
        self.assertTrue(reset.is_alive())

        try:
            release_first.set()
            first.join(1.0)
            reset.join(1.0)
        finally:
            release_first.set()
            first.join(1.0)
            reset.join(1.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(reset.is_alive())
        self.assertEqual(set(), controller._emergency_true_pending)
        self.assertFalse(controller.emergency_stop_active)
        self.assertFalse(controller.is_running)

        controller._start_cb(String(data=json.dumps({
            'task_id': 'formation-after-reset',
            'formation_type': 'triangle',
        })))
        self.assertEqual(
            'formation-after-reset', controller.current_task_id
        )
        self.assertTrue(controller.is_running)

    def test_late_motion_after_emergency_stop_is_rezeroed(self):
        controller = make_controller()
        controller.current_task_id = 'emergency-late-motion'
        controller.is_running = True
        controller.EMERGENCY_STOP_TIMEOUT = 0.1
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        controller.cmd_vel_pubs = {'tb3_0': blocked}
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        command = Twist()
        command.linear.x = 0.22
        batch_results = []

        def publish_motion():
            with controller.command_lock:
                with controller.lock:
                    batch_results.append(
                        controller._publish_motion_batch_locked({
                            'tb3_0': command,
                        })
                    )

        motion = threading.Thread(target=publish_motion)
        emergency_stop = threading.Thread(
            target=controller._emergency_stop_cb,
            args=(Bool(data=True),),
        )
        motion.start()
        self.assertTrue(blocked.entered.wait(0.5))

        try:
            emergency_stop.start()
            self.assertTrue(self.wait_until(lambda: (
                controller.emergency_stop_active
            )))
        finally:
            release_blocked.set()
            motion.join(1.0)
            emergency_stop.join(1.0)

        self.assertFalse(motion.is_alive())
        self.assertFalse(emergency_stop.is_alive())
        self.assertEqual([False], batch_results)
        self.assertTrue(any(
            message.linear.x > 0.0 for message in blocked.messages
        ))
        self.assertTrue(self.wait_until(lambda: (
            blocked.messages
            and blocked.messages[-1].linear.x == 0.0
            and blocked.messages[-1].angular.z == 0.0
        )))

    def test_empty_fleet_cancels_task_and_tears_down_robot_resources(self):
        controller = make_controller()
        controller.current_task_id = 'task-current'
        controller.is_running = True
        controller.is_paused = True
        controller._initial_formation_acquired = True
        publisher = controller.cmd_vel_pubs['tb3_0']
        subscriber = FakeResource()
        avoidance = FakeResource()
        controller._odom_subs['tb3_0'] = subscriber
        controller.avoidance['tb3_0'] = avoidance

        controller._fleet_list_cb(String(data=''))

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertFalse(controller._initial_formation_acquired)
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

    def test_empty_fleet_keeps_failed_stop_correlated_to_the_task(self):
        controller = make_controller()
        controller.current_task_id = 'empty-fleet-stop-task'
        controller.is_running = True
        broken = FlakyPublisher(failures=10)
        controller.cmd_vel_pubs = {'tb3_0': broken}

        controller._fleet_list_cb(String(data=''))

        self.assertEqual('empty-fleet-stop-task', controller.current_task_id)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertEqual([], controller.robot_ids)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('empty-fleet-stop-task', status['task_id'])
        self.assertEqual('failed', status['state'])
        self.assertEqual(
            'empty-fleet-stop-task',
            status['stop_publication']['task_id'],
        )
        self.assertFalse(
            status['stop_publication']['publication_confirmed']
        )

    def test_partial_fleet_change_does_not_replan_after_stop_failure(self):
        controller = make_controller()
        controller.current_task_id = 'fleet-change-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.robot_poses['tb3_1'] = Pose()
        controller.robot_yaws['tb3_1'] = 0.0
        controller.odom_received_at['tb3_1'] = time.monotonic()
        controller.odom_confirmed_for_task['tb3_1'] = True
        controller.pid_linear['tb3_1'] = FakePid()
        controller.pid_angular['tb3_1'] = FakePid()
        controller._odom_subs['tb3_1'] = FakeResource()
        controller.avoidance['tb3_1'] = FakeResource()
        controller.cmd_vel_pubs = {
            'tb3_0': FakePublisher(),
            'tb3_1': FlakyPublisher(failures=10),
        }

        with mock.patch.object(
            controller, '_recompute_formation_locked'
        ) as replan:
            controller._fleet_list_cb(String(data='tb3_0'))

        replan.assert_not_called()
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual(['tb3_0'], controller.robot_ids)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('failed', status['state'])
        self.assertEqual(
            'fleet_change', status['stop_publication']['reason']
        )
        self.assertFalse(
            status['stop_publication']['publication_confirmed']
        )

    def test_fleet_resize_keeps_the_failed_publisher_debt_until_it_recovers(self):
        controller = make_controller()
        controller.current_task_id = 'resize-stop-task'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.robot_poses['tb3_1'] = Pose()
        controller.robot_yaws['tb3_1'] = 0.0
        controller.odom_received_at['tb3_1'] = time.monotonic()
        controller.odom_confirmed_for_task['tb3_1'] = True
        controller.pid_linear['tb3_1'] = FakePid()
        controller.pid_angular['tb3_1'] = FakePid()
        controller._odom_subs['tb3_1'] = FakeResource()
        controller.avoidance['tb3_1'] = FakeResource()
        healthy = FakePublisher()
        broken = FlakyPublisher(failures=3)
        controller.cmd_vel_pubs = {
            'tb3_0': healthy,
            'tb3_1': broken,
        }

        with mock.patch.object(
            controller, '_recompute_formation_locked', return_value=None
        ):
            controller._fleet_list_cb(String(data='tb3_0'))

        first_status = json.loads(controller.status_pub.messages[-1].data)
        first_debt = first_status['stop_publication'][
            'unconfirmed_publishers'
        ][0]
        generation = first_debt['publisher_generation']
        self.assertEqual('tb3_1', first_debt['robot_id'])
        self.assertEqual('resize-stop-task', first_debt['task_id'])
        self.assertEqual('fleet_change', first_debt['reason'])
        self.assertFalse(broken.unregistered)
        self.assertEqual(1, broken.calls)
        self.assertEqual(['tb3_0'], controller.robot_ids)

        old_task_stop = String(data=json.dumps({
            'task_id': 'resize-stop-task'
        }))
        controller._stop_cb(old_task_stop)
        reduced_status = json.loads(
            controller.status_pub.messages[-1].data
        )
        self.assertFalse(
            reduced_status['stop_publication']['publication_confirmed']
        )
        self.assertEqual(
            ['tb3_1'], reduced_status['stop_publication']['failed_robots']
        )
        self.assertEqual(
            2, reduced_status['stop_publication']['requested_count']
        )
        self.assertEqual(
            generation,
            reduced_status['stop_publication'][
                'unconfirmed_publishers'
            ][0]['publisher_generation'],
        )

        controller._start_cb(String(data=json.dumps({
            'task_id': 'must-not-start'
        })))
        self.assertFalse(controller.is_running)
        self.assertEqual('resize-stop-task', controller.current_task_id)

        retained_avoidance = types.SimpleNamespace(
            max_linear_velocity=0.22,
            max_angular_velocity=2.84,
            shutdown=lambda: None,
        )
        with mock.patch.object(
            formation.rospy, 'Publisher', create=True
        ) as create_publisher, mock.patch.object(
            formation.rospy,
            'Subscriber',
            return_value=FakeResource(),
            create=True,
        ), mock.patch.object(
            formation,
            'ObstacleAvoidance',
            return_value=retained_avoidance,
        ), mock.patch.object(
            controller, '_recompute_formation_locked', return_value=None
        ) as replan:
            controller._fleet_list_cb(String(data='tb3_0,tb3_1'))

        create_publisher.assert_not_called()
        replan.assert_not_called()
        self.assertIs(broken, controller.cmd_vel_pubs['tb3_1'])
        self.assertFalse(broken.unregistered)
        self.assertEqual(4, broken.calls)
        self.assertEqual(2, len(controller._command_publisher_generations))
        self.assertEqual({}, controller._stop_publication_debts)
        recovered_status = json.loads(
            controller.status_pub.messages[-1].data
        )
        self.assertTrue(
            recovered_status['stop_publication']['publication_confirmed']
        )
        self.assertEqual(
            [], recovered_status['stop_publication'][
                'unconfirmed_publishers'
            ]
        )
        self.assertEqual({}, controller._stop_publication_debts)
        self.assertIs(broken, controller.cmd_vel_pubs['tb3_1'])
        self.assertFalse(broken.unregistered)
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.assignment_pending)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )

        with mock.patch.object(
            controller, '_prepare_assignment_locked', return_value=None
        ):
            controller._start_cb(String(data=json.dumps({
                'task_id': 'recovered-task'
            })))
        self.assertTrue(controller.is_running)
        self.assertEqual('recovered-task', controller.current_task_id)

    def test_shutdown_discards_robot_setup_started_before_snapshot(self):
        class ProvisionedAvoidance:
            max_linear_velocity = 0.22
            max_angular_velocity = 2.84

            def __init__(self, _robot_id):
                self.closed = False

            def shutdown(self):
                self.closed = True

        controller = make_controller()
        controller.current_task_id = 'formation-roster-race'
        controller.is_running = True
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        publisher_entered = threading.Event()
        release_publisher = threading.Event()
        created_publishers = []
        created_subscribers = []
        created_avoidance = []

        def blocking_publisher(topic, *_args, **_kwargs):
            if topic == '/tb3_1/cmd_vel':
                publisher_entered.set()
                release_publisher.wait(1.0)
            publisher = FakePublisher()
            created_publishers.append(publisher)
            return publisher

        def recording_subscriber(*_args, **_kwargs):
            subscriber = FakeResource()
            created_subscribers.append(subscriber)
            return subscriber

        def make_avoidance(robot_id):
            avoidance = ProvisionedAvoidance(robot_id)
            created_avoidance.append(avoidance)
            return avoidance

        with mock.patch.object(
            formation.rospy,
            'Publisher',
            side_effect=blocking_publisher,
            create=True,
        ), mock.patch.object(
            formation.rospy,
            'Subscriber',
            side_effect=recording_subscriber,
            create=True,
        ), mock.patch.object(
            formation,
            'ObstacleAvoidance',
            side_effect=make_avoidance,
        ):
            roster = threading.Thread(
                target=controller._fleet_list_cb,
                args=(String(data='tb3_0,tb3_1'),),
            )
            roster.start()
            self.assertTrue(publisher_entered.wait(0.5))
            try:
                controller._shutdown()
            finally:
                release_publisher.set()
                roster.join(1.0)

        self.assertFalse(roster.is_alive())
        self.assertEqual(1, len(created_publishers))
        self.assertTrue(created_publishers[0].unregistered)
        self.assertEqual(1, len(created_subscribers))
        self.assertTrue(created_subscribers[0].closed)
        self.assertEqual(1, len(created_avoidance))
        self.assertTrue(created_avoidance[0].closed)
        self.assertNotIn('tb3_1', controller.robot_ids)
        self.assertNotIn('tb3_1', controller.cmd_vel_pubs)
        self.assertFalse(any(
            robot_id == 'tb3_1'
            for robot_id, _publisher, _generation, _debt
            in controller._shutdown_publisher_snapshot
        ))

    def test_shutdown_deadline_includes_a_blocked_lifecycle_lock(self):
        controller = make_controller()
        controller.current_task_id = 'locked-shutdown-task'
        controller.is_running = True
        controller.assignment_pending = True
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        planner_entered = threading.Event()
        release_planner = threading.Event()

        def planner_holding_both_state_locks():
            with controller.command_lock:
                with controller.lock:
                    planner_entered.set()
                    release_planner.wait(1.0)

        planner = threading.Thread(target=planner_holding_both_state_locks)
        controller._assignment_worker = planner
        planner.start()
        self.assertTrue(planner_entered.wait(0.5))
        started_at = time.monotonic()
        try:
            controller._shutdown()
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.25)
            self.assertTrue(planner.is_alive())
            self.assertTrue(controller._assignment_worker_stop.is_set())
            self.assertFalse(controller.is_running)
            self.assertTrue(controller.assignment_pending)
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
            command = controller.cmd_vel_pubs['tb3_0'].messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
            status = json.loads(controller.status_pub.messages[-1].data)
            self.assertEqual('locked-shutdown-task', status['task_id'])
            self.assertEqual('failed', status['state'])
            self.assertIn('could not acquire the lifecycle lock', status['error'])
            self.assertIn('cleanup were not confirmed', status['error'])
            self.assertTrue(
                status['stop_publication']['publication_confirmed']
            )
        finally:
            release_planner.set()
            planner.join(1.0)

        self.assertFalse(planner.is_alive())

    def test_blocked_publisher_does_not_delay_other_shutdown_stops(self):
        controller = make_controller()
        controller.current_task_id = 'blocked-publisher-shutdown'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_blocked_publisher = threading.Event()
        blocked = BlockingPublisher(release_blocked_publisher)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        started_at = time.monotonic()
        try:
            controller._shutdown()
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.25)
            self.assertTrue(blocked.entered.is_set())
            self.assertEqual(1, len(healthy.messages))
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
            status = json.loads(controller.status_pub.messages[-1].data)
            self.assertEqual('failed', status['state'])
            self.assertEqual(
                2, status['stop_publication']['requested_count']
            )
            self.assertEqual(
                1, status['stop_publication']['accepted_count']
            )
            self.assertEqual(
                ['tb3_0'], status['stop_publication']['failed_robots']
            )
            self.assertFalse(
                status['stop_publication']['publication_confirmed']
            )
        finally:
            release_blocked_publisher.set()

        deadline = time.monotonic() + 0.5
        while not blocked.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(1, len(blocked.messages))

    def test_shutdown_continues_when_the_first_zero_worker_cannot_start(self):
        controller = make_controller()
        controller.current_task_id = 'thread-start-shutdown'
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        first = FakePublisher()
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()
        targets = controller._cached_command_publisher_snapshot()
        original_start = threading.Thread.start
        failed_once = []

        def fail_first_shutdown_worker(worker):
            if (
                worker.name.startswith('formation-shutdown-stop-')
                and not failed_once
            ):
                failed_once.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            formation.threading.Thread,
            'start',
            new=fail_first_shutdown_worker,
        ):
            publication = controller._stop_from_shutdown_snapshot(
                targets,
                'shutdown',
                'thread-start-shutdown',
                time.monotonic() + 0.2,
            )

        self.assertEqual(1, len(failed_once))
        self.assertTrue(first.messages)
        self.assertEqual(0.0, first.messages[-1].linear.x)
        self.assertEqual(0.0, first.messages[-1].angular.z)
        self.assertTrue(healthy.messages)
        self.assertEqual(0.0, healthy.messages[-1].linear.x)
        self.assertEqual(0.0, healthy.messages[-1].angular.z)
        self.assertEqual(2, publication['requested_count'])
        self.assertEqual(2, publication['accepted_count'])
        self.assertEqual([], publication['failed_robots'])
        self.assertTrue(publication['publication_confirmed'])

    def test_shutdown_reaches_later_publishers_after_two_start_failures(self):
        controller = make_controller()
        controller.current_task_id = 'persistent-thread-start-shutdown'
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        first = FakePublisher()
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()
        targets = controller._cached_command_publisher_snapshot()
        first_generation = targets[0]['publisher_generation']
        fallback_lane = formation.SafetyPublishLane('test-formation-tb3-0')
        self.assertTrue(fallback_lane.available)
        controller._safety_fallback_lanes[id(first)] = fallback_lane
        original_start = threading.Thread.start
        failures = []

        def fail_first_target_workers(worker):
            if worker.name.startswith(
                'formation-shutdown-stop-{}-'.format(first_generation)
            ):
                failures.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        try:
            with mock.patch.object(
                formation.threading.Thread,
                'start',
                new=fail_first_target_workers,
            ):
                publication = controller._stop_from_shutdown_snapshot(
                    targets,
                    'shutdown',
                    'persistent-thread-start-shutdown',
                    time.monotonic() + 0.2,
                )
        finally:
            fallback_lane.close()

        self.assertEqual(2, len(failures))
        self.assertTrue(first.messages)
        self.assertEqual(0.0, first.messages[-1].linear.x)
        self.assertEqual(0.0, first.messages[-1].angular.z)
        self.assertTrue(healthy.messages)
        self.assertEqual(0.0, healthy.messages[-1].linear.x)
        self.assertEqual(0.0, healthy.messages[-1].angular.z)
        self.assertEqual(2, publication['accepted_count'])
        self.assertEqual([], publication['failed_robots'])
        self.assertTrue(publication['publication_confirmed'])

    def test_shutdown_bypasses_a_regular_stop_blocked_on_one_publisher(self):
        controller = make_controller()
        controller.current_task_id = 'regular-stop-race'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_blocked_publisher = threading.Event()
        blocked = BlockingPublisher(release_blocked_publisher)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        regular_stop = threading.Thread(
            target=controller._stop_cb,
            args=(String(data=json.dumps({
                'task_id': 'regular-stop-race'
            })),),
        )
        regular_stop.start()
        self.assertTrue(blocked.entered.wait(0.5))

        started_at = time.monotonic()
        try:
            controller._shutdown()
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.25)
            self.assertTrue(regular_stop.is_alive())
            self.assertGreaterEqual(len(healthy.messages), 2)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            status = json.loads(controller.status_pub.messages[-1].data)
            self.assertEqual('failed', status['state'])
            self.assertEqual('regular-stop-race', status['task_id'])
            self.assertEqual(
                2, status['stop_publication']['requested_count']
            )
            self.assertEqual(
                1, status['stop_publication']['accepted_count']
            )
            self.assertEqual(
                ['tb3_0'], status['stop_publication']['failed_robots']
            )
            self.assertFalse(
                status['stop_publication']['publication_confirmed']
            )
            shutdown_publication = controller._shutdown_publication
            shutdown_status_count = len(controller.status_pub.messages)
        finally:
            release_blocked_publisher.set()
            regular_stop.join(1.0)

        self.assertFalse(regular_stop.is_alive())
        self.assertIs(
            shutdown_publication, controller._last_stop_publication
        )
        self.assertEqual({}, controller._stop_publication_debts)
        self.assertEqual(
            shutdown_status_count, len(controller.status_pub.messages)
        )
        self.assertTrue(controller._shutdown_started)
        final_error = controller.placement_error

        controller._stop_cb(String(data=json.dumps({
            'task_id': 'regular-stop-race'
        })))
        controller._emergency_stop_cb(Bool(data=True))
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        self.assertEqual(final_error, controller.placement_error)

        controller._start_cb(String(data=json.dumps({
            'task_id': 'task-after-shutdown'
        })))
        self.assertFalse(controller.is_running)
        self.assertEqual('regular-stop-race', controller.current_task_id)

    def test_late_motion_is_rezeroed_without_releasing_the_rest_of_batch(self):
        controller = make_controller()
        controller.current_task_id = 'motion-shutdown-race'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_blocked_publisher = threading.Event()
        blocked = BlockingPublisher(release_blocked_publisher)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        commands = {'tb3_0': Twist(), 'tb3_1': Twist()}
        commands['tb3_0'].linear.x = 0.22
        commands['tb3_1'].linear.x = 0.22
        batch_results = []

        def publish_regular_batch():
            with controller.command_lock:
                with controller.lock:
                    batch_results.append(
                        controller._publish_motion_batch_locked(commands)
                    )

        regular_control = threading.Thread(target=publish_regular_batch)
        regular_control.start()
        self.assertTrue(blocked.entered.wait(0.5))

        try:
            controller._shutdown()
            self.assertTrue(regular_control.is_alive())
            self.assertEqual(1, len(healthy.messages))
            self.assertEqual(0.0, healthy.messages[0].linear.x)
            self.assertEqual(0.0, healthy.messages[0].angular.z)
        finally:
            release_blocked_publisher.set()
            regular_control.join(1.0)

        self.assertFalse(regular_control.is_alive())
        self.assertEqual([False], batch_results)
        self.assertTrue(any(
            command.linear.x > 0.0 for command in blocked.messages
        ))
        self.assertEqual(0.0, blocked.messages[-1].linear.x)
        self.assertEqual(0.0, blocked.messages[-1].angular.z)
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in healthy.messages
        ))

    def test_late_motion_uses_its_second_zero_when_first_thread_start_fails(self):
        controller = make_controller()
        publisher = FakePublisher()
        original_start = threading.Thread.start
        failed_once = []

        def fail_first_late_zero(worker):
            if (
                worker.name.startswith('formation-late-zero-')
                and not failed_once
            ):
                failed_once.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            formation.threading.Thread,
            'start',
            new=fail_first_late_zero,
        ):
            result = controller._compensate_late_motion(
                'tb3_0', publisher, 7
            )

        self.assertFalse(result)
        self.assertEqual(1, len(failed_once))
        deadline = time.monotonic() + 0.5
        while not publisher.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(publisher.messages)
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)

    def test_late_motion_uses_prestarted_lane_when_both_threads_fail(self):
        controller = make_controller()
        publisher = FakePublisher()
        fallback_lane = formation.SafetyPublishLane(
            'test-formation-late-motion'
        )
        self.assertTrue(fallback_lane.available)
        controller._safety_fallback_lanes[id(publisher)] = fallback_lane
        original_start = threading.Thread.start
        failed_starts = []

        def fail_late_zero_workers(worker):
            if worker.name.startswith('formation-late-zero-'):
                failed_starts.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        try:
            with mock.patch.object(
                formation.threading.Thread,
                'start',
                new=fail_late_zero_workers,
            ):
                result = controller._compensate_late_motion(
                    'tb3_0', publisher, 8
                )
            self.assertFalse(result)
            self.assertEqual(2, len(failed_starts))
            deadline = time.monotonic() + 0.5
            while not publisher.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(publisher.messages)
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)
        finally:
            fallback_lane.close()

    def test_late_motion_uses_lane_after_publish_and_thread_start_failures(self):
        controller = make_controller()
        publisher = FlakyPublisher(failures=1)
        controller._register_safety_fallback_lane('tb3-0', publisher)
        generation = 9
        controller._motion_publish_inflight.add(generation)
        original_start = threading.Thread.start
        starts = []

        def start_first_late_zero_only(worker):
            if not worker.name.startswith('formation-late-zero-'):
                return original_start(worker)
            starts.append(worker.name)
            if len(starts) == 2:
                raise RuntimeError('thread capacity exhausted')
            result = original_start(worker)
            deadline = time.monotonic() + 0.5
            while publisher.calls < 1 and time.monotonic() < deadline:
                time.sleep(0.005)
            return result

        with mock.patch.object(
            formation.threading.Thread,
            'start',
            new=start_first_late_zero_only,
        ):
            result = controller._compensate_late_motion(
                'tb3_0', publisher, generation
            )

        self.assertFalse(result)
        self.assertEqual(2, len(starts))
        self.assertEqual(3, publisher.calls)
        self.assertEqual(2, len(publisher.messages))
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)
        self.assertNotIn(generation, controller._motion_publish_inflight)

    def test_inflight_motion_keeps_shutdown_unconfirmed_if_rezero_fails(self):
        controller = make_controller()
        controller.current_task_id = 'failed-rezero-race'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_motion = threading.Event()
        late = LateCompensationFailingPublisher(release_motion)
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': late,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        commands = {'tb3_0': Twist(), 'tb3_1': Twist()}
        commands['tb3_0'].linear.x = 0.22
        commands['tb3_1'].linear.x = 0.22
        batch_results = []

        def publish_regular_batch():
            with controller.command_lock:
                with controller.lock:
                    batch_results.append(
                        controller._publish_motion_batch_locked(commands)
                    )

        regular_control = threading.Thread(target=publish_regular_batch)
        regular_control.start()
        self.assertTrue(late.entered.wait(0.5))

        try:
            controller._shutdown()
            status = json.loads(controller.status_pub.messages[-1].data)
            stop = status['stop_publication']
            self.assertEqual('failed', status['state'])
            self.assertEqual(2, stop['requested_count'])
            self.assertEqual(1, stop['accepted_count'])
            self.assertEqual(['tb3_0'], stop['failed_robots'])
            self.assertFalse(stop['publication_confirmed'])
            self.assertEqual(1, len(healthy.messages))
            self.assertEqual(0.0, healthy.messages[0].linear.x)
        finally:
            release_motion.set()
            regular_control.join(1.0)

        self.assertFalse(regular_control.is_alive())
        self.assertEqual([False], batch_results)
        retry_deadline = time.monotonic() + 0.5
        while late.calls < 4 and time.monotonic() < retry_deadline:
            time.sleep(0.005)
        self.assertEqual(4, late.calls)
        self.assertEqual(0.0, late.messages[-1].linear.x)
        self.assertEqual(0.0, late.messages[-1].angular.z)
        self.assertFalse(
            controller._shutdown_publication['publication_confirmed']
        )

    def test_blocked_rezero_does_not_suppress_the_redundant_attempt(self):
        controller = make_controller()
        controller.current_task_id = 'blocked-rezero-race'
        controller.is_running = True
        controller.robot_ids = ['tb3_0', 'tb3_1']
        controller.robot_count = 2
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_motion = threading.Event()
        release_compensation = threading.Event()
        late = LateCompensationBlockingPublisher(
            release_motion, release_compensation
        )
        healthy = FakePublisher()
        controller.cmd_vel_pubs = {
            'tb3_0': late,
            'tb3_1': healthy,
        }
        with controller.lock:
            controller._refresh_shutdown_publisher_snapshot_locked()

        commands = {'tb3_0': Twist(), 'tb3_1': Twist()}
        commands['tb3_0'].linear.x = 0.22
        commands['tb3_1'].linear.x = 0.22
        batch_results = []

        def publish_regular_batch():
            with controller.command_lock:
                with controller.lock:
                    batch_results.append(
                        controller._publish_motion_batch_locked(commands)
                    )

        regular_control = threading.Thread(target=publish_regular_batch)
        regular_control.start()
        self.assertTrue(late.motion_entered.wait(0.5))

        try:
            controller._shutdown()
            stop = json.loads(
                controller.status_pub.messages[-1].data
            )['stop_publication']
            self.assertEqual(1, stop['accepted_count'])
            self.assertEqual(['tb3_0'], stop['failed_robots'])
            self.assertFalse(stop['publication_confirmed'])

            release_motion.set()
            regular_control.join(0.5)
            self.assertFalse(regular_control.is_alive())
            self.assertTrue(late.compensation_entered.wait(0.5))
            redundant_deadline = time.monotonic() + 0.5
            while late.calls < 4 and time.monotonic() < redundant_deadline:
                time.sleep(0.005)
            self.assertEqual(4, late.calls)
            self.assertEqual(0.0, late.messages[-1].linear.x)
            self.assertEqual(0.0, late.messages[-1].angular.z)
            self.assertEqual([False], batch_results)
            self.assertTrue(all(
                command.linear.x == 0.0 and command.angular.z == 0.0
                for command in healthy.messages
            ))
        finally:
            release_motion.set()
            release_compensation.set()
            regular_control.join(1.0)

    def test_late_regular_status_is_followed_by_final_shutdown_status(self):
        controller = make_controller()
        controller.current_task_id = 'status-shutdown-race'
        controller.is_running = True
        controller.formation_state = formation.FormationState.FORMING
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_regular_status = threading.Event()
        status_publisher = FirstPublishBlockingPublisher(
            release_regular_status
        )
        controller.status_pub = status_publisher

        def publish_regular_status():
            with controller.command_lock:
                controller._publish_status([], 0.4)

        regular_status = threading.Thread(target=publish_regular_status)
        regular_status.start()
        self.assertTrue(status_publisher.entered.wait(0.5))

        try:
            controller._shutdown()
            self.assertTrue(regular_status.is_alive())
            states_at_shutdown = [
                json.loads(message.data)['state']
                for message in status_publisher.messages
            ]
            self.assertEqual(['failed'], states_at_shutdown)
        finally:
            release_regular_status.set()
            regular_status.join(1.0)

        self.assertFalse(regular_status.is_alive())
        reassert_deadline = time.monotonic() + 0.5
        while status_publisher.calls < 4 and time.monotonic() < reassert_deadline:
            time.sleep(0.005)
        states = [
            json.loads(message.data)['state']
            for message in status_publisher.messages
        ]
        self.assertEqual(['failed', 'forming'], states[:2])
        self.assertEqual('failed', states[-1])
        self.assertGreaterEqual(states.count('failed'), 2)
        final_status = json.loads(status_publisher.messages[-1].data)
        self.assertIn('could not acquire the lifecycle lock', final_status['error'])
        self.assertEqual(
            'shutdown', final_status['stop_publication']['reason']
        )

    def test_blocked_status_reassertion_does_not_hide_final_status(self):
        controller = make_controller()
        controller.current_task_id = 'blocked-status-reassertion'
        controller.is_running = True
        controller.formation_state = formation.FormationState.FORMING
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        release_regular = threading.Event()
        release_reassertion = threading.Event()
        status_publisher = StatusReassertionBlockingPublisher(
            release_regular, release_reassertion
        )
        controller.status_pub = status_publisher

        def publish_regular_status():
            with controller.command_lock:
                controller._publish_status([], 0.4)

        regular_status = threading.Thread(target=publish_regular_status)
        regular_status.start()
        self.assertTrue(status_publisher.regular_entered.wait(0.5))

        try:
            controller._shutdown()
            self.assertEqual(
                ['failed'],
                [
                    json.loads(message.data)['state']
                    for message in status_publisher.messages
                ],
            )

            release_regular.set()
            regular_status.join(0.5)
            self.assertFalse(regular_status.is_alive())
            self.assertTrue(status_publisher.reassertion_entered.wait(0.5))
            redundant_deadline = time.monotonic() + 0.5
            while (
                status_publisher.calls < 4
                and time.monotonic() < redundant_deadline
            ):
                time.sleep(0.005)
            states = [
                json.loads(message.data)['state']
                for message in status_publisher.messages
            ]
            self.assertEqual(['failed', 'forming'], states[:2])
            self.assertEqual('failed', states[-1])
        finally:
            release_regular.set()
            release_reassertion.set()
            regular_status.join(1.0)

        final_status = json.loads(status_publisher.messages[-1].data)
        self.assertEqual('failed', final_status['state'])
        self.assertIn('could not acquire the lifecycle lock', final_status['error'])

    def test_start_waiting_on_state_lock_cannot_survive_shutdown(self):
        controller = make_controller()
        controller.current_task_id = 'task-before-shutdown'
        controller.is_running = False
        controller.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 0.08
        controller.lock.acquire()
        start = threading.Thread(
            target=controller._start_cb,
            args=(String(data=json.dumps({
                'task_id': 'late-start',
                'formation_type': 'line',
            })),),
        )
        start.start()

        owns_command_lock = False
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if not controller.command_lock.acquire(blocking=False):
                owns_command_lock = True
                break
            controller.command_lock.release()
            time.sleep(0.005)
        self.assertTrue(owns_command_lock)

        try:
            controller._shutdown()
            self.assertTrue(start.is_alive())
            self.assertFalse(controller.is_running)
            self.assertEqual(
                formation.FormationState.FAILED,
                controller.formation_state,
            )
        finally:
            controller.lock.release()
            start.join(1.0)

        self.assertFalse(start.is_alive())
        self.assertFalse(controller.is_running)
        self.assertEqual('task-before-shutdown', controller.current_task_id)
        self.assertEqual(
            formation.FormationState.FAILED,
            controller.formation_state,
        )
        final_status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual('task-before-shutdown', final_status['task_id'])
        self.assertEqual('failed', final_status['state'])

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
