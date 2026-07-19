#!/usr/bin/env python3

import json
import math
import pathlib
import threading
import time
import unittest

import yaml

from test_ros_lifecycle import (
    FakeAvoidance, FakePublisher, ModelStates, Pose, ROS, String,
)


FOLLOW = ROS["follow"]
PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


def pose_at(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose


def place_robot(controller, name, x, y, yaw=0.0):
    controller.poses[name] = pose_at(x, y)
    controller.yaws[name] = yaw


def place_spawn_pattern(controller, pattern, spacing=0.6):
    count = len(controller.robot_names)
    if pattern == "grid":
        columns = int(math.ceil(math.sqrt(count)))
        rows = int(math.ceil(count / float(columns)))
        x_offset = -(columns - 1) * spacing / 2.0
        y_offset = -(rows - 1) * spacing / 2.0
        for index, name in enumerate(controller.robot_names):
            place_robot(
                controller,
                name,
                x_offset + (index % columns) * spacing,
                y_offset + (index // columns) * spacing,
            )
        return

    if pattern == "circle":
        radius = spacing / (2.0 * math.sin(math.pi / count))
        for index, name in enumerate(controller.robot_names):
            angle = 2.0 * math.pi * index / count
            place_robot(
                controller,
                name,
                radius * math.cos(angle),
                radius * math.sin(angle),
                FOLLOW.normalize_angle(angle + math.pi),
            )
        return

    start_x = -(count - 1) * spacing / 2.0
    for index, name in enumerate(controller.robot_names):
        place_robot(controller, name, start_x + index * spacing, 0.0)


def simulate_follow(controller, duration):
    """Small deterministic unicycle simulation for controller regressions."""
    steps = int(round(duration / controller.dt))
    leader_commands = []
    for _ in range(steps):
        controller._control_step(None)
        commands = {
            name: controller.cmd_pubs[name].messages[-1]
            for name in controller.robot_names
        }
        leader_commands.append(commands[controller.robot_names[0]].linear.x)

        updates = {}
        for name, command in commands.items():
            yaw = controller.yaws[name]
            middle_yaw = yaw + command.angular.z * controller.dt * 0.5
            updates[name] = (
                controller.poses[name].position.x
                + command.linear.x * math.cos(middle_yaw) * controller.dt,
                controller.poses[name].position.y
                + command.linear.x * math.sin(middle_yaw) * controller.dt,
                FOLLOW.normalize_angle(
                    yaw + command.angular.z * controller.dt
                ),
            )

        for name, (x, y, yaw) in updates.items():
            place_robot(controller, name, x, y, yaw)

    gaps = []
    for first, second in zip(
        controller.robot_names, controller.robot_names[1:]
    ):
        first_pose = controller.poses[first]
        second_pose = controller.poses[second]
        gaps.append(math.hypot(
            first_pose.position.x - second_pose.position.x,
            first_pose.position.y - second_pose.position.y,
        ))
    return gaps, leader_commands


def make_controller(mode="circular", count=3, follow_distance=0.65):
    controller = FOLLOW.FollowTheLeader.__new__(FOLLOW.FollowTheLeader)
    controller.command_lock = threading.RLock()
    controller.lock = threading.RLock()
    controller.robot_names = ["tb3_{}".format(i) for i in range(count)]
    controller.poses = {
        name: pose_at(0.0, 0.0) for name in controller.robot_names
    }
    controller.yaws = {name: 0.0 for name in controller.robot_names}
    controller.leader_mode = mode
    controller.follow_distance = follow_distance
    controller.max_linear_vel = 0.2
    controller.max_angular_vel = 1.5
    controller.follower_speed_reserve = 0.05
    controller.dt = 0.05
    controller.requested_path_radius = 1.5
    controller.path_radius = 1.5
    controller.arena_size = 100.0
    controller.arena_margin = 0.35
    controller.arena_profile = "swarm_arena"
    controller.spawn_obstacle_clearance = 0.30
    controller.spawn_exclusion_zones = []
    controller.model_poses = {}
    controller.path_relocation_step = 0.50
    controller.path_relocation_limit = 128
    controller.path_planner_async = False
    controller.path_t = 0.0
    controller.path_anchor_x = 0.0
    controller.path_anchor_y = 0.0
    controller.path_anchor_yaw = 0.0
    controller.path_rotation = 0.0
    controller.path_phase = 0.0
    controller.path_direction = 1.0
    controller.path_initial_heading_error = 0.0
    controller.path_anchor_ready = False
    controller.path_radius_was_adapted = False
    controller.path_error = None
    controller.path_planning = False
    controller.path_plan_generation = 0
    controller.path_plan_thread = None
    controller.path_planning_started_at = None
    controller.path_planning_wall_s = 0.0
    controller.path_relocated = False
    controller.path_staging = False
    controller.staging_targets = {}
    controller.staging_routes = {}
    controller.staging_route_indices = {}
    controller.staging_batches = []
    controller.staging_batch_index = 0
    controller.staging_reached = {}
    controller.staging_phase = None
    controller.staging_settle_ticks = 0
    controller.chain_assembled = False
    controller.chain_settle_ticks = 0
    controller.chain_settle_required = 7
    controller.chain_settle_tolerance = 0.14
    controller.leader_speed_scale = 0.0
    controller.leader_ramp_step = controller.dt / 1.2
    controller.current_waypoint_idx = 0
    controller.waypoints = []
    controller.current_task_id = "follow-test"
    controller.is_active = True
    controller.is_paused = False
    controller.emergency_stop_active = False
    controller.manual_twist = FOLLOW.Twist()
    controller.leader_trace = FOLLOW.ArcLengthTrace(minimum_step=0.015)
    controller.status_pub = FakePublisher()
    controller.marker_pub = FakePublisher()
    controller.cmd_pubs = {
        name: FakePublisher() for name in controller.robot_names
    }
    controller.linear_pids = {
        name: FOLLOW.PIDController(1.0, 0.0, 0.3, 0.2)
        for name in controller.robot_names
    }
    controller.angular_pids = {
        name: FOLLOW.PIDController(2.0, 0.0, 0.4, 1.5)
        for name in controller.robot_names
    }
    controller.avoidance = {
        name: FakeAvoidance() for name in controller.robot_names
    }
    return controller


class FollowLeaderPathTests(unittest.TestCase):
    def test_status_reports_closed_path_lap_progress(self):
        for mode in FOLLOW.PARAMETRIC_MODES:
            with self.subTest(mode=mode):
                controller = make_controller(mode=mode, count=6)
                controller.path_anchor_ready = True
                period = controller._path_period(mode, controller.path_radius)
                controller.path_t = period * 2.25

                controller._publish_status([])

                status = json.loads(controller.status_pub.messages[-1].data)
                self.assertAlmostEqual(
                    period, status["path_period_s"], places=3
                )
                self.assertAlmostEqual(
                    2.25, status["path_progress_laps"], places=3
                )
                self.assertAlmostEqual(
                    0.25, status["current_lap_progress"], places=3
                )
                self.assertEqual(2, status["completed_laps"])

    def test_setup_time_does_not_count_toward_a_path_lap(self):
        controller = make_controller(mode="figure8", count=10)
        controller.path_anchor_ready = True
        controller.chain_assembled = False

        for _ in range(20):
            controller._control_step(None)

        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(0.0, controller.path_t)
        self.assertEqual(0.0, status["path_progress_laps"])
        self.assertEqual(0, status["completed_laps"])

    def test_non_parametric_mode_has_no_closed_path_progress(self):
        controller = make_controller(mode="waypoint", count=3)

        controller._publish_status([])

        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertIsNone(status["path_period_s"])
        self.assertIsNone(status["path_progress_laps"])
        self.assertIsNone(status["current_lap_progress"])
        self.assertEqual(0, status["completed_laps"])

    def test_every_parametric_path_starts_at_the_leader_heading(self):
        anchor_x = 1.25
        anchor_y = -0.80
        anchor_yaw = 1.10

        for mode in FOLLOW.PARAMETRIC_MODES:
            with self.subTest(mode=mode):
                controller = make_controller(mode=mode)
                controller.path_anchor_x = anchor_x
                controller.path_anchor_y = anchor_y
                controller.path_anchor_yaw = anchor_yaw
                controller.path_rotation = anchor_yaw
                controller.path_anchor_ready = True

                start = controller._generate_leader_path(mode, 0.0)
                velocity = controller._leader_path_velocity(
                    mode, 0.0, dt_sample=0.001
                )
                forward = (math.cos(anchor_yaw), math.sin(anchor_yaw))
                left = (-forward[1], forward[0])
                forward_speed = (
                    velocity[0] * forward[0] + velocity[1] * forward[1]
                )
                lateral_speed = (
                    velocity[0] * left[0] + velocity[1] * left[1]
                )

                self.assertAlmostEqual(anchor_x, start[0], places=9)
                self.assertAlmostEqual(anchor_y, start[1], places=9)
                self.assertGreater(forward_speed, 0.05)
                self.assertLess(abs(lateral_speed), 0.001)

    def test_start_anchors_the_path_to_current_leader_pose(self):
        controller = make_controller(mode="circular", count=3)
        leader = controller.robot_names[0]
        controller.poses[leader] = pose_at(1.2, -0.7)
        controller.yaws[leader] = 0.6

        controller._start_cb(String(data=json.dumps({
            "task_id": "anchored-task",
            "leader_mode": "circular",
            "radius": 1.6,
            "follow_distance": 0.65,
        })))

        self.assertTrue(controller.path_anchor_ready)
        self.assertIsNone(controller.path_error)
        self.assertAlmostEqual(1.2, controller.path_anchor_x)
        self.assertAlmostEqual(-0.7, controller.path_anchor_y)
        self.assertAlmostEqual(0.6, controller.path_anchor_yaw)
        self.assertEqual(
            (1.2, -0.7),
            controller._generate_leader_path("circular", 0.0),
        )

    def test_radius_grows_before_a_closed_path_can_wrap(self):
        cases = (
            ("circular", 20, 20 * 0.65 / (2.0 * math.pi)),
            ("square", 30, 30 * 0.65 / 8.0),
        )

        for mode, count, expected_radius in cases:
            with self.subTest(mode=mode, count=count):
                controller = make_controller(mode=mode, count=count)
                controller.requested_path_radius = 0.5
                controller.path_radius = 0.5

                configured = controller._configure_parametric_path(
                    pose_at(0.0, 0.0), 0.0, count
                )

                self.assertTrue(configured)
                self.assertTrue(controller.path_radius_was_adapted)
                self.assertAlmostEqual(
                    expected_radius, controller.path_radius, places=7
                )

                if mode == "circular":
                    safe_length = 2.0 * math.pi * controller.path_radius
                elif mode == "square":
                    safe_length = 8.0 * controller.path_radius
                else:
                    safe_length = (
                        FOLLOW.FIGURE8_LOBE_LENGTH * controller.path_radius
                    )
                self.assertGreaterEqual(
                    safe_length + 1e-9, count * controller.follow_distance
                )

    def test_figure8_lobe_grows_to_hold_the_chain(self):
        controller = make_controller(mode="figure8", count=10)
        controller.requested_path_radius = 1.5
        controller.path_radius = 1.5

        configured = controller._configure_parametric_path(
            pose_at(0.0, 0.0), 0.0, 10
        )

        self.assertTrue(configured)
        self.assertTrue(controller.path_radius_was_adapted)
        self.assertAlmostEqual(
            10 * 0.65 / FOLLOW.FIGURE8_LOBE_LENGTH,
            controller.path_radius,
        )

    def test_arena_infeasibility_is_reported_in_status(self):
        controller = make_controller(mode="circular", count=3)
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.requested_path_radius = 5.0

        configured = controller._configure_parametric_path(
            pose_at(0.0, 0.0), 0.0, 3
        )
        controller._publish_status([])
        status = json.loads(controller.status_pub.messages[-1].data)

        self.assertFalse(configured)
        self.assertFalse(controller.path_anchor_ready)
        self.assertIn("3 robots", controller.path_error)
        self.assertIn("does not fit", controller.path_error)
        self.assertIn("requested 5.00 m", controller.path_error)
        self.assertEqual("failed", status["state"])
        self.assertFalse(status["active"])
        self.assertEqual(controller.path_error, status["error"])

    def test_edge_start_uses_an_alternate_feasible_placement(self):
        controller = make_controller(mode="figure8", count=10)
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.requested_path_radius = 1.5

        configured = controller._configure_parametric_path(
            pose_at(-4.5, 0.0), 0.0, 10
        )

        self.assertTrue(configured)
        self.assertTrue(controller.path_radius_was_adapted)
        self.assertTrue(controller.path_anchor_ready)
        self.assertIsNone(controller.path_error)
        self.assertEqual(
            (-4.5, 0.0),
            controller._generate_leader_path("figure8", 0.0),
        )

    def test_closed_path_turns_away_from_a_cylinder(self):
        controller = make_controller(mode="circular", count=3)
        controller.arena_size = 10.0
        controller.requested_path_radius = 1.0
        controller.spawn_exclusion_zones = [{
            "name": "test_cylinder",
            "worlds": ["swarm_arena"],
            "shape": "circle",
            "x": 0.0,
            "y": 2.0,
            "radius": 0.25,
        }]

        self.assertFalse(controller._path_fits_arena(
            "circular", 1.0, 0.0, 0.0, 0.0
        ))
        configured = controller._configure_parametric_path(
            pose_at(0.0, 0.0), 0.0, 3
        )

        self.assertTrue(configured)
        self.assertTrue(controller._path_fits_arena(
            "circular",
            controller.path_radius,
            controller.path_anchor_x,
            controller.path_anchor_y,
            controller.path_rotation,
            controller.path_phase,
            controller.path_direction,
        ))

    def test_rotated_box_blocks_a_square_lap(self):
        controller = make_controller(mode="square", count=3)
        controller.arena_size = 10.0
        controller.spawn_exclusion_zones = [{
            "name": "diagonal_wall",
            "worlds": ["swarm_arena"],
            "shape": "box",
            "x": 1.0,
            "y": 0.0,
            "width": 0.7,
            "height": 0.12,
            "yaw": 0.65,
        }]

        self.assertFalse(controller._path_fits_arena(
            "square", 1.0, 0.0, 0.0, 0.0
        ))

    def test_obstacle_bound_path_fails_closed_when_no_placement_exists(self):
        controller = make_controller(mode="figure8", count=10)
        controller.arena_size = 10.0
        controller.requested_path_radius = 1.5
        controller.spawn_exclusion_zones = [{
            "name": "blocked_arena",
            "worlds": ["swarm_arena"],
            "shape": "circle",
            "x": 0.0,
            "y": 0.0,
            "radius": 20.0,
        }]

        configured = controller._configure_parametric_path(
            pose_at(0.0, 0.0), 0.0, 10
        )

        self.assertFalse(configured)
        self.assertFalse(controller.path_anchor_ready)
        self.assertIn("configured obstacles", controller.path_error)

    def test_live_payload_pose_can_invalidate_an_existing_path(self):
        controller = make_controller(mode="square", count=3)
        controller.arena_size = 20.0
        controller.spawn_exclusion_zones = [{
            "name": "transport_object",
            "model": "transport_object",
            "worlds": ["swarm_arena"],
            "shape": "box",
            "x": 8.0,
            "y": 8.0,
            "width": 0.8,
            "height": 0.1,
            "yaw_offset": math.pi / 4.0,
        }]

        self.assertTrue(controller._path_fits_arena(
            "square", 1.0, 0.0, 0.0, 0.0
        ))

        payload_pose = pose_at(1.0, 0.55)
        payload_pose.orientation.z = math.sin(math.pi / 8.0)
        payload_pose.orientation.w = math.cos(math.pi / 8.0)
        model_states = ModelStates()
        model_states.name = ["transport_object"]
        model_states.pose = [payload_pose]
        controller._model_states_cb(model_states)

        self.assertAlmostEqual(
            math.pi / 4.0,
            controller.model_poses["transport_object"][2],
        )
        self.assertFalse(controller._path_fits_arena(
            "square", 1.0, 0.0, 0.0, 0.0
        ))

    def test_follower_target_stays_inside_the_usable_arena(self):
        controller = make_controller(mode="figure8", count=3)
        controller.arena_size = 10.0
        controller.arena_margin = 0.35

        clamped_x, clamped_y = controller._clamp_follow_target(-9.0, 9.0)

        self.assertAlmostEqual(-4.47, clamped_x)
        self.assertAlmostEqual(4.47, clamped_y)

    def test_far_follower_keeps_crawling_while_turning(self):
        controller = make_controller(mode="figure8", count=2)
        controller.poses["tb3_0"] = pose_at(0.0, 0.0)
        controller.yaws["tb3_0"] = 0.0
        controller.poses["tb3_1"] = pose_at(0.0, -1.4)
        controller.yaws["tb3_1"] = math.pi
        controller.leader_trace.seed_line(0.0, 0.0, 0.0, 0.65)

        cmd = controller._update_follower(1, 0.05)

        self.assertGreater(cmd.linear.x, 0.0)
        self.assertLessEqual(cmd.linear.x, controller.max_linear_vel)

    def test_close_follower_targets_outward_spacing(self):
        controller = make_controller(mode="figure8", count=2)
        controller.poses["tb3_0"] = pose_at(0.0, 0.0)
        controller.yaws["tb3_0"] = 0.0
        controller.poses["tb3_1"] = pose_at(0.35, 0.0)
        controller.yaws["tb3_1"] = 0.0
        controller.leader_trace.seed_line(0.0, 0.0, 0.0, 0.65)

        cmd = controller._update_follower(1, 0.05)

        self.assertGreater(cmd.linear.x, 0.0)

    def test_infeasible_path_keeps_every_robot_stopped(self):
        controller = make_controller(mode="circular", count=3)
        controller.path_error = "path does not fit"

        controller._control_loop(None)

        for publisher in controller.cmd_pubs.values():
            self.assertTrue(publisher.messages)
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual("failed", status["state"])

    def test_stale_odometry_fails_the_active_behavior_closed(self):
        controller = make_controller(mode="circular", count=3)
        now = FOLLOW.time.monotonic()
        controller.task_started_at = now - 1.0
        controller.odom_timeout_wall_s = 0.5
        controller.odom_received_at = {
            name: now - 1.0 for name in controller.robot_names
        }

        controller._control_loop(None)

        self.assertIn("Odometry became stale", controller.path_error)
        for publisher in controller.cmd_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual("failed", status["state"])
        self.assertEqual(controller.robot_names, status["stale_odometry"])

    def test_orderly_shutdown_zeroes_every_follower_command(self):
        controller = make_controller(mode="circular", count=3)

        controller._shutdown()

        self.assertFalse(controller.is_active)
        self.assertFalse(controller.is_paused)
        for publisher in controller.cmd_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

    def test_fleet_growth_reanchors_before_resuming(self):
        controller = make_controller(mode="circular", count=3)
        controller.path_anchor_ready = True
        controller.path_t = 12.0

        def add_robot(name):
            controller.poses[name] = pose_at(0.0, 0.0)
            controller.yaws[name] = 0.0
            controller.linear_pids[name] = FOLLOW.PIDController(
                1.0, 0.0, 0.3, 0.2
            )
            controller.angular_pids[name] = FOLLOW.PIDController(
                2.0, 0.0, 0.4, 1.5
            )

        controller._add_robot = add_robot
        controller._sync_fleet([
            "tb3_{}".format(index) for index in range(8)
        ])

        self.assertFalse(controller.path_anchor_ready)
        self.assertEqual(0.0, controller.path_t)
        self.assertIsNone(controller.path_error)
        self.assertEqual(8, len(controller.robot_names))

    def test_sorted_roster_heartbeat_preserves_the_spatial_chain(self):
        controller = make_controller(mode="figure8", count=3)
        controller.robot_names = ["tb3_2", "tb3_1", "tb3_0"]
        controller.path_anchor_ready = True
        controller.path_t = 12.0
        controller.leader_trace.seed_line(0.0, 0.0, 0.0, 1.3)
        original_span = controller.leader_trace.span
        original_generation = controller.path_plan_generation

        controller._sync_fleet(["tb3_0", "tb3_1", "tb3_2"])

        self.assertEqual(
            ["tb3_2", "tb3_1", "tb3_0"], controller.robot_names
        )
        self.assertTrue(controller.path_anchor_ready)
        self.assertEqual(12.0, controller.path_t)
        self.assertEqual(original_span, controller.leader_trace.span)
        self.assertEqual(
            original_generation, controller.path_plan_generation
        )

    def test_slow_path_planning_keeps_start_and_status_nonblocking(self):
        controller = make_controller(mode="circular", count=3)
        controller.path_planner_async = True
        planner_started = threading.Event()
        release_planner = threading.Event()
        original_builder = controller._build_path_plan

        def slow_builder(snapshot):
            planner_started.set()
            release_planner.wait(2.0)
            return original_builder(snapshot)

        controller._build_path_plan = slow_builder
        started_at = time.monotonic()
        controller._start_cb(String(data=json.dumps({
            "task_id": "slow-planner",
            "leader_mode": "circular",
            "radius": 1.6,
            "follow_distance": 0.65,
        })))
        callback_time = time.monotonic() - started_at

        self.assertTrue(planner_started.wait(0.5))
        self.assertLess(callback_time, 0.25)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual("slow-planner", status["task_id"])
        self.assertEqual("planning_path", status["state"])
        self.assertGreaterEqual(status["planning_wall_s"], 0.0)

        controller._control_loop(None)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual("planning_path", status["state"])
        for publisher in controller.cmd_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

        release_planner.set()
        controller.path_plan_thread.join(2.0)
        self.assertFalse(controller.path_plan_thread.is_alive())
        self.assertTrue(controller.path_anchor_ready)
        controller._publish_status([])
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertGreater(status["planning_wall_s"], 0.0)

    def test_coarse_screen_never_replaces_final_path_verification(self):
        controller = make_controller(mode="circular", count=3)
        controller.arena_size = 10.0
        angle = math.radians(2.5)
        path_x = math.sin(angle)
        path_y = 1.0 - math.cos(angle)
        controller.spawn_exclusion_zones = [{
            "name": "near_threshold_point",
            "worlds": ["swarm_arena"],
            "shape": "circle",
            "x": path_x + 0.299 * math.sin(angle),
            "y": path_y - 0.299 * math.cos(angle),
            "radius": 0.0,
        }]
        period = controller._path_period("circular", 1.0)
        active_zones = list(controller._active_exclusion_zones())

        coarse_fit = controller._path_fits_arena_samples(
            "circular", 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            period, FOLLOW.PATH_PLACEMENT_COARSE_SAMPLES,
            active_zones, {},
        )

        self.assertTrue(coarse_fit)
        self.assertFalse(controller._path_fits_arena(
            "circular", 1.0, 0.0, 0.0, 0.0
        ))

    def test_n3_grid_circle_relocates_to_a_reachable_safe_start(self):
        with (PACKAGE_ROOT / "config" / "arena_spawn_zones.yaml").open() as stream:
            spawn_config = yaml.safe_load(stream)
        controller = make_controller(mode="circular", count=3)
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.requested_path_radius = 1.6
        controller.spawn_obstacle_clearance = spawn_config[
            "spawn_obstacle_clearance"
        ]
        controller.spawn_exclusion_zones = spawn_config[
            "spawn_exclusion_zones"
        ]
        place_robot(controller, "tb3_0", -0.5, -0.5)
        place_robot(controller, "tb3_1", 0.5, -0.5)
        place_robot(controller, "tb3_2", -0.5, 0.5)

        controller._start_cb(String(data=json.dumps({
            "task_id": "relocated-circle",
            "leader_mode": "circular",
            "radius": 1.6,
            "follow_distance": 0.65,
        })))

        self.assertTrue(controller.path_relocated)
        self.assertTrue(controller.path_staging)
        self.assertFalse(controller.path_anchor_ready)
        self.assertEqual((0.0, -0.5), (
            controller.path_anchor_x, controller.path_anchor_y
        ))
        self.assertEqual(0.0, controller.path_phase)
        self.assertEqual(-1.0, controller.path_direction)
        self.assertAlmostEqual(-4.450542717585032, controller.path_rotation)
        self.assertTrue(controller.staging_routes["tb3_1"])
        self.assertTrue(controller._path_fits_arena(
            "circular",
            controller.path_radius,
            controller.path_anchor_x,
            controller.path_anchor_y,
            controller.path_rotation,
            controller.path_phase,
            controller.path_direction,
            controller.model_poses,
        ))

        simulate_follow(controller, 20.0)

        self.assertFalse(controller.path_staging)
        self.assertTrue(controller.path_anchor_ready)
        self.assertGreater(controller.path_t, 0.0)

    def test_n10_line_figure8_keeps_its_known_safe_exact_placement(self):
        with (PACKAGE_ROOT / "config" / "arena_spawn_zones.yaml").open() as stream:
            spawn_config = yaml.safe_load(stream)
        controller = make_controller(mode="figure8", count=10)
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.requested_path_radius = 1.5
        controller.spawn_obstacle_clearance = spawn_config[
            "spawn_obstacle_clearance"
        ]
        controller.spawn_exclusion_zones = spawn_config[
            "spawn_exclusion_zones"
        ]
        for index, name in enumerate(controller.robot_names):
            place_robot(controller, name, -4.5 + index, -0.1)

        controller._start_cb(String(data=json.dumps({
            "task_id": "exact-figure8",
            "leader_mode": "figure8",
            "radius": 1.5,
            "follow_distance": 0.65,
        })))

        self.assertFalse(controller.path_relocated)
        self.assertFalse(controller.path_staging)
        self.assertTrue(controller.path_anchor_ready)
        self.assertEqual((4.5, -0.1), (
            controller.path_anchor_x, controller.path_anchor_y
        ))
        self.assertAlmostEqual(148.84991758995693, controller.path_phase)
        self.assertEqual(1.0, controller.path_direction)
        self.assertAlmostEqual(-3.9269725386968566, controller.path_rotation)
        self.assertTrue(controller._path_fits_arena(
            "figure8",
            controller.path_radius,
            controller.path_anchor_x,
            controller.path_anchor_y,
            controller.path_rotation,
            controller.path_phase,
            controller.path_direction,
            controller.model_poses,
        ))

    def test_line_spawn_uses_front_robot_as_follow_leader(self):
        controller = make_controller(mode="figure8", count=10)
        for index, name in enumerate(controller.robot_names):
            controller.poses[name] = pose_at(-4.5 + index, 0.0)
            controller.yaws[name] = 0.0

        ordered = controller._spatial_chain_order(controller.robot_names)

        self.assertEqual("tb3_9", ordered[0])
        self.assertEqual(
            ["tb3_{}".format(index) for index in reversed(range(10))],
            ordered,
        )

    def test_parametric_start_waits_for_complete_odometry(self):
        controller = make_controller(mode="figure8", count=3)
        controller.path_anchor_ready = False
        controller.poses["tb3_1"] = None

        controller._control_step(None)

        self.assertFalse(controller.path_anchor_ready)
        self.assertTrue(controller.status_pub.messages)

    def test_path_generation_does_not_raise_the_speed_limit(self):
        for mode in FOLLOW.PARAMETRIC_MODES:
            with self.subTest(mode=mode):
                controller = make_controller(mode=mode)
                controller.path_anchor_ready = True
                period = controller._path_period(mode, controller.path_radius)
                speeds = []
                for index in range(400):
                    t = period * index / 400.0
                    vx, vy = controller._leader_path_velocity(
                        mode, t, dt_sample=0.001
                    )
                    speeds.append(math.hypot(vx, vy))

                self.assertLessEqual(
                    max(speeds), controller.max_linear_vel + 1e-4
                )

    def test_parametric_leader_keeps_follower_catchup_headroom(self):
        for count in (1, 3, 6, 10, 20):
            with self.subTest(count=count):
                controller = make_controller(count=count)
                path_speed = controller._parametric_path_speed()

                self.assertLessEqual(
                    path_speed, controller._leader_speed_limit()
                )
                self.assertLessEqual(
                    controller._leader_speed_limit(), 0.15 + 1e-9
                )
                self.assertGreaterEqual(
                    controller.max_linear_vel
                    - controller._leader_speed_limit(),
                    controller.follower_speed_reserve - 1e-9,
                )

    def test_trace_is_seeded_from_the_actual_spatial_chain(self):
        controller = make_controller(mode="square", count=6)
        place_spawn_pattern(controller, "circle")
        controller.robot_names = controller._spatial_chain_order(
            controller.robot_names
        )

        controller._seed_trace_from_chain(
            controller.robot_names,
            controller.yaws[controller.robot_names[0]],
        )

        errors = controller._chain_target_errors(controller.robot_names)
        self.assertEqual(5, len(errors))
        self.assertLess(max(errors), 0.3)
        self.assertGreaterEqual(
            controller.leader_trace.span,
            5 * controller.follow_distance - 1e-9,
        )

    def test_live_matrix_sizes_converge_to_requested_spacing(self):
        cases = (
            ("circular", 3, "grid", 42.0),
            ("square", 6, "circle", 48.0),
            ("figure8", 10, "line", 54.0),
        )
        for mode, count, pattern, duration in cases:
            with self.subTest(mode=mode, count=count):
                controller = make_controller(mode=mode, count=count)
                place_spawn_pattern(controller, pattern)
                controller._start_cb(String(data=json.dumps({
                    "task_id": "{}-{}".format(mode, count),
                    "leader_mode": mode,
                    "radius": 1.6 if mode != "figure8" else 1.5,
                    "follow_distance": 0.65,
                })))

                gaps, _ = simulate_follow(controller, duration)
                errors = [abs(gap - 0.65) for gap in gaps]

                self.assertTrue(controller.chain_assembled)
                self.assertLess(max(errors), 0.18)

    def test_straight_spawn_converges_for_arbitrary_fleet_sizes(self):
        for count in range(1, 13):
            with self.subTest(count=count):
                controller = make_controller(mode="circular", count=count)
                for index, name in enumerate(controller.robot_names):
                    place_robot(controller, name, -0.58 * index, 0.0)
                controller.path_anchor_ready = True
                controller.path_anchor_x = 0.0
                controller.path_anchor_y = 0.0
                controller.path_rotation = 0.0
                controller._seed_trace_from_chain(
                    controller.robot_names, 0.0
                )

                gaps, _ = simulate_follow(controller, 35.0)
                errors = [abs(gap - 0.65) for gap in gaps]

                self.assertTrue(controller.chain_assembled)
                self.assertLess(max(errors, default=0.0), 0.18)


if __name__ == "__main__":
    unittest.main()
