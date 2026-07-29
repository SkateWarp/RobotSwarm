#!/usr/bin/env python3
"""Visible Gazebo acceptance runner for the RobotSwarm ROS 1 container.

The runner deliberately uses the public /swarm/commands interface.  It watches
Gazebo ground truth only to measure safety and motion; it never drives a robot
directly.  Each case prints one machine-readable RESULT_JSON line, followed by
one SUMMARY_JSON line for the whole run.
"""

import argparse
from collections import deque
import json
import math
import re
import signal
import statistics
import sys
import threading
import time
import uuid

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


ROBOT_RE = re.compile(r"^tb3_\d+$")
TERMINAL_STATES = {"completed", "failed", "stopped"}
TRANSPORT_PROGRESS_EPSILON_M = 0.001
MINIMUM_FORMATION_ACTIVE_SECONDS = 15.0
MINIMUM_FOLLOW_ACTIVE_SECONDS = 15.0
TRANSPORT_PHASES = frozenset({
    "SEARCH", "APPROACH", "PUSH", "DONE", "FAILED",
})
TRANSPORT_PHASE_ORDER = {
    "SEARCH": 0,
    "APPROACH": 1,
    "PUSH": 2,
    "DONE": 3,
    "FAILED": 3,
}
# The payload and complete push chains are docked only after APPROACH has
# finished.  DONE is included because the transport and aggregate-status
# publishers are independent; the terminal phase can be observed one callback
# before the last PUSH collision-state edge.
TRANSPORT_DOCKING_PHASES = frozenset({"PUSH", "DONE"})
TRANSPORT_PHASE_TIMEOUT_TOLERANCE_WALL_S = 0.25
MAXIMUM_TRANSPORT_PHASE_TRANSITIONS = 32
COLLISION_EVENT_STREAM_VERSION = 1
MAXIMUM_COLLISION_EVENT_HISTORY = 128
COLLISION_TASK_TYPES = frozenset({
    "follow_leader", "formation", "transport",
})
RESULT_BEHAVIOR_STATUS_KEYS = (
    "task_id", "state", "phase", "setup_phase", "active", "paused",
    "formation_type", "leader_mode", "movement_mode", "robot_count",
    "maximum_position_error", "settled_for", "stale_odometry",
    "waiting_for_odometry", "follow_distance",
    "trace_length", "chain_ready", "requested_radius", "effective_radius",
    "path_t", "path_relocated", "planning_wall_s", "leader_speed_scale",
    "path_period_s", "path_progress_laps", "current_lap_progress",
    "completed_laps", "robots", "distance_to_target", "planner",
    "grf_mcmc_iterations", "engagement_complete",
    "synchronized_push_started", "error", "control_sequence",
    "control_sim_time", "control_commands", "batch_publish_span_s",
    "push_reference_speed", "push_arbitration", "object_pos", "target_pos",
    "progress", "object_z", "route_robot",
    "route_kind", "route_complete", "queue_docking_started",
    "queue_settling", "approach_stage", "rendezvous_ready_count",
    "compression_progress", "searching_robot_count", "route_target",
    "assembly_routes", "discovery",
    "robot_assignments", "routing", "planning", "useful_contributor_count",
    "useful_contributor_ids", "current_useful_pusher_count",
    "current_useful_pusher_ids", "all_pusher_proof_minimum_speed",
    "all_pushers_confirmed", "arrival_latched",
)


def scenario(name, behavior, count, pattern, **values):
    item = {
        "name": name,
        "behavior": behavior,
        "count": count,
        "pattern": pattern,
        "timeout": values.pop("timeout", 45.0),
    }
    item.update(values)
    return item


SCENARIOS = [
    scenario("formation_triangle_n3", "formation", 3, "grid",
             shape="triangle", spacing=0.65, timeout=35),
    # The routed production spawn can require several staged corridors before
    # the fifth robot is released.  The 40-second cap represented only 108
    # simulated seconds at the acceptance RTF floor and rejected a healthy
    # zero-collision run while it was still converging.  Ninety seconds covers
    # 243 simulated seconds without weakening the position or safety gates.
    scenario("formation_square_n5", "formation", 5, "circle",
             shape="square", spacing=0.65, timeout=90),
    # The production-pattern N=7 baseline reached the moving state after
    # 156.65 simulated seconds.  At the conservative timeout-sizing floor of
    # RTF 2.7, 65 wall seconds retain 18.85 simulated seconds of margin.
    scenario("formation_A_n7", "formation", 7, "line",
             shape="A", spacing=0.55, timeout=65),
    # A later routed production placement was still converging after 165.67
    # simulated seconds.  Keep the same bounded 243-second envelope used for
    # the other medium formations.
    scenario("formation_V_n8", "formation", 8, "grid",
             shape="V", spacing=0.55, timeout=90),
    # The previous 50-second cap stopped a collision-free routed assembly at
    # 150.53 simulated seconds.  This budget changes liveness observation
    # only; the 0.12 m final-error gate remains unchanged.
    scenario("formation_diamond_n9", "formation", 9, "circle",
             shape="diamond", spacing=0.55, timeout=90),
    # A second safe production placement still had nine robots advancing after
    # 252.816 simulated seconds: one long corridor had not cleared for the
    # final robot.  This was not a stall (the controller reported no stalled
    # member or replan), and every safety gate remained green.  A 120-second
    # wall cap at RTF 2.7 covers 324 simulated seconds, preserving 71.184
    # seconds beyond that observed healthy-progress edge without changing the
    # 0.12 m accuracy limit or any collision threshold.
    scenario("formation_S_n10", "formation", 10, "grid",
             shape="S", spacing=0.55, timeout=120),
    # Follow cases stop as soon as the leader path clock completes a lap.
    # These wall-time caps include chain assembly and enough margin to finish
    # at the minimum accepted real-time factor.
    scenario("follow_circular_n3", "follow", 3, "grid",
             mode="circular", radius=1.6, duration=120, required_laps=1),
    scenario("follow_square_n6", "follow", 6, "circle",
             mode="square", radius=1.6, duration=150, required_laps=1),
    scenario("follow_figure8_n10", "follow", 10, "line",
             mode="figure8", radius=1.5, duration=240, required_laps=1),
    # Keep the payload outside initial sensor range in every transport case.
    # This makes search, finder notification and fleet rendezvous observable
    # instead of accepting a controller that starts directly in PUSH.
    # The visible N=1 coverage pass took 209.2 wall seconds and APPROACH 6.1.
    # Its first healthy PUSH advanced 0.4129 m in 35.3 seconds, which projects
    # the 0.50 m contract to about 42.7 seconds.  A 55-second PUSH budget keeps
    # useful margin without weakening the progress gate.  The three phase
    # budgets total 320 seconds; the five extra hard-cap seconds cover their
    # 0.25-second edge tolerances and status-observation jitter.
    scenario("transport_grf_n1", "transport", 1, "grid",
             object_start=(-3.5, -3.0), target=(-3.5, -4.0),
             min_object_travel=0.55, timeout=325, require_search=True,
             phase_timeouts={"SEARCH": 245.0, "APPROACH": 20.0,
                             "PUSH": 55.0}),
    # Exercise the smallest collaborative fleet explicitly.  Until this case
    # has its own measured phase baseline, keep the established N=1 hard cap
    # and let the acceptance evidence, rather than a guessed split, describe
    # how long search, rendezvous and pushing take.
    scenario("transport_grf_n2", "transport", 2, "line",
             object_start=(-3.5, -3.0), target=(-3.5, -4.0),
             min_object_travel=0.55, timeout=325, require_search=True),
    # Visible N=3 runs at RTF 2.996 spent about 92 seconds searching.  Assembly
    # varied from 79.8 seconds to just over 100 seconds; the slower run reached
    # its last staging point and was still turning into alignment at the old
    # edge.  Keep independent phase limits so extra time cannot conceal a
    # stuck search or rendezvous.  The five remaining hard-cap seconds cover
    # edge tolerance and observation jitter without weakening any physical
    # acceptance gate.
    scenario("transport_grf_n3", "transport", 3, "line",
             object_start=(-3.5, -3.0), target=(-3.5, -4.0),
             min_object_travel=0.55, timeout=290, require_search=True,
             phase_timeouts={"SEARCH": 115.0, "APPROACH": 125.0,
                             "PUSH": 45.0}),
    # The 0.75 kg acceptance payload moves much more slowly than the practice
    # crate.  Its first complete four-robot run completed SEARCH and entered
    # APPROACH in 39.3 seconds, entered PUSH in 110.4, then advanced 0.2335 m
    # during 70.0 seconds of synchronized pushing.  That rate projects the
    # unchanged 0.50 m gate to about 150 seconds of PUSH.  Phase budgets retain
    # bounded liveness for both payload profiles; the five hard-cap seconds
    # cover observation jitter.
    scenario("transport_grf_n4", "transport", 4, "circle",
             object_start=(-3.5, -3.0), target=(-3.5, -4.0),
             min_object_travel=0.55, timeout=355, require_search=True,
             phase_timeouts={"SEARCH": 60.0, "APPROACH": 100.0,
                             "PUSH": 190.0}),
    scenario("transport_grf_n10", "transport", 10, "grid",
             object_start=(-3.5, -3.0), target=(-3.5, -4.0),
             min_object_travel=0.55, timeout=220, require_search=True),
]

SMOKE_NAMES = {
    "formation_triangle_n3", "formation_A_n7",
    "follow_square_n6", "transport_grf_n4",
}


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def rounded(value, digits=4):
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def rounded_json(value, digits=4):
    """Round finite floats throughout a small diagnostic JSON structure."""
    if isinstance(value, dict):
        return {
            key: rounded_json(item, digits)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [rounded_json(item, digits) for item in value]
    if isinstance(value, float):
        return rounded(value, digits)
    return value


def finite_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def required_transport_goal_progress(
    configured_minimum, initial_goal_distance, goal_tolerance,
):
    """Keep the progress gate reachable under the delivery contract.

    Transport completes once the remaining goal distance falls below the
    controller's arrival tolerance.  A short scenario therefore cannot be
    required to travel farther than its initial distance minus that tolerance.
    Longer scenarios retain the configured acceptance minimum unchanged.
    """
    configured = finite_number(configured_minimum)
    initial = finite_number(initial_goal_distance)
    tolerance = finite_number(goal_tolerance)
    if configured is None or configured <= 0.0:
        return TRANSPORT_PROGRESS_EPSILON_M
    if initial is None or tolerance is None:
        return configured

    contract_progress = max(
        TRANSPORT_PROGRESS_EPSILON_M,
        initial - max(0.0, tolerance),
    )
    return min(configured, contract_progress)


def follow_lap_requirement_met(status, task_id, mode, required_laps):
    """Return True only for a correlated, running, completed path lap."""
    if not isinstance(status, dict):
        return False
    if not str(task_id or ""):
        return False
    if str(status.get("task_id") or "") != str(task_id or ""):
        return False
    if status.get("state") != "running" or not bool(status.get("active")):
        return False
    if status.get("leader_mode") != mode or not bool(status.get("chain_ready")):
        return False

    period = finite_number(status.get("path_period_s"))
    progress = finite_number(status.get("path_progress_laps"))
    completed = finite_number(status.get("completed_laps"))
    if period is None or period <= 0.0:
        return False
    if progress is None or completed is None:
        return False

    required = max(1, int(required_laps))
    return progress >= required and math.floor(completed) >= required


def summary(values):
    if not values:
        return {"samples": 0, "mean": None, "p95": None, "max": None}
    return {
        "samples": len(values),
        "mean": rounded(statistics.fmean(values)),
        "p95": rounded(percentile(values, 0.95)),
        "max": rounded(max(values)),
    }


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def point_clearance(x, y, zone, live_models):
    """Signed point-to-footprint distance for a circle or rotated box."""
    cx = float(zone.get("x", 0.0))
    cy = float(zone.get("y", 0.0))
    yaw = float(zone.get("yaw", 0.0))
    model = zone.get("model")
    if model in live_models:
        cx, cy, live_yaw = live_models[model]
        yaw = live_yaw + float(zone.get("yaw_offset", 0.0))

    if zone.get("shape") == "circle":
        return math.hypot(x - cx, y - cy) - float(zone.get("radius", 0.0))

    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = x - cx, y - cy
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    qx = abs(local_x) - float(zone.get("width", 0.0)) / 2.0
    qy = abs(local_y) - float(zone.get("height", 0.0)) / 2.0
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    return outside + min(max(qx, qy), 0.0)


def clean_robot_name(value):
    if value is None:
        return None
    name = str(value).strip().strip("/")
    return name if ROBOT_RE.match(name) else None


def acceptance_robot_ids(count):
    """Use a bounded set of model names across a sequential matrix.

    Gazebo Classic does not reliably return every plugin allocation to the
    operating system after a model is deleted.  Reusing the same namespaces
    keeps a long acceptance matrix from also creating an unbounded series of
    sensor, transport, and ROS topic names.
    """
    return ["tb3_{}".format(index) for index in range(max(0, int(count)))]


def robot_sort_key(name):
    """Sort TurtleBot names by their numeric suffix."""
    match = ROBOT_RE.match(str(name))
    return int(str(name).rsplit("_", 1)[1]) if match else math.inf


def formation_active_status_failures(
    status, task_id, case, robot_ids, maximum_error,
):
    """Validate one fresh status sample from the moving-formation window."""
    failures = []
    if not isinstance(status, dict):
        return ["moving formation status was unavailable"]
    if str(status.get("task_id") or "") != str(task_id or ""):
        failures.append("moving formation status was not task-correlated")
    if status.get("state") != "moving":
        failures.append("moving formation did not remain in moving state")
    if status.get("movement_mode") != "moving":
        failures.append("moving formation status reported a different mode")
    if status.get("paused") is not False:
        failures.append("moving formation status was paused")
    if status.get("error"):
        failures.append("moving formation status reported an error")
    if status.get("stale_odometry") or status.get("waiting_for_odometry"):
        failures.append("moving formation did not retain complete odometry")

    expected_shape = str(case.get("shape") or "").strip().lower()
    reported_shape = str(
        status.get("formation_type") or ""
    ).strip().lower()
    if reported_shape != expected_shape:
        failures.append("moving formation status reported a different shape")
    try:
        reported_count = int(status.get("robot_count"))
    except (TypeError, ValueError):
        reported_count = None
    if reported_count != int(case.get("count", 0)):
        failures.append("moving formation status did not cover the fleet")

    assignments = status.get("robot_assignments")
    assignment_names = set()
    if isinstance(assignments, dict):
        assignment_names = {
            clean_robot_name(name) for name in assignments
        }
        assignment_names.discard(None)
    if assignment_names != set(robot_ids):
        failures.append("moving formation assignments were incomplete")

    reported_error = finite_number(status.get("maximum_position_error"))
    if reported_error is None:
        failures.append("moving formation omitted its position error")
    elif reported_error > maximum_error:
        failures.append("moving formation position error exceeded the limit")
    return failures


def formation_is_ready_or_terminal(
    task, status, task_id, case, robot_ids, maximum_error,
):
    """Wake a moving-formation wait only for readiness or a final state."""
    if (
        not isinstance(task, dict)
        or task.get("task_id") != task_id
    ):
        return False
    task_status = task.get("status")
    if task_status in TERMINAL_STATES:
        return True
    return (
        task_status == "running"
        and not formation_active_status_failures(
            status, task_id, case, robot_ids, maximum_error
        )
    )


def validate_formation_active_selection(seconds, cases):
    """Keep the optional matrix window explicit and bounded."""
    try:
        active_seconds = float(seconds)
    except (TypeError, ValueError):
        raise ValueError("--formation-active-seconds must be a number")
    if not math.isfinite(active_seconds):
        raise ValueError("--formation-active-seconds must be finite")
    if active_seconds < 0.0:
        raise ValueError("--formation-active-seconds cannot be negative")
    if 0.0 < active_seconds < MINIMUM_FORMATION_ACTIVE_SECONDS:
        raise ValueError(
            "--formation-active-seconds must be zero or at least {:.0f}".format(
                MINIMUM_FORMATION_ACTIVE_SECONDS
            )
        )
    if active_seconds > 120.0:
        raise ValueError("--formation-active-seconds cannot exceed 120")
    if active_seconds and (
        len(cases) != 1 or cases[0]["behavior"] != "formation"
    ):
        raise ValueError(
            "--formation-active-seconds requires exactly one formation scenario"
        )
    return active_seconds


def validate_follow_active_selection(seconds, cases):
    """Keep a selected follow task alive long enough for an external GUI gate."""
    try:
        active_seconds = float(seconds)
    except (TypeError, ValueError):
        raise ValueError("--follow-active-seconds must be a number")
    if not math.isfinite(active_seconds):
        raise ValueError("--follow-active-seconds must be finite")
    if active_seconds < 0.0:
        raise ValueError("--follow-active-seconds cannot be negative")
    if 0.0 < active_seconds < MINIMUM_FOLLOW_ACTIVE_SECONDS:
        raise ValueError(
            "--follow-active-seconds must be zero or at least {:.0f}".format(
                MINIMUM_FOLLOW_ACTIVE_SECONDS
            )
        )
    if active_seconds > 240.0:
        raise ValueError("--follow-active-seconds cannot exceed 240")
    if active_seconds and (
        len(cases) != 1 or cases[0]["behavior"] != "follow"
    ):
        raise ValueError(
            "--follow-active-seconds requires exactly one follow scenario"
        )
    if active_seconds and active_seconds > float(cases[0]["duration"]):
        raise ValueError(
            "--follow-active-seconds cannot exceed the scenario duration"
        )
    return active_seconds


def transport_role_metadata(status):
    """Normalize current and older transport role-status layouts."""
    if not isinstance(status, dict):
        return {}

    raw_roles = None
    for key in ("robot_assignments", "transport_roles", "robot_roles", "roles"):
        candidate = status.get(key)
        if isinstance(candidate, dict):
            raw_roles = candidate
            break
    if raw_roles is None:
        return {}

    roles = {}
    for raw_name, raw_role in raw_roles.items():
        name = clean_robot_name(raw_name)
        if name is None or not isinstance(raw_role, dict):
            continue
        role_name = str(raw_role.get("role", "")).strip().lower()
        if role_name in {"push", "pusher", "direct", "payload"}:
            role_name = "payload_push"
        elif role_name in {"support", "companion", "chain_push"}:
            role_name = "companion_push"
        if role_name not in {"payload_push", "companion_push"}:
            continue

        parent = None
        for key in (
            "parent_namespace", "predecessor_namespace", "predecessor", "parent"
        ):
            parent = clean_robot_name(raw_role.get(key))
            if parent is not None:
                break
        try:
            chain_index = int(raw_role.get("chain_index"))
        except (TypeError, ValueError):
            chain_index = None
        try:
            depth = int(raw_role.get(
                "chain_depth", raw_role.get("depth", 0)
            ))
        except (TypeError, ValueError):
            depth = 0
        roles[name] = {
            "role": role_name,
            "chain_index": chain_index,
            "depth": depth,
            "parent_namespace": parent,
        }

    # parent_namespace is authoritative.  Chain coordinates are a fallback
    # for brief compatibility with controller builds that did not publish it.
    owners = {
        (item["chain_index"], item["depth"]): name
        for name, item in roles.items()
        if item["chain_index"] is not None
    }
    for item in roles.values():
        if item["role"] != "companion_push" or item["parent_namespace"]:
            continue
        item["parent_namespace"] = owners.get(
            (item["chain_index"], item["depth"] - 1)
        )
    return roles


def transport_search_motion_failures(
    response, robot_count, require_search=False,
):
    """Validate fleet motion, optionally requiring a real SEARCH window."""
    failures = []
    search_supported = bool(response.get("search_motion_window_supported"))
    if require_search and not search_supported:
        failures.append(
            "active search was not observed for the required minimum motion "
            "window"
        )
    if (
        search_supported
        and int(response.get("peak_simultaneous_movers", 0) or 0)
        < robot_count
    ):
        failures.append(
            "the complete fleet was never observed moving simultaneously "
            "during active search"
        )
    missing_search = response.get(
        "robots_not_observed_moving_during_search", []
    )
    if search_supported and missing_search:
        failures.append(
            "robots were not observed participating in active search: "
            + ", ".join(missing_search)
        )
    if require_search and search_supported:
        if "robots_below_required_search_travel" not in response:
            failures.append("per-robot active-search travel was not measured")
        else:
            below_required = response.get(
                "robots_below_required_search_travel", []
            )
            if below_required:
                failures.append(
                    "robots did not sustain enough active-search travel: "
                    + ", ".join(below_required)
                )
    return failures


def classify_contact_episodes(
    behavior, collision_delta, metrics, args, collision_attribution=None,
):
    """Classify the filtered safety-contact counter without relabelling it.

    ObstacleAvoidance removes the payload and declared chain neighbours from
    this signal before it is published.  Required docking is proved separately
    by payload/predecessor contact samples and ground-truth geometry.  A rise
    in this already-filtered counter is therefore unexpected in every phase;
    final docking evidence must never turn it back into an allowed contact.
    """
    try:
        raw_delta = max(0, int(collision_delta))
    except (TypeError, ValueError):
        raw_delta = 0

    participation = metrics.get("transport_participation", {})
    if not isinstance(participation, dict):
        participation = {}

    expected_contact_robots = []
    missing_expected_contact_robots = []
    declared_chain_pairs = set()
    for robot, item in sorted(participation.items()):
        if not isinstance(item, dict):
            missing_expected_contact_robots.append(robot)
            continue
        role = item.get("role")
        if role == "payload_push":
            contact_seen = (
                finite_number(item.get("direct_contact_samples")) or 0.0
            ) > 0.0
        elif role == "companion_push":
            contact_seen = (
                finite_number(item.get("companion_contact_samples")) or 0.0
            ) > 0.0
            parents = item.get("declared_parent_namespaces", [])
            if not isinstance(parents, (list, tuple, set)):
                parents = []
            for parent in parents:
                if parent in participation:
                    declared_chain_pairs.add(frozenset((robot, parent)))
        else:
            contact_seen = False

        target = (
            expected_contact_robots
            if contact_seen else missing_expected_contact_robots
        )
        target.append(robot)

    robot_count = len(participation)
    possible_pairs = robot_count * (robot_count - 1) // 2
    unexpected_pair_possible = possible_pairs > len(declared_chain_pairs)

    geometry = {
        "minimum_unexpected_robot_center_distance_m": finite_number(metrics.get(
            "minimum_unexpected_robot_center_distance_m"
        )),
        "minimum_static_obstacle_clearance_m": finite_number(metrics.get(
            "minimum_static_obstacle_clearance_m"
        )),
        "minimum_boundary_clearance_m": finite_number(metrics.get(
            "minimum_boundary_clearance_m"
        )),
        "minimum_declared_chain_center_distance_m": finite_number(metrics.get(
            "minimum_declared_chain_center_distance_m"
        )),
    }
    limits = {
        "minimum_unexpected_robot_center_distance_m": (
            args.min_center_distance
        ),
        "minimum_static_obstacle_clearance_m": (
            args.min_obstacle_clearance
        ),
        "minimum_boundary_clearance_m": args.min_boundary_clearance,
        "minimum_declared_chain_center_distance_m": (
            args.min_transport_chain_center_distance
        ),
    }
    crossed_limits = []
    for name, value in geometry.items():
        if value is not None and value < limits[name]:
            crossed_limits.append(name)

    missing_geometry = []
    if unexpected_pair_possible and geometry[
        "minimum_unexpected_robot_center_distance_m"
    ] is None:
        missing_geometry.append(
            "minimum_unexpected_robot_center_distance_m"
        )
    if metrics.get("model_samples", 0) and geometry[
        "minimum_boundary_clearance_m"
    ] is None:
        missing_geometry.append("minimum_boundary_clearance_m")

    attribution = (
        collision_attribution
        if isinstance(collision_attribution, dict) else {}
    )
    temporal_attribution_complete = (
        raw_delta == 0
        or bool(attribution.get("temporal_attribution_complete"))
    )
    try:
        pre_docking_delta = max(
            0, int(attribution.get("pre_docking_episode_count", 0) or 0)
        )
    except (TypeError, ValueError):
        pre_docking_delta = 0
    # A sampled early edge is authoritative negative evidence even when
    # packet loss prevents complete attribution of the aggregate counter.
    # Clamp it to the authoritative delta so diagnostics remain additive.
    pre_docking_delta = min(raw_delta, pre_docking_delta)

    if raw_delta == 0:
        classification = "none"
        reasons = ["the filtered safety-contact counter did not increase"]
    else:
        classification = "unexpected_contact"
        reasons = [
            "the safety-contact signal already excludes declared payload "
            "and chain docking",
        ]
        if behavior != "transport":
            reasons.append(
                "{} does not require physical docking".format(behavior)
            )
        if not temporal_attribution_complete:
            reasons.append(
                "not every aggregate episode had a correlated sealed event"
            )
        if pre_docking_delta:
            reasons.append(
                "at least one episode was observed before the docking window"
            )
        if crossed_limits:
            reasons.append("ground-truth clearance crossed a safety limit")
        if missing_geometry:
            reasons.append("ground-truth clearance telemetry was incomplete")

    expected_delta = 0
    unexpected_delta = raw_delta
    return {
        "raw_collision_count_delta": raw_delta,
        "counter_metric": "per_robot_geometric_contact_episodes",
        "classification_basis": "filtered_safety_contact_counter",
        "classification_is_inferred": True,
        "temporal_attribution_used": bool(attribution),
        "temporal_attribution_complete": temporal_attribution_complete,
        "pre_docking_episode_count": pre_docking_delta,
        "transport_docking_phases": sorted(TRANSPORT_DOCKING_PHASES),
        "classification": classification,
        "classified_expected_contact_count_delta": expected_delta,
        "classified_unexpected_contact_count_delta": unexpected_delta,
        "hard_failure": unexpected_delta > 0,
        "expected_contact_robots": expected_contact_robots,
        "missing_expected_contact_robots": missing_expected_contact_robots,
        "unexpected_robot_pair_measurement_required": (
            unexpected_pair_possible
        ),
        "crossed_clearance_limits": crossed_limits,
        "missing_clearance_measurements": missing_geometry,
        "clearance_measurements_m": geometry,
        "clearance_limits_m": limits,
        "reasons": reasons,
    }


def collision_attribution_report(
    aggregate_delta, episode_counts, status_samples, missing_robot_ids,
    episode_records=None, protocol_errors=None, duplicate_event_count=0,
    baseline_sequence=None, last_sequence=None,
):
    """Corroborate the aggregate counter with sealed collision events.

    The producer records task identity and transport phase under the same lock
    that increments the counter.  Any stream discontinuity or correlation
    error makes attribution incomplete, so docking cannot excuse the raw
    aggregate delta.
    """
    aggregate = max(0, int(aggregate_delta or 0))
    counts = {
        name: max(0, int(count))
        for name, count in sorted(
            (episode_counts or {}).items(), key=lambda item: robot_sort_key(item[0])
        )
        if int(count) > 0
    }
    observed = sum(counts.values())
    missing = sorted(set(missing_robot_ids or ()), key=robot_sort_key)
    samples = max(0, int(status_samples or 0))
    errors = [
        str(error) for error in (protocol_errors or ()) if str(error)
    ]
    matched = (
        samples > 0
        and not missing
        and not errors
        and observed == aggregate
    )
    records = []
    phase_counts = {}
    pre_docking_count = 0
    docking_count = 0
    for item in episode_records or ():
        if not isinstance(item, dict):
            continue
        robot = clean_robot_name(item.get("robot_id"))
        phase = str(item.get("phase") or "UNOBSERVED").strip().upper()
        if phase not in TRANSPORT_PHASES:
            phase = "UNOBSERVED"
        docking_window_open = bool(item.get("docking_window_open"))
        if docking_window_open and phase not in TRANSPORT_DOCKING_PHASES:
            # Never trust a caller-provided boolean that contradicts the
            # phase recorded at the same edge.
            docking_window_open = False
        record = {
            "robot_id": robot or str(item.get("robot_id") or "unknown"),
            "phase": phase,
            "docking_window_open": docking_window_open,
        }
        sequence = item.get("sequence")
        if isinstance(sequence, int) and not isinstance(sequence, bool):
            record["sequence"] = sequence
        for key in ("task_id", "task_type"):
            value = item.get(key)
            if isinstance(value, str) and value:
                record[key] = value
        source_id = item.get("source_id")
        if isinstance(source_id, str) and source_id:
            record["source_id"] = source_id
        for key in ("source_sequence", "source_control_sequence"):
            value = item.get(key)
            if (
                isinstance(value, int)
                and not isinstance(value, bool)
                and value > 0
            ):
                record[key] = value
        for key in (
            "wall_elapsed_s", "sim_elapsed_s",
            "observed_wall_elapsed_s", "observed_sim_elapsed_s",
            "source_wall_elapsed_s", "source_sim_elapsed_s",
            "source_wall_time", "source_sim_time",
        ):
            value = finite_number(item.get(key))
            if value is not None:
                record[key] = rounded(value)
        records.append(record)
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
        if docking_window_open:
            docking_count += 1
        else:
            pre_docking_count += 1

    if errors:
        reason = "sealed collision event stream failed validation"
    elif samples == 0:
        reason = "no sealed collision event stream was sampled"
    elif missing:
        reason = "some correlated status samples omitted case robots"
    elif observed != aggregate:
        reason = (
            "sampled per-robot rising edges did not match the aggregate "
            "episode delta"
        )
    else:
        reason = (
            "sealed rising edges matched the authoritative aggregate "
            "episode delta"
        )

    return {
        "source": "/swarm/status collision_events.events sealed rising edges",
        "event_stream_version": COLLISION_EVENT_STREAM_VERSION,
        "aggregate_collision_count_delta": aggregate,
        "correlated_status_samples": samples,
        "observed_episode_count": observed,
        "episode_count_by_robot": counts,
        "episode_count_by_phase": {
            phase: phase_counts[phase] for phase in sorted(phase_counts)
        },
        "episode_records": records,
        "pre_docking_episode_count": pre_docking_count,
        "docking_window_episode_count": docking_count,
        "transport_docking_phases": sorted(TRANSPORT_DOCKING_PHASES),
        "temporal_attribution_complete": (
            matched and len(records) == aggregate
        ),
        "robots_with_observed_episodes": list(counts),
        "missing_robot_ids": missing,
        "aggregate_count_matched": observed == aggregate,
        "attribution_corroborated": matched,
        "unattributed_episode_count": max(0, aggregate - observed),
        "protocol_errors": errors,
        "duplicate_event_count": max(0, int(duplicate_event_count or 0)),
        "baseline_sequence": baseline_sequence,
        "last_sequence": last_sequence,
        "reason": reason,
    }


class CaseMetrics:
    def __init__(self, case, robot_ids, sim_time, wall_time, args):
        self.case = case
        self.args = args
        self.robot_ids = set(robot_ids)
        self.wall_start = wall_time
        self.wall_end = wall_time
        self.sim_start = sim_time
        self.sim_end = sim_time
        self.model_samples = 0
        self.initial_positions = {}
        self.last_positions = {}
        self.final_positions = {}
        self.travel = {name: 0.0 for name in robot_ids}
        self.speeds = []
        self.accelerations = []
        self.instantaneous_accelerations = []
        self.maximum_acceleration_event = None
        self.maximum_instantaneous_acceleration_event = None
        self.last_velocity = {}
        self.velocity_history = {
            name: deque() for name in robot_ids
        }
        self.last_model_sim = None
        self.correlated_behavior_samples = 0
        self.last_behavior_status = {}
        self.formation_targets = {}
        self.follow_chain = []
        self.follow_distance = None
        self.min_center_distance = None
        self.min_center_pair = None
        self.min_unexpected_center_distance = None
        self.min_unexpected_center_pair = None
        self.min_unexpected_center_event = None
        self.min_declared_chain_distance = None
        self.min_declared_chain_pair = None
        self.min_declared_chain_event = None
        self.min_obstacle_clearance = None
        self.min_obstacle_pair = None
        self.min_boundary_clearance = None
        self.object_initial = None
        self.object_final = None
        self.object_initial_yaw = None
        self.object_final_yaw = None
        self.object_last = None
        self.object_travel = 0.0
        self.object_goal_initial_distance = None
        self.object_goal_final_distance = None
        self.object_goal_best_distance = None
        self.transport_push_samples = 0
        self.transport_active_push_latched = False
        self.transport_push_object_initial = None
        self.transport_push_object_last = None
        self.transport_push_object_final = None
        self.transport_push_object_travel = 0.0
        self.transport_push_goal_initial_distance = None
        self.transport_push_goal_final_distance = None
        self.transport_grf_samples = 0
        self.transport_control_sequences = set()
        self.transport_duplicate_control_sequences = 0
        self.transport_last_control_sequence = None
        self.transport_last_control_sim_time = None
        self.transport_best_control_goal_distance = None
        self.transport_positive_goal_progress = 0.0
        self.transport_reference_pace_samples = 0
        self.transport_all_contribution_progress = 0.0
        self.transport_contribution_progress = {
            name: 0.0 for name in robot_ids
        }
        self.transport_command_goal_speeds = {
            name: [] for name in robot_ids
        }
        self.transport_role_reference_goal_speeds = {
            name: [] for name in robot_ids
        }
        self.transport_adaptive_intent_thresholds = {
            name: [] for name in robot_ids
        }
        self.first_transport_connection_loss = None
        self.transport_all_useful_samples = 0
        self.transport_first_all_useful_sim_time = None
        self.transport_first_all_useful_wall_time = None
        self.current_all_useful_start = None
        self.current_all_useful_samples = 0
        self.last_all_useful_sample = None
        self.max_all_useful_duration = 0.0
        self.max_all_useful_samples = 0
        self.transport_all_nominal_pace_samples = 0
        self.transport_all_reference_pace_samples = 0
        self.current_all_nominal_pace_start = None
        self.current_all_nominal_pace_samples = 0
        self.last_all_nominal_pace_sample = None
        self.max_all_nominal_pace_duration = 0.0
        self.max_all_nominal_pace_samples = 0
        self.transport_counts = {
            name: {
                "declared": 0,
                "direct_contact": 0,
                "companion_contact": 0,
                "connected": 0,
                "push_intent": 0,
                "useful_pushing": 0,
                "nominal_pace_intent": 0,
                "nominal_pace_pushing": 0,
                "reference_pace_intent": 0,
            }
            for name in robot_ids
        }
        self.transport_role_counts = {name: {} for name in robot_ids}
        self.transport_parents = {name: set() for name in robot_ids}
        self.current_push_streak_start = {name: None for name in robot_ids}
        self.current_push_streak_samples = {name: 0 for name in robot_ids}
        self.last_push_streak_sample = {name: None for name in robot_ids}
        self.max_push_streak_duration = {name: 0.0 for name in robot_ids}
        self.max_push_streak_samples = {name: 0 for name in robot_ids}
        self.current_pace_streak_start = {name: None for name in robot_ids}
        self.current_pace_streak_samples = {name: 0 for name in robot_ids}
        self.last_pace_streak_sample = {name: None for name in robot_ids}
        self.max_pace_streak_duration = {name: 0.0 for name in robot_ids}
        self.max_pace_streak_samples = {name: 0 for name in robot_ids}
        self.transport_discovery_notice = None
        self.transport_discovery_observed_sim = None
        self.transport_discovery_observed_wall = None
        self.transport_discovery_ack_available = False
        self.transport_discovery_acknowledged = set()
        self.transport_discovery_roles = {}
        self.transport_discovery_start_positions = {}
        self.transport_discovery_last_positions = {}
        self.transport_discovery_final_positions = {}
        self.transport_discovery_travel = {
            name: 0.0 for name in robot_ids
        }
        self.transport_discovery_first_motion_sim = {}
        self.transport_discovery_first_motion_wall = {}
        self.transport_peak_simultaneous_movers = set()
        self.transport_peak_rendezvous_movers = set()
        self.transport_search_motion_samples = 0
        self.transport_rendezvous_motion_samples = 0
        self.transport_search_movers = set()
        self.transport_search_last_positions = {}
        self.transport_search_travel = {
            name: 0.0 for name in robot_ids
        }

    def _observe_transport_discovery_status(self, status):
        """Remember who received the payload notice and their assigned role."""
        if self.case["behavior"] != "transport":
            return

        discovery = status.get("discovery")
        if isinstance(discovery, dict) and discovery.get("announced", True):
            if self.transport_discovery_notice is None:
                self.transport_discovery_notice = dict(discovery)

        roles = transport_role_metadata(status)
        if roles:
            self.transport_discovery_roles.update(roles)

        assignments = status.get("robot_assignments")
        if not isinstance(assignments, dict):
            return
        for raw_name, assignment in assignments.items():
            name = clean_robot_name(raw_name)
            if name not in self.robot_ids or not isinstance(assignment, dict):
                continue
            if "notice_received" in assignment:
                self.transport_discovery_ack_available = True
            if assignment.get("notice_received") is True:
                self.transport_discovery_acknowledged.add(name)

    def observe_behavior_status(self, status):
        """Remember geometry from a status already matched to this task."""
        if not isinstance(status, dict) or not status:
            return

        self.correlated_behavior_samples += 1
        self.last_behavior_status = dict(status)
        self._observe_transport_discovery_status(status)

        if self.case["behavior"] == "formation":
            assignments = status.get("robot_assignments")
            if not isinstance(assignments, dict):
                return
            targets = {}
            for raw_name, assignment in assignments.items():
                name = clean_robot_name(raw_name)
                if name not in self.robot_ids or not isinstance(assignment, dict):
                    continue
                try:
                    target = (
                        float(assignment["target_x"]),
                        float(assignment["target_y"]),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                if all(math.isfinite(value) for value in target):
                    targets[name] = target
            if targets:
                self.formation_targets = targets

        if self.case["behavior"] == "follow":
            entries = status.get("robots")
            ordered = []
            if isinstance(entries, list):
                for entry in entries:
                    if not isinstance(entry, dict):
                        continue
                    name = clean_robot_name(entry.get("name"))
                    if name not in self.robot_ids:
                        continue
                    try:
                        chain_index = int(entry.get("chain_index"))
                    except (TypeError, ValueError):
                        continue
                    ordered.append((chain_index, name))
            chain = [name for _, name in sorted(ordered)]
            if len(chain) == len(self.robot_ids) and set(chain) == self.robot_ids:
                self.follow_chain = chain
            try:
                distance = float(status.get("follow_distance"))
            except (TypeError, ValueError):
                distance = None
            if distance is not None and math.isfinite(distance) and distance > 0.0:
                self.follow_distance = distance

    def _transport_discovery_motion(
        self, robot_positions, twist_by_name, behavior_status, sim_time,
        wall_time,
    ):
        """Measure concurrent motion during search and the notice response."""
        phase = str(behavior_status.get("phase", "")).strip().upper()
        movers = set()
        for robot in self.robot_ids:
            if robot not in robot_positions:
                continue
            twist = twist_by_name.get(robot)
            if twist is None:
                continue
            speed = math.hypot(twist.linear.x, twist.linear.y)
            if speed >= self.args.transport_rendezvous_moving_speed:
                movers.add(robot)
        motion_frame_complete = (
            self.robot_ids <= set(robot_positions)
            and all(twist_by_name.get(robot) is not None
                    for robot in self.robot_ids)
        )

        if phase == "SEARCH":
            if not motion_frame_complete:
                return
            for robot in self.robot_ids:
                position = robot_positions[robot]
                previous = self.transport_search_last_positions.get(robot)
                if previous is not None:
                    self.transport_search_travel[robot] += math.hypot(
                        position[0] - previous[0],
                        position[1] - previous[1],
                    )
                self.transport_search_last_positions[robot] = position
            self.transport_search_motion_samples += 1
            self.transport_search_movers.update(movers)
            if len(movers) > len(self.transport_peak_simultaneous_movers):
                self.transport_peak_simultaneous_movers = set(movers)
            return

        if self.transport_discovery_notice is None:
            return

        if self.transport_discovery_observed_wall is None:
            self.transport_discovery_observed_wall = wall_time
            self.transport_discovery_observed_sim = sim_time
            for robot, position in robot_positions.items():
                self.transport_discovery_start_positions[robot] = position
                self.transport_discovery_last_positions[robot] = position
                self.transport_discovery_final_positions[robot] = position

        if phase != "APPROACH":
            return
        if motion_frame_complete:
            self.transport_rendezvous_motion_samples += 1

        for robot in self.robot_ids:
            position = robot_positions.get(robot)
            if position is None:
                continue

            self.transport_discovery_start_positions.setdefault(
                robot, position
            )
            previous = self.transport_discovery_last_positions.get(robot)
            if previous is not None:
                self.transport_discovery_travel[robot] += math.hypot(
                    position[0] - previous[0], position[1] - previous[1]
                )
            self.transport_discovery_last_positions[robot] = position
            self.transport_discovery_final_positions[robot] = position

            if (
                robot not in self.transport_discovery_first_motion_wall
                and self.transport_discovery_travel[robot]
                >= self.args.transport_motion_detection_distance
            ):
                self.transport_discovery_first_motion_sim[robot] = sim_time
                self.transport_discovery_first_motion_wall[robot] = wall_time

        if (
            motion_frame_complete
            and len(movers) > len(self.transport_peak_rendezvous_movers)
        ):
            self.transport_peak_rendezvous_movers = set(movers)

    def _update_push_streak(self, robot, pushing, sample_time):
        start = self.current_push_streak_start[robot]
        previous = self.last_push_streak_sample[robot]
        samples = self.current_push_streak_samples[robot]
        is_contiguous = (
            start is not None
            and previous is not None
            and 0.0 <= sample_time - previous
            <= self.args.transport_control_gap_tolerance
        )
        if pushing:
            if not is_contiguous:
                start = sample_time
                samples = 0
            samples += 1
            self.current_push_streak_start[robot] = start
            self.current_push_streak_samples[robot] = samples
            self.last_push_streak_sample[robot] = sample_time
            duration = max(0.0, sample_time - start)
            self.max_push_streak_duration[robot] = max(
                self.max_push_streak_duration[robot], duration
            )
            self.max_push_streak_samples[robot] = max(
                self.max_push_streak_samples[robot], samples
            )
            return
        self.current_push_streak_start[robot] = None
        self.current_push_streak_samples[robot] = 0
        self.last_push_streak_sample[robot] = None

    def _update_pace_streak(self, robot, at_pace, sample_time):
        start = self.current_pace_streak_start[robot]
        previous = self.last_pace_streak_sample[robot]
        samples = self.current_pace_streak_samples[robot]
        is_contiguous = (
            start is not None
            and previous is not None
            and 0.0 <= sample_time - previous
            <= self.args.transport_control_gap_tolerance
        )
        if at_pace:
            if not is_contiguous:
                start = sample_time
                samples = 0
            samples += 1
            self.current_pace_streak_start[robot] = start
            self.current_pace_streak_samples[robot] = samples
            self.last_pace_streak_sample[robot] = sample_time
            duration = max(0.0, sample_time - start)
            self.max_pace_streak_duration[robot] = max(
                self.max_pace_streak_duration[robot], duration
            )
            self.max_pace_streak_samples[robot] = max(
                self.max_pace_streak_samples[robot], samples
            )
            return
        self.current_pace_streak_start[robot] = None
        self.current_pace_streak_samples[robot] = 0
        self.last_pace_streak_sample[robot] = None

    def _transport_control_batch(self, status):
        """Return one new, complete controller batch from transport status."""
        try:
            sequence = int(status.get("control_sequence"))
        except (TypeError, ValueError):
            return None
        control_time = finite_number(status.get("control_sim_time"))
        raw_commands = status.get("control_commands")
        if sequence <= 0 or control_time is None or not isinstance(
            raw_commands, dict
        ):
            return None
        if sequence in self.transport_control_sequences:
            self.transport_duplicate_control_sequences += 1
            return None

        commands = {}
        for raw_name, raw_command in raw_commands.items():
            name = clean_robot_name(raw_name)
            if name not in self.robot_ids or not isinstance(raw_command, dict):
                continue
            linear = finite_number(raw_command.get("linear"))
            angular = finite_number(raw_command.get("angular"))
            if linear is None:
                continue
            commands[name] = {
                "linear": linear,
                "angular": 0.0 if angular is None else angular,
            }

        self.transport_control_sequences.add(sequence)
        self.transport_last_control_sequence = sequence
        self.transport_last_control_sim_time = control_time
        return sequence, control_time, commands

    def _transport_command_intent(
        self, robot, role, command, model_pose, twist_by_name,
        goal_x, goal_y, reference_speed,
    ):
        """Describe contribution intent and the two independent pace checks."""
        pose = model_pose.get(robot)
        if pose is None or command is None:
            return {
                "positive": False,
                "nominal_pace": False,
                "reference_pace": False,
                "command_goal_speed": None,
                "reference_goal_speed": None,
                "adaptive_threshold": None,
            }

        heading_x, heading_y = math.cos(pose[2]), math.sin(pose[2])
        goal_cosine = heading_x * goal_x + heading_y * goal_y
        command_goal_speed = command["linear"] * goal_cosine

        reference_name = "transport_object"
        if role.get("role") == "companion_push":
            reference_name = role.get("parent_namespace")
        reference_twist = twist_by_name.get(reference_name)
        reference_goal_speed = 0.0
        if reference_twist is not None:
            reference_goal_speed = max(
                0.0,
                float(reference_twist.linear.x) * goal_x
                + float(reference_twist.linear.y) * goal_y,
            )

        # Ground-truth velocity can briefly exceed the requested fleet pace:
        # two payload roots act together and the box also carries momentum.
        # That must not make a robot fail for obeying the controller's own
        # published reference.  The measured velocity remains in diagnostics,
        # while the requested pace caps the tracking demand used for intent.
        tracking_reference_speed = reference_goal_speed
        if reference_speed is not None:
            tracking_reference_speed = min(
                tracking_reference_speed,
                max(0.0, reference_speed),
            )

        noise_floor = self.args.transport_contribution_noise_floor
        tolerance = self.args.transport_contribution_speed_tolerance
        tracking_fraction = (
            self.args.transport_contribution_tracking_fraction
        )
        adaptive_threshold = max(
            noise_floor,
            min(
                tracking_reference_speed - tolerance,
                tracking_fraction * tracking_reference_speed,
            ),
        )
        positive = command_goal_speed >= adaptive_threshold

        # Keep the original fixed-throughput check intact.  It is deliberately
        # not the contribution definition: near the goal a coordinated chain
        # can still exert useful positive intent below this nominal pace.
        nominal_pace = (
            command["linear"] >= self.args.transport_cmd_min_speed
            and goal_cosine >= self.args.transport_cmd_min_goal_cosine
        )
        reference_pace = None
        if reference_speed is not None:
            reference_threshold = max(
                noise_floor, max(0.0, reference_speed) - tolerance
            )
            reference_pace = command_goal_speed >= reference_threshold

        return {
            "positive": positive,
            "nominal_pace": nominal_pace,
            "reference_pace": reference_pace,
            "command_goal_speed": command_goal_speed,
            "reference_goal_speed": reference_goal_speed,
            "adaptive_threshold": adaptive_threshold,
        }

    def _transport_participation(
        self, robot_positions, model_pose, twist_by_name, behavior_status,
        cmd_velocities, sim_time, wall_time, zones,
    ):
        phase = str(behavior_status.get("phase", "")).strip().upper()
        if not self.transport_active_push_latched and phase != "PUSH":
            return
        if self.transport_active_push_latched and phase not in {
            "PUSH", "APPROACH"
        }:
            return
        object_state = model_pose.get("transport_object")
        if object_state is None:
            return

        roles = transport_role_metadata(behavior_status)
        object_x, object_y, object_yaw = object_state
        target_x, target_y = self.case["target"]
        goal_x, goal_y = target_x - object_x, target_y - object_y
        goal_norm = math.hypot(goal_x, goal_y)
        if goal_norm <= 1e-9:
            return
        goal_x, goal_y = goal_x / goal_norm, goal_y / goal_norm
        goal_left_x, goal_left_y = -goal_y, goal_x
        goal_bearing = math.atan2(goal_y, goal_x)

        payload_zone = next(
            (zone for zone in zones
             if zone.get("model") == "transport_object"),
            {
                "shape": "box", "model": "transport_object",
                "width": 0.4, "height": 0.4,
            },
        )
        direct = set()
        payload_geometry = {}
        for robot, position in robot_positions.items():
            clearance = point_clearance(
                position[0], position[1], payload_zone, model_pose
            )
            relative_x = position[0] - object_x
            relative_y = position[1] - object_y
            rear_offset = (
                relative_x * goal_x + relative_y * goal_y
            )
            lane_offset = (
                relative_x * goal_left_x + relative_y * goal_left_y
            )
            has_direct_contact = (
                clearance <= self.args.transport_direct_contact_clearance
                and rear_offset <= -0.05
            )
            payload_geometry[robot] = {
                "surface_clearance_m": clearance,
                "rear_offset_m": rear_offset,
                "lane_offset_m": lane_offset,
                "direct_contact": has_direct_contact,
            }
            if has_direct_contact:
                direct.add(robot)

        companion_contact = set()
        parent_geometry = {}
        for robot, role in roles.items():
            parent = role.get("parent_namespace")
            if (
                robot not in robot_positions
                or parent not in robot_positions
                or role.get("role") != "companion_push"
            ):
                continue
            child_pos = robot_positions[robot]
            parent_pos = robot_positions[parent]
            to_parent_x = parent_pos[0] - child_pos[0]
            to_parent_y = parent_pos[1] - child_pos[1]
            distance = math.hypot(to_parent_x, to_parent_y)
            parent_ahead = to_parent_x * goal_x + to_parent_y * goal_y
            parent_lateral = (
                to_parent_x * goal_left_x + to_parent_y * goal_left_y
            )
            if distance > 1e-9:
                parent_alignment = parent_ahead / distance
                parent_angle = math.atan2(to_parent_y, to_parent_x)
                parent_goal_error = math.atan2(
                    math.sin(goal_bearing - parent_angle),
                    math.cos(goal_bearing - parent_angle),
                )
            else:
                parent_alignment = None
                parent_goal_error = None
            has_companion_contact = (
                distance <= self.args.transport_companion_contact_distance
                and parent_ahead >= 0.05
            )
            parent_geometry[robot] = {
                "namespace": parent,
                "center_distance_m": distance,
                "ahead_offset_m": parent_ahead,
                "lateral_offset_m": parent_lateral,
                "goal_alignment_cosine": parent_alignment,
                "goal_angle_deg": (
                    math.degrees(parent_goal_error)
                    if parent_goal_error is not None else None
                ),
                "contact": has_companion_contact,
            }
            if has_companion_contact:
                companion_contact.add(robot)

        # A chain member counts only when every predecessor leads to a robot
        # that has preferred direct contact with the payload.
        connected = set(direct)
        changed = True
        while changed:
            changed = False
            for robot in companion_contact - connected:
                parent = roles.get(robot, {}).get("parent_namespace")
                if parent in connected:
                    connected.add(robot)
                    changed = True

        engagement_complete = bool(
            behavior_status.get("engagement_complete", False)
        )
        synchronized = bool(
            behavior_status.get("synchronized_push_started", False)
        )
        try:
            grf_iterations = int(
                behavior_status.get("grf_mcmc_iterations", 0) or 0
            )
        except (TypeError, ValueError):
            grf_iterations = 0
        roles_complete = self.robot_ids <= set(roles)
        geometry_complete = self.robot_ids <= connected
        active_control = (
            phase == "PUSH"
            and engagement_complete
            and synchronized
            and grf_iterations > 0
        )
        if not self.transport_active_push_latched:
            if not (
                active_control and roles_complete and geometry_complete
            ):
                return
            self.transport_active_push_latched = True
            self.transport_push_object_initial = (object_x, object_y)
            self.transport_push_object_last = (object_x, object_y)
            self.transport_push_object_final = (object_x, object_y)
            self.transport_push_goal_initial_distance = goal_norm
            self.transport_best_control_goal_distance = goal_norm

        if (
            self.first_transport_connection_loss is None
            and not geometry_complete
        ):
            object_twist = twist_by_name.get("transport_object")
            object_velocity_x = None
            object_velocity_y = None
            object_angular_velocity = None
            if object_twist is not None:
                object_velocity_x = float(object_twist.linear.x)
                object_velocity_y = float(object_twist.linear.y)
                object_angular_velocity = float(object_twist.angular.z)

            robot_geometry = {}
            for robot in sorted(self.robot_ids, key=robot_sort_key):
                pose = model_pose.get(robot)
                role = roles.get(robot)
                command = cmd_velocities.get(robot)
                command_geometry = None
                if command is not None:
                    command_age = max(
                        0.0, wall_time - command["wall_time"]
                    )
                    command_geometry = {
                        "linear_mps": command["linear_x"],
                        "angular_rps": command["angular_z"],
                        "age_s": command_age,
                        "fresh": (
                            command_age
                            <= self.args.transport_cmd_max_age
                        ),
                    }

                gazebo_velocity = None
                robot_twist = twist_by_name.get(robot)
                if robot_twist is not None:
                    velocity_x = float(robot_twist.linear.x)
                    velocity_y = float(robot_twist.linear.y)
                    angular = getattr(robot_twist, "angular", None)
                    angular_velocity = getattr(angular, "z", None)
                    gazebo_velocity = {
                        "linear_velocity_xy_mps": [
                            velocity_x, velocity_y
                        ],
                        "goal_velocity_mps": (
                            velocity_x * goal_x + velocity_y * goal_y
                        ),
                        "lateral_velocity_mps": (
                            velocity_x * goal_left_x
                            + velocity_y * goal_left_y
                        ),
                        "speed_mps": math.hypot(
                            velocity_x, velocity_y
                        ),
                        "angular_velocity_rps": (
                            float(angular_velocity)
                            if angular_velocity is not None else None
                        ),
                    }

                heading_alignment = None
                heading_angle = None
                pose_geometry = None
                if pose is not None:
                    heading_alignment = (
                        math.cos(pose[2]) * goal_x
                        + math.sin(pose[2]) * goal_y
                    )
                    heading_error = math.atan2(
                        math.sin(goal_bearing - pose[2]),
                        math.cos(goal_bearing - pose[2]),
                    )
                    heading_angle = math.degrees(heading_error)
                    pose_geometry = {
                        "xy": [pose[0], pose[1]],
                        "yaw_rad": pose[2],
                    }

                robot_geometry[robot] = {
                    "pose": pose_geometry,
                    "command": command_geometry,
                    "gazebo_world_velocity": gazebo_velocity,
                    "role": role.get("role") if role else None,
                    "chain_index": (
                        role.get("chain_index") if role else None
                    ),
                    "depth": role.get("depth") if role else None,
                    "parent": (
                        role.get("parent_namespace") if role else None
                    ),
                    "heading_goal_alignment_cosine": heading_alignment,
                    "heading_goal_angle_deg": heading_angle,
                    "payload": payload_geometry.get(robot),
                    "parent_link": parent_geometry.get(robot),
                    "connected": robot in connected,
                }

            root_link_failures = []
            for robot in sorted(
                self.robot_ids - connected, key=robot_sort_key
            ):
                role = roles.get(robot)
                if robot not in robot_positions:
                    reason = "missing_pose"
                elif role is None:
                    reason = "missing_role"
                elif role.get("role") == "payload_push":
                    reason = "payload_contact"
                elif robot not in companion_contact:
                    reason = "parent_contact"
                else:
                    # This robot still has local contact; its missing parent is
                    # the broken upstream link and is recorded elsewhere.
                    continue
                root_link_failures.append({
                    "robot": robot,
                    "reason": reason,
                    "parent": (
                        role.get("parent_namespace") if role else None
                    ),
                })

            self.first_transport_connection_loss = {
                "sim_time_s": sim_time,
                "phase": phase,
                "missing_robots": sorted(
                    self.robot_ids - connected, key=robot_sort_key
                ),
                "connected_robots": sorted(
                    connected, key=robot_sort_key
                ),
                "root_link_failures": root_link_failures,
                "contact_thresholds": {
                    "payload_surface_clearance_m": (
                        self.args.transport_direct_contact_clearance
                    ),
                    "companion_center_distance_m": (
                        self.args.transport_companion_contact_distance
                    ),
                    "minimum_parent_ahead_offset_m": 0.05,
                },
                "object": {
                    "xy": [object_x, object_y],
                    "yaw_rad": object_yaw,
                    "linear_velocity_xy_mps": [
                        object_velocity_x, object_velocity_y
                    ],
                    "angular_velocity_rps": object_angular_velocity,
                    "goal_velocity_mps": (
                        object_velocity_x * goal_x
                        + object_velocity_y * goal_y
                        if object_velocity_x is not None else None
                    ),
                    "lateral_velocity_mps": (
                        object_velocity_x * goal_left_x
                        + object_velocity_y * goal_left_y
                        if object_velocity_x is not None else None
                    ),
                },
                "target": {
                    "xy": [target_x, target_y],
                    "distance_m": goal_norm,
                    "direction_xy": [goal_x, goal_y],
                    "bearing_rad": goal_bearing,
                },
                "robots": robot_geometry,
            }

        if self.transport_push_object_last is not None:
            self.transport_push_object_travel += math.hypot(
                object_x - self.transport_push_object_last[0],
                object_y - self.transport_push_object_last[1],
            )
        self.transport_push_object_last = (object_x, object_y)
        self.transport_push_object_final = (object_x, object_y)
        self.transport_push_goal_final_distance = goal_norm

        # APPROACH is still observed above so a broken link is preserved in
        # diagnostics.  It is not a push-control sample, and terminal stop
        # commands must never dilute an already proven contribution window.
        if phase != "PUSH" or not active_control:
            return
        batch = self._transport_control_batch(behavior_status)
        if batch is None:
            return
        _sequence, sample_time, control_commands = batch

        self.transport_grf_samples += 1
        self.transport_push_samples += 1
        best_distance = self.transport_best_control_goal_distance
        positive_progress = (
            max(0.0, best_distance - goal_norm)
            if best_distance is not None else 0.0
        )
        if best_distance is None or goal_norm < best_distance:
            self.transport_best_control_goal_distance = goal_norm
        self.transport_positive_goal_progress += positive_progress

        reference_speed = finite_number(
            behavior_status.get("push_reference_speed")
        )
        if reference_speed is not None:
            self.transport_reference_pace_samples += 1
        useful_robots = set()
        nominal_pace_robots = set()
        reference_pace_robots = set()
        for robot in sorted(self.robot_ids):
            role = roles.get(robot)
            counts = self.transport_counts[robot]
            if role is not None:
                counts["declared"] += 1
                role_name = role["role"]
                role_counts = self.transport_role_counts[robot]
                role_counts[role_name] = role_counts.get(role_name, 0) + 1
                parent = role.get("parent_namespace")
                if parent:
                    self.transport_parents[robot].add(parent)
            if robot in direct:
                counts["direct_contact"] += 1
            if robot in companion_contact:
                counts["companion_contact"] += 1
            if robot in connected:
                counts["connected"] += 1

            intent = None
            if role is not None:
                intent = self._transport_command_intent(
                    robot, role, control_commands.get(robot), model_pose,
                    twist_by_name, goal_x, goal_y, reference_speed,
                )
            has_push_intent = bool(intent and intent["positive"])
            has_nominal_pace = bool(intent and intent["nominal_pace"])
            has_reference_pace = bool(
                intent and intent["reference_pace"] is True
            )
            if intent is not None:
                if intent["command_goal_speed"] is not None:
                    self.transport_command_goal_speeds[robot].append(
                        intent["command_goal_speed"]
                    )
                if intent["reference_goal_speed"] is not None:
                    self.transport_role_reference_goal_speeds[robot].append(
                        intent["reference_goal_speed"]
                    )
                if intent["adaptive_threshold"] is not None:
                    self.transport_adaptive_intent_thresholds[robot].append(
                        intent["adaptive_threshold"]
                    )
            if has_push_intent:
                counts["push_intent"] += 1
            if has_nominal_pace:
                counts["nominal_pace_intent"] += 1
            if has_reference_pace:
                counts["reference_pace_intent"] += 1
                if role is not None and robot in connected:
                    reference_pace_robots.add(robot)
            useful = role is not None and robot in connected and has_push_intent
            at_nominal_pace = (
                role is not None
                and robot in connected
                and has_nominal_pace
            )
            if useful:
                counts["useful_pushing"] += 1
                useful_robots.add(robot)
                self.transport_contribution_progress[robot] += (
                    positive_progress
                )
            if at_nominal_pace:
                counts["nominal_pace_pushing"] += 1
                nominal_pace_robots.add(robot)
            self._update_push_streak(robot, useful, sample_time)
            self._update_pace_streak(robot, at_nominal_pace, sample_time)

        all_useful = self.robot_ids <= useful_robots
        if all_useful:
            if self.transport_first_all_useful_sim_time is None:
                self.transport_first_all_useful_sim_time = sample_time
                self.transport_first_all_useful_wall_time = wall_time
            self.transport_all_contribution_progress += positive_progress
        if all_useful:
            previous = self.last_all_useful_sample
            contiguous = (
                self.current_all_useful_start is not None
                and previous is not None
                and 0.0 <= sample_time - previous
                <= self.args.transport_control_gap_tolerance
            )
            if not contiguous:
                self.current_all_useful_start = sample_time
                self.current_all_useful_samples = 0
            self.current_all_useful_samples += 1
            self.last_all_useful_sample = sample_time
            self.transport_all_useful_samples += 1
            duration = max(
                0.0, sample_time - self.current_all_useful_start
            )
            self.max_all_useful_duration = max(
                self.max_all_useful_duration, duration
            )
            self.max_all_useful_samples = max(
                self.max_all_useful_samples,
                self.current_all_useful_samples,
            )
        else:
            self.current_all_useful_start = None
            self.current_all_useful_samples = 0
            self.last_all_useful_sample = None

        all_at_nominal_pace = self.robot_ids <= nominal_pace_robots
        if all_at_nominal_pace:
            previous = self.last_all_nominal_pace_sample
            contiguous = (
                self.current_all_nominal_pace_start is not None
                and previous is not None
                and 0.0 <= sample_time - previous
                <= self.args.transport_control_gap_tolerance
            )
            if not contiguous:
                self.current_all_nominal_pace_start = sample_time
                self.current_all_nominal_pace_samples = 0
            self.current_all_nominal_pace_samples += 1
            self.last_all_nominal_pace_sample = sample_time
            self.transport_all_nominal_pace_samples += 1
            duration = max(
                0.0, sample_time - self.current_all_nominal_pace_start
            )
            self.max_all_nominal_pace_duration = max(
                self.max_all_nominal_pace_duration, duration
            )
            self.max_all_nominal_pace_samples = max(
                self.max_all_nominal_pace_samples,
                self.current_all_nominal_pace_samples,
            )
        else:
            self.current_all_nominal_pace_start = None
            self.current_all_nominal_pace_samples = 0
            self.last_all_nominal_pace_sample = None

        if (
            reference_speed is not None
            and self.robot_ids <= reference_pace_robots
        ):
            self.transport_all_reference_pace_samples += 1

    def update(self, names, poses, twists, sim_time, wall_time, zones,
               arena_half_size, behavior_status, cmd_velocities):
        self.observe_behavior_status(behavior_status)
        model_pose = {
            name: (pose.position.x, pose.position.y,
                   yaw_from_quaternion(pose.orientation))
            for name, pose in zip(names, poses)
        }
        pose_by_name = dict(zip(names, poses))
        twist_by_name = dict(zip(names, twists))
        transport_roles = transport_role_metadata(behavior_status)
        declared_chain_pairs = {
            frozenset((robot, role["parent_namespace"]))
            for robot, role in transport_roles.items()
            if (
                role.get("role") == "companion_push"
                and role.get("parent_namespace") in self.robot_ids
            )
        }
        robot_positions = {}
        for robot in self.robot_ids:
            pose = pose_by_name.get(robot)
            if pose is not None:
                robot_positions[robot] = (pose.position.x, pose.position.y)

        if not robot_positions:
            return
        self.model_samples += 1
        self.wall_end = wall_time
        self.sim_end = sim_time

        dt = None
        if sim_time is not None and self.last_model_sim is not None:
            dt = sim_time - self.last_model_sim
        if sim_time is not None:
            self.last_model_sim = sim_time

        for robot, position in robot_positions.items():
            self.initial_positions.setdefault(robot, position)
            previous = self.last_positions.get(robot)
            if previous is not None:
                self.travel[robot] += math.hypot(
                    position[0] - previous[0], position[1] - previous[1]
                )
            self.last_positions[robot] = position
            self.final_positions[robot] = position

            twist = twist_by_name.get(robot)
            if twist is not None:
                velocity = (twist.linear.x, twist.linear.y)
                speed = math.hypot(*velocity)
                self.speeds.append(speed)
                old_velocity = self.last_velocity.get(robot)
                if old_velocity is not None and dt is not None and dt > 1e-4:
                    instantaneous = math.hypot(
                        velocity[0] - old_velocity[0],
                        velocity[1] - old_velocity[1],
                    ) / dt
                    self.instantaneous_accelerations.append(instantaneous)
                    if (
                        self.maximum_instantaneous_acceleration_event is None
                        or instantaneous
                        > self.maximum_instantaneous_acceleration_event[
                            "value_mps2"
                        ]
                    ):
                        self.maximum_instantaneous_acceleration_event = {
                            "robot": robot,
                            "value_mps2": instantaneous,
                            "dt_s": dt,
                            "previous_velocity_mps": list(old_velocity),
                            "velocity_mps": list(velocity),
                            "sim_time_s": sim_time,
                            "phase": behavior_status.get("phase"),
                        }
                self.last_velocity[robot] = velocity

                if sim_time is not None:
                    history = self.velocity_history[robot]
                    window = max(0.02, self.args.acceleration_window)
                    reference = next((
                        sample for sample in reversed(history)
                        if sim_time - sample[0] >= window
                    ), None)
                    if reference is not None:
                        reference_time, reference_velocity = reference
                        elapsed = sim_time - reference_time
                        acceleration = math.hypot(
                            velocity[0] - reference_velocity[0],
                            velocity[1] - reference_velocity[1],
                        ) / elapsed
                        self.accelerations.append(acceleration)
                        if (
                            self.maximum_acceleration_event is None
                            or acceleration
                            > self.maximum_acceleration_event["value_mps2"]
                        ):
                            self.maximum_acceleration_event = {
                                "robot": robot,
                                "value_mps2": acceleration,
                                "window_s": elapsed,
                                "previous_velocity_mps": list(
                                    reference_velocity
                                ),
                                "velocity_mps": list(velocity),
                                "sim_time_s": sim_time,
                                "phase": behavior_status.get("phase"),
                            }
                    history.append((sim_time, velocity))
                    history_limit = max(0.5, 4.0 * window)
                    while (
                        history
                        and sim_time - history[0][0] > history_limit
                    ):
                        history.popleft()

            boundary = arena_half_size - max(abs(position[0]), abs(position[1]))
            if self.min_boundary_clearance is None or boundary < self.min_boundary_clearance:
                self.min_boundary_clearance = boundary

            for zone in zones:
                # Contact with the movable payload is required during transport.
                if zone.get("model") == "transport_object":
                    continue
                clearance = point_clearance(
                    position[0], position[1], zone, model_pose
                )
                if (self.min_obstacle_clearance is None or
                        clearance < self.min_obstacle_clearance):
                    self.min_obstacle_clearance = clearance
                    self.min_obstacle_pair = [robot, zone.get("name")]

        ordered = sorted(robot_positions)
        for index, first in enumerate(ordered):
            for second in ordered[index + 1:]:
                distance = math.hypot(
                    robot_positions[first][0] - robot_positions[second][0],
                    robot_positions[first][1] - robot_positions[second][1],
                )
                if self.min_center_distance is None or distance < self.min_center_distance:
                    self.min_center_distance = distance
                    self.min_center_pair = [first, second]
                pair = frozenset((first, second))
                if pair in declared_chain_pairs:
                    if (
                        self.min_declared_chain_distance is None
                        or distance < self.min_declared_chain_distance
                    ):
                        self.min_declared_chain_distance = distance
                        self.min_declared_chain_pair = [first, second]
                        self.min_declared_chain_event = {
                            "pair": [first, second],
                            "distance_m": distance,
                            "sim_time_s": sim_time,
                            "phase": behavior_status.get("phase"),
                        }
                elif (
                    self.min_unexpected_center_distance is None
                    or distance < self.min_unexpected_center_distance
                ):
                    self.min_unexpected_center_distance = distance
                    self.min_unexpected_center_pair = [first, second]
                    self.min_unexpected_center_event = {
                        "pair": [first, second],
                        "distance_m": distance,
                        "sim_time_s": sim_time,
                        "phase": behavior_status.get("phase"),
                    }

        object_pose = pose_by_name.get("transport_object")
        if object_pose is not None:
            position = (object_pose.position.x, object_pose.position.y)
            object_yaw = yaw_from_quaternion(object_pose.orientation)
            if self.object_initial is None:
                self.object_initial = position
                self.object_initial_yaw = object_yaw
            if self.object_last is not None:
                self.object_travel += math.hypot(
                    position[0] - self.object_last[0],
                    position[1] - self.object_last[1],
                )
            self.object_last = position
            self.object_final = position
            self.object_final_yaw = object_yaw
            if self.case["behavior"] == "transport":
                target_x, target_y = self.case["target"]
                distance = math.hypot(
                    position[0] - target_x, position[1] - target_y
                )
                if self.object_goal_initial_distance is None:
                    self.object_goal_initial_distance = distance
                self.object_goal_final_distance = distance
                if (
                    self.object_goal_best_distance is None
                    or distance < self.object_goal_best_distance
                ):
                    self.object_goal_best_distance = distance

        if self.case["behavior"] == "transport":
            self._transport_discovery_motion(
                robot_positions, twist_by_name, behavior_status,
                sim_time, wall_time,
            )
            self._transport_participation(
                robot_positions, model_pose, twist_by_name, behavior_status,
                cmd_velocities, sim_time, wall_time, zones,
            )

    def finish(self, sim_time, wall_time):
        self.wall_end = wall_time
        self.sim_end = sim_time

    def report(self):
        wall_duration = max(0.0, self.wall_end - self.wall_start)
        simulated_duration = None
        if self.sim_start is not None and self.sim_end is not None:
            simulated_duration = max(0.0, self.sim_end - self.sim_start)
        rtf = (simulated_duration / wall_duration
               if simulated_duration is not None and wall_duration > 0.0
               else None)

        displacements = {}
        for robot, initial in self.initial_positions.items():
            final = self.final_positions.get(robot, initial)
            displacements[robot] = math.hypot(
                final[0] - initial[0], final[1] - initial[1]
            )

        formation_target_errors = {}
        for robot, target in sorted(
            self.formation_targets.items(), key=lambda item: robot_sort_key(item[0])
        ):
            position = self.final_positions.get(robot)
            if position is None:
                continue
            formation_target_errors[robot] = math.hypot(
                position[0] - target[0], position[1] - target[1]
            )
        missing_formation_targets = sorted(
            self.robot_ids - set(formation_target_errors), key=robot_sort_key
        )

        follow_chain = list(self.follow_chain)
        if not follow_chain:
            follow_chain = sorted(self.robot_ids, key=robot_sort_key)
        follow_spacing = {}
        follow_spacing_errors = []
        for first, second in zip(follow_chain, follow_chain[1:]):
            first_position = self.final_positions.get(first)
            second_position = self.final_positions.get(second)
            if first_position is None or second_position is None:
                continue
            distance = math.hypot(
                first_position[0] - second_position[0],
                first_position[1] - second_position[1],
            )
            pair_name = "{}->{}".format(first, second)
            follow_spacing[pair_name] = rounded(distance)
            if self.follow_distance is not None:
                follow_spacing_errors.append(abs(distance - self.follow_distance))
        object_displacement = None
        if self.object_initial is not None and self.object_final is not None:
            object_displacement = math.hypot(
                self.object_final[0] - self.object_initial[0],
                self.object_final[1] - self.object_initial[1],
            )
        object_goal_progress = None
        object_goal_efficiency = None
        if (
            self.object_goal_initial_distance is not None
            and self.object_goal_final_distance is not None
        ):
            object_goal_progress = (
                self.object_goal_initial_distance
                - self.object_goal_final_distance
            )
            if self.object_travel > 1e-6:
                object_goal_efficiency = object_goal_progress / self.object_travel
        push_goal_progress = None
        push_goal_efficiency = None
        if (
            self.transport_push_goal_initial_distance is not None
            and self.transport_push_goal_final_distance is not None
        ):
            push_goal_progress = (
                self.transport_push_goal_initial_distance
                - self.transport_push_goal_final_distance
            )
            if self.transport_push_object_travel > 1e-6:
                push_goal_efficiency = (
                    push_goal_progress
                    / self.transport_push_object_travel
                )
        pre_active_displacement = None
        if (
            self.object_initial is not None
            and self.transport_push_object_initial is not None
        ):
            pre_active_displacement = math.hypot(
                self.transport_push_object_initial[0]
                - self.object_initial[0],
                self.transport_push_object_initial[1]
                - self.object_initial[1],
            )

        participation = {}
        for robot in sorted(self.robot_ids):
            counts = self.transport_counts[robot]
            denominator = self.transport_push_samples
            positive_progress = self.transport_positive_goal_progress
            role_counts = self.transport_role_counts[robot]
            role = None
            if role_counts:
                role = max(
                    sorted(role_counts), key=lambda name: role_counts[name]
                )
            participation[robot] = {
                "role": role,
                "declared_parent_namespaces": sorted(
                    self.transport_parents[robot]
                ),
                "push_phase_samples": denominator,
                "declared_samples": counts["declared"],
                "direct_contact_samples": counts["direct_contact"],
                "companion_contact_samples": counts["companion_contact"],
                "connected_samples": counts["connected"],
                "push_intent_samples": counts["push_intent"],
                "useful_pushing_samples": counts["useful_pushing"],
                "nominal_pace_intent_samples": (
                    counts["nominal_pace_intent"]
                ),
                "nominal_pace_pushing_samples": (
                    counts["nominal_pace_pushing"]
                ),
                "reference_pace_intent_samples": (
                    counts["reference_pace_intent"]
                ),
                "contributed_positive_goal_progress_m": rounded(
                    self.transport_contribution_progress[robot]
                ),
                "command_goal_speed_mps": summary(
                    self.transport_command_goal_speeds[robot]
                ),
                "role_reference_goal_speed_mps": summary(
                    self.transport_role_reference_goal_speeds[robot]
                ),
                "adaptive_contribution_threshold_mps": summary(
                    self.transport_adaptive_intent_thresholds[robot]
                ),
                "declared_fraction": rounded(
                    counts["declared"] / denominator
                    if denominator else 0.0
                ),
                "direct_contact_fraction": rounded(
                    counts["direct_contact"] / denominator
                    if denominator else 0.0
                ),
                "companion_contact_fraction": rounded(
                    counts["companion_contact"] / denominator
                    if denominator else 0.0
                ),
                "connected_fraction": rounded(
                    counts["connected"] / denominator
                    if denominator else 0.0
                ),
                "push_intent_fraction": rounded(
                    counts["push_intent"] / denominator
                    if denominator else 0.0
                ),
                "useful_pushing_batch_fraction": rounded(
                    counts["useful_pushing"] / denominator
                    if denominator else 0.0
                ),
                "useful_pushing_fraction": rounded(
                    self.transport_contribution_progress[robot]
                    / positive_progress
                    if positive_progress > 1e-9 else 0.0
                ),
                "nominal_pace_pushing_fraction": rounded(
                    counts["nominal_pace_pushing"] / denominator
                    if denominator else 0.0
                ),
                "reference_pace_intent_fraction": rounded(
                    counts["reference_pace_intent"]
                    / self.transport_reference_pace_samples
                    if self.transport_reference_pace_samples else 0.0
                ),
                "nominal_pace_warning": (
                    self.max_pace_streak_duration[robot]
                    < self.args.min_transport_push_duration
                    or self.max_pace_streak_samples[robot]
                    < self.args.min_transport_push_samples
                    or (
                        counts["nominal_pace_pushing"] / denominator
                        if denominator else 0.0
                    ) < self.args.min_transport_useful_fraction
                ),
                "maximum_continuous_useful_pushing_s": rounded(
                    self.max_push_streak_duration[robot]
                ),
                "maximum_continuous_useful_pushing_samples": (
                    self.max_push_streak_samples[robot]
                ),
                "maximum_continuous_nominal_pace_s": rounded(
                    self.max_pace_streak_duration[robot]
                ),
                "maximum_continuous_nominal_pace_samples": (
                    self.max_pace_streak_samples[robot]
                ),
            }

        discovery = self.transport_discovery_notice or {}
        finder = clean_robot_name(discovery.get("finder"))
        raw_notified = discovery.get("notified_robots", [])
        notified = set()
        if isinstance(raw_notified, (list, tuple, set)):
            notified = {
                name for name in (
                    clean_robot_name(item) for item in raw_notified
                )
                if name in self.robot_ids
            }
        notice_recipients = set(notified)
        if finder in self.robot_ids:
            notice_recipients.add(finder)

        announcement_sim = finite_number(discovery.get("sim_time"))
        if announcement_sim is None:
            announcement_sim = self.transport_discovery_observed_sim
        discovery_robots = {}
        below_required_travel = []
        notified_below_required_travel = []
        for robot in sorted(self.robot_ids, key=robot_sort_key):
            role = self.transport_discovery_roles.get(robot, {}).get("role")
            is_root = role == "payload_push" or robot == finder
            required_travel = (
                self.args.min_transport_root_rendezvous_travel
                if is_root
                else self.args.min_transport_rendezvous_travel
            )
            travel = self.transport_discovery_travel.get(robot, 0.0)
            initial = self.transport_discovery_start_positions.get(robot)
            final = self.transport_discovery_final_positions.get(robot)
            displacement = None
            if initial is not None and final is not None:
                displacement = math.hypot(
                    final[0] - initial[0], final[1] - initial[1]
                )
            first_motion_sim = self.transport_discovery_first_motion_sim.get(
                robot
            )
            motion_latency = None
            if first_motion_sim is not None and announcement_sim is not None:
                motion_latency = max(0.0, first_motion_sim - announcement_sim)
            elif robot in self.transport_discovery_first_motion_wall:
                motion_latency = max(
                    0.0,
                    self.transport_discovery_first_motion_wall[robot]
                    - self.transport_discovery_observed_wall,
                )
            met_requirement = travel >= required_travel
            if not met_requirement:
                below_required_travel.append(robot)
                if robot in notified:
                    notified_below_required_travel.append(robot)
            discovery_robots[robot] = {
                "role": role,
                "finder": robot == finder,
                "notified": robot in notified,
                "notice_recipient": robot in notice_recipients,
                "status_acknowledged": (
                    robot in self.transport_discovery_acknowledged
                ),
                "motion_detected": (
                    robot in self.transport_discovery_first_motion_wall
                ),
                "discovery_to_motion_latency_s": rounded(motion_latency),
                "pre_push_path_length_m": rounded(travel),
                "pre_push_displacement_m": rounded(displacement),
                "required_pre_push_path_length_m": rounded(required_travel),
                "met_pre_push_path_requirement": met_requirement,
            }

        missing_notice_recipients = sorted(
            self.robot_ids - notice_recipients, key=robot_sort_key
        )
        missing_acknowledgements = []
        if self.transport_discovery_ack_available:
            missing_acknowledgements = sorted(
                notice_recipients - self.transport_discovery_acknowledged,
                key=robot_sort_key,
            )
        minimum_search_travel = max(
            0.0, self.args.min_transport_search_travel
        )
        search_travel = {
            robot: self.transport_search_travel.get(robot, 0.0)
            for robot in sorted(self.robot_ids, key=robot_sort_key)
        }
        below_required_search_travel = [
            robot for robot, travel in search_travel.items()
            if travel < minimum_search_travel
        ]
        discovery_response = {
            "notice_observed": bool(self.transport_discovery_notice),
            "event_id": discovery.get("event_id"),
            "announced_sim_time_s": rounded(announcement_sim),
            "observed_sim_time_s": rounded(
                self.transport_discovery_observed_sim
            ),
            "finder": finder,
            "notified_robots": sorted(notified, key=robot_sort_key),
            "notice_recipients": sorted(
                notice_recipients, key=robot_sort_key
            ),
            "missing_notice_recipients": missing_notice_recipients,
            "status_acknowledgement_available": (
                self.transport_discovery_ack_available
            ),
            "status_acknowledged_robots": sorted(
                self.transport_discovery_acknowledged, key=robot_sort_key
            ),
            "missing_status_acknowledgements": missing_acknowledgements,
            "motion_detection_distance_m": rounded(
                self.args.transport_motion_detection_distance
            ),
            "moving_speed_threshold_mps": rounded(
                self.args.transport_rendezvous_moving_speed
            ),
            "search_motion_samples": self.transport_search_motion_samples,
            "rendezvous_motion_samples": (
                self.transport_rendezvous_motion_samples
            ),
            "simultaneous_motion_window_supported": (
                self.transport_search_motion_samples >= 3
            ),
            "simultaneous_search_motion_window_supported": (
                self.transport_search_motion_samples >= 3
            ),
            "search_motion_window_supported": (
                self.transport_search_motion_samples >= 3
            ),
            "robots_observed_moving_during_search": sorted(
                self.transport_search_movers, key=robot_sort_key
            ),
            "robots_not_observed_moving_during_search": sorted(
                self.robot_ids - self.transport_search_movers,
                key=robot_sort_key,
            ),
            "minimum_search_path_length_m": rounded(
                minimum_search_travel
            ),
            "search_path_length_m": {
                robot: rounded(travel)
                for robot, travel in search_travel.items()
            },
            "robots_below_required_search_travel": (
                below_required_search_travel
            ),
            "minimum_rendezvous_path_length_m": rounded(
                self.args.min_transport_rendezvous_travel
            ),
            "minimum_payload_root_path_length_m": rounded(
                self.args.min_transport_root_rendezvous_travel
            ),
            "peak_simultaneous_movers": len(
                self.transport_peak_simultaneous_movers
            ),
            "peak_simultaneous_search_movers": len(
                self.transport_peak_simultaneous_movers
            ),
            "peak_simultaneous_mover_names": sorted(
                self.transport_peak_simultaneous_movers,
                key=robot_sort_key,
            ),
            "peak_simultaneous_mover_fraction": rounded(
                len(self.transport_peak_simultaneous_movers)
                / len(self.robot_ids)
                if self.robot_ids else 0.0
            ),
            "peak_simultaneous_rendezvous_movers": len(
                self.transport_peak_rendezvous_movers
            ),
            "peak_simultaneous_rendezvous_mover_names": sorted(
                self.transport_peak_rendezvous_movers,
                key=robot_sort_key,
            ),
            "robots_below_required_travel": below_required_travel,
            "notified_robots_below_required_travel": (
                notified_below_required_travel
            ),
            "robots": discovery_robots,
        }

        return {
            "wall_duration_s": rounded(wall_duration),
            "simulated_duration_s": rounded(simulated_duration),
            "real_time_factor": rounded(rtf),
            "model_samples": self.model_samples,
            "correlated_behavior_samples": self.correlated_behavior_samples,
            "minimum_robot_center_distance_m": rounded(self.min_center_distance),
            "minimum_robot_center_pair": self.min_center_pair,
            "minimum_unexpected_robot_center_distance_m": rounded(
                self.min_unexpected_center_distance
            ),
            "minimum_unexpected_robot_center_pair": self.min_unexpected_center_pair,
            "minimum_unexpected_robot_center_event": (
                {
                    key: rounded(value) if isinstance(value, float) else value
                    for key, value in self.min_unexpected_center_event.items()
                }
                if self.min_unexpected_center_event else None
            ),
            "minimum_declared_chain_center_distance_m": rounded(
                self.min_declared_chain_distance
            ),
            "minimum_declared_chain_center_pair": self.min_declared_chain_pair,
            "minimum_declared_chain_center_event": (
                {
                    key: rounded(value) if isinstance(value, float) else value
                    for key, value in self.min_declared_chain_event.items()
                }
                if self.min_declared_chain_event else None
            ),
            "minimum_static_obstacle_clearance_m": rounded(self.min_obstacle_clearance),
            "minimum_static_obstacle_pair": self.min_obstacle_pair,
            "minimum_boundary_clearance_m": rounded(self.min_boundary_clearance),
            "robot_travel_m": {key: rounded(value) for key, value in sorted(self.travel.items())},
            "robot_displacement_m": {key: rounded(value) for key, value in sorted(displacements.items())},
            "formation_final_target_error_m": {
                key: rounded(value)
                for key, value in sorted(
                    formation_target_errors.items(),
                    key=lambda item: robot_sort_key(item[0]),
                )
            },
            "formation_final_target_error_summary_m": summary(
                list(formation_target_errors.values())
            ),
            "formation_missing_final_targets": missing_formation_targets,
            "follow_chain": follow_chain,
            "follow_target_spacing_m": rounded(self.follow_distance),
            "follow_final_adjacent_spacing_m": follow_spacing,
            "follow_final_adjacent_spacing_error_summary_m": summary(
                follow_spacing_errors
            ),
            "linear_speed_mps": summary(self.speeds),
            "linear_acceleration_mps2": summary(self.accelerations),
            "instantaneous_linear_acceleration_mps2": summary(
                self.instantaneous_accelerations
            ),
            "maximum_linear_acceleration_event": (
                {
                    key: (
                        [rounded(value) for value in raw]
                        if isinstance(raw, list)
                        else rounded(raw) if isinstance(raw, float) else raw
                    )
                    for key, raw in self.maximum_acceleration_event.items()
                }
                if self.maximum_acceleration_event else None
            ),
            "maximum_instantaneous_acceleration_event": (
                {
                    key: (
                        [rounded(value) for value in raw]
                        if isinstance(raw, list)
                        else rounded(raw) if isinstance(raw, float) else raw
                    )
                    for key, raw in (
                        self.maximum_instantaneous_acceleration_event.items()
                    )
                }
                if self.maximum_instantaneous_acceleration_event else None
            ),
            "object_initial_xy": ([rounded(v) for v in self.object_initial]
                                  if self.object_initial else None),
            "object_final_xy": ([rounded(v) for v in self.object_final]
                                if self.object_final else None),
            "object_initial_yaw_rad": rounded(self.object_initial_yaw),
            "object_final_yaw_rad": rounded(self.object_final_yaw),
            "object_displacement_m": rounded(object_displacement),
            "object_path_length_m": rounded(self.object_travel),
            "object_initial_goal_distance_m": rounded(
                self.object_goal_initial_distance
            ),
            "object_final_goal_distance_m": rounded(
                self.object_goal_final_distance
            ),
            "object_best_goal_distance_m": rounded(
                self.object_goal_best_distance
            ),
            "object_goal_progress_m": rounded(object_goal_progress),
            "object_goal_progress_efficiency": rounded(
                object_goal_efficiency
            ),
            "transport_push_phase_samples": self.transport_push_samples,
            "transport_distinct_control_batches": self.transport_push_samples,
            "transport_last_control_sequence": (
                self.transport_last_control_sequence
            ),
            "transport_last_control_sim_time_s": rounded(
                self.transport_last_control_sim_time
            ),
            "transport_duplicate_control_status_samples": (
                self.transport_duplicate_control_sequences
            ),
            "transport_active_push_latched": (
                self.transport_active_push_latched
            ),
            "transport_grf_samples": self.transport_grf_samples,
            "transport_first_connection_loss": (
                rounded_json(self.first_transport_connection_loss)
                if self.first_transport_connection_loss else None
            ),
            "transport_pre_active_object_displacement_m": rounded(
                pre_active_displacement
            ),
            "transport_push_initial_goal_distance_m": rounded(
                self.transport_push_goal_initial_distance
            ),
            "transport_push_object_initial_xy": (
                [rounded(v) for v in self.transport_push_object_initial]
                if self.transport_push_object_initial else None
            ),
            "transport_push_object_final_xy": (
                [rounded(v) for v in self.transport_push_object_final]
                if self.transport_push_object_final else None
            ),
            "transport_push_object_path_length_m": rounded(
                self.transport_push_object_travel
            ),
            "transport_push_goal_progress_m": rounded(
                push_goal_progress
            ),
            "transport_push_goal_progress_efficiency": rounded(
                push_goal_efficiency
            ),
            "transport_positive_control_goal_progress_m": rounded(
                self.transport_positive_goal_progress
            ),
            "transport_all_useful_samples": (
                self.transport_all_useful_samples
            ),
            "transport_first_all_useful_sim_time_s": rounded(
                self.transport_first_all_useful_sim_time
            ),
            "transport_time_to_first_all_useful_batch_sim_s": rounded(
                max(
                    0.0,
                    self.transport_first_all_useful_sim_time - self.sim_start,
                )
                if self.transport_first_all_useful_sim_time is not None
                else None
            ),
            "transport_time_to_first_all_useful_batch_wall_s": rounded(
                max(
                    0.0,
                    self.transport_first_all_useful_wall_time
                    - self.wall_start,
                )
                if self.transport_first_all_useful_wall_time is not None
                else None
            ),
            "transport_all_useful_batch_fraction": rounded(
                self.transport_all_useful_samples
                / self.transport_push_samples
                if self.transport_push_samples else 0.0
            ),
            "transport_all_useful_fraction": rounded(
                self.transport_all_contribution_progress
                / self.transport_positive_goal_progress
                if self.transport_positive_goal_progress > 1e-9 else 0.0
            ),
            "transport_maximum_continuous_all_useful_s": rounded(
                self.max_all_useful_duration
            ),
            "transport_maximum_continuous_all_useful_samples": (
                self.max_all_useful_samples
            ),
            "transport_all_nominal_pace_samples": (
                self.transport_all_nominal_pace_samples
            ),
            "transport_all_nominal_pace_fraction": rounded(
                self.transport_all_nominal_pace_samples
                / self.transport_push_samples
                if self.transport_push_samples else 0.0
            ),
            "transport_all_nominal_pace_warning": (
                self.max_all_nominal_pace_duration
                < self.args.min_transport_push_duration
                or self.max_all_nominal_pace_samples
                < self.args.min_transport_push_samples
                or (
                    self.transport_all_nominal_pace_samples
                    / self.transport_push_samples
                    if self.transport_push_samples else 0.0
                ) < self.args.min_transport_useful_fraction
            ),
            "transport_maximum_continuous_all_nominal_pace_s": rounded(
                self.max_all_nominal_pace_duration
            ),
            "transport_maximum_continuous_all_nominal_pace_samples": (
                self.max_all_nominal_pace_samples
            ),
            "transport_reference_pace_samples": (
                self.transport_reference_pace_samples
            ),
            "transport_all_reference_pace_samples": (
                self.transport_all_reference_pace_samples
            ),
            "transport_all_reference_pace_fraction": rounded(
                self.transport_all_reference_pace_samples
                / self.transport_reference_pace_samples
                if self.transport_reference_pace_samples else 0.0
            ),
            "transport_nominal_pace_warning_robots": [
                robot for robot in sorted(
                    self.robot_ids, key=robot_sort_key
                )
                if (
                    self.max_pace_streak_duration[robot]
                    < self.args.min_transport_push_duration
                    or self.max_pace_streak_samples[robot]
                    < self.args.min_transport_push_samples
                    or (
                        self.transport_counts[robot][
                            "nominal_pace_pushing"
                        ] / self.transport_push_samples
                        if self.transport_push_samples else 0.0
                    ) < self.args.min_transport_useful_fraction
                )
            ],
            "transport_participation": participation,
            "transport_discovery_response": discovery_response,
        }


class AcceptanceHarness:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.metrics_update_lock = threading.Lock()
        self.command_pub = rospy.Publisher("/swarm/commands", String, queue_size=10)
        self.roster = []
        self.swarm_task = {}
        self.emergency_stop = False
        self.collision_count = 0
        self.behavior_status = {"formation": {}, "follow": {}, "transport": {}}
        self.behavior_status_sequences = {
            "formation": 0, "follow": 0, "transport": 0,
        }
        self.behavior_status_wall_times = {
            "formation": None, "follow": None, "transport": None,
        }
        self.sim_time = None
        self.model_names = set()
        self.cmd_velocities = {}
        self.cmd_vel_subscribers = {}
        self.cmd_vel_subscriber_tokens = {}
        self.next_cmd_vel_subscriber_token = 0
        self.robot_collision_active = {}
        self.collision_event_stream_seen = False
        self.collision_event_last_sequence = 0
        self.active_collision_robot_ids = set()
        self.active_collision_episode_counts = {}
        self.active_collision_episode_records = []
        self.active_collision_status_samples = 0
        self.active_collision_missing_robots = set()
        self.active_collision_protocol_errors = []
        self.active_collision_event_fingerprints = {}
        self.active_collision_duplicate_count = 0
        self.active_collision_baseline_sequence = 0
        self.active_collision_next_sequence = 1
        self.active_collision_expected_task_type = None
        self.active_collision_source_id = None
        self.active_collision_source_last_sequence = None
        self.active_collision_source_last_control_sequence = 0
        self.last_reset_cleanup = {}
        self.active_metrics = None
        self.active_task_id = None
        self.last_metrics_sim = None
        self.active_case_wall_start = None
        self.active_case_sim_start = None
        self.active_transport_phase = None
        self.active_transport_phase_started_wall = None
        self.active_transport_phase_error = None
        self.active_transport_phase_timeline = []
        self.active_transport_phase_timeline_dropped = 0
        self.stop_requested = False
        self.case_cleanup_failures = []
        self.last_cleanup_failures = []

        self.arena_half_size = float(
            rospy.get_param("/fleet_manager/arena_size", 10.0)
        ) / 2.0
        profile = rospy.get_param("/fleet_manager/arena_profile", "swarm_arena")
        raw_zones = rospy.get_param("/fleet_manager/spawn_exclusion_zones", [])
        self.zones = []
        for zone in raw_zones:
            worlds = zone.get("worlds", []) if isinstance(zone, dict) else []
            if isinstance(worlds, str):
                worlds = [worlds]
            if isinstance(zone, dict) and zone.get("shape") in {"box", "circle"}:
                if not worlds or profile in worlds:
                    self.zones.append(zone)

        rospy.Subscriber("/fleet/robot_list", String, self._roster_cb, queue_size=1)
        rospy.Subscriber("/swarm/status", String, self._swarm_cb, queue_size=1)
        rospy.Subscriber("/formation/status", String,
                         lambda msg: self._behavior_cb("formation", msg), queue_size=1)
        rospy.Subscriber("/follow_leader/status", String,
                         lambda msg: self._behavior_cb("follow", msg), queue_size=1)
        rospy.Subscriber("/transport/status", String,
                         lambda msg: self._behavior_cb("transport", msg), queue_size=1)
        rospy.Subscriber("/clock", Clock, self._clock_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._models_cb, queue_size=1)

    def _roster_cb(self, msg):
        roster = []
        seen = set()
        for item in msg.data.split(","):
            namespace = clean_robot_name(item)
            if namespace is not None and namespace not in seen:
                roster.append(namespace)
                seen.add(namespace)

        departed_subscribers = []
        with self.lock:
            self.roster = roster
            live_names = set(roster)

            for namespace in set(self.cmd_vel_subscribers) - live_names:
                subscriber = self.cmd_vel_subscribers.pop(namespace, None)
                if subscriber is not None:
                    departed_subscribers.append(subscriber)
                self.cmd_vel_subscriber_tokens.pop(namespace, None)

            self.cmd_velocities = {
                namespace: command
                for namespace, command in self.cmd_velocities.items()
                if namespace in live_names
            }

            for namespace in self.roster:
                if namespace in self.cmd_vel_subscribers:
                    continue
                self.next_cmd_vel_subscriber_token += 1
                token = self.next_cmd_vel_subscriber_token
                self.cmd_vel_subscriber_tokens[namespace] = token
                subscriber = rospy.Subscriber(
                    "/{}/cmd_vel".format(namespace), Twist,
                    lambda command, robot=namespace, generation=token: self._cmd_vel_cb(
                        robot, command, generation
                    ),
                    queue_size=1,
                )
                self.cmd_vel_subscribers[namespace] = subscriber

        # XML-RPC unregistration can block briefly.  Do it after releasing the
        # callback-state lock so high-rate telemetry remains responsive.
        for subscriber in departed_subscribers:
            subscriber.unregister()

    def _cmd_vel_cb(self, namespace, msg, token=None):
        with self.lock:
            if namespace not in self.roster:
                return
            if (
                token is not None
                and self.cmd_vel_subscriber_tokens.get(namespace) != token
            ):
                return
            self.cmd_velocities[namespace] = {
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z),
                "wall_time": time.monotonic(),
            }

    def _swarm_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return

        collision_states = {}
        robots = status.get("robots", [])
        if isinstance(robots, list):
            for item in robots:
                if not isinstance(item, dict):
                    continue
                name = clean_robot_name(item.get("id"))
                if name is not None and "collision" in item:
                    collision_states[name] = bool(item.get("collision"))

        with self.lock:
            self.swarm_task = status.get("task", {})
            self.emergency_stop = bool(status.get("emergency_stop", False))
            self.collision_count = int(status.get("collisions", 0) or 0)
            self.robot_collision_active = collision_states
            self._observe_collision_events(
                status.get("collision_events"), self.collision_count
            )

    def _begin_collision_attribution(self, robot_ids, task_type=None):
        """Baseline the sealed event stream for one acceptance case."""
        self.active_collision_robot_ids = set(robot_ids)
        self.active_collision_episode_counts = {
            name: 0 for name in robot_ids
        }
        self.active_collision_episode_records = []
        self.active_collision_status_samples = 0
        self.active_collision_missing_robots = set()
        self.active_collision_protocol_errors = []
        self.active_collision_event_fingerprints = {}
        self.active_collision_duplicate_count = 0
        self.active_collision_baseline_sequence = max(
            int(getattr(self, "collision_event_last_sequence", 0) or 0),
            int(self.collision_count or 0),
        )
        self.active_collision_next_sequence = (
            self.active_collision_baseline_sequence + 1
        )
        normalized_type = {
            "follow": "follow_leader",
        }.get(task_type, task_type)
        self.active_collision_expected_task_type = (
            normalized_type if normalized_type in COLLISION_TASK_TYPES else None
        )
        self.active_collision_source_id = None
        self.active_collision_source_last_sequence = None
        self.active_collision_source_last_control_sequence = 0

    def _collision_protocol_error(self, reason):
        """Retain a small, de-duplicated set of fail-closed diagnostics."""
        if reason not in self.active_collision_protocol_errors:
            if len(self.active_collision_protocol_errors) < 32:
                self.active_collision_protocol_errors.append(reason)

    @staticmethod
    def _collision_sequence(value):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
        return value

    def _observe_collision_events(self, stream, aggregate_count):
        """Consume the producer's bounded, sealed rising-edge stream."""
        active = bool(self.active_collision_robot_ids)
        baseline = getattr(self, "active_collision_baseline_sequence", 0)
        if not isinstance(stream, dict):
            if active and aggregate_count > baseline:
                self._collision_protocol_error(
                    "collision event stream missing after counter increase"
                )
            return

        version = self._collision_sequence(stream.get("version"))
        first = self._collision_sequence(stream.get("first_sequence"))
        last = self._collision_sequence(stream.get("last_sequence"))
        events = stream.get("events")
        malformed = (
            version != COLLISION_EVENT_STREAM_VERSION
            or first is None
            or last is None
            or not isinstance(events, list)
            or len(events) > MAXIMUM_COLLISION_EVENT_HISTORY
            or last != aggregate_count
        )
        if malformed:
            if active:
                self._collision_protocol_error(
                    "collision event stream metadata is malformed or mismatched"
                )
            return

        parsed = []
        previous_sequence = None
        for item in events:
            if not isinstance(item, dict):
                malformed = True
                break
            sequence = self._collision_sequence(item.get("sequence"))
            if sequence is None or sequence == 0:
                malformed = True
                break
            if previous_sequence is not None and sequence != previous_sequence + 1:
                malformed = True
                break
            previous_sequence = sequence
            parsed.append((sequence, item))

        expected_first = parsed[0][0] if parsed else last + 1
        if (
            malformed
            or first != expected_first
            or (parsed and parsed[-1][0] != last)
        ):
            if active:
                self._collision_protocol_error(
                    "collision event stream has a sequence discontinuity"
                )
            return

        previous_last = int(
            getattr(self, "collision_event_last_sequence", 0) or 0
        )
        if (
            getattr(self, "collision_event_stream_seen", False)
            and last < previous_last
        ):
            if active:
                self._collision_protocol_error(
                    "collision event stream sequence regressed"
                )
            return
        self.collision_event_stream_seen = True
        self.collision_event_last_sequence = last
        if not active:
            return

        self.active_collision_status_samples += 1
        next_sequence = self.active_collision_next_sequence
        if first > next_sequence and last >= next_sequence:
            self._collision_protocol_error(
                "collision event stream dropped sequence {}".format(
                    next_sequence
                )
            )

        for sequence, item in parsed:
            if sequence <= baseline:
                continue
            task_id = item.get("task_id")
            task_type = item.get("task_type")
            name = clean_robot_name(item.get("robot_id"))
            phase = str(item.get("task_phase") or "").strip().upper()
            source_id = None
            source_sequence = None
            source_control_sequence = None
            source_sim_time = None
            source_wall_time = None
            source_valid = True
            if task_type == "transport":
                source_id = item.get("source_id")
                source_sequence = self._collision_sequence(
                    item.get("source_sequence")
                )
                source_control_sequence = self._collision_sequence(
                    item.get("source_control_sequence")
                )
                raw_source_sim_time = item.get("source_sim_time")
                raw_source_wall_time = item.get("source_wall_time")
                source_sim_time = (
                    None if isinstance(raw_source_sim_time, bool)
                    else finite_number(raw_source_sim_time)
                )
                source_wall_time = (
                    None if isinstance(raw_source_wall_time, bool)
                    else finite_number(raw_source_wall_time)
                )
                source_valid = (
                    isinstance(source_id, str)
                    and bool(source_id)
                    and len(source_id) <= 128
                    and source_id.isprintable()
                    and source_sequence not in (None, 0)
                    and source_control_sequence not in (None, 0)
                    and source_sim_time is not None
                    and source_sim_time >= 0.0
                    and source_wall_time is not None
                    and source_wall_time >= 0.0
                )
            fingerprint = (
                item.get("robot_id"), task_id, task_type,
                item.get("task_phase"), source_id, source_sequence,
                source_control_sequence, source_sim_time, source_wall_time,
            )
            if sequence < self.active_collision_next_sequence:
                previous = self.active_collision_event_fingerprints.get(
                    sequence
                )
                if previous == fingerprint:
                    self.active_collision_duplicate_count += 1
                else:
                    self._collision_protocol_error(
                        "collision sequence {} was reused with different data".format(
                            sequence
                        )
                    )
                continue
            if sequence > self.active_collision_next_sequence:
                self._collision_protocol_error(
                    "collision event stream skipped sequence {}".format(
                        self.active_collision_next_sequence
                    )
                )
            self.active_collision_next_sequence = sequence + 1
            if (
                len(self.active_collision_event_fingerprints)
                >= MAXIMUM_COLLISION_EVENT_HISTORY
                or len(self.active_collision_episode_records)
                >= MAXIMUM_COLLISION_EVENT_HISTORY
            ):
                self._collision_protocol_error(
                    "collision attribution consumer capacity exceeded"
                )
                continue
            self.active_collision_event_fingerprints[sequence] = fingerprint

            if task_id != str(self.active_task_id or ""):
                self._collision_protocol_error(
                    "collision event {} belongs to another task".format(sequence)
                )
                continue
            expected_type = self.active_collision_expected_task_type
            if expected_type is not None and task_type != expected_type:
                self._collision_protocol_error(
                    "collision event {} has a mismatched task type".format(sequence)
                )
                continue
            if name not in self.active_collision_robot_ids:
                self._collision_protocol_error(
                    "collision event {} names a robot outside the case".format(
                        sequence
                    )
                )
                continue
            if task_type == "transport" and phase not in TRANSPORT_PHASES:
                self._collision_protocol_error(
                    "collision event {} lacks an authoritative transport phase".format(
                        sequence
                    )
                )
                phase = "UNOBSERVED"
            elif task_type != "transport":
                phase = "UNOBSERVED"
            if task_type == "transport":
                if not source_valid:
                    self._collision_protocol_error(
                        "collision event {} lacks valid causal source data".format(
                            sequence
                        )
                    )
                    continue
                active_source_id = getattr(
                    self, "active_collision_source_id", None
                )
                last_source_sequence = getattr(
                    self, "active_collision_source_last_sequence", None
                )
                last_control_sequence = int(getattr(
                    self,
                    "active_collision_source_last_control_sequence",
                    0,
                ) or 0)
                if (
                    active_source_id is not None
                    and source_id != active_source_id
                ):
                    self._collision_protocol_error(
                        "transport collision source restarted during the case"
                    )
                    continue
                if (
                    last_source_sequence is not None
                    and source_sequence != last_source_sequence + 1
                ):
                    self._collision_protocol_error(
                        "transport collision source sequence is discontinuous"
                    )
                    continue
                if source_control_sequence < last_control_sequence:
                    self._collision_protocol_error(
                        "transport collision control sequence regressed"
                    )
                    continue
                self.active_collision_source_id = source_id
                self.active_collision_source_last_sequence = source_sequence
                self.active_collision_source_last_control_sequence = (
                    source_control_sequence
                )

            self.active_collision_episode_counts[name] += 1
            observed_wall_elapsed = (
                max(0.0, time.monotonic() - self.active_case_wall_start)
                if self.active_case_wall_start is not None else None
            )
            observed_sim_elapsed = (
                max(0.0, self.sim_time - self.active_case_sim_start)
                if (
                    self.sim_time is not None
                    and self.active_case_sim_start is not None
                ) else None
            )
            wall_elapsed = observed_wall_elapsed
            sim_elapsed = observed_sim_elapsed
            record = {
                "sequence": sequence,
                "robot_id": name,
                "task_id": task_id,
                "task_type": task_type,
                "phase": phase,
                "docking_window_open": (
                    phase in TRANSPORT_DOCKING_PHASES
                ),
                "wall_elapsed_s": wall_elapsed,
                "sim_elapsed_s": sim_elapsed,
                "observed_wall_elapsed_s": observed_wall_elapsed,
                "observed_sim_elapsed_s": observed_sim_elapsed,
            }
            if task_type == "transport":
                source_wall_elapsed = (
                    max(0.0, source_wall_time - self.active_case_wall_start)
                    if self.active_case_wall_start is not None else None
                )
                source_sim_elapsed = (
                    max(0.0, source_sim_time - self.active_case_sim_start)
                    if self.active_case_sim_start is not None else None
                )
                record.update({
                    "source_id": source_id,
                    "source_sequence": source_sequence,
                    "source_control_sequence": source_control_sequence,
                    "source_sim_time": source_sim_time,
                    "source_wall_time": source_wall_time,
                    "source_wall_elapsed_s": source_wall_elapsed,
                    "source_sim_elapsed_s": source_sim_elapsed,
                    # Temporal classification uses the edge time, not the
                    # later /swarm/status subscriber callback.
                    "wall_elapsed_s": source_wall_elapsed,
                    "sim_elapsed_s": source_sim_elapsed,
                })
            self.active_collision_episode_records.append(record)

    def _finish_collision_attribution(self, collision_delta):
        expected_last = (
            self.active_collision_baseline_sequence
            + max(0, int(collision_delta or 0))
        )
        observed_last = self.active_collision_next_sequence - 1
        if observed_last != expected_last:
            self._collision_protocol_error(
                "sealed event watermark did not match the aggregate delta"
            )
        report = collision_attribution_report(
            collision_delta,
            self.active_collision_episode_counts,
            self.active_collision_status_samples,
            self.active_collision_missing_robots,
            self.active_collision_episode_records,
            self.active_collision_protocol_errors,
            self.active_collision_duplicate_count,
            self.active_collision_baseline_sequence,
            observed_last,
        )
        self.active_collision_robot_ids = set()
        self.active_collision_episode_counts = {}
        self.active_collision_episode_records = []
        self.active_collision_status_samples = 0
        self.active_collision_missing_robots = set()
        self.active_collision_protocol_errors = []
        self.active_collision_event_fingerprints = {}
        self.active_collision_duplicate_count = 0
        self.active_collision_expected_task_type = None
        self.active_collision_source_id = None
        self.active_collision_source_last_sequence = None
        self.active_collision_source_last_control_sequence = 0
        return report

    def _behavior_cb(self, name, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(status, dict):
            return
        observed_wall = time.monotonic()
        with self.lock:
            self.behavior_status[name] = status
            self.behavior_status_sequences[name] += 1
            self.behavior_status_wall_times[name] = observed_wall
            if name == "transport":
                self._record_transport_phase(status, observed_wall)

    def _clock_cb(self, msg):
        with self.lock:
            self.sim_time = msg.clock.to_sec()

    def _models_cb(self, msg):
        wall_now = time.monotonic()
        with self.lock:
            self.model_names = set(msg.name)
            metrics = self.active_metrics
            if metrics is None:
                return
            sim_time = self.sim_time
            if (
                sim_time is not None
                and self.last_metrics_sim is not None
                and sim_time - self.last_metrics_sim < 0.02
            ):
                return
            self.last_metrics_sim = sim_time
            behavior_name = metrics.case["behavior"]
            behavior = dict(self.behavior_status[behavior_name])
            if str(behavior.get("task_id") or "") != str(
                self.active_task_id or ""
            ):
                behavior = {}
            commands = {
                name: dict(command)
                for name, command in self.cmd_velocities.items()
            }

        # ModelStates can arrive hundreds of times per simulated second.
        # Keep the shared callback lock free while calculating geometry so
        # transport status and cmd_vel callbacks cannot be starved.
        with self.metrics_update_lock:
            metrics.update(
                msg.name, msg.pose, msg.twist, sim_time, wall_now,
                self.zones, self.arena_half_size, behavior, commands,
            )

    def log(self, message):
        print("[acceptance] " + message, file=sys.stderr, flush=True)

    def send(self, command, parameters=None):
        payload = {"command": command, "parameters": parameters or {}}
        self.command_pub.publish(String(data=json.dumps(payload)))

    def _record_transport_phase(self, status, observed_wall):
        """Keep a bounded, task-correlated timeline of phase changes.

        The caller holds ``self.lock``.  Only small scalar fields are retained,
        so this evidence cannot grow with the status publication rate or carry
        arbitrary task data into RESULT_JSON.
        """
        task_id = str(status.get("task_id") or "")
        if not task_id or task_id != str(self.active_task_id or ""):
            return
        phase = str(status.get("phase") or "").strip().upper()
        if phase not in TRANSPORT_PHASES:
            if not self.active_transport_phase_error:
                self.active_transport_phase_error = (
                    "unknown transport phase: " + (phase or "<empty>")
                )
            return
        if phase == self.active_transport_phase:
            return

        previous_phase = self.active_transport_phase
        if previous_phase is not None and (
            TRANSPORT_PHASE_ORDER[phase]
            < TRANSPORT_PHASE_ORDER[previous_phase]
            or previous_phase in {"DONE", "FAILED"}
        ):
            if not self.active_transport_phase_error:
                self.active_transport_phase_error = (
                    "transport phase regressed from {} to {}".format(
                        previous_phase, phase
                    )
                )
            return

        self.active_transport_phase = phase
        self.active_transport_phase_started_wall = observed_wall
        wall_start = self.active_case_wall_start
        sim_start = self.active_case_sim_start
        wall_elapsed = (
            max(0.0, observed_wall - wall_start)
            if wall_start is not None else None
        )
        sim_elapsed = None
        if self.sim_time is not None and sim_start is not None:
            sim_elapsed = max(0.0, self.sim_time - sim_start)
        transition = {
            "phase": phase,
            "wall_elapsed_s": rounded(wall_elapsed),
            "sim_elapsed_s": rounded(sim_elapsed),
            "status_sequence": self.behavior_status_sequences["transport"],
        }
        if (
            len(self.active_transport_phase_timeline)
            < MAXIMUM_TRANSPORT_PHASE_TRANSITIONS
        ):
            self.active_transport_phase_timeline.append(transition)
        else:
            self.active_transport_phase_timeline_dropped += 1

    def _transport_timeline_report(self):
        """Copy the bounded phase history while its task is still active."""
        with self.lock:
            return {
                "wall_clock": "time.monotonic",
                "simulation_clock": "/clock",
                "transitions": [
                    dict(item) for item in self.active_transport_phase_timeline
                ],
                "maximum_transitions": MAXIMUM_TRANSPORT_PHASE_TRANSITIONS,
                "dropped_transitions": (
                    self.active_transport_phase_timeline_dropped
                ),
                "protocol_error": self.active_transport_phase_error,
            }

    def wait_for(
        self, predicate, timeout, description, continue_after_stop=False,
    ):
        deadline = time.monotonic() + timeout
        while (
            not rospy.is_shutdown()
            and (continue_after_stop or not self.stop_requested)
        ):
            with self.lock:
                if predicate():
                    return True
            if time.monotonic() >= deadline:
                self.log("timeout waiting for " + description)
                return False
            time.sleep(0.1)
        return False

    def _wait_for_transport_completion(self, case, task_id):
        """Wait for a terminal task with optional per-phase wall budgets."""
        started_at = time.monotonic()
        hard_timeout = max(0.0, float(case["timeout"]))
        raw_phase_timeouts = case.get("phase_timeouts", {})
        phase_timeouts = {
            str(phase).upper(): max(0.0, float(value))
            for phase, value in raw_phase_timeouts.items()
            if str(phase).upper() in TRANSPORT_PHASES
        }

        def response(
            reason, now, phase=None, phase_elapsed=None,
            phase_error=None,
        ):
            timed_out = reason in {"phase_timeout", "scenario_timeout"}
            return {
                "completion_wait_satisfied": reason in {
                    "task_completed", "task_failed", "task_stopped"
                },
                "termination_reason": reason,
                "timeout_phase": phase if timed_out else None,
                "completion_wait_elapsed_wall_s": rounded(
                    max(0.0, now - started_at)
                ),
                "completion_wait_timeout_wall_s": rounded(hard_timeout),
                "timeout_phase_elapsed_wall_s": (
                    rounded(phase_elapsed) if timed_out else None
                ),
                "transport_phase_timeouts_wall_s": {
                    key: rounded(value)
                    for key, value in sorted(phase_timeouts.items())
                },
                "transport_phase_timeout_tolerance_wall_s": (
                    TRANSPORT_PHASE_TIMEOUT_TOLERANCE_WALL_S
                ),
                "transport_phase_protocol_error": phase_error,
            }

        while True:
            now = time.monotonic()
            with self.lock:
                task = dict(self.swarm_task)
                emergency = self.emergency_stop
                phase = self.active_transport_phase
                phase_started = self.active_transport_phase_started_wall
                phase_error = self.active_transport_phase_error

            correlated = str(task.get("task_id") or "") == str(task_id)
            task_state = str(task.get("status") or "").strip().lower()
            if emergency:
                return response("emergency_stop", now, phase)
            if phase_error:
                return response(
                    "phase_protocol_error", now, phase,
                    phase_error=phase_error,
                )
            if correlated and task_state in TERMINAL_STATES:
                return response("task_" + task_state, now, phase)
            if rospy.is_shutdown():
                return response("ros_shutdown", now, phase)
            if self.stop_requested:
                return response("stop_requested", now, phase)

            phase_limit = phase_timeouts.get(phase)
            phase_elapsed = (
                max(0.0, now - phase_started)
                if phase_started is not None else None
            )
            if (
                phase_limit is not None
                and phase_elapsed is not None
                and phase_elapsed
                >= phase_limit + TRANSPORT_PHASE_TIMEOUT_TOLERANCE_WALL_S
            ):
                self.log(
                    "timeout waiting for transport {} phase liveness".format(
                        phase
                    )
                )
                return response(
                    "phase_timeout", now, phase, phase_elapsed
                )

            elapsed = max(0.0, now - started_at)
            if elapsed >= hard_timeout:
                self.log("timeout waiting for task completion")
                return response(
                    "scenario_timeout", now, phase, phase_elapsed
                )
            time.sleep(0.05)

    def stop_task(self, continue_after_stop=False):
        with self.lock:
            active_task_id = str(self.active_task_id or "")
            status_task_id = str(self.swarm_task.get("task_id") or "")
            task_id = active_task_id or status_task_id
            task_status = str(
                self.swarm_task.get("status") or "idle"
            ).lower()

        if not task_id:
            if task_status != "idle":
                raise RuntimeError(
                    "active swarm task has no task_id for correlated cleanup"
                )
            return
        if task_status in TERMINAL_STATES and status_task_id == task_id:
            with self.lock:
                self.active_task_id = None
            return

        self.send("stop_task", {"task_id": task_id})
        stopped = self.wait_for(
            lambda: (
                str(self.swarm_task.get("task_id") or "") == task_id
                and str(self.swarm_task.get("status") or "").lower()
                in TERMINAL_STATES
            ),
            4.0, "task stop",
            continue_after_stop=continue_after_stop,
        )
        if not stopped:
            raise RuntimeError(
                "task {} did not confirm a correlated stop".format(task_id)
            )
        with self.lock:
            self.active_task_id = None

    def _robot_model_names(self):
        return {
            name for name in self.model_names
            if ROBOT_RE.fullmatch(str(name))
        }

    def _delete_fleet(self, continue_after_stop=False):
        """Delete the fleet and prove its Gazebo and monitor state is gone."""
        with self.lock:
            departed = set(self.roster) | self._robot_model_names()

        self.send("delete_robots", {})
        if not self.wait_for(
            lambda: not self.roster,
            30.0,
            "empty fleet",
            continue_after_stop=continue_after_stop,
        ):
            raise RuntimeError("fleet deletion did not finish")
        if not self.wait_for(
            lambda: not self._robot_model_names(),
            8.0,
            "Gazebo model deletion",
            continue_after_stop=continue_after_stop,
        ):
            raise RuntimeError(
                "robot models remained in Gazebo after fleet deletion"
            )
        if not self.wait_for(
            lambda: not self.cmd_vel_subscribers and not self.cmd_velocities,
            3.0,
            "cmd_vel monitor cleanup",
            continue_after_stop=continue_after_stop,
        ):
            raise RuntimeError(
                "cmd_vel monitors remained after fleet deletion"
            )
        return departed

    def _cleanup_case(self, case):
        """Stop the exact case before restoring any shared Gazebo state."""
        self.stop_task(continue_after_stop=True)
        if case["behavior"] == "transport":
            self.reset_object()

    def _final_cleanup(self):
        """Finish task and optional fleet cleanup without hiding a failure."""
        failures = []
        try:
            self.stop_task(continue_after_stop=True)
        except Exception as exc:
            failures.append("task stop failed: " + str(exc))
            return failures

        if not self.args.delete_after:
            return failures

        try:
            self._delete_fleet(continue_after_stop=True)
        except Exception as exc:
            failures.append("fleet deletion failed: " + str(exc))
        return failures

    def reset_object(self, position=None):
        rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        state = ModelState()
        state.model_name = "transport_object"
        state.reference_frame = "world"
        # Most cases begin within sensor range.  A scenario may place the
        # payload farther away when it needs to exercise SEARCH explicitly.
        object_x, object_y = position or (-0.8, -1.6)
        state.pose.position.x = object_x
        state.pose.position.y = object_y
        state.pose.position.z = 0.1
        state.pose.orientation.w = 1.0
        response = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )(state)
        if not response.success:
            raise RuntimeError("could not reset transport object: " + response.status_message)
        time.sleep(0.4)

    def reset_fleet(
        self, count, pattern, reset_payload=False, payload_position=None,
    ):
        self.stop_task()
        old_robots = self._delete_fleet()

        with self.lock:
            self.last_reset_cleanup = {
                "departed_robot_ids": sorted(
                    old_robots, key=robot_sort_key
                ),
                "fleet_roster_empty": not self.roster,
                "gazebo_departed_models_remaining": sorted(
                    old_robots.intersection(self.model_names),
                    key=robot_sort_key,
                ),
                "harness_cmd_vel_subscribers_remaining": sorted(
                    old_robots.intersection(self.cmd_vel_subscribers),
                    key=robot_sort_key,
                ),
                "harness_cmd_velocity_entries_remaining": sorted(
                    old_robots.intersection(self.cmd_velocities),
                    key=robot_sort_key,
                ),
            }
        if reset_payload:
            self.reset_object(payload_position)
        self.send("spawn_robots", {
            "robot_count": count,
            "spawn_pattern": pattern,
            "robot_ids": acceptance_robot_ids(count),
        })
        if not self.wait_for(lambda: len(self.roster) == count,
                             45.0, "fleet roster"):
            raise RuntimeError("fleet spawn did not produce {} robots".format(count))
        if not self.wait_for(lambda: set(self.roster).issubset(self.model_names),
                             10.0, "Gazebo robot models"):
            raise RuntimeError("spawned robots are missing from model_states")
        time.sleep(0.8)

    def task_parameters(self, case, task_id):
        common = {"task_id": task_id}
        if case["behavior"] == "formation":
            active_seconds = float(
                getattr(self.args, "formation_active_seconds", 0.0) or 0.0
            )
            common.update({
                "task_type": "formation",
                "formation_type": case["shape"],
                "movement_mode": (
                    "moving" if active_seconds > 0.0 else "static"
                ),
                "config": {"spacing": case["spacing"]},
            })
        elif case["behavior"] == "follow":
            common.update({
                "task_type": "follow_leader",
                "leader_mode": case["mode"],
                "config": {
                    "radius": case["radius"],
                    "follow_distance": case.get("follow_distance", 0.65),
                },
            })
        else:
            common.update({
                "task_type": "transport",
                "target_x": case["target"][0],
                "target_y": case["target"][1],
                "config": {"transport_planner": "grf"},
            })
        return common

    def run_case(self, case):
        self.log("starting {} ({} robots, {} spawn)".format(
            case["name"], case["count"], case["pattern"]
        ))
        requested_task_id = str(
            getattr(self.args, "task_id", "") or ""
        ).strip()
        task_id = requested_task_id or "accept-{}-{}".format(
            case["name"], uuid.uuid4().hex[:8]
        )
        result = {
            "scenario": case["name"],
            "behavior": case["behavior"],
            "robot_count": case["count"],
            "spawn_pattern": case["pattern"],
            "task_id": task_id,
            "passed": False,
            "failures": [],
            "cleanup_failures": [],
            "warnings": [],
        }
        required_follow_laps = max(1, int(case.get("required_laps", 1)))
        formation_active_seconds = (
            float(getattr(self.args, "formation_active_seconds", 0.0) or 0.0)
            if case["behavior"] == "formation" else 0.0
        )
        follow_active_seconds = (
            float(getattr(self.args, "follow_active_seconds", 0.0) or 0.0)
            if case["behavior"] == "follow" else 0.0
        )
        follow_lap_completed = False
        collision_start = 0
        metrics = None
        try:
            self.reset_fleet(
                case["count"], case["pattern"],
                reset_payload=case["behavior"] == "transport",
                payload_position=case.get("object_start"),
            )
            with self.lock:
                robot_ids = list(self.roster)
                result["pre_spawn_cleanup"] = dict(self.last_reset_cleanup)
                collision_start = self.collision_count
                case_wall_start = time.monotonic()
                metrics = CaseMetrics(
                    case, robot_ids, self.sim_time, case_wall_start, self.args
                )
                self.active_metrics = metrics
                self.active_task_id = task_id
                self.last_metrics_sim = None
                self.active_case_wall_start = case_wall_start
                self.active_case_sim_start = self.sim_time
                self.active_transport_phase = None
                self.active_transport_phase_started_wall = None
                self.active_transport_phase_error = None
                self.active_transport_phase_timeline = []
                self.active_transport_phase_timeline_dropped = 0
                self._begin_collision_attribution(
                    robot_ids, case["behavior"]
                )

            self.send("start_task", self.task_parameters(case, task_id))
            started = self.wait_for(
                lambda: (self.swarm_task.get("task_id") == task_id and
                         self.swarm_task.get("status") in
                         {"running", "completed", "failed"}),
                15.0, "task dispatch",
            )
            if not started:
                result["failures"].append("task did not start")
                result.update({
                    "completion_wait_satisfied": False,
                    "termination_reason": "task_not_started",
                    "timeout_phase": None,
                    "completion_wait_elapsed_wall_s": 0.0,
                    "completion_wait_timeout_wall_s": case["timeout"],
                    "timeout_phase_elapsed_wall_s": None,
                })
            elif case["behavior"] == "formation" and formation_active_seconds:
                ready_or_terminal = self.wait_for(
                    lambda: formation_is_ready_or_terminal(
                        self.swarm_task,
                        self.behavior_status["formation"],
                        task_id,
                        case,
                        robot_ids,
                        self.args.max_formation_error,
                    ),
                    case["timeout"],
                    "complete or terminal moving formation",
                )
                with self.lock:
                    task_after_ready_wait = dict(self.swarm_task)
                    formation_after_ready_wait = dict(
                        self.behavior_status["formation"]
                    )
                ready = (
                    ready_or_terminal
                    and task_after_ready_wait.get("task_id") == task_id
                    and task_after_ready_wait.get("status") == "running"
                    and not formation_active_status_failures(
                        formation_after_ready_wait,
                        task_id,
                        case,
                        robot_ids,
                        self.args.max_formation_error,
                    )
                )
                window_started = time.monotonic()
                fresh_samples = 0
                last_sequence = None
                last_fresh_at = window_started
                window_failure = None
                if not ready:
                    result["failures"].append(
                        "formation did not become complete in moving mode"
                    )
                else:
                    with self.lock:
                        last_sequence = self.behavior_status_sequences[
                            "formation"
                        ]
                    deadline = window_started + formation_active_seconds
                    while (
                        time.monotonic() < deadline
                        and not self.stop_requested
                    ):
                        now = time.monotonic()
                        with self.lock:
                            task = dict(self.swarm_task)
                            status = dict(self.behavior_status["formation"])
                            sequence = self.behavior_status_sequences[
                                "formation"
                            ]
                            status_at = self.behavior_status_wall_times[
                                "formation"
                            ]
                            emergency = self.emergency_stop
                            collisions = max(
                                0, self.collision_count - collision_start
                            )
                        if (
                            task.get("task_id") != task_id
                            or task.get("status") != "running"
                        ):
                            window_failure = (
                                "moving formation task did not remain running"
                            )
                            break
                        if emergency or collisions:
                            window_failure = (
                                "moving formation crossed a live safety gate"
                            )
                            break
                        if sequence != last_sequence:
                            sample_failures = formation_active_status_failures(
                                status,
                                task_id,
                                case,
                                robot_ids,
                                self.args.max_formation_error,
                            )
                            if sample_failures:
                                window_failure = sample_failures[0]
                                break
                            fresh_samples += 1
                            last_sequence = sequence
                            last_fresh_at = status_at or now
                        if now - last_fresh_at > 2.0:
                            window_failure = (
                                "moving formation status stopped updating"
                            )
                            break
                        time.sleep(0.05)

                    observed_seconds = max(
                        0.0, time.monotonic() - window_started
                    )
                    window_complete = (
                        window_failure is None
                        and observed_seconds + 0.001
                        >= formation_active_seconds
                        and fresh_samples >= 5
                    )
                    result["formation_active_window"] = {
                        "requested_wall_seconds": formation_active_seconds,
                        "observed_wall_seconds": rounded(observed_seconds),
                        "fresh_status_samples": fresh_samples,
                        "state": "moving",
                        "complete_fleet_required": True,
                        "live_safety_signals_monitored": True,
                        "completed": window_complete,
                    }
                    if window_failure:
                        result["failures"].append(window_failure)
                    elif observed_seconds + 0.001 < formation_active_seconds:
                        result["failures"].append(
                            "moving formation active window was interrupted"
                        )
                    elif fresh_samples < 5:
                        result["failures"].append(
                            "moving formation produced too few fresh status samples"
                        )
                    elif window_complete:
                        result["task_outcome"] = "running_for_duration"
                if "task_outcome" not in result:
                    with self.lock:
                        result["task_outcome"] = self.swarm_task.get(
                            "status", "unknown"
                        )
            elif case["behavior"] == "follow":
                deadline = time.monotonic() + case["duration"]
                active_window_started = time.monotonic()
                while time.monotonic() < deadline and not self.stop_requested:
                    with self.lock:
                        state = self.swarm_task.get("status")
                        emergency = self.emergency_stop
                        follow_status = dict(self.behavior_status["follow"])
                    follow_lap_completed = follow_lap_requirement_met(
                        follow_status,
                        task_id,
                        case["mode"],
                        required_follow_laps,
                    )
                    active_window_complete = (
                        not follow_active_seconds
                        or time.monotonic() - active_window_started
                        >= follow_active_seconds
                    )
                    if follow_lap_completed and active_window_complete:
                        break
                    if state in {"failed", "stopped"} or emergency:
                        break
                    time.sleep(0.2)
                if not follow_lap_completed and time.monotonic() >= deadline:
                    self.log(
                        "timeout waiting for {} complete {} lap(s)".format(
                            case["name"], required_follow_laps
                        )
                    )
                observed_active_seconds = max(
                    0.0, time.monotonic() - active_window_started
                )
                if follow_active_seconds:
                    active_window_complete = (
                        observed_active_seconds + 0.001
                        >= follow_active_seconds
                    )
                    result["follow_active_window"] = {
                        "requested_wall_seconds": follow_active_seconds,
                        "observed_wall_seconds": rounded(
                            observed_active_seconds
                        ),
                        "required_laps": required_follow_laps,
                        "lap_requirement_met": follow_lap_completed,
                        "completed": (
                            active_window_complete and follow_lap_completed
                        ),
                    }
                    if not active_window_complete:
                        result["failures"].append(
                            "follow active window was interrupted"
                        )
                with self.lock:
                    state = self.swarm_task.get("status")
                result["task_outcome"] = (
                    "running_for_duration" if state == "running" else state
                )
            elif case["behavior"] == "transport":
                result.update(self._wait_for_transport_completion(
                    case, task_id
                ))
                with self.lock:
                    result["task_outcome"] = self.swarm_task.get(
                        "status", "unknown"
                    )
                result["transport_phase_timeline"] = (
                    self._transport_timeline_report()
                )
            else:
                completion_started = time.monotonic()
                completion_wait_satisfied = self.wait_for(
                    lambda: (self.swarm_task.get("task_id") == task_id and
                             self.swarm_task.get("status") in TERMINAL_STATES),
                    case["timeout"], "task completion",
                )
                with self.lock:
                    result["task_outcome"] = self.swarm_task.get("status", "unknown")
                completion_elapsed = max(
                    0.0, time.monotonic() - completion_started
                )
                result.update({
                    "completion_wait_satisfied": completion_wait_satisfied,
                    "termination_reason": (
                        "task_" + str(result["task_outcome"]).lower()
                        if completion_wait_satisfied
                        else "scenario_timeout"
                    ),
                    "timeout_phase": None,
                    "completion_wait_elapsed_wall_s": rounded(
                        completion_elapsed
                    ),
                    "completion_wait_timeout_wall_s": case["timeout"],
                    "timeout_phase_elapsed_wall_s": None,
                })

            with self.lock:
                final_sim_time = self.sim_time
                swarm_task = dict(self.swarm_task)
                emergency = self.emergency_stop
                collision_delta = max(0, self.collision_count - collision_start)
                behavior = dict(self.behavior_status[case["behavior"]])
                collision_attribution = self._finish_collision_attribution(
                    collision_delta
                )
                self.active_metrics = None
            with self.metrics_update_lock:
                if str(behavior.get("task_id") or "") == task_id:
                    metrics.observe_behavior_status(behavior)
                metrics.finish(final_sim_time, time.monotonic())
                metric_report = metrics.report()

            result["metrics"] = metric_report
            result["task"] = swarm_task
            result["behavior_status"] = {
                key: behavior.get(key)
                for key in RESULT_BEHAVIOR_STATUS_KEYS
                if key in behavior
            }
            if case["behavior"] == "follow":
                follow_lap_completed = (
                    follow_lap_completed
                    or follow_lap_requirement_met(
                        behavior,
                        task_id,
                        case["mode"],
                        required_follow_laps,
                    )
                )
                result["follow_required_laps"] = required_follow_laps
                result["follow_lap_requirement_met"] = follow_lap_completed
            result["collision_count_delta"] = collision_delta
            result["collision_episode_attribution"] = collision_attribution
            contact_classification = classify_contact_episodes(
                case["behavior"], collision_delta, metric_report, self.args,
                collision_attribution,
            )
            result["contact_episode_classification"] = (
                contact_classification
            )
            result["expected_contact_count_delta"] = (
                contact_classification[
                    "classified_expected_contact_count_delta"
                ]
            )
            result["unexpected_contact_count_delta"] = (
                contact_classification[
                    "classified_unexpected_contact_count_delta"
                ]
            )
            result["acceptance_thresholds"] = {
                "minimum_robot_center_distance_m": self.args.min_center_distance,
                "minimum_static_obstacle_clearance_m": (
                    self.args.min_obstacle_clearance
                ),
                "minimum_boundary_clearance_m": self.args.min_boundary_clearance,
                "minimum_real_time_factor": self.args.min_rtf,
                "maximum_linear_speed_mps": self.args.max_speed,
                "maximum_linear_acceleration_mps2": self.args.max_acceleration,
                "linear_acceleration_measurement_window_s": (
                    self.args.acceleration_window
                ),
                "maximum_formation_target_error_m": self.args.max_formation_error,
                "minimum_moving_formation_wall_seconds": (
                    formation_active_seconds
                    if formation_active_seconds else None
                ),
                "minimum_follow_robot_travel_m": self.args.min_follow_travel,
                "minimum_follow_active_wall_seconds": (
                    follow_active_seconds if follow_active_seconds else None
                ),
                "minimum_follow_completed_laps": (
                    required_follow_laps
                    if case["behavior"] == "follow" else None
                ),
                "follow_lap_timeout_wall_s": (
                    case.get("duration")
                    if case["behavior"] == "follow" else None
                ),
                "maximum_follow_adjacent_spacing_error_m": (
                    self.args.max_follow_spacing_error
                ),
            }

            if not result.get("task_outcome"):
                result["task_outcome"] = swarm_task.get("status", "unknown")
            if result.get("termination_reason") == "phase_timeout":
                result["failures"].append(
                    "transport {} phase exceeded its liveness budget".format(
                        result.get("timeout_phase") or "unknown"
                    )
                )
            elif result.get("termination_reason") == "phase_protocol_error":
                result["failures"].append(
                    result.get("transport_phase_protocol_error")
                    or "transport reported an invalid phase transition"
                )
            elif result.get("termination_reason") == "scenario_timeout":
                result["failures"].append(
                    "task completion exceeded the scenario wall-time budget"
                )
            if case["behavior"] == "formation" and formation_active_seconds:
                if result["task_outcome"] != "running_for_duration":
                    result["failures"].append(
                        "moving formation did not remain running for its window"
                    )
            elif case["behavior"] in {"formation", "transport"}:
                if result["task_outcome"] != "completed":
                    result["failures"].append(
                        "task outcome was " + str(result["task_outcome"])
                    )
            elif result["task_outcome"] != "running_for_duration":
                result["failures"].append(
                    "continuous task did not remain running"
                )

            if str(swarm_task.get("task_id") or "") != task_id:
                result["failures"].append(
                    "swarm status was not correlated to the requested task"
                )
            if str(behavior.get("task_id") or "") != task_id:
                result["failures"].append(
                    "behavior status was not correlated to the requested task"
                )
            if metric_report["correlated_behavior_samples"] < 1:
                result["failures"].append(
                    "no correlated behavior status was measured"
                )

            if emergency:
                result["failures"].append("emergency stop became active")
            if metric_report["model_samples"] < 5:
                result["failures"].append("insufficient model-state samples")
            if case["behavior"] in {"formation", "follow"}:
                minimum_pair = metric_report["minimum_robot_center_distance_m"]
                if case["count"] > 1 and minimum_pair is None:
                    result["failures"].append(
                        "robot centre-distance telemetry was unavailable"
                    )
                elif (
                    minimum_pair is not None
                    and minimum_pair < self.args.min_center_distance
                ):
                    result["failures"].append(
                        "robot center-distance safety limit crossed"
                    )
            else:
                minimum_pair = metric_report[
                    "minimum_unexpected_robot_center_distance_m"
                ]
                if (
                    minimum_pair is not None
                    and minimum_pair < self.args.min_center_distance
                ):
                    result["failures"].append(
                        "unexpected robot center-distance safety limit crossed"
                    )
            if contact_classification["hard_failure"]:
                result["failures"].append(
                    "unexpected contact was reported during the task"
                )
            minimum_obstacle = metric_report["minimum_static_obstacle_clearance_m"]
            if minimum_obstacle is not None and minimum_obstacle < self.args.min_obstacle_clearance:
                result["failures"].append("static obstacle clearance safety limit crossed")
            minimum_boundary = metric_report["minimum_boundary_clearance_m"]
            if minimum_boundary is not None and minimum_boundary < self.args.min_boundary_clearance:
                result["failures"].append("arena boundary clearance safety limit crossed")
            rtf = metric_report["real_time_factor"]
            if rtf is None or rtf < self.args.min_rtf:
                result["failures"].append("real-time factor below {:.2f}".format(self.args.min_rtf))
            max_speed = metric_report["linear_speed_mps"]["max"]
            if max_speed is not None and max_speed > self.args.max_speed:
                result["failures"].append("linear speed safety limit crossed")
            max_acceleration = metric_report["linear_acceleration_mps2"]["max"]
            if max_acceleration is None:
                result["failures"].append("linear acceleration was not measured")
            elif max_acceleration > self.args.max_acceleration:
                result["failures"].append(
                    "linear acceleration safety limit crossed"
                )

            if case["behavior"] == "formation":
                expected_movement_mode = (
                    "moving" if formation_active_seconds else "static"
                )
                expected_state = (
                    "moving" if formation_active_seconds else "formed"
                )
                requested_shape = str(case["shape"]).strip().lower()
                reported_shape = str(
                    behavior.get("formation_type") or ""
                ).strip().lower()
                if reported_shape != requested_shape:
                    result["failures"].append(
                        "formation status did not report the requested shape"
                    )
                try:
                    reported_count = int(behavior.get("robot_count"))
                except (TypeError, ValueError):
                    reported_count = None
                if reported_count != case["count"]:
                    result["failures"].append(
                        "formation status did not report the requested robot count"
                    )
                if behavior.get("movement_mode") != expected_movement_mode:
                    result["failures"].append(
                        "formation status did not report the requested movement mode"
                    )
                if behavior.get("state") != expected_state:
                    result["failures"].append(
                        "formation status did not reach the requested state"
                    )
                reported_assignments = behavior.get("robot_assignments")
                assignment_names = set()
                if isinstance(reported_assignments, dict):
                    assignment_names = {
                        clean_robot_name(name) for name in reported_assignments
                    }
                    assignment_names.discard(None)
                if assignment_names != set(metric_report["robot_travel_m"]):
                    result["failures"].append(
                        "formation status did not assign the complete fleet"
                    )
                reported_error = finite_number(
                    behavior.get("maximum_position_error")
                )
                if reported_error is None:
                    result["failures"].append(
                        "formation status omitted its final position error"
                    )
                elif reported_error > self.args.max_formation_error:
                    result["failures"].append(
                        "formation reported position error above the limit"
                    )
                actual_errors = metric_report[
                    "formation_final_target_error_summary_m"
                ]
                if (
                    actual_errors["samples"] != case["count"]
                    or metric_report["formation_missing_final_targets"]
                ):
                    result["failures"].append(
                        "formation final targets did not cover the complete fleet"
                    )
                elif actual_errors["max"] > self.args.max_formation_error:
                    result["failures"].append(
                        "Gazebo final target error exceeded the formation limit"
                    )

            if case["behavior"] == "follow":
                if not bool(behavior.get("active", False)):
                    result["failures"].append(
                        "follow behavior was not active at the end of the window"
                    )
                if behavior.get("leader_mode") != case["mode"]:
                    result["failures"].append(
                        "follow status did not report the requested leader mode"
                    )
                if not follow_lap_completed:
                    result["failures"].append(
                        "follow leader did not complete the required {} "
                        "parametric lap(s)".format(required_follow_laps)
                    )
                status_robots = behavior.get("robots")
                status_robot_names = []
                status_chain_indices = []
                if isinstance(status_robots, list):
                    for entry in status_robots:
                        if not isinstance(entry, dict):
                            continue
                        status_robot_names.append(
                            clean_robot_name(entry.get("name"))
                        )
                        try:
                            status_chain_indices.append(
                                int(entry.get("chain_index"))
                            )
                        except (TypeError, ValueError):
                            pass
                expected_robots = set(metric_report["robot_travel_m"])
                status_covers_fleet = (
                    len(status_robot_names) == case["count"]
                    and None not in status_robot_names
                    and set(status_robot_names) == expected_robots
                    and len(set(status_chain_indices)) == case["count"]
                )
                if not status_covers_fleet:
                    result["failures"].append(
                        "follow status did not cover the complete fleet"
                    )
                stationary = [
                    robot for robot, travel in metric_report["robot_travel_m"].items()
                    if travel is None or travel < self.args.min_follow_travel
                ]
                if stationary:
                    result["failures"].append(
                        "follow robots did not all move far enough: "
                        + ", ".join(sorted(stationary, key=robot_sort_key))
                    )
                expected_spacing = case.get("follow_distance", 0.65)
                reported_spacing = finite_number(
                    behavior.get("follow_distance")
                )
                if (
                    reported_spacing is None
                    or abs(reported_spacing - expected_spacing) > 0.02
                ):
                    result["failures"].append(
                        "follow status did not preserve the requested spacing"
                    )
                spacing_errors = metric_report[
                    "follow_final_adjacent_spacing_error_summary_m"
                ]
                if spacing_errors["samples"] != max(0, case["count"] - 1):
                    result["failures"].append(
                        "adjacent follow spacing was not measured for the full chain"
                    )
                elif spacing_errors["max"] > self.args.max_follow_spacing_error:
                    result["failures"].append(
                        "adjacent follow spacing error exceeded the limit"
                    )
            if case["behavior"] == "transport":
                result["transport_acceptance_thresholds"] = {
                    "direct_payload_surface_clearance_m": (
                        self.args.transport_direct_contact_clearance
                    ),
                    "minimum_rearward_goal_projection_m": 0.05,
                    "companion_center_distance_m": (
                        self.args.transport_companion_contact_distance
                    ),
                    "minimum_chain_center_distance_m": (
                        self.args.min_transport_chain_center_distance
                    ),
                    "cmd_vel_min_forward_speed_mps": (
                        self.args.transport_cmd_min_speed
                    ),
                    "cmd_vel_min_goal_alignment_cosine": (
                        self.args.transport_cmd_min_goal_cosine
                    ),
                    "cmd_vel_max_wall_age_s": (
                        self.args.transport_cmd_max_age
                    ),
                    "contribution_goal_velocity_noise_floor_mps": (
                        self.args.transport_contribution_noise_floor
                    ),
                    "contribution_goal_velocity_tolerance_mps": (
                        self.args.transport_contribution_speed_tolerance
                    ),
                    "contribution_minimum_tracking_fraction": (
                        self.args.transport_contribution_tracking_fraction
                    ),
                    "contribution_definition": (
                        "connected role-directed control tracking the "
                        "payload or parent goal velocity"
                    ),
                    "contribution_policy": "hard_gate",
                    "maximum_control_sample_gap_s": (
                        self.args.transport_control_gap_tolerance
                    ),
                    "nominal_pace_min_forward_speed_mps": (
                        self.args.transport_cmd_min_speed
                    ),
                    "nominal_pace_policy": "warning",
                    "published_reference_pace_policy": "warning",
                    "minimum_continuous_useful_push_s": (
                        self.args.min_transport_push_duration
                    ),
                    "minimum_continuous_useful_push_samples": (
                        self.args.min_transport_push_samples
                    ),
                    "minimum_useful_push_fraction": (
                        self.args.min_transport_useful_fraction
                    ),
                    "configured_minimum_goal_progress_m": (
                        case["min_object_travel"]
                    ),
                    "goal_arrival_tolerance_m": case.get(
                        "goal_tolerance", 0.5
                    ),
                    "goal_progress_contract_epsilon_m": (
                        TRANSPORT_PROGRESS_EPSILON_M
                    ),
                    "goal_progress_contract_basis": (
                        "distance at synchronized PUSH start minus the "
                        "controller arrival tolerance"
                    ),
                    "minimum_goal_progress_efficiency": (
                        self.args.min_transport_goal_efficiency
                    ),
                    "maximum_pre_active_payload_motion_m": 0.05,
                    "minimum_grf_samples": (
                        self.args.min_transport_push_samples
                    ),
                    "minimum_simultaneous_all_robot_push_fraction": (
                        self.args.min_transport_useful_fraction
                    ),
                    "simultaneous_all_robot_push_fraction_basis": (
                        "distinct coherent control batches"
                    ),
                    "simultaneous_positive_progress_policy": "warning",
                    "minimum_notified_robot_rendezvous_travel_m": (
                        self.args.min_transport_rendezvous_travel
                    ),
                    "minimum_payload_root_rendezvous_travel_m": (
                        self.args.min_transport_root_rendezvous_travel
                    ),
                    "rendezvous_motion_detection_distance_m": (
                        self.args.transport_motion_detection_distance
                    ),
                    "rendezvous_moving_speed_mps": (
                        self.args.transport_rendezvous_moving_speed
                    ),
                    "minimum_simultaneous_active_search_movers": (
                        case["count"]
                    ),
                    "minimum_motion_window_samples_for_gate": 3,
                    "minimum_active_search_path_length_per_robot_m": (
                        self.args.min_transport_search_travel
                    ),
                    "active_search_required": bool(
                        case.get("require_search", False)
                    ),
                    "maximum_time_to_first_all_useful_batch_wall_s": (
                        self.args.max_transport_first_all_useful_wall_time
                    ),
                }
                response = metric_report["transport_discovery_response"]
                required_progress = required_transport_goal_progress(
                    case["min_object_travel"],
                    metric_report[
                        "transport_push_initial_goal_distance_m"
                    ],
                    case.get("goal_tolerance", 0.5),
                )
                result["transport_acceptance_thresholds"][
                    "minimum_goal_progress_m"
                ] = rounded(required_progress)
                if not response["notice_observed"]:
                    result["failures"].append(
                        "payload discovery notice was not observed"
                    )
                if response["missing_notice_recipients"]:
                    result["failures"].append(
                        "payload discovery did not dispatch the complete fleet: "
                        + ", ".join(response["missing_notice_recipients"])
                    )
                if (
                    response["status_acknowledgement_available"]
                    and response["missing_status_acknowledgements"]
                ):
                    result["failures"].append(
                        "robots did not acknowledge the payload notice: "
                        + ", ".join(
                            response["missing_status_acknowledgements"]
                        )
                    )
                if response["notified_robots_below_required_travel"]:
                    result["failures"].append(
                        "notified robots did not move far enough before PUSH: "
                        + ", ".join(
                            response[
                                "notified_robots_below_required_travel"
                            ]
                        )
                    )
                result["failures"].extend(
                    transport_search_motion_failures(
                        response,
                        case["count"],
                        require_search=case.get("require_search", False),
                    )
                )
                minimum_chain_distance = metric_report[
                    "minimum_declared_chain_center_distance_m"
                ]
                if (
                    minimum_chain_distance is not None
                    and minimum_chain_distance
                    < self.args.min_transport_chain_center_distance
                ):
                    result["failures"].append(
                        "declared push chain compressed below safety limit"
                    )
                if not metric_report["transport_active_push_latched"]:
                    result["failures"].append(
                        "synchronized GRF push never became active"
                    )
                first_useful_wall = metric_report[
                    "transport_time_to_first_all_useful_batch_wall_s"
                ]
                if self.args.max_transport_first_all_useful_wall_time > 0.0:
                    if first_useful_wall is None:
                        result["failures"].append(
                            "the first complete-fleet useful push batch was "
                            "not observed"
                        )
                    elif (
                        first_useful_wall
                        > self.args.max_transport_first_all_useful_wall_time
                    ):
                        result["failures"].append(
                            "the first complete-fleet useful push batch took "
                            "longer than {:.1f} wall seconds".format(
                                self.args.max_transport_first_all_useful_wall_time
                            )
                        )
                if (
                    metric_report["transport_grf_samples"]
                    < self.args.min_transport_push_samples
                ):
                    result["failures"].append(
                        "GRF did not execute for enough samples"
                    )
                pre_active_motion = metric_report[
                    "transport_pre_active_object_displacement_m"
                ]
                if (
                    pre_active_motion is not None
                    and pre_active_motion > 0.05
                ):
                    result["failures"].append(
                        "payload moved too far before synchronized pushing"
                    )
                progress = metric_report[
                    "transport_push_goal_progress_m"
                ]
                if progress is None or progress < required_progress:
                    result["failures"].append(
                        "post-engagement payload progress was too small"
                    )
                efficiency = metric_report[
                    "transport_push_goal_progress_efficiency"
                ]
                if (
                    efficiency is None
                    or efficiency < self.args.min_transport_goal_efficiency
                ):
                    result["failures"].append(
                        "payload path was not sufficiently goal-directed"
                    )
                if case["count"] > 1:
                    if (
                        metric_report[
                            "transport_maximum_continuous_all_useful_s"
                        ] < self.args.min_transport_push_duration
                        or metric_report[
                            "transport_maximum_continuous_all_useful_samples"
                        ] < self.args.min_transport_push_samples
                    ):
                        result["failures"].append(
                            "the complete fleet never sustained one "
                            "simultaneous useful push interval"
                        )
                    if (
                        metric_report[
                            "transport_all_useful_batch_fraction"
                        ] < self.args.min_transport_useful_fraction
                    ):
                        result["failures"].append(
                            "the complete fleet pushed simultaneously in "
                            "less than {:.0%} of measured control batches".format(
                                self.args.min_transport_useful_fraction
                            )
                        )
                    participation = metric_report["transport_participation"]
                    for robot in sorted(participation):
                        item = participation[robot]
                        if item["role"] not in {
                            "payload_push", "companion_push"
                        }:
                            result["failures"].append(
                                "{} had no declared transport-push role".format(
                                    robot
                                )
                            )
                            continue
                        if (
                            item["role"] == "companion_push"
                            and not item["declared_parent_namespaces"]
                        ):
                            result["failures"].append(
                                "{} had no declared chain predecessor".format(
                                    robot
                                )
                            )
                        if (
                            item["maximum_continuous_useful_pushing_s"]
                            < self.args.min_transport_push_duration
                            or item[
                                "maximum_continuous_useful_pushing_samples"
                            ] < self.args.min_transport_push_samples
                        ):
                            result["failures"].append(
                                "{} did not sustain connected goal-directed "
                                "pushing".format(robot)
                            )
                        if (
                            item["useful_pushing_fraction"]
                            < self.args.min_transport_useful_fraction
                        ):
                            result["failures"].append(
                                "{} contributed for less than {:.0%} of "
                                "positive payload progress".format(
                                    robot,
                                    self.args.min_transport_useful_fraction,
                                )
                            )

                    if (
                        metric_report["transport_all_useful_fraction"]
                        < self.args.min_transport_useful_fraction
                    ):
                        result["warnings"].append(
                            "the complete fleet contributed together for less "
                            "than {:.0%} of positive payload progress".format(
                                self.args.min_transport_useful_fraction
                            )
                        )

                    pace_warnings = metric_report[
                        "transport_nominal_pace_warning_robots"
                    ]
                    if pace_warnings:
                        result["warnings"].append(
                            "connected contribution fell below the nominal "
                            "{:.3f} m/s pace for: {}".format(
                                self.args.transport_cmd_min_speed,
                                ", ".join(pace_warnings),
                            )
                        )
                    if metric_report[
                        "transport_all_nominal_pace_warning"
                    ]:
                        result["warnings"].append(
                            "the complete fleet did not sustain the nominal "
                            "fixed pace in one simultaneous control window"
                        )
                    if (
                        metric_report["transport_reference_pace_samples"]
                        and metric_report[
                            "transport_all_reference_pace_fraction"
                        ] < self.args.min_transport_useful_fraction
                    ):
                        result["warnings"].append(
                            "the complete fleet tracked the published push "
                            "reference for less than {:.0%} of measured "
                            "control batches".format(
                                self.args.min_transport_useful_fraction
                            )
                        )

            result["passed"] = not result["failures"]
        except Exception as exc:  # Keep later matrix cases runnable.
            result["failures"].append("{}: {}".format(type(exc).__name__, exc))
            result["task_outcome"] = "harness_error"
            if metrics is not None:
                with self.lock:
                    final_sim_time = self.sim_time
                    collision_delta = max(
                        0, self.collision_count - collision_start
                    )
                    result["collision_count_delta"] = collision_delta
                    result["collision_episode_attribution"] = (
                        self._finish_collision_attribution(collision_delta)
                    )
                    self.active_metrics = None
                with self.metrics_update_lock:
                    metrics.finish(final_sim_time, time.monotonic())
                    result["metrics"] = metrics.report()
        finally:
            if (
                case["behavior"] == "transport"
                and "transport_phase_timeline" not in result
            ):
                result["transport_phase_timeline"] = (
                    self._transport_timeline_report()
                )
            with self.lock:
                if self.active_collision_robot_ids:
                    self._finish_collision_attribution(0)
            try:
                self._cleanup_case(case)
            except Exception as exc:
                cleanup_failure = str(exc)
                result["cleanup_failures"].append(cleanup_failure)
                with self.lock:
                    self.case_cleanup_failures.append(
                        "{}: {}".format(case["name"], cleanup_failure)
                    )
                result["failures"].append(
                    "cleanup failed: " + cleanup_failure
                )
            result["passed"] = not result["failures"]

        print("RESULT_JSON " + json.dumps(
            result, sort_keys=True, separators=(",", ":"), allow_nan=False
        ), flush=True)
        return result

    def run(self, cases):
        results = []
        with self.lock:
            self.case_cleanup_failures = []
        try:
            if not self.wait_for(
                lambda: self.command_pub.get_num_connections() > 0,
                10.0,
                "/swarm/commands subscriber",
            ):
                raise RuntimeError("task orchestrator is not connected")
            if not self.wait_for(
                lambda: self.sim_time is not None and bool(self.model_names),
                10.0,
                "Gazebo telemetry",
            ):
                raise RuntimeError(
                    "/clock or /gazebo/model_states is unavailable"
                )
            for case in cases:
                if self.stop_requested:
                    break
                results.append(self.run_case(case))
        finally:
            with self.lock:
                case_cleanup_failures = list(self.case_cleanup_failures)
            self.last_cleanup_failures = (
                case_cleanup_failures + self._final_cleanup()
            )

        passed = sum(1 for result in results if result["passed"])
        cleanup_passed = not self.last_cleanup_failures
        final = {
            "selected": len(cases),
            "executed": len(results),
            "passed": passed,
            "failed": len(results) - passed,
            "all_passed": (
                len(results) == len(cases)
                and passed == len(cases)
                and cleanup_passed
            ),
            "failed_scenarios": [
                result["scenario"] for result in results if not result["passed"]
            ],
            "cleanup_passed": cleanup_passed,
            "cleanup_failures": list(self.last_cleanup_failures),
        }
        print("SUMMARY_JSON " + json.dumps(
            final, sort_keys=True, separators=(",", ":")
        ), flush=True)
        return 0 if final["all_passed"] else 1


def choose_cases(args):
    by_name = {item["name"]: item for item in SCENARIOS}
    if args.scenario:
        unknown = sorted(set(args.scenario) - set(by_name))
        if unknown:
            raise ValueError("unknown scenario(s): " + ", ".join(unknown))
        return [by_name[name] for name in args.scenario]
    if args.group in {"smoke", "quick"}:
        return [item for item in SCENARIOS if item["name"] in SMOKE_NAMES]
    if args.group == "formations":
        return [item for item in SCENARIOS if item["behavior"] == "formation"]
    if args.group == "follow":
        return [item for item in SCENARIOS if item["behavior"] == "follow"]
    if args.group == "transport":
        return [item for item in SCENARIOS if item["behavior"] == "transport"]
    return list(SCENARIOS)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run visible RobotSwarm Gazebo acceptance scenarios through /swarm/commands."
    )
    parser.add_argument("--group", default="smoke",
                        choices=["smoke", "quick", "formations", "follow", "transport", "full"])
    parser.add_argument("--scenario", action="append",
                        help="run a named scenario; repeat to build a custom matrix")
    parser.add_argument(
        "--task-id",
        help=(
            "use an explicit correlated task id; requires exactly one "
            "selected scenario"
        ),
    )
    parser.add_argument(
        "--formation-active-seconds",
        type=float,
        default=0.0,
        help=(
            "run one selected formation in moving mode for a bounded active "
            "window; zero keeps the normal static acceptance behavior"
        ),
    )
    parser.add_argument(
        "--follow-active-seconds",
        type=float,
        default=0.0,
        help=(
            "keep one selected follow task active for a bounded wall-time "
            "window after dispatch; zero keeps the normal lap-only behavior"
        ),
    )
    parser.add_argument("--list", action="store_true", help="list scenario names and exit")
    parser.add_argument("--min-rtf", type=float, default=2.90)
    parser.add_argument("--min-center-distance", type=float, default=0.24)
    parser.add_argument(
        "--min-obstacle-clearance", type=float, default=0.13,
        help=(
            "minimum robot-center clearance from a static obstacle in "
            "metres (default: 0.13, just outside Burger contact geometry)"
        ),
    )
    parser.add_argument("--min-boundary-clearance", type=float, default=0.10)
    parser.add_argument("--max-speed", type=float, default=0.35)
    parser.add_argument(
        "--max-acceleration", type=float, default=1.0,
        help="maximum ground-truth linear acceleration in m/s^2",
    )
    parser.add_argument(
        "--acceleration-window", type=float, default=0.06,
        help=(
            "minimum simulation-time window used for the acceleration "
            "comfort gate; raw one-frame impulses are still reported"
        ),
    )
    parser.add_argument(
        "--max-formation-error", type=float, default=0.12,
        help="maximum reported and measured final formation error in metres",
    )
    parser.add_argument("--min-follow-travel", type=float, default=0.25)
    parser.add_argument(
        "--max-follow-spacing-error", type=float, default=0.25,
        help="maximum final centre-spacing error for any adjacent chain pair",
    )
    parser.add_argument(
        "--transport-direct-contact-clearance", type=float, default=0.075,
        help="maximum robot-center clearance from the payload face (metres)",
    )
    parser.add_argument(
        "--transport-companion-contact-distance", type=float, default=0.16,
        help="maximum declared child/predecessor centre distance (metres)",
    )
    parser.add_argument(
        "--min-transport-chain-center-distance", type=float, default=0.12,
        help="minimum declared child/predecessor centre distance (metres)",
    )
    parser.add_argument(
        "--transport-cmd-min-speed", type=float, default=0.015,
        help="minimum forward cmd_vel speed that counts as push intent",
    )
    parser.add_argument(
        "--transport-cmd-min-goal-cosine", type=float, default=0.50,
        help="minimum alignment of robot heading with the payload goal",
    )
    parser.add_argument(
        "--transport-cmd-max-age", type=float, default=0.75,
        help="maximum wall-clock age of a cmd_vel sample (seconds)",
    )
    parser.add_argument(
        "--transport-contribution-noise-floor", type=float, default=0.003,
        help=(
            "minimum role-directed command speed that can count as a "
            "transport contribution"
        ),
    )
    parser.add_argument(
        "--transport-contribution-speed-tolerance", type=float, default=0.003,
        help=(
            "allowed command-speed lag behind the payload or parent goal "
            "velocity"
        ),
    )
    parser.add_argument(
        "--transport-contribution-tracking-fraction", type=float,
        default=0.75,
        help=(
            "minimum fraction of payload or parent goal velocity that a "
            "connected forward command must track"
        ),
    )
    parser.add_argument(
        "--transport-control-gap-tolerance", type=float, default=1.0,
        help=(
            "largest simulated-time gap between coherent control batches "
            "that can belong to one continuous push interval"
        ),
    )
    parser.add_argument(
        "--min-transport-push-duration", type=float, default=0.75,
        help="continuous simulated seconds each robot must usefully push",
    )
    parser.add_argument(
        "--min-transport-push-samples", type=int, default=5,
        help="minimum samples in each robot's longest useful push interval",
    )
    parser.add_argument(
        "--min-transport-useful-fraction", type=float, default=0.50,
        help="minimum fraction of PUSH each robot must contribute usefully",
    )
    parser.add_argument(
        "--min-transport-goal-efficiency", type=float, default=0.50,
        help="minimum goal-distance reduction divided by payload path length",
    )
    parser.add_argument(
        "--min-transport-search-travel", type=float, default=0.05,
        help=(
            "minimum ground-truth path length each robot must cover during "
            "active SEARCH"
        ),
    )
    parser.add_argument(
        "--min-transport-rendezvous-travel", type=float, default=0.10,
        help=(
            "minimum post-notice path length each notified robot must travel "
            "before PUSH"
        ),
    )
    parser.add_argument(
        "--min-transport-root-rendezvous-travel", type=float, default=0.03,
        help=(
            "smaller post-notice path requirement for payload-push roots "
            "that begin beside the object"
        ),
    )
    parser.add_argument(
        "--transport-motion-detection-distance", type=float, default=0.01,
        help=(
            "post-notice path length that marks a robot's first movement "
            "for response-latency reporting"
        ),
    )
    parser.add_argument(
        "--transport-rendezvous-moving-speed", type=float, default=0.02,
        help=(
            "ground-truth speed used to count simultaneous search and "
            "rendezvous movers"
        ),
    )
    parser.add_argument(
        "--max-transport-first-all-useful-wall-time", type=float,
        default=0.0,
        help=(
            "maximum wall seconds from task dispatch to the first coherent "
            "control batch where every robot contributes; zero disables it"
        ),
    )
    parser.add_argument("--delete-after", action="store_true",
                        help="delete the final fleet after the matrix")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.list:
        for item in SCENARIOS:
            print("{:<26} {:<10} n={:<2} spawn={}".format(
                item["name"], item["behavior"], item["count"], item["pattern"]
            ))
        return 0
    try:
        cases = choose_cases(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.task_id and len(cases) != 1:
        parser.error("--task-id requires exactly one selected scenario")
    try:
        args.formation_active_seconds = validate_formation_active_selection(
            args.formation_active_seconds, cases
        )
        args.follow_active_seconds = validate_follow_active_selection(
            args.follow_active_seconds, cases
        )
    except ValueError as exc:
        parser.error(str(exc))

    rospy.init_node("robotswarm_live_acceptance", anonymous=True,
                    disable_signals=True)
    harness = AcceptanceHarness(args)

    def request_stop(_signum, _frame):
        harness.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        return harness.run(cases)
    except Exception as exc:
        cleanup_failures = list(getattr(
            harness, "last_cleanup_failures", []
        ))
        print("SUMMARY_JSON " + json.dumps({
            "selected": len(cases), "executed": 0, "passed": 0,
            "failed": len(cases), "all_passed": False,
            "harness_error": "{}: {}".format(type(exc).__name__, exc),
            "cleanup_passed": not cleanup_failures,
            "cleanup_failures": cleanup_failures,
        }, sort_keys=True, separators=(",", ":")), flush=True)
        return 2
    finally:
        try:
            harness.stop_task(continue_after_stop=True)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
