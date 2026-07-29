#!/usr/bin/env python3

import importlib.util
import itertools
import json
import math
import pathlib
import random
import sys
import threading
import time
import types
import unittest
from unittest import mock

import numpy as np


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"


class Message:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class Vector3(Message):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x=x, y=y, z=z)


class Point(Vector3):
    pass


class Quaternion(Message):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        super().__init__(x=x, y=y, z=z, w=w)


class Pose(Message):
    def __init__(self):
        super().__init__(position=Point(), orientation=Quaternion())


class Twist(Message):
    def __init__(self):
        super().__init__(linear=Vector3(), angular=Vector3())


class Odometry(Message):
    def __init__(self):
        super().__init__(
            pose=Message(pose=Pose()),
            twist=Message(twist=Twist()),
        )


class LaserScan(Message):
    def __init__(self):
        super().__init__(
            ranges=[],
            angle_min=0.0,
            angle_max=0.0,
            angle_increment=0.0,
            range_min=0.0,
            range_max=3.5,
        )


class String(Message):
    def __init__(self, data=""):
        super().__init__(data=data)


class Bool(Message):
    def __init__(self, data=False):
        super().__init__(data=data)


class Float32(Message):
    def __init__(self, data=0.0):
        super().__init__(data=data)


class Float64(Message):
    def __init__(self, data=0.0):
        super().__init__(data=data)


class Empty(Message):
    pass


class Marker(Message):
    LINE_STRIP = 4
    SPHERE = 2
    CYLINDER = 3
    ARROW = 0
    ADD = 0

    def __init__(self):
        super().__init__(
            header=Message(frame_id="", stamp=None),
            pose=Pose(),
            scale=Vector3(),
            color=Message(r=0.0, g=0.0, b=0.0, a=0.0),
            points=[],
            lifetime=None,
        )


class MarkerArray(Message):
    def __init__(self):
        super().__init__(markers=[])


class ModelStates(Message):
    def __init__(self):
        super().__init__(name=[], pose=[])


class ModelState(Message):
    def __init__(self):
        super().__init__(
            model_name="",
            reference_frame="",
            pose=Pose(),
            twist=Twist(),
        )


class FakePublisher:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.messages = []
        self.unregistered = False
        self.connections = 1

    def publish(self, message):
        self.messages.append(message)

    def unregister(self):
        self.unregistered = True

    def get_num_connections(self):
        return self.connections


class BlockingPublisher(FakePublisher):
    def __init__(self, release):
        super().__init__()
        self.release = release
        self.entered = threading.Event()
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            self.calls += 1
        self.entered.set()
        self.release.wait(1.0)
        super().publish(message)


class FirstPublishBlockingPublisher(FakePublisher):
    """Hold the first command while later safety commands remain writable."""

    def __init__(self, release):
        super().__init__()
        self.release = release
        self.entered = threading.Event()
        self.calls = 0
        self.calls_lock = threading.Lock()

    def publish(self, message):
        with self.calls_lock:
            self.calls += 1
            call_number = self.calls
        if call_number == 1:
            self.entered.set()
            self.release.wait(1.0)
        super().publish(message)


class BlockingMotionWithFailedLateZeros(FakePublisher):
    """Fail immediate zeros after a delayed motion, then accept a retry."""

    def __init__(self, release, failed_late_zeros=2):
        super().__init__()
        self.release = release
        self.entered = threading.Event()
        self.failed_late_zeros = failed_late_zeros
        self.failed_zero_count = 0
        self.motion_started = False
        self.motion_returned = False
        self.lock = threading.Lock()

    def publish(self, message):
        is_motion = (
            getattr(message.linear, 'x', 0.0) != 0.0
            or getattr(message.angular, 'z', 0.0) != 0.0
        )
        with self.lock:
            block_motion = is_motion and not self.motion_started
            if block_motion:
                self.motion_started = True

        if block_motion:
            self.entered.set()
            self.release.wait(1.0)
            super().publish(message)
            with self.lock:
                self.motion_returned = True
            return

        with self.lock:
            fail_zero = (
                not is_motion
                and self.motion_returned
                and self.failed_zero_count < self.failed_late_zeros
            )
            if fail_zero:
                self.failed_zero_count += 1
        if fail_zero:
            raise RuntimeError('late zero failed before delivery')
        super().publish(message)


class BlockingFalsePublisher(FakePublisher):
    """Hold only an emergency reset, while later True messages stay writable."""

    def __init__(self, release):
        super().__init__()
        self.release = release
        self.entered = threading.Event()

    def publish(self, message):
        if getattr(message, 'data', None) is False:
            self.entered.set()
            self.release.wait(1.0)
        super().publish(message)


class BlockingTruePublisher(FakePublisher):
    """Hold an emergency latch so reset ordering can be tested."""

    def __init__(self, release):
        super().__init__()
        self.release = release
        self.entered = threading.Event()

    def publish(self, message):
        if getattr(message, 'data', None) is True:
            self.entered.set()
            self.release.wait(1.0)
        super().publish(message)


class FailOncePublisher(FakePublisher):
    """Raise on the first publish and accept the next one."""

    def __init__(self):
        super().__init__()
        self.calls = 0
        self.first_failure_returned = threading.Event()

    def publish(self, message):
        self.calls += 1
        if self.calls == 1:
            self.first_failure_returned.set()
            raise RuntimeError('temporary publish failure')
        super().publish(message)


class RecordThenRaisePublisher(FakePublisher):
    """Model a send that may reach its peer before the local call fails."""

    def publish(self, message):
        super().publish(message)
        raise RuntimeError('delivery outcome is unknown')


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


class FakeSubscriber:
    def __init__(self, *args, **kwargs):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class FakeTimer:
    def __init__(self, *args, **kwargs):
        pass


class FakeAvoidance:
    def __init__(self):
        self.shutdown_called = False
        self.reset_motion_count = 0

    def shutdown(self):
        self.shutdown_called = True

    def update_robot_positions(self, _positions):
        pass

    def apply_avoidance(self, command, *args, **kwargs):
        return command

    def reset_motion(self):
        self.reset_motion_count += 1


class FakeTrace:
    def __init__(self):
        self.clear_count = 0

    def clear(self):
        self.clear_count += 1


class ExitHookLock:
    """RLock that runs a hook after a selected outer context exits."""

    def __init__(self, exit_number, hook):
        self._lock = threading.RLock()
        self._exit_number = exit_number
        self._hook = hook
        self._exit_count = 0

    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self._lock.release()
        self._exit_count += 1
        if self._exit_count == self._exit_number:
            self._hook()


class WaiterAwareRLock:
    """RLock exposing when another operation has started to acquire it."""

    def __init__(self):
        self._lock = threading.RLock()
        self.acquire_attempted = threading.Event()

    def acquire(self, *args, **kwargs):
        self.acquire_attempted.set()
        return self._lock.acquire(*args, **kwargs)

    def release(self):
        self._lock.release()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.release()


def module(name, **attributes):
    result = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(result, key, value)
    return result


def load_script(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def load_ros_scripts():
    rospy = module(
        "rospy",
        Publisher=FakePublisher,
        Subscriber=FakeSubscriber,
        Timer=FakeTimer,
        Duration=lambda value: value,
        Time=Message(now=lambda: 0.0),
        ROSInterruptException=RuntimeError,
        get_param=lambda _name, default=None: default,
        init_node=lambda *args, **kwargs: None,
        on_shutdown=lambda *args, **kwargs: None,
        spin=lambda: None,
        sleep=lambda _duration: None,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
        logwarn_throttle=lambda *args, **kwargs: None,
        logerr_throttle=lambda *args, **kwargs: None,
    )
    geometry_msgs_msg = module(
        "geometry_msgs.msg",
        Twist=Twist,
        Point=Point,
        Pose=Pose,
        Quaternion=Quaternion,
    )
    nav_msgs_msg = module("nav_msgs.msg", Odometry=Odometry)
    sensor_msgs_msg = module("sensor_msgs.msg", LaserScan=LaserScan)
    std_msgs_msg = module(
        "std_msgs.msg",
        String=String,
        Bool=Bool,
        Float32=Float32,
        Float64=Float64,
        Empty=Empty,
    )
    visualization_msgs_msg = module(
        "visualization_msgs.msg",
        Marker=Marker,
        MarkerArray=MarkerArray,
    )
    gazebo_msgs_msg = module(
        "gazebo_msgs.msg",
        ModelState=ModelState,
        ModelStates=ModelStates,
    )
    replacements = {
        "rospy": rospy,
        "geometry_msgs": module(
            "geometry_msgs", msg=geometry_msgs_msg
        ),
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": module("nav_msgs", msg=nav_msgs_msg),
        "nav_msgs.msg": nav_msgs_msg,
        "sensor_msgs": module("sensor_msgs", msg=sensor_msgs_msg),
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": module("std_msgs", msg=std_msgs_msg),
        "std_msgs.msg": std_msgs_msg,
        "visualization_msgs": module(
            "visualization_msgs", msg=visualization_msgs_msg
        ),
        "visualization_msgs.msg": visualization_msgs_msg,
        "gazebo_msgs": module("gazebo_msgs", msg=gazebo_msgs_msg),
        "gazebo_msgs.msg": gazebo_msgs_msg,
    }
    previous = {
        name: sys.modules.get(name)
        for name in replacements
    }
    sys.modules.update(replacements)

    core = module("core")
    core.__path__ = []
    core_previous = sys.modules.get("core")
    obstacle_previous = sys.modules.get("core.obstacle_avoidance")
    try:
        obstacle = load_script(
            "obstacle_avoidance_under_test",
            SCRIPTS / "core" / "obstacle_avoidance.py",
        )
        core.obstacle_avoidance = obstacle
        sys.modules["core"] = core
        sys.modules["core.obstacle_avoidance"] = obstacle

        loaded = {
            "rospy": rospy,
            "obstacle": obstacle,
            "orchestrator": load_script(
                "task_orchestrator_under_test",
                SCRIPTS / "core" / "task_orchestrator.py",
            ),
            "follow": load_script(
                "follow_leader_under_test",
                SCRIPTS / "behaviors" / "follow_leader.py",
            ),
            "transport": load_script(
                "collaborative_transport_under_test",
                SCRIPTS / "behaviors" / "collaborative_transport.py",
            ),
        }
        return loaded
    finally:
        if core_previous is None:
            sys.modules.pop("core", None)
        else:
            sys.modules["core"] = core_previous
        if obstacle_previous is None:
            sys.modules.pop("core.obstacle_avoidance", None)
        else:
            sys.modules["core.obstacle_avoidance"] = obstacle_previous
        for name, original in previous.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


ROS = load_ros_scripts()


def lifecycle_payload(task_id):
    return String(data=json.dumps({"task_id": task_id}))


def transport_collision_event(
    sequence,
    task_id="task-a",
    phase="APPROACH",
    source_id="transport-source-a",
    control_sequence=1,
):
    return {
        "sequence": sequence,
        "source_sequence": sequence,
        "source_id": source_id,
        "robot_id": "tb3_0",
        "task_id": task_id,
        "task_type": "transport",
        "task_phase": phase,
        "control_sequence": control_sequence,
        "sim_time": float(control_sequence),
        "wall_time": float(control_sequence),
        "valid": True,
    }


def transport_collision_stream(
    task_id="task-a",
    phase="APPROACH",
    events=(),
    source_id="transport-source-a",
    task_start_sequence=0,
    watermark=None,
):
    events = list(events)
    if watermark is None:
        watermark = (
            events[-1]["sequence"] if events else task_start_sequence
        )
    terminal = phase in ("DONE", "FAILED")
    stream = {
        "version": 2,
        "source_id": source_id,
        "task_id": task_id,
        "task_start_sequence": task_start_sequence,
        "history_limit": 128,
        "first_sequence": (
            events[0]["sequence"] if events else watermark + 1
        ),
        "last_sequence": watermark,
        "watermark": watermark,
        "terminal": terminal,
        "valid": True,
        "protocol_errors": [],
        "events": events,
    }
    if terminal:
        stream["terminal_watermark"] = watermark
    return stream


def make_orchestrator(task_id="task-a", heartbeat_ready=True):
    orchestrator = ROS["orchestrator"].TaskOrchestrator.__new__(
        ROS["orchestrator"].TaskOrchestrator
    )
    orchestrator.current_task_id = task_id
    orchestrator.current_task_type = "follow_leader"
    orchestrator.current_task_config = {
        "leader_mode": "circular",
        "waypoints": [],
        "radius": 2.0,
        "follow_distance": 0.7,
        "task_id": task_id,
    }
    orchestrator.task_state = ROS["orchestrator"].TaskState.STOPPED
    orchestrator.task_progress = 0.0
    orchestrator.task_result = None
    orchestrator.task_error = None
    orchestrator.task_dispatched = False
    orchestrator.task_ever_dispatched = False
    orchestrator.task_lock = threading.RLock()
    orchestrator.emergency_stop_active = False
    orchestrator.supervised_stop_active = False
    orchestrator.supervised_stop_context = None
    orchestrator._supervised_stop_state_started = False
    orchestrator._pending_safety_stop_acks = {}
    orchestrator._safety_ack_publish_inflight = set()
    orchestrator._safety_ack_retry_requested = set()
    orchestrator._shutdown_started = False
    orchestrator._safety_zero_lock = threading.Lock()
    orchestrator._emergency_order_condition = threading.Condition()
    orchestrator._emergency_order_generation = 0
    orchestrator._pending_emergency_true_generations = set()
    orchestrator._safety_zero_inflight = set()
    orchestrator._safety_zero_followups = set()
    orchestrator._safety_zero_receipts = {}
    orchestrator._retired_safety_publishers = {}
    orchestrator.ordinary_stop_active = False
    orchestrator._ordinary_stop_sequence = 0
    orchestrator._ordinary_stop_operations = {}
    orchestrator._emergency_publication_debts = {}
    orchestrator._emergency_publication_inflight = set()
    orchestrator._safety_fallback_lanes = {}
    orchestrator._safety_fallback_lane_factory = FakeSafetyLane
    orchestrator.robots = {
        "tb3_0": {
            "pose": Pose(),
            "velocity": Twist(),
            "status": "active",
            "threat_level": 0.0,
            "collision_active": False,
            "last_threat_at": None,
            "last_collision_at": None,
            "last_odom_at": 0.0,
            "last_scan_at": 0.0,
            "subscribed_at": 0.0,
            "role": "follower",
        }
    }
    orchestrator.robot_sensor_data = {"tb3_0": {}}
    orchestrator.robot_count = 1
    orchestrator.collision_count = 0
    orchestrator.collision_event_sequence = 0
    orchestrator.collision_events = ROS["orchestrator"].deque(
        maxlen=ROS["orchestrator"].COLLISION_EVENT_HISTORY_LIMIT
    )
    orchestrator.safety_status_timeout = 1.0
    orchestrator.emergency_stop_pub = FakePublisher()
    orchestrator.safety_stop_ack_pub = FakePublisher()
    orchestrator.odom_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.threat_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.collision_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.scan_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.cmd_vel_pubs = {"tb3_0": FakePublisher()}
    orchestrator.leader_cmd_pub = FakePublisher()
    orchestrator._refresh_safety_publisher_snapshot()
    orchestrator.behavior_start_pubs = {
        "follow_leader": FakePublisher(),
        "formation": FakePublisher(),
        "transport": FakePublisher(),
    }
    orchestrator.behavior_stop_pubs = [
        FakePublisher(), FakePublisher(), FakePublisher()
    ]
    orchestrator._register_safety_fallback_lane(
        'safety-stop-ack', orchestrator.safety_stop_ack_pub
    )
    orchestrator.behavior_pause_pubs = {}
    orchestrator.behavior_resume_pubs = {}
    orchestrator._stop_all_robots = lambda: {
        'requested_count': 0,
        'scheduled_count': 0,
        'scheduled_publisher_ids': [],
        'failed_robots': [],
        'scheduling_confirmed': True,
    }
    orchestrator.control_watchdog_enabled = True
    orchestrator.behavior_connection_timeout = 0.1
    orchestrator.behavior_status_timeout = 3.0
    orchestrator.formation_planning_status_timeout = 30.0
    orchestrator.task_dispatched_at = None
    orchestrator.last_behavior_status_at = None
    orchestrator.last_behavior_status = None
    orchestrator.control_heartbeat_timeout = 10.0
    orchestrator.control_heartbeat_max_future = 15.0
    orchestrator.control_watchdog_startup_grace = 15.0
    orchestrator.control_heartbeat_seen = heartbeat_ready
    orchestrator.last_control_heartbeat = (
        0.0 if heartbeat_ready else None
    )
    orchestrator.last_control_heartbeat_deadline = (
        14.0 if heartbeat_ready else None
    )
    orchestrator.control_watchdog_started_at = 0.0
    orchestrator.control_watchdog_tripped = False
    orchestrator._control_clock = lambda: 0.0
    orchestrator._control_watchdog_stop = threading.Event()
    orchestrator._control_watchdog_thread = None
    return orchestrator


class ObstacleAvoidanceSafetyTests(unittest.TestCase):
    @staticmethod
    def fresh_avoidance():
        avoidance = ROS["obstacle"].ObstacleAvoidance("tb3_0")
        avoidance._scan_received_at = avoidance._clock()
        return avoidance

    def test_fast_turn_can_translate_away_from_rear_emergency(self):
        avoidance = self.fresh_avoidance()
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        avoidance.sector_min[4] = 0.1

        desired = Twist()
        desired.linear.x = 0.1
        desired.angular.z = 1.0
        safe = avoidance.apply_avoidance(desired)

        self.assertGreater(safe.linear.x, 0.0)
        self.assertGreater(safe.angular.z, 0.0)

    def test_fast_turn_still_stops_for_front_emergency(self):
        avoidance = self.fresh_avoidance()
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        avoidance.sector_min[0] = 0.1

        desired = Twist()
        desired.linear.x = 0.1
        desired.angular.z = 1.0
        safe = avoidance.apply_avoidance(desired)

        self.assertEqual(0.0, safe.linear.x)
        self.assertEqual(0.0, safe.angular.z)

    def test_circular_burger_can_rotate_in_place_near_wall(self):
        avoidance = self.fresh_avoidance()
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        avoidance.sector_min[0] = 0.1

        desired = Twist()
        desired.angular.z = -1.0
        safe = avoidance.apply_avoidance(desired)

        self.assertEqual(0.0, safe.linear.x)
        self.assertLess(safe.angular.z, 0.0)


class OrchestratorLifecycleTests(unittest.TestCase):
    def test_delete_command_forwards_the_optional_request_id(self):
        orchestrator = make_orchestrator()
        orchestrator.fleet_delete_pub = FakePublisher()

        orchestrator._handle_delete_robots({
            "request_id": "cleanup-42",
        })

        command = json.loads(orchestrator.fleet_delete_pub.messages[-1].data)
        self.assertTrue(command["all"])
        self.assertEqual("cleanup-42", command["request_id"])

    def test_invalid_task_config_is_correlated_and_never_dispatched(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS["orchestrator"].TaskState.IDLE

        orchestrator._handle_start_task({
            "task_id": "bad-transport",
            "task_type": "transport",
            "target_x": "NaN",
            "target_y": 1.0,
        })

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual("bad-transport", orchestrator.current_task_id)
        self.assertIn("target_x must be finite", orchestrator.task_error)
        self.assertFalse(orchestrator.task_dispatched)

    def test_waypoint_mode_rejects_malformed_coordinates(self):
        with self.assertRaisesRegex(ValueError, "waypoint 0 y must be a number"):
            ROS["orchestrator"].TaskOrchestrator._validated_task_config(
                "follow_leader",
                {"leader_mode": "waypoint"},
                {"waypoints": [{"x": 1.0}]},
            )

    def test_valid_task_config_is_normalized_before_dispatch(self):
        config = ROS["orchestrator"].TaskOrchestrator._validated_task_config(
            "formation",
            {"formation_type": "a", "movement_mode": "STATIC"},
            {"spacing": "0.7"},
        )

        self.assertEqual({
            "formation_type": "A",
            "movement_mode": "static",
            "spacing": 0.7,
        }, config)

    def test_transport_arrival_margin_is_explicit_and_bounded(self):
        config = ROS["orchestrator"].TaskOrchestrator._validated_task_config(
            "transport",
            {
                "target_x": "-2.5",
                "target_y": 1.25,
                "arrival_tolerance": "0.25",
            },
            {"transport_planner": "grf"},
        )

        self.assertEqual({
            "target_x": -2.5,
            "target_y": 1.25,
            "arrival_tolerance": 0.25,
            "transport_planner": "grf",
        }, config)
        with self.assertRaisesRegex(
            ValueError, "arrival_tolerance must be between"
        ):
            ROS["orchestrator"].TaskOrchestrator._validated_task_config(
                "transport",
                {"arrival_tolerance": 0.10},
                {},
            )

    def test_collision_count_uses_contact_state_not_emergency_threat(self):
        orchestrator = make_orchestrator()

        orchestrator._threat_cb("tb3_0", Float32(data=1.0))
        self.assertEqual(0, orchestrator.collision_count)

        orchestrator._collision_cb("tb3_0", Bool(data=True))
        orchestrator._collision_cb("tb3_0", Bool(data=True))
        self.assertEqual(1, orchestrator.collision_count)

        orchestrator._collision_cb("tb3_0", Bool(data=False))
        orchestrator._collision_cb("tb3_0", Bool(data=True))
        self.assertEqual(2, orchestrator.collision_count)

    def test_transport_source_keeps_edge_phase_before_delayed_callback(self):
        task_id = "transport-1"
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.robot_namespaces = ["tb3_0"]
        controller._ensure_transport_collision_stream()
        controller._begin_transport_collision_stream(task_id)

        avoidance = ROS["obstacle"].ObstacleAvoidance(
            "tb3_0",
            collision_edge_callback=(
                controller._record_transport_collision_edge
            ),
        )
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        avoidance.sector_min[0] = 0.10
        avoidance.publish_safety_state(collision_context={
            "task_id": task_id,
            "task_phase": "APPROACH",
            "control_sequence": 17,
            "sim_time": 4.2,
            "wall_time": 8.4,
        })

        # The behavior advances before either the Bool or status subscriber is
        # scheduled. The source event must retain the phase of the OA edge.
        source_stream = controller._transport_collision_stream_snapshot(
            task_id, terminal=False
        )
        orchestrator = make_orchestrator(task_id)
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator._collision_cb("tb3_0", Bool(data=True))
        self.assertEqual(0, orchestrator.collision_count)
        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": task_id,
                "phase": "PUSH",
                "collision_events": source_stream,
            })),
        )

        status = orchestrator._build_status()
        event = status["collision_events"]["events"][0]
        self.assertEqual("APPROACH", event["task_phase"])
        self.assertEqual(17, event["source_control_sequence"])
        self.assertEqual(1, status["collisions"])
        self.assertEqual(
            "PUSH", status["task"]["result"]["transport"]["phase"]
        )

        terminal_stream = controller._transport_collision_stream_snapshot(
            task_id, terminal=True
        )
        self.assertEqual(
            terminal_stream["watermark"],
            terminal_stream["terminal_watermark"],
        )
        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": task_id,
                "phase": "DONE",
                "collision_events": terminal_stream,
            })),
        )
        self.assertEqual(
            ROS["orchestrator"].TaskState.COMPLETED,
            orchestrator.task_state,
        )
        self.assertEqual(1, orchestrator.collision_count)

    def test_collision_event_history_is_bounded_and_contiguous(self):
        orchestrator = make_orchestrator()
        limit = ROS["orchestrator"].COLLISION_EVENT_HISTORY_LIMIT

        for _ in range(limit + 3):
            orchestrator._collision_cb("tb3_0", Bool(data=True))
            orchestrator._collision_cb("tb3_0", Bool(data=False))

        stream = orchestrator._build_status()["collision_events"]
        self.assertEqual(limit, len(stream["events"]))
        self.assertEqual(4, stream["first_sequence"])
        self.assertEqual(limit + 3, stream["last_sequence"])
        self.assertEqual(
            list(range(4, limit + 4)),
            [event["sequence"] for event in stream["events"]],
        )

    def test_transport_collision_source_gap_fails_closed(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        event = transport_collision_event(2)

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "APPROACH",
                "collision_events": transport_collision_stream(
                    events=[event], watermark=2
                ),
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertIn("dropped", orchestrator.task_error)
        self.assertEqual(0, orchestrator.collision_count)

    def test_transport_collision_source_restart_fails_closed(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING

        for source_id in ("transport-source-a", "transport-source-b"):
            orchestrator._behavior_status_callback(
                "transport",
                String(data=json.dumps({
                    "task_id": "task-a",
                    "phase": "SEARCH",
                    "collision_events": transport_collision_stream(
                        phase="SEARCH", source_id=source_id
                    ),
                })),
            )

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertIn("restarted", orchestrator.task_error)

    def test_inactive_safety_feed_expires_from_status(self):
        orchestrator = make_orchestrator()
        clock = [10.0]
        orchestrator._control_clock = lambda: clock[0]
        orchestrator._threat_cb("tb3_0", Float32(data=0.9))
        orchestrator._collision_cb("tb3_0", Bool(data=True))
        orchestrator.robots["tb3_0"]["last_odom_at"] = clock[0]
        orchestrator.robots["tb3_0"]["last_scan_at"] = clock[0]

        active = orchestrator._build_status()["robots"][0]
        self.assertEqual(0.9, active["threat_level"])
        self.assertTrue(active["collision"])
        self.assertFalse(active["safety_stale"])

        clock[0] = 11.1
        stale = orchestrator._build_status()["robots"][0]
        self.assertEqual(1.0, stale["threat_level"])
        self.assertTrue(stale["collision"])
        self.assertTrue(stale["safety_stale"])
        self.assertEqual(
            ["odometry", "lidar", "threat", "collision"],
            stale["safety_stale_inputs"],
        )

    def test_runtime_health_fails_on_stale_behavior_heartbeat(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched_at = 0.0
        orchestrator.last_behavior_status_at = 0.0
        orchestrator._control_clock = lambda: 3.1

        task_id, error = orchestrator._runtime_health_error()

        self.assertEqual("task-a", task_id)
        self.assertIn("Behavior status heartbeat became stale", error)

    def test_runtime_health_allows_bounded_large_formation_planning(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "formation"
        orchestrator.robot_count = 10
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched_at = 0.0
        orchestrator.last_behavior_status_at = 0.0
        orchestrator.last_behavior_status = {
            "task_id": "task-a",
            "state": "forming",
            "robot_assignments": {},
        }
        orchestrator._control_clock = lambda: 11.0
        for robot in orchestrator.robots.values():
            robot["last_odom_at"] = 11.0
            robot["last_scan_at"] = 11.0

        _, error = orchestrator._runtime_health_error()

        self.assertIsNone(error)

        orchestrator._control_clock = lambda: 20.1
        _, error = orchestrator._runtime_health_error()
        self.assertIn("Behavior status heartbeat became stale", error)

    def test_runtime_health_keeps_short_timeout_after_assignment(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "formation"
        orchestrator.robot_count = 10
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched_at = 0.0
        orchestrator.last_behavior_status_at = 0.0
        orchestrator.last_behavior_status = {
            "task_id": "task-a",
            "state": "forming",
            "robot_assignments": {"tb3_0": {"slot": 0}},
        }
        orchestrator._control_clock = lambda: 3.1

        _, error = orchestrator._runtime_health_error()

        self.assertIn("Behavior status heartbeat became stale", error)

    def test_runtime_health_does_not_reuse_a_previous_planning_grace(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "formation"
        orchestrator.robot_count = 10
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched_at = 0.0
        orchestrator.last_behavior_status_at = None
        orchestrator.last_behavior_status = {
            "task_id": "previous-task",
            "state": "forming",
            "robot_assignments": {},
        }
        orchestrator._control_clock = lambda: 3.1

        _, error = orchestrator._runtime_health_error()

        self.assertIn("Behavior status heartbeat became stale", error)

    def test_runtime_health_fails_on_stale_odom_or_lidar(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched_at = 0.0
        orchestrator.last_behavior_status_at = 2.0
        orchestrator._control_clock = lambda: 2.0
        orchestrator.robots["tb3_0"]["last_odom_at"] = 0.0
        orchestrator.robots["tb3_0"]["last_scan_at"] = 2.0

        _, error = orchestrator._runtime_health_error()

        self.assertIn("Safety telemetry became stale", error)
        self.assertIn("tb3_0: odometry", error)

    def test_unsafe_formation_status_fails_the_correlated_task(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "formation"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "formation",
            String(data=json.dumps({
                "task_id": "task-a",
                "state": "failed",
                "error": "No collision-free placement fits",
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual(0.0, orchestrator.task_progress)
        self.assertEqual(
            "No collision-free placement fits",
            orchestrator._build_status()["task"]["error"],
        )
        self.assertTrue(all(
            publisher.messages
            for publisher in orchestrator.behavior_stop_pubs
        ))

    def test_invalid_payload_status_fails_the_transport_task(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "FAILED",
                "error": "Transport payload left the supported floor plane",
                "collision_events": transport_collision_stream(
                    phase="FAILED"
                ),
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual(
            "Transport payload left the supported floor plane",
            orchestrator.task_error,
        )

    def test_transport_discovery_is_correlated_and_narrowed(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "APPROACH",
                "progress": 0.2,
                "searching_robot_count": 0,
                "all_pushers_confirmed": True,
                "useful_contributor_count": 3,
                "useful_contributor_ids": ["tb3_2", "tb3_0", "tb3_1"],
                "arrival_tolerance": 0.25,
                "target_marker": {
                    "model_name": "target_marker",
                    "command_published": True,
                    "synchronized": True,
                    "position": {"x": -2.5, "y": 1.25, "z": 0.0},
                    "private_diagnostic": "drop-me",
                },
                "collision_events": transport_collision_stream(),
                "discovery": {
                    "event": "payload_found",
                    "event_id": "task-a:payload-found",
                    "task_id": "task-a",
                    "announced": True,
                    "finder": "tb3_0",
                    "distance": 0.42,
                    "object_position": {"x": 1.25, "y": -0.75},
                    "sim_time": 12.5,
                    "notified_robots": ["tb3_1", "tb3_2"],
                },
                "robot_assignments": {"tb3_0": {"command": "private"}},
            })),
        )

        task = orchestrator._build_status()["task"]
        self.assertEqual({
            "transport": {
                "phase": "APPROACH",
                "searching_robot_count": 0,
                "all_pushers_confirmed": True,
                "useful_contributor_count": 3,
                "useful_contributor_ids": ["tb3_0", "tb3_1", "tb3_2"],
                "arrival_tolerance": 0.25,
                "target_marker": {
                    "model_name": "target_marker",
                    "published": True,
                    "synchronized": True,
                    "position": {"x": -2.5, "y": 1.25},
                },
                "discovery": {
                    "event": "payload_found",
                    "event_id": "task-a:payload-found",
                    "task_id": "task-a",
                    "announced": True,
                    "finder": "tb3_0",
                    "distance": 0.42,
                    "object_position": {"x": 1.25, "y": -0.75},
                    "sim_time": 12.5,
                    "notified_robots": ["tb3_1", "tb3_2"],
                },
            },
        }, task["result"])
        self.assertNotIn("robot_assignments", task["result"]["transport"])

    def test_transport_push_evidence_rejects_malformed_scalars(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "PUSH",
                "all_pushers_confirmed": 1,
                "useful_contributor_count": True,
                "useful_contributor_ids": ["tb3_0"],
                "arrival_tolerance": "0.25",
                "target_marker": {
                    "model_name": "../../private-model",
                    "command_published": "yes",
                    "synchronized": 1,
                    "position": {"x": "-2.5", "y": 1.25},
                },
                "control_commands": {"tb3_0": {"linear": 0.1}},
                "collision_events": transport_collision_stream(
                    phase="PUSH"
                ),
            })),
        )

        result = orchestrator._build_status()["task"]["result"]["transport"]
        self.assertIsNone(result["all_pushers_confirmed"])
        self.assertIsNone(result["useful_contributor_count"])
        self.assertIsNone(result["useful_contributor_ids"])
        self.assertIsNone(result["arrival_tolerance"])
        self.assertIsNone(result["target_marker"])
        self.assertNotIn("control_commands", result)

    def test_transport_target_marker_failure_stays_optional(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "SEARCH",
                "arrival_tolerance": 0.5,
                "target_marker": {
                    "model_name": "target_marker",
                    "command_published": False,
                    "synchronized": False,
                    "position": None,
                },
                "collision_events": transport_collision_stream(),
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.RUNNING,
            orchestrator.task_state,
        )
        result = orchestrator.task_result["transport"]
        self.assertEqual(0.5, result["arrival_tolerance"])
        self.assertEqual({
            "model_name": "target_marker",
            "published": False,
            "synchronized": False,
            "position": None,
        }, result["target_marker"])

    def test_transport_target_marker_rejects_unbounded_or_incoherent_data(self):
        valid = {
            "model_name": "target_marker",
            "command_published": True,
            "synchronized": True,
            "position": {"x": -2.5, "y": 1.25},
        }
        malformed = [
            {**valid, "model_name": "private-model"},
            {**valid, "command_published": 1},
            {**valid, "synchronized": "yes"},
            {**valid, "position": {"x": "-2.5", "y": 1.25}},
            {**valid, "position": {"x": 4.01, "y": 1.25}},
            {**valid, "position": {"x": float("nan"), "y": 1.25}},
            {**valid, "position": None},
        ]
        sanitize = (
            ROS["orchestrator"].TaskOrchestrator
            ._transport_target_marker
        )

        for marker in malformed:
            with self.subTest(marker=marker):
                self.assertIsNone(sanitize(marker))

    def test_stale_or_nested_uncorrelated_discovery_is_not_exposed(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = "transport"
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_result = {"transport": {"phase": "SEARCH"}}

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "old-task",
                "phase": "APPROACH",
                "searching_robot_count": 0,
                "discovery": None,
            })),
        )
        self.assertEqual(
            {"transport": {"phase": "SEARCH"}},
            orchestrator.task_result,
        )

        orchestrator._behavior_status_callback(
            "transport",
            String(data=json.dumps({
                "task_id": "task-a",
                "phase": "APPROACH",
                "searching_robot_count": 0,
                "collision_events": transport_collision_stream(),
                "discovery": {
                    "event": "payload_found",
                    "event_id": "old-task:payload-found",
                    "task_id": "old-task",
                    "announced": True,
                    "finder": "tb3_9",
                    "distance": 0.1,
                    "object_position": {"x": 0.0, "y": 0.0},
                    "sim_time": 1.0,
                    "notified_robots": ["tb3_9"],
                },
            })),
        )
        self.assertIsNone(
            orchestrator.task_result["transport"]["discovery"]
        )

    def test_new_task_clears_previous_behavior_result(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS["orchestrator"].TaskState.IDLE
        orchestrator.task_result = {
            "transport": {"phase": "APPROACH"},
        }
        orchestrator.collision_count = 7
        orchestrator.collision_event_sequence = 7
        orchestrator.collision_events.append({
            "sequence": 7,
            "robot_id": "tb3_0",
            "task_id": "previous-task",
            "task_type": "transport",
            "task_phase": "APPROACH",
        })

        with mock.patch.object(ROS["orchestrator"].time, "sleep"):
            orchestrator._handle_start_task({
                "task_id": "new-task",
                "task_type": "follow_leader",
            })

        self.assertIsNone(orchestrator.task_result)
        stream = orchestrator._build_status()["collision_events"]
        self.assertEqual(7, stream["last_sequence"])
        self.assertEqual(8, stream["first_sequence"])
        self.assertEqual([], stream["events"])

    def test_invalid_follow_path_status_fails_the_correlated_task(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "follow_leader",
            String(data=json.dumps({
                "task_id": "task-a",
                "state": "failed",
                "error": "The requested leader path does not fit",
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual(0.0, orchestrator.task_progress)
        self.assertEqual(
            "The requested leader path does not fit",
            orchestrator.task_error,
        )
        self.assertTrue(all(
            publisher.messages
            for publisher in orchestrator.behavior_stop_pubs
        ))

    def test_stale_follow_failure_cannot_fail_the_current_task(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_progress = 0.35
        orchestrator.task_dispatched = True

        orchestrator._behavior_status_callback(
            "follow_leader",
            String(data=json.dumps({
                "task_id": "old-task",
                "state": "failed",
                "error": "Failure from an earlier task",
            })),
        )

        self.assertEqual(
            ROS["orchestrator"].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertEqual(0.35, orchestrator.task_progress)
        self.assertIsNone(orchestrator.task_error)
        self.assertFalse(any(
            publisher.messages
            for publisher in orchestrator.behavior_stop_pubs
        ))

    def test_dispatched_same_task_redelivery_is_idempotent(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_progress = 0.42
        orchestrator.task_dispatched = True

        orchestrator._handle_start_task({
            "task_id": "task-a",
            "task_type": "follow_leader",
        })

        self.assertEqual(
            ROS["orchestrator"].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertEqual(0.42, orchestrator.task_progress)
        self.assertFalse(
            orchestrator.behavior_start_pubs["follow_leader"].messages
        )

    def test_invalid_same_task_redelivery_keeps_the_active_task_running(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_progress = 0.42
        orchestrator.task_dispatched = True
        zero_count = len(orchestrator.cmd_vel_pubs['tb3_0'].messages)

        orchestrator._handle_start_task({
            'task_id': 'task-a',
            'task_type': 'follow_leader',
            'config': {'radius': 99.0},
        })

        self.assertEqual(
            ROS['orchestrator'].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertEqual(0.42, orchestrator.task_progress)
        self.assertTrue(orchestrator.task_dispatched)
        self.assertIsNone(orchestrator.task_error)
        self.assertFalse(any(
            publisher.messages
            for publisher in orchestrator.behavior_stop_pubs
        ))
        self.assertEqual(
            zero_count,
            len(orchestrator.cmd_vel_pubs['tb3_0'].messages),
        )

    def test_undispatched_same_task_can_be_redelivered(self):
        orchestrator = make_orchestrator()

        orchestrator._handle_start_task({
            "task_id": "task-a",
            "task_type": "follow_leader",
        })

        self.assertEqual(
            ROS["orchestrator"].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertTrue(orchestrator.task_dispatched)
        self.assertEqual(
            1,
            len(orchestrator.behavior_start_pubs["follow_leader"].messages),
        )

    def test_stale_stop_is_ignored_and_exact_stop_is_correlated(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True

        orchestrator._handle_stop_task({"task_id": "old-task"})
        self.assertEqual(
            ROS["orchestrator"].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertFalse(any(
            pub.messages for pub in orchestrator.behavior_stop_pubs
        ))

        orchestrator._handle_stop_task({"task_id": "task-a"})
        self.assertEqual(
            ROS["orchestrator"].TaskState.STOPPED,
            orchestrator.task_state,
        )
        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual("task-a", payload["task_id"])

    def test_fleet_loss_during_initialization_prevents_dispatch(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS["orchestrator"].TaskState.IDLE
        initialization_started = threading.Event()
        release_initialization = threading.Event()
        original_sleep = ROS["orchestrator"].time.sleep

        def blocked_sleep(_duration):
            initialization_started.set()
            release_initialization.wait(timeout=1.0)

        ROS["orchestrator"].time.sleep = blocked_sleep
        try:
            worker = threading.Thread(
                target=orchestrator._handle_start_task,
                args=({
                    "task_id": "task-b",
                    "task_type": "follow_leader",
                },),
            )
            worker.start()
            self.assertTrue(initialization_started.wait(timeout=1.0))

            orchestrator._robot_list_callback(String(data=""))
            release_initialization.set()
            worker.join(timeout=1.0)
        finally:
            ROS["orchestrator"].time.sleep = original_sleep

        self.assertFalse(worker.is_alive())
        self.assertEqual({}, orchestrator.robots)
        self.assertEqual(
            ROS["orchestrator"].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertFalse(
            orchestrator.behavior_start_pubs["follow_leader"].messages
        )

    def test_missing_behavior_subscriber_fails_instead_of_losing_start(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS["orchestrator"].TaskState.IDLE
        orchestrator.behavior_connection_timeout = 0.001
        publisher = orchestrator.behavior_start_pubs["follow_leader"]
        publisher.connections = 0

        with mock.patch.object(ROS["orchestrator"].time, "sleep"):
            orchestrator._handle_start_task({
                "task_id": "task-without-controller",
                "task_type": "follow_leader",
            })

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertFalse(publisher.messages)
        self.assertIn(
            "did not subscribe",
            orchestrator._build_status()["task"]["error"],
        )


class ControlHeartbeatWatchdogTests(unittest.TestCase):
    @staticmethod
    def wait_until(predicate, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return predicate()

    def assert_ambiguous_lifecycle_is_secured(
        self, orchestrator, task_id, command_publisher
    ):
        self.assertTrue(self.wait_until(lambda: (
            orchestrator.emergency_stop_pub.messages
            and orchestrator.emergency_stop_pub.messages[-1].data is True
            and all(
                publisher.messages
                for publisher in orchestrator.behavior_stop_pubs
            )
            and command_publisher.messages
            and not orchestrator._emergency_publication_debts
            and not orchestrator._safety_zero_inflight
        )))
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertEqual(0.0, command_publisher.messages[-1].linear.x)
        self.assertEqual(0.0, command_publisher.messages[-1].angular.z)
        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual(task_id, payload['task_id'])

    def latch_and_reset(self, orchestrator):
        orchestrator._handle_emergency_stop({})
        self.assertTrue(self.wait_until(lambda: (
            orchestrator.emergency_stop_active
            and not orchestrator._emergency_publication_debts
        )))
        orchestrator._handle_reset_emergency_stop({})
        self.assertFalse(orchestrator.emergency_stop_active)

    def test_normal_stop_fans_out_past_a_blocked_command_socket(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        orchestrator.cmd_vel_pubs = {
            'tb3_0': blocked,
            'tb3_1': healthy,
        }
        orchestrator._refresh_safety_publisher_snapshot()
        orchestrator._stop_all_robots = types.MethodType(
            ROS['orchestrator'].TaskOrchestrator._stop_all_robots,
            orchestrator,
        )

        try:
            self.assertTrue(self.wait_until(lambda: (
                not orchestrator._safety_zero_inflight
            )))
            orchestrator._handle_stop_task({'task_id': 'task-a'})
            self.assertTrue(blocked.entered.wait(0.5))
            self.assertTrue(self.wait_until(lambda: healthy.messages))
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            self.assertEqual(
                ROS['orchestrator'].TaskState.STOPPED,
                orchestrator.task_state,
            )
        finally:
            release_blocked.set()
            self.assertTrue(self.wait_until(lambda: (
                not orchestrator._safety_zero_inflight
            )))

    def test_manual_emergency_preempts_a_blocked_resume(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        orchestrator.behavior_resume_pubs = {'follow_leader': blocked}
        orchestrator.cmd_vel_pubs = {'tb3_0': healthy}
        orchestrator._refresh_safety_publisher_snapshot()
        resume = threading.Thread(
            target=orchestrator._handle_resume_task,
            args=({'task_id': 'task-a'},),
        )
        emergency = threading.Thread(
            target=orchestrator._handle_emergency_stop,
            args=({},),
        )

        resume.start()
        self.assertTrue(blocked.entered.wait(0.5))
        try:
            emergency.start()
            self.assertTrue(self.wait_until(lambda: (
                orchestrator.emergency_stop_active
                and orchestrator.emergency_stop_pub.messages
                and orchestrator.emergency_stop_pub.messages[-1].data is True
                and healthy.messages
            )))
            self.assertTrue(resume.is_alive())
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
        finally:
            release_blocked.set()
            resume.join(1.0)
            emergency.join(1.0)

        self.assertFalse(resume.is_alive())
        self.assertFalse(emergency.is_alive())

    def test_watchdog_preempts_a_blocked_resume(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        orchestrator.control_heartbeat_seen = True
        orchestrator.last_control_heartbeat = 10.0
        orchestrator.control_heartbeat_timeout = 1.0
        clock = [10.0]
        orchestrator._control_clock = lambda: clock[0]
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        orchestrator.behavior_resume_pubs = {'follow_leader': blocked}
        orchestrator.cmd_vel_pubs = {'tb3_0': healthy}
        orchestrator._refresh_safety_publisher_snapshot()
        resume = threading.Thread(
            target=orchestrator._handle_resume_task,
            args=({'task_id': 'task-a'},),
        )
        watchdog = threading.Thread(
            target=orchestrator._check_control_watchdog,
            kwargs={'now': 11.0},
        )

        resume.start()
        self.assertTrue(blocked.entered.wait(0.5))
        try:
            clock[0] = 11.0
            watchdog.start()
            self.assertTrue(self.wait_until(lambda: (
                orchestrator.control_watchdog_tripped
                and orchestrator.emergency_stop_active
                and orchestrator.emergency_stop_pub.messages
                and orchestrator.emergency_stop_pub.messages[-1].data is True
                and healthy.messages
                and all(
                    publisher.messages
                    for publisher in orchestrator.behavior_stop_pubs
                )
                and not orchestrator._emergency_publication_debts
            )))
            self.assertTrue(resume.is_alive())
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            first_stop_counts = [
                len(publisher.messages)
                for publisher in orchestrator.behavior_stop_pubs
            ]
        finally:
            release_blocked.set()
            resume.join(1.0)
            watchdog.join(1.0)

        self.assertFalse(resume.is_alive())
        self.assertFalse(watchdog.is_alive())
        self.assertTrue(self.wait_until(lambda: all(
            len(publisher.messages) > previous
            for publisher, previous in zip(
                orchestrator.behavior_stop_pubs, first_stop_counts
            )
        )))
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)

    def test_resume_rejects_every_nonfresh_heartbeat(self):
        heartbeat_cases = (
            ('missing', False, None, 0.0),
            ('negative-age', True, 1.0, 0.0),
            ('timeout-boundary', True, 0.0, 10.0),
        )

        for label, seen, last_heartbeat, now in heartbeat_cases:
            with self.subTest(case=label):
                orchestrator = make_orchestrator()
                orchestrator.task_state = (
                    ROS['orchestrator'].TaskState.PAUSED
                )
                orchestrator.task_dispatched = True
                orchestrator.control_heartbeat_seen = seen
                orchestrator.last_control_heartbeat = last_heartbeat
                orchestrator._control_clock = lambda value=now: value
                publisher = FakePublisher()
                orchestrator.behavior_resume_pubs = {
                    'follow_leader': publisher,
                }

                orchestrator._handle_resume_task({'task_id': 'task-a'})

                self.assertFalse(publisher.messages)
                self.assertEqual(
                    ROS['orchestrator'].TaskState.PAUSED,
                    orchestrator.task_state,
                )

    def test_start_publish_error_latches_after_a_possible_delivery(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS['orchestrator'].TaskState.IDLE
        ambiguous_publisher = RecordThenRaisePublisher()
        command_publisher = FakePublisher()
        orchestrator.behavior_start_pubs['follow_leader'] = (
            ambiguous_publisher
        )
        orchestrator.cmd_vel_pubs = {'tb3_0': command_publisher}
        orchestrator._refresh_safety_publisher_snapshot()

        with mock.patch.object(ROS['orchestrator'].time, 'sleep'):
            orchestrator._handle_start_task({
                'task_id': 'ambiguous-start',
                'task_type': 'follow_leader',
            })

        self.assertEqual(1, len(ambiguous_publisher.messages))
        delivered = json.loads(ambiguous_publisher.messages[0].data)
        self.assertEqual('ambiguous-start', delivered['task_id'])
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertIn('delivery outcome is unknown', orchestrator.task_error)
        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assertFalse(orchestrator.task_dispatched)
        self.assertTrue(orchestrator.task_ever_dispatched)
        self.assert_ambiguous_lifecycle_is_secured(
            orchestrator, 'ambiguous-start', command_publisher
        )

    def test_resume_publish_error_latches_after_a_possible_delivery(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        ambiguous_publisher = RecordThenRaisePublisher()
        command_publisher = FakePublisher()
        orchestrator.behavior_resume_pubs = {
            'follow_leader': ambiguous_publisher,
        }
        orchestrator.cmd_vel_pubs = {'tb3_0': command_publisher}
        orchestrator._refresh_safety_publisher_snapshot()

        orchestrator._handle_resume_task({'task_id': 'task-a'})

        self.assertEqual(1, len(ambiguous_publisher.messages))
        delivered = json.loads(ambiguous_publisher.messages[0].data)
        self.assertEqual('task-a', delivered['task_id'])
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertIn('delivery outcome is unknown', orchestrator.task_error)
        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assert_ambiguous_lifecycle_is_secured(
            orchestrator, 'task-a', command_publisher
        )

    def test_ambiguous_publish_stops_original_and_new_active_tasks(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_id = 'task-b'
        orchestrator.current_task_config['task_id'] = 'task-b'
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        failing = FailOncePublisher()
        healthy = FakePublisher()
        orchestrator.behavior_stop_pubs = [blocked, failing, healthy]

        try:
            orchestrator._secure_ambiguous_lifecycle_publish(
                'task-a', 'identity-race'
            )
            self.assertTrue(blocked.entered.wait(0.5))
            self.assertTrue(self.wait_until(lambda: (
                blocked.calls >= 2
                and failing.first_failure_returned.is_set()
            )))
            with orchestrator._safety_zero_lock:
                pending_task_ids = {
                    key[1]
                    for key in orchestrator._emergency_publication_debts
                    if key[0] == 'stop'
                }
            self.assertEqual(
                {'task-a', 'task-b'}, pending_task_ids
            )
        finally:
            release_blocked.set()

        deadline = time.monotonic() + 0.5
        while (
            orchestrator._emergency_publication_debts
            and time.monotonic() < deadline
        ):
            orchestrator._safety_stop_zero_timer(None)
            time.sleep(0.005)

        self.assertFalse(orchestrator._emergency_publication_debts)
        self.assertFalse(orchestrator._emergency_publication_inflight)
        for publisher in (blocked, failing, healthy):
            delivered_task_ids = {
                json.loads(message.data)['task_id']
                for message in publisher.messages
            }
            self.assertEqual(
                {'task-a', 'task-b'}, delivered_task_ids
            )
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)
        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assertEqual('task-b', orchestrator.current_task_id)
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)

    def test_start_does_not_publish_after_a_latch_and_reset_race(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS['orchestrator'].TaskState.IDLE
        publisher = orchestrator.behavior_start_pubs['follow_leader']
        orchestrator._safety_zero_lock = ExitHookLock(
            1, lambda: self.latch_and_reset(orchestrator)
        )

        with mock.patch.object(ROS['orchestrator'].time, 'sleep'):
            orchestrator._handle_start_task({
                'task_id': 'start-reset-race',
                'task_type': 'follow_leader',
            })

        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(publisher.messages)
        self.assertFalse(orchestrator.task_dispatched)

    def test_resume_does_not_publish_after_a_latch_and_reset_race(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        publisher = FakePublisher()
        orchestrator.behavior_resume_pubs = {'follow_leader': publisher}
        orchestrator._safety_zero_lock = ExitHookLock(
            1, lambda: self.latch_and_reset(orchestrator)
        )

        orchestrator._handle_resume_task({'task_id': 'task-a'})

        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(publisher.messages)
        self.assertFalse(orchestrator.task_dispatched)

    def test_watchdog_preempts_a_blocked_start_and_compensates_it(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS['orchestrator'].TaskState.IDLE
        clock = [0.0]
        orchestrator._control_clock = lambda: clock[0]
        release_start = threading.Event()
        start_publisher = BlockingPublisher(release_start)
        healthy = FakePublisher()
        orchestrator.behavior_start_pubs['follow_leader'] = start_publisher
        orchestrator.cmd_vel_pubs = {'tb3_0': healthy}
        orchestrator._refresh_safety_publisher_snapshot()
        start = threading.Thread(
            target=orchestrator._handle_start_task,
            args=({
                'task_id': 'watchdog-start-race',
                'task_type': 'follow_leader',
            },),
        )
        watchdog = threading.Thread(
            target=orchestrator._check_control_watchdog,
            kwargs={'now': 10.0},
        )

        start.start()
        self.assertTrue(start_publisher.entered.wait(0.7))
        try:
            clock[0] = 10.0
            watchdog.start()
            self.assertTrue(self.wait_until(lambda: (
                orchestrator.control_watchdog_tripped
                and orchestrator.emergency_stop_active
                and orchestrator.emergency_stop_pub.messages
                and orchestrator.emergency_stop_pub.messages[-1].data is True
                and healthy.messages
                and all(
                    publisher.messages
                    for publisher in orchestrator.behavior_stop_pubs
                )
                and not orchestrator._emergency_publication_debts
            )))
            self.assertTrue(start.is_alive())
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            first_stop_counts = [
                len(publisher.messages)
                for publisher in orchestrator.behavior_stop_pubs
            ]
        finally:
            release_start.set()
            start.join(1.0)
            watchdog.join(1.0)

        self.assertFalse(start.is_alive())
        self.assertFalse(watchdog.is_alive())
        self.assertTrue(start_publisher.messages)
        self.assertTrue(self.wait_until(lambda: all(
            len(publisher.messages) > previous
            for publisher, previous in zip(
                orchestrator.behavior_stop_pubs, first_stop_counts
            )
        )))
        self.assertTrue(orchestrator.control_watchdog_tripped)
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.task_dispatched)

    def test_pause_debt_blocks_resume_until_its_final_zero_returns(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_pause = threading.Event()
        blocked_pause = BlockingPublisher(release_pause)
        resume_publisher = FakePublisher()
        orchestrator.behavior_pause_pubs = {
            'follow_leader': blocked_pause,
        }
        orchestrator.behavior_resume_pubs = {
            'follow_leader': resume_publisher,
        }

        orchestrator._handle_pause_task({'task_id': 'task-a'})
        self.assertTrue(blocked_pause.entered.wait(0.5))
        self.assertTrue(orchestrator.ordinary_stop_active)

        orchestrator._handle_resume_task({'task_id': 'task-a'})
        self.assertFalse(resume_publisher.messages)
        self.assertEqual(
            ROS['orchestrator'].TaskState.PAUSED,
            orchestrator.task_state,
        )

        release_pause.set()
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator.ordinary_stop_active
        )))
        orchestrator._handle_resume_task({'task_id': 'task-a'})
        self.assertTrue(resume_publisher.messages)
        self.assertEqual(
            ROS['orchestrator'].TaskState.RUNNING,
            orchestrator.task_state,
        )

    def test_ordinary_stop_reasserts_zero_already_returning_from_socket(self):
        orchestrator = make_orchestrator()

        class CountingPublisher(FakePublisher):
            def __init__(self):
                super().__init__()
                self.calls = 0

            def publish(self, message):
                self.calls += 1
                super().publish(message)

        publisher = CountingPublisher()
        receipt_key = ('tb3_0', id(publisher))
        lane_key = receipt_key + ('supervisor',)
        operation_id = 1
        orchestrator._safety_publisher_snapshot = (
            ('tb3_0', publisher),
        )
        orchestrator._ordinary_stop_sequence = operation_id
        orchestrator._ordinary_stop_operations[operation_id] = {
            'task_id': 'task-a',
            'context': 'pause',
            'pending_publications': set(),
            'publications': {},
            'inflight_publications': set(),
            'failure_reporting': set(),
            'required_zero_receipts': {receipt_key: 2},
        }
        orchestrator.ordinary_stop_active = True

        # Model a zero which has returned from the ROS socket but is still
        # waiting to record its receipt behind the safety lock. The stop debt
        # must not mistake that old return for its final, causal zero.
        with orchestrator._safety_zero_lock:
            orchestrator._safety_zero_inflight.add(lane_key)
            orchestrator._safety_zero_followups.add(lane_key)
            returning_zero = threading.Thread(
                target=orchestrator._publish_supervised_zero,
                args=(lane_key, 'tb3_0', publisher),
            )
            returning_zero.start()
            self.assertTrue(self.wait_until(lambda: publisher.calls == 1))

        returning_zero.join(1.0)
        self.assertFalse(returning_zero.is_alive())
        self.assertTrue(self.wait_until(lambda: (
            publisher.calls == 2
            and orchestrator._safety_zero_receipts.get(receipt_key) == 2
            and not orchestrator.ordinary_stop_active
        )))
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)

    def test_replacement_start_waits_for_the_previous_stop_debt(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_stop = threading.Event()
        blocked_stop = BlockingPublisher(release_stop)
        orchestrator.behavior_stop_pubs[0] = blocked_stop
        new_task = {
            'task_id': 'task-b',
            'task_type': 'formation',
        }

        orchestrator._handle_start_task(new_task)
        self.assertTrue(blocked_stop.entered.is_set())
        self.assertTrue(orchestrator.ordinary_stop_active)
        self.assertEqual('task-a', orchestrator.current_task_id)
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(
            orchestrator.behavior_start_pubs['formation'].messages
        )

        release_stop.set()
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator.ordinary_stop_active
        )))
        orchestrator._handle_start_task(new_task)
        self.assertEqual('task-b', orchestrator.current_task_id)
        self.assertEqual(
            ROS['orchestrator'].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertTrue(
            orchestrator.behavior_start_pubs['formation'].messages
        )

    def test_unscheduled_stop_fails_closed_and_retries_from_the_timer(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        original_start = threading.Thread.start

        def reject_initial_stop_workers(worker):
            if (
                worker.name.startswith('orchestrator-safety-task-stop')
                or worker.name.startswith('supervised-zero-')
            ):
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=reject_initial_stop_workers,
        ):
            orchestrator._handle_stop_task({'task_id': 'task-a'})

        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertTrue(orchestrator.ordinary_stop_active)
        self.assertIn('could not schedule', orchestrator.task_error)

        orchestrator._safety_stop_zero_timer(None)
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator.ordinary_stop_active
        )))
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )

    def test_failed_stop_publish_leaves_the_operation_ready_for_retry(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        failing_publisher = FailOncePublisher()
        orchestrator.behavior_stop_pubs = [failing_publisher]

        orchestrator._handle_stop_task({'task_id': 'task-a'})
        operation_id = orchestrator._ordinary_stop_sequence

        self.assertTrue(failing_publisher.first_failure_returned.wait(0.5))
        self.assertTrue(self.wait_until(lambda: (
            0 not in orchestrator._ordinary_stop_operations[
                operation_id
            ]['inflight_publications']
            and orchestrator.task_state
            == ROS['orchestrator'].TaskState.FAILED
        )))
        self.assertTrue(orchestrator.ordinary_stop_active)
        self.assertIn('publication failed', orchestrator.task_error)
        self.assertIn(
            0,
            orchestrator._ordinary_stop_operations[
                operation_id
            ]['pending_publications'],
        )

        orchestrator._retry_ordinary_stop_publications()

        self.assertTrue(self.wait_until(lambda: (
            failing_publisher.calls == 2
            and not orchestrator.ordinary_stop_active
        )))
        self.assertEqual(1, len(failing_publisher.messages))
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )

    def test_formation_shutdown_latches_independent_periodic_zeros(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        command_publisher = orchestrator.cmd_vel_pubs['tb3_0']
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'shutdown-request-1',
        }))

        orchestrator._safety_stop_request_callback(request)

        self.assertTrue(orchestrator.supervised_stop_active)
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.supervised_stop_context['valid'])
        acknowledgement = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertEqual(
            'shutdown-request-1', acknowledgement['request_id']
        )
        self.assertTrue(acknowledgement['accepted'])
        self.assertTrue(acknowledgement['supervisor_latched'])
        self.assertTrue(
            acknowledgement['supervisor_will_remain_active']
        )
        self.assertTrue(acknowledgement['valid_request'])
        self.assertTrue(self.wait_until(lambda: command_publisher.messages))
        self.assertEqual(0.0, command_publisher.messages[-1].linear.x)
        self.assertTrue(self.wait_until(lambda: (
            orchestrator.task_state
            == ROS['orchestrator'].TaskState.FAILED
        )))
        self.assertIn('formation', orchestrator.task_error)

        first_zero_count = len(command_publisher.messages)
        self.assertTrue(self.wait_until(
            lambda: not orchestrator._safety_zero_inflight
        ))
        orchestrator._safety_stop_zero_timer(None)
        self.assertTrue(self.wait_until(
            lambda: len(command_publisher.messages) > first_zero_count
        ))
        orchestrator._handle_reset_emergency_stop({})
        self.assertTrue(orchestrator.supervised_stop_active)
        self.assertTrue(orchestrator.emergency_stop_active)

    def test_supervisor_ack_waits_for_zero_publish_to_return(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        release_zero = threading.Event()
        blocked = BlockingPublisher(release_zero)
        orchestrator.cmd_vel_pubs = {'tb3_0': blocked}
        orchestrator._refresh_safety_publisher_snapshot()
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'blocked-zero-receipt',
        }))

        try:
            orchestrator._safety_stop_request_callback(request)
            self.assertTrue(blocked.entered.wait(0.5))
            acknowledgements = [
                json.loads(message.data)
                for message in orchestrator.safety_stop_ack_pub.messages
            ]
            self.assertFalse(any(
                acknowledgement['accepted']
                for acknowledgement in acknowledgements
            ))
            self.assertFalse(
                acknowledgements[-1]['zero_publications_confirmed']
            )
        finally:
            release_zero.set()

        self.assertTrue(self.wait_until(lambda: any(
            json.loads(message.data).get('accepted') is True
            for message in orchestrator.safety_stop_ack_pub.messages
        )))
        acknowledgement = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertTrue(acknowledgement['zero_publications_confirmed'])
        self.assertEqual(
            acknowledgement['zero_target_count'],
            acknowledgement['zero_publish_return_count'],
        )

    def test_blocked_ack_never_blocks_periodic_zero_reassertion(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        release_ack = threading.Event()
        blocked_ack = BlockingPublisher(release_ack)
        ack_lane = ROS['orchestrator'].SafetyPublishLane(
            'test-blocked-ack'
        )
        self.assertTrue(ack_lane.available)
        orchestrator.safety_stop_ack_pub = blocked_ack
        orchestrator._safety_fallback_lanes[id(blocked_ack)] = ack_lane
        command_publisher = orchestrator.cmd_vel_pubs['tb3_0']
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'blocked-ack-link',
        }))

        try:
            started_at = time.monotonic()
            orchestrator._safety_stop_request_callback(request)
            self.assertLess(time.monotonic() - started_at, 0.2)
            self.assertTrue(blocked_ack.entered.wait(0.5))
            self.assertTrue(self.wait_until(lambda: (
                command_publisher.messages
                and not orchestrator._safety_zero_inflight
            )))
            first_zero_count = len(command_publisher.messages)

            for _ in range(3):
                timer_started = time.monotonic()
                orchestrator._safety_stop_zero_timer(None)
                self.assertLess(time.monotonic() - timer_started, 0.2)
                self.assertTrue(self.wait_until(lambda: (
                    len(command_publisher.messages) > first_zero_count
                    and not orchestrator._safety_zero_inflight
                )))
                first_zero_count = len(command_publisher.messages)

            self.assertEqual(1, blocked_ack.calls)
            self.assertEqual(
                {'blocked-ack-link'},
                orchestrator._safety_ack_publish_inflight,
            )
        finally:
            release_ack.set()

        self.assertTrue(self.wait_until(lambda: (
            'blocked-ack-link'
            not in orchestrator._safety_ack_publish_inflight
        )))
        ack_lane.close()

    def test_malformed_supervisor_request_fails_closed(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        command_publisher = orchestrator.cmd_vel_pubs['tb3_0']

        orchestrator._safety_stop_request_callback(String(data='{broken'))

        self.assertTrue(orchestrator.supervised_stop_active)
        self.assertTrue(orchestrator.emergency_stop_active)
        context = orchestrator.supervised_stop_context
        self.assertFalse(context['valid'])
        self.assertIn('JSON object', context['validation_error'])
        self.assertFalse(orchestrator.safety_stop_ack_pub.messages)
        self.assertTrue(self.wait_until(lambda: command_publisher.messages))
        self.assertEqual(0.0, command_publisher.messages[-1].linear.x)

    def test_supervised_state_worker_start_failure_is_retryable(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'state-worker-retry',
        }))
        original_start = threading.Thread.start
        failed_once = []

        def fail_first_state_worker(worker):
            if (
                worker.name == 'supervised-safety-stop-state'
                and len(failed_once) < 2
            ):
                failed_once.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=fail_first_state_worker,
        ):
            orchestrator._safety_stop_request_callback(request)
            self.assertEqual(2, len(failed_once))
            self.assertFalse(orchestrator._supervised_stop_state_started)
            acknowledgement = json.loads(
                orchestrator.safety_stop_ack_pub.messages[-1].data
            )
            self.assertFalse(acknowledgement['accepted'])
            self.assertFalse(acknowledgement['state_worker_started'])
            orchestrator._safety_stop_zero_timer(None)

        self.assertTrue(self.wait_until(lambda: (
            orchestrator.task_state == ROS['orchestrator'].TaskState.FAILED
        )))
        self.assertTrue(orchestrator._supervised_stop_state_started)
        self.assertGreaterEqual(
            len(orchestrator.safety_stop_ack_pub.messages), 2
        )
        acknowledgement = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertEqual('state-worker-retry', acknowledgement['request_id'])
        self.assertTrue(acknowledgement['accepted'])
        self.assertTrue(acknowledgement['state_worker_started'])
        self.assertEqual({}, orchestrator._pending_safety_stop_acks)

    def test_zero_worker_start_failure_republishes_positive_ack_on_retry(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'zero-worker-retry',
        }))
        original_start = threading.Thread.start
        failed_starts = []

        def fail_robot_zero_workers(worker):
            if (
                worker.name.startswith('supervised-zero-tb3_0-')
                and len(failed_starts) < 2
            ):
                failed_starts.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=fail_robot_zero_workers,
        ):
            orchestrator._safety_stop_request_callback(request)
            self.assertEqual(2, len(failed_starts))
            first_ack = json.loads(
                orchestrator.safety_stop_ack_pub.messages[-1].data
            )
            self.assertFalse(first_ack['accepted'])
            self.assertIn('zero-worker-retry', (
                orchestrator._pending_safety_stop_acks
            ))
            orchestrator._safety_stop_zero_timer(None)

        final_ack = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertEqual('zero-worker-retry', final_ack['request_id'])
        self.assertTrue(final_ack['accepted'])
        self.assertEqual(
            final_ack['zero_target_count'],
            final_ack['zero_publish_return_count'],
        )
        self.assertTrue(final_ack['zero_publications_confirmed'])
        self.assertEqual({}, orchestrator._pending_safety_stop_acks)

    def test_request_after_shutdown_never_receives_a_positive_ack(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        with orchestrator._safety_zero_lock:
            orchestrator._shutdown_started = True
            orchestrator._supervised_stop_state_started = True
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'request-after-shutdown',
        }))

        orchestrator._safety_stop_request_callback(request)

        acknowledgement = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertFalse(acknowledgement['accepted'])
        self.assertFalse(acknowledgement['state_worker_started'])
        self.assertFalse(
            acknowledgement['supervisor_will_remain_active']
        )
        orchestrator._safety_stop_zero_timer(None)
        final_ack = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertFalse(final_ack['accepted'])
        self.assertFalse(final_ack['supervisor_will_remain_active'])

    def test_positive_ack_is_republished_after_reverse_link_connects(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        orchestrator.safety_stop_ack_pub.connections = 0
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'late-ack-link',
        }))

        orchestrator._safety_stop_request_callback(request)

        first_ack = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertTrue(first_ack['accepted'])
        self.assertIn(
            'late-ack-link', orchestrator._pending_safety_stop_acks
        )

        orchestrator.safety_stop_ack_pub.connections = 1
        orchestrator._safety_stop_zero_timer(None)

        self.assertGreaterEqual(
            len(orchestrator.safety_stop_ack_pub.messages), 2
        )
        final_ack = json.loads(
            orchestrator.safety_stop_ack_pub.messages[-1].data
        )
        self.assertEqual('late-ack-link', final_ack['request_id'])
        self.assertTrue(final_ack['accepted'])
        self.assertEqual({}, orchestrator._pending_safety_stop_acks)

    def test_constructor_starts_and_shutdown_stops_wall_watchdog(self):
        orchestrator = ROS["orchestrator"].TaskOrchestrator()
        self.assertTrue(orchestrator._control_watchdog_thread.is_alive())
        self.assertFalse(orchestrator.control_heartbeat_seen)
        self.assertTrue(orchestrator.safety_stop_ack_pub.kwargs['latch'])
        self.assertNotIn(
            'queue_size', orchestrator.safety_stop_ack_pub.kwargs
        )

        orchestrator._shutdown()

        self.assertFalse(orchestrator._control_watchdog_thread.is_alive())
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.supervised_stop_active)

    def test_orderly_shutdown_stops_behaviors_and_zeroes_every_robot(self):
        orchestrator = make_orchestrator()
        orchestrator._stop_all_robots = types.MethodType(
            ROS["orchestrator"].TaskOrchestrator._stop_all_robots,
            orchestrator,
        )
        orchestrator._control_watchdog_stop = threading.Event()
        orchestrator._control_watchdog_thread = None

        orchestrator._shutdown()

        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual("task-a", payload["task_id"])
        command = orchestrator.cmd_vel_pubs["tb3_0"].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)
        leader_command = orchestrator.leader_cmd_pub.messages[-1]
        self.assertEqual(0.0, leader_command.linear.x)
        self.assertEqual(0.0, leader_command.angular.z)
        self.assertTrue(orchestrator._control_watchdog_stop.is_set())

    def test_shutdown_fanout_survives_blocked_command_and_behavior_pubs(self):
        orchestrator = make_orchestrator()
        release_command = threading.Event()
        release_behavior = threading.Event()
        blocked_command = BlockingPublisher(release_command)
        healthy_command = FakePublisher()
        blocked_behavior = BlockingPublisher(release_behavior)
        healthy_behaviors = [FakePublisher(), FakePublisher()]
        orchestrator.cmd_vel_pubs = {
            'tb3_0': blocked_command,
            'tb3_1': healthy_command,
        }
        orchestrator.behavior_stop_pubs = [
            blocked_behavior,
            *healthy_behaviors,
        ]
        orchestrator._refresh_safety_publisher_snapshot()
        orchestrator.SHUTDOWN_TIMEOUT = 0.08

        started_at = time.monotonic()
        try:
            orchestrator._shutdown()
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.25)
            self.assertTrue(blocked_command.entered.is_set())
            self.assertTrue(blocked_behavior.entered.is_set())
            self.assertTrue(healthy_command.messages)
            command = healthy_command.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)
            for publisher in healthy_behaviors:
                payload = json.loads(publisher.messages[-1].data)
                self.assertEqual('task-a', payload['task_id'])
            self.assertTrue(orchestrator.supervised_stop_active)
            self.assertTrue(orchestrator.emergency_stop_active)
            self.assertTrue(orchestrator._control_watchdog_stop.is_set())
        finally:
            release_command.set()
            release_behavior.set()

    def test_shutdown_fanout_continues_after_one_thread_start_failure(self):
        orchestrator = make_orchestrator()
        first = FakePublisher()
        healthy = FakePublisher()
        orchestrator.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': healthy,
        }
        orchestrator._refresh_safety_publisher_snapshot()
        original_start = threading.Thread.start
        failed_once = []

        def fail_first_command_worker(worker):
            if (
                worker.name == 'orchestrator-shutdown-tb3_0-1'
                and not failed_once
            ):
                failed_once.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=fail_first_command_worker,
        ):
            orchestrator._shutdown()

        self.assertEqual(1, len(failed_once))
        self.assertTrue(first.messages)
        self.assertEqual(0.0, first.messages[-1].linear.x)
        self.assertEqual(0.0, first.messages[-1].angular.z)
        self.assertTrue(healthy.messages)
        self.assertEqual(0.0, healthy.messages[-1].linear.x)
        self.assertEqual(0.0, healthy.messages[-1].angular.z)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)
        for publisher in orchestrator.behavior_stop_pubs:
            self.assertTrue(publisher.messages)

    def test_shutdown_reaches_later_targets_after_both_start_attempts_fail(self):
        orchestrator = make_orchestrator()
        first = FakePublisher()
        healthy = FakePublisher()
        orchestrator.cmd_vel_pubs = {
            'tb3_0': first,
            'tb3_1': healthy,
        }
        orchestrator._refresh_safety_publisher_snapshot()
        fallback_lane = ROS['orchestrator'].SafetyPublishLane(
            'test-tb3-0'
        )
        self.assertTrue(fallback_lane.available)
        orchestrator._safety_fallback_lanes[id(first)] = fallback_lane
        original_start = threading.Thread.start
        failures = []

        def fail_first_command_workers(worker):
            if worker.name.startswith('orchestrator-shutdown-tb3_0-'):
                failures.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        try:
            with mock.patch.object(
                ROS['orchestrator'].threading.Thread,
                'start',
                new=fail_first_command_workers,
            ):
                orchestrator._shutdown()
        finally:
            fallback_lane.close()

        self.assertEqual(2, len(failures))
        self.assertTrue(first.messages)
        self.assertEqual(0.0, first.messages[-1].linear.x)
        self.assertEqual(0.0, first.messages[-1].angular.z)
        self.assertTrue(healthy.messages)
        self.assertEqual(0.0, healthy.messages[-1].linear.x)
        self.assertEqual(0.0, healthy.messages[-1].angular.z)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)

    def test_shutdown_during_previous_stop_cannot_restore_initializing(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        release_stop = threading.Event()
        blocked_stop = BlockingPublisher(release_stop)
        orchestrator.behavior_stop_pubs[0] = blocked_stop
        new_start = orchestrator.behavior_start_pubs['formation']
        start = threading.Thread(
            target=orchestrator._handle_start_task,
            args=({
                'task_id': 'replacement-task',
                'task_type': 'formation',
            },),
        )

        start.start()
        self.assertTrue(blocked_stop.entered.wait(0.5))
        try:
            orchestrator._shutdown()
        finally:
            release_stop.set()
            start.join(1.0)

        self.assertFalse(start.is_alive())
        self.assertEqual('task-a', orchestrator.current_task_id)
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertFalse(new_start.messages)

    def test_shutdown_after_start_postcheck_keeps_final_failed_state(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS['orchestrator'].TaskState.IDLE
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        orchestrator._safety_zero_lock = ExitHookLock(
            2, orchestrator._shutdown
        )

        orchestrator._handle_start_task({
            'task_id': 'postcheck-start',
            'task_type': 'follow_leader',
        })

        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertIsNone(orchestrator.task_dispatched_at)
        self.assertIsNone(orchestrator.last_behavior_status_at)
        self.assertIn('shut down', orchestrator.task_error)

    def test_shutdown_after_resume_postcheck_clears_dispatch_metadata(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        orchestrator.task_dispatched_at = 5.0
        orchestrator.behavior_resume_pubs = {
            'follow_leader': FakePublisher(),
        }
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        orchestrator._safety_zero_lock = ExitHookLock(
            1, orchestrator._shutdown
        )

        orchestrator._handle_resume_task({'task_id': 'task-a'})

        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertIsNone(orchestrator.task_dispatched_at)
        self.assertIsNone(orchestrator.last_behavior_status_at)
        self.assertIn('shut down', orchestrator.task_error)

    def test_shutdown_after_reset_postcheck_preserves_latched_failure(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        orchestrator.emergency_stop_active = True
        orchestrator.task_dispatched = True
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        orchestrator._safety_zero_lock = ExitHookLock(
            2, orchestrator._shutdown
        )

        orchestrator._handle_reset_emergency_stop({})

        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.task_dispatched)
        self.assertIsNone(orchestrator.task_dispatched_at)
        self.assertIn('shut down', orchestrator.task_error)
        self.assertFalse(orchestrator.emergency_stop_pub.messages[0].data)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)

    def test_shutdown_behavior_stops_keep_the_latched_task_id(self):
        orchestrator = make_orchestrator()
        original_start = threading.Thread.start
        injected = []

        def reject_during_shutdown(worker):
            if (
                worker.name == 'orchestrator-shutdown-emergency-stop-1'
                and not injected
            ):
                injected.append(worker.name)
                orchestrator._record_rejected_task(
                    'bad-task', 'formation', 'invalid test task'
                )
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=reject_during_shutdown,
        ):
            orchestrator._shutdown()

        self.assertEqual(1, len(injected))
        self.assertEqual('task-a', orchestrator.current_task_id)
        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual('task-a', payload['task_id'])

    def test_shutdown_preempts_a_blocked_start_and_compensates_it(self):
        orchestrator = make_orchestrator(task_id=None)
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS['orchestrator'].TaskState.IDLE
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        release_start = threading.Event()
        start_publisher = BlockingPublisher(release_start)
        healthy = FakePublisher()
        orchestrator.behavior_start_pubs['follow_leader'] = start_publisher
        orchestrator.cmd_vel_pubs = {'tb3_0': healthy}
        orchestrator._refresh_safety_publisher_snapshot()
        start = threading.Thread(
            target=orchestrator._handle_start_task,
            args=({
                'task_id': 'late-start-task',
                'task_type': 'follow_leader',
            },),
        )
        shutdown = threading.Thread(target=orchestrator._shutdown)

        start.start()
        self.assertTrue(start_publisher.entered.wait(0.7))
        try:
            shutdown.start()
            shutdown.join(0.5)
            self.assertFalse(shutdown.is_alive())
            self.assertTrue(start.is_alive())
            self.assertTrue(orchestrator._shutdown_started)
            self.assertTrue(orchestrator.emergency_stop_active)
            self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)
            self.assertTrue(healthy.messages)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            first_stop_counts = [
                len(publisher.messages)
                for publisher in orchestrator.behavior_stop_pubs
            ]
        finally:
            release_start.set()
            start.join(1.0)
            shutdown.join(1.0)

        self.assertFalse(start.is_alive())
        self.assertFalse(shutdown.is_alive())
        self.assertTrue(start_publisher.messages)
        self.assertTrue(self.wait_until(lambda: all(
            len(publisher.messages) > previous
            for publisher, previous in zip(
                orchestrator.behavior_stop_pubs, first_stop_counts
            )
        )))
        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual('late-start-task', payload['task_id'])
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)

    def test_late_resume_after_shutdown_is_followed_by_behavior_stops(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator.task_dispatched = True
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        release_resume = threading.Event()
        resume_publisher = BlockingPublisher(release_resume)
        orchestrator.behavior_resume_pubs = {
            'follow_leader': resume_publisher,
        }
        resume = threading.Thread(
            target=orchestrator._handle_resume_task,
            args=({'task_id': 'task-a'},),
        )

        resume.start()
        self.assertTrue(resume_publisher.entered.wait(0.5))
        try:
            orchestrator._shutdown()
            stop_counts = [
                len(publisher.messages)
                for publisher in orchestrator.behavior_stop_pubs
            ]
        finally:
            release_resume.set()
            resume.join(1.0)

        self.assertFalse(resume.is_alive())
        self.assertTrue(resume_publisher.messages)
        self.assertTrue(self.wait_until(lambda: all(
            len(publisher.messages) > previous
            for publisher, previous in zip(
                orchestrator.behavior_stop_pubs, stop_counts
            )
        )))
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)

    def test_late_reset_after_shutdown_restores_true_as_final_signal(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        orchestrator.task_dispatched = True
        orchestrator.emergency_stop_active = True
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        release_reset = threading.Event()
        release_command = threading.Event()
        emergency_publisher = BlockingFalsePublisher(release_reset)
        command_publisher = BlockingPublisher(release_command)
        orchestrator.emergency_stop_pub = emergency_publisher
        orchestrator.cmd_vel_pubs = {'tb3_0': command_publisher}
        orchestrator._refresh_safety_publisher_snapshot()
        orchestrator._stop_all_robots = types.MethodType(
            ROS['orchestrator'].TaskOrchestrator._stop_all_robots,
            orchestrator,
        )
        reset = threading.Thread(
            target=orchestrator._handle_reset_emergency_stop,
            args=({},),
        )

        reset.start()
        self.assertTrue(emergency_publisher.entered.wait(0.5))
        try:
            orchestrator._shutdown()
            self.assertTrue(any(
                message.data is True
                for message in emergency_publisher.messages
            ))
        finally:
            release_reset.set()
            self.assertTrue(self.wait_until(lambda: (
                emergency_publisher.messages
                and emergency_publisher.messages[-1].data is True
            )))
            self.assertTrue(reset.is_alive())
            release_command.set()
            reset.join(1.0)

        self.assertFalse(reset.is_alive())
        self.assertTrue(self.wait_until(lambda: (
            emergency_publisher.messages
            and emergency_publisher.messages[-1].data is True
        )))
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.control_watchdog_tripped)
        self.assertFalse(orchestrator.task_dispatched)
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )

    def test_late_completion_cannot_overwrite_shutdown_failure(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        release_stop = threading.Event()
        blocked_stop = BlockingPublisher(release_stop)
        orchestrator.behavior_stop_pubs[0] = blocked_stop
        completion = threading.Thread(
            target=orchestrator._complete_current_task,
            args=('task-a',),
        )

        completion.start()
        self.assertTrue(blocked_stop.entered.wait(0.5))
        try:
            orchestrator._shutdown()
        finally:
            release_stop.set()
            completion.join(1.0)

        self.assertFalse(completion.is_alive())
        self.assertEqual(
            ROS['orchestrator'].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual(0.0, orchestrator.task_progress)
        self.assertFalse(orchestrator.task_dispatched)

    def test_emergency_fanout_continues_when_signal_publisher_blocks(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_signal = threading.Event()
        blocked_signal = BlockingPublisher(release_signal)
        orchestrator.emergency_stop_pub = blocked_signal

        try:
            started_at = time.monotonic()
            orchestrator._handle_emergency_stop({})
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.2)
            self.assertTrue(blocked_signal.entered.wait(0.5))
            self.assertTrue(self.wait_until(lambda: all(
                publisher.messages
                for publisher in orchestrator.behavior_stop_pubs
            )))
            command_publisher = orchestrator.cmd_vel_pubs['tb3_0']
            self.assertTrue(self.wait_until(lambda: (
                command_publisher.messages
                and command_publisher.messages[-1].linear.x == 0.0
            )))
            self.assertEqual(
                ROS['orchestrator'].TaskState.STOPPED,
                orchestrator.task_state,
            )
        finally:
            release_signal.set()

    def test_emergency_signal_publish_failures_remain_pending_for_retry(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        emergency_publisher = FailOncePublisher()
        stop_publishers = [FailOncePublisher() for _ in range(3)]
        orchestrator.emergency_stop_pub = emergency_publisher
        orchestrator.behavior_stop_pubs = stop_publishers

        orchestrator._handle_emergency_stop({})

        publishers = [emergency_publisher] + stop_publishers
        self.assertTrue(self.wait_until(lambda: all(
            publisher.calls == 1 for publisher in publishers
        )))
        self.assertTrue(orchestrator._emergency_publication_debts)
        self.assertTrue(all(
            not publisher.messages for publisher in publishers
        ))

        deadline = time.monotonic() + 0.5
        while (
            orchestrator._emergency_publication_debts
            and time.monotonic() < deadline
        ):
            orchestrator._safety_stop_zero_timer(None)
            time.sleep(0.005)

        self.assertFalse(orchestrator._emergency_publication_debts)
        self.assertTrue(emergency_publisher.messages[-1].data)
        for publisher in stop_publishers:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual('task-a', payload['task_id'])
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )

    def test_older_reset_cannot_clear_a_newer_emergency_true(self):
        orchestrator = make_orchestrator()
        orchestrator.emergency_stop_active = True
        task_lock = WaiterAwareRLock()
        orchestrator.task_lock = task_lock
        reset = threading.Thread(
            target=orchestrator._handle_reset_emergency_stop,
            args=({},),
        )
        emergency = threading.Thread(
            target=orchestrator._handle_emergency_stop,
            args=({},),
        )

        task_lock.acquire()
        task_lock.acquire_attempted.clear()
        try:
            reset.start()
            self.assertTrue(task_lock.acquire_attempted.wait(0.5))

            emergency.start()
            self.assertTrue(self.wait_until(lambda: (
                orchestrator._emergency_order_generation == 1
                and orchestrator._pending_emergency_true_generations
            )))
            self.assertTrue(self.wait_until(lambda: (
                not orchestrator._emergency_publication_debts
            )))
            self.assertTrue(reset.is_alive())
            self.assertTrue(emergency.is_alive())
        finally:
            task_lock.release()
            reset.join(1.0)
            emergency.join(1.0)

        self.assertFalse(reset.is_alive())
        self.assertFalse(emergency.is_alive())
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertFalse(any(
            message.data is False
            for message in orchestrator.emergency_stop_pub.messages
        ))
        self.assertFalse(
            orchestrator._pending_emergency_true_generations
        )

    def test_emergency_true_then_reset_false_precede_a_new_start(self):
        orchestrator = make_orchestrator()
        release_false = threading.Event()
        publication_order = []

        class OrderedEmergencyPublisher(BlockingFalsePublisher):
            def publish(self, message):
                super().publish(message)
                publication_order.append(('emergency', message.data))

        class OrderedStartPublisher(FakePublisher):
            def publish(self, message):
                super().publish(message)
                task = json.loads(message.data)
                publication_order.append(('start', task['task_id']))

        emergency_publisher = OrderedEmergencyPublisher(release_false)
        start_publisher = OrderedStartPublisher()
        orchestrator.emergency_stop_pub = emergency_publisher
        orchestrator.behavior_start_pubs['formation'] = start_publisher

        orchestrator._handle_emergency_stop({})
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator._emergency_publication_debts
        )))
        self.assertEqual([('emergency', True)], publication_order)

        task_lock = WaiterAwareRLock()
        orchestrator.task_lock = task_lock
        reset = threading.Thread(
            target=orchestrator._handle_reset_emergency_stop,
            args=({},),
        )
        start = threading.Thread(
            target=orchestrator._handle_start_task,
            args=({
                'task_id': 'ordered-replacement',
                'task_type': 'formation',
            },),
        )

        reset.start()
        self.assertTrue(emergency_publisher.entered.wait(0.5))
        task_lock.acquire_attempted.clear()
        start.start()
        self.assertTrue(task_lock.acquire_attempted.wait(0.5))
        self.assertFalse(start_publisher.messages)

        release_false.set()
        reset.join(1.0)
        start.join(1.0)

        self.assertFalse(reset.is_alive())
        self.assertFalse(start.is_alive())
        self.assertEqual([
            ('emergency', True),
            ('emergency', False),
            ('start', 'ordered-replacement'),
        ], publication_order)
        self.assertEqual(
            ROS['orchestrator'].TaskState.RUNNING,
            orchestrator.task_state,
        )

    def test_reset_waits_until_the_emergency_true_message_returns(self):
        orchestrator = make_orchestrator()
        release_true = threading.Event()
        emergency_publisher = BlockingTruePublisher(release_true)
        orchestrator.emergency_stop_pub = emergency_publisher

        orchestrator._handle_emergency_stop({})
        self.assertTrue(emergency_publisher.entered.wait(0.5))
        self.assertTrue(orchestrator.emergency_stop_active)

        orchestrator._handle_reset_emergency_stop({})
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertFalse(any(
            message.data is False
            for message in emergency_publisher.messages
        ))

        release_true.set()
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator._emergency_publication_debts
        )))
        orchestrator._handle_reset_emergency_stop({})

        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(emergency_publisher.messages[-1].data)

    def test_spurious_reset_does_not_stop_a_running_task(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator.task_ever_dispatched = True
        zero_count = len(orchestrator.cmd_vel_pubs['tb3_0'].messages)

        orchestrator._handle_reset_emergency_stop({})

        self.assertEqual(
            ROS['orchestrator'].TaskState.RUNNING,
            orchestrator.task_state,
        )
        self.assertTrue(orchestrator.task_dispatched)
        self.assertFalse(orchestrator.emergency_stop_pub.messages)
        self.assertEqual(
            zero_count,
            len(orchestrator.cmd_vel_pubs['tb3_0'].messages),
        )

    def test_stale_start_cannot_revive_a_task_after_emergency_reset(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator.task_ever_dispatched = True

        orchestrator._handle_emergency_stop({})
        self.assertTrue(self.wait_until(lambda: (
            not orchestrator._emergency_publication_debts
        )))
        orchestrator._handle_reset_emergency_stop({})
        start_count = len(
            orchestrator.behavior_start_pubs['follow_leader'].messages
        )

        orchestrator._handle_start_task({
            'task_id': 'task-a',
            'task_type': 'follow_leader',
        })

        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertEqual(
            start_count,
            len(orchestrator.behavior_start_pubs['follow_leader'].messages),
        )

    def test_detached_safety_publish_retries_a_thread_start_failure(self):
        orchestrator = make_orchestrator()
        publisher = FakePublisher()
        original_start = threading.Thread.start
        failed_once = []

        def fail_first_detached_worker(worker):
            if (
                worker.name == 'orchestrator-safety-retry-signal-1'
                and not failed_once
            ):
                failed_once.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        with mock.patch.object(
            ROS['orchestrator'].threading.Thread,
            'start',
            new=fail_first_detached_worker,
        ):
            started = orchestrator._start_detached_safety_publish(
                'retry-signal', publisher, Bool(data=True)
            )

        self.assertTrue(started)
        self.assertEqual(1, len(failed_once))
        self.assertTrue(self.wait_until(lambda: publisher.messages))
        self.assertTrue(publisher.messages[-1].data)

    def test_detached_safety_publish_uses_prestarted_lane_when_threads_fail(self):
        orchestrator = make_orchestrator()
        publisher = FakePublisher()
        fallback_lane = ROS['orchestrator'].SafetyPublishLane(
            'test-detached-signal'
        )
        self.assertTrue(fallback_lane.available)
        orchestrator._safety_fallback_lanes[id(publisher)] = fallback_lane
        original_start = threading.Thread.start
        failed_starts = []

        def fail_detached_workers(worker):
            if worker.name.startswith(
                'orchestrator-safety-persistent-signal-'
            ):
                failed_starts.append(worker.name)
                raise RuntimeError('thread capacity exhausted')
            return original_start(worker)

        try:
            with mock.patch.object(
                ROS['orchestrator'].threading.Thread,
                'start',
                new=fail_detached_workers,
            ):
                started = orchestrator._start_detached_safety_publish(
                    'persistent-signal', publisher, Bool(data=True)
                )
            self.assertTrue(started)
            self.assertEqual(2, len(failed_starts))
            self.assertTrue(self.wait_until(lambda: publisher.messages))
            self.assertTrue(publisher.messages[-1].data)
        finally:
            fallback_lane.close()

    def test_safety_lane_never_accepts_a_job_behind_close_sentinel(self):
        lane = ROS['orchestrator'].SafetyPublishLane('submit-close-race')
        self.assertTrue(lane.available)
        callback_ran = threading.Event()
        enqueue_entered = threading.Event()
        release_enqueue = threading.Event()
        original_put = lane._jobs.put_nowait

        def blocking_put(item):
            if item is not None:
                enqueue_entered.set()
                release_enqueue.wait(0.5)
            original_put(item)

        lane._jobs.put_nowait = blocking_put
        submit_result = []
        submit = threading.Thread(
            target=lambda: submit_result.append(
                lane.submit(callback_ran.set)
            )
        )
        close = threading.Thread(target=lane.close)
        submit.start()
        self.assertTrue(enqueue_entered.wait(0.5))
        close.start()
        self.assertTrue(close.is_alive())

        release_enqueue.set()
        submit.join(0.5)
        close.join(0.5)

        self.assertEqual([True], submit_result)
        self.assertTrue(callback_ran.wait(0.5))
        self.assertFalse(lane.available)

    def test_robot_announced_after_shutdown_is_not_added_to_stop_snapshot(self):
        orchestrator = make_orchestrator()
        orchestrator._shutdown()
        original_snapshot = tuple(orchestrator._safety_publisher_snapshot)

        orchestrator._robot_list_callback(String(data='tb3_0,tb3_1'))

        self.assertNotIn('tb3_1', orchestrator.robots)
        self.assertNotIn('tb3_1', orchestrator.cmd_vel_pubs)
        self.assertEqual(1, orchestrator.robot_count)
        self.assertEqual(
            original_snapshot,
            orchestrator._safety_publisher_snapshot,
        )

    def test_shutdown_during_robot_setup_discards_all_late_resources(self):
        orchestrator = make_orchestrator()
        orchestrator.SHUTDOWN_TIMEOUT = 0.08
        publisher_entered = threading.Event()
        release_publisher = threading.Event()
        created_publishers = []
        created_subscribers = []
        original_publisher = ROS['orchestrator'].rospy.Publisher
        original_subscriber = ROS['orchestrator'].rospy.Subscriber

        def blocking_publisher(topic, *args, **kwargs):
            if topic != '/tb3_1/cmd_vel':
                return original_publisher(topic, *args, **kwargs)
            publisher_entered.set()
            release_publisher.wait(1.0)
            publisher = FakePublisher()
            created_publishers.append(publisher)
            return publisher

        def recording_subscriber(*args, **kwargs):
            subscriber = original_subscriber(*args, **kwargs)
            created_subscribers.append(subscriber)
            return subscriber

        with mock.patch.object(
            ROS['orchestrator'].rospy,
            'Publisher',
            side_effect=blocking_publisher,
        ), mock.patch.object(
            ROS['orchestrator'].rospy,
            'Subscriber',
            side_effect=recording_subscriber,
        ):
            roster = threading.Thread(
                target=orchestrator._robot_list_callback,
                args=(String(data='tb3_0,tb3_1'),),
            )
            roster.start()
            self.assertTrue(publisher_entered.wait(0.5))
            try:
                orchestrator._shutdown()
            finally:
                release_publisher.set()
                roster.join(1.0)

        self.assertFalse(roster.is_alive())
        self.assertEqual(1, len(created_publishers))
        self.assertTrue(created_publishers[0].unregistered)
        self.assertEqual(4, len(created_subscribers))
        self.assertTrue(all(
            subscriber.unregistered for subscriber in created_subscribers
        ))
        self.assertNotIn('tb3_1', orchestrator.robots)
        self.assertNotIn('tb3_1', orchestrator.cmd_vel_pubs)
        self.assertFalse(any(
            robot_id == 'tb3_1'
            for robot_id, _publisher
            in orchestrator._safety_publisher_snapshot
        ))

    def test_failed_replacement_roster_cancels_a_task_with_no_robots(self):
        class UnavailableLane:
            def __init__(self, _label, _error_logger=None):
                self.available = False

            def close(self):
                pass

        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator._safety_fallback_lane_factory = UnavailableLane

        orchestrator._robot_list_callback(String(data='tb3_1'))

        self.assertEqual({}, orchestrator.robots)
        self.assertEqual({}, orchestrator.cmd_vel_pubs)
        self.assertEqual(0, orchestrator.robot_count)
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertIsNone(orchestrator.task_dispatched_at)
        self.assertIsNone(orchestrator.last_behavior_status_at)

    def test_emergency_zero_publish_does_not_wait_on_ros_clock(self):
        orchestrator = make_orchestrator()
        with mock.patch.object(
            ROS["orchestrator"].rospy,
            "sleep",
            side_effect=AssertionError("ROS-time sleep used"),
        ), mock.patch.object(
            ROS["orchestrator"].time,
            "sleep",
        ) as wall_sleep:
            ROS["orchestrator"].TaskOrchestrator._stop_all_robots(
                orchestrator
            )

        wall_sleep.assert_not_called()
        command = orchestrator.cmd_vel_pubs["tb3_0"].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)

    def test_removed_robot_releases_its_stop_publisher(self):
        orchestrator = make_orchestrator()
        publisher = orchestrator.cmd_vel_pubs["tb3_0"]

        orchestrator._unsubscribe_from_robot("tb3_0")

        self.assertTrue(self.wait_until(lambda: publisher.messages))
        self.assertTrue(self.wait_until(lambda: publisher.unregistered))
        self.assertNotIn("tb3_0", orchestrator.cmd_vel_pubs)

    def test_removed_publisher_remains_a_stop_debt_until_zero_is_accepted(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        release = threading.Event()
        publisher = BlockingPublisher(release)
        orchestrator.cmd_vel_pubs['tb3_0'] = publisher
        orchestrator._refresh_safety_publisher_snapshot()

        try:
            orchestrator._unsubscribe_from_robot('tb3_0')
            self.assertTrue(publisher.entered.wait(0.5))
            self.assertTrue(any(
                name == 'tb3_0' and candidate is publisher
                for name, candidate
                in orchestrator._safety_publisher_snapshot
            ))
            self.assertIn(
                ('tb3_0', id(publisher)),
                orchestrator._retired_safety_publishers,
            )

            orchestrator._safety_stop_request_callback(String(data=json.dumps({
                'source': 'formation',
                'reason': 'shutdown',
                'task_id': 'task-a',
                'request_id': 'retired-publisher-request',
            })))
            self.assertTrue(self.wait_until(lambda: publisher.calls >= 2))
            self.assertFalse(publisher.unregistered)
        finally:
            release.set()

        self.assertTrue(self.wait_until(lambda: publisher.unregistered))
        self.assertTrue(self.wait_until(lambda: not any(
            candidate is publisher
            for _name, candidate in orchestrator._safety_publisher_snapshot
        )))

    def test_reset_and_supervised_request_have_one_atomic_order(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        orchestrator.emergency_stop_active = True
        clock_entered = threading.Event()
        release_clock = threading.Event()
        callback_finished = threading.Event()

        def blocking_clock():
            clock_entered.set()
            release_clock.wait(0.5)
            return 10.0

        orchestrator.last_control_heartbeat = 10.0
        orchestrator.last_control_heartbeat_deadline = 24.0
        orchestrator._control_clock = blocking_clock
        reset = threading.Thread(
            target=orchestrator._handle_reset_emergency_stop,
            args=({},),
        )
        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'reset-race-request',
        }))

        def latch_request():
            orchestrator._safety_stop_request_callback(request)
            callback_finished.set()

        callback = threading.Thread(target=latch_request)
        reset.start()
        self.assertTrue(clock_entered.wait(0.5))
        callback.start()
        self.assertFalse(callback_finished.wait(0.03))

        release_clock.set()
        reset.join(0.5)
        callback.join(0.5)
        self.assertFalse(reset.is_alive())
        self.assertFalse(callback.is_alive())
        self.assertTrue(orchestrator.supervised_stop_active)
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(self.wait_until(lambda: (
            orchestrator.emergency_stop_pub.messages
            and orchestrator.emergency_stop_pub.messages[-1].data is True
        )))
        self.assertFalse(orchestrator.emergency_stop_pub.messages[0].data)

    def test_supervised_latch_blocks_start_resume_and_manual_motion(self):
        orchestrator = make_orchestrator()
        orchestrator.supervised_stop_active = True
        # The supervised bit is authoritative even if another callback has
        # not yet mirrored it into the ordinary emergency-stop flag.
        orchestrator.emergency_stop_active = False

        start_publisher = orchestrator.behavior_start_pubs['follow_leader']
        orchestrator._handle_start_task({
            'task_id': 'must-not-start',
            'task_type': 'follow_leader',
        })
        self.assertFalse(start_publisher.messages)

        resume_publisher = FakePublisher()
        orchestrator.behavior_resume_pubs = {
            'follow_leader': resume_publisher,
        }
        orchestrator.task_state = ROS['orchestrator'].TaskState.PAUSED
        orchestrator._handle_resume_task({'task_id': 'task-a'})
        self.assertFalse(resume_publisher.messages)
        self.assertEqual(
            ROS['orchestrator'].TaskState.PAUSED,
            orchestrator.task_state,
        )

        orchestrator._handle_control_leader({
            'linear_velocity': 0.22,
            'angular_velocity': 0.5,
        })
        command = orchestrator.leader_cmd_pub.messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)

    def test_manual_control_rejects_every_nonfresh_heartbeat(self):
        heartbeat_cases = (
            ('missing', False, None, 0.0),
            ('negative-age', True, 1.0, 0.0),
            ('timeout-boundary', True, 0.0, 10.0),
        )

        for label, seen, last_heartbeat, now in heartbeat_cases:
            with self.subTest(case=label):
                orchestrator = make_orchestrator()
                orchestrator.control_heartbeat_seen = seen
                orchestrator.last_control_heartbeat = last_heartbeat
                orchestrator._control_clock = lambda value=now: value
                publisher = FakePublisher()
                orchestrator.leader_cmd_pub = publisher
                orchestrator._refresh_safety_publisher_snapshot()

                orchestrator._handle_control_leader({
                    'linear_velocity': 0.22,
                    'angular_velocity': 0.5,
                })

                self.assertTrue(self.wait_until(lambda: (
                    publisher.messages
                    and not orchestrator._safety_zero_inflight
                )))
                self.assertFalse(any(
                    command.linear.x > 0.0
                    or command.angular.z > 0.0
                    for command in publisher.messages
                ))
                self.assertEqual(0.0, publisher.messages[-1].linear.x)
                self.assertEqual(0.0, publisher.messages[-1].angular.z)

    def test_manual_at_heartbeat_boundary_latches_watchdog_durably(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        orchestrator._control_clock = lambda: 10.0
        publisher = FakePublisher()
        orchestrator.leader_cmd_pub = publisher
        orchestrator._refresh_safety_publisher_snapshot()

        orchestrator._handle_control_leader({
            'linear_velocity': 0.22,
            'angular_velocity': 0.5,
        })

        self.assertTrue(orchestrator.control_watchdog_tripped)
        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assertFalse(any(
            command.linear.x > 0.0 or command.angular.z > 0.0
            for command in publisher.messages
        ))
        self.assert_ambiguous_lifecycle_is_secured(
            orchestrator, 'task-a', publisher
        )
        self.assertEqual(
            ROS['orchestrator'].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertFalse(orchestrator.task_dispatched)

    def test_manual_publish_error_puts_zero_after_possible_delivery(self):
        orchestrator = make_orchestrator()
        publisher = RecordThenRaisePublisher()
        orchestrator.leader_cmd_pub = publisher
        orchestrator._refresh_safety_publisher_snapshot()

        with self.assertRaisesRegex(
            RuntimeError, 'delivery outcome is unknown'
        ):
            orchestrator._handle_control_leader({
                'linear_velocity': 0.22,
                'angular_velocity': 0.5,
            })

        self.assertTrue(self.wait_until(lambda: (
            len(publisher.messages) >= 2
            and not orchestrator._safety_zero_inflight
        )))
        possible_delivery = publisher.messages[0]
        self.assertEqual(0.22, possible_delivery.linear.x)
        self.assertEqual(0.5, possible_delivery.angular.z)
        self.assertEqual(1, orchestrator._emergency_order_generation)
        self.assert_ambiguous_lifecycle_is_secured(
            orchestrator, 'task-a', publisher
        )

    def test_watchdog_preempts_a_blocked_manual_command_and_rezeroes_it(self):
        orchestrator = make_orchestrator()
        clock = [0.0]
        orchestrator._control_clock = lambda: clock[0]
        release_manual = threading.Event()
        publisher = FirstPublishBlockingPublisher(release_manual)
        orchestrator.leader_cmd_pub = publisher
        orchestrator._refresh_safety_publisher_snapshot()
        manual = threading.Thread(
            target=orchestrator._handle_control_leader,
            args=({
                'linear_velocity': 0.22,
                'angular_velocity': 0.5,
            },),
        )

        manual.start()
        self.assertTrue(publisher.entered.wait(0.5))
        try:
            clock[0] = 10.0
            self.assertTrue(orchestrator._check_control_watchdog(now=10.0))
            self.assertTrue(self.wait_until(lambda: (
                publisher.calls >= 2
                and publisher.messages
                and orchestrator.emergency_stop_pub.messages
                and orchestrator.emergency_stop_pub.messages[-1].data is True
            )))
            self.assertTrue(manual.is_alive())
            self.assertTrue(all(
                command.linear.x == 0.0
                and command.angular.z == 0.0
                for command in publisher.messages
            ))
        finally:
            release_manual.set()
            manual.join(1.0)

        self.assertFalse(manual.is_alive())
        self.assertTrue(self.wait_until(lambda: (
            publisher.calls >= 3
            and not orchestrator._safety_zero_inflight
        )))
        self.assertTrue(any(
            command.linear.x > 0.0
            for command in publisher.messages
        ))
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)
        self.assertTrue(orchestrator.control_watchdog_tripped)

    def test_completed_ordinary_stop_rezeroes_a_blocked_manual_command(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS['orchestrator'].TaskState.RUNNING
        orchestrator.task_dispatched = True
        release_manual = threading.Event()
        publisher = BlockingMotionWithFailedLateZeros(release_manual)
        orchestrator.leader_cmd_pub = publisher
        orchestrator._refresh_safety_publisher_snapshot()
        initial_stop_sequence = orchestrator._ordinary_stop_sequence
        initial_emergency_generation = (
            orchestrator._emergency_order_generation
        )
        manual = threading.Thread(
            target=orchestrator._handle_control_leader,
            args=({
                'linear_velocity': 0.22,
                'angular_velocity': 0.5,
            },),
        )

        manual.start()
        self.assertTrue(publisher.entered.wait(0.5))
        try:
            orchestrator._handle_stop_task({'task_id': 'task-a'})
            self.assertTrue(self.wait_until(lambda: (
                not orchestrator.ordinary_stop_active
                and publisher.messages
                and not orchestrator._safety_zero_inflight
            )))
            completed_stop_message_count = len(publisher.messages)
            self.assertGreater(
                orchestrator._ordinary_stop_sequence,
                initial_stop_sequence,
            )
            self.assertTrue(manual.is_alive())
            self.assertTrue(all(
                command.linear.x == 0.0
                and command.angular.z == 0.0
                for command in publisher.messages
            ))
        finally:
            release_manual.set()
            manual.join(1.0)

        self.assertFalse(manual.is_alive())
        self.assertTrue(self.wait_until(lambda: (
            len(publisher.messages) > completed_stop_message_count
            and publisher.failed_zero_count == 2
            and not orchestrator._safety_zero_inflight
            and not orchestrator._emergency_publication_debts
            and all(
                stop_publisher.messages
                for stop_publisher in orchestrator.behavior_stop_pubs
            )
            and orchestrator.emergency_stop_pub.messages
            and orchestrator.emergency_stop_pub.messages[-1].data is True
        )))
        self.assertEqual(
            initial_emergency_generation + 1,
            orchestrator._emergency_order_generation,
        )
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertGreater(publisher.messages[-1].linear.x, 0.0)

        orchestrator._safety_stop_zero_timer(None)
        self.assertTrue(self.wait_until(lambda: (
            len(publisher.messages) > completed_stop_message_count + 1
            and any(
                command.linear.x > 0.0
                for command in publisher.messages
            )
            and publisher.messages[-1].linear.x == 0.0
            and publisher.messages[-1].angular.z == 0.0
            and not orchestrator._safety_zero_inflight
        )))
        positive_positions = [
            index for index, command in enumerate(publisher.messages)
            if command.linear.x > 0.0
        ]
        self.assertEqual(1, len(positive_positions))
        self.assertLess(
            positive_positions[0], len(publisher.messages) - 1
        )
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)
        self.assert_ambiguous_lifecycle_is_secured(
            orchestrator, 'task-a', publisher
        )

    def test_late_manual_leader_command_is_followed_by_a_zero(self):
        orchestrator = make_orchestrator()
        orchestrator.current_task_type = 'formation'
        release = threading.Event()
        leader_publisher = BlockingPublisher(release)
        orchestrator.leader_cmd_pub = leader_publisher
        orchestrator._refresh_safety_publisher_snapshot()
        manual = threading.Thread(
            target=orchestrator._handle_control_leader,
            args=({
                'linear_velocity': 0.22,
                'angular_velocity': 0.5,
            },),
        )
        manual.start()
        self.assertTrue(leader_publisher.entered.wait(0.5))

        orchestrator._safety_stop_request_callback(String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'task_id': 'task-a',
            'request_id': 'leader-race-request',
        })))
        self.assertTrue(self.wait_until(lambda: leader_publisher.calls >= 2))

        release.set()
        manual.join(0.5)
        self.assertFalse(manual.is_alive())
        self.assertTrue(self.wait_until(lambda: (
            leader_publisher.calls >= 3
            and not orchestrator._safety_zero_inflight
        )))
        self.assertTrue(any(
            command.linear.x > 0.0
            for command in leader_publisher.messages
        ))
        final_command = leader_publisher.messages[-1]
        self.assertEqual(0.0, final_command.linear.x)
        self.assertEqual(0.0, final_command.angular.z)

    def test_missing_first_heartbeat_latches_at_startup_grace_boundary(self):
        orchestrator = make_orchestrator(heartbeat_ready=False)
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True
        clock = [100.0]
        orchestrator._control_clock = lambda: clock[0]
        orchestrator.control_watchdog_started_at = 100.0

        self.assertFalse(orchestrator._check_control_watchdog(now=114.999))
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.emergency_stop_pub.messages)

        self.assertTrue(orchestrator._check_control_watchdog(now=115.0))

        self.assertTrue(orchestrator.control_watchdog_tripped)
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertEqual(
            ROS["orchestrator"].TaskState.STOPPED,
            orchestrator.task_state,
        )
        self.assertTrue(orchestrator.emergency_stop_pub.messages[-1].data)
        for publisher in orchestrator.behavior_stop_pubs:
            payload = json.loads(publisher.messages[-1].data)
            self.assertEqual("task-a", payload["task_id"])

    def test_heartbeat_callback_validates_the_absolute_deadline(self):
        orchestrator = make_orchestrator(heartbeat_ready=False)
        orchestrator._control_clock = lambda: 100.0

        for deadline in (math.nan, 100.0, 115.001):
            with self.subTest(deadline=deadline):
                orchestrator._control_heartbeat_callback(
                    Float64(data=deadline)
                )
                self.assertFalse(orchestrator.control_heartbeat_seen)
                self.assertIsNone(orchestrator.last_control_heartbeat)
                self.assertIsNone(
                    orchestrator.last_control_heartbeat_deadline
                )

        orchestrator._control_heartbeat_callback(Float64(data=114.5))

        self.assertTrue(orchestrator.control_heartbeat_seen)
        self.assertEqual(100.0, orchestrator.last_control_heartbeat)
        self.assertEqual(
            114.5, orchestrator.last_control_heartbeat_deadline
        )

    def test_start_task_without_heartbeat_fails_without_publishing(self):
        orchestrator = make_orchestrator(
            task_id=None, heartbeat_ready=False
        )
        orchestrator.current_task_type = None
        orchestrator.current_task_config = {}
        orchestrator.task_state = ROS["orchestrator"].TaskState.IDLE
        publisher = orchestrator.behavior_start_pubs["follow_leader"]

        orchestrator._handle_start_task({
            "task_id": "no-worker-heartbeat",
            "task_type": "follow_leader",
        })

        self.assertEqual(
            ROS["orchestrator"].TaskState.FAILED,
            orchestrator.task_state,
        )
        self.assertEqual(
            "Control heartbeat watchdog is not armed or is stale",
            orchestrator.task_error,
        )
        self.assertFalse(orchestrator.task_dispatched)
        self.assertFalse(orchestrator.task_ever_dispatched)
        self.assertFalse(publisher.messages)

    def test_late_heartbeat_requires_explicit_reset_and_stale_reset_fails(self):
        orchestrator = make_orchestrator(heartbeat_ready=False)
        clock = [10.0]
        orchestrator._control_clock = lambda: clock[0]
        orchestrator._control_heartbeat_callback(Float64(data=24.0))
        self.assertTrue(orchestrator._check_control_watchdog(now=20.0))

        clock[0] = 20.0
        orchestrator._handle_reset_emergency_stop({})
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.control_watchdog_tripped)

        clock[0] = 21.0
        orchestrator._control_heartbeat_callback(Float64(data=35.0))
        self.assertTrue(orchestrator.emergency_stop_active)
        orchestrator._handle_reset_emergency_stop({})
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.control_watchdog_tripped)
        self.assertFalse(orchestrator.emergency_stop_pub.messages[-1].data)


class BehaviorLifecycleTests(unittest.TestCase):
    @staticmethod
    def _transport_estop_controller(publishers):
        controller = ROS['transport'].CollaborativeTransport.__new__(
            ROS['transport'].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.control_cycle_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.phase_lock = threading.Lock()
        controller.current_task_id = 'transport-estop-test'
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.emergency_reset_pending = False
        controller.phase = ROS['transport'].TransportPhase.PUSH
        controller.robot_namespaces = list(publishers)
        controller.robot_count = len(publishers)
        controller.cmd_vel_pubs = dict(publishers)
        controller.avoidance_modules = {
            namespace: FakeAvoidance() for namespace in publishers
        }
        controller.transport_last_commands = {}
        controller.normal_stop_timeout_wall_s = 0.05
        controller._finalize_emergency_reset = lambda: None
        controller._ensure_safety_fanout()
        for namespace, publisher in publishers.items():
            controller._register_safety_lane(namespace, publisher)
        controller._refresh_safety_publisher_snapshot(
            tuple(publishers.items())
        )
        return controller

    @staticmethod
    def _transport_layout_controller():
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = -0.8
        controller.target_y = -3.0
        controller.object_name = "transport_object"
        controller.object_half_width = 0.20
        controller.object_half_height = 0.20
        controller.object_velocity = np.zeros(2)
        controller.robot_forward_contact_extent = 0.038
        controller.robot_rear_contact_extent = 0.102
        controller.transport_contact_slop = 0.005
        controller.transport_companion_contact_distance = 0.145
        controller.transport_chain_staging_contact_tolerance = 0.003
        controller.max_pushing_robots = 2
        controller.transport_payload_lane_offset = 0.15
        controller.transport_staging_clearance = 0.25
        controller.transport_chain_staging_spacing = 0.31
        controller.transport_large_fleet_staging_spacing = 0.38
        controller.transport_large_fleet_staging_lateral_offset = 0.22
        controller.transport_chain_assembly_gap = 0.10
        controller.transport_roles = {}
        controller.transport_staged = set()
        controller.transport_pre_staged = set()
        controller.transport_chain_released = set()
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.arena_profile = "swarm_arena"
        controller.transport_route_obstacle_clearance = 0.30
        controller.transport_payload_target_margin = 0.05
        controller.transport_payload_route_awareness = 0.80
        controller.transport_payload_route_lidar_offset = 0.032
        controller.transport_payload_route_margin = 0.05
        controller.spawn_exclusion_zones = []
        controller.model_poses = {}
        controller.model_lock = threading.Lock()
        return controller

    @staticmethod
    def _transport_search_controller(robot_count):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        namespaces = ["tb3_{}".format(index) for index in range(robot_count)]
        controller.command_lock = threading.RLock()
        controller.control_cycle_lock = threading.Lock()
        controller.phase_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.model_lock = threading.Lock()
        controller.command_epoch = 4
        controller.current_task_id = "search-task"
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.phase = ROS["transport"].TransportPhase.SEARCH
        controller.robot_namespaces = namespaces
        controller.robot_positions = {
            namespace: np.array([
                -1.5 + 0.35 * index,
                -1.0 + 0.45 * (index % 2),
            ])
            for index, namespace in enumerate(namespaces)
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in namespaces
        }
        controller.robot_odom_received_at = {
            namespace: 0.0 for namespace in namespaces
        }
        controller.transport_odom_timeout = 2.0
        controller.cmd_vel_pubs = {
            namespace: FakePublisher() for namespace in namespaces
        }
        controller._ensure_safety_fanout()
        controller._safety_publish_lane_factory = FakeSafetyLane
        for namespace, publisher in controller.cmd_vel_pubs.items():
            controller._register_safety_lane(namespace, publisher)
        controller._refresh_safety_publisher_snapshot(
            tuple(controller.cmd_vel_pubs.items())
        )
        controller.avoidance_modules = {
            namespace: FakeAvoidance() for namespace in namespaces
        }
        controller.transport_last_commands = {}
        controller.transport_discovery = None
        controller.discovery_pub = FakePublisher()
        controller.object_position = np.array([20.0, 20.0])
        controller.object_found = True
        controller.object_error = None
        controller.failure_reason = None
        controller.model_states_received_at = ROS["transport"].time.monotonic()
        controller.model_states_timeout_wall_s = 0.75
        controller.sensing_range = 0.6
        controller.vmax = 0.16
        controller.search_speed = 0.12
        controller.search_angular_gain = 1.4
        controller.search_max_angular_speed = 0.8
        controller.search_waypoint_tolerance = 0.20
        controller.search_lane_overlap = 0.75
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.arena_profile = "swarm_arena"
        controller.robot_radius = 0.11
        controller.object_name = "transport_object"
        controller.transport_route_obstacle_clearance = 0.30
        controller.spawn_exclusion_zones = []
        controller.model_poses = {}
        controller._reset_search_routes()
        return controller

    @staticmethod
    def _mutual_rendezvous_controller():
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.16
        controller.transport_chain_staging_speed = 0.08
        controller.transport_route_robot_clearance = 0.32
        controller.transport_rendezvous_clearance_hysteresis = 0.04
        controller.transport_rendezvous_release_hold_time = 0.40
        controller.transport_rendezvous_yield_distance = 0.28
        controller.transport_rendezvous_recoveries = {}
        controller.transport_rendezvous_recovery_cooldowns = {}
        controller.transport_rendezvous_pair_decisions = {}
        controller.transport_pre_staged = set()

        # Reproduce the N=10 conflict: robot 0 has to continue through robot
        # 5's present corridor, while robot 5 can clear it on a shorter leg.
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_5": np.array([0.30, 0.0]),
        }
        for index in (1, 2, 3, 4, 6, 7, 8, 9):
            positions["tb3_{}".format(index)] = np.array([
                2.0 + 0.50 * index,
                1.5 + 0.30 * (index % 2),
            ])
        destinations = {
            "tb3_0": np.array([0.441, 0.0]),
            "tb3_5": np.array([0.090, 0.0]),
        }
        targets = {
            namespace: {
                "position": destination.copy(),
                "staging_position": destination.copy(),
            }
            for namespace, destination in destinations.items()
        }
        controller.transport_assembly_route_states = {
            namespace: {
                "kind": "rendezvous",
                "target": destination.copy(),
                "waypoints": [destination.copy()],
                "waypoint_index": 0,
            }
            for namespace, destination in destinations.items()
        }
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )
        return controller, positions, targets

    @staticmethod
    def _transport_chain_engagement_controller():
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.command_lock = threading.RLock()
        controller.phase_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.avoidance_modules = {}
        controller.transport_engaged = set()
        controller.transport_engagement_complete = False
        controller.transport_engagement_ready_since = None
        controller.transport_engagement_last_ready = {}
        controller.transport_engagement_anchors = {}
        controller.transport_engagement_parent_distances = {}
        controller.transport_engagement_hold_time = 10.0
        controller.transport_engagement_release_hold_time = 0.35
        controller.transport_synchronized_push_started = False
        controller.transport_contact_slop = 0.005
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_closing_speed = 0.018
        controller.vmax = 0.20

        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([-0.145, 0.0]),
        }
        yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_depth": 0,
                "chain_index": 0,
                "parent_namespace": None,
                "position": np.array([0.0, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "role": "companion_push",
                "chain_depth": 1,
                "chain_index": 0,
                "parent_namespace": "tb3_0",
                "parent_position": positions["tb3_0"],
                "position": np.array([-0.145, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        raw_contact = {"tb3_0": True, "tb3_1": True}
        published = {}

        controller._transport_targets = lambda *_args: targets
        controller._payload_contact_near = (
            lambda *_args, **_kwargs: raw_contact["tb3_0"]
        )
        controller._companion_engagement_geometry = (
            lambda _position, parent, direction, **_kwargs: (
                raw_contact["tb3_1"],
                np.asarray(parent) - np.asarray(direction) * (
                    controller.transport_companion_contact_distance
                    - controller.transport_contact_slop
                ),
            )
        )
        controller._robot_lidar_masks = lambda *_args: ()
        controller._apply_transport_avoidance = (
            lambda _namespace, command, *_args, **_kwargs: command
        )
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )
        return controller, positions, yaws, raw_contact, published

    @staticmethod
    def _transport_publish_controller(targets, payload_contacts):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.phase_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.robot_velocities = {}
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.object_yaw = 0.0
        controller.robot_radius = 0.11
        controller.vmax = 0.16
        controller.transport_payload_recovery_margin = 0.05
        controller.transport_contact_closing_speed = 0.018
        controller.transport_min_useful_push_speed = 0.035
        controller.transport_push_angular_limit = 0.25

        contact_by_position = {
            tuple(np.asarray(position, dtype=float)): ready
            for position, ready in payload_contacts
        }
        published = {}
        controller._transport_targets = lambda *_args: targets
        controller._payload_contact_near = (
            lambda position, *_args, **_kwargs:
            contact_by_position.get(
                tuple(np.asarray(position, dtype=float)), False
            )
        )
        controller._slot_contact_ready = (
            lambda *_args, **_kwargs: True
        )
        controller._goal_push_velocity = (
            lambda sampled_velocity, *_args, **_kwargs: sampled_velocity
        )
        controller._keep_chain_push_pressure = (
            lambda _namespace, command, *_args, **_kwargs: command
        )
        controller._apply_transport_avoidance = (
            lambda _namespace, command, *_args, **_kwargs: command
        )
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )
        return controller, published

    @classmethod
    def _connected_transport_publish_controller(cls):
        positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.385, 0.0]),
        }
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_depth": 0,
                "chain_index": 0,
                "max_chain_depth": 1,
                "parent_namespace": None,
                "parent_position": np.zeros(2),
                "position": positions["tb3_0"],
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "role": "companion_push",
                "chain_depth": 1,
                "chain_index": 0,
                "max_chain_depth": 1,
                "parent_namespace": "tb3_0",
                "parent_position": positions["tb3_0"],
                "position": positions["tb3_1"],
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        controller, published = cls._transport_publish_controller(
            targets, [(positions["tb3_0"], True)]
        )
        controller.transport_engaged = set(positions)
        controller.transport_physical_engaged = set(positions)
        controller.transport_synchronized_push_started = True
        controller.transport_push_reference_speed = 0.030
        controller.transport_useful_contributors = set()
        controller.transport_all_pushers_confirmed = False
        controller.transport_control_sequence = 0
        empty_neighbours = {namespace: set() for namespace in positions}
        controller._transport_neighbours = lambda _targets: (
            empty_neighbours, empty_neighbours, empty_neighbours
        )
        controller._nearby_chain_contacts = lambda *_args, **_kwargs: ()
        controller._parallel_lane_contacts = lambda *_args, **_kwargs: ()
        controller._robot_lidar_masks = lambda *_args, **_kwargs: ()

        def to_twist(vx, _vy, _yaw):
            command = Twist()
            command.linear.x = vx
            return command

        controller._holonomic_to_diff_drive = to_twist
        controller._stabilize_push_steering = lambda command: command
        return controller, positions, targets, published

    @staticmethod
    def _loaded_transport_topology(
        robot_count, link_gaps=None, lane_count=None
    ):
        """Build one or two ordered push lanes with measured bumper gaps."""
        link_gaps = link_gaps or {}
        if lane_count is None:
            lane_count = 2
        chain_count = min(lane_count, robot_count)
        chain_lengths = [
            robot_count // chain_count
            + (1 if lane < robot_count % chain_count else 0)
            for lane in range(chain_count)
        ]
        positions = {}
        targets = {}
        robot_index = 0
        for lane, chain_length in enumerate(chain_lengths):
            parent = None
            parent_x = 0.0
            for depth in range(chain_length):
                namespace = "tb3_{}".format(robot_index)
                if parent is None:
                    x_position = 0.0
                else:
                    gap = float(link_gaps.get(namespace, 0.140))
                    x_position = parent_x - gap
                positions[namespace] = np.array([
                    x_position, 0.30 * lane,
                ])
                targets[namespace] = {
                    "role": (
                        "payload_push" if depth == 0
                        else "companion_push"
                    ),
                    "chain_index": lane,
                    "chain_depth": depth,
                    "max_chain_depth": chain_length - 1,
                    "parent_namespace": parent,
                    "parent_position": (
                        np.zeros(2) if parent is None
                        else positions[parent]
                    ),
                    "position": positions[namespace],
                    "push_direction": np.array([1.0, 0.0]),
                }
                parent = namespace
                parent_x = x_position
                robot_index += 1
        return positions, targets

    def test_companion_queue_stage_requires_live_parent_contact(self):
        controller = self._transport_layout_controller()
        controller.transport_assembly_contact_lateral_tolerance = 0.022
        position = np.array([-0.140, 0.0])
        target = {
            "role": "companion_push",
            "parent_position": np.array([0.0, 0.0]),
            "position": position.copy(),
            "assembly_position": position.copy(),
            "push_direction": np.array([1.0, 0.0]),
        }

        self.assertTrue(controller._slot_contact_ready(
            position, 0.0, np.zeros(2), 0.0, target,
            require_inward_heading=False,
            contact_margin=0.025,
            staging=True,
        ))
        route_destination = (
            controller._companion_assembly_route_destination(target)
        )
        self.assertAlmostEqual(-0.210, route_destination[0])
        self.assertAlmostEqual(0.0, route_destination[1])

        # The child is still on its world slot, but its parent was bumped
        # forward. A fixed-position check alone must not release this queue.
        target["parent_position"] = np.array([0.010, 0.0])
        self.assertFalse(controller._slot_contact_ready(
            position, 0.0, np.zeros(2), 0.0, target,
            require_inward_heading=False,
            contact_margin=0.025,
            staging=True,
        ))

        # Run 19 produced two independently valid fixed poses with 18 mm of
        # live lateral error. That is a shallow, safe contact angle; the old
        # 15 mm gate left the otherwise settled second lane deadlocked.
        run19_position = np.array([-0.983, -0.968])
        run19_target = {
            "role": "companion_push",
            "parent_position": np.array([-0.965, -1.108]),
            "position": np.array([-0.980, -0.967]),
            "assembly_position": np.array([-0.980, -0.967]),
            "push_direction": np.array([0.0, -1.0]),
        }
        self.assertTrue(controller._slot_contact_ready(
            run19_position,
            -math.pi / 2.0,
            np.zeros(2),
            0.0,
            run19_target,
            require_inward_heading=True,
            contact_margin=0.025,
            staging=True,
        ))
        run19_position[0] -= 0.005
        self.assertFalse(controller._slot_contact_ready(
            run19_position,
            -math.pi / 2.0,
            np.zeros(2),
            0.0,
            run19_target,
            require_inward_heading=True,
            contact_margin=0.025,
            staging=True,
        ))

    def test_companion_staging_capture_does_not_claim_engagement(self):
        controller = self._transport_layout_controller()
        controller.transport_assembly_contact_lateral_tolerance = 0.022
        controller.transport_companion_engagement_angle = 0.35
        controller.transport_engagement_release_angle_margin = 0.30
        parent = np.array([0.0, 0.0])
        child = np.array([-0.1476, 0.0])
        target = {
            "role": "companion_push",
            "parent_position": parent,
            "position": np.array([-0.145, 0.0]),
            "assembly_position": np.array([-0.1476, 0.011]),
            "push_direction": np.array([1.0, 0.0]),
        }

        # Gazebo can settle a correctly aligned child a few millimetres
        # outside the physical engagement gate. It may leave assembly staging,
        # but that same pose is not physical engagement.
        self.assertTrue(controller._slot_contact_ready(
            child, 0.0, np.zeros(2), 0.0, target,
            require_inward_heading=True,
            contact_margin=0.025,
            staging=True,
        ))
        engaged, _ = controller._companion_engagement_geometry(
            child, parent, target["push_direction"]
        )
        self.assertFalse(engaged)

        # Keep the staging band narrow even when an object made without the
        # normal constructor carries a bad oversized value.
        controller.transport_chain_staging_contact_tolerance = 0.05
        child = np.array([-0.1481, 0.0])
        target["assembly_position"] = child.copy()
        self.assertFalse(controller._slot_contact_ready(
            child, 0.0, np.zeros(2), 0.0, target,
            require_inward_heading=True,
            contact_margin=0.025,
            staging=True,
        ))

    def test_transport_live_defaults_follow_burger_contact_geometry(self):
        controller = ROS["transport"].CollaborativeTransport()

        self.assertEqual(2, controller.max_pushing_robots)
        self.assertAlmostEqual(0.038, controller.robot_forward_contact_extent)
        self.assertAlmostEqual(0.102, controller.robot_rear_contact_extent)
        self.assertAlmostEqual(0.005, controller.transport_contact_slop)
        expected_distance = (
            controller.robot_forward_contact_extent
            + controller.robot_rear_contact_extent
            + controller.transport_contact_slop
        )
        self.assertAlmostEqual(0.145, expected_distance)
        self.assertAlmostEqual(
            expected_distance,
            controller.transport_companion_contact_distance,
        )
        self.assertAlmostEqual(
            0.003,
            controller.transport_chain_staging_contact_tolerance,
        )
        self.assertAlmostEqual(
            0.018,
            controller.transport_companion_preload,
        )
        self.assertAlmostEqual(
            0.018,
            controller.transport_contact_closing_speed,
        )
        self.assertAlmostEqual(
            0.025, controller.object_lidar_closer_tolerance
        )
        self.assertAlmostEqual(
            0.035, controller.object_lidar_docking_closer_tolerance
        )
        self.assertLess(
            controller.object_lidar_docking_closer_tolerance,
            controller.object_lidar_contact_closer_tolerance,
        )
        self.assertAlmostEqual(
            0.025,
            controller.transport_push_recovery_closing_speed,
        )
        self.assertGreater(
            controller.transport_push_recovery_closing_speed,
            controller.transport_contact_closing_speed,
        )
        self.assertAlmostEqual(
            0.018, controller.transport_push_ramp_initial_speed
        )
        self.assertAlmostEqual(
            0.75, controller.transport_all_push_hold_time
        )
        self.assertAlmostEqual(
            0.010, controller.transport_terminal_closing_speed
        )
        self.assertAlmostEqual(
            0.10, controller.transport_arrival_release_margin
        )
        self.assertAlmostEqual(
            ROS["transport"].DEFAULT_ENGAGEMENT_HOLD_TIME,
            controller.transport_engagement_hold_time,
        )
        self.assertAlmostEqual(
            ROS["transport"].DEFAULT_COMPRESSION_TRACKING_TOLERANCE,
            controller.transport_compression_tracking_tolerance,
        )

    def test_transport_avoidance_uses_one_complete_pose_snapshot(self):
        class RecordingAvoidance:
            def __init__(self):
                self.fleet_updates = []
                self.pose_updates = []

            def update_robot_positions(self, points):
                self.fleet_updates.append({
                    name: (point.x, point.y)
                    for name, point in points
                })

            def set_position(self, x, y, yaw):
                self.pose_updates.append((x, y, yaw))

        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.cmd_vel_pubs = {
            "tb3_0": FakePublisher(),
            "tb3_1": FakePublisher(),
        }
        controller.robot_positions = {
            "tb3_0": np.array([50.0, 50.0]),
            "tb3_1": np.array([60.0, 60.0]),
        }
        controller.robot_yaws = {"tb3_0": 2.0, "tb3_1": 2.5}
        controller.robot_velocities = {
            "tb3_0": np.zeros(2),
            "tb3_1": np.zeros(2),
        }
        controller.robot_odom_received_at = {}
        controller.avoidance_modules = {
            "tb3_0": RecordingAvoidance(),
            "tb3_1": RecordingAvoidance(),
        }
        positions = {
            "tb3_0": np.array([1.0, -2.0]),
            "tb3_1": np.array([3.5, 4.25]),
        }
        yaws = {"tb3_0": 0.25, "tb3_1": -0.75}

        controller._sync_avoidance_snapshot(positions, yaws)

        expected_fleet = {
            "tb3_0": (1.0, -2.0),
            "tb3_1": (3.5, 4.25),
        }
        for namespace, avoidance in controller.avoidance_modules.items():
            self.assertEqual([expected_fleet], avoidance.fleet_updates)
            self.assertEqual([
                (
                    positions[namespace][0],
                    positions[namespace][1],
                    yaws[namespace],
                )
            ], avoidance.pose_updates)

        # A later odometry callback updates controller state only. It cannot
        # splice a newer x/y/yaw into the avoidance batch already installed.
        odometry = Odometry()
        odometry.pose.pose.position.x = 9.0
        odometry.pose.pose.position.y = 8.0
        odometry.pose.pose.orientation.z = math.sin(0.5)
        odometry.pose.pose.orientation.w = math.cos(0.5)
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=11.0,
            create=True,
        ):
            controller._odom_callback("tb3_0", odometry)

        self.assertTrue(np.allclose(
            np.array([9.0, 8.0]), controller.robot_positions["tb3_0"]
        ))
        for avoidance in controller.avoidance_modules.values():
            self.assertEqual([expected_fleet], avoidance.fleet_updates)
            self.assertEqual(1, len(avoidance.pose_updates))

    def test_transport_avoidance_prefers_explicit_payload_yaw(self):
        class RecordingAvoidance:
            def __init__(self):
                self.arguments = None

            def apply_avoidance(self, command, **kwargs):
                self.arguments = kwargs
                return command

        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.object_yaw = math.pi / 2.0
        controller.grf_object_radius = 0.25
        controller.object_avoidance_range = 0.85
        controller.object_lidar_tolerance = 0.08
        controller.object_lidar_closer_tolerance = 0.01
        controller.object_lidar_contact_closer_tolerance = 0.05
        controller.object_lidar_corner_radius = 0.08
        controller.object_lidar_envelope_tolerance = 0.10
        controller.object_lidar_envelope_closer_tolerance = 0.02
        controller.object_half_width = 0.20
        controller.object_half_height = 0.15
        avoidance = RecordingAvoidance()
        controller.avoidance_modules = {"tb3_0": avoidance}
        snapshot_yaw = 0.37

        command = Twist()
        returned = controller._apply_transport_avoidance(
            "tb3_0",
            command,
            np.array([0.5, -0.25]),
            object_yaw=snapshot_yaw,
        )

        self.assertIs(command, returned)
        masks = avoidance.arguments["lidar_range_mask"]
        self.assertGreaterEqual(len(masks), 1)
        self.assertAlmostEqual(snapshot_yaw, masks[0].yaw)
        self.assertNotAlmostEqual(controller.object_yaw, masks[0].yaw)

    def test_transport_search_moves_every_robot_on_persistent_coverage_lanes(self):
        for robot_count in (1, 3, 10):
            with self.subTest(robot_count=robot_count):
                controller = self._transport_search_controller(robot_count)

                controller._search_phase(controller.command_epoch)

                first_targets = {}
                for namespace, publisher in controller.cmd_vel_pubs.items():
                    self.assertEqual(1, len(publisher.messages))
                    command = publisher.messages[-1]
                    self.assertGreater(
                        abs(command.linear.x) + abs(command.angular.z),
                        0.01,
                    )
                    index = controller.search_route_indices[namespace]
                    first_targets[namespace] = (
                        controller.search_routes[namespace][index].copy()
                    )

                controller._search_phase(controller.command_epoch)

                for namespace, target in first_targets.items():
                    index = controller.search_route_indices[namespace]
                    np.testing.assert_allclose(
                        target,
                        controller.search_routes[namespace][index],
                    )

                lane_y = {
                    round(float(route[0][1]), 6)
                    for route in controller.search_routes.values()
                }
                self.assertEqual(robot_count, len(lane_y))

    def test_transport_rejects_malicious_avoidance_output_and_stops(self):
        class MaliciousAvoidance(FakeAvoidance):
            def __init__(self, bad_value):
                super().__init__()
                self.bad_value = bad_value

            def apply_avoidance(self, _command, *args, **kwargs):
                command = Twist()
                command.linear.x = self.bad_value
                return command

        for bad_value in (float("nan"), float("inf")):
            with self.subTest(value=bad_value):
                controller = self._transport_search_controller(2)
                controller.avoidance_modules["tb3_1"] = MaliciousAvoidance(
                    bad_value
                )
                controller.target_x = 2.0
                controller.target_y = 1.0
                controller._active_planner = "grf"
                controller._active_grf_iterations = 0
                controller.transport_roles = {}
                controller.status_pub = FakePublisher()

                controller._search_phase(controller.command_epoch)
                controller._publish_status(
                    controller.phase,
                    task_id=controller.current_task_id,
                    paused=False,
                )

                self.assertFalse(controller.is_running)
                self.assertEqual(
                    ROS["transport"].TransportPhase.FAILED,
                    controller.phase,
                )
                self.assertEqual("search-task", controller.current_task_id)
                self.assertIn("non-finite", controller.failure_reason)
                # The first robot's valid search command must not leak out
                # before the second robot's corrupt command is discovered.
                for publisher in controller.cmd_vel_pubs.values():
                    self.assertEqual(1, len(publisher.messages))
                    self.assertEqual(0.0, publisher.messages[0].linear.x)
                    self.assertEqual(0.0, publisher.messages[0].angular.z)
                status = json.loads(controller.status_pub.messages[-1].data)
                self.assertEqual("search-task", status["task_id"])
                self.assertEqual("FAILED", status["phase"])
                self.assertIn("non-finite", status["error"])

    def test_transport_command_gate_checks_every_twist_component(self):
        cases = (
            ("linear", "x", float("nan")),
            ("linear", "y", float("inf")),
            ("linear", "z", -float("inf")),
            ("angular", "x", float("nan")),
            ("angular", "y", float("inf")),
            ("angular", "z", -float("inf")),
            ("linear", "y", 0.01),
            ("linear", "z", -0.01),
            ("angular", "x", 0.01),
            ("angular", "y", -0.01),
            (
                "linear",
                "x",
                ROS["transport"].BURGER_MAX_LINEAR_SPEED + 0.01,
            ),
            (
                "angular",
                "z",
                ROS["transport"].BURGER_MAX_ANGULAR_SPEED + 0.01,
            ),
        )
        for vector, axis, value in cases:
            with self.subTest(vector=vector, axis=axis, value=value):
                controller = self._transport_search_controller(1)
                command = Twist()
                setattr(getattr(command, vector), axis, value)

                published = controller._publish_command(
                    "tb3_0", command, controller.command_epoch
                )

                self.assertFalse(published)
                self.assertFalse(controller.is_running)
                self.assertEqual(
                    ROS["transport"].TransportPhase.FAILED,
                    controller.phase,
                )
                self.assertEqual("search-task", controller.current_task_id)
                messages = controller.cmd_vel_pubs["tb3_0"].messages
                self.assertEqual(1, len(messages))
                self.assertEqual(0.0, messages[0].linear.x)
                self.assertEqual(0.0, messages[0].angular.z)

    def test_transport_control_exception_publishes_correlated_failure(self):
        controller = self._transport_search_controller(2)
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 0
        controller.transport_roles = {}
        controller.status_pub = FakePublisher()
        controller._control_loop_serialized = mock.Mock(
            side_effect=ValueError("malformed avoidance data")
        )

        controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED,
            controller.phase,
        )
        self.assertEqual("search-task", controller.current_task_id)
        for publisher in controller.cmd_vel_pubs.values():
            self.assertEqual(1, len(publisher.messages))
            self.assertEqual(0.0, publisher.messages[0].linear.x)
            self.assertEqual(0.0, publisher.messages[0].angular.z)
        self.assertEqual(1, len(controller.status_pub.messages))
        status = json.loads(controller.status_pub.messages[0].data)
        self.assertEqual("search-task", status["task_id"])
        self.assertEqual("FAILED", status["phase"])
        self.assertIn("invalid live data", status["error"])

    def test_transport_search_announces_the_nearest_finder_once(self):
        controller = self._transport_search_controller(3)
        controller.sensing_range = 1.0
        controller.object_position = np.array([0.0, 0.0])
        controller.robot_positions = {
            "tb3_0": np.array([0.50, 0.0]),
            "tb3_1": np.array([0.20, 0.0]),
            "tb3_2": np.array([-0.20, 0.0]),
        }
        controller.robot_odom_received_at = {
            namespace: 12.5 for namespace in controller.robot_namespaces
        }

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=12.5,
            create=True,
        ):
            controller._search_phase(controller.command_epoch)

        self.assertEqual(
            ROS["transport"].TransportPhase.APPROACH,
            controller.phase,
        )
        self.assertEqual(1, len(controller.discovery_pub.messages))
        notice = json.loads(controller.discovery_pub.messages[0].data)
        self.assertEqual("payload_found", notice["event"])
        self.assertEqual("search-task", notice["task_id"])
        self.assertEqual("tb3_1", notice["finder"])
        self.assertEqual(["tb3_0", "tb3_2"], notice["notified_robots"])
        self.assertEqual({"x": 0.0, "y": 0.0}, notice["object_position"])

        announced_again = controller._announce_payload_found(
            "tb3_2",
            0.2,
            controller.object_position,
            controller.command_epoch,
        )
        self.assertFalse(announced_again)
        self.assertEqual(1, len(controller.discovery_pub.messages))

        controller._reset_search_state()
        self.assertIsNone(controller.transport_discovery)
        self.assertEqual({}, controller.search_routes)

    def test_transport_search_requires_static_obstacle_line_of_sight(self):
        blocking_zone = {
            "name": "blocking_wall",
            "model": "obstacle_wall",
            "worlds": ["swarm_arena"],
            "shape": "box",
            "x": 0.0,
            "y": 0.0,
            "width": 0.20,
            "height": 1.0,
            "yaw": 0.0,
        }

        blocked = self._transport_search_controller(1)
        blocked.sensing_range = 1.5
        blocked.robot_positions["tb3_0"] = np.array([-0.5, 0.0])
        blocked.object_position = np.array([0.5, 0.0])
        blocked.spawn_exclusion_zones = [blocking_zone]

        blocked._search_phase(blocked.command_epoch)

        self.assertEqual(ROS["transport"].TransportPhase.SEARCH, blocked.phase)
        self.assertEqual([], blocked.discovery_pub.messages)
        self.assertEqual(1, len(blocked.cmd_vel_pubs["tb3_0"].messages))

        visible = self._transport_search_controller(1)
        visible.sensing_range = 1.5
        visible.robot_positions["tb3_0"] = np.array([-0.5, 0.0])
        visible.object_position = np.array([-0.5, 0.5])
        visible.spawn_exclusion_zones = [blocking_zone]

        visible._search_phase(visible.command_epoch)

        self.assertEqual(
            ROS["transport"].TransportPhase.APPROACH,
            visible.phase,
        )
        self.assertEqual(1, len(visible.discovery_pub.messages))
        notice = json.loads(visible.discovery_pub.messages[0].data)
        self.assertEqual("tb3_0", notice["finder"])

    def test_transport_search_stops_before_using_stale_odometry(self):
        controller = self._transport_search_controller(3)
        controller.robot_odom_received_at["tb3_1"] = -3.0
        failures = []
        controller._fail_transport = (
            lambda reason, epoch: failures.append((reason, epoch))
        )

        controller._search_phase(controller.command_epoch)

        self.assertEqual(1, len(failures))
        self.assertIn("stale: tb3_1", failures[0][0])
        self.assertEqual(controller.command_epoch, failures[0][1])
        self.assertTrue(all(
            not publisher.messages
            for publisher in controller.cmd_vel_pubs.values()
        ))

    def test_transport_search_routes_around_known_static_obstacles(self):
        controller = self._transport_search_controller(1)
        controller.object_name = "transport_object"
        controller.arena_profile = "swarm_arena"
        controller.transport_route_obstacle_clearance = 0.30
        controller.model_poses = {}
        controller.spawn_exclusion_zones = [{
            "name": "blocking_wall",
            "model": "obstacle_wall",
            "worlds": ["swarm_arena"],
            "shape": "box",
            "x": 0.0,
            "y": 0.0,
            "width": 0.2,
            "height": 1.0,
            "yaw": 0.0,
        }]

        route = controller._plan_search_segment(
            np.array([-0.20, 0.0]),
            np.array([1.0, 0.0]),
        )

        self.assertGreaterEqual(len(route), 2)
        self.assertGreater(abs(float(route[0][1])), 0.5)
        np.testing.assert_allclose([1.0, 0.0], route[-1])

    def test_transport_search_holds_and_retries_when_no_route_exists(self):
        controller = self._transport_search_controller(1)
        controller.arena_size = 2.0
        controller.arena_margin = 0.10
        controller.transport_route_obstacle_clearance = 0.10
        controller.robot_positions["tb3_0"] = np.array([-0.5, 0.0])
        destination = np.array([0.5, 0.0])
        controller.search_routes = {"tb3_0": [destination]}
        controller.search_route_indices = {"tb3_0": 0}
        controller.search_route_directions = {"tb3_0": 1}
        controller.search_route_signature = ("tb3_0",)
        controller.spawn_exclusion_zones = [{
            "name": "sealed_wall",
            "model": "obstacle_wall",
            "worlds": ["swarm_arena"],
            "shape": "box",
            "x": 0.0,
            "y": 0.0,
            "width": 0.20,
            "height": 2.0,
            "yaw": 0.0,
        }]

        planner = ROS["transport"].plan_obstacle_aware_route
        with mock.patch.object(
            ROS["transport"],
            "plan_obstacle_aware_route",
            wraps=planner,
        ) as plan:
            controller._search_phase(controller.command_epoch)
            controller._search_phase(controller.command_epoch)

        self.assertEqual(2, plan.call_count)
        self.assertEqual([], controller.search_navigation_routes["tb3_0"])
        self.assertNotIn("tb3_0", controller.search_navigation_targets)
        commands = controller.cmd_vel_pubs["tb3_0"].messages
        self.assertEqual(2, len(commands))
        self.assertTrue(all(
            command.linear.x == 0.0 and command.angular.z == 0.0
            for command in commands
        ))

    def test_transport_rejects_a_missing_or_sunken_payload_pose(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.object_name = "transport_object"
        controller.object_rest_z = 0.10
        controller.object_z_tolerance = 0.05
        controller.object_position = np.array([8.0, 8.0])
        controller.object_yaw = 0.0
        controller.object_z = 0.10
        controller.object_error = None
        controller.object_found = False
        controller.obstacle_positions = []

        valid_pose = Pose()
        valid_pose.position.x = -0.8
        valid_pose.position.y = -1.6
        valid_pose.position.z = 0.10
        valid = ModelStates()
        valid.name = ["transport_object"]
        valid.pose = [valid_pose]
        controller._model_states_callback(valid)

        self.assertTrue(controller.object_found)
        np.testing.assert_allclose([-0.8, -1.6], controller.object_position)
        self.assertIsNone(controller.object_error)

        sunken_pose = Pose()
        sunken_pose.position.z = -0.101
        sunken = ModelStates()
        sunken.name = ["transport_object"]
        sunken.pose = [sunken_pose]
        controller._model_states_callback(sunken)

        self.assertFalse(controller.object_found)
        self.assertIsNone(controller.object_position)
        self.assertIn("supported floor plane", controller.object_error)

        controller._model_states_callback(ModelStates())
        self.assertFalse(controller.object_found)
        self.assertIsNone(controller.object_position)
        self.assertIn("missing", controller.object_error)

    def test_transport_truncated_model_states_invalidates_live_payload(self):
        controller = self._transport_search_controller(2)
        controller.object_position = np.array([3.0, 4.0])
        controller.object_found = True
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 0
        controller.transport_roles = {}
        controller.status_pub = FakePublisher()
        previous_stamp = controller.model_states_received_at
        truncated = ModelStates()
        truncated.name = ["transport_object"]
        truncated.pose = []

        controller._model_states_callback(truncated)

        self.assertIsNotNone(previous_stamp)
        self.assertIsNone(controller.model_states_received_at)
        self.assertIn(
            "truncated", controller.model_states_invalid_reason
        )
        self.assertFalse(controller.object_found)
        self.assertIsNone(controller.object_position)
        self.assertFalse(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED,
            controller.phase,
        )
        self.assertEqual("search-task", controller.current_task_id)
        self.assertIn("truncated", controller.failure_reason)
        self.assertIn(
            "truncated", controller._transport_model_state_error()
        )
        for publisher in controller.cmd_vel_pubs.values():
            self.assertEqual(1, len(publisher.messages))
            self.assertEqual(0.0, publisher.messages[0].linear.x)
            self.assertEqual(0.0, publisher.messages[0].angular.z)
        self.assertEqual(1, len(controller.status_pub.messages))
        status = json.loads(controller.status_pub.messages[0].data)
        self.assertEqual("search-task", status["task_id"])
        self.assertEqual("FAILED", status["phase"])
        self.assertIn("truncated", status["error"])

    def test_transport_duplicate_model_names_require_a_unique_snapshot(self):
        controller = self._transport_search_controller(1)
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller.object_rest_z = 0.10
        controller.object_z_tolerance = 0.05
        controller.status_pub = FakePublisher()
        first_pose = Pose()
        first_pose.position.x = -0.8
        first_pose.position.y = -1.6
        first_pose.position.z = 0.10
        conflicting_pose = Pose()
        conflicting_pose.position.x = 3.0
        conflicting_pose.position.y = 4.0
        conflicting_pose.position.z = 0.10
        duplicate = ModelStates()
        duplicate.name = ["transport_object", "transport_object"]
        duplicate.pose = [first_pose, conflicting_pose]

        controller._model_states_callback(duplicate)

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.object_found)
        self.assertIsNone(controller.object_position)
        self.assertIn("duplicate", controller.model_states_invalid_reason)
        self.assertIn("duplicate", controller.failure_reason)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED, controller.phase
        )
        for publisher in controller.cmd_vel_pubs.values():
            self.assertTrue(publisher.messages)
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)

        unique = ModelStates()
        unique.name = ["transport_object"]
        unique.pose = [first_pose]
        controller._model_states_callback(unique)
        self.assertIsNone(controller.model_states_invalid_reason)
        self.assertTrue(controller.object_found)
        np.testing.assert_allclose([-0.8, -1.6], controller.object_position)

    def test_transport_odometry_rejects_non_finite_raw_quaternion(self):
        controller = self._transport_search_controller(1)
        namespace = "tb3_0"
        old_position = controller.robot_positions[namespace].copy()
        old_yaw = controller.robot_yaws[namespace]
        controller.robot_odom_received_at[namespace] = 7.0
        message = Odometry()
        message.pose.pose.position.x = 9.0
        message.pose.pose.position.y = 8.0
        # This quaternion produces a finite atan2 result unless its raw
        # components are checked before yaw conversion.
        message.pose.pose.orientation.x = float("inf")
        message.pose.pose.orientation.y = 1.0
        message.pose.pose.orientation.z = 0.0
        message.pose.pose.orientation.w = 0.0

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=12.0,
            create=True,
        ):
            controller._odom_callback(namespace, message)

        np.testing.assert_allclose(
            old_position, controller.robot_positions[namespace]
        )
        self.assertEqual(old_yaw, controller.robot_yaws[namespace])
        self.assertEqual(
            7.0, controller.robot_odom_received_at[namespace]
        )

    def test_transport_rejects_missing_and_stale_odometry_before_roles(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([-0.8, -1.6])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.robot_namespaces = ["tb3_0", "tb3_1"]
        controller.robot_positions = {
            "tb3_0": np.array([-1.0, -1.0]),
            "tb3_1": np.array([-0.5, -1.0]),
        }
        controller.robot_yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        controller.robot_velocities = {
            "tb3_0": np.zeros(2), "tb3_1": np.zeros(2)
        }
        controller.robot_odom_received_at = {"tb3_0": 4.0}
        controller.transport_odom_timeout = 2.0
        failures = []
        controller._fail_transport = (
            lambda reason, epoch: failures.append((reason, epoch)) or True
        )
        controller._transport_targets = mock.Mock(
            side_effect=AssertionError("roles must not be assigned")
        )

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._approach_phase(7)

        self.assertEqual(1, len(failures))
        self.assertEqual(7, failures[0][1])
        self.assertIn("fresh odometry", failures[0][0])
        self.assertIn("missing: tb3_1", failures[0][0])
        self.assertIn("stale: tb3_0", failures[0][0])
        controller._transport_targets.assert_not_called()

    def test_transport_push_fails_closed_when_model_states_become_stale(self):
        controller = self._transport_search_controller(1)
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.model_states_received_at = 10.0
        controller.model_states_timeout_wall_s = 0.75
        controller._push_phase = mock.Mock()
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()

        with mock.patch.object(
            ROS["transport"].time, "monotonic", return_value=11.0,
        ):
            controller._control_loop(None)

        controller._push_phase.assert_not_called()
        self.assertFalse(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED,
            controller.phase,
        )
        self.assertIn("model states became stale", controller.failure_reason)
        command = controller.cmd_vel_pubs["tb3_0"].messages[-1]
        self.assertEqual(0.0, command.linear.x)
        self.assertEqual(0.0, command.angular.z)

    def test_transport_rechecks_model_freshness_before_first_batch_twist(self):
        controller = self._transport_search_controller(2)
        controller.model_states_received_at = 10.0
        controller.model_states_timeout_wall_s = 0.75
        controller.robot_odom_received_at = {
            namespace: 10.0 for namespace in controller.robot_namespaces
        }
        controller._sync_avoidance_snapshot = mock.Mock()
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()
        wall_clock = [10.0]

        def delayed_search(expected_epoch):
            wall_clock[0] = 11.0
            commands = {}
            for namespace in controller.robot_namespaces:
                command = Twist()
                command.linear.x = 0.12
                commands[namespace] = command
            controller._publish_command_batch(
                commands, expected_epoch, controller.robot_namespaces
            )

        controller._search_phase = delayed_search
        with mock.patch.object(
            ROS["transport"].time,
            "monotonic",
            side_effect=lambda: wall_clock[0],
        ), mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=10.0,
            create=True,
        ):
            controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED,
            controller.phase,
        )
        self.assertIn("model states became stale", controller.failure_reason)
        for publisher in controller.cmd_vel_pubs.values():
            self.assertTrue(publisher.messages)
            self.assertTrue(all(
                command.linear.x == 0.0 and command.angular.z == 0.0
                for command in publisher.messages
            ))

    def test_transport_rechecks_odometry_before_first_batch_twist(self):
        controller = self._transport_search_controller(2)
        controller.model_states_received_at = 10.0
        controller.robot_odom_received_at = {
            namespace: 10.0 for namespace in controller.robot_namespaces
        }
        controller.transport_odom_timeout = 2.0
        controller._sync_avoidance_snapshot = mock.Mock()
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()
        simulation_clock = [10.0]

        def delayed_search(expected_epoch):
            simulation_clock[0] = 13.0
            commands = {}
            for namespace in controller.robot_namespaces:
                command = Twist()
                command.angular.z = 0.4
                commands[namespace] = command
            controller._publish_command_batch(
                commands, expected_epoch, controller.robot_namespaces
            )

        controller._search_phase = delayed_search
        with mock.patch.object(
            ROS["transport"].time, "monotonic", return_value=10.0
        ), mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            side_effect=lambda: simulation_clock[0],
            create=True,
        ):
            controller._control_loop(None)

        self.assertFalse(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.FAILED,
            controller.phase,
        )
        self.assertIn("stale: tb3_0, tb3_1", controller.failure_reason)
        for publisher in controller.cmd_vel_pubs.values():
            self.assertTrue(publisher.messages)
            self.assertTrue(all(
                command.linear.x == 0.0 and command.angular.z == 0.0
                for command in publisher.messages
            ))

    def test_transport_zero_batch_is_allowed_with_stale_inputs(self):
        controller = self._transport_search_controller(2)
        controller.model_states_received_at = 10.0
        commands = {
            namespace: Twist() for namespace in controller.robot_namespaces
        }

        with mock.patch.object(
            ROS["transport"].time, "monotonic", return_value=20.0
        ):
            published = controller._publish_command_batch(
                commands,
                controller.command_epoch,
                controller.robot_namespaces,
            )

        self.assertTrue(published)
        self.assertTrue(controller.is_running)
        for publisher in controller.cmd_vel_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

    def test_transport_batch_does_not_split_at_the_freshness_boundary(self):
        controller = self._transport_search_controller(2)
        controller.model_states_received_at = 10.0
        controller.model_states_timeout_wall_s = 0.75
        controller.robot_odom_received_at = {
            namespace: 10.0 for namespace in controller.robot_namespaces
        }
        commands = {}
        for namespace in controller.robot_namespaces:
            command = Twist()
            command.linear.x = 0.12
            commands[namespace] = command

        samples = iter((10.74, 10.74, 10.76))
        with mock.patch.object(
            ROS["transport"].time,
            "monotonic",
            side_effect=lambda: next(samples, 10.76),
        ), mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=10.0,
            create=True,
        ):
            published = controller._publish_command_batch(
                commands,
                controller.command_epoch,
                controller.robot_namespaces,
            )

        self.assertTrue(published)
        self.assertTrue(controller.is_running)
        motion_states = [
            publisher.messages[-1].linear.x > 0.0
            for publisher in controller.cmd_vel_pubs.values()
        ]
        self.assertEqual([True, True], motion_states)

    def test_transport_push_uses_the_common_odometry_freshness_gate(self):
        controller = self._transport_search_controller(1)
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.model_states_received_at = 10.0
        controller.robot_odom_received_at["tb3_0"] = 4.0
        controller._push_phase = mock.Mock()
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()

        with mock.patch.object(
            ROS["transport"].time, "monotonic", return_value=10.0,
        ), mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=10.0,
            create=True,
        ):
            controller._control_loop(None)

        controller._push_phase.assert_not_called()
        self.assertFalse(controller.is_running)
        self.assertIn("stale: tb3_0", controller.failure_reason)

    def test_transport_payload_updates_cannot_erase_terminal_failure_reason(self):
        controller = self._transport_search_controller(1)
        controller.object_rest_z = 0.10
        controller.object_z_tolerance = 0.05
        controller.target_x = 1.0
        controller.target_y = 0.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 12
        controller.status_pub = FakePublisher()

        self.assertTrue(controller._fail_transport(
            "Transport target is infeasible", controller.command_epoch
        ))

        valid_pose = Pose()
        valid_pose.position.x = -0.8
        valid_pose.position.y = -1.6
        valid_pose.position.z = 0.10
        valid = ModelStates()
        valid.name = ["transport_object"]
        valid.pose = [valid_pose]
        controller._model_states_callback(valid)

        self.assertIsNone(controller.object_error)
        self.assertEqual(
            "Transport target is infeasible", controller.failure_reason
        )
        controller._publish_status(ROS["transport"].TransportPhase.FAILED)
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(
            "Transport target is infeasible", status["error"]
        )

    def test_transport_layout_accepts_feasible_ten_robot_fleet(self):
        controller = self._transport_layout_controller()
        object_position = np.array([-0.8, -1.6])
        positions = {
            "tb3_{}".format(index): np.array([
                -1.8 + 0.45 * (index % 5),
                -0.7 + 0.45 * (index // 5),
            ])
            for index in range(10)
        }

        targets = controller._transport_targets(
            list(positions), positions, object_position, 0.0
        )

        self.assertIsNone(controller._transport_target_error())
        self.assertIsNone(controller._transport_layout_error(
            targets, object_position, 0.0, fleet_size=10
        ))
        self.assertTrue(all(
            "final_position" in target for target in targets.values()
        ))

    def test_transport_layout_rejects_oversized_fleet_before_motion(self):
        controller = self._transport_layout_controller()
        object_position = np.array([-0.8, -1.6])
        positions = {
            "tb3_{}".format(index): np.array([
                -1.8 + 0.45 * (index % 7),
                -0.9 + 0.45 * (index // 7),
            ])
            for index in range(41)
        }

        targets = controller._transport_targets(
            list(positions), positions, object_position, 0.0
        )
        error = controller._transport_layout_error(
            targets, object_position, 0.0, fleet_size=41
        )

        self.assertIsNotNone(error)
        self.assertIn("infeasible for 41 robots", error)
        self.assertIn("usable arena", error)

    def test_transport_target_and_parking_check_static_obstacles(self):
        controller = self._transport_layout_controller()
        object_position = np.array([-0.8, -1.6])
        positions = {
            "tb3_0": np.array([-1.0, -0.8]),
            "tb3_1": np.array([-0.6, -0.8]),
            "tb3_2": np.array([-1.0, -0.3]),
            "tb3_3": np.array([-0.6, -0.3]),
        }
        targets = controller._transport_targets(
            list(positions), positions, object_position, 0.0
        )
        parking = next(
            target["staging_position"]
            for target in targets.values()
            if target["role"] == "companion_push"
        )
        controller.spawn_exclusion_zones = [{
            "name": "blocked_parking",
            "model": "obstacle_blocked_parking",
            "worlds": ["swarm_arena"],
            "shape": "circle",
            "x": float(parking[0]),
            "y": float(parking[1]),
            "radius": 0.10,
        }]

        self.assertIsNotNone(controller._transport_layout_error(
            targets, object_position, 0.0, fleet_size=4
        ))

        controller.spawn_exclusion_zones = []
        controller.target_x = float("nan")
        self.assertIn("target is infeasible", controller._transport_target_error())
        controller.target_x = 5.0
        self.assertIn("target is infeasible", controller._transport_target_error())

    def test_transport_assigns_every_robot_to_balanced_push_chains(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = -0.8
        controller.target_y = -3.0
        controller.object_half_width = 0.20
        controller.object_half_height = 0.20
        controller.robot_forward_contact_extent = 0.038
        controller.robot_rear_contact_extent = 0.102
        controller.transport_contact_slop = 0.005
        controller.transport_contact_heading_tolerance = 0.45
        controller.transport_companion_contact_distance = 0.145
        controller.max_pushing_robots = 2
        controller.transport_push_slot_angle = 0.65
        controller.transport_payload_lane_offset = 0.15
        controller.transport_orbit_radius = 0.60
        controller.transport_staging_clearance = 0.25
        controller.transport_engagement_speed = 0.10
        controller.transport_push_heading_cone = 0.18
        controller.transport_companion_preload = 0.04
        controller.transport_payload_recovery_margin = 0.05
        controller.transport_push_recovery_closing_speed = 0.025
        controller.transport_payload_contact_release_margin = 0.04
        controller.transport_companion_contact_distance = 0.145
        controller.transport_chain_staging_spacing = 0.31
        controller.transport_large_fleet_staging_spacing = 0.38
        controller.transport_large_fleet_staging_lateral_offset = 0.22
        controller.transport_chain_assembly_gap = 0.03
        controller.transport_roles = {}
        controller.vmax = 0.15

        object_pos = np.array([-0.8, -1.6])
        positions = {
            "tb3_0": np.array([-0.62, -1.30]),
            "tb3_1": np.array([-0.98, -1.30]),
            "tb3_2": np.array([-0.8, -0.75]),
            "tb3_3": np.array([-0.4, -0.75]),
            "tb3_4": np.array([-1.2, -0.75]),
            "tb3_5": np.array([-0.6, -0.35]),
            "tb3_6": np.array([-1.0, -0.35]),
        }
        targets = controller._transport_targets(
            list(positions), positions, object_pos, 0.0
        )
        payload_targets = [
            target for target in targets.values()
            if target["role"] == "payload_push"
        ]
        companion_targets = [
            target for target in targets.values()
            if target["role"] == "companion_push"
        ]

        self.assertEqual(7, len(targets))
        self.assertEqual(
            {3}, {target["max_chain_depth"] for target in targets.values()}
        )
        self.assertEqual(2, len(payload_targets))
        self.assertEqual(5, len(companion_targets))
        self.assertTrue(all(
            target["role"].endswith("push")
            for target in targets.values()
        ))
        self.assertEqual(
            {"tb3_0", "tb3_1"},
            {
                namespace for namespace, target in targets.items()
                if target["role"] == "payload_push"
            },
        )
        self.assertGreater(
            np.linalg.norm(
                payload_targets[0]["position"]
                - payload_targets[1]["position"]
            ),
            0.25,
        )

        goal, rear, lateral = controller._transport_frame(object_pos)
        self.assertAlmostEqual(
            float(np.dot(
                payload_targets[0]["position"] - object_pos, rear
            )),
            float(np.dot(
                payload_targets[1]["position"] - object_pos, rear
            )),
            places=6,
        )
        for target in payload_targets:
            staging_offset = (
                target["staging_position"] - target["position"]
            )
            self.assertAlmostEqual(
                0.25, float(np.dot(staging_offset, rear)), places=6
            )
            self.assertTrue(controller._slot_contact_ready(
                target["staging_position"],
                math.atan2(goal[1], goal[0]),
                object_pos,
                0.0,
                target,
                staging=True,
            ))
            recovery_position = (
                target["position"] + rear * 0.04
            )
            self.assertTrue(controller._slot_contact_ready(
                recovery_position,
                math.atan2(goal[1], goal[0]),
                object_pos,
                0.0,
                target,
                require_inward_heading=False,
                contact_margin=(
                    controller.transport_payload_recovery_margin
                ),
            ))
            self.assertFalse(controller._slot_contact_ready(
                target["position"] + rear * 0.06,
                math.atan2(goal[1], goal[0]),
                object_pos,
                0.0,
                target,
                require_inward_heading=False,
                contact_margin=(
                    controller.transport_payload_recovery_margin
                ),
            ))
            closing_velocity, allow_payload = controller._slot_velocity(
                target["staging_position"],
                object_pos,
                target,
                pushing=True,
            )
            self.assertTrue(allow_payload)
            closing_speed = closing_velocity.dot(
                ROS["transport"].Vec2(*goal)
            )
            self.assertGreater(closing_speed, 0.0)
            self.assertLessEqual(
                closing_speed,
                controller.transport_push_recovery_closing_speed + 1e-9,
            )
            slid_contact = target["position"] + np.array([0.03, 0.0])
            self.assertTrue(controller._payload_contact_near(
                slid_contact, object_pos, 0.0
            ))
            launch_gap = target["position"] + rear * 0.02
            self.assertFalse(controller._payload_contact_near(
                launch_gap,
                object_pos,
                0.0,
                margin=controller.transport_contact_slop,
            ))
            self.assertTrue(controller._payload_contact_near(
                launch_gap, object_pos, 0.0, margin=0.03
            ))
        chain_depths = {0: [], 1: []}
        payload_by_chain = {
            target["chain_index"]: target
            for target in payload_targets
        }
        for target in companion_targets:
            parent = targets[target["parent_namespace"]]
            self.assertEqual(target["chain_index"], parent["chain_index"])
            self.assertEqual(
                target["chain_depth"] - 1,
                parent["chain_depth"],
            )
            chain_depths[target["chain_index"]].append(
                target["chain_depth"]
            )
            parent_position = positions[target["parent_namespace"]]
            contact_offset = target["position"] - parent_position
            self.assertAlmostEqual(
                0.145, float(np.linalg.norm(contact_offset)), places=6
            )
            self.assertAlmostEqual(
                0.145, float(np.dot(contact_offset, rear)), places=6
            )
            expected_assembly_position = (
                payload_by_chain[target["chain_index"]]["position"]
                + rear * (
                    controller.transport_staging_clearance
                    + target["chain_depth"]
                    * controller._loaded_companion_distance()
                )
            )
            np.testing.assert_allclose(
                expected_assembly_position,
                target["assembly_position"],
            )
            expected_staging_position = (
                payload_by_chain[target["chain_index"]]["position"]
                + rear * (
                    controller.transport_staging_clearance
                    + target["chain_depth"]
                    * max(
                        controller.transport_large_fleet_staging_spacing,
                        getattr(
                            controller,
                            "transport_route_robot_clearance",
                            0.32,
                        ) + 0.04,
                    )
                )
            )
            side = np.sign(float(np.dot(
                payload_by_chain[target["chain_index"]]["position"]
                - object_pos,
                lateral,
            )))
            expected_staging_position += (
                lateral
                * side
                * max(
                    controller.transport_large_fleet_staging_lateral_offset,
                    getattr(
                        controller,
                        "transport_route_robot_clearance",
                        0.32,
                    ) + 0.02,
                )
            )
            np.testing.assert_allclose(
                expected_staging_position,
                target["staging_position"],
            )
        self.assertEqual([[1, 2, 3], [1, 2]], [
            sorted(chain_depths[0]), sorted(chain_depths[1])
        ])
        companion_names = [
            namespace for namespace, target in targets.items()
            if target["role"] == "companion_push"
        ]
        rendezvous_positions = [
            targets[namespace]["staging_position"]
            for namespace in companion_names
        ]
        assigned_cost = sum(
            float(np.dot(
                positions[namespace]
                - targets[namespace]["staging_position"],
                positions[namespace]
                - targets[namespace]["staging_position"],
            ))
            for namespace in companion_names
        )
        minimum_cost = min(
            sum(
                float(np.dot(
                    positions[namespace] - rendezvous_positions[slot_index],
                    positions[namespace] - rendezvous_positions[slot_index],
                ))
                for namespace, slot_index in zip(
                    companion_names, permutation
                )
            )
            for permutation in itertools.permutations(
                range(len(companion_names))
            )
        )
        self.assertAlmostEqual(minimum_cost, assigned_cost, places=9)

        companion_target = companion_targets[0]
        displaced = companion_target["position"] + np.array([0.2, 0.1])
        chase_velocity, allow_contact = controller._slot_velocity(
            displaced, object_pos, companion_target
        )
        chase = np.array([chase_velocity.x, chase_velocity.y])
        direction = companion_target["position"] - displaced
        self.assertGreater(float(np.dot(chase, direction)), 0.0)
        self.assertTrue(allow_contact)

        settled_positions = {
            namespace: target["position"].copy()
            for namespace, target in targets.items()
        }
        targets = controller._transport_targets(
            list(settled_positions), settled_positions, object_pos, 0.0
        )
        goal_yaw = math.atan2(goal[1], goal[0])
        for target in targets.values():
            self.assertTrue(controller._slot_contact_ready(
                target["position"], goal_yaw, object_pos, 0.0, target,
                require_inward_heading=False,
            ))
            holding_velocity, allow_contact = controller._slot_velocity(
                target["position"], object_pos, target, pushing=True
            )
            self.assertTrue(allow_contact)
            expected_pressure = (
                0.04 if target["role"] == "companion_push" else 0.03
            )
            self.assertAlmostEqual(
                expected_pressure, holding_velocity.norm()
            )
            self.assertGreater(
                holding_velocity.x * goal[0]
                + holding_velocity.y * goal[1],
                expected_pressure - 0.001,
            )

        moving_parent_target = {
            "role": "companion_push",
            "position": np.array([-0.8, -1.0]),
            "parent_velocity": goal * 0.10,
            "robot_velocity": goal * 0.04,
        }
        following_velocity = controller._goal_push_velocity(
            None, object_pos, chain_depth=2,
            position=moving_parent_target["position"],
            target=moving_parent_target,
        )
        self.assertGreater(
            following_velocity.dot(ROS["transport"].Vec2(*goal)),
            0.10,
        )

        fast_chain_target = dict(
            moving_parent_target,
            max_chain_depth=4,
            parent_velocity=goal * 0.18,
            robot_velocity=goal * 0.18,
        )
        normal_chain_target = dict(
            fast_chain_target,
            parent_velocity=goal * 0.10,
            robot_velocity=goal * 0.10,
        )
        fast_parent_command = controller._goal_push_velocity(
            None, object_pos, chain_depth=4,
            position=fast_chain_target["position"],
            target=fast_chain_target,
        )
        normal_parent_command = controller._goal_push_velocity(
            None, object_pos, chain_depth=4,
            position=normal_chain_target["position"],
            target=normal_chain_target,
        )
        self.assertGreater(
            fast_parent_command.dot(ROS["transport"].Vec2(*goal)),
            normal_parent_command.dot(ROS["transport"].Vec2(*goal)),
        )

        lateral = np.array([-goal[1], goal[0]])
        lateral_error_position = (
            moving_parent_target["position"] - lateral * 0.40
        )
        bounded_turn = controller._goal_push_velocity(
            None, object_pos, chain_depth=2,
            position=lateral_error_position,
            target=moving_parent_target,
        )
        forward_component = bounded_turn.dot(
            ROS["transport"].Vec2(*goal)
        )
        lateral_component = abs(bounded_turn.dot(
            ROS["transport"].Vec2(*lateral)
        ))
        self.assertLessEqual(
            lateral_component,
            forward_component
            * math.tan(controller.transport_push_heading_cone)
            + 1e-9,
        )

        alignment = controller._staging_alignment_command(
            np.array([0.0, 0.0]),
            math.pi / 2.0,
            {
                "role": "payload_push",
                "push_direction": np.array([1.0, 0.0]),
            },
        )
        self.assertEqual(0.0, alignment.linear.x)
        self.assertLess(alignment.angular.z, 0.0)

        moving_payload_target = dict(payload_targets[0])
        moving_payload_target["parent_velocity"] = goal * 0.08
        direct_following, _ = controller._slot_velocity(
            moving_payload_target["position"], object_pos,
            moving_payload_target, pushing=True,
        )
        self.assertGreaterEqual(
            direct_following.dot(ROS["transport"].Vec2(*goal)),
            0.10,
        )

        chain_target = {"position": np.array([-0.8, -1.0])}
        settled = controller._goal_push_velocity(
            None, object_pos, chain_depth=1,
            position=chain_target["position"], target=chain_target,
        )
        compressed = controller._goal_push_velocity(
            None, object_pos, chain_depth=1,
            position=chain_target["position"] + goal * 0.05,
            target=chain_target,
        )
        open_gap = controller._goal_push_velocity(
            None, object_pos, chain_depth=1,
            position=chain_target["position"] - goal * 0.05,
            target=chain_target,
        )
        self.assertAlmostEqual(
            compressed.dot(ROS["transport"].Vec2(*goal)),
            settled.dot(ROS["transport"].Vec2(*goal)),
        )
        self.assertLess(
            settled.dot(ROS["transport"].Vec2(*goal)),
            open_gap.dot(ROS["transport"].Vec2(*goal)),
        )

        lead_target = {
            "role": "payload_push",
            "position": chain_target["position"],
            "max_chain_depth": 0,
        }
        deep_chain_target = dict(lead_target, max_chain_depth=4)
        short_chain_speed = controller._goal_push_velocity(
            None, object_pos, position=lead_target["position"],
            target=lead_target,
        ).dot(ROS["transport"].Vec2(*goal))
        deep_chain_speed = controller._goal_push_velocity(
            None, object_pos, position=deep_chain_target["position"],
            target=deep_chain_target,
        ).dot(ROS["transport"].Vec2(*goal))
        self.assertGreater(short_chain_speed, deep_chain_speed)
        self.assertAlmostEqual(
            0.15 * controller.vmax,
            deep_chain_speed,
        )

        controller._ramped_push_speed = (
            lambda requested_speed, now=None: min(requested_speed, 0.025)
        )
        open_direct = controller._goal_push_velocity(
            None,
            object_pos,
            position=lead_target["position"] - goal * 0.05,
            target=lead_target,
        )
        open_companion = controller._goal_push_velocity(
            None,
            object_pos,
            chain_depth=4,
            position=fast_chain_target["position"] - goal * 0.05,
            target=fast_chain_target,
        )
        self.assertLessEqual(
            open_direct.dot(ROS["transport"].Vec2(*goal)), 0.025 + 1e-9
        )
        self.assertLessEqual(
            open_companion.dot(ROS["transport"].Vec2(*goal)), 0.025 + 1e-9
        )
        del controller._ramped_push_speed

        three_positions = {
            namespace: positions[namespace]
            for namespace in ("tb3_0", "tb3_1", "tb3_2")
        }
        three_targets = controller._transport_targets(
            list(three_positions), three_positions, object_pos, 0.0
        )
        self.assertEqual(
            2,
            sum(
                target["role"] == "payload_push"
                for target in three_targets.values()
            ),
        )
        self.assertEqual(
            [0, 0, 1],
            sorted(
                target["chain_depth"]
                for target in three_targets.values()
            ),
        )
        chain_neighbours, shielded_neighbours, _ = (
            controller._transport_neighbours(three_targets)
        )
        first_companion = next(
            namespace for namespace, target in three_targets.items()
            if target["role"] == "companion_push"
        )
        first_parent = three_targets[first_companion]["parent_namespace"]
        other_payload = next(
            namespace for namespace, target in three_targets.items()
            if target["role"] == "payload_push" and namespace != first_parent
        )
        self.assertEqual(
            {first_companion}, chain_neighbours[first_parent]
        )
        self.assertEqual(
            {first_parent}, chain_neighbours[first_companion]
        )
        self.assertEqual(set(), chain_neighbours[other_payload])
        self.assertTrue(all(
            not neighbours for neighbours in shielded_neighbours.values()
        ))

        captured_distance = 0.151
        loaded_distance = (
            controller.transport_companion_contact_distance
            - controller.transport_contact_slop
        )
        controller.transport_engagement_parent_distances = {
            first_companion: captured_distance,
        }
        neutral_targets = controller._transport_targets(
            list(three_positions), three_positions, object_pos, 0.0
        )
        captured_goal, _, _ = controller._transport_frame(object_pos)
        np.testing.assert_allclose(
            three_positions[first_parent] - captured_goal * (
                controller.transport_companion_contact_distance
            ),
            neutral_targets[first_companion]["position"],
        )

        controller.transport_synchronized_push_started = True
        loaded_targets = controller._transport_targets(
            list(three_positions), three_positions, object_pos, 0.0
        )
        np.testing.assert_allclose(
            three_positions[first_parent] - captured_goal * loaded_distance,
            loaded_targets[first_companion]["position"],
        )
        self.assertLess(
            float(np.linalg.norm(
                loaded_targets[first_companion]["position"]
                - three_positions[first_parent]
            )),
            controller.transport_companion_contact_distance,
        )

        controller.target_x = float(object_pos[0] + 2.0)
        controller.target_y = float(object_pos[1])
        rotated_targets = controller._transport_targets(
            list(three_positions), three_positions, object_pos, 0.0
        )
        rotated_goal, _, _ = controller._transport_frame(object_pos)
        np.testing.assert_allclose(
            three_positions[first_parent] - rotated_goal * loaded_distance,
            rotated_targets[first_companion]["position"],
        )
        self.assertAlmostEqual(
            loaded_distance,
            float(np.linalg.norm(
                rotated_targets[first_companion]["position"]
                - three_positions[first_parent]
            )),
        )

    def test_transport_accordion_preserves_ordered_gaps_for_n1_to_n12(self):
        object_position = np.zeros(2)
        progress_steps = (0.0, 0.20, 0.50, 0.80, 1.0)

        for robot_count in range(1, 13):
            with self.subTest(robot_count=robot_count):
                controller = self._transport_layout_controller()
                controller.target_x = 2.0
                controller.target_y = 0.0
                controller.transport_route_robot_clearance = 0.32
                controller.transport_parallel_row_minimum_clearance = 0.25
                namespaces = [
                    "tb3_{}".format(index)
                    for index in range(robot_count)
                ]
                positions = {
                    namespace: np.array([
                        -1.8 + 0.13 * index,
                        -0.8 + 0.31 * (index % 5),
                    ])
                    for index, namespace in enumerate(namespaces)
                }
                targets = controller._transport_targets(
                    namespaces, positions, object_position, 0.0
                )

                self.assertEqual(robot_count, len(targets))
                _, rear, _ = controller._transport_frame(object_position)
                payload_by_chain = {
                    target["chain_index"]: namespace
                    for namespace, target in targets.items()
                    if target["role"] == "payload_push"
                }
                companion_count = robot_count - len(payload_by_chain)
                configured_spacing = (
                    controller.transport_large_fleet_staging_spacing
                    if companion_count > 2
                    else controller.transport_chain_staging_spacing
                )
                rendezvous_gap = max(
                    configured_spacing,
                    controller.transport_route_robot_clearance + 0.04,
                )
                loaded_gap = controller._loaded_companion_distance()

                for progress in progress_steps:
                    points = {}
                    for namespace, target in targets.items():
                        if target["role"] == "payload_push":
                            points[namespace] = np.asarray(
                                target["staging_position"], dtype=float
                            )
                        else:
                            points[namespace] = controller._compression_target(
                                target, progress
                            )

                    if progress <= 0.5:
                        # The first half only removes the wide lateral fan.
                        # No rear robot may close its longitudinal gap yet.
                        ideal_gap = rendezvous_gap
                    else:
                        longitudinal_progress = (progress - 0.5) * 2.0
                        ideal_gap = (
                            (1.0 - longitudinal_progress) * rendezvous_gap
                            + longitudinal_progress * loaded_gap
                        )
                    for chain_index, payload_namespace in (
                        payload_by_chain.items()
                    ):
                        chain = [payload_namespace]
                        chain.extend(
                            namespace
                            for namespace, target in sorted(
                                targets.items(),
                                key=lambda item: item[1]["chain_depth"],
                            )
                            if (
                                target["role"] == "companion_push"
                                and target["chain_index"] == chain_index
                            )
                        )
                        rear_coordinates = [
                            float(np.dot(points[namespace], rear))
                            for namespace in chain
                        ]
                        self.assertEqual(
                            rear_coordinates,
                            sorted(rear_coordinates),
                        )
                        for parent, child in zip(chain, chain[1:]):
                            offset = points[child] - points[parent]
                            self.assertAlmostEqual(
                                ideal_gap,
                                float(np.dot(offset, rear)),
                                places=8,
                            )
                            if progress == 0.5:
                                self.assertGreater(
                                    rendezvous_gap, loaded_gap
                                )
                                self.assertAlmostEqual(
                                    rendezvous_gap,
                                    float(np.dot(offset, rear)),
                                    places=8,
                                )
                            self.assertGreaterEqual(
                                float(np.linalg.norm(offset)) + 1e-9,
                                loaded_gap,
                            )

                    # Opposite lanes close together as one accordion too,
                    # but never collapse below the parallel-row safety gate.
                    for depth in range(
                        0,
                        max(
                            target["chain_depth"]
                            for target in targets.values()
                        ) + 1,
                    ):
                        row = [
                            namespace
                            for namespace, target in targets.items()
                            if target["chain_depth"] == depth
                        ]
                        if len(row) == 2:
                            self.assertGreaterEqual(
                                float(np.linalg.norm(
                                    points[row[0]] - points[row[1]]
                                )) + 1e-9,
                                controller
                                .transport_parallel_row_minimum_clearance,
                            )

    def test_n10_longitudinal_compression_waits_for_every_push_heading(self):
        controller = self._transport_layout_controller()
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_route_robot_clearance = 0.32
        controller.transport_compression_speed = 0.12
        controller.transport_compression_tracking_tolerance = 0.025
        controller.transport_assembly_heading_tolerance = 0.10

        namespaces = ["tb3_{}".format(index) for index in range(10)]
        assignment_positions = {
            namespace: np.array([
                -1.8 + 0.15 * index,
                -0.9 + 0.34 * (index % 6),
            ])
            for index, namespace in enumerate(namespaces)
        }
        object_position = np.zeros(2)
        targets = controller._transport_targets(
            namespaces, assignment_positions, object_position, 0.0
        )
        controller.transport_compression_progress = 0.5
        controller.transport_compression_updated_at = 0.0
        positions = {
            namespace: controller._compression_target(target, 0.5)
            for namespace, target in targets.items()
        }
        yaws = {
            namespace: math.atan2(
                float(target["push_direction"][1]),
                float(target["push_direction"][0]),
            )
            for namespace, target in targets.items()
        }
        companions = [
            namespace for namespace, target in targets.items()
            if target["role"] == "companion_push"
        ]
        reversed_tail = max(
            companions,
            key=lambda namespace: targets[namespace]["chain_depth"],
        )
        yaws[reversed_tail] = controller._normalize_angle(
            yaws[reversed_tail] + math.pi
        )

        clock = [1.0]
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            side_effect=lambda: clock[0],
            create=True,
        ):
            blocked = controller._advance_transport_compression(
                set(targets), positions, targets, yaws
            )
            self.assertEqual(0.5, blocked)

            clock[0] = 2.0
            yaws[reversed_tail] = math.atan2(
                float(targets[reversed_tail]["push_direction"][1]),
                float(targets[reversed_tail]["push_direction"][0]),
            )
            released = controller._advance_transport_compression(
                set(targets), positions, targets, yaws
            )

        self.assertGreater(released, 0.5)

    def test_n10_accordion_converges_with_hybrid_compression_controller(self):
        controller = self._transport_layout_controller()
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_route_robot_clearance = 0.32
        controller.transport_compression_speed = 0.12
        controller.transport_compression_tracking_tolerance = 0.025
        controller.transport_chain_staging_tolerance = 0.025
        controller.transport_assembly_heading_tolerance = 0.10
        controller.transport_compression_progress = 0.0
        controller.transport_compression_updated_at = None

        namespaces = ["tb3_{}".format(index) for index in range(10)]
        assignment_positions = {
            namespace: np.array([
                -1.8 + 0.15 * index,
                -0.9 + 0.34 * (index % 6),
            ])
            for index, namespace in enumerate(namespaces)
        }
        object_position = np.zeros(2)
        targets = controller._transport_targets(
            namespaces, assignment_positions, object_position, 0.0
        )
        companions = {
            namespace
            for namespace, target in targets.items()
            if target["role"] == "companion_push"
        }
        self.assertEqual(8, len(companions))

        poses = {
            namespace: np.asarray(
                target["staging_position"], dtype=float
            ).copy()
            for namespace, target in targets.items()
        }
        payload_names = set(targets) - companions
        for index, namespace in enumerate(sorted(payload_names)):
            # The roots are part of the progression gate. Start them inside
            # the rendezvous release band but outside the tighter compression
            # tracking band, so the accordion must wait for their recovery.
            direction = targets[namespace]["push_direction"]
            poses[namespace] = (
                poses[namespace]
                + direction * (0.035 if index % 2 == 0 else -0.035)
            )
        yaws = {
            namespace: (0.35 if index % 2 == 0 else -0.35)
            for index, namespace in enumerate(namespaces)
        }
        clock = [0.0]
        time_step = 0.05
        completed_step = None
        progress_started_step = None
        handoff_step = None
        longitudinal_started_step = None
        reversed_tail = max(
            companions,
            key=lambda namespace: targets[namespace]["chain_depth"],
        )
        forced_reversed_handoff = False

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            side_effect=lambda: clock[0],
            create=True,
        ):
            for step in range(2400):
                progress = controller._advance_transport_compression(
                    set(targets), poses, targets, yaws
                )
                if progress > 0.0 and progress_started_step is None:
                    progress_started_step = step
                    self.assertLessEqual(
                        max(
                            float(np.linalg.norm(
                                poses[namespace]
                                - targets[namespace]["staging_position"]
                            ))
                            for namespace in payload_names
                        ),
                        (
                            controller
                            .transport_compression_tracking_tolerance
                            + 1e-9
                        ),
                    )
                if progress == 0.5 and handoff_step is None:
                    handoff_step = step
                    # Reproduce the visible N=10 failure: the deepest rear
                    # robot finishes its fan move facing away from the common
                    # push direction.  Longitudinal closing must wait while
                    # the hybrid controller turns this Burger in place.
                    desired_yaw = math.atan2(
                        float(targets[reversed_tail]["push_direction"][1]),
                        float(targets[reversed_tail]["push_direction"][0]),
                    )
                    yaws[reversed_tail] = controller._normalize_angle(
                        desired_yaw + math.pi
                    )
                    forced_reversed_handoff = True
                if progress > 0.5 and longitudinal_started_step is None:
                    longitudinal_started_step = step
                destinations = {}
                for namespace, target in targets.items():
                    if namespace in companions:
                        destination = controller._compression_target(
                            target, progress
                        )
                    else:
                        destination = np.asarray(
                            target["staging_position"], dtype=float
                        )
                    destinations[namespace] = destination
                    if namespace in companions and progress > 0.5:
                        command = controller._chain_pose_hold_command(
                            poses[namespace],
                            yaws[namespace],
                            destination,
                            target["push_direction"],
                            forward_limit=(
                                controller.transport_compression_speed
                            ),
                            reverse_limit=0.06,
                        )
                    else:
                        command = controller._payload_staging_command(
                            poses[namespace],
                            yaws[namespace],
                            destination,
                            target["push_direction"],
                            forward_limit=(
                                controller.transport_compression_speed
                            ),
                            reverse_limit=0.06,
                        )

                    yaw = yaws[namespace]
                    poses[namespace] = poses[namespace] + time_step * (
                        command.linear.x
                        * np.array([math.cos(yaw), math.sin(yaw)])
                    )
                    yaws[namespace] = controller._normalize_angle(
                        yaw + time_step * command.angular.z
                    )
                clock[0] += time_step

                final_errors = [
                    float(np.linalg.norm(
                        poses[namespace] - (
                            np.asarray(
                                targets[namespace]["assembly_position"],
                                dtype=float,
                            )
                            if namespace in companions
                            else np.asarray(
                                targets[namespace]["staging_position"],
                                dtype=float,
                            )
                        )
                    ))
                    for namespace in namespaces
                ]
                heading_errors = [
                    abs(controller._normalize_angle(
                        math.atan2(
                            float(targets[namespace]["push_direction"][1]),
                            float(targets[namespace]["push_direction"][0]),
                        ) - yaws[namespace]
                    ))
                    for namespace in namespaces
                ]
                if (
                    progress >= 1.0
                    and max(final_errors) <= 0.013
                    and max(heading_errors) <= 0.03
                ):
                    completed_step = step
                    break

        self.assertIsNotNone(
            completed_step,
            msg=(
                "progress={!r}, max_position_error={!r}, "
                "max_heading_error={!r}, tracking_errors={!r}".format(
                    progress,
                    max(final_errors),
                    max(heading_errors),
                    {
                        namespace: float(np.linalg.norm(
                            controller._compression_target(
                                targets[namespace], progress
                            ) - poses[namespace]
                        ))
                        for namespace in namespaces
                    },
                )
            ),
        )
        self.assertIsNotNone(progress_started_step)
        self.assertTrue(forced_reversed_handoff)
        self.assertIsNotNone(handoff_step)
        self.assertIsNotNone(longitudinal_started_step)
        self.assertGreater(longitudinal_started_step, handoff_step)
        self.assertGreater(progress_started_step, 0)
        self.assertEqual(1.0, controller.transport_compression_progress)
        self.assertLessEqual(max(final_errors), 0.013)
        self.assertLessEqual(max(heading_errors), 0.03)

    def test_transport_companion_spacing_eases_and_recovers_smoothly(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_companion_contact_distance = 0.145
        controller.transport_companion_preload = 0.025
        controller.transport_push_recovery_closing_speed = 0.012
        controller.transport_push_heading_cone = 0.18
        controller.transport_push_ramp_started_at = None
        controller.transport_push_ramp_rate = 0.0
        controller.vmax = 0.16

        object_position = np.array([0.0, 0.0])
        push_direction = np.array([1.0, 0.0])
        parent_position = np.array([0.50, 0.0])
        nominal_position = (
            parent_position
            - push_direction * controller.transport_companion_contact_distance
        )
        parent_velocity = push_direction * 0.10
        target = {
            "role": "companion_push",
            "position": nominal_position,
            "push_direction": push_direction,
            "parent_position": parent_position,
            "parent_velocity": parent_velocity,
            "robot_velocity": parent_velocity,
            "max_chain_depth": 4,
            "companion_count": 2,
        }

        loaded = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=nominal_position, target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        compressed = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=nominal_position + push_direction * 0.02,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        open_link = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=nominal_position - push_direction * 0.04,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))

        self.assertGreaterEqual(compressed, 0.0)
        self.assertLess(compressed, loaded)
        recovery_limit = 0.10 + 0.012
        self.assertGreater(loaded, recovery_limit)
        self.assertLess(open_link, loaded)
        self.assertLessEqual(open_link, recovery_limit + 1e-9)

    def test_transport_companion_follows_its_parent_without_a_speed_floor(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_companion_contact_distance = 0.145
        controller.transport_companion_preload = 0.025
        controller.transport_push_recovery_closing_speed = 0.012
        controller.transport_push_heading_cone = 0.18
        controller.transport_push_ramp_started_at = None
        controller.transport_push_ramp_rate = 0.0
        controller.vmax = 0.16

        object_position = np.zeros(2)
        push_direction = np.array([1.0, 0.0])
        parent_position = np.array([0.50, 0.0])
        nominal_position = (
            parent_position
            - push_direction * controller.transport_companion_contact_distance
        )
        target = {
            "role": "companion_push",
            "position": nominal_position,
            "push_direction": push_direction,
            "parent_position": parent_position,
            "parent_velocity": np.zeros(2),
            "robot_velocity": np.zeros(2),
            "max_chain_depth": 4,
            "companion_count": 2,
        }

        compressed_position = nominal_position + push_direction * 0.02
        decompression_request = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=compressed_position, target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertLessEqual(decompression_request, 0.005)

        tracked_speeds = []
        for parent_speed in (0.04, 0.10):
            target["parent_velocity"] = push_direction * parent_speed
            target["robot_velocity"] = push_direction * parent_speed
            requested = controller._goal_push_velocity(
                None, object_position, chain_depth=2,
                position=nominal_position, target=target,
            ).dot(ROS["transport"].Vec2(*push_direction))
            tracked_speeds.append(requested)
            self.assertAlmostEqual(
                parent_speed + controller.transport_companion_preload,
                requested,
            )

        self.assertGreater(tracked_speeds[1], tracked_speeds[0])

    def test_transport_companion_relative_speed_has_one_combined_bound(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_companion_contact_distance = 0.145
        controller.transport_companion_preload = 0.018
        controller.transport_push_recovery_closing_speed = 0.012
        controller.transport_push_heading_cone = 0.18
        controller.transport_push_ramp_started_at = None
        controller.transport_push_ramp_rate = 0.0
        controller.vmax = 0.16

        object_position = np.zeros(2)
        push_direction = np.array([1.0, 0.0])
        parent_position = np.array([0.50, 0.0])
        nominal_position = (
            parent_position
            - push_direction * controller.transport_companion_contact_distance
        )
        parent_speed = 0.08
        target = {
            "role": "companion_push",
            # Stress the combined limit with a target ahead of the measured
            # contact position and a child moving slower than its parent.
            "position": nominal_position + push_direction * 0.04,
            "push_direction": push_direction,
            "parent_position": parent_position,
            "parent_velocity": push_direction * parent_speed,
            "robot_velocity": np.zeros(2),
            "max_chain_depth": 4,
            "companion_count": 2,
        }

        catching_up = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=nominal_position, target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertLessEqual(catching_up, parent_speed + 0.025 + 1e-9)

        target["position"] = nominal_position
        target["parent_velocity"] = push_direction * 0.10
        target["robot_velocity"] = push_direction * 0.16
        compressed_position = nominal_position + push_direction * 0.04
        easing_off = controller._goal_push_velocity(
            None, object_position, chain_depth=2,
            position=compressed_position, target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertGreaterEqual(easing_off, 0.10 - 0.040 - 1e-9)

    def test_transport_synchronized_companions_share_one_batch_speed(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_recovery_closing_speed = 0.025
        controller.transport_push_reference_speed = 0.056
        controller.transport_chain_heading_cone = 0.06
        controller.vmax = 0.16
        controller._ramped_push_speed = mock.Mock(return_value=0.004)

        object_position = np.zeros(2)
        push_direction = np.array([1.0, 0.0])
        parent_position = np.array([0.50, 0.0])
        loaded_position = parent_position - push_direction * 0.140
        target = {
            "role": "companion_push",
            "position": loaded_position.copy(),
            "push_direction": push_direction,
            "parent_position": parent_position,
            "max_chain_depth": 4,
            "companion_count": 2,
        }

        # Each row receives the same batch reference even though the measured
        # predecessor speed gets progressively slower toward the rear.
        baseline_speeds = []
        for depth, parent_speed in enumerate(
            (0.050, 0.035, 0.020, 0.005), start=1
        ):
            target["parent_velocity"] = push_direction * parent_speed
            target["robot_velocity"] = push_direction * parent_speed
            velocity = controller._goal_push_velocity(
                None,
                object_position,
                chain_depth=depth,
                position=loaded_position,
                target=target,
            )
            baseline_speeds.append(
                velocity.dot(ROS["transport"].Vec2(*push_direction))
            )

        for speed in baseline_speeds:
            self.assertAlmostEqual(
                controller.transport_push_reference_speed, speed
            )

        target["parent_velocity"] = push_direction * 0.056
        target["robot_velocity"] = push_direction * 0.056
        open_position = loaded_position - push_direction * 0.04
        open_speed = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=4,
            position=open_position,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertAlmostEqual(0.056 + 0.008, open_speed)

        compressed_position = loaded_position + push_direction * 0.01
        compressed_speed = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=4,
            position=compressed_position,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertAlmostEqual(
            0.056 - 1.5 * (0.010 - 0.003), compressed_speed
        )
        self.assertLess(
            compressed_speed, controller.transport_push_reference_speed
        )

        # A faster child does not get a negative correction. A slower child
        # receives no more than the small per-link catch-up allowance.
        target["parent_velocity"] = push_direction * 0.056
        target["robot_velocity"] = push_direction * 0.10
        faster_child_speed = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=4,
            position=loaded_position,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertAlmostEqual(
            controller.transport_push_reference_speed,
            faster_child_speed,
        )

        target["robot_velocity"] = np.zeros(2)
        slower_child_speed = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=4,
            position=loaded_position,
            target=target,
        ).dot(ROS["transport"].Vec2(*push_direction))
        self.assertGreater(slower_child_speed, baseline_speeds[-1])
        self.assertLessEqual(
            slower_child_speed,
            controller.transport_push_reference_speed + 0.008 + 1e-9,
        )

        # The shared reference is already the fleet ramp. Applying the older
        # per-robot ramp here would recreate the depth-dependent speed cascade.
        controller._ramped_push_speed.assert_not_called()

    def test_transport_synchronized_gap_feedback_is_signed(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.056
        controller.transport_push_recovery_closing_speed = 0.025
        controller.transport_chain_heading_cone = 0.06
        controller.vmax = 0.16

        push_direction = np.array([1.0, 0.0])
        parent_position = np.array([0.50, 0.0])
        target = {
            "role": "companion_push",
            "position": parent_position - push_direction * 0.140,
            "push_direction": push_direction,
            "parent_position": parent_position,
            "parent_velocity": push_direction * 0.056,
            "robot_velocity": push_direction * 0.056,
            "max_chain_depth": 4,
            "companion_count": 2,
        }

        cases = (
            ("open", 0.150, 0.0640),
            ("nominal", 0.140, 0.0560),
            ("compressed", 0.130, 0.0455),
        )
        for label, gap, expected_speed in cases:
            with self.subTest(link_state=label):
                position = parent_position - push_direction * gap
                velocity = controller._goal_push_velocity(
                    None,
                    np.zeros(2),
                    chain_depth=2,
                    position=position,
                    target=target,
                )
                self.assertAlmostEqual(
                    expected_speed,
                    velocity.dot(
                        ROS["transport"].Vec2(*push_direction)
                    ),
                )

    def test_transport_loaded_arbitration_is_local_to_compressed_subtree(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.018

        # Ten robots form two depth-four lanes. Only the second link in the
        # first lane is compressed and slowed by post-planner avoidance.
        positions, targets = self._loaded_transport_topology(
            10, {"tb3_2": 0.13034}
        )
        raw_speeds = {
            namespace: (0.006 if namespace == "tb3_2" else 0.018)
            for namespace in targets
        }
        commands = {}
        for namespace, speed in raw_speeds.items():
            commands[namespace] = Twist()
            commands[namespace].linear.x = speed

        controller._coordinate_loaded_chain_commands(
            commands, targets, positions
        )
        speeds = {
            namespace: command.linear.x
            for namespace, command in commands.items()
        }

        self.assertAlmostEqual(0.018, speeds["tb3_0"])
        self.assertAlmostEqual(0.018, speeds["tb3_5"])
        for namespace in ("tb3_5", "tb3_6", "tb3_7", "tb3_8", "tb3_9"):
            self.assertAlmostEqual(0.018, speeds[namespace])
        self.assertAlmostEqual(0.018, speeds["tb3_1"])
        self.assertTrue(all(
            speeds[namespace] > 0.0
            for namespace in ("tb3_2", "tb3_3", "tb3_4")
        ))
        self.assertTrue(all(
            speeds[namespace] <= raw_speeds[namespace] + 1e-12
            for namespace in targets
        ))
        self.assertEqual(
            "compressed",
            controller.transport_push_link_states["tb3_2"][0],
        )
        self.assertEqual((), controller.transport_push_hard_stop_sources)

    def test_transport_loaded_arbitration_distinguishes_yield_from_stop(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.018

        def coordinate(gap, root_speed, child_speed):
            positions, targets = self._loaded_transport_topology(
                2, {"tb3_1": gap}, lane_count=1
            )
            commands = {"tb3_0": Twist(), "tb3_1": Twist()}
            commands["tb3_0"].linear.x = root_speed
            commands["tb3_1"].linear.x = child_speed
            controller._coordinate_loaded_chain_commands(
                commands, targets, positions
            )
            return {
                namespace: command.linear.x
                for namespace, command in commands.items()
            }

        # A stopped compressed child is already yielding; its parent must be
        # allowed to pull the bumper gap back toward the nominal distance.
        compressed = coordinate(0.130, 0.018, 0.0)
        self.assertAlmostEqual(0.018, compressed["tb3_0"])
        self.assertEqual(0.0, compressed["tb3_1"])
        self.assertEqual((), controller.transport_push_hard_stop_sources)

        cases = (
            ("nominal companion", 0.140, 0.018, 0.0, "tb3_1"),
            ("payload root", 0.130, 0.0, 0.018, "tb3_0"),
        )
        for label, gap, root_speed, child_speed, source in cases:
            with self.subTest(stop_source=label):
                stopped = coordinate(gap, root_speed, child_speed)
                self.assertEqual(
                    {"tb3_0": 0.0, "tb3_1": 0.0}, stopped
                )
                self.assertEqual(
                    (source,), controller.transport_push_hard_stop_sources
                )

    def test_transport_adjacent_compressed_links_never_close_further(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.018
        positions, targets = self._loaded_transport_topology(
            3,
            {"tb3_1": 0.130, "tb3_2": 0.130},
            lane_count=1,
        )
        commands = {
            "tb3_0": Twist(),
            "tb3_1": Twist(),
            "tb3_2": Twist(),
        }
        commands["tb3_0"].linear.x = 0.018
        commands["tb3_1"].linear.x = 0.0
        commands["tb3_2"].linear.x = 0.018

        controller._coordinate_loaded_chain_commands(
            commands, targets, positions
        )

        self.assertAlmostEqual(0.018, commands["tb3_0"].linear.x)
        self.assertEqual(0.0, commands["tb3_1"].linear.x)
        self.assertEqual(0.0, commands["tb3_2"].linear.x)
        self.assertEqual((), controller.transport_push_hard_stop_sources)

        # A very slow middle robot must still let the rear gap reopen. The
        # nominal yield floor cannot make a compressed child match its parent.
        commands["tb3_0"].linear.x = 0.018
        commands["tb3_1"].linear.x = 0.002
        commands["tb3_2"].linear.x = 0.018
        controller._coordinate_loaded_chain_commands(
            commands, targets, positions
        )
        self.assertAlmostEqual(0.002, commands["tb3_1"].linear.x)
        self.assertGreaterEqual(commands["tb3_2"].linear.x, 0.0)
        self.assertLess(
            commands["tb3_2"].linear.x,
            commands["tb3_1"].linear.x,
        )

    def test_transport_compressed_link_reopens_monotonically_within_one_second(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.018

        positions, targets = self._loaded_transport_topology(
            2, {"tb3_1": 0.13034}, lane_count=1
        )
        time_step = 0.05
        elapsed = 0.0
        gaps = [positions["tb3_0"][0] - positions["tb3_1"][0]]
        recovered_at = None
        for _ in range(int(1.0 / time_step)):
            commands = {"tb3_0": Twist(), "tb3_1": Twist()}
            commands["tb3_0"].linear.x = 0.018
            commands["tb3_1"].linear.x = 0.006
            controller._coordinate_loaded_chain_commands(
                commands, targets, positions
            )
            for namespace, command in commands.items():
                positions[namespace][0] += command.linear.x * time_step
            elapsed += time_step
            gap = positions["tb3_0"][0] - positions["tb3_1"][0]
            gaps.append(gap)
            if recovered_at is None and gap >= 0.137:
                recovered_at = elapsed

        self.assertTrue(all(
            after + 1e-12 >= before
            for before, after in zip(gaps, gaps[1:])
        ))
        self.assertGreater(gaps[-1], gaps[0])
        self.assertGreaterEqual(gaps[-1], 0.137)
        self.assertIsNotNone(recovered_at)
        self.assertLessEqual(recovered_at, 1.0)

    def test_transport_loaded_arbitration_preserves_randomized_topologies(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_push_reference_speed = 0.018
        generator = random.Random(7319)

        for robot_count in range(1, 13):
            for sample in range(8):
                gaps = {}
                for robot_index in range(2, robot_count):
                    gaps["tb3_{}".format(robot_index)] = generator.choice(
                        (0.130, 0.140, 0.150)
                    )
                positions, targets = self._loaded_transport_topology(
                    robot_count, gaps
                )
                raw_speeds = {
                    namespace: generator.uniform(0.004, 0.040)
                    for namespace in targets
                }
                commands = {}
                for namespace, speed in raw_speeds.items():
                    commands[namespace] = Twist()
                    commands[namespace].linear.x = speed

                controller._coordinate_loaded_chain_commands(
                    commands, targets, positions
                )
                speeds = {
                    namespace: command.linear.x
                    for namespace, command in commands.items()
                }

                with self.subTest(robots=robot_count, sample=sample):
                    for namespace, target in targets.items():
                        self.assertGreaterEqual(speeds[namespace], 0.0)
                        self.assertLessEqual(
                            speeds[namespace],
                            raw_speeds[namespace] + 1e-12,
                        )
                        parent = target["parent_namespace"]
                        if parent is None:
                            continue
                        self.assertEqual(
                            target["chain_index"],
                            targets[parent]["chain_index"],
                        )
                        self.assertEqual(
                            target["chain_depth"] - 1,
                            targets[parent]["chain_depth"],
                        )
                        state = controller.transport_push_link_states[
                            namespace
                        ][0]
                        allowance = 0.008 if state == "open" else 0.0
                        self.assertLessEqual(
                            speeds[namespace],
                            speeds[parent] + allowance + 1e-12,
                        )

                    root_speeds = [
                        speeds[namespace]
                        for namespace, target in targets.items()
                        if target["role"] == "payload_push"
                    ]
                    self.assertLessEqual(
                        max(root_speeds) - min(root_speeds), 1e-12
                    )
                    self.assertEqual(
                        (), controller.transport_push_hard_stop_sources
                    )

    def test_transport_open_links_recover_without_striking_the_parent(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 0.0
        controller.target_y = -2.0
        controller.object_half_width = 0.20
        controller.object_half_height = 0.20
        controller.object_yaw = 0.0
        controller.robot_forward_contact_extent = 0.038
        controller.transport_contact_slop = 0.005
        controller.transport_companion_contact_distance = 0.145
        controller.transport_companion_preload = 0.018
        controller.transport_contact_closing_speed = 0.018
        controller.transport_push_recovery_closing_speed = 0.025
        controller.transport_single_companion_preload = 0.025
        controller.transport_single_companion_recovery_speed = 0.025
        controller.transport_push_heading_cone = 0.18
        controller.transport_push_ramp_started_at = None
        controller.transport_push_ramp_rate = 0.0
        controller.vmax = 0.16

        object_position = np.array([0.0, 0.0])
        goal = np.array([0.0, -1.0])
        parent_position = np.array([0.0, -0.50])
        contact_position = parent_position - goal * 0.14
        parent_velocity = goal * 0.06
        companion_target = {
            "role": "companion_push",
            "position": contact_position.copy(),
            "push_direction": goal.copy(),
            "parent_position": parent_position.copy(),
            "parent_velocity": parent_velocity.copy(),
            "robot_velocity": parent_velocity.copy(),
            "max_chain_depth": 1,
            "companion_count": 2,
        }

        loaded = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=1,
            position=contact_position,
            target=companion_target,
        )
        self.assertGreater(loaded.dot(ROS["transport"].Vec2(*goal)), 0.075)

        open_position = contact_position - goal * 0.04
        recovering = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=1,
            position=open_position,
            target=companion_target,
        )
        recovery_limit = 0.06 + 0.025
        recovering_speed = recovering.dot(
            ROS["transport"].Vec2(*goal)
        )
        self.assertAlmostEqual(recovery_limit, recovering_speed)
        self.assertGreater(
            recovering_speed,
            0.06 + controller.transport_contact_closing_speed,
        )

        chasing, _ = controller._slot_velocity(
            open_position,
            object_position,
            companion_target,
            pushing=True,
        )
        self.assertAlmostEqual(
            recovery_limit,
            chasing.dot(ROS["transport"].Vec2(*goal)),
        )

        companion_target["companion_count"] = 1
        single_recovery = controller._goal_push_velocity(
            None,
            object_position,
            chain_depth=1,
            position=open_position,
            target=companion_target,
        )
        self.assertAlmostEqual(
            recovery_limit,
            single_recovery.dot(ROS["transport"].Vec2(*goal)),
        )
        companion_target["companion_count"] = 2

        payload_contact = np.array([0.0, 0.243])
        payload_target = {
            "role": "payload_push",
            "position": payload_contact,
            "push_direction": goal.copy(),
            "parent_position": object_position.copy(),
            "parent_velocity": parent_velocity.copy(),
        }
        payload_recovery, _ = controller._slot_velocity(
            payload_contact - goal * 0.10,
            object_position,
            payload_target,
            pushing=True,
        )
        self.assertAlmostEqual(
            recovery_limit,
            payload_recovery.dot(ROS["transport"].Vec2(*goal)),
        )

    def test_transport_push_steering_only_caps_a_useful_aligned_push(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_push_angular_limit = 0.25
        controller.transport_min_useful_push_speed = 0.035

        useful_push = Twist()
        useful_push.linear.x = 0.06
        useful_push.angular.z = 0.70
        controller._stabilize_push_steering(useful_push)
        self.assertAlmostEqual(0.25, useful_push.angular.z)

        recovery_turn = Twist()
        recovery_turn.linear.x = 0.018
        recovery_turn.angular.z = -0.70
        controller._stabilize_push_steering(recovery_turn)
        self.assertAlmostEqual(-0.70, recovery_turn.angular.z)

    def test_loaded_chain_straightens_before_pushing_off_axis(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.16
        controller.transport_launch_heading_tolerance = 0.08
        velocity = ROS["transport"].Vec2(0.08, 0.0)
        target = {
            "max_chain_depth": 4,
            "push_direction": np.array([1.0, 0.0]),
        }

        aligned = controller._loaded_chain_command(
            velocity, 0.0, target
        )
        self.assertAlmostEqual(0.08, aligned.linear.x)
        self.assertAlmostEqual(0.0, aligned.angular.z)

        off_axis = controller._loaded_chain_command(
            velocity, 0.24, target
        )
        self.assertLessEqual(off_axis.linear.x, aligned.linear.x * 0.05)
        self.assertAlmostEqual(-0.48, off_axis.angular.z)

    def test_transport_pressure_floor_is_reserved_for_payload_leads(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_push_heading_cone = 0.18
        controller.transport_min_useful_push_speed = 0.035
        controller.transport_push_ramp_started_at = None
        controller.transport_push_ramp_rate = 0.0
        controller.vmax = 0.16

        companion_target = {"role": "companion_push"}
        for requested_speed in (0.0, 0.012):
            companion_command = Twist()
            companion_command.linear.x = requested_speed
            held = controller._keep_chain_push_pressure(
                "tb3_1", companion_command, np.zeros(2), 0.0,
                companion_target, contact_ready=True, contact_allowed=True,
            )
            self.assertIs(companion_command, held)
            self.assertEqual(requested_speed, held.linear.x)

        payload_target = {"role": "payload_push"}

        misaligned = Twist()
        held = controller._keep_chain_push_pressure(
            "tb3_0", misaligned, np.zeros(2), math.pi / 2.0,
            payload_target, contact_ready=True, contact_allowed=True,
        )
        self.assertEqual(0.0, held.linear.x)

        aligned = Twist()
        held = controller._keep_chain_push_pressure(
            "tb3_0", aligned, np.zeros(2), 0.0,
            payload_target, contact_ready=True, contact_allowed=True,
        )
        self.assertGreaterEqual(held.linear.x, 0.045)

    def test_transport_single_pusher_stays_centered_with_inset_lanes(self):
        controller = self._transport_layout_controller()
        object_position = np.array([-0.8, -1.6])
        positions = {"tb3_0": np.array([-0.8, -0.8])}

        target = controller._transport_targets(
            list(positions), positions, object_position, 0.0
        )["tb3_0"]
        _, rear, lateral = controller._transport_frame(object_position)
        offset = target["position"] - object_position

        self.assertAlmostEqual(0.0, float(np.dot(offset, lateral)), places=9)
        self.assertGreater(float(np.dot(offset, rear)), 0.0)

    def test_transport_chain_contacts_require_the_chain_corridor(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_orbit_radius = 0.60
        controller.transport_companion_contact_distance = 0.145
        controller.transport_chain_corridor_width = 0.12
        controller.transport_chain_alignment_tolerance = 0.55
        targets = {
            "tb3_0": {
                "role": "payload_push", "chain_index": 0,
                "chain_depth": 0, "position": np.array([0.0, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
                "parent_namespace": None,
            },
            "tb3_1": {
                "role": "companion_push", "chain_index": 0,
                "chain_depth": 1, "position": np.array([-0.145, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
                "parent_namespace": "tb3_0",
            },
            "tb3_2": {
                "role": "companion_push", "chain_index": 0,
                "chain_depth": 2, "position": np.array([-0.29, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
                "parent_namespace": "tb3_1",
            },
        }
        contacts, shielded, _ = controller._transport_neighbours(targets)
        positions = {
            namespace: target["position"].copy()
            for namespace, target in targets.items()
        }
        nearby = controller._nearby_chain_contacts

        self.assertEqual({"tb3_1"}, set(nearby(
            "tb3_0", contacts, positions, targets
        )))
        self.assertEqual({"tb3_0", "tb3_2"}, set(nearby(
            "tb3_1", contacts, positions, targets
        )))
        self.assertEqual({"tb3_2"}, set(nearby(
            "tb3_0", shielded, positions, targets
        )))

        positions["tb3_1"] = np.array([0.145, 0.0])
        self.assertEqual(set(), set(nearby(
            "tb3_0", contacts, positions, targets
        )))
        self.assertEqual({"tb3_2"}, set(nearby(
            "tb3_1", contacts, positions, targets
        )))
        self.assertEqual(set(), set(nearby(
            "tb3_0", shielded, positions, targets
        )))

        positions["tb3_1"] = np.array([-0.145, 0.13])
        self.assertEqual(set(), set(nearby(
            "tb3_0", contacts, positions, targets
        )))
        self.assertEqual(set(), set(nearby(
            "tb3_1", contacts, positions, targets
        )))

    def test_transport_rendezvous_and_lane_compression_are_concurrent(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.object_position = np.zeros(2)
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.robot_namespaces = [
            "tb3_0", "tb3_1", "tb3_2", "tb3_3", "tb3_4"
        ]
        controller.robot_positions = {
            namespace: np.array([-2.0 - index * 0.2, 0.4 * (index % 2)])
            for index, namespace in enumerate(controller.robot_namespaces)
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.robot_velocities = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_odom_received_at = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.transport_pre_staged = set()
        controller.transport_staged = set()
        controller.transport_chain_released = set()
        controller.transport_compression_progress = 0.0
        controller.transport_compression_updated_at = None
        controller.transport_assembly_route_states = {}
        controller.transport_last_rendezvous_log_count = None
        controller.transport_ready_hold_time = 0.0
        controller.transport_all_ready_since = None

        goal = np.array([1.0, 0.0])
        targets = {}
        for index, namespace in enumerate(controller.robot_namespaces):
            if index < 2:
                lane = index
                staging = np.array([-0.50, -0.18 + 0.36 * lane])
                targets[namespace] = {
                    "role": "payload_push",
                    "chain_index": lane,
                    "chain_depth": 0,
                    "position": np.array([-0.24, -0.18 + 0.36 * lane]),
                    "staging_position": staging,
                    "push_direction": goal.copy(),
                    "parent_namespace": None,
                    "parent_position": np.zeros(2),
                }
                continue
            lane = (index - 2) % 2
            depth = (index - 2) // 2 + 1
            parent = "tb3_{}".format(lane if depth == 1 else index - 2)
            targets[namespace] = {
                "role": "companion_push",
                "chain_index": lane,
                "chain_depth": depth,
                "position": np.array([-0.50 - 0.14 * depth, -0.18 + 0.36 * lane]),
                "assembly_position": np.array([
                    -0.50 - 0.14 * depth, -0.18 + 0.36 * lane
                ]),
                "staging_position": np.array([
                    -0.50 - 0.38 * depth, -0.52 + 1.04 * lane
                ]),
                "push_direction": goal.copy(),
                "parent_namespace": parent,
                "parent_position": controller.robot_positions[parent],
            }

        controller._transport_odometry_error = lambda *_args: None
        controller._transport_target_error = lambda: None
        controller._transport_layout_error = lambda *_args, **_kwargs: None
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = lambda *_args, **_kwargs: False
        controller._rendezvous_pose_ready = lambda *_args: False
        controller._payload_staging_route_destination = (
            lambda target: np.asarray(target["staging_position"])
        )
        route_calls = []

        def route_command(namespace, *_args, **_kwargs):
            route_calls.append(namespace)
            command = Twist()
            command.linear.x = 0.08
            return command

        controller._transport_route_command = route_command
        published = {}
        controller._publish_concurrent_approach_command = (
            lambda namespace, command, *_args, **kwargs:
            published.__setitem__(namespace, (command, kwargs))
        )

        controller._approach_phase(4)

        self.assertEqual(set(controller.robot_namespaces), set(route_calls))
        self.assertEqual(set(controller.robot_namespaces), set(published))
        self.assertTrue(all(
            command.linear.x > 0.0 for command, _kwargs in published.values()
        ))
        self.assertEqual(set(), controller.transport_chain_released)

        record_publish = controller._publish_concurrent_approach_command
        controller._publish_concurrent_approach_command = mock.Mock(
            side_effect=RuntimeError("avoidance failed")
        )
        with self.assertRaises(RuntimeError):
            controller._approach_phase(4)
        self.assertIsNone(controller._pending_approach_commands)
        controller._publish_concurrent_approach_command = record_publish

        controller.transport_pre_staged = set(controller.robot_namespaces)
        compression_calls = []

        def advance_compression(namespaces, *_args):
            compression_calls.append(set(namespaces))
            return 0.5

        controller._advance_transport_compression = advance_compression
        chain_hold_calls = []
        point_staging_calls = []

        def chain_pose_hold(*args, **kwargs):
            chain_hold_calls.append((args, kwargs))
            return Message(
                linear=Message(x=0.06), angular=Message(z=0.0)
            )

        def point_staging(*args, **kwargs):
            point_staging_calls.append((args, kwargs))
            return Message(
                linear=Message(x=0.04), angular=Message(z=0.0)
            )

        controller._chain_pose_hold_command = chain_pose_hold
        controller._payload_staging_command = point_staging
        published.clear()
        controller._approach_phase(4)

        companions = {
            namespace for namespace, target in targets.items()
            if target["role"] == "companion_push"
        }
        self.assertEqual([set(targets)], compression_calls)
        self.assertEqual(companions, controller.transport_chain_released)
        self.assertEqual(set(controller.robot_namespaces), set(published))
        self.assertEqual(0, len(chain_hold_calls))
        self.assertEqual(len(targets), len(point_staging_calls))

        # The exact midpoint is a settling gate.  Once it releases, every
        # companion must keep the common push heading while rows shorten.
        chain_hold_calls.clear()
        point_staging_calls.clear()
        published.clear()

        def advance_longitudinal(namespaces, *_args):
            compression_calls.append(set(namespaces))
            return 0.75

        controller._advance_transport_compression = advance_longitudinal
        controller._approach_phase(4)

        self.assertEqual([set(targets), set(targets)], compression_calls)
        self.assertEqual(set(controller.robot_namespaces), set(published))
        self.assertEqual(len(companions), len(chain_hold_calls))
        self.assertEqual(
            len(targets) - len(companions), len(point_staging_calls)
        )
        for namespace in companions:
            command, kwargs = published[namespace]
            self.assertGreater(command.linear.x, 0.0)
            self.assertTrue(kwargs["chain_motion"])

        controller._slot_contact_ready = lambda *_args, **_kwargs: True
        controller._advance_transport_compression = lambda *_args: 1.0
        controller._parallel_push_rows_ready = lambda *_args: True
        phases = []
        controller._set_phase = (
            lambda phase, _epoch: phases.append(phase) or True
        )
        controller._approach_phase(4)
        self.assertEqual(
            [ROS["transport"].TransportPhase.PUSH], phases
        )

    def test_midpoint_heading_turn_bypasses_avoidance_after_position_settles(
        self,
    ):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.object_position = np.zeros(2)
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.robot_namespaces = ["tb3_0", "tb3_1", "tb3_2"]
        controller.robot_velocities = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_odom_received_at = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }

        push_direction = np.array([1.0, 0.0])
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_index": 0,
                "chain_depth": 0,
                "position": np.array([-0.24, 0.0]),
                "staging_position": np.array([-0.50, 0.0]),
                "push_direction": push_direction.copy(),
                "parent_namespace": None,
                "parent_position": np.zeros(2),
            },
            "tb3_1": {
                "role": "companion_push",
                "chain_index": 0,
                "chain_depth": 1,
                "position": np.array([-0.64, 0.0]),
                "assembly_position": np.array([-0.64, 0.0]),
                "staging_position": np.array([-1.00, 0.36]),
                "push_direction": push_direction.copy(),
                "parent_namespace": "tb3_0",
                "parent_position": np.array([-0.50, 0.0]),
            },
            "tb3_2": {
                "role": "companion_push",
                "chain_index": 0,
                "chain_depth": 2,
                "position": np.array([-0.78, 0.0]),
                "assembly_position": np.array([-0.78, 0.0]),
                "staging_position": np.array([-1.40, -0.36]),
                "push_direction": push_direction.copy(),
                "parent_namespace": "tb3_1",
                "parent_position": np.array([-1.00, 0.0]),
            },
        }
        midpoint_positions = {
            namespace: controller._compression_target(target, 0.5)
            for namespace, target in targets.items()
        }
        controller.robot_positions = {
            namespace: position.copy()
            for namespace, position in midpoint_positions.items()
        }
        # tb3_1 is exactly at its midpoint and only needs to turn. tb3_2 is
        # still three centimetres off target and must retain normal guards.
        controller.robot_positions["tb3_2"] += np.array([0.03, 0.0])
        controller.robot_yaws = {
            "tb3_0": 0.0,
            "tb3_1": math.pi / 2.0,
            "tb3_2": 0.0,
        }
        controller.transport_pre_staged = set(controller.robot_namespaces)
        controller.transport_staged = set()
        controller.transport_chain_released = {"tb3_1", "tb3_2"}
        controller.transport_compression_progress = 0.5
        controller.transport_compression_updated_at = 12.0
        controller.transport_assembly_route_states = {}
        controller.transport_last_rendezvous_log_count = None
        controller.transport_ready_hold_time = 0.0
        controller.transport_all_ready_since = None
        controller.avoidance_modules = {}

        controller._transport_odometry_error = lambda *_args: None
        controller._transport_target_error = lambda: None
        controller._transport_layout_error = lambda *_args, **_kwargs: None
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = lambda *_args, **_kwargs: False
        controller._transport_neighbours = lambda *_args: (
            {namespace: set() for namespace in targets},
            {namespace: set() for namespace in targets},
            {namespace: set() for namespace in targets},
        )
        controller._advance_transport_compression = lambda *_args: 0.5
        controller._nearby_chain_contacts = lambda *_args, **_kwargs: ()
        controller._parallel_lane_contacts = lambda *_args, **_kwargs: ()
        controller._transport_robot_lidar_masks = (
            lambda *_args, **_kwargs: ()
        )

        avoidance_calls = []

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            avoidance_calls.append(namespace)
            return command

        controller._apply_transport_avoidance = apply_avoidance
        published = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )
        original_publish = controller._publish_concurrent_approach_command
        publication_options = {}

        def capture_publish(namespace, *args, **kwargs):
            publication_options[namespace] = kwargs
            return original_publish(namespace, *args, **kwargs)

        controller._publish_concurrent_approach_command = capture_publish

        controller._approach_phase(4)

        settled_turn = published["tb3_1"]
        self.assertEqual(0.0, settled_turn.linear.x)
        self.assertNotEqual(0.0, settled_turn.angular.z)
        self.assertTrue(publication_options["tb3_1"]["alignment_only"])
        self.assertNotIn("tb3_1", avoidance_calls)

        guarded_translation = published["tb3_2"]
        self.assertNotEqual(0.0, guarded_translation.linear.x)
        self.assertFalse(publication_options["tb3_2"]["alignment_only"])
        self.assertIn("tb3_2", avoidance_calls)

    def _legacy_transport_waits_for_every_push_link_before_push_phase(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.25
        controller.robot_namespaces = ["tb3_0", "tb3_1"]
        controller.robot_positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.47, 0.0]),
        }
        controller.robot_yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        controller.robot_odom_received_at = {
            "tb3_0": 0.0, "tb3_1": 0.0
        }
        controller.transport_odom_timeout = 2.0
        controller.transport_contact_slop = 0.005
        controller._transport_target_error = lambda: None
        controller._transport_layout_error = lambda *_args, **_kwargs: None

        targets = {
            "tb3_0": {
                "role": "payload_push", "ready": False,
                "position_ready": False,
                "chain_index": 0, "chain_depth": 0,
                "position": np.array([-0.24, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
                "parent_namespace": None, "parent_position": np.zeros(2),
            },
            "tb3_1": {
                "role": "companion_push", "ready": False,
                "position_ready": False,
                "chain_index": 0, "chain_depth": 1,
                "position": np.array([-0.47, 0.0]),
                "staging_position": np.array([-0.47, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
                "parent_namespace": "tb3_0",
                "parent_position": controller.robot_positions["tb3_0"],
            },
        }
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = (
            lambda _position, _yaw, _object, _object_yaw, target,
            **kwargs: (
                target["ready"]
                if kwargs.get("require_inward_heading", True)
                else target["position_ready"]
            )
        )
        controller._slot_velocity = (
            lambda *_args: (ROS["transport"].Vec2(0.1, 0.0), True)
        )
        controller._transport_route_command = lambda *_args, **_kwargs: None
        def to_twist(vx, _vy, _yaw):
            command = Twist()
            command.linear.x = vx
            return command
        controller._holonomic_to_diff_drive = to_twist
        controller._payload_staging_command = (
            lambda *_args, **_kwargs: to_twist(0.1, 0.0, 0.0)
        )
        controller._guard_payload_approach = (
            lambda _position, _yaw, command, _object, _object_yaw:
            (command, False)
        )
        avoidance_calls = {}
        def apply_transport_avoidance(
            namespace, command, _object, **kwargs
        ):
            avoidance_calls[namespace] = kwargs
            return command
        controller._apply_transport_avoidance = apply_transport_avoidance
        published = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )
        phases = []
        controller._set_phase = (
            lambda phase, _epoch: phases.append(phase) or True
        )

        controller._approach_phase(3)
        self.assertEqual([], phases)
        self.assertEqual(0.1, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        self.assertTrue(
            avoidance_calls["tb3_0"]["allow_payload_contact"]
        )
        self.assertNotIn("tb3_1", avoidance_calls)

        targets["tb3_0"]["position_ready"] = True
        controller._approach_phase(3)
        self.assertEqual([], phases)
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        self.assertNotIn("tb3_1", avoidance_calls)
        self.assertEqual(
            set(), controller.transport_chain_released
        )

        targets["tb3_0"]["ready"] = True
        controller._approach_phase(3)
        self.assertEqual(0.1, published["tb3_1"].linear.x)
        self.assertEqual(
            {"tb3_1"}, controller.transport_chain_released
        )

        controller.robot_positions["tb3_0"] = np.array([-0.28, 0.0])
        controller._approach_phase(3)
        self.assertGreater(published["tb3_0"].linear.x, 0.0)
        controller.robot_positions["tb3_0"] = np.array([-0.24, 0.0])

        controller.robot_positions["tb3_1"] = np.array([-0.095, 0.0])
        avoidance_calls.clear()
        controller._approach_phase(3)
        child_call = avoidance_calls["tb3_1"]
        self.assertIsNone(child_call["allowed_contact_namespace"])
        self.assertEqual(
            set(), set(child_call["allowed_contact_namespaces"])
        )
        self.assertEqual(0, len(child_call["additional_lidar_masks"]))

        controller.robot_positions["tb3_1"] = np.array([-0.47, 0.0])
        targets["tb3_0"]["position_ready"] = False
        targets["tb3_0"]["ready"] = False
        controller._approach_phase(3)
        self.assertNotIn("tb3_0", controller.transport_staged)
        # Let the payload lead recover its slot before its child resumes.
        # This also keeps the lead's non-assembly route cache from being
        # replaced by an assembly mover later in the same control cycle.
        self.assertEqual(0.0, published["tb3_1"].linear.x)

        targets["tb3_0"]["position_ready"] = True
        targets["tb3_0"]["ready"] = True
        targets["tb3_1"]["position_ready"] = True
        targets["tb3_1"]["ready"] = True
        controller._parallel_push_rows_ready = lambda *_args: False
        controller._approach_phase(3)
        self.assertEqual([], phases)

        controller._parallel_push_rows_ready = lambda *_args: True
        controller._approach_phase(3)
        self.assertEqual(
            [ROS["transport"].TransportPhase.PUSH], phases
        )

    def test_transport_staging_requires_one_current_connected_chain(self):
        targets = {
            "tb3_0": {
                "chain_depth": 0,
                "parent_namespace": None,
            },
            "tb3_1": {
                "chain_depth": 1,
                "parent_namespace": "tb3_0",
            },
            "tb3_2": {
                "chain_depth": 2,
                "parent_namespace": "tb3_1",
            },
        }

        ready = ROS["transport"].CollaborativeTransport._ready_chain_prefix(
            targets, {"tb3_0", "tb3_2"}
        )
        self.assertEqual({"tb3_0"}, ready)

        ready = ROS["transport"].CollaborativeTransport._ready_chain_prefix(
            targets, set(targets)
        )
        self.assertEqual(set(targets), ready)

    def _legacy_large_transport_routes_all_parking_before_first_release(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.25
        controller.robot_namespaces = [
            "tb3_0", "tb3_1", "tb3_2", "tb3_3", "tb3_4"
        ]
        controller.robot_positions = {
            "tb3_0": np.array([-0.24, 0.18]),
            "tb3_1": np.array([-0.24, -0.18]),
            "tb3_2": np.array([-1.40, 0.70]),
            "tb3_3": np.array([-1.30, -0.70]),
            "tb3_4": np.array([-1.60, 0.35]),
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.robot_velocities = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_odom_received_at = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.transport_odom_timeout = 2.0
        controller.transport_contact_slop = 0.005
        controller.transport_chain_staging_tolerance = 0.025
        controller.transport_chain_staging_release_tolerance = 0.06
        controller.transport_pre_staged = set()
        controller.transport_staged = set()
        controller.transport_chain_released = set()
        controller.vmax = 0.20
        controller.avoidance_modules = {}
        controller._transport_target_error = lambda: None
        controller._transport_layout_error = (
            lambda *_args, **_kwargs: None
        )
        controller._transport_odometry_error = (
            lambda *_args, **_kwargs: None
        )

        goal = np.array([1.0, 0.0])
        targets = {
            "tb3_0": {
                "role": "payload_push", "chain_index": 0,
                "chain_depth": 0, "ready": True,
                "position": np.array([-0.24, 0.18]),
                "staging_position": np.array([-0.24, 0.18]),
                "push_direction": goal.copy(),
                "parent_namespace": None,
                "parent_position": np.zeros(2),
            },
            "tb3_1": {
                "role": "payload_push", "chain_index": 1,
                "chain_depth": 0, "ready": True,
                "position": np.array([-0.24, -0.18]),
                "staging_position": np.array([-0.24, -0.18]),
                "push_direction": goal.copy(),
                "parent_namespace": None,
                "parent_position": np.zeros(2),
            },
            "tb3_2": {
                "role": "companion_push", "chain_index": 0,
                "chain_depth": 1, "ready": False,
                "position": np.array([-0.385, 0.18]),
                "assembly_position": np.array([-0.49, 0.18]),
                "staging_position": np.array([-0.80, 0.18]),
                "push_direction": goal.copy(),
                "parent_namespace": "tb3_0",
                "parent_position": controller.robot_positions["tb3_0"],
            },
            "tb3_3": {
                "role": "companion_push", "chain_index": 1,
                "chain_depth": 1, "ready": False,
                "position": np.array([-0.385, -0.18]),
                "assembly_position": np.array([-0.49, -0.18]),
                "staging_position": np.array([-0.80, -0.18]),
                "push_direction": goal.copy(),
                "parent_namespace": "tb3_1",
                "parent_position": controller.robot_positions["tb3_1"],
            },
            "tb3_4": {
                "role": "companion_push", "chain_index": 0,
                "chain_depth": 2, "ready": False,
                "position": np.array([-0.53, 0.18]),
                "assembly_position": np.array([-0.735, 0.18]),
                "staging_position": np.array([-1.18, 0.18]),
                "push_direction": goal.copy(),
                "parent_namespace": "tb3_2",
                "parent_position": controller.robot_positions["tb3_2"],
            },
        }
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = (
            lambda _position, _yaw, _object, _object_yaw, target,
            **_kwargs: target["ready"]
        )
        empty_neighbours = {
            namespace: set() for namespace in controller.robot_namespaces
        }
        controller._transport_neighbours = lambda _targets: (
            empty_neighbours, empty_neighbours, empty_neighbours
        )
        route_calls = []

        def route_command(
            namespace, _position, _yaw, destination, _positions,
            route_kind, **kwargs
        ):
            route_calls.append({
                "namespace": namespace,
                "destination": np.asarray(destination).copy(),
                "kind": route_kind,
                "close": set(kwargs.get(
                    "close_approach_namespaces", ()
                )),
                "handoff": kwargs.get("final_handoff_tolerance"),
            })
            command = Twist()
            command.linear.x = 0.08
            return command

        controller._transport_route_command = route_command
        controller._guard_payload_approach = (
            lambda _position, _yaw, command, _object, _object_yaw:
            (command, False)
        )
        controller._apply_transport_avoidance = (
            lambda _namespace, command, _object, **_kwargs: command
        )
        controller._publish_command = lambda *_args, **_kwargs: True
        controller._set_phase = lambda *_args, **_kwargs: True

        controller._approach_phase(4)
        self.assertEqual(set(), controller.transport_chain_released)
        self.assertEqual(
            [("tb3_2", "pre_staging")],
            [(call["namespace"], call["kind"]) for call in route_calls],
        )
        self.assertEqual(set(), route_calls[-1]["close"])
        self.assertAlmostEqual(
            controller.transport_chain_staging_tolerance,
            route_calls[-1]["handoff"],
        )

        controller.robot_positions["tb3_2"] = (
            targets["tb3_2"]["staging_position"].copy()
        )
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(set(), controller.transport_chain_released)
        self.assertEqual("tb3_3", route_calls[-1]["namespace"])
        self.assertEqual("pre_staging", route_calls[-1]["kind"])
        self.assertEqual(set(), route_calls[-1]["close"])

        # A row that already reached and faced forward at its parking point
        # stays qualified while another row moves. Minor passive yaw drift
        # must not steal the singleton parking route back from tb3_3.
        controller.robot_yaws["tb3_2"] = math.pi
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual("tb3_3", route_calls[-1]["namespace"])
        self.assertEqual("pre_staging", route_calls[-1]["kind"])
        controller.robot_yaws["tb3_2"] = 0.0

        controller.robot_positions["tb3_3"] = (
            targets["tb3_3"]["staging_position"].copy()
        )
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(set(), controller.transport_chain_released)
        self.assertEqual("tb3_4", route_calls[-1]["namespace"])
        self.assertEqual("pre_staging", route_calls[-1]["kind"])
        self.assertEqual(set(), route_calls[-1]["close"])

        controller.robot_positions["tb3_4"] = (
            targets["tb3_4"]["staging_position"].copy()
        )
        controller.robot_yaws["tb3_4"] = math.pi
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(set(), controller.transport_chain_released)
        self.assertEqual(
            [("tb3_4", "pre_staging")],
            [(call["namespace"], call["kind"]) for call in route_calls],
        )

        controller.robot_yaws["tb3_4"] = 0.0
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(
            {"tb3_2", "tb3_3"},
            controller.transport_chain_released,
        )
        self.assertFalse(any(
            call["kind"] == "pre_staging" for call in route_calls
        ))
        self.assertEqual(
            [("tb3_2", "assembly"), ("tb3_3", "assembly")],
            [(call["namespace"], call["kind"]) for call in route_calls],
        )

        # Both independent route caches stay active on the next control pass.
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(
            [("tb3_2", "assembly"), ("tb3_3", "assembly")],
            [(call["namespace"], call["kind"]) for call in route_calls],
        )

        # The completed lane can release its next link without interrupting
        # the mover that is still closing in the opposite lane.
        controller.robot_positions["tb3_2"] = (
            targets["tb3_2"]["assembly_position"].copy()
        )
        targets["tb3_2"]["ready"] = True
        route_calls.clear()
        controller._approach_phase(4)
        self.assertEqual(
            {"tb3_2", "tb3_3", "tb3_4"},
            controller.transport_chain_released,
        )
        self.assertEqual(
            [("tb3_3", "assembly"), ("tb3_4", "assembly")],
            [(call["namespace"], call["kind"]) for call in route_calls],
        )

    def test_transport_route_goes_around_a_held_robot(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.model_lock = threading.Lock()
        controller.model_poses = {}
        controller.spawn_exclusion_zones = []
        controller.arena_size = 10.0
        controller.arena_margin = 0.35
        controller.arena_profile = "swarm_arena"
        controller.transport_route_obstacle_clearance = 0.30
        controller.transport_route_robot_clearance = 0.26
        positions = {
            "tb3_0": np.array([-1.0, 0.0]),
            "tb3_1": np.array([0.0, 0.0]),
        }

        route = controller._plan_transport_route(
            "tb3_0",
            positions["tb3_0"],
            np.array([1.0, 0.0]),
            positions,
        )
        direct_route = controller._plan_transport_route(
            "tb3_0",
            positions["tb3_0"],
            np.array([1.0, 0.0]),
            positions,
            ignored_namespaces=("tb3_1",),
        )

        self.assertIsNotNone(route)
        self.assertGreater(len(route), 2)
        self.assertEqual(2, len(direct_route))
        self.assertEqual((1.0, 0.0), route[-1])

    def test_transport_route_command_reuses_and_advances_waypoints(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        plan_calls = []
        controller._plan_transport_route = lambda *args, **kwargs: (
            plan_calls.append((args, kwargs))
            or [(-1.0, 0.0), (0.0, 1.0), (1.0, 0.0)]
        )
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )
        positions = {"tb3_0": np.array([-1.0, 0.0])}

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=1.0,
            create=True,
        ):
            first = controller._transport_route_command(
                "tb3_0", positions["tb3_0"], 0.0,
                np.array([1.0, 0.0]), positions, "parking",
            )
            second = controller._transport_route_command(
                "tb3_0", np.array([0.0, 0.95]), 0.0,
                np.array([1.0, 0.0]), positions, "parking",
            )
            final = controller._transport_route_command(
                "tb3_0", np.array([0.95, 0.0]), 0.0,
                np.array([1.0, 0.0]), positions, "parking",
            )
            after_handoff = controller._transport_route_command(
                "tb3_0", np.array([0.70, 0.0]), 0.0,
                np.array([1.0, 0.0]), positions, "parking",
            )

        self.assertEqual(1, len(plan_calls))
        self.assertGreater(first.vy, 0.0)
        self.assertGreater(second.vx, 0.0)
        self.assertIsNone(final)
        self.assertIsNone(after_handoff)

    def test_parking_route_reaches_a_detour_corner_before_turning(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller.transport_parking_route_waypoint_tolerance = 0.04
        controller._reset_transport_route()
        controller._plan_transport_route = (
            lambda _namespace, start, end, *_args, **_kwargs: [
                tuple(start), (0.0, 1.0), tuple(end)
            ]
        )
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )
        reset_motion = mock.Mock()
        controller.avoidance_modules = {
            "tb3_0": Message(reset_motion=reset_motion)
        }
        positions = {"tb3_0": np.array([-1.0, 0.0])}

        controller._transport_route_command(
            "tb3_0", positions["tb3_0"], 0.0,
            np.array([1.0, 0.0]), positions, "pre_staging",
        )
        before_corner = controller._transport_route_command(
            "tb3_0", np.array([0.0, 0.95]), 0.0,
            np.array([1.0, 0.0]), positions, "pre_staging",
        )
        after_corner = controller._transport_route_command(
            "tb3_0", np.array([0.0, 0.97]), 0.0,
            np.array([1.0, 0.0]), positions, "pre_staging",
        )

        self.assertAlmostEqual(0.0, before_corner.vx)
        self.assertGreater(before_corner.vy, 0.0)
        self.assertGreater(after_corner.vx, 0.0)
        self.assertLess(after_corner.vy, 0.0)
        reset_motion.assert_called_once_with()

    def test_concurrent_assembly_routes_keep_independent_waypoints(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        plan_calls = []

        def plan_route(namespace, start, end, *_args, **_kwargs):
            plan_calls.append(namespace)
            middle = (
                (0.0, 1.0)
                if namespace == "tb3_0"
                else (0.0, -1.0)
            )
            return [tuple(start), middle, tuple(end)]

        controller._plan_transport_route = plan_route
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )
        positions = {
            "tb3_0": np.array([-1.0, 0.0]),
            "tb3_1": np.array([-1.0, 0.0]),
        }
        destination = np.array([1.0, 0.0])

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=1.0,
            create=True,
        ):
            left_first = controller._transport_route_command(
                "tb3_0", positions["tb3_0"], 0.0,
                destination, positions, "assembly",
            )
            right_first = controller._transport_route_command(
                "tb3_1", positions["tb3_1"], 0.0,
                destination, positions, "assembly",
            )
            left_next = controller._transport_route_command(
                "tb3_0", np.array([0.0, 0.95]), 0.0,
                destination, positions, "assembly",
            )
            right_again = controller._transport_route_command(
                "tb3_1", positions["tb3_1"], 0.0,
                destination, positions, "assembly",
            )

        self.assertEqual(["tb3_0", "tb3_1"], plan_calls)
        self.assertGreater(left_first.vy, 0.0)
        self.assertLess(right_first.vy, 0.0)
        self.assertGreater(left_next.vx, 0.0)
        self.assertLess(right_again.vy, 0.0)

        states = controller.transport_assembly_route_states
        self.assertEqual({"tb3_0", "tb3_1"}, set(states))
        self.assertEqual(1, states["tb3_0"]["waypoint_index"])
        self.assertEqual(0, states["tb3_1"]["waypoint_index"])

    def test_mutual_rendezvous_blockers_freeze_shortest_route_priority(self):
        outcomes = []
        for start_order in (("tb3_0", "tb3_5"), ("tb3_5", "tb3_0")):
            controller, positions, targets = (
                self._mutual_rendezvous_controller()
            )
            with mock.patch.object(
                ROS["transport"].rospy,
                "get_time",
                return_value=3.0,
                create=True,
            ):
                for namespace in start_order:
                    self.assertTrue(controller._start_rendezvous_recovery(
                        namespace,
                        positions[namespace],
                        targets[namespace],
                        positions,
                    ))

            pair = ("tb3_0", "tb3_5")
            decision = controller.transport_rendezvous_pair_decisions[pair]
            self.assertEqual("tb3_5", decision["priority_namespace"])
            self.assertEqual("tb3_0", decision["yielding_namespace"])
            self.assertAlmostEqual(
                0.210, decision["route_lengths"]["tb3_5"]
            )
            self.assertAlmostEqual(
                0.441, decision["route_lengths"]["tb3_0"]
            )

            winner = controller.transport_rendezvous_recoveries["tb3_5"]
            yielder = controller.transport_rendezvous_recoveries["tb3_0"]
            self.assertTrue(winner["right_of_way"])
            self.assertFalse(yielder["right_of_way"])
            self.assertIsNone(controller._rendezvous_recovery_command(
                "tb3_5", positions["tb3_5"], 0.0,
                targets["tb3_5"], positions,
            ))
            yield_command = controller._rendezvous_recovery_command(
                "tb3_0", positions["tb3_0"], 0.0,
                targets["tb3_0"], positions,
            )
            self.assertGreater(math.hypot(
                yield_command.vx, yield_command.vy
            ), 0.0)
            radial = (
                positions["tb3_0"] - positions["tb3_5"]
            )
            radial /= np.linalg.norm(radial)
            self.assertGreater(float(np.dot(
                yielder["direction"], radial
            )), 0.0)

            # The winner's next plan sees the yielder as a held obstacle,
            # while unrelated active movers stay concurrent and ignored.
            active = set(positions)
            winner_ignored = (
                controller._rendezvous_route_ignored_namespaces(
                    "tb3_5", active
                )
            )
            self.assertNotIn("tb3_0", winner_ignored)
            self.assertIn("tb3_7", winner_ignored)
            self.assertIn(
                "tb3_5",
                controller._rendezvous_route_ignored_namespaces(
                    "tb3_0", active
                ),
            )

            # Replacing the winner's route cache cannot swap the frozen pair
            # decision during this contention epoch.
            controller.transport_assembly_route_states["tb3_5"] = {
                "kind": "rendezvous",
                "target": np.array([4.0, 4.0]),
                "waypoints": [np.array([4.0, 4.0])],
                "waypoint_index": 0,
            }
            controller._coordinate_mutual_rendezvous_recovery(
                pair, positions
            )
            self.assertEqual(
                "tb3_5",
                controller.transport_rendezvous_pair_decisions[pair][
                    "priority_namespace"
                ],
            )
            outcomes.append((
                decision["passing_sign"], yielder["direction"].copy()
            ))

        self.assertEqual(outcomes[0][0], outcomes[1][0])
        np.testing.assert_allclose(outcomes[0][1], outcomes[1][1])
        self.assertEqual(
            ("tb3_2", "tb3_10"),
            controller._rendezvous_pair_key("tb3_10", "tb3_2"),
        )

    def test_rendezvous_recovery_starts_when_avoidance_only_allows_turning(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.avoidance_modules = {}

        positions = {"tb3_0": np.array([0.0, 0.0])}
        yaws = {"tb3_0": 0.0}
        target = {
            "role": "companion_push",
            "parent_namespace": None,
            "push_direction": np.array([1.0, 0.0]),
        }
        targets = {"tb3_0": target}
        empty_neighbours = ({"tb3_0": set()},) * 3

        controller._rendezvous_recovery_command = mock.Mock(
            side_effect=[None, Message(
                linear=Message(x=0.06), angular=Message(z=0.0)
            )]
        )
        controller._start_rendezvous_recovery = mock.Mock(return_value=True)
        controller._transport_robot_lidar_masks = lambda *_args: ()
        controller._nearby_chain_contacts = lambda *_args: ()
        controller._parallel_lane_contacts = lambda *_args: ()

        avoidance_calls = []

        def apply_avoidance(_namespace, command, *_args, **_kwargs):
            avoidance_calls.append(command)
            if len(avoidance_calls) == 1:
                return Message(
                    linear=Message(x=0.0), angular=Message(z=0.45)
                )
            return command

        controller._apply_transport_avoidance = apply_avoidance
        published = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )

        requested = Twist()
        requested.linear.x = 0.08
        controller._publish_concurrent_approach_command(
            "tb3_0",
            requested,
            target,
            positions,
            yaws,
            targets,
            np.array([1.0, 1.0]),
            0.0,
            empty_neighbours,
            4,
        )

        controller._start_rendezvous_recovery.assert_called_once()
        _args, call_kwargs = (
            controller._start_rendezvous_recovery.call_args
        )
        np.testing.assert_allclose(
            np.array([1.0, 0.0]), call_kwargs["travel_direction"]
        )
        self.assertEqual(2, len(avoidance_calls))
        self.assertAlmostEqual(0.06, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_0"].angular.z)

    def test_rendezvous_recovery_selects_a_blocker_in_front(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_route_robot_clearance = 0.32
        controller.transport_rendezvous_recoveries = {}
        controller.transport_rendezvous_recovery_cooldowns = {}
        controller.transport_rendezvous_pair_decisions = {}
        controller.transport_assembly_route_states = {}
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([0.25, 0.04]),
            "tb3_2": np.array([-0.08, 0.0]),
            "tb3_3": np.array([0.05, 0.40]),
        }
        target = {
            "position": np.array([1.0, 0.0]),
            "staging_position": np.array([1.0, 0.0]),
        }

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=3.0,
            create=True,
        ):
            started = controller._start_rendezvous_recovery(
                "tb3_0",
                positions["tb3_0"],
                target,
                positions,
                travel_direction=np.array([1.0, 0.0]),
            )

        self.assertTrue(started)
        self.assertEqual(
            "tb3_1",
            controller.transport_rendezvous_recoveries["tb3_0"][
                "obstacle_namespace"
            ],
        )

    def test_rendezvous_recovery_ignores_robots_outside_route(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_route_robot_clearance = 0.32
        controller.transport_rendezvous_recoveries = {}
        controller.transport_rendezvous_recovery_cooldowns = {}
        controller.transport_rendezvous_pair_decisions = {}
        controller.transport_assembly_route_states = {}
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([-0.08, 0.0]),
            "tb3_2": np.array([0.05, 0.40]),
        }
        target = {
            "position": np.array([1.0, 0.0]),
            "staging_position": np.array([1.0, 0.0]),
        }

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=3.0,
            create=True,
        ):
            started = controller._start_rendezvous_recovery(
                "tb3_0",
                positions["tb3_0"],
                target,
                positions,
                travel_direction=np.array([1.0, 0.0]),
            )

        self.assertFalse(started)
        self.assertEqual({}, controller.transport_rendezvous_recoveries)

    def test_static_obstacle_turn_does_not_invent_a_robot_detour(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.avoidance_modules = {}
        controller.transport_route_robot_clearance = 0.32
        controller.transport_rendezvous_recoveries = {}
        controller.transport_rendezvous_recovery_cooldowns = {}
        controller.transport_rendezvous_pair_decisions = {}
        controller.transport_assembly_route_states = {}

        positions = {"tb3_0": np.array([0.0, 0.0])}
        yaws = {"tb3_0": 0.0}
        target = {
            "role": "companion_push",
            "parent_namespace": None,
            "position": np.array([1.0, 0.0]),
            "staging_position": np.array([1.0, 0.0]),
        }
        targets = {"tb3_0": target}
        empty_neighbours = ({"tb3_0": set()},) * 3

        controller._transport_robot_lidar_masks = lambda *_args: ()
        controller._nearby_chain_contacts = lambda *_args: ()
        controller._parallel_lane_contacts = lambda *_args: ()
        controller._apply_transport_avoidance = (
            lambda *_args, **_kwargs: Message(
                linear=Message(x=0.0), angular=Message(z=0.45)
            )
        )
        published = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )

        requested = Twist()
        requested.linear.x = 0.08
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=3.0,
            create=True,
        ):
            controller._publish_concurrent_approach_command(
                "tb3_0",
                requested,
                target,
                positions,
                yaws,
                targets,
                np.array([1.0, 1.0]),
                0.0,
                empty_neighbours,
                4,
            )

        self.assertEqual({}, controller.transport_rendezvous_recoveries)
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertAlmostEqual(0.45, published["tb3_0"].angular.z)

    def test_mutual_rendezvous_yield_separates_and_does_not_release_early(self):
        controller, positions, targets = self._mutual_rendezvous_controller()
        clock = [0.0]
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            side_effect=lambda: clock[0],
            create=True,
        ):
            controller._start_rendezvous_recovery(
                "tb3_0", positions["tb3_0"], targets["tb3_0"], positions
            )
            controller._start_rendezvous_recovery(
                "tb3_5", positions["tb3_5"], targets["tb3_5"], positions
            )

            initial_yielder = positions["tb3_0"].copy()
            initial_winner = positions["tb3_5"].copy()
            minimum_separation = float(np.linalg.norm(
                initial_winner - initial_yielder
            ))
            checked_old_release_distance = False
            for _step in range(100):
                yielder_command = controller._rendezvous_recovery_command(
                    "tb3_0", positions["tb3_0"], 0.0,
                    targets["tb3_0"], positions,
                )
                if yielder_command is None:
                    break
                yielder_velocity = np.array([
                    getattr(yielder_command, "vx", 0.0),
                    getattr(yielder_command, "vy", 0.0),
                ])
                positions["tb3_0"] += yielder_velocity * 0.10

                # Stand in for the normal avoidance guard: the winner keeps
                # making route progress once the yielding pocket is clear.
                separation = float(np.linalg.norm(
                    positions["tb3_5"] - positions["tb3_0"]
                ))
                winner_remaining = (
                    targets["tb3_5"]["staging_position"]
                    - positions["tb3_5"]
                )
                winner_distance = float(np.linalg.norm(winner_remaining))
                if separation >= 0.34 and winner_distance > 0.0:
                    winner_step = min(0.006, winner_distance)
                    positions["tb3_5"] += (
                        winner_remaining / winner_distance * winner_step
                    )

                separation = float(np.linalg.norm(
                    positions["tb3_5"] - positions["tb3_0"]
                ))
                minimum_separation = min(minimum_separation, separation)
                travelled = float(np.linalg.norm(
                    positions["tb3_0"] - initial_yielder
                ))
                if travelled >= 0.17 and not checked_old_release_distance:
                    checked_old_release_distance = True
                    self.assertIn(
                        "tb3_0", controller.transport_rendezvous_recoveries
                    )

                if winner_distance <= 0.02:
                    controller.transport_pre_staged.add("tb3_5")
                clock[0] += 0.10
            else:
                self.fail("mutual rendezvous pair did not separate")

        self.assertTrue(checked_old_release_distance)
        self.assertGreaterEqual(minimum_separation, 0.28)
        self.assertGreater(float(np.linalg.norm(
            positions["tb3_0"] - initial_yielder
        )), 0.17)
        self.assertGreater(float(np.linalg.norm(
            positions["tb3_5"] - initial_winner
        )), 0.17)
        self.assertEqual({}, controller.transport_rendezvous_recoveries)
        self.assertEqual({}, controller.transport_rendezvous_pair_decisions)

    def test_mutual_rendezvous_release_requires_conflict_free_dwell(self):
        controller, positions, targets = self._mutual_rendezvous_controller()
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=1.0,
            create=True,
        ):
            controller._start_rendezvous_recovery(
                "tb3_0", positions["tb3_0"], targets["tb3_0"], positions
            )
            controller._start_rendezvous_recovery(
                "tb3_5", positions["tb3_5"], targets["tb3_5"], positions
            )

        pair = ("tb3_0", "tb3_5")
        # First keep two safely separated paths crossing one another. Distance
        # alone must not release the pair.
        positions["tb3_0"] = np.array([-0.5, 0.0])
        positions["tb3_5"] = np.array([0.5, 0.0])
        controller.transport_rendezvous_recoveries["tb3_0"][
            "destination"
        ] = np.array([1.0, 0.0])
        controller.transport_rendezvous_recoveries["tb3_5"][
            "destination"
        ] = np.array([-1.0, 0.0])
        controller.transport_assembly_route_states = {}
        self.assertFalse(controller._rendezvous_pair_is_clear(
            pair, positions, 2.0
        ))

        # Once the remaining paths diverge, the geometry must remain clear
        # for a complete dwell instead of releasing on one noisy sample.
        controller.transport_rendezvous_recoveries["tb3_0"][
            "destination"
        ] = np.array([-1.0, 0.0])
        controller.transport_rendezvous_recoveries["tb3_5"][
            "destination"
        ] = np.array([1.0, 0.0])
        self.assertFalse(controller._rendezvous_pair_is_clear(
            pair, positions, 3.0
        ))
        self.assertFalse(controller._rendezvous_pair_is_clear(
            pair, positions, 3.39
        ))
        self.assertTrue(controller._rendezvous_pair_is_clear(
            pair, positions, 3.41
        ))

    def test_mixed_concurrent_route_kinds_keep_independent_caches(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        middles = {
            "tb3_0": np.array([0.0, 1.0]),
            "tb3_1": np.array([0.0, 0.0]),
            "tb3_2": np.array([0.0, -1.0]),
        }
        starts = {
            "tb3_0": np.array([-1.0, 0.2]),
            "tb3_1": np.array([-1.0, 0.0]),
            "tb3_2": np.array([-1.0, -0.2]),
        }
        destinations = {
            "tb3_0": np.array([1.0, 0.2]),
            "tb3_1": np.array([1.0, 0.0]),
            "tb3_2": np.array([1.0, -0.2]),
        }
        kinds = {
            "tb3_0": "rendezvous",
            "tb3_1": "payload_staging",
            "tb3_2": "assembly",
        }
        plan_calls = []

        def plan_route(namespace, start, end, *_args, **_kwargs):
            plan_calls.append((namespace, kinds[namespace]))
            return [tuple(start), tuple(middles[namespace]), tuple(end)]

        controller._plan_transport_route = plan_route
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=1.0,
            create=True,
        ):
            for namespace in starts:
                controller._transport_route_command(
                    namespace,
                    starts[namespace],
                    0.0,
                    destinations[namespace],
                    starts,
                    kinds[namespace],
                )

            # Advancing one rendezvous route must not advance or replace the
            # payload and assembly routes that share this control cycle.
            controller._transport_route_command(
                "tb3_0",
                middles["tb3_0"] + np.array([0.0, -0.04]),
                0.0,
                destinations["tb3_0"],
                starts,
                kinds["tb3_0"],
            )
            controller._transport_route_command(
                "tb3_2",
                starts["tb3_2"],
                0.0,
                destinations["tb3_2"],
                starts,
                kinds["tb3_2"],
            )

        self.assertEqual(
            [(namespace, kinds[namespace]) for namespace in starts],
            plan_calls,
        )
        states = controller.transport_assembly_route_states
        self.assertEqual(set(starts), set(states))
        for namespace, route_kind in kinds.items():
            self.assertEqual(route_kind, states[namespace]["kind"])
            np.testing.assert_allclose(
                destinations[namespace], states[namespace]["target"]
            )
        self.assertEqual(1, states["tb3_0"]["waypoint_index"])
        self.assertEqual(0, states["tb3_1"]["waypoint_index"])
        self.assertEqual(0, states["tb3_2"]["waypoint_index"])

    def test_companion_route_uses_its_tight_handoff_tolerance(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller.transport_route_final_handoff_tolerance = 0.25
        controller._reset_transport_route()
        controller._plan_transport_route = (
            lambda _namespace, start, end, *_args, **_kwargs:
            [tuple(start), tuple(end)]
        )
        controller._holonomic_to_diff_drive = (
            lambda vx, vy, _yaw: Message(vx=vx, vy=vy)
        )
        destination = np.array([1.0, 0.0])
        positions = {"tb3_0": np.array([0.95, 0.0])}

        still_routing = controller._transport_route_command(
            "tb3_0",
            positions["tb3_0"],
            0.0,
            destination,
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )
        handoff = controller._transport_route_command(
            "tb3_0",
            np.array([0.99, 0.0]),
            0.0,
            destination,
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )

        self.assertGreater(still_routing.vx, 0.0)
        self.assertIsNone(handoff)

    def test_transport_route_at_destination_hands_off_immediately(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        controller._plan_transport_route = (
            lambda _namespace, start, _end, *_args, **_kwargs:
            [tuple(start)]
        )
        destination = np.array([1.0, 0.0])
        positions = {"tb3_0": destination.copy()}

        command = controller._transport_route_command(
            "tb3_0",
            destination,
            0.0,
            destination,
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )

        self.assertIsNone(command)
        self.assertTrue(
            controller.transport_assembly_route_states[
                "tb3_0"
            ]["complete"]
        )

    def test_transport_route_reverses_when_the_last_point_is_behind(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        controller._plan_transport_route = (
            lambda _namespace, start, end, *_args, **_kwargs:
            [tuple(start), tuple(end)]
        )
        positions = {"tb3_0": np.array([0.0, 0.0])}

        command = controller._transport_route_command(
            "tb3_0",
            positions["tb3_0"],
            0.0,
            np.array([-1.0, 0.0]),
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )

        self.assertLess(command.linear.x, 0.0)
        self.assertAlmostEqual(0.0, command.angular.z, places=6)

        turn = controller._transport_route_command(
            "tb3_0",
            np.array([-0.89, 0.0]),
            0.0,
            np.array([-1.0, 0.0]),
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )
        self.assertAlmostEqual(0.0, turn.linear.x, places=6)
        self.assertAlmostEqual(0.90, abs(turn.angular.z), places=6)
        self.assertFalse(controller.transport_route_reverse_active)
        self.assertTrue(controller.transport_route_reverse_finished)
        turn_direction = math.copysign(1.0, turn.angular.z)
        self.assertEqual(
            turn_direction,
            controller.transport_assembly_route_states[
                "tb3_0"
            ]["turn_direction"],
        )

        aligned_stop = controller._transport_route_command(
            "tb3_0",
            np.array([-0.89, 0.0]),
            math.pi,
            np.array([-1.0, 0.0]),
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )
        self.assertAlmostEqual(0.0, aligned_stop.linear.x, places=6)
        self.assertAlmostEqual(0.0, aligned_stop.angular.z, places=6)
        self.assertEqual(
            0.0,
            controller.transport_assembly_route_states[
                "tb3_0"
            ]["turn_direction"],
        )

        forward = controller._transport_route_command(
            "tb3_0",
            np.array([-0.89, 0.0]),
            math.pi,
            np.array([-1.0, 0.0]),
            positions,
            "assembly",
            final_handoff_tolerance=0.015,
        )
        self.assertGreater(forward.linear.x, 0.0)

    def test_transport_parking_route_keeps_its_forward_heading(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.20
        controller.transport_chain_staging_speed = 0.16
        controller.transport_route_waypoint_tolerance = 0.09
        controller._reset_transport_route()
        controller._plan_transport_route = (
            lambda _namespace, start, end, *_args, **_kwargs:
            [tuple(start), tuple(end)]
        )
        positions = {"tb3_0": np.array([0.0, 0.0])}

        command = controller._transport_route_command(
            "tb3_0",
            positions["tb3_0"],
            0.0,
            np.array([-0.20, 0.0]),
            positions,
            "pre_staging",
            final_handoff_tolerance=0.015,
        )

        self.assertAlmostEqual(0.0, command.linear.x, places=6)
        self.assertGreater(abs(command.angular.z), 1.0)

    def test_payload_staging_controller_converges_from_diagonal_handoff(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_chain_staging_tolerance = 0.025
        destination = np.array([-0.8, -1.107])
        push_direction = np.array([0.0, -1.0])
        position = np.array([-0.64449, -0.81125])
        yaw = -2.24214
        maximum_payload_side_error = 0.0

        for _ in range(2000):
            command = controller._payload_staging_command(
                position, yaw, destination, push_direction
            )
            step = 0.02
            position += np.array([math.cos(yaw), math.sin(yaw)]) * (
                command.linear.x * step
            )
            yaw = controller._normalize_angle(
                yaw + command.angular.z * step
            )
            maximum_payload_side_error = max(
                maximum_payload_side_error,
                float(np.dot(position - destination, push_direction)),
            )

        self.assertLess(float(np.linalg.norm(position - destination)), 0.0125)
        self.assertLess(
            abs(controller._normalize_angle(yaw + math.pi / 2.0)),
            0.01,
        )
        self.assertLess(maximum_payload_side_error, 0.01)

    def test_payload_staging_controller_reverses_after_overshoot(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_chain_staging_tolerance = 0.025
        command = controller._payload_staging_command(
            np.array([0.10, 0.0]),
            0.0,
            np.array([0.0, 0.0]),
            np.array([1.0, 0.0]),
        )

        self.assertLess(command.linear.x, 0.0)
        self.assertAlmostEqual(0.0, command.angular.z, places=6)

    def test_payload_staging_controller_rotates_before_diagonal_motion(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_chain_staging_tolerance = 0.025
        command = controller._payload_staging_command(
            np.array([0.0, 0.0]),
            0.0,
            np.array([1.0, 1.0]),
            np.array([1.0, 0.0]),
        )

        self.assertAlmostEqual(0.0, command.linear.x, places=6)
        self.assertGreater(command.angular.z, 0.0)

    def test_payload_route_handoff_stays_behind_staging_plane(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_route_waypoint_tolerance = 0.09
        controller.transport_route_final_handoff_tolerance = 0.25
        controller.transport_chain_staging_tolerance = 0.025
        controller.robot_forward_contact_extent = 0.038
        controller.transport_contact_slop = 0.005
        controller.transport_staging_clearance = 0.25
        controller.transport_payload_route_awareness = 0.80
        controller.transport_payload_route_lidar_offset = 0.032
        controller.transport_payload_route_margin = 0.05
        target = {
            "position": np.array([0.0, 0.0]),
            "staging_position": np.array([-0.25, 0.0]),
            "push_direction": np.array([1.0, 0.0]),
        }

        route_destination = controller._payload_staging_route_destination(
            target
        )
        offset = float(np.dot(
            target["staging_position"] - route_destination,
            target["push_direction"],
        ))

        staging_surface_gap = 0.038 + 0.005 + 0.25
        self.assertGreaterEqual(
            staging_surface_gap + offset,
            0.80 + 0.032 + 0.05,
        )

    def test_payload_approach_guard_stops_only_closing_motion(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.object_half_width = 0.20
        controller.object_half_height = 0.20
        controller.robot_forward_contact_extent = 0.038
        controller.transport_contact_slop = 0.005
        controller.transport_payload_approach_min_clearance = 0.08
        closing = Twist()
        closing.linear.x = 0.08
        retreating = Twist()
        retreating.linear.x = -0.06
        position = np.array([-0.30, 0.0])
        object_position = np.array([0.0, 0.0])

        guarded, stopped = controller._guard_payload_approach(
            position, 0.0, closing, object_position, 0.0
        )
        retreat, retreat_stopped = controller._guard_payload_approach(
            position, 0.0, retreating, object_position, 0.0
        )

        self.assertTrue(stopped)
        self.assertEqual(0.0, guarded.linear.x)
        self.assertFalse(retreat_stopped)
        self.assertLess(retreat.linear.x, 0.0)

    def test_transport_advances_a_complete_queue_to_the_payload(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.command_lock = threading.RLock()
        controller.phase_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.avoidance_modules = {}
        controller.transport_engaged = set()
        controller.transport_engagement_complete = False
        controller.transport_engagement_ready_since = None
        controller.transport_engagement_hold_time = 0.0
        controller.transport_engagement_release_hold_time = 0.0
        controller.transport_queue_docking_started = False
        controller.transport_queue_settle_started_at = None
        controller.transport_launch_settle_time = 0.35
        controller.transport_launch_settle_timeout = 1.25
        controller.transport_launch_settle_speed = 0.004
        controller.transport_contact_heading_tolerance = 0.45
        controller.transport_contact_slop = 0.005
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_closing_speed = 0.018
        controller.vmax = 0.20
        controller.transport_engagement_speed = 0.12
        positions = {
            "tb3_0": np.array([-0.10, 0.0]),
            "tb3_1": np.array([-0.245, 0.0]),
        }
        yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        targets = {
            "tb3_0": {
                "role": "payload_push", "chain_depth": 0,
                "chain_index": 0, "parent_namespace": None,
                "position": np.array([0.0, 0.0]),
                "staging_position": np.array([-0.10, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "role": "companion_push", "chain_depth": 1,
                "chain_index": 0, "parent_namespace": "tb3_0",
                "parent_position": positions["tb3_0"],
                "position": np.array([-0.145, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = (
            lambda position, _yaw, _obj_pos, _obj_yaw, target,
            **_kwargs: np.linalg.norm(
                position - target["position"]
            ) <= 0.015
        )
        controller._slot_velocity = (
            lambda *_args, **_kwargs:
            (ROS["transport"].Vec2(0.10, 0.0), True)
        )
        def to_twist(vx, _vy, _yaw):
            command = Twist()
            command.linear.x = vx
            return command
        controller._holonomic_to_diff_drive = to_twist
        avoidance_calls = {}
        avoidance_limits = {}
        def apply_transport_avoidance(
            namespace, command, *_args, **kwargs
        ):
            avoidance_calls[namespace] = kwargs
            if namespace in avoidance_limits:
                command.linear.x = min(
                    command.linear.x, avoidance_limits[namespace]
                )
            return command
        controller._apply_transport_avoidance = apply_transport_avoidance
        payload_contact_margins = []
        def payload_contact_near(position, *_args, **kwargs):
            payload_contact_margins.append(kwargs.get("margin"))
            return abs(float(position[0])) <= 0.015
        controller._payload_contact_near = payload_contact_near
        controller._robot_lidar_masks = lambda *_args: ()
        published = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published.__setitem__(namespace, command) or True
        )
        stops = []
        controller._stop_all_robots = lambda: stops.append(True)
        controller.robot_velocities = {
            namespace: np.array([0.02, 0.0])
            for namespace in positions
        }

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertTrue(waiting)
        self.assertEqual(
            controller.transport_contact_slop,
            payload_contact_margins[0],
        )
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        self.assertFalse(controller.transport_queue_docking_started)
        self.assertEqual(
            10.0, controller.transport_queue_settle_started_at
        )

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.4,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        self.assertFalse(controller.transport_queue_docking_started)
        self.assertEqual(
            10.0, controller.transport_queue_settle_started_at
        )

        controller.robot_velocities = {
            namespace: np.array([0.003, 0.0])
            for namespace in positions
        }
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.5,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertAlmostEqual(0.012, published["tb3_0"].linear.x)
        self.assertAlmostEqual(0.012, published["tb3_1"].linear.x)
        self.assertTrue(controller.transport_queue_docking_started)
        self.assertIsNone(controller.transport_queue_settle_started_at)
        self.assertTrue(avoidance_calls["tb3_0"]["payload_docking"])
        self.assertNotIn("payload_docking", avoidance_calls["tb3_1"])

        avoidance_limits["tb3_1"] = 0.006
        controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )
        self.assertAlmostEqual(0.006, published["tb3_0"].linear.x)
        self.assertAlmostEqual(0.006, published["tb3_1"].linear.x)

        avoidance_limits["tb3_1"] = -0.006
        controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        avoidance_limits["tb3_1"] = 0.006

        # If a lead reaches the payload while one queue link opens, keep the
        # verified prefix braked and repair that link in place. Re-parking the
        # lead would reverse it into its child and deadlock the hard robot
        # clearance guard.
        positions["tb3_0"] = np.array([0.0, 0.0])
        positions["tb3_1"] = np.array([-0.245, 0.0])
        waiting = controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )
        self.assertTrue(waiting)
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertGreater(published["tb3_1"].linear.x, 0.0)
        self.assertEqual(
            ROS["transport"].TransportPhase.PUSH,
            controller.phase,
        )
        self.assertTrue(controller.transport_queue_docking_started)

    def test_docking_repairs_an_internal_link_without_reparking(self):
        (
            controller,
            positions,
            yaws,
            raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()
        positions["tb3_1"] = np.array([-0.16, 0.0])
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_depth": 0,
                "chain_index": 0,
                "max_chain_depth": 1,
                "parent_namespace": None,
                "position": np.array([0.10, 0.0]),
                "staging_position": np.array([0.0, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "role": "companion_push",
                "chain_depth": 1,
                "chain_index": 0,
                "max_chain_depth": 1,
                "parent_namespace": "tb3_0",
                "parent_position": positions["tb3_0"],
                "position": np.array([-0.045, 0.0]),
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        controller._transport_targets = lambda *_args: targets
        controller._transport_robot_lidar_masks = lambda *_args: ()
        controller.transport_queue_docking_started = True
        controller.transport_queue_settle_started_at = None
        controller.transport_pre_staged = {"tb3_1"}
        controller.transport_staged = set(positions)
        controller.transport_chain_released = {"tb3_1"}
        raw_contact["tb3_0"] = False
        raw_contact["tb3_1"] = False

        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertEqual(
            ROS["transport"].TransportPhase.PUSH,
            controller.phase,
        )
        self.assertTrue(controller.transport_queue_docking_started)
        self.assertEqual({"tb3_1"}, controller.transport_pre_staged)
        self.assertEqual({"tb3_1"}, controller.transport_chain_released)
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertGreater(published["tb3_1"].linear.x, 0.0)

        # Once the local repair closes, the complete lane resumes docking as
        # one speed-matched unit.
        raw_contact["tb3_1"] = True
        controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )
        self.assertGreater(published["tb3_0"].linear.x, 0.0)
        self.assertGreater(published["tb3_1"].linear.x, 0.0)

    def test_complete_queues_with_unsafe_rows_return_to_staging(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            _published,
        ) = self._transport_chain_engagement_controller()
        controller._parallel_push_rows_ready = lambda *_args: False
        return_to_staging = mock.Mock(return_value=True)
        controller._return_transport_queues_to_staging = return_to_staging

        waiting = controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )

        self.assertTrue(waiting)
        return_to_staging.assert_called_once()
        self.assertIn(
            "lost their safe clearance",
            return_to_staging.call_args.args[-1],
        )

    def test_loaded_queues_with_unsafe_rows_fail_instead_of_deadlocking(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            _published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_queue_docking_started = True
        controller._parallel_push_rows_ready = lambda *_args: False
        fail_transport = mock.Mock(return_value=True)
        controller._fail_transport = fail_transport

        waiting = controller._engage_push_chains(
            list(positions), positions, yaws, np.zeros(2), 0.0, 4
        )

        self.assertTrue(waiting)
        fail_transport.assert_called_once()
        self.assertIn(
            "while a lead robot was loaded",
            fail_transport.call_args.args[0],
        )

    def test_connected_chain_links_do_not_reverse_before_launch(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertEqual({"tb3_0", "tb3_1"}, controller.transport_engaged)
        self.assertFalse(controller.transport_engagement_complete)

        # Let both connected robots drift slightly ahead of the contact
        # poses captured on the previous cycle. They may brake in place, but
        # reversing either link would compress the robot behind it.
        positions["tb3_0"] = np.array([0.01, 0.0])
        positions["tb3_1"] = np.array([-0.125, 0.0])
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.1,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertGreaterEqual(published["tb3_0"].linear.x, 0.0)
        self.assertGreaterEqual(published["tb3_1"].linear.x, 0.0)

    def test_connected_companion_holds_zero_before_launch(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()
        positions["tb3_1"] = np.array([-0.144, 0.0])

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertEqual({"tb3_0", "tb3_1"}, controller.transport_engaged)
        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertEqual(0.0, published["tb3_1"].linear.x)

    def test_chain_launch_hold_requires_continuous_raw_contact(self):
        (
            controller,
            positions,
            yaws,
            raw_contact,
            _published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_engagement_hold_time = 0.20

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertFalse(controller.transport_engagement_complete)

        # This sample is inside release grace, so the companion can remain in
        # the connected prefix. It must still break the launch hold because
        # no raw contact was measured on this cycle.
        raw_contact["tb3_1"] = False
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.25,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertEqual({"tb3_0", "tb3_1"}, controller.transport_engaged)
        self.assertFalse(controller.transport_engagement_complete)

        raw_contact["tb3_1"] = True
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.26,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertFalse(controller.transport_engagement_complete)

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.47,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertTrue(controller.transport_engagement_complete)

    def test_transport_push_speed_ramps_after_chain_engagement(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_push_ramp_started_at = 20.0
        controller.transport_push_ramp_rate = 0.01
        controller.transport_push_ramp_initial_speed = 0.015
        controller.transport_min_useful_push_speed = 0.035

        self.assertAlmostEqual(
            0.015, controller._ramped_push_speed(0.14, now=20.0)
        )
        self.assertAlmostEqual(
            0.035, controller._ramped_push_speed(0.14, now=22.0)
        )
        self.assertAlmostEqual(
            0.05, controller._ramped_push_speed(0.05, now=30.0)
        )

        controller.transport_push_ramp_started_at = None
        self.assertAlmostEqual(
            0.14, controller._ramped_push_speed(0.14, now=20.0)
        )

    def test_companion_engagement_uses_contact_then_release_hysteresis(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_companion_engagement_angle = 0.35
        controller.transport_engagement_release_angle_margin = 0.30
        controller.transport_alignment_retreat_clearance = 0.055
        parent = np.array([0.0, 0.0])
        push_direction = np.array([1.0, 0.0])

        ready, destination = controller._companion_engagement_geometry(
            np.array([-0.144, 0.0]), parent, push_direction
        )
        self.assertTrue(ready)
        np.testing.assert_allclose(destination, [-0.140, 0.0])

        ready, destination = controller._companion_engagement_geometry(
            np.array([-0.155, 0.0]), parent, push_direction
        )
        self.assertFalse(ready)
        np.testing.assert_allclose(destination, [-0.135, 0.0])

        ready, destination = controller._companion_engagement_geometry(
            np.array([-0.1454, 0.0]), parent, push_direction
        )
        self.assertFalse(ready)
        np.testing.assert_allclose(destination, [-0.1254, 0.0])

        ready, _ = controller._companion_engagement_geometry(
            np.array([-0.155, 0.0]), parent, push_direction,
            release=True,
        )
        self.assertTrue(ready)

        angled_contact = np.array([-0.135, 0.06])
        ready, _ = controller._companion_engagement_geometry(
            angled_contact, parent, push_direction
        )
        self.assertFalse(ready)
        ready, destination = controller._companion_engagement_geometry(
            angled_contact, parent, push_direction,
            allow_alignment_retreat=True,
        )
        self.assertFalse(ready)
        np.testing.assert_allclose(destination, [-0.20, 0.0])
        ready, _ = controller._companion_engagement_geometry(
            angled_contact, parent, push_direction, release=True
        )
        self.assertTrue(ready)

        ready, _ = controller._companion_engagement_geometry(
            np.array([-0.115, 0.097]), parent, push_direction,
            release=True,
        )
        self.assertFalse(ready)

    def test_transport_contact_release_grace_is_brief_and_correlated(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_engagement_release_hold_time = 0.35
        controller.transport_engagement_last_ready = {}

        self.assertTrue(controller._engagement_contact_with_grace(
            "tb3_0", True, set(), 10.0
        ))
        self.assertTrue(controller._engagement_contact_with_grace(
            "tb3_0", False, {"tb3_0"}, 10.30
        ))
        self.assertFalse(controller._engagement_contact_with_grace(
            "tb3_0", False, {"tb3_0"}, 10.36
        ))
        self.assertFalse(controller._engagement_contact_with_grace(
            "tb3_1", False, {"tb3_0"}, 10.10
        ))

    def test_transport_parallel_lane_contacts_need_measured_clearance(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.robot_radius = 0.11
        targets = {
            "tb3_0": {
                "chain_index": 0,
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "chain_index": 1,
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_2": {
                "chain_index": 1,
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([0.0, 0.34]),
            "tb3_2": np.array([0.30, 0.27]),
        }

        neighbours = controller._parallel_lane_contacts(
            "tb3_0", positions, targets
        )

        self.assertEqual(("tb3_1",), neighbours)

    def test_transport_launch_requires_parallel_row_clearance(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.robot_radius = 0.11
        targets = {
            "tb3_0": {
                "chain_index": 0,
                "chain_depth": 0,
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "chain_index": 1,
                "chain_depth": 0,
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([0.0, 0.249]),
        }

        self.assertFalse(controller._parallel_push_rows_ready(
            positions, targets
        ))
        positions["tb3_1"] = np.array([0.0, 0.25])
        self.assertTrue(controller._parallel_push_rows_ready(
            positions, targets
        ))

        # Launch safety uses the physical row gap. The wider 0.29 m rule is
        # deliberately retained only for bypassing repulsion and LiDAR masks.
        positions["tb3_1"] = np.array([0.0, 0.28])
        self.assertTrue(controller._parallel_push_rows_ready(
            positions, targets
        ))
        self.assertNotIn(
            "tb3_1",
            controller._parallel_lane_contacts(
                "tb3_0", positions, targets
            ),
        )

    def test_transport_publish_does_not_mask_a_collapsed_parallel_row(self):
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([0.0, 0.27]),
        }
        targets = {
            namespace: {
                "role": "payload_push",
                "chain_index": index,
                "chain_depth": 0,
                "parent_namespace": None,
                "parent_position": np.zeros(2),
                "position": position.copy(),
                "push_direction": np.array([1.0, 0.0]),
            }
            for index, (namespace, position) in enumerate(positions.items())
        }
        controller, _ = self._transport_publish_controller(
            targets,
            [(position, True) for position in positions.values()],
        )
        avoidance_calls = {}
        controller._robot_lidar_masks = (
            lambda namespaces, *_args: tuple(namespaces)
        )

        def capture_avoidance(namespace, command, *_args, **kwargs):
            avoidance_calls[namespace] = kwargs
            return command

        controller._apply_transport_avoidance = capture_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.08, 0.0)
            for namespace in positions
        }
        controller._publish_grf_commands(
            list(positions), commands, positions,
            {namespace: 0.0 for namespace in positions},
            np.zeros(2), 0.0, 4,
        )

        for namespace, other_namespace in (
            ("tb3_0", "tb3_1"), ("tb3_1", "tb3_0")
        ):
            call = avoidance_calls[namespace]
            self.assertNotIn(
                other_namespace,
                call["parallel_motion_exempt_namespaces"],
            )
            self.assertNotIn(
                other_namespace,
                call["repulsion_exempt_namespaces"],
            )
            self.assertNotIn(
                other_namespace, call["additional_lidar_masks"]
            )

    def test_transport_payload_row_gap_caps_the_whole_fleet(self):
        positions = {
            "tb3_0": np.array([0.0, 0.0]),
            "tb3_1": np.array([0.0, 0.30]),
            "tb3_2": np.array([-0.145, 0.0]),
        }
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_index": 0,
                "chain_depth": 0,
                "parent_namespace": None,
                "parent_position": np.zeros(2),
                "position": positions["tb3_0"].copy(),
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_1": {
                "role": "payload_push",
                "chain_index": 1,
                "chain_depth": 0,
                "parent_namespace": None,
                "parent_position": np.zeros(2),
                "position": positions["tb3_1"].copy(),
                "push_direction": np.array([1.0, 0.0]),
            },
            "tb3_2": {
                "role": "companion_push",
                "chain_index": 0,
                "chain_depth": 1,
                "parent_namespace": "tb3_0",
                "parent_position": positions["tb3_0"].copy(),
                "position": positions["tb3_2"].copy(),
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        controller, published = self._transport_publish_controller(
            targets,
            [
                (positions["tb3_0"], True),
                (positions["tb3_1"], False),
            ],
        )
        commands = {
            namespace: ROS["transport"].Vec2(0.08, 0.0)
            for namespace in positions
        }

        controller._publish_grf_commands(
            list(positions), commands, positions,
            {namespace: 0.0 for namespace in positions},
            np.zeros(2), 0.0, 4,
        )

        self.assertEqual(set(positions), set(published))
        for command in published.values():
            self.assertGreater(command.linear.x, 0.0)
            self.assertLessEqual(command.linear.x, 0.018 + 1e-9)

    def test_transport_single_pusher_is_not_subject_to_paired_row_cap(self):
        positions = {"tb3_0": np.array([0.0, 0.0])}
        targets = {
            "tb3_0": {
                "role": "payload_push",
                "chain_index": 0,
                "chain_depth": 0,
                "parent_namespace": None,
                "parent_position": np.zeros(2),
                "position": positions["tb3_0"].copy(),
                "push_direction": np.array([1.0, 0.0]),
            },
        }
        # A solo pusher may be recovering its payload surface, but there is no
        # opposite lead to wait for. The paired-row symmetry gate must not
        # reduce its otherwise valid recovery command.
        controller, published = self._transport_publish_controller(
            targets, [(positions["tb3_0"], False)]
        )

        controller._publish_grf_commands(
            ["tb3_0"],
            {"tb3_0": ROS["transport"].Vec2(0.08, 0.0)},
            positions,
            {"tb3_0": 0.0},
            np.zeros(2), 0.0, 4,
        )

        self.assertAlmostEqual(0.08, published["tb3_0"].linear.x)
        self.assertGreater(
            published["tb3_0"].linear.x,
            controller.transport_contact_closing_speed,
        )

    def test_chain_pose_hold_aligns_before_translating(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )

        turning = controller._chain_pose_hold_command(
            np.array([0.0, 0.0]),
            1.0,
            np.array([0.20, 0.0]),
            np.array([1.0, 0.0]),
        )
        aligned = controller._chain_pose_hold_command(
            np.array([0.0, 0.0]),
            0.0,
            np.array([0.20, 0.0]),
            np.array([1.0, 0.0]),
        )

        self.assertEqual(0.0, turning.linear.x)
        self.assertLess(turning.angular.z, 0.0)
        self.assertGreater(aligned.linear.x, 0.0)

    def test_companion_assembly_straightens_after_route_handoff(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_engagement_angle = 0.20
        controller.transport_assembly_closing_speed = 0.012
        controller.transport_assembly_handoff_tolerance = 0.015
        controller.transport_assembly_heading_tolerance = 0.10
        controller.transport_launch_heading_tolerance = 0.08
        controller.transport_chain_staging_speed = 0.16
        position = np.array([-0.591, -0.938])
        target = {
            "role": "companion_push",
            "position": np.array([-0.620, -0.967]),
            "assembly_position": np.array([-0.620, -0.967]),
            "parent_position": np.array([-0.613, -1.108]),
            "push_direction": np.array([0.0, -1.0]),
        }

        turning = controller._companion_assembly_command(
            position, -2.078, target
        )
        closing = controller._companion_assembly_command(
            np.array([-0.620, -0.938]), -math.pi / 2.0, target
        )

        # This is the pose that orbit-locked in the visible N=10 run. The
        # final contact controller must finish the turn without translating.
        self.assertEqual(0.0, turning.linear.x)
        self.assertGreater(turning.angular.z, 0.0)
        self.assertGreater(closing.linear.x, 0.0)
        self.assertLessEqual(
            closing.linear.x,
            controller.transport_assembly_closing_speed,
        )

        invalid_target = dict(
            target, push_direction=np.array([float("nan"), 0.0])
        )
        invalid = controller._companion_assembly_command(
            position, 0.0, invalid_target
        )
        self.assertEqual(0.0, invalid.linear.x)
        self.assertEqual(0.0, invalid.angular.z)

    def test_companion_assembly_converges_from_run18_release_pose(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_companion_engagement_angle = 0.20
        controller.transport_assembly_closing_speed = 0.012
        controller.transport_assembly_handoff_tolerance = 0.015
        controller.transport_assembly_heading_tolerance = 0.10
        controller.transport_launch_heading_tolerance = 0.08
        controller.transport_chain_staging_speed = 0.16
        controller.transport_chain_staging_tolerance = 0.025
        controller.transport_route_parent_clearance = 0.20
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        position = np.array([-0.4234, -0.7425])
        yaw = -2.45
        parent = np.array([-0.620, -1.107])
        target = {
            "role": "companion_push",
            "position": np.array([-0.620, -0.967]),
            "assembly_position": np.array([-0.620, -0.967]),
            "parent_position": parent,
            "push_direction": np.array([0.0, -1.0]),
        }

        ready = False
        for _ in range(800):
            command = controller._companion_assembly_command(
                position, yaw, target
            )
            step = 0.02
            position += np.array([
                math.cos(yaw), math.sin(yaw)
            ]) * command.linear.x * step
            yaw = controller._normalize_angle(
                yaw + command.angular.z * step
            )
            to_parent = parent - position
            gap = float(np.linalg.norm(to_parent))
            lateral_error = abs(float(to_parent[0]))
            heading_error = abs(controller._normalize_angle(
                -math.pi / 2.0 - yaw
            ))
            ready = (
                0.135 <= gap <= 0.145
                and lateral_error <= 0.015
                and heading_error <= 0.10
            )
            if ready:
                break

        self.assertTrue(ready)

    def test_transport_stall_reassigns_contact_roles(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.transport_best_distance = 0.60
        controller.transport_last_progress_time = 10.0
        controller.transport_progress_delta = 0.03
        controller.transport_progress_timeout = 12.0
        controller.transport_roles = {
            "tb3_0": {
                "role": "payload_push", "chain_index": 0, "depth": 0,
            },
            "tb3_1": {
                "role": "companion_push", "chain_index": 0, "depth": 1,
            },
        }
        controller.transport_pre_staged = {"tb3_1"}
        controller.transport_chain_released = {"tb3_1"}
        phases = []
        controller._set_phase = lambda phase, _epoch: phases.append(phase) or True
        controller._stop_all_robots = lambda: {
            'confirmed': True,
            'failed_robots': [],
        }

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=23.0,
            create=True,
        ):
            recovered = controller._reposition_if_stalled(0.60, 8)

        self.assertTrue(recovered)
        self.assertEqual({}, controller.transport_roles)
        self.assertEqual(set(), controller.transport_pre_staged)
        self.assertEqual(set(), controller.transport_chain_released)
        self.assertEqual(
            [ROS["transport"].TransportPhase.APPROACH], phases
        )

    def test_transport_push_phase_starts_a_fresh_progress_window(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.phase_lock = threading.RLock()
        controller.command_epoch = 8
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.transport_best_distance = 0.60
        controller.transport_last_progress_time = 10.0
        controller.phase = ROS["transport"].TransportPhase.APPROACH

        changed = controller._set_phase(
            ROS["transport"].TransportPhase.PUSH, 8
        )

        self.assertTrue(changed)
        self.assertEqual(
            ROS["transport"].TransportPhase.PUSH, controller.phase
        )
        self.assertIsNone(controller.transport_best_distance)
        self.assertIsNone(controller.transport_last_progress_time)

    def test_transport_drift_does_not_finish_before_a_synchronized_push(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.obstacle_positions = []
        controller.target_x = 0.20
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.robot_namespaces = ["tb3_0"]
        controller.robot_positions = {"tb3_0": np.array([-0.24, 0.0])}
        controller.robot_yaws = {"tb3_0": 0.0}
        controller.transport_best_distance = None
        controller.transport_last_progress_time = None
        controller.transport_near_target_recovery_started_at = None
        controller.transport_arrival_latched = False
        controller.transport_arrival_direction = None
        controller.transport_progress_timeout = 10.0
        controller.transport_synchronized_push_started = False
        controller.transport_planner = "legacy"

        engagement_waiting = [True]
        controller._engage_push_chains = (
            lambda *_args: engagement_waiting[0]
        )
        controller._reposition_if_stalled = lambda *_args: False
        legacy_calls = []
        def publish_legacy_batch(*_args, **_kwargs):
            legacy_calls.append(True)
            controller.transport_synchronized_push_started = True

        controller._legacy_push_step = publish_legacy_batch
        completions = []
        controller._complete_transport = (
            lambda distance, _epoch: completions.append(distance) or True
        )

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._push_phase(4)
        self.assertEqual([], completions)
        self.assertEqual([], legacy_calls)
        self.assertIsNone(controller.transport_best_distance)
        self.assertIsNone(controller.transport_last_progress_time)

        controller.transport_engagement_complete = True
        controller.transport_best_distance = 1.0
        controller.transport_last_progress_time = 10.0
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=154.0,
            create=True,
        ):
            controller._push_phase(4)
        self.assertAlmostEqual(0.20, controller.transport_best_distance)
        self.assertEqual(154.0, controller.transport_last_progress_time)

        engagement_waiting[0] = False
        controller._push_phase(4)
        self.assertEqual([], completions)
        self.assertEqual([True], legacy_calls)
        self.assertTrue(controller.transport_synchronized_push_started)

        controller.transport_engaged = {"tb3_0"}
        controller.transport_physical_engaged = {"tb3_0"}
        controller.transport_all_pushers_confirmed = True
        controller.transport_current_useful_pushers = {"tb3_0"}
        controller._push_phase(4)
        self.assertEqual([0.20], completions)

    def test_large_fleet_uses_the_normal_delivery_tolerance(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([0.60, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.obstacle_positions = []
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.robot_namespaces = [
            "tb3_{}".format(index) for index in range(6)
        ]
        controller.robot_positions = {
            namespace: np.array([-0.20 - 0.15 * index, 0.0])
            for index, namespace in enumerate(controller.robot_namespaces)
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        # Keep the old shortcut's state populated so this test would fail if
        # projected-distance completion is accidentally restored.
        controller.transport_object_start = np.array([0.0, 0.0])
        controller.transport_large_fleet_progress_goal = 0.555
        controller.transport_best_distance = 1.40
        controller.transport_last_progress_time = 10.0
        controller.transport_near_target_recovery_started_at = None
        controller.transport_arrival_latched = False
        controller.transport_arrival_direction = None
        controller.transport_progress_timeout = 10.0
        controller.transport_synchronized_push_started = True
        controller.transport_engagement_complete = True
        controller.transport_engaged = set(controller.robot_namespaces)
        controller.transport_physical_engaged = set(
            controller.robot_namespaces
        )
        controller.transport_all_pushers_confirmed = True
        controller.transport_current_useful_pushers = set(
            controller.robot_namespaces
        )
        controller.transport_planner = "legacy"
        controller._engage_push_chains = lambda *_args: False
        controller._reposition_if_stalled = lambda *_args: False
        legacy_calls = []
        controller._legacy_push_step = (
            lambda *_args, **_kwargs: legacy_calls.append(True)
        )
        completions = []
        controller._complete_transport = (
            lambda distance, _epoch: completions.append(distance) or True
        )

        # Moving sixty centimetres is useful progress, but it is not a
        # delivery while the payload still sits well outside the requested
        # target tolerance.
        controller._push_phase(4)
        self.assertEqual([], completions)
        self.assertEqual([True], legacy_calls)

        controller.object_position = np.array([1.55, 0.0])
        controller._push_phase(4)
        self.assertEqual(1, len(completions))
        self.assertAlmostEqual(0.45, completions[0])
        self.assertEqual([True], legacy_calls)

    def test_follow_leader_stale_stop_and_empty_fleet_do_not_resume(self):
        controller = ROS["follow"].FollowTheLeader.__new__(
            ROS["follow"].FollowTheLeader
        )
        controller.command_lock = threading.RLock()
        controller.lock = threading.RLock()
        controller.current_task_id = "task-new"
        controller.is_active = True
        controller.is_paused = False
        controller.robot_names = ["tb3_0"]
        controller.poses = {"tb3_0": Pose()}
        controller.yaws = {"tb3_0": 0.0}
        controller.linear_pids = {"tb3_0": object()}
        controller.angular_pids = {"tb3_0": object()}
        controller.cmd_pubs = {"tb3_0": FakePublisher()}
        controller.odom_subs = {"tb3_0": FakeSubscriber()}
        controller.avoidance = {"tb3_0": FakeAvoidance()}
        controller.leader_trace = FakeTrace()

        controller._stop_cb(lifecycle_payload("task-old"))
        self.assertTrue(controller.is_active)

        controller._stop_cb(lifecycle_payload("task-new"))
        self.assertFalse(controller.is_active)
        controller.is_active = True

        old_pub = controller.cmd_pubs["tb3_0"]
        controller._sync_fleet([])
        self.assertFalse(controller.is_active)
        self.assertEqual([], controller.robot_names)
        self.assertTrue(old_pub.unregistered)
        self.assertTrue(old_pub.messages)

    def test_transport_pause_resume_preserves_completed_compression(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.data_lock = threading.Lock()
        controller.model_lock = threading.Lock()
        controller.phase_lock = threading.Lock()
        controller.current_task_id = "transport-pause"
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.phase = ROS["transport"].TransportPhase.APPROACH
        controller.object_position = np.zeros(2)
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.transport_ready_hold_time = 0.0
        controller.transport_all_ready_since = None

        controller.robot_namespaces = [
            "tb3_0", "tb3_1", "tb3_2", "tb3_3"
        ]
        controller.robot_positions = {
            "tb3_0": np.array([-0.49, -0.18]),
            "tb3_1": np.array([-0.49, 0.18]),
            "tb3_2": np.array([-0.63, -0.18]),
            "tb3_3": np.array([-0.63, 0.18]),
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.robot_velocities = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_odom_received_at = {
            namespace: 1.0 for namespace in controller.robot_namespaces
        }
        controller.cmd_vel_pubs = {
            namespace: FakePublisher()
            for namespace in controller.robot_namespaces
        }
        controller.avoidance_modules = {
            namespace: FakeAvoidance()
            for namespace in controller.robot_namespaces
        }
        controller.transport_last_commands = {
            namespace: (0.08, 0.2)
            for namespace in controller.robot_namespaces
        }

        push_direction = np.array([1.0, 0.0])
        targets = {
            "tb3_0": {
                "role": "payload_push", "chain_index": 0,
                "chain_depth": 0,
                "position": np.array([-0.24, -0.18]),
                "staging_position": np.array([-0.49, -0.18]),
                "push_direction": push_direction.copy(),
                "parent_namespace": None,
                "parent_position": np.zeros(2),
            },
            "tb3_1": {
                "role": "payload_push", "chain_index": 1,
                "chain_depth": 0,
                "position": np.array([-0.24, 0.18]),
                "staging_position": np.array([-0.49, 0.18]),
                "push_direction": push_direction.copy(),
                "parent_namespace": None,
                "parent_position": np.zeros(2),
            },
            "tb3_2": {
                "role": "companion_push", "chain_index": 0,
                "chain_depth": 1,
                "position": np.array([-0.63, -0.18]),
                "assembly_position": np.array([-0.63, -0.18]),
                "staging_position": np.array([-0.87, -0.52]),
                "push_direction": push_direction.copy(),
                "parent_namespace": "tb3_0",
                "parent_position": controller.robot_positions["tb3_0"],
            },
            "tb3_3": {
                "role": "companion_push", "chain_index": 1,
                "chain_depth": 1,
                "position": np.array([-0.63, 0.18]),
                "assembly_position": np.array([-0.63, 0.18]),
                "staging_position": np.array([-0.87, 0.52]),
                "push_direction": push_direction.copy(),
                "parent_namespace": "tb3_1",
                "parent_position": controller.robot_positions["tb3_1"],
            },
        }
        all_namespaces = set(controller.robot_namespaces)
        companions = {"tb3_2", "tb3_3"}
        controller.transport_roles = {
            namespace: {
                "role": target["role"],
                "chain_index": target["chain_index"],
                "depth": target["chain_depth"],
            }
            for namespace, target in targets.items()
        }
        controller.transport_pre_staged = set(all_namespaces)
        controller.transport_staged = set(all_namespaces)
        controller.transport_chain_released = set(companions)
        controller.transport_compression_progress = 1.0
        controller.transport_compression_updated_at = 12.0

        command = lifecycle_payload("transport-pause")
        controller._pause_callback(command)
        self.assertTrue(controller.is_paused)
        self.assertEqual(5, controller.command_epoch)
        self.assertEqual(1.0, controller.transport_compression_progress)
        self.assertEqual(all_namespaces, controller.transport_pre_staged)
        self.assertEqual(all_namespaces, controller.transport_staged)
        self.assertEqual(companions, controller.transport_chain_released)

        controller._resume_callback(command)
        self.assertFalse(controller.is_paused)
        self.assertEqual(6, controller.command_epoch)
        self.assertEqual(1.0, controller.transport_compression_progress)
        self.assertEqual(all_namespaces, controller.transport_pre_staged)
        self.assertEqual(all_namespaces, controller.transport_staged)
        self.assertEqual(companions, controller.transport_chain_released)

        controller._transport_odometry_error = lambda *_args: None
        controller._transport_target_error = lambda: None
        controller._transport_layout_error = lambda *_args, **_kwargs: None
        controller._transport_targets = lambda *_args: targets
        controller._slot_contact_ready = lambda *_args, **_kwargs: True
        controller._parallel_push_rows_ready = lambda *_args: True
        published = set()
        controller._publish_concurrent_approach_command = (
            lambda namespace, *_args, **_kwargs: published.add(namespace)
        )

        controller._approach_phase(6)

        self.assertEqual(all_namespaces, published)
        self.assertEqual(
            ROS["transport"].TransportPhase.PUSH,
            controller.phase,
        )

    def test_transport_status_progress_uses_task_baseline_and_never_regresses(self):
        controller = self._transport_search_controller(3)
        controller.object_position = np.array([1.0, 1.0])
        controller.arrival_tolerance = 0.5
        controller.transport_planner = "grf"
        controller.grf_mcmc_iterations = 60
        controller.grf_large_fleet_iterations = 12
        controller._grf_kernels = {}
        controller.status_pub = FakePublisher()

        controller._start_callback(String(data=json.dumps({
            "task_id": "progress-task",
            "target_x": 4.0,
            "target_y": 5.0,
            "transport_planner": "grf",
        })))

        self.assertAlmostEqual(
            5.0, controller.transport_initial_target_distance
        )
        self.assertEqual(0.0, controller.transport_reported_progress)

        def publish_at(position):
            with controller.model_lock:
                controller.object_position = np.array(position, dtype=float)
            controller._publish_status(
                ROS["transport"].TransportPhase.PUSH
            )
            return json.loads(controller.status_pub.messages[-1].data)

        self.assertEqual(0.0, publish_at([1.0, 1.0])["progress"])
        self.assertEqual(0.5, publish_at([1.25, 5.0])["progress"])
        self.assertEqual(0.5, publish_at([0.5, 5.0])["progress"])
        self.assertEqual(1.0, publish_at([3.5, 5.0])["progress"])

        controller._stop_callback(lifecycle_payload("progress-task"))
        self.assertIsNone(controller.transport_initial_target_distance)
        self.assertEqual(0.0, controller.transport_reported_progress)

    def test_stop_before_start_prevents_late_transport_activation(self):
        controller = self._transport_search_controller(2)
        cancelled_task_id = 'cancelled-transport-task'

        controller._stop_callback(lifecycle_payload(cancelled_task_id))
        controller.is_running = False
        controller._start_callback(String(data=json.dumps({
            'task_id': cancelled_task_id,
            'target_x': 1.0,
            'target_y': 2.0,
        })))

        self.assertIn(cancelled_task_id, controller._cancelled_task_ids)
        self.assertFalse(controller.is_running)
        self.assertEqual('search-task', controller.current_task_id)

    def test_transport_start_moves_and_observes_the_gazebo_target_ghost(self):
        controller = self._transport_search_controller(2)
        controller.object_position = np.array([1.0, 1.0])
        controller.default_arrival_tolerance = 0.5
        controller.arrival_tolerance = 0.5
        controller.transport_planner = "grf"
        controller.grf_mcmc_iterations = 60
        controller.grf_large_fleet_iterations = 12
        controller._grf_kernels = {}
        controller.target_marker_name = "target_marker"
        controller.target_marker_position = None
        controller.target_marker_synced = False
        controller.target_marker_command_published = False
        controller.target_marker_sync_tolerance = 0.02
        controller.target_marker_pub = FakePublisher()

        controller._start_callback(String(data=json.dumps({
            "task_id": "ghost-task",
            "target_x": -2.5,
            "target_y": 1.25,
            "arrival_tolerance": 0.25,
            "transport_planner": "grf",
        })))

        self.assertEqual(0.25, controller.arrival_tolerance)
        self.assertTrue(controller.target_marker_command_published)
        self.assertFalse(controller.target_marker_synced)
        marker_command = controller.target_marker_pub.messages[-1]
        self.assertEqual("target_marker", marker_command.model_name)
        self.assertEqual("world", marker_command.reference_frame)
        self.assertEqual(-2.5, marker_command.pose.position.x)
        self.assertEqual(1.25, marker_command.pose.position.y)
        self.assertEqual(0.0, marker_command.pose.position.z)
        self.assertEqual(1.0, marker_command.pose.orientation.w)

        payload_pose = Pose()
        payload_pose.position.x = 1.0
        payload_pose.position.y = 1.0
        payload_pose.position.z = 0.1
        marker_pose = Pose()
        marker_pose.position.x = -2.5
        marker_pose.position.y = 1.25
        models = ModelStates()
        models.name = ["transport_object", "target_marker"]
        models.pose = [payload_pose, marker_pose]
        controller.object_rest_z = 0.1
        controller.object_z_tolerance = 0.05
        controller._model_states_callback(models)

        self.assertTrue(controller.target_marker_synced)
        np.testing.assert_allclose(
            [-2.5, 1.25], controller.target_marker_position
        )
        controller.status_pub = FakePublisher()
        controller._publish_status(
            ROS["transport"].TransportPhase.SEARCH
        )
        status = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(0.25, status["arrival_tolerance"])
        self.assertEqual({
            "model_name": "target_marker",
            "command_published": True,
            "synchronized": True,
            "position": {"x": -2.5, "y": 1.25},
        }, status["target_marker"])

    def test_transport_continues_when_the_target_ghost_cannot_be_published(self):
        class FailingPublisher:
            def publish(self, _message):
                raise RuntimeError("publisher unavailable")

        controller = self._transport_search_controller(1)
        controller.object_position = np.array([1.0, 1.0])
        controller.default_arrival_tolerance = 0.5
        controller.arrival_tolerance = 0.5
        controller.transport_planner = "grf"
        controller.grf_mcmc_iterations = 60
        controller.grf_large_fleet_iterations = 12
        controller._grf_kernels = {}
        controller.target_marker_position = None
        controller.target_marker_synced = False
        controller.target_marker_command_published = False
        controller.target_marker_pub = FailingPublisher()

        controller._start_callback(String(data=json.dumps({
            "task_id": "ghost-rejected-task",
            "target_x": 2.0,
            "target_y": 2.0,
        })))

        self.assertTrue(controller.is_running)
        self.assertEqual(
            ROS["transport"].TransportPhase.SEARCH,
            controller.phase,
        )
        self.assertFalse(controller.target_marker_command_published)
        self.assertFalse(controller.target_marker_synced)

    def test_transport_without_arrival_override_restores_its_default(self):
        controller = self._transport_search_controller(1)
        controller.object_position = np.array([1.0, 1.0])
        controller.default_arrival_tolerance = 0.5
        controller.arrival_tolerance = 0.25
        controller.transport_planner = "grf"
        controller.grf_mcmc_iterations = 60
        controller.grf_large_fleet_iterations = 12
        controller._grf_kernels = {}

        controller._start_callback(String(data=json.dumps({
            "task_id": "default-margin-task",
            "target_x": 2.0,
            "target_y": 2.0,
        })))

        self.assertEqual(0.5, controller.arrival_tolerance)

    def test_transport_progress_starts_complete_inside_arrival_tolerance(self):
        controller = self._transport_search_controller(1)
        controller.object_position = np.array([1.75, -0.1])
        controller.target_x = 2.0
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.5

        controller._begin_transport_progress()

        self.assertAlmostEqual(
            math.hypot(0.25, 0.1),
            controller.transport_initial_target_distance,
        )
        self.assertEqual(1.0, controller.transport_reported_progress)

    def test_transport_stop_zeroes_status_snapshot_and_resets_avoidance(self):
        controller = self._transport_search_controller(3)
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 0
        controller.transport_roles = {}
        controller.status_pub = FakePublisher()
        controller.transport_last_commands = {
            namespace: (0.12, -0.35)
            for namespace in controller.robot_namespaces
        }

        controller._stop_all_robots()
        controller._publish_status(
            ROS["transport"].TransportPhase.SEARCH
        )

        status = json.loads(controller.status_pub.messages[-1].data)
        for namespace in controller.robot_namespaces:
            self.assertEqual(
                {"linear": 0.0, "angular": 0.0},
                status["robot_assignments"][namespace]["command"],
            )
            stop = controller.cmd_vel_pubs[namespace].messages[-1]
            self.assertEqual(0.0, stop.linear.x)
            self.assertEqual(0.0, stop.angular.z)
            self.assertEqual(
                1,
                controller.avoidance_modules[namespace].reset_motion_count,
            )

    def test_transport_orderly_shutdown_zeroes_all_robot_commands(self):
        controller = self._transport_search_controller(3)
        previous_epoch = controller.command_epoch

        controller._shutdown()

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertEqual(previous_epoch + 1, controller.command_epoch)
        self.assertIsNone(controller.failure_reason)
        self.assertIsNone(controller.object_error)
        self.assertEqual(
            ROS["transport"].TransportPhase.SEARCH, controller.phase
        )
        for publisher in controller.cmd_vel_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

    def test_transport_correlated_stop_remains_idle_not_failed(self):
        controller = self._transport_search_controller(2)

        controller._stop_callback(lifecycle_payload("search-task"))

        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertIsNone(controller.failure_reason)
        self.assertIsNone(controller.object_error)
        self.assertEqual(
            ROS["transport"].TransportPhase.IDLE, controller.phase
        )
        for publisher in controller.cmd_vel_pubs.values():
            command = publisher.messages[-1]
            self.assertEqual(0.0, command.linear.x)
            self.assertEqual(0.0, command.angular.z)

        previous_counts = {
            namespace: len(publisher.messages)
            for namespace, publisher in controller.cmd_vel_pubs.items()
        }
        stale_command = Twist()
        stale_command.linear.x = float("nan")
        self.assertFalse(controller._publish_command(
            "tb3_0", stale_command, controller.command_epoch
        ))
        self.assertIsNone(controller.failure_reason)
        self.assertEqual(
            ROS["transport"].TransportPhase.IDLE, controller.phase
        )
        self.assertEqual(previous_counts, {
            namespace: len(publisher.messages)
            for namespace, publisher in controller.cmd_vel_pubs.items()
        })

        stale_batch = {
            "tb3_0": Twist(),
            "tb3_1": Twist(),
        }
        stale_batch["tb3_1"].angular.z = float("inf")
        self.assertFalse(controller._publish_command_batch(
            stale_batch,
            controller.command_epoch,
            ["tb3_0", "tb3_1"],
        ))
        self.assertIsNone(controller.failure_reason)
        self.assertEqual(
            ROS["transport"].TransportPhase.IDLE, controller.phase
        )
        self.assertEqual(previous_counts, {
            namespace: len(publisher.messages)
            for namespace, publisher in controller.cmd_vel_pubs.items()
        })

    def test_transport_new_start_waits_for_old_cycle_then_resets_its_state(self):
        controller = ROS["transport"].CollaborativeTransport()
        controller.current_task_id = "task-old"
        controller.is_running = True
        controller.phase = ROS["transport"].TransportPhase.APPROACH
        controller.model_states_received_at = ROS["transport"].time.monotonic()
        with controller.data_lock:
            controller.robot_odom_received_at = {
                namespace: 0.0 for namespace in controller.robot_namespaces
            }
        with controller.model_lock:
            controller.object_position = np.array([-0.8, -1.6])
            controller.object_found = True
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()

        phase_entered = threading.Event()
        release_phase = threading.Event()

        def finish_old_phase(_epoch):
            phase_entered.set()
            release_phase.wait(timeout=2.0)
            controller.transport_roles = {"stale-role": {}}

        controller._approach_phase = finish_old_phase
        control_thread = threading.Thread(
            target=controller._control_loop, args=(None,)
        )
        control_thread.start()
        self.assertTrue(phase_entered.wait(timeout=1.0))

        start_finished = threading.Event()

        def start_new_task():
            controller._start_callback(String(data=json.dumps({
                "task_id": "task-new",
            })))
            start_finished.set()

        start_thread = threading.Thread(target=start_new_task)
        start_thread.start()
        self.assertFalse(start_finished.wait(timeout=0.05))

        release_phase.set()
        control_thread.join(timeout=2.0)
        start_thread.join(timeout=2.0)

        self.assertFalse(control_thread.is_alive())
        self.assertFalse(start_thread.is_alive())
        self.assertTrue(start_finished.is_set())
        self.assertEqual("task-new", controller.current_task_id)
        self.assertEqual({}, controller.transport_roles)
        self.assertEqual(
            ROS["transport"].TransportPhase.SEARCH,
            controller.phase,
        )

    def test_transport_estop_returns_before_active_cycle_finishes(self):
        controller = ROS["transport"].CollaborativeTransport()
        controller.current_task_id = "task-old"
        controller.is_running = True
        controller.phase = ROS["transport"].TransportPhase.APPROACH
        controller.model_states_received_at = ROS["transport"].time.monotonic()
        with controller.data_lock:
            controller.robot_odom_received_at = {
                namespace: 0.0 for namespace in controller.robot_namespaces
            }
        with controller.model_lock:
            controller.object_position = np.array([-0.8, -1.6])
            controller.object_found = True
        controller._publish_status = mock.Mock()
        controller._publish_markers = mock.Mock()

        zero_published = threading.Event()

        class StopAwarePublisher(FakePublisher):
            def publish(self, message):
                super().publish(message)
                if (
                    message.linear.x == 0.0
                    and message.angular.z == 0.0
                ):
                    zero_published.set()

        with controller.data_lock:
            old_publishers = list(controller.cmd_vel_pubs.values())
            controller.cmd_vel_pubs = {
                namespace: StopAwarePublisher()
                for namespace in controller.robot_namespaces
            }
            replacement_publishers = tuple(
                controller.cmd_vel_pubs.items()
            )
        for publisher in old_publishers:
            controller._discard_safety_lane(publisher)
        for namespace, publisher in replacement_publishers:
            self.assertTrue(controller._register_safety_lane(
                namespace, publisher
            ))
        controller._refresh_safety_publisher_snapshot(
            replacement_publishers
        )

        phase_entered = threading.Event()
        release_phase = threading.Event()

        def finish_old_phase(_epoch):
            phase_entered.set()
            release_phase.wait(timeout=2.0)
            controller.transport_engaged = {"stale-engagement"}

        controller._approach_phase = finish_old_phase
        control_thread = threading.Thread(
            target=controller._control_loop, args=(None,)
        )
        control_thread.start()
        self.assertTrue(phase_entered.wait(timeout=1.0))

        estop_finished = threading.Event()

        def emergency_stop():
            controller._emergency_stop_callback(Bool(data=True))
            estop_finished.set()

        estop_thread = threading.Thread(target=emergency_stop)
        estop_thread.start()
        self.assertTrue(zero_published.wait(timeout=0.5))
        self.assertTrue(estop_finished.wait(timeout=0.5))
        self.assertTrue(control_thread.is_alive())

        release_phase.set()
        control_thread.join(timeout=2.0)
        estop_thread.join(timeout=2.0)

        self.assertFalse(control_thread.is_alive())
        self.assertFalse(estop_thread.is_alive())
        self.assertTrue(estop_finished.is_set())
        self.assertEqual(set(), controller.transport_engaged)
        self.assertFalse(controller.is_running)
        for _namespace, publisher in replacement_publishers:
            controller._discard_safety_lane(publisher)

    def test_transport_estop_fanout_bypasses_a_blocked_publisher(self):
        release_blocked = threading.Event()
        blocked = BlockingPublisher(release_blocked)
        healthy = FakePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': blocked,
            'tb3_1': healthy,
        })

        try:
            started_at = time.monotonic()
            controller._emergency_stop_callback(Bool(data=True))
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.2)
            self.assertTrue(blocked.entered.wait(0.5))
            deadline = time.monotonic() + 0.5
            while not healthy.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(healthy.messages)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
            self.assertTrue(controller.emergency_stop_active)
            self.assertFalse(controller.is_running)
        finally:
            release_blocked.set()
            controller._wait_for_safety_zeros(1.0)
            controller._discard_safety_lane(blocked)
            controller._discard_safety_lane(healthy)

    def test_transport_estop_retries_a_zero_publish_exception(self):
        publisher = FailOncePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })

        try:
            controller._emergency_stop_callback(Bool(data=True))
            self.assertTrue(publisher.first_failure_returned.wait(0.5))
            deadline = time.monotonic() + 0.5
            while not publisher.messages and time.monotonic() < deadline:
                controller._control_loop(None)
                time.sleep(0.005)
            self.assertTrue(publisher.messages)
            self.assertGreaterEqual(publisher.calls, 2)
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)
        finally:
            controller._wait_for_safety_zeros(1.0)
            controller._discard_safety_lane(publisher)

    def test_transport_normal_stop_retries_before_returning(self):
        publisher = FailOncePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })

        try:
            controller._stop_all_robots()
            self.assertEqual(2, publisher.calls)
            self.assertEqual(1, len(publisher.messages))
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)
        finally:
            controller._discard_safety_lane(publisher)

    def test_transport_completion_fails_when_zero_is_never_accepted(self):
        class AlwaysFailPublisher(FakePublisher):
            def __init__(self):
                super().__init__()
                self.calls = 0

            def publish(self, _message):
                self.calls += 1
                raise RuntimeError('publisher remains disconnected')

        publisher = AlwaysFailPublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })
        controller.object_error = None
        controller.failure_reason = None

        try:
            completed = controller._complete_transport(0.1, 4)

            self.assertFalse(completed)
            self.assertGreaterEqual(publisher.calls, 3)
            self.assertFalse(controller.is_running)
            self.assertEqual(5, controller.command_epoch)
            self.assertEqual(
                ROS['transport'].TransportPhase.FAILED,
                controller.phase,
            )
            self.assertIn('could not confirm zero', controller.failure_reason)
        finally:
            controller._discard_safety_lane(publisher)

    def test_transport_shutdown_retries_a_transient_zero_failure(self):
        publisher = FailOncePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })

        try:
            controller._shutdown()

            self.assertGreaterEqual(publisher.calls, 2)
            self.assertTrue(publisher.messages)
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            self.assertEqual(0.0, publisher.messages[-1].angular.z)
        finally:
            controller._discard_safety_lane(publisher)

    def test_transport_shutdown_does_not_wait_for_a_blocked_motion_publish(self):
        release_motion = threading.Event()
        blocked = FirstPublishBlockingPublisher(release_motion)
        healthy = FakePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': blocked,
            'tb3_1': healthy,
        })
        command = Twist()
        command.linear.x = 0.1
        motion_result = []

        def publish_motion():
            with controller.command_lock:
                motion_result.append(
                    controller._publish_validated_command_locked(
                        'tb3_0', command, 4
                    )
                )

        motion = threading.Thread(target=publish_motion)
        shutdown = threading.Thread(target=controller._shutdown)
        motion.start()
        self.assertTrue(blocked.entered.wait(0.5))

        try:
            shutdown.start()
            deadline = time.monotonic() + 0.5
            while not healthy.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(healthy.messages)
            shutdown.join(0.5)
            self.assertFalse(shutdown.is_alive())
            self.assertTrue(motion.is_alive())
            self.assertFalse(controller.is_running)
            self.assertTrue(controller._shutdown_started)
        finally:
            release_motion.set()
            motion.join(1.0)
            shutdown.join(1.0)
            controller._wait_for_safety_zeros(1.0)
            controller._discard_safety_lane(blocked)
            controller._discard_safety_lane(healthy)

        self.assertFalse(motion.is_alive())
        self.assertEqual([False], motion_result)
        self.assertEqual(0.0, blocked.messages[-1].linear.x)
        self.assertEqual(0.0, blocked.messages[-1].angular.z)

    def test_transport_normal_stop_does_not_complete_before_zero_returns(self):
        release = threading.Event()
        publisher = BlockingPublisher(release)
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })
        finished = threading.Event()

        def stop_robots():
            controller._stop_all_robots()
            finished.set()

        stop = threading.Thread(target=stop_robots)

        stop.start()
        self.assertTrue(publisher.entered.wait(0.5))
        self.assertFalse(finished.is_set())
        try:
            release.set()
            stop.join(1.0)
        finally:
            release.set()
            stop.join(1.0)
            controller._discard_safety_lane(publisher)

        self.assertFalse(stop.is_alive())
        self.assertTrue(finished.is_set())
        self.assertEqual(0.0, publisher.messages[-1].linear.x)

    def test_transport_normal_stop_reaches_a_healthy_publisher_in_parallel(self):
        release = threading.Event()
        blocked = BlockingPublisher(release)
        healthy = FakePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': blocked,
            'tb3_1': healthy,
        })
        result = []
        stop = threading.Thread(
            target=lambda: result.append(controller._stop_all_robots())
        )

        try:
            stop.start()
            self.assertTrue(blocked.entered.wait(0.5))
            deadline = time.monotonic() + 0.5
            while not healthy.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(healthy.messages)
            self.assertEqual(0.0, healthy.messages[-1].linear.x)
            self.assertEqual(0.0, healthy.messages[-1].angular.z)
        finally:
            release.set()
            stop.join(1.0)
            controller._discard_safety_lane(blocked)
            controller._discard_safety_lane(healthy)

        self.assertFalse(stop.is_alive())
        self.assertTrue(result)

    def test_transport_stop_receipt_follows_an_inflight_zero_and_late_motion(self):
        publisher = FakePublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })
        controller.data_lock.acquire()
        drain_result = []
        drain = None

        try:
            controller._schedule_safety_zeros()
            deadline = time.monotonic() + 0.5
            while not publisher.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(publisher.messages)

            late_motion = Twist()
            late_motion.linear.x = 0.1
            publisher.publish(late_motion)
            drain = threading.Thread(
                target=lambda: drain_result.append(
                    controller._drain_safety_zeros(0.5)
                )
            )
            drain.start()
            time.sleep(0.03)
            self.assertTrue(drain.is_alive())
        finally:
            controller.data_lock.release()
            if drain is not None:
                drain.join(1.0)
            controller._discard_safety_lane(publisher)

        self.assertFalse(drain.is_alive())
        self.assertEqual([True], [result['confirmed'] for result in drain_result])
        self.assertGreaterEqual(len(publisher.messages), 3)
        self.assertTrue(any(
            message.linear.x > 0.0 for message in publisher.messages
        ))
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)

    def test_transport_late_positive_command_is_followed_by_a_zero(self):
        release_motion = threading.Event()
        publisher = FirstPublishBlockingPublisher(release_motion)
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })
        command = Twist()
        command.linear.x = 0.1
        results = []

        def publish_motion():
            with controller.command_lock:
                results.append(controller._publish_validated_command_locked(
                    'tb3_0', command, 4
                ))

        motion = threading.Thread(target=publish_motion)
        emergency = threading.Thread(
            target=controller._emergency_stop_callback,
            args=(Bool(data=True),),
        )
        motion.start()
        self.assertTrue(publisher.entered.wait(0.5))

        try:
            emergency.start()
            deadline = time.monotonic() + 0.5
            while not publisher.messages and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(publisher.messages)
            self.assertEqual(0.0, publisher.messages[-1].linear.x)
            release_motion.set()
            motion.join(1.0)
            emergency.join(1.0)
            self.assertFalse(motion.is_alive())
            self.assertFalse(emergency.is_alive())
            deadline = time.monotonic() + 0.5
            while (
                publisher.messages[-1].linear.x != 0.0
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
        finally:
            release_motion.set()
            motion.join(1.0)
            emergency.join(1.0)
            controller._wait_for_safety_zeros(1.0)
            controller._discard_safety_lane(publisher)

        self.assertEqual([False], results)
        self.assertTrue(any(
            message.linear.x > 0.0 for message in publisher.messages
        ))
        self.assertEqual(0.0, publisher.messages[-1].linear.x)
        self.assertEqual(0.0, publisher.messages[-1].angular.z)

    def test_transport_stale_stop_and_empty_fleet_cancel_epoch(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.data_lock = threading.Lock()
        controller.phase_lock = threading.Lock()
        controller.current_task_id = "task-new"
        controller.is_running = True
        controller.is_paused = False
        controller.command_epoch = 4
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.robot_namespaces = ["tb3_0"]
        controller.robot_count = 1
        controller.cmd_vel_pubs = {"tb3_0": FakePublisher()}
        controller.odom_subs = {"tb3_0": FakeSubscriber()}
        controller.scan_subs = {"tb3_0": FakeSubscriber()}
        controller.avoidance_modules = {"tb3_0": FakeAvoidance()}
        controller.robot_positions = {"tb3_0": np.zeros(2)}
        controller.robot_yaws = {"tb3_0": 0.0}
        controller.robot_velocities = {"tb3_0": np.zeros(2)}
        controller.robot_scans = {}

        controller._stop_callback(lifecycle_payload("task-old"))
        self.assertTrue(controller.is_running)
        self.assertEqual(4, controller.command_epoch)

        controller._stop_callback(lifecycle_payload("task-new"))
        self.assertFalse(controller.is_running)
        self.assertEqual(5, controller.command_epoch)
        controller.is_running = True
        controller.command_epoch = 5
        controller.phase = ROS["transport"].TransportPhase.PUSH

        old_pub = controller.cmd_vel_pubs["tb3_0"]
        controller._update_fleet([])
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertEqual(6, controller.command_epoch)
        self.assertEqual(
            ROS["transport"].TransportPhase.IDLE,
            controller.phase,
        )
        self.assertEqual([], controller.robot_namespaces)
        self.assertTrue(old_pub.unregistered)
        self.assertTrue(old_pub.messages)

    def test_transport_teardown_cleans_roster_when_disposal_blocks_or_fails(self):
        release = threading.Event()

        class BlockingDisposalPublisher(FakePublisher):
            def __init__(self):
                super().__init__()
                self.disposal_started = threading.Event()

            def unregister(self):
                self.disposal_started.set()
                release.wait(1.0)
                self.unregistered = True

        class FailingSubscriber(FakeSubscriber):
            def unregister(self):
                raise RuntimeError('subscriber is disconnected')

        class FailingAvoidance(FakeAvoidance):
            def shutdown(self):
                raise RuntimeError('avoidance shutdown failed')

        publisher = BlockingDisposalPublisher()
        controller = self._transport_estop_controller({
            'tb3_0': publisher,
        })
        controller.odom_subs = {'tb3_0': FailingSubscriber()}
        controller.scan_subs = {'tb3_0': FailingSubscriber()}
        controller.avoidance_modules = {'tb3_0': FailingAvoidance()}
        controller.robot_positions = {'tb3_0': np.zeros(2)}
        controller.robot_yaws = {'tb3_0': 0.0}
        controller.robot_velocities = {'tb3_0': np.zeros(2)}
        controller.robot_scans = {'tb3_0': object()}
        controller.robot_odom_received_at = {'tb3_0': 1.0}
        controller.transport_last_commands = {'tb3_0': (0.0, 0.0)}

        started_at = time.monotonic()
        try:
            controller._teardown_robot('tb3_0')
            elapsed = time.monotonic() - started_at
            self.assertLess(elapsed, 0.2)
            self.assertTrue(publisher.disposal_started.wait(0.5))
            self.assertEqual([], controller.robot_namespaces)
            self.assertEqual({}, controller.cmd_vel_pubs)
            self.assertEqual({}, controller.odom_subs)
            self.assertEqual({}, controller.scan_subs)
            self.assertEqual({}, controller.avoidance_modules)
            self.assertEqual({}, controller.robot_positions)
            self.assertEqual({}, controller.robot_yaws)
            self.assertEqual({}, controller.robot_velocities)
            self.assertEqual({}, controller.robot_scans)
            self.assertEqual({}, controller.robot_odom_received_at)
            self.assertEqual({}, controller.transport_last_commands)
            self.assertEqual((), controller._safety_publisher_snapshot)
            self.assertNotIn(
                id(publisher), controller._safety_publish_lanes
            )
        finally:
            release.set()

    def test_transport_shutdown_rejects_late_start_and_fleet_changes(self):
        controller = self._transport_search_controller(1)
        original_task_id = controller.current_task_id
        original_roster = list(controller.robot_namespaces)

        controller._shutdown()
        controller._start_callback(String(data=json.dumps({
            'task_id': 'after-shutdown',
            'target_x': 1.0,
            'target_y': 1.0,
        })))
        controller._update_fleet(['tb3_9'])

        self.assertTrue(controller._shutdown_started)
        self.assertFalse(controller.is_running)
        self.assertFalse(controller.is_paused)
        self.assertEqual(original_task_id, controller.current_task_id)
        self.assertEqual(original_roster, controller.robot_namespaces)

    def test_transport_active_roster_change_cancels_epoch_and_stops_old_fleet(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.data_lock = threading.Lock()
        controller.phase_lock = threading.Lock()
        controller.is_running = True
        controller.is_paused = False
        controller.command_epoch = 4
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.robot_namespaces = ["tb3_0", "tb3_1"]
        controller.robot_count = 2
        old_publishers = {
            namespace: FakePublisher()
            for namespace in controller.robot_namespaces
        }
        controller.cmd_vel_pubs = dict(old_publishers)
        controller.odom_subs = {
            namespace: FakeSubscriber()
            for namespace in controller.robot_namespaces
        }
        controller.scan_subs = {
            namespace: FakeSubscriber()
            for namespace in controller.robot_namespaces
        }
        controller.avoidance_modules = {
            namespace: FakeAvoidance()
            for namespace in controller.robot_namespaces
        }
        controller.robot_positions = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_yaws = {
            namespace: 0.0 for namespace in controller.robot_namespaces
        }
        controller.robot_velocities = {
            namespace: np.zeros(2)
            for namespace in controller.robot_namespaces
        }
        controller.robot_scans = {}
        controller.robot_odom_received_at = {
            namespace: 1.0 for namespace in controller.robot_namespaces
        }
        controller.transport_last_commands = {}

        def setup_robot(namespace):
            controller.cmd_vel_pubs[namespace] = FakePublisher()
            controller.odom_subs[namespace] = FakeSubscriber()
            controller.scan_subs[namespace] = FakeSubscriber()
            controller.avoidance_modules[namespace] = FakeAvoidance()
            controller.robot_positions[namespace] = np.zeros(2)
            controller.robot_yaws[namespace] = 0.0
            controller.robot_velocities[namespace] = np.zeros(2)
            controller.robot_odom_received_at[namespace] = None

        controller._setup_robot = mock.Mock(side_effect=setup_robot)

        controller._update_fleet(["tb3_1", "tb3_2"])

        self.assertEqual(5, controller.command_epoch)
        self.assertTrue(controller.is_running)
        self.assertEqual(["tb3_1", "tb3_2"], controller.robot_namespaces)
        controller._setup_robot.assert_called_once_with("tb3_2")
        for publisher in old_publishers.values():
            self.assertTrue(publisher.messages)
            self.assertEqual(0.0, publisher.messages[0].linear.x)
            self.assertEqual(0.0, publisher.messages[0].angular.z)

    def test_transport_done_status_keeps_cycle_task_id(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.current_task_id = "task-old"
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.command_epoch = 9
        controller.phase = ROS["transport"].TransportPhase.PUSH
        controller.phase_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.model_lock = threading.Lock()
        controller.robot_namespaces = []
        controller.robot_positions = {}
        controller.avoidance_modules = {}
        controller.object_position = np.array([1.0, 1.0])
        controller.model_states_received_at = ROS["transport"].time.monotonic()
        controller.model_states_timeout_wall_s = 0.75
        controller.target_x = 1.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 24
        controller._publish_markers = lambda _phase: None
        published_status = []

        def capture_status(phase, task_id=None, paused=None):
            published_status.append((phase, task_id, paused))

        controller._publish_status = capture_status

        def start_new_task_after_validation():
            controller.current_task_id = "task-new"
            controller.command_epoch += 1
            controller.is_running = True
            with controller.phase_lock:
                controller.phase = ROS["transport"].TransportPhase.SEARCH

        controller.command_lock = ExitHookLock(
            exit_number=2,
            hook=start_new_task_after_validation,
        )

        def finish_push(expected_epoch):
            controller.is_running = False
            controller.command_epoch = expected_epoch + 1
            with controller.phase_lock:
                controller.phase = ROS["transport"].TransportPhase.DONE

        controller._push_phase = finish_push
        controller._control_loop(None)

        self.assertEqual([
            (
                ROS["transport"].TransportPhase.DONE,
                "task-old",
                False,
            )
        ], published_status)
        self.assertEqual("task-new", controller.current_task_id)

    def test_transport_status_prefers_explicit_cycle_task_snapshot(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.current_task_id = "task-new"
        controller.is_paused = False
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.object_position = np.array([1.0, 1.0])
        controller.target_x = 1.0
        controller.target_y = 1.0
        controller.robot_namespaces = []
        controller.robot_positions = {}
        controller._active_planner = "grf"
        controller._active_grf_iterations = 24
        controller.transport_assembly_route_states = {
            "tb3_7": {
                "target": np.array([0.5, -0.25]),
                "waypoints": [np.array([0.5, -0.25])],
                "waypoint_index": 0,
                "complete": False,
                "reverse_active": True,
                "turn_direction": 0.0,
            }
        }
        controller.status_pub = FakePublisher()

        controller._publish_status(
            ROS["transport"].TransportPhase.DONE,
            task_id="task-old",
            paused=False,
        )

        status = json.loads(controller.status_pub.messages[0].data)
        self.assertEqual("task-old", status["task_id"])
        self.assertEqual("DONE", status["phase"])
        self.assertTrue(status["collision_events"]["terminal"])
        self.assertEqual(
            status["collision_events"]["watermark"],
            status["collision_events"]["terminal_watermark"],
        )
        self.assertEqual(
            {
                "waypoint_index": 0,
                "waypoint_count": 1,
                "complete": False,
                "reversing": True,
                "turning_for_handoff": False,
                "target": {"x": 0.5, "y": -0.25},
            },
            status["assembly_routes"]["tb3_7"],
        )

    def test_transport_status_exposes_search_and_discovery_state(self):
        controller = self._transport_search_controller(3)
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 0
        controller.transport_roles = {}
        controller.status_pub = FakePublisher()
        controller._build_search_routes(
            controller.robot_namespaces,
            controller.robot_positions,
        )

        controller._publish_status(
            ROS["transport"].TransportPhase.SEARCH
        )

        searching = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(3, searching["searching_robot_count"])
        self.assertIsNone(searching["discovery"])
        self.assertTrue(all(
            assignment["activity"] == "searching"
            and "search_target" in assignment
            for assignment in searching["robot_assignments"].values()
        ))

        controller.transport_discovery = {
            "event": "payload_found",
            "event_id": "search-task:payload-found",
            "task_id": "search-task",
            "announced": True,
            "finder": "tb3_1",
            "distance": 0.2,
            "object_position": {"x": 0.0, "y": 0.0},
            "sim_time": 12.5,
            "notified_robots": ["tb3_0", "tb3_2"],
        }
        controller.transport_useful_contributors = {"tb3_2", "tb3_0"}
        controller.transport_current_useful_pushers = {"tb3_2", "tb3_0"}
        controller.transport_push_reference_speed = 0.018
        controller._publish_status(
            ROS["transport"].TransportPhase.APPROACH
        )

        regrouping = json.loads(controller.status_pub.messages[-1].data)
        self.assertEqual(0, regrouping["searching_robot_count"])
        self.assertEqual("tb3_1", regrouping["discovery"]["finder"])
        self.assertEqual(2, regrouping["useful_contributor_count"])
        self.assertEqual(
            ["tb3_0", "tb3_2"],
            regrouping["useful_contributor_ids"],
        )
        self.assertEqual(2, regrouping["current_useful_pusher_count"])
        self.assertEqual(
            ["tb3_0", "tb3_2"],
            regrouping["current_useful_pusher_ids"],
        )
        self.assertEqual(
            0.009, regrouping["all_pusher_proof_minimum_speed"]
        )
        self.assertEqual(
            "rendezvousing",
            regrouping["robot_assignments"]["tb3_1"]["activity"],
        )
        self.assertTrue(
            regrouping["robot_assignments"]["tb3_1"]["finder"]
        )
        self.assertTrue(all(
            assignment["notice_received"]
            for assignment in regrouping["robot_assignments"].values()
        ))

    def test_transport_status_exposes_queue_docking_lifecycle(self):
        controller = self._transport_search_controller(1)
        controller.target_x = 2.0
        controller.target_y = 1.0
        controller._active_planner = "grf"
        controller._active_grf_iterations = 0
        controller.status_pub = FakePublisher()
        controller.transport_queue_docking_started = False
        controller.transport_queue_settle_started_at = None

        def publish_queue_status():
            controller._publish_status(
                ROS["transport"].TransportPhase.PUSH
            )
            return json.loads(controller.status_pub.messages[-1].data)

        waiting = publish_queue_status()
        self.assertFalse(waiting["queue_settling"])
        self.assertFalse(waiting["queue_docking_started"])

        controller.transport_queue_settle_started_at = 10.0
        settling = publish_queue_status()
        self.assertTrue(settling["queue_settling"])
        self.assertFalse(settling["queue_docking_started"])

        controller.transport_queue_settle_started_at = None
        controller.transport_queue_docking_started = True
        docking = publish_queue_status()
        self.assertFalse(docking["queue_settling"])
        self.assertTrue(docking["queue_docking_started"])

        controller._stop_callback(lifecycle_payload("search-task"))
        stopped = publish_queue_status()
        self.assertFalse(stopped["queue_settling"])
        self.assertFalse(stopped["queue_docking_started"])

    def test_transport_coordinated_speed_scales_with_chain_depth(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.16

        expected_scales = {
            0: 0.88,
            1: 0.30,
            2: 0.25,
            3: 0.20,
            4: 0.15,
            12: 0.15,
        }
        for depth, scale in expected_scales.items():
            with self.subTest(depth=depth):
                self.assertAlmostEqual(
                    scale * controller.vmax,
                    controller._coordinated_push_speed(depth),
                )

    def test_transport_push_reference_moves_once_per_connected_batch(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.vmax = 0.16
        controller.transport_contact_closing_speed = 0.018
        controller.transport_push_ramp_step = 0.006
        controller.transport_push_ramp_hold_batches = 2
        controller.transport_push_ramp_initial_speed = 0.018
        controller.transport_push_reference_speed = 0.018
        controller.transport_push_ramp_batches = 0
        targets = {
            "tb3_{}".format(index): {"max_chain_depth": 3}
            for index in range(4)
        }

        controller._advance_push_reference(targets, True, True)
        controller._advance_push_reference(targets, True, True)
        self.assertAlmostEqual(
            0.018, controller.transport_push_reference_speed
        )
        self.assertEqual(2, controller.transport_push_ramp_batches)

        controller._advance_push_reference(targets, True, True)
        self.assertAlmostEqual(
            0.024, controller.transport_push_reference_speed
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

        # Every step gets a fresh settling window, not just the first one.
        controller._advance_push_reference(targets, True, True)
        controller._advance_push_reference(targets, True, True)
        self.assertAlmostEqual(
            0.024, controller.transport_push_reference_speed
        )
        self.assertEqual(2, controller.transport_push_ramp_batches)
        controller._advance_push_reference(targets, True, True)
        self.assertAlmostEqual(
            0.030, controller.transport_push_reference_speed
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

        # A long chain tops out at the coordinated fleet pace, even if the
        # next complete batch would otherwise step past it.
        controller.transport_push_reference_speed = 0.055
        controller.transport_push_ramp_batches = 2
        controller._advance_push_reference(targets, True, True)
        self.assertAlmostEqual(
            0.20 * controller.vmax,
            controller.transport_push_reference_speed,
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

        # Poor wheel tracking backs off one step and restarts the hold.
        controller.transport_push_reference_speed = 0.030
        controller.transport_push_ramp_batches = 2
        controller._advance_push_reference(targets, True, False)
        self.assertAlmostEqual(
            0.024, controller.transport_push_reference_speed
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

        controller._advance_push_reference(targets, False, False)
        self.assertAlmostEqual(
            0.018, controller.transport_push_reference_speed
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

        # Disconnect recovery must never raise an already calmer reference.
        controller.transport_push_reference_speed = 0.012
        controller.transport_push_ramp_batches = 4
        controller._advance_push_reference(targets, False, False)
        self.assertAlmostEqual(
            0.012, controller.transport_push_reference_speed
        )
        self.assertEqual(0, controller.transport_push_ramp_batches)

    def test_transport_all_pusher_proof_is_sticky_until_task_boundary(self):
        (
            controller,
            positions,
            targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller.transport_useful_contributors = set(positions)
        controller.transport_all_push_hold_time = 0.75
        controller._advance_push_reference = mock.Mock()
        final_speeds = {"tb3_0": 0.030, "tb3_1": 0.014}

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            command.linear.x = final_speeds[namespace]
            return command

        controller._apply_transport_avoidance = apply_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=9.0,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )

        self.assertEqual(
            set(), controller.transport_current_useful_pushers
        )
        self.assertFalse(controller.transport_all_pushers_confirmed)

        final_speeds["tb3_1"] = 0.016
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )

        self.assertEqual(
            set(positions), controller.transport_current_useful_pushers
        )
        self.assertFalse(controller.transport_all_pushers_confirmed)

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.74,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        self.assertFalse(controller.transport_all_pushers_confirmed)

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.75,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        self.assertTrue(controller.transport_all_pushers_confirmed)

        # A later weak batch clears only the in-progress interval. The proven
        # fleet contribution remains sticky for terminal reconnect recovery.
        final_speeds["tb3_1"] = 0.014
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.76,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        self.assertTrue(controller.transport_all_pushers_confirmed)
        self.assertIsNone(controller.transport_all_pushers_since)

        final_speeds["tb3_1"] = 0.016
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=11.0,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        self.assertTrue(controller.transport_all_pushers_confirmed)
        self.assertAlmostEqual(11.0, controller.transport_all_pushers_since)

        controller._set_phase(ROS["transport"].TransportPhase.PUSH, 4)
        self.assertFalse(controller.transport_all_pushers_confirmed)
        self.assertIsNone(controller.transport_all_pushers_since)
        self.assertEqual(set(), controller.transport_useful_contributors)

    def test_transport_all_pusher_proof_tracks_calm_reference_speed(self):
        (
            controller,
            positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller.transport_push_reference_speed = 0.018
        controller.transport_all_push_hold_time = 0.75
        controller._advance_push_reference = mock.Mock()
        final_speeds = {"tb3_0": 0.011, "tb3_1": 0.008}

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            command.linear.x = final_speeds[namespace]
            return command

        controller._apply_transport_avoidance = apply_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.018, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=9.0,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        # Loaded-chain coordination uses the weakest wheel command for the
        # whole column, so one lagging robot makes this entire batch weak.
        self.assertEqual(
            set(), controller.transport_current_useful_pushers
        )
        self.assertFalse(controller.transport_all_pushers_confirmed)

        final_speeds["tb3_1"] = 0.011
        for timestamp in (10.0, 10.75):
            with mock.patch.object(
                ROS["transport"].rospy, "get_time",
                return_value=timestamp, create=True,
            ):
                controller._publish_grf_commands(
                    list(positions), commands, positions, yaws,
                    np.zeros(2), 0.0, 4,
                )

        self.assertEqual(
            set(positions), controller.transport_current_useful_pushers
        )
        self.assertTrue(controller.transport_all_pushers_confirmed)

    def test_transport_control_sequence_survives_same_task_recovery(self):
        (
            controller,
            _positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller.transport_control_sequence = 7

        controller._reset_transport_route()
        self.assertEqual(7, controller.transport_control_sequence)

        controller._set_phase(ROS["transport"].TransportPhase.PUSH, 4)
        self.assertEqual(7, controller.transport_control_sequence)

        # A new task or terminal reset starts a fresh sequence namespace.
        controller._reset_transport_route(reset_control_sequence=True)
        self.assertEqual(0, controller.transport_control_sequence)

    def test_transport_credits_completed_command_interval_before_weak_batch(self):
        (
            controller,
            positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller.transport_all_push_hold_time = 0.75
        controller._advance_push_reference = mock.Mock()
        final_speeds = {namespace: 0.016 for namespace in positions}

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            command.linear.x = final_speeds[namespace]
            return command

        controller._apply_transport_avoidance = apply_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )
        self.assertFalse(controller.transport_all_pushers_confirmed)
        self.assertAlmostEqual(10.0, controller.transport_all_pushers_since)

        final_speeds["tb3_1"] = 0.014
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            side_effect=(10.8, 10.9),
            create=True,
        ):
            controller._publish_grf_commands(
                list(positions), commands, positions, yaws,
                np.zeros(2), 0.0, 4,
            )

        self.assertTrue(controller.transport_all_pushers_confirmed)
        self.assertIsNone(controller.transport_all_pushers_since)

    def test_transport_epoch_flip_after_final_publish_skips_batch_commit(self):
        (
            controller,
            positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller.transport_synchronized_push_started = False
        controller.transport_control_sequence = 7
        controller.transport_last_batch_publish_span = 1.25
        controller.transport_useful_contributors = {"old_contributor"}
        controller.transport_current_useful_pushers = {"old_pusher"}
        controller.transport_all_pushers_confirmed = True
        controller.transport_all_pushers_since = 3.5
        controller.transport_push_reference_speed = 0.030
        controller.transport_push_ramp_batches = 2
        old_arbitration = {
            "raw": {"old_robot": 0.021},
            "coordinated": {"old_robot": 0.019},
            "links": {"old_robot": ("nominal", 0.140, 0.0)},
            "stops": ("old_stop",),
        }
        controller.transport_push_raw_speeds = old_arbitration["raw"].copy()
        controller.transport_push_coordinated_speeds = (
            old_arbitration["coordinated"].copy()
        )
        controller.transport_push_link_states = old_arbitration["links"].copy()
        controller.transport_push_hard_stop_sources = old_arbitration["stops"]
        controller._advance_push_reference = mock.Mock()
        commit_commands = {
            namespace: mock.Mock() for namespace in positions
        }
        controller.avoidance_modules = {
            namespace: types.SimpleNamespace(
                commit_published_command=commit_commands[namespace],
                update_robot_positions=mock.Mock(),
                set_position=mock.Mock(),
            )
            for namespace in positions
        }
        publish_count = [0]

        def publish_and_cancel_epoch(_namespace, _command, _epoch):
            publish_count[0] += 1
            if publish_count[0] == len(positions):
                controller.command_epoch += 1
            return True

        controller._publish_command = publish_and_cancel_epoch
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        committed = controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )

        self.assertFalse(committed)
        self.assertEqual(len(positions), publish_count[0])
        self.assertEqual(5, controller.command_epoch)
        self.assertEqual(7, controller.transport_control_sequence)
        self.assertAlmostEqual(
            1.25, controller.transport_last_batch_publish_span
        )
        self.assertEqual(
            {"old_contributor"}, controller.transport_useful_contributors
        )
        self.assertEqual(
            {"old_pusher"}, controller.transport_current_useful_pushers
        )
        self.assertTrue(controller.transport_all_pushers_confirmed)
        self.assertAlmostEqual(3.5, controller.transport_all_pushers_since)
        self.assertAlmostEqual(
            0.030, controller.transport_push_reference_speed
        )
        self.assertEqual(2, controller.transport_push_ramp_batches)
        self.assertFalse(controller.transport_synchronized_push_started)
        self.assertEqual(
            old_arbitration["raw"], controller.transport_push_raw_speeds
        )
        self.assertEqual(
            old_arbitration["coordinated"],
            controller.transport_push_coordinated_speeds,
        )
        self.assertEqual(
            old_arbitration["links"], controller.transport_push_link_states
        )
        self.assertEqual(
            old_arbitration["stops"],
            controller.transport_push_hard_stop_sources,
        )
        publish_order = sorted(
            positions,
            key=lambda namespace: -int(namespace.rsplit("_", 1)[1]),
        )
        commit_commands[publish_order[0]].assert_called_once()
        commit_commands[publish_order[-1]].assert_not_called()
        controller._advance_push_reference.assert_not_called()

    def test_transport_disconnected_batch_clears_arbitration_atomically(self):
        (
            controller,
            positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller._advance_push_reference = mock.Mock()
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        self.assertTrue(controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        ))
        self.assertTrue(controller.transport_push_raw_speeds)
        self.assertTrue(controller.transport_push_coordinated_speeds)
        self.assertTrue(controller.transport_push_link_states)

        controller.transport_physical_engaged = {"tb3_0"}
        self.assertTrue(controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        ))

        self.assertEqual(2, controller.transport_control_sequence)
        self.assertEqual({}, controller.transport_push_raw_speeds)
        self.assertEqual({}, controller.transport_push_coordinated_speeds)
        self.assertEqual({}, controller.transport_push_link_states)
        self.assertEqual((), controller.transport_push_hard_stop_sources)

    def test_transport_ramp_waits_for_post_avoidance_tracking(self):
        (
            controller,
            positions,
            targets,
            _published,
        ) = self._connected_transport_publish_controller()
        advance_reference = mock.Mock()
        controller._advance_push_reference = advance_reference
        final_speeds = {namespace: 0.017 for namespace in positions}

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            command.linear.x = final_speeds[namespace]
            return command

        controller._apply_transport_avoidance = apply_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        advance_reference.assert_called_once_with(targets, True, False)

        advance_reference.reset_mock()
        final_speeds.update({namespace: 0.019 for namespace in positions})
        controller.robot_velocities = {
            namespace: np.array([0.010, 0.0])
            for namespace in positions
        }
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        advance_reference.assert_called_once_with(targets, True, False)

        advance_reference.reset_mock()
        controller.robot_velocities = {
            namespace: np.array([0.022, 0.0])
            for namespace in positions
        }
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        advance_reference.assert_called_once_with(targets, True, True)

    def test_complete_transport_chain_uses_the_slowest_safe_pace(self):
        (
            controller,
            positions,
            _targets,
            published,
        ) = self._connected_transport_publish_controller()
        controller._advance_push_reference = mock.Mock()
        avoidance = {
            namespace: mock.Mock() for namespace in positions
        }
        controller.avoidance_modules = avoidance

        def slow_rear_link(namespace, command, *_args, **_kwargs):
            if namespace == "tb3_1":
                command.linear.x = 0.016
            return command

        controller._apply_transport_avoidance = slow_rear_link
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )

        self.assertAlmostEqual(0.016, published["tb3_0"].linear.x)
        self.assertAlmostEqual(0.016, published["tb3_1"].linear.x)
        for namespace in positions:
            avoidance[
                namespace
            ].commit_published_command.assert_called_once_with(
                published[namespace]
            )

    def test_open_transport_link_stops_its_connected_prefix(self):
        (
            controller,
            positions,
            _targets,
            published,
        ) = self._connected_transport_publish_controller()
        controller._advance_push_reference = mock.Mock()
        controller.transport_physical_engaged = {"tb3_0"}
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )

        self.assertEqual(0.0, published["tb3_0"].linear.x)
        self.assertGreater(published["tb3_1"].linear.x, 0.0)

    def test_transport_requests_pressure_floor_for_safe_local_links(self):
        (
            controller,
            positions,
            _targets,
            _published,
        ) = self._connected_transport_publish_controller()
        controller._advance_push_reference = mock.Mock()
        controller.transport_push_reference_speed = 0.018
        requested_floors = {}

        def capture_floor(namespace, command, *_args, **kwargs):
            requested_floors[namespace] = kwargs["minimum_linear_speed"]
            return command

        controller._apply_transport_avoidance = capture_floor
        commands = {
            namespace: ROS["transport"].Vec2(0.030, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        controller.transport_physical_engaged = set(positions)
        controller.transport_aligned_engaged = set(positions)
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        self.assertEqual(
            {namespace: 0.018 for namespace in positions}, requested_floors
        )

        requested_floors.clear()
        yaws["tb3_1"] = 0.081
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        self.assertEqual(
            {"tb3_0": 0.018, "tb3_1": 0.0}, requested_floors
        )

        requested_floors.clear()
        yaws["tb3_1"] = 0.0
        controller.transport_physical_engaged = {"tb3_0"}
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        self.assertEqual(
            {"tb3_0": 0.0, "tb3_1": 0.018}, requested_floors
        )

        requested_floors.clear()
        positions["tb3_1"] = positions["tb3_0"] - np.array([0.1601, 0.0])
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        self.assertEqual(
            {"tb3_0": 0.0, "tb3_1": 0.018}, requested_floors
        )

        requested_floors.clear()
        positions["tb3_1"] = positions["tb3_0"] - np.array([0.176, 0.0])
        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )
        self.assertEqual(
            {"tb3_0": 0.0, "tb3_1": 0.0}, requested_floors
        )

    def test_transport_launch_waits_for_every_robot_heading(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            _published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_engagement_hold_time = 0.0
        controller.transport_launch_heading_tolerance = 0.08
        controller.transport_push_ramp_initial_speed = 0.015

        # Both bumpers are physically connected, but one robot is not yet
        # pointing closely enough along the shared push direction.
        yaws["tb3_1"] = 0.081
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertEqual(set(positions), controller.transport_engaged)
        self.assertEqual(
            set(positions), controller.transport_physical_engaged
        )
        self.assertEqual({"tb3_0"}, controller.transport_aligned_engaged)
        self.assertFalse(controller.transport_engagement_complete)

        yaws["tb3_1"] = 0.08
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.1,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertTrue(controller.transport_engagement_complete)
        self.assertEqual(
            set(positions), controller.transport_aligned_engaged
        )
        self.assertAlmostEqual(
            0.015, controller.transport_push_reference_speed
        )

    def test_touching_companion_turns_to_the_shared_push_heading(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_launch_heading_tolerance = 0.08

        # The child is touching at a small lateral offset. Its parent bearing
        # is already close enough for contact, but it is not the direction in
        # which the complete lane must push.
        positions["tb3_1"] = np.array([-0.144, 0.018])
        yaws["tb3_1"] = -0.12
        with mock.patch.object(
            ROS["transport"].rospy,
            "get_time",
            return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertEqual(0.0, published["tb3_1"].linear.x)
        self.assertGreater(published["tb3_1"].angular.z, 0.20)

    def test_transport_launch_clears_approach_momentum_before_push(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_engagement_hold_time = 0.0
        controller.transport_launch_settle_time = 0.35
        controller.transport_launch_settle_timeout = 1.25
        controller.transport_launch_settle_speed = 0.004
        controller.transport_push_ramp_initial_speed = 0.018
        controller.robot_velocities = {
            namespace: np.array([0.02, 0.0])
            for namespace in positions
        }
        avoidance = {
            namespace: mock.Mock() for namespace in positions
        }
        controller.avoidance_modules = avoidance

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertTrue(controller.transport_engagement_complete)
        self.assertAlmostEqual(
            10.0, controller.transport_launch_settle_started_at
        )
        for namespace in positions:
            self.assertEqual(0.0, published[namespace].linear.x)
            self.assertEqual(0.0, published[namespace].angular.z)
            avoidance[namespace].reset_motion.assert_called_once_with()

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.4,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertTrue(waiting)

        controller.robot_velocities = {
            namespace: np.array([0.003, 0.0])
            for namespace in positions
        }
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.5,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )
        self.assertFalse(waiting)
        self.assertIsNone(controller.transport_launch_settle_started_at)

    def test_transport_separates_exact_contact_from_release_hysteresis(self):
        (
            controller,
            positions,
            yaws,
            raw_contact,
            _published,
        ) = self._transport_chain_engagement_controller()

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertEqual(set(positions), controller.transport_engaged)
        self.assertEqual(
            set(positions), controller.transport_physical_engaged
        )
        self.assertEqual(
            set(positions), controller.transport_aligned_engaged
        )

        # The rear link has opened beyond exact contact, but remains inside
        # the wider release envelope. It stays in the calm connected prefix
        # without satisfying the physical launch/completion gate.
        raw_contact["tb3_1"] = False

        def companion_geometry(
            position, parent, direction, release=False, **_kwargs
        ):
            close_target = (
                np.asarray(parent)
                - np.asarray(direction)
                * controller.transport_companion_contact_distance
            )
            return bool(release), close_target

        controller._companion_engagement_geometry = companion_geometry
        controller.transport_synchronized_push_started = True
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.1,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertFalse(waiting)
        self.assertEqual(set(positions), controller.transport_engaged)
        self.assertEqual({"tb3_0"}, controller.transport_physical_engaged)
        self.assertEqual({"tb3_0"}, controller.transport_aligned_engaged)

    def test_touching_misaligned_chain_turns_without_creeping(self):
        (
            controller,
            positions,
            yaws,
            _raw_contact,
            published,
        ) = self._transport_chain_engagement_controller()
        controller.transport_launch_heading_tolerance = 0.08
        yaws.update({namespace: math.pi / 2.0 for namespace in positions})
        avoidance = {
            namespace: mock.Mock() for namespace in positions
        }
        controller.avoidance_modules = avoidance
        apply_avoidance = mock.Mock(
            side_effect=lambda _namespace, command, *_args, **_kwargs:
            command
        )
        controller._apply_transport_avoidance = apply_avoidance

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                list(positions), positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertFalse(controller.transport_engagement_complete)
        self.assertEqual(set(positions), controller.transport_engaged)
        self.assertEqual(
            set(positions), controller.transport_physical_engaged
        )
        self.assertEqual(set(), controller.transport_aligned_engaged)
        for namespace in positions:
            self.assertEqual(0.0, published[namespace].linear.x)
            self.assertAlmostEqual(-2.84, published[namespace].angular.z)
            avoidance[namespace].reset_motion.assert_called_once_with()
        apply_avoidance.assert_not_called()

    def test_engagement_publishes_zero_prefix_rear_first(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.avoidance_modules = {}
        controller.transport_engaged = set()
        controller.transport_engagement_complete = False
        controller.transport_engagement_ready_since = None
        controller.transport_engagement_last_ready = {}
        controller.transport_engagement_anchors = {}
        controller.transport_engagement_parent_distances = {}
        controller.transport_engagement_hold_time = 10.0
        controller.transport_engagement_release_hold_time = 0.35
        controller.transport_synchronized_push_started = False
        controller.transport_contact_slop = 0.005
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_closing_speed = 0.018
        controller.transport_launch_heading_tolerance = 0.08
        controller.vmax = 0.20

        namespaces = ["tb3_{}".format(index) for index in range(4)]
        positions = {
            namespace: np.array([-0.145 * index, 0.0])
            for index, namespace in enumerate(namespaces)
        }
        targets = {}
        for depth, namespace in enumerate(namespaces):
            parent = None if depth == 0 else namespaces[depth - 1]
            targets[namespace] = {
                "role": (
                    "payload_push" if depth == 0 else "companion_push"
                ),
                "chain_depth": depth,
                "chain_index": 0,
                "max_chain_depth": 3,
                "parent_namespace": parent,
                "parent_position": (
                    np.zeros(2) if parent is None else positions[parent]
                ),
                "position": positions[namespace].copy(),
                "push_direction": np.array([1.0, 0.0]),
            }

        controller._transport_targets = lambda *_args: targets
        controller._payload_contact_near = (
            lambda *_args, **_kwargs: True
        )
        controller._companion_engagement_geometry = (
            lambda position, *_args, **_kwargs: (True, position.copy())
        )
        controller._chain_pose_hold_command = (
            lambda *_args, **_kwargs: Twist()
        )
        empty_neighbours = {namespace: set() for namespace in namespaces}
        controller._transport_neighbours = lambda _targets: (
            empty_neighbours, empty_neighbours, empty_neighbours
        )
        controller._nearby_chain_contacts = lambda *_args, **_kwargs: ()
        controller._parallel_lane_contacts = lambda *_args, **_kwargs: ()
        controller._robot_lidar_masks = lambda *_args, **_kwargs: ()

        events = []

        def calculate(namespace, command, *_args, **_kwargs):
            events.append(("calculate", namespace))
            return command

        def publish(namespace, _command, _epoch):
            events.append(("publish", namespace))
            return True

        controller._apply_transport_avoidance = calculate
        controller._publish_command = publish
        yaws = {namespace: 0.0 for namespace in namespaces}

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            waiting = controller._engage_push_chains(
                namespaces, positions, yaws, np.zeros(2), 0.0, 4
            )

        self.assertTrue(waiting)
        self.assertEqual(
            [
                ("publish", namespace)
                for namespace in reversed(namespaces)
            ],
            events,
        )

    def test_transport_arrival_requires_exact_reconnected_current_push(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.obstacle_positions = []
        controller.target_x = 0.20
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.robot_namespaces = ["tb3_0", "tb3_1"]
        controller.robot_positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.385, 0.0]),
        }
        controller.robot_yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        controller.transport_arrival_latched = False
        controller.transport_arrival_direction = None
        controller.transport_near_target_recovery_started_at = None
        controller.transport_progress_timeout = 10.0
        controller.transport_synchronized_push_started = True
        # Release hysteresis may still call both links engaged, but terminal
        # completion requires a current exact-contact chain.
        controller.transport_engaged = set(controller.robot_namespaces)
        controller.transport_physical_engaged = {"tb3_0"}
        controller.transport_all_pushers_confirmed = True
        controller.transport_current_useful_pushers = set(
            controller.robot_namespaces
        )
        controller.transport_planner = "legacy"
        controller._engage_push_chains = lambda *_args: False
        controller._reposition_if_stalled = lambda *_args: False
        legacy_calls = []
        controller._legacy_push_step = (
            lambda *_args, **_kwargs: legacy_calls.append(True)
        )
        completions = []
        controller._complete_transport = (
            lambda distance, _epoch: completions.append(distance) or True
        )

        controller._push_phase(4)
        self.assertEqual([], completions)

        controller.transport_physical_engaged = set(
            controller.robot_namespaces
        )
        controller.transport_current_useful_pushers = {"tb3_0"}
        controller._push_phase(4)
        self.assertEqual([], completions)

        controller.transport_current_useful_pushers = set(
            controller.robot_namespaces
        )
        controller._push_phase(4)
        self.assertEqual([0.20], completions)
        self.assertEqual(2, len(legacy_calls))

    def test_transport_terminal_recovery_caps_four_deep_open_subtree(self):
        positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.385, 0.0]),
            # The open root is outside exact contact; its child is locally
            # loaded at a deliberately unambiguous sub-threshold distance.
            "tb3_2": np.array([-0.535, 0.0]),
            "tb3_3": np.array([-0.679, 0.0]),
        }
        targets = {}
        for depth, namespace in enumerate(positions):
            parent = None if depth == 0 else "tb3_{}".format(depth - 1)
            targets[namespace] = {
                "role": (
                    "payload_push" if depth == 0 else "companion_push"
                ),
                "chain_depth": depth,
                "chain_index": 0,
                "max_chain_depth": 3,
                "parent_namespace": parent,
                "parent_position": (
                    np.zeros(2) if parent is None else positions[parent]
                ),
                "position": positions[namespace],
                "push_direction": np.array([1.0, 0.0]),
            }

        controller, _published = self._transport_publish_controller(
            targets, [(positions["tb3_0"], True)]
        )
        controller.target_x = 0.20
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.transport_synchronized_push_started = True
        controller.transport_arrival_latched = True
        controller.transport_arrival_direction = np.array([1.0, 0.0])
        controller.transport_terminal_closing_speed = 0.010
        controller.transport_engaged = set(positions)
        controller.transport_physical_engaged = {"tb3_0", "tb3_1"}
        controller.transport_aligned_engaged = {"tb3_0", "tb3_1"}
        targets["tb3_2"]["parent_velocity"] = np.array([0.020, 0.0])
        targets["tb3_3"]["parent_velocity"] = np.array([0.030, 0.0])
        empty_neighbours = {namespace: set() for namespace in positions}
        controller._transport_neighbours = lambda _targets: (
            empty_neighbours, empty_neighbours, empty_neighbours
        )
        controller._nearby_chain_contacts = lambda *_args, **_kwargs: ()
        controller._parallel_lane_contacts = lambda *_args, **_kwargs: ()
        controller._robot_lidar_masks = lambda *_args, **_kwargs: ()

        def to_twist(vx, _vy, _yaw):
            command = Twist()
            command.linear.x = vx
            return command

        controller._holonomic_to_diff_drive = to_twist
        controller._stabilize_push_steering = lambda command: command
        controller._advance_push_reference = mock.Mock()
        pre_avoidance_speeds = {}

        def apply_avoidance(namespace, command, *_args, **_kwargs):
            pre_avoidance_speeds[namespace] = command.linear.x
            if namespace == "tb3_3":
                command.linear.x = 0.007
            return command

        controller._apply_transport_avoidance = apply_avoidance
        commands = {
            namespace: ROS["transport"].Vec2(0.08, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}
        published_batch = {}
        controller._publish_command = (
            lambda namespace, command, _epoch:
            published_batch.__setitem__(namespace, command.linear.x) or True
        )

        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )

        self.assertEqual(0.0, published_batch["tb3_0"])
        self.assertEqual(0.0, published_batch["tb3_1"])
        self.assertAlmostEqual(0.030, pre_avoidance_speeds["tb3_2"])
        self.assertAlmostEqual(0.030, pre_avoidance_speeds["tb3_3"])
        self.assertAlmostEqual(0.030, published_batch["tb3_2"])
        self.assertAlmostEqual(0.007, published_batch["tb3_3"])

    def test_transport_near_target_recovery_skips_stalled_reposition(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.obstacle_positions = []
        controller.target_x = 0.20
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.robot_namespaces = ["tb3_0", "tb3_1"]
        controller.robot_positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.385, 0.0]),
        }
        controller.robot_yaws = {"tb3_0": 0.0, "tb3_1": 0.0}
        controller.transport_arrival_latched = False
        controller.transport_arrival_direction = None
        controller.transport_near_target_recovery_started_at = None
        controller.transport_progress_timeout = 10.0
        controller.transport_synchronized_push_started = True
        controller.transport_engaged = set(controller.robot_namespaces)
        controller.transport_physical_engaged = {"tb3_0"}
        controller.transport_all_pushers_confirmed = True
        controller.transport_planner = "legacy"
        controller._engage_push_chains = lambda *_args: False
        controller._reposition_if_stalled = mock.Mock(return_value=True)
        controller._legacy_push_step = mock.Mock()
        controller._complete_transport = mock.Mock()

        controller._push_phase(4)

        controller._reposition_if_stalled.assert_not_called()
        controller._legacy_push_step.assert_called_once()
        controller._complete_transport.assert_not_called()

    def test_transport_arrival_latch_freezes_frame_and_persists_timeout(self):
        controller = ROS["transport"].CollaborativeTransport.__new__(
            ROS["transport"].CollaborativeTransport
        )
        controller.command_lock = threading.RLock()
        controller.model_lock = threading.Lock()
        controller.data_lock = threading.Lock()
        controller.command_epoch = 4
        controller.is_running = True
        controller.is_paused = False
        controller.emergency_stop_active = False
        controller.object_position = np.array([0.0, 0.0])
        controller.object_yaw = 0.0
        controller.object_error = None
        controller.obstacle_positions = []
        controller.target_x = 0.20
        controller.target_y = 0.0
        controller.arrival_tolerance = 0.50
        controller.transport_arrival_release_margin = 0.10
        controller.robot_namespaces = ["tb3_0"]
        controller.robot_positions = {"tb3_0": np.array([-0.24, 0.0])}
        controller.robot_yaws = {"tb3_0": 0.0}
        controller.transport_synchronized_push_started = True
        controller.transport_arrival_latched = False
        controller.transport_arrival_direction = None
        controller.transport_near_target_recovery_started_at = None
        controller.transport_progress_timeout = 1.0
        controller.transport_engaged = {"tb3_0"}
        controller.transport_physical_engaged = set()
        controller.transport_all_pushers_confirmed = True
        controller.transport_current_useful_pushers = set()
        controller.transport_planner = "legacy"
        controller._engage_push_chains = lambda *_args: False
        controller._reposition_if_stalled = mock.Mock(return_value=True)
        controller._legacy_push_step = mock.Mock()
        controller._complete_transport = mock.Mock()
        controller._fail_transport = mock.Mock()

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.0,
            create=True,
        ):
            controller._push_phase(4)

        self.assertTrue(controller.transport_arrival_latched)
        np.testing.assert_allclose(
            [1.0, 0.0], controller.transport_arrival_direction
        )
        self.assertAlmostEqual(
            10.0, controller.transport_near_target_recovery_started_at
        )
        controller._reposition_if_stalled.assert_not_called()

        # Crossing beyond the target would reverse an ordinary target vector.
        # The latched delivery frame and watchdog epoch remain unchanged.
        controller.object_position = np.array([0.75, 0.0])
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=10.5,
            create=True,
        ):
            controller._push_phase(4)
        goal, _, _ = controller._transport_frame(
            controller.object_position
        )
        np.testing.assert_allclose([1.0, 0.0], goal)
        self.assertAlmostEqual(
            10.0, controller.transport_near_target_recovery_started_at
        )
        controller._reposition_if_stalled.assert_not_called()

        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=11.0,
            create=True,
        ):
            controller._push_phase(4)

        controller._fail_transport.assert_called_once()
        self.assertIn(
            "complete fleet",
            controller._fail_transport.call_args.args[0],
        )
        self.assertEqual(2, controller._legacy_push_step.call_count)

        controller._fail_transport.reset_mock()
        controller.object_position = np.array([0.81, 0.0])
        with mock.patch.object(
            ROS["transport"].rospy, "get_time", return_value=11.1,
            create=True,
        ):
            controller._push_phase(4)
        controller._fail_transport.assert_called_once()
        self.assertIn(
            "left the target envelope",
            controller._fail_transport.call_args.args[0],
        )
        self.assertEqual(2, controller._legacy_push_step.call_count)

    def test_transport_publishes_one_rear_to_front_recovery_batch(self):
        positions = {
            "tb3_0": np.array([-0.24, 0.0]),
            "tb3_1": np.array([-0.385, 0.0]),
            "tb3_2": np.array([-0.530, 0.0]),
            "tb3_3": np.array([-0.675, 0.0]),
        }
        targets = {}
        for depth, namespace in enumerate(positions):
            parent = None if depth == 0 else "tb3_{}".format(depth - 1)
            targets[namespace] = {
                "role": (
                    "payload_push" if depth == 0 else "companion_push"
                ),
                "chain_depth": depth,
                "chain_index": 0,
                "max_chain_depth": 3,
                "parent_namespace": parent,
                "parent_position": (
                    np.zeros(2) if parent is None else positions[parent]
                ),
                "position": positions[namespace],
                "push_direction": np.array([1.0, 0.0]),
            }

        controller, _published = self._transport_publish_controller(
            targets, [(positions["tb3_0"], True)]
        )
        controller.transport_engaged = {"tb3_0", "tb3_1"}
        controller.transport_contact_closing_speed = 0.018
        controller.transport_control_sequence = 7
        empty_neighbours = {
            namespace: set() for namespace in positions
        }
        controller._transport_neighbours = lambda _targets: (
            empty_neighbours, empty_neighbours, empty_neighbours
        )
        controller._nearby_chain_contacts = (
            lambda *_args, **_kwargs: ()
        )
        controller._parallel_lane_contacts = (
            lambda *_args, **_kwargs: ()
        )
        controller._robot_lidar_masks = lambda *_args, **_kwargs: ()

        def to_twist(vx, _vy, _yaw):
            command = Twist()
            command.linear.x = vx
            return command

        controller._holonomic_to_diff_drive = to_twist
        controller._stabilize_push_steering = lambda command: command
        advance_reference = mock.Mock()
        controller._advance_push_reference = advance_reference
        published_batch = []

        def publish(namespace, command, _epoch):
            published_batch.append((namespace, command.linear.x))
            return True

        controller._publish_command = publish
        commands = {
            namespace: ROS["transport"].Vec2(0.08, 0.0)
            for namespace in positions
        }
        yaws = {namespace: 0.0 for namespace in positions}

        controller._publish_grf_commands(
            list(positions), commands, positions, yaws,
            np.zeros(2), 0.0, 4,
        )

        self.assertEqual(
            ["tb3_3", "tb3_2", "tb3_1", "tb3_0"],
            [namespace for namespace, _speed in published_batch],
        )
        speed_by_robot = dict(published_batch)
        self.assertAlmostEqual(0.018, speed_by_robot["tb3_0"])
        self.assertAlmostEqual(0.018, speed_by_robot["tb3_1"])
        self.assertAlmostEqual(0.08, speed_by_robot["tb3_2"])
        self.assertAlmostEqual(0.08, speed_by_robot["tb3_3"])
        self.assertEqual(8, controller.transport_control_sequence)
        advance_reference.assert_called_once_with(targets, False, False)


if __name__ == "__main__":
    unittest.main()
