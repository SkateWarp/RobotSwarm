#!/usr/bin/env python3

import importlib.util
import json
import math
import pathlib
import sys
import threading
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


class FakePublisher:
    def __init__(self, *args, **kwargs):
        self.messages = []
        self.unregistered = False

    def publish(self, message):
        self.messages.append(message)

    def unregister(self):
        self.unregistered = True


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

    def shutdown(self):
        self.shutdown_called = True

    def update_robot_positions(self, _positions):
        pass


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
        Empty=Empty,
    )
    visualization_msgs_msg = module(
        "visualization_msgs.msg",
        Marker=Marker,
        MarkerArray=MarkerArray,
    )
    gazebo_msgs_msg = module(
        "gazebo_msgs.msg",
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


def make_orchestrator(task_id="task-a"):
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
    orchestrator.task_dispatched = False
    orchestrator.task_lock = threading.RLock()
    orchestrator.emergency_stop_active = False
    orchestrator.robots = {
        "tb3_0": {
            "pose": Pose(),
            "velocity": Twist(),
            "status": "active",
            "threat_level": 0.0,
            "role": "follower",
        }
    }
    orchestrator.robot_sensor_data = {"tb3_0": {}}
    orchestrator.robot_count = 1
    orchestrator.emergency_stop_pub = FakePublisher()
    orchestrator.odom_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.threat_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.scan_subs = {"tb3_0": FakeSubscriber()}
    orchestrator.behavior_start_pubs = {
        "follow_leader": FakePublisher(),
    }
    orchestrator.behavior_stop_pubs = [
        FakePublisher(), FakePublisher(), FakePublisher()
    ]
    orchestrator.behavior_pause_pubs = {}
    orchestrator.behavior_resume_pubs = {}
    orchestrator._stop_all_robots = lambda: None
    orchestrator.control_watchdog_enabled = True
    orchestrator.control_heartbeat_timeout = 10.0
    orchestrator.control_heartbeat_seen = False
    orchestrator.last_control_heartbeat = None
    orchestrator.control_watchdog_tripped = False
    orchestrator._control_clock = lambda: 0.0
    return orchestrator


class ObstacleAvoidanceSafetyTests(unittest.TestCase):
    def test_fast_turn_checks_rear_clearance_and_stops(self):
        avoidance = ROS["obstacle"].ObstacleAvoidance("tb3_0")
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        avoidance.sector_min[4] = 0.1

        desired = Twist()
        desired.linear.x = 0.1
        desired.angular.z = 1.0
        safe = avoidance.apply_avoidance(desired)

        self.assertEqual(0.0, safe.linear.x)
        self.assertEqual(0.0, safe.angular.z)


class OrchestratorLifecycleTests(unittest.TestCase):
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
        original_sleep = ROS["orchestrator"].rospy.sleep

        def blocked_sleep(_duration):
            initialization_started.set()
            release_initialization.wait(timeout=1.0)

        ROS["orchestrator"].rospy.sleep = blocked_sleep
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
            ROS["orchestrator"].rospy.sleep = original_sleep

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


class ControlHeartbeatWatchdogTests(unittest.TestCase):
    def test_constructor_starts_and_shutdown_stops_wall_watchdog(self):
        orchestrator = ROS["orchestrator"].TaskOrchestrator()
        self.assertTrue(orchestrator._control_watchdog_thread.is_alive())
        self.assertFalse(orchestrator.control_heartbeat_seen)

        orchestrator._shutdown()

        self.assertFalse(orchestrator._control_watchdog_thread.is_alive())
        self.assertFalse(orchestrator.emergency_stop_active)

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

        wall_sleep.assert_called_once_with(0.01)

    def test_watchdog_waits_for_first_heartbeat_then_latches_estop(self):
        orchestrator = make_orchestrator()
        orchestrator.task_state = ROS["orchestrator"].TaskState.RUNNING
        orchestrator.task_dispatched = True
        clock = [100.0]
        orchestrator._control_clock = lambda: clock[0]

        self.assertFalse(orchestrator._check_control_watchdog(now=1000.0))
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.emergency_stop_pub.messages)

        orchestrator._control_heartbeat_callback(Empty())
        self.assertTrue(orchestrator.control_heartbeat_seen)
        self.assertFalse(orchestrator._check_control_watchdog(now=109.9))
        self.assertTrue(orchestrator._check_control_watchdog(now=110.1))

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

    def test_late_heartbeat_requires_explicit_reset_and_stale_reset_fails(self):
        orchestrator = make_orchestrator()
        clock = [10.0]
        orchestrator._control_clock = lambda: clock[0]
        orchestrator._control_heartbeat_callback(Empty())
        self.assertTrue(orchestrator._check_control_watchdog(now=20.1))

        clock[0] = 30.0
        orchestrator._handle_reset_emergency_stop({})
        self.assertTrue(orchestrator.emergency_stop_active)
        self.assertTrue(orchestrator.control_watchdog_tripped)

        clock[0] = 31.0
        orchestrator._control_heartbeat_callback(Empty())
        self.assertTrue(orchestrator.emergency_stop_active)
        orchestrator._handle_reset_emergency_stop({})
        self.assertFalse(orchestrator.emergency_stop_active)
        self.assertFalse(orchestrator.control_watchdog_tripped)
        self.assertFalse(orchestrator.emergency_stop_pub.messages[-1].data)


class BehaviorLifecycleTests(unittest.TestCase):
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
        controller.status_pub = FakePublisher()

        controller._publish_status(
            ROS["transport"].TransportPhase.DONE,
            task_id="task-old",
            paused=False,
        )

        status = json.loads(controller.status_pub.messages[0].data)
        self.assertEqual("task-old", status["task_id"])
        self.assertEqual("DONE", status["phase"])


if __name__ == "__main__":
    unittest.main()
