#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import sys
import threading
import types
import unittest
from unittest import mock


SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "core"
    / "fleet_manager.py"
)


class Message:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class Point(Message):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x=x, y=y, z=z)


class Quaternion(Message):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        super().__init__(x=x, y=y, z=z, w=w)


class Pose(Message):
    def __init__(self):
        super().__init__(position=Point(), orientation=Quaternion())


class String(Message):
    def __init__(self, data=""):
        super().__init__(data=data)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def module(name, **attributes):
    result = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(result, key, value)
    return result


def load_fleet_manager():
    rospy = module(
        "rospy",
        ServiceException=RuntimeError,
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logerr=lambda *args, **kwargs: None,
    )
    std_msgs_msg = module("std_msgs.msg", String=String)
    geometry_msgs_msg = module(
        "geometry_msgs.msg",
        Pose=Pose,
        Point=Point,
        Quaternion=Quaternion,
    )
    gazebo_msgs_srv = module(
        "gazebo_msgs.srv",
        SpawnModel=type("SpawnModel", (), {}),
        SpawnModelRequest=type("SpawnModelRequest", (), {}),
        DeleteModel=type("DeleteModel", (), {}),
        DeleteModelRequest=type("DeleteModelRequest", (), {}),
    )
    gazebo_msgs_msg = module(
        "gazebo_msgs.msg",
        ModelStates=type("ModelStates", (), {}),
    )
    transformations = module(
        "tf.transformations",
        quaternion_from_euler=lambda _r, _p, yaw: (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        ),
    )

    replacements = {
        "rospy": rospy,
        "rospkg": module("rospkg", RosPack=type("RosPack", (), {})),
        "xacro": module("xacro"),
        "std_msgs": module("std_msgs", msg=std_msgs_msg),
        "std_msgs.msg": std_msgs_msg,
        "geometry_msgs": module("geometry_msgs", msg=geometry_msgs_msg),
        "geometry_msgs.msg": geometry_msgs_msg,
        "gazebo_msgs": module(
            "gazebo_msgs", srv=gazebo_msgs_srv, msg=gazebo_msgs_msg
        ),
        "gazebo_msgs.srv": gazebo_msgs_srv,
        "gazebo_msgs.msg": gazebo_msgs_msg,
        "tf": module("tf", transformations=transformations),
        "tf.transformations": transformations,
    }
    previous = {
        name: sys.modules.get(name)
        for name in replacements
    }
    sys.modules.update(replacements)
    try:
        spec = importlib.util.spec_from_file_location(
            "fleet_manager_under_test", SCRIPT
        )
        loaded = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(loaded)
        return loaded
    finally:
        for name, original in previous.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


FLEET_MANAGER = load_fleet_manager()


def make_pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose


def make_manager(max_robots=0):
    manager = FLEET_MANAGER.FleetManager.__new__(
        FLEET_MANAGER.FleetManager
    )
    manager.max_robots = max_robots
    manager.default_spacing = 0.6
    manager.minimum_spawn_spacing = 0.35
    manager.arena_size = 10.0
    manager.arena_margin = 0.35
    manager.robots = {}
    manager._lock = threading.Lock()
    manager._spawn_lock = threading.Lock()
    manager._next_index = 0
    manager._retired_robot_ids = set()
    manager.robot_list_pub = Publisher()
    return manager


class FleetManagerSpawnTests(unittest.TestCase):
    def test_incremental_grid_spawn_avoids_existing_center_pose(self):
        manager = make_manager()
        manager.robots["tb3_0"] = FLEET_MANAGER.RobotRecord(
            "tb3_0", make_pose(0.0, 0.0)
        )

        def spawn(robot_name, pose):
            with manager._lock:
                manager.robots[robot_name] = FLEET_MANAGER.RobotRecord(
                    robot_name, pose
                )
            return True

        manager.spawn_single_robot = spawn
        spawned = manager.spawn_robots(
            1, "grid", robot_ids=["tb3_1"]
        )

        self.assertEqual(["tb3_1"], spawned)
        pose = manager.robots["tb3_1"].pose
        clearance = math.hypot(pose.position.x, pose.position.y)
        self.assertGreaterEqual(clearance, manager.default_spacing - 1e-9)
        self.assertNotEqual(
            (0.0, 0.0),
            (pose.position.x, pose.position.y),
        )

    def test_concurrent_spawn_respects_max_robots_barrier(self):
        manager = make_manager(max_robots=1)
        start_barrier = threading.Barrier(3)
        second_spawn_entered = threading.Event()
        spawn_count_lock = threading.Lock()
        spawn_count = 0
        results = {}

        def spawn(robot_name, pose):
            nonlocal spawn_count
            with spawn_count_lock:
                spawn_count += 1
                call_number = spawn_count
            if call_number == 1:
                second_spawn_entered.wait(timeout=0.25)
            else:
                second_spawn_entered.set()
            with manager._lock:
                manager.robots[robot_name] = FLEET_MANAGER.RobotRecord(
                    robot_name, pose
                )
            return True

        manager.spawn_single_robot = spawn

        def run(key, robot_id):
            start_barrier.wait()
            results[key] = manager.spawn_robots(
                1, "grid", robot_ids=[robot_id]
            )

        first = threading.Thread(target=run, args=("first", "tb3_0"))
        second = threading.Thread(target=run, args=("second", "tb3_1"))
        first.start()
        second.start()
        start_barrier.wait()
        first.join(timeout=2.0)
        second.join(timeout=2.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(1, len(manager.robots))
        self.assertEqual(
            [0, 1],
            sorted(len(result) for result in results.values()),
        )


class FleetManagerDeleteTests(unittest.TestCase):
    def test_stale_model_state_does_not_restore_deleted_robots(self):
        manager = make_manager()
        manager.robots = {
            "tb3_0": FLEET_MANAGER.RobotRecord(
                "tb3_0", make_pose(-0.3, 0.0)
            ),
            "tb3_1": FLEET_MANAGER.RobotRecord(
                "tb3_1", make_pose(0.3, 0.0)
            ),
        }

        def delete(robot_name):
            with manager._lock:
                manager.robots.pop(robot_name, None)
            return True

        manager.delete_single_robot = delete
        stale_snapshot = Message(
            name=["ground_plane", "tb3_0", "tb3_1"],
            pose=[
                make_pose(0.0, 0.0),
                make_pose(-0.3, 0.0),
                make_pose(0.3, 0.0),
            ],
        )

        self.assertEqual(2, manager.delete_robots())
        manager._on_model_states(stale_snapshot)

        self.assertEqual([], manager.get_robot_names())
        self.assertEqual({"tb3_0", "tb3_1"}, manager._retired_robot_ids)

        manager._on_model_states(
            Message(name=["ground_plane"], pose=[make_pose(0.0, 0.0)])
        )
        manager._on_model_states(stale_snapshot)

        self.assertEqual([], manager.get_robot_names())
        self.assertEqual({"tb3_0", "tb3_1"}, manager._retired_robot_ids)

    def test_successful_spawn_reactivates_a_retired_robot_id(self):
        manager = make_manager()
        manager._retired_robot_ids.add("tb3_0")
        manager._robot_description_xml = "<robot />"
        manager._spawn_srv = lambda _request: Message(
            success=True, status_message=""
        )
        manager._launch_state_publisher = lambda _robot_name: None

        with mock.patch.object(
            FLEET_MANAGER.rospy,
            "set_param",
            create=True,
        ):
            spawned = manager.spawn_single_robot(
                "tb3_0", make_pose(0.0, 0.0)
            )

        self.assertTrue(spawned)
        self.assertEqual(["tb3_0"], manager.get_robot_names())
        self.assertEqual(set(), manager._retired_robot_ids)

    def test_failed_gazebo_delete_keeps_the_robot_active(self):
        manager = make_manager()
        manager.robots["tb3_0"] = FLEET_MANAGER.RobotRecord(
            "tb3_0", make_pose(0.0, 0.0)
        )
        manager._delete_srv = lambda _request: Message(
            success=False,
            status_message="model is busy",
        )

        self.assertEqual(0, manager.delete_robots(["tb3_0"]))
        self.assertEqual(["tb3_0"], manager.get_robot_names())
        self.assertEqual(set(), manager._retired_robot_ids)


if __name__ == "__main__":
    unittest.main()
