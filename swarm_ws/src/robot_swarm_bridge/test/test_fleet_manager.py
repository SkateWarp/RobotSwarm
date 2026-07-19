#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import sys
import threading
import types
import unittest
import xml.etree.ElementTree as ET
from xml.dom import minidom
from unittest import mock

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "core" / "fleet_manager.py"
SPAWN_CONFIG = PACKAGE_ROOT / "config" / "arena_spawn_zones.yaml"
ARENA_WORLD = PACKAGE_ROOT / "worlds" / "swarm_arena.world"

with SPAWN_CONFIG.open(encoding="utf-8") as config_file:
    SPAWN_SETTINGS = yaml.safe_load(config_file)


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
        GetPhysicsProperties=type("GetPhysicsProperties", (), {}),
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
        "std_srvs": module("std_srvs"),
        "std_srvs.srv": module("std_srvs.srv", Empty=type("Empty", (), {})),
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
    manager.arena_profile = "swarm_arena"
    manager.publish_robot_tf = True
    manager.spawn_obstacle_clearance = SPAWN_SETTINGS[
        "spawn_obstacle_clearance"
    ]
    manager.spawn_search_step = SPAWN_SETTINGS["spawn_search_step"]
    manager.spawn_exclusion_zones = SPAWN_SETTINGS[
        "spawn_exclusion_zones"
    ]
    manager.robots = {}
    manager._lock = threading.Lock()
    manager._spawn_lock = threading.Lock()
    manager._next_index = 0
    manager._retired_robot_ids = set()
    manager._model_poses = {}
    manager._get_physics_srv = lambda: Message(
        success=True, pause=False, status_message=""
    )
    manager._pause_srv = lambda: None
    manager._unpause_srv = lambda: None
    manager.robot_list_pub = Publisher()
    return manager


class FleetManagerSpawnTests(unittest.TestCase):
    def test_processed_urdf_uses_the_controller_acceleration_limit(self):
        manager = make_manager()
        manager.gazebo_wheel_acceleration = 0.0
        document = minidom.parseString(
            '<robot><plugin><wheelAcceleration>1</wheelAcceleration>'
            '</plugin></robot>'
        )

        manager._configure_drive_plugin(document)

        value = document.getElementsByTagName(
            'wheelAcceleration'
        )[0].firstChild.nodeValue
        self.assertEqual('0', value)

    def test_spawn_launches_state_publisher_when_tf_is_enabled(self):
        manager = make_manager()
        manager.publish_robot_tf = True
        manager._robot_description_xml = "<robot />"
        manager._spawn_srv = lambda _request: Message(
            success=True, status_message=""
        )
        process = object()
        manager._launch_state_publisher = mock.Mock(return_value=process)

        with mock.patch.object(
            FLEET_MANAGER.rospy,
            "set_param",
            create=True,
        ):
            spawned = manager.spawn_single_robot(
                "tb3_0", make_pose(0.0, 0.0)
            )

        self.assertTrue(spawned)
        manager._launch_state_publisher.assert_called_once_with("tb3_0")
        self.assertIs(process, manager.robots["tb3_0"].state_pub_proc)

    def test_spawn_skips_state_publisher_when_tf_is_disabled(self):
        manager = make_manager()
        manager.publish_robot_tf = False
        manager._robot_description_xml = "<robot />"
        manager._spawn_srv = lambda _request: Message(
            success=True, status_message=""
        )
        manager._launch_state_publisher = mock.Mock()

        with mock.patch.object(
            FLEET_MANAGER.rospy,
            "set_param",
            create=True,
        ):
            spawned = manager.spawn_single_robot(
                "tb3_0", make_pose(0.0, 0.0)
            )

        self.assertTrue(spawned)
        manager._launch_state_publisher.assert_not_called()
        self.assertIsNone(manager.robots["tb3_0"].state_pub_proc)

    def test_spawn_clearance_stays_outside_the_emergency_stop_band(self):
        self.assertGreaterEqual(
            SPAWN_SETTINGS["spawn_obstacle_clearance"],
            0.20 + 0.05,
        )

    def assert_pose_is_safe(self, manager, pose):
        usable_half = manager.arena_size / 2.0 - manager.arena_margin
        self.assertLessEqual(abs(pose.position.x), usable_half + 1e-9)
        self.assertLessEqual(abs(pose.position.y), usable_half + 1e-9)

        for zone in manager._active_exclusion_zones():
            with self.subTest(zone=zone["name"]):
                self.assertGreaterEqual(
                    manager._clearance_from_zone(
                        pose.position.x, pose.position.y, zone
                    ),
                    manager.spawn_obstacle_clearance - 1e-9,
                )

    def test_arena_spawn_config_matches_every_collision_obstacle(self):
        world_root = ET.parse(ARENA_WORLD).getroot()
        models = {
            model.attrib["name"]: model
            for model in world_root.findall("./world/model")
        }
        collision_models = {
            name
            for name, model in models.items()
            if (name.startswith("obstacle_") or name == "transport_object")
            and model.find(".//collision/geometry") is not None
        }
        configured_models = {
            zone["model"] for zone in SPAWN_SETTINGS["spawn_exclusion_zones"]
        }
        self.assertEqual(collision_models, configured_models)

        for zone in SPAWN_SETTINGS["spawn_exclusion_zones"]:
            model = models[zone["model"]]
            world_pose = [float(value) for value in model.findtext("pose").split()]
            self.assertAlmostEqual(world_pose[0], zone["x"])
            self.assertAlmostEqual(world_pose[1], zone["y"])
            self.assertAlmostEqual(world_pose[5], zone.get("yaw", 0.0))

            if zone["shape"] == "box":
                size = [
                    float(value)
                    for value in model.findtext(
                        ".//collision/geometry/box/size"
                    ).split()
                ]
                self.assertAlmostEqual(size[0], zone["width"])
                self.assertAlmostEqual(size[1], zone["height"])
            else:
                radius = float(model.findtext(
                    ".//collision/geometry/cylinder/radius"
                ))
                self.assertAlmostEqual(radius, zone["radius"])

    def test_all_supported_patterns_are_safe_for_one_to_ten_robots(self):
        for pattern in ("grid", "circle", "line"):
            for count in range(1, 11):
                with self.subTest(pattern=pattern, count=count):
                    manager = make_manager()
                    manager.default_spacing = 1.0
                    ideal = manager._generate_positions(count, pattern)
                    placed = manager._generate_available_positions(
                        count, pattern, []
                    )

                    self.assertEqual(count, len(placed))
                    offsets = {
                        (
                            round(actual.position.x - requested.position.x, 9),
                            round(actual.position.y - requested.position.y, 9),
                        )
                        for requested, actual in zip(ideal, placed)
                    }
                    self.assertEqual(1, len(offsets))

                    for pose in placed:
                        self.assert_pose_is_safe(manager, pose)

                    for first_index, first in enumerate(placed):
                        for second in placed[first_index + 1:]:
                            distance = math.hypot(
                                first.position.x - second.position.x,
                                first.position.y - second.position.y,
                            )
                            self.assertGreaterEqual(
                                distance, manager.default_spacing - 1e-9
                            )

    def test_ten_robot_grid_moves_as_one_piece_away_from_diagonal_wall(self):
        manager = make_manager()
        manager.default_spacing = 1.0
        requested = manager._generate_positions(10, "grid")
        placed = manager._generate_available_positions(10, "grid", [])

        self.assertTrue(any(
            not manager._spawn_point_is_clear(
                pose.position.x, pose.position.y, []
            )
            for pose in requested
        ))
        offset = (
            placed[0].position.x - requested[0].position.x,
            placed[0].position.y - requested[0].position.y,
        )
        self.assertNotEqual((0.0, 0.0), offset)
        for requested_pose, placed_pose in zip(requested, placed):
            self.assertAlmostEqual(
                placed_pose.position.x - requested_pose.position.x,
                offset[0],
            )
            self.assertAlmostEqual(
                placed_pose.position.y - requested_pose.position.y,
                offset[1],
            )
            self.assert_pose_is_safe(manager, placed_pose)

    def test_live_transport_object_pose_is_used_for_clearance(self):
        manager = make_manager()
        object_pose = make_pose(0.0, 0.0)
        object_pose.orientation.z = math.sin(0.4 / 2.0)
        object_pose.orientation.w = math.cos(0.4 / 2.0)
        manager._on_model_states(Message(
            name=["transport_object"], pose=[object_pose]
        ))

        placed = manager._generate_available_positions(1, "grid", [])

        self.assertAlmostEqual(
            0.4, manager._model_poses["transport_object"][2]
        )
        self.assertEqual(1, len(placed))
        self.assertNotEqual(
            (0.0, 0.0),
            (placed[0].position.x, placed[0].position.y),
        )
        transport_zone = next(
            zone for zone in manager.spawn_exclusion_zones
            if zone["model"] == "transport_object"
        )
        self.assertGreaterEqual(
            manager._clearance_from_zone(
                placed[0].position.x,
                placed[0].position.y,
                transport_zone,
            ),
            manager.spawn_obstacle_clearance - 1e-9,
        )

    def test_model_states_stay_nonblocking_during_fleet_transaction(self):
        manager = make_manager()
        transport_pose = make_pose(1.2, -0.4)
        external_pose = make_pose(-0.8, 0.7)
        snapshot = Message(
            name=["transport_object", "tb3_42"],
            pose=[transport_pose, external_pose],
        )

        manager._spawn_lock.acquire()
        callback = threading.Thread(
            target=manager._on_model_states,
            args=(snapshot,),
        )
        callback.start()
        callback.join(timeout=0.25)
        try:
            self.assertFalse(
                callback.is_alive(),
                "model-state handling must not backpressure Gazebo",
            )
        finally:
            manager._spawn_lock.release()
            callback.join(timeout=1.0)

        self.assertEqual((1.2, -0.4), manager._model_poses["transport_object"][:2])
        self.assertNotIn("tb3_42", manager.robots)

        # External discovery is deferred until the next snapshot, after the
        # transaction has released the name/capacity lock.
        manager._on_model_states(snapshot)
        self.assertIn("tb3_42", manager.robots)
        self.assertEqual(43, manager._next_index)

    def test_placement_is_deterministic(self):
        manager = make_manager()
        manager.default_spacing = 1.0

        first = manager._generate_available_positions(10, "grid", [])
        second = manager._generate_available_positions(10, "grid", [])

        self.assertEqual(
            [(pose.position.x, pose.position.y) for pose in first],
            [(pose.position.x, pose.position.y) for pose in second],
        )

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
    @staticmethod
    def manager_with_robots(count=2):
        manager = make_manager()
        manager.robots = {
            "tb3_{}".format(index): FLEET_MANAGER.RobotRecord(
                "tb3_{}".format(index), make_pose(index * 0.5, 0.0)
            )
            for index in range(count)
        }
        return manager

    def test_running_simulation_is_paused_around_the_whole_delete_batch(self):
        manager = self.manager_with_robots()
        events = []
        manager._get_physics_srv = lambda: (
            events.append("state")
            or Message(success=True, pause=False, status_message="")
        )
        manager._pause_srv = lambda: events.append("pause")
        manager._unpause_srv = lambda: events.append("unpause")

        def delete(robot_name):
            events.append("delete {}".format(robot_name))
            with manager._lock:
                manager.robots.pop(robot_name, None)
            return True

        manager.delete_single_robot = delete

        self.assertEqual(2, manager.delete_robots())
        self.assertEqual(
            [
                "state",
                "pause",
                "delete tb3_0",
                "delete tb3_1",
                "unpause",
            ],
            events,
        )

    def test_simulation_that_was_already_paused_stays_paused(self):
        manager = self.manager_with_robots(1)
        events = []
        manager._get_physics_srv = lambda: Message(
            success=True, pause=True, status_message=""
        )
        manager._pause_srv = lambda: events.append("pause")
        manager._unpause_srv = lambda: events.append("unpause")

        def delete(robot_name):
            events.append("delete {}".format(robot_name))
            with manager._lock:
                manager.robots.pop(robot_name, None)
            return True

        manager.delete_single_robot = delete

        self.assertEqual(1, manager.delete_robots())
        self.assertEqual(["delete tb3_0"], events)

    def test_pause_failure_aborts_before_deleting_any_model(self):
        manager = self.manager_with_robots(1)
        manager._get_physics_srv = lambda: Message(
            success=True, pause=False, status_message=""
        )

        def fail_to_pause():
            raise FLEET_MANAGER.rospy.ServiceException("service unavailable")

        manager._pause_srv = fail_to_pause
        manager.delete_single_robot = mock.Mock(return_value=True)

        self.assertEqual(0, manager.delete_robots())
        manager.delete_single_robot.assert_not_called()
        self.assertEqual(["tb3_0"], manager.get_robot_names())

    def test_delete_exception_still_resumes_a_running_simulation(self):
        manager = self.manager_with_robots(1)
        resumed = []
        manager._get_physics_srv = lambda: Message(
            success=True, pause=False, status_message=""
        )
        manager._pause_srv = lambda: None
        manager._unpause_srv = lambda: resumed.append(True)

        def fail_to_delete(_robot_name):
            raise ValueError("unexpected model teardown failure")

        manager.delete_single_robot = fail_to_delete

        with self.assertRaisesRegex(ValueError, "teardown failure"):
            manager.delete_robots()
        self.assertEqual([True], resumed)

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
