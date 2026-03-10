#!/usr/bin/env python3
"""
Fleet Manager for TurtleBot3 Waffle Swarm
Handles dynamic spawning, deletion, and tracking of TurtleBot3 Waffle robots
in Gazebo using URDF/xacro processing and topic-based command interface.

Robot namespaces: tb3_0, tb3_1, ..., tb3_{N-1}
Topics per robot: /{ns}/cmd_vel, /{ns}/odom, /{ns}/scan
Fleet info: /fleet/robot_list (String, comma-separated namespaces, latched)

Command interface (topic-based, JSON):
  - /fleet/spawn_command  (String) -> /fleet/spawn_result  (String)
  - /fleet/delete_command (String) -> /fleet/delete_result (String)
"""

import rospy
import rospkg
import xacro
import numpy as np
import json
import math
import os
import signal
import subprocess
import threading
from typing import Dict, List, Optional, Tuple

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from gazebo_msgs.msg import ModelStates
import tf.transformations


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

class RobotRecord:
    """Tracks a single spawned robot."""

    def __init__(self, name: str, pose: Pose):
        self.name = name          # e.g. "tb3_0"
        self.pose = pose          # latest known pose
        self.state_pub_proc: Optional[subprocess.Popen] = None  # robot_state_publisher


# ---------------------------------------------------------------------------
# Fleet Manager
# ---------------------------------------------------------------------------

class FleetManager:
    """
    Manages a fleet of TurtleBot3 Waffle robots in Gazebo.

    Responsibilities
    ----------------
    * Spawn robots via ``/gazebo/spawn_urdf_model`` after xacro processing
    * Launch a ``robot_state_publisher`` per robot (subprocess)
    * Delete robots via ``/gazebo/delete_model``, kill child processes, clean params
    * Generate spawn positions in *grid*, *circle*, or *line* patterns
    * Track live poses from ``/gazebo/model_states``
    * Publish fleet roster on ``/fleet/robot_list`` (latched, comma-separated)
    * Accept spawn / delete commands via topics (JSON encoded)
    """

    def __init__(self):
        rospy.init_node('fleet_manager', anonymous=False)

        # ----- configuration via rospy params -----
        self.max_robots: int = rospy.get_param('~max_robots', 20)
        self.default_count: int = rospy.get_param('~robot_count', 5)
        self.default_pattern: str = rospy.get_param('~spawn_pattern', 'grid')
        self.default_spacing: float = rospy.get_param('~spawn_spacing', 1.5)
        self.arena_size: float = rospy.get_param('~arena_size', 10.0)  # meters, centered at origin
        self.auto_spawn: bool = rospy.get_param('~auto_spawn', False)

        # ----- internal state -----
        self.robots: Dict[str, RobotRecord] = {}
        self._lock = threading.Lock()
        self._next_index: int = 0  # monotonically increasing robot index

        # ----- prepare URDF template once -----
        self._robot_description_xml: str = self._process_urdf()

        # ----- Gazebo service proxies -----
        rospy.loginfo("Waiting for Gazebo spawn/delete services ...")
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=60.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=60.0)
        self._spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self._delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.loginfo("Gazebo services available.")

        # ----- publishers -----
        self.robot_list_pub = rospy.Publisher(
            '/fleet/robot_list', String, queue_size=1, latch=True
        )
        self.spawn_result_pub = rospy.Publisher(
            '/fleet/spawn_result', String, queue_size=10
        )
        self.delete_result_pub = rospy.Publisher(
            '/fleet/delete_result', String, queue_size=10
        )

        # ----- subscribers -----
        rospy.Subscriber(
            '/fleet/spawn_command', String, self._on_spawn_command, queue_size=5
        )
        rospy.Subscriber(
            '/fleet/delete_command', String, self._on_delete_command, queue_size=5
        )
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self._on_model_states, queue_size=1
        )

        # ----- periodic fleet list publish -----
        self._status_timer = rospy.Timer(rospy.Duration(1.0), self._publish_robot_list)

        # ----- auto-spawn -----
        if self.auto_spawn:
            rospy.loginfo(
                "Auto-spawn enabled: spawning %d robots in '%s' pattern",
                self.default_count, self.default_pattern,
            )
            # Small delay so Gazebo is fully ready
            rospy.Timer(
                rospy.Duration(2.0),
                lambda _evt: self.spawn_robots(self.default_count, self.default_pattern),
                oneshot=True,
            )

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("Fleet Manager initialised and ready.")

    # ------------------------------------------------------------------
    # URDF processing
    # ------------------------------------------------------------------

    def _process_urdf(self) -> str:
        """Process TurtleBot3 Waffle xacro into raw URDF XML string."""
        try:
            rospack = rospkg.RosPack()
            tb3_desc = rospack.get_path('turtlebot3_description')
            urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf.xacro')
            rospy.loginfo("Processing xacro: %s", urdf_path)
            doc = xacro.process_file(urdf_path)
            robot_description = doc.toxml()
            rospy.loginfo("URDF processed successfully (%d bytes).", len(robot_description))
            return robot_description
        except Exception as exc:
            rospy.logerr("Failed to process TurtleBot3 xacro: %s", exc)
            return ""

    # ------------------------------------------------------------------
    # Spawn patterns
    # ------------------------------------------------------------------

    def _generate_positions(self, count: int, pattern: str) -> List[Pose]:
        """
        Generate *count* spawn poses inside the arena (``arena_size`` x ``arena_size``
        centred at the origin).

        Supported patterns: ``grid``, ``circle``, ``line``.
        """
        half = self.arena_size / 2.0
        spacing = self.default_spacing
        poses: List[Pose] = []

        if pattern == 'grid':
            cols = int(math.ceil(math.sqrt(count)))
            rows = int(math.ceil(count / float(cols)))
            # centre the grid
            x_off = -(cols - 1) * spacing / 2.0
            y_off = -(rows - 1) * spacing / 2.0
            for i in range(count):
                r = i // cols
                c = i % cols
                x = x_off + c * spacing
                y = y_off + r * spacing
                # clamp to arena
                x = max(-half + 0.5, min(half - 0.5, x))
                y = max(-half + 0.5, min(half - 0.5, y))
                pose = Pose()
                pose.position = Point(x=x, y=y, z=0.0)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                poses.append(pose)

        elif pattern == 'circle':
            radius = max(spacing, (count * spacing) / (2.0 * math.pi))
            radius = min(radius, half - 0.5)  # keep inside arena
            for i in range(count):
                angle = 2.0 * math.pi * i / count
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                # orient towards centre
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, angle + math.pi)
                pose = Pose()
                pose.position = Point(x=x, y=y, z=0.0)
                pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                poses.append(pose)

        elif pattern == 'line':
            total_len = (count - 1) * spacing if count > 1 else 0.0
            start_x = -total_len / 2.0
            start_x = max(-half + 0.5, start_x)
            for i in range(count):
                x = start_x + i * spacing
                x = max(-half + 0.5, min(half - 0.5, x))
                pose = Pose()
                pose.position = Point(x=x, y=0.0, z=0.0)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                poses.append(pose)
        else:
            rospy.logwarn("Unknown pattern '%s', falling back to grid.", pattern)
            return self._generate_positions(count, 'grid')

        return poses

    # ------------------------------------------------------------------
    # Spawn / delete primitives
    # ------------------------------------------------------------------

    def _allocate_name(self) -> str:
        """Return the next ``tb3_N`` name and bump the counter."""
        name = f"tb3_{self._next_index}"
        self._next_index += 1
        return name

    def spawn_single_robot(self, robot_name: str, pose: Pose) -> bool:
        """
        Spawn one TurtleBot3 Waffle in Gazebo.

        Steps
        -----
        1. Set ``/{robot_name}/robot_description`` param.
        2. Call ``/gazebo/spawn_urdf_model``.
        3. Launch ``robot_state_publisher`` in the robot namespace.

        Returns True on success.
        """
        if not self._robot_description_xml:
            rospy.logerr("No URDF available – cannot spawn %s.", robot_name)
            return False

        # 1. Upload robot_description to param server
        rospy.set_param(f'/{robot_name}/robot_description', self._robot_description_xml)

        # 2. Call Gazebo spawn service
        try:
            req = SpawnModelRequest()
            req.model_name = robot_name
            req.model_xml = self._robot_description_xml
            req.robot_namespace = robot_name
            req.initial_pose = pose
            req.reference_frame = 'world'
            resp = self._spawn_srv(req)

            if not resp.success:
                rospy.logerr("Gazebo refused to spawn %s: %s", robot_name, resp.status_message)
                return False
        except rospy.ServiceException as exc:
            rospy.logerr("Spawn service call failed for %s: %s", robot_name, exc)
            return False

        # 3. Launch robot_state_publisher
        proc = self._launch_state_publisher(robot_name)

        # 4. Book-keeping
        record = RobotRecord(robot_name, pose)
        record.state_pub_proc = proc
        with self._lock:
            self.robots[robot_name] = record

        rospy.loginfo("Spawned robot '%s' at (%.2f, %.2f).",
                       robot_name, pose.position.x, pose.position.y)
        return True

    def _launch_state_publisher(self, robot_name: str) -> Optional[subprocess.Popen]:
        """Launch ``robot_state_publisher`` in the given namespace."""
        try:
            proc = subprocess.Popen(
                [
                    'rosrun', 'robot_state_publisher', 'robot_state_publisher',
                    f'__ns:={robot_name}',
                    f'_tf_prefix:={robot_name}',
                    f'robot_description:=/{robot_name}/robot_description',
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            rospy.loginfo("Launched robot_state_publisher for '%s' (pid %d).",
                           robot_name, proc.pid)
            return proc
        except Exception as exc:
            rospy.logerr("Failed to launch robot_state_publisher for '%s': %s",
                          robot_name, exc)
            return None

    def delete_single_robot(self, robot_name: str) -> bool:
        """
        Remove a robot from Gazebo.

        Steps
        -----
        1. Call ``/gazebo/delete_model``.
        2. Kill associated ``robot_state_publisher`` process.
        3. Delete ``/{robot_name}/robot_description`` from param server.
        4. Remove internal tracking record.

        Returns True on success.
        """
        # 1. Delete from Gazebo
        try:
            req = DeleteModelRequest()
            req.model_name = robot_name
            resp = self._delete_srv(req)
            if not resp.success:
                rospy.logwarn("Gazebo delete for '%s': %s", robot_name, resp.status_message)
        except rospy.ServiceException as exc:
            rospy.logerr("Delete service call failed for '%s': %s", robot_name, exc)
            return False

        with self._lock:
            record = self.robots.pop(robot_name, None)

        if record is None:
            rospy.logwarn("Robot '%s' was not tracked internally.", robot_name)
            return True  # still consider it a success in Gazebo

        # 2. Kill robot_state_publisher
        if record.state_pub_proc is not None:
            try:
                record.state_pub_proc.send_signal(signal.SIGINT)
                record.state_pub_proc.wait(timeout=5)
            except Exception:
                record.state_pub_proc.kill()
            rospy.loginfo("Killed robot_state_publisher for '%s'.", robot_name)

        # 3. Clean up param server
        try:
            if rospy.has_param(f'/{robot_name}/robot_description'):
                rospy.delete_param(f'/{robot_name}/robot_description')
            if rospy.has_param(f'/{robot_name}'):
                rospy.delete_param(f'/{robot_name}')
        except Exception as exc:
            rospy.logwarn("Param cleanup for '%s' failed: %s", robot_name, exc)

        rospy.loginfo("Deleted robot '%s'.", robot_name)
        return True

    # ------------------------------------------------------------------
    # High-level spawn / delete
    # ------------------------------------------------------------------

    def spawn_robots(self, count: int, pattern: str = 'grid') -> List[str]:
        """
        Spawn *count* robots using the given pattern.

        Returns list of successfully spawned robot names.
        """
        with self._lock:
            current = len(self.robots)
        if current + count > self.max_robots:
            rospy.logwarn("Cannot spawn %d robots (current %d, max %d).",
                           count, current, self.max_robots)
            count = self.max_robots - current
            if count <= 0:
                return []

        poses = self._generate_positions(count, pattern)
        spawned: List[str] = []
        for pose in poses:
            name = self._allocate_name()
            if self.spawn_single_robot(name, pose):
                spawned.append(name)
        self._publish_robot_list(None)
        return spawned

    def delete_robots(self, robot_ids: Optional[List[str]] = None) -> int:
        """
        Delete specified robots, or all if *robot_ids* is ``None`` / empty.

        Returns the number of successfully deleted robots.
        """
        if not robot_ids:
            with self._lock:
                robot_ids = list(self.robots.keys())

        deleted = 0
        for rid in robot_ids:
            if self.delete_single_robot(rid):
                deleted += 1
        self._publish_robot_list(None)
        return deleted

    # ------------------------------------------------------------------
    # Topic-based command interface
    # ------------------------------------------------------------------

    def _on_spawn_command(self, msg: String):
        """
        Handle ``/fleet/spawn_command`` messages.

        Expected JSON payload::

            {"count": 5, "pattern": "grid"}

        Publishes result on ``/fleet/spawn_result``.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            result = {"success": False, "error": f"Invalid JSON: {exc}"}
            self.spawn_result_pub.publish(String(data=json.dumps(result)))
            return

        count = int(data.get('count', self.default_count))
        pattern = str(data.get('pattern', self.default_pattern))

        rospy.loginfo("Spawn command received: count=%d, pattern='%s'", count, pattern)
        spawned = self.spawn_robots(count, pattern)

        result = {
            "success": len(spawned) > 0,
            "requested": count,
            "spawned": len(spawned),
            "robot_ids": spawned,
        }
        self.spawn_result_pub.publish(String(data=json.dumps(result)))

    def _on_delete_command(self, msg: String):
        """
        Handle ``/fleet/delete_command`` messages.

        Expected JSON payload::

            {"robot_ids": ["tb3_0", "tb3_2"]}   # specific
            {"all": true}                         # delete all

        Publishes result on ``/fleet/delete_result``.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            result = {"success": False, "error": f"Invalid JSON: {exc}"}
            self.delete_result_pub.publish(String(data=json.dumps(result)))
            return

        if data.get('all', False):
            ids = None  # means "all"
        else:
            ids = data.get('robot_ids', None)

        rospy.loginfo("Delete command received: ids=%s", ids if ids else "ALL")
        deleted = self.delete_robots(ids)

        result = {
            "success": deleted > 0,
            "deleted_count": deleted,
        }
        self.delete_result_pub.publish(String(data=json.dumps(result)))

    # ------------------------------------------------------------------
    # Model-state tracking (Gazebo ground truth)
    # ------------------------------------------------------------------

    def _on_model_states(self, msg: ModelStates):
        """
        Update tracked robot poses from ``/gazebo/model_states``.
        Also auto-detect externally-spawned robots with the ``tb3_`` prefix.
        """
        with self._lock:
            # Update poses for already-tracked robots
            for robot_name, record in self.robots.items():
                try:
                    idx = msg.name.index(robot_name)
                    record.pose = msg.pose[idx]
                except (ValueError, IndexError):
                    pass

            # Auto-detect new tb3_* models spawned externally
            for i, model_name in enumerate(msg.name):
                if model_name.startswith('tb3_') and model_name not in self.robots:
                    rospy.loginfo("Auto-detected externally spawned robot: %s", model_name)
                    self.robots[model_name] = RobotRecord(
                        name=model_name, pose=msg.pose[i]
                    )
                    # Update next_index to avoid naming collisions
                    try:
                        idx_num = int(model_name.split('_')[1])
                        self._next_index = max(self._next_index, idx_num + 1)
                    except (IndexError, ValueError):
                        pass

    # ------------------------------------------------------------------
    # Fleet roster publishing
    # ------------------------------------------------------------------

    def _publish_robot_list(self, _event):
        """Publish comma-separated list of active robot namespaces."""
        with self._lock:
            names = sorted(self.robots.keys())
        roster = ','.join(names)
        self.robot_list_pub.publish(String(data=roster))

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def get_robot_names(self) -> List[str]:
        """Return sorted list of active robot namespaces."""
        with self._lock:
            return sorted(self.robots.keys())

    def get_robot_poses(self) -> List[Tuple[str, Pose]]:
        """Return list of ``(name, Pose)`` for all tracked robots."""
        with self._lock:
            return [(n, r.pose) for n, r in self.robots.items()]

    def get_robot_positions(self) -> List[Tuple[str, float, float]]:
        """Return list of ``(name, x, y)`` for all tracked robots."""
        with self._lock:
            return [
                (n, r.pose.position.x, r.pose.position.y)
                for n, r in self.robots.items()
            ]

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown(self):
        """Clean up child processes on node shutdown."""
        rospy.loginfo("Fleet Manager shutting down – cleaning up ...")
        with self._lock:
            for name, record in self.robots.items():
                if record.state_pub_proc is not None:
                    try:
                        record.state_pub_proc.send_signal(signal.SIGINT)
                        record.state_pub_proc.wait(timeout=3)
                    except Exception:
                        record.state_pub_proc.kill()
                    rospy.loginfo("Stopped robot_state_publisher for '%s'.", name)
        rospy.loginfo("Fleet Manager shutdown complete.")


# ======================================================================
# Main entry point
# ======================================================================

if __name__ == '__main__':
    try:
        fm = FleetManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
