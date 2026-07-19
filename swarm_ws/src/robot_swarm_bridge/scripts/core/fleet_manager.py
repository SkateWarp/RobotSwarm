#!/usr/bin/env python3
"""
Fleet Manager for TurtleBot3 Burger swarms.
Handles dynamic spawning, deletion, and tracking of TurtleBot3 Burger robots
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
import sys
import threading
from typing import Dict, List, Optional, Tuple

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    DeleteModel,
    DeleteModelRequest,
    GetPhysicsProperties,
)
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
import tf.transformations

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.robot_ids import robot_id_sort_key, validate_robot_ids


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
    Manages a fleet of TurtleBot3 Burger robots in Gazebo.

    Responsibilities
    ----------------
    * Spawn robots via ``/gazebo/spawn_urdf_model`` after xacro processing
    * Optionally launch a ``robot_state_publisher`` per robot (subprocess)
    * Delete robots via ``/gazebo/delete_model``, kill child processes, clean params
    * Generate spawn positions in *grid*, *circle*, or *line* patterns
    * Track live poses from ``/gazebo/model_states``
    * Publish fleet roster on ``/fleet/robot_list`` (latched, comma-separated)
    * Accept spawn / delete commands via topics (JSON encoded)
    """

    def __init__(self):
        rospy.init_node('fleet_manager', anonymous=False)

        # ----- configuration via rospy params -----
        # Zero means that the worker/arena capacity profile, not this node,
        # decides the maximum fleet size.
        self.max_robots: int = rospy.get_param('~max_robots', 0)
        self.default_count: int = rospy.get_param('~robot_count', 5)
        self.default_pattern: str = rospy.get_param('~spawn_pattern', 'grid')
        self.default_spacing: float = rospy.get_param('~spawn_spacing', 0.6)
        self.arena_size: float = rospy.get_param('~arena_size', 10.0)  # meters, centered at origin
        self.robot_model: str = rospy.get_param('~robot_model', 'burger')
        self.minimum_spawn_spacing: float = rospy.get_param(
            '~minimum_spawn_spacing', 0.35
        )
        self.arena_margin: float = rospy.get_param('~arena_margin', 0.35)
        self.arena_profile: str = rospy.get_param(
            '~arena_profile', 'swarm_arena'
        )
        # Motion is already ramped by each behaviour's safety controller.
        # Gazebo Classic's second acceleration integrator can retain an old
        # wheel speed after a command changes, especially at 3x real time.
        self.gazebo_wheel_acceleration: float = max(
            0.0, float(rospy.get_param('~gazebo_wheel_acceleration', 0.0))
        )
        # Standalone users keep the historic TF behaviour.  The main swarm
        # launch disables it because Gazebo already supplies the state used by
        # our controllers, and one publisher per robot is costly at scale.
        self.publish_robot_tf: bool = rospy.get_param(
            '~publish_robot_tf', True
        )
        self.spawn_obstacle_clearance: float = max(
            0.0, rospy.get_param('~spawn_obstacle_clearance', 0.30)
        )
        self.spawn_search_step: float = max(
            0.05, rospy.get_param('~spawn_search_step', 0.10)
        )
        self.spawn_exclusion_zones: List[Dict] = rospy.get_param(
            '~spawn_exclusion_zones', []
        )
        self.auto_spawn: bool = rospy.get_param('~auto_spawn', False)
        if self.publish_robot_tf:
            rospy.loginfo("Per-robot TF publishing is enabled.")
        else:
            rospy.loginfo(
                "Per-robot TF publishing is disabled; Gazebo state remains available."
            )

        # ----- internal state -----
        self.robots: Dict[str, RobotRecord] = {}
        self._lock = threading.Lock()
        self._spawn_lock = threading.Lock()
        self._next_index: int = 0  # monotonically increasing robot index
        self._retired_robot_ids = set()
        self._model_poses: Dict[str, Tuple[float, float, float]] = {}

        # ----- prepare URDF template once -----
        self._robot_description_xml: str = self._process_urdf()

        # ----- Gazebo service proxies -----
        rospy.loginfo("Waiting for Gazebo spawn/delete services ...")
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=60.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=60.0)
        rospy.wait_for_service('/gazebo/get_physics_properties', timeout=60.0)
        rospy.wait_for_service('/gazebo/pause_physics', timeout=60.0)
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=60.0)
        self._spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self._delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self._get_physics_srv = rospy.ServiceProxy(
            '/gazebo/get_physics_properties', GetPhysicsProperties
        )
        self._pause_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
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
        """Process the configured TurtleBot3 xacro into raw URDF XML."""
        try:
            rospack = rospkg.RosPack()
            tb3_desc = rospack.get_path('turtlebot3_description')
            urdf_path = os.path.join(
                tb3_desc, 'urdf', f'turtlebot3_{self.robot_model}.urdf.xacro'
            )
            rospy.loginfo("Processing xacro: %s", urdf_path)
            doc = xacro.process_file(urdf_path)
            self._configure_drive_plugin(doc)
            robot_description = doc.toxml()
            rospy.loginfo("URDF processed successfully (%d bytes).", len(robot_description))
            return robot_description
        except Exception as exc:
            rospy.logerr("Failed to process TurtleBot3 xacro: %s", exc)
            return ""

    def _configure_drive_plugin(self, document) -> None:
        """Apply the simulator-specific wheel limit to the processed URDF."""
        elements = document.getElementsByTagName('wheelAcceleration')
        if not elements:
            raise ValueError(
                'TurtleBot3 URDF has no wheelAcceleration setting'
            )

        value = '{:g}'.format(self.gazebo_wheel_acceleration)
        for element in elements:
            if element.firstChild is None:
                element.appendChild(document.createTextNode(value))
            else:
                element.firstChild.nodeValue = value

        rospy.loginfo(
            "Gazebo wheel acceleration set to %sm/s^2", value
        )

    # ------------------------------------------------------------------
    # Spawn patterns
    # ------------------------------------------------------------------

    def _generate_positions(self, count: int, pattern: str) -> List[Pose]:
        """
        Generate *count* spawn poses inside the arena (``arena_size`` x ``arena_size``
        centred at the origin).

        Supported patterns: ``grid``, ``circle``, ``line``.
        """
        if count <= 0:
            return []

        half = self.arena_size / 2.0
        spacing = max(self.default_spacing, self.minimum_spawn_spacing)
        usable_half = half - self.arena_margin
        if usable_half <= 0.0:
            rospy.logerr(
                "Arena %.2fm is too small for margin %.2fm.",
                self.arena_size, self.arena_margin,
            )
            return []
        poses: List[Pose] = []

        if pattern == 'grid':
            cols = int(math.ceil(math.sqrt(count)))
            rows = int(math.ceil(count / float(cols)))
            if ((cols - 1) * spacing > 2.0 * usable_half or
                    (rows - 1) * spacing > 2.0 * usable_half):
                rospy.logerr(
                    "A %dx%d grid with %.2fm spacing does not fit in the %.2fm arena.",
                    cols, rows, spacing, self.arena_size,
                )
                return []
            # centre the grid
            x_off = -(cols - 1) * spacing / 2.0
            y_off = -(rows - 1) * spacing / 2.0
            for i in range(count):
                r = i // cols
                c = i % cols
                x = x_off + c * spacing
                y = y_off + r * spacing
                pose = Pose()
                pose.position = Point(x=x, y=y, z=0.0)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                poses.append(pose)

        elif pattern == 'circle':
            if count == 1:
                radius = 0.0
            else:
                radius = spacing / (2.0 * math.sin(math.pi / count))
            if radius > usable_half:
                rospy.logerr(
                    "A %d-robot circle with %.2fm spacing needs %.2fm radius; "
                    "only %.2fm is available.",
                    count, spacing, radius, usable_half,
                )
                return []
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
            if total_len > 2.0 * usable_half:
                rospy.logerr(
                    "A %d-robot line with %.2fm spacing needs %.2fm; only %.2fm "
                    "is available.",
                    count, spacing, total_len, 2.0 * usable_half,
                )
                return []
            start_x = -total_len / 2.0
            for i in range(count):
                x = start_x + i * spacing
                pose = Pose()
                pose.position = Point(x=x, y=0.0, z=0.0)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                poses.append(pose)
        else:
            rospy.logwarn("Unknown pattern '%s', falling back to grid.", pattern)
            return self._generate_positions(count, 'grid')

        return poses

    def _generate_available_positions(
        self,
        count: int,
        pattern: str,
        existing_positions: List[Tuple[float, float]],
    ) -> List[Pose]:
        """Place a complete pattern clear of robots and arena obstacles."""
        poses = self._generate_positions(count, pattern)
        if not poses:
            return []

        return self._translate_pattern_to_clear_space(
            poses, existing_positions
        )

    def _translate_pattern_to_clear_space(
        self,
        poses: List[Pose],
        existing_positions: List[Tuple[float, float]],
    ) -> List[Pose]:
        """
        Find the smallest safe translation for a requested pattern.

        Moving the whole pattern keeps its spacing and shape intact.  The
        search is deterministic so replacing a fleet gives the same initial
        layout every time.
        """
        if not poses:
            return []

        for dx, dy in self._translation_candidates(poses):
            points = [
                (pose.position.x + dx, pose.position.y + dy)
                for pose in poses
            ]
            if not all(
                self._spawn_point_is_clear(x, y, existing_positions)
                for x, y in points
            ):
                continue

            translated: List[Pose] = []
            for pose, (x, y) in zip(poses, points):
                moved = Pose()
                moved.position = Point(x=x, y=y, z=pose.position.z)
                moved.orientation = Quaternion(
                    x=pose.orientation.x,
                    y=pose.orientation.y,
                    z=pose.orientation.z,
                    w=pose.orientation.w,
                )
                translated.append(moved)
            return translated

        rospy.logerr(
            "No obstacle-free location fits this %d-robot pattern in the arena.",
            len(poses),
        )
        return []

    def _translation_candidates(
        self, poses: List[Pose]
    ) -> List[Tuple[float, float]]:
        """Return in-bounds translations, nearest to the origin first."""
        usable_half = self.arena_size / 2.0 - self.arena_margin
        min_x = min(pose.position.x for pose in poses)
        max_x = max(pose.position.x for pose in poses)
        min_y = min(pose.position.y for pose in poses)
        max_y = max(pose.position.y for pose in poses)

        min_dx = -usable_half - min_x
        max_dx = usable_half - max_x
        min_dy = -usable_half - min_y
        max_dy = usable_half - max_y
        if min_dx > max_dx or min_dy > max_dy:
            return []

        step = self.spawn_search_step
        x_start = int(math.ceil((min_dx - 1e-9) / step))
        x_end = int(math.floor((max_dx + 1e-9) / step))
        y_start = int(math.ceil((min_dy - 1e-9) / step))
        y_end = int(math.floor((max_dy + 1e-9) / step))

        candidates = [
            (round(ix * step, 10), round(iy * step, 10))
            for ix in range(x_start, x_end + 1)
            for iy in range(y_start, y_end + 1)
        ]
        return sorted(candidates, key=self._translation_sort_key)

    @staticmethod
    def _translation_sort_key(offset: Tuple[float, float]):
        """Prefer a short move, then a vertical and positive move on ties."""
        dx, dy = offset
        return (
            round(dx * dx + dy * dy, 12),
            abs(dx) + abs(dy),
            abs(dx),
            0 if dy >= 0.0 else 1,
            0 if dx >= 0.0 else 1,
        )

    def _spawn_point_is_clear(
        self,
        x: float,
        y: float,
        existing_positions: List[Tuple[float, float]],
    ) -> bool:
        """Check arena bounds, live robots, and configured collision zones."""
        usable_half = self.arena_size / 2.0 - self.arena_margin
        if abs(x) > usable_half + 1e-9 or abs(y) > usable_half + 1e-9:
            return False

        robot_clearance = max(
            self.default_spacing, self.minimum_spawn_spacing
        )
        if any(
            math.hypot(x - other_x, y - other_y)
            < robot_clearance - 1e-9
            for other_x, other_y in existing_positions
        ):
            return False

        for zone in self._active_exclusion_zones():
            padding = max(0.0, float(zone.get('padding', 0.0)))
            required = self.spawn_obstacle_clearance + padding
            if self._clearance_from_zone(x, y, zone) < required - 1e-9:
                return False

        return True

    def _active_exclusion_zones(self):
        """Yield zones that belong to the selected arena profile."""
        for zone in self.spawn_exclusion_zones:
            if not isinstance(zone, dict):
                continue
            worlds = zone.get('worlds')
            if isinstance(worlds, str):
                worlds = [worlds]
            if worlds and self.arena_profile not in worlds:
                continue
            if zone.get('shape') in ('box', 'circle'):
                yield zone

    def _clearance_from_zone(
        self, x: float, y: float, zone: Dict
    ) -> float:
        """Return signed distance from a point to one collision footprint."""
        center_x = float(zone.get('x', 0.0))
        center_y = float(zone.get('y', 0.0))
        yaw = float(zone.get('yaw', 0.0))

        model_name = zone.get('model')
        if model_name in self._model_poses:
            center_x, center_y, yaw = self._model_poses[model_name]
            yaw += float(zone.get('yaw_offset', 0.0))

        if zone.get('shape') == 'circle':
            radius = max(0.0, float(zone.get('radius', 0.0)))
            return math.hypot(x - center_x, y - center_y) - radius

        half_width = max(0.0, float(zone.get('width', 0.0))) / 2.0
        half_height = max(0.0, float(zone.get('height', 0.0))) / 2.0
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        relative_x = x - center_x
        relative_y = y - center_y
        local_x = cosine * relative_x + sine * relative_y
        local_y = -sine * relative_x + cosine * relative_y

        outside_x = abs(local_x) - half_width
        outside_y = abs(local_y) - half_height
        if outside_x > 0.0 or outside_y > 0.0:
            return math.hypot(max(outside_x, 0.0), max(outside_y, 0.0))

        # A negative value also makes malformed or overlapping placements
        # straightforward to identify in diagnostics and tests.
        return -min(half_width - abs(local_x), half_height - abs(local_y))

    @staticmethod
    def _yaw_from_quaternion(quaternion: Quaternion) -> float:
        """Extract planar yaw without depending on a second tf conversion."""
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    # ------------------------------------------------------------------
    # Spawn / delete primitives
    # ------------------------------------------------------------------

    def _allocate_name(self, occupied_names=None) -> str:
        """Return the next ``tb3_N`` name and bump the counter."""
        occupied = occupied_names or set()
        while True:
            name = f"tb3_{self._next_index}"
            self._next_index += 1
            if name not in occupied:
                return name

    def spawn_single_robot(self, robot_name: str, pose: Pose) -> bool:
        """
        Spawn one TurtleBot3 Burger in Gazebo.

        Steps
        -----
        1. Set ``/{robot_name}/robot_description`` param.
        2. Call ``/gazebo/spawn_urdf_model``.
        3. Launch ``robot_state_publisher`` when TF publishing is enabled.

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

        # 3. TF is optional for simulation-only fleets.  Keeping this choice
        # here also covers robots added later through the command interface.
        proc = None
        if self.publish_robot_tf:
            proc = self._launch_state_publisher(robot_name)

        # 4. Book-keeping
        record = RobotRecord(robot_name, pose)
        record.state_pub_proc = proc
        with self._lock:
            self._retired_robot_ids.discard(robot_name)
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
                return False
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

    def spawn_robots(
        self,
        count: int,
        pattern: str = 'grid',
        robot_ids: Optional[List[str]] = None,
    ) -> List[str]:
        """
        Spawn *count* robots using the given pattern.

        Returns list of successfully spawned robot names.
        """
        count = int(count)
        if count <= 0:
            rospy.logwarn("Robot count must be positive; received %d.", count)
            return []

        requested_names = None
        if robot_ids is not None:
            requested_names = validate_robot_ids(robot_ids)
            if len(requested_names) != count:
                raise ValueError(
                    "robot_ids length ({}) must match count ({})".format(
                        len(requested_names), count
                    )
                )

        with self._spawn_lock:
            with self._lock:
                current = len(self.robots)
                existing_names = set(self.robots.keys())
                existing_positions = [
                    (record.pose.position.x, record.pose.position.y)
                    for record in self.robots.values()
                ]

            if self.max_robots > 0 and current + count > self.max_robots:
                rospy.logwarn(
                    "Cannot spawn %d robots (current %d, max %d).",
                    count, current, self.max_robots,
                )
                return []

            if requested_names is not None:
                conflicts = existing_names.intersection(requested_names)
                if conflicts:
                    raise ValueError(
                        "robot IDs already exist: {}".format(
                            ", ".join(sorted(
                                conflicts, key=robot_id_sort_key
                            ))
                        )
                    )
                planned_names = requested_names
            else:
                occupied_names = set(existing_names)
                planned_names = []
                for _ in range(count):
                    name = self._allocate_name(occupied_names)
                    planned_names.append(name)
                    occupied_names.add(name)

            poses = self._generate_available_positions(
                count, pattern, existing_positions
            )
            if len(poses) != count:
                return []

            spawned: List[str] = []
            for name, pose in zip(planned_names, poses):
                if self.spawn_single_robot(name, pose):
                    spawned.append(name)
                    if requested_names is not None:
                        try:
                            numeric_id = int(name.rsplit('_', 1)[1])
                            self._next_index = max(
                                self._next_index, numeric_id + 1
                            )
                        except (IndexError, ValueError):
                            pass
        self._publish_robot_list(None)
        return spawned

    def delete_robots(self, robot_ids: Optional[List[str]] = None) -> int:
        """
        Delete specified robots, or all if *robot_ids* is ``None`` / empty.

        Returns the number of successfully deleted robots.
        """
        with self._spawn_lock:
            if not robot_ids:
                with self._lock:
                    robot_ids = list(self.robots.keys())
            else:
                robot_ids = list(robot_ids)

            if not robot_ids:
                return 0

            was_paused = self._pause_for_safe_deletion()
            if was_paused is None:
                return 0

            try:
                # Gazebo can deliver old model-state snapshots after deletion.
                # Ignore those IDs until this manager explicitly spawns them again.
                with self._lock:
                    self._retired_robot_ids.update(robot_ids)

                deleted = 0
                for rid in robot_ids:
                    if self.delete_single_robot(rid):
                        deleted += 1
                    else:
                        with self._lock:
                            self._retired_robot_ids.discard(rid)
            finally:
                if not was_paused:
                    self._resume_after_deletion()
        self._publish_robot_list(None)
        return deleted

    def _pause_for_safe_deletion(self) -> Optional[bool]:
        """
        Quiesce Gazebo before removing models and return its prior pause state.

        TurtleBot models own sensor and drive plugins that run on Gazebo's
        update loop.  Removing several live models can race those callbacks and
        stall the server, so deletion is only attempted once physics is paused.
        ``None`` means that the state could not be read or changed safely.
        """
        try:
            physics = self._get_physics_srv()
            if not physics.success:
                rospy.logerr(
                    "Cannot read Gazebo pause state: %s",
                    physics.status_message,
                )
                return None

            was_paused = bool(physics.pause)
            if not was_paused:
                self._pause_srv()
            return was_paused
        except rospy.ServiceException as exc:
            rospy.logerr(
                "Cannot pause Gazebo for safe fleet deletion: %s", exc
            )
            return None

    def _resume_after_deletion(self):
        """Resume a simulation that this manager paused for model removal."""
        try:
            self._unpause_srv()
        except rospy.ServiceException as exc:
            # Keep the server in its safe paused state and make the operational
            # failure visible; never hide an exception from the delete itself.
            rospy.logerr(
                "Fleet deletion finished, but Gazebo could not resume: %s",
                exc,
            )

    # ------------------------------------------------------------------
    # Topic-based command interface
    # ------------------------------------------------------------------

    def _on_spawn_command(self, msg: String):
        """
        Handle ``/fleet/spawn_command`` messages.

        Expected JSON payload::

            {"count": 5, "pattern": "grid", "robot_ids": ["tb3_0", ...]}

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
        robot_ids = data.get('robot_ids')

        rospy.loginfo("Spawn command received: count=%d, pattern='%s'", count, pattern)
        try:
            spawned = self.spawn_robots(count, pattern, robot_ids)
        except (TypeError, ValueError) as exc:
            result = {
                "success": False,
                "requested": count,
                "spawned": 0,
                "robot_ids": [],
                "error": str(exc),
            }
            self.spawn_result_pub.publish(String(data=json.dumps(result)))
            return

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
        model_entries = list(zip(msg.name, msg.pose))
        with self._lock:
            # Pose tracking must stay responsive while a batch spawn/delete
            # owns _spawn_lock. Blocking this high-rate callback can fill the
            # Gazebo publisher socket and, in turn, wedge /delete_model.
            for model_name, pose in model_entries:
                self._model_poses[model_name] = (
                    pose.position.x,
                    pose.position.y,
                    self._yaw_from_quaternion(pose.orientation),
                )
                record = self.robots.get(model_name)
                if record is not None:
                    record.pose = pose

            external_candidates = [
                (model_name, pose)
                for model_name, pose in model_entries
                if (
                    model_name.startswith('tb3_')
                    and model_name not in self.robots
                    and model_name not in self._retired_robot_ids
                )
            ]

        if not external_candidates:
            return

        # Discovery affects the same names and capacity as spawn_robots(). If
        # a fleet transaction is active, skip this snapshot; Gazebo publishes
        # another one immediately and no pose update is worth blocking here.
        if not self._spawn_lock.acquire(blocking=False):
            return
        try:
            with self._lock:
                for model_name, pose in external_candidates:
                    if (
                        model_name in self.robots
                        or model_name in self._retired_robot_ids
                    ):
                        continue
                    rospy.loginfo(
                        "Auto-detected externally spawned robot: %s",
                        model_name,
                    )
                    self.robots[model_name] = RobotRecord(
                        name=model_name, pose=pose
                    )
                    try:
                        index = int(model_name.rsplit('_', 1)[1])
                        self._next_index = max(self._next_index, index + 1)
                    except (IndexError, ValueError):
                        pass
        finally:
            self._spawn_lock.release()

    # ------------------------------------------------------------------
    # Fleet roster publishing
    # ------------------------------------------------------------------

    def _publish_robot_list(self, _event):
        """Publish comma-separated list of active robot namespaces."""
        with self._lock:
            names = sorted(self.robots.keys(), key=robot_id_sort_key)
        roster = ','.join(names)
        self.robot_list_pub.publish(String(data=roster))

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def get_robot_names(self) -> List[str]:
        """Return sorted list of active robot namespaces."""
        with self._lock:
            return sorted(self.robots.keys(), key=robot_id_sort_key)

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
