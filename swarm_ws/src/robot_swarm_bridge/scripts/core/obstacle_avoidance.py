#!/usr/bin/env python3
"""
Obstacle Avoidance Module for TurtleBot3 Burger swarms.

Smooth potential-field avoidance with:
  - Cosine proximity scaling (gradual deceleration, no sudden stops)
  - Gentle repulsive steering vectors
  - Inter-robot repulsion with wide reaction zone
  - Exponential velocity smoothing between frames
  - Emergency stop only as a last-resort safety net

Can be **imported** by behaviour scripts::

    from core.obstacle_avoidance import ObstacleAvoidance
    oa = ObstacleAvoidance('tb3_0')
    safe_cmd = oa.apply_avoidance(desired_twist)

Or run **standalone** as a ROS node that publishes threat level and
RViz markers for a single robot.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple, Union

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Sector layout (8 sectors, 45 deg each, starting from the front)
# ---------------------------------------------------------------------------

SECTOR_NAMES = [
    'front',        # 0:   -22.5 ..  22.5 deg
    'front_left',   # 1:    22.5 ..  67.5
    'left',         # 2:    67.5 .. 112.5
    'back_left',    # 3:   112.5 .. 157.5
    'back',         # 4:   157.5 .. 202.5
    'back_right',   # 5:   202.5 .. 247.5
    'right',        # 6:   247.5 .. 292.5
    'front_right',  # 7:   292.5 .. 337.5
]

NUM_SECTORS = len(SECTOR_NAMES)
SECTOR_WIDTH_RAD = 2.0 * math.pi / NUM_SECTORS  # 45 deg


@dataclass(frozen=True)
class LidarRangeMask:
    """One known object whose own LiDAR surface may be ignored.

    A scan return is normally removed only when it falls close to the surface
    predicted for its individual beam. ``occludes_behind`` is reserved for a
    deliberately contacted object: a farther return on the same intersecting
    ray is hidden behind that known object, while a closer return stays visible
    to the safety layer.

    ``half_width`` and ``half_height`` select an oriented box.  Without them,
    ``radius`` keeps the original circular-mask behaviour.
    """

    center_x: float
    center_y: float
    radius: float
    maximum_center_distance: float
    tolerance: float = 0.08
    closer_tolerance: float = 0.01
    half_width: Optional[float] = None
    half_height: Optional[float] = None
    yaw: float = 0.0
    occludes_behind: bool = False
    accept_sensor_floor: bool = False


LidarMaskInput = Optional[
    Union[LidarRangeMask, Iterable[LidarRangeMask]]
]


def _normalise_lidar_masks(masks: LidarMaskInput) -> Tuple[LidarRangeMask, ...]:
    """Accept one mask or a collection without complicating call sites."""
    if masks is None:
        return ()
    if isinstance(masks, LidarRangeMask):
        return (masks,)
    try:
        return tuple(
            mask for mask in masks
            if isinstance(mask, LidarRangeMask)
        )
    except TypeError:
        return ()


def _smooth_factor(d, inner, outer):
    """
    Cosine-based smooth interpolation from 0.0 (at inner) to 1.0 (at outer).

    Uses a half-cosine curve so the transition has zero derivative at both
    endpoints — no sudden jumps in velocity or acceleration.

        t = clamp((d - inner) / (outer - inner), 0, 1)
        return 0.5 * (1 - cos(pi * t))

    At d <= inner: returns 0.0  (full effect / full stop)
    At d >= outer: returns 1.0  (no effect / full speed)
    In between:    smooth S-curve
    """
    if d <= inner:
        return 0.0
    if d >= outer:
        return 1.0
    t = (d - inner) / (outer - inner)
    return 0.5 * (1.0 - math.cos(math.pi * t))


def _normalise_sector_indices(
    sector_indices: Optional[Iterable[int]],
) -> Set[int]:
    """Return valid LiDAR sector indices, ignoring malformed input."""
    normalised: Set[int] = set()
    if sector_indices is None:
        return normalised

    for value in sector_indices:
        try:
            index = int(value)
        except (TypeError, ValueError):
            continue
        if 0 <= index < NUM_SECTORS:
            normalised.add(index)
    return normalised


# ---------------------------------------------------------------------------
# ObstacleAvoidance class
# ---------------------------------------------------------------------------

class ObstacleAvoidance:
    """
    Smooth obstacle avoidance that can be used as a library or standalone.

    Instead of steep 1/d² repulsion that barely reacts then slams, this uses:
      1. **Proximity scaling**: multiplies desired speed by a smooth S-curve
         that gradually reduces velocity as the robot approaches obstacles.
      2. **Gentle steering**: soft repulsive vectors to steer around obstacles
         rather than stop in front of them.
      3. **Velocity smoothing**: exponential moving average between frames
         for natural acceleration/deceleration.
      4. **Emergency stop**: only triggers at very close range as a safety net.
    """

    def __init__(self, robot_name: str):
        self.robot_name: str = robot_name

        # ----- tuneable parameters (from param server) --------------------
        # Awareness radius: robot starts gently decelerating here
        self.awareness_radius: float = rospy.get_param('~awareness_radius', 0.8)
        # Safety radius: significant deceleration zone
        self.safety_radius: float = rospy.get_param('~safety_radius', 0.4)
        # Emergency radius: hard stop (last resort)
        self.emergency_radius: float = rospy.get_param('~emergency_radius', 0.20)
        # Steering gain (how strongly to steer away)
        self.steer_gain: float = rospy.get_param('~steer_gain', 0.8)
        # Robot physical radius
        self.robot_radius: float = rospy.get_param('~robot_radius', 0.11)
        # Inter-robot interaction distance
        self.robot_interact_dist: float = rospy.get_param('~robot_interact_dist', 0.6)
        self.robot_emergency_dist: float = rospy.get_param(
            '~robot_emergency_dist', 0.28
        )
        self.shielded_chain_collapse_dist: float = rospy.get_param(
            '~shielded_chain_collapse_dist', 0.24
        )
        self.allowed_contact_min_dist: float = rospy.get_param(
            '~allowed_contact_min_dist', 0.13
        )
        self.parallel_lane_clearance: float = rospy.get_param(
            '~parallel_lane_clearance', self.robot_emergency_dist + 0.01
        )
        self.robot_lidar_tolerance: float = rospy.get_param(
            '~robot_lidar_tolerance', 0.03
        )
        self.robot_lidar_closer_tolerance: float = rospy.get_param(
            '~robot_lidar_closer_tolerance', 0.01
        )
        self.collision_distance: float = rospy.get_param(
            '~collision_distance', self.robot_radius + 0.015
        )
        self.robot_collision_distance: float = rospy.get_param(
            '~robot_collision_distance', 2.0 * self.robot_radius + 0.01
        )
        self.collision_release_margin: float = rospy.get_param(
            '~collision_release_margin', 0.03
        )
        self.max_linear_velocity: float = rospy.get_param(
            '~max_linear_velocity', 0.22
        )
        self.max_angular_velocity: float = rospy.get_param(
            '~max_angular_velocity', 2.84
        )
        self.max_linear_acceleration: float = rospy.get_param(
            '~max_linear_acceleration', 0.5
        )
        self.max_angular_acceleration: float = rospy.get_param(
            '~max_angular_acceleration', 3.0
        )
        self.control_dt: float = rospy.get_param('~control_dt', 0.05)
        self.scan_timeout_wall_s: float = max(
            0.2, float(rospy.get_param('~scan_timeout_wall_s', 0.75))
        )

        # Velocity smoothing factor (0 = no smoothing, 1 = frozen)
        # 0.4 at 20Hz gives a ~80ms time constant — responsive but smooth
        self.smoothing_alpha: float = rospy.get_param('~smoothing_alpha', 0.4)

        # Lidar filtering
        self.min_valid_range: float = rospy.get_param('~min_valid_range', 0.05)
        self.max_valid_range: float = rospy.get_param('~max_valid_range', 3.5)
        # TurtleBot3 Burger's LDS is mounted 32 mm behind base_link.  Mask
        # geometry must originate at the laser, while motion safety and robot
        # positions continue to use the base centre.
        self.lidar_offset_x: float = rospy.get_param('~lidar_offset_x', -0.032)
        self.lidar_offset_y: float = rospy.get_param('~lidar_offset_y', 0.0)

        # ----- internal state --------------------------------------------
        self.latest_scan: Optional[LaserScan] = None
        self.scan_ranges: List[float] = []
        self._scan_lock = threading.Lock()
        self._clock = time.monotonic
        self._scan_received_at: Optional[float] = None

        # Sector minimum distances
        self.sector_min: List[float] = [self.max_valid_range] * NUM_SECTORS

        # Own position
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._pos_theta: float = 0.0
        self._pos_lock = threading.Lock()

        # Velocity smoothing state
        self._prev_linear: float = 0.0
        self._prev_angular: float = 0.0
        self._collision_active: bool = False

        # Other robot positions (set by update_robot_positions)
        self._other_positions: List[Tuple[float, float]] = []
        self._other_positions_by_name: Dict[str, Tuple[float, float]] = {}

        # ----- ROS subscriber for scan ------------------------------------
        self._scan_sub = rospy.Subscriber(
            f'/{robot_name}/scan',
            LaserScan,
            self._scan_callback,
            queue_size=1,
        )
        # Behavior nodes use ObstacleAvoidance as an embedded helper.  Keeping
        # the publisher here makes their filtered safety result observable on
        # the topic the orchestrator already consumes, without a second OA
        # process (and a second controller) for every robot.
        self._threat_pub = rospy.Publisher(
            f'/{robot_name}/threat_level', Float32, queue_size=1
        )
        self._collision_pub = rospy.Publisher(
            f'/{robot_name}/collision_state', Bool, queue_size=1
        )

        rospy.loginfo(
            "[%s] ObstacleAvoidance initialised "
            "(awareness=%.2fm, safety=%.2fm, emergency=%.2fm, "
            "steer=%.1f, robot_r=%.2fm, smoothing=%.2f)",
            self.robot_name, self.awareness_radius, self.safety_radius,
            self.emergency_radius, self.steer_gain, self.robot_radius,
            self.smoothing_alpha,
        )

    def shutdown(self) -> None:
        """Release ROS resources owned by this helper."""
        scan_sub = self._scan_sub
        self._scan_sub = None
        if scan_sub is not None:
            scan_sub.unregister()

        threat_pub = self._threat_pub
        self._threat_pub = None
        if threat_pub is not None:
            threat_pub.unregister()

        collision_pub = self._collision_pub
        self._collision_pub = None
        if collision_pub is not None:
            collision_pub.unregister()

    # ------------------------------------------------------------------
    # Position setter
    # ------------------------------------------------------------------

    def set_position(self, x: float, y: float, theta: float) -> None:
        with self._pos_lock:
            self._pos_x = x
            self._pos_y = y
            self._pos_theta = theta

    # ------------------------------------------------------------------
    # Scan callback
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan) -> None:
        """Process incoming LaserScan into 8 named sectors."""
        with self._scan_lock:
            num_readings = len(msg.ranges)
            if num_readings == 0:
                return

            self.latest_scan = msg
            self.scan_ranges = list(msg.ranges)
            self._scan_received_at = self._clock()

            sector_min = [self.max_valid_range] * NUM_SECTORS

            for i, r in enumerate(msg.ranges):
                if not (self.min_valid_range <= r <= self.max_valid_range):
                    continue
                if math.isnan(r) or math.isinf(r):
                    continue

                angle = msg.angle_min + i * msg.angle_increment
                angle = angle % (2.0 * math.pi)

                sector_idx = int(
                    ((angle + SECTOR_WIDTH_RAD / 2.0) % (2.0 * math.pi))
                    / SECTOR_WIDTH_RAD
                ) % NUM_SECTORS

                if r < sector_min[sector_idx]:
                    sector_min[sector_idx] = r

            self.sector_min = sector_min

    def scan_is_fresh(self, now: Optional[float] = None) -> bool:
        """Return whether a real LiDAR sample arrived recently enough to move."""
        with self._scan_lock:
            received_at = self._scan_received_at
        if received_at is None:
            return False
        if now is None:
            now = self._clock()
        age = now - received_at
        return 0.0 <= age <= self.scan_timeout_wall_s

    @staticmethod
    def _sector_index(angle: float) -> int:
        """Map a robot-local beam angle to the eight-sector layout."""
        wrapped = angle % (2.0 * math.pi)
        return int(
            ((wrapped + SECTOR_WIDTH_RAD / 2.0) % (2.0 * math.pi))
            / SECTOR_WIDTH_RAD
        ) % NUM_SECTORS

    def _surface_range_for_mask(
        self,
        beam_angle: float,
        mask: LidarRangeMask,
        lidar_pose: Tuple[float, float, float],
    ) -> Optional[float]:
        """Return the predicted surface range for one known object."""
        lidar_x, lidar_y, robot_yaw = lidar_pose
        offset_x = mask.center_x - lidar_x
        offset_y = mask.center_y - lidar_y
        center_distance = math.hypot(offset_x, offset_y)
        if (
            center_distance > max(0.0, mask.maximum_center_distance)
        ):
            return None

        ray_angle = robot_yaw + beam_angle
        direction_x = math.cos(ray_angle)
        direction_y = math.sin(ray_angle)
        if (
            mask.half_width is not None
            and mask.half_height is not None
            and mask.half_width > 0.0
            and mask.half_height > 0.0
        ):
            surface_range = self._box_surface_range(
                lidar_x,
                lidar_y,
                direction_x,
                direction_y,
                mask,
            )
        else:
            if mask.radius <= 0.0:
                return None
            projection = offset_x * direction_x + offset_y * direction_y
            if projection <= 0.0:
                return None

            perpendicular_sq = center_distance ** 2 - projection ** 2
            radius_sq = mask.radius ** 2
            if perpendicular_sq > radius_sq:
                return None

            surface_range = projection - math.sqrt(
                max(0.0, radius_sq - perpendicular_sq)
            )

        if surface_range is None or surface_range <= 0.0:
            return None
        return surface_range

    def _range_matches_mask(
        self,
        measured_range: float,
        beam_angle: float,
        mask: LidarRangeMask,
        lidar_pose: Tuple[float, float, float],
        sensor_min_range: float,
    ) -> bool:
        """Return whether one beam agrees with the masked object's surface."""
        surface_range = self._surface_range_for_mask(
            beam_angle, mask, lidar_pose
        )
        if surface_range is None:
            return False

        if (
            mask.accept_sensor_floor
            and sensor_min_range - 0.002
            <= measured_range
            <= sensor_min_range + 0.002
        ):
            return True

        # Burger's LDS cannot represent a payload surface below range_min.
        # Depending on the Gazebo ray, it reports either the sensor floor or
        # a background surface behind the invisible payload.  Both are safe
        # to discard because the known payload geometrically occludes that
        # beam first.  A sub-floor value, if a driver does provide one, stays
        # visible to avoidance as a distinguishable closer obstacle.
        if surface_range < sensor_min_range:
            return measured_range >= sensor_min_range - 0.002

        if measured_range < (
            surface_range - max(0.0, mask.closer_tolerance)
        ):
            return False
        if mask.occludes_behind:
            return True
        return measured_range <= (
            surface_range + max(0.0, mask.tolerance)
        )

    @staticmethod
    def _box_surface_range(
        lidar_x: float,
        lidar_y: float,
        direction_x: float,
        direction_y: float,
        mask: LidarRangeMask,
    ) -> Optional[float]:
        """Intersect a world-frame LiDAR ray with an oriented rectangle."""
        cos_yaw = math.cos(mask.yaw)
        sin_yaw = math.sin(mask.yaw)

        relative_x = lidar_x - mask.center_x
        relative_y = lidar_y - mask.center_y
        origin_x = relative_x * cos_yaw + relative_y * sin_yaw
        origin_y = -relative_x * sin_yaw + relative_y * cos_yaw
        local_dx = direction_x * cos_yaw + direction_y * sin_yaw
        local_dy = -direction_x * sin_yaw + direction_y * cos_yaw

        entry = -float('inf')
        exit_distance = float('inf')
        for origin, direction, half_size in (
            (origin_x, local_dx, float(mask.half_width)),
            (origin_y, local_dy, float(mask.half_height)),
        ):
            if abs(direction) < 1e-9:
                if abs(origin) > half_size:
                    return None
                continue

            near = (-half_size - origin) / direction
            far = (half_size - origin) / direction
            if near > far:
                near, far = far, near
            entry = max(entry, near)
            exit_distance = min(exit_distance, far)
            if entry > exit_distance:
                return None

        if exit_distance <= 0.0:
            return None
        return entry if entry > 0.0 else exit_distance

    def _sector_distances(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> List[float]:
        """Return sector minima after applying any beam-level object mask."""
        ignored = _normalise_sector_indices(ignored_sector_indices)
        with self._scan_lock:
            fallback = list(self.sector_min)
            scan = self.latest_scan
            ranges = list(self.scan_ranges)

        allowed_contact_mask = self._allowed_contact_lidar_mask(
            allowed_contact_position
        )
        range_masks = list(_normalise_lidar_masks(lidar_range_mask))
        if allowed_contact_mask is not None:
            range_masks.append(allowed_contact_mask)
        if not range_masks or scan is None or not ranges:
            return [
                self.max_valid_range if index in ignored else distance
                for index, distance in enumerate(fallback)
            ]

        with self._pos_lock:
            cos_yaw = math.cos(self._pos_theta)
            sin_yaw = math.sin(self._pos_theta)
            lidar_pose = (
                self._pos_x
                + self.lidar_offset_x * cos_yaw
                - self.lidar_offset_y * sin_yaw,
                self._pos_y
                + self.lidar_offset_x * sin_yaw
                + self.lidar_offset_y * cos_yaw,
                self._pos_theta,
            )

        sensor_min_range = float(getattr(scan, 'range_min', 0.0))
        if not math.isfinite(sensor_min_range) or sensor_min_range <= 0.0:
            sensor_min_range = self.min_valid_range

        distances = [self.max_valid_range] * NUM_SECTORS
        sector_hits = [None] * NUM_SECTORS
        for index, measured_range in enumerate(ranges):
            if (
                math.isnan(measured_range)
                or math.isinf(measured_range)
                or not (
                    self.min_valid_range
                    <= measured_range
                    <= self.max_valid_range
                )
            ):
                continue

            beam_angle = scan.angle_min + index * scan.angle_increment
            sector = self._sector_index(beam_angle)
            if sector in ignored:
                continue
            if any(
                self._range_matches_mask(
                    measured_range,
                    beam_angle,
                    mask,
                    lidar_pose,
                    sensor_min_range,
                )
                for mask in range_masks
            ):
                continue
            if measured_range < distances[sector]:
                distances[sector] = measured_range
                sector_hits[sector] = (measured_range, beam_angle)

        self._last_filtered_sector_hits = sector_hits
        return distances

    # ------------------------------------------------------------------
    # Core: proximity speed scale
    # ------------------------------------------------------------------

    def _proximity_speed_scale(
        self,
        linear_velocity: float,
        angular_velocity: float = 0.0,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> float:
        """
        Compute a [0, 1] multiplier for the desired speed based on the
        closest obstacle distance.

        Uses a two-stage smooth cosine curve:
          - awareness_radius → safety_radius: gentle slowdown (1.0 → 0.4)
          - safety_radius → emergency_radius: strong slowdown  (0.4 → 0.0)

        The result is a smooth S-curve across the full range with no
        discontinuities in velocity or acceleration.
        """
        min_dist = self._minimum_distance_for_motion(
            linear_velocity,
            angular_velocity,
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        )

        if min_dist >= self.awareness_radius:
            return 1.0

        if min_dist <= self.emergency_radius:
            return 0.0

        if min_dist >= self.safety_radius:
            # Outer zone: gentle slowdown from 1.0 to 0.4
            t = _smooth_factor(min_dist, self.safety_radius, self.awareness_radius)
            return 0.4 + 0.6 * t
        else:
            # Inner zone: strong slowdown from 0.4 to 0.0
            t = _smooth_factor(min_dist, self.emergency_radius, self.safety_radius)
            return 0.4 * t

    # ------------------------------------------------------------------
    # Core: steering repulsion
    # ------------------------------------------------------------------

    def _obstacle_steering(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> Tuple[float, float]:
        """
        Compute a gentle steering vector that nudges the robot away from
        obstacles. Uses smooth cosine weighting instead of 1/d².

        The vector points away from nearby obstacles with magnitude
        proportional to how deep into the safety zone the obstacle is.
        """
        steer = np.array([0.0, 0.0])

        distances = self._sector_distances(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        )
        for idx, dist in enumerate(distances):
            if dist >= self.awareness_radius:
                continue

            sector_angle = idx * SECTOR_WIDTH_RAD
            away_angle = sector_angle + math.pi

            # Smooth weight: 0 at awareness_radius, 1 at emergency_radius
            weight = 1.0 - _smooth_factor(
                dist, self.emergency_radius, self.awareness_radius
            )

            steer[0] += self.steer_gain * weight * math.cos(away_angle)
            steer[1] += self.steer_gain * weight * math.sin(away_angle)

        # A perfectly centred obstacle has no lateral component, which used
        # to leave a robot stopped nose-to-wall indefinitely. Pick the more
        # open side, with a stable robot-specific tie break, so it can pass
        # without twitching between left and right on consecutive scans.
        front_distance = distances[0]
        if front_distance < self.awareness_radius and abs(steer[1]) < 0.05:
            left_clearance = min(distances[1], distances[2])
            right_clearance = min(distances[7], distances[6])
            if abs(left_clearance - right_clearance) > 0.03:
                turn_direction = 1.0 if left_clearance > right_clearance else -1.0
            else:
                turn_direction = (
                    1.0
                    if sum(ord(char) for char in self.robot_name) % 2 == 0
                    else -1.0
                )
            front_weight = 1.0 - _smooth_factor(
                front_distance,
                self.emergency_radius,
                self.awareness_radius,
            )
            steer[1] += self.steer_gain * 0.65 * front_weight * turn_direction

        # Soft cap
        mag = np.linalg.norm(steer)
        if mag > self.steer_gain * 2.0:
            steer = steer / mag * self.steer_gain * 2.0

        return (float(steer[0]), float(steer[1]))

    # ------------------------------------------------------------------
    # Core: inter-robot repulsion
    # ------------------------------------------------------------------

    def _robot_repulsion(
        self, other_positions: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Smooth repulsion from neighbouring robots.

        Uses cosine profile over [robot_radius .. robot_interact_dist] so
        the force ramps up gradually as robots approach each other.
        """
        rep = np.array([0.0, 0.0])

        with self._pos_lock:
            my_x = self._pos_x
            my_y = self._pos_y
            my_theta = self._pos_theta

        for (ox, oy) in other_positions:
            dx = my_x - ox
            dy = my_y - oy
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= self.robot_interact_dist or dist < 1e-4:
                continue

            # Unit vector away from the other robot (world frame)
            ux = dx / dist
            uy = dy / dist

            # Smooth weight: 0 at interact_dist, peaks at robot_radius
            weight = 1.0 - _smooth_factor(
                dist, self.robot_radius, self.robot_interact_dist
            )
            magnitude = self.steer_gain * 1.5 * weight

            # Convert world-frame repulsion to robot-local frame
            cos_t = math.cos(-my_theta)
            sin_t = math.sin(-my_theta)
            local_x = ux * cos_t - uy * sin_t
            local_y = ux * sin_t + uy * cos_t

            rep[0] += magnitude * local_x
            rep[1] += magnitude * local_y

        # Soft cap
        mag = np.linalg.norm(rep)
        if mag > self.steer_gain * 2.0:
            rep = rep / mag * self.steer_gain * 2.0

        return (float(rep[0]), float(rep[1]))

    def _robot_motion_scale(
        self,
        linear_velocity: float,
        other_positions: Optional[List[Tuple[float, float]]] = None,
        parallel_motion_exempt_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
        shielded_motion_exempt_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
        guarded_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
    ) -> float:
        """Slow only for neighbours lying in the current travel direction."""
        if abs(linear_velocity) < 1e-3:
            return 1.0

        with self._pos_lock:
            my_x = self._pos_x
            my_y = self._pos_y
            my_theta = self._pos_theta

        closest = self.robot_interact_dist
        cos_t = math.cos(-my_theta)
        sin_t = math.sin(-my_theta)
        neighbours = (
            self._other_positions
            if other_positions is None
            else other_positions
        )
        parallel_peers = {
            (float(x), float(y))
            for x, y in parallel_motion_exempt_positions or ()
        }
        shielded_peers = {
            (float(x), float(y))
            for x, y in shielded_motion_exempt_positions or ()
        }
        guarded_contacts = {}
        for x, y in guarded_contact_positions or ():
            position = (float(x), float(y))
            guarded_contacts[position] = (
                guarded_contacts.get(position, 0) + 1
            )
        guarded_scale = 1.0
        for other_x, other_y in neighbours:
            dx = other_x - my_x
            dy = other_y - my_y
            local_x = dx * cos_t - dy * sin_t
            local_y = dx * sin_t + dy * cos_t
            distance = math.hypot(local_x, local_y)

            if (
                (float(other_x), float(other_y)) in parallel_peers
                and abs(local_y) >= self.parallel_lane_clearance
            ):
                # A slightly staggered robot in a known parallel lane is not
                # a forward obstacle. If the lateral clearance collapses, it
                # returns to the normal predictive and hard-stop path.
                continue

            moving_towards = (
                local_x > 0.0 if linear_velocity > 0.0 else local_x < 0.0
            )

            position = (float(other_x), float(other_y))
            contact_count = guarded_contacts.get(position, 0)
            if contact_count:
                guarded_contacts[position] = contact_count - 1
                if moving_towards:
                    guarded_scale = min(
                        guarded_scale,
                        self._minimum_distance_motion_scale(
                            distance,
                            linear_velocity,
                            self.allowed_contact_min_dist,
                            stop_if_overspeed=False,
                        ),
                    )
                continue

            if position in shielded_peers:
                # The adjacent parent physically blocks this same-lane robot.
                # Keep a predictive folded-chain guard, but do not apply the
                # wider generic braking distance to its grandparent.
                if moving_towards:
                    guarded_scale = min(
                        guarded_scale,
                        self._minimum_distance_motion_scale(
                            distance,
                            linear_velocity,
                            self.shielded_chain_collapse_dist,
                        ),
                    )
                continue

            if not moving_towards:
                continue

            if distance < closest:
                closest = distance

        # Leave enough room to decelerate before the configured hard center
        # distance.  The extra control-period term covers one scan/command
        # cycle of latency in Gazebo and becomes zero for a settled robot.
        closing_speed = max(abs(linear_velocity), abs(self._prev_linear))
        braking_distance = (
            closing_speed ** 2
            / (2.0 * max(0.01, self.max_linear_acceleration))
            + closing_speed * max(0.0, self.control_dt)
        )
        stop_distance = min(
            self.robot_interact_dist - 0.01,
            self.robot_emergency_dist + braking_distance,
        )
        return min(
            guarded_scale,
            _smooth_factor(
                closest,
                stop_distance,
                self.robot_interact_dist,
            ),
        )

    def _minimum_distance_motion_scale(
        self,
        distance: float,
        requested_velocity: float,
        minimum_distance: float,
        stop_if_overspeed: bool = True,
    ) -> float:
        """Limit closing speed so the next command cannot cross a guard."""
        clearance = distance - minimum_distance
        if clearance <= 0.0:
            return 0.0

        acceleration = max(0.01, self.max_linear_acceleration)
        control_dt = max(0.0, self.control_dt)
        safe_speed = (
            math.sqrt(
                (acceleration * control_dt) ** 2
                + 2.0 * acceleration * clearance
            )
            - acceleration * control_dt
        )

        # If the previous command already carries too much closing speed,
        # stop this cycle and clear the smoother before trying again.
        if (
            stop_if_overspeed
            and abs(self._prev_linear) > safe_speed + 1e-9
        ):
            return 0.0

        requested_speed = abs(requested_velocity)
        if requested_speed <= 1e-9:
            return 1.0
        return min(1.0, safe_speed / requested_speed)

    def _is_robot_emergency(
        self,
        linear_velocity: float,
        other_positions: Optional[List[Tuple[float, float]]] = None,
    ) -> bool:
        """Check explicit fleet positions even when LiDAR sectors are masked."""
        if abs(linear_velocity) < 1e-3:
            return False
        return self._robot_motion_scale(
            linear_velocity, other_positions
        ) == 0.0

    # ------------------------------------------------------------------
    # Core: compute_avoidance_velocity
    # ------------------------------------------------------------------

    def compute_avoidance_velocity(
        self,
        desired_vx: float,
        desired_vy: float,
        other_robot_positions: Optional[List[Tuple[float, float]]] = None,
        desired_angular: float = 0.0,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        repulsion_robot_positions: Optional[List[Tuple[float, float]]] = None,
        parallel_motion_exempt_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
        shielded_motion_exempt_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
        guarded_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
        soft_steering: bool = True,
    ) -> Tuple[float, float]:
        """
        Compute a modified velocity that smoothly avoids obstacles and robots.

        1. Scale desired speed by proximity (gradual deceleration)
        2. Add gentle steering away from obstacles
        3. Add inter-robot repulsion
        """
        ignored_sectors = _normalise_sector_indices(
            ignored_sector_indices
        )
        safety_positions = self._positions_without_allowed_contact(
            other_robot_positions, allowed_contact_position
        )
        repulsion_positions = (
            safety_positions
            if repulsion_robot_positions is None
            else self._positions_without_allowed_contact(
                repulsion_robot_positions, allowed_contact_position
            )
        )

        # Emergency stop (last resort safety net)
        lidar_clearance = self._minimum_distance_for_motion(
            desired_vx,
            desired_angular,
            ignored_sectors,
            lidar_range_mask,
            allowed_contact_position,
        )
        robot_motion_scale = self._robot_motion_scale(
            desired_vx,
            safety_positions,
            parallel_motion_exempt_positions,
            shielded_motion_exempt_positions,
            guarded_contact_positions,
        )
        lidar_emergency = lidar_clearance <= self.emergency_radius
        robot_emergency = robot_motion_scale == 0.0
        if lidar_emergency or robot_emergency:
            sector_distances = self._sector_distances(
                ignored_sectors,
                lidar_range_mask,
                allowed_contact_position,
            )
            if abs(desired_vx) < 1e-3:
                motion_sectors = range(NUM_SECTORS)
            elif desired_vx > 0.0:
                motion_sectors = (7, 0, 1, 6, 2)
            else:
                motion_sectors = (3, 4, 5, 2, 6)
            nearest_sector = min(
                motion_sectors,
                key=sector_distances.__getitem__,
            )
            with self._pos_lock:
                my_x = self._pos_x
                my_y = self._pos_y
                my_yaw = self._pos_theta
            nearest_robot = min(
                (
                    math.hypot(x - my_x, y - my_y)
                    for x, y in safety_positions
                ),
                default=self.robot_interact_dist,
            )
            hit = getattr(
                self, '_last_filtered_sector_hits', [None] * NUM_SECTORS
            )[nearest_sector]
            mask_surfaces = []
            if hit is not None:
                _, beam_angle = hit
                cos_yaw = math.cos(my_yaw)
                sin_yaw = math.sin(my_yaw)
                lidar_pose = (
                    my_x
                    + self.lidar_offset_x * cos_yaw
                    - self.lidar_offset_y * sin_yaw,
                    my_y
                    + self.lidar_offset_x * sin_yaw
                    + self.lidar_offset_y * cos_yaw,
                    my_yaw,
                )
                for mask in _normalise_lidar_masks(lidar_range_mask):
                    surface = self._surface_range_for_mask(
                        beam_angle, mask, lidar_pose
                    )
                    if surface is not None:
                        mask_surfaces.append(round(surface, 3))
            rospy.logwarn_throttle(
                2.0,
                "[%s] EMERGENCY STOP (lidar=%.3fm sector=%s, "
                "nearest_robot=%.3fm, robot_brake=%s, masks=%s)",
                self.robot_name, lidar_clearance,
                SECTOR_NAMES[nearest_sector], nearest_robot,
                robot_emergency, mask_surfaces,
            )
            return (0.0, 0.0)

        # 1. Scale desired speed smoothly based on proximity
        speed_scale = self._proximity_speed_scale(
            desired_vx,
            desired_angular,
            ignored_sectors,
            lidar_range_mask,
            allowed_contact_position,
        )
        speed_scale = min(
            speed_scale,
            self._robot_motion_scale(
                desired_vx,
                safety_positions,
                parallel_motion_exempt_positions,
                shielded_motion_exempt_positions,
                guarded_contact_positions,
            ),
        )
        mod_vx = desired_vx * speed_scale
        mod_vy = desired_vy * speed_scale

        if soft_steering:
            # 2. Add obstacle steering
            _steer_x, steer_y = self._obstacle_steering(
                ignored_sectors,
                lidar_range_mask,
                allowed_contact_position,
            )

            # 3. Add inter-robot repulsion
            robot_steer_y = 0.0
            if repulsion_positions:
                _robot_x, robot_steer_y = self._robot_repulsion(
                    repulsion_positions
                )

            # Steering should shape an intended movement, not create movement
            # of its own. Fading it with speed lets a robot settle accurately
            # into a close formation slot while hard distance layers remain.
            motion_intent = min(
                1.0,
                abs(desired_vx) / max(0.01, self.max_linear_velocity),
            )
            mod_vy += (steer_y + robot_steer_y) * motion_intent

        return (mod_vx, mod_vy)

    # ------------------------------------------------------------------
    # Threat level
    # ------------------------------------------------------------------

    @staticmethod
    def _threat_from_distance(
        distance: float, emergency_distance: float, awareness_distance: float
    ) -> float:
        if distance >= awareness_distance:
            return 0.0
        if distance <= emergency_distance:
            return 1.0
        return 1.0 - _smooth_factor(
            distance, emergency_distance, awareness_distance
        )

    @staticmethod
    def _allowed_contact_index(
        positions: List[Tuple[float, float]],
        allowed_contact_position: Optional[Tuple[float, float]],
    ) -> Optional[int]:
        """Find one fleet entry that precisely matches the contact request."""
        if allowed_contact_position is None:
            return None

        try:
            allowed_x = float(allowed_contact_position[0])
            allowed_y = float(allowed_contact_position[1])
        except (IndexError, TypeError, ValueError):
            return None
        if not math.isfinite(allowed_x) or not math.isfinite(allowed_y):
            return None

        for index, (other_x, other_y) in enumerate(positions):
            if (
                math.isclose(
                    float(other_x), allowed_x, rel_tol=0.0, abs_tol=1e-9
                )
                and math.isclose(
                    float(other_y), allowed_y, rel_tol=0.0, abs_tol=1e-9
                )
            ):
                return index
        return None

    @staticmethod
    def _positions_without_allowed_contact(
        positions: Optional[List[Tuple[float, float]]],
        allowed_contact_position: Optional[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """Return neighbours except for one explicitly allowed contact.

        The controller passes a position taken from the same fleet snapshot
        used to update this helper.  Match it precisely and remove at most one
        entry so a nearby (or even co-located) second robot stays protected.
        Malformed and stale positions simply leave normal safety unchanged.
        """
        return ObstacleAvoidance._positions_without_allowed_contacts(
            positions, (allowed_contact_position,)
        )

    @staticmethod
    def _positions_without_allowed_contacts(
        positions: Optional[List[Tuple[float, float]]],
        allowed_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ],
    ) -> List[Tuple[float, float]]:
        """Remove each explicitly declared contact once from a fleet view."""
        neighbours = list(positions or [])
        for position in allowed_contact_positions or ():
            index = ObstacleAvoidance._allowed_contact_index(
                neighbours, position
            )
            if index is not None:
                del neighbours[index]
        return neighbours

    def _allowed_contact_lidar_mask(
        self,
        allowed_contact_position: Optional[Tuple[float, float]],
    ) -> Optional[LidarRangeMask]:
        """Describe only the designated companion's visible round surface."""
        neighbours = list(getattr(self, '_other_positions', []))
        index = self._allowed_contact_index(
            neighbours, allowed_contact_position
        )
        if index is None:
            return None

        center_x, center_y = neighbours[index]
        lidar_mount_distance = math.hypot(
            self.lidar_offset_x, self.lidar_offset_y
        )
        return LidarRangeMask(
            center_x=float(center_x),
            center_y=float(center_y),
            radius=max(0.0, self.robot_radius),
            maximum_center_distance=(
                max(0.0, self.robot_interact_dist) + lidar_mount_distance
            ),
            tolerance=max(0.0, self.robot_lidar_tolerance),
            closer_tolerance=max(
                0.0, self.robot_lidar_closer_tolerance
            ),
            occludes_behind=True,
        )

    def _nearest_robot_distance(
        self,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        allowed_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
    ) -> float:
        with self._pos_lock:
            my_x = self._pos_x
            my_y = self._pos_y
        contacts = list(allowed_contact_positions or ())
        if allowed_contact_position is not None:
            contacts.append(allowed_contact_position)
        neighbours = self._positions_without_allowed_contacts(
            self._other_positions, contacts
        )
        return min(
            (
                math.hypot(other_x - my_x, other_y - my_y)
                for other_x, other_y in neighbours
            ),
            default=self.robot_interact_dist,
        )

    def get_threat_level(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        allowed_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
    ) -> float:
        """
        Return a scalar threat level in [0.0, 1.0].
        Smooth cosine profile matching both avoidance safety layers.
        """
        if not self.scan_is_fresh():
            return 1.0
        lidar_threat = self._threat_from_distance(
            self._overall_min_distance(
                ignored_sector_indices,
                lidar_range_mask,
                allowed_contact_position,
            ),
            self.emergency_radius,
            self.awareness_radius,
        )
        robot_threat = self._threat_from_distance(
            self._nearest_robot_distance(
                allowed_contact_position, allowed_contact_positions
            ),
            self.robot_emergency_dist,
            self.robot_interact_dist,
        )
        return max(lidar_threat, robot_threat)

    def _update_collision_state(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        allowed_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
    ) -> bool:
        """Track physical-contact episodes with a small release hysteresis."""
        release_margin = (
            max(0.0, self.collision_release_margin)
            if self._collision_active
            else 0.0
        )
        lidar_contact = self._overall_min_distance(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        ) <= self.collision_distance + release_margin
        robot_contact = self._nearest_robot_distance(
            allowed_contact_position, allowed_contact_positions
        ) <= (
            self.robot_collision_distance + release_margin
        )
        self._collision_active = lidar_contact or robot_contact
        return self._collision_active

    def publish_safety_state(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        allowed_contact_positions: Optional[
            Iterable[Tuple[float, float]]
        ] = None,
    ) -> float:
        """Publish filtered threat and physical-contact state together."""
        threat = self.get_threat_level(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
            allowed_contact_positions,
        )
        collision = self._update_collision_state(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
            allowed_contact_positions,
        )
        if self._threat_pub is not None:
            self._threat_pub.publish(Float32(data=threat))
        if self._collision_pub is not None:
            self._collision_pub.publish(Bool(data=collision))
        return threat

    def publish_threat_level(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> float:
        """Backward-compatible name for publishing the full safety state."""
        return self.publish_safety_state(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        )

    # ------------------------------------------------------------------
    # Convenience wrappers (used by behaviour scripts)
    # ------------------------------------------------------------------

    def update_robot_positions(
        self, positions: List[Tuple[str, 'Point']]
    ) -> None:
        """
        Store world-frame positions of all robots. Called once per tick.
        """
        others: List[Tuple[float, float]] = []
        others_by_name: Dict[str, Tuple[float, float]] = {}

        for name, pt in positions:
            if name == self.robot_name:
                with self._pos_lock:
                    self._pos_x = pt.x
                    self._pos_y = pt.y
            else:
                position = (pt.x, pt.y)
                others.append(position)
                others_by_name[name] = position

        self._other_positions = others
        self._other_positions_by_name = others_by_name

    def reset_motion(self) -> None:
        """Clear smoothing when a higher-level controller asks for a hold."""
        self._prev_linear = 0.0
        self._prev_angular = 0.0

    def commit_published_command(self, command: 'Twist') -> None:
        """Keep smoothing state equal to the command that reached the robot."""
        try:
            linear = float(command.linear.x)
            angular = float(command.angular.z)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.reset_motion()
            return
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.reset_motion()
            return
        self._prev_linear = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, linear),
        )
        self._prev_angular = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, angular),
        )

    def apply_avoidance(
        self,
        cmd: 'Twist',
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
        allowed_contact_namespace: Optional[str] = None,
        allowed_contact_namespaces: Optional[Iterable[str]] = None,
        repulsion_exempt_namespaces: Optional[Iterable[str]] = None,
        parallel_motion_exempt_namespaces: Optional[
            Iterable[str]
        ] = None,
        shielded_motion_exempt_namespaces: Optional[
            Iterable[str]
        ] = None,
        soft_steering: bool = True,
        minimum_linear_speed: float = 0.0,
    ) -> 'Twist':
        """
        High-level wrapper: takes a desired Twist, returns a safe Twist
        with smooth obstacle avoidance and velocity smoothing applied.
        """
        if not self.scan_is_fresh():
            self.reset_motion()
            self.publish_safety_state()
            return Twist()

        others = list(getattr(self, '_other_positions', []))
        positions_by_name = getattr(self, '_other_positions_by_name', {})
        contact_names = set()
        if allowed_contact_namespace:
            current_contact_position = positions_by_name.get(
                allowed_contact_namespace
            )
            if current_contact_position is not None:
                allowed_contact_position = current_contact_position
                contact_names.add(allowed_contact_namespace)

        allowed_positions = []
        if self._allowed_contact_index(
            others, allowed_contact_position
        ) is not None:
            allowed_positions.append(allowed_contact_position)
        for namespace in allowed_contact_namespaces or ():
            if namespace in contact_names:
                continue
            position = positions_by_name.get(namespace)
            if position is None:
                continue
            contact_names.add(namespace)
            allowed_positions.append(position)

        safety_others = self._positions_without_allowed_contacts(
            others, allowed_positions
        )
        motion_others = list(safety_others)
        motion_others.extend(allowed_positions)
        repulsion_others = list(safety_others)
        for namespace in repulsion_exempt_namespaces or ():
            position = positions_by_name.get(namespace)
            if position is not None:
                repulsion_others = self._positions_without_allowed_contact(
                    repulsion_others, position
                )
        parallel_motion_positions = [
            positions_by_name[namespace]
            for namespace in parallel_motion_exempt_namespaces or ()
            if namespace in positions_by_name
        ]
        shielded_motion_positions = [
            positions_by_name[namespace]
            for namespace in shielded_motion_exempt_namespaces or ()
            if namespace in positions_by_name
        ]
        ignored_sectors = _normalise_sector_indices(
            ignored_sector_indices
        )
        active_lidar_masks = list(
            _normalise_lidar_masks(lidar_range_mask)
        )
        for position in allowed_positions:
            contact_mask = self._allowed_contact_lidar_mask(position)
            if contact_mask is not None:
                active_lidar_masks.append(contact_mask)
        active_lidar_masks = tuple(active_lidar_masks)

        mod_vx, mod_vy = self.compute_avoidance_velocity(
            cmd.linear.x,
            0.0,
            motion_others,
            desired_angular=cmd.angular.z,
            ignored_sector_indices=ignored_sectors,
            lidar_range_mask=active_lidar_masks,
            repulsion_robot_positions=repulsion_others,
            parallel_motion_exempt_positions=parallel_motion_positions,
            shielded_motion_exempt_positions=shielded_motion_positions,
            guarded_contact_positions=allowed_positions,
            soft_steering=soft_steering,
        )
        guarded_motion_scale = 1.0
        if allowed_positions:
            # Evaluate the declared contacts on their own so the final
            # smoothing stage knows whether the close-contact speed guard is
            # the layer limiting this command.  Reusing the normal robot
            # motion calculation also keeps parallel-lane exemptions exact.
            guarded_motion_scale = self._robot_motion_scale(
                cmd.linear.x,
                allowed_positions,
                parallel_motion_positions,
                (),
                allowed_positions,
            )
        self.publish_safety_state(
            ignored_sectors,
            active_lidar_masks,
            allowed_contact_positions=allowed_positions,
        )

        safe = Twist()
        if (
            mod_vx == 0.0
            and mod_vy == 0.0
            and (
                self._is_emergency(
                    cmd.linear.x,
                    cmd.angular.z,
                    ignored_sectors,
                    active_lidar_masks,
                )
                or self._robot_motion_scale(
                    cmd.linear.x,
                    motion_others,
                    parallel_motion_positions,
                    shielded_motion_positions,
                    allowed_positions,
                ) == 0.0
            )
        ):
            # Emergency stop — reset smoothing state so we don't drift
            self._prev_linear = 0.0
            self._prev_angular = 0.0
            return safe

        # Compute raw output
        raw_linear = mod_vx
        raw_angular = cmd.angular.z + mod_vy * 1.2  # lateral → steering

        # Exponential smoothing for natural acceleration/deceleration
        alpha = self.smoothing_alpha
        smooth_linear = alpha * self._prev_linear + (1.0 - alpha) * raw_linear
        smooth_angular = alpha * self._prev_angular + (1.0 - alpha) * raw_angular

        # A loaded transport chain can opt into a small useful-pressure floor.
        # This sits after every LiDAR and robot safety calculation, so it only
        # bypasses comfort smoothing when the raw safe velocity itself permits
        # that speed.  The acceleration limiter below remains authoritative.
        try:
            requested_floor = max(0.0, float(minimum_linear_speed))
        except (TypeError, ValueError, OverflowError):
            requested_floor = 0.0
        if (
            requested_floor > 0.0
            and raw_linear * cmd.linear.x > 0.0
            and smooth_linear * raw_linear >= 0.0
            and abs(raw_linear) >= requested_floor
            and abs(smooth_linear) < requested_floor
        ):
            smooth_linear = math.copysign(
                requested_floor, raw_linear
            )

        # Acceleration limiting removes the final abrupt command changes that
        # an exponential average alone can still produce.
        linear_delta = self.max_linear_acceleration * self.control_dt
        angular_delta = self.max_angular_acceleration * self.control_dt
        limited_linear = self._prev_linear + max(
            -linear_delta, min(linear_delta, smooth_linear - self._prev_linear)
        )
        limited_angular = self._prev_angular + max(
            -angular_delta, min(angular_delta, smooth_angular - self._prev_angular)
        )

        if (
            0.0 < guarded_motion_scale < 1.0
            and limited_linear * cmd.linear.x > 0.0
            and abs(limited_linear) > abs(raw_linear)
        ):
            # Comfort smoothing may decelerate more slowly than the guarded
            # contact geometry allows.  It must never reintroduce a faster
            # closing command than the safe velocity calculated above.
            limited_linear = raw_linear

        safe.linear.x = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, limited_linear),
        )
        safe.angular.z = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, limited_angular),
        )
        self._prev_linear = safe.linear.x
        self._prev_angular = safe.angular.z
        return safe

    def compute_threat_level(self) -> float:
        """Alias for get_threat_level."""
        return self.get_threat_level()

    def compute_repulsion_force(self) -> 'Point':
        """Return the current obstacle steering as a Point (local frame)."""
        rx, ry = self._obstacle_steering()
        return Point(x=rx, y=ry, z=0.0)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _overall_min_distance(
        self,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> float:
        distances = self._sector_distances(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        )
        return min(distances) if distances else self.max_valid_range

    def _minimum_distance_for_motion(
        self,
        linear_velocity: float,
        angular_velocity: float = 0.0,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> float:
        """Return clearance in the translated half of the Burger footprint.

        Burger's footprint is circular, so rotating in place does not reduce
        clearance.  For a combined turn and translation, only the half swept
        by the translation can get closer.  This lets a robot turn or drive
        away from a wall that is already just behind it without weakening
        front and side emergency protection.
        """
        if abs(linear_velocity) < 1e-3:
            if abs(angular_velocity) > 1e-3:
                return self.max_valid_range
            indices = range(NUM_SECTORS)
        elif linear_velocity > 0.0:
            indices = (7, 0, 1)
        else:
            indices = (3, 4, 5)

        sector_distances = self._sector_distances(
            ignored_sector_indices,
            lidar_range_mask,
            allowed_contact_position,
        )
        distances = [sector_distances[index] for index in indices]
        return min(distances) if distances else self.max_valid_range

    def _is_emergency(
        self,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        ignored_sector_indices: Optional[Iterable[int]] = None,
        lidar_range_mask: LidarMaskInput = None,
        allowed_contact_position: Optional[Tuple[float, float]] = None,
    ) -> bool:
        return (
            self._minimum_distance_for_motion(
                linear_velocity,
                angular_velocity,
                ignored_sector_indices,
                lidar_range_mask,
                allowed_contact_position,
            )
            <= self.emergency_radius
        )

    # ------------------------------------------------------------------
    # Visualisation helpers (used by standalone mode)
    # ------------------------------------------------------------------

    def create_avoidance_markers(self) -> MarkerArray:
        ma = MarkerArray()

        with self._pos_lock:
            px, py = self._pos_x, self._pos_y

        threat = self.get_threat_level()

        # --- threat-ring marker ---
        ring = Marker()
        ring.header.frame_id = 'map'
        ring.header.stamp = rospy.Time.now()
        ring.ns = f'{self.robot_name}_threat'
        ring.id = 0
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position = Point(x=px, y=py, z=0.01)
        ring.pose.orientation.w = 1.0
        ring.scale.x = 2.0 * self.awareness_radius
        ring.scale.y = 2.0 * self.awareness_radius
        ring.scale.z = 0.02
        ring.color.r = min(1.0, 2.0 * threat)
        ring.color.g = min(1.0, 2.0 * (1.0 - threat))
        ring.color.b = 0.0
        ring.color.a = 0.25
        ring.lifetime = rospy.Duration(0.5)
        ma.markers.append(ring)

        # --- steering arrow ---
        steer_x, steer_y = self._obstacle_steering()
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = rospy.Time.now()
        arrow.ns = f'{self.robot_name}_steer'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start = Point(x=px, y=py, z=0.15)
        end = Point(x=px + steer_x * 0.4, y=py + steer_y * 0.4, z=0.15)
        arrow.points = [start, end]
        arrow.scale.x = 0.04
        arrow.scale.y = 0.08
        arrow.scale.z = 0.0
        arrow.color.r = 0.2
        arrow.color.g = 0.4
        arrow.color.b = 1.0
        arrow.color.a = 0.9
        arrow.lifetime = rospy.Duration(0.5)
        ma.markers.append(arrow)

        return ma


# ======================================================================
# Standalone node
# ======================================================================

def _standalone_main() -> None:
    rospy.init_node('obstacle_avoidance', anonymous=True)

    robot_name: str = rospy.get_param('~robot_name', 'tb3_0')
    rate_hz: float = rospy.get_param('~rate', 20.0)

    oa = ObstacleAvoidance(robot_name)

    marker_pub = rospy.Publisher(
        f'/{robot_name}/avoidance_markers', MarkerArray, queue_size=1
    )

    from nav_msgs.msg import Odometry
    import tf.transformations as tft

    def _odom_cb(msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        oa.set_position(pos.x, pos.y, yaw)

    rospy.Subscriber(f'/{robot_name}/odom', Odometry, _odom_cb, queue_size=1)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("[%s] Obstacle avoidance standalone node running at %.0f Hz.",
                   robot_name, rate_hz)

    while not rospy.is_shutdown():
        oa.publish_safety_state()
        marker_pub.publish(oa.create_avoidance_markers())
        rate.sleep()


if __name__ == '__main__':
    try:
        _standalone_main()
    except rospy.ROSInterruptException:
        pass
