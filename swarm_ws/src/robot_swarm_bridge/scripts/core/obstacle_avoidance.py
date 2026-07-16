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
from typing import List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
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
        self.emergency_radius: float = rospy.get_param('~emergency_radius', 0.14)
        # Steering gain (how strongly to steer away)
        self.steer_gain: float = rospy.get_param('~steer_gain', 0.8)
        # Robot physical radius
        self.robot_radius: float = rospy.get_param('~robot_radius', 0.11)
        # Inter-robot interaction distance
        self.robot_interact_dist: float = rospy.get_param('~robot_interact_dist', 0.6)
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

        # Velocity smoothing factor (0 = no smoothing, 1 = frozen)
        # 0.4 at 20Hz gives a ~80ms time constant — responsive but smooth
        self.smoothing_alpha: float = rospy.get_param('~smoothing_alpha', 0.4)

        # Lidar filtering
        self.min_valid_range: float = rospy.get_param('~min_valid_range', 0.12)
        self.max_valid_range: float = rospy.get_param('~max_valid_range', 3.5)

        # ----- internal state --------------------------------------------
        self.latest_scan: Optional[LaserScan] = None
        self.scan_ranges: List[float] = []
        self._scan_lock = threading.Lock()

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

        # Other robot positions (set by update_robot_positions)
        self._other_positions: List[Tuple[float, float]] = []

        # ----- ROS subscriber for scan ------------------------------------
        self._scan_sub = rospy.Subscriber(
            f'/{robot_name}/scan',
            LaserScan,
            self._scan_callback,
            queue_size=1,
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
        """Release the scan subscription owned by this helper."""
        scan_sub = self._scan_sub
        self._scan_sub = None
        if scan_sub is not None:
            scan_sub.unregister()

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
            self.latest_scan = msg
            self.scan_ranges = list(msg.ranges)

            num_readings = len(msg.ranges)
            if num_readings == 0:
                return

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

    # ------------------------------------------------------------------
    # Core: proximity speed scale
    # ------------------------------------------------------------------

    def _proximity_speed_scale(
        self, linear_velocity: float, angular_velocity: float = 0.0
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
            linear_velocity, angular_velocity
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

    def _obstacle_steering(self) -> Tuple[float, float]:
        """
        Compute a gentle steering vector that nudges the robot away from
        obstacles. Uses smooth cosine weighting instead of 1/d².

        The vector points away from nearby obstacles with magnitude
        proportional to how deep into the safety zone the obstacle is.
        """
        steer = np.array([0.0, 0.0])

        with self._scan_lock:
            for idx, dist in enumerate(self.sector_min):
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

    # ------------------------------------------------------------------
    # Core: compute_avoidance_velocity
    # ------------------------------------------------------------------

    def compute_avoidance_velocity(
        self,
        desired_vx: float,
        desired_vy: float,
        other_robot_positions: Optional[List[Tuple[float, float]]] = None,
        desired_angular: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Compute a modified velocity that smoothly avoids obstacles and robots.

        1. Scale desired speed by proximity (gradual deceleration)
        2. Add gentle steering away from obstacles
        3. Add inter-robot repulsion
        """
        # Emergency stop (last resort safety net)
        if self._is_emergency(desired_vx, desired_angular):
            rospy.logwarn_throttle(
                2.0, "[%s] EMERGENCY STOP – obstacle within %.2fm!",
                self.robot_name, self.emergency_radius,
            )
            return (0.0, 0.0)

        # 1. Scale desired speed smoothly based on proximity
        speed_scale = self._proximity_speed_scale(
            desired_vx, desired_angular
        )
        mod_vx = desired_vx * speed_scale
        mod_vy = desired_vy * speed_scale

        # 2. Add obstacle steering
        steer_x, steer_y = self._obstacle_steering()
        mod_vx += steer_x
        mod_vy += steer_y

        # 3. Add inter-robot repulsion
        if other_robot_positions:
            rx, ry = self._robot_repulsion(other_robot_positions)
            mod_vx += rx
            mod_vy += ry

        return (mod_vx, mod_vy)

    # ------------------------------------------------------------------
    # Threat level
    # ------------------------------------------------------------------

    def get_threat_level(self) -> float:
        """
        Return a scalar threat level in [0.0, 1.0].
        Smooth cosine profile matching the avoidance behavior.
        """
        min_dist = self._overall_min_distance()

        if min_dist >= self.awareness_radius:
            return 0.0
        if min_dist <= self.emergency_radius:
            return 1.0

        return 1.0 - _smooth_factor(
            min_dist, self.emergency_radius, self.awareness_radius
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

        for name, pt in positions:
            if name == self.robot_name:
                with self._pos_lock:
                    self._pos_x = pt.x
                    self._pos_y = pt.y
            else:
                others.append((pt.x, pt.y))

        self._other_positions = others

    def apply_avoidance(self, cmd: 'Twist') -> 'Twist':
        """
        High-level wrapper: takes a desired Twist, returns a safe Twist
        with smooth obstacle avoidance and velocity smoothing applied.
        """
        others = getattr(self, '_other_positions', None)

        mod_vx, mod_vy = self.compute_avoidance_velocity(
            cmd.linear.x,
            0.0,
            others,
            desired_angular=cmd.angular.z,
        )

        safe = Twist()
        if (
            mod_vx == 0.0
            and mod_vy == 0.0
            and self._is_emergency(cmd.linear.x, cmd.angular.z)
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

    def _overall_min_distance(self) -> float:
        with self._scan_lock:
            return min(self.sector_min) if self.sector_min else self.max_valid_range

    def _minimum_distance_for_motion(
        self, linear_velocity: float, angular_velocity: float = 0.0
    ) -> float:
        """Return clearance in the swept half of the Burger footprint."""
        with self._scan_lock:
            if not self.sector_min:
                return self.max_valid_range
            if abs(linear_velocity) < 1e-3 or abs(angular_velocity) > 0.4:
                indices = range(NUM_SECTORS)
            elif linear_velocity > 0.0:
                indices = (7, 0, 1, 6, 2)
            else:
                indices = (3, 4, 5, 2, 6)
            return min(self.sector_min[index] for index in indices)

    def _is_emergency(
        self, linear_velocity: float = 0.0, angular_velocity: float = 0.0
    ) -> bool:
        return (
            self._minimum_distance_for_motion(linear_velocity, angular_velocity)
            < self.emergency_radius
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

    threat_pub = rospy.Publisher(
        f'/{robot_name}/threat_level', Float32, queue_size=1
    )
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
        threat = oa.get_threat_level()
        threat_pub.publish(Float32(data=threat))
        marker_pub.publish(oa.create_avoidance_markers())
        rate.sleep()


if __name__ == '__main__':
    try:
        _standalone_main()
    except rospy.ROSInterruptException:
        pass
