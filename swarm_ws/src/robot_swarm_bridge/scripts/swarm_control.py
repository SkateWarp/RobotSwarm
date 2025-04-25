#!/usr/bin/env python3
import rospy
import math
import tf
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from enum import Enum

# --- Constants ---
NODE_NAME = 'enhanced_swarm_controller'
DEFAULT_CONTROL_RATE = 15 # Hz
DEFAULT_INIT_DURATION = 15.0 # seconds
DEFAULT_RECOVERY_TIMEOUT = 5.0 # seconds
ANGLE_NORMALIZATION_THRESHOLD = 0.01 # Radians, threshold for small angle changes

# --- Helper Functions ---
def normalize_angle(angle):
    """Keep angles within [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    # Using atan2 is robust but potentially slower for repeated calls if not needed
    # return math.atan2(math.sin(angle), math.cos(angle))
    return angle

# --- State Machine for Followers ---
class FollowerState(Enum):
    INITIALIZING = 1
    FORMING = 2
    AVOIDING_OBSTACLE = 3
    AVOIDING_ROBOT = 4
    RECOVERING = 5

# --- Leader Class ---
class SwarmLeader:
    """ Controls the leader robot's movement and provides state information. """
    def __init__(self):
        """ Initializes the leader robot. """
        rospy.loginfo("Initializing Swarm Leader...")
        # Load parameters with defaults
        self.linear_vel = rospy.get_param('~linear_vel', 0.15)
        self.angular_vel = rospy.get_param('~angular_vel', 0.25)
        self.ns = rospy.get_param('~leader_ns', 'robot/1') # Initial guess for namespace
        self.obstacle_check_dist = rospy.get_param('~leader_obstacle_dist', 0.5) # m
        self.obstacle_avoid_factor = rospy.get_param('~leader_avoid_factor', 1.5) # Multiplier for angular vel during avoidance

        # Track actual angular velocity for smoother follower prediction
        self.last_yaw = 0.0
        self.last_yaw_time = rospy.Time.now().to_sec()
        self.actual_angular_vel = 0.0 # Start at zero
        self.yaw_filter_alpha = 0.3 # Smoothing factor for actual angular velocity

        # ROS setup - Publishers and Subscribers will be finalized after namespace confirmation
        self.cmd_pub = None
        self.odom_sub = None
        self.scan_sub = None

        # State variables
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.odom_ready = False # Flag to indicate if odometry has been received
        self.scan_ready = False # Flag to indicate if scan has been received
        self.obstacle_detected = False

    def finalize_ros_setup(self, actual_ns):
        """ Sets up ROS interfaces once the correct namespace is confirmed. """
        self.ns = actual_ns
        rospy.loginfo(f"Finalizing Leader ROS setup for namespace: {self.ns}")
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb, queue_size=1)
        self.scan_sub = rospy.Subscriber(f'/{self.ns}/scan', LaserScan, self.scan_cb, queue_size=1)
        # Short delay to allow connections
        rospy.sleep(0.2)

    def odom_cb(self, msg):
        """ Callback for Odometry messages. Updates position, yaw, and actual angular velocity. """
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        current_yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]

        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_yaw_time

        if dt > ANGLE_NORMALIZATION_THRESHOLD: # Avoid division by zero or instability
            # Calculate yaw change, handling wrap-around
            yaw_change = normalize_angle(current_yaw - self.last_yaw)

            # Calculate instantaneous angular velocity
            inst_angular_vel = yaw_change / dt

            # Apply a simple low-pass filter to smooth the actual angular velocity
            self.actual_angular_vel = (1.0 - self.yaw_filter_alpha) * self.actual_angular_vel + \
                                      self.yaw_filter_alpha * inst_angular_vel

            self.last_yaw = current_yaw
            self.last_yaw_time = current_time

        self.yaw = current_yaw
        if not self.odom_ready:
            rospy.loginfo(f"Leader Odometry Ready (NS: {self.ns})")
            self.odom_ready = True

    def scan_cb(self, msg):
        """ Callback for LaserScan messages. Performs simple forward obstacle detection. """
        if not msg.ranges:
            self.obstacle_detected = False
            return

        num_ranges = len(msg.ranges)
        # Check front 90 degrees (adjust indices as needed based on LIDAR setup)
        # Example: If range[0] is front, check +/- 45 degrees
        center_index = num_ranges // 2 # Assuming center index corresponds to 0 radians relative angle
        angle_increment = msg.angle_increment
        front_angle_range = math.pi / 4 # +/- 45 degrees

        if angle_increment <= 0: # Safety check for invalid angle_increment
             rospy.logwarn_throttle(5, f"[{self.ns}] Invalid angle_increment in LaserScan: {angle_increment}")
             self.obstacle_detected = False
             return

        indices_per_side = int(math.ceil(front_angle_range / angle_increment))

        # Calculate start and end indices, handling wrap-around if needed
        # This depends heavily on the specific LIDAR's angle convention (e.g., where index 0 points)
        # Assuming index 0 is front for simplicity here (adjust if necessary)
        start_idx = -indices_per_side
        end_idx = indices_per_side

        # Extract ranges, handling potential index wrapping
        front_ranges = []
        for i in range(start_idx, end_idx + 1):
             actual_index = i % num_ranges
             front_ranges.append(msg.ranges[actual_index])

        # Filter valid ranges within the detection distance
        valid_ranges = [r for r in front_ranges if msg.range_min < r < self.obstacle_check_dist]

        self.obstacle_detected = len(valid_ranges) > 0
        self.scan_ready = True


    def move(self):
        """ Publishes movement commands for the leader (circular motion with basic avoidance). """
        if not self.odom_ready or not self.cmd_pub:
             rospy.logwarn_throttle(5, f"[{self.ns}] Leader move called before ready or publisher setup.")
             return

        cmd = Twist()
        if self.obstacle_detected:
            # Simple avoidance: slow down and turn more sharply away from the default turn direction
            cmd.linear.x = self.linear_vel * 0.5
            # Turn slightly sharper than usual, assuming default turn is positive angular.z
            # A more robust method would determine which side the obstacle is on.
            cmd.angular.z = self.angular_vel * self.obstacle_avoid_factor
            rospy.logwarn_throttle(2, f"[{self.ns}] Leader avoiding obstacle.")
        else:
            # Normal circular motion
            cmd.linear.x = self.linear_vel
            cmd.angular.z = self.angular_vel

        # Apply safety limits (optional, but good practice)
        cmd.linear.x = max(-self.linear_vel, min(self.linear_vel, cmd.linear.x))
        cmd.angular.z = max(-self.angular_vel * 2, min(self.angular_vel * 2, cmd.angular.z)) # Allow sharper turns for avoidance

        self.cmd_pub.publish(cmd)

    def stop(self):
        """ Stops the leader robot. """
        if self.cmd_pub:
            rospy.loginfo(f"[{self.ns}] Leader stopping.")
            self.cmd_pub.publish(Twist()) # Zero velocity command
        else:
             rospy.logwarn(f"[{self.ns}] Leader stop called before publisher setup.")


# --- Follower Class ---
class SwarmMember:
    """ Controls a follower robot, maintaining formation and avoiding collisions. """
    def __init__(self, namespace, leader, formation_config):
        """ Initializes the follower robot. """
        self.ns = namespace
        self.leader = leader # Reference to the leader object
        self.formation_angle = formation_config['angle'] # Desired angle relative to leader's heading
        self.formation_radius = formation_config['radius'] # Desired distance from leader
        self.robot_spacing = formation_config['spacing'] # Additional spacing factor

        rospy.loginfo(f"Initializing Swarm Member: {self.ns} (Angle: {self.formation_angle:.2f} rad, Radius: {self.formation_radius:.2f} m)")

        # --- Control Parameters ---
        # PID Gains for Formation Control
        self.k_p_lin = rospy.get_param('~k_p_linear', 0.7)
        self.k_i_lin = rospy.get_param('~k_i_linear', 0.05)
        self.k_d_lin = rospy.get_param('~k_d_linear', 0.1)
        self.k_p_ang = rospy.get_param('~k_p_angular', 1.5)
        self.k_i_ang = rospy.get_param('~k_i_angular', 0.1)
        self.k_d_ang = rospy.get_param('~k_d_angular', 0.2)

        # Safety and Avoidance Parameters
        self.safe_dist_obstacle = rospy.get_param('~safety_distance_obstacle', 0.6) # Min distance to static obstacles
        self.safe_dist_robot = rospy.get_param('~safety_distance_robot', 0.7)     # Min distance to other robots (leader/followers)
        self.obstacle_repulsion_gain = rospy.get_param('~obstacle_repulsion_gain', 0.8) # How strongly to push away from obstacles
        self.robot_repulsion_gain = rospy.get_param('~robot_repulsion_gain', 1.0)       # How strongly to push away from other robots
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)

        # Recovery Parameters
        self.recovery_timeout = rospy.get_param('~recovery_timeout', DEFAULT_RECOVERY_TIMEOUT)
        self.stuck_dist_threshold = rospy.get_param('~stuck_dist_threshold', 0.02) # Min distance change to be considered "moving"
        self.stuck_error_threshold = rospy.get_param('~stuck_error_threshold', 0.2) # Min distance error to target to trigger stuck check

        # --- State Variables ---
        self.state = FollowerState.INITIALIZING # Initial state
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacle_data = {} # Dictionary {angle: distance} for detected obstacles
        self.sensors_ready = False # Flag for odom and scan readiness
        self.init_position_reached = False # Flag for initialization phase completion

        # PID Control State Variables
        self.integral_lin_err = 0.0
        self.prev_lin_err = 0.0
        self.integral_ang_err = 0.0
        self.prev_ang_err = 0.0

        # Time and Stuck Detection
        self.last_update_time = rospy.Time.now().to_sec()
        self.stuck_timer_start = None
        self.last_dist_error = float('inf')

        # ROS Interfaces
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(f'/{self.ns}/scan', LaserScan, self.scan_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.loginfo(f"[{self.ns}] ROS Interfaces setup.")


    def odom_cb(self, msg):
        """ Callback for Odometry messages. Updates position and yaw. """
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]
        if not self.sensors_ready:
             # Consider sensors ready only after first odom AND scan
             if hasattr(self, 'scan_received') and self.scan_received:
                  rospy.loginfo(f"[{self.ns}] Sensors Ready (Odom)")
                  self.sensors_ready = True
             self.odom_received = True


    def scan_cb(self, msg):
        """ Callback for LaserScan. Processes data into obstacle sectors. """
        if not msg.ranges:
            self.obstacle_data = {}
            return

        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 8 # Divide 360 degrees into 8 sectors
        new_obstacle_data = {}

        for i in range(8):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size
            sector_ranges = msg.ranges[start_idx:end_idx]

            # Filter valid ranges (within sensor limits and reasonable detection range)
            valid_ranges = [r for r in sector_ranges if msg.range_min < r < msg.range_max and r < 3.5] # Limit detection range

            if valid_ranges:
                min_dist_in_sector = min(valid_ranges)
                # Angle represents the center of the sector (0 is front, pi/4 is front-right, etc.)
                # Adjust if LIDAR zero angle is not forward
                angle = normalize_angle(i * math.pi / 4)
                new_obstacle_data[angle] = min_dist_in_sector

        self.obstacle_data = new_obstacle_data
        if not self.sensors_ready:
             if hasattr(self, 'odom_received') and self.odom_received:
                  rospy.loginfo(f"[{self.ns}] Sensors Ready (Scan)")
                  self.sensors_ready = True
             self.scan_received = True

    def _calculate_target_position(self, current_time):
        """ Calculates the desired target position based on leader state and formation config. """
        # Predict leader's future yaw based on current yaw and smoothed actual angular velocity
        # This dt should ideally match the control loop rate for better prediction
        dt_pred = current_time - self.leader.last_yaw_time # Time since leader's last yaw update
        predicted_leader_yaw = normalize_angle(self.leader.yaw + self.leader.actual_angular_vel * dt_pred)

        # Calculate the absolute desired angle in the world frame
        desired_world_angle = normalize_angle(predicted_leader_yaw + self.formation_angle)

        # Calculate target coordinates relative to the leader's current position
        # Add spacing to the radius for the final target distance
        target_radius = self.formation_radius + self.robot_spacing
        target_x = self.leader.position[0] + target_radius * math.cos(desired_world_angle)
        target_y = self.leader.position[1] + target_radius * math.sin(desired_world_angle)

        return target_x, target_y

    def _calculate_pid_control(self, dist_error, angle_error, dt):
        """ Calculates linear and angular velocities using PID controllers. """
        # --- Linear Velocity PID ---
        self.integral_lin_err += dist_error * dt
        # Anti-windup (clamp integral term)
        self.integral_lin_err = max(-1.0, min(1.0, self.integral_lin_err)) # Adjust limits as needed
        derivative_lin_err = (dist_error - self.prev_lin_err) / dt if dt > 0 else 0.0
        self.prev_lin_err = dist_error

        linear_vel = (self.k_p_lin * dist_error +
                      self.k_i_lin * self.integral_lin_err +
                      self.k_d_lin * derivative_lin_err)

        # --- Angular Velocity PID ---
        self.integral_ang_err += angle_error * dt
        # Anti-windup
        self.integral_ang_err = max(-math.pi, min(math.pi, self.integral_ang_err)) # Adjust limits
        derivative_ang_err = (angle_error - self.prev_ang_err) / dt if dt > 0 else 0.0
        self.prev_ang_err = angle_error

        angular_vel = (self.k_p_ang * angle_error +
                       self.k_i_ang * self.integral_ang_err +
                       self.k_d_ang * derivative_ang_err)

        return linear_vel, angular_vel

    def _calculate_repulsive_velocity(self, all_followers):
        """ Calculates repulsive velocity components from obstacles and other robots. """
        repulsive_lin = 0.0
        repulsive_ang = 0.0
        obstacle_force_active = False
        robot_force_active = False

        # --- Obstacle Repulsion ---
        min_obs_dist = float('inf')
        closest_obs_angle_rel = 0.0 # Angle relative to robot's front

        for angle_abs, dist in self.obstacle_data.items():
             if dist < self.safe_dist_obstacle:
                  # Calculate angle relative to robot heading
                  relative_angle = normalize_angle(angle_abs - self.yaw) # Assuming angle_abs is world frame sector center

                  # Simple repulsive force: stronger for closer obstacles, pushes away
                  # We want angular velocity to turn away from relative_angle
                  # We might want linear velocity to slow down if obstacle is ahead
                  repulsion_strength = self.obstacle_repulsion_gain * (1.0 / (dist + 0.1) - 1.0 / self.safe_dist_obstacle)

                  # Add angular component (turn away from the obstacle)
                  # Negative sign because positive angle is CCW, we want to turn away
                  repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle) * (math.pi - abs(relative_angle))/math.pi # Weight by how much it's NOT behind

                  # Add linear component (slow down if obstacle is in front)
                  # Cosine is positive in front +/- 90deg
                  if abs(relative_angle) < math.pi / 2:
                       repulsive_lin -= repulsion_strength * math.cos(relative_angle) * 0.5 # Reduce linear speed more if directly ahead

                  obstacle_force_active = True
                  if dist < min_obs_dist:
                       min_obs_dist = dist
                       closest_obs_angle_rel = relative_angle


        # --- Robot Repulsion (Leader and Followers) ---
        min_robot_dist = float('inf')
        closest_robot_angle_rel = 0.0

        # Check Leader
        dx_leader = self.leader.position[0] - self.position[0]
        dy_leader = self.leader.position[1] - self.position[1]
        dist_leader = math.hypot(dx_leader, dy_leader)

        if dist_leader < self.safe_dist_robot:
             relative_angle_leader = normalize_angle(math.atan2(dy_leader, dx_leader) - self.yaw)
             repulsion_strength = self.robot_repulsion_gain * (1.0 / (dist_leader + 0.1) - 1.0 / self.safe_dist_robot)
             repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle_leader) * (math.pi - abs(relative_angle_leader))/math.pi
             if abs(relative_angle_leader) < math.pi / 2:
                   repulsive_lin -= repulsion_strength * math.cos(relative_angle_leader) * 0.8 # Stronger linear repulsion from robots
             robot_force_active = True
             if dist_leader < min_robot_dist:
                  min_robot_dist = dist_leader
                  closest_robot_angle_rel = relative_angle_leader


        # Check Other Followers
        if all_followers:
            for other in all_followers:
                if other.ns == self.ns: continue # Skip self

                dx_other = other.position[0] - self.position[0]
                dy_other = other.position[1] - self.position[1]
                dist_other = math.hypot(dx_other, dy_other)

                if dist_other < self.safe_dist_robot:
                    relative_angle_other = normalize_angle(math.atan2(dy_other, dx_other) - self.yaw)
                    repulsion_strength = self.robot_repulsion_gain * (1.0 / (dist_other + 0.1) - 1.0 / self.safe_dist_robot)
                    repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle_other) * (math.pi - abs(relative_angle_other))/math.pi
                    if abs(relative_angle_other) < math.pi / 2:
                         repulsive_lin -= repulsion_strength * math.cos(relative_angle_other) * 0.8
                    robot_force_active = True
                    if dist_other < min_robot_dist:
                         min_robot_dist = dist_other
                         closest_robot_angle_rel = relative_angle_other

        return repulsive_lin, repulsive_ang, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist


    def update_state(self, dist_error, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist, current_time):
         """ Updates the follower's state based on current conditions. """
         # --- Stuck Detection ---
         is_stuck = False
         if abs(dist_error - self.last_dist_error) < self.stuck_dist_threshold and dist_error > self.stuck_error_threshold:
             if self.stuck_timer_start is None:
                 self.stuck_timer_start = current_time
             elif current_time - self.stuck_timer_start > self.recovery_timeout:
                 is_stuck = True
                 # Reset timer if we enter recovery
                 self.stuck_timer_start = current_time
         else:
             self.stuck_timer_start = None # Reset timer if moving or close enough

         self.last_dist_error = dist_error

         # --- State Transitions ---
         previous_state = self.state

         # Highest priority: Recovery if stuck
         if is_stuck and self.state != FollowerState.INITIALIZING:
             self.state = FollowerState.RECOVERING
         # Next priority: Avoid other robots
         elif robot_force_active and self.state != FollowerState.INITIALIZING:
             self.state = FollowerState.AVOIDING_ROBOT
         # Next priority: Avoid obstacles
         elif obstacle_force_active and self.state != FollowerState.INITIALIZING:
             self.state = FollowerState.AVOIDING_OBSTACLE
         # If not avoiding or recovering, go back to forming (or stay initializing)
         elif self.state != FollowerState.INITIALIZING:
             self.state = FollowerState.FORMING
             # Reset stuck timer when returning to forming
             self.stuck_timer_start = None

         # Handle Initialization Completion
         if self.state == FollowerState.INITIALIZING:
             # Check if close enough to initial target (e.g., within 0.2m)
             if dist_error < 0.2:
                 self.init_position_reached = True
                 # Transition to FORMING once init position is reached (handled in main loop)

         if self.state != previous_state:
             rospy.loginfo(f"[{self.ns}] State transition: {previous_state.name} -> {self.state.name}")
             # Reset PID integrals on state change to avoid sudden jumps
             self.integral_lin_err = 0.0
             self.integral_ang_err = 0.0
             self.prev_lin_err = dist_error # Use current error as prev for next step
             self.prev_ang_err = 0.0 # Reset angular prev error


    def calculate_command(self, all_followers=None):
        """ Calculates the appropriate Twist command based on the current state. """
        if not self.sensors_ready or not self.leader.odom_ready:
            rospy.logwarn_throttle(5, f"[{self.ns}] Calculate command called before sensors ready.")
            return Twist() # Send zero command if not ready

        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_update_time
        if dt <= 0: # Avoid issues with time going backwards or zero dt
             dt = 1.0 / DEFAULT_CONTROL_RATE # Estimate dt

        # --- Calculate Target and Errors ---
        target_x, target_y = self._calculate_target_position(current_time)
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        dist_error = math.hypot(dx, dy)
        # Angle needed to point towards the target
        target_world_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_world_angle - self.yaw)

        # --- Calculate Repulsive Forces ---
        repulsive_lin, repulsive_ang, obs_active, rob_active, min_obs, min_rob = self._calculate_repulsive_velocity(all_followers)

        # --- Update State Machine ---
        self.update_state(dist_error, obs_active, rob_active, min_obs, min_rob, current_time)

        # --- Calculate Control Command based on State ---
        cmd = Twist()
        attractive_lin, attractive_ang = self._calculate_pid_control(dist_error, angle_error, dt)

        if self.state == FollowerState.INITIALIZING:
            # Simplified P-control during initialization for faster convergence
            # Use higher gains initially maybe? Or just P control.
            init_k_p_lin = 1.0
            init_k_p_ang = 2.0
            cmd.linear.x = init_k_p_lin * dist_error
            cmd.angular.z = init_k_p_ang * angle_error
            # Add some basic repulsion even during init
            cmd.linear.x += repulsive_lin * 0.5 # Lower weight during init
            cmd.angular.z += repulsive_ang * 0.5
            rospy.loginfo_throttle(1, f"[{self.ns}] Initializing: DistErr={dist_error:.2f}, AngErr={angle_error:.2f}")


        elif self.state == FollowerState.FORMING:
            # Combine attractive PID forces with repulsive forces (if any linger slightly)
            cmd.linear.x = attractive_lin + repulsive_lin # Repulsive lin usually negative
            cmd.angular.z = attractive_ang + repulsive_ang
            rospy.loginfo_throttle(1, f"[{self.ns}] Forming: DistErr={dist_error:.2f}, AngErr={angle_error:.2f}")


        elif self.state == FollowerState.AVOIDING_OBSTACLE:
            # Prioritize repulsion, potentially reduce attraction
            cmd.linear.x = attractive_lin * 0.2 + repulsive_lin # Heavily weight repulsion
            cmd.angular.z = attractive_ang * 0.1 + repulsive_ang
            rospy.logwarn_throttle(1, f"[{self.ns}] Avoiding Obstacle: MinDist={min_obs:.2f}")


        elif self.state == FollowerState.AVOIDING_ROBOT:
            # Prioritize repulsion, potentially reduce attraction
            cmd.linear.x = attractive_lin * 0.1 + repulsive_lin # Very heavily weight repulsion
            cmd.angular.z = attractive_ang * 0.05 + repulsive_ang
            rospy.logwarn_throttle(1, f"[{self.ns}] Avoiding Robot: MinDist={min_rob:.2f}")


        elif self.state == FollowerState.RECOVERING:
            # Simple recovery: move forward slowly and oscillate turning
            rospy.logwarn_throttle(1, f"[{self.ns}] Recovering!")
            cmd.linear.x = 0.1 # Move forward slowly
            # Oscillate based on time since recovery started
            time_in_recovery = current_time - (self.stuck_timer_start or current_time)
            cmd.angular.z = 0.6 * math.sin(time_in_recovery * 1.5) # Oscillating turn


        # --- Apply Velocity Limits ---
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd.linear.x))
        # Limit angular velocity more strictly if moving fast linearly to prevent instability
        current_max_angular = self.max_angular_vel / (1.0 + 1.0 * abs(cmd.linear.x / self.max_linear_vel))
        cmd.angular.z = max(-current_max_angular, min(current_max_angular, cmd.angular.z))


        # --- Update Time ---
        self.last_update_time = current_time

        # --- Publish Command ---
        self.cmd_pub.publish(cmd)
        # Return command mainly for potential debugging/logging if needed outside
        return cmd


    def stop(self):
        """ Stops the follower robot. """
        rospy.loginfo(f"[{self.ns}] Stopping.")
        self.cmd_pub.publish(Twist())


# --- Main Controller Function ---
def swarm_controller():
    """ Initializes and runs the swarm controller node. """
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    rospy.loginfo("Starting Enhanced Swarm Controller Node...")

    leader = None
    followers = []

    try:
        # --- Initialize Leader ---
        leader = SwarmLeader()

        # Dynamically find leader namespace
        possible_leader_ns = [
            leader.ns, # Try configured default first
            f"tb3_{leader.ns.split('/')[-1]}", # Try tb3_X format
            f"robot_{leader.ns.split('/')[-1]}", # Try robot_X format
            # Add more potential formats if needed
        ]
        confirmed_leader_ns = None
        rospy.loginfo(f"Attempting to connect to Leader Odometry using formats: {possible_leader_ns}...")
        for ns_format in possible_leader_ns:
            try:
                # Wait briefly for the topic to potentially become available
                rospy.wait_for_message(f'/{ns_format}/odom', Odometry, timeout=60)
                confirmed_leader_ns = ns_format
                rospy.loginfo(f"Successfully connected to Leader Odometry on namespace: {confirmed_leader_ns}")
                break
            except rospy.ROSException:
                rospy.logdebug(f"No odometry message received for namespace '{ns_format}' within timeout.")
                continue
            except Exception as e:
                 rospy.logwarn(f"Error checking namespace {ns_format}: {e}")


        if not confirmed_leader_ns:
            rospy.logerr("Could not connect to Leader Odometry using any known namespace format. Exiting.")
            return

        leader.finalize_ros_setup(confirmed_leader_ns)

        # Wait until leader's odometry is ready
        while not leader.odom_ready and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, f"Waiting for Leader ({leader.ns}) odometry data...")
            rospy.sleep(0.5)
        if rospy.is_shutdown(): return


        # --- Initialize Followers ---
        num_followers = rospy.get_param('~num_followers', 3) # Default to 3 followers
        base_radius = rospy.get_param('~formation_radius', 1.5)
        robot_spacing = rospy.get_param('~robot_spacing', 0.3)

        # Define potential namespace formats for followers (adjust indices/patterns as needed)
        # Assumes followers are numbered sequentially starting from an index (e.g., robot/2, robot/3 or tb3_1, tb3_2)
        leader_index_str = confirmed_leader_ns.split('_')[-1].split('/')[-1]
        try:
             leader_index = int(leader_index_str)
        except ValueError:
             rospy.logwarn(f"Could not parse index from leader namespace '{confirmed_leader_ns}'. Assuming follower indices start from 1 or 2.")
             leader_index = 0 # Default assumption

        # Generate potential follower namespace generation functions
        follower_ns_generators = [
            lambda i: f'robot/{i+leader_index+1}', # robot/X (if leader is robot/1, followers are robot/2, ...)
            lambda i: f'tb3_{i+1}',              # tb3_X (if leader is tb3_0, followers are tb3_1, ...) - adjust base index if needed
            lambda i: f'robot_{i+leader_index+1}', # robot_X (if leader is robot_1, followers are robot_2, ...)
        ]

        followers = []
        angle_step = 2 * math.pi / num_followers if num_followers > 0 else 0

        rospy.loginfo(f"Attempting to initialize {num_followers} followers...")
        for i in range(num_followers):
            formation_config = {
                'angle': angle_step * i,
                'radius': base_radius,
                'spacing': robot_spacing
            }
            follower_found = False
            for ns_gen in follower_ns_generators:
                follower_ns = ns_gen(i)
                try:
                    rospy.logdebug(f"Checking for follower namespace: {follower_ns}")
                    # Check if odometry topic exists for this namespace
                    rospy.wait_for_message(f'/{follower_ns}/odom', Odometry, timeout=60) # Shorter timeout for followers
                    # If message received, create the follower
                    follower = SwarmMember(follower_ns, leader, formation_config)
                    followers.append(follower)
                    rospy.loginfo(f"Successfully initialized follower: {follower_ns}")
                    follower_found = True
                    break # Stop checking formats for this follower index once found
                except rospy.ROSException:
                    rospy.logdebug(f"No odometry message for follower namespace '{follower_ns}'.")
                    continue # Try next format
                except Exception as e:
                     rospy.logwarn(f"Error checking follower namespace {follower_ns}: {e}")


            if not follower_found:
                rospy.logwarn(f"Could not initialize follower {i+1}. No suitable namespace found.")

        if not followers:
            rospy.logerr("No followers could be initialized. Exiting.")
            return

        rospy.loginfo(f"Successfully initialized {len(followers)} out of {num_followers} requested followers.")

        # Wait for all followers' sensors to be ready
        all_ready = False
        start_wait_time = rospy.Time.now()
        wait_timeout = 30.0 # Max seconds to wait for followers
        while not all_ready and not rospy.is_shutdown() and (rospy.Time.now() - start_wait_time).to_sec() < wait_timeout:
             all_ready = all(f.sensors_ready for f in followers)
             rospy.loginfo_throttle(5, f"Waiting for {len([f for f in followers if not f.sensors_ready])} followers to become ready...")
             rospy.sleep(0.5)

        if not all_ready:
             rospy.logwarn("Timeout waiting for all followers to become ready. Continuing with available followers...")


        # ==============================================
        # INITIALIZATION PHASE
        # ==============================================
        rospy.loginfo("Starting Initialization Phase...")
        leader.stop() # Keep leader stationary during initialization

        init_duration = rospy.get_param('~init_duration', DEFAULT_INIT_DURATION)
        init_rate = rospy.Rate(10) # Control rate during initialization
        init_start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - init_start_time).to_sec()

            # Check if duration exceeded
            if elapsed_time >= init_duration:
                rospy.loginfo("Initialization duration reached.")
                break

            # Keep leader stopped
            leader.stop()

            # Update followers in initialization state
            all_positioned = True
            for follower in followers:
                if follower.state == FollowerState.INITIALIZING:
                    # Calculate command (will use init logic internally)
                    follower.calculate_command(followers)
                    if not follower.init_position_reached:
                        all_positioned = False
                else:
                     # Should not happen, but stop follower if not initializing
                     follower.stop()

            # Check if all followers reached their positions
            if all_positioned:
                rospy.loginfo("All followers reached initial positions. Ending initialization phase early.")
                break

            init_rate.sleep()

        # Transition all followers to FORMING state after initialization
        rospy.loginfo("Initialization complete. Transitioning followers to FORMING state.")
        for follower in followers:
            follower.state = FollowerState.FORMING
            follower.init_position_reached = True # Mark as done
            # Reset PID errors after initialization phase
            follower.integral_lin_err = 0.0
            follower.prev_lin_err = 0.0
            follower.integral_ang_err = 0.0
            follower.prev_ang_err = 0.0
            follower.stuck_timer_start = None # Reset stuck timer


        rospy.loginfo("Starting Formation Control Phase...")
        # ==============================================
        # FORMATION PHASE
        # ==============================================
        control_rate = rospy.Rate(rospy.get_param('~control_rate', DEFAULT_CONTROL_RATE))

        while not rospy.is_shutdown():
            # 1. Leader moves based on its logic (circular path + basic avoidance)
            leader.move()

            # 2. Followers update based on their state machine and calculations
            for follower in followers:
                # The calculate_command method now handles state transitions and control internally
                follower.calculate_command(followers)

            # 3. Sleep for the control loop duration
            control_rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C detected. Shutting down swarm...")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException. Shutting down swarm...")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in swarm controller: {e}", exc_info=True) # Log traceback
    finally:
        # --- Emergency Stop ---
        rospy.loginfo("Executing emergency stop for all robots.")
        try:
            if leader and leader.cmd_pub:
                leader.stop()
            if followers:
                for follower in followers:
                    if follower.cmd_pub:
                        follower.stop()
            rospy.loginfo("Emergency stop commands sent.")
        except Exception as stop_e:
            rospy.logerr(f"Error during emergency stop: {stop_e}")


if __name__ == '__main__':
    try:
        swarm_controller()
    except Exception as main_e:
         # Catch exceptions during init that might happen before ROS is fully running
         print(f"Critical error during swarm controller startup: {main_e}")
