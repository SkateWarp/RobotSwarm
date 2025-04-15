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
MIN_RECOVERY_DURATION = 2.0 # Minimum time (s) to stay in recovery state
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
            # Ensure scan_ready is set eventually even if first scan is empty
            if not self.scan_ready: self.scan_ready = True
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
             if not self.scan_ready: self.scan_ready = True
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
             # Check if range value is valid number before appending
             if isinstance(msg.ranges[actual_index], (int, float)) and not math.isnan(msg.ranges[actual_index]) and not math.isinf(msg.ranges[actual_index]):
                 front_ranges.append(msg.ranges[actual_index])

        # Filter valid ranges within the detection distance
        valid_ranges = [r for r in front_ranges if msg.range_min < r < self.obstacle_check_dist]

        self.obstacle_detected = len(valid_ranges) > 0
        if not self.scan_ready: self.scan_ready = True


    def move(self):
        """ Publishes movement commands for the leader (circular motion with basic avoidance). """
        if not self.odom_ready or not self.scan_ready or not self.cmd_pub:
             rospy.logwarn_throttle(5, f"[{self.ns}] Leader move called before ready or publisher setup (odom:{self.odom_ready}, scan:{self.scan_ready}, pub:{self.cmd_pub is not None}).")
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
        self.min_recovery_duration = rospy.get_param('~min_recovery_duration', MIN_RECOVERY_DURATION)
        self.stuck_dist_threshold = rospy.get_param('~stuck_dist_threshold', 0.02) # Min distance change to be considered "moving"
        self.stuck_error_threshold = rospy.get_param('~stuck_error_threshold', 0.2) # Min distance error to target to trigger stuck check
        self.recovery_backup_dist = rospy.get_param('~recovery_backup_dist', 0.1) # Distance to back up during recovery
        self.recovery_turn_angle = rospy.get_param('~recovery_turn_angle', math.pi / 2) # Angle to turn during recovery

        # --- State Variables ---
        self.state = FollowerState.INITIALIZING # Initial state
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacle_data = {} # Dictionary {angle: distance} for detected obstacles
        self.sensors_ready = False # Flag for odom and scan readiness
        self.odom_received = False # Separate flags for better sensor readiness check
        self.scan_received = False
        self.init_position_reached = False # Flag for initialization phase completion

        # PID Control State Variables
        self.integral_lin_err = 0.0
        self.prev_lin_err = 0.0
        self.integral_ang_err = 0.0
        self.prev_ang_err = 0.0

        # Time and State Management
        self.last_update_time = rospy.Time.now().to_sec()
        self.stuck_timer_start = None
        self.state_timer_start = None # Timer for current state duration (used for recovery)
        self.last_dist_error = float('inf')
        self.recovery_phase = 'start' # 'start', 'backing_up', 'turning', 'done'

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
        if not self.odom_received:
             self.odom_received = True
             rospy.loginfo(f"[{self.ns}] Odom received.")
             self._check_sensors_ready()


    def scan_cb(self, msg):
        """ Callback for LaserScan. Processes data into obstacle sectors. """
        if not msg.ranges:
            self.obstacle_data = {}
            if not self.scan_received:
                 self.scan_received = True
                 rospy.loginfo(f"[{self.ns}] Scan received (empty).")
                 self._check_sensors_ready()
            return

        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 8 # Divide 360 degrees into 8 sectors
        new_obstacle_data = {}

        for i in range(8):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size
            sector_ranges = msg.ranges[start_idx:end_idx]

            # Filter valid ranges (within sensor limits and reasonable detection range)
            valid_ranges = [r for r in sector_ranges if isinstance(r, (int, float)) and msg.range_min < r < msg.range_max and r < 3.5] # Limit detection range

            if valid_ranges:
                min_dist_in_sector = min(valid_ranges)
                # Angle represents the center of the sector (0 is front, pi/4 is front-right, etc.)
                # Adjust if LIDAR zero angle is not forward
                angle = normalize_angle(i * math.pi / 4)
                new_obstacle_data[angle] = min_dist_in_sector

        self.obstacle_data = new_obstacle_data
        if not self.scan_received:
             self.scan_received = True
             rospy.loginfo(f"[{self.ns}] Scan received (valid).")
             self._check_sensors_ready()

    def _check_sensors_ready(self):
        """ Checks if both odom and scan have been received at least once. """
        if self.odom_received and self.scan_received and not self.sensors_ready:
            self.sensors_ready = True
            rospy.loginfo(f"[{self.ns}] Sensors Ready (Odom & Scan received).")


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
        derivative_lin_err = (dist_error - self.prev_lin_err) / dt if dt > ANGLE_NORMALIZATION_THRESHOLD else 0.0 # Use threshold dt
        self.prev_lin_err = dist_error

        linear_vel = (self.k_p_lin * dist_error +
                      self.k_i_lin * self.integral_lin_err +
                      self.k_d_lin * derivative_lin_err)

        # --- Angular Velocity PID ---
        self.integral_ang_err += angle_error * dt
        # Anti-windup
        self.integral_ang_err = max(-math.pi, min(math.pi, self.integral_ang_err)) # Adjust limits
        derivative_ang_err = (angle_error - self.prev_ang_err) / dt if dt > ANGLE_NORMALIZATION_THRESHOLD else 0.0 # Use threshold dt
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
                  # Assumes obstacle_data stores angles relative to world frame or a fixed frame
                  # If angles are relative to robot already, remove '- self.yaw'
                  relative_angle = normalize_angle(angle_abs - self.yaw) # Check if angle_abs is world or robot relative

                  # Simple repulsive force: stronger for closer obstacles, pushes away
                  repulsion_strength = self.obstacle_repulsion_gain * (1.0 / (dist + 0.1) - 1.0 / self.safe_dist_obstacle)
                  repulsion_strength = max(0, repulsion_strength) # Ensure non-negative

                  # Add angular component (turn away from the obstacle)
                  # Weight by how much it's NOT directly behind (cosine of angle to +/- pi/2)
                  weight = max(0, math.cos(relative_angle)) # Stronger repulsion from front
                  repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle) * weight

                  # Add linear component (slow down if obstacle is in front)
                  if abs(relative_angle) < math.pi / 2:
                       repulsive_lin -= repulsion_strength * math.cos(relative_angle) * 0.5 # Reduce linear speed more if directly ahead

                  obstacle_force_active = True
                  if dist < min_obs_dist:
                       min_obs_dist = dist
                       closest_obs_angle_rel = relative_angle


        # --- Robot Repulsion (Leader and Followers) ---
        min_robot_dist = float('inf')
        closest_robot_angle_rel = 0.0
        robots_to_check = []
        if self.leader:
            robots_to_check.append(self.leader)
        if all_followers:
            robots_to_check.extend([f for f in all_followers if f.ns != self.ns]) # Add other followers

        for robot in robots_to_check:
             # Ensure the other robot has valid position data
             if not isinstance(robot.position, (list, tuple)) or len(robot.position) != 2:
                  rospy.logwarn_throttle(10, f"[{self.ns}] Invalid position data for robot {getattr(robot, 'ns', 'leader')}. Skipping repulsion calculation.")
                  continue

             dx = robot.position[0] - self.position[0]
             dy = robot.position[1] - self.position[1]
             dist = math.hypot(dx, dy)

             if 0 < dist < self.safe_dist_robot: # Check dist > 0 to avoid self-repulsion if leader==self somehow
                  relative_angle = normalize_angle(math.atan2(dy, dx) - self.yaw)
                  repulsion_strength = self.robot_repulsion_gain * (1.0 / (dist + 0.1) - 1.0 / self.safe_dist_robot)
                  repulsion_strength = max(0, repulsion_strength) # Ensure non-negative

                  # Angular repulsion
                  weight = max(0, math.cos(relative_angle)) # Stronger repulsion from front
                  repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle) * weight

                  # Linear repulsion (slow down/reverse)
                  if abs(relative_angle) < math.pi / 2:
                       repulsive_lin -= repulsion_strength * math.cos(relative_angle) * 0.8 # Stronger linear repulsion from robots

                  robot_force_active = True
                  if dist < min_robot_dist:
                       min_robot_dist = dist
                       closest_robot_angle_rel = relative_angle

        return repulsive_lin, repulsive_ang, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist


    def update_state(self, dist_error, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist, current_time):
         """ Updates the follower's state based on current conditions. """
         # --- Stuck Detection ---
         is_stuck = False
         # Only check for stuck if not already recovering or initializing
         if self.state not in [FollowerState.RECOVERING, FollowerState.INITIALIZING]:
             # Check if distance error hasn't improved significantly AND we are far from target
             if abs(dist_error - self.last_dist_error) < self.stuck_dist_threshold and dist_error > self.stuck_error_threshold:
                 if self.stuck_timer_start is None:
                     self.stuck_timer_start = current_time
                     rospy.logdebug(f"[{self.ns}] Stuck timer started (DistErr: {dist_error:.2f})")
                 # If timer exceeds timeout, trigger recovery
                 elif current_time - self.stuck_timer_start > self.recovery_timeout:
                     is_stuck = True
                     rospy.logwarn(f"[{self.ns}] Stuck detected! Entering RECOVERING state.")
                     # Reset timer once recovery is triggered
                     self.stuck_timer_start = None
             else:
                 # Reset timer if moving significantly or close enough to target
                 if self.stuck_timer_start is not None:
                      rospy.logdebug(f"[{self.ns}] Stuck timer reset.")
                 self.stuck_timer_start = None

         self.last_dist_error = dist_error

         # --- State Transitions ---
         previous_state = self.state
         current_state_duration = (current_time - self.state_timer_start) if self.state_timer_start else 0.0

         # 1. Handle Recovery State Exit Condition
         if self.state == FollowerState.RECOVERING:
             # Exit recovery ONLY if minimum duration passed AND no longer stuck AND no immediate collision threats
             if current_state_duration > self.min_recovery_duration and not is_stuck and not robot_force_active and not obstacle_force_active:
                 rospy.loginfo(f"[{self.ns}] Exiting RECOVERING state after {current_state_duration:.2f}s.")
                 self.state = FollowerState.FORMING
                 self.recovery_phase = 'start' # Reset recovery phase tracking
             else:
                 # Stay in recovery
                 pass # Handled in calculate_command

         # 2. Handle Transitions into High-Priority States (if not already recovering)
         elif is_stuck:
             self.state = FollowerState.RECOVERING
         elif robot_force_active:
             self.state = FollowerState.AVOIDING_ROBOT
         elif obstacle_force_active:
             self.state = FollowerState.AVOIDING_OBSTACLE

         # 3. Handle Transitions back to Forming (if none of the above apply)
         elif self.state != FollowerState.INITIALIZING:
             # If currently avoiding, but no longer need to, go back to forming
             if self.state in [FollowerState.AVOIDING_OBSTACLE, FollowerState.AVOIDING_ROBOT]:
                  rospy.loginfo(f"[{self.ns}] Exiting {previous_state.name} state.")
                  self.state = FollowerState.FORMING
             # Otherwise, stay forming
             elif self.state == FollowerState.FORMING:
                  pass # Already forming

         # 4. Handle Initialization Completion
         if self.state == FollowerState.INITIALIZING:
             # Check if close enough to initial target (e.g., within 0.15m)
             if dist_error < 0.15:
                 self.init_position_reached = True
                 # Actual transition to FORMING happens in the main loop after init phase

         # --- Actions on State Change ---
         if self.state != previous_state:
             rospy.loginfo(f"[{self.ns}] State transition: {previous_state.name} -> {self.state.name}")
             self.state_timer_start = current_time # Reset state timer
             # Reset PID integrals on state change to avoid sudden jumps, except maybe for recovery?
             if self.state != FollowerState.RECOVERING: # Keep integral during recovery? Maybe reset only lin?
                 self.integral_lin_err = 0.0
                 self.integral_ang_err = 0.0
                 self.prev_lin_err = dist_error # Use current error as prev for next step
                 self.prev_ang_err = angle_error # Use current error

             # Reset recovery phase tracking if entering recovery
             if self.state == FollowerState.RECOVERING:
                  self.recovery_phase = 'start'
                  self.recovery_start_pos = self.position # Store position at start of recovery
                  self.recovery_start_yaw = self.yaw

             # Reset stuck timer when leaving recovery or avoidance states
             if previous_state == FollowerState.RECOVERING or \
                previous_state == FollowerState.AVOIDING_OBSTACLE or \
                previous_state == FollowerState.AVOIDING_ROBOT:
                 self.stuck_timer_start = None


    def calculate_command(self, all_followers=None):
        """ Calculates the appropriate Twist command based on the current state. """
        if not self.sensors_ready or not self.leader.odom_ready:
            # Only log warning if sensors were previously ready
            if self.sensors_ready:
                 rospy.logwarn_throttle(5, f"[{self.ns}] Calculate command called but sensors not ready (odom:{self.leader.odom_ready}, self:{self.sensors_ready}).")
            else:
                 rospy.logdebug_throttle(10, f"[{self.ns}] Calculate command called before sensors ready.")
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
        # Calculate attractive force (PID) regardless of state, but apply differently
        attractive_lin, attractive_ang = self._calculate_pid_control(dist_error, angle_error, dt)

        if self.state == FollowerState.INITIALIZING:
            # Simplified P-control during initialization for faster convergence
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
            rospy.loginfo_throttle(1, f"[{self.ns}] Forming: DistErr={dist_error:.2f}, AngErr={angle_error:.2f}, Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")


        elif self.state == FollowerState.AVOIDING_OBSTACLE:
            # Prioritize repulsion, potentially reduce attraction
            cmd.linear.x = attractive_lin * 0.2 + repulsive_lin # Heavily weight repulsion
            cmd.angular.z = attractive_ang * 0.1 + repulsive_ang
            rospy.logwarn_throttle(1, f"[{self.ns}] Avoiding Obstacle: MinDist={min_obs:.2f}, Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")


        elif self.state == FollowerState.AVOIDING_ROBOT:
            # Prioritize repulsion, potentially reduce attraction
            cmd.linear.x = attractive_lin * 0.1 + repulsive_lin # Very heavily weight repulsion
            cmd.angular.z = attractive_ang * 0.05 + repulsive_ang
            rospy.logwarn_throttle(1, f"[{self.ns}] Avoiding Robot: MinDist={min_rob:.2f}, Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")


        elif self.state == FollowerState.RECOVERING:
            # Enhanced recovery: Back up, then turn
            rospy.logwarn_throttle(1, f"[{self.ns}] Recovering! Phase: {self.recovery_phase}")
            current_state_duration = current_time - (self.state_timer_start or current_time)

            if self.recovery_phase == 'start':
                 # Start backing up
                 self.recovery_phase = 'backing_up'
                 self.recovery_start_pos = self.position # Record position when starting backup

            if self.recovery_phase == 'backing_up':
                 dist_backed_up = math.hypot(self.position[0] - self.recovery_start_pos[0],
                                              self.position[1] - self.recovery_start_pos[1])
                 if dist_backed_up < self.recovery_backup_dist:
                      cmd.linear.x = -0.1 # Slow backup speed
                      cmd.angular.z = 0.0
                 else:
                      # Finished backing up, start turning
                      self.recovery_phase = 'turning'
                      self.recovery_start_yaw = self.yaw # Record yaw when starting turn
                      cmd.linear.x = 0.0
                      cmd.angular.z = 0.0 # Stop briefly before turning

            if self.recovery_phase == 'turning':
                 # Turn roughly by recovery_turn_angle (e.g., 90 degrees)
                 # Decide turn direction based on nearest obstacle/robot if possible, otherwise default (e.g., CCW)
                 turn_direction = 1.0 # Default CCW
                 # Optional: Add logic here to check obstacle_data or robot positions to pick a better turn direction

                 angle_turned = abs(normalize_angle(self.yaw - self.recovery_start_yaw))

                 if angle_turned < abs(self.recovery_turn_angle):
                      cmd.linear.x = 0.0
                      cmd.angular.z = 0.5 * turn_direction # Moderate turning speed
                 else:
                      # Finished turning
                      self.recovery_phase = 'done' # Mark phase as done, state transition logic will handle exit
                      cmd.linear.x = 0.0
                      cmd.angular.z = 0.0
                      rospy.loginfo(f"[{self.ns}] Recovery maneuver (backup & turn) complete. State logic will check for exit.")

            # If phase is 'done', output zero velocity until state transitions out
            if self.recovery_phase == 'done':
                 cmd.linear.x = 0.0
                 cmd.angular.z = 0.0


        # --- Apply Velocity Limits ---
        # Limit linear velocity first
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd.linear.x))
        # Limit angular velocity, potentially based on linear velocity
        # Reduce max angular if moving fast linearly to prevent skidding/instability
        # Example: Reduce max angular proportionally down to 50% at max linear speed
        reduction_factor = 1.0 - 0.5 * abs(cmd.linear.x / self.max_linear_vel) if self.max_linear_vel > 0 else 1.0
        current_max_angular = self.max_angular_vel * reduction_factor
        cmd.angular.z = max(-current_max_angular, min(current_max_angular, cmd.angular.z))


        # --- Update Time ---
        self.last_update_time = current_time

        # --- Publish Command ---
        # Only publish if the publisher exists
        if self.cmd_pub:
             self.cmd_pub.publish(cmd)
        else:
             rospy.logwarn_throttle(10, f"[{self.ns}] cmd_pub not initialized, cannot publish command.")

        # Return command mainly for potential debugging/logging if needed outside
        return cmd


    def stop(self):
        """ Stops the follower robot. """
        if self.cmd_pub:
             rospy.loginfo(f"[{self.ns}] Stopping.")
             self.cmd_pub.publish(Twist())
        else:
             rospy.logwarn(f"[{self.ns}] Stop called before publisher setup.")


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
        # Increase timeout for finding the leader initially
        leader_connect_timeout = rospy.get_param('~leader_connect_timeout', 60.0)
        for ns_format in possible_leader_ns:
            try:
                rospy.loginfo(f"Checking for leader odom on: /{ns_format}/odom")
                rospy.wait_for_message(f'/{ns_format}/odom', Odometry, timeout=leader_connect_timeout)
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

        # Wait until leader's sensors are ready
        while not (leader.odom_ready and leader.scan_ready) and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, f"Waiting for Leader ({leader.ns}) sensors (odom:{leader.odom_ready}, scan:{leader.scan_ready})...")
            rospy.sleep(0.5)
        if rospy.is_shutdown(): return


        # --- Initialize Followers ---
        num_followers = rospy.get_param('~num_followers', 3) # Default to 3 followers
        base_radius = rospy.get_param('~formation_radius', 1.5)
        robot_spacing = rospy.get_param('~robot_spacing', 0.3)
        follower_connect_timeout = rospy.get_param('~follower_connect_timeout', 10.0) # Shorter timeout for followers

        # Define potential namespace formats for followers (adjust indices/patterns as needed)
        leader_index_str = confirmed_leader_ns.split('_')[-1].split('/')[-1]
        try:
             leader_index = int(leader_index_str)
        except ValueError:
             rospy.logwarn(f"Could not parse index from leader namespace '{confirmed_leader_ns}'. Assuming follower indices start from 1 or 2.")
             leader_index = 0 # Default assumption if leader is robot_0 or similar

        # Generate potential follower namespace generation functions
        follower_ns_generators = [
            # Adjust indices based on common conventions (e.g., leader robot/1 -> followers robot/2, robot/3...)
            # Or leader tb3_0 -> followers tb3_1, tb3_2...
            lambda i: f'robot/{i+leader_index+1}',
            lambda i: f'tb3_{i+1}', # Assumes leader might be tb3_0
            lambda i: f'robot_{i+leader_index+1}',
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
                    rospy.wait_for_message(f'/{follower_ns}/odom', Odometry, timeout=follower_connect_timeout)
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

        # Wait for all initialized followers' sensors to be ready
        all_ready = False
        start_wait_time = rospy.Time.now()
        # Adjust timeout based on number of followers?
        wait_timeout = rospy.get_param('~follower_ready_timeout', 30.0)
        while not all_ready and not rospy.is_shutdown() and (rospy.Time.now() - start_wait_time).to_sec() < wait_timeout:
             num_not_ready = len([f for f in followers if not f.sensors_ready])
             if num_not_ready == 0:
                  all_ready = True
             else:
                  rospy.loginfo_throttle(5, f"Waiting for {num_not_ready} followers to become sensor ready...")
                  rospy.sleep(0.5)

        if not all_ready:
             num_ready = len(followers) - num_not_ready
             rospy.logwarn(f"Timeout waiting for all followers to become ready. Continuing with {num_ready} ready followers...")
             # Optional: Remove non-ready followers? Or let them potentially become ready later?
             # followers = [f for f in followers if f.sensors_ready]
             # if not followers:
             #      rospy.logerr("No followers became ready. Exiting.")
             #      return


        # ==============================================
        # INITIALIZATION PHASE
        # ==============================================
        rospy.loginfo("Starting Initialization Phase...")
        leader.stop() # Keep leader stationary during initialization

        init_duration = rospy.get_param('~init_duration', DEFAULT_INIT_DURATION)
        init_rate = rospy.Rate(10) # Control rate during initialization
        init_start_time = rospy.Time.now()
        init_pos_threshold = rospy.get_param('~init_pos_threshold', 0.15) # meters

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
            # Check if all *ready* followers are positioned
            all_positioned = True
            num_initializing = 0
            for follower in followers:
                # Only control followers that are ready and initializing
                if follower.sensors_ready and follower.state == FollowerState.INITIALIZING:
                    num_initializing += 1
                    # Calculate command (will use init logic internally)
                    follower.calculate_command(followers) # Pass other followers for potential init-phase repulsion
                    # Check if this specific follower reached position based on its internal flag
                    if not follower.init_position_reached:
                        all_positioned = False
                elif not follower.sensors_ready:
                     all_positioned = False # If any follower isn't ready, we aren't all positioned
                # else: Follower is ready but not initializing (shouldn't happen here)

            # If no followers are in the initializing state anymore (e.g., they timed out becoming ready)
            if num_initializing == 0 and len(followers) > 0:
                 rospy.logwarn("No followers left in INITIALIZING state during init phase.")
                 break

            # Check if all ready followers reached their positions
            if all_positioned and num_initializing > 0:
                rospy.loginfo("All ready followers reached initial positions. Ending initialization phase early.")
                break

            init_rate.sleep()

        # Transition all *ready* followers to FORMING state after initialization
        rospy.loginfo("Initialization complete/ended. Transitioning ready followers to FORMING state.")
        num_transitioned = 0
        for follower in followers:
             if follower.sensors_ready:
                  # Even if init wasn't fully reached, move to FORMING
                  follower.state = FollowerState.FORMING
                  follower.init_position_reached = True # Mark as done with init attempt
                  # Reset PID errors after initialization phase
                  follower.integral_lin_err = 0.0
                  follower.prev_lin_err = 0.0 # Reset based on current error?
                  follower.integral_ang_err = 0.0
                  follower.prev_ang_err = 0.0
                  follower.stuck_timer_start = None # Reset stuck timer
                  follower.state_timer_start = rospy.Time.now().to_sec() # Start timer for FORMING state
                  num_transitioned += 1
             else:
                  rospy.logwarn(f"Follower {follower.ns} is not sensor ready after init phase, will not transition to FORMING yet.")

        if num_transitioned == 0 and len(followers) > 0:
             rospy.logerr("No followers were ready to transition to FORMING state. Swarm may not function.")
             # Consider exiting if this happens?
             # return

        rospy.loginfo(f"Starting Formation Control Phase with {num_transitioned} followers...")
        # ==============================================
        # FORMATION PHASE
        # ==============================================
        control_rate = rospy.Rate(rospy.get_param('~control_rate', DEFAULT_CONTROL_RATE))

        while not rospy.is_shutdown():
            # 1. Leader moves based on its logic (circular path + basic avoidance)
            # Ensure leader is ready before moving
            if leader.odom_ready and leader.scan_ready:
                 leader.move()
            else:
                 leader.stop() # Ensure leader stays stopped if not ready

            # 2. Followers update based on their state machine and calculations
            for follower in followers:
                # Only calculate command for followers that are ready and not initializing
                if follower.sensors_ready and follower.state != FollowerState.INITIALIZING:
                     follower.calculate_command(followers)
                elif not follower.sensors_ready:
                     # Attempt to check readiness again if a follower wasn't ready initially
                     follower._check_sensors_ready()
                     if follower.sensors_ready:
                          rospy.loginfo(f"Follower {follower.ns} became ready during formation phase. Setting state to FORMING.")
                          # Transition late-comer to FORMING state
                          follower.state = FollowerState.FORMING
                          follower.init_position_reached = True # Skip init phase for late joiners
                          follower.integral_lin_err = 0.0
                          follower.prev_lin_err = 0.0
                          follower.integral_ang_err = 0.0
                          follower.prev_ang_err = 0.0
                          follower.stuck_timer_start = None
                          follower.state_timer_start = rospy.Time.now().to_sec()


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
        # Use locals() to check if variables exist before accessing them
        if 'leader' in locals() and leader and leader.cmd_pub:
            leader.stop()
        if 'followers' in locals() and followers:
            for follower in followers:
                if follower and follower.cmd_pub:
                    follower.stop()
        rospy.loginfo("Emergency stop commands sent.")


if __name__ == '__main__':
    try:
        swarm_controller()
    except Exception as main_e:
         # Catch exceptions during init that might happen before ROS is fully running
         print(f"Critical error during swarm controller startup: {main_e}")

