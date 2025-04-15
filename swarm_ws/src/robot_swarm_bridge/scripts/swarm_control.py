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
        # Add flags to track if data has been processed at least once
        self.odom_processed_once = False
        self.scan_processed_once = False


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
            yaw_change = normalize_angle(current_yaw - self.last_yaw)
            inst_angular_vel = yaw_change / dt
            self.actual_angular_vel = (1.0 - self.yaw_filter_alpha) * self.actual_angular_vel + \
                                      self.yaw_filter_alpha * inst_angular_vel
            self.last_yaw = current_yaw
            self.last_yaw_time = current_time

        self.yaw = current_yaw
        if not self.odom_ready:
             # Consider ready if position is not default AND yaw calculation happened
             if self.position != (0.0, 0.0) and dt > 0:
                  rospy.loginfo(f"Leader Odometry Ready (NS: {self.ns})")
                  self.odom_ready = True
                  self.odom_processed_once = True # Mark as processed
        elif not self.odom_processed_once:
             self.odom_processed_once = True # Mark as processed if already ready


    def scan_cb(self, msg):
        """ Callback for LaserScan messages. Performs simple forward obstacle detection. """
        scan_processed = False # Flag within callback
        if not msg.ranges:
            self.obstacle_detected = False
            scan_processed = True # Processed empty scan
        else:
            num_ranges = len(msg.ranges)
            angle_increment = msg.angle_increment
            front_angle_range = math.pi / 4

            if angle_increment <= 0:
                 rospy.logwarn_throttle(5, f"[{self.ns}] Invalid angle_increment in LaserScan: {angle_increment}")
                 self.obstacle_detected = False
                 scan_processed = True # Processed invalid scan
            else:
                indices_per_side = int(math.ceil(front_angle_range / angle_increment))
                start_idx = -indices_per_side
                end_idx = indices_per_side
                front_ranges = []
                for i in range(start_idx, end_idx + 1):
                     actual_index = i % num_ranges
                     if isinstance(msg.ranges[actual_index], (int, float)) and not math.isnan(msg.ranges[actual_index]) and not math.isinf(msg.ranges[actual_index]):
                         front_ranges.append(msg.ranges[actual_index])

                valid_ranges = [r for r in front_ranges if msg.range_min < r < self.obstacle_check_dist]
                self.obstacle_detected = len(valid_ranges) > 0
                scan_processed = True # Processed valid scan

        # Update scan_ready status based on processing
        if scan_processed and not self.scan_ready:
             rospy.loginfo(f"Leader Scan Ready (NS: {self.ns})")
             self.scan_ready = True
             self.scan_processed_once = True
        elif scan_processed and not self.scan_processed_once:
             self.scan_processed_once = True


    def move(self):
        """ Publishes movement commands for the leader (circular motion with basic avoidance). """
        if not self.odom_ready or not self.scan_ready or not self.cmd_pub:
             rospy.logwarn_throttle(5, f"[{self.ns}] Leader move called before ready or publisher setup (odom:{self.odom_ready}, scan:{self.scan_ready}, pub:{self.cmd_pub is not None}). Cannot move.")
             return # Explicitly return if not ready

        cmd = Twist()
        if self.obstacle_detected:
            cmd.linear.x = self.linear_vel * 0.5
            cmd.angular.z = self.angular_vel * self.obstacle_avoid_factor
            rospy.logdebug_throttle(2, f"[{self.ns}] Leader avoiding obstacle. Cmd Vel: Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")
        else:
            cmd.linear.x = self.linear_vel
            cmd.angular.z = self.angular_vel
            rospy.logdebug_throttle(2, f"[{self.ns}] Leader normal move. Cmd Vel: Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")

        # Apply safety limits
        cmd.linear.x = max(-self.linear_vel, min(self.linear_vel, cmd.linear.x))
        cmd.angular.z = max(-self.angular_vel * 2, min(self.angular_vel * 2, cmd.angular.z))

        rospy.logdebug_throttle(1, f"[{self.ns}] Publishing Leader Cmd Vel: Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")
        self.cmd_pub.publish(cmd)

    def stop(self):
        """ Stops the leader robot. """
        if self.cmd_pub:
            rospy.loginfo(f"[{self.ns}] Leader stopping.")
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            rospy.logdebug(f"[{self.ns}] Published Leader Stop Cmd Vel: Lin={stop_cmd.linear.x:.2f}, Ang={stop_cmd.angular.z:.2f}")
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
        self.k_p_lin = rospy.get_param('~k_p_linear', 0.7)
        self.k_i_lin = rospy.get_param('~k_i_linear', 0.05)
        self.k_d_lin = rospy.get_param('~k_d_linear', 0.1)
        self.k_p_ang = rospy.get_param('~k_p_angular', 1.5)
        self.k_i_ang = rospy.get_param('~k_i_angular', 0.1)
        self.k_d_ang = rospy.get_param('~k_d_angular', 0.2)
        self.safe_dist_obstacle = rospy.get_param('~safety_distance_obstacle', 0.6)
        self.safe_dist_robot = rospy.get_param('~safety_distance_robot', 0.7)
        self.obstacle_repulsion_gain = rospy.get_param('~obstacle_repulsion_gain', 0.8)
        self.robot_repulsion_gain = rospy.get_param('~robot_repulsion_gain', 1.0)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
        self.recovery_timeout = rospy.get_param('~recovery_timeout', DEFAULT_RECOVERY_TIMEOUT)
        self.min_recovery_duration = rospy.get_param('~min_recovery_duration', MIN_RECOVERY_DURATION)
        self.stuck_dist_threshold = rospy.get_param('~stuck_dist_threshold', 0.02)
        self.stuck_error_threshold = rospy.get_param('~stuck_error_threshold', 0.2)
        self.recovery_backup_dist = rospy.get_param('~recovery_backup_dist', 0.1)
        self.recovery_turn_angle = rospy.get_param('~recovery_turn_angle', math.pi / 2)

        # --- State Variables ---
        self.state = FollowerState.INITIALIZING
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacle_data = {}
        self.sensors_ready = False
        self.odom_received_flag = False # Use different names to avoid confusion
        self.scan_received_flag = False
        self.init_position_reached = False
        self.integral_lin_err = 0.0
        self.prev_lin_err = 0.0
        self.integral_ang_err = 0.0
        self.prev_ang_err = 0.0
        self.last_update_time = rospy.Time.now().to_sec()
        self.stuck_timer_start = None
        self.state_timer_start = None
        self.last_dist_error = float('inf')
        self.recovery_phase = 'start'
        self.recovery_start_pos = (0.0, 0.0) # Initialize tuple
        self.recovery_start_yaw = 0.0

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
        if not self.odom_received_flag:
             self.odom_received_flag = True
             rospy.loginfo(f"[{self.ns}] Odom received.")
             self._check_sensors_ready()


    def scan_cb(self, msg):
        """ Callback for LaserScan. Processes data into obstacle sectors. """
        scan_processed = False
        if not msg.ranges:
            self.obstacle_data = {}
            scan_processed = True
        else:
            num_ranges = len(msg.ranges)
            sector_size = num_ranges // 8
            new_obstacle_data = {}
            angle_increment = msg.angle_increment # Get angle_increment here

            # Check angle_increment validity
            if angle_increment is None or angle_increment <= 0:
                 rospy.logwarn_throttle(10, f"[{self.ns}] Invalid angle_increment in LaserScan: {angle_increment}. Skipping scan processing.")
                 scan_processed = True # Mark as processed (even though skipped)
            else:
                for i in range(8):
                    start_idx = i * sector_size
                    end_idx = (i + 1) * sector_size
                    sector_ranges = msg.ranges[start_idx:end_idx]
                    valid_ranges = [r for r in sector_ranges if isinstance(r, (int, float)) and msg.range_min < r < msg.range_max and r < 3.5]
                    if valid_ranges:
                        min_dist_in_sector = min(valid_ranges)
                        angle = normalize_angle(i * math.pi / 4)
                        new_obstacle_data[angle] = min_dist_in_sector
                self.obstacle_data = new_obstacle_data
                scan_processed = True

        if scan_processed and not self.scan_received_flag:
             self.scan_received_flag = True
             rospy.loginfo(f"[{self.ns}] Scan received (Processed: {scan_processed}).")
             self._check_sensors_ready()

    def _check_sensors_ready(self):
        """ Checks if both odom and scan have been received at least once. """
        # Consider ready only if flags are true AND position is not default
        if self.odom_received_flag and self.scan_received_flag and self.position != (0.0, 0.0) and not self.sensors_ready:
            self.sensors_ready = True
            rospy.loginfo(f"[{self.ns}] Sensors Ready (Odom & Scan received, Pos: {self.position}).")


    def _calculate_target_position(self, current_time):
        """ Calculates the desired target position based on leader state and formation config. """
        # Ensure leader position is valid before using it
        if not isinstance(self.leader.position, (list, tuple)) or len(self.leader.position) != 2:
             rospy.logwarn_throttle(5, f"[{self.ns}] Invalid leader position ({self.leader.position}). Using own position as target.")
             return self.position[0], self.position[1] # Return current position as fallback

        dt_pred = current_time - self.leader.last_yaw_time
        predicted_leader_yaw = normalize_angle(self.leader.yaw + self.leader.actual_angular_vel * dt_pred)
        desired_world_angle = normalize_angle(predicted_leader_yaw + self.formation_angle)
        target_radius = self.formation_radius + self.robot_spacing
        target_x = self.leader.position[0] + target_radius * math.cos(desired_world_angle)
        target_y = self.leader.position[1] + target_radius * math.sin(desired_world_angle)
        return target_x, target_y

    def _calculate_pid_control(self, dist_error, angle_error, dt):
        """ Calculates linear and angular velocities using PID controllers. """
        # --- Linear Velocity PID ---
        self.integral_lin_err += dist_error * dt
        self.integral_lin_err = max(-1.0, min(1.0, self.integral_lin_err))
        derivative_lin_err = (dist_error - self.prev_lin_err) / dt if dt > ANGLE_NORMALIZATION_THRESHOLD else 0.0
        self.prev_lin_err = dist_error
        linear_vel = (self.k_p_lin * dist_error +
                      self.k_i_lin * self.integral_lin_err +
                      self.k_d_lin * derivative_lin_err)

        # --- Angular Velocity PID ---
        self.integral_ang_err += angle_error * dt
        self.integral_ang_err = max(-math.pi, min(math.pi, self.integral_ang_err))
        derivative_ang_err = (angle_error - self.prev_ang_err) / dt if dt > ANGLE_NORMALIZATION_THRESHOLD else 0.0
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
        min_obs_dist = float('inf')
        min_robot_dist = float('inf')

        # --- Obstacle Repulsion ---
        for angle_abs, dist in self.obstacle_data.items():
             if dist < self.safe_dist_obstacle:
                  relative_angle = normalize_angle(angle_abs - self.yaw) # Assumes angle_abs is world frame
                  repulsion_strength = self.obstacle_repulsion_gain * (1.0 / (dist + 0.1) - 1.0 / self.safe_dist_obstacle)
                  repulsion_strength = max(0, repulsion_strength)
                  weight = max(0, math.cos(relative_angle))
                  repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle) * weight
                  if abs(relative_angle) < math.pi / 2:
                       repulsive_lin -= repulsion_strength * math.cos(relative_angle) * 0.5
                  obstacle_force_active = True
                  if dist < min_obs_dist: min_obs_dist = dist

        # --- Robot Repulsion ---
        robots_to_check = []
        if self.leader: robots_to_check.append(self.leader)
        if all_followers: robots_to_check.extend([f for f in all_followers if f.ns != self.ns])

        for robot in robots_to_check:
             if not isinstance(robot.position, (list, tuple)) or len(robot.position) != 2: continue
             dx = robot.position[0] - self.position[0]
             dy = robot.position[1] - self.position[1]
             dist = math.hypot(dx, dy)
             if 0 < dist < self.safe_dist_robot:
                  relative_angle = normalize_angle(math.atan2(dy, dx) - self.yaw)
                  repulsion_strength = self.robot_repulsion_gain * (1.0 / (dist + 0.1) - 1.0 / self.safe_dist_robot)
                  repulsion_strength = max(0, repulsion_strength)
                  weight = max(0, math.cos(relative_angle))
                  repulsive_ang -= repulsion_strength * math.copysign(1.0, relative_angle) * weight
                  if abs(relative_angle) < math.pi / 2:
                       repulsive_lin -= repulsion_strength * math.cos(relative_angle) * 0.8
                  robot_force_active = True
                  if dist < min_robot_dist: min_robot_dist = dist

        return repulsive_lin, repulsive_ang, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist


    def update_state(self, dist_error, angle_error, obstacle_force_active, robot_force_active, min_obs_dist, min_robot_dist, current_time):
         """ Updates the follower's state based on current conditions. """
         # --- Stuck Detection ---
         is_stuck = False
         if self.state not in [FollowerState.RECOVERING, FollowerState.INITIALIZING]:
             if abs(dist_error - self.last_dist_error) < self.stuck_dist_threshold and dist_error > self.stuck_error_threshold:
                 if self.stuck_timer_start is None:
                     self.stuck_timer_start = current_time
                     rospy.logdebug(f"[{self.ns}] Stuck timer started (DistErr: {dist_error:.2f})")
                 elif current_time - self.stuck_timer_start > self.recovery_timeout:
                     is_stuck = True
                     rospy.logwarn(f"[{self.ns}] Stuck detected! Entering RECOVERING state.")
                     self.stuck_timer_start = None # Reset timer
             else:
                 if self.stuck_timer_start is not None: rospy.logdebug(f"[{self.ns}] Stuck timer reset.")
                 self.stuck_timer_start = None

         self.last_dist_error = dist_error # Update last error *after* checking

         # --- State Transitions ---
         previous_state = self.state
         current_state_duration = (current_time - self.state_timer_start) if self.state_timer_start else 0.0

         # 1. Handle Recovery State Exit Condition
         if self.state == FollowerState.RECOVERING:
             # Check conditions for exiting recovery
             can_exit_recovery = (current_state_duration > self.min_recovery_duration and
                                  self.recovery_phase == 'done' and # Ensure maneuver finished
                                  not is_stuck and # Check stuck condition again
                                  not robot_force_active and
                                  not obstacle_force_active)

             if can_exit_recovery:
                 rospy.loginfo(f"[{self.ns}] Exiting RECOVERING state. Conditions met: duration={current_state_duration:.2f}s > {self.min_recovery_duration}s, phase='{self.recovery_phase}', stuck={is_stuck}, rob_active={robot_force_active}, obs_active={obstacle_force_active}")
                 self.state = FollowerState.FORMING
                 self.recovery_phase = 'start'
             else:
                  # Log why recovery exit is blocked (only log periodically)
                  rospy.logdebug_throttle(3, f"[{self.ns}] Staying in RECOVERING. Conditions: duration={current_state_duration:.2f}s, phase='{self.recovery_phase}', stuck={is_stuck}, rob_active={robot_force_active}, obs_active={obstacle_force_active}")
                  pass # Stay in recovery

         # 2. Handle Transitions into High-Priority States (if not already recovering)
         elif is_stuck:
             self.state = FollowerState.RECOVERING
         elif robot_force_active:
             self.state = FollowerState.AVOIDING_ROBOT
         elif obstacle_force_active:
             self.state = FollowerState.AVOIDING_OBSTACLE

         # 3. Handle Transitions back to Forming (if none of the above apply)
         elif self.state != FollowerState.INITIALIZING:
             if self.state in [FollowerState.AVOIDING_OBSTACLE, FollowerState.AVOIDING_ROBOT]:
                  rospy.loginfo(f"[{self.ns}] Exiting {previous_state.name} state.")
                  self.state = FollowerState.FORMING
             elif self.state == FollowerState.FORMING: pass

         # 4. Handle Initialization Completion
         if self.state == FollowerState.INITIALIZING:
             if dist_error < 0.15: # Use parameter?
                 self.init_position_reached = True

         # --- Actions on State Change ---
         if self.state != previous_state:
             rospy.loginfo(f"[{self.ns}] State transition: {previous_state.name} -> {self.state.name}")
             self.state_timer_start = current_time
             if self.state != FollowerState.RECOVERING:
                 self.integral_lin_err = 0.0
                 self.integral_ang_err = 0.0
                 self.prev_lin_err = dist_error
                 self.prev_ang_err = angle_error
             if self.state == FollowerState.RECOVERING:
                  self.recovery_phase = 'start'
                  self.recovery_start_pos = self.position
                  self.recovery_start_yaw = self.yaw
             if previous_state == FollowerState.RECOVERING or \
                previous_state == FollowerState.AVOIDING_OBSTACLE or \
                previous_state == FollowerState.AVOIDING_ROBOT:
                 self.stuck_timer_start = None


    def calculate_command(self, all_followers=None):
        """ Calculates the appropriate Twist command based on the current state. """
        # Check sensor readiness at the beginning of calculation
        if not self.sensors_ready:
             # Check if leader is ready, if not, follower shouldn't move either
             if not self.leader or not self.leader.odom_ready or not self.leader.scan_ready:
                  rospy.logwarn_throttle(5, f"[{self.ns}] Leader not ready, follower stopping.")
                  self.stop() # Explicitly stop if leader isn't ready
                  return Twist()

             # If leader is ready but follower isn't, log and return zero cmd
             rospy.logwarn_throttle(5, f"[{self.ns}] Calculate command called but self.sensors_ready is False. Returning zero velocity.")
             return Twist()

        # Check leader readiness again (paranoid check)
        if not self.leader.odom_ready or not self.leader.scan_ready:
             rospy.logwarn_throttle(5, f"[{self.ns}] Leader became not ready during calculation, follower stopping.")
             self.stop()
             return Twist()


        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_update_time
        if dt <= 0: dt = 1.0 / DEFAULT_CONTROL_RATE

        # --- Calculate Target and Errors ---
        target_x, target_y = self._calculate_target_position(current_time)
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        dist_error = math.hypot(dx, dy)
        target_world_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_world_angle - self.yaw)

        # --- Calculate Repulsive Forces ---
        repulsive_lin, repulsive_ang, obs_active, rob_active, min_obs, min_rob = self._calculate_repulsive_velocity(all_followers)

        # --- Update State Machine ---
        # Pass angle_error needed for PID reset logic
        self.update_state(dist_error, angle_error, obs_active, rob_active, min_obs, min_rob, current_time)

        # --- Calculate Control Command based on State ---
        cmd = Twist()
        attractive_lin, attractive_ang = self._calculate_pid_control(dist_error, angle_error, dt)
        final_lin = 0.0
        final_ang = 0.0

        state_str = self.state.name # For logging

        if self.state == FollowerState.INITIALIZING:
            init_k_p_lin = 1.0
            init_k_p_ang = 2.0
            final_lin = init_k_p_lin * dist_error + repulsive_lin * 0.5
            final_ang = init_k_p_ang * angle_error + repulsive_ang * 0.5
            state_str += f" (DistErr={dist_error:.2f}, AngErr={angle_error:.2f})"

        elif self.state == FollowerState.FORMING:
            final_lin = attractive_lin + repulsive_lin
            final_ang = attractive_ang + repulsive_ang
            state_str += f" (DistErr={dist_error:.2f}, AngErr={angle_error:.2f})"

        elif self.state == FollowerState.AVOIDING_OBSTACLE:
            final_lin = attractive_lin * 0.2 + repulsive_lin
            final_ang = attractive_ang * 0.1 + repulsive_ang
            state_str += f" (MinDist={min_obs:.2f})"

        elif self.state == FollowerState.AVOIDING_ROBOT:
            final_lin = attractive_lin * 0.1 + repulsive_lin
            final_ang = attractive_ang * 0.05 + repulsive_ang
            state_str += f" (MinDist={min_rob:.2f})"

        elif self.state == FollowerState.RECOVERING:
            state_str += f" (Phase: {self.recovery_phase})"
            current_state_duration = current_time - (self.state_timer_start or current_time)

            if self.recovery_phase == 'start':
                 self.recovery_phase = 'backing_up'
                 self.recovery_start_pos = self.position

            if self.recovery_phase == 'backing_up':
                 dist_backed_up = math.hypot(self.position[0] - self.recovery_start_pos[0],
                                              self.position[1] - self.recovery_start_pos[1])
                 if dist_backed_up < self.recovery_backup_dist:
                      final_lin = -0.1
                      final_ang = 0.0
                 else:
                      self.recovery_phase = 'turning'
                      self.recovery_start_yaw = self.yaw
                      final_lin = 0.0
                      final_ang = 0.0

            if self.recovery_phase == 'turning':
                 turn_direction = 1.0
                 angle_turned = abs(normalize_angle(self.yaw - self.recovery_start_yaw))
                 if angle_turned < abs(self.recovery_turn_angle):
                      final_lin = 0.0
                      final_ang = 0.5 * turn_direction
                 else:
                      self.recovery_phase = 'done'
                      final_lin = 0.0
                      final_ang = 0.0
                      rospy.loginfo(f"[{self.ns}] Recovery maneuver complete.")

            if self.recovery_phase == 'done':
                 final_lin = 0.0
                 final_ang = 0.0


        # --- Apply Velocity Limits ---
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, final_lin))
        reduction_factor = 1.0 - 0.5 * abs(cmd.linear.x / self.max_linear_vel) if self.max_linear_vel > 0 else 1.0
        current_max_angular = self.max_angular_vel * reduction_factor
        cmd.angular.z = max(-current_max_angular, min(current_max_angular, final_ang))

        # --- Log Final Command ---
        rospy.logdebug_throttle(1, f"[{self.ns}] State={state_str}, Final Cmd Vel: Lin={cmd.linear.x:.3f}, Ang={cmd.angular.z:.3f} (Before Limit: Lin={final_lin:.3f}, Ang={final_ang:.3f})")


        # --- Update Time ---
        self.last_update_time = current_time

        # --- Publish Command ---
        if self.cmd_pub:
             self.cmd_pub.publish(cmd)
        else:
             rospy.logwarn_throttle(10, f"[{self.ns}] cmd_pub not initialized, cannot publish command.")

        return cmd # Return the final command


    def stop(self):
        """ Stops the follower robot. """
        if self.cmd_pub:
             rospy.loginfo(f"[{self.ns}] Stopping.")
             stop_cmd = Twist()
             self.cmd_pub.publish(stop_cmd)
             rospy.logdebug(f"[{self.ns}] Published Stop Cmd Vel: Lin={stop_cmd.linear.x:.2f}, Ang={stop_cmd.angular.z:.2f}")
        else:
             rospy.logwarn(f"[{self.ns}] Stop called before publisher setup.")


# --- Main Controller Function ---
def swarm_controller():
    """ Initializes and runs the swarm controller node. """
    # Change default log level to DEBUG to see more messages
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.loginfo("Starting Enhanced Swarm Controller Node...")

    leader = None
    followers = []

    try:
        # --- Initialize Leader ---
        leader = SwarmLeader()
        possible_leader_ns = [ leader.ns, f"tb3_{leader.ns.split('/')[-1]}", f"robot_{leader.ns.split('/')[-1]}" ]
        confirmed_leader_ns = None
        rospy.loginfo(f"Attempting to connect to Leader Odometry using formats: {possible_leader_ns}...")
        leader_connect_timeout = rospy.get_param('~leader_connect_timeout', 60.0)
        for ns_format in possible_leader_ns:
            try:
                rospy.logdebug(f"Checking for leader odom on: /{ns_format}/odom")
                rospy.wait_for_message(f'/{ns_format}/odom', Odometry, timeout=5.0) # Shorter check timeout
                confirmed_leader_ns = ns_format
                rospy.loginfo(f"Successfully connected to Leader Odometry on namespace: {confirmed_leader_ns}")
                break
            except rospy.ROSException: rospy.logdebug(f"No odom for ns '{ns_format}'.")
            except Exception as e: rospy.logwarn(f"Error checking ns {ns_format}: {e}")
        if not confirmed_leader_ns:
            rospy.logerr("Could not connect to Leader Odometry. Exiting.")
            return
        leader.finalize_ros_setup(confirmed_leader_ns)
        while not (leader.odom_ready and leader.scan_ready) and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, f"Waiting for Leader ({leader.ns}) sensors (odom:{leader.odom_ready}, scan:{leader.scan_ready})...")
            rospy.sleep(0.5)
        if rospy.is_shutdown(): return
        rospy.loginfo("Leader sensors ready.")


        # --- Initialize Followers ---
        num_followers = rospy.get_param('~num_followers', 3)
        base_radius = rospy.get_param('~formation_radius', 1.5)
        robot_spacing = rospy.get_param('~robot_spacing', 0.3)
        follower_connect_timeout = rospy.get_param('~follower_connect_timeout', 10.0)
        leader_index_str = confirmed_leader_ns.split('_')[-1].split('/')[-1]
        try: leader_index = int(leader_index_str)
        except ValueError:
             rospy.logwarn(f"Could not parse index from leader ns '{confirmed_leader_ns}'. Assuming base index 0.")
             leader_index = 0
        follower_ns_generators = [ lambda i: f'robot/{i+leader_index+1}', lambda i: f'tb3_{i+1}', lambda i: f'robot_{i+leader_index+1}' ]
        followers = []
        angle_step = 2 * math.pi / num_followers if num_followers > 0 else 0
        rospy.loginfo(f"Attempting to initialize {num_followers} followers...")
        for i in range(num_followers):
            # ... (follower initialization loop - shortened for brevity, no logic change) ...
            formation_config = { 'angle': angle_step * i, 'radius': base_radius, 'spacing': robot_spacing }
            follower_found = False
            for ns_gen in follower_ns_generators:
                follower_ns = ns_gen(i)
                try:
                    rospy.logdebug(f"Checking for follower ns: {follower_ns}")
                    rospy.wait_for_message(f'/{follower_ns}/odom', Odometry, timeout=follower_connect_timeout)
                    follower = SwarmMember(follower_ns, leader, formation_config)
                    followers.append(follower)
                    rospy.loginfo(f"Successfully initialized follower: {follower_ns}")
                    follower_found = True
                    break
                except rospy.ROSException: rospy.logdebug(f"No odom for follower ns '{follower_ns}'.")
                except Exception as e: rospy.logwarn(f"Error checking follower ns {follower_ns}: {e}")
            if not follower_found: rospy.logwarn(f"Could not initialize follower {i+1}.")
        if not followers:
            rospy.logerr("No followers could be initialized. Exiting.")
            return
        rospy.loginfo(f"Successfully initialized {len(followers)} followers.")

        # --- Wait for Followers Ready ---
        all_ready = False
        start_wait_time = rospy.Time.now()
        wait_timeout = rospy.get_param('~follower_ready_timeout', 30.0)
        while not all_ready and not rospy.is_shutdown() and (rospy.Time.now() - start_wait_time).to_sec() < wait_timeout:
             num_not_ready = len([f for f in followers if not f.sensors_ready])
             if num_not_ready == 0: all_ready = True
             else:
                  rospy.loginfo_throttle(5, f"Waiting for {num_not_ready} followers to become sensor ready...")
                  rospy.sleep(0.5)
        if not all_ready:
             num_ready = len(followers) - num_not_ready
             rospy.logwarn(f"Timeout waiting for all followers ready. Continuing with {num_ready} ready followers...")


        # ==============================================
        # INITIALIZATION PHASE
        # ==============================================
        rospy.loginfo("Starting Initialization Phase...")
        # ... (Initialization Phase loop - shortened for brevity, no logic change) ...
        leader.stop()
        init_duration = rospy.get_param('~init_duration', DEFAULT_INIT_DURATION)
        init_rate = rospy.Rate(10)
        init_start_time = rospy.Time.now()
        init_pos_threshold = rospy.get_param('~init_pos_threshold', 0.15)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - init_start_time).to_sec()
            if elapsed_time >= init_duration:
                rospy.loginfo("Initialization duration reached.")
                break
            leader.stop()
            all_positioned = True
            num_initializing = 0
            for follower in followers:
                if follower.sensors_ready and follower.state == FollowerState.INITIALIZING:
                    num_initializing += 1
                    follower.calculate_command(followers)
                    if not follower.init_position_reached: all_positioned = False
                elif not follower.sensors_ready: all_positioned = False
            if num_initializing == 0 and len(followers) > 0:
                 rospy.logwarn("No followers left in INITIALIZING state during init phase.")
                 break
            if all_positioned and num_initializing > 0:
                rospy.loginfo("All ready followers reached initial positions. Ending initialization phase early.")
                break
            init_rate.sleep()

        # --- Transition to FORMING ---
        rospy.loginfo("Initialization complete/ended. Transitioning ready followers to FORMING state.")
        num_transitioned = 0
        for follower in followers:
             if follower.sensors_ready:
                  follower.state = FollowerState.FORMING
                  follower.init_position_reached = True
                  follower.integral_lin_err = 0.0; follower.prev_lin_err = 0.0
                  follower.integral_ang_err = 0.0; follower.prev_ang_err = 0.0
                  follower.stuck_timer_start = None
                  follower.state_timer_start = rospy.Time.now().to_sec()
                  num_transitioned += 1
             else: rospy.logwarn(f"Follower {follower.ns} not ready, will not transition.")
        if num_transitioned == 0 and len(followers) > 0:
             rospy.logerr("No followers ready for FORMING state.")
             # return # Optional: Exit if no followers are ready

        rospy.loginfo(f"Starting Formation Control Phase with {num_transitioned} followers...")
        # ==============================================
        # FORMATION PHASE
        # ==============================================
        control_rate = rospy.Rate(rospy.get_param('~control_rate', DEFAULT_CONTROL_RATE))
        loop_count = 0 # Add loop counter for periodic logging

        while not rospy.is_shutdown():
            loop_start_time = rospy.Time.now()
            loop_count += 1

            # --- Leader ---
            leader_ready = leader.odom_ready and leader.scan_ready
            if leader_ready:
                 leader.move()
            else:
                 leader.stop()

            # --- Followers ---
            active_followers = 0
            for follower in followers:
                follower_ready = follower.sensors_ready
                # Check again if follower became ready late
                if not follower_ready: follower._check_sensors_ready(); follower_ready = follower.sensors_ready

                if follower_ready and follower.state != FollowerState.INITIALIZING:
                     follower.calculate_command(followers)
                     active_followers += 1
                elif follower_ready and follower.state == FollowerState.INITIALIZING:
                     # Should not happen here, but stop just in case
                     rospy.logwarn_throttle(10, f"Follower {follower.ns} stuck in INITIALIZING state during formation phase. Stopping.")
                     follower.stop()
                # else: follower not ready, do nothing

            # --- Periodic Logging ---
            if loop_count % (DEFAULT_CONTROL_RATE * 5) == 0: # Log every 5 seconds approx
                 rospy.loginfo(f"Main Loop {loop_count}: Leader Ready={leader_ready}, Active Followers={active_followers}/{len(followers)}")
                 # Add more detailed periodic status if needed

            # --- Sleep ---
            control_rate.sleep()
            loop_duration = (rospy.Time.now() - loop_start_time).to_sec()
            if loop_duration > (1.5 / DEFAULT_CONTROL_RATE): # Warn if loop takes too long
                 rospy.logwarn(f"Main loop iteration took longer than expected: {loop_duration:.4f}s")


    except KeyboardInterrupt: rospy.loginfo("Ctrl+C detected. Shutting down swarm...")
    except rospy.ROSInterruptException: rospy.loginfo("ROSInterruptException. Shutting down swarm...")
    except Exception as e: rospy.logerr(f"Unhandled exception in swarm controller: {e}", exc_info=True)
    finally:
        # --- Emergency Stop ---
        rospy.loginfo("Executing emergency stop for all robots.")
        if 'leader' in locals() and leader and leader.cmd_pub: leader.stop()
        if 'followers' in locals() and followers:
            for follower in followers:
                if follower and follower.cmd_pub: follower.stop()
        rospy.loginfo("Emergency stop commands sent.")


if __name__ == '__main__':
    try:
        swarm_controller()
    except Exception as main_e:
         print(f"Critical error during swarm controller startup: {main_e}")

