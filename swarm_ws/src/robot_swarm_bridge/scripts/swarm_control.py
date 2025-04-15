#!/usr/bin/env python3
import rospy
import math
import tf
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SwarmLeader:
    def __init__(self):
        # Load parameters with defaults
        self.linear_vel = rospy.get_param('~linear_vel', 0.15)
        self.angular_vel = rospy.get_param('~angular_vel', 0.25)  # Slowed down for stability
        self.ns = rospy.get_param('~leader_ns', 'robot/1')
        
        # Track actual angular velocity
        self.last_yaw = 0.0
        self.last_yaw_time = time.time()
        self.actual_angular_vel = self.angular_vel
        
        # ROS setup
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb)
        self.scan_sub = rospy.Subscriber(f'/{self.ns}/scan', LaserScan, self.scan_cb)
        
        # State variables
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.odom_ready = False
        self.obstacle_detected = False

    def odom_cb(self, msg):
        """Update leader position and orientation"""
        self.position = (msg.pose.pose.position.x, 
                         msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        current_yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]
        
        # Calculate actual angular velocity for more accurate formation
        current_time = time.time()
        dt = current_time - self.last_yaw_time
        if dt > 0.01:  # Only update if enough time has passed
            yaw_change = self.normalize_angle(current_yaw - self.last_yaw)
            self.actual_angular_vel = 0.7 * self.actual_angular_vel + 0.3 * (yaw_change / dt)  # Smooth
            self.last_yaw = current_yaw
            self.last_yaw_time = current_time
            
        self.yaw = current_yaw
        self.odom_ready = True

    def scan_cb(self, msg):
        """Simple obstacle detection for leader"""
        if not msg.ranges:
            return
            
        # Check front 90° for obstacles
        num_ranges = len(msg.ranges)
        front_indices = list(range(-num_ranges//8, num_ranges//8))
        front_ranges = [msg.ranges[i % num_ranges] for i in front_indices]
        valid_ranges = [r for r in front_ranges if 0.1 < r < 0.5]
        
        self.obstacle_detected = len(valid_ranges) > 0

    def move(self):
        """Publish continuous circular motion with basic obstacle avoidance"""
        cmd = Twist()
        
        if self.obstacle_detected:
            # Simple obstacle avoidance - turn more sharply
            cmd.linear.x = self.linear_vel * 0.5
            cmd.angular.z = self.angular_vel * 1.5
        else:
            cmd.linear.x = self.linear_vel
            cmd.angular.z = self.angular_vel
            
        self.cmd_pub.publish(cmd)
        
    def stop(self):
        """Stop the leader"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
    @staticmethod
    def normalize_angle(angle):
        """Keep angles within [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))


class SwarmMember:
    def __init__(self, namespace, leader, formation_config):
        # Configuration
        self.ns = namespace
        self.leader = leader
        self.formation_angle = formation_config['angle']
        self.formation_radius = formation_config['radius']
        self.robot_spacing = formation_config['spacing']

        # Control parameters
        self.k_lin = rospy.get_param('~k_linear', 0.6)
        self.k_ang = rospy.get_param('~k_angular', 1.2)
        self.safe_dist = rospy.get_param('~safety_distance', 0.7)
        
        # Time tracking for formation
        self.last_update_time = rospy.Time.now().to_sec()
        
        # Recovery behavior
        self.stuck_time = None
        self.recovery_mode = False
        self.last_dist_error = float('inf')
        self.recovery_timeout = rospy.get_param('~recovery_timeout', 5.0)
        
        # Initialization status
        self.initialized = False
        self.init_position_reached = False

        # ROS interfaces
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(f'/{self.ns}/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb)

        # State management
        self.obstacle_dists = {}  # Store obstacles by angle sector
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.sensors_ready = False

    def scan_cb(self, msg):
        """Process LIDAR data for 360° obstacle detection"""
        if not msg.ranges:
            return
            
        num_ranges = len(msg.ranges)
        # Divide the 360° scan into 8 sectors
        sector_size = num_ranges // 8
        
        # Reset obstacle map
        self.obstacle_dists = {}
        
        # Process each sector
        for i in range(8):
            start_idx = i * sector_size
            end_idx = (i+1) * sector_size
            
            # Get valid ranges in this sector
            sector_ranges = msg.ranges[start_idx:end_idx]
            valid_ranges = [r for r in sector_ranges if 0.1 < r < 3.5]
            
            if valid_ranges:
                # Store minimum distance for this sector
                angle = i * math.pi/4  # Convert sector to angle (0 is front)
                self.obstacle_dists[angle] = min(valid_ranges)
        
        self.sensors_ready = True

    def odom_cb(self, msg):
        """Update member position and orientation"""
        self.position = (msg.pose.pose.position.x,
                         msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]
        self.sensors_ready = True

    def calculate_init_command(self, phase_progress):
        """Generate command for initialization phase"""
        # Calculate a temporary gathering point at a safe distance from leader
        # Use phase_progress (0-1) to gradually move from current pos to target formation
        
        # Phase 1: Move closer to leader first (at reduced radius)
        init_radius = self.formation_radius * (0.5 + 0.5 * phase_progress)  # Start at 50% of final radius
        
        # Compute target position relative to leader
        init_angle = self.formation_angle
        target_x = self.leader.position[0] + init_radius * math.cos(init_angle)
        target_y = self.leader.position[1] + init_radius * math.sin(init_angle)
        
        # Compute error
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        dist_error = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)
        
        # Check if close enough to initial position
        if dist_error < 0.2:
            self.init_position_reached = True
            
        # Create command
        cmd = Twist()
        
        # Consider obstacles during initialization too
        if self.get_min_obstacle_distance() < self.safe_dist * 0.8:
            # Emergency obstacle avoidance
            obstacle_dist = self.get_min_obstacle_distance()
            obstacle_angle = self.get_closest_obstacle_angle()
            
            avoidance_gain = 1.5 / (obstacle_dist + 0.1)
            cmd.linear.x = 0.1  # Slow movement during avoidance
            cmd.angular.z = -avoidance_gain * math.copysign(1.0, obstacle_angle)
        else:
            # Normal initialization movement
            # Use higher gain during initialization for faster convergence
            cmd.linear.x = 0.3 * math.tanh(2.0 * dist_error)  # Limit speed for stability
            cmd.angular.z = 1.5 * angle_error  # Higher angular gain for quicker alignment
        
        # Safety limits during initialization
        cmd.linear.x = max(-0.2, min(0.2, cmd.linear.x))
        cmd.angular.z = max(-0.8, min(0.8, cmd.angular.z))
        
        return cmd

    def calculate_command(self, all_followers=None):
        """Generate movement command with enhanced collision avoidance"""
        # Get current time for time-based formation
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_update_time
        
        # Calculate desired position using time-based rotation
        # Use leader's actual angular velocity for smoother formation
        phase_offset = self.leader.yaw + self.leader.actual_angular_vel * dt
        desired_angle = phase_offset + self.formation_angle
        
        target_x = self.leader.position[0] + (self.formation_radius + self.robot_spacing) * math.cos(desired_angle)
        target_y = self.leader.position[1] + (self.formation_radius + self.robot_spacing) * math.sin(desired_angle)
        
        # Update time
        self.last_update_time = current_time

        # Compute error
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        dist_error = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)
        
        # Check for leader collision
        leader_dist = math.hypot(
            self.position[0] - self.leader.position[0],
            self.position[1] - self.leader.position[1]
        )
        leader_angle = math.atan2(
            self.leader.position[1] - self.position[1], 
            self.leader.position[0] - self.position[0]
        )
        relative_leader_angle = self.normalize_angle(leader_angle - self.yaw)
        
        # Check for other robot collisions
        robot_collision = False
        closest_robot_dist = float('inf')
        closest_robot_angle = 0
        
        if all_followers:
            for other in all_followers:
                if other.ns == self.ns:
                    continue  # Skip self
                    
                # Calculate distance to other robot
                robot_dist = math.hypot(
                    self.position[0] - other.position[0],
                    self.position[1] - other.position[1]
                )
                
                if robot_dist < 0.5:  # Detection threshold
                    robot_collision = True
                    if robot_dist < closest_robot_dist:
                        closest_robot_dist = robot_dist
                        closest_robot_angle = math.atan2(
                            other.position[1] - self.position[1], 
                            other.position[0] - self.position[0]
                        )
                        closest_robot_angle = self.normalize_angle(closest_robot_angle - self.yaw)
        
        # Update stuck detection
        if abs(dist_error - self.last_dist_error) < 0.02 and dist_error > 0.2:
            if self.stuck_time is None:
                self.stuck_time = current_time
            elif current_time - self.stuck_time > self.recovery_timeout:
                self.recovery_mode = True
        else:
            self.stuck_time = None
            self.recovery_mode = False
            
        self.last_dist_error = dist_error

        # Create command
        cmd = Twist()
        
        # Prioritized collision avoidance
        if leader_dist < self.safe_dist * 0.7:
            # 1. Avoid leader collision (highest priority)
            avoidance_gain = 1.5 / (leader_dist + 0.1)
            cmd.linear.x = 0.05  # Slow down
            # Turn away from leader
            cmd.angular.z = -1.2 * avoidance_gain * math.copysign(1.0, relative_leader_angle)
            rospy.logwarn(f"{self.ns} avoiding leader! Distance: {leader_dist:.2f}")
            
        elif robot_collision:
            # 2. Avoid other robots (second priority)
            avoidance_gain = 1.0 / (closest_robot_dist + 0.1)
            cmd.linear.x = 0.1
            # Turn away from other robot
            cmd.angular.z = -1.0 * avoidance_gain * math.copysign(1.0, closest_robot_angle)
            rospy.logwarn(f"{self.ns} avoiding robot collision! Distance: {closest_robot_dist:.2f}")
            
        elif self.get_min_obstacle_distance() < self.safe_dist:
            # 3. Avoid environmental obstacles (third priority)
            obstacle_dist = self.get_min_obstacle_distance()
            obstacle_angle = self.get_closest_obstacle_angle()
            
            avoidance_gain = 1.2 / (obstacle_dist + 0.1)
            cmd.linear.x = max(0, self.k_lin * dist_error * 0.5)
            
            # Turn away from obstacle, biased by formation goal
            obstacle_avoidance = -avoidance_gain * math.copysign(1.0, obstacle_angle)
            formation_influence = 0.3 * self.k_ang * angle_error  # Reduced weight
            cmd.angular.z = obstacle_avoidance + formation_influence
            
        elif self.recovery_mode:
            # 4. Recovery behavior for stuck robots
            rospy.logwarn(f"{self.ns} in recovery mode!")
            cmd.linear.x = 0.15
            # Use time to create varying recovery pattern
            recovery_pattern = math.sin(current_time * 0.5)
            cmd.angular.z = recovery_pattern
            
        else:
            # 5. Normal formation control (lowest priority)
            cmd.linear.x = self.k_lin * dist_error
            cmd.angular.z = self.k_ang * angle_error

        # Safety limits
        cmd.linear.x = max(-0.3, min(0.3, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        return cmd
        
    def get_min_obstacle_distance(self):
        """Get minimum obstacle distance from any direction"""
        if not self.obstacle_dists:
            return float('inf')
        return min(self.obstacle_dists.values())
        
    def get_closest_obstacle_angle(self):
        """Get angle of the closest obstacle"""
        if not self.obstacle_dists:
            return 0.0
        
        min_dist = float('inf')
        min_angle = 0.0
        
        for angle, dist in self.obstacle_dists.items():
            if dist < min_dist:
                min_dist = dist
                min_angle = angle
                
        # Convert to robot-relative angle
        return self.normalize_angle(min_angle)

    @staticmethod
    def normalize_angle(angle):
        """Keep angles within [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))


def swarm_controller():
    rospy.init_node('staged_swarm_controller')

    try:
        # Initialize leader
        leader = SwarmLeader()
        
        # Wait for leader odometry
        rospy.loginfo("Waiting for leader odometry...")
        odom_msg_received = False
        
        # Try both namespace formats
        namespace_formats = [
            leader.ns,  # Try original format first (robot/1)
            f"tb3_{leader.ns.split('/')[-1]}", # Try tb3_X format
            f"robot_{leader.ns.split('/')[-1]}" # Try robot_X format (matching your GUI)
        ]
        
        for ns_format in namespace_formats:
            try:
                if rospy.wait_for_message(f'/{ns_format}/odom', Odometry, timeout=60):
                    leader.ns = ns_format
                    leader.cmd_pub = rospy.Publisher(f'/{leader.ns}/cmd_vel', Twist, queue_size=1)
                    leader.odom_sub = rospy.Subscriber(f'/{leader.ns}/odom', Odometry, leader.odom_cb)
                    leader.scan_sub = rospy.Subscriber(f'/{leader.ns}/scan', LaserScan, leader.scan_cb)
                    odom_msg_received = True
                    rospy.loginfo(f"Connected to leader with namespace: {leader.ns}")
                    break
            except rospy.ROSException:
                continue
                
        if not odom_msg_received:
            rospy.logerr("Could not connect to leader with any namespace format!")
            return
            
        # Wait for leader odometry data to be processed
        rospy.sleep(0.5)
        
        # Formation configuration
        num_followers = rospy.get_param('~num_followers', 5)  # Default to 5
        base_radius = rospy.get_param('~formation_radius', 1.8)  # Larger radius for 5 robots
        robot_spacing = rospy.get_param('~robot_spacing', 0.4)
        
        # Create possible namespace formats for followers
        base_formats = [
            lambda i: f'robot/{i+2}',    # robot/X format
            lambda i: f'tb3_{i+1}',      # tb3_X format
            lambda i: f'robot_{i+2}'     # robot_X format (matching your GUI)
        ]

        # Create followers
        followers = []
        angle_step = 2 * math.pi / num_followers
        
        for i in range(num_followers):
            config = {
                'angle': angle_step * i,
                'radius': base_radius,
                'spacing': robot_spacing
            }
            
            follower_created = False
            
            # Try each namespace format for this follower
            for ns_format in base_formats:
                follower_ns = ns_format(i)
                try:
                    # Check if this namespace exists
                    if rospy.wait_for_message(f'/{follower_ns}/odom', Odometry, timeout=60):
                        follower = SwarmMember(follower_ns, leader, config)
                        followers.append(follower)
                        rospy.loginfo(f"Added follower {follower_ns}")
                        follower_created = True
                        break
                except rospy.ROSException:
                    continue
                    
            if not follower_created:
                rospy.logwarn(f"Could not add follower {i+1} - no matching namespace found")
        
        if not followers:
            rospy.logerr("No followers could be initialized!")
            return
            
        rospy.loginfo(f"Swarm initialized with {len(followers)} followers")

        # ==============================================
        # INITIALIZATION PHASE
        # ==============================================
        rospy.loginfo("Starting initialization phase...")
        
        # Stop the leader during initialization
        leader.stop()
        
        # Configuration for initialization
        init_duration = rospy.get_param('~init_duration', 15.0)  # 15 seconds for initialization
        rate = rospy.Rate(10)  # 10Hz for initialization
        
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < init_duration and not rospy.is_shutdown():
            # Calculate phase progress (0 to 1)
            phase_progress = min(1.0, (rospy.Time.now() - start_time).to_sec() / init_duration)
            
            # Keep leader stationary
            leader.stop()
            
            # Move followers to initial positions
            for follower in followers:
                cmd = follower.calculate_init_command(phase_progress)
                follower.cmd_pub.publish(cmd)
                
            # Check if all followers are in position
            all_positioned = all(follower.init_position_reached for follower in followers)
            if all_positioned and phase_progress > 0.5:  # At least 50% of init time passed
                rospy.loginfo("All followers in position, ending initialization phase")
                break
                
            rate.sleep()
            
        # Mark all followers as initialized
        for follower in followers:
            follower.initialized = True
            
        rospy.loginfo("Initialization complete, starting formation control")
        
        # ==============================================
        # FORMATION PHASE
        # ==============================================
        
        # Control loop
        control_rate = rospy.Rate(rospy.get_param('~control_rate', 15))
        
        rospy.loginfo("Swarm controller operational")
        while not rospy.is_shutdown():
            # Leader moves
            leader.move()
            
            # Followers maintain formation
            for follower in followers:
                cmd = follower.calculate_command(followers)
                follower.cmd_pub.publish(cmd)
                
            control_rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down swarm")
    except Exception as e:
        rospy.logerr(f"Error in swarm controller: {e}")
    finally:
        # Emergency stop all robots
        try:
            stop_cmd = Twist()
            if 'leader' in locals():
                leader.cmd_pub.publish(stop_cmd)
            if 'followers' in locals():
                for follower in followers:
                    follower.cmd_pub.publish(stop_cmd)
        except:
            pass


if __name__ == '__main__':
    try:
        swarm_controller()
    except rospy.ROSInterruptException:
        pass