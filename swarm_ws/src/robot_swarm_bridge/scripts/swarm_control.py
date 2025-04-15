#!/usr/bin/env python3
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SwarmLeader:
    def __init__(self):
        # Load parameters with defaults
        self.linear_vel = rospy.get_param('~linear_vel', 0.15)
        self.angular_vel = rospy.get_param('~angular_vel', 0.3)
        # Change the default leader namespace from 'tb3_0' to 'robot/1'
        self.ns = rospy.get_param('~leader_ns', 'robot/1')

        # ROS setup
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb)
        
        # State variables
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.odom_ready = False

    def odom_cb(self, msg):
        """Update leader position and orientation"""
        self.position = (msg.pose.pose.position.x, 
                         msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]
        self.odom_ready = True

    def move(self):
        """Publish continuous circular motion"""
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = self.angular_vel
        self.cmd_pub.publish(cmd)


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

        # ROS interfaces
        self.cmd_pub = rospy.Publisher(f'/{self.ns}/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(f'/{self.ns}/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber(f'/{self.ns}/odom', Odometry, self.odom_cb)

        # State management
        self.obstacle_dist = float('inf')
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.sensors_ready = False

    def scan_cb(self, msg):
        """Process LIDAR data for obstacle detection"""
        if not msg.ranges:
            return

        # Analyze 120-degree front sector
        num_ranges = len(msg.ranges)
        front_sector = msg.ranges[num_ranges//3:2*num_ranges//3]
        valid = [r for r in front_sector if 0.1 < r < 3.5]
        self.obstacle_dist = min(valid) if valid else float('inf')

    def odom_cb(self, msg):
        """Update member position and orientation"""
        self.position = (msg.pose.pose.position.x,
                         msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])[2]
        self.sensors_ready = True

    def calculate_command(self):
        """Generate movement command with collision avoidance"""
        # Compute the desired position based on the leader's current position and heading.
        # The follower's desired position is at a fixed offset (formation_angle) relative to the leader's heading.
        desired_angle = self.leader.yaw + self.formation_angle
        target_x = self.leader.position[0] + (self.formation_radius + self.robot_spacing) * math.cos(desired_angle)
        target_y = self.leader.position[1] + (self.formation_radius + self.robot_spacing) * math.sin(desired_angle)

        # Compute position error.
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        dist_error = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)

        cmd = Twist()

        # Collision avoidance: if an obstacle is close, reduce forward speed
        # and add an additional angular rotation to help steer clear.
        if self.obstacle_dist < self.safe_dist:
            avoidance = 1.2 / (self.obstacle_dist + 0.1)
            cmd.linear.x = max(0, self.k_lin * dist_error * 0.5)
            cmd.angular.z = self.k_ang * angle_error + avoidance
        else:
            cmd.linear.x = self.k_lin * dist_error
            cmd.angular.z = self.k_ang * angle_error

        # Enforce safety limits.
        cmd.linear.x = max(-0.4, min(0.4, cmd.linear.x))
        cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
        return cmd


    @staticmethod
    def normalize_angle(angle):
        """Keep angles within [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))


def swarm_controller():
    rospy.init_node('intelligent_swarm_controller')

    # Initialize leader
    leader = SwarmLeader()
    if not rospy.wait_for_message(f'/{leader.ns}/odom', Odometry, timeout=60):
        rospy.logerr("Leader odometry not available!")
        return

    # Formation configuration
    num_followers = rospy.get_param('~num_followers', 3)
    base_radius = rospy.get_param('~formation_radius', 1.5)
    robot_spacing = rospy.get_param('~robot_spacing', 0.5)

    # Create followers
    followers = []
    angle_step = 2 * math.pi / num_followers
    # Since the leader is '/robot/1', we assign followers with namespaces '/robot/2', '/robot/3', etc.
    for i in range(num_followers):
        config = {
            'angle': angle_step * i,
            'radius': base_radius,
            'spacing': robot_spacing
        }
        follower_ns = f'robot/{i+2}'  # Start numbering followers from 2
        follower = SwarmMember(follower_ns, leader, config)
        if rospy.wait_for_message(f'/{follower.ns}/odom', Odometry, timeout=60):
            followers.append(follower)
        else:
            rospy.logwarn(f"Follower with namespace {follower_ns} not available!")

    control_rate = rospy.Rate(rospy.get_param('~control_rate', 15))
    try:
        rospy.loginfo("Swarm controller operational")
        while not rospy.is_shutdown():
            leader.move()
            for follower in followers:
                cmd = follower.calculate_command()
                follower.cmd_pub.publish(cmd)
            control_rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down swarm")
    finally:
        stop_cmd = Twist()
        leader.cmd_pub.publish(stop_cmd)
        for follower in followers:
            follower.cmd_pub.publish(stop_cmd)


if __name__ == '__main__':
    try:
        swarm_controller()
    except rospy.ROSInterruptException:
        pass
