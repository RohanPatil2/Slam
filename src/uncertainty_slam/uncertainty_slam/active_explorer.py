#!/usr/bin/env python3
"""
Active Exploration Node

This node implements uncertainty-driven active exploration.
It subscribes to the entropy map and directs the robot to
high-entropy regions to reduce map uncertainty.

This demonstrates the utility of the entropy map for active sensing.

Author: Rohan Upendra Patil
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Bool
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import math


class ActiveExplorer(Node):
    """
    Active exploration using entropy-driven navigation.

    Finds the nearest high-entropy region and navigates towards it
    to reduce map uncertainty.
    """

    def __init__(self):
        super().__init__('active_explorer')

        # Declare parameters
        self.declare_parameter('entropy_map_topic', '/entropy_map')
        self.declare_parameter('occupancy_map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('min_entropy_threshold', 50.0)  # Out of 100
        self.declare_parameter('max_exploration_distance', 5.0)  # meters
        self.declare_parameter('control_frequency', 10.0)  # Hz
        self.declare_parameter('linear_velocity', 0.3)  # m/s
        self.declare_parameter('angular_velocity', 0.5)  # rad/s
        self.declare_parameter('goal_tolerance', 0.3)  # meters
        self.declare_parameter('exploration_enabled', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        # Get parameters
        entropy_topic = self.get_parameter('entropy_map_topic').value
        occupancy_topic = self.get_parameter('occupancy_map_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.min_entropy = self.get_parameter('min_entropy_threshold').value
        self.max_distance = self.get_parameter('max_exploration_distance').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.exploration_enabled = self.get_parameter('exploration_enabled').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        # State variables
        self.entropy_map = None
        self.occupancy_map = None
        self.current_goal = None
        self.robot_pose = None
        self.entropy_map_lock = False
        self.occupancy_map_lock = False

        # TF2 for robot localization
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.entropy_sub = self.create_subscription(
            OccupancyGrid,
            entropy_topic,
            self.entropy_callback,
            10
        )

        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            occupancy_topic,
            self.occupancy_callback,
            10
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/active_exploration/enable',
            self.enable_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/active_exploration/current_goal',
            10
        )

        self.status_pub = self.create_publisher(
            Float64,
            '/active_exploration/goal_entropy',
            10
        )

        # Control timer
        timer_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        # Goal selection timer (slower)
        self.goal_timer = self.create_timer(2.0, self.select_high_entropy_goal)

        self.get_logger().info('Active Explorer initialized')
        self.get_logger().info(f'Exploration enabled: {self.exploration_enabled}')

    def entropy_callback(self, msg):
        """Receive entropy map updates."""
        if not self.entropy_map_lock:
            self.entropy_map = msg

    def occupancy_callback(self, msg):
        """Receive occupancy map updates."""
        if not self.occupancy_map_lock:
            self.occupancy_map = msg

    def enable_callback(self, msg):
        """Enable/disable exploration."""
        self.exploration_enabled = msg.data
        self.get_logger().info(f'Exploration {"enabled" if msg.data else "disabled"}')

        if not msg.data:
            # Stop robot
            self.publish_velocity(0.0, 0.0)
            self.current_goal = None

    def get_robot_pose(self):
        """
        Get current robot pose from TF.

        Returns:
            tuple: (x, y, theta) in map frame, or None if unavailable
        """
        try:
            # Use zero time for latest available transform
            # This works properly with use_sim_time=True
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw from quaternion
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return (x, y, theta)

        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {str(e)}')
            return None

    def world_to_map(self, x, y, map_msg):
        """
        Convert world coordinates to map indices.

        Args:
            x, y: World coordinates (meters)
            map_msg: OccupancyGrid message

        Returns:
            tuple: (map_x, map_y) indices, or None if out of bounds
        """
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        # Check bounds
        if (0 <= map_x < map_msg.info.width and
            0 <= map_y < map_msg.info.height):
            return (map_x, map_y)

        return None

    def map_to_world(self, map_x, map_y, map_msg):
        """
        Convert map indices to world coordinates.

        Args:
            map_x, map_y: Map indices
            map_msg: OccupancyGrid message

        Returns:
            tuple: (x, y) world coordinates
        """
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        x = origin_x + (map_x + 0.5) * resolution
        y = origin_y + (map_y + 0.5) * resolution

        return (x, y)

    def is_cell_free(self, map_x, map_y, map_msg):
        """
        Check if a map cell is free space.

        Args:
            map_x, map_y: Map indices
            map_msg: OccupancyGrid message

        Returns:
            bool: True if free, False otherwise
        """
        if map_msg is None:
            return False

        idx = map_y * map_msg.info.width + map_x
        if 0 <= idx < len(map_msg.data):
            return map_msg.data[idx] == 0  # 0 = free space
        return False

    def select_high_entropy_goal(self):
        """
        Select a high-entropy region as the next goal.

        Uses a simple strategy:
        1. Find all cells with entropy > threshold
        2. Filter by occupancy (must be navigable)
        3. Select nearest high-entropy cell within max_distance
        """
        if not self.exploration_enabled:
            return

        if self.entropy_map is None or self.occupancy_map is None:
            return

        # Get robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, robot_theta = robot_pose

        # Find high-entropy candidates
        entropy_array = np.array(self.entropy_map.data, dtype=np.float32)
        width = self.entropy_map.info.width
        height = self.entropy_map.info.height

        # Reshape to 2D
        entropy_grid = entropy_array.reshape((height, width))

        # Find cells above threshold
        high_entropy_mask = entropy_grid > self.min_entropy

        # Also check occupancy (must be free or unknown)
        occupancy_array = np.array(self.occupancy_map.data, dtype=np.int8)
        occupancy_grid = occupancy_array.reshape((height, width))
        navigable_mask = (occupancy_grid == 0) | (occupancy_grid == -1)

        # Combine masks
        candidate_mask = high_entropy_mask & navigable_mask

        # Get candidate coordinates
        candidates_y, candidates_x = np.where(candidate_mask)

        if len(candidates_x) == 0:
            self.get_logger().debug('No high-entropy regions found')
            return

        # Convert to world coordinates and compute distances
        best_goal = None
        best_distance = float('inf')
        best_entropy = 0.0

        for map_x, map_y in zip(candidates_x, candidates_y):
            world_x, world_y = self.map_to_world(map_x, map_y, self.entropy_map)

            # Compute distance
            distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)

            # Check distance constraint
            if distance > self.max_distance:
                continue

            # Get entropy value
            idx = map_y * width + map_x
            entropy_value = entropy_grid[map_y, map_x]

            # Select closest high-entropy cell (could use other criteria)
            if distance < best_distance:
                best_distance = distance
                best_goal = (world_x, world_y)
                best_entropy = entropy_value

        # Set new goal if found
        if best_goal is not None and best_goal != self.current_goal:
            self.current_goal = best_goal
            self.get_logger().info(
                f'New high-entropy goal: ({best_goal[0]:.2f}, {best_goal[1]:.2f}), '
                f'entropy: {best_entropy:.1f}, distance: {best_distance:.2f}m'
            )

            # Publish goal for visualization
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.map_frame
            goal_msg.pose.position.x = best_goal[0]
            goal_msg.pose.position.y = best_goal[1]
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)

            # Publish goal entropy
            entropy_msg = Float64()
            entropy_msg.data = float(best_entropy)
            self.status_pub.publish(entropy_msg)

    def control_loop(self):
        """
        Simple proportional controller to navigate to goal.

        Uses differential drive kinematics.
        """
        if not self.exploration_enabled or self.current_goal is None:
            return

        # Get current robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, robot_theta = robot_pose
        goal_x, goal_y = self.current_goal

        # Compute error
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if goal reached
        if distance < self.goal_tolerance:
            self.get_logger().info('Goal reached! Searching for next high-entropy region...')
            self.publish_velocity(0.0, 0.0)
            self.current_goal = None
            return

        # Compute desired heading
        desired_theta = math.atan2(dy, dx)

        # Compute angular error
        angle_error = desired_theta - robot_theta

        # Normalize angle to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Simple proportional control
        angular_gain = 2.0
        linear_gain = 0.5

        # Angular velocity
        angular = angular_gain * angle_error
        angular = np.clip(angular, -self.angular_vel, self.angular_vel)

        # Linear velocity (reduce when turning)
        if abs(angle_error) > 0.5:  # > ~30 degrees, rotate in place
            linear = 0.0
        else:
            linear = linear_gain * distance
            linear = np.clip(linear, 0.0, self.linear_vel)

        # Publish velocity
        self.publish_velocity(linear, angular)

    def publish_velocity(self, linear, angular):
        """Publish velocity command."""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ActiveExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on exit
        node.publish_velocity(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
