#!/usr/bin/env python3
"""
Synthetic Robot Data Generator for Uncertainty-Aware SLAM Testing
Generates realistic laser scan data and robot motion without requiring a simulator.
Allows full control via keyboard or autonomous exploration patterns.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from std_msgs.msg import String
import tf2_ros
import numpy as np
import math
import sys
import termios
import tty
import select


class VirtualEnvironment:
    """Simulates a 2D environment with walls and obstacles."""

    def __init__(self, width=12.0, height=12.0):
        self.width = width
        self.height = height
        self.obstacles = []
        self._create_default_environment()

    def _create_default_environment(self):
        """Create a room with obstacles matching cave.world."""
        # Outer walls
        self.obstacles.append({'type': 'wall', 'x1': -6, 'y1': -6, 'x2': 6, 'y2': -6})  # Bottom
        self.obstacles.append({'type': 'wall', 'x1': 6, 'y1': -6, 'x2': 6, 'y2': 6})    # Right
        self.obstacles.append({'type': 'wall', 'x1': 6, 'y1': 6, 'x2': -6, 'y2': 6})    # Top
        self.obstacles.append({'type': 'wall', 'x1': -6, 'y1': 6, 'x2': -6, 'y2': -6})  # Left

        # Interior obstacles (blocks)
        self.obstacles.append({'type': 'box', 'cx': -2, 'cy': -2, 'w': 0.5, 'h': 0.5})
        self.obstacles.append({'type': 'box', 'cx': 2, 'cy': -2, 'w': 0.5, 'h': 0.5})
        self.obstacles.append({'type': 'box', 'cx': -2, 'cy': 2, 'w': 0.5, 'h': 0.5})
        self.obstacles.append({'type': 'box', 'cx': 2, 'cy': 2, 'w': 0.5, 'h': 0.5})
        self.obstacles.append({'type': 'box', 'cx': 0, 'cy': 0, 'w': 1.0, 'h': 1.0})
        self.obstacles.append({'type': 'box', 'cx': -4, 'cy': 0, 'w': 0.5, 'h': 0.5})
        self.obstacles.append({'type': 'box', 'cx': 4, 'cy': 0, 'w': 0.5, 'h': 0.5})

    def raycast(self, x, y, angle, max_range=10.0):
        """Cast a ray from (x,y) at given angle and return distance to nearest obstacle."""
        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        min_dist = max_range

        # Check all obstacles
        for obs in self.obstacles:
            if obs['type'] == 'wall':
                # Line-line intersection
                dist = self._ray_wall_intersection(x, y, dx, dy,
                                                   obs['x1'], obs['y1'],
                                                   obs['x2'], obs['y2'])
                if dist is not None and dist < min_dist:
                    min_dist = dist

            elif obs['type'] == 'box':
                # Check all 4 walls of the box
                cx, cy, w, h = obs['cx'], obs['cy'], obs['w'], obs['h']
                walls = [
                    (cx - w/2, cy - h/2, cx + w/2, cy - h/2),  # Bottom
                    (cx + w/2, cy - h/2, cx + w/2, cy + h/2),  # Right
                    (cx + w/2, cy + h/2, cx - w/2, cy + h/2),  # Top
                    (cx - w/2, cy + h/2, cx - w/2, cy - h/2),  # Left
                ]
                for x1, y1, x2, y2 in walls:
                    dist = self._ray_wall_intersection(x, y, dx, dy, x1, y1, x2, y2)
                    if dist is not None and dist < min_dist:
                        min_dist = dist

        # Add noise
        noise = np.random.normal(0, 0.01)  # 1cm noise
        return max(0.1, min_dist + noise)

    def _ray_wall_intersection(self, rx, ry, rdx, rdy, wx1, wy1, wx2, wy2):
        """Calculate intersection distance between ray and wall segment."""
        # Wall direction
        wdx = wx2 - wx1
        wdy = wy2 - wy1

        # Solve: ray_origin + t1 * ray_dir = wall_start + t2 * wall_dir
        denominator = rdx * wdy - rdy * wdx

        if abs(denominator) < 1e-10:  # Parallel
            return None

        t1 = ((wx1 - rx) * wdy - (wy1 - ry) * wdx) / denominator
        t2 = ((wx1 - rx) * rdy - (wy1 - ry) * rdx) / denominator

        # Check if intersection is in valid range
        if t1 >= 0 and 0 <= t2 <= 1:
            return t1

        return None

    def is_collision(self, x, y, robot_radius=0.18):
        """Check if robot at (x,y) collides with any obstacle."""
        # Check walls
        if (x - robot_radius <= -self.width/2 or
            x + robot_radius >= self.width/2 or
            y - robot_radius <= -self.height/2 or
            y + robot_radius >= self.height/2):
            return True

        # Check boxes
        for obs in self.obstacles:
            if obs['type'] == 'box':
                cx, cy, w, h = obs['cx'], obs['cy'], obs['w'], obs['h']
                # Expanded box check (box + robot radius)
                if (abs(x - cx) < w/2 + robot_radius and
                    abs(y - cy) < h/2 + robot_radius):
                    return True

        return False


class SyntheticRobotNode(Node):
    """
    Synthetic robot that publishes laser scans, odometry, and TF.
    Supports keyboard control and autonomous exploration modes.
    """

    def __init__(self):
        super().__init__('synthetic_robot')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber for autonomous goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/exploration_goal',
            self.goal_callback,
            10
        )

        # Robot state
        self.x = -4.0
        self.y = -4.0
        self.theta = math.pi / 4
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Control parameters
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0
        self.linear_accel = 0.3
        self.angular_accel = 0.5

        # Mode: 'manual', 'autonomous', 'exploration_pattern'
        self.mode = 'manual'
        self.autonomous_goal = None
        self.exploration_pattern_index = 0

        # Environment
        self.environment = VirtualEnvironment()

        # Laser scan parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 0.01  # ~628 rays
        self.range_min = 0.1
        self.range_max = 10.0

        # Timers
        self.dt = 0.05  # 20 Hz
        self.create_timer(self.dt, self.update_robot)
        self.create_timer(0.1, self.publish_scan)  # 10 Hz scan

        # Keyboard control (non-blocking)
        self.cmd_vel = {'linear': 0.0, 'angular': 0.0}

        self.get_logger().info('=== Synthetic Robot Started ===')
        self.get_logger().info('Mode: MANUAL (use keyboard)')
        self.get_logger().info('Controls:')
        self.get_logger().info('  w/x: forward/backward')
        self.get_logger().info('  a/d: rotate left/right')
        self.get_logger().info('  s: stop')
        self.get_logger().info('  m: switch to manual mode')
        self.get_logger().info('  e: switch to exploration pattern mode')
        self.get_logger().info('  q: quit')
        self.get_logger().info(f'Starting position: ({self.x:.2f}, {self.y:.2f})')

        # Start keyboard listener
        self.create_timer(0.05, self.check_keyboard)

    def goal_callback(self, msg):
        """Receive autonomous exploration goals."""
        self.autonomous_goal = (msg.pose.position.x, msg.pose.position.y)
        if self.mode != 'autonomous':
            self.mode = 'autonomous'
            self.get_logger().info(f'Switching to AUTONOMOUS mode, goal: {self.autonomous_goal}')
            self.publish_mode()

    def check_keyboard(self):
        """Non-blocking keyboard input check."""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                key = sys.stdin.read(1)
                self.handle_key(key)
            except:
                pass

    def handle_key(self, key):
        """Process keyboard commands."""
        if key == 'w':
            self.cmd_vel['linear'] = self.max_linear_vel
            self.mode = 'manual'
        elif key == 'x':
            self.cmd_vel['linear'] = -self.max_linear_vel
            self.mode = 'manual'
        elif key == 'a':
            self.cmd_vel['angular'] = self.max_angular_vel
            self.mode = 'manual'
        elif key == 'd':
            self.cmd_vel['angular'] = -self.max_angular_vel
            self.mode = 'manual'
        elif key == 's':
            self.cmd_vel['linear'] = 0.0
            self.cmd_vel['angular'] = 0.0
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        elif key == 'm':
            self.mode = 'manual'
            self.autonomous_goal = None
            self.get_logger().info('Switched to MANUAL mode')
            self.publish_mode()
        elif key == 'e':
            self.mode = 'exploration_pattern'
            self.exploration_pattern_index = 0
            self.get_logger().info('Switched to EXPLORATION PATTERN mode')
            self.publish_mode()
        elif key == 'q':
            self.get_logger().info('Quitting...')
            rclpy.shutdown()

    def publish_mode(self):
        """Publish current robot mode."""
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def update_robot(self):
        """Update robot state based on current mode."""
        if self.mode == 'manual':
            # Manual control from keyboard
            target_linear = self.cmd_vel['linear']
            target_angular = self.cmd_vel['angular']

        elif self.mode == 'autonomous':
            # Navigate to autonomous goal
            if self.autonomous_goal is None:
                target_linear = 0.0
                target_angular = 0.0
            else:
                target_linear, target_angular = self.compute_control_to_goal(
                    self.autonomous_goal[0], self.autonomous_goal[1]
                )

                # Check if reached goal
                dist = math.sqrt(
                    (self.x - self.autonomous_goal[0])**2 +
                    (self.y - self.autonomous_goal[1])**2
                )
                if dist < 0.2:
                    self.get_logger().info('Goal reached!')
                    self.autonomous_goal = None
                    self.mode = 'manual'
                    self.publish_mode()

        elif self.mode == 'exploration_pattern':
            # Follow predefined exploration pattern
            target_linear, target_angular = self.exploration_pattern()

        else:
            target_linear = 0.0
            target_angular = 0.0

        # Apply acceleration limits
        if target_linear > self.linear_vel:
            self.linear_vel = min(self.linear_vel + self.linear_accel * self.dt, target_linear)
        else:
            self.linear_vel = max(self.linear_vel - self.linear_accel * self.dt, target_linear)

        if target_angular > self.angular_vel:
            self.angular_vel = min(self.angular_vel + self.angular_accel * self.dt, target_angular)
        else:
            self.angular_vel = max(self.angular_vel - self.angular_accel * self.dt, target_angular)

        # Update pose
        new_x = self.x + self.linear_vel * math.cos(self.theta) * self.dt
        new_y = self.y + self.linear_vel * math.sin(self.theta) * self.dt
        new_theta = self.theta + self.angular_vel * self.dt

        # Check collision
        if not self.environment.is_collision(new_x, new_y):
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
        else:
            # Collision - stop and rotate
            self.linear_vel = 0.0
            self.angular_vel = 0.5
            self.get_logger().warn('Collision detected! Rotating...')

        # Publish odometry and TF
        self.publish_odometry()
        self.publish_tf()

    def compute_control_to_goal(self, goal_x, goal_y):
        """Simple proportional controller to navigate to goal."""
        # Distance and angle to goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Angle error
        angle_error = angle_to_goal - self.theta
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Control gains
        k_linear = 0.5
        k_angular = 2.0

        # If angle error is large, rotate in place
        if abs(angle_error) > 0.3:
            linear = 0.0
            angular = k_angular * angle_error
        else:
            linear = k_linear * distance
            angular = k_angular * angle_error

        # Clamp to limits
        linear = max(-self.max_linear_vel, min(self.max_linear_vel, linear))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))

        return linear, angular

    def exploration_pattern(self):
        """Execute systematic exploration pattern."""
        # Waypoints for systematic exploration
        waypoints = [
            (-4, -4), (4, -4), (4, 4), (-4, 4),  # Perimeter
            (-2, -2), (2, -2), (2, 2), (-2, 2),  # Inner square
            (0, -4), (4, 0), (0, 4), (-4, 0),    # Cross pattern
        ]

        # Current target
        target = waypoints[self.exploration_pattern_index % len(waypoints)]

        # Distance to target
        dist = math.sqrt((self.x - target[0])**2 + (self.y - target[1])**2)

        # If reached, move to next waypoint
        if dist < 0.3:
            self.exploration_pattern_index += 1
            self.get_logger().info(f'Waypoint {self.exploration_pattern_index} reached')
            target = waypoints[self.exploration_pattern_index % len(waypoints)]

        # Compute control
        return self.compute_control_to_goal(target[0], target[1])

    def publish_scan(self):
        """Generate and publish laser scan."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Generate ranges
        ranges = []
        num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)

        for i in range(num_rays):
            angle = self.angle_min + i * self.angle_increment
            world_angle = self.theta + angle
            distance = self.environment.raycast(self.x, self.y, world_angle, self.range_max)
            ranges.append(float(distance))

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_odometry(self):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom)

    def publish_tf(self):
        """Publish TF transforms."""
        now = self.get_clock().now().to_msg()

        # map -> odom (static)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0

        # odom -> base_footprint
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_footprint'
        t2.transform.translation.x = self.x
        t2.transform.translation.y = self.y
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = math.sin(self.theta / 2)
        t2.transform.rotation.w = math.cos(self.theta / 2)

        # base_footprint -> base_scan (laser scanner offset)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_footprint'
        t3.child_frame_id = 'base_scan'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.2
        t3.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t1, t2, t3])


def main(args=None):
    """Main entry point."""
    # Set terminal to raw mode for keyboard input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init(args=args)
        node = SyntheticRobotNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
