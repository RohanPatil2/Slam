#!/usr/bin/env python3
"""
Advanced Active Explorer with A* Path Planning

Implements sophisticated entropy-driven exploration with:
- A* path planning for collision-free navigation
- Frontier detection for efficient exploration
- Information gain estimation
- Dynamic goal selection

Author: Rohan Upendra Patil
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
import numpy as np
import math
from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass(order=True)
class PrioritizedNode:
    """Node for A* priority queue."""
    priority: float
    position: Tuple[int, int] = field(compare=False)
    parent: Optional[Tuple[int, int]] = field(default=None, compare=False)


class AdvancedExplorer(Node):
    """
    Advanced active explorer with A* path planning.

    Features:
    - A* path planning for optimal collision-free paths
    - Frontier-based exploration
    - Information gain estimation
    - Multi-criteria goal selection (entropy + distance + information gain)
    - Path visualization
    """

    def __init__(self):
        super().__init__('advanced_explorer')

        # Parameters
        self.declare_parameter('min_entropy', 30.0)  # Minimum entropy threshold (0-100)
        self.declare_parameter('max_goal_distance', 8.0)  # Max distance to goals (m)
        self.declare_parameter('frontier_cluster_size', 5)  # Min frontier size
        self.declare_parameter('path_resolution', 0.1)  # Path step size (m)
        self.declare_parameter('update_rate', 1.0)  # Goal update rate (Hz)
        self.declare_parameter('info_gain_weight', 0.3)  # Weight for information gain
        self.declare_parameter('entropy_weight', 0.4)  # Weight for entropy
        self.declare_parameter('distance_weight', 0.3)  # Weight for distance

        self.min_entropy = self.get_parameter('min_entropy').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.frontier_cluster_size = self.get_parameter('frontier_cluster_size').value
        self.path_resolution = self.get_parameter('path_resolution').value
        self.update_rate = self.get_parameter('update_rate').value
        self.w_info = self.get_parameter('info_gain_weight').value
        self.w_entropy = self.get_parameter('entropy_weight').value
        self.w_distance = self.get_parameter('distance_weight').value

        # State
        self.entropy_map = None
        self.occupancy_map = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_path = None
        self.current_goal = None

        # Subscribers
        self.entropy_sub = self.create_subscription(
            OccupancyGrid,
            '/entropy_map',
            self.entropy_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/exploration_goal', 10)
        self.path_pub = self.create_publisher(Path, '/exploration_path', 10)
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)
        self.info_gain_pub = self.create_publisher(Float64, '/expected_info_gain', 10)

        # Timer for goal updates
        self.create_timer(1.0 / self.update_rate, self.update_exploration_goal)

        self.get_logger().info('Advanced Explorer started')
        self.get_logger().info(f'  Entropy weight: {self.w_entropy}')
        self.get_logger().info(f'  Distance weight: {self.w_distance}')
        self.get_logger().info(f'  Info gain weight: {self.w_info}')

    def entropy_callback(self, msg):
        """Receive entropy map updates."""
        self.entropy_map = msg

    def map_callback(self, msg):
        """Receive occupancy map updates."""
        self.occupancy_map = msg

        # Extract robot position from map frame (simplified - normally use TF)
        # For synthetic robot, we can use odometry or TF
        # Here we'll just keep the last known position

    def update_exploration_goal(self):
        """Main exploration loop - find and publish new goal."""
        if self.entropy_map is None or self.occupancy_map is None:
            return

        # Get robot position (simplified - normally from TF)
        # For now, assume robot position is tracked separately
        # In real implementation, use tf2_ros to get transform

        # Find frontiers
        frontiers = self.detect_frontiers()

        if not frontiers:
            self.get_logger().info('No frontiers found - exploration may be complete')
            return

        # Visualize frontiers
        self.publish_frontier_markers(frontiers)

        # Find high-entropy regions
        high_entropy_cells = self.find_high_entropy_cells()

        # Select best goal based on multiple criteria
        best_goal = self.select_best_goal(frontiers, high_entropy_cells)

        if best_goal is None:
            return

        # Plan path to goal using A*
        path = self.plan_path(best_goal)

        if path is None:
            self.get_logger().warn(f'No path found to goal {best_goal}')
            return

        # Publish goal and path
        self.publish_goal(best_goal)
        self.publish_path(path)

        # Estimate and publish information gain
        info_gain = self.estimate_information_gain(best_goal)
        self.publish_info_gain(info_gain)

        self.current_goal = best_goal
        self.current_path = path

    def detect_frontiers(self) -> List[Tuple[int, int]]:
        """
        Detect frontier cells (boundary between known free space and unknown).

        A frontier cell is:
        - Free (value = 0)
        - Adjacent to at least one unknown cell (value = -1)
        """
        if self.occupancy_map is None:
            return []

        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        data = np.array(self.occupancy_map.data).reshape((height, width))

        frontiers = []

        # Check each cell
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Must be free space
                if data[y, x] != 0:
                    continue

                # Check if adjacent to unknown
                neighbors = [
                    data[y-1, x], data[y+1, x],
                    data[y, x-1], data[y, x+1],
                    data[y-1, x-1], data[y-1, x+1],
                    data[y+1, x-1], data[y+1, x+1],
                ]

                if -1 in neighbors:
                    frontiers.append((x, y))

        # Cluster frontiers and keep only significant ones
        frontiers = self.cluster_frontiers(frontiers)

        return frontiers

    def cluster_frontiers(self, frontiers: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Group nearby frontiers and return cluster centers."""
        if not frontiers:
            return []

        # Simple clustering: group cells within 3 pixels
        clusters = []
        used = set()

        for fx, fy in frontiers:
            if (fx, fy) in used:
                continue

            # Find all nearby frontiers
            cluster = [(fx, fy)]
            used.add((fx, fy))

            for ox, oy in frontiers:
                if (ox, oy) in used:
                    continue
                if abs(ox - fx) <= 3 and abs(oy - fy) <= 3:
                    cluster.append((ox, oy))
                    used.add((ox, oy))

            # Only keep significant clusters
            if len(cluster) >= self.frontier_cluster_size:
                # Use cluster center
                cx = int(np.mean([x for x, y in cluster]))
                cy = int(np.mean([y for x, y in cluster]))
                clusters.append((cx, cy))

        return clusters

    def find_high_entropy_cells(self) -> List[Tuple[int, int, float]]:
        """Find cells with high entropy values."""
        if self.entropy_map is None:
            return []

        width = self.entropy_map.info.width
        height = self.entropy_map.info.height
        data = np.array(self.entropy_map.data).reshape((height, width))

        high_entropy = []

        for y in range(height):
            for x in range(width):
                if data[y, x] >= self.min_entropy:
                    high_entropy.append((x, y, data[y, x]))

        return high_entropy

    def select_best_goal(self, frontiers: List[Tuple[int, int]],
                        high_entropy: List[Tuple[int, int, float]]) -> Optional[Tuple[int, int]]:
        """
        Select best exploration goal using multi-criteria scoring.

        Criteria:
        1. Distance (closer is better)
        2. Entropy (higher is better)
        3. Information gain (more unknown space visible is better)
        """
        if not frontiers:
            return None

        # Get robot position in map coordinates
        robot_mx, robot_my = self.world_to_map(self.robot_x, self.robot_y)

        best_score = -np.inf
        best_goal = None

        for fx, fy in frontiers:
            # Distance score (closer is better)
            dist = math.sqrt((fx - robot_mx)**2 + (fy - robot_my)**2)
            dist_world = dist * self.occupancy_map.info.resolution

            if dist_world > self.max_goal_distance:
                continue

            dist_score = 1.0 / (1.0 + dist_world)  # Normalize to [0,1]

            # Entropy score (higher entropy in region is better)
            entropy_score = self.get_local_entropy(fx, fy) / 100.0  # Normalize to [0,1]

            # Information gain score (estimated unknown cells visible from this position)
            info_gain_score = self.estimate_information_gain((fx, fy)) / 100.0  # Normalize

            # Combined score
            score = (self.w_distance * dist_score +
                    self.w_entropy * entropy_score +
                    self.w_info * info_gain_score)

            if score > best_score:
                best_score = score
                best_goal = (fx, fy)

        return best_goal

    def get_local_entropy(self, mx: int, my: int, radius: int = 5) -> float:
        """Get average entropy in region around cell."""
        if self.entropy_map is None:
            return 0.0

        width = self.entropy_map.info.width
        height = self.entropy_map.info.height
        data = np.array(self.entropy_map.data).reshape((height, width))

        values = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = mx + dx
                y = my + dy
                if 0 <= x < width and 0 <= y < height:
                    if data[y, x] >= 0:  # Valid entropy value
                        values.append(data[y, x])

        return np.mean(values) if values else 0.0

    def estimate_information_gain(self, goal: Tuple[int, int]) -> float:
        """
        Estimate how much unknown space would be observed from goal position.

        Uses simple raycasting to count unknown cells visible from goal.
        """
        if self.occupancy_map is None:
            return 0.0

        gx, gy = goal
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        data = np.array(self.occupancy_map.data).reshape((height, width))

        # Raycast in 36 directions (every 10 degrees)
        unknown_count = 0
        sensor_range_cells = int(10.0 / self.occupancy_map.info.resolution)  # 10m range

        for angle_deg in range(0, 360, 10):
            angle = math.radians(angle_deg)
            dx = math.cos(angle)
            dy = math.sin(angle)

            # Cast ray
            for step in range(1, sensor_range_cells):
                x = int(gx + dx * step)
                y = int(gy + dy * step)

                if not (0 <= x < width and 0 <= y < height):
                    break

                cell_value = data[y, x]

                # Unknown cell
                if cell_value == -1:
                    unknown_count += 1

                # Hit obstacle - stop ray
                if cell_value > 50:
                    break

        return float(unknown_count)

    def plan_path(self, goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Plan collision-free path to goal using A* algorithm.

        Returns list of (x, y) map coordinates, or None if no path found.
        """
        if self.occupancy_map is None:
            return None

        start = self.world_to_map(self.robot_x, self.robot_y)

        return self.astar(start, goal)

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding algorithm."""
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        data = np.array(self.occupancy_map.data).reshape((height, width))

        # Priority queue: (f_score, position, parent)
        open_set = PriorityQueue()
        open_set.put(PrioritizedNode(0.0, start, None))

        # Track visited nodes and their costs
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            current_node = open_set.get()
            current = current_node.position

            # Reached goal
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # Check all 8 neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                # Check bounds
                if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                    continue

                # Check if occupied
                if data[neighbor[1], neighbor[0]] > 50:  # Occupied threshold
                    continue

                # Cost to move to neighbor
                move_cost = math.sqrt(dx**2 + dy**2)
                tentative_g = g_score[current] + move_cost

                # Better path found
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    open_set.put(PrioritizedNode(f, neighbor, current))

        # No path found
        return None

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic for A*."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from A* came_from map."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to map coordinates."""
        if self.occupancy_map is None:
            return (0, 0)

        resolution = self.occupancy_map.info.resolution
        origin_x = self.occupancy_map.info.origin.position.x
        origin_y = self.occupancy_map.info.origin.position.y

        mx = int((wx - origin_x) / resolution)
        my = int((wy - origin_y) / resolution)

        return (mx, my)

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map coordinates to world coordinates."""
        if self.occupancy_map is None:
            return (0.0, 0.0)

        resolution = self.occupancy_map.info.resolution
        origin_x = self.occupancy_map.info.origin.position.x
        origin_y = self.occupancy_map.info.origin.position.y

        wx = mx * resolution + origin_x
        wy = my * resolution + origin_y

        return (wx, wy)

    def publish_goal(self, goal: Tuple[int, int]):
        """Publish exploration goal."""
        wx, wy = self.map_to_world(goal[0], goal[1])

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = wx
        msg.pose.position.y = wy
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def publish_path(self, path: List[Tuple[int, int]]):
        """Publish planned path."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for mx, my in path:
            wx, wy = self.map_to_world(mx, my)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def publish_frontier_markers(self, frontiers: List[Tuple[int, int]]):
        """Publish frontier visualization markers."""
        marker_array = MarkerArray()

        for i, (fx, fy) in enumerate(frontiers):
            wx, wy = self.map_to_world(fx, fy)

            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.frontier_pub.publish(marker_array)

    def publish_info_gain(self, info_gain: float):
        """Publish expected information gain."""
        msg = Float64()
        msg.data = info_gain
        self.info_gain_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = AdvancedExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
