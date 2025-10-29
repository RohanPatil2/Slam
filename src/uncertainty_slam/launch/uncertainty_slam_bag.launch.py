#!/usr/bin/env python3
"""
Launch file for uncertainty-aware SLAM using ROS2 bag playback.

This works WITHOUT any simulator - just plays back recorded data!

Launches:
1. ROS2 bag playback (you provide the bag file)
2. SLAM Toolbox node
3. Uncertainty tracking node
4. RViz for visualization

Author: Rohan Upendra Patil
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for uncertainty SLAM with bag playback."""

    # Declare launch arguments
    bag_file = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to ROS2 bag file to play'
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag file'
    )

    # Get package directory
    uncertainty_slam_dir = get_package_share_directory('uncertainty_slam')

    # Bag playback
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'), '--clock'],
        output='screen'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'resolution': 0.05,
            'max_laser_range': 12.0,
        }]
    )

    # Uncertainty tracking node
    uncertainty_node = Node(
        package='uncertainty_slam',
        executable='uncertainty_node',
        name='uncertainty_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'entropy_publish_rate': 10.0,
            'map_topic': '/map',
            'entropy_grid_topic': '/entropy_map',
            'min_observations': 10,
        }]
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uncertainty_slam'),
        'config',
        'uncertainty_slam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        bag_file,
        use_rviz,
        use_sim_time,
        bag_play,
        slam_toolbox_node,
        uncertainty_node,
        rviz_node,
    ])
