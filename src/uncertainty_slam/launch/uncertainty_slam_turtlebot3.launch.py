#!/usr/bin/env python3
"""
Launch file for uncertainty-aware SLAM with TurtleBot3 simulation.

This launch file works WITHOUT Stage simulator - uses TurtleBot3 Gazebo instead.

Launches:
1. TurtleBot3 Gazebo simulation (if available)
2. SLAM Toolbox node
3. Uncertainty tracking node
4. ECS logger (optional)
5. RViz for visualization

Author: Rohan Upendra Patil
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for uncertainty SLAM with TurtleBot3."""

    # Declare launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    enable_ecs_logger = DeclareLaunchArgument(
        'enable_ecs_logger',
        default_value='false',
        description='Enable ECS logging'
    )

    experiment_name = DeclareLaunchArgument(
        'experiment_name',
        default_value='turtlebot3_exp',
        description='Name for the ECS logging experiment'
    )

    # Get package directory
    uncertainty_slam_dir = get_package_share_directory('uncertainty_slam')

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'map_update_interval': 1.0,
        }]
    )

    # Uncertainty tracking node
    uncertainty_node = Node(
        package='uncertainty_slam',
        executable='uncertainty_node',
        name='uncertainty_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'entropy_publish_rate': 10.0,
            'map_topic': '/map',
            'entropy_grid_topic': '/entropy_map',
            'min_observations': 10,
        }]
    )

    # ECS Logger node
    ecs_logger_node = Node(
        package='uncertainty_slam',
        executable='ecs_logger',
        name='ecs_logger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ecs_logger')),
        parameters=[{
            'use_sim_time': True,
            'output_dir': os.path.join(uncertainty_slam_dir, 'results', 'ecs_logs'),
            'experiment_name': LaunchConfiguration('experiment_name'),
            'time_weight_factor': 2.0,
            'record_duration': 120.0,
            'auto_save_interval': 10.0,
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
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        use_rviz,
        enable_ecs_logger,
        experiment_name,
        slam_toolbox_node,
        uncertainty_node,
        ecs_logger_node,
        rviz_node,
    ])
