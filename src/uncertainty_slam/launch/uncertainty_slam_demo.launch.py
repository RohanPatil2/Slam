#!/usr/bin/env python3
"""
Launch file for complete uncertainty-aware SLAM demo with Stage simulator.

Launches:
1. Stage simulator with TurtleBot3
2. SLAM Toolbox node (replaces GMapping)
3. Uncertainty tracking node
4. ECS logger
5. Active explorer (optional)
6. RViz for visualization

Author: Rohan Upendra Patil
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for uncertainty SLAM demo."""

    # Declare launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    enable_explorer = DeclareLaunchArgument(
        'enable_explorer',
        default_value='false',
        description='Enable active exploration node'
    )

    enable_ecs_logger = DeclareLaunchArgument(
        'enable_ecs_logger',
        default_value='true',
        description='Enable ECS logging'
    )

    experiment_name = DeclareLaunchArgument(
        'experiment_name',
        default_value='demo_exp',
        description='Name for the ECS logging experiment'
    )

    record_duration = DeclareLaunchArgument(
        'record_duration',
        default_value='120.0',
        description='Duration to record ECS data (seconds)'
    )

    # Get package directories
    uncertainty_slam_dir = get_package_share_directory('uncertainty_slam')
    # slam_gmapping_dir is no longer needed

    # Stage simulator node
    stage_node = Node(
        package='stage_ros2',         # <-- CHANGED
        executable='stage_ros2',      # <-- CHANGED
        name='stageros',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('uncertainty_slam'),
                'worlds',
                'test_environment.world'
            ])
        ]
    )

    # SLAM Toolbox node (replaces GMapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',           # <-- CHANGED
        executable='async_slam_toolbox_node', # <-- CHANGED
        name='slam_toolbox',            # <-- CHANGED
        output='screen',
        parameters=[{
            'use_sim_time': True,
            
            # Replaced GMapping params with slam_toolbox equivalents
            'map_publish_period': 0.5,  # GMapping 'map_update_interval'
            'max_laser_range': 8.0,       # GMapping 'maxUrange'
            'resolution': 0.05,         # GMapping 'delta'
            'occupied_thresh': 0.65,      # slam_toolbox default (GMapping 'occ_thresh' was 0.25)
            'free_thresh': 0.25,        # slam_toolbox default
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'particle_filter_update_rate': 1.0, # GMapping 'temporalUpdate'
            'linear_update_distance': 1.0,    # GMapping 'linearUpdate'
            'angular_update_distance': 0.5,   # GMapping 'angularUpdate'
            'minimum_travel_distance': 0.1,   # GMapping 'srr' related
            'minimum_travel_heading': 0.2,    # GMapping 'srt' related
            'transform_publish_period': 0.1,  # GMapping 'str'/'stt' related
            'particle_count': 30,         # GMapping 'particles'
            'scan_buffer_size': 10,
            'map_start_x': -10.0,         # GMapping 'xmin'
            'map_start_y': -10.0,         # GMapping 'ymin'
            'map_size_x': 20.0,           # GMapping 'xmax'/'xmin'
            'map_size_y': 20.0            # GMapping 'ymax'/'ymin'
        }],
        remappings=[
            ('scan', '/base_scan') # This remapping is still valid
        ]
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
            'record_duration': LaunchConfiguration('record_duration'),
            'auto_save_interval': 10.0,
        }]
    )

    # Active Explorer node
    active_explorer_node = Node(
        package='uncertainty_slam',
        executable='active_explorer',
        name='active_explorer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_explorer')),
        parameters=[{
            'use_sim_time': True,
            'entropy_map_topic': '/entropy_map',
            'occupancy_map_topic': '/map',
            'cmd_vel_topic': '/cmd_vel',
            'min_entropy_threshold': 50.0,
            'max_exploration_distance': 5.0,
            'control_frequency': 10.0,
            'linear_velocity': 0.3,
            'angular_velocity': 0.5,
            'goal_tolerance': 0.3,
            'exploration_enabled': True,
            'base_frame': 'base_link',
            'map_frame': 'map',
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
        enable_explorer,
        enable_ecs_logger,
        experiment_name,
        record_duration,
        stage_node,
        slam_toolbox_node,  # <-- Use the corrected node
        uncertainty_node,
        ecs_logger_node,
        active_explorer_node,
        rviz_node,
    ])