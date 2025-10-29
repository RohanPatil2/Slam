from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('uncertainty_slam')
    world_file = os.path.join(pkg_dir, 'worlds', 'test_environment.world')
    
    return LaunchDescription([
        # Stage simulator
        Node(
            package='stage_ros2',         # <-- CHANGED
            executable='stage_ros2',      # <-- CHANGED
            name='stageros',
            arguments=[world_file],
            output='screen',
            parameters=[{
                'use_sim_time': True      # <-- ADDED
            }]
        ),
        
        # SLAM (now slam_toolbox)
        Node(
            package='slam_toolbox',       # <-- CHANGED
            executable='async_slam_toolbox_node', # <-- CHANGED
            name='slam_toolbox',          # <-- CHANGED
            output='screen',
            parameters=[{
                'use_sim_time': True,     # <-- ADDED
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'map_publish_period': 0.5 # Replaced 'map_update_interval'
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{
                'use_sim_time': True      # <-- ADDED
            }]
        )
    ])