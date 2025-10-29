#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('uncertainty_slam')
    world_file = os.path.join(pkg_dir, 'worlds', 'cave.world')
    
    print(f"[INFO] World file: {world_file}")
    print(f"[INFO] Exists: {os.path.exists(world_file)}")
    
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage_ros2',
        arguments=[world_file],
        output='screen',
    )
    
    return LaunchDescription([stage_node])
