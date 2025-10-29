#!/usr/bin/env python3
"""
Fixed Stage launcher that handles the path correctly.

This script works around the stage_ros2 path bug by:
1. Copying the world file to current directory
2. Running stage_ros2 with just the filename
3. Cleaning up after

Usage:
    python3 run_stage_fixed.py /path/to/world.world
"""

import sys
import os
import shutil
import subprocess
import signal
import tempfile


def run_stage_with_world(world_file_path):
    """Run stage_ros2 with proper world file handling."""

    # Resolve to absolute path
    world_file_path = os.path.abspath(world_file_path)

    if not os.path.exists(world_file_path):
        print(f"Error: World file not found: {world_file_path}")
        return 1

    print(f"World file: {world_file_path}")
    print(f"File exists: {os.path.exists(world_file_path)}")
    print(f"File size: {os.path.getsize(world_file_path)} bytes")
    print("")

    # Get just the filename
    world_filename = os.path.basename(world_file_path)

    # Create temporary directory
    temp_dir = tempfile.mkdtemp(prefix='stage_world_')
    temp_world = os.path.join(temp_dir, world_filename)

    try:
        # Copy world file to temp directory
        shutil.copy2(world_file_path, temp_world)
        print(f"Copied world file to: {temp_world}")

        # Change to temp directory
        original_dir = os.getcwd()
        os.chdir(temp_dir)
        print(f"Changed directory to: {temp_dir}")
        print("")

        # Run stage_ros2 with just the filename
        print(f"Running: ros2 run stage_ros2 stage_ros2 {world_filename}")
        print("=" * 60)

        process = subprocess.Popen(
            ['ros2', 'run', 'stage_ros2', 'stage_ros2', world_filename],
            stdout=sys.stdout,
            stderr=sys.stderr
        )

        # Wait for process
        try:
            process.wait()
        except KeyboardInterrupt:
            print("\nShutting down...")
            process.send_signal(signal.SIGINT)
            process.wait()

        return process.returncode

    finally:
        # Cleanup
        os.chdir(original_dir)
        try:
            shutil.rmtree(temp_dir)
            print(f"\nCleaned up temporary directory: {temp_dir}")
        except:
            pass


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 run_stage_fixed.py <world_file.world>")
        print("")
        print("Example:")
        print("  python3 run_stage_fixed.py ~/slam_uncertainty_ws/src/uncertainty_slam/worlds/cave.world")
        sys.exit(1)

    world_file = sys.argv[1]
    sys.exit(run_stage_with_world(world_file))
