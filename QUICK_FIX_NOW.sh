#!/bin/bash

echo "=================================="
echo "FIXING CRITICAL BUG AND REBUILDING"
echo "=================================="
echo ""

cd ~/slam_uncertainty_ws

echo "1. Stopping old processes..."
pkill -9 -f "ros2|rviz2|slam" 2>/dev/null || true
sleep 2

echo "2. Sourcing ROS 2..."
source /opt/ros/humble/setup.bash

echo "3. Rebuilding package..."
colcon build --packages-select uncertainty_slam --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ BUILD SUCCESSFUL!"
    echo ""
    echo "4. Sourcing workspace..."
    source install/setup.bash
    
    echo ""
    echo "5. Launching system..."
    echo ""
    ros2 launch uncertainty_slam complete_system.launch.py use_rviz:=true use_results_generator:=true
else
    echo ""
    echo "❌ BUILD FAILED - Check errors above"
    exit 1
fi

