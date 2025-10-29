#!/bin/bash
#
# COMPLETE UNCERTAINTY-AWARE SLAM SYSTEM
# Runs autonomous exploration with automatic visualization generation
#

set -e  # Exit on error

echo "============================================================"
echo "  UNCERTAINTY-AWARE SLAM - COMPLETE SYSTEM LAUNCHER"
echo "============================================================"
echo ""

# Navigate to workspace
cd ~/slam_uncertainty_ws

echo "🛑 Stopping any existing ROS processes..."
pkill -9 -f "ros2|rviz2|slam|synthetic" 2>/dev/null || true
sleep 2

echo "🔧 Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "🔨 Building uncertainty_slam package..."
colcon build --packages-select uncertainty_slam --symlink-install

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ BUILD FAILED! Check errors above."
    exit 1
fi

echo ""
echo "✅ Build successful!"
echo ""

echo "📦 Sourcing workspace..."
source install/setup.bash

# Create results directory
mkdir -p ~/slam_uncertainty_ws/results/visualizations
echo "📁 Results directory: ~/slam_uncertainty_ws/results/visualizations"
echo ""

echo "============================================================"
echo "  SYSTEM CONFIGURATION"
echo "============================================================"
echo "  • Synthetic Robot: ENABLED (exploration_pattern mode)"
echo "  • SLAM Toolbox: ENABLED (async mapping)"
echo "  • Uncertainty Node: ENABLED (entropy computation)"
echo "  • RViz Visualization: ENABLED"
echo "  • Results Generator: ENABLED (auto-triggers after 100%)"
echo ""
echo "  📊 Exploration Pattern:"
echo "     - 139 waypoints for complete coverage"
echo "     - Automatic obstacle avoidance"
echo "     - Stuck detection & recovery"
echo "     - 0.35m waypoint tolerance"
echo ""
echo "  ⏱️  Expected Duration: 10-15 minutes"
echo "============================================================"
echo ""
echo "🚀 Launching system in 3 seconds..."
sleep 3

# Launch the complete system
ros2 launch uncertainty_slam complete_system.launch.py \
    use_rviz:=true \
    use_results_generator:=true \
    use_active_explorer:=false \
    use_ecs_logger:=false

echo ""
echo "============================================================"
echo "  System shutdown complete"
echo "============================================================"
