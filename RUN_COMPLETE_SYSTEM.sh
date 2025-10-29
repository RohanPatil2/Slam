#!/bin/bash

#
# Complete System Launcher
# Runs the entire Uncertainty-Aware SLAM system with synthetic robot
# NO EXTERNAL SIMULATOR REQUIRED!
#

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}=============================================="
echo "Uncertainty-Aware SLAM - Complete System"
echo "=============================================="
echo -e "${NC}"
echo ""

# Check if we're in the workspace
if [ ! -d "src/uncertainty_slam" ]; then
    echo -e "${YELLOW}Warning: Not in slam_uncertainty_ws directory${NC}"
    echo "Please run from: ~/slam_uncertainty_ws"
    exit 1
fi

# Build the package
echo -e "${YELLOW}Building package...${NC}"
colcon build --packages-select uncertainty_slam --symlink-install
echo ""

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash
echo ""

# Show menu
echo "Choose what to run:"
echo ""
echo "1) Complete system (Robot + SLAM + Uncertainty + RViz)"
echo "2) Complete system + Active Explorer"
echo "3) Complete system + ECS Logger"
echo "4) Complete system + Active Explorer + ECS Logger (FULL)"
echo "5) Just synthetic robot (for testing)"
echo ""
read -p "Choice [1-5]: " choice

case $choice in
    1)
        echo ""
        echo -e "${GREEN}Launching: Robot + SLAM + Uncertainty + RViz${NC}"
        echo ""
        ros2 launch uncertainty_slam complete_system.launch.py \
            use_rviz:=true \
            use_active_explorer:=false \
            use_ecs_logger:=false
        ;;

    2)
        echo ""
        echo -e "${GREEN}Launching: Robot + SLAM + Uncertainty + RViz + Active Explorer${NC}"
        echo ""
        ros2 launch uncertainty_slam complete_system.launch.py \
            use_rviz:=true \
            use_active_explorer:=true \
            use_ecs_logger:=false
        ;;

    3)
        echo ""
        echo -e "${GREEN}Launching: Robot + SLAM + Uncertainty + RViz + ECS Logger${NC}"
        echo ""
        read -p "Experiment name: " exp_name
        ros2 launch uncertainty_slam complete_system.launch.py \
            use_rviz:=true \
            use_active_explorer:=false \
            use_ecs_logger:=true \
            experiment_name:=$exp_name
        ;;

    4)
        echo ""
        echo -e "${GREEN}Launching FULL SYSTEM${NC}"
        echo ""
        read -p "Experiment name: " exp_name
        ros2 launch uncertainty_slam complete_system.launch.py \
            use_rviz:=true \
            use_active_explorer:=true \
            use_ecs_logger:=true \
            experiment_name:=$exp_name
        ;;

    5)
        echo ""
        echo -e "${GREEN}Launching: Synthetic Robot Only${NC}"
        echo ""
        echo "Controls:"
        echo "  w/x: forward/backward"
        echo "  a/d: rotate left/right"
        echo "  s: stop"
        echo "  m: manual mode"
        echo "  e: exploration pattern mode"
        echo "  q: quit"
        echo ""
        ros2 run uncertainty_slam synthetic_robot
        ;;

    *)
        echo -e "${YELLOW}Invalid choice${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}System shutdown complete${NC}"
