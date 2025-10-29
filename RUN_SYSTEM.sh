#!/bin/bash

# Comprehensive launch script for Uncertainty-Aware SLAM
# This script helps you launch all components correctly

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================"
echo "Uncertainty-Aware SLAM Launch Helper"
echo -e "======================================${NC}"
echo ""

# Get absolute paths
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CAVE_WORLD="${WORKSPACE_DIR}/src/uncertainty_slam/worlds/cave.world"
TEST_WORLD="${WORKSPACE_DIR}/src/uncertainty_slam/worlds/test_environment.world"
RVIZ_CONFIG="${WORKSPACE_DIR}/install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz"

# Check if files exist
if [ ! -f "$CAVE_WORLD" ]; then
    echo -e "${RED}Error: cave.world not found at $CAVE_WORLD${NC}"
    echo "Please rebuild: colcon build --symlink-install"
    exit 1
fi

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo -e "${RED}Error: RViz config not found${NC}"
    echo "Please rebuild: colcon build --symlink-install"
    exit 1
fi

echo -e "${GREEN}✓ All files found${NC}"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo "Cave world: $CAVE_WORLD"
echo ""

# Function to print commands
print_terminal_commands() {
    echo -e "${YELLOW}======================================"
    echo "TERMINAL COMMANDS TO RUN"
    echo -e "======================================${NC}"
    echo ""

    echo -e "${BLUE}Terminal 1 - Stage Simulator:${NC}"
    echo "cd $WORKSPACE_DIR"
    echo "source install/setup.bash"
    echo "ros2 run stage_ros2 stage_ros2 \"${CAVE_WORLD}\""
    echo ""

    echo -e "${BLUE}Terminal 2 - SLAM Toolbox:${NC}"
    echo "source /opt/ros/humble/setup.bash"
    echo "ros2 launch slam_toolbox online_async_launch.py"
    echo ""

    echo -e "${BLUE}Terminal 3 - Uncertainty Node:${NC}"
    echo "cd $WORKSPACE_DIR"
    echo "source install/setup.bash"
    echo "ros2 run uncertainty_slam uncertainty_node"
    echo ""

    echo -e "${BLUE}Terminal 4 - RViz:${NC}"
    echo "cd $WORKSPACE_DIR"
    echo "source install/setup.bash"
    echo "ros2 run rviz2 rviz2 -d \"${RVIZ_CONFIG}\""
    echo ""

    echo -e "${BLUE}Terminal 5 - Robot Control:${NC}"
    echo "source /opt/ros/humble/setup.bash"
    echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo ""

    echo -e "${YELLOW}Controls: W=forward, S=backward, A=left, D=right, X=stop${NC}"
    echo ""
}

# Main menu
echo "What would you like to do?"
echo ""
echo "1) Show terminal commands (recommended)"
echo "2) Launch Stage simulator only"
echo "3) Launch everything (requires tmux)"
echo "4) Test with test_environment.world instead"
echo "5) Exit"
echo ""
read -p "Choice [1-5]: " choice

case $choice in
    1)
        print_terminal_commands
        ;;
    2)
        echo ""
        echo -e "${GREEN}Launching Stage simulator...${NC}"
        cd "$WORKSPACE_DIR"
        source install/setup.bash
        ros2 run stage_ros2 stage_ros2 "$CAVE_WORLD"
        ;;
    3)
        if ! command -v tmux &> /dev/null; then
            echo -e "${RED}Error: tmux not installed${NC}"
            echo "Install: sudo apt-get install tmux"
            echo ""
            echo "Or use option 1 to see individual terminal commands"
            exit 1
        fi

        echo ""
        echo -e "${GREEN}Launching all components in tmux...${NC}"
        echo "Press Ctrl+B then D to detach from tmux"
        echo "Run 'tmux attach' to reattach"
        sleep 2

        # Create tmux session
        tmux new-session -d -s slam

        # Window 0: Stage
        tmux send-keys -t slam "cd $WORKSPACE_DIR && source install/setup.bash && ros2 run stage_ros2 stage_ros2 '$CAVE_WORLD'" C-m

        # Window 1: SLAM
        tmux new-window -t slam
        tmux send-keys -t slam "source /opt/ros/humble/setup.bash && ros2 launch slam_toolbox online_async_launch.py" C-m

        # Window 2: Uncertainty
        tmux new-window -t slam
        tmux send-keys -t slam "cd $WORKSPACE_DIR && source install/setup.bash && ros2 run uncertainty_slam uncertainty_node" C-m

        # Window 3: RViz
        tmux new-window -t slam
        tmux send-keys -t slam "cd $WORKSPACE_DIR && source install/setup.bash && ros2 run rviz2 rviz2 -d '$RVIZ_CONFIG'" C-m

        # Window 4: Teleop
        tmux new-window -t slam
        tmux send-keys -t slam "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

        # Attach to session
        tmux attach -t slam
        ;;
    4)
        echo ""
        echo -e "${GREEN}Using test_environment.world...${NC}"
        cd "$WORKSPACE_DIR"
        source install/setup.bash
        ros2 run stage_ros2 stage_ros2 "$TEST_WORLD"
        ;;
    5)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac
