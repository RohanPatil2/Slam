#!/bin/bash

# Simple test script to verify all files are in place
# Run this to check if everything is ready

echo "======================================"
echo "Uncertainty SLAM - File Verification"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check functions
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Missing: $1"
        return 1
    fi
}

check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} Command available: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Command not found: $1"
        return 1
    fi
}

# Test counter
PASSED=0
FAILED=0

echo "1. Checking world files..."
if check_file "src/uncertainty_slam/worlds/cave.world"; then ((PASSED++)); else ((FAILED++)); fi
if check_file "src/uncertainty_slam/worlds/test_environment.world"; then ((PASSED++)); else ((FAILED++)); fi
echo ""

echo "2. Checking config files..."
if check_file "src/uncertainty_slam/config/uncertainty_slam.rviz"; then ((PASSED++)); else ((FAILED++)); fi
echo ""

echo "3. Checking installed files..."
if check_file "install/uncertainty_slam/share/uncertainty_slam/worlds/cave.world"; then ((PASSED++)); else ((FAILED++)); fi
if check_file "install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz"; then ((PASSED++)); else ((FAILED++)); fi
echo ""

echo "4. Checking Python nodes..."
if check_file "install/uncertainty_slam/lib/uncertainty_slam/uncertainty_node"; then ((PASSED++)); else ((FAILED++)); fi
if check_file "install/uncertainty_slam/lib/uncertainty_slam/active_explorer"; then ((PASSED++)); else ((FAILED++)); fi
if check_file "install/uncertainty_slam/lib/uncertainty_slam/ecs_logger"; then ((PASSED++)); else ((FAILED++)); fi
echo ""

echo "5. Checking ROS2 packages..."
source /opt/ros/humble/setup.bash 2>/dev/null
if ros2 pkg list | grep -q "slam_toolbox"; then
    echo -e "${GREEN}✓${NC} SLAM Toolbox installed"
    ((PASSED++))
else
    echo -e "${RED}✗${NC} SLAM Toolbox not found - Install with: sudo apt-get install ros-humble-slam-toolbox"
    ((FAILED++))
fi

if ros2 pkg list | grep -q "uncertainty_slam"; then
    echo -e "${GREEN}✓${NC} uncertainty_slam package found"
    ((PASSED++))
else
    echo -e "${RED}✗${NC} uncertainty_slam package not found"
    ((FAILED++))
fi
echo ""

echo "6. Checking commands..."
if check_command "ros2"; then ((PASSED++)); else ((FAILED++)); fi
if check_command "rviz2"; then ((PASSED++)); else ((FAILED++)); fi
echo ""

# Summary
echo "======================================"
echo "Summary:"
echo -e "${GREEN}Passed: $PASSED${NC}"
echo -e "${RED}Failed: $FAILED${NC}"
echo "======================================"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed! Ready to test.${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Read TESTING_GUIDE.md for detailed instructions"
    echo "2. Or run quick test:"
    echo "   Terminal 1: ros2 run stage_ros2 stage_ros2 install/uncertainty_slam/share/uncertainty_slam/worlds/cave.world"
    echo "   Terminal 2: ros2 launch slam_toolbox online_async_launch.py"
    echo "   Terminal 3: ros2 run uncertainty_slam uncertainty_node"
    echo "   Terminal 4: ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz"
    echo "   Terminal 5: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    exit 0
else
    echo -e "${RED}✗ Some checks failed. Please fix the issues above.${NC}"
    echo ""
    echo "Common fixes:"
    echo "- Missing files: cd ~/slam_uncertainty_ws && colcon build --symlink-install"
    echo "- Missing slam_toolbox: sudo apt-get install ros-humble-slam-toolbox"
    echo "- Missing stage: sudo apt-get install ros-humble-stage-ros2"
    exit 1
fi
