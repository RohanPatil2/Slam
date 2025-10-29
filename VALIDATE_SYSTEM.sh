#!/bin/bash
#
# System Validation Script
# Checks if all components are properly configured
#

echo "============================================================"
echo "  UNCERTAINTY-AWARE SLAM - SYSTEM VALIDATION"
echo "============================================================"
echo ""

PASS=0
FAIL=0

# Check 1: Workspace directory
echo -n "✓ Checking workspace directory... "
if [ -d ~/slam_uncertainty_ws ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 2: Source package
echo -n "✓ Checking uncertainty_slam package... "
if [ -d ~/slam_uncertainty_ws/src/uncertainty_slam ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 3: Python files
echo -n "✓ Checking synthetic_robot.py... "
if [ -f ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/synthetic_robot.py ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

echo -n "✓ Checking results_generator.py... "
if [ -f ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/results_generator.py ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

echo -n "✓ Checking uncertainty_node.py... "
if [ -f ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/uncertainty_node.py ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 4: Launch files
echo -n "✓ Checking complete_system.launch.py... "
if [ -f ~/slam_uncertainty_ws/src/uncertainty_slam/launch/complete_system.launch.py ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 5: Setup.py
echo -n "✓ Checking setup.py has results_generator... "
if grep -q "results_generator" ~/slam_uncertainty_ws/src/uncertainty_slam/setup.py; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 6: Build directory
echo -n "✓ Checking if package is built... "
if [ -d ~/slam_uncertainty_ws/build/uncertainty_slam ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "⚠️  WARN - Run: colcon build --packages-select uncertainty_slam"
fi

# Check 7: Install directory
echo -n "✓ Checking if package is installed... "
if [ -d ~/slam_uncertainty_ws/install/uncertainty_slam ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "⚠️  WARN - Run: colcon build --packages-select uncertainty_slam"
fi

# Check 8: Results directory
echo -n "✓ Checking results directory... "
if [ -d ~/slam_uncertainty_ws/results/visualizations ]; then
    echo "✅ PASS (already exists)"
    ((PASS++))
else
    echo "⚠️  Creating... "
    mkdir -p ~/slam_uncertainty_ws/results/visualizations
    echo "✅ CREATED"
    ((PASS++))
fi

# Check 9: Launch script
echo -n "✓ Checking RUN_COMPLETE_SYSTEM.sh... "
if [ -f ~/slam_uncertainty_ws/RUN_COMPLETE_SYSTEM.sh ]; then
    if [ -x ~/slam_uncertainty_ws/RUN_COMPLETE_SYSTEM.sh ]; then
        echo "✅ PASS (executable)"
        ((PASS++))
    else
        echo "⚠️  Making executable..."
        chmod +x ~/slam_uncertainty_ws/RUN_COMPLETE_SYSTEM.sh
        echo "✅ FIXED"
        ((PASS++))
    fi
else
    echo "❌ FAIL"
    ((FAIL++))
fi

# Check 10: ROS 2 installation
echo -n "✓ Checking ROS 2 Humble... "
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL - ROS 2 Humble not found"
    ((FAIL++))
fi

# Check 11: Code verification - waypoint tolerance
echo -n "✓ Checking waypoint tolerance (should be 0.35)... "
if grep -q "if dist < 0.35" ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/synthetic_robot.py; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL - Tolerance not updated"
    ((FAIL++))
fi

# Check 12: Code verification - stuck detection
echo -n "✓ Checking stuck detection code... "
if grep -q "waypoint_timeout" ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/synthetic_robot.py; then
    echo "✅ PASS"
    ((PASS++))
else
    echo "❌ FAIL - Stuck detection not added"
    ((FAIL++))
fi

# Check 13: Waypoint count
echo -n "✓ Checking waypoint count (should be ~139)... "
WP_COUNT=$(grep -c "(-\?[0-9.]\+, -\?[0-9.]\+)" ~/slam_uncertainty_ws/src/uncertainty_slam/uncertainty_slam/synthetic_robot.py | head -1)
if [ "$WP_COUNT" -gt "100" ]; then
    echo "✅ PASS ($WP_COUNT waypoints found)"
    ((PASS++))
else
    echo "⚠️  WARN ($WP_COUNT waypoints - expected ~139)"
fi

echo ""
echo "============================================================"
echo "  VALIDATION RESULTS"
echo "============================================================"
echo "Passed: $PASS"
echo "Failed: $FAIL"
echo ""

if [ $FAIL -eq 0 ]; then
    echo "✅ ALL CHECKS PASSED!"
    echo ""
    echo "System is ready to run. Execute:"
    echo "  cd ~/slam_uncertainty_ws"
    echo "  ./RUN_COMPLETE_SYSTEM.sh"
    echo ""
else
    echo "❌ SOME CHECKS FAILED"
    echo ""
    echo "Please fix the failed items above before running."
    echo ""
fi

echo "============================================================"
