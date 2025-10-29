#!/bin/bash

# Complete installation and run script for Uncertainty-Aware SLAM
# This installs everything needed and provides multiple ways to test

set -e  # Exit on error

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================"
echo "Uncertainty-Aware SLAM"
echo "Installation and Testing"
echo -e "======================================${NC}"
echo ""

# Check if we're in the right directory
if [ ! -d "src/uncertainty_slam" ]; then
    echo -e "${RED}Error: Please run this from ~/slam_uncertainty_ws${NC}"
    exit 1
fi

echo "Choose installation/test method:"
echo ""
echo "1) Install TurtleBot3 and test (RECOMMENDED)"
echo "2) Test with Python Stage wrapper (if Stage installed)"
echo "3) Create fake scan data for testing (no simulator needed)"
echo "4) Show status and available options"
echo "5) Exit"
echo ""
read -p "Choice [1-5]: " choice

case $choice in
    1)
        echo ""
        echo -e "${YELLOW}Installing TurtleBot3 packages...${NC}"
        echo "This requires sudo password"
        echo ""

        sudo apt-get update
        sudo apt-get install -y \
            ros-humble-turtlebot3 \
            ros-humble-turtlebot3-simulations \
            ros-humble-turtlebot3-gazebo \
            ros-humble-turtlebot3-teleop

        echo ""
        echo -e "${GREEN}✓ TurtleBot3 installed!${NC}"
        echo ""
        echo "To test, open 5 terminals and run:"
        echo ""
        echo -e "${BLUE}Terminal 1:${NC}"
        echo "  export TURTLEBOT3_MODEL=burger"
        echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
        echo ""
        echo -e "${BLUE}Terminal 2:${NC}"
        echo "  ros2 launch slam_toolbox online_async_launch.py"
        echo ""
        echo -e "${BLUE}Terminal 3:${NC}"
        echo "  cd ~/slam_uncertainty_ws && source install/setup.bash"
        echo "  ros2 run uncertainty_slam uncertainty_node"
        echo ""
        echo -e "${BLUE}Terminal 4:${NC}"
        echo "  cd ~/slam_uncertainty_ws && source install/setup.bash"
        echo "  ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz"
        echo ""
        echo -e "${BLUE}Terminal 5:${NC}"
        echo "  export TURTLEBOT3_MODEL=burger"
        echo "  ros2 run turtlebot3_teleop teleop_keyboard"
        echo ""
        ;;

    2)
        echo ""
        echo -e "${GREEN}Using Python Stage wrapper...${NC}"
        echo ""

        if ! command -v ros2 &> /dev/null; then
            echo -e "${RED}Error: ROS2 not found. Source your setup file.${NC}"
            exit 1
        fi

        source install/setup.bash

        if [ -f "src/uncertainty_slam/worlds/cave.world" ]; then
            echo "World file found. Launching..."
            python3 src/uncertainty_slam/scripts/run_stage_fixed.py \
                src/uncertainty_slam/worlds/cave.world
        else
            echo -e "${RED}Error: cave.world not found${NC}"
            echo "Rebuild: colcon build --packages-select uncertainty_slam"
        fi
        ;;

    3)
        echo ""
        echo -e "${YELLOW}Creating fake scan publisher for testing...${NC}"
        echo ""

        # Create a simple fake scan publisher
        cat > /tmp/fake_scan.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.angle = 0.0
        self.get_logger().info('Fake scan publisher started')

    def publish_scan(self):
        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_footprint'
        t2.transform.translation.x = math.cos(self.angle) * 2.0
        t2.transform.translation.y = math.sin(self.angle) * 2.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

        # Publish scan
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = -3.14
        scan.angle_max = 3.14
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0

        ranges = []
        for i in range(628):  # 360 degrees
            angle = -3.14 + i * 0.01
            # Create a simple square room
            if abs(angle) < 0.5 or abs(angle - 3.14) < 0.5:
                ranges.append(3.0)
            elif abs(angle - 1.57) < 0.5 or abs(angle + 1.57) < 0.5:
                ranges.append(3.0)
            else:
                ranges.append(5.0 + math.sin(angle * 2) * 0.5)

        scan.ranges = ranges
        self.publisher.publish(scan)
        self.angle += 0.01

def main():
    rclpy.init()
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

        echo -e "${GREEN}✓ Fake scan publisher created${NC}"
        echo ""
        echo "To test with fake data, open 4 terminals:"
        echo ""
        echo -e "${BLUE}Terminal 1: Fake Robot${NC}"
        echo "  python3 /tmp/fake_scan.py"
        echo ""
        echo -e "${BLUE}Terminal 2: SLAM${NC}"
        echo "  ros2 launch slam_toolbox online_async_launch.py"
        echo ""
        echo -e "${BLUE}Terminal 3: Uncertainty Node${NC}"
        echo "  cd ~/slam_uncertainty_ws && source install/setup.bash"
        echo "  ros2 run uncertainty_slam uncertainty_node"
        echo ""
        echo -e "${BLUE}Terminal 4: RViz${NC}"
        echo "  cd ~/slam_uncertainty_ws && source install/setup.bash"
        echo "  ros2 run rviz2 rviz2"
        echo ""
        echo -e "${YELLOW}Note: Fake robot moves in a circle automatically${NC}"
        echo ""

        read -p "Launch fake scan publisher now? (y/n): " launch
        if [ "$launch" = "y" ]; then
            python3 /tmp/fake_scan.py
        fi
        ;;

    4)
        echo ""
        echo -e "${BLUE}Checking installation status...${NC}"
        echo ""

        # Check TurtleBot3
        if ros2 pkg list | grep -q turtlebot3_gazebo; then
            echo -e "${GREEN}✓ TurtleBot3 installed${NC}"
        else
            echo -e "${RED}✗ TurtleBot3 not installed${NC}"
            echo "  Install: sudo apt-get install ros-humble-turtlebot3-gazebo"
        fi

        # Check uncertainty_slam
        if ros2 pkg list | grep -q uncertainty_slam; then
            echo -e "${GREEN}✓ uncertainty_slam package available${NC}"
        else
            echo -e "${RED}✗ uncertainty_slam not built${NC}"
            echo "  Build: colcon build --packages-select uncertainty_slam"
        fi

        # Check SLAM Toolbox
        if ros2 pkg list | grep -q slam_toolbox; then
            echo -e "${GREEN}✓ SLAM Toolbox installed${NC}"
        else
            echo -e "${RED}✗ SLAM Toolbox not installed${NC}"
            echo "  Install: sudo apt-get install ros-humble-slam-toolbox"
        fi

        # Check files
        if [ -f "src/uncertainty_slam/worlds/cave.world" ]; then
            echo -e "${GREEN}✓ cave.world exists${NC}"
        else
            echo -e "${RED}✗ cave.world missing${NC}"
        fi

        if [ -f "src/uncertainty_slam/scripts/run_stage_fixed.py" ]; then
            echo -e "${GREEN}✓ Stage wrapper exists${NC}"
        else
            echo -e "${RED}✗ Stage wrapper missing${NC}"
        fi

        echo ""
        echo "Available testing methods:"
        echo "  1. TurtleBot3 (if installed)"
        echo "  2. Stage with wrapper"
        echo "  3. Fake scan data (always works)"
        echo ""
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
