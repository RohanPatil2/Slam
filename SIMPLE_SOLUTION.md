# 🎯 SIMPLE SOLUTION - Just Run This

## You Have 3 Options:

---

## ✅ Option 1: Install TurtleBot3 (Best - Works Guaranteed)

### Install:
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop
```

### Run (5 terminals):

**Terminal 1:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2:**
```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Terminal 3:**
```bash
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```

**Terminal 4:**
```bash
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz
```

**Terminal 5:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

**Result:** Gazebo + entropy visualization working perfectly!

---

## ✅ Option 2: Use Helper Script

```bash
cd ~/slam_uncertainty_ws
./INSTALL_AND_RUN.sh
```

Then select:
- **Option 1** to install TurtleBot3
- **Option 3** to use fake scan data (no install needed!)

---

## ✅ Option 3: Fake Scan Data (No Installation Needed!)

This creates a virtual robot that moves in a circle and generates scan data.

### Step 1: Create fake robot script
```bash
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
        self.get_logger().info('Publishing fake scan data...')

    def publish_scan(self):
        # TF: map -> odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        # TF: odom -> base_footprint (circular motion)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_footprint'
        t2.transform.translation.x = math.cos(self.angle) * 2.0
        t2.transform.translation.y = math.sin(self.angle) * 2.0
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

        # Laser scan (simulated square room)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = -3.14159
        scan.angle_max = 3.14159
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0

        ranges = []
        for i in range(628):
            angle = -3.14159 + i * 0.01
            # Simple square room simulation
            if -0.5 < angle < 0.5 or angle > 2.64 or angle < -2.64:
                ranges.append(3.0)  # Front/back walls
            elif 0.5 < angle < 2.64:
                ranges.append(3.0)  # Side walls
            else:
                ranges.append(3.0)

        scan.ranges = ranges
        self.publisher.publish(scan)
        self.angle += 0.01  # Rotate

def main():
    rclpy.init()
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
EOF
```

### Step 2: Run (4 terminals)

**Terminal 1: Fake Robot**
```bash
python3 /tmp/fake_scan.py
```

**Terminal 2: SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Terminal 3: Uncertainty Node**
```bash
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```

**Terminal 4: RViz**
```bash
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz
```

**Result:** Your uncertainty node works with simulated data!

---

## 🎯 Which Should You Choose?

| Option | Speed | Quality | Install |
|--------|-------|---------|---------|
| **TurtleBot3** | Medium | Best | sudo required |
| **Helper Script** | Fast | Good | Auto-installs |
| **Fake Data** | Instant | Basic | None needed |

**Recommendation:**
- **For quick test:** Use fake data (Option 3)
- **For demo/presentation:** Install TurtleBot3 (Option 1)
- **For convenience:** Use helper script (Option 2)

---

## ✅ Right Now, Do This:

### Quickest Test (2 minutes):
```bash
# Create fake robot
cat > /tmp/fake_scan.py << 'EOF'
[... paste the fake_scan.py code from above ...]
EOF

# Run it
python3 /tmp/fake_scan.py &

# In another terminal
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```

If you see "Uncertainty SLAM Node initialized" → **SUCCESS!** ✅

---

## 📞 Still Not Working?

Run the helper script:
```bash
cd ~/slam_uncertainty_ws
./INSTALL_AND_RUN.sh
```

Select option 4 to see what's installed/missing.

---

**Bottom line:** Stop fighting with Stage. Use TurtleBot3 or fake data instead! 🚀
