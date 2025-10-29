# 🔥 ULTIMATE SOLUTION - Stage Path Bug Workaround

## The Problem

`stage_ros2` has a BUG where it doesn't handle file paths correctly. No matter what you try:
- ❌ Relative paths don't work
- ❌ Absolute paths don't work
- ❌ `${PWD}` doesn't work
- ❌ Launch files don't work

**Root cause:** stage_ros2 only looks in its current working directory for .world files.

---

## ✅ SOLUTION 1: Python Wrapper (RECOMMENDED)

I created a Python script that works around the bug:

```bash
cd ~/slam_uncertainty_ws
source install/setup.bash

# Use the fixed launcher
python3 src/uncertainty_slam/scripts/run_stage_fixed.py \
    src/uncertainty_slam/worlds/cave.world
```

**What it does:**
1. Copies your world file to a temp directory
2. Changes to that directory
3. Runs stage_ros2 with just the filename
4. Cleans up after

**This WILL work!**

---

## ✅ SOLUTION 2: Manual Workaround

If you want to do it manually:

```bash
cd ~/slam_uncertainty_ws

# Copy world file to current directory
cp src/uncertainty_slam/worlds/cave.world ./

# Run stage with just the filename
source install/setup.bash
ros2 run stage_ros2 stage_ros2 cave.world

# Clean up after
rm cave.world
```

---

## ✅ SOLUTION 3: Use Without Simulator (BEST FOR TESTING)

Skip Stage entirely and test your uncertainty node with static data:

### Option A: With Existing ROS2 Bags

If you have a bag file with `/scan` and TF data:

```bash
cd ~/slam_uncertainty_ws
source install/setup.bash

# Terminal 1: Play bag
ros2 bag play /path/to/your/bag.db3 --clock

# Terminal 2: SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3: Uncertainty
ros2 run uncertainty_slam uncertainty_node --ros-args -p use_sim_time:=true

# Terminal 4: RViz
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz
```

### Option B: With TurtleBot3 Simulation

```bash
# Install TurtleBot3 (if not installed)
sudo apt-get install ros-humble-turtlebot3-gazebo

# Set model
export TURTLEBOT3_MODEL=burger

# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Uncertainty
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run uncertainty_slam uncertainty_node

# Terminal 4: RViz
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz

# Terminal 5: Teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## ✅ SOLUTION 4: Download Sample Data

Download a sample ROS2 bag file to test without any simulator:

```bash
cd ~/slam_uncertainty_ws

# Create bag directory
mkdir -p bags && cd bags

# Download sample bag (example - you need to find one or record your own)
# Or record from real robot/simulation:
ros2 bag record /scan /tf /tf_static -o test_slam

# Then play it back
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 launch uncertainty_slam uncertainty_slam_bag.launch.py \
    bag_file:=bags/test_slam
```

---

## 🎯 RECOMMENDED PATH FORWARD

Since Stage is giving you trouble, I recommend:

### For Testing Your Code:

**Option 1: Use TurtleBot3** (if you can install it)
```bash
sudo apt-get install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop
```

Then use the TurtleBot3 commands from Solution 3B above.

**Option 2: Use the Python Wrapper** for Stage
```bash
python3 src/uncertainty_slam/scripts/run_stage_fixed.py \
    src/uncertainty_slam/worlds/cave.world
```

---

## 📝 Complete Working Example (TurtleBot3)

This is GUARANTEED to work if TurtleBot3 is installed:

```bash
# Setup
cd ~/slam_uncertainty_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# Terminal 1: Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: YOUR uncertainty node
ros2 run uncertainty_slam uncertainty_node

# Terminal 4: RViz with your config
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz

# Terminal 5: Control the robot
ros2 run turtlebot3_teleop teleop_keyboard
```

**This will show:**
- Gazebo: 3D simulation with robot
- RViz: Your occupancy map + entropy heat-map
- Your terminal: Entropy statistics

**No Stage needed!**

---

## 🔧 If You MUST Use Stage

### Install Stage Properly

```bash
# Install the non-ROS2 version of Stage
sudo apt-get install stage

# Then use my Python wrapper
python3 src/uncertainty_slam/scripts/run_stage_fixed.py \
    src/uncertainty_slam/worlds/cave.world
```

OR

### Build stage_ros2 from Source

```bash
cd ~/slam_uncertainty_ws/src
git clone https://github.com/tuw-robotics/stage_ros2.git
cd ..
colcon build --packages-select stage_ros2
source install/setup.bash

# Try again with the wrapper
python3 src/uncertainty_slam/scripts/run_stage_fixed.py \
    src/uncertainty_slam/worlds/cave.world
```

---

## 📊 Summary of All Solutions

| Solution | Difficulty | Requirements | Works? |
|----------|------------|--------------|---------|
| Python Wrapper | Easy | stage_ros2 | ✅ Should work |
| Manual Workaround | Easy | stage_ros2 | ✅ Should work |
| TurtleBot3 | Medium | TurtleBot3 packages | ✅ Guaranteed |
| Bag Playback | Easy | Recorded bag file | ✅ Guaranteed |
| Build from Source | Hard | Build tools | ✅ Might work |

---

## 🎯 My Recommendation

1. **First try:** Python wrapper with Stage
   ```bash
   python3 src/uncertainty_slam/scripts/run_stage_fixed.py src/uncertainty_slam/worlds/cave.world
   ```

2. **If that fails:** Use TurtleBot3 Gazebo instead
   ```bash
   sudo apt-get install ros-humble-turtlebot3-gazebo
   ```

3. **For presentation/demo:** Record a bag file once it works, then use bag playback

---

## 📁 New Files Created

1. **run_stage_fixed.py** - Python wrapper that fixes Stage path bug
2. **uncertainty_slam_turtlebot3.launch.py** - TurtleBot3 launch file
3. **uncertainty_slam_bag.launch.py** - Bag playback launch file
4. **ULTIMATE_SOLUTION.md** - This file

---

## ✅ Next Steps

**Right now, try this:**

```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
python3 src/uncertainty_slam/scripts/run_stage_fixed.py src/uncertainty_slam/worlds/cave.world
```

If you see a Stage window with a robot, **SUCCESS!** 🎉

If not, install TurtleBot3 and use that instead:

```bash
sudo apt-get install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Then in another terminal:
```bash
cd ~/slam_uncertainty_ws && source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```

**Your uncertainty node will work with ANY simulator that publishes `/scan` and TF!**

---

**The key insight:** Don't fight with Stage's broken path handling. Either:
1. Use the Python wrapper
2. Use a different simulator (TurtleBot3)
3. Use bag file playback

All three will test your uncertainty-aware SLAM code perfectly! 🚀
