# Testing Guide: Uncertainty-Aware SLAM

## Quick Start (5 Minutes)

This guide will get your uncertainty-aware SLAM system running step-by-step.

---

## ✅ Files Created

I've created the following files for you:

1. **cave.world** - Stage simulator environment
   - Location: `src/uncertainty_slam/worlds/cave.world`
   - 12x12m room with obstacles

2. **uncertainty_slam.rviz** - Visualization configuration
   - Location: `src/uncertainty_slam/config/uncertainty_slam.rviz`
   - Shows occupancy grid + entropy heat-map

3. **Package rebuilt** - All files installed

---

## 🚀 Method 1: Full Launch (Recommended for Testing)

Open **5 separate terminals** and run each command in order:

### Terminal 1: Stage Simulator
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run stage_ros2 stage_ros2 install/uncertainty_slam/share/uncertainty_slam/worlds/cave.world
```
**Expected:** Stage window appears with robot in 12x12m room

**If error:** Make sure stage_ros2 is installed:
```bash
sudo apt-get install ros-humble-stage-ros2
```

---

### Terminal 2: SLAM Toolbox
```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```
**Expected:**
```
[async_slam_toolbox_node]: Message Filter dropping message...
[async_slam_toolbox_node]: Registering sensor...
```

---

### Terminal 3: Uncertainty Node
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```
**Expected:**
```
[uncertainty_slam_node]: Uncertainty SLAM Node initialized
[uncertainty_slam_node]: Publishing entropy grid at 10.0 Hz
```

---

### Terminal 4: RViz Visualization
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz
```
**Expected:** RViz window opens with:
- Grid
- Occupancy map (gray/black)
- Entropy map (colored heat-map)
- Laser scan (white points)
- TF frames

---

### Terminal 5: Robot Control
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Expected:** Control instructions appear

**Controls:**
- `W` - Move forward
- `S` - Move backward
- `A` - Turn left
- `D` - Turn right
- `X` - Stop
- `Q` / `Z` - Increase/decrease speed

---

## 🎯 Verification Checklist

Run these commands to verify everything works:

### Check Topics
```bash
ros2 topic list | grep -E "map|entropy|scan"
```
**Expected output:**
```
/entropy_map
/map
/map_average_entropy
/map_max_entropy
/scan
```

### Check Entropy Publishing Rate
```bash
ros2 topic hz /entropy_map
```
**Expected:** `average rate: ~10.0 Hz`

### Check Entropy Values
```bash
ros2 topic echo /map_average_entropy --once
```
**Expected:**
```
data: 0.123  # Some value between 0.0 and 1.0
```

### Check Map Publishing
```bash
ros2 topic echo /map --once | head -20
```
**Expected:** OccupancyGrid message with data array

---

## 📊 What You Should See

### In Stage Window:
- 12x12 meter room with walls (gray)
- Colored obstacle blocks (red, blue, green, yellow, orange, purple, cyan)
- Blue robot at starting position (-4, -4)
- Robot moves when you press W/A/S/D/X

### In RViz:
- **Gray/Black Grid**: Occupancy map from SLAM Toolbox
  - White = unknown
  - Gray = free space
  - Black = obstacles

- **Colored Heat-map**: Entropy map (uncertainty)
  - Blue/Green = Low uncertainty (confident)
  - Yellow/Orange = Medium uncertainty
  - Red = High uncertainty (needs exploration)

- **White Points**: Laser scan rays
- **Colored Axes**: TF frames (map, odom, base_link)

### In Terminal 3 (Uncertainty Node):
```
[uncertainty_slam_node]: Entropy Stats - Avg: 0.3245 bits, Max: 0.8234 bits, Valid cells: 5234
```
(Updates every 5 seconds)

---

## 🐛 Troubleshooting

### Error: "cave.world does not exist"
**Solution:**
```bash
cd ~/slam_uncertainty_ws/src/uncertainty_slam/worlds
ls -l cave.world
# Should show: -rw-r--r-- 1 rohan rohan ... cave.world

# If missing, rebuild:
cd ~/slam_uncertainty_ws
colcon build --packages-select uncertainty_slam --symlink-install
```

---

### Error: "Failed to load library for plugin"
**Solution:** This is the old RViz config issue - it's been fixed. Make sure you rebuilt:
```bash
cd ~/slam_uncertainty_ws
rm -rf build/uncertainty_slam install/uncertainty_slam
colcon build --packages-select uncertainty_slam --symlink-install
source install/setup.bash
```

---

### Error: Stage window doesn't appear
**Solution:**
```bash
# Check if stage_ros2 is installed
dpkg -l | grep stage

# If not found, install:
sudo apt-get update
sudo apt-get install ros-humble-stage-ros2

# Or try regular stage_ros:
sudo apt-get install ros-humble-stage-ros
```

---

### Warning: "Shutdown errors" when pressing Ctrl+C
**Solution:** These are **harmless**. They occur because:
1. ROS2 nodes shutdown in random order
2. Some nodes try to communicate with already-shutdown nodes
3. This doesn't affect functionality

**Safe to ignore:**
```
[ERROR] [rcl]: Failed to fini publisher...
[ERROR] [rcl]: Failed to fini subscription...
```

---

### Entropy map not showing in RViz
**Solution:**
1. Check uncertainty node is running (Terminal 3)
2. Check topics exist:
   ```bash
   ros2 topic list | grep entropy
   ```
3. In RViz, manually add the display:
   - Click "Add" button
   - Select "By topic"
   - Find `/entropy_map` → Map
   - Click OK
   - Set "Color Scheme" to "costmap"

---

### Robot doesn't move
**Solution:**
1. Make sure teleop terminal is focused
2. Press `X` to stop, then `W` to move forward
3. Check cmd_vel topic:
   ```bash
   ros2 topic echo /cmd_vel
   # Press W in teleop terminal
   # Should see velocity values
   ```

---

## 🎓 Understanding the Output

### Entropy Values Meaning

The uncertainty node computes:

```
Entropy = Variance / MaxVariance (normalized to 0-1)
```

For each cell:
- **0.0** = Very certain (observed many times, consistent values)
- **0.5** = Moderate uncertainty
- **1.0** = Very uncertain (observed few times, or conflicting observations)

### Expected Behavior

1. **Start:** Entropy high everywhere (unknown map)
2. **During exploration:**
   - Entropy decreases in visited areas
   - Entropy remains high in unvisited areas
3. **After full exploration:**
   - Low entropy in open spaces (consistent free)
   - Low entropy at obstacles (consistent occupied)
   - Higher entropy at edges/boundaries

---

## 📈 Next Steps

### Run Active Exploration
```bash
# After basic test works, try active exploration:
ros2 run uncertainty_slam active_explorer
```

The robot will automatically navigate to high-entropy regions!

### Log ECS Data
```bash
ros2 run uncertainty_slam ecs_logger \
    --ros-args \
    -p experiment_name:=test_run \
    -p record_duration:=60.0
```

Results saved to: `src/uncertainty_slam/results/ecs_logs/`

### Benchmark Accuracy
```bash
# First, generate ground truth:
cd ~/slam_uncertainty_ws/src/uncertainty_slam
python3 scripts/generate_ground_truth.py \
    -o results/ground_truth/cave_gt.json \
    -r 0.05

# Then run benchmark:
ros2 run uncertainty_slam benchmark_accuracy \
    --ros-args \
    -p ground_truth_map_file:=$(pwd)/results/ground_truth/cave_gt.json
```

---

## 🎯 Success Criteria

You know it's working when:

- [ ] Stage window shows robot and environment
- [ ] Robot moves with keyboard (W/A/S/D/X)
- [ ] RViz shows occupancy grid building in real-time
- [ ] RViz shows entropy heat-map updating
- [ ] Entropy decreases in explored areas
- [ ] Entropy remains high in unexplored areas
- [ ] Terminal 3 shows entropy statistics every 5 seconds
- [ ] No critical errors in any terminal

---

## 📞 Still Having Issues?

1. **Check ROS2 environment:**
   ```bash
   echo $ROS_DISTRO
   # Should show: humble
   ```

2. **Verify package installation:**
   ```bash
   ros2 pkg list | grep uncertainty
   # Should show: uncertainty_slam
   ```

3. **Check file exists:**
   ```bash
   ls -l ~/slam_uncertainty_ws/install/uncertainty_slam/share/uncertainty_slam/worlds/cave.world
   # Should show file details
   ```

4. **Full clean rebuild:**
   ```bash
   cd ~/slam_uncertainty_ws
   rm -rf build install log
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 📚 Additional Resources

- **README.md** - Complete documentation
- **QUICKSTART.md** - 5-minute quick start
- **FIXES_APPLIED.md** - All issues fixed
- **IMPLEMENTATION_SUMMARY.md** - Technical details

---

**Your system is ready to test!** 🎉

Just follow Terminal 1-5 instructions above and you should see everything working.
