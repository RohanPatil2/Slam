# Quick Start Guide

This guide will help you get the uncertainty-aware SLAM system running in under 5 minutes.

## Step 1: Build the Workspace

```bash
cd ~/slam_uncertainty_ws
colcon build --symlink-install
source install/setup.bash
```

Expected output: Build should complete without errors.

## Step 2: Launch the Demo

Run the complete system with one command:

```bash
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py
```

This will open:
- **Stage simulator** window showing the robot in a test environment
- **RViz** window for visualization

## Step 3: Verify Topics

In a new terminal, check that all topics are publishing:

```bash
source ~/slam_uncertainty_ws/install/setup.bash
ros2 topic list
```

You should see:
- `/map` - Occupancy grid from GMapping
- `/entropy_map` - Uncertainty heat-map
- `/entropy` - Pose entropy
- `/map_average_entropy` - Average cell entropy
- `/map_max_entropy` - Maximum cell entropy

Check the publishing rate:

```bash
ros2 topic hz /entropy_map
```

Expected: ~10 Hz or higher

## Step 4: Control the Robot

### Option A: Keyboard Teleoperation

In a new terminal:

```bash
sudo apt-get install ros-humble-teleop-twist-keyboard  # if not installed
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys shown on screen to drive the robot.

### Option B: Active Exploration (Automated)

Stop the current demo (Ctrl+C) and relaunch with active exploration:

```bash
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py enable_explorer:=true
```

The robot will now autonomously navigate to high-entropy regions!

## Step 5: View the Entropy Map in RViz

In RViz:

1. Click "Add" button (bottom left)
2. Select "By topic" tab
3. Find `/entropy_map` → Map
4. Click OK

The entropy map should now appear:
- **Green/Blue**: Low uncertainty (confident)
- **Yellow/Orange**: Medium uncertainty
- **Red**: High uncertainty (needs exploration)

## Step 6: Monitor the ECS Logger

The ECS logger is running in the background. Check the logs:

```bash
# In the terminal where you launched the demo, you'll see periodic updates like:
# [ecs_logger]: Entropy Stats - Avg: 0.3245 bits, Max: 0.8234 bits, Valid cells: 5234
```

Results are automatically saved to:
```
~/slam_uncertainty_ws/src/uncertainty_slam/results/ecs_logs/
```

## Step 7: View Results

After the demo (default 120 seconds), find results:

```bash
cd ~/slam_uncertainty_ws/src/uncertainty_slam/results/ecs_logs/
ls -lt  # Show latest files
```

You'll find:
- `*_report.json` - ECS metrics and summary
- `*_raw.json` - Raw time-series data
- `*_plots.png` - Visualization plots

View the plots:

```bash
eog *_plots.png  # or your preferred image viewer
```

## Common Issues & Solutions

### Issue: "Stage not found"

**Solution:**
```bash
sudo apt-get install ros-humble-stage-ros
```

### Issue: "No map published"

**Solution:**
- Check that laser scanner topic is correct: `ros2 topic echo /base_scan`
- Verify TF transforms: `ros2 run tf2_tools view_frames`
- Restart the demo

### Issue: "Entropy map shows all zeros"

**Solution:**
- Drive the robot to explore more of the environment
- Wait for sufficient observations (~10 seconds)
- Check that GMapping is running: `ros2 node list | grep slam_gmapping`

### Issue: "Active explorer not moving"

**Solution:**
```bash
# Check if exploration is enabled
ros2 topic pub /active_exploration/enable std_msgs/msg/Bool "data: true" --once

# Lower the entropy threshold
ros2 param set /active_explorer min_entropy_threshold 30.0
```

## Next Steps

### Run Custom Experiments

```bash
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py \
    experiment_name:=my_experiment \
    record_duration:=300.0 \
    enable_explorer:=true
```

### Generate Ground Truth for Benchmarking

```bash
cd ~/slam_uncertainty_ws/src/uncertainty_slam
python3 scripts/generate_ground_truth.py -o results/ground_truth.json
```

### Run Benchmark Comparison

```bash
ros2 run uncertainty_slam benchmark_accuracy \
    --ros-args \
    -p ground_truth_map_file:=$(pwd)/results/ground_truth.json
```

### Compare ECS Scores

Run multiple experiments with different GMapping parameters:

```bash
# Experiment 1: Default parameters
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py \
    experiment_name:=exp_default

# Experiment 2: Higher particle count
# (Edit launch file or use parameter overrides)

# Compare ECS scores in the reports
```

## Understanding the Output

### ECS Report Structure

```json
{
  "experiment": "my_experiment",
  "timestamp": "2025-10-28T...",
  "pose_ecs": {
    "ecs": 0.3245,              // Lower is better
    "auc": 0.4123,              // Area under curve
    "convergence_rate": 0.45    // 45% reduction in entropy
  },
  "avg_map_ecs": { ... },
  "map_statistics": { ... }
}
```

### Key Metrics

- **ECS < 0.3**: Excellent convergence
- **ECS 0.3-0.5**: Good convergence
- **ECS > 0.5**: Slow convergence (needs parameter tuning)

## Tips for Best Results

1. **Let the robot explore**: Drive through the entire environment
2. **Avoid rapid rotation**: Smooth motions give better results
3. **Run for 2+ minutes**: Allows entropy to converge
4. **Compare multiple runs**: Statistical significance
5. **Tune GMapping params**: See README for parameter guide

## Video Recording

To record a demo video:

```bash
# Install dependencies
sudo apt-get install ros-humble-rosbag2

# Start recording
ros2 bag record -o my_demo_bag /map /entropy_map /scan /tf /tf_static

# In another terminal, run the demo
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py enable_explorer:=true

# Stop recording after sufficient time (Ctrl+C)
```

## Getting Help

- **README.md**: Full documentation
- **Check logs**: Look for errors in terminal output
- **ROS2 debugging**: `ros2 doctor --report`
- **GitHub Issues**: Report bugs and ask questions

## Success Checklist

- [ ] Stage simulator window opens
- [ ] RViz shows map building
- [ ] Entropy map publishes at ≥10 Hz
- [ ] Robot navigates (manually or autonomously)
- [ ] ECS logger saves results after demo
- [ ] Plots generated successfully

If all boxes are checked, you're ready to run experiments! 🎉

---

**Next:** Read the full [README.md](README.md) for advanced usage and parameter tuning.
