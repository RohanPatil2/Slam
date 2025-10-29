# Quick Reference Card

## 🚀 Start Testing in 30 Seconds

```bash
cd ~/slam_uncertainty_ws
./RUN_COMPLETE_SYSTEM.sh
```
Choose option 1, then press `w` to drive!

---

## 📋 Most Common Commands

### Run Complete System
```bash
./RUN_COMPLETE_SYSTEM.sh
```

### Build Package
```bash
colcon build --packages-select uncertainty_slam --symlink-install
source install/setup.bash
```

### Run Components Individually
```bash
# Synthetic robot only
ros2 run uncertainty_slam synthetic_robot

# Uncertainty node only
ros2 run uncertainty_slam uncertainty_node

# Advanced explorer only
ros2 run uncertainty_slam advanced_explorer

# ECS logger only
ros2 run uncertainty_slam ecs_logger --ros-args -p experiment_name:=test1
```

### Launch Complete System
```bash
# Basic
ros2 launch uncertainty_slam complete_system.launch.py

# With active exploration
ros2 launch uncertainty_slam complete_system.launch.py use_active_explorer:=true

# Full system with logging
ros2 launch uncertainty_slam complete_system.launch.py \
    use_active_explorer:=true \
    use_ecs_logger:=true \
    experiment_name:=my_test
```

---

## ⌨️ Keyboard Controls (Synthetic Robot)

| Key | Action |
|-----|--------|
| `w` | Move forward |
| `x` | Move backward |
| `a` | Rotate left |
| `d` | Rotate right |
| `s` | Stop |
| `m` | Manual mode |
| `e` | Exploration pattern mode |
| `q` | Quit |

---

## 📊 Generate Visualizations

### Single Experiment Plot
```bash
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --ecs-report results/ecs_logs/my_exp/ecs_report.json \
    --output-dir results/plots
```

### Compare Multiple Experiments
```bash
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --compare \
        results/ecs_logs/exp1/ecs_report.json \
        results/ecs_logs/exp2/ecs_report.json \
    --labels "Experiment 1" "Experiment 2" \
    --output-dir results/comparison
```

### Publication Figure
```bash
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --ecs-report results/ecs_logs/my_exp/ecs_report.json \
    --publication \
    --output-dir results/paper
```

---

## 🔍 Check System Status

### Topic Rates
```bash
ros2 topic hz /scan                # Should be ~10 Hz
ros2 topic hz /entropy_map         # Should be ~10 Hz
ros2 topic hz /odom                # Should be ~20 Hz
```

### Current Values
```bash
ros2 topic echo /map_average_entropy --once
ros2 topic echo /map_max_entropy --once
ros2 topic echo /robot_mode --once
```

### List All Topics
```bash
ros2 topic list
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
```

---

## 📁 Important Files

### Core Code
- `src/uncertainty_slam/uncertainty_slam/synthetic_robot.py` - Virtual robot
- `src/uncertainty_slam/uncertainty_slam/uncertainty_node.py` - Entropy computation
- `src/uncertainty_slam/uncertainty_slam/advanced_explorer.py` - A* path planning
- `src/uncertainty_slam/uncertainty_slam/ecs_logger.py` - Metrics logging

### Launch Files
- `src/uncertainty_slam/launch/complete_system.launch.py` - Main launcher

### Scripts
- `RUN_COMPLETE_SYSTEM.sh` - Easy system launcher
- `src/uncertainty_slam/scripts/advanced_visualization.py` - Plotting tools

### Documentation
- `README.md` - Main documentation
- `ADVANCED_FEATURES.md` - **Detailed feature guide** ⭐
- `COMPLETE_SYSTEM_GUIDE.md` - Complete usage instructions
- `QUICKSTART.md` - 5-minute getting started
- `PUSH_TO_GITHUB.md` - GitHub upload instructions

### Results
- `results/ecs_logs/<experiment>/` - Experiment data and plots
- `results/visualizations/` - Generated plots
- `results/benchmarks/` - Accuracy metrics

---

## 🛠️ Common Tasks

### Add New Obstacle to Environment
Edit `src/uncertainty_slam/uncertainty_slam/synthetic_robot.py`:
```python
# In _create_default_environment():
self.obstacles.append({
    'type': 'box',
    'cx': 3.0, 'cy': 3.0,  # center
    'w': 1.0, 'h': 1.0      # size
})
```

### Change Robot Speed
Edit `src/uncertainty_slam/uncertainty_slam/synthetic_robot.py`:
```python
self.max_linear_vel = 0.3   # m/s (default: 0.5)
self.max_angular_vel = 0.7  # rad/s (default: 1.0)
```

### Tune Explorer Weights
Edit `src/uncertainty_slam/uncertainty_slam/advanced_explorer.py`:
```python
self.w_distance = 0.3   # Distance weight
self.w_entropy = 0.4    # Entropy weight
self.w_info = 0.3       # Info gain weight
```

### Change Entropy Update Rate
Edit launch file or run:
```bash
ros2 run uncertainty_slam uncertainty_node \
    --ros-args -p publish_rate:=20.0
```

---

## 🐛 Quick Troubleshooting

### Robot doesn't move
- Make sure terminal with robot is focused
- Check you're in correct mode: `ros2 topic echo /robot_mode`

### No entropy map
- Wait for robot to explore some area
- Check topic: `ros2 topic echo /entropy_map --once`

### Build errors
```bash
rm -rf build install log
colcon build --packages-select uncertainty_slam --symlink-install
```

### Missing dependencies
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-slam-toolbox \
    python3-numpy \
    python3-matplotlib
```

---

## 📈 Typical Workflow

### 1. Quick Test (2 min)
```bash
./RUN_COMPLETE_SYSTEM.sh  # Option 1
# Press 'w' to move forward
# Watch entropy build in RViz
```

### 2. Run Experiment (5 min)
```bash
./RUN_COMPLETE_SYSTEM.sh  # Option 4
# Enter name: test_1
# Let run for 5 minutes
```

### 3. Generate Plots
```bash
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --ecs-report results/ecs_logs/test_1/ecs_report.json \
    --publication
```

### 4. Compare Multiple Runs
```bash
# Run 3 experiments with different settings
./RUN_COMPLETE_SYSTEM.sh  # Manual control
./RUN_COMPLETE_SYSTEM.sh  # Basic explorer
./RUN_COMPLETE_SYSTEM.sh  # Advanced explorer

# Compare
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --compare results/ecs_logs/*/ecs_report.json \
    --labels "Manual" "Basic" "Advanced"
```

---

## 🎯 For Your Proposal

### Demonstrate Real-time Entropy
```bash
./RUN_COMPLETE_SYSTEM.sh  # Option 1
# In RViz: Add → By Topic → /entropy_map → Map
# Show heat map updating at 10 Hz
```

### Demonstrate Active Exploration
```bash
./RUN_COMPLETE_SYSTEM.sh  # Option 2
# Robot automatically explores high-entropy regions
```

### Generate ECS Metrics
```bash
./RUN_COMPLETE_SYSTEM.sh  # Option 3
# Experiment name: proposal_demo
# Check: results/ecs_logs/proposal_demo/ecs_report.json
```

### Create Publication Figures
```bash
python3 src/uncertainty_slam/scripts/advanced_visualization.py \
    --ecs-report results/ecs_logs/proposal_demo/ecs_report.json \
    --publication \
    --output-dir results/proposal_figures
```

---

## 📚 Documentation Hierarchy

**Start here:**
1. `QUICK_REFERENCE.md` ← You are here! ⭐
2. `README.md` - Overview
3. `QUICKSTART.md` - 5-minute guide

**For detailed usage:**
4. `COMPLETE_SYSTEM_GUIDE.md` - Comprehensive guide
5. `ADVANCED_FEATURES.md` - **Feature documentation** ⭐

**For specific tasks:**
6. `TESTING_GUIDE.md` - Testing procedures
7. `PUSH_TO_GITHUB.md` - GitHub upload
8. `IMPLEMENTATION_SUMMARY.md` - Technical details

---

## 💡 Tips

- **Press `q` to quit** the synthetic robot (not Ctrl+C)
- **Results auto-save** to `results/ecs_logs/`
- **Build errors?** Delete build/ and rebuild
- **Can't see map?** In RViz, set Fixed Frame to "map"
- **Robot stuck?** Press 's' to stop, then 'm' for manual mode

---

## 🎓 What You Have

✅ Complete working SLAM system
✅ Zero-setup testing (synthetic robot)
✅ Advanced path planning (A* algorithm)
✅ Publication-quality visualizations
✅ Automated metrics (ECS scoring)
✅ Comprehensive documentation
✅ Ready for GitHub
✅ **Ready for submission!**

**You're all set! 🚀**
