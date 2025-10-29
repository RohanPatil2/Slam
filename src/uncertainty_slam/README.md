# Real-Time Uncertainty-Aware 2-D SLAM

**Author:** Rohan Upendra Patil

A ROS2 implementation of uncertainty-aware SLAM that extends GMapping with real-time Shannon entropy quantification for 2D occupancy grids.

## Overview

This project addresses a key limitation in traditional occupancy-grid SLAM: all occupied/free estimates are treated as equally reliable. By adding a lightweight Bayesian uncertainty head to the Rao-Blackwellised Particle Filter (RBPF) in GMapping, this system provides:

1. **Real-time entropy heat-maps** (≥10 Hz) showing per-cell uncertainty
2. **Entropy Convergence Score (ECS)** metric for judging convergence speed
3. **Active exploration** capabilities using entropy-driven navigation

## Key Features

### 1. Uncertainty Quantification Node (`uncertainty_node`)
- Tracks per-particle variance for each grid cell
- Computes Shannon entropy from variance using Bayesian approximation
- Publishes entropy grid at ≥10 Hz for real-time visualization
- Minimal computational overhead

### 2. Entropy Convergence Score (ECS) Logger (`ecs_logger`)
- Time-weighted metric: ECS = ∫[0,T] (entropy(t) * w(t)) dt / T
- Emphasizes early convergence with exponential time weighting
- Automatic data logging and visualization
- Comprehensive performance metrics

### 3. Active Explorer (`active_explorer`)
- Identifies high-entropy regions in the map
- Navigates robot to uncertain areas to reduce entropy
- Demonstrates practical utility of entropy map
- Simple proportional controller for differential drive

### 4. Benchmarking Tools
- Ground truth comparison with RMSE calculation
- Accuracy, precision, recall, and F1 metrics
- Automated evaluation at regular intervals

## Installation

### Prerequisites
- ROS2 Humble
- Python 3.10+
- Stage simulator
- GMapping (included in workspace)

### Dependencies

```bash
sudo apt-get install ros-humble-stage-ros ros-humble-tf2-* ros-humble-nav-msgs \
    ros-humble-sensor-msgs ros-humble-geometry-msgs python3-numpy python3-matplotlib
```

### Build

```bash
cd ~/slam_uncertainty_ws
colcon build --packages-select uncertainty_slam slam_gmapping openslam_gmapping
source install/setup.bash
```

## Usage

### Quick Start - Full Demo

Launch the complete uncertainty-aware SLAM system with Stage simulator:

```bash
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py
```

This launches:
- Stage simulator with TurtleBot3 in test environment
- GMapping SLAM node
- Uncertainty tracking node
- ECS logger
- RViz visualization

### With Active Exploration

Enable the active explorer to autonomously navigate to high-entropy regions:

```bash
ros2 launch uncertainty_slam uncertainty_slam_demo.launch.py enable_explorer:=true
```

### Manual Teleoperation

In a separate terminal, use keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Running Individual Nodes

#### 1. Uncertainty Node Only
```bash
ros2 run uncertainty_slam uncertainty_node
```

#### 2. ECS Logger with Custom Parameters
```bash
ros2 run uncertainty_slam ecs_logger \
    --ros-args \
    -p experiment_name:=my_experiment \
    -p record_duration:=300.0 \
    -p time_weight_factor:=2.5
```

#### 3. Active Explorer
```bash
ros2 run uncertainty_slam active_explorer \
    --ros-args \
    -p min_entropy_threshold:=60.0 \
    -p max_exploration_distance:=5.0
```

#### 4. Benchmark Accuracy (with ground truth)
```bash
ros2 run uncertainty_slam benchmark_accuracy \
    --ros-args \
    -p ground_truth_map_file:=/path/to/ground_truth.json
```

## ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/entropy_map` | `nav_msgs/OccupancyGrid` | Per-cell Shannon entropy (0-100 scale) |
| `/map_average_entropy` | `std_msgs/Float64` | Mean entropy across all known cells |
| `/map_max_entropy` | `std_msgs/Float64` | Maximum cell entropy |
| `/entropy` | `std_msgs/Float64` | Pose entropy from particle filter |
| `/map` | `nav_msgs/OccupancyGrid` | Standard occupancy grid from GMapping |
| `/active_exploration/current_goal` | `geometry_msgs/PoseStamped` | Current exploration goal |
| `/active_exploration/goal_entropy` | `std_msgs/Float64` | Entropy at current goal |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid for uncertainty tracking |
| `/active_exploration/enable` | `std_msgs/Bool` | Enable/disable active exploration |

## Parameters

### Uncertainty Node

```yaml
uncertainty_slam_node:
  ros__parameters:
    entropy_publish_rate: 10.0          # Hz (target: ≥10 Hz)
    map_topic: /map
    entropy_grid_topic: /entropy_map
    pose_entropy_topic: /entropy
    particle_count: 30
    entropy_window_size: 10
```

### ECS Logger

```yaml
ecs_logger:
  ros__parameters:
    output_dir: ./results/ecs_logs
    experiment_name: default_exp
    time_weight_factor: 2.0             # Exponential decay factor
    record_duration: 120.0              # seconds
    auto_save_interval: 10.0            # seconds
```

### Active Explorer

```yaml
active_explorer:
  ros__parameters:
    min_entropy_threshold: 50.0         # 0-100 scale
    max_exploration_distance: 5.0       # meters
    control_frequency: 10.0             # Hz
    linear_velocity: 0.3                # m/s
    angular_velocity: 0.5               # rad/s
    goal_tolerance: 0.3                 # meters
```

## Project Structure

```
uncertainty_slam/
├── uncertainty_slam/           # Python package
│   ├── uncertainty_node.py    # Main uncertainty quantification
│   ├── active_explorer.py     # Active exploration
│   ├── ecs_logger.py          # ECS metric logger
│   └── __init__.py
├── scripts/                   # Standalone scripts
│   ├── ecs_logger.py
│   └── benchmark_accuracy.py
├── launch/                    # Launch files
│   ├── uncertainty_slam_demo.launch.py
│   └── test_gmapping.launch.py
├── worlds/                    # Stage simulator worlds
│   └── test_environment.world
├── config/                    # Configuration files
│   └── uncertainty_slam.rviz
├── results/                   # Output directory
│   ├── ecs_logs/
│   └── benchmarks/
└── README.md
```

## Algorithms

### Shannon Entropy Computation

For each cell, we track variance across particle observations:

```
σ² = E[X²] - E[X]²
```

Variance is mapped to Shannon entropy using a Beta distribution approximation:

```
H(p) = -p log₂(p) - (1-p) log₂(1-p)
```

where `p` is approximated from normalized variance.

### Entropy Convergence Score (ECS)

Time-weighted integral emphasizing early convergence:

```
ECS = (1/T) ∫₀ᵀ entropy(t) · exp(-α·t/T) dt
```

Lower ECS indicates faster convergence (better performance).

### Active Exploration Strategy

1. Identify cells with entropy > threshold
2. Filter by occupancy (navigable cells only)
3. Select nearest high-entropy cell within max distance
4. Navigate using proportional controller

## Evaluation Metrics

### 1. Map Accuracy (vs Ground Truth)
- **RMSE**: Root Mean Square Error in occupancy values
- **Accuracy**: Percentage of correctly classified cells
- **F1 Score**: Harmonic mean of precision and recall
- **Target**: RMSE ≤ 0.05 m

### 2. Uncertainty Metrics
- **Entropy Convergence Score (ECS)**: Time-weighted entropy integral
- **Average Entropy**: Mean uncertainty across map
- **Max Entropy**: Worst-case cell uncertainty

### 3. Real-Time Performance
- **Entropy Publishing Rate**: Target ≥10 Hz
- **Computation Overhead**: Minimal impact on SLAM

## Expected Outcomes

Based on the project proposal, this implementation provides:

1. ✅ **Real-time entropy grid** at ≥10 Hz
2. ✅ **ECS metric** with automated logging and visualization
3. ✅ **Active exploration** demonstrating entropy map utility
4. ✅ **Benchmark tools** with ground truth comparison
5. ✅ **Open-source release** with documentation

### Performance Targets

- Occupancy grid RMSE ≤ 0.05 m (against ground truth)
- ECS reduction ≥ 25% with tuned parameters
- Real-time operation (entropy updates ≥ 10 Hz)

## Demonstration Scenarios

### Scenario 1: Passive Mapping
Robot navigates via teleoperation while system builds both occupancy and entropy maps.

### Scenario 2: Active Exploration
Robot autonomously seeks high-entropy regions, measurably reducing local uncertainty.

### Scenario 3: Parameter Tuning
Adjust GMapping sensor noise parameters and measure impact on ECS.

## Visualization in RViz

The entropy map can be visualized in RViz:

1. Add a `Map` display
2. Set topic to `/entropy_map`
3. Use a color scheme where:
   - **Blue/Green**: Low entropy (high confidence)
   - **Yellow/Orange**: Medium entropy
   - **Red**: High entropy (high uncertainty)

## Results Directory

After running experiments, find results in:

```
results/
├── ecs_logs/
│   ├── experiment_name_YYYYMMDD_HHMMSS_raw.json
│   ├── experiment_name_YYYYMMDD_HHMMSS_report.json
│   └── experiment_name_YYYYMMDD_HHMMSS_plots.png
└── benchmarks/
    └── benchmark_YYYYMMDD_HHMMSS.json
```

## Troubleshooting

### Entropy map not publishing
- Check that GMapping is publishing `/map`
- Verify particle count matches between nodes
- Ensure sufficient observations (≥100 valid cells)

### Active explorer not moving
- Check `/active_exploration/enable` topic
- Verify entropy map has high-entropy regions
- Adjust `min_entropy_threshold` parameter

### Poor map accuracy
- Tune GMapping parameters (especially motion model)
- Increase particle count
- Adjust laser sensor parameters

## Future Enhancements

- [ ] Multi-resolution entropy grids
- [ ] Information gain-based path planning
- [ ] Integration with Nav2 for navigation
- [ ] 3D entropy visualization
- [ ] Online parameter adaptation

## References

1. Grisetti, G., Stachniss, C., Burgard, W. "Improved Techniques for Grid Mapping with Rao-Blackwellised Particle Filters," IROS 2007.
2. Yu, S., Liu, H., Wang, T. "Gaussian Process Occupancy Maps for Mobile Robot Mapping," RA-L 2019.

## License

Apache-2.0

## Author

**Rohan Upendra Patil**

For questions or issues, please open an issue on the repository.

## Acknowledgments

Based on the open-source GMapping implementation and inspired by uncertainty-aware robotics research.
