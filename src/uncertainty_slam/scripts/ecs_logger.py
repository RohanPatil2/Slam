#!/usr/bin/env python3
"""
Entropy Convergence Score (ECS) Logger

This script logs entropy metrics over time and computes the
Entropy Convergence Score - a time-weighted metric to judge
convergence speed of the SLAM uncertainty estimates.

ECS = ∫[0,T] (entropy(t) * w(t)) dt / T

where w(t) is a time-weighting function that emphasizes early convergence.

Author: Rohan Upendra Patil
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np
import json
import os
from datetime import datetime
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt


class ECSLogger(Node):
    """
    Logs entropy data and computes Entropy Convergence Score.

    Tracks multiple entropy metrics:
    - Pose entropy (from particle filter weights)
    - Average map entropy (cell-wise)
    - Max map entropy (worst-case cell)
    """

    def __init__(self):
        super().__init__('ecs_logger')

        # Declare parameters
        self.declare_parameter('output_dir', './results/ecs_logs')
        self.declare_parameter('experiment_name', 'default_exp')
        self.declare_parameter('time_weight_factor', 2.0)  # For exponential weighting
        self.declare_parameter('record_duration', 120.0)  # seconds
        self.declare_parameter('auto_save_interval', 10.0)  # seconds

        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.exp_name = self.get_parameter('experiment_name').value
        self.weight_factor = self.get_parameter('time_weight_factor').value
        self.record_duration = self.get_parameter('record_duration').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Data storage
        self.start_time = None
        self.pose_entropy_data = []  # [(time, entropy), ...]
        self.avg_map_entropy_data = []
        self.max_map_entropy_data = []
        self.map_size_data = []  # Track map growth

        # Subscribers
        self.pose_entropy_sub = self.create_subscription(
            Float64,
            '/entropy',
            self.pose_entropy_callback,
            10
        )

        self.avg_entropy_sub = self.create_subscription(
            Float64,
            '/map_average_entropy',
            self.avg_entropy_callback,
            10
        )

        self.max_entropy_sub = self.create_subscription(
            Float64,
            '/map_max_entropy',
            self.max_entropy_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Auto-save timer
        self.save_timer = self.create_timer(
            self.auto_save_interval,
            self.auto_save_callback
        )

        # Recording timer
        if self.record_duration > 0:
            self.record_timer = self.create_timer(
                self.record_duration,
                self.finish_recording
            )

        self.get_logger().info(f'ECS Logger initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Experiment: {self.exp_name}')

    def pose_entropy_callback(self, msg):
        """Log pose entropy from particle filter weights."""
        current_time = self._get_elapsed_time()
        if current_time is not None:
            self.pose_entropy_data.append((current_time, msg.data))

    def avg_entropy_callback(self, msg):
        """Log average map entropy."""
        current_time = self._get_elapsed_time()
        if current_time is not None:
            self.avg_map_entropy_data.append((current_time, msg.data))

    def max_entropy_callback(self, msg):
        """Log maximum map entropy."""
        current_time = self._get_elapsed_time()
        if current_time is not None:
            self.max_map_entropy_data.append((current_time, msg.data))

    def map_callback(self, msg):
        """Track map size evolution."""
        current_time = self._get_elapsed_time()
        if current_time is not None:
            map_cells = msg.info.width * msg.info.height
            known_cells = sum(1 for cell in msg.data if cell >= 0)
            self.map_size_data.append((current_time, map_cells, known_cells))

    def _get_elapsed_time(self):
        """Get elapsed time since start."""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return 0.0

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        return elapsed

    def _compute_time_weight(self, t, T):
        """
        Compute time-dependent weight.

        Emphasizes early convergence with exponential decay:
        w(t) = exp(-α * t / T)

        where α is the weight_factor parameter.

        Args:
            t: Current time
            T: Total duration

        Returns:
            float: Weight in [0, 1]
        """
        if T <= 0:
            return 1.0
        normalized_time = t / T
        weight = np.exp(-self.weight_factor * normalized_time)
        return weight

    def compute_ecs(self, entropy_data):
        """
        Compute Entropy Convergence Score.

        ECS = ∫[0,T] (entropy(t) * w(t)) dt / T

        Lower ECS indicates faster convergence (better).

        Args:
            entropy_data: List of (time, entropy) tuples

        Returns:
            dict: Contains ECS, AUC, and other metrics
        """
        if len(entropy_data) < 2:
            return None

        times = np.array([t for t, _ in entropy_data])
        entropies = np.array([e for _, e in entropy_data])

        T = times[-1]
        if T <= 0:
            return None

        # Compute time weights
        weights = np.array([self._compute_time_weight(t, T) for t in times])

        # Compute weighted entropy
        weighted_entropies = entropies * weights

        # Numerical integration using trapezoidal rule
        ecs = np.trapz(weighted_entropies, times) / T

        # Also compute unweighted AUC for comparison
        auc = np.trapz(entropies, times) / T

        # Compute final entropy (convergence value)
        final_entropy = entropies[-1]

        # Compute initial entropy
        initial_entropy = entropies[0]

        # Compute convergence rate (relative reduction)
        if initial_entropy > 0:
            convergence_rate = (initial_entropy - final_entropy) / initial_entropy
        else:
            convergence_rate = 0.0

        return {
            'ecs': float(ecs),
            'auc': float(auc),
            'initial_entropy': float(initial_entropy),
            'final_entropy': float(final_entropy),
            'convergence_rate': float(convergence_rate),
            'duration': float(T),
            'num_samples': len(entropy_data)
        }

    def generate_report(self):
        """
        Generate comprehensive ECS report.

        Returns:
            dict: Complete analysis results
        """
        report = {
            'experiment': self.exp_name,
            'timestamp': datetime.now().isoformat(),
            'parameters': {
                'time_weight_factor': self.weight_factor,
                'record_duration': self.record_duration
            }
        }

        # Compute ECS for each metric
        if self.pose_entropy_data:
            report['pose_ecs'] = self.compute_ecs(self.pose_entropy_data)

        if self.avg_map_entropy_data:
            report['avg_map_ecs'] = self.compute_ecs(self.avg_map_entropy_data)

        if self.max_map_entropy_data:
            report['max_map_ecs'] = self.compute_ecs(self.max_map_entropy_data)

        # Map statistics
        if self.map_size_data:
            times, total_cells, known_cells = zip(*self.map_size_data)
            report['map_statistics'] = {
                'final_total_cells': int(total_cells[-1]),
                'final_known_cells': int(known_cells[-1]),
                'exploration_ratio': float(known_cells[-1] / total_cells[-1])
            }

        return report

    def save_results(self):
        """Save all logged data and computed metrics."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_filename = f"{self.exp_name}_{timestamp}"

        # Save raw data
        raw_data = {
            'pose_entropy': self.pose_entropy_data,
            'avg_map_entropy': self.avg_map_entropy_data,
            'max_map_entropy': self.max_map_entropy_data,
            'map_size': self.map_size_data
        }

        raw_file = os.path.join(self.output_dir, f"{base_filename}_raw.json")
        with open(raw_file, 'w') as f:
            json.dump(raw_data, f, indent=2)

        self.get_logger().info(f'Saved raw data to {raw_file}')

        # Generate and save report
        report = self.generate_report()
        report_file = os.path.join(self.output_dir, f"{base_filename}_report.json")
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)

        self.get_logger().info(f'Saved report to {report_file}')

        # Generate plots
        self.generate_plots(base_filename)

        return report

    def generate_plots(self, base_filename):
        """Generate visualization plots."""
        try:
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            fig.suptitle(f'Entropy Analysis - {self.exp_name}', fontsize=16)

            # Plot 1: Pose entropy over time
            if self.pose_entropy_data:
                times, entropies = zip(*self.pose_entropy_data)
                axes[0, 0].plot(times, entropies, 'b-', linewidth=2)
                axes[0, 0].set_xlabel('Time (s)')
                axes[0, 0].set_ylabel('Pose Entropy (bits)')
                axes[0, 0].set_title('Pose Entropy Evolution')
                axes[0, 0].grid(True, alpha=0.3)

            # Plot 2: Average map entropy over time
            if self.avg_map_entropy_data:
                times, entropies = zip(*self.avg_map_entropy_data)
                axes[0, 1].plot(times, entropies, 'g-', linewidth=2)
                axes[0, 1].set_xlabel('Time (s)')
                axes[0, 1].set_ylabel('Avg Map Entropy (bits)')
                axes[0, 1].set_title('Average Map Entropy Evolution')
                axes[0, 1].grid(True, alpha=0.3)

            # Plot 3: Max map entropy over time
            if self.max_map_entropy_data:
                times, entropies = zip(*self.max_map_entropy_data)
                axes[1, 0].plot(times, entropies, 'r-', linewidth=2)
                axes[1, 0].set_xlabel('Time (s)')
                axes[1, 0].set_ylabel('Max Map Entropy (bits)')
                axes[1, 0].set_title('Maximum Map Entropy Evolution')
                axes[1, 0].grid(True, alpha=0.3)

            # Plot 4: Map exploration progress
            if self.map_size_data:
                times, total_cells, known_cells = zip(*self.map_size_data)
                exploration_ratio = [k/t*100 if t > 0 else 0 for k, t in zip(known_cells, total_cells)]
                axes[1, 1].plot(times, exploration_ratio, 'm-', linewidth=2)
                axes[1, 1].set_xlabel('Time (s)')
                axes[1, 1].set_ylabel('Exploration (%)')
                axes[1, 1].set_title('Map Exploration Progress')
                axes[1, 1].grid(True, alpha=0.3)

            plt.tight_layout()

            plot_file = os.path.join(self.output_dir, f"{base_filename}_plots.png")
            plt.savefig(plot_file, dpi=150, bbox_inches='tight')
            plt.close()

            self.get_logger().info(f'Saved plots to {plot_file}')

        except Exception as e:
            self.get_logger().error(f'Failed to generate plots: {str(e)}')

    def auto_save_callback(self):
        """Periodic auto-save."""
        if len(self.pose_entropy_data) > 0 or len(self.avg_map_entropy_data) > 0:
            self.get_logger().info('Auto-saving data...')
            self.save_results()

    def finish_recording(self):
        """Finish recording and save final results."""
        self.get_logger().info('Recording duration completed. Saving final results...')
        report = self.save_results()

        # Print summary
        self.get_logger().info('=' * 60)
        self.get_logger().info('ENTROPY CONVERGENCE SCORE SUMMARY')
        self.get_logger().info('=' * 60)

        if 'pose_ecs' in report and report['pose_ecs']:
            ecs_data = report['pose_ecs']
            self.get_logger().info(f"Pose ECS: {ecs_data['ecs']:.6f}")
            self.get_logger().info(f"  - AUC: {ecs_data['auc']:.6f}")
            self.get_logger().info(f"  - Convergence rate: {ecs_data['convergence_rate']*100:.2f}%")

        if 'avg_map_ecs' in report and report['avg_map_ecs']:
            ecs_data = report['avg_map_ecs']
            self.get_logger().info(f"Avg Map ECS: {ecs_data['ecs']:.6f}")
            self.get_logger().info(f"  - AUC: {ecs_data['auc']:.6f}")
            self.get_logger().info(f"  - Convergence rate: {ecs_data['convergence_rate']*100:.2f}%")

        if 'map_statistics' in report:
            stats = report['map_statistics']
            self.get_logger().info(f"Map exploration: {stats['exploration_ratio']*100:.2f}%")

        self.get_logger().info('=' * 60)

        # Cancel timers
        self.save_timer.cancel()
        if hasattr(self, 'record_timer'):
            self.record_timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = ECSLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save on exit
        node.get_logger().info('Interrupted. Saving final data...')
        node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
