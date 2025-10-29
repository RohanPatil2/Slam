#!/usr/bin/env python3
"""
Benchmark script to compare generated map against ground truth.

Computes RMSE and other accuracy metrics between SLAM-generated
occupancy grid and a known ground truth map.

Author: Rohan Upendra Patil
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import json
import os
from datetime import datetime


class MapBenchmark(Node):
    """
    Benchmark node for map accuracy evaluation.

    Compares SLAM-generated map to ground truth and computes metrics:
    - RMSE (Root Mean Square Error)
    - Accuracy (percentage of correctly classified cells)
    - Precision and Recall for occupied cells
    """

    def __init__(self):
        super().__init__('map_benchmark')

        # Declare parameters
        self.declare_parameter('ground_truth_map_file', '')
        self.declare_parameter('slam_map_topic', '/map')
        self.declare_parameter('output_dir', './results/benchmarks')
        self.declare_parameter('evaluation_interval', 5.0)  # seconds
        self.declare_parameter('num_evaluations', 20)
        self.declare_parameter('occupied_threshold', 50)  # OccupancyGrid threshold

        # Get parameters
        self.gt_map_file = self.get_parameter('ground_truth_map_file').value
        slam_topic = self.get_parameter('slam_map_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.eval_interval = self.get_parameter('evaluation_interval').value
        self.num_evals = self.get_parameter('num_evaluations').value
        self.occ_threshold = self.get_parameter('occupied_threshold').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # State
        self.ground_truth_map = None
        self.current_slam_map = None
        self.evaluation_results = []
        self.evaluation_count = 0

        # Load ground truth
        if self.gt_map_file and os.path.exists(self.gt_map_file):
            self.load_ground_truth()
        else:
            self.get_logger().warn('No ground truth file provided or file does not exist')

        # Subscriber
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid,
            slam_topic,
            self.slam_map_callback,
            10
        )

        # Evaluation timer
        if self.ground_truth_map is not None:
            self.eval_timer = self.create_timer(
                self.eval_interval,
                self.evaluate_callback
            )

        self.get_logger().info('Map Benchmark node initialized')

    def load_ground_truth(self):
        """Load ground truth map from file (JSON or PGM format)."""
        try:
            if self.gt_map_file.endswith('.json'):
                with open(self.gt_map_file, 'r') as f:
                    gt_data = json.load(f)
                    # Assume format: {"width": W, "height": H, "data": [...]}
                    self.ground_truth_map = {
                        'width': gt_data['width'],
                        'height': gt_data['height'],
                        'resolution': gt_data.get('resolution', 0.05),
                        'data': np.array(gt_data['data'], dtype=np.int8)
                    }
                    self.get_logger().info(f'Loaded ground truth map: {self.ground_truth_map["width"]}x{self.ground_truth_map["height"]}')
            else:
                self.get_logger().error(f'Unsupported ground truth format: {self.gt_map_file}')

        except Exception as e:
            self.get_logger().error(f'Failed to load ground truth: {str(e)}')
            self.ground_truth_map = None

    def slam_map_callback(self, msg):
        """Receive SLAM-generated map."""
        self.current_slam_map = msg

    def align_maps(self, slam_map_msg):
        """
        Align SLAM map with ground truth.

        Handles different origins and resolutions.

        Returns:
            tuple: (gt_aligned, slam_aligned) numpy arrays
        """
        # For simplicity, assume same resolution and overlapping region
        # In practice, you'd need proper alignment based on origins

        slam_width = slam_map_msg.info.width
        slam_height = slam_map_msg.info.height
        slam_data = np.array(slam_map_msg.data, dtype=np.int8).reshape((slam_height, slam_width))

        gt_width = self.ground_truth_map['width']
        gt_height = self.ground_truth_map['height']
        gt_data = self.ground_truth_map['data'].reshape((gt_height, gt_width))

        # Find overlapping region
        min_width = min(slam_width, gt_width)
        min_height = min(slam_height, gt_height)

        # Extract overlapping region
        slam_aligned = slam_data[:min_height, :min_width]
        gt_aligned = gt_data[:min_height, :min_width]

        return gt_aligned, slam_aligned

    def compute_metrics(self, gt_map, slam_map):
        """
        Compute accuracy metrics.

        Args:
            gt_map: Ground truth map array
            slam_map: SLAM-generated map array

        Returns:
            dict: Metrics dictionary
        """
        # Filter known cells (not -1)
        gt_known = gt_map >= 0
        slam_known = slam_map >= 0
        both_known = gt_known & slam_known

        if np.sum(both_known) == 0:
            return None

        gt_cells = gt_map[both_known]
        slam_cells = slam_map[both_known]

        # RMSE
        rmse = np.sqrt(np.mean((gt_cells - slam_cells) ** 2))

        # Convert to binary (occupied vs free)
        gt_occupied = gt_cells > self.occ_threshold
        slam_occupied = slam_cells > self.occ_threshold

        # Accuracy
        accuracy = np.mean(gt_occupied == slam_occupied)

        # True positives, false positives, false negatives
        tp = np.sum(gt_occupied & slam_occupied)
        fp = np.sum(~gt_occupied & slam_occupied)
        fn = np.sum(gt_occupied & ~slam_occupied)
        tn = np.sum(~gt_occupied & ~slam_occupied)

        # Precision and Recall
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1_score = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

        return {
            'rmse': float(rmse),
            'accuracy': float(accuracy),
            'precision': float(precision),
            'recall': float(recall),
            'f1_score': float(f1_score),
            'num_cells_compared': int(np.sum(both_known)),
            'true_positives': int(tp),
            'false_positives': int(fp),
            'false_negatives': int(fn),
            'true_negatives': int(tn),
        }

    def evaluate_callback(self):
        """Periodic evaluation of map accuracy."""
        if self.current_slam_map is None:
            return

        if self.evaluation_count >= self.num_evals:
            self.get_logger().info('Evaluation complete. Saving results...')
            self.save_results()
            self.eval_timer.cancel()
            return

        try:
            # Align maps
            gt_aligned, slam_aligned = self.align_maps(self.current_slam_map)

            # Compute metrics
            metrics = self.compute_metrics(gt_aligned, slam_aligned)

            if metrics:
                metrics['timestamp'] = self.get_clock().now().nanoseconds / 1e9
                metrics['evaluation_num'] = self.evaluation_count
                self.evaluation_results.append(metrics)

                self.get_logger().info(
                    f'Eval {self.evaluation_count}: '
                    f'RMSE={metrics["rmse"]:.4f}, '
                    f'Accuracy={metrics["accuracy"]*100:.2f}%, '
                    f'F1={metrics["f1_score"]:.4f}'
                )

                self.evaluation_count += 1

        except Exception as e:
            self.get_logger().error(f'Evaluation failed: {str(e)}')

    def save_results(self):
        """Save benchmark results to file."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'benchmark_{timestamp}.json'
        filepath = os.path.join(self.output_dir, filename)

        results = {
            'timestamp': datetime.now().isoformat(),
            'ground_truth_file': self.gt_map_file,
            'evaluations': self.evaluation_results,
        }

        # Compute summary statistics
        if self.evaluation_results:
            rmse_values = [e['rmse'] for e in self.evaluation_results]
            acc_values = [e['accuracy'] for e in self.evaluation_results]
            f1_values = [e['f1_score'] for e in self.evaluation_results]

            results['summary'] = {
                'mean_rmse': float(np.mean(rmse_values)),
                'std_rmse': float(np.std(rmse_values)),
                'final_rmse': float(rmse_values[-1]),
                'mean_accuracy': float(np.mean(acc_values)),
                'final_accuracy': float(acc_values[-1]),
                'mean_f1': float(np.mean(f1_values)),
                'final_f1': float(f1_values[-1]),
            }

            self.get_logger().info('=' * 60)
            self.get_logger().info('BENCHMARK SUMMARY')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f"Mean RMSE: {results['summary']['mean_rmse']:.6f} m")
            self.get_logger().info(f"Final RMSE: {results['summary']['final_rmse']:.6f} m")
            self.get_logger().info(f"Mean Accuracy: {results['summary']['mean_accuracy']*100:.2f}%")
            self.get_logger().info(f"Final Accuracy: {results['summary']['final_accuracy']*100:.2f}%")
            self.get_logger().info(f"Mean F1 Score: {results['summary']['mean_f1']:.4f}")
            self.get_logger().info('=' * 60)

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        self.get_logger().info(f'Benchmark results saved to {filepath}')


def main(args=None):
    rclpy.init(args=args)

    node = MapBenchmark()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
