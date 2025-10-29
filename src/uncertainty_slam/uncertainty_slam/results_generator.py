#!/usr/bin/env python3
"""
Results Generator for Uncertainty-Aware SLAM
Automatically generates visualizations and reports when exploration completes.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64, String
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from datetime import datetime
import os
import json

class ResultsGenerator(Node):
    """
    Monitors the system and generates visualizations when exploration completes.
    """

    def __init__(self):
        super().__init__('results_generator')

        # Parameters
        self.declare_parameter('output_dir', '~/slam_uncertainty_ws/results/visualizations')
        self.declare_parameter('auto_generate', True)

        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.auto_generate = self.get_parameter('auto_generate').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Data storage
        self.occupancy_map = None
        self.entropy_map = None
        self.entropy_history = []
        self.timestamps = []
        self.robot_mode = None
        self.exploration_status = None
        self.results_generated = False
        self.generation_timer = None  # For delayed generation
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.entropy_sub = self.create_subscription(
            OccupancyGrid, '/entropy_map', self.entropy_callback, 10
        )
        self.avg_entropy_sub = self.create_subscription(
            Float64, '/map_average_entropy', self.avg_entropy_callback, 10
        )
        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/exploration_status', self.status_callback, 10
        )
        
        # Timer for periodic checks
        self.create_timer(10.0, self.check_completion)
        
        self.get_logger().info('Results Generator initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')
    
    def map_callback(self, msg):
        """Store latest occupancy map."""
        self.occupancy_map = msg
    
    def entropy_callback(self, msg):
        """Store latest entropy map."""
        self.entropy_map = msg
    
    def avg_entropy_callback(self, msg):
        """Track entropy over time."""
        self.entropy_history.append(msg.data)
        self.timestamps.append(self.get_clock().now().nanoseconds / 1e9)
    
    def mode_callback(self, msg):
        """Track robot mode."""
        self.robot_mode = msg.data
    
    def status_callback(self, msg):
        """Track exploration status."""
        self.exploration_status = msg.data
        self.get_logger().info(f'Exploration status: {self.exploration_status}')

        # Trigger result generation when complete
        if self.exploration_status == 'COMPLETE' and not self.results_generated:
            self.get_logger().info('🎉 Exploration complete signal received!')
            self.get_logger().info('⏳ Waiting 30 seconds for SLAM to finalize map...')
            # Create a timer that will fire once after 30 seconds
            self.generation_timer = self.create_timer(30.0, self.trigger_generation)

    def trigger_generation(self):
        """Trigger result generation after delay (called by timer)."""
        if not self.results_generated:
            self.get_logger().info('📊 Generating final results...')
            self.generate_all_visualizations()
            self.results_generated = True

            # Cancel the timer after it fires once
            if self.generation_timer is not None:
                self.generation_timer.cancel()
                self.generation_timer = None
    
    def check_completion(self):
        """
        Periodic check - DISABLED FOR USER REQUIREMENTS.
        
        Results now ONLY generate when robot explicitly signals 'COMPLETE' via /exploration_status.
        This ensures visualizations are created AFTER 100% map exploration, NOT during.
        No fallback generation - waits for explicit signal only.
        """
        # Completely disabled - rely ONLY on explicit completion signal from robot
        pass
    
    def generate_all_visualizations(self):
        """Generate all visualization outputs."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        try:
            # 1. Occupancy Map
            if self.occupancy_map is not None:
                self.save_occupancy_map(timestamp)
            
            # 2. Entropy Heat-map
            if self.entropy_map is not None:
                self.save_entropy_map(timestamp)
            
            # 3. Entropy Over Time
            if len(self.entropy_history) > 0:
                self.save_entropy_plot(timestamp)
            
            # 4. Combined View
            if self.occupancy_map is not None and self.entropy_map is not None:
                self.save_combined_view(timestamp)
            
            # 5. Statistics Report
            self.save_statistics_report(timestamp)
            
            self.get_logger().info(f'✅ All visualizations saved to: {self.output_dir}')
            
        except Exception as e:
            self.get_logger().error(f'Error generating visualizations: {e}')
    
    def save_occupancy_map(self, timestamp):
        """Save occupancy grid as image."""
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        data = np.array(self.occupancy_map.data).reshape((height, width))
        
        plt.figure(figsize=(10, 10))
        plt.imshow(data, cmap='gray', origin='lower', vmin=-1, vmax=100)
        plt.colorbar(label='Occupancy Probability')
        plt.title('Occupancy Grid Map', fontsize=16, fontweight='bold')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.tight_layout()
        
        filename = os.path.join(self.output_dir, f'occupancy_map_{timestamp}.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Saved occupancy map: {filename}')
    
    def save_entropy_map(self, timestamp):
        """Save entropy heat-map as image."""
        width = self.entropy_map.info.width
        height = self.entropy_map.info.height
        data = np.array(self.entropy_map.data).reshape((height, width))
        
        plt.figure(figsize=(10, 10))
        plt.imshow(data, cmap='hot', origin='lower', vmin=0, vmax=100)
        plt.colorbar(label='Entropy (scaled 0-100)')
        plt.title('Uncertainty Heat-Map (Shannon Entropy)', fontsize=16, fontweight='bold')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.tight_layout()
        
        filename = os.path.join(self.output_dir, f'entropy_map_{timestamp}.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Saved entropy map: {filename}')
    
    def save_entropy_plot(self, timestamp):
        """Save entropy evolution over time."""
        if len(self.timestamps) == 0:
            return
        
        # Normalize timestamps to start at 0
        t_start = self.timestamps[0]
        times = [(t - t_start) for t in self.timestamps]
        
        plt.figure(figsize=(12, 6))
        plt.plot(times, self.entropy_history, 'b-', linewidth=2)
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Average Entropy (bits)', fontsize=12)
        plt.title('Entropy Convergence Over Time', fontsize=16, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        filename = os.path.join(self.output_dir, f'entropy_evolution_{timestamp}.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Saved entropy plot: {filename}')
    
    def save_combined_view(self, timestamp):
        """Save combined occupancy + entropy overlay."""
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        occ_data = np.array(self.occupancy_map.data).reshape((height, width))
        ent_data = np.array(self.entropy_map.data).reshape((height, width))
        
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        # Occupancy
        im1 = axes[0].imshow(occ_data, cmap='gray', origin='lower', vmin=-1, vmax=100)
        axes[0].set_title('Occupancy Grid', fontsize=14, fontweight='bold')
        axes[0].set_xlabel('X (cells)')
        axes[0].set_ylabel('Y (cells)')
        plt.colorbar(im1, ax=axes[0], label='Occupancy')
        
        # Entropy
        im2 = axes[1].imshow(ent_data, cmap='hot', origin='lower', vmin=0, vmax=100)
        axes[1].set_title('Entropy Heat-Map', fontsize=14, fontweight='bold')
        axes[1].set_xlabel('X (cells)')
        axes[1].set_ylabel('Y (cells)')
        plt.colorbar(im2, ax=axes[1], label='Entropy')
        
        # Overlay
        axes[2].imshow(occ_data, cmap='gray', origin='lower', vmin=-1, vmax=100, alpha=0.7)
        im3 = axes[2].imshow(ent_data, cmap='hot', origin='lower', vmin=0, vmax=100, alpha=0.5)
        axes[2].set_title('Combined View', fontsize=14, fontweight='bold')
        axes[2].set_xlabel('X (cells)')
        axes[2].set_ylabel('Y (cells)')
        plt.colorbar(im3, ax=axes[2], label='Entropy')
        
        plt.suptitle('Uncertainty-Aware SLAM Results', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        filename = os.path.join(self.output_dir, f'combined_view_{timestamp}.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Saved combined view: {filename}')
    
    def save_statistics_report(self, timestamp):
        """Save text report with statistics."""
        report = {
            'timestamp': timestamp,
            'total_samples': len(self.entropy_history),
            'duration_seconds': self.timestamps[-1] - self.timestamps[0] if self.timestamps else 0,
            'initial_entropy': float(self.entropy_history[0]) if self.entropy_history else 0,
            'final_entropy': float(self.entropy_history[-1]) if self.entropy_history else 0,
            'min_entropy': float(min(self.entropy_history)) if self.entropy_history else 0,
            'max_entropy': float(max(self.entropy_history)) if self.entropy_history else 0,
            'avg_entropy': float(np.mean(self.entropy_history)) if self.entropy_history else 0,
            'entropy_reduction': float(self.entropy_history[0] - self.entropy_history[-1]) if len(self.entropy_history) > 1 else 0,
        }
        
        # Save JSON
        json_filename = os.path.join(self.output_dir, f'statistics_{timestamp}.json')
        with open(json_filename, 'w') as f:
            json.dump(report, f, indent=2)
        
        # Save human-readable text
        txt_filename = os.path.join(self.output_dir, f'report_{timestamp}.txt')
        with open(txt_filename, 'w') as f:
            f.write('='*60 + '\n')
            f.write('UNCERTAINTY-AWARE SLAM - RESULTS REPORT\n')
            f.write('='*60 + '\n\n')
            f.write(f'Generated: {timestamp}\n\n')
            f.write('ENTROPY STATISTICS\n')
            f.write('-'*60 + '\n')
            f.write(f'Duration: {report["duration_seconds"]:.1f} seconds\n')
            f.write(f'Total Samples: {report["total_samples"]}\n')
            f.write(f'Initial Entropy: {report["initial_entropy"]:.4f} bits\n')
            f.write(f'Final Entropy: {report["final_entropy"]:.4f} bits\n')
            f.write(f'Min Entropy: {report["min_entropy"]:.4f} bits\n')
            f.write(f'Max Entropy: {report["max_entropy"]:.4f} bits\n')
            f.write(f'Average Entropy: {report["avg_entropy"]:.4f} bits\n')
            f.write(f'Entropy Reduction: {report["entropy_reduction"]:.4f} bits\n\n')
            
            if self.occupancy_map:
                f.write('MAP INFORMATION\n')
                f.write('-'*60 + '\n')
                f.write(f'Map Size: {self.occupancy_map.info.width} × {self.occupancy_map.info.height} cells\n')
                f.write(f'Resolution: {self.occupancy_map.info.resolution:.3f} m/cell\n')
                f.write(f'Physical Size: {self.occupancy_map.info.width * self.occupancy_map.info.resolution:.1f} × ')
                f.write(f'{self.occupancy_map.info.height * self.occupancy_map.info.resolution:.1f} m\n\n')
            
            f.write('='*60 + '\n')
            f.write('Visualizations saved in same directory\n')
            f.write('='*60 + '\n')
        
        self.get_logger().info(f'Saved report: {txt_filename}')


def main(args=None):
    rclpy.init(args=args)
    node = ResultsGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

