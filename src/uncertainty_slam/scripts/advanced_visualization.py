#!/usr/bin/env python3
"""
Advanced Visualization Tools for Uncertainty-Aware SLAM

Features:
- Real-time entropy heat map plotting
- 3D entropy surface visualization
- Time-series entropy analysis
- Comparative plots for multiple experiments
- Video generation from ROS bag files
- Publication-quality figure export

Author: Rohan Upendra Patil
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import json
import argparse
from pathlib import Path
from typing import List, Dict, Optional
import subprocess
import sys


class EntropyVisualizer:
    """Advanced visualization for entropy maps and metrics."""

    def __init__(self, output_dir: str = "results/visualizations"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Publication style
        plt.style.use('seaborn-v0_8-paper')
        plt.rcParams['figure.figsize'] = (10, 6)
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.labelsize'] = 14
        plt.rcParams['axes.titlesize'] = 16
        plt.rcParams['xtick.labelsize'] = 11
        plt.rcParams['ytick.labelsize'] = 11
        plt.rcParams['legend.fontsize'] = 12
        plt.rcParams['figure.titlesize'] = 18

    def plot_entropy_heatmap(self, entropy_map: np.ndarray,
                            title: str = "Entropy Heat Map",
                            save_name: str = "entropy_heatmap.png"):
        """
        Create 2D heat map visualization of entropy.

        Args:
            entropy_map: 2D numpy array of entropy values (0-100)
            title: Plot title
            save_name: Output filename
        """
        fig, ax = plt.subplots(figsize=(12, 10))

        # Create heat map
        im = ax.imshow(entropy_map, cmap='hot', interpolation='nearest',
                      vmin=0, vmax=100, origin='lower')

        # Add colorbar
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Entropy (0-100)', rotation=270, labelpad=20)

        # Labels
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')
        ax.set_title(title)

        # Grid
        ax.grid(False)

        # Save
        output_path = self.output_dir / save_name
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved heat map to {output_path}")

        plt.close()

    def plot_entropy_surface_3d(self, entropy_map: np.ndarray,
                               title: str = "3D Entropy Surface",
                               save_name: str = "entropy_3d.png"):
        """
        Create 3D surface plot of entropy.

        Args:
            entropy_map: 2D numpy array of entropy values
            title: Plot title
            save_name: Output filename
        """
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Create coordinate grids
        height, width = entropy_map.shape
        x = np.arange(0, width, 1)
        y = np.arange(0, height, 1)
        X, Y = np.meshgrid(x, y)

        # Create surface
        surf = ax.plot_surface(X, Y, entropy_map, cmap='hot',
                              linewidth=0, antialiased=True, alpha=0.8)

        # Add colorbar
        cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
        cbar.set_label('Entropy', rotation=270, labelpad=20)

        # Labels
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')
        ax.set_zlabel('Entropy')
        ax.set_title(title)

        # View angle
        ax.view_init(elev=30, azim=45)

        # Save
        output_path = self.output_dir / save_name
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved 3D surface to {output_path}")

        plt.close()

    def plot_entropy_timeseries(self, ecs_data: Dict,
                               title: str = "Entropy Over Time",
                               save_name: str = "entropy_timeseries.png"):
        """
        Plot entropy metrics over time from ECS log data.

        Args:
            ecs_data: Dictionary loaded from ecs_report.json
            title: Plot title
            save_name: Output filename
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))

        # Extract data
        avg_entropy_data = ecs_data.get('avg_entropy_data', [])
        max_entropy_data = ecs_data.get('max_entropy_data', [])

        if not avg_entropy_data:
            print("No entropy data found in ECS report")
            return

        times = [t for t, _ in avg_entropy_data]
        avg_entropies = [e for _, e in avg_entropy_data]
        max_entropies = [e for _, e in max_entropy_data] if max_entropy_data else []

        # Plot 1: Average Entropy
        axes[0].plot(times, avg_entropies, 'b-', linewidth=2, label='Average Entropy')
        axes[0].set_ylabel('Average Entropy')
        axes[0].set_title('Map Average Entropy')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        # Plot 2: Maximum Entropy
        if max_entropies:
            axes[1].plot(times, max_entropies, 'r-', linewidth=2, label='Maximum Entropy')
            axes[1].set_ylabel('Maximum Entropy')
            axes[1].set_title('Map Maximum Entropy')
            axes[1].grid(True, alpha=0.3)
            axes[1].legend()

        # Plot 3: Entropy Convergence (exponentially weighted)
        if len(times) > 1:
            T = times[-1]
            alpha = 2.0
            weights = [np.exp(-alpha * t / T) for t in times]
            weighted_entropy = [e * w for e, w in zip(avg_entropies, weights)]

            axes[2].plot(times, weighted_entropy, 'g-', linewidth=2, label='Weighted Entropy')
            axes[2].fill_between(times, 0, weighted_entropy, alpha=0.3, color='g')
            axes[2].set_ylabel('Weighted Entropy')
            axes[2].set_xlabel('Time (s)')
            axes[2].set_title(f'Time-Weighted Entropy (ECS = {ecs_data.get("ecs_score", {}).get("ecs", 0):.3f})')
            axes[2].grid(True, alpha=0.3)
            axes[2].legend()

        plt.tight_layout()

        # Save
        output_path = self.output_dir / save_name
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved time-series plot to {output_path}")

        plt.close()

    def plot_experiment_comparison(self, experiments: List[Dict],
                                  labels: List[str],
                                  title: str = "Experiment Comparison",
                                  save_name: str = "experiment_comparison.png"):
        """
        Compare multiple experiments on same plot.

        Args:
            experiments: List of ECS data dictionaries
            labels: Labels for each experiment
            title: Plot title
            save_name: Output filename
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        colors = plt.cm.tab10(np.linspace(0, 1, len(experiments)))

        for exp, label, color in zip(experiments, labels, colors):
            avg_data = exp.get('avg_entropy_data', [])
            if not avg_data:
                continue

            times = [t for t, _ in avg_data]
            entropies = [e for _, e in avg_data]

            # Plot 1: Average entropy over time
            axes[0, 0].plot(times, entropies, label=label, color=color, linewidth=2)

            # Plot 2: Entropy reduction rate
            if len(entropies) > 10:
                # Compute derivative (rate of change)
                dt = np.diff(times)
                de = np.diff(entropies)
                rate = de / dt
                axes[0, 1].plot(times[1:], rate, label=label, color=color, linewidth=2, alpha=0.7)

        # Plot 3: ECS scores bar chart
        ecs_scores = [exp.get('ecs_score', {}).get('ecs', 0) for exp in experiments]
        axes[1, 0].bar(range(len(labels)), ecs_scores, color=colors, alpha=0.7)
        axes[1, 0].set_xticks(range(len(labels)))
        axes[1, 0].set_xticklabels(labels, rotation=45, ha='right')
        axes[1, 0].set_ylabel('ECS Score')
        axes[1, 0].set_title('Entropy Convergence Score Comparison')
        axes[1, 0].grid(True, alpha=0.3, axis='y')

        # Plot 4: Final entropy values
        final_entropies = []
        for exp in experiments:
            avg_data = exp.get('avg_entropy_data', [])
            if avg_data:
                final_entropies.append(avg_data[-1][1])
            else:
                final_entropies.append(0)

        axes[1, 1].bar(range(len(labels)), final_entropies, color=colors, alpha=0.7)
        axes[1, 1].set_xticks(range(len(labels)))
        axes[1, 1].set_xticklabels(labels, rotation=45, ha='right')
        axes[1, 1].set_ylabel('Final Entropy')
        axes[1, 1].set_title('Final Map Entropy')
        axes[1, 1].grid(True, alpha=0.3, axis='y')

        # Configure plot 1
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Average Entropy')
        axes[0, 0].set_title('Entropy Convergence Over Time')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # Configure plot 2
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Entropy Rate (dE/dt)')
        axes[0, 1].set_title('Entropy Reduction Rate')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        plt.suptitle(title, y=1.00)
        plt.tight_layout()

        # Save
        output_path = self.output_dir / save_name
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved comparison plot to {output_path}")

        plt.close()

    def create_entropy_animation(self, entropy_maps: List[np.ndarray],
                                times: List[float],
                                title: str = "Entropy Evolution",
                                save_name: str = "entropy_animation.mp4",
                                fps: int = 10):
        """
        Create animated video of entropy map evolution.

        Args:
            entropy_maps: List of 2D entropy arrays
            times: List of timestamps
            title: Animation title
            save_name: Output filename
            fps: Frames per second
        """
        if not entropy_maps:
            print("No entropy maps provided")
            return

        fig, ax = plt.subplots(figsize=(10, 8))

        # Initialize plot
        im = ax.imshow(entropy_maps[0], cmap='hot', vmin=0, vmax=100,
                      interpolation='nearest', origin='lower')
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Entropy (0-100)', rotation=270, labelpad=20)

        time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                           color='white', fontsize=14, weight='bold',
                           bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))

        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')
        ax.set_title(title)

        def update(frame):
            """Update function for animation."""
            im.set_data(entropy_maps[frame])
            time_text.set_text(f'Time: {times[frame]:.1f}s')
            return [im, time_text]

        # Create animation
        anim = animation.FuncAnimation(fig, update, frames=len(entropy_maps),
                                      interval=1000/fps, blit=True)

        # Save
        output_path = self.output_dir / save_name
        writer = animation.FFMpegWriter(fps=fps, bitrate=2000)

        try:
            anim.save(str(output_path), writer=writer)
            print(f"Saved animation to {output_path}")
        except Exception as e:
            print(f"Error saving animation: {e}")
            print("Make sure ffmpeg is installed: sudo apt-get install ffmpeg")

        plt.close()

    def generate_publication_figure(self, ecs_data: Dict,
                                   entropy_map: Optional[np.ndarray] = None,
                                   save_name: str = "publication_figure.png"):
        """
        Generate comprehensive publication-quality figure.

        Args:
            ecs_data: ECS report data
            entropy_map: Optional final entropy map
            save_name: Output filename
        """
        if entropy_map is not None:
            fig = plt.figure(figsize=(16, 12))
            gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        else:
            fig = plt.figure(figsize=(16, 8))
            gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)

        # Extract data
        avg_data = ecs_data.get('avg_entropy_data', [])
        max_data = ecs_data.get('max_entropy_data', [])

        if not avg_data:
            print("No data available")
            return

        times = [t for t, _ in avg_data]
        avg_entropies = [e for _, e in avg_data]
        max_entropies = [e for _, e in max_data] if max_data else []

        # Plot 1: Entropy time series
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(times, avg_entropies, 'b-', linewidth=2, label='Average')
        if max_entropies:
            ax1.plot(times, max_entropies, 'r--', linewidth=2, label='Maximum', alpha=0.7)
        ax1.fill_between(times, 0, avg_entropies, alpha=0.2, color='b')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Entropy')
        ax1.set_title('Entropy Evolution Over Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: ECS Score components
        ax2 = fig.add_subplot(gs[1, 0])
        ecs_score = ecs_data.get('ecs_score', {})
        metrics = {
            'ECS': ecs_score.get('ecs', 0),
            'AUC': ecs_score.get('auc', 0) / 100,  # Normalize
            'Final': ecs_score.get('final_entropy', 0) / 100,
            'Peak': ecs_score.get('peak_entropy', 0) / 100,
        }
        ax2.bar(metrics.keys(), metrics.values(), color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])
        ax2.set_ylabel('Normalized Score')
        ax2.set_title('Performance Metrics')
        ax2.grid(True, alpha=0.3, axis='y')

        # Plot 3: Entropy histogram
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.hist(avg_entropies, bins=30, color='blue', alpha=0.7, edgecolor='black')
        ax3.axvline(np.mean(avg_entropies), color='r', linestyle='--',
                   linewidth=2, label=f'Mean: {np.mean(avg_entropies):.2f}')
        ax3.set_xlabel('Entropy Value')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Entropy Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4 & 5: Entropy heat maps (if provided)
        if entropy_map is not None:
            # 2D heat map
            ax4 = fig.add_subplot(gs[2, 0])
            im4 = ax4.imshow(entropy_map, cmap='hot', vmin=0, vmax=100,
                            interpolation='nearest', origin='lower')
            plt.colorbar(im4, ax=ax4, fraction=0.046, pad=0.04)
            ax4.set_xlabel('X (cells)')
            ax4.set_ylabel('Y (cells)')
            ax4.set_title('Final Entropy Heat Map')

            # 3D surface
            ax5 = fig.add_subplot(gs[2, 1], projection='3d')
            height, width = entropy_map.shape
            x = np.arange(0, width, 1)
            y = np.arange(0, height, 1)
            X, Y = np.meshgrid(x, y)
            surf = ax5.plot_surface(X, Y, entropy_map, cmap='hot',
                                   linewidth=0, antialiased=True, alpha=0.8)
            ax5.set_xlabel('X')
            ax5.set_ylabel('Y')
            ax5.set_zlabel('Entropy')
            ax5.set_title('3D Entropy Surface')
            ax5.view_init(elev=30, azim=45)

        # Overall title
        exp_name = ecs_data.get('experiment_name', 'Unknown')
        fig.suptitle(f'Uncertainty-Aware SLAM Analysis: {exp_name}',
                    fontsize=20, weight='bold')

        # Save
        output_path = self.output_dir / save_name
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved publication figure to {output_path}")

        plt.close()


def main():
    """Command-line interface for visualization tools."""
    parser = argparse.ArgumentParser(description='Advanced Visualization for Uncertainty-Aware SLAM')

    parser.add_argument('--ecs-report', type=str,
                       help='Path to ECS report JSON file')
    parser.add_argument('--compare', nargs='+',
                       help='List of ECS report files to compare')
    parser.add_argument('--labels', nargs='+',
                       help='Labels for comparison experiments')
    parser.add_argument('--output-dir', type=str, default='results/visualizations',
                       help='Output directory for plots')
    parser.add_argument('--publication', action='store_true',
                       help='Generate publication-quality figure')

    args = parser.parse_args()

    viz = EntropyVisualizer(output_dir=args.output_dir)

    if args.compare:
        # Load all experiments
        experiments = []
        for report_file in args.compare:
            with open(report_file, 'r') as f:
                experiments.append(json.load(f))

        labels = args.labels if args.labels else [f"Exp {i+1}" for i in range(len(experiments))]

        viz.plot_experiment_comparison(experiments, labels)

    elif args.ecs_report:
        # Load single experiment
        with open(args.ecs_report, 'r') as f:
            ecs_data = json.load(f)

        viz.plot_entropy_timeseries(ecs_data)

        if args.publication:
            viz.generate_publication_figure(ecs_data)

    else:
        print("Please provide --ecs-report or --compare")
        parser.print_help()


if __name__ == '__main__':
    main()
