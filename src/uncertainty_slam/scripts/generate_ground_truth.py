#!/usr/bin/env python3
"""
Ground Truth Map Generator

Generates a ground truth occupancy grid from the Stage world file
for benchmarking purposes.

Author: Rohan Upendra Patil
"""

import numpy as np
import json
import argparse
import os


def generate_test_environment_ground_truth(output_file, resolution=0.05):
    """
    Generate ground truth for test_environment.world.

    Args:
        output_file: Path to save ground truth JSON
        resolution: Map resolution in meters/cell
    """

    # World bounds from test_environment.world
    world_size_x = 12.0  # meters
    world_size_y = 12.0  # meters

    # Calculate map dimensions
    width = int(world_size_x / resolution)
    height = int(world_size_y / resolution)

    # Initialize map (0 = free, 100 = occupied, -1 = unknown)
    ground_truth = np.zeros((height, width), dtype=np.int8)

    # Origin at (-6, -6) to center the map
    origin_x = -6.0
    origin_y = -6.0

    def world_to_grid(x, y):
        """Convert world coordinates to grid indices."""
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        return grid_x, grid_y

    def add_rectangle(center_x, center_y, size_x, size_y, value=100):
        """Add a rectangular obstacle to the map."""
        x1 = center_x - size_x / 2
        x2 = center_x + size_x / 2
        y1 = center_y - size_y / 2
        y2 = center_y + size_y / 2

        gx1, gy1 = world_to_grid(x1, y1)
        gx2, gy2 = world_to_grid(x2, y2)

        # Ensure bounds
        gx1 = max(0, gx1)
        gy1 = max(0, gy1)
        gx2 = min(width - 1, gx2)
        gy2 = min(height - 1, gy2)

        ground_truth[gy1:gy2+1, gx1:gx2+1] = value

    # Add walls (from test_environment.world)
    add_rectangle(0, 6, 12, 0.2)    # North wall
    add_rectangle(0, -6, 12, 0.2)   # South wall
    add_rectangle(6, 0, 0.2, 12)    # East wall
    add_rectangle(-6, 0, 0.2, 12)   # West wall

    # Add obstacles
    add_rectangle(2, 2, 1, 1)       # Red block
    add_rectangle(-2, 3, 1, 1)      # Blue block
    add_rectangle(3, -3, 1, 1)      # Green block
    add_rectangle(-3, -2, 1, 1)     # Yellow block
    add_rectangle(0, 0, 0.5, 2)     # Orange central obstacle

    # Flatten for JSON
    data = ground_truth.flatten().tolist()

    # Create ground truth structure
    gt_data = {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': {
            'x': origin_x,
            'y': origin_y,
            'z': 0.0
        },
        'data': data,
        'description': 'Ground truth for test_environment.world',
        'occupied_threshold': 50
    }

    # Save to file
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)

    with open(output_file, 'w') as f:
        json.dump(gt_data, f, indent=2)

    print(f'Ground truth map generated: {width}x{height} cells')
    print(f'Resolution: {resolution} m/cell')
    print(f'Saved to: {output_file}')

    # Statistics
    occupied_cells = np.sum(ground_truth == 100)
    free_cells = np.sum(ground_truth == 0)
    print(f'Occupied cells: {occupied_cells} ({occupied_cells/(width*height)*100:.1f}%)')
    print(f'Free cells: {free_cells} ({free_cells/(width*height)*100:.1f}%)')


def main():
    parser = argparse.ArgumentParser(description='Generate ground truth map for benchmarking')
    parser.add_argument('--output', '-o', type=str, default='./ground_truth/test_environment_gt.json',
                        help='Output JSON file path')
    parser.add_argument('--resolution', '-r', type=float, default=0.05,
                        help='Map resolution in meters/cell')

    args = parser.parse_args()

    generate_test_environment_ground_truth(args.output, args.resolution)


if __name__ == '__main__':
    main()
