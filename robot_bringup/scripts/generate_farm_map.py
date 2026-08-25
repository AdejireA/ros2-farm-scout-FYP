#!/usr/bin/env python3
"""
Generate a Nav2 occupancy grid map from known farm.sdf geometry.

This avoids SLAM entirely — the map is constructed directly from the
known positions of walls, crop rows, and obstacles as defined in
farm_world/worlds/farm.sdf.

Usage:
  python3 generate_farm_map.py [output_dir]
  Default output: ~/ros2_ws/src/nav2_config/maps/
"""

import os
import sys
import numpy as np
from PIL import Image

# Farm geometry from farm.sdf
WALL_X = 11.0
WALL_Y = 12.0
CROP_ROWS = [-8, -4, 0, 4, 8]
CROP_Y = [-9, -7, -5, -3, -1, 1, 3, 5, 7, 9]
OBSTACLES = [
    {'name': 'rock',  'x': -6, 'y': -4, 'sx': 1.8, 'sy': 1.5},
    {'name': 'crate', 'x':  6, 'y':  3, 'sx': 1.8, 'sy': 1.5},
    {'name': 'post',  'x':  0, 'y': -6, 'sx': 1.0, 'sy': 1.0},
]

RESOLUTION = 0.05  # m/pixel
CROP_SIZE = 0.6    # m, obstacle footprint per plant
X_MIN, X_MAX = -13.0, 13.0
Y_MIN, Y_MAX = -14.0, 14.0

def world_to_pixel(wx, wy, width, height):
    px = int((wx - X_MIN) / RESOLUTION)
    py = int((Y_MAX - wy) / RESOLUTION)
    return px, py

def draw_box(grid, cx, cy, sx, sy):
    height, width = grid.shape
    for wx in np.arange(cx - sx/2, cx + sx/2, RESOLUTION):
        for wy in np.arange(cy - sy/2, cy + sy/2, RESOLUTION):
            px, py = world_to_pixel(wx, wy, width, height)
            if 0 <= py < height and 0 <= px < width:
                grid[py, px] = 0

def draw_wall(grid, x1, y1, x2, y2, thickness=0.4):
    if x1 == x2:
        draw_box(grid, x1, (y1+y2)/2, thickness, abs(y2-y1)+thickness)
    else:
        draw_box(grid, (x1+x2)/2, y1, abs(x2-x1)+thickness, thickness)

def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/ros2_ws/src/nav2_config/maps')
    os.makedirs(outdir, exist_ok=True)

    width = int((X_MAX - X_MIN) / RESOLUTION)
    height = int((Y_MAX - Y_MIN) / RESOLUTION)
    grid = np.full((height, width), 205, dtype=np.uint8)

    # Walls
    draw_wall(grid, -WALL_X, WALL_Y, WALL_X, WALL_Y, 0.4)
    draw_wall(grid, -WALL_X, -WALL_Y, WALL_X, -WALL_Y, 0.4)
    draw_wall(grid, -WALL_X, -WALL_Y, -WALL_X, WALL_Y, 0.4)
    draw_wall(grid, WALL_X, -WALL_Y, WALL_X, WALL_Y, 0.4)

    # Crops
    for rx in CROP_ROWS:
        for cy in CROP_Y:
            draw_box(grid, rx, cy, CROP_SIZE, CROP_SIZE)

    # Obstacles
    for obs in OBSTACLES:
        draw_box(grid, obs['x'], obs['y'], obs['sx'], obs['sy'])

    # Save
    outpath = os.path.join(outdir, 'farm_map')
    img = Image.fromarray(grid)
    img.save(outpath + '.pgm')

    with open(outpath + '.yaml', 'w') as f:
        f.write(f"image: farm_map.pgm\n")
        f.write(f"mode: trinary\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{X_MIN}, {Y_MIN}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.25\n")

    print(f'Map generated: {width}x{height} pixels, {RESOLUTION}m/pix')
    print(f'Saved: {outpath}.pgm, {outpath}.yaml')

if __name__ == '__main__':
    main()
