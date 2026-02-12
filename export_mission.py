"""
SUPARNA Mission Exporter
Generates a map, runs the planner, and exports mission.json for the 3D visualizer.

Usage:
    python export_mission.py                    # Random flat map
    python export_mission.py --seed 42          # Reproducible random map
    python export_mission.py --map lac          # LAC border terrain
    python export_mission.py --map lac --seed 7 # LAC with specific seed
"""

import json
import os
import sys
import math
import argparse
import numpy as np

from src.core.geometry import Point
from src.core.loiter import LoiterType
from src.core.random_map import generate_random_map
from src.core.lac_sector import generate_lac_sector
from src.planners.coverage import CoveragePlanner


def export_mission(seed=None, map_type='random'):
    print("=" * 55)
    print("  SUPARNA - Mission Planner & Exporter")
    print("=" * 55)

    heightmap = None
    landmarks = None

    if map_type == 'lac':
        # LAC border terrain
        print(f"\n  Loading LAC Sector (Ladakh border, seed={seed})...")
        smap, heightmap, landmarks = generate_lac_sector(seed=seed or 42)
        print(f"  Map: {smap.width}x{smap.height}m, {len(smap.obstacles)} features")
        print(f"  Home (Indian FOB): ({smap.start_position.x:.0f}, {smap.start_position.y:.0f})")
        print(f"  Landmarks: {', '.join(landmarks.keys())}")

        planner = CoveragePlanner(
            surveillance_map=smap,
            loiter_type=LoiterType.STANDARD,
            loiter_radius=200.0,    # Wider for larger area
            turn_radius=60.0,
            overlap_factor=0.25,
            coverage_threshold=95.0,
            max_loiters=50,
        )
    else:
        # Random flat map
        print(f"\n  Generating random map (seed={seed})...")
        smap = generate_random_map(
            width=1000.0, height=700.0, resolution=10.0, seed=seed,
        )
        print(f"  Map: {smap.width}x{smap.height}m, {len(smap.obstacles)} obstacles")
        print(f"  Home: ({smap.start_position.x:.0f}, {smap.start_position.y:.0f})")

        planner = CoveragePlanner(
            surveillance_map=smap,
            loiter_type=LoiterType.STANDARD,
            loiter_radius=70.0,
            turn_radius=40.0,
            overlap_factor=0.25,
            coverage_threshold=98.0,
            max_loiters=50,
        )

    # Run coverage planner
    print("\n  Running Greedy Set Cover planner...")
    mission = planner.plan(start_position=smap.start_position)
    print(f"  Planned {len(mission.loiters)} loiters")

    # Build waypoint sequence
    waypoints = []
    home = smap.start_position

    waypoints.append({'x': home.x, 'y': home.y, 'type': 'home'})
    for i, loiter in enumerate(mission.loiters):
        waypoints.append({
            'x': loiter.center.x, 'y': loiter.center.y,
            'type': 'loiter', 'radius': loiter.radius, 'index': i + 1,
        })
    waypoints.append({'x': home.x, 'y': home.y, 'type': 'return'})

    # Build JSON output
    data = {
        'map': {
            'width': smap.width, 'height': smap.height,
            'resolution': smap.resolution, 'type': map_type,
        },
        'home': {'x': home.x, 'y': home.y},
        'obstacles': [
            {
                'x': obs.center.x, 'y': obs.center.y,
                'radius': obs.radius, 'name': obs.name,
                'is_no_fly': obs.is_no_fly,
            }
            for obs in smap.obstacles
        ],
        'loiters': [
            {
                'x': l.center.x, 'y': l.center.y, 'radius': l.radius,
                'type': l.loiter_type.name, 'index': i + 1,
            }
            for i, l in enumerate(mission.loiters)
        ],
        'waypoints': waypoints,
        'stats': {
            'total_loiters': len(mission.loiters),
            'total_distance': mission.total_distance,
            'total_energy': mission.total_energy,
            'num_obstacles': len(smap.obstacles),
        },
    }

    # Add heightmap if LAC terrain
    if heightmap is not None:
        rows, cols = heightmap.shape
        # Downsample for JSON size (every 2nd cell)
        step = 2
        data['heightmap'] = {
            'rows': rows // step,
            'cols': cols // step,
            'step': step * smap.resolution,
            'min_elevation': float(np.min(heightmap)),
            'max_elevation': float(np.max(heightmap)),
            'data': [
                round(float(heightmap[r][c]), 1)
                for r in range(0, rows, step)
                for c in range(0, cols, step)
            ],
        }

    # Add landmarks if LAC terrain
    if landmarks is not None:
        data['landmarks'] = landmarks

    # Write JSON
    out_path = 'web/mission.json'
    os.makedirs('web', exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)

    size_kb = os.path.getsize(out_path) / 1024
    print(f"\n  Exported to {out_path} ({size_kb:.0f} KB)")
    print(f"  Stats: {len(mission.loiters)} loiters, "
          f"{mission.total_distance:.0f}m distance")
    if heightmap is not None:
        print(f"  Heightmap: {data['heightmap']['rows']}x{data['heightmap']['cols']} "
              f"({data['heightmap']['min_elevation']:.0f}-"
              f"{data['heightmap']['max_elevation']:.0f}m)")
    print("  Done!")

    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SUPARNA Mission Exporter')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    parser.add_argument('--map', type=str, default='random',
                        choices=['random', 'lac'],
                        help='Map type: random or lac (Ladakh border)')
    args = parser.parse_args()
    export_mission(seed=args.seed, map_type=args.map)
