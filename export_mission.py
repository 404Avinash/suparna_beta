"""
SUPARNA Mission Exporter
Generates a random map, runs the planner, and exports mission.json for the 3D visualizer.

Usage:
    python export_mission.py              # Random map
    python export_mission.py --seed 42    # Reproducible map
"""

import json
import sys
import math
import argparse

from src.core.geometry import Point
from src.core.loiter import LoiterType
from src.core.random_map import generate_random_map
from src.planners.coverage import CoveragePlanner


def export_mission(seed=None):
    print("=" * 55)
    print("  SUPARNA - Mission Planner & Exporter")
    print("=" * 55)

    # Generate random map
    print(f"\n  Generating random map (seed={seed})...")
    smap = generate_random_map(
        width=1000.0,
        height=700.0,
        resolution=10.0,
        seed=seed,
    )
    print(f"  Map: {smap.width}x{smap.height}m, {len(smap.obstacles)} obstacles")
    print(f"  Home: ({smap.start_position.x:.0f}, {smap.start_position.y:.0f})")

    # Run coverage planner
    print("\n  Running Greedy Set Cover planner...")
    planner = CoveragePlanner(
        surveillance_map=smap,
        loiter_type=LoiterType.STANDARD,
        loiter_radius=70.0,
        turn_radius=40.0,
        overlap_factor=0.25,
        coverage_threshold=98.0,
        max_loiters=50,
    )
    mission = planner.plan(start_position=smap.start_position)
    print(f"  Planned {len(mission.loiters)} loiters")

    # Build waypoint sequence with obstacle avoidance info
    waypoints = []
    home = smap.start_position

    # Start at home
    waypoints.append({
        'x': home.x, 'y': home.y, 'type': 'home'
    })

    # Loiter centers
    for i, loiter in enumerate(mission.loiters):
        waypoints.append({
            'x': loiter.center.x,
            'y': loiter.center.y,
            'type': 'loiter',
            'radius': loiter.radius,
            'index': i + 1,
        })

    # Return home
    waypoints.append({
        'x': home.x, 'y': home.y, 'type': 'return'
    })

    # Build JSON output
    data = {
        'map': {
            'width': smap.width,
            'height': smap.height,
            'resolution': smap.resolution,
        },
        'home': {'x': home.x, 'y': home.y},
        'obstacles': [
            {
                'x': obs.center.x,
                'y': obs.center.y,
                'radius': obs.radius,
                'name': obs.name,
                'is_no_fly': obs.is_no_fly,
            }
            for obs in smap.obstacles
        ],
        'loiters': [
            {
                'x': l.center.x,
                'y': l.center.y,
                'radius': l.radius,
                'type': l.loiter_type.name,
                'index': i + 1,
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

    # Write JSON
    out_path = 'web/mission.json'
    import os
    os.makedirs('web', exist_ok=True)

    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"\n  Exported to {out_path}")
    print(f"  Stats: {len(mission.loiters)} loiters, "
          f"{mission.total_distance:.0f}m distance")
    print("  Done!")

    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SUPARNA Mission Exporter')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    args = parser.parse_args()
    export_mission(seed=args.seed)
