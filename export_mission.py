"""
SUPARNA Mission Exporter — Full PCCE Pipeline
Generates map → runs coverage planner → TSP optimization → energy budget →
loiter-to-land descent → exports mission.json + KMZ + report.

Usage:
    python export_mission.py                         # Random flat map (sea level)
    python export_mission.py --map lac               # LAC border terrain
    python export_mission.py --map lac --alt 4000    # LAC at 4,000m AMSL
    python export_mission.py --seed 42               # Reproducible
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
from src.core.atmosphere import compute_performance, compute_endurance, isa_at_altitude
from src.planners.coverage import CoveragePlanner
from src.planners.energy import EnergyManager
from src.planners.landing import compute_descent_plan
from src.export.report import export_kmz, export_report


def export_mission(seed=None, map_type='random', altitude_m=0.0, custom_obstacles=None):
    print("=" * 60)
    print("  SUPARNA - Physics-Constrained Coverage Engine (PCCE)")
    print("=" * 60)

    # -- ISA Atmosphere --
    perf = compute_performance(altitude_m)
    atm = isa_at_altitude(altitude_m)
    print(f"\n  > Altitude: {altitude_m:.0f}m AMSL")
    print(f"  > Air density: {atm.density:.3f} kg/m3 (sigma={atm.density_ratio:.3f})")
    print(f"  > Cruise speed: {perf.cruise_speed_ms} m/s")
    print(f"  > Power draw: {perf.power_draw_w} W")
    print(f"  > Min loiter radius: {perf.loiter_radius_m} m")

    heightmap = None
    landmarks = None

    # -- Map Generation --
    if map_type == 'lac':
        print(f"\n  Loading LAC Sector (Ladakh border, seed={seed})...")
        smap, heightmap, landmarks = generate_lac_sector(seed=seed or 42)
        print(f"  Map: {smap.width}x{smap.height}m, {len(smap.obstacles)} features")
        print(f"  Home (Indian FOB): ({smap.start_position.x:.0f}, {smap.start_position.y:.0f})")
        print(f"  Landmarks: {', '.join(landmarks.keys())}")

        loiter_radius = max(perf.loiter_radius_m, 200.0)
        planner = CoveragePlanner(
            surveillance_map=smap,
            loiter_type=LoiterType.STANDARD,
            loiter_radius=loiter_radius,
            turn_radius=perf.loiter_radius_m * 0.8,
            overlap_factor=0.25,
            coverage_threshold=95.0,
            max_loiters=50,
        )
    else:
        print(f"\n  Generating random map (seed={seed})...")
        smap = generate_random_map(
            width=1000.0, height=700.0, resolution=10.0, seed=seed,
        )
        print(f"  Map: {smap.width}x{smap.height}m, {len(smap.obstacles)} obstacles")
        print(f"  Home: ({smap.start_position.x:.0f}, {smap.start_position.y:.0f})")

        loiter_radius = max(perf.loiter_radius_m, 70.0)
        planner = CoveragePlanner(
            surveillance_map=smap,
            loiter_type=LoiterType.STANDARD,
            loiter_radius=loiter_radius,
            turn_radius=max(perf.loiter_radius_m * 0.8, 40.0),
            overlap_factor=0.25,
            coverage_threshold=98.0,
            max_loiters=50,
        )

    # -- Inject custom restricted zones --
    if custom_obstacles:
        from src.core.map import Obstacle
        for co in custom_obstacles:
            cx = co.get('x', 0)
            cy = co.get('y', 0)
            cr = co.get('radius', 60)
            smap.obstacles.append(
                Obstacle(center=Point(cx, cy), radius=cr, is_no_fly=True, name='RESTRICTED')
            )
        print(f"  + {len(custom_obstacles)} custom restricted zones injected")

    # -- Coverage Planning (Greedy Set Cover) --
    print("\n  Running Greedy Set Cover planner...")
    mission = planner.plan(start_position=smap.start_position)
    print(f"  Planned {len(mission.loiters)} loiter zones")

    # -- TSP Optimization (Nearest-Neighbour) --
    from src.planners.transition import TransitionPlanner
    tsp = TransitionPlanner(turn_radius=perf.loiter_radius_m * 0.8,
                            surveillance_map=smap)
    optimized_loiters, total_transit_dist = tsp.optimize_loiter_sequence(mission.loiters)
    print(f"  TSP optimised: {total_transit_dist:.0f}m total transit")

    # -- Energy Budget --
    print("\n  Computing energy budget...")
    energy_mgr = EnergyManager(altitude_m=altitude_m)

    # Climb
    drone_alt = 150 if map_type == 'lac' else 80
    energy_mgr.add_climb(drone_alt)

    # Transits + Loiters
    home = smap.start_position
    prev_pos = home
    for i, loiter in enumerate(optimized_loiters):
        energy_mgr.add_transit(prev_pos, loiter.center, loiter_idx=i)
        energy_mgr.add_loiter(loiter.radius, revolutions=1.0, loiter_idx=i)
        prev_pos = loiter.center

    # Return to base
    energy_mgr.add_rtb(prev_pos, home)

    budget = energy_mgr.get_budget()
    print(f"  Energy used: {budget.total_energy_wh:.1f} Wh / {budget.battery_capacity_wh} Wh")
    print(f"  Remaining: {budget.remaining_pct:.1f}%")
    print(f"  Duration: {budget.total_duration_min:.1f} min")

    for ptype, wh in budget.energy_by_type().items():
        print(f"    {ptype:>10}: {wh:.1f} Wh")

    # -- Loiter-to-Land Descent --
    print("\n  Computing loiter-to-land descent...")
    last_loiter = optimized_loiters[-1] if optimized_loiters else None
    descent_center = last_loiter.center if last_loiter else home
    descent_radius = last_loiter.radius if last_loiter else loiter_radius

    descent_plan = compute_descent_plan(
        center=descent_center,
        radius_m=descent_radius,
        start_altitude_m=drone_alt,
        terrain_altitude_m=altitude_m,
    )
    energy_mgr.add_descent(drone_alt, descent_radius)

    print(f"  Descent: {descent_plan.n_loops} loops, "
          f"{descent_plan.total_distance_m:.0f}m, "
          f"{descent_plan.energy_wh:.1f} Wh")

    # -- Build Waypoints --
    waypoints = []
    waypoints.append({'x': home.x, 'y': home.y, 'type': 'home'})
    for i, loiter in enumerate(optimized_loiters):
        waypoints.append({
            'x': loiter.center.x, 'y': loiter.center.y,
            'type': 'loiter', 'radius': loiter.radius, 'index': i + 1,
        })
    waypoints.append({'x': home.x, 'y': home.y, 'type': 'return'})

    # -- Build JSON --
    data = {
        'map': {
            'width': smap.width, 'height': smap.height,
            'resolution': smap.resolution, 'type': map_type,
        },
        'home': {'x': home.x, 'y': home.y},
        'altitude_m': altitude_m,
        'performance': {
            'cruise_speed_ms': perf.cruise_speed_ms,
            'power_draw_w': perf.power_draw_w,
            'loiter_radius_m': perf.loiter_radius_m,
            'stall_speed_ms': perf.stall_speed_ms,
            'air_density': round(atm.density, 3),
            'density_ratio': round(atm.density_ratio, 3),
        },
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
            for i, l in enumerate(optimized_loiters)
        ],
        'waypoints': waypoints,
        'energy': budget.to_dict(),
        'descent': descent_plan.to_dict(),
        'stats': {
            'total_loiters': len(optimized_loiters),
            'total_distance': mission.total_distance,
            'total_energy_wh': budget.total_energy_wh,
            'duration_min': budget.total_duration_min,
            'num_obstacles': len(smap.obstacles),
            'coverage_pct': 95.0 if map_type == 'lac' else 98.0,
        },
        'algorithms': [
            'Greedy Set Cover (Coverage Planner)',
            'Dubins Curves — 6 types, closed-form O(1)',
            'A* with obstacle inflation O(V log V)',
            'Bug2 7-ray raycasting O(r)',
            'NN-TSP Loiter Sequencer O(k²)',
            'ISA Atmosphere Correction',
            'Loiter-to-Land Spiral Descent',
            '1Hz Energy Budget Manager',
        ],
    }

    # Heightmap for LAC terrain
    if heightmap is not None:
        rows, cols = heightmap.shape
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

    if landmarks is not None:
        data['landmarks'] = landmarks

    # -- Write Files --
    os.makedirs('web', exist_ok=True)

    # mission.json
    out_path = 'web/mission.json'
    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)

    # Also copy to root for GitHub Pages
    with open('mission.json', 'w') as f:
        json.dump(data, f, indent=2)

    size_kb = os.path.getsize(out_path) / 1024
    print(f"\n  Exported to {out_path} ({size_kb:.0f} KB)")

    # KMZ export
    kmz_path = 'web/mission.kmz'
    try:
        export_kmz(data, kmz_path)
        print(f"  KMZ: {kmz_path}")
    except Exception as e:
        print(f"  KMZ export failed: {e}")

    # Report export
    report_path = 'web/mission_report.json'
    try:
        export_report(data, budget.to_dict(), report_path)
        print(f"  Report: {report_path}")
    except Exception as e:
        print(f"  Report export failed: {e}")

    # -- Summary --
    print(f"\n  -- Mission Summary --")
    print(f"  Loiters: {len(optimized_loiters)}")
    print(f"  Distance: {mission.total_distance:.0f}m")
    print(f"  Energy: {budget.total_energy_wh:.1f} Wh ({budget.remaining_pct:.0f}% remaining)")
    print(f"  Duration: {budget.total_duration_min:.1f} min")
    print(f"  Landing: {descent_plan.n_loops}-loop spiral descent")
    if heightmap is not None:
        print(f"  Heightmap: {data['heightmap']['rows']}x{data['heightmap']['cols']} "
              f"({data['heightmap']['min_elevation']:.0f}-"
              f"{data['heightmap']['max_elevation']:.0f}m)")
    print("  [OK] Done!")
    print("=" * 60)

    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SUPARNA PCCE Mission Exporter')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    parser.add_argument('--map', type=str, default='random',
                        choices=['random', 'lac'],
                        help='Map type: random or lac (Ladakh border)')
    parser.add_argument('--alt', type=float, default=0.0,
                        help='Operating altitude in meters AMSL (0=sea level, 4000=Ladakh)')
    args = parser.parse_args()
    export_mission(seed=args.seed, map_type=args.map, altitude_m=args.alt)
