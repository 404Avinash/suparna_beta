"""
Random map generator for Project SUPARNA
Generates randomized SurveillanceMap with obstacles and no-fly zones
"""

import random
import math
from typing import List, Optional
from ..core.geometry import Point
from ..core.map import SurveillanceMap, Obstacle


def generate_random_map(
    width: float = 1000.0,
    height: float = 700.0,
    resolution: float = 10.0,
    num_obstacles: int = 0,
    num_no_fly: int = 0,
    seed: Optional[int] = None,
) -> SurveillanceMap:
    """
    Generate a random surveillance map.

    Args:
        width: Map width in meters
        height: Map height in meters
        resolution: Grid resolution in meters
        num_obstacles: Number of obstacles (0 = random 3-8)
        num_no_fly: Number of no-fly zones (0 = random 0-2)
        seed: Random seed for reproducibility
    """
    if seed is not None:
        random.seed(seed)

    if num_obstacles == 0:
        num_obstacles = random.randint(3, 8)
    if num_no_fly == 0:
        num_no_fly = random.randint(0, 2)

    # Home position — always in the left region, clear of center
    home = Point(
        random.uniform(60, width * 0.15),
        random.uniform(height * 0.3, height * 0.7),
    )

    obstacles: List[Obstacle] = []

    # Generate physical obstacles (buildings, towers, etc.)
    names = ['building', 'tower', 'hill', 'structure', 'antenna', 'tree_cluster',
             'hangar', 'water_tower']
    for i in range(num_obstacles):
        for _attempt in range(50):
            radius = random.uniform(25, 70)
            cx = random.uniform(radius + 50, width - radius - 50)
            cy = random.uniform(radius + 50, height - radius - 50)
            center = Point(cx, cy)

            # Check not too close to home or other obstacles
            if center.distance_to(home) < radius + 120:
                continue
            too_close = False
            for obs in obstacles:
                if center.distance_to(obs.center) < radius + obs.radius + 60:
                    too_close = True
                    break
            if too_close:
                continue

            obstacles.append(Obstacle(
                center=center,
                radius=radius,
                name=f"{random.choice(names)}_{i+1}",
            ))
            break

    # Generate no-fly zones (larger, marked as no_fly)
    for i in range(num_no_fly):
        for _attempt in range(50):
            radius = random.uniform(40, 90)
            cx = random.uniform(radius + 80, width - radius - 80)
            cy = random.uniform(radius + 80, height - radius - 80)
            center = Point(cx, cy)

            if center.distance_to(home) < radius + 150:
                continue
            too_close = False
            for obs in obstacles:
                if center.distance_to(obs.center) < radius + obs.radius + 40:
                    too_close = True
                    break
            if too_close:
                continue

            obstacles.append(Obstacle(
                center=center,
                radius=radius,
                is_no_fly=True,
                name=f"no_fly_zone_{i+1}",
            ))
            break

    return SurveillanceMap(
        width=width,
        height=height,
        resolution=resolution,
        obstacles=obstacles,
        start_position=home,
        obstacle_margin=20.0,
        no_fly_margin=30.0,
    )
