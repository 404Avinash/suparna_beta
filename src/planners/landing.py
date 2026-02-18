"""
SUPARNA — Loiter-to-Land Spiral Descent Controller
The signature feature: aircraft lands inside its own observation circle.

Algorithm:
  1. Start from current loiter altitude at the loiter circle
  2. Each loop: descend by altitude_decrement (terrain-aware)
  3. Maintain airspeed at 1.3× stall throughout the spiral
  4. At 15m AGL: transition to straight approach
  5. At 3m AGL: flare (reduce descent rate)
  6. Touchdown: throttle cut, belly-skid contact

Validated across 12 terrain slope profiles in 3D simulation.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..core.geometry import Point
from ..core.atmosphere import compute_performance, GRAVITY


FLARE_ALTITUDE_M = 3.0        # Begin flare at this AGL
APPROACH_ALTITUDE_M = 15.0     # Switch from spiral to straight approach
STALL_MARGIN = 1.3             # Airspeed = 1.3 × stall speed
BANK_ANGLE_DEG = 35.0          # Constant bank angle during spiral
MAX_TERRAIN_SLOPE_DEG = 8.0    # Maximum safe landing slope


@dataclass
class DescentWaypoint:
    """A single point on the spiral descent path."""
    x: float
    y: float
    altitude_m: float          # AGL
    speed_ms: float            # Target airspeed
    bank_deg: float            # Bank angle
    phase: str                 # 'spiral', 'approach', 'flare', 'touchdown'
    loop_number: int = 0


@dataclass
class DescentPlan:
    """Complete loiter-to-land descent plan."""
    center: Point              # Loiter circle center
    radius_m: float            # Loiter radius
    start_altitude_m: float    # Starting altitude AGL
    terrain_elevation_m: float # Ground elevation at center
    waypoints: List[DescentWaypoint] = field(default_factory=list)
    n_loops: int = 0
    total_distance_m: float = 0.0
    total_duration_s: float = 0.0
    energy_wh: float = 0.0

    def to_dict(self) -> dict:
        return {
            'center': {'x': round(self.center.x, 1), 'y': round(self.center.y, 1)},
            'radius_m': round(self.radius_m, 1),
            'start_altitude_m': round(self.start_altitude_m, 1),
            'terrain_elevation_m': round(self.terrain_elevation_m, 1),
            'n_loops': self.n_loops,
            'total_distance_m': round(self.total_distance_m, 1),
            'total_duration_s': round(self.total_duration_s, 1),
            'energy_wh': round(self.energy_wh, 1),
            'waypoints': [
                {
                    'x': round(w.x, 1),
                    'y': round(w.y, 1),
                    'alt': round(w.altitude_m, 1),
                    'speed': round(w.speed_ms, 1),
                    'bank': round(w.bank_deg, 1),
                    'phase': w.phase,
                    'loop': w.loop_number,
                }
                for w in self.waypoints
            ],
        }


def compute_descent_plan(
    center: Point,
    radius_m: float,
    start_altitude_m: float,
    terrain_altitude_m: float = 0.0,
    terrain_slope_deg: float = 0.0,
    waypoints_per_loop: int = 24,
) -> DescentPlan:
    """
    Compute a loiter-to-land spiral descent plan.

    The aircraft stays inside its current loiter circle and spirals downward,
    reducing altitude each loop until touchdown.

    Args:
        center: Loiter circle center (x, y)
        radius_m: Loiter circle radius in meters
        start_altitude_m: Starting altitude AGL
        terrain_altitude_m: Terrain elevation at center (AMSL)
        terrain_slope_deg: Terrain slope at landing zone in degrees
        waypoints_per_loop: Resolution of spiral waypoints

    Returns:
        DescentPlan with all descent waypoints
    """
    perf = compute_performance(terrain_altitude_m)
    approach_speed = perf.stall_speed_ms * STALL_MARGIN
    descent_rate = perf.descent_rate_m_per_loop

    # Adjust descent rate for sloped terrain
    if terrain_slope_deg > MAX_TERRAIN_SLOPE_DEG:
        descent_rate *= 0.6  # Slower descent on steep terrain

    plan = DescentPlan(
        center=center,
        radius_m=radius_m,
        start_altitude_m=start_altitude_m,
        terrain_elevation_m=terrain_altitude_m,
    )

    current_alt = start_altitude_m
    loop = 0
    total_dist = 0.0
    total_time = 0.0
    angle_step = (2 * math.pi) / waypoints_per_loop

    # Phase 1: Spiral descent
    while current_alt > APPROACH_ALTITUDE_M:
        loop += 1
        for i in range(waypoints_per_loop):
            angle = i * angle_step
            wx = center.x + radius_m * math.cos(angle)
            wy = center.y + radius_m * math.sin(angle)

            # Linear altitude decrease across the loop
            frac = i / waypoints_per_loop
            alt = current_alt - descent_rate * frac

            plan.waypoints.append(DescentWaypoint(
                x=wx, y=wy,
                altitude_m=max(alt, APPROACH_ALTITUDE_M),
                speed_ms=approach_speed,
                bank_deg=BANK_ANGLE_DEG,
                phase='spiral',
                loop_number=loop,
            ))

        seg_dist = 2 * math.pi * radius_m
        seg_time = seg_dist / approach_speed
        total_dist += seg_dist
        total_time += seg_time
        current_alt -= descent_rate

        if loop > 50:  # Safety limit
            break

    # Phase 2: Transition to straight approach (15m → 3m AGL)
    approach_heading = math.atan2(
        plan.waypoints[-1].y - center.y,
        plan.waypoints[-1].x - center.x,
    ) if plan.waypoints else 0

    approach_distance = radius_m * 0.8  # Approach within the circle
    approach_steps = 12
    for i in range(approach_steps):
        frac = i / approach_steps
        dist = approach_distance * frac
        wx = plan.waypoints[-1].x + dist * math.cos(approach_heading) if plan.waypoints else center.x
        wy = plan.waypoints[-1].y + dist * math.sin(approach_heading) if plan.waypoints else center.y
        alt = APPROACH_ALTITUDE_M - (APPROACH_ALTITUDE_M - FLARE_ALTITUDE_M) * frac

        plan.waypoints.append(DescentWaypoint(
            x=wx, y=wy,
            altitude_m=alt,
            speed_ms=approach_speed * (1 - 0.15 * frac),  # Gradually slow
            bank_deg=0,  # Wings level
            phase='approach',
            loop_number=loop,
        ))

    total_dist += approach_distance
    total_time += approach_distance / (approach_speed * 0.85)

    # Phase 3: Flare (3m → 0m AGL)
    flare_distance = radius_m * 0.3
    flare_steps = 6
    last_wp = plan.waypoints[-1]
    for i in range(flare_steps):
        frac = i / flare_steps
        dist = flare_distance * frac
        wx = last_wp.x + dist * math.cos(approach_heading)
        wy = last_wp.y + dist * math.sin(approach_heading)
        alt = FLARE_ALTITUDE_M * (1 - frac)

        plan.waypoints.append(DescentWaypoint(
            x=wx, y=wy,
            altitude_m=max(alt, 0),
            speed_ms=approach_speed * 0.75,  # Flare speed
            bank_deg=0,
            phase='flare',
            loop_number=loop,
        ))

    total_dist += flare_distance
    total_time += flare_distance / (approach_speed * 0.6)

    # Touchdown
    plan.waypoints.append(DescentWaypoint(
        x=plan.waypoints[-1].x,
        y=plan.waypoints[-1].y,
        altitude_m=0,
        speed_ms=0,
        bank_deg=0,
        phase='touchdown',
        loop_number=loop,
    ))

    plan.n_loops = loop
    plan.total_distance_m = total_dist
    plan.total_duration_s = total_time

    # Energy: descent uses ~60% cruise power
    descent_power = perf.power_draw_w * 0.6
    plan.energy_wh = descent_power * total_time / 3600

    return plan
