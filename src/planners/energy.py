"""
SUPARNA — Energy Budget Manager
Per-loiter and per-transition energy tracking with real-time budget monitoring.

Tracks Wh consumed for every phase of the mission:
  - Climb to altitude
  - Transit between loiter zones (Dubins curve transitions)
  - Loiter observation patterns
  - Loiter-to-land spiral descent
  - Reserve margin (22% battery for RTB)
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

from ..core.atmosphere import (
    compute_performance, FlightPerformance,
    BATTERY_CAPACITY_WH, BASELINE_CRUISE_SPEED, BASELINE_POWER_DRAW,
)
from ..core.geometry import Point


RESERVE_FRACTION = 0.22  # 22% battery reserve for RTB
CLIMB_POWER_FACTOR = 1.8  # Climbing uses ~1.8× cruise power
CLIMB_RATE_MS = 3.0        # m/s vertical climb rate


@dataclass
class PhaseEnergy:
    """Energy consumed during a single mission phase."""
    phase_name: str
    phase_type: str        # 'climb', 'transit', 'loiter', 'descent', 'rtb'
    distance_m: float
    duration_s: float
    energy_wh: float
    start_wh: float        # Battery level at phase start
    end_wh: float          # Battery level at phase end
    loiter_index: int = -1  # Which loiter zone (-1 for transit)


@dataclass
class EnergyBudget:
    """Complete mission energy breakdown."""
    battery_capacity_wh: float = BATTERY_CAPACITY_WH
    reserve_wh: float = 0.0
    usable_wh: float = 0.0
    phases: List[PhaseEnergy] = field(default_factory=list)

    def __post_init__(self):
        self.reserve_wh = self.battery_capacity_wh * RESERVE_FRACTION
        self.usable_wh = self.battery_capacity_wh - self.reserve_wh

    @property
    def total_energy_wh(self) -> float:
        return sum(p.energy_wh for p in self.phases)

    @property
    def remaining_wh(self) -> float:
        return self.battery_capacity_wh - self.total_energy_wh

    @property
    def remaining_pct(self) -> float:
        return (self.remaining_wh / self.battery_capacity_wh) * 100

    @property
    def total_distance_m(self) -> float:
        return sum(p.distance_m for p in self.phases)

    @property
    def total_duration_s(self) -> float:
        return sum(p.duration_s for p in self.phases)

    @property
    def total_duration_min(self) -> float:
        return self.total_duration_s / 60

    def energy_by_type(self) -> dict:
        """Breakdown of energy by phase type."""
        breakdown = {}
        for p in self.phases:
            if p.phase_type not in breakdown:
                breakdown[p.phase_type] = 0.0
            breakdown[p.phase_type] += p.energy_wh
        return breakdown

    def is_within_budget(self) -> bool:
        return self.remaining_wh >= self.reserve_wh

    def to_dict(self) -> dict:
        return {
            'battery_capacity_wh': self.battery_capacity_wh,
            'reserve_wh': round(self.reserve_wh, 1),
            'usable_wh': round(self.usable_wh, 1),
            'total_energy_wh': round(self.total_energy_wh, 1),
            'remaining_wh': round(self.remaining_wh, 1),
            'remaining_pct': round(self.remaining_pct, 1),
            'total_distance_m': round(self.total_distance_m, 0),
            'total_duration_min': round(self.total_duration_min, 1),
            'energy_by_type': {k: round(v, 1) for k, v in self.energy_by_type().items()},
            'phases': [
                {
                    'name': p.phase_name,
                    'type': p.phase_type,
                    'distance_m': round(p.distance_m, 1),
                    'duration_s': round(p.duration_s, 1),
                    'energy_wh': round(p.energy_wh, 1),
                }
                for p in self.phases
            ],
        }


class EnergyManager:
    """
    Real-time energy budget manager for SUPARNA missions.

    Computes energy for each mission phase using ISA-corrected performance.
    Tracks a running battery level and warns if reserve is breached.
    """

    def __init__(self, altitude_m: float = 0.0, battery_wh: float = BATTERY_CAPACITY_WH):
        self.altitude_m = altitude_m
        self.perf = compute_performance(altitude_m)
        self.budget = EnergyBudget(battery_capacity_wh=battery_wh)
        self._current_wh = battery_wh

    def _consume(self, phase: PhaseEnergy):
        """Record energy consumption for a phase."""
        phase.start_wh = self._current_wh
        self._current_wh -= phase.energy_wh
        phase.end_wh = self._current_wh
        self.budget.phases.append(phase)

    def add_climb(self, target_altitude_m: float) -> PhaseEnergy:
        """
        Energy for climbing from ground to operating altitude.
        """
        climb_height = max(0, target_altitude_m)
        duration = climb_height / CLIMB_RATE_MS
        power = self.perf.power_draw_w * CLIMB_POWER_FACTOR
        energy = power * duration / 3600  # Wh
        # Horizontal distance during climb (climb at cruise speed)
        distance = self.perf.cruise_speed_ms * duration

        phase = PhaseEnergy(
            phase_name='Climb to altitude',
            phase_type='climb',
            distance_m=distance,
            duration_s=duration,
            energy_wh=energy,
            start_wh=0, end_wh=0,
        )
        self._consume(phase)
        return phase

    def add_transit(self, from_pos: Point, to_pos: Point, loiter_idx: int = -1) -> PhaseEnergy:
        """
        Energy for transit between two points (Dubins curve transition).
        """
        distance = from_pos.distance_to(to_pos)
        duration = distance / self.perf.cruise_speed_ms
        energy = self.perf.power_draw_w * duration / 3600

        phase = PhaseEnergy(
            phase_name=f'Transit → Loiter {loiter_idx + 1}' if loiter_idx >= 0 else 'Transit',
            phase_type='transit',
            distance_m=distance,
            duration_s=duration,
            energy_wh=energy,
            start_wh=0, end_wh=0,
            loiter_index=loiter_idx,
        )
        self._consume(phase)
        return phase

    def add_loiter(self, radius: float, revolutions: float = 1.0, loiter_idx: int = 0) -> PhaseEnergy:
        """
        Energy for a loiter observation pattern.
        """
        circumference = 2 * math.pi * radius
        distance = circumference * revolutions
        duration = distance / self.perf.cruise_speed_ms
        # Loiter uses slightly less power than cruise (no climb, steady bank)
        loiter_power = self.perf.power_draw_w * 0.92
        energy = loiter_power * duration / 3600

        phase = PhaseEnergy(
            phase_name=f'Loiter {loiter_idx + 1}',
            phase_type='loiter',
            distance_m=distance,
            duration_s=duration,
            energy_wh=energy,
            start_wh=0, end_wh=0,
            loiter_index=loiter_idx,
        )
        self._consume(phase)
        return phase

    def add_descent(self, from_altitude_m: float, loiter_radius: float) -> PhaseEnergy:
        """
        Energy for loiter-to-land spiral descent.
        """
        descent_rate = self.perf.descent_rate_m_per_loop
        n_loops = math.ceil(from_altitude_m / descent_rate)
        circumference = 2 * math.pi * loiter_radius
        distance = circumference * n_loops
        duration = distance / (self.perf.cruise_speed_ms * 0.85)  # Slower during descent
        # Descent uses less power (descending, reduced throttle)
        descent_power = self.perf.power_draw_w * 0.6
        energy = descent_power * duration / 3600

        phase = PhaseEnergy(
            phase_name=f'Loiter-to-Land ({n_loops} loops)',
            phase_type='descent',
            distance_m=distance,
            duration_s=duration,
            energy_wh=energy,
            start_wh=0, end_wh=0,
        )
        self._consume(phase)
        return phase

    def add_rtb(self, from_pos: Point, home_pos: Point) -> PhaseEnergy:
        """
        Energy for return-to-base transit.
        """
        distance = from_pos.distance_to(home_pos)
        duration = distance / self.perf.cruise_speed_ms
        energy = self.perf.power_draw_w * duration / 3600

        phase = PhaseEnergy(
            phase_name='Return to Base',
            phase_type='rtb',
            distance_m=distance,
            duration_s=duration,
            energy_wh=energy,
            start_wh=0, end_wh=0,
        )
        self._consume(phase)
        return phase

    def get_budget(self) -> EnergyBudget:
        return self.budget

    def can_afford_loiter(self, radius: float, revolutions: float = 1.0) -> bool:
        """Check if we have enough energy for one more loiter + RTB reserve."""
        circumference = 2 * math.pi * radius
        distance = circumference * revolutions
        duration = distance / self.perf.cruise_speed_ms
        energy = self.perf.power_draw_w * 0.92 * duration / 3600
        return (self._current_wh - energy) >= self.budget.reserve_wh
