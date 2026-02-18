"""
SUPARNA — ISA Atmosphere Model
International Standard Atmosphere corrections for high-altitude operations.

At 4,000m AMSL (Ladakh), air density drops to ~0.819 kg/m³ vs 1.225 at sea level.
This affects cruise speed, power draw, loiter radius, and descent rates.
"""

import math
from dataclasses import dataclass


# ISA Sea Level Constants
SEA_LEVEL_DENSITY = 1.225       # kg/m³
SEA_LEVEL_TEMP = 288.15         # K (15°C)
SEA_LEVEL_PRESSURE = 101325.0   # Pa
TEMP_LAPSE_RATE = 0.0065        # K/m (troposphere)
GRAVITY = 9.80665               # m/s²
GAS_CONSTANT = 287.058          # J/(kg·K) for dry air


@dataclass
class AtmosphereState:
    """Atmospheric conditions at a given altitude."""
    altitude_m: float           # meters AMSL
    density: float              # kg/m³
    temperature: float          # K
    pressure: float             # Pa
    density_ratio: float        # σ = ρ/ρ₀ (sigma)

    @property
    def temperature_celsius(self) -> float:
        return self.temperature - 273.15


def isa_at_altitude(altitude_m: float) -> AtmosphereState:
    """
    Compute ISA atmospheric properties at a given altitude.

    Uses the barometric formula for the troposphere (valid to ~11,000m):
        T = T₀ - L × h
        P = P₀ × (T/T₀)^(g/(L×R))
        ρ = P / (R × T)

    Args:
        altitude_m: Altitude in meters above mean sea level

    Returns:
        AtmosphereState with density, temperature, pressure
    """
    altitude_m = max(0, min(altitude_m, 11000))  # Clamp to troposphere

    T = SEA_LEVEL_TEMP - TEMP_LAPSE_RATE * altitude_m
    exponent = GRAVITY / (TEMP_LAPSE_RATE * GAS_CONSTANT)
    P = SEA_LEVEL_PRESSURE * (T / SEA_LEVEL_TEMP) ** exponent
    rho = P / (GAS_CONSTANT * T)

    return AtmosphereState(
        altitude_m=altitude_m,
        density=rho,
        temperature=T,
        pressure=P,
        density_ratio=rho / SEA_LEVEL_DENSITY,
    )


@dataclass
class FlightPerformance:
    """Adjusted flight performance at a given altitude."""
    altitude_m: float
    cruise_speed_ms: float      # m/s — adjusted for density
    power_draw_w: float         # watts — adjusted for density
    loiter_radius_m: float      # meters — minimum safe loiter radius
    stall_speed_ms: float       # m/s — adjusted stall speed
    descent_rate_m_per_loop: float  # m per loiter-to-land loop


# SUPARNA baseline performance (sea level)
BASELINE_CRUISE_SPEED = 19.0    # m/s
BASELINE_POWER_DRAW = 133.0     # watts
BASELINE_LOITER_RADIUS = 55.0   # meters
BASELINE_STALL_SPEED = 12.0     # m/s
BASELINE_BANK_ANGLE = 35.0      # degrees
BATTERY_CAPACITY_WH = 370.0     # watt-hours (6S4P Li-Ion)


def compute_performance(altitude_m: float) -> FlightPerformance:
    """
    Compute altitude-adjusted flight performance.

    Key relationships:
        - Cruise speed scales as 1/√σ (need more speed in thin air)
        - Power scales as V³ × ρ (cube-speed law × density)
        - Loiter radius = V² / (g × tan(bank))
        - Stall speed scales as 1/√σ

    Args:
        altitude_m: Altitude in meters AMSL

    Returns:
        FlightPerformance with adjusted parameters
    """
    atm = isa_at_altitude(altitude_m)
    sigma = atm.density_ratio

    # Speed increases with altitude (1/√σ)
    cruise_speed = BASELINE_CRUISE_SPEED / math.sqrt(sigma)
    stall_speed = BASELINE_STALL_SPEED / math.sqrt(sigma)

    # Power: P ∝ D × V = (½ρV²CdA) × V = ½ρV³CdA
    # Ratio: P_alt / P_0 = σ × (V_alt/V_0)³ = σ × σ^(-3/2) = σ^(-1/2)
    power_draw = BASELINE_POWER_DRAW / math.sqrt(sigma)

    # Loiter radius: R = V² / (g × tan(bank))
    bank_rad = math.radians(BASELINE_BANK_ANGLE)
    loiter_radius = cruise_speed ** 2 / (GRAVITY * math.tan(bank_rad))

    # Descent rate per loop (loiter-to-land): ~4-6m per loop
    # At altitude, larger radius = longer loop = more time to descend
    loop_circumference = 2 * math.pi * loiter_radius
    loop_time = loop_circumference / cruise_speed
    descent_rate = 3.0 + (altitude_m / 2000.0)  # Increases slightly with altitude

    return FlightPerformance(
        altitude_m=altitude_m,
        cruise_speed_ms=round(cruise_speed, 1),
        power_draw_w=round(power_draw, 1),
        loiter_radius_m=round(loiter_radius, 1),
        stall_speed_ms=round(stall_speed, 1),
        descent_rate_m_per_loop=round(descent_rate, 1),
    )


def compute_endurance(altitude_m: float, solar_boost_w: float = 0.0) -> dict:
    """
    Compute mission endurance at a given altitude.

    Args:
        altitude_m: Operating altitude in meters AMSL
        solar_boost_w: Average power from solar panel (0 for battery-only)

    Returns:
        Dictionary with endurance metrics
    """
    perf = compute_performance(altitude_m)
    net_power = perf.power_draw_w - solar_boost_w

    endurance_hours = BATTERY_CAPACITY_WH / net_power
    endurance_minutes = endurance_hours * 60

    coverage_per_loiter = math.pi * perf.loiter_radius_m ** 2
    # Estimate loiters per mission (60% of time in loiter, 40% transit)
    loiter_time_fraction = 0.6
    loiter_time_per = (2 * math.pi * perf.loiter_radius_m) / perf.cruise_speed_ms / 60  # minutes
    n_loiters = int(endurance_minutes * loiter_time_fraction / loiter_time_per)
    total_coverage_km2 = n_loiters * coverage_per_loiter / 1e6

    return {
        'altitude_m': altitude_m,
        'endurance_hours': round(endurance_hours, 2),
        'endurance_minutes': round(endurance_minutes, 1),
        'power_draw_w': perf.power_draw_w,
        'cruise_speed_ms': perf.cruise_speed_ms,
        'loiter_radius_m': perf.loiter_radius_m,
        'coverage_per_loiter_m2': round(coverage_per_loiter, 0),
        'estimated_loiters': n_loiters,
        'total_coverage_km2': round(total_coverage_km2, 2),
        'battery_capacity_wh': BATTERY_CAPACITY_WH,
        'solar_boost_w': solar_boost_w,
    }


# Pre-computed performance table for common altitudes
PERFORMANCE_TABLE = {
    0: compute_performance(0),
    1000: compute_performance(1000),
    2000: compute_performance(2000),
    2500: compute_performance(2500),
    3000: compute_performance(3000),
    3500: compute_performance(3500),
    4000: compute_performance(4000),
    4500: compute_performance(4500),
    5000: compute_performance(5000),
}
