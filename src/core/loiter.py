"""
Loiter Pattern Definitions for Project SUPARNA
The heart of Swift-inspired surveillance - no hover, only loiter
"""

import math
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

from .geometry import Point, generate_circle_points, rotate_point


class LoiterType(Enum):
    """Types of loiter patterns - each optimized for different ISR needs"""
    TIGHT = auto()      # 50-100m radius - target focus, detailed surveillance
    STANDARD = auto()   # 100-200m radius - general area surveillance
    WIDE = auto()       # 200-500m radius - broad area scan, patrol
    RACETRACK = auto()  # Variable - linear feature surveillance (roads, borders)


# Default radius ranges for each loiter type (in meters)
LOITER_RADIUS_RANGES = {
    LoiterType.TIGHT: (50, 100),
    LoiterType.STANDARD: (100, 200),
    LoiterType.WIDE: (200, 500),
    LoiterType.RACETRACK: (100, 300),  # Semi-circle radius
}

# Energy cost multipliers (lower = more efficient)
LOITER_ENERGY_COST = {
    LoiterType.TIGHT: 1.3,      # Tighter turns cost more
    LoiterType.STANDARD: 1.0,   # Baseline
    LoiterType.WIDE: 0.8,       # Most efficient
    LoiterType.RACETRACK: 0.9,  # Efficient for linear areas
}


@dataclass
class Loiter:
    """
    A loiter pattern - the fundamental unit of surveillance in SUPARNA
    
    Unlike traditional waypoints, a Loiter represents a circular or racetrack
    pattern that the drone flies to provide continuous coverage of an area.
    """
    center: Point
    radius: float
    loiter_type: LoiterType
    entry_heading: float = 0.0      # Radians, direction drone enters the loiter
    exit_heading: float = 0.0       # Radians, direction drone exits the loiter
    revolutions: float = 0.5        # Half circle = quick surveillance pass
    clockwise: bool = True          # Direction of loiter
    altitude: float = 100.0         # Meters AGL
    
    # For racetrack patterns
    racetrack_length: Optional[float] = None  # Length of straight segments
    racetrack_heading: Optional[float] = None  # Orientation of racetrack
    
    # Computed properties
    _waypoints: List[Point] = field(default_factory=list, repr=False)
    
    def __post_init__(self):
        """Generate waypoints after initialization"""
        self._generate_waypoints()
    
    @property
    def circumference(self) -> float:
        """Total path length for one revolution"""
        if self.loiter_type == LoiterType.RACETRACK and self.racetrack_length:
            return 2 * math.pi * self.radius + 2 * self.racetrack_length
        return 2 * math.pi * self.radius
    
    @property
    def total_distance(self) -> float:
        """Total distance for all revolutions"""
        return self.circumference * self.revolutions
    
    @property
    def coverage_area(self) -> float:
        """
        Approximate area covered by sensor during loiter
        Assumes sensor has same footprint as loiter radius
        """
        if self.loiter_type == LoiterType.RACETRACK and self.racetrack_length:
            # Rectangle + two semicircles
            return (self.racetrack_length * 2 * self.radius + 
                    math.pi * self.radius**2)
        return math.pi * self.radius**2
    
    @property
    def energy_cost(self) -> float:
        """
        Estimated energy cost (arbitrary units)
        Based on distance and loiter type efficiency
        """
        base_cost = self.total_distance
        multiplier = LOITER_ENERGY_COST[self.loiter_type]
        return base_cost * multiplier
    
    @property
    def waypoints(self) -> List[Point]:
        """Get waypoints defining this loiter pattern"""
        return self._waypoints
    
    def _generate_waypoints(self, points_per_revolution: int = 8) -> None:
        """Generate waypoints for the loiter pattern (8 points = quick pass)"""
        if self.loiter_type == LoiterType.RACETRACK:
            self._generate_racetrack_waypoints(points_per_revolution)
        else:
            self._generate_circular_waypoints(points_per_revolution)
    
    def _generate_circular_waypoints(self, points_per_rev: int) -> None:
        """Generate waypoints for circular loiter"""
        total_points = int(points_per_rev * self.revolutions)
        direction = 1 if self.clockwise else -1
        
        self._waypoints = []
        for i in range(total_points + 1):
            angle = self.entry_heading + direction * 2 * math.pi * i / points_per_rev
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)
            self._waypoints.append(Point(x, y))
    
    def _generate_racetrack_waypoints(self, points_per_turn: int) -> None:
        """Generate waypoints for racetrack loiter pattern"""
        if not self.racetrack_length or not self.racetrack_heading:
            # Fall back to circular if not properly configured
            self._generate_circular_waypoints(points_per_turn)
            return
        
        self._waypoints = []
        half_points = points_per_turn // 2
        heading = self.racetrack_heading
        
        # Calculate the two center points for the semicircles
        offset_x = (self.racetrack_length / 2) * math.cos(heading)
        offset_y = (self.racetrack_length / 2) * math.sin(heading)
        
        center1 = Point(self.center.x - offset_x, self.center.y - offset_y)
        center2 = Point(self.center.x + offset_x, self.center.y + offset_y)
        
        for rev in range(int(self.revolutions)):
            # First semicircle
            for i in range(half_points + 1):
                angle = heading + math.pi/2 + math.pi * i / half_points
                x = center1.x + self.radius * math.cos(angle)
                y = center1.y + self.radius * math.sin(angle)
                self._waypoints.append(Point(x, y))
            
            # Second semicircle
            for i in range(half_points + 1):
                angle = heading - math.pi/2 + math.pi * i / half_points
                x = center2.x + self.radius * math.cos(angle)
                y = center2.y + self.radius * math.sin(angle)
                self._waypoints.append(Point(x, y))
    
    def get_entry_point(self) -> Point:
        """Get the point where drone enters the loiter"""
        x = self.center.x + self.radius * math.cos(self.entry_heading)
        y = self.center.y + self.radius * math.sin(self.entry_heading)
        return Point(x, y)
    
    def get_exit_point(self) -> Point:
        """Get the point where drone exits the loiter"""
        # Exit after completing revolutions
        exit_angle = self.entry_heading + (1 if self.clockwise else -1) * 2 * math.pi * self.revolutions
        x = self.center.x + self.radius * math.cos(exit_angle)
        y = self.center.y + self.radius * math.sin(exit_angle)
        return Point(x, y)
    
    def contains_point(self, point: Point) -> bool:
        """Check if a point is within the loiter coverage area"""
        if self.loiter_type == LoiterType.RACETRACK and self.racetrack_length:
            # Simplified: check if point is within the racetrack bounds
            # (This is an approximation)
            return point.distance_to(self.center) <= self.radius + self.racetrack_length/2
        return point.distance_to(self.center) <= self.radius
    
    def get_coverage_polygon(self, num_points: int = 36) -> List[Point]:
        """Get polygon representing the coverage footprint"""
        return generate_circle_points(self.center, self.radius, num_points)


def create_loiter(
    center: Point,
    loiter_type: LoiterType,
    radius: Optional[float] = None,
    **kwargs
) -> Loiter:
    """
    Factory function to create a loiter with sensible defaults
    
    Args:
        center: Center point of the loiter
        loiter_type: Type of loiter pattern
        radius: Optional radius (uses default for type if not specified)
        **kwargs: Additional arguments passed to Loiter constructor
    
    Returns:
        Configured Loiter instance
    """
    if radius is None:
        # Use middle of the range for this type
        min_r, max_r = LOITER_RADIUS_RANGES[loiter_type]
        radius = (min_r + max_r) / 2
    
    return Loiter(
        center=center,
        radius=radius,
        loiter_type=loiter_type,
        **kwargs
    )


def estimate_loiter_for_area(
    area_size: float,
    sensor_fov: float = 60.0,
    altitude: float = 100.0
) -> Tuple[LoiterType, float]:
    """
    Estimate optimal loiter type and radius for a given area
    
    Args:
        area_size: Area to cover in square meters
        sensor_fov: Sensor field of view in degrees
        altitude: Flight altitude in meters
    
    Returns:
        Tuple of (recommended LoiterType, recommended radius)
    """
    # Sensor footprint at given altitude
    fov_rad = math.radians(sensor_fov)
    sensor_width = 2 * altitude * math.tan(fov_rad / 2)
    
    # Optimal radius is roughly where sensor can see the center
    optimal_radius = sensor_width / 2
    
    # Match to loiter type
    for ltype, (min_r, max_r) in LOITER_RADIUS_RANGES.items():
        if min_r <= optimal_radius <= max_r:
            return ltype, optimal_radius
    
    # Default to standard if outside ranges
    if optimal_radius < 50:
        return LoiterType.TIGHT, 50
    elif optimal_radius > 500:
        return LoiterType.WIDE, 500
    else:
        return LoiterType.STANDARD, optimal_radius
