"""
Drone with visible loitering and coverage tracking
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Set
from enum import Enum, auto

from ..core.geometry import Point, normalize_angle


class DroneState(Enum):
    IDLE = auto()
    FLYING = auto()
    LOITERING = auto()    # Visible circular loiter
    RETURNING = auto()
    LANDED = auto()


@dataclass
class SimpleDrone:
    """
    Drone with visible loitering and coverage tracking
    """
    position: Point = field(default_factory=lambda: Point(50, 50))
    heading: float = 0.0
    speed: float = 30.0
    
    # Navigation
    waypoints: List[Point] = field(default_factory=list)
    loiter_indices: Set[int] = field(default_factory=set)  # Which waypoints trigger loiter
    current_waypoint_idx: int = 0
    waypoint_radius: float = 20.0
    
    # Loiter state
    loiter_angle: float = 0.0          # Current angle in loiter circle
    loiter_radius: float = 60.0        # Loiter circle radius
    loiter_center: Optional[Point] = None
    loiter_revolutions: float = 0.0    # How many circles completed
    loiter_target_revs: float = 1.0    # Do 1 full circle
    
    # Physical
    turn_rate: float = 2.5
    
    # Energy
    battery: float = 100.0
    energy_rate: float = 0.12
    
    # State
    state: DroneState = DroneState.IDLE
    distance_traveled: float = 0.0
    path_history: List[Point] = field(default_factory=list)
    
    # Coverage - points that have been scanned
    coverage_points: List[Point] = field(default_factory=list)
    
    def set_mission(
        self, 
        path: List[Point], 
        loiter_indices: Optional[List[int]] = None
    ) -> None:
        """Set mission with loiter points"""
        self.waypoints = path
        self.loiter_indices = set(loiter_indices or [])
        self.current_waypoint_idx = 0
        self.state = DroneState.FLYING
        self.path_history = [Point(self.position.x, self.position.y)]
        self.coverage_points = []
    
    def update(self, dt: float) -> bool:
        """Update drone - returns True if waypoint reached"""
        if self.state in (DroneState.IDLE, DroneState.LANDED):
            return False
        
        # Handle loitering
        if self.state == DroneState.LOITERING:
            return self._update_loiter(dt)
        
        # Check mission complete
        if self.current_waypoint_idx >= len(self.waypoints):
            self.state = DroneState.LANDED
            return False
        
        target = self.waypoints[self.current_waypoint_idx]
        
        # Turn toward target
        target_heading = self.position.heading_to(target)
        heading_error = normalize_angle(target_heading - self.heading)
        
        max_turn = self.turn_rate * dt
        if abs(heading_error) > max_turn:
            self.heading += max_turn if heading_error > 0 else -max_turn
        else:
            self.heading = target_heading
        self.heading = normalize_angle(self.heading)
        
        # Move forward
        distance = self.speed * dt
        self.position = Point(
            self.position.x + distance * math.cos(self.heading),
            self.position.y + distance * math.sin(self.heading)
        )
        
        self.distance_traveled += distance
        self.battery -= self.energy_rate * dt
        
        # Record path and coverage
        if len(self.path_history) == 0 or self.position.distance_to(self.path_history[-1]) > 5:
            self.path_history.append(Point(self.position.x, self.position.y))
        
        # Add coverage points along path
        self._add_coverage(self.position, 40)
        
        # Check waypoint reached
        if self.position.distance_to(target) < self.waypoint_radius:
            # Check if this is a loiter point
            if self.current_waypoint_idx in self.loiter_indices:
                self._start_loiter(target)
            else:
                self.current_waypoint_idx += 1
            
            # Check if returning
            if self.current_waypoint_idx >= len(self.waypoints) - 3:
                if self.state != DroneState.LOITERING:
                    self.state = DroneState.RETURNING
            
            return True
        
        return False
    
    def _start_loiter(self, center: Point) -> None:
        """Start a loiter pattern at this point"""
        self.state = DroneState.LOITERING
        self.loiter_center = center
        self.loiter_angle = self.heading - math.pi / 2  # Start tangent to entry
        self.loiter_revolutions = 0.0
    
    def _update_loiter(self, dt: float) -> bool:
        """Update loiter circle motion"""
        if self.loiter_center is None:
            self.state = DroneState.FLYING
            return False
        
        # Angular speed (rad/s) based on linear speed and radius
        angular_speed = self.speed / self.loiter_radius
        
        # Update angle
        self.loiter_angle += angular_speed * dt
        self.loiter_revolutions += angular_speed * dt / (2 * math.pi)
        
        # Calculate position on circle
        self.position = Point(
            self.loiter_center.x + self.loiter_radius * math.cos(self.loiter_angle),
            self.loiter_center.y + self.loiter_radius * math.sin(self.loiter_angle)
        )
        
        # Heading is tangent to circle
        self.heading = normalize_angle(self.loiter_angle + math.pi / 2)
        
        self.distance_traveled += self.speed * dt
        self.battery -= self.energy_rate * dt
        
        # Record path
        if len(self.path_history) == 0 or self.position.distance_to(self.path_history[-1]) > 3:
            self.path_history.append(Point(self.position.x, self.position.y))
        
        # Add coverage in loiter area (larger radius during loiter)
        self._add_coverage(self.loiter_center, self.loiter_radius + 30)
        
        # Check if loiter complete
        if self.loiter_revolutions >= self.loiter_target_revs:
            self.state = DroneState.FLYING
            self.loiter_center = None
            self.current_waypoint_idx += 1
            return True
        
        return False
    
    def _add_coverage(self, center: Point, radius: float) -> None:
        """Add coverage points around a center"""
        # Add some random points within radius to show coverage
        for _ in range(3):
            angle = self.heading + (hash(str(self.distance_traveled)) % 100) / 50.0 - 1.0
            dist = radius * 0.5
            pt = Point(
                center.x + dist * math.cos(angle),
                center.y + dist * math.sin(angle)
            )
            self.coverage_points.append(pt)
    
    @property
    def is_loitering(self) -> bool:
        return self.state == DroneState.LOITERING
    
    @property
    def progress(self) -> float:
        if not self.waypoints:
            return 0.0
        return 100.0 * self.current_waypoint_idx / len(self.waypoints)
    
    def get_status(self) -> dict:
        return {
            'position': (self.position.x, self.position.y),
            'heading': math.degrees(self.heading),
            'speed': self.speed,
            'battery': self.battery,
            'state': self.state.name,
            'distance': self.distance_traveled,
            'progress': self.progress,
            'waypoint': f"{self.current_waypoint_idx + 1}/{len(self.waypoints)}",
            'loiter_center': self.loiter_center,
            'coverage_count': len(self.coverage_points)
        }
