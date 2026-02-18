"""
Dubins Path Generation for Project SUPARNA
Optimal paths for fixed-wing aircraft with minimum turn radius

Dubins paths are the shortest paths between two configurations (position + heading)
for a vehicle that can only move forward and has a minimum turn radius.
Perfect for our Swift-inspired drone!
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum, auto
import numpy as np

from .geometry import Point, normalize_angle


class DubinsPathType(Enum):
    """Types of Dubins paths (combinations of Left, Straight, Right)"""
    LSL = auto()  # Left - Straight - Left
    LSR = auto()  # Left - Straight - Right
    RSL = auto()  # Right - Straight - Left
    RSR = auto()  # Right - Straight - Right
    RLR = auto()  # Right - Left - Right
    LRL = auto()  # Left - Right - Left


@dataclass
class DubinsPath:
    """
    A Dubins path connecting two configurations
    
    Consists of three segments:
    1. Initial turn (or straight)
    2. Middle segment (straight or turn)
    3. Final turn (or straight)
    """
    start: Point
    end: Point
    start_heading: float        # Radians
    end_heading: float          # Radians
    turn_radius: float          # Minimum turn radius
    path_type: DubinsPathType
    
    # Segment lengths
    segment1_length: float = 0.0
    segment2_length: float = 0.0
    segment3_length: float = 0.0
    
    # Precomputed waypoints
    _waypoints: List[Point] = None
    
    @property
    def total_length(self) -> float:
        """Total path length"""
        return self.segment1_length + self.segment2_length + self.segment3_length
    
    @property
    def waypoints(self) -> List[Point]:
        """Get waypoints along the path"""
        if self._waypoints is None:
            self._generate_waypoints()
        return self._waypoints
    
    def _generate_waypoints(self, step_size: float = 5.0) -> None:
        """Generate waypoints along the path"""
        self._waypoints = []
        
        # Parse path type
        segments = self.path_type.name  # e.g., "LSL"
        
        current_pos = self.start
        current_heading = self.start_heading
        
        lengths = [self.segment1_length, self.segment2_length, self.segment3_length]
        
        for i, (seg_type, length) in enumerate(zip(segments, lengths)):
            if length <= 0:
                continue
                
            if seg_type == 'S':
                # Straight segment
                waypoints, new_pos, new_heading = self._generate_straight_segment(
                    current_pos, current_heading, length, step_size
                )
            elif seg_type == 'L':
                # Left turn
                waypoints, new_pos, new_heading = self._generate_turn_segment(
                    current_pos, current_heading, length, step_size, left=True
                )
            else:  # 'R'
                # Right turn
                waypoints, new_pos, new_heading = self._generate_turn_segment(
                    current_pos, current_heading, length, step_size, left=False
                )
            
            self._waypoints.extend(waypoints)
            current_pos = new_pos
            current_heading = new_heading
        
        # Add final point
        self._waypoints.append(self.end)
    
    def _generate_straight_segment(
        self, 
        start: Point, 
        heading: float, 
        length: float,
        step_size: float
    ) -> Tuple[List[Point], Point, float]:
        """Generate waypoints for a straight segment"""
        waypoints = []
        steps = max(1, int(length / step_size))
        
        dx = math.cos(heading)
        dy = math.sin(heading)
        
        for i in range(steps):
            t = i / steps
            waypoints.append(Point(
                start.x + t * length * dx,
                start.y + t * length * dy
            ))
        
        end_point = Point(start.x + length * dx, start.y + length * dy)
        return waypoints, end_point, heading
    
    def _generate_turn_segment(
        self,
        start: Point,
        heading: float,
        arc_length: float,
        step_size: float,
        left: bool
    ) -> Tuple[List[Point], Point, float]:
        """Generate waypoints for a turn segment"""
        waypoints = []
        
        # Calculate turn center
        perpendicular = heading + (math.pi/2 if left else -math.pi/2)
        center = Point(
            start.x + self.turn_radius * math.cos(perpendicular),
            start.y + self.turn_radius * math.sin(perpendicular)
        )
        
        # Calculate arc angle
        arc_angle = arc_length / self.turn_radius
        if not left:
            arc_angle = -arc_angle
        
        # Generate waypoints along arc
        steps = max(1, int(arc_length / step_size))
        start_angle = math.atan2(start.y - center.y, start.x - center.x)
        
        for i in range(steps):
            t = i / steps
            angle = start_angle + t * arc_angle
            waypoints.append(Point(
                center.x + self.turn_radius * math.cos(angle),
                center.y + self.turn_radius * math.sin(angle)
            ))
        
        # Calculate end point and heading
        end_angle = start_angle + arc_angle
        end_point = Point(
            center.x + self.turn_radius * math.cos(end_angle),
            center.y + self.turn_radius * math.sin(end_angle)
        )
        end_heading = normalize_angle(heading + arc_angle)
        
        return waypoints, end_point, end_heading


def generate_dubins_path(
    start: Point,
    start_heading: float,
    end: Point,
    end_heading: float,
    turn_radius: float
) -> Optional[DubinsPath]:
    """
    Generate the shortest Dubins path between two configurations
    
    Args:
        start: Starting position
        start_heading: Starting heading in radians
        end: Ending position
        end_heading: Ending heading in radians
        turn_radius: Minimum turn radius
    
    Returns:
        DubinsPath object representing the shortest path, or None if no path exists
    """
    # Normalize to unit turn radius for calculations
    dx = end.x - start.x
    dy = end.y - start.y
    d = math.sqrt(dx*dx + dy*dy) / turn_radius
    
    # Angle from start to end
    theta = math.atan2(dy, dx)
    alpha = normalize_angle(start_heading - theta)
    beta = normalize_angle(end_heading - theta)
    
    # Try all path types and find shortest
    best_path = None
    best_length = float('inf')
    
    for path_type in DubinsPathType:
        result = _compute_dubins_segments(d, alpha, beta, path_type)
        if result is not None:
            t, p, q = result
            length = (t + p + q) * turn_radius
            
            if length < best_length:
                best_length = length
                best_path = DubinsPath(
                    start=start,
                    end=end,
                    start_heading=start_heading,
                    end_heading=end_heading,
                    turn_radius=turn_radius,
                    path_type=path_type,
                    segment1_length=t * turn_radius,
                    segment2_length=p * turn_radius,
                    segment3_length=q * turn_radius
                )
    
    return best_path


def _compute_dubins_segments(
    d: float, 
    alpha: float, 
    beta: float, 
    path_type: DubinsPathType
) -> Optional[Tuple[float, float, float]]:
    """
    Compute segment lengths for a specific Dubins path type
    
    Returns (t, p, q) segment lengths in units of turn radius, or None if invalid
    """
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    cab = math.cos(alpha - beta)
    
    if path_type == DubinsPathType.LSL:
        tmp = 2 + d*d - 2*cab + 2*d*(sa - sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(cb - ca, d + sa - sb)
        t = normalize_angle(-alpha + theta)
        q = normalize_angle(beta - theta)
        
    elif path_type == DubinsPathType.RSR:
        tmp = 2 + d*d - 2*cab + 2*d*(sb - sa)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(ca - cb, d - sa + sb)
        t = normalize_angle(alpha - theta)
        q = normalize_angle(-beta + theta)
        
    elif path_type == DubinsPathType.LSR:
        tmp = -2 + d*d + 2*cab + 2*d*(sa + sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2, p)
        t = normalize_angle(-alpha + theta)
        q = normalize_angle(-beta + theta)
        
    elif path_type == DubinsPathType.RSL:
        tmp = -2 + d*d + 2*cab - 2*d*(sa + sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2, p)
        t = normalize_angle(alpha - theta)
        q = normalize_angle(beta - theta)
        
    elif path_type == DubinsPathType.RLR:
        tmp = (6 - d*d + 2*cab + 2*d*(sa - sb)) / 8
        if abs(tmp) > 1:
            return None
        p = 2*math.pi - math.acos(tmp)
        theta = math.atan2(ca - cb, d - sa + sb)
        t = normalize_angle(alpha - theta + p/2)
        q = normalize_angle(alpha - beta - t + p)
        
    elif path_type == DubinsPathType.LRL:
        tmp = (6 - d*d + 2*cab + 2*d*(sb - sa)) / 8
        if abs(tmp) > 1:
            return None
        p = 2*math.pi - math.acos(tmp)
        theta = math.atan2(ca - cb, d + sa - sb)
        t = normalize_angle(-alpha + theta + p/2)
        q = normalize_angle(beta - alpha - t + p)
    
    else:
        return None
    
    # Validate segment lengths
    if t < 0:
        t += 2 * math.pi
    if q < 0:
        q += 2 * math.pi
    
    return (t, p, q)


def connect_loiters(
    exit_point: Point,
    exit_heading: float,
    entry_point: Point,
    entry_heading: float,
    turn_radius: float
) -> Optional[DubinsPath]:
    """
    Generate a Dubins path connecting the exit of one loiter to the entry of another
    
    This is the primary function used by the path planner to connect loiter patterns.
    """
    return generate_dubins_path(
        exit_point, exit_heading,
        entry_point, entry_heading,
        turn_radius
    )
