"""
Reactive Obstacle Avoidance for Project SUPARNA
Bug Algorithm inspired edge-following behavior

When the drone detects an obstacle ahead:
1. Turn to follow the obstacle's edge
2. Keep the obstacle on one side while moving forward
3. Resume original path when obstacle is cleared
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from enum import Enum, auto

from ..core.geometry import Point, normalize_angle
from ..core.map import SurveillanceMap, CellType


class AvoidanceState(Enum):
    """States for the obstacle avoidance state machine"""
    NORMAL = auto()         # Flying normally toward target
    AVOIDING = auto()       # Following obstacle edge
    RECOVERING = auto()     # Returning to original path


@dataclass
class ObstacleDetector:
    """
    Detects obstacles ahead of the drone using raycasting
    """
    surveillance_map: SurveillanceMap
    detection_range: float = 80.0       # How far ahead to look (meters)
    detection_angle: float = 60.0       # FOV for detection (degrees)
    num_rays: int = 7                   # Number of detection rays
    safety_margin: float = 30.0         # Minimum distance to maintain
    
    def detect_obstacle_ahead(
        self, 
        position: Point, 
        heading: float
    ) -> Tuple[bool, Optional[float], Optional[Point]]:
        """
        Cast rays ahead to detect obstacles
        
        Returns:
            (obstacle_detected, distance_to_obstacle, obstacle_point)
        """
        half_angle = math.radians(self.detection_angle / 2)
        
        closest_dist = float('inf')
        closest_point = None
        
        for i in range(self.num_rays):
            # Spread rays across the detection FOV
            ray_angle = heading - half_angle + (2 * half_angle * i / (self.num_rays - 1))
            
            # Cast ray
            dist, hit_point = self._cast_ray(position, ray_angle)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_point = hit_point
        
        if closest_dist < self.detection_range:
            return True, closest_dist, closest_point
        
        return False, None, None
    
    def _cast_ray(
        self, 
        origin: Point, 
        angle: float
    ) -> Tuple[float, Optional[Point]]:
        """Cast a single ray and return distance to first obstacle"""
        step_size = self.surveillance_map.resolution
        max_steps = int(self.detection_range / step_size)
        
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        for step in range(1, max_steps + 1):
            dist = step * step_size
            check_point = Point(
                origin.x + dist * dx,
                origin.y + dist * dy
            )
            
            if not self.surveillance_map.is_point_safe(check_point):
                return dist, check_point
        
        return float('inf'), None
    
    def get_clear_direction(
        self, 
        position: Point, 
        current_heading: float,
        prefer_right: bool = True
    ) -> Optional[float]:
        """
        Find the nearest clear direction to avoid obstacle
        
        Scans left and right to find a heading that's obstacle-free
        """
        # Check directions in order of preference
        angles_to_check = []
        
        for delta in range(10, 180, 10):
            if prefer_right:
                angles_to_check.append(current_heading - math.radians(delta))
                angles_to_check.append(current_heading + math.radians(delta))
            else:
                angles_to_check.append(current_heading + math.radians(delta))
                angles_to_check.append(current_heading - math.radians(delta))
        
        for angle in angles_to_check:
            dist, _ = self._cast_ray(position, angle)
            if dist > self.detection_range * 0.8:
                return normalize_angle(angle)
        
        return None
    
    def get_edge_following_direction(
        self, 
        position: Point, 
        current_heading: float,
        obstacle_point: Point,
        follow_right: bool = True
    ) -> float:
        """
        Calculate heading to follow obstacle edge
        
        The drone will move parallel to the obstacle, keeping it on one side
        """
        # Vector from drone to obstacle
        to_obstacle = Point(
            obstacle_point.x - position.x,
            obstacle_point.y - position.y
        )
        
        # Perpendicular direction (along the edge)
        if follow_right:
            # Keep obstacle on right, so turn left (perpendicular)
            edge_heading = math.atan2(to_obstacle.y, to_obstacle.x) + math.pi/2
        else:
            # Keep obstacle on left, so turn right
            edge_heading = math.atan2(to_obstacle.y, to_obstacle.x) - math.pi/2
        
        return normalize_angle(edge_heading)


@dataclass 
class ReactiveAvoidance:
    """
    Reactive obstacle avoidance controller
    
    Implements a modified Bug2 algorithm:
    1. Move toward goal
    2. If obstacle detected, follow its edge
    3. Resume direct path when obstacle is cleared
    """
    detector: ObstacleDetector
    state: AvoidanceState = AvoidanceState.NORMAL
    
    # Memory for avoidance
    original_target: Optional[Point] = None
    follow_right: bool = True
    avoidance_start_pos: Optional[Point] = None
    min_avoidance_distance: float = 50.0  # Min distance before checking if clear
    
    def get_adjusted_heading(
        self,
        position: Point,
        current_heading: float,
        target: Point
    ) -> Tuple[float, bool]:
        """
        Get the heading adjusted for obstacle avoidance
        
        Returns:
            (adjusted_heading, is_avoiding)
        """
        # Direct heading to target
        target_heading = position.heading_to(target)
        
        # Check for obstacles
        obstacle_ahead, dist, obstacle_point = self.detector.detect_obstacle_ahead(
            position, current_heading
        )
        
        if self.state == AvoidanceState.NORMAL:
            if obstacle_ahead and dist < self.detector.safety_margin * 2:
                # Start avoidance
                self.state = AvoidanceState.AVOIDING
                self.original_target = target
                self.avoidance_start_pos = position
                
                # Decide which way to go around (prefer the side closer to target)
                angle_to_target = target_heading - current_heading
                self.follow_right = angle_to_target < 0
                
                # Get edge-following direction
                new_heading = self.detector.get_edge_following_direction(
                    position, current_heading, obstacle_point, self.follow_right
                )
                return new_heading, True
            else:
                # Normal navigation toward target
                return target_heading, False
        
        elif self.state == AvoidanceState.AVOIDING:
            # Check if we can resume direct path
            dist_from_start = position.distance_to(self.avoidance_start_pos) if self.avoidance_start_pos else 0
            
            if dist_from_start > self.min_avoidance_distance:
                # Check if path to target is clear
                target_clear, target_dist, _ = self.detector.detect_obstacle_ahead(
                    position, target_heading
                )
                
                if not target_clear or target_dist > self.detector.detection_range * 0.9:
                    # Path is clear, resume normal navigation
                    self.state = AvoidanceState.NORMAL
                    self.original_target = None
                    self.avoidance_start_pos = None
                    return target_heading, False
            
            # Continue edge following
            if obstacle_ahead and obstacle_point:
                new_heading = self.detector.get_edge_following_direction(
                    position, current_heading, obstacle_point, self.follow_right
                )
            else:
                # Lost the obstacle edge, curve back toward target
                curve_amount = math.radians(30)
                if self.follow_right:
                    new_heading = normalize_angle(current_heading + curve_amount)
                else:
                    new_heading = normalize_angle(current_heading - curve_amount)
            
            return new_heading, True
        
        return target_heading, False
    
    def reset(self):
        """Reset avoidance state"""
        self.state = AvoidanceState.NORMAL
        self.original_target = None
        self.avoidance_start_pos = None
