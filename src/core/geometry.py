"""
Geometry utilities for Project SUPARNA
Basic geometric operations optimized for drone path planning
"""

import math
from dataclasses import dataclass
from typing import Tuple, List
import numpy as np


@dataclass
class Point:
    """2D point with x, y coordinates"""
    x: float
    y: float
    
    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar: float) -> 'Point':
        return Point(self.x * scalar, self.y * scalar)
    
    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y])
    
    @classmethod
    def from_tuple(cls, t: Tuple[float, float]) -> 'Point':
        return cls(t[0], t[1])
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate Euclidean distance to another point"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def heading_to(self, other: 'Point') -> float:
        """Calculate heading angle (radians) to another point"""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.atan2(dy, dx)


def calculate_distance(p1: Point, p2: Point) -> float:
    """Calculate Euclidean distance between two points"""
    return p1.distance_to(p2)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def point_in_circle(point: Point, center: Point, radius: float) -> bool:
    """Check if a point is inside a circle"""
    return calculate_distance(point, center) <= radius


def circle_intersection(c1: Point, r1: float, c2: Point, r2: float) -> List[Point]:
    """Find intersection points of two circles (if any)"""
    d = calculate_distance(c1, c2)
    
    # No intersection cases
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return []
    
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(max(0, r1**2 - a**2))
    
    # Point on line between centers
    px = c1.x + a * (c2.x - c1.x) / d
    py = c1.y + a * (c2.y - c1.y) / d
    
    # Intersection points
    p1 = Point(
        px + h * (c2.y - c1.y) / d,
        py - h * (c2.x - c1.x) / d
    )
    p2 = Point(
        px - h * (c2.y - c1.y) / d,
        py + h * (c2.x - c1.x) / d
    )
    
    if h == 0:
        return [p1]
    return [p1, p2]


def rotate_point(point: Point, center: Point, angle: float) -> Point:
    """Rotate a point around a center by given angle (radians)"""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    # Translate to origin
    tx = point.x - center.x
    ty = point.y - center.y
    
    # Rotate
    rx = tx * cos_a - ty * sin_a
    ry = tx * sin_a + ty * cos_a
    
    # Translate back
    return Point(rx + center.x, ry + center.y)


def generate_circle_points(center: Point, radius: float, num_points: int = 36) -> List[Point]:
    """Generate points along a circle perimeter"""
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        points.append(Point(x, y))
    return points


def line_circle_intersection(
    line_start: Point, 
    line_end: Point, 
    center: Point, 
    radius: float
) -> List[Point]:
    """Find intersection points of a line segment with a circle"""
    dx = line_end.x - line_start.x
    dy = line_end.y - line_start.y
    
    fx = line_start.x - center.x
    fy = line_start.y - center.y
    
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    
    discriminant = b * b - 4 * a * c
    
    if discriminant < 0:
        return []
    
    intersections = []
    discriminant = math.sqrt(discriminant)
    
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    
    for t in [t1, t2]:
        if 0 <= t <= 1:
            intersections.append(Point(
                line_start.x + t * dx,
                line_start.y + t * dy
            ))
    
    return intersections
