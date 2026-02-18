"""
Transition Planner for Project SUPARNA
Plans smooth transitions between loiter patterns using Dubins paths
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..core.geometry import Point
from ..core.loiter import Loiter
from ..core.dubins import DubinsPath, generate_dubins_path
from ..core.map import SurveillanceMap


@dataclass
class TransitionPlanner:
    """
    Plans optimal transitions between loiter patterns
    
    Uses Dubins paths to ensure smooth, flyable transitions
    that respect minimum turn radius constraints.
    """
    turn_radius: float = 50.0
    safety_margin: float = 20.0
    surveillance_map: Optional[SurveillanceMap] = None
    
    def plan_transition(
        self,
        from_loiter: Loiter,
        to_loiter: Loiter
    ) -> Optional[DubinsPath]:
        """
        Plan a Dubins path from one loiter exit to another loiter entry
        """
        exit_point = from_loiter.get_exit_point()
        exit_heading = from_loiter.exit_heading
        
        # Calculate optimal entry heading for the target loiter
        # (heading that aligns with the loiter's rotation direction)
        entry_point = to_loiter.get_entry_point()
        entry_heading = to_loiter.entry_heading
        
        path = generate_dubins_path(
            exit_point, exit_heading,
            entry_point, entry_heading,
            self.turn_radius
        )
        
        # Validate path doesn't intersect obstacles
        if path and self.surveillance_map:
            if not self._validate_path(path):
                # Try alternative entry points
                path = self._find_safe_transition(from_loiter, to_loiter)
        
        return path
    
    def plan_return_to_base(
        self,
        final_loiter: Loiter,
        base_position: Point,
        base_heading: float = 0.0
    ) -> Optional[DubinsPath]:
        """
        Plan return path from final loiter to base
        """
        exit_point = final_loiter.get_exit_point()
        exit_heading = final_loiter.exit_heading
        
        return generate_dubins_path(
            exit_point, exit_heading,
            base_position, base_heading,
            self.turn_radius
        )
    
    def _validate_path(self, path: DubinsPath) -> bool:
        """Check if a path is safe (doesn't intersect obstacles)"""
        if not self.surveillance_map:
            return True
        
        for waypoint in path.waypoints:
            if not self.surveillance_map.is_point_safe(waypoint):
                return False
        
        return True
    
    def _find_safe_transition(
        self,
        from_loiter: Loiter,
        to_loiter: Loiter
    ) -> Optional[DubinsPath]:
        """
        Try to find a safe transition by adjusting entry/exit points
        """
        # Try different entry angles
        for angle_offset in [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2]:
            adjusted_entry_heading = to_loiter.entry_heading + angle_offset
            
            # Calculate new entry point
            entry_x = to_loiter.center.x + to_loiter.radius * math.cos(adjusted_entry_heading)
            entry_y = to_loiter.center.y + to_loiter.radius * math.sin(adjusted_entry_heading)
            entry_point = Point(entry_x, entry_y)
            
            exit_point = from_loiter.get_exit_point()
            exit_heading = from_loiter.exit_heading
            
            path = generate_dubins_path(
                exit_point, exit_heading,
                entry_point, adjusted_entry_heading,
                self.turn_radius
            )
            
            if path and self._validate_path(path):
                return path
        
        return None
    
    def optimize_loiter_sequence(
        self,
        loiters: List[Loiter]
    ) -> Tuple[List[Loiter], float]:
        """
        Optimize the order of loiters to minimize total transition distance
        
        Uses a simple nearest-neighbor heuristic (could be improved with TSP solvers)
        """
        if len(loiters) <= 2:
            return loiters, self._calculate_total_transition_distance(loiters)
        
        # Start with first loiter fixed
        optimized = [loiters[0]]
        remaining = list(loiters[1:])
        
        while remaining:
            current = optimized[-1]
            exit_point = current.get_exit_point()
            
            # Find nearest remaining loiter
            best_idx = 0
            best_dist = float('inf')
            
            for i, loiter in enumerate(remaining):
                dist = exit_point.distance_to(loiter.center)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            
            optimized.append(remaining.pop(best_idx))
        
        total_distance = self._calculate_total_transition_distance(optimized)
        return optimized, total_distance
    
    def _calculate_total_transition_distance(
        self,
        loiters: List[Loiter]
    ) -> float:
        """Calculate total transition distance for a sequence of loiters"""
        total = 0.0
        
        for i in range(len(loiters) - 1):
            path = self.plan_transition(loiters[i], loiters[i + 1])
            if path:
                total += path.total_length
            else:
                # Fallback to straight-line distance
                total += loiters[i].get_exit_point().distance_to(
                    loiters[i + 1].get_entry_point()
                )
        
        return total
