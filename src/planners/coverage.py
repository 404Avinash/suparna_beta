"""
Coverage Planner for Project KALPANA
Loiter-first coverage strategy - the heart of Swift-inspired path planning

Instead of traditional lawnmower patterns, we place optimal loiter circles
that maximize coverage while minimizing energy consumption.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set
import numpy as np

from ..core.geometry import Point
from ..core.map import SurveillanceMap, CellType
from ..core.loiter import Loiter, LoiterType, create_loiter, LOITER_RADIUS_RANGES
from ..core.dubins import connect_loiters, DubinsPath


@dataclass
class MissionPath:
    """
    Complete mission path consisting of loiter patterns and transitions
    
    This is the output of the coverage planner - a sequence of loiters
    connected by Dubins paths that covers the surveillance area.
    """
    loiters: List[Loiter] = field(default_factory=list)
    transitions: List[Optional[DubinsPath]] = field(default_factory=list)
    
    @property
    def total_loiter_distance(self) -> float:
        """Total distance spent in loiter patterns"""
        return sum(l.total_distance for l in self.loiters)
    
    @property
    def total_transition_distance(self) -> float:
        """Total distance spent transitioning between loiters"""
        return sum(t.total_length for t in self.transitions if t is not None)
    
    @property
    def total_distance(self) -> float:
        """Total mission distance"""
        return self.total_loiter_distance + self.total_transition_distance
    
    @property
    def total_energy(self) -> float:
        """Estimated total energy consumption"""
        loiter_energy = sum(l.energy_cost for l in self.loiters)
        # Transitions are slightly more efficient than tight loiters
        transition_energy = self.total_transition_distance * 0.9
        return loiter_energy + transition_energy
    
    @property
    def coverage_area(self) -> float:
        """Total area covered by all loiters"""
        return sum(l.coverage_area for l in self.loiters)
    
    def get_all_waypoints(self) -> List[Point]:
        """Get all waypoints in order for the entire mission"""
        waypoints = []
        for i, loiter in enumerate(self.loiters):
            # Add transition waypoints first (if not the first loiter)
            if i > 0 and i-1 < len(self.transitions) and self.transitions[i-1]:
                waypoints.extend(self.transitions[i-1].waypoints)
            # Add loiter waypoints
            waypoints.extend(loiter.waypoints)
        return waypoints


@dataclass
class CoveragePlanner:
    """
    Loiter-based coverage planner
    
    Strategy: Greedy Set Cover with Energy Weights
    - Place loiters to maximize coverage per energy unit
    - Connect loiters with Dubins paths
    - Respect obstacles and no-fly zones
    """
    surveillance_map: SurveillanceMap
    
    # Planner parameters
    loiter_type: LoiterType = LoiterType.STANDARD
    loiter_radius: float = 150.0
    turn_radius: float = 50.0         # Minimum turn radius for Dubins paths
    overlap_factor: float = 0.3       # 30% overlap between adjacent loiters
    max_loiters: int = 100            # Maximum number of loiters
    coverage_threshold: float = 95.0  # Target coverage percentage
    
    def plan(self, start_position: Optional[Point] = None) -> MissionPath:
        """
        Generate a coverage path starting from the given position
        
        Args:
            start_position: Starting position (uses map's start if not specified)
        
        Returns:
            MissionPath with loiters and transitions
        """
        if start_position is None:
            start_position = self.surveillance_map.start_position
        
        mission = MissionPath()
        current_pos = start_position
        current_heading = 0.0  # Initial heading (east)
        
        # Track covered cells
        covered_cells: Set[Tuple[int, int]] = set()
        
        # Get all uncovered cells
        uncovered = set(self.surveillance_map.get_uncovered_cells())
        total_free_cells = len(uncovered)
        
        if total_free_cells == 0:
            return mission
        
        iteration = 0
        while iteration < self.max_loiters:
            iteration += 1
            
            # Find best loiter position
            best_loiter, best_score = self._find_best_loiter(
                current_pos, current_heading, uncovered
            )
            
            if best_loiter is None or best_score <= 0:
                break
            
            # Add transition from current position to loiter entry
            if len(mission.loiters) > 0:
                transition = self._plan_transition(
                    current_pos, current_heading,
                    best_loiter.get_entry_point(), best_loiter.entry_heading
                )
                mission.transitions.append(transition)
            
            # Add the loiter
            mission.loiters.append(best_loiter)
            
            # Update coverage
            newly_covered = self._mark_loiter_coverage(best_loiter, uncovered)
            covered_cells.update(newly_covered)
            
            # Update current position and heading
            current_pos = best_loiter.get_exit_point()
            current_heading = best_loiter.exit_heading
            
            # Check if we've reached target coverage
            coverage_pct = 100.0 * len(covered_cells) / total_free_cells
            if coverage_pct >= self.coverage_threshold:
                break
        
        return mission
    
    def _find_best_loiter(
        self,
        current_pos: Point,
        current_heading: float,
        uncovered: Set[Tuple[int, int]]
    ) -> Tuple[Optional[Loiter], float]:
        """
        Find the best loiter position using greedy set cover
        
        Score = coverage / (transition_cost + loiter_cost)
        """
        best_loiter = None
        best_score = -1.0
        
        # Generate candidate positions (grid of potential loiter centers)
        candidates = self._generate_candidates(uncovered)
        
        for center in candidates:
            # Skip if too close to obstacles
            if not self._is_valid_loiter_position(center):
                continue
            
            # Create candidate loiter
            loiter = create_loiter(
                center=center,
                loiter_type=self.loiter_type,
                radius=self.loiter_radius,
                entry_heading=current_pos.heading_to(center)
            )
            
            # Calculate coverage (cells that would be newly covered)
            coverage = self._estimate_coverage(loiter, uncovered)
            
            if coverage == 0:
                continue
            
            # Calculate cost (transition + loiter)
            transition_cost = current_pos.distance_to(center)
            loiter_cost = loiter.energy_cost
            total_cost = transition_cost + loiter_cost
            
            # Score: coverage per unit cost
            score = coverage / total_cost
            
            if score > best_score:
                best_score = score
                best_loiter = loiter
        
        return best_loiter, best_score
    
    def _generate_candidates(
        self, 
        uncovered: Set[Tuple[int, int]]
    ) -> List[Point]:
        """Generate candidate loiter center positions"""
        candidates = []
        
        # Spacing between candidates (based on loiter radius and overlap)
        spacing = self.loiter_radius * (2 - self.overlap_factor)
        
        # Get bounds from uncovered cells
        if not uncovered:
            return candidates
        
        resolution = self.surveillance_map.resolution
        
        # Grid-based candidates
        min_x = min(c[0] for c in uncovered) * resolution
        max_x = max(c[0] for c in uncovered) * resolution
        min_y = min(c[1] for c in uncovered) * resolution
        max_y = max(c[1] for c in uncovered) * resolution
        
        x = min_x + self.loiter_radius
        while x < max_x:
            y = min_y + self.loiter_radius
            while y < max_y:
                candidates.append(Point(x, y))
                y += spacing
            x += spacing
        
        # Also add candidates centered on clusters of uncovered cells
        # (This helps catch isolated uncovered areas)
        if len(uncovered) < 1000:  # Only for smaller sets
            sample_size = min(50, len(uncovered))
            sampled = list(uncovered)[:sample_size]
            for cell in sampled:
                center = self.surveillance_map.cell_to_point(cell)
                candidates.append(center)
        
        return candidates
    
    def _is_valid_loiter_position(self, center: Point) -> bool:
        """Check if a loiter can be placed at this position"""
        # Check if center is in bounds
        if center.x < 0 or center.x > self.surveillance_map.width:
            return False
        if center.y < 0 or center.y > self.surveillance_map.height:
            return False
        
        # Check if loiter would intersect obstacles
        for obstacle in self.surveillance_map.obstacles:
            min_distance = obstacle.radius + self.loiter_radius
            if obstacle.is_no_fly:
                min_distance += self.surveillance_map.no_fly_margin
            else:
                min_distance += self.surveillance_map.obstacle_margin
            
            if center.distance_to(obstacle.center) < min_distance:
                return False
        
        return True
    
    def _estimate_coverage(
        self, 
        loiter: Loiter, 
        uncovered: Set[Tuple[int, int]]
    ) -> int:
        """Estimate how many uncovered cells this loiter would cover"""
        count = 0
        resolution = self.surveillance_map.resolution
        
        # Check cells within loiter radius
        min_x = int((loiter.center.x - loiter.radius) / resolution)
        max_x = int((loiter.center.x + loiter.radius) / resolution) + 1
        min_y = int((loiter.center.y - loiter.radius) / resolution)
        max_y = int((loiter.center.y + loiter.radius) / resolution) + 1
        
        for cy in range(min_y, max_y):
            for cx in range(min_x, max_x):
                if (cx, cy) in uncovered:
                    cell_center = self.surveillance_map.cell_to_point((cx, cy))
                    if cell_center.distance_to(loiter.center) <= loiter.radius:
                        count += 1
        
        return count
    
    def _mark_loiter_coverage(
        self, 
        loiter: Loiter, 
        uncovered: Set[Tuple[int, int]]
    ) -> Set[Tuple[int, int]]:
        """Mark cells as covered and return the newly covered cells"""
        newly_covered = set()
        resolution = self.surveillance_map.resolution
        
        min_x = int((loiter.center.x - loiter.radius) / resolution)
        max_x = int((loiter.center.x + loiter.radius) / resolution) + 1
        min_y = int((loiter.center.y - loiter.radius) / resolution)
        max_y = int((loiter.center.y + loiter.radius) / resolution) + 1
        
        for cy in range(min_y, max_y):
            for cx in range(min_x, max_x):
                if (cx, cy) in uncovered:
                    cell_center = self.surveillance_map.cell_to_point((cx, cy))
                    if cell_center.distance_to(loiter.center) <= loiter.radius:
                        uncovered.discard((cx, cy))
                        newly_covered.add((cx, cy))
                        self.surveillance_map.mark_covered(
                            cell_center, 
                            resolution/2, 
                            1.0
                        )
        
        return newly_covered
    
    def _plan_transition(
        self,
        from_pos: Point,
        from_heading: float,
        to_pos: Point,
        to_heading: float
    ) -> Optional[DubinsPath]:
        """Plan a Dubins path transition between two configurations"""
        return connect_loiters(
            from_pos, from_heading,
            to_pos, to_heading,
            self.turn_radius
        )


def quick_plan(
    map_width: float = 1000.0,
    map_height: float = 1000.0,
    num_obstacles: int = 5,
    loiter_type: LoiterType = LoiterType.STANDARD
) -> Tuple[SurveillanceMap, MissionPath]:
    """
    Quick planning function for testing and demos
    
    Creates a random map and generates a coverage path.
    """
    import random
    
    # Create map with random obstacles
    survey_map = SurveillanceMap(
        width=map_width,
        height=map_height,
        resolution=10.0,
        start_position=Point(50, 50)
    )
    
    for i in range(num_obstacles):
        x = random.uniform(100, map_width - 100)
        y = random.uniform(100, map_height - 100)
        radius = random.uniform(30, 80)
        is_no_fly = random.random() < 0.3
        survey_map.add_circular_obstacle(
            Point(x, y), radius, is_no_fly, f"Obstacle {i+1}"
        )
    
    # Plan coverage
    planner = CoveragePlanner(
        surveillance_map=survey_map,
        loiter_type=loiter_type,
        loiter_radius=LOITER_RADIUS_RANGES[loiter_type][0]
    )
    
    mission = planner.plan()
    
    return survey_map, mission
