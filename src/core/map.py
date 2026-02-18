"""
Surveillance Map Representation for Project SUPARNA
Grid-based map with obstacles, no-fly zones, and coverage tracking
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set
from enum import IntEnum

from .geometry import Point


class CellType(IntEnum):
    """Types of cells in the surveillance map"""
    FREE = 0            # Safe to fly and survey
    OBSTACLE = 1        # Physical obstacle (building, mountain, tree)
    NO_FLY = 2          # No-fly zone (restricted airspace)
    SOFT_NO_FLY = 3     # Discouraged zone (penalized but not forbidden)
    COVERED = 4         # Already surveyed
    START = 5           # Starting position


@dataclass
class Obstacle:
    """Represents an obstacle or no-fly zone"""
    center: Point
    radius: float               # For circular obstacles
    is_no_fly: bool = False     # True = no-fly zone, False = physical obstacle
    is_soft: bool = False       # Soft no-fly = discouraged but allowed
    name: str = ""
    
    def contains(self, point: Point) -> bool:
        """Check if a point is inside this obstacle"""
        return point.distance_to(self.center) <= self.radius


@dataclass 
class SurveillanceMap:
    """
    Grid-based surveillance map for drone path planning
    
    The map is represented as a 2D numpy array where each cell has:
    - A type (free, obstacle, no-fly, covered)
    - Coverage status tracking
    
    Coordinates:
    - Origin (0,0) is bottom-left
    - X increases rightward
    - Y increases upward
    - All measurements in meters
    """
    width: float                    # Map width in meters
    height: float                   # Map height in meters
    resolution: float = 10.0        # Meters per grid cell
    
    # Computed grid properties
    grid_width: int = field(init=False)
    grid_height: int = field(init=False)
    grid: np.ndarray = field(init=False)
    coverage_grid: np.ndarray = field(init=False)  # Float 0-1 coverage
    
    # Map elements
    obstacles: List[Obstacle] = field(default_factory=list)
    start_position: Point = field(default_factory=lambda: Point(0, 0))
    
    # Safety margins
    obstacle_margin: float = 20.0   # Safety buffer around obstacles (meters)
    no_fly_margin: float = 50.0     # Safety buffer around no-fly zones (meters)
    
    def __post_init__(self):
        """Initialize the grid"""
        self.grid_width = int(np.ceil(self.width / self.resolution))
        self.grid_height = int(np.ceil(self.height / self.resolution))
        
        # Initialize grids
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.coverage_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        
        # Mark start position
        start_cell = self.point_to_cell(self.start_position)
        if self._is_valid_cell(start_cell):
            self.grid[start_cell[1], start_cell[0]] = CellType.START
    
    def _is_valid_cell(self, cell: Tuple[int, int]) -> bool:
        """Check if a cell coordinate is within grid bounds"""
        return 0 <= cell[0] < self.grid_width and 0 <= cell[1] < self.grid_height
    
    def point_to_cell(self, point: Point) -> Tuple[int, int]:
        """Convert world coordinates to grid cell"""
        x = int(point.x / self.resolution)
        y = int(point.y / self.resolution)
        return (x, y)
    
    def cell_to_point(self, cell: Tuple[int, int]) -> Point:
        """Convert grid cell to world coordinates (center of cell)"""
        x = (cell[0] + 0.5) * self.resolution
        y = (cell[1] + 0.5) * self.resolution
        return Point(x, y)
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Add an obstacle to the map and update the grid"""
        self.obstacles.append(obstacle)
        self._rasterize_obstacle(obstacle)
    
    def _rasterize_obstacle(self, obstacle: Obstacle) -> None:
        """Rasterize an obstacle onto the grid"""
        # Determine cell type
        if obstacle.is_no_fly:
            cell_type = CellType.SOFT_NO_FLY if obstacle.is_soft else CellType.NO_FLY
            margin = self.no_fly_margin
        else:
            cell_type = CellType.OBSTACLE
            margin = self.obstacle_margin
        
        # Expanded radius with margin
        total_radius = obstacle.radius + margin
        
        # Find cells within the obstacle's bounding box
        min_x = max(0, int((obstacle.center.x - total_radius) / self.resolution))
        max_x = min(self.grid_width, int((obstacle.center.x + total_radius) / self.resolution) + 1)
        min_y = max(0, int((obstacle.center.y - total_radius) / self.resolution))
        max_y = min(self.grid_height, int((obstacle.center.y + total_radius) / self.resolution) + 1)
        
        # Mark cells within the obstacle
        for cy in range(min_y, max_y):
            for cx in range(min_x, max_x):
                cell_center = self.cell_to_point((cx, cy))
                dist = cell_center.distance_to(obstacle.center)
                
                if dist <= obstacle.radius:
                    # Hard obstacle/no-fly
                    if self.grid[cy, cx] < cell_type:  # Don't downgrade
                        self.grid[cy, cx] = cell_type
                elif dist <= total_radius:
                    # Margin zone (soft penalty)
                    if self.grid[cy, cx] == CellType.FREE:
                        self.grid[cy, cx] = CellType.SOFT_NO_FLY
    
    def add_circular_obstacle(
        self, 
        center: Point, 
        radius: float, 
        is_no_fly: bool = False,
        name: str = ""
    ) -> None:
        """Convenience method to add a circular obstacle"""
        obstacle = Obstacle(center, radius, is_no_fly, False, name)
        self.add_obstacle(obstacle)
    
    def is_point_safe(self, point: Point, check_soft: bool = False) -> bool:
        """Check if a point is safe to fly through"""
        cell = self.point_to_cell(point)
        if not self._is_valid_cell(cell):
            return False
        
        cell_type = self.grid[cell[1], cell[0]]
        
        if cell_type in (CellType.OBSTACLE, CellType.NO_FLY):
            return False
        if check_soft and cell_type == CellType.SOFT_NO_FLY:
            return False
        return True
    
    def is_path_safe(
        self, 
        start: Point, 
        end: Point, 
        check_soft: bool = False,
        step_size: float = None
    ) -> bool:
        """Check if a straight path between two points is safe"""
        if step_size is None:
            step_size = self.resolution / 2
        
        dist = start.distance_to(end)
        if dist == 0:
            return self.is_point_safe(start, check_soft)
        
        steps = int(np.ceil(dist / step_size))
        for i in range(steps + 1):
            t = i / steps
            point = Point(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y)
            )
            if not self.is_point_safe(point, check_soft):
                return False
        return True
    
    def mark_covered(self, center: Point, radius: float, coverage_value: float = 1.0) -> int:
        """
        Mark an area as covered (surveyed)
        
        Args:
            center: Center of covered area
            radius: Radius of covered area
            coverage_value: Coverage intensity (0-1)
        
        Returns:
            Number of newly covered cells
        """
        newly_covered = 0
        
        min_x = max(0, int((center.x - radius) / self.resolution))
        max_x = min(self.grid_width, int((center.x + radius) / self.resolution) + 1)
        min_y = max(0, int((center.y - radius) / self.resolution))
        max_y = min(self.grid_height, int((center.y + radius) / self.resolution) + 1)
        
        for cy in range(min_y, max_y):
            for cx in range(min_x, max_x):
                cell_center = self.cell_to_point((cx, cy))
                dist = cell_center.distance_to(center)
                
                if dist <= radius:
                    if self.grid[cy, cx] == CellType.FREE:
                        old_coverage = self.coverage_grid[cy, cx]
                        # Combine coverage (max, not additive)
                        self.coverage_grid[cy, cx] = max(old_coverage, coverage_value)
                        if old_coverage < 0.5 and self.coverage_grid[cy, cx] >= 0.5:
                            newly_covered += 1
        
        return newly_covered
    
    def get_coverage_percentage(self) -> float:
        """Calculate percentage of free area that has been covered"""
        free_cells = np.sum(self.grid == CellType.FREE)
        if free_cells == 0:
            return 100.0
        
        covered_cells = np.sum(
            (self.grid == CellType.FREE) & (self.coverage_grid >= 0.5)
        )
        return 100.0 * covered_cells / free_cells
    
    def get_uncovered_cells(self) -> List[Tuple[int, int]]:
        """Get list of uncovered free cells"""
        uncovered = []
        for cy in range(self.grid_height):
            for cx in range(self.grid_width):
                if (self.grid[cy, cx] == CellType.FREE and 
                    self.coverage_grid[cy, cx] < 0.5):
                    uncovered.append((cx, cy))
        return uncovered
    
    def get_obstacle_at(self, point: Point) -> Optional[Obstacle]:
        """Get the obstacle at a given point, if any"""
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return obstacle
        return None
    
    def get_distance_to_nearest_obstacle(self, point: Point) -> float:
        """Get distance to the nearest obstacle from a point"""
        min_dist = float('inf')
        for obstacle in self.obstacles:
            dist = point.distance_to(obstacle.center) - obstacle.radius
            min_dist = min(min_dist, dist)
        return min_dist
    
    def get_traversable_area(self) -> float:
        """Get total traversable (free) area in square meters"""
        free_cells = np.sum(self.grid == CellType.FREE)
        return free_cells * self.resolution * self.resolution
    
    def to_numpy(self) -> np.ndarray:
        """Get the grid as a numpy array for visualization"""
        return self.grid.copy()
    
    def get_coverage_array(self) -> np.ndarray:
        """Get coverage grid for visualization"""
        return self.coverage_grid.copy()
    
    @classmethod
    def create_test_map(cls, size: float = 1000.0) -> 'SurveillanceMap':
        """Create a test map with some obstacles for development"""
        survey_map = cls(
            width=size,
            height=size,
            resolution=10.0,
            start_position=Point(50, 50)
        )
        
        # Add some test obstacles
        survey_map.add_circular_obstacle(
            Point(300, 300), 80, is_no_fly=False, name="Building A"
        )
        survey_map.add_circular_obstacle(
            Point(600, 400), 100, is_no_fly=True, name="No-Fly Zone 1"
        )
        survey_map.add_circular_obstacle(
            Point(200, 700), 60, is_no_fly=False, name="Tower"
        )
        survey_map.add_circular_obstacle(
            Point(800, 200), 50, is_no_fly=False, name="Building B"
        )
        survey_map.add_circular_obstacle(
            Point(500, 800), 120, is_no_fly=True, name="No-Fly Zone 2"
        )
        
        return survey_map
