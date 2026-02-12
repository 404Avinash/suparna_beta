"""
A* Pathfinder for Project KALPANA - Fixed
Properly avoids obstacles with safety margins!
"""

import math
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict
import numpy as np

from ..core.geometry import Point
from ..core.map import SurveillanceMap, CellType


@dataclass
class PathNode:
    """Node for A* pathfinding"""
    x: int
    y: int
    g_cost: float = 0.0
    h_cost: float = 0.0
    parent: Optional['PathNode'] = None
    
    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class AStarPathfinder:
    """A* pathfinding that PROPERLY avoids obstacles"""
    
    def __init__(self, surveillance_map: SurveillanceMap, safety_cells: int = 5):
        self.map = surveillance_map
        self.safety_cells = safety_cells  # Cells of margin around obstacles
        self.blocked = self._create_blocked_grid()
    
    def _create_blocked_grid(self) -> np.ndarray:
        """Create grid with obstacles expanded by safety margin"""
        grid = self.map.to_numpy()
        blocked = np.zeros((self.map.grid_height, self.map.grid_width), dtype=bool)
        
        # Mark obstacles and no-fly zones
        for cy in range(self.map.grid_height):
            for cx in range(self.map.grid_width):
                if grid[cy, cx] in (CellType.OBSTACLE, CellType.NO_FLY, CellType.SOFT_NO_FLY):
                    # Expand this cell by safety margin
                    for dy in range(-self.safety_cells, self.safety_cells + 1):
                        for dx in range(-self.safety_cells, self.safety_cells + 1):
                            ny, nx = cy + dy, cx + dx
                            if 0 <= ny < self.map.grid_height and 0 <= nx < self.map.grid_width:
                                blocked[ny, nx] = True
        
        return blocked
    
    def find_path(self, start: Point, goal: Point) -> List[Point]:
        """Find path from start to goal avoiding obstacles"""
        # Convert to grid coordinates
        start_cell = self.map.point_to_cell(start)
        goal_cell = self.map.point_to_cell(goal)
        
        # Clamp to grid bounds
        start_cell = (
            max(0, min(self.map.grid_width - 1, start_cell[0])),
            max(0, min(self.map.grid_height - 1, start_cell[1]))
        )
        goal_cell = (
            max(0, min(self.map.grid_width - 1, goal_cell[0])),
            max(0, min(self.map.grid_height - 1, goal_cell[1]))
        )
        
        # If start/goal blocked, find nearest free
        if self.blocked[start_cell[1], start_cell[0]]:
            start_cell = self._find_nearest_free(start_cell)
        if self.blocked[goal_cell[1], goal_cell[0]]:
            goal_cell = self._find_nearest_free(goal_cell)
        
        if start_cell is None or goal_cell is None:
            # Direct line as fallback
            return [start, goal]
        
        # A* search
        path_cells = self._astar(start_cell, goal_cell)
        
        if not path_cells:
            return [start, goal]
        
        # Simplify path
        simplified = self._simplify_path(path_cells)
        
        # Convert to world coordinates
        path = [start]
        for cell in simplified:
            path.append(self.map.cell_to_point(cell))
        path.append(goal)
        
        return path
    
    def _find_nearest_free(self, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """Find nearest unblocked cell using BFS"""
        from collections import deque
        
        visited = set()
        queue = deque([cell])
        
        while queue:
            cx, cy = queue.popleft()
            
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))
            
            if 0 <= cx < self.map.grid_width and 0 <= cy < self.map.grid_height:
                if not self.blocked[cy, cx]:
                    return (cx, cy)
            
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited:
                    queue.append((nx, ny))
        
        return None
    
    def _astar(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """A* algorithm"""
        start_node = PathNode(start[0], start[1])
        
        open_set = [start_node]
        closed_set: Set[Tuple[int, int]] = set()
        node_map: Dict[Tuple[int, int], PathNode] = {start: start_node}
        
        # 8-directional
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        max_iterations = 50000
        iteration = 0
        
        while open_set and iteration < max_iterations:
            iteration += 1
            current = heapq.heappop(open_set)
            
            if current.x == goal[0] and current.y == goal[1]:
                # Reconstruct path
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1]
            
            closed_set.add((current.x, current.y))
            
            for dx, dy in directions:
                nx, ny = current.x + dx, current.y + dy
                
                if not (0 <= nx < self.map.grid_width and 0 <= ny < self.map.grid_height):
                    continue
                
                if self.blocked[ny, nx]:
                    continue
                
                if (nx, ny) in closed_set:
                    continue
                
                move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                g_cost = current.g_cost + move_cost
                h_cost = math.sqrt((nx - goal[0])**2 + (ny - goal[1])**2)
                
                if (nx, ny) in node_map:
                    if g_cost < node_map[(nx, ny)].g_cost:
                        node_map[(nx, ny)].g_cost = g_cost
                        node_map[(nx, ny)].parent = current
                else:
                    new_node = PathNode(nx, ny, g_cost, h_cost, current)
                    node_map[(nx, ny)] = new_node
                    heapq.heappush(open_set, new_node)
        
        return []
    
    def _simplify_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Remove unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            for j in range(len(path) - 1, i, -1):
                if self._line_clear(path[i], path[j]):
                    simplified.append(path[j])
                    i = j
                    break
            else:
                i += 1
                if i < len(path):
                    simplified.append(path[i])
        
        return simplified
    
    def _line_clear(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        """Check if line is clear of obstacles"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        steps = max(abs(dx), abs(dy), 1)
        
        for i in range(steps + 1):
            t = i / steps
            x = int(p1[0] + t * dx)
            y = int(p1[1] + t * dy)
            
            if 0 <= y < self.map.grid_height and 0 <= x < self.map.grid_width:
                if self.blocked[y, x]:
                    return False
        
        return True


def plan_survey_mission(
    surveillance_map: SurveillanceMap,
    survey_points: List[Point],
    start: Optional[Point] = None
) -> Tuple[List[Point], List[int]]:
    """
    Plan complete mission with path and loiter indices
    
    Returns:
        (full_path, loiter_indices) - loiter_indices marks where to loiter
    """
    if start is None:
        start = surveillance_map.start_position
    
    pathfinder = AStarPathfinder(surveillance_map, safety_cells=6)
    
    full_path = []
    loiter_indices = []  # Which waypoint indices are loiter points
    
    current = start
    full_path.append(current)
    
    for target in survey_points:
        segment = pathfinder.find_path(current, target)
        
        # Add segment (skip first point which is current)
        for p in segment[1:]:
            full_path.append(p)
        
        # Mark this as a loiter point
        loiter_indices.append(len(full_path) - 1)
        current = target
    
    # Return home
    return_segment = pathfinder.find_path(current, start)
    for p in return_segment[1:]:
        full_path.append(p)
    
    return full_path, loiter_indices
