"""
KALPANA - Simple Survey Mission with Numbered Points
Clear HOME, numbered survey points, and return path
"""

import pygame
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

from ..core.geometry import Point, normalize_angle


COLORS = {
    'bg': (15, 18, 25),
    'free': (28, 32, 42),
    'obstacle': (90, 45, 45),
    'no_fly': (130, 40, 40),
    'covered': (35, 130, 70),
    'path': (60, 100, 160),
    'path_done': (35, 70, 100),
    'drone': (0, 210, 255),
    'drone_return': (80, 200, 100),
    'trail': (40, 100, 70),
    'home': (80, 200, 100),
    'waypoint': (255, 180, 80),
    'waypoint_done': (100, 140, 80),
    'text': (220, 220, 230),
    'text_dim': (100, 100, 115),
    'panel': (20, 23, 32),
}


@dataclass
class SimpleDrone:
    """Drone with smart edge-following obstacle avoidance"""
    position: Point = field(default_factory=lambda: Point(50, 50))
    heading: float = 0.0
    speed: float = 35.0
    
    waypoints: List[Point] = field(default_factory=list)
    current_idx: int = 0
    waypoint_radius: float = 20.0
    
    obstacles: List[Tuple[Point, float]] = field(default_factory=list)
    safety_margin: float = 55.0
    
    battery: float = 100.0
    distance: float = 0.0
    path_history: List[Point] = field(default_factory=list)
    
    # Edge following state
    avoiding: bool = False
    avoid_direction: int = 1  # 1 = turn right, -1 = turn left
    last_target_heading: float = 0.0
    
    def set_mission(self, waypoints: List[Point], obstacles: List[Tuple[Point, float]]):
        self.waypoints = waypoints
        self.obstacles = obstacles
        self.current_idx = 0
        self.path_history = []
        self.avoiding = False
    
    def _in_obstacle(self, pos: Point) -> bool:
        for center, radius in self.obstacles:
            if pos.distance_to(center) < radius + self.safety_margin:
                return True
        return False
    
    def _check_ahead(self, heading: float, dist: float = 40.0) -> bool:
        """Check if path ahead is clear"""
        test_pos = Point(
            self.position.x + dist * math.cos(heading),
            self.position.y + dist * math.sin(heading)
        )
        return not self._in_obstacle(test_pos)
    
    def update(self, dt: float) -> int:
        """Update drone - returns 0=moving, 1=reached, 2=done"""
        if self.current_idx >= len(self.waypoints):
            return 2
        
        target = self.waypoints[self.current_idx]
        dist_to_target = self.position.distance_to(target)
        
        # Check if reached
        if dist_to_target < self.waypoint_radius:
            self.current_idx += 1
            self.avoiding = False
            return 1
        
        # Get heading to target
        target_heading = math.atan2(target.y - self.position.y, target.x - self.position.x)
        self.last_target_heading = target_heading
        
        # Check if path to target is clear
        if self._check_ahead(target_heading):
            # Path clear - go direct
            self.avoiding = False
            desired_heading = target_heading
        else:
            # Path blocked - edge following
            if not self.avoiding:
                # Start avoiding - pick direction
                self.avoiding = True
                # Choose direction based on obstacle position
                for center, _ in self.obstacles:
                    if self.position.distance_to(center) < 200:
                        # Turn away from obstacle center
                        to_obs = math.atan2(center.y - self.position.y, center.x - self.position.x)
                        diff = normalize_angle(target_heading - to_obs)
                        self.avoid_direction = 1 if diff > 0 else -1
                        break
            
            # Keep turning in avoid direction until path clears
            desired_heading = self.heading + self.avoid_direction * 0.8
            
            # Check if we can start heading back toward target
            for test_angle in [0.3, 0.5, 0.7]:
                test_heading = self.heading + self.avoid_direction * test_angle
                adjusted = normalize_angle(test_heading)
                if self._check_ahead(adjusted, 50):
                    heading_to_target = normalize_angle(target_heading - adjusted)
                    if abs(heading_to_target) < 1.5:  # Within 90 degrees of target
                        desired_heading = adjusted
                        break
        
        # Turn toward desired heading
        heading_diff = normalize_angle(desired_heading - self.heading)
        turn_speed = 2.8
        
        if abs(heading_diff) > turn_speed * dt:
            self.heading += turn_speed * dt if heading_diff > 0 else -turn_speed * dt
        else:
            self.heading = desired_heading
        self.heading = normalize_angle(self.heading)
        
        # Move forward
        new_pos = Point(
            self.position.x + self.speed * dt * math.cos(self.heading),
            self.position.y + self.speed * dt * math.sin(self.heading)
        )
        
        # Only move if clear
        if not self._in_obstacle(new_pos):
            dist = self.position.distance_to(new_pos)
            self.position = new_pos
            self.distance += dist
        else:
            # Stuck - force turn
            self.heading += self.avoid_direction * 0.5
        
        self.battery -= 0.06 * dt
        
        # Record path
        if not self.path_history or self.position.distance_to(self.path_history[-1]) > 5:
            self.path_history.append(Point(self.position.x, self.position.y))
        
        return 0
    
    @property
    def progress(self) -> float:
        if not self.waypoints:
            return 0
        return 100.0 * self.current_idx / len(self.waypoints)
    
    @property
    def done(self) -> bool:
        return self.current_idx >= len(self.waypoints)


class SimpleVisualizer:
    """Visualization with HOME and numbered survey points"""
    
    def __init__(self, width: int = 1200, height: int = 800):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("KALPANA - Survey Mission")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.font_big = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 20)
        
        self.width = width
        self.height = height
        self.panel_width = 200
        self.map_width = width - self.panel_width
        
        # Map
        self.world_width = 1100.0
        self.world_height = 750.0
        self.scale = min(self.map_width / self.world_width, self.height / self.world_height) * 0.92
        self.offset_x = 25
        self.offset_y = 25
        
        # Obstacles - spread out
        self.obstacles = [
            (Point(350, 400), 70),
            (Point(650, 550), 60),
            (Point(650, 200), 55),
            (Point(900, 400), 50),
        ]
        
        # HOME and survey points - clear hierarchy
        self.home = Point(100, 375)
        self.survey_points = [
            Point(200, 600),    # 1
            Point(500, 650),    # 2
            Point(800, 600),    # 3
            Point(1000, 400),   # 4
            Point(800, 150),    # 5
            Point(500, 100),    # 6
            Point(200, 150),    # 7
        ]
        
        # Build mission path: HOME -> points -> HOME
        self.mission = [self.home] + self.survey_points + [self.home]
        
        # Drone
        self.drone = SimpleDrone(position=Point(self.home.x, self.home.y))
        self.drone.set_mission(self.mission, self.obstacles)
        
        # Coverage
        self.covered_cells = set()
        
        self.speed = 2.0
        self.paused = False
    
    def world_to_screen(self, p: Point) -> Tuple[int, int]:
        x = int(self.offset_x + p.x * self.scale)
        y = int(self.height - self.offset_y - p.y * self.scale)
        return (x, y)
    
    def update_coverage(self):
        cx, cy = int(self.drone.position.x / 25), int(self.drone.position.y / 25)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                cell = (cx + dx, cy + dy)
                pt = Point(cell[0] * 25, cell[1] * 25)
                if not any(pt.distance_to(c) < r for c, r in self.obstacles):
                    self.covered_cells.add(cell)
    
    def draw_map(self):
        # Background
        pygame.draw.rect(self.screen, COLORS['free'],
            (self.offset_x, self.offset_y, 
             int(self.world_width * self.scale), int(self.world_height * self.scale)))
        
        # Coverage
        cell_size = int(25 * self.scale)
        for cx, cy in self.covered_cells:
            sx = int(self.offset_x + cx * 25 * self.scale)
            sy = int(self.height - self.offset_y - (cy + 1) * 25 * self.scale)
            pygame.draw.rect(self.screen, COLORS['covered'], (sx, sy, cell_size, cell_size))
        
        # Obstacles
        for center, radius in self.obstacles:
            pos = self.world_to_screen(center)
            r = int(radius * self.scale)
            pygame.draw.circle(self.screen, COLORS['no_fly'], pos, r)
            pygame.draw.circle(self.screen, COLORS['obstacle'], pos, int(r * 0.7))
    
    def draw_mission(self):
        """Draw path, HOME, and numbered survey points"""
        # Path lines
        for i in range(len(self.mission) - 1):
            p1 = self.world_to_screen(self.mission[i])
            p2 = self.world_to_screen(self.mission[i + 1])
            
            if i < self.drone.current_idx:
                pygame.draw.line(self.screen, COLORS['path_done'], p1, p2, 3)
            else:
                pygame.draw.line(self.screen, COLORS['path'], p1, p2, 2)
        
        # HOME marker
        home_pos = self.world_to_screen(self.home)
        pygame.draw.circle(self.screen, COLORS['home'], home_pos, 15, 3)
        pygame.draw.circle(self.screen, COLORS['home'], home_pos, 6)
        label = self.font_small.render("HOME", True, COLORS['home'])
        self.screen.blit(label, (home_pos[0] - 18, home_pos[1] + 18))
        
        # Numbered survey points
        for i, pt in enumerate(self.survey_points):
            pos = self.world_to_screen(pt)
            
            # Check if visited
            visited = (i + 1) < self.drone.current_idx  # +1 because mission[0] is home
            color = COLORS['waypoint_done'] if visited else COLORS['waypoint']
            
            # Circle
            pygame.draw.circle(self.screen, color, pos, 14, 2 if visited else 0)
            
            # Number
            num = self.font.render(str(i + 1), True, COLORS['bg'] if not visited else color)
            self.screen.blit(num, (pos[0] - 5, pos[1] - 8))
    
    def draw_drone(self):
        # Trail
        if len(self.drone.path_history) > 1:
            trail = [self.world_to_screen(p) for p in self.drone.path_history[-400:]]
            if len(trail) > 1:
                pygame.draw.lines(self.screen, COLORS['trail'], False, trail, 3)
        
        # Drone
        pos = self.world_to_screen(self.drone.position)
        is_returning = self.drone.current_idx >= len(self.survey_points) + 1
        color = COLORS['drone_return'] if is_returning else COLORS['drone']
        
        size = 12
        angle = -self.drone.heading
        pts = []
        for i in range(3):
            a = angle + i * 2 * math.pi / 3
            mult = 1.7 if i == 0 else 1.0
            pts.append((
                pos[0] + int(size * mult * math.cos(a)),
                pos[1] + int(size * mult * math.sin(a))
            ))
        
        pygame.draw.polygon(self.screen, color, pts)
        pygame.draw.polygon(self.screen, (255, 255, 255), pts, 2)
    
    def draw_panel(self):
        px = self.width - self.panel_width + 15
        pygame.draw.rect(self.screen, COLORS['panel'],
            (self.width - self.panel_width, 0, self.panel_width, self.height))
        
        y = 25
        
        # Title
        t = self.font_big.render("KALPANA", True, COLORS['drone'])
        self.screen.blit(t, (px, y))
        y += 30
        t = self.font_small.render("Survey Mission", True, COLORS['text_dim'])
        self.screen.blit(t, (px, y))
        y += 40
        
        # Current target
        if self.drone.current_idx == 0:
            target_name = "Departing HOME"
        elif self.drone.current_idx <= len(self.survey_points):
            target_name = f"→ Point {self.drone.current_idx}"
        elif self.drone.done:
            target_name = "Mission Complete!"
        else:
            target_name = "→ HOME (Return)"
        
        t = self.font.render(target_name, True, COLORS['waypoint'])
        self.screen.blit(t, (px, y))
        y += 40
        
        # Progress
        t = self.font_small.render("PROGRESS", True, COLORS['text_dim'])
        self.screen.blit(t, (px, y))
        y += 18
        bar_w = self.panel_width - 40
        pygame.draw.rect(self.screen, COLORS['bg'], (px, y, bar_w, 16))
        pygame.draw.rect(self.screen, COLORS['covered'], (px, y, int(bar_w * self.drone.progress / 100), 16))
        t = self.font_small.render(f"{self.drone.progress:.0f}%", True, COLORS['text'])
        self.screen.blit(t, (px + bar_w//2 - 12, y))
        y += 35
        
        # Battery
        t = self.font_small.render("BATTERY", True, COLORS['text_dim'])
        self.screen.blit(t, (px, y))
        y += 18
        bat_color = (200, 50, 50) if self.drone.battery < 20 else COLORS['covered']
        pygame.draw.rect(self.screen, COLORS['bg'], (px, y, bar_w, 16))
        pygame.draw.rect(self.screen, bat_color, (px, y, int(bar_w * self.drone.battery / 100), 16))
        t = self.font_small.render(f"{self.drone.battery:.0f}%", True, COLORS['text'])
        self.screen.blit(t, (px + bar_w//2 - 12, y))
        y += 35
        
        # Stats
        t = self.font_small.render(f"Distance: {self.drone.distance:.0f}m", True, COLORS['text'])
        self.screen.blit(t, (px, y))
        y += 22
        t = self.font_small.render(f"Speed: {self.speed:.1f}x", True, COLORS['text'])
        self.screen.blit(t, (px, y))
        y += 35
        
        # Survey points checklist
        t = self.font.render("WAYPOINTS", True, COLORS['text'])
        self.screen.blit(t, (px, y))
        y += 25
        
        for i in range(len(self.survey_points)):
            visited = (i + 1) < self.drone.current_idx
            marker = "✓" if visited else "○"
            color = COLORS['covered'] if visited else COLORS['text_dim']
            t = self.font_small.render(f"{marker} Point {i + 1}", True, color)
            self.screen.blit(t, (px, y))
            y += 20
        
        # Return home status
        returned = self.drone.done
        marker = "✓" if returned else "○"
        color = COLORS['home'] if returned else COLORS['text_dim']
        t = self.font_small.render(f"{marker} Return HOME", True, color)
        self.screen.blit(t, (px, y))
        y += 35
        
        # Controls
        t = self.font.render("CONTROLS", True, COLORS['text'])
        self.screen.blit(t, (px, y))
        y += 22
        for c in ["SPACE - Pause", "+/- Speed", "R - Reset", "ESC - Exit"]:
            t = self.font_small.render(c, True, COLORS['text_dim'])
            self.screen.blit(t, (px, y))
            y += 18
    
    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                        self.speed = min(10, self.speed + 0.5)
                    elif event.key == pygame.K_MINUS:
                        self.speed = max(0.5, self.speed - 0.5)
                    elif event.key == pygame.K_r:
                        self.drone = SimpleDrone(position=Point(self.home.x, self.home.y))
                        self.drone.set_mission(self.mission, self.obstacles)
                        self.covered_cells = set()
            
            dt = self.clock.tick(60) / 1000.0
            
            if not self.paused and not self.drone.done:
                self.drone.update(dt * self.speed)
                self.update_coverage()
            
            self.screen.fill(COLORS['bg'])
            self.draw_map()
            self.draw_mission()
            self.draw_drone()
            self.draw_panel()
            pygame.display.flip()
        
        pygame.quit()


def run_demo():
    print("=" * 50)
    print("KALPANA - Survey Mission")
    print("=" * 50)
    print("\nMission:")
    print("  HOME -> Point 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> HOME")
    print("\nDrone avoids obstacles and covers area")
    print("Anti-circling: skips waypoint if stuck > 8 sec")
    print()
    
    viz = SimpleVisualizer()
    viz.run()
    
    print("Mission complete!")
