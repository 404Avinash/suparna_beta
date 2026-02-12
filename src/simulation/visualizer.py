"""
SUPARNA — Integrated Mission Visualization
Uses CoveragePlanner, Loiter patterns, Dubins transitions, and real coverage tracking
"""

import pygame
import math
import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field

from ..core.geometry import Point, normalize_angle
from ..core.map import SurveillanceMap, Obstacle, CellType
from ..core.loiter import Loiter, LoiterType
from ..planners.coverage import CoveragePlanner, MissionPath


# ── Color Palette (dark theme) ──────────────────────────────────────────────
COLORS = {
    'bg':             (12, 15, 22),
    'grid_bg':        (22, 26, 35),
    'obstacle':       (100, 45, 45),
    'obstacle_glow':  (55, 30, 30),
    'covered':        (28, 110, 58),
    'drone':          (0, 210, 255),
    'drone_loiter':   (255, 210, 60),
    'trail':          (0, 130, 170),
    'home':           (80, 200, 100),
    'loiter_pending': (255, 180, 80),
    'loiter_active':  (255, 220, 80),
    'loiter_done':    (70, 155, 70),
    'path_line':      (45, 75, 120),
    'text':           (210, 210, 220),
    'text_dim':       (90, 95, 110),
    'text_accent':    (0, 200, 240),
    'panel_bg':       (16, 19, 28),
    'bar_bg':         (30, 34, 45),
    'bar_good':       (30, 140, 80),
    'bar_warn':       (200, 150, 40),
    'bar_danger':     (190, 50, 50),
}


class SuparnaVisualizer:
    """Full integrated visualization using real planning algorithms"""

    def __init__(self, width: int = 1200, height: int = 800):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SUPARNA — Autonomous Surveillance")
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.Font(None, 34)
        self.font = pygame.font.Font(None, 26)
        self.font_small = pygame.font.Font(None, 20)
        self.font_tiny = pygame.font.Font(None, 16)

        self.width = width
        self.height = height
        self.panel_w = 220
        self.map_w = width - self.panel_w

        # World dimensions
        self.world_w = 1000.0
        self.world_h = 700.0
        self.scale = min(self.map_w / self.world_w, self.height / self.world_h) * 0.9
        self.ox = 15
        self.oy = 15

        self._setup()

        self.speed_mult = 2.0
        self.paused = False

    # ── Setup ────────────────────────────────────────────────────────────────

    def _setup(self):
        """Create map, run planner, init drone"""
        self.home = Point(80, 350)

        obstacles = [
            Obstacle(center=Point(350, 400), radius=60, name='building_1'),
            Obstacle(center=Point(650, 550), radius=50, name='building_2'),
            Obstacle(center=Point(650, 200), radius=45, name='tower_1'),
            Obstacle(center=Point(900, 380), radius=40, name='building_3'),
        ]

        self.smap = SurveillanceMap(
            width=self.world_w,
            height=self.world_h,
            resolution=10.0,
            obstacles=obstacles,
            start_position=self.home,
            obstacle_margin=20.0,
            no_fly_margin=30.0,
        )

        # ── Run coverage planner (Greedy Set Cover) ──
        planner = CoveragePlanner(
            surveillance_map=self.smap,
            loiter_type=LoiterType.STANDARD,
            loiter_radius=80.0,
            turn_radius=40.0,
            overlap_factor=0.3,
            coverage_threshold=98.0,
            max_loiters=50,
        )
        self.mission: MissionPath = planner.plan(start_position=self.home)

        # Extract loiter targets for drawing
        self.loiter_targets = [(l.center, l.radius) for l in self.mission.loiters]

        # Build waypoints: home → loiter centers → home
        self.waypoints: List[Point] = [self.home]
        self.wp_is_loiter: List[bool] = [False]
        self.wp_loiter_r: List[float] = [0.0]

        for loiter in self.mission.loiters:
            self.waypoints.append(loiter.center)
            self.wp_is_loiter.append(True)
            self.wp_loiter_r.append(loiter.radius)

        self.waypoints.append(self.home)
        self.wp_is_loiter.append(False)
        self.wp_loiter_r.append(0.0)

        self._reset_drone()

        # Coverage tracking
        self.total_free = int(np.sum(self.smap.grid == CellType.FREE))
        self.total_free += int(np.sum(self.smap.grid == CellType.START))
        self.total_free = max(1, self.total_free)

    def _reset_drone(self):
        """Reset drone state and coverage"""
        self.pos = Point(self.home.x, self.home.y)
        self.heading = 0.0
        self.spd = 35.0
        self.turn_rate = 2.5
        self.wp_idx = 0
        self.state = 'FLY'  # FLY | LOITER | DONE

        self.loiter_center: Optional[Point] = None
        self.loiter_angle = 0.0
        self.loiter_revs = 0.0
        self.loiter_r = 60.0

        # Obstacle avoidance state
        self.avoiding = False
        self.avoid_dir = 1  # 1=right, -1=left

        self.trail: List[Point] = []
        self.dist = 0.0
        self.battery = 100.0
        self.n_loiters_done = 0
        self.covered_cells: Set[Tuple[int, int]] = set()
        self.smap.coverage_grid[:] = 0.0

    # ── Obstacle avoidance helpers ────────────────────────────────────────────

    def _in_obstacle(self, pos: Point) -> bool:
        """Check if position is inside any obstacle's safety margin"""
        for obs in self.smap.obstacles:
            if pos.distance_to(obs.center) < obs.radius + self.smap.obstacle_margin + 10:
                return True
        return False

    def _check_ahead(self, heading: float, dist: float = 50.0) -> bool:
        """Check if path ahead is clear of obstacles"""
        test = Point(
            self.pos.x + dist * math.cos(heading),
            self.pos.y + dist * math.sin(heading),
        )
        return not self._in_obstacle(test)

    # ── Drone update ─────────────────────────────────────────────────────────

    def _update(self, dt: float):
        if self.state == 'DONE':
            return
        if self.state == 'LOITER':
            self._do_loiter(dt)
            return
        if self.wp_idx >= len(self.waypoints):
            self.state = 'DONE'
            return

        target = self.waypoints[self.wp_idx]

        # Target heading
        th = math.atan2(target.y - self.pos.y, target.x - self.pos.x)

        # ── Obstacle avoidance (edge-following) ──
        if self._check_ahead(th):
            # Path clear — fly direct
            self.avoiding = False
            desired = th
        else:
            # Path blocked — edge-follow around obstacle
            if not self.avoiding:
                self.avoiding = True
                # Pick avoidance direction: turn away from nearest obstacle
                for obs in self.smap.obstacles:
                    if self.pos.distance_to(obs.center) < 200:
                        to_obs = math.atan2(obs.center.y - self.pos.y,
                                            obs.center.x - self.pos.x)
                        diff = normalize_angle(th - to_obs)
                        self.avoid_dir = 1 if diff > 0 else -1
                        break

            desired = self.heading + self.avoid_dir * 0.8

            # Try to curve back toward target
            for test_a in [0.3, 0.5, 0.7]:
                test_h = normalize_angle(self.heading + self.avoid_dir * test_a)
                if self._check_ahead(test_h, 60):
                    gap = abs(normalize_angle(th - test_h))
                    if gap < 1.5:
                        desired = test_h
                        break

        # Smooth turning
        err = normalize_angle(desired - self.heading)
        mt = self.turn_rate * dt
        if abs(err) > mt:
            self.heading += mt if err > 0 else -mt
        else:
            self.heading = desired
        self.heading = normalize_angle(self.heading)

        # Move forward (only if clear)
        d = self.spd * dt
        new_pos = Point(
            self.pos.x + d * math.cos(self.heading),
            self.pos.y + d * math.sin(self.heading),
        )
        if not self._in_obstacle(new_pos):
            self.pos = new_pos
            self.dist += d
        else:
            # Stuck — force turn
            self.heading += self.avoid_dir * 0.5

        self.battery -= 0.08 * dt

        # Trail
        if not self.trail or self.pos.distance_to(self.trail[-1]) > 4:
            self.trail.append(Point(self.pos.x, self.pos.y))

        # Coverage scan while flying
        self._mark_coverage(self.pos, 4)

        # Check waypoint reached
        if self.pos.distance_to(target) < 20:
            self.avoiding = False
            if self.wp_is_loiter[self.wp_idx]:
                self.loiter_r = self.wp_loiter_r[self.wp_idx]
                self._enter_loiter(target)
            else:
                self.wp_idx += 1

    def _enter_loiter(self, center: Point):
        self.state = 'LOITER'
        self.loiter_center = center
        self.loiter_angle = self.heading - math.pi / 2
        self.loiter_revs = 0.0

    def _do_loiter(self, dt: float):
        if not self.loiter_center:
            self.state = 'FLY'
            return

        w = self.spd / self.loiter_r
        self.loiter_angle += w * dt
        self.loiter_revs += w * dt / (2 * math.pi)

        self.pos = Point(
            self.loiter_center.x + self.loiter_r * math.cos(self.loiter_angle),
            self.loiter_center.y + self.loiter_r * math.sin(self.loiter_angle),
        )
        self.heading = normalize_angle(self.loiter_angle + math.pi / 2)

        self.dist += self.spd * dt
        self.battery -= 0.05 * dt  # Loiter is more energy-efficient

        if not self.trail or self.pos.distance_to(self.trail[-1]) > 3:
            self.trail.append(Point(self.pos.x, self.pos.y))

        # Wider coverage scan during loiter
        r_cells = int(self.loiter_r / self.smap.resolution) + 2
        self._mark_coverage(self.loiter_center, r_cells)

        if self.loiter_revs >= 1.0:
            self.n_loiters_done += 1
            self.state = 'FLY'
            self.loiter_center = None
            self.wp_idx += 1

    def _mark_coverage(self, center: Point, radius_cells: int):
        cell = self.smap.point_to_cell(center)
        if cell is None:
            return
        cx, cy = cell
        r2 = radius_cells * radius_cells
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= r2:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.smap.grid_width and 0 <= ny < self.smap.grid_height:
                        if self.smap.grid[ny, nx] in (CellType.FREE, CellType.START):
                            self.covered_cells.add((nx, ny))

    @property
    def coverage_pct(self) -> float:
        return 100.0 * len(self.covered_cells) / self.total_free

    # ── Coordinate transform ─────────────────────────────────────────────────

    def w2s(self, p: Point) -> Tuple[int, int]:
        """World coordinates to screen pixels"""
        return (
            int(self.ox + p.x * self.scale),
            int(self.height - self.oy - p.y * self.scale),
        )

    # ── Drawing ──────────────────────────────────────────────────────────────

    def draw_map(self):
        # Map background
        mr = (self.ox, self.oy,
              int(self.world_w * self.scale), int(self.world_h * self.scale))
        pygame.draw.rect(self.screen, COLORS['grid_bg'], mr)

        # Coverage cells (green)
        cs = max(2, int(self.smap.resolution * self.scale))
        for (cx, cy) in self.covered_cells:
            sx = int(self.ox + cx * self.smap.resolution * self.scale)
            sy = int(self.height - self.oy - (cy + 1) * self.smap.resolution * self.scale)
            pygame.draw.rect(self.screen, COLORS['covered'], (sx, sy, cs, cs))

        # Obstacles
        for obs in self.smap.obstacles:
            pos = self.w2s(obs.center)
            r = int(obs.radius * self.scale)
            mr_r = int((obs.radius + self.smap.obstacle_margin) * self.scale)
            pygame.draw.circle(self.screen, COLORS['obstacle_glow'], pos, mr_r)
            pygame.draw.circle(self.screen, COLORS['obstacle'], pos, r)

    def draw_loiters(self):
        """Draw loiter target circles with status"""
        for i, (center, radius) in enumerate(self.loiter_targets):
            pos = self.w2s(center)
            r = int(radius * self.scale)

            if i < self.n_loiters_done:
                # Completed
                pygame.draw.circle(self.screen, COLORS['loiter_done'], pos, r, 2)
                lbl = self.font_small.render("✓", True, COLORS['loiter_done'])
                self.screen.blit(lbl, (pos[0] - 5, pos[1] - 8))
            elif i == self.n_loiters_done and self.state == 'LOITER':
                # Active — draw progress arc
                pygame.draw.circle(self.screen, COLORS['loiter_active'], pos, r, 3)
                prog = min(self.loiter_revs, 1.0)
                arc_r = pygame.Rect(pos[0] - r, pos[1] - r, r * 2, r * 2)
                if prog > 0.01:
                    pygame.draw.arc(self.screen, COLORS['drone'], arc_r,
                                    0, prog * 2 * math.pi, 4)
            else:
                # Pending
                pygame.draw.circle(self.screen, COLORS['loiter_pending'], pos, r, 1)

            # Loiter number
            num = self.font_tiny.render(str(i + 1), True,
                                        COLORS['loiter_done'] if i < self.n_loiters_done
                                        else COLORS['loiter_pending'])
            self.screen.blit(num, (pos[0] + r + 3, pos[1] - 6))

    def draw_path(self):
        """Draw planned path lines between waypoints"""
        pts = [self.home] + [c for c, _ in self.loiter_targets] + [self.home]
        for i in range(len(pts) - 1):
            p1, p2 = self.w2s(pts[i]), self.w2s(pts[i + 1])
            done = i < self.wp_idx
            if done:
                pygame.draw.line(self.screen, COLORS['path_line'], p1, p2, 2)
            else:
                # Dashed line for future path
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                length = math.sqrt(dx * dx + dy * dy)
                if length < 2:
                    continue
                steps = max(1, int(length / 10))
                for s in range(0, steps, 2):
                    t1 = s / steps
                    t2 = min((s + 1) / steps, 1.0)
                    a = (int(p1[0] + dx * t1), int(p1[1] + dy * t1))
                    b = (int(p1[0] + dx * t2), int(p1[1] + dy * t2))
                    pygame.draw.line(self.screen, COLORS['path_line'], a, b, 1)

    def draw_drone(self):
        # Trail
        trail_pts = [self.w2s(p) for p in self.trail[-500:]]
        for i in range(1, len(trail_pts)):
            alpha = i / len(trail_pts)
            c = tuple(int(v * alpha) for v in COLORS['trail'])
            pygame.draw.line(self.screen, c, trail_pts[i - 1], trail_pts[i], 2)

        # Drone body
        pos = self.w2s(self.pos)
        color = COLORS['drone_loiter'] if self.state == 'LOITER' else COLORS['drone']
        a = -self.heading  # Screen y is inverted
        sz = 12
        pts = [
            (pos[0] + int(sz * 1.8 * math.cos(a)),
             pos[1] + int(sz * 1.8 * math.sin(a))),
            (pos[0] + int(sz * math.cos(a + 2.3)),
             pos[1] + int(sz * math.sin(a + 2.3))),
            (pos[0] + int(sz * math.cos(a - 2.3)),
             pos[1] + int(sz * math.sin(a - 2.3))),
        ]
        pygame.draw.polygon(self.screen, color, pts)
        pygame.draw.polygon(self.screen, (255, 255, 255), pts, 2)

        # Scan radius glow
        scan_r = int(40 * self.scale)
        glow = pygame.Surface((scan_r * 2, scan_r * 2), pygame.SRCALPHA)
        pygame.draw.circle(glow, (*color, 25), (scan_r, scan_r), scan_r)
        self.screen.blit(glow, (pos[0] - scan_r, pos[1] - scan_r))

    def draw_home(self):
        pos = self.w2s(self.home)
        pygame.draw.circle(self.screen, COLORS['home'], pos, 14, 3)
        pygame.draw.circle(self.screen, COLORS['home'], pos, 5)
        lbl = self.font_small.render("HOME", True, COLORS['home'])
        self.screen.blit(lbl, (pos[0] - 18, pos[1] + 16))

    def draw_panel(self):
        px = self.width - self.panel_w
        pygame.draw.rect(self.screen, COLORS['panel_bg'],
                         (px, 0, self.panel_w, self.height))
        pygame.draw.line(self.screen, COLORS['text_dim'],
                         (px, 0), (px, self.height), 1)

        x = px + 15
        y = 20
        bw = self.panel_w - 35  # bar width

        # Title
        t = self.font_title.render("SUPARNA", True, COLORS['text_accent'])
        self.screen.blit(t, (x, y)); y += 26
        t = self.font_tiny.render("Swift-Inspired Surveillance", True, COLORS['text_dim'])
        self.screen.blit(t, (x, y)); y += 30

        # Status
        if self.state == 'DONE':
            status = "MISSION COMPLETE"
            sc = COLORS['home']
        elif self.state == 'LOITER':
            status = f"LOITERING ({self.n_loiters_done + 1}/{len(self.loiter_targets)})"
            sc = COLORS['drone_loiter']
        elif self.wp_idx >= len(self.waypoints) - 1:
            status = "RETURNING HOME"
            sc = COLORS['home']
        else:
            status = f"EN ROUTE → L{self.n_loiters_done + 1}"
            sc = COLORS['drone']
        t = self.font.render(status, True, sc)
        self.screen.blit(t, (x, y)); y += 32

        # Coverage bar
        t = self.font_small.render("COVERAGE", True, COLORS['text_dim'])
        self.screen.blit(t, (x, y)); y += 18
        cov = self.coverage_pct
        pygame.draw.rect(self.screen, COLORS['bar_bg'], (x, y, bw, 14))
        cov_color = COLORS['bar_good'] if cov > 80 else COLORS['bar_warn']
        pygame.draw.rect(self.screen, cov_color,
                         (x, y, int(bw * min(cov, 100) / 100), 14))
        t = self.font_tiny.render(f"{cov:.1f}%", True, COLORS['text'])
        self.screen.blit(t, (x + bw // 2 - 14, y + 1)); y += 28

        # Battery bar
        t = self.font_small.render("BATTERY", True, COLORS['text_dim'])
        self.screen.blit(t, (x, y)); y += 18
        bat = max(0, self.battery)
        bat_c = COLORS['bar_danger'] if bat < 20 else (
            COLORS['bar_warn'] if bat < 40 else COLORS['bar_good'])
        pygame.draw.rect(self.screen, COLORS['bar_bg'], (x, y, bw, 14))
        pygame.draw.rect(self.screen, bat_c,
                         (x, y, int(bw * min(bat, 100) / 100), 14))
        t = self.font_tiny.render(f"{bat:.0f}%", True, COLORS['text'])
        self.screen.blit(t, (x + bw // 2 - 14, y + 1)); y += 28

        # Stats
        for label, val in [
            ("Distance", f"{self.dist:.0f}m"),
            ("Speed", f"{self.speed_mult:.1f}x"),
            ("Loiters", f"{self.n_loiters_done}/{len(self.loiter_targets)}"),
        ]:
            t = self.font_small.render(f"{label}: {val}", True, COLORS['text'])
            self.screen.blit(t, (x, y)); y += 22
        y += 10

        # Loiter checklist
        t = self.font.render("MISSION", True, COLORS['text'])
        self.screen.blit(t, (x, y)); y += 24
        for i in range(len(self.loiter_targets)):
            done = i < self.n_loiters_done
            active = i == self.n_loiters_done and self.state == 'LOITER'
            sym = "✓" if done else ("▶" if active else "○")
            c = COLORS['loiter_done'] if done else (
                COLORS['loiter_active'] if active else COLORS['text_dim'])
            t = self.font_small.render(f"{sym} Loiter {i + 1}", True, c)
            self.screen.blit(t, (x, y)); y += 20

        done_all = self.state == 'DONE'
        sym = "✓" if done_all else "○"
        c = COLORS['home'] if done_all else COLORS['text_dim']
        t = self.font_small.render(f"{sym} Return HOME", True, c)
        self.screen.blit(t, (x, y)); y += 30

        # Algorithm info
        t = self.font.render("ALGORITHMS", True, COLORS['text'])
        self.screen.blit(t, (x, y)); y += 22
        for algo in ["Greedy Set Cover", "Dubins Curves", "A* Pathfinder",
                      "Bug2 Avoidance", "Loiter Patterns"]:
            t = self.font_tiny.render(f"• {algo}", True, COLORS['text_dim'])
            self.screen.blit(t, (x, y)); y += 16
        y += 12

        # Controls
        t = self.font.render("CONTROLS", True, COLORS['text'])
        self.screen.blit(t, (x, y)); y += 22
        for c_txt in ["SPACE  Pause", "+/-    Speed", "R      Reset", "ESC    Exit"]:
            t = self.font_tiny.render(c_txt, True, COLORS['text_dim'])
            self.screen.blit(t, (x, y)); y += 16

    # ── Main loop ────────────────────────────────────────────────────────────

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
                        self.speed_mult = min(10, self.speed_mult + 0.5)
                    elif event.key == pygame.K_MINUS:
                        self.speed_mult = max(0.5, self.speed_mult - 0.5)
                    elif event.key == pygame.K_r:
                        self._reset_drone()

            dt = self.clock.tick(60) / 1000.0

            if not self.paused and self.state != 'DONE':
                self._update(dt * self.speed_mult)

            # Draw everything
            self.screen.fill(COLORS['bg'])
            self.draw_map()
            self.draw_path()
            self.draw_loiters()
            self.draw_home()
            self.draw_drone()
            self.draw_panel()
            pygame.display.flip()

        pygame.quit()


# ── Public API ───────────────────────────────────────────────────────────────

class SimpleVisualizer(SuparnaVisualizer):
    """Backward-compatible alias"""
    pass


def run_demo():
    print("=" * 55)
    print("  SUPARNA — Swift-Inspired Autonomous Surveillance")
    print("=" * 55)
    print()
    print("  Algorithms: Greedy Set Cover + Dubins + A* + Bug2")
    print("  Planning mission with loiter patterns...")
    print()

    viz = SuparnaVisualizer()

    n = len(viz.loiter_targets)
    print(f"  Mission planned: {n} loiter{'s' if n != 1 else ''}")
    print(f"  HOME -> {'->'.join(f'L{i+1}' for i in range(n))} -> HOME")
    print()
    print("  Starting simulation...")
    print()

    viz.run()

    print(f"  Coverage: {viz.coverage_pct:.1f}%")
    print(f"  Distance: {viz.dist:.0f}m")
    print(f"  Battery:  {viz.battery:.0f}%")
    print("  Mission complete!")
