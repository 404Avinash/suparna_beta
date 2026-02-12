"""
SUPARNA — LAC (Line of Actual Control) Sector Map
Generates a realistic Ladakh border patrol terrain with:
  - Procedural heightmap (Perlin-like noise)
  - Named mountain peaks as obstacles
  - Chinese military posts as no-fly zones
  - Indian FOB as home base
"""

import math
import random
import numpy as np
from src.core.geometry import Point
from src.core.map import SurveillanceMap, Obstacle


# ── Perlin-like noise (simple gradient noise) ──────────────────────────────
def _fade(t):
    return t * t * t * (t * (t * 6 - 15) + 10)

def _lerp(a, b, t):
    return a + t * (b - a)

def _grad(h, x, y):
    vectors = [(1,1),(-1,1),(1,-1),(-1,-1),(1,0),(-1,0),(0,1),(0,-1)]
    g = vectors[h % 8]
    return g[0]*x + g[1]*y

def perlin_noise(nx, ny, perm):
    """Single octave of 2D Perlin noise."""
    x0 = int(math.floor(nx)); x1 = x0 + 1
    y0 = int(math.floor(ny)); y1 = y0 + 1
    sx = _fade(nx - x0); sy = _fade(ny - y0)

    n00 = _grad(perm[(perm[x0 % 256] + y0) % 256], nx-x0, ny-y0)
    n10 = _grad(perm[(perm[x1 % 256] + y0) % 256], nx-x1, ny-y0)
    n01 = _grad(perm[(perm[x0 % 256] + y1) % 256], nx-x0, ny-y1)
    n11 = _grad(perm[(perm[x1 % 256] + y1) % 256], nx-x1, ny-y1)

    ix0 = _lerp(n00, n10, sx)
    ix1 = _lerp(n01, n11, sx)
    return _lerp(ix0, ix1, sy)

def generate_heightmap(width, height, resolution, seed=42):
    """
    Generate a 2D heightmap array for mountain terrain.
    Returns elevation grid in meters (3500-5500m range, typical Ladakh).
    """
    rng = random.Random(seed)
    perm = list(range(256))
    rng.shuffle(perm)
    perm = perm + perm  # double for overflow

    cols = int(width / resolution)
    rows = int(height / resolution)
    heightmap = np.zeros((rows, cols), dtype=np.float32)

    for r in range(rows):
        for c in range(cols):
            x = c / cols
            y = r / rows

            # Layer 1: Large mountain ranges
            e = perlin_noise(x * 4, y * 4, perm) * 1.0
            # Layer 2: Ridge patterns
            e += perlin_noise(x * 8, y * 8, perm) * 0.5
            # Layer 3: Rocky detail
            e += perlin_noise(x * 16, y * 16, perm) * 0.25
            # Layer 4: Fine texture
            e += perlin_noise(x * 32, y * 32, perm) * 0.12

            # Normalize to 0-1 range
            e = (e + 1.2) / 2.4
            e = max(0, min(1, e))

            # Map to Ladakh elevation range: 3500m (valley) to 5500m (peak)
            heightmap[r][c] = 3500 + e * 2000

    return heightmap


# ── LAC Sector Landmarks ──────────────────────────────────────────────────
# Positions are in meters within a 5000 x 3500m sector

LAC_LANDMARKS = {
    'Galwan Valley':    {'pos': (1200, 800),  'type': 'valley'},
    'Pangong Tso':      {'pos': (3800, 2800), 'type': 'lake'},
    'Hot Springs':      {'pos': (2500, 1200), 'type': 'region'},
    'Depsang Plains':   {'pos': (800, 2600),  'type': 'plains'},
    'Gogra Heights':    {'pos': (2000, 2200), 'type': 'ridge'},
    'Finger 4 Ridge':   {'pos': (3200, 2400), 'type': 'ridge'},
    'Chang Chenmo':     {'pos': (4200, 1500), 'type': 'river'},
    'Karakoram Pass':   {'pos': (600, 3000),  'type': 'pass'},
}

# Mountain peaks (obstacles the drone must fly around)
LAC_PEAKS = [
    {'name': 'Peak 5490',        'x': 1000, 'y': 500,  'radius': 120, 'elevation': 5490},
    {'name': 'Saser Kangri',     'x': 500,  'y': 1800, 'radius': 150, 'elevation': 5600},
    {'name': 'Ridge Alpha',      'x': 2200, 'y': 700,  'radius': 100, 'elevation': 5200},
    {'name': 'Peak 5180',        'x': 3000, 'y': 1400, 'radius': 110, 'elevation': 5180},
    {'name': 'Karakoram Spur',   'x': 1500, 'y': 2800, 'radius': 130, 'elevation': 5350},
    {'name': 'Glacier Point',    'x': 3600, 'y': 600,  'radius': 90,  'elevation': 5100},
    {'name': 'Ridge Bravo',      'x': 4300, 'y': 2200, 'radius': 100, 'elevation': 5250},
    {'name': 'Peak 4980',        'x': 2800, 'y': 2600, 'radius': 85,  'elevation': 4980},
]

# No-fly zones (Chinese military installations)
LAC_NO_FLY = [
    {'name': 'PLA Post Alpha',   'x': 3500, 'y': 1000, 'radius': 80},
    {'name': 'PLA Post Bravo',   'x': 4500, 'y': 2800, 'radius': 90},
    {'name': 'PLA Radar Site',   'x': 1800, 'y': 1500, 'radius': 70},
]

# Indian FOB (home base)
LAC_HOME = Point(300, 1700)


def generate_lac_sector(seed=42):
    """
    Generate a SurveillanceMap representing a Ladakh border sector.
    Returns (surveillance_map, heightmap_grid, landmarks_dict).
    """
    WIDTH = 5000.0
    HEIGHT = 3500.0
    RESOLUTION = 20.0  # 20m per cell for larger area

    # Create base map
    smap = SurveillanceMap(
        width=WIDTH,
        height=HEIGHT,
        resolution=RESOLUTION,
        start_position=LAC_HOME,
        obstacle_margin=30.0,
        no_fly_margin=60.0,
    )

    # Add mountain peak obstacles
    for peak in LAC_PEAKS:
        obs = Obstacle(
            center=Point(peak['x'], peak['y']),
            radius=peak['radius'],
            name=peak['name'],
            is_no_fly=False,
        )
        smap.add_obstacle(obs)

    # Add no-fly zones
    for nfz in LAC_NO_FLY:
        obs = Obstacle(
            center=Point(nfz['x'], nfz['y']),
            radius=nfz['radius'],
            name=nfz['name'],
            is_no_fly=True,
        )
        smap.add_obstacle(obs)

    # Generate heightmap
    heightmap = generate_heightmap(WIDTH, HEIGHT, RESOLUTION, seed=seed)

    # Raise elevation under mountain peaks
    cols = int(WIDTH / RESOLUTION)
    rows = int(HEIGHT / RESOLUTION)
    for peak in LAC_PEAKS:
        cx = int(peak['x'] / RESOLUTION)
        cy = int(peak['y'] / RESOLUTION)
        r_cells = int((peak['radius'] + 100) / RESOLUTION)
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < cols and 0 <= ny < rows:
                    dist = math.sqrt(dx*dx + dy*dy) * RESOLUTION
                    if dist < peak['radius'] + 100:
                        # Smooth falloff
                        t = 1.0 - dist / (peak['radius'] + 100)
                        boost = t * t * (peak['elevation'] - 4000)
                        heightmap[ny][nx] = max(heightmap[ny][nx],
                                                4000 + boost)

    # Build landmarks export dict
    landmarks = {}
    for name, info in LAC_LANDMARKS.items():
        landmarks[name] = {
            'x': info['pos'][0],
            'y': info['pos'][1],
            'type': info['type'],
        }

    return smap, heightmap, landmarks
