# SUPARNA Project - Complete Architecture Guide

## Overview

**SUPARNA** (सुपर्ण) is an **autonomous drone surveillance path planning system** inspired by the Swift bird. Unlike traditional drones that hover at observation points (energy-intensive), SUPARNA uses **continuous loiter patterns** and **energy-aware path planning** to maximize coverage while minimizing energy consumption.

### Key Innovation
- **0% Hover Time**: Observation happens via circular loiter patterns, not static hovering
- **95%+ Coverage**: vs ~60-70% with traditional lawnmower patterns
- **Non-holonomic Constraints**: Paths respect real fixed-wing aircraft turning radius
- **Biomimetic Design**: Inspired by the Common Swift which flies non-stop for 10 months

---

## 1. Project Structure

```
a_b/
├── main.py                    # Demo entry point (Pygame visualization)
├── export_mission.py          # Generate mission & export JSON
├── README.md                  # Project overview & badges
├── STUDY_GUIDE.md            # Algorithm overview (original)
├── PROJECT_ARCHITECTURE.md   # This file - full system explanation
├── requirements.txt          # Dependencies
├── src/
│   ├── core/                 # Geometric & map algorithms
│   │   ├── dubins.py        # Dubins path generation
│   │   ├── geometry.py      # Point, collision, math utilities
│   │   ├── loiter.py        # Loiter pattern definitions
│   │   ├── map.py           # Surveillance map representation
│   │   ├── lac_sector.py    # LAC border terrain generation
│   │   └── random_map.py    # Random map generation
│   ├── planners/             # Mission planning algorithms
│   │   ├── coverage.py      # Greedy Set Cover planner (MAIN)
│   │   ├── pathfinder.py    # A* grid pathfinding
│   │   ├── reactive.py      # Real-time obstacle avoidance
│   │   └── transition.py    # Dubins path connections
│   └── simulation/           # Visualization & simulation
│       ├── drone.py         # Drone state machine
│       ├── visualizer.py    # Pygame visualization
│       └── __init__.py      # run_demo() entry
└── web/                      # 3D web visualization (optional)
    ├── index.html
    └── mission.json
```

---

## 2. Core Workflow: From Idea to Flight

### Step 1: Map Generation → `generate_random_map()` or `generate_lac_sector()`

**Location**: `src/core/random_map.py`, `src/core/lac_sector.py`

**What happens**:
1. Create a 2D grid map (e.g., 1000m × 700m)
2. Place obstacles (circles representing mountains, buildings, no-fly zones)
3. Define start position (home/base)
4. Return a `SurveillanceMap` object

**Example**:
```python
# Random map: 1000m × 700m with random obstacles
smap = generate_random_map(width=1000, height=700, seed=42)

# Or LAC border terrain with realistic peaks
smap, heightmap, landmarks = generate_lac_sector(seed=42)
```

**Key Data Structure** (`SurveillanceMap`):
```python
@dataclass
class SurveillanceMap:
    width: float                    # Map width in meters
    height: float                   # Map height in meters
    start_position: Point           # Home/launch point
    obstacles: List[Obstacle]       # Mountains, no-fly zones
    coverage_grid: np.ndarray       # 2D grid: 0=uncovered, 1=covered
    resolution: float               # Grid cell size
```

---

### Step 2: Coverage Planning → `CoveragePlanner.plan()`

**Location**: `src/planners/coverage.py`

**Problem**: How to cover the entire map using the fewest loiter circles?

**Algorithm**: **Greedy Set Cover** (NP-hard, but ~80% optimal for discrete coverage)

**How it works**:

```
1. Initialize: coverage_grid = all zeros (everything uncovered)

2. LOOP until 98% covered:
   a. For each possible loiter position:
      - Simulate: If we place a loiter circle here, how much NEW area is covered?
      - Score: new_coverage / energy_cost
   
   b. Pick loiter with best score
   c. Place it on the map
   d. Mark its coverage area as "covered" in the grid
   e. Check energy budget (don't exceed max_loiters)

3. Return: List of optimal loiter positions
```

**Example**:
```python
planner = CoveragePlanner(
    surveillance_map=smap,
    loiter_type=LoiterType.STANDARD,
    loiter_radius=70.0,        # Circle radius (observation range)
    turn_radius=40.0,           # Minimum turning radius
    overlap_factor=0.25,        # 25% overlap between loiters
    coverage_threshold=98.0,    # Stop at 98% coverage
    max_loiters=50,             # Max loiter circles allowed
)

mission = planner.plan(start_position=smap.start_position)
# mission.loiters → List[Loiter] (optimal positions)
```

**Key Concept**: Loiter Pattern

A **loiter** is a circular flight pattern where the drone flies in circles at a constant altitude:

```python
@dataclass
class Loiter:
    center: Point          # Circle center (lat, lon)
    radius: float          # Circle radius in meters
    loiter_type: LoiterType
    
    @property
    def coverage_area(self) -> float:
        """Area covered by sensor (πr²)"""
    
    @property
    def total_distance(self) -> float:
        """Total flight distance for N revolutions"""
```

**Why loiter instead of hover?**
- **Hover**: Quadcopter = max energy drain, no forward speed
- **Loiter**: Fixed-wing drone = continuous motion, natural pattern, sensor can track targets

---

### Step 3: Path Connection → `connect_loiters()` (Dubins Paths)

**Location**: `src/core/dubins.py`

**Problem**: Connect loiter circles with smooth, feasible paths respecting turning radius

**Dubins Path**: Optimal path between two configurations (position + heading) for a vehicle with **minimum turning radius**

**Path Types** (6 possible combinations):
- **LSL**: Left-Straight-Left (two left turns with straight segment)
- **LSR**: Left-Straight-Right
- **RSL**: Right-Straight-Left
- **RSR**: Right-Straight-Right
- **RLR**: Right-Left-Right (simple U-turn)
- **LRL**: Left-Right-Left

**Visual**:
```
         Loiter B
         (target heading)
        /
       /
     /← Dubins path
    /
  Loiter A
  (current heading)
```

**Algorithm**:
1. Calculate center of turning circles (left & right) at start loiter
2. Calculate center of turning circles at end loiter
3. Find tangent line between circles (6 possibilities)
4. Compute distances for each type
5. Return shortest valid path

**Code Example**:
```python
from src.core.dubins import DubinsPath, connect_loiters

# Connect two loiters
path = connect_loiters(
    loiter_a_center=Point(100, 100),
    loiter_a_heading=0.5,           # Heading in radians
    loiter_b_center=Point(300, 250),
    loiter_b_heading=1.2,
    turn_radius=40.0                # Fixed-wing constraint
)

print(f"Path type: {path.path_type}")  # e.g., LSR
print(f"Total length: {path.total_length}m")
print(f"Waypoints: {path.waypoints}")   # List[Point] to follow
```

---

### Step 4: Mission Assembly → `MissionPath`

**Location**: `src/planners/coverage.py`

After planning:
1. Arrange loiters in optimal order (using TSP heuristic)
2. Connect each pair with Dubins path
3. Add return path to home

**Data Structure**:
```python
@dataclass
class MissionPath:
    loiters: List[Loiter]                    # Observation positions
    transitions: List[Optional[DubinsPath]]   # Paths between loiters
    
    def get_all_waypoints(self) -> List[Point]:
        """Flatten into single sequence for the drone"""
        # Returns: Home → Loiter1 → Loiter2 → ... → Loiter N → Home
```

**Example Output**:
```
Waypoints:
  0: (50, 50)    [HOME]
  1-50: Loiter 1 (circular pattern)
  51-100: Dubins transition path
  101-150: Loiter 2 (circular pattern)
  ...
  Last: (50, 50)  [RETURN HOME]
```

---

## 3. Simulation: Drone State Machine

**Location**: `src/simulation/drone.py`

The drone executes the mission through a state machine:

### Drone States:
```
┌─────────┐
│  IDLE   │ (on ground, not moving)
└────┬────┘
     │ set_mission() called
     ↓
┌─────────┐      ┌──────────────┐
│ FLYING  │──→   │  LOITERING   │ (circular observation pattern)
│         │←─    │              │
└────┬────┘      └──┬───────────┘
     │              │
     │ waypoint reached,
     │ no more loiters
     ↓
┌──────────┐
│RETURNING │ (fly back to home)
└────┬─────┘
     │ reached home
     ↓
┌────────┐
│LANDED  │
└────────┘
```

### Update Loop (runs 60x per second):

```python
def update(self, dt: float):
    """
    dt = time delta (e.g., 0.016 seconds for 60 FPS)
    """
    
    if state == LOITERING:
        # Update circular motion
        loiter_angle += (speed / loiter_radius) * dt
        new_position = loiter_center + radius * (cos(angle), sin(angle))
    else:  # FLYING or RETURNING
        # Turn toward target waypoint
        target_heading = calculate_heading_to(target)
        heading += min(turn_rate * dt, angular_error)
        
        # Move forward
        position += velocity * heading * dt
        
        # Check if reached waypoint
        if distance(position, target) < waypoint_radius:
            current_waypoint += 1
            if waypoint is loiter point:
                state = LOITERING
    
    # Energy management
    battery -= energy_rate * dt
    distance_traveled += speed * dt
    
    # Coverage tracking
    if state in (FLYING, LOITERING, RETURNING):
        coverage_points.append(position)  # For visualization
```

### Key Physics:

```python
@dataclass
class SimpleDrone:
    position: Point = (50, 50)
    heading: float = 0.0              # Radians, 0 = East
    speed: float = 30.0               # m/s (constant)
    turn_rate: float = 2.5            # rad/s max turn rate
    battery: float = 100.0            # Energy %
    energy_rate: float = 0.12         # Battery drain per second
```

---

## 4. Visualization Pipeline

### Entry Point: `src/simulation/visualizer.py` → Pygame

**What it displays**:
1. **Map**: Grid with obstacles (red) and no-fly zones (dark red)
2. **Drone**: Blue dot showing current position, heading (arrow)
3. **Path**: Trail of where drone has flown
4. **Loiter Circles**: Orange circles (pending) → Yellow (active) → Green (completed)
5. **Coverage**: Green overlay on map showing scanned areas
6. **HUD**: Battery %, distance, altitude, mission progress

**Code Flow**:
```python
visualizer = SuparnaVisualizer(width=1200, height=800)

# Create map and plan
smap = generate_random_map(...)
planner = CoveragePlanner(smap, ...)
mission = planner.plan(smap.start_position)

# Convert mission to drone-friendly format
drone = SimpleDrone()
drone.set_mission(
    path=mission.get_all_waypoints(),
    loiter_indices=[...]  # Which waypoints are loiter centers?
)

# Run simulation loop
while not done:
    dt = clock.tick(60) / 1000  # 60 FPS
    drone.update(dt)
    visualizer.render(drone, mission, smap)
```

---

## 5. Export for Web Visualization

**Location**: `export_mission.py`

Generates `web/mission.json` for 3D visualization (Three.js):

**JSON Structure**:
```json
{
  "map": {
    "width": 1000,
    "height": 700,
    "resolution": 10,
    "type": "random"
  },
  "home": {"x": 50, "y": 50},
  "obstacles": [
    {
      "x": 300,
      "y": 400,
      "radius": 80,
      "name": "Mountain A",
      "is_no_fly": true
    }
  ],
  "waypoints": [
    {"x": 50, "y": 50, "type": "home"},
    {"x": 250, "y": 300, "type": "loiter", "radius": 70, "index": 1},
    {"x": 500, "y": 250, "type": "loiter", "radius": 70, "index": 2},
    {"x": 50, "y": 50, "type": "return"}
  ]
}
```

---

## 6. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        INPUT: Environment                           │
│  ┌────────────────────┐  OR  ┌──────────────────────┐              │
│  │  Random Map Gen   │       │  LAC Sector Terrain  │              │
│  │  (flat obstacles) │       │ (realistic peaks)    │              │
│  └────────┬──────────┘       └──────────┬──────────┘              │
└───────────┼─────────────────────────────┼──────────────────────────┘
            │                             │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │   SurveillanceMap Object    │
            │ - Grid (1000×700m)          │
            │ - Obstacles (list)          │
            │ - Start position            │
            │ - Coverage grid (0/1)       │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │   CoveragePlanner (Greedy)  │
            │                             │
            │  1. For each grid cell:     │
            │     - Place loiter circle   │
            │     - Calculate coverage    │
            │  2. Pick best (max coverage │
            │     per energy)             │
            │  3. Repeat until 98%        │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │   List of Loiter Positions  │
            │  (centers + radii)          │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │   Dubins Path Connector     │
            │                             │
            │  For each pair of loiters:  │
            │  - Calculate min turn radius│
            │  - Generate smooth path     │
            │  - Collect waypoints        │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │   MissionPath Object        │
            │ - Loiters (List)            │
            │ - Transitions (Dubins)      │
            │ - Waypoints (flattened)     │
            └──────────────┬──────────────┘
                           ↓
            ┌─────────────────────────────┐
            │    SimpleDrone State        │
            │ - Position, heading         │
            │ - Waypoints, current index  │
            │ - Battery, speed            │
            │ - State machine (FLYING...) │
            └──────────────┬──────────────┘
                           ↓
          ┌────────────────────────────────┐
          │   Simulation Loop (60 FPS)     │
          │                                │
          │  Each frame:                   │
          │  1. dt = time_delta            │
          │  2. drone.update(dt)           │
          │  3. visualizer.render()        │
          │                                │
          │  Outputs:                      │
          │  - Pygame window (live view)   │
          │  - JSON export (web view)      │
          └────────────────────────────────┘
```

---

## 7. Key Algorithms in Detail

### 7.1 Greedy Set Cover (Coverage Planner)

**Problem Statement**: Cover a 2D region with minimum number of circles

**NP-Hard**: No known polynomial algorithm (but approximation works well)

**Greedy Approach**:
```
Uncovered = all cells in grid
Loiters = []
Energy = 0

WHILE uncovered > 0.02 * total_cells:  # Until 98% covered
    best_loiter = None
    best_score = -∞
    
    FOR each candidate position (x, y):
        loiter = create_loiter(x, y, radius=70)
        new_coverage = count_newly_covered_cells(loiter, Uncovered)
        energy_cost = compute_loiter_energy(loiter)
        
        score = new_coverage / energy_cost
        
        IF score > best_score:
            best_score = score
            best_loiter = loiter
    
    Loiters.append(best_loiter)
    Mark(best_loiter.coverage_area as covered)
    Energy += energy_cost

RETURN Loiters
```

**Approximation Ratio**: ~ln(n) where n = number of cells
**In Practice**: 95%+ coverage with reasonable loiter count

---

### 7.2 Dubins Paths (Continuous Curvature)

**Problem**: Shortest path with bounded curvature (min turn radius r)

**Formulation**:
```
Given:
  - Start: position (x₀, y₀), heading θ₀
  - End: position (x₁, y₁), heading θ₁
  - Min turn radius: r

Find: Shortest path respecting radius constraint

Path structure: Curve + Line + Curve
  - Curve 1: Turn left/right with radius r
  - Line: Straight segment
  - Curve 2: Turn left/right with radius r
```

**6 Candidate Paths**:
1. **LSL** (Left-Straight-Left): Two left arcs
2. **LSR** (Left-Straight-Right): Left arc, straight, right arc
3. **RSL** (Right-Straight-Left): Right arc, straight, left arc
4. **RSR** (Right-Straight-Right): Two right arcs
5. **RLR** (Right-Left-Right): Three arcs (tight turn)
6. **LRL** (Left-Right-Left): Three arcs (tight turn)

**Algorithm** (simplified):
```
FOR each path type in [LSL, LSR, RSL, RSR, RLR, LRL]:
    c_left_start = center of left turn at start
    c_right_start = center of right turn at start
    c_left_end = center of left turn at end
    c_right_end = center of right turn at end
    
    tangent_line = find_tangent(start_circle, end_circle)
    
    IF path_type == LSL:
        length = arc(c_left_start, ...) + line(...) + arc(..., c_left_end)
    ...
    
    IF is_valid(path):
        candidates.append((path_type, length))

RETURN candidate with minimum length
```

**Complexity**: O(1) - closed form solution!

---

### 7.3 A* Pathfinding (Obstacle Avoidance)

**Location**: `src/planners/pathfinder.py`

Used for local navigation around obstacles

**Algorithm**:
```
open_set = {start}
closed_set = {}
g_score = {start: 0}
f_score = {start: heuristic(start, goal)}

WHILE open_set not empty:
    current = node in open_set with lowest f_score
    
    IF current == goal:
        return reconstruct_path(current)
    
    open_set.remove(current)
    closed_set.add(current)
    
    FOR neighbor in neighbors(current):
        IF neighbor in closed_set:
            continue
        
        tentative_g = g_score[current] + distance(current, neighbor)
        
        IF neighbor not in open_set:
            open_set.add(neighbor)
        ELSE IF tentative_g >= g_score[neighbor]:
            continue  # Not a better path
        
        g_score[neighbor] = tentative_g
        f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

RETURN failure  # No path found
```

**Heuristic**: Euclidean distance (straight line to goal)

---

## 8. Configuration Parameters

### Map Generation
```python
# Random map
width = 1000           # meters
height = 700           # meters
resolution = 10        # cell size
seed = 42              # reproducibility

# LAC sector
seed = 7               # terrain variation
```

### Coverage Planner
```python
loiter_radius = 70.0        # Observation circle radius (m)
turn_radius = 40.0          # Min turning radius (m)
overlap_factor = 0.25       # 25% overlap between loiters
coverage_threshold = 98.0   # Stop at 98% coverage
max_loiters = 50            # Safety limit
```

### Drone Physics
```python
speed = 30.0           # Constant forward speed (m/s)
turn_rate = 2.5        # Max turn rate (rad/s)
energy_rate = 0.12     # Battery drain per second
battery = 100.0        # Starting battery %
```

### Visualization
```python
window_width = 1200    # Pygame window
window_height = 800
FPS = 60               # Refresh rate
```

---

## 9. Running the Project

### Option 1: Pygame Demo (Interactive)
```bash
python main.py
# Shows real-time Pygame visualization
# - Flying drone
# - Loiter circles
# - Coverage overlay
# - Battery/distance HUD
```

### Option 2: Export Mission (JSON)
```bash
# Generate random mission
python export_mission.py

# Generate LAC border mission
python export_mission.py --map lac --seed 42

# View output in web/mission.json
```

### Option 3: Web 3D Visualization
```bash
# Open web/index.html in browser
# Uses Three.js for 3D rendering
# - Terrain height map
# - 3D drone model
# - Camera following drone
```

---

## 10. Key Takeaways

| Aspect | Details |
|--------|---------|
| **Core Problem** | Cover surveillance area with minimum energy |
| **Main Algorithm** | Greedy Set Cover (loiter placement) |
| **Path Planning** | Dubins curves (fixed-wing constraint) |
| **Coverage Method** | Circular loiter patterns (not hover) |
| **Visualization** | Pygame (2D) + Three.js (3D web) |
| **Energy Model** | Continuous motion = lower energy than hover |
| **Obstacle Handling** | A* pathfinding + geometry collision detection |
| **State Machine** | IDLE → FLYING → LOITERING → RETURNING → LANDED |

---

## 11. Learning Path

To fully understand SUPARNA:

1. **Basics** (1-2 days)
   - Read: README.md + this file
   - Run: `python main.py`
   - Understand: Loiter patterns, basic drone motion

2. **Algorithms** (2-3 days)
   - Study: [STUDY_GUIDE.md](STUDY_GUIDE.md)
   - Code: `src/core/dubins.py` (Dubins paths)
   - Code: `src/planners/coverage.py` (Greedy set cover)

3. **Integration** (2-3 days)
   - Code: `src/simulation/drone.py` (state machine)
   - Code: `src/simulation/visualizer.py` (rendering)
   - Experiment: Modify parameters in `export_mission.py`

4. **Advanced** (3-5 days)
   - Implement: Your own coverage algorithm
   - Extend: Multi-drone coordination
   - Optimize: Energy efficiency improvements
   - Deploy: Real drone implementation

---

## 12. Common Questions

### Q: Why not just use A* for the whole problem?
**A**: A* finds collision-free paths, but not optimal coverage. Coverage requires optimization (Greedy Set Cover) + path planning (Dubins) as two separate steps.

### Q: What's the difference between loiter and hover?
**A**: 
- **Hover**: Drone stays in one spot (quadcopter). Energy-intensive.
- **Loiter**: Drone circles at a location (fixed-wing). More energy-efficient, continuous observation.

### Q: Why Dubins paths?
**A**: Fixed-wing drones can't turn on a dime. Dubins paths respect minimum turning radius and are provably optimal for that constraint.

### Q: How long does a mission take?
**A**: Depends on:
- Map size (1000m = ~10 minutes)
- Number of loiters (more = longer)
- Speed (30 m/s = 108 km/h default)
- Example: 20 loiters × 5 minutes loiter time = 100 minutes total

### Q: Can this handle dynamic obstacles?
**A**: Currently, no. Map is static. Future work: Real-time replanning with reactive avoidance.

---

**End of Guide**
