# SUPARNA Project - Study Guide & Technical Roadmap

This document breaks down the technical concepts, algorithms, and technologies used in **Project SUPARNA**. Use this to explain the project during reviews or interviews.

## 1. Core Algorithms Used

### A. Greedy Set Cover (Optimization)

**Where used**: `src/planners/coverage.py`

- **Problem**: How to cover a large complex shape (the map) using the fewest number of circles (loiter patterns). This is an NP-hard problem.
- **Solution**: We use a "Greedy Approximation Algorithm".
    1. Find the point on the map that covers the *most* new uncovered area.
    2. Place a loiter circle there.
    3. Mark that area as covered.
    4. Repeat until 98% of the map is covered.
- **Key Concept**: Optimization, NP-Hard Problems, Approximation Algorithms.

### B. Dubins Curves (Vehicle Kinematics)

**Where used**: `src/core/dubins.py`

- **Problem**: A fixed-wing drone cannot turn on a dime (like a car or quadcopter). It has a minimum turning radius.
- **Solution**: Calculate the shortest path between two points with a specific heading using circular arcs and straight lines.
- **Types**: There are 6 types (LSL, RSR, LSR, RSL, LRL, RLR) - e.g., Left-Straight-Left.
- **Key Concept**: Non-holonomic constrains, Path Smoothing, Differential Constraints.

### C. A* (A-Star) Pathfinding (Search)

**Where used**: `src/planners/pathfinder.py`

- **Problem**: Finding a collision-free path from point A to B through a grid of obstacles.
- **Solution**: A graph search algorithm that uses a heuristic (distance to goal) to guide the search faster than Dijkstra's algorithm.
- **Key Concept**: Graph Theory, Heuristics, Grid Search.

### D. Perlin Noise (Procedural Generation)

**Where used**: `src/core/lac_sector.py`

- **Problem**: Creating a realistic mountain terrain without manually sculpting every peak.
- **Solution**: Combining multiple layers ("octaves") of smooth random noise.
  - Large noise = mountain ranges.
  - Medium noise = ridges/valleys.
  - Small noise = rocky texture.
- **Key Concept**: Fractal Geometry, Noise Functions, Heightmaps.

### E. Computational Geometry

**Where used**: `src/core/geometry.py`, `src/core/map.py`

- **Problem**: Detecting if a path hits an obstacle or if a point is inside a polygon.
- **Solution**:
  - **Ray Casting / Line Integration**: Checking intersection between line segments and circles.
  - **Point-in-Polygon**: Checking if a coordinate is inside a no-fly zone.
- **Key Concept**: Vector Math, Collision Detection.

---

## 2. Technology Stack

### Backend (Python)

- **Language**: Python 3.10+
- **`numpy`**: Used for the 2D grid map. It handles the "coverage grid" (a large matrix where 0=uncovered, 1=covered) very efficiently.
- **`argparse`**: Parsing command line arguments (like `--seed` or `--map`).

### Frontend (Web/JS)

- **HTML5/CSS3**: Layout and UI overlay.
- **Three.js (WebGL)**: A library that simplifies writing WebGL (3D graphics) in the browser.
  - **Scene Graph**: Managing objects (meshes, lights, cameras).
  - **Geometry & Materials**: Creating the 3D shapes (cones for mountains, torus for loiters).
  - **Animation Loop**: Updating the drone position 60 times per second.

---

## 3. Study Roadmap (What to learn next)

If you want to master these topics, follow this order:

### Phase 1: The Basics (Grid Search)

- **Learn**: Breadth-First Search (BFS), Depth-First Search (DFS), Dijkstra's Algorithm.
- **Practice**: Implement A* on a simple 2D grid maze.

### Phase 2: Optimization (Set Cover)

- **Learn**: Greedy algorithms, NP-Completeness (basic concept).
- **Practice**: Try to solve the "Art Gallery Problem" (placing guards to see the whole room).

### Phase 3: Kinematics (How things move)

- **Learn**: Turning radius, Holonomic vs Non-holonomic vehicles (car vs omni-wheel robot).
- **Practice**: Write a simple simulator where a car drives towards a target with limited steering angle (Dubins paths).

### Phase 4: Procedural Generation (Terrain)

- **Learn**: Perlin Noise, Simplex Noise, Fractal Brownian Motion (fBM).
- **Practice**: Generate a 2D cloud texture using noise in Python or JavaScript.

### Phase 5: 3D Graphics (Three.js)

- **Learn**: coordinate systems (X, Y, Z), Meshes, Vertices, Shaders.
- **Practice**: Create a spinning cube in Three.js and add a light source.

---

## 4. Key Terminology for Interviews

- **"Non-holonomic"**: The system has constraints on its movement (e.g., a car cannot move sideways directly).
- **"Heuristic"**: A rule of thumb (like straight-line distance) used to speed up search algorithms.
- **"NP-Hard"**: A problem that cannot be solved perfectly in reasonable time; we use approximations.
- **"Discretization"**: Breaking a continuous world (real life) into a grid (matrix) for computers to process.
