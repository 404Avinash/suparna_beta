# Project SUPARNA 🦅

> **सुपर्ण — Swift-Inspired Autonomous Surveillance Path Optimization**

A biomimetic approach to drone path planning that works with physics instead of fighting it.

## 🎯 Philosophy

Traditional drone path planners waste energy fighting aerodynamics with complex algorithms. SUPARNA flips the script:

**"Why fight battery, turns, hover penalties in software… when nature already solved it in hardware?"**

Inspired by the **Common Swift** (*Apus apus*), which flies 10 months non-stop, we design for:

- **No hover** — all surveillance via loiter patterns
- **Banking turns** — energy-efficient directional changes (Dubins curves)
- **Forward flight native** — continuous motion, never stationary

## 🚀 Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run the demo
python main.py
```

## 🎮 Controls

| Key | Action |
|-----|--------|
| `SPACE` | Pause/Resume simulation |
| `+` / `-` | Speed up/slow down |
| `R` | Reset mission |
| `ESC` | Exit |

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│          COVERAGE PLANNER               │
│  Greedy Set Cover + Energy Weights      │
│  Score = Coverage / (Transition + E)    │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│       LOITER PATTERN GENERATOR          │
│  TIGHT | STANDARD | WIDE | RACETRACK   │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│       TRANSITION PLANNER                │
│  Dubins Paths (6 optimal curve types)   │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│       A* PATHFINDER                     │
│  8-dir grid search + obstacle inflation │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│    REACTIVE AVOIDANCE (Bug2)            │
│  7-ray raycasting + edge following      │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│       PYGAME VISUALIZER                 │
│  Real-time drone + coverage display     │
└─────────────────────────────────────────┘
```

## 📁 Project Structure

```
├── src/
│   ├── core/
│   │   ├── geometry.py     # Point, distance, angle utilities
│   │   ├── loiter.py       # 4 loiter pattern definitions
│   │   ├── map.py          # Grid map with obstacles & no-fly zones
│   │   └── dubins.py       # Dubins path generation (6 path types)
│   │
│   ├── planners/
│   │   ├── coverage.py     # Greedy set cover coverage planner
│   │   ├── pathfinder.py   # A* pathfinder with safety inflation
│   │   ├── reactive.py     # Bug2 reactive obstacle avoidance
│   │   └── transition.py   # Dubins transition planner
│   │
│   └── simulation/
│       ├── drone.py        # Forward-flight drone physics model
│       └── visualizer.py   # Pygame real-time visualization
│
├── main.py                 # Entry point
└── requirements.txt        # Dependencies
```

## 🔬 Core Algorithms

| Algorithm | Purpose | Complexity |
|-----------|---------|------------|
| Greedy Set Cover | Coverage planning (energy-weighted) | O(k × C × L) |
| Dubins Curves | Shortest flyable transitions (6 types) | O(1) closed-form |
| A* Search | Obstacle-aware pathfinding | O(V log V) |
| Bug2 (Modified) | Reactive obstacle avoidance | O(r) per frame |
| Nearest-Neighbour | Loiter sequence optimization (TSP) | O(k²) |

## 🦅 Why Swift-Inspired?

The Common Swift achieves incredible efficiency through:

- **Extreme lift-to-drag ratio** → design for endurance, not sprint
- **Zero hover capability** → eliminate hover from the algorithm
- **Continuous forward flight** → persistent coverage without stationary phases
- **Banking turns** → Dubins curves with minimum turn radius

Our algorithm collapses the optimization search space by making these aerodynamic principles our constraints, not our enemies.

---

*"Aerodynamic-first design collapses the optimisation search space."*

**Track:** Defence & Security | **PS:** Autonomous Surveillance Path Optimisation
