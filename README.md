<div align="center">

# 🦅 Project SUPARNA

### *सुपर्ण — Swift-Inspired Autonomous Surveillance Path Optimization*

[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org)
[![Pygame](https://img.shields.io/badge/Pygame-00CC00?style=for-the-badge&logo=python&logoColor=white)](https://pygame.org)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)](LICENSE)
[![Track](https://img.shields.io/badge/Track-Defence_&_Security-red?style=for-the-badge)]()

---

**A biomimetic approach to drone path planning that works WITH physics, not against it.**

*Inspired by the Common Swift (Apus apus) — a bird that flies 10 months non-stop without ever hovering.*

</div>

---

## 🎯 The Problem

> Existing drone surveillance systems rely on **hover-based platforms** and **brute-force path planning** — fighting aerodynamics with software instead of leveraging physics for efficiency.

Traditional approaches waste energy on:

- ❌ Hovering at observation points (most energy-expensive maneuver)
- ❌ Sharp yaw turns (high drag penalties)
- ❌ Lawnmower sweep patterns (redundant overlap, no intelligence)
- ❌ Post-hoc collision checking (plan first, validate later)

## 💡 Our Solution: Constraint Inversion

```
"The best path optimisation is the one you never have to compute."
```

Suparna **encodes aerodynamic constraints directly into the algorithm design**, structurally eliminating inefficient solutions before the solver even runs.

| Aspect | Traditional Planner | SUPARNA |
|--------|-------------------|---------|
| **Observation** | Hover & stare | Continuous loiter patterns |
| **Energy Model** | Fight gravity | Exploit forward-flight lift |
| **Turns** | Yaw corrections | Banking via Dubins curves |
| **Coverage** | Lawnmower sweep | Greedy Set Cover (energy-weighted) |
| **Obstacles** | Post-hoc checking | Pre-baked into map inflation |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────┐
│              🗺️  MAP PREPROCESSING              │
│   Obstacle inflation (20m) + No-fly zones (50m) │
└──────────────────────┬──────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────┐
│           📡  COVERAGE PLANNER                   │
│   Greedy Set Cover + Energy-Weighted Scoring     │
│   Score = Coverage / (Transition + LoiterEnergy) │
└──────────────────────┬──────────────────────────┘
                       │
              ┌────────┴────────┐
              ▼                 ▼
┌──────────────────┐  ┌──────────────────────────┐
│  🔄 LOITER GEN   │  │  ✈️  TRANSITION PLANNER   │
│  TIGHT|STD|WIDE  │  │  Dubins Curves (6 types) │
│  RACETRACK       │  │  Min turn radius enforced │
└──────────────────┘  └──────────────────────────┘
              │                 │
              └────────┬────────┘
                       ▼
┌─────────────────────────────────────────────────┐
│            🧭  A* PATHFINDER                     │
│   8-directional + obstacle inflation + simplify  │
└──────────────────────┬──────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────┐
│          🛡️  REACTIVE AVOIDANCE (Bug2)           │
│   7-ray raycasting → edge-following → rejoin     │
└──────────────────────┬──────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────┐
│           🎮  DRONE SIMULATION + VISUALIZER      │
│   Forward-flight physics + real-time Pygame      │
└─────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

```bash
# Clone the repository
git clone https://github.com/404Avinash/suparna_beta.git
cd suparna_beta

# Install dependencies
pip install -r requirements.txt

# Run the simulation
python main.py
```

## 🎮 Controls

| Key | Action |
|:---:|--------|
| `SPACE` | Pause / Resume simulation |
| `+` / `-` | Speed up / Slow down |
| `R` | Reset mission |
| `ESC` | Exit |

---

## 📁 Project Structure

```
suparna_beta/
│
├── 📄 main.py                    # Entry point — orchestrates everything
├── 📄 requirements.txt           # Dependencies (numpy, pygame)
│
└── 📂 src/
    ├── 📂 core/                   # Domain primitives
    │   ├── geometry.py            # Point, distance, angle utilities
    │   ├── loiter.py              # 4 loiter pattern definitions + energy costs
    │   ├── map.py                 # Grid map, obstacles, coverage tracking
    │   └── dubins.py              # Dubins path generation (6 curve types)
    │
    ├── 📂 planners/               # Planning algorithms
    │   ├── coverage.py            # Greedy set cover coverage planner
    │   ├── pathfinder.py          # A* with safety inflation
    │   ├── reactive.py            # Bug2 reactive obstacle avoidance
    │   └── transition.py          # Dubins transition planner
    │
    └── 📂 simulation/             # Execution & visualization
        ├── drone.py               # Forward-flight drone physics model
        └── visualizer.py          # Pygame real-time rendering
```

---

<details>
<summary><h2>🔬 Core Algorithms (click to expand)</h2></summary>

### 1. Greedy Set Cover — Coverage Planning

- Places loiters to maximize `Coverage ÷ Energy`
- Each loiter is a "set" of grid cells it covers
- NP-hard problem → greedy gives **O(ln n) approximation**
- Repeats until **≥95% of free area** is covered

### 2. Dubins Curves — Transition Planning

- **Provably shortest path** between two oriented configurations
- Evaluates all 6 path types: `LSL`, `LSR`, `RSL`, `RSR`, `RLR`, `LRL`
- Closed-form trigonometric computation → **O(1) per query**
- Every output path is **guaranteed flyable**

### 3. A* Search — Pathfinding

- 8-directional grid search with Euclidean heuristic
- Operates on **pre-inflated obstacle grid** (safety baked in)
- Capped at 50,000 iterations with direct-line fallback
- Post-processed with **line-of-sight simplification**

### 4. Modified Bug2 — Reactive Avoidance

- 7-ray raycasting scans 80m ahead
- 3-state machine: `NORMAL → AVOIDING → RECOVERING`
- Handles **unknown/dynamic obstacles** the planner didn't see
- Guarantees circumnavigation + path rejoin

### 5. Loiter Patterns — Core Innovation

| Type | Radius | Use Case |
|------|--------|----------|
| `TIGHT` | 50–100m | Target focus, detailed surveillance |
| `STANDARD` | 100–200m | General area surveillance |
| `WIDE` | 200–500m | Broad area scan, patrol |
| `RACETRACK` | Variable | Linear features (roads, borders) |

</details>

---

<details>
<summary><h2>📊 Complexity Analysis (click to expand)</h2></summary>

| Component | Time | Space |
|-----------|------|-------|
| Coverage Planning | O(k × C × L) | O(n²) |
| Dubins Path | **O(1)** closed-form | O(W) waypoints |
| A* Pathfinder | O(V log V) | O(V) |
| Obstacle Inflation | O(n² × s²) | O(n²) |
| Reactive Avoidance | O(r) per frame | O(1) |
| Loiter Sequence TSP | O(k²) | O(k) |

**Key optimization:** The no-hover + min turn radius constraints **structurally collapse the search space** — inefficient solutions are eliminated before the solver runs, not penalized during search.

</details>

---

## 🦅 Why Swift-Inspired?

<div align="center">

| Swift Trait | Engineering Equivalent |
|------------|------------------------|
| Never hovers | All observation via loiter patterns |
| Extreme endurance | Energy-weighted cost function |
| Banking turns | Dubins curves with min turn radius |
| 10-month non-stop flight | Design for max coverage per joule |

</div>

> *"Aerodynamic-first design collapses the optimisation search space. The Common Swift didn't evolve hover — and neither should our algorithm."*

---

## 🛡️ Robustness Guarantees

- **Safety is structural** — obstacles inflated before planning; unsafe paths can't be generated
- **Physics is hardcoded** — Dubins curves guarantee every turn is flyable
- **Every layer has a fallback** — A* fails → direct line; Dubins blocked → retry angles; unknown obstacle → Bug2
- **Works with bad maps** — reactive layer runs independently of planned path
- **Modular & swappable** — replace any module without breaking the pipeline

---

<div align="center">

**Track:** Defence & Security | **PS:** Autonomous Surveillance Path Optimisation

*Built for SAH 2.0 — Smart ABES Hackathon*

---

Made with ❤️ by Team ALGORYTHM

</div>
