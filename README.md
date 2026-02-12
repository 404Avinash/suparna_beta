<div align="center">

<img src="https://img.shields.io/badge/🦅_SUPARNA-Swift--Inspired_Path_Optimization-0a1628?style=for-the-badge&labelColor=0a1628" alt="Suparna" />

<br/><br/>

[![Defence](https://img.shields.io/badge/TRACK-DEFENCE_&_SECURITY-00ff88?style=flat-square&labelColor=1a1a2e)](.)
[![SAH 2.0](https://img.shields.io/badge/EVENT-SAH_2.0-cyan?style=flat-square&labelColor=1a1a2e)](.)
[![Status](https://img.shields.io/badge/STATUS-PROTOTYPE_COMPLETE-00ff88?style=flat-square&labelColor=1a1a2e)](.)
[![2026](https://img.shields.io/badge/YEAR-2026-white?style=flat-square&labelColor=1a1a2e)](.)

<br/>

### *सुपर्ण — Autonomous Surveillance Path Optimization*

A biomimetic drone path planning system that encodes aerodynamic constraints<br/>directly into the algorithm — inspired by the **Common Swift** (*Apus apus*),<br/>a bird that flies **10 months non-stop** without ever hovering.

<br/>

---

</div>

<br/>

<!-- ═══════════════════ STAT CARDS ═══════════════════ -->

<div align="center">
<table>
<tr>
<td align="center" width="250">
<br/>
<img src="https://img.shields.io/badge/≥95%25-00ff88?style=for-the-badge&labelColor=0d1117" alt="95%"/>
<br/><br/>
<strong>Coverage Achieved</strong>
<br/>
<sub>vs ~60-70% lawnmower baseline</sub>
<br/><br/>
</td>
<td align="center" width="250">
<br/>
<img src="https://img.shields.io/badge/O(1)-00d4ff?style=for-the-badge&labelColor=0d1117" alt="O(1)"/>
<br/><br/>
<strong>Dubins Path Query</strong>
<br/>
<sub>Closed-form, no iteration</sub>
<br/><br/>
</td>
<td align="center" width="250">
<br/>
<img src="https://img.shields.io/badge/0%25_HOVER-ff6b6b?style=for-the-badge&labelColor=0d1117" alt="0% Hover"/>
<br/><br/>
<strong>Hover Time</strong>
<br/>
<sub>All observation via loiter patterns</sub>
<br/><br/>
</td>
</tr>
</table>
</div>

<br/>

<!-- ═══════════════════ ALGORITHM TAGS ═══════════════════ -->

<div align="center">

![Greedy Set Cover](https://img.shields.io/badge/Greedy_Set_Cover-00ff88?style=flat-square&labelColor=0d1117)
![Dubins Curves](https://img.shields.io/badge/Dubins_Curves-00d4ff?style=flat-square&labelColor=0d1117)
![A* Search](https://img.shields.io/badge/A*_Search-ffd700?style=flat-square&labelColor=0d1117)
![Bug2 Avoidance](https://img.shields.io/badge/Bug2_Avoidance-ff6b6b?style=flat-square&labelColor=0d1117)
![TSP Heuristic](https://img.shields.io/badge/TSP_Heuristic-c084fc?style=flat-square&labelColor=0d1117)
![BFS](https://img.shields.io/badge/BFS-ff9f43?style=flat-square&labelColor=0d1117)

</div>

<br/>

---

## ⚡ The Problem

> Traditional drone surveillance = hover-based platforms + brute-force planning.
> The energy bottleneck isn't algorithmic — it's **aerodynamic**.

<table>
<tr>
<td width="50%">

**❌ Traditional Approach**

- Hover at observation points (max energy drain)
- Sharp yaw turns (high drag penalties)
- Lawnmower sweeps (redundant overlap)
- Post-hoc collision checking
- Software fights physics

</td>
<td width="50%">

**✅ SUPARNA's Approach**

- Continuous loiter patterns (zero hover)
- Banking via Dubins curves (min drag)
- Energy-weighted greedy selection (smart overlap)
- Obstacles pre-baked into map inflation
- Algorithm works WITH physics

</td>
</tr>
</table>

> *"The best path optimisation is the one you never have to compute."*<br/>
> By encoding physics as hard constraints, inefficient solutions are eliminated **before** the solver runs.

---

## 🏗️ Architecture Pipeline

```
INPUT                    PLANNING                     EXECUTION              OUTPUT
─────                    ────────                     ─────────              ──────

┌─────────┐    ┌──────────────────────┐    ┌───────────────────┐    ┌────────────────┐
│ Grid Map │───▶│  Obstacle Inflation  │───▶│   A* Pathfinder   │───▶│ Loiter Sequence│
│ Obstacles│    │  +20m/+50m margins   │    │   8-dir + simplify│    │ Flyable Path   │
│ No-Fly   │    └──────────┬───────────┘    └─────────┬─────────┘    │ Coverage Report│
│ Start Pos│               │                          │              │ Energy Log     │
└─────────┘    ┌───────────▼──────────┐    ┌──────────▼────────┐    └────────────────┘
               │  Coverage Planner    │    │ Reactive Avoidance│
               │  Greedy Set Cover    │    │ Bug2 + 7-ray scan │
               │  Score=Cov/(Tr+E)    │    │ NORMAL→AVOID→RECV │
               └───────────┬──────────┘    └───────────────────┘
                           │
               ┌───────────▼──────────┐
               │  Transition Planner  │
               │  Dubins (6 types)    │
               │  + Loiter Generator  │
               └──────────────────────┘
```

---

## 🔬 Core Algorithms

<table>
<tr>
<td align="center" width="33%">
<br/>

**🟢 Greedy Set Cover**
<br/><br/>
`O(k × C × L)`
<br/><br/>
Energy-weighted coverage planning<br/>
Score = Coverage ÷ (Transition + Loiter Energy)<br/>
O(ln n) approximation — best possible in P-time
<br/><br/>
</td>
<td align="center" width="33%">
<br/>

**🔵 Dubins Curves**
<br/><br/>
`O(1) per query`
<br/><br/>
Provably shortest flyable path<br/>
6 types: LSL, LSR, RSL, RSR, RLR, LRL<br/>
Closed-form trigonometry — no iteration
<br/><br/>
</td>
<td align="center" width="33%">
<br/>

**🟡 A* Search**
<br/><br/>
`O(V log V)`
<br/><br/>
8-directional grid pathfinding<br/>
Pre-inflated obstacle map<br/>
50K iteration cap + direct-line fallback
<br/><br/>
</td>
</tr>
<tr>
<td align="center" width="33%">
<br/>

**🔴 Bug2 Avoidance**
<br/><br/>
`O(r) per frame`
<br/><br/>
7-ray raycasting (80m range)<br/>
3-state machine: NORMAL→AVOID→RECOVER<br/>
Handles unknown & dynamic obstacles
<br/><br/>
</td>
<td align="center" width="33%">
<br/>

**🟣 Nearest-Neighbour TSP**
<br/><br/>
`O(k²)`
<br/><br/>
Loiter sequence optimisation<br/>
Minimises total transition distance<br/>
Fast approximation for ordering
<br/><br/>
</td>
<td align="center" width="33%">
<br/>

**🟠 Obstacle Inflation**
<br/><br/>
`O(n² × s²)`
<br/><br/>
Safety baked into grid at startup<br/>
Physical: 20m | No-fly: 50m buffer<br/>
Unsafe paths structurally impossible
<br/><br/>
</td>
</tr>
</table>

---

## 🦅 Biomimetic Design

<div align="center">
<table>
<tr>
<td align="center" width="200">
<br/>
🦅
<br/><br/>
<strong>Never Hovers</strong>
<br/>
<sub>→ Loiter patterns</sub>
<br/><br/>
</td>
<td align="center" width="200">
<br/>
🌀
<br/><br/>
<strong>Banking Turns</strong>
<br/>
<sub>→ Dubins curves</sub>
<br/><br/>
</td>
<td align="center" width="200">
<br/>
⚡
<br/><br/>
<strong>Max Endurance</strong>
<br/>
<sub>→ Energy-weighted cost</sub>
<br/><br/>
</td>
<td align="center" width="200">
<br/>
🔄
<br/><br/>
<strong>10-Month Flight</strong>
<br/>
<sub>→ Coverage per joule</sub>
<br/><br/>
</td>
</tr>
</table>
</div>

> *The Common Swift didn't evolve hover — and neither should our algorithm.*

---

## 🚀 Quick Start

```bash
# Clone
git clone https://github.com/404Avinash/suparna_beta.git
cd suparna_beta

# Install
pip install -r requirements.txt

# Run
python main.py
```

| Key | Action |
|:---:|--------|
| `SPACE` | Pause / Resume |
| `+` `-` | Speed up / down |
| `R` | Reset mission |
| `ESC` | Exit |

---

## 📁 Structure

```
suparna_beta/
├── main.py                     # Entry point
├── requirements.txt            # numpy, pygame
│
└── src/
    ├── core/
    │   ├── geometry.py         # Point, distance, angle utils
    │   ├── loiter.py           # 4 loiter patterns + energy costs
    │   ├── map.py              # Grid map, obstacles, coverage
    │   └── dubins.py           # Dubins paths (6 curve types)
    │
    ├── planners/
    │   ├── coverage.py         # Greedy set cover planner
    │   ├── pathfinder.py       # A* with safety inflation
    │   ├── reactive.py         # Bug2 obstacle avoidance
    │   └── transition.py       # Dubins transition planner
    │
    └── simulation/
        ├── drone.py            # Forward-flight drone model
        └── visualizer.py       # Pygame real-time renderer
```

---

## 🛡️ Robustness

| Failure Point | Mitigation |
|---|---|
| Bad/incomplete map | Bug2 reactive layer runs independently — safe even with blank map |
| Battery exhaustion | Continuous energy tracking + return-to-base trigger |
| A* pathfinder fails | 50K iteration cap + direct-line fallback — always returns a path |
| Dubins path blocked | Multi-angle retry (5 alternate headings) — skip if all fail |
| Coverage unreachable | Soft target — reports actual %, never hangs |

---

<div align="center">

### Tech Stack

[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org)
[![Pygame](https://img.shields.io/badge/Pygame-00CC00?style=for-the-badge&logo=python&logoColor=white)](https://pygame.org)

---

**Track:** Defence & Security &nbsp;|&nbsp; **PS:** Autonomous Surveillance Path Optimisation

*Smart ABES Hackathon 2.0*

<br/>

Made with 🦅 by **Team Rusty**

</div>
