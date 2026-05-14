# COSC 494 Project 4 — Path Planning

**Student:** Hameed Ijadunola
**Course:** COSC 494/594 — Fall 2026
**Instructor:** Fei Liu

This project implements a sampling-based motion planner: Halton roadmap construction,
state/edge collision checking, A* and Lazy A* search, path shortcutting, Dubins
SE(2) planning, an RRT extra-credit planner, and a ROS 2 navigation demo on the
MuSHR car simulator.

---

## Repository Layout

```
src/proj4/proj4/
├── samplers.py       # Halton / Lattice / Random samplers
├── problems.py       # PlanarProblem, R2Problem, SE2Problem (validity, steer)
├── roadmap.py        # Roadmap construction & edge validity
├── search.py         # A*, Lazy A*, path shortcutting
├── dubins.py         # SE(2) Dubins path planning wrapper
├── rrt.py            # Extra-credit Rapidly-exploring Random Tree
├── plot_roadmap.py   # CLI: visualize a roadmap
├── run_search.py     # CLI: run A*/Lazy A*/RRT and plot the path
└── planner_node.py   # ROS 2 planner node
src/proj4/launch/launch_car_sim_proj4.py   # ROS 2 simulation launch
src/proj4/maps/                             # Text and YAML maps
```

Submission artifacts (plots / video) live at the repo root: `1.1.png` … `5.1.png`,
plus `Screencast from 2026-05-13 16-44-09.webm`.

---

## Build & Source

This is a ROS 2 workspace. From the repo root:

```bash
colcon build --packages-select proj4
source install/setup.bash
```

The non-ROS planning code (Q1–Q3, Q5) only needs `numpy`, `networkx`, and
`matplotlib`; it can be run directly with `python3` once the `proj4` package is
on `PYTHONPATH` (sourcing `install/setup.bash` after `colcon build` is the
simplest way).

---

## Reproducible Commands

### Q1.1 — Halton Sampling
```bash
python3 ./src/proj4/proj4/plot_roadmap.py --num-vertices 100 --lazy
```
Output: [1.1.png](1.1.png)

### Q1.2 — State Validity Checking
```bash
python3 ./src/proj4/proj4/plot_roadmap.py \
    --text-map ./src/proj4/maps/map1.txt \
    --num-vertices 100 --lazy
```
Output: [1.2.png](1.2.png)

### Q1.3 — Edge Validity Checking
```bash
python3 ./src/proj4/proj4/plot_roadmap.py \
    -m ./src/proj4/maps/map1.txt \
    -n 25 -r 3.0 --show-edges
```
Output: [1.3.png](1.3.png)

### Q2.1 — A* Search
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    -n 25 -r 3.0 --show-edges r2 -s 1 1 -g 7 8
```
Output: [2.1.png](2.1.png), log: [2.1.txt](2.1.txt) — path length **12.96**

### Q2.2 — Lazy A*
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    -n 25 -r 3.0 --lazy --show-edges r2 -s 1 1 -g 7 8
```
Output: [2.2.png](2.2.png), log: [2.2.txt](2.2.txt) — path length **12.96**

### Q2.3 — Path Shortcutting
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    -n 25 -r 3.0 --lazy --shortcut --show-edges r2 -s 1 1 -g 7 8
```
Output: [2.3.png](2.3.png), [2.3_1.png](2.3_1.png), log: [2.3.txt](2.3.txt)

### Q3.1 — SE(2) / Dubins (Lazy A*)
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    -n 40 -r 4 --lazy --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3
```
Output: [3.1.png](3.1.png), [3.1_1.png](3.1_1.png)

With shortcutting:
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    -n 40 -r 4 --lazy --shortcut --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3
```
Output: [3.1_1_2.png](3.1_1_2.png), log: [3.1.txt](3.1.txt) — path length **15.50**

### Q4 — ROS 2 Navigation
Three terminals:
```bash
# T1: simulation + planner + RViz
ros2 launch proj4 launch_car_sim_proj4.py

# T2: keyboard teleop bridge (press `p` to toggle AUTO mode)
ros2 run mushr_sim keyboard_teleop_terminal
```
Then in RViz, use **2D Goal Pose** to set a goal. Hold the controller's R1 bumper
(or press `p` in the teleop terminal) to switch to autonomous mode; the planner
publishes a path and the controller drives the car.

Recording: [Screencast from 2026-05-13 16-44-09.webm](Screencast%20from%202026-05-13%2016-44-09.webm)

### Q5 — Extra Credit RRT
```bash
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt \
    --algorithm rrt -x 1000 -e 0.5 -b 0.05 --show-edges r2 -s 1 1 -g 8 7
```
Output: [5.1.png](5.1.png)

---

## Module Notes

### `samplers.py` — Halton low-discrepancy sampler
`HaltonSampler.compute_sample(k, b)` implements the radical-inverse function
`H_b(k) = Σ a_i / b^(i+1)` with the project's `k ← k+1` shift so the zero index
does not collapse to the origin. Each configuration dimension uses a unique
prime base from `[2, 3, 5, 7, 11, 13]` (the first `dim` primes are dropped from
the front of the sequence to avoid early-term clustering). `sample(N)` linearly
scales the unit-cube samples to `extents`.

### `problems.py` — Validity, steering, and heuristics
- `PlanarProblem.check_state_validity` is fully vectorized: it first masks states
  outside `extents`, converts the survivors to pixel indices via `world_to_map`,
  then indexes `permissible_region[yi, xi]`. Boolean masking ensures we never
  index out-of-bounds with states that already failed the bounds check.
- `R2Problem.steer` produces straight-line interpolation `x(α) = (1-α)q1 + α q2`
  with `ceil(length / resolution)` waypoints (≥ 2). Returns `(path, length)`.
- `SE2Problem.steer` delegates to `dubins.path_planning`.

### `roadmap.py` — Graph construction
`check_weighted_edges_validity` filters the candidate edge list by calling
`check_edge_validity` (which itself runs `steer` then `check_state_validity` on
the interpolated states). The boolean mask is explicitly typed `dtype=bool` so
NumPy boolean indexing keeps only valid rows. Lazy mode skips this step at
construction and defers the check to search time.

### `search.py` — A*, Lazy A*, and shortcutting
Standard A* with a `PriorityQueue` ordered by `f = g + h`. `QueueEntry` carries
`(f_value, counter, node, parent, cost_to_come)`. Tie-breaking by a unique
counter prevents NumPy node comparisons inside the heap.
- **Lazy A*** (`if rm.lazy`): when an entry is popped, the parent→node edge is
  collision-checked via `Roadmap.check_edge_validity`. Invalid entries are
  discarded and the search continues with the next best one.
- `extract_path` walks parent pointers from the goal back to the start
  (`current = parents[current]` until `NULL`) and reverses.
- `shortcut(rm, vpath)` picks two non-adjacent indices `(i, j)`, accepts the
  direct connection only if it is collision-free **and** strictly shorter than
  the original `vpath[i:j+1]`, then replaces the segment with the direct edge.

### `dubins.py` — SE(2) Dubins wrapper
`path_planning(start, end, curvature, ...)` transforms the goal into the start's
local frame (translate by `-start[:2]`, rotate by `R(start_θ)`, subtract the
start heading), invokes `path_planning_from_origin`, then transforms the
resulting waypoints back to global coordinates and adds the start heading. All
heading angles are wrapped into (-π, π] via `pi_2_pi`. The returned length is
the real arc length: `normalized_length / curvature`.

### `rrt.py` — Rapidly-exploring Random Tree (extra credit)
Each iteration: sample (with goal bias `b`), find nearest tree vertex, extend by
step size ε, validate both the new state and the connecting edge, then add to
the tree. Termination when `goal_criterion` is satisfied. The path is recovered
by walking `tree.edges` backwards from the goal node. Cost is set to `0` per the
project spec; total path length is computed afterwards from the recovered
configurations.

### `planner_node.py` + `launch_car_sim_proj4.py` — ROS 2 integration
The launch file spins up `nav2_map_server`, the MuSHR simulator and fake VESC,
RViz, the controller from project 3, and `planner_node` from this project. The
planner subscribes to a goal pose and publishes a planned path that the
controller follows in autonomous mode.

Default planner parameters (in `launch_car_sim_proj4.py`):

```python
{
    "num_vertices":    500,
    "connection_radius": 10.0,
    "curvature":         1.0,
    "cache_roadmap":     False,
}
```

---

## Known Issues & Notes

- **Video format.** The Q4 submission is currently a `.webm` screen recording.
  The rubric example references `.mp4`; convert with
  `ffmpeg -i "Screencast from 2026-05-13 16-44-09.webm" q4_navigation.mp4`
  if a strict MP4 is required.
- **`np.outer` interpolation.** In `R2Problem.steer`, `np.outer` is used to
  produce a `(num_steps, D)` linear interpolation. This works for any
  `D`-dimensional `q1, q2`, but R2Problem only ever calls it with `D = 2`.
- **`check_resolution`.** Edge collision checks use `PlanarProblem.check_resolution`
  (default `0.1` for R2, `0.01` for SE(2)). Smaller values increase fidelity but
  slow down validity checks proportionally.
- **Halton burn-in.** The first `primes[dim]` Halton terms are skipped at
  construction (`self.index = self.primes[self.dim]`) to avoid early-sequence
  clustering, as recommended for higher-prime bases.
- **Lazy roadmap visualization.** Edges drawn under `--lazy` may visibly cross
  obstacles — they are unchecked at construction and only validated lazily
  during search. This is expected and matches the rubric's note for Q2.2.
- **RRT determinism.** `rrt.py` seeds `np.random.seed(0)` so plots are
  reproducible across runs.

---

## File Index of Submitted Plots

| Question | Plot(s) |
|---|---|
| Q1.1 | [1.1.png](1.1.png) |
| Q1.2 | [1.2.png](1.2.png) |
| Q1.3 | [1.3.png](1.3.png) |
| Q2.1 | [2.1.png](2.1.png) |
| Q2.2 | [2.2.png](2.2.png) |
| Q2.3 | [2.3.png](2.3.png), [2.3_1.png](2.3_1.png) |
| Q3.1 | [3.1.png](3.1.png), [3.1_1.png](3.1_1.png), [3.1_1_2.png](3.1_1_2.png) |
| Q4   | [Screencast from 2026-05-13 16-44-09.webm](Screencast%20from%202026-05-13%2016-44-09.webm) |
| Q5.1 | [5.1.png](5.1.png) |
