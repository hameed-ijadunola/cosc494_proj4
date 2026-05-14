# COSC 494 Project 4 — Deliverables Checklist

Student: Hameed Ijadunola

Status legend: `[x]` complete · `[ ]` not yet done · `[?]` needs verification

---

## Q1. Roadmap Construction

### Q1.1 Halton Sampling — [src/proj4/proj4/samplers.py](src/proj4/proj4/samplers.py)

- [X] `compute_sample` implemented via radical inverse function ([samplers.py:36-62](src/proj4/proj4/samplers.py#L36-L62))
- [X] Index shift `k ← k+1` applied ([samplers.py:50](src/proj4/proj4/samplers.py#L50))
- [X] Unique prime base per dimension (primes = [2, 3, 5, 7, 11, 13]) ([samplers.py:29](src/proj4/proj4/samplers.py#L29))
- [X] `sample` linearly scales to workspace extents ([samplers.py:91-95](src/proj4/proj4/samplers.py#L91-L95))
- [X] Plot generated: [1.1.png](1.1.png)
- [X] Run command: `python3 ./src/proj4/proj4/plot_roadmap.py --num-vertices 100 --lazy`

### Q1.2 State Validity Checking — [src/proj4/proj4/problems.py](src/proj4/proj4/problems.py)

- [X] `PlanarProblem.check_state_validity` implemented ([problems.py:35-95](src/proj4/proj4/problems.py#L35-L95))
- [X] Strict boundary check against `self.extents` ([problems.py:59-62](src/proj4/proj4/problems.py#L59-L62))
- [X] Maps continuous states to grid indices via `world_to_map` then casts to int ([problems.py:68-84](src/proj4/proj4/problems.py#L68-L84))
- [X] Queries `permissible_region[yi, xi]` for occupancy ([problems.py:88](src/proj4/proj4/problems.py#L88))
- [X] Vectorized batch input (operates on N×D arrays, no Python loops)
- [X] Plot generated: [1.2.png](1.2.png)
- [X] Run command: `python3 ./src/proj4/proj4/plot_roadmap.py --text-map ./src/proj4/maps/map1.txt --num-vertices 100 --lazy`

### Q1.3 Edge Validity Checking — [src/proj4/proj4/roadmap.py](src/proj4/proj4/roadmap.py), [src/proj4/proj4/problems.py](src/proj4/proj4/problems.py)

- [X] `R2Problem.steer` interpolates with `x(α) = (1-α)q1 + α q2` ([problems.py:172-211](src/proj4/proj4/problems.py#L172-L211))
- [X] `Roadmap.check_weighted_edges_validity` removes invalid edges ([roadmap.py:73-98](src/proj4/proj4/roadmap.py#L73-L98))
- [X] Calls `check_edge_validity` for each candidate edge; only collision-free rows preserved
- [X] Boolean mask correctly typed (recent fix in commit `6bfd950`)
- [X] Multiple edges handled — iterates over full `weighted_edges` array
- [X] Plot generated: [1.3.png](1.3.png)
- [X] Run command: `python3 ./src/proj4/proj4/plot_roadmap.py -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --show-edges`

---

## Q2. Graph Search

### Q2.1 A* Search — [src/proj4/proj4/search.py](src/proj4/proj4/search.py)

- [X] Priority queue (`PriorityQueue`) used for OPEN list ([search.py:48-58](src/proj4/proj4/search.py#L48-L58))
- [X] g(n) cost-to-come tracked per `QueueEntry` ([search.py:113](src/proj4/proj4/search.py#L113))
- [X] h(n) heuristic via `rm.heuristic(neighbor, goal)` ([search.py:98](src/proj4/proj4/search.py#L98))
- [X] f = g + h priority computation ([search.py:117](src/proj4/proj4/search.py#L117))
- [X] Parent pointers tracked via `parents` array ([search.py:44, 89](src/proj4/proj4/search.py#L44))
- [X] `extract_path` backtraces start→goal cleanly via parents ([search.py:129-152](src/proj4/proj4/search.py#L129-L152))
- [X] Plot generated: [2.1.png](2.1.png) — path length 12.96 ([2.1.txt](2.1.txt))
- [X] Run command: `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --show-edges r2 -s 1 1 -g 7 8`

### Q2.2 Lazy A* — [src/proj4/proj4/search.py](src/proj4/proj4/search.py)

- [X] Edge collision check deferred until pop ([search.py:66-86](src/proj4/proj4/search.py#L66-L86))
- [X] Uses `Roadmap.check_edge_validity(parent, node)` when entry is popped
- [X] Discards entry on collision and continues with next-best entry
- [X] Skips check when parent is `NULL` (start node)
- [X] Plot generated: [2.2.png](2.2.png) — path length 12.96 ([2.2.txt](2.2.txt))
- [X] Run command: `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --lazy --show-edges r2 -s 1 1 -g 7 8`

### Q2.3 Path Shortcutting — [src/proj4/proj4/search.py](src/proj4/proj4/search.py)

- [X] Two non-adjacent indices selected at random ([search.py:176-182](src/proj4/proj4/search.py#L176-L182))
- [X] Direct segment collision-checked via `rm.check_edge_validity` ([search.py:187](src/proj4/proj4/search.py#L187))
- [X] Replacement only when shortcut is shorter than original segment ([search.py:190-194](src/proj4/proj4/search.py#L190-L194))
- [X] Plots generated: [2.3.png](2.3.png), [2.3_1.png](2.3_1.png) ([2.3.txt](2.3.txt))
- [X] Run command: `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --lazy --shortcut --show-edges r2 -s 1 1 -g 7 8`

---

## Q3. Kinematics & Dubins

### Q3.1 SE2 / Dubins Path Planning — [src/proj4/proj4/dubins.py](src/proj4/proj4/dubins.py)

- [X] Inputs converted to NumPy arrays ([dubins.py:40-41](src/proj4/proj4/dubins.py#L40-L41))
- [X] Translate goal relative to start ([dubins.py:51](src/proj4/proj4/dubins.py#L51))
- [X] Rotate into start's local frame using `rotation_matrix(start_θ)` ([dubins.py:57-58](src/proj4/proj4/dubins.py#L57-L58))
- [X] Local goal heading = `θ_g - θ_s` ([dubins.py:61](src/proj4/proj4/dubins.py#L61))
- [X] `path_planning_from_origin` invoked in local frame ([dubins.py:68-73](src/proj4/proj4/dubins.py#L68-L73))
- [X] Inverse rotation back to global frame ([dubins.py:80-84](src/proj4/proj4/dubins.py#L80-L84))
- [X] Heading angles re-summed and normalized via `pi_2_pi` to (-π, π] ([dubins.py:87-90](src/proj4/proj4/dubins.py#L87-L90))
- [X] Real Dubins path length returned (length / curvature) ([dubins.py:96](src/proj4/proj4/dubins.py#L96))
- [X] State-space and resulting path plots: [3.1.png](3.1.png), [3.1_1.png](3.1_1.png), [3.1_1_2.png](3.1_1_2.png) ([3.1.txt](3.1.txt))
- [X] Run commands:
  - `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt -n 40 -r 4 --lazy --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3`
  - `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt -n 40 -r 4 --lazy --shortcut --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3`

---

## Q4. ROS 2 Navigation Video

- [X] Screen recording exists: [Screencast from 2026-05-13 16-44-09.webm](Screencast%20from%202026-05-13%2016-44-09.webm)

- [?] Verify video clearly shows the **Map** in RViz
- [?] Verify video clearly shows the **Initial Pose**
- [?] Verify video clearly shows the **Goal Pose** (set via 2D Goal Pose tool)
- [?] Verify video clearly shows the **Planned Path** overlay
- [?] Verify the vehicle is moving in **Autonomous Mode** (R1 bumper held / `p` toggled to AUTO)

- [ ] Optional: convert `.webm` → `.mp4` if rubric strictly requires mp4 format
- [X] Launch command in submission notes: `ros2 launch proj4 launch_car_sim_proj4.py`

---

## Administrative Requirements

### README & Documentation

- [ ] **README missing** — no `README.md` exists at project root or in `src/proj4/`. Required to include:
  - [ ] Student name (Hameed Ijadunola)
  - [ ] Reproducible terminal commands for each question (Q1.1 → Q5.1)
  - [ ] Short technical explanation per module (`samplers.py`, `problems.py`, `roadmap.py`, `search.py`, `dubins.py`, `rrt.py`, `planner_node.py`)
  - [ ] List of known issues / specific parameters used (e.g., `num_vertices`, `connection_radius`, `curvature` from `launch_car_sim_proj4.py`)

### Submission Bundle

- [?] Verify all plots are bundled with the code submission (1.1, 1.2, 1.3, 2.1, 2.2, 2.3, 2.3_1, 3.1, 3.1_1, 3.1_1_2, 5.1)
- [?] Verify ROS 2 video is bundled
- [?] Verify final code is committed to git (current branch: `main`; `rubric.md` is currently untracked)

---

## Q5. Extra Credit — RRT — [src/proj4/proj4/rrt.py](src/proj4/proj4/rrt.py)

- [X] Tree initialized at start configuration via `RRTTree` ([rrt.py:47, 53](src/proj4/proj4/rrt.py#L47))
- [X] Random sampling with goal bias ([rrt.py:65](src/proj4/proj4/rrt.py#L65))
- [X] Nearest-neighbor search via `tree.GetNearestVertex` ([rrt.py:69](src/proj4/proj4/rrt.py#L69))
- [X] Incremental extension with step size ε via `extend` ([rrt.py:74](src/proj4/proj4/rrt.py#L74))
- [X] Collision check on both new state and edge ([rrt.py:90-91](src/proj4/proj4/rrt.py#L90-L91))
- [X] Vertex/edge added to tree on success; goal criterion checked ([rrt.py:96-104](src/proj4/proj4/rrt.py#L96-L104))
- [X] Path extracted by walking back from goal via `tree.edges` ([rrt.py:113-123](src/proj4/proj4/rrt.py#L113-L123))
- [X] Plot generated: [5.1.png](5.1.png)
- [X] Run command: `python3 ./src/proj4/proj4/run_search.py -m ./src/proj4/maps/map1.txt --algorithm rrt -x 1000 -e 0.5 -b 0.05 --show-edges r2 -s 1 1 -g 8 7`

---

## Outstanding Action Items (to reach "Excellent")

1. [X] Write `README.md` at project root with student name, per-question reproducible commands, per-module technical explanations, known issues, and parameter notes.
2. [ ] Re-watch the screen recording and confirm all five visual elements (map, init pose, goal pose, planned path, autonomous-mode motion) are clearly visible; re-record if any are missing.
3. [ ] (Optional) Convert `Screencast from 2026-05-13 16-44-09.webm` → `.mp4` for compatibility with the rubric's stated format.
4. [ ] Commit `rubric.md` (currently untracked) and any other outstanding files before final submission.
