# COSC494 Project 4 - Path Planning Implementation Plan

## Context
Implement missing components of a sampling-based motion planning system for a MuSHR autonomous car. The project uses Probabilistic Roadmaps (PRM) with A* search, Dubins paths, and RRT. Several question stubs exist across 5 files.

**Note:** Q1.1 (Halton sampling), Q2.1 (A* neighbor expansion + extract_path), and Q2.3 (shortcutting) are already fully implemented in the skeleton. Only the items below need to be written.

---

## Files to Modify

| File | Questions |
|------|-----------|
| `src/proj4/proj4/problems.py` | Q1.2, Q1.3 |
| `src/proj4/proj4/roadmap.py` | Q1.3 |
| `src/proj4/proj4/search.py` | Q2.2 |
| `src/proj4/proj4/dubins.py` | Q3.1 |
| `src/proj4/proj4/rrt.py` | Q5.1 |

---

## Implementation Details

### Q1.2 — `PlanarProblem.check_state_validity` (problems.py:54–74)

Two separate blocks to fill between the existing `# BEGIN` / `# END` markers.

**Block 1** — extents check (before `world_to_map` is called):
```python
# Invalidate any state whose x coordinate falls outside [x_min, x_max].
# self.extents[0, 0] is the lower bound and self.extents[0, 1] is the upper
# bound for the x dimension (in world/meter units when map_info is set,
# or in pixel units for plain text maps).
valid &= (x >= self.extents[0, 0]) & (x <= self.extents[0, 1])

# Same check for the y dimension using self.extents[1, :].
valid &= (y >= self.extents[1, 0]) & (y <= self.extents[1, 1])
```

**Block 2** — permissible region check (after `world_to_map` converts states to pixel coords):
```python
# After world_to_map the x and y columns of `states` now hold pixel indices.
# Extract the pixel column (x) and row (y) for only the still-valid states
# so we never index out-of-bounds on states that failed the extents check.
xi = states[valid, 0].astype(int)   # column index into the occupancy grid
yi = states[valid, 1].astype(int)   # row index (height dimension comes first)

# permissible_region is a 2-D Boolean array shaped (height, width).
# Index as [row, col] = [y_pixel, x_pixel].
# Update the valid mask: a state is free only if its grid cell is permissible.
valid[valid] &= self.permissible_region[yi, xi]
```

---

### Q1.3a — `R2Problem.steer` (problems.py:175–177)

Replace the stub (between `# BEGIN QUESTION 1.3` and `# END QUESTION 1.3`):
```python
# Euclidean distance between the two R^2 configurations.
length = np.linalg.norm(q2 - q1)

if not interpolate_line or length == 0:
    # When interpolation is disabled or the two points are identical,
    # return just the endpoints — no intermediate states needed.
    path = np.array([q1, q2])
else:
    # Compute the number of evenly-spaced waypoints so that consecutive
    # states are at most `resolution` apart.  We always need at least
    # 2 points (start and end), hence the max(..., 2).
    num_steps = max(int(np.ceil(length / resolution)), 2)

    # alpha sweeps from 0 (at q1) to 1 (at q2).
    alphas = np.linspace(0, 1, num_steps)

    # Linear interpolation: x(alpha) = (1 - alpha)*q1 + alpha*q2.
    # np.outer produces an (num_steps,) array of scaled q1 and q2 vectors.
    path = np.outer(1 - alphas, q1) + np.outer(alphas, q2)

# `return path, length` already exists at line 179 — do NOT duplicate it.
```

---

### Q1.3b — `Roadmap.check_weighted_edges_validity` (roadmap.py:86–88)

Replace the stub (between `# BEGIN QUESTION 1.3` and `# END QUESTION 1.3`):
```python
# For each candidate edge (u, v, edge_length) in the input array,
# call check_edge_validity which internally calls problem.check_edge_validity.
# check_edge_validity uses R2Problem.steer to interpolate states along the
# straight-line segment and then checks every interpolated state for collisions.
valid_mask = np.array([
    self.check_edge_validity(int(u), int(v))
    for u, v, _ in weighted_edges   # `_` is the pre-computed edge length
])

# Return only the rows (edges) that passed collision checking.
# weighted_edges has shape (num_edges, 3); boolean indexing keeps valid rows.
return weighted_edges[valid_mask]
```

---

### Q2.2 — Lazy A* edge check (search.py:71–74)

Insert inside the existing `if rm.lazy:` block, **before** `expanded[entry.node] = True`:
```python
# In Lazy A*, edges are NOT collision-checked when the roadmap is built.
# Instead, we check each edge the first time it would be "used" — i.e.,
# when we are about to expand the node that was reached via that edge.
#
# Only check if this node actually has a parent (the start node does not).
if entry.parent != NULL:
    # Collision-check the edge that connects the parent to the current node.
    # check_edge_validity interpolates states along the edge and verifies
    # each one is in the permissible region.
    if not rm.check_edge_validity(entry.parent, entry.node):
        # This edge passes through an obstacle.  Discard the queue entry
        # and continue with the next-best entry — there may be another
        # path to this node through a collision-free edge.
        continue
```

---

### Q3.1 — `dubins.path_planning` (dubins.py:36–83)

Replace the entire stub with:
```python
# Step 1: Ensure start and end are NumPy float arrays so arithmetic works.
start = np.array(start, dtype=float)   # [sx, sy, s_theta]
end   = np.array(end,   dtype=float)   # [ex, ey, e_theta]

# Step 2: Transform the goal into the local coordinate frame of the start.
#
# The Dubins solver (path_planning_from_origin) assumes the start state
# is always at (0, 0, 0).  We convert the goal accordingly.
#
# 2a. Translate: express goal position relative to start position.
p_prime = end[:2] - start[:2]         # shape (2,), global translation vector

# 2b. Rotate: align with start orientation so start's x-axis points forward.
#     utils.rotation_matrix(theta) returns a (2, 2) rotation matrix R(theta).
#     Multiplying a row vector on the left: v_local = v_global @ R(theta)
#     rotates the vector INTO the local frame.
R = utils.rotation_matrix(start[2])   # (2, 2) rotation by start heading
p_local = np.matmul(p_prime, R)       # goal position in start's local frame

# 2c. The goal heading in the local frame is simply the difference in headings.
local_goal = np.array([p_local[0], p_local[1], end[2] - start[2]])

# Step 3: Plan the Dubins path from the origin to the local goal.
#   `length` here is a *normalized* cost (sum of arc/line segment angles,
#   dimensionless when the turning radius is 1).
path, mode, length = path_planning_from_origin(
    local_goal,
    curvature,
    resolution=resolution,
    interpolate_line=interpolate_line,
)

# Step 4: Transform the computed path back into the global frame.
#
# 4a. Rotate: apply the inverse rotation R(-theta_s) to undo the local frame.
R_inv = utils.rotation_matrix(-start[2])   # (2, 2) rotation by -start heading
path[:, :2] = np.matmul(path[:, :2], R_inv)

# 4b. Translate: shift from start-relative position to global position.
path[:, :2] += start[:2]

# 4c. Add the start heading back to every waypoint's heading.
path[:, 2] += start[2]

# 4d. Wrap all heading angles into (-pi, pi] to avoid discontinuities.
pi_2_pi(path[:, 2])

# Step 5: Convert the normalized Dubins cost to a real-world arc length.
#   The turning radius is 1/curvature, so real_length = normalized_length / curvature.
real_path_length = length * (1.0 / curvature)

return path, real_path_length
```

---

### Q5.1 — `rrt` main loop (rrt.py:60–72)

**First BEGIN QUESTION 5 block** — sampling and extension:
```python
# 1. Draw a random sample from the configuration space.
#    With probability `bias` the goal itself is returned (goal biasing),
#    otherwise a uniformly random free configuration is sampled.
#    `sample` returns shape (1, D).
x_rand = sample(rm, goal_config, bias)

# 2. Find the nearest vertex already in the RRT tree.
#    GetNearestVertex returns (vertex_id, vertex_config) where
#    vertex_config has shape (1, D).
x_near_id, x_near = tree.GetNearestVertex(x_rand)

# 3. Extend from x_near TOWARD x_rand by a fixed step size eta.
#    extend computes: x_new = x_near + eta * (x_rand - x_near)
#    This limits how far the tree grows in one iteration.
x_new = extend(x_near, x_rand, eta)   # shape (1, D)
```

**Second BEGIN QUESTION 5 block** — validity check and tree update:
```python
# 1. Only add x_new to the tree if BOTH conditions hold:
#    a) x_new itself is a collision-free configuration, AND
#    b) the straight-line edge from x_near to x_new is collision-free.
#
#    check_state_validity expects shape (N, D) → pass x_new directly.
#    check_edge_validity expects 1-D arrays → index [0] to drop batch dim.
if (rm.problem.check_state_validity(x_new).all() and
        rm.problem.check_edge_validity(x_near[0], x_new[0])):

    # 2. Add the new valid configuration as a vertex in the tree and
    #    record the directed edge from x_near to x_new.
    #    RRT does not track path cost during expansion, so cost = 0.
    x_new_id = tree.AddVertex(x_new, cost=0)
    tree.AddEdge(x_near_id, x_new_id)

    # 3. Check whether x_new satisfies the goal criterion
    #    (within goal_thresh distance of the goal configuration).
    #    If so, record the goal node ID and stop expanding.
    if rm.problem.goal_criterion(goal_config, x_new).all():
        x_goal_id = x_new_id
        break
```

---

## Verification Commands

Run in order to test each question:

```bash
# Q1.1 — Halton sampling without map (already implemented, sanity check)
python3 ./src/proj4/proj4/plot_roadmap.py --num-vertices 100 --lazy

# Q1.2 — State validity with text map
python3 ./src/proj4/proj4/plot_roadmap.py \
    --text-map ./src/proj4/maps/map1.txt \
    --num-vertices 100 --lazy

# Q1.3 — Edge validity + steer
python3 ./src/proj4/proj4/plot_roadmap.py \
    -m ./src/proj4/maps/map1.txt \
    -n 25 -r 3.0 --show-edges

# Q2.1 — Eager A*
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --show-edges r2 -s 1 1 -g 7 8

# Q2.2 — Lazy A*
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --lazy --show-edges r2 -s 1 1 -g 7 8

# Q2.3 — Shortcutting
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt -n 25 -r 3.0 --lazy --shortcut --show-edges r2 -s 1 1 -g 7 8

# Q3.1 — SE2/Dubins lazy A*
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt -n 40 -r 4 --lazy --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3

# Q3.1 — SE2/Dubins with shortcutting
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt -n 40 -r 4 --lazy --shortcut --show-edges se2 -s 1 1 0 -g 7 8 45 -c 3

# Q5.1 — RRT
python3 ./src/proj4/proj4/run_search.py \
    -m ./src/proj4/maps/map1.txt --algorithm rrt -x 1000 -e 0.5 -b 0.05 --show-edges r2 -s 1 1 -g 8 7
```
