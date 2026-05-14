# Project 4 Instruction Manual: Motion Planning & Navigation

## Overview
This document serves as the ground-truth technical specification for Project 4. To achieve an **Excellent** rating, all implementations must adhere to the specific constraints and validation logic outlined below.

---

## 1. Sampling & Validity
### Q1.1 Halton Sampling
* **Mechanism:** Implement using a radical inverse function.
* **Constraints:**
    * Shift $k$ by 1 ($k+1$).
    * Use unique prime bases for each dimension.
    * Scale the samples precisely to the defined workspace bounds.
* **Output:** Must produce a clean plot of the sampled distribution.

### Q1.2 State Validity Checking
* **Logic:**
    * Perform strict boundary checking.
    * Map continuous states to grid indices for obstacle lookup.
    * Query the `permissible_region` for occupancy.
* **Requirement:** Function **must** support batch (vectorized) inputs for efficiency.

### Q1.3 Edge Validity Checking
* **Logic:** Use interpolation between two nodes.
* **Constraints:**
    * Check all intermediate states for collisions.
    * Ensure invalid edges are completely removed from the roadmap.
    * Correctly handle and validate multiple edges within the graph.

---

## 2. Search Algorithms
### Q2.1 A* Search
* **Components:** Priority queue, $g(n)$ cost tracking, $h(n)$ heuristic calculation, and parent pointers.
* **Success Criteria:** Path extraction must accurately backtrack from the goal to the start without loops or breaks.

### Q2.2 Lazy A*
* **Mechanism:** Defer edge validity checks.
* **Logic:** Only validate an edge when it is popped from the priority queue. If invalid, discard it and continue the search from the next best node.

### Q2.3 Path Shortcutting
* **Algorithm:**
    * Randomly select two non-adjacent points on the existing path.
    * Perform a collision check for the direct line segment between them.
    * Replace the segment only if the shortcut is shorter than the original path length.

---

## 3. Kinematics & ROS 2
### Q3.1 SE2 / Dubins Planning
* **Coordinate Transforms:** Implement local-to-global and global-to-local transformations using rotation matrices.
* **Math:** Calculate relative headings and ensure strict **angle normalization** (e.g., mapping to $[-\pi, \pi]$).
* **Verification:** Provide plots for both the state space and the resulting path.

### Q4 ROS 2 Navigation Video
* **Content:** A video submission demonstrating the full stack.
* **Visual Requirements:** Must clearly show the Map, Initial/Goal Poses, Planned Path, and the vehicle moving in **Autonomous Mode**.

---

## 4. Administrative Requirements
### README & Documentation
* Include student name and clear, reproducible terminal commands.
* Provide short technical explanations for each module.
* List any known issues or specific parameters.

### Q5 Extra Credit: RRT
* **Logic:** Implement a tree-growth strategy using nearest-neighbor search and incremental extensions.
* **Validation:** Collision-check the extension step and extract the path once the goal region is reached.