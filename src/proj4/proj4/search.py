"""Implementation of A* and shortcutting."""

import collections
import numpy as np
import networkx as nx

from itertools import count
from proj4.utils import PriorityQueue

# Entries in the PriorityQueue are prioritized in lexicographic order, i.e.  by
# the first element of QueueEntry (the f-value). Ties are broken by proceeding
# to the next element, a unique counter. This prevents the underlying
# PriorityQueue from breaking ties by comparing nodes. QueueEntry objects
# contain the node, a possible parent, and cost-to-come via that parent. These
# can be accessed via dot notation, e.g. `entry.node`.
QueueEntry = collections.namedtuple(
    "QueueEntry",
    ["f_value", "counter", "node", "parent", "cost_to_come"],
)

NULL = -1


def astar(rm, start, goal):
    """Compute the shortest path from start to goal on a roadmap.

    Args:
        rm: Roadmap instance to plan on
        start, goal: integer labels for the start and goal states

    Returns:
        vpath: a sequence of node labels, including the start and goal
    """
    if start not in rm.graph or goal not in rm.graph:
        msg = "Either start {} or goal {} is not in G"
        raise nx.NodeNotFound(msg.format(start, goal))

    # expanded is a Boolean array that tracks whether each node has previously
    # been expanded, in order to avoid expanding nodes multiple times.
    expanded = np.zeros(rm.num_vertices, dtype=bool)

    # parents is an int array that tracks the best parent for each node. It
    # defaults to NULL (-1) for each node.
    parents = NULL * np.ones(rm.num_vertices, dtype=int)

    c = count()

    queue = PriorityQueue()

    queue.push(
        QueueEntry(
            rm.heuristic(start, goal),
            next(c),
            start,
            NULL,
            0,
        )
    )

    while len(queue) > 0:
        entry = queue.pop()

        if expanded[entry.node]:
            continue

        if rm.lazy:
            # Collision check the edge between the parent and this node (if a
            # parent exists). If it's in collision, stop processing this entry;
            # we'll wait for another possible parent later in the queue.

            # BEGIN QUESTION 2.2
            # In Lazy A*, edges are NOT collision-checked when the roadmap is built.
            # We defer the check to the first time we would "use" an edge — i.e.,
            # when we are about to expand the node reached via that edge.
            #
            # Only check when this node has an actual parent (the start node has none).
            if entry.parent != NULL:
                # Collision-check the edge connecting the parent to the current node.
                # check_edge_validity interpolates states along the edge and verifies
                # each one lies inside the permissible region.
                if not rm.check_edge_validity(entry.parent, entry.node):
                    # This edge passes through an obstacle.  Discard the queue entry
                    # and continue with the next-best entry — there may be another
                    # collision-free path to this node through a different parent.
                    continue
            # END QUESTION 2.2

        expanded[entry.node] = True
        parents[entry.node] = entry.parent

        if entry.node == goal:
            path = extract_path(parents, goal)
            return path, parents

        for neighbor, w in rm.graph[entry.node].items():
            # Get the edge weight (length) and heuristic value
            weight = w.get("weight")
            h = rm.heuristic(neighbor, goal)

            # Compute this neighbor's cost-to-come via entry.node, and insert a
            # new QueueEntry.
            #
            # Since this may be Lazy A*, it is misleading to compare the f-value
            # against any current QueueEntry objects for this neighbor that are
            # already in the queue. Those may turn out to be in collision and
            # therefore unusable. Therefore, this entry should be inserted even
            # if it seems more costly than other entries already in the queue.
            #
            # However, if the neighbor has already been expanded, it's no longer
            # necessary to insert this QueueEntry.
            # BEGIN QUESTION 2.1
            if not expanded[neighbor]:
                cost_to_come = entry.cost_to_come + weight

                queue.push(
                    QueueEntry(
                        cost_to_come + h,
                        next(c),
                        neighbor,
                        entry.node,
                        cost_to_come,
                    )
                )
            # END QUESTION 2.1

    raise nx.NetworkXNoPath("Node {} not reachable from {}".format(goal, start))


def extract_path(parents, goal):
    """Extract the shortest path from start to goal.

    Args:
        parents: np.array of integer node labels
        goal: integer node label for the goal state

    Returns:
        vpath: a sequence of node labels from the start to the goal
    """
    # Follow the parents of the node until a NULL entry is reached
    vpath = []

    # BEGIN QUESTION 2.1
    current = goal

    while current != NULL:
        vpath.append(current)
        current = parents[current]

    vpath.reverse()
    # END QUESTION 2.1

    return vpath


def shortcut(rm, vpath, num_trials=100):
    """Shortcut the path between the start and goal.

    Args:
        rm: Roadmap instance to plan on
        vpath: a sequence of node labels from the start to the goal
        num_trials: number of random trials to attempt

    Returns:
        vpath: a subset of the original vpath that connects vertices directly
    """
    for _ in range(num_trials):
        if len(vpath) == 2:
            break

        # Randomly select two indices to try and connect. Verify that an edge
        # connecting the two vertices would be collision-free, and that
        # connecting the two indices would actually be shorter.
        #
        # You may find these Roadmap methods useful: check_edge_validity,
        # heuristic, and compute_path_length.
        indices = np.random.choice(len(vpath), size=2, replace=False)

        i, j = np.sort(indices)

        # BEGIN QUESTION 2.3
        if j <= i + 1:
            continue

        u = vpath[i]
        v = vpath[j]

        if not rm.check_edge_validity(u, v):
            continue

        old_length = rm.compute_path_length(vpath[i : j + 1])
        new_length = rm.heuristic(u, v)

        if new_length < old_length:
            vpath = vpath[: i + 1] + vpath[j:]
        # END QUESTION 2.3

    return vpath
