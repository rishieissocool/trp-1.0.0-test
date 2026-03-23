#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diamond Path Planner — A* over virtual waypoint nodes around obstacles.

  1. Place virtual nodes in a ring around each obstacle (outside clearance).
  2. Filter nodes that land inside another obstacle's clearance.
  3. Connect visible node pairs, then run A* from start to goal.
  4. Smooth the path by shortcutting unnecessary intermediate nodes.

Works with the existing Obstacle class from voronoi_planner.obstacle.
"""

import heapq
import numpy as np

from TeamControl.voronoi_planner.obstacle import Obstacle

# --- Tunable constants ---------------------------------------------------
CLEARANCE    = 100    # mm — min distance to maintain from obstacles
ROBOT_RADIUS = 90     # mm — robot body radius (180 mm diameter)
NODE_COUNT   = 8      # virtual nodes per obstacle ring
NODE_MARGIN  = -50    # mm — offset nodes away from clearance edge

FIELD_X = 9000
FIELD_Y = 6000
X_MIN, X_MAX = -4500, 4500
Y_MIN, Y_MAX = -3000, 3000


class DiamondPlanner:
    """A* path planner that places waypoint nodes around circular obstacles."""

    def __init__(self, obstacles, clearance=CLEARANCE):
        """
        Parameters
        ----------
        obstacles : list[Obstacle]
            All obstacles to plan around (opponents, teammates, etc).
            Do NOT include the planning robot itself.
        clearance : float
            Minimum distance to maintain from obstacles.
        """
        self.obstacles = obstacles
        self.clearance = clearance
        self.nodes = np.empty((0, 2))
        self._build_graph()

    def _build_graph(self):
        self.nodes = []

        for obs in self.obstacles:
            center = obs.centre()
            r = obs.radius + self.clearance - NODE_MARGIN

            angles = np.linspace(0, 2 * np.pi, NODE_COUNT, endpoint=False)
            ring = np.stack([np.cos(angles), np.sin(angles)], axis=1) * r + center

            ring[:, 0] = np.clip(ring[:, 0], X_MIN, X_MAX)
            ring[:, 1] = np.clip(ring[:, 1], Y_MIN, Y_MAX)

            for node in ring:
                inside_other = False
                for other in self.obstacles:
                    if np.linalg.norm(node - other.centre()) < (other.radius + self.clearance):
                        inside_other = True
                        break
                if not inside_other:
                    self.nodes.append(node)

        self.nodes = np.array(self.nodes) if self.nodes else np.empty((0, 2))

    def is_path_free(self, start, goal):
        """Check if a straight line between start and goal is collision-free."""
        for obs in self.obstacles:
            if obs.intersects_line(start, goal, self.clearance):
                return False
        return True

    def plan_path(self, start, goal):
        """Compute a collision-free path from start to goal.

        Returns a list of waypoints [start, ..., goal].
        The path is smoothed — unnecessary intermediate nodes are removed.
        """
        start = np.asarray(start, dtype=float)
        goal  = np.asarray(goal, dtype=float)

        # Handle goal inside an obstacle's clearance
        goal_inside = any(
            np.linalg.norm(goal - obs.centre()) < (obs.radius + self.clearance)
            for obs in self.obstacles
        )
        if goal_inside and len(self.nodes) > 0:
            best_node, best_dist = None, float("inf")
            for node in self.nodes:
                if any(np.linalg.norm(node - o.centre()) < (o.radius + self.clearance)
                       for o in self.obstacles):
                    continue
                d = np.linalg.norm(node - goal)
                if d < best_dist:
                    best_dist = d
                    best_node = node
            if best_node is not None:
                goal = best_node

        # Direct path?
        if self.is_path_free(start, goal):
            return [start, goal]

        if len(self.nodes) == 0:
            return [start, goal]

        # Build extended node list: [nodes..., start, goal]
        nodes_ext = np.vstack([self.nodes, start, goal])
        start_idx = len(nodes_ext) - 2
        goal_idx  = len(nodes_ext) - 1
        n_nodes   = len(self.nodes)

        adj = {i: [] for i in range(len(nodes_ext))}

        # Connect node pairs that can see each other
        for i in range(n_nodes):
            for j in range(i + 1, n_nodes):
                if self.is_path_free(nodes_ext[i], nodes_ext[j]):
                    adj[i].append(j)
                    adj[j].append(i)

        # Connect start and goal to visible nodes
        for i in range(n_nodes):
            if self.is_path_free(start, nodes_ext[i]):
                adj[start_idx].append(i)
                adj[i].append(start_idx)
            if self.is_path_free(goal, nodes_ext[i]):
                adj[goal_idx].append(i)
                adj[i].append(goal_idx)

        if self.is_path_free(start, goal):
            adj[start_idx].append(goal_idx)
            adj[goal_idx].append(start_idx)

        # A* search
        def h(a, b):
            return np.linalg.norm(a - b)

        open_heap = [(h(start, goal), 0.0, start_idx)]
        came_from = {}
        g_score = {start_idx: 0.0}

        while open_heap:
            _, g_curr, curr = heapq.heappop(open_heap)
            if curr == goal_idx:
                path = [goal]
                while curr in came_from:
                    curr = came_from[curr]
                    path.append(nodes_ext[curr])
                path.reverse()
                return self._smooth(path)

            for nb in adj[curr]:
                g_new = g_score[curr] + np.linalg.norm(nodes_ext[curr] - nodes_ext[nb])
                if nb not in g_score or g_new < g_score[nb]:
                    g_score[nb] = g_new
                    heapq.heappush(open_heap, (g_new + h(nodes_ext[nb], goal), g_new, nb))
                    came_from[nb] = curr

        return [start, goal]

    def _smooth(self, path):
        """Shortcut unnecessary intermediate waypoints.

        Greedily skip ahead: from path[i], find the furthest path[j]
        reachable by a straight line, then jump there.
        """
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Find the furthest visible point from path[i]
            best = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self.is_path_free(path[i], path[j]):
                    best = j
                    break
            smoothed.append(path[best])
            i = best
        return smoothed
