#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Add src folder to sys.path
src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from TeamControl.voronoi_planner.obstacle import Obstacle

# --- Global variables ---
# Default values:
# CLEARANCE = 100
# ROBOT_RADIUS = 90
# NODE_COUNT = 8
# NODE_MARGIN = -50
CLEARANCE = 100 # space around the obstacle to avoid collision (PURPLE)
ROBOT_RADIUS = 90 # actual robot size, 180mm diameter/90mm radius (YELLOW)
NODE_COUNT = 8 # increase for fine tuning, decrease for faster calculations
NODE_MARGIN = -50  # offset nodes away from clearance to avoid touching obstacles
                   # -50 means 50mm away the clearance, a positive value would
                   # be inside the clearance

FIELD_X = 9000
FIELD_Y = 6000
X_MIN, X_MAX = -4500, 4500
Y_MIN, Y_MAX = -3000, 3000


class DiamondPlanner:
    def __init__(self, obstacles, starts, clearance=CLEARANCE):
        self.static_obstacles = obstacles
        self.starts = starts
        self.clearance = clearance
        self.nodes = []
        self.all_obstacles = []
        self.build_graph()

    def build_graph(self):
        self.nodes = []  # Clear nodes to regenerate them

        # Treat starts as obstacles
        start_obstacles = [
            Obstacle(tuple(s), ROBOT_RADIUS, 1000 + i, True)
            for i, s in enumerate(self.starts)
        ]
        
        self.all_obstacles = self.static_obstacles + start_obstacles

        for obs in self.all_obstacles:
            center = obs.centre()
            r = obs.radius + self.clearance - NODE_MARGIN  # offset inside clearance

            angles = np.linspace(0, 2*np.pi, NODE_COUNT, endpoint=False)
            ring_nodes = np.stack([np.cos(angles), np.sin(angles)], axis=1) * r + center

            # Clip nodes inside field
            ring_nodes[:, 0] = np.clip(ring_nodes[:, 0], X_MIN, X_MAX)
            ring_nodes[:, 1] = np.clip(ring_nodes[:, 1], Y_MIN, Y_MAX)

            # Remove nodes that fall within any other robot's clearance
            valid_nodes = []
            for node in ring_nodes:
                valid = True
                for other_obs in self.all_obstacles:
                    if np.linalg.norm(node - other_obs.centre()) < (other_obs.radius + self.clearance):
                        valid = False
                        break
                if valid:
                    valid_nodes.append(node)

            # Add valid nodes to the list
            self.nodes.extend(valid_nodes)

        self.nodes = np.array(self.nodes)

    def is_path_free(self, start, goal):
        for obs in self.all_obstacles:
            # Skip obstacle if start or goal is its center
            if np.allclose(obs.centre(), start) or np.allclose(obs.centre(), goal):
                continue
            if obs.intersects_line(start, goal, self.clearance):
                return False
        return True


    def plan_path(self, start, goal):
        import heapq

        # --- Check if goal is inside any clearance ---
        goal_inside_clearance = False
        for obs in self.all_obstacles:
            if np.linalg.norm(goal - obs.centre()) < (obs.radius + self.clearance):
                goal_inside_clearance = True
                break

        if goal_inside_clearance:
            # Replace goal with closest visible node outside all clearances
            closest_node = None
            closest_dist = float("inf")
            for node in self.nodes:
                # Check node is outside all obstacle clearances
                node_ok = True
                for obs in self.all_obstacles:
                    if np.linalg.norm(node - obs.centre()) < (obs.radius + self.clearance):
                        node_ok = False
                        break
                if not node_ok:
                    continue

                # Node must be visible from start
                if not self.is_path_free(start, node):
                    continue

                dist_to_goal = np.linalg.norm(node - goal)
                if dist_to_goal < closest_dist:
                    closest_dist = dist_to_goal
                    closest_node = node

            if closest_node is not None:
                goal = closest_node

        # If straight line is free → done
        if self.is_path_free(start, goal):
            return [start, goal]

        # Helper: Euclidean distance
        def heuristic(a, b):
            return np.linalg.norm(a - b)

        # Create temporary start/goal nodes
        nodes_extended = np.vstack([self.nodes, start, goal])
        start_idx = len(nodes_extended) - 2
        goal_idx = len(nodes_extended) - 1

        # Build adjacency list
        adjacency = {i: [] for i in range(len(nodes_extended))}
        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                if self.is_path_free(nodes_extended[i], nodes_extended[j]):
                    adjacency[i].append(j)
                    adjacency[j].append(i)

        # Connect start and goal to visible nodes
        for i in range(len(self.nodes)):
            if self.is_path_free(start, nodes_extended[i]):
                adjacency[start_idx].append(i)
                adjacency[i].append(start_idx)
            if self.is_path_free(goal, nodes_extended[i]):
                adjacency[goal_idx].append(i)
                adjacency[i].append(goal_idx)

        # Optionally connect start to goal directly
        if self.is_path_free(start, goal):
            adjacency[start_idx].append(goal_idx)
            adjacency[goal_idx].append(start_idx)

        # A* search
        open_heap = [(heuristic(start, goal), 0, start_idx)]
        came_from = {}
        g_score = {start_idx: 0}

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current == goal_idx:
                # Reconstruct path
                path = [goal]
                while current in came_from:
                    current = came_from[current]
                    path.append(nodes_extended[current])
                path.reverse()
                return path

            for neighbor in adjacency[current]:
                tentative_g = g_score[current] + np.linalg.norm(nodes_extended[current] - nodes_extended[neighbor])
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(nodes_extended[neighbor], goal)
                    heapq.heappush(open_heap, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current

        # Fallback if no path found
        return [start, goal]

    def plot(self, starts, goals, paths):
        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        ax.set_xlim(X_MIN, X_MAX)
        ax.set_ylim(Y_MIN, Y_MAX)
        ax.set_aspect('equal')

        # Draw obstacles and clearances
        for obs in self.all_obstacles:
            # clearance
            clearance_circle = Circle(
                obs.centre(),
                obs.radius + self.clearance,
                color='blue',
                alpha=0.2
            )
            ax.add_patch(clearance_circle)

            # robot body
            robot_circle = Circle(
                obs.centre(),
                obs.radius,
                color='yellow',
                alpha=0.8
            )
            ax.add_patch(robot_circle)

        # Virtual nodes
        ax.scatter(self.nodes[:, 0], self.nodes[:, 1], c='k', s=20)

        # Starts and goals
        for s, g in zip(starts, goals):
            ax.plot(s[0], s[1], 'go', markersize=8)
            ax.plot(g[0], g[1], 'ro', markersize=8)

        # Paths
        for path in paths:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], '-', linewidth=2)

        plt.grid(True)
        plt.show()


def main():
    #np.random.seed(42)  # for reproducibility

    # Example obstacles (opponents)
    obs_centers = np.array([
        [-2000, -2000], [1000, -1000], [2500, 2000],
        [-3000, 1000], [0, 0], [3500, -2500], [2000, 1000], [1000, 0]
    ])
    radius = 90
    unums = range(1, len(obs_centers) + 1)
    obstacles = Obstacle.from_numpy_array(obs_centers, radius, unums, isYellow=True)

    # --- Random teammates and opponents positions ---
    team_size = 6
    total_robots = team_size * 2

    # Random positions inside field, avoiding edges by 500mm
    margin = 500
    robot_positions = np.random.uniform(
        low=[X_MIN+margin, Y_MIN+margin],
        high=[X_MAX-margin, Y_MAX-margin],
        size=(total_robots, 2)
    )

    # Split into two teams
    teammates = robot_positions[:team_size]
    opponents = robot_positions[team_size:]

    # Random ball position
    ball = np.random.uniform(
        low=[X_MIN+margin, Y_MIN+margin],
        high=[X_MAX-margin, Y_MAX-margin],
        size=(1,2)
    )[0]

    # Start = teammates, Goal = ball
    starts = teammates
    goals = np.tile(ball, (team_size, 1))  # all teammates aim at ball

    planner = DiamondPlanner(obstacles, starts)

    paths = []
    for s, g in zip(starts, goals):
        paths.append(planner.plan_path(s, g))

    # Plot
    planner.plot(starts, goals, paths)

    print("Ball position:", ball)
    print("Teammates positions:\n", teammates)
    print("Opponents positions:\n", opponents)

if __name__ == "__main__":
    main()