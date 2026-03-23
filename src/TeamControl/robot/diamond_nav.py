"""
Shared Diamond path-planning helper for robot behavior modules.

Builds a DiamondPlanner from the current vision frame and returns the next
waypoint for a robot to drive toward.  Caches obstacle positions and only
replans when obstacles move, the goal changes, or the current path becomes
invalid.

Usage:
    from TeamControl.robot.diamond_nav import DiamondNav

    dnav = DiamondNav()
    ...
    waypoint = dnav.next_waypoint(frame, is_yellow, robot_id, me_xy, goal_xy)
    if waypoint is not None:
        # drive toward waypoint instead of goal directly
"""

import numpy as np
from TeamControl.voronoi_planner.diamond_planner import DiamondPlanner, ROBOT_RADIUS
from TeamControl.voronoi_planner.obstacle import Obstacle


# How close (mm) to a waypoint before advancing to the next one
_WP_REACHED = 120
# Replan when goal moves more than this (mm)
_GOAL_REPLAN = 250
# Replan when any obstacle has moved more than this (mm)
_OBS_MOVE_THRESH = 150
# Max age (ticks) before forcing a replan even if nothing seems to have moved
_MAX_PLAN_AGE = 30


class DiamondNav:
    """Stateful path-planner wrapper — one instance per robot."""

    def __init__(self):
        self._path = []
        self._wp_idx = 0
        self._last_goal = None
        self._last_obs_positions = None   # np array of obstacle centers
        self._plan_age = 0
        self._exclude_ids = set()         # (color, id) pairs to skip

    def set_exclude(self, *pairs):
        """Extra robot IDs to exclude from obstacles, e.g. teammate.

        Call like: dnav.set_exclude((True, 3), (False, 1))
        """
        self._exclude_ids = set(pairs)

    def next_waypoint(self, frame, is_yellow, robot_id, me_xy, goal_xy,
                      replan_dist=_GOAL_REPLAN):
        """
        Return the next waypoint (x, y) the robot should drive toward,
        or *None* if the straight line to goal is clear.
        """
        me   = np.asarray(me_xy[:2], dtype=float)
        goal = np.asarray(goal_xy[:2], dtype=float)

        self._plan_age += 1

        # --- Decide if we need a (re)plan ---
        need_plan = len(self._path) == 0 or self._last_goal is None

        # Goal moved?
        if not need_plan and np.linalg.norm(goal - self._last_goal) > replan_dist:
            need_plan = True

        # Obstacles moved?
        obs_list = _obstacles_from_frame(
            frame, is_yellow, robot_id, self._exclude_ids)
        curr_obs_pos = np.array([o.centre() for o in obs_list]) \
            if obs_list else np.empty((0, 2))

        if not need_plan and self._last_obs_positions is not None:
            if curr_obs_pos.shape == self._last_obs_positions.shape and len(curr_obs_pos) > 0:
                max_move = np.max(np.linalg.norm(
                    curr_obs_pos - self._last_obs_positions, axis=1))
                if max_move > _OBS_MOVE_THRESH:
                    need_plan = True
            elif curr_obs_pos.shape != self._last_obs_positions.shape:
                need_plan = True

        # Stale plan?
        if not need_plan and self._plan_age > _MAX_PLAN_AGE:
            need_plan = True

        # Current path segment blocked?
        if not need_plan and self._wp_idx < len(self._path):
            planner = DiamondPlanner(obs_list)
            wp_pos = np.asarray(self._path[self._wp_idx])
            if not planner.is_path_free(me, wp_pos):
                need_plan = True

        # --- Plan ---
        if need_plan:
            planner = DiamondPlanner(obs_list)
            self._path = planner.plan_path(me, goal)
            self._wp_idx = 1  # skip index 0 (current position)
            self._last_goal = goal.copy()
            self._last_obs_positions = curr_obs_pos.copy() if len(curr_obs_pos) > 0 else None
            self._plan_age = 0

        # --- Advance through reached waypoints ---
        while (self._wp_idx < len(self._path) - 1
               and np.linalg.norm(me - np.asarray(self._path[self._wp_idx])) < _WP_REACHED):
            self._wp_idx += 1

        # --- Return ---
        if self._wp_idx >= len(self._path):
            return None

        # If only [start, goal] and start is us, straight line is clear
        if len(self._path) <= 2 and self._wp_idx >= len(self._path) - 1:
            return None

        wp = self._path[self._wp_idx]
        return (float(wp[0]), float(wp[1]))

    def clear(self):
        """Reset the planner state (e.g. after a mode change)."""
        self._path = []
        self._wp_idx = 0
        self._last_goal = None
        self._last_obs_positions = None
        self._plan_age = 0


def _obstacles_from_frame(frame, is_yellow, robot_id, extra_exclude=None):
    """Extract Obstacle list from a vision frame, excluding the given robot
    and any additional (color, id) pairs in extra_exclude."""
    if extra_exclude is None:
        extra_exclude = set()
    obstacles = []
    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            if (color, oid) in extra_exclude:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                obstacles.append(other.obstacle)
            except Exception:
                continue
    return obstacles
