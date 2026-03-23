"""
Shared Diamond path-planning helper for robot behavior modules.

Provides a lightweight wrapper that builds a DiamondPlanner from a vision
frame and returns the next waypoint for a robot to drive toward.  Models
that previously relied solely on reactive obstacle avoidance can use this
to get global, collision-free paths through cluttered fields.

Usage in a behavior loop:
    from TeamControl.robot.diamond_nav import DiamondNav

    dnav = DiamondNav()
    ...
    waypoint = dnav.next_waypoint(frame, is_yellow, robot_id, me_xy, goal_xy)
    if waypoint is not None:
        # drive toward waypoint instead of goal directly
"""

import numpy as np
from TeamControl.voronoi_planner.diamond_planner import DiamondPlanner, ROBOT_RADIUS


# How close (mm) to a waypoint before advancing to the next one
_WP_REACHED = 150


class DiamondNav:
    """Stateful path-planner wrapper — one instance per robot."""

    def __init__(self):
        self._path = []           # current planned path (list of np arrays)
        self._wp_idx = 0          # index of next waypoint to reach
        self._last_goal = None    # goal used for latest plan (detect changes)

    def next_waypoint(self, frame, is_yellow, robot_id, me_xy, goal_xy,
                      replan_dist=300):
        """
        Return the next waypoint (x, y) the robot should drive toward,
        or *None* if no planning is needed (direct path is fine).

        Parameters
        ----------
        frame : vision frame
        is_yellow : bool
        robot_id : int
        me_xy : tuple (x, y) — robot position in mm
        goal_xy : tuple (x, y) — desired destination in mm
        replan_dist : float — replan when goal moves more than this

        Returns
        -------
        tuple (x, y) | None
            Next waypoint, or None if straight-line to goal is clear.
        """
        me  = np.asarray(me_xy[:2], dtype=float)
        goal = np.asarray(goal_xy[:2], dtype=float)

        # Decide if we need a (re)plan
        need_plan = (
            len(self._path) == 0
            or self._last_goal is None
            or np.linalg.norm(goal - self._last_goal) > replan_dist
        )

        if need_plan:
            obstacles = _obstacles_from_frame(frame, is_yellow, robot_id)
            starts = np.array([me])
            planner = DiamondPlanner(obstacles, starts)
            self._path = planner.plan_path(me, goal)
            self._wp_idx = 1  # skip index 0 which is our current position
            self._last_goal = goal.copy()

        # Advance through waypoints we've already reached
        while (self._wp_idx < len(self._path) - 1
               and np.linalg.norm(me - np.asarray(self._path[self._wp_idx])) < _WP_REACHED):
            self._wp_idx += 1

        # If only the final goal remains, return None (caller can drive direct)
        if self._wp_idx >= len(self._path) - 1:
            # Check if the path had intermediate nodes; if only [start, goal]
            # that means the straight line was clear.
            if len(self._path) <= 2:
                return None
            # Return the final goal as the waypoint
            return tuple(self._path[-1])

        wp = self._path[self._wp_idx]
        return (float(wp[0]), float(wp[1]))

    def clear(self):
        """Reset the planner state (e.g. after a mode change)."""
        self._path = []
        self._wp_idx = 0
        self._last_goal = None


def _obstacles_from_frame(frame, is_yellow, robot_id):
    """Extract Obstacle list from a vision frame, excluding the given robot."""
    obstacles = []
    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                obstacles.append(other.obstacle)
            except Exception:
                continue
    return obstacles
