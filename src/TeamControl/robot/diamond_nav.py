"""
Simple obstacle-steering navigation for robot behavior modules.

Every tick: check if straight line to goal is blocked by any robot on the
field.  If blocked, compute a waypoint that steers around the closest
blocker.  No graph, no A* — just geometry.  Fast, reliable, never crashes.

AVOIDS EVERY ROBOT ON THE FIELD except the planning robot itself (and
optionally excluded IDs like a teammate).

Usage:
    from TeamControl.robot.diamond_nav import DiamondNav

    dnav = DiamondNav()
    dnav.set_exclude((True, 3))   # optional: skip teammate
    wp = dnav.next_waypoint(frame, is_yellow, robot_id, me_xy, goal_xy)
    if wp is not None:
        # drive toward wp instead of goal
"""

import math

# Clearance from obstacle centre (obstacle radius + this margin)
AVOID_MARGIN = 300   # mm — very wide margin to prevent any contact


class DiamondNav:
    """Stateful obstacle-steering planner — one instance per robot."""

    def __init__(self):
        self._exclude_ids = set()

    def set_exclude(self, *pairs):
        """Robot IDs to skip as obstacles, e.g. teammate.
        Call like: dnav.set_exclude((True, 3))
        """
        self._exclude_ids = set(pairs)

    def next_waypoint(self, frame, is_yellow, robot_id, me_xy, goal_xy,
                      replan_dist=None):
        """Return (x, y) waypoint to steer around obstacles, or None if
        the straight line to goal is clear."""
        me = (float(me_xy[0]), float(me_xy[1]))
        goal = (float(goal_xy[0]), float(goal_xy[1]))

        # Collect every robot on field except me (and excludes)
        obstacles = _get_all_obstacles(frame, is_yellow, robot_id,
                                       self._exclude_ids)
        if not obstacles:
            return None

        # Find the first obstacle blocking the straight line to goal
        blocker = _first_blocker(me, goal, obstacles)
        if blocker is None:
            return None  # path is clear

        # Compute two tangent waypoints around the blocker
        left, right = _steer_around(me, goal, blocker)

        # Pick the side that keeps us closer to the goal
        dl = _dist(left, goal)
        dr = _dist(right, goal)
        wp = left if dl < dr else right

        # Check if the chosen waypoint is itself inside another obstacle
        # If so, push it further out
        for obs in obstacles:
            ox, oy, orad = obs
            d_to_obs = _dist(wp, (ox, oy))
            safe_r = orad + AVOID_MARGIN
            if d_to_obs < safe_r and d_to_obs > 1:
                # Push waypoint outward from this obstacle
                dx = wp[0] - ox
                dy = wp[1] - oy
                scale = safe_r / d_to_obs
                wp = (ox + dx * scale, oy + dy * scale)

        return wp

    def clear(self):
        """API compat — nothing to reset in this simple planner."""
        pass


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _first_blocker(start, goal, obstacles):
    """Find the closest obstacle along the line from start to goal that
    blocks the path.  Returns (cx, cy, radius) or None."""
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    if length < 1:
        return None
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux  # normal

    best = None
    best_along = float('inf')

    for (ox, oy, orad) in obstacles:
        # Vector from start to obstacle centre
        rx, ry = ox - sx, oy - sy
        along = rx * ux + ry * uy  # projection along line
        perp = abs(rx * nx + ry * ny)  # distance from line

        safe_r = orad + AVOID_MARGIN

        # Only consider obstacles that are between start and goal
        if along < -safe_r or along > length + safe_r:
            continue

        # Check if obstacle is close enough to the line to block it
        if perp < safe_r:
            if along < best_along:
                best_along = along
                best = (ox, oy, orad)

    return best


def _steer_around(me, goal, blocker):
    """Compute two waypoints (left, right) that go around the blocker."""
    ox, oy, orad = blocker
    safe_r = orad + AVOID_MARGIN

    # Direction from me to goal
    dx, dy = goal[0] - me[0], goal[1] - me[1]
    length = max(math.hypot(dx, dy), 1.0)
    ux, uy = dx / length, dy / length

    # Perpendicular
    px, py = -uy, ux

    # Two waypoints: left and right of the obstacle
    left = (ox + px * safe_r, oy + py * safe_r)
    right = (ox - px * safe_r, oy - py * safe_r)

    return left, right


def _get_all_obstacles(frame, is_yellow, robot_id, exclude_ids):
    """Get (cx, cy, radius) for every robot on field except self and excludes."""
    obstacles = []
    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            if (color, oid) in exclude_ids:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                op = other.position
                # Use obstacle radius if available, otherwise default 90mm
                rad = 90
                if hasattr(other, 'obstacle') and other.obstacle is not None:
                    rad = other.obstacle.radius
                obstacles.append((float(op[0]), float(op[1]), rad))
            except Exception:
                continue
    return obstacles


def get_all_obstacles(frame, is_yellow, robot_id, exclude_ids=None):
    """Public version — returns list of (cx, cy, radius) for all robots
    on field except self and optionally excluded IDs."""
    return _get_all_obstacles(frame, is_yellow, robot_id,
                              exclude_ids or set())
