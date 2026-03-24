"""
Obstacle-steering navigation with waypoint persistence.

Every tick: check if straight line to goal is blocked by any robot on the
field.  If blocked, compute a waypoint that steers around the closest
blocker and STICK WITH IT until reached or invalidated.

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

# Waypoint persistence — prevents oscillation
WP_REACH_DIST   = 200  # mm — waypoint considered "reached"
WP_STALE_DIST   = 600  # mm — replan if cached wp is this far from new candidate
SIDE_HYSTERESIS  = 150  # mm — must be this far on opposite side to flip commitment


class DiamondNav:
    """Stateful obstacle-steering planner — one instance per robot."""

    def __init__(self):
        self._exclude_ids = set()
        self._cached_wp = None       # persisted waypoint
        self._committed_side = None  # +1 or -1: left or right of blocker
        self._last_blocker = None    # (cx, cy) of blocker we're avoiding

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
            self._cached_wp = None
            self._committed_side = None
            self._last_blocker = None
            return None

        # Find the first obstacle blocking the straight line to goal
        blocker = _first_blocker(me, goal, obstacles)
        if blocker is None:
            self._cached_wp = None
            self._committed_side = None
            self._last_blocker = None
            return None  # path is clear

        # -- Committed side persistence --
        # Determine which side of the blocker we're on
        ox, oy, orad = blocker
        dx, dy = goal[0] - me[0], goal[1] - me[1]
        length = max(math.hypot(dx, dy), 1.0)
        ux, uy = dx / length, dy / length
        px, py = -uy, ux  # perpendicular

        # Robot's lateral offset relative to blocker along perp axis
        rx, ry = me[0] - ox, me[1] - oy
        lateral = rx * px + ry * py

        # Commit to a side with hysteresis
        if self._committed_side is None:
            self._committed_side = 1 if lateral >= 0 else -1
        else:
            # Only flip if clearly on the other side
            if self._committed_side == 1 and lateral < -SIDE_HYSTERESIS:
                self._committed_side = -1
            elif self._committed_side == -1 and lateral > SIDE_HYSTERESIS:
                self._committed_side = 1

        # Compute waypoint on the committed side
        safe_r = orad + AVOID_MARGIN
        if self._committed_side == 1:
            wp = (ox + px * safe_r, oy + py * safe_r)
        else:
            wp = (ox - px * safe_r, oy - py * safe_r)

        # Push waypoint out of any other obstacle's clearance
        for obs in obstacles:
            o2x, o2y, o2rad = obs
            d_to_obs = _dist(wp, (o2x, o2y))
            s_r = o2rad + AVOID_MARGIN
            if d_to_obs < s_r and d_to_obs > 1:
                ddx = wp[0] - o2x
                ddy = wp[1] - o2y
                scale = s_r / d_to_obs
                wp = (o2x + ddx * scale, o2y + ddy * scale)

        # -- Waypoint caching: stick with previous wp if still valid --
        if self._cached_wp is not None:
            d_to_cached = _dist(me, self._cached_wp)
            d_cached_to_new = _dist(self._cached_wp, wp)

            if d_to_cached < WP_REACH_DIST:
                # Reached the cached waypoint — accept the new one
                self._cached_wp = wp
            elif d_cached_to_new < WP_STALE_DIST:
                # New wp is close to cached — keep the cached one (stable)
                wp = self._cached_wp
            else:
                # Situation changed significantly — accept new wp
                self._cached_wp = wp
        else:
            self._cached_wp = wp

        self._last_blocker = (ox, oy)
        return wp

    def clear(self):
        """Reset cached state — call when goal/role changes."""
        self._cached_wp = None
        self._committed_side = None
        self._last_blocker = None


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


def _get_all_obstacles(frame, is_yellow, robot_id, exclude_ids):
    """Get (cx, cy, radius) for every robot on field except self and excludes.

    Filters out phantom robots (confidence too low or sitting at origin)
    which grSim reports for inactive robot slots.
    """
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
                # Skip phantom robots — low confidence or parked at origin
                if hasattr(other, 'confidence') and other.confidence < 0.1:
                    continue
                op = other.position
                ox, oy = float(op[0]), float(op[1])
                # Robots sitting exactly at origin are almost certainly phantoms
                if abs(ox) < 1 and abs(oy) < 1:
                    continue
                # Use obstacle radius if available, otherwise default 90mm
                rad = 90
                if hasattr(other, 'obstacle') and other.obstacle is not None:
                    rad = other.obstacle.radius
                obstacles.append((ox, oy, rad))
            except Exception:
                continue
    return obstacles


def get_all_obstacles(frame, is_yellow, robot_id, exclude_ids=None):
    """Public version — returns list of (cx, cy, radius) for all robots
    on field except self and optionally excluded IDs."""
    return _get_all_obstacles(frame, is_yellow, robot_id,
                              exclude_ids or set())
