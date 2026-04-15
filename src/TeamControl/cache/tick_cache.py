"""Aggregator — one TickCache per robot process.

Usage in a tick loop:

    cache = TickCache(wm)
    while is_running.is_set():
        now = time.time()
        if not cache.refresh(now):
            time.sleep(LOOP_RATE)
            continue
        ball = cache.ball.position
        vx, vy, speed = cache.ball.velocity
        me = cache.robots.get_position(is_yellow, my_id)
        goal_x = cache.team.goal_x(is_yellow)
        ...

`refresh(now)` pulls the latest frame and version from `wm` and
propagates them into the category caches. Returns False when no frame
is available yet (caller should skip the tick).
"""

from .ball_cache import BallCache
from .robot_cache import RobotCache
from .team_cache import TeamCache
from .game_state_cache import GameStateCache


class TickCache:
    """Bundle of per-process category caches driven by the frame version."""

    def __init__(self, wm, ball_history_size=None):
        self._wm = wm
        if ball_history_size is None:
            self.ball = BallCache()
        else:
            self.ball = BallCache(history_size=ball_history_size)
        self.robots = RobotCache()
        self.team = TeamCache(wm)
        self.game = GameStateCache(wm)

        self._frame = None
        self._version = None
        self._frame_changed = False

    def refresh(self, now):
        """Pull latest frame + version from wm and refresh all categories.

        Returns True if a frame is available (either freshly pulled or
        still the last-known one), False if we've never seen one.
        """
        try:
            f = self._wm.get_latest_frame()
            if f is not None:
                self._frame = f
        except Exception:
            pass

        try:
            v = self._wm.get_version()
        except Exception:
            v = None

        self._frame_changed = (v != self._version)
        self._version = v

        if self._frame is not None:
            self.ball.update(self._frame, now, v)
            self.robots.update(self._frame, v)
        self.team.refresh()
        self.game.refresh()

        return self._frame is not None

    @property
    def frame(self):
        return self._frame

    @property
    def version(self):
        return self._version

    @property
    def frame_changed(self):
        """True when the most recent refresh() advanced the frame version."""
        return self._frame_changed

    @property
    def wm(self):
        return self._wm
