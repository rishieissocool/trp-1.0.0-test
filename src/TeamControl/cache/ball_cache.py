"""Ball-related per-frame state.

Holds the ball position, the rolling history buffer, and lazily computed
velocity / prediction. All are invalidated when the frame version
advances. `last_known()` lets callers fall back to the most recent
visible ball when the camera loses it briefly.
"""

from TeamControl.robot.ball_nav import (
    ball_velocity, predict_ball, update_ball_history,
)
from TeamControl.robot.constants import BALL_HISTORY_SIZE


class BallCache:
    """Per-frame ball state.

    Fields:
      position  -> (x, y) or None when occluded
      visible   -> True when this frame had a ball
      velocity  -> (vx, vy, speed) computed from the history buffer
      history   -> rolling [(t, x, y), ...] up to BALL_HISTORY_SIZE
    """

    def __init__(self, history_size=BALL_HISTORY_SIZE):
        self.history = []
        self._history_size = history_size
        self._last_ball_xy = None
        self._frame_version = None

        self._position = None
        self._visible = False
        self._velocity = None
        self._last_known = None
        self._last_known_time = 0.0

    def update(self, frame, now, version):
        """Refresh from `frame`. Cheap when version hasn't changed."""
        if version is not None and version == self._frame_version:
            return
        self._frame_version = version

        self._visible = frame is not None and frame.ball is not None
        if self._visible:
            bp = frame.ball.position
            self._position = (float(bp[0]), float(bp[1]))
            self._last_known = self._position
            self._last_known_time = now
            self._last_ball_xy = update_ball_history(
                self.history, now, self._position, self._last_ball_xy,
                max_size=self._history_size,
            )
        else:
            self._position = None
        self._velocity = None

    @property
    def position(self):
        return self._position

    @property
    def visible(self):
        return self._visible

    @property
    def velocity(self):
        if self._velocity is None:
            self._velocity = ball_velocity(self.history)
        return self._velocity

    def last_known(self, now=None, memory_time=None):
        """Most recent visible position, optionally requiring it be fresh."""
        if self._last_known is None:
            return None
        if now is not None and memory_time is not None:
            if (now - self._last_known_time) > memory_time:
                return None
        return self._last_known

    @property
    def last_known_time(self):
        return self._last_known_time

    def predict(self, dt):
        """Predict ball position `dt` seconds ahead."""
        if self._position is None:
            return None
        vx, vy, _ = self.velocity
        return predict_ball(self._position, (vx, vy), dt)
