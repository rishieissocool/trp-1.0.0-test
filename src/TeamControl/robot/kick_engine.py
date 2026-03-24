"""
Shared kick engine — reliable kick-to-target for any robot behavior.

Simple approach:
  1. Drive straight at the ball, dribbler on
  2. Once ball is on dribbler, rotate in place to face the aim
  3. When aligned, kick hard

No arc approach, no complex geometry. The dribbler holds the ball
while the robot rotates to face the target.
"""

import math
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import clamp, move_toward
from TeamControl.robot.constants import (
    KICK_RANGE, BALL_NEAR,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN,
)

# -- Tuning ---------------------------------------------------------------
CONTACT_DIST    = 160       # mm — ball on dribbler
KICK_ALIGN_TOL  = 0.12      # rad (~7 deg) — precise alignment for accurate kicks
KICK_BURST_T    = 0.55      # s — sustain kick=1 so grSim registers
FORCE_KICK_TIME = 2.0       # s — patient, give time to rotate fully
DRIBBLE_SPD     = DRIBBLE_SPEED
HOLD_VX_ALIGNED = DRIBBLE_SPD * 0.15  # tiny creep only when nearly aligned
HOLD_VY_GAIN    = 0.15


class KickState:
    """Persistent state for the kick engine — one per robot."""
    __slots__ = (
        "near_ball_since", "last_kick", "committed_side",
        "kick_end_time", "bursting",
    )

    def __init__(self):
        self.near_ball_since = 0.0
        self.last_kick = 0.0
        self.committed_side = None
        self.kick_end_time = 0.0
        self.bursting = False

    def reset(self):
        self.near_ball_since = 0.0
        self.committed_side = None
        self.bursting = False


class KickResult:
    """Output of a single kick_tick call."""
    __slots__ = (
        "vx", "vy", "w", "kick", "dribble",
        "kick_started", "burst_done", "committed_side",
    )

    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.kick = 0
        self.dribble = 0
        self.kick_started = False
        self.burst_done = False
        self.committed_side = None


def kick_tick(ks, me, ball, aim, now, rel_ball=None, d_ball=None):
    """
    Run one tick of the kick engine.

    Parameters
    ----------
    ks       : KickState — persistent state (mutated in place)
    me       : (x, y, orientation) — robot pose
    ball     : (x, y) — ball world position, or None
    aim      : (x, y) — world target to kick toward (mate, goal, etc.)
    now      : float — current time
    rel_ball : (rx, ry) — ball in robot-local coords (optional)
    d_ball   : float — distance to ball (optional)

    Returns
    -------
    KickResult with vx, vy, w, kick, dribble, and status flags.
    """
    r = KickResult()

    if ball is None:
        ks.bursting = False
        return r

    if rel_ball is None:
        rel_ball = world2robot(me, ball)
    if d_ball is None:
        d_ball = math.hypot(rel_ball[0], rel_ball[1])

    ang_ball = math.atan2(rel_ball[1], rel_ball[0])
    rel_aim = world2robot(me, aim)
    ang_aim = math.atan2(rel_aim[1], rel_aim[0])

    # ==================================================================
    #  BURST — kick already in progress, drive into ball
    # ==================================================================
    if ks.bursting:
        r.dribble = 1
        r.kick = 1
        r.vx, r.vy = move_toward(rel_ball, CHARGE_SPEED,
                                  ramp_dist=80, stop_dist=0)
        r.w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

        if now > ks.kick_end_time:
            ks.bursting = False
            r.burst_done = True

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  CONTACT — ball on dribbler.
    #  STOP MOVING. Rotate in place to face aim. Then kick.
    #  Only allow tiny forward creep when nearly aligned.
    # ==================================================================
    if d_ball < CONTACT_DIST and rel_ball[0] > -10:
        r.dribble = 1

        # ROTATE IN PLACE — strong rotation, no translation
        r.w = clamp(ang_aim * TURN_GAIN * 3.0, -MAX_W, MAX_W)

        misalignment = abs(ang_aim)

        if misalignment > 0.20:
            # Still turning — stop all movement, just spin
            r.vx = 0.0
            r.vy = 0.0
        else:
            # Nearly aligned — tiny forward creep to maintain contact
            r.vx = HOLD_VX_ALIGNED
            r.vy = clamp(rel_ball[1] * HOLD_VY_GAIN,
                          -DRIBBLE_SPD * 0.08, DRIBBLE_SPD * 0.08)

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now
        time_near = now - ks.near_ball_since
        can_kick = (now - ks.last_kick) > KICK_COOLDOWN
        aligned = misalignment < KICK_ALIGN_TOL
        force_kick = time_near > FORCE_KICK_TIME

        if can_kick and (aligned or force_kick):
            ks.bursting = True
            ks.kick_end_time = now + KICK_BURST_T
            ks.last_kick = now
            ks.near_ball_since = 0.0
            r.kick = 1
            r.kick_started = True
            r.vx, r.vy = move_toward(rel_ball, CHARGE_SPEED,
                                      ramp_dist=80, stop_dist=0)

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  CLOSE — ball nearby and in front. Creep toward it slowly.
    # ==================================================================
    if d_ball < KICK_RANGE and rel_ball[0] > -10:
        r.dribble = 1
        # Very slow — creep in so we don't push ball away
        r.vx, r.vy = move_toward(rel_ball, DRIBBLE_SPD * 0.35,
                                  ramp_dist=80, stop_dist=0)
        # Face ball, gentle aim blend
        blend = clamp(1.0 - d_ball / KICK_RANGE, 0.0, 0.4)
        w_ball = ang_ball * TURN_GAIN
        w_aim = ang_aim * TURN_GAIN
        r.w = clamp(w_ball * (1.0 - blend) + w_aim * blend,
                     -MAX_W, MAX_W)

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  FAR — turn to face ball first, then drive.
    #  If not facing ball, rotate in place. Only move when facing it.
    # ==================================================================
    ks.near_ball_since = 0.0
    r.dribble = 1 if d_ball < BALL_NEAR else 0

    r.w = clamp(ang_ball * TURN_GAIN * 1.5, -MAX_W, MAX_W)

    if abs(ang_ball) > 0.5:
        # Not facing ball — rotate in place, don't move
        r.vx = 0.0
        r.vy = 0.0
    else:
        # Facing ball — drive toward it, slower as we get closer
        if d_ball > BALL_NEAR:
            speed = CRUISE_SPEED * 0.7
        elif d_ball > KICK_RANGE * 2:
            speed = DRIBBLE_SPD * 0.6
        else:
            speed = DRIBBLE_SPD * 0.35

        r.vx, r.vy = move_toward(rel_ball, speed,
                                  ramp_dist=400, stop_dist=10)

        # Reduce speed proportionally to remaining angle error
        if abs(ang_ball) > 0.2:
            scale = max(1.0 - abs(ang_ball) / 0.5, 0.2)
            r.vx *= scale
            r.vy *= scale

    r.committed_side = ks.committed_side
    return r
