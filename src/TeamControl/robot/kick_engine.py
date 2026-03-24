"""
Shared kick engine — reliable kick-to-target for any robot behavior.

Usage:
    ks = KickState()

    # Each tick:
    result = kick_tick(ks, me, ball, aim, now, rel_ball, d_ball)
    vx, vy, w       = result.vx, result.vy, result.w
    kick, dribble    = result.kick, result.dribble
    if result.kick_started:
        print("kick burst started!")

The engine handles:
  - Contact detection (ball touching dribbler)
  - Hold-and-rotate to face aim before kicking
  - Blended approach (face ball -> face aim as we close in)
  - Sustained kick burst (kick=1 for multiple ticks so grSim registers)
  - Force kick timeout (last resort if alignment takes too long)
  - Cooldown tracking
"""

import math
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import clamp, move_toward, compute_arc_nav, turn_then_move
from TeamControl.robot.constants import (
    KICK_RANGE, BALL_NEAR, BEHIND_DIST, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN,
)

# -- Tuning (shared across all users) ----------------------------------
CONTACT_DIST    = 130       # mm — ball touching dribbler
KICK_ALIGN_TOL  = 0.10      # rad (~6 deg) — tight alignment for precise kicks
KICK_BURST_T    = 0.55      # s — sustain kick=1 longer so grSim reliably registers
FORCE_KICK_TIME = 2.5       # s — be patient, wait for proper alignment
DRIBBLE_SPD     = DRIBBLE_SPEED
APPROACH_SPD    = CRUISE_SPEED

# Forward pressure while dribbling/rotating — keeps ball on dribbler
HOLD_VX         = DRIBBLE_SPD * 0.35  # gentle pressure — don't push ball away while turning
# Lateral correction gain — nudge sideways to stay centered on ball
HOLD_VY_GAIN    = 0.3


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
    rel_ball : (rx, ry) — ball in robot-local coords (optional, computed if None)
    d_ball   : float — distance to ball (optional, computed if None)

    Returns
    -------
    KickResult with vx, vy, w, kick, dribble, and status flags.
    """
    r = KickResult()

    if ball is None:
        ks.bursting = False
        return r

    # Compute robot-local ball if not provided
    if rel_ball is None:
        rel_ball = world2robot(me, ball)
    if d_ball is None:
        d_ball = math.hypot(rel_ball[0], rel_ball[1])

    ang_ball = math.atan2(rel_ball[1], rel_ball[0])
    rel_aim = world2robot(me, aim)
    ang_aim = math.atan2(rel_aim[1], rel_aim[0])

    # ==================================================================
    #  BURST MODE — sustained kick already in progress
    # ==================================================================
    if ks.bursting:
        r.dribble = 1
        r.kick = 1  # always kick during burst — maximise contact window
        # Drive hard into ball to keep contact
        r.vx, r.vy = move_toward(rel_ball, CHARGE_SPEED,
                                  ramp_dist=80, stop_dist=0)
        r.w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

        if now > ks.kick_end_time:
            ks.bursting = False
            r.burst_done = True

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  P1: Ball touching dribbler — hold + rotate to face aim, then kick
    #
    #  Only if ball is in front of dribbler (rel_ball[0] > 0) and we're
    #  at least somewhat facing the aim. If we hit the ball from the
    #  side, fall through to P3 to reposition.
    # ==================================================================
    if d_ball < CONTACT_DIST and rel_ball[0] > 0 and abs(ang_aim) < 1.2:
        r.dribble = 1

        # Rotate to face aim — prioritise turning, reduce movement
        r.w = clamp(ang_aim * TURN_GAIN * 2.0, -MAX_W, MAX_W)

        # Scale forward pressure by alignment — stop pushing when misaligned
        align_factor = max(1.0 - abs(ang_aim) / 0.5, 0.1)
        r.vx = HOLD_VX * align_factor
        # Lateral correction — nudge toward ball center
        r.vy = clamp(rel_ball[1] * HOLD_VY_GAIN, -DRIBBLE_SPD * 0.2,
                      DRIBBLE_SPD * 0.2) * align_factor

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now
        time_near = now - ks.near_ball_since
        force_kick = time_near > FORCE_KICK_TIME
        can_kick = (now - ks.last_kick) > KICK_COOLDOWN
        aligned = abs(ang_aim) < KICK_ALIGN_TOL

        # Also kick if we've been near long enough and roughly aligned
        roughly_aligned = abs(ang_aim) < KICK_ALIGN_TOL * 3.0
        patient_kick = time_near > (FORCE_KICK_TIME * 0.7) and roughly_aligned

        if can_kick and (aligned or force_kick or patient_kick):
            # Start kick burst
            ks.bursting = True
            ks.kick_end_time = now + KICK_BURST_T
            ks.last_kick = now
            ks.near_ball_since = 0.0
            ks.committed_side = None
            r.kick = 1
            r.kick_started = True
            # Charge forward into ball
            r.vx, r.vy = move_toward(rel_ball, CHARGE_SPEED,
                                      ramp_dist=80, stop_dist=0)

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  P2: Ball close + in front + LINED UP BEHIND IT
    #
    #  Only engage if we're actually approaching from behind (facing
    #  roughly toward the aim). Otherwise fall through to P3 arc.
    #  This prevents driving onto the ball from the side.
    # ==================================================================
    if (d_ball < KICK_RANGE and rel_ball[0] > -10
            and abs(ang_aim) < 0.6):  # ~34 deg — must be roughly behind
        r.dribble = 1
        # Decelerate as we close in — arrive at ball slowly and controlled
        approach_t = clamp((d_ball - CONTACT_DIST) /
                           max(KICK_RANGE - CONTACT_DIST, 1.0), 0.0, 1.0)
        drive_speed = DRIBBLE_SPD * (0.3 + 0.7 * approach_t)
        r.vx, r.vy = move_toward(rel_ball, drive_speed,
                                  ramp_dist=100, stop_dist=0)
        # Blend rotation: face ball far out, face aim when close
        blend = clamp(1.0 - d_ball / KICK_RANGE, 0.0, 1.0)
        w_ball = ang_ball * TURN_GAIN
        w_aim = ang_aim * TURN_GAIN * 1.5
        r.w = clamp(w_ball * (1.0 - blend) + w_aim * blend,
                     -MAX_W, MAX_W)

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  P3: Get behind ball, then drive at it.
    #
    #  Simple: compute "behind point" = opposite side of ball from aim.
    #  Go to that point. Once there, drive at ball.
    #  No complex arcs — just go to the behind point.
    # ==================================================================
    ks.near_ball_since = 0.0

    # Unit vector ball → aim
    ba_dx = aim[0] - ball[0]
    ba_dy = aim[1] - ball[1]
    ba_d = max(math.hypot(ba_dx, ba_dy), 1.0)
    aim_ux, aim_uy = ba_dx / ba_d, ba_dy / ba_d

    # Behind point = ball - aim_direction * BEHIND_DIST
    behind = (ball[0] - aim_ux * BEHIND_DIST,
              ball[1] - aim_uy * BEHIND_DIST)

    # How far am I from the behind point?
    rel_behind = world2robot(me, behind)
    d_behind = math.hypot(rel_behind[0], rel_behind[1])

    # Am I at the behind point and facing the ball?
    at_behind = d_behind < 200
    facing_ball = abs(ang_ball) < 0.5

    if at_behind and facing_ball:
        # At the behind point, facing ball — drive straight in
        r.dribble = 1
        r.vx, r.vy = move_toward(rel_ball, DRIBBLE_SPD,
                                  ramp_dist=250, stop_dist=0)
        r.w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)
    else:
        # Go to behind point
        ang_behind = math.atan2(rel_behind[1], rel_behind[0])
        r.w = clamp(ang_behind * TURN_GAIN, -MAX_W, MAX_W)
        r.vx, r.vy = move_toward(rel_behind, CRUISE_SPEED * 0.7,
                                  ramp_dist=300, stop_dist=30)
        # Slow down if not facing the target
        if abs(ang_behind) > 0.5:
            scale = max(1.0 - abs(ang_behind) / 1.5, 0.1)
            r.vx *= scale
            r.vy *= scale

    r.committed_side = ks.committed_side
    return r
