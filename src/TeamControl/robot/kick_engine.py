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
CONTACT_DIST    = 115       # mm — ball touching dribbler (90 robot + 21 ball + small margin)
KICK_ALIGN_TOL  = 0.18      # rad (~10 deg) — slightly more forgiving alignment
KICK_BURST_T    = 0.50      # s — sustain kick=1 longer so grSim reliably registers
FORCE_KICK_TIME = 1.5       # s — force kick sooner if stuck near ball
DRIBBLE_SPD     = DRIBBLE_SPEED
APPROACH_SPD    = CRUISE_SPEED

# Forward pressure while dribbling/rotating — keeps ball on dribbler
HOLD_VX         = DRIBBLE_SPD * 0.55
# Lateral correction gain — nudge sideways to stay centered on ball
HOLD_VY_GAIN    = 0.4


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
    #  Key: strong forward pressure so the ball stays on the dribbler
    #  while we rotate. Small lateral correction keeps us centered.
    # ==================================================================
    if d_ball < CONTACT_DIST and rel_ball[0] > -10:
        r.dribble = 1

        # Forward pressure — keep ball pressed against dribbler
        r.vx = HOLD_VX
        # Lateral correction — if ball drifts sideways, nudge toward it
        r.vy = clamp(rel_ball[1] * HOLD_VY_GAIN, -DRIBBLE_SPD * 0.3,
                      DRIBBLE_SPD * 0.3)
        # Rotate to face aim with boosted gain
        r.w = clamp(ang_aim * TURN_GAIN * 1.8, -MAX_W, MAX_W)

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now
        time_near = now - ks.near_ball_since
        force_kick = time_near > FORCE_KICK_TIME
        can_kick = (now - ks.last_kick) > KICK_COOLDOWN
        aligned = abs(ang_aim) < KICK_ALIGN_TOL

        # Also kick if we've been near long enough and roughly aligned
        roughly_aligned = abs(ang_aim) < KICK_ALIGN_TOL * 2.5
        patient_kick = time_near > (FORCE_KICK_TIME * 0.6) and roughly_aligned

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
    #  P2: Ball close + in front — drive in with dribbler, start aligning
    #
    #  Drive harder toward ball (CHARGE not DRIBBLE) so we close the gap
    #  fast. Start blending rotation toward aim as we get closer.
    # ==================================================================
    if d_ball < KICK_RANGE and rel_ball[0] > -10:
        r.dribble = 1
        # Use charge speed to close the gap fast — don't putter around
        drive_speed = DRIBBLE_SPD + (CHARGE_SPEED - DRIBBLE_SPD) * 0.5
        r.vx, r.vy = move_toward(rel_ball, drive_speed,
                                  ramp_dist=120, stop_dist=0)
        # Blend rotation: face ball far out, face aim when close
        blend = clamp(1.0 - d_ball / KICK_RANGE, 0.0, 1.0)
        w_ball = ang_ball * TURN_GAIN
        w_aim = ang_aim * TURN_GAIN * 1.2
        r.w = clamp(w_ball * (1.0 - blend) + w_aim * blend,
                     -MAX_W, MAX_W)

        if ks.near_ball_since == 0.0:
            ks.near_ball_since = now

        r.committed_side = ks.committed_side
        return r

    # ==================================================================
    #  P3: Arc approach — get behind ball relative to aim, drive in
    #
    #  Once lined up behind ball, switch to dribble speed and charge
    #  straight at the ball with aim-aligned rotation.
    # ==================================================================
    ks.near_ball_since = 0.0

    nav, ks.committed_side, is_behind = compute_arc_nav(
        robot_xy=(me[0], me[1]),
        ball=ball, aim=aim,
        behind_dist=BEHIND_DIST,
        avoid_radius=AVOID_RADIUS,
        committed_side=ks.committed_side,
    )

    rel_nav = world2robot(me, nav)
    d_nav = math.hypot(rel_nav[0], rel_nav[1])

    if is_behind and d_nav < 250 and d_ball < BALL_NEAR:
        # Lined up behind — charge straight at ball
        r.dribble = 1
        r.vx, r.vy = move_toward(rel_ball, CHARGE_SPEED * 0.8,
                                  ramp_dist=250, stop_dist=0)
        r.w = clamp(ang_aim * TURN_GAIN * 0.8, -MAX_W, MAX_W)
    else:
        # Still arcing around — navigate to behind-ball point
        r.vx, r.vy = move_toward(rel_nav, APPROACH_SPD,
                                  ramp_dist=350, stop_dist=10)
        r.w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
        # Slow down translation when facing far from ball
        r.vx, r.vy = turn_then_move(r.vx, r.vy, r.w, abs(ang_ball))

    r.committed_side = ks.committed_side
    return r
