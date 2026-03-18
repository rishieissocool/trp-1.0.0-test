"""
High-performance path planning and velocity computation for robot movement.

Key features:
- Full-speed operation (no artificial throttle)
- Smooth proportional angular velocity (no step-function jerkiness)
- Trapezoidal speed profiles for fast yet precise arrivals
- Curved-path support for simultaneous move+rotate
- Arc-based approach for getting behind the ball without looping
"""

import math
from typing import Tuple, Optional

from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.constants import MAX_W, PP_GAIN, PP_MIN_IMPULSE, HALF_WID


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def move_toward(
    robot_pos: Tuple[float, float, float],
    target_world: Tuple[float, float],
    max_speed: float,
    stop_radius: float = 40.0,
    ramp_dist: float = 350.0,
    min_speed: float = 0.25,
) -> Tuple[float, float]:
    """
    Compute (vx, vy) in robot frame to move toward a world-frame target.
    Trapezoidal profile: full speed when far, smooth ramp to zero near target.
    """
    rel = world2robot(robot_pos, target_world)
    dist = math.hypot(rel[0], rel[1])
    if dist < stop_radius:
        return 0.0, 0.0
    if dist < ramp_dist:
        t = (dist - stop_radius) / max(ramp_dist - stop_radius, 1.0)
        # Quadratic ramp for smoother deceleration
        speed = max(max_speed * t * t, min_speed)
    else:
        speed = max_speed
    if dist < 1e-6:
        return 0.0, 0.0
    vx = (rel[0] / dist) * speed
    vy = (rel[1] / dist) * speed
    return vx, vy


def move_toward_relative(
    rel_target: Tuple[float, float],
    max_speed: float,
    stop_radius: float = 40.0,
    ramp_dist: float = 350.0,
    min_speed: float = 0.25,
) -> Tuple[float, float]:
    """
    Same as move_toward but target is already in robot frame.
    """
    dist = math.hypot(rel_target[0], rel_target[1])
    if dist < stop_radius:
        return 0.0, 0.0
    if dist < ramp_dist:
        t = (dist - stop_radius) / max(ramp_dist - stop_radius, 1.0)
        speed = max(max_speed * t * t, min_speed)
    else:
        speed = max_speed
    if dist < 1e-6:
        return 0.0, 0.0
    vx = (rel_target[0] / dist) * speed
    vy = (rel_target[1] / dist) * speed
    return vx, vy


def turn_toward(
    rel_target: Tuple[float, float],
    epsilon: float = 0.05,
    speed_scale: float = 1.0,
    max_w: float = MAX_W,
    gain: float = PP_GAIN,
) -> float:
    """
    Smooth proportional angular velocity to face rel_target in robot frame.
    Uses proportional control with saturation instead of step function.
    """
    if rel_target is None or (abs(rel_target[0]) < 1e-9 and abs(rel_target[1]) < 1e-9):
        return 0.0
    angle = math.atan2(rel_target[1], rel_target[0])
    if abs(angle) < epsilon:
        return 0.0
    # Proportional control with minimum impulse for small angles
    w = speed_scale * gain * angle
    # Minimum angular velocity to overcome friction when close to target
    if 0 < abs(w) < PP_MIN_IMPULSE:
        w = math.copysign(PP_MIN_IMPULSE, w)
    return clamp(w, -max_w, max_w)


def move_and_face(
    robot_pos: Tuple[float, float, float],
    move_target_world: Tuple[float, float],
    face_target_world: Tuple[float, float],
    max_linear_speed: float,
    max_angular_speed: float = 9.0,
    stop_radius: float = 40.0,
    ramp_dist: float = 350.0,
    turn_epsilon: float = 0.05,
) -> Tuple[float, float, float]:
    """
    (vx, vy, w) to move toward move_target while turning to face face_target.
    """
    rel_move = world2robot(robot_pos, move_target_world)
    rel_face = world2robot(robot_pos, face_target_world)
    vx, vy = move_toward_relative(
        rel_move, max_linear_speed,
        stop_radius=stop_radius, ramp_dist=ramp_dist,
    )
    w = turn_toward(rel_face, epsilon=turn_epsilon, max_w=max_angular_speed)
    return vx, vy, w


# ═══════════════════════════════════════════════════════════════════
#  ARC-BASED APPROACH — smooth curved path to get behind the ball
# ═══════════════════════════════════════════════════════════════════

def compute_arc_nav(
    robot_xy: Tuple[float, float],
    ball: Tuple[float, float],
    aim: Tuple[float, float],
    behind_dist: float,
    avoid_radius: float,
    committed_side: Optional[int],
) -> Tuple[Tuple[float, float], int, bool]:
    """
    Compute a navigation target for approaching behind the ball without
    crossing through it.  Uses arc-based pathing with committed-side
    hysteresis to prevent the oscillation / looping that happens when the
    robot keeps flipping between left and right detours.

    Args:
        robot_xy:       (x, y) robot position in world frame
        ball:           (x, y) ball position
        aim:            (x, y) aim target (usually inside opponent goal)
        behind_dist:    how far behind the ball to line up
        avoid_radius:   minimum clearance from ball while arcing
        committed_side: +1 / -1 from previous tick, or None on first call

    Returns:
        (nav_target, updated_committed_side, robot_is_behind_ball)
    """
    # Unit vector ball → aim
    ba_dx = aim[0] - ball[0]
    ba_dy = aim[1] - ball[1]
    ba_d = max(math.hypot(ba_dx, ba_dy), 1.0)
    ux, uy = ba_dx / ba_d, ba_dy / ba_d

    # Perpendicular (90° CCW of aim direction)
    px, py = -uy, ux

    # Behind-ball point
    behind = (ball[0] - ux * behind_dist, ball[1] - uy * behind_dist)

    # Robot relative to ball, decomposed along aim axis
    rbx = robot_xy[0] - ball[0]
    rby = robot_xy[1] - ball[1]
    along = rbx * ux + rby * uy          # >0 → aim-side (wrong side)
    perp  = rbx * px + rby * py          # lateral offset
    d_ball = math.hypot(rbx, rby)

    # ── Do we need to arc? ──────────────────────────────────────
    # Arc required when robot is on wrong side AND close enough that a
    # straight-line to behind point would cross near the ball.
    need_arc = (along > -behind_dist * 0.15) and (d_ball < avoid_radius * 3.0)
    # Also arc if very close to ball from any front angle
    if d_ball < avoid_radius * 0.9 and along > -behind_dist * 0.5:
        need_arc = True

    if not need_arc:
        # Already behind — go straight to behind point
        side = committed_side if committed_side is not None else (1 if perp >= 0 else -1)
        return behind, side, True

    # ── Commit to a side with strong hysteresis ─────────────────
    HYSTERESIS = avoid_radius * 0.6

    if committed_side is None:
        if abs(perp) < 80:
            # Nearly on the center line — pick side considering field walls
            if abs(robot_xy[1]) > HALF_WID - 350:
                committed_side = -1 if robot_xy[1] > 0 else 1
            else:
                committed_side = 1 if perp >= 0 else -1
        else:
            committed_side = 1 if perp > 0 else -1
    else:
        # Only flip if robot has moved well past center to the other side
        if committed_side == 1 and perp < -HYSTERESIS:
            committed_side = -1
        elif committed_side == -1 and perp > HYSTERESIS:
            committed_side = 1

    # ── Compute arc waypoint ────────────────────────────────────
    # Blend between "behind" direction and "side" direction based on
    # how deep on the wrong side we are.  This creates a natural
    # sweeping curve: deeply wrong → aim sideways; nearly behind →
    # aim toward behind point.
    wrong_ratio = clamp(
        (along + behind_dist * 0.3) / max(avoid_radius * 2.0, 1.0),
        0.0, 1.0,
    )

    arc_r = avoid_radius * 1.3   # clearance radius

    # "behind" unit direction
    bk_x, bk_y = -ux, -uy
    # "side" unit direction
    sd_x, sd_y = px * committed_side, py * committed_side

    # Blend
    blend = wrong_ratio * 0.85
    dir_x = bk_x * (1.0 - blend) + sd_x * blend
    dir_y = bk_y * (1.0 - blend) + sd_y * blend
    dir_d = max(math.hypot(dir_x, dir_y), 1e-9)
    dir_x /= dir_d
    dir_y /= dir_d

    nav = (ball[0] + dir_x * arc_r, ball[1] + dir_y * arc_r)
    return nav, committed_side, False


def move_with_ramp(
    rel: Tuple[float, float],
    speed: float,
    ramp_dist: float = 350.0,
    stop_dist: float = 25.0,
    min_speed: float = 0.10,
) -> Tuple[float, float]:
    """
    Move toward a robot-local point with linear deceleration near target.
    Better than raw unit-vector * speed because it prevents overshoot.
    """
    d = math.hypot(rel[0], rel[1])
    if d < stop_dist:
        return 0.0, 0.0
    if d < ramp_dist:
        t = (d - stop_dist) / max(ramp_dist - stop_dist, 1.0)
        speed = max(speed * t, min_speed)
    return (rel[0] / d) * speed, (rel[1] / d) * speed
