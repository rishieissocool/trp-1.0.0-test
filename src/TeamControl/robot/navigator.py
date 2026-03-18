"""
Ball-chasing navigation with smooth obstacle avoidance.

The robot pursues the ball at a moderate, smooth speed while curving
around any other robots in its path.  No fixed waypoints — the ball
IS the target.

Key features:
- Smooth, moderate speed (not too fast, not too slow)
- Tangential + repulsive obstacle avoidance for curved paths
- Deceleration ramp near the ball so the robot doesn't overshoot
- Faces the ball at all times
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
)
from TeamControl.robot.constants import (
    FIELD_LENGTH, FIELD_WIDTH, HALF_LEN, HALF_WID,
    CRUISE_SPEED, SPRINT_SPEED, MAX_W, TURN_GAIN,
    LOOP_RATE, FRAME_INTERVAL,
)

# ── Obstacle avoidance tuning ────────────────────────────────────
AVOID_DIST = 1000         # start avoiding at this range (mm)
AVOID_STRENGTH = 3.0      # base repulsive strength
AVOID_CRITICAL = 450      # very close → emergency avoidance

# ── Ball-chase speed ─────────────────────────────────────────────
CHASE_SPEED = CRUISE_SPEED * 0.85   # smooth, moderate pace
NEAR_BALL_DIST = 300                # start slowing down here
STOP_DIST = 80                      # stop this close to ball

# Keep old exports so main.py import doesn't break
WAYPOINTS_A = [
    (HALF_LEN - 500, HALF_WID - 200),
    (HALF_LEN - 500, -HALF_WID + 200),
    (-HALF_LEN + 500, -HALF_WID + 200),
    (-HALF_LEN + 500, HALF_WID - 200),
]
WAYPOINTS_B = [
    (-HALF_LEN + 500, -HALF_WID + 200),
    (-HALF_LEN + 500, HALF_WID - 200),
    (HALF_LEN - 500, HALF_WID - 200),
    (HALF_LEN - 500, -HALF_WID + 200),
]


def _compute_avoidance(rpos, frame, is_yellow, robot_id, rel_target,
                       prev_obs_pos=None):
    """
    Compute tangential + repulsive avoidance force from all other robots.
    Uses predictive avoidance: avoids where robots WILL be in ~0.3s, not
    just where they are now.
    Returns (avoid_vx, avoid_vy, closest_obstacle_distance, updated_obs_pos).
    """
    avoid_vx, avoid_vy = 0.0, 0.0
    closest_obs = 9999.0
    current_obs_pos = {}
    PREDICT_T = 0.5  # seconds to predict ahead

    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                op = other.position
                ox, oy = float(op[0]), float(op[1])
                obs_key = (color, oid)
                current_obs_pos[obs_key] = (ox, oy)

                pred_x, pred_y = ox, oy
                if prev_obs_pos and obs_key in prev_obs_pos:
                    px, py = prev_obs_pos[obs_key]
                    est_vx = (ox - px) / max(FRAME_INTERVAL, 0.01)
                    est_vy = (oy - py) / max(FRAME_INTERVAL, 0.01)
                    pred_x = ox + est_vx * PREDICT_T
                    pred_y = oy + est_vy * PREDICT_T

                avoid_x = ox * 0.3 + pred_x * 0.7
                avoid_y = oy * 0.3 + pred_y * 0.7

                rel_obs = world2robot(rpos, (avoid_x, avoid_y))
                d_obs = math.hypot(rel_obs[0], rel_obs[1])

                if d_obs < closest_obs:
                    closest_obs = d_obs

                if d_obs < AVOID_DIST and d_obs > 1:
                    if d_obs < AVOID_CRITICAL:
                        strength = AVOID_STRENGTH * 3.0
                    elif d_obs < AVOID_DIST * 0.5:
                        ratio = (AVOID_DIST - d_obs) / AVOID_DIST
                        strength = ratio * AVOID_STRENGTH * 1.8
                    else:
                        ratio = (AVOID_DIST - d_obs) / AVOID_DIST
                        strength = ratio * ratio * AVOID_STRENGTH

                    rep_x = -(rel_obs[0] / d_obs) * strength
                    rep_y = -(rel_obs[1] / d_obs) * strength

                    tang_x =  rel_obs[1] / d_obs
                    tang_y = -rel_obs[0] / d_obs
                    if tang_x * rel_target[0] + tang_y * rel_target[1] < 0:
                        tang_x, tang_y = -tang_x, -tang_y

                    avoid_vx += rep_x + tang_x * strength * 0.8
                    avoid_vy += rep_y + tang_y * strength * 0.8
            except Exception:
                continue

    return avoid_vx, avoid_vy, closest_obs, current_obs_pos


# ═════════════════════════════════════════════════════════════════
#  MAIN ENTRY POINT — ball chaser with obstacle avoidance
# ═════════════════════════════════════════════════════════════════

def run_navigator(is_running, dispatch_q, wm, robot_id, is_yellow,
                  waypoints=None):
    """
    Chase the ball smoothly while avoiding every other robot on the field.
    The `waypoints` argument is accepted for API compatibility but ignored.
    """
    frame = None
    prev_obs_pos = {}  # for predictive avoidance

    while is_running.is_set():
        now = time.time()

        # ── Fetch frame (every tick for fast obstacle reaction) ─
        try:
            f = wm.get_latest_frame()
            if f is not None:
                frame = f
        except Exception:
            pass

        if frame is None or frame.ball is None:
            time.sleep(LOOP_RATE)
            continue

        try:
            robot = frame.get_yellow_robots(isYellow=is_yellow,
                                            robot_id=robot_id)
        except Exception:
            time.sleep(LOOP_RATE)
            continue

        if isinstance(robot, int):
            time.sleep(LOOP_RATE)
            continue

        rp = robot.position
        bp = frame.ball.position
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))
        ball = (float(bp[0]), float(bp[1]))

        # ── Target: the ball ─────────────────────────────────
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])

        # ── Obstacle avoidance (predictive) ──────────────────
        avoid_vx, avoid_vy, closest_obs, prev_obs_pos = _compute_avoidance(
            rpos, frame, is_yellow, robot_id, rel_ball, prev_obs_pos)

        # ── Base navigation toward ball ──────────────────────
        if closest_obs < AVOID_CRITICAL:
            base_speed = CHASE_SPEED * 0.55
        elif closest_obs < AVOID_DIST * 0.6:
            base_speed = CHASE_SPEED * 0.75
        else:
            base_speed = CHASE_SPEED

        nav_vx, nav_vy = move_toward(rel_ball, base_speed,
                                     ramp_dist=NEAR_BALL_DIST,
                                     stop_dist=STOP_DIST,
                                     min_speed=0.06)

        # ── Blend navigation + avoidance ─────────────────────
        vx = nav_vx + avoid_vx
        vy = nav_vy + avoid_vy

        # Cap speed — allow higher when actively dodging
        speed = math.hypot(vx, vy)
        max_spd = CHASE_SPEED * 1.4 if closest_obs < AVOID_DIST else CHASE_SPEED * 1.15
        if speed > max_spd:
            vx = vx / speed * max_spd
            vy = vy / speed * max_spd

        # ── Wall braking ────────────────────────────────────
        vx, vy = wall_brake(rpos[0], rpos[1], vx, vy)

        # ── Face the ball ────────────────────────────────────
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if abs(ang_ball) < 0.05:
            w = 0.0
        else:
            w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # ── Rotation compensation ───────────────────────────
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=0, dribble=0, isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
