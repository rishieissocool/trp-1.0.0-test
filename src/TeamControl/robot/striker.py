"""
Simple striker — approach, align, kick using shared kick engine.

Behaviors checked each tick:
  1. WAIT  — ball inside penalty box -> hold outside, face ball
  2. KICK  — ball reachable -> kick engine handles approach + align + burst
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.robot.constants import (
    FIELD_LENGTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, GOAL_DEPTH,
    PENALTY_DEPTH, PENALTY_HW,
    KICK_RANGE, BALL_NEAR, BEHIND_DIST, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
    DEFENSE_DEPTH,
)

# -- Tuning ---------------------------------------------------------------
APPROACH_SPD   = CRUISE_SPEED
WAIT_SPD       = CRUISE_SPEED * 0.6

BALL_MEMORY_TIME = 0.5    # seconds to trust last-known ball when occluded


def _in_penalty_box(px, py, goal_x):
    return abs(px - goal_x) < PENALTY_DEPTH and abs(py) < PENALTY_HW


def _pick_aim(ball, goal_x, frame, is_yellow):
    """Pick aim point in goal mouth, away from opponent goalie."""
    aim_inward = 1 if goal_x > 0 else -1
    aim_x = goal_x + aim_inward * (GOAL_DEPTH * 0.5)

    gk_y = None
    opp_yellow = not is_yellow
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            if abs(float(op[0]) - goal_x) < DEFENSE_DEPTH:
                gk_y = float(op[1])
                break
        except Exception:
            continue

    if gk_y is not None:
        aim_y = -GOAL_HW * 0.6 if gk_y > 0 else GOAL_HW * 0.6
    else:
        aim_y = 0.0

    return (aim_x, aim_y)


# -- Main loop ------------------------------------------------------------

def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    frame = None
    last_ft = 0.0
    last_ball = None
    last_ball_time = 0.0
    last_d_ball = float('inf')
    prev_obs_pos = {}

    ks = KickState()

    while is_running.is_set():
        now = time.time()

        # -- Get frame --------------------------------------------------
        if now - last_ft > FRAME_INTERVAL:
            try:
                f = wm.get_latest_frame()
                if f is not None:
                    frame = f
            except Exception:
                pass
            last_ft = now

        if frame is None:
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
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))

        # -- Resolve ball (use memory if occluded) ----------------------
        ball_visible = frame.ball is not None
        if ball_visible:
            bp = frame.ball.position
            ball = (float(bp[0]), float(bp[1]))
            last_ball = ball
            last_ball_time = now
        elif last_ball is not None and \
             (now - last_ball_time) < BALL_MEMORY_TIME and \
             last_d_ball < BALL_NEAR:
            ball = last_ball
        else:
            time.sleep(LOOP_RATE)
            continue

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        if is_yellow:
            goal_x = -HALF_LEN if us_positive else HALF_LEN
        else:
            goal_x = HALF_LEN if us_positive else -HALF_LEN

        # -- Robot-local vectors ----------------------------------------
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if ball_visible:
            last_d_ball = d_ball

        aim = _pick_aim(ball, goal_x, frame, is_yellow)

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ==============================================================
        #  KICK ENGINE — handles approach, align, contact, burst
        # ==============================================================
        kr = kick_tick(ks, rpos, ball, aim, now, rel_ball, d_ball)
        vx, vy, w = kr.vx, kr.vy, kr.w
        kick, dribble = kr.kick, kr.dribble

        # -- Obstacle avoidance (reduced when dribbling/kicking) --------
        avoid_vx, avoid_vy, _closest, prev_obs_pos = _compute_avoidance(
            rpos, frame, is_yellow, robot_id, rel_ball, prev_obs_pos)
        if dribble == 1 or kick == 1:
            avoid_vx *= 0.3
            avoid_vy *= 0.3
        vx += avoid_vx
        vy += avoid_vy

        # -- Face ball when idle ----------------------------------------
        if kick == 0 and dribble == 0:
            if abs(ang_ball) < 0.04:
                w = 0.0
            else:
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # -- Wall braking -----------------------------------------------
        vx, vy = wall_brake(rpos[0], rpos[1], vx, vy)

        # -- Rotation compensation --------------------------------------
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
