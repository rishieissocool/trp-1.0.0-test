"""
Striker with possession awareness — approach, align, kick.

Uses the shared kick_engine for reliable approach + alignment + sustained
kick bursts.  In 1v1 mode the two strikers won't both blindly chase the
ball; instead, only the possessor attacks while the other repositions.

Behaviors:
  POSSESS  — I have the ball or it's loose near me -> kick_engine
  SHADOW   — opponent has the ball -> stay between ball and my goal
  WAIT     — ball in penalty box -> hold outside
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
    turn_then_move, ball_velocity, update_ball_history, predict_ball,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.robot.diamond_nav import DiamondNav
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
APPROACH_SPD     = CRUISE_SPEED
WAIT_SPD         = CRUISE_SPEED * 0.6
SHADOW_SPD       = CRUISE_SPEED * 0.7
BALL_MEMORY_TIME = 0.5
POSSESS_DIST     = 400      # mm — opponent "has" ball if closer than this
CHALLENGE_DIST   = 1200     # mm — only challenge when ball this close to me
SHADOW_DEPTH     = 800      # mm — stay well back from ball when shadowing
MIN_SHADOW_GAP   = 900      # mm — minimum distance to keep from ball when opp has it


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


def _get_opponent(frame, is_yellow):
    """Find the closest opponent robot, return (x, y, orientation) or None."""
    opp_yellow = not is_yellow
    best = None
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            if best is None:
                best = (float(op[0]), float(op[1]), float(op[2]))
        except Exception:
            continue
    return best


def _opponent_has_ball(opp, ball):
    """Check if opponent is possessing the ball (close + ball in front)."""
    if opp is None or ball is None:
        return False
    d = math.hypot(opp[0] - ball[0], opp[1] - ball[1])
    if d > POSSESS_DIST:
        return False
    # Check if ball is roughly in front of opponent
    rel = world2robot(opp, ball)
    return rel[0] > -30  # ball not behind them


# -- Main loop ------------------------------------------------------------

def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    frame = None
    last_ft = 0.0
    last_ball = None
    last_ball_time = 0.0
    last_d_ball = float('inf')
    prev_obs_pos = {}

    ball_history = []
    last_ball_xy = None

    ks = KickState()
    dnav = DiamondNav()

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

        # -- Ball velocity ----------------------------------------------
        if ball_visible:
            last_ball_xy = update_ball_history(
                ball_history, now, ball, last_ball_xy)
        bvx, bvy, bspeed = ball_velocity(ball_history)

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        # Goal positions
        if is_yellow:
            goal_x = -HALF_LEN if us_positive else HALF_LEN
        else:
            goal_x = HALF_LEN if us_positive else -HALF_LEN
        our_goal_x = -goal_x

        # -- Robot-local vectors ----------------------------------------
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if ball_visible:
            last_d_ball = d_ball

        # -- Opponent info ----------------------------------------------
        opp = _get_opponent(frame, is_yellow)
        opp_has_ball = _opponent_has_ball(opp, ball)
        opp_dist_to_ball = math.hypot(opp[0] - ball[0], opp[1] - ball[1]) \
            if opp is not None else float('inf')

        aim = _pick_aim(ball, goal_x, frame, is_yellow)

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ==============================================================
        #  SHADOW — opponent has possession, give them space
        #
        #  Stay well back on the line between ball and our goal.
        #  Only engage when ball becomes loose or comes to us.
        # ==============================================================
        if opp_has_ball and d_ball > CHALLENGE_DIST and not ks.bursting:
            ks.reset()

            bx, by = ball
            # Vector from ball toward our goal
            dx = our_goal_x - bx
            dy = 0.0 - by
            dd = max(math.hypot(dx, dy), 1.0)

            # Shadow point: MIN_SHADOW_GAP behind ball, on the ball-goal line
            # but no further than halfway to our goal
            gap = max(MIN_SHADOW_GAP, dd * 0.4)
            sx = bx + dx / dd * gap
            sy = by + dy / dd * gap
            sx = max(-HALF_LEN + 200, min(HALF_LEN - 200, sx))
            sy = max(-HALF_WID + 150, min(HALF_WID - 150, sy))

            shadow_pt = (sx, sy)
            # Use diamond planner to navigate around obstacles to shadow position
            wp = dnav.next_waypoint(frame, is_yellow, robot_id, rpos, shadow_pt)
            nav_target = wp if wp is not None else shadow_pt
            rel_shadow = world2robot(rpos, nav_target)
            vx, vy = move_toward(rel_shadow, SHADOW_SPD,
                                  ramp_dist=400, stop_dist=80)
            rel_b = world2robot(rpos, ball)
            ang_b = math.atan2(rel_b[1], rel_b[0])
            w = clamp(ang_b * TURN_GAIN, -MAX_W, MAX_W)

            # If ball comes toward me (opponent kicked/lost), intercept
            if bspeed > 200:
                dx_me = rpos[0] - ball[0]
                dy_me = rpos[1] - ball[1]
                dd_me = max(math.hypot(dx_me, dy_me), 1.0)
                approach_val = (bvx * dx_me + bvy * dy_me) / dd_me
                if approach_val > 150:
                    t_arr = max(dd_me / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(ball, (bvx, bvy), min(t_arr, 1.5))
                    dribble = 1
                    rel_int = world2robot(rpos, intercept)
                    vx, vy = move_toward(rel_int, APPROACH_SPD,
                                          ramp_dist=500, stop_dist=30)
                    rel_b2 = world2robot(rpos, ball)
                    ang_b2 = math.atan2(rel_b2[1], rel_b2[0])
                    w = clamp(ang_b2 * TURN_GAIN, -MAX_W, MAX_W)

        # ==============================================================
        #  POSSESS — ball is mine or loose, kick engine handles it
        # ==============================================================
        else:
            kr = kick_tick(ks, rpos, ball, aim, now, rel_ball, d_ball)
            vx, vy, w = kr.vx, kr.vy, kr.w
            kick, dribble = kr.kick, kr.dribble

        # -- Obstacle avoidance (reduced when dribbling/kicking) --------
        if not ks.bursting:
            avoid_vx, avoid_vy, _closest, prev_obs_pos = _compute_avoidance(
                rpos, frame, is_yellow, robot_id, rel_ball, prev_obs_pos)
            if dribble == 1 or kick == 1:
                avoid_vx *= 0.3
                avoid_vy *= 0.3
            vx += avoid_vx
            vy += avoid_vy

        # -- Face ball when idle ----------------------------------------
        if kick == 0 and dribble == 0 and not ks.bursting:
            if abs(ang_ball) < 0.04:
                w = 0.0
            else:
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # -- Turn-then-move: slow down when facing away from target ------
        if not ks.bursting:
            vx, vy = turn_then_move(vx, vy, w, abs(ang_ball))

        # -- Wall braking -----------------------------------------------
        vx, vy = wall_brake(rpos[0], rpos[1], vx, vy)

        # -- Rotation compensation --------------------------------------
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
