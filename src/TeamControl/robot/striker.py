"""
Simple striker — approach, align, kick.

Three behaviors, checked in order each tick:
  1. WAIT     — ball inside a penalty box → hold outside, face ball
  2. KICK     — ball close and in front → dribble, align to goal, kick
  3. APPROACH — default: arc around to get behind ball, drive toward it
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, compute_arc_nav, wall_brake, rotation_compensate,
)
from TeamControl.robot.constants import (
    FIELD_LENGTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, GOAL_DEPTH,
    PENALTY_DEPTH, PENALTY_HW,
    BEHIND_DIST, KICK_RANGE, BALL_NEAR, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
    DEFENSE_DEPTH,
)

# ── Tuning ───────────────────────────────────────────────────────
APPROACH_SPD   = CHARGE_SPEED       # moderate approach
DRIBBLE_SPD    = DRIBBLE_SPEED      # gentle when close
WAIT_SPD       = CRUISE_SPEED * 0.6 # repositioning outside box
DWELL_TIME     = 0.25               # seconds ball must be stable before kick
KICK_ALIGN_TOL = 0.12               # rad — alignment tolerance for dwell


def _in_penalty_box(px, py, goal_x):
    return abs(px - goal_x) < PENALTY_DEPTH and abs(py) < PENALTY_HW


def _pick_aim(ball, goal_x, frame, is_yellow):
    """Pick aim point in goal mouth, away from opponent goalie."""
    aim_inward = 1 if goal_x > 0 else -1
    aim_x = goal_x + aim_inward * (GOAL_DEPTH * 0.5)

    # Find opponent goalie
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

    # Pick the side of goal furthest from goalie
    if gk_y is not None:
        aim_y = -GOAL_HW * 0.6 if gk_y > 0 else GOAL_HW * 0.6
    else:
        aim_y = 0.0

    return (aim_x, aim_y)


# ── Main loop ─────────────────────────────────────────────────────

BALL_MEMORY_TIME = 0.5    # seconds to trust last-known ball when occluded

def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    last_kick = 0.0
    frame = None
    last_ft = 0.0
    committed_side = None
    dwell_start = 0.0       # when alignment started
    aligned = False         # True while continuously aligned
    last_ball = None         # last known ball (x, y)
    last_ball_time = 0.0     # when ball was last seen
    last_d_ball = float('inf')  # distance to ball when last seen

    while is_running.is_set():
        now = time.time()

        # ── Get frame ────────────────────────────────────────
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

        # ── Resolve ball position (use memory if occluded) ───
        ball_visible = frame.ball is not None
        if ball_visible:
            bp = frame.ball.position
            ball = (float(bp[0]), float(bp[1]))
            last_ball = ball
            last_ball_time = now
        elif last_ball is not None and \
             (now - last_ball_time) < BALL_MEMORY_TIME and \
             last_d_ball < BALL_NEAR:
            # Ball disappeared while close — likely occluded by us
            ball = last_ball
        else:
            time.sleep(LOOP_RATE)
            continue

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        # Opponent goal — where we want to score
        if is_yellow:
            goal_x = -HALF_LEN if us_positive else HALF_LEN
        else:
            goal_x = HALF_LEN if us_positive else -HALF_LEN

        our_goal_x = -goal_x

        # ── Robot-local vectors ──────────────────────────────
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if ball_visible:
            last_d_ball = d_ball

        aim = _pick_aim(ball, goal_x, frame, is_yellow)
        rel_aim = world2robot(rpos, aim)
        ang_aim = math.atan2(rel_aim[1], rel_aim[0])

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ═════════════════════════════════════════════════════
        #  1. WAIT — ball inside a penalty box, hold outside
        # ═════════════════════════════════════════════════════
        if _in_penalty_box(ball[0], ball[1], goal_x) or \
           _in_penalty_box(ball[0], ball[1], our_goal_x):
            box_x = goal_x if _in_penalty_box(ball[0], ball[1], goal_x) \
                else our_goal_x
            inward = -1.0 if box_x > 0 else 1.0
            wait_pt = (box_x + inward * (PENALTY_DEPTH + 120),
                       clamp(ball[1], -PENALTY_HW, PENALTY_HW))
            rel_w = world2robot(rpos, wait_pt)
            vx, vy = move_toward(rel_w, WAIT_SPD)
            w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
            committed_side = None

        # ═════════════════════════════════════════════════════
        #  2. KICK — ball close and in front, align and shoot
        # ═════════════════════════════════════════════════════
        elif d_ball < KICK_RANGE and (rel_ball[0] > 0 or not ball_visible):
            dribble = 1
            vx, vy = move_toward(rel_ball, DRIBBLE_SPD * 0.4, ramp_dist=120,
                                 stop_dist=10)
            committed_side = None

            # Rotate toward aim target
            w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

            # Ball occluded while very close — kick immediately
            if not ball_visible and (now - last_kick) > KICK_COOLDOWN:
                kick = 1
                dribble = 0
                vx = DRIBBLE_SPD
                vy = 0.0
                last_kick = now
                aligned = False
            else:
                # Kick when aligned to goal AND held stable for DWELL_TIME
                dx_goal = abs(goal_x - ball[0])
                kick_tol = clamp(
                    math.atan2(GOAL_WIDTH * 0.35, max(dx_goal, 350)),
                    0.08, 0.35)

                if abs(ang_aim) < KICK_ALIGN_TOL:
                    if not aligned:
                        aligned = True
                        dwell_start = now
                    elif (now - dwell_start) >= DWELL_TIME and \
                         abs(ang_aim) < kick_tol and \
                         (now - last_kick) > KICK_COOLDOWN:
                        kick = 1
                        dribble = 0
                        vx = DRIBBLE_SPD
                        vy = 0.0
                        last_kick = now
                        aligned = False
                else:
                    aligned = False

        # ═════════════════════════════════════════════════════
        #  3. APPROACH — arc around ball, get behind, drive in
        # ═════════════════════════════════════════════════════
        else:
            nav, committed_side, is_behind = compute_arc_nav(
                robot_xy=(rpos[0], rpos[1]),
                ball=ball,
                aim=aim,
                behind_dist=BEHIND_DIST,
                avoid_radius=AVOID_RADIUS,
                committed_side=committed_side,
            )

            # Keep out of penalty boxes
            for gx in (goal_x, our_goal_x):
                if _in_penalty_box(nav[0], nav[1], gx):
                    inward = -1.0 if gx > 0 else 1.0
                    nav = (gx + inward * (PENALTY_DEPTH + 120), nav[1])

            rel_nav = world2robot(rpos, nav)
            d_nav = math.hypot(rel_nav[0], rel_nav[1])

            if is_behind and d_nav < 200 and d_ball < BALL_NEAR:
                # Lined up — gentle approach to ball
                dribble = 1
                vx, vy = move_toward(rel_ball, DRIBBLE_SPD, ramp_dist=350,
                                     stop_dist=10)
                w = clamp(ang_aim * TURN_GAIN * 0.7, -MAX_W, MAX_W)
            else:
                # Navigate toward arc waypoint or behind-ball point
                vx, vy = move_toward(rel_nav, APPROACH_SPD, ramp_dist=400,
                                     stop_dist=10)
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # ── Wall braking ───────────────────────────────────
        vx, vy = wall_brake(rpos[0], rpos[1], vx, vy)

        # ── Rotation compensation ──────────────────────────
        vx, vy = rotation_compensate(vx, vy, w)

        # ── Always face the ball ───────────────────────────
        if kick == 0 and dribble == 0:
            ang_ball = math.atan2(rel_ball[1], rel_ball[0])
            if abs(ang_ball) < 0.04:
                w = 0.0
            else:
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
