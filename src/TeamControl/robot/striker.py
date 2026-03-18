"""
Striker AI — arc-based approach, committed-side hysteresis, smooth deceleration.

Eliminates the looping / oscillation that plagued the old approach logic.
The robot now takes clean curved paths around the ball to line up behind it,
then drives straight through toward the aim target.

States:
  WAIT      — ball in a penalty box → hold outside, face ball
  APPROACH  — arc around ball if on wrong side, then navigate to behind-ball
  CHARGE    — close to behind-ball point → drive through ball, dribble
  POSSESS   — ball on dribbler → face aim, kick when aligned
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.path_planner import compute_arc_nav, move_with_ramp
from TeamControl.robot.constants import (
    FIELD_LENGTH, FIELD_WIDTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, GOAL_DEPTH,
    PENALTY_DEPTH, PENALTY_HW,
    BEHIND_DIST, KICK_RANGE, BALL_NEAR, AVOID_RADIUS,
    SPRINT_SPEED, CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
    BALL_HISTORY_SIZE, FRICTION, INTERCEPT_MAX_T, INTERCEPT_STEPS,
    BALL_MOVING_THRESH, PRESSURE_DIST,
    DEFENSE_DEPTH,
)


# ── Helpers ───────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _move(rel, speed, ramp_dist=350.0, stop_dist=25.0, min_speed=0.10):
    """Unit-direction velocity with linear deceleration near target."""
    d = math.hypot(rel[0], rel[1])
    if d < stop_dist:
        return 0.0, 0.0
    if d < ramp_dist:
        t = (d - stop_dist) / max(ramp_dist - stop_dist, 1.0)
        speed = max(speed * t, min_speed)
    return (rel[0] / d) * speed, (rel[1] / d) * speed


def _in_penalty_box(px, py, goal_x):
    return abs(px - goal_x) < PENALTY_DEPTH and abs(py) < PENALTY_HW


def _predict_ball(ball, bvx, bvy, dt):
    """Predict ball position after *dt* seconds with friction decay."""
    bx, by = ball
    vx, vy = bvx, bvy
    t, step = 0.0, 0.02
    while t < dt:
        s = min(step, dt - t)
        bx += vx * s
        by += vy * s
        f = max(1.0 - FRICTION * s, 0.0)
        vx *= f
        vy *= f
        t += s
        if math.hypot(vx, vy) < 30:
            break
    return (_clamp(bx, -HALF_LEN, HALF_LEN),
            _clamp(by, -HALF_WID, HALF_WID))


def _best_intercept(rpos, ball, bvx, bvy):
    """Where should the robot go to meet a moving ball?"""
    best_pt = ball
    best_t = math.hypot(rpos[0] - ball[0], rpos[1] - ball[1]) / max(SPRINT_SPEED * 1000, 1)
    for i in range(1, INTERCEPT_STEPS + 1):
        dt = i * (INTERCEPT_MAX_T / INTERCEPT_STEPS)
        bp = _predict_ball(ball, bvx, bvy, dt)
        t_robot = math.hypot(rpos[0] - bp[0], rpos[1] - bp[1]) / max(SPRINT_SPEED * 1000, 1)
        if t_robot <= dt and t_robot < best_t:
            best_pt = bp
            best_t = t_robot
    return best_pt


def _pick_aim(ball, goal_x, goalie_y, goalie_x=None):
    """Pick the best aim point inside the goal mouth."""
    aim_inward = 1 if goal_x > 0 else -1
    aim_x = goal_x + aim_inward * (GOAL_DEPTH * 0.5)

    best_score = -999
    best = (aim_x, 0)

    for frac in (-0.35, -0.18, 0.0, 0.18, 0.35):
        ty = frac * GOAL_WIDTH
        score = abs(ty) / GOAL_HW * 0.4

        if goalie_y is not None:
            gk_dist = abs(ty - goalie_y)
            if gk_dist < 150:
                score -= 0.8
            elif gk_dist < 300:
                score -= 0.3
            else:
                score += 0.4

        d_shot = math.hypot(ball[0] - aim_x, ball[1] - ty)
        score += max(0, 0.3 - d_shot / 8000)

        dx = abs(ball[0] - goal_x)
        if dx > 100:
            angle = 2.0 * math.atan2(GOAL_HW, dx)
            score += min(angle / math.radians(30), 0.4)

        if score > best_score:
            best_score = score
            best = (aim_x, ty)

    return best


def _find_goalie(frame, is_yellow, goal_x):
    """Return (x, y) of the opponent goalie, or (None, None)."""
    opp_yellow = not is_yellow
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            ox, oy = float(op[0]), float(op[1])
            if abs(ox - goal_x) < DEFENSE_DEPTH:
                return ox, oy
        except Exception:
            continue
    return None, None


def _nearest_opp(rpos, frame, is_yellow, robot_id):
    """Distance to closest opponent."""
    opp_yellow = not is_yellow
    best = 9999.0
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            d = math.hypot(float(op[0]) - rpos[0], float(op[1]) - rpos[1])
            if d < best:
                best = d
        except Exception:
            continue
    return best


# ── Main loop ─────────────────────────────────────────────────────

def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    last_kick = 0.0
    frame = None
    last_ft = 0.0
    ball_hist = []
    last_bxy = None

    # ── Persistent state for arc approach ─────────────────────────
    committed_side = None   # +1 / -1: which side to arc around the ball

    while is_running.is_set():
        now = time.time()

        # ── Fetch frame ────────────────────────────────────────
        if now - last_ft > FRAME_INTERVAL:
            try:
                f = wm.get_latest_frame()
                if f is not None:
                    frame = f
            except Exception:
                pass
            last_ft = now

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
        ball = (float(bp[0]), float(bp[1]))
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        if is_yellow:
            goal_x = -HALF_LEN if us_positive else HALF_LEN
        else:
            goal_x = HALF_LEN if us_positive else -HALF_LEN

        # ── Ball velocity ──────────────────────────────────────
        if last_bxy is None or ball != last_bxy:
            ball_hist.append((now, ball[0], ball[1]))
            if len(ball_hist) > BALL_HISTORY_SIZE:
                ball_hist.pop(0)
            last_bxy = ball

        bvx = bvy = bspeed = 0.0
        if len(ball_hist) >= 2:
            dt = ball_hist[-1][0] - ball_hist[0][0]
            if dt > 0.02:
                bvx = (ball_hist[-1][1] - ball_hist[0][1]) / dt
                bvy = (ball_hist[-1][2] - ball_hist[0][2]) / dt
                bspeed = math.hypot(bvx, bvy)

        # ── Where to meet the ball ─────────────────────────────
        if bspeed > BALL_MOVING_THRESH:
            target_ball = _best_intercept((rpos[0], rpos[1]), ball,
                                          bvx, bvy)
        else:
            target_ball = ball

        # ── Aim & goalie ───────────────────────────────────────
        gk_x, gk_y = _find_goalie(frame, is_yellow, goal_x)
        aim = _pick_aim(ball, goal_x, gk_y, gk_x)

        # ── Robot-local vectors ────────────────────────────────
        rel_ball = world2robot(rpos, ball)
        rel_aim = world2robot(rpos, aim)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        ang_aim = math.atan2(rel_aim[1], rel_aim[0])

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        our_goal_x = -goal_x

        # ═══════════════════════════════════════════════════════
        #  WAIT — ball inside a penalty box (can't enter either)
        # ═══════════════════════════════════════════════════════
        if _in_penalty_box(ball[0], ball[1], goal_x):
            inward = -1.0 if goal_x > 0 else 1.0
            wait = (goal_x + inward * (PENALTY_DEPTH + 120),
                    _clamp(ball[1], -PENALTY_HW, PENALTY_HW))
            rel_w = world2robot(rpos, wait)
            vx, vy = _move(rel_w, CRUISE_SPEED)
            w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
            committed_side = None   # reset arc state

        elif _in_penalty_box(ball[0], ball[1], our_goal_x):
            inward = -1.0 if our_goal_x > 0 else 1.0
            wait = (our_goal_x + inward * (PENALTY_DEPTH + 120),
                    _clamp(ball[1], -PENALTY_HW, PENALTY_HW))
            rel_w = world2robot(rpos, wait)
            vx, vy = _move(rel_w, CRUISE_SPEED)
            w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
            committed_side = None

        # ═══════════════════════════════════════════════════════
        #  POSSESS — ball on the dribbler
        # ═══════════════════════════════════════════════════════
        elif d_ball < KICK_RANGE and rel_ball[0] > 0:
            dribble = 1
            vx, vy = _move(rel_ball, DRIBBLE_SPEED * 0.5, ramp_dist=150, stop_dist=15)
            committed_side = None   # reset arc state

            if abs(ang_ball) > 0.4:
                # Ball not centered — rotate to face it first
                w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
            else:
                # Ball centered — rotate toward aim, shoot when ready
                w = _clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

                dx_goal = abs(goal_x - ball[0])
                kick_tol = _clamp(
                    math.atan2(GOAL_WIDTH * 0.35, max(dx_goal, 350)),
                    0.08, 0.35)

                if (abs(ang_aim) < kick_tol
                        and (now - last_kick) > KICK_COOLDOWN):
                    kick = 1
                    dribble = 0
                    vx = CHARGE_SPEED
                    vy = 0.0
                    last_kick = now

                elif (_nearest_opp(rpos, frame, is_yellow, robot_id)
                      < PRESSURE_DIST
                      and abs(ang_aim) < 0.7
                      and (now - last_kick) > KICK_COOLDOWN):
                    kick = 1
                    dribble = 0
                    vx = CHARGE_SPEED
                    vy = 0.0
                    last_kick = now

        # ═══════════════════════════════════════════════════════
        #  APPROACH — arc around ball, then drive through
        # ═══════════════════════════════════════════════════════
        else:
            # Use arc navigation to get behind the ball smoothly
            nav, committed_side, is_behind = compute_arc_nav(
                robot_xy=(rpos[0], rpos[1]),
                ball=target_ball,
                aim=aim,
                behind_dist=BEHIND_DIST,
                avoid_radius=AVOID_RADIUS,
                committed_side=committed_side,
            )

            # Clamp nav to stay out of both penalty boxes
            for gx in (goal_x, our_goal_x):
                if _in_penalty_box(nav[0], nav[1], gx):
                    inward = -1.0 if gx > 0 else 1.0
                    nav = (gx + inward * (PENALTY_DEPTH + 120), nav[1])

            rel_nav = world2robot(rpos, nav)
            d_nav = math.hypot(rel_nav[0], rel_nav[1])

            if is_behind and d_nav < 200 and d_ball < BALL_NEAR:
                # We've lined up behind the ball — charge through it
                dribble = 1
                vx, vy = _move(rel_ball, CHARGE_SPEED, ramp_dist=200, stop_dist=15)
                w = _clamp(ang_aim * TURN_GAIN * 0.7, -MAX_W, MAX_W)
            elif is_behind and d_nav < 350:
                # Close to behind point — approach at controlled speed
                dribble = 1 if d_ball < BALL_NEAR else 0
                vx, vy = _move(rel_nav, CHARGE_SPEED, ramp_dist=300, stop_dist=20)
                # Face the ball to be ready for contact
                w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
            else:
                # Navigate toward arc waypoint or behind-ball point
                if d_nav > 1200:
                    speed = SPRINT_SPEED
                elif d_nav > 500:
                    speed = CRUISE_SPEED
                else:
                    speed = CHARGE_SPEED
                vx, vy = _move(rel_nav, speed, ramp_dist=400, stop_dist=30)

                # Face the ball while approaching
                w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # ── Rotation compensation ───────────────────────────
        # Pre-rotate velocity to account for simultaneous rotation,
        # so the world-frame path stays on target while facing the ball.
        if abs(w) > 0.01:
            half_rot = -w * LOOP_RATE * 0.5
            cos_r = math.cos(half_rot)
            sin_r = math.sin(half_rot)
            vx, vy = vx * cos_r - vy * sin_r, vx * sin_r + vy * cos_r

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
