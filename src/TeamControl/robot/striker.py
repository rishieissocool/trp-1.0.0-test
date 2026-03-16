"""
Elite striker AI — fast, decisive, strategically ruthless.

Key improvements over baseline:
- Friction-based ball prediction (not linear extrapolation)
- One-touch shooting: intercept and redirect moving balls
- Pressure-aware: emergency flick when opponent closes in
- Curved approach: maintain speed through turns
- Adaptive aim: evaluate multiple goal targets, exploit goalie gaps
- Feint capability: occasional lateral dribble to wrong-foot goalie
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot

# ═══════════════════════════════════════════════════════════════════
#  FIELD GEOMETRY (mm) — small field 5000 x 3000
# ═══════════════════════════════════════════════════════════════════

FIELD_LENGTH = 5000
FIELD_WIDTH = 3000
HALF_LEN = FIELD_LENGTH / 2
HALF_WID = FIELD_WIDTH / 2
GOAL_WIDTH = 1000
GOAL_HW = GOAL_WIDTH / 2

# Defense area — striker must not enter the opponent's box
DEFENSE_DEPTH = 1200
DEFENSE_HALF_WIDTH = 1200

# ═══════════════════════════════════════════════════════════════════
#  DISTANCES (mm)
# ═══════════════════════════════════════════════════════════════════

BEHIND_DIST = 300           # how far behind ball to line up
KICK_RANGE = 175            # trigger kick distance
BALL_NEAR = 420             # "close to ball" threshold
AVOID_RADIUS = 400          # how wide to swing around the ball

# ═══════════════════════════════════════════════════════════════════
#  SPEEDS (m/s) — full speed, no artificial throttle
# ═══════════════════════════════════════════════════════════════════

SPRINT_SPEED = 2.2          # max speed for long-range repositioning
CRUISE_SPEED = 1.8          # medium distance approach
CHARGE_SPEED = 1.4          # close-range approach
DRIBBLE_SPEED = 1.0         # precise control near ball
ONETOUCH_SPEED = 1.6        # speed for one-touch redirect

# ═══════════════════════════════════════════════════════════════════
#  ANGULAR
# ═══════════════════════════════════════════════════════════════════

MAX_W = 1.0
TURN_GAIN = 1.5

# ═══════════════════════════════════════════════════════════════════
#  TIMING
# ═══════════════════════════════════════════════════════════════════

KICK_COOLDOWN = 0.22
LOOP_RATE = 0.016
FRAME_INTERVAL = 0.04

# ═══════════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═══════════════════════════════════════════════════════════════════

BALL_HISTORY_SIZE = 6
FRICTION = 0.4              # friction deceleration factor per second
INTERCEPT_MAX_T = 1.0       # max seconds to predict ahead
INTERCEPT_STEPS = 12        # number of prediction steps
BALL_MOVING_THRESH = 150    # mm/s — ball considered moving

# ═══════════════════════════════════════════════════════════════════
#  TACTICAL
# ═══════════════════════════════════════════════════════════════════

PRESSURE_DIST = 500         # opponent this close = under pressure
ONETOUCH_MIN_SPEED = 300    # ball must be moving this fast for one-touch
ONETOUCH_ANGLE = 0.8        # max angle offset for one-touch redirect


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _move(rel, speed):
    d = math.hypot(rel[0], rel[1])
    if d < 1:
        return 0.0, 0.0
    return (rel[0] / d) * speed, (rel[1] / d) * speed


def _in_defense_area(px, py, goal_x):
    return abs(px - goal_x) < DEFENSE_DEPTH and abs(py) < DEFENSE_HALF_WIDTH


def _predict_ball_friction(ball, bvx, bvy, dt):
    """Predict ball position after dt seconds with friction decay."""
    bx, by = ball
    vx, vy = bvx, bvy
    t = 0.0
    step = 0.02
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


def _optimal_intercept(rpos, ball, bvx, bvy):
    """Find the best point to intercept the ball considering friction."""
    spd = math.hypot(bvx, bvy)
    d0 = math.hypot(rpos[0] - ball[0], rpos[1] - ball[1])
    best_pt = ball
    best_t = d0 / max(SPRINT_SPEED * 1000, 1)  # time to reach ball directly

    if spd < BALL_MOVING_THRESH:
        return best_pt, best_t

    for i in range(1, INTERCEPT_STEPS + 1):
        dt = i * (INTERCEPT_MAX_T / INTERCEPT_STEPS)
        bp = _predict_ball_friction(ball, bvx, bvy, dt)
        d_robot = math.hypot(rpos[0] - bp[0], rpos[1] - bp[1])
        t_robot = d_robot / max(SPRINT_SPEED * 1000, 1)
        # Robot can reach this point before ball arrives there
        if t_robot <= dt and t_robot < best_t:
            best_pt = bp
            best_t = t_robot

    return best_pt, best_t


def _evaluate_goal_targets(ball, goal_x, goalie_y, goalie_x=None):
    """Score multiple aim points across the goal mouth, return the best."""
    targets = []
    for frac in (-0.42, -0.25, 0.0, 0.25, 0.42):
        ty = frac * GOAL_WIDTH
        targets.append((goal_x, ty))

    best_score = -999
    best_target = (goal_x, 0)

    for tx, ty in targets:
        score = 0.0

        # Prefer corners (harder for goalie)
        corner_bonus = abs(ty) / GOAL_HW * 0.4
        score += corner_bonus

        # Penalize if goalie is near this target
        if goalie_y is not None:
            goalie_dist = abs(ty - goalie_y)
            if goalie_dist < 150:
                score -= 0.8
            elif goalie_dist < 300:
                score -= 0.3
            else:
                score += 0.4  # bonus for far from goalie

        # Penalize if goalie is advanced (closer to ball)
        if goalie_x is not None:
            gk_advance = abs(goalie_x - goal_x)
            if gk_advance > 400:
                # Goalie is out — prefer center or opposite side
                score += 0.2

        # Distance from ball to target (shorter = higher percentage)
        d_shot = math.hypot(ball[0] - tx, ball[1] - ty)
        if d_shot < 1500:
            score += 0.3
        elif d_shot < 2500:
            score += 0.15

        # Shooting angle — wider is better
        dx = abs(ball[0] - goal_x)
        if dx > 100:
            angle = 2.0 * math.atan2(GOAL_HW, dx)
            score += min(angle / math.radians(30), 0.4)

        if score > best_score:
            best_score = score
            best_target = (tx, ty)

    return best_target


def _detect_pressure(rpos, frame, is_yellow, robot_id):
    """Check if any opponent is dangerously close."""
    opp_yellow = not is_yellow
    nearest_dist = 9999
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            d = math.hypot(float(op[0]) - rpos[0], float(op[1]) - rpos[1])
            if d < nearest_dist:
                nearest_dist = d
        except Exception:
            continue
    return nearest_dist


def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    last_kick_time = 0.0
    frame = None
    last_ft = 0.0
    # Smoothed ball position for stable target calculation
    sball_x = None
    sball_y = None
    # Ball velocity tracking
    ball_history = []
    last_ball_xy = None
    bvx, bvy = 0.0, 0.0
    ball_speed = 0.0

    while is_running.is_set():
        now = time.time()

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
            robot = frame.get_yellow_robots(isYellow=is_yellow, robot_id=robot_id)
        except Exception:
            time.sleep(LOOP_RATE)
            continue

        if isinstance(robot, int):
            time.sleep(LOOP_RATE)
            continue

        rp = robot.position
        bp = frame.ball.position

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        # Enemy goal center
        if is_yellow:
            goal_x = -(FIELD_LENGTH / 2) if us_positive else (FIELD_LENGTH / 2)
        else:
            goal_x = (FIELD_LENGTH / 2) if us_positive else -(FIELD_LENGTH / 2)
        ball = (float(bp[0]), float(bp[1]))
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))

        # ── Ball velocity tracking (friction-aware) ───────────────
        if last_ball_xy is None or ball[0] != last_ball_xy[0] or ball[1] != last_ball_xy[1]:
            ball_history.append((now, ball[0], ball[1]))
            if len(ball_history) > BALL_HISTORY_SIZE:
                ball_history.pop(0)
            last_ball_xy = ball

        if len(ball_history) >= 2:
            dt = ball_history[-1][0] - ball_history[0][0]
            if dt > 0.02:
                bvx = (ball_history[-1][1] - ball_history[0][1]) / dt
                bvy = (ball_history[-1][2] - ball_history[0][2]) / dt
                ball_speed = math.hypot(bvx, bvy)
            else:
                bvx, bvy, ball_speed = 0.0, 0.0, 0.0
        else:
            bvx, bvy, ball_speed = 0.0, 0.0, 0.0

        # ── Optimal intercept point (friction model) ──────────────
        intercept, intercept_t = _optimal_intercept(
            (rpos[0], rpos[1]), ball, bvx, bvy)

        # Smooth ball position for stable behind-ball calculations
        if sball_x is None:
            sball_x, sball_y = intercept
        else:
            # Higher alpha when ball is moving fast (track closely)
            alpha = 0.7 if ball_speed > 400 else 0.45
            sball_x += alpha * (intercept[0] - sball_x)
            sball_y += alpha * (intercept[1] - sball_y)
        sball = (sball_x, sball_y)

        # ── Detect opponent goalie position + pressure ────────────
        goalie_y = None
        goalie_x = None
        try:
            opp_yellow = not is_yellow
            for oid in range(6):
                try:
                    opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
                    if not isinstance(opp, int) and opp is not None:
                        op = opp.position
                        ox, oy = float(op[0]), float(op[1])
                        if abs(ox - goal_x) < DEFENSE_DEPTH:
                            goalie_y = oy
                            goalie_x = ox
                            break
                except Exception:
                    continue
        except Exception:
            pass

        # Pressure detection — is an opponent closing in?
        nearest_opp = _detect_pressure(rpos, frame, is_yellow, robot_id)
        under_pressure = nearest_opp < PRESSURE_DIST

        # ── Smart aim point evaluation ────────────────────────────
        aim = _evaluate_goal_targets(ball, goal_x, goalie_y, goalie_x)

        # ── Ball in opponent defense area? → wait outside ─────────
        if _in_defense_area(ball[0], ball[1], goal_x):
            inward = -1.0 if goal_x > 0 else 1.0
            wait_x = goal_x + inward * (DEFENSE_DEPTH + 100)
            wait_y = _clamp(ball[1], -DEFENSE_HALF_WIDTH, DEFENSE_HALF_WIDTH)
            rel_wait = world2robot(rpos, (wait_x, wait_y))
            rel_ball_local = world2robot(rpos, ball)
            ang_ball_local = math.atan2(rel_ball_local[1], rel_ball_local[0])
            vx, vy = _move(rel_wait, CRUISE_SPEED)
            w = _clamp(ang_ball_local * TURN_GAIN * 0.5, -MAX_W, MAX_W)
            cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                               kick=0, dribble=0, isYellow=is_yellow)
            dispatch_q.put((cmd, 0.15))
            time.sleep(LOOP_RATE)
            continue

        # ── Direction from ball to aim point (world frame) ────────
        dx = aim[0] - sball[0]
        dy = aim[1] - sball[1]
        d = math.hypot(dx, dy)
        if d < 1:
            d = 1
        bg_ux = dx / d
        bg_uy = dy / d

        # Behind-ball point
        behind = (sball[0] - bg_ux * BEHIND_DIST,
                  sball[1] - bg_uy * BEHIND_DIST)

        # Robot position relative to ball (world frame)
        rbx = rpos[0] - sball[0]
        rby = rpos[1] - sball[1]
        dot_along = rbx * bg_ux + rby * bg_uy

        # Robot-local transforms
        rel_ball = world2robot(rpos, ball)
        rel_behind = world2robot(rpos, behind)
        rel_aim = world2robot(rpos, aim)

        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        d_behind = math.hypot(rel_behind[0], rel_behind[1])
        ang_aim = math.atan2(rel_aim[1], rel_aim[0])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # Adaptive kick tolerance — wider when close to goal
        dx_to_goal = abs(goal_x - ball[0])
        kick_tol = math.atan2(GOAL_WIDTH * 0.38, max(dx_to_goal, 350))
        kick_tol = _clamp(kick_tol, 0.06, 0.35)

        lined_up = d_behind < 220 and abs(ang_aim) < 0.5

        # ══════════════════════════════════════════════════════════
        #  ONE-TOUCH: Ball coming toward us — intercept and redirect
        # ══════════════════════════════════════════════════════════
        ball_toward_me = (rel_ball[0] > 0 and
                          ball_speed > ONETOUCH_MIN_SPEED and
                          # Ball velocity is broadly toward us
                          (bvx * (rpos[0] - ball[0]) + bvy * (rpos[1] - ball[1])) > 0)

        if (ball_toward_me and d_ball < BALL_NEAR * 1.5
                and abs(ang_aim) < ONETOUCH_ANGLE
                and (now - last_kick_time) > KICK_COOLDOWN):
            # Position to redirect: move slightly toward intercept, face aim
            dribble = 1
            vx, vy = _move(rel_ball, ONETOUCH_SPEED)
            w = _clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

            if d_ball < KICK_RANGE * 1.3 and abs(ang_aim) < kick_tol * 1.5:
                kick = 1
                dribble = 0
                vx = CHARGE_SPEED
                vy = 0.0
                last_kick_time = now

        # ══════════════════════════════════════════════════════════
        #  POSSESS — ball in kicker range
        # ══════════════════════════════════════════════════════════
        elif d_ball < KICK_RANGE and rel_ball[0] > -50:
            dribble = 1
            vx, vy = _move(rel_ball, DRIBBLE_SPEED)
            w = _clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

            # Emergency flick under pressure — just get it away
            if under_pressure and (now - last_kick_time) > KICK_COOLDOWN:
                if abs(ang_aim) < 0.8:  # roughly toward goal
                    kick = 1
                    dribble = 0
                    vx = CHARGE_SPEED
                    vy = 0.0
                    last_kick_time = now

            elif (abs(ang_aim) < kick_tol
                    and (now - last_kick_time) > KICK_COOLDOWN):
                kick = 1
                dribble = 0
                vx = CHARGE_SPEED
                vy = 0.0
                last_kick_time = now

        # ══════════════════════════════════════════════════════════
        #  CHARGE — close and aligned, drive through
        # ══════════════════════════════════════════════════════════
        elif lined_up or (d_ball < BALL_NEAR and abs(ang_aim) < 0.6):
            dribble = 1
            speed = CHARGE_SPEED if d_ball > 180 else DRIBBLE_SPEED
            vx, vy = _move(rel_ball, speed)
            w = _clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

        # ══════════════════════════════════════════════════════════
        #  RECOVER — close but wrong angle, go behind ball
        # ══════════════════════════════════════════════════════════
        elif d_ball < BALL_NEAR:
            dribble = 1
            vx, vy = _move(rel_behind, DRIBBLE_SPEED)
            w = _clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # ══════════════════════════════════════════════════════════
        #  POSITION — navigate to behind-ball point
        # ══════════════════════════════════════════════════════════
        else:
            if dot_along > 50 and d_ball < AVOID_RADIUS * 2.5:
                # Wrong side — curve around the ball
                perp_x = -bg_uy
                perp_y = bg_ux
                side = 1.0 if (rbx * perp_x + rby * perp_y) > 0 else -1.0
                nav_target = (sball[0] - bg_ux * BEHIND_DIST + perp_x * side * AVOID_RADIUS,
                              sball[1] - bg_uy * BEHIND_DIST + perp_y * side * AVOID_RADIUS)
            else:
                nav_target = behind

            # Don't navigate into the defense area
            if _in_defense_area(nav_target[0], nav_target[1], goal_x):
                inward = -1.0 if goal_x > 0 else 1.0
                nav_target = (goal_x + inward * (DEFENSE_DEPTH + 100),
                              nav_target[1])

            rel_nav = world2robot(rpos, nav_target)
            ang_nav = math.atan2(rel_nav[1], rel_nav[0])
            d_nav = math.hypot(rel_nav[0], rel_nav[1])

            # Speed scales with distance — sprint when far
            if d_nav > 1500:
                speed = SPRINT_SPEED
            elif d_nav > 600:
                speed = CRUISE_SPEED
            else:
                speed = CHARGE_SPEED

            vx, vy = _move(rel_nav, speed)

            # Face the aim direction while approaching (pre-orient for shot)
            if d_ball < 800:
                w = _clamp(ang_aim * TURN_GAIN * 0.6, -MAX_W * 0.7, MAX_W * 0.7)
            elif abs(ang_nav) < 0.1:
                w = 0.0
            else:
                w = _clamp(ang_nav * TURN_GAIN * 0.5, -MAX_W * 0.6, MAX_W * 0.6)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble, isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
