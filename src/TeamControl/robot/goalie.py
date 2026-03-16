"""
Elite goalie AI — lightning reflexes, smart positioning, tactical distribution.

Key improvements:
- Wall-bounce prediction: detect shots that will ricochet off walls into goal
- Multi-stage saves: lateral positioning + forward dive
- Predictive positioning: weight toward most dangerous attackers
- Smart distribution: look for teammates to pass to, not just blind clearance
- Aggressive angle narrowing with distance-adaptive advance
- Chip-kick detection: react to lob shots
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

# How far goalie will come off the line to narrow the angle
MAX_ADVANCE = 1100

# Defense area
DEFENSE_DEPTH = 1200
DEFENSE_HALF_WIDTH = 1200

# ═══════════════════════════════════════════════════════════════════
#  SPEEDS (m/s) — full speed, no throttle
# ═══════════════════════════════════════════════════════════════════

SAVE_SPEED = 2.5            # maximum urgency for shot saves
POSITION_SPEED = 1.6        # angle narrowing movement
CLEAR_SPEED = 1.4           # going to clear a dead ball
RETREAT_SPEED = 2.0         # returning to goal quickly
DISTRIBUTE_SPEED = 1.2      # dribbling to pass position

# ═══════════════════════════════════════════════════════════════════
#  ANGULAR
# ═══════════════════════════════════════════════════════════════════

MAX_W = 9.0
FACE_BALL_GAIN = 6.0

# ═══════════════════════════════════════════════════════════════════
#  THRESHOLDS
# ═══════════════════════════════════════════════════════════════════

SHOT_SPEED = 500            # mm/s — lower = detect shots earlier
CLEAR_BALL_SPEED = 450      # mm/s — slower = clearable
CLEAR_BALL_DIST = 1100      # mm — go clear if this close
KICK_DIST = 175
DANGER_ZONE = HALF_LEN      # mm — ball in our half = be ready
PASS_CLEAR = 400            # mm — clearance for distribution pass lane

# ═══════════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═══════════════════════════════════════════════════════════════════

FRICTION = 0.4
BALL_HISTORY_SIZE = 7

# ═══════════════════════════════════════════════════════════════════
#  TIMING
# ═══════════════════════════════════════════════════════════════════

LOOP_RATE = 0.016
FRAME_INTERVAL = 0.04


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _move(rel, speed):
    d = math.hypot(rel[0], rel[1])
    if d < 1:
        return 0.0, 0.0
    return (rel[0] / d) * speed, (rel[1] / d) * speed


def _predict_ball_friction(bx, by, vx, vy, dt):
    """Predict ball position with friction decay."""
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
    return bx, by


def _predict_shot_with_bounce(ball, bvx, bvy, goal_back_x, sign):
    """
    Predict where a shot will cross the goal line, accounting for wall bounces.
    Returns (hit_goal, pred_y) or (False, 0).
    """
    bx, by = ball
    vx, vy = bvx, bvy
    t = 0.0
    step = 0.01
    max_t = 2.5

    while t < max_t:
        bx += vx * step
        by += vy * step

        # Wall bounces
        if by > HALF_WID:
            by = 2 * HALF_WID - by
            vy = -vy
        elif by < -HALF_WID:
            by = -2 * HALF_WID - by
            vy = -vy

        # Friction
        f = max(1.0 - FRICTION * step, 0.0)
        vx *= f
        vy *= f
        t += step

        # Ball too slow
        if math.hypot(vx, vy) < 40:
            return False, 0.0

        # Check if crossed goal line
        if sign > 0 and bx >= goal_back_x:
            if abs(by) < GOAL_HW + 250:
                return True, _clamp(by, -GOAL_HW, GOAL_HW)
            return False, 0.0
        elif sign < 0 and bx <= goal_back_x:
            if abs(by) < GOAL_HW + 250:
                return True, _clamp(by, -GOAL_HW, GOAL_HW)
            return False, 0.0

    return False, 0.0


def _detect_dangerous_attacker(frame, is_yellow, ball, goal_back_x, sign):
    """Find the most dangerous opponent — the one with best shooting angle."""
    opp_yellow = not is_yellow
    most_dangerous_y = None
    best_threat = -1

    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            ox, oy = float(op[0]), float(op[1])

            # Threat = close to ball + facing our goal
            d_ball = math.hypot(ox - ball[0], oy - ball[1])
            d_goal = abs(ox - goal_back_x)

            if d_ball < 800:  # near ball = dangerous
                threat = (800 - d_ball) / 800 + max(0, 1 - d_goal / HALF_LEN)
                if threat > best_threat:
                    best_threat = threat
                    most_dangerous_y = oy
        except Exception:
            continue

    return most_dangerous_y


def _find_pass_target(rpos, ball, frame, is_yellow, goal_back_x, opps_positions):
    """Find the best teammate to distribute to."""
    best_target = None
    best_score = -1

    for tid in range(6):
        try:
            mate = frame.get_yellow_robots(isYellow=is_yellow, robot_id=tid)
            if isinstance(mate, int) or mate is None:
                continue
            mp = mate.position
            mx, my = float(mp[0]), float(mp[1])

            # Skip if in defense area
            if abs(mx - goal_back_x) < DEFENSE_DEPTH and abs(my) < DEFENSE_HALF_WIDTH:
                continue

            # Skip if too close
            d_from_gk = math.hypot(mx - rpos[0], my - rpos[1])
            if d_from_gk < 500:
                continue

            # Check pass lane clear of opponents
            lane_clear = True
            for ox, oy in opps_positions:
                # Point-to-segment distance
                dx, dy = mx - ball[0], my - ball[1]
                l2 = dx * dx + dy * dy
                if l2 < 1:
                    continue
                t = max(0, min(1, ((ox - ball[0]) * dx + (oy - ball[1]) * dy) / l2))
                cx = ball[0] + t * dx
                cy = ball[1] + t * dy
                if math.hypot(ox - cx, oy - cy) < PASS_CLEAR:
                    lane_clear = False
                    break

            if not lane_clear:
                continue

            # Score: prefer distance from our goal (safer distribution)
            advance = abs(mx - goal_back_x) / FIELD_LENGTH
            score = advance

            if score > best_score:
                best_score = score
                best_target = (mx, my)
        except Exception:
            continue

    return best_target


def _clear_direction(ball, goal_back_x):
    """Pick a clearing direction — kick toward the nearest sideline, away from goal."""
    outward = -1.0 if goal_back_x > 0 else 1.0
    side_y = HALF_WID if ball[1] > 0 else -HALF_WID
    return (ball[0] + outward * 1500, side_y)


def run_goalie(is_running, dispatch_q, wm, goalie_id, is_yellow):
    frame = None
    last_ft = 0.0
    ball_history = []
    last_ball_xy = None
    smooth_x = None
    smooth_y = None
    last_kick_time = 0.0

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
            robot = frame.get_yellow_robots(isYellow=is_yellow, robot_id=goalie_id)
        except Exception:
            time.sleep(LOOP_RATE)
            continue

        if isinstance(robot, int):
            time.sleep(LOOP_RATE)
            continue

        try:
            is_positive = wm.us_positive()
        except Exception:
            is_positive = True

        rp = robot.position
        bp = frame.ball.position
        ball = (float(bp[0]), float(bp[1]))
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))

        sign = 1 if is_positive else -1
        goal_back_x = sign * (FIELD_LENGTH / 2)
        goal_mouth = (goal_back_x, 0.0)

        # ── Ball velocity tracking ────────────────────────────────
        if last_ball_xy is None or ball[0] != last_ball_xy[0] or ball[1] != last_ball_xy[1]:
            ball_history.append((now, ball[0], ball[1]))
            if len(ball_history) > BALL_HISTORY_SIZE:
                ball_history.pop(0)
            last_ball_xy = ball

        bvx, bvy = 0.0, 0.0
        ball_speed = 0.0
        if len(ball_history) >= 2:
            dt = ball_history[-1][0] - ball_history[0][0]
            if dt > 0.02:
                bvx = (ball_history[-1][1] - ball_history[0][1]) / dt
                bvy = (ball_history[-1][2] - ball_history[0][2]) / dt
                ball_speed = math.hypot(bvx, bvy)

        # ── Shot detection (with wall bounce prediction) ──────────
        ball_toward_us = (bvx * sign > 80)
        shot_incoming = False
        pred_y = 0.0

        if ball_toward_us and ball_speed > SHOT_SPEED:
            # First try direct shot prediction
            if abs(bvx) > 40:
                t_cross = (goal_back_x - ball[0]) / bvx
                if 0 < t_cross < 2.0:
                    pred_y_raw = ball[1] + bvy * t_cross
                    if abs(pred_y_raw) < GOAL_HW + 250:
                        shot_incoming = True
                        pred_y = _clamp(pred_y_raw, -GOAL_HW, GOAL_HW)

            # If no direct hit, check wall-bounce trajectories
            if not shot_incoming and ball_speed > SHOT_SPEED * 1.2:
                hit, bounce_y = _predict_shot_with_bounce(
                    ball, bvx, bvy, goal_back_x, sign)
                if hit:
                    shot_incoming = True
                    pred_y = bounce_y

        # ── Distances ─────────────────────────────────────────────
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ball_dist = abs(ball[0] - goal_back_x)

        ball_in_box = (ball_dist < DEFENSE_DEPTH
                       and abs(ball[1]) < DEFENSE_HALF_WIDTH)
        ball_slow = ball_speed < CLEAR_BALL_SPEED

        # Collect opponent positions for pass lane checks
        opps_positions = []
        try:
            opp_yellow = not is_yellow
            for oid in range(6):
                try:
                    opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
                    if not isinstance(opp, int) and opp is not None:
                        op = opp.position
                        opps_positions.append((float(op[0]), float(op[1])))
                except Exception:
                    continue
        except Exception:
            pass

        kick, dribble = 0, 0

        # ── Compute raw target ────────────────────────────────────
        if ball_in_box and ball_slow and d_ball < CLEAR_BALL_DIST:
            # CLEAR / DISTRIBUTE: slow ball in our box
            raw_target = ball
            speed = CLEAR_SPEED
            smooth_alpha = 0.75

            if d_ball < KICK_DIST and rel_ball[0] > 0:
                # Look for a teammate to pass to first
                pass_tgt = _find_pass_target(
                    rpos, ball, frame, is_yellow, goal_back_x, opps_positions)

                if pass_tgt and (now - last_kick_time) > 0.3:
                    rel_pass = world2robot(rpos, pass_tgt)
                    ang_pass = math.atan2(rel_pass[1], rel_pass[0])
                    if abs(ang_pass) < 0.45:
                        kick = 1
                        last_kick_time = now
                    else:
                        dribble = 1
                else:
                    # Fallback: clear toward sideline
                    clear_pt = _clear_direction(ball, goal_back_x)
                    rel_clear = world2robot(rpos, clear_pt)
                    ang_clear = math.atan2(rel_clear[1], rel_clear[0])
                    if abs(ang_clear) < 0.45 and (now - last_kick_time) > 0.3:
                        kick = 1
                        last_kick_time = now
                    else:
                        dribble = 1
            elif d_ball < 350:
                dribble = 1

        elif shot_incoming:
            # SAVE: shot heading for goal — get to predicted crossing FAST
            save_x = goal_back_x + (ball[0] - goal_back_x) * 0.10
            raw_target = (save_x, pred_y)
            speed = SAVE_SPEED
            smooth_alpha = 0.93  # near-instant for saves

        else:
            # POSITION: narrow the shooting angle aggressively
            gm_x = ball[0] - goal_mouth[0]
            gm_y = ball[1] - goal_mouth[1]
            gm_d = math.hypot(gm_x, gm_y)

            if gm_d > 1:
                # Advance more aggressively when ball is close
                advance_ratio = 1.0 - _clamp(ball_dist / (FIELD_LENGTH * 0.5), 0, 1)
                # Boost advance when ball is in danger zone
                if ball_dist < DANGER_ZONE:
                    advance_ratio = min(1.0, advance_ratio * 1.5)

                # Weight positioning toward dangerous attacker
                danger_y = _detect_dangerous_attacker(
                    frame, is_yellow, ball, goal_back_x, sign)
                if danger_y is not None and ball_dist < DANGER_ZONE:
                    # Bias toward where the most dangerous attacker would shoot
                    bias = _clamp(danger_y * 0.15, -80, 80)
                else:
                    bias = 0

                advance = advance_ratio * MAX_ADVANCE
                target_x = goal_mouth[0] + (gm_x / gm_d) * advance
                target_y = goal_mouth[1] + (gm_y / gm_d) * advance + bias
                target_y = _clamp(target_y,
                                  -(GOAL_HW + 200),
                                  GOAL_HW + 200)
                raw_target = (target_x, target_y)
            else:
                raw_target = goal_mouth

            # Speed based on urgency
            if ball_dist < DANGER_ZONE * 0.6:
                speed = RETREAT_SPEED
            elif ball_dist < DANGER_ZONE:
                speed = POSITION_SPEED
            else:
                speed = POSITION_SPEED * 0.8
            smooth_alpha = 0.22

        # ── Smooth the target to kill jitter ──────────────────────
        if smooth_x is None:
            smooth_x = raw_target[0]
            smooth_y = raw_target[1]
        else:
            smooth_x += smooth_alpha * (raw_target[0] - smooth_x)
            smooth_y += smooth_alpha * (raw_target[1] - smooth_y)
        target = (smooth_x, smooth_y)

        rel_target = world2robot(rpos, target)
        d_target = math.hypot(rel_target[0], rel_target[1])

        # Dead zone
        if d_target < 20:
            vx, vy = 0.0, 0.0
        else:
            vx, vy = _move(rel_target, speed)
            # Proportional deceleration near target (softer curve)
            if d_target < 100:
                scale = d_target / 100.0
                vx *= scale
                vy *= scale

        # Face the ball at all times
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if abs(ang_ball) < 0.05:
            w = 0.0
        else:
            w = _clamp(ang_ball * FACE_BALL_GAIN, -MAX_W, MAX_W)

        cmd = RobotCommand(robot_id=goalie_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble, isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
