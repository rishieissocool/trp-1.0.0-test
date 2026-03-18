"""
Goalie AI — smooth movement, fast saves, smart clearing.

Key improvements over previous version:
- _move() with deceleration ramp: no more overshoot at target positions
- Smoother save transitions: near-instant response to shots
- Better clearing: dribble toward clear angle before kicking
- Smart distribution: pass to open teammates when possible
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.constants import (
    FIELD_LENGTH, FIELD_WIDTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, MAX_ADVANCE,
    PENALTY_DEPTH, PENALTY_HW,
    DEFENSE_DEPTH, DEFENSE_HALF_WIDTH,
    SAVE_SPEED, POSITION_SPEED, CLEAR_SPEED, RETREAT_SPEED, DISTRIBUTE_SPEED,
    MAX_W, FACE_BALL_GAIN,
    SHOT_SPEED, CLEAR_BALL_SPEED, CLEAR_BALL_DIST, KICK_DIST,
    DANGER_ZONE, PASS_CLEAR,
    FRICTION, BALL_HISTORY_SIZE,
    LOOP_RATE, FRAME_INTERVAL,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _move(rel, speed, ramp_dist=250.0, stop_dist=20.0, min_speed=0.08):
    """Move toward a robot-local point with smooth deceleration near target."""
    d = math.hypot(rel[0], rel[1])
    if d < stop_dist:
        return 0.0, 0.0
    if d < ramp_dist:
        t = (d - stop_dist) / max(ramp_dist - stop_dist, 1.0)
        speed = max(speed * t, min_speed)
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

        if by > HALF_WID:
            by = 2 * HALF_WID - by
            vy = -vy
        elif by < -HALF_WID:
            by = -2 * HALF_WID - by
            vy = -vy

        f = max(1.0 - FRICTION * step, 0.0)
        vx *= f
        vy *= f
        t += step

        if math.hypot(vx, vy) < 40:
            return False, 0.0

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
    """Find the most dangerous opponent — the one closest to ball near our goal."""
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

            d_ball = math.hypot(ox - ball[0], oy - ball[1])
            d_goal = abs(ox - goal_back_x)

            if d_ball < 800:
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

            if abs(mx - goal_back_x) < DEFENSE_DEPTH and abs(my) < DEFENSE_HALF_WIDTH:
                continue

            d_from_gk = math.hypot(mx - rpos[0], my - rpos[1])
            if d_from_gk < 500:
                continue

            lane_clear = True
            for ox, oy in opps_positions:
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


def _clamp_to_penalty_box(x, y, goal_x):
    """Hard-clamp a position to stay inside the penalty box."""
    margin = 50
    if goal_x > 0:
        x = _clamp(x, goal_x - PENALTY_DEPTH + margin, goal_x - margin)
    else:
        x = _clamp(x, goal_x + margin, goal_x + PENALTY_DEPTH - margin)
    y = _clamp(y, -PENALTY_HW + margin, PENALTY_HW - margin)
    return x, y


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
            if abs(bvx) > 40:
                t_cross = (goal_back_x - ball[0]) / bvx
                if 0 < t_cross < 2.0:
                    pred_y_raw = ball[1] + bvy * t_cross
                    if abs(pred_y_raw) < GOAL_HW + 250:
                        shot_incoming = True
                        pred_y = _clamp(pred_y_raw, -GOAL_HW, GOAL_HW)

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

        ball_in_box = (ball_dist < PENALTY_DEPTH
                       and abs(ball[1]) < PENALTY_HW)
        ball_slow = ball_speed < CLEAR_BALL_SPEED

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
            # CLEAR / DISTRIBUTE
            raw_target = ball
            speed = CLEAR_SPEED
            smooth_alpha = 0.75

            if d_ball < KICK_DIST and rel_ball[0] > 0:
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
            # SAVE — near-instant response
            save_x = goal_back_x + (ball[0] - goal_back_x) * 0.10
            raw_target = (save_x, pred_y)
            speed = SAVE_SPEED
            smooth_alpha = 0.95

        else:
            # POSITION — narrow the shooting angle
            gm_x = ball[0] - goal_mouth[0]
            gm_y = ball[1] - goal_mouth[1]
            gm_d = math.hypot(gm_x, gm_y)

            if gm_d > 1:
                advance_ratio = 1.0 - _clamp(ball_dist / (FIELD_LENGTH * 0.5), 0, 1)
                if ball_dist < DANGER_ZONE:
                    advance_ratio = min(1.0, advance_ratio * 1.5)

                danger_y = _detect_dangerous_attacker(
                    frame, is_yellow, ball, goal_back_x, sign)
                if danger_y is not None and ball_dist < DANGER_ZONE:
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

        smooth_x, smooth_y = _clamp_to_penalty_box(
            smooth_x, smooth_y, goal_back_x)
        target = (smooth_x, smooth_y)

        rel_target = world2robot(rpos, target)
        d_target = math.hypot(rel_target[0], rel_target[1])

        if d_target < 15:
            vx, vy = 0.0, 0.0
        else:
            vx, vy = _move(rel_target, speed, ramp_dist=220, stop_dist=15)

        # Face the ball at all times
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if abs(ang_ball) < 0.04:
            w = 0.0
        else:
            w = _clamp(ang_ball * FACE_BALL_GAIN, -MAX_W, MAX_W)

        cmd = RobotCommand(robot_id=goalie_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble, isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
