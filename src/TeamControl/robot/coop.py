"""
Cooperative 2-bot striker — two robots work together to score.

Role assignment each tick (with hysteresis):
  - Closer to ball  → ATTACKER: approach ball, shoot or pass to support
  - Farther from ball → SUPPORT: find open receiving spot, park there facing
    ball, await pass, then one-touch / dribble-and-shoot into goal.

Both robots avoid every other robot on the field.
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
    compute_arc_nav,
)
from TeamControl.robot.navigator import _compute_avoidance, _smooth_strength
from TeamControl.robot.constants import (
    FIELD_LENGTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, GOAL_DEPTH,
    PENALTY_DEPTH, PENALTY_HW,
    BEHIND_DIST, KICK_RANGE, BALL_NEAR, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED, SPRINT_SPEED,
    MAX_W, TURN_GAIN, FACE_BALL_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
)

# ── Tuning ───────────────────────────────────────────────────────
SHOOT_RANGE     = 350       # mm — shoot from this far if aimed
SHOOT_ALIGN     = 0.18      # rad — alignment for distance shot
KICK_ALIGN      = 0.18      # rad — alignment for close kick
PASS_ALIGN      = 0.25      # rad — alignment tolerance for passing
PASS_MIN_DIST   = 400       # mm — teammate must be at least this far to pass
FORCE_KICK_TIME = 0.8       # s  — force kick after this long near ball
POSSESSION_DIST = 350       # mm — attacker "has" the ball if this close
ROLE_HOLD_MIN   = 0.6       # s  — minimum time before role can switch
ROLE_HYST_DIST  = 200       # mm — distance advantage needed to steal attacker role
APPROACH_SPD    = CRUISE_SPEED
DRIBBLE_SPD     = DRIBBLE_SPEED

# ── Support / receiver tuning ────────────────────────────────────
RECV_OFFSET_FWD   = 1200    # mm — how far ahead of ball (toward goal) to position
RECV_OFFSET_LAT   = 800     # mm — lateral offset from ball-goal line
RECV_STOP_DIST    = 80      # mm — stop moving when this close to target spot
RECV_OPP_CLEAR    = 500     # mm — minimum clearance from any opponent
RECV_MATE_CLEAR   = 600     # mm — minimum clearance from attacker/ball
RECV_BALL_INCOMING = 500    # mm/s — ball speed threshold to consider "pass incoming"
RECV_CLOSE        = 450     # mm — ball close enough to transition to finish
RECV_FINISH_ALIGN = 0.30    # rad — alignment tolerance for one-touch finish
RECV_FORCE_KICK   = 0.6     # s  — force kick if ball lingers near support
RECV_FIELD_MARGIN = 300     # mm — stay this far from walls

# Ball velocity estimation
BALL_HIST_SIZE = 6


def _get_robot(frame, is_yellow, rid):
    """Get a robot's (x, y, theta) or None."""
    try:
        r = frame.get_yellow_robots(isYellow=is_yellow, robot_id=rid)
        if isinstance(r, int) or r is None:
            return None
        p = r.position
        return (float(p[0]), float(p[1]), float(p[2]))
    except Exception:
        return None


def _get_opponents(frame, is_yellow):
    """Return list of (x, y) for all opponent robots."""
    opps = []
    opp_color = not is_yellow
    for oid in range(16):
        try:
            r = frame.get_yellow_robots(isYellow=opp_color, robot_id=oid)
            if isinstance(r, int) or r is None:
                continue
            p = r.position
            opps.append((float(p[0]), float(p[1])))
        except Exception:
            continue
    return opps


def _in_penalty_box(px, py, goal_x):
    return abs(px - goal_x) < PENALTY_DEPTH and abs(py) < PENALTY_HW


def _field_clamp(x, y):
    return (clamp(x, -HALF_LEN + RECV_FIELD_MARGIN, HALF_LEN - RECV_FIELD_MARGIN),
            clamp(y, -HALF_WID + RECV_FIELD_MARGIN, HALF_WID - RECV_FIELD_MARGIN))


# ── Candidate receiving spots ───────────────────────────────────
# Offsets relative to ball, in attacking direction.  (fwd, lat)
_RECV_CANDIDATES = [
    (RECV_OFFSET_FWD,  RECV_OFFSET_LAT),
    (RECV_OFFSET_FWD, -RECV_OFFSET_LAT),
    (RECV_OFFSET_FWD * 0.8,  RECV_OFFSET_LAT * 1.4),
    (RECV_OFFSET_FWD * 0.8, -RECV_OFFSET_LAT * 1.4),
    (RECV_OFFSET_FWD * 1.3,  RECV_OFFSET_LAT * 0.5),
    (RECV_OFFSET_FWD * 1.3, -RECV_OFFSET_LAT * 0.5),
    (RECV_OFFSET_FWD * 0.6,  RECV_OFFSET_LAT * 0.7),
    (RECV_OFFSET_FWD * 0.6, -RECV_OFFSET_LAT * 0.7),
    (RECV_OFFSET_FWD * 1.5,  0),
    (RECV_OFFSET_FWD * 0.5,  RECV_OFFSET_LAT * 1.8),
    (RECV_OFFSET_FWD * 0.5, -RECV_OFFSET_LAT * 1.8),
]


def _pick_receive_spot(ball, goal_x, me, mate, opps):
    """Pick the best open spot for the support robot to receive a pass.

    Scores candidates based on:
      - clearance from opponents (must be > RECV_OPP_CLEAR)
      - clearance from attacker / ball
      - not inside penalty box
      - has a clear angle to goal (shot potential)
      - on-field
    Returns (x, y) in world coordinates.
    """
    atk_dir = 1.0 if goal_x > 0 else -1.0
    best_score = -9999
    best_pt = None

    for fwd, lat in _RECV_CANDIDATES:
        cx = ball[0] + fwd * atk_dir
        cy = ball[1] + lat
        cx, cy = _field_clamp(cx, cy)

        # Skip if inside penalty box
        if _in_penalty_box(cx, cy, goal_x):
            continue

        score = 0.0

        # Clearance from opponents — reject if too close
        min_opp_d = min((math.hypot(cx - ox, cy - oy) for ox, oy in opps),
                        default=9999)
        if min_opp_d < RECV_OPP_CLEAR:
            score -= 8.0
        else:
            score += min(min_opp_d / 1000, 3.0)

        # Clearance from ball / attacker
        d_ball = math.hypot(cx - ball[0], cy - ball[1])
        if d_ball < RECV_MATE_CLEAR:
            score -= 4.0

        if mate is not None:
            d_mate = math.hypot(cx - mate[0], cy - mate[1])
            if d_mate < RECV_MATE_CLEAR:
                score -= 3.0

        # Closer to goal is better (but not too close)
        d_goal = abs(cx - goal_x)
        if d_goal < PENALTY_DEPTH + 200:
            score -= 2.0
        else:
            score += max(0, 3.0 - d_goal / 2000)

        # Lateral: prefer positions with a clear angle to goal
        if abs(cy) < GOAL_HW + 400:
            score += 2.0
        if abs(cy) < GOAL_HW:
            score += 1.0

        # Not behind the ball (in attacking direction)
        advance = (cx - ball[0]) * atk_dir
        if advance < 200:
            score -= 5.0
        else:
            score += min(advance / 1000, 2.0)

        if score > best_score:
            best_score = score
            best_pt = (cx, cy)

    # Fallback: offset from ball toward goal
    if best_pt is None:
        fx = ball[0] + RECV_OFFSET_FWD * atk_dir
        fy = ball[1] + RECV_OFFSET_LAT * (1.0 if ball[1] < 0 else -1.0)
        best_pt = _field_clamp(fx, fy)

    return best_pt


def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    """Cooperative striker process for one robot.

    Both robots run this function (with swapped robot_id / teammate_id).
    Each independently decides its role every tick based on who is closer
    to the ball, with hysteresis to prevent rapid back-and-forth switching.

    If mate_is_yellow is provided, the teammate is looked up on that team
    instead of is_yellow (cross-team coop mode).

    If attack_positive is provided (True/False), both robots attack toward
    +x (True) or -x (False), overriding the per-team goal logic.  This is
    needed when the two robots are on different teams but cooperating.
    """
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    frame = None
    last_ft = 0.0
    last_kick = 0.0
    committed_side = None
    near_ball_since = 0.0
    prev_obs_pos = {}
    prev_role_attacker = None   # None = undecided, True/False = last role
    role_since = 0.0            # when current role was assigned

    # Ball velocity tracking
    ball_history = []
    last_ball_xy = None

    # Support state
    recv_spot = None            # current receiving spot (world coords)
    recv_near_since = 0.0       # when ball first got close to support

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

        if frame is None or frame.ball is None:
            time.sleep(LOOP_RATE)
            continue

        me = _get_robot(frame, is_yellow, robot_id)
        mate = _get_robot(frame, mate_is_yellow, teammate_id)
        if me is None:
            time.sleep(LOOP_RATE)
            continue

        bp = frame.ball.position
        ball = (float(bp[0]), float(bp[1]))

        # ── Ball velocity ────────────────────────────────────
        if (last_ball_xy is None
                or ball[0] != last_ball_xy[0]
                or ball[1] != last_ball_xy[1]):
            ball_history.append((now, ball[0], ball[1]))
            if len(ball_history) > BALL_HIST_SIZE:
                ball_history.pop(0)
            last_ball_xy = ball

        bvx, bvy, ball_speed = 0.0, 0.0, 0.0
        if len(ball_history) >= 2:
            dt = ball_history[-1][0] - ball_history[0][0]
            if dt > 0.02:
                bvx = (ball_history[-1][1] - ball_history[0][1]) / dt
                bvy = (ball_history[-1][2] - ball_history[0][2]) / dt
                ball_speed = math.hypot(bvx, bvy)

        try:
            us_positive = wm.us_positive()
        except Exception:
            us_positive = True

        # Opponent goal — in cross-team coop, both attack the same goal
        if attack_positive is not None:
            goal_x = HALF_LEN if attack_positive else -HALF_LEN
        elif is_yellow:
            goal_x = -HALF_LEN if us_positive else HALF_LEN
        else:
            goal_x = HALF_LEN if us_positive else -HALF_LEN

        # ── Opponents list (for spot evaluation) ─────────────
        opps = _get_opponents(frame, is_yellow)
        # In cross-team coop, also exclude the mate from the opponent list
        if mate_is_yellow != is_yellow and mate is not None:
            mate_2d = (mate[0], mate[1])
            opps = [o for o in opps
                    if math.hypot(o[0] - mate_2d[0], o[1] - mate_2d[1]) > 50]

        # ── Decide role (with hysteresis) ───────────────────
        my_d = math.hypot(me[0] - ball[0], me[1] - ball[1])
        if mate is not None:
            mate_d = math.hypot(mate[0] - ball[0], mate[1] - ball[1])
            raw_attacker = (my_d < mate_d) or (my_d == mate_d and robot_id < teammate_id)

            if prev_role_attacker is None:
                i_am_attacker = raw_attacker
            elif (now - role_since) < ROLE_HOLD_MIN:
                i_am_attacker = prev_role_attacker
            elif raw_attacker != prev_role_attacker:
                advantage = mate_d - my_d if raw_attacker else my_d - mate_d
                if advantage > ROLE_HYST_DIST:
                    i_am_attacker = raw_attacker
                else:
                    i_am_attacker = prev_role_attacker
            else:
                i_am_attacker = prev_role_attacker
        else:
            i_am_attacker = True

        if i_am_attacker != prev_role_attacker:
            role_since = now
            recv_spot = None          # reset receiving spot on role change
            recv_near_since = 0.0
        prev_role_attacker = i_am_attacker

        # ── Possession check ─────────────────────────────────
        attacker_has_ball = False
        if mate is not None:
            if i_am_attacker:
                attacker_has_ball = my_d < POSSESSION_DIST
            else:
                attacker_has_ball = mate_d < POSSESSION_DIST

        # ── Common vectors ───────────────────────────────────
        rel_ball = world2robot(me, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])

        aim = (goal_x, 0.0)
        rel_aim = world2robot(me, aim)
        ang_aim = math.atan2(rel_aim[1], rel_aim[0])

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        if i_am_attacker:
            # ═══════════════════════════════════════════════════
            #  ATTACKER — go for the ball, shoot or pass
            # ═══════════════════════════════════════════════════
            recv_near_since = 0.0

            # Check if teammate is ready to receive a pass
            can_pass = False
            pass_target = None
            if mate is not None and (now - last_kick) > KICK_COOLDOWN:
                mate_to_goal = abs(mate[0] - goal_x)
                me_to_goal = abs(me[0] - goal_x)
                mate_clear = abs(mate[1]) < GOAL_HW + 400
                # Pass if mate is ahead, far enough, and in a scoring zone
                if (mate_to_goal < me_to_goal
                        and mate_d > PASS_MIN_DIST
                        and mate_clear):
                    rel_mate = world2robot(me, (mate[0], mate[1]))
                    ang_mate = math.atan2(rel_mate[1], rel_mate[0])
                    if abs(ang_mate) < PASS_ALIGN and rel_mate[0] > 0:
                        can_pass = True
                        pass_target = (mate[0], mate[1])

            # Penalty box — wait outside
            if _in_penalty_box(ball[0], ball[1], goal_x):
                inward = -1.0 if goal_x > 0 else 1.0
                wait_pt = (goal_x + inward * (PENALTY_DEPTH + 120),
                           clamp(ball[1], -PENALTY_HW, PENALTY_HW))
                rel_w = world2robot(me, wait_pt)
                vx, vy = move_toward(rel_w, CRUISE_SPEED * 0.6)
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
                committed_side = None
                near_ball_since = 0.0

            # Distance shot — ball in front, aimed at goal
            elif (d_ball < SHOOT_RANGE and rel_ball[0] > 0
                  and abs(ang_aim) < SHOOT_ALIGN
                  and (now - last_kick) > KICK_COOLDOWN):
                kick = 1
                vx, vy = move_toward(rel_ball, CHARGE_SPEED,
                                     ramp_dist=100, stop_dist=0)
                w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)
                last_kick = now
                near_ball_since = 0.0
                committed_side = None

            # Pass to teammate
            elif (can_pass and d_ball < KICK_RANGE * 1.5
                  and rel_ball[0] > 0
                  and (now - last_kick) > KICK_COOLDOWN):
                kick = 1
                vx, vy = move_toward(rel_ball, CHARGE_SPEED,
                                     ramp_dist=100, stop_dist=0)
                rel_pt = world2robot(me, pass_target)
                w = clamp(math.atan2(rel_pt[1], rel_pt[0]) * TURN_GAIN,
                          -MAX_W, MAX_W)
                last_kick = now
                near_ball_since = 0.0
                committed_side = None

            # Close kick
            elif d_ball < KICK_RANGE and rel_ball[0] > 0:
                dribble = 1
                vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                     ramp_dist=150, stop_dist=10)
                w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)
                committed_side = None

                if near_ball_since == 0.0:
                    near_ball_since = now
                force = (now - near_ball_since) > FORCE_KICK_TIME

                if (now - last_kick) > KICK_COOLDOWN and (
                    abs(ang_aim) < KICK_ALIGN or force
                ):
                    kick = 1
                    dribble = 0
                    vx = DRIBBLE_SPD
                    vy = 0.0
                    last_kick = now
                    near_ball_since = 0.0

            # Approach — arc behind ball
            else:
                near_ball_since = 0.0
                nav, committed_side, is_behind = compute_arc_nav(
                    robot_xy=(me[0], me[1]),
                    ball=ball, aim=aim,
                    behind_dist=BEHIND_DIST,
                    avoid_radius=AVOID_RADIUS,
                    committed_side=committed_side,
                )
                rel_nav = world2robot(me, nav)
                d_nav = math.hypot(rel_nav[0], rel_nav[1])

                if is_behind and d_nav < 300 and d_ball < BALL_NEAR:
                    dribble = 1
                    vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                         ramp_dist=300, stop_dist=10)
                    w = clamp(ang_aim * TURN_GAIN * 0.7, -MAX_W, MAX_W)
                else:
                    vx, vy = move_toward(rel_nav, APPROACH_SPD,
                                         ramp_dist=400, stop_dist=10)
                    w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        else:
            # ═══════════════════════════════════════════════════
            #  SUPPORT — find open spot, park, await pass, finish
            # ═══════════════════════════════════════════════════
            committed_side = None
            near_ball_since = 0.0

            # Detect if the ball is incoming toward me (pass in flight)
            ball_toward_me = False
            if ball_speed > RECV_BALL_INCOMING and d_ball < 2000:
                # Check if ball velocity vector points roughly at me
                to_me_x = me[0] - ball[0]
                to_me_y = me[1] - ball[1]
                to_me_d = math.hypot(to_me_x, to_me_y)
                if to_me_d > 1:
                    cos_angle = (bvx * to_me_x + bvy * to_me_y) / (ball_speed * to_me_d)
                    if cos_angle > 0.5:   # within ~60 degrees
                        ball_toward_me = True

            # ── STATE: Ball arrived / very close → FINISH ────
            if d_ball < RECV_CLOSE or (ball_toward_me and d_ball < RECV_CLOSE * 1.5):
                # Pick best aim point in goal (away from center for corner)
                aim_y = -GOAL_HW * 0.5 if me[1] > 0 else GOAL_HW * 0.5
                finish_aim = (goal_x, aim_y)
                rel_finish = world2robot(me, finish_aim)
                ang_finish = math.atan2(rel_finish[1], rel_finish[0])

                # Track how long ball has been near us
                if recv_near_since == 0.0:
                    recv_near_since = now
                near_duration = now - recv_near_since
                force_kick = near_duration > RECV_FORCE_KICK

                if d_ball < KICK_RANGE and rel_ball[0] > -40:
                    # Ball in kicker range — try to finish
                    if ((abs(ang_finish) < RECV_FINISH_ALIGN or force_kick)
                            and (now - last_kick) > KICK_COOLDOWN):
                        # One-touch kick into goal
                        kick = 1
                        dribble = 0
                        vx, vy = move_toward(rel_ball, CHARGE_SPEED,
                                             ramp_dist=100, stop_dist=0)
                        w = clamp(ang_finish * TURN_GAIN, -MAX_W, MAX_W)
                        last_kick = now
                        recv_near_since = 0.0
                    else:
                        # Dribble and align to goal
                        dribble = 1
                        vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                             ramp_dist=150, stop_dist=10)
                        w = clamp(ang_finish * TURN_GAIN, -MAX_W, MAX_W)
                else:
                    # Ball close but not in kicker — move toward it
                    vx, vy = move_toward(rel_ball, APPROACH_SPD,
                                         ramp_dist=300, stop_dist=10)
                    w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

            # ── STATE: Ball incoming → INTERCEPT ─────────────
            elif ball_toward_me:
                recv_near_since = 0.0
                # Move toward the ball to intercept it cleanly
                dribble = 1
                vx, vy = move_toward(rel_ball, APPROACH_SPD * 0.8,
                                     ramp_dist=400, stop_dist=20)
                # Pre-orient toward goal while intercepting
                aim_y = -GOAL_HW * 0.5 if me[1] > 0 else GOAL_HW * 0.5
                finish_aim = (goal_x, aim_y)
                rel_finish = world2robot(me, finish_aim)
                ang_finish = math.atan2(rel_finish[1], rel_finish[0])
                # Blend: mostly face ball, but bias toward goal
                blend_ang = ang_ball * 0.6 + ang_finish * 0.4
                w = clamp(blend_ang * TURN_GAIN, -MAX_W, MAX_W)

            # ── STATE: Awaiting pass → POSITION at open spot ─
            else:
                recv_near_since = 0.0

                # Re-evaluate receiving spot periodically or if we don't have one
                mate_2d = (mate[0], mate[1]) if mate is not None else None
                recv_spot = _pick_receive_spot(ball, goal_x, me, mate_2d, opps)

                target = recv_spot
                rel_t = world2robot(me, target)
                d_target = math.hypot(rel_t[0], rel_t[1])

                if d_target < RECV_STOP_DIST:
                    # Parked at spot — face the ball, stay still
                    vx, vy = 0.0, 0.0
                    w = clamp(ang_ball * FACE_BALL_GAIN, -MAX_W, MAX_W)
                else:
                    # Move toward the receiving spot
                    speed = CRUISE_SPEED if d_target > 500 else CRUISE_SPEED * 0.6
                    vx, vy = move_toward(rel_t, speed, ramp_dist=400,
                                         stop_dist=RECV_STOP_DIST)
                    # Face the ball while moving
                    w = clamp(ang_ball * FACE_BALL_GAIN, -MAX_W, MAX_W)

        # ── Obstacle avoidance ────────────────────────────────
        rel_target = world2robot(me, ball)
        avoid_vx, avoid_vy, closest_obs, prev_obs_pos = _compute_avoidance(
            me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
        vx += avoid_vx
        vy += avoid_vy

        # Cap speed
        spd = math.hypot(vx, vy)
        max_spd = SPRINT_SPEED if kick else CRUISE_SPEED * 1.2
        if spd > max_spd:
            vx = vx / spd * max_spd
            vy = vy / spd * max_spd

        # ── Wall braking ──────────────────────────────────────
        vx, vy = wall_brake(me[0], me[1], vx, vy)

        # ── Rotation compensation ─────────────────────────────
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
