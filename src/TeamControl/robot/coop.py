"""
Cooperative passing drill — two robots kick the ball back and forth.

Yellow in left half, blue in right half. Each robot uses arc-nav to get
behind the ball, drives into it with dribbler, aligns toward mate, then
fires a SUSTAINED kick (kick=1 for ~0.3s) to ensure grSim registers
the kick rather than just pushing the ball with body inertia.

Field is 4500 x 2230 mm  (HALF_LEN = 2250, HALF_WID = 1115).
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
    ball_velocity, update_ball_history, predict_ball,
    compute_arc_nav,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.grSimPacketFactory import grSimPacketFactory
from TeamControl.robot.constants import (
    HALF_LEN, HALF_WID,
    GOAL_HW, GOAL_DEPTH,
    KICK_RANGE, BALL_NEAR, BEHIND_DIST, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
)


# =====================================================================
#  LAYOUT
#
#       y
#    +1115 +----------------------------------------------+
#          |                                              |
#        0 |    [YELLOW]  <-- ball -->    [BLUE]          |
#          |   (-1000,0)                (1000,0)          |
#          |                                              |
#    -1115 +----------------------------------------------+
#        -2250              0               +2250  x
# =====================================================================

HOME_YELLOW     = (-1000, 0)
HOME_BLUE       = (1000, 0)
BALL_START      = (-500, 0)

# -- Tuning ------------------------------------------------------------
KICK_ALIGN_TOL  = 0.22      # rad — alignment to fire kick
FORCE_KICK_TIME = 1.0       # s — force kick after this long near ball
CLAIM_DIST      = 1400      # mm — ball within this = I go for it
KICK_BURST_T    = 0.35      # s — sustain kick=1 for this long
CONTACT_DIST    = 130       # mm — ball touching dribbler (90 robot + 21 ball + margin)
DRIBBLE_SPD     = DRIBBLE_SPEED
APPROACH_SPD    = CRUISE_SPEED
SETUP_PAUSE     = 2.0

# Exported for UI overlay
ATK_START       = HOME_YELLOW
SUP_START       = HOME_BLUE
BALL_TRIGGER    = BALL_START
BALL_TRIGGER_R  = CLAIM_DIST
SUP_SHOOT_SPOT  = HOME_BLUE
GOAL_X          = HALF_LEN


# =====================================================================
#  HELPERS
# =====================================================================

def _get_robot(frame, is_yellow, rid):
    try:
        r = frame.get_yellow_robots(isYellow=is_yellow, robot_id=rid)
        if isinstance(r, int) or r is None:
            return None
        p = r.position
        return (float(p[0]), float(p[1]), float(p[2]))
    except Exception:
        return None


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _go_to(me, target, speed, stop_r=40, ramp=400):
    rel = world2robot(me, target)
    return move_toward(rel, speed, ramp_dist=ramp, stop_dist=stop_r)


def _face(me, target):
    rel = world2robot(me, target)
    ang = math.atan2(rel[1], rel[0])
    return clamp(ang * TURN_GAIN, -MAX_W, MAX_W)


def _at(me, target, radius):
    return _dist(me, target) < radius


# =====================================================================
#  MAIN — one process per robot
#
#  Modes:
#    setup   — teleport, place ball
#    home    — predict incoming ball, intercept passes
#    active  — arc behind ball, dribble, align to mate, kick
#    kicking — sustained kick burst
#    retreat — go back to home
# =====================================================================

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = 0.0 if is_yellow else math.pi

    sim = None
    try:
        sim = grSimSender("127.0.0.1", 20011)
    except Exception as e:
        print(f"[coop] grSim sender failed: {e}")

    # -- Teleport -------------------------------------------------------
    if sim:
        try:
            pkt = grSimPacketFactory.robot_replacement_command(
                x=home[0] / 1000.0, y=home[1] / 1000.0,
                orientation=home_ang,
                robot_id=robot_id, isYellow=is_yellow)
            sim.send_packet(pkt)
            time.sleep(0.1)
        except Exception as e:
            print(f"[coop] teleport failed: {e}")

    tag = "yellow" if is_yellow else "blue"
    print(f"[coop {tag}] Passing drill — home={home}")

    # -- State ----------------------------------------------------------
    mode = "setup"
    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False
    pass_count = 0

    ball_history = []
    last_ball_xy = None

    committed_side = None
    near_ball_since = 0.0
    kick_end_time = 0.0

    while is_running.is_set():
        now = time.time()

        # -- Fetch frame ------------------------------------------------
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

        me = _get_robot(frame, is_yellow, robot_id)
        if me is None:
            time.sleep(LOOP_RATE)
            continue

        ball = None
        if frame.ball is not None:
            bp = frame.ball.position
            ball = (float(bp[0]), float(bp[1]))

        mate = _get_robot(frame, mate_is_yellow, teammate_id)
        mate_pos = (mate[0], mate[1]) if mate is not None else \
                   (HOME_BLUE if is_yellow else HOME_YELLOW)

        # -- Ball velocity ----------------------------------------------
        if ball is not None:
            last_ball_xy = update_ball_history(
                ball_history, now, ball, last_ball_xy)
        bvx, bvy, bspeed = ball_velocity(ball_history)

        # -- Distances --------------------------------------------------
        my_dist = _dist(ball, me[:2]) if ball else float('inf')
        mate_dist = _dist(ball, mate_pos) if ball else float('inf')

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ==============================================================
        #  SETUP
        # ==============================================================
        if mode == "setup":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, mate_pos)
            if _at(me, home, 200) and (now - setup_time) > SETUP_PAUSE:
                if is_yellow and not ball_placed and sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_START[0] / 1000.0,
                            y=BALL_START[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                        print(f"[coop yellow] ball placed at {BALL_START}")
                    except Exception:
                        pass
                    ball_placed = True
                mode = "home"
                print(f"[coop {tag}] ready")

        # ==============================================================
        #  HOME — predict incoming ball, move to intercept
        # ==============================================================
        elif mode == "home":
            if ball is not None:
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me

                if bspeed > 150 and approach_spd > 100:
                    # Ball coming toward me — predict intercept
                    t_arrive = max(dd_to_me / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_arrive, 2.0))
                    dribble = 1
                    vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                else:
                    vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                    w = _face(me, ball)

                if my_dist < 600:
                    dribble = 1

                # Ball is mine -> go active
                if my_dist < CLAIM_DIST and my_dist < mate_dist - 100:
                    mode = "active"
                    committed_side = None
                    near_ball_since = 0.0
                    print(f"[coop {tag}] ball is mine — going for it")
            else:
                vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                w = _face(me, mate_pos)

        # ==============================================================
        #  ACTIVE — get ball, align to mate, trigger kick burst
        #
        #  Priority:
        #    1. Ball touching + aligned to mate -> kick burst
        #    2. Ball close + in front -> dribble, align
        #    3. Ball moving toward me -> intercept
        #    4. Arc behind ball toward mate, drive in
        # ==============================================================
        elif mode == "active":
            if ball is None:
                mode = "home"
                time.sleep(LOOP_RATE)
                continue

            # Ball went to mate — retreat
            if my_dist > mate_dist + 200 and my_dist > CLAIM_DIST * 0.7:
                mode = "retreat"
                committed_side = None
                near_ball_since = 0.0
                print(f"[coop {tag}] ball went to mate — retreating")
                time.sleep(LOOP_RATE)
                continue

            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])
            ang_ball = math.atan2(rel_ball[1], rel_ball[0])

            rel_mate = world2robot(me, mate_pos)
            ang_mate = math.atan2(rel_mate[1], rel_mate[0])

            # -- P1: Ball touching + aligned -> KICK BURST -------------
            if d_ball < CONTACT_DIST and rel_ball[0] > 0:
                dribble = 1
                vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                     ramp_dist=100, stop_dist=0)
                w = clamp(ang_mate * TURN_GAIN, -MAX_W, MAX_W)

                if near_ball_since == 0.0:
                    near_ball_since = now
                force_kick = (now - near_ball_since) > FORCE_KICK_TIME

                can_kick = (now - last_kick) > KICK_COOLDOWN
                if can_kick and (abs(ang_mate) < KICK_ALIGN_TOL or force_kick):
                    mode = "kicking"
                    kick_end_time = now + KICK_BURST_T
                    last_kick = now
                    near_ball_since = 0.0
                    committed_side = None
                    pass_count += 1
                    print(f"[coop {tag}] KICKING pass #{pass_count}!")

            # -- P2: Ball close + in front -> dribble, align -----------
            elif d_ball < KICK_RANGE and rel_ball[0] > 0:
                dribble = 1
                vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                     ramp_dist=150, stop_dist=0)
                w = clamp(ang_mate * TURN_GAIN, -MAX_W, MAX_W)

                if near_ball_since == 0.0:
                    near_ball_since = now

            # -- P3: Ball moving toward me -> intercept ----------------
            elif bspeed > 200 and ball is not None:
                near_ball_since = 0.0
                dx = me[0] - ball[0]
                dy = me[1] - ball[1]
                dd = max(math.hypot(dx, dy), 1.0)
                approach_spd_val = (bvx * dx + bvy * dy) / dd

                if approach_spd_val > 150:
                    t_arrive = max(dd / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_arrive, 1.5))
                    dribble = 1
                    vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                else:
                    nav, committed_side, is_behind = compute_arc_nav(
                        robot_xy=(me[0], me[1]),
                        ball=ball, aim=mate_pos,
                        behind_dist=BEHIND_DIST,
                        avoid_radius=AVOID_RADIUS,
                        committed_side=committed_side)
                    rel_nav = world2robot(me, nav)
                    if is_behind and d_ball < BALL_NEAR:
                        dribble = 1
                        vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                             ramp_dist=300, stop_dist=0)
                        w = clamp(ang_mate * TURN_GAIN * 0.7,
                                  -MAX_W, MAX_W)
                    else:
                        vx, vy = move_toward(rel_nav, APPROACH_SPD,
                                             ramp_dist=400, stop_dist=10)
                        w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

            # -- P4: Arc behind ball toward mate, drive in -------------
            else:
                near_ball_since = 0.0
                nav, committed_side, is_behind = compute_arc_nav(
                    robot_xy=(me[0], me[1]),
                    ball=ball, aim=mate_pos,
                    behind_dist=BEHIND_DIST,
                    avoid_radius=AVOID_RADIUS,
                    committed_side=committed_side)

                rel_nav = world2robot(me, nav)
                d_nav = math.hypot(rel_nav[0], rel_nav[1])

                if is_behind and d_nav < 300 and d_ball < BALL_NEAR:
                    dribble = 1
                    vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                         ramp_dist=300, stop_dist=0)
                    w = clamp(ang_mate * TURN_GAIN * 0.7,
                              -MAX_W, MAX_W)
                else:
                    vx, vy = move_toward(rel_nav, APPROACH_SPD,
                                         ramp_dist=400, stop_dist=10)
                    w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

            # Face ball when not dribbling
            if dribble == 0 and ball is not None:
                if abs(ang_ball) > 0.04:
                    w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # ==============================================================
        #  KICKING — sustained kick burst toward mate
        # ==============================================================
        elif mode == "kicking":
            if ball is not None:
                rel_ball = world2robot(me, ball)
                d_ball = math.hypot(rel_ball[0], rel_ball[1])
                if d_ball < CONTACT_DIST + 30:
                    kick = 1
                dribble = 1
                vx, vy = move_toward(rel_ball, CHARGE_SPEED,
                                     ramp_dist=100, stop_dist=0)
            else:
                kick = 1
                dribble = 1
                vx = CHARGE_SPEED
                vy = 0.0
            w = _face(me, mate_pos)

            if now > kick_end_time:
                mode = "retreat"
                print(f"[coop {tag}] kick burst done — retreating")

        # ==============================================================
        #  RETREAT — go home, but intercept if ball incoming
        # ==============================================================
        elif mode == "retreat":
            if ball is not None and bspeed > 150:
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd_val = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me
                if approach_spd_val > 100:
                    mode = "home"
                    committed_side = None
                    print(f"[coop {tag}] ball incoming — switching to home")
                    time.sleep(LOOP_RATE)
                    continue

            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, mate_pos)

            if _at(me, home, 200):
                mode = "home"
                committed_side = None
                print(f"[coop {tag}] home — waiting")

        # -- Obstacle avoidance (not during kick/dribble) ---------------
        if mode in ("active", "retreat") and dribble == 0 and kick == 0:
            target_for_avoid = ball if ball is not None else mate_pos
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy

        # -- Speed cap --------------------------------------------------
        spd = math.hypot(vx, vy)
        max_spd = CHARGE_SPEED if kick else APPROACH_SPD * 1.2
        if spd > max_spd:
            vx = vx / spd * max_spd
            vy = vy / spd * max_spd

        # -- Wall braking -----------------------------------------------
        vx, vy = wall_brake(me[0], me[1], vx, vy)

        # -- Rotation compensation --------------------------------------
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
