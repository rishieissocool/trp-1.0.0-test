"""
Cooperative pass-and-score — continuous looping play.

Yellow (passer) and Blue (scorer) work together to pass and score on the +x
goal. After each cycle (pass → receive → shoot), both robots reset to home
and the ball is re-placed so the play loops for ~1 minute.

The scorer *advances* to a receive spot as soon as the passer claims the ball,
and the passer leads the pass to that spot (not the scorer's current position).

Field is 4500 x 2230 mm  (HALF_LEN = 2250, HALF_WID = 1115).
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
    ball_velocity, update_ball_history, predict_ball,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.robot.kick_engine import KickState, kick_tick
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
#  LAYOUT — both start on the left, attack the +x goal together
#
#       y
#    +1115 +----------------------------------------------+
#          |                                        [GOAL] |
#          |  [BLUE]         [RECEIVE]               (+x)  |
#          | (-1500, 500)    (400, 450)                    |
#        0 |   ball(-1300,0)                               |
#          |  [YELLOW]                                     |
#          | (-1500,-500)                                  |
#    -1115 +----------------------------------------------+
#        -2250              0               +2250  x
#
#  Loop:
#   1. Yellow goes for ball, Blue advances to RECEIVE_SPOT
#   2. Yellow passes toward RECEIVE_SPOT (leading the pass)
#   3. Blue intercepts/receives, advances, shoots at +x goal
#   4. Both retreat home, ball re-placed, repeat
# =====================================================================

HOME_YELLOW     = (-1500, -500)
HOME_BLUE       = (-1500, 500)
BALL_START      = (-1300, 0)
GOAL_TARGET     = (HALF_LEN, 0)

# Scorer runs here when passer claims ball — forward and offset
RECEIVE_SPOT    = (400, 450)

# -- Tuning ------------------------------------------------------------
CLAIM_DIST      = 1800      # mm — ball within this = I go for it
APPROACH_SPD    = CRUISE_SPEED
SETUP_PAUSE     = 2.0
RESET_PAUSE     = 2.5       # seconds at home before next cycle
CYCLE_TIMEOUT   = 15.0      # max seconds per cycle before forced reset

# Exported for UI overlay
ATK_START       = HOME_YELLOW
SUP_START       = HOME_BLUE
BALL_TRIGGER    = BALL_START
BALL_TRIGGER_R  = CLAIM_DIST
SUP_SHOOT_SPOT  = RECEIVE_SPOT
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
#  MAIN — one process per robot, continuous looping play
#
#  Roles:
#    passer (yellow) — pick up ball, pass to RECEIVE_SPOT
#    scorer (blue)   — advance to RECEIVE_SPOT, receive, shoot at goal
#
#  Modes:
#    setup   — teleport, place ball
#    home    — wait at home, watch for ball / mate activity
#    run     — (scorer only) advance to RECEIVE_SPOT when passer claims ball
#    active  — kick_engine: approach + align + burst
#    retreat — go home after kicking
#    reset   — pause at home, re-place ball, start next cycle
# =====================================================================

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    role = "passer" if is_yellow else "scorer"
    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = math.atan2(HOME_BLUE[1] - HOME_YELLOW[1],
                          HOME_BLUE[0] - HOME_YELLOW[0]) if is_yellow \
               else 0.0

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
    print(f"[coop {tag}] role={role} home={home}")

    # -- State ----------------------------------------------------------
    mode = "setup"
    frame = None
    last_ft = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False

    ball_history = []
    last_ball_xy = None

    ks = KickState()
    cycle_start = time.time()
    cycle_count = 0
    reset_arrived_time = None

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

        # -- Passer aims at RECEIVE_SPOT, scorer aims at goal -----------
        aim = RECEIVE_SPOT if role == "passer" else GOAL_TARGET

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # -- Cycle timeout: force reset if stuck too long ---------------
        if mode not in ("setup", "reset") and (now - cycle_start) > CYCLE_TIMEOUT:
            mode = "retreat"
            ks.reset()
            print(f"[coop {tag}] cycle timeout — retreating")

        # ==============================================================
        #  SETUP
        # ==============================================================
        if mode == "setup":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, aim)
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
                cycle_start = now
                print(f"[coop {tag}] ready — {role}")

        # ==============================================================
        #  HOME — passer waits then claims ball; scorer watches for
        #         passer activity or incoming ball
        # ==============================================================
        elif mode == "home":
            if ball is not None:
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me

                # Intercept fast incoming ball
                if bspeed > 150 and approach_spd > 100:
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

                # PASSER: claim ball and go active
                if role == "passer":
                    if my_dist < CLAIM_DIST and my_dist < mate_dist - 100:
                        mode = "active"
                        ks.reset()
                        print(f"[coop {tag}] ball claimed — passing")

                # SCORER: detect passer going for ball → advance to receive
                if role == "scorer":
                    # Mate is closer to ball = passer is active → run to receive
                    if mate_dist < CLAIM_DIST and mate_dist < my_dist - 100:
                        mode = "run"
                        print(f"[coop {tag}] passer active — advancing to receive")
                    # Ball very close to me = I have it → shoot
                    elif my_dist < 500 and my_dist < mate_dist - 100:
                        mode = "active"
                        ks.reset()
                        print(f"[coop {tag}] ball is mine — shooting")

            else:
                vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                w = _face(me, aim)

        # ==============================================================
        #  RUN — scorer advances to RECEIVE_SPOT to receive the pass
        # ==============================================================
        elif mode == "run":
            if ball is not None:
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me

                # Ball coming toward me fast — intercept it
                if bspeed > 150 and approach_spd > 80:
                    t_arrive = max(dd_to_me / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_arrive, 2.0))
                    dribble = 1
                    vx, vy = _go_to(me, intercept, APPROACH_SPD * 1.1,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                else:
                    # Advance toward receive spot, face back toward ball
                    vx, vy = _go_to(me, RECEIVE_SPOT, APPROACH_SPD,
                                    stop_r=60, ramp=500)
                    w = _face(me, ball)

                if my_dist < 600:
                    dribble = 1

                # Ball is mine → go active (shoot)
                if my_dist < 500 and my_dist < mate_dist - 100:
                    mode = "active"
                    ks.reset()
                    print(f"[coop {tag}] received pass — shooting!")
            else:
                # No ball visible, keep advancing
                vx, vy = _go_to(me, RECEIVE_SPOT, APPROACH_SPD, stop_r=60)
                w = _face(me, GOAL_TARGET)

        # ==============================================================
        #  ACTIVE — kick engine handles approach + align + burst
        # ==============================================================
        elif mode == "active":
            if ball is None:
                mode = "home"
                time.sleep(LOOP_RATE)
                continue

            # Passer: ball went to mate = pass delivered → retreat
            if role == "passer":
                if my_dist > mate_dist + 200 and my_dist > CLAIM_DIST * 0.7:
                    mode = "retreat"
                    ks.reset()
                    print(f"[coop {tag}] pass delivered — retreating")
                    time.sleep(LOOP_RATE)
                    continue

            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])

            # Intercept fast incoming ball before kick_tick
            if bspeed > 200 and not ks.bursting:
                dx = me[0] - ball[0]
                dy = me[1] - ball[1]
                dd = max(math.hypot(dx, dy), 1.0)
                approach_spd_val = (bvx * dx + bvy * dy) / dd
                if approach_spd_val > 150 and d_ball > KICK_RANGE:
                    t_arrive = max(dd / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_arrive, 1.5))
                    dribble = 1
                    vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                else:
                    kr = kick_tick(ks, me, ball, aim, now, rel_ball, d_ball)
                    vx, vy, w = kr.vx, kr.vy, kr.w
                    kick, dribble = kr.kick, kr.dribble
                    if kr.kick_started:
                        action = "PASSING" if role == "passer" else "SHOOTING"
                        print(f"[coop {tag}] {action}!")
                    if kr.burst_done:
                        mode = "retreat"
                        ks.reset()
                        if role == "passer":
                            print(f"[coop {tag}] pass done — retreating")
                        else:
                            print(f"[coop {tag}] SHOT! — retreating")
            else:
                kr = kick_tick(ks, me, ball, aim, now, rel_ball, d_ball)
                vx, vy, w = kr.vx, kr.vy, kr.w
                kick, dribble = kr.kick, kr.dribble
                if kr.kick_started:
                    action = "PASSING" if role == "passer" else "SHOOTING"
                    print(f"[coop {tag}] {action}!")
                if kr.burst_done:
                    mode = "retreat"
                    ks.reset()
                    if role == "passer":
                        print(f"[coop {tag}] pass done — retreating")
                    else:
                        print(f"[coop {tag}] SHOT! — retreating")

        # ==============================================================
        #  RETREAT — go home, then enter reset for next cycle
        # ==============================================================
        elif mode == "retreat":
            # If ball comes back toward me, react
            if ball is not None and bspeed > 150:
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd_val = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me
                if approach_spd_val > 100 and my_dist < CLAIM_DIST:
                    mode = "home"
                    ks.reset()
                    print(f"[coop {tag}] ball incoming — back to home")
                    time.sleep(LOOP_RATE)
                    continue

            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, ball if ball is not None else aim)

            if _at(me, home, 200):
                mode = "reset"
                reset_arrived_time = now
                ks.reset()
                print(f"[coop {tag}] home — waiting for reset")

        # ==============================================================
        #  RESET — pause briefly at home, re-place ball, start next cycle
        # ==============================================================
        elif mode == "reset":
            vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
            if ball is not None:
                w = _face(me, ball)
            else:
                w = _face(me, aim)

            if reset_arrived_time and (now - reset_arrived_time) > RESET_PAUSE:
                # Passer re-places ball for next cycle
                if is_yellow and sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_START[0] / 1000.0,
                            y=BALL_START[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                        cycle_count += 1
                        print(f"[coop yellow] cycle {cycle_count} — ball reset")
                    except Exception:
                        pass
                mode = "home"
                cycle_start = now
                reset_arrived_time = None
                print(f"[coop {tag}] starting next cycle")

        # -- Obstacle avoidance (all modes except kick burst) -----------
        if not ks.bursting:
            if mode in ("home", "active", "run"):
                target_for_avoid = ball if ball is not None else aim
            else:
                target_for_avoid = home
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, closest_obs, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)

            if dribble == 1 and mode == "active":
                avoid_vx *= 0.4
                avoid_vy *= 0.4

            vx += avoid_vx
            vy += avoid_vy

            if closest_obs < 550:
                if closest_obs <= 260:
                    speed_frac = 0.5
                else:
                    t = (550.0 - closest_obs) / (550.0 - 260.0)
                    speed_frac = 1.0 - 0.25 * (1.0 - math.cos(math.pi * t))
                spd = math.hypot(vx, vy)
                if spd > 0.01:
                    max_near = APPROACH_SPD * speed_frac
                    if spd > max_near:
                        vx = vx / spd * max_near
                        vy = vy / spd * max_near

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
