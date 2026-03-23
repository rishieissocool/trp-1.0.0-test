"""
Cooperative passing drill — two robots kick the ball back and forth.

Yellow in left half, blue in right half. Each robot uses the SAME proven
kick logic from striker.py: arc behind ball → drive into ball with dribbler
facing the aim target → kick when aligned. Aim target = teammate.

After kicking, robot retreats home and waits for the return pass.
Receiving robot faces the incoming ball with dribbler on to catch it,
then runs the same kick sequence back.

Field is 4500 × 2230 mm  (HALF_LEN = 2250, HALF_WID = 1115).
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


# ═══════════════════════════════════════════════════════════════════
#  LAYOUT
#
#       y
#    +1115 ┌──────────────────────────────────────┐
#          │                                      │
#        0 │    [YELLOW]  ← ball →    [BLUE]      │
#          │   (-1000,0)              (1000,0)     │
#          │                                      │
#    -1115 └──────────────────────────────────────┘
#        -2250              0               +2250  x
# ═══════════════════════════════════════════════════════════════════

HOME_YELLOW     = (-1000, 0)
HOME_BLUE       = (1000, 0)
BALL_START      = (-500, 0)

# ── Tuning (same as striker) ────────────────────────────────────
SHOOT_RANGE     = 350       # mm — take a kick from this far if aimed
SHOOT_ALIGN     = 0.18      # rad — alignment for long-range kick
KICK_ALIGN_TOL  = 0.18      # rad — alignment for close-range kick
FORCE_KICK_TIME = 0.6       # s — force kick after this long near ball
CLAIM_DIST      = 1400      # mm — ball within this = my ball
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


# ═══════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════
#  MAIN — one process per robot
# ═══════════════════════════════════════════════════════════════════

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

    # ── Teleport ─────────────────────────────────────────────
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

    # ── Tracking state ───────────────────────────────────────
    # Three modes: "setup", "home", "active", "retreat"
    #   setup   — wait, place ball
    #   home    — at home, face ball, dribbler on if ball near. Ball mine → active
    #   active  — striker-like: arc behind ball, drive+dribble, kick when aligned
    #   retreat — go back home after kicking
    mode = "setup"

    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False
    pass_count = 0

    # Ball tracking for velocity
    ball_history = []
    last_ball_xy = None

    # Striker-like state
    committed_side = None
    near_ball_since = 0.0

    while is_running.is_set():
        now = time.time()

        # ── Fetch frame ──────────────────────────────────────
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

        # ── Ball velocity ────────────────────────────────────
        if ball is not None:
            last_ball_xy = update_ball_history(
                ball_history, now, ball, last_ball_xy)
        bvx, bvy, bspeed = ball_velocity(ball_history)

        # ── Ownership: am I closer to the ball? ──────────────
        my_dist = _dist(ball, me[:2]) if ball else float('inf')
        mate_dist = _dist(ball, mate_pos) if ball else float('inf')

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ══════════════════════════════════════════════════════
        #  SETUP — teleport, auto-place ball
        # ══════════════════════════════════════════════════════
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

        # ══════════════════════════════════════════════════════
        #  HOME — wait for the ball to come to me
        # ══════════════════════════════════════════════════════
        elif mode == "home":
            # Stay at home, face the ball
            vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)

            if ball is not None:
                w = _face(me, ball)

                # Dribbler on when ball is approaching (catch it)
                if my_dist < 500:
                    dribble = 1

                # Ball is mine → go get it
                if my_dist < CLAIM_DIST and my_dist < mate_dist - 100:
                    mode = "active"
                    committed_side = None
                    near_ball_since = 0.0
                    print(f"[coop {tag}] ball is mine — going for it")
            else:
                w = _face(me, mate_pos)

        # ══════════════════════════════════════════════════════
        #  ACTIVE — striker-like kick logic (proven pattern)
        #
        #  Priority each tick:
        #    1. SHOOT: close + aimed → kick + charge
        #    2. KICK:  close + in front → dribble + align + kick
        #    3. APPROACH: arc behind ball → drive toward ball
        # ══════════════════════════════════════════════════════
        elif mode == "active":
            if ball is None:
                mode = "home"
                time.sleep(LOOP_RATE)
                continue

            # Ball went to mate? Give up and go home.
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

            kicked = False

            # ── 1. SHOOT — ball in front, aimed at mate, kick now ──
            if (d_ball < SHOOT_RANGE and rel_ball[0] > 0
                    and abs(ang_mate) < SHOOT_ALIGN
                    and (now - last_kick) > KICK_COOLDOWN):
                kick = 1
                dribble = 0
                # Charge forward into ball (stop_dist=0 → drive THROUGH)
                vx, vy = move_toward(rel_ball, CHARGE_SPEED,
                                     ramp_dist=100, stop_dist=0)
                w = clamp(ang_mate * TURN_GAIN, -MAX_W, MAX_W)
                last_kick = now
                near_ball_since = 0.0
                committed_side = None
                kicked = True

            # ── 2. KICK — ball close + in front, dribble & align ──
            elif d_ball < KICK_RANGE and rel_ball[0] > 0:
                dribble = 1
                # Drive into ball with dribbler
                vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                     ramp_dist=150, stop_dist=10)
                committed_side = None

                # Rotate toward mate (the aim)
                w = clamp(ang_mate * TURN_GAIN, -MAX_W, MAX_W)

                # Track time near ball
                if near_ball_since == 0.0:
                    near_ball_since = now
                force_kick = (now - near_ball_since) > FORCE_KICK_TIME

                # Kick when aligned OR forced after timeout
                if (now - last_kick) > KICK_COOLDOWN and (
                        abs(ang_mate) < KICK_ALIGN_TOL or force_kick):
                    kick = 1
                    dribble = 0
                    vx = DRIBBLE_SPD
                    vy = 0.0
                    last_kick = now
                    near_ball_since = 0.0
                    kicked = True

            # ── 3. APPROACH — arc behind ball, drive toward it ─────
            else:
                near_ball_since = 0.0

                # If ball is moving fast toward me, intercept it
                if bspeed > 200:
                    # Approach speed for ball → me
                    dx = me[0] - ball[0]
                    dy = me[1] - ball[1]
                    dd = max(math.hypot(dx, dy), 1.0)
                    approach_spd = (bvx * dx + bvy * dy) / dd

                    if approach_spd > 150:
                        # Ball coming toward me — predict intercept point
                        t_arrive = max(dd / max(bspeed, 1.0), 0.1)
                        intercept = predict_ball(
                            ball, (bvx, bvy), min(t_arrive, 1.5))
                        # Go to intercept, face ball, dribbler on
                        dribble = 1
                        vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                        stop_r=30, ramp=500)
                        w = _face(me, ball)
                    else:
                        # Ball moving but not toward me — chase it
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
                                                 ramp_dist=300, stop_dist=10)
                            w = clamp(ang_mate * TURN_GAIN * 0.7,
                                      -MAX_W, MAX_W)
                        else:
                            vx, vy = move_toward(rel_nav, APPROACH_SPD,
                                                 ramp_dist=400, stop_dist=10)
                            w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)
                else:
                    # Ball is slow/stationary — standard arc approach
                    nav, committed_side, is_behind = compute_arc_nav(
                        robot_xy=(me[0], me[1]),
                        ball=ball, aim=mate_pos,
                        behind_dist=BEHIND_DIST,
                        avoid_radius=AVOID_RADIUS,
                        committed_side=committed_side)

                    rel_nav = world2robot(me, nav)
                    d_nav = math.hypot(rel_nav[0], rel_nav[1])

                    if is_behind and d_nav < 300 and d_ball < BALL_NEAR:
                        # Behind ball — drive into it with dribbler, face mate
                        dribble = 1
                        vx, vy = move_toward(rel_ball, DRIBBLE_SPD,
                                             ramp_dist=300, stop_dist=10)
                        w = clamp(ang_mate * TURN_GAIN * 0.7,
                                  -MAX_W, MAX_W)
                    else:
                        # Navigate to arc waypoint, face ball
                        vx, vy = move_toward(rel_nav, APPROACH_SPD,
                                             ramp_dist=400, stop_dist=10)
                        w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

            # Always face ball when not dribbling/kicking
            if kick == 0 and dribble == 0 and ball is not None:
                ang_ball = math.atan2(rel_ball[1], rel_ball[0])
                if abs(ang_ball) > 0.04:
                    w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

            # After kicking → retreat home
            if kicked:
                pass_count += 1
                print(f"[coop {tag}] PASS #{pass_count}!")
                mode = "retreat"

        # ══════════════════════════════════════════════════════
        #  RETREAT — go back home after kicking
        # ══════════════════════════════════════════════════════
        elif mode == "retreat":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, mate_pos)

            if _at(me, home, 200):
                mode = "home"
                committed_side = None
                print(f"[coop {tag}] home — waiting")

        # ── Obstacle avoidance (not while possessing ball) ───
        if mode in ("active",) and dribble == 0 and kick == 0:
            target_for_avoid = ball if ball is not None else mate_pos
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy
        elif mode == "retreat":
            rel_target = world2robot(me, mate_pos)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy

        # ── Speed cap ────────────────────────────────────────
        spd = math.hypot(vx, vy)
        max_spd = CHARGE_SPEED if kick else APPROACH_SPD * 1.2
        if spd > max_spd:
            vx = vx / spd * max_spd
            vy = vy / spd * max_spd

        # ── Wall braking ─────────────────────────────────────
        vx, vy = wall_brake(me[0], me[1], vx, vy)

        # ── Rotation compensation ────────────────────────────
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
