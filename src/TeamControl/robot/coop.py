"""
Cooperative passing drill — two robots kick the ball back and forth.

Yellow stays in the left half, blue in the right half.  Ball is auto-placed
near yellow to start.  Each robot runs the same state machine:

  IDLE → INTERCEPT → APPROACH → POSSESS → KICK → RETREAT → IDLE (loop)

Uses ball velocity prediction for interception, arc navigation to get behind
the ball, and careful dribble-align-kick sequencing.

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
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED, SPRINT_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
)


# ═══════════════════════════════════════════════════════════════════
#  LAYOUT
#
#       y
#    +1115 ┌──────────────────────────────────────┐
#          │                                      │
#          │                                      │
#        0 │    [YELLOW]  ← ball →    [BLUE]      │
#          │   (-1000,0)              (1000,0)     │
#          │                                      │
#    -1115 └──────────────────────────────────────┘
#        -2250              0               +2250  x
#
#   ~2000mm apart — clear kick distance
# ═══════════════════════════════════════════════════════════════════

HOME_YELLOW     = (-1000, 0)
HOME_BLUE       = (1000, 0)
BALL_START      = (-500, 0)

# ── Thresholds ──────────────────────────────────────────────────
CLAIM_R         = 1200      # ball within this → I go get it
CLOSE_R         = 400       # close enough to start arc-behind approach
POSSESS_R       = KICK_RANGE + 20   # ball trapped on dribbler
KICK_ALIGN      = 0.18      # rad — alignment tolerance for kick
MIN_POSSESS_T   = 0.25      # seconds — hold ball before kicking (let alignment settle)
KICK_BURST_T    = 0.25      # seconds — keep kick+forward on after firing

# ── Speeds ──────────────────────────────────────────────────────
SPD_CRUISE      = CRUISE_SPEED
SPD_DRIBBLE     = DRIBBLE_SPEED
SPD_CHARGE      = CHARGE_SPEED

# ── Timing ──────────────────────────────────────────────────────
SETUP_PAUSE     = 2.0

# Exported for UI overlay
ATK_START       = HOME_YELLOW
SUP_START       = HOME_BLUE
BALL_TRIGGER    = BALL_START
BALL_TRIGGER_R  = CLAIM_R
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


def _face_angle(me, desired_angle):
    err = desired_angle - me[2]
    while err > math.pi:
        err -= 2 * math.pi
    while err < -math.pi:
        err += 2 * math.pi
    return clamp(err * TURN_GAIN, -MAX_W, MAX_W)


def _at(me, target, radius):
    return _dist(me, target) < radius


def _ang_err(me, target):
    """Signed angle from robot heading to target point, radians."""
    rel = world2robot(me, target)
    return math.atan2(rel[1], rel[0])


# ═══════════════════════════════════════════════════════════════════
#  MAIN — one process per robot, both run the same loop
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
    print()
    print("=" * 55)
    print("  COOP PASSING DRILL")
    print("=" * 55)
    print(f"  Yellow: {HOME_YELLOW}   Blue: {HOME_BLUE}")
    print(f"  Ball start: {BALL_START}")
    print("=" * 55)
    print()

    # ── State ────────────────────────────────────────────────
    state = "SETUP"
    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False
    pass_count = 0

    # Ball tracking
    ball_history = []
    last_ball_xy = None

    # Possess timing
    possess_since = 0.0

    # Kick burst
    kick_burst_until = 0.0

    # Arc navigation
    committed_side = None

    print(f"[coop {tag}] robot {robot_id} → SETUP")

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

        # ── Ball velocity tracking ───────────────────────────
        if ball is not None:
            last_ball_xy = update_ball_history(
                ball_history, now, ball, last_ball_xy)
        bvx, bvy, bspeed = ball_velocity(ball_history)

        # ── Am I the closer robot? ───────────────────────────
        my_dist = _dist(ball, me[:2]) if ball else float('inf')
        mate_dist = _dist(ball, mate_pos) if ball else float('inf')
        ball_is_mine = my_dist < mate_dist - 50  # small hysteresis

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ── Active kick burst (keep driving after kick) ──────
        if now < kick_burst_until:
            kick = 1
            dribble = 0
            vx = SPD_CHARGE
            vy = 0.0
            w = _face(me, mate_pos)
        # ══════════════════════════════════════════════════════
        #  STATE MACHINE
        # ══════════════════════════════════════════════════════

        elif state == "SETUP":
            vx, vy = _go_to(me, home, SPD_CRUISE)
            w = _face_angle(me, home_ang)
            if _at(me, home, 200) and (now - setup_time) > SETUP_PAUSE:
                if is_yellow and not ball_placed and sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_START[0] / 1000.0,
                            y=BALL_START[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                        print("[coop yellow] ball placed")
                    except Exception:
                        pass
                    ball_placed = True
                state = "IDLE"
                print(f"[coop {tag}] ready — waiting for ball")

        elif state == "IDLE":
            # Stay at home, face ball
            vx, vy = _go_to(me, home, SPD_CRUISE, stop_r=80)
            if ball is not None:
                w = _face(me, ball)
            else:
                w = _face(me, mate_pos)

            # Ball is mine and within claim range → go intercept
            if ball is not None and ball_is_mine and my_dist < CLAIM_R:
                state = "INTERCEPT"
                committed_side = None
                print(f"[coop {tag}] ball is mine — intercepting")

        elif state == "INTERCEPT":
            # Move toward where ball will be (predict 0.5s ahead)
            if ball is None:
                state = "IDLE"
                time.sleep(LOOP_RATE)
                continue

            # Predict ball position
            if bspeed > 100:
                target = predict_ball(ball, (bvx, bvy), 0.5)
            else:
                target = ball

            vx, vy = _go_to(me, target, SPD_CRUISE, stop_r=20, ramp=500)
            w = _face(me, ball)

            # Ball stopped or very close → approach properly
            if my_dist < CLOSE_R:
                state = "APPROACH"
                print(f"[coop {tag}] ball close — getting behind it")

            # Ball went to mate → go back
            if not ball_is_mine and my_dist > CLAIM_R * 0.8:
                state = "IDLE"
                committed_side = None
                print(f"[coop {tag}] ball went to mate — returning")

        elif state == "APPROACH":
            # Use arc navigation to get behind ball aimed at mate
            if ball is None:
                state = "IDLE"
                time.sleep(LOOP_RATE)
                continue

            nav, committed_side, is_behind = compute_arc_nav(
                robot_xy=(me[0], me[1]),
                ball=ball,
                aim=mate_pos,
                behind_dist=BEHIND_DIST,
                avoid_radius=AVOID_RADIUS,
                committed_side=committed_side,
            )

            rel_nav = world2robot(me, nav)
            d_nav = math.hypot(rel_nav[0], rel_nav[1])
            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])

            if is_behind and d_ball < BALL_NEAR:
                # Lined up behind ball — drive into it with dribbler
                dribble = 1
                vx, vy = move_toward(rel_ball, SPD_DRIBBLE,
                                     ramp_dist=250, stop_dist=10)
                # Face the mate (kick direction) not the ball
                ang_to_mate = _ang_err(me, mate_pos)
                w = clamp(ang_to_mate * TURN_GAIN * 0.7, -MAX_W, MAX_W)

                if d_ball < POSSESS_R:
                    state = "POSSESS"
                    possess_since = now
                    print(f"[coop {tag}] ball on dribbler — aligning to mate")
            else:
                # Navigate toward arc waypoint
                vx, vy = move_toward(rel_nav, SPD_CRUISE,
                                     ramp_dist=400, stop_dist=10)
                w = _face(me, ball)

            # Ball escaped to mate
            if not ball_is_mine and d_ball > CLAIM_R * 0.8:
                state = "IDLE"
                committed_side = None

        elif state == "POSSESS":
            # Ball on dribbler — turn to face mate, then kick
            if ball is None:
                state = "IDLE"
                time.sleep(LOOP_RATE)
                continue

            dribble = 1
            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])

            # Ball escaped dribbler
            if d_ball > POSSESS_R * 2:
                state = "APPROACH"
                committed_side = None
                print(f"[coop {tag}] ball escaped — re-approaching")
                time.sleep(LOOP_RATE)
                continue

            # Gentle forward pressure to keep ball on dribbler
            vx = SPD_DRIBBLE * 0.3
            vy = 0.0

            # Turn toward mate — slow rotation to not lose ball
            ang_to_mate = _ang_err(me, mate_pos)
            w = clamp(ang_to_mate * TURN_GAIN * 0.5, -MAX_W * 0.5, MAX_W * 0.5)

            # Check if aligned long enough and kick cooldown clear
            aligned = abs(ang_to_mate) < KICK_ALIGN
            held_long = (now - possess_since) > MIN_POSSESS_T
            can_kick = (now - last_kick) > KICK_COOLDOWN

            if aligned and held_long and can_kick:
                # FIRE
                kick = 1
                dribble = 0
                vx = SPD_CHARGE
                vy = 0.0
                last_kick = now
                kick_burst_until = now + KICK_BURST_T
                pass_count += 1
                print(f"[coop {tag}] PASS #{pass_count}!")
                state = "RETREAT"

        elif state == "RETREAT":
            # Go back to home
            vx, vy = _go_to(me, home, SPD_CRUISE)
            w = _face(me, mate_pos)
            if _at(me, home, 200):
                state = "IDLE"
                committed_side = None
                print(f"[coop {tag}] home — waiting")

            # If ball bounced back to us before we got home, grab it
            if ball is not None and ball_is_mine and my_dist < CLOSE_R:
                state = "APPROACH"
                committed_side = None

        # ── Obstacle avoidance (only during movement) ────────
        if state in ("INTERCEPT", "APPROACH", "RETREAT"):
            target_for_avoid = ball if ball is not None else mate_pos
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy

        # ── Speed cap ────────────────────────────────────────
        spd = math.hypot(vx, vy)
        max_spd = SPD_CHARGE if kick else SPD_CRUISE * 1.2
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
