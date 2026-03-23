"""
Cooperative passing drill — two robots kick the ball back and forth.

Yellow stays in the left half, blue stays in the right half.
Ball is auto-placed near yellow to start. Each robot:
  WAIT → ball comes near → COLLECT → ALIGN toward mate → KICK → RETURN home → repeat

Both robots run identical logic, just mirrored positions.

Field is 4500 × 2230 mm  (HALF_LEN = 2250, HALF_WID = 1115).
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.grSimPacketFactory import grSimPacketFactory
from TeamControl.robot.constants import (
    HALF_LEN, HALF_WID,
    GOAL_HW, GOAL_DEPTH,
    KICK_RANGE,
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
#          │                                      │
#        0 │    [YELLOW]  ←  ball  →   [BLUE]     │
#          │   (-1000,0)    (start)   (1000,0)    │
#          │                                      │
#          │                                      │
#    -1115 └──────────────────────────────────────┘
#        -2250              0               +2250  x
#
#   ~2000mm between robots — nice visible kick distance
# ═══════════════════════════════════════════════════════════════════

# Home positions — each robot stays in its own half
HOME_YELLOW     = (-1000, 0)
HOME_BLUE       = (1000, 0)

# Ball starts near yellow
BALL_START      = (-500, 0)

# How close ball must be to trigger collection
RECEIVE_R       = 700
# Ball trapped
COLLECT_R       = 180
# How close to home before considered "returned"
HOME_R          = 200

# Kick alignment
KICK_ALIGN_TOL  = 0.22             # rad

# Speeds
SPD_MOVE        = CRUISE_SPEED
SPD_DRIBBLE     = DRIBBLE_SPEED
SPD_CHARGE      = CHARGE_SPEED

# Timing
SETUP_PAUSE     = 2.0              # seconds before ball auto-placed

# Exported for UI overlay
ATK_START       = HOME_YELLOW
SUP_START       = HOME_BLUE
BALL_TRIGGER    = BALL_START
BALL_TRIGGER_R  = RECEIVE_R
SUP_SHOOT_SPOT  = HOME_BLUE        # reuse for overlay path endpoint
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


def _angle_err_to(me, target):
    rel = world2robot(me, target)
    return math.atan2(rel[1], rel[0])


def _behind_ball_toward(ball, target, dist_behind=200):
    dx = target[0] - ball[0]
    dy = target[1] - ball[1]
    d = max(math.hypot(dx, dy), 1.0)
    ux, uy = dx / d, dy / d
    return (ball[0] - ux * dist_behind, ball[1] - uy * dist_behind)


# ═══════════════════════════════════════════════════════════════════
#  MAIN — one process per robot, both run same logic
# ═══════════════════════════════════════════════════════════════════

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    # Pick home position based on team
    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = 0.0 if is_yellow else math.pi  # face toward center

    sim = None
    try:
        sim = grSimSender("127.0.0.1", 20011)
    except Exception as e:
        print(f"[coop] grSim sender failed: {e}")

    # ── Teleport to home ─────────────────────────────────────
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
    print(f"  Yellow home: {HOME_YELLOW}")
    print(f"  Blue home:   {HOME_BLUE}")
    print(f"  Ball start:  {BALL_START} (auto-placed)")
    print(f"  Distance:    {_dist(HOME_YELLOW, HOME_BLUE):.0f} mm")
    print("=" * 55)
    print()

    # ── State machine ────────────────────────────────────────
    # States: SETUP → WAIT → COLLECT → ALIGN → KICK → RETURN → WAIT → ...
    state = "SETUP"
    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False
    pass_count = 0

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

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ══════════════════════════════════════════════════════

        if state == "SETUP":
            # Go to home, then yellow auto-places ball after pause
            vx, vy = _go_to(me, home, SPD_MOVE)
            w = _face_angle(me, home_ang)

            if _at(me, home, HOME_R) and (now - setup_time) > SETUP_PAUSE:
                # Only yellow places the ball
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

                state = "WAIT"
                print(f"[coop {tag}] at home — waiting for ball")

        elif state == "WAIT":
            # Stand at home, face the ball (or mate if no ball visible)
            vx, vy = 0.0, 0.0
            if ball is not None:
                w = _face(me, ball)
            else:
                w = _face(me, mate_pos)

            # Ball comes within receive range → go collect
            if ball is not None and _dist(ball, me) < RECEIVE_R:
                state = "COLLECT"
                print(f"[coop {tag}] ball incoming — collecting")

        elif state == "COLLECT":
            # Approach ball with dribbler on
            if ball is None:
                # Lost sight, go back to waiting
                state = "WAIT"
                time.sleep(LOOP_RATE)
                continue

            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])
            dribble = 1
            vx, vy = move_toward(rel_ball, SPD_DRIBBLE,
                                 ramp_dist=200, stop_dist=10)
            w = _face(me, ball)

            if d_ball < COLLECT_R:
                state = "ALIGN"
                print(f"[coop {tag}] collected — aligning toward mate")

        elif state == "ALIGN":
            # Have ball, turn to face teammate
            dribble = 1
            ball_pos = ball if ball is not None else me[:2]
            rel_ball = world2robot(me, ball_pos)
            vx, vy = move_toward(rel_ball, SPD_DRIBBLE * 0.5,
                                 ramp_dist=100, stop_dist=10)
            w = _face(me, mate_pos)

            ang_to_mate = _angle_err_to(me, mate_pos)

            # Aligned and cooldown elapsed → kick
            if abs(ang_to_mate) < KICK_ALIGN_TOL and \
               (now - last_kick) > KICK_COOLDOWN:
                state = "KICK"
                print(f"[coop {tag}] aligned — kicking!")

        elif state == "KICK":
            # Fire the kick
            kick = 1
            dribble = 0
            vx = SPD_CHARGE
            vy = 0.0
            w = _face(me, mate_pos)
            last_kick = now
            pass_count += 1
            print(f"[coop {tag}] PASS #{pass_count} sent!")
            state = "RETURN"

        elif state == "RETURN":
            # Go back to home position
            vx, vy = _go_to(me, home, SPD_MOVE)
            w = _face(me, mate_pos)

            if _at(me, home, HOME_R):
                state = "WAIT"
                print(f"[coop {tag}] back home — waiting")

        # ── Obstacle avoidance (only while moving) ───────────
        if state in ("COLLECT", "RETURN"):
            target_for_avoid = ball if ball is not None else mate_pos
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy

        # Cap speed
        spd = math.hypot(vx, vy)
        max_spd = SPD_CHARGE if kick else SPD_MOVE * 1.2
        if spd > max_spd:
            vx = vx / spd * max_spd
            vy = vy / spd * max_spd

        # Wall braking
        vx, vy = wall_brake(me[0], me[1], vx, vy)

        # Rotation compensation
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
