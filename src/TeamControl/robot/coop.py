"""
Cooperative set-piece — staged kick-pass and score for qualification video.

Fully automatic: robots teleport, ball auto-placed, then:
  ATTACKER (yellow):  approach ball → align toward support → KICK pass
  SUPPORT  (blue):    wait at mark → collect kicked ball → dribble → KICK to score

Both passes use proper kick commands (each robot has its own 5s cooldown).

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
#  PLAY GEOMETRY  (all mm)
#
#       y
#    +1115 ┌──────────────────────────────────────┐
#          │                                      │
#     +550 │                [SUP]                 │
#          │              (300,550)                │
#          │            ↗                         │
#          │         kick                [SHOOT]   │
#          │       ↗                   (1800,150) │
#        0 │ [ATK]···[BALL]·····················[GOAL]
#          │(-1500,0)(-900,0)              +2250  │
#          │                                      │
#    -1115 └──────────────────────────────────────┘
#        -2250              0               +2250  x
#
#   Distance ball→support: ~1300mm (nice visible kick)
#   Distance support→shoot: ~1550mm (clear dribble run)
#   Distance shoot→goal: ~450mm (close range finish)
# ═══════════════════════════════════════════════════════════════════

# ── Ball ─────────────────────────────────────────────────────────
BALL_TRIGGER    = (-900, 0)
BALL_TRIGGER_R  = 300

# ── Goal ─────────────────────────────────────────────────────────
GOAL_X          = HALF_LEN         # +2250

# ── Attacker (yellow) ───────────────────────────────────────────
ATK_START       = (-1500, 0)       # well behind ball
ATK_START_ANG   = 0.0

# ── Support (blue) ──────────────────────────────────────────────
SUP_START       = (300, 550)       # far ahead and high — good pass distance
SUP_START_ANG   = math.pi + 0.5   # facing back toward ball area

SUP_RECEIVE_R   = 550              # ball "arrived" when this close
SUP_COLLECT_R   = 180              # ball trapped

# After collecting, dribble to shooting spot
SUP_SHOOT_SPOT  = (1800, 150)      # close to goal, slightly above center
SUP_SHOOT_R     = 200

SUP_SHOOT_ALIGN = 0.22             # rad alignment tolerance
FORCE_SHOOT_T   = 1.5              # force kick after this long at shoot spot

# ── Speeds ──────────────────────────────────────────────────────
SPD_MOVE        = CRUISE_SPEED
SPD_DRIBBLE     = DRIBBLE_SPEED
SPD_CHARGE      = CHARGE_SPEED

# ── Timing ──────────────────────────────────────────────────────
SETUP_PAUSE     = 2.0              # seconds after teleport before ball placement
BALL_SETTLE     = 0.5              # seconds after ball placement before play starts

# ── Kick approach ───────────────────────────────────────────────
KICK_APPROACH_R = KICK_RANGE + 30  # get within this to start aligning
KICK_ALIGN_TOL  = 0.20             # rad — how aligned before kicking


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
    """Return signed angle error from robot heading to target, in radians."""
    rel = world2robot(me, target)
    return math.atan2(rel[1], rel[0])


def _behind_ball_toward(ball, target, dist_behind=250):
    """Point that is `dist_behind` mm behind `ball` on the line ball→target."""
    dx = target[0] - ball[0]
    dy = target[1] - ball[1]
    d = max(math.hypot(dx, dy), 1.0)
    ux, uy = dx / d, dy / d
    return (ball[0] - ux * dist_behind, ball[1] - uy * dist_behind)


def _pick_goal_aim(frame, is_yellow):
    """Aim at the far corner away from any goalie."""
    aim_x = GOAL_X + GOAL_DEPTH * 0.3
    opp_yellow = not is_yellow

    gk_y = None
    for oid in range(6):
        try:
            opp = frame.get_yellow_robots(isYellow=opp_yellow, robot_id=oid)
            if isinstance(opp, int) or opp is None:
                continue
            op = opp.position
            if abs(float(op[0]) - GOAL_X) < 400:
                gk_y = float(op[1])
                break
        except Exception:
            continue

    if gk_y is not None:
        aim_y = -GOAL_HW * 0.6 if gk_y > 0 else GOAL_HW * 0.6
    else:
        aim_y = -GOAL_HW * 0.4

    return (aim_x, aim_y)


# ═══════════════════════════════════════════════════════════════════
#  MAIN — one process per robot
# ═══════════════════════════════════════════════════════════════════

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    sim = None
    try:
        sim = grSimSender("127.0.0.1", 20011)
    except Exception as e:
        print(f"[coop] grSim sender failed: {e}")

    # ── Teleport to starting marks ───────────────────────────
    if sim:
        try:
            if is_yellow:
                pkt = grSimPacketFactory.robot_replacement_command(
                    x=ATK_START[0] / 1000.0, y=ATK_START[1] / 1000.0,
                    orientation=ATK_START_ANG,
                    robot_id=robot_id, isYellow=True)
            else:
                pkt = grSimPacketFactory.robot_replacement_command(
                    x=SUP_START[0] / 1000.0, y=SUP_START[1] / 1000.0,
                    orientation=SUP_START_ANG,
                    robot_id=robot_id, isYellow=False)
            sim.send_packet(pkt)
            time.sleep(0.1)
        except Exception as e:
            print(f"[coop] teleport failed: {e}")

    # ── Console info ─────────────────────────────────────────
    tag = "yellow" if is_yellow else "blue"
    print()
    print("=" * 55)
    print("  COOP SET-PIECE — QUALIFICATION PLAY")
    print("=" * 55)
    if is_yellow:
        print(f"  Attacker (yellow #{robot_id}):  {ATK_START}")
        print(f"  Support  (blue #{teammate_id}):   {SUP_START}")
        print(f"  Ball auto-placed at:       {BALL_TRIGGER}")
        print(f"  Shoot spot:                {SUP_SHOOT_SPOT}")
        print(f"  Scoring on:                +x goal (x = {GOAL_X})")
        print("-" * 55)
        print(f"  Play auto-starts in ~{SETUP_PAUSE + BALL_SETTLE}s")
    print("=" * 55)
    print()

    # ── State machine ────────────────────────────────────────
    if is_yellow:
        state = "ATK_SETUP"
    else:
        state = "SUP_SETUP"

    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    shoot_spot_since = 0.0
    setup_time = time.time()
    ball_placed_time = 0.0

    print(f"[coop {tag}] robot {robot_id} → {state}")

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

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ══════════════════════════════════════════════════════
        #  ATTACKER (yellow) — approach ball, align, kick-pass
        # ══════════════════════════════════════════════════════

        if state == "ATK_SETUP":
            vx, vy = _go_to(me, ATK_START, SPD_MOVE)
            w = _face(me, BALL_TRIGGER)
            if _at(me, ATK_START, 120) and (now - setup_time) > SETUP_PAUSE:
                if sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_TRIGGER[0] / 1000.0,
                            y=BALL_TRIGGER[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                        print("[coop yellow] ball placed — settling...")
                    except Exception as e:
                        print(f"[coop yellow] ball placement failed: {e}")
                ball_placed_time = now
                state = "ATK_WAIT_SETTLE"

        elif state == "ATK_WAIT_SETTLE":
            vx, vy = 0.0, 0.0
            w = _face(me, BALL_TRIGGER)
            if (now - ball_placed_time) > BALL_SETTLE:
                state = "ATK_APPROACH"
                print("[coop yellow] approaching ball")

        elif state == "ATK_APPROACH":
            # Drive to behind-ball position aimed at support
            ball_pos = ball if ball is not None else BALL_TRIGGER
            sup_pos = (mate[0], mate[1]) if mate is not None else SUP_START
            behind = _behind_ball_toward(ball_pos, sup_pos, dist_behind=200)

            vx, vy = _go_to(me, behind, SPD_MOVE, stop_r=20, ramp=300)
            w = _face(me, sup_pos)

            # Close enough and roughly aimed → start final alignment
            if _at(me, behind, 120):
                state = "ATK_ALIGN"
                print("[coop yellow] behind ball — aligning for kick-pass")

        elif state == "ATK_ALIGN":
            # Creep toward ball with dribbler, rotate to face support
            ball_pos = ball if ball is not None else BALL_TRIGGER
            sup_pos = (mate[0], mate[1]) if mate is not None else SUP_START

            dribble = 1
            rel_ball = world2robot(me, ball_pos)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])
            vx, vy = move_toward(rel_ball, SPD_DRIBBLE,
                                 ramp_dist=200, stop_dist=10)
            w = _face(me, sup_pos)

            # Check alignment to support
            ang_to_sup = _angle_err_to(me, sup_pos)

            if d_ball < KICK_RANGE and abs(ang_to_sup) < KICK_ALIGN_TOL \
               and (now - last_kick) > KICK_COOLDOWN:
                kick = 1
                dribble = 0
                vx = SPD_CHARGE
                vy = 0.0
                last_kick = now
                state = "ATK_DONE"
                print("[coop yellow] KICK-PASS delivered!")

        elif state == "ATK_DONE":
            vx, vy, w = 0.0, 0.0, 0.0

        # ══════════════════════════════════════════════════════
        #  SUPPORT (blue) — wait, collect, dribble, kick-score
        # ══════════════════════════════════════════════════════

        elif state == "SUP_SETUP":
            vx, vy = _go_to(me, SUP_START, SPD_MOVE)
            w = _face_angle(me, SUP_START_ANG)
            if _at(me, SUP_START, 120):
                state = "SUP_WAIT"
                print("[coop blue] at receive mark — waiting for pass")

        elif state == "SUP_WAIT":
            vx, vy = 0.0, 0.0
            if ball is not None:
                w = _face(me, ball)
            else:
                w = _face(me, BALL_TRIGGER)
            if ball is not None and _dist(ball, me) < SUP_RECEIVE_R:
                state = "SUP_COLLECT"
                print("[coop blue] ball incoming — collecting")

        elif state == "SUP_COLLECT":
            if ball is None:
                time.sleep(LOOP_RATE)
                continue
            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])
            dribble = 1
            vx, vy = move_toward(rel_ball, SPD_DRIBBLE,
                                 ramp_dist=200, stop_dist=10)
            w = _face(me, ball)
            if d_ball < SUP_COLLECT_R:
                state = "SUP_DRIBBLE"
                print("[coop blue] ball collected — dribbling to shoot spot")

        elif state == "SUP_DRIBBLE":
            dribble = 1
            goal_aim = _pick_goal_aim(frame, is_yellow)
            vx, vy = _go_to(me, SUP_SHOOT_SPOT, SPD_DRIBBLE, stop_r=30)
            w = _face(me, goal_aim)
            if _at(me, SUP_SHOOT_SPOT, SUP_SHOOT_R):
                state = "SUP_SHOOT"
                shoot_spot_since = now
                print("[coop blue] at shoot spot — aligning for shot")

        elif state == "SUP_SHOOT":
            dribble = 1
            vx, vy = 0.0, 0.0
            goal_aim = _pick_goal_aim(frame, is_yellow)
            rel_aim = world2robot(me, goal_aim)
            ang_aim = math.atan2(rel_aim[1], rel_aim[0])
            w = clamp(ang_aim * TURN_GAIN, -MAX_W, MAX_W)

            force = (now - shoot_spot_since) > FORCE_SHOOT_T

            if (abs(ang_aim) < SUP_SHOOT_ALIGN or force) and \
               (now - last_kick) > KICK_COOLDOWN:
                kick = 1
                dribble = 0
                vx = SPD_CHARGE
                vy = 0.0
                last_kick = now
                state = "SUP_DONE"
                print("[coop blue] SHOT TAKEN — GOAL!")

        elif state == "SUP_DONE":
            vx, vy, w = 0.0, 0.0, 0.0

        # ── Obstacle avoidance (only while actively moving) ──
        if state not in ("ATK_SETUP", "ATK_WAIT_SETTLE", "ATK_DONE",
                         "SUP_SETUP", "SUP_WAIT", "SUP_DONE"):
            target_for_avoid = ball if ball is not None else BALL_TRIGGER
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
