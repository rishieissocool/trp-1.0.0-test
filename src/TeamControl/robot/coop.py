"""
Cooperative set-piece — scripted pass-and-score play with smart targeting.

Both bots teleport to fixed starting positions on the left side of the field.
The play starts when you place the ball at the trigger spot and it stays
there for 2 seconds.

  ATTACKER (yellow):  go to ball → compute angle toward support → charge through ball
  SUPPORT  (blue):    park at receive spot → collect ball → dribble to goal → shoot

Positions are hardcoded starting marks.  Angles and charge targets are
computed dynamically from actual robot positions each tick.

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
#  HARDCODED PLAY COORDINATES  (all mm, facing +x = right)
#
#  Field: 4500 × 2230,  x ∈ [-2250, +2250],  y ∈ [-1115, +1115]
#
#  Layout (to scale-ish):
#
#       y
#    +1115 ┌──────────────────────────────────────┐
#          │                                      │
#     +400 │       [SUP]                          │
#          │      (-400, 400)                     │
#        0 │  [ATK]·····[BALL]···············[GOAL]
#          │(-1200,0) (-700,0)              +2250│
#     -400 │                                      │
#          │                                      │
#    -1115 └──────────────────────────────────────┘
#        -2250              0               +2250  x
#
# ═══════════════════════════════════════════════════════════════════

# Where the ball must be placed to trigger the play
BALL_TRIGGER    = (-700, 0)
BALL_TRIGGER_R  = 300        # mm — ball must be within this radius
BALL_STABLE_T   = 2.0        # seconds — ball must stay in zone for this long

# Goal we're scoring on (right side)
GOAL_X          = HALF_LEN   # +2250

# ── Attacker (yellow) ────────────────────────────────────────────
ATK_START       = (-1200, -100)    # starting mark — behind ball, slightly below
ATK_START_ANG   = 0.0              # facing right

ATK_ARRIVE_R    = 120              # mm — close enough to lineup point

# ── Support (blue) ───────────────────────────────────────────────
SUP_START       = (-400, 400)      # receiving mark — ahead, offset up
SUP_START_ANG   = math.pi + 0.4   # ~207° — facing back toward ball

SUP_RECEIVE_R   = 450              # mm — ball is "arrived" when this close
SUP_COLLECT_R   = 180              # mm — ball is collected (trapped)

# After collecting, dribble to this shooting spot
SUP_SHOOT_SPOT  = (1500, 200)      # centered-ish in front of goal
SUP_SHOOT_R     = 250              # mm — close enough to shooting spot

# Aim inside the goal
SUP_GOAL_AIM    = (GOAL_X + GOAL_DEPTH * 0.3, -GOAL_HW * 0.35)
SUP_SHOOT_ALIGN = 0.25             # rad — alignment to fire

# Speeds
SPD_MOVE        = CRUISE_SPEED
SPD_DRIBBLE     = DRIBBLE_SPEED
SPD_CHARGE      = CHARGE_SPEED
SPD_SPRINT      = SPRINT_SPEED

FORCE_SHOOT_T   = 1.5              # seconds — force kick after this long at shoot spot


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


def _compute_lineup_and_charge(ball, support_pos):
    """Dynamically compute the attacker's lineup point and charge target
    based on where the ball and support actually are."""
    bx, by = ball[0], ball[1]
    sx, sy = support_pos[0], support_pos[1]

    # Direction from ball toward support
    dx = sx - bx
    dy = sy - by
    d = math.hypot(dx, dy)
    if d < 1:
        dx, dy = 1.0, 0.0
        d = 1.0
    ux, uy = dx / d, dy / d

    # Lineup: 200mm behind the ball (opposite side from support)
    lineup = (bx - ux * 200, by - uy * 200)
    lineup_ang = math.atan2(uy, ux)

    # Charge target: 400mm past the support (overshoot)
    charge_tgt = (sx + ux * 400, sy + uy * 400)

    return lineup, lineup_ang, charge_tgt


def _pick_goal_aim(frame, is_yellow):
    """Pick a goal aim point, preferring the side away from any goalie."""
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
        aim_y = -GOAL_HW * 0.35

    return (aim_x, aim_y)


# ═══════════════════════════════════════════════════════════════════
#  MAIN ENTRY — one process per robot
# ═══════════════════════════════════════════════════════════════════

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    # ── Teleport to starting marks ───────────────────────────
    try:
        sim = grSimSender("127.0.0.1", 20011)
        if is_yellow:
            pkt = grSimPacketFactory.robot_replacement_command(
                x=ATK_START[0] / 1000.0, y=ATK_START[1] / 1000.0,
                orientation=ATK_START_ANG,
                robot_id=robot_id, isYellow=True)
            sim.send_packet(pkt)
        else:
            pkt = grSimPacketFactory.robot_replacement_command(
                x=SUP_START[0] / 1000.0, y=SUP_START[1] / 1000.0,
                orientation=SUP_START_ANG,
                robot_id=robot_id, isYellow=False)
            sim.send_packet(pkt)
        time.sleep(0.15)
    except Exception as e:
        print(f"[coop] teleport failed: {e}")

    # ── Print the set positions for the user ─────────────────
    print()
    print("=" * 55)
    print("  COOP SET-PIECE — STARTING POSITIONS")
    print("=" * 55)
    print(f"  Attacker (yellow {robot_id if is_yellow else teammate_id}):"
          f"  ({ATK_START[0]}, {ATK_START[1]}) mm")
    print(f"  Support  (blue   {teammate_id if is_yellow else robot_id}):"
          f"  ({SUP_START[0]}, {SUP_START[1]}) mm")
    print(f"  Ball trigger zone:"
          f"  ({BALL_TRIGGER[0]}, {BALL_TRIGGER[1]}) ± {BALL_TRIGGER_R} mm")
    print(f"  Scoring on:        +x goal (x = {GOAL_X})")
    print("-" * 55)
    print(f"  Place the ball near ({BALL_TRIGGER[0]}, {BALL_TRIGGER[1]})")
    print(f"  and hold it still for {BALL_STABLE_T}s to start the play.")
    print("=" * 55)
    print()

    # ── State machine ────────────────────────────────────────
    if is_yellow:
        state = "ATK_GO_TO_MARK"
    else:
        state = "SUP_GO_TO_MARK"

    frame = None
    last_ft = 0.0
    last_kick = 0.0
    prev_obs_pos = {}
    shoot_spot_since = 0.0

    # Ball-in-zone timer (shared concept, each process tracks independently)
    ball_in_zone_since = 0.0

    print(f"[coop {'yellow' if is_yellow else 'blue'}] robot {robot_id} → {state}")

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

        # Get teammate position for dynamic computations
        mate = _get_robot(frame, mate_is_yellow, teammate_id)

        # ── Track ball stability in trigger zone ─────────────
        if ball is not None and _dist(ball, BALL_TRIGGER) < BALL_TRIGGER_R:
            if ball_in_zone_since == 0.0:
                ball_in_zone_since = now
        else:
            ball_in_zone_since = 0.0

        ball_stable = (ball_in_zone_since > 0.0 and
                       (now - ball_in_zone_since) >= BALL_STABLE_T)

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ══════════════════════════════════════════════════════
        #  ATTACKER STATE MACHINE  (yellow)
        # ══════════════════════════════════════════════════════

        if state == "ATK_GO_TO_MARK":
            vx, vy = _go_to(me, ATK_START, SPD_MOVE)
            w = _face_angle(me, ATK_START_ANG)
            if _at(me, ATK_START, 120):
                state = "ATK_WAIT_BALL"
                print(f"[coop yellow] at mark — place ball near"
                      f" ({BALL_TRIGGER[0]}, {BALL_TRIGGER[1]})"
                      f" and hold for {BALL_STABLE_T}s")

        elif state == "ATK_WAIT_BALL":
            # Sit at mark, face toward ball trigger spot
            vx, vy = 0.0, 0.0
            w = _face(me, BALL_TRIGGER)
            # Only start when ball has been stable for 2 seconds
            if ball_stable:
                state = "ATK_LINEUP"
                print("[coop yellow] ball stable — lining up for pass")

        elif state == "ATK_LINEUP":
            # Compute lineup dynamically from ball and support positions
            ball_pos = ball if ball is not None else BALL_TRIGGER
            sup_pos = (mate[0], mate[1]) if mate is not None else SUP_START
            lineup, lineup_ang, _ = _compute_lineup_and_charge(ball_pos, sup_pos)

            vx, vy = _go_to(me, lineup, SPD_MOVE, stop_r=30)
            w = _face_angle(me, lineup_ang)
            if _at(me, lineup, ATK_ARRIVE_R):
                state = "ATK_CHARGE"
                print("[coop yellow] lined up — CHARGING through ball")

        elif state == "ATK_CHARGE":
            # Compute charge target dynamically
            ball_pos = ball if ball is not None else BALL_TRIGGER
            sup_pos = (mate[0], mate[1]) if mate is not None else SUP_START
            _, _, charge_tgt = _compute_lineup_and_charge(ball_pos, sup_pos)

            rel_tgt = world2robot(me, charge_tgt)
            vx, vy = move_toward(rel_tgt, SPD_SPRINT,
                                 ramp_dist=50, stop_dist=0)
            w = _face(me, charge_tgt)
            # Stop once we've cleared the ball trigger area
            if _dist(me, BALL_TRIGGER) > 500:
                state = "ATK_DONE"
                print("[coop yellow] pass delivered — stopping")

        elif state == "ATK_DONE":
            vx, vy, w = 0.0, 0.0, 0.0

        # ══════════════════════════════════════════════════════
        #  SUPPORT STATE MACHINE  (blue)
        # ══════════════════════════════════════════════════════

        elif state == "SUP_GO_TO_MARK":
            vx, vy = _go_to(me, SUP_START, SPD_MOVE)
            w = _face_angle(me, SUP_START_ANG)
            if _at(me, SUP_START, 120):
                state = "SUP_WAIT"
                print("[coop blue] at receive mark — waiting for pass")

        elif state == "SUP_WAIT":
            vx, vy = 0.0, 0.0
            w = _face(me, BALL_TRIGGER)
            # Wait until ball arrives near us
            if ball is not None and _dist(ball, me) < SUP_RECEIVE_R:
                state = "SUP_COLLECT"
                print("[coop blue] ball arriving — collecting")

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
                state = "SUP_DRIBBLE_TO_GOAL"
                print("[coop blue] collected — dribbling to shooting spot")

        elif state == "SUP_DRIBBLE_TO_GOAL":
            dribble = 1
            # Dynamically pick goal aim based on goalie position
            goal_aim = _pick_goal_aim(frame, is_yellow)
            vx, vy = _go_to(me, SUP_SHOOT_SPOT, SPD_DRIBBLE, stop_r=30)
            w = _face(me, goal_aim)
            if _at(me, SUP_SHOOT_SPOT, SUP_SHOOT_R):
                state = "SUP_SHOOT"
                shoot_spot_since = now
                print("[coop blue] at shooting spot — aligning to goal")

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
                print("[coop blue] SHOT TAKEN!")

        elif state == "SUP_DONE":
            vx, vy, w = 0.0, 0.0, 0.0

        # ── Obstacle avoidance (only while moving) ───────────
        if state not in ("ATK_WAIT_BALL", "ATK_DONE",
                         "SUP_WAIT", "SUP_DONE"):
            target_for_avoid = ball if ball is not None else BALL_TRIGGER
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, _, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)
            vx += avoid_vx
            vy += avoid_vy

        # Cap speed
        spd = math.hypot(vx, vy)
        max_spd = SPD_SPRINT if (kick or state == "ATK_CHARGE") else SPD_MOVE * 1.2
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
