"""
Cooperative set-piece — fully scripted pass-and-score play.

Both bots teleport to fixed starting positions on the left side.
The play starts when you place the ball at the trigger spot.

  ATTACKER (yellow):  go to ball → push it toward support → stop
  SUPPORT  (blue):    park at receive spot → collect ball → dribble to goal → shoot

Every position, speed, and threshold is hardcoded.  Nothing is dynamic.
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
# ═══════════════════════════════════════════════════════════════════

# Where the ball must be placed to trigger the play
BALL_TRIGGER    = (-2000, 0)
BALL_TRIGGER_R  = 400        # mm — ball must be within this radius to start

# Goal we're scoring on (right side)
GOAL_X          = HALF_LEN   # +4500
GOAL_AIM        = (GOAL_X, 0)

# ── Attacker (yellow) ────────────────────────────────────────────
ATK_START       = (-2800, -400)   # starting mark — behind and below ball
ATK_START_ANG   = 0.0             # facing right

# Point behind ball, on the line from ball toward support
# so the attacker drives through the ball aimed at the support
ATK_LINEUP      = (-2300, -150)   # get behind ball on the pass line
ATK_LINEUP_ANG  = 0.51            # ~29° — angle toward support spot

# After reaching lineup, the attacker charges straight through the ball
ATK_CHARGE_TGT  = (-500, 1000)    # overshoot past the ball toward support
ATK_CHARGE_SPD  = SPRINT_SPEED    # full speed ram

ATK_ARRIVE_R    = 150             # mm — close enough to lineup point
ATK_PUSH_DONE_R = 350             # mm — stop after passing ball by this much

# ── Support (blue) ───────────────────────────────────────────────
SUP_START       = (-200, 1200)    # receiving mark — ahead and to the side
SUP_START_ANG   = math.pi + 0.5  # ~209° — facing back toward ball origin

SUP_RECEIVE_R   = 500             # mm — ball is "arrived" when this close
SUP_COLLECT_R   = 200             # mm — ball is collected

# After collecting, support dribbles to this shooting spot
SUP_SHOOT_SPOT  = (3000, 400)     # in front of goal, slightly off-center
SUP_SHOOT_R     = 300             # mm — close enough to shooting spot

# Aim inside the goal (slightly off-center for a clean finish)
SUP_GOAL_AIM    = (GOAL_X + GOAL_DEPTH * 0.3, -GOAL_HW * 0.4)
SUP_SHOOT_ALIGN = 0.25            # rad — alignment to fire

# Speeds
SPD_MOVE        = CRUISE_SPEED    # moving to marks
SPD_SLOW        = CRUISE_SPEED * 0.5
SPD_DRIBBLE     = DRIBBLE_SPEED
SPD_CHARGE      = CHARGE_SPEED
SPD_SPRINT      = SPRINT_SPEED

FORCE_SHOOT_T   = 1.5             # seconds — just kick after this long at shoot spot


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
    """Returns (vx, vy) in robot frame to reach target."""
    rel = world2robot(me, target)
    return move_toward(rel, speed, ramp_dist=ramp, stop_dist=stop_r)


def _face(me, target):
    """Returns w to turn toward target."""
    rel = world2robot(me, target)
    ang = math.atan2(rel[1], rel[0])
    return clamp(ang * TURN_GAIN, -MAX_W, MAX_W)


def _face_angle(me, desired_angle):
    """Returns w to turn to a fixed world angle."""
    err = desired_angle - me[2]
    # Normalize to [-pi, pi]
    while err > math.pi:
        err -= 2 * math.pi
    while err < -math.pi:
        err += 2 * math.pi
    return clamp(err * TURN_GAIN, -MAX_W, MAX_W)


def _at(me, target, radius):
    return _dist(me, target) < radius


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

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ══════════════════════════════════════════════════════
        #  ATTACKER STATE MACHINE  (yellow)
        # ══════════════════════════════════════════════════════

        if state == "ATK_GO_TO_MARK":
            # Drive to starting mark
            vx, vy = _go_to(me, ATK_START, SPD_MOVE)
            w = _face_angle(me, ATK_START_ANG)
            if _at(me, ATK_START, 120):
                state = "ATK_WAIT_BALL"
                print(f"[coop yellow] at mark — waiting for ball near {BALL_TRIGGER}")

        elif state == "ATK_WAIT_BALL":
            # Sit at mark, face toward ball trigger spot
            vx, vy = 0.0, 0.0
            w = _face(me, BALL_TRIGGER)
            if ball is not None and _dist(ball, BALL_TRIGGER) < BALL_TRIGGER_R:
                state = "ATK_LINEUP"
                print("[coop yellow] ball detected — lining up behind ball")

        elif state == "ATK_LINEUP":
            # Move to the point behind the ball on the pass line
            vx, vy = _go_to(me, ATK_LINEUP, SPD_MOVE, stop_r=30)
            w = _face_angle(me, ATK_LINEUP_ANG)
            if _at(me, ATK_LINEUP, ATK_ARRIVE_R):
                state = "ATK_CHARGE"
                print("[coop yellow] lined up — CHARGING through ball")

        elif state == "ATK_CHARGE":
            # Full speed ram through ball toward support position
            rel_tgt = world2robot(me, ATK_CHARGE_TGT)
            vx, vy = move_toward(rel_tgt, ATK_CHARGE_SPD,
                                 ramp_dist=50, stop_dist=0)
            w = _face(me, ATK_CHARGE_TGT)
            # Stop once we've passed through the ball area
            if ball is not None and _dist(me, BALL_TRIGGER) > 600:
                state = "ATK_DONE"
                print("[coop yellow] pass delivered — stopping")

        elif state == "ATK_DONE":
            # Stop completely
            vx, vy, w = 0.0, 0.0, 0.0

        # ══════════════════════════════════════════════════════
        #  SUPPORT STATE MACHINE  (blue)
        # ══════════════════════════════════════════════════════

        elif state == "SUP_GO_TO_MARK":
            # Drive to receiving mark
            vx, vy = _go_to(me, SUP_START, SPD_MOVE)
            w = _face_angle(me, SUP_START_ANG)
            if _at(me, SUP_START, 120):
                state = "SUP_WAIT"
                print(f"[coop blue] at receive mark — waiting for pass")

        elif state == "SUP_WAIT":
            # Parked at receiving spot, facing toward ball origin
            vx, vy = 0.0, 0.0
            w = _face(me, BALL_TRIGGER)
            # Transition when ball gets close to us
            if ball is not None and _dist(ball, me) < SUP_RECEIVE_R:
                state = "SUP_COLLECT"
                print("[coop blue] ball arriving — collecting")

        elif state == "SUP_COLLECT":
            # Move toward ball with dribbler to trap it
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
            # Dribble toward the shooting spot in front of goal
            dribble = 1
            vx, vy = _go_to(me, SUP_SHOOT_SPOT, SPD_DRIBBLE, stop_r=30)
            w = _face(me, SUP_GOAL_AIM)
            if _at(me, SUP_SHOOT_SPOT, SUP_SHOOT_R):
                state = "SUP_SHOOT"
                shoot_spot_since = now
                print("[coop blue] at shooting spot — aligning to goal")

        elif state == "SUP_SHOOT":
            # Align to goal and fire
            dribble = 1
            vx, vy = 0.0, 0.0
            rel_aim = world2robot(me, SUP_GOAL_AIM)
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

        # ── Obstacle avoidance ────────────────────────────────
        # Only apply when moving (not when stopped/done)
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
