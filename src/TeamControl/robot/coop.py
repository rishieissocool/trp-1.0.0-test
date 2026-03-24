"""
Cooperative goal scoring — two-robot teamwork.

Yellow robot is ALWAYS the carrier first. It goes to the ball, gets
possession, then passes to the blue robot (support). Support waits at a
fixed receiving point ahead of the ball. After receiving, support becomes
carrier and scores.

Both robots attack the +x goal.
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


# Starting positions — carrier behind ball, support ahead
HOME_YELLOW     = (-1800, -200)    # carrier starts behind ball
HOME_BLUE       = ( -400,  300)    # support starts ahead of ball
BALL_START      = (-1300,    0)
GOAL_TARGET     = (HALF_LEN, 0)

# -- Support receiving position (fixed, well ahead of ball) ---------------
SUP_RECEIVE_X   = 1200      # mm — far ahead so there's a real pass distance
SUP_RECEIVE_Y   = 200       # mm — slight y offset so not blocking shot line

# -- Decision thresholds -------------------------------------------------
SHOOT_RANGE     = 1800      # mm — shoot if closer to goal than this
CARRIER_MARGIN  = 500       # mm — large hysteresis so roles don't flip-flop
APPROACH_SPD    = CRUISE_SPEED
SETUP_PAUSE     = 2.0
RESET_PAUSE     = 2.0
CYCLE_TIMEOUT   = 20.0
SUP_STOP_DIST   = 100       # mm — stop moving when this close to target

# Exported for UI overlay
ATK_START       = HOME_YELLOW
SUP_START       = HOME_BLUE
BALL_TRIGGER    = BALL_START
BALL_TRIGGER_R  = 1800
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


def _pick_goal_aim(me):
    """Aim inside the goal mouth toward the far post."""
    offset = clamp(-me[1] * 0.3, -GOAL_HW * 0.7, GOAL_HW * 0.7)
    return (HALF_LEN, offset)


def _get_static_obstacles(frame, is_yellow, robot_id, mate_is_yellow, mate_id):
    """Get (x, y) positions of all real robots except self and teammate."""
    obs = []
    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            if color == mate_is_yellow and oid == mate_id:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                if hasattr(other, 'confidence') and other.confidence < 0.1:
                    continue
                op = other.position
                ox, oy = float(op[0]), float(op[1])
                if abs(ox) < 1 and abs(oy) < 1:
                    continue
                obs.append((ox, oy))
            except Exception:
                continue
    return obs


def _line_blocked(start, end, obstacles, clearance=250):
    """True if any obstacle is within clearance of the line segment."""
    sx, sy = start
    ex, ey = end
    dx, dy = ex - sx, ey - sy
    length = math.hypot(dx, dy)
    if length < 1:
        return False
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    for ox, oy in obstacles:
        rx, ry = ox - sx, oy - sy
        along = rx * ux + ry * uy
        if along < 0 or along > length:
            continue
        perp = abs(rx * nx + ry * ny)
        if perp < clearance:
            return True
    return False


def _find_receive_pos(frame, is_yellow, robot_id, mate_is_yellow, mate_id):
    """Find a receiving position that has a clear line to the goal."""
    obs = _get_static_obstacles(frame, is_yellow, robot_id,
                                 mate_is_yellow, mate_id)
    candidates = [
        (1200,  200), (1200, -200), (1200,    0),
        (1000,  350), (1000, -350), (1000,    0),
        ( 800,  400), ( 800, -400), ( 800,  200),
        (1400,  150), (1400, -150),
    ]
    goal = (HALF_LEN, 0)
    for cx, cy in candidates:
        if not _line_blocked((cx, cy), goal, obs, clearance=200):
            return (cx, cy)
    return (SUP_RECEIVE_X, SUP_RECEIVE_Y)


def _avoid_all_robots(me, frame, is_yellow, robot_id, mate_is_yellow, mate_id):
    """Reactive repulsion from EVERY other robot on the field."""
    AVOID_DIST = 400
    AVOID_STRENGTH = 5.0

    total_vx, total_vy = 0.0, 0.0

    for color in (True, False):
        for oid in range(16):
            if color == is_yellow and oid == robot_id:
                continue
            if color == mate_is_yellow and oid == mate_id:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                if hasattr(other, 'confidence') and other.confidence < 0.1:
                    continue
                op = other.position
                ox, oy = float(op[0]), float(op[1])
                if abs(ox) < 1 and abs(oy) < 1:
                    continue

                rel = world2robot(me, (ox, oy))
                d = math.hypot(rel[0], rel[1])
                if d >= AVOID_DIST or d < 1:
                    continue

                t = (AVOID_DIST - d) / AVOID_DIST
                strength = AVOID_STRENGTH * t * t
                inv_d = 1.0 / d
                total_vx -= rel[0] * inv_d * strength
                total_vy -= rel[1] * inv_d * strength
            except Exception:
                continue

    # Also avoid teammate (softer)
    MATE_DIST = 500
    MATE_STR = 2.5
    try:
        other = frame.get_yellow_robots(isYellow=mate_is_yellow,
                                         robot_id=mate_id)
        if not isinstance(other, int) and other is not None:
            op = other.position
            rel = world2robot(me, (float(op[0]), float(op[1])))
            d = math.hypot(rel[0], rel[1])
            if 1 < d < MATE_DIST:
                t = (MATE_DIST - d) / MATE_DIST
                strength = MATE_STR * t
                inv_d = 1.0 / d
                total_vx -= rel[0] * inv_d * strength
                total_vy -= rel[1] * inv_d * strength
    except Exception:
        pass

    return total_vx, total_vy


# =====================================================================
#  MAIN
# =====================================================================

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = 0.0

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
    print(f"[coop {tag}] home={home}")

    # -- State ----------------------------------------------------------
    mode = "setup"
    my_role = "carrier" if is_yellow else "support"
    frame = None
    last_ft = 0.0
    setup_time = time.time()
    ball_placed = False
    cycle_start = time.time()
    cycle_count = 0
    reset_time = 0.0

    ball_history = []
    last_ball_xy = None

    # Kick state — inline, no kick engine
    last_kick_time = 0.0
    bursting = False
    burst_end = 0.0

    # Pass tracking
    pass_count = 0
    prev_role = my_role
    kicked = False

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

        # -- Goal detection -> reset ------------------------------------
        if mode == "play" and ball is not None:
            if ball[0] > HALF_LEN - 80:
                cycle_count += 1
                print(f"[coop {tag}] GOAL! (cycle {cycle_count}, passes={pass_count})")
                mode = "reset"
                reset_time = now
                bursting = False
                pass_count = 0
                kicked = False

        # -- Cycle timeout -> reset -------------------------------------
        if mode == "play" and (now - cycle_start) > CYCLE_TIMEOUT:
            print(f"[coop {tag}] timeout -- resetting")
            mode = "reset"
            reset_time = now
            bursting = False
            pass_count = 0
            kicked = False

        # -- Role determination -----------------------------------------
        if mode == "play" and ball is not None:
            if kicked:
                my_role = "support"
            elif bursting:
                pass
            elif my_dist < mate_dist - CARRIER_MARGIN:
                if my_role != "carrier":
                    if prev_role == "support":
                        pass_count += 1
                        print(f"[coop {tag}] pass received! (count={pass_count})")
                    my_role = "carrier"
                    print(f"[coop {tag}] -> carrier")
            elif mate_dist < my_dist - CARRIER_MARGIN:
                if my_role != "support":
                    my_role = "support"
                    print(f"[coop {tag}] -> support")
            prev_role = my_role

        # ==============================================================
        #  SETUP
        # ==============================================================
        if mode == "setup":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, ball if ball is not None else GOAL_TARGET)
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
                mode = "play"
                cycle_start = now
                print(f"[coop {tag}] play!")

        # ==============================================================
        #  PLAY
        # ==============================================================
        elif mode == "play":
            if ball is None:
                vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                w = _face(me, GOAL_TARGET)

            # ----------------------------------------------------------
            #  CARRIER — go to ball, kick toward aim
            #
            #  Same style as right-click "go to ball & kick":
            #  Far  -> cruise to ball, face ball
            #  Mid  -> slow, dribble, blend facing toward aim
            #  Close -> dribble, rotate to aim, kick when ~aligned
            # ----------------------------------------------------------
            elif my_role == "carrier":
                rel_ball = world2robot(me, ball)
                d_ball = math.hypot(rel_ball[0], rel_ball[1])
                ang_ball = math.atan2(rel_ball[1], rel_ball[0])
                dist_to_goal = _dist(me[:2], GOAL_TARGET)

                # Decide aim
                goal_aim = _pick_goal_aim(me)
                can_shoot = (pass_count >= 1 and dist_to_goal < SHOOT_RANGE)
                aim = goal_aim if can_shoot else (mate_pos[0], mate_pos[1])

                rel_aim = world2robot(me, aim)
                ang_aim = math.atan2(rel_aim[1], rel_aim[0])

                if bursting:
                    # Kick burst — drive forward + kick for a bit
                    kick = 1
                    dribble = 1
                    vx = 0.3
                    vy = 0.0
                    w = clamp(ang_aim * 0.5, -MAX_W, MAX_W)
                    if now > burst_end:
                        bursting = False
                        if not can_shoot:
                            kicked = True
                            my_role = "support"
                            print(f"[coop {tag}] kicked, retreating")

                elif d_ball > 600:
                    # Far — cruise toward ball
                    vx, vy = move_toward(rel_ball, 0.6,
                                         ramp_dist=800, stop_dist=250)
                    w = clamp(ang_ball * 0.6, -MAX_W, MAX_W)

                elif d_ball > 130:
                    # Mid — slow down, dribble, start facing aim
                    dribble = 1
                    vx, vy = move_toward(rel_ball, 0.25,
                                         ramp_dist=400, stop_dist=60)
                    w = clamp(ang_ball * 0.5 + ang_aim * 0.2,
                              -MAX_W, MAX_W)

                else:
                    # Close — dribble, rotate to aim, kick when good enough
                    dribble = 1
                    vx = 0.08
                    vy = clamp(rel_ball[1] * 0.3, -0.06, 0.06)
                    w = clamp(ang_aim * 1.2, -MAX_W, MAX_W)

                    can_kick = (now - last_kick_time) > KICK_COOLDOWN
                    aligned = abs(ang_aim) < 0.35   # ~20 deg
                    if can_kick and aligned:
                        kick = 1
                        dribble = 0
                        vx = 0.3
                        vy = 0.0
                        bursting = True
                        burst_end = now + 0.5
                        last_kick_time = now
                        if can_shoot:
                            print(f"[coop {tag}] SHOT!")
                        else:
                            print(f"[coop {tag}] PASS to mate!")

            # ----------------------------------------------------------
            #  SUPPORT
            # ----------------------------------------------------------
            else:
                if kicked:
                    # Already kicked — retreat out of the way
                    retreat_x = ball[0] - 1500
                    retreat_y = -me[1] * 0.6
                    retreat_x = clamp(retreat_x, -HALF_LEN + 300, HALF_LEN - 600)
                    retreat_y = clamp(retreat_y, -HALF_WID + 300, HALF_WID - 300)
                    retreat_pos = (retreat_x, retreat_y)

                    d_to_retreat = _dist(me[:2], retreat_pos)
                    if d_to_retreat < 150:
                        vx, vy = 0.0, 0.0
                        w = _face(me, ball)
                    else:
                        vx, vy = _go_to(me, retreat_pos, CRUISE_SPEED,
                                        stop_r=100, ramp=400)
                        w = _face(me, retreat_pos)

                else:
                    # Waiting for pass
                    dx_to_me = me[0] - ball[0]
                    dy_to_me = me[1] - ball[1]
                    dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                    approach_spd = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me

                    ball_coming = (bspeed > 300 and approach_spd > 150
                                   and dd_to_me < 3000)

                    if ball_coming:
                        # Ball passed to me — intercept
                        t_a = max(dd_to_me / max(bspeed, 1.0), 0.1)
                        intercept = predict_ball(
                            ball, (bvx, bvy), min(t_a, 2.0))
                        dribble = 1
                        vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                        stop_r=30, ramp=500)
                        w = _face(me, ball)

                    else:
                        # Go to clear receiving position and wait
                        receive_pos = _find_receive_pos(
                            frame, is_yellow, robot_id,
                            mate_is_yellow, teammate_id)
                        d_to_pos = _dist(me[:2], receive_pos)

                        if d_to_pos < SUP_STOP_DIST:
                            vx, vy = 0.0, 0.0
                            w = _face(me, ball)
                            if my_dist < 200:
                                dribble = 1
                        else:
                            vx, vy = _go_to(me, receive_pos, APPROACH_SPD,
                                            stop_r=SUP_STOP_DIST, ramp=400)
                            w = _face(me, ball)

        # ==============================================================
        #  RESET
        # ==============================================================
        elif mode == "reset":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, ball if ball is not None else GOAL_TARGET)

            if _at(me, home, 250) and (now - reset_time) > RESET_PAUSE:
                if is_yellow and sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_START[0] / 1000.0,
                            y=BALL_START[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                    except Exception:
                        pass
                mode = "play"
                cycle_start = now
                my_role = "carrier" if is_yellow else "support"
                prev_role = my_role
                pass_count = 0
                kicked = False
                bursting = False
                ball_history.clear()
                print(f"[coop {tag}] next cycle")

        # -- Avoid all robots on field ----------------------------------
        if not bursting:
            avoid_vx, avoid_vy = _avoid_all_robots(
                me, frame, is_yellow, robot_id,
                mate_is_yellow, teammate_id)
            vx += avoid_vx
            vy += avoid_vy

        # -- Speed cap --------------------------------------------------
        spd = math.hypot(vx, vy)
        max_spd = CHARGE_SPEED if kick else APPROACH_SPD
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
