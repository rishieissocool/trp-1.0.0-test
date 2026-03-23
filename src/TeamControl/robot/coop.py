"""
Cooperative goal scoring — two-robot teamwork.

Only two robots participate: robot_id and teammate_id. Every other robot
on the field (even same colour) is treated as an obstacle — never a pass
target. The carrier kicks to the teammate or shoots at goal. The support
goes to a planned receiving position, stops, faces the ball, and waits
for the pass.  Positions only update when the ball moves significantly.

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
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.robot.diamond_nav import DiamondNav
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


# Starting positions (setup + reset only)
HOME_YELLOW     = (-1500, -500)
HOME_BLUE       = (-1500,  500)
BALL_START      = (-1300,    0)
GOAL_TARGET     = (HALF_LEN, 0)

# -- Decision thresholds -------------------------------------------------
SHOOT_RANGE     = 1800      # mm — shoot if closer to goal than this
PASS_MAX_DIST   = 2500      # mm — max distance for a pass attempt
CARRIER_MARGIN  = 150       # mm — hysteresis for role switching
APPROACH_SPD    = CRUISE_SPEED
SETUP_PAUSE     = 2.0
RESET_PAUSE     = 2.0
CYCLE_TIMEOUT   = 15.0

# -- Support positioning -------------------------------------------------
SUP_ADVANCE_FRAC = 0.4     # fraction of ball-to-goal distance to advance
SUP_ADVANCE_MIN  = 500
SUP_ADVANCE_MAX  = 1500
SUP_LATERAL      = 550     # mm lateral offset from carrier side
LANE_CLEARANCE   = 250     # mm obstacle clearance for pass/shot lane
SUP_REPLAN_DIST  = 400     # mm — only replan support position when ball moves this far
SUP_STOP_DIST    = 80      # mm — stop moving when this close to target position

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


def _support_pos(ball, carrier_pos):
    """Compute a receiving position ahead of ball, laterally offset."""
    bx, by = ball
    remaining = HALF_LEN - bx
    advance = clamp(remaining * SUP_ADVANCE_FRAC,
                    SUP_ADVANCE_MIN, SUP_ADVANCE_MAX)
    target_x = bx + advance

    # Offset to opposite side of carrier to create passing angle
    if carrier_pos[1] >= by:
        target_y = by - SUP_LATERAL
    else:
        target_y = by + SUP_LATERAL

    target_x = clamp(target_x, -HALF_LEN + 300, HALF_LEN - 500)
    target_y = clamp(target_y, -HALF_WID + 200, HALF_WID - 200)
    return (target_x, target_y)


def _get_obstacles(frame, my_yellow, my_id, mate_yellow, mate_id):
    """Positions of every robot except me and my teammate."""
    obs = []
    for is_y in (True, False):
        for rid in range(6):
            if is_y == my_yellow and rid == my_id:
                continue
            if is_y == mate_yellow and rid == mate_id:
                continue
            r = _get_robot(frame, is_y, rid)
            if r is not None:
                obs.append((r[0], r[1]))
    return obs


def _lane_clear(start, end, obstacles, clearance=LANE_CLEARANCE):
    """True if no obstacle is within *clearance* mm of the segment."""
    sx, sy = start[0], start[1]
    ex, ey = end[0], end[1]
    dx, dy = ex - sx, ey - sy
    length = math.hypot(dx, dy)
    if length < 1:
        return True
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    for ox, oy in obstacles:
        rx, ry = ox - sx, oy - sy
        along = rx * ux + ry * uy
        if along < 0 or along > length:
            continue
        perp = abs(rx * nx + ry * ny)
        if perp < clearance:
            return False
    return True


def _pick_goal_aim(me):
    """Aim inside the goal mouth toward the far post."""
    offset = clamp(-me[1] * 0.3, -GOAL_HW * 0.7, GOAL_HW * 0.7)
    return (HALF_LEN, offset)


def _coop_avoidance(me, frame, is_yellow, robot_id, mate_is_yellow,
                     mate_id, rel_target):
    """Obstacle avoidance that skips the teammate — only avoids everyone else.

    Returns (avoid_vx, avoid_vy, closest_obs_dist).
    """
    AVOID_DIST = 550
    AVOID_CRITICAL = 260
    AVOID_STRENGTH = 2.5
    TANGENT_RATIO = 0.3

    avoid_vx, avoid_vy = 0.0, 0.0
    closest_obs = 9999.0

    for color in (True, False):
        for oid in range(16):
            # Skip myself
            if color == is_yellow and oid == robot_id:
                continue
            # Skip my teammate
            if color == mate_is_yellow and oid == mate_id:
                continue
            try:
                other = frame.get_yellow_robots(isYellow=color, robot_id=oid)
                if isinstance(other, int) or other is None:
                    continue
                op = other.position
                ox, oy = float(op[0]), float(op[1])

                rel_obs = world2robot(me, (ox, oy))
                d_obs = math.hypot(rel_obs[0], rel_obs[1])

                if d_obs < closest_obs:
                    closest_obs = d_obs

                if d_obs >= AVOID_DIST:
                    continue
                if d_obs <= AVOID_CRITICAL:
                    strength = AVOID_STRENGTH
                else:
                    t = (AVOID_DIST - d_obs) / (AVOID_DIST - AVOID_CRITICAL)
                    strength = AVOID_STRENGTH * 0.5 * (1.0 - math.cos(math.pi * t))

                if strength > 0.001 and d_obs > 1:
                    inv_d = 1.0 / d_obs
                    rep_x = -rel_obs[0] * inv_d * strength
                    rep_y = -rel_obs[1] * inv_d * strength
                    tang_x = rel_obs[1] * inv_d
                    tang_y = -rel_obs[0] * inv_d
                    if tang_x * rel_target[0] + tang_y * rel_target[1] < 0:
                        tang_x, tang_y = -tang_x, -tang_y
                    avoid_vx += rep_x + tang_x * strength * TANGENT_RATIO
                    avoid_vy += rep_y + tang_y * strength * TANGENT_RATIO
            except Exception:
                continue

    return avoid_vx, avoid_vy, closest_obs


# =====================================================================
#  MAIN
# =====================================================================

def run_coop(is_running, dispatch_q, wm, robot_id, teammate_id,
             is_yellow=True, mate_is_yellow=None, attack_positive=None):
    if mate_is_yellow is None:
        mate_is_yellow = is_yellow

    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = (math.atan2(HOME_BLUE[1] - HOME_YELLOW[1],
                           HOME_BLUE[0] - HOME_YELLOW[0])
                if is_yellow else 0.0)

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
    ks = KickState()
    dnav = DiamondNav()
    dnav.set_exclude((mate_is_yellow, teammate_id))

    # Support position planning — only update when ball moves enough
    sup_planned_pos = None       # the locked-in support position
    sup_last_ball = None         # ball position when we last planned

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

        # -- Goal detection → reset ------------------------------------
        if mode == "play" and ball is not None:
            if ball[0] > HALF_LEN - 80:
                cycle_count += 1
                print(f"[coop {tag}] GOAL! (cycle {cycle_count})")
                mode = "reset"
                reset_time = now
                ks.reset()
                sup_planned_pos = None

        # -- Cycle timeout → reset -------------------------------------
        if mode == "play" and (now - cycle_start) > CYCLE_TIMEOUT:
            print(f"[coop {tag}] timeout — resetting")
            mode = "reset"
            reset_time = now
            ks.reset()
            sup_planned_pos = None

        # -- Role determination (play mode only) ------------------------
        if mode == "play" and ball is not None:
            if ks.bursting:
                pass
            elif my_dist < mate_dist - CARRIER_MARGIN:
                if my_role != "carrier":
                    my_role = "carrier"
                    ks.reset()
                    sup_planned_pos = None
                    dnav.clear()
                    print(f"[coop {tag}] -> carrier")
            elif mate_dist < my_dist - CARRIER_MARGIN:
                if my_role != "support":
                    my_role = "support"
                    ks.reset()
                    dnav.clear()
                    print(f"[coop {tag}] -> support")

        # ==============================================================
        #  SETUP
        # ==============================================================
        if mode == "setup":
            wp = dnav.next_waypoint(frame, is_yellow, robot_id, me, home)
            nav_target = wp if wp is not None else home
            vx, vy = _go_to(me, nav_target, APPROACH_SPD)
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
                dnav.clear()
                print(f"[coop {tag}] play!")

        # ==============================================================
        #  PLAY — carrier / support
        # ==============================================================
        elif mode == "play":
            if ball is None:
                vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                w = _face(me, GOAL_TARGET)  # no ball visible, face goal

            # ----------------------------------------------------------
            #  CARRIER — approach ball, decide shoot / pass / dribble
            #
            #  Only passes to teammate_id. All other robots are obstacles.
            # ----------------------------------------------------------
            elif my_role == "carrier":
                obstacles = _get_obstacles(
                    frame, is_yellow, robot_id,
                    mate_is_yellow, teammate_id)

                rel_ball = world2robot(me, ball)
                d_ball = math.hypot(rel_ball[0], rel_ball[1])
                dist_to_goal = _dist(me[:2], GOAL_TARGET)

                # Is my teammate ahead and reachable?
                mate_ahead = mate_pos[0] > ball[0] + 200
                mate_reachable = (_dist(me[:2], mate_pos) < PASS_MAX_DIST
                                  and _lane_clear(me[:2], mate_pos, obstacles))

                # -- Choose aim: shoot, pass to mate, or dribble -------
                goal_aim = _pick_goal_aim(me)
                if (dist_to_goal < SHOOT_RANGE
                        and _lane_clear(me[:2], goal_aim, obstacles)):
                    aim = goal_aim
                elif mate_ahead and mate_reachable:
                    aim = mate_pos
                else:
                    aim = (min(ball[0] + 1000, HALF_LEN - 100),
                           ball[1] * 0.5)

                # -- Kick engine (with fast-ball intercept) -------------
                use_ke = True
                if bspeed > 200 and not ks.bursting:
                    dx = me[0] - ball[0]
                    dy = me[1] - ball[1]
                    dd = max(math.hypot(dx, dy), 1.0)
                    appr = (bvx * dx + bvy * dy) / dd
                    if appr > 150 and d_ball > KICK_RANGE:
                        t_a = max(dd / max(bspeed, 1.0), 0.1)
                        intercept = predict_ball(
                            ball, (bvx, bvy), min(t_a, 1.5))
                        dribble = 1
                        vx, vy = _go_to(me, intercept, APPROACH_SPD,
                                        stop_r=30, ramp=500)
                        w = _face(me, ball)
                        use_ke = False

                if use_ke:
                    kr = kick_tick(ks, me, ball, aim, now,
                                   rel_ball, d_ball)
                    vx, vy, w = kr.vx, kr.vy, kr.w
                    kick, dribble = kr.kick, kr.dribble
                    if kr.kick_started:
                        if _dist(aim, GOAL_TARGET) < 800:
                            print(f"[coop {tag}] SHOT!")
                        else:
                            print(f"[coop {tag}] PASS to mate!")
                    if kr.burst_done:
                        ks.reset()

            # ----------------------------------------------------------
            #  SUPPORT — go to planned position, stop, face ball, wait
            #
            #  Only recalculates position when ball moves significantly.
            #  Once at position, stands still and watches ball.
            #  If a pass comes, intercept it.
            # ----------------------------------------------------------
            else:
                # -- Compute approach speed of ball toward me ----------
                dx_to_me = me[0] - ball[0]
                dy_to_me = me[1] - ball[1]
                dd_to_me = max(math.hypot(dx_to_me, dy_to_me), 1.0)
                approach_spd = (bvx * dx_to_me + bvy * dy_to_me) / dd_to_me

                ball_coming = bspeed > 150 and approach_spd > 80

                if ball_coming:
                    # -- Ball is coming toward me — intercept it -------
                    t_a = max(dd_to_me / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_a, 2.0))
                    dribble = 1
                    vx, vy = _go_to(me, intercept, APPROACH_SPD * 1.1,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                    sup_planned_pos = None  # replan after receiving

                else:
                    # -- Plan or hold receiving position ----------------
                    # Only replan when ball has moved enough
                    need_replan = (
                        sup_planned_pos is None
                        or sup_last_ball is None
                        or _dist(ball, sup_last_ball) > SUP_REPLAN_DIST
                    )
                    if need_replan:
                        sup_planned_pos = _support_pos(ball, mate_pos)
                        sup_last_ball = ball
                        dnav.clear()

                    d_to_pos = _dist(me[:2], sup_planned_pos)

                    if d_to_pos < SUP_STOP_DIST:
                        # -- At position: stand still, face ball -------
                        vx, vy = 0.0, 0.0
                        w = _face(me, ball)
                    else:
                        # -- Navigate to position via diamond planner --
                        wp = dnav.next_waypoint(
                            frame, is_yellow, robot_id,
                            me, sup_planned_pos)
                        nav_target = wp if wp is not None else sup_planned_pos
                        vx, vy = _go_to(me, nav_target, APPROACH_SPD,
                                        stop_r=SUP_STOP_DIST, ramp=400)
                        w = _face(me, ball)

                if my_dist < 600:
                    dribble = 1

        # ==============================================================
        #  RESET — go home, re-place ball, loop
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
                ks.reset()
                ball_history.clear()
                sup_planned_pos = None
                dnav.clear()
                print(f"[coop {tag}] next cycle")

        # -- Obstacle avoidance (skips teammate) -----------------------
        if not ks.bursting:
            if mode == "play" and ball is not None:
                target_for_avoid = ball
            else:
                target_for_avoid = home
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, closest_obs = _coop_avoidance(
                me, frame, is_yellow, robot_id,
                mate_is_yellow, teammate_id, rel_target)

            if dribble == 1 and my_role == "carrier":
                avoid_vx *= 0.4
                avoid_vy *= 0.4

            vx += avoid_vx
            vy += avoid_vy

            if closest_obs < 550:
                if closest_obs <= 260:
                    speed_frac = 0.5
                else:
                    t = (550.0 - closest_obs) / (550.0 - 260.0)
                    speed_frac = 1.0 - 0.25 * \
                        (1.0 - math.cos(math.pi * t))
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
