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
    turn_then_move, ball_velocity, update_ball_history, predict_ball,
)
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.robot.diamond_nav import DiamondNav, get_all_obstacles
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

# -- Decision thresholds -------------------------------------------------
SHOOT_RANGE     = 1800      # mm — shoot if closer to goal than this
PASS_MAX_DIST   = 3500      # mm — max distance for a pass attempt (wider spacing)
CARRIER_MARGIN  = 500       # mm — large hysteresis so roles don't flip-flop
APPROACH_SPD    = CRUISE_SPEED
SETUP_PAUSE     = 2.0
RESET_PAUSE     = 2.0
CYCLE_TIMEOUT   = 15.0

# -- Support positioning — ahead of ball, small lateral offset -----------
SUP_ADVANCE_FRAC = 0.55    # fraction of ball-to-goal distance to advance
SUP_ADVANCE_MIN  = 1200    # minimum forward distance from ball
SUP_ADVANCE_MAX  = 2200    # maximum forward distance from ball
SUP_LATERAL      = 200     # mm — minimal lateral offset, pass goes lengthways
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
    """Support position: directly ahead of ball along +x.

    The carrier kicks straight forward, so support stands in the
    path of the ball, far ahead, ready to receive and score.
    """
    bx, by = ball
    remaining = HALF_LEN - bx
    advance = clamp(remaining * SUP_ADVANCE_FRAC,
                    SUP_ADVANCE_MIN, SUP_ADVANCE_MAX)
    target_x = bx + advance

    # Almost no lateral offset — stay in the ball's forward path
    # Tiny offset just so support isn't perfectly blocking carrier's shot line
    target_y = by * 0.3  # drift toward centre, stay near ball's y

    target_x = clamp(target_x, -HALF_LEN + 300, HALF_LEN - 400)
    target_y = clamp(target_y, -HALF_WID + 300, HALF_WID - 300)
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


# -- Obstacle safety constants -----------------------------------------
OBS_EMERGENCY_DIST  = 350   # mm — emergency repulsion kicks in here
OBS_EMERGENCY_STR   = 6.0   # strong push — last-resort collision prevention
OBS_SLOWDOWN_DIST   = 600   # mm — start slowing down at this distance
OBS_SLOWDOWN_MIN    = 0.15  # minimum speed factor when very close


def _emergency_avoidance(me, obstacles_list):
    """Reactive repulsion from ALL nearby obstacles (robots).

    This is a safety net — the path planner should avoid obstacles, but
    if the robot overshoots or drifts too close, this pushes it away.

    Args:
        me:             (x, y, orientation) robot pose
        obstacles_list: list of (cx, cy, radius) from get_all_obstacles

    Returns:
        (avoid_vx, avoid_vy) repulsion velocity in robot frame.
    """
    total_vx, total_vy = 0.0, 0.0
    for ox, oy, orad in obstacles_list:
        rel = world2robot(me, (ox, oy))
        d = math.hypot(rel[0], rel[1])
        safe_r = orad + OBS_EMERGENCY_DIST
        if d >= safe_r or d < 1:
            continue
        # Strength ramps up as we get closer
        t = (safe_r - d) / safe_r  # 0 at edge, 1 at centre
        strength = OBS_EMERGENCY_STR * t * t  # quadratic — much stronger close up
        inv_d = 1.0 / d
        total_vx -= rel[0] * inv_d * strength
        total_vy -= rel[1] * inv_d * strength
    return total_vx, total_vy


def _obstacle_speed_factor(me, obstacles_list):
    """Compute a speed multiplier [OBS_SLOWDOWN_MIN, 1.0] based on
    proximity to the closest obstacle.  Robots slow down when near
    obstacles so they don't overshoot waypoints into them."""
    min_gap = float('inf')
    for ox, oy, orad in obstacles_list:
        d = math.hypot(me[0] - ox, me[1] - oy)
        gap = d - orad  # distance from obstacle edge
        if gap < min_gap:
            min_gap = gap
    if min_gap >= OBS_SLOWDOWN_DIST:
        return 1.0
    if min_gap <= 0:
        return OBS_SLOWDOWN_MIN
    t = min_gap / OBS_SLOWDOWN_DIST  # 0..1
    return OBS_SLOWDOWN_MIN + (1.0 - OBS_SLOWDOWN_MIN) * t


def _teammate_avoidance(me, frame, mate_is_yellow, mate_id):
    """Repulsion from teammate only — static obstacles handled by path planner.

    Returns (avoid_vx, avoid_vy).
    """
    MATE_AVOID_DIST = 700       # start pushing away at this distance
    MATE_AVOID_CRITICAL = 300   # max strength below this
    MATE_AVOID_STRENGTH = 4.0   # strong repulsion — never collide

    try:
        other = frame.get_yellow_robots(isYellow=mate_is_yellow,
                                         robot_id=mate_id)
        if isinstance(other, int) or other is None:
            return 0.0, 0.0
        op = other.position
        rel = world2robot(me, (float(op[0]), float(op[1])))
        d = math.hypot(rel[0], rel[1])

        if d >= MATE_AVOID_DIST or d < 1:
            return 0.0, 0.0

        if d <= MATE_AVOID_CRITICAL:
            strength = MATE_AVOID_STRENGTH
        else:
            t = (MATE_AVOID_DIST - d) / (MATE_AVOID_DIST - MATE_AVOID_CRITICAL)
            strength = MATE_AVOID_STRENGTH * 0.5 * (1.0 - math.cos(math.pi * t))

        inv_d = 1.0 / d
        return -rel[0] * inv_d * strength, -rel[1] * inv_d * strength
    except Exception:
        return 0.0, 0.0


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

    # Pass tracking — must complete at least 1 pass before shooting
    pass_count = 0               # passes completed this cycle
    prev_role = my_role          # detect role switches (= pass received)

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
                print(f"[coop {tag}] GOAL! (cycle {cycle_count}, passes={pass_count})")
                mode = "reset"
                reset_time = now
                ks.reset()
                sup_planned_pos = None
                pass_count = 0

        # -- Cycle timeout → reset -------------------------------------
        if mode == "play" and (now - cycle_start) > CYCLE_TIMEOUT:
            print(f"[coop {tag}] timeout — resetting")
            mode = "reset"
            reset_time = now
            ks.reset()
            sup_planned_pos = None
            pass_count = 0

        # -- Role determination (play mode only) ------------------------
        if mode == "play" and ball is not None:
            if ks.bursting:
                pass
            elif my_dist < mate_dist - CARRIER_MARGIN:
                if my_role != "carrier":
                    # Was support, now carrier = I received a pass
                    if prev_role == "support":
                        pass_count += 1
                        print(f"[coop {tag}] pass received! (count={pass_count})")
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
            prev_role = my_role

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
            #  CARRIER — get behind ball, kick it FORWARD (+x)
            #
            #  1. Use diamond nav to reach ball area (avoids obstacles)
            #  2. Once close and clear, kick engine lines up behind
            #     ball and kicks toward aim (straight +x)
            # ----------------------------------------------------------
            elif my_role == "carrier":
                obstacles = _get_obstacles(
                    frame, is_yellow, robot_id,
                    mate_is_yellow, teammate_id)

                rel_ball = world2robot(me, ball)
                d_ball = math.hypot(rel_ball[0], rel_ball[1])
                dist_to_goal = _dist(me[:2], GOAL_TARGET)

                # -- Aim: always straight forward (+x) ----------------
                # Pass aim = point far ahead along +x, tiny y bias
                # This makes carrier approach from behind (-x side)
                # and kick the ball straight forward every time.
                goal_aim = _pick_goal_aim(me)
                can_shoot = (pass_count >= 1
                             and dist_to_goal < SHOOT_RANGE
                             and _lane_clear(me[:2], goal_aim, obstacles))

                if can_shoot:
                    aim = goal_aim
                elif pass_count < 1:
                    # Must pass first — aim toward teammate, not goal
                    aim = (mate_pos[0], mate_pos[1] * 0.5)
                else:
                    # Already passed, but can't shoot yet — aim forward
                    aim = (ball[0] + 2000, ball[1] * 0.1)

                # -- Get to ball safely via diamond nav ---------------
                wp = dnav.next_waypoint(
                    frame, is_yellow, robot_id, me, ball)
                path_clear = (wp is None)

                if not path_clear and d_ball > BALL_NEAR and not ks.bursting:
                    # Far from ball and obstacles in the way — navigate
                    vx, vy = _go_to(me, wp, APPROACH_SPD,
                                    stop_r=40, ramp=400)
                    w = _face(me, ball)
                else:
                    # Close to ball or path clear — kick engine
                    kr = kick_tick(ks, me, ball, aim, now,
                                   rel_ball, d_ball)
                    vx, vy, w = kr.vx, kr.vy, kr.w
                    kick, dribble = kr.kick, kr.dribble
                    if kr.kick_started:
                        if can_shoot:
                            print(f"[coop {tag}] SHOT!")
                        else:
                            print(f"[coop {tag}] PASS forward!")
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

                # Only intercept when ball is clearly kicked toward me
                ball_coming = (bspeed > 350 and approach_spd > 200
                               and dd_to_me < 2500)

                if ball_coming:
                    # -- Ball is clearly passed to me — intercept it ---
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

                # Only dribble if ball is literally touching me
                if my_dist < 150:
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
                prev_role = my_role
                pass_count = 0
                ks.reset()
                ball_history.clear()
                sup_planned_pos = None
                dnav.clear()
                print(f"[coop {tag}] next cycle")

        # -- Collect all obstacles for safety systems -------------------
        all_obs = get_all_obstacles(
            frame, is_yellow, robot_id,
            {(mate_is_yellow, teammate_id)})

        # -- Teammate avoidance -----------------------------------------
        if not ks.bursting:
            avoid_vx, avoid_vy = _teammate_avoidance(
                me, frame, mate_is_yellow, teammate_id)
            vx += avoid_vx
            vy += avoid_vy

        # -- Emergency obstacle avoidance (safety net for ALL robots) ---
        if not ks.bursting:
            em_vx, em_vy = _emergency_avoidance(me, all_obs)
            vx += em_vx
            vy += em_vy

        # -- Proximity-based speed reduction (prevent overshooting) -----
        obs_factor = _obstacle_speed_factor(me, all_obs)
        vx *= obs_factor
        vy *= obs_factor

        # -- Speed cap --------------------------------------------------
        spd = math.hypot(vx, vy)
        max_spd = CHARGE_SPEED if kick else APPROACH_SPD * 1.2
        if spd > max_spd:
            vx = vx / spd * max_spd
            vy = vy / spd * max_spd

        # -- Turn-then-move: slow down when facing away from target ------
        if ball is not None and not ks.bursting:
            rel_face = world2robot(me, ball)
            ang_err = abs(math.atan2(rel_face[1], rel_face[0]))
            vx, vy = turn_then_move(vx, vy, w, ang_err)

        # -- Wall braking -----------------------------------------------
        vx, vy = wall_brake(me[0], me[1], vx, vy)

        # -- Rotation compensation --------------------------------------
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
