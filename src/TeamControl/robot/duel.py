"""
1v1 Duel — two robots take turns shooting at each other's goals.

Each robot attacks the opponent's goal and defends its own.
When the ball is on my side or coming at me, I save/intercept then
counter-attack. When I kick, I retreat to defend while the opponent
takes their turn.

Yellow defends -x goal, attacks +x goal.
Blue defends +x goal, attacks -x goal.

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


# =====================================================================
#  LAYOUT
#
#       y
#    +1115 +----------------------------------------------+
#          |                                              |
#   [GOAL] |  [YELLOW]    <-- ball -->      [BLUE]  [GOAL]
#    (-x)  | (-1500,0)                    (1500,0)   (+x)
#          |                                              |
#    -1115 +----------------------------------------------+
#        -2250              0               +2250  x
#
#  Yellow: defends -x goal, shoots at +x goal
#  Blue:   defends +x goal, shoots at -x goal
# =====================================================================

HOME_YELLOW     = (-1500, 0)
HOME_BLUE       = (1500, 0)
BALL_START      = (0, 0)

# -- Tuning ------------------------------------------------------------
CLAIM_DIST      = 2000      # mm — ball within this = I go for it
APPROACH_SPD    = CRUISE_SPEED
SAVE_SPD        = CRUISE_SPEED * 0.9
SETUP_PAUSE     = 2.0
SAVE_X_OFFSET   = 400      # mm — how far in front of goal line to save

# Exported for UI overlay
GOAL_POS_X      = HALF_LEN
GOAL_NEG_X      = -HALF_LEN


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


def _pick_goal_aim(ball, goal_x):
    """Pick aim inside goal mouth — favor the side the ball is on."""
    aim_inward = 1 if goal_x > 0 else -1
    aim_x = goal_x + aim_inward * (GOAL_DEPTH * 0.4)
    # Slight y offset to not hit dead center
    if ball is not None:
        aim_y = max(-GOAL_HW * 0.7, min(GOAL_HW * 0.7, ball[1] * 0.3))
    else:
        aim_y = 0.0
    return (aim_x, aim_y)


def _save_position(me, ball, bvx, bvy, bspeed, our_goal_x):
    """
    Compute where to stand to save a shot.
    Positions on the goal line, predicting where ball will cross.
    """
    goal_sign = 1 if our_goal_x > 0 else -1
    save_x = our_goal_x - goal_sign * SAVE_X_OFFSET

    if ball is None:
        return (save_x, 0.0)

    # If ball is moving toward our goal, predict where it crosses
    if bspeed > 100:
        # Time for ball to reach goal line
        dx_to_goal = our_goal_x - ball[0]
        if abs(bvx) > 50 and (dx_to_goal * bvx > 0):  # ball moving toward goal
            t_cross = dx_to_goal / bvx
            if 0 < t_cross < 3.0:
                pred = predict_ball(ball, (bvx, bvy), t_cross)
                pred_y = max(-GOAL_HW * 0.9, min(GOAL_HW * 0.9, pred[1]))
                return (save_x, pred_y)

    # Default: track ball y position
    track_y = max(-GOAL_HW * 0.8, min(GOAL_HW * 0.8, ball[1]))
    return (save_x, track_y)


# =====================================================================
#  MAIN — one process per robot
#
#  Modes:
#    setup    — teleport, place ball
#    defend   — stay near own goal, save shots, intercept
#    attack   — kick engine drives toward opponent goal
#    retreat  — go back to defend after kicking
# =====================================================================

def run_duel(is_running, dispatch_q, wm, robot_id, opponent_id,
             is_yellow=True, opp_is_yellow=None):
    if opp_is_yellow is None:
        opp_is_yellow = not is_yellow

    home = HOME_YELLOW if is_yellow else HOME_BLUE
    home_ang = 0.0 if is_yellow else math.pi

    # Yellow defends -x, attacks +x.  Blue is opposite.
    our_goal_x = -HALF_LEN if is_yellow else HALF_LEN
    opp_goal_x = HALF_LEN if is_yellow else -HALF_LEN

    sim = None
    try:
        sim = grSimSender("127.0.0.1", 20011)
    except Exception as e:
        print(f"[duel] grSim sender failed: {e}")

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
            print(f"[duel] teleport failed: {e}")

    tag = "yellow" if is_yellow else "blue"
    print(f"[duel {tag}] home={home} defend={our_goal_x} attack={opp_goal_x}")

    # -- State ----------------------------------------------------------
    mode = "setup"
    frame = None
    last_ft = 0.0
    prev_obs_pos = {}
    setup_time = time.time()
    ball_placed = False
    shot_count = 0

    ball_history = []
    last_ball_xy = None

    ks = KickState()
    dnav = DiamondNav()

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

        opp = _get_robot(frame, opp_is_yellow, opponent_id)
        opp_pos = (opp[0], opp[1]) if opp is not None else \
                  (HOME_BLUE if is_yellow else HOME_YELLOW)

        # -- Ball velocity ----------------------------------------------
        if ball is not None:
            last_ball_xy = update_ball_history(
                ball_history, now, ball, last_ball_xy)
        bvx, bvy, bspeed = ball_velocity(ball_history)

        # -- Distances --------------------------------------------------
        my_dist = _dist(ball, me[:2]) if ball else float('inf')
        opp_dist = _dist(ball, opp_pos) if ball else float('inf')

        # -- Which half is ball on? ------------------------------------
        # Ball is "on my side" if it's closer to my goal than opponent's
        ball_on_my_side = False
        if ball is not None:
            my_goal_dist = abs(ball[0] - our_goal_x)
            opp_goal_dist = abs(ball[0] - opp_goal_x)
            ball_on_my_side = my_goal_dist < opp_goal_dist

        # -- Is ball heading toward my goal? ---------------------------
        ball_toward_me = False
        if ball is not None and bspeed > 100:
            goal_sign = 1 if our_goal_x > 0 else -1
            ball_toward_me = (bvx * goal_sign) > 80

        # -- Aim target for shooting -----------------------------------
        aim = _pick_goal_aim(ball, opp_goal_x)

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ==============================================================
        #  SETUP
        # ==============================================================
        if mode == "setup":
            vx, vy = _go_to(me, home, APPROACH_SPD)
            w = _face(me, opp_pos)
            if _at(me, home, 200) and (now - setup_time) > SETUP_PAUSE:
                if is_yellow and not ball_placed and sim:
                    try:
                        pkt = grSimPacketFactory.ball_replacement_command(
                            x=BALL_START[0] / 1000.0,
                            y=BALL_START[1] / 1000.0,
                            vx=0.0, vy=0.0)
                        sim.send_packet(pkt)
                        print(f"[duel yellow] ball placed at {BALL_START}")
                    except Exception:
                        pass
                    ball_placed = True
                mode = "defend"
                print(f"[duel {tag}] ready — defending")

        # ==============================================================
        #  DEFEND — position to save, intercept incoming balls
        #
        #  When ball is on my side or heading toward me:
        #    - Position between ball and goal to save
        #    - Intercept fast balls
        #    - When I get the ball, switch to attack
        # ==============================================================
        elif mode == "defend":
            if ball is not None:
                # Save position: between ball and our goal
                save_pt = _save_position(me, ball, bvx, bvy, bspeed,
                                         our_goal_x)

                # Ball coming fast toward my goal — move to intercept
                if ball_toward_me and bspeed > 200:
                    t_arrive = max(my_dist / max(bspeed, 1.0), 0.1)
                    intercept = predict_ball(
                        ball, (bvx, bvy), min(t_arrive, 2.0))
                    # Clamp intercept to stay near our goal
                    goal_sign = 1 if our_goal_x > 0 else -1
                    max_x = our_goal_x - goal_sign * 800
                    if goal_sign > 0:
                        ix = max(max_x, intercept[0])
                    else:
                        ix = min(max_x, intercept[0])
                    iy = max(-HALF_WID + 100, min(HALF_WID - 100, intercept[1]))
                    dribble = 1
                    vx, vy = _go_to(me, (ix, iy), SAVE_SPD,
                                    stop_r=30, ramp=500)
                    w = _face(me, ball)
                else:
                    # Track ball from save position via diamond planner
                    wp = dnav.next_waypoint(frame, is_yellow, robot_id,
                                            me, save_pt)
                    nav_target = wp if wp is not None else save_pt
                    vx, vy = _go_to(me, nav_target, SAVE_SPD, stop_r=50)
                    w = _face(me, ball)

                if my_dist < 600:
                    dribble = 1

                # I have the ball or it's close and loose — attack!
                if my_dist < CLAIM_DIST and my_dist < opp_dist - 100:
                    if not ball_toward_me or my_dist < 400:
                        mode = "attack"
                        ks.reset()
                        print(f"[duel {tag}] got the ball — attacking!")
            else:
                vx, vy = _go_to(me, home, APPROACH_SPD, stop_r=80)
                w = _face(me, opp_pos)

        # ==============================================================
        #  ATTACK — kick engine aims at opponent's goal
        # ==============================================================
        elif mode == "attack":
            if ball is None:
                mode = "defend"
                time.sleep(LOOP_RATE)
                continue

            # Lost possession — opponent got it or ball went far
            if opp_dist < my_dist - 200 and my_dist > 500:
                mode = "retreat"
                ks.reset()
                print(f"[duel {tag}] lost ball — retreating to defend")
                time.sleep(LOOP_RATE)
                continue

            # Ball heading toward my goal — abandon attack, defend
            if ball_toward_me and bspeed > 300 and my_dist > 600:
                mode = "defend"
                ks.reset()
                print(f"[duel {tag}] ball heading to my goal — defending!")
                time.sleep(LOOP_RATE)
                continue

            rel_ball = world2robot(me, ball)
            d_ball = math.hypot(rel_ball[0], rel_ball[1])

            # Intercept fast incoming ball
            if bspeed > 200 and not ks.bursting:
                dx = me[0] - ball[0]
                dy = me[1] - ball[1]
                dd = max(math.hypot(dx, dy), 1.0)
                approach_val = (bvx * dx + bvy * dy) / dd
                if approach_val > 150 and d_ball > KICK_RANGE:
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
                        shot_count += 1
                        print(f"[duel {tag}] SHOOTING #{shot_count}!")
                    if kr.burst_done:
                        mode = "retreat"
                        ks.reset()
                        print(f"[duel {tag}] shot done — retreating to defend")
            else:
                kr = kick_tick(ks, me, ball, aim, now, rel_ball, d_ball)
                vx, vy, w = kr.vx, kr.vy, kr.w
                kick, dribble = kr.kick, kr.dribble
                if kr.kick_started:
                    shot_count += 1
                    print(f"[duel {tag}] SHOOTING #{shot_count}!")
                if kr.burst_done:
                    mode = "retreat"
                    ks.reset()
                    print(f"[duel {tag}] shot done — retreating to defend")

        # ==============================================================
        #  RETREAT — go back to defend after shooting
        # ==============================================================
        elif mode == "retreat":
            # Ball coming back — go defend immediately
            if ball_toward_me and bspeed > 150:
                mode = "defend"
                ks.reset()
                dnav.clear()
                print(f"[duel {tag}] ball incoming — defending!")
                time.sleep(LOOP_RATE)
                continue

            # Ball near me and loose — attack again
            if ball is not None and my_dist < 500 and opp_dist > my_dist + 200:
                mode = "attack"
                ks.reset()
                dnav.clear()
                print(f"[duel {tag}] ball came back — attacking!")
                time.sleep(LOOP_RATE)
                continue

            # Use diamond planner to navigate home around obstacles
            wp = dnav.next_waypoint(frame, is_yellow, robot_id, me, home)
            nav_target = wp if wp is not None else home
            vx, vy = _go_to(me, nav_target, APPROACH_SPD)
            w = _face(me, ball if ball is not None else opp_pos)

            if _at(me, home, 200):
                mode = "defend"
                ks.reset()
                print(f"[duel {tag}] home — defending")

        # -- Obstacle avoidance (all modes except kick burst) -----------
        if not ks.bursting:
            if mode == "defend":
                target_for_avoid = ball if ball is not None else (our_goal_x, 0)
            elif mode == "attack":
                target_for_avoid = ball if ball is not None else aim
            else:
                target_for_avoid = home
            rel_target = world2robot(me, target_for_avoid)
            avoid_vx, avoid_vy, closest_obs, prev_obs_pos = _compute_avoidance(
                me, frame, is_yellow, robot_id, rel_target, prev_obs_pos)

            if dribble == 1:
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
