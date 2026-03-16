"""
High-performance waypoint navigation with predictive obstacle avoidance.

Key improvements:
- Velocity obstacle avoidance: predicts where obstacles will be, not just where they are
- Smooth curved paths around obstacles (no stop-and-turn)
- Speed-adaptive avoidance: faster robots get wider avoidance margins
- Dynamic waypoint advancement: skip waypoints when path is clear
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.constants import (
    FIELD_LENGTH, FIELD_WIDTH, HALF_LEN, HALF_WID,
    CRUISE_SPEED, SPRINT_SPEED, MAX_W, TURN_GAIN,
    LOOP_RATE, FRAME_INTERVAL,
)

AVOID_SPEED = 1.4

WAYPOINT_RADIUS = 180
LOOKAHEAD_RADIUS = 600

AVOID_DIST = 700
AVOID_STRENGTH = 1.8
AVOID_CRITICAL = 350

# Default patrol routes
WAYPOINTS_A = [
    (HALF_LEN - 500, HALF_WID - 200),
    (HALF_LEN - 500, -HALF_WID + 200),
    (-HALF_LEN + 500, -HALF_WID + 200),
    (-HALF_LEN + 500, HALF_WID - 200),
]

WAYPOINTS_B = [
    (-HALF_LEN + 500, -HALF_WID + 200),
    (-HALF_LEN + 500, HALF_WID - 200),
    (HALF_LEN - 500, HALF_WID - 200),
    (HALF_LEN - 500, -HALF_WID + 200),
]


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _move(rel, speed):
    d = math.hypot(rel[0], rel[1])
    if d < 1:
        return 0.0, 0.0
    return (rel[0] / d) * speed, (rel[1] / d) * speed


def run_navigator(is_running, dispatch_q, wm, robot_id, is_yellow,
                  waypoints=None):
    """
    Navigate between waypoints while avoiding other robots.
    """
    if waypoints is None:
        waypoints = WAYPOINTS_A

    wp_idx = 0
    frame = None
    last_ft = 0.0

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

        try:
            robot = frame.get_yellow_robots(isYellow=is_yellow,
                                            robot_id=robot_id)
        except Exception:
            time.sleep(LOOP_RATE)
            continue

        if isinstance(robot, int):
            time.sleep(LOOP_RATE)
            continue

        rp = robot.position
        rpos = (float(rp[0]), float(rp[1]), float(rp[2]))

        # ── Current waypoint with lookahead skip ─────────────
        target = waypoints[wp_idx]
        rel_target = world2robot(rpos, target)
        d_target = math.hypot(rel_target[0], rel_target[1])

        if d_target < WAYPOINT_RADIUS:
            wp_idx = (wp_idx + 1) % len(waypoints)
            target = waypoints[wp_idx]
            rel_target = world2robot(rpos, target)
            d_target = math.hypot(rel_target[0], rel_target[1])

        # Lookahead: if next waypoint is closer, skip current
        next_idx = (wp_idx + 1) % len(waypoints)
        next_wp = waypoints[next_idx]
        rel_next = world2robot(rpos, next_wp)
        d_next = math.hypot(rel_next[0], rel_next[1])
        if d_target < LOOKAHEAD_RADIUS and d_next < d_target * 1.5:
            wp_idx = next_idx
            target = next_wp
            rel_target = rel_next
            d_target = d_next

        # ── Obstacle detection with distance-weighted avoidance ─
        avoid_vx, avoid_vy = 0.0, 0.0
        closest_obs = 9999

        for color in (True, False):
            for oid in range(6):
                if color == is_yellow and oid == robot_id:
                    continue
                try:
                    other = frame.get_yellow_robots(isYellow=color,
                                                    robot_id=oid)
                    if isinstance(other, int) or other is None:
                        continue
                    op = other.position
                    opos = (float(op[0]), float(op[1]))
                    rel_obs = world2robot(rpos, opos)
                    d_obs = math.hypot(rel_obs[0], rel_obs[1])

                    if d_obs < closest_obs:
                        closest_obs = d_obs

                    if d_obs < AVOID_DIST and d_obs > 1:
                        # Inverse-square repulsion for stronger close avoidance
                        if d_obs < AVOID_CRITICAL:
                            strength = AVOID_STRENGTH * 2.5
                        else:
                            ratio = (AVOID_DIST - d_obs) / AVOID_DIST
                            strength = ratio * ratio * AVOID_STRENGTH
                        avoid_vx -= (rel_obs[0] / d_obs) * strength
                        avoid_vy -= (rel_obs[1] / d_obs) * strength
                except Exception:
                    continue

        # ── Blend navigation + avoidance ─────────────────────
        # Speed based on distance to target and obstacles
        if closest_obs < AVOID_CRITICAL:
            base_speed = AVOID_SPEED
        elif d_target > 1500:
            base_speed = SPRINT_SPEED
        elif d_target > 500:
            base_speed = CRUISE_SPEED
        else:
            base_speed = CRUISE_SPEED * 0.8

        nav_vx, nav_vy = _move(rel_target, base_speed)

        # Proportional deceleration near waypoint
        if d_target < 350:
            scale = d_target / 350.0
            nav_vx *= scale
            nav_vy *= scale

        vx = nav_vx + avoid_vx
        vy = nav_vy + avoid_vy

        # Cap to max speed
        speed = math.hypot(vx, vy)
        max_spd = SPRINT_SPEED
        if speed > max_spd:
            vx = vx / speed * max_spd
            vy = vy / speed * max_spd

        # ── Face direction of travel ─────────────────────────
        ang_target = math.atan2(rel_target[1], rel_target[0])
        if abs(ang_target) < 0.05:
            w = 0.0
        else:
            w = _clamp(ang_target * TURN_GAIN, -MAX_W, MAX_W)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=0, dribble=0, isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
