# from TeamControl.voronoi_planner.planner import VoronoiPlanner
# from TeamControl.voronoi_planner.planner_new import VoronoiPlanner
from TeamControl.voronoi_planner.diamond_planner import DiamondPlanner
from TeamControl.world.model import WorldModel as wm
from TeamControl.robot.Movement import RobotMovement
from TeamControl.network.robot_command import RobotCommand
from TeamControl.voronoi_planner.obstacle import Obstacle
import numpy as np
import time

ROBOT_RADIUS = 90


class PathPlanner():
    def __init__(self, world_model: wm, dispatcher_q, robot_id):
        self.isYellow = True
        self.robot_id = robot_id
        self.version = 0
        self.wm = world_model
        self.timeout = 30
        self.output_q = dispatcher_q
        self.goals = [(0, 0)]
        self.planner = None

    def check_wm_update(self):
        self.isYellow = self.wm.us_yellow() if hasattr(self.wm, "us_yellow") else self.isYellow
        new_version = self.wm.get_version()
        if self.version <= new_version:
            self.version = new_version
            self.frame = self.wm.get_latest_frame()
            return True
        return False

    def running(self):
        robot_id = self.robot_id
        target_pos = 0, 0
        new_target = True
        path_planed = False
        waypoints = [[]]

        while True:
            is_updated = self.check_wm_update()
            if is_updated is True and self.frame is not None:
                robot = self.frame.get_yellow_robots(isYellow=self.isYellow, robot_id=robot_id)
                if isinstance(robot, int) or self.frame.ball is None:
                    continue
                target_pos1 = self.frame.ball.position
                robot_pos = robot.position

                if not self.close_to_point(a=target_pos, b=target_pos1):
                    target_pos = target_pos1
                    self.goals = [target_pos]
                    print("target has changed")
                    new_target = True
                    path_planed = False

                self._rebuild_planner(frame=self.frame)

                if new_target or path_planed is False:
                    new_target = False
                    waypoints = self.pathplanning()
                    print(f"{waypoints[0]=}, {robot_pos=}, {target_pos=}")

                point = waypoints[0][0] if len(waypoints[0]) > 1 else target_pos
                if len(waypoints[0]) > 1:
                    path_planed = True

                if self.close_to_point(a=robot_pos[:2], b=point) and len(waypoints[0]) > 1:
                    waypoints[0].pop(0)

                vx, vy, w = RobotMovement.velocity_to_target(
                    robot_pos=robot_pos, target=point, speed=0.1, stop_threshold=70)
                command = RobotCommand(robot_id, vx, vy, 0, 0, 0)
                runtime = 1
                self.output_q.put((command, runtime))
                time.sleep(0.5)

    def close_to_point(self, a: tuple[float, float], b: tuple[float, float], threshold=150):
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        delta = a - b
        return np.dot(delta, delta) < threshold ** 2

    def _rebuild_planner(self, frame):
        """Rebuild DiamondPlanner from the current frame state."""
        me = frame.get_yellow_robots(isYellow=self.isYellow, robot_id=self.robot_id)
        my_pos = np.array([me.position[0], me.position[1]])

        # Collect all other robots as obstacles
        our_robot_obs = [r.obstacle for r in
                         frame.get_all_in_team_except(isYellow=self.isYellow, exclude=[])]
        enemy_robot_obs = [r.obstacle for r in
                           frame.get_all_in_team_except(isYellow=not self.isYellow, exclude=[])]
        all_obstacles = our_robot_obs + enemy_robot_obs

        self.planner = DiamondPlanner(all_obstacles)

    def pathplanning(self):
        """Plan path for this robot to its goal using DiamondPlanner."""
        if self.planner is None:
            return [list(self.goals)]

        me_frame = self.frame.get_yellow_robots(isYellow=self.isYellow, robot_id=self.robot_id)
        start = np.array([me_frame.position[0], me_frame.position[1]])
        goal = np.array(self.goals[0])

        path = self.planner.plan_path(start, goal)
        # Strip the start position, return only waypoints to follow
        waypoints = [list(path[1:])] if len(path) > 1 else [list(path)]
        return waypoints


def run_planner(world_model: wm, dispatcher_q, robot_id):
    planner = PathPlanner(world_model, dispatcher_q, robot_id)
    planner.running()
