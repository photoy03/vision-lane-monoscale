"""목표 pose 까지의 주차 경로. Autoware freespace A* (Hybrid A* + Reeds-Shepp).

legacy `isaac_autoware_bridge/parking_planner.py` 에서 가져왔다. A* 구성, 파라미터
기본값, 막힘 재계획이 그쪽 것이다. 바뀐 것:

- 자차 pose 를 monoscale 오도메트리에서 읽는다.
- **계획 창을 새로 만든다.** Autoware 는 미지 셀을 장애물로 치는데 카메라 격자는
  대부분이 미지이고 매핑한 창만 발행된다. core.planning_costmap 참고.
- 막힘 검사를 원이 아니라 A* 와 같은 차체 상자로 한다.
- 경로 이탈도 재계획한다. 사람이 운전하므로 벗어나는 것이 정상이다.
- 계획이 실패하면 빈 궤적을 낸다. 낡은 목표선은 없는 것보다 나쁘다.

A* 는 수 초 걸릴 수 있어 coach_node 와 다른 프로세스로 돈다.
"""

import array
import math
import time
from typing import List, Optional

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
import autoware_freespace_planning_algorithms.astar_search as fp
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from hero_coach.core import Waypoint, densify, nearest_index, path_blocked, planning_costmap
from hero_coach.ros_io import (
    LATCHED, LATEST, declare_footprint, ego_from_odometry, grid_array, yaw_of,
)


class ParkingPlanner(Node):
    def __init__(self):
        super().__init__('parking_planner')
        declare = self.declare_parameter
        declare('grid_topic', '/perception/occupancy_grid_map')
        declare('odometry_topic', '/localization/kinematic_state')
        declare('goal_topic', '/goal_pose')
        declare('trajectory_topic', '/hero_coach/trajectory')
        declare('tick_hz', 2.0)
        declare('window_margin_m', 8.0)
        declare('replan_cooldown_sec', 1.0)
        declare('course_out_m', 1.0)
        declare('arrive_m', 0.5)
        declare('check_step_m', 0.25)

        # --- 차량 (A* 충돌 모양) ---
        self.footprint = declare_footprint(self)
        declare('vehicle.wheelbase_m', 2.819)
        # 자전거 등가 최대각. CARLA Prius 안쪽 70° 에서 Ackermann 으로 0.9925 rad (실측).
        declare('vehicle.max_steer_rad', 0.9925)
        declare('vehicle_shape_margin_m', 0.1)

        # --- A* (legacy 기본값) ---
        declare('time_limit_ms', 10000.0)
        declare('max_turning_ratio', 0.8)
        declare('turning_steps', 3)
        declare('theta_size', 60)
        declare('angle_goal_range_deg', 10.0)
        declare('curve_weight', 0.5)
        declare('reverse_weight', 1.0)
        declare('direction_change_weight', 1.0)
        declare('lateral_goal_range_m', 0.5)
        declare('longitudinal_goal_range_m', 2.0)
        declare('obstacle_threshold', 65)
        declare('search_method', 'forward')
        declare('use_back', True)
        declare('only_behind_solutions', False)
        declare('adapt_expansion_distance', True)
        declare('expansion_distance', 1.0)
        declare('near_goal_distance', 3.0)
        declare('distance_heuristic_weight', 1.0)
        declare('smoothness_weight', 0.5)
        declare('obstacle_distance_weight', 1.0)
        declare('goal_lat_distance_weight', 1.0)

        self.p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.astar = self.build_astar()

        self.grid = None
        self.grid_frame = 'map'
        self.ego = None
        self.goal: Optional[PoseStamped] = None
        self.goal_is_new = False
        self.path: List[Waypoint] = []
        self.progress = 0
        self.last_plan = -math.inf
        self.arrived = False

        self.pub = self.create_publisher(Trajectory, str(self.p('trajectory_topic')), LATCHED)
        self.create_subscription(OccupancyGrid, str(self.p('grid_topic')), self.on_grid, LATCHED)
        self.create_subscription(Odometry, str(self.p('odometry_topic')), self.on_odometry, LATEST)
        self.create_subscription(PoseStamped, str(self.p('goal_topic')), self.on_goal, 1)
        # 벽시계. 시뮬레이터가 느려도 재계획 판단은 사람이 보는 시간으로 돈다.
        self.create_timer(1.0 / float(self.p('tick_hz')), self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def build_astar(self):
        fpr = self.footprint
        margin = float(self.p('vehicle_shape_margin_m'))
        shape = fp.VehicleShape(
            float(fpr.front - fpr.rear + 2.0 * margin),
            float(2.0 * fpr.half_width + 2.0 * margin),
            float(self.p('vehicle.wheelbase_m')),
            float(self.p('vehicle.max_steer_rad')),
            float(-fpr.rear + margin),
        )
        common = fp.PlannerCommonParam()
        common.time_limit = float(self.p('time_limit_ms'))
        common.max_turning_ratio = float(self.p('max_turning_ratio'))
        common.turning_steps = int(self.p('turning_steps'))
        common.theta_size = int(self.p('theta_size'))
        common.angle_goal_range = float(self.p('angle_goal_range_deg'))
        common.curve_weight = float(self.p('curve_weight'))
        common.reverse_weight = float(self.p('reverse_weight'))
        common.direction_change_weight = float(self.p('direction_change_weight'))
        common.lateral_goal_range = float(self.p('lateral_goal_range_m'))
        common.longitudinal_goal_range = float(self.p('longitudinal_goal_range_m'))
        common.obstacle_threshold = int(self.p('obstacle_threshold'))

        astar = fp.AstarParam()
        astar.search_method = str(self.p('search_method'))
        astar.only_behind_solutions = bool(self.p('only_behind_solutions'))
        astar.use_back = bool(self.p('use_back'))
        astar.adapt_expansion_distance = bool(self.p('adapt_expansion_distance'))
        astar.expansion_distance = float(self.p('expansion_distance'))
        astar.near_goal_distance = float(self.p('near_goal_distance'))
        astar.distance_heuristic_weight = float(self.p('distance_heuristic_weight'))
        astar.smoothness_weight = float(self.p('smoothness_weight'))
        astar.obstacle_distance_weight = float(self.p('obstacle_distance_weight'))
        astar.goal_lat_distance_weight = float(self.p('goal_lat_distance_weight'))
        return fp.AstarSearch(common, shape, astar)

    # ------------------------------------------------------------ 입력
    def on_grid(self, msg: OccupancyGrid):
        self.grid = grid_array(msg)
        self.grid_frame = msg.header.frame_id or 'map'

    def on_odometry(self, msg: Odometry):
        self.ego = ego_from_odometry(msg)

    def on_goal(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.grid_frame:
            self.get_logger().warn(
                f'goal frame "{msg.header.frame_id}" is not the grid frame '
                f'"{self.grid_frame}"; map and odom are taken as the same')
        self.goal = msg
        self.goal_is_new = True
        self.arrived = False
        # 계획이 나올 때까지 이전 목표의 궤적을 거둔다. 남겨 두면 코치가 이미 끝난
        # 궤적을 보고 새 목표를 받자마자 P 를 권고한다 (CARLA 에서 실제로 그랬다).
        self.path, self.progress = [], 0
        self.publish([], [])
        self.get_logger().info(
            f'goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    # ------------------------------------------------------------ 판단
    def tick(self):
        if self.goal is None or self.grid is None or self.ego is None:
            return
        x, y, _ = self.ego
        goal = self.goal.pose.position
        if math.hypot(goal.x - x, goal.y - y) < float(self.p('arrive_m')):
            if not self.arrived:
                self.get_logger().info('arrived')
            self.arrived = True
        if self.arrived:
            return

        reason = self.replan_reason()
        if reason is None:
            return
        now = time.monotonic()
        if not self.goal_is_new and now - self.last_plan < float(self.p('replan_cooldown_sec')):
            return
        self.last_plan = now
        self.goal_is_new = False
        self.plan(reason)

    def replan_reason(self) -> Optional[str]:
        if self.goal_is_new:
            return 'new goal'
        if not self.path:
            return 'no path'
        x, y, _ = self.ego
        self.progress = nearest_index(self.path, x, y, self.progress)
        here = self.path[self.progress]
        if math.hypot(here.x - x, here.y - y) > float(self.p('course_out_m')):
            return 'course out'
        data, res, origin = self.grid
        if path_blocked(data, res, origin, self.path[self.progress:], self.footprint,
                        int(self.p('obstacle_threshold'))):
            return 'blocked'
        return None

    def plan(self, reason: str):
        data, res, origin = self.grid
        goal = self.goal.pose
        costmap, costmap_origin = planning_costmap(
            data, res, origin, [self.ego[:2], (goal.position.x, goal.position.y)],
            float(self.p('window_margin_m')), ego=self.ego, footprint=self.footprint)

        grid = OccupancyGrid()
        grid.header.frame_id = self.grid_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = res
        grid.info.width = costmap.shape[1]
        grid.info.height = costmap.shape[0]
        grid.info.origin.position.x = costmap_origin[0]
        grid.info.origin.position.y = costmap_origin[1]
        grid.info.origin.orientation.w = 1.0
        grid.data = array.array('b', costmap.tobytes())

        start = Pose()
        start.position.x, start.position.y = self.ego[0], self.ego[1]
        start.orientation.z = math.sin(0.5 * self.ego[2])
        start.orientation.w = math.cos(0.5 * self.ego[2])

        began = time.monotonic()
        try:
            self.astar.setMap(grid)
            ok = self.astar.makePlan(start, goal)
        except RuntimeError as error:
            self.get_logger().error(f'makePlan: {error}')
            ok = False
        elapsed = time.monotonic() - began

        waypoints = []
        if ok:
            planned = self.astar.getWaypoints().waypoints
            poses = [w.pose for w in planned]
            waypoints = [
                Waypoint(p.position.x, p.position.y, yaw_of(p.orientation), bool(w.is_back))
                for w, p in zip(planned, poses)]
        if len(waypoints) < 2:
            self.get_logger().warn(f'{reason}: no plan ({elapsed:.2f} s)')
            self.path, self.progress = [], 0
            self.publish([], [])
            return

        self.path = densify(waypoints, float(self.p('check_step_m')))
        self.progress = 0
        switches = sum(a.is_back != b.is_back for a, b in zip(waypoints, waypoints[1:]))
        self.get_logger().info(
            f'{reason}: {len(waypoints)} waypoints, {switches} direction changes '
            f'({elapsed:.2f} s)')
        self.publish(waypoints, poses)

    def publish(self, waypoints: List[Waypoint], poses: List[Pose]):
        out = Trajectory()
        out.header.frame_id = self.grid_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for w, pose in zip(waypoints, poses):
            point = TrajectoryPoint()
            point.pose = pose
            point.longitudinal_velocity_mps = -1.0 if w.is_back else 1.0
            out.points.append(point)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ParkingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
