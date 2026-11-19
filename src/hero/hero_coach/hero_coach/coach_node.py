"""monoscale 격자·오도메트리 + 계획 궤적 → /hero/scene_overlay.

HUD 는 바보다. 시간 동기, 좌표 변환, "지금 어느 구간인가"는 여기서 끝낸다.
발행은 벽시계 10 Hz — HUD 의 장면 타임아웃이 벽시계이기 때문이다.
"""

import time

from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point32
from hero_hud.msg import (
    ObstacleMarker, SceneOverlay, SourceHealth, TrajectoryRibbon, VehicleState,
)
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from hero_coach.core import (
    UNKNOWN, OdometryWatch, SegmentTracker, advised_gear, densify, grid_obstacles,
    ribbon_points, split_segments, threat_of,
)
from hero_coach.ros_io import (
    LATCHED, LATEST, declare_footprint, ego_from_odometry, grid_array, trajectory_waypoints,
)


class CoachNode(Node):
    def __init__(self):
        super().__init__('coach_node')
        declare = self.declare_parameter
        declare('grid_topic', '/perception/occupancy_grid_map')
        declare('odometry_topic', '/localization/kinematic_state')
        declare('trajectory_topic', '/hero_coach/trajectory')
        declare('output_topic', '/hero/scene_overlay')
        declare('rate_hz', 10.0)
        declare('obstacle_threshold', 65)
        declare('obstacle_bin_m', 0.3)
        declare('window_behind_m', 10.0)
        declare('window_ahead_m', 15.0)
        declare('window_lateral_m', 6.0)
        declare('arrive_m', 0.5)
        declare('ribbon_step_m', 0.25)
        declare('vehicle_topic', '/hero/vehicle_state')
        # 정상 주기의 지식은 상류에 있다(SourceHealth.msg). 격자는 타이머 5 Hz,
        # 오도메트리는 누적 시차마다 — 기준은 core.OdometryWatch 가 달린 거리로 정한다.
        declare('occupancy_timeout_sec', 2.0)
        self.p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.footprint = declare_footprint(self)

        self.grid = None
        self.grid_at = None
        self.ego = None
        self.odometry = OdometryWatch()
        self.tracker = None

        self.pub = self.create_publisher(SceneOverlay, str(self.p('output_topic')), LATEST)
        self.create_subscription(OccupancyGrid, str(self.p('grid_topic')), self.on_grid, LATCHED)
        self.create_subscription(Odometry, str(self.p('odometry_topic')), self.on_odometry, LATEST)
        self.create_subscription(
            VehicleState, str(self.p('vehicle_topic')), self.on_vehicle, LATEST)
        self.create_subscription(
            Trajectory, str(self.p('trajectory_topic')), self.on_trajectory, LATCHED)
        self.create_timer(1.0 / float(self.p('rate_hz')), self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_grid(self, msg: OccupancyGrid):
        self.grid = grid_array(msg)
        self.grid_at = time.monotonic()

    def on_odometry(self, msg: Odometry):
        self.ego = ego_from_odometry(msg)
        self.odometry.on_odometry(time.monotonic())

    def on_vehicle(self, msg: VehicleState):
        self.odometry.on_speed(time.monotonic(), msg.speed_mps)

    def on_trajectory(self, msg: Trajectory):
        waypoints = densify(trajectory_waypoints(msg), float(self.p('ribbon_step_m')))
        self.tracker = SegmentTracker(split_segments(waypoints), float(self.p('arrive_m')))

    @staticmethod
    def fresh(at, timeout) -> bool:
        return at is not None and time.monotonic() - at < timeout

    def tick(self):
        out = SceneOverlay()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.advised_gear = UNKNOWN

        now = time.monotonic()
        odometry_ok = self.odometry.ok(now)
        occupancy_ok = self.fresh(self.grid_at, float(self.p('occupancy_timeout_sec')))
        grid_age = -1.0 if self.grid_at is None else now - self.grid_at
        # age -1: 한 번도 안 왔다
        for name, age, ok in (('odometry', self.odometry.age(now), odometry_ok),
                              ('occupancy', grid_age, occupancy_ok)):
            health = SourceHealth()
            health.name = name
            health.age_sec = float(age)
            health.ok = ok
            out.health.append(health)

        if odometry_ok and occupancy_ok:
            self.fill_obstacles(out)
        if odometry_ok and self.tracker is not None:
            self.fill_plan(out)
        self.pub.publish(out)

    def fill_obstacles(self, out: SceneOverlay):
        data, res, origin = self.grid
        obstacles = grid_obstacles(
            data, res, origin, self.ego, self.footprint,
            threshold=int(self.p('obstacle_threshold')),
            x_range=(-float(self.p('window_behind_m')), float(self.p('window_ahead_m'))),
            half_lateral=float(self.p('window_lateral_m')),
            bin_m=float(self.p('obstacle_bin_m')))
        for o in obstacles:
            marker = ObstacleMarker()
            marker.position = Point32(x=o.x, y=o.y, z=0.0)
            marker.extent_m = o.extent
            marker.threat = threat_of(o.distance)
            marker.source = ObstacleMarker.SOURCE_FUSED
            marker.confidence = 1.0
            out.obstacles.append(marker)

    def fill_plan(self, out: SceneOverlay):
        tracker = self.tracker
        tracker.update(self.ego[0], self.ego[1])
        out.advised_gear = advised_gear(tracker)
        segment = tracker.current
        if segment is None:
            return
        points = ribbon_points(segment, tracker.progress, self.ego)
        if len(points) < 2:
            return
        out.has_ideal = True
        out.ideal = TrajectoryRibbon(
            points=[Point32(x=x, y=y, z=0.0) for x, y in points],
            width_m=2.0 * self.footprint.half_width,
            confidence=1.0)


def main():
    rclpy.init()
    node = CoachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
