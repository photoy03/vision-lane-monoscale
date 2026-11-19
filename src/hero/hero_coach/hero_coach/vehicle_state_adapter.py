"""CARLA 브리지의 차량 상태 → /hero/vehicle_state.

따로 사는 노드다. HUD 의 "항상 살아 있는" 계층을 계획기나 격자처럼 잘 죽는 것에
묶지 않는다. 속도 보고가 박동이다 — 그것이 올 때마다 한 번 낸다.
"""

from autoware_vehicle_msgs.msg import GearReport, SteeringReport, VelocityReport
from hero_hud.msg import VehicleState
import rclpy
from rclpy.node import Node

from hero_coach.core import UNKNOWN, bicycle_from_front_left, gear_from_autoware
from hero_coach.ros_io import LATEST


class VehicleStateAdapter(Node):
    def __init__(self):
        super().__init__('vehicle_state_adapter')
        declare = self.declare_parameter
        declare('velocity_topic', '/vehicle/status/velocity_status')
        declare('steering_topic', '/vehicle/status/steering_status')
        declare('gear_topic', '/vehicle/status/gear_status')
        declare('output_topic', '/hero/vehicle_state')
        # CARLA Prius 물리 제원(get_physics_control, 2026-09-17).
        declare('wheelbase_m', 2.819)
        declare('front_half_track_m', 0.8145)
        get = lambda name: str(self.get_parameter(name).value)  # noqa: E731
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.half_track = float(self.get_parameter('front_half_track_m').value)

        self.steering = 0.0
        self.gear = UNKNOWN
        self.pub = self.create_publisher(VehicleState, get('output_topic'), LATEST)
        self.create_subscription(SteeringReport, get('steering_topic'), self.on_steering, LATEST)
        self.create_subscription(GearReport, get('gear_topic'), self.on_gear, LATEST)
        self.create_subscription(VelocityReport, get('velocity_topic'), self.on_velocity, LATEST)

    def on_steering(self, msg: SteeringReport):
        # 조향륜 각도, 좌회전 양수 — 조향비 변환은 필요 없지만 FL 바퀴 하나의 각도다.
        self.steering = bicycle_from_front_left(
            msg.steering_tire_angle, self.wheelbase, self.half_track)

    def on_gear(self, msg: GearReport):
        self.gear = gear_from_autoware(msg.report)

    def on_velocity(self, msg: VelocityReport):
        out = VehicleState()
        out.header = msg.header
        out.gear = self.gear
        out.speed_mps = msg.longitudinal_velocity
        out.steering_angle_rad = self.steering
        out.yaw_rate_rps = msg.heading_rate
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VehicleStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
