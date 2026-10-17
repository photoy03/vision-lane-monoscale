#!/usr/bin/env python3
"""Stop the whole pipeline when the ROS clock jumps back (a bag played again, sim time reset).

Only the map nodes can start over on a new time line; the tracker, odometry and
pose stabilizer carry their estimate across and reject or jump on the new time
line. So this node exits instead, and the launch's process-exit guard shuts
everything down; whoever started the pipeline (tools/run/supervise_launch.sh,
systemd, the operator) starts it again clean. Normal runs never get here.
"""
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.clock import Clock, ClockType, JumpThreshold

EXIT_CLOCK_JUMPED_BACK = 3


class ClockGuard(Node):
    def __init__(self):
        super().__init__('clock_guard')
        self.min_backward_sec = float(self.declare_parameter('min_backward_sec', 1.0).value)
        self.jumped = None
        threshold = JumpThreshold(min_forward=None,
                                  min_backward=Duration(seconds=-self.min_backward_sec),
                                  on_clock_change=False)
        self._handle = self.get_clock().create_jump_callback(threshold, post_callback=self.on_jump)
        # A steady timer: one on ROS time would not fire again until the clock
        # caught up with where it was before the jump.
        self.create_timer(0.2, self.check, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_jump(self, jump):
        self.jumped = jump.delta.nanoseconds * 1e-9

    def check(self):
        if self.jumped is not None:
            self.get_logger().error(
                f'ROS clock jumped back by {-self.jumped:.1f} s: stopping the pipeline for a clean restart')
            raise SystemExit(EXIT_CLOCK_JUMPED_BACK)


def main():
    rclpy.init()
    node = ClockGuard()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as stop:
        code = int(stop.code or 0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
