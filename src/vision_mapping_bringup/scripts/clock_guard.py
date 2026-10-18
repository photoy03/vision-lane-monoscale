#!/usr/bin/env python3
"""Stop the whole pipeline when the ROS clock jumps back (a bag played again, sim time reset).

Only the map nodes can start over on a new time line; the tracker, odometry and
pose stabilizer carry their estimate across and reject or jump on the new time
line. So this node exits instead, and the launch's process-exit guard shuts
everything down; whoever started the pipeline (tools/run/supervise_launch.sh,
systemd, the operator) starts it again clean. Normal runs never get here.

The exit code does not survive `ros2 launch` (it exits 0 after a guard
shutdown), so under the supervisor the reason is handed over as a file:
$VISION_RESTART_REQUEST_DIR/$VISION_RUN_ID.restart, written atomically and
naming the run it belongs to.
"""
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
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
            request_restart(f'ROS clock jumped back by {-self.jumped:.1f} s')
            raise SystemExit(EXIT_CLOCK_JUMPED_BACK)


def request_restart(reason):
    """Leave the restart request for the supervisor of this run, if there is one."""
    directory, run_id = os.environ.get('VISION_RESTART_REQUEST_DIR'), os.environ.get('VISION_RUN_ID')
    if not directory or not run_id:
        return
    target = Path(directory) / f'{run_id}.restart'
    temporary = target.with_suffix('.restart.tmp')
    temporary.write_text(json.dumps({'run_id': run_id, 'reason': reason, 'wall_time': time.time()}))
    os.replace(temporary, target)  # atomic: the supervisor never reads half a file


def main():
    rclpy.init()
    node = ClockGuard()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as stop:
        code = int(stop.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: the context was shut down from outside
        # (launch stopping everything); a normal stop, not an error.
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
