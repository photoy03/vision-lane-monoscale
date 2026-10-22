#!/usr/bin/env python3
"""Fixed time-slot frame gate for the compressed camera topics (image-input experiment,
2026-10-22).

  python3 tools/run/frame_gate_relay.py --ros-args -p period_sec:=0.2 -p use_sim_time:=true

For every camera topic T (default the two /measurement/*/image_raw/compressed), republishes
on T + "_gated" at most one frame per slot floor(stamp / period_sec): the first frame of a
new slot passes, later ones in the same slot are dropped. The slot grid is the same for
every camera, so front and rear keep frames from the same instants. Publishes with depth 1
(consumers then always take the newest frame). Logs passed / dropped counts per camera.
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrameGate(Node):
    def __init__(self):
        super().__init__("frame_gate_relay")
        topics = self.declare_parameter(
            "topics", ["/measurement/front/image_raw/compressed", "/measurement/rear/image_raw/compressed"]).value
        self.period_ns = int(float(self.declare_parameter("period_sec", 0.2).value) * 1e9)
        if self.period_ns <= 0:
            raise ValueError("period_sec must be positive")
        out_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = {}
        for topic in topics:
            pub = self.create_publisher(CompressedImage, topic + "_gated", out_qos)
            self.state[topic] = {"pub": pub, "slot": None, "passed": 0, "dropped": 0}
            self.create_subscription(CompressedImage, topic, lambda m, t=topic: self.on_frame(t, m),
                                     qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"frame gate: period {self.period_ns / 1e9:.3f} s on {topics}")

    def on_frame(self, topic, message):
        state = self.state[topic]
        slot = (message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec) // self.period_ns
        if state["slot"] is not None and slot <= state["slot"]:
            state["dropped"] += 1
            return
        state["slot"] = slot
        state["passed"] += 1
        state["pub"].publish(message)

    def report(self):
        self.get_logger().info("frame_gate " + " ".join(
            f"{t.split('/')[2]}: passed={s['passed']} dropped={s['dropped']}" for t, s in self.state.items()))


def main():
    rclpy.init()
    node = FrameGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
