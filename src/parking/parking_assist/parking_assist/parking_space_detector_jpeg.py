#!/usr/bin/env python3
"""The owner's parking_space_detector.py reading the camera's JPEG topic instead of a raw image topic.

parking_space_detector.py (included unchanged) subscribes to sensor_msgs/Image. On the vehicle the
cameras deliver JPEG and the pipeline's nodes decode it themselves (image_input compressed_gray), so no
raw image topic exists and the detector waits forever (found 2026-11-20 with the live launch fed by a
recording). Decoding the 1920x1080 colour image in another process and sending it over costs 186 MB/s
of transport at 30 Hz. Here the same detector runs with one change of input: the newest JPEG is kept as
it arrives and is decoded only when the detector asks for a frame, as it does with raw images
(newest frame only, older ones dropped).

  parking_space_detector_jpeg --jpeg-topic /measurement/front/image_raw/compressed <detector arguments>

Without --jpeg-topic this is the unchanged detector.
"""
import sys
import time

import cv2
import numpy as np

import parking_assist.parking_space_detector as detector


def main() -> int:
    argv = sys.argv
    if "--jpeg-topic" not in argv:
        return detector.main()
    index = argv.index("--jpeg-topic")
    topic = argv[index + 1]
    del argv[index:index + 2]
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    class JpegSource(detector.RosFrameAndPoseSource):
        def __init__(self, args):
            self._jpeg = None
            self._decode_errors = 0
            super().__init__(args)
            self.node.create_subscription(CompressedImage, topic, self._jpeg_callback, qos_profile_sensor_data)
            self.node.get_logger().info(f"front camera JPEG from {topic}, decoded here (newest frame only)")

        def _jpeg_callback(self, message) -> None:
            arrival = time.perf_counter()
            period = None if self.previous_arrival_time is None else (arrival - self.previous_arrival_time) * 1000.0
            self.previous_arrival_time = arrival
            with self.lock:
                self._jpeg = (message.data, arrival, period, self._stamp_seconds(message.header.stamp))

        def get_latest(self):
            with self.lock:
                item, self._jpeg = self._jpeg, None
            if item is None:
                return super().get_latest()               # a raw image, if somebody publishes one
            data, arrival, period, stamp = item
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.ndim != 3:
                self._decode_errors += 1
                if self._decode_errors in (1, 10, 100) or self._decode_errors % 1000 == 0:
                    self.node.get_logger().error(f"JPEG not decodable ({self._decode_errors} so far)")
                return None, None, None, None
            return frame, arrival, period, stamp

    detector.RosFrameAndPoseSource = JpegSource
    return detector.main()


if __name__ == "__main__":
    raise SystemExit(main())
