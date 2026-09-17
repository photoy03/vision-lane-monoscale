#!/usr/bin/env python3
"""Hardware-free ROS smoke test. Requires the stage-1 workspace to be built.

Launches only its own lane node; never opens a camera or IMU device.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vision_lane_interfaces.msg import MarkingSegmentArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/vision_lane_bringup'))
from vision_lane_bringup.rig import read_calibration, write_opencv_yaml


def main():
    rclpy.init()
    node = rclpy.create_node('stage1_sync_test_driver')
    outputs = {}
    images = node.create_publisher(Image, '/stage1_test/image', 10)
    odoms = node.create_publisher(Odometry, '/stage1_test/odom', 10)
    def key(stamp):
        return stamp.sec * 10**9 + stamp.nanosec
    def receive(message):
        outputs[key(message.header.stamp)] = message
    subscription = node.create_subscription(MarkingSegmentArray, '/stage1_test/segments', receive, 10)
    process = None
    with tempfile.TemporaryDirectory(prefix='stage1_ros_smoke_') as directory:
        tmp = Path(directory)
        k, d, size = read_calibration(ROOT / 'src/vision_lane_bringup/config/fisheye_calibration.npz')
        write_opencv_yaml(tmp / 'camera.yaml', k, d, size)
        params = {
            'input_mode': 'topic', 'image_topic': '/stage1_test/image',
            'odom_topic': '/stage1_test/odom', 'output_topic': '/stage1_test/segments',
            'status_topic': '/stage1_test/status', 'camera_frame': 'test_camera',
            'map_frame': 'test_map', 'base_frame': 'base_link', 'use_static_pose': 'false',
            'sync_images_to_odom': 'true', 'sync_max_wait_sec': '0.08',
            'sync_queue_size': '8', 'startup_warmup_frames': '2',
            'publish_debug_map': 'false', 'preprocess_backend': 'cpu',
            'calibration_yaml': str(tmp / 'camera.yaml'), 'profile_csv_path': str(tmp / 'profile.csv'),
        }
        command = ['ros2', 'run', 'vision_lane_global_mapping_cpp', 'global_mapping_node',
                   '--ros-args', '-r', '__node:=stage1_lane_test']
        for name, value in params.items():
            command += ['-p', f'{name}:={value}']
        with (tmp / 'node.log').open('w+') as log:
            try:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                def wait_for(predicate, seconds=3.):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=.01)
                        if process.poll() is not None:
                            raise RuntimeError('lane process exited')
                        if predicate():
                            return
                    raise AssertionError('timeout waiting for ROS test result')
                wait_for(lambda: images.get_subscription_count() and odoms.get_subscription_count()
                         and node.count_publishers('/stage1_test/segments'), 10.)
                def image():
                    message = Image()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = 'test_camera'
                    message.width, message.height = 320, 180
                    message.encoding, message.step = 'bgr8', 960
                    message.data = bytes(320 * 180 * 3)
                    images.publish(message)
                    return key(message.header.stamp)
                def odom(stamp, x=0., frame='test_map'):
                    message = Odometry()
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                    message.header.frame_id, message.child_frame_id = frame, 'base_link'
                    message.pose.pose.orientation.w = 1.
                    message.pose.pose.position.x = float(x)
                    odoms.publish(message)
                first = image()
                wait_for(lambda: first in outputs)
                assert not outputs[first].pose_valid, 'no odom must not map'
                second = image()
                odom(second, frame='wrong_map')
                wait_for(lambda: second in outputs)
                assert not outputs[second].pose_valid, 'wrong frame must be rejected'
                third = image()
                deadline = time.monotonic() + .02
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.005)
                odom(third, x=1.25)
                wait_for(lambda: third in outputs)
                assert outputs[third].pose_valid
                assert abs(outputs[third].observer_pose.x - 1.25) < 1e-9
                fourth = image()
                odom(fourth - 10_000_000, x=9.)
                wait_for(lambda: fourth in outputs)
                assert not outputs[fourth].pose_valid, 'old-only pose must not extrapolate'
                fifth = image()
                odom(fifth - 10_000_000, x=1.)
                odom(fifth + 10_000_000, x=3.)
                wait_for(lambda: fifth in outputs)
                assert outputs[fifth].pose_valid
                assert abs(outputs[fifth].observer_pose.x - 2.) < 1e-6
                print('PASS: missing odom, wrong frame, delayed exact odom, old-only rejection, interpolation')
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
