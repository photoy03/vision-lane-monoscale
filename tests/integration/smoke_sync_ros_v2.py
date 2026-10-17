#!/usr/bin/env python3
"""Stage-1 transport/synchronization diagnostic, v2. No camera/IMU required.

Run via the stage-1 CPU environment:
  bash tools/run/with_env.sh cpu python3 tests/integration/smoke_sync_ros_v2.py
  bash tools/run/with_env.sh cpu python3 tests/integration/smoke_sync_ros_v2.py --width 320 --height 180

Does NOT modify or rebuild C++, CUDA, RMW configuration or sensor settings.
Small 64x36 BGR probes are the default; the image size is fixed for the entire
child process because the lane node caches its remap. Best-effort image QoS is
explicit (matching SensorDataQoS); reliable->best-effort is also compatible,
so this choice is not a claim that the old QoS pair was incompatible.

Unlike v1: fresh repeated probes before assertions, status subscription,
unique topics, direct executable launch, bounded retries only for absent
results, and logs retained in /tmp/stage1_sync_diag_*/. A semantic mismatch
fails immediately; retries do not turn an incorrect pose into a PASS.
"""
import argparse
from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]


class CheckFailed(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def transport_code(sync, output_count):
    if output_count:
        return 'PARTIAL_IMAGE_DELIVERY'
    if sync is None:
        return 'NO_STATUS_OR_OUTPUT'
    if sync.get('images_received', 0) == 0:
        return 'IMAGE_NOT_RECEIVED'
    return 'IMAGE_RECEIVED_NO_OUTPUT'


def check_pose(message, valid, expected_x=None):
    if bool(message.pose_valid) != valid:
        raise CheckFailed('POSE_ASSERTION',
                          f'expected pose_valid={valid}, got {message.pose_valid}')
    if expected_x is not None:
        x = message.observer_pose.x
        if not math.isfinite(x) or abs(x - expected_x) > 1e-6:
            raise CheckFailed('POSE_ASSERTION', f'expected x={expected_x}, got {x}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--width', type=int, default=64)
    parser.add_argument('--height', type=int, default=36)
    parser.add_argument('--image-qos', choices=('best_effort', 'reliable'), default='best_effort')
    parser.add_argument('--package-calibration', action='store_true',
                        help='Exercise installed package-relative calibration lookup')
    args = parser.parse_args(argv)
    if args.width <= 0 or args.height <= 0 or args.width * args.height > 4_000_000:
        parser.error('image size must be positive and <= 4 million pixels')

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.utilities import get_rmw_implementation_identifier
    from ament_index_python.packages import get_package_prefix
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from vision_lane_interfaces.msg import MarkingSegmentArray

    root = args.workspace.expanduser().resolve()
    sys.path.insert(0, str(root / 'src/vision_mapping_bringup'))
    from vision_mapping_bringup.rig import read_calibration, write_opencv_yaml
    exe = Path(get_package_prefix('lane_detection')) / 'lib/lane_detection/global_mapping_node'
    if not exe.is_file():
        raise RuntimeError(f'Built executable not found: {exe}')
    if exe.resolve() != (root / 'install/lane_detection/lib/lane_detection/global_mapping_node').resolve():
        raise RuntimeError('Wrong workspace overlay: use this workspace tools/run/with_env.sh')

    run = Path(tempfile.mkdtemp(prefix='stage1_sync_diag_'))
    calibration = root / 'src/vision_mapping_bringup/config/calibration/front.npz'
    k, d, size = read_calibration(calibration)
    write_opencv_yaml(run / 'calibration.yaml', k, d, size)
    tag = 'stage1_diag_' + uuid.uuid4().hex[:8]
    topic = '/' + tag
    camera_frame, map_frame = tag + '_camera', tag + '_map'
    rclpy.init(args=[])
    node = rclpy.create_node(tag + '_driver')
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=(ReliabilityPolicy.BEST_EFFORT if args.image_qos == 'best_effort'
                     else ReliabilityPolicy.RELIABLE),
        durability=DurabilityPolicy.VOLATILE)
    images = node.create_publisher(Image, topic + '/image', qos)
    odoms = node.create_publisher(Odometry, topic + '/odom', 10)
    outputs = OrderedDict()
    state = {'sync': None, 'status_messages': 0, 'vector_messages': 0,
             'images_sent': 0, 'odom_sent': 0, 'last_stamp_ns': 0, 'events': []}
    process = None
    log = (run / 'node.log').open('w', encoding='utf-8')
    report = {
        'version': 2, 'workspace': str(root), 'executable': str(exe),
        'image_size': [args.width, args.height], 'image_qos': args.image_qos,
        'rmw': get_rmw_implementation_identifier(),
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '(default)'),
        'ROS_LOCALHOST_ONLY': os.environ.get('ROS_LOCALHOST_ONLY', '(default)'),
        'topics_prefix': topic, 'run_directory': str(run), 'phase': 'discovery',
        'outcome': 'NOT_COMPLETED',
    }

    def stamp_key(stamp):
        return stamp.sec * 10**9 + stamp.nanosec

    def on_output(message):
        state['vector_messages'] += 1
        outputs[stamp_key(message.header.stamp)] = message
        while len(outputs) > 256:
            outputs.popitem(last=False)

    def on_sync(message):
        try:
            state['sync'] = json.loads(message.data)
            state['status_messages'] += 1
        except ValueError:
            pass

    subscriptions = [
        node.create_subscription(MarkingSegmentArray, topic + '/segments', on_output, 10),
        node.create_subscription(String, topic + '/status/sync', on_sync, 10),
    ]

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            if process.poll() is not None:
                raise CheckFailed('NODE_EXITED', f'child exited with code {process.returncode}')
            if predicate():
                return True
        return False

    def settle(seconds=.12):
        spin_until(lambda: False, seconds)

    blank = bytes(args.width * args.height * 3)

    def send_image():
        message = Image()
        stamp = max(node.get_clock().now().nanoseconds, state['last_stamp_ns'] + 1)
        state['last_stamp_ns'] = stamp
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.header.frame_id = camera_frame
        message.width, message.height = args.width, args.height
        message.encoding, message.step = 'bgr8', args.width * 3
        message.data = blank
        images.publish(message)
        state['images_sent'] += 1
        return stamp

    def send_odom(stamp, x=0., frame=None):
        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.header.frame_id = frame if frame is not None else map_frame
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.
        message.pose.pose.position.x = float(x)
        odoms.publish(message)
        state['odom_sent'] += 1

    def case(name, action, valid, expected_x=None):
        report['phase'] = name
        for attempt in range(1, 4):
            settle()
            stamp = send_image()
            action(stamp)
            if spin_until(lambda: stamp in outputs, .65):
                message = outputs[stamp]
                # Do not retry a wrong result: a synchronization bug must fail.
                check_pose(message, valid, expected_x)
                if message.header.frame_id != 'base_link':
                    raise CheckFailed('FRAME_ASSERTION', 'lane output must use base_link')
                state['events'].append({'case': name, 'attempt': attempt,
                                        'stamp_ns': stamp, 'pose_valid': bool(message.pose_valid)})
                print(f'[PASS {name}] attempt={attempt}', flush=True)
                return
            print(f'[RETRY {name}] no matching output; fresh timestamp on next attempt', flush=True)
        raise CheckFailed(transport_code(state['sync'], 0), f'{name}: no output for three fresh images')

    def graph():
        def endpoints(info):
            return [{'node': p.node_name, 'namespace': p.node_namespace,
                     'type': p.topic_type, 'reliability': str(p.qos_profile.reliability),
                     'durability': str(p.qos_profile.durability)} for p in info]
        return {suffix: {
            'publishers': endpoints(node.get_publishers_info_by_topic(topic + suffix)),
            'subscriptions': endpoints(node.get_subscriptions_info_by_topic(topic + suffix)),
        } for suffix in ('/image', '/odom', '/segments', '/status/sync')}

    params = {
        'input_mode': 'topic', 'image_topic': topic + '/image', 'odom_topic': topic + '/odom',
        'output_topic': topic + '/segments', 'status_topic': topic + '/status',
        'map_image_topic': topic + '/unused_map', 'camera_frame': camera_frame,
        'map_frame': map_frame, 'base_frame': 'base_link', 'use_static_pose': 'false',
        'require_odom_for_mapping': 'true', 'allow_local_without_odom': 'true',
        'sync_images_to_odom': 'true', 'sync_max_wait_sec': '0.08', 'sync_queue_size': '8',
        'startup_warmup_frames': '0', 'publish_debug_map': 'false', 'publish_debug_vectors': 'false',
        'preprocess_backend': 'cpu', 'vectorize_every_n': '1', 'meters_per_pixel': '0.0125',
        'calibration_yaml': (str(ROOT / 'src/vision_mapping_bringup/config/calibration/front_opencv.yaml') if args.package_calibration
                             else str(run / 'calibration.yaml')),
        'profile_csv_path': str(run / 'profile.csv'),
    }
    command = [str(exe), '--ros-args', '-r', '__node:=' + tag + '_lane']
    for name, value in params.items():
        command += ['-p', f'{name}:={value}']
    report['parameters'] = params
    print(f'[DIAG] RMW={report["rmw"]}, image={args.width}x{args.height} '
          f'({len(blank)} bytes), qos={args.image_qos}', flush=True)
    print(f'[LOGS] {run}', flush=True)
    result = 1
    try:
        # Direct child, not ros2-run wrapper: cleanup targets only this process.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        discovered = spin_until(lambda: images.get_subscription_count() > 0
                                and odoms.get_subscription_count() > 0
                                and node.count_publishers(topic + '/segments') > 0, 10.)
        if not discovered:
            raise CheckFailed('DISCOVERY', 'expected publishers/subscribers not discovered')
        print('[PASS DISCOVERY] Graph endpoints found; checking actual payload delivery next', flush=True)
        report['phase'] = 'image_probe'
        probe_stamps = []
        end = time.monotonic() + 8.
        last_progress = 0.
        while time.monotonic() < end:
            probe_stamps.append(send_image())
            spin_until(lambda: False, .10)
            received = [outputs[t] for t in probe_stamps if t in outputs]
            for message in received:
                check_pose(message, False)
            if len(received) >= 3:
                break
            if time.monotonic() - last_progress >= 1.:
                print(f'[PROBE] sent={state["images_sent"]}, vectors={state["vector_messages"]}, '
                      f'status={state["sync"]}', flush=True)
                last_progress = time.monotonic()
        else:
            raise CheckFailed(transport_code(state['sync'], state['vector_messages']),
                              'fewer than three probe frames returned within 8 seconds')
        print('[PASS IMAGE_PROBE] three timestamp-preserving lane outputs received', flush=True)
        # Get a receiver-side heartbeat before testing wrong-frame rejection.
        if not spin_until(lambda: state['sync'] is not None, 2.5):
            raise CheckFailed('STATUS_MISSING', 'lane output works but no sync heartbeat')
        settle(.2)
        case('NO_ODOM', lambda stamp: None, False)
        before = state['sync'].get('rejected_odom', 0)
        case('WRONG_FRAME', lambda stamp: send_odom(stamp, frame='wrong_map'), False)
        if not spin_until(lambda: state['sync'].get('rejected_odom', 0) > before, 2.5):
            raise CheckFailed('ODOM_NOT_CONFIRMED', 'wrong-frame odom rejection was not observed')

        def delayed(stamp):
            settle(.02)
            send_odom(stamp, 1.25)
        case('DELAYED_EXACT', delayed, True, 1.25)
        case('OLD_ONLY', lambda stamp: send_odom(stamp - 10_000_000, 9.), False)

        def bracket(stamp):
            send_odom(stamp - 10_000_000, 1.)
            send_odom(stamp + 10_000_000, 3.)
        case('INTERPOLATION', bracket, True, 2.)
        settle(1.1)
        report['outcome'] = 'PASS'
        result = 0
        print('[PASS ALL] Synthetic image transport and pose synchronization passed. '
              'This does not verify camera FPS, IMU accuracy or real lane mapping.', flush=True)
    except CheckFailed as exc:
        report['outcome'], report['error'] = exc.code, str(exc)
        print(f'[FAIL {exc.code}] phase={report["phase"]}: {exc}', flush=True)
    except KeyboardInterrupt:
        report['outcome'] = 'INTERRUPTED'
        print('[INTERRUPTED] stopping only the diagnostic child', flush=True)
    except Exception as exc:
        report['outcome'], report['error'] = 'EXCEPTION', repr(exc)
        print(f'[FAIL EXCEPTION] {exc!r}', flush=True)
    finally:
        try:
            report['graph'] = graph()
        except Exception as exc:
            report['graph_error'] = repr(exc)
        report['counts'] = {key: state[key] for key in
                            ('images_sent', 'odom_sent', 'vector_messages', 'status_messages')}
        report['last_sync_status'], report['events'] = state['sync'], state['events']
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.)
        log.close()
        (run / 'diagnostic.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print('[SUMMARY] ' + json.dumps(report['counts']) + '; sync=' + json.dumps(state['sync']), flush=True)
        if result:
            print('--- child log (last 25 lines) ---', flush=True)
            print('\n'.join((run / 'node.log').read_text(errors='replace').splitlines()[-25:]), flush=True)
        print(f'[LOGS] {run}/diagnostic.json and {run}/node.log', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
