import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import sys
import tempfile
import types
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/vision_lane_bringup'))
from vision_lane_bringup.odom_filter import LaneOdomFilter, Pose2


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


policy = load('odom_policy', ROOT / 'src/vision_lane_bringup/vision_lane_bringup/odom_policy.py')
summary = load('summary', ROOT / 'scripts/summarize_odom.py')


class OdomTests(unittest.TestCase):
    def test_launch_shadow_isolation_without_ros(self):
        # Validate the actual launch construction; these mocks do not test ROS/DDS.
        class Action:
            def __init__(self, *args, **kwargs):
                self.kwargs = kwargs

        class Configuration:
            def __init__(self, name):
                self.name = name
            def perform(self, context):
                return context[self.name]

        modules = {}
        def module(name, **attributes):
            item = types.ModuleType(name)
            item.__dict__.update(attributes)
            modules[name] = item
        module('ament_index_python')
        module('ament_index_python.packages', get_package_share_directory=lambda name: str(ROOT / 'src' / name))
        module('launch', LaunchDescription=list)
        module('launch.actions', **{name: Action for name in (
            'DeclareLaunchArgument', 'EmitEvent', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler')})
        module('launch.event_handlers', OnProcessExit=Action)
        module('launch.events', Shutdown=Action)
        module('launch.substitutions', LaunchConfiguration=Configuration)
        module('launch_ros')
        module('launch_ros.actions', Node=Action)
        with patch.dict(sys.modules, modules), patch.object(sys, 'path', [str(ROOT / 'src/vision_lane_bringup'), *sys.path]):
            launch = load('single_camera_launch', ROOT / 'src/vision_lane_bringup/launch/single_camera_lane.launch.py')
            for enable in ('false', 'true'):
                with tempfile.TemporaryDirectory() as directory:
                    args = dict(device='/dev/video2', imu_device='/dev/ttyACM0',
                                imu_topic='/sensing/imu/imu_data', width='2560', height='1440', fps='30',
                                warmup_frames='120', use_sim_time='false', launch_camera='true',
                                launch_f9r='true', launch_viewer='true', calibration='', rig_file='', output_dir=directory,
                                odom_profile='bounded', imu_accel_shadow=enable,
                                image_quality_gate_mode='reject',
                                image_quality_min_sharpness='25.0',
                                rear_device='', rear_rig_file='', rear_calibration='')
                    nodes = [a.kwargs for a in launch._launch(args) if 'package' in a.kwargs]
                    self.assertEqual(sum(n['package'] == 'gscam' for n in nodes), 1)
                    self.assertEqual(sum(n['package'] == 'rqt_image_view' for n in nodes), 1)
                    odoms = [n for n in nodes if n['package'] == 'monoscale_odometry']
                    self.assertEqual(len(odoms), 2 if enable == 'true' else 1)
                    self.assertEqual(sum(n['parameters'][-1]['publish_tf'] for n in odoms), 0)
                    main = odoms[0]['parameters'][-1]
                    self.assertTrue(main['use_imu_yaw'])
                    self.assertFalse(main['imu_yaw_from_gyro'])
                    self.assertFalse(main['use_inertial_prediction'])
                    self.assertEqual(main['cameras'], ['front'])
                    self.assertEqual(main['odom_frame'], 'odom')
                    stabilizer = next(n for n in nodes if n['package'] == 'vision_lane_bringup')
                    lane = next(n for n in nodes if n['package'] == 'vision_lane_global_mapping_cpp')
                    self.assertEqual(stabilizer['parameters'][-1]['input_odom_topic'], main['odometry_topic'])
                    self.assertEqual(stabilizer['parameters'][-1]['input_parent_frame'], 'odom')
                    self.assertEqual(stabilizer['parameters'][-1]['parent_frame'], 'map')
                    self.assertEqual(lane['parameters'][-1]['odom_topic'], stabilizer['parameters'][-1]['output_odom_topic'])
                    self.assertTrue(stabilizer['parameters'][-1]['publish_tf'])
                    self.assertEqual(lane['parameters'][-1]['image_quality_gate_mode'], 'reject')
                    if enable == 'true':
                        shadow = odoms[1]['parameters'][-1]
                        self.assertNotEqual(shadow['odometry_topic'], main['odometry_topic'])
                        self.assertTrue(shadow['use_inertial_prediction'])
                        self.assertEqual(shadow['cameras'], ['front'])
                        self.assertFalse(shadow['publish_tf'])
                        self.assertTrue(all(isinstance(p, dict) for p in odoms[1]['parameters']))

    def test_bounded_policy(self):
        p = policy.odom_policy()
        self.assertTrue(p['attitude_from_imu'])
        self.assertTrue(p['use_imu_yaw'])
        self.assertFalse(p['imu_yaw_from_gyro'])
        self.assertFalse(p['use_inertial_prediction'])
        self.assertFalse(p['inertial_use_acceleration'])
        self.assertEqual(p['fusion_model'], 'velocity')
        self.assertFalse(p['coast_on_reject'])

    def test_shadow_is_not_authority(self):
        p = policy.odom_policy(shadow=True)
        self.assertFalse(p['publish_tf'])
        self.assertTrue(p['use_inertial_prediction'])
        self.assertEqual(p['fusion_model'], 'displacement')
        self.assertEqual(policy.odom_policy('legacy'), policy.odom_policy('bounded'))
        with self.assertRaises(ValueError):
            policy.odom_policy('fake')

    def test_lane_odom_filter_rejects_jump_and_only_smooths_stationary(self):
        f = LaneOdomFilter(stationary_tau_sec=.2)
        first = f.update(1.0, Pose2(0., 0., 0.))
        self.assertTrue(first.accepted)
        jitter = f.update(1.1, Pose2(.002, 0., .001))
        self.assertTrue(jitter.stationary)
        self.assertGreater(jitter.pose.x, 0.)
        self.assertLess(jitter.pose.x, .002)
        moving = f.update(1.2, Pose2(.102, 0., .02))
        self.assertTrue(moving.accepted)
        self.assertFalse(moving.stationary)
        self.assertEqual(moving.pose.x, .102)
        jump = f.update(1.3, Pose2(2., 0., .02))
        self.assertFalse(jump.accepted)
        self.assertEqual(jump.reason, 'translation_jump')
        recovered = f.update(1.4, Pose2(.202, 0., .03))
        self.assertTrue(recovered.accepted)

    def test_lane_odom_filter_handles_yaw_wrap(self):
        f = LaneOdomFilter()
        f.update(1.0, Pose2(0., 0., 3.13))
        result = f.update(1.1, Pose2(0., 0., -3.13))
        self.assertTrue(result.accepted)
        self.assertLess(abs(result.raw_yaw_rad), .03)

    def test_rejected_yaw_stream_cannot_bypass_limits_after_timeout(self):
        f = LaneOdomFilter(reset_gap_sec=.75)
        f.update(1.0, Pose2(0., 0., 0.))
        # This models the observed continuous but wrong yaw source: messages
        # continue arriving every 100 ms, so it is not an input outage.
        for index in range(1, 13):
            result = f.update(1.0 + .1 * index, Pose2(0., 0., math.radians(20 + 5 * index)))
            self.assertFalse(result.accepted)
            self.assertEqual(result.reason, 'yaw_jump')
        self.assertAlmostEqual(result.pose.yaw, 0.)

    def test_true_input_outage_can_reinitialize(self):
        f = LaneOdomFilter(reset_gap_sec=.75)
        f.update(1.0, Pose2(0., 0., 0.))
        result = f.update(2.0, Pose2(2., 0., math.radians(90)))
        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, 'gap_reinitialized')

    def test_lane_sync_limits_are_explicit(self):
        lane = yaml.safe_load((ROOT / 'src/vision_lane_bringup/config/lane_stage1.yaml').read_text())['global_mapping_cpp']['ros__parameters']
        self.assertEqual(lane['max_odom_interpolation_gap_sec'], .20)
        self.assertEqual(lane['sync_max_wait_sec'], .25)

    def test_summary_does_not_confuse_endpoint_with_path(self):
        rows = [dict(event='solve', stamp=t, x=x, y=0., published=True,
                     reason='accepted', arrival_age_ms=20., solve_dt_ms=100.)
                for t, x in [(1., 0.), (1.1, 1.), (1.2, 0.)]]
        r = summary.summarize(rows)
        self.assertEqual(r['endpoint_delta_m']['norm'], 0.)
        self.assertEqual(r['estimated_path_length_m'], 2.)
        self.assertAlmostEqual(r['effective_odom_hz'], 10.)
        self.assertEqual(r['gaps_over_200ms'], 0)
        self.assertIn('UNKNOWN', r['accuracy'])

    def test_empty_or_rejected_has_no_rate(self):
        for rows in ([], [dict(event='solve', published=False, reason='no_visual_solution')]):
            r = summary.summarize(rows)
            self.assertIsNone(r['effective_odom_hz'])
            self.assertIsNone(r['published_gap_ms'])

    def test_gaps_and_bad_stamp_are_visible(self):
        rows = [dict(event='solve', stamp=t, published=True, arrival_age_ms=-20.)
                for t in (1., 1.4, 1.4, 1.3)]
        r = summary.summarize(rows)
        self.assertEqual(r['gaps_over_200ms'], 1)
        self.assertEqual(r['nonincreasing_stamps'], 2)
        self.assertEqual(r['future_dated_published'], 4)


if __name__ == '__main__':
    unittest.main()
