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
sys.path.insert(0, str(ROOT / 'src/vision_mapping_bringup'))
sys.path.insert(0, str(ROOT / 'src/monoscale/monoscale_pose'))
from monoscale_pose.odom_filter import (
    ESM_MARKER,
    LaneOdomFilter,
    Pose2,
    VisionYawCorrector,
    VisionYawSample,
    interpolate_imu_yaw,
    interpolate_pose,
    imu_straight_resample_safe,
    imu_stationary_resample_safe,
    resampled_pose_sigmas,
    use_imu_yaw_for_resample,
    resample_span_kind,
    select_imu_resample_kind,
    imu_orientation_is_usable,
    covariance_matrix_is_usable,
    interpolate_pose_covariance,
    can_resample_span,
    parse_tracker_esm,
)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


from workspace_paths import package_source

policy = load('odom_policy', ROOT / 'src/vision_mapping_bringup/vision_mapping_bringup/odom_policy.py')
summary = load('summary', ROOT / 'tools/eval/summarize_odom.py')


class OdomTests(unittest.TestCase):
    def test_launch_shadow_isolation_without_ros(self):
        # Validate the actual launch construction; these mocks do not test ROS/DDS.
        class Action:
            def __init__(self, *args, **kwargs):
                self.args = args
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
        module('ament_index_python.packages', get_package_share_directory=package_source)
        module('launch', LaunchDescription=list)
        module('launch.actions', **{name: Action for name in (
            'DeclareLaunchArgument', 'EmitEvent', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler')})
        module('launch.event_handlers', OnProcessExit=Action)
        module('launch.events', Shutdown=Action)
        module('launch.substitutions', LaunchConfiguration=Configuration)
        module('launch_ros')
        module('launch_ros.actions', Node=Action)
        with patch.dict(sys.modules, modules), patch.object(sys, 'path', [str(ROOT / 'src/vision_mapping_bringup'), *sys.path]):
            launch = load('single_camera_launch', ROOT / 'src/vision_mapping_bringup/vision_mapping_bringup/pipeline.py')
            for enable in ('false', 'true'):
                with tempfile.TemporaryDirectory() as directory:
                    args = dict(device='/dev/video2', imu_device='/dev/ttyACM0',
                                imu_topic='/sensing/imu/imu_data', width='2560', height='1440', fps='30',
                                warmup_frames='120', use_sim_time='false', launch_camera='true',
                                launch_f9r='true', launch_viewer='true', calibration='', rig_file='', output_dir=directory,
                                launch_rviz='false', launch_occupancy='false',
                                occupancy_topic='/perception/occupancy_grid_map',
                                occupancy_processing_width='1280', occupancy_use_cuda='true',
                                occupancy_rolling_radius_m='60.0', occupancy_keyframe_travel_m='0.25',
                                occupancy_occupied_probability='0.85',
                                occupancy_free_probability='0.40',
                                occupancy_max_pending_keyframes='2',
                                occupancy_max_odom_gap_sec='0.20',
                                occupancy_image_quality_gate_mode='reject',
                                occupancy_image_quality_min_sharpness='25.0',
                                occupancy_filter_lanes='false', map_extent_mode='github_fixed',
                                map_fusion_mode='legacy', compare_lane_registration='false',
                                yaw_correction_enabled='true', vision_yaw_sign='1.0',
                                imu_yaw_rate_tau_sec='0.12', yaw_bias_tau_sec='30.0',
                                yaw_bias_limit_deg_s='0.35', yaw_straight_rate_deg_s='3.0',
                                yaw_straight_hold_sec='0.8', yaw_turn_blend='0.01',
                                straight_resample_span_sec='0.0',
                                lane_sync_wait_sec='1.20',
                                lane_prediction_wait_sec='1.05',
                                lane_sync_queue_size='40',
                                lane_odom_buffer_sec='4.0',
                                lane_duplicate_render_offset_m='0.20',
                                lane_duplicate_candidate_offset_m='0.50',
                                yaw_max_visual_correction_deg='0.12',
                                resample_odom_to_camera='true',
                                odom_profile='bounded', imu_accel_shadow=enable,
                                image_quality_gate_mode='reject',
                                image_quality_min_sharpness='25.0',
                                rear_enabled='false', rear_device='', rear_rig_file='', rear_calibration='')
                    # Inherit newly exposed launch options; keep this fixture's
                    # explicit test values without changing production defaults.
                    description = launch.generate_launch_description()
                    declarations = description.args[0] if hasattr(description, 'args') else description
                    defaults = {a.args[0]: a.kwargs['default_value']
                                for a in declarations
                                if a.args and 'default_value' in a.kwargs}
                    args = {**defaults, **args}
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
                    stabilizer = next(n for n in nodes if n['package'] == 'monoscale_pose')
                    lane = next(n for n in nodes if n['package'] == 'lane_detection')
                    self.assertEqual(stabilizer['parameters'][-1]['input_odom_topic'], main['odometry_topic'])
                    self.assertEqual(stabilizer['parameters'][-1]['input_parent_frame'], 'odom')
                    self.assertEqual(stabilizer['parameters'][-1]['parent_frame'], 'map')
                    self.assertEqual(lane['parameters'][-1]['odom_topic'], stabilizer['parameters'][-1]['output_odom_topic'])
                    self.assertTrue(stabilizer['parameters'][-1]['publish_tf'])
                    self.assertTrue(stabilizer['parameters'][-1]['yaw_correction_enabled'])
                    self.assertTrue(stabilizer['parameters'][-1]['resample_to_vision_stamps'])
                    self.assertEqual(stabilizer['parameters'][-1]['vision_tracks_topic'], '/vision/tracks/front')
                    self.assertEqual(lane['parameters'][-1]['image_quality_gate_mode'], 'reject')
                    if enable == 'true':
                        shadow = odoms[1]['parameters'][-1]
                        self.assertNotEqual(shadow['odometry_topic'], main['odometry_topic'])
                        self.assertTrue(shadow['use_inertial_prediction'])
                        self.assertEqual(shadow['cameras'], ['front'])
                        self.assertFalse(shadow['publish_tf'])
                        self.assertTrue(all(isinstance(p, dict) for p in odoms[1]['parameters']))

    def test_registration_comparison_is_separate_from_shared_map(self):
        package = ROOT / 'src/vision_mapping_bringup/vision_mapping_bringup'
        text = '\n'.join(path.read_text() for path in sorted(package.rglob('*.py')))
        self.assertIn("'enable_pose_registration': False", text)
        self.assertIn("'enable_pose_registration': True", text)
        self.assertIn('/parking/cpp/global_marking_map_registered_compare', text)
        self.assertIn("'occupancy_filter_enabled': False", text)

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

    def test_long_resampling_requires_confirmed_stationary_endpoints(self):
        start = Pose2(0., 0., 0.)
        self.assertTrue(can_resample_span(start, Pose2(.04, 0., math.radians(1)), 2.0))
        self.assertFalse(can_resample_span(start, Pose2(.08, 0., 0.), 2.0))
        self.assertFalse(can_resample_span(start, Pose2(0., 0., math.radians(10)), 2.0))
        self.assertFalse(can_resample_span(start, start, 3.1))
        self.assertTrue(can_resample_span(start, Pose2(.20, 0., 0.), .2))
        self.assertFalse(can_resample_span(start, start, float('nan')))

    def test_straight_gap_resampling_is_opt_in_and_bounded(self):
        start = Pose2(0.0, 0.0, 0.0)
        settings = dict(straight_limit=2.0, straight_translation=0.5,
                        straight_yaw_deg=2.0, straight_lateral=0.1)
        self.assertFalse(can_resample_span(start, Pose2(0.2, 0.0, 0.0), 1.0))
        self.assertEqual(
            resample_span_kind(start, Pose2(0.2, 0.02, math.radians(1)), 1.0, **settings),
            "straight")
        self.assertEqual(
            resample_span_kind(start, Pose2(0.2, 0.02, math.radians(1)), 2.1, **settings),
            "reject")
        self.assertEqual(
            resample_span_kind(start, Pose2(0.2, 0.02, math.radians(8)), 1.0, **settings),
            "reject")
        self.assertEqual(
            resample_span_kind(start, Pose2(0.2, 0.11, 0.0), 1.0, **settings),
            "reject")
        self.assertEqual(
            resample_span_kind(start, Pose2(-0.2, 0.0, 0.0), 1.0, **settings),
            "reject")
    def test_imu_assisted_straight_gap_accepts_only_continuous_low_yaw_motion(self):
        start, end = 1.0, 1.6
        straight = [(start + index * 0.01, 0.01 * index / 60.0)
                    for index in range(61)]
        self.assertTrue(imu_straight_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.4, 0.0, 0.01),
            start, end, straight))

        # The endpoints look straight, but the intervening IMU saw a turn.
        hidden_turn = []
        for index in range(61):
            phase = index / 60.0
            excursion = math.radians(35.0) * (1.0 - abs(2.0 * phase - 1.0))
            hidden_turn.append((start + index * 0.01, excursion))
        self.assertFalse(imu_straight_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.4, 0.0, 0.0),
            start, end, hidden_turn))

        # A true 90-degree turn and an IMU dropout both remain rejected.
        turn_90 = [(start + index * 0.01, math.pi / 2.0 * index / 60.0)
                   for index in range(61)]
        self.assertFalse(imu_straight_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.4, 0.0, math.pi / 2.0),
            start, end, turn_90))
        dropout = [row for index, row in enumerate(straight)
                   if not 26 <= index <= 40]
        self.assertFalse(imu_straight_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.4, 0.0, 0.01),
            start, end, dropout))

    def test_stationary_backfill_requires_continuous_stable_imu_yaw(self):
        start, end = 1.0, 2.8
        flat = [(start + i * 0.02, 0.001 * i / 90.0) for i in range(91)]
        self.assertTrue(imu_stationary_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.01, 0.0, 0.001), start, end, flat))
        hidden_turn = [(t, yaw + (math.radians(4.0) if 35 <= i <= 55 else 0.0))
                       for i, (t, yaw) in enumerate(flat)]
        self.assertFalse(imu_stationary_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.01, 0.0, 0.001), start, end, hidden_turn))
        dropped = [row for i, row in enumerate(flat) if not 30 <= i <= 40]
        self.assertFalse(imu_stationary_resample_safe(
            Pose2(0.0, 0.0, 0.0), Pose2(0.01, 0.0, 0.001), start, end, dropped))

    def test_stationary_guard_rejection_cannot_fall_through_to_straight_guard(self):
        start, end = 10.0, 10.5
        hidden_bump = [
            (start + i * 0.05,
             math.radians(1.0) * math.sin(math.pi * i / 10.0))
            for i in range(11)
        ]
        a = Pose2(0.0, 0.0, 0.0)
        b = Pose2(0.0, 0.0, 0.0)
        stationary_safe = imu_stationary_resample_safe(
            a, b, start, end, hidden_bump,
            max_curve_yaw_deg=0.5, max_sample_gap_sec=0.12)
        straight_safe = imu_straight_resample_safe(
            a, b, start, end, hidden_bump,
            max_curve_yaw_deg=3.0, max_lateral_m=0.12,
            max_sample_gap_sec=0.12)
        self.assertFalse(stationary_safe)
        self.assertTrue(straight_safe)
        self.assertEqual(
            select_imu_resample_kind("stationary", stationary_safe, straight_safe),
            "reject")
        self.assertEqual(
            select_imu_resample_kind("reject", False, True), "imu_straight")

    def test_stationary_guard_accepts_small_reverse_settling(self):
        samples = [(1.0 + i * 0.02, 0.0) for i in range(26)]
        self.assertTrue(imu_stationary_resample_safe(
            Pose2(0.01, 0.0, 0.0), Pose2(0.0, 0.0, 0.0),
            1.0, 1.5, samples))

    def test_imu_orientation_unavailable_flag_is_rejected(self):
        q = (0.0, 0.0, 0.0, 1.0)
        self.assertFalse(imu_orientation_is_usable([-1.0] + [0.0] * 8, q))
        self.assertTrue(imu_orientation_is_usable([0.0] * 9, q))
        self.assertTrue(imu_orientation_is_usable(
            __import__('array').array('d', [0.0] * 9), q))
        self.assertFalse(imu_orientation_is_usable([0.0] * 9, (0.0, 0.0, 0.0, 0.0)))
        self.assertFalse(imu_orientation_is_usable([float('nan')] * 9, q))

    def test_stationary_imu_is_gate_only_not_resampled_yaw(self):
        self.assertTrue(use_imu_yaw_for_resample("imu_straight"))
        self.assertFalse(use_imu_yaw_for_resample("imu_stationary"))
        self.assertFalse(use_imu_yaw_for_resample("short"))

    def test_resampled_pose_uncertainty_grows_with_gap_and_distance(self):
        short = resampled_pose_sigmas(.2, .05, "short")
        long = resampled_pose_sigmas(.8, .6, "imu_straight")
        stationary = resampled_pose_sigmas(2.0, .01, "imu_stationary")
        self.assertLess(short[0], long[0])
        self.assertLess(short[1], long[1])
        self.assertLess(stationary[0], long[0])
        self.assertGreater(stationary[1], 0.0)
        with self.assertRaises(ValueError):
            resampled_pose_sigmas(float("nan"), 0.0, "short")

    def test_imu_yaw_interpolation_handles_wraparound_and_unbracketed_times(self):
        near_pi = interpolate_imu_yaw(
            [(1.0, math.radians(179.0)), (1.01, math.radians(-179.0))],
            1.005,
        )
        self.assertIsNotNone(near_pi)
        self.assertLess(abs(abs(near_pi) - math.pi), math.radians(0.1))
        self.assertIsNone(interpolate_imu_yaw([(1.0, 0.0)], 1.1))

    def test_lane_sync_limits_are_explicit(self):
        lane = yaml.safe_load((ROOT / 'src/lane_mapping/lane_detection/config/global_mapping_cpp.yaml').read_text())['global_mapping_cpp']['ros__parameters']
        self.assertEqual(lane['max_odom_interpolation_gap_sec'], .25)
        self.assertEqual(lane['max_odom_prediction_sec'], .40)
        self.assertEqual(lane['sync_max_wait_sec'], 1.20)
        self.assertGreaterEqual(lane['sync_queue_size'], 30)
        odom = yaml.safe_load((ROOT / 'src/monoscale/monoscale_pose/config/odom_stabilizer.yaml').read_text())['lane_odom_stabilizer']['ros__parameters']
        self.assertLessEqual(odom['max_resample_span_sec'], .25)
        self.assertEqual(odom['imu_stationary_max_curve_yaw_deg'], .5)
        self.assertEqual(odom['imu_stationary_max_endpoint_yaw_error_deg'], 4.0)
        fusion = yaml.safe_load((ROOT / 'src/lane_mapping/lane_map/config/map_fusion.yaml').read_text())['/**']['ros__parameters']
        self.assertFalse(fusion['reject_late_input'])

    def test_tracker_esm_parser_preserves_invalid_frames(self):
        count = 2
        after = 4 + 5 * count
        data = [0.0] * (after + 30)
        data[0], data[1], data[2], data[3] = 1.25, count, 640, 400
        data[after] = 0.12
        data[after + 15] = 0.03
        data[after + 29] = ESM_MARKER
        sample = parse_tracker_esm(data)
        self.assertTrue(sample.valid)
        self.assertAlmostEqual(sample.yaw_delta, .03)
        data[after + 29] = 0.0
        self.assertFalse(parse_tracker_esm(data).valid)

    def test_visual_yaw_learns_straight_gyro_bias(self):
        corrector = VisionYawCorrector(
            bias_tau_sec=.5, turn_blend=0.0, min_linear_speed_mps=.01,
            straight_hold_sec=.1, bias_limit_rad_s=.03,
        )
        corrector.update(1.0, Pose2(0., 0., 0.))
        result = None
        for i in range(1, 31):
            stamp = 1.0 + .1 * i
            corrector.add_vision_sample(VisionYawSample(stamp, 0.0, .1, True))
            result = corrector.update(stamp, Pose2(.1 * i, 0., .002 * i))
        self.assertTrue(result.vision_used)
        self.assertGreater(result.gyro_bias_rad_s, .015)
        self.assertLess(abs(result.pose.yaw), .02)

    def test_visual_turn_is_bounded_and_rotates_xy_consistently(self):
        corrector = VisionYawCorrector(
            turn_blend=.25, min_linear_speed_mps=.01,
            max_visual_adjustment_rad=math.pi,
        )
        corrector.update(1.0, Pose2(0., 0., 0.))
        corrector.add_vision_sample(VisionYawSample(1.1, .10, .1, True))
        first = corrector.update(1.1, Pose2(1., 0., .20))
        self.assertAlmostEqual(first.corrected_yaw_delta, .175, places=6)
        corrector.add_vision_sample(VisionYawSample(1.2, 0.0, .1, True))
        second = corrector.update(1.2, Pose2(2., 0., .20))
        self.assertLess(second.pose.y, -.02)
        self.assertLess(second.pose.x, 2.0)

    def test_visual_turn_correction_is_capped_and_does_not_learn_bias(self):
        cap = math.radians(.12)
        corrector = VisionYawCorrector(
            turn_blend=.5, min_linear_speed_mps=.01,
            max_visual_adjustment_rad=cap, straight_hold_sec=.1,
        )
        corrector.update(1.0, Pose2(0., 0., 0.))
        corrector.add_vision_sample(VisionYawSample(1.1, 0.0, .1, True))
        result = corrector.update(1.1, Pose2(.1, 0., math.radians(8.)))
        self.assertAlmostEqual(result.corrected_yaw_delta, math.radians(8.) - cap)
        self.assertEqual(result.gyro_bias_rad_s, 0.0)

    def test_pose_interpolation_uses_short_yaw_arc(self):
        held = interpolate_pose(Pose2(0., 0., 3.0), Pose2(2., 0., -3.0), .5)
        self.assertAlmostEqual(held.x, 1.0)
        self.assertGreater(abs(held.yaw), 3.0)

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

    def test_pose_covariance_interpolation_is_psd_and_conservative(self):
        previous = [0.0] * 36
        current = [0.0] * 36
        for covariance in (previous, current):
            covariance[0] = covariance[7] = covariance[35] = 0.04
            covariance[1] = covariance[6] = 0.03
            covariance[14] = covariance[21] = covariance[28] = 1.0e6
        self.assertTrue(covariance_matrix_is_usable(previous))

        result = interpolate_pose_covariance(previous, current, 0.5, 0.2, 0.0, "short")
        self.assertTrue(covariance_matrix_is_usable(result))
        self.assertGreater(result[0], 0.04)
        self.assertGreater(result[7], 0.04)
        self.assertGreater(result[35], 0.04)
        self.assertGreaterEqual(result[0] - abs(result[1]), -1e-10)

    def test_unknown_or_non_psd_pose_covariance_stays_unknown(self):
        valid = [0.0] * 36
        valid[0] = valid[7] = valid[35] = 0.04
        self.assertEqual(
            interpolate_pose_covariance([0.0] * 36, valid, 0.5, 0.2, 0.0, "short")[0],
            -1.0)

        invalid = list(valid)
        invalid[1] = invalid[6] = 0.05
        self.assertFalse(covariance_matrix_is_usable(invalid))
        result = interpolate_pose_covariance(invalid, valid, 0.5, 0.2, 0.0, "short")
        self.assertEqual(result[0], -1.0)
        self.assertFalse(covariance_matrix_is_usable(result))

        nonfinite = list(valid)
        nonfinite[8] = float("nan")
        self.assertFalse(covariance_matrix_is_usable(nonfinite))

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
