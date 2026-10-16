"""Construction-only launch check with ROS stubs; not a ROS integration test."""
import importlib.util
import sys
import tempfile
import types
from pathlib import Path
import numpy as np
import yaml
from workspace_paths import package_source
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/monoscale/monoscale_pose'))
sys.path.insert(0,str(root/'src/vision_mapping_bringup'))
class Item:
    def __init__(self,*args,**kw):self.args=args;self.kw=kw
class LC:
    def __init__(self,key):self.key=key
    def perform(self,context):return context[self.key]
def module(name,**attrs):
    m=types.ModuleType(name)
    for k,v in attrs.items():setattr(m,k,v)
    sys.modules[name]=m
module('ament_index_python');module('ament_index_python.packages',get_package_share_directory=package_source)
module('launch',LaunchDescription=Item)
module('launch.actions',**{n:Item for n in ['DeclareLaunchArgument','EmitEvent','LogInfo','OpaqueFunction','RegisterEventHandler']})
module('launch.event_handlers',OnProcessExit=Item);module('launch.events',Shutdown=Item)
module('launch.substitutions',LaunchConfiguration=LC);module('launch_ros');module('launch_ros.actions',Node=Item)
spec=importlib.util.spec_from_file_location('pipeline',root/'src/vision_mapping_bringup/vision_mapping_bringup/pipeline.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with tempfile.TemporaryDirectory() as td:
    t=Path(td)
    np.savez(t/'cal.npz',K=np.array([[500.,0,640],[0,500,360],[0,0,1]]),D=np.zeros(4),image_size=[1280,720])
    rig=yaml.safe_load((root/'src/vision_mapping_bringup/config/rig.yaml').read_text())
    rig['imu_axes_aligned_with_base']=True
    (t/'rig.yaml').write_text(yaml.safe_dump(rig))
    base=dict(device='/dev/video2',width='1280',height='720',fps='30',imu_device='/dev/ttyACM0',imu_topic='/sensing/imu/imu_data',launch_camera='true',launch_f9r='true',launch_viewer='true',launch_rviz='true',launch_occupancy='true',occupancy_topic='/perception/occupancy_grid_map',occupancy_processing_width='640',occupancy_use_cuda='true',occupancy_rolling_radius_m='60.0',occupancy_keyframe_travel_m='0.25',occupancy_occupied_probability='0.85',occupancy_free_probability='0.40',occupancy_max_pending_keyframes='2',occupancy_max_odom_gap_sec='0.20',occupancy_image_quality_gate_mode='reject',occupancy_image_quality_min_sharpness='25.0',occupancy_filter_lanes='false',map_extent_mode='github_fixed',lane_duplicate_render_offset_m='0.20',lane_duplicate_candidate_offset_m='0.50',compare_lane_registration='false',yaw_correction_enabled='true',vision_yaw_sign='1.0',imu_yaw_rate_tau_sec='0.12',yaw_bias_tau_sec='30.0',yaw_bias_limit_deg_s='0.35',yaw_straight_rate_deg_s='3.0',yaw_straight_hold_sec='0.8',yaw_turn_blend='0.01',yaw_max_visual_correction_deg='0.12',resample_odom_to_camera='true',use_sim_time='false',warmup_frames='120',calibration=str(t/'cal.npz'),rig_file=str(t/'rig.yaml'),odom_profile='bounded',imu_accel_shadow='false',image_quality_gate_mode='reject',image_quality_min_sharpness='25.0',rear_device='',rear_calibration='',rear_rig_file='')
    base.update(occupancy_free_cost_max='1000.0', occupancy_free_cap='0.5',
                occupancy_free_to_occupied='1.0', occupancy_occlusion_aware='0.55')
    base.update(occupancy_max_odom_gap_sec='0.40',
                occupancy_max_odom_translation_m='0.10',
                occupancy_max_odom_yaw_deg='3.0')
    base['rear_enabled']='false'
    base.update(straight_resample_span_sec='0.0',
                lane_sync_wait_sec='1.20',
                lane_prediction_wait_sec='1.05',
                lane_sync_queue_size='40',
                lane_odom_buffer_sec='4.0')
    base.update(recovery_session_id='', lane_pending_recovery='false',
                lane_pending_ttl_sec='12.0', lane_pending_gap_grace_sec='2.0',
                lane_pending_max_count='128', lane_pending_commit_budget='2')
    declarations=m.generate_launch_description().args[0]
    defaults={item.args[0]:item.kw['default_value'] for item in declarations[:-1]}
    base={**defaults, **base}
    for mode in ('legacy','continuation','compare'):
      for n in [1,2]:
        ctx=dict(base,map_fusion_mode=mode,output_dir=str(t/f'{mode}_{n}'))
        if n==2:ctx.update(rear_device='/dev/video4',rear_calibration='',rear_rig_file='')
        nodes=m._launch(ctx)
        odom=[x for x in nodes if x.kw.get('package')=='monoscale_odometry']
        assert len(odom)==1
        stabilizer=[x for x in nodes if x.kw.get('package')=='monoscale_pose']
        assert len(stabilizer)==1
        assert odom[0].kw['parameters'][-1]['publish_tf'] is False
        assert odom[0].kw['parameters'][-1]['use_imu_yaw'] is True
        assert odom[0].kw['parameters'][-1]['imu_yaw_from_gyro'] is False
        assert odom[0].kw['parameters'][-1]['use_inertial_prediction'] is False
        assert odom[0].kw['parameters'][-1]['odom_frame']=='odom'
        assert stabilizer[0].kw['parameters'][-1]['publish_tf'] is True
        assert odom[0].kw['parameters'][-1]['odometry_topic']==stabilizer[0].kw['parameters'][-1]['input_odom_topic']
        assert len(odom[0].kw['parameters'][-1]['cameras'])==n
        if n==2:
            rear=odom[0].kw['parameters'][-1]
            assert rear['rear.k']==rear['front.k'] and rear['rear.d']==rear['front.d']
            assert rear['rear.translation_base_from_camera']==[-0.3,0.0,1.2]
        assert len([x for x in nodes if x.kw.get('package')=='gscam'])==n
        expected=2 if mode=='compare' else 1
        viewers=[x for x in nodes if x.kw.get('package')=='rqt_image_view']
        fusion=[x for x in nodes if x.kw.get('package')=='lane_map']
        occupancy=[x for x in nodes if x.kw.get('package')=='monoscale_occupancy_grid_map']
        rviz=[x for x in nodes if x.kw.get('package')=='rviz2']
        assert len(occupancy)==1 and len(rviz)==1
        assert occupancy[0].kw['parameters'][-1]['map_frame']=='map'
        assert occupancy[0].kw['parameters'][-1]['odometry_topic']=='/localization/kinematic_state'
        assert occupancy[0].kw['parameters'][-1]['use_cuda'] is True
        assert occupancy[0].kw['parameters'][-1]['free_cost_max'] == 1000.0
        assert occupancy[0].kw['parameters'][-1]['free_cap'] == 0.5
        assert occupancy[0].kw['parameters'][-1]['free_to_occupied'] == 1.0
        assert occupancy[0].kw['parameters'][-1]['occlusion_aware'] == 0.55
        assert occupancy[0].kw['parameters'][-1]['rolling_radius_m']==60.0
        assert occupancy[0].kw['parameters'][-1]['keyframe_travel_m']==0.25
        assert occupancy[0].kw['parameters'][-1]['occupied_probability']==0.85
        assert occupancy[0].kw['parameters'][-1]['free_probability']==0.40
        assert occupancy[0].kw['parameters'][-1]['max_pending_keyframes']==2
        assert occupancy[0].kw['parameters'][-1]['base_frame']=='base_link'
        assert occupancy[0].kw['parameters'][-1]['max_odom_interpolation_gap_sec']==0.40
        assert occupancy[0].kw['parameters'][-1]['max_odom_interpolation_translation_m']==0.10
        assert occupancy[0].kw['parameters'][-1]['max_odom_interpolation_yaw_deg']==3.0
        assert occupancy[0].kw['parameters'][-1]['front.d']==[0.0,0.0,0.0,0.0]
        assert occupancy[0].kw['parameters'][-1]['front.frame_id']=='front_fisheye_camera_optical_frame'
        assert occupancy[0].kw['parameters'][-1]['extent_mode']=='github_fixed'
        assert len(occupancy[0].kw['parameters'][-1]['cameras'])==n
        assert len(viewers)==expected and len(fusion)==expected
        enabled=[x.kw['parameters'][-1]['continuation_mode'] for x in fusion]
        assert enabled==({'legacy':[False],'continuation':[True],'compare':[False,True]}[mode])
        assert all(x.kw['parameters'][-1]['map_frame']=='map' for x in fusion)
        assert all(x.kw['parameters'][-1]['extent_mode']=='github_fixed' for x in fusion)
        assert all(x.kw['parameters'][-1]['enable_pose_registration'] is False for x in fusion)
        assert all(x.kw['parameters'][-1]['occupancy_filter_enabled'] is False for x in fusion)
        assert all(x.kw['parameters'][-1]['duplicate_render_offset_m']==0.20 for x in fusion)
        assert all(x.kw['parameters'][-1]['duplicate_candidate_offset_m']==0.50 for x in fusion)
        topics=[x.kw['parameters'][-1]['output_image_topic'] for x in fusion]
        assert len(topics)==len(set(topics))
    replay=dict(base,map_fusion_mode='legacy',output_dir=str(t/'replay'),
                use_sim_time='true',launch_camera='false',launch_f9r='false',
                launch_viewer='false',launch_rviz='false',launch_occupancy='false',
                rear_enabled='true')
    replay_nodes=m._launch(replay)
    assert not [x for x in replay_nodes if x.kw.get('package')=='gscam']
    replay_odom=[x for x in replay_nodes if x.kw.get('package')=='monoscale_odometry']
    assert replay_odom[0].kw['parameters'][-1]['cameras']==['front','rear']
    recovery=dict(replay,output_dir=str(t/'recovery'),lane_pending_recovery='true',
                  recovery_session_id='test_recovery')
    recovery_nodes=m._launch(recovery)
    mapper=[x for x in recovery_nodes if x.kw.get('package')=='lane_detection'][0]
    fusion=[x for x in recovery_nodes if x.kw.get('package')=='lane_map'][0]
    mp=mapper.kw['parameters'][-1];fp=fusion.kw['parameters'][-1]
    assert mp['pending_observations_enabled'] is True and mp['recovered_output_enabled'] is True
    assert mp['recovery_session_id']=='test_recovery' and len(mp['recovery_calibration_id'])==64
    assert fp['recovered_input_enabled'] is True and fp['warmup_frames']==0
    assert fp['expected_recovery_session_id']=='test_recovery'
    assert fp['expected_calibration_id']==mp['recovery_calibration_id']
print('PASS: one/two-camera launch, one odom authority, legacy/continuation/compare map modes')
