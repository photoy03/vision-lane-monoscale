"""Construction-only launch check with ROS stubs; not a ROS integration test."""
import importlib.util
import sys
import tempfile
import types
from pathlib import Path
import numpy as np
import yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/vision_lane_bringup'))
class Item:
    def __init__(self,*args,**kw):self.kw=kw
class LC:
    def __init__(self,key):self.key=key
    def perform(self,context):return context[self.key]
def module(name,**attrs):
    m=types.ModuleType(name)
    for k,v in attrs.items():setattr(m,k,v)
    sys.modules[name]=m
module('ament_index_python');module('ament_index_python.packages',get_package_share_directory=lambda p:str(root/'src'/p))
module('launch',LaunchDescription=Item)
module('launch.actions',**{n:Item for n in ['DeclareLaunchArgument','EmitEvent','LogInfo','OpaqueFunction','RegisterEventHandler']})
module('launch.event_handlers',OnProcessExit=Item);module('launch.events',Shutdown=Item)
module('launch.substitutions',LaunchConfiguration=LC);module('launch_ros');module('launch_ros.actions',Node=Item)
spec=importlib.util.spec_from_file_location('v4',root/'src/vision_lane_bringup/launch/single_camera_lane.launch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with tempfile.TemporaryDirectory() as td:
    t=Path(td)
    np.savez(t/'cal.npz',K=np.array([[500.,0,640],[0,500,360],[0,0,1]]),D=np.zeros(4),image_size=[1280,720])
    rig=yaml.safe_load((root/'src/vision_lane_bringup/config/rig.yaml').read_text())
    rig['imu_axes_aligned_with_base']=True
    (t/'rig.yaml').write_text(yaml.safe_dump(rig))
    ctx=dict(device='/dev/video2',width='1280',height='720',fps='30',imu_device='/dev/ttyACM0',imu_topic='/sensing/imu/imu_data',launch_camera='true',launch_f9r='true',launch_viewer='true',use_sim_time='false',warmup_frames='120',calibration=str(t/'cal.npz'),rig_file=str(t/'rig.yaml'),output_dir=str(t/'one'),odom_profile='bounded',imu_accel_shadow='false',image_quality_gate_mode='reject',image_quality_min_sharpness='25.0',rear_device='',rear_calibration='',rear_rig_file='')
    for n in [1,2]:
        if n==2:ctx.update(rear_device='/dev/video4',rear_calibration=str(t/'cal.npz'),rear_rig_file=str(t/'rig.yaml'),output_dir=str(t/'two'))
        nodes=m._launch(ctx)
        odom=[x for x in nodes if x.kw.get('package')=='monoscale_odometry']
        assert len(odom)==1
        stabilizer=[x for x in nodes if x.kw.get('package')=='vision_lane_bringup']
        assert len(stabilizer)==1
        assert odom[0].kw['parameters'][-1]['publish_tf'] is False
        assert odom[0].kw['parameters'][-1]['use_imu_yaw'] is True
        assert odom[0].kw['parameters'][-1]['imu_yaw_from_gyro'] is False
        assert odom[0].kw['parameters'][-1]['use_inertial_prediction'] is False
        assert odom[0].kw['parameters'][-1]['odom_frame']=='odom'
        assert stabilizer[0].kw['parameters'][-1]['publish_tf'] is True
        assert odom[0].kw['parameters'][-1]['odometry_topic']==stabilizer[0].kw['parameters'][-1]['input_odom_topic']
        assert len(odom[0].kw['parameters'][-1]['cameras'])==n
        assert len([x for x in nodes if x.kw.get('package')=='gscam'])==n
        assert len([x for x in nodes if x.kw.get('package')=='rqt_image_view'])==1
        assert len([x for x in nodes if x.kw.get('package')=='vision_lane_map_fusion'])==1
print('PASS: one/two-camera launch construction, one odom authority, integrated map node')
