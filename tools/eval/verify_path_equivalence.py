#!/usr/bin/env python3
"""Compare source launch graphs after path normalization; no ROS devices run."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import yaml

NEW = Path(__file__).resolve().parents[2]
LAYOUT = {
    'vision_lane_bringup': 'src/vision_mapping_bringup',
    'vision_lane_global_mapping_cpp': 'src/lane_mapping/lane_detection',
    'vision_lane_map_fusion': 'src/lane_mapping/lane_map',
    'vision_lane_interfaces': 'src/interfaces/vision_lane_interfaces',
    'monoscale_ublox': 'src/sensors/monoscale_ublox',
    **{name:'src/monoscale/'+name for name in ('monoscale_core','monoscale_tracker','monoscale_odometry','monoscale_occupancy_grid_map','monoscale_pose')},
}

class Action:
    def __init__(self,*args,**kwargs): self.args=args; self.kw=kwargs

class Configuration:
    def __init__(self,key): self.key=key
    def perform(self,context): return context[self.key]

def graph(root, layout, case, group='all'):
    shares={name:root/path for name,path in layout.items()}
    def module(name,**attrs):
        result=types.ModuleType(name); result.__dict__.update(attrs); sys.modules[name]=result
    module('ament_index_python'); module('ament_index_python.packages',get_package_share_directory=lambda name:str(shares[name]))
    module('launch',LaunchDescription=list)
    module('launch.actions',**{name:Action for name in ('DeclareLaunchArgument','EmitEvent','LogInfo','OpaqueFunction','RegisterEventHandler')})
    module('launch.event_handlers',OnProcessExit=Action); module('launch.events',Shutdown=Action)
    module('launch.substitutions',LaunchConfiguration=Configuration)
    module('launch_ros'); module('launch_ros.actions',Node=Action)
    for name in list(sys.modules):
        if name.split('.')[0] in ('vision_lane_bringup', 'monoscale_pose'):
            del sys.modules[name]
    previous_path = list(sys.path)
    sys.path[:0] = [str(shares['vision_lane_bringup']), str(shares['monoscale_pose'])]
    factory=shares['vision_lane_bringup']/'vision_lane_bringup/pipeline.py'
    modern=factory.exists()
    if not modern: factory=shares['vision_lane_bringup']/'launch/single_camera_lane.launch.py'
    spec=importlib.util.spec_from_file_location('migration_launch',factory)
    launch=importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    defaults={a.args[0]:a.kw['default_value'] for a in launch.generate_launch_description() if a.args and 'default_value' in a.kw}
    with tempfile.TemporaryDirectory(prefix='support-equivalence-') as output:
        context={**defaults,**case,'output_dir':output,'recovery_session_id':'equivalence'}
        actions=launch._launch(context,group) if modern else launch._launch(context)
        nodes=[dict(item.kw) for item in actions if 'package' in item.kw]
        for node in nodes:
            effective={}
            for parameter in node.get('parameters',[]):
                if isinstance(parameter,dict): effective.update(parameter)
                else:
                    data=yaml.safe_load(Path(parameter).read_text())
                    for key in ('/**',node.get('name'),'/' + node.get('name','')):
                        effective.update(data.get(key,{}).get('ros__parameters',{}))
            if 'parameters' in node: node['parameters']=effective
            if node.get('executable')=='odom_stabilizer_node.py': node['package']='@POSE_PACKAGE'
        def normalize(value):
            if isinstance(value,str):
                value=value.replace(output,'@OUTPUT')
                if value.endswith(('/lane_occupancy.rviz','/mapping.rviz')): return '@RVIZ'
                for name,path in shares.items(): value=value.replace(str(path),'@PACKAGE/'+name)
                return value
            if isinstance(value,list): return [normalize(x) for x in value]
            if isinstance(value,tuple): return tuple(normalize(x) for x in value)
            if isinstance(value,dict): return {k:normalize(v) for k,v in value.items()}
            return value
        result=normalize(nodes)
    sys.path[:]=previous_path
    return defaults,result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); old=args.reference.resolve() if args.reference else None
    old_layout=LAYOUT if old and (old/'src/vision_mapping_bringup').exists() else {name:'src/'+name for name in LAYOUT}
    cases=[]
    for dual in (False,True):
        for mode in ('legacy','continuation','compare'):
            case={'rear_enabled':str(dual).lower(),'rear_device':'/dev/video4' if dual else '',
                  'launch_occupancy':'true','launch_rviz':'true','launch_viewer':'false','map_fusion_mode':mode}
            for shadow in (('false',) if dual else ('false','true')):
                case['imu_accel_shadow']=shadow
                b,gb=graph(NEW,LAYOUT,case)
                if old:
                    a,ga=graph(old,old_layout,case)
                    assert a==b, 'Launch defaults changed'
                    assert ga==gb, f'Effective node/parameter graph changed: {case}'
                parts=[]
                for group in ('sensors','monoscale','lanes'):
                    _,part=graph(NEW,LAYOUT,case,group); parts.extend(part)
                signature=lambda item:json.dumps(item,sort_keys=True)
                assert sorted(map(signature,parts))==sorted(map(signature,gb)), 'Group union changed'
                cases.append(dict(case=dict(case),nodes=len(gb),passed=True))
    result=dict(status='PASS_EFFECTIVE_GRAPH_NOT_LIVE_ACCURACY',reference=str(old),workspace=str(NEW),cases=cases)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],cases=len(cases))))

if __name__=='__main__': main()
