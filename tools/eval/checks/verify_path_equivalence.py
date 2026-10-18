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

NEW = Path(__file__).resolve().parents[3]
LAYOUT = {
    'vision_mapping_bringup': 'src/vision_mapping_bringup',
    'lane_detection': 'src/lane_mapping/lane_detection',
    'lane_map': 'src/lane_mapping/lane_map',
    'vision_lane_interfaces': 'src/interfaces/vision_lane_interfaces',
    'map_assembly': 'src/mapping/map_assembly',
    'monoscale_ublox': 'src/sensors/monoscale_ublox',
    **{name:'src/monoscale/'+name for name in ('monoscale_core','monoscale_tracker','monoscale_odometry','monoscale_occupancy_grid_map','monoscale_pose')},
}
# Reference workspaces use the pre-rename ROS package names; compare under the new names.
RENAMED = {'vision_lane_bringup': 'vision_mapping_bringup',
           'vision_lane_global_mapping_cpp': 'lane_detection',
           'vision_lane_map_fusion': 'lane_map'}
OLD_NAMES = {new: old for old, new in RENAMED.items()}

# Parameters now passed explicitly (package defaults / shared map.yaml) that the
# reference left to the node's C++ declaration. Each value equals that default.
ADDED_EXPLICIT = {
    'monoscale_occupancy_grid_map': {
        'min_blob_reference_pixels': 80, 'publish_rate_hz': 5.0, 'map_resolution_m': 0.1,
        'map_x_min_m': -30.0, 'map_x_max_m': 30.0, 'map_y_min_m': -30.0, 'map_y_max_m': 30.0,
    },
    # Lane delay fix: intended behavior change (node default stays off).
    'global_mapping_cpp': {'pending_drop_unresolvable_head': True},
    # Item 5: IMU-rate fill parameters. imu_fill_enabled=True is the one intended
    # behavior change (the node's own default stays False); the rest equal defaults.
    'lane_odom_stabilizer': {
        'imu_fill_enabled': True, 'imu_fill_rate_hz': 20.0, 'imu_fill_max_span_sec': 3.0,
        'imu_fill_max_closure_m': 0.25, 'imu_fill_max_closure_ratio': 0.3,
    },
}


# Nodes the reference never had. clock_guard only acts when the ROS clock jumps
# back (it then stops the pipeline for a clean restart); it changes nothing else.
ADDED_NODES = {'clock_guard'}


def without_added(nodes):
    result = []
    for node in nodes:
        if node.get('name') in ADDED_NODES:
            continue
        node = dict(node)
        added = ADDED_EXPLICIT.get(node.get('name'), {})
        if isinstance(node.get('parameters'), dict) and added:
            node['parameters'] = {key: value for key, value in node['parameters'].items()
                                  if not (key in added and value == added[key])}
        result.append(node)
    return result


# Launch defaults intentionally changed against the reference (new, non-empty).
INTENDED_LAUNCH_DEFAULTS = {'lane_mapping_owner': 'map'}


def launch_defaults_compatible(reference, current):
    # An empty default now means "use the package default"; the effective
    # graph comparison proves the resulting values did not change.
    # New arguments are allowed only with an empty (package-default) value.
    return reference.keys() <= current.keys() and all(
        reference.get(key, '') == current[key] or current[key] == '' or
        INTENDED_LAUNCH_DEFAULTS.get(key) == current[key] for key in current)


class Action:
    def __init__(self,*args,**kwargs): self.args=args; self.kw=kwargs

class Configuration:
    def __init__(self,key): self.key=key
    def perform(self,context): return context[self.key]

def graph(root, layout, case, group='all'):
    shares={name:root/path for name,path in layout.items()}
    bringup='vision_mapping_bringup' if 'vision_mapping_bringup' in layout else 'vision_lane_bringup'
    def module(name,**attrs):
        result=types.ModuleType(name); result.__dict__.update(attrs); sys.modules[name]=result
    module('ament_index_python'); module('ament_index_python.packages',get_package_share_directory=lambda name:str(shares[name]))
    module('launch',LaunchDescription=list)
    module('launch.actions',**{name:Action for name in ('DeclareLaunchArgument','EmitEvent','LogInfo','OpaqueFunction','RegisterEventHandler')})
    module('launch.event_handlers',OnProcessExit=Action); module('launch.events',Shutdown=Action)
    module('launch.substitutions',LaunchConfiguration=Configuration)
    module('launch_ros'); module('launch_ros.actions',Node=Action)
    for name in list(sys.modules):
        if name.split('.')[0] in ('vision_lane_bringup', 'vision_mapping_bringup', 'monoscale_pose'):
            del sys.modules[name]
    previous_path = list(sys.path)
    sys.path[:0] = [str(shares[bringup]), str(shares['monoscale_pose'])]
    factory=shares[bringup]/bringup/'pipeline.py'
    modern=factory.exists()
    if not modern: factory=shares[bringup]/'launch/single_camera_lane.launch.py'
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
            node['package']=RENAMED.get(node.get('package'),node.get('package'))
            if node.get('executable')=='odom_stabilizer_node.py': node['package']='@POSE_PACKAGE'
        def normalize(value):
            if isinstance(value,str):
                value=value.replace(output,'@OUTPUT')
                if value.endswith(('/lane_occupancy.rviz','/mapping.rviz')): return '@RVIZ'
                for name,path in shares.items(): value=value.replace(str(path),'@PACKAGE/'+RENAMED.get(name,name))
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
    old_layout=({OLD_NAMES.get(n,n):'src/'+OLD_NAMES.get(n,n) for n in LAYOUT}
                if old and not (old/'src/vision_mapping_bringup').exists() else LAYOUT)
    cases=[]
    for dual in (False,True):
        for mode in ('legacy','continuation','compare'):
            # Device paths have no default any more; use the reference's former defaults.
            # OLD has no lane mapper: compare the selectable previous path
            # (lane_mapping_owner:=detection); the map default is checked below.
            case={'lane_mapping_owner':'detection','device':'/dev/video2','imu_device':'/dev/ttyACM0',
                  'rear_enabled':str(dual).lower(),'rear_device':'/dev/video4' if dual else '',
                  'launch_occupancy':'true','launch_rviz':'true','launch_viewer':'false','map_fusion_mode':mode}
            for shadow in (('false',) if dual else ('false','true')):
                case['imu_accel_shadow']=shadow
                b,gb=graph(NEW,LAYOUT,case)
                if old:
                    a,ga=graph(old,old_layout,case)
                    assert launch_defaults_compatible(a,b), 'Launch defaults changed'
                    assert without_added(ga)==without_added(gb), f'Effective node/parameter graph changed: {case}'
                parts=[]
                for group in ('sensors','monoscale','lanes'):
                    _,part=graph(NEW,LAYOUT,case,group); parts.extend(part)
                signature=lambda item:json.dumps(item,sort_keys=True)
                assert sorted(map(signature,parts))==sorted(map(signature,gb)), 'Group union changed'
                cases.append(dict(case=dict(case),nodes=len(gb),passed=True))
    # The default (no override) must be the lane_map-owned mapping path.
    _,default_graph=graph(NEW,LAYOUT,{'device':'/dev/video2','imu_device':'/dev/ttyACM0'})
    lane_maps=[n for n in default_graph if n.get('name')=='lane_map_fusion']
    assert lane_maps and lane_maps[0]['parameters'].get('mapper_enabled') is True, 'lane_mapping_owner default is not map'
    detectors=[n for n in default_graph if n.get('name')=='global_mapping_cpp']
    assert detectors[0]['parameters'].get('mapping_owner')=='map', 'lane_detection is not in detection-only mode'
    result=dict(status='PASS_EFFECTIVE_GRAPH_NOT_LIVE_ACCURACY',reference=str(old),workspace=str(NEW),cases=cases)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],cases=len(cases))))

if __name__=='__main__': main()
