"""Time-matched map set (docs/STRUCTURE_KO.md "통합 지도").

map_assembler combines the lane map and the occupancy grid snapshots that are
both exact at one capture time T (plus the pose at T) into /mapping/fused_map;
map_visualizer draws each set as one image. Control pose and TF do not pass
through here. Off unless launch_fusion:=true.
"""
from vision_mapping_bringup.config import node_parameters


def enabled(args):
    value = args.arg('launch_fusion').strip().lower()
    if value not in ('', 'true', 'false'):
        raise ValueError('launch_fusion must be true, false or empty')
    return value == 'true'


def nodes(ctx):
    if not enabled(ctx.args):
        return []
    if not (ctx.launch_occupancy and ctx.launch_lane_fusion):
        raise ValueError('launch_fusion needs launch_occupancy:=true and launch_lane_fusion:=true')
    common = {'use_sim_time': ctx.sim_time}
    provenance_topic = ctx.args.arg('fusion_provenance_topic').strip()
    return [
        ctx.make_node(package='map_assembly', executable='map_assembler_node', name='map_assembler',
                      output='screen', parameters=[node_parameters('runtime.yaml', 'map_assembler'),
                                                   {**common, 'pose_topic': ctx.topics.odom,
                                                    'provenance_topic': provenance_topic}]),
        ctx.make_node(package='map_assembly', executable='map_visualizer_node', name='map_visualizer',
                      output='screen', parameters=[node_parameters('runtime.yaml', 'map_visualizer'), common]),
    ] + viewer(ctx)


def viewer(ctx):
    """The fused-map image (and, before the first map, the readiness card)
    opens with the other viewers; closing it does not stop mapping."""
    if not ctx.args.flag('launch_viewer'):
        return []
    return [ctx.make_node(package='rqt_image_view', executable='rqt_image_view',
                          name='fused_map_viewer', output='screen',
                          arguments=['/mapping/fused_map/image'])]
