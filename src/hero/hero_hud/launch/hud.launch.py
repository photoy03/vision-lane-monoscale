"""HUD 렌더러 + (선택) 테스트 시나리오 발행 노드.

    ros2 launch hero_hud hud.launch.py                      # 렌더러만
    ros2 launch hero_hud hud.launch.py scenario:=approach   # 목 데이터와 함께
    ros2 launch hero_hud hud.launch.py scenario:=degraded   # 저하 상태 시연

Orin에서는 데스크톱 없이:

    QT_QPA_PLATFORM=eglfs ros2 launch hero_hud hud.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hero_hud')
    params = os.path.join(share, 'config', 'hud.yaml')

    scenario = LaunchConfiguration('scenario')
    lane_only = LaunchConfiguration('lane_only')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='none',
            description="목 발행 노드의 시나리오. 'none' 이면 안 띄운다. "
                        "approach | degraded",
        ),
        DeclareLaunchArgument('parking_view', default_value='false',
                              description='Top-down parking plan view: ego, slots, selected slot, path'),
        DeclareLaunchArgument('mirror', default_value='true',
                              description='Mirror the picture for the combiner (false on a desktop monitor)'),
        DeclareLaunchArgument('lane_only', default_value='false',
                              description='Show road lane detections without the driving corridor'),
        Node(
            package='hero_hud',
            executable='hud_node',
            name='hero_hud',
            parameters=[params, {'road.lane_only': ParameterValue(lane_only, value_type=bool),
                                 'parking.plan_view': ParameterValue(LaunchConfiguration('parking_view'), value_type=bool),
                                 'display.mirror_horizontal': ParameterValue(LaunchConfiguration('mirror'), value_type=bool)}],
            output='screen',
        ),
        Node(
            package='hero_hud',
            executable='scenario_publisher',
            name='hero_hud_scenario',
            parameters=[{'scenario': scenario}],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", scenario, "' != 'none'"])),
        ),
    ])
