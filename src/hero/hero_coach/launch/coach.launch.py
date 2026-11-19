"""브리지가 떠 있는 CARLA 위에서: fisheye 조립 → monoscale → 코치 → HUD (+ RViz).

CARLA 서버와 브리지는 tools/carla_live.sh 가 먼저 띄운다. 이 파일은 ROS 쪽만 맡는다.
운전은 다른 터미널에서 tools/teleop.sh.

    ros2 launch hero_coach coach.launch.py            # 전부
    ros2 launch hero_coach coach.launch.py rviz:=false voice:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    coach_config = PathJoinSubstitution([FindPackageShare('hero_coach'), 'config'])

    # 벤치 녹화와 같은 조립이다 (hero-release/sim/rec_slalom.sh).
    def fisheye(end, yaw):
        return Node(
            package='monoscale_carla', executable='fisheye_assembler',
            name=f'fisheye_assembler_{end}', output='screen',
            parameters=[sim_time, {
                'ends': [end], 'base_yaw_deg': [yaw],
                'source_reliability': 'best_effort', 'width': 1280, 'height': 720,
            }])

    monoscale = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('monoscale_odometry'), 'launch', 'odometry.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

    def coach(executable, **kwargs):
        return Node(package='hero_coach', executable=executable, output='screen',
                    parameters=[sim_time], **kwargs)

    hud = Node(
        package='hero_hud', executable='hud_node', name='hero_hud', output='screen',
        condition=IfCondition(LaunchConfiguration('hud')),
        parameters=[
            PathJoinSubstitution([FindPackageShare('hero_hud'), 'config', 'hud.yaml']),
            PathJoinSubstitution([coach_config, 'hud_carla.yaml']),
        ])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('hero_coach'), 'rviz', 'coach.rviz'])],
        parameters=[sim_time])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('hud', default_value='true'),
        DeclareLaunchArgument('voice', default_value='true'),
        fisheye('front', 0.0),
        fisheye('rear', 180.0),
        monoscale,
        coach('vehicle_state_adapter'),
        coach('parking_planner'),
        coach('coach_node'),
        # 말은 벽시계로 돈다(시각을 time.monotonic 으로 잡는다). 모델이 없으면
        # 한 번 경고하고 조용히 지낸다 — tools/fetch_voice.sh.
        coach('voice_node', condition=IfCondition(LaunchConfiguration('voice'))),
        hud,
        rviz,
    ])
