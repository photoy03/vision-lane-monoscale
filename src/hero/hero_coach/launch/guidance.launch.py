"""코치와 HUD 만. 인지는 이미 떠 있다고 본다.

`~/carla_wheel` 리그가 조작 창·브리지·어안 조립·monoscale 을 이미 올리므로, 그 위에
얹을 것은 안내 계층뿐이다. `coach.launch.py` 는 인지까지 같이 올리기 때문에 그 리그와
겹친다 — 겹쳐 띄우면 같은 토픽에 발행자가 둘이 된다.

    ros2 launch hero_coach guidance.launch.py                 # 코치 + HUD + 음성
    ros2 launch hero_coach guidance.launch.py hud:=false      # 화면 없이 계획만

HUD 의 화면 선택과 전체화면은 환경변수로 간다 (HERO_HUD_SCREEN, HERO_HUD_FULLSCREEN).
런치가 띄우는 프로세스가 환경을 물려받으므로 run_demo.sh 가 거기서 정한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {'use_sim_time': True}

    def coach(executable, **kwargs):
        return Node(package='hero_coach', executable=executable, output='screen',
                    parameters=[sim_time], **kwargs)

    hud = Node(
        package='hero_hud', executable='hud_node', name='hero_hud', output='screen',
        condition=IfCondition(LaunchConfiguration('hud')),
        parameters=[
            PathJoinSubstitution([FindPackageShare('hero_hud'), 'config', 'hud.yaml']),
            # CARLA 차량은 Prius 다 — 축거가 코리도 곡률을 정한다.
            PathJoinSubstitution([FindPackageShare('hero_coach'), 'config', 'hud_carla.yaml']),
            {'display.mirror_horizontal': LaunchConfiguration('mirror')},
        ])

    return LaunchDescription([
        DeclareLaunchArgument('hud', default_value='true'),
        DeclareLaunchArgument('voice', default_value='true'),
        # 반사면(텔레프롬프터) 없이 모니터로 직접 보는 자리에서는 끈다.
        DeclareLaunchArgument('mirror', default_value='false'),
        coach('vehicle_state_adapter'),
        coach('parking_planner'),
        coach('coach_node'),
        coach('voice_node', condition=IfCondition(LaunchConfiguration('voice'))),
        hud,
    ])
