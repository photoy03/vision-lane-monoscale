from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = PathJoinSubstitution([
        FindPackageShare("parking_slot_mapping"), "config", "parking_slot_mapping.yaml"
    ])
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="parking_slot_mapping",
            executable="slot_map_node",
            name="parking_slot_map_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="parking_slot_mapping",
            executable="goal_selector_node",
            name="parking_goal_selector_node",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
    ])
