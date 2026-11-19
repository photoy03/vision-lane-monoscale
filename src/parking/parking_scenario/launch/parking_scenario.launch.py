"""Scenario nodes on top of a running pipeline (pose_source:=lidar, launch_parking_assist:=true).

  ros2 launch parking_scenario parking_scenario.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    number = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_x_m", default_value="-2.71",
                              description="LiDAR in the pose's base_link (rig point)"),
        DeclareLaunchArgument("lidar_yaw_offset_deg", default_value="31.1"),
        DeclareLaunchArgument("rig_to_rear_axle_m", default_value="3.61"),
        DeclareLaunchArgument("slot_width", default_value="2.60"),
        DeclareLaunchArgument("slot_depth", default_value="4.37"),
        DeclareLaunchArgument("display_filter", default_value="true"),
        DeclareLaunchArgument("line_memory_sec", default_value="2.0"),
        DeclareLaunchArgument("release_distance_m", default_value="0.0", description="0 = off"),
        DeclareLaunchArgument("line_max_yaw_rate_dps", default_value="0.0", description="0 = off; ruling 7 value 8.0"),
        DeclareLaunchArgument("max_plan_distance_m", default_value="0.0", description="0 = off"),
        DeclareLaunchArgument("retry_cooldown_sec", default_value="0.0", description="0 = off"),
        Node(package="parking_scenario", executable="scenario_node", output="screen",
             parameters=[sim, {"lidar_x_m": number("lidar_x_m"),
                               "lidar_yaw_offset_deg": number("lidar_yaw_offset_deg"),
                               "rig_to_rear_axle_m": number("rig_to_rear_axle_m"),
                               "slot_width": number("slot_width"), "slot_depth": number("slot_depth"),
                               "line_memory_sec": number("line_memory_sec"),
                               "release_distance_m": number("release_distance_m"),
                               "line_max_yaw_rate_dps": number("line_max_yaw_rate_dps")}]),
        Node(package="parking_scenario", executable="planner_node", output="screen",
             parameters=[sim, {"max_plan_distance_m": number("max_plan_distance_m"),
                               "retry_cooldown_sec": number("retry_cooldown_sec")}]),
        Node(package="parking_scenario", executable="display_filter_node", output="screen",
             parameters=[sim, {"filter_enabled": ParameterValue(LaunchConfiguration("display_filter"),
                                                                value_type=bool)}]),
    ])
