"""Scenario nodes on top of a running pipeline (pose_source:=lidar, launch_parking_assist:=true).

  ros2 launch parking_scenario parking_scenario.launch.py use_sim_time:=true

"Nearest visible slot" behaviour (every option is off by default; these are the evaluated values):
  select_mode:=nearest select_max_distance_m:=15.0 line_max_yaw_rate_dps:=8.0 max_plan_distance_m:=25.0
  planner_step_m:=0.6 planner_xy_resolution_m:=0.4 planner_footprint_check:=true display_predict_hz:=30.0
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
        DeclareLaunchArgument("select_mode", default_value="sticky",
                              description="sticky = keep the first choice; nearest = follow the nearest usable slot"),
        DeclareLaunchArgument("select_max_distance_m", default_value="25.0"),
        DeclareLaunchArgument("select_switch_margin_m", default_value="3.0"),
        DeclareLaunchArgument("max_plan_distance_m", default_value="0.0", description="0 = off"),
        DeclareLaunchArgument("retry_cooldown_sec", default_value="0.0", description="0 = off"),
        DeclareLaunchArgument("planner_step_m", default_value="0.35", description="evaluated faster value 0.6"),
        DeclareLaunchArgument("planner_xy_resolution_m", default_value="0.25", description="evaluated faster value 0.4"),
        DeclareLaunchArgument("planner_footprint_check", default_value="false"),
        DeclareLaunchArgument("display_align_rows", default_value="false", description="HUD only: draw the slots of a row as a regular row"),
        DeclareLaunchArgument("display_fill_slots", default_value="0", description="HUD only, with display_align_rows: draw up to N missing places between slots of a row (as unknown)"),
        DeclareLaunchArgument("display_free_only", default_value="false", description="HUD only: draw free slots only"),
        DeclareLaunchArgument("display_ego_smooth_sec", default_value="0.0", description="0 = off; with display_predict_hz: steady vehicle pose in the HUD picture, e.g. 0.15"),
        DeclareLaunchArgument("display_predict_hz", default_value="0.0", description="0 = off; HUD picture rate, e.g. 30"),
        Node(package="parking_scenario", executable="scenario_node", output="screen",
             parameters=[sim, {"lidar_x_m": number("lidar_x_m"),
                               "lidar_yaw_offset_deg": number("lidar_yaw_offset_deg"),
                               "rig_to_rear_axle_m": number("rig_to_rear_axle_m"),
                               "slot_width": number("slot_width"), "slot_depth": number("slot_depth"),
                               "line_memory_sec": number("line_memory_sec"),
                               "release_distance_m": number("release_distance_m"),
                               "select_mode": LaunchConfiguration("select_mode"),
                               "select_max_distance_m": number("select_max_distance_m"),
                               "select_switch_margin_m": number("select_switch_margin_m"),
                               "line_max_yaw_rate_dps": number("line_max_yaw_rate_dps")}]),
        Node(package="parking_scenario", executable="planner_node", output="screen",
             parameters=[sim, {"max_plan_distance_m": number("max_plan_distance_m"),
                               "retry_cooldown_sec": number("retry_cooldown_sec"),
                               "step_m": number("planner_step_m"),
                               "xy_resolution_m": number("planner_xy_resolution_m"),
                               "footprint_check": ParameterValue(LaunchConfiguration("planner_footprint_check"),
                                                                 value_type=bool)}]),
        Node(package="parking_scenario", executable="display_filter_node", output="screen",
             parameters=[sim, {"filter_enabled": ParameterValue(LaunchConfiguration("display_filter"),
                                                                value_type=bool),
                               "predict_hz": number("display_predict_hz"),
                               "ego_smooth_sec": number("display_ego_smooth_sec"),
                               "align_rows": ParameterValue(LaunchConfiguration("display_align_rows"), value_type=bool),
                               "align_fill_slots": ParameterValue(LaunchConfiguration("display_fill_slots"), value_type=int),
                               "free_only": ParameterValue(LaunchConfiguration("display_free_only"), value_type=bool)}]),
    ])
