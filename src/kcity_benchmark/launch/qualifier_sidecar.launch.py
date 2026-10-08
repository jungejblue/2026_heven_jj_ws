"""Qualifier scenario and evaluator alongside an already running CARLA/JJ bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("kcity_scenario_manager")) / "config" / "qualifier.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(config)),
        DeclareLaunchArgument("show_hud", default_value="false"),
        DeclareLaunchArgument("enable_traffic_light_stub", default_value="false"),
        DeclareLaunchArgument("lane_geometry_source", default_value=""),
        DeclareLaunchArgument("lane_debug_draw", default_value="false"),
        Node(package="kcity_scenario_manager", executable="qualifier_scenario_manager",
             name="qualifier_scenario_manager", output="screen",
             parameters=[{"config_file": LaunchConfiguration("config_file"),
                          "require_ego_role": True, "use_sim_time": True}]),
        Node(package="kcity_benchmark", executable="benchmark_runner",
             name="kcity_qualifier_benchmark", output="screen",
             parameters=[{"suite": "qualifier",
                          "config_file": LaunchConfiguration("config_file"),
                          "lane_geometry_source": LaunchConfiguration("lane_geometry_source"),
                          "lane_debug_draw": LaunchConfiguration("lane_debug_draw"),
                          "require_ego_role": True, "use_sim_time": True}]),
        Node(package="kcity_benchmark", executable="benchmark_hud",
             name="benchmark_hud", output="screen",
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("show_hud"))),
        Node(package="kcity_scenario_manager", executable="traffic_light_perception_stub",
             name="traffic_light_perception_stub", output="screen",
             parameters=[{"config_file": LaunchConfiguration("config_file"),
                          "suite": "qualifier", "publish_rate_hz": 20.0,
                          "use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("enable_traffic_light_stub"))),
    ])
