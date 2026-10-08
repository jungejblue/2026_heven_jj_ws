"""Qualifier terminal role: eval."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kcity_benchmark.qualifier_launch import evaluation


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('show_hud', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('lane_geometry_source', default_value=''),
        DeclareLaunchArgument('lane_debug_draw', default_value='false'),
        OpaqueFunction(function=lambda context: evaluation(
            LaunchConfiguration("show_hud").perform(context),
            LaunchConfiguration("launch_rviz").perform(context),
            LaunchConfiguration("lane_geometry_source").perform(context),
            LaunchConfiguration("lane_debug_draw").perform(context),
        )),
    ])
