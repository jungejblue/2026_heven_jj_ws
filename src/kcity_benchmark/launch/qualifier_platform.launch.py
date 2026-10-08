"""Qualifier terminal role: platform."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kcity_benchmark.qualifier_launch import platform


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('existing_ego_policy', default_value='fail'),
        OpaqueFunction(function=lambda context: platform(LaunchConfiguration("existing_ego_policy").perform(context))),
    ])
