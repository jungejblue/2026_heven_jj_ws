"""Qualifier terminal role: control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kcity_benchmark.qualifier_launch import control


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='route_test'),
        OpaqueFunction(function=lambda context: control(LaunchConfiguration("control_mode").perform(context))),
    ])
