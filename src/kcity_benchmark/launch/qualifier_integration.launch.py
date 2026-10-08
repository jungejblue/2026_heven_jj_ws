"""Preserved all-in-one entry point, composed from the three terminal roles."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from kcity_benchmark.qualifier_launch import after, gate, include, validate_mode


def start(context):
    mode = LaunchConfiguration("control_mode").perform(context)
    validate_mode(mode)
    ready = gate("ready")
    armed = gate("control", 180)
    evaluation = include("qualifier_eval.launch.py",
        launch_rviz=LaunchConfiguration("launch_rviz").perform(context),
        show_hud=LaunchConfiguration("show_hud").perform(context))
    controller = include("qualifier_control.launch.py", control_mode=mode)
    return [after(ready, [evaluation, armed], "evaluation"),
            after(armed, [controller], "controller"),
            include("qualifier_platform.launch.py",
                existing_ego_policy=LaunchConfiguration("existing_ego_policy").perform(context)),
            ready]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="route_test"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("show_hud", default_value="false"),
        DeclareLaunchArgument("existing_ego_policy", default_value="fail"),
        OpaqueFunction(function=start),
    ])
