"""CARLA platform, HEVEN sensor/control adapters and simulation-only mounts."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    adapter = Path(get_package_share_directory("heven_carla_adapter"))
    bringup = Path(get_package_share_directory("heven_carla_bringup"))
    scenario = Path(get_package_share_directory("kcity_scenario_manager"))
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    role = LaunchConfiguration("ego_role_name")
    config = LaunchConfiguration("adapter_config")
    traffic_config = LaunchConfiguration("traffic_config")
    enable_traffic = LaunchConfiguration("enable_traffic")
    robot_description = (adapter / "urdf" / "heven_sim_vehicle.urdf").read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="localhost"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("ego_role_name", default_value="ego_vehicle", choices=["ego_vehicle"],
                              description="The full profile uses the existing ego_vehicle readiness and spawn topics."),
        DeclareLaunchArgument("adapter_config", default_value=str(adapter / "config" / "adapter.yaml")),
        DeclareLaunchArgument("vehicle_config", default_value=str(adapter / "config" / "heven_sim_vehicle_qualifying.json")),
        DeclareLaunchArgument("sensor_config", default_value=str(adapter / "config" / "heven_sim_sensors.json")),
        DeclareLaunchArgument("traffic_config", default_value=str(scenario / "config" / "qualifier.yaml"),
                              description="Canonical trigger/light configuration shared by the observer and scenario manager."),
        DeclareLaunchArgument("enable_traffic", default_value="false",
                              description="Publish CARLA traffic state for HEVEN; enable after inspecting the regions."),
        DeclareLaunchArgument("start_scenario", default_value="false",
                              description="Start the integrated qualifier scenario manager."),
        DeclareLaunchArgument("enable_benchmark", default_value="false",
                              description="Start the integrated qualifier evaluator without a CSV route controller."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("startup_delay", default_value="2.0"),
        DeclareLaunchArgument("warmup_seconds", default_value="2.0"),
        DeclareLaunchArgument("discard_complete_sets", default_value="5"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch" / "heven_bringup.launch.py")),
            launch_arguments={
                "vehicle_config": LaunchConfiguration("vehicle_config"),
                "sensor_config": LaunchConfiguration("sensor_config"),
                "host": host, "port": port,
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "startup_delay": LaunchConfiguration("startup_delay"),
                "warmup_seconds": LaunchConfiguration("warmup_seconds"),
                "discard_complete_sets": LaunchConfiguration("discard_complete_sets"),
            }.items(),
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="heven_sim_robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="heven_carla_adapter", executable="sensor_adapter",
             name="heven_sensor_adapter", output="screen",
             parameters=[config, {
                 "use_sim_time": True,
                 "host": ParameterValue(host, value_type=str),
                 "port": ParameterValue(port, value_type=int),
                 "ego_role_name": ParameterValue(role, value_type=str),
             }]),
        Node(package="heven_carla_adapter", executable="control_adapter",
             name="heven_control_adapter", output="screen",
             parameters=[config, {
                 "use_sim_time": True,
                 "host": ParameterValue(host, value_type=str),
                 "port": ParameterValue(port, value_type=int),
                 "role_name": ParameterValue(role, value_type=str),
             }]),
        # Keep this node present while publishing is off so the accepted
        # detection boxes can still be drawn and edited in CARLA.
        Node(package="heven_carla_adapter", executable="traffic_light_adapter",
             name="heven_traffic_light_adapter", output="screen",
             parameters=[config, {
                 "use_sim_time": True,
                 "config_file": ParameterValue(traffic_config, value_type=str),
                 "publish_enabled": ParameterValue(enable_traffic, value_type=bool),
                 "carla_host": ParameterValue(host, value_type=str),
                 "carla_port": ParameterValue(port, value_type=int),
                 "ego_role_name": ParameterValue(role, value_type=str),
             }]),
        # This manager only changes traffic-light states. The CARLA CSV route
        # driver and the canonical route_test controller are never started.
        Node(package="kcity_scenario_manager", executable="qualifier_scenario_manager",
             name="qualifier_scenario_manager", output="screen",
             parameters=[{"config_file": ParameterValue(traffic_config, value_type=str),
                          "require_ego_role": True, "use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("start_scenario"))),
        Node(package="kcity_benchmark", executable="benchmark_runner",
             name="kcity_qualifier_benchmark", output="screen",
             parameters=[{"suite": "qualifier",
                          "config_file": ParameterValue(traffic_config, value_type=str),
                          "require_ego_role": True, "use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("enable_benchmark"))),
    ])
