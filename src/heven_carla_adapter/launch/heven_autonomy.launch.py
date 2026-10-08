"""Run original HEVEN localization and its 2 m straight-drive initializer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(share, filename, *, scoped=True, **arguments):
    # HEVEN's controller launch and CARLA's spawner both use vehicle_config.
    # Isolate their launch configurations, including delayed spawn actions.
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / filename)),
        launch_arguments=arguments.items(),
    )
    return GroupAction(actions=[include], scoped=True) if scoped else include


def generate_launch_description():
    adapter = Path(get_package_share_directory("heven_carla_adapter"))
    localization = Path(get_package_share_directory("jj_localization"))
    planner = Path(get_package_share_directory("jj_planner"))
    control = Path(get_package_share_directory("jj_control"))
    vehicle = Path(get_package_share_directory("jj_vehicle_driver"))
    scenario = Path(get_package_share_directory("kcity_scenario_manager"))
    course = LaunchConfiguration("course")
    traffic = LaunchConfiguration("enable_traffic")
    localization_config = LaunchConfiguration("localization_config")

    return LaunchDescription([
        DeclareLaunchArgument("path_csv", description="Required HEVEN latitude,longitude route CSV; use the route for this CARLA map."),
        DeclareLaunchArgument("course", default_value="qualifying", choices=["qualifying", "final"]),
        DeclareLaunchArgument("controller", default_value="profile_stanley", choices=[
            "stanley", "pure_pursuit", "profile_stanley", "profile_pure_pursuit"]),
        DeclareLaunchArgument("missions_config", default_value=[str(control / "config" / "missions_"), course, ".yaml"]),
        DeclareLaunchArgument("localization_config", default_value=str(localization / "config" / "localization.yaml")),
        DeclareLaunchArgument("heading_distance_m", default_value="2.0",
                              description="Original Kalman forward-displacement threshold; default is 2 m."),
        DeclareLaunchArgument("controller_config", default_value="", description="Optional HEVEN controller tuning YAML override."),
        DeclareLaunchArgument("heven_vehicle_config", default_value=str(vehicle / "config" / "vehicle.yaml"),
                              description="Original HEVEN driver calibration YAML; separate from CARLA vehicle_config JSON."),
        DeclareLaunchArgument("host", default_value="localhost"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("ego_role_name", default_value="ego_vehicle", choices=["ego_vehicle"]),
        DeclareLaunchArgument("adapter_config", default_value=str(adapter / "config" / "adapter.yaml")),
        DeclareLaunchArgument("vehicle_config", default_value=[str(adapter / "config" / "heven_sim_vehicle_"), course, ".json"]),
        DeclareLaunchArgument("sensor_config", default_value=str(adapter / "config" / "heven_sim_sensors.json")),
        DeclareLaunchArgument("traffic_config", default_value=str(scenario / "config" / "qualifier.yaml")),
        DeclareLaunchArgument("enable_traffic", default_value="false"),
        DeclareLaunchArgument("start_scenario", default_value="false", description="Existing qualifier traffic manager only; leave false for the final course."),
        DeclareLaunchArgument("enable_benchmark", default_value="false", description="Integrated qualifier evaluator; leave false for the final course."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),

        # CARLA TimerAction/OnProcessExit callbacks resolve their launch
        # configurations later. Keep platform defaults in the outer context.
        _include(adapter, "heven_simulation.launch.py", scoped=False, **{
            name: LaunchConfiguration(name) for name in (
                "host", "port", "ego_role_name", "adapter_config", "vehicle_config", "sensor_config",
                "traffic_config", "enable_traffic", "start_scenario", "enable_benchmark", "launch_rviz"
            )
        }),
        # The original HEVEN launch has no use_sim_time argument. Construct
        # its original Kalman node directly so every sensor timestamp uses
        # CARLA /clock, without changing any HEVEN source or launch file.
        Node(package="jj_localization", executable="gnss_imu_kalman_node",
             name="gnss_imu_kalman_localization", namespace="/jj/localization", output="screen",
             parameters=[localization_config, {
                 "use_sim_time": True,
                 "heading_distance_m": ParameterValue(LaunchConfiguration("heading_distance_m"), value_type=float),
                 "heading_update_distance_m": 1.0,
             }],
             remappings=[("imu", "/jj/sensors/imu/data"),
                         ("navpvt", "/jj/sensors/gnss/navpvt"),
                         ("fix", "/jj/sensors/gnss/fix")]),
        _include(planner, "global_path.launch.py",
                 course=course, csv_path=LaunchConfiguration("path_csv"),
                 localization_config=localization_config, use_sim_time="true"),
        _include(control, "controller_variant.launch.py",
                 controller=LaunchConfiguration("controller"), initialize="true", preview="false",
                 config=LaunchConfiguration("controller_config"),
                 vehicle_config=LaunchConfiguration("heven_vehicle_config"),
                 missions_config=LaunchConfiguration("missions_config"),
                 localization_config=localization_config, traffic=traffic,
                 obstacle="false", use_sim_time="true"),
    ])
