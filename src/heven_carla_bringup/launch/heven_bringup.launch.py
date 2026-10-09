"""Bring up CARLA bridge, HEVEN vehicle, delayed sensors, gate, and RViz."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("heven_carla_bringup"))

    bridge_config = str(package_share / "config" / "bridge.yaml")
    default_vehicle_config = str(package_share / "config" / "vehicle_only.json")
    default_sensor_config = str(package_share / "config" / "heven_sensors.json")
    default_rviz_config = str(package_share / "config" / "heven_sensors.rviz")

    launch_rviz = LaunchConfiguration("launch_rviz")
    startup_delay = LaunchConfiguration("startup_delay")
    warmup_seconds = LaunchConfiguration("warmup_seconds")
    discard_complete_sets = LaunchConfiguration("discard_complete_sets")
    sensor_config = LaunchConfiguration("sensor_config")
    vehicle_config = LaunchConfiguration("vehicle_config")
    rviz_config = LaunchConfiguration("rviz_config")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    bridge = Node(
        package="carla_ros_bridge",
        executable="bridge",
        name="carla_ros_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[bridge_config, {
            "host": ParameterValue(host, value_type=str),
            "port": ParameterValue(port, value_type=int),
        }],
        on_exit=EmitEvent(
            event=Shutdown(reason="CARLA ROS Bridge exited")
        ),
    )

    # Start the spawner executable directly. The ttgamage Humble fork installs
    # its launch file in share/carla_spawn_objects/ (without a launch/
    # directory), so including a hard-coded upstream launch path is brittle.
    vehicle_spawner = Node(
        package="carla_spawn_objects",
        executable="carla_spawn_objects",
        name="heven_vehicle_spawner",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"objects_definition_file": vehicle_config},
            {"spawn_sensors_only": False},
        ],
    )

    warmup_guard = Node(
        package="heven_carla_bringup",
        executable="heven_warmup_guard",
        name="heven_warmup_guard",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"warmup_seconds": ParameterValue(warmup_seconds, value_type=float)},
        ],
    )

    sensor_spawner = Node(
        package="carla_spawn_objects",
        executable="carla_spawn_objects",
        name="heven_sensor_spawner",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"objects_definition_file": sensor_config},
            {"spawn_sensors_only": True},
        ],
    )

    def on_warmup_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg="Vehicle warm-up passed; attaching HEVEN sensors."),
                sensor_spawner,
            ]
        return [
            # ROS 2 Humble exposes LogInfo but not LogError from
            # launch.actions. Prefix the message and then emit a Shutdown
            # event so this failure path remains explicit and portable.
            LogInfo(
                msg=(
                    "[ERROR] Vehicle warm-up failed with return code "
                    f"{event.returncode}; sensor spawning is cancelled."
                )
            ),
            EmitEvent(event=Shutdown(reason="HEVEN vehicle warm-up failed")),
        ]

    start_vehicle_phase = TimerAction(
        period=startup_delay,
        actions=[
            LogInfo(msg="Starting HEVEN vehicle spawn and warm-up phase."),
            vehicle_spawner,
            warmup_guard,
        ],
    )

    sensor_gate = Node(
        package="heven_carla_bringup",
        executable="heven_sensor_gate",
        name="heven_sensor_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "discard_complete_sets": ParameterValue(
                    discard_complete_sets,
                    value_type=int,
                )
            },
        ],
    )

    readiness_monitor = Node(
        package="heven_carla_bringup",
        executable="heven_readiness_monitor",
        name="heven_readiness_monitor",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="heven_sensor_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=default_vehicle_config,
                description="CARLA vehicle object JSON; the default preserves the original spawn pose.",
            ),
            DeclareLaunchArgument(
                "sensor_config",
                default_value=default_sensor_config,
                description="CARLA sensor object JSON; the default preserves the original bringup.",
            ),
            DeclareLaunchArgument("host", default_value="localhost"),
            DeclareLaunchArgument("port", default_value="2000"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz display profile; the default shows raw CARLA sensors.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz with the HEVEN sensor configuration.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="2.0",
                description="Wall-time delay for bridge service startup.",
            ),
            DeclareLaunchArgument(
                "warmup_seconds",
                default_value="2.0",
                description="Vehicle stabilization time measured using /clock.",
            ),
            DeclareLaunchArgument(
                "discard_complete_sets",
                default_value="5",
                description="Complete equal-stamp sets to ignore before ready.",
            ),
            bridge,
            sensor_gate,
            readiness_monitor,
            rviz,
            start_vehicle_phase,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=warmup_guard,
                    on_exit=on_warmup_exit,
                )
            ),
        ]
    )

