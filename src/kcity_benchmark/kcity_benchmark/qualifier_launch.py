"""Optional CSV route-test orchestration within the integrated bridge workspace.

HEVEN autonomy is started by heven_carla_adapter/heven_autonomy.launch.py.
This older launch family retains its separate CSV control owner and must run
on its own, rather than alongside that autonomy launch.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from kcity_scenario_manager.qualifier_ownership import acquire
import yaml

# Keep cooperative locks for the launch process lifetime, including shutdown.
_leases = []


def share(package):
    return Path(get_package_share_directory(package))


def include(filename, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        share("kcity_benchmark") / "launch" / filename)),
        launch_arguments=arguments.items())


def gate(phase, timeout=120, allow_stopped_sync=False):
    scenario = share("kcity_scenario_manager")
    return Node(package="kcity_scenario_manager", executable="qualifier_integration_guard",
                name=f"qualifier_integration_{phase}", output="screen", arguments=[
                    "--phase", phase, "--timeout-sec", str(timeout),
                    "--config-file", str(scenario / "config/qualifier.yaml"),
                    "--vehicle-file", str(scenario / "config/qualifier_vehicle.json"),
                    "--route-csv", str(scenario / "routes/qualifier.csv"),
                    *(["--allow-stopped-sync"] if allow_stopped_sync else [])])


def after(process, actions, label):
    def exited(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg=f"Qualifier: {label}"), *actions]
        return [EmitEvent(event=Shutdown(reason=f"Qualifier {label} gate failed ({event.returncode})"))]
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited))


def fatal(process, label):
    return RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=lambda _event, _context: [EmitEvent(event=Shutdown(reason=f"{label} exited"))]))


def validate_mode(mode):
    if mode == "jj_autonomy":
        raise RuntimeError("jj_autonomy uses the integrated entry point: ros2 launch "
                           "heven_carla_adapter heven_autonomy.launch.py. "
                           "This qualifier launch family only starts route_test.")
    if mode != "route_test":
        raise RuntimeError(f"Unsupported control_mode={mode!r}; choose route_test or jj_autonomy")


def platform(policy):
    if policy not in {"fail", "respawn"}:
        raise RuntimeError("existing_ego_policy must be fail or respawn")
    _leases.append(acquire("platform"))
    scenario = share("kcity_scenario_manager")
    preflight = gate("preflight", 15, allow_stopped_sync=policy == "respawn")
    ready, monitor = gate("ready"), gate("platform_monitor")
    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        share("heven_carla_bringup") / "launch/heven_bringup.launch.py")),
        launch_arguments={"vehicle_config": str(scenario / "config/qualifier_vehicle.json"),
                          "sensor_config": str(share("heven_carla_adapter") /
                                               "config/heven_sim_sensors.json"),
                          "launch_rviz": "false"}.items())
    actions = [after(preflight, [base, ready], "platform bringup"),
               after(ready, [monitor], "PLATFORM READY; start evaluation"),
               fatal(monitor, "Platform invariant monitor")]
    if policy == "respawn":
        cleanup = Node(package="kcity_scenario_manager", executable="qualifier_ego_tool",
                       arguments=["destroy", "--platform-preflight"], output="screen")
        actions += [after(cleanup, [preflight], "explicit ego cleanup"), cleanup]
    else:
        actions.append(preflight)
    return actions


def evaluation(show_hud, launch_rviz, lane_geometry_source="", lane_debug_draw="false"):
    _leases.append(acquire("eval"))
    preflight, adapted, armed = gate("eval_preflight"), gate("adapter", 45), gate("armed", 45)
    monitor = gate("eval_monitor")
    with (share("heven_carla_adapter") / "config/adapter.yaml").open(encoding="utf-8") as stream:
        adapter_params = yaml.safe_load(stream)["heven_sensor_adapter"]["ros__parameters"]
    adapter = Node(package="heven_carla_adapter", executable="sensor_adapter",
                   name="heven_carla_sensor_adapter", output="screen",
                   parameters=[adapter_params, {"use_sim_time": True}])
    sidecar = include("qualifier_sidecar.launch.py", show_hud=show_hud,
                      enable_traffic_light_stub="false",
                      lane_geometry_source=lane_geometry_source,
                      lane_debug_draw=lane_debug_draw)
    displays = []
    if launch_rviz.lower() in {"true", "1"}:
        displays.append(Node(package="rviz2", executable="rviz2", name="heven_sensor_rviz",
            arguments=["-d", str(share("heven_carla_bringup") / "config/heven_sensors.rviz")],
            parameters=[{"use_sim_time": True}], output="screen"))
    return [after(preflight, [adapter, adapted], "sensor adapter"),
            after(adapted, [sidecar, armed, *displays], "evaluation sidecar"),
            after(armed, [monitor], "EVALUATION ARMED; start control"),
            fatal(adapter, "Sensor adapter"), fatal(monitor, "Evaluation invariant monitor"), preflight]


def control(mode):
    validate_mode(mode)
    _leases.append(acquire("control"))
    armed, monitor = gate("control", 180), gate("monitor", 15)
    driver = Node(package="kcity_scenario_manager", executable="csv_route_agent",
                  name="csv_route_agent", output="screen", parameters=[{
                      "route_csv": str(share("kcity_scenario_manager") / "routes/qualifier.csv"),
                      "ego_role_name": "ego_vehicle", "expected_map": "heven_kcity/Maps/kcity/kcity",
                      "disable_tm_autopilot": False, "use_sim_time": True}])
    return [after(armed, [driver, monitor], "route_test control"),
            fatal(driver, "CSV route agent"), fatal(monitor, "Control invariant monitor"), armed]
