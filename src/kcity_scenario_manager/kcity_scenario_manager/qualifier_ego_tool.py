"""Explicit ego maintenance. Never spawns actors or advances the world.

Respawn delegates the whole platform lifecycle to the existing generic bringup.
Destroy/respawn require Terminal 2 stopped; reset requires Terminals 3/4 stopped.
"""
import argparse
import json
import os
from pathlib import Path
import time

import carla
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from .qualifier_ownership import acquire
from .qualifier_integration_guard import CONTROL_TOPICS, EVALUATION_NODES, OTHER_CONTROL_NODES


def egos_in(actors):
    return [a for a in actors if a.type_id.startswith("vehicle.")
            and a.attributes.get("role_name") == "ego_vehicle"]


def attached_sensors(actors, egos):
    ids = {a.id for a in egos}
    return [a for a in actors if a.type_id.startswith("sensor.")
            and getattr(getattr(a, "parent", None), "id", None) in ids]


def start_transform(vehicle_file):
    objects = json.loads(Path(vehicle_file).read_text())["objects"]
    if len(objects) != 1 or objects[0].get("id") != "ego_vehicle":
        raise RuntimeError("Expected exactly one ego_vehicle in vehicle configuration")
    p = objects[0]["spawn_point"]
    # Same ROS right-handed -> CARLA conversion used by carla_spawn_objects/Bridge.
    return carla.Transform(carla.Location(x=p["x"], y=-p["y"], z=p["z"]),
                           carla.Rotation(roll=p["roll"], pitch=-p["pitch"], yaw=-p["yaw"]))


def check_idle(node, require_platform_stopped):
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    names = {name for name, _ in node.get_node_names_and_namespaces()}
    forbidden = {*EVALUATION_NODES, *OTHER_CONTROL_NODES, "csv_route_agent", "stanley"}
    if require_platform_stopped:
        forbidden |= {"carla_ros_bridge", "heven_vehicle_spawner", "heven_sensor_spawner"}
    found = names & forbidden
    topics = [t for t in CONTROL_TOPICS if node.count_publishers(t)]
    if require_platform_stopped and node.count_publishers("/clock"):
        topics.append("/clock")
    if found or topics:
        raise RuntimeError(f"Stop owning terminals before maintenance: nodes={sorted(found)}, topics={topics}")


def destroy_selected(actors, egos):
    for actor in [*attached_sensors(actors, egos), *egos]:
        if actor.is_alive and not actor.destroy():
            raise RuntimeError(f"Failed to destroy actor {actor.id}; refusing further cleanup")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "reset_start", "destroy", "respawn"))
    parser.add_argument("--platform-preflight", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.platform_preflight and args.action != "destroy":
        parser.error("internal platform preflight only supports destroy")
    if args.action == "respawn":
        # Stays in the foreground as the new Terminal 2. No duplicate spawn implementation.
        os.execvp("ros2", ["ros2", "launch", "kcity_benchmark", "qualifier_platform.launch.py",
                           "existing_ego_policy:=respawn"])
    leases = []
    node = None
    try:
        if args.action != "status":
            leases.extend([acquire("control"), acquire("eval")])
            if args.action == "destroy" and not args.platform_preflight:
                leases.append(acquire("platform"))
            rclpy.init()
            node = Node("qualifier_ego_tool")
            check_idle(node, require_platform_stopped=args.action == "destroy")
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        actors = list(world.get_actors())
        egos = egos_in(actors)
        if args.action == "status":
            print(f"ego_vehicle count={len(egos)}")
            for ego in egos:
                print(f"id={ego.id} type={ego.type_id} transform={ego.get_transform()} "
                      f"attached_sensors={[s.id for s in attached_sensors(actors, [ego])]}")
        elif args.action == "destroy":
            destroy_selected(actors, egos)
            print(f"Destroyed only ego_vehicle IDs {[e.id for e in egos]} and their attached sensors")
        else:
            if len(egos) != 1:
                raise RuntimeError(f"reset_start requires exactly one ego_vehicle, found {len(egos)}")
            from .qualifier_integration_guard import EXPECTED_MAP, vehicle_config_error
            if world.get_map().name != EXPECTED_MAP:
                raise RuntimeError(f"reset_start requires map {EXPECTED_MAP}")
            share = Path(get_package_share_directory("kcity_scenario_manager"))
            vehicle = share / "config/qualifier_vehicle.json"
            problem = vehicle_config_error(vehicle, str(share / "config/qualifier.yaml"), world)
            if problem:
                raise RuntimeError(problem)
            ego = egos[0]
            ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0,
                                                  hand_brake=True, reverse=False))
            ego.set_target_velocity(carla.Vector3D())
            ego.set_target_angular_velocity(carla.Vector3D())
            ego.set_transform(start_transform(vehicle))
            print(f"Reset ego {ego.id}; sensors preserved; brake held. Restart Terminal 3 then 4.")
    except (RuntimeError, OSError, ValueError) as exc:
        parser.exit(1, f"qualifier_ego_tool: {exc}\n")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for lease in leases:
            lease.close()


if __name__ == "__main__":
    main()
