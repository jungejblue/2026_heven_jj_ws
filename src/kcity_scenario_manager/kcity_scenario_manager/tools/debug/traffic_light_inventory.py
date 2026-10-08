"""CARLA K-City 신호등 actor inventory 도구.

ROS topic을 사용하지 않고 CARLA Python API에 직접 연결한다.

사용:
    ros2 run kcity_scenario_manager traffic_light_inventory

Coordinates can be compared with qualifier.yaml/final.yaml traffic-light locations.
"""

from __future__ import annotations

import argparse
import time

import carla


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--control-test",
        action="store_true",
        help="첫 번째 신호를 Red/Yellow/Green으로 직접 변경",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()

    print("Map:", world.get_map().name)

    lights = list(
        world.get_actors().filter("traffic.traffic_light*")
    )
    print("TrafficLight 수:", len(lights))

    if not lights:
        raise RuntimeError(
            "traffic.traffic_light actor가 0개입니다. "
            "Unreal에서 BP_TrafficLight 배치/Play 상태를 확인하세요."
        )

    for light in sorted(lights, key=lambda actor: actor.id):
        transform = light.get_transform()
        location = transform.location

        print(
            f"id={light.id} "
            f"| pole={light.get_pole_index()} "
            f"| state={light.get_state()} "
            f"| opendrive_id={light.get_opendrive_id()} "
            f"| loc=({location.x:.3f}, "
            f"{location.y:.3f}, {location.z:.3f}) "
            f"| group={[x.id for x in light.get_group_traffic_lights()]} "
            f"| affected_lanes="
            f"{len(light.get_affected_lane_waypoints())} "
            f"| stop_points={len(light.get_stop_waypoints())}"
        )

    if not args.control_test:
        return

    # 주의:
    # 이 도구에서도 freeze(True)는 사용하지 않는다.
    # 현재 CARLA checkout에서는 전역 동결이기 때문이다.
    light = lights[0]

    for state in (
        carla.TrafficLightState.Red,
        carla.TrafficLightState.Yellow,
        carla.TrafficLightState.Green,
    ):
        light.set_state(state)
        print(
            f"set={state} / server={light.get_state()}"
        )
        time.sleep(3.0)


if __name__ == "__main__":
    main()
