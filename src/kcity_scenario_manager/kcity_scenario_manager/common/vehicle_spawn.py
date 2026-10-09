"""Vehicle blueprint and spawn-plan helpers for K-City tools.

The qualifier guard checks the external Bridge spawn configuration against an
upstream START-lane plan built with Waypoint.previous(). Deployed launches let
CARLA ROS Bridge spawn the ego; scenario managers only observe it. Direct
spawn helpers and the nearest-driving-waypoint/explicit-transform modes remain
available for callers outside that launch lifecycle.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

import carla
import yaml

from ..competition_common import TriggerBox, get_trigger
from ..route_csv import read_route_csv, resolve_route_csv_path


@dataclass(frozen=True)
class BeforeStartSpawnConfig:
    back_distance_m: float
    minimum_trigger_clearance_m: float
    topology_step_m: float = 1.0
    heading_tolerance_deg: float = 30.0


@dataclass(frozen=True)
class SpawnPlan:
    transform: carla.Transform
    reason: str
    log_lines: tuple[str, ...] = ()


def angle_difference_deg(first: float, second: float) -> float:
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def parse_before_start_spawn_config(spawn_cfg: Mapping[str, Any]):
    settings = BeforeStartSpawnConfig(
        back_distance_m=float(spawn_cfg.get("back_distance_m", 8.0)),
        minimum_trigger_clearance_m=float(
            spawn_cfg.get("minimum_trigger_clearance_m", 2.0)
        ),
        topology_step_m=float(spawn_cfg.get("topology_step_m", 1.0)),
        heading_tolerance_deg=float(
            spawn_cfg.get("route_heading_tolerance_deg", 30.0)
        ),
    )
    if settings.back_distance_m <= 0.0:
        raise RuntimeError("before_start_trigger back_distance_m must be positive")
    if settings.minimum_trigger_clearance_m < 0.0:
        raise RuntimeError(
            "before_start_trigger minimum_trigger_clearance_m must be non-negative"
        )
    if settings.topology_step_m <= 0.0:
        raise RuntimeError("before_start_trigger topology_step_m must be positive")
    if not 0.0 < settings.heading_tolerance_deg <= 90.0:
        raise RuntimeError(
            "before_start_trigger route_heading_tolerance_deg must be in (0, 90]"
        )
    return settings


def load_suite_config_for_spawn(spawn_cfg, suite_config_file):
    mode = str(spawn_cfg.get("mode", "nearest_driving_waypoint")).strip().lower()
    if mode != "before_start_trigger":
        return None
    if not str(suite_config_file).strip():
        raise RuntimeError(
            "suite_config_file is required for spawn.mode=before_start_trigger"
        )
    path = Path(str(suite_config_file)).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"invalid suite_config_file {path}: {exc}") from exc
    if not isinstance(cfg, Mapping):
        raise RuntimeError(f"invalid suite_config_file {path}: root must be a mapping")
    return dict(cfg)


def trigger_boundary_clearance_m(trigger: TriggerBox, location) -> float:
    """Shortest XY distance from an outside point to an oriented trigger box."""
    local_x, local_y, _ = trigger.local_coords(location)
    outside_x = max(abs(local_x) - trigger.extent_x, 0.0)
    outside_y = max(abs(local_y) - trigger.extent_y, 0.0)
    return math.hypot(outside_x, outside_y)


def _waypoint_identity(waypoint):
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
        round(float(waypoint.s), 4),
    )


def _driving_waypoints(waypoints):
    return [wp for wp in waypoints if wp.lane_type == carla.LaneType.Driving]


def choose_topology_candidate(
    candidates,
    *,
    current_waypoint,
    route_road_id,
    route_lane_id,
    route_heading_deg,
):
    """Choose a branch deterministically, preferring the route lane and heading."""
    driving = _driving_waypoints(candidates)
    if not driving:
        raise RuntimeError("topology traversal found no Driving waypoint candidate")

    def score(waypoint):
        route_lane_rank = 0 if (
            int(waypoint.road_id) == int(route_road_id)
            and int(waypoint.lane_id) == int(route_lane_id)
        ) else 1
        current_lane_rank = 0 if (
            int(waypoint.road_id) == int(current_waypoint.road_id)
            and int(waypoint.lane_id) == int(current_waypoint.lane_id)
        ) else 1
        heading_error = angle_difference_deg(
            waypoint.transform.rotation.yaw, route_heading_deg
        )
        loc = waypoint.transform.location
        return (
            route_lane_rank,
            current_lane_rank,
            heading_error,
            int(waypoint.road_id),
            int(waypoint.section_id),
            int(waypoint.lane_id),
            float(waypoint.s),
            float(loc.x),
            float(loc.y),
        )

    return min(driving, key=score)


def validate_spawn_candidate(
    waypoint,
    start_trigger,
    finish_trigger,
    minimum_trigger_clearance_m,
):
    location = waypoint.transform.location
    if waypoint.lane_type != carla.LaneType.Driving:
        raise RuntimeError("before_start_trigger spawn candidate is not a Driving lane")
    if start_trigger.contains(location):
        raise RuntimeError("before_start_trigger spawn candidate is inside START")
    clearance = trigger_boundary_clearance_m(start_trigger, location)
    if clearance < float(minimum_trigger_clearance_m):
        raise RuntimeError(
            "before_start_trigger spawn candidate START clearance "
            f"{clearance:.3f}m is below {minimum_trigger_clearance_m:.3f}m"
        )
    if finish_trigger.contains(location):
        raise RuntimeError("before_start_trigger spawn candidate is inside FINISH")
    return clearance


def select_upstream_spawn_waypoint(
    start_waypoint,
    *,
    start_trigger,
    finish_trigger,
    route_road_id,
    route_lane_id,
    route_heading_deg,
    settings,
):
    """Walk previous() until distance and trigger-clearance requirements are met."""
    current = start_waypoint
    chain = [start_waypoint]
    distance = 0.0
    visited = {_waypoint_identity(start_waypoint)}
    max_distance = settings.back_distance_m + max(
        50.0, settings.minimum_trigger_clearance_m + 10.0
    )
    last_rejection = "back distance not reached"
    while distance < max_distance:
        chosen = choose_topology_candidate(
            current.previous(settings.topology_step_m),
            current_waypoint=current,
            route_road_id=route_road_id,
            route_lane_id=route_lane_id,
            route_heading_deg=route_heading_deg,
        )
        identity = _waypoint_identity(chosen)
        if identity in visited:
            raise RuntimeError("before_start_trigger previous() traversal formed a loop")
        visited.add(identity)
        step_distance = float(
            current.transform.location.distance(chosen.transform.location)
        )
        if step_distance <= 1e-4:
            raise RuntimeError("before_start_trigger previous() made no progress")
        distance += step_distance
        chain.append(chosen)
        current = chosen
        heading_error = angle_difference_deg(
            chosen.transform.rotation.yaw, route_heading_deg
        )
        if heading_error > settings.heading_tolerance_deg:
            last_rejection = (
                f"heading error {heading_error:.2f}deg exceeds "
                f"{settings.heading_tolerance_deg:.2f}deg"
            )
            continue
        if distance + 1e-6 < settings.back_distance_m:
            last_rejection = (
                f"topology distance {distance:.3f}m is below "
                f"{settings.back_distance_m:.3f}m"
            )
            continue
        try:
            clearance = validate_spawn_candidate(
                chosen,
                start_trigger,
                finish_trigger,
                settings.minimum_trigger_clearance_m,
            )
        except RuntimeError as exc:
            last_rejection = str(exc)
            continue
        return chosen, chain, distance, clearance
    raise RuntimeError(
        "before_start_trigger found no safe upstream waypoint within "
        f"{max_distance:.1f}m: {last_rejection}"
    )


def validate_start_crossing(
    backward_chain,
    *,
    start_trigger,
    route_road_id,
    route_lane_id,
    route_heading_deg,
    settings,
):
    """Confirm candidate -> START has an entry and the configured forward exit."""
    forward = list(reversed(backward_chain))
    for first, second in zip(forward[:-1], forward[1:]):
        successor_ids = {
            _waypoint_identity(candidate)
            for candidate in _driving_waypoints(
                first.next(settings.topology_step_m)
            )
        }
        if _waypoint_identity(second) not in successor_ids:
            raise RuntimeError(
                "previous() spawn chain is not forward-connected to START"
            )
    entry_seen = any(
        not start_trigger.contains(first.transform.location)
        and start_trigger.contains(second.transform.location)
        for first, second in zip(forward[:-1], forward[1:])
    )
    current = forward[-1]
    exit_seen = False
    for _ in range(max(10, int(math.ceil(30.0 / settings.topology_step_m)))):
        chosen = choose_topology_candidate(
            current.next(settings.topology_step_m),
            current_waypoint=current,
            route_road_id=route_road_id,
            route_lane_id=route_lane_id,
            route_heading_deg=route_heading_deg,
        )
        if angle_difference_deg(
            chosen.transform.rotation.yaw, route_heading_deg
        ) > settings.heading_tolerance_deg:
            raise RuntimeError("START forward topology path diverges from route heading")
        forward.append(chosen)
        if start_trigger.exit_crossing_fraction(
            current.transform.location, chosen.transform.location
        ) is not None:
            exit_seen = True
            break
        current = chosen
    if not entry_seen:
        raise RuntimeError("spawn candidate topology path does not enter START trigger")
    if not exit_seen:
        raise RuntimeError(
            "spawn candidate topology path does not cross START configured "
            f"exit_edge={start_trigger.exit_edge}"
        )
    return forward


def waypoint_spawn_transform(waypoint, z_offset):
    source = waypoint.transform
    return carla.Transform(
        carla.Location(
            x=source.location.x,
            y=source.location.y,
            z=source.location.z + float(z_offset),
        ),
        carla.Rotation(
            pitch=source.rotation.pitch,
            yaw=source.rotation.yaw,
            roll=source.rotation.roll,
        ),
    )


def _route_start_reference(suite_cfg, suite_config_file, start_waypoint):
    benchmark = suite_cfg.get("benchmark", {})
    configured = str(benchmark.get("route_csv", "")).strip()
    start_transform = start_waypoint.transform
    if not configured:
        return (
            int(start_waypoint.road_id),
            int(start_waypoint.lane_id),
            float(start_transform.rotation.yaw),
            "suite has no benchmark.route_csv; START waypoint heading used",
        )
    route_path = resolve_route_csv_path(suite_config_file, configured)
    if not route_path.exists():
        raise RuntimeError(f"suite benchmark.route_csv not found: {route_path}")
    first = read_route_csv(route_path)[0]
    start_road = int(start_waypoint.road_id)
    start_lane = int(start_waypoint.lane_id)
    if (int(first["road_id"]), int(first["lane_id"])) != (start_road, start_lane):
        raise RuntimeError(
            "START projected waypoint disagrees with route CSV first lane: "
            f"projected=road {start_road} lane {start_lane}, "
            f"csv=road {first['road_id']} lane {first['lane_id']}"
        )
    csv_location = carla.Location(
        x=float(first["x"]), y=float(first["y"]), z=float(first["z"])
    )
    distance = float(start_transform.location.distance(csv_location))
    if distance > 2.0:
        raise RuntimeError(
            "START projected waypoint is too far from route CSV first point: "
            f"{distance:.3f}m"
        )
    return (
        int(first["road_id"]),
        int(first["lane_id"]),
        float(first["yaw"]),
        f"route_csv={route_path}, first-point distance={distance:.3f}m",
    )


def make_before_start_spawn_plan(world, spawn_cfg, suite_cfg, suite_config_file):
    settings = parse_before_start_spawn_config(spawn_cfg)
    if suite_cfg is None:
        raise RuntimeError(
            "suite_config_file is required for spawn.mode=before_start_trigger"
        )
    start_trigger = get_trigger(suite_cfg, "START", require_exit_edge=True)
    finish_trigger = get_trigger(suite_cfg, "FINISH")
    if not start_trigger.enabled:
        raise RuntimeError("START trigger must be enabled for before_start_trigger spawn")
    if not finish_trigger.enabled:
        raise RuntimeError("FINISH trigger must be enabled for before_start_trigger spawn")
    start_location = carla.Location(
        x=start_trigger.center_x,
        y=start_trigger.center_y,
        z=start_trigger.center_z,
    )
    start_waypoint = world.get_map().get_waypoint(
        start_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if start_waypoint is None:
        raise RuntimeError("START trigger center did not project to a Driving waypoint")
    route_road, route_lane, route_heading, route_detail = _route_start_reference(
        suite_cfg, suite_config_file, start_waypoint
    )
    start_heading_error = angle_difference_deg(
        start_waypoint.transform.rotation.yaw, route_heading
    )
    if start_heading_error > settings.heading_tolerance_deg:
        raise RuntimeError(
            "START projected waypoint yaw disagrees with route CSV heading: "
            f"error={start_heading_error:.2f}deg"
        )
    selected, backward_chain, distance, clearance = select_upstream_spawn_waypoint(
        start_waypoint,
        start_trigger=start_trigger,
        finish_trigger=finish_trigger,
        route_road_id=route_road,
        route_lane_id=route_lane,
        route_heading_deg=route_heading,
        settings=settings,
    )
    validate_start_crossing(
        backward_chain,
        start_trigger=start_trigger,
        route_road_id=route_road,
        route_lane_id=route_lane,
        route_heading_deg=route_heading,
        settings=settings,
    )
    transform = waypoint_spawn_transform(
        selected, float(spawn_cfg.get("z_offset", 0.5))
    )
    start_tf = start_waypoint.transform
    selected_tf = selected.transform
    log_lines = (
        "START projected waypoint: "
        f"road={start_waypoint.road_id} lane={start_waypoint.lane_id} "
        f"loc=({start_tf.location.x:.3f}, {start_tf.location.y:.3f}, "
        f"{start_tf.location.z:.3f}) yaw={start_tf.rotation.yaw:.3f}",
        f"START route consistency: {route_detail}; heading_error={start_heading_error:.3f}deg",
        "selected spawn waypoint: "
        f"road={selected.road_id} lane={selected.lane_id} "
        f"loc=({selected_tf.location.x:.3f}, {selected_tf.location.y:.3f}, "
        f"{selected_tf.location.z:.3f}) yaw={selected_tf.rotation.yaw:.3f} "
        f"distance_to_start={distance:.3f}m "
        f"START_clearance={clearance:.3f}m inside_START=false inside_FINISH=false",
        "START crossing validation: "
        f"entry=PASS exit=PASS expected_exit_edge={start_trigger.exit_edge}; "
        "selection=same route road/lane then minimum heading error",
    )
    return SpawnPlan(
        transform=transform,
        reason=(
            "before_start_trigger "
            f"road={selected.road_id} lane={selected.lane_id} "
            f"distance_to_start={distance:.3f}m "
            f"exit_edge={start_trigger.exit_edge}"
        ),
        log_lines=log_lines,
    )


def blueprint_candidates(
    world: carla.World,
    *,
    exact_id: str = "",
    contains: str = "heven",
):
    """spawn 가능한 blueprint 후보를 우선순위 순으로 반환한다."""
    library = world.get_blueprint_library()

    exact_id = str(exact_id or "").strip()
    token = str(contains or "").strip().lower()

    result = []

    if exact_id:
        try:
            result.append(library.find(exact_id))
        except (IndexError, RuntimeError):
            pass

    if token:
        try:
            matches = list(library.filter(f"*{token}*"))
        except RuntimeError:
            matches = []

        for bp in matches:
            if all(bp.id != existing.id for existing in result):
                result.append(bp)

    return result


def choose_blueprint(
    world: carla.World,
    *,
    exact_id: str = "",
    contains: str = "heven",
):
    """첫 번째 적합 blueprint를 반환한다."""
    candidates = blueprint_candidates(
        world,
        exact_id=exact_id,
        contains=contains,
    )
    return candidates[0] if candidates else None


def blueprint_ids(world: carla.World, contains: str = ""):
    """디버깅용 blueprint id 목록."""
    library = world.get_blueprint_library()
    token = str(contains or "").strip().lower()

    try:
        all_bps = list(library)
    except TypeError:
        try:
            all_bps = list(library.filter("*"))
        except RuntimeError:
            all_bps = []

    ids = [bp.id for bp in all_bps]
    if token:
        ids = [bp_id for bp_id in ids if token in bp_id.lower()]

    return sorted(ids)


def make_spawn_transform(
    world: carla.World,
    spawn_cfg: Dict[str, Any],
    *,
    suite_cfg=None,
    suite_config_file="",
) -> Tuple[carla.Transform, str]:
    """YAML spawn 설정을 CARLA Transform으로 변환한다.

    mode=nearest_driving_waypoint:
      seed_location을 가장 가까운 driving lane에 projection.
      따라서 사용자가 yaw를 일일이 맞출 필요가 없다.

    mode=transform:
      x/y/z/yaw/pitch/roll을 그대로 사용.
    """
    mode = str(
        spawn_cfg.get("mode", "nearest_driving_waypoint")
    ).strip().lower()

    if mode == "before_start_trigger":
        plan = make_before_start_spawn_plan(
            world, spawn_cfg, suite_cfg, suite_config_file
        )
        return plan.transform, plan.reason

    if mode == "nearest_driving_waypoint":
        seed = spawn_cfg.get("seed_location", {})
        seed_location = carla.Location(
            x=float(seed.get("x", 0.0)),
            y=float(seed.get("y", 0.0)),
            z=float(seed.get("z", 0.0)),
        )

        waypoint = world.get_map().get_waypoint(
            seed_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

        if waypoint is None:
            raise RuntimeError(
                "spawn seed 주변에서 Driving waypoint를 찾지 못했습니다: "
                f"({seed_location.x:.2f}, {seed_location.y:.2f}, "
                f"{seed_location.z:.2f})"
            )

        transform = waypoint.transform
        z_offset = float(spawn_cfg.get("z_offset", 0.5))

        # waypoint.transform 객체를 직접 수정하지 않고 새 Transform 생성.
        transform = carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z + z_offset,
            ),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll,
            ),
        )

        return (
            transform,
            (
                "nearest_driving_waypoint "
                f"seed=({seed_location.x:.2f},"
                f"{seed_location.y:.2f},{seed_location.z:.2f})"
            ),
        )

    if mode == "transform":
        transform = carla.Transform(
            carla.Location(
                x=float(spawn_cfg.get("x", 0.0)),
                y=float(spawn_cfg.get("y", 0.0)),
                z=float(spawn_cfg.get("z", 0.5)),
            ),
            carla.Rotation(
                pitch=float(spawn_cfg.get("pitch", 0.0)),
                yaw=float(spawn_cfg.get("yaw", 0.0)),
                roll=float(spawn_cfg.get("roll", 0.0)),
            ),
        )
        return transform, "explicit transform"

    raise ValueError(
        "ego.spawn.mode는 "
        "'before_start_trigger', 'nearest_driving_waypoint' 또는 "
        "'transform'이어야 합니다: "
        f"{mode}"
    )


def configure_blueprint(
    blueprint,
    *,
    role_name: str = "ego_vehicle",
):
    """spawn 전 role_name 등 안전한 attribute만 설정한다."""
    role_name = str(role_name or "").strip()

    if (
        role_name
        and hasattr(blueprint, "has_attribute")
        and blueprint.has_attribute("role_name")
    ):
        blueprint.set_attribute("role_name", role_name)

    return blueprint


def spawn_vehicle(
    world: carla.World,
    *,
    exact_blueprint_id: str = "",
    blueprint_contains: str = "heven",
    role_name: str = "ego_vehicle",
    spawn_cfg: Optional[Dict[str, Any]] = None,
    suite_cfg=None,
    suite_config_file="",
    log_info=None,
):
    """BP_HEVEN 계열 vehicle actor를 spawn한다.

    collision 때문에 첫 시도가 실패하면 z를 조금씩 높여 몇 차례 재시도한다.
    """
    spawn_cfg = dict(spawn_cfg or {})

    blueprint = choose_blueprint(
        world,
        exact_id=exact_blueprint_id,
        contains=blueprint_contains,
    )

    if blueprint is None:
        raise RuntimeError(
            "spawn 가능한 BP_HEVEN blueprint를 찾지 못했습니다. "
            f"exact_id='{exact_blueprint_id}', "
            f"contains='{blueprint_contains}'. "
            "BP_HEVEN이 CARLA Blueprint Library에 vehicle blueprint로 "
            "등록되어 있어야 터미널 spawn이 가능합니다."
        )

    configure_blueprint(
        blueprint,
        role_name=role_name,
    )

    mode = str(spawn_cfg.get("mode", "nearest_driving_waypoint")).strip().lower()
    if mode == "before_start_trigger":
        plan = make_before_start_spawn_plan(
            world, spawn_cfg, suite_cfg, suite_config_file
        )
        base_transform, reason = plan.transform, plan.reason
        if log_info is not None:
            for line in plan.log_lines:
                log_info(line)
    else:
        base_transform, reason = make_spawn_transform(world, spawn_cfg)

    retry_z_offsets = spawn_cfg.get(
        "retry_z_offsets",
        [0.0, 0.5, 1.0, 1.5],
    )

    last_transform = None

    for extra_z in retry_z_offsets:
        transform = carla.Transform(
            carla.Location(
                x=base_transform.location.x,
                y=base_transform.location.y,
                z=base_transform.location.z + float(extra_z),
            ),
            carla.Rotation(
                pitch=base_transform.rotation.pitch,
                yaw=base_transform.rotation.yaw,
                roll=base_transform.rotation.roll,
            ),
        )
        last_transform = transform

        actor = world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            try:
                actor.set_simulate_physics(True)
            except (AttributeError, RuntimeError):
                pass

            return actor, blueprint.id, transform, reason

    raise RuntimeError(
        "BP_HEVEN blueprint는 찾았지만 actor spawn에 실패했습니다. "
        "주변에 다른 actor/벽이 겹쳤거나 seed 위치가 부적절할 수 있습니다. "
        f"blueprint={blueprint.id}, "
        f"last=({last_transform.location.x:.2f}, "
        f"{last_transform.location.y:.2f}, "
        f"{last_transform.location.z:.2f})"
    )
