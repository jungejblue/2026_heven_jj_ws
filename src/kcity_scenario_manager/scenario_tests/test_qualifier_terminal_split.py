"""Ownership, maintenance targeting and stage invariants without a CARLA server."""
from pathlib import Path
from types import SimpleNamespace as NS
import time

import pytest

from kcity_scenario_manager.qualifier_ego_tool import (
    attached_sensors, destroy_selected, egos_in, start_transform,
)
from kcity_scenario_manager.qualifier_integration_guard import (
    IntegrationGuard, EXPECTED_MAP, EXPECTED_SENSORS, RAW_TOPICS, JJ_TOPICS, EVALUATION_TOPICS,
)
from kcity_scenario_manager.qualifier_ownership import acquire


def actor(id, kind, role, parent=None):
    result = NS(id=id, type_id=kind, attributes={"role_name": role}, parent=parent,
                is_alive=True, destroyed=False)
    def destroy():
        result.destroyed = True
        return True
    result.destroy = destroy
    return result


def test_destroy_only_exact_role_and_its_attached_sensors():
    ego = actor(1, "vehicle.heven.ev", "ego_vehicle")
    other = actor(2, "vehicle.heven.ev", "ego_vehicle_extra")
    sensor = actor(3, "sensor.other.imu", "imu", ego)
    other_sensor = actor(4, "sensor.other.imu", "imu", other)
    prop = actor(5, "static.prop", "ego_vehicle", ego)
    actors = [ego, other, sensor, other_sensor, prop]
    assert egos_in(actors) == [ego]
    assert attached_sensors(actors, [ego]) == [sensor]
    destroy_selected(actors, egos_in(actors))
    assert [a.id for a in actors if a.destroyed] == [1, 3]


def test_start_pose_uses_bridge_coordinate_conversion():
    path = Path(__file__).parents[1] / "config/qualifier_vehicle.json"
    transform = start_transform(path)
    assert transform.location.x == pytest.approx(-44.487778)
    assert transform.location.y == pytest.approx(403.239105)
    assert transform.rotation.yaw == pytest.approx(-62.344051)


def test_role_lease_rejects_overlap_and_releases():
    first = acquire("test-only")
    try:
        with pytest.raises(RuntimeError, match="already running"):
            acquire("test-only")
    finally:
        first.close()
    acquire("test-only").close()


def guard_fixture(phase):
    ego = actor(1, "vehicle.heven.ev", "ego_vehicle")
    actors = [ego] + [actor(i+2, kind, role, ego)
                     for i, (role, kind) in enumerate(EXPECTED_SENSORS.items())]
    nodes = {"carla_ros_bridge": 1}
    publishers = {topic: 1 for topic in (*RAW_TOPICS, "/clock", "/heven/sensors_ready")}
    now = time.monotonic()
    guard = NS(phase=phase, ready=True, clock_stamps=[1, 2], last_clock_monotonic=now,
               raw_seen={topic: now for topic in RAW_TOPICS},
               jj_last_seen={topic: now for topic in JJ_TOPICS}, benchmark_status="armed",
               _publisher_count=lambda t: publishers.get(t, 0),
               _node_count=lambda n: nodes.get(n, 0),
               world=NS(get_map=lambda: NS(name=EXPECTED_MAP), get_actors=lambda: actors,
                        get_settings=lambda: NS(synchronous_mode=True, fixed_delta_seconds=0.05)))
    return guard, actors, nodes, publishers


def test_evaluation_gate_rejects_duplicate_adapter_and_active_controller():
    guard, _, nodes, _ = guard_fixture("eval_preflight")
    assert IntegrationGuard.check(guard) == ""
    nodes["heven_carla_sensor_adapter"] = 1
    assert "already running" in IntegrationGuard.check(guard)
    nodes.pop("heven_carla_sensor_adapter")
    nodes["csv_route_agent"] = 1
    assert "stop Terminal 4" in IntegrationGuard.check(guard)


def test_topic_presence_is_not_sensor_liveness():
    guard, _, _, _ = guard_fixture("ready")
    guard.raw_seen[RAW_TOPICS[0]] -= 6
    assert "fresh CARLA sensor samples" in IntegrationGuard.check(guard)


def test_platform_survives_eval_lifecycle_but_rejects_duplicate_sensor_or_jj_control():
    guard, actors, _, publishers = guard_fixture("platform_monitor")
    assert IntegrationGuard.check(guard) == ""
    actors.append(actor(99, "sensor.other.collision", "collision", actors[0]))
    assert IntegrationGuard.check(guard) == ""
    actors.append(actor(100, "sensor.other.collision", "collision", actors[0]))
    assert "mismatch" in IntegrationGuard.check(guard)
    actors.pop()
    actors.pop()
    assert IntegrationGuard.check(guard) == ""
    publishers["/jj/drive/command"] = 1
    assert "control publisher" in IntegrationGuard.check(guard)


def test_controller_requires_armed_benchmark_and_single_adapter():
    guard, actors, nodes, publishers = guard_fixture("control")
    actors.append(actor(99, "sensor.other.collision", "collision", actors[0]))
    nodes.update(heven_carla_sensor_adapter=1, qualifier_scenario_manager=1,
                 kcity_qualifier_benchmark=1)
    publishers.update({topic: 1 for topic in JJ_TOPICS})
    publishers.update({topic: 1 for topic in EVALUATION_TOPICS})
    assert IntegrationGuard.check(guard) == ""
    guard.benchmark_status = "finished"
    assert "armed state" in IntegrationGuard.check(guard)
    guard.benchmark_status = "armed"
    nodes["csv_route_agent"] = 1
    assert "stop Terminal 4" in IntegrationGuard.check(guard)


def test_respawn_sync_exception_does_not_bypass_existing_owner(monkeypatch):
    from kcity_scenario_manager import qualifier_integration_guard as module
    guard, actors, nodes, publishers = guard_fixture("preflight")
    guard.allow_stopped_sync = False
    assert "already synchronous" in IntegrationGuard.check(guard)
    guard.allow_stopped_sync = True
    actors.clear()
    assert "Bridge or /clock" in IntegrationGuard.check(guard)
    nodes.clear()
    publishers.clear()
    guard.route_csv = guard.vehicle_file = guard.config_file = "unused-in-test"
    guard.world.get_blueprint_library = lambda: NS(find=lambda _name: object())
    monkeypatch.setattr(module, "read_route_csv", lambda _path: [{}] * 499)
    monkeypatch.setattr(module, "reconstruct_csv_waypoint", lambda *_args: None)
    monkeypatch.setattr(module, "vehicle_config_error", lambda *_args: "")
    assert IntegrationGuard.check(guard) == ""
