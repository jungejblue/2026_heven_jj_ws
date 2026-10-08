"""The integration gate must reject duplicates before route control starts."""

from types import SimpleNamespace

from kcity_scenario_manager.qualifier_integration_guard import (
    EXPECTED_SENSORS, sensor_inventory_error,
)


def actor(name, kind, ego):
    return SimpleNamespace(id=42, type_id=kind,
                           attributes={"role_name": name}, parent=ego)


def test_exact_sensor_suite_and_benchmark_collision():
    ego = SimpleNamespace(id=17)
    sensors = [actor(name, kind, ego) for name, kind in EXPECTED_SENSORS.items()]
    assert sensor_inventory_error(sensors, ego, False) == ""
    assert sensor_inventory_error(sensors, ego, True)
    sensors.append(actor("collision", "sensor.other.collision", ego))
    assert sensor_inventory_error(sensors, ego, True) == ""


def test_duplicate_and_wrong_parent_are_rejected():
    ego = SimpleNamespace(id=17)
    sensors = [actor(name, kind, ego) for name, kind in EXPECTED_SENSORS.items()]
    sensors.append(actor("imu", "sensor.other.imu", ego))
    assert "mismatch" in sensor_inventory_error(sensors, ego, False)
    sensors.pop()
    sensors[0].parent = SimpleNamespace(id=99)
    assert "not attached" in sensor_inventory_error(sensors, ego, False)
