from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import carla
import yaml

from kcity_scenario_manager.common.vehicle_spawn import (
    BeforeStartSpawnConfig,
    choose_topology_candidate,
    load_suite_config_for_spawn,
    make_before_start_spawn_plan,
    make_spawn_transform,
    parse_before_start_spawn_config,
    select_upstream_spawn_waypoint,
    validate_spawn_candidate,
    validate_start_crossing,
    waypoint_spawn_transform,
)
from kcity_scenario_manager.competition_common import TriggerBox
from kcity_scenario_manager.route_csv import write_route_csv


class FakeWaypoint:
    def __init__(self, x, *, y=0.0, yaw=0.0, road=3, lane=-2, s=0.0):
        self.transform = carla.Transform(
            carla.Location(x=float(x), y=float(y), z=0.0),
            carla.Rotation(pitch=1.0, yaw=float(yaw), roll=2.0),
        )
        self.road_id = road
        self.section_id = 0
        self.lane_id = lane
        self.s = float(s)
        self.lane_type = carla.LaneType.Driving
        self._previous = []
        self._next = []

    def previous(self, _distance):
        return list(self._previous)

    def next(self, _distance):
        return list(self._next)


class FakeMap:
    def __init__(self, waypoint):
        self.waypoint = waypoint

    def get_waypoint(self, *_args, **_kwargs):
        return self.waypoint


class FakeWorld:
    def __init__(self, waypoint):
        self.map = FakeMap(waypoint)

    def get_map(self):
        return self.map


def box(center_x, extent_x=0.5, exit_edge="+x"):
    return TriggerBox(
        center_x, 0.0, 0.0,
        extent_x, 1.0, 1.0,
        0.0, True, exit_edge, "center",
    )


def connect_linear(points):
    for previous, current in zip(points[:-1], points[1:]):
        previous._next = [current]
        current._previous = [previous]
    return points


def route_row(index, x, yaw=0.0):
    return {
        "index": index,
        "route_s_m": float(index),
        "x": float(x),
        "y": 0.0,
        "z": 0.0,
        "yaw": float(yaw),
        "road_id": 3,
        "section_id": 0,
        "lane_id": -2,
        "opendrive_s": float(index),
        "is_junction": False,
    }


class BeforeStartSpawnTests(unittest.TestCase):
    def test_before_start_trigger_config_parsing(self):
        cfg = parse_before_start_spawn_config({
            "back_distance_m": 8.0,
            "minimum_trigger_clearance_m": 2.0,
        })
        self.assertEqual(cfg.back_distance_m, 8.0)
        self.assertEqual(cfg.minimum_trigger_clearance_m, 2.0)
        self.assertEqual(cfg.topology_step_m, 1.0)

    def test_start_and_finish_inside_candidates_are_rejected(self):
        start = box(0.0, 1.0)
        finish = box(10.0, 1.0)
        with self.assertRaisesRegex(RuntimeError, "inside START"):
            validate_spawn_candidate(FakeWaypoint(0.0), start, finish, 0.0)
        with self.assertRaisesRegex(RuntimeError, "inside FINISH"):
            validate_spawn_candidate(FakeWaypoint(10.0), start, finish, 0.0)

    def test_previous_waypoint_selection_honors_distance_and_clearance(self):
        points = connect_linear([
            FakeWaypoint(x, s=x + 4.0) for x in (-4.0, -3.0, -2.0, -1.0, 0.0)
        ])
        selected, chain, distance, clearance = select_upstream_spawn_waypoint(
            points[-1],
            start_trigger=box(0.0, 0.5),
            finish_trigger=box(20.0),
            route_road_id=3,
            route_lane_id=-2,
            route_heading_deg=0.0,
            settings=BeforeStartSpawnConfig(3.0, 2.0),
        )
        self.assertIs(selected, points[1])
        self.assertEqual([wp.transform.location.x for wp in chain], [0, -1, -2, -3])
        self.assertEqual(distance, 3.0)
        self.assertEqual(clearance, 2.5)

    def test_multiple_previous_candidates_use_route_lane_then_heading(self):
        current = FakeWaypoint(0.0)
        wrong_lane = FakeWaypoint(-1.0, yaw=0.0, road=775, lane=1)
        route_lane = FakeWaypoint(-1.2, yaw=5.0, road=3, lane=-2)
        chosen = choose_topology_candidate(
            [wrong_lane, route_lane],
            current_waypoint=current,
            route_road_id=3,
            route_lane_id=-2,
            route_heading_deg=0.0,
        )
        self.assertIs(chosen, route_lane)

    def test_spawn_transform_uses_waypoint_yaw_and_rotation(self):
        waypoint = FakeWaypoint(5.0, y=6.0, yaw=-63.25)
        transform = waypoint_spawn_transform(waypoint, 0.5)
        self.assertEqual(transform.location.x, 5.0)
        self.assertEqual(transform.location.y, 6.0)
        self.assertEqual(transform.location.z, 0.5)
        self.assertEqual(transform.rotation.yaw, -63.25)
        self.assertEqual(transform.rotation.pitch, 1.0)
        self.assertEqual(transform.rotation.roll, 2.0)

    def test_start_entry_and_configured_exit_crossing(self):
        points = connect_linear([
            FakeWaypoint(-3.0), FakeWaypoint(-2.0), FakeWaypoint(-1.0),
            FakeWaypoint(0.0), FakeWaypoint(1.0),
        ])
        forward = validate_start_crossing(
            [points[3], points[2], points[1], points[0]],
            start_trigger=box(0.0, 0.5, "+x"),
            route_road_id=3,
            route_lane_id=-2,
            route_heading_deg=0.0,
            settings=BeforeStartSpawnConfig(3.0, 1.0),
        )
        self.assertIs(forward[-1], points[4])

    def test_route_heading_validation_rejects_wrong_csv_yaw(self):
        points = connect_linear([
            FakeWaypoint(-3.0), FakeWaypoint(-2.0), FakeWaypoint(-1.0),
            FakeWaypoint(0.0), FakeWaypoint(1.0),
        ])
        with TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "config" / "qualifier.yaml"
            route_path = root / "routes" / "qualifier.csv"
            config_path.parent.mkdir()
            write_route_csv(route_path, [route_row(0, 0.0, 180.0), route_row(1, 1.0, 180.0)])
            suite = {
                "triggers": {
                    "START": {
                        "enabled": True,
                        "center": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "extent": {"x": 0.5, "y": 1.0, "z": 1.0},
                        "yaw_deg": 0.0,
                        "exit_edge": "+x",
                    },
                    "FINISH": {
                        "enabled": True,
                        "center": {"x": 20.0, "y": 0.0, "z": 0.0},
                        "extent": {"x": 0.5, "y": 1.0, "z": 1.0},
                        "yaw_deg": 0.0,
                        "exit_edge": "+x",
                    },
                },
                "benchmark": {"route_csv": "routes/qualifier.csv"},
            }
            config_path.write_text(yaml.safe_dump(suite), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "yaw disagrees"):
                make_before_start_spawn_plan(
                    FakeWorld(points[3]),
                    {"mode": "before_start_trigger", "back_distance_m": 3.0},
                    suite,
                    str(config_path),
                )

    def test_suite_config_file_missing_and_invalid_errors(self):
        spawn = {"mode": "before_start_trigger"}
        with self.assertRaisesRegex(RuntimeError, "suite_config_file is required"):
            load_suite_config_for_spawn(spawn, "")
        with self.assertRaisesRegex(RuntimeError, "invalid suite_config_file"):
            load_suite_config_for_spawn(spawn, "/does/not/exist/qualifier.yaml")

    def test_legacy_nearest_driving_waypoint_regression(self):
        waypoint = FakeWaypoint(2.0, y=3.0, yaw=42.0)
        transform, reason = make_spawn_transform(
            FakeWorld(waypoint),
            {
                "mode": "nearest_driving_waypoint",
                "seed_location": {"x": 1.0, "y": 1.0, "z": 0.0},
                "z_offset": 0.5,
            },
        )
        self.assertEqual(transform.location.x, 2.0)
        self.assertEqual(transform.rotation.yaw, 42.0)
        self.assertIn("nearest_driving_waypoint", reason)


if __name__ == "__main__":
    unittest.main()
