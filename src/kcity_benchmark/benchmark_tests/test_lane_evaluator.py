from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

from kcity_benchmark.lane_evaluator import (
    CsvCorridorEvaluator,
    LaneGeometryEvaluator,
    LanePenaltyState,
    RouteMatcher,
    classify_wheels,
    lane_change_permitted,
    point_in_polygon,
)
from kcity_benchmark.lane_reference import load_lane_reference, sha256_file


ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).parent / "fixtures/lane_qualifier_20260927_212315.json"
ROUTE = ROOT / "src/kcity_scenario_manager/routes/qualifier.csv"
MAP_SNAPSHOT = Path(__file__).parent / "fixtures/qualifier_map.xodr"
LANE_REFERENCE = ROOT / "src/kcity_benchmark/kcity_benchmark/data/qualifier_lane_reference.json"


def rectangle(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def wheel_points(front_left_y=0.6, front_right_y=-0.6,
                 rear_left_y=0.6, rear_right_y=-0.6,
                 front_x=1.0, rear_x=-1.0):
    return {
        "FL": (front_x, front_left_y),
        "FR": (front_x, front_right_y),
        "RL": (rear_x, rear_left_y),
        "RR": (rear_x, rear_right_y),
    }


def lane_penalty_total(state, samples):
    total = 0.0
    emitted = []
    for now, count in samples:
        events = state.update(now, count, "synthetic regression")
        total += sum(event.penalty_sec for event in events)
        emitted.extend(event.event for event in events)
    return total, emitted


def load_carla_api(test_case):
    try:
        import carla
        return carla
    except ImportError:
        egg_dir = Path.home() / "CARLA_0.9.15/PythonAPI/carla/dist"
        eggs = list(egg_dir.glob(
            f"carla-*-py{sys.version_info.major}.{sys.version_info.minor}-*.egg"
        ))
        if len(eggs) != 1:
            test_case.skipTest("CARLA 0.9.15 Python API egg is unavailable")
        sys.path.insert(0, str(eggs[0]))
        import carla
        return carla


class LaneEvaluatorRegressionTests(unittest.TestCase):
    def test_lane_change_queries_full_configured_behind_and_ahead_distance(self):
        for source_lane, target_lane, expected_stations in (
            (-1, -2, (93.0, 113.0)),
            (1, 2, (87.0, 107.0)),
        ):
            with self.subTest(source_lane=source_lane):
                evaluator = LaneGeometryEvaluator.__new__(LaneGeometryEvaluator)
                evaluator.behind_m, evaluator.ahead_m = 8.0, 12.0
                evaluator.rows = [
                    dict(road_id=1, section_id=0, lane_id=source_lane,
                         opendrive_s=100.0, route_s_m=0.0),
                    dict(road_id=1, section_id=0, lane_id=target_lane,
                         opendrive_s=105.0, route_s_m=5.0),
                ]
                stations = {source_lane:[], target_lane:[]}
                def lookup(_road, _section, lane, station):
                    stations[lane].append(station)
                    return None
                evaluator._lookup_exact = lookup
                evaluator.carla = SimpleNamespace(LaneType=SimpleNamespace(Driving='Driving'))
                evaluator._lane_change_polygons(
                    dict(source_index=0, target_index=1), route_s=1.0
                )
                for lane in (source_lane, target_lane):
                    self.assertEqual(len(stations[lane]), 17)
                    self.assertEqual((min(stations[lane]), max(stations[lane])),
                                     expected_stations)

    def test_same_xodr_identifiers_on_changed_map_reject_stale_csv_geometry(self):
        driving = "Driving"
        waypoint = SimpleNamespace(
            road_id=1, section_id=0, lane_id=-1, lane_type=driving,
            transform=SimpleNamespace(location=SimpleNamespace(x=100.0, y=0.0)),
            next=lambda _:[],
        )
        carla_map = SimpleNamespace(get_waypoint_xodr=lambda *_: waypoint)
        row = dict(index=0, road_id=1, section_id=0, lane_id=-1,
                   opendrive_s=1.0, route_s_m=0.0, x=0.0, y=0.0)
        carla = SimpleNamespace(LaneType=SimpleNamespace(Driving=driving))
        with self.assertRaisesRegex(RuntimeError, "index=0.*100.000m from CSV x/y"):
            LaneGeometryEvaluator(carla_map, carla, [row, dict(row, index=1)])

    def test_current_xodr_waypoint_within_csv_tolerance_is_accepted(self):
        driving = "Driving"
        waypoint = SimpleNamespace(
            road_id=1, section_id=0, lane_id=-1, lane_type=driving,
            transform=SimpleNamespace(location=SimpleNamespace(x=0.02, y=0.0)),
        )
        evaluator = LaneGeometryEvaluator.__new__(LaneGeometryEvaluator)
        evaluator.map = SimpleNamespace(get_waypoint_xodr=lambda *_: waypoint)
        evaluator.carla = SimpleNamespace(LaneType=SimpleNamespace(Driving=driving))
        evaluator.waypoint_tolerance_m = 1.0
        row = dict(index=0, road_id=1, section_id=0, lane_id=-1,
                   opendrive_s=1.0, x=0.0, y=0.0)
        self.assertIs(evaluator._route_waypoint(row), waypoint)

    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        with ROUTE.open(newline="", encoding="utf-8") as stream:
            cls.route_rows = list(csv.DictReader(stream))
        for row in cls.route_rows:
            for key in ("index", "road_id", "section_id", "lane_id"):
                row[key] = int(row[key])
            for key in ("route_s_m", "x", "y", "z", "yaw", "opendrive_s"):
                row[key] = float(row[key])
        cls.reference = load_lane_reference(
            LANE_REFERENCE, cls.route_rows, sha256_file(ROUTE)
        )

    def test_ten_recorded_false_positives_match_reference_route(self):
        cases = self.fixture["cases"]
        self.assertEqual(len(cases), 10)
        expected_reasons = [
            "allowed_lane_change",
            "junction_route_mismatch",
            "junction_route_mismatch",
            "junction_route_mismatch",
            "curved_lane",
            "curved_lane",
            "junction_route_mismatch",
            "junction_route_mismatch",
            "junction_route_mismatch",
            "allowed_lane_change",
        ]
        for case, reason in zip(cases, expected_reasons):
            self.assertFalse(case["expected_violation"], case["case"])
            self.assertEqual(case["classification"],
                             "B" if reason == "allowed_lane_change" else "D")
            route = case["route"]
            matcher = RouteMatcher(self.route_rows)
            matcher.previous_index = max(0, route["index"] - 1)
            pose = case["ego_pose"]
            match = matcher.match(pose["x"], pose["y"], pose["yaw_deg"])
            matched_row = self.route_rows[match.index]
            self.assertEqual(match.index, route["index"], f"case #{case['case']}")
            self.assertEqual(
                (matched_row["road_id"], matched_row["section_id"], matched_row["lane_id"]),
                (route["road_id"], route["section_id"], route["lane_id"]),
            )
            if case["classification"] == "B":
                option_index = route["index"]
                options_nearby = [
                    self.route_rows[index]["road_option"]
                    for index in range(option_index, min(len(self.route_rows), option_index + 2))
                ]
                self.assertTrue(any(option in ("CHANGELANELEFT", "CHANGELANERIGHT")
                                    for option in options_nearby))
                self.assertEqual(case["allowed_lane_change_union_outside_over_1mm_count"], 0)
            else:
                self.assertGreater(case["correct_route_footprint_clearance_m"], 0.0)

    def test_ten_recorded_footprints_fit_route_curved_corridors_offline(self):
        carla = load_carla_api(self)
        map_bytes = MAP_SNAPSHOT.read_bytes()
        self.assertEqual(hashlib.sha256(map_bytes).hexdigest(),
                         self.fixture["map_sha256"])
        carla_map = carla.Map("qualifier regression fixture", map_bytes.decode("utf-8"))
        evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        bbox = self.fixture["actor_bbox"]
        extent_x, extent_y = bbox["extent"]["x"], bbox["extent"]["y"]
        offset_x, offset_y = bbox["location"]["x"], bbox["location"]["y"]
        corners = [(-extent_x, -extent_y), (extent_x, -extent_y),
                   (extent_x, extent_y), (-extent_x, extent_y)]

        for case in self.fixture["cases"]:
            route = case["route"]
            evaluator.matcher.previous_index = max(0, route["index"] - 1)
            pose = case["ego_pose"]
            match = evaluator.matcher.match(pose["x"], pose["y"], pose["yaw_deg"])
            self.assertEqual(match.index, route["index"], f"case #{case['case']}")
            polygons, _ = evaluator.corridor_polygons_for_match(
                match.index, match.route_s_m
            )
            yaw = math.radians(pose["yaw_deg"])
            perimeter = []
            for first, second in zip(corners, corners[1:] + corners[:1]):
                count = math.ceil(math.dist(first, second) / 0.02)
                for sample in range(count):
                    fraction = sample / count
                    local_x = first[0] + fraction * (second[0] - first[0]) + offset_x
                    local_y = first[1] + fraction * (second[1] - first[1]) + offset_y
                    perimeter.append((
                        pose["x"] + local_x * math.cos(yaw) - local_y * math.sin(yaw),
                        pose["y"] + local_x * math.sin(yaw) + local_y * math.cos(yaw),
                    ))
            outside = [point for point in perimeter if not any(
                point_in_polygon(point, polygon, 0.05) for polygon in polygons
            )]
            self.assertEqual(outside, [], f"case #{case['case']} bbox escaped corridor")

    def test_csv_corridor_keeps_all_ten_recorded_footprint_regressions_inside(self):
        """The raw fixture has bbox poses, not recorded wheel centers."""
        evaluator = CsvCorridorEvaluator(
            self.reference, None, self.route_rows, boundary_tolerance_m=0.05
        )
        bbox = self.fixture["actor_bbox"]
        extent_x, extent_y = bbox["extent"]["x"], bbox["extent"]["y"]
        offset_x, offset_y = bbox["location"]["x"], bbox["location"]["y"]
        route_reach = math.hypot(extent_x + abs(offset_x),
                                 extent_y + abs(offset_y)) + evaluator.max_cell_span_m
        corners = [(-extent_x, -extent_y), (extent_x, -extent_y),
                   (extent_x, extent_y), (-extent_x, extent_y)]

        for case in self.fixture["cases"]:
            route_index = case["route"]["index"]
            pose = case["ego_pose"]
            evaluator.matcher.previous_index = max(0, route_index - 1)
            center_match = evaluator.matcher.match(
                pose["x"], pose["y"], pose["yaw_deg"]
            )
            route_reach += center_match.distance_m
            yaw = math.radians(pose["yaw_deg"])
            active_change_ids = {
                int(change["lane_change_id"])
                for change in self.reference["lane_change_windows"]
                if float(change["route_s_start_m"]) - route_reach
                <= center_match.route_s_m
                <= float(change["route_s_end_m"]) + route_reach
            }
            transition_cells = [
                cell for cell in evaluator.cells
                if cell.get("kind") == "lane_change"
                and int(cell["lane_change_id"]) in active_change_ids
            ]
            local_cells = evaluator._nearby_cells(
                center_match.route_s_m, route_reach
            )
            allowed_cells = local_cells + transition_cells
            perimeter = []
            for first, second in zip(corners, corners[1:] + corners[:1]):
                count = math.ceil(math.dist(first, second) / 0.02)
                for sample in range(count):
                    fraction = sample / count
                    local_x = first[0] + fraction * (second[0] - first[0]) + offset_x
                    local_y = first[1] + fraction * (second[1] - first[1]) + offset_y
                    perimeter.append((
                        pose["x"] + local_x * math.cos(yaw) - local_y * math.sin(yaw),
                        pose["y"] + local_x * math.sin(yaw) + local_y * math.cos(yaw),
                    ))
            for point in perimeter:
                self.assertTrue(any(
                    point_in_polygon(point, cell["polygon"], 0.05)
                    for cell in allowed_cells
                ), f"case #{case['case']} recorded footprint escaped CSV corridor")

    def test_csv_corridor_normal_one_wheel_four_wheel_and_lane_changes(self):
        carla = load_carla_api(self)

        def fake_vehicle(offsets, transform):
            return SimpleNamespace(
                get_physics_control=lambda: SimpleNamespace(wheels=[
                    SimpleNamespace(position=carla.Vector3D(
                        x=point.x * 100.0, y=point.y * 100.0,
                        z=point.z * 100.0))
                    for point in (transform.transform(carla.Location(x=x, y=y, z=z))
                    for x, y, z in offsets
                    )
                ]),
                bounding_box=SimpleNamespace(
                    location=carla.Location(x=0.0, y=0.0, z=0.5)
                ),
            )

        index = 100
        row = self.route_rows[index]
        transform = carla.Transform(
            carla.Location(x=row["x"], y=row["y"], z=row["z"]),
            carla.Rotation(yaw=row["yaw"]),
        )
        scenarios = (
            ("normal", (0.7, -0.7, 0.7, -0.7), 0, ()),
            ("one_wheel_exit", (2.8, -0.7, 0.7, -0.7), 1, ("FL",)),
            ("four_wheel_exit", (3.0, -3.0, 3.0, -3.0), 4,
             ("FL", "FR", "RL", "RR")),
        )
        for label, lateral, expected_count, expected_names in scenarios:
            evaluator = CsvCorridorEvaluator(
                self.reference, carla, self.route_rows,
                boundary_tolerance_m=0.05,
            )
            evaluator.matcher.previous_index = index
            offsets = ((0.8, lateral[0], 0.0), (0.8, lateral[1], 0.0),
                       (-0.8, lateral[2], 0.0), (-0.8, lateral[3], 0.0))
            result = evaluator.evaluate(fake_vehicle(offsets, transform), transform)
            self.assertEqual(result.wheel_out_count, expected_count, label)
            self.assertEqual(result.outside_wheels, expected_names, label)
            state = LanePenaltyState()
            events = state.update(0.0, result.wheel_out_count, label)
            if expected_count:
                self.assertIn("LANE_CROSS_START", [event.event for event in events])
                self.assertEqual(sum(event.penalty_sec for event in events), 20.0)
            if expected_count == 4:
                self.assertIn("FULL_LANE_EXIT", [event.event for event in events])
                self.assertTrue(state.emergency_stop_center_relocation_required)

        for case_index in (0, 9):
            case = self.fixture["cases"][case_index]
            pose = case["ego_pose"]
            transform = carla.Transform(
                carla.Location(x=pose["x"], y=pose["y"], z=pose["z"]),
                carla.Rotation(yaw=pose["yaw_deg"]),
            )
            evaluator = CsvCorridorEvaluator(
                self.reference, carla, self.route_rows,
                boundary_tolerance_m=0.05,
            )
            evaluator.matcher.previous_index = max(0, case["route"]["index"] - 1)
            offsets = ((0.8, 0.7, 0.0), (0.8, -0.7, 0.0),
                       (-0.8, 0.7, 0.0), (-0.8, -0.7, 0.0))
            result = evaluator.evaluate(fake_vehicle(offsets, transform), transform)
            self.assertFalse(result.violation, f"case #{case['case']}")

    def test_csv_corridor_rejects_unprescribed_adjacent_lane(self):
        carla = load_carla_api(self)
        carla_map = carla.Map(
            "CSV corridor adjacent-lane regression", MAP_SNAPSHOT.read_bytes().decode("utf-8")
        )
        index = 100
        row = self.route_rows[index]
        waypoint = carla_map.get_waypoint_xodr(
            row["road_id"], row["lane_id"], row["opendrive_s"]
        )
        adjacent = waypoint.get_left_lane()
        self.assertIsNotNone(adjacent)
        evaluator = CsvCorridorEvaluator(
            self.reference, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        evaluator.matcher.previous_index = index
        offsets = ((0.8, 0.6, 0.0), (0.8, -0.6, 0.0),
                   (-0.8, 0.6, 0.0), (-0.8, -0.6, 0.0))
        vehicle = SimpleNamespace(
            get_physics_control=lambda: SimpleNamespace(wheels=[
                SimpleNamespace(position=carla.Vector3D(
                    x=point.x * 100.0, y=point.y * 100.0,
                    z=point.z * 100.0))
                for point in (adjacent.transform.transform(
                    carla.Location(x=x, y=y, z=z))
                for x, y, z in offsets
                )
            ]),
            bounding_box=SimpleNamespace(
                location=carla.Location(x=0.0, y=0.0, z=0.5)
            ),
        )
        result = evaluator.evaluate(vehicle, adjacent.transform)
        self.assertTrue(result.violation)
        self.assertEqual(result.matched_route_index, index)
        state = LanePenaltyState()
        events = state.update(12.0, result.wheel_out_count, result.detail())
        self.assertEqual(sum(event.penalty_sec for event in events), 20.0)

    def test_static_lane_reference_geometry_sanity_and_provenance(self):
        source = self.reference["source"]
        sanity = self.reference["sanity"]
        self.assertEqual(source["route_point_count"], 499)
        self.assertEqual(source["route_sha256"], sha256_file(ROUTE))
        self.assertEqual(source["xodr_sha256"], self.fixture["map_sha256"])
        self.assertEqual(sanity["invalid_polygon_cell_count"], 0)
        self.assertEqual(sanity["max_route_progress_coverage_gap_m"], 0.0)
        self.assertLessEqual(sanity["sample_spacing_m"]["max"], 0.25)
        self.assertGreater(sanity["lane_width_m"]["max"], 3.5)
        self.assertEqual(len(self.reference["lane_change_windows"]), 2)
        self.assertGreaterEqual(
            sanity["nonlocal_center_or_boundary_intersection_count"], 0
        )
        self.assertEqual(sanity["abrupt_lane_width_jump_count_over_0_5m"], 0)

    def test_vehicle_physics_positions_convert_world_centimeters_to_local_meters(self):
        carla = load_carla_api(self)
        map_bytes = MAP_SNAPSHOT.read_bytes()
        carla_map = carla.Map("wheel position regression", map_bytes.decode("utf-8"))
        evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        route_row = self.route_rows[300]
        waypoint = carla_map.get_waypoint_xodr(
            route_row["road_id"], route_row["lane_id"], route_row["opendrive_s"]
        )
        transform = waypoint.transform
        local = ((1.0, 0.8, 0.0), (1.0, -0.8, 0.0),
                 (-1.0, 0.8, 0.0), (-1.0, -0.8, 0.0))

        def fake_vehicle(positions):
            return SimpleNamespace(
                get_physics_control=lambda: SimpleNamespace(wheels=[
                    SimpleNamespace(position=carla.Vector3D(x=x, y=y, z=z))
                    for x, y, z in positions
                ]),
                bounding_box=SimpleNamespace(
                    location=carla.Location(x=0.0, y=0.0, z=0.5)
                ),
            )

        world_positions = [
            (point.x * 100.0, point.y * 100.0, point.z * 100.0)
            for point in (transform.transform(carla.Location(x=x, y=y, z=z))
                          for x, y, z in local)
        ]
        world_evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        world_evaluator._capture_wheel_offsets(
            fake_vehicle(world_positions), transform
        )
        self.assertEqual(world_evaluator._wheel_position_frame,
                         "world_cm_to_vehicle_local_m")
        for actual, expected in zip(world_evaluator._wheel_offsets, local):
            for actual_axis, expected_axis in zip(actual, expected):
                self.assertAlmostEqual(actual_axis, expected_axis, places=4)
        self.assertAlmostEqual(world_evaluator._wheelbase_m, 2.0, places=3)
        self.assertAlmostEqual(world_evaluator._front_track_m, 1.6, places=3)
        self.assertAlmostEqual(world_evaluator._rear_track_m, 1.6, places=3)

    def test_live_heven_ego_wheel_sample_stays_inside_xodr_lane(self):
        """Regression from the stopped CARLA 0.9.15 vehicle.heven.ev probe."""
        carla = load_carla_api(self)
        carla_map = carla.Map(
            "live wheel regression", MAP_SNAPSHOT.read_bytes().decode("utf-8")
        )
        transform = carla.Transform(
            carla.Location(x=-40.825668, y=396.124725, z=0.000190),
            carla.Rotation(pitch=-0.041808, yaw=-63.166489,
                           roll=-0.011902),
        )
        raw_cm = (
            (-4103.157, 39527.609, 25.874),
            (-4002.025, 39578.766, 25.874),
            (-4162.961, 39646.410, 25.874),
            (-4062.196, 39697.336, 25.874),
        )
        vehicle = SimpleNamespace(get_physics_control=lambda: SimpleNamespace(
            wheels=[SimpleNamespace(position=carla.Vector3D(*point))
                    for point in raw_cm]
        ))
        evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        result = evaluator.evaluate(vehicle, transform)
        self.assertEqual(result.wheel_out_count, 0)
        self.assertEqual(result.matched_route_index, 0)
        self.assertEqual(result.wheel_position_frame,
                         "world_cm_to_vehicle_local_m")
        self.assertAlmostEqual(result.wheelbase_m, 1.3298, places=2)
        self.assertAlmostEqual(result.front_track_m, 1.1333, places=2)
        self.assertAlmostEqual(result.rear_track_m, 1.1290, places=2)
        for name, raw in zip(("FL", "FR", "RL", "RR"), raw_cm):
            for axis, cm in zip(("x", "y", "z"), raw):
                self.assertAlmostEqual(
                    result.wheel_positions[name][axis], cm / 100.0, places=2
                )
        self.assertEqual(LanePenaltyState().update(0.0, result.wheel_out_count,
                                                   result.detail()), [])

        # The first fixed live lap left one false +20 s event exactly at FINISH:
        # the front wheels passed the final route point while staying in lane.
        finish_transform = carla.Transform(
            carla.Location(x=-51.2971, y=392.1099, z=0.0006),
            carla.Rotation(yaw=115.6163),
        )
        finish_raw_cm = [
            finish_transform.transform(carla.Location(x=x, y=y, z=z))
            for x, y, z in evaluator._wheel_offsets
        ]
        finish_vehicle = SimpleNamespace(get_physics_control=lambda:
            SimpleNamespace(wheels=[SimpleNamespace(position=carla.Vector3D(
                x=point.x * 100.0, y=point.y * 100.0, z=point.z * 100.0
            )) for point in finish_raw_cm])
        )
        finish_evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        finish_evaluator.matcher.previous_index = 497
        finish = finish_evaluator.evaluate(finish_vehicle, finish_transform)
        self.assertEqual(finish.matched_route_index, 498)
        self.assertEqual(finish.wheel_out_count, 0)

    def test_unprescribed_adjacent_lane_move_is_true_positive_on_carla_map(self):
        carla = load_carla_api(self)
        carla_map = carla.Map(
            "unauthorized lane regression", MAP_SNAPSHOT.read_bytes().decode("utf-8")
        )
        evaluator = LaneGeometryEvaluator(
            carla_map, carla, self.route_rows, boundary_tolerance_m=0.05
        )
        index = 100
        row = self.route_rows[index]
        route_waypoint = carla_map.get_waypoint_xodr(
            row["road_id"], row["lane_id"], row["opendrive_s"]
        )
        adjacent = route_waypoint.get_left_lane()
        self.assertIsNotNone(adjacent)
        self.assertEqual(row["road_option"], "LANEFOLLOW")
        self.assertEqual(adjacent.lane_type, carla.LaneType.Driving)
        evaluator.matcher.previous_index = index

        offsets = ((0.8, 0.6, 0.0), (0.8, -0.6, 0.0),
                   (-0.8, 0.6, 0.0), (-0.8, -0.6, 0.0))
        vehicle = SimpleNamespace(
            get_physics_control=lambda: SimpleNamespace(wheels=[
                SimpleNamespace(position=carla.Vector3D(
                    x=point.x * 100.0, y=point.y * 100.0,
                    z=point.z * 100.0))
                for point in (adjacent.transform.transform(
                    carla.Location(x=x, y=y, z=z))
                for x, y, z in offsets
                )
            ]),
            bounding_box=SimpleNamespace(
                location=carla.Location(x=0.0, y=0.0, z=0.5)
            ),
        )
        evaluation = evaluator.evaluate(vehicle, adjacent.transform)
        self.assertGreater(evaluation.wheel_out_count, 0)
        self.assertTrue(evaluation.violation)
        self.assertEqual(evaluation.matched_route_index, index)
        self.assertEqual(evaluation.allowed_lane_ids, ((68, 0, -1),))
        penalty = LanePenaltyState()
        events = penalty.update(12.0, evaluation.wheel_out_count,
                                evaluation.detail())
        self.assertEqual(sum(event.penalty_sec for event in events), 20.0)

    def test_synthetic_normal_vehicle_has_no_violation(self):
        corridor = rectangle(-2.0, 2.0, -1.0, 1.0)
        outside = classify_wheels(wheel_points(), [corridor], tolerance_m=0.01)
        self.assertEqual(outside, ())
        penalty = LanePenaltyState()
        self.assertEqual(penalty.update(0.0, len(outside), "normal"), [])

    def test_synthetic_single_wheel_exit_gets_initial_penalty(self):
        corridor = rectangle(-2.0, 2.0, -1.0, 1.0)
        points = wheel_points(front_left_y=1.10)
        outside = classify_wheels(points, [corridor], tolerance_m=0.01)
        self.assertEqual(outside, ("FL",))
        penalty = LanePenaltyState()
        events = penalty.update(10.0, len(outside), "one wheel outside")
        self.assertEqual([event.event for event in events], ["LANE_CROSS_START"])
        self.assertEqual(sum(event.penalty_sec for event in events), 20.0)

    def test_synthetic_short_and_continuous_penalty_thresholds(self):
        for duration, expected in ((0.2, 20.0), (4.9, 20.0)):
            state = LanePenaltyState()
            total, events = lane_penalty_total(
                state, [(0.0, 1), (duration, 0)]
            )
            self.assertEqual(total, expected)
            self.assertIn("LANE_CROSS_END", events)

        state = LanePenaltyState()
        total, events = lane_penalty_total(
            state, [(0.0, 1), (5.0, 1), (5.1, 0)]
        )
        self.assertEqual(total, 40.0)
        self.assertEqual(events.count("LANE_CROSS_CONTINUOUS"), 1)

        state = LanePenaltyState()
        total, events = lane_penalty_total(
            state, [(0.0, 1), (5.0, 1), (10.0, 1), (10.1, 0)]
        )
        self.assertEqual(total, 60.0)
        self.assertEqual(events.count("LANE_CROSS_CONTINUOUS"), 2)

    def test_clear_then_reentry_starts_a_new_violation(self):
        state = LanePenaltyState()
        total, events = lane_penalty_total(
            state, [(1.0, 1), (1.2, 0), (2.0, 1), (2.2, 0)]
        )
        self.assertEqual(total, 40.0)
        self.assertEqual(events.count("LANE_CROSS_START"), 2)

    def test_synthetic_four_wheel_exit_sets_recovery_flag(self):
        state = LanePenaltyState()
        events = state.update(3.0, 4, "four wheels outside")
        self.assertEqual(
            [event.event for event in events],
            ["LANE_CROSS_START", "FULL_LANE_EXIT"],
        )
        self.assertTrue(state.full_exit_active)
        self.assertTrue(state.emergency_stop_center_relocation_required)
        self.assertEqual(sum(event.penalty_sec for event in events), 20.0)

    def test_authorized_lane_change_uses_source_target_union(self):
        class Marking:
            lane_change = "Both"

        self.assertTrue(lane_change_permitted(Marking(), "Left"))
        source = rectangle(-2.0, 2.0, -1.0, 1.0)
        target = rectangle(-2.0, 2.0, 1.0, 3.0)
        points = wheel_points(front_left_y=1.5, front_right_y=0.5,
                              rear_left_y=1.5, rear_right_y=0.5)
        self.assertEqual(classify_wheels(points, [source, target]), ())

    def test_unauthorized_route_exit_remains_a_true_positive(self):
        main_route = rectangle(-2.0, 2.0, -1.0, 1.0)
        unrelated_lane = rectangle(-2.0, 2.0, 2.0, 4.0)
        points = wheel_points(front_left_y=3.0, front_right_y=3.0,
                              rear_left_y=3.0, rear_right_y=3.0)
        outside = classify_wheels(points, [main_route])
        self.assertEqual(outside, ("FL", "FR", "RL", "RR"))
        self.assertEqual(classify_wheels(points, [main_route, unrelated_lane]), ())
        self.assertEqual(outside, ("FL", "FR", "RL", "RR"))


if __name__ == "__main__":
    unittest.main()
