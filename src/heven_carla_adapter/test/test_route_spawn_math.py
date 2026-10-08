"""Round-trip checks for route entry generation against runtime map georefs."""

import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from heven_carla_adapter.geodesy import EARTH_RADIUS_M, project_latlon
from heven_carla_adapter.route_spawn_math import (build_spawn_config,
                                                 invert_latlon,
                                                 load_latlon_csv, route_entry)


def affine_map():
    latitude, longitude = 37.24, 126.77
    north_scale = math.radians(1.0) * EARTH_RADIUS_M
    east_scale = north_scale * math.cos(math.radians(latitude))

    def geolocate(x, y, z):
        east, north = 2.0 * x - 0.5 * y + 120, 0.25 * x + 1.5 * y - 300
        return latitude + north / north_scale, longitude + east / east_scale, z
    return geolocate


def carla_mercator_map(latitude_origin, longitude_origin):
    # CARLA's geographic transform uses a scaled spherical Mercator mapping.
    radius = 6378137.0
    scale = math.cos(math.radians(latitude_origin))
    origin_y = scale * radius * math.log(math.tan(math.radians(90.0 + latitude_origin) / 2.0))

    def geolocate(x, y, z):
        longitude = longitude_origin + math.degrees(x / (scale * radius))
        latitude = math.degrees(2.0 * math.atan(math.exp((origin_y - y) / (scale * radius)))) - 90.0
        return latitude, longitude, z
    return geolocate


def local_latlon(east, north, origin=(37.24, 126.77)):
    north_scale = math.radians(1.0) * EARTH_RADIUS_M
    east_scale = north_scale * math.cos(math.radians(origin[0]))
    return origin[0] + north / north_scale, origin[1] + east / east_scale


class RouteSpawnTests(unittest.TestCase):
    def test_newton_inverts_rotated_scaled_shifted_reference(self):
        geolocate = affine_map()
        target = geolocate(33.25, -81.5, 4)
        x, y = invert_latlon(*target[:2], geolocate)
        self.assertAlmostEqual(x, 33.25, places=5)
        self.assertAlmostEqual(y, -81.5, places=5)
        self.assertLess(math.hypot(*project_latlon(*geolocate(x, y, 4)[:2], *target[:2])), 0.001)

    def test_public_kcity_mercator_reference_roundtrip(self):
        lat0, lon0 = 37.2427, 126.773665
        geolocate = carla_mercator_map(lat0, lon0)
        target = (37.2389504, 126.7729808)
        x, y = invert_latlon(*target, geolocate)
        scale, radius = math.cos(math.radians(lat0)), 6378137.0
        expected_x = scale * radius * math.radians(target[1] - lon0)
        expected_y = scale * radius * (math.log(math.tan(math.radians(90 + lat0) / 2))
            - math.log(math.tan(math.radians(90 + target[0]) / 2)))
        self.assertAlmostEqual(x, expected_x, places=4)
        self.assertAlmostEqual(y, expected_y, places=4)

    def test_runtime_georef_change_moves_spawn_but_preserves_absolute_gnss(self):
        target = (37.2389504, 126.7729808)
        public_map = carla_mercator_map(37.2427, 126.773665)
        diagnostic_map = carla_mercator_map(37.24273, 126.77363)
        public_xy = invert_latlon(*target, public_map)
        runtime_xy = invert_latlon(*target, diagnostic_map)
        distance = math.hypot(public_xy[0] - runtime_xy[0], public_xy[1] - runtime_xy[1])
        self.assertGreater(distance, 4)
        self.assertLess(distance, 5)
        for geolocate, xy in ((public_map, public_xy), (diagnostic_map, runtime_xy)):
            actual = geolocate(*xy, 4)
            self.assertAlmostEqual(actual[0], target[0], places=8)
            self.assertAlmostEqual(actual[1], target[1], places=8)

    def test_spawn_json_converts_native_y_and_yaw_to_ros(self):
        def geolocate(x, y, z):
            return (*local_latlon(x, -y), z)
        points = [local_latlon(10, -20), local_latlon(16, -12)]
        config = build_spawn_config(points, geolocate)
        vehicle = config["objects"][0]
        self.assertEqual((vehicle["type"], vehicle["id"], vehicle["sensors"]),
                         ("vehicle.heven.ev", "ego_vehicle", []))
        spawn = vehicle["spawn_point"]
        self.assertAlmostEqual(spawn["x"], 10, places=4)
        self.assertAlmostEqual(spawn["y"], -20, places=4)
        self.assertAlmostEqual(spawn["yaw"], math.degrees(math.atan2(8, 6)), places=4)
        self.assertEqual((spawn["z"], spawn["roll"], spawn["pitch"]), (4, 0, 0))

    def test_route_tangent_uses_five_metres_of_polyline(self):
        points = [local_latlon(0, 0), local_latlon(2, 0), local_latlon(2, 10)]
        first, ahead = route_entry(points, 5)
        east, north = project_latlon(*ahead, *first)
        self.assertAlmostEqual(east, 2, places=5)
        self.assertAlmostEqual(north, 3, places=5)
        self.assertEqual(first, points[0])
        self.assertEqual(route_entry(points, 50)[1], points[-1])

    def test_csv_accepts_heven_headerless_and_optional_bom_header(self):
        with tempfile.TemporaryDirectory() as directory:
            route = Path(directory) / "route.csv"
            route.write_text("37.2389504,126.7729808\n37.2389532,126.7729818\n", encoding="utf-8")
            expected = load_latlon_csv(route)
            route.write_text("\ufefflatitude,longitude\n37.2389504,126.7729808\n\n37.2389532,126.7729818\n", encoding="utf-8")
            self.assertEqual(load_latlon_csv(route), expected)
            route.write_text("lat,lon\n91,126\n37,126\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "line 2"):
                load_latlon_csv(route)

    def test_singular_reference_and_nonconverged_solver_fail(self):
        with self.assertRaisesRegex(ValueError, "singular"):
            invert_latlon(37.24, 126.77, lambda x, y, z: (37.25, 126.78, z))
        geolocate = carla_mercator_map(37.2427, 126.773665)
        with self.assertRaisesRegex(ValueError, "did not converge"):
            invert_latlon(37.2389504, 126.7729808, geolocate, iterations=1, tolerance_m=1e-6)

    def test_stationary_route_and_invalid_lookahead_fail(self):
        first = local_latlon(0, 0)
        with self.assertRaisesRegex(ValueError, "no usable"):
            route_entry([first, first, first])
        for lookahead in (0, -1, math.nan):
            with self.subTest(lookahead=lookahead), self.assertRaises(ValueError):
                route_entry([first, local_latlon(0, 10)], lookahead)


if __name__ == "__main__":
    unittest.main()
