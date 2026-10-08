"""Pure tests of the CARLA/HEVEN units, heading and geographic contracts."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from heven_carla_adapter.geodesy import (EARTH_RADIUS_M, ORIGIN_LATITUDE_DEG,
    ORIGIN_LONGITUDE_DEG, enu_displacement, finite_direction_yaw,
    finite_velocity_enu, project_latlon)
from heven_carla_adapter.sensor_math import (SIM_RTK_FIXED_FLAGS, navpvt_values,
    simulation_i_tow, stamp_nanoseconds, yaw_from_quaternion)


def geographic_map(matrix):
    """Make an affine CARLA-world -> JJ ENU reference for coordinate tests."""
    north_scale = math.pi / 180.0 * EARTH_RADIUS_M
    east_scale = north_scale * math.cos(math.radians(ORIGIN_LATITUDE_DEG))

    def geolocate(x, y, z):
        world = (x, y, z)
        east, north, up = (sum(row[i] * world[i] for i in range(3)) for row in matrix)
        return (ORIGIN_LATITUDE_DEG + north / north_scale,
                ORIGIN_LONGITUDE_DEG + east / east_scale, 20.0 + up)
    return geolocate


class SensorContractTests(unittest.TestCase):
    def test_navpvt_rtk_flags_and_scaled_units(self):
        fields = navpvt_values(12, 345678901, 37.1234567, 126.7654321,
                               12.345, (3.0, 4.0, 5.0))
        self.assertEqual(fields["i_tow"], 12345)
        self.assertEqual(fields["fix_type"], 3)
        self.assertEqual(fields["flags"], 131)
        self.assertEqual(fields["flags"], SIM_RTK_FIXED_FLAGS)
        self.assertEqual(fields["lat"], 371234567)
        self.assertEqual(fields["lon"], 1267654321)
        self.assertEqual(fields["height"], 12345)
        self.assertEqual(fields["h_msl"], 12345)
        self.assertEqual(fields["h_acc"], 10)
        self.assertEqual(fields["v_acc"], 10)
        self.assertEqual((fields["vel_e"], fields["vel_n"], fields["vel_d"]), (3000, 4000, -5000))
        self.assertEqual(fields["g_speed"], 5000)
        self.assertEqual(fields["head_mot"], 3686990)

    def test_stationary_vehicle_has_zero_speed_and_course(self):
        fields = navpvt_values(1, 0, ORIGIN_LATITUDE_DEG, ORIGIN_LONGITUDE_DEG,
                               0.0, (0.0, 0.0, 0.0))
        self.assertEqual(tuple(fields[key] for key in ("vel_e", "vel_n", "vel_d", "g_speed", "head_mot")),
                         (0, 0, 0, 0, 0))

    def test_course_is_clockwise_from_north(self):
        for enu, expected in (((0, 2, 0), 0), ((2, 0, 0), 9000000),
                              ((0, -2, 0), 18000000), ((-2, 0, 0), 27000000)):
            with self.subTest(velocity=enu):
                self.assertEqual(navpvt_values(1, 0, 0, 0, 0, enu)["head_mot"], expected)

    def test_simulated_time_wraps_only_at_gps_week(self):
        self.assertEqual(stamp_nanoseconds(2, 3), 2000000003)
        self.assertEqual(simulation_i_tow(604799, 999999999), 604799999)
        self.assertEqual(simulation_i_tow(604800, 0), 0)
        self.assertEqual(simulation_i_tow(604800, 123456789), 123)
        # Positive sub-millisecond stamps remain valid even if iTOW is zero.
        self.assertEqual(simulation_i_tow(0, 1), 0)

    def test_invalid_ros_stamps_are_rejected(self):
        for stamp in ((0, 0), (-1, 0), (1, -1), (1, 1000000000), (1.5, 0), (True, 0)):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                stamp_nanoseconds(*stamp)

    def test_navpvt_rejects_invalid_coordinates_and_accuracy(self):
        for lat, lon, acc in ((91, 0, 10), (0, -181, 10), (math.nan, 0, 10),
                              (0, 0, 0), (0, 0, -1), (0, 0, 10.5), (0, 0, 4294967296)):
            with self.subTest(latitude=lat, longitude=lon, accuracy=acc), self.assertRaises(ValueError):
                navpvt_values(1, 0, lat, lon, 0, (0, 0, 0), acc)
        with self.assertRaises(ValueError):
            navpvt_values(1, 0, 0, 0, 0, (3000000, 0, 0))

    def test_measured_status_orientation_preserves_ros_positive_yaw(self):
        for yaw in (-math.pi, -0.7, 0, math.pi / 2, math.pi):
            with self.subTest(yaw=yaw):
                actual = yaw_from_quaternion((0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
                self.assertAlmostEqual(math.sin(actual), math.sin(yaw))
                self.assertAlmostEqual(math.cos(actual), math.cos(yaw))
        with self.assertRaises(ValueError):
            yaw_from_quaternion((0, 0, 0, 0))

    def test_projection_matches_jj_fixed_origin_formula(self):
        latitude, longitude = 37.24, 126.78
        east, north = project_latlon(latitude, longitude)
        expected_east = math.radians(longitude - ORIGIN_LONGITUDE_DEG) * EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LATITUDE_DEG))
        expected_north = math.radians(latitude - ORIGIN_LATITUDE_DEG) * EARTH_RADIUS_M
        self.assertAlmostEqual(east, expected_east, places=10)
        self.assertAlmostEqual(north, expected_north, places=10)
        self.assertEqual(project_latlon(ORIGIN_LATITUDE_DEG, ORIGIN_LONGITUDE_DEG), (0.0, 0.0))

    def test_carla_world_y_right_maps_to_ros_heading_and_velocity(self):
        geolocate = geographic_map(((1, 0, 0), (0, -1, 0), (0, 0, 1)))
        yaw = finite_direction_yaw((5, 10, 2), (0, 1, 0), geolocate)
        self.assertAlmostEqual(yaw, -math.pi / 2, places=7)
        velocity = finite_velocity_enu((5, 10, 2), (3, 4, 5), geolocate)
        for actual, expected in zip(velocity, (3, -4, 5)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_rotated_georeference_uses_projected_forward_direction(self):
        # Map axes may rotate relative to compass; hard-coding -CARLA yaw
        # would incorrectly produce zero for this world's forward direction.
        geolocate = geographic_map(((0, -1, 0), (1, 0, 0), (0, 0, 1)))
        self.assertAlmostEqual(finite_direction_yaw((0, 0, 0), (1, 0, 0), geolocate), math.pi / 2, places=7)
        velocity = finite_velocity_enu((4, 7, 2), (3, 4, 5), geolocate)
        for actual, expected in zip(velocity, (-4, 3, 5)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_velocity_uses_all_geographic_jacobian_axes(self):
        matrix = ((2, 0.5, 0.1), (-0.2, -1.5, 0.3), (0.4, 0.5, 2))
        geolocate = geographic_map(matrix)
        measured = (3, 4, -2)
        actual = finite_velocity_enu((15, -23, 5), measured, geolocate, sample_distance_m=0.5)
        expected = (7.8, -7.2, -0.8)
        for component, reference in zip(actual, expected):
            self.assertAlmostEqual(component, reference, places=6)

    def test_true_zero_motion_does_not_infer_velocity_from_heading(self):
        def unavailable_map(*xyz):
            raise AssertionError("Zero motion must not require a geographic sample")
        self.assertEqual(finite_velocity_enu((0, 0, 0), (0, 0, 0), unavailable_map), (0, 0, 0))

    def test_direction_rejects_degenerate_geographic_reference(self):
        with self.assertRaises(ValueError):
            finite_direction_yaw((0, 0, 0), (0, 0, 0), geographic_map(((1, 0, 0), (0, -1, 0), (0, 0, 1))))
        with self.assertRaises(ValueError):
            finite_direction_yaw((0, 0, 0), (1, 0, 0), lambda *xyz: (37, 126, 0))
        with self.assertRaises(ValueError):
            project_latlon(math.nan, 126)

    def test_absolute_altitude_difference_supplies_enu_up(self):
        delta = enu_displacement((37.0, 126.0, 20.0), (37.0, 126.0, 23.5))
        self.assertEqual(delta, (0, 0, 3.5))


if __name__ == "__main__":
    unittest.main()
