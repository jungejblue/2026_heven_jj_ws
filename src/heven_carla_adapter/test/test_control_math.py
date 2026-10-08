"""Pure conversion checks; also collected by ament/pytest without ROS or CARLA."""

import math
import unittest

from heven_carla_adapter.control_math import (
    SteeringCalibration, drive_to_carla, front_max_steer_deg,
    road_wheel_to_carla, steering_curve_scale,
)


class ControlMathTests(unittest.TestCase):
    def test_sensor_calibration_preserves_center_signs_and_asymmetric_ranges(self):
        calibration = SteeringCalibration()
        self.assertEqual(calibration.sensor_to_road_wheel(-5.5), 0)
        self.assertEqual(calibration.sensor_to_road_wheel(-14.5), -30)
        self.assertEqual(calibration.sensor_to_road_wheel(3.5), 30)
        self.assertEqual(calibration.sensor_to_road_wheel(-100), -30)
        self.assertEqual(calibration.sensor_to_road_wheel(100), 30)
        asymmetric = SteeringCalibration(-5.5, -17.5, 3.5, 30.0)
        self.assertEqual(asymmetric.sensor_to_road_wheel(-11.5), -15)
        self.assertEqual(asymmetric.sensor_to_road_wheel(-1.0), 15)
        for sensor in (-17.5, -11.5, -5.5, -1.0, 3.5):
            self.assertAlmostEqual(asymmetric.road_wheel_to_sensor(
                asymmetric.sensor_to_road_wheel(sensor)), sensor)

    def test_measured_angle_feedback_does_not_hide_physics_excursions(self):
        self.assertEqual(SteeringCalibration().road_wheel_to_sensor(45), 8.0)
        with self.assertRaises(ValueError):
            SteeringCalibration().sensor_to_road_wheel(math.nan)

    def test_front_physics_limits_and_speed_curve_compensation(self):
        self.assertEqual(front_max_steer_deg([40, 38, 90, 90], 30), 40)
        self.assertEqual(front_max_steer_deg([0, 0], 30), 30)
        curve = [(0, 1), (20, 0.75), (40, 0.5)]
        self.assertAlmostEqual(steering_curve_scale(10, curve), 0.875)
        self.assertEqual(steering_curve_scale(-40, curve), 0.5)
        self.assertEqual(steering_curve_scale(100, curve), 0.5)
        self.assertEqual(steering_curve_scale(0, []), 1)
        self.assertEqual(steering_curve_scale(0, [(0, 1.5)]), 1)
        self.assertEqual(steering_curve_scale(0, [(0, -0.2)]), 0)
        # Identical wheel-angle demand needs twice the input at multiplier 0.5.
        self.assertEqual(road_wheel_to_carla(15, 40, 1), 0.375)
        self.assertEqual(road_wheel_to_carla(15, 40, 0.5), 0.75)
        self.assertEqual(road_wheel_to_carla(-30, 40, 0.5), -1)
        self.assertEqual(road_wheel_to_carla(30, 40, 0.5), 1)
        self.assertEqual(road_wheel_to_carla(10, 40, 0), 0)

    def test_drive_flags_override_torque_without_automatic_braking(self):
        self.assertAlmostEqual(drive_to_carla(True, 500, False).throttle, 500 / 3200)
        self.assertEqual(drive_to_carla(True, 4000, False).throttle, 1)
        self.assertEqual(drive_to_carla(True, -1, False).throttle, 0)
        disabled = drive_to_carla(False, 3200, False)
        self.assertEqual(disabled.throttle, 0)
        self.assertEqual(disabled.brake, 0)
        self.assertFalse(disabled.motor_enabled)
        stopped = drive_to_carla(True, 3200, True)
        self.assertEqual(stopped.throttle, 0)
        self.assertEqual(stopped.brake, 1)
        self.assertEqual(stopped.torque_raw, 0)
        self.assertFalse(stopped.motor_enabled)
        self.assertEqual(drive_to_carla(True, 500, False, 1000).throttle, 0.5)


if __name__ == '__main__':
    unittest.main()
