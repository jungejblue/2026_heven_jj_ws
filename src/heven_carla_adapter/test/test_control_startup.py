"""Exercise actual control tick at the original HEVEN initialization boundary.

Only external ROS/CARLA endpoints are mocked. No simulator is ticked and no
message types or clock data from a running ROS installation are required.
"""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from heven_carla_adapter.control_math import SteeringCalibration


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def load_control_module():
    modules = {}
    for name in ('carla', 'rclpy', 'rclpy.node', 'rclpy.qos', 'carla_msgs',
                 'carla_msgs.msg', 'jj_interface', 'jj_interface.msg', 'std_msgs', 'std_msgs.msg'):
        modules[name] = types.ModuleType(name)
    modules['rclpy.node'].Node = type('Node', (), {})
    modules['rclpy.qos'].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    modules['rclpy.qos'].QoSProfile = lambda **kwargs: kwargs
    modules['carla'].VehicleWheelLocation = types.SimpleNamespace(FL_Wheel=0, FR_Wheel=1)
    modules['carla_msgs.msg'].CarlaEgoVehicleControl = Message
    modules['std_msgs.msg'].Bool = Message
    for name in ('DriveCommand', 'DriveState', 'SteeringCommand', 'SteeringState'):
        setattr(modules['jj_interface.msg'], name, Message)
    path = Path(__file__).resolve().parents[1] / 'heven_carla_adapter' / 'control_adapter.py'
    spec = importlib.util.spec_from_file_location('heven_carla_adapter._startup_test_control', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class ControlStartupTests(unittest.TestCase):
    def test_waiting_init_commands_do_not_release_warmup_then_raw_500_drives(self):
        module = load_control_module()
        node = module.ControlAdapter.__new__(module.ControlAdapter)
        node.calibration = SteeringCalibration()
        node.max_torque = 3200
        node.compensate_curve = True
        node.physics_max_angle = 30.0
        node.steering_curve = []
        node.target_sensor = -5.5
        node.auto_enabled = False
        node.last_steer = 0.0
        node.require_sensors_ready = True
        node.sensors_ready = False
        node.have_drive = True
        node.drive_request = types.SimpleNamespace(
            motor_enabled=False, motor_torque_raw=0, brake_requested=False)
        node.control_pub, node.steering_pub, node.drive_pub = Publisher(), Publisher(), Publisher()
        node.find_ego = lambda: True
        node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: 'simulation_stamp'))
        node.ego = types.SimpleNamespace(
            get_velocity=lambda: types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            get_transform=lambda: types.SimpleNamespace(
                get_forward_vector=lambda: types.SimpleNamespace(x=1.0, y=0.0, z=0.0)),
            get_wheel_steer_angle=lambda wheel: 0.0)
        node.last_error = ''

        # LocalizationInit::command(false, false) arrives before sensors attach.
        # Keep backend readiness feedback, but leave warmup as the only writer.
        for _ in range(3):
            node.tick()
        self.assertEqual(node.control_pub.messages, [])
        feedback = node.steering_pub.messages[-1]
        self.assertTrue(feedback.driver_ready)
        self.assertEqual(feedback.current_external_steering_sensor_angle_deg, -5.5)
        self.assertEqual(feedback.sensor_feedback_age_sec, 0.0)
        self.assertFalse(feedback.pid_ack_received)

        # Existing init node now accepts RTK/IMU/state and requests straight 500.
        node.on_ready(types.SimpleNamespace(data=True))
        node.on_steering(types.SimpleNamespace(auto_enabled=True,
            external_steering_sensor_angle_deg=-5.5))
        node.on_drive(types.SimpleNamespace(motor_enabled=True,
            motor_torque_raw=500, brake_requested=False))
        node.tick()
        command = node.control_pub.messages[-1]
        self.assertAlmostEqual(command.throttle, 500 / 3200)
        self.assertEqual(command.steer, 0.0)
        self.assertEqual(command.brake, 0.0)
        self.assertFalse(command.hand_brake)
        self.assertFalse(command.reverse)
        self.assertEqual(command.header.stamp, 'simulation_stamp')
        self.assertEqual(node.drive_pub.messages[-1].motor_torque_raw_commanded, 500)

        # Successful initializer exits without a stop. Preserve its command
        # during launch handover, then accept the selected controller's command.
        node.tick()
        self.assertAlmostEqual(node.control_pub.messages[-1].throttle, 500 / 3200)
        node.on_drive(types.SimpleNamespace(motor_enabled=True,
            motor_torque_raw=800, brake_requested=False))
        node.tick()
        self.assertEqual(node.control_pub.messages[-1].throttle, 0.25)


if __name__ == '__main__':
    unittest.main()
