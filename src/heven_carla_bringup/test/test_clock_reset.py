"""Exercise simulation resets through the real bringup node methods.

Only the ROS endpoints are mocked; set accounting, clock callbacks and brake
commands execute the installed node implementation without a running ROS graph.
"""

from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(
            frame_id="sensor", stamp=types.SimpleNamespace(sec=0, nanosec=0))
        self.clock = types.SimpleNamespace(sec=0, nanosec=0)
        self.data = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


def load_node_module(filename, parameter_overrides=None):
    """Load actual node code with lightweight, per-test ROS endpoint stubs."""
    overrides = dict(parameter_overrides or {})

    class Node:
        def __init__(self, name):
            self.parameters = {}
            self.subscriptions = []
            self.timers = []

        def declare_parameter(self, name, default):
            self.parameters[name] = overrides.get(name, default)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, message_type, topic, qos):
            return Publisher()

        def create_subscription(self, message_type, topic, callback, qos):
            subscription = types.SimpleNamespace(topic=topic, callback=callback)
            self.subscriptions.append(subscription)
            return subscription

        def create_timer(self, period, callback):
            timer = types.SimpleNamespace(period=period, callback=callback)
            self.timers.append(timer)
            return timer

        def get_logger(self):
            return types.SimpleNamespace(info=lambda _: None, warning=lambda _: None)

    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
        "rosgraph_msgs", "rosgraph_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
        "std_msgs", "std_msgs.msg", "carla_msgs", "carla_msgs.msg")}
    modules["rclpy.node"].Node = Node
    modules["rclpy.executors"].ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {})
    qos = modules["rclpy.qos"]
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.qos_profile_sensor_data = object()
    modules["rosgraph_msgs.msg"].Clock = Message
    modules["std_msgs.msg"].Bool = Message
    for name in ("Image", "Imu", "NavSatFix", "PointCloud2"):
        setattr(modules["sensor_msgs.msg"], name, Message)
    for name in ("CarlaEgoVehicleControl", "CarlaEgoVehicleStatus"):
        setattr(modules["carla_msgs.msg"], name, Message)

    path = Path(__file__).resolve().parents[1] / "heven_carla_bringup" / filename
    name = "heven_carla_bringup._clock_test_" + path.stem
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def clock(sec, nanosec=0):
    message = Message()
    message.clock.sec, message.clock.nanosec = sec, nanosec
    return message


def measurement(sec, nanosec=0):
    message = Message()
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    return message


def complete_set(node, sec, nanosec=0):
    for name in node.expected_names:
        node._on_measurement(name, measurement(sec, nanosec))


class SensorGateResetTests(unittest.TestCase):
    def test_ready_gate_requires_five_fresh_sets_after_clock_rollback(self):
        node = load_node_module("sensor_gate.py").SensorGate()
        self.assertIn("/clock", [item.topic for item in node.subscriptions])
        node._on_clock(clock(100))
        for index in range(1, 6):
            complete_set(node, 100, index * 50_000_000)
        self.assertTrue(node.ready)
        self.assertTrue(node.ready_publisher.messages[-1].data)

        node._on_clock(clock(1))
        self.assertFalse(node.ready)
        self.assertEqual(node.complete_set_count, 0)
        self.assertEqual(node.completed_stamps, set())
        self.assertFalse(node.ready_publisher.messages[-1].data)
        # A periodic heartbeat must not restore the previous epoch's True.
        node._publish_state()
        self.assertFalse(node.ready_publisher.messages[-1].data)

        for index in range(1, 5):
            complete_set(node, 1, index * 50_000_000)
            complete_set(node, 1, index * 50_000_000)  # Duplicate delivery.
            self.assertEqual(node.complete_set_count, index)
            self.assertFalse(node.ready)
        complete_set(node, 1, 250_000_000)
        self.assertTrue(node.ready)
        self.assertTrue(node.ready_publisher.messages[-1].data)

    def test_partial_sets_and_completed_stamp_history_do_not_cross_epochs(self):
        node = load_node_module("sensor_gate.py", {"discard_complete_sets": 3}).SensorGate()
        node._on_clock(clock(0, 300_000_000))
        complete_set(node, 0, 50_000_000)
        names = sorted(node.expected_names)
        node._on_measurement(names[0], measurement(0, 100_000_000))
        self.assertEqual(node.complete_set_count, 1)
        self.assertTrue(node.pending)

        node._on_clock(clock(0))
        self.assertEqual(node.complete_set_count, 0)
        self.assertFalse(node.pending)
        # Reused timestamps in the new epoch must count again.
        complete_set(node, 0, 50_000_000)
        self.assertEqual(node.complete_set_count, 1)
        for name in names[1:]:
            node._on_measurement(name, measurement(0, 100_000_000))
        self.assertEqual(node.complete_set_count, 1)
        self.assertFalse(node.ready)
        node._on_measurement(names[0], measurement(0, 100_000_000))
        self.assertEqual(node.complete_set_count, 2)
        complete_set(node, 0, 150_000_000)
        self.assertTrue(node.ready)

    def test_monotonic_and_equal_clocks_preserve_warmup_progress(self):
        node = load_node_module("sensor_gate.py").SensorGate()
        node._on_clock(clock(1))
        complete_set(node, 1, 50_000_000)
        node._on_clock(clock(1))
        node._on_clock(clock(2))
        self.assertEqual(node.complete_set_count, 1)
        self.assertEqual(node.completed_stamps, {(1, 50_000_000)})

    def test_zero_discard_stays_immediately_ready_after_clock_rollback(self):
        node = load_node_module("sensor_gate.py", {"discard_complete_sets": 0}).SensorGate()
        self.assertTrue(node.ready)
        node._on_clock(clock(100))
        node._on_clock(clock(1))
        self.assertTrue(node.ready)
        self.assertTrue(all(message.data for message in node.ready_publisher.messages))


class WarmupGuardResetTests(unittest.TestCase):
    def test_rollback_restarts_two_second_hold_and_keeps_full_brake(self):
        node = load_node_module("warmup_guard.py").WarmupGuard()
        node._on_clock(clock(100))
        node._on_vehicle_status(Message())
        node._on_clock(clock(101))
        node._on_timer()
        self.assertFalse(node.completed)

        node._on_clock(clock(1))
        self.assertEqual(node.start_clock_ns, 1_000_000_000)
        node._on_clock(clock(2, 999_000_000))
        node._on_timer()
        self.assertFalse(node.completed)
        node._on_clock(clock(3))
        node._on_timer()
        self.assertTrue(node.completed)
        self.assertTrue(node.publisher.messages)
        for command in node.publisher.messages:
            self.assertEqual((command.throttle, command.steer, command.brake), (0.0, 0.0, 1.0))
            self.assertTrue(command.hand_brake)
            self.assertFalse(command.reverse)

    def test_first_and_monotonic_clocks_preserve_existing_start_conditions(self):
        node = load_node_module("warmup_guard.py").WarmupGuard()
        node._on_clock(clock(100))
        node._on_timer()
        self.assertIsNone(node.start_clock_ns)
        self.assertFalse(node.publisher.messages)
        node._on_vehicle_status(Message())
        node._on_clock(clock(100))
        node._on_clock(clock(101, 999_000_000))
        node._on_timer()
        self.assertFalse(node.completed)
        self.assertEqual(node.start_clock_ns, 100_000_000_000)
        node._on_clock(clock(102))
        node._on_timer()
        self.assertTrue(node.completed)


if __name__ == "__main__":
    unittest.main()
