"""Declare the HEVEN sensor suite ready after synchronized warm-up samples."""

from __future__ import annotations

from collections import OrderedDict
from functools import partial
from typing import Dict, Set, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Bool

from .topic_contract import READY_TOPIC, SENSOR_TOPICS


Stamp = Tuple[int, int]


def _header_stamp(message) -> Stamp:
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


class SensorGate(Node):
    """Observe equal-rate raw topics without republishing conflicting names."""

    def __init__(self) -> None:
        super().__init__("heven_sensor_gate")

        self.declare_parameter("discard_complete_sets", 5)
        self.discard_complete_sets = int(
            self.get_parameter("discard_complete_sets").value
        )
        if self.discard_complete_sets < 0:
            raise ValueError("discard_complete_sets must be non-negative")

        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_publisher = self.create_publisher(Bool, READY_TOPIC, ready_qos)

        subscriptions = {
            "left_cam": (Image, SENSOR_TOPICS["left_cam"]),
            "right_cam": (Image, SENSOR_TOPICS["right_cam"]),
            "front_cam": (Image, SENSOR_TOPICS["front_cam"]),
            "lidar": (PointCloud2, SENSOR_TOPICS["lidar"]),
            "imu": (Imu, SENSOR_TOPICS["imu"]),
            "gnss": (NavSatFix, SENSOR_TOPICS["gnss"]),
        }
        self.expected_names = frozenset(subscriptions)
        self._subscriptions = []
        for name, (message_type, topic) in subscriptions.items():
            subscription = self.create_subscription(
                message_type,
                topic,
                partial(self._on_measurement, name),
                qos_profile_sensor_data,
            )
            self._subscriptions.append(subscription)

        self.pending: "OrderedDict[Stamp, Set[str]]" = OrderedDict()
        self.completed_stamps: Set[Stamp] = set()
        self.complete_set_count = 0
        self.ready = self.discard_complete_sets == 0
        self._last_clock_ns = None
        self._subscriptions.append(
            self.create_subscription(Clock, "/clock", self._on_clock, 10)
        )
        self.create_timer(1.0, self._publish_state)
        self._publish_state()

        self.get_logger().info(
            "Waiting for six equal-stamp sensor messages. "
            f"The first {self.discard_complete_sets} complete sets are logical "
            "warm-up samples and must not be consumed by perception/recording nodes."
        )
        if self.ready:
            self.get_logger().info(
                "discard_complete_sets=0; HEVEN sensor suite is immediately ready."
            )

    def _on_clock(self, message: Clock) -> None:
        stamp = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if self._last_clock_ns is not None and stamp < self._last_clock_ns:
            self.pending.clear()
            self.completed_stamps.clear()
            self.complete_set_count = 0
            self.ready = self.discard_complete_sets == 0
            self._publish_state()
            self.get_logger().info("Simulation clock restarted; sensor warm-up state reset.")
        self._last_clock_ns = stamp

    def _on_measurement(self, name: str, message) -> None:
        if self.ready:
            return

        if not str(message.header.frame_id).strip():
            self.get_logger().warning(
                f"Ignoring {name} message with an empty frame_id."
            )
            return

        stamp = _header_stamp(message)
        if stamp == (0, 0):
            self.get_logger().warning(
                f"Ignoring {name} message with zero header timestamp."
            )
            return
        if stamp in self.completed_stamps:
            return

        names = self.pending.setdefault(stamp, set())
        names.add(name)

        if names == self.expected_names:
            self.completed_stamps.add(stamp)
            self.pending.pop(stamp, None)
            self.complete_set_count += 1
            self.get_logger().info(
                f"Synchronized warm-up set {self.complete_set_count}/"
                f"{self.discard_complete_sets}: stamp={stamp[0]}.{stamp[1]:09d}"
            )

            if self.complete_set_count >= self.discard_complete_sets:
                self.ready = True
                self.get_logger().info(
                    "HEVEN sensor suite is ready. Consumers may now process "
                    "the original /carla/ego_vehicle sensor topics."
                )
                self._publish_state()

        while len(self.pending) > 100:
            stale_stamp, stale_names = self.pending.popitem(last=False)
            missing = sorted(self.expected_names.difference(stale_names))
            self.get_logger().warning(
                f"Dropping stale incomplete stamp {stale_stamp}; missing={missing}."
            )

    def _publish_state(self) -> None:
        message = Bool()
        message.data = self.ready
        self.ready_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Humble's SIGINT handler may already have invalidated the default
        # context before control reaches this block.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
