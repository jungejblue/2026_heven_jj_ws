"""Hold the ego vehicle stopped for a configured amount of simulation time."""

from __future__ import annotations

import sys
import time
from typing import Optional

import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from .topic_contract import VEHICLE_CONTROL_TOPIC, VEHICLE_STATUS_TOPIC


def _clock_nanoseconds(message: Clock) -> int:
    return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)


class WarmupGuard(Node):
    """Start timing only after both vehicle status and simulation clock exist."""

    def __init__(self) -> None:
        super().__init__("heven_warmup_guard")

        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("startup_timeout_seconds", 30.0)

        self.warmup_seconds = float(self.get_parameter("warmup_seconds").value)
        self.startup_timeout_seconds = float(
            self.get_parameter("startup_timeout_seconds").value
        )
        if self.warmup_seconds <= 0.0:
            raise ValueError("warmup_seconds must be positive")

        self.publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            VEHICLE_CONTROL_TOPIC,
            10,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(
            CarlaEgoVehicleStatus,
            VEHICLE_STATUS_TOPIC,
            self._on_vehicle_status,
            10,
        )
        self.create_timer(0.05, self._on_timer)

        self.created_monotonic = time.monotonic()
        self.latest_clock_ns: Optional[int] = None
        self.start_clock_ns: Optional[int] = None
        self.vehicle_seen = False
        self.completed = False

        self.get_logger().info(
            "Waiting for ego_vehicle status; the vehicle will then be held "
            f"for {self.warmup_seconds:.3f} seconds of simulation time."
        )

    def _on_clock(self, message: Clock) -> None:
        stamp = _clock_nanoseconds(message)
        if self.latest_clock_ns is not None and stamp < self.latest_clock_ns:
            self.start_clock_ns = None
            self.completed = False
            self.get_logger().info("Simulation clock restarted; vehicle warm-up timing reset.")
        self.latest_clock_ns = stamp
        self._try_start()

    def _on_vehicle_status(self, _: CarlaEgoVehicleStatus) -> None:
        if not self.vehicle_seen:
            self.vehicle_seen = True
            self.get_logger().info("ego_vehicle status detected; applying full brake.")
        self._try_start()

    def _try_start(self) -> None:
        if (
            self.vehicle_seen
            and self.latest_clock_ns is not None
            and self.start_clock_ns is None
        ):
            self.start_clock_ns = self.latest_clock_ns
            self.get_logger().info(
                f"Vehicle warm-up started at simulation time "
                f"{self.start_clock_ns / 1e9:.6f} s."
            )

    def _publish_brake(self) -> None:
        command = CarlaEgoVehicleControl()
        command.throttle = 0.0
        command.steer = 0.0
        command.brake = 1.0
        command.hand_brake = True
        command.reverse = False
        command.gear = 0
        command.manual_gear_shift = False
        self.publisher.publish(command)

    def _on_timer(self) -> None:
        if self.vehicle_seen:
            self._publish_brake()

        if self.start_clock_ns is None or self.latest_clock_ns is None:
            return

        elapsed_seconds = (self.latest_clock_ns - self.start_clock_ns) / 1e9
        if elapsed_seconds + 1e-9 >= self.warmup_seconds:
            self._publish_brake()
            self.completed = True
            self.get_logger().info(
                f"Vehicle warm-up completed after {elapsed_seconds:.3f} "
                "seconds of simulation time. The brake remains applied."
            )

    def startup_timed_out(self) -> bool:
        if self.start_clock_ns is not None:
            return False
        return (
            time.monotonic() - self.created_monotonic
            >= self.startup_timeout_seconds
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WarmupGuard()
    exit_code = 0

    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.startup_timed_out():
                node.get_logger().error(
                    "Timed out waiting for /clock and ego vehicle status. "
                    "Sensors will not be spawned."
                )
                exit_code = 2
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv)
