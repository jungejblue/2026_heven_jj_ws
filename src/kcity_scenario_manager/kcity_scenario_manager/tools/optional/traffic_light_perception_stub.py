#!/usr/bin/env python3
"""Publish CARLA traffic-light state with the unchanged JJ perception contract."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ...competition_common import (
    TriggerBox,
    connect_carla,
    find_ego_vehicle,
    get_trigger,
    load_yaml,
    resolve_traffic_light,
    suite_traffic_light_configs,
    traffic_state_name,
)


@dataclass(frozen=True)
class PerceptionObservation:
    label: str = ""
    class_names: tuple[str, ...] = ()
    confidence: float = 0.0
    detections: int = 0


NO_DETECTION = PerceptionObservation()

# These are existing labels from jj_camera_perception/scripts/labels.py on the
# real stack's origin/exp/week3-day1 branch.  The stub does not invent a new
# enum: it emits one valid detector label and its exact class_names expansion.
_STRAIGHT_STATE_OBSERVATIONS = {
    "Red": PerceptionObservation("1301", ("RED",), 1.0, 1),
    "Yellow": PerceptionObservation("1302", ("ORANGE",), 1.0, 1),
    "Green": PerceptionObservation("1300", ("GREEN",), 1.0, 1),
}
_LEFT_GREEN_OBSERVATION = PerceptionObservation("1305", ("LEFT",), 1.0, 1)


def observation_for_carla_state(
    state_name: str,
    signal_type: str,
) -> PerceptionObservation:
    """Map a current CARLA state to one observation in the real JJ contract."""
    state = str(state_name).split(".")[-1]
    semantic = str(signal_type).strip().lower()
    if semantic not in {"straight", "left"}:
        raise ValueError(f"unsupported signal_type={signal_type!r}; use straight or left")
    if semantic == "left" and state == "Green":
        return _LEFT_GREEN_OBSERVATION
    return _STRAIGHT_STATE_OBSERVATIONS.get(state, NO_DETECTION)


def publisher_is_unique(publisher_count: int) -> bool:
    """The count includes this node's publisher, so exactly one is allowed."""
    return int(publisher_count) == 1


@dataclass
class SignalRuntime:
    key: str
    cfg: dict
    trigger: TriggerBox
    signal_type: str
    actor: object = None
    resolution_error: Optional[str] = None


class TrafficLightPerceptionEngine:
    """Pure trigger/state sampling used by the ROS node and unit tests."""

    def __init__(self, signals: Sequence[SignalRuntime]):
        self.signals = list(signals)

    def active_signals(self, ego_location) -> list[SignalRuntime]:
        if ego_location is None:
            return []
        return [signal for signal in self.signals if signal.trigger.contains(ego_location)]

    def sample(self, ego_location) -> tuple[PerceptionObservation, Optional[str]]:
        active = self.active_signals(ego_location)
        if not active:
            return NO_DETECTION, None
        signal = active[0]
        actor = signal.actor
        if actor is None or not getattr(actor, "is_alive", False):
            return NO_DETECTION, signal.key
        return (
            observation_for_carla_state(traffic_state_name(actor), signal.signal_type),
            signal.key,
        )


def signal_runtimes_from_config(config: dict, suite: str) -> list[SignalRuntime]:
    lights = suite_traffic_light_configs(config, suite)
    if not lights:
        raise RuntimeError(f"scenario.{suite}.traffic_lights must define at least one light")
    runtimes = []
    for key, cfg in lights.items():
        signal_type = str(cfg.get("signal_type", "")).strip().lower()
        if signal_type not in {"straight", "left"}:
            raise RuntimeError(
                f"traffic light {key!r} requires signal_type: straight or left"
            )
        runtimes.append(SignalRuntime(
            key=key,
            cfg=cfg,
            trigger=get_trigger(config, str(cfg["trigger_key"])),
            signal_type=signal_type,
        ))
    return runtimes


class TrafficLightPerceptionStub(Node):
    def __init__(self):
        super().__init__("traffic_light_perception_stub")
        self.declare_parameter("config_file", "")
        self.declare_parameter("suite", "qualifier")
        self.declare_parameter("output_topic", "/jj/perception/traffic_light/state")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("carla_host", "")
        self.declare_parameter("carla_port", 0)
        self.declare_parameter("carla_timeout_sec", 0.0)

        config_file = str(self.get_parameter("config_file").value)
        if not config_file:
            raise RuntimeError("config_file is required")
        suite = str(self.get_parameter("suite").value).strip().lower()
        output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if suite not in {"qualifier", "final"}:
            raise RuntimeError("suite must be qualifier or final")
        if publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")

        # Import lazily so pure mapping/trigger tests do not require the JJ
        # interface overlay.  A real node startup still fails immediately if
        # the exact message package is unavailable.
        from jj_interface.msg import TrafficLightState

        self.message_type = TrafficLightState
        self.config = load_yaml(config_file)
        carla_config = dict(self.config.get("carla", {}))
        carla_host = str(self.get_parameter("carla_host").value).strip()
        carla_port = int(self.get_parameter("carla_port").value)
        carla_timeout_sec = float(self.get_parameter("carla_timeout_sec").value)
        if carla_host:
            carla_config["host"] = carla_host
        if carla_port > 0:
            carla_config["port"] = carla_port
        if carla_timeout_sec > 0.0:
            carla_config["timeout_sec"] = carla_timeout_sec
        self.client, self.world = connect_carla(carla_config)
        self.role_name = str(
            self.config.get("carla", {}).get("ego_role_name", "ego_vehicle")
        )
        self.engine = TrafficLightPerceptionEngine(
            signal_runtimes_from_config(self.config, suite)
        )
        self.ego = None
        self.output_topic = output_topic
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(TrafficLightState, output_topic, qos)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.tick)
        self._last_signature = None
        self._duplicate_reported = False
        self._overlap_reported = False
        self.get_logger().info(
            f"JJ traffic-light stub: suite={suite}, topic={output_topic}, "
            f"rate={publish_rate_hz:.3f} Hz, role_name={self.role_name}"
        )

    def _ensure_ego(self):
        if self.ego is None or not self.ego.is_alive:
            # Exact role only.  A sole unrelated vehicle is never accepted.
            self.ego = find_ego_vehicle(self.world, self.role_name, strict=True)
        return self.ego

    def _ensure_actors(self):
        for signal in self.engine.signals:
            if signal.actor is not None and signal.actor.is_alive:
                continue
            try:
                signal.actor = resolve_traffic_light(self.world, signal.cfg)
            except RuntimeError as exc:
                message = str(exc)
                if signal.resolution_error != message:
                    self.get_logger().warning(f"{signal.key}: {message}")
                    signal.resolution_error = message
                signal.actor = None
                continue
            signal.resolution_error = None
            location = signal.actor.get_location()
            self.get_logger().info(
                f"{signal.key} -> actor_id={signal.actor.id}, "
                f"location=({location.x:.3f}, {location.y:.3f}, {location.z:.3f}), "
                f"signal_type={signal.signal_type}"
            )

    def _publish(self, observation: PerceptionObservation):
        message = self.message_type()
        message.stamp = self.get_clock().now().to_msg()
        message.label = observation.label
        message.class_names = list(observation.class_names)
        message.confidence = observation.confidence
        # One configured CARLA actor represents one selected camera detection.
        message.right_label = observation.label
        message.right_class_names = list(observation.class_names)
        message.right_confidence = observation.confidence
        message.detections = observation.detections
        self.publisher.publish(message)

    def tick(self):
        publisher_count = self.count_publishers(self.output_topic)
        if not publisher_is_unique(publisher_count):
            if not self._duplicate_reported:
                self.get_logger().error(
                    f"duplicate publishers on {self.output_topic}: count={publisher_count}; "
                    "stub publication is suppressed"
                )
                self._duplicate_reported = True
            return
        self._duplicate_reported = False

        self._ensure_actors()
        ego = self._ensure_ego()
        location = ego.get_location() if ego is not None else None
        active = self.engine.active_signals(location)
        if len(active) > 1:
            if not self._overlap_reported:
                keys = ", ".join(signal.key for signal in active)
                self.get_logger().error(
                    f"ego is inside overlapping signal triggers ({keys}); "
                    "publishing no detection"
                )
                self._overlap_reported = True
            observation, key = NO_DETECTION, None
        else:
            self._overlap_reported = False
            observation, key = self.engine.sample(location)

        signature = (key, observation.label, observation.class_names)
        if signature != self._last_signature:
            rendered = list(observation.class_names) if observation.detections else "NO DETECTION"
            self.get_logger().info(f"signal={key or '-'} observation={rendered}")
            self._last_signature = signature
        self._publish(observation)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightPerceptionStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
