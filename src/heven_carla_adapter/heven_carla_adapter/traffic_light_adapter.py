"""Optional CARLA traffic-light observer for the HEVEN controller.

Publication is disabled by default. The scenario manager remains responsible
for changing light phases. Run this observer instead of the older perception
stub or the camera-based traffic_light_node on the same output topic.
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml

from .traffic_math import resolve_light, sample_signals, signals_from_config, simulation_stamp


class TrafficLightAdapter(Node):
    def __init__(self):
        super().__init__("traffic_light_adapter")
        from ament_index_python.packages import get_package_share_directory
        from jj_interface.msg import TrafficLightState

        default_config = str(Path(get_package_share_directory("kcity_scenario_manager")) /
                             "config" / "qualifier.yaml")
        for name, value in {
            "config_file": default_config,
            "output_topic": "/jj/perception/traffic_light/state",
            "publish_enabled": False,
            "publish_rate_hz": 20.0,
            "draw_debug_boxes": True,
            "carla_host": "127.0.0.1",
            "carla_port": 2000,
            "carla_timeout_sec": 5.0,
            "ego_role_name": "ego_vehicle",
        }.items():
            self.declare_parameter(name, value)
        path = Path(str(self.get_parameter("config_file").value)).expanduser()
        with path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.signals = signals_from_config(self.config)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        import carla  # Runtime dependency only; geometry tests do not import it.
        self.carla = carla
        self.client = carla.Client(str(self.get_parameter("carla_host").value),
                                   int(self.get_parameter("carla_port").value))
        self.client.set_timeout(float(self.get_parameter("carla_timeout_sec").value))
        self.world = self.client.get_world()
        self.ego_role = str(self.get_parameter("ego_role_name").value)
        self.ego = None
        self.message_type = TrafficLightState
        self.publisher = None
        self._binding_errors = {}
        self._last_signature = None
        self._next_draw_time = 0.0
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"traffic regions: {path}; publish_enabled="
            f"{bool(self.get_parameter('publish_enabled').value)}; "
            "light phases are observed only"
        )

    def _ensure_actors(self):
        actors = self.world.get_actors()
        if self.ego is None or not self.ego.is_alive:
            self.ego = next((actor for actor in actors.filter("vehicle.*")
                             if actor.attributes.get("role_name") == self.ego_role), None)
        lights = list(actors.filter("traffic.traffic_light*"))
        for signal in self.signals:
            if signal.actor is not None and signal.actor.is_alive:
                continue
            try:
                signal.actor = resolve_light(lights, signal.config)
            except (ValueError, KeyError) as exc:
                message = str(exc)
                if self._binding_errors.get(signal.key) != message:
                    self.get_logger().warning(f"{signal.key}: {message}")
                    self._binding_errors[signal.key] = message
                signal.actor = None
            else:
                self._binding_errors.pop(signal.key, None)
                self.get_logger().info(f"{signal.key} bound to CARLA actor {signal.actor.id}")

    def _draw(self):
        carla = self.carla
        for signal in self.signals:
            box = signal.trigger
            if not box.enabled:
                continue
            bounds = carla.BoundingBox(carla.Location(*box.center), carla.Vector3D(*box.extent))
            self.world.debug.draw_box(bounds, carla.Rotation(yaw=box.yaw_deg),
                                      thickness=0.05, color=carla.Color(0, 255, 255),
                                      life_time=1.2)
            cx, cy, cz = box.center
            self.world.debug.draw_string(carla.Location(cx, cy, cz + box.extent[2] + 0.5),
                                         signal.key, color=carla.Color(255, 255, 255),
                                         life_time=1.2)
            start, end = box.exit_line()
            self.world.debug.draw_line(carla.Location(*start), carla.Location(*end),
                                       thickness=0.15, color=carla.Color(255, 0, 0),
                                       life_time=1.2)

    def tick(self):
        elapsed = float(self.world.get_snapshot().timestamp.elapsed_seconds)
        if bool(self.get_parameter("draw_debug_boxes").value):
            if elapsed >= self._next_draw_time:
                self._draw()
                self._next_draw_time = elapsed + 1.0
        enabled = bool(self.get_parameter("publish_enabled").value)
        if not enabled:
            if self.publisher is not None:
                self.destroy_publisher(self.publisher)
                self.publisher = None
            return
        if self.publisher is None:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.create_publisher(
                self.message_type, str(self.get_parameter("output_topic").value), qos)
        self._ensure_actors()
        location = self.ego.get_location() if self.ego is not None else None
        observation, key = sample_signals(self.signals, location)
        message = self.message_type()
        message.stamp.sec, message.stamp.nanosec = simulation_stamp(elapsed)
        message.label = observation.label
        message.class_names = list(observation.class_names)
        message.confidence = observation.confidence
        message.right_label = observation.label
        message.right_class_names = list(observation.class_names)
        message.right_confidence = observation.confidence
        message.detections = observation.detections
        self.publisher.publish(message)
        signature = key, observation.label
        if signature != self._last_signature:
            self.get_logger().info(f"signal={key or '-'} classes={list(observation.class_names)}")
            self._last_signature = signature


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficLightAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
