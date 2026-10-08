"""Read-only gates for the single-owner qualifier integration launch.

The guard never advances CARLA. It rejects a dirty starting graph, checks the
spawned actors and ROS publishers, and waits for the existing readiness path.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String

from .common.vehicle_spawn import make_before_start_spawn_plan
from .competition_common import load_yaml
from .control_graph import AUTOPILOT_TOPIC, CONTROL_TOPIC, MANUAL_TOPIC
from .csv_route_agent import reconstruct_csv_waypoint
from .route_csv import read_route_csv


EXPECTED_MAP = "heven_kcity/Maps/kcity/kcity"
EXPECTED_SENSORS = {
    "left_cam": "sensor.camera.rgb",
    "right_cam": "sensor.camera.rgb",
    "front_cam": "sensor.camera.rgb",
    "lidar": "sensor.lidar.ray_cast",
    "imu": "sensor.other.imu",
    "gnss": "sensor.other.gnss",
}
RAW_TOPICS = (
    "/carla/ego_vehicle/left_cam/image",
    "/carla/ego_vehicle/right_cam/image",
    "/carla/ego_vehicle/front_cam/image",
    "/carla/ego_vehicle/lidar",
    "/carla/ego_vehicle/imu",
    "/carla/ego_vehicle/gnss",
    "/carla/ego_vehicle/vehicle_status",
)
JJ_TOPICS = (
    "/jj/sensors/imu/data",
    "/jj/sensors/gnss/fix",
    "/jj/sensors/gnss/navpvt",
    "/jj/sensors/gnss/velocity",
)
OTHER_CONTROL_NODES = {
    "bridge_wasd_control", "bridge_autopilot_control", "csv_autopilot",
    "ManualControl", "kcity_ego_manager", "carla_ad_agent",
}
CONTROL_TOPICS = (CONTROL_TOPIC, MANUAL_TOPIC, AUTOPILOT_TOPIC,
                  "/jj/steering/command", "/jj/drive/command")
EVALUATION_NODES = ("heven_carla_sensor_adapter", "qualifier_scenario_manager",
                    "kcity_qualifier_benchmark")
EVALUATION_TOPICS = ("/kcity/scenario/events", "/kcity/benchmark/status")
MONITOR_PHASES = {"monitor", "platform_monitor", "eval_monitor"}
RAW_SAMPLE_PHASES = {"ready", "eval_preflight", "platform_monitor"}


def vehicle_config_error(path, suite_path, world) -> str:
    """Validate the external qualifier spawn against the current CARLA map."""
    with open(path, encoding="utf-8") as stream:
        objects = json.load(stream)["objects"]
    if len(objects) != 1 or objects[0].get("type") != "vehicle.heven.ev":
        return "vehicle_config_file must define exactly one vehicle.heven.ev"
    vehicle = objects[0]
    if vehicle.get("id") != "ego_vehicle" or vehicle.get("sensors") != []:
        return "vehicle_config_file must define ego_vehicle without sensors"
    suite = load_yaml(suite_path)
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    ego_config = load_yaml(str(Path(get_package_share_directory(
        "kcity_scenario_manager")) / "config" / "ego.yaml"))
    plan = make_before_start_spawn_plan(
        world, ego_config["ego"]["spawn"], suite, suite_path
    )
    point = vehicle["spawn_point"]
    actual = plan.transform
    # carla_spawn_objects passes a ROS pose to the Bridge, which reflects Y/yaw.
    xy_error = math.hypot(float(point["x"]) - actual.location.x,
                          -float(point["y"]) - actual.location.y)
    yaw_error = abs((-float(point["yaw"]) - actual.rotation.yaw + 180) % 360 - 180)
    if xy_error > 0.10 or abs(float(point["z"]) - actual.location.z) > 0.10 or yaw_error > 1.0:
        return ("qualifier_vehicle.json differs from START upstream plan: "
                f"xy={xy_error:.3f}m yaw={yaw_error:.3f}deg")
    return ""


def sensor_inventory_error(actors, ego, allow_collision: bool) -> str:
    misplaced = [actor.id for actor in actors if actor.type_id.startswith("sensor.")
                and actor.attributes.get("role_name") in EXPECTED_SENSORS
                and getattr(getattr(actor, "parent", None), "id", None) != ego.id]
    if misplaced:
        return f"named HEVEN sensors are not attached to ego_vehicle: {misplaced}"
    attached = [actor for actor in actors if actor.type_id.startswith("sensor.")
                and getattr(getattr(actor, "parent", None), "id", None) == ego.id]
    found = Counter((actor.attributes.get("role_name", ""), actor.type_id)
                    for actor in attached)
    expected = Counter((name, kind) for name, kind in EXPECTED_SENSORS.items())
    collisions = sum(count for (_, kind), count in found.items()
                     if kind == "sensor.other.collision")
    if allow_collision and collisions == 1:
        found = Counter({key: count for key, count in found.items()
                         if key[1] != "sensor.other.collision"})
    if found != expected or (allow_collision and collisions != 1):
        return f"ego sensor inventory mismatch: expected={dict(expected)}, actual={dict(found)}, collision={collisions}"
    return ""


class IntegrationGuard(Node):
    def __init__(self, phase: str, config_file: str, vehicle_file: str,
                 route_csv: str, host: str, port: int):
        super().__init__(f"qualifier_integration_{phase}")
        self.phase = phase
        self.allow_stopped_sync = False
        self.config_file = config_file
        self.vehicle_file = vehicle_file
        self.route_csv = route_csv
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.ready = False
        self.clock_stamps = []
        self.last_clock_monotonic = None
        self.raw_seen = {}
        self.jj_last_seen = {}
        self.benchmark_status = None
        if phase != "preflight":
            ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, "/heven/sensors_ready", self._on_ready, ready_qos)
            self.create_subscription(Clock, "/clock", self._on_clock, 10)
        # Only the platform monitor retains high-bandwidth camera subscriptions.
        # Evaluation samples inputs before startup; other stages check graph/JJ data.
        if phase in RAW_SAMPLE_PHASES:
            from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
            from carla_msgs.msg import CarlaEgoVehicleStatus
            for topic, kind in zip(RAW_TOPICS, (Image, Image, Image, PointCloud2, Imu, NavSatFix,
                                               CarlaEgoVehicleStatus)):
                self.create_subscription(kind, topic,
                    lambda _msg, name=topic: self.raw_seen.update({name: time.monotonic()}),
                    qos_profile_sensor_data)
        if phase in {"adapter", "armed", "control", "monitor", "eval_monitor"}:
            from geometry_msgs.msg import TwistWithCovarianceStamped
            from sensor_msgs.msg import Imu, NavSatFix
            from ublox_msgs.msg import NavPVT
            types = (Imu, NavSatFix, NavPVT, TwistWithCovarianceStamped)
            for topic, kind in zip(JJ_TOPICS, types):
                self.create_subscription(kind, topic,
                                         lambda _msg, name=topic: self.jj_last_seen.update({name: time.monotonic()}),
                                         qos_profile_sensor_data)
        if phase in {"armed", "control"}:
            self.create_subscription(String, "/kcity/benchmark/status",
                                     self._on_benchmark, 10)

    def _on_ready(self, message):
        self.ready = bool(message.data)

    def _on_clock(self, message):
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if stamp > 0 and (not self.clock_stamps or stamp > self.clock_stamps[-1]):
            self.clock_stamps.append(stamp)
            self.clock_stamps = self.clock_stamps[-2:]
            self.last_clock_monotonic = time.monotonic()

    def _on_benchmark(self, message):
        try:
            self.benchmark_status = json.loads(message.data).get("status")
        except (ValueError, AttributeError):
            self.benchmark_status = None

    def _publisher_count(self, topic):
        return len(self.get_publishers_info_by_topic(topic))

    def _node_count(self, name):
        return sum(node == name for node, _ in self.get_node_names_and_namespaces())

    def check(self) -> str:
        """Return an empty string when this stage's observable invariants hold."""
        if self.world.get_map().name != EXPECTED_MAP:
            return f"expected CARLA map {EXPECTED_MAP}, got {self.world.get_map().name}"
        actors = list(self.world.get_actors())
        egos = [a for a in actors if a.type_id.startswith("vehicle.")
                and a.attributes.get("role_name") == "ego_vehicle"]
        if self.phase == "preflight":
            if self.world.get_settings().synchronous_mode and not self.allow_stopped_sync:
                return "CARLA world is already synchronous before the Bridge starts"
            if egos:
                return f"pre-existing ego_vehicle actors: {len(egos)}"
            if any(a.type_id.startswith("sensor.") and
                   a.attributes.get("role_name") in EXPECTED_SENSORS for a in actors):
                return "pre-existing named HEVEN sensor actors"
            if self._node_count("carla_ros_bridge") or self._publisher_count("/clock"):
                return "a Bridge or /clock publisher already exists"
            if any(self._node_count(name) for name in (
                    "qualifier_scenario_manager", "kcity_qualifier_benchmark",
                    "heven_carla_sensor_adapter", "csv_route_agent", *OTHER_CONTROL_NODES)):
                return "a qualifier integration or control node already exists"
            if any(self._publisher_count(topic) for topic in
                   CONTROL_TOPICS):
                return "pre-existing ego control publisher"
            rows = read_route_csv(self.route_csv)
            if len(rows) != 499:
                return f"expected 499 qualifier route points, got {len(rows)}"
            try:
                self.world.get_blueprint_library().find("vehicle.heven.ev")
            except (IndexError, RuntimeError):
                return "CARLA world lacks vehicle.heven.ev blueprint"
            carla_map = self.world.get_map()
            for row in rows:
                reconstruct_csv_waypoint(carla_map, row, 1.0)
            return vehicle_config_error(self.vehicle_file, self.config_file, self.world)

        if len(egos) != 1 or egos[0].type_id != "vehicle.heven.ev":
            return f"expected one vehicle.heven.ev ego_vehicle, got {[(a.id, a.type_id) for a in egos]}"
        if self._node_count("carla_ros_bridge") != 1 or self._publisher_count("/clock") != 1:
            return "expected exactly one Bridge node and one /clock publisher"
        if not self.world.get_settings().synchronous_mode:
            return "Bridge has not enabled synchronous mode"
        delta = self.world.get_settings().fixed_delta_seconds
        if delta is None or abs(delta - 0.05) > 1e-6:
            return f"expected CARLA fixed_delta_seconds=0.05, got {delta}"
        if self._node_count("heven_warmup_guard"):
            return "vehicle warmup guard has not exited"
        if not self.ready or len(self.clock_stamps) < 2:
            return "waiting for sensors_ready=true and advancing /clock"
        if time.monotonic() - self.last_clock_monotonic > 5.0:
            return "/clock stopped advancing"
        if self._publisher_count("/heven/sensors_ready") != 1:
            return "expected one sensor readiness publisher"
        if any(self._publisher_count(topic) != 1 for topic in RAW_TOPICS):
            return "expected one raw CARLA publisher per camera/LiDAR/IMU/GNSS topic"
        stale = [topic for topic in RAW_TOPICS
                 if time.monotonic() - self.raw_seen.get(topic, 0.0) > 5.0]
        if self.phase in RAW_SAMPLE_PHASES and stale:
            return f"waiting for fresh CARLA sensor samples: {stale}"
        collision_required = self.phase in {"armed", "control", "monitor", "eval_monitor"}
        # The platform remains healthy before, during and after evaluation.
        if self.phase == "platform_monitor":
            collision_required = any(a.type_id == "sensor.other.collision" and
                getattr(getattr(a, "parent", None), "id", None) == egos[0].id for a in actors)
        problem = sensor_inventory_error(actors, egos[0], collision_required)
        if problem:
            return problem
        if any(self._node_count(name) for name in OTHER_CONTROL_NODES):
            return "another ego spawn/control node is running"
        if any(self._publisher_count(topic) for topic in
               CONTROL_TOPICS):
            return "another ROS ego control publisher is running"
        route_owners = self._node_count("csv_route_agent")
        if route_owners > 1 or (self.phase in {"eval_preflight", "adapter", "control"} and route_owners):
            return "unexpected/duplicate csv_route_agent; stop Terminal 4 first"
        if any(self._node_count(name) > 1 for name in EVALUATION_NODES):
            return "duplicate adapter/scenario/benchmark instance"
        if self.phase == "eval_preflight" and (
                any(self._node_count(name) for name in EVALUATION_NODES) or
                any(self._publisher_count(topic) for topic in (*EVALUATION_TOPICS, *JJ_TOPICS))):
            return "evaluation already running; stop Terminal 3 first"
        if self.phase in {"adapter", "armed", "control", "monitor", "eval_monitor"}:
            if self._node_count("heven_carla_sensor_adapter") != 1:
                return "expected one CARLA-to-JJ sensor adapter"
            if any(self._publisher_count(topic) != 1 for topic in JJ_TOPICS):
                return "expected one publisher for each JJ sensor topic"
            stale = [topic for topic in JJ_TOPICS
                     if time.monotonic() - self.jj_last_seen.get(topic, 0.0) > 5.0]
            if stale:
                return f"waiting for fresh JJ sensor samples: {stale}"
        if self.phase in {"armed", "control", "monitor", "eval_monitor"}:
            if (self._node_count("qualifier_scenario_manager") != 1 or
                    self._node_count("kcity_qualifier_benchmark") != 1):
                return "expected one qualifier scenario manager and benchmark"
            if any(self._publisher_count(topic) != 1 for topic in EVALUATION_TOPICS):
                return "expected one publisher per scenario/benchmark output"
        if self.phase in {"armed", "control"} and self.benchmark_status != "armed":
            return f"waiting for benchmark armed state: {self.benchmark_status}"
        if self.phase == "monitor" and self._node_count("csv_route_agent") != 1:
            return "expected exactly one csv_route_agent control owner"
        return ""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("preflight", "ready", "adapter", "armed", "monitor",
        "eval_preflight", "control", "platform_monitor", "eval_monitor"), required=True)
    parser.add_argument("--config-file", required=True)
    parser.add_argument("--vehicle-file", required=True)
    parser.add_argument("--route-csv", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--allow-stopped-sync", action="store_true",
                        help="Explicit respawn only: accept a synchronous world with no advancing frames")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    guard = None
    try:
        guard = IntegrationGuard(args.phase, args.config_file, args.vehicle_file,
                                 args.route_csv, args.host, args.port)
        # Allow DDS discovery before the one-shot clean-world decision.
        if args.phase == "preflight":
            initial_frame = guard.world.get_snapshot().frame
            discovery_deadline = time.monotonic() + 2.0
            while rclpy.ok() and time.monotonic() < discovery_deadline:
                rclpy.spin_once(guard, timeout_sec=0.1)
            guard.allow_stopped_sync = (args.allow_stopped_sync and
                guard.world.get_snapshot().frame == initial_frame)
        deadline = time.monotonic() + args.timeout_sec
        previous = None
        monitor_healthy = False
        monitor_failures = 0
        while rclpy.ok():
            rclpy.spin_once(guard, timeout_sec=0.2)
            problem = guard.check()
            if not problem:
                if previous != "":
                    guard.get_logger().info(f"qualifier integration {args.phase}: PASS")
                if args.phase not in MONITOR_PHASES:
                    return
                monitor_healthy = True
                monitor_failures = 0
            elif problem != previous:
                guard.get_logger().info(f"qualifier integration {args.phase}: {problem}")
            if args.phase in MONITOR_PHASES and problem:
                monitor_failures += 1
                if monitor_healthy and monitor_failures >= 3:
                    raise RuntimeError(problem)
            previous = problem
            if args.phase == "preflight" or (time.monotonic() >= deadline and
                    (args.phase not in MONITOR_PHASES or not monitor_healthy)):
                raise RuntimeError(problem or f"{args.phase} timed out")
    except KeyboardInterrupt:
        return
    except (OSError, RuntimeError, KeyError, ValueError) as exc:
        if guard is not None:
            guard.get_logger().error(f"qualifier integration {args.phase}: {exc}")
        else:
            print(f"qualifier integration {args.phase}: {exc}")
        raise SystemExit(1) from exc
    finally:
        if guard is not None:
            guard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
