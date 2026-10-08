"""CARLA bridge sensors -> HEVEN contracts, without publishing GT odometry.

The upstream bridge has already converted IMU and cloud axes to ROS. This
node only changes their frame aliases. Raw absolute GNSS coordinates remain
unchanged. CARLA supplies measured velocity; the original HEVEN localizer
initializes heading from its GNSS displacement and mounted IMU measurements.
"""

from __future__ import annotations

from copy import deepcopy
from functools import partial
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from carla_msgs.msg import CarlaEgoVehicleStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import Bool
from ublox_msgs.msg import NavPVT

from .geodesy import finite_velocity_enu, project_latlon
from .sensor_math import navpvt_values, stamp_nanoseconds, yaw_from_quaternion


class CarlaSensorAdapter(Node):
    def __init__(self) -> None:
        super().__init__("heven_sensor_adapter")
        defaults = {
            "host": "localhost", "port": 2000, "ego_role_name": "ego_vehicle",
            "origin_latitude_deg": 37.2388873, "origin_longitude_deg": 126.7729325,
            "earth_radius_m": 6378135.0, "map_frame": "map",
            "imu_frame": "imu_link", "gnss_frame": "gnss_link", "lidar_frame": "os_lidar",
            "raw_imu_topic": "/carla/ego_vehicle/imu",
            "raw_gnss_topic": "/carla/ego_vehicle/gnss",
            "raw_vehicle_status_topic": "/carla/ego_vehicle/vehicle_status",
            "raw_lidar_topic": "/carla/ego_vehicle/lidar",
            "require_sensors_ready": True,
            "h_acc_mm": 10,
            "status_timeout_sec": 0.5,
        }
        for camera, raw_name in (("left", "left_cam"), ("middle", "front_cam"), ("right", "right_cam")):
            defaults[f"raw_{camera}_image_topic"] = f"/carla/ego_vehicle/{raw_name}/image"
            defaults[f"raw_{camera}_camera_info_topic"] = f"/carla/ego_vehicle/{raw_name}/camera_info"
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        self.host, self.port = str(self.params["host"]), int(self.params["port"])
        self.role = str(self.params["ego_role_name"])
        self.origin_lat = float(self.params["origin_latitude_deg"])
        self.origin_lon = float(self.params["origin_longitude_deg"])
        self.radius = float(self.params["earth_radius_m"])
        project_latlon(self.origin_lat, self.origin_lon, self.origin_lat, self.origin_lon, self.radius)
        self.h_acc_mm = int(self.params["h_acc_mm"])
        self.status_timeout = float(self.params["status_timeout_sec"])
        if not 0 < self.h_acc_mm <= 4294967295:
            raise ValueError("h_acc_mm must be a positive uint32 millimetre value")
        if not math.isfinite(self.status_timeout) or self.status_timeout <= 0:
            raise ValueError("status_timeout_sec must be positive")
        self.ready = not bool(self.params["require_sensors_ready"])
        self._last_clock = 0
        self._last_gnss_stamp = 0
        self._status = None
        self._status_received = 0.0
        self._client = self._world = self._map = self._ego = self._carla = None
        self._last_location = None
        self._next_connect = 0.0
        self._last_world_check = 0.0
        self._last_warning = 0.0
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # RELIABLE outputs also match HEVEN's BEST_EFFORT SensorDataQoS
        # consumers, and let ordinary ROS subscribers inspect every alias.
        output_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/heven/sensors_ready", self._on_ready, ready_qos)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(CarlaEgoVehicleStatus, str(self.params["raw_vehicle_status_topic"]),
                                 self._on_status, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/jj/sensors/imu/data", output_qos)
        self.fix_pub = self.create_publisher(NavSatFix, "/jj/sensors/gnss/fix", output_qos)
        self.pvt_pub = self.create_publisher(NavPVT, "/jj/sensors/gnss/navpvt", output_qos)
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped,
                                                 "/jj/sensors/gnss/velocity", output_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/jj/sensors/lidar/points", output_qos)
        self.create_subscription(Imu, str(self.params["raw_imu_topic"]), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, str(self.params["raw_gnss_topic"]), self._on_gnss, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(self.params["raw_lidar_topic"]), self._on_cloud, qos_profile_sensor_data)
        self.camera_pubs = {}
        for camera in ("left", "middle", "right"):
            for kind, message_type, output_suffix in (("image", Image, "image_raw"),
                                                      ("camera_info", CameraInfo, "camera_info")):
                publisher = self.create_publisher(message_type,
                    f"/jj/sensors/camera/{camera}/{output_suffix}", output_qos)
                self.camera_pubs[(camera, kind)] = publisher
                self.create_subscription(message_type, str(self.params[f"raw_{camera}_{kind}_topic"]),
                    partial(self._on_camera, camera, kind), qos_profile_sensor_data)
        self.get_logger().info("CARLA -> HEVEN sensors; simulated RTK FIX, no injected noise; "
                               "original HEVEN heading initialization and localization remain active.")

    def _on_ready(self, message: Bool) -> None:
        self.ready = bool(message.data) or not bool(self.params["require_sensors_ready"])

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock.sec * 1000000000 + message.clock.nanosec
        if stamp < self._last_clock:
            self._last_gnss_stamp = 0
            self._status = None
            self._client = self._world = self._map = self._ego = None
            self._last_location = None
            self._next_connect = 0.0
            self.ready = not bool(self.params["require_sensors_ready"])
        self._last_clock = stamp

    @staticmethod
    def _stamp(message) -> int:
        return stamp_nanoseconds(message.header.stamp.sec, message.header.stamp.nanosec)

    def _on_status(self, message: CarlaEgoVehicleStatus) -> None:
        self._status, self._status_received = message, time.monotonic()

    def _forward(self, message, frame: str, publisher) -> None:
        if not self.ready:
            return
        try:
            self._stamp(message)
        except (ValueError, OverflowError):
            return
        output = deepcopy(message)
        output.header.frame_id = frame
        publisher.publish(output)

    def _on_imu(self, message: Imu) -> None:
        self._forward(message, str(self.params["imu_frame"]), self.imu_pub)

    def _on_cloud(self, message: PointCloud2) -> None:
        # PointCloud2 x/y/z/intensity and bytes are already ROS convention.
        self._forward(message, str(self.params["lidar_frame"]), self.cloud_pub)

    def _on_camera(self, camera: str, kind: str, message) -> None:
        # Upstream camera frame already has the optical-axis rotation.
        self._forward(message, f"{camera}_optical_frame", self.camera_pubs[(camera, kind)])

    def _geolocate(self, x: float, y: float, z: float):
        location = self._carla.Location(x=float(x), y=float(y), z=float(z))
        geo = self._map.transform_to_geolocation(location)
        return (geo.latitude, geo.longitude, geo.altitude)

    def _carla_motion(self):
        now = time.monotonic()
        if self._client is None:
            if now < self._next_connect:
                return None
            self._next_connect = now + 1.0
            import carla  # CARLA is needed only by the ROS node, not pure unit tests.
            self._carla = carla
            self._client = carla.Client(self.host, self.port)
            self._client.set_timeout(2.0)
            self._world = self._client.get_world()
            self._map = self._world.get_map()
        if now - self._last_world_check >= 1.0:
            self._world = self._client.get_world()
            self._map = self._world.get_map()
            self._last_world_check = now
        if self._ego is None or not self._ego.is_alive:
            candidates = [actor for actor in self._world.get_actors().filter("vehicle.*")
                          if actor.attributes.get("role_name") == self.role]
            if len(candidates) != 1:
                raise RuntimeError(f"Expected exactly one CARLA vehicle role={self.role}, got {len(candidates)}")
            self._ego = candidates[0]
        location, velocity = self._ego.get_location(), self._ego.get_velocity()
        xyz = (location.x, location.y, location.z)
        self._last_location = xyz
        enu = finite_velocity_enu(xyz, (velocity.x, velocity.y, velocity.z), self._geolocate,
                                  self.origin_lat, self.radius)
        return enu

    def _status_motion(self):
        # A fresh measured scalar speed/orientation may bridge an actor RPC
        # hiccup. Use the same geographic Jacobian, never a target speed.
        if (self._status is None or self._last_location is None or self._map is None or
                time.monotonic() - self._status_received > self.status_timeout):
            return None
        message = self._status
        q = message.orientation
        yaw_ros = yaw_from_quaternion((q.x, q.y, q.z, q.w))
        speed = float(message.velocity)
        if not math.isfinite(speed) or speed < 0:
            return None
        if message.control.reverse:
            speed = -speed
        forward = (math.cos(yaw_ros), -math.sin(yaw_ros), 0.0)
        velocity = tuple(speed * component for component in forward)
        enu = finite_velocity_enu(self._last_location, velocity, self._geolocate,
                                  self.origin_lat, self.radius)
        return enu

    def _motion(self):
        try:
            return self._carla_motion()
        except (RuntimeError, OSError, ValueError, ImportError, AttributeError) as exc:
            now = time.monotonic()
            if now - self._last_warning > 2.0:
                self.get_logger().warning(f"Waiting for CARLA measured motion: {exc}")
                self._last_warning = now
            # Drop the actor handle but retain the map and last location for
            # the bounded measured-status fallback.
            self._ego = None
            try:
                fallback = self._status_motion()
            except (RuntimeError, ValueError, AttributeError):
                fallback = None
            if fallback is None:
                self._client = None
            return fallback

    def _on_gnss(self, message: NavSatFix) -> None:
        if not self.ready:
            return
        try:
            stamp = self._stamp(message)
            if stamp <= self._last_gnss_stamp:
                return
            velocity_enu = self._motion()
            if velocity_enu is None:
                return
            fields = navpvt_values(message.header.stamp.sec, message.header.stamp.nanosec,
                message.latitude, message.longitude, message.altitude, velocity_enu, self.h_acc_mm)
        except (ValueError, OverflowError):
            return
        self._last_gnss_stamp = stamp
        sigma = self.h_acc_mm * 0.001
        fix = deepcopy(message)
        fix.header.frame_id = str(self.params["gnss_frame"])
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance = [sigma * sigma, 0.0, 0.0, 0.0, sigma * sigma, 0.0, 0.0, 0.0, sigma * sigma]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        pvt = NavPVT()
        for name, value in fields.items():
            setattr(pvt, name, value)
        velocity = TwistWithCovarianceStamped()
        velocity.header = deepcopy(message.header)
        velocity.header.frame_id = str(self.params["map_frame"])
        velocity.twist.twist.linear.x, velocity.twist.twist.linear.y, velocity.twist.twist.linear.z = velocity_enu
        # Unknown/unused angular components have a large covariance. Positive
        # linear covariance avoids presenting virtual measurements as exact.
        for index in (0, 7, 14):
            velocity.twist.covariance[index] = 0.0001
        for index in (21, 28, 35):
            velocity.twist.covariance[index] = 1000000.0
        self.fix_pub.publish(fix)
        self.pvt_pub.publish(pvt)
        self.velocity_pub.publish(velocity)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaSensorAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
