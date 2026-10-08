"""Replace the HEVEN CAN driver with a CARLA ROS Bridge actuator endpoint.

The HEVEN controller keeps its existing longitudinal PID. This node does not
subscribe to target_speed_kmh and does not add a second speed controller. Its
Python API client only reads actor/physics feedback; ROS Bridge remains the sole
world-tick owner and applies every VehicleControl command.
"""

import math

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from carla_msgs.msg import CarlaEgoVehicleControl
from jj_interface.msg import DriveCommand, DriveState, SteeringCommand, SteeringState
from std_msgs.msg import Bool

from .control_math import (
    SteeringCalibration, drive_to_carla, front_max_steer_deg,
    road_wheel_to_carla, steering_curve_scale,
)


class ControlAdapter(Node):
    def __init__(self) -> None:
        super().__init__('heven_control_adapter')
        defaults = {
            'host': 'localhost', 'port': 2000, 'timeout': 2.0,
            'role_name': 'ego_vehicle', 'control_period_sec': 0.05,
            'steering_center_deg': -5.5, 'steering_min_deg': -14.5,
            'steering_max_deg': 3.5, 'max_road_wheel_angle_deg': 30.0,
            'max_torque_raw': 3200, 'carla_max_steer_fallback_deg': 30.0,
            'compensate_steering_curve': True, 'require_sensors_ready': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.calibration = SteeringCalibration(
            float(get('steering_center_deg')), float(get('steering_min_deg')),
            float(get('steering_max_deg')), float(get('max_road_wheel_angle_deg')))
        self.max_torque = int(get('max_torque_raw'))
        drive_to_carla(False, 0, False, self.max_torque)  # Validate shared raw units.
        period = float(get('control_period_sec'))
        if not math.isfinite(period) or period <= 0:
            raise ValueError('control_period_sec must be finite and positive')
        self.fallback_angle = float(get('carla_max_steer_fallback_deg'))
        front_max_steer_deg([], self.fallback_angle)
        self.compensate_curve = bool(get('compensate_steering_curve'))
        self.require_sensors_ready = bool(get('require_sensors_ready'))
        self.sensors_ready = not self.require_sensors_ready
        self.role_name = str(get('role_name'))
        self.client = carla.Client(str(get('host')), int(get('port')))
        self.client.set_timeout(float(get('timeout')))
        self.ego = None
        self.physics_max_angle = self.fallback_angle
        self.steering_curve = []
        self.target_sensor = self.calibration.center_deg
        self.auto_enabled = False
        self.last_steer = 0.0
        self.drive_request = DriveCommand()
        self.have_drive = False
        self.last_error = ''
        prefix = f'/carla/{self.role_name}'
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl, prefix + '/vehicle_control_cmd', 1)
        self.steering_pub = self.create_publisher(SteeringState, '/jj/steering/state', 1)
        self.drive_pub = self.create_publisher(DriveState, '/jj/drive/state', 1)
        self.steering_sub = self.create_subscription(
            SteeringCommand, '/jj/steering/command', self.on_steering, 1)
        self.drive_sub = self.create_subscription(
            DriveCommand, '/jj/drive/command', self.on_drive, 1)
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_sub = self.create_subscription(
            Bool, '/heven/sensors_ready', self.on_ready, ready_qos)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            'SIMULATION: HEVEN raw torque -> throttle fraction; existing HEVEN '
            'speed PID retained. Physical wheel feedback is read from CARLA; '
            'CAN ACK/electrical/RPM feedback is unavailable.')

    def on_steering(self, message: SteeringCommand) -> None:
        # Like the CAN driver, disabling Auto does not install a new angle target.
        if message.auto_enabled:
            self.calibration.sensor_to_road_wheel(message.external_steering_sensor_angle_deg)
            self.target_sensor = max(self.calibration.minimum_deg, min(
                self.calibration.maximum_deg, message.external_steering_sensor_angle_deg))
        self.auto_enabled = bool(message.auto_enabled)

    def on_drive(self, message: DriveCommand) -> None:
        self.drive_request = message
        self.have_drive = True

    def on_ready(self, message: Bool) -> None:
        self.sensors_ready = bool(message.data) or not self.require_sensors_ready

    def find_ego(self) -> bool:
        if self.ego is not None and self.ego.is_alive:
            return True
        self.ego = None
        world = self.client.get_world()
        vehicles = [actor for actor in world.get_actors().filter('vehicle.*')
                    if actor.attributes.get('role_name') == self.role_name]
        if not vehicles:
            return False
        if len(vehicles) != 1:
            raise RuntimeError(f'Expected one vehicle with role_name={self.role_name}')
        self.ego = vehicles[0]
        physics = self.ego.get_physics_control()
        self.physics_max_angle = front_max_steer_deg(
            [wheel.max_steer_angle for wheel in physics.wheels], self.fallback_angle)
        self.steering_curve = [(point.x, point.y) for point in physics.steering_curve]
        steering_curve_scale(0, self.steering_curve)  # Check the physics curve once.
        self.get_logger().info(
            f'Connected to CARLA actor {self.ego.id}; front max steering '
            f'{self.physics_max_angle:.3f} deg, HEVEN assumed max '
            f'{self.calibration.max_road_wheel_deg:.3f} deg')
        return True

    def tick(self) -> None:
        try:
            if not self.find_ego():
                self.publish_states(False, float('nan'), float('inf'), 'SIMULATION: waiting for ego vehicle')
                return
            velocity = self.ego.get_velocity()
            forward = self.ego.get_transform().get_forward_vector()
            forward_speed_kmh = 3.6 * abs(
                velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z)
            if self.auto_enabled:
                angle = self.calibration.sensor_to_road_wheel(self.target_sensor)
                scale = steering_curve_scale(forward_speed_kmh, self.steering_curve) \
                    if self.compensate_curve else 1.0
                self.last_steer = road_wheel_to_carla(angle, self.physics_max_angle, scale)
            drive = drive_to_carla(
                self.drive_request.motor_enabled, self.drive_request.motor_torque_raw,
                self.drive_request.brake_requested, self.max_torque)
            # The original HEVEN localization initializer emits disabled drive
            # commands while waiting for sensors. Those include brake=False;
            # forwarding them early would fight the warmup guard's held brake.
            if self.have_drive and self.sensors_ready:
                command = CarlaEgoVehicleControl()
                command.header.stamp = self.get_clock().now().to_msg()
                command.throttle = drive.throttle
                command.steer = self.last_steer
                command.brake = drive.brake
                # Warmup guard leaves the parking brake on. VCU braking maps to
                # service brake; its release must also clear CARLA's parking brake.
                command.hand_brake = False
                command.reverse = False
                command.gear = 1
                command.manual_gear_shift = False
                self.control_pub.publish(command)
            try:
                front_left = self.ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
                front_right = self.ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
                # The HEVEN single sensor is represented by the mean physical
                # front-wheel angle, not by get_control().steer command readback.
                measured = self.calibration.road_wheel_to_sensor((front_left + front_right) / 2)
                feedback_age = 0.0
                status = 'SIMULATION: CARLA wheel feedback; no CAN or ECU PID ACK'
            except (RuntimeError, AttributeError, ValueError) as error:
                measured, feedback_age = float('nan'), float('inf')
                status = f'SIMULATION: physical wheel feedback unavailable: {error}'
            self.publish_states(True, measured, feedback_age, status)
            self.last_error = ''
        except (RuntimeError, AttributeError, ValueError) as error:
            reason = str(error)
            if reason != self.last_error:
                self.get_logger().warning(f'CARLA control adapter: {reason}')
                self.last_error = reason
            self.ego = None
            self.publish_states(False, float('nan'), float('inf'), f'SIMULATION: {reason}')

    def publish_states(self, ready: bool, measured: float, age: float, status: str) -> None:
        steering = SteeringState()
        steering.driver_ready = ready
        steering.pid_ack_received = False
        steering.pid_ack_value = 0
        steering.auto_commanded = ready and self.sensors_ready and self.auto_enabled
        steering.target_external_steering_sensor_angle_deg = self.target_sensor
        steering.current_external_steering_sensor_angle_deg = measured
        steering.sensor_feedback_age_sec = age
        steering.driver_status = status
        self.steering_pub.publish(steering)
        request = self.drive_request
        effective = drive_to_carla(ready and self.sensors_ready and request.motor_enabled,
                                   request.motor_torque_raw, request.brake_requested,
                                   self.max_torque)
        state = DriveState()
        state.motor_enable_commanded = effective.motor_enabled
        state.motor_torque_raw_commanded = effective.torque_raw
        state.brake_requested = request.brake_requested
        # CARLA has no VCU CAN response, electric bus measurements or motor RPM.
        state.vcu_feedback_received = False
        state.vcu_enable_feedback = False
        state.vcu_mode_feedback = 0
        state.vcu_torque_raw_feedback = 0
        state.vcu_feedback_age_sec = float('inf')
        state.motor_feedback_valid = False
        state.motor_bus_voltage_v = float('nan')
        state.motor_bus_current_a = float('nan')
        state.motor_phase_current_a = float('nan')
        state.motor_rpm = float('nan')
        state.motor_feedback_age_sec = float('inf')
        state.driver_status = status
        self.drive_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
