"""HEVEN command units and CARLA 0.9.15 actuator conversions.

The torque conversion deliberately preserves the HEVEN speed PID output as a
linear throttle fraction. It is not a conversion from a calibrated torque in Nm.
Functions in this module require neither ROS nor a running simulator.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


@dataclass(frozen=True)
class SteeringCalibration:
    center_deg: float = -5.5
    minimum_deg: float = -14.5
    maximum_deg: float = 3.5
    max_road_wheel_deg: float = 30.0

    def __post_init__(self) -> None:
        values = (self.center_deg, self.minimum_deg, self.maximum_deg,
                  self.max_road_wheel_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Steering calibration must be finite')
        if not self.minimum_deg < self.center_deg < self.maximum_deg:
            raise ValueError('Steering minimum < center < maximum is required')
        if self.max_road_wheel_deg <= 0:
            raise ValueError('Maximum road-wheel angle must be positive')

    def sensor_to_road_wheel(self, sensor_deg: float) -> float:
        """Invert the controller's mapping; positive road-wheel degrees are right."""
        sensor = clamp(_finite(sensor_deg, 'Sensor angle'),
                       self.minimum_deg, self.maximum_deg)
        offset = sensor - self.center_deg
        span = (self.center_deg - self.minimum_deg if offset < 0 else
                self.maximum_deg - self.center_deg)
        return self.max_road_wheel_deg * offset / span

    def road_wheel_to_sensor(self, road_wheel_deg: float) -> float:
        """Represent measured wheel angle without clipping away feedback excursions."""
        normalized = _finite(road_wheel_deg, 'Road-wheel angle') / self.max_road_wheel_deg
        span = (self.center_deg - self.minimum_deg if normalized < 0 else
                self.maximum_deg - self.center_deg)
        return self.center_deg + normalized * span


def front_max_steer_deg(wheel_limits: Iterable[float], fallback_deg: float) -> float:
    """CARLA wheel order is FL, FR, BL, BR; use front wheel physics limits."""
    limits = [float(value) for value in list(wheel_limits)[:2]]
    valid = [value for value in limits if math.isfinite(value) and value > 0]
    if valid:
        return max(valid)
    fallback = _finite(fallback_deg, 'CARLA steering fallback')
    if fallback <= 0:
        raise ValueError('CARLA steering fallback must be positive')
    return fallback


def steering_curve_scale(speed_kmh: float, points: Iterable[Tuple[float, float]]) -> float:
    """Interpolate the PhysX forward-speed (km/h) steering limit multiplier."""
    speed = abs(_finite(speed_kmh, 'Forward speed'))
    # CARLA/UE4 PhysX builds an eight-sample linear lookup and clamps every
    # multiplier to [0, 1] (WheeledVehicleMovementComponentNW.cpp, 0.9.15).
    curve = sorted((_finite(x, 'Curve speed'), clamp(_finite(y, 'Curve scale'), 0.0, 1.0))
                   for x, y in points)[:8]
    if not curve:
        return 1.0
    if speed <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if speed <= x1:
            return y1 if x1 == x0 else y0 + (y1 - y0) * (speed - x0) / (x1 - x0)
    return curve[-1][1]


def road_wheel_to_carla(road_wheel_deg: float, physics_max_deg: float,
                       curve_scale: float = 1.0) -> float:
    """Convert a right-positive wheel angle to right-positive normalized steer."""
    angle = _finite(road_wheel_deg, 'Road-wheel angle')
    maximum = _finite(physics_max_deg, 'CARLA maximum wheel angle')
    scale = _finite(curve_scale, 'Steering curve scale')
    if maximum <= 0 or scale < 0:
        raise ValueError('CARLA angle limit must be positive and curve scale nonnegative')
    if scale == 0:
        return 0.0  # No wheel steering is available at this speed.
    return clamp(angle / (maximum * scale), -1.0, 1.0)


@dataclass(frozen=True)
class DriveControl:
    throttle: float
    brake: float
    motor_enabled: bool
    torque_raw: int


def drive_to_carla(motor_enabled: bool, torque_raw: int, brake_requested: bool,
                   max_torque_raw: int = 3200) -> DriveControl:
    """HEVEN motor-disable means zero throttle; braking only follows its flag."""
    if not 1 <= int(max_torque_raw) <= 3200:
        raise ValueError('Maximum raw torque must be in 1..3200')
    active = bool(motor_enabled) and not bool(brake_requested)
    torque = int(clamp(int(torque_raw), 0, int(max_torque_raw))) if active else 0
    return DriveControl(torque / float(max_torque_raw),
                        1.0 if brake_requested else 0.0, active, torque)
