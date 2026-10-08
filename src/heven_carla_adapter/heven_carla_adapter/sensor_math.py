"""ROS-independent units and validation for the noiseless simulation sensors.

RTK flags and nonzero accuracy are simulation metadata, not a model of an
RTCM receiver. Coordinates and measured velocity are never perturbed.
"""

from __future__ import annotations

import math
from typing import Sequence


GPS_WEEK_MS = 604800000
SIM_RTK_FIXED_FLAGS = 0x01 | 0x02 | 0x80


def stamp_nanoseconds(sec: int, nanosec: int) -> int:
    if isinstance(sec, bool) or isinstance(nanosec, bool):
        raise ValueError("ROS stamp must contain integer seconds/nanoseconds")
    if int(sec) != sec or int(nanosec) != nanosec or sec < 0 or not 0 <= nanosec < 1000000000:
        raise ValueError("Invalid ROS stamp")
    stamp = int(sec) * 1000000000 + int(nanosec)
    if stamp <= 0:
        raise ValueError("Sensor stamp must be positive")
    return stamp


def simulation_i_tow(sec: int, nanosec: int) -> int:
    """Convert simulation epoch to ordered millisecond ticks modulo a GPS week.

This is a synthetic epoch, not UTC/GPS time synchronization. A valid positive
ROS stamp can wrap to iTOW=0 at the GPS-week boundary.
"""
    return (stamp_nanoseconds(sec, nanosec) // 1000000) % GPS_WEEK_MS


def _finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _int32(value: float, name: str) -> int:
    result = round(_finite(value, name))
    if not -2147483648 <= result <= 2147483647:
        raise ValueError(f"{name} exceeds NAV-PVT int32 range")
    return result


def yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    """Extract measured ROS body yaw for the scalar-speed status fallback."""
    x, y, z, w = (_finite(value, "quaternion") for value in quaternion)
    norm = x * x + y * y + z * z + w * w
    if not 0.5 <= norm <= 1.5:
        raise ValueError("Quaternion is not a valid orientation")
    scale = 1.0 / math.sqrt(norm)
    x, y, z, w = (value * scale for value in (x, y, z, w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def navpvt_values(sec: int, nanosec: int, latitude: float, longitude: float,
                  altitude: float, velocity_enu: Sequence[float],
                  h_acc_mm: int = 10) -> dict[str, int]:
    """Build the fields HEVEN consumes, using UBX scaled integer units.

Latitude/longitude: 1e-7 deg; height/accuracy: mm; velocity: mm/s;
head_mot: clockwise from north in 1e-5 degrees. Genuine zero speed remains
zero; a target command never creates a measured velocity.
"""
    latitude = _finite(latitude, "latitude")
    longitude = _finite(longitude, "longitude")
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("GNSS coordinates are outside their geographic range")
    if int(h_acc_mm) != h_acc_mm or not 0 < h_acc_mm <= 4294967295:
        raise ValueError("h_acc_mm must be a positive uint32 millimetre value")
    east, north, up = (_finite(value, "velocity_enu") for value in velocity_enu)
    speed = math.hypot(east, north)
    course = math.degrees(math.atan2(east, north)) % 360.0 if speed > 0 else 0.0
    height = _int32(_finite(altitude, "altitude") * 1000.0, "height")
    return {
        "i_tow": simulation_i_tow(sec, nanosec),
        "fix_type": 3,
        "flags": SIM_RTK_FIXED_FLAGS,
        "lat": _int32(latitude * 10000000.0, "lat"),
        "lon": _int32(longitude * 10000000.0, "lon"),
        "height": height,
        # CARLA does not provide geoid undulation; this synthetic field uses
        # the same supplied altitude. HEVEN consumes lat/lon, not h_msl.
        "h_msl": height,
        "h_acc": int(h_acc_mm),
        "v_acc": int(h_acc_mm),
        "vel_e": _int32(east * 1000.0, "vel_e"),
        "vel_n": _int32(north * 1000.0, "vel_n"),
        "vel_d": _int32(-up * 1000.0, "vel_d"),
        "g_speed": _int32(speed * 1000.0, "g_speed"),
        "head_mot": _int32(course * 100000.0, "head_mot"),
    }
