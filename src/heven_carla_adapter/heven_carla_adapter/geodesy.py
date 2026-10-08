"""Geographic projection used by HEVEN localization and planning.

This is the same local equirectangular approximation as the JJ nodes: east
uses the cosine of the fixed origin latitude, and north uses latitude change.
It is deliberately not a WGS84 ECEF/ENU conversion. CARLA-to-geographic
sampling is supplied by the caller, so this module needs neither ROS nor
CARLA and makes no assumption about the CARLA map's axis orientation.
"""

from __future__ import annotations

import math
from typing import Callable, Sequence


ORIGIN_LATITUDE_DEG = 37.2388873
ORIGIN_LONGITUDE_DEG = 126.7729325
EARTH_RADIUS_M = 6378135.0

Vector3 = tuple[float, float, float]
Geolocate = Callable[[float, float, float], Sequence[float]]


def _finite(value: float, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _latitude(value: float, name: str) -> float:
    latitude = _finite(value, name)
    if not -90.0 <= latitude <= 90.0:
        raise ValueError(f"{name} must be between -90 and 90 degrees")
    return latitude


def _longitude(value: float, name: str) -> float:
    longitude = _finite(value, name)
    if not -180.0 <= longitude <= 180.0:
        raise ValueError(f"{name} must be between -180 and 180 degrees")
    return longitude


def _positive(value: float, name: str) -> float:
    number = _finite(value, name)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive")
    return number


def _vector3(values: Sequence[float], name: str) -> Vector3:
    try:
        x, y, z = values
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain exactly three numbers") from exc
    return (_finite(x, f"{name}[0]"), _finite(y, f"{name}[1]"),
            _finite(z, f"{name}[2]"))


def _geographic(values: Sequence[float], name: str) -> Vector3:
    latitude, longitude, altitude = _vector3(values, name)
    return (_latitude(latitude, f"{name}.latitude"),
            _longitude(longitude, f"{name}.longitude"), altitude)


def _projection_scale(origin_latitude: float, earth_radius_m: float) -> tuple[float, float]:
    latitude = _latitude(origin_latitude, "origin_latitude")
    scale = math.radians(1.0) * _positive(earth_radius_m, "earth_radius_m")
    return scale * math.cos(math.radians(latitude)), scale


def project_latlon(
    latitude: float,
    longitude: float,
    origin_latitude: float = ORIGIN_LATITUDE_DEG,
    origin_longitude: float = ORIGIN_LONGITUDE_DEG,
    earth_radius_m: float = EARTH_RADIUS_M,
) -> tuple[float, float]:
    """Return JJ map (east, north) in metres from absolute latitude/longitude.

    Angles are degrees. Longitude differences remain unwrapped, matching the
    JJ implementation; this local projection is intended for one map region.
    """
    latitude = _latitude(latitude, "latitude")
    longitude = _longitude(longitude, "longitude")
    origin_latitude = _latitude(origin_latitude, "origin_latitude")
    origin_longitude = _longitude(origin_longitude, "origin_longitude")
    east_scale, north_scale = _projection_scale(origin_latitude, earth_radius_m)
    return (_finite((longitude - origin_longitude) * east_scale, "east"),
            _finite((latitude - origin_latitude) * north_scale, "north"))


def enu_displacement(
    geo_a: Sequence[float],
    geo_b: Sequence[float],
    origin_latitude: float = ORIGIN_LATITUDE_DEG,
    earth_radius_m: float = EARTH_RADIUS_M,
) -> Vector3:
    """Return geographic displacement from a to b as JJ east/north/up metres.

    Geographic tuples contain (latitude degrees, longitude degrees, altitude
    metres). The altitude difference supplies up without imposing an origin.
    """
    lat_a, lon_a, alt_a = _geographic(geo_a, "geo_a")
    lat_b, lon_b, alt_b = _geographic(geo_b, "geo_b")
    east_scale, north_scale = _projection_scale(origin_latitude, earth_radius_m)
    return (_finite((lon_b - lon_a) * east_scale, "east"),
            _finite((lat_b - lat_a) * north_scale, "north"),
            _finite(alt_b - alt_a, "up"))


def _sample_parameters(
    location_xyz: Sequence[float],
    geolocate: Geolocate,
    origin_latitude: float,
    earth_radius_m: float,
    sample_distance_m: float,
) -> tuple[Vector3, float]:
    location = _vector3(location_xyz, "location_xyz")
    if not callable(geolocate):
        raise ValueError("geolocate must be callable with x, y, z")
    _projection_scale(origin_latitude, earth_radius_m)
    return location, _positive(sample_distance_m, "sample_distance_m")


def finite_direction_yaw(
    location_xyz: Sequence[float],
    forward_xyz: Sequence[float],
    geolocate: Geolocate,
    origin_latitude: float = ORIGIN_LATITUDE_DEG,
    earth_radius_m: float = EARTH_RADIUS_M,
    sample_distance_m: float = 1.0,
) -> float:
    """Return forward direction as ENU yaw radians, positive towards north.

    Geolocate the current point and a point one sample distance along the
    normalized Cartesian forward vector. Yaw is atan2(north, east), allowing
    rotated/georeferenced maps without assuming a fixed world-y sign.
    """
    location, distance = _sample_parameters(
        location_xyz, geolocate, origin_latitude, earth_radius_m, sample_distance_m)
    forward = _vector3(forward_xyz, "forward_xyz")
    magnitude = math.hypot(*forward)
    if not math.isfinite(magnitude) or magnitude == 0.0:
        raise ValueError("forward_xyz must have a finite nonzero magnitude")
    sample = _vector3(tuple(position + distance * (axis / magnitude)
                            for position, axis in zip(location, forward)), "sample_xyz")
    displacement = enu_displacement(geolocate(*location), geolocate(*sample),
                                   origin_latitude, earth_radius_m)
    if math.hypot(displacement[0], displacement[1]) <= 1e-9:
        raise ValueError("forward sample has no measurable horizontal geographic displacement")
    return math.atan2(displacement[1], displacement[0])


def finite_velocity_enu(
    location_xyz: Sequence[float],
    velocity_xyz: Sequence[float],
    geolocate: Geolocate,
    origin_latitude: float = ORIGIN_LATITUDE_DEG,
    earth_radius_m: float = EARTH_RADIUS_M,
    sample_distance_m: float = 1.0,
) -> Vector3:
    """Map Cartesian m/s velocity to JJ east/north/up using a local Jacobian.

    Each Cartesian basis axis is sampled independently at the current point;
    the resulting displacement columns divided by sample distance transform
    the velocity vector. A genuine zero velocity returns exactly zero and
    does not infer motion from geographic samples or a heading seed.
    """
    location, distance = _sample_parameters(
        location_xyz, geolocate, origin_latitude, earth_radius_m, sample_distance_m)
    velocity = _vector3(velocity_xyz, "velocity_xyz")
    if all(component == 0.0 for component in velocity):
        return (0.0, 0.0, 0.0)
    current_geo = _geographic(geolocate(*location), "current_geo")
    columns = []
    for axis in range(3):
        sample = list(location)
        sample[axis] += distance
        sample_xyz = _vector3(sample, "sample_xyz")
        delta = enu_displacement(current_geo, geolocate(*sample_xyz),
                                 origin_latitude, earth_radius_m)
        columns.append(tuple(_finite(component / distance, "jacobian")
                             for component in delta))
    return tuple(_finite(sum(columns[axis][row] * velocity[axis] for axis in range(3)),
                         f"velocity_enu[{row}]") for row in range(3))
