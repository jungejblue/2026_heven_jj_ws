"""Resolve HEVEN absolute geographic routes against the actual CARLA map.

There is deliberately no fixed K-City geographic centre here. The caller
supplies the running map's Cartesian-to-geographic conversion.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Callable, Sequence

from .geodesy import project_latlon


Geolocate = Callable[[float, float, float], Sequence[float]]


def load_latlon_csv(path: str | Path) -> list[tuple[float, float]]:
    """Read HEVEN's latitude,longitude CSV, with an optional named header."""
    points = []
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            if not row or not any(value.strip() for value in row) or row[0].lstrip().startswith("#"):
                continue
            if (not points and len(row) >= 2 and row[0].strip().lower() in ("lat", "latitude")
                    and row[1].strip().lower() in ("lon", "longitude", "lng")):
                continue
            try:
                latitude, longitude = float(row[0]), float(row[1])
                project_latlon(latitude, longitude, latitude, longitude)
            except (IndexError, ValueError, OverflowError) as exc:
                raise ValueError(f"Invalid latitude,longitude at CSV line {line_number}") from exc
            points.append((latitude, longitude))
    if len(points) < 2:
        raise ValueError("Route CSV needs at least two latitude,longitude points")
    return points


def route_entry(points: Sequence[Sequence[float]], lookahead_m: float = 5.0):
    """Return the first geographic point and a point along the initial route.

Interpolate along polyline distance rather than counting CSV rows, so the
heading reference is independent of the recording's sample density.
"""
    lookahead = float(lookahead_m)
    if not math.isfinite(lookahead) or lookahead <= 0:
        raise ValueError("lookahead_m must be positive")
    if len(points) < 2:
        raise ValueError("Route needs at least two points")
    coordinates = []
    for point in points:
        latitude, longitude = (float(value) for value in point)
        project_latlon(latitude, longitude, latitude, longitude)
        coordinates.append((latitude, longitude))
    first = coordinates[0]
    remaining = lookahead
    selected = first
    for previous, current in zip(coordinates, coordinates[1:]):
        east, north = project_latlon(*current, *previous)
        distance = math.hypot(east, north)
        if distance == 0:
            continue
        if remaining <= distance:
            fraction = remaining / distance
            selected = tuple(a + fraction * (b - a) for a, b in zip(previous, current))
            break
        remaining -= distance
        selected = current
    east, north = project_latlon(*selected, *first)
    if math.hypot(east, north) <= 1e-6:
        raise ValueError("Route has no usable initial heading direction")
    return first, selected


def invert_latlon(latitude: float, longitude: float, geolocate: Geolocate,
                   z: float = 4.0, initial_xy=(0.0, 0.0), iterations: int = 8,
                   tolerance_m: float = 0.001) -> tuple[float, float]:
    """Find native CARLA x,y for absolute latitude/longitude by Newton steps.

Residuals are local metres at the target latitude. A finite one-metre basis
sample forms the 2×2 Jacobian; rotated and shifted georeferences are supported.
"""
    latitude, longitude, z = float(latitude), float(longitude), float(z)
    project_latlon(latitude, longitude, latitude, longitude)
    x, y = (float(value) for value in initial_xy)
    if not all(math.isfinite(value) for value in (x, y, z, tolerance_m)) or tolerance_m <= 0:
        raise ValueError("Solver coordinates and positive tolerance must be finite")
    if isinstance(iterations, bool) or int(iterations) != iterations or iterations <= 0:
        raise ValueError("iterations must be a positive integer")

    def residual(sample_x, sample_y):
        geo = geolocate(sample_x, sample_y, z)
        return project_latlon(geo[0], geo[1], latitude, longitude)

    for _ in range(int(iterations)):
        east, north = residual(x, y)
        if math.hypot(east, north) <= tolerance_m:
            return x, y
        sample_x, sample_y = residual(x + 1.0, y), residual(x, y + 1.0)
        a, c = sample_x[0] - east, sample_x[1] - north
        b, d = sample_y[0] - east, sample_y[1] - north
        determinant = a * d - b * c
        if not math.isfinite(determinant) or abs(determinant) <= 1e-12:
            raise ValueError("CARLA map has a singular geographic XY Jacobian")
        x -= (d * east - b * north) / determinant
        y -= (-c * east + a * north) / determinant
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Geographic inversion produced nonfinite coordinates")
    if math.hypot(*residual(x, y)) <= tolerance_m:
        return x, y
    raise ValueError(f"Geographic inversion did not converge within {iterations} iterations")


def build_spawn_config(points: Sequence[Sequence[float]], geolocate: Geolocate,
                       z: float = 4.0, lookahead_m: float = 5.0) -> dict:
    """Return carla_spawn_objects JSON in ROS axes, for the HEVEN ego only."""
    first, ahead = route_entry(points, lookahead_m)
    x, y = invert_latlon(*first, geolocate, z)
    ahead_x, ahead_y = invert_latlon(*ahead, geolocate, z, initial_xy=(x, y))
    dx, dy = ahead_x - x, ahead_y - y
    if math.hypot(dx, dy) <= 1e-6:
        raise ValueError("Route heading points collapse in the CARLA map")
    native_yaw_deg = math.degrees(math.atan2(dy, dx))
    return {"objects": [{"type": "vehicle.heven.ev", "id": "ego_vehicle",
        "spawn_point": {"x": x, "y": -y, "z": float(z),
                        "roll": 0.0, "pitch": 0.0, "yaw": -native_yaw_deg},
        "sensors": []}]}
