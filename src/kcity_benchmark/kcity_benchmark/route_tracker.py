#!/usr/bin/env python3
"""
Polyline tracking utilities.

Important:
- route_points are used primarily for progress / completion.
- Distance from route centerline is reported only as a diagnostic.
- It is NOT used in the benchmark score, because a legal out-in-out
  trajectory can be faster and better while intentionally leaving the
  lane centerline.
- planner_reference_points can optionally be scored as controller-tracking
  diagnostics, but they also do not directly affect the whole-stack score.
"""

from __future__ import annotations

import math

from kcity_scenario_manager.route_csv import (
    read_route_csv,
    resolve_route_csv_path,
    route_sha256,
)


def wrap_deg(angle_deg: float) -> float:
    """Wrap angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_route_reference(config_file,benchmark_cfg):
    configured_csv=str(benchmark_cfg.get('route_csv','')).strip()
    if configured_csv:
        path=resolve_route_csv_path(config_file,configured_csv)
        if not path.exists():
            raise RuntimeError(f'benchmark route_csv not found: {path}')
        rows=read_route_csv(path)
        points=[(row['x'],row['y']) for row in rows]
        return points,{
            'route_source':'route_csv',
            'route_csv':configured_csv,
            'route_point_count':len(rows),
            'route_length_m':float(rows[-1]['route_s_m']),
            'route_sha256':route_sha256(path),
        }

    points=benchmark_cfg.get('route_points',[])
    normalized=[
        (float(point[0]),float(point[1]))
        for point in points
        if isinstance(point,(list,tuple)) and len(point)>=2
    ]
    length=sum(
        math.hypot(second[0]-first[0],second[1]-first[1])
        for first,second in zip(normalized[:-1],normalized[1:])
    )
    return normalized,{
        'route_source':'route_points',
        'route_csv':None,
        'route_point_count':len(normalized),
        'route_length_m':length,
        'route_sha256':None,
    }


class PolylineTracker:
    def __init__(self, points):
        self.points = [
            (float(p[0]), float(p[1]))
            for p in points
            if isinstance(p, (list, tuple)) and len(p) >= 2
        ]

        self.segment_lengths = []
        self.cumulative = [0.0]

        for a, b in zip(self.points[:-1], self.points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            self.segment_lengths.append(length)
            self.cumulative.append(self.cumulative[-1] + length)

        self.total_length = self.cumulative[-1] if self.cumulative else 0.0
        self.max_progress_m = 0.0

    @property
    def enabled(self):
        return len(self.points) >= 2 and self.total_length > 1e-6

    def update(self, x: float, y: float):
        """
        Returns
        -------
        progress_pct : float | None
        distance_m   : float | None
            Unsigned 2-D distance to the nearest polyline segment.
        ref_heading_deg : float | None
            Heading of the nearest reference segment.
        """
        if not self.enabled:
            return None, None, None

        best_dist = float("inf")
        best_s = 0.0
        best_heading = None

        for i, (a, b) in enumerate(zip(self.points[:-1], self.points[1:])):
            ax, ay = a
            bx, by = b

            vx = bx - ax
            vy = by - ay
            seg2 = vx * vx + vy * vy
            if seg2 <= 1e-12:
                continue

            wx = x - ax
            wy = y - ay

            u = max(0.0, min(1.0, (wx * vx + wy * vy) / seg2))
            px = ax + u * vx
            py = ay + u * vy

            distance = math.hypot(x - px, y - py)

            if distance < best_dist:
                best_dist = distance
                best_s = self.cumulative[i] + u * self.segment_lengths[i]
                best_heading = math.degrees(math.atan2(vy, vx))

        self.max_progress_m = max(self.max_progress_m, best_s)
        progress_pct = 100.0 * self.max_progress_m / self.total_length

        return progress_pct, best_dist, best_heading
