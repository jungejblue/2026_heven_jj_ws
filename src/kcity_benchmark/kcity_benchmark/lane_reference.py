"""Offline builder and validator for a fixed qualifier lane corridor."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .lane_evaluator import (
    LANE_CHANGE_OPTIONS,
    LaneGeometryEvaluator,
    lane_change_permitted,
    wrap_degrees,
)


REFERENCE_SCHEMA_VERSION = 1


def _xy(point):
    return float(point[0]), float(point[1])


def _cross(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _segments_intersect(first, second, third, fourth, eps=1e-9):
    ax, ay = _xy(first)
    bx, by = _xy(second)
    cx, cy = _xy(third)
    dx, dy = _xy(fourth)
    if (max(ax, bx) + eps < min(cx, dx) or max(cx, dx) + eps < min(ax, bx)
            or max(ay, by) + eps < min(cy, dy)
            or max(cy, dy) + eps < min(ay, by)):
        return False
    ab_c = _cross(ax, ay, bx, by, cx, cy)
    ab_d = _cross(ax, ay, bx, by, dx, dy)
    cd_a = _cross(cx, cy, dx, dy, ax, ay)
    cd_b = _cross(cx, cy, dx, dy, bx, by)
    return ((ab_c > eps and ab_d < -eps or ab_c < -eps and ab_d > eps)
            and (cd_a > eps and cd_b < -eps or cd_a < -eps and cd_b > eps))


def _polygon_area(points):
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def _is_simple_cell(points):
    if len(points) != 4 or abs(_polygon_area(points)) < 1e-7:
        return False
    return not (
        _segments_intersect(points[0], points[1], points[2], points[3])
        or _segments_intersect(points[1], points[2], points[3], points[0])
    )


def _sample_sides(waypoint):
    center = waypoint.transform.location
    right = waypoint.transform.get_right_vector()
    half_width = float(waypoint.lane_width) * 0.5
    rx, ry = float(right.x), float(right.y)
    norm = math.hypot(rx, ry)
    if norm <= 1e-9:
        raise RuntimeError("OpenDRIVE waypoint has a zero-length right vector")
    rx, ry = rx / norm, ry / norm
    cx, cy = float(center.x), float(center.y)
    return {
        "center": [cx, cy],
        "left": [cx - rx * half_width, cy - ry * half_width],
        "right": [cx + rx * half_width, cy + ry * half_width],
        "lane_width_m": float(waypoint.lane_width),
        "yaw_deg": float(waypoint.transform.rotation.yaw),
        "road_id": int(waypoint.road_id),
        "section_id": int(waypoint.section_id),
        "lane_id": int(waypoint.lane_id),
        "junction": bool(waypoint.is_junction),
    }


def _expanded_pairs(first, second, spacing_m):
    distance = math.dist(first[1]["center"], second[1]["center"])
    route_distance = float(second[0]) - float(first[0])
    pieces = max(1, math.ceil(max(distance, route_distance) / spacing_m))
    points = []
    for step in range(pieces + 1):
        fraction = step / pieces
        if step == 0:
            geometry = first[1]
        elif step == pieces:
            geometry = second[1]
        else:
            geometry = {}
            for key in ("center", "left", "right"):
                geometry[key] = [
                    first[1][key][axis] * (1.0 - fraction)
                    + second[1][key][axis] * fraction
                    for axis in range(2)
                ]
            geometry["lane_width_m"] = (
                float(first[1]["lane_width_m"]) * (1.0 - fraction)
                + float(second[1]["lane_width_m"]) * fraction
            )
            geometry["yaw_deg"] = float(first[1]["yaw_deg"]) + fraction * wrap_degrees(
                float(second[1]["yaw_deg"]) - float(first[1]["yaw_deg"])
            )
            geometry["road_id"] = first[1]["road_id"]
            geometry["section_id"] = first[1]["section_id"]
            geometry["lane_id"] = first[1]["lane_id"]
            geometry["junction"] = bool(first[1]["junction"] or second[1]["junction"])
        points.append((
            float(first[0]) + fraction * route_distance,
            geometry,
            int(first[2]),
        ))
    return points


def _cell(first, second, kind, road_option, route_index):
    start_s, first_geometry, _ = first
    end_s, second_geometry, _ = second
    if end_s - start_s <= 1e-8:
        return None
    polygon = [
        first_geometry["left"], second_geometry["left"],
        second_geometry["right"], first_geometry["right"],
    ]
    if not _is_simple_cell(polygon):
        return {
            "route_s_start_m": float(start_s),
            "route_s_end_m": float(end_s),
            "invalid": True,
            "polygon": polygon,
        }
    lane = first_geometry
    return {
        "route_s_start_m": float(start_s),
        "route_s_end_m": float(end_s),
        "polygon": polygon,
        "kind": kind,
        "road_option": road_option,
        "route_index": int(route_index),
        "road_id": int(lane["road_id"]),
        "section_id": int(lane["section_id"]),
        "lane_id": int(lane["lane_id"]),
        "junction": bool(first_geometry["junction"] or second_geometry["junction"]),
        "lane_width_start_m": float(first_geometry["lane_width_m"]),
        "lane_width_end_m": float(second_geometry["lane_width_m"]),
    }


def _line_intersections(groups, minimum_progress_separation_m=2.0):
    """Count nonlocal center/boundary crossings; local adjoining cells are ignored."""
    findings = []
    for group_index, samples in enumerate(groups):
        for side in ("center", "left", "right"):
            segments = []
            for first, second in zip(samples[:-1], samples[1:]):
                if second[0] - first[0] <= 1e-8:
                    continue
                segments.append((first[0], second[0], first[1][side], second[1][side]))
            for i, first in enumerate(segments):
                for second in segments[i + 2:]:
                    if second[0] - first[1] < minimum_progress_separation_m:
                        continue
                    if _segments_intersect(first[2], first[3], second[2], second[3]):
                        findings.append({
                            "group": group_index,
                            "boundary": side,
                            "route_s_ranges_m": [
                                [first[0], first[1]], [second[0], second[1]]
                            ],
                        })
    return findings


def _coverage_gaps(cells, route_end_s):
    intervals = sorted((float(cell["route_s_start_m"]),
                        float(cell["route_s_end_m"]))
                       for cell in cells if not cell.get("invalid"))
    if not intervals:
        return [float(route_end_s)]
    gaps = []
    covered = 0.0
    for start, end in intervals:
        if start > covered + 1e-4:
            gaps.append(start - covered)
        covered = max(covered, end)
    if covered < route_end_s - 1e-4:
        gaps.append(route_end_s - covered)
    return gaps


def build_lane_reference(route_rows, carla_map, carla_module, *, route_sha256,
                         xodr_sha256, map_name, spacing_m=0.25,
                         boundary_tolerance_m=0.0):
    """Freeze XODR-derived curved route cells for map-free runtime evaluation."""
    spacing_m = float(spacing_m)
    if spacing_m <= 0.0 or spacing_m > 0.5:
        raise ValueError("reference spacing must be in (0, 0.5] m")
    route = list(route_rows)
    source = LaneGeometryEvaluator(
        carla_map, carla_module, route,
        boundary_tolerance_m=boundary_tolerance_m,
    )
    expected_changes = [
        index for index, row in enumerate(route)
        if str(row.get("road_option", "LANEFOLLOW")).upper() in LANE_CHANGE_OPTIONS
    ]
    resolved_changes = {item["source_index"] for item in source._lane_change_windows}
    if set(expected_changes) != resolved_changes:
        missing = sorted(set(expected_changes) - resolved_changes)
        raise RuntimeError(
            f"unvalidated qualifier lane-change rows: {missing}; "
            "each must be an adjacent Driving lane with a permitted marking"
        )

    cells = []
    geometry_groups = []
    spacing_values = []
    yaw_jumps = []
    widths = []
    for group_id, samples in sorted(source._samples_by_group.items()):
        geometry_samples = [(
            float(route_s), _sample_sides(waypoint), int(route_index)
        ) for route_s, waypoint, route_index in samples]
        geometry_groups.append(geometry_samples)
        widths.extend(sample[1]["lane_width_m"] for sample in geometry_samples)
        for first, second in zip(geometry_samples[:-1], geometry_samples[1:]):
            if second[0] - first[0] <= 1e-8:
                continue
            expanded = _expanded_pairs(first, second, spacing_m)
            for a, b in zip(expanded[:-1], expanded[1:]):
                cell = _cell(
                    a, b, "route", str(route[min(first[2], len(route) - 1)].get(
                        "road_option", "LANEFOLLOW"
                    )).upper(),
                    min(first[2], len(route) - 1),
                )
                if cell is not None:
                    if cell.get("invalid"):
                        raise RuntimeError(
                            "invalid route corridor cell at route_s="
                            f"{cell['route_s_start_m']:.3f}..{cell['route_s_end_m']:.3f}"
                        )
                    cells.append(cell)
                    spacing_values.append(math.dist(a[1]["center"], b[1]["center"]))
                    yaw_jumps.append(abs(wrap_degrees(
                        b[1]["yaw_deg"] - a[1]["yaw_deg"]
                    )))

    lane_change_summaries = []
    for change_index, change in enumerate(source._lane_change_windows):
        source_index = int(change["source_index"])
        target_index = int(change["target_index"])
        source_row, target_row = route[source_index], route[target_index]
        start_s = float(source_row["route_s_m"])
        end_s = float(target_row["route_s_m"])
        if end_s <= start_s:
            raise RuntimeError(f"lane change at row {source_index} has no route progress")
        source_wp = source._waypoints[source_index]
        target_wp = source._waypoints[target_index]
        option = str(change["option"])
        direction = "LEFT" if option == "CHANGELANELEFT" else "RIGHT"
        marking = getattr(source_wp, f"{direction.lower()}_lane_marking", None)
        if not lane_change_permitted(marking, direction):
            raise RuntimeError(f"lane change row {source_index} is prohibited by marking")
        sign = 1.0 if int(source_row["lane_id"]) < 0 else -1.0
        first_s = float(source_row["opendrive_s"])
        steps = max(1, math.ceil((end_s - start_s) / spacing_m))
        lane_samples = {"source": [], "target": []}
        for step in range(steps + 1):
            fraction = step / steps
            route_s = start_s + fraction * (end_s - start_s)
            for role, lane_id in (("source", int(source_row["lane_id"])),
                                  ("target", int(target_row["lane_id"]))):
                row_station = (first_s if role == "source"
                               else float(target_row["opendrive_s"]))
                route_offset = (route_s - start_s if role == "source"
                                else route_s - end_s)
                station = row_station + sign * route_offset
                waypoint = source._lookup_exact(
                    int(source_row["road_id"]), int(source_row["section_id"]),
                    lane_id, station,
                )
                if waypoint is None or not _is_driving_waypoint(waypoint, carla_module):
                    raise RuntimeError(
                        f"lane-change corridor lost Driving lane {lane_id} "
                        f"at OpenDRIVE s={station:.6f}"
                    )
                lane_samples[role].append((route_s, _sample_sides(waypoint), source_index))
        for role, samples in lane_samples.items():
            geometry_groups.append(samples)
            widths.extend(sample[1]["lane_width_m"] for sample in samples)
            for first, second in zip(samples[:-1], samples[1:]):
                expanded = _expanded_pairs(first, second, spacing_m)
                for a, b in zip(expanded[:-1], expanded[1:]):
                    cell = _cell(a, b, "lane_change", option, source_index)
                    if cell is None:
                        continue
                    if cell.get("invalid"):
                        raise RuntimeError(
                            f"invalid lane-change cell at route_s="
                            f"{cell['route_s_start_m']:.3f}..{cell['route_s_end_m']:.3f}"
                        )
                    cell["lane_change_id"] = change_index
                    cells.append(cell)
                    spacing_values.append(math.dist(a[1]["center"], b[1]["center"]))
                    yaw_jumps.append(abs(wrap_degrees(
                        b[1]["yaw_deg"] - a[1]["yaw_deg"]
                    )))
        lane_change_summaries.append({
            "lane_change_id": change_index,
            "route_index": source_index,
            "route_s_start_m": start_s,
            "route_s_end_m": end_s,
            "road_id": int(source_row["road_id"]),
            "section_id": int(source_row["section_id"]),
            "source_lane_id": int(source_row["lane_id"]),
            "target_lane_id": int(target_row["lane_id"]),
            "road_option": option,
            "lane_marking_permission": str(marking.lane_change),
        })

    invalid_cells = sum(not _is_simple_cell(cell["polygon"]) for cell in cells)
    route_end = float(route[-1]["route_s_m"])
    gaps = _coverage_gaps(cells, route_end)
    route_duplicates = sum(
        math.hypot(float(route[i + 1]["x"]) - float(route[i]["x"]),
                   float(route[i + 1]["y"]) - float(route[i]["y"])) < 0.01
        for i in range(len(route) - 1)
    )
    abrupt_width_changes = []
    for group_index, samples in enumerate(geometry_groups):
        for first, second in zip(samples[:-1], samples[1:]):
            first_lane = tuple(first[1][key]
                               for key in ("road_id", "section_id", "lane_id"))
            second_lane = tuple(second[1][key]
                                for key in ("road_id", "section_id", "lane_id"))
            delta = abs(first[1]["lane_width_m"] - second[1]["lane_width_m"])
            if first_lane == second_lane and delta > 0.5:
                abrupt_width_changes.append({
                    "group": group_index,
                    "road_section_lane": list(first_lane),
                    "route_s_range_m": [first[0], second[0]],
                    "delta_m": delta,
                })
    intersections = _line_intersections(geometry_groups)
    diagnostics = {
        "route_point_count": len(route),
        "route_duplicate_point_count_lt_0_01m": int(route_duplicates),
        "corridor_cell_count": len(cells),
        "lane_change_count": len(lane_change_summaries),
        "sample_spacing_m": {
            "min": min(spacing_values) if spacing_values else 0.0,
            "median": sorted(spacing_values)[len(spacing_values) // 2]
            if spacing_values else 0.0,
            "max": max(spacing_values) if spacing_values else 0.0,
        },
        "lane_width_m": {
            "min": min(widths) if widths else 0.0,
            "max": max(widths) if widths else 0.0,
        },
        "max_sample_heading_jump_deg": max(yaw_jumps) if yaw_jumps else 0.0,
        "invalid_polygon_cell_count": int(invalid_cells),
        "max_route_progress_coverage_gap_m": max(gaps, default=0.0),
        "nonlocal_center_or_boundary_intersection_count": len(intersections),
        "nonlocal_intersection_examples": intersections[:20],
        "abrupt_lane_width_jump_count_over_0_5m": len(abrupt_width_changes),
        "abrupt_lane_width_jumps_over_0_5m": abrupt_width_changes[:20],
    }
    if invalid_cells or gaps:
        raise RuntimeError(
            "lane reference geometry failed sanity checks: "
            f"invalid_cells={invalid_cells}, max_coverage_gap_m={max(gaps, default=0.0):.3f}"
        )
    return {
        "schema_version": REFERENCE_SCHEMA_VERSION,
        "source": {
            "route_file": "routes/qualifier.csv",
            "route_sha256": str(route_sha256),
            "route_point_count": len(route),
            "map_name": str(map_name),
            "xodr_sha256": str(xodr_sha256),
            "corridor_spacing_m": spacing_m,
            "boundary_rule": "OpenDRIVE waypoint lane_width / 2 on each side",
            "wheel_rule": "four VehiclePhysicsControl wheel centers at runtime",
        },
        "lane_change_windows": lane_change_summaries,
        "cells": cells,
        "sanity": diagnostics,
    }


def _is_driving_waypoint(waypoint, carla_module):
    return (waypoint is not None
            and waypoint.lane_type == carla_module.LaneType.Driving)


def validate_lane_reference(reference, route_rows=None, route_sha256=None):
    if int(reference.get("schema_version", -1)) != REFERENCE_SCHEMA_VERSION:
        raise RuntimeError("unsupported lane reference schema version")
    source = reference.get("source", {})
    if route_sha256 and source.get("route_sha256") != route_sha256:
        raise RuntimeError("lane reference was built from a different route CSV")
    if route_rows is not None and len(route_rows) != int(source.get("route_point_count", -1)):
        raise RuntimeError("lane reference point count does not match route CSV")
    cells = reference.get("cells")
    if not isinstance(cells, list) or not cells:
        raise RuntimeError("lane reference has no corridor cells")
    previous_start = -math.inf
    for index, cell in enumerate(sorted(cells, key=lambda item: item["route_s_start_m"])):
        start = float(cell["route_s_start_m"])
        end = float(cell["route_s_end_m"])
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            raise RuntimeError(f"invalid lane reference progress at cell {index}")
        if start < previous_start:
            raise RuntimeError("lane reference cells are not sortable by route progress")
        previous_start = start
        polygon = cell.get("polygon", [])
        if not _is_simple_cell(polygon):
            raise RuntimeError(f"invalid lane reference polygon at cell {index}")
    return reference


def load_lane_reference(path, route_rows=None, route_sha256=None):
    reference = json.loads(Path(path).read_text(encoding="utf-8"))
    return validate_lane_reference(reference, route_rows, route_sha256)


def write_lane_reference(path, reference):
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(reference, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return output


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
