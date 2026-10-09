"""Route-aware wheel corridor evaluation and qualifier lane penalties.

The geometry evaluator reads the qualifier route as a reference. It never
advances the simulator or changes vehicle state.
"""

from __future__ import annotations

from dataclasses import dataclass
from bisect import bisect_left, bisect_right
import math


WHEEL_NAMES = ("FL", "FR", "RL", "RR")
LANE_CHANGE_OPTIONS = {"CHANGELANELEFT", "CHANGELANERIGHT"}
XODR_S_EPSILONS = (0.0, 1e-5, -1e-5, 1e-4, -1e-4, 1e-3, -1e-3)


@dataclass(frozen=True)
class RouteMatch:
    index: int
    route_s_m: float
    distance_m: float


@dataclass(frozen=True)
class LaneEvaluation:
    wheel_out_count: int
    outside_wheels: tuple[str, ...]
    allowed_lane_ids: tuple[tuple[int, int, int], ...]
    matched_route_index: int
    matched_road_id: int
    matched_section_id: int
    matched_lane_id: int
    junction: bool
    road_option: str
    route_match_distance_m: float
    wheel_position_frame: str
    wheelbase_m: float
    front_track_m: float
    rear_track_m: float
    wheel_positions: dict | None = None

    @property
    def violation(self):
        return self.wheel_out_count > 0

    @property
    def full_lane_exit(self):
        return self.wheel_out_count == 4

    def detail(self, continuous_violation_sec=0.0):
        lanes = ";".join(f"{road}/{section}/{lane}"
                         for road, section, lane in self.allowed_lane_ids)
        outside = ",".join(self.outside_wheels) or "none"
        return (
            f"wheel_out_count={self.wheel_out_count}, outside_wheels={outside}, "
            f"allowed_lane_ids={lanes}, matched_route_index={self.matched_route_index}, "
            f"matched_road_id={self.matched_road_id}, "
            f"matched_section_id={self.matched_section_id}, "
            f"matched_lane_id={self.matched_lane_id}, "
            f"junction={int(self.junction)}, RoadOption={self.road_option}, "
            f"continuous_violation_sec={continuous_violation_sec:.3f}, "
            f"route_match_distance_m={self.route_match_distance_m:.3f}, "
            f"wheel_position_frame={self.wheel_position_frame}, "
            f"wheelbase_m={self.wheelbase_m:.3f}, "
            f"front_track_m={self.front_track_m:.3f}, "
            f"rear_track_m={self.rear_track_m:.3f}"
        )


@dataclass(frozen=True)
class LanePenaltyEvent:
    event: str
    penalty_sec: float
    detail: str


class WheelPositionProvider:
    """Read the ego's four configured wheel positions without vehicle-size guesses."""

    def __init__(self, carla_module):
        self.carla = carla_module
        self._wheel_offsets = None
        self._wheel_position_frame = "world_cm_to_vehicle_local_m"
        self._wheelbase_m = 0.0
        self._front_track_m = 0.0
        self._rear_track_m = 0.0

    def _capture_wheel_offsets(self, vehicle, transform):
        if self._wheel_offsets is not None:
            return
        physics = vehicle.get_physics_control()
        wheels = list(physics.wheels)
        if len(wheels) != 4:
            raise RuntimeError(
                f"qualifier ego must expose 4 physics wheels; got {len(wheels)}"
            )
        # Unreal UVehicleWheel::Location is a world-space FVector in cm.
        # CARLA 0.9.15 copies it into WheelPhysicsControl.position without
        # converting units. Convert to CARLA API meters before deriving the
        # vehicle-local offsets cached for subsequent actor transforms.
        self._wheel_offsets = tuple(
            _inverse_transform_point(
                transform,
                self.carla.Location(
                    x=float(wheel.position.x) / 100.0,
                    y=float(wheel.position.y) / 100.0,
                    z=float(wheel.position.z) / 100.0,
                ),
            )
            for wheel in wheels
        )

        # CARLA 0.9.15 returns the four-wheel array in FL, FR, RL, RR order.
        fl, fr, rl, rr = self._wheel_offsets
        self._wheelbase_m = abs((fl[0] + fr[0] - rl[0] - rr[0]) * 0.5)
        self._front_track_m = abs(fl[1] - fr[1])
        self._rear_track_m = abs(rl[1] - rr[1])

    def _wheel_world_positions(self, vehicle, transform):
        self._capture_wheel_offsets(vehicle, transform)
        points = []
        for offset in self._wheel_offsets:
            local = self.carla.Location(x=offset[0], y=offset[1], z=offset[2])
            world = transform.transform(local)
            points.append({"x": float(world.x), "y": float(world.y),
                           "z": float(world.z)})
        return dict(zip(WHEEL_NAMES, points))


class RouteMatcher:
    """Match to a bounded, forward-continuous segment of the route CSV."""

    def __init__(self, route_rows, backward_window=2, forward_window=40,
                 max_forward_jump=15):
        self.rows = list(route_rows)
        if len(self.rows) < 2:
            raise ValueError("lane evaluation requires at least two route rows")
        self.backward_window = int(backward_window)
        self.forward_window = int(forward_window)
        self.max_forward_jump = int(max_forward_jump)
        self.previous_index = 0

    def match(self, x, y, yaw_deg):
        previous = max(0, min(self.previous_index, len(self.rows) - 1))
        first = max(0, previous - self.backward_window)
        last_segment = min(
            len(self.rows) - 2,
            previous + min(self.forward_window, self.max_forward_jump),
        )
        candidates = []
        for index in range(first, last_segment + 1):
            a, b = self.rows[index], self.rows[index + 1]
            dx, dy = float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])
            length2 = dx * dx + dy * dy
            if length2 <= 1e-12:
                continue
            u = max(0.0, min(1.0, ((x - float(a["x"])) * dx
                                  + (y - float(a["y"])) * dy) / length2))
            px, py = float(a["x"]) + u * dx, float(a["y"]) + u * dy
            distance = math.hypot(x - px, y - py)
            route_yaw = float(a.get("yaw", math.degrees(math.atan2(dy, dx))))
            yaw_error = abs(wrap_degrees(yaw_deg - route_yaw))
            # Heading breaks ties at overlapping junction geometry. Distance
            # remains dominant, and route order is bounded independently.
            score = distance * distance + 0.75 * (yaw_error / 180.0) ** 2
            candidate_index = index if u < 0.5 else index + 1
            if candidate_index < previous - self.backward_window:
                continue
            if candidate_index > previous + self.max_forward_jump:
                continue
            route_s = float(a["route_s_m"]) + u * (
                float(b["route_s_m"]) - float(a["route_s_m"])
            )
            candidates.append((score, distance, candidate_index, route_s))

        if not candidates:
            row = self.rows[previous]
            return RouteMatch(previous, float(row["route_s_m"]),
                              math.hypot(x - float(row["x"]), y - float(row["y"])))
        _, distance, index, route_s = min(candidates)
        index = max(previous, index)
        self.previous_index = index
        return RouteMatch(index, route_s, distance)

    def project_nearby(self, x, y, yaw_deg, center_route_s, progress_window_m):
        """Project a wheel point locally without changing ego route progress."""
        low = float(center_route_s) - float(progress_window_m)
        high = float(center_route_s) + float(progress_window_m)
        candidates = []
        for index in range(len(self.rows) - 1):
            a, b = self.rows[index], self.rows[index + 1]
            first_s, second_s = float(a["route_s_m"]), float(b["route_s_m"])
            if max(first_s, second_s) < low or min(first_s, second_s) > high:
                continue
            dx, dy = float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])
            length2 = dx * dx + dy * dy
            if length2 <= 1e-12:
                continue
            fraction = max(0.0, min(1.0, ((x - float(a["x"])) * dx
                                          + (y - float(a["y"])) * dy) / length2))
            px, py = float(a["x"]) + fraction * dx, float(a["y"]) + fraction * dy
            distance = math.hypot(x - px, y - py)
            route_yaw = float(a.get("yaw", math.degrees(math.atan2(dy, dx))))
            yaw_error = abs(wrap_degrees(yaw_deg - route_yaw))
            score = distance * distance + 0.75 * (yaw_error / 180.0) ** 2
            route_s = first_s + fraction * (second_s - first_s)
            candidates.append((score, distance, index, route_s))
        if not candidates:
            row_index = min(range(len(self.rows)),
                            key=lambda i: abs(float(self.rows[i]["route_s_m"])
                                              - float(center_route_s)))
            row = self.rows[row_index]
            return RouteMatch(
                row_index, float(row["route_s_m"]),
                math.hypot(x - float(row["x"]), y - float(row["y"])),
            )
        _, distance, index, route_s = min(candidates)
        return RouteMatch(index, route_s, distance)


def wrap_degrees(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def _enum_name(value):
    return str(getattr(value, "name", value)).rsplit(".", 1)[-1].upper()


def lane_change_permitted(marking, direction):
    if marking is None:
        return False
    permission = _enum_name(getattr(marking, "lane_change", "NONE"))
    direction = str(direction).upper()
    return permission in (direction, "BOTH")


def _lane_key(waypoint):
    return (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))


def _is_driving(waypoint, carla):
    return waypoint is not None and waypoint.lane_type == carla.LaneType.Driving


def _point_xy(point):
    if isinstance(point, dict):
        return float(point["x"]), float(point["y"])
    if isinstance(point, (tuple, list)) and len(point) >= 2:
        return float(point[0]), float(point[1])
    return float(point.x), float(point.y)


def _xyz(point):
    return float(point.x), float(point.y), float(point.z)


def _inverse_transform_point(transform, point):
    origin = transform.location
    delta = (float(point.x) - float(origin.x),
             float(point.y) - float(origin.y),
             float(point.z) - float(origin.z))
    basis = (transform.get_forward_vector(), transform.get_right_vector(),
             transform.get_up_vector())
    return tuple(sum(delta[axis] * float(getattr(vector, name))
                     for axis, name in enumerate(("x", "y", "z")))
                 for vector in basis)


def _distance_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1e-15:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + u * dx), py - (ay + u * dy))


def point_in_polygon(point, polygon, tolerance_m=0.01):
    """Boundary-inclusive 2D polygon test with a small numeric tolerance."""
    px, py = _point_xy(point)
    vertices = [(_point_xy(item)) for item in polygon]
    if len(vertices) < 3:
        return False
    inside = False
    for (ax, ay), (bx, by) in zip(vertices, vertices[1:] + vertices[:1]):
        if _distance_to_segment(px, py, ax, ay, bx, by) <= tolerance_m:
            return True
        if (ay > py) != (by > py):
            crossing_x = (bx - ax) * (py - ay) / (by - ay) + ax
            if px < crossing_x:
                inside = not inside
    return inside


def classify_wheels(wheel_positions, corridor_polygons, tolerance_m=0.01):
    """Return the names of wheel centers outside all allowed corridors."""
    if isinstance(wheel_positions, dict):
        ordered = [(name, wheel_positions[name]) for name in WHEEL_NAMES]
    else:
        ordered = list(zip(WHEEL_NAMES, wheel_positions))
    if len(ordered) != 4:
        raise ValueError(f"expected exactly four wheel positions, got {len(ordered)}")
    polygons = list(corridor_polygons)
    return tuple(
        name for name, point in ordered
        if not any(point_in_polygon(point, polygon, tolerance_m) for polygon in polygons)
    )


class LaneGeometryEvaluator(WheelPositionProvider):
    """Evaluate wheel centers against route-derived curved driving corridors."""

    def __init__(self, carla_map, carla_module, route_rows,
                 boundary_tolerance_m=0.01, corridor_behind_m=8.0,
                 corridor_ahead_m=12.0, waypoint_tolerance_m=1.0):
        self.map = carla_map
        super().__init__(carla_module)
        self.rows = list(route_rows)
        self.tolerance_m = float(boundary_tolerance_m)
        self.behind_m = float(corridor_behind_m)
        self.ahead_m = float(corridor_ahead_m)
        self.waypoint_tolerance_m = float(waypoint_tolerance_m)
        if not math.isfinite(self.waypoint_tolerance_m) or self.waypoint_tolerance_m <= 0:
            raise ValueError("waypoint_tolerance_m must be finite and positive")
        if self.tolerance_m < 0.0 or self.tolerance_m > 0.05:
            raise ValueError("boundary_tolerance_m must be in [0, 0.05] m")
        self.matcher = RouteMatcher(self.rows)
        self._waypoints = [self._route_waypoint(row) for row in self.rows]
        self._lane_change_windows = []
        self._split_edges = set()
        self._build_lane_change_windows()
        self._row_groups = self._assign_groups()
        self._samples_by_group = self._build_route_samples()

    def _route_waypoint(self, row):
        road_id = int(row["road_id"])
        section_id = int(row["section_id"])
        lane_id = int(row["lane_id"])
        requested_s = float(row["opendrive_s"])
        for epsilon in XODR_S_EPSILONS:
            waypoint = self.map.get_waypoint_xodr(
                road_id, lane_id, requested_s + epsilon
            )
            if waypoint is None:
                continue
            if _lane_key(waypoint) != (road_id, section_id, lane_id):
                continue
            if not _is_driving(waypoint, self.carla):
                continue
            location = waypoint.transform.location
            distance = math.hypot(float(location.x) - float(row["x"]),
                                  float(location.y) - float(row["y"]))
            if distance > self.waypoint_tolerance_m:
                raise RuntimeError(
                    f"qualifier route index={row['index']}: reconstructed waypoint "
                    f"is {distance:.3f}m from CSV x/y; "
                    f"waypoint_tolerance_m={self.waypoint_tolerance_m:.3f}"
                )
            return waypoint
        raise RuntimeError(
            "qualifier route cannot be resolved exactly on current map at "
            f"index={row['index']} road/section/lane/s="
            f"{road_id}/{section_id}/{lane_id}/{requested_s:.6f}"
        )

    def _build_lane_change_windows(self):
        for option_index, row in enumerate(self.rows):
            option = str(row.get("road_option", "LANEFOLLOW")).upper()
            if option not in LANE_CHANGE_OPTIONS:
                continue
            # csv_route_agent passes each row's RoadOption to the segment from
            # that row to the following row (including the first route row).
            source_index = option_index
            target_index = min(option_index + 1, len(self.rows) - 1)
            if source_index == target_index:
                continue
            source, target = self._waypoints[source_index], self._waypoints[target_index]
            if not (_is_driving(source, self.carla) and _is_driving(target, self.carla)):
                continue
            direction = "LEFT" if option == "CHANGELANELEFT" else "RIGHT"
            marking = getattr(source, f"{direction.lower()}_lane_marking", None)
            if not lane_change_permitted(marking, direction):
                continue
            adjacent_method = getattr(source, f"get_{direction.lower()}_lane", None)
            adjacent = adjacent_method() if adjacent_method else None
            if not _is_driving(adjacent, self.carla) or _lane_key(adjacent) != _lane_key(target):
                continue
            if source.road_id != target.road_id or source.section_id != target.section_id:
                continue
            if abs(int(source.lane_id) - int(target.lane_id)) != 1:
                continue
            self._split_edges.add(source_index)
            self._lane_change_windows.append({
                "source_index": source_index,
                "target_index": target_index,
                "source_group": None,
                "target_group": None,
                "option": option,
                "start_s": min(float(self.rows[source_index]["route_s_m"]),
                                float(self.rows[target_index]["route_s_m"])) - 2.0,
                "end_s": max(float(self.rows[source_index]["route_s_m"]),
                              float(self.rows[target_index]["route_s_m"])) + 2.0,
            })

    def _assign_groups(self):
        groups = [0] * len(self.rows)
        group = 0
        for index in range(len(self.rows) - 1):
            groups[index] = group
            if index in self._split_edges:
                group += 1
        groups[-1] = group
        for change in self._lane_change_windows:
            change["source_group"] = groups[change["source_index"]]
            change["target_group"] = groups[change["target_index"]]
        return groups

    def _lookup_exact(self, road_id, section_id, lane_id, s):
        for epsilon in XODR_S_EPSILONS:
            waypoint = self.map.get_waypoint_xodr(road_id, lane_id, s + epsilon)
            if waypoint is not None and _lane_key(waypoint) == (
                    road_id, section_id, lane_id):
                return waypoint
        return None

    def _build_route_samples(self):
        samples = {group: [] for group in set(self._row_groups)}
        for index, row in enumerate(self.rows):
            group = self._row_groups[index]
            samples[group].append((float(row["route_s_m"]), self._waypoints[index], index))
            if index + 1 >= len(self.rows) or index in self._split_edges:
                continue
            following = self.rows[index + 1]
            same_lane = all(int(row[key]) == int(following[key])
                            for key in ("road_id", "section_id", "lane_id"))
            if not same_lane:
                continue
            first_s = float(row["opendrive_s"])
            last_s = float(following["opendrive_s"])
            steps = max(1, math.ceil(abs(last_s - first_s) / 0.25))
            for step in range(1, steps):
                fraction = step / steps
                opendrive_s = first_s + fraction * (last_s - first_s)
                waypoint = self._lookup_exact(
                    int(row["road_id"]), int(row["section_id"]),
                    int(row["lane_id"]), opendrive_s,
                )
                if waypoint is None:
                    continue
                route_s = float(row["route_s_m"]) + fraction * (
                    float(following["route_s_m"]) - float(row["route_s_m"])
                )
                samples[group].append((route_s, waypoint, index))
        # The finish line is crossed by the actor center while its front wheels
        # are already beyond the final route waypoint. Extend the terminal cap
        # one meter along the same OpenDRIVE lane; lateral lane boundaries
        # and all penalty rules remain unchanged.
        last_waypoint = self._waypoints[-1]
        terminal_lane = _lane_key(last_waypoint)
        continuation = [waypoint for waypoint in last_waypoint.next(1.0)
                        if _lane_key(waypoint) == terminal_lane
                        and _is_driving(waypoint, self.carla)]
        if continuation:
            continuation.sort(key=lambda waypoint: abs(wrap_degrees(
                float(waypoint.transform.rotation.yaw)
                - float(last_waypoint.transform.rotation.yaw)
            )))
            last_row = self.rows[-1]
            samples[self._row_groups[-1]].append((
                float(last_row["route_s_m"]) + 1.0,
                continuation[0], int(last_row["index"]),
            ))
        for group_samples in samples.values():
            group_samples.sort(key=lambda item: item[0])
        return samples

    @staticmethod
    def _sample_boundary(waypoint):
        center = waypoint.transform.location
        right = waypoint.transform.get_right_vector()
        return (float(center.x), float(center.y), float(right.x), float(right.y),
                float(waypoint.lane_width) * 0.5, _lane_key(waypoint))

    def _extend_sparse_group(self, group, route_s, selected):
        if len(selected) >= 3:
            return selected
        all_samples = self._samples_by_group[group]
        if len(all_samples) >= 3:
            nearest = min(range(len(all_samples)),
                          key=lambda i: abs(all_samples[i][0] - route_s))
            first, last = max(0, nearest - 2), min(len(all_samples), nearest + 3)
            return all_samples[first:last]
        seed = min(all_samples, key=lambda item: abs(item[0] - route_s))[1]
        extra = []
        for method_name, sign in (("previous", -1), ("next", 1)):
            cursor = seed
            for step in range(1, 9):
                method = getattr(cursor, method_name, None)
                candidates = method(1.0) if method else []
                if not candidates:
                    break
                expected_yaw = float(cursor.transform.rotation.yaw)
                cursor = min(candidates, key=lambda item: abs(wrap_degrees(
                    float(item.transform.rotation.yaw) - expected_yaw
                )))
                extra.append((route_s + sign * step, cursor, -1))
        return sorted(list(selected) + extra, key=lambda item: item[0])

    def _corridor_polygon(self, group, route_s):
        first_s, last_s = route_s - self.behind_m, route_s + self.ahead_m
        all_samples = self._samples_by_group[group]
        selected = [sample for sample in all_samples
                    if first_s <= sample[0] <= last_s]
        selected = self._extend_sparse_group(group, route_s, selected)
        return self._polygon_from_waypoints(
            [waypoint for _, waypoint, _ in selected]
        )

    def _polygon_from_waypoints(self, waypoints):
        boundaries = []
        seen = set()
        for waypoint in waypoints:
            key = (round(float(waypoint.transform.location.x), 4),
                   round(float(waypoint.transform.location.y), 4),
                   _lane_key(waypoint))
            if key in seen:
                continue
            seen.add(key)
            boundaries.append(self._sample_boundary(waypoint))
        if len(boundaries) < 2:
            return [], set()
        left, right = [], []
        lane_ids = set()
        for x, y, rx, ry, half_width, lane_key in boundaries:
            norm = math.hypot(rx, ry)
            if norm <= 1e-9:
                continue
            rx, ry = rx / norm, ry / norm
            left.append((x - rx * half_width, y - ry * half_width))
            right.append((x + rx * half_width, y + ry * half_width))
            lane_ids.add(lane_key)
        return left + list(reversed(right)), lane_ids

    def _lane_change_polygons(self, change, route_s):
        source_index = change["source_index"]
        target_index = change["target_index"]
        source_row = self.rows[source_index]
        target_row = self.rows[target_index]
        road_id = int(source_row["road_id"])
        section_id = int(source_row["section_id"])
        source_lane = int(source_row["lane_id"])
        target_lane = int(target_row["lane_id"])
        direction = 1.0 if source_lane < 0 else -1.0
        source_s = float(source_row["opendrive_s"]) + direction * (
            route_s - float(source_row["route_s_m"])
        )
        source_points, target_points = [], []
        # Sample both adjacent lanes at the same OpenDRIVE s values. CSV
        # transition rows can be several metres apart longitudinally, so their
        # two row centers must not be treated as simultaneous lane centers.
        for progress_m in [(self.behind_m + self.ahead_m) * i / 16.0 - self.behind_m
                           for i in range(17)]:
            sample_s = source_s + direction * progress_m
            source_wp = self._lookup_exact(road_id, section_id, source_lane, sample_s)
            target_wp = self._lookup_exact(road_id, section_id, target_lane, sample_s)
            if _is_driving(source_wp, self.carla):
                source_points.append(source_wp)
            if _is_driving(target_wp, self.carla):
                target_points.append(target_wp)
        polygons = []
        lane_ids = set()
        for points in (source_points, target_points):
            polygon, lanes = self._polygon_from_waypoints(points)
            if polygon:
                polygons.append(polygon)
                lane_ids.update(lanes)
        return polygons, lane_ids

    def corridor_polygons_for_match(self, route_index, route_s_m):
        """Return route-allowed curved polygons and their lane identifiers."""
        route_index = max(0, min(int(route_index), len(self.rows) - 1))
        allowed_groups = {self._row_groups[route_index]}
        active_changes = [
            change for change in self._lane_change_windows
            if change["start_s"] <= float(route_s_m) <= change["end_s"]
        ]
        polygons, allowed_lane_ids = [], set()
        for group in sorted(allowed_groups):
            polygon, lane_ids = self._corridor_polygon(group, float(route_s_m))
            if polygon:
                polygons.append(polygon)
                allowed_lane_ids.update(lane_ids)
        for change in active_changes:
            change_polygons, lane_ids = self._lane_change_polygons(
                change, float(route_s_m)
            )
            polygons.extend(change_polygons)
            allowed_lane_ids.update(lane_ids)
        return polygons, allowed_lane_ids

    def evaluate(self, vehicle, transform):
        location = transform.location
        match = self.matcher.match(
            float(location.x), float(location.y), float(transform.rotation.yaw)
        )
        route_index = max(0, min(match.index, len(self.rows) - 1))
        row = self.rows[route_index]
        waypoint = self._waypoints[route_index]
        polygons, allowed_lane_ids = self.corridor_polygons_for_match(
            route_index, match.route_s_m
        )
        active_lane_change_options = [
            change["option"] for change in self._lane_change_windows
            if change["start_s"] <= match.route_s_m <= change["end_s"]
        ]
        wheel_positions = self._wheel_world_positions(vehicle, transform)
        outside = classify_wheels(wheel_positions, polygons, self.tolerance_m)
        return LaneEvaluation(
            wheel_out_count=len(outside),
            outside_wheels=outside,
            allowed_lane_ids=tuple(sorted(allowed_lane_ids)),
            matched_route_index=route_index,
            matched_road_id=int(row["road_id"]),
            matched_section_id=int(row["section_id"]),
            matched_lane_id=int(row["lane_id"]),
            junction=bool(waypoint.is_junction),
            road_option=(active_lane_change_options[0]
                         if active_lane_change_options
                         else str(row.get("road_option", "LANEFOLLOW"))),
            route_match_distance_m=match.distance_m,
            wheel_position_frame=self._wheel_position_frame,
            wheelbase_m=self._wheelbase_m,
            front_track_m=self._front_track_m,
            rear_track_m=self._rear_track_m,
            wheel_positions=wheel_positions,
        )


def _reference_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes"}


class CsvCorridorEvaluator(WheelPositionProvider):
    """Evaluate wheel centers against precomputed route-indexed corridor cells."""

    def __init__(self, reference, carla_module, route_rows,
                 boundary_tolerance_m=0.01, route_backward_window=2,
                 route_forward_window=40, route_max_forward_jump=15):
        super().__init__(carla_module)
        self.reference = reference
        self.rows = list(route_rows)
        self.tolerance_m = float(boundary_tolerance_m)
        if self.tolerance_m < 0.0 or self.tolerance_m > 0.05:
            raise ValueError("boundary_tolerance_m must be in [0, 0.05] m")
        if len(self.rows) < 2:
            raise ValueError("lane evaluation requires at least two route rows")
        cells = reference.get("cells")
        if not isinstance(cells, list) or not cells:
            raise ValueError("CSV lane reference must contain corridor cells")
        self.cells = sorted(cells, key=lambda cell: float(cell["route_s_start_m"]))
        self.cell_starts = [float(cell["route_s_start_m"]) for cell in self.cells]
        self.max_cell_span_m = max(
            float(cell["route_s_end_m"]) - float(cell["route_s_start_m"])
            for cell in self.cells
        )
        self.matcher = RouteMatcher(
            self.rows, backward_window=route_backward_window,
            forward_window=route_forward_window,
            max_forward_jump=route_max_forward_jump,
        )
        self.last_debug_cells = []

    def _nearby_cells(self, route_s_m, wheel_reach_m):
        low = float(route_s_m) - wheel_reach_m
        high = float(route_s_m) + wheel_reach_m
        first = bisect_left(self.cell_starts, low - self.max_cell_span_m)
        last = bisect_right(self.cell_starts, high)
        return [
            cell for cell in self.cells[first:last]
            if float(cell["route_s_end_m"]) >= low
        ]

    def evaluate(self, vehicle, transform):
        location = transform.location
        match = self.matcher.match(
            float(location.x), float(location.y), float(transform.rotation.yaw)
        )
        route_index = max(0, min(match.index, len(self.rows) - 1))
        row = self.rows[route_index]
        wheel_positions = self._wheel_world_positions(vehicle, transform)
        wheel_reach = max(
            math.hypot(offset[0], offset[1]) for offset in self._wheel_offsets
        ) + self.max_cell_span_m + match.distance_m
        active_changes = [
            change for change in self.reference.get("lane_change_windows", [])
            if float(change["route_s_start_m"]) - wheel_reach
            <= match.route_s_m
            <= float(change["route_s_end_m"]) + wheel_reach
        ]
        active_change_ids = {int(change["lane_change_id"])
                             for change in active_changes}
        transition_cells = [
            cell for cell in self.cells
            if cell.get("kind") == "lane_change"
            and int(cell.get("lane_change_id", -1)) in active_change_ids
        ]
        wheel_cells = {}
        outside = []
        for name in WHEEL_NAMES:
            point = wheel_positions[name]
            wheel_match = self.matcher.project_nearby(
                float(point["x"]), float(point["y"]),
                float(transform.rotation.yaw), match.route_s_m, wheel_reach,
            )
            local_cells = self._nearby_cells(
                wheel_match.route_s_m, self.max_cell_span_m
            )
            allowed_cells = local_cells + transition_cells
            wheel_cells[name] = allowed_cells
            if not any(point_in_polygon(point, cell["polygon"], self.tolerance_m)
                       for cell in allowed_cells):
                outside.append(name)
        cells = list({id(cell): cell for selected in wheel_cells.values()
                      for cell in selected}.values())
        self.last_debug_cells = cells
        allowed_lane_ids = tuple(sorted({
            (int(cell["road_id"]), int(cell["section_id"]), int(cell["lane_id"]))
            for cell in cells
        }))
        road_option = (
            str(active_changes[0]["road_option"])
            if active_changes else str(row.get("road_option", "LANEFOLLOW"))
        )
        junction = _reference_bool(row.get("is_junction", False))
        return LaneEvaluation(
            wheel_out_count=len(outside),
            outside_wheels=tuple(outside),
            allowed_lane_ids=allowed_lane_ids,
            matched_route_index=route_index,
            matched_road_id=int(row["road_id"]),
            matched_section_id=int(row["section_id"]),
            matched_lane_id=int(row["lane_id"]),
            junction=junction,
            road_option=road_option,
            route_match_distance_m=match.distance_m,
            wheel_position_frame=self._wheel_position_frame,
            wheelbase_m=self._wheelbase_m,
            front_track_m=self._front_track_m,
            rear_track_m=self._rear_track_m,
            wheel_positions=wheel_positions,
        )


class LanePenaltyState:
    """Competition penalty clock, independent from corridor geometry."""

    def __init__(self, first_penalty_sec=20.0, continuous_penalty_sec=20.0,
                 continuous_interval_sec=5.0):
        self.first_penalty_sec = float(first_penalty_sec)
        self.continuous_penalty_sec = float(continuous_penalty_sec)
        self.continuous_interval_sec = float(continuous_interval_sec)
        if self.first_penalty_sec < 0 or self.continuous_penalty_sec < 0:
            raise ValueError("lane penalties cannot be negative")
        if self.continuous_interval_sec <= 0:
            raise ValueError("continuous penalty interval must be positive")
        self.violation_start_t = None
        self.next_penalty_t = None
        self.last_violation_detail = ""
        self.full_exit_active = False
        self.emergency_stop_center_relocation_required = False

    def continuous_sec(self, now):
        if self.violation_start_t is None:
            return 0.0
        return max(0.0, float(now) - self.violation_start_t)

    def update(self, now, wheel_out_count, detail):
        now = float(now)
        count = int(wheel_out_count)
        if not 0 <= count <= 4:
            raise ValueError("wheel_out_count must be between zero and four")
        events = []

        # Charge every elapsed five-second threshold, including a threshold
        # reached at the sample that clears the violation.
        if self.violation_start_t is not None:
            penalty_detail = detail if count > 0 else self.last_violation_detail
            while self.next_penalty_t is not None and now + 1e-9 >= self.next_penalty_t:
                continuous = self.next_penalty_t - self.violation_start_t
                events.append(LanePenaltyEvent(
                    "LANE_CROSS_CONTINUOUS", self.continuous_penalty_sec,
                    f"{penalty_detail}, continuous_violation_sec={continuous:.3f}"
                ))
                self.next_penalty_t += self.continuous_interval_sec
            if count > 0:
                self.last_violation_detail = detail

        if count > 0 and self.violation_start_t is None:
            self.violation_start_t = now
            self.next_penalty_t = now + self.continuous_interval_sec
            self.last_violation_detail = detail
            events.append(LanePenaltyEvent(
                "LANE_CROSS_START", self.first_penalty_sec,
                f"{detail}, continuous_violation_sec=0.000"
            ))
        elif count == 0 and self.violation_start_t is not None:
            continuous = self.continuous_sec(now)
            events.append(LanePenaltyEvent(
                "LANE_CROSS_END", 0.0,
                f"{detail}, continuous_violation_sec={continuous:.3f}"
            ))
            self.violation_start_t = None
            self.next_penalty_t = None
            self.last_violation_detail = ""

        if count == 4 and not self.full_exit_active:
            self.full_exit_active = True
            self.emergency_stop_center_relocation_required = True
            events.append(LanePenaltyEvent(
                "FULL_LANE_EXIT", 0.0,
                f"{detail}, emergency_stop_center_relocation_required=1"
            ))
        elif count < 4:
            self.full_exit_active = False
        return events
