#!/usr/bin/env python3
"""
K-City competition + research benchmark.

Two outputs are intentionally separated.

A. Competition record
   final_record = driving_time + official penalty time

B. Benchmark score (0-100)
   configurable weighted score of:
   - Completion
   - Safety
   - Mission
   - Efficiency
   - Comfort

Key design decision
-------------------
Lane-center CTE is NOT part of the benchmark score.
A legal out-in-out trajectory should not lose points simply because it is far
from the lane center. Route-center distance is stored only as a diagnostic.

If a planner/controller reference path is supplied separately, its tracking
error is also stored as a controller diagnostic, not as a whole-stack score.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
import traceback
from typing import Dict, Optional

import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kcity_scenario_manager.route_csv import read_route_csv, resolve_route_csv_path

from .common import (
    TriggerBox,
    evaluation_location,
    get_trigger,
    connect_carla,
    find_ego_vehicle,
    load_yaml,
    resolve_traffic_light,
    speed_mps,
    state_name,
    suite_traffic_light_configs,
)
from .result_manager import ResultManager
from .lap_timer import LapTimer
from .lane_evaluator import (
    CsvCorridorEvaluator,
    LaneGeometryEvaluator,
    LanePenaltyState,
)
from .lane_reference import load_lane_reference, sha256_file
from .route_tracker import PolylineTracker, load_route_reference, wrap_deg
from .scoring import build_scorecard
from .score_schema import score_details


def percentile(values, q):
    if not values:
        return None
    values = sorted(float(v) for v in values)
    if len(values) == 1:
        return values[0]

    pos = (len(values) - 1) * float(q)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]

    frac = pos - lo
    return values[lo] * (1.0 - frac) + values[hi] * frac


@dataclass
class MissionRuntime:
    cfg: dict
    trigger: TriggerBox
    status: str = "PENDING"
    applied_penalty_sec: float = 0.0

    stop_start_t: Optional[float] = None
    best_continuous_stop_sec: float = 0.0
    stop_threshold_events_sent: set[float] = field(default_factory=set)
    stop_requirement_event_sent: bool = False
    signal_last_state: Optional[str] = None
    signal_stop_start_t: Optional[float] = None
    signal_stop_started_event_sent: bool = False
    red_stop_success_event_sent: bool = False
    collision_seen: bool = False
    was_inside: bool = False
    prev_eval_loc: Optional[carla.Location] = None


def update_stop_progress(runtime, now, inside, speed):
    threshold=float(runtime.cfg.get('stop_speed_threshold_mps',0.10))
    if inside and speed<=threshold:
        if runtime.stop_start_t is None:
            runtime.stop_start_t=now
        runtime.best_continuous_stop_sec=max(
            runtime.best_continuous_stop_sec,
            now-runtime.stop_start_t,
        )
    else:
        runtime.stop_start_t=None


def evaluate_traffic_light_mission(mission_cfg,traffic_lights):
    key=str(mission_cfg.get('light_key',''))
    light=traffic_lights.get(key)
    if light is None:
        return None
    signal_state=state_name(light)
    fail_states=[str(x) for x in mission_cfg.get('fail_states',['Red'])]
    return signal_state not in fail_states,signal_state


class CompetitionBenchmark(Node):
    def __init__(self):
        super().__init__("kcity_benchmark")

        self.declare_parameter("suite", "qualifier")
        self.declare_parameter("config_file", "")
        self.declare_parameter("require_ego_role", False)
        self.declare_parameter("lane_geometry_source", "")
        self.declare_parameter("lane_debug_draw", False)

        self.suite = str(self.get_parameter("suite").value)
        config_file = str(self.get_parameter("config_file").value)
        if not config_file:
            raise RuntimeError("config_file parameter is required.")

        self.cfg = load_yaml(config_file)
        self.bcfg = self.cfg["benchmark"]
        self.client, self.world = connect_carla(self.cfg.get("carla", {}))

        self.ego_role = str(
            self.cfg.get("carla", {}).get("ego_role_name", "ego_vehicle")
        )
        self.strict_role = bool(self.get_parameter("require_ego_role").value)
        self.time_limit_sec = float(self.bcfg.get("time_limit_sec", 600.0))

        self.start_trigger = get_trigger(
            self.cfg,str(self.bcfg.get('start_trigger_key','START')),
            require_exit_edge=True,
        )
        self.finish_trigger = get_trigger(
            self.cfg,str(self.bcfg.get('finish_trigger_key','FINISH')),
            require_exit_edge=True,
        )
        self.lap_timer = LapTimer(self.start_trigger, self.finish_trigger)

        route_points,self.route_metadata=load_route_reference(
            config_file,self.bcfg
        )
        self.route_tracker=PolylineTracker(route_points)
        self.planner_tracker = PolylineTracker(
            self.bcfg.get("planner_reference_points", [])
        )

        # -----------------------------------------------------
        # Official lane rule
        # -----------------------------------------------------
        lane_cfg = self.bcfg.get("lane_rule", {})
        self.lane_enabled = bool(lane_cfg.get("enabled", True))
        self.lane_first_penalty = float(
            lane_cfg.get("first_crossing_penalty_sec", 20.0)
        )
        self.lane_cont_penalty = float(
            lane_cfg.get("continuous_penalty_sec", 20.0)
        )
        self.lane_cont_interval = float(
            lane_cfg.get("continuous_interval_sec", 5.0)
        )
        self.lane_tolerance = float(
            lane_cfg.get("boundary_tolerance_m", 0.05)
        )
        self.lane_geometry_source = str(
            self.get_parameter("lane_geometry_source").value
            or lane_cfg.get("geometry_source", "xodr")
        ).strip().lower()
        self.lane_debug_draw = bool(self.get_parameter("lane_debug_draw").value)
        self.lane_reference_sha256 = None
        self.lane_evaluator = None
        self.lane_penalty_state = LanePenaltyState(
            self.lane_first_penalty,
            self.lane_cont_penalty,
            self.lane_cont_interval,
        )
        self.latest_lane_evaluation = None
        if self.lane_enabled:
            route_csv = str(self.bcfg.get("route_csv", "")).strip()
            if not route_csv:
                raise RuntimeError(
                    "lane evaluation requires benchmark.route_csv metadata"
                )
            route_path = resolve_route_csv_path(config_file, route_csv)
            lane_route_rows = read_route_csv(route_path)
            if self.lane_geometry_source == "xodr":
                self.lane_evaluator = LaneGeometryEvaluator(
                    self.world.get_map(),
                    carla,
                    lane_route_rows,
                    boundary_tolerance_m=self.lane_tolerance,
                )
            elif self.lane_geometry_source == "csv_corridor":
                configured_reference = str(lane_cfg.get(
                    "reference_file", "data/qualifier_lane_reference.json"
                )).strip()
                reference_path = Path(configured_reference).expanduser()
                if not reference_path.is_absolute():
                    reference_path = Path(__file__).resolve().parent / reference_path
                route_digest = sha256_file(route_path)
                reference = load_lane_reference(
                    reference_path, lane_route_rows, route_digest
                )
                self.lane_reference_sha256 = sha256_file(reference_path)
                self.lane_evaluator = CsvCorridorEvaluator(
                    reference,
                    carla,
                    lane_route_rows,
                    boundary_tolerance_m=self.lane_tolerance,
                )
            else:
                raise RuntimeError(
                    "benchmark.lane_rule.geometry_source must be 'xodr' or "
                    f"'csv_corridor', got {self.lane_geometry_source!r}"
                )
        # -----------------------------------------------------
        # Research benchmark score configuration
        # -----------------------------------------------------
        scoring_cfg = self.bcfg.get("scoring", {})
        self.quality_weights = scoring_cfg.get(
            "quality_weights",
            {
                "safety": 0.40,
                "mission": 0.35,
                "efficiency": 0.15,
                "comfort": 0.10,
            },
        )

        efficiency_cfg = scoring_cfg.get("efficiency", {})
        self.reference_time_sec = efficiency_cfg.get("reference_time_sec")

        safety_cfg = scoring_cfg.get("safety", {})
        self.full_exit_multiplier = float(
            safety_cfg.get("full_lane_exit_multiplier", 0.50)
        )

        self.collision_actor_multipliers = safety_cfg.get(
            "collision_actor_multipliers",
            {
                "pedestrian": 0.50,
                "vehicle": 0.60,
                "static": 0.65,
                "other": 0.65,
            },
        )
        severity_cfg = safety_cfg.get("collision_severity", {})
        self.severity_thresholds = severity_cfg.get(
            "delta_v_thresholds_mps",
            [0.5, 2.0, 5.0],
        )
        self.severity_weights = severity_cfg.get(
            "weights",
            {
                "contact": 0.25,
                "minor": 0.50,
                "moderate": 1.00,
                "severe": 1.50,
            },
        )
        self.collision_dedupe_sec = float(
            safety_cfg.get("collision_dedupe_sec", 1.0)
        )

        speed_cfg = scoring_cfg.get("speed_rule", {})
        self.speed_rule_enabled = bool(speed_cfg.get("enabled", False))
        self.speed_tolerance_mps = float(
            speed_cfg.get("overspeed_tolerance_mps", 0.5)
        )
        self.include_speed_in_safety = bool(
            speed_cfg.get("include_in_safety_score", False)
        )

        comfort_cfg = scoring_cfg.get("comfort", {})
        # Bench2Drive-like defaults. All are configurable.
        self.comfort_thresholds = {
            "lat_accel_abs_max": float(
                comfort_cfg.get("lat_accel_abs_max_mps2", 4.89)
            ),
            "long_accel_min": float(
                comfort_cfg.get("long_accel_min_mps2", -4.05)
            ),
            "long_accel_max": float(
                comfort_cfg.get("long_accel_max_mps2", 2.40)
            ),
            "jerk_abs_max": float(
                comfort_cfg.get("jerk_abs_max_mps3", 8.37)
            ),
            "long_jerk_abs_max": float(
                comfort_cfg.get("long_jerk_abs_max_mps3", 4.13)
            ),
            "yaw_rate_abs_max": float(
                comfort_cfg.get("yaw_rate_abs_max_radps", 0.95)
            ),
            "yaw_accel_abs_max": float(
                comfort_cfg.get("yaw_accel_abs_max_radps2", 1.93)
            ),
        }

        # -----------------------------------------------------
        # Mission configs
        # -----------------------------------------------------
        self.missions: Dict[str, MissionRuntime] = {}
        for mcfg in self.bcfg.get("missions", []):
            self.missions[str(mcfg["id"])] = MissionRuntime(
                cfg=mcfg,
                trigger=get_trigger(
                    self.cfg,str(mcfg["trigger_key"]),require_exit_edge=True,
                ),
            )

        self.traffic_light_cfgs=suite_traffic_light_configs(
            self.cfg,self.suite
        )

        self.traffic_lights = {}
        self.traffic_light_resolution_errors = {}

        output_root = str(
            self.bcfg.get("output_root", "~/.ros/heven_carla/results")
        )
        self.results = ResultManager(self.suite, output_root)

        # -----------------------------------------------------
        # Runtime state
        # -----------------------------------------------------
        self.ego = None
        self.collision_sensor = None
        self.pending_collisions = []
        self.last_collision_by_actor = {}

        self.started = False
        self.finished = False
        self.start_t = None
        self.finish_t = None
        self.start_wall_monotonic = None
        self.start_wall_timestamp = None
        self.finish_wall_timestamp = None
        self.wall_time_sec = None
        self.latest_sim_time = None
        self.latest_mission_result = None
        self.current_mission = None
        self.final_status = None
        self.final_scores = None
        self.final_record_sec = None
        self.status_pub = self.create_publisher(String, "/kcity/benchmark/status", 10)
        self.status_timer = self.create_timer(0.2, self.publish_status)

        self.lane_crossing = False
        self.full_lane_exit = False
        self.emergency_stop_center_relocation_required = False

        self.lane_crossing_time_sec = 0.0
        self.full_lane_exit_count = 0
        self.overspeed_time_sec = 0.0

        self.collision_factor = 1.0
        self.collision_records = []

        self.route_center_distances = []
        self.planner_tracking_errors = []
        self.planner_heading_errors = []

        self.comfortable_time_sec = 0.0
        self.comfort_evaluated_time_sec = 0.0

        self.prev_sample_t = None
        self.prev_accel_vec = None
        self.prev_long_accel = None
        self.prev_yaw_rate = None

        self.last_progress = None

        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f"K-City benchmark v2 started: suite={self.suite}, config={config_file}"
        )
        self.get_logger().info(
            "Competition record = drive time + official penalties; "
            "Benchmark score = Completion/Safety/Mission/Efficiency/Comfort."
        )
        self.get_logger().info(
            "Lane-center CTE is diagnostic only and does NOT reduce score."
        )

    # ---------------------------------------------------------
    # CARLA setup
    # ---------------------------------------------------------
    def ensure_ego(self) -> bool:
        if self.ego is not None and self.ego.is_alive:
            return True

        self.ego = find_ego_vehicle(self.world, self.ego_role, self.strict_role)
        if self.ego is None:
            return False

        self.get_logger().info(f"Ego vehicle resolved: id={self.ego.id}")
        self.spawn_collision_sensor()
        return True

    def spawn_collision_sensor(self):
        if self.collision_sensor is not None:
            try:
                if self.collision_sensor.is_alive:
                    return
            except Exception:
                pass

        bp = self.world.get_blueprint_library().find("sensor.other.collision")
        self.collision_sensor = self.world.spawn_actor(
            bp,
            carla.Transform(),
            attach_to=self.ego,
        )
        self.collision_sensor.listen(self.on_collision)

    def on_collision(self, event):
        impulse = event.normal_impulse
        intensity = math.sqrt(
            impulse.x * impulse.x
            + impulse.y * impulse.y
            + impulse.z * impulse.z
        )

        self.pending_collisions.append(
            {
                "sim_time": float(event.timestamp),
                "other_actor_id": int(event.other_actor.id),
                "other_type": str(event.other_actor.type_id),
                "impulse_norm": float(intensity),
            }
        )

    def ensure_lights(self):
        for key, cfg in self.traffic_light_cfgs.items():
            actor = self.traffic_lights.get(key)
            if actor is None or not actor.is_alive:
                try:
                    self.traffic_lights[key] = resolve_traffic_light(
                        self.world, cfg
                    )
                except RuntimeError as exc:
                    message=str(exc)
                    if self.traffic_light_resolution_errors.get(key)!=message:
                        self.get_logger().warning(f'{key}: {message}')
                        self.traffic_light_resolution_errors[key]=message
                else:
                    self.traffic_light_resolution_errors.pop(key,None)

    # ---------------------------------------------------------
    # Competition timing
    # ---------------------------------------------------------
    def should_start(self, loc) -> bool:
        """Compatibility helper; actual start is handled by LapTimer crossing."""
        return self.start_trigger.contains(loc)

    def should_finish(self, loc) -> bool:
        if (
            not self.finish_trigger.enabled
            and self.last_progress is not None
            and self.last_progress >= 99.5
        ):
            return True

        return False

    # ---------------------------------------------------------
    # Lane / corridor safety
    # ---------------------------------------------------------
    def evaluate_lane(self, now: float, dt: float, transform):
        if not self.lane_enabled:
            return False, False
        if self.lane_evaluator is None:
            raise RuntimeError("lane evaluator was not initialized")

        evaluation = self.lane_evaluator.evaluate(self.ego, transform)
        self.latest_lane_evaluation = evaluation
        if self.lane_debug_draw:
            self.draw_lane_debug(evaluation)
        crossing = evaluation.violation
        full_exit = evaluation.full_lane_exit
        if crossing:
            self.lane_crossing_time_sec += max(dt, 0.0)

        detail = evaluation.detail()
        for event in self.lane_penalty_state.update(
            now, evaluation.wheel_out_count, detail
        ):
            if event.event == "FULL_LANE_EXIT":
                self.full_lane_exit_count += 1
                self.emergency_stop_center_relocation_required = True
                category = "safety"
            else:
                category = "lane"
            self.results.add_event(
                now,
                event.event,
                detail=event.detail,
                penalty_sec=event.penalty_sec,
                category=category,
            )

        self.lane_crossing = self.lane_penalty_state.violation_start_t is not None
        self.full_lane_exit = self.lane_penalty_state.full_exit_active
        return crossing, full_exit

    def draw_lane_debug(self, evaluation):
        """Optional, non-controlling CARLA debug overlay for lane geometry."""
        debug = self.world.debug
        lifetime = 0.15
        base_z = float(self.ego.get_location().z) + 0.25
        cells = getattr(self.lane_evaluator, "last_debug_cells", [])
        for cell in cells:
            polygon = cell.get("polygon", [])
            if len(polygon) != 4:
                continue
            color = (carla.Color(255, 0, 220)
                     if cell.get("kind") == "lane_change"
                     else carla.Color(0, 210, 255))
            points = [carla.Location(x=float(point[0]), y=float(point[1]), z=base_z)
                      for point in polygon]
            for first, second in zip(points, points[1:] + points[:1]):
                debug.draw_line(first, second, thickness=0.025,
                                color=color, life_time=lifetime)
            center_first = carla.Location(
                x=(polygon[0][0] + polygon[3][0]) * 0.5,
                y=(polygon[0][1] + polygon[3][1]) * 0.5,
                z=base_z,
            )
            center_second = carla.Location(
                x=(polygon[1][0] + polygon[2][0]) * 0.5,
                y=(polygon[1][1] + polygon[2][1]) * 0.5,
                z=base_z,
            )
            debug.draw_line(center_first, center_second, thickness=0.015,
                            color=carla.Color(30, 120, 255), life_time=lifetime)
        outside = set(evaluation.outside_wheels)
        for name, point in (evaluation.wheel_positions or {}).items():
            location = carla.Location(
                x=float(point["x"]), y=float(point["y"]), z=float(point["z"]) + 0.15
            )
            color = carla.Color(255, 30, 30) if name in outside else carla.Color(40, 255, 40)
            debug.draw_point(location, size=0.12, color=color, life_time=lifetime)
            debug.draw_string(location + carla.Location(z=0.15), name,
                              color=color, life_time=lifetime)
        ego_location = self.ego.get_location()
        label_location = carla.Location(
            x=float(ego_location.x), y=float(ego_location.y),
            z=float(ego_location.z) + 2.0,
        )
        debug.draw_string(
            label_location,
            f"route_index={evaluation.matched_route_index} "
            f"out={evaluation.wheel_out_count} RoadOption={evaluation.road_option}",
            color=carla.Color(255, 255, 255), life_time=lifetime,
        )

    # ---------------------------------------------------------
    # Collision safety
    # ---------------------------------------------------------
    def classify_actor(self, type_id: str) -> str:
        if type_id.startswith("walker.pedestrian"):
            return "pedestrian"
        if type_id.startswith("vehicle."):
            return "vehicle"
        if type_id.startswith("static.") or type_id.startswith("traffic."):
            return "static"
        return "other"

    def collision_severity(self, delta_v_mps: float):
        t = [float(x) for x in self.severity_thresholds]
        if delta_v_mps < t[0]:
            return "contact"
        if delta_v_mps < t[1]:
            return "minor"
        if delta_v_mps < t[2]:
            return "moderate"
        return "severe"

    def consume_collisions(self, now: float):
        if not self.pending_collisions:
            return

        try:
            mass = float(self.ego.get_physics_control().mass)
        except Exception:
            mass = 1.0
        mass = max(mass, 1e-6)

        for collision in self.pending_collisions:
            actor_id = collision["other_actor_id"]
            event_t = collision["sim_time"]
            if self.start_t is not None and event_t < self.start_t:
                continue

            # Deduplicate repeated collision callbacks with the same object.
            last_t = self.last_collision_by_actor.get(actor_id)
            if (
                last_t is not None
                and event_t - last_t < self.collision_dedupe_sec
            ):
                continue
            self.last_collision_by_actor[actor_id] = event_t

            delta_v = collision["impulse_norm"] / mass
            severity = self.collision_severity(delta_v)
            actor_class = self.classify_actor(collision["other_type"])

            actor_multiplier = float(
                self.collision_actor_multipliers.get(
                    actor_class,
                    self.collision_actor_multipliers.get("other", 0.65),
                )
            )
            severity_weight = float(
                self.severity_weights.get(severity, 1.0)
            )

            # Make very light contact less severe than the base actor multiplier,
            # and strong collisions more severe.
            event_multiplier = 1.0 - severity_weight * (
                1.0 - actor_multiplier
            )
            event_multiplier = max(0.05, min(1.0, event_multiplier))

            self.collision_factor *= event_multiplier

            record = dict(collision)
            record.update(
                {
                    "actor_class": actor_class,
                    "equivalent_delta_v_mps": delta_v,
                    "severity": severity,
                    "benchmark_multiplier": event_multiplier,
                }
            )
            self.collision_records.append(record)

            self.results.add_event(
                now,
                "COLLISION",
                detail=(
                    f"type={collision['other_type']}, "
                    f"severity={severity}, "
                    f"eq_delta_v={delta_v:.3f}m/s, "
                    f"benchmark_multiplier={event_multiplier:.3f}"
                ),
                penalty_sec=0,
                category="safety",
            )

    # ---------------------------------------------------------
    # Comfort / dynamics
    # ---------------------------------------------------------
    def evaluate_dynamics(self, now: float, dt: float, transform):
        accel = self.ego.get_acceleration()
        forward = transform.get_forward_vector()
        right = transform.get_right_vector()

        long_accel = (
            accel.x * forward.x
            + accel.y * forward.y
            + accel.z * forward.z
        )
        lat_accel = (
            accel.x * right.x
            + accel.y * right.y
            + accel.z * right.z
        )

        # CARLA angular velocity is reported in degrees/s.
        angular = self.ego.get_angular_velocity()
        yaw_rate = math.radians(float(angular.z))

        jerk = None
        long_jerk = None
        yaw_accel = None
        comfortable = None

        if (
            dt > 1e-6
            and self.prev_accel_vec is not None
            and self.prev_long_accel is not None
            and self.prev_yaw_rate is not None
        ):
            dax = accel.x - self.prev_accel_vec.x
            day = accel.y - self.prev_accel_vec.y
            daz = accel.z - self.prev_accel_vec.z

            jerk = math.sqrt(dax * dax + day * day + daz * daz) / dt
            long_jerk = (long_accel - self.prev_long_accel) / dt
            yaw_accel = (yaw_rate - self.prev_yaw_rate) / dt

            th = self.comfort_thresholds
            comfortable = (
                abs(lat_accel) <= th["lat_accel_abs_max"]
                and th["long_accel_min"]
                <= long_accel
                <= th["long_accel_max"]
                and abs(jerk) <= th["jerk_abs_max"]
                and abs(long_jerk) <= th["long_jerk_abs_max"]
                and abs(yaw_rate) <= th["yaw_rate_abs_max"]
                and abs(yaw_accel) <= th["yaw_accel_abs_max"]
            )

            self.comfort_evaluated_time_sec += dt
            if comfortable:
                self.comfortable_time_sec += dt

        self.prev_accel_vec = carla.Vector3D(accel.x, accel.y, accel.z)
        self.prev_long_accel = long_accel
        self.prev_yaw_rate = yaw_rate

        return (
            long_accel,
            lat_accel,
            jerk,
            long_jerk,
            yaw_rate,
            yaw_accel,
            comfortable,
        )

    # ---------------------------------------------------------
    # Optional speed compliance
    # ---------------------------------------------------------
    def evaluate_speed(self, dt: float, speed: float) -> bool:
        if not self.speed_rule_enabled:
            return False

        try:
            speed_limit_kmh = float(self.ego.get_speed_limit())
        except Exception:
            return False

        if speed_limit_kmh <= 0:
            return False

        speed_limit_mps = speed_limit_kmh / 3.6
        overspeed = speed > speed_limit_mps + self.speed_tolerance_mps

        if overspeed:
            self.overspeed_time_sec += max(dt, 0.0)

        return overspeed

    # ---------------------------------------------------------
    # Mission evaluation
    # ---------------------------------------------------------
    def apply_mission_result(
        self,
        runtime: MissionRuntime,
        now: float,
        passed: bool,
        detail: str,
    ):
        if runtime.status != "PENDING":
            return

        runtime.status = "PASS" if passed else "FAIL"
        self.latest_mission_result = {
            "id": str(runtime.cfg["id"]), "status": runtime.status,
            "detail": detail, "sim_time_sec": now,
        }
        penalty = 0.0 if passed else float(
            runtime.cfg.get("penalty_sec", 0.0)
        )
        runtime.applied_penalty_sec = penalty

        self.results.add_event(
            now,
            "MISSION_PASS" if passed else "MISSION_FAIL",
            mission_id=str(runtime.cfg["id"]),
            detail=detail,
            penalty_sec=penalty,
            category="mission",
        )

    def add_logging_event(
        self,
        runtime: MissionRuntime,
        now: float,
        event: str,
        detail: str,
    ):
        """Record an informational mission event without changing score state."""
        self.results.add_event(
            now,
            event,
            mission_id=str(runtime.cfg["id"]),
            detail=detail,
            category="mission",
        )

    @staticmethod
    def stop_event_detail(speed: float, continuous: float, required: float, **fields):
        parts = [
            f"speed={speed:.3f}m/s",
            f"continuous_stop_sec={continuous:.3f}",
            f"required_stop_sec={required:.3f}",
        ]
        parts.extend(f"{key}={value}" for key, value in fields.items())
        return ", ".join(parts)

    def update_stop_logging(self, runtime: MissionRuntime, now: float, inside: bool, speed: float):
        """Add stop-progress events while preserving the existing progress calculation."""
        previous_start = runtime.stop_start_t
        update_stop_progress(runtime, now, inside, speed)
        if runtime.stop_start_t is None:
            return

        required = float(runtime.cfg.get("required_stop_sec", 3.0))
        continuous = max(0.0, now - runtime.stop_start_t)
        if previous_start is None:
            self.add_logging_event(
                runtime,
                now,
                "STOP_STARTED",
                self.stop_event_detail(speed, continuous, required),
            )

        for threshold in (1.0, 2.0):
            if continuous >= threshold and threshold not in runtime.stop_threshold_events_sent:
                runtime.stop_threshold_events_sent.add(threshold)
                self.add_logging_event(
                    runtime,
                    now,
                    "STOP_PROGRESS",
                    self.stop_event_detail(
                        speed,
                        continuous,
                        required,
                        threshold_sec=f"{threshold:.1f}",
                    ),
                )

        if continuous >= required and not runtime.stop_requirement_event_sent:
            runtime.stop_requirement_event_sent = True
            self.add_logging_event(
                runtime,
                now,
                "STOP_REQUIREMENT_MET",
                self.stop_event_detail(speed, continuous, required),
            )

    def traffic_light_state(self, runtime: MissionRuntime):
        light_key = str(runtime.cfg.get("light_key", ""))
        light = self.traffic_lights.get(light_key)
        return None if light is None else state_name(light)

    def traffic_light_detail(self, runtime: MissionRuntime, signal_state: Optional[str], **fields):
        light_key = str(runtime.cfg.get("light_key", ""))
        parts = [
            f"trigger={runtime.cfg.get('trigger_key', '')}",
            f"signal={signal_state if signal_state is not None else 'Unavailable'}",
        ]
        signal_type = str(
            self.traffic_light_cfgs.get(light_key, {}).get("signal_type", "")
        ).strip()
        if signal_type:
            parts.append(f"signal_type={signal_type}")
        parts.extend(f"{key}={value}" for key, value in fields.items())
        return ", ".join(parts)

    def update_signal_logging(
        self,
        runtime: MissionRuntime,
        now: float,
        inside: bool,
        speed: float,
        signal_state: Optional[str],
    ):
        """Log observed in-trigger signal behavior without participating in scoring."""
        if not inside or signal_state is None:
            runtime.signal_last_state = None
            runtime.signal_stop_start_t = None
            return

        previous_state = runtime.signal_last_state
        previous_red_stop_start = runtime.signal_stop_start_t
        if previous_state is not None and signal_state != previous_state:
            self.add_logging_event(
                runtime,
                now,
                "SIGNAL_STATE_CHANGED",
                self.traffic_light_detail(
                    runtime,
                    signal_state,
                    from_signal=previous_state,
                    to_signal=signal_state,
                    speed=f"{speed:.3f}m/s",
                ),
            )
            if (
                previous_state == "Red"
                and signal_state != "Red"
                and previous_red_stop_start is not None
                and not runtime.red_stop_success_event_sent
            ):
                runtime.red_stop_success_event_sent = True
                self.add_logging_event(
                    runtime,
                    now,
                    "RED_STOP_SUCCESS",
                    self.traffic_light_detail(
                        runtime,
                        signal_state,
                        from_signal=previous_state,
                        stopped_on_red_sec=f"{max(0.0, now - previous_red_stop_start):.3f}",
                        speed=f"{speed:.3f}m/s",
                    ),
                )

        stop_threshold = float(runtime.cfg.get("stop_speed_threshold_mps", 0.10))
        if signal_state == "Red" and speed <= stop_threshold:
            if runtime.signal_stop_start_t is None:
                runtime.signal_stop_start_t = now
                if not runtime.signal_stop_started_event_sent:
                    runtime.signal_stop_started_event_sent = True
                    self.add_logging_event(
                        runtime,
                        now,
                        "SIGNAL_STOP_STARTED",
                        self.traffic_light_detail(
                            runtime,
                            signal_state,
                            speed=f"{speed:.3f}m/s",
                        ),
                    )
        else:
            runtime.signal_stop_start_t = None
        runtime.signal_last_state = signal_state


    def process_missions(self, now, loc, speed):
        for runtime in self.missions.values():
            if runtime.status != "PENDING":
                continue

            inside = runtime.trigger.contains(loc)
            eval_loc = evaluation_location(self.ego, runtime.trigger)
            crossed_exit = runtime.trigger.crossed_exit(runtime.prev_eval_loc, eval_loc)
            mtype = str(runtime.cfg.get("type", "checkpoint"))
            signal_state = (
                self.traffic_light_state(runtime)
                if mtype == "traffic_light"
                else None
            )

            if inside and not runtime.was_inside:
                detail = str(runtime.cfg.get("trigger_key", ""))
                if mtype == "traffic_light":
                    detail = self.traffic_light_detail(runtime, signal_state)
                self.results.add_event(now, "MISSION_TRIGGER_ENTER", mission_id=str(runtime.cfg["id"]), detail=detail, category="mission")

            if mtype == "stop":
                self.update_stop_logging(runtime,now,inside,speed)

            if mtype == "traffic_light":
                self.update_signal_logging(
                    runtime, now, inside, speed, signal_state
                )

            if mtype == "obstacle" and inside and self.pending_collisions:
                runtime.collision_seen = True

            if crossed_exit:
                if mtype == "checkpoint":
                    self.apply_mission_result(runtime, now, True, "trigger exit edge crossed")
                elif mtype == "stop":
                    required = float(runtime.cfg.get("required_stop_sec", 3.0))
                    passed = runtime.best_continuous_stop_sec >= required
                    self.apply_mission_result(runtime, now, passed, f"best continuous stop={runtime.best_continuous_stop_sec:.2f}s, required={required:.2f}s")
                elif mtype == "traffic_light":
                    result=evaluate_traffic_light_mission(
                        runtime.cfg,self.traffic_lights
                    )
                    if result is not None:
                        passed,signal_state=result
                        self.apply_mission_result(
                            runtime,now,passed,
                            f"trigger exit edge crossed with signal={signal_state}",
                        )
                elif mtype == "obstacle":
                    self.apply_mission_result(runtime, now, not runtime.collision_seen, f"obstacle exit edge crossed; collision_seen={runtime.collision_seen}")

            runtime.was_inside = inside
            runtime.prev_eval_loc = eval_loc

    # ---------------------------------------------------------
    # Finalization
    # ---------------------------------------------------------
    def fail_unresolved_missions(self, now: float):
        for runtime in self.missions.values():
            if runtime.status != "PENDING":
                continue

            penalty = float(runtime.cfg.get("penalty_sec", 0.0))
            runtime.status = "MISSED"
            runtime.applied_penalty_sec = penalty
            self.latest_mission_result = {
                "id": str(runtime.cfg["id"]), "status": "MISSED",
                "detail": "run ended before trigger exit edge", "sim_time_sec": now,
            }

            self.results.add_event(
                now,
                "MISSION_MISSED",
                mission_id=str(runtime.cfg["id"]),
                detail="run ended before trigger exit edge was crossed",
                penalty_sec=penalty,
                category="mission",
            )

    def mission_summary(self):
        summary = []
        for runtime in self.missions.values():
            summary.append(
                {
                    "id": str(runtime.cfg["id"]),
                    "name": str(runtime.cfg.get("name", "")),
                    "type": str(runtime.cfg.get("type", "")),
                    "trigger_key": str(runtime.cfg.get("trigger_key", "")),
                    "status": runtime.status,
                    "configured_penalty_sec": float(
                        runtime.cfg.get("penalty_sec", 0.0)
                    ),
                    "applied_penalty_sec": runtime.applied_penalty_sec,
                    "scored": bool(runtime.cfg.get("scored", False)),
                    "score_weight": float(
                        runtime.cfg.get("score_weight", 1.0)
                    ),
                }
            )
        return summary

    def stats_or_none(self, values):
        if not values:
            return {
                "rmse": None,
                "p95": None,
                "max": None,
            }

        rmse = math.sqrt(
            sum(float(v) ** 2 for v in values) / len(values)
        )
        return {
            "rmse": rmse,
            "p95": percentile(values, 0.95),
            "max": max(values),
        }

    def finish_run(self, now: float, status: str):
        if self.finished:
            return

        self.finished = True
        self.final_status = status
        self.finish_t = (
            self.lap_timer.finish_sim_time_sec
            if status == "FINISH" and self.lap_timer.finish_sim_time_sec is not None
            else now
        )
        self.finish_wall_timestamp = datetime.now(timezone.utc).isoformat()
        if self.start_wall_monotonic is not None:
            self.wall_time_sec = round(time.monotonic() - self.start_wall_monotonic, 3)
        self.results.add_event(
            self.finish_t, "RUN_FINISH" if status == "FINISH" else f"RUN_{status}",
            category="run",
        )
        self.fail_unresolved_missions(now)

        driving_time = max(0.0, self.finish_t - self.start_t)

        mission_penalty = float(
            self.results.penalty_by_category.get("mission", 0.0)
        )
        lane_penalty = float(
            self.results.penalty_by_category.get("lane", 0.0)
        )
        other_penalty = (
            self.results.penalty_total
            - mission_penalty
            - lane_penalty
        )
        final_time = driving_time + self.results.penalty_total

        missions = self.mission_summary()

        finished_normally = status == "FINISH"

        card, score_extra = build_scorecard(
            route_completion_pct=self.last_progress,
            finished_normally=finished_normally,
            missions=missions,
            driving_time_sec=driving_time,
            time_limit_sec=self.time_limit_sec,
            reference_time_sec=self.reference_time_sec,
            lane_crossing_time_sec=self.lane_crossing_time_sec,
            collision_factor=self.collision_factor,
            full_exit_count=self.full_lane_exit_count,
            full_exit_multiplier=self.full_exit_multiplier,
            comfortable_time_sec=self.comfortable_time_sec,
            comfort_evaluated_time_sec=self.comfort_evaluated_time_sec,
            quality_weights=self.quality_weights,
            overspeed_time_sec=self.overspeed_time_sec,
            include_speed_in_safety=self.include_speed_in_safety,
        )

        route_stats = self.stats_or_none(
            self.route_center_distances
        )
        planner_stats = self.stats_or_none(
            self.planner_tracking_errors
        )
        planner_heading_stats = self.stats_or_none(
            self.planner_heading_errors
        )

        lane_ratio = (
            100.0 * self.lane_crossing_time_sec / driving_time
            if driving_time > 1e-6
            else 0.0
        )

        benchmark_scores = score_details(card, score_extra)

        diagnostics = {
            "route_completion_pct": (
                None
                if self.last_progress is None
                else round(self.last_progress, 3)
            ),
            "lane_crossing_time_sec": round(
                self.lane_crossing_time_sec, 3
            ),
            "lane_crossing_ratio_pct": round(lane_ratio, 3),
            "full_lane_exit_count": int(self.full_lane_exit_count),
            "emergency_stop_center_relocation_required": bool(
                self.emergency_stop_center_relocation_required
            ),
            "lane_violation_active_at_finish": bool(
                self.lane_penalty_state.violation_start_t is not None
            ),
            "lane_evaluator": {
                "geometry": "four VehiclePhysicsControl wheel positions against route corridor",
                "geometry_source": self.lane_geometry_source,
                "reference_sha256": self.lane_reference_sha256,
                "boundary_tolerance_m": self.lane_tolerance,
                "first_penalty_sec": self.lane_first_penalty,
                "continuous_penalty_sec": self.lane_cont_penalty,
                "continuous_interval_sec": self.lane_cont_interval,
                "last_sample": None if self.latest_lane_evaluation is None else {
                    "wheel_out_count": self.latest_lane_evaluation.wheel_out_count,
                    "outside_wheels": list(
                        self.latest_lane_evaluation.outside_wheels
                    ),
                    "allowed_lane_ids": [
                        list(lane) for lane in
                        self.latest_lane_evaluation.allowed_lane_ids
                    ],
                    "matched_route_index": (
                        self.latest_lane_evaluation.matched_route_index
                    ),
                    "matched_road_id": self.latest_lane_evaluation.matched_road_id,
                    "matched_section_id": (
                        self.latest_lane_evaluation.matched_section_id
                    ),
                    "matched_lane_id": self.latest_lane_evaluation.matched_lane_id,
                    "junction": self.latest_lane_evaluation.junction,
                    "RoadOption": self.latest_lane_evaluation.road_option,
                    "route_match_distance_m": (
                        self.latest_lane_evaluation.route_match_distance_m
                    ),
                    "continuous_violation_sec": (
                        self.lane_penalty_state.continuous_sec(self.finish_t)
                    ),
                    "wheel_position_frame": (
                        self.latest_lane_evaluation.wheel_position_frame
                    ),
                    "wheelbase_m": self.latest_lane_evaluation.wheelbase_m,
                    "front_track_m": self.latest_lane_evaluation.front_track_m,
                    "rear_track_m": self.latest_lane_evaluation.rear_track_m,
                },
                "emergency_stop_center_relocation_required": bool(
                    self.emergency_stop_center_relocation_required
                ),
            },
            "collision_count": len(self.collision_records),
            "collision_factor": round(self.collision_factor, 5),
            "collision_records": self.collision_records,
            "overspeed_time_sec": round(self.overspeed_time_sec, 3),
            "comfort_evaluated_time_sec": round(
                self.comfort_evaluated_time_sec, 3
            ),
            "comfortable_time_sec": round(
                self.comfortable_time_sec, 3
            ),

            # Route-center values are diagnostics only.
            "route_center_distance_rmse_m": (
                None if route_stats["rmse"] is None else round(route_stats["rmse"], 4)
            ),
            "route_center_distance_p95_m": (
                None if route_stats["p95"] is None else round(route_stats["p95"], 4)
            ),
            "route_center_distance_max_m": (
                None if route_stats["max"] is None else round(route_stats["max"], 4)
            ),

            # Optional controller diagnostics.
            "planner_tracking_rmse_m": (
                None if planner_stats["rmse"] is None else round(planner_stats["rmse"], 4)
            ),
            "planner_tracking_p95_m": (
                None if planner_stats["p95"] is None else round(planner_stats["p95"], 4)
            ),
            "planner_tracking_max_m": (
                None if planner_stats["max"] is None else round(planner_stats["max"], 4)
            ),
            "planner_heading_rmse_deg": (
                None
                if planner_heading_stats["rmse"] is None
                else round(planner_heading_stats["rmse"], 4)
            ),
        }

        summary = {
            "benchmark_version": "kcity-v4",
            "suite": self.suite,
            "status": status,
            "lap_time_sec": round(driving_time, 3),
            "wall_time_sec": self.wall_time_sec,
            "timestamps": {
                "start_sim_time_sec": self.start_t,
                "finish_sim_time_sec": self.finish_t,
                "start_wall_utc": self.start_wall_timestamp,
                "finish_wall_utc": self.finish_wall_timestamp,
            },
            "competition": {
                "driving_time_sec": round(driving_time, 3),
                "mission_penalty_sec": round(mission_penalty, 3),
                "lane_penalty_sec": round(lane_penalty, 3),
                "other_penalty_sec": round(other_penalty, 3),
                "penalty_total_sec": round(
                    self.results.penalty_total, 3
                ),
                "final_time_sec": round(final_time, 3),
            },
            "benchmark_scores": benchmark_scores,
            "quality_composition": {
                "quality": card.quality,
                "configured_weights": dict(self.quality_weights),
                "active_weights": dict(card.active_quality_weights),
            },
            **self.route_metadata,
            "diagnostics": diagnostics,
            "missions": missions,
        }

        self.results.finalize(summary)
        self.final_scores = summary["benchmark_scores"]
        self.final_record_sec = round(final_time, 3)
        self.publish_status()

        self.get_logger().info("=" * 65)
        self.get_logger().info(
            f"FINAL COMPETITION RECORD : {final_time:.2f} s"
        )
        self.get_logger().info(
            f"TOTAL BENCHMARK SCORE    : {benchmark_scores['total']}"
        )
        self.get_logger().info(
            f"  Completion : {benchmark_scores['completion']}"
        )
        self.get_logger().info(
            f"  Safety     : {benchmark_scores['safety']}"
        )
        self.get_logger().info(
            f"  Mission    : {benchmark_scores['mission']}"
        )
        self.get_logger().info(
            f"  Efficiency : {benchmark_scores['efficiency']}"
        )
        self.get_logger().info(
            f"  Comfort    : {benchmark_scores['comfort']}"
        )
        self.get_logger().info(
            f"  Quality    : {benchmark_scores['quality']}"
        )
        self.get_logger().info(
            f"Results: {self.results.run_dir}"
        )
        self.get_logger().info("=" * 65)

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    def publish_status(self):
        lap = self.lap_timer.lap_time_sec
        if self.started and not self.finished and self.latest_sim_time is not None:
            lap = max(0.0, self.latest_sim_time - self.start_t)
        elif self.finished and self.start_t is not None:
            lap = max(0.0, self.finish_t - self.start_t)
        payload = {
            "status": "finished" if self.finished else "running" if self.started else "armed",
            "run_status": self.final_status,
            "suite": self.suite,
            "lap_time_sec": None if lap is None else round(lap, 3),
            "completion": None if not self.started or self.last_progress is None else round(self.last_progress, 3),
            "accumulated_penalty_sec": round(self.results.penalty_total, 3),
            "mission_penalty_sec": round(
                self.results.penalty_by_category.get("mission", 0.0), 3
            ),
            "lane_penalty_sec": round(
                self.results.penalty_by_category.get("lane", 0.0), 3
            ),
            "total_penalty_sec": round(self.results.penalty_total, 3),
            "emergency_stop_center_relocation_required": bool(
                self.emergency_stop_center_relocation_required
            ),
            "lane_diagnostics": (
                None if self.latest_lane_evaluation is None else
                self.latest_lane_evaluation.detail(
                    self.lane_penalty_state.continuous_sec(
                        self.latest_sim_time or 0.0
                    )
                )
            ),
            "current_mission": self.current_mission,
            "latest_mission_result": self.latest_mission_result,
            "final_record_sec": self.final_record_sec,
            "scores": self.final_scores,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(msg)

    def on_timer(self):
        if self.finished:
            return

        try:
            if not self.ensure_ego():
                return

            self.ensure_lights()

            snapshot = self.world.get_snapshot()
            now = float(snapshot.timestamp.elapsed_seconds)
            self.latest_sim_time = now

            transform = self.ego.get_transform()
            loc = transform.location
            speed = speed_mps(self.ego)

            if self.prev_sample_t is None:
                dt = 0.0
            else:
                dt = max(0.0, now - self.prev_sample_t)

            # Route progress / center distance.
            progress, route_center_distance, _ = self.route_tracker.update(
                loc.x, loc.y
            )
            self.last_progress = progress

            # Optional planner-reference tracking diagnostics.
            (
                _,
                planner_tracking_error,
                planner_ref_heading,
            ) = self.planner_tracker.update(loc.x, loc.y)

            planner_heading_error = None
            if planner_ref_heading is not None:
                planner_heading_error = wrap_deg(
                    transform.rotation.yaw - planner_ref_heading
                )

            start_loc = evaluation_location(self.ego, self.start_trigger)
            finish_loc = evaluation_location(self.ego, self.finish_trigger)
            crossing = self.lap_timer.observe(now, start_loc, finish_loc)
            if not self.started:
                if crossing != "START":
                    self.prev_sample_t = now
                    self.pending_collisions.clear()
                    return

                self.started = True
                self.start_t = self.lap_timer.start_sim_time_sec
                self.start_wall_monotonic = time.monotonic()
                self.start_wall_timestamp = datetime.now(timezone.utc).isoformat()
                self.route_tracker.max_progress_m = 0.0
                progress, route_center_distance, _ = self.route_tracker.update(loc.x, loc.y)
                self.last_progress = progress
                dt = max(0.0, now - self.start_t)
                self.results.add_event(
                    self.start_t,
                    "RUN_START",
                    category="run",
                )

            if route_center_distance is not None:
                self.route_center_distances.append(route_center_distance)
            if planner_tracking_error is not None:
                self.planner_tracking_errors.append(planner_tracking_error)
            if planner_heading_error is not None:
                self.planner_heading_errors.append(abs(planner_heading_error))

            # Evaluate actual legal corridor, not lane-center distance.
            lane_crossing, full_lane_exit = self.evaluate_lane(
                now, dt, transform
            )
            if self.finished:
                return

            # Safety collision scoring.
            self.consume_collisions(now)

            # Mission / official penalty evaluation.
            self.process_missions(now, loc, speed)
            self.current_mission = next(
                (mission_id for mission_id, runtime in self.missions.items()
                 if runtime.status == "PENDING" and runtime.trigger.contains(loc)),
                None,
            )

            # Dynamics / comfort.
            (
                long_accel,
                lat_accel,
                jerk,
                long_jerk,
                yaw_rate,
                yaw_accel,
                comfortable,
            ) = self.evaluate_dynamics(now, dt, transform)

            overspeed = self.evaluate_speed(dt, speed)

            self.results.log_raw(
                sim_time=now,
                transform=transform,
                speed=speed,
                route_progress=progress,
                route_center_distance=route_center_distance,
                planner_tracking_error=planner_tracking_error,
                planner_heading_error=planner_heading_error,
                lane_crossing=lane_crossing,
                full_lane_exit=full_lane_exit,
                long_accel=long_accel,
                lat_accel=lat_accel,
                jerk=jerk,
                long_jerk=long_jerk,
                yaw_rate=yaw_rate,
                yaw_accel=yaw_accel,
                comfortable=comfortable,
                overspeed=overspeed,
            )

            self.pending_collisions.clear()
            self.prev_sample_t = now

            elapsed = now - self.start_t

            if elapsed >= self.time_limit_sec:
                self.finish_run(now, "TIMEOUT")
                return

            if crossing == "FINISH" or (
                not self.finish_trigger.enabled and self.should_finish(loc)
            ):
                self.finish_run(now, "FINISH")

        except Exception as exc:
            self.get_logger().error(f"Benchmark error: {exc}")
            self.get_logger().debug(traceback.format_exc())

    def destroy_node(self):
        try:
            if self.collision_sensor is not None:
                if self.collision_sensor.is_listening:
                    self.collision_sensor.stop()
                if self.collision_sensor.is_alive:
                    self.collision_sensor.destroy()
        except Exception:
            pass

        try:
            self.results.close()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionBenchmark()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.started and not node.finished:
            now = float(
                node.world.get_snapshot().timestamp.elapsed_seconds
            )
            node.finish_run(now, "INTERRUPTED")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
