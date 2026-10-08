#!/usr/bin/env python3
from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Dict, List


class ResultManager:
    def __init__(self, suite: str, output_root: str):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(output_root).expanduser() / f"{suite}_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.events_path = self.run_dir / "events.csv"
        self.raw_path = self.run_dir / "raw.csv"
        self.scorecard_path = self.run_dir / "scorecard.csv"

        self.events_fp = self.events_path.open("w", newline="", encoding="utf-8")
        self.events_writer = csv.writer(self.events_fp)
        self.events_writer.writerow(
            [
                "sim_time",
                "category",
                "event",
                "mission_id",
                "detail",
                "penalty_sec",
                "penalty_total_sec",
            ]
        )

        self.raw_fp = self.raw_path.open("w", newline="", encoding="utf-8")
        self.raw_writer = csv.writer(self.raw_fp)
        self.raw_writer.writerow(
            [
                "sim_time",
                "x",
                "y",
                "z",
                "yaw_deg",
                "speed_mps",
                "route_progress_pct",
                "route_center_distance_m",
                "planner_tracking_error_m",
                "planner_heading_error_deg",
                "lane_crossing",
                "full_lane_exit",
                "long_accel_mps2",
                "lat_accel_mps2",
                "jerk_mps3",
                "long_jerk_mps3",
                "yaw_rate_radps",
                "yaw_accel_radps2",
                "comfortable",
                "overspeed",
                "penalty_total_sec",
            ]
        )

        self.penalty_total = 0.0
        self.penalty_by_category = {}
        self.events: List[Dict] = []

    def add_event(
        self,
        sim_time: float,
        event: str,
        mission_id: str = "",
        detail: str = "",
        penalty_sec: float = 0.0,
        category: str = "other",
    ):
        penalty_sec = float(penalty_sec)
        self.penalty_total += penalty_sec
        self.penalty_by_category[category] = (
            self.penalty_by_category.get(category, 0.0) + penalty_sec
        )

        record = {
            "sim_time": float(sim_time),
            "category": category,
            "event": event,
            "mission_id": mission_id,
            "detail": detail,
            "penalty_sec": penalty_sec,
            "penalty_total_sec": self.penalty_total,
        }
        self.events.append(record)

        self.events_writer.writerow(
            [
                f"{sim_time:.3f}",
                category,
                event,
                mission_id,
                detail,
                f"{penalty_sec:.3f}",
                f"{self.penalty_total:.3f}",
            ]
        )
        self.events_fp.flush()

    def log_raw(
        self,
        *,
        sim_time,
        transform,
        speed,
        route_progress,
        route_center_distance,
        planner_tracking_error,
        planner_heading_error,
        lane_crossing,
        full_lane_exit,
        long_accel,
        lat_accel,
        jerk,
        long_jerk,
        yaw_rate,
        yaw_accel,
        comfortable,
        overspeed,
    ):
        loc = transform.location

        def fmt(value, digits=4):
            if value is None:
                return ""
            return f"{float(value):.{digits}f}"

        self.raw_writer.writerow(
            [
                f"{sim_time:.3f}",
                fmt(loc.x),
                fmt(loc.y),
                fmt(loc.z),
                fmt(transform.rotation.yaw),
                fmt(speed),
                fmt(route_progress),
                fmt(route_center_distance),
                fmt(planner_tracking_error),
                fmt(planner_heading_error),
                int(bool(lane_crossing)),
                int(bool(full_lane_exit)),
                fmt(long_accel),
                fmt(lat_accel),
                fmt(jerk),
                fmt(long_jerk),
                fmt(yaw_rate),
                fmt(yaw_accel),
                "" if comfortable is None else int(bool(comfortable)),
                int(bool(overspeed)),
                fmt(self.penalty_total, 3),
            ]
        )
        self.raw_fp.flush()

    def finalize(self, summary: Dict):
        summary = dict(summary)
        summary["penalty_total_sec"] = self.penalty_total
        summary["penalty_by_category_sec"] = self.penalty_by_category
        summary["events"] = self.events

        score = summary.get("benchmark_scores", {})
        competition = summary.get("competition", {})
        flat_scores = {
            name: score.get(name)
            for name in ("completion", "safety", "mission", "efficiency",
                         "comfort", "quality", "total")
        }
        flat_scores["lap_time_sec"] = summary.get(
            "lap_time_sec", competition.get("driving_time_sec"))
        summary["scores"] = flat_scores
        summary["final_record_sec"] = competition.get("final_time_sec")
        summary["penalties"] = {
            "total_sec": self.penalty_total,
            "by_category_sec": dict(self.penalty_by_category),
        }

        def json_safe(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {str(k): json_safe(v) for k, v in value.items()}
            if isinstance(value, (list, tuple)):
                return [json_safe(v) for v in value]
            return value

        summary = json_safe(summary)
        flat_scores = json_safe(flat_scores)
        (self.run_dir / "scores.json").write_text(
            json.dumps(flat_scores, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        (self.run_dir / "result.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

        (self.run_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

        score = summary.get("benchmark_scores", {})
        comp = summary.get("competition", {})
        diagnostics = summary.get("diagnostics", {})

        # Flat scorecard.csv for quick comparison in Excel / pandas.
        with self.scorecard_path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(["metric", "value", "unit"])
            rows = [
                ("final_competition_time", comp.get("final_time_sec"), "s"),
                ("driving_time", comp.get("driving_time_sec"), "s"),
                ("official_penalty_total", comp.get("penalty_total_sec"), "s"),
                ("benchmark_quality_score", score.get("quality"), "point"),
                ("benchmark_total_score", score.get("total"), "point"),
                ("completion_score", score.get("completion"), "point"),
                ("safety_score", score.get("safety"), "point"),
                ("mission_score", score.get("mission"), "point"),
                ("efficiency_score", score.get("efficiency"), "point"),
                ("comfort_score", score.get("comfort"), "point"),
                ("mission_success_rate", score.get("mission_success_rate_pct"), "%"),
                ("route_completion", diagnostics.get("route_completion_pct"), "%"),
                ("lane_crossing_time", diagnostics.get("lane_crossing_time_sec"), "s"),
                ("lane_crossing_ratio", diagnostics.get("lane_crossing_ratio_pct"), "%"),
                ("collision_count", diagnostics.get("collision_count"), "count"),
                ("full_lane_exit_count", diagnostics.get("full_lane_exit_count"), "count"),
                ("route_center_distance_rmse", diagnostics.get("route_center_distance_rmse_m"), "m"),
                ("route_center_distance_p95", diagnostics.get("route_center_distance_p95_m"), "m"),
                ("route_center_distance_max", diagnostics.get("route_center_distance_max_m"), "m"),
                ("planner_tracking_rmse", diagnostics.get("planner_tracking_rmse_m"), "m"),
                ("planner_tracking_p95", diagnostics.get("planner_tracking_p95_m"), "m"),
                ("planner_tracking_max", diagnostics.get("planner_tracking_max_m"), "m"),
                ("planner_heading_rmse", diagnostics.get("planner_heading_rmse_deg"), "deg"),
            ]
            for row in rows:
                writer.writerow(row)

        lines = [
            "============================================================",
            f"K-City Benchmark: {summary.get('suite')}",
            "============================================================",
            "",
            "[Competition record]",
            f"Driving time        : {comp.get('driving_time_sec')} s",
            f"Mission penalty     : {comp.get('mission_penalty_sec')} s",
            f"Lane penalty        : {comp.get('lane_penalty_sec')} s",
            f"Other penalty       : {comp.get('other_penalty_sec')} s",
            f"Penalty total       : {comp.get('penalty_total_sec')} s",
            f"FINAL RECORD        : {comp.get('final_time_sec')} s",
            "",
            "[Benchmark score: 0-100]",
            f"Completion          : {score.get('completion')}",
            f"Safety              : {score.get('safety')}",
            f"Mission             : {score.get('mission')}",
            f"Efficiency          : {score.get('efficiency')}",
            f"Comfort             : {score.get('comfort')}",
            f"Quality             : {score.get('quality')}",
            f"TOTAL               : {score.get('total')}",
            f"Mission success     : {score.get('mission_success_rate_pct')} %",
            f"Active quality w.   : {score.get('active_quality_weights')}",
            "",
            "[Diagnostics - NOT directly scored]",
            f"Route completion    : {diagnostics.get('route_completion_pct')} %",
            f"Lane crossing time  : {diagnostics.get('lane_crossing_time_sec')} s",
            f"Lane crossing ratio : {diagnostics.get('lane_crossing_ratio_pct')} %",
            f"Collision count     : {diagnostics.get('collision_count')}",
            f"Full lane exits     : {diagnostics.get('full_lane_exit_count')}",
            f"Route-center RMSE   : {diagnostics.get('route_center_distance_rmse_m')} m",
            f"Planner-track RMSE  : {diagnostics.get('planner_tracking_rmse_m')} m",
            f"Planner-head RMSE   : {diagnostics.get('planner_heading_rmse_deg')} deg",
            "",
            "[Mission results]",
        ]

        for mission in summary.get("missions", []):
            lines.append(
                f"- {mission['id']} {mission['name']}: {mission['status']} "
                f"(official penalty={mission.get('applied_penalty_sec', 0)} s, "
                f"scored={mission.get('scored', False)})"
            )

        (self.run_dir / "summary.txt").write_text(
            "\n".join(lines) + "\n",
            encoding="utf-8",
        )

    def close(self):
        try:
            self.events_fp.close()
        except Exception:
            pass
        try:
            self.raw_fp.close()
        except Exception:
            pass
