#!/usr/bin/env python3
"""
K-City closed-loop scoring model (v4).

Two independent outputs are produced:

A. Competition-style record
   final_time = driving_time + official-style penalty time

B. Benchmark score (0~100)
   detailed:
     - Completion
     - Safety
     - Mission
     - Efficiency
     - Comfort

   total:
     Total = CompletionFactor × QualityScore

     QualityScore = weighted mean of
       Safety / Mission / Efficiency / Comfort

Why gate by Completion?
-----------------------
A vehicle that is very safe/comfortable but only completes half the route
should not receive a high overall score. This follows the same broad idea as
closed-loop driving benchmarks that combine route completion with infraction
quality, without copying any benchmark's exact official formula.

Important:
- Lane-center CTE is NOT scored.
- A legal out-in-out trajectory is therefore not punished just for being away
  from the lane center.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional


def clamp(x: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


@dataclass
class ScoreCard:
    completion: Optional[float]
    safety: Optional[float]
    mission: Optional[float]
    efficiency: Optional[float]
    comfort: Optional[float]
    quality: Optional[float]
    total: Optional[float]
    active_quality_weights: Dict[str, float]


def completion_score(
    route_completion_pct: Optional[float],
    finished_normally: bool,
) -> float:
    if route_completion_pct is None:
        return 100.0 if finished_normally else 0.0
    return 100.0 * clamp(float(route_completion_pct) / 100.0)


def mission_score(missions: Iterable[dict]):
    scored = [m for m in missions if bool(m.get("scored", False))]
    if not scored:
        return None, None

    total_weight = 0.0
    passed_weight = 0.0
    passed_count = 0

    for mission in scored:
        w = max(0.0, float(mission.get("score_weight", 1.0)))
        total_weight += w
        if mission.get("status") == "PASS":
            passed_weight += w
            passed_count += 1

    weighted = None
    if total_weight > 0.0:
        weighted = 100.0 * passed_weight / total_weight

    success_rate = 100.0 * passed_count / len(scored)
    return weighted, success_rate


def efficiency_score(
    driving_time_sec: float,
    time_limit_sec: float,
    reference_time_sec: Optional[float],
):
    """
    Fixed-course efficiency:
      drive <= reference -> 100
      drive == time_limit -> 0
      between -> linear

    If reference_time_sec is not calibrated yet, return None.
    """
    if reference_time_sec in (None, 0, 0.0):
        return None

    reference = float(reference_time_sec)
    limit = float(time_limit_sec)
    drive = float(driving_time_sec)

    if limit <= reference:
        return None

    value = (limit - drive) / (limit - reference)
    return 100.0 * clamp(value)


def safety_score(
    driving_time_sec: float,
    lane_crossing_time_sec: float,
    collision_factor: float,
    full_exit_count: int,
    full_exit_multiplier: float,
    overspeed_time_sec: float = 0.0,
    include_speed: bool = False,
):
    drive = max(float(driving_time_sec), 1e-6)

    lane_safe_ratio = clamp(
        1.0 - float(lane_crossing_time_sec) / drive
    )
    collision_factor = clamp(float(collision_factor))

    full_exit_factor = 1.0
    if int(full_exit_count) > 0:
        full_exit_factor = (
            clamp(float(full_exit_multiplier)) ** int(full_exit_count)
        )

    speed_factor = 1.0
    if include_speed:
        speed_factor = clamp(
            1.0 - float(overspeed_time_sec) / drive
        )

    total_factor = (
        lane_safe_ratio
        * collision_factor
        * full_exit_factor
        * speed_factor
    )

    return 100.0 * clamp(total_factor), {
        "lane_safe_ratio": lane_safe_ratio,
        "collision_factor": collision_factor,
        "full_exit_factor": full_exit_factor,
        "speed_factor": speed_factor,
    }


def comfort_score(
    comfortable_time_sec: float,
    evaluated_time_sec: float,
):
    if evaluated_time_sec <= 1e-9:
        return None

    return 100.0 * clamp(
        float(comfortable_time_sec) / float(evaluated_time_sec)
    )


def weighted_available(
    scores: Dict[str, Optional[float]],
    configured_weights: Dict[str, float],
):
    available = {
        name: float(score)
        for name, score in scores.items()
        if score is not None
        and float(configured_weights.get(name, 0.0)) > 0.0
    }

    if not available:
        return None, {}

    raw_sum = sum(
        float(configured_weights[name])
        for name in available
    )
    if raw_sum <= 0.0:
        return None, {}

    active = {
        name: float(configured_weights[name]) / raw_sum
        for name in available
    }

    value = sum(
        available[name] * active[name]
        for name in available
    )
    return value, active


def build_scorecard(
    *,
    route_completion_pct,
    finished_normally,
    missions,
    driving_time_sec,
    time_limit_sec,
    reference_time_sec,
    lane_crossing_time_sec,
    collision_factor,
    full_exit_count,
    full_exit_multiplier,
    comfortable_time_sec,
    comfort_evaluated_time_sec,
    quality_weights,
    overspeed_time_sec=0.0,
    include_speed_in_safety=False,
):
    completion = completion_score(
        route_completion_pct,
        finished_normally,
    )

    mission, success_rate = mission_score(missions)

    efficiency = efficiency_score(
        driving_time_sec,
        time_limit_sec,
        reference_time_sec,
    )

    safety, safety_details = safety_score(
        driving_time_sec=driving_time_sec,
        lane_crossing_time_sec=lane_crossing_time_sec,
        collision_factor=collision_factor,
        full_exit_count=full_exit_count,
        full_exit_multiplier=full_exit_multiplier,
        overspeed_time_sec=overspeed_time_sec,
        include_speed=include_speed_in_safety,
    )

    comfort = comfort_score(
        comfortable_time_sec,
        comfort_evaluated_time_sec,
    )

    quality, active = weighted_available(
        {
            "safety": safety,
            "mission": mission,
            "efficiency": efficiency,
            "comfort": comfort,
        },
        quality_weights,
    )

    total = None
    if quality is not None:
        total = (completion / 100.0) * quality

    return (
        ScoreCard(
            completion=completion,
            safety=safety,
            mission=mission,
            efficiency=efficiency,
            comfort=comfort,
            quality=quality,
            total=total,
            active_quality_weights=active,
        ),
        {
            "mission_success_rate_pct": success_rate,
            "safety_details": safety_details,
        },
    )
