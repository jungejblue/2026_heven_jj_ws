"""CARLA trigger geometry and the existing HEVEN traffic perception contract.

This module is intentionally independent of ROS, CARLA, and the scenario package.
The observer samples the actor's current state; it never changes a light phase.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class TrafficObservation:
    label: str = ""
    class_names: Tuple[str, ...] = ()
    confidence: float = 0.0
    detections: int = 0


NO_DETECTION = TrafficObservation()
_STATE_OBSERVATIONS = {
    "Red": TrafficObservation("1301", ("RED",), 1.0, 1),
    "Yellow": TrafficObservation("1302", ("ORANGE",), 1.0, 1),
    "Green": TrafficObservation("1300", ("GREEN",), 1.0, 1),
}


def observation_for_state(state_name: str, signal_type: str) -> TrafficObservation:
    semantic = str(signal_type).strip().lower()
    if semantic not in {"straight", "left"}:
        raise ValueError("signal_type must be straight or left")
    state = str(state_name).split(".")[-1]
    if semantic == "left" and state == "Green":
        return TrafficObservation("1305", ("LEFT",), 1.0, 1)
    return _STATE_OBSERVATIONS.get(state, NO_DETECTION)


def xyz(location) -> Tuple[float, float, float]:
    if isinstance(location, Mapping):
        return tuple(float(location[key]) for key in ("x", "y", "z"))
    if hasattr(location, "x"):
        return float(location.x), float(location.y), float(location.z)
    return tuple(float(value) for value in location)


@dataclass(frozen=True)
class TriggerBox:
    center: Tuple[float, float, float]
    extent: Tuple[float, float, float]
    yaw_deg: float
    enabled: bool = True
    exit_edge: str = "-y"

    @classmethod
    def from_config(cls, config: Mapping):
        box = cls(xyz(config["center"]), xyz(config["extent"]),
                  float(config.get("yaw_deg", 0.0)),
                  bool(config.get("enabled", False)),
                  str(config.get("exit_edge", "-y")))
        if not all(math.isfinite(v) for v in (*box.center, *box.extent, box.yaw_deg)):
            raise ValueError("trigger coordinates must be finite")
        if any(value <= 0.0 for value in box.extent):
            raise ValueError("trigger extents must be positive half lengths")
        if box.exit_edge not in {"+x", "-x", "+y", "-y"}:
            raise ValueError("exit_edge must be +x, -x, +y, or -y")
        return box

    def local_coords(self, location) -> Tuple[float, float, float]:
        dx, dy, dz = (value - origin for value, origin in zip(xyz(location), self.center))
        yaw = math.radians(self.yaw_deg)
        return (dx * math.cos(yaw) + dy * math.sin(yaw),
                -dx * math.sin(yaw) + dy * math.cos(yaw), dz)

    def contains(self, location) -> bool:
        return self.enabled and all(
            abs(value) <= extent + 1e-9
            for value, extent in zip(self.local_coords(location), self.extent)
        )

    def exit_line(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Return the configured exit edge in CARLA world coordinates for drawing."""
        ex, ey, _ = self.extent
        if self.exit_edge == "+x":
            local_points = ((ex, -ey), (ex, ey))
        elif self.exit_edge == "-x":
            local_points = ((-ex, -ey), (-ex, ey))
        elif self.exit_edge == "+y":
            local_points = ((-ex, ey), (ex, ey))
        else:
            local_points = ((-ex, -ey), (ex, -ey))
        yaw = math.radians(self.yaw_deg)
        cx, cy, cz = self.center
        return tuple((cx + x * math.cos(yaw) - y * math.sin(yaw),
                      cy + x * math.sin(yaw) + y * math.cos(yaw), cz + 0.2)
                     for x, y in local_points)


@dataclass
class TrafficSignal:
    key: str
    config: Mapping
    trigger: TriggerBox
    signal_type: str
    actor: object = None


def signals_from_config(config: Mapping) -> list:
    """Read the existing qualifier YAML schema, including a full scenario YAML."""
    lights = config.get("scenario", {}).get("qualifier", {}).get("traffic_lights", {})
    if not isinstance(lights, Mapping) or not lights:
        raise ValueError("scenario.qualifier.traffic_lights must contain actor bindings")
    signals = []
    for key, light in lights.items():
        signal_type = str(light.get("signal_type", "")).strip().lower()
        observation_for_state("Unknown", signal_type)
        trigger_key = str(light["trigger_key"])
        signals.append(TrafficSignal(str(key), light,
                                    TriggerBox.from_config(config["triggers"][trigger_key]),
                                    signal_type))
    return signals


def resolve_light(actors: Sequence, config: Mapping):
    """Match an explicit actor ID or one unambiguous actor within the configured radius."""
    actor_id = config.get("actor_id")
    if actor_id not in (None, "", 0, "0"):
        actor_id = int(actor_id)
        matches = [actor for actor in actors if int(actor.id) == actor_id]
        if len(matches) != 1:
            raise ValueError(f"traffic light actor_id={actor_id} not found")
        return matches[0]
    target = xyz(config["location"])
    radius = float(config.get("match_radius_m", 5.0))
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("match_radius_m must be positive")
    candidates = [actor for actor in actors
                  if math.dist(xyz(actor.get_location()), target) <= radius]
    if len(candidates) != 1:
        raise ValueError(f"expected one traffic light within {radius:g} m; found {len(candidates)}")
    return candidates[0]


def sample_signals(signals: Sequence[TrafficSignal], ego_location) -> Tuple[TrafficObservation, Optional[str]]:
    if ego_location is None:
        return NO_DETECTION, None
    active = [signal for signal in signals if signal.trigger.contains(ego_location)]
    if len(active) != 1:
        return NO_DETECTION, None
    signal = active[0]
    if signal.actor is None or not getattr(signal.actor, "is_alive", False):
        return NO_DETECTION, signal.key
    return observation_for_state(signal.actor.get_state(), signal.signal_type), signal.key


def simulation_stamp(elapsed_seconds: float) -> Tuple[int, int]:
    """Convert CARLA snapshot elapsed_seconds to the same epoch as /clock."""
    elapsed = float(elapsed_seconds)
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError("simulation time must be finite and nonnegative")
    nanoseconds = int(round(elapsed * 1_000_000_000))
    return divmod(nanoseconds, 1_000_000_000)
