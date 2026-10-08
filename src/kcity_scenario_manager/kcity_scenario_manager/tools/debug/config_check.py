#!/usr/bin/env python3
"""
Pre-run configuration checker.

It does not modify CARLA. It reports:
- disabled/missing triggers
- zero/placeholder trigger geometry
- unresolved traffic-light coordinates
- missing route_points
- uncalibrated efficiency reference time
- obstacle spawn placeholders
"""

from pathlib import Path
import math

import rclpy
from rclpy.node import Node

from ...competition_common import load_yaml, suite_traffic_light_configs
from ...route_csv import read_route_csv, resolve_route_csv_path


class ConfigCheck(Node):
    def __init__(self):
        super().__init__("competition_config_check")

        self.declare_parameter("config_file", "")
        path = str(self.get_parameter("config_file").value)

        if not path:
            raise RuntimeError("config_file parameter is required.")

        cfg = load_yaml(path)
        self.check(cfg, path)

        # One-shot node: main destroys the node and shuts down rclpy.

    def warn(self, text):
        self.get_logger().warning(text)

    def ok(self, text):
        self.get_logger().info(text)

    @staticmethod
    def all_zero_xyz(data):
        return all(
            abs(float(data.get(k, 0.0))) < 1e-9
            for k in ("x", "y", "z")
        )

    def check(self, cfg, path):
        self.ok(f"Checking: {path}")

        triggers = cfg.get("triggers", {})
        benchmark = cfg.get("benchmark", {})
        missions = benchmark.get("missions", [])

        required_trigger_keys = {
            str(benchmark.get("start_trigger_key", "START")),
            str(benchmark.get("finish_trigger_key", "FINISH")),
        }
        for mission in missions:
            required_trigger_keys.add(str(mission.get("trigger_key", "")))

        for key in sorted(required_trigger_keys):
            if not key:
                continue
            if key not in triggers:
                self.warn(f"[TRIGGER] missing: {key}")
                continue

            t = triggers[key]
            if not bool(t.get("enabled", False)):
                self.warn(f"[TRIGGER] disabled: {key}")

            center = t.get("center", {})
            extent = t.get("extent", {})

            if self.all_zero_xyz(center):
                self.warn(f"[TRIGGER] center still placeholder: {key}")

            if any(float(extent.get(k, 0.0)) <= 0.0 for k in ("x", "y", "z")):
                self.warn(f"[TRIGGER] invalid extent: {key} -> {extent}")

            exit_edge=str(t.get("exit_edge", ""))
            if not exit_edge:
                self.warn(
                    f"[TRIGGER] exit_edge is required for crossing evaluation: {key}"
                )
            elif exit_edge not in {"+x", "-x", "+y", "-y"}:
                self.warn(f"[TRIGGER] invalid exit_edge: {key} -> {exit_edge}")

        route_csv=str(benchmark.get('route_csv','')).strip()
        route=benchmark.get("route_points", [])
        if route_csv:
            route_path=resolve_route_csv_path(path,route_csv)
            if not route_path.exists():
                self.warn(f'[ROUTE] route_csv not found: {route_path}')
            else:
                try:
                    rows=read_route_csv(route_path)
                except (OSError,RuntimeError,ValueError) as exc:
                    self.warn(f'[ROUTE] invalid route_csv: {exc}')
                else:
                    self.ok(
                        f'[ROUTE] route_csv={route_path}, points={len(rows)}, '
                        f'length={rows[-1]["route_s_m"]:.3f}m'
                    )
        elif len(route) < 2:
            self.warn("[ROUTE] route_points not populated; route completion will be limited.")

        scoring = benchmark.get("scoring", {})
        ref = scoring.get("efficiency", {}).get("reference_time_sec")
        if ref in (None, 0, 0.0):
            self.warn("[SCORING] efficiency reference_time_sec is not calibrated; Efficiency will be N/A.")
        else:
            self.ok(f"[SCORING] reference_time_sec={ref}")

        suite = benchmark.get("suite", "")
        try:
            lights=suite_traffic_light_configs(cfg,suite)
        except (TypeError,ValueError) as exc:
            self.warn(f"[LIGHT] invalid traffic_lights config: {exc}")
            lights={}

        for key,light in lights.items():
            if not light:
                continue
            loc = light.get("location", {})
            if self.all_zero_xyz(loc):
                self.warn(f"[LIGHT] location still placeholder: {key}")
            if light.get("actor_id") not in (None, "", 0, "0"):
                self.warn(
                    f"[LIGHT] {key} uses actor_id={light.get('actor_id')}; "
                    "runtime IDs can change. Location matching is safer."
                )

        for mission in missions:
            if str(mission.get('type',''))!='traffic_light':
                continue
            light_key=str(mission.get('light_key',''))
            if light_key not in lights:
                self.warn(
                    f"[MISSION] {mission.get('id','?')} references unknown "
                    f"light_key={light_key!r}"
                )

        if suite == "final":
            for i, obs in enumerate(
                cfg.get("scenario", {}).get("final", {}).get("obstacles", [])
            ):
                if bool(obs.get("enabled", False)):
                    if not str(obs.get("blueprint", "")).strip():
                        self.warn(f"[OBSTACLE] #{i} enabled but blueprint is empty.")
                    loc = obs.get("transform", {}).get("location", {})
                    if self.all_zero_xyz(loc):
                        self.warn(f"[OBSTACLE] #{i} location still placeholder.")

        self.ok("Config check finished.")


def main(args=None):
    rclpy.init(args=args)
    node = ConfigCheck()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
