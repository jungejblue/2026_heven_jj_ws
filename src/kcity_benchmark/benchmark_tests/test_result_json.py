import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from kcity_benchmark.result_manager import ResultManager
from kcity_benchmark.scoring import build_scorecard
from kcity_benchmark.score_schema import score_details


class ResultJsonTests(unittest.TestCase):
    def test_efficiency_unavailable_keeps_quality_and_renormalizes(self):
        card, _ = build_scorecard(
            route_completion_pct=100.0, finished_normally=True,
            missions=[{"scored": True, "score_weight": 1, "status": "PASS"}],
            driving_time_sec=10, time_limit_sec=100, reference_time_sec=None,
            lane_crossing_time_sec=0, collision_factor=1, full_exit_count=0,
            full_exit_multiplier=0.5, comfortable_time_sec=10,
            comfort_evaluated_time_sec=10,
            quality_weights={"safety": 0.4, "mission": 0.35,
                             "efficiency": 0.15, "comfort": 0.1},
        )
        self.assertIsNone(card.efficiency)
        self.assertEqual(card.quality, 100.0)
        self.assertEqual(card.total, 100.0)
        self.assertAlmostEqual(sum(card.active_quality_weights.values()), 1.0)
        self.assertNotIn("efficiency", card.active_quality_weights)
        details = score_details(card, {
            "mission_success_rate_pct": 100.0,
            "safety_details": {"lane_safe_ratio": 1.0},
        })
        self.assertIsNone(details["efficiency"])
        self.assertEqual(details["quality"], 100.0)
        self.assertEqual(details["total"], 100.0)

    def test_result_files_have_flat_scores_and_quality(self):
        with TemporaryDirectory() as directory:
            manager = ResultManager("qualifier", directory)
            manager.finalize({
                "suite": "qualifier", "status": "FINISH",
                "competition": {"driving_time_sec": 10, "final_time_sec": 10},
                "benchmark_scores": {
                    "completion": 100.0, "safety": 100.0, "mission": 75.0,
                    "efficiency": None, "comfort": 99.0, "quality": 89.0,
                    "total": 89.0, "active_quality_weights": {"safety": 0.5},
                },
                "route_source": "route_csv",
                "route_csv": "routes/qualifier.csv",
                "route_point_count": 42,
                "route_length_m": 123.4,
                "route_sha256": "abc123",
                "diagnostics": {}, "missions": [],
            })
            manager.close()
            run_dir = Path(manager.run_dir)
            for filename in ("scores.json", "result.json", "events.csv", "raw.csv", "summary.txt"):
                self.assertTrue((run_dir / filename).exists())
            scores = json.loads((run_dir / "scores.json").read_text())
            result = json.loads((run_dir / "result.json").read_text())
            self.assertIsNone(scores["efficiency"])
            self.assertEqual(scores["quality"], 89.0)
            self.assertEqual(scores["lap_time_sec"], 10.0)
            self.assertTrue(all(not isinstance(v, (dict, list)) for v in scores.values()))
            self.assertEqual(result["benchmark_scores"]["quality"], 89.0)
            self.assertEqual(result["route_source"], "route_csv")
            self.assertEqual(result["route_csv"], "routes/qualifier.csv")
            self.assertEqual(result["route_point_count"], 42)
            self.assertEqual(result["route_length_m"], 123.4)
            self.assertEqual(result["route_sha256"], "abc123")
            self.assertIn("Quality             : 89.0", (run_dir / "summary.txt").read_text())
