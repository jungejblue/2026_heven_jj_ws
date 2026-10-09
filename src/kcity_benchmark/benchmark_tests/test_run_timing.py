"""Exercise runner end-of-run decisions without advancing CARLA."""

from types import SimpleNamespace
import math
import unittest
from unittest.mock import Mock, patch

from kcity_benchmark.benchmark_runner import CompetitionBenchmark


def runner(sample_time, finish_time=None):
    location = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    transform = SimpleNamespace(location=location, rotation=SimpleNamespace(yaw=0.0))
    node = SimpleNamespace(
        finished=False, started=True, start_t=0.0, time_limit_sec=600.0,
        ensure_ego=lambda:True, ensure_lights=lambda:None,
        world=SimpleNamespace(get_snapshot=lambda:SimpleNamespace(
            timestamp=SimpleNamespace(elapsed_seconds=sample_time))),
        ego=SimpleNamespace(get_transform=lambda:transform),
        prev_sample_t=599.95,
        route_tracker=SimpleNamespace(update=lambda *_:(100.0, 0.0, None)),
        planner_tracker=SimpleNamespace(update=lambda *_:(None, None, None)),
        start_trigger=object(), finish_trigger=SimpleNamespace(enabled=True),
        lap_timer=SimpleNamespace(
            observe=lambda *_:'FINISH' if finish_time is not None else None,
            finish_sim_time_sec=finish_time),
        route_center_distances=[], planner_tracking_errors=[], planner_heading_errors=[],
        evaluate_lane=Mock(return_value=(False, False)), consume_collisions=Mock(),
        process_missions=Mock(), missions={},
        evaluate_dynamics=Mock(return_value=(0, 0, 0, 0, 0, 0, True)),
        evaluate_speed=Mock(return_value=False), results=SimpleNamespace(log_raw=Mock()),
        pending_collisions=[], finish_run=Mock(),
        get_logger=lambda:SimpleNamespace(error=Mock(), debug=Mock()),
    )
    return node, location


class RunTimingTests(unittest.TestCase):
    def observe(self, node, location):
        with patch('kcity_benchmark.benchmark_runner.evaluation_location',
                   return_value=location), \
                patch('kcity_benchmark.benchmark_runner.speed_mps', return_value=0.0):
            CompetitionBenchmark.on_timer(node)

    def test_finish_before_deadline_wins_and_clips_metric_interval(self):
        node, location = runner(600.01, finish_time=599.98)
        self.observe(node, location)
        node.finish_run.assert_called_once_with(599.98, 'FINISH')
        self.assertAlmostEqual(node.evaluate_lane.call_args.args[1], 0.03)
        self.assertAlmostEqual(node.evaluate_dynamics.call_args.args[1], 0.06)
        self.assertAlmostEqual(node.evaluate_dynamics.call_args.kwargs['evaluated_dt'], 0.03)
        node.consume_collisions.assert_called_once_with(599.98, end_time=599.98)
        # The pose still belongs to the observed snapshot, even when only part
        # of its sample interval contributes to duration metrics.
        self.assertEqual(node.results.log_raw.call_args.kwargs['sim_time'], 600.01)

    def test_finish_after_deadline_is_timeout_at_deadline(self):
        node, location = runner(600.03, finish_time=600.01)
        self.observe(node, location)
        node.finish_run.assert_called_once_with(600.0, 'TIMEOUT')
        self.assertAlmostEqual(node.evaluate_lane.call_args.args[1], 0.05)

    def test_timeout_does_not_include_snapshot_tail(self):
        node, location = runner(601.0)
        self.observe(node, location)
        node.finish_run.assert_called_once_with(600.0, 'TIMEOUT')
        self.assertAlmostEqual(node.evaluate_dynamics.call_args.args[1], 1.05)
        self.assertAlmostEqual(node.evaluate_dynamics.call_args.kwargs['evaluated_dt'], 0.05)
        self.assertAlmostEqual(node.evaluate_speed.call_args.args[0], 0.05)

    def test_dynamics_derivatives_use_snapshot_interval_while_duration_is_clipped(self):
        node = SimpleNamespace(
            ego=SimpleNamespace(
                get_acceleration=lambda:SimpleNamespace(x=4.0, y=0.0, z=0.0),
                get_angular_velocity=lambda:SimpleNamespace(z=2.0)),
            prev_accel_vec=SimpleNamespace(x=3.4, y=0.0, z=0.0),
            prev_long_accel=3.4, prev_yaw_rate=0.0,
            comfort_thresholds=dict(lat_accel_abs_max=10.0, long_accel_min=-10.0,
                long_accel_max=10.0, jerk_abs_max=15.0, long_jerk_abs_max=15.0,
                yaw_rate_abs_max=1.0, yaw_accel_abs_max=1.0),
            comfort_evaluated_time_sec=0.0, comfortable_time_sec=0.0,
        )
        transform = SimpleNamespace(
            get_forward_vector=lambda:SimpleNamespace(x=1.0, y=0.0, z=0.0),
            get_right_vector=lambda:SimpleNamespace(x=0.0, y=1.0, z=0.0),
        )
        dynamics = CompetitionBenchmark.evaluate_dynamics(
            node, 599.98, 0.06, transform, evaluated_dt=0.03
        )
        self.assertAlmostEqual(dynamics[2], 10.0)
        self.assertAlmostEqual(dynamics[3], 10.0)
        self.assertAlmostEqual(dynamics[5], math.radians(2.0) / 0.06)
        self.assertTrue(dynamics[-1])
        self.assertAlmostEqual(node.comfort_evaluated_time_sec, 0.03)
        self.assertAlmostEqual(node.comfortable_time_sec, 0.03)

    def test_collision_after_interpolated_finish_is_excluded(self):
        collision = dict(other_actor_id=1, other_type='vehicle.other', impulse_norm=100.0)
        node = SimpleNamespace(
            ego=SimpleNamespace(get_physics_control=lambda:SimpleNamespace(mass=1000.0)),
            start_t=0.0, pending_collisions=[dict(collision, sim_time=599.97),
                                           dict(collision, sim_time=600.0, other_actor_id=2)],
            last_collision_by_actor={}, collision_dedupe_sec=1.0,
            collision_severity=lambda _: 'contact', classify_actor=lambda _: 'vehicle',
            collision_actor_multipliers={'vehicle':0.6}, severity_weights={'contact':0.25},
            collision_factor=1.0, collision_records=[], results=SimpleNamespace(add_event=Mock()),
        )
        CompetitionBenchmark.consume_collisions(node, 599.98, end_time=599.98)
        self.assertEqual([item['sim_time'] for item in node.collision_records], [599.97])
        self.assertEqual(node.results.add_event.call_count, 1)


if __name__ == '__main__':
    unittest.main()
