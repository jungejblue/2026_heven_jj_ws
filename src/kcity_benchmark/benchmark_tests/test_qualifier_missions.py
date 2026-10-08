from pathlib import Path
import csv
import tempfile
import unittest

import carla

from kcity_benchmark.benchmark_runner import (
    CompetitionBenchmark,
    MissionRuntime,
    evaluate_traffic_light_mission,
    update_stop_progress,
)
from kcity_benchmark.common import TriggerBox, load_yaml, suite_traffic_light_configs
from kcity_benchmark.result_manager import ResultManager


class FakeLight:
    def __init__(self,state):
        self.state=state

    def get_state(self):
        return self.state


class FakeEgo:
    def __init__(self):
        self.location = carla.Location(x=-2.0, y=0.0, z=0.0)

    def get_location(self):
        return self.location


class QualifierMissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path=(
            Path(__file__).resolve().parents[2]
            / 'kcity_scenario_manager/config/qualifier.yaml'
        )
        cls.cfg=load_yaml(cls.path)

    def test_qualifier_config_preserves_new_geometry_and_uses_route_csv(self):
        self.assertEqual(
            list(self.cfg['triggers']),
            ['START','FINISH','Q_STOP1','Q_SIGNAL1','Q_STOP2','Q_SIGNAL2'],
        )
        self.assertEqual(
            self.cfg['triggers']['Q_SIGNAL1']['center'],
            {'x':22.1804,'y':236.3491,'z':1.0},
        )
        self.assertEqual(
            self.cfg['triggers']['Q_SIGNAL2']['center'],
            {'x':58.0567,'y':297.7779,'z':1.0},
        )
        lights=suite_traffic_light_configs(self.cfg,'qualifier')
        self.assertEqual(lights['Q_TL1']['location'],{
            'x':36.4,'y':248.3,'z':0.0,
        })
        self.assertEqual(lights['Q_TL2']['location'],{
            'x':45.4,'y':309.5,'z':0.0,
        })
        self.assertEqual(
            self.cfg['benchmark']['route_csv'],'routes/qualifier.csv'
        )
        self.assertNotIn('route_points',self.cfg['benchmark'])

    def test_qualifier_uses_only_four_new_missions(self):
        missions=self.cfg['benchmark']['missions']
        self.assertEqual(
            [mission['id'] for mission in missions],
            ['Q_STOP1','Q_SIGNAL1','Q_STOP2','Q_SIGNAL2'],
        )
        self.assertFalse(
            any(mission['id'].startswith('Q0') for mission in missions)
        )
        by_id={mission['id']:mission for mission in missions}
        self.assertEqual(by_id['Q_STOP1']['required_stop_sec'],3.0)
        self.assertEqual(by_id['Q_STOP2']['required_stop_sec'],3.0)
        self.assertEqual(by_id['Q_SIGNAL1']['light_key'],'Q_TL1')
        self.assertEqual(by_id['Q_SIGNAL2']['light_key'],'Q_TL2')

    def test_stop_progress_requires_best_continuous_three_seconds(self):
        runtime=MissionRuntime(
            cfg={
                'required_stop_sec':3.0,
                'stop_speed_threshold_mps':0.1,
            },
            trigger=object(),
        )
        update_stop_progress(runtime,10.0,True,0.0)
        update_stop_progress(runtime,12.9,True,0.0)
        self.assertLess(runtime.best_continuous_stop_sec,3.0)
        update_stop_progress(runtime,13.0,True,0.0)
        self.assertEqual(runtime.best_continuous_stop_sec,3.0)
        update_stop_progress(runtime,13.1,True,0.2)
        update_stop_progress(runtime,20.0,True,0.0)
        update_stop_progress(runtime,21.0,True,0.0)
        self.assertEqual(runtime.best_continuous_stop_sec,3.0)

    def test_traffic_light_mission_uses_its_light_key(self):
        lights={
            'Q_TL1':FakeLight(carla.TrafficLightState.Red),
            'Q_TL2':FakeLight(carla.TrafficLightState.Green),
        }
        result=evaluate_traffic_light_mission(
            {'light_key':'Q_TL2','fail_states':['Red']},lights
        )
        self.assertEqual(result,(True,'Green'))
        lights['Q_TL2'].state=carla.TrafficLightState.Red
        self.assertEqual(
            evaluate_traffic_light_mission(
                {'light_key':'Q_TL2','fail_states':['Red']},lights
            ),
            (False,'Red'),
        )
        self.assertIsNone(
            evaluate_traffic_light_mission({'light_key':'missing'},lights)
        )

    def make_benchmark(self, mission_cfg, light=None, signal_type='straight'):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        benchmark = CompetitionBenchmark.__new__(CompetitionBenchmark)
        benchmark.results = ResultManager('qualifier', temporary.name)
        self.addCleanup(benchmark.results.close)
        benchmark.ego = FakeEgo()
        benchmark.pending_collisions = []
        benchmark.latest_mission_result = None
        benchmark.traffic_lights = {'Q_TL1': light} if light else {}
        benchmark.traffic_light_cfgs = {'Q_TL1': {'signal_type': signal_type}}
        trigger = TriggerBox(0, 0, 0, 1, 1, 1, 0, True, '+x', 'center')
        runtime = MissionRuntime(cfg=mission_cfg, trigger=trigger)
        benchmark.missions = {mission_cfg['id']: runtime}
        return benchmark, runtime

    def sample(self, benchmark, now, x, speed):
        benchmark.ego.location = carla.Location(x=x, y=0, z=0)
        benchmark.process_missions(now, benchmark.ego.location, speed)

    def csv_rows(self, benchmark):
        with benchmark.results.events_path.open(newline='', encoding='utf-8') as fp:
            return list(csv.DictReader(fp))

    def test_stop_logging_reset_thresholds_and_unchanged_exit_score(self):
        cfg = {'id': 'Q_STOP1', 'type': 'stop', 'trigger_key': 'Q_STOP1',
               'required_stop_sec': 3.0, 'stop_speed_threshold_mps': 0.1,
               'penalty_sec': 17.0}
        benchmark, runtime = self.make_benchmark(cfg)
        for now, x, speed in [(0, -2, 1), (1, 0, 0.1), (2, 0, 0),
                              (2.5, 0, 0.2), (3, 0, 0), (4, 0, 0),
                              (5, 0, 0), (6, 0, 0), (7, 1.2, 1)]:
            self.sample(benchmark, now, x, speed)
        rows = self.csv_rows(benchmark)
        events = [row['event'] for row in rows]
        self.assertEqual(events.count('STOP_STARTED'), 2)
        progress = [row for row in rows if row['event'] == 'STOP_PROGRESS']
        self.assertEqual(len(progress), 2)
        self.assertEqual([row['sim_time'] for row in progress], ['2.000', '5.000'])
        self.assertEqual(len([row for row in rows if row['event'] == 'STOP_REQUIREMENT_MET']), 1)
        self.assertEqual(runtime.best_continuous_stop_sec, 3.0)
        self.assertEqual(runtime.status, 'PASS')
        self.assertEqual(events[-1], 'MISSION_PASS')
        self.assertEqual(benchmark.results.penalty_total, 0.0)
        for row in rows:
            if row['event'].startswith('STOP_'):
                for field in ('speed=', 'continuous_stop_sec=', 'required_stop_sec='):
                    self.assertIn(field, row['detail'])
                self.assertEqual(row['penalty_sec'], '0.000')

        failed, failed_runtime = self.make_benchmark(cfg)
        for now, x, speed in [(0, -2, 1), (1, 0, 0), (2, 0, 0),
                              (2.1, 0, 1), (3, 0, 0), (4, 1.2, 1)]:
            self.sample(failed, now, x, speed)
        self.assertEqual(failed_runtime.status, 'FAIL')
        self.assertEqual(failed.results.penalty_total, 17.0)
        self.assertEqual(self.csv_rows(failed)[-1]['event'], 'MISSION_FAIL')

    def test_signal_enter_transition_and_exit_policy(self):
        cfg = {'id': 'Q_SIGNAL1', 'type': 'traffic_light',
               'trigger_key': 'Q_SIGNAL1', 'light_key': 'Q_TL1',
               'fail_states': ['Red'], 'penalty_sec': 19.0}
        for state, expected, penalty in [
            (carla.TrafficLightState.Red, 'FAIL', 19.0),
            (carla.TrafficLightState.Green, 'PASS', 0.0),
            (carla.TrafficLightState.Yellow, 'PASS', 0.0),
        ]:
            with self.subTest(state=state):
                light = FakeLight(state)
                benchmark, runtime = self.make_benchmark(cfg, light, 'left')
                self.sample(benchmark, 0, -2, 1)
                self.sample(benchmark, 1, 0, 1)
                self.sample(benchmark, 2, 1.2, 1)
                rows = self.csv_rows(benchmark)
                self.assertIn(f'signal={state.name}', rows[0]['detail'])
                self.assertIn('signal_type=left', rows[0]['detail'])
                self.assertEqual(runtime.status, expected)
                self.assertEqual(benchmark.results.penalty_total, penalty)
                self.assertEqual(rows[-1]['event'], f'MISSION_{expected}')

        light = FakeLight(carla.TrafficLightState.Red)
        benchmark, runtime = self.make_benchmark(cfg, light)
        self.sample(benchmark, 0, -2, 1)
        self.sample(benchmark, 1, 0, 0)
        light.state = carla.TrafficLightState.Green
        self.sample(benchmark, 2, 0, 0)
        self.sample(benchmark, 3, 1.2, 1)
        rows = self.csv_rows(benchmark)
        self.assertEqual([row['event'] for row in rows], [
            'MISSION_TRIGGER_ENTER', 'SIGNAL_STOP_STARTED',
            'SIGNAL_STATE_CHANGED', 'RED_STOP_SUCCESS', 'MISSION_PASS',
        ])
        self.assertIn('from_signal=Red', rows[2]['detail'])
        self.assertIn('to_signal=Green', rows[2]['detail'])
        self.assertEqual(runtime.status, 'PASS')
        self.assertTrue(all(row['penalty_sec'] == '0.000' for row in rows))


if __name__=='__main__':
    unittest.main()
