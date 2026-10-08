from pathlib import Path
import json
from types import SimpleNamespace
import unittest

import carla

from kcity_scenario_manager.competition_common import (
    get_trigger,
    load_yaml,
    resolve_traffic_light,
    suite_traffic_light_configs,
)
from kcity_scenario_manager.qualifier_manager import (
    QualifierLightRuntime,
    QualifierScenarioManager,
)


class FakeActor:
    def __init__(self,actor_id,x,y=0.0,z=0.0):
        self.id=actor_id
        self._location=carla.Location(x=x,y=y,z=z)

    def get_location(self):
        return self._location


class FakeActors(list):
    def filter(self,_pattern):
        return self


class FakeWorld:
    def __init__(self,actors):
        self._actors=FakeActors(actors)

    def get_actors(self):
        return self._actors


class TrafficLightConfigTests(unittest.TestCase):
    def test_qualifier_dictionary_and_final_list_are_normalized(self):
        config_dir=(
            Path(__file__).resolve().parents[1]
            / 'config'
        )
        qualifier=load_yaml(config_dir/'qualifier.yaml')
        final=load_yaml(config_dir/'final.yaml')
        self.assertEqual(
            list(suite_traffic_light_configs(qualifier,'qualifier')),
            ['Q_TL1','Q_TL2'],
        )
        self.assertEqual(
            list(suite_traffic_light_configs(final,'final')),
            ['F01_TL','F02_TL','F03_TL','F05_TL'],
        )

    def test_two_qualifier_lights_resolve_independently(self):
        first=FakeActor(101,0.0)
        second=FakeActor(202,10.0)
        final_light=FakeActor(303,20.0)
        world=FakeWorld([first,second,final_light])
        configs={
            'Q_TL1':{
                'actor_id':None,
                'location':{'x':0.0,'y':0.0,'z':0.0},
                'match_radius_m':1.0,
            },
            'Q_TL2':{
                'actor_id':None,
                'location':{'x':10.0,'y':0.0,'z':0.0},
                'match_radius_m':1.0,
            },
        }
        resolved={
            key:resolve_traffic_light(world,cfg)
            for key,cfg in configs.items()
        }
        self.assertIs(resolved['Q_TL1'],first)
        self.assertIs(resolved['Q_TL2'],second)
        self.assertNotIn(final_light,resolved.values())

    def test_actor_id_has_priority_over_location(self):
        first=FakeActor(101,0.0)
        second=FakeActor(202,10.0)
        resolved=resolve_traffic_light(FakeWorld([first,second]),{
            'actor_id':202,
            'location':{'x':0.0,'y':0.0,'z':0.0},
            'match_radius_m':1.0,
        })
        self.assertIs(resolved,second)

    def test_location_match_rejects_zero_and_multiple_candidates(self):
        with self.assertRaisesRegex(RuntimeError,'no traffic light within'):
            resolve_traffic_light(FakeWorld([FakeActor(1,10.0)]),{
                'actor_id':None,
                'location':{'x':0.0,'y':0.0,'z':0.0},
                'match_radius_m':1.0,
            })
        with self.assertRaisesRegex(RuntimeError,'ambiguous traffic light match'):
            resolve_traffic_light(
                FakeWorld([FakeActor(1,-0.5),FakeActor(2,0.5)]),
                {
                    'actor_id':None,
                    'location':{'x':0.0,'y':0.0,'z':0.0},
                    'match_radius_m':1.0,
                },
            )

    def test_runtime_state_is_not_shared_between_lights(self):
        trigger=object()
        first=QualifierLightRuntime({},trigger)
        second=QualifierLightRuntime({},trigger)
        first.forced=True
        first.force_t=12.0
        first.released=True
        self.assertFalse(second.forced)
        self.assertIsNone(second.force_t)
        self.assertFalse(second.released)

    def test_missing_exit_edge_never_silently_defaults_to_positive_x(self):
        cfg={'triggers':{'SIGNAL':{
            'enabled':True,
            'center':{'x':0,'y':0,'z':0},
            'extent':{'x':1,'y':1,'z':1},
        }}}
        with self.assertRaisesRegex(RuntimeError,'explicit exit_edge'):
            get_trigger(cfg,'SIGNAL',require_exit_edge=True)

    def test_qualifier_event_json_includes_light_key(self):
        published=[]
        manager=SimpleNamespace(
            pub=SimpleNamespace(publish=published.append)
        )
        QualifierScenarioManager.event(
            manager,'QUALIFIER_FORCE_RED','Q_TL2',sim_time=12.5
        )
        payload=json.loads(published[0].data)
        self.assertEqual(payload['suite'],'qualifier')
        self.assertEqual(payload['light_key'],'Q_TL2')


if __name__=='__main__':
    unittest.main()
