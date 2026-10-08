import ast
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace
import unittest

import carla

from kcity_scenario_manager.csv_route_agent import (
    build_exact_global_plan,
    finish_reached,
    install_exact_global_plan,
    parse_road_option,
    reconstruct_csv_waypoint,
    update_progress,
)


class FakeRoadOption(IntEnum):
    LEFT=1
    RIGHT=2
    STRAIGHT=3
    LANEFOLLOW=4
    CHANGELANELEFT=5
    CHANGELANERIGHT=6


class FakeWaypoint:
    def __init__(self,road=3,section=2,lane=-2,s=10.0,x=1.0,y=2.0):
        self.road_id=road
        self.section_id=section
        self.lane_id=lane
        self.s=float(s)
        self.transform=carla.Transform(carla.Location(x=x,y=y,z=0.0))


class FakeMap:
    def __init__(self,resolver):
        self.resolver=resolver
        self.calls=[]

    def get_waypoint_xodr(self,road,lane,s):
        self.calls.append((road,lane,s))
        return self.resolver(road,lane,s)


class FakePlanner:
    def __init__(self):
        self.call=None

    def set_global_plan(self,plan,**kwargs):
        self.call=(list(plan),kwargs)


def row(index=0,**overrides):
    result={
        'index':index,'route_s_m':float(index),'x':1.0,'y':2.0,'z':0.0,
        'yaw':0.0,'road_id':3,'section_id':2,'lane_id':-2,
        'opendrive_s':10.0+index,'is_junction':False,
        'road_option':'LANEFOLLOW',
    }
    result.update(overrides)
    return result


class CsvRouteAgentTests(unittest.TestCase):
    def test_csv_road_lane_s_waypoint_reconstruction(self):
        waypoint=FakeWaypoint()
        fake_map=FakeMap(lambda road,lane,s:waypoint)
        resolved,epsilon,distance=reconstruct_csv_waypoint(
            fake_map,row(),1.0
        )
        self.assertIs(resolved,waypoint)
        self.assertEqual(fake_map.calls[0],(3,-2,10.0))
        self.assertEqual(epsilon,0.0)
        self.assertEqual(distance,0.0)

    def test_section_boundary_lookup_uses_same_road_lane_only(self):
        def resolve(road,lane,s):
            section=3 if s>10.0 else 2
            return FakeWaypoint(road=road,section=section,lane=lane,s=s)
        waypoint,epsilon,_=reconstruct_csv_waypoint(
            FakeMap(resolve),row(section_id=3),1.0
        )
        self.assertEqual(waypoint.section_id,3)
        self.assertGreater(epsilon,0.0)

    def test_waypoint_coordinate_tolerance(self):
        fake_map=FakeMap(
            lambda road,lane,s:FakeWaypoint(
                road=road,lane=lane,s=s,x=3.0,y=2.0
            )
        )
        with self.assertRaisesRegex(RuntimeError,'waypoint_tolerance_m'):
            reconstruct_csv_waypoint(fake_map,row(),1.0)

    def test_wrong_road_lane_lookup_failure(self):
        fake_map=FakeMap(
            lambda _road,_lane,s:FakeWaypoint(road=99,lane=1,s=s)
        )
        with self.assertRaisesRegex(RuntimeError,'exact OpenDRIVE'):
            reconstruct_csv_waypoint(fake_map,row(),1.0)
        missing=FakeMap(lambda _road,_lane,_s:None)
        with self.assertRaisesRegex(RuntimeError,'exact OpenDRIVE'):
            reconstruct_csv_waypoint(missing,row(),1.0)

    def test_road_option_parsing(self):
        for name in (
            'LANEFOLLOW','STRAIGHT','LEFT','RIGHT',
            'CHANGELANELEFT','CHANGELANERIGHT',
        ):
            self.assertEqual(
                parse_road_option(name,FakeRoadOption).name,name
            )

    def test_unknown_road_option_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'unsupported CSV road_option'):
            parse_road_option('UTURN',FakeRoadOption)

    def test_plan_order_and_lane_changes_are_preserved(self):
        rows=[
            row(0,road_option='CHANGELANELEFT',opendrive_s=10.0),
            row(1,road_option='LANEFOLLOW',opendrive_s=11.0),
            row(2,road_option='CHANGELANERIGHT',opendrive_s=12.0),
        ]
        def resolve(road,lane,s):
            return FakeWaypoint(
                road=road,section=2,lane=lane,s=s,x=1.0,y=2.0
            )
        data=build_exact_global_plan(
            FakeMap(resolve),rows,FakeRoadOption,1.0
        )
        self.assertEqual([wp.s for wp,_ in data.plan],[10.0,11.0,12.0])
        self.assertEqual(
            [option.name for _,option in data.plan],
            ['CHANGELANELEFT','LANEFOLLOW','CHANGELANERIGHT'],
        )

    def test_exact_plan_install_signature(self):
        planner=FakePlanner()
        plan=[('wp0',FakeRoadOption.LANEFOLLOW)]
        install_exact_global_plan(planner,plan)
        self.assertEqual(planner.call[0],plan)
        self.assertEqual(planner.call[1],{
            'stop_waypoint_creation':True,'clean_queue':True,
        })

    def test_traffic_manager_and_destination_apis_are_not_used(self):
        source=Path(__file__).resolve().parents[1]/(
            'kcity_scenario_manager/csv_route_agent.py'
        )
        tree=ast.parse(source.read_text(encoding='utf-8'))
        called_attributes={
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
        }
        self.assertNotIn('set_path',called_attributes)
        self.assertNotIn('set_destination',called_attributes)
        self.assertNotIn('tick',called_attributes)

    def test_progress_uses_forward_window_and_rejects_large_jump(self):
        rows=[row(i,x=float(i),y=0.0) for i in range(100)]
        accepted=update_progress(
            rows,10,carla.Location(x=13.1,y=0.0),
            forward_window=40,max_forward_jump=15,
        )
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.index,13)
        rejected=update_progress(
            rows,10,carla.Location(x=35.0,y=0.0),
            forward_window=40,max_forward_jump=15,
        )
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.index,10)
        self.assertIn('forward progress jump',rejected.reason)

    def test_progress_does_not_rewind(self):
        rows=[row(i,x=float(i),y=0.0) for i in range(20)]
        result=update_progress(
            rows,10,carla.Location(x=9.0,y=0.0),
            backward_window=2,
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.index,10)
        self.assertIn('backward progress',result.reason)

    def test_finish_detection(self):
        self.assertTrue(finish_reached(True,498,499,0.8,3.0))
        self.assertFalse(finish_reached(False,498,499,0.8,3.0))
        self.assertFalse(finish_reached(True,450,499,0.8,3.0))
        self.assertFalse(finish_reached(True,498,499,4.0,3.0))


if __name__=='__main__':
    unittest.main()
