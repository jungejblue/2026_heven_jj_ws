from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import carla

from kcity_scenario_manager.tools.route.lane_route_builder import (
    PREVIEW_COLORS,
    RoutePoint,
    TopologyConnection,
    calculate_exit_edge,
    detect_route_loops,
    directed_topology_connection,
    generate_preview_lines,
    merge_route_segments,
    parse_required_triggers,
    parse_route_anchors,
    project_route_anchor,
    remove_duplicate_waypoints,
    route_rows,
    segment_detour_result,
    validate_route_transition,
    validate_required_trigger_order,
)
from kcity_scenario_manager.competition_common import TriggerBox,load_yaml
from kcity_scenario_manager.route_csv import (
    downsample_locations, read_route_csv, route_locations, write_route_csv,
)


class FakeWaypoint:
    def __init__(
        self,x,y=0.0,z=0.0,index=0,road=1,lane=-1,yaw=0.0,
        junction=False,
    ):
        self.transform=SimpleNamespace(
            location=carla.Location(x=x,y=y,z=z),
            rotation=SimpleNamespace(yaw=yaw),
        )
        self.road_id=road
        self.section_id=0
        self.lane_id=lane
        self.s=float(index)
        self.is_junction=junction
        self.lane_type=carla.LaneType.Driving
        self._successors=[]
        self._left=None
        self._right=None

    def next(self,distance):
        del distance
        return list(self._successors)

    def get_left_lane(self):
        return self._left

    def get_right_lane(self):
        return self._right


def trigger(center_x,center_y=0.0,extent_x=0.4,extent_y=0.4):
    return {
        'enabled':True,
        'center':{'x':center_x,'y':center_y,'z':0.0},
        'extent':{'x':extent_x,'y':extent_y,'z':1.0},
        'yaw_deg':0.0,
    }


class LaneRouteBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path=(
            Path(__file__).resolve().parents[1]/'config/qualifier.yaml'
        )
        cls.cfg=load_yaml(cls.config_path)

    def test_route_anchor_and_trigger_source_parsing(self):
        anchors=parse_route_anchors(self.cfg)
        self.assertEqual(
            [anchor.key for anchor in anchors],
            ['START','A01','A02','A03','A04','A05','A06','A07','A08','FINISH'],
        )
        self.assertEqual(anchors[0].source,'trigger')
        self.assertEqual(
            anchors[0].location,
            (-42.5,395.3,0.8),
        )
        self.assertEqual(anchors[1].location,(-14.6,334.6,0.0))
        self.assertEqual(anchors[5].key,'A05')
        self.assertEqual(anchors[5].location,(72.0,267.2,0.0))
        self.assertEqual((anchors[5].road_id,anchors[5].lane_id),(31,-1))
        self.assertEqual(
            parse_required_triggers(self.cfg),
            ['Q_STOP1','Q_SIGNAL1','Q_STOP2','Q_SIGNAL2'],
        )

    def test_lane_hint_selects_only_matching_road_and_lane(self):
        anchor=parse_route_anchors({'route_anchors':{
            'A05':{
                'source':'location',
                'location':{'x':72.0,'y':267.2,'z':0.0},
                'road_id':31,
                'lane_id':-1,
            },
            'END':{
                'source':'location',
                'location':{'x':73.0,'y':267.2,'z':0.0},
            },
        }})[0]
        nearest_wrong=FakeWaypoint(72.0,267.2)
        nearest_wrong.road_id=775
        nearest_wrong.lane_id=1
        hinted=FakeWaypoint(67.5,260.6)
        hinted.road_id=31
        hinted.lane_id=-1
        fake_map=SimpleNamespace()
        self.assertIs(
            project_route_anchor(fake_map,anchor,[nearest_wrong,hinted]),hinted
        )
        with self.assertRaisesRegex(RuntimeError,'matched no generated'):
            project_route_anchor(fake_map,anchor,[nearest_wrong])

    def test_partial_lane_hint_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'specify road_id and lane_id together'):
            parse_route_anchors({'route_anchors':{
                'A':{
                    'source':'location',
                    'location':{'x':0,'y':0,'z':0},
                    'road_id':31,
                },
                'B':{
                    'source':'location',
                    'location':{'x':1,'y':0,'z':0},
                },
            }})

    def test_segment_merge_and_duplicate_removal(self):
        a,b,c=FakeWaypoint(0,index=0),FakeWaypoint(1,index=1),FakeWaypoint(2,index=2)
        near_b=FakeWaypoint(1.01,index=1)
        self.assertEqual(len(remove_duplicate_waypoints([a,a,b])),2)
        merged,issues=merge_route_segments(
            [[a,b],[near_b,c]],
            lambda first,second,grp_trace:TopologyConnection(
                True,first.transform.location.distance(second.transform.location),
                0.0,0,'test connection',
            ),
            ['A -> B','B -> C'],
        )
        self.assertEqual(issues,[])
        self.assertEqual([wp.transform.location.x for wp in merged],[0.0,1.0,2.0])

    def test_duplicate_route_point_keeps_later_road_option(self):
        source=FakeWaypoint(0,index=0,road=3,lane=-2)
        target=FakeWaypoint(1,index=1,road=3,lane=-1)
        cleaned=remove_duplicate_waypoints([
            RoutePoint(source,'CHANGELANELEFT'),
            RoutePoint(target,'CHANGELANELEFT'),
            RoutePoint(target,'LANEFOLLOW'),
        ])
        self.assertEqual(len(cleaned),2)
        self.assertEqual(cleaned[0].road_option,'CHANGELANELEFT')
        self.assertEqual(cleaned[1].road_option,'LANEFOLLOW')

    def test_segment_merge_reports_disconnected_topology_boundary(self):
        a,b,c,d=(
            FakeWaypoint(0,index=0),FakeWaypoint(1,index=1),
            FakeWaypoint(2,index=2),FakeWaypoint(3,index=3),
        )
        merged,issues=merge_route_segments(
            [[a,b],[c,d]],
            lambda first,second,grp_trace:TopologyConnection(
                False,first.transform.location.distance(second.transform.location),
                None,0,'not connected',
            ),
            ['A -> B','B -> C'],
        )
        self.assertEqual(len(merged),4)
        self.assertRegex(issues[0],r'merge boundary gap=1\.000m')

    def test_directed_topology_connection_rejects_lateral_gap(self):
        start=FakeWaypoint(0,index=0)
        middle=FakeWaypoint(1,index=1)
        destination=FakeWaypoint(2,index=2)
        start._successors=[middle]
        middle._successors=[destination]
        self.assertTrue(
            directed_topology_connection(start,destination,1.0).connected
        )
        lateral=FakeWaypoint(2,2,index=2)
        lateral.lane_id=-2
        self.assertFalse(
            directed_topology_connection(start,lateral,1.0).connected
        )

    def validate_transition(self,first,second,option,grp_trace=True):
        return validate_route_transition(
            RoutePoint(first,option),RoutePoint(second,'LANEFOLLOW'),
            sampling_resolution_m=1.0,max_continuity_gap_m=3.0,
            lane_change_max_gap_m=8.0,
            lane_change_max_yaw_delta_deg=45.0,
            grp_trace=grp_trace,
        )

    def test_lanefollow_continuity(self):
        start=FakeWaypoint(0,index=0)
        destination=FakeWaypoint(1,index=1)
        start._successors=[destination]
        result=self.validate_transition(start,destination,'LANEFOLLOW',False)
        self.assertTrue(result.connected)
        self.assertEqual(result.classification,'longitudinal')

    def test_valid_lane_change_left(self):
        start=FakeWaypoint(0,0,index=10,road=3,lane=-2,yaw=-63)
        destination=FakeWaypoint(6,2,index=15,road=3,lane=-1,yaw=-62)
        start._left=destination
        result=self.validate_transition(
            start,destination,'CHANGELANELEFT'
        )
        self.assertTrue(result.connected)
        self.assertEqual(result.classification,'lane_change')

    def test_valid_lane_change_right(self):
        start=FakeWaypoint(0,0,index=10,road=3,lane=1,yaw=117)
        destination=FakeWaypoint(5,3,index=15,road=3,lane=2,yaw=116)
        start._right=destination
        self.assertTrue(self.validate_transition(
            start,destination,'CHANGELANERIGHT'
        ).connected)

    def test_lane_change_lateral_jump_rejected(self):
        start=FakeWaypoint(0,0,index=10,road=3,lane=-2)
        destination=FakeWaypoint(10,0,index=15,road=3,lane=-1)
        start._left=destination
        result=self.validate_transition(
            start,destination,'CHANGELANELEFT'
        )
        self.assertFalse(result.connected)
        self.assertIn('lane_change_max_gap_m',result.reason)

    def test_junction_road_id_transition_accepted(self):
        first=FakeWaypoint(1,index=0,road=649,junction=True)
        second=FakeWaypoint(1,index=0,road=650,junction=True)
        result=self.validate_transition(first,second,'LANEFOLLOW',True)
        self.assertTrue(result.connected)
        self.assertEqual(result.classification,'junction_connector')

    def test_true_disconnected_route_rejected(self):
        first=FakeWaypoint(0,index=0)
        second=FakeWaypoint(2,index=2)
        result=self.validate_transition(first,second,'LANEFOLLOW',False)
        self.assertFalse(result.connected)
        self.assertEqual(result.classification,'disconnected')

    def test_segment_detour_rejected(self):
        start=FakeWaypoint(0,index=0)
        end=FakeWaypoint(50,index=50)
        route=[start,FakeWaypoint(0,100,index=100),end]
        result=segment_detour_result('A -> B',route,start,end,2.5)
        self.assertFalse(result.accepted)
        self.assertGreater(result.ratio,2.5)

    def test_a04_a05_351m_detour_regression_rejected(self):
        start=FakeWaypoint(0,index=0)
        end=FakeWaypoint(53,index=53)
        route=[start,FakeWaypoint(202.2995,index=202),end]
        result=segment_detour_result('A04 -> A05',route,start,end,2.5)
        self.assertAlmostEqual(result.route_length_m,351.599,places=3)
        self.assertFalse(result.accepted)

    def test_loop_detection_uses_road_lane_s_revisit(self):
        route=[]
        for x in (0,10,20,30,20,10,0):
            route.append(FakeWaypoint(x,index=x,road=7,lane=-1))
        issues=detect_route_loops(
            route,s_bin_m=5.0,min_route_separation_m=25.0,
            location_tolerance_m=1.0,
        )
        self.assertTrue(issues)
        self.assertIn('loop/revisit',issues[0])

    def test_draw_preview_data_generation(self):
        route=[
            RoutePoint(FakeWaypoint(0),'LANEFOLLOW'),
            RoutePoint(FakeWaypoint(1),'STRAIGHT'),
            RoutePoint(FakeWaypoint(2),'CHANGELANELEFT'),
            RoutePoint(FakeWaypoint(3),'LANEFOLLOW'),
        ]
        lines=generate_preview_lines(route)
        self.assertEqual(len(lines),3)
        self.assertEqual(lines[0].color,PREVIEW_COLORS['LANEFOLLOW'])
        self.assertEqual(lines[1].color,PREVIEW_COLORS['STRAIGHT'])
        self.assertEqual(lines[2].color,PREVIEW_COLORS['CHANGELANELEFT'])

    def test_route_s_and_csv_round_trip_and_autopilot_parsing(self):
        rows=route_rows([
            FakeWaypoint(0,index=0),
            FakeWaypoint(3,index=1),
            FakeWaypoint(3,4,index=2),
        ])
        self.assertEqual([row['route_s_m'] for row in rows],[0.0,3.0,7.0])
        with TemporaryDirectory() as directory:
            path=Path(directory)/'route.csv'
            write_route_csv(path,rows)
            loaded=read_route_csv(path)
            self.assertEqual([row['route_s_m'] for row in loaded],[0.0,3.0,7.0])
            self.assertEqual(
                [row['road_option'] for row in loaded],['LANEFOLLOW']*3
            )
            points=downsample_locations(route_locations(loaded),4.0)
            self.assertEqual(points,[(0.0,0.0,0.0),(3.0,4.0,0.0)])

    def test_csv_allows_zero_length_road_connector(self):
        first=FakeWaypoint(1,index=0,road=649,junction=True)
        second=FakeWaypoint(1,index=0,road=650,junction=True)
        rows=route_rows([
            RoutePoint(first,'LANEFOLLOW'),
            RoutePoint(second,'STRAIGHT'),
        ])
        self.assertEqual([row['route_s_m'] for row in rows],[0.0,0.0])
        with TemporaryDirectory() as directory:
            path=Path(directory)/'connector.csv'
            write_route_csv(path,rows)
            self.assertEqual(len(read_route_csv(path)),2)

    def test_required_trigger_order_validation(self):
        cfg={'triggers':{
            'ONE':trigger(1.0),
            'TWO':trigger(2.0),
            'THREE':trigger(3.0),
        }}
        route=[FakeWaypoint(x,index=i) for i,x in enumerate((0,1,2,3,4))]
        indices=validate_required_trigger_order(
            route,cfg,['ONE','TWO','THREE']
        )
        self.assertEqual(indices,{'ONE':1,'TWO':2,'THREE':3})
        with self.assertRaisesRegex(RuntimeError,'out of order'):
            validate_required_trigger_order(route,cfg,['TWO','ONE','THREE'])

    def test_exit_edge_uses_inside_to_outside_crossing(self):
        route=[FakeWaypoint(x,index=i) for i,x in enumerate((-2,-0.5,0.5,2))]
        box=TriggerBox.from_dict(trigger(0.0,extent_x=1.0,extent_y=1.0))
        self.assertEqual(calculate_exit_edge(route,box),'+x')
        reverse=list(reversed(route))
        self.assertEqual(calculate_exit_edge(reverse,box),'-x')


if __name__=='__main__':
    unittest.main()
