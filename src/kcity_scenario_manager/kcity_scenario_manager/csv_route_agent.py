#!/usr/bin/env python3
"""Execute an exact CSV Waypoint/RoadOption plan for benchmark smoke tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys

import carla
import rclpy
from rclpy.node import Node

from .competition_common import speed_mps
from .control_graph import AUTOPILOT_TOPIC, CONTROL_TOPIC, MANUAL_TOPIC, publisher_names
from .route_csv import read_route_csv


SUPPORTED_ROAD_OPTIONS=(
    'LANEFOLLOW','STRAIGHT','LEFT','RIGHT',
    'CHANGELANELEFT','CHANGELANERIGHT',
)
TRANSITION_OPTIONS={
    'STRAIGHT','LEFT','RIGHT','CHANGELANELEFT','CHANGELANERIGHT',
}
XODR_S_EPSILONS=(0.0,1e-5,-1e-5,1e-4,-1e-4,1e-3,-1e-3)


class CsvRouteAgentFailure(RuntimeError):
    """A braked route-test failure that must terminate the control process."""


@dataclass(frozen=True)
class ProgressMatch:
    index: int
    distance_m: float
    candidate_index: int
    accepted: bool
    reason: str


@dataclass(frozen=True)
class RoutePlanData:
    rows: list
    plan: list
    boundary_adjustments: int


def load_agent_api():
    """Load the installed CARLA 0.9.15 LocalPlanner without invoking GRP."""
    try:
        from agents.navigation.local_planner import LocalPlanner,RoadOption
        return LocalPlanner,RoadOption
    except ModuleNotFoundError:
        roots=[]
        if os.environ.get('CARLA_ROOT'):
            roots.append(Path(os.environ['CARLA_ROOT']).expanduser())
        roots.extend((Path.home()/'CARLA_0.9.15',Path('/opt/carla-simulator')))
        for root in roots:
            agents_parent=root/'PythonAPI/carla'
            if (agents_parent/'agents/navigation/local_planner.py').exists():
                sys.path.insert(0,str(agents_parent))
                from agents.navigation.local_planner import LocalPlanner,RoadOption
                return LocalPlanner,RoadOption
        raise RuntimeError(
            'CARLA agents package is unavailable; set CARLA_ROOT or add '
            'CARLA PythonAPI/carla to PYTHONPATH'
        )


def parse_road_option(value,road_option_enum):
    name=str(value).strip().upper()
    if name not in SUPPORTED_ROAD_OPTIONS:
        raise RuntimeError(f'unsupported CSV road_option={value!r}')
    try:
        return getattr(road_option_enum,name)
    except AttributeError as exc:
        raise RuntimeError(
            f'installed CARLA RoadOption has no {name} member'
        ) from exc


def _xy_distance(waypoint,row):
    location=waypoint.transform.location
    return math.hypot(
        float(location.x)-float(row['x']),
        float(location.y)-float(row['y']),
    )


def reconstruct_csv_waypoint(carla_map,row,tolerance_m):
    """Resolve only the requested OpenDRIVE road/lane/s and section."""
    road_id=int(row['road_id'])
    section_id=int(row['section_id'])
    lane_id=int(row['lane_id'])
    opendrive_s=float(row['opendrive_s'])
    candidates=[]
    for epsilon in XODR_S_EPSILONS:
        waypoint=carla_map.get_waypoint_xodr(
            road_id,lane_id,opendrive_s+epsilon
        )
        if waypoint is None:
            continue
        if int(waypoint.road_id)!=road_id or int(waypoint.lane_id)!=lane_id:
            continue
        if int(waypoint.section_id)!=section_id:
            continue
        candidates.append((_xy_distance(waypoint,row),abs(epsilon),waypoint))
    if not candidates:
        raise RuntimeError(
            f'CSV index {row["index"]}: exact OpenDRIVE waypoint lookup failed '
            f'for road/section/lane/s='
            f'{road_id}/{section_id}/{lane_id}/{opendrive_s:.6f}'
        )
    distance,epsilon,waypoint=min(candidates,key=lambda item:(item[1],item[0]))
    if distance>float(tolerance_m):
        raise RuntimeError(
            f'CSV index {row["index"]}: reconstructed waypoint is '
            f'{distance:.3f}m from CSV x/y; '
            f'waypoint_tolerance_m={float(tolerance_m):.3f}'
        )
    return waypoint,epsilon,distance


def build_exact_global_plan(carla_map,rows,road_option_enum,tolerance_m=1.0):
    if float(tolerance_m)<=0:
        raise ValueError('waypoint_tolerance_m must be positive')
    plan=[]
    boundary_adjustments=0
    for row in rows:
        waypoint,epsilon,_=reconstruct_csv_waypoint(
            carla_map,row,tolerance_m
        )
        if epsilon:
            boundary_adjustments+=1
        option=parse_road_option(row['road_option'],road_option_enum)
        plan.append((waypoint,option))
    return RoutePlanData(list(rows),plan,boundary_adjustments)


def option_runs(rows):
    if not rows:
        return []
    result=[]
    start=0
    option=str(rows[0]['road_option'])
    for index,row in enumerate(rows[1:],start=1):
        current=str(row['road_option'])
        if current!=option:
            result.append((start,index-1,option))
            start=index
            option=current
    result.append((start,len(rows)-1,option))
    return result


def update_progress(
    rows,previous_index,location,forward_window=40,max_forward_jump=15,
    backward_window=2,
):
    if not rows:
        raise ValueError('progress tracking requires route rows')
    previous=max(0,min(int(previous_index),len(rows)-1))
    start=max(0,previous-int(backward_window))
    end=min(len(rows),previous+int(forward_window)+1)
    candidate=min(
        range(start,end),
        key=lambda index:math.hypot(
            float(rows[index]['x'])-float(location.x),
            float(rows[index]['y'])-float(location.y),
        ),
    )
    candidate_distance=math.hypot(
        float(rows[candidate]['x'])-float(location.x),
        float(rows[candidate]['y'])-float(location.y),
    )
    if candidate<previous:
        expected_distance=math.hypot(
            float(rows[previous]['x'])-float(location.x),
            float(rows[previous]['y'])-float(location.y),
        )
        return ProgressMatch(
            previous,expected_distance,candidate,False,
            f'backward progress {previous}->{candidate} rejected',
        )
    if candidate-previous>int(max_forward_jump):
        expected_distance=math.hypot(
            float(rows[previous]['x'])-float(location.x),
            float(rows[previous]['y'])-float(location.y),
        )
        return ProgressMatch(
            previous,expected_distance,candidate,False,
            f'forward progress jump {previous}->{candidate} rejected',
        )
    return ProgressMatch(
        candidate,candidate_distance,candidate,True,'local forward match',
    )


def finish_reached(agent_done,progress_index,total_points,distance_to_finish_m,
                   finish_tolerance_m):
    return (
        bool(agent_done)
        and int(progress_index)>=int(total_points)-2
        and float(distance_to_finish_m)<=float(finish_tolerance_m)
    )


class ExactGlobalPlanAgent:
    """Small agent facade backed only by LocalPlanner; no GRP is constructed."""

    def __init__(self,vehicle,carla_map,plan,target_speed_kmh,control_hz,
                 local_planner_class):
        self._planner=local_planner_class(
            vehicle,
            opt_dict={
                'target_speed':float(target_speed_kmh),
                'dt':1.0/float(control_hz),
            },
            map_inst=carla_map,
        )
        install_exact_global_plan(self._planner,plan)

    def run_step(self):
        return self._planner.run_step()

    def done(self):
        return self._planner.done()


def install_exact_global_plan(agent,plan):
    agent.set_global_plan(
        plan,stop_waypoint_creation=True,clean_queue=True
    )


class CsvRouteAgent(Node):
    def __init__(self):
        super().__init__('csv_route_agent')
        self.declare_parameter('host','127.0.0.1')
        self.declare_parameter('port',2000)
        self.declare_parameter('timeout_sec',10.0)
        self.declare_parameter('expected_map','heven_kcity/Maps/kcity/kcity')
        self.declare_parameter('ego_role_name','ego_vehicle')
        self.declare_parameter('ego_blueprint_contains','heven')
        self.declare_parameter('route_csv','')
        self.declare_parameter('waypoint_tolerance_m',1.0)
        self.declare_parameter('target_speed_kmh',15.0)
        self.declare_parameter('control_hz',20.0)
        self.declare_parameter('tm_port',8000)
        self.declare_parameter('disable_tm_autopilot',True)
        self.declare_parameter('progress_forward_window',40)
        self.declare_parameter('progress_max_forward_jump',15)
        self.declare_parameter('progress_backward_window',2)
        self.declare_parameter('finish_tolerance_m',3.0)
        self.declare_parameter('transition_log_radius_m',3.0)

        route_csv=str(self.get_parameter('route_csv').value).strip()
        if not route_csv:
            raise RuntimeError('route_csv is required')
        self.rows=read_route_csv(route_csv)
        self.client=carla.Client(
            str(self.get_parameter('host').value),
            int(self.get_parameter('port').value),
        )
        self.client.set_timeout(float(self.get_parameter('timeout_sec').value))
        self.world=self.client.get_world()
        self.carla_map=self.world.get_map()
        expected=str(self.get_parameter('expected_map').value).strip('/')
        actual=str(self.carla_map.name).strip('/')
        if expected and not actual.endswith(expected):
            raise RuntimeError(f'expected CARLA map {expected!r}, got {actual!r}')
        self.local_planner_class,self.road_option_enum=load_agent_api()
        self.plan_data=build_exact_global_plan(
            self.carla_map,self.rows,self.road_option_enum,
            float(self.get_parameter('waypoint_tolerance_m').value),
        )
        self.role=str(self.get_parameter('ego_role_name').value)
        self.blueprint_contains=str(
            self.get_parameter('ego_blueprint_contains').value
        ).lower()
        self.control_hz=float(self.get_parameter('control_hz').value)
        if self.control_hz<=0:
            raise RuntimeError('control_hz must be positive')
        self.ego=None
        self.agent=None
        self.state='WAITING_FOR_EGO'
        self.progress_index=0
        self.progress_match=None
        self.logged_transitions=set()
        self.last_progress_warning=''
        self.activation_sim_time=None
        self.done_sim_time=None
        self.deviation_started=False
        self.deviation_sum=0.0
        self.deviation_count=0
        self.deviation_max=0.0
        self.last_control_check=0.0
        self.last_wait_log=-float('inf')
        self.control_timer=self.create_timer(1.0/self.control_hz,self.control_step)
        self.diagnostic_timer=self.create_timer(1.0,self.log_diagnostic)
        first=self.plan_data.plan[0][0]
        last=self.plan_data.plan[-1][0]
        runs=' -> '.join(
            f'{start}-{end}:{option}'
            for start,end,option in option_runs(self.rows)
        )
        self.get_logger().info(
            f'[CSV_AGENT] exact global plan loaded: '
            f'points={len(self.plan_data.plan)}, '
            f'boundary_s_adjustments={self.plan_data.boundary_adjustments}'
        )
        self.get_logger().info(
            f'[CSV_AGENT] first={self._waypoint_text(first)}; '
            f'last={self._waypoint_text(last)}'
        )
        self.get_logger().info(f'[CSV_AGENT] RoadOption sequence: {runs}')
        self.get_logger().info(
            '[CSV_AGENT] smoke-test-only direct vehicle.apply_control; '
            'no TrafficManager.set_path, no set_destination, no world.tick'
        )

    @staticmethod
    def _waypoint_text(waypoint):
        location=waypoint.transform.location
        return (
            f'road/section/lane/s={waypoint.road_id}/'
            f'{waypoint.section_id}/{waypoint.lane_id}/{waypoint.s:.3f} '
            f'location=({location.x:.3f},{location.y:.3f},{location.z:.3f})'
        )

    def _stop_control(self):
        control=carla.VehicleControl()
        control.throttle=0.0
        control.brake=1.0
        control.hand_brake=False
        return control

    def _control_sources(self):
        return (
            publisher_names(self,CONTROL_TOPIC),
            publisher_names(self,MANUAL_TOPIC),
            publisher_names(self,AUTOPILOT_TOPIC),
        )

    def _competing_source_reason(self):
        control,manual,autopilot=self._control_sources()
        publishers=control+manual+autopilot
        if publishers:
            return 'ROS ego control publishers active: '+', '.join(publishers)
        forbidden={
            'csv_autopilot','bridge_wasd_control','bridge_autopilot_control',
        }
        active={name for name,_namespace in self.get_node_names_and_namespaces()}
        conflicts=sorted(active & forbidden)
        if conflicts:
            return 'competing control nodes active: '+', '.join(conflicts)
        return ''

    def _find_ego(self):
        matches=[
            actor for actor in self.world.get_actors().filter('vehicle.*')
            if actor.attributes.get('role_name','')==self.role
        ]
        if len(matches)>1:
            raise RuntimeError(
                f'{len(matches)} vehicles have role_name={self.role}'
            )
        if not matches:
            return None
        ego=matches[0]
        if self.blueprint_contains not in str(ego.type_id).lower():
            raise RuntimeError(
                f'role_name={self.role} actor is not BP_HEVEN: '
                f'type_id={ego.type_id!r}'
            )
        return ego

    def _activate(self,ego):
        conflict=self._competing_source_reason()
        if conflict:
            raise RuntimeError(conflict)
        if bool(self.get_parameter('disable_tm_autopilot').value):
            ego.set_autopilot(False,int(self.get_parameter('tm_port').value))
        self.ego=ego
        self.agent=ExactGlobalPlanAgent(
            ego,self.carla_map,self.plan_data.plan,
            float(self.get_parameter('target_speed_kmh').value),
            self.control_hz,self.local_planner_class,
        )
        self.activation_sim_time=float(
            self.world.get_snapshot().timestamp.elapsed_seconds
        )
        self.state='RUNNING'
        tm_status=(
            'Traffic Manager autopilot disabled'
            if bool(self.get_parameter('disable_tm_autopilot').value)
            else 'freshly spawned ego has Traffic Manager autopilot off'
        )
        self.get_logger().info(
            f'[CSV_AGENT] RUNNING ego_id={ego.id} type={ego.type_id} '
            f'target_speed_kmh='
            f'{float(self.get_parameter("target_speed_kmh").value):.1f}; '
            f'{tm_status}'
        )

    def _fail(self,message):
        if self.state=='FAILED':
            return
        self.state='FAILED'
        if self.ego is not None and self.ego.is_alive:
            self.ego.apply_control(self._stop_control())
        row=self.rows[self.progress_index]
        self.get_logger().error(
            f'[CSV_AGENT] FAILED idx={self.progress_index}/{len(self.rows)} '
            f'road/section/lane={row["road_id"]}/'
            f'{row["section_id"]}/{row["lane_id"]} '
            f'option={row["road_option"]}: {message}'
        )
        raise CsvRouteAgentFailure(message)

    def _update_progress(self,location):
        match=update_progress(
            self.rows,self.progress_index,location,
            int(self.get_parameter('progress_forward_window').value),
            int(self.get_parameter('progress_max_forward_jump').value),
            int(self.get_parameter('progress_backward_window').value),
        )
        self.progress_match=match
        if match.accepted:
            self.progress_index=match.index
            self.last_progress_warning=''
        elif match.reason!=self.last_progress_warning:
            self.get_logger().warning(f'[CSV_AGENT] {match.reason}')
            self.last_progress_warning=match.reason
        if not self.deviation_started and self.progress_index==0:
            self.deviation_started=match.distance_m<=3.0
        if self.deviation_started:
            self.deviation_sum+=match.distance_m
            self.deviation_count+=1
            self.deviation_max=max(self.deviation_max,match.distance_m)
        self._log_reached_transitions(location)

    def _log_reached_transitions(self,location):
        radius=float(self.get_parameter('transition_log_radius_m').value)
        for start,_end,option in option_runs(self.rows):
            if option not in TRANSITION_OPTIONS or start in self.logged_transitions:
                continue
            if start>self.progress_index:
                continue
            row=self.rows[start]
            distance=math.hypot(
                float(row['x'])-float(location.x),
                float(row['y'])-float(location.y),
            )
            if distance>radius and self.progress_index==start:
                continue
            self.logged_transitions.add(start)
            self.get_logger().info(
                f'[CSV_AGENT_TRANSITION] idx={start}/{len(self.rows)} '
                f's={float(row["route_s_m"]):.3f}m '
                f'road/section/lane={row["road_id"]}/'
                f'{row["section_id"]}/{row["lane_id"]} '
                f'option={option} expected_location='
                f'({float(row["x"]):.3f},{float(row["y"]):.3f})'
            )

    def control_step(self):
        if self.state in {'DONE','FAILED'}:
            if self.ego is not None and self.ego.is_alive:
                self.ego.apply_control(self._stop_control())
            return
        if self.ego is None:
            try:
                ego=self._find_ego()
                if ego is None:
                    now=float(self.world.get_snapshot().timestamp.elapsed_seconds)
                    if now-self.last_wait_log>=5.0:
                        self.get_logger().info(
                            f'[CSV_AGENT] waiting for BP_HEVEN role_name={self.role}'
                        )
                        self.last_wait_log=now
                    return
                self._activate(ego)
            except RuntimeError as exc:
                self._fail(str(exc))
                return
        if self.ego is None or not self.ego.is_alive:
            self._fail('ego actor disappeared')
            return
        now=float(self.world.get_snapshot().timestamp.elapsed_seconds)
        if now-self.last_control_check>=0.5:
            conflict=self._competing_source_reason()
            if conflict:
                self._fail(conflict)
                return
            self.last_control_check=now
        location=self.ego.get_location()
        self._update_progress(location)
        control=self.agent.run_step()
        self.ego.apply_control(control)
        last_location=self.plan_data.plan[-1][0].transform.location
        finish_distance=float(location.distance(last_location))
        if finish_reached(
            self.agent.done(),self.progress_index,len(self.rows),finish_distance,
            float(self.get_parameter('finish_tolerance_m').value),
        ):
            self.ego.apply_control(self._stop_control())
            self.done_sim_time=now
            self.state='DONE'
            mean=(
                self.deviation_sum/self.deviation_count
                if self.deviation_count else float('nan')
            )
            self.get_logger().info(
                f'[CSV_AGENT] DONE points={len(self.rows)} '
                f'duration_sec={now-self.activation_sim_time:.3f} '
                f'max_deviation_m={self.deviation_max:.3f} '
                f'mean_deviation_m={mean:.3f}'
            )
        elif self.agent.done():
            self._fail(
                f'LocalPlanner queue ended {finish_distance:.3f}m from FINISH'
            )

    def log_diagnostic(self):
        if self.ego is None or not self.ego.is_alive:
            return
        location=self.ego.get_location()
        waypoint=self.carla_map.get_waypoint(
            location,project_to_road=True,lane_type=carla.LaneType.Driving
        )
        row=self.rows[self.progress_index]
        deviation=(
            self.progress_match.distance_m
            if self.progress_match is not None else float('nan')
        )
        self.get_logger().info(
            f'[CSV_AGENT] state={self.state} '
            f'location=({location.x:.3f},{location.y:.3f},{location.z:.3f}) '
            f'speed_kmh={speed_mps(self.ego)*3.6:.2f} '
            f'ego road/section/lane={waypoint.road_id}/'
            f'{waypoint.section_id}/{waypoint.lane_id} '
            f'idx={self.progress_index}/{len(self.rows)} '
            f's={float(row["route_s_m"]):.3f}m '
            f'expected road/lane={row["road_id"]}/{row["lane_id"]} '
            f'option={row["road_option"]} deviation={deviation:.3f}m'
        )

    def destroy_node(self):
        if self.ego is not None and self.ego.is_alive:
            try:
                self.ego.apply_control(self._stop_control())
            except RuntimeError as exc:
                self.get_logger().warning(
                    f'[CSV_AGENT] failed to brake on shutdown: {exc}'
                )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=CsvRouteAgent()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except CsvRouteAgentFailure:
        raise SystemExit(1) from None
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
