#!/usr/bin/env python3
"""Build and validate a lane-center qualifier route from ordered anchors."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import sys

import carla
import rclpy
from rclpy.node import Node

from ...competition_common import TriggerBox,connect_carla,load_yaml
from ...route_csv import resolve_route_csv_path,write_route_csv


EXIT_EDGES=('+x','-x','+y','-y')
RECOMMENDATION_KEYS=('START','FINISH','Q_STOP1','Q_SIGNAL1','Q_STOP2','Q_SIGNAL2')
LONGITUDINAL_OPTIONS=('LANEFOLLOW','LEFT','RIGHT','STRAIGHT')
LANE_CHANGE_OPTIONS=('CHANGELANELEFT','CHANGELANERIGHT')
PREVIEW_COLORS={
    'LANEFOLLOW':(0,255,64),
    'LEFT':(0,220,255),
    'RIGHT':(0,220,255),
    'STRAIGHT':(0,220,255),
    'CHANGELANELEFT':(255,0,220),
    'CHANGELANERIGHT':(255,0,220),
}


@dataclass(frozen=True)
class RouteAnchor:
    key: str
    location: tuple
    source: str
    trigger_key: str = ''
    road_id: int | None = None
    lane_id: int | None = None


@dataclass(frozen=True)
class RoutePoint:
    waypoint: object
    road_option: object


@dataclass(frozen=True)
class TopologyConnection:
    connected: bool
    gap_m: float
    path_length_m: float | None
    hops: int
    reason: str
    classification: str = 'longitudinal'
    road_option: str = 'LANEFOLLOW'


@dataclass(frozen=True)
class SegmentDetour:
    name: str
    route_length_m: float
    anchor_distance_m: float
    ratio: float
    accepted: bool


@dataclass(frozen=True)
class PreviewLine:
    start: object
    end: object
    color: tuple
    road_option: str


def route_waypoint(item):
    return item.waypoint if isinstance(item,RoutePoint) else item


def road_option_name(option):
    if isinstance(option,RoutePoint):
        option=option.road_option
    if option is None:
        return 'LANEFOLLOW'
    if hasattr(option,'transform') and hasattr(option,'road_id'):
        return 'LANEFOLLOW'
    name=getattr(option,'name',None)
    if name:
        return str(name).upper()
    return str(option).rsplit('.',1)[-1].upper()


def as_route_point(item,default_option='LANEFOLLOW'):
    if isinstance(item,RoutePoint):
        return item
    if isinstance(item,tuple) and len(item)==2:
        return RoutePoint(item[0],item[1])
    return RoutePoint(item,default_option)


def angle_difference_deg(first,second):
    return abs((float(first)-float(second)+180.0)%360.0-180.0)


def xyz(data,context):
    if not isinstance(data,dict) or any(axis not in data for axis in ('x','y','z')):
        raise RuntimeError(f'{context} requires x, y, and z')
    values=tuple(float(data[axis]) for axis in ('x','y','z'))
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f'{context} contains NaN/Inf')
    return values


def parse_route_anchors(cfg):
    raw=cfg.get('route_anchors')
    if not isinstance(raw,dict) or not raw:
        raise RuntimeError('route_anchors must be a non-empty mapping')
    anchors=[]
    for key,spec in raw.items():
        if not isinstance(spec,dict):
            raise RuntimeError(f'route anchor {key} must be a mapping')
        source=str(spec.get('source','location'))
        trigger_key=''
        if source=='trigger':
            trigger_key=str(spec.get('trigger_key',''))
            trigger=cfg.get('triggers',{}).get(trigger_key)
            if trigger is None:
                raise RuntimeError(
                    f'route anchor {key} references missing trigger {trigger_key!r}'
                )
            location=xyz(trigger.get('center'),f'trigger {trigger_key} center')
        elif source=='location':
            location=xyz(spec.get('location'),f'route anchor {key} location')
        else:
            raise RuntimeError(f'route anchor {key} has unsupported source={source!r}')
        road_id=spec.get('road_id')
        lane_id=spec.get('lane_id')
        if (road_id is None)!=(lane_id is None):
            raise RuntimeError(
                f'route anchor {key} must specify road_id and lane_id together'
            )
        anchors.append(RouteAnchor(
            str(key),location,source,trigger_key,
            None if road_id is None else int(road_id),
            None if lane_id is None else int(lane_id),
        ))
    if len(anchors)<2:
        raise RuntimeError('at least two route anchors are required')
    return anchors


def parse_required_triggers(cfg):
    keys=cfg.get('route_required_triggers',[])
    if not isinstance(keys,list):
        raise RuntimeError('route_required_triggers must be a list')
    result=[str(key) for key in keys]
    missing=[key for key in result if key not in cfg.get('triggers',{})]
    if missing:
        raise RuntimeError(
            'route_required_triggers references missing triggers: '
            + ', '.join(missing)
        )
    return result


def project_route_anchor(carla_map,anchor,generated_waypoints=None):
    source=carla.Location(*anchor.location)
    if anchor.road_id is None:
        waypoint=carla_map.get_waypoint(
            source,project_to_road=True,lane_type=carla.LaneType.Driving
        )
        if waypoint is None:
            raise RuntimeError(
                f'{anchor.key} did not project to a Driving waypoint'
            )
        return waypoint

    if generated_waypoints is None:
        raise RuntimeError(
            f'{anchor.key} lane hint requires generated waypoint inventory'
        )
    candidates=[
        waypoint for waypoint in generated_waypoints
        if int(waypoint.road_id)==anchor.road_id
        and int(waypoint.lane_id)==anchor.lane_id
        and waypoint.lane_type==carla.LaneType.Driving
    ]
    if not candidates:
        raise RuntimeError(
            f'{anchor.key} lane hint road_id={anchor.road_id}, '
            f'lane_id={anchor.lane_id} matched no generated Driving waypoint'
        )
    return min(
        candidates,
        key=lambda waypoint:waypoint.transform.location.distance(source),
    )


def waypoint_distance(first,second):
    first=route_waypoint(first)
    second=route_waypoint(second)
    return float(first.transform.location.distance(second.transform.location))


def waypoint_token(waypoint):
    waypoint=route_waypoint(waypoint)
    waypoint_id=getattr(waypoint,'id',None)
    if waypoint_id is not None:
        return ('id',int(waypoint_id))
    location=waypoint.transform.location
    return (
        int(waypoint.road_id),int(waypoint.section_id),int(waypoint.lane_id),
        round(float(waypoint.s),3),round(float(location.x),3),
        round(float(location.y),3),round(float(location.z),3),
    )


def same_topology_waypoint(first,second,location_tolerance_m=0.05):
    first=route_waypoint(first)
    second=route_waypoint(second)
    return (
        int(first.road_id)==int(second.road_id)
        and int(first.section_id)==int(second.section_id)
        and int(first.lane_id)==int(second.lane_id)
        and waypoint_distance(first,second)<=float(location_tolerance_m)
    )


def directed_topology_connection(first,second,sampling_resolution_m):
    """Prove that ``second`` is reachable from ``first`` using Waypoint.next()."""
    first=route_waypoint(first)
    second=route_waypoint(second)
    gap=waypoint_distance(first,second)
    if same_topology_waypoint(first,second):
        return TopologyConnection(True,gap,0.0,0,'same topology waypoint')

    step=max(0.25,min(float(sampling_resolution_m),0.5))
    target_tolerance=max(0.35,1.1*step)
    search_limit=max(4.0,2.0*gap+2.0*step)
    frontier=[(first,0.0,0)]
    seen={waypoint_token(first):0.0}
    while frontier:
        current,path_length,hops=frontier.pop(0)
        if (
            int(current.road_id)==int(second.road_id)
            and int(current.section_id)==int(second.section_id)
            and int(current.lane_id)==int(second.lane_id)
            and waypoint_distance(current,second)<=target_tolerance
        ):
            return TopologyConnection(
                True,gap,path_length,hops,
                f'reachable with next({step:.2f}m)',
            )
        if path_length+step>search_limit:
            continue
        successors=[
            waypoint for waypoint in current.next(step)
            if waypoint.lane_type==carla.LaneType.Driving
        ]
        for successor in successors:
            travelled=path_length+step
            token=waypoint_token(successor)
            if seen.get(token,float('inf'))<=travelled:
                continue
            seen[token]=travelled
            frontier.append((successor,travelled,hops+1))
    return TopologyConnection(
        False,gap,None,0,
        f'not reachable with directed next() within {search_limit:.2f}m',
    )


def _driving_lane(waypoint):
    return waypoint is not None and waypoint.lane_type==carla.LaneType.Driving


def _same_lane_identity(first,second):
    return (
        int(first.road_id)==int(second.road_id)
        and int(first.section_id)==int(second.section_id)
        and int(first.lane_id)==int(second.lane_id)
    )


def validate_lane_change_transition(
    first,second,road_option,lane_change_max_gap_m,
    lane_change_max_yaw_delta_deg,
):
    first=route_waypoint(first)
    second=route_waypoint(second)
    option=road_option_name(road_option)
    gap=waypoint_distance(first,second)
    if option not in LANE_CHANGE_OPTIONS:
        return TopologyConnection(
            False,gap,None,0,f'unsupported lane-change option {option}',
            'lane_change',option,
        )
    adjacent=(
        first.get_left_lane()
        if option=='CHANGELANELEFT'
        else first.get_right_lane()
    )
    if not _driving_lane(adjacent):
        return TopologyConnection(
            False,gap,None,0,
            f'{option} has no adjacent Driving lane in that direction',
            'lane_change',option,
        )
    if not _same_lane_identity(adjacent,second):
        return TopologyConnection(
            False,gap,None,0,
            f'{option} adjacent lane is '
            f'{waypoint_topology_text(adjacent)}, not '
            f'{waypoint_topology_text(second)}',
            'lane_change',option,
        )
    yaw_delta=angle_difference_deg(
        first.transform.rotation.yaw,second.transform.rotation.yaw
    )
    if yaw_delta>float(lane_change_max_yaw_delta_deg):
        return TopologyConnection(
            False,gap,None,0,
            f'lane-change heading delta {yaw_delta:.3f}deg exceeds '
            f'{float(lane_change_max_yaw_delta_deg):.3f}deg',
            'lane_change',option,
        )
    if gap>float(lane_change_max_gap_m):
        return TopologyConnection(
            False,gap,None,0,
            f'lane-change gap exceeds lane_change_max_gap_m='
            f'{float(lane_change_max_gap_m):.3f}m',
            'lane_change',option,
        )
    return TopologyConnection(
        True,gap,gap,1,
        f'{option} matches adjacent Driving lane; '
        f'heading_delta={yaw_delta:.3f}deg',
        'lane_change',option,
    )


def validate_route_transition(
    first,second,sampling_resolution_m,max_continuity_gap_m,
    lane_change_max_gap_m,lane_change_max_yaw_delta_deg,
    grp_trace=False,
):
    """Validate a GRP maneuver without trying to recreate the GRP graph."""
    first_point=as_route_point(first)
    second_point=as_route_point(second)
    first_wp=first_point.waypoint
    second_wp=second_point.waypoint
    option=road_option_name(first_point)
    gap=waypoint_distance(first_wp,second_wp)
    if option in LANE_CHANGE_OPTIONS:
        return validate_lane_change_transition(
            first_wp,second_wp,option,lane_change_max_gap_m,
            lane_change_max_yaw_delta_deg,
        )
    if option not in LONGITUDINAL_OPTIONS:
        return TopologyConnection(
            False,gap,None,0,f'unsupported RoadOption {option}',
            'unknown',option,
        )

    road_changed=int(first_wp.road_id)!=int(second_wp.road_id)
    lane_changed=int(first_wp.lane_id)!=int(second_wp.lane_id)
    junction=bool(first_wp.is_junction or second_wp.is_junction)
    if lane_changed and not road_changed:
        return TopologyConnection(
            False,gap,None,0,
            'lane identity changed without CHANGELANE RoadOption',
            'longitudinal',option,
        )
    if gap>float(max_continuity_gap_m):
        return TopologyConnection(
            False,gap,None,0,
            f'geometric gap exceeds max_continuity_gap_m='
            f'{float(max_continuity_gap_m):.3f}m',
            'geometric_discontinuity',option,
        )
    if same_topology_waypoint(first_wp,second_wp):
        return TopologyConnection(
            True,gap,0.0,0,'same topology waypoint',
            'longitudinal',option,
        )

    if road_changed and junction:
        return TopologyConnection(
            True,gap,gap,1,
            'junction/road connector accepted from contiguous GRP geometry',
            'junction_connector',option,
        )
    if road_changed and grp_trace:
        return TopologyConnection(
            True,gap,gap,1,
            'road connector accepted from the same GRP trace',
            'road_connector',option,
        )

    directed=directed_topology_connection(
        first_wp,second_wp,sampling_resolution_m
    )
    if directed.connected:
        return TopologyConnection(
            True,gap,directed.path_length_m,directed.hops,directed.reason,
            'longitudinal',option,
        )
    if grp_trace and _same_lane_identity(first_wp,second_wp):
        return TopologyConnection(
            True,gap,gap,1,
            'same-lane contiguous geometry accepted from the same GRP trace',
            'longitudinal',option,
        )
    return TopologyConnection(
        False,gap,None,0,directed.reason,'disconnected',option,
    )


def remove_duplicate_waypoints(waypoints,tolerance_m=0.05):
    result=[]
    route_points=any(isinstance(item,RoutePoint) for item in waypoints)
    for waypoint in waypoints:
        if not result or not same_topology_waypoint(
            result[-1],waypoint,tolerance_m
        ):
            result.append(waypoint)
        elif route_points:
            # GRP commonly emits the same waypoint twice when RoadOption changes.
            # Keep the later option while retaining the maneuver on the prior point.
            result[-1]=waypoint
    return result


def merge_route_segments(segments,connection_checker,segment_names,tolerance_m=0.05):
    merged=[]
    issues=[]
    for index,segment in enumerate(segments):
        cleaned=remove_duplicate_waypoints(segment,tolerance_m)
        if not cleaned:
            issues.append(f'{segment_names[index]} is empty after duplicate removal')
            continue
        if merged:
            connection=connection_checker(merged[-1],cleaned[0],False)
            if not connection.connected:
                issues.append(
                    f'{segment_names[index-1]} -> {segment_names[index]} merge '
                    f'boundary gap={connection.gap_m:.3f}m '
                    f'({waypoint_topology_text(merged[-1])} -> '
                    f'{waypoint_topology_text(cleaned[0])}): {connection.reason}'
                )
        for waypoint in cleaned:
            if not merged or not same_topology_waypoint(
                merged[-1],waypoint,tolerance_m
            ):
                merged.append(waypoint)
    return merged,issues


def validate_directed_sequence(
    waypoints,connection_checker,max_continuity_gap_m,label,grp_trace=True
):
    issues=[]
    approved_large_gaps=[]
    for index,(first,second) in enumerate(zip(waypoints[:-1],waypoints[1:])):
        connection=connection_checker(first,second,grp_trace)
        if not connection.connected:
            issues.append(
                f'{label} waypoint {index}->{index+1} '
                f'gap={connection.gap_m:.3f}m '
                f'({waypoint_topology_text(first)} -> '
                f'{waypoint_topology_text(second)}): {connection.reason}'
            )
        elif connection.gap_m>float(max_continuity_gap_m):
            approved_large_gaps.append((index,connection))
    return issues,approved_large_gaps


def waypoint_log_lines(waypoint):
    waypoint=route_waypoint(waypoint)
    transform=waypoint.transform
    return (
        f'  road/section/lane: {waypoint.road_id}/'
        f'{waypoint.section_id}/{waypoint.lane_id}\n'
        f'  x/y/yaw: {transform.location.x:.3f}/'
        f'{transform.location.y:.3f}/{transform.rotation.yaw:.3f}'
    )


def waypoint_topology_text(waypoint):
    waypoint=route_waypoint(waypoint)
    return (
        f'{waypoint.road_id}/{waypoint.section_id}/{waypoint.lane_id} '
        f's={float(waypoint.s):.3f}'
    )


def transition_diagnostic(index,first,second,connection):
    first_point=as_route_point(first)
    second_point=as_route_point(second)
    previous=first_point.waypoint
    following=second_point.waypoint
    return (
        f'transition {index}->{index+1}: '
        f'road_option={road_option_name(first_point)} '
        f'classification={connection.classification} '
        f'validation={"PASS" if connection.connected else "FAIL"}\n'
        f'previous: {waypoint_topology_text(previous)} '
        f'location=({previous.transform.location.x:.3f},'
        f'{previous.transform.location.y:.3f},'
        f'{previous.transform.location.z:.3f}) '
        f'yaw={previous.transform.rotation.yaw:.3f}\n'
        f'next: {waypoint_topology_text(following)} '
        f'location=({following.transform.location.x:.3f},'
        f'{following.transform.location.y:.3f},'
        f'{following.transform.location.z:.3f}) '
        f'yaw={following.transform.rotation.yaw:.3f}\n'
        f'Euclidean gap={connection.gap_m:.3f}m; {connection.reason}'
    )


def find_trigger_segment(segments,cfg,trigger_key):
    trigger=TriggerBox.from_dict(cfg['triggers'][trigger_key])
    for segment_index,segment in enumerate(segments):
        for waypoint_index,waypoint in enumerate(segment):
            waypoint=route_waypoint(waypoint)
            if trigger.contains(waypoint.transform.location):
                return segment_index,waypoint_index
    return None,None


def validate_post_signal_route(
    route,segments,anchors,projected,cfg,connection_checker,
    max_continuity_gap_m,sampling_resolution_m,
):
    signal_key='Q_SIGNAL2'
    segment_index,_=find_trigger_segment(segments,cfg,signal_key)
    trigger_index=first_trigger_indices(route,cfg,[signal_key])[signal_key]
    if segment_index is None or trigger_index is None:
        return [f'{signal_key} post-route validation could not locate trigger'],[],{}

    suffix_issues,approved=validate_directed_sequence(
        route[trigger_index:],connection_checker,max_continuity_gap_m,
        f'{signal_key} -> FINISH',
    )
    anchor_indices={signal_key:trigger_index}
    cursor=trigger_index
    match_limit=max(3.0,3.0*float(sampling_resolution_m))
    for anchor,waypoint in zip(
        anchors[segment_index+1:],projected[segment_index+1:]
    ):
        candidates=[
            (index,waypoint_distance(candidate,waypoint))
            for index,candidate in enumerate(route[cursor:],start=cursor)
            if int(route_waypoint(candidate).road_id)==int(waypoint.road_id)
            and int(route_waypoint(candidate).section_id)==int(waypoint.section_id)
            and int(route_waypoint(candidate).lane_id)==int(waypoint.lane_id)
        ]
        if not candidates:
            suffix_issues.append(
                f'{signal_key} post-route misses anchor {anchor.key} topology '
                f'{waypoint.road_id}/{waypoint.section_id}/{waypoint.lane_id}'
            )
            continue
        index,distance=min(candidates,key=lambda item:item[1])
        if distance>match_limit:
            suffix_issues.append(
                f'{signal_key} post-route anchor {anchor.key} is '
                f'{distance:.3f}m from its projected waypoint'
            )
            continue
        anchor_indices[anchor.key]=index
        cursor=index
    expected=[signal_key]+[
        anchor.key for anchor in anchors[segment_index+1:]
    ]
    if list(anchor_indices)!=expected:
        missing=[key for key in expected if key not in anchor_indices]
        suffix_issues.append(
            f'{signal_key} post-route anchor order validation missing: '
            +', '.join(missing)
        )
    return suffix_issues,approved,anchor_indices


def route_length(waypoints):
    return sum(
        waypoint_distance(first,second)
        for first,second in zip(waypoints[:-1],waypoints[1:])
    )


def route_rows(waypoints):
    rows=[]
    cumulative=0.0
    for index,item in enumerate(waypoints):
        waypoint=route_waypoint(item)
        if index:
            cumulative+=waypoint_distance(waypoints[index-1],waypoint)
        transform=waypoint.transform
        location=transform.location
        values=(
            cumulative,location.x,location.y,location.z,
            transform.rotation.yaw,float(waypoint.s),
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise RuntimeError(f'route contains NaN/Inf at index {index}')
        rows.append({
            'index':index,
            'route_s_m':round(cumulative,6),
            'x':round(float(location.x),6),
            'y':round(float(location.y),6),
            'z':round(float(location.z),6),
            'yaw':round(float(transform.rotation.yaw),6),
            'road_id':int(waypoint.road_id),
            'section_id':int(waypoint.section_id),
            'lane_id':int(waypoint.lane_id),
            'opendrive_s':round(float(waypoint.s),6),
            'is_junction':bool(waypoint.is_junction),
            'road_option':road_option_name(item),
        })
    return rows


def segment_detour_result(name,waypoints,start_waypoint,end_waypoint,max_ratio):
    length=route_length(waypoints)
    anchor_distance=waypoint_distance(start_waypoint,end_waypoint)
    ratio=(length/anchor_distance) if anchor_distance>1e-6 else float('inf')
    return SegmentDetour(
        name,length,anchor_distance,ratio,ratio<=float(max_ratio)
    )


def detect_route_loops(
    waypoints,s_bin_m=5.0,min_route_separation_m=25.0,
    location_tolerance_m=3.0,
):
    if float(s_bin_m)<=0 or float(min_route_separation_m)<=0:
        raise ValueError('loop thresholds must be positive')
    points=[route_waypoint(item) for item in waypoints]
    cumulative=[0.0]
    for first,second in zip(points[:-1],points[1:]):
        cumulative.append(cumulative[-1]+waypoint_distance(first,second))
    visits={}
    issues=[]
    reported=set()
    for index,waypoint in enumerate(points):
        key=(
            int(waypoint.road_id),int(waypoint.section_id),int(waypoint.lane_id),
            math.floor(float(waypoint.s)/float(s_bin_m)),
        )
        previous=visits.get(key)
        if previous is not None:
            previous_index,previous_route_s,previous_location=previous
            separated=(
                cumulative[index]-previous_route_s
                >=float(min_route_separation_m)
            )
            near=(
                previous_location.distance(waypoint.transform.location)
                <=float(location_tolerance_m)
            )
            if separated and near and key not in reported:
                issues.append(
                    'loop/revisit detected at route indices '
                    f'{previous_index}->{index}: road/section/lane/s-bin='
                    f'{key[0]}/{key[1]}/{key[2]}/{key[3]}, '
                    f'route separation='
                    f'{cumulative[index]-previous_route_s:.3f}m'
                )
                reported.add(key)
        visits[key]=(
            index,cumulative[index],waypoint.transform.location
        )
    return issues


def generate_preview_lines(waypoints,z_offset_m=0.15):
    route=[as_route_point(item) for item in waypoints]
    lines=[]
    for first,second in zip(route[:-1],route[1:]):
        option=road_option_name(first)
        color=PREVIEW_COLORS.get(option,PREVIEW_COLORS['LANEFOLLOW'])
        lines.append(PreviewLine(
            first.waypoint.transform.location+carla.Location(z=z_offset_m),
            second.waypoint.transform.location+carla.Location(z=z_offset_m),
            color,option,
        ))
    return lines


def first_trigger_indices(waypoints,cfg,trigger_keys):
    indices={}
    for key in trigger_keys:
        trigger=TriggerBox.from_dict(cfg['triggers'][key])
        indices[key]=next(
             (index for index,waypoint in enumerate(waypoints)
             if trigger.contains(route_waypoint(waypoint).transform.location)),
            None,
        )
    return indices


def validate_required_trigger_order(waypoints,cfg,trigger_keys):
    indices=first_trigger_indices(waypoints,cfg,trigger_keys)
    missing=[key for key in trigger_keys if indices[key] is None]
    if missing:
        raise RuntimeError(
            'generated route misses required triggers: '+', '.join(missing)
        )
    ordered=[indices[key] for key in trigger_keys]
    if ordered!=sorted(ordered) or len(set(ordered))!=len(ordered):
        detail=', '.join(f'{key}@{indices[key]}' for key in trigger_keys)
        raise RuntimeError(
            f'generated route crosses required triggers out of order: {detail}'
        )
    return indices


def _inside_local(local,trigger,tolerance=1e-6):
    x,y,z=local
    return (
        abs(x)<=trigger.extent_x+tolerance
        and abs(y)<=trigger.extent_y+tolerance
        and abs(z)<=trigger.extent_z+tolerance
    )


def calculate_exit_edge(waypoints,trigger):
    locals_=[
        trigger.local_coords(route_waypoint(wp).transform.location)
        for wp in waypoints
    ]
    boundaries=(
        ('+x',0,trigger.extent_x),('-x',0,-trigger.extent_x),
        ('+y',1,trigger.extent_y),('-y',1,-trigger.extent_y),
    )
    for previous,current in zip(locals_[:-1],locals_[1:]):
        if not _inside_local(previous,trigger) or _inside_local(current,trigger):
            continue
        candidates=[]
        for edge,axis,boundary in boundaries:
            delta=current[axis]-previous[axis]
            if abs(delta)<1e-12:
                continue
            fraction=(boundary-previous[axis])/delta
            if not 0.0<=fraction<=1.0:
                continue
            point=tuple(
                previous[i]+fraction*(current[i]-previous[i]) for i in range(3)
            )
            if axis==0:
                valid=(
                    abs(point[1])<=trigger.extent_y+1e-6
                    and abs(point[2])<=trigger.extent_z+1e-6
                )
            else:
                valid=(
                    abs(point[0])<=trigger.extent_x+1e-6
                    and abs(point[2])<=trigger.extent_z+1e-6
                )
            if valid:
                candidates.append((fraction,edge))
        if candidates:
            return min(candidates)[1]
    return None


def recommend_exit_edges(waypoints,cfg,keys=RECOMMENDATION_KEYS):
    recommendations={}
    for key in keys:
        recommendations[key]=calculate_exit_edge(
            waypoints,TriggerBox.from_dict(cfg['triggers'][key])
        )
    return recommendations


def extend_through_terminal_trigger(waypoints,trigger,sampling_resolution_m):
    if calculate_exit_edge(waypoints,trigger) is not None:
        return waypoints
    result=list(waypoints)
    if not trigger.contains(route_waypoint(result[-1]).transform.location):
        raise RuntimeError(
            'route does not reach the FINISH trigger before terminal extension'
        )
    for _ in range(30):
        candidates=[
            waypoint for waypoint in route_waypoint(result[-1]).next(
                float(sampling_resolution_m)
            )
            if waypoint.lane_type==carla.LaneType.Driving
        ]
        if len(candidates)!=1:
            raise RuntimeError(
                'FINISH terminal extension encountered '
                f'{len(candidates)} Driving successors; refusing to choose a branch'
            )
        result.append(
            RoutePoint(candidates[0],'LANEFOLLOW')
            if isinstance(result[-1],RoutePoint) else candidates[0]
        )
        if calculate_exit_edge(result[-2:],trigger) is not None:
            return result
    raise RuntimeError('FINISH terminal extension did not leave the trigger within 30m')


def load_global_route_planner():
    try:
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        return GlobalRoutePlanner
    except ModuleNotFoundError:
        roots=[]
        if os.environ.get('CARLA_ROOT'):
            roots.append(Path(os.environ['CARLA_ROOT']).expanduser())
        roots.extend((Path.home()/'CARLA_0.9.15',Path('/opt/carla-simulator')))
        for root in roots:
            agents_parent=root/'PythonAPI/carla'
            if (agents_parent/'agents/navigation/global_route_planner.py').exists():
                sys.path.insert(0,str(agents_parent))
                from agents.navigation.global_route_planner import GlobalRoutePlanner
                return GlobalRoutePlanner
        raise RuntimeError(
            'CARLA GlobalRoutePlanner is unavailable; set CARLA_ROOT or add '
            'CARLA PythonAPI/carla to PYTHONPATH'
        )


def update_exit_edges_yaml(config_path,recommendations):
    unresolved=[key for key,value in recommendations.items() if value not in EXIT_EDGES]
    if unresolved:
        raise RuntimeError(
            'cannot write unresolved exit edges: '+', '.join(unresolved)
        )
    path=Path(config_path).expanduser().resolve()
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    backup=path.with_name(f'{path.name}.backup_{stamp}')
    shutil.copy2(path,backup)
    lines=path.read_text(encoding='utf-8').splitlines(keepends=True)
    trigger_start=next(
        (i for i,line in enumerate(lines) if line.rstrip()=='triggers:'),None
    )
    scenario_start=next(
        (i for i,line in enumerate(lines) if line.rstrip()=='scenario:'),len(lines)
    )
    if trigger_start is None:
        raise RuntimeError('triggers section not found in YAML')
    for key,edge in recommendations.items():
        header=f'  {key}:'
        start=next(
            (i for i in range(trigger_start+1,scenario_start)
             if lines[i].rstrip()==header),None
        )
        if start is None:
            raise RuntimeError(f'trigger block not found while writing: {key}')
        end=next(
            (i for i in range(start+1,scenario_start)
             if lines[i].startswith('  ') and not lines[i].startswith('    ')
             and lines[i].rstrip().endswith(':')),
            scenario_start,
        )
        existing=next(
            (i for i in range(start+1,end)
             if lines[i].startswith('    exit_edge:')),
            None,
        )
        if existing is not None:
            lines[existing]=f'    exit_edge: {edge}\n'
        else:
            yaw_line=next(
                (i for i in range(start+1,end)
                 if lines[i].startswith('    yaw_deg:')),
                None,
            )
            if yaw_line is None:
                raise RuntimeError(f'yaw_deg not found while writing trigger {key}')
            lines.insert(yaw_line+1,f'    exit_edge: {edge}\n')
            scenario_start+=1
    path.write_text(''.join(lines),encoding='utf-8')
    return backup


class LaneRouteBuilder(Node):
    def __init__(self,cli_write_exit_edges=False):
        super().__init__('lane_route_builder')
        self.declare_parameter('config_file','')
        self.declare_parameter('output_csv','')
        self.declare_parameter('preview',True)
        self.declare_parameter('preview_lifetime_sec',60.0)
        self.declare_parameter('sampling_resolution_m',1.0)
        self.declare_parameter('expected_map','heven_kcity/Maps/kcity/kcity')
        self.declare_parameter('write_exit_edges',False)

        self.config_file=str(self.get_parameter('config_file').value)
        if not self.config_file:
            raise RuntimeError('config_file is required')
        self.cfg=load_yaml(self.config_file)
        validation=self.cfg.get('route_validation',{})
        self.declare_parameter(
            'max_continuity_gap_m',
            float(validation.get('max_continuity_gap_m',3.0)),
        )
        self.declare_parameter(
            'lane_change_max_gap_m',
            float(validation.get('lane_change_max_gap_m',8.0)),
        )
        self.declare_parameter(
            'lane_change_max_yaw_delta_deg',
            float(validation.get('lane_change_max_yaw_delta_deg',45.0)),
        )
        self.declare_parameter(
            'segment_detour_max_ratio',
            float(validation.get('segment_detour_max_ratio',2.5)),
        )
        self.declare_parameter(
            'loop_s_bin_m',float(validation.get('loop_s_bin_m',5.0)),
        )
        self.declare_parameter(
            'loop_min_route_separation_m',
            float(validation.get('loop_min_route_separation_m',25.0)),
        )
        self.sampling=float(self.get_parameter('sampling_resolution_m').value)
        if self.sampling<=0:
            raise RuntimeError('sampling_resolution_m must be positive')
        self.client,self.world=connect_carla(self.cfg.get('carla',{}))
        self.carla_map=self.world.get_map()
        expected=str(self.get_parameter('expected_map').value).strip('/')
        actual=str(self.carla_map.name).strip('/')
        if expected and not actual.endswith(expected):
            raise RuntimeError(f'expected CARLA map {expected!r}, got {actual!r}')
        self.write_exit_edges=(
            bool(cli_write_exit_edges)
            or bool(self.get_parameter('write_exit_edges').value)
        )
        self.build()

    def build(self):
        anchors=parse_route_anchors(self.cfg)
        required=parse_required_triggers(self.cfg)
        generated_waypoints=(
            self.carla_map.generate_waypoints(self.sampling)
            if any(anchor.road_id is not None for anchor in anchors)
            else None
        )
        projected=[]
        for anchor in anchors:
            source=carla.Location(*anchor.location)
            waypoint=project_route_anchor(
                self.carla_map,anchor,generated_waypoints
            )
            distance=source.distance(waypoint.transform.location)
            hint=(
                f', hint=road {anchor.road_id} lane {anchor.lane_id}'
                if anchor.road_id is not None else ''
            )
            self.get_logger().info(
                f'{anchor.key}: projection_distance={distance:.3f}m -> '
                f'({waypoint.transform.location.x:.3f}, '
                f'{waypoint.transform.location.y:.3f}, '
                f'{waypoint.transform.location.z:.3f}), '
                f'road={waypoint.road_id} section={waypoint.section_id} '
                f'lane={waypoint.lane_id}{hint}'
            )
            projected.append(waypoint)

        planner=load_global_route_planner()(self.carla_map,self.sampling)
        segments=[]
        segment_names=[]
        issues=[]
        continuity_limit=float(
            self.get_parameter('max_continuity_gap_m').value
        )
        lane_change_limit=float(
            self.get_parameter('lane_change_max_gap_m').value
        )
        lane_change_yaw_limit=float(
            self.get_parameter('lane_change_max_yaw_delta_deg').value
        )
        detour_limit=float(
            self.get_parameter('segment_detour_max_ratio').value
        )
        loop_s_bin=float(self.get_parameter('loop_s_bin_m').value)
        loop_separation=float(
            self.get_parameter('loop_min_route_separation_m').value
        )
        if min(
            continuity_limit,lane_change_limit,lane_change_yaw_limit,
            detour_limit,loop_s_bin,loop_separation,
        )<=0:
            raise RuntimeError('route validation thresholds must be positive')
        for first_anchor,second_anchor,first_wp,second_wp in zip(
            anchors[:-1],anchors[1:],projected[:-1],projected[1:]
        ):
            segment_name=f'{first_anchor.key} -> {second_anchor.key}'
            trace=planner.trace_route(
                first_wp.transform.location,second_wp.transform.location
            )
            segment=remove_duplicate_waypoints([
                RoutePoint(waypoint,road_option)
                for waypoint,road_option in trace
            ])
            if not segment:
                raise RuntimeError(
                    f'route segment failed: {segment_name}'
                )
            end_distance=route_waypoint(segment[-1]).transform.location.distance(
                second_wp.transform.location
            )
            if end_distance>max(3.0,3.0*self.sampling):
                issues.append(
                    f'route segment {segment_name} '
                    f'ends {end_distance:.2f}m from projected destination'
                )
            detour=segment_detour_result(
                segment_name,segment,first_wp,second_wp,detour_limit
            )
            self.get_logger().info(
                f'{segment_name}\n'
                f'start:\n{waypoint_log_lines(segment[0])}\n'
                f'end:\n{waypoint_log_lines(segment[-1])}\n'
                f'points: {len(segment)}\n'
                f'length_m: {detour.route_length_m:.3f}\n'
                f'anchor_euclidean_m: {detour.anchor_distance_m:.3f}\n'
                f'detour_ratio: {detour.ratio:.3f} '
                f'(limit={detour_limit:.3f}, '
                f'{"PASS" if detour.accepted else "FAIL"})'
            )
            if not detour.accepted:
                issues.append(
                    f'route segment {segment_name} detour ratio '
                    f'{detour.ratio:.3f} exceeds '
                    f'segment_detour_max_ratio={detour_limit:.3f} '
                    f'(length={detour.route_length_m:.3f}m, '
                    f'anchor_distance={detour.anchor_distance_m:.3f}m)'
                )
            segments.append(segment)
            segment_names.append(segment_name)

        finish_trigger=TriggerBox.from_dict(self.cfg['triggers']['FINISH'])
        try:
            segments[-1]=extend_through_terminal_trigger(
                segments[-1],finish_trigger,self.sampling
            )
        except RuntimeError as exc:
            issues.append(str(exc))

        connection_cache={}
        def connection_checker(first,second,grp_trace=True):
            key=(
                waypoint_token(first),waypoint_token(second),
                road_option_name(first),bool(grp_trace),
            )
            if key not in connection_cache:
                connection_cache[key]=validate_route_transition(
                    first,second,self.sampling,continuity_limit,
                    lane_change_limit,lane_change_yaw_limit,
                    grp_trace=grp_trace,
                )
            return connection_cache[key]

        for segment_name,segment in zip(segment_names,segments):
            segment_issues,approved=validate_directed_sequence(
                segment,connection_checker,continuity_limit,segment_name,True
            )
            issues.extend(segment_issues)
            for index,(first,second) in enumerate(zip(segment[:-1],segment[1:])):
                connection=connection_checker(first,second,True)
                first_wp=route_waypoint(first)
                second_wp=route_waypoint(second)
                if (
                    connection.classification in {
                        'lane_change','junction_connector','road_connector'
                    }
                    or int(first_wp.road_id)!=int(second_wp.road_id)
                    or int(first_wp.lane_id)!=int(second_wp.lane_id)
                ):
                    self.get_logger().info(
                        f'{segment_name}\n'
                        +transition_diagnostic(index,first,second,connection)
                    )
            for index,connection in approved:
                self.get_logger().warn(
                    f'{segment_name} waypoint {index}->{index+1}: '
                    f'{connection.gap_m:.3f}m exceeds '
                    f'max_continuity_gap_m={continuity_limit:.3f}m but is '
                    f'accepted as {connection.classification} '
                    f'({connection.reason})'
                )

        route,merge_issues=merge_route_segments(
            segments,connection_checker,segment_names
        )
        issues.extend(merge_issues)
        issues.extend(detect_route_loops(
            route,s_bin_m=loop_s_bin,
            min_route_separation_m=loop_separation,
            location_tolerance_m=continuity_limit,
        ))
        jumps=[waypoint_distance(a,b) for a,b in zip(route[:-1],route[1:])]
        max_jump=max(jumps,default=0.0)
        self.get_logger().info(
            f'route continuity: max_gap={max_jump:.3f}m, '
            f'configured_limit={continuity_limit:.3f}m'
        )
        start_distance=route_waypoint(route[0]).transform.location.distance(
            projected[0].transform.location
        )
        finish_distance=route_waypoint(route[-1]).transform.location.distance(
            projected[-1].transform.location
        )
        if start_distance>max(2.5,2.5*self.sampling):
            issues.append(f'route starts {start_distance:.3f}m from START projection')
        finish_limit=max(
            finish_trigger.extent_x,finish_trigger.extent_y
        )+2.5*self.sampling
        if finish_distance>finish_limit:
            issues.append(f'route ends {finish_distance:.3f}m from FINISH projection')
        trigger_indices=first_trigger_indices(route,self.cfg,required)
        try:
            trigger_indices=validate_required_trigger_order(
                route,self.cfg,required
            )
        except RuntimeError as exc:
            issues.append(str(exc))
        post_issues,post_approved,post_anchor_indices=validate_post_signal_route(
            route,segments,anchors,projected,self.cfg,connection_checker,
            continuity_limit,self.sampling,
        )
        issues.extend(post_issues)
        signal_segment_index,_=find_trigger_segment(
            segments,self.cfg,'Q_SIGNAL2'
        )
        if signal_segment_index is not None:
            self.get_logger().info(
                f'Q_SIGNAL2 lies in segment '
                f'{segment_names[signal_segment_index]}; validating directed '
                f'topology through FINISH'
            )
        if post_anchor_indices:
            self.get_logger().info(
                'Q_SIGNAL2 post-route anchor order: '
                + ' -> '.join(
                    f'{key}@{index}'
                    for key,index in post_anchor_indices.items()
                )
            )
        for index,connection in post_approved:
            self.get_logger().warn(
                f'Q_SIGNAL2 -> FINISH waypoint {index}->{index+1}: '
                f'{connection.gap_m:.3f}m exceeds '
                f'max_continuity_gap_m={continuity_limit:.3f}m but is '
                f'topology-connected ({connection.reason})'
            )
        recommendations=recommend_exit_edges(route,self.cfg)
        unresolved=[key for key,value in recommendations.items() if value is None]
        if unresolved:
            issues.append('no inside-to-outside crossing for: '+', '.join(unresolved))

        for key in required:
            if trigger_indices.get(key) is not None:
                self.get_logger().info(
                    f'{key}: route_index={trigger_indices[key]} validation=PASS'
                )
            else:
                self.get_logger().error(f'{key}: validation=FAIL (not crossed)')
        for key,value in recommendations.items():
            self.get_logger().info(f'{key} -> {value or "UNRESOLVED"}')
        if issues:
            if bool(self.get_parameter('preview').value):
                self.preview(route,anchors,required)
            raise RuntimeError('route validation failed: '+'; '.join(issues))

        rows=route_rows(route)
        benchmark=self.cfg.get('benchmark',{})
        configured=str(benchmark.get('route_csv','routes/qualifier.csv'))
        output=str(self.get_parameter('output_csv').value)
        output_path=(
            Path(output).expanduser()
            if output else resolve_route_csv_path(self.config_file,configured)
        )
        if bool(self.get_parameter('preview').value):
            self.preview(route,anchors,required)
        write_route_csv(output_path,rows)
        self.get_logger().info(
            f'wrote {len(rows)} points, length={rows[-1]["route_s_m"]:.3f}m '
            f'-> {output_path}'
        )
        if self.write_exit_edges:
            backup=update_exit_edges_yaml(self.config_file,recommendations)
            self.get_logger().info(f'exit_edge backup: {backup}')

    def preview(self,route,anchors,required):
        lifetime=float(self.get_parameter('preview_lifetime_sec').value)
        lines=generate_preview_lines(route)
        for line in lines:
            self.world.debug.draw_line(
                line.start,line.end,thickness=0.08,
                color=carla.Color(*line.color),life_time=lifetime,
            )
        self.get_logger().info(
            f'preview: drew {len(lines)} connected route lines; '
            f'world.tick() was not called'
        )
        for anchor in anchors:
            point=carla.Location(*anchor.location)+carla.Location(z=0.4)
            self.world.debug.draw_point(
                point,size=0.18,color=carla.Color(0,80,255),life_time=lifetime
            )
            self.world.debug.draw_string(
                point+carla.Location(z=0.35),anchor.key,
                color=carla.Color(0,80,255),life_time=lifetime,
            )
        for key in required:
            trigger=self.cfg['triggers'][key]
            point=carla.Location(*xyz(trigger['center'],f'trigger {key} center'))
            point+=carla.Location(z=0.4)
            self.world.debug.draw_point(
                point,size=0.2,color=carla.Color(255,255,0),life_time=lifetime
            )
            self.world.debug.draw_string(
                point+carla.Location(z=0.35),key,
                color=carla.Color(255,255,0),life_time=lifetime,
            )


def main(args=None):
    cli_args=list(sys.argv[1:] if args is None else args)
    write_flag='--write-exit-edges' in cli_args
    cli_args=[arg for arg in cli_args if arg!='--write-exit-edges']
    rclpy.init(args=cli_args)
    node=None
    try:
        node=LaneRouteBuilder(cli_write_exit_edges=write_flag)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
