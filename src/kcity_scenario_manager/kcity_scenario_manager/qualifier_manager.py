#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import traceback

import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .competition_common import (
    TriggerBox,
    connect_carla,
    evaluation_location,
    find_ego_vehicle,
    get_trigger,
    json_msg,
    load_yaml,
    resolve_traffic_light,
    sim_time,
    speed_mps,
    suite_traffic_light_configs,
    traffic_state_name,
)


@dataclass
class QualifierLightRuntime:
    cfg: dict
    trigger: TriggerBox
    actor: Optional[carla.TrafficLight] = None
    was_inside: bool = False
    prev_eval: Optional[carla.Location] = None
    forced: bool = False
    force_t: Optional[float] = None
    stop_t: Optional[float] = None
    released: bool = False
    violation_t: Optional[float] = None
    resolution_error: Optional[str] = None


class QualifierScenarioManager(Node):
    def __init__(self):
        super().__init__('qualifier_scenario_manager')
        self.declare_parameter('config_file','')
        self.declare_parameter('require_ego_role',False)
        config_file=self.get_parameter('config_file').value
        if not config_file:
            raise RuntimeError('config_file is required')

        self.cfg=load_yaml(config_file)
        self.client,self.world=connect_carla(self.cfg.get('carla',{}))
        self.role=str(self.cfg.get('carla',{}).get('ego_role_name','ego_vehicle'))
        self.strict_role=bool(self.get_parameter('require_ego_role').value)
        scenario=self.cfg['scenario']
        qualifier=scenario['qualifier']
        self.poll_hz=float(scenario.get('poll_hz',20))
        light_cfgs=suite_traffic_light_configs(self.cfg,'qualifier')
        if not light_cfgs:
            raise RuntimeError('scenario.qualifier.traffic_lights must define at least one light')
        self.lights={
            key:QualifierLightRuntime(
                cfg,
                get_trigger(self.cfg,str(cfg['trigger_key']),require_exit_edge=True),
            )
            for key,cfg in light_cfgs.items()
        }

        self.force_red=bool(qualifier.get('force_red_on_trigger',True))
        self.min_red=float(qualifier.get('min_red_sec',1.0))
        self.stop_threshold=float(qualifier.get('stop_speed_threshold_mps',0.10))
        self.stop_confirm=float(qualifier.get('required_stopped_sec_before_green',0.50))
        self.reset_delay=float(qualifier.get('reset_after_violation_sec',1.0))
        self.ego=None
        self.pub=self.create_publisher(String,'/kcity/scenario/events',10)
        self.timer=self.create_timer(1.0/max(self.poll_hz,1.0),self.tick)

    def event(self,name,light_key,**fields):
        message=String()
        message.data=json_msg(
            event=name,
            suite='qualifier',
            light_key=light_key,
            **fields,
        )
        self.pub.publish(message)

    def ensure_actors(self):
        if self.ego is None or not self.ego.is_alive:
            self.ego=find_ego_vehicle(self.world,self.role,self.strict_role)
        if self.ego is None:
            return False

        for key,runtime in self.lights.items():
            if runtime.actor is not None and runtime.actor.is_alive:
                continue
            try:
                runtime.actor=resolve_traffic_light(self.world,runtime.cfg)
            except RuntimeError as exc:
                message=str(exc)
                if runtime.resolution_error!=message:
                    self.get_logger().warning(f'{key}: {message}')
                    runtime.resolution_error=message
                continue
            runtime.resolution_error=None
            location=runtime.actor.get_location()
            self.get_logger().info(
                f'{key} -> actor_id={runtime.actor.id}, '
                f'location=({location.x:.3f}, {location.y:.3f}, {location.z:.3f})'
            )
        return True

    def process_light(self,key,runtime,now,center):
        inside=runtime.trigger.contains(center)
        eval_location=evaluation_location(self.ego,runtime.trigger)
        crossed=runtime.trigger.crossed_exit(runtime.prev_eval,eval_location)

        if inside and not runtime.was_inside and self.force_red and not runtime.forced:
            runtime.forced=True
            runtime.force_t=now
            runtime.actor.set_state(carla.TrafficLightState.Red)
            self.event(
                'QUALIFIER_FORCE_RED',key,sim_time=now,light_id=runtime.actor.id
            )

        if runtime.forced and not runtime.released:
            runtime.actor.set_state(carla.TrafficLightState.Red)
            if inside and speed_mps(self.ego)<=self.stop_threshold:
                if runtime.stop_t is None:
                    runtime.stop_t=now
            else:
                runtime.stop_t=None

            stopped=(
                runtime.stop_t is not None
                and now-runtime.stop_t>=self.stop_confirm
            )
            exposed=(
                runtime.force_t is not None
                and now-runtime.force_t>=self.min_red
            )
            if stopped and exposed:
                runtime.actor.set_state(carla.TrafficLightState.Green)
                runtime.released=True
                self.event(
                    'QUALIFIER_RELEASE_GREEN',key,
                    sim_time=now,light_id=runtime.actor.id,
                )

        if crossed:
            state=traffic_state_name(runtime.actor)
            self.event(
                'QUALIFIER_TRIGGER_EXIT',key,
                sim_time=now,state=state,light_id=runtime.actor.id,
            )
            if runtime.forced and not runtime.released and state=='Red':
                runtime.violation_t=now

        if (
            runtime.violation_t is not None
            and not runtime.released
            and now-runtime.violation_t>=self.reset_delay
        ):
            runtime.actor.set_state(carla.TrafficLightState.Green)
            runtime.released=True

        runtime.was_inside=inside
        runtime.prev_eval=eval_location

    def tick(self):
        try:
            if not self.ensure_actors():
                return
            now=sim_time(self.world)
            center=self.ego.get_location()
            for key,runtime in self.lights.items():
                if runtime.actor is None or not runtime.actor.is_alive:
                    continue
                self.process_light(key,runtime,now,center)
        except Exception as exc:
            self.get_logger().error(f'qualifier scenario error: {exc}')
            self.get_logger().debug(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node=QualifierScenarioManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
