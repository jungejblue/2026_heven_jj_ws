#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import traceback, carla, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .competition_common import TriggerBox,connect_carla,evaluation_location,find_ego_vehicle,get_trigger,json_msg,load_yaml,resolve_traffic_light,sim_time,speed_mps,suite_traffic_light_configs,traffic_state_name

@dataclass
class LightRuntime:
    cfg: dict; trigger: TriggerBox; actor: Optional[carla.TrafficLight]=None
    was_inside: bool=False; prev_eval: Optional[carla.Location]=None; passed: bool=False
    forced: bool=False; force_t: Optional[float]=None; stop_t: Optional[float]=None; released: bool=False; violation_t: Optional[float]=None

class FinalScenarioManager(Node):
    def __init__(self):
        super().__init__('final_scenario_manager'); self.declare_parameter('config_file',''); self.declare_parameter('require_ego_role',False)
        f=self.get_parameter('config_file').value
        if not f: raise RuntimeError('config_file is required')
        self.cfg=load_yaml(f); self.client,self.world=connect_carla(self.cfg.get('carla',{})); self.role=str(self.cfg.get('carla',{}).get('ego_role_name','ego_vehicle')); self.strict_role=bool(self.get_parameter('require_ego_role').value)
        s=self.cfg['scenario']; q=s['final']; self.poll_hz=float(s.get('poll_hz',20)); self.threshold=int(q.get('green_pass_threshold',3))
        self.min_red=float(q.get('min_red_sec',1.0)); self.stop_threshold=float(q.get('stop_speed_threshold_mps',0.10)); self.stop_confirm=float(q.get('required_stopped_sec_before_green',0.50)); self.reset_delay=float(q.get('reset_after_violation_sec',1.0))
        light_cfgs=suite_traffic_light_configs(self.cfg,'final')
        self.lights={k:LightRuntime(x,get_trigger(self.cfg,str(x['trigger_key']),require_exit_edge=True)) for k,x in light_cfgs.items()}
        self.ego=None; self.green_count=0; self.red_guarantee_used=False; self.spawned=[]; self.obstacles_done=False; self.obstacle_cfg=q.get('obstacles',[])
        self.pub=self.create_publisher(String,'/kcity/scenario/events',10); self.timer=self.create_timer(1.0/max(self.poll_hz,1.0),self.tick)

    def event(self,name,**x):
        m=String(); m.data=json_msg(event=name,suite='final',**x); self.pub.publish(m)

    def ensure(self):
        if self.ego is None or not self.ego.is_alive: self.ego=find_ego_vehicle(self.world,self.role,self.strict_role)
        if self.ego is None: return False
        ok=True
        for k,r in self.lights.items():
            if r.actor is None or not r.actor.is_alive:
                try: r.actor=resolve_traffic_light(self.world,r.cfg)
                except Exception as e: self.get_logger().warning(f'{k}: {e}'); ok=False
        if not self.obstacles_done: self.spawn_obstacles()
        return ok

    def spawn_obstacles(self):
        self.obstacles_done=True; lib=self.world.get_blueprint_library()
        for o in self.obstacle_cfg:
            if not bool(o.get('enabled',False)): continue
            bid=str(o.get('blueprint',''))
            if not bid: continue
            try: bp=lib.find(bid)
            except Exception: self.get_logger().warning(f'obstacle blueprint not found: {bid}'); continue
            t=o.get('transform',{}); l=t.get('location',{}); r=t.get('rotation',{})
            tf=carla.Transform(carla.Location(x=float(l.get('x',0)),y=float(l.get('y',0)),z=float(l.get('z',0))),carla.Rotation(roll=float(r.get('roll',0)),pitch=float(r.get('pitch',0)),yaw=float(r.get('yaw',0))))
            a=self.world.try_spawn_actor(bp,tf)
            if a: self.spawned.append(a); self.event('OBSTACLE_SPAWNED',actor_id=a.id,blueprint=bid)

    def maybe_force(self,k,r,now):
        if self.red_guarantee_used or self.green_count<self.threshold or r.passed or r.forced: return
        r.forced=True; r.force_t=now; r.actor.set_state(carla.TrafficLightState.Red); self.red_guarantee_used=True
        self.event('FINAL_FORCE_RED',sim_time=now,light_key=k,green_pass_count=self.green_count)

    def maintain(self,k,r,now,inside):
        if not r.forced or r.released: return
        r.actor.set_state(carla.TrafficLightState.Red)
        if inside and speed_mps(self.ego)<=self.stop_threshold:
            if r.stop_t is None: r.stop_t=now
        else: r.stop_t=None
        if r.stop_t is not None and now-r.stop_t>=self.stop_confirm and r.force_t is not None and now-r.force_t>=self.min_red:
            r.actor.set_state(carla.TrafficLightState.Green); r.released=True; self.event('FINAL_RELEASE_GREEN',sim_time=now,light_key=k)
        if r.violation_t is not None and not r.released and now-r.violation_t>=self.reset_delay:
            r.actor.set_state(carla.TrafficLightState.Green); r.released=True

    def tick(self):
        try:
            if not self.ensure(): return
            now=sim_time(self.world); center=self.ego.get_location()
            for k,r in self.lights.items():
                inside=r.trigger.contains(center); ev=evaluation_location(self.ego,r.trigger); crossed=r.trigger.crossed_exit(r.prev_eval,ev)
                if inside and not r.was_inside: self.maybe_force(k,r,now)
                self.maintain(k,r,now,inside)
                if crossed and not r.passed:
                    st=traffic_state_name(r.actor); r.passed=True
                    if st=='Green': self.green_count+=1
                    if r.forced and not r.released and st=='Red': r.violation_t=now
                    self.event('FINAL_LIGHT_PASS',sim_time=now,light_key=k,state=st,green_pass_count=self.green_count)
                r.was_inside=inside; r.prev_eval=ev
        except Exception as e:
            self.get_logger().error(f'final scenario error: {e}'); self.get_logger().debug(traceback.format_exc())

    def destroy_node(self):
        for a in self.spawned:
            try:
                if a.is_alive: a.destroy()
            except Exception: pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args); n=FinalScenarioManager()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
