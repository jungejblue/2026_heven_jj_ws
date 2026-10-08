#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass
import json, math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple
import carla, yaml

def load_yaml(path: str) -> Dict[str, Any]:
    with open(Path(path).expanduser(), 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

@dataclass
class TriggerBox:
    center_x: float; center_y: float; center_z: float
    extent_x: float; extent_y: float; extent_z: float
    yaw_deg: float = 0.0
    enabled: bool = False
    exit_edge: str = ''
    evaluation_point: str = 'front_bumper'

    @classmethod
    def from_dict(cls, data):
        data = data or {}; c=data.get('center',{}); e=data.get('extent',{})
        return cls(float(c.get('x',0)),float(c.get('y',0)),float(c.get('z',0)),
                   float(e.get('x',0)),float(e.get('y',0)),float(e.get('z',0)),
                   float(data.get('yaw_deg',0)), bool(data.get('enabled',False)),
                   str(data.get('exit_edge','')).lower(),
                   str(data.get('evaluation_point','front_bumper')).lower())

    def local_coords(self, loc):
        dx,dy,dz=loc.x-self.center_x,loc.y-self.center_y,loc.z-self.center_z
        y=math.radians(self.yaw_deg)
        return dx*math.cos(y)+dy*math.sin(y), -dx*math.sin(y)+dy*math.cos(y), dz

    def contains(self, loc):
        if not self.enabled: return False
        x,y,z=self.local_coords(loc)
        return abs(x)<=self.extent_x and abs(y)<=self.extent_y and abs(z)<=self.extent_z

    def _edge(self,x,y):
        if self.exit_edge=='+x': return x,self.extent_x,y,self.extent_y
        if self.exit_edge=='-x': return -x,self.extent_x,y,self.extent_y
        if self.exit_edge=='+y': return y,self.extent_y,x,self.extent_x
        if self.exit_edge=='-y': return -y,self.extent_y,x,self.extent_x
        raise ValueError(f'Unsupported exit_edge={self.exit_edge}')

    def exit_crossing_fraction(self, prev_loc, curr_loc, tolerance_m=0.05):
        """Return the fraction of a sample interval at a forward exit-edge crossing."""
        if not self.enabled or prev_loc is None: return None
        px,py,pz=self.local_coords(prev_loc); cx,cy,cz=self.local_coords(curr_loc)
        ps,limit,plat,latlim=self._edge(px,py); cs,_,clat,_=self._edge(cx,cy)
        if not (ps < limit-tolerance_m and cs >= limit-tolerance_m): return None
        ds=cs-ps
        if abs(ds)<1e-9: return None
        a=max(0.0,min(1.0,(limit-ps)/ds))
        lat=plat+a*(clat-plat); z=pz+a*(cz-pz)
        if abs(lat)<=latlim+tolerance_m and abs(z)<=self.extent_z+tolerance_m:
            return a
        return None

    def crossed_exit(self, prev_loc, curr_loc, tolerance_m=0.05):
        return self.exit_crossing_fraction(prev_loc, curr_loc, tolerance_m) is not None

def get_trigger(cfg,key,require_exit_edge=False):
    if key not in cfg.get('triggers',{}): raise KeyError(f"Trigger '{key}' missing in YAML")
    trigger=TriggerBox.from_dict(cfg['triggers'][key])
    if require_exit_edge and trigger.exit_edge not in {'+x','-x','+y','-y'}:
        raise RuntimeError(
            f"Trigger '{key}' requires an explicit exit_edge (+x, -x, +y, or -y)"
        )
    return trigger

def normalize_traffic_light_configs(raw):
    """Return traffic-light config as {light_key: config}.

    The dictionary form is canonical. The legacy list form remains accepted so
    existing final configs continue to load during their later migration.
    """
    if raw is None:
        return {}
    if isinstance(raw, Mapping):
        items=raw.items()
    elif isinstance(raw, list):
        items=[]
        for index,item in enumerate(raw):
            if not isinstance(item, Mapping):
                raise TypeError(f'traffic_lights[{index}] must be a mapping')
            key=str(item.get('key','')).strip()
            if not key:
                raise ValueError(f'traffic_lights[{index}] is missing key')
            items.append((key,item))
    else:
        raise TypeError('traffic_lights must be a mapping or list')

    result={}
    for raw_key,item in items:
        key=str(raw_key).strip()
        if not key:
            raise ValueError('traffic light key must not be empty')
        if not isinstance(item, Mapping):
            raise TypeError(f"traffic light '{key}' must be a mapping")
        cfg=dict(item)
        configured_key=str(cfg.get('key',key)).strip()
        if configured_key != key:
            raise ValueError(
                f"traffic light key mismatch: container={key}, config={configured_key}"
            )
        cfg['key']=key
        if key in result:
            raise ValueError(f"duplicate traffic light key: {key}")
        result[key]=cfg
    return result

def suite_traffic_light_configs(cfg,suite):
    section=cfg.get('scenario',{}).get(str(suite),{})
    raw=section.get('traffic_lights')
    if raw is None and str(suite)=='qualifier' and section.get('traffic_light'):
        raw=[section['traffic_light']]
    return normalize_traffic_light_configs(raw)

def connect_carla(cfg):
    c=carla.Client(str(cfg.get('host','127.0.0.1')),int(cfg.get('port',2000)))
    c.set_timeout(float(cfg.get('timeout_sec',10.0))); return c,c.get_world()

def find_ego_vehicle(world, role_name, strict=False):
    vs=list(world.get_actors().filter('vehicle.*'))
    for v in vs:
        if v.attributes.get('role_name','')==role_name: return v
    return vs[0] if not strict and len(vs)==1 else None

def ego_front_location(ego):
    tf=ego.get_transform(); bb=ego.bounding_box; f=tf.get_forward_vector(); r=tf.get_right_vector()
    fx=float(bb.location.x+bb.extent.x); sy=float(bb.location.y)
    return carla.Location(x=tf.location.x+f.x*fx+r.x*sy,
                          y=tf.location.y+f.y*fx+r.y*sy,
                          z=tf.location.z+float(bb.location.z))

def evaluation_location(ego, trigger):
    return ego.get_location() if trigger.evaluation_point=='center' else ego_front_location(ego)

def speed_mps(actor):
    v=actor.get_velocity(); return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)

def resolve_traffic_light(world,cfg):
    ls=list(world.get_actors().filter('traffic.traffic_light*'))
    if not ls: raise RuntimeError('No CARLA traffic-light actors found')
    aid=cfg.get('actor_id')
    if aid not in (None,'',0,'0'):
        try:
            actor_id=int(aid)
        except (TypeError,ValueError) as exc:
            raise RuntimeError(f'invalid traffic light actor_id={aid!r}') from exc
        for l in ls:
            if l.id==actor_id: return l
        raise RuntimeError(f'traffic light actor_id={aid} not found')
    q=cfg.get('location')
    if not isinstance(q,Mapping) or any(k not in q for k in ('x','y','z')):
        raise RuntimeError('traffic light location requires explicit x, y, and z')
    target=carla.Location(x=float(q['x']),y=float(q['y']),z=float(q['z']))
    rad=float(cfg.get('match_radius_m',5.0))
    if rad<=0: raise RuntimeError(f'traffic light match_radius_m must be positive: {rad}')
    candidates=[]
    for light in ls:
        distance=float(light.get_location().distance(target))
        if distance<=rad:
            candidates.append((distance,light))
    candidates.sort(key=lambda item:(item[0],item[1].id))
    target_text=f'({target.x:.3f}, {target.y:.3f}, {target.z:.3f})'
    if not candidates:
        nearest=min(float(x.get_location().distance(target)) for x in ls)
        raise RuntimeError(
            f'no traffic light within {rad:.2f}m of {target_text}; nearest={nearest:.2f}m'
        )
    if len(candidates)>1:
        details=', '.join(f'id={light.id} distance={distance:.2f}m' for distance,light in candidates)
        raise RuntimeError(
            f'ambiguous traffic light match within {rad:.2f}m of {target_text}: {details}'
        )
    return candidates[0][1]

def traffic_state_name(light): return str(light.get_state()).split('.')[-1]
def sim_time(world): return float(world.get_snapshot().timestamp.elapsed_seconds)
def json_msg(**kwargs): return json.dumps(kwargs,ensure_ascii=False)
