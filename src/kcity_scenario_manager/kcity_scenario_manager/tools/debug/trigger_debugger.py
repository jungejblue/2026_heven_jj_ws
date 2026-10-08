#!/usr/bin/env python3
import math, carla, rclpy
from rclpy.node import Node
from ...competition_common import connect_carla,load_yaml,TriggerBox
class TriggerDebugger(Node):
    def __init__(self):
        super().__init__('trigger_debugger'); self.declare_parameter('config_file',''); f=self.get_parameter('config_file').value
        if not f: raise RuntimeError('config_file is required')
        self.cfg=load_yaml(f); self.client,self.world=connect_carla(self.cfg.get('carla',{})); self.triggers={k:TriggerBox.from_dict(v) for k,v in self.cfg.get('triggers',{}).items()}; self.timer=self.create_timer(1.0,self.tick)
    def edge(self,k,t):
        y=math.radians(t.yaw_deg); f=(math.cos(y),math.sin(y)); r=(-math.sin(y),math.cos(y))
        if t.exit_edge=='+x': c=(t.center_x+f[0]*t.extent_x,t.center_y+f[1]*t.extent_x); d=r; h=t.extent_y
        elif t.exit_edge=='-x': c=(t.center_x-f[0]*t.extent_x,t.center_y-f[1]*t.extent_x); d=r; h=t.extent_y
        elif t.exit_edge=='+y': c=(t.center_x+r[0]*t.extent_y,t.center_y+r[1]*t.extent_y); d=f; h=t.extent_x
        else: c=(t.center_x-r[0]*t.extent_y,t.center_y-r[1]*t.extent_y); d=f; h=t.extent_x
        z=t.center_z+0.2; p1=carla.Location(x=c[0]-d[0]*h,y=c[1]-d[1]*h,z=z); p2=carla.Location(x=c[0]+d[0]*h,y=c[1]+d[1]*h,z=z)
        self.world.debug.draw_line(p1,p2,thickness=0.15,color=carla.Color(255,0,0),life_time=1.2)
    def tick(self):
        for k,t in self.triggers.items():
            if not t.enabled: continue
            b=carla.BoundingBox(carla.Location(x=t.center_x,y=t.center_y,z=t.center_z),carla.Vector3D(t.extent_x,t.extent_y,t.extent_z))
            self.world.debug.draw_box(b,carla.Rotation(yaw=t.yaw_deg),thickness=0.05,color=carla.Color(0,255,255),life_time=1.2)
            self.world.debug.draw_string(carla.Location(x=t.center_x,y=t.center_y,z=t.center_z+t.extent_z+0.5),k,color=carla.Color(255,255,255),life_time=1.2); self.edge(k,t)
def main(args=None):
    rclpy.init(args=args); n=TriggerDebugger()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
