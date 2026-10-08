#!/usr/bin/env python3
from pathlib import Path
import math, yaml, rclpy
from rclpy.node import Node
from ...competition_common import connect_carla,find_ego_vehicle
class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder'); self.declare_parameter('host','127.0.0.1'); self.declare_parameter('port',2000); self.declare_parameter('ego_role_name','ego_vehicle'); self.declare_parameter('spacing_m',1.0); self.declare_parameter('output_file','~/.ros/heven_carla/recorded_route.yaml')
        self.client,self.world=connect_carla({'host':self.get_parameter('host').value,'port':self.get_parameter('port').value,'timeout_sec':10.0}); self.role=self.get_parameter('ego_role_name').value; self.spacing=float(self.get_parameter('spacing_m').value); self.output=Path(str(self.get_parameter('output_file').value)).expanduser(); self.points=[]; self.timer=self.create_timer(0.05,self.tick)
    def tick(self):
        e=find_ego_vehicle(self.world,self.role)
        if e is None:return
        l=e.get_location()
        if not self.points or math.hypot(l.x-self.points[-1][0],l.y-self.points[-1][1])>=self.spacing:self.points.append([float(l.x),float(l.y)])
    def save(self):
        self.output.parent.mkdir(parents=True,exist_ok=True); self.output.write_text(yaml.safe_dump({'route_points':self.points},sort_keys=False),encoding='utf-8'); self.get_logger().info(f'saved {len(self.points)} points -> {self.output}')
def main(args=None):
    rclpy.init(args=args); n=RouteRecorder()
    try:rclpy.spin(n)
    except KeyboardInterrupt:n.save()
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
