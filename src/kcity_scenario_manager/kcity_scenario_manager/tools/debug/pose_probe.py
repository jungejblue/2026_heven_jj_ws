#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ...competition_common import connect_carla,ego_front_location,find_ego_vehicle
class PoseProbe(Node):
    def __init__(self):
        super().__init__('competition_pose_probe'); self.declare_parameter('host','127.0.0.1'); self.declare_parameter('port',2000); self.declare_parameter('ego_role_name','ego_vehicle')
        self.client,self.world=connect_carla({'host':self.get_parameter('host').value,'port':self.get_parameter('port').value,'timeout_sec':10.0}); self.role=self.get_parameter('ego_role_name').value
        self.timer=self.create_timer(1.0,self.tick)
    def tick(self):
        e=find_ego_vehicle(self.world,self.role)
        if e is None: self.get_logger().warning('ego not found'); return
        t=e.get_transform(); l=t.location; f=ego_front_location(e)
        self.get_logger().info(f'EGO_CENTER x={l.x:.3f} y={l.y:.3f} z={l.z:.3f} yaw={t.rotation.yaw:.3f}')
        self.get_logger().info(f'EGO_FRONT  x={f.x:.3f} y={f.y:.3f} z={f.z:.3f}')
        ls=list(self.world.get_actors().filter('traffic.traffic_light*')); ls.sort(key=lambda a:a.get_location().distance(l))
        for a in ls[:6]:
            p=a.get_location(); self.get_logger().info(f'TL id={a.id} d={p.distance(l):.2f} loc=({p.x:.3f},{p.y:.3f},{p.z:.3f}) state={a.get_state()}')
def main(args=None):
    rclpy.init(args=args); n=PoseProbe()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
