#!/usr/bin/env python3
import argparse,json,time,rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Sender(Node):
    def __init__(self,topic): super().__init__('send_shelf_goal'); self.pub=self.create_publisher(String,topic,10)
    def send(self,data):
        end=time.monotonic()+0.5
        while time.monotonic()<end and rclpy.ok(): rclpy.spin_once(self,timeout_sec=0.05)
        self.pub.publish(String(data=data)); self.get_logger().info(f'published: {data}')
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--topic',default='/task/shelf_goal'); ap.add_argument('--shelf'); ap.add_argument('--row'); ap.add_argument('--col'); ap.add_argument('--cell'); ap.add_argument('--task-type',default='pickup'); ap.add_argument('--x',type=float); ap.add_argument('--y',type=float); ap.add_argument('--yaw',type=float,default=0.0); ap.add_argument('--raw')
    a=ap.parse_args()
    if a.raw: payload=a.raw
    else:
        if not a.shelf: raise SystemExit('--shelf is required unless --raw is used')
        d={'task_type':a.task_type,'shelf_id':a.shelf}
        if a.row is not None:
            try: d['row']=int(a.row)
            except Exception: d['row']=a.row
        if a.col is not None: d['col']=a.col
        if a.cell is not None: d['cell_id']=a.cell
        elif 'row' in d and 'col' in d: d['cell_id']=f"{a.shelf}-L{d['row']}-{d['col']}"
        if a.x is not None and a.y is not None: d.update({'x':a.x,'y':a.y,'yaw':a.yaw})
        payload=json.dumps(d,ensure_ascii=False)
    rclpy.init(); n=Sender(a.topic)
    try: n.send(payload)
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
