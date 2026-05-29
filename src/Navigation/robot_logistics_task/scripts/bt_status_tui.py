#!/usr/bin/env python3
import json,rclpy
from rclpy.node import Node
from std_msgs.msg import String
class StatusTUI(Node):
    def __init__(self):
        super().__init__('bt_status_tui')
        self.create_subscription(String,'/task/current_state',lambda m: print(f'[STATE] {m.data}'),10)
        self.create_subscription(String,'/task/status',self.on_status,10)
        self.create_subscription(String,'/task/bt_trace',self.on_trace,10)
        self.create_subscription(String,'/task/current_goal',lambda m: print(f'[GOAL] {m.data}'),10)
    def on_status(self,m):
        try: d=json.loads(m.data); print(f"[STATUS] {d.get('status')} state={d.get('state')} msg={d.get('message')}")
        except Exception: print(f'[STATUS] {m.data}')
    def on_trace(self,m):
        try: d=json.loads(m.data); print(f"[TRACE] {d.get('node')} -> {d.get('result')}")
        except Exception: print(f'[TRACE] {m.data}')
def main():
    rclpy.init(); n=StatusTUI(); print('Listening to /task/current_state /task/status /task/bt_trace /task/current_goal ...')
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
