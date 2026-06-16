#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Check(Node):
    def __init__(self):
        super().__init__("check_oak_geometry_target")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.semantic = None
        self.create_subscription(String, "/semantic/current_shelf", self.on_semantic, 10)
        self.timer = self.create_timer(1.0, self.tick)

    def on_semantic(self, msg):
        try:
            self.semantic = json.loads(msg.data)
        except Exception:
            pass

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "oak-cam-Link", rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception as e:
            print("[TF ERROR]", e)
            return

        ox = tf.transform.translation.x
        oy = tf.transform.translation.y

        q = tf.transform.rotation
        yaw_x = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        print("=" * 60)
        print(f"OAK position in map: x={ox:.3f}, y={oy:.3f}")
        print(f"OAK +X yaw in map: {yaw_x:.3f} rad / {math.degrees(yaw_x):.1f} deg")

        if self.semantic:
            sx = self.semantic.get("x", self.semantic.get("shelf_x", None))
            sy = self.semantic.get("y", self.semantic.get("shelf_y", None))

            pose = self.semantic.get("pose", {})
            pos = pose.get("position", {}) if isinstance(pose, dict) else {}

            if sx is None:
                sx = pos.get("x", None)
            if sy is None:
                sy = pos.get("y", None)

            if sx is not None and sy is not None:
                sx = float(sx)
                sy = float(sy)
                target = math.atan2(sy - oy, sx - ox)
                err = normalize(target - yaw_x)
                print(f"Shelf point: x={sx:.3f}, y={sy:.3f}")
                print(f"Target yaw from OAK to shelf point: {target:.3f} rad / {math.degrees(target):.1f} deg")
                print(f"Yaw error using OAK +X: {err:.3f} rad / {math.degrees(err):.1f} deg")
            else:
                print("Semantic shelf received, but no x/y found.")
        else:
            print("No /semantic/current_shelf received yet.")


def main():
    rclpy.init()
    node = Check()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
