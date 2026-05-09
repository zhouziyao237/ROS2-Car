#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class TargetTFTransformNode(Node):
    def __init__(self):
        super().__init__('target_tf_transform_node')
        self.declare_parameter('input_topic', '/target_pose_camera')
        self.declare_parameter('output_topic', '/target_pose_base')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('debug_log', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointStamped, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(PointStamped, self.output_topic, 10)

        self.get_logger().info('target_tf_transform_node started')
        self.get_logger().info(f'{self.input_topic} -> {self.output_topic}, target_frame={self.target_frame}')

    def callback(self, msg: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            out = do_transform_point(msg, transform)
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.target_frame
            self.pub.publish(out)
            if self.debug_log:
                self.get_logger().info(
                    f'target {self.target_frame}: '
                    f'x={out.point.x*1000.0:.1f} y={out.point.y*1000.0:.1f} z={out.point.z*1000.0:.1f} mm'
                )
        except Exception as e:
            self.get_logger().warn(f'Failed to transform from {msg.header.frame_id} to {self.target_frame}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetTFTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
