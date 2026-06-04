#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np

try:
    import zxingcpp
    HAS_ZXING = True
except:
    HAS_ZXING = False

class OakDataMatrixPnP(Node):
    def __init__(self):
        super().__init__('oak_datamatrix_pnp')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.depth_image = None
        self.rgb_image = None
        self.camera_matrix = None

        # 参数
        self.tag_size = 0.05  # 标签物理尺寸（米）
        self.publish_tf = True
        self.show_debug = True

        # 订阅 RGB, Depth, CameraInfo
        self.create_subscription(Image, '/oak/rgb/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, '/oak/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/oak/rgb/camera_info', self.caminfo_cb, 10)

        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

    def rgb_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def caminfo_cb(self, msg):
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        self.camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])

    def process_frame(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_matrix is None or not HAS_ZXING:
            return

        img = self.rgb_image.copy()
        debug_img = img.copy()
        results = zxingcpp.read_barcodes(img)

        for r in results:
            text = getattr(r, "text", "")
            pts = getattr(r, "position", None)
            if pts is None:
                continue

            # 安全提取四个角点
            try:
                corners = [
                    (pts.top_left.x, pts.top_left.y),
                    (pts.top_right.x, pts.top_right.y),
                    (pts.bottom_right.x, pts.bottom_right.y),
                    (pts.bottom_left.x, pts.bottom_left.y)
                ]
            except AttributeError:
                continue  # 如果 Position 不完整就跳过

            # PnP 位姿
            obj_pts = np.array([[-self.tag_size/2, -self.tag_size/2,0],
                                [ self.tag_size/2, -self.tag_size/2,0],
                                [ self.tag_size/2,  self.tag_size/2,0],
                                [-self.tag_size/2,  self.tag_size/2,0]], dtype=np.float32)
            img_pts = np.array(corners, dtype=np.float32)

            retval, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix, None)
            if not retval:
                continue

            # 转换 tvec 米 -> mm 并确保标量
            tvec_mm = tvec.flatten() * 1000
            x, y, z = tvec_mm[0], tvec_mm[1], tvec_mm[2]
            distance_mm = np.linalg.norm(tvec_mm)

            # 打印位置信息
            self.get_logger().info(f"Tag {text}: X={x:.1f}mm Y={y:.1f}mm Z={z:.1f}mm Distance={distance_mm:.1f}mm")

            # 发布 TF
            if self.publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'oak_link'
                tf_msg.child_frame_id = f'tag_{text}'
                tf_msg.transform.translation.x = float(x)
                tf_msg.transform.translation.y = float(y)
                tf_msg.transform.translation.z = float(z)
                rot_matrix,_ = cv2.Rodrigues(rvec)
                qx,qy,qz,qw = self.rot_to_quaternion(rot_matrix)
                tf_msg.transform.rotation.x = qx
                tf_msg.transform.rotation.y = qy
                tf_msg.transform.rotation.z = qz
                tf_msg.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf_msg)

            # Debug 可视化
            if self.show_debug:
                pts_int = np.array(corners, dtype=np.int32).reshape((-1,1,2))
                cv2.polylines(debug_img, [pts_int], True, (0,255,0), 2)
                cv2.putText(debug_img, text, (int(corners[0][0]), int(corners[0][1])-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        if self.show_debug:
            cv2.imshow("DataMatrix Debug", debug_img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rclpy.shutdown()

    def rot_to_quaternion(self, R):
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1]-R[1,2])/(4*qw)
        qy = (R[0,2]-R[2,0])/(4*qw)
        qz = (R[1,0]-R[0,1])/(4*qw)
        return [qx,qy,qz,qw]

def main(args=None):
    rclpy.init(args=args)
    node = OakDataMatrixPnP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()