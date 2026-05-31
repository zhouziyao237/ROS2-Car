#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import tf2_ros


def yaw_to_quat(yaw: float):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def parse_json(text: str) -> Dict:
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


class FrontierExplorerNode(Node):
    """
    Semantic frontier explorer.

    用途：
      当任务目标货架不在 semantic_library.json 里时，
      mission_btcpp_node 会进入 SearchShelfReference 状态。
      本节点监听 /task/status、/task/shelf_goal、/task/explore_cmd，
      然后周期性发布 /goal_pose，让 Nav2 带机器人在当前位置附近探索。

    停止条件：
      1. semantic_library.json 中已经出现目标 shelf_id；
      2. 行为树进入 NavigateToShelf / WaitNavArrive / PublishTaskDone；
      3. 收到 FAILED / DONE / IDLE 等终止状态。
    """

    def __init__(self):
        super().__init__("frontier_explorer_node")

        self.declare_parameter("task_status_topic", "/task/status")
        self.declare_parameter("task_goal_topic", "/task/shelf_goal")
        self.declare_parameter("explore_cmd_topic", "/task/explore_cmd")
        self.declare_parameter("semantic_observation_topic", "/vision/semantic_observation")
        self.declare_parameter("goal_pose_topic", "/goal_pose")

        self.declare_parameter("semantic_map_path", "/host-home/maps/semantic_library.json")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("enabled", True)

        # 探索目标发布周期。不要太短，否则会频繁打断 Nav2。
        self.declare_parameter("publish_period_sec", 4.0)

        # 每次探索目标离当前机器人多远。
        self.declare_parameter("goal_distance_m", 1.2)

        # 左右侧向探索距离。
        self.declare_parameter("side_distance_m", 0.7)

        # 扇形探索角度。
        self.declare_parameter("yaw_step_deg", 45.0)

        # 避免重复发布太近的目标。
        self.declare_parameter("min_goal_separation_m", 0.35)

        self.declare_parameter("debug_log", True)

        self.active = False
        self.target_shelf_id = ""
        self.target_cell_id = ""
        self.goal_index = 0
        self.last_goal_xy: Optional[Tuple[float, float]] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("goal_pose_topic").value),
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/task/explore_status",
            10,
        )

        self.task_status_sub = self.create_subscription(
            String,
            str(self.get_parameter("task_status_topic").value),
            self.on_task_status,
            10,
        )

        self.task_goal_sub = self.create_subscription(
            String,
            str(self.get_parameter("task_goal_topic").value),
            self.on_task_goal,
            10,
        )

        self.explore_cmd_sub = self.create_subscription(
            String,
            str(self.get_parameter("explore_cmd_topic").value),
            self.on_explore_cmd,
            10,
        )

        self.semantic_obs_sub = self.create_subscription(
            String,
            str(self.get_parameter("semantic_observation_topic").value),
            self.on_semantic_observation,
            10,
        )

        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(max(0.5, period), self.on_timer)

        self.get_logger().info(
            "frontier_explorer_node started: "
            f"task_status={self.get_parameter('task_status_topic').value}, "
            f"task_goal={self.get_parameter('task_goal_topic').value}, "
            f"goal_pose={self.get_parameter('goal_pose_topic').value}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def debug(self, msg: str):
        if bool(self.get_parameter("debug_log").value):
            self.get_logger().info(msg)

    def publish_status(self, state: str, message: str):
        msg = String()
        msg.data = json.dumps(
            {
                "node": "frontier_explorer_node",
                "state": state,
                "target_shelf_id": self.target_shelf_id,
                "target_cell_id": self.target_cell_id,
                "active": self.active,
                "message": message,
                "stamp": self.now_sec(),
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)

    def extract_task_target(self, data: Dict):
        shelf_id = (
            data.get("shelf_id")
            or data.get("target_shelf_id")
            or ""
        )

        cell_id = (
            data.get("cell_id")
            or data.get("target_cell_id")
            or ""
        )

        if not shelf_id and cell_id:
            shelf_id = str(cell_id).split("-")[0]

        if shelf_id:
            self.target_shelf_id = str(shelf_id)

        if cell_id:
            self.target_cell_id = str(cell_id)

    def on_task_goal(self, msg: String):
        data = parse_json(msg.data)
        self.extract_task_target(data)

        # 新任务到来时，先不立即 active。
        # active 由 SearchShelfReference / EnterExploreMode 状态触发。
        self.goal_index = 0
        self.last_goal_xy = None

        self.debug(
            f"task goal received: target_shelf={self.target_shelf_id}, "
            f"cell={self.target_cell_id}"
        )

    def on_explore_cmd(self, msg: String):
        data = parse_json(msg.data)
        self.extract_task_target(data)

        self.active = True
        self.goal_index = 0
        self.last_goal_xy = None

        self.publish_status("ACTIVE", "explore command received")
        self.debug(
            f"explore cmd received: target_shelf={self.target_shelf_id}, raw={msg.data}"
        )

    def on_task_status(self, msg: String):
        data = parse_json(msg.data)
        state = str(data.get("state", ""))
        status = str(data.get("status", ""))
        message = str(data.get("message", ""))

        if state in ["EnterExploreMode", "SearchShelfReference"]:
            if not self.active:
                self.debug(f"activate exploration by task state: {state}")
            self.active = True
            return

        # 导航阶段、完成阶段、失败阶段停止探索。
        stop_states = [
            "NavigateToShelf",
            "WaitNavArrive",
            "VerifyShelfQR",
            "AlignToCell",
            "SendPlatformTarget",
            "WaitPickupDone",
            "PublishTaskDone",
            "DONE",
            "IDLE",
            "FAILED",
        ]

        if state in stop_states:
            if self.active:
                self.debug(f"stop exploration by task state: {state}")
            self.active = False
            self.publish_status("STOPPED", f"task state={state}, status={status}, msg={message}")

    def stop_exploration_and_hold(self, reason: str):
        """
        Stop exploration immediately.

        Important:
          Just setting active=False only prevents future exploration goals.
          Nav2 may still be executing the last exploration goal.
          So we also publish a hold goal at the robot's current map pose.
        """
        was_active = self.active
        self.active = False

        pose = self.get_robot_pose()
        if pose is not None:
            rx, ry, ryaw = pose
            hold = PoseStamped()
            hold.header.stamp = self.get_clock().now().to_msg()
            hold.header.frame_id = str(self.get_parameter("map_frame").value)
            hold.pose.position.x = rx
            hold.pose.position.y = ry
            hold.pose.position.z = 0.0
            qz, qw = yaw_to_quat(ryaw)
            hold.pose.orientation.z = qz
            hold.pose.orientation.w = qw
            self.goal_pub.publish(hold)

        self.publish_status("FOUND_STOP", reason)

        if was_active:
            self.get_logger().info(f"target found, stop exploration and hold current pose: {reason}")
        else:
            self.get_logger().info(f"target found while inactive: {reason}")

    def on_semantic_observation(self, msg: String):
        """
        Stop exploration as soon as the target shelf QR/semantic observation appears.
        """
        if not self.active:
            return

        if not self.target_shelf_id:
            return

        data = parse_json(msg.data)

        obs_shelf = (
            data.get("shelf_id")
            or data.get("shelf")
            or data.get("target_shelf_id")
            or ""
        )

        if str(obs_shelf) == str(self.target_shelf_id):
            self.stop_exploration_and_hold(
                f"target shelf observed from semantic observation: {self.target_shelf_id}"
            )

    def semantic_has_target(self) -> bool:
        if not self.target_shelf_id:
            return False

        p = Path(str(self.get_parameter("semantic_map_path").value))
        if not p.exists():
            return False

        try:
            data = json.loads(p.read_text())
        except Exception:
            return False

        shelf = data.get("shelves", {}).get(self.target_shelf_id)
        if not isinstance(shelf, dict):
            return False

        pose = shelf.get("approach_pose") or shelf.get("current_map_pose")
        if isinstance(pose, dict) and bool(pose.get("valid", False)):
            return True

        return False

    def get_robot_pose(self):
        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(f"cannot lookup {map_frame}->{base_frame}: {repr(e)}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return x, y, yaw

    def make_goal(self, rx: float, ry: float, gx: float, gy: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)

        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0

        goal_yaw = math.atan2(gy - ry, gx - rx)
        qz, qw = yaw_to_quat(goal_yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def goal_far_enough(self, gx: float, gy: float) -> bool:
        if self.last_goal_xy is None:
            return True

        lx, ly = self.last_goal_xy
        dist = math.hypot(gx - lx, gy - ly)
        return dist >= float(self.get_parameter("min_goal_separation_m").value)

    def on_timer(self):
        if not bool(self.get_parameter("enabled").value):
            return

        if not self.active:
            return

        # 如果目标货架已经进入语义库，停止探索。
        # 后续由 mission_bt 的 QueryShelfPose / NavigateToShelf 接管。
        if self.semantic_has_target():
            self.stop_exploration_and_hold(
                f"target shelf {self.target_shelf_id} exists in semantic library"
            )
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry, ryaw = pose

        forward = float(self.get_parameter("goal_distance_m").value)
        side = float(self.get_parameter("side_distance_m").value)
        yaw_step = math.radians(float(self.get_parameter("yaw_step_deg").value))

        # 探索模式：
        # 前方、左前、右前、左转前方、右转前方、更远左前、更远右前。
        pattern = [
            (forward, 0.0, 0.0),
            (forward * 0.8, side, 0.0),
            (forward * 0.8, -side, 0.0),
            (forward, 0.0, yaw_step),
            (forward, 0.0, -yaw_step),
            (forward * 1.3, side * 0.7, yaw_step),
            (forward * 1.3, -side * 0.7, -yaw_step),
        ]

        fx, fy, dyaw = pattern[self.goal_index % len(pattern)]
        self.goal_index += 1

        gyaw = ryaw + dyaw
        gx = rx + fx * math.cos(gyaw) - fy * math.sin(gyaw)
        gy = ry + fx * math.sin(gyaw) + fy * math.cos(gyaw)

        if not self.goal_far_enough(gx, gy):
            return

        goal = self.make_goal(rx, ry, gx, gy)
        self.goal_pub.publish(goal)
        self.last_goal_xy = (gx, gy)

        self.publish_status(
            "GOAL_PUBLISHED",
            f"explore goal x={gx:.2f}, y={gy:.2f}"
        )

        self.debug(
            f"publish explore goal: shelf={self.target_shelf_id}, "
            f"x={gx:.2f}, y={gy:.2f}, robot=({rx:.2f},{ry:.2f}), idx={self.goal_index}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
