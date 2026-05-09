cmake_minimum_required(VERSION 3.8)
project(target_pose_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(target_pose_msgs REQUIRED)
find_package(yolo_msg REQUIRED)

install(PROGRAMS
  target_pose_bridge/target_3d_fusion_node.py
  target_pose_bridge/target_tf_transform_node.py
  target_pose_bridge/target_position_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
