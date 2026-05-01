cmake_minimum_required(VERSION 3.8)
project(ros_astra_s_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/rgbd_position_node.py
  scripts/depth_calibration_node.py
  scripts/target_pixel_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  calib
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
