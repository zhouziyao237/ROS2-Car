cmake_minimum_required(VERSION 3.8)
project(lidar_odom_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scan_match_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
