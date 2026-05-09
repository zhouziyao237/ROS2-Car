cmake_minimum_required(VERSION 3.8)
project(target_auto_decision)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/auto_decision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
