cmake_minimum_required(VERSION 3.8)
project(robot2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_core_topics.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
