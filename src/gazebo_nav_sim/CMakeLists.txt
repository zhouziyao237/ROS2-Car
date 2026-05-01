cmake_minimum_required(VERSION 3.8)
project(gazebo_nav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_tag_zone_publisher.py
  scripts/sim_cmd_vel_mode_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
