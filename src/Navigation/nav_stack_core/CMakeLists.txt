cmake_minimum_required(VERSION 3.8)
project(nav_stack_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclpy REQUIRED)

add_library(nav_stack_mode_manager SHARED
  src/nav_stack_mode_manager.cpp
)

target_include_directories(nav_stack_mode_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav_stack_mode_manager
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

add_executable(nav_stack_mode_manager_node src/nav_stack_mode_manager_node.cpp)
target_link_libraries(nav_stack_mode_manager_node nav_stack_mode_manager)
ament_target_dependencies(nav_stack_mode_manager_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

install(PROGRAMS
  scripts/cmd_vel_mode_mux.py 
  scripts/fake_odom_publisher.py
  scripts/odom_tf_broadcaster.py
  scripts/goal_pose_bridge.py
  scripts/serial_imu_bridge.py
  scripts/wheel_twist_odom.py
  scripts/front_scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  nav_stack_mode_manager
  nav_stack_mode_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz description
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nav_stack_mode_manager)
ament_package()
