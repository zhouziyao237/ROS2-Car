cmake_minimum_required(VERSION 3.8)
project(mecanum_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MecanumCmd.msg"
  DEPENDENCIES std_msgs
)

add_library(mecanum_drive_plugin SHARED src/mecanum_drive_plugin.cpp)
target_compile_features(mecanum_drive_plugin PUBLIC cxx_std_17)
target_include_directories(mecanum_drive_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mecanum_drive_plugin
  rclcpp
  gazebo_ros  
  yaml_cpp_vendor
)

# 链接 Gazebo 库
target_link_libraries(mecanum_drive_plugin ${GAZEBO_LIBRARIES})

# 链接自定义消息的 typesupport 库（推荐新写法）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mecanum_drive_plugin "${cpp_typesupport_target}")

add_executable(unified_odometry src/unified_odometry.cpp)
target_compile_features(unified_odometry PUBLIC cxx_std_17)

ament_target_dependencies(unified_odometry
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target_odom ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(unified_odometry "${cpp_typesupport_target_odom}")

install(TARGETS mecanum_drive_plugin unified_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()