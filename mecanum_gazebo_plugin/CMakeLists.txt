cmake_minimum_required(VERSION 3.8)
project(mecanum_gazebo_plugin)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MecanumCmd.msg"
  DEPENDENCIES std_msgs
)

# Gazebo 插件库
add_library(mecanum_drive_plugin SHARED src/mecanum_drive_plugin.cpp)
target_include_directories(mecanum_drive_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mecanum_drive_plugin
  rclcpp
  gazebo_ros
)
target_link_libraries(mecanum_drive_plugin
  ${GAZEBO_LIBRARIES}
)
# 链接自定义消息库
rosidl_target_interfaces(mecanum_drive_plugin ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装插件库
install(TARGETS mecanum_drive_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装插件描述文件
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

# 安装消息目录
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()