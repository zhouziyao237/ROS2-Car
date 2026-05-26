cmake_minimum_required(VERSION 3.8)
project(qr_platform_lift_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS qr_platform_lift_control/qr_lift_control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_lift_control_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
