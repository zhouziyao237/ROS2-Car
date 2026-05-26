cmake_minimum_required(VERSION 3.8)
project(qr_chassis_align_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS qr_chassis_align_control/qr_chassis_align_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_chassis_align_node
)

install(
  DIRECTORY config launch resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
