cmake_minimum_required(VERSION 3.8)
project(target_qr_matcher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS target_qr_matcher/qr_matcher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME qr_matcher_node
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()