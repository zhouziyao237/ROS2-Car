cmake_minimum_required(VERSION 3.8)
project(astra_s_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- OpenNI2 SDK detection ----
# Default SDK layout used in this project:
#   /host-home/Software/OpenNi2/sdk/include/OpenNI.h
#   /host-home/Software/OpenNi2/sdk/libs/libOpenNI2.so
#
# You can override at build time:
#   export OPENNI2_INCLUDE=/your/OpenNi2/sdk/include
#   export OPENNI2_REDIST=/your/OpenNi2/sdk/libs

find_path(OPENNI2_INCLUDE_DIR
  NAMES OpenNI.h
  PATHS
    $ENV{OPENNI2_INCLUDE}
    /host-home/Software/OpenNi2/sdk/include
    /host-home/Software/OpenNi2/sdk/includes
    /usr/include/openni2
    /usr/include/ni
    /usr/include
)

find_library(OPENNI2_LIBRARY
  NAMES OpenNI2
  PATHS
    $ENV{OPENNI2_REDIST}
    /host-home/Software/OpenNi2/sdk/libs
    /usr/lib
    /usr/lib/x86_64-linux-gnu
)

if(NOT OPENNI2_INCLUDE_DIR)
  message(FATAL_ERROR "OpenNI2 include dir not found. Set OPENNI2_INCLUDE or check /host-home/Software/OpenNi2/sdk/include")
endif()

if(NOT OPENNI2_LIBRARY)
  message(FATAL_ERROR "OpenNI2 library not found. Set OPENNI2_REDIST or check /host-home/Software/OpenNi2/sdk/libs")
endif()

message(STATUS "OPENNI2_INCLUDE_DIR=${OPENNI2_INCLUDE_DIR}")
message(STATUS "OPENNI2_LIBRARY=${OPENNI2_LIBRARY}")

add_executable(astra_s_driver_node
  src/astra_s_driver_node.cpp
)

target_include_directories(astra_s_driver_node PRIVATE
  ${OPENNI2_INCLUDE_DIR}
)

ament_target_dependencies(astra_s_driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(astra_s_driver_node
  ${OPENNI2_LIBRARY}
)

install(TARGETS
  astra_s_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rgbd_locator_node.py
  scripts/depth_calib_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  calib
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
