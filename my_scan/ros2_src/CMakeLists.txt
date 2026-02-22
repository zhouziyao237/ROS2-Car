cmake_minimum_required(VERSION 3.10)
project(ros2_lidar_driver)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 查找依赖包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 设置输出路径
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)

# SDK 路径设置
set(SDKV3_PATH ${CMAKE_SOURCE_DIR}/../SDK_V3.0)
if(EXISTS ${SDKV3_PATH})
    message(STATUS "found sdk for ${SDKV3_PATH}")
else()
    message(FATAL_ERROR "not found sdk for ${SDKV3_PATH}")
endif()

# 编译 SDK 库
add_subdirectory(${SDKV3_PATH}/derive ${CMAKE_BINARY_DIR}/derive)

# 创建接口库
add_library(ROS2LIBS INTERFACE)
target_link_libraries(ROS2LIBS INTERFACE
    sdk_lib
    rclcpp::rclcpp
    ${sensor_msgs_LIBRARIES}
)
target_include_directories(ROS2LIBS INTERFACE
    sdk_lib
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

set(PYBIND11_DIR ${SDKV3_PATH}/pybind11)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

add_subdirectory(${PYBIND11_DIR} ${PYBIND11_DIR})

pybind11_add_module(sdk_py ${SDKV3_PATH}/derive/src/bindings.cpp)

target_link_libraries(sdk_py PRIVATE sdk_lib)
target_include_directories(sdk_py PRIVATE sdk_lib)