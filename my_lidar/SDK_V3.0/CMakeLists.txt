project(CAMSENSE_SDK)
cmake_minimum_required(VERSION 3.10)

# 全局设置PIC选项
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-std=c++11) # Use C++11

# 修复路径变量（使用CMAKE_SOURCE_DIR）
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)

add_subdirectory(derive)
add_subdirectory(example)

# 显式指定 pybind11 路径
set(PYBIND11_DIR ${CMAKE_SOURCE_DIR}/pybind11)

# 添加 pybind11 子目录
add_subdirectory(${PYBIND11_DIR})

find_package(Python COMPONENTS Interpreter Development REQUIRED)

pybind11_add_module(sdk_py derive/src/bindings.cpp)

target_link_libraries(sdk_py PRIVATE sdk_lib)
target_include_directories(sdk_py PRIVATE sdk_lib)