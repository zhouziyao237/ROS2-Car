project(CAMSENSE_SDK)
cmake_minimum_required (VERSION 2.8)
add_definitions(-std=c++11) # Use C++11

# 修复路径变量（使用CMAKE_SOURCE_DIR）
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/lib CACHE PATH "Build directory" FORCE)

add_subdirectory(derive)
add_subdirectory(example)