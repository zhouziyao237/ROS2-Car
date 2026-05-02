project(lidar)

set(LIDAR_SRC hc_test.cpp LidarTest.cpp)
add_library(lidar_lib ${LIDAR_SRC})
target_include_directories(lidar_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lidar_lib PUBLIC base_lib)

set_property(TARGET lidar_lib PROPERTY POSITION_INDEPENDENT_CODE ON)