
include_directories(.)
set(LIDAR_SRC hc_test.cpp LidarTest.cpp)
add_library(lidar ${LIDAR_SRC})

target_include_directories(lidar
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(WIN32)
else()
    target_link_libraries(lidar PRIVATE base)
endif()