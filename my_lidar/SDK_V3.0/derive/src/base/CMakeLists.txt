project(base)

set(LIDAR_SRC hchead.cpp hclidar.cpp lidar.cpp ReadParsePackage.cpp HC_serial.cpp HcSDK.cpp)
add_library(base_lib ${LIDAR_SRC})
target_include_directories(base_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set_property(TARGET base_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(base_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)