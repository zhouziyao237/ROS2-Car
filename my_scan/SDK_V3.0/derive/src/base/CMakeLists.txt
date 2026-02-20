
include_directories(.)
set(LIDAR_SRC hchead.cpp hclidar.cpp lidar.cpp ReadParsePackage.cpp HC_serial.cpp HcSDK.cpp)
add_library(base ${LIDAR_SRC})

if(WIN32)
else()
    # target_link_libraries(base PRIVATE sdk)
    target_include_directories(base
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    )
endif()