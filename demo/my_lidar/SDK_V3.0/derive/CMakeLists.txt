project(sdk)

include_directories(sdk_lib INTERFACE
    ${PROJECT_SOURCE_DIR}/dep
    ${PROJECT_SOURCE_DIR}/dep/eigen3
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib
)

add_subdirectory(src)

add_library(sdk_lib INTERFACE)
target_link_libraries(sdk_lib INTERFACE base_lib lidar_lib)
target_include_directories(sdk_lib INTERFACE ${PROJECT_SOURCE_DIR}/include)