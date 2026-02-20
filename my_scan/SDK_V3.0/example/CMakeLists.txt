project (demo)

add_executable(sdk_V3_demo.elf sdk_V3_demo.cpp)
target_link_libraries(sdk_V3_demo.elf PRIVATE lidar base)

add_executable(sdk_scanData.elf sdk_scanData.cpp)
target_link_libraries(sdk_scanData.elf PRIVATE lidar base)