project (example)

add_executable(sdk_V3_demo.os sdk_V3_demo.cpp) 
target_link_libraries(sdk_V3_demo.os PUBLIC sdk_lib)

add_executable(sdk_scanData.os sdk_scanData.cpp)
target_link_libraries(sdk_scanData.os PUBLIC sdk_lib)

add_executable(sdk_demo.os sdk_demo.cpp)
target_link_libraries(sdk_demo.os PUBLIC sdk_lib)