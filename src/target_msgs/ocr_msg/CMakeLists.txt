cmake_minimum_required(VERSION 3.8)
project(ocr_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OcrRequest.msg"
  "msg/OcrRequestArray.msg"
  "msg/OcrResult.msg"
  "msg/OcrResultArray.msg"
  DEPENDENCIES std_msgs yolo_msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
