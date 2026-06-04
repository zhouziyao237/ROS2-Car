cmake_minimum_required(VERSION 3.8)
project(target_matcher_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msg REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OcrLine.msg"
  "msg/OcrCache.msg"
  "msg/TrackedTarget.msg"
  "msg/TrackedTargetArray.msg"
  "msg/MatchedTarget.msg"
  DEPENDENCIES builtin_interfaces std_msgs yolo_msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
