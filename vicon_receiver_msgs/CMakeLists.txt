cmake_minimum_required(VERSION 3.8)
project(vicon_receiver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# One ViconFrame per Vicon frame; the other messages are its parts
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ViconFrame.msg"
  "msg/SubjectState.msg"
  "msg/SegmentState.msg"
  "msg/MarkerState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
