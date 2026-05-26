cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/DetectionConfidenceArray.msg"
  "msg/RobotState.msg"
  "srv/ChangeMode.srv"
  "srv/SolveIK.srv"
  "srv/SolveFK.srv"
  "action/ExecutePickPlace.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
