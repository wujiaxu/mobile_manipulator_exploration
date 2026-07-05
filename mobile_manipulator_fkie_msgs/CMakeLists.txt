cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_fkie_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundaryPolygon.msg"
  "msg/MeasurementEstimation.msg"
  "msg/MeasurementEstimationValue.msg"
  "action/NbvPlanner.action"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
