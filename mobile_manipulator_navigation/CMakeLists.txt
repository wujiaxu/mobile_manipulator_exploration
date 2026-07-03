cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(wrist_depth_octomap_node src/wrist_depth_octomap_node.cpp)
target_link_libraries(wrist_depth_octomap_node ${OCTOMAP_LIBRARIES})
target_include_directories(wrist_depth_octomap_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(
  wrist_depth_octomap_node
  geometry_msgs
  octomap_msgs
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  TARGETS wrist_depth_octomap_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
