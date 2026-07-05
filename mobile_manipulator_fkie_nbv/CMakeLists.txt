cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_fkie_nbv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mobile_manipulator_fkie_msgs REQUIRED)
find_package(mobile_manipulator_moveit_bridge REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(fkie_nbv_planner_node
  src/fkie_nbv_planner_node.cpp
  src/rrt_node.cpp
)
target_link_libraries(fkie_nbv_planner_node ${OCTOMAP_LIBRARIES})
target_include_directories(fkie_nbv_planner_node
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(
  fkie_nbv_planner_node
  geometry_msgs
  mobile_manipulator_fkie_msgs
  octomap
  octomap_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(send_fkie_nbv_goal_client src/send_fkie_nbv_goal_client.cpp)
ament_target_dependencies(
  send_fkie_nbv_goal_client
  geometry_msgs
  mobile_manipulator_fkie_msgs
  rclcpp
  rclcpp_action
)

add_executable(nbv_arm_target_adapter src/nbv_arm_target_adapter.cpp)
ament_target_dependencies(
  nbv_arm_target_adapter
  geometry_msgs
  mobile_manipulator_fkie_msgs
  mobile_manipulator_moveit_bridge
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  TARGETS
    fkie_nbv_planner_node
    nbv_arm_target_adapter
    send_fkie_nbv_goal_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
