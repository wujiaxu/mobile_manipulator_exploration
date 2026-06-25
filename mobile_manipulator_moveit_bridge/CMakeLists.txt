cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_moveit_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(trajectory_sampler src/trajectory_sampler.cpp)
target_include_directories(trajectory_sampler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_sampler trajectory_msgs)

add_executable(isaac_trajectory_bridge src/isaac_trajectory_bridge.cpp)
target_link_libraries(isaac_trajectory_bridge trajectory_sampler)
ament_target_dependencies(
  isaac_trajectory_bridge
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(pose_goal_planner src/pose_goal_planner.cpp)
ament_target_dependencies(
  pose_goal_planner
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2_geometry_msgs
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS trajectory_sampler isaac_trajectory_bridge pose_goal_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_sampler test/test_trajectory_sampler.cpp)
  target_link_libraries(test_trajectory_sampler trajectory_sampler)
  ament_target_dependencies(test_trajectory_sampler trajectory_msgs)
endif()

ament_package()
