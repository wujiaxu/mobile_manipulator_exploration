#ifndef MOBILE_MANIPULATOR_FKIE_NBV__NBV_PARAMETERS_HPP_
#define MOBILE_MANIPULATOR_FKIE_NBV__NBV_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mobile_manipulator_fkie_nbv
{

struct NbvParameters
{
  int num_frontiers = 10;
  int frontiers_N_max_tries = 50;
  int max_num_cached_points_to_keep = 50;
  int max_branch_nodes = 8;
  double min_gain_frontier = -10000.0;
  bool free_space_frontier = true;

  int planner_boundary_id = 0;
  double arm_height_min = 0.15;
  double arm_height_max = 1.4;
  double self_collision_height = 2.0;
  double arm_goal_range = 1.3;
  double raycast_dr = 0.1;
  double raycast_dphi = 10.0;
  double raycast_dtheta = 10.0;
  double gain_r_min = 0.3;
  double gain_r_max = 1.5;
  double camera_hfov = 86.0;
  double camera_vfov = 57.0;
  double visited_cells_grid_size = 1.0;
  double min_distance_to_obstacle = 0.3;
  bool compute_yaw_from_free_space = true;
  bool compute_yaw_from_measurement = false;

  int path_optimizer_sampling_target_count = 1;
  double shift_points_max_length = 2.0;
  double shift_points_threshold = 0.9;
  bool optimize_path_using_convex_hull = false;
  bool optimize_path_using_sampling = false;
  bool optimize_path_remove_points_inside_bbx = true;
  bool optimize_path_shift_points_towards_obstacles = false;

  int rrt_tree_length = 600;
  int rrt_N_max_tries = 50;
  double rrt_sampling_radius = 30.0;
  double rrt_collision_cyl_radius = 0.3;
  double rrt_sample_threshold_distance = 0.4;
  double rrt_step_size = 0.5;
  bool sample_in_unknown = false;
  double empty_octomap_resolution = 0.05;
  bool do_rrt_star = false;
  int random_seed = 7;

  double utility_lambda = 5.0;
  double utility_min_gain = 0.0;
  double utility_weight_measurement = 0.0;
  double utility_weight_free_space = 5.0;
  double utility_weight_visited_cell = 500.0;
  double utility_weight_euclidean_cost = 0.5;
  double utility_max_value_free_space = 1.70;
  double utility_max_value_measurement = 500.0;
  double utility_max_value_visited_cell = 1.0;

  std::vector<double> planner_boundary_x;
  std::vector<double> planner_boundary_y;
  std::string world_frame = "map";
  std::string robot_sampling_frame = "base_link";

  static NbvParameters declare_and_load(rclcpp::Node & node)
  {
    NbvParameters parameters;

    parameters.num_frontiers = node.declare_parameter<int>(
      "num_frontiers", parameters.num_frontiers);
    parameters.frontiers_N_max_tries = node.declare_parameter<int>(
      "frontiers_N_max_tries", parameters.frontiers_N_max_tries);
    parameters.max_num_cached_points_to_keep = node.declare_parameter<int>(
      "max_num_cached_points_to_keep", parameters.max_num_cached_points_to_keep);
    parameters.max_branch_nodes = node.declare_parameter<int>(
      "max_branch_nodes", parameters.max_branch_nodes);
    parameters.min_gain_frontier = node.declare_parameter<double>(
      "min_gain_frontier", parameters.min_gain_frontier);
    parameters.free_space_frontier = node.declare_parameter<bool>(
      "free_space_frontier", parameters.free_space_frontier);

    parameters.planner_boundary_id = node.declare_parameter<int>(
      "planner_boundary_id", parameters.planner_boundary_id);
    parameters.arm_height_min = node.declare_parameter<double>(
      "arm_height_min", parameters.arm_height_min);
    parameters.arm_height_max = node.declare_parameter<double>(
      "arm_height_max", parameters.arm_height_max);
    parameters.self_collision_height = node.declare_parameter<double>(
      "self_collision_height", parameters.self_collision_height);
    parameters.arm_goal_range = node.declare_parameter<double>(
      "arm_goal_range", parameters.arm_goal_range);
    parameters.raycast_dr = node.declare_parameter<double>(
      "raycast_dr", parameters.raycast_dr);
    parameters.raycast_dphi = node.declare_parameter<double>(
      "raycast_dphi", parameters.raycast_dphi);
    parameters.raycast_dtheta = node.declare_parameter<double>(
      "raycast_dtheta", parameters.raycast_dtheta);
    parameters.gain_r_min = node.declare_parameter<double>(
      "gain_r_min", parameters.gain_r_min);
    parameters.gain_r_max = node.declare_parameter<double>(
      "gain_r_max", parameters.gain_r_max);
    parameters.camera_hfov = node.declare_parameter<double>(
      "camera_hfov", parameters.camera_hfov);
    parameters.camera_vfov = node.declare_parameter<double>(
      "camera_vfov", parameters.camera_vfov);
    parameters.visited_cells_grid_size = node.declare_parameter<double>(
      "visited_cells_grid_size", parameters.visited_cells_grid_size);
    parameters.min_distance_to_obstacle = node.declare_parameter<double>(
      "min_distance_to_obstacle", parameters.min_distance_to_obstacle);
    parameters.compute_yaw_from_free_space = node.declare_parameter<bool>(
      "compute_yaw_from_free_space", parameters.compute_yaw_from_free_space);
    parameters.compute_yaw_from_measurement = node.declare_parameter<bool>(
      "compute_yaw_from_measurement", parameters.compute_yaw_from_measurement);

    parameters.path_optimizer_sampling_target_count = node.declare_parameter<int>(
      "path_optimizer_sampling_target_count", parameters.path_optimizer_sampling_target_count);
    parameters.shift_points_max_length = node.declare_parameter<double>(
      "shift_points_max_length", parameters.shift_points_max_length);
    parameters.shift_points_threshold = node.declare_parameter<double>(
      "shift_points_threshold", parameters.shift_points_threshold);
    parameters.optimize_path_using_convex_hull = node.declare_parameter<bool>(
      "optimize_path_using_convex_hull", parameters.optimize_path_using_convex_hull);
    parameters.optimize_path_using_sampling = node.declare_parameter<bool>(
      "optimize_path_using_sampling", parameters.optimize_path_using_sampling);
    parameters.optimize_path_remove_points_inside_bbx = node.declare_parameter<bool>(
      "optimize_path_remove_points_inside_bbx",
      parameters.optimize_path_remove_points_inside_bbx);
    parameters.optimize_path_shift_points_towards_obstacles = node.declare_parameter<bool>(
      "optimize_path_shift_points_towards_obstacles",
      parameters.optimize_path_shift_points_towards_obstacles);

    parameters.rrt_tree_length = node.declare_parameter<int>(
      "rrt_tree_length", parameters.rrt_tree_length);
    parameters.rrt_N_max_tries = node.declare_parameter<int>(
      "rrt_N_max_tries", parameters.rrt_N_max_tries);
    parameters.rrt_sampling_radius = node.declare_parameter<double>(
      "rrt_sampling_radius", parameters.rrt_sampling_radius);
    parameters.rrt_collision_cyl_radius = node.declare_parameter<double>(
      "rrt_collision_cyl_radius", parameters.rrt_collision_cyl_radius);
    parameters.rrt_sample_threshold_distance = node.declare_parameter<double>(
      "rrt_sample_threshold_distance", parameters.rrt_sample_threshold_distance);
    parameters.rrt_step_size = node.declare_parameter<double>(
      "rrt_step_size", parameters.rrt_step_size);
    parameters.sample_in_unknown = node.declare_parameter<bool>(
      "sample_in_unknown", parameters.sample_in_unknown);
    parameters.empty_octomap_resolution = node.declare_parameter<double>(
      "empty_octomap_resolution", parameters.empty_octomap_resolution);
    parameters.do_rrt_star = node.declare_parameter<bool>(
      "do_rrt_star", parameters.do_rrt_star);
    parameters.random_seed = node.declare_parameter<int>(
      "random_seed", parameters.random_seed);

    parameters.utility_lambda = node.declare_parameter<double>(
      "utility_lambda", parameters.utility_lambda);
    parameters.utility_min_gain = node.declare_parameter<double>(
      "utility_min_gain", parameters.utility_min_gain);
    parameters.utility_weight_measurement = node.declare_parameter<double>(
      "utility_weight_measurement", parameters.utility_weight_measurement);
    parameters.utility_weight_free_space = node.declare_parameter<double>(
      "utility_weight_free_space", parameters.utility_weight_free_space);
    parameters.utility_weight_visited_cell = node.declare_parameter<double>(
      "utility_weight_visited_cell", parameters.utility_weight_visited_cell);
    parameters.utility_weight_euclidean_cost = node.declare_parameter<double>(
      "utility_weight_euclidean_cost", parameters.utility_weight_euclidean_cost);
    parameters.utility_max_value_free_space = node.declare_parameter<double>(
      "utility_max_value_free_space", parameters.utility_max_value_free_space);
    parameters.utility_max_value_measurement = node.declare_parameter<double>(
      "utility_max_value_measurement", parameters.utility_max_value_measurement);
    parameters.utility_max_value_visited_cell = node.declare_parameter<double>(
      "utility_max_value_visited_cell", parameters.utility_max_value_visited_cell);

    parameters.planner_boundary_x = node.declare_parameter<std::vector<double>>(
      "planner_boundary_x", parameters.planner_boundary_x);
    parameters.planner_boundary_y = node.declare_parameter<std::vector<double>>(
      "planner_boundary_y", parameters.planner_boundary_y);
    parameters.world_frame = node.declare_parameter<std::string>(
      "world_frame", parameters.world_frame);
    parameters.robot_sampling_frame = node.declare_parameter<std::string>(
      "robot_sampling_frame", parameters.robot_sampling_frame);

    return parameters;
  }
};

}  // namespace mobile_manipulator_fkie_nbv

#endif  // MOBILE_MANIPULATOR_FKIE_NBV__NBV_PARAMETERS_HPP_
