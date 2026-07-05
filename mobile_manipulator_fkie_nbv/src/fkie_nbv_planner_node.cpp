#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <map>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp"
#include "mobile_manipulator_fkie_nbv/nanoflann.hpp"
#include "mobile_manipulator_fkie_nbv/nbv_grid.hpp"
#include "mobile_manipulator_fkie_nbv/nbv_parameters.hpp"
#include "mobile_manipulator_fkie_nbv/nbv_utils.hpp"
#include "mobile_manipulator_fkie_nbv/rrt_node.hpp"
#include "mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mobile_manipulator_fkie_nbv
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kOccupancyMarkerResolution = 0.25;

double deg_to_rad(const double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Quaternion optical_z_yaw_to_quaternion(const double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // ROS optical frames look along +Z. Keep optical +Y down and rotate +Z in the map XY plane.
  tf2::Matrix3x3 rotation(
    sin_yaw, 0.0, cos_yaw,
    -cos_yaw, 0.0, sin_yaw,
    0.0, -1.0, 0.0);
  tf2::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = quaternion.x();
  orientation.y = quaternion.y();
  orientation.z = quaternion.z();
  orientation.w = quaternion.w();
  return orientation;
}
}  // namespace

class FkieNbvPlannerNode : public rclcpp::Node
{
public:
  using NbvPlanner = mobile_manipulator_fkie_msgs::action::NbvPlanner;
  using GoalHandleNbvPlanner = rclcpp_action::ServerGoalHandle<NbvPlanner>;
  using KdTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<double, TreeNanoflannAdapter>,
    TreeNanoflannAdapter,
    3>;

  FkieNbvPlannerNode()
  : Node("fkie_nbv_planner"),
    action_name_(declare_parameter<std::string>("action_name", "nbv_rrt")),
    camera_pose_topic_(declare_parameter<std::string>("camera_pose_topic", "/camera_pose")),
    octomap_topic_(declare_parameter<std::string>("octomap_topic", "/octomap_full")),
    rrt_marker_topic_(declare_parameter<std::string>("rrt_marker_topic", "/fkie_nbv/rrt_markers")),
    robot_footprint_topic_(
      declare_parameter<std::string>(
        "robot_footprint_topic", "/mobile_manipulator_mbf/global_costmap/footprint")),
    max_known_free_marker_voxels_(
      declare_parameter<int>("max_known_free_marker_voxels", 5000)),
    max_known_occupied_marker_voxels_(
      declare_parameter<int>("max_known_occupied_marker_voxels", 50000)),
    max_unknown_marker_voxels_(
      declare_parameter<int>("max_unknown_marker_voxels", 10000)),
    parameters_(NbvParameters::declare_and_load(*this)),
    random_engine_(static_cast<std::mt19937::result_type>(parameters_.random_seed)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    current_utility_max_value_free_space_(parameters_.utility_max_value_free_space),
    current_utility_max_value_measurement_(parameters_.utility_max_value_measurement),
    current_utility_max_value_visited_cell_(parameters_.utility_max_value_visited_cell)
  {
    validate_parameters();
    reset_tree();

    camera_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      camera_pose_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        latest_camera_pose_ = message;
        visited_grid_.add_value(PositionGrid(message->pose), VisitedValue(true));
      });
    octomap_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&FkieNbvPlannerNode::octomap_callback, this, std::placeholders::_1));
    robot_footprint_subscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      robot_footprint_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const geometry_msgs::msg::PolygonStamped::SharedPtr message) {
        latest_robot_footprint_ = message;
        current_robot_footprint_ = message->polygon;
      });
    rrt_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      rrt_marker_topic_, rclcpp::QoS(1).transient_local().reliable());

    action_server_ = rclcpp_action::create_server<NbvPlanner>(
      this,
      action_name_,
      std::bind(&FkieNbvPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FkieNbvPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FkieNbvPlannerNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Started FKIE NBV action server '%s' with RRT planner",
      action_name_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (parameters_.arm_height_max < parameters_.arm_height_min) {
      throw std::runtime_error("arm_height_max must be >= arm_height_min");
    }
    if (parameters_.rrt_tree_length < 1) {
      throw std::runtime_error("rrt_tree_length must be >= 1");
    }
    if (parameters_.max_branch_nodes < 0) {
      throw std::runtime_error("max_branch_nodes must be >= 0");
    }
    if (max_known_free_marker_voxels_ < 0 ||
      max_known_occupied_marker_voxels_ < 0 ||
      max_unknown_marker_voxels_ < 0)
    {
      throw std::runtime_error("FKIE marker voxel limits must be >= 0");
    }
    if (parameters_.rrt_N_max_tries < 1) {
      throw std::runtime_error("rrt_N_max_tries must be >= 1");
    }
    if (parameters_.rrt_step_size <= 0.0) {
      throw std::runtime_error("rrt_step_size must be positive");
    }
    if (parameters_.rrt_sample_threshold_distance >= parameters_.rrt_step_size) {
      RCLCPP_WARN(
        get_logger(),
        "rrt_sample_threshold_distance should be smaller than rrt_step_size");
    }
    if (parameters_.raycast_dr <= 0.0 ||
      parameters_.raycast_dphi <= 0.0 ||
      parameters_.raycast_dtheta <= 0.0)
    {
      throw std::runtime_error("raycast increments must be positive");
    }
  }

  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr message)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*message));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (tree == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-OcTree octomap message '%s'",
        message->id.c_str());
      return;
    }

    auto copied_tree = std::make_unique<octomap::OcTree>(*tree);
    {
      std::lock_guard<std::mutex> lock(octomap_mutex_);
      latest_octomap_tree_ = std::move(copied_tree);
    }
    publish_live_map_markers();
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NbvPlanner::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(), "Received NBV goal with %zu boundary x point(s) and %zu boundary y point(s)",
      goal->boundary_x.size(), goal->boundary_y.size());
    if (goal->boundary_x.size() < 3 || goal->boundary_y.size() < 3 ||
      goal->boundary_x.size() != goal->boundary_y.size())
    {
      RCLCPP_WARN(get_logger(), "Rejecting NBV goal with invalid flat boundary arrays");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNbvPlanner>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNbvPlanner> goal_handle)
  {
    execute(goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandleNbvPlanner> goal_handle)
  {
    auto result = std::make_shared<NbvPlanner::Result>();
    result->request_base_pose = false;
    result->complete_exploration = false;
    result->explored_boundary_id = goal_handle->get_goal()->boundary_id;

    if (!latest_camera_pose_ || !latest_robot_footprint_) {
      RCLCPP_WARN(
        get_logger(), "Cannot run FKIE RRT: camera pose or robot footprint is missing");
      result->complete_exploration = true;
      goal_handle->succeed(result);
      return;
    }

    std::unique_ptr<octomap::OcTree> tree_snapshot;
    {
      std::lock_guard<std::mutex> lock(octomap_mutex_);
      if (latest_octomap_tree_) {
        tree_snapshot = std::make_unique<octomap::OcTree>(*latest_octomap_tree_);
      }
    }
    if (!tree_snapshot) {
      RCLCPP_WARN(
        get_logger(),
        "Starting FKIE RRT with an empty OctoMap because '%s' has not been received yet",
        octomap_topic_.c_str());
      tree_snapshot = std::make_unique<octomap::OcTree>(parameters_.empty_octomap_resolution);
    }

    current_boundary_ = make_boundary_polygon(*goal_handle->get_goal());
    normalize_boundary_z_limits(
      goal_handle->get_goal()->boundary_min_z, goal_handle->get_goal()->boundary_max_z);
    if (current_boundary_.points.size() < 3) {
      RCLCPP_WARN(get_logger(), "Cannot run FKIE RRT: boundary has fewer than 3 points");
      result->complete_exploration = true;
      goal_handle->succeed(result);
      return;
    }

    current_tree_ = tree_snapshot.get();
    measurement_grid_.set_grid_size(goal_handle->get_goal()->estimations.grid_size);
    for (const auto & value : goal_handle->get_goal()->estimations.values) {
      measurement_grid_.add_value(
        PositionGrid(value.x, value.y, value.z), MeasurementValue(value.mean, value.variance));
    }
    current_utility_max_value_free_space_ = parameters_.utility_max_value_free_space;
    current_utility_max_value_measurement_ = parameters_.utility_max_value_measurement;
    current_utility_max_value_visited_cell_ = parameters_.utility_max_value_visited_cell;

    update_gain_cached_nodes();
    remove_low_gain_cache_nodes();
    const std::vector<RRTNode> pre_rrt_frontiers = extract_cached_frontiers();
    best_branch_ = get_nbv_branch(pre_rrt_frontiers);
    const std::vector<RRTNode> post_rrt_frontiers = extract_cached_frontiers();
    last_frontiers_ = post_rrt_frontiers;
    publish_rrt_markers(post_rrt_frontiers);

    if (!best_branch_.empty()) {
      result->goals = best_branch_;
      result->request_base_pose = false;
    } else if (best_frontier_) {
      result->goal_pose_3d = best_frontier_->to_pose_stamped(now());
      result->request_base_pose = true;
    } else if (post_rrt_frontiers.empty()) {
      result->complete_exploration = true;
      result->request_base_pose = false;
    } else {
      result->complete_exploration = true;
      result->request_base_pose = false;
    }

    if (result->complete_exploration && post_rrt_frontiers.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "FKIE RRT produced no executable target. Check previous RRT diagnostics for root or "
        "sample rejection reasons.");
    }
    RCLCPP_INFO(
      get_logger(),
      "FKIE RRT result: branch_goals=%zu pre_frontiers=%zu post_frontiers=%zu cached_nodes=%zu "
      "request_base_pose=%s complete=%s",
      result->goals.size(), pre_rrt_frontiers.size(), post_rrt_frontiers.size(),
      cached_nodes_.size(), result->request_base_pose ? "true" : "false",
      result->complete_exploration ? "true" : "false");
    current_tree_ = nullptr;
    goal_handle->succeed(result);
  }

  void reset_tree()
  {
    tree_adapter_.clear();
    kdtree_ = std::make_unique<KdTree>(3, tree_adapter_);
    node_count_ = 0;
  }

  void add_tree_node(const std::shared_ptr<RRTNode> & node)
  {
    tree_adapter_.add_node(node);
    kdtree_->addPoints(0, tree_adapter_.kdtree_get_point_count() - 1);
  }

  std::shared_ptr<RRTNode> initialize_root()
  {
    if (!latest_camera_pose_) {
      root_initialized_ = false;
      return nullptr;
    }

    const Eigen::Vector3d root_position = pose_to_eigen_vector3d(latest_camera_pose_->pose);
    if (!point_in_polygon(root_position[0], root_position[1], current_boundary_) ||
      root_position[2] < parameters_.arm_height_min ||
      root_position[2] > parameters_.arm_height_max)
    {
      RCLCPP_WARN(
        get_logger(),
        "FKIE RRT root rejected: camera pose x=%.3f y=%.3f z=%.3f, boundary_points=%zu, "
        "arm_height=[%.3f, %.3f]",
        root_position[0], root_position[1], root_position[2], current_boundary_.points.size(),
        parameters_.arm_height_min, parameters_.arm_height_max);
      root_initialized_ = false;
      return nullptr;
    }

    auto root = std::make_shared<RRTNode>(parameters_);
    root->set_pose(latest_camera_pose_->pose);
    root->node_id = node_count_++;
    add_tree_node(root);
    root_initialized_ = true;
    return root;
  }

  bool expand_rrt(const std::vector<RRTNode> &)
  {
    reset_tree();
    best_node_.reset();
    const auto root = initialize_root();
    if (!root) {
      RCLCPP_WARN(get_logger(), "FKIE RRT expansion skipped because root initialization failed");
      return false;
    }

    int rejected_invalid = 0;
    int rejected_occupied = 0;
    int rejected_unknown = 0;
    int rejected_collision = 0;
    int rejected_boundary = 0;
    int rejected_footprint = 0;
    int rejected_close = 0;
    int accepted_nodes = 0;
    const geometry_msgs::msg::Pose & center_pose = latest_camera_pose_->pose;
    for (int i = 0; i < parameters_.rrt_tree_length; ++i) {
      bool accepted_sample = false;
      Eigen::Vector3d q_new;
      std::shared_ptr<RRTNode> nearest_node;

      for (int try_count = 0; try_count < parameters_.rrt_N_max_tries; ++try_count) {
        Eigen::Vector3d q_rand = generate_random_sample(
          parameters_.rrt_sampling_radius, parameters_, random_engine_);
        q_rand[0] += center_pose.position.x;
        q_rand[1] += center_pose.position.y;

        if (!find_closest_neighbor(q_rand, nearest_node)) {
          continue;
        }

        const Eigen::Vector3d q_near = pose_to_eigen_vector3d(nearest_node->get_pose());
        find_new_rrt_node(q_near, q_rand, q_new);
        if (!std::isfinite(q_new[0]) || !std::isfinite(q_new[1]) || !std::isfinite(q_new[2])) {
          ++rejected_invalid;
          continue;
        }

        const octomap::OcTreeNode * octomap_result =
          current_tree_->search(octomap::point3d(q_new[0], q_new[1], q_new[2]));
        if (parameters_.sample_in_unknown) {
          if (octomap_result != nullptr && octomap_result->getLogOdds() > 0.0) {
            ++rejected_occupied;
            continue;
          }
        } else if (octomap_result == nullptr) {
          ++rejected_unknown;
          continue;
        }

        if (collision_line(q_near, q_new, parameters_.rrt_collision_cyl_radius)) {
          ++rejected_collision;
          continue;
        }
        if (!is_sample_in_polygon(q_new, current_boundary_, parameters_.arm_height_max, parameters_)) {
          ++rejected_boundary;
          continue;
        }
        if (is_sample_in_polygon(q_new, current_robot_footprint_, parameters_.self_collision_height, parameters_)) {
          ++rejected_footprint;
          continue;
        }
        if (is_sample_close_to_rrt_node(q_new, parameters_.rrt_sample_threshold_distance)) {
          ++rejected_close;
          continue;
        }

        accepted_sample = true;
        break;
      }

      if (!accepted_sample || !nearest_node) {
        continue;
      }

      auto new_node = std::make_shared<RRTNode>(parameters_);
      if (!new_node->set_pose(q_new)) {
        continue;
      }
      new_node->node_id = node_count_++;
      update_gain(new_node);
      compute_yaw(new_node);
      new_node->parent_wptr = nearest_node;
      nearest_node->children.push_back(new_node);
      new_node->compute_score();

      if (!best_node_ || new_node->get_score() > best_node_->get_score()) {
        best_node_ = new_node;
      }

      add_tree_node(new_node);
      cached_nodes_.push_back(new_node->copy_to_rrt_node());
      ++accepted_nodes;
    }

    RCLCPP_INFO(
      get_logger(),
      "Current RRT size: %zu nodes accepted=%d rejected_invalid=%d rejected_occupied=%d "
      "rejected_unknown=%d rejected_collision=%d rejected_boundary=%d rejected_footprint=%d "
      "rejected_close=%d",
      tree_adapter_.kdtree_get_point_count(), accepted_nodes, rejected_invalid, rejected_occupied,
      rejected_unknown, rejected_collision, rejected_boundary, rejected_footprint, rejected_close);
    return tree_adapter_.kdtree_get_point_count() > 1;
  }

  bool find_closest_neighbor(
    const Eigen::Vector3d & sample,
    std::shared_ptr<RRTNode> & closest_neighbor) const
  {
    if (!kdtree_ || tree_adapter_.nodes.empty()) {
      return false;
    }
    double query_point[3] = {sample[0], sample[1], sample[2]};
    std::size_t result_index = 0;
    double output_distance_squared = 0.0;
    nanoflann::KNNResultSet<double> result_set(1);
    result_set.init(&result_index, &output_distance_squared);
    if (kdtree_->findNeighbors(result_set, query_point, nanoflann::SearchParams(10))) {
      closest_neighbor = tree_adapter_.nodes[result_index];
      return true;
    }
    return false;
  }

  void find_new_rrt_node(
    const Eigen::Vector3d & origin,
    const Eigen::Vector3d & point,
    Eigen::Vector3d & qnew) const
  {
    Eigen::Vector3d direction(point - origin);
    if (direction.norm() > parameters_.rrt_step_size) {
      direction = parameters_.rrt_step_size * direction.normalized();
    }
    qnew = origin + direction;
  }

  bool is_sample_close_to_rrt_node(const Eigen::Vector3d & sample, const double distance) const
  {
    if (!kdtree_ || tree_adapter_.nodes.empty()) {
      return false;
    }
    double query_point[3] = {sample[0], sample[1], sample[2]};
    std::size_t result_index = 0;
    double output_distance_squared = 0.0;
    nanoflann::KNNResultSet<double> result_set(1);
    result_set.init(&result_index, &output_distance_squared);
    if (kdtree_->findNeighbors(result_set, query_point, nanoflann::SearchParams(10))) {
      return output_distance_squared < distance * distance;
    }
    return false;
  }

  bool collision_line(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const double radius) const
  {
    if (current_tree_ == nullptr) {
      return true;
    }

    const octomap::point3d p1(start[0], start[1], start[2]);
    const octomap::point3d p2(end[0], end[1], end[2]);
    const octomap::point3d minimum(
      std::min(start[0], end[0]) - radius,
      std::min(start[1], end[1]) - radius,
      std::min(start[2], end[2]) - radius);
    const octomap::point3d maximum(
      std::max(start[0], end[0]) + radius,
      std::max(start[1], end[1]) + radius,
      std::max(start[2], end[2]) + radius);

    const double length_squared = (p2 - p1).norm_sq();
    const double radius_squared = radius * radius;
    for (auto it = current_tree_->begin_leafs_bbx(minimum, maximum),
      end_it = current_tree_->end_leafs_bbx(); it != end_it; ++it)
    {
      if (it->getLogOdds() <= 0.0) {
        continue;
      }
      const octomap::point3d point(it.getX(), it.getY(), it.getZ());
      if (cylinder_caps_first(
          p1, p2, static_cast<float>(length_squared),
          static_cast<float>(radius_squared), point) > 0.0F ||
        (p2 - point).norm() < radius)
      {
        return true;
      }
    }
    return false;
  }

  std::pair<double, double> gain_cubature(const geometry_msgs::msg::Pose & pose) const
  {
    if (current_tree_ == nullptr) {
      return std::make_pair(0.0, 0.0);
    }

    std::map<int, double> gain_per_yaw;
    const Eigen::Vector3d origin(pose.position.x, pose.position.y, pose.position.z);
    const double dtheta_rad = deg_to_rad(parameters_.raycast_dtheta);
    const double dphi_rad = deg_to_rad(parameters_.raycast_dphi);

    for (int theta = -180; theta < 180; theta += static_cast<int>(parameters_.raycast_dtheta)) {
      const double theta_rad = deg_to_rad(static_cast<double>(theta));
      for (int phi = static_cast<int>(90.0 - parameters_.camera_vfov / 2.0);
        phi < static_cast<int>(90.0 + parameters_.camera_vfov / 2.0);
        phi += static_cast<int>(parameters_.raycast_dphi))
      {
        const double phi_rad = deg_to_rad(static_cast<double>(phi));
        double ray_gain = 0.0;

        for (double range = parameters_.gain_r_min; range < parameters_.gain_r_max;
          range += parameters_.raycast_dr)
        {
          Eigen::Vector3d query_point;
          query_point[0] = pose.position.x + range * std::cos(theta_rad) * std::sin(phi_rad);
          query_point[1] = pose.position.y + range * std::sin(theta_rad) * std::sin(phi_rad);
          query_point[2] = pose.position.z + range * std::cos(phi_rad);

          if (!is_sample_in_polygon(
              query_point, current_boundary_, parameters_.arm_height_max, parameters_) ||
            is_sample_in_polygon(
              query_point, current_robot_footprint_, parameters_.arm_height_max, parameters_))
          {
            break;
          }

          const octomap::point3d query(query_point[0], query_point[1], query_point[2]);
          const octomap::OcTreeNode * result = current_tree_->search(query);
          if (result != nullptr) {
            if (result->getLogOdds() > 0.0) {
              break;
            }
          } else {
            ray_gain +=
              (2.0 * range * range * parameters_.raycast_dr +
              (parameters_.raycast_dr * parameters_.raycast_dr * parameters_.raycast_dr) / 6.0) *
              dtheta_rad * std::sin(phi_rad) * std::sin(dphi_rad / 2.0);
          }
        }

        gain_per_yaw[theta] += ray_gain;
      }
    }

    int best_yaw = 0;
    double best_yaw_score = 0.0;
    for (int yaw = -180; yaw < 180; ++yaw) {
      double yaw_score = 0.0;
      for (int hfov = static_cast<int>(-parameters_.camera_hfov / 2.0);
        hfov < static_cast<int>(parameters_.camera_hfov / 2.0); ++hfov)
      {
        int theta = yaw + hfov;
        if (theta < -180) {
          theta += 360;
        }
        if (theta > 180) {
          theta -= 360;
        }
        yaw_score += gain_per_yaw[theta];
      }
      if (best_yaw_score < yaw_score) {
        best_yaw_score = yaw_score;
        best_yaw = yaw;
      }
    }

    return {best_yaw_score, deg_to_rad(static_cast<double>(best_yaw))};
  }

  void update_gain(const std::shared_ptr<RRTNode> & node)
  {
    const auto [free_space_gain, yaw] = gain_cubature(node->get_pose());
    node->cubature_best_yaw = yaw;
    current_utility_max_value_free_space_ =
      std::max(current_utility_max_value_free_space_, free_space_gain);
    const MeasurementValue measurement_value = measurement_grid_.get_value(PositionGrid(node->get_pose()));
    const VisitedValue visited_value = visited_grid_.get_value(PositionGrid(node->get_pose()));
    current_utility_max_value_measurement_ =
      std::max(current_utility_max_value_measurement_, measurement_value.get_value());
    current_utility_max_value_visited_cell_ =
      std::max(current_utility_max_value_visited_cell_, visited_value.get_value());
    node->set_gain(
      free_space_gain / current_utility_max_value_free_space_,
      measurement_value.get_value() / current_utility_max_value_measurement_,
      (visited_value.get_value() / current_utility_max_value_visited_cell_) * -1.0);
  }

  void update_gain(RRTNode & node)
  {
    const auto [free_space_gain, yaw] = gain_cubature(node.get_pose());
    node.cubature_best_yaw = yaw;
    current_utility_max_value_free_space_ =
      std::max(current_utility_max_value_free_space_, free_space_gain);
    const MeasurementValue measurement_value = measurement_grid_.get_value(PositionGrid(node.get_pose()));
    const VisitedValue visited_value = visited_grid_.get_value(PositionGrid(node.get_pose()));
    current_utility_max_value_measurement_ =
      std::max(current_utility_max_value_measurement_, measurement_value.get_value());
    current_utility_max_value_visited_cell_ =
      std::max(current_utility_max_value_visited_cell_, visited_value.get_value());
    node.set_gain(
      free_space_gain / current_utility_max_value_free_space_,
      measurement_value.get_value() / current_utility_max_value_measurement_,
      (visited_value.get_value() / current_utility_max_value_visited_cell_) * -1.0);
  }

  void compute_yaw(const std::shared_ptr<RRTNode> & node) const
  {
    double yaw = 0.0;
    if (parameters_.compute_yaw_from_free_space || measurement_grid_.size() == 0) {
      yaw = node->cubature_best_yaw;
    }
    if (parameters_.compute_yaw_from_measurement) {
      std::vector<IndexGrid> indices;
      const PositionGrid node_position(node->get_pose());
      measurement_grid_.get_index_neighbors(node_position, 1, indices);
      const MeasurementValue reference = measurement_grid_.get_value(node_position);
      double max_difference = 0.0;
      std::optional<IndexGrid> max_index;
      for (const auto & index : indices) {
        const MeasurementValue value = measurement_grid_.get_value(index);
        const double difference = (value - reference).mean;
        if (difference > max_difference) {
          max_difference = difference;
          max_index = index;
        }
      }
      if (max_index) {
        const PositionGrid position_max = measurement_grid_.get_position(*max_index);
        yaw = std::atan2(position_max.y - node_position.y, position_max.x - node_position.x);
      }
    }
    node->set_orientation(optical_z_yaw_to_quaternion(yaw));
  }

  void update_gain_cached_nodes()
  {
    for (auto & node : cached_nodes_) {
      update_gain(node);
    }
  }

  void remove_low_gain_cache_nodes()
  {
    if (cached_nodes_.empty()) {
      return;
    }
    std::sort(
      cached_nodes_.begin(), cached_nodes_.end(),
      [](const RRTNode & lhs, const RRTNode & rhs) {
        return lhs.get_gain() > rhs.get_gain();
      });
    if (cached_nodes_.size() > static_cast<size_t>(parameters_.max_num_cached_points_to_keep)) {
      cached_nodes_.erase(
        cached_nodes_.begin() + parameters_.max_num_cached_points_to_keep, cached_nodes_.end());
    }
    min_gain_ = cached_nodes_.back().get_gain();
  }

  bool find_high_utility_frontier(
    const std::vector<RRTNode> & frontiers, RRTNode & best_frontier) const
  {
    if (!latest_camera_pose_) {
      return false;
    }
    double closest_distance = std::numeric_limits<double>::max();
    bool found = false;
    geometry_msgs::msg::PoseStamped robot_position;
    robot_position.pose = latest_camera_pose_->pose;
    for (const auto & frontier : frontiers) {
      geometry_msgs::msg::PoseStamped frontier_pose;
      frontier_pose.pose = frontier.get_pose();
      const double current_distance = distance_pose_stamped(robot_position, frontier_pose);
      if (current_distance < closest_distance && current_distance > parameters_.arm_goal_range) {
        closest_distance = current_distance;
        best_frontier = frontier;
        found = true;
      }
    }
    return found;
  }

  std::vector<std::shared_ptr<RRTNode>> get_best_node_branch(
    const std::shared_ptr<RRTNode> & node) const
  {
    std::vector<std::shared_ptr<RRTNode>> branch;
    auto current = node;
    while (current && current->parent_wptr.lock()) {
      branch.push_back(current);
      current = current->parent_wptr.lock();
    }
    std::reverse(branch.begin(), branch.end());
    return branch;
  }

  std::vector<geometry_msgs::msg::PoseStamped> extract_nbv_poses()
  {
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    if (!best_node_ || !latest_camera_pose_) {
      return goals;
    }

    const auto branch = get_best_node_branch(best_node_);
    for (const auto & node : branch) {
      if (node->get_gain() <= min_gain_) {
        continue;
      }

      auto goal_pose = node->to_pose_stamped(now());
      goals.push_back(goal_pose);
      if (parameters_.max_branch_nodes > 0 &&
        goals.size() >= static_cast<size_t>(parameters_.max_branch_nodes))
      {
        break;
      }
    }
    return goals;
  }

  std::vector<geometry_msgs::msg::PoseStamped> get_nbv_branch(
    const std::vector<RRTNode> & current_frontiers)
  {
    if (!expand_rrt(current_frontiers)) {
      return {};
    }
    if (!best_node_) {
      return {};
    }
    return extract_nbv_poses();
  }

  std::vector<RRTNode> extract_cached_frontiers()
  {
    std::vector<RRTNode> frontiers;
    if (cached_nodes_.empty()) {
      best_frontier_.reset();
      return frontiers;
    }

    std::sort(
      cached_nodes_.begin(), cached_nodes_.end(),
      [](const RRTNode & lhs, const RRTNode & rhs) {
        return lhs.get_gain() > rhs.get_gain();
      });
    if (cached_nodes_.size() > static_cast<size_t>(parameters_.max_num_cached_points_to_keep)) {
      cached_nodes_.erase(
        cached_nodes_.begin() + parameters_.max_num_cached_points_to_keep, cached_nodes_.end());
    }

    const size_t frontier_count = std::min(
      cached_nodes_.size(), static_cast<size_t>(parameters_.num_frontiers));
    for (size_t i = 0; i < frontier_count; ++i) {
      if (parameters_.free_space_frontier &&
        cached_nodes_[i].get_gain_free_space() <= parameters_.min_gain_frontier)
      {
        continue;
      }
      if (!parameters_.free_space_frontier &&
        cached_nodes_[i].get_gain() <= parameters_.min_gain_frontier)
      {
        continue;
      }
      frontiers.push_back(cached_nodes_[i]);
    }

    if (!frontiers.empty()) {
      RRTNode selected_frontier(parameters_);
      if (find_high_utility_frontier(frontiers, selected_frontier)) {
        best_frontier_ = std::make_unique<RRTNode>(selected_frontier);
      } else {
        best_frontier_ = std::make_unique<RRTNode>(frontiers.front());
      }
    } else {
      best_frontier_.reset();
    }
    return frontiers;
  }

  visualization_msgs::msg::Marker make_marker(
    const int id,
    const std::string & ns,
    const int32_t type,
    const std_msgs::msg::ColorRGBA & color,
    const double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parameters_.world_frame;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    return marker;
  }

  std_msgs::msg::ColorRGBA make_color(
    const float r, const float g, const float b, const float a) const
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  bool point_inside_current_boundary_volume(const double x, const double y, const double z) const
  {
    return z >= current_boundary_min_z_ && z <= current_boundary_max_z_ &&
           point_in_polygon(x, y, current_boundary_);
  }

  void add_voxel_marker_point(
    visualization_msgs::msg::Marker & marker,
    const double x,
    const double y,
    const double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);
  }

  void populate_occupancy_markers(
    visualization_msgs::msg::Marker & known_free,
    visualization_msgs::msg::Marker & known_occupied,
    visualization_msgs::msg::Marker & unknown) const
  {
    if (current_tree_ == nullptr || current_boundary_.points.size() < 3) {
      return;
    }

    double min_x = current_boundary_.points.front().x;
    double max_x = current_boundary_.points.front().x;
    double min_y = current_boundary_.points.front().y;
    double max_y = current_boundary_.points.front().y;
    for (const auto & point : current_boundary_.points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    const double resolution = std::max(kOccupancyMarkerResolution, current_tree_->getResolution());
    known_free.scale.x = resolution;
    known_free.scale.y = resolution;
    known_free.scale.z = resolution;
    known_occupied.scale = known_free.scale;
    unknown.scale = known_free.scale;

    for (auto it = current_tree_->begin_leafs(), end = current_tree_->end_leafs(); it != end; ++it) {
      const double x = it.getX();
      const double y = it.getY();
      const double z = it.getZ();
      if (!point_inside_current_boundary_volume(x, y, z)) {
        continue;
      }
      if (current_tree_->isNodeOccupied(*it)) {
        if (known_occupied.points.size() <
          static_cast<size_t>(max_known_occupied_marker_voxels_))
        {
          add_voxel_marker_point(known_occupied, x, y, z);
        }
      } else if (known_free.points.size() < static_cast<size_t>(max_known_free_marker_voxels_)) {
        add_voxel_marker_point(known_free, x, y, z);
      }
    }

    for (double x = min_x + resolution * 0.5; x <= max_x; x += resolution) {
      for (double y = min_y + resolution * 0.5; y <= max_y; y += resolution) {
        if (!point_in_polygon(x, y, current_boundary_)) {
          continue;
        }
        for (double z = current_boundary_min_z_ + resolution * 0.5;
          z <= current_boundary_max_z_;
          z += resolution)
        {
          if (unknown.points.size() >= static_cast<size_t>(max_unknown_marker_voxels_)) {
            return;
          }
          if (current_tree_->search(octomap::point3d(x, y, z)) != nullptr) {
            continue;
          }
          add_voxel_marker_point(unknown, x, y, z);
        }
      }
    }
  }

  void publish_rrt_markers(const std::vector<RRTNode> & frontiers)
  {
    if (!rrt_marker_publisher_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    auto nodes = make_marker(
      0, "fkie_rrt_nodes", visualization_msgs::msg::Marker::SPHERE_LIST,
      make_color(0.1F, 0.6F, 1.0F, 0.8F), 0.06);
    auto edges = make_marker(
      1, "fkie_rrt_edges", visualization_msgs::msg::Marker::LINE_LIST,
      make_color(0.2F, 0.8F, 0.9F, 0.45F), 0.015);
    auto branch = make_marker(
      2, "fkie_rrt_best_branch", visualization_msgs::msg::Marker::LINE_STRIP,
      make_color(1.0F, 0.8F, 0.0F, 1.0F), 0.04);
    auto frontier_marker = make_marker(
      3, "fkie_rrt_frontiers", visualization_msgs::msg::Marker::SPHERE_LIST,
      make_color(1.0F, 0.1F, 0.1F, 0.9F), 0.12);
    auto branch_view_dirs = make_marker(
      4, "fkie_rrt_best_branch_view_dirs", visualization_msgs::msg::Marker::LINE_LIST,
      make_color(0.1F, 1.0F, 0.2F, 1.0F), 0.035);
    auto boundary_marker = make_marker(
      5, "fkie_exploration_boundary", visualization_msgs::msg::Marker::LINE_LIST,
      make_color(1.0F, 0.0F, 1.0F, 0.9F), 0.04);
    auto boundary_label = make_marker(
      6, "fkie_exploration_boundary_label", visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      make_color(1.0F, 0.0F, 1.0F, 1.0F), 0.28);
    auto known_free_voxels = make_marker(
      7, "fkie_known_free_voxels", visualization_msgs::msg::Marker::CUBE_LIST,
      make_color(0.6F, 0.6F, 0.6F, 0.3F), kOccupancyMarkerResolution);
    auto known_occupied_voxels = make_marker(
      8, "fkie_known_occupied_voxels", visualization_msgs::msg::Marker::CUBE_LIST,
      make_color(1.0F, 0.1F, 0.1F, 0.3F), kOccupancyMarkerResolution);
    auto unknown_voxels = make_marker(
      9, "fkie_unknown_voxels", visualization_msgs::msg::Marker::CUBE_LIST,
      make_color(0.1F, 0.4F, 1.0F, 0.3F), kOccupancyMarkerResolution);
    const double camera_direction_length = 0.35;

    for (const auto & node : tree_adapter_.nodes) {
      nodes.points.push_back(node->get_pose().position);
      if (const auto parent = node->parent_wptr.lock()) {
        edges.points.push_back(parent->get_pose().position);
        edges.points.push_back(node->get_pose().position);
      }
    }

    if (best_node_) {
      const auto best_branch = get_best_node_branch(best_node_);
      for (size_t branch_index = 0; branch_index < best_branch.size(); ++branch_index) {
        const auto & node = best_branch[branch_index];
        const auto pose = node->get_pose();
        branch.points.push_back(pose.position);

        tf2::Quaternion orientation;
        tf2::fromMsg(pose.orientation, orientation);
        const tf2::Vector3 direction =
          tf2::quatRotate(orientation.normalized(), tf2::Vector3(0.0, 0.0, 1.0));
        geometry_msgs::msg::Point direction_end = pose.position;
        direction_end.x += camera_direction_length * direction.x();
        direction_end.y += camera_direction_length * direction.y();
        direction_end.z += camera_direction_length * direction.z();
        branch_view_dirs.points.push_back(pose.position);
        branch_view_dirs.points.push_back(direction_end);
      }
    }

    for (const auto & frontier : frontiers) {
      frontier_marker.points.push_back(frontier.get_pose().position);
    }

    if (current_boundary_.points.size() >= 3) {
      double center_x = 0.0;
      double center_y = 0.0;
      for (size_t i = 0; i < current_boundary_.points.size(); ++i) {
        const auto & current = current_boundary_.points[i];
        const auto & next = current_boundary_.points[(i + 1) % current_boundary_.points.size()];
        center_x += current.x;
        center_y += current.y;

        geometry_msgs::msg::Point current_low;
        current_low.x = current.x;
        current_low.y = current.y;
        current_low.z = current_boundary_min_z_;
        geometry_msgs::msg::Point next_low;
        next_low.x = next.x;
        next_low.y = next.y;
        next_low.z = current_boundary_min_z_;
        geometry_msgs::msg::Point current_high = current_low;
        current_high.z = current_boundary_max_z_;
        geometry_msgs::msg::Point next_high = next_low;
        next_high.z = current_boundary_max_z_;

        boundary_marker.points.push_back(current_low);
        boundary_marker.points.push_back(next_low);
        boundary_marker.points.push_back(current_high);
        boundary_marker.points.push_back(next_high);
        boundary_marker.points.push_back(current_low);
        boundary_marker.points.push_back(current_high);
      }
      center_x /= static_cast<double>(current_boundary_.points.size());
      center_y /= static_cast<double>(current_boundary_.points.size());
      boundary_label.pose.position.x = center_x;
      boundary_label.pose.position.y = center_y;
      boundary_label.pose.position.z = current_boundary_max_z_ + 0.25;
      boundary_label.text = "FKIE target area";
    }
    populate_occupancy_markers(known_free_voxels, known_occupied_voxels, unknown_voxels);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "FKIE volume markers: known_free=%zu/%d known_occupied=%zu/%d unknown=%zu/%d "
      "boundary_points=%zu z=[%.2f, %.2f]",
      known_free_voxels.points.size(), max_known_free_marker_voxels_,
      known_occupied_voxels.points.size(), max_known_occupied_marker_voxels_,
      unknown_voxels.points.size(), max_unknown_marker_voxels_,
      current_boundary_.points.size(),
      current_boundary_min_z_, current_boundary_max_z_);

    markers.markers.push_back(nodes);
    markers.markers.push_back(edges);
    markers.markers.push_back(branch);
    markers.markers.push_back(frontier_marker);
    markers.markers.push_back(branch_view_dirs);
    markers.markers.push_back(boundary_marker);
    markers.markers.push_back(boundary_label);
    markers.markers.push_back(known_free_voxels);
    markers.markers.push_back(known_occupied_voxels);
    markers.markers.push_back(unknown_voxels);
    rrt_marker_publisher_->publish(markers);
  }

  void publish_live_map_markers()
  {
    if (!rrt_marker_publisher_ || current_boundary_.points.size() < 3) {
      return;
    }

    const rclcpp::Time current_time = now();
    if (last_live_map_marker_publish_time_.nanoseconds() != 0 &&
      (current_time - last_live_map_marker_publish_time_).seconds() < 1.0)
    {
      return;
    }
    last_live_map_marker_publish_time_ = current_time;

    std::unique_ptr<octomap::OcTree> tree_snapshot;
    {
      std::lock_guard<std::mutex> lock(octomap_mutex_);
      if (latest_octomap_tree_) {
        tree_snapshot = std::make_unique<octomap::OcTree>(*latest_octomap_tree_);
      }
    }
    if (!tree_snapshot) {
      return;
    }

    current_tree_ = tree_snapshot.get();
    publish_rrt_markers(last_frontiers_);
    current_tree_ = nullptr;
  }

  geometry_msgs::msg::Polygon make_boundary_polygon(const NbvPlanner::Goal & goal) const
  {
    geometry_msgs::msg::Polygon polygon;
    if (goal.boundary_x.size() != goal.boundary_y.size()) {
      return polygon;
    }

    polygon.points.reserve(goal.boundary_x.size());
    for (size_t i = 0; i < goal.boundary_x.size(); ++i) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(goal.boundary_x[i]);
      point.y = static_cast<float>(goal.boundary_y[i]);
      point.z = 0.0F;
      polygon.points.push_back(point);
    }
    return polygon;
  }

  void normalize_boundary_z_limits(const double requested_min_z, const double requested_max_z)
  {
    current_boundary_min_z_ = std::min(requested_min_z, requested_max_z);
    current_boundary_max_z_ = std::max(requested_min_z, requested_max_z);
    if (requested_min_z > requested_max_z) {
      RCLCPP_WARN(
        get_logger(),
        "Received reversed FKIE boundary z limits [%.3f, %.3f]; normalized to [%.3f, %.3f]",
        requested_min_z, requested_max_z, current_boundary_min_z_, current_boundary_max_z_);
    }
  }

  bool point_in_polygon(
    const double x, const double y, const geometry_msgs::msg::Polygon & polygon) const
  {
    if (polygon.points.size() < 3) {
      return false;
    }

    bool inside = false;
    size_t previous = polygon.points.size() - 1;
    for (size_t current = 0; current < polygon.points.size(); ++current) {
      const auto & pi = polygon.points[current];
      const auto & pj = polygon.points[previous];
      const bool crosses_y = (pi.y > y) != (pj.y > y);
      if (crosses_y) {
        const double x_intersection =
          (static_cast<double>(pj.x - pi.x) * (y - pi.y) / static_cast<double>(pj.y - pi.y)) +
          pi.x;
        if (x < x_intersection) {
          inside = !inside;
        }
      }
      previous = current;
    }
    return inside;
  }

  bool is_occupied(const octomap::OcTree & tree, const octomap::OcTreeNode & node) const
  {
    return tree.isNodeOccupied(&node);
  }

  std::string action_name_;
  std::string camera_pose_topic_;
  std::string octomap_topic_;
  std::string rrt_marker_topic_;
  std::string robot_footprint_topic_;
  int max_known_free_marker_voxels_;
  int max_known_occupied_marker_voxels_;
  int max_unknown_marker_voxels_;
  NbvParameters parameters_;
  std::mt19937 random_engine_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  TreeNanoflannAdapter tree_adapter_;
  std::unique_ptr<KdTree> kdtree_;
  int node_count_ = 0;
  double min_gain_ = -1000.0;
  double current_boundary_min_z_ = 0.0;
  double current_boundary_max_z_ = 0.0;
  double current_utility_max_value_free_space_;
  double current_utility_max_value_measurement_;
  double current_utility_max_value_visited_cell_;
  bool root_initialized_ = false;
  SparseGrid<MeasurementValue, IndexGrid, IndexGridHasher> measurement_grid_{"measurement_grid"};
  SparseGrid<VisitedValue, IndexGrid2D, IndexGrid2DHasher> visited_grid_{"visited_grid"};

  std::shared_ptr<RRTNode> best_node_;
  std::unique_ptr<RRTNode> best_frontier_;
  std::vector<geometry_msgs::msg::PoseStamped> best_branch_;
  std::vector<RRTNode> cached_nodes_;
  std::vector<RRTNode> last_frontiers_;
  geometry_msgs::msg::Polygon current_boundary_;
  geometry_msgs::msg::Polygon current_robot_footprint_;
  octomap::OcTree * current_tree_ = nullptr;
  rclcpp::Time last_live_map_marker_publish_time_{0, 0, RCL_ROS_TIME};

  geometry_msgs::msg::PoseStamped::SharedPtr latest_camera_pose_;
  geometry_msgs::msg::PolygonStamped::SharedPtr latest_robot_footprint_;
  std::unique_ptr<octomap::OcTree> latest_octomap_tree_;
  mutable std::mutex octomap_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_subscription_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr robot_footprint_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_marker_publisher_;
  rclcpp_action::Server<NbvPlanner>::SharedPtr action_server_;
};
}  // namespace mobile_manipulator_fkie_nbv

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_fkie_nbv::FkieNbvPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
