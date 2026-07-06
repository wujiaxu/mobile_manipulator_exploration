#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp"
#include "mobile_manipulator_moveit_bridge/action/move_arm.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using NbvPlanner = mobile_manipulator_fkie_msgs::action::NbvPlanner;
using GoalHandleNbvPlanner = rclcpp_action::ClientGoalHandle<NbvPlanner>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using MoveArm = mobile_manipulator_moveit_bridge::action::MoveArm;

class NbvArmTargetAdapter : public rclcpp::Node
{
public:
  NbvArmTargetAdapter()
  : Node("nbv_arm_target_adapter"),
    action_name_(declare_parameter<std::string>("action_name", "/nbv_rrt")),
    base_action_name_(declare_parameter<std::string>("base_action_name", "/navigate_to_pose")),
    arm_action_name_(declare_parameter<std::string>("arm_action_name", "/move_arm")),
    execution_marker_topic_(
      declare_parameter<std::string>("execution_marker_topic", "/fkie_nbv/execution_markers")),
    target_topic_(declare_parameter<std::string>("target_topic", "/arm_target_pose")),
    named_target_topic_(declare_parameter<std::string>("named_target_topic", "/arm_named_target")),
    frame_id_(declare_parameter<std::string>("frame_id", "map")),
    robot_frame_(declare_parameter<std::string>("robot_frame", "base_link")),
    end_effector_frame_(declare_parameter<std::string>("end_effector_frame", "link_eef")),
    camera_frame_(declare_parameter<std::string>(
      "camera_frame", "wrist_camera_color_optical_frame")),
    min_x_(declare_parameter<double>("min_x", -3.0)),
    max_x_(declare_parameter<double>("max_x", 3.0)),
    min_y_(declare_parameter<double>("min_y", -3.0)),
    max_y_(declare_parameter<double>("max_y", 3.0)),
    min_z_(declare_parameter<double>("min_z", 0.4)),
    max_z_(declare_parameter<double>("max_z", 1.4)),
    measurement_grid_size_(declare_parameter<double>("measurement_grid_size", 0.5)),
    wait_timeout_s_(declare_parameter<double>("wait_timeout_s", 10.0)),
    base_goal_result_timeout_s_(declare_parameter<double>("base_goal_result_timeout_s", 60.0)),
    auto_explore_(declare_parameter<bool>("auto_explore", false)),
    max_exploration_iterations_(declare_parameter<int>("max_exploration_iterations", 20)),
    map_update_wait_s_(declare_parameter<double>("map_update_wait_s", 2.0)),
    continue_on_motion_failure_(declare_parameter<bool>("continue_on_motion_failure", true)),
    continue_base_on_stow_failure_(
      declare_parameter<bool>("continue_base_on_stow_failure", true)),
    stow_arm_before_base_motion_(declare_parameter<bool>("stow_arm_before_base_motion", true)),
    publish_debug_arm_targets_(declare_parameter<bool>("publish_debug_arm_targets", false)),
    retry_arm_after_base_fallback_(declare_parameter<bool>("retry_arm_after_base_fallback", true)),
    base_goal_standoff_(declare_parameter<double>("base_goal_standoff", 0.45)),
    arm_named_target_retry_count_(declare_parameter<int>("arm_named_target_retry_count", 1)),
    arm_named_target_retry_delay_s_(declare_parameter<double>("arm_named_target_retry_delay_s", 1.0)),
    stow_arm_wait_s_(declare_parameter<double>("stow_arm_wait_s", 8.0)),
    arm_goal_range_(declare_parameter<double>("arm_goal_range", 1.3)),
    arm_goal_publish_delay_s_(declare_parameter<double>("arm_goal_publish_delay_s", 8.0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    client_ = rclcpp_action::create_client<NbvPlanner>(this, action_name_);
    base_client_ = rclcpp_action::create_client<NavigateToPose>(this, base_action_name_);
    arm_client_ = rclcpp_action::create_client<MoveArm>(this, arm_action_name_);
    target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_topic_, 10);
    named_target_publisher_ = create_publisher<std_msgs::msg::String>(named_target_topic_, 10);
    execution_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
        execution_marker_topic_, rclcpp::QoS(1).transient_local().reliable());
  }

  int run()
  {
    RCLCPP_INFO(get_logger(), "Waiting for NBV action server '%s'", action_name_.c_str());
    if (!client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_s_))) {
      RCLCPP_ERROR(get_logger(), "NBV action server not available: %s", action_name_.c_str());
      return 1;
    }

    if (auto_explore_) {
      return run_auto_exploration();
    }

    int error_code = 0;
    const auto result = request_nbv_result(1, error_code);
    if (!result) {
      return error_code;
    }
    return dispatch_nbv_result(result, false);
  }

private:
  int run_auto_exploration()
  {
    const int max_iterations = std::max(1, max_exploration_iterations_);
    for (int iteration = 1; rclcpp::ok() && iteration <= max_iterations; ++iteration) {
      RCLCPP_INFO(
        get_logger(), "FKIE auto exploration iteration %d/%d", iteration, max_iterations);

      int error_code = 0;
      const auto result = request_nbv_result(iteration, error_code);
      if (!result) {
        return error_code;
      }
      if (result->complete_exploration) {
        RCLCPP_INFO(get_logger(), "FKIE auto exploration stopped: planner reported complete");
        return 0;
      }

      const int motion_result = dispatch_nbv_result(result, true);
      if (motion_result != 0) {
        RCLCPP_WARN(
          get_logger(), "FKIE auto exploration motion step failed with code %d", motion_result);
        if (!continue_on_motion_failure_) {
          return motion_result;
        }
      }

      RCLCPP_INFO(
        get_logger(), "Waiting %.2f s for OctoMap update before next FKIE iteration",
        map_update_wait_s_);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(map_update_wait_s_)));
    }

    RCLCPP_WARN(
      get_logger(), "FKIE auto exploration stopped after reaching max_exploration_iterations=%d",
      max_iterations);
    return 0;
  }

  std::shared_ptr<const NbvPlanner::Result> request_nbv_result(
    const int iteration,
    int & error_code)
  {
    (void)iteration;
    auto goal_handle_future = client_->async_send_goal(make_goal());
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for NBV goal response");
      error_code = 2;
      return nullptr;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "NBV goal rejected");
      error_code = 3;
      return nullptr;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for NBV result");
      error_code = 4;
      return nullptr;
    }

    error_code = 0;
    return result_future.get().result;
  }

  int dispatch_nbv_result(
    const std::shared_ptr<const NbvPlanner::Result> & result,
    const bool wait_for_base_result)
  {
    if (!result->goals.empty()) {
      return dispatch_branch_goals(result);
    }

    if (result->request_base_pose) {
      return dispatch_base_goal(result->goal_pose_3d, wait_for_base_result);
    }

    if (result->complete_exploration) {
      RCLCPP_INFO(get_logger(), "NBV planner reported complete exploration");
      return 0;
    }

    RCLCPP_ERROR(get_logger(), "NBV result contained no executable arm or base goal");
    return 5;
  }

  NbvPlanner::Goal make_goal() const
  {
    NbvPlanner::Goal goal;
    goal.header.frame_id = frame_id_;
    goal.boundary_id = 1;
    goal.boundary_source_type = "fixed_test_boundary";
    goal.boundary_frame_id = frame_id_;
    goal.boundary_min_z = min_z_;
    goal.boundary_max_z = max_z_;
    goal.boundary_x = {min_x_, max_x_, max_x_, min_x_};
    goal.boundary_y = {min_y_, min_y_, max_y_, max_y_};
    goal.estimations.header.frame_id = frame_id_;
    goal.estimations.grid_size = static_cast<float>(measurement_grid_size_);
    goal.arm_in_prepacked = false;
    return goal;
  }

  int dispatch_branch_goals(const std::shared_ptr<const NbvPlanner::Result> & result)
  {
    std::vector<int> target_states(result->goals.size(), 0);
    publish_execution_markers(result->goals, target_states);
    for (std::size_t target_index = 0; target_index < result->goals.size(); ++target_index) {
      auto camera_goal = result->goals[target_index];
      camera_goal.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(
        get_logger(), "NBV branch camera target %zu/%zu: frame=%s x=%.3f y=%.3f z=%.3f",
        target_index + 1, result->goals.size(), camera_goal.header.frame_id.c_str(),
        camera_goal.pose.position.x, camera_goal.pose.position.y, camera_goal.pose.position.z);
      if (!is_arm_reachable(camera_goal)) {
        target_states[target_index] = 2;
        publish_execution_markers(result->goals, target_states);
        RCLCPP_WARN(
          get_logger(),
          "Skipping NBV branch target %zu/%zu because it is outside current arm reach",
          target_index + 1, result->goals.size());
        continue;
      }

      const auto arm_target = camera_pose_to_eef_pose(camera_goal);
      if (!arm_target) {
        return 6;
      }
      RCLCPP_INFO(
        get_logger(), "NBV branch EEF target %zu/%zu: frame=%s x=%.3f y=%.3f z=%.3f",
        target_index + 1, result->goals.size(), arm_target->header.frame_id.c_str(),
        arm_target->pose.position.x, arm_target->pose.position.y, arm_target->pose.position.z);

      if (publish_debug_arm_targets_) {
        target_publisher_->publish(*arm_target);
      }
      const int arm_result = dispatch_arm_pose_goal(arm_target);
      if (arm_result != 0) {
        target_states[target_index] = 2;
        publish_execution_markers(result->goals, target_states);
        RCLCPP_WARN(
          get_logger(),
          "Skipping NBV branch target %zu/%zu because MoveIt could not execute it",
          target_index + 1, result->goals.size());
        continue;
      }
      target_states[target_index] = 1;
      publish_execution_markers(result->goals, target_states);
      RCLCPP_INFO(
        get_logger(), "Executed NBV arm target %zu/%zu: frame=%s x=%.3f y=%.3f z=%.3f",
        target_index + 1, result->goals.size(), arm_target->header.frame_id.c_str(),
        arm_target->pose.position.x, arm_target->pose.position.y, arm_target->pose.position.z);

      (void)arm_goal_publish_delay_s_;
    }

    const bool reached_final_target = !result->goals.empty() && target_states.back() == 1;
    if (!reached_final_target && !result->goals.empty()) {
      const auto base_goal = base_goal_from_camera_pose(result->goals.back());
      RCLCPP_WARN(
        get_logger(),
        "Branch target q_t+1 was not reached by the arm; moving base to its projection");
      return dispatch_base_goal(base_goal, true);
    }
    return 0;
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

  visualization_msgs::msg::Marker make_execution_marker(
    const int id,
    const std::string & ns,
    const int32_t type,
    const std_msgs::msg::ColorRGBA & color,
    const double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
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

  void publish_execution_markers(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals,
    const std::vector<int> & states) const
  {
    if (!execution_marker_publisher_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    auto pending = make_execution_marker(
      0, "fkie_pending_targets", visualization_msgs::msg::Marker::SPHERE_LIST,
      make_color(1.0F, 1.0F, 0.0F, 0.9F), 0.14);
    auto reached = make_execution_marker(
      1, "fkie_reached_targets", visualization_msgs::msg::Marker::SPHERE_LIST,
      make_color(0.0F, 1.0F, 0.0F, 0.9F), 0.16);
    auto unreachable = make_execution_marker(
      2, "fkie_unreachable_targets", visualization_msgs::msg::Marker::SPHERE_LIST,
      make_color(1.0F, 0.0F, 0.0F, 0.9F), 0.16);

    for (std::size_t i = 0; i < goals.size(); ++i) {
      const int state = i < states.size() ? states[i] : 0;
      if (state == 1) {
        reached.points.push_back(goals[i].pose.position);
      } else if (state == 2) {
        unreachable.points.push_back(goals[i].pose.position);
      } else {
        pending.points.push_back(goals[i].pose.position);
      }

      const auto label_color = state == 1 ? make_color(0.0F, 1.0F, 0.0F, 1.0F) :
        state == 2 ? make_color(1.0F, 0.0F, 0.0F, 1.0F) :
        make_color(1.0F, 1.0F, 0.0F, 1.0F);
      auto label = make_execution_marker(
        100 + static_cast<int>(i), "fkie_execution_target_numbers",
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, label_color, 0.22);
      label.pose.position = goals[i].pose.position;
      label.pose.position.z += 0.22;
      label.text = std::to_string(i + 1);
      markers.markers.push_back(label);
    }

    markers.markers.push_back(pending);
    markers.markers.push_back(reached);
    markers.markers.push_back(unreachable);
    execution_marker_publisher_->publish(markers);
  }

  bool is_arm_reachable(const geometry_msgs::msg::PoseStamped & camera_goal)
  {
    try {
      const auto goal_in_robot_frame = tf_buffer_.transform(
        camera_goal, robot_frame_, tf2::durationFromSec(wait_timeout_s_));
      const double distance = std::hypot(
        goal_in_robot_frame.pose.position.x,
        goal_in_robot_frame.pose.position.y,
        goal_in_robot_frame.pose.position.z);
      return distance <= arm_goal_range_;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform branch target from '%s' to '%s' for reach check: %s",
        camera_goal.header.frame_id.c_str(), robot_frame_.c_str(), error.what());
      return false;
    }
  }

  geometry_msgs::msg::PoseStamped nearest_branch_pose_for_base(
    const std::vector<geometry_msgs::msg::PoseStamped> & branch,
    const std::size_t target_index) const
  {
    const auto & target = branch[target_index];
    if (target_index == 0) {
      return target;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < target_index; ++i) {
      const double dx = branch[i].pose.position.x - target.pose.position.x;
      const double dy = branch[i].pose.position.y - target.pose.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }
    return branch[nearest_index];
  }

  geometry_msgs::msg::PoseStamped base_goal_from_camera_pose(
    const geometry_msgs::msg::PoseStamped & camera_goal) const
  {
    tf2::Quaternion orientation;
    tf2::fromMsg(camera_goal.pose.orientation, orientation);
    const tf2::Vector3 view_direction =
      tf2::quatRotate(orientation.normalized(), tf2::Vector3(0.0, 0.0, 1.0));
    const double yaw = std::atan2(view_direction.y(), view_direction.x());

    geometry_msgs::msg::PoseStamped base_goal = camera_goal;
    base_goal.pose.position.x =
      camera_goal.pose.position.x - base_goal_standoff_ * std::cos(yaw);
    base_goal.pose.position.y =
      camera_goal.pose.position.y - base_goal_standoff_ * std::sin(yaw);
    base_goal.pose.position.z = 0.0;

    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    base_goal.pose.orientation = tf2::toMsg(yaw_only);
    RCLCPP_INFO(
      get_logger(),
      "Computed NBV base fallback from camera target: camera=(%.3f, %.3f, %.3f) "
      "base=(%.3f, %.3f, %.3f) yaw=%.3f standoff=%.3f",
      camera_goal.pose.position.x, camera_goal.pose.position.y, camera_goal.pose.position.z,
      base_goal.pose.position.x, base_goal.pose.position.y, base_goal.pose.position.z,
      yaw, base_goal_standoff_);
    return base_goal;
  }

  int dispatch_base_goal(const geometry_msgs::msg::PoseStamped & goal_pose_3d, const bool wait_for_result)
  {
    if (stow_arm_before_base_motion_) {
      const int stow_result = stow_arm_for_base_motion();
      if (stow_result != 0) {
        if (!continue_base_on_stow_failure_) {
          return stow_result;
        }
        RCLCPP_WARN(
          get_logger(),
          "Continuing with NBV base motion despite stow failure code %d; "
          "set continue_base_on_stow_failure=false to make this fatal",
          stow_result);
      }
    }

    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server '%s'", base_action_name_.c_str());
    if (!base_client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_s_))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available: %s", base_action_name_.c_str());
      return 7;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal_pose_3d;
    nav_goal.pose.header.frame_id =
      nav_goal.pose.header.frame_id.empty() ? frame_id_ : nav_goal.pose.header.frame_id;
    nav_goal.pose.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    nav_goal.pose.pose.position.z = 0.0;

    const double yaw = tf2::getYaw(goal_pose_3d.pose.orientation);
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    nav_goal.pose.pose.orientation = tf2::toMsg(yaw_only);

    auto goal_handle_future = base_client_->async_send_goal(nav_goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for Nav2 goal response");
      return 8;
    }

    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Nav2 goal rejected");
      return 9;
    }

    RCLCPP_INFO(
      get_logger(), "Sent NBV base fallback goal to '%s': frame=%s x=%.3f y=%.3f yaw=%.3f",
      base_action_name_.c_str(), nav_goal.pose.header.frame_id.c_str(),
      nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y, yaw);

    if (wait_for_result) {
      auto result_future = base_client_->async_get_result(goal_handle);
      const auto wait_code = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future, std::chrono::duration<double>(base_goal_result_timeout_s_));
      if (wait_code != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Timed out waiting for Nav2 result during NBV branch execution");
        return 10;
      }
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(get_logger(), "Nav2 failed during NBV branch execution");
        return 11;
      }
    }
    return 0;
  }

  int stow_arm_for_base_motion()
  {
    std_msgs::msg::String home_message;
    home_message.data = "home";
    if (publish_debug_arm_targets_) {
      named_target_publisher_->publish(home_message);
    }
    const int arm_result = dispatch_arm_named_target();
    if (arm_result != 0) {
      RCLCPP_ERROR(get_logger(), "Arm failed to stow before base motion");
      return arm_result;
    }
    RCLCPP_INFO(
      get_logger(), "Requested arm named target '%s' on '%s' before base motion",
      home_message.data.c_str(), named_target_topic_.c_str());
    (void)stow_arm_wait_s_;
    return 0;
  }

  int dispatch_arm_named_target()
  {
    RCLCPP_INFO(get_logger(), "Waiting for arm action server '%s'", arm_action_name_.c_str());
    if (!arm_client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_s_))) {
      RCLCPP_ERROR(get_logger(), "Arm action server not available: %s", arm_action_name_.c_str());
      return 12;
    }

    const int max_attempts = std::max(1, arm_named_target_retry_count_ + 1);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      MoveArm::Goal arm_goal;
      arm_goal.use_named_target = true;
      arm_goal.named_target = "home";
      auto goal_handle_future = arm_client_->async_send_goal(arm_goal);
      if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Failed while waiting for arm named target goal response");
        return 13;
      }
      const auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Arm named target goal rejected");
        return 14;
      }
      auto result_future = arm_client_->async_get_result(goal_handle);
      if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Failed while waiting for arm named target result");
        return 15;
      }
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->success)
      {
        return 0;
      }
      RCLCPP_WARN(
        get_logger(), "Arm named target action attempt %d/%d failed: %s",
        attempt, max_attempts, wrapped_result.result->message.c_str());
      if (attempt < max_attempts) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(arm_named_target_retry_delay_s_)));
      }
    }
    RCLCPP_ERROR(get_logger(), "Arm named target action failed after retry attempts");
    return 16;
  }

  int dispatch_arm_pose_goal(const std::optional<geometry_msgs::msg::PoseStamped> & arm_target)
  {
    if (!arm_target) {
      return 6;
    }
    RCLCPP_INFO(get_logger(), "Waiting for arm action server '%s'", arm_action_name_.c_str());
    if (!arm_client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_s_))) {
      RCLCPP_ERROR(get_logger(), "Arm action server not available: %s", arm_action_name_.c_str());
      return 17;
    }

    MoveArm::Goal arm_goal;
    arm_goal.use_named_target = false;
    arm_goal.target_pose = *arm_target;
    auto goal_handle_future = arm_client_->async_send_goal(arm_goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for arm pose goal response");
      return 18;
    }
    const auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Arm pose goal rejected");
      return 19;
    }
    auto result_future = arm_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for arm pose result");
      return 20;
    }
    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_result.result->success)
    {
      RCLCPP_ERROR(get_logger(), "Arm pose action failed: %s", wrapped_result.result->message.c_str());
      return 21;
    }
    return 0;
  }

  std::optional<geometry_msgs::msg::PoseStamped> camera_pose_to_eef_pose(
    const geometry_msgs::msg::PoseStamped & camera_target)
  {
    geometry_msgs::msg::TransformStamped eef_to_camera_msg;
    try {
      eef_to_camera_msg = tf_buffer_.lookupTransform(
        end_effector_frame_, camera_frame_, tf2::TimePointZero,
        tf2::durationFromSec(wait_timeout_s_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(
        get_logger(), "Cannot lookup %s -> %s transform: %s",
        end_effector_frame_.c_str(), camera_frame_.c_str(), error.what());
      return std::nullopt;
    }

    tf2::Transform map_to_camera;
    tf2::fromMsg(camera_target.pose, map_to_camera);
    tf2::Transform eef_to_camera;
    tf2::fromMsg(eef_to_camera_msg.transform, eef_to_camera);
    const tf2::Transform map_to_eef = map_to_camera * eef_to_camera.inverse();

    geometry_msgs::msg::PoseStamped eef_target;
    eef_target.header = camera_target.header;
    eef_target.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    const auto map_to_eef_msg = tf2::toMsg(map_to_eef);
    eef_target.pose.position.x = map_to_eef_msg.translation.x;
    eef_target.pose.position.y = map_to_eef_msg.translation.y;
    eef_target.pose.position.z = map_to_eef_msg.translation.z;
    eef_target.pose.orientation = map_to_eef_msg.rotation;
    return eef_target;
  }

  std::string action_name_;
  std::string base_action_name_;
  std::string arm_action_name_;
  std::string execution_marker_topic_;
  std::string target_topic_;
  std::string named_target_topic_;
  std::string frame_id_;
  std::string robot_frame_;
  std::string end_effector_frame_;
  std::string camera_frame_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  double measurement_grid_size_;
  double wait_timeout_s_;
  double base_goal_result_timeout_s_;
  bool auto_explore_;
  int max_exploration_iterations_;
  double map_update_wait_s_;
  bool continue_on_motion_failure_;
  bool continue_base_on_stow_failure_;
  bool stow_arm_before_base_motion_;
  bool publish_debug_arm_targets_;
  bool retry_arm_after_base_fallback_;
  double base_goal_standoff_;
  int arm_named_target_retry_count_;
  double arm_named_target_retry_delay_s_;
  double stow_arm_wait_s_;
  double arm_goal_range_;
  double arm_goal_publish_delay_s_;
  rclcpp_action::Client<NbvPlanner>::SharedPtr client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr base_client_;
  rclcpp_action::Client<MoveArm>::SharedPtr arm_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr named_target_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr execution_marker_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<NbvArmTargetAdapter>();
  const int exit_code = node->run();
  rclcpp::shutdown();
  return exit_code;
}
