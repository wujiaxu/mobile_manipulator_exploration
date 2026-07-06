#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manipulator_moveit_bridge/action/move_arm.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mobile_manipulator_moveit_bridge
{

namespace
{

constexpr char kTargetTopic[] = "/arm_target_pose";
constexpr char kNamedTargetTopic[] = "/arm_named_target";
constexpr char kMoveArmAction[] = "/move_arm";
constexpr char kPlanningGroup[] = "xarm7";
constexpr char kEndEffectorLink[] = "link_eef";
constexpr char kPlanningFrame[] = "base_link";

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

class BusyReset
{
public:
  explicit BusyReset(std::atomic_bool & busy) : busy_(busy) {}
  ~BusyReset() {busy_.store(false);}

  BusyReset(const BusyReset &) = delete;
  BusyReset & operator=(const BusyReset &) = delete;

private:
  std::atomic_bool & busy_;
};

}  // namespace

class PoseGoalPlanner
{
public:
  using MoveArm = mobile_manipulator_moveit_bridge::action::MoveArm;
  using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

  explicit PoseGoalPlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    move_group_(node_, kPlanningGroup)
  {
    move_group_.setPoseReferenceFrame(kPlanningFrame);
    if (!move_group_.setEndEffectorLink(kEndEffectorLink)) {
      throw std::runtime_error("MoveIt rejected link_eef as the end-effector link");
    }
    move_group_.setPlanningTime(5.0);
    move_group_.setNumPlanningAttempts(5);

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      kTargetTopic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        target_callback(message);
      },
      options);
    named_target_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      kNamedTargetTopic, 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        execute_named_target(message->data);
      },
      options);
    action_server_ = rclcpp_action::create_server<MoveArm>(
      node_,
      kMoveArmAction,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const MoveArm::Goal> goal)
      {
        return handle_goal(goal);
      },
      [this](const std::shared_ptr<GoalHandleMoveArm>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveArm> goal_handle) {
        std::thread{[this, goal_handle]() {execute_move_arm_goal(goal_handle);}}.detach();
      });
  }

private:
  rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<const MoveArm::Goal> & goal)
  {
    if (goal->use_named_target) {
      if (goal->named_target.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Rejecting MoveArm goal with empty named target");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    if (goal->target_pose.header.frame_id.empty() || !finite_pose(goal->target_pose.pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting MoveArm pose goal with invalid target pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleMoveArm> & goal_handle,
    const std::string & phase) const
  {
    auto feedback = std::make_shared<MoveArm::Feedback>();
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
  }

  void finish_action(
    const std::shared_ptr<GoalHandleMoveArm> & goal_handle,
    const bool success,
    const std::string & message) const
  {
    auto result = std::make_shared<MoveArm::Result>();
    result->success = success;
    result->message = message;
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void execute_move_arm_goal(const std::shared_ptr<GoalHandleMoveArm> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    if (goal->use_named_target) {
      execute_named_target(goal->named_target, goal_handle);
    } else {
      execute_pose_target(goal->target_pose, goal_handle);
    }
  }

  bool execute_named_target(
    const std::string & target_name,
    const std::shared_ptr<GoalHandleMoveArm> & goal_handle = nullptr)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting named target '%s' while the arm is busy", target_name.c_str());
      if (goal_handle) {
        finish_action(goal_handle, false, "arm is busy");
      }
      return false;
    }
    BusyReset reset(busy_);

    if (goal_handle) {
      publish_feedback(goal_handle, "planning_named_target");
    }
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setNamedTarget(target_name)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt rejected named target '%s'", target_name.c_str());
      move_group_.clearPoseTargets();
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt rejected named target");
      }
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_.plan(plan);
    if (!planning_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt failed to plan named target '%s'", target_name.c_str());
      move_group_.clearPoseTargets();
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt failed to plan named target");
      }
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "MoveIt named target '%s' plan succeeded; starting execution", target_name.c_str());
    if (goal_handle) {
      publish_feedback(goal_handle, "executing_named_target");
    }
    const auto execution_result = move_group_.execute(plan);
    move_group_.clearPoseTargets();
    if (!execution_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt named target '%s' execution failed", target_name.c_str());
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt named target execution failed");
      }
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt named target '%s' execution succeeded", target_name.c_str());
    if (goal_handle) {
      finish_action(goal_handle, true, "MoveIt named target execution succeeded");
    }
    return true;
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    (void)execute_pose_target(*message);
  }

  bool execute_pose_target(
    const geometry_msgs::msg::PoseStamped & message,
    const std::shared_ptr<GoalHandleMoveArm> & goal_handle = nullptr)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target while the arm is busy");
      if (goal_handle) {
        finish_action(goal_handle, false, "arm is busy");
      }
      return false;
    }
    BusyReset reset(busy_);

    if (message.header.frame_id.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with an empty frame_id");
      if (goal_handle) {
        finish_action(goal_handle, false, "empty target frame");
      }
      return false;
    }
    if (!finite_pose(message.pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with non-finite values");
      if (goal_handle) {
        finish_action(goal_handle, false, "non-finite target pose");
      }
      return false;
    }

    const double quaternion_norm = std::sqrt(
      message.pose.orientation.x * message.pose.orientation.x +
      message.pose.orientation.y * message.pose.orientation.y +
      message.pose.orientation.z * message.pose.orientation.z +
      message.pose.orientation.w * message.pose.orientation.w);
    if (quaternion_norm < 1e-9) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with a zero quaternion");
      if (goal_handle) {
        finish_action(goal_handle, false, "zero target quaternion");
      }
      return false;
    }

    geometry_msgs::msg::PoseStamped normalized = message;
    normalized.pose.orientation.x /= quaternion_norm;
    normalized.pose.orientation.y /= quaternion_norm;
    normalized.pose.orientation.z /= quaternion_norm;
    normalized.pose.orientation.w /= quaternion_norm;

    geometry_msgs::msg::PoseStamped target;
    try {
      target = tf_buffer_.transform(
        normalized, kPlanningFrame, tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cannot transform pose target from %s to %s: %s",
        normalized.header.frame_id.c_str(), kPlanningFrame, error.what());
      if (goal_handle) {
        finish_action(goal_handle, false, "cannot transform pose target");
      }
      return false;
    }

    if (goal_handle) {
      publish_feedback(goal_handle, "planning_pose_target");
    }
    move_group_.setStartStateToCurrentState();
    if (!move_group_.setPoseTarget(target, kEndEffectorLink)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt rejected the link_eef pose target");
      move_group_.clearPoseTargets();
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt rejected pose target");
      }
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_.plan(plan);
    if (!planning_result) {
      RCLCPP_ERROR(
        node_->get_logger(), "MoveIt failed to plan the requested link_eef pose");
      move_group_.clearPoseTargets();
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt failed to plan pose target");
      }
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt pose plan succeeded; starting execution");

    if (goal_handle) {
      publish_feedback(goal_handle, "executing_pose_target");
    }
    const auto execution_result = move_group_.execute(plan);
    move_group_.clearPoseTargets();
    if (!execution_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt pose execution failed");
      if (goal_handle) {
        finish_action(goal_handle, false, "MoveIt pose execution failed");
      }
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt pose execution succeeded");
    if (goal_handle) {
      finish_action(goal_handle, true, "MoveIt pose execution succeeded");
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_target_subscription_;
  rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
  std::atomic_bool busy_{false};
};

}  // namespace mobile_manipulator_moveit_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pose_goal_planner", options);
  auto planner = std::make_shared<mobile_manipulator_moveit_bridge::PoseGoalPlanner>(node);
  (void)planner;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
