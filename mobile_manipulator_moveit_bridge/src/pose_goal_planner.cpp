#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
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
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target while the arm is busy");
      return;
    }
    BusyReset reset(busy_);

    if (message->header.frame_id.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with an empty frame_id");
      return;
    }
    if (!finite_pose(message->pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with non-finite values");
      return;
    }

    const double quaternion_norm = std::sqrt(
      message->pose.orientation.x * message->pose.orientation.x +
      message->pose.orientation.y * message->pose.orientation.y +
      message->pose.orientation.z * message->pose.orientation.z +
      message->pose.orientation.w * message->pose.orientation.w);
    if (quaternion_norm < 1e-9) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting pose target with a zero quaternion");
      return;
    }

    geometry_msgs::msg::PoseStamped normalized = *message;
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
      return;
    }

    move_group_.setStartStateToCurrentState();
    if (!move_group_.setPoseTarget(target, kEndEffectorLink)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt rejected the link_eef pose target");
      move_group_.clearPoseTargets();
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_.plan(plan);
    if (!planning_result) {
      RCLCPP_ERROR(
        node_->get_logger(), "MoveIt failed to plan the requested link_eef pose");
      move_group_.clearPoseTargets();
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt pose plan succeeded; starting execution");

    const auto execution_result = move_group_.execute(plan);
    move_group_.clearPoseTargets();
    if (!execution_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt pose execution failed");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt pose execution succeeded");
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
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
