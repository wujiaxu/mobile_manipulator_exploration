#include "mobile_manipulator_fkie_nbv/rrt_node.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace mobile_manipulator_fkie_nbv
{

RRTNode::RRTNode(const NbvParameters & parameters)
: parameters_(&parameters)
{
}

geometry_msgs::msg::Pose RRTNode::get_pose() const
{
  return pose_;
}

bool RRTNode::set_pose(const geometry_msgs::msg::Pose & pose)
{
  if (!std::isfinite(pose.position.x) ||
    !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) ||
    !std::isfinite(pose.orientation.w))
  {
    pose_ = geometry_msgs::msg::Pose();
    return false;
  }

  pose_ = pose;
  return true;
}

bool RRTNode::set_pose(const Eigen::Vector3d & pose)
{
  if (!std::isfinite(pose[0]) || !std::isfinite(pose[1]) || !std::isfinite(pose[2])) {
    pose_ = geometry_msgs::msg::Pose();
    return false;
  }

  pose_.position.x = pose[0];
  pose_.position.y = pose[1];
  pose_.position.z = pose[2];
  pose_.orientation.w = 1.0;
  return true;
}

bool RRTNode::set_pose(const Eigen::Vector4d & pose)
{
  if (!std::isfinite(pose[0]) || !std::isfinite(pose[1]) || !std::isfinite(pose[2])) {
    pose_ = geometry_msgs::msg::Pose();
    return false;
  }

  pose_.position.x = pose[0];
  pose_.position.y = pose[1];
  pose_.position.z = pose[2];

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, pose[3]);
  pose_.orientation.x = quaternion.x();
  pose_.orientation.y = quaternion.y();
  pose_.orientation.z = quaternion.z();
  pose_.orientation.w = quaternion.w();
  return true;
}

double RRTNode::get_gain() const
{
  return gain_;
}

double RRTNode::get_gain_free_space() const
{
  return gain_free_space_;
}

double RRTNode::get_cubature_best_yaw() const
{
  return cubature_best_yaw;
}

double RRTNode::get_cost() const
{
  return cost_;
}

double RRTNode::get_cost_till_root() const
{
  return cost_till_root_;
}

double RRTNode::get_cost_to_parent() const
{
  return cost_to_parent_;
}

double RRTNode::get_score() const
{
  return score_;
}

void RRTNode::set_tree_costs()
{
  cost_to_parent_ = 0.0;
  cost_till_root_ = 0.0;

  const auto parent = parent_wptr.lock();
  if (parent) {
    cost_to_parent_ = euclidean_distance_to_parent(parent);
    cost_till_root_ = cost_to_parent_ + parent->cost_till_root_;
  }
}

void RRTNode::set_gain(
  const double free_space_gain,
  const double measurement_gain,
  const double visited_gain)
{
  gain_free_space_ = free_space_gain;
  gain_measurement_ = measurement_gain;
  gain_visited_cell_ = visited_gain;

  const double gain_m = parameters_->utility_weight_measurement * gain_measurement_;
  const double gain_fs = parameters_->utility_weight_free_space * gain_free_space_;
  const double gain_v = parameters_->utility_weight_visited_cell * gain_visited_cell_;
  gain_ = gain_m + gain_fs + gain_v;
}

void RRTNode::set_orientation(const geometry_msgs::msg::Quaternion & orientation)
{
  pose_.orientation = orientation;
}

void RRTNode::compute_score()
{
  score_ = 0.0;
  compute_score_weighted_sum();
}

geometry_msgs::msg::Pose RRTNode::to_pose() const
{
  return pose_;
}

geometry_msgs::msg::PoseStamped RRTNode::to_pose_stamped(const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = stamp;
  goal.header.frame_id = parameters_->world_frame;
  goal.pose = pose_;
  return goal;
}

RRTNode RRTNode::copy_to_rrt_node() const
{
  RRTNode node(*parameters_);
  node.node_id = node_id;
  node.pose_ = pose_;
  node.gain_ = gain_;
  node.gain_free_space_ = gain_free_space_;
  node.gain_measurement_ = gain_measurement_;
  node.gain_visited_cell_ = gain_visited_cell_;
  node.cost_ = cost_;
  node.cost_till_root_ = cost_till_root_;
  node.cost_to_parent_ = cost_to_parent_;
  node.score_ = score_;
  node.cubature_best_yaw = cubature_best_yaw;
  return node;
}

std::string RRTNode::to_string(const bool show_costs, const bool show_utility) const
{
  std::string text;
  text += "id:[" + std::to_string(node_id) + "] ";
  text += "(" + fixed_string(pose_.position.x, 2) + ",";
  text += fixed_string(pose_.position.y, 2) + ",";
  text += fixed_string(pose_.position.z, 2) + ") ";
  if (show_utility) {
    text += "gain: [" + fixed_string(gain_free_space_, 3) + "] ";
    text += "score: [" + fixed_string(score_, 3) + "] ";
  } else if (show_costs) {
    text += "cost_till_root: [" + fixed_string(cost_till_root_, 3) + "] ";
  }
  return text;
}

double RRTNode::euclidean_distance_to_parent(const std::shared_ptr<RRTNode> & parent) const
{
  const Eigen::Vector3d difference(
    pose_.position.x - parent->pose_.position.x,
    pose_.position.y - parent->pose_.position.y,
    pose_.position.z - parent->pose_.position.z);
  return difference.norm();
}

void RRTNode::compute_score_weighted_sum()
{
  const auto parent = parent_wptr.lock();

  score_ = 0.0;
  if (parent) {
    cost_to_parent_ = euclidean_distance_to_parent(parent);
    cost_till_root_ = cost_to_parent_ + parent->cost_till_root_;
    cost_ = cost_till_root_;
    score_ = get_gain() - parameters_->utility_weight_euclidean_cost * get_cost();
  } else {
    cost_ = 0.0;
    score_ += parameters_->utility_weight_measurement * gain_measurement_;
    score_ += parameters_->utility_weight_free_space * gain_free_space_;
  }
}

}  // namespace mobile_manipulator_fkie_nbv
