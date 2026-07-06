#ifndef MOBILE_MANIPULATOR_FKIE_NBV__RRT_NODE_HPP_
#define MOBILE_MANIPULATOR_FKIE_NBV__RRT_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manipulator_fkie_nbv/nbv_parameters.hpp"
#include "rclcpp/time.hpp"

namespace mobile_manipulator_fkie_nbv
{

class RRTNode : public std::enable_shared_from_this<RRTNode>
{
public:
  explicit RRTNode(const NbvParameters & parameters);

  int node_id = 0;
  std::weak_ptr<RRTNode> parent_wptr;
  std::vector<std::shared_ptr<RRTNode>> children;
  double cubature_best_yaw = 0.0;

  geometry_msgs::msg::Pose get_pose() const;
  bool set_pose(const geometry_msgs::msg::Pose & pose);
  bool set_pose(const Eigen::Vector3d & pose);
  bool set_pose(const Eigen::Vector4d & pose);

  double get_gain() const;
  double get_gain_free_space() const;
  double get_cubature_best_yaw() const;
  double get_cost() const;
  double get_cost_till_root() const;
  double get_cost_to_parent() const;
  double get_score() const;

  void set_tree_costs();
  void set_gain(double free_space_gain, double measurement_gain, double visited_gain);
  void set_orientation(const geometry_msgs::msg::Quaternion & orientation);
  void compute_score();

  geometry_msgs::msg::Pose to_pose() const;
  geometry_msgs::msg::PoseStamped to_pose_stamped(const rclcpp::Time & stamp) const;
  RRTNode copy_to_rrt_node() const;
  std::string to_string(bool show_costs = true, bool show_utility = true) const;

private:
  double euclidean_distance_to_parent(const std::shared_ptr<RRTNode> & parent) const;
  void compute_score_weighted_sum();

  template<typename T>
  std::string fixed_string(const T value, const int precision = 5) const
  {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;
    return out.str();
  }

  const NbvParameters * parameters_;
  geometry_msgs::msg::Pose pose_;
  double gain_ = 0.0;
  double gain_free_space_ = 0.0;
  double gain_measurement_ = 0.0;
  double gain_visited_cell_ = 0.0;
  double cost_ = 0.0;
  double cost_till_root_ = 0.0;
  double cost_to_parent_ = 0.0;
  double score_ = 0.0;
};

}  // namespace mobile_manipulator_fkie_nbv

#endif  // MOBILE_MANIPULATOR_FKIE_NBV__RRT_NODE_HPP_
