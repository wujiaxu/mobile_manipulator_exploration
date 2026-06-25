#ifndef MOBILE_MANIPULATOR_MOVEIT_BRIDGE__TRAJECTORY_SAMPLER_HPP_
#define MOBILE_MANIPULATOR_MOVEIT_BRIDGE__TRAJECTORY_SAMPLER_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace mobile_manipulator_moveit_bridge
{

inline constexpr std::size_t kArmJointCount = 7;
inline constexpr std::array<std::string_view, kArmJointCount> kArmJoints = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

struct ValidationResult
{
  bool ok{false};
  std::string message;
};

struct PreparedTrajectory
{
  std::vector<double> times;
  std::vector<std::array<double, kArmJointCount>> positions;
};

class TrajectorySampler
{
public:
  ValidationResult prepare(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    PreparedTrajectory & prepared) const;

  std::array<double, kArmJointCount> sample(
    const PreparedTrajectory & trajectory, double elapsed_seconds) const;
};

}  // namespace mobile_manipulator_moveit_bridge

#endif  // MOBILE_MANIPULATOR_MOVEIT_BRIDGE__TRAJECTORY_SAMPLER_HPP_
