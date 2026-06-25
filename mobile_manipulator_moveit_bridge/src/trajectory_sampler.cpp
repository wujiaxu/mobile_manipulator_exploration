#include "mobile_manipulator_moveit_bridge/trajectory_sampler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace mobile_manipulator_moveit_bridge
{

namespace
{

constexpr std::array<double, kArmJointCount> kLowerLimits = {
  -6.283185307179586, -2.059, -6.283185307179586, -0.19198,
  -6.283185307179586, -1.69297, -6.283185307179586};
constexpr std::array<double, kArmJointCount> kUpperLimits = {
  6.283185307179586, 2.0944, 6.283185307179586, 3.927,
  6.283185307179586, 3.141592653589793, 6.283185307179586};

double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

ValidationResult error(const std::string & message)
{
  return {false, message};
}

}  // namespace

ValidationResult TrajectorySampler::prepare(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  PreparedTrajectory & prepared) const
{
  prepared = PreparedTrajectory{};

  if (trajectory.joint_names.size() != kArmJointCount) {
    return error("trajectory must contain exactly seven arm joints");
  }
  if (trajectory.points.empty()) {
    return error("trajectory must contain at least one point");
  }

  const std::unordered_set<std::string> unique_names(
    trajectory.joint_names.begin(), trajectory.joint_names.end());
  if (unique_names.size() != kArmJointCount) {
    return error("trajectory contains duplicate joint names");
  }

  std::array<std::size_t, kArmJointCount> source_indices{};
  for (std::size_t canonical_index = 0; canonical_index < kArmJointCount;
    ++canonical_index)
  {
    const auto source = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(),
      kArmJoints[canonical_index]);
    if (source == trajectory.joint_names.end()) {
      return error("trajectory joint set does not match the xArm7 joints");
    }
    source_indices[canonical_index] = static_cast<std::size_t>(
      std::distance(trajectory.joint_names.begin(), source));
  }

  PreparedTrajectory candidate;
  candidate.times.reserve(trajectory.points.size());
  candidate.positions.reserve(trajectory.points.size());
  double previous_time = -std::numeric_limits<double>::infinity();

  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points[point_index];
    if (point.positions.size() != kArmJointCount) {
      return error("every trajectory point must contain seven positions");
    }

    const double point_time = seconds(point.time_from_start);
    if (!std::isfinite(point_time) || point_time < 0.0 || point_time <= previous_time) {
      return error("trajectory times must be finite, nonnegative, and strictly increasing");
    }
    previous_time = point_time;

    std::array<double, kArmJointCount> ordered_positions{};
    for (std::size_t joint_index = 0; joint_index < kArmJointCount; ++joint_index) {
      const double position = point.positions[source_indices[joint_index]];
      if (!std::isfinite(position)) {
        return error("trajectory contains a non-finite position");
      }
      if (position < kLowerLimits[joint_index] || position > kUpperLimits[joint_index]) {
        std::ostringstream message;
        message << kArmJoints[joint_index] << " position is outside its URDF limits";
        return error(message.str());
      }
      ordered_positions[joint_index] = position;
    }

    candidate.times.push_back(point_time);
    candidate.positions.push_back(ordered_positions);
  }

  prepared = std::move(candidate);
  return {true, {}};
}

std::array<double, kArmJointCount> TrajectorySampler::sample(
  const PreparedTrajectory & trajectory, double elapsed_seconds) const
{
  if (trajectory.times.empty() || trajectory.positions.empty()) {
    return {};
  }
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= trajectory.times.front()) {
    return trajectory.positions.front();
  }
  if (elapsed_seconds >= trajectory.times.back()) {
    return trajectory.positions.back();
  }

  const auto upper = std::upper_bound(
    trajectory.times.begin(), trajectory.times.end(), elapsed_seconds);
  const auto upper_index = static_cast<std::size_t>(
    std::distance(trajectory.times.begin(), upper));
  const auto lower_index = upper_index - 1;
  const double segment_duration =
    trajectory.times[upper_index] - trajectory.times[lower_index];
  const double ratio =
    (elapsed_seconds - trajectory.times[lower_index]) / segment_duration;

  std::array<double, kArmJointCount> result{};
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    const double lower = trajectory.positions[lower_index][index];
    const double upper_position = trajectory.positions[upper_index][index];
    result[index] = lower + ratio * (upper_position - lower);
  }
  return result;
}

}  // namespace mobile_manipulator_moveit_bridge
