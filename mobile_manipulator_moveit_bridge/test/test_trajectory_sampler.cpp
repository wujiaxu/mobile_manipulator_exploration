#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "mobile_manipulator_moveit_bridge/trajectory_sampler.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using mobile_manipulator_moveit_bridge::PreparedTrajectory;
using mobile_manipulator_moveit_bridge::TrajectorySampler;
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

JointTrajectory valid_trajectory()
{
  JointTrajectory trajectory;
  trajectory.joint_names = {
    "joint7", "joint1", "joint6", "joint2", "joint5", "joint3", "joint4"};

  JointTrajectoryPoint first;
  first.positions = {0.7, 0.1, 0.6, 0.2, 0.5, 0.3, 0.4};
  first.time_from_start.sec = 1;

  JointTrajectoryPoint second;
  second.positions = {1.4, 0.2, 1.2, 0.4, 1.0, 0.6, 0.8};
  second.time_from_start.sec = 3;

  trajectory.points = {first, second};
  return trajectory;
}

TEST(TrajectorySamplerTest, AcceptsAndReordersAValidTrajectory)
{
  TrajectorySampler sampler;
  PreparedTrajectory prepared;
  const auto result = sampler.prepare(valid_trajectory(), prepared);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(prepared.positions.size(), 2U);
  EXPECT_EQ(
    prepared.positions.front(),
    (std::array<double, 7>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}));
}

TEST(TrajectorySamplerTest, RejectsInvalidJointSets)
{
  TrajectorySampler sampler;
  PreparedTrajectory prepared;

  auto missing = valid_trajectory();
  missing.joint_names.pop_back();
  for (auto & point : missing.points) {
    point.positions.pop_back();
  }
  EXPECT_FALSE(sampler.prepare(missing, prepared).ok);

  auto duplicate = valid_trajectory();
  duplicate.joint_names.back() = "joint1";
  EXPECT_FALSE(sampler.prepare(duplicate, prepared).ok);

  auto unknown = valid_trajectory();
  unknown.joint_names.back() = "gripper_joint";
  EXPECT_FALSE(sampler.prepare(unknown, prepared).ok);
}

TEST(TrajectorySamplerTest, RejectsInvalidPoints)
{
  TrajectorySampler sampler;
  PreparedTrajectory prepared;

  auto incomplete = valid_trajectory();
  incomplete.points.front().positions.pop_back();
  EXPECT_FALSE(sampler.prepare(incomplete, prepared).ok);

  auto non_finite = valid_trajectory();
  non_finite.points.front().positions.front() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sampler.prepare(non_finite, prepared).ok);

  auto non_increasing = valid_trajectory();
  non_increasing.points.back().time_from_start.sec = 1;
  EXPECT_FALSE(sampler.prepare(non_increasing, prepared).ok);

  auto outside_limit = valid_trajectory();
  outside_limit.points.front().positions[3] = 3.0;
  EXPECT_FALSE(sampler.prepare(outside_limit, prepared).ok);
}

TEST(TrajectorySamplerTest, InterpolatesAndClampsTrajectorySamples)
{
  TrajectorySampler sampler;
  PreparedTrajectory prepared;
  ASSERT_TRUE(sampler.prepare(valid_trajectory(), prepared).ok);

  EXPECT_DOUBLE_EQ(sampler.sample(prepared, 0.0)[0], 0.1);
  EXPECT_DOUBLE_EQ(sampler.sample(prepared, 1.0)[0], 0.1);
  EXPECT_NEAR(sampler.sample(prepared, 2.0)[0], 0.15, 1e-12);
  EXPECT_DOUBLE_EQ(sampler.sample(prepared, 3.0)[0], 0.2);
  EXPECT_DOUBLE_EQ(sampler.sample(prepared, 5.0)[0], 0.2);
}

}  // namespace
