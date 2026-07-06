#ifndef MOBILE_MANIPULATOR_FKIE_NBV__NBV_UTILS_HPP_
#define MOBILE_MANIPULATOR_FKIE_NBV__NBV_UTILS_HPP_

#include <cmath>
#include <random>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manipulator_fkie_nbv/nbv_parameters.hpp"
#include "octomap/octomap.h"

namespace mobile_manipulator_fkie_nbv
{

inline Eigen::Vector3d generate_random_sample(
  const double radius, const NbvParameters & parameters, std::mt19937 & random_engine)
{
  Eigen::Vector3d sample;
  do {
    std::uniform_real_distribution<double> theta_distribution(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> noise_distribution(0.0, 1.0);
    const double theta = theta_distribution(random_engine);
    const double noise = noise_distribution(random_engine);
    const double phi = std::acos((2.0 * noise) - 1.0);
    const double sampled_radius = radius * std::cbrt(noise_distribution(random_engine));

    sample[0] = sampled_radius * std::sin(phi) * std::cos(theta);
    sample[1] = sampled_radius * std::sin(phi) * std::sin(theta);
    sample[2] = sampled_radius * std::cos(phi);
  } while (
    sample[2] < parameters.arm_height_min || sample[2] > parameters.arm_height_max);

  return sample;
}

inline bool is_sample_in_polygon(
  const Eigen::Vector3d & sample,
  const geometry_msgs::msg::Polygon & polygon,
  const double max_height,
  const NbvParameters & parameters)
{
  bool inside = false;
  if (sample[2] < parameters.arm_height_min || sample[2] > max_height) {
    return false;
  }

  for (size_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++) {
    if ((polygon.points[i].y > sample[1]) != (polygon.points[j].y > sample[1]) &&
      (sample[0] <
      (polygon.points[j].x - polygon.points[i].x) *
      (sample[1] - polygon.points[i].y) /
      (polygon.points[j].y - polygon.points[i].y) +
      polygon.points[i].x))
    {
      inside = !inside;
    }
  }

  return inside;
}

inline bool is_sample_in_polygon(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Polygon & polygon,
  const double max_height,
  const NbvParameters & parameters)
{
  const Eigen::Vector3d sample(pose.position.x, pose.position.y, pose.position.z);
  return is_sample_in_polygon(sample, polygon, max_height, parameters);
}

inline float cylinder_caps_first(
  const octomap::point3d & start,
  const octomap::point3d & end,
  const float length_squared,
  const float radius_squared,
  const octomap::point3d & point)
{
  const float dx = end.x() - start.x();
  const float dy = end.y() - start.y();
  const float dz = end.z() - start.z();

  const float pdx = point.x() - start.x();
  const float pdy = point.y() - start.y();
  const float pdz = point.z() - start.z();

  const float dot = pdx * dx + pdy * dy + pdz * dz;
  if (dot < 0.0F || dot > length_squared) {
    return -1.0F;
  }

  const float distance_squared =
    (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / length_squared;
  if (distance_squared > radius_squared) {
    return -1.0F;
  }
  return distance_squared;
}

inline double distance_pose_stamped(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double linear_curve_length(const std::vector<geometry_msgs::msg::PoseStamped> & points)
{
  if (points.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    sum += distance_pose_stamped(points[i - 1], points[i]);
  }
  return sum;
}

inline Eigen::Vector3d pose_to_eigen_vector3d(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

}  // namespace mobile_manipulator_fkie_nbv

#endif  // MOBILE_MANIPULATOR_FKIE_NBV__NBV_UTILS_HPP_
