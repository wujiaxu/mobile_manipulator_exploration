#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap/OcTree.h"
#include "octomap/Pointcloud.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace mobile_manipulator_navigation
{
class WristDepthOctomapNode : public rclcpp::Node
{
public:
  WristDepthOctomapNode()
  : Node("wrist_depth_octomap"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    camera_frame_(declare_parameter<std::string>("camera_frame", "wrist_camera_color_optical_frame")),
    cloud_topic_(declare_parameter<std::string>("cloud_topic", "/wrist_camera/depth/points")),
    point_cloud_alias_topic_(
      declare_parameter<std::string>("point_cloud_alias_topic", "/realsense/depth/points2")),
    full_octomap_topic_(declare_parameter<std::string>("full_octomap_topic", "/octomap_full")),
    binary_octomap_topic_(declare_parameter<std::string>("binary_octomap_topic", "/octomap_binary")),
    occupied_cloud_topic_(
      declare_parameter<std::string>("occupied_cloud_topic", "/octomap_occupied_points")),
    camera_pose_topic_(declare_parameter<std::string>("camera_pose_topic", "/camera_pose")),
    resolution_(declare_parameter<double>("resolution", 0.05)),
    max_range_(declare_parameter<double>("max_range", 3.0)),
    min_z_(declare_parameter<double>("min_z", 0.05)),
    max_z_(declare_parameter<double>("max_z", 2.5)),
    self_filter_enabled_(declare_parameter<bool>("self_filter_enabled", true)),
    self_filter_frame_(declare_parameter<std::string>("self_filter_frame", "base_link")),
    self_filter_min_x_(declare_parameter<double>("self_filter_min_x", -0.45)),
    self_filter_max_x_(declare_parameter<double>("self_filter_max_x", 0.45)),
    self_filter_min_y_(declare_parameter<double>("self_filter_min_y", -0.35)),
    self_filter_max_y_(declare_parameter<double>("self_filter_max_y", 0.35)),
    self_filter_min_z_(declare_parameter<double>("self_filter_min_z", -0.10)),
    self_filter_max_z_(declare_parameter<double>("self_filter_max_z", 1.60)),
    transform_timeout_(declare_parameter<double>("transform_timeout", 0.5)),
    publish_every_n_clouds_(declare_parameter<int>("publish_every_n_clouds", 1)),
    tree_(std::make_unique<octomap::OcTree>(resolution_)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be positive");
    }
    if (max_range_ <= 0.0) {
      throw std::runtime_error("max_range must be positive");
    }
    if (publish_every_n_clouds_ < 1) {
      throw std::runtime_error("publish_every_n_clouds must be >= 1");
    }
    if (self_filter_min_x_ >= self_filter_max_x_ ||
      self_filter_min_y_ >= self_filter_max_y_ ||
      self_filter_min_z_ >= self_filter_max_z_)
    {
      throw std::runtime_error("self filter min bounds must be smaller than max bounds");
    }

    point_cloud_alias_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      point_cloud_alias_topic_, rclcpp::SensorDataQoS());
    full_octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      full_octomap_topic_, rclcpp::QoS(1).transient_local().reliable());
    binary_octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      binary_octomap_topic_, rclcpp::QoS(1).transient_local().reliable());
    occupied_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      occupied_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    camera_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      camera_pose_topic_, rclcpp::QoS(1).transient_local().reliable());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WristDepthOctomapNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Building wrist depth OctoMap in frame '%s' from '%s', publishing '%s'",
      map_frame_.c_str(), cloud_topic_.c_str(), full_octomap_topic_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cloud with empty frame_id");
      return;
    }

    point_cloud_alias_publisher_->publish(*message);

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, message->header.frame_id, message->header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for cloud transform to %s: %s",
        map_frame_.c_str(), ex.what());
      return;
    }
    publish_camera_pose(message->header.stamp);

    sensor_msgs::msg::PointCloud2 map_cloud;
    tf2::doTransform(*message, map_cloud, transform);

    geometry_msgs::msg::TransformStamped self_filter_transform;
    bool have_self_filter_transform = false;
    if (self_filter_enabled_) {
      try {
        self_filter_transform = tf_buffer_.lookupTransform(
          self_filter_frame_, map_frame_, message->header.stamp,
          rclcpp::Duration::from_seconds(transform_timeout_));
        have_self_filter_transform = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Waiting for self filter transform %s <- %s: %s",
          self_filter_frame_.c_str(), map_frame_.c_str(), ex.what());
      }
    }

    octomap::Pointcloud octomap_cloud;
    int self_filtered_points = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(map_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(map_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(map_cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < min_z_ || z > max_z_) {
        continue;
      }
      if (have_self_filter_transform && inside_self_filter_box(x, y, z, self_filter_transform)) {
        ++self_filtered_points;
        continue;
      }
      octomap_cloud.push_back(x, y, z);
    }

    if (octomap_cloud.size() == 0) {
      return;
    }

    const auto & translation = transform.transform.translation;
    const octomap::point3d sensor_origin(translation.x, translation.y, translation.z);
    tree_->insertPointCloud(octomap_cloud, sensor_origin, max_range_, true, true);
    tree_->updateInnerOccupancy();

    ++clouds_integrated_;
    if (clouds_integrated_ % publish_every_n_clouds_ == 0) {
      RCLCPP_DEBUG(
        get_logger(), "Integrated wrist cloud with %zu points after self-filtering %d points",
        octomap_cloud.size(), self_filtered_points);
      publish_octomaps(message->header.stamp);
      publish_occupied_cloud(message->header.stamp);
    }
  }

  bool inside_self_filter_box(
    const float map_x,
    const float map_y,
    const float map_z,
    const geometry_msgs::msg::TransformStamped & self_filter_transform) const
  {
    geometry_msgs::msg::PointStamped map_point;
    map_point.header.frame_id = map_frame_;
    map_point.point.x = map_x;
    map_point.point.y = map_y;
    map_point.point.z = map_z;

    geometry_msgs::msg::PointStamped self_point;
    tf2::doTransform(map_point, self_point, self_filter_transform);
    return self_point.point.x >= self_filter_min_x_ && self_point.point.x <= self_filter_max_x_ &&
           self_point.point.y >= self_filter_min_y_ && self_point.point.y <= self_filter_max_y_ &&
           self_point.point.z >= self_filter_min_z_ && self_point.point.z <= self_filter_max_z_;
  }

  void publish_camera_pose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped camera_transform;
    try {
      camera_transform = tf_buffer_.lookupTransform(
        map_frame_, camera_frame_, stamp, rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for camera pose transform to %s: %s",
        map_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = stamp;
    pose.pose.position.x = camera_transform.transform.translation.x;
    pose.pose.position.y = camera_transform.transform.translation.y;
    pose.pose.position.z = camera_transform.transform.translation.z;
    pose.pose.orientation = camera_transform.transform.rotation;
    camera_pose_publisher_->publish(pose);
  }

  void publish_octomaps(const rclcpp::Time & stamp)
  {
    octomap_msgs::msg::Octomap full_message;
    full_message.header.frame_id = map_frame_;
    full_message.header.stamp = stamp;
    if (!octomap_msgs::fullMapToMsg(*tree_, full_message)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to serialize full OctoMap");
      return;
    }
    full_octomap_publisher_->publish(full_message);

    octomap_msgs::msg::Octomap binary_message;
    binary_message.header.frame_id = map_frame_;
    binary_message.header.stamp = stamp;
    if (!octomap_msgs::binaryMapToMsg(*tree_, binary_message)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to serialize binary OctoMap");
      return;
    }
    binary_octomap_publisher_->publish(binary_message);
  }

  void publish_occupied_cloud(const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp = stamp;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    size_t occupied_count = 0;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (tree_->isNodeOccupied(*it)) {
        ++occupied_count;
      }
    }

    modifier.resize(occupied_count);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        continue;
      }
      *iter_x = static_cast<float>(it.getX());
      *iter_y = static_cast<float>(it.getY());
      *iter_z = static_cast<float>(it.getZ());
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    occupied_cloud_publisher_->publish(cloud);
  }

  std::string map_frame_;
  std::string camera_frame_;
  std::string cloud_topic_;
  std::string point_cloud_alias_topic_;
  std::string full_octomap_topic_;
  std::string binary_octomap_topic_;
  std::string occupied_cloud_topic_;
  std::string camera_pose_topic_;
  double resolution_;
  double max_range_;
  double min_z_;
  double max_z_;
  bool self_filter_enabled_;
  std::string self_filter_frame_;
  double self_filter_min_x_;
  double self_filter_max_x_;
  double self_filter_min_y_;
  double self_filter_max_y_;
  double self_filter_min_z_;
  double self_filter_max_z_;
  double transform_timeout_;
  int publish_every_n_clouds_;
  int clouds_integrated_ = 0;
  std::unique_ptr<octomap::OcTree> tree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_alias_publisher_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_octomap_publisher_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_octomap_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_;
};
}  // namespace mobile_manipulator_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_navigation::WristDepthOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
