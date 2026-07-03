#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap/OcTree.h"
#include "octomap/Pointcloud.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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
    cloud_topic_(declare_parameter<std::string>("cloud_topic", "/wrist_camera/depth/points")),
    octomap_topic_(declare_parameter<std::string>("octomap_topic", "/octomap_binary")),
    occupied_cloud_topic_(
      declare_parameter<std::string>("occupied_cloud_topic", "/octomap_occupied_points")),
    resolution_(declare_parameter<double>("resolution", 0.05)),
    max_range_(declare_parameter<double>("max_range", 3.0)),
    min_z_(declare_parameter<double>("min_z", 0.05)),
    max_z_(declare_parameter<double>("max_z", 2.5)),
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

    octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic_, rclcpp::QoS(1).transient_local().reliable());
    occupied_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      occupied_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WristDepthOctomapNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Building wrist depth OctoMap in frame '%s' from '%s', publishing '%s'",
      map_frame_.c_str(), cloud_topic_.c_str(), octomap_topic_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cloud with empty frame_id");
      return;
    }

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

    sensor_msgs::msg::PointCloud2 map_cloud;
    tf2::doTransform(*message, map_cloud, transform);

    octomap::Pointcloud octomap_cloud;
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
      publish_octomap(message->header.stamp);
      publish_occupied_cloud(message->header.stamp);
    }
  }

  void publish_octomap(const rclcpp::Time & stamp)
  {
    octomap_msgs::msg::Octomap message;
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    if (!octomap_msgs::binaryMapToMsg(*tree_, message)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to serialize OctoMap");
      return;
    }
    octomap_publisher_->publish(message);
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
  std::string cloud_topic_;
  std::string octomap_topic_;
  std::string occupied_cloud_topic_;
  double resolution_;
  double max_range_;
  double min_z_;
  double max_z_;
  double transform_timeout_;
  int publish_every_n_clouds_;
  int clouds_integrated_ = 0;
  std::unique_ptr<octomap::OcTree> tree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_publisher_;
};
}  // namespace mobile_manipulator_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_navigation::WristDepthOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
