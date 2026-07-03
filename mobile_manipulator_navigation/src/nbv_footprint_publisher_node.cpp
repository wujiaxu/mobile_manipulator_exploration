#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mobile_manipulator_navigation
{
class NbvFootprintPublisherNode : public rclcpp::Node
{
public:
  NbvFootprintPublisherNode()
  : Node("nbv_footprint_publisher"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    footprint_topic_(
      declare_parameter<std::string>(
        "footprint_topic", "/mobile_manipulator_mbf/global_costmap/footprint")),
    footprint_xy_(declare_parameter<std::vector<double>>(
      "footprint_xy", {-0.30, -0.25, -0.30, 0.25, 0.30, 0.25, 0.30, -0.25})),
    publish_rate_(declare_parameter<double>("publish_rate", 5.0)),
    transform_timeout_(declare_parameter<double>("transform_timeout", 0.5)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (footprint_xy_.size() < 6 || footprint_xy_.size() % 2 != 0) {
      throw std::runtime_error("footprint_xy must contain at least three x/y pairs");
    }
    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }
    if (transform_timeout_ <= 0.0) {
      throw std::runtime_error("transform_timeout must be positive");
    }

    footprint_publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(1).transient_local().reliable());
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NbvFootprintPublisherNode::publish_footprint, this));

    RCLCPP_INFO(
      get_logger(), "Publishing passive NBV footprint on '%s' in frame '%s'",
      footprint_topic_.c_str(), map_frame_.c_str());
  }

private:
  void publish_footprint()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for footprint transform to %s: %s",
        map_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PolygonStamped footprint;
    footprint.header.frame_id = map_frame_;
    footprint.header.stamp = get_clock()->now();
    footprint.polygon.points.reserve(footprint_xy_.size() / 2);

    const auto & q = transform.transform.rotation;
    const auto & t = transform.transform.translation;
    const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double r01 = 2.0 * (q.x * q.y - q.z * q.w);
    const double r10 = 2.0 * (q.x * q.y + q.z * q.w);
    const double r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    const double r20 = 2.0 * (q.x * q.z - q.y * q.w);
    const double r21 = 2.0 * (q.y * q.z + q.x * q.w);

    for (size_t i = 0; i < footprint_xy_.size(); i += 2) {
      const double x_base = footprint_xy_[i];
      const double y_base = footprint_xy_[i + 1];

      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(t.x + r00 * x_base + r01 * y_base);
      point.y = static_cast<float>(t.y + r10 * x_base + r11 * y_base);
      point.z = static_cast<float>(t.z + r20 * x_base + r21 * y_base);
      footprint.polygon.points.push_back(point);
    }

    footprint_publisher_->publish(footprint);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string footprint_topic_;
  std::vector<double> footprint_xy_;
  double publish_rate_;
  double transform_timeout_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mobile_manipulator_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_navigation::NbvFootprintPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
