#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace mobile_manipulator_moveit_bridge
{
class OctomapVoxelPlanningSceneBridge : public rclcpp::Node
{
public:
  OctomapVoxelPlanningSceneBridge()
  : Node("octomap_voxel_planning_scene_bridge"),
    occupied_cloud_topic_(declare_parameter<std::string>(
      "occupied_cloud_topic", "/octomap_occupied_points")),
    planning_scene_topic_(declare_parameter<std::string>("planning_scene_topic", "/planning_scene")),
    apply_planning_scene_service_(
      declare_parameter<std::string>("apply_planning_scene_service", "/apply_planning_scene")),
    planning_frame_(declare_parameter<std::string>("planning_frame", "base_link")),
    workspace_min_x_(declare_parameter<double>("workspace_min_x", -0.5)),
    workspace_max_x_(declare_parameter<double>("workspace_max_x", 1.2)),
    workspace_min_y_(declare_parameter<double>("workspace_min_y", -0.8)),
    workspace_max_y_(declare_parameter<double>("workspace_max_y", 0.8)),
    workspace_min_z_(declare_parameter<double>("workspace_min_z", 0.0)),
    workspace_max_z_(declare_parameter<double>("workspace_max_z", 1.8)),
    voxel_box_size_(declare_parameter<double>("voxel_box_size", 0.05)),
    max_boxes_(declare_parameter<int>("max_boxes", 2500)),
    publish_every_n_clouds_(declare_parameter<int>("publish_every_n_clouds", 3)),
    transform_timeout_(declare_parameter<double>("transform_timeout", 0.5)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (workspace_min_x_ >= workspace_max_x_ || workspace_min_y_ >= workspace_max_y_ ||
      workspace_min_z_ >= workspace_max_z_)
    {
      throw std::runtime_error("workspace min values must be smaller than max values");
    }
    if (voxel_box_size_ <= 0.0) {
      throw std::runtime_error("voxel_box_size must be positive");
    }
    if (max_boxes_ <= 0) {
      throw std::runtime_error("max_boxes must be positive");
    }
    if (publish_every_n_clouds_ <= 0) {
      throw std::runtime_error("publish_every_n_clouds must be positive");
    }
    if (transform_timeout_ <= 0.0) {
      throw std::runtime_error("transform_timeout must be positive");
    }

    planning_scene_publisher_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      planning_scene_topic_, rclcpp::QoS(1).reliable());
    apply_planning_scene_client_ =
      create_client<moveit_msgs::srv::ApplyPlanningScene>(apply_planning_scene_service_);
    occupied_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      occupied_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OctomapVoxelPlanningSceneBridge::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing MoveIt PlanningScene voxels from '%s' into frame '%s'",
      occupied_cloud_topic_.c_str(), planning_frame_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cloud with empty frame_id");
      return;
    }

    ++clouds_seen_;
    if (clouds_seen_ % publish_every_n_clouds_ != 0) {
      return;
    }

    const std::string source_frame = cloud->header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        planning_frame_, source_frame, cloud->header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for voxel cloud transform to %s: %s",
        planning_frame_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 planning_cloud;
    tf2::doTransform(*cloud, planning_cloud, transform);
    publish_planning_scene(planning_cloud);
  }

  void publish_planning_scene(const sensor_msgs::msg::PointCloud2 & planning_cloud)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = "nbv_octomap_occupied_voxels";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_object.primitives.reserve(static_cast<size_t>(max_boxes_));
    collision_object.primitive_poses.reserve(static_cast<size_t>(max_boxes_));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(planning_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(planning_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(planning_cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (!inside_workspace(x, y, z)) {
        continue;
      }

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {voxel_box_size_, voxel_box_size_, voxel_box_size_};

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation.w = 1.0;

      collision_object.primitives.push_back(box);
      collision_object.primitive_poses.push_back(pose);
      if (static_cast<int>(collision_object.primitives.size()) >= max_boxes_) {
        break;
      }
    }

    if (collision_object.primitives.empty()) {
      collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    }

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(collision_object);
    planning_scene_publisher_->publish(scene);
    if (apply_planning_scene_client_->service_is_ready()) {
      auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      request->scene = scene;
      apply_planning_scene_client_->async_send_request(request);
    }
  }

  bool inside_workspace(const double x, const double y, const double z) const
  {
    return x >= workspace_min_x_ && x <= workspace_max_x_ &&
           y >= workspace_min_y_ && y <= workspace_max_y_ &&
           z >= workspace_min_z_ && z <= workspace_max_z_;
  }

  std::string occupied_cloud_topic_;
  std::string planning_scene_topic_;
  std::string apply_planning_scene_service_;
  std::string planning_frame_;
  double workspace_min_x_;
  double workspace_max_x_;
  double workspace_min_y_;
  double workspace_max_y_;
  double workspace_min_z_;
  double workspace_max_z_;
  double voxel_box_size_;
  int max_boxes_;
  int publish_every_n_clouds_;
  double transform_timeout_;
  int clouds_seen_ = 0;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_subscription_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
};
}  // namespace mobile_manipulator_moveit_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<mobile_manipulator_moveit_bridge::OctomapVoxelPlanningSceneBridge>());
  rclcpp::shutdown();
  return 0;
}
