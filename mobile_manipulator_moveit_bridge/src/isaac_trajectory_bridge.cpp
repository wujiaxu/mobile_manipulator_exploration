#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "mobile_manipulator_moveit_bridge/trajectory_sampler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace mobile_manipulator_moveit_bridge
{

namespace
{

constexpr char kActionName[] = "/arm_controller/follow_joint_trajectory";
constexpr char kCommandTopic[] = "/arm_joint_commands";
constexpr char kStateTopic[] = "/joint_states";

std::vector<std::string> arm_joint_names()
{
  std::vector<std::string> names;
  names.reserve(kArmJointCount);
  for (const auto name : kArmJoints) {
    names.emplace_back(name);
  }
  return names;
}

builtin_interfaces::msg::Duration duration_message(double seconds)
{
  const auto nanoseconds = rclcpp::Duration::from_seconds(
    std::max(0.0, seconds)).nanoseconds();
  builtin_interfaces::msg::Duration message;
  message.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  message.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return message;
}

}  // namespace

class IsaacTrajectoryBridge : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  IsaacTrajectoryBridge()
  : Node("isaac_trajectory_bridge"), joint_names_(arm_joint_names())
  {
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    goal_position_tolerance_ =
      declare_parameter<double>("goal_position_tolerance", 0.02);
    path_position_tolerance_ =
      declare_parameter<double>("path_position_tolerance", 0.25);
    goal_time_tolerance_ = declare_parameter<double>("goal_time_tolerance", 2.0);

    if (publish_rate_ <= 0.0 || goal_position_tolerance_ <= 0.0 ||
      path_position_tolerance_ <= 0.0 || goal_time_tolerance_ < 0.0)
    {
      throw std::invalid_argument("trajectory bridge tolerances and publish rate must be positive");
    }

    command_publisher_ = create_publisher<sensor_msgs::msg::JointState>(kCommandTopic, 10);
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      kStateTopic, rclcpp::SensorDataQoS(),
      std::bind(&IsaacTrajectoryBridge::state_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, kActionName,
      std::bind(
        &IsaacTrajectoryBridge::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &IsaacTrajectoryBridge::handle_cancel, this, std::placeholders::_1),
      std::bind(
        &IsaacTrajectoryBridge::handle_accepted, this, std::placeholders::_1));
  }

  ~IsaacTrajectoryBridge() override
  {
    shutting_down_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    PreparedTrajectory prepared;
    const auto validation = sampler_.prepare(goal->trajectory, prepared);
    if (!validation.ok) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory: %s", validation.message.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory while another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(&IsaacTrajectoryBridge::execute, this, goal_handle);
  }

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::array<double, kArmJointCount> measured{};
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      const auto name = std::find(
        message->name.begin(), message->name.end(), kArmJoints[index]);
      if (name == message->name.end()) {
        return;
      }
      const auto source_index = static_cast<std::size_t>(
        std::distance(message->name.begin(), name));
      if (source_index >= message->position.size() ||
        !std::isfinite(message->position[source_index]))
      {
        return;
      }
      measured[index] = message->position[source_index];
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    measured_positions_ = measured;
    has_measured_state_ = true;
  }

  bool measured_state(std::array<double, kArmJointCount> & positions) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_measured_state_) {
      return false;
    }
    positions = measured_positions_;
    return true;
  }

  void publish_command(const std::array<double, kArmJointCount> & positions)
  {
    sensor_msgs::msg::JointState command;
    command.header.stamp = now();
    command.name = joint_names_;
    command.position.assign(positions.begin(), positions.end());
    command_publisher_->publish(command);
  }

  static double maximum_error(
    const std::array<double, kArmJointCount> & desired,
    const std::array<double, kArmJointCount> & actual)
  {
    double result = 0.0;
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      result = std::max(result, std::abs(desired[index] - actual[index]));
    }
    return result;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::array<double, kArmJointCount> & desired, double elapsed_seconds)
  {
    std::array<double, kArmJointCount> actual{};
    if (!measured_state(actual)) {
      return;
    }

    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->header.stamp = now();
    feedback->joint_names = joint_names_;
    feedback->desired.positions.assign(desired.begin(), desired.end());
    feedback->actual.positions.assign(actual.begin(), actual.end());
    feedback->error.positions.resize(kArmJointCount);
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      feedback->error.positions[index] = desired[index] - actual[index];
    }
    feedback->desired.time_from_start = duration_message(elapsed_seconds);
    feedback->actual.time_from_start = duration_message(elapsed_seconds);
    feedback->error.time_from_start = duration_message(elapsed_seconds);
    goal_handle->publish_feedback(feedback);
  }

  void hold_measured_position()
  {
    std::array<double, kArmJointCount> actual{};
    if (measured_state(actual)) {
      publish_command(actual);
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    PreparedTrajectory trajectory;
    const auto validation = sampler_.prepare(goal_handle->get_goal()->trajectory, trajectory);
    if (!validation.ok) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = validation.message;
      goal_handle->abort(result);
      goal_active_.store(false);
      return;
    }

    rclcpp::WallRate rate(publish_rate_);
    const rclcpp::Time started = now();
    const double final_time = trajectory.times.back();

    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        hold_measured_position();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "trajectory canceled; holding measured position";
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      const double elapsed = (now() - started).seconds();
      const auto desired = sampler_.sample(trajectory, elapsed);
      publish_command(desired);
      publish_feedback(goal_handle, desired, elapsed);

      std::array<double, kArmJointCount> actual{};
      if (elapsed > 0.1 && measured_state(actual) &&
        maximum_error(desired, actual) > path_position_tolerance_)
      {
        hold_measured_position();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "measured arm position exceeded path_position_tolerance";
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      if (elapsed >= final_time) {
        break;
      }
      rate.sleep();
    }

    const auto final_positions = trajectory.positions.back();
    const rclcpp::Time convergence_deadline =
      now() + rclcpp::Duration::from_seconds(goal_time_tolerance_);
    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        hold_measured_position();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "trajectory canceled; holding measured position";
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      publish_command(final_positions);
      std::array<double, kArmJointCount> actual{};
      if (measured_state(actual) &&
        maximum_error(final_positions, actual) <= goal_position_tolerance_)
      {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "trajectory reached goal tolerance";
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }
      if (now() >= convergence_deadline) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "arm did not reach goal_position_tolerance before timeout";
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }
      rate.sleep();
    }

    goal_active_.store(false);
  }

  TrajectorySampler sampler_;
  std::vector<std::string> joint_names_;
  double publish_rate_{100.0};
  double goal_position_tolerance_{0.02};
  double path_position_tolerance_{0.25};
  double goal_time_tolerance_{2.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  mutable std::mutex state_mutex_;
  std::array<double, kArmJointCount> measured_positions_{};
  bool has_measured_state_{false};

  std::atomic_bool goal_active_{false};
  std::atomic_bool shutting_down_{false};
  std::mutex worker_mutex_;
  std::thread worker_;
};

}  // namespace mobile_manipulator_moveit_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<mobile_manipulator_moveit_bridge::IsaacTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
