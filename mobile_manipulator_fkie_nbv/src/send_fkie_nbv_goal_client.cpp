#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>

#include "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
using NbvPlanner = mobile_manipulator_fkie_msgs::action::NbvPlanner;
using GoalHandleNbvPlanner = rclcpp_action::ClientGoalHandle<NbvPlanner>;

class FkieNbvGoalClient : public rclcpp::Node
{
public:
  FkieNbvGoalClient()
  : Node("send_fkie_nbv_goal_client"),
    action_name_(declare_parameter<std::string>("action_name", "/nbv_rrt")),
    frame_id_(declare_parameter<std::string>("frame_id", "map")),
    min_x_(declare_parameter<double>("min_x", -3.0)),
    max_x_(declare_parameter<double>("max_x", 3.0)),
    min_y_(declare_parameter<double>("min_y", -3.0)),
    max_y_(declare_parameter<double>("max_y", 3.0)),
    min_z_(declare_parameter<double>("min_z", 0.4)),
    max_z_(declare_parameter<double>("max_z", 1.4)),
    measurement_grid_size_(declare_parameter<double>("measurement_grid_size", 0.5)),
    wait_timeout_s_(declare_parameter<double>("wait_timeout_s", 10.0))
  {
    client_ = rclcpp_action::create_client<NbvPlanner>(this, action_name_);
  }

  int run()
  {
    RCLCPP_INFO(get_logger(), "Waiting for action server '%s'", action_name_.c_str());
    if (!client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_s_))) {
      RCLCPP_ERROR(get_logger(), "Action server not available: %s", action_name_.c_str());
      return 1;
    }

    auto goal = make_goal();
    print_goal(goal);

    rclcpp_action::Client<NbvPlanner>::SendGoalOptions options;
    auto goal_handle_future = client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for goal response");
      return 2;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return 3;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted");
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed while waiting for result");
      return 4;
    }

    print_result(result_future.get());
    return 0;
  }

private:
  NbvPlanner::Goal make_goal() const
  {
    NbvPlanner::Goal goal;
    goal.header.frame_id = frame_id_;
    goal.boundary_id = 1;
    goal.boundary_source_type = "fixed_test_boundary";
    goal.boundary_frame_id = frame_id_;
    goal.boundary_min_z = min_z_;
    goal.boundary_max_z = max_z_;
    goal.boundary_x = {min_x_, max_x_, max_x_, min_x_};
    goal.boundary_y = {min_y_, min_y_, max_y_, max_y_};
    goal.estimations.header.frame_id = frame_id_;
    goal.estimations.grid_size = static_cast<float>(measurement_grid_size_);
    goal.arm_in_prepacked = false;
    return goal;
  }

  void print_goal(const NbvPlanner::Goal & goal) const
  {
    std::cout << "Sending FKIE NBV C++ goal:\n";
    std::cout << "  action_name: " << action_name_ << "\n";
    std::cout << "  frame_id: " << frame_id_ << "\n";
    std::cout << "  boundary: x=[" << goal.boundary_x[0] << ", " << goal.boundary_x[1] << "]";
    std::cout << ", y=[" << goal.boundary_y[0] << ", " << goal.boundary_y[2] << "]";
    std::cout << ", z=[" << goal.boundary_min_z << ", ";
    std::cout << goal.boundary_max_z << "]\n";
    std::cout << "  boundary_points: " << goal.boundary_x.size() << "\n";
  }

  void print_result(const GoalHandleNbvPlanner::WrappedResult & wrapped_result) const
  {
    const auto & result = wrapped_result.result;
    std::cout << "Result:\n";
    std::cout << "  complete_exploration: " << std::boolalpha << result->complete_exploration << "\n";
    std::cout << "  request_base_pose: " << std::boolalpha << result->request_base_pose << "\n";
    std::cout << "  explored_boundary_id: " << result->explored_boundary_id << "\n";
    std::cout << "  goals: " << result->goals.size() << "\n";

    if (!result->goals.empty()) {
      const auto & goal = result->goals.front();
      const auto & pose = goal.pose;
      std::cout << "Best candidate pose:\n";
      std::cout << "  frame_id: " << goal.header.frame_id << "\n";
      std::cout << "  position: x=" << pose.position.x;
      std::cout << ", y=" << pose.position.y;
      std::cout << ", z=" << pose.position.z << "\n";
      std::cout << "  orientation: x=" << pose.orientation.x;
      std::cout << ", y=" << pose.orientation.y;
      std::cout << ", z=" << pose.orientation.z;
      std::cout << ", w=" << pose.orientation.w << "\n";
    }
  }

  std::string action_name_;
  std::string frame_id_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  double measurement_grid_size_;
  double wait_timeout_s_;
  rclcpp_action::Client<NbvPlanner>::SharedPtr client_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<FkieNbvGoalClient>();
  const int exit_code = node->run();
  rclcpp::shutdown();
  return exit_code;
}
