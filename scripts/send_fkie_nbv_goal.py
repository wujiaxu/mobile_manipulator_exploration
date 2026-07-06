#!/usr/bin/env python3
import os
import sys

import rclpy
from mobile_manipulator_fkie_msgs.action import NbvPlanner
from rclpy.action import ActionClient


def env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def make_goal() -> NbvPlanner.Goal:
    frame_id = os.environ.get("FKIE_WORLD_FRAME", "map")
    min_x = env_float("FKIE_BOUNDARY_MIN_X", -3.0)
    max_x = env_float("FKIE_BOUNDARY_MAX_X", 3.0)
    min_y = env_float("FKIE_BOUNDARY_MIN_Y", -3.0)
    max_y = env_float("FKIE_BOUNDARY_MAX_Y", 3.0)
    min_z = env_float("FKIE_BOUNDARY_MIN_Z", 0.4)
    max_z = env_float("FKIE_BOUNDARY_MAX_Z", 1.4)

    goal = NbvPlanner.Goal()
    goal.header.frame_id = frame_id
    goal.boundary_id = 1
    goal.boundary_source_type = "fixed_test_boundary"
    goal.boundary_frame_id = frame_id
    goal.boundary_min_z = min_z
    goal.boundary_max_z = max_z
    goal.boundary_x = [min_x, max_x, max_x, min_x]
    goal.boundary_y = [min_y, min_y, max_y, max_y]
    goal.estimations.header.frame_id = frame_id
    goal.estimations.grid_size = env_float("FKIE_MEASUREMENT_GRID_SIZE", 0.5)
    goal.estimations.values = []
    goal.arm_in_prepacked = False
    return goal


def print_goal(goal: NbvPlanner.Goal) -> None:
    print("Sending FKIE NBV goal:")
    print(f"  frame_id: {goal.header.frame_id}")
    print(
        "  boundary: "
        f"x=[{goal.boundary_x[0]:.3f}, {goal.boundary_x[1]:.3f}], "
        f"y=[{goal.boundary_y[0]:.3f}, {goal.boundary_y[2]:.3f}], "
        f"z=[{goal.boundary_min_z:.3f}, "
        f"{goal.boundary_max_z:.3f}]"
    )
    print(f"  boundary_points: {len(goal.boundary_x)}")


def main() -> int:
    action_name = os.environ.get("FKIE_NBV_ACTION_NAME", "/nbv_rrt")

    rclpy.init()
    node = rclpy.create_node("send_fkie_nbv_goal")
    client = ActionClient(node, NbvPlanner, action_name)

    print(f"Waiting for action server {action_name}...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"Action server not available: {action_name}")
        rclpy.shutdown()
        return 1

    goal = make_goal()
    print_goal(goal)

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("Goal rejected")
        rclpy.shutdown()
        return 2

    print(f"Goal accepted: {goal_handle.goal_id.uuid}")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    wrapped_result = result_future.result()
    result = wrapped_result.result

    print("Result:")
    print(f"  complete_exploration: {result.complete_exploration}")
    print(f"  request_base_pose: {result.request_base_pose}")
    print(f"  explored_boundary_id: {result.explored_boundary_id}")
    print(f"  goals: {len(result.goals)}")
    if result.goals:
        pose = result.goals[0].pose
        print("Best candidate pose:")
        print(f"  frame_id: {result.goals[0].header.frame_id}")
        print(
            "  position: "
            f"x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}"
        )
        print(
            "  orientation: "
            f"x={pose.orientation.x:.6f}, "
            f"y={pose.orientation.y:.6f}, "
            f"z={pose.orientation.z:.6f}, "
            f"w={pose.orientation.w:.6f}"
        )

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
