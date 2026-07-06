#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
MSG_PACKAGE = ROOT / "mobile_manipulator_fkie_msgs"
NBV_PACKAGE = ROOT / "mobile_manipulator_fkie_nbv"
CHECK_SCRIPT = ROOT / "scripts" / "check_fkie_inputs.sh"
SEND_GOAL_SCRIPT = ROOT / "scripts" / "send_fkie_nbv_goal.sh"
SEND_GOAL_PY = ROOT / "scripts" / "send_fkie_nbv_goal.py"
EXECUTE_NBV_SCRIPT = ROOT / "scripts" / "execute_fkie_nbv_arm_target.sh"
RUNTIME_DOC = ROOT / "docs" / "references" / "fkie_reproduction_runtime.md"


class FkieMsgsContractTest(unittest.TestCase):
    def test_minimal_fkie_messages_and_action_exist(self):
        expected_files = (
            "msg/BoundaryPolygon.msg",
            "msg/MeasurementEstimation.msg",
            "msg/MeasurementEstimationValue.msg",
            "action/NbvPlanner.action",
            "CMakeLists.txt",
            "package.xml",
        )
        for relative in expected_files:
            self.assertTrue((MSG_PACKAGE / relative).is_file(), relative)

    def test_message_definitions_match_fkie_contract(self):
        boundary = (MSG_PACKAGE / "msg/BoundaryPolygon.msg").read_text(encoding="utf-8")
        for literal in (
            "int16 id",
            "string source_type",
            "float32 min_height",
            "float32 max_height",
            "geometry_msgs/PolygonStamped polygon",
        ):
            self.assertIn(literal, boundary)

        estimation = (MSG_PACKAGE / "msg/MeasurementEstimation.msg").read_text(
            encoding="utf-8"
        )
        for literal in (
            "std_msgs/Header header",
            "float32 grid_size",
            "mobile_manipulator_fkie_msgs/MeasurementEstimationValue[] values",
        ):
            self.assertIn(literal, estimation)

        value = (MSG_PACKAGE / "msg/MeasurementEstimationValue.msg").read_text(
            encoding="utf-8"
        )
        for literal in (
            "float32 x",
            "float32 y",
            "float32 z",
            "float32 mean",
            "float32 variance",
        ):
            self.assertIn(literal, value)

    def test_action_definition_matches_fkie_contract_with_local_message_namespace(self):
        action = (MSG_PACKAGE / "action/NbvPlanner.action").read_text(encoding="utf-8")
        for literal in (
            "std_msgs/Header header",
            "int16 boundary_id",
            "string boundary_source_type",
            "string boundary_frame_id",
            "float64 boundary_min_z",
            "float64 boundary_max_z",
            "float64[] boundary_x",
            "float64[] boundary_y",
            "mobile_manipulator_fkie_msgs/MeasurementEstimation estimations",
            "bool arm_in_prepacked",
            "geometry_msgs/PoseStamped[] goals",
            "bool request_base_pose",
            "geometry_msgs/PoseStamped goal_pose_3d",
            "bool complete_exploration",
            "int16 explored_boundary_id",
        ):
            self.assertIn(literal, action)
        self.assertEqual(action.count("---"), 2)
        self.assertTrue(action.rstrip().endswith("---"))

    def test_message_package_declares_rosidl_dependencies(self):
        package = ET.parse(MSG_PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in package
            if element.tag in {"depend", "buildtool_depend", "member_of_group"}
        }
        expected = {
            "ament_cmake",
            "geometry_msgs",
            "rosidl_default_generators",
            "rosidl_default_runtime",
            "std_msgs",
            "rosidl_interface_packages",
        }
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

        cmake = (MSG_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for literal in (
            "rosidl_generate_interfaces",
            "msg/BoundaryPolygon.msg",
            "msg/MeasurementEstimation.msg",
            "msg/MeasurementEstimationValue.msg",
            "action/NbvPlanner.action",
            "geometry_msgs",
            "std_msgs",
        ):
            self.assertIn(literal, cmake)


class FkieNbvSkeletonContractTest(unittest.TestCase):
    def test_skeleton_planner_package_files_exist(self):
        expected_files = (
            "CMakeLists.txt",
            "package.xml",
            "config/fkie_nbv_planner.yaml",
            "launch/fkie_nbv_planner.launch.py",
            "include/mobile_manipulator_fkie_nbv/nbv_parameters.hpp",
            "include/mobile_manipulator_fkie_nbv/nbv_grid.hpp",
            "include/mobile_manipulator_fkie_nbv/nbv_utils.hpp",
            "include/mobile_manipulator_fkie_nbv/rrt_node.hpp",
            "include/mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp",
            "src/send_fkie_nbv_goal_client.cpp",
            "src/nbv_arm_target_adapter.cpp",
            "src/fkie_nbv_planner_node.cpp",
            "src/rrt_node.cpp",
        )
        for relative in expected_files:
            self.assertTrue((NBV_PACKAGE / relative).is_file(), relative)

    def test_skeleton_config_uses_current_project_topics_and_frames(self):
        params = yaml.safe_load(
            (NBV_PACKAGE / "config/fkie_nbv_planner.yaml").read_text(encoding="utf-8")
        )["fkie_nbv_planner"]["ros__parameters"]
        self.assertEqual(params["action_name"], "nbv_rrt")
        self.assertEqual(params["world_frame"], "map")
        self.assertEqual(params["camera_pose_topic"], "/camera_pose")
        self.assertEqual(params["octomap_topic"], "/octomap_full")
        self.assertEqual(
            params["robot_footprint_topic"],
            "/mobile_manipulator_mbf/global_costmap/footprint",
        )
        for key in (
            "num_frontiers",
            "frontiers_N_max_tries",
            "max_num_cached_points_to_keep",
            "min_gain_frontier",
            "free_space_frontier",
            "arm_height_min",
            "arm_height_max",
            "self_collision_height",
            "arm_goal_range",
            "raycast_dr",
            "raycast_dphi",
            "raycast_dtheta",
            "gain_r_min",
            "gain_r_max",
            "camera_hfov",
            "camera_vfov",
            "visited_cells_grid_size",
            "compute_yaw_from_free_space",
            "compute_yaw_from_measurement",
            "rrt_tree_length",
            "rrt_N_max_tries",
            "rrt_sampling_radius",
            "rrt_collision_cyl_radius",
            "rrt_sample_threshold_distance",
            "rrt_step_size",
            "sample_in_unknown",
            "empty_octomap_resolution",
            "do_rrt_star",
            "utility_weight_measurement",
            "utility_weight_free_space",
            "utility_weight_visited_cell",
            "utility_weight_euclidean_cost",
            "utility_max_value_free_space",
            "utility_max_value_measurement",
            "utility_max_value_visited_cell",
            "robot_sampling_frame",
        ):
            self.assertIn(key, params)
        self.assertTrue(
            params["sample_in_unknown"],
            "Isaac wrist OctoMap does not explicitly store enough known-free cells for RRT growth",
        )
        self.assertLess(
            params["arm_height_min"],
            0.4,
            "FKIE camera pose sampling should include low viewpoints below 0.4 m",
        )
        self.assertGreater(
            params["utility_weight_euclidean_cost"],
            0.0,
            "FKIE tuning should penalize long RRT branches during live Isaac trials",
        )

    def test_planner_node_uses_original_rrt_algorithm_shape(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "rclcpp_action::Server",
            "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp",
            "mobile_manipulator_fkie_nbv/nbv_parameters.hpp",
            "mobile_manipulator_fkie_nbv/rrt_node.hpp",
            "mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp",
            "octomap_msgs/conversions.h",
            "octomap::OcTree",
            "octomap_msgs/msg/octomap.hpp",
            "geometry_msgs/msg/pose_stamped.hpp",
            "geometry_msgs/msg/polygon_stamped.hpp",
            "camera_pose_topic_",
            "octomap_topic_",
            "robot_footprint_topic_",
            "handle_accepted",
            "complete_exploration",
            "request_base_pose",
            "reset_tree",
            "initialize_root",
            "expand_rrt",
            "find_closest_neighbor",
            "find_new_rrt_node",
            "collision_line",
            "gain_cubature",
            "update_gain",
            "compute_yaw",
            "get_best_node_branch",
            "extract_nbv_poses",
            "cached_nodes_",
            "best_node_",
            "tree_adapter_",
            "kdtree_",
            "point_in_polygon",
            "is_occupied",
        ):
            self.assertIn(literal, source)
        self.assertNotIn("make_placeholder_goal", source)
        self.assertNotIn("compute_best_view_candidate", source)
        self.assertNotIn("score_candidate", source)

    def test_planner_publishes_rtt_visualization_markers(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        cmake = (NBV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (NBV_PACKAGE / "package.xml").read_text(encoding="utf-8")
        for literal in (
            "visualization_msgs/msg/marker_array.hpp",
            "rrt_marker_topic_",
            "rrt_marker_publisher_",
            "publish_rrt_markers",
            "visualization_msgs::msg::MarkerArray",
            "SPHERE_LIST",
            "LINE_LIST",
            "LINE_STRIP",
            "TEXT_VIEW_FACING",
            "CUBE_LIST",
            "fkie_rrt_best_branch_view_dirs",
            "fkie_exploration_boundary",
            "fkie_exploration_boundary_label",
            "fkie_known_free_voxels",
            "fkie_known_occupied_voxels",
            "fkie_unknown_voxels",
            "fkie_rrt_nodes",
            "fkie_rrt_edges",
            "fkie_rrt_best_branch",
            "camera_direction_length",
            "tf2::quatRotate",
            "tf2::Vector3(0.0, 0.0, 1.0)",
            "FKIE target area",
            "populate_occupancy_markers",
            "publish_live_map_markers",
            "last_live_map_marker_publish_time_",
            "last_frontiers_",
            "publish_rrt_markers(last_frontiers_)",
            "kOccupancyMarkerResolution",
            "max_known_free_marker_voxels_",
            "max_known_occupied_marker_voxels_",
            "max_unknown_marker_voxels_",
            'declare_parameter<int>("max_known_free_marker_voxels", 5000)',
            'declare_parameter<int>("max_known_occupied_marker_voxels", 50000)',
            'declare_parameter<int>("max_unknown_marker_voxels", 10000)',
            "FKIE volume markers:",
            "current_boundary_min_z_",
            "current_boundary_max_z_",
            "normalize_boundary_z_limits",
            "Received reversed FKIE boundary z limits",
            "max_branch_nodes",
            "parameters_.max_branch_nodes",
            "optical_z_yaw_to_quaternion",
            "ROS optical frames look along +Z",
            "/fkie_nbv/rrt_markers",
        ):
            self.assertIn(literal, source)
        self.assertIn("find_package(visualization_msgs REQUIRED)", cmake)
        self.assertIn("visualization_msgs", cmake)
        self.assertIn("<depend>visualization_msgs</depend>", package)

    def test_planner_node_ports_rrt_expansion_mechanics(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "mobile_manipulator_fkie_nbv/nanoflann.hpp",
            "KDTreeSingleIndexDynamicAdaptor",
            "nanoflann::L2_Simple_Adaptor",
            "KNNResultSet",
            "SearchParams",
            "addPoints",
            "generate_random_sample",
            "parameters_.rrt_N_max_tries",
            "parameters_.rrt_tree_length",
            "parameters_.sample_in_unknown",
            "parameters_.rrt_collision_cyl_radius",
            "parameters_.rrt_sample_threshold_distance",
            "is_sample_in_polygon(q_new, current_boundary_",
            "is_sample_in_polygon(q_new, current_robot_footprint_",
            "new_node->parent_wptr = nearest_node",
            "nearest_node->children.push_back(new_node)",
            "update_gain(new_node)",
            "compute_yaw(new_node)",
            "new_node->compute_score()",
            "best_node_ = new_node",
            "cached_nodes_.push_back(new_node->copy_to_rrt_node())",
        ):
            self.assertIn(literal, source)
        self.assertNotIn("full original RRT expansion is ported in the next task", source)

    def test_planner_node_ports_gain_yaw_and_frontier_scoring(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        grid = (
            NBV_PACKAGE / "include/mobile_manipulator_fkie_nbv/nbv_grid.hpp"
        ).read_text(encoding="utf-8")
        for literal in (
            "MeasurementValue",
            "VisitedValue",
            "PositionGrid",
            "IndexGrid",
            "SparseGrid",
            "position_to_index",
            "get_index_neighbors",
        ):
            self.assertIn(literal, grid)
        for literal in (
            "measurement_grid_.set_grid_size",
            "measurement_grid_.add_value",
            "visited_grid_.add_value",
            "measurement_grid_.get_value",
            "visited_grid_.get_value",
            "gain_per_yaw",
            "parameters_.camera_hfov",
            "parameters_.camera_vfov",
            "parameters_.gain_r_min",
            "parameters_.gain_r_max",
            "parameters_.raycast_dr",
            "parameters_.raycast_dphi",
            "parameters_.raycast_dtheta",
            "current_utility_max_value_free_space_",
            "current_utility_max_value_measurement_",
            "current_utility_max_value_visited_cell_",
            "node->set_orientation(optical_z_yaw_to_quaternion(yaw))",
            "parameters_.compute_yaw_from_free_space",
            "parameters_.compute_yaw_from_measurement",
            "measurement_grid_.get_index_neighbors",
            "update_gain_cached_nodes",
            "remove_low_gain_cache_nodes",
            "find_high_utility_frontier",
        ):
            self.assertIn(literal, source)
        self.assertNotIn("return {0.0, 0.0};", source)

    def test_planner_node_ports_branch_result_semantics(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        cmake = (NBV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (NBV_PACKAGE / "package.xml").read_text(encoding="utf-8")
        for literal in (
            "tf2_ros/buffer.h",
            "tf2_ros/transform_listener.h",
            "tf2_geometry_msgs/tf2_geometry_msgs.hpp",
            "parameters_.world_frame",
            "parameters_.arm_goal_range",
            "node->get_gain() <= min_gain_",
            "goals.push_back(goal_pose)",
            "result->goals = best_branch_",
            "result->request_base_pose = false",
            "result->complete_exploration = true",
            "result->goal_pose_3d = best_frontier_->to_pose_stamped",
            "result->request_base_pose = true",
        ):
            self.assertIn(literal, source)
        self.assertIn("tf2_ros::Buffer tf_buffer_", source)
        self.assertIn("tf2_ros::TransformListener tf_listener_", source)
        self.assertNotIn(
            "goals.push_back(tf_buffer_.transform(goal_pose, parameters_.robot_sampling_frame))",
            source,
        )
        self.assertNotIn("distance_to_robot > parameters_.arm_goal_range", source)
        self.assertNotIn("FKIE RRT shell result", source)
        self.assertIn("tf2_ros", cmake)
        self.assertIn("tf2_geometry_msgs", cmake)
        self.assertIn("<depend>tf2_ros</depend>", package)
        self.assertIn("<depend>tf2_geometry_msgs</depend>", package)

    def test_planner_uses_post_rrt_frontiers_before_declaring_complete(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "const std::vector<RRTNode> pre_rrt_frontiers = extract_cached_frontiers()",
            "best_branch_ = get_nbv_branch(pre_rrt_frontiers)",
            "const std::vector<RRTNode> post_rrt_frontiers = extract_cached_frontiers()",
            "} else if (best_frontier_) {",
            "} else if (post_rrt_frontiers.empty()) {",
            "branch_goals=%zu pre_frontiers=%zu post_frontiers=%zu cached_nodes=%zu",
        ):
            self.assertIn(literal, source)
        self.assertLess(
            source.index("const std::vector<RRTNode> post_rrt_frontiers = extract_cached_frontiers()"),
            source.index("} else if (best_frontier_) {"),
        )
        self.assertLess(
            source.index("} else if (best_frontier_) {"),
            source.index("} else if (post_rrt_frontiers.empty()) {"),
        )

    def test_planner_can_start_with_empty_octomap(self):
        source = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (NBV_PACKAGE / "config/fkie_nbv_planner.yaml").read_text(encoding="utf-8")
        )["fkie_nbv_planner"]["ros__parameters"]
        self.assertEqual(config["empty_octomap_resolution"], 0.05)
        for literal in (
            "empty_octomap_resolution",
            "std::make_unique<octomap::OcTree>(parameters_.empty_octomap_resolution)",
            "Starting FKIE RRT with an empty OctoMap",
        ):
            self.assertIn(literal, source)

    def test_rrt_node_ports_original_gain_and_score_model(self):
        header = (
            NBV_PACKAGE / "include/mobile_manipulator_fkie_nbv/rrt_node.hpp"
        ).read_text(encoding="utf-8")
        source = (NBV_PACKAGE / "src/rrt_node.cpp").read_text(encoding="utf-8")
        for literal in (
            "parent_wptr",
            "children",
            "cubature_best_yaw",
            "set_gain",
            "compute_score",
            "cost_till_root",
            "cost_to_parent",
            "to_pose_stamped",
        ):
            self.assertIn(literal, header)
        for literal in (
            "parameters_->utility_weight_measurement",
            "parameters_->utility_weight_free_space",
            "parameters_->utility_weight_visited_cell",
            "parameters_->utility_weight_euclidean_cost",
            "gain_ = gain_m + gain_fs + gain_v",
            "score_ = get_gain() - parameters_->utility_weight_euclidean_cost * get_cost()",
        ):
            self.assertIn(literal, source)

    def test_rrt_utilities_and_nanoflann_adapter_are_ported(self):
        utils = (
            NBV_PACKAGE / "include/mobile_manipulator_fkie_nbv/nbv_utils.hpp"
        ).read_text(encoding="utf-8")
        adapter = (
            NBV_PACKAGE
            / "include/mobile_manipulator_fkie_nbv/tree_nanoflann_adapter.hpp"
        ).read_text(encoding="utf-8")
        for literal in (
            "generate_random_sample",
            "is_sample_in_polygon",
            "cylinder_caps_first",
            "distance_pose_stamped",
            "pose_to_eigen_vector3d",
            "std::mt19937",
            "radius * std::cbrt(noise_distribution(random_engine))",
        ):
            self.assertIn(literal, utils)
        self.assertNotIn("std::pow(radius_distribution(random_engine), 1.0 / 3.0)", utils)
        for literal in (
            "TreeNanoflannAdapter",
            "std::vector<std::shared_ptr<RRTNode>> nodes",
            "kdtree_get_point_count",
            "kdtree_get_pt",
        ):
            self.assertIn(literal, adapter)

    def test_skeleton_launch_starts_planner_with_config(self):
        launch = (NBV_PACKAGE / "launch/fkie_nbv_planner.launch.py").read_text(
            encoding="utf-8"
        )
        for literal in (
            "fkie_nbv_planner_node",
            "fkie_nbv_planner.yaml",
            "use_sim_time",
            "rmw_implementation",
            "rmw_cyclonedds_cpp",
            "SetEnvironmentVariable",
            'DeclareLaunchArgument("max_known_free_marker_voxels"',
            'DeclareLaunchArgument("max_known_occupied_marker_voxels"',
            'DeclareLaunchArgument("max_unknown_marker_voxels"',
            '"max_known_occupied_marker_voxels": max_known_occupied_marker_voxels',
        ):
            self.assertIn(literal, launch)

    def test_cpp_goal_client_constructs_boundary_directly(self):
        source = (NBV_PACKAGE / "src/send_fkie_nbv_goal_client.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "rclcpp_action::Client",
            "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp",
            "boundary_x",
            "boundary_y",
            "min_x",
            "max_x",
            "min_y",
            "max_y",
            "Best candidate pose",
        ):
            self.assertIn(literal, source)

        cmake = (NBV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for literal in (
            "add_executable(send_fkie_nbv_goal_client",
            "send_fkie_nbv_goal_client",
            "RUNTIME DESTINATION lib/${PROJECT_NAME}",
        ):
            self.assertIn(literal, cmake)

    def test_nbv_arm_target_adapter_uses_existing_moveit_pose_topic(self):
        source = (NBV_PACKAGE / "src/nbv_arm_target_adapter.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "rclcpp_action::Client",
            "mobile_manipulator_fkie_msgs/action/nbv_planner.hpp",
            "/arm_target_pose",
            "wrist_camera_color_optical_frame",
            "link_eef",
            "tf_buffer_",
            "lookupTransform",
            "async_send_goal",
            "target_publisher_",
        ):
            self.assertIn(literal, source)

        cmake = (NBV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for literal in (
            "add_executable(nbv_arm_target_adapter",
            "tf2_ros",
            "tf2_geometry_msgs",
            "nbv_arm_target_adapter",
        ):
            self.assertIn(literal, cmake)

    def test_nbv_adapter_dispatches_arm_goals_or_nav2_fallback(self):
        source = (NBV_PACKAGE / "src/nbv_arm_target_adapter.cpp").read_text(
            encoding="utf-8"
        )
        cmake = (NBV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        package = (NBV_PACKAGE / "package.xml").read_text(encoding="utf-8")
        for literal in (
            "nav2_msgs/action/navigate_to_pose.hpp",
            "std_msgs/msg/string.hpp",
            "visualization_msgs/msg/marker_array.hpp",
            "mobile_manipulator_moveit_bridge/action/move_arm.hpp",
            "using NavigateToPose = nav2_msgs::action::NavigateToPose",
            "using MoveArm = mobile_manipulator_moveit_bridge::action::MoveArm",
            "base_action_name_",
            "base_client_",
            "arm_action_name_",
            "arm_client_",
            "execution_marker_topic_",
            "execution_marker_publisher_",
            "named_target_topic_",
            "named_target_publisher_",
            "stow_arm_before_base_motion_",
            "auto_explore_",
            "max_exploration_iterations_",
            "map_update_wait_s_",
            "continue_on_motion_failure_",
            "continue_base_on_stow_failure_",
            "run_auto_exploration",
            "request_nbv_result",
            "dispatch_nbv_result",
            "FKIE auto exploration iteration %d/%d",
            "Waiting %.2f s for OctoMap update before next FKIE iteration",
            "retry_arm_after_base_fallback_",
            "base_goal_standoff_",
            "arm_named_target_retry_count_",
            "arm_named_target_retry_delay_s_",
            "stow_arm_for_base_motion",
            "publish_execution_markers",
            "dispatch_arm_pose_goal",
            "dispatch_arm_named_target",
            "dispatch_branch_goals",
            "is_arm_reachable",
            "nearest_branch_pose_for_base",
            "base_goal_from_camera_pose",
            "dispatch_base_goal",
            "result->request_base_pose",
            "result->goal_pose_3d",
            "async_send_goal(nav_goal",
            "async_get_result(goal_handle)",
            "tf2::quatRotate(orientation.normalized(), tf2::Vector3(0.0, 0.0, 1.0))",
            "base_goal_standoff_ * std::cos(yaw)",
            "Computed NBV base fallback from camera target",
            "NBV branch camera target %zu/%zu",
            "NBV branch EEF target %zu/%zu",
            "fkie_pending_targets",
            "fkie_reached_targets",
            "fkie_unreachable_targets",
            "fkie_execution_target_numbers",
            "label_color",
            "state == 1 ? make_color(0.0F, 1.0F, 0.0F, 1.0F)",
            "state == 2 ? make_color(1.0F, 0.0F, 0.0F, 1.0F)",
            "nav_goal.pose.pose.position.z = 0.0",
            "tf2::getYaw",
            "target_publisher_->publish",
            'home_message.data = "home"',
            'arm_goal.use_named_target = true',
            'arm_goal.named_target = "home"',
            "Arm named target action attempt %d/%d failed",
            "Arm named target action failed after retry attempts",
            "Continuing with NBV base motion despite stow failure",
            "arm_goal.target_pose = *arm_target",
            "const bool reached_final_target = !result->goals.empty() && target_states.back() == 1",
            "if (!reached_final_target && !result->goals.empty())",
            "Branch target q_t+1 was not reached by the arm; moving base to its projection",
            "for (std::size_t target_index = 0; target_index < result->goals.size(); ++target_index)",
        ):
            self.assertIn(literal, source)
        self.assertIn('arm_goal_range_(declare_parameter<double>("arm_goal_range", 1.3))', source)
        self.assertIn(
            'retry_arm_after_base_fallback_(declare_parameter<bool>("retry_arm_after_base_fallback", true))',
            source,
        )
        self.assertIn('declare_parameter<bool>("continue_base_on_stow_failure", true)', source)
        self.assertIn(
            'base_goal_standoff_(declare_parameter<double>("base_goal_standoff", 0.45))',
            source,
        )
        self.assertIn(
            'arm_named_target_retry_count_(declare_parameter<int>("arm_named_target_retry_count", 1))',
            source,
        )
        self.assertIn("find_package(nav2_msgs REQUIRED)", cmake)
        self.assertIn("find_package(mobile_manipulator_moveit_bridge REQUIRED)", cmake)
        self.assertIn("nav2_msgs", cmake)
        self.assertIn("mobile_manipulator_moveit_bridge", cmake)
        self.assertIn("<depend>nav2_msgs</depend>", package)
        self.assertIn("<depend>mobile_manipulator_moveit_bridge</depend>", package)

    def test_nbv_execution_targets_use_latest_tf_time(self):
        planner = (NBV_PACKAGE / "src/fkie_nbv_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        adapter = (NBV_PACKAGE / "src/nbv_arm_target_adapter.cpp").read_text(
            encoding="utf-8"
        )
        for literal in (
            "node->to_pose_stamped(now())",
            "best_frontier_->to_pose_stamped(now())",
        ):
            self.assertIn(literal, planner)
        for literal in (
            "camera_goal.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type())",
            "nav_goal.pose.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type())",
            "eef_target.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type())",
        ):
            self.assertIn(literal, adapter)
        self.assertNotIn(
            "node->to_pose_stamped(rclcpp::Time(latest_camera_pose_->header.stamp))",
            planner,
        )

    def test_fkie_adapter_script_prefers_arm_before_base_repositioning(self):
        script = (ROOT / "scripts/execute_fkie_nbv_arm_target.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('FKIE_ARM_GOAL_RANGE:-1.3', script)


class FkieRuntimeHarnessContractTest(unittest.TestCase):
    def test_runtime_input_check_harness_exists(self):
        self.assertTrue(CHECK_SCRIPT.is_file())
        self.assertTrue(RUNTIME_DOC.is_file())

        script = CHECK_SCRIPT.read_text(encoding="utf-8")
        for literal in (
            "/octomap_full",
            "/realsense/depth/points2",
            "/camera_pose",
            "/mobile_manipulator_mbf/global_costmap/footprint",
            "/octomap_occupied_points",
            "/apply_planning_scene",
        ):
            self.assertIn(literal, script)

    def test_fixed_boundary_goal_script_exists(self):
        self.assertTrue(SEND_GOAL_SCRIPT.is_file())
        self.assertTrue(SEND_GOAL_PY.is_file())
        self.assertTrue(EXECUTE_NBV_SCRIPT.is_file())
        script = SEND_GOAL_SCRIPT.read_text(encoding="utf-8")
        for literal in (
            "/nbv_rrt",
            "FKIE_BOUNDARY_MIN_X",
            "FKIE_BOUNDARY_MAX_X",
            "RMW_IMPLEMENTATION",
            "rmw_cyclonedds_cpp",
            "ros2 run mobile_manipulator_fkie_nbv send_fkie_nbv_goal_client",
            "min_x",
            "max_x",
        ):
            self.assertIn(literal, script)
        client = SEND_GOAL_PY.read_text(encoding="utf-8")
        for literal in (
            "ActionClient",
            "NbvPlanner",
            "boundary_x",
            "boundary_y",
            "estimations",
            "Best candidate pose",
        ):
            self.assertIn(literal, client)

    def test_runtime_doc_lists_required_fkie_inputs(self):
        doc = RUNTIME_DOC.read_text(encoding="utf-8")
        for literal in (
            "scripts/check_fkie_inputs.sh",
            "scripts/send_fkie_nbv_goal.sh",
            "scripts/execute_fkie_nbv_arm_target.sh",
            "/octomap_full",
            "/realsense/depth/points2",
            "/camera_pose",
            "/mobile_manipulator_mbf/global_costmap/footprint",
            "/apply_planning_scene",
        ):
            self.assertIn(literal, doc)


if __name__ == "__main__":
    unittest.main()
