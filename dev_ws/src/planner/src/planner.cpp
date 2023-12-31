/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum) {
  fprintf(stderr,
          "[test_xarm_planner_api_pose] Ctrl-C caught, exit process...\n");
  exit(-1);
}

int main(int argc, char **argv) {
  printf("Starting Planner Node...");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("test_xarm_planner_api_pose", node_options);
  RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_pose start");

  const auto logger = node->get_logger();

  signal(SIGINT, exit_sig_handler);

  int dof;
  node->get_parameter_or("dof", dof, 7);
  std::string robot_type;
  node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
  std::string group_name = robot_type;
  if (robot_type != "uf850")
    group_name = robot_type + std::to_string(dof);

  RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s",
              node->get_namespace(), group_name.c_str());

  xarm_planner::XArmPlanner planner(node, group_name);

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      planner.move_group->getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  std::vector<geometry_msgs::msg::Pose> poses;

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.2;
  target_pose1.orientation.x = 1;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 0;
  poses.push_back(target_pose1);

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = 0.3;
  target_pose2.position.y = 0.1;
  target_pose2.position.z = 0.2;
  target_pose2.orientation.x = 1;
  target_pose2.orientation.y = 0;
  target_pose2.orientation.z = 0;
  target_pose2.orientation.w = 0;
  poses.push_back(target_pose2);

  geometry_msgs::msg::Pose target_pose3;
  target_pose3.position.x = 0.3;
  target_pose3.position.y = 0.1;
  target_pose3.position.z = 0.4;
  target_pose3.orientation.x = 1;
  target_pose3.orientation.y = 0;
  target_pose3.orientation.z = 0;
  target_pose3.orientation.w = 0;
  poses.push_back(target_pose3);

  geometry_msgs::msg::Pose target_pose4;
  target_pose4.position.x = 0.3;
  target_pose4.position.y = -0.1;
  target_pose4.position.z = 0.4;
  target_pose4.orientation.x = 1;
  target_pose4.orientation.y = 0;
  target_pose4.orientation.z = 0;
  target_pose4.orientation.w = 0;
  poses.push_back(target_pose4);

  while (rclcpp::ok()) {

    for (const auto &pose : poses) {

      // Create a plan to that target pose
      prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
      moveit_visual_tools.trigger();
      auto const success = planner.planPoseTarget(pose);

      // Execute the plan
      if (success) {
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        moveit_visual_tools.trigger();
        planner.executePath();
      } else {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_pose over");
  rclcpp::shutdown(); // <--- This will cause the spin function in the thread to
                      // return
  spinner.join();     // <--- Join the thread before exiting
  return 0;
}
