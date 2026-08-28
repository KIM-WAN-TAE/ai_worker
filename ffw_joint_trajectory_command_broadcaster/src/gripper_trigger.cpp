// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "joint_trajectory_command_broadcaster/gripper_edge_gate.hpp"

namespace ffw_gripper_trigger
{

class GripperTrigger : public rclcpp::Node
{
public:
  GripperTrigger()
  : rclcpp::Node("gripper_trigger")
  {
    gripper_threshold_ = declare_parameter<double>("gripper_threshold", -0.7);
    save_pose_id_ = static_cast<uint8_t>(
      declare_parameter<int>("save_pose_id", 3));

    leader_joint_states_topic_ = declare_parameter<std::string>(
      "leader_joint_states_topic", "/leader/joint_states");
    left_gripper_joint_name_ = declare_parameter<std::string>(
      "left_gripper_joint_name", "gripper_l_joint1");
    right_gripper_joint_name_ = declare_parameter<std::string>(
      "right_gripper_joint_name", "gripper_r_joint1");

    left_enable_topic_ = declare_parameter<std::string>(
      "left_enable_topic", "/leader/left_command");
    right_enable_topic_ = declare_parameter<std::string>(
      "right_enable_topic", "/leader/right_command");

    auto qos = rclcpp::SystemDefaultsQoS();
    left_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(left_enable_topic_, qos);
    right_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(right_enable_topic_, qos);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        check_joint_edge(
          *msg, left_gripper_joint_name_, left_edge_gate_, left_enable_pub_, "left");
        check_joint_edge(
          *msg, right_gripper_joint_name_, right_edge_gate_, right_enable_pub_, "right");
      });

    RCLCPP_INFO(get_logger(),
      "gripper_trigger started (topic=%s, threshold=%.2f, save_pose_id=%u)",
      leader_joint_states_topic_.c_str(), gripper_threshold_, save_pose_id_);
  }

private:
  // Edge-triggered: publish save_pose_id when gripper crosses threshold downward.
  // Re-arms when gripper goes back above threshold.
  void check_joint_edge(
    const sensor_msgs::msg::JointState & msg,
    const std::string & joint_name,
    GripperEdgeGate & edge_gate,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const char * tag)
  {
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      if (msg.name[i] == joint_name) {
        check_edge(msg.position[i], edge_gate, enable_pub, tag);
        return;
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[%s] gripper joint '%s' not found in %s",
      tag, joint_name.c_str(), leader_joint_states_topic_.c_str());
  }

  void check_edge(
    double gripper_pos,
    GripperEdgeGate & edge_gate,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const char * tag)
  {
    if (!std::isfinite(gripper_pos)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[%s] ignoring non-finite gripper position", tag);
      return;
    }

    const bool was_initialized = edge_gate.initialized();
    const bool falling_edge = edge_gate.observe(gripper_pos, gripper_threshold_);
    if (!was_initialized) {
      RCLCPP_INFO(
        get_logger(),
        "[%s] gripper trigger baseline initialized at %.2f; no command published",
        tag, gripper_pos);
    }

    if (falling_edge) {
      // Downward edge: publish once
      std_msgs::msg::UInt8 msg;
      msg.data = save_pose_id_;
      enable_pub->publish(msg);
      RCLCPP_WARN(get_logger(),
        "[%s] Gripper crossed threshold (%.2f < %.2f), published enable=%u",
        tag, gripper_pos, gripper_threshold_, save_pose_id_);
    }
  }

  // Params
  double gripper_threshold_;
  uint8_t save_pose_id_;
  std::string leader_joint_states_topic_;
  std::string left_gripper_joint_name_;
  std::string right_gripper_joint_name_;
  std::string left_enable_topic_;
  std::string right_enable_topic_;

  // Edge state per side
  GripperEdgeGate left_edge_gate_;
  GripperEdgeGate right_edge_gate_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_enable_pub_;
};

}  // namespace ffw_gripper_trigger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_gripper_trigger::GripperTrigger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
