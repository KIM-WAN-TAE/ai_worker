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
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
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
    enable_press_gestures_ = declare_parameter<bool>("enable_press_gestures", false);
    gripper_release_threshold_ = declare_parameter<double>(
      "gripper_release_threshold", -0.3);
    long_press_duration_ = declare_parameter<double>("long_press_duration", 2.0);
    const auto short_press_command = declare_parameter<int>("short_press_command", 2);
    if (short_press_command < 0 || short_press_command > 255) {
      throw std::invalid_argument("short_press_command must be between 0 and 255");
    }
    short_press_command_ = static_cast<uint8_t>(short_press_command);
    left_long_press_trigger_ = declare_parameter<std::string>(
      "left_long_press_trigger", "left");
    right_long_press_trigger_ = declare_parameter<std::string>(
      "right_long_press_trigger", "right");
    tact_trigger_topic_ = declare_parameter<std::string>(
      "tact_trigger_topic", "/leader/joystick_controller/tact_trigger");

    if (enable_press_gestures_ && gripper_release_threshold_ <= gripper_threshold_) {
      throw std::invalid_argument(
              "gripper_release_threshold must be greater than gripper_threshold");
    }
    if (enable_press_gestures_ && long_press_duration_ <= 0.0) {
      throw std::invalid_argument("long_press_duration must be positive");
    }

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
    tact_trigger_pub_ = create_publisher<std_msgs::msg::String>(tact_trigger_topic_, qos);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        check_joint(
          *msg, left_gripper_joint_name_, left_edge_gate_, left_gesture_gate_,
          left_enable_pub_, left_long_press_trigger_, "left");
        check_joint(
          *msg, right_gripper_joint_name_, right_edge_gate_, right_gesture_gate_,
          right_enable_pub_, right_long_press_trigger_, "right");
      });

    if (enable_press_gestures_) {
      RCLCPP_INFO(
        get_logger(),
        "gripper_trigger gesture mode started (topic=%s, press<%.2f, release>%.2f, "
        "short_command=%u, long_press=%.2fs)",
        leader_joint_states_topic_.c_str(), gripper_threshold_, gripper_release_threshold_,
        short_press_command_, long_press_duration_);
    } else {
      RCLCPP_INFO(get_logger(),
        "gripper_trigger legacy mode started (topic=%s, threshold=%.2f, save_pose_id=%u)",
        leader_joint_states_topic_.c_str(), gripper_threshold_, save_pose_id_);
    }
  }

private:
  void check_joint(
    const sensor_msgs::msg::JointState & msg,
    const std::string & joint_name,
    GripperEdgeGate & edge_gate,
    GripperPressGestureGate & gesture_gate,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const std::string & long_press_trigger,
    const char * tag)
  {
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      if (msg.name[i] == joint_name) {
        if (enable_press_gestures_) {
          check_gesture(
            msg.position[i], gesture_gate, enable_pub, long_press_trigger, tag);
        } else {
          check_edge(msg.position[i], edge_gate, enable_pub, tag);
        }
        return;
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[%s] gripper joint '%s' not found in %s",
      tag, joint_name.c_str(), leader_joint_states_topic_.c_str());
  }

  void check_gesture(
    double gripper_pos,
    GripperPressGestureGate & gesture_gate,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const std::string & long_press_trigger,
    const char * tag)
  {
    if (!std::isfinite(gripper_pos)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[%s] ignoring non-finite gripper position", tag);
      return;
    }

    const bool was_initialized = gesture_gate.initialized();
    const auto action = gesture_gate.observe(
      gripper_pos, gripper_threshold_, gripper_release_threshold_,
      now().seconds(), long_press_duration_);
    if (!was_initialized) {
      RCLCPP_INFO(
        get_logger(),
        "[%s] gripper gesture baseline initialized at %.2f; no command published",
        tag, gripper_pos);
    }

    if (action == GripperGestureAction::SHORT_PRESS) {
      std_msgs::msg::UInt8 msg;
      msg.data = short_press_command_;
      enable_pub->publish(msg);
      RCLCPP_INFO(
        get_logger(), "[%s] Short gripper press released; published command=%u",
        tag, short_press_command_);
    } else if (action == GripperGestureAction::LONG_PRESS) {
      if (long_press_trigger.empty()) {
        RCLCPP_WARN(get_logger(), "[%s] Long gripper press has no configured trigger", tag);
        return;
      }
      std_msgs::msg::String msg;
      msg.data = long_press_trigger;
      tact_trigger_pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(), "[%s] Long gripper press; published recording trigger='%s'",
        tag, long_press_trigger.c_str());
    }
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
  double gripper_release_threshold_;
  double long_press_duration_;
  uint8_t save_pose_id_;
  uint8_t short_press_command_;
  bool enable_press_gestures_;
  std::string leader_joint_states_topic_;
  std::string left_gripper_joint_name_;
  std::string right_gripper_joint_name_;
  std::string left_enable_topic_;
  std::string right_enable_topic_;
  std::string tact_trigger_topic_;
  std::string left_long_press_trigger_;
  std::string right_long_press_trigger_;

  // Edge state per side
  GripperEdgeGate left_edge_gate_;
  GripperEdgeGate right_edge_gate_;
  GripperPressGestureGate left_gesture_gate_;
  GripperPressGestureGate right_gesture_gate_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tact_trigger_pub_;
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
