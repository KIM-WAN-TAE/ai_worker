// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "urdf/model.h"

#include "waypoint_telep/path_math.hpp"
#include "waypoint_telep/joint_clock.hpp"
#include "waypoint_telep/equal_time.hpp"
#include "waypoint_telep/path_refinement.hpp"
#include "waypoint_telep/limit_ik.hpp"

namespace waypoint_telep
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using namespace std::chrono_literals;

class WaypointTelepNode : public rclcpp::Node
{
public:
  enum class PathMode
  {
    LINEAR,
    JOINT
  };

  WaypointTelepNode()
  : Node("waypoint_telep")
  {
    base_link_ = declare_parameter<std::string>("base_link", "arm_base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    tip_link_ = declare_parameter<std::string>("tip_link", "end_effector_l_link");
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper_l_joint1");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    clicked_point_topic_ = declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    action_topic_ = declare_parameter<std::string>(
      "action_topic", "/arm_l_controller/follow_joint_trajectory");
    external_command_topic_ = declare_parameter<std::string>(
      "external_command_topic",
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
    publish_enabled_ = declare_parameter<bool>("publish_enabled", false);
    direct_action_enabled_ = declare_parameter<bool>("direct_action_enabled", false);
    joint_smoothing_enabled_ = declare_parameter<bool>("joint_smoothing_enabled", true);
    const std::string configured_path_mode = declare_parameter<std::string>(
      "path_mode", "linear");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
    display_rate_hz_ = declare_parameter<double>("display_rate_hz", 20.0);
    linear_speed_mps_ = declare_parameter<double>("linear_speed_mps", 0.02);
    angular_speed_radps_ = declare_parameter<double>("angular_speed_radps", 0.20);
    joint_speed_radps_ = declare_parameter<double>("joint_speed_radps", 0.20);
    gripper_speed_radps_ = declare_parameter<double>("gripper_speed_radps", 0.20);
    slow_start_speed_scale_ = declare_parameter<double>("slow_start_speed_scale", 1.0);
    minimum_segment_duration_s_ = declare_parameter<double>(
      "minimum_segment_duration_s", 0.50);
    max_waypoint_dwell_s_ = declare_parameter<double>("max_waypoint_dwell_s", 30.0);
    max_waypoint_speed_scale_ = declare_parameter<double>(
      "max_waypoint_speed_scale", 3.0);
    max_uniform_tcp_speed_mps_ = declare_parameter<double>(
      "max_uniform_tcp_speed_mps", 1.8);
    max_joint_step_rad_ = declare_parameter<double>("max_joint_step_rad", 0.05);
    max_cartesian_jog_step_m_ = declare_parameter<double>(
      "max_cartesian_jog_step_m", 0.0025);
    max_tool_path_correction_m_ = declare_parameter<double>(
      "max_tool_path_correction_m", 0.05);
    max_start_velocity_radps_ = declare_parameter<double>(
      "max_start_velocity_radps", 0.02);
    command_quiet_s_ = declare_parameter<double>("command_quiet_s", 0.50);
    ik_epsilon_ = declare_parameter<double>("ik_epsilon", 1.0e-5);
    ik_max_iterations_ = declare_parameter<int>("ik_max_iterations", 200);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, false);

    validate_parameters();
    path_mode_ = parse_path_mode(configured_path_mode);

    const auto robot_description_qos = rclcpp::QoS(1).reliable().transient_local();
    robot_description_subscription_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", robot_description_qos,
      std::bind(&WaypointTelepNode::robot_description_callback, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WaypointTelepNode::joint_state_callback, this, std::placeholders::_1));
    clicked_point_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic_, 10,
      std::bind(&WaypointTelepNode::clicked_point_callback, this, std::placeholders::_1));
    waypoint_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/waypoint_pose", 10,
      std::bind(&WaypointTelepNode::waypoint_pose_callback, this, std::placeholders::_1));
    absolute_position_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "~/absolute_position", 10,
      std::bind(&WaypointTelepNode::absolute_position_callback, this, std::placeholders::_1));
    absolute_orientation_subscription_ =
      create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "~/absolute_orientation", 10,
      std::bind(
        &WaypointTelepNode::absolute_orientation_callback, this, std::placeholders::_1));
    relative_waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/relative_waypoint", 10,
      std::bind(&WaypointTelepNode::relative_waypoint_callback, this, std::placeholders::_1));
    frame_relative_waypoint_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/frame_relative_waypoint", 10,
      std::bind(
        &WaypointTelepNode::frame_relative_waypoint_callback, this,
        std::placeholders::_1));
    relative_reference_subscription_ = create_subscription<std_msgs::msg::Int32>(
      "~/relative_reference", 10,
      std::bind(&WaypointTelepNode::relative_reference_callback, this, std::placeholders::_1));
    cartesian_jog_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "~/cartesian_jog", 10,
      std::bind(&WaypointTelepNode::cartesian_jog_callback, this, std::placeholders::_1));
    joint_waypoint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_waypoint", 10,
      std::bind(&WaypointTelepNode::joint_waypoint_callback, this, std::placeholders::_1));
    tool_path_correction_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/tool_path_correction", 10,
      std::bind(
        &WaypointTelepNode::tool_path_correction_callback, this,
        std::placeholders::_1));
    path_mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "~/path_mode", 10,
      std::bind(&WaypointTelepNode::path_mode_callback, this, std::placeholders::_1));
    waypoint_dwell_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_dwell", 10,
      std::bind(&WaypointTelepNode::waypoint_dwell_callback, this, std::placeholders::_1));
    waypoint_speed_scale_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_speed_scale", 10,
      std::bind(
        &WaypointTelepNode::waypoint_speed_scale_callback, this, std::placeholders::_1));
    equal_segment_time_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/equal_segment_time", 10,
      std::bind(&WaypointTelepNode::equal_segment_time_callback, this, std::placeholders::_1));
    equal_total_time_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/equal_total_time", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        assign_segment_times(message, true);
      });
    uniform_total_time_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/uniform_total_time", 10,
      std::bind(&WaypointTelepNode::assign_uniform_total_time, this, std::placeholders::_1));
    waypoint_gripper_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_gripper", 10,
      std::bind(&WaypointTelepNode::waypoint_gripper_callback, this, std::placeholders::_1));
    waypoint_mode_subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "~/waypoint_mode", 10,
      std::bind(&WaypointTelepNode::waypoint_mode_callback, this, std::placeholders::_1));
    joint_smoothing_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "~/joint_smoothing", 10,
      std::bind(&WaypointTelepNode::joint_smoothing_callback, this, std::placeholders::_1));
    refinement_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/path_refinement", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        if (executing_) {publish_error("cannot change path refinement during execution"); return;}
        try {
          const auto settings = parse_path_refinement(message->data);
          refinement_ = settings;
          refinement_state_publisher_->publish(*message);
          invalidate_plan();
          refinement_diagnostic("idle", 0, settings.subdivisions, 0, 0, 0, 0, "settings updated");
          publish_status("PATH_REFINEMENT_UPDATED");
        } catch (const std::exception & e) {publish_error(e.what());}
      });
    joint_timing_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_timing", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (executing_) {publish_error("cannot change timing during execution"); return;}
        const auto & d = msg->data;
        if (d.size() != 3 || !std::all_of(d.begin(), d.end(), [](double x) {
            return std::isfinite(x);}) || d[0] < 0.0 || d[0] > max_uniform_tcp_speed_mps_ ||
          d[1] < 0 || d[2] < d[1] || std::floor(d[1]) != d[1] ||
          std::floor(d[2]) != d[2] ||
          (d[0] > 0 && d[1] < 2 && !(d[1] == 0 && d[2] == 0)))
        {publish_error("invalid joint timing [speed_mps, preserve_start, preserve_end]"); return;}
        joint_timing_ = d;
        joint_timing_publisher_->publish(*msg);
        invalidate_plan();
        publish_status("JOINT_TIMING_UPDATED");
      });
    start_waypoint_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      "~/start_waypoint", 10,
      std::bind(&WaypointTelepNode::start_waypoint_callback, this, std::placeholders::_1));
    end_waypoint_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      "~/end_waypoint", 10,
      std::bind(&WaypointTelepNode::end_waypoint_callback, this, std::placeholders::_1));
    reorder_waypoint_subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "~/reorder_waypoint", 10,
      std::bind(&WaypointTelepNode::reorder_waypoint_callback, this, std::placeholders::_1));
    remove_waypoint_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      "~/remove_waypoint", 10,
      std::bind(&WaypointTelepNode::remove_waypoint_callback, this, std::placeholders::_1));
    external_command_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      external_command_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WaypointTelepNode::external_command_callback, this, std::placeholders::_1));

    preview_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "~/preview_path", rclcpp::QoS(1).reliable().transient_local());
    waypoints_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "~/waypoints", rclcpp::QoS(1).reliable().transient_local());
    current_world_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/current_world_pose", rclcpp::QoS(1).reliable().transient_local());
    world_waypoints_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "~/world_waypoints", rclcpp::QoS(1).reliable().transient_local());
    world_root_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/world_root_pose", rclcpp::QoS(1).reliable().transient_local());
    waypoint_dwells_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_dwells", rclcpp::QoS(1).reliable().transient_local());
    waypoint_speed_scales_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_speed_scales", rclcpp::QoS(1).reliable().transient_local());
    waypoint_grippers_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_grippers", rclcpp::QoS(1).reliable().transient_local());
    waypoint_modes_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "~/waypoint_modes", rclcpp::QoS(1).reliable().transient_local());
    joint_smoothing_enabled_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "~/joint_smoothing_enabled", rclcpp::QoS(1).reliable().transient_local());
    waypoint_joint_values_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/waypoint_joint_values", rclcpp::QoS(1).reliable().transient_local());
    selected_start_waypoint_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "~/selected_start_waypoint", rclcpp::QoS(1).reliable().transient_local());
    selected_end_waypoint_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "~/selected_end_waypoint", rclcpp::QoS(1).reliable().transient_local());
    joint_timing_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/joint_timing_state", rclcpp::QoS(1).reliable().transient_local());
    refinement_state_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/path_refinement_state", rclcpp::QoS(1).reliable().transient_local());
    refinement_diagnostics_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/path_refinement_diagnostics", rclcpp::QoS(32).reliable().transient_local());
    std_msgs::msg::Float64MultiArray initial_refinement;
    initial_refinement.data = {0, 10, 1, 0.001, 0.017453292519943295, 0.01, 5, 1};
    refinement_state_publisher_->publish(initial_refinement);
    std_msgs::msg::Float64MultiArray initial_timing;
    initial_timing.data = joint_timing_;
    joint_timing_publisher_->publish(initial_timing);
    planned_waypoint_targets_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/planned_waypoint_targets", rclcpp::QoS(1).reliable().transient_local());
    planned_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", rclcpp::QoS(1).reliable().transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).reliable().transient_local());
    cartesian_jog_target_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/cartesian_jog_target", 10);

    capture_service_ = create_service<std_srvs::srv::Trigger>(
      "~/capture_current",
      std::bind(
        &WaypointTelepNode::capture_current_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    remove_last_service_ = create_service<std_srvs::srv::Trigger>(
      "~/remove_last",
      std::bind(
        &WaypointTelepNode::remove_last_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      "~/clear",
      std::bind(
        &WaypointTelepNode::clear_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    select_linear_service_ = create_service<std_srvs::srv::Trigger>(
      "~/select_linear",
      std::bind(
        &WaypointTelepNode::select_linear_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    select_joint_service_ = create_service<std_srvs::srv::Trigger>(
      "~/select_joint",
      std::bind(
        &WaypointTelepNode::select_joint_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    preview_service_ = create_service<std_srvs::srv::Trigger>(
      "~/preview",
      std::bind(
        &WaypointTelepNode::preview_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    preview_slow_start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/preview_slow_start",
      std::bind(
        &WaypointTelepNode::preview_slow_start_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute",
      std::bind(
        &WaypointTelepNode::execute_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(
        &WaypointTelepNode::stop_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_topic_);

    publish_status(
      publish_enabled_ ?
      (direct_action_enabled_ ?
      "READY_CONFIG: direct action execution enabled; waiting for robot model and joint state" :
      "READY_CONFIG: UI WAYPOINT command-source execution enabled") :
      "PREVIEW_ONLY: publish_enabled=false");
    publish_waypoints();
    publish_joint_smoothing();
  }

private:
  struct Plan
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    nav_msgs::msg::Path path;
    std_msgs::msg::Float64MultiArray waypoint_targets;
  };

  struct Waypoint
  {
    CartesianPose pose;
    std::optional<std::vector<double>> joint_positions;
    double gripper_position{0.0};
    double dwell_s{0.0};
    PathMode interpolation_mode{PathMode::LINEAR};
    double speed_scale{1.0};
  };

  void validate_parameters() const
  {
    if (base_link_.empty() || world_frame_.empty() || tip_link_.empty() || gripper_joint_.empty()) {
      throw std::invalid_argument(
              "base_link, world_frame, tip_link, and gripper_joint must not be empty");
    }
    if (
      control_rate_hz_ <= 0.0 || linear_speed_mps_ <= 0.0 ||
      !std::isfinite(display_rate_hz_) || display_rate_hz_ <= 0.0 ||
      angular_speed_radps_ <= 0.0 || joint_speed_radps_ <= 0.0 ||
      gripper_speed_radps_ <= 0.0 ||
      slow_start_speed_scale_ <= 0.0 || slow_start_speed_scale_ > 1.0 ||
      minimum_segment_duration_s_ <= 0.0 ||
      max_waypoint_dwell_s_ <= 0.0 ||
      !std::isfinite(max_waypoint_speed_scale_) || max_waypoint_speed_scale_ < 1.0 ||
      !std::isfinite(max_uniform_tcp_speed_mps_) || max_uniform_tcp_speed_mps_ <= 0.0 ||
      max_joint_step_rad_ <= 0.0 || max_cartesian_jog_step_m_ <= 0.0 ||
      !std::isfinite(max_tool_path_correction_m_) || max_tool_path_correction_m_ <= 0.0 ||
      max_start_velocity_radps_ < 0.0 ||
      command_quiet_s_ < 0.0 || ik_epsilon_ <= 0.0 || ik_max_iterations_ <= 0)
    {
      throw std::invalid_argument("waypoint_telep parameters contain an invalid limit");
    }
  }

  static PathMode parse_path_mode(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
    if (value == "linear") {
      return PathMode::LINEAR;
    }
    if (value == "joint") {
      return PathMode::JOINT;
    }
    throw std::invalid_argument("path_mode must be 'linear' or 'joint'");
  }

  static std::string path_mode_name(PathMode mode)
  {
    return mode == PathMode::LINEAR ? "LINEAR" : "JOINT";
  }

  std::string path_mode_name() const
  {
    return path_mode_name(path_mode_);
  }

  std::string plan_mode_name() const
  {
    if (waypoints_.empty()) {
      return path_mode_name();
    }
    const auto indices = selected_waypoint_indices();
    if (indices.empty()) {
      return path_mode_name();
    }
    const PathMode first_mode = waypoints_[indices.front()].interpolation_mode;
    const bool all_same = std::all_of(
      indices.begin(), indices.end(), [this, first_mode](std::size_t index) {
        return waypoints_[index].interpolation_mode == first_mode;
      });
    if (all_same && first_mode == PathMode::JOINT && joint_smoothing_enabled_) {
      return "JOINT_CONTINUOUS";
    }
    return all_same ? path_mode_name(first_mode) : "MIXED";
  }

  void publish_status(const std::string & message)
  {
    std_msgs::msg::String status;
    status.data = message;
    status_publisher_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  void publish_error(const std::string & message)
  {
    std_msgs::msg::String status;
    status.data = "ERROR: " + message;
    status_publisher_->publish(status);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  void publish_waypoints()
  {
    geometry_msgs::msg::PoseArray message;
    message.header.frame_id = base_link_;
    message.header.stamp = now();
    message.poses.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = waypoint.pose.position[0];
      pose.position.y = waypoint.pose.position[1];
      pose.position.z = waypoint.pose.position[2];
      pose.orientation.x = waypoint.pose.orientation.x;
      pose.orientation.y = waypoint.pose.orientation.y;
      pose.orientation.z = waypoint.pose.orientation.z;
      pose.orientation.w = waypoint.pose.orientation.w;
      message.poses.push_back(std::move(pose));
    }
    waypoints_publisher_->publish(message);

    std_msgs::msg::Float64MultiArray dwell_message;
    dwell_message.data.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      dwell_message.data.push_back(waypoint.dwell_s);
    }
    waypoint_dwells_publisher_->publish(dwell_message);

    std_msgs::msg::Float64MultiArray speed_scale_message;
    speed_scale_message.data.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      speed_scale_message.data.push_back(waypoint.speed_scale);
    }
    waypoint_speed_scales_publisher_->publish(speed_scale_message);

    std_msgs::msg::Float64MultiArray gripper_message;
    gripper_message.data.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      gripper_message.data.push_back(waypoint.gripper_position);
    }
    waypoint_grippers_publisher_->publish(gripper_message);

    std_msgs::msg::Int32MultiArray mode_message;
    mode_message.data.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      mode_message.data.push_back(
        waypoint.interpolation_mode == PathMode::LINEAR ? 0 : 1);
    }
    waypoint_modes_publisher_->publish(mode_message);

    std_msgs::msg::Float64MultiArray joint_values_message;
    joint_values_message.data.reserve(waypoints_.size() * (joint_names_.size() + 1));
    for (const auto & waypoint : waypoints_) {
      if (waypoint.joint_positions && waypoint.joint_positions->size() == joint_names_.size()) {
        joint_values_message.data.insert(
          joint_values_message.data.end(),
          waypoint.joint_positions->begin(), waypoint.joint_positions->end());
      } else {
        joint_values_message.data.insert(
          joint_values_message.data.end(), joint_names_.size(),
          std::numeric_limits<double>::quiet_NaN());
      }
      joint_values_message.data.push_back(waypoint.gripper_position);
    }
    waypoint_joint_values_publisher_->publish(joint_values_message);

    std_msgs::msg::UInt32 start_message;
    start_message.data = static_cast<uint32_t>(start_waypoint_index_);
    selected_start_waypoint_publisher_->publish(start_message);

    std_msgs::msg::UInt32 end_message;
    end_message.data = static_cast<uint32_t>(selected_end_waypoint_index());
    selected_end_waypoint_publisher_->publish(end_message);

    publish_world_pose_state(true);
  }

  void publish_world_pose_state(bool force_waypoints = false)
  {
    if (!joint_state_ready_ || !fk_solver_) {
      return;
    }

    // Display telemetry does not need the controller's joint feedback rate.
    // Bound its DDS/Python callback cost independently of command timing.
    // Plan edits bypass this limit so imported/edited points appear at once.
    const auto wall_time = std::chrono::steady_clock::now();
    if (!force_waypoints && last_world_pose_publish_time_ &&
      std::chrono::duration<double>(wall_time - *last_world_pose_publish_time_).count() <
      1.0 / display_rate_hz_)
    {
      return;
    }

    std::string error;
    const auto world_to_base = world_to_base_frame(error);
    const auto world_to_root = world_to_robot_root_frame(error);
    if (!world_to_base || !world_to_root) {
      return;
    }

    KDL::Frame base_to_tip;
    if (fk_solver_->JntToCart(current_positions_, base_to_tip) < 0) {
      return;
    }

    const auto stamp = now();
    const CartesianPose current_world = from_kdl(*world_to_base * base_to_tip);
    geometry_msgs::msg::PoseStamped current_message;
    current_message.header.frame_id = world_frame_;
    current_message.header.stamp = stamp;
    current_message.pose.position.x = current_world.position[0];
    current_message.pose.position.y = current_world.position[1];
    current_message.pose.position.z = current_world.position[2];
    current_message.pose.orientation.x = current_world.orientation.x;
    current_message.pose.orientation.y = current_world.orientation.y;
    current_message.pose.orientation.z = current_world.orientation.z;
    current_message.pose.orientation.w = current_world.orientation.w;
    current_world_pose_publisher_->publish(current_message);

    const CartesianPose root_world = from_kdl(*world_to_root);
    geometry_msgs::msg::PoseStamped root_message;
    root_message.header = current_message.header;
    root_message.pose.position.x = root_world.position[0];
    root_message.pose.position.y = root_world.position[1];
    root_message.pose.position.z = root_world.position[2];
    root_message.pose.orientation.x = root_world.orientation.x;
    root_message.pose.orientation.y = root_world.orientation.y;
    root_message.pose.orientation.z = root_world.orientation.z;
    root_message.pose.orientation.w = root_world.orientation.w;
    world_root_pose_publisher_->publish(root_message);
    last_world_pose_publish_time_ = wall_time;

    // The waypoint array only changes when the plan or its base frame changes.
    // Keep joint feedback at full rate without making the Python command
    // streamer deserialize the entire plan on every joint-state update.
    if (!force_waypoints && last_world_waypoints_frame_) {
      const auto change = KDL::diff(*last_world_waypoints_frame_, *world_to_base);
      if ((change.vel.Norm() <= 1e-9 && change.rot.Norm() <= 1e-9) ||
        wall_time - last_world_waypoints_publish_time_ < std::chrono::milliseconds(100))
      {
        return;
      }
    }

    geometry_msgs::msg::PoseArray waypoint_message;
    waypoint_message.header = current_message.header;
    waypoint_message.poses.reserve(waypoints_.size());
    for (const auto & waypoint : waypoints_) {
      const CartesianPose world_pose = from_kdl(*world_to_base * to_kdl(waypoint.pose));
      geometry_msgs::msg::Pose pose;
      pose.position.x = world_pose.position[0];
      pose.position.y = world_pose.position[1];
      pose.position.z = world_pose.position[2];
      pose.orientation.x = world_pose.orientation.x;
      pose.orientation.y = world_pose.orientation.y;
      pose.orientation.z = world_pose.orientation.z;
      pose.orientation.w = world_pose.orientation.w;
      waypoint_message.poses.push_back(std::move(pose));
    }
    world_waypoints_publisher_->publish(waypoint_message);
    last_world_waypoints_frame_ = *world_to_base;
    last_world_waypoints_publish_time_ = wall_time;
  }

  void publish_joint_smoothing()
  {
    std_msgs::msg::Bool message;
    message.data = joint_smoothing_enabled_;
    joint_smoothing_enabled_publisher_->publish(message);
  }

  std::size_t selected_end_waypoint_index() const
  {
    if (waypoints_.empty()) {
      return 1;
    }
    if (end_waypoint_index_ == 0 || end_waypoint_index_ > waypoints_.size()) {
      return waypoints_.size();
    }
    return end_waypoint_index_;
  }

  std::vector<std::size_t> selected_waypoint_indices() const
  {
    std::vector<std::size_t> indices;
    if (waypoints_.empty() || start_waypoint_index_ < 1 ||
      start_waypoint_index_ > waypoints_.size())
    {
      return indices;
    }
    const std::size_t end = selected_end_waypoint_index();
    if (end < 1 || end > waypoints_.size()) {
      return indices;
    }
    std::size_t current = start_waypoint_index_;
    while (true) {
      indices.push_back(current - 1);
      if (current == end) {
        break;
      }
      current = current < end ? current + 1 : current - 1;
    }
    return indices;
  }

  std::vector<std::string> command_joint_names() const
  {
    std::vector<std::string> names = joint_names_;
    names.push_back(gripper_joint_);
    return names;
  }

  double next_gripper_target() const
  {
    return waypoints_.empty() ? current_gripper_position_ : waypoints_.back().gripper_position;
  }

  void invalidate_plan()
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = command_joint_names();
    trajectory.header.frame_id = base_link_;
    planned_trajectory_publisher_->publish(trajectory);
    planned_waypoint_targets_publisher_->publish(std_msgs::msg::Float64MultiArray());
    nav_msgs::msg::Path path;
    path.header.frame_id = base_link_;
    path.header.stamp = now();
    preview_path_publisher_->publish(path);
  }

  void robot_description_callback(const std_msgs::msg::String::SharedPtr message)
  {
    if (message->data.empty() || executing_) {
      return;
    }
    if (robot_description_ == message->data && chain_ready_) {
      return;
    }
    initialize_chain(message->data);
  }

  void initialize_chain(const std::string & robot_description)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
      publish_error("failed to parse /robot_description as a KDL tree");
      return;
    }

    urdf::Model urdf_model;
    if (!urdf_model.initString(robot_description)) {
      publish_error("failed to parse /robot_description joint limits");
      return;
    }
    const auto root = urdf_model.getRoot();
    robot_root_link_ = root ? root->name : std::string{};

    KDL::Chain chain;
    if (!tree.getChain(base_link_, tip_link_, chain)) {
      publish_error("KDL chain not found: " + base_link_ + " -> " + tip_link_);
      return;
    }

    world_chain_ready_ = false;
    world_state_ready_ = false;
    world_to_base_joint_names_.clear();
    world_to_base_fk_solver_.reset();
    world_reference_link_ = world_frame_;
    KDL::Chain world_to_base_chain;
    bool has_world_chain = tree.getChain(world_reference_link_, base_link_, world_to_base_chain);
    if (!has_world_chain && normalized_frame(world_frame_) == "world") {
      if (root) {
        world_reference_link_ = root->name;
        has_world_chain = tree.getChain(
          world_reference_link_, base_link_, world_to_base_chain);
      }
    }
    if (has_world_chain) {
      for (
        unsigned int segment_index = 0;
        segment_index < world_to_base_chain.getNrOfSegments(); ++segment_index)
      {
        const KDL::Joint & joint =
          world_to_base_chain.getSegment(segment_index).getJoint();
        if (joint.getType() != KDL::Joint::None) {
          world_to_base_joint_names_.push_back(joint.getName());
        }
      }
      world_to_base_chain_ = world_to_base_chain;
      world_to_base_positions_ = KDL::JntArray(world_to_base_chain_.getNrOfJoints());
      world_to_base_fk_solver_ =
        std::make_unique<KDL::ChainFkSolverPos_recursive>(world_to_base_chain_);
      world_chain_ready_ = true;
      world_state_ready_ = world_to_base_joint_names_.empty();
    }

    std::vector<std::string> joint_names;
    std::vector<double> velocity_limits;
    KDL::JntArray lower(chain.getNrOfJoints());
    KDL::JntArray upper(chain.getNrOfJoints());
    unsigned int joint_index = 0;
    for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
      const KDL::Joint & kdl_joint = chain.getSegment(segment_index).getJoint();
      if (kdl_joint.getType() == KDL::Joint::None) {
        continue;
      }
      const auto urdf_joint = urdf_model.getJoint(kdl_joint.getName());
      if (!urdf_joint) {
        publish_error("URDF joint not found: " + kdl_joint.getName());
        return;
      }
      if (urdf_joint->type == urdf::Joint::CONTINUOUS) {
        lower(joint_index) = -M_PI;
        upper(joint_index) = M_PI;
      } else if (urdf_joint->limits) {
        lower(joint_index) = urdf_joint->limits->lower;
        upper(joint_index) = urdf_joint->limits->upper;
      } else {
        publish_error("joint limits are missing: " + kdl_joint.getName());
        return;
      }
      velocity_limits.push_back(urdf_joint->limits && urdf_joint->limits->velocity > 0 ?
        urdf_joint->limits->velocity : joint_speed_radps_);
      joint_names.push_back(kdl_joint.getName());
      ++joint_index;
    }

    if (joint_index == 0 || joint_index != chain.getNrOfJoints()) {
      publish_error("KDL chain has an invalid movable-joint layout");
      return;
    }

    const auto gripper_joint = urdf_model.getJoint(gripper_joint_);
    if (!gripper_joint) {
      publish_error("URDF gripper joint not found: " + gripper_joint_);
      return;
    }
    if (gripper_joint->type == urdf::Joint::CONTINUOUS) {
      gripper_lower_limit_ = -M_PI;
      gripper_upper_limit_ = M_PI;
    } else if (gripper_joint->limits) {
      gripper_lower_limit_ = gripper_joint->limits->lower;
      gripper_upper_limit_ = gripper_joint->limits->upper;
    } else {
      publish_error("gripper joint limits are missing: " + gripper_joint_);
      return;
    }

    velocity_limits.push_back(gripper_joint->limits && gripper_joint->limits->velocity > 0 ?
      gripper_joint->limits->velocity : gripper_speed_radps_);
    velocity_limits_ = velocity_limits;
    chain_ = chain;
    joint_names_ = std::move(joint_names);
    lower_limits_ = lower;
    upper_limits_ = upper;
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, lower_limits_, upper_limits_, *fk_solver_, *velocity_solver_,
      ik_max_iterations_, ik_epsilon_);
    robot_description_ = robot_description;
    chain_ready_ = true;

    std::ostringstream stream;
    stream << "MODEL_READY: " << base_link_ << " -> " << tip_link_
           << " (" << joint_names_.size() << " joints)";
    publish_status(stream.str());
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::unordered_map<std::string, std::size_t> indexes;
    indexes.reserve(message->name.size());
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      indexes[message->name[index]] = index;
    }

    if (!chain_ready_) {
      return;
    }

    KDL::JntArray positions(joint_names_.size());
    std::vector<double> velocities(joint_names_.size(), 0.0);
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      const auto found = indexes.find(joint_names_[index]);
      if (found == indexes.end() || found->second >= message->position.size()) {
        joint_state_ready_ = false;
        return;
      }
      const double position = message->position[found->second];
      if (!std::isfinite(position)) {
        joint_state_ready_ = false;
        return;
      }
      positions(index) = position;
      if (found->second < message->velocity.size()) {
        velocities[index] = message->velocity[found->second];
      } else {
        velocities[index] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    const auto gripper_found = indexes.find(gripper_joint_);
    if (gripper_found == indexes.end() || gripper_found->second >= message->position.size()) {
      joint_state_ready_ = false;
      return;
    }
    const double gripper_position = message->position[gripper_found->second];
    if (!std::isfinite(gripper_position)) {
      joint_state_ready_ = false;
      return;
    }
    current_gripper_position_ = gripper_position;

    current_positions_ = positions;
    current_velocities_ = std::move(velocities);
    last_joint_state_time_ = now();
    joint_state_ready_ = true;

    if (world_chain_ready_ && !world_to_base_joint_names_.empty()) {
      KDL::JntArray world_positions(world_to_base_joint_names_.size());
      for (std::size_t index = 0; index < world_to_base_joint_names_.size(); ++index) {
        const auto found = indexes.find(world_to_base_joint_names_[index]);
        if (found == indexes.end() || found->second >= message->position.size() ||
          !std::isfinite(message->position[found->second]))
        {
          world_state_ready_ = false;
          return;
        }
        world_positions(index) = message->position[found->second];
      }
      world_to_base_positions_ = world_positions;
      world_state_ready_ = true;
    }
    publish_world_pose_state();
  }

  std::optional<CartesianPose> current_pose(std::string & error) const
  {
    if (!chain_ready_) {
      error = "robot model has not been received";
      return std::nullopt;
    }
    if (!joint_state_ready_) {
      error = "complete joint state has not been received";
      return std::nullopt;
    }
    KDL::Frame frame;
    if (fk_solver_->JntToCart(current_positions_, frame) < 0) {
      error = "forward kinematics failed";
      return std::nullopt;
    }
    return from_kdl(frame);
  }

  bool frame_matches(const std::string & frame_id) const
  {
    return frame_id.empty() || normalized_frame(frame_id) == normalized_frame(base_link_);
  }

  bool world_frame_matches(const std::string & frame_id) const
  {
    return normalized_frame(frame_id) == normalized_frame(world_frame_);
  }

  static std::string normalized_frame(std::string frame_id)
  {
    while (!frame_id.empty() && frame_id.front() == '/') {
      frame_id.erase(frame_id.begin());
    }
    return frame_id;
  }

  void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add a waypoint while executing");
      return;
    }
    if (!frame_matches(message->header.frame_id)) {
      publish_error(
        "clicked point frame must be " + base_link_ + ", got " + message->header.frame_id);
      return;
    }
    std::string error;
    auto pose = current_pose(error);
    if (!pose) {
      publish_error("cannot use clicked point: " + error);
      return;
    }
    pose->position = {message->point.x, message->point.y, message->point.z};
    waypoints_.push_back(
      Waypoint{*pose, std::nullopt, next_gripper_target(), 0.0, path_mode_});
    publish_waypoints();
    invalidate_plan();
    publish_status("WAYPOINT_ADDED: count=" + std::to_string(waypoints_.size()));
  }

  void waypoint_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add a waypoint while executing");
      return;
    }
    if (!joint_state_is_fresh()) {
      publish_error("cannot add a waypoint pose: complete arm and gripper state is unavailable");
      return;
    }
    const bool base_target = frame_matches(message->header.frame_id);
    const bool world_target = world_frame_matches(message->header.frame_id);
    if (!base_target && !world_target) {
      publish_error(
        "waypoint pose frame must be " + base_link_ + " or " + world_frame_ +
        ", got " + message->header.frame_id);
      return;
    }
    try {
      CartesianPose pose;
      pose.position = {message->pose.position.x, message->pose.position.y,
        message->pose.position.z};
      pose.orientation = normalize(Quaternion{
          message->pose.orientation.x,
          message->pose.orientation.y,
          message->pose.orientation.z,
          message->pose.orientation.w});

      std::optional<std::vector<double>> joint_positions;
      if (world_target && !base_target) {
        append_world_waypoint(
          pose, "absolute world target", "ABSOLUTE_WAYPOINT_ADDED");
        return;
      }

      waypoints_.push_back(
        Waypoint{pose, std::move(joint_positions), next_gripper_target(), 0.0, path_mode_});
      publish_waypoints();
      invalidate_plan();
      publish_status(
        (world_target && !base_target ? "ABSOLUTE_WAYPOINT_ADDED: count=" :
        "WAYPOINT_ADDED: count=") + std::to_string(waypoints_.size()));
    } catch (const std::exception & exception) {
      publish_error(std::string("invalid waypoint pose: ") + exception.what());
    }
  }

  std::optional<KDL::Frame> world_to_base_frame(std::string & error) const
  {
    if (world_chain_ready_ && world_to_base_fk_solver_) {
      if (!world_state_ready_) {
        error = "complete world-to-base joint state has not been received";
        return std::nullopt;
      }
      KDL::Frame world_to_base;
      if (world_to_base_fk_solver_->JntToCart(
          world_to_base_positions_, world_to_base) < 0)
      {
        error = "world-to-base forward kinematics failed";
        return std::nullopt;
      }
      return world_to_base;
    }
    return lookup_world_transform(base_link_, error);
  }

  std::optional<KDL::Frame> world_to_robot_root_frame(std::string & error) const
  {
    if (robot_root_link_.empty()) {
      error = "robot root link is unavailable";
      return std::nullopt;
    }
    if (normalized_frame(world_reference_link_) == normalized_frame(robot_root_link_) &&
      world_chain_ready_)
    {
      return KDL::Frame::Identity();
    }
    return lookup_world_transform(robot_root_link_, error);
  }

  std::optional<KDL::Frame> lookup_world_transform(
    const std::string & target_frame, std::string & error) const
  {
    if (normalized_frame(world_frame_) == normalized_frame(target_frame)) {
      return KDL::Frame::Identity();
    }
    if (!tf_buffer_) {
      error = "TF buffer is unavailable";
      return std::nullopt;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        world_frame_, target_frame, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      return KDL::Frame(
        KDL::Rotation::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        KDL::Vector(translation.x, translation.y, translation.z));
    } catch (const tf2::TransformException & exception) {
      error = "TF transform is unavailable: " + world_frame_ + " -> " + target_frame +
        " (" + exception.what() + ")";
      return std::nullopt;
    }
  }

  std::optional<CartesianPose> current_world_pose(std::string & error) const
  {
    if (!joint_state_is_fresh()) {
      error = "current joint state is missing or stale";
      return std::nullopt;
    }
    const auto base_pose = current_pose(error);
    if (!base_pose) {
      return std::nullopt;
    }
    const auto world_to_base = world_to_base_frame(error);
    if (!world_to_base) {
      return std::nullopt;
    }
    return from_kdl(*world_to_base * to_kdl(*base_pose));
  }

  bool append_world_waypoint(
    const CartesianPose & world_pose, const std::string & target_label,
    const std::string & status_prefix)
  {
    if (!std::all_of(
        world_pose.position.begin(), world_pose.position.end(),
        [](double value) {return std::isfinite(value);}))
    {
      publish_error(target_label + " position contains a non-finite value");
      return false;
    }
    std::string error;
    const auto world_to_base = world_to_base_frame(error);
    if (!world_to_base) {
      publish_error(error);
      return false;
    }
    CartesianPose base_pose;
    try {
      base_pose = from_kdl(world_to_base->Inverse() * to_kdl(world_pose));
    } catch (const std::exception & exception) {
      publish_error(target_label + ": " + exception.what());
      return false;
    }
    std::vector<double> solved_positions;
    if (!solve_waypoint_pose(base_pose, solved_positions, error, target_label)) {
      publish_error(error);
      return false;
    }
    waypoints_.push_back(
      Waypoint{
        base_pose, std::move(solved_positions), next_gripper_target(), 0.0, path_mode_});
    publish_waypoints();
    invalidate_plan();
    publish_status(status_prefix + ": count=" + std::to_string(waypoints_.size()));
    return true;
  }

  void absolute_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add an absolute position waypoint while executing");
      return;
    }
    if (!world_frame_matches(message->header.frame_id)) {
      publish_error(
        "absolute position frame must be " + world_frame_ + ", got " +
        message->header.frame_id);
      return;
    }
    std::string error;
    auto target = current_world_pose(error);
    if (!target) {
      publish_error("cannot preserve current orientation: " + error);
      return;
    }
    target->position = {message->point.x, message->point.y, message->point.z};
    append_world_waypoint(
      *target, "absolute world XYZ target", "ABSOLUTE_POSITION_WAYPOINT_ADDED");
  }

  void absolute_orientation_callback(
    const geometry_msgs::msg::QuaternionStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add an absolute orientation waypoint while executing");
      return;
    }
    if (!world_frame_matches(message->header.frame_id)) {
      publish_error(
        "absolute orientation frame must be " + world_frame_ + ", got " +
        message->header.frame_id);
      return;
    }
    std::string error;
    auto target = current_world_pose(error);
    if (!target) {
      publish_error("cannot preserve current position: " + error);
      return;
    }
    try {
      target->orientation = normalize(Quaternion{
          message->quaternion.x, message->quaternion.y,
          message->quaternion.z, message->quaternion.w});
    } catch (const std::exception & exception) {
      publish_error(std::string("invalid absolute orientation: ") + exception.what());
      return;
    }
    append_world_waypoint(
      *target, "absolute world RPY target", "ABSOLUTE_ORIENTATION_WAYPOINT_ADDED");
  }

  int solve_ik(
    const KDL::JntArray & seed, const KDL::Frame & pose, KDL::JntArray & solution) const
  {
    // Retain the established IK branch, including valid boundary poses, when
    // optional refinement is off. Inward reseeding belongs to the new planner.
    if (!refinement_.enabled) {return ik_solver_->CartToJnt(seed, pose, solution);}
    LimitIkDiagnostics diagnostics;
    const int result = limit_aware_ik(chain_, *fk_solver_, *velocity_solver_, *ik_solver_,
      lower_limits_, upper_limits_, seed, pose, solution, ik_max_iterations_, ik_epsilon_,
      &diagnostics, max_joint_step_rad_);
    if (diagnostics.attempts > 1) {
      ++ik_searches_;
      if (result >= 0 && diagnostics.relaxed) {++ik_relaxed_solutions_;}
      RCLCPP_DEBUG(get_logger(), "IK inward search: attempts=%u, margin=%g rad, relaxed=%d",
        diagnostics.attempts, diagnostics.minimum_margin, diagnostics.relaxed);
    }
    if (result >= 0) {ik_min_margin_ = std::min(ik_min_margin_, diagnostics.minimum_margin);}
    return result;
  }

  bool solve_waypoint_pose(
    const CartesianPose & target, std::vector<double> & stored_positions,
    std::string & error, const std::string & target_label)
  {
    if (!chain_ready_ || !joint_state_ready_) {
      error = "robot model or complete joint state is unavailable";
      return false;
    }

    KDL::JntArray seed = current_positions_;
    if (!waypoints_.empty()) {
      const Waypoint & previous = waypoints_.back();
      if (previous.joint_positions) {
        for (unsigned int index = 0; index < seed.rows(); ++index) {
          seed(index) = previous.joint_positions->at(index);
        }
      } else {
        KDL::JntArray previous_solution(chain_.getNrOfJoints());
        const int previous_result = solve_ik(
          seed, to_kdl(previous.pose), previous_solution);
        if (previous_result < 0) {
          error = "IK failed for the previous waypoint (code " +
            std::to_string(previous_result) + ")";
          return false;
        }
        seed = previous_solution;
      }
    }

    KDL::JntArray solution(chain_.getNrOfJoints());
    const int result = solve_ik(seed, to_kdl(target), solution);
    if (result < 0) {
      error = target_label + " IK failed (code " + std::to_string(result) + ")";
      return false;
    }
    stored_positions.resize(solution.rows());
    for (unsigned int index = 0; index < solution.rows(); ++index) {
      const double value = solution(index);
      if (!std::isfinite(value) || value < lower_limits_(index) - 1.0e-9 ||
        value > upper_limits_(index) + 1.0e-9)
      {
        error = target_label + " IK is outside the URDF limit for " + joint_names_[index];
        return false;
      }
      stored_positions[index] = value;
    }
    return true;
  }

  void relative_reference_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change the relative reference while executing");
      return;
    }
    const int32_t reference = message->data;
    if (reference < -1 ||
      (reference > 0 && static_cast<std::size_t>(reference) > waypoints_.size()))
    {
      publish_error("relative reference is outside the current waypoint list");
      return;
    }
    relative_reference_index_ = reference;
    const std::string label = reference < 0 ? "last" :
      (reference == 0 ? "current" : "waypoint=" + std::to_string(reference));
    publish_status("RELATIVE_REFERENCE_SELECTED: " + label);
  }

  void relative_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add a relative waypoint while executing");
      return;
    }
    if (message->header.frame_id != tip_link_ && message->header.frame_id != "/" + tip_link_) {
      publish_error(
        "relative waypoint frame must be " + tip_link_ + ", got " +
        message->header.frame_id);
      return;
    }

    std::string error;
    const auto live_pose = current_pose(error);
    if (!live_pose) {
      publish_error("cannot add a relative waypoint: " + error);
      return;
    }

    try {
      CartesianPose delta;
      delta.position = {
        message->pose.position.x, message->pose.position.y, message->pose.position.z};
      if (!std::all_of(delta.position.begin(), delta.position.end(), [](double value) {
          return std::isfinite(value);
        }))
      {
        throw std::invalid_argument("translation contains a non-finite value");
      }
      delta.orientation = normalize(Quaternion{
          message->pose.orientation.x,
          message->pose.orientation.y,
          message->pose.orientation.z,
          message->pose.orientation.w});

      const int32_t requested_reference = relative_reference_index_;
      relative_reference_index_ = -1;
      const CartesianPose * reference = nullptr;
      if (requested_reference == 0 || waypoints_.empty()) {
        reference = &*live_pose;
      } else if (requested_reference < 0) {
        reference = &waypoints_.back().pose;
      } else if (static_cast<std::size_t>(requested_reference) <= waypoints_.size()) {
        reference = &waypoints_[static_cast<std::size_t>(requested_reference - 1)].pose;
      } else {
        publish_error("relative reference is outside the current waypoint list");
        return;
      }
      const CartesianPose target = from_kdl(to_kdl(*reference) * to_kdl(delta));
      std::vector<double> joint_positions;
      if (!solve_waypoint_pose(target, joint_positions, error, "relative target")) {
        publish_error(error);
        return;
      }
      waypoints_.push_back(
        Waypoint{
          target, std::move(joint_positions), next_gripper_target(), 0.0, path_mode_});
      publish_waypoints();
      invalidate_plan();
      publish_status("RELATIVE_WAYPOINT_ADDED: count=" + std::to_string(waypoints_.size()));
    } catch (const std::exception & exception) {
      publish_error(std::string("invalid relative waypoint: ") + exception.what());
    }
  }

  void frame_relative_waypoint_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add a frame-relative waypoint while executing");
      return;
    }
    if (!world_frame_matches(message->header.frame_id)) {
      publish_error(
        "frame-relative waypoint frame must be " + world_frame_ + ", got " +
        message->header.frame_id);
      return;
    }

    std::string error;
    const auto live_pose = current_pose(error);
    if (!live_pose) {
      publish_error("cannot add a frame-relative waypoint: " + error);
      return;
    }
    const auto world_to_base = world_to_base_frame(error);
    if (!world_to_base) {
      publish_error("cannot add a frame-relative waypoint: " + error);
      return;
    }

    try {
      const std::array<double, 3> translation = {
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z,
      };
      if (!std::all_of(translation.begin(), translation.end(), [](double value) {
          return std::isfinite(value);
        }))
      {
        throw std::invalid_argument("translation contains a non-finite value");
      }
      const Quaternion delta_orientation = normalize(Quaternion{
          message->pose.orientation.x,
          message->pose.orientation.y,
          message->pose.orientation.z,
          message->pose.orientation.w});

      const int32_t requested_reference = relative_reference_index_;
      relative_reference_index_ = -1;
      const CartesianPose * reference = nullptr;
      if (requested_reference == 0 || waypoints_.empty()) {
        reference = &*live_pose;
      } else if (requested_reference < 0) {
        reference = &waypoints_.back().pose;
      } else if (static_cast<std::size_t>(requested_reference) <= waypoints_.size()) {
        reference = &waypoints_[static_cast<std::size_t>(requested_reference - 1)].pose;
      } else {
        publish_error("frame-relative reference is outside the current waypoint list");
        return;
      }

      const KDL::Frame reference_world = *world_to_base * to_kdl(*reference);
      const KDL::Rotation world_delta_rotation = KDL::Rotation::Quaternion(
        delta_orientation.x, delta_orientation.y,
        delta_orientation.z, delta_orientation.w);
      KDL::Frame target_world;
      target_world.p = reference_world.p + KDL::Vector(
        translation[0], translation[1], translation[2]);
      // The delta is expressed in the fixed world frame, so apply it from
      // the left. This differs from the gripper-local T_reference * T_delta
      // composition used by relative_waypoint_callback().
      target_world.M = world_delta_rotation * reference_world.M;
      const CartesianPose target = from_kdl(world_to_base->Inverse() * target_world);

      std::vector<double> joint_positions;
      if (!solve_waypoint_pose(
          target, joint_positions, error, "world-frame relative target"))
      {
        publish_error(error);
        return;
      }
      waypoints_.push_back(
        Waypoint{
          target, std::move(joint_positions), next_gripper_target(), 0.0, path_mode_});
      publish_waypoints();
      invalidate_plan();
      publish_status(
        "FRAME_RELATIVE_WAYPOINT_ADDED: count=" + std::to_string(waypoints_.size()));
    } catch (const std::exception & exception) {
      publish_error(std::string("invalid frame-relative waypoint: ") + exception.what());
    }
  }

  void cartesian_jog_callback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot jog while executing");
      return;
    }
    if (!world_frame_matches(message->header.frame_id)) {
      publish_error(
        "Cartesian jog frame must be " + world_frame_ + ", got " +
        message->header.frame_id);
      return;
    }
    if (!joint_state_is_fresh()) {
      publish_error("cannot jog: current joint state is missing or stale");
      return;
    }

    const std::array<double, 3> delta = {
      message->point.x, message->point.y, message->point.z};
    std::size_t active_axes = 0;
    for (const double value : delta) {
      if (!std::isfinite(value)) {
        publish_error("Cartesian jog delta contains a non-finite value");
        return;
      }
      if (std::abs(value) > 1.0e-12) {
        ++active_axes;
      }
    }
    const double distance = std::sqrt(
      delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    if (active_axes != 1 || distance <= 0.0 ||
      distance > max_cartesian_jog_step_m_ + 1.0e-12)
    {
      publish_error(
        "Cartesian jog must contain exactly one axis within " +
        std::to_string(max_cartesian_jog_step_m_) + " m");
      return;
    }

    std::string error;
    auto target_world = current_world_pose(error);
    if (!target_world) {
      publish_error("cannot jog: " + error);
      return;
    }
    for (std::size_t index = 0; index < delta.size(); ++index) {
      target_world->position[index] += delta[index];
    }
    const auto world_to_base = world_to_base_frame(error);
    if (!world_to_base) {
      publish_error("cannot jog: " + error);
      return;
    }

    KDL::JntArray solution(chain_.getNrOfJoints());
    try {
      const KDL::Frame base_target = world_to_base->Inverse() * to_kdl(*target_world);
      const int result = solve_ik(current_positions_, base_target, solution);
      if (result < 0) {
        publish_error("Cartesian jog IK failed (code " + std::to_string(result) + ")");
        return;
      }
    } catch (const std::exception & exception) {
      publish_error(std::string("invalid Cartesian jog target: ") + exception.what());
      return;
    }

    sensor_msgs::msg::JointState target;
    target.header.stamp = now();
    target.name = command_joint_names();
    target.position.reserve(target.name.size());
    for (unsigned int index = 0; index < solution.rows(); ++index) {
      const double value = solution(index);
      if (!std::isfinite(value) || value < lower_limits_(index) - 1.0e-9 ||
        value > upper_limits_(index) + 1.0e-9)
      {
        publish_error("Cartesian jog IK is outside the URDF joint limits");
        return;
      }
      if (std::abs(value - current_positions_(index)) > max_joint_step_rad_ + 1.0e-9) {
        publish_error(
          "Cartesian jog joint step exceeds the configured limit for " +
          joint_names_[index]);
        return;
      }
      target.position.push_back(value);
    }
    target.position.push_back(current_gripper_position_);
    cartesian_jog_target_publisher_->publish(target);
    publish_status("CARTESIAN_JOG_TARGET_READY");
  }

  void joint_waypoint_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot add a joint waypoint while executing");
      return;
    }
    if (!chain_ready_) {
      publish_error("cannot add a joint waypoint: robot model has not been received");
      return;
    }

    std::unordered_map<std::string, std::size_t> indexes;
    indexes.reserve(message->name.size());
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      indexes[message->name[index]] = index;
    }

    KDL::JntArray positions(joint_names_.size());
    std::vector<double> stored_positions(joint_names_.size());
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      const auto found = indexes.find(joint_names_[index]);
      if (found == indexes.end() || found->second >= message->position.size()) {
        publish_error("joint waypoint is missing " + joint_names_[index]);
        return;
      }
      const double value = message->position[found->second];
      if (!std::isfinite(value)) {
        publish_error("joint waypoint contains a non-finite value for " + joint_names_[index]);
        return;
      }
      if (value < lower_limits_(index) || value > upper_limits_(index)) {
        publish_error("joint waypoint is outside the URDF limit for " + joint_names_[index]);
        return;
      }
      positions(index) = value;
      stored_positions[index] = value;
    }

    const auto gripper_found = indexes.find(gripper_joint_);
    if (gripper_found == indexes.end() || gripper_found->second >= message->position.size()) {
      publish_error("joint waypoint is missing " + gripper_joint_);
      return;
    }
    const double gripper_position = message->position[gripper_found->second];
    if (!std::isfinite(gripper_position)) {
      publish_error("joint waypoint contains a non-finite value for " + gripper_joint_);
      return;
    }
    if (gripper_position < gripper_lower_limit_ || gripper_position > gripper_upper_limit_) {
      publish_error("joint waypoint is outside the URDF limit for " + gripper_joint_);
      return;
    }

    KDL::Frame frame;
    if (fk_solver_->JntToCart(positions, frame) < 0) {
      publish_error("forward kinematics failed for joint waypoint");
      return;
    }
    waypoints_.push_back(
      Waypoint{
        from_kdl(frame), std::move(stored_positions), gripper_position, 0.0, path_mode_});
    publish_waypoints();
    invalidate_plan();
    publish_status("JOINT_WAYPOINT_ADDED: count=" + std::to_string(waypoints_.size()));
  }

  void tool_path_correction_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot generate a corrected tool path while executing");
      return;
    }
    if (!chain_ready_ || !ik_solver_) {
      publish_error("cannot generate a corrected tool path: robot model is not ready");
      return;
    }
    if (message->data.size() != 5) {
      publish_error(
        "tool path correction requires approach start, grasp waypoint, and local XYZ");
      return;
    }

    const auto parse_index = [](double value, const char * label) -> std::size_t {
        if (!std::isfinite(value) || value < 1.0 ||
          std::abs(value - std::round(value)) > 1.0e-9)
        {
          throw std::invalid_argument(std::string(label) + " must be a positive integer");
        }
        return static_cast<std::size_t>(std::llround(value));
      };

    try {
      const std::size_t approach_start = parse_index(message->data[0], "approach start");
      const std::size_t grasp_waypoint = parse_index(message->data[1], "grasp waypoint");
      if (waypoints_.empty()) {
        throw std::invalid_argument("cannot generate a corrected tool path from an empty list");
      }
      if (approach_start > grasp_waypoint || grasp_waypoint > waypoints_.size()) {
        throw std::invalid_argument(
                "tool path correction waypoint range is outside the current list");
      }

      const std::array<double, 3> local_translation{
        message->data[2], message->data[3], message->data[4]};
      const double correction_norm = std::sqrt(
        local_translation[0] * local_translation[0] +
        local_translation[1] * local_translation[1] +
        local_translation[2] * local_translation[2]);
      if (!std::isfinite(correction_norm) || correction_norm > max_tool_path_correction_m_) {
        throw std::invalid_argument(
                "tool path correction exceeds max_tool_path_correction_m");
      }

      std::vector<Waypoint> corrected = waypoints_;
      for (std::size_t index = approach_start - 1; index < waypoints_.size(); ++index) {
        const Waypoint & reference = waypoints_[index];
        if (!reference.joint_positions ||
          reference.joint_positions->size() != chain_.getNrOfJoints())
        {
          throw std::runtime_error(
                  "reference waypoint " + std::to_string(index + 1) +
                  " has no stored joint target");
        }
        KDL::JntArray seed(chain_.getNrOfJoints());
        for (unsigned int joint = 0; joint < seed.rows(); ++joint) {
          seed(joint) = reference.joint_positions->at(joint);
        }

        const double blend = tool_path_correction_blend(
          index + 1, approach_start, grasp_waypoint);
        if (blend <= 0.0) {
          continue;
        }

        const CartesianPose target = apply_local_translation(
          reference.pose, local_translation, blend);
        KDL::JntArray solution(chain_.getNrOfJoints());
        const int result = solve_ik(seed, to_kdl(target), solution);
        if (result < 0) {
          throw std::runtime_error(
                  "corrected path IK failed at waypoint " +
                  std::to_string(index + 1) + " (code " +
                  std::to_string(result) + ")");
        }

        std::vector<double> stored(solution.rows());
        for (unsigned int joint = 0; joint < solution.rows(); ++joint) {
          const double value = solution(joint);
          if (!std::isfinite(value) || value < lower_limits_(joint) - 1.0e-9 ||
            value > upper_limits_(joint) + 1.0e-9)
          {
            throw std::runtime_error(
                    "corrected path is outside the URDF limit for " +
                    joint_names_[joint] + " at waypoint " +
                    std::to_string(index + 1));
          }
          stored[joint] = value;
        }
        corrected[index].pose = target;
        corrected[index].joint_positions = std::move(stored);
      }

      waypoints_ = std::move(corrected);
      invalidate_plan();
      publish_waypoints();
      publish_status(
        "TOOL_PATH_GENERATED: approach_start=" + std::to_string(approach_start) +
        ", grasp=" + std::to_string(grasp_waypoint) +
        ", count=" + std::to_string(waypoints_.size()));
    } catch (const std::exception & exception) {
      publish_error(exception.what());
    }
  }

  void path_mode_callback(const std_msgs::msg::String::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change path mode while executing");
      return;
    }
    try {
      path_mode_ = parse_path_mode(message->data);
      invalidate_plan();
      publish_status("DEFAULT_PATH_MODE_SELECTED: " + path_mode_name());
    } catch (const std::exception & exception) {
      publish_error(exception.what());
    }
  }

  void waypoint_dwell_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change waypoint dwell while executing");
      return;
    }
    if (message->data.size() != 2) {
      publish_error("waypoint dwell requires [one-based index, seconds]");
      return;
    }
    const double index_value = message->data[0];
    const double dwell_s = message->data[1];
    if (!std::isfinite(index_value) || index_value < 1.0 ||
      std::floor(index_value) != index_value)
    {
      publish_error("waypoint dwell index must be a positive integer");
      return;
    }
    const auto index = static_cast<std::size_t>(index_value);
    if (index > waypoints_.size()) {
      publish_error("waypoint dwell index is outside the current list");
      return;
    }
    if (!std::isfinite(dwell_s) || dwell_s < 0.0 || dwell_s > max_waypoint_dwell_s_) {
      publish_error(
        "waypoint dwell must be between 0 and " +
        std::to_string(max_waypoint_dwell_s_) + " seconds");
      return;
    }
    waypoints_[index - 1].dwell_s = dwell_s;
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_DWELL_UPDATED: index=" + std::to_string(index) +
      ", dwell_s=" + std::to_string(dwell_s));
  }

  void waypoint_speed_scale_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change waypoint speed while executing");
      return;
    }
    if (message->data.size() != 2) {
      publish_error("waypoint speed requires [one-based index, scale]");
      return;
    }
    const double index_value = message->data[0];
    const double speed_scale = message->data[1];
    if (!std::isfinite(index_value) || index_value < 1.0 ||
      std::floor(index_value) != index_value)
    {
      publish_error("waypoint speed index must be a positive integer");
      return;
    }
    const auto index = static_cast<std::size_t>(index_value);
    if (index > waypoints_.size()) {
      publish_error("waypoint speed index is outside the current list");
      return;
    }
    if (!std::isfinite(speed_scale) || speed_scale < 0.01 ||
      speed_scale > max_waypoint_speed_scale_)
    {
      publish_error(
        "waypoint speed scale must be between 0.01 and " +
        std::to_string(max_waypoint_speed_scale_));
      return;
    }
    waypoints_[index - 1].speed_scale = speed_scale;
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_SPEED_UPDATED: index=" + std::to_string(index) +
      ", speed_scale=" + std::to_string(speed_scale));
  }

  void assign_uniform_total_time(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (executing_) {publish_error("cannot assign timing while executing"); return;}
    const auto saved_timing = joint_timing_;
    const auto saved_start = start_waypoint_index_;
    const auto saved_end = end_waypoint_index_;
    const auto saved_positions = current_positions_;
    const double saved_gripper = current_gripper_position_;
    const bool saved_ready = joint_state_ready_;
    const bool offline = !joint_state_is_fresh();
    auto restore = [&]() {
        joint_timing_ = saved_timing;
        start_waypoint_index_ = saved_start;
        end_waypoint_index_ = saved_end;
        current_positions_ = saved_positions;
        current_gripper_position_ = saved_gripper;
        joint_state_ready_ = saved_ready;
      };
    try {
      if (!chain_ready_ || waypoints_.size() < 2 || (!refinement_.enabled &&
        (!joint_smoothing_enabled_ || !std::all_of(waypoints_.begin(), waypoints_.end(),
          [](const Waypoint & w) {return w.interpolation_mode == PathMode::JOINT;}))))
      {
        throw std::invalid_argument(
          "uniform TCP timing requires a continuous or refined JOINT path");
      }
      if (message->data.empty() || !std::isfinite(message->data[0]) || message->data[0] <= 0.0) {
        throw std::invalid_argument("uniform timing requires a positive total duration");
      }
      const double target = message->data[0];
      std::vector<std::size_t> excluded;
      for (std::size_t i = 1; i < message->data.size(); ++i) {
        const double v = message->data[i];
        if (!std::isfinite(v) || std::floor(v) != v || v < 2 || v > waypoints_.size()) {
          throw std::invalid_argument("invalid preserved waypoint number");
        }
        excluded.push_back(static_cast<std::size_t>(v));
      }
      std::sort(excluded.begin(), excluded.end());
      excluded.erase(std::unique(excluded.begin(), excluded.end()), excluded.end());
      if (!excluded.empty() &&
        (excluded.back()-excluded.front()+1 != excluded.size() ||
        excluded.size() >= waypoints_.size()-1))
      {
        throw std::invalid_argument("uniform timing requires one contiguous preserved range");
      }
      if (offline) {
        std::vector<double> initial;
        std::string error;
        if (!resolve_joint_target(waypoints_.front(), current_positions_, 0, initial, error)) {
          throw std::runtime_error(error);
        }
        current_positions_ = KDL::JntArray(joint_names_.size());
        for (std::size_t j = 0; j < joint_names_.size(); ++j) {current_positions_(j) = initial[j];}
        current_gripper_position_ = initial.back();
        joint_state_ready_ = true;
      }
      // Preserve the original joint spline and user speed scales. Its FK arc
      // derivative, rather than waypoint count, controls the new clock.
      start_waypoint_index_ = 1;
      end_waypoint_index_ = waypoints_.size();
      joint_timing_ = {max_uniform_tcp_speed_mps_,
        excluded.empty() ? 0.0 : static_cast<double>(excluded.front()),
        excluded.empty() ? 0.0 : static_cast<double>(excluded.back())};
      const std::size_t stride = joint_names_.size()+2;
      auto duration = [&](const Plan & plan) {
          if (plan.trajectory.points.empty() ||
            plan.waypoint_targets.data.size() != stride*waypoints_.size())
          {throw std::runtime_error("uniform plan is missing waypoint arrival times");}
          const auto & t = plan.trajectory.points.back().time_from_start;
          return static_cast<double>(t.sec)+1.e-9*t.nanosec-plan.waypoint_targets.data[0];
        };
      Plan verified;
      std::string error;
      if (!make_plan(verified, error, false)) {throw std::runtime_error(error);}
      const double minimum = duration(verified);
      if (target < minimum-0.001) {
        throw std::invalid_argument("target is shorter than the feasible uniform duration: " +
          std::to_string(minimum) + " seconds (TCP speed <= " +
          std::to_string(max_uniform_tcp_speed_mps_ * 1000.0) +
          " mm/s; joint velocity/acceleration limits also apply)");
      }
      double actual = minimum, lo = 0.0, hi = max_uniform_tcp_speed_mps_;
      // A bounded clock search leaves the geometric curve and every waypoint
      // scale untouched. Each candidate respects the same joint envelopes.
      for (unsigned int i = 0; std::abs(actual-target) > 0.001 && i < 28; ++i) {
        const double speed = (lo+hi)*0.5;
        joint_timing_[0] = speed;
        Plan candidate;
        if (!make_plan(candidate, error, false)) {throw std::runtime_error(error);}
        actual = duration(candidate);
        verified = std::move(candidate);
        if (actual > target) {lo = speed;} else {hi = speed;}
      }
      if (std::abs(actual-target) > 0.001) {
        throw std::runtime_error("uniform target duration did not converge");
      }
      const auto committed_timing = joint_timing_;
      const double alignment = verified.waypoint_targets.data[0];
      const double execution_total = actual+alignment;
      start_waypoint_index_ = saved_start;
      end_waypoint_index_ = saved_end;
      Plan display;
      if (saved_start == 1 && selected_end_waypoint_index() == waypoints_.size()) {
        display = std::move(verified);
      } else if (!make_plan(display, error, false)) {
        throw std::runtime_error(error);
      }
      restore();
      joint_timing_ = committed_timing;
      invalidate_plan();
      std_msgs::msg::Float64MultiArray timing_message;
      timing_message.data = joint_timing_;
      joint_timing_publisher_->publish(timing_message);
      publish_waypoints();
      publish_plan(display);
      double dwell = 0.0;
      for (const auto & w : waypoints_) {dwell += w.dwell_s;}
      std::ostringstream status;
      status << std::setprecision(17) << "WAYPOINT_TIMES_VERIFIED: duration_s=0, updated=" <<
        waypoints_.size()-1-excluded.size() << ", excluded=" << excluded.size() <<
        ", max_error_s=0, total_s=" << actual << ", target_total_s=" << target <<
        ", total_error_s=" << std::abs(actual-target) << ", alignment_s=" << alignment <<
        ", execution_total_s=" << execution_total << ", fixed_s=" << dwell <<
        ", speed_mps=" << joint_timing_[0] << ", mode=uniform, reference=" <<
        (offline ? "first_waypoint" : "live_joint_state");
      publish_status(status.str());
    } catch (const std::exception & e) {
      restore();
      publish_error(std::string("uniform timing failed; settings restored: ")+e.what());
    }
  }

  void equal_segment_time_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    assign_segment_times(message, false);
  }

  void assign_segment_times(
    const std_msgs::msg::Float64MultiArray::SharedPtr message, bool total_time)
  {
    if (executing_) {publish_error("cannot assign segment times while executing"); return;}
    if (!chain_ready_ || waypoints_.size() < 2) {
      publish_error("equal timing requires a robot model and at least two waypoints");
      return;
    }
    const auto saved_waypoints = waypoints_;
    const auto saved_timing = joint_timing_;
    const auto saved_start = start_waypoint_index_;
    const auto saved_end = end_waypoint_index_;
    const auto saved_positions = current_positions_;
    const double saved_gripper_position = current_gripper_position_;
    const bool saved_joint_state_ready = joint_state_ready_;
    const bool offline = !joint_state_is_fresh();
    try {
      if (message->data.empty()) {throw std::invalid_argument("missing duration");}
      if (offline) {
        if (current_positions_.rows() != chain_.getNrOfJoints()) {
          current_positions_ = KDL::JntArray(chain_.getNrOfJoints());
        }
        std::vector<double> initial;
        std::string error;
        if (!resolve_joint_target(waypoints_.front(), current_positions_, 0, initial, error)) {
          throw std::runtime_error(error);
        }
        for (unsigned int j = 0; j < current_positions_.rows(); ++j) {
          current_positions_(j) = initial[j];
        }
        current_gripper_position_ = initial.back();
        // Only the local preview uses this seed. Never publish measured state.
        joint_state_ready_ = true;
      }
      std::vector<bool> excluded(waypoints_.size(), false);
      excluded[0] = true;
      for (std::size_t j = 1; j < message->data.size(); ++j) {
        const double index = message->data[j];
        if (!std::isfinite(index) || index < 2.0 || index > waypoints_.size() ||
          std::floor(index) != index)
        {
          throw std::invalid_argument("excluded destination must be in 2..waypoint count");
        }
        // Outside the preserved range, the old TCP clock changes actual time.
        // Reject rather than silently changing the time of an excluded segment.
        if (saved_timing[0] > 0.0 &&
          (index < saved_timing[1] || index > saved_timing[2]))
        {
          throw std::invalid_argument(
            "with TCP timing active, exclusions must be inside its preserved insertion range");
        }
        excluded[static_cast<std::size_t>(index) - 1] = true;
      }
      std::vector<double> base(waypoints_.size(), 0.0), scales, dwells;
      KDL::JntArray seed = current_positions_;
      std::vector<double> previous;
      std::string error;
      for (std::size_t i = 0; i < waypoints_.size(); ++i) {
        const auto & waypoint = waypoints_[i];
        std::vector<double> target;
        if (!resolve_joint_target(waypoint, seed, i, target, error)) {
          throw std::runtime_error(error);
        }
        if (i > 0) {
          base[i] = waypoint.interpolation_mode == PathMode::JOINT ?
            joint_segment_duration_s(previous, target, 1.0) :
            std::max(
              segment_duration(waypoints_[i - 1].pose, waypoint.pose,
                linear_speed_mps_, angular_speed_radps_, minimum_segment_duration_s_),
              std::abs(target.back() - previous.back()) / gripper_speed_radps_);
        }
        for (unsigned int j = 0; j < seed.rows(); ++j) {seed(j) = target[j];}
        previous = target;
        scales.push_back(waypoint.speed_scale);
        dwells.push_back(waypoint.dwell_s);
      }
      if (refinement_.enabled) {
        // Measure the same refined planner used for preview/execution. IK
        // detours may need longer than an endpoint-only duration estimate.
        start_waypoint_index_ = 1;
        end_waypoint_index_ = waypoints_.size();
        for (auto & waypoint : waypoints_) {waypoint.speed_scale = 1.0;}
        Plan baseline;
        if (!make_plan(baseline, error, false)) {throw std::runtime_error(error);}
        const std::size_t stride = joint_names_.size() + 2;
        if (baseline.waypoint_targets.data.size() != stride * waypoints_.size()) {
          throw std::runtime_error("refined baseline is missing waypoint arrival times");
        }
        for (std::size_t i = 1; i < waypoints_.size(); ++i) {
          base[i] = baseline.waypoint_targets.data[i * stride] -
            baseline.waypoint_targets.data[(i - 1) * stride] - waypoints_[i - 1].dwell_s;
        }
        for (std::size_t i = 0; i < waypoints_.size(); ++i) {waypoints_[i].speed_scale = scales[i];}
      }
      auto result = total_time ? equal_segment_scales_for_total(
        base, scales, excluded, dwells, message->data[0], max_waypoint_speed_scale_) :
        equal_segment_scales(base, scales, excluded, message->data[0], max_waypoint_speed_scale_);
      joint_timing_ = {0.0, 0.0, 0.0};
      start_waypoint_index_ = 1;
      end_waypoint_index_ = waypoints_.size();
      Plan verification;
      for (unsigned int attempt = 0; ; ++attempt) {
        for (std::size_t i = 1; i < waypoints_.size(); ++i) {
          waypoints_[i].speed_scale = result.scales[i];
        }
        verification = Plan();
        if (make_plan(verification, error, false)) {break;}
        const bool step_error = error.find("sample step exceeds the limit") != std::string::npos ||
          error.find("discontinuity") != std::string::npos;
        // Auto timing may lengthen every included segment together. Keep the
        // existing sample-step limits and excluded segment times unchanged.
        const double next_duration = std::min(result.maximum_s, result.duration_s * 1.5);
        if (total_time || message->data[0] != 0.0 || !step_error || attempt >= 12 ||
          next_duration <= result.duration_s)
        {
          throw std::runtime_error(error);
        }
        result = equal_segment_scales(
          base, scales, excluded, next_duration, max_waypoint_speed_scale_);
      }
      const std::size_t stride = joint_names_.size() + 2;
      const auto & arrivals = verification.waypoint_targets.data;
      if (arrivals.size() != stride * waypoints_.size()) {
        throw std::runtime_error("verification is missing waypoint arrival times");
      }
      double max_error = 0.0;
      std::size_t updated = 0, excluded_count = 0;
      for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        const double actual = arrivals[i * stride] - arrivals[(i - 1) * stride] -
          waypoints_[i - 1].dwell_s;
        const double expected = excluded[i] ? base[i] / scales[i] : result.duration_s;
        const double deviation = std::abs(actual - expected);
        if (!std::isfinite(actual) || deviation > 1.e-6) {
          throw std::runtime_error("planned segment time does not match the assignment");
        }
        if (excluded[i]) {++excluded_count;} else {
          ++updated;
          max_error = std::max(max_error, deviation);
        }
      }
      const auto & finish = verification.trajectory.points.back().time_from_start;
      const double execution_total = static_cast<double>(finish.sec) + 1.e-9 * finish.nanosec;
      const double alignment = arrivals[0];
      const double actual_total = execution_total - alignment;
      const double total_error = total_time ? std::abs(actual_total - message->data[0]) : 0.0;
      if (!std::isfinite(actual_total) || (total_time && total_error > 1.e-6)) {
        throw std::runtime_error("planned total time does not match the target");
      }
      start_waypoint_index_ = saved_start;
      end_waypoint_index_ = saved_end;
      Plan display_plan;
      if (saved_start != 1 || selected_end_waypoint_index() != waypoints_.size()) {
        if (!make_plan(display_plan, error, false)) {throw std::runtime_error(error);}
      } else {display_plan = std::move(verification);}
      current_positions_ = saved_positions;
      current_gripper_position_ = saved_gripper_position;
      joint_state_ready_ = saved_joint_state_ready;
      reset_joint_timing();
      invalidate_plan();
      publish_waypoints();
      publish_plan(display_plan);
      std::ostringstream status;
      status << std::setprecision(17) << "WAYPOINT_TIMES_VERIFIED: duration_s=" <<
        result.duration_s << ", updated=" << updated << ", excluded=" << excluded_count <<
        ", max_error_s=" << max_error << ", total_s=" << actual_total <<
        ", target_total_s=" << (total_time ? message->data[0] : 0.0) <<
        ", total_error_s=" << total_error << ", alignment_s=" << alignment <<
        ", execution_total_s=" << execution_total << ", fixed_s=" << result.fixed_s <<
        ", mode=" << (total_time ? "total" : "segment") << ", reference=" <<
        (offline ? "first_waypoint" : "live_joint_state");
      publish_status(status.str());
    } catch (const std::exception & exception) {
      waypoints_ = saved_waypoints;
      joint_timing_ = saved_timing;
      start_waypoint_index_ = saved_start;
      end_waypoint_index_ = saved_end;
      current_positions_ = saved_positions;
      current_gripper_position_ = saved_gripper_position;
      joint_state_ready_ = saved_joint_state_ready;
      publish_error(std::string("equal-time verification failed; settings restored: ") + exception.what());
    }
  }

  void waypoint_mode_callback(const std_msgs::msg::Int32MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change waypoint interpolation mode while executing");
      return;
    }
    if (message->data.size() != 2) {
      publish_error("waypoint mode requires [one-based index, mode code]");
      return;
    }
    const int32_t index_value = message->data[0];
    const int32_t mode_value = message->data[1];
    if (index_value < 1 || static_cast<std::size_t>(index_value) > waypoints_.size()) {
      publish_error("waypoint mode index is outside the current list");
      return;
    }
    if (mode_value != 0 && mode_value != 1) {
      publish_error("waypoint mode code must be 0 (LINEAR) or 1 (JOINT)");
      return;
    }
    const auto index = static_cast<std::size_t>(index_value);
    const PathMode mode = mode_value == 0 ? PathMode::LINEAR : PathMode::JOINT;
    waypoints_[index - 1].interpolation_mode = mode;
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_MODE_UPDATED: index=" + std::to_string(index) +
      ", mode=" + path_mode_name(mode));
  }

  void reset_joint_timing()
  {
    joint_timing_ = {0.0, 0.0, 0.0};
    std_msgs::msg::Float64MultiArray message;
    message.data = joint_timing_;
    joint_timing_publisher_->publish(message);
  }

  void joint_smoothing_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change continuous joint smoothing while executing");
      return;
    }
    joint_smoothing_enabled_ = message->data;
    invalidate_plan();
    publish_joint_smoothing();
    publish_status(
      std::string("JOINT_SMOOTHING_UPDATED: enabled=") +
      (joint_smoothing_enabled_ ? "true" : "false"));
  }

  void waypoint_gripper_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change waypoint gripper while executing");
      return;
    }
    if (message->data.size() != 2) {
      publish_error("waypoint gripper requires [one-based index, position]");
      return;
    }
    const double index_value = message->data[0];
    const double position = message->data[1];
    if (!std::isfinite(index_value) || index_value < 1.0 ||
      std::floor(index_value) != index_value)
    {
      publish_error("waypoint gripper index must be a positive integer");
      return;
    }
    const auto index = static_cast<std::size_t>(index_value);
    if (index > waypoints_.size()) {
      publish_error("waypoint gripper index is outside the current list");
      return;
    }
    if (!std::isfinite(position) || position < gripper_lower_limit_ ||
      position > gripper_upper_limit_)
    {
      publish_error("waypoint gripper position is outside the URDF limit");
      return;
    }
    waypoints_[index - 1].gripper_position = position;
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_GRIPPER_UPDATED: index=" + std::to_string(index) +
      ", position=" + std::to_string(position));
  }

  void start_waypoint_callback(const std_msgs::msg::UInt32::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change the start waypoint while executing");
      return;
    }
    const auto index = static_cast<std::size_t>(message->data);
    if (index < 1 || index > waypoints_.size()) {
      publish_error("start waypoint index is outside the current list");
      return;
    }
    start_waypoint_index_ = index;
    publish_waypoints();
    invalidate_plan();
    publish_status("START_WAYPOINT_SELECTED: index=" + std::to_string(index));
  }

  void end_waypoint_callback(const std_msgs::msg::UInt32::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot change the end waypoint while executing");
      return;
    }
    const auto index = static_cast<std::size_t>(message->data);
    if (index < 1 || index > waypoints_.size()) {
      publish_error("end waypoint index is outside the current list");
      return;
    }
    end_waypoint_index_ = index;
    publish_waypoints();
    invalidate_plan();
    publish_status("END_WAYPOINT_SELECTED: index=" + std::to_string(index));
  }

  void reorder_waypoint_callback(const std_msgs::msg::Int32MultiArray::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot reorder waypoints while executing");
      return;
    }
    if (message->data.size() != 2) {
      publish_error("waypoint reorder requires [from index, to index]");
      return;
    }
    const auto from_value = message->data[0];
    const auto to_value = message->data[1];
    if (from_value < 1 || to_value < 1 ||
      static_cast<std::size_t>(from_value) > waypoints_.size() ||
      static_cast<std::size_t>(to_value) > waypoints_.size())
    {
      publish_error("waypoint reorder index is outside the current list");
      return;
    }
    const auto from = static_cast<std::size_t>(from_value - 1);
    const auto to = static_cast<std::size_t>(to_value - 1);
    if (from == to) {
      publish_status(
        "WAYPOINT_REORDERED: from=" + std::to_string(from + 1) +
        ", to=" + std::to_string(to + 1));
      return;
    }

    const auto remap_index = [from, to](std::size_t index) {
        const std::size_t zero_based = index - 1;
        if (zero_based == from) {
          return to + 1;
        }
        if (from < zero_based && zero_based <= to) {
          return zero_based;
        }
        if (to <= zero_based && zero_based < from) {
          return zero_based + 2;
        }
        return index;
      };
    start_waypoint_index_ = remap_index(start_waypoint_index_);
    if (end_waypoint_index_ != 0) {
      end_waypoint_index_ = remap_index(end_waypoint_index_);
    }

    Waypoint moved = std::move(waypoints_[from]);
    reset_joint_timing();
    waypoints_.erase(waypoints_.begin() + static_cast<std::ptrdiff_t>(from));
    waypoints_.insert(
      waypoints_.begin() + static_cast<std::ptrdiff_t>(to), std::move(moved));
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_REORDERED: from=" + std::to_string(from + 1) +
      ", to=" + std::to_string(to + 1));
  }

  void remove_waypoint_callback(const std_msgs::msg::UInt32::SharedPtr message)
  {
    if (executing_) {
      publish_error("cannot remove a waypoint while executing");
      return;
    }
    const auto index = static_cast<std::size_t>(message->data);
    if (index < 1 || index > waypoints_.size()) {
      publish_error("waypoint removal index is outside the current list");
      return;
    }
    reset_joint_timing();
    waypoints_.erase(waypoints_.begin() + static_cast<std::ptrdiff_t>(index - 1));
    if (waypoints_.empty()) {
      start_waypoint_index_ = 1;
      end_waypoint_index_ = 0;
    } else if (index < start_waypoint_index_) {
      --start_waypoint_index_;
    } else if (start_waypoint_index_ > waypoints_.size()) {
      start_waypoint_index_ = waypoints_.size();
    }
    if (end_waypoint_index_ != 0) {
      if (index < end_waypoint_index_) {
        --end_waypoint_index_;
      } else if (end_waypoint_index_ > waypoints_.size()) {
        end_waypoint_index_ = waypoints_.size();
      }
    }
    publish_waypoints();
    invalidate_plan();
    publish_status(
      "WAYPOINT_REMOVED: index=" + std::to_string(index) +
      ", count=" + std::to_string(waypoints_.size()));
  }

  void external_command_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr /*message*/)
  {
    last_external_command_time_ = now();
    if (executing_) {
      executing_ = false;
      action_client_->async_cancel_all_goals();
      publish_error("external command detected; active waypoint goal cancelled");
    }
  }

  void capture_current_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (executing_) {
      response->success = false;
      response->message = "cannot capture while executing";
      return;
    }
    std::string error;
    const auto pose = current_pose(error);
    if (!pose) {
      response->success = false;
      response->message = error;
      return;
    }
    std::vector<double> joint_positions(current_positions_.rows());
    for (unsigned int index = 0; index < current_positions_.rows(); ++index) {
      joint_positions[index] = current_positions_(index);
    }
    waypoints_.push_back(
      Waypoint{
        *pose, std::move(joint_positions), current_gripper_position_, 0.0, path_mode_});
    publish_waypoints();
    invalidate_plan();
    response->success = true;
    response->message = "captured waypoint " + std::to_string(waypoints_.size());
    publish_status("WAYPOINT_CAPTURED: count=" + std::to_string(waypoints_.size()));
  }

  void remove_last_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (executing_ || waypoints_.empty()) {
      response->success = false;
      response->message = executing_ ? "cannot edit while executing" : "waypoint list is empty";
      return;
    }
    reset_joint_timing();
    waypoints_.pop_back();
    if (waypoints_.empty()) {
      start_waypoint_index_ = 1;
      end_waypoint_index_ = 0;
    } else if (start_waypoint_index_ > waypoints_.size()) {
      start_waypoint_index_ = waypoints_.size();
    }
    if (end_waypoint_index_ != 0 && end_waypoint_index_ > waypoints_.size()) {
      end_waypoint_index_ = waypoints_.size();
    }
    publish_waypoints();
    invalidate_plan();
    response->success = true;
    response->message = "remaining waypoints: " + std::to_string(waypoints_.size());
    publish_status("WAYPOINT_REMOVED: count=" + std::to_string(waypoints_.size()));
  }

  void clear_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (executing_) {
      response->success = false;
      response->message = "cannot clear while executing";
      return;
    }
    reset_joint_timing();
    waypoints_.clear();
    start_waypoint_index_ = 1;
    end_waypoint_index_ = 0;
    publish_waypoints();
    invalidate_plan();
    response->success = true;
    response->message = "waypoints cleared";
    publish_status("WAYPOINTS_CLEARED");
  }

  void select_linear_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    select_path_mode(PathMode::LINEAR, response);
  }

  void select_joint_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    select_path_mode(PathMode::JOINT, response);
  }

  void select_path_mode(
    PathMode mode, const std_srvs::srv::Trigger::Response::SharedPtr & response)
  {
    if (executing_) {
      response->success = false;
      response->message = "cannot change path mode while executing";
      return;
    }
    path_mode_ = mode;
    invalidate_plan();
    response->success = true;
    response->message = "selected default " + path_mode_name();
    publish_status("DEFAULT_PATH_MODE_SELECTED: " + path_mode_name());
  }

  void preview_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    Plan plan;
    std::string error;
    if (!make_plan(plan, error, false)) {
      invalidate_plan();
      response->success = false;
      response->message = error;
      publish_error("preview failed: " + error);
      return;
    }
    publish_plan(plan);
    response->success = true;
    response->message = plan_summary(plan);
    publish_status("PREVIEW_READY: " + response->message);
  }

  void preview_slow_start_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    Plan plan;
    std::string error;
    if (!make_plan(plan, error, true)) {
      invalidate_plan();
      response->success = false;
      response->message = error;
      publish_error("slow-start preview failed: " + error);
      return;
    }
    publish_plan(plan);
    response->success = true;
    response->message = "SLOW_START, " + plan_summary(plan);
    publish_status("SLOW_START_PREVIEW_READY: " + response->message);
  }

  void execute_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!publish_enabled_) {
      response->success = false;
      response->message = "publish_enabled=false; preview-only mode";
      return;
    }
    if (!direct_action_enabled_) {
      response->success = false;
      response->message =
        "direct action execution is disabled; use the UI WAYPOINT command source";
      return;
    }
    if (executing_) {
      response->success = false;
      response->message = "a waypoint goal is already active";
      return;
    }
    if (!joint_state_is_fresh()) {
      response->success = false;
      response->message = "joint state is stale or unavailable";
      return;
    }
    if (!arm_is_stationary()) {
      response->success = false;
      response->message = "arm is moving or joint velocity is unavailable";
      return;
    }
    if (external_command_is_recent()) {
      response->success = false;
      response->message = "external command topic is active";
      return;
    }
    if (!action_client_->wait_for_action_server(500ms)) {
      response->success = false;
      response->message = "FollowJointTrajectory action server is unavailable";
      return;
    }

    Plan plan;
    std::string error;
    if (!make_plan(plan, error, false)) {
      response->success = false;
      response->message = error;
      publish_error("execution plan failed: " + error);
      return;
    }
    publish_plan(plan);

    FollowJointTrajectory::Goal goal;
    goal.trajectory = plan.trajectory;
    goal.trajectory.joint_names.pop_back();
    for (auto & point : goal.trajectory.points) {
      point.positions.pop_back();
    }
    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          executing_ = false;
          publish_error("trajectory goal was rejected");
          return;
        }
        publish_status("EXECUTING");
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        executing_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL)
        {
          publish_status("EXECUTION_COMPLETE");
        } else {
          publish_error("trajectory execution did not complete successfully");
        }
      };

    executing_ = true;
    action_client_->async_send_goal(goal, options);
    response->success = true;
    response->message = "trajectory goal submitted: " + plan_summary(plan);
  }

  void stop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    action_client_->async_cancel_all_goals();
    executing_ = false;
    response->success = true;
    response->message = "cancel request submitted";
    publish_status("STOP_REQUESTED");
  }

  struct JointSplineKnot
  {
    std::size_t waypoint_index{0};
    std::vector<double> positions;
    double incoming_duration_s{0.0};
  };

  bool resolve_joint_target(
    const Waypoint & waypoint, const KDL::JntArray & seed,
    std::size_t waypoint_index, std::vector<double> & target,
    std::string & error) const
  {
    KDL::JntArray solution(chain_.getNrOfJoints());
    if (waypoint.joint_positions) {
      if (waypoint.joint_positions->size() != solution.rows()) {
        error = "stored joint target has the wrong size at waypoint " +
          std::to_string(waypoint_index + 1);
        return false;
      }
      for (unsigned int joint = 0; joint < solution.rows(); ++joint) {
        solution(joint) = waypoint.joint_positions->at(joint);
      }
    } else {
      const int result = solve_ik(seed, to_kdl(waypoint.pose), solution);
      if (result < 0) {
        error = "goal IK failed at waypoint " + std::to_string(waypoint_index + 1) +
          " (code " + std::to_string(result) + ")";
        return false;
      }
    }

    target.resize(solution.rows() + 1);
    for (unsigned int joint = 0; joint < solution.rows(); ++joint) {
      const double value = solution(joint);
      if (!std::isfinite(value) || value < lower_limits_(joint) - 1.0e-9 ||
        value > upper_limits_(joint) + 1.0e-9)
      {
        error = "non-finite or out-of-limit joint target for " + joint_names_[joint];
        return false;
      }
      target[joint] = value;
    }
    target.back() = waypoint.gripper_position;
    return true;
  }

  double joint_segment_duration_s(
    const std::vector<double> & start, const std::vector<double> & goal,
    double speed_scale) const
  {
    double maximum_delta = 0.0;
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
      maximum_delta = std::max(maximum_delta, std::abs(goal[joint] - start[joint]));
    }
    const double arm_duration =
      std::max(minimum_segment_duration_s_, maximum_delta / joint_speed_radps_) /
      speed_scale;
    const double gripper_duration =
      std::abs(goal.back() - start.back()) / (gripper_speed_radps_ * speed_scale);
    return std::max(arm_duration, gripper_duration);
  }

  bool append_joint_sample(
    Plan & plan, const std::vector<double> & values,
    std::vector<double> & previous, double sample_period_s,
    double & elapsed_s, std::string & error) const
  {
    if (values.size() != joint_names_.size() + 1 || previous.size() != values.size()) {
      error = "continuous joint sample has the wrong size";
      return false;
    }
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = values;
    KDL::JntArray arm_values(joint_names_.size());
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
      const double value = values[joint];
      if (!std::isfinite(value) || value < lower_limits_(joint) - 1.0e-9 ||
        value > upper_limits_(joint) + 1.0e-9)
      {
        error = "continuous joint sample is outside the limit for " + joint_names_[joint];
        return false;
      }
      const double delta = std::abs(value - previous[joint]);
      if (delta > max_joint_step_rad_ + 1.0e-9) {
        error = "continuous joint sample step exceeds the limit for " + joint_names_[joint] +
          ": " + std::to_string(delta) + " rad";
        return false;
      }
      arm_values(joint) = value;
    }
    const double gripper_value = values.back();
    if (!std::isfinite(gripper_value) ||
      gripper_value < gripper_lower_limit_ - 1.0e-9 ||
      gripper_value > gripper_upper_limit_ + 1.0e-9)
    {
      error = "continuous joint sample is outside the limit for " + gripper_joint_;
      return false;
    }
    const double gripper_delta = std::abs(gripper_value - previous.back());
    if (gripper_delta > max_joint_step_rad_ + 1.0e-9) {
      error = "continuous joint sample step exceeds the limit for " + gripper_joint_ +
        ": " + std::to_string(gripper_delta) + " rad";
      return false;
    }

    KDL::Frame frame;
    if (fk_solver_->JntToCart(arm_values, frame) < 0) {
      error = "forward kinematics failed while previewing continuous joint path";
      return false;
    }
    elapsed_s += sample_period_s;
    point.time_from_start = rclcpp::Duration::from_seconds(elapsed_s);
    plan.trajectory.points.push_back(std::move(point));
    append_path_pose(plan.path, from_kdl(frame));
    previous = values;
    return true;
  }

  void append_waypoint_target(
    Plan & plan, double elapsed_s, const std::vector<double> & positions) const
  {
    plan.waypoint_targets.data.push_back(elapsed_s);
    plan.waypoint_targets.data.insert(
      plan.waypoint_targets.data.end(), positions.begin(), positions.end());
  }

  bool append_joint_dwell(
    Plan & plan, const std::vector<double> & positions, double dwell_s,
    std::vector<double> & previous, double & elapsed_s, std::string & error) const
  {
    if (dwell_s <= 0.0) {
      return true;
    }
    const std::size_t sample_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(dwell_s * control_rate_hz_)));
    const double sample_period_s = dwell_s / static_cast<double>(sample_count);
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      if (!append_joint_sample(
          plan, positions, previous, sample_period_s, elapsed_s, error))
      {
        return false;
      }
    }
    return true;
  }


  bool append_retimed_joint_curve(
    Plan & plan, const std::vector<JointSplineKnot> & knots,
    const std::vector<std::vector<double>> & tangents,
    std::vector<double> & previous, double & elapsed, std::string & error,
    const std::vector<bool> & originals = {}) const
  {
    const std::size_t n = knots.size(), dims = previous.size();
    std::vector<double> source(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
      source[i] = source[i-1]+knots[i].incoming_duration_s;
    }
    // The original polynomial is evaluated unchanged. Only its clock changes.
    auto evaluate = [&](double s, std::vector<double> * qs = nullptr,
        std::vector<double> * qss = nullptr) {
        auto end = std::upper_bound(source.begin(), source.end(), s);
        std::size_t k = std::clamp<std::size_t>(end-source.begin(), 1, n-1);
        const double h = knots[k].incoming_duration_s;
        const double u = std::clamp((s-source[k-1])/h, 0.0, 1.0);
        std::vector<double> q(dims);
        if (qs) {qs->resize(dims);}
        if (qss) {qss->resize(dims);}
        for (std::size_t j = 0; j < dims; ++j) {
          const double c1 = tangents[k-1][j]*h;
          const double c2 = 3*(knots[k].positions[j]-knots[k-1].positions[j])-
            2*c1-tangents[k][j]*h;
          const double c3 = -2*(knots[k].positions[j]-knots[k-1].positions[j])+
            c1+tangents[k][j]*h;
          q[j] = knots[k-1].positions[j]+u*(c1+u*(c2+u*c3));
          if (qs) {(*qs)[j] = (c1+2*c2*u+3*c3*u*u)/h;}
          if (qss) {(*qss)[j] = (2*c2+6*c3*u)/(h*h);}
        }
        return q;
      };
    auto xyz = [&](double s) {
        const auto q = evaluate(s);
        KDL::JntArray arm(joint_names_.size());
        for (std::size_t j = 0; j < joint_names_.size(); ++j) {arm(j) = q[j];}
        KDL::Frame f;
        if (fk_solver_->JntToCart(arm, f) < 0) {throw std::runtime_error("timing FK failed");}
        return f.p;
      };
    struct Span {ClockNode a; ClockNode b; bool hold;};
    std::vector<Span> spans;
    const double origin = elapsed;
    double end_time = origin;
    auto original = [&](std::size_t k) {return originals.empty() || originals[k];};
    auto append_hold = [&](std::size_t k) {
        const double dwell = original(k) ? waypoints_[knots[k].waypoint_index].dwell_s : 0.0;
        if (dwell > 0) {
          spans.push_back({{source[k], 1, 1, 0, end_time},
              {source[k], 1, 1, 0, end_time+dwell}, true});
          end_time += dwell;
        }
      };
    auto retained = [&](std::size_t k) {
        const auto & a = knots[k-1].positions;
        const auto & b = knots[k].positions;
        double arm_delta = 0.0;
        for (std::size_t j = 0; j+1 < dims; ++j) {
          arm_delta = std::max(arm_delta, std::abs(b[j]-a[j]));
        }
        const std::size_t number = knots[k].waypoint_index+1;
        // Include the incoming guard segment so the insertion boundary tangent
        // and original timing remain unchanged on both sides of the join.
        return arm_delta < 1.e-10 ||
               (number >= joint_timing_[1] && number <= joint_timing_[2]);
      };
    try {
      for (std::size_t first = 0; first+1 < n;) {
        if (retained(first+1)) {
          const std::size_t last = first+1;
          const double h = source[last]-source[first];
          spans.push_back({{source[first], 1, 1, 1, end_time},
              {source[last], 1, 1, 1, end_time+h}, false});
          end_time += h;
          if (original(last)) {append_waypoint_target(plan, end_time, knots[last].positions);}
          append_hold(last);
          first = last;
          continue;
        }
        std::size_t last = first+1;
        while (last+1 < n && !retained(last+1) &&
          (!original(last) || waypoints_[knots[last].waypoint_index].dwell_s <= 0.0)) {++last;}
        std::vector<double> grid;
        const std::size_t count = std::max<std::size_t>(2,
          static_cast<std::size_t>(std::ceil((source[last]-source[first])/.001)));
        for (std::size_t i = 0; i <= count; ++i) {
          grid.push_back(source[first]+(source[last]-source[first])*i/count);
        }
        grid.insert(grid.end(), source.begin()+first, source.begin()+last+1);
        std::sort(grid.begin(), grid.end());
        grid.erase(std::unique(grid.begin(), grid.end(), [](double a, double b) {
              return std::abs(a-b) < 1.e-10;}), grid.end());
        std::vector<ClockNode> clock;
        for (double s : grid) {
          std::vector<double> qs, qss;
          evaluate(s, &qs, &qss);
          const double lo = std::max(source[first], s-.0001);
          const double hi = std::min(source[last], s+.0001);
          const KDL::Vector delta = xyz(hi)-xyz(lo);
          // FK differences can be sub-micrometre at low speed. KDL's default
          // Norm tolerance would round these to zero and stall the clock.
          const double ee_derivative = std::hypot(delta.x(), delta.y(), delta.z())/(hi-lo);
          double cap = joint_timing_[0]/std::max(ee_derivative, 1.e-12);
          double max_qs = 1.e-12, max_qss = 1.e-12;
          for (std::size_t j = 0; j < dims; ++j) {
            cap = std::min(cap, velocity_limits_[j]/std::max(std::abs(qs[j]), 1.e-12));
            max_qs = std::max(max_qs, std::abs(qs[j]));
            max_qss = std::max(max_qss, std::abs(qss[j]));
          }
          cap = std::min(cap, std::sqrt(1.8/max_qss));
          clock.push_back({s, cap, .8/max_qs, 0.0, 0.0,
            joint_timing_[0]/std::max(ee_derivative, 1.e-12)});
        }
        auto boundary_rate = [&](std::size_t k) {
            return std::any_of(tangents[k].begin(), tangents[k].end(), [](double v) {
                return std::abs(v) > 1.e-10;}) ? 1.0 : 0.0;
          };
        retime_clock(clock, boundary_rate(first), boundary_rate(last));
        smooth_clock_slowdowns(clock, 50);
        for (auto & node : clock) {node.t += end_time;}
        for (std::size_t i = 1; i < clock.size(); ++i) {
          spans.push_back({clock[i-1], clock[i], false});
        }
        // Target timestamps come from the same clock, without adding commands.
        for (std::size_t k = first+1; k <= last; ++k) {
          if (!original(k)) {continue;}
          auto it = std::lower_bound(clock.begin(), clock.end(), source[k],
            [](const ClockNode & a, double s) {return a.s < s;});
          if (it != clock.begin() &&
            (it == clock.end() || std::abs((it-1)->s-source[k]) < std::abs(it->s-source[k])))
          {
            --it;
          }
          if (it == clock.end() || std::abs(it->s-source[k]) > 1.e-8) {
            throw std::runtime_error("waypoint missing from integration grid");
          }
          append_waypoint_target(plan, it->t, knots[k].positions);
        }
        end_time = clock.back().t;
        append_hold(last);
        first = last;
      }
      std::size_t span = 0;
      const double period = 1.0/control_rate_hz_;
      const std::size_t count = static_cast<std::size_t>(std::ceil((end_time-origin)/period));
      if (count + plan.trajectory.points.size() > 250000) {
        throw std::runtime_error("retimed trajectory sample limit exceeded (250000)");
      }
      for (std::size_t i = 1; i <= count; ++i) {
        const double t = std::min(origin+i*period, end_time);
        while (span+1 < spans.size() && t > spans[span].b.t) {++span;}
        const auto & seg = spans[span];
        const double s = seg.hold ? seg.a.s : clock_parameter(seg.a, seg.b, t);
        if (!append_joint_sample(plan, evaluate(s), previous, t-elapsed, elapsed, error)) {
          return false;
        }
      }
    } catch (const std::exception & e) {
      error = std::string("joint clock: ")+e.what();
      return false;
    }
    return !plan.trajectory.points.empty();
  }

  bool make_continuous_joint_plan(
    Plan & plan, const std::vector<std::size_t> & waypoint_indices,
    const KDL::Frame & start_frame, std::string & error,
    bool slow_start) const
  {
    if (!std::all_of(
        waypoint_indices.begin(), waypoint_indices.end(), [this](std::size_t index) {
          return waypoints_[index].interpolation_mode == PathMode::JOINT;
        }))
    {
      error = "continuous joint smoothing requires every selected waypoint to use JOINT mode";
      return false;
    }

    plan.trajectory.joint_names = command_joint_names();
    plan.trajectory.header.frame_id = base_link_;
    plan.path.header.frame_id = base_link_;
    plan.path.header.stamp = now();
    append_path_pose(plan.path, from_kdl(start_frame));

    std::vector<double> live_positions;
    live_positions.reserve(joint_names_.size() + 1);
    for (unsigned int joint = 0; joint < current_positions_.rows(); ++joint) {
      live_positions.push_back(current_positions_(joint));
    }
    // Arm alignment begins at fresh measured joints, but the gripper command
    // begins at the first selected waypoint. A measured gripper can lag its
    // command under load; using it as the command-space seed would create an
    // unintended dip before returning to the first waypoint value.
    live_positions.push_back(
      waypoints_[waypoint_indices.front()].gripper_position);

    std::vector<JointSplineKnot> knots;
    knots.reserve(waypoint_indices.size());
    KDL::JntArray ik_seed = current_positions_;
    std::vector<double> previous_target = live_positions;
    for (std::size_t sequence_index = 0;
      sequence_index < waypoint_indices.size(); ++sequence_index)
    {
      const std::size_t waypoint_index = waypoint_indices[sequence_index];
      std::vector<double> target;
      if (!resolve_joint_target(
          waypoints_[waypoint_index], ik_seed, waypoint_index, target, error))
      {
        return false;
      }
      const double speed_scale = slow_start && sequence_index == 0 ?
        std::min(waypoints_[waypoint_index].speed_scale, slow_start_speed_scale_) :
        waypoints_[waypoint_index].speed_scale;
      const double duration = joint_segment_duration_s(
        previous_target, target, speed_scale);
      knots.push_back(JointSplineKnot{waypoint_index, target, duration});
      for (unsigned int joint = 0; joint < ik_seed.rows(); ++joint) {
        ik_seed(joint) = target[joint];
      }
      previous_target = target;
    }

    double elapsed_s = 0.0;
    std::vector<double> previous_sample = live_positions;
    const JointSplineKnot & first = knots.front();
    const std::size_t alignment_samples = std::max<std::size_t>(
      2, static_cast<std::size_t>(std::ceil(first.incoming_duration_s * control_rate_hz_)));
    const double alignment_period =
      first.incoming_duration_s / static_cast<double>(alignment_samples);
    // Start from fresh measured positions and settle at the first stored waypoint
    // before entering the pass-through spline. This avoids a command jump without
    // requiring the operator to align the robot to the first waypoint manually.
    for (std::size_t sample = 1; sample <= alignment_samples; ++sample) {
      const double progress = static_cast<double>(sample) /
        static_cast<double>(alignment_samples);
      std::vector<double> values(live_positions.size());
      for (std::size_t joint = 0; joint < live_positions.size(); ++joint) {
        values[joint] = interpolate_scalar(
          live_positions[joint], first.positions[joint], progress);
      }
      if (!append_joint_sample(
          plan, values, previous_sample, alignment_period, elapsed_s, error))
      {
        return false;
      }
    }
    append_waypoint_target(plan, elapsed_s, first.positions);
    if (!append_joint_dwell(
        plan, first.positions, waypoints_[first.waypoint_index].dwell_s,
        previous_sample, elapsed_s, error))
    {
      return false;
    }
    if (knots.size() == 1) {
      return true;
    }

    const std::size_t dimensions = live_positions.size();
    std::vector<std::vector<double>> tangents(
      knots.size(), std::vector<double>(dimensions, 0.0));
    for (std::size_t knot = 1; knot + 1 < knots.size(); ++knot) {
      if (waypoints_[knots[knot].waypoint_index].dwell_s > 0.0) {
        continue;
      }
      const double previous_duration = knots[knot].incoming_duration_s;
      const double next_duration = knots[knot + 1].incoming_duration_s;
      for (std::size_t joint = 0; joint < dimensions; ++joint) {
        const double previous_slope =
          (knots[knot].positions[joint] - knots[knot - 1].positions[joint]) /
          previous_duration;
        const double next_slope =
          (knots[knot + 1].positions[joint] - knots[knot].positions[joint]) /
          next_duration;
        tangents[knot][joint] = pchip_tangent(
          previous_slope, next_slope, previous_duration, next_duration);
      }
    }

    if (joint_timing_[0] > 0.0) {
      return append_retimed_joint_curve(
        plan, knots, tangents, previous_sample, elapsed_s, error);
    }

    for (std::size_t segment = 0; segment + 1 < knots.size(); ++segment) {
      const JointSplineKnot & start = knots[segment];
      const JointSplineKnot & goal = knots[segment + 1];
      const double duration = goal.incoming_duration_s;
      const std::size_t sample_count = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(duration * control_rate_hz_)));
      const double sample_period = duration / static_cast<double>(sample_count);
      for (std::size_t sample = 1; sample <= sample_count; ++sample) {
        const double progress = static_cast<double>(sample) /
          static_cast<double>(sample_count);
        std::vector<double> values(dimensions);
        for (std::size_t joint = 0; joint < dimensions; ++joint) {
          values[joint] = interpolate_cubic_hermite(
            start.positions[joint], goal.positions[joint],
            tangents[segment][joint], tangents[segment + 1][joint],
            duration, progress);
        }
        if (!append_joint_sample(
            plan, values, previous_sample, sample_period, elapsed_s, error))
        {
          return false;
        }
      }
      append_waypoint_target(plan, elapsed_s, goal.positions);
      if (!append_joint_dwell(
          plan, goal.positions, waypoints_[goal.waypoint_index].dwell_s,
          previous_sample, elapsed_s, error))
      {
        return false;
      }
    }
    return !plan.trajectory.points.empty();
  }

  void refinement_diagnostic(
    const std::string & stage, std::size_t attempt, std::size_t subdivisions,
    std::size_t segment, double position, double orientation, double joint,
    const std::string & reason, double effective_smoothing = -1,
    std::size_t knot_count = 0, const std::vector<std::size_t> & segment_counts = {}) const
  {
    std::string escaped;
    for (char c : reason) {
      if (c == '"' || c == '\\') {escaped += '\\';}
      escaped += (c == '\n' || c == '\r' || c == '\t') ? ' ' : c;
    }
    std::ostringstream out;
    const std::string code = (stage != "failed" && stage != "retry") ? "INFO" :
      reason.find("IK failed") != std::string::npos ? "IK_FAILURE" :
      reason.find("limit exceeded") != std::string::npos || reason.find("generation limit") != std::string::npos ? "RESOURCE_LIMIT" :
      reason.find("step exceeds") != std::string::npos ? "JOINT_STEP" :
      reason.find("limit") != std::string::npos ? "JOINT_LIMIT" :
      reason.find("tolerance exceeded") != std::string::npos ? "TOLERANCE_EXCEEDED" : "INFO";
    out << std::setprecision(12) << "{\"stage\":\"" << stage << "\",\"attempt\":" << attempt
        << ",\"subdivisions\":" << subdivisions << ",\"segment_end_waypoint\":" << segment
        << ",\"max_position_error_mm\":" << position * 1000
        << ",\"max_orientation_error_deg\":" << orientation * 180.0 / M_PI
        << ",\"max_joint_error_rad\":" << joint
        << ",\"smoothing_percent\":" << refinement_.smoothing * 100
        << ",\"effective_smoothing_percent\":" <<
      (effective_smoothing < 0 ? refinement_.smoothing : effective_smoothing) * 100
        << ",\"automatic_subdivisions\":" << (refinement_.automatic_subdivisions ? "true" : "false")
        << ",\"joint_subdivisions\":" <<
      (refinement_.automatic_subdivisions ? "null" : std::to_string(refinement_.subdivisions))
        << ",\"joint_intermediate_points\":" <<
      (refinement_.automatic_subdivisions ? "null" : std::to_string(refinement_.subdivisions - 1))
        << ",\"segment_subdivisions\":[";
    for (std::size_t i = 1; i < segment_counts.size(); ++i) {
      if (i > 1) {out << ',';}
      out << segment_counts[i];
    }
    out << ']'
        << ",\"knot_count\":" << knot_count
        << ",\"ik_searches\":" << ik_searches_
        << ",\"ik_relaxed_solutions\":" << ik_relaxed_solutions_
        << ",\"ik_min_margin_rad\":" << (std::isfinite(ik_min_margin_) ? ik_min_margin_ : -1.0)
        << ",\"position_tolerance_mm\":" << refinement_.position_tolerance * 1000
        << ",\"orientation_tolerance_deg\":" << refinement_.orientation_tolerance * 180.0 / M_PI
        << ",\"joint_tolerance_rad\":" << refinement_.joint_tolerance
        << ",\"reason_code\":\"" << code << "\""
        << ",\"reason\":\"" << escaped << "\"}";
    std_msgs::msg::String message;
    message.data = out.str();
    refinement_diagnostics_publisher_->publish(message);
    if (stage == "failed" || stage == "retry") {
      RCLCPP_WARN(get_logger(), "path refinement: %s", message.data.c_str());
    }
  }

  bool make_refined_joint_plan(
    Plan & plan, const std::vector<std::size_t> & indices,
    const KDL::Frame & start_frame, std::string & error, bool slow_start) const
  {
    struct Knot
    {
      std::vector<double> q;
      double duration{0};
      std::size_t source{0};  // Position in the selected original waypoint range.
      double progress{0};
      bool original{false};
    };
    const bool has_cartesian_segment = std::any_of(
      indices.begin() + 1, indices.end(), [this](std::size_t index) {
        return waypoints_[index].interpolation_mode == PathMode::LINEAR;
      });
    std::size_t subdivisions = refinement_.automatic_subdivisions ? 1 : refinement_.subdivisions;
    for (std::size_t attempt = 0; attempt <= refinement_.max_refinements; ++attempt) {
      ik_searches_ = 0;
      ik_relaxed_solutions_ = 0;
      ik_min_margin_ = std::numeric_limits<double>::infinity();
      const double smoothing = refinement_.smoothing / std::pow(2.0, attempt);
      double max_position = 0, max_orientation = 0, max_joint = 0;
      std::size_t failing_segment = indices.front() + 1;
      refinement_diagnostic("generating", attempt + 1, subdivisions, 0, 0, 0, 0,
        "building joint knots", smoothing);
      error.clear();
      try {
        std::vector<std::vector<double>> anchors;
        KDL::JntArray seed = current_positions_;
        for (auto index : indices) {
          failing_segment = index + 1;
          std::vector<double> target;
          if (!resolve_joint_target(waypoints_[index], seed, index, target, error)) {
            throw std::runtime_error(error);
          }
          anchors.push_back(target);
          for (unsigned int j = 0; j < seed.rows(); ++j) {seed(j) = target[j];}
        }
        const auto fk = [this](const std::vector<double> & q) {
            KDL::JntArray joints(joint_names_.size());
            for (unsigned int j = 0; j < joints.rows(); ++j) {joints(j) = q[j];}
            KDL::Frame frame;
            if (fk_solver_->JntToCart(joints, frame) < 0) {
              throw std::runtime_error("FK failed");
            }
            return from_kdl(frame);
          };
        std::vector<std::size_t> segment_counts(indices.size(), 0);
        std::size_t expected_knots = 1;
        for (std::size_t source = 1; source < indices.size(); ++source) {
          const bool cartesian = waypoints_[indices[source]].interpolation_mode == PathMode::LINEAR;
          const std::size_t base_count = refinement_.automatic_subdivisions ?
            automatic_segment_subdivisions(
            fk(anchors[source - 1]), fk(anchors[source]), anchors[source - 1], anchors[source]) :
            refinement_.subdivisions;
          const std::size_t count = cartesian ?
            (refinement_.automatic_subdivisions ? base_count * subdivisions : subdivisions) : base_count;
          if (count > 4096 || expected_knots + count > 50000) {
            throw std::runtime_error("generation limit: at most 4096 subdivisions / 50000 knots");
          }
          segment_counts[source] = count;
          expected_knots += count;
        }
        // Each original segment retains its Cartesian or joint-space reference.
        const auto reference = [&](std::size_t source, double t) {
            const auto & goal = waypoints_[indices[source]];
            if (goal.interpolation_mode == PathMode::LINEAR) {
              return reference_pose(waypoints_[indices[source - 1]].pose, goal.pose, t);
            }
            auto q = anchors[source - 1];
            for (std::size_t j = 0; j < q.size(); ++j) {
              q[j] += t * (anchors[source][j] - q[j]);
            }
            return fk(q);
          };
        std::vector<Knot> knots{{anchors.front(), 0, 0, 0, true}};
        for (std::size_t source = 1; source < indices.size(); ++source) {
          failing_segment = indices[source] + 1;
          const auto & goal = waypoints_[indices[source]];
          const std::size_t segment_count = segment_counts[source];
          double duration = joint_segment_duration_s(
            anchors[source - 1], anchors[source], goal.speed_scale);
          if (goal.interpolation_mode == PathMode::LINEAR) {
            duration = std::max(duration, segment_duration(
                waypoints_[indices[source - 1]].pose, goal.pose,
                linear_speed_mps_, angular_speed_radps_, minimum_segment_duration_s_) /
              goal.speed_scale);
          }
          const std::size_t first_knot = knots.size();
          for (std::size_t k = 1; k <= segment_count; ++k) {
            const double t = static_cast<double>(k) / segment_count;
            auto q = anchors[source - 1];
            for (std::size_t j = 0; j < q.size(); ++j) {
              q[j] += t * (anchors[source][j] - q[j]);
            }
            if (goal.interpolation_mode == PathMode::LINEAR && k < segment_count) {
              for (unsigned int j = 0; j < seed.rows(); ++j) {seed(j) = knots.back().q[j];}
              KDL::JntArray solution(seed.rows());
              const int code = solve_ik(seed, to_kdl(reference(source, t)), solution);
              if (code < 0) {
                throw std::runtime_error("IK failed at fraction " + std::to_string(t) +
                        " (KDL code " + std::to_string(code) + ")");
              }
              for (unsigned int j = 0; j < solution.rows(); ++j) {q[j] = solution(j);}
            }
            // All generated points in a source segment have the same time
            // spacing. One shared duration scales linearly with speed_scale.
            for (std::size_t j = 0; j < q.size(); ++j) {
              const double speed = j < joint_names_.size() ? joint_speed_radps_ : gripper_speed_radps_;
              duration = std::max(duration, segment_count * 1.875 *
                std::abs(q[j] - knots.back().q[j]) / (speed * goal.speed_scale));
            }
            knots.push_back({q, 0, source, t, k == segment_count});
          }
          for (std::size_t k = first_knot; k < knots.size(); ++k) {
            knots[k].duration = duration / segment_count;
          }
        }
        std::vector<std::vector<double>> tangent(knots.size(), std::vector<double>(anchors[0].size(), 0));
        for (std::size_t k = 1; k + 1 < knots.size(); ++k) {
          if (knots[k].original && waypoints_[indices[knots[k].source]].dwell_s > 0) {continue;}
          for (std::size_t j = 0; j < tangent[k].size(); ++j) {
            const double before = (knots[k].q[j] - knots[k - 1].q[j]) / knots[k].duration;
            const double after = (knots[k + 1].q[j] - knots[k].q[j]) / knots[k + 1].duration;
            tangent[k][j] = (knots[k].original ? smoothing : 1.0) * pchip_tangent(
              before, after, knots[k].duration, knots[k + 1].duration);
          }
        }
        Plan candidate;
        candidate.trajectory.joint_names = command_joint_names();
        candidate.trajectory.header.frame_id = base_link_;
        candidate.path.header.frame_id = base_link_;
        candidate.path.header.stamp = now();
        append_path_pose(candidate.path, from_kdl(start_frame));
        std::vector<double> previous(anchors[0].size());
        for (unsigned int j = 0; j < current_positions_.rows(); ++j) {previous[j] = current_positions_(j);}
        previous.back() = anchors[0].back();
        const auto live = previous;
        failing_segment = indices.front() + 1;
        const double alignment = joint_segment_duration_s(live, anchors[0], slow_start ?
          std::min(waypoints_[indices[0]].speed_scale, slow_start_speed_scale_) :
          waypoints_[indices[0]].speed_scale);
        double elapsed = 0;
        const std::size_t alignment_count = std::max<std::size_t>(2, std::ceil(alignment * control_rate_hz_));
        if (alignment_count > 250000) {throw std::runtime_error("alignment sample limit exceeded");}
        for (std::size_t s = 1; s <= alignment_count; ++s) {
          auto q = live;
          for (std::size_t j = 0; j < q.size(); ++j) {
            q[j] = interpolate_scalar(live[j], anchors[0][j], static_cast<double>(s) / alignment_count);
          }
          if (!append_joint_sample(candidate, q, previous, alignment / alignment_count, elapsed, error)) {
            throw std::runtime_error("initial alignment: " + error);
          }
        }
        append_waypoint_target(candidate, elapsed, anchors[0]);
        if (!append_joint_dwell(candidate, anchors[0], waypoints_[indices[0]].dwell_s, previous, elapsed, error)) {
          throw std::runtime_error(error);
        }
        // Retiming starts after the same measured-pose alignment and first
        // dwell. Keep the validated refined polynomial and its exact tangents.
        const Plan timing_prefix = joint_timing_[0] > 0.0 ? candidate : Plan();
        const auto timing_previous = previous;
        const double timing_elapsed = elapsed;
        double curve_time = elapsed;
        double next_command_time = elapsed + 1.0 / control_rate_hz_;
        bool within_tolerance = true;
        refinement_diagnostic("validating", attempt + 1, subdivisions, 0, 0, 0, 0,
          "checking FK and joint limits", smoothing, knots.size());
        for (std::size_t k = 1; k < knots.size(); ++k) {
          const auto & a = knots[k - 1];
          const auto & b = knots[k];
          const std::size_t source = b.source;
          const double start_t = a.source == source ? a.progress : 0.0;
          const std::size_t count = std::max<std::size_t>(2, std::ceil(b.duration * control_rate_hz_));
          if (candidate.trajectory.points.size() + count > 250000) {
            throw std::runtime_error("trajectory sample limit exceeded (250000)");
          }
          // Dense knots refine geometry without increasing the command publish rate.
          // Check the curve at >=4x command density and at every actual command time.
          std::vector<std::pair<double, bool>> checks;
          for (std::size_t s = 0; s <= count * 4; ++s) {
            checks.emplace_back(static_cast<double>(s) / (count * 4), false);
          }
          const double piece_end = curve_time + b.duration;
          while (next_command_time <= piece_end + 1.e-10) {
            checks.emplace_back(std::clamp((next_command_time - curve_time) / b.duration, 0.0, 1.0), true);
            next_command_time += 1.0 / control_rate_hz_;
          }
          if (b.original) {checks.emplace_back(1.0, true);}
          std::sort(checks.begin(), checks.end());
          std::vector<std::pair<double, bool>> merged;
          for (const auto & check : checks) {
            if (!merged.empty() && std::abs(merged.back().first - check.first) < 1.e-10) {
              merged.back().second = merged.back().second || check.second;
            } else {merged.push_back(check);}
          }
          for (const auto & check : merged) {
            const double u = check.first;
            const double t = start_t + u * (b.progress - start_t);
            auto q = a.q;
            for (std::size_t j = 0; j < q.size(); ++j) {
              q[j] = interpolate_cubic_hermite(a.q[j], b.q[j], tangent[k - 1][j], tangent[k][j], b.duration, u);
              const double lo = j < joint_names_.size() ? lower_limits_(j) : gripper_lower_limit_;
              const double hi = j < joint_names_.size() ? upper_limits_(j) : gripper_upper_limit_;
              if (!std::isfinite(q[j]) || q[j] < lo - 1e-9 || q[j] > hi + 1e-9) {
                failing_segment = indices[source] + 1;
                throw std::runtime_error("joint limit: " + command_joint_names()[j] +
                        " value=" + std::to_string(q[j]) + " allowed=[" + std::to_string(lo) +
                        ", " + std::to_string(hi) + "], fraction " + std::to_string(t));
              }
            }
            const auto actual = fk(q);
            const bool joint_segment = waypoints_[indices[source]].interpolation_mode == PathMode::JOINT;
            const double reference_t = joint_segment ?
              joint_path_progress(anchors[source - 1], anchors[source], q) : t;
            const auto expected = reference(source, reference_t);
            const double pe = translation_distance(actual, expected);
            const double oe = angular_distance(actual.orientation, expected.orientation);
            double je = 0;
            if (joint_segment) {
              for (std::size_t j = 0; j < q.size(); ++j) {
                je = std::max(je, std::abs(q[j] - (anchors[source - 1][j] +
                    reference_t * (anchors[source][j] - anchors[source - 1][j]))));
              }
            }
            max_position = std::max(max_position, pe);
            max_orientation = std::max(max_orientation, oe);
            max_joint = std::max(max_joint, je);
            if (pe > refinement_.position_tolerance || oe > refinement_.orientation_tolerance ||
              je > refinement_.joint_tolerance)
            {
              within_tolerance = false;
              failing_segment = indices[source] + 1;
              error = "tolerance exceeded at fraction " + std::to_string(t) +
                "; increase subdivisions/retries or reduce smoothing strength; "
                "an endpoint residual may require correcting the original waypoint/IK target";
            }
            const double command_dt = curve_time + u * b.duration - elapsed;
            if (check.second && command_dt > 1.e-10 && !append_joint_sample(
                candidate, q, previous, command_dt, elapsed, error))
            {
              failing_segment = indices[source] + 1;
              throw std::runtime_error(error + "; max_joint_step_rad=" + std::to_string(max_joint_step_rad_));
            }
          }
          curve_time = piece_end;
          if (b.original) {
            append_waypoint_target(candidate, elapsed, b.q);
            if (!append_joint_dwell(candidate, b.q, waypoints_[indices[source]].dwell_s, previous, elapsed, error)) {
              throw std::runtime_error(error);
            }
            curve_time = elapsed;
            next_command_time = elapsed + 1.0 / control_rate_hz_;
          }
        }
        if (within_tolerance) {
          if (joint_timing_[0] > 0.0 && knots.size() > 1) {
            std::vector<JointSplineKnot> clock_knots;
            std::vector<bool> original_knots;
            for (const auto & knot : knots) {
              clock_knots.push_back({indices[knot.source], knot.q, knot.duration});
              original_knots.push_back(knot.original);
            }
            Plan timed = timing_prefix;
            auto timed_previous = timing_previous;
            double timed_elapsed = timing_elapsed;
            if (!append_retimed_joint_curve(timed, clock_knots, tangent,
                timed_previous, timed_elapsed, error, original_knots))
            {
              // A clock constraint must not silently reduce geometric
              // smoothing to find a different path during the speed search.
              return false;
            }
            candidate = std::move(timed);
          }
          plan = std::move(candidate);
          refinement_diagnostic("success", attempt + 1, subdivisions, 0,
            max_position, max_orientation, max_joint,
            "verified at 4x command density; original waypoints and dwell preserved; initial alignment excluded",
            smoothing, knots.size(), segment_counts);
          return true;
        }
      } catch (const std::exception & e) {error = e.what();}
      const bool last = attempt == refinement_.max_refinements ||
        (has_cartesian_segment && subdivisions >= 4096);
      refinement_diagnostic(last ? "failed" : "retry", attempt + 1, subdivisions,
        failing_segment, max_position, max_orientation, max_joint, error, smoothing);
      if (last) {
        error = "refined JOINT path failed at waypoint " + std::to_string(failing_segment) + ": " + error;
        return false;
      }
      if (has_cartesian_segment) {subdivisions *= 2;}
    }
    return false;
  }

  bool make_plan(Plan & plan, std::string & error, bool slow_start) const
  {
    if (!chain_ready_ || !joint_state_ready_) {
      error = "robot model or complete joint state is unavailable";
      return false;
    }
    if (waypoints_.empty()) {
      error = "at least one waypoint is required";
      return false;
    }
    if (start_waypoint_index_ < 1 || start_waypoint_index_ > waypoints_.size()) {
      error = "selected start waypoint is outside the current list";
      return false;
    }
    const std::size_t end_waypoint_index = selected_end_waypoint_index();
    if (end_waypoint_index < 1 || end_waypoint_index > waypoints_.size()) {
      error = "selected end waypoint is outside the current list";
      return false;
    }
    const auto waypoint_indices = selected_waypoint_indices();
    if (waypoint_indices.empty()) {
      error = "selected waypoint range is empty";
      return false;
    }

    KDL::Frame start_frame;
    if (fk_solver_->JntToCart(current_positions_, start_frame) < 0) {
      error = "forward kinematics failed";
      return false;
    }

    const bool all_selected_waypoints_are_joint = std::all_of(
      waypoint_indices.begin(), waypoint_indices.end(), [this](std::size_t index) {
        return waypoints_[index].interpolation_mode == PathMode::JOINT;
      });
    if (refinement_.enabled) {
      return make_refined_joint_plan(plan, waypoint_indices, start_frame, error, slow_start);
    }
    if (joint_smoothing_enabled_ && all_selected_waypoints_are_joint) {
      return make_continuous_joint_plan(
        plan, waypoint_indices, start_frame, error, slow_start);
    }

    plan.trajectory.joint_names = command_joint_names();
    plan.trajectory.header.frame_id = base_link_;
    plan.path.header.frame_id = base_link_;
    plan.path.header.stamp = now();
    append_path_pose(plan.path, from_kdl(start_frame));

    KDL::JntArray seed = current_positions_;
    // Keep the gripper at the first selected waypoint command while the arm
    // aligns to that waypoint. Later waypoint segments still interpolate
    // their stored gripper commands normally.
    double gripper_seed = waypoints_[waypoint_indices.front()].gripper_position;
    CartesianPose segment_start = from_kdl(start_frame);
    double elapsed_s = 0.0;

    for (std::size_t sequence_index = 0;
      sequence_index < waypoint_indices.size(); ++sequence_index)
    {
      const std::size_t waypoint_index = waypoint_indices[sequence_index];
      const Waypoint & waypoint = waypoints_[waypoint_index];
      const CartesianPose & segment_goal = waypoint.pose;
      const KDL::JntArray segment_seed = seed;
      const double gripper_segment_seed = gripper_seed;
      const double speed_scale = slow_start && sequence_index == 0 ?
        std::min(waypoint.speed_scale, slow_start_speed_scale_) :
        waypoint.speed_scale;
      double duration_s = minimum_segment_duration_s_;
      KDL::JntArray joint_goal(chain_.getNrOfJoints());
      if (waypoint.interpolation_mode == PathMode::LINEAR) {
        try {
          duration_s = segment_duration(
            segment_start, segment_goal, linear_speed_mps_, angular_speed_radps_,
            minimum_segment_duration_s_) / speed_scale;
        } catch (const std::exception & exception) {
          error = exception.what();
          return false;
        }
      } else {
        if (waypoint.joint_positions) {
          for (unsigned int joint = 0; joint < joint_goal.rows(); ++joint) {
            joint_goal(joint) = waypoint.joint_positions->at(joint);
          }
        } else {
          const int result = solve_ik(seed, to_kdl(segment_goal), joint_goal);
          if (result < 0) {
            error = "goal IK failed at waypoint " + std::to_string(waypoint_index + 1) +
              " (code " + std::to_string(result) + ")";
            return false;
          }
        }
        double maximum_delta = 0.0;
        for (unsigned int joint = 0; joint < joint_goal.rows(); ++joint) {
          maximum_delta = std::max(
            maximum_delta, std::abs(joint_goal(joint) - segment_seed(joint)));
        }
        duration_s =
          std::max(minimum_segment_duration_s_, maximum_delta / joint_speed_radps_) /
          speed_scale;
      }
      duration_s = std::max(
        duration_s,
        std::abs(waypoint.gripper_position - gripper_segment_seed) /
        (gripper_speed_radps_ * speed_scale));
      const std::size_t sample_count = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(duration_s * control_rate_hz_)));
      const double sample_period_s = duration_s / static_cast<double>(sample_count);

      for (std::size_t sample = 1; sample <= sample_count; ++sample) {
        const double progress = static_cast<double>(sample) / static_cast<double>(sample_count);
        KDL::JntArray solution(chain_.getNrOfJoints());
        CartesianPose target_pose;
        if (waypoint.interpolation_mode == PathMode::LINEAR) {
          target_pose = interpolate_pose(segment_start, segment_goal, progress);
          const int result = solve_ik(seed, to_kdl(target_pose), solution);
          if (result < 0) {
            error = "IK failed at waypoint " + std::to_string(waypoint_index + 1) +
              ", sample " + std::to_string(sample) + " (code " +
              std::to_string(result) + ")";
            return false;
          }
        } else {
          for (unsigned int joint = 0; joint < solution.rows(); ++joint) {
            solution(joint) = interpolate_scalar(
              segment_seed(joint), joint_goal(joint), progress);
          }
          KDL::Frame interpolated_frame;
          if (fk_solver_->JntToCart(solution, interpolated_frame) < 0) {
            error = "forward kinematics failed while previewing joint path";
            return false;
          }
          target_pose = from_kdl(interpolated_frame);
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(solution.rows() + 1);
        for (unsigned int joint = 0; joint < solution.rows(); ++joint) {
          const double value = solution(joint);
          const double delta = std::abs(value - seed(joint));
          if (!std::isfinite(value) || value < lower_limits_(joint) - 1.0e-9 ||
            value > upper_limits_(joint) + 1.0e-9)
          {
            error = "non-finite or out-of-limit IK result for " + joint_names_[joint];
            return false;
          }
          if (delta > max_joint_step_rad_) {
            error = "joint discontinuity for " + joint_names_[joint] + ": " +
              std::to_string(delta) + " rad";
            return false;
          }
          point.positions[joint] = value;
        }
        const double gripper_value = interpolate_scalar(
          gripper_segment_seed, waypoint.gripper_position, progress);
        const double gripper_delta = std::abs(gripper_value - gripper_seed);
        if (!std::isfinite(gripper_value) ||
          gripper_value < gripper_lower_limit_ - 1.0e-9 ||
          gripper_value > gripper_upper_limit_ + 1.0e-9)
        {
          error = "non-finite or out-of-limit gripper result for " + gripper_joint_;
          return false;
        }
        if (gripper_delta > max_joint_step_rad_) {
          error = "joint discontinuity for " + gripper_joint_ + ": " +
            std::to_string(gripper_delta) + " rad";
          return false;
        }
        point.positions[solution.rows()] = gripper_value;
        elapsed_s += sample_period_s;
        point.time_from_start = rclcpp::Duration::from_seconds(elapsed_s);
        plan.trajectory.points.push_back(std::move(point));
        append_path_pose(plan.path, target_pose);
        seed = solution;
        gripper_seed = gripper_value;
      }
      segment_start = segment_goal;
      plan.waypoint_targets.data.push_back(elapsed_s);
      for (unsigned int joint = 0; joint < seed.rows(); ++joint) {
        plan.waypoint_targets.data.push_back(seed(joint));
      }
      plan.waypoint_targets.data.push_back(gripper_seed);

      if (waypoint.dwell_s > 0.0) {
        const std::size_t dwell_sample_count = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(waypoint.dwell_s * control_rate_hz_)));
        const double dwell_sample_period_s =
          waypoint.dwell_s / static_cast<double>(dwell_sample_count);
        for (std::size_t sample = 0; sample < dwell_sample_count; ++sample) {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.resize(seed.rows() + 1);
          for (unsigned int joint = 0; joint < seed.rows(); ++joint) {
            point.positions[joint] = seed(joint);
          }
          point.positions[seed.rows()] = gripper_seed;
          elapsed_s += dwell_sample_period_s;
          point.time_from_start = rclcpp::Duration::from_seconds(elapsed_s);
          plan.trajectory.points.push_back(std::move(point));
          append_path_pose(plan.path, segment_goal);
        }
      }
    }
    return !plan.trajectory.points.empty();
  }

  void publish_plan(const Plan & plan)
  {
    preview_path_publisher_->publish(plan.path);
    planned_trajectory_publisher_->publish(plan.trajectory);
    planned_waypoint_targets_publisher_->publish(plan.waypoint_targets);
  }

  std::string plan_summary(const Plan & plan) const
  {
    const auto & last = plan.trajectory.points.back().time_from_start;
    const double duration = static_cast<double>(last.sec) + 1.0e-9 * last.nanosec;
    std::ostringstream stream;
    stream << plan_mode_name() << ", "
           << selected_waypoint_indices().size()
           << " waypoint(s), start waypoint " << start_waypoint_index_
           << ", end waypoint " << selected_end_waypoint_index() << ", "
           << plan.trajectory.points.size()
           << " samples, " << duration << " s";
    return stream.str();
  }

  bool joint_state_is_fresh() const
  {
    return joint_state_ready_ && (now() - last_joint_state_time_).seconds() <= 0.25;
  }

  bool arm_is_stationary() const
  {
    if (current_velocities_.size() != joint_names_.size()) {
      return false;
    }
    return std::all_of(
      current_velocities_.begin(), current_velocities_.end(), [this](double velocity) {
        return std::isfinite(velocity) && std::abs(velocity) <= max_start_velocity_radps_;
      });
  }

  bool external_command_is_recent() const
  {
    return last_external_command_time_.has_value() &&
           (now() - *last_external_command_time_).seconds() <= command_quiet_s_;
  }

  static CartesianPose from_kdl(const KDL::Frame & frame)
  {
    CartesianPose pose;
    pose.position = {frame.p.x(), frame.p.y(), frame.p.z()};
    frame.M.GetQuaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    pose.orientation = normalize(pose.orientation);
    return pose;
  }

  static KDL::Frame to_kdl(const CartesianPose & pose)
  {
    const Quaternion orientation = normalize(pose.orientation);
    return KDL::Frame(
      KDL::Rotation::Quaternion(
        orientation.x, orientation.y, orientation.z, orientation.w),
      KDL::Vector(pose.position[0], pose.position[1], pose.position[2]));
  }

  void append_path_pose(nav_msgs::msg::Path & path, const CartesianPose & pose) const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header = path.header;
    message.pose.position.x = pose.position[0];
    message.pose.position.y = pose.position[1];
    message.pose.position.z = pose.position[2];
    message.pose.orientation.x = pose.orientation.x;
    message.pose.orientation.y = pose.orientation.y;
    message.pose.orientation.z = pose.orientation.z;
    message.pose.orientation.w = pose.orientation.w;
    path.poses.push_back(std::move(message));
  }

  std::string base_link_;
  std::string world_frame_;
  std::string world_reference_link_;
  std::string robot_root_link_;
  std::string tip_link_;
  std::string gripper_joint_;
  std::string joint_states_topic_;
  std::string clicked_point_topic_;
  std::string action_topic_;
  std::string external_command_topic_;
  bool publish_enabled_{false};
  bool direct_action_enabled_{false};
  bool joint_smoothing_enabled_{true};
  PathRefinement refinement_;
  mutable std::size_t ik_searches_{0};
  mutable std::size_t ik_relaxed_solutions_{0};
  mutable double ik_min_margin_{std::numeric_limits<double>::infinity()};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr refinement_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr refinement_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr refinement_diagnostics_publisher_;
  std::vector<double> joint_timing_{0.0, 0.0, 0.0};
  std::vector<double> velocity_limits_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_timing_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_timing_publisher_;
  PathMode path_mode_{PathMode::LINEAR};
  std::size_t start_waypoint_index_{1};
  std::size_t end_waypoint_index_{0};
  double control_rate_hz_{100.0};
  double display_rate_hz_{20.0};
  double linear_speed_mps_{0.02};
  double angular_speed_radps_{0.20};
  double joint_speed_radps_{0.20};
  double gripper_speed_radps_{0.20};
  double slow_start_speed_scale_{1.0};
  double minimum_segment_duration_s_{0.50};
  double max_waypoint_dwell_s_{30.0};
  double max_waypoint_speed_scale_{3.0};
  double max_uniform_tcp_speed_mps_{1.8};
  double max_joint_step_rad_{0.05};
  double max_cartesian_jog_step_m_{0.0025};
  double max_tool_path_correction_m_{0.05};
  double max_start_velocity_radps_{0.02};
  double command_quiet_s_{0.50};
  double ik_epsilon_{1.0e-5};
  int ik_max_iterations_{200};

  bool chain_ready_{false};
  bool joint_state_ready_{false};
  bool executing_{false};
  std::string robot_description_;
  KDL::Chain chain_;
  KDL::Chain world_to_base_chain_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> world_to_base_joint_names_;
  KDL::JntArray lower_limits_;
  KDL::JntArray upper_limits_;
  KDL::JntArray current_positions_;
  double current_gripper_position_{0.0};
  double gripper_lower_limit_{0.0};
  double gripper_upper_limit_{0.0};
  KDL::JntArray world_to_base_positions_;
  std::vector<double> current_velocities_;
  std::vector<Waypoint> waypoints_;
  std::optional<KDL::Frame> last_world_waypoints_frame_;
  std::optional<std::chrono::steady_clock::time_point> last_world_pose_publish_time_;
  std::chrono::steady_clock::time_point last_world_waypoints_publish_time_{};
  int32_t relative_reference_index_{-1};
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> last_external_command_time_;
  bool world_chain_ready_{false};
  bool world_state_ready_{false};

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> world_to_base_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> velocity_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    absolute_position_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr
    absolute_orientation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relative_waypoint_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    frame_relative_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr relative_reference_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    cartesian_jog_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    tool_path_correction_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_dwell_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_speed_scale_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    equal_segment_time_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    equal_total_time_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    uniform_total_time_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    waypoint_gripper_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
  waypoint_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joint_smoothing_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr start_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr end_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
    reorder_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr remove_waypoint_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    external_command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_world_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr world_waypoints_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_root_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_dwells_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_speed_scales_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_grippers_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr waypoint_modes_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joint_smoothing_enabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  waypoint_joint_values_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr selected_start_waypoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr selected_end_waypoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    planned_waypoint_targets_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    planned_trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    cartesian_jog_target_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr select_linear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr select_joint_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_slow_start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

}  // namespace waypoint_telep

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<waypoint_telep::WaypointTelepNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_telep"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
