// Copyright 2021 ros2_control development team
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

#include "joint_trajectory_command_broadcaster/joint_trajectory_command_broadcaster.hpp"
#include "joint_trajectory_command_broadcaster/leader_action_latch.hpp"

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>
#include <iterator>

#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

struct JointTrajectoryCommandBroadcaster::GroupPoseKinematics
{
  struct ChainKinematics
  {
    explicit ChainKinematics(const KDL::Chain & configured_chain)
    : chain(configured_chain),
      fk_solver(std::make_unique<KDL::ChainFkSolverPos_recursive>(chain)),
      jacobian_solver(std::make_unique<KDL::ChainJntToJacSolver>(chain))
    {
    }

    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
    std::vector<size_t> group_joint_indices;
  };

  GroupPoseKinematics(
    const KDL::Chain & leader_chain, const KDL::Chain & follower_chain)
  : leader(leader_chain), follower(follower_chain)
  {
  }

  ChainKinematics leader;
  ChainKinematics follower;
  RelativePoseMapper relative_mapper;
};

namespace
{
template<typename KinematicsT>
bool compute_pose(
  KinematicsT & kinematics,
  const std::vector<double> & group_position,
  Eigen::Isometry3d & pose,
  KDL::JntArray * joint_array_out = nullptr)
{
  if (kinematics.group_joint_indices.size() != kinematics.chain.getNrOfJoints()) {
    return false;
  }
  KDL::JntArray joints(kinematics.chain.getNrOfJoints());
  for (size_t i = 0; i < kinematics.group_joint_indices.size(); ++i) {
    const size_t group_index = kinematics.group_joint_indices[i];
    if (group_index >= group_position.size() || !std::isfinite(group_position[group_index])) {
      return false;
    }
    joints(static_cast<unsigned int>(i)) = group_position[group_index];
  }

  KDL::Frame frame;
  if (kinematics.fk_solver->JntToCart(joints, frame) < 0) {
    return false;
  }
  pose.setIdentity();
  for (size_t row = 0; row < 3; ++row) {
    pose.translation()(static_cast<Eigen::Index>(row)) = frame.p[static_cast<unsigned int>(row)];
    for (size_t col = 0; col < 3; ++col) {
      pose.linear()(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
        frame.M(static_cast<unsigned int>(row), static_cast<unsigned int>(col));
    }
  }
  if (joint_array_out != nullptr) {
    *joint_array_out = joints;
  }
  return pose.matrix().allFinite();
}
}  // namespace

JointTrajectoryCommandBroadcaster::JointTrajectoryCommandBroadcaster() {}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryCommandBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointTrajectoryCommandBroadcaster::
state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.left_joints) {
    state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
  }
  for (const auto & joint : params_.right_joints) {
    state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  const auto reference_mode = parse_teleop_reference_mode(params_.teleop_reference_mode);
  if (!reference_mode) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid teleop_reference_mode '%s'; expected 'absolute' or 'relative'",
      params_.teleop_reference_mode.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  teleop_reference_mode_ = *reference_mode;
  if (teleop_reference_mode_ == TeleopReferenceMode::RELATIVE &&
    (params_.relative_kp_position <= 0.0 || params_.relative_kp_orientation <= 0.0 ||
    params_.relative_weight_position <= 0.0 || params_.relative_weight_orientation <= 0.0 ||
    params_.relative_damping_weight <= 0.0))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Relative-pose gains, weights, and damping must all be positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(params_.gripper_clutch_deadband) ||
    !std::isfinite(params_.gripper_clutch_scale) ||
    params_.gripper_clutch_deadband < 0.0 || params_.gripper_clutch_scale <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "gripper_clutch_deadband must be non-negative and gripper_clutch_scale must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "Teleop reference mode: %s",
    teleop_reference_mode_ == TeleopReferenceMode::RELATIVE ? "relative" : "absolute");

  // Map interface if needed
  map_interface_to_joint_state_.clear();
  map_interface_to_joint_state_[HW_IF_POSITION] = params_.map_interface_to_joint_state.position;

  try {
    // Create publishers for left and right groups
    std::vector<std::string> groups = {"left", "right"};

    for (const auto & group_name : groups) {
      // Get joints for this group
      std::vector<std::string> group_joints;
      if (group_name == "left" && !params_.left_joints.empty()) {
        group_joints = params_.left_joints;
      } else if (group_name == "right" && !params_.right_joints.empty()) {
        group_joints = params_.right_joints;
      }

      if (group_joints.empty()) {
        continue;  // Skip empty groups
      }

      group_joint_names_[group_name] = group_joints;

      // Get offsets for this group
      if (group_name == "left" && !params_.left_offsets.empty()) {
        group_joint_offsets_[group_name] = params_.left_offsets;
      } else if (group_name == "right" && !params_.right_offsets.empty()) {
        group_joint_offsets_[group_name] = params_.right_offsets;
      } else {
        // Initialize empty offsets if not provided
        group_joint_offsets_[group_name] = std::vector<double>();
      }

      // Get reverse joints for this group
      if (group_name == "left" && !params_.left_reverse_joints.empty()) {
        group_reverse_joints_[group_name] = params_.left_reverse_joints;
      } else if (group_name == "right" && !params_.right_reverse_joints.empty()) {
        group_reverse_joints_[group_name] = params_.right_reverse_joints;
      } else {
        // Initialize empty reverse joints if not provided
        group_reverse_joints_[group_name] = std::vector<std::string>();
      }


      // Create topic name with group-specific namespace
      std::string topic_name;
      topic_name = "joint_trajectory_command_broadcaster_" + group_name + "/joint_trajectory";
      group_topic_names_[group_name] = topic_name;

      // Create publisher for this group
      joint_trajectory_publishers_[group_name] =
        get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        topic_name, rclcpp::SystemDefaultsQoS());

      realtime_joint_trajectory_publishers_[group_name] =
        std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
        joint_trajectory_publishers_[group_name]);

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Created joint trajectory publisher for group '%s' on topic: %s with %zu joints",
        group_name.c_str(), topic_name.c_str(), group_joints.size());

      // Create joint state publisher for this group with timestamp from update() function
      std::string joint_state_topic_name;
      if (group_name == "left" && !params_.left_joint_states_stamped_topic.empty()) {
        joint_state_topic_name = params_.left_joint_states_stamped_topic;
      } else if (group_name == "right" && !params_.right_joint_states_stamped_topic.empty()) {
        joint_state_topic_name = params_.right_joint_states_stamped_topic;
      } else {
        joint_state_topic_name =
          "joint_trajectory_command_broadcaster_" + group_name + "/joint_states_stamped";
      }
      joint_state_stamped_publishers_[group_name] =
        get_node()->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_name, rclcpp::SystemDefaultsQoS());

      realtime_joint_state_stamped_publishers_[group_name] =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        joint_state_stamped_publishers_[group_name]);

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Created joint state stamped publisher for group '%s' on topic: %s",
        group_name.c_str(), joint_state_topic_name.c_str());
    }

    // Store the groups for later use
    trajectory_groups_ = groups;

    // Initialize group runtimes with mode from parameters
    for (const auto & group_name : trajectory_groups_) {
      bool init_enabled = (group_name == "left") ?
        params_.left_enabled_init : params_.right_enabled_init;
      group_runtime_[group_name].mode = init_enabled ? Mode::TELEOP : Mode::IDLE;
      group_enable_command_buffers_[group_name] =
        std::make_shared<realtime_tools::RealtimeBuffer<EnableCommand>>(EnableCommand{});
      group_last_enable_command_sequence_[group_name] = 0;
      group_follower_state_buffers_[group_name] =
        std::make_shared<realtime_tools::RealtimeBuffer<std::vector<double>>>(
        std::vector<double>{});
      group_executed_action_buffers_[group_name] =
        std::make_shared<realtime_tools::RealtimeBuffer<std::vector<double>>>(
        std::vector<double>{});
    }
    RCLCPP_INFO(get_node()->get_logger(),
      "Initial mode: left=%s, right=%s",
      params_.left_enabled_init ? "TELEOP" : "IDLE",
      params_.right_enabled_init ? "TELEOP" : "IDLE");

    // Load save poses per group from parameters (left_save_pose_<id> / right_save_pose_<id>)
    for (int64_t id : params_.save_pose_ids) {
      for (const auto & group_name : trajectory_groups_) {
        std::string pname = group_name + "_save_pose_" + std::to_string(id);
        if (!get_node()->has_parameter(pname)) {
          get_node()->declare_parameter(pname, std::vector<double>{});
        }
        auto values = get_node()->get_parameter(pname).as_double_array();
        if (!values.empty()) {
          group_save_poses_[group_name][static_cast<uint8_t>(id)] = values;
          RCLCPP_INFO(get_node()->get_logger(),
            "Loaded %s (%zu values)", pname.c_str(), values.size());
        }
      }
    }

    // Follower feedback establishes the initial output and remains available
    // to refresh the idle target after another command source moves the robot.
    for (const auto & group_name : trajectory_groups_) {
      group_last_target_initialized_[group_name] = false;
    }

    auto make_follower_cb = [this](const std::string & group_name) {
        return [this, group_name](sensor_msgs::msg::JointState::SharedPtr msg) {
                 const auto & joints = group_joint_names_[group_name];
                 if (joints.empty()) {
                   return;
                 }
                 std::vector<double> positions(joints.size(), kUninitializedValue);
                 std::vector<std::string> missing_joints;
                 const size_t n = std::min(msg->name.size(), msg->position.size());
                 for (size_t i = 0; i < joints.size(); ++i) {
                   for (size_t j = 0; j < n; ++j) {
                     if (msg->name[j] == joints[i]) {
                       positions[i] = msg->position[j];
                       break;
                     }
                   }
                   if (!std::isfinite(positions[i])) {
                     missing_joints.push_back(joints[i]);
                   }
                 }
                 if (!missing_joints.empty()) {
                   RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "[%s] waiting for follower joint_states on topic with required joints; "
            "missing %zu/%zu joints, first missing: %s",
            group_name.c_str(), missing_joints.size(), joints.size(),
              missing_joints.front().c_str());
                   return;
                 }
                 auto buffer_it = group_follower_state_buffers_.find(group_name);
                 if (buffer_it == group_follower_state_buffers_.end() || !buffer_it->second) {
                   RCLCPP_ERROR(
                     get_node()->get_logger(), "[%s] follower state buffer is unavailable",
                     group_name.c_str());
                   return;
                 }
                 buffer_it->second->writeFromNonRT(positions);
               };
      };

    left_follower_js_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.left_follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      make_follower_cb("left"));
    right_follower_js_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.right_follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      make_follower_cb("right"));

    // The measured gripper position can deflect under contact. Preserve the
    // command that actually reached the follower so an IDLE rebase does not
    // convert that deflection into a zero-error (and therefore weak) hold.
    auto make_executed_action_cb = [this](const std::string & group_name) {
        return [this, group_name](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                 const auto & joints = group_joint_names_[group_name];
                 if (msg->joint_names != joints || msg->points.size() != 1 ||
                   msg->points.front().positions.size() != joints.size())
                 {
                   RCLCPP_WARN_THROTTLE(
                     get_node()->get_logger(), *get_node()->get_clock(), 2000,
                     "[%s] Ignoring malformed final executed action",
                     group_name.c_str());
                   return;
                 }
                 std::vector<double> positions;
                 if (!try_store_finite_action(msg->points.front().positions, positions)) {
                   RCLCPP_WARN_THROTTLE(
                     get_node()->get_logger(), *get_node()->get_clock(), 2000,
                     "[%s] Ignoring non-finite final executed action",
                     group_name.c_str());
                   return;
                 }
                 auto buffer_it = group_executed_action_buffers_.find(group_name);
                 if (buffer_it == group_executed_action_buffers_.end() || !buffer_it->second) {
                   RCLCPP_ERROR(
                     get_node()->get_logger(), "[%s] executed action buffer is unavailable",
                     group_name.c_str());
                   return;
                 }
                 buffer_it->second->writeFromNonRT(positions);
               };
      };

    left_executed_action_sub_ =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      params_.left_executed_action_topic, rclcpp::QoS(1).reliable().transient_local(),
      make_executed_action_cb("left"));
    right_executed_action_sub_ =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      params_.right_executed_action_topic, rclcpp::QoS(1).reliable().transient_local(),
      make_executed_action_cb("right"));

    // Enable topic subscriptions
    //   0=disable, 1=enable, 2=toggle, 3+=disable + trigger save pose <N>
    left_enable_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
      "/leader/left_command", rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("left", msg->data);
      });

    right_enable_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
      "/leader/right_command", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("right", msg->data);
      });

    // Block until follower joint_states init last_target for every group
    while (rclcpp::ok()) {
      bool all_init = true;
      for (const auto & g : trajectory_groups_) {
        if (!group_last_target_initialized_[g] && sync_follower_target(g)) {
          group_last_target_initialized_[g] = true;
          RCLCPP_INFO(
            get_node()->get_logger(),
            "[%s] last_target initialized from follower joint_states using joint-name order",
            g.c_str());
        }
        if (!group_last_target_initialized_[g]) {
          all_init = false;
        }
      }
      if (all_init) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Waiting for follower joint_states to initialize last_target...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_node()->get_logger(), "Controller configured successfully.");
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();
  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description. Will proceed without URDF-based filtering.");
  }

  // Subscribe to follower's robot_description once to extract joint limits for TELEOP clamping
  bool follower_urdf_loaded = false;
  auto urdf_sub = get_node()->create_subscription<std_msgs::msg::String>(
    params_.follower_robot_description_topic,
    rclcpp::QoS(1).transient_local().reliable(),
    [this, &follower_urdf_loaded](std_msgs::msg::String::SharedPtr msg) {
      if (follower_urdf_loaded) {
        return;
      }
      urdf::Model follower_model;
      if (!follower_model.initString(msg->data)) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "Failed to parse follower robot_description");
        return;
      }
      for (const auto & group_name : trajectory_groups_) {
        const auto & joints = group_joint_names_[group_name];
        std::vector<double> lowers, uppers, velocities;
        for (const auto & jn : joints) {
          auto j = follower_model.getJoint(jn);
          if (j && j->limits) {
            lowers.push_back(j->limits->lower);
            uppers.push_back(j->limits->upper);
            velocities.push_back(j->limits->velocity);
          } else {
            lowers.push_back(-std::numeric_limits<double>::infinity());
            uppers.push_back(std::numeric_limits<double>::infinity());
            velocities.push_back(std::numeric_limits<double>::infinity());
            RCLCPP_WARN(get_node()->get_logger(),
              "[%s] No limit for joint '%s' in follower URDF",
              group_name.c_str(), jn.c_str());
          }
        }
        group_lower_limits_[group_name] = lowers;
        group_upper_limits_[group_name] = uppers;
        group_velocity_limits_[group_name] = velocities;
        RCLCPP_INFO(get_node()->get_logger(),
          "[%s] Loaded follower joint limits (%zu joints)",
          group_name.c_str(), lowers.size());
      }
      if (teleop_reference_mode_ == TeleopReferenceMode::RELATIVE &&
        !configure_relative_pose_kinematics(model_, follower_model))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Failed to configure relative-pose kinematics");
      }
      follower_urdf_loaded = true;
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "Waiting for follower robot_description on '%s'...",
    params_.follower_robot_description_topic.c_str());
  while (rclcpp::ok() && !follower_urdf_loaded) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Still waiting for follower robot_description...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!rclcpp::ok()) {
    return CallbackReturn::ERROR;
  }
  if (teleop_reference_mode_ == TeleopReferenceMode::RELATIVE &&
    group_pose_kinematics_.size() != group_joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Relative-pose kinematics are incomplete (%zu/%zu groups)",
      group_pose_kinematics_.size(), group_joint_names_.size());
    return CallbackReturn::ERROR;
  }
  // urdf_sub goes out of scope here, effectively unsubscribing

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  // Check offsets for each group
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];
    const size_t num_joints = group_joints.size();

    if (group_joint_offsets_[group_name].empty()) {
      // If no offsets provided, use zeros
      group_joint_offsets_[group_name].assign(num_joints, 0.0);
    } else if (group_joint_offsets_[group_name].size() != num_joints) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The number of provided offsets (%zu) for group '%s' does not match the number of "
        "joints (%zu).",
        group_joint_offsets_[group_name].size(), group_name.c_str(), num_joints);
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Group '%s' configured with %zu joints and %zu offsets",
      group_name.c_str(), num_joints, group_joint_offsets_[group_name].size());
  }

  // Pre-allocate joint state message size for each group (real-time safety)
  // Must be done after init_joint_data() so group_joint_names_ is populated
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];

    auto it = realtime_joint_state_stamped_publishers_.find(group_name);
    if (it != realtime_joint_state_stamped_publishers_.end() && it->second) {
      auto & msg = it->second->msg_;
      const size_t num_joints = group_joints.size();
      msg.name.resize(num_joints);
      msg.position.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
      msg.velocity.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
      msg.effort.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

      // Set joint names once (done in on_activate, not in update loop)
      for (size_t i = 0; i < num_joints; ++i) {
        msg.name[i] = group_joints[i];
      }

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Pre-allocated joint state message for group '%s' with %zu joints",
        group_name.c_str(), num_joints);
    }
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();
  group_joint_names_.clear();
  group_joint_offsets_.clear();
  group_topic_names_.clear();
  group_reverse_joints_.clear();
  group_lower_limits_.clear();
  group_upper_limits_.clear();
  group_velocity_limits_.clear();
  group_pose_kinematics_.clear();
  group_last_target_.clear();
  group_follower_state_buffers_.clear();
  group_executed_action_buffers_.clear();
  group_runtime_.clear();

  return CallbackReturn::SUCCESS;
}

template<typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  for (const auto & key_item : map) {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend()) {
      return true;
    }
  }
  return false;
}

bool JointTrajectoryCommandBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty()) {
    return false;
  }

  // Initialize mapping
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0) {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // Filter out joints without position interface (since we want positions)
  for (const auto & name_ifv : name_if_value_mapping_) {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION})) {
      if (
        !params_.use_urdf_to_filter || !is_model_loaded_ ||
        model_.getJoint(name_ifv.first))
      {
        joint_names_.push_back(name_ifv.first);
      }
    }
  }

  return true;
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Update stored values
  for (const auto & state_interface : state_interfaces_) {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    auto value = state_interface.get_optional();
    if (value) {
      name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = *value;
    }
  }

  // Publish JointTrajectory messages for each group with current positions
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];

    // Safely get group offsets and reverse joints
    std::vector<double> group_offsets;
    std::vector<std::string> group_reverse_joints;

    auto offsets_it = group_joint_offsets_.find(group_name);
    if (offsets_it != group_joint_offsets_.end()) {
      group_offsets = offsets_it->second;
    }

    auto reverse_it = group_reverse_joints_.find(group_name);
    if (reverse_it != group_reverse_joints_.end()) {
      group_reverse_joints = reverse_it->second;
    }

    const size_t num_joints = group_joints.size();
    auto & last_target = group_last_target_[group_name];
    auto & rt = group_runtime_[group_name];

    auto command_buffer_it = group_enable_command_buffers_.find(group_name);
    if (command_buffer_it != group_enable_command_buffers_.end() && command_buffer_it->second) {
      const auto * command = command_buffer_it->second->readFromRT();
      if (command && command->sequence != 0 &&
        command->sequence != group_last_enable_command_sequence_[group_name])
      {
        group_last_enable_command_sequence_[group_name] = command->sequence;
        apply_enable_msg(group_name, command->data);
      }
    }

    // Update last_target based on mode
    switch (rt.mode) {
      case Mode::IDLE: {
        // WAYPOINT/POLICY can move the physical follower while the leader is
        // idle. Keep the pending TELEOP sample at the measured pose so route
        // selection cannot replay the pre-handoff command.
          if (!sync_follower_target(group_name)) {
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 2000,
              "[%s] No valid follower feedback; holding the last finite action",
              group_name.c_str());
          }
          break;
        }

      case Mode::TELEOP: {
          const auto & lowers = group_lower_limits_[group_name];
          const auto & uppers = group_upper_limits_[group_name];
          const bool clamp_enabled =
            lowers.size() == num_joints && uppers.size() == num_joints;

          std::vector<double> leader_reference(num_joints, kUninitializedValue);
          for (size_t i = 0; i < num_joints; ++i) {
            double leader_val =
              get_value(name_if_value_mapping_, group_joints[i], HW_IF_POSITION);
            if (!std::isfinite(leader_val)) {
              break;
            }
            if (std::find(group_reverse_joints.begin(), group_reverse_joints.end(),
              group_joints[i]) != group_reverse_joints.end())
            {
              leader_val = -leader_val;
            }
            if (i < group_offsets.size()) {
              leader_val += group_offsets[i];
            }
            leader_reference[i] = leader_val;
          }

          if (!std::all_of(
              leader_reference.begin(), leader_reference.end(), [](const double value) {
                return std::isfinite(value);
              }))
          {
            RCLCPP_ERROR_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 2000,
              "[%s] Invalid leader state; holding the last finite action",
              group_name.c_str());
            break;
          }

          if (!rt.gripper_clutch.anchored) {
            if (last_target.size() != num_joints ||
              !std::all_of(last_target.begin(), last_target.end(), [](const double value) {
                return std::isfinite(value);
              }))
            {
              RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "[%s] Cannot capture gripper clutch without a finite held action",
                group_name.c_str());
              break;
            }
            rt.gripper_clutch.leader_anchor = leader_reference;
            rt.gripper_clutch.command_anchor = last_target;
            rt.gripper_clutch.anchored = true;
          }

          std::vector<double> candidate(num_joints, kUninitializedValue);
          if (teleop_reference_mode_ == TeleopReferenceMode::RELATIVE) {
            if (!update_relative_pose_target(
                group_name, leader_reference, period.seconds(), candidate))
            {
              RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "[%s] Relative-pose FK/IK failed; holding last action",
                group_name.c_str());
              break;
            }
          } else {
            double blend_alpha = 1.0;
            if (rt.blend.active) {
              const double elapsed = (get_node()->now() - rt.blend.start_time).seconds();
              const double t =
                std::clamp(elapsed / params_.leader_blend_duration, 0.0, 1.0);
              blend_alpha = 3.0 * t * t - 2.0 * t * t * t; // cubic smoothstep
              if (t >= 1.0) {
                rt.blend.active = false;
              }
            }
            for (size_t i = 0; i < num_joints; ++i) {
              if (is_gripper_joint(group_joints[i])) {
                if (!try_map_gripper_clutch_target(
                    leader_reference[i], rt.gripper_clutch.leader_anchor[i],
                    rt.gripper_clutch.command_anchor[i], params_.gripper_clutch_deadband,
                    params_.gripper_clutch_scale, candidate[i]))
                {
                  candidate[i] = kUninitializedValue;
                  continue;
                }
                if (clamp_enabled) {
                  candidate[i] = std::clamp(candidate[i], lowers[i], uppers[i]);
                }
                continue;
              }
              double leader_val = leader_reference[i];
              if (rt.blend.active && i < rt.blend.start_pos.size()) {
                leader_val = (1.0 - blend_alpha) * rt.blend.start_pos[i] +
                  blend_alpha * leader_val;
              }
              if (clamp_enabled) {
                leader_val = std::clamp(leader_val, lowers[i], uppers[i]);
              }
              candidate[i] = leader_val;
            }
          }

          if (!try_store_finite_action(candidate, last_target)) {
            RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "[%s] Invalid leader state; holding the last finite action",
            group_name.c_str());
          }
          break;
        }

      case Mode::SAVE_POSE: {
        // Cubic interp to target; hold at target after completion
          if (rt.interp.active) {
            double elapsed = (get_node()->now() - rt.interp.start_time).seconds();
            double t = std::clamp(elapsed / rt.interp.duration_sec, 0.0, 1.0);
            double s = 3.0 * t * t - 2.0 * t * t * t;
            if (t >= 1.0) {
              rt.interp.active = false;
            }

            last_target.resize(num_joints);
            for (size_t i = 0; i < num_joints; ++i) {
              if (i < rt.interp.start_pos.size() && i < rt.interp.target_pos.size()) {
                last_target[i] = rt.interp.start_pos[i] +
                  s * (rt.interp.target_pos[i] - rt.interp.start_pos[i]);
              }
            }
          }
        // else: hold last_target (which should equal target after completion)
          break;
        }
    }

    // last_target is valid only after first enable/interp
    const bool valid = last_target.size() == num_joints &&
      std::all_of(last_target.begin(), last_target.end(), [](double value) {
          return std::isfinite(value);
      });

    // Publish trajectory (always when last_target valid)
    auto & realtime_publisher = realtime_joint_trajectory_publishers_[group_name];
    if (valid && realtime_publisher) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = rclcpp::Time(0, 0);
      traj_msg.joint_names = group_joints;
      traj_msg.points.resize(1);
      traj_msg.points[0].positions = last_target;
      traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);
      realtime_publisher->try_publish(traj_msg);
    }

    // Publish joint state for this group with timestamp from update() function
    // Real-time safe: message size pre-allocated in on_activate(), only copy values here
    // Apply same transformations (reverse, offsets) as joint_trajectory
    auto joint_state_pub_it = realtime_joint_state_stamped_publishers_.find(group_name);
    if (joint_state_pub_it != realtime_joint_state_stamped_publishers_.end() &&
      joint_state_pub_it->second)
    {
      auto & realtime_joint_state_publisher = joint_state_pub_it->second;
      if (realtime_joint_state_publisher->trylock()) {
        auto & msg = realtime_joint_state_publisher->msg_;

        // Set timestamp from update() function argument (actual sensor read time)
        msg.header.stamp = time;
        msg.header.frame_id = "";

        // Copy values from name_if_value_mapping_ to pre-allocated message
        // Apply same reverse and offset transformations as joint_trajectory
        // No string operations, no dynamic allocation - only value copying
        for (size_t i = 0; i < group_joints.size() && i < msg.name.size(); ++i) {
          const std::string & joint_name = group_joints[i];
          double pos_value = (i < last_target.size()) ? last_target[i] :
            std::numeric_limits<double>::quiet_NaN();

          msg.position[i] = std::isnan(pos_value) ? std::numeric_limits<double>::quiet_NaN() :
            pos_value;

          // Get velocity if available (no reverse/offset for velocity)
          if (name_if_value_mapping_.count(joint_name) > 0) {
            const auto & interfaces = name_if_value_mapping_.at(joint_name);
            auto vel_it = interfaces.find(HW_IF_VELOCITY);
            if (vel_it != interfaces.end() && !std::isnan(vel_it->second)) {
              // Apply reverse sign to velocity if joint is reversed
              double vel_value = vel_it->second;
              if (
                std::find(
                  group_reverse_joints.begin(),
                  group_reverse_joints.end(),
                  joint_name) != group_reverse_joints.end())
              {
                vel_value = -vel_value;
              }
              msg.velocity[i] = vel_value;
            } else {
              msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            }

            // Get effort if available (no reverse/offset for effort)
            auto eff_it = interfaces.find(HW_IF_EFFORT);
            if (eff_it != interfaces.end() && !std::isnan(eff_it->second)) {
              msg.effort[i] = eff_it->second;
            } else {
              msg.effort[i] = std::numeric_limits<double>::quiet_NaN();
            }
          } else {
            msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            msg.effort[i] = std::numeric_limits<double>::quiet_NaN();
          }
        }

        realtime_joint_state_publisher->unlockAndPublish();
      }
    }
  }

  return controller_interface::return_type::OK;
}

void JointTrajectoryCommandBroadcaster::handle_enable_msg(
  const std::string & group_name, uint8_t data)
{
  auto buffer_it = group_enable_command_buffers_.find(group_name);
  if (buffer_it == group_enable_command_buffers_.end() || !buffer_it->second) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "[%s] Cannot queue enable command: unknown group",
      group_name.c_str());
    return;
  }

  EnableCommand command;
  command.sequence = next_enable_command_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
  command.data = data;
  buffer_it->second->writeFromNonRT(command);
}

void JointTrajectoryCommandBroadcaster::apply_enable_msg(
  const std::string & group_name, uint8_t data)
{
  auto & rt = group_runtime_[group_name];

  switch (data) {
    case 0: // stop
      rt.toggle_gate.reset();
      rt.mode = Mode::IDLE;
      rt.blend.active = false;
      rt.interp.active = false;
      rt.active_save_pose_id = 0;
      // Route setup sends command=0 before TELEOP is enabled. Rebase the
      // complete pending action here, including the gripper, so an executed
      // action left by an earlier session cannot close the gripper at startup.
      sync_follower_target(group_name, false);
      reset_relative_pose_anchor(group_name);
      reset_gripper_clutch(group_name);
      RCLCPP_INFO(
        get_node()->get_logger(), "[%s] Teleop stopped; synchronizing with follower feedback",
        group_name.c_str());
      return;

    case 1: // teleop
      rt.toggle_gate.reset();
      if (rt.mode != Mode::TELEOP) {
        // Relative TELEOP starts from the newest measured arm pose, but its
        // gripper anchor must remain the most recently executed command. A
        // loaded gripper's measured deflection is not the commanded grasp.
        const bool preserve_commanded_grippers =
          teleop_reference_mode_ == TeleopReferenceMode::RELATIVE;
        sync_follower_target(group_name, preserve_commanded_grippers);
        prepare_teleop_entry(group_name);
      }
      rt.mode = Mode::TELEOP;
      rt.interp.active = false;
      rt.active_save_pose_id = 0;
      RCLCPP_INFO(get_node()->get_logger(), "[%s] Teleop enabled", group_name.c_str());
      return;

    case 2:  // toggle
      if (!rt.toggle_gate.accept(
          get_node()->now().seconds(), params_.toggle_debounce_duration_sec))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "[%s] Ignoring duplicate toggle inside %.3fs debounce window",
          group_name.c_str(), params_.toggle_debounce_duration_sec);
        return;
      }
      if (rt.mode == Mode::TELEOP) {
        rt.mode = Mode::IDLE;
        rt.blend.active = false;
        rt.active_save_pose_id = 0;
        sync_follower_target(group_name, false);
        reset_relative_pose_anchor(group_name);
        reset_gripper_clutch(group_name);
        RCLCPP_INFO(
          get_node()->get_logger(),
          "[%s] Teleop toggled off; synchronizing with follower feedback",
          group_name.c_str());
      } else {
        const bool preserve_commanded_grippers =
          teleop_reference_mode_ == TeleopReferenceMode::RELATIVE;
        sync_follower_target(group_name, preserve_commanded_grippers);
        prepare_teleop_entry(group_name);
        rt.mode = Mode::TELEOP;
        rt.interp.active = false;
        rt.active_save_pose_id = 0;
        RCLCPP_INFO(get_node()->get_logger(), "[%s] Teleop toggled on", group_name.c_str());
      }
      return;

    default: {  // 3, 4, ... : save pose N
        rt.toggle_gate.reset();
        if (rt.mode == Mode::SAVE_POSE && rt.active_save_pose_id == data) {
          RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "[%s] Ignoring repeated save-pose trigger id=%u",
          group_name.c_str(), data);
          return;
        }
        auto gp_it = group_save_poses_.find(group_name);
        if (gp_it == group_save_poses_.end()) {
          return;
        }
        auto pose_it = gp_it->second.find(data);
        if (pose_it == gp_it->second.end()) {
          RCLCPP_WARN(get_node()->get_logger(),
          "[%s] No save pose defined for id=%u", group_name.c_str(), data);
          return;
        }
        if (!start_save_pose_interp(group_name, pose_it->second)) {
          return;
        }
        rt.mode = Mode::SAVE_POSE;
        rt.blend.active = false;
        rt.active_save_pose_id = data;
        reset_relative_pose_anchor(group_name);
        reset_gripper_clutch(group_name);
        RCLCPP_INFO(get_node()->get_logger(),
        "[%s] Start interpolation to save pose %u", group_name.c_str(), data);
        return;
      }
  }
}

bool JointTrajectoryCommandBroadcaster::configure_relative_pose_kinematics(
  const urdf::Model & leader_model, const urdf::Model & follower_model)
{
  group_pose_kinematics_.clear();
  KDL::Tree leader_tree;
  if (!kdl_parser::treeFromUrdfModel(leader_model, leader_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build KDL tree from leader URDF");
    return false;
  }
  KDL::Tree follower_tree;
  if (!kdl_parser::treeFromUrdfModel(follower_model, follower_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build KDL tree from follower URDF");
    return false;
  }

  for (const auto & group_item : group_joint_names_) {
    const std::string & group_name = group_item.first;
    const std::vector<std::string> & group_joints = group_item.second;
    const std::string end_effector = group_name == "left" ?
      params_.left_end_effector_link : params_.right_end_effector_link;
    KDL::Chain leader_chain;
    KDL::Chain follower_chain;
    std::vector<size_t> leader_indices;
    std::vector<size_t> follower_indices;
    const auto build_chain =
      [this, &group_name, &group_joints, &end_effector](
      const KDL::Tree & tree, const char * role, KDL::Chain & chain,
      std::vector<size_t> & group_joint_indices) -> bool
      {
        if (!tree.getChain(params_.relative_base_link, end_effector, chain)) {
          RCLCPP_ERROR(
            get_node()->get_logger(), "[%s] Cannot build %s kinematic chain %s -> %s",
            group_name.c_str(), role, params_.relative_base_link.c_str(), end_effector.c_str());
          return false;
        }
        for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments();
          ++segment_index)
        {
          const KDL::Joint & joint = chain.getSegment(segment_index).getJoint();
          if (joint.getType() == KDL::Joint::None) {
            continue;
          }
          const auto group_joint =
            std::find(group_joints.begin(), group_joints.end(), joint.getName());
          if (group_joint == group_joints.end()) {
            RCLCPP_ERROR(
              get_node()->get_logger(), "[%s] %s kinematic joint '%s' is absent from group joints",
              group_name.c_str(), role, joint.getName().c_str());
            return false;
          }
          group_joint_indices.push_back(
            static_cast<size_t>(std::distance(group_joints.begin(), group_joint)));
        }
        if (group_joint_indices.size() != chain.getNrOfJoints() || chain.getNrOfJoints() == 0U) {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "[%s] Invalid %s relative-pose chain (%u KDL joints, %zu mapped)",
            group_name.c_str(), role, chain.getNrOfJoints(), group_joint_indices.size());
          return false;
        }
        return true;
      };
    if (!build_chain(leader_tree, "leader", leader_chain, leader_indices) ||
      !build_chain(follower_tree, "follower", follower_chain, follower_indices))
    {
      group_pose_kinematics_.clear();
      return false;
    }

    auto kinematics =
      std::make_shared<GroupPoseKinematics>(leader_chain, follower_chain);
    kinematics->leader.group_joint_indices = std::move(leader_indices);
    kinematics->follower.group_joint_indices = std::move(follower_indices);
    group_pose_kinematics_[group_name] = std::move(kinematics);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[%s] Relative-pose kinematics configured: leader %s -> %s (%u joints), "
      "follower %s -> %s (%u joints)",
      group_name.c_str(), params_.relative_base_link.c_str(), end_effector.c_str(),
      leader_chain.getNrOfJoints(), params_.relative_base_link.c_str(), end_effector.c_str(),
      follower_chain.getNrOfJoints());
  }
  return !group_pose_kinematics_.empty();
}

bool JointTrajectoryCommandBroadcaster::update_relative_pose_target(
  const std::string & group_name,
  const std::vector<double> & leader_reference,
  const double period_sec,
  std::vector<double> & candidate)
{
  const auto kinematics_item = group_pose_kinematics_.find(group_name);
  const auto target_item = group_last_target_.find(group_name);
  if (kinematics_item == group_pose_kinematics_.end() || !kinematics_item->second ||
    target_item == group_last_target_.end() || leader_reference.size() != target_item->second.size() ||
    !std::isfinite(period_sec) || period_sec <= 0.0)
  {
    return false;
  }
  auto & kinematics = *kinematics_item->second;
  const std::vector<double> & current_action = target_item->second;
  KDL::JntArray current_joints;
  Eigen::Isometry3d leader_pose;
  Eigen::Isometry3d current_pose;
  if (!compute_pose(kinematics.leader, leader_reference, leader_pose) ||
    !compute_pose(kinematics.follower, current_action, current_pose, &current_joints))
  {
    return false;
  }

  if (!kinematics.relative_mapper.anchored()) {
    if (!kinematics.relative_mapper.capture(leader_pose, current_pose)) {
      return false;
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "[%s] Relative end-effector anchors captured",
      group_name.c_str());
  }

  Eigen::Isometry3d goal;
  if (!kinematics.relative_mapper.map(leader_pose, goal)) {
    return false;
  }

  Eigen::Matrix<double, 6, 1> desired_velocity;
  desired_velocity.head<3>() =
    params_.relative_kp_position * (goal.translation() - current_pose.translation());
  desired_velocity.tail<3>() = params_.relative_kp_orientation *
    shortest_orientation_error(goal.linear(), current_pose.linear());

  KDL::Jacobian kdl_jacobian(kinematics.follower.chain.getNrOfJoints());
  if (kinematics.follower.jacobian_solver->JntToJac(current_joints, kdl_jacobian) < 0) {
    return false;
  }
  const Eigen::Index joint_count =
    static_cast<Eigen::Index>(kinematics.follower.chain.getNrOfJoints());
  Eigen::MatrixXd jacobian(6, joint_count);
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index col = 0; col < joint_count; ++col) {
      jacobian(row, col) = kdl_jacobian(
        static_cast<unsigned int>(row), static_cast<unsigned int>(col));
    }
  }
  Eigen::Matrix<double, 6, 1> weights;
  weights.head<3>().setConstant(params_.relative_weight_position);
  weights.tail<3>().setConstant(params_.relative_weight_orientation);
  Eigen::VectorXd joint_velocity;
  if (!solve_weighted_damped_velocity(
      jacobian, desired_velocity, weights, params_.relative_damping_weight, joint_velocity))
  {
    return false;
  }

  candidate = current_action;
  const auto & lower = group_lower_limits_[group_name];
  const auto & upper = group_upper_limits_[group_name];
  const auto & velocity_limit = group_velocity_limits_[group_name];
  std::vector<bool> controlled(candidate.size(), false);
  for (Eigen::Index i = 0; i < joint_count; ++i) {
    const size_t group_index =
      kinematics.follower.group_joint_indices[static_cast<size_t>(i)];
    controlled[group_index] = true;
    double velocity = joint_velocity[i];
    if (group_index < velocity_limit.size() && std::isfinite(velocity_limit[group_index])) {
      velocity = std::clamp(
        velocity, -velocity_limit[group_index], velocity_limit[group_index]);
    }
    double position = current_action[group_index] + period_sec * velocity;
    if (group_index < lower.size() && group_index < upper.size()) {
      position = std::clamp(position, lower[group_index], upper[group_index]);
    }
    candidate[group_index] = position;
  }

  // Cyclo treats grippers as auxiliary references, outside RelativePoseMode.
  // Keep the final executed gripper action until the leader moves outside the
  // clutch deadband; other auxiliary joints follow the leader directly.
  const auto runtime_item = group_runtime_.find(group_name);
  if (runtime_item == group_runtime_.end()) {
    return false;
  }
  const auto & clutch = runtime_item->second.gripper_clutch;
  for (size_t i = 0; i < candidate.size(); ++i) {
    if (controlled[i]) {
      continue;
    }
    double position = leader_reference[i];
    if (is_gripper_joint(group_joint_names_[group_name][i])) {
      if (!clutch.anchored || clutch.leader_anchor.size() != candidate.size() ||
        clutch.command_anchor.size() != candidate.size() ||
        !try_map_gripper_clutch_target(
          leader_reference[i], clutch.leader_anchor[i], clutch.command_anchor[i],
          params_.gripper_clutch_deadband, params_.gripper_clutch_scale, position))
      {
        return false;
      }
    }
    if (i < lower.size() && i < upper.size()) {
      position = std::clamp(position, lower[i], upper[i]);
    }
    candidate[i] = position;
  }
  return std::all_of(candidate.begin(), candidate.end(), [](const double value) {
           return std::isfinite(value);
    });
}

void JointTrajectoryCommandBroadcaster::reset_relative_pose_anchor(
  const std::string & group_name)
{
  const auto item = group_pose_kinematics_.find(group_name);
  if (item != group_pose_kinematics_.end() && item->second) {
    item->second->relative_mapper.reset();
  }
}

void JointTrajectoryCommandBroadcaster::reset_gripper_clutch(
  const std::string & group_name)
{
  auto & clutch = group_runtime_[group_name].gripper_clutch;
  clutch.leader_anchor.clear();
  clutch.command_anchor.clear();
  clutch.anchored = false;
}

bool JointTrajectoryCommandBroadcaster::sync_follower_target(
  const std::string & group_name, const bool preserve_commanded_grippers)
{
  auto buffer_it = group_follower_state_buffers_.find(group_name);
  if (buffer_it == group_follower_state_buffers_.end() || !buffer_it->second) {
    return false;
  }
  const auto * follower_positions = buffer_it->second->readFromRT();
  if (!follower_positions || follower_positions->empty()) {
    return false;
  }
  auto & held_action = group_last_target_[group_name];
  if (held_action.empty() || !preserve_commanded_grippers) {
    // At process startup there is no command provenance yet. Explicit callers
    // may also request a complete measured-state rebase, including grippers.
    return try_store_finite_action(*follower_positions, held_action);
  }

  const std::vector<double> * commanded_action = &held_action;
  auto executed_it = group_executed_action_buffers_.find(group_name);
  if (executed_it != group_executed_action_buffers_.end() && executed_it->second) {
    const auto * executed_action = executed_it->second->readFromRT();
    if (executed_action && !executed_action->empty()) {
      commanded_action = executed_action;
    }
  }
  return try_rebase_measured_action_preserving_grippers(
    *follower_positions, *commanded_action, group_joint_names_[group_name], held_action);
}

void JointTrajectoryCommandBroadcaster::prepare_teleop_entry(
  const std::string & group_name)
{
  auto & rt = group_runtime_[group_name];
  reset_relative_pose_anchor(group_name);
  reset_gripper_clutch(group_name);
  if (teleop_reference_mode_ == TeleopReferenceMode::RELATIVE) {
    rt.blend.active = false;
    return;
  }
  start_teleop_blend(group_name);
}

void JointTrajectoryCommandBroadcaster::start_teleop_blend(
  const std::string & group_name)
{
  auto last_it = group_last_target_.find(group_name);
  if (last_it == group_last_target_.end() || last_it->second.empty() ||
    !std::all_of(last_it->second.begin(), last_it->second.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return;
  }
  auto & bs = group_runtime_[group_name].blend;
  bs.start_pos = last_it->second;   // snapshot
  bs.start_time = get_node()->now();
  bs.active = true;
  RCLCPP_INFO(get_node()->get_logger(),
    "[%s] Start teleop blend (%.1fs)",
    group_name.c_str(), params_.leader_blend_duration);
}

bool JointTrajectoryCommandBroadcaster::start_save_pose_interp(
  const std::string & group_name, const std::vector<double> & target)
{
  auto last_it = group_last_target_.find(group_name);
  if (last_it == group_last_target_.end() || last_it->second.empty() ||
    !std::all_of(last_it->second.begin(), last_it->second.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] last_target not initialized; skip interpolation", group_name.c_str());
    return false;
  }
  if (target.size() != last_it->second.size() ||
    !std::all_of(target.begin(), target.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] save pose is invalid (size=%zu, expected=%zu)",
      group_name.c_str(), target.size(), last_it->second.size());
    return false;
  }
  auto & st = group_runtime_[group_name].interp;
  st.start_time = get_node()->now();
  st.start_pos = last_it->second;
  st.target_pos = target;
  st.duration_sec = params_.save_pose_duration;
  st.active = true;
  return true;
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
