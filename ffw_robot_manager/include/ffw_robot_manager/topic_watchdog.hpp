// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Woojin Wie

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ffw_robot_manager
{

template<typename MsgT>
class TopicWatchdog {
public:
  using TimeoutCallback = std::function<void()>;
  using HealthCheck = std::function<bool(const MsgT &)>;

  TopicWatchdog(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & topic_name,
    std::chrono::milliseconds timeout,
    TimeoutCallback on_timeout_cb,
    TimeoutCallback on_recovered_cb = {},
    HealthCheck is_healthy = {})
  : node_(node),
    topic_name_(topic_name),
    timeout_(timeout),
    on_timeout_cb_(on_timeout_cb),
    on_recovered_cb_(on_recovered_cb),
    is_healthy_(is_healthy)
  {
    sub_ = node_->create_subscription<MsgT>(
      topic_name_, 10,
      [this](const typename MsgT::SharedPtr msg) {
        const auto now = std::chrono::steady_clock::now();
        if (is_healthy_ && !is_healthy_(*msg)) {
          healthy_since_ = {};
          return;
        }
        if (now - last_msg_time_ > timeout_) {
          healthy_since_ = {};
        }
        last_msg_time_ = now;
        if (timed_out_) {
          if (healthy_since_ == std::chrono::steady_clock::time_point{}) {
            healthy_since_ = now;
          }
          // A single packet must not clear an intermittent communication fault.
          if (now - healthy_since_ >= std::chrono::seconds(2)) {
            timed_out_ = false;
            if (on_recovered_cb_) {on_recovered_cb_();}
          }
        }
      });
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TopicWatchdog::check_timeout, this));
    last_msg_time_ = std::chrono::steady_clock::now();
  }

  bool is_timed_out() const {return timed_out_;}

private:
  void check_timeout()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_msg_time_ > timeout_) {
      healthy_since_ = {};
      if (!timed_out_) {
        timed_out_ = true;
        if (on_timeout_cb_) {on_timeout_cb_();}
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode * node_;
  std::string topic_name_;
  std::chrono::milliseconds timeout_;
  TimeoutCallback on_timeout_cb_;
  TimeoutCallback on_recovered_cb_;
  HealthCheck is_healthy_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_msg_time_;
  std::chrono::steady_clock::time_point healthy_since_{};
  bool timed_out_ = false;
};

}  // namespace ffw_robot_manager
