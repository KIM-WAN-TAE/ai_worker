#include <chrono>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include <dynamixel_interfaces/msg/dynamixel_state.hpp>
#include "ffw_robot_manager/topic_watchdog.hpp"

using namespace std::chrono_literals;
using State = dynamixel_interfaces::msg::DynamixelState;

class WatchdogTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("led_watchdog_test");
    publisher_node = std::make_shared<rclcpp::Node>("led_watchdog_test_publisher");
    publisher = publisher_node->create_publisher<State>("/test_led_watchdog/state", 10);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node->get_node_base_interface());
    watchdog = std::make_unique<ffw_robot_manager::TopicWatchdog<State>>(
      node.get(), "/test_led_watchdog/state", 500ms,
      [this]() {++timeouts;}, [this]() {++recoveries;},
      [](const State & msg) {return msg.comm_state == 0;});
  }
  void TearDown() override
  {
    watchdog.reset();
    executor.reset();
    publisher.reset();
    publisher_node.reset();
    node.reset();
    rclcpp::shutdown();
  }
  void run(std::chrono::milliseconds duration, bool publish, int comm = 0)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      if (publish) {
        State msg;
        msg.comm_state = comm;
        publisher->publish(msg);
      }
      executor->spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr publisher_node;
  rclcpp::Publisher<State>::SharedPtr publisher;
  std::unique_ptr<ffw_robot_manager::TopicWatchdog<State>> watchdog;
  int timeouts = 0;
  int recoveries = 0;
};

TEST_F(WatchdogTest, RecoversOnlyAfterSustainedHealthyTraffic)
{
  run(700ms, false);
  ASSERT_EQ(timeouts, 1);
  run(1000ms, true);
  EXPECT_EQ(recoveries, 0);
  run(700ms, false);
  run(1500ms, true);
  EXPECT_EQ(recoveries, 0);
  run(700ms, true);
  EXPECT_EQ(recoveries, 1);
  run(700ms, false);
  EXPECT_EQ(timeouts, 2);
}

TEST_F(WatchdogTest, ErrorTrafficCannotClearWarning)
{
  run(700ms, false);
  run(2200ms, true, -11);
  EXPECT_EQ(timeouts, 1);
  EXPECT_EQ(recoveries, 0);
  EXPECT_TRUE(watchdog->is_timed_out());
}

TEST_F(WatchdogTest, HealthyTrafficDoesNotProduceWarning)
{
  run(1200ms, true);
  EXPECT_EQ(timeouts, 0);
  EXPECT_EQ(recoveries, 0);
}
