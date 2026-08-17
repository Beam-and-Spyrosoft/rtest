// Copyright 2025 Spyrosoft Limited.
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
// @file      clock.cpp
// @author    Wiktor Bajor (wiktorbajor1@gmail.com)
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2025-09-15
//
// @brief     Unit tests for the Test Clock implementation.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rtest/test_clock.hpp>

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node
{
public:
  TimerNode()
  : rclcpp::Node(
      "timer_node",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}))
  {
  }

  template <typename Period>
  void add_timer(Period timer_period, std::function<void()> cb)
  {
    timers_.push_back(create_timer(timer_period, cb));
  }

  std::vector<rclcpp::TimerBase::SharedPtr> & timers() { return timers_; }

private:
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

TEST(ClockTests, WhenTheTimeIsMovedByTimerPeriodCallbackShouldBeExecuted)
{
  auto node = std::make_shared<TimerNode>();
  auto triggering_test_clock = rtest::TriggeringTestClock{node};

  // Add 5ms timer
  int call_counter_5ms{0};
  node->add_timer(5ms, [&]() { call_counter_5ms++; });

  // Clock starts at 0.0s
  ASSERT_EQ(triggering_test_clock.now(), 0UL);

  triggering_test_clock.advance(4ms);
  ASSERT_EQ(triggering_test_clock.now(), 4000000UL);  // clock -> 0.004s
  EXPECT_EQ(call_counter_5ms, 0);

  // We expect the timer to trigger every 5ms
  triggering_test_clock.advance(1ms);
  ASSERT_EQ(triggering_test_clock.now(), 5000000UL);  // clock -> 0.005s
  EXPECT_EQ(call_counter_5ms, 1);

  // We do not expect the timer to trigger after one period expires but before the next begins
  triggering_test_clock.advance(1ms);
  ASSERT_EQ(triggering_test_clock.now(), 6000000UL);  // clock -> 0.006s
  EXPECT_EQ(call_counter_5ms, 1);

  // We expect the timer to trigger every 500ms, so when the expiry time passes, the callback should fire
  triggering_test_clock.advance(5ms);
  ASSERT_EQ(triggering_test_clock.now(), 11000000UL);  // clock -> 0.011s
  EXPECT_EQ(call_counter_5ms, 2);

  // Add 50us timer
  int call_counter_50us{0};
  node->add_timer(50us, [&]() { call_counter_50us++; });

  // Advance by less than 50us, no timer triggered
  triggering_test_clock.advance(49us);
  ASSERT_EQ(triggering_test_clock.now(), 11049000UL);  // clock -> 0.011049s
  EXPECT_EQ(call_counter_5ms, 2);
  EXPECT_EQ(call_counter_50us, 0);

  // Trigger 50us timer once, but not 500ms yet
  triggering_test_clock.advance(1us);
  ASSERT_EQ(triggering_test_clock.now(), 11050000UL);  // clock -> 0.011050s
  EXPECT_EQ(call_counter_5ms, 2);
  EXPECT_EQ(call_counter_50us, 1);

  // Trigger 500ms timer once and 9019 times the 50us timer
  triggering_test_clock.advance(4ms);                  // 4000us/50 us = 80
  ASSERT_EQ(triggering_test_clock.now(), 15050000UL);  // clock -> 0.015050s
  EXPECT_EQ(call_counter_5ms, 3);
  EXPECT_EQ(call_counter_50us, 81);

  // Add 50ns timer
  int call_counter_50ns{0};
  node->add_timer(50ns, [&]() { call_counter_50ns++; });

  // Advance by less than 5ns, no timer triggered
  triggering_test_clock.advance(49ns);
  ASSERT_EQ(triggering_test_clock.now(), 15050049UL);  // clock -> 0.015050049s
  EXPECT_EQ(call_counter_5ms, 3);
  EXPECT_EQ(call_counter_50us, 81);
  EXPECT_EQ(call_counter_50ns, 0);

  // Trigger 5ns timer once only
  triggering_test_clock.advance(1ns);
  ASSERT_EQ(triggering_test_clock.now(), 15050050UL);  // clock -> 0.015050050s
  EXPECT_EQ(call_counter_5ms, 3);
  EXPECT_EQ(call_counter_50us, 81);
  EXPECT_EQ(call_counter_50ns, 1);

  // Trigger all timers (50ns_timer -> 100000-times; 50us_timer -> 100 times and 5ms_timers once)
  triggering_test_clock.advance(5ms);
  ASSERT_EQ(triggering_test_clock.now(), 20050050UL);  // clock -> 0.020050050s
  EXPECT_EQ(call_counter_5ms, 4);
  EXPECT_EQ(call_counter_50us, 181);
  EXPECT_EQ(call_counter_50ns, 100001);
}
