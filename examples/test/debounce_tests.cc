/**
 * @file      debounce_tests.cc
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-12-4
 * @copyright Copyright (c) 2024 Beam Limited.
 * @brief
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <test_composition/debounce.h>
#include <ros2_test_framework/test_clock.h>

using namespace std::chrono_literals;

class DebounceTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(DebounceTest, When_NodeUseSimTimeParameter_NotSetTrue_ThrowException) {
  auto node = std::make_shared<test_composition::DebounceNode>("default_opts", 123ms, opts);

  // No use_sim_time option
  EXPECT_THROW(ros2_test_framework::TestClock{node}, std::invalid_argument);

  opts = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", false)});
  node = std::make_shared<test_composition::DebounceNode>("use_sim_time_false", 123ms, opts);

  // use_sim_time option set to false
  EXPECT_THROW(ros2_test_framework::TestClock{node}, std::invalid_argument);

  opts = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  node = std::make_shared<test_composition::DebounceNode>("use_sim_time_true", 123ms, opts);

  // use_sim_time option set to true
  EXPECT_NO_THROW(ros2_test_framework::TestClock{node});
}

TEST_F(DebounceTest, When_TimeElapsedIsLessThanDebounceTime_ValueIsSetOnlyOnce) {
  const auto DEBOUNCE_TIME{123ms};

  opts = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto debounce_int = std::make_shared<test_composition::DebounceNode>("test_node", DEBOUNCE_TIME, opts);

  auto test_clock = ros2_test_framework::TestClock{debounce_int};

  // Allow to set the value initially by advancing the time to t > DEBOUNCE_TIME
  test_clock.advance(DEBOUNCE_TIME + 1ms);

  // First set allowed
  debounce_int->set(42);
  EXPECT_EQ(debounce_int->get(), 42);

  // Every another set ignored
  debounce_int->set(123);
  EXPECT_EQ(debounce_int->get(), 42);
  debounce_int->set(-1);
  EXPECT_EQ(debounce_int->get(), 42);

  // Still not reached DEBOUNCE_TIME
  test_clock.advance(DEBOUNCE_TIME - 1ms);

  debounce_int->set(321);
  EXPECT_EQ(debounce_int->get(), 42);

  // Now currnet time is == last_set_time + DEBOUNCE_TIME
  test_clock.advance(1ms);

  // Allow set the value once
  debounce_int->set(123);
  EXPECT_EQ(debounce_int->get(), 123);

  // Forbid set again if last set time < DEBOUNCE_TIME
  debounce_int->set(42);
  EXPECT_EQ(debounce_int->get(), 123);

  test_clock.advance(1ms);
  debounce_int->set(42);
  EXPECT_EQ(debounce_int->get(), 123);
}