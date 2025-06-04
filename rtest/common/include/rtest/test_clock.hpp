// Copyright 2024 Beam Limited.
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
// @file      test_clock.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-12-02
//
// @brief     ROS2 test clock utility.

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace rtest
{

/**
 * @brief Test utility for manual time control. Takes over control over the given Node's clock.
 *        The Node must be constructed with parameter "use_sim_time" set to true.
 *
 */
class TestClock
{
public:
  TestClock(rclcpp::Node::SharedPtr node)
  {
    if (!node) {
      throw std::invalid_argument{"TestClock - invalid node ptr"};
    }
    auto use_sim_time = node->get_parameter("use_sim_time");
    if (!use_sim_time.as_bool()) {
      throw std::invalid_argument{"TestClock - The node must be set with use_sim_time = true"};
    }

    clock_ = node->get_clock()->get_clock_handle();
    resetClock();
  }

  void advance(std::chrono::milliseconds milliseconds)
  {
    now_ += (milliseconds.count() * 1000000L);
    if (rcl_set_ros_time_override(clock_, now_) != RCL_RET_OK) {
      throw std::runtime_error{"TestClock::advanceMs() error"};
    }
  }

  void advanceMs(int64_t milliseconds) { advance(std::chrono::milliseconds(milliseconds)); }

  void resetClock(const rcl_time_point_value_t tv = 0L)
  {
    if (rcl_set_ros_time_override(clock_, tv) != RCL_RET_OK) {
      throw std::runtime_error{"TestClock::advanceMs() error"};
    }
  }

private:
  rcl_clock_t * clock_{nullptr};
  rcl_time_point_value_t now_{0L};
};
}  // namespace rtest