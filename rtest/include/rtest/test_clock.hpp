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

    timers_ = findTimers(node);
    clock_ = node->get_clock()->get_clock_handle();
    resetClock();
  }

  void advance(std::chrono::milliseconds milliseconds, bool with_timers_callbacks=true)
  {
    const auto target_time = now_ + std::chrono::nanoseconds(milliseconds).count();
    const auto step_size_ns = with_timers_callbacks ? std::chrono::nanoseconds(std::chrono::milliseconds(1)).count() : target_time;

    while (now_ < target_time) {
      const auto remaining = target_time - now_;
      const auto step = std::min(step_size_ns, remaining);
      now_ += step;

      if (rcl_set_ros_time_override(clock_, now_) != RCL_RET_OK) {
        throw std::runtime_error{"TestClock::advance() error"};
      }

      if (with_timers_callbacks) {
        fire_all_timer_callbacks();
      }
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
  void fire_all_timer_callbacks() {
    for (auto& timer : timers_) {
      if (timer->is_ready()) {
        auto data = timer->call();
        if (!data) {
          continue;
        }
        timer->execute_callback(data);
      }
    }
  }

  rcl_clock_t * clock_{nullptr};
  rcl_time_point_value_t now_{0L};
  std::vector<std::shared_ptr<rclcpp::TimerBase>> timers_{};
};
}  // namespace rtest