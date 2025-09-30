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
#include <type_traits>

namespace rtest
{

template <typename T>
struct is_chrono_duration : std::false_type
{
};

template <typename Rep, typename Period>
struct is_chrono_duration<std::chrono::duration<Rep, Period>> : std::true_type
{
};

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

  rcl_time_point_value_t now() const { return now_; }

  template <typename Duration>
  void advance(Duration duration)
  {
    static_assert(
      is_chrono_duration<Duration>::value, "duration must be a std::chrono::duration type");

    now_ += std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    if (rcl_set_ros_time_override(clock_, now_) != RCL_RET_OK) {
      throw std::runtime_error{"TestClock::advance() error"};
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

/**
 * @brief Test utility for manual time control. Takes over control over the given Node's clock.
 *        The Node must be constructed with parameter "use_sim_time" set to true.
 *        This implementation triggers timers' callbacks as well.
 */
class TriggeringTestClock
{
public:
  TriggeringTestClock(rclcpp::Node::SharedPtr node) : clock_{TestClock(node)}, node_{node} {}

  rcl_time_point_value_t now() const { return clock_.now(); }

  template <typename Duration>
  void advance(Duration target_time)
  {
    static_assert(
      is_chrono_duration<Duration>::value, "target_time must be a std::chrono::duration type");

    // Nodes might have added/removed/changed timers -> update the timers list
    const auto timers = findTimers(node_.lock());

    if (!timers.empty()) {
      const auto time_step = get_timers_min_period(timers);

      auto start_point = std::chrono::nanoseconds(0);
      while (start_point < target_time) {
        const auto remaining = target_time - start_point;
        const auto step =
          std::min(std::chrono::duration_cast<std::chrono::nanoseconds>(time_step), remaining);
        start_point += step;
        clock_.advance(step);
        fire_all_timer_callbacks(timers);
      }
    } else {
      clock_.advance(target_time);
    }
  }

  void resetClock(const rcl_time_point_value_t tv = 0L) { clock_.resetClock(tv); }

private:
  std::chrono::nanoseconds get_timers_min_period(
    const std::vector<std::shared_ptr<rclcpp::TimerBase>> & timers)
  {
    int64_t min_timer_period_ns{std::numeric_limits<int64_t>::max()};
    for (auto & timer : timers) {
      int64_t timer_period;
      if (rcl_timer_get_period(timer->get_timer_handle().get(), &timer_period) != RCL_RET_OK) {
        throw std::runtime_error{"TriggeringTestClock: rcl_timer_get_period() error"};
      }
      min_timer_period_ns = std::min(min_timer_period_ns, timer_period);
    }
    return std::chrono::nanoseconds{min_timer_period_ns};
  }

  void fire_all_timer_callbacks(const std::vector<std::shared_ptr<rclcpp::TimerBase>> & timers)
  {
    for (auto & timer : timers) {
      if (timer->is_ready()) {
        auto data = timer->call();
        if (!data) {
          continue;
        }
        timer->execute_callback(data);
      }
    }
  }

  TestClock clock_;
  rclcpp::Node::WeakPtr node_;
};

}  // namespace rtest
