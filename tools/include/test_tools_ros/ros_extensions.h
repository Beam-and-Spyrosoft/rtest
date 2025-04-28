/**
 * @file       ros_extensions.h
 * @brief      Useful extensions for ROS when testing.
 * @date       7 Feb 2019
 * @author     Magnus Maynard
 *
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief    extensions for waiting for messages and conditions
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

#pragma once

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>

namespace test_tools {

/** @brief Waits for a given duration, allowing ROS messages to
 *  be sent.
 *
 *  @param seconds The duration to wait.
 */
static void Wait(double seconds = 0.2) {
  rclcpp::Rate rate(20);
  auto start = std::chrono::system_clock::now();
  auto end = start + std::chrono::milliseconds(static_cast<int>(seconds * 1000));

  while (std::chrono::system_clock::now() < end && rclcpp::ok()) {
    rate.sleep();
  }
}

/** @brief Waits until a given time, allowing ROS messages to
 *  be sent.
 *
 *  @param time The rclcpp time to wait until.
 */
static void WaitUntil(const rclcpp::Time time) {
  rclcpp::Rate rate(20);

  while (rclcpp::Clock().now() < time) {
    rate.sleep();
  }
}

/** @brief Waits until a given condition is true, or the maximum
 *  time has been reached. ROS messages can be sent whilst waiting. Type safe overload using Chrono
 *
 *  @param conditional_function Function when returns true, will exit the wait.
 *  @param max_wait Time until wait times out.
 *  @returns True when condition has been meet or false when the wait times out.
 */
template <typename T, class ChronoRep, class ChronoPeriod>
static bool WaitUntil(T &&condition_function, const std::chrono::duration<ChronoRep, ChronoPeriod> max_wait) {
  rclcpp::Rate rate(20);
  auto start = std::chrono::system_clock::now();
  auto end = start + max_wait;

  while (std::chrono::system_clock::now() < end && rclcpp::ok()) {
    if (condition_function()) {
      return true;
    }
    rate.sleep();
  }
  std::cerr << "Wait timed out, condition never met." << std::endl;
  return false;
}

/** @brief Overload taking in a 'double' for wait time in seconds which calls an overload using chrono time
 *
 *  @param conditional_function Function when returns true, will exit the wait.
 *  @param max_wait_seconds Seconds until wait times out.
 *  @returns True when condition has been meet or false when the wait times out.
 */
template <typename T>
static bool WaitUntil(T &&condition_function, const double max_wait_seconds) {
  return WaitUntil(condition_function, std::chrono::milliseconds(static_cast<int>(max_wait_seconds * 1000)));
}

/**
 * @brief Wait for a callback to be triggered
 *
 * @param timeout Maximum wait before thread continues
 * @param callback_cv Conditional variable within the callback which must notify a thread to wake up
 * @param callback_mutex Mutex owned by callback
 */
template <class ChronoRep, class ChronoPeriod>
static void WaitForMessage(
    const std::chrono::duration<ChronoRep, ChronoPeriod> &timeout,
    std::condition_variable &callback_cv,
    std::mutex &callback_mutex) {
  std::unique_lock<std::mutex> lk(callback_mutex);
  callback_cv.wait_for(lk, timeout);
}

}  // namespace test_tools
