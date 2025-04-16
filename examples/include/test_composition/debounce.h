/**
 * @file      debounce.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-12-4
 * @copyright Copyright (c) 2024 Beam Limited.
 *   
 * @brief   Simple implementation that uses time measurement and avoids setting a value when some arbitrary time hasn't
 * elapsed.
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

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace test_composition {

/**
 * @brief Simple implementation that uses time measurement and avoids setting a value when some arbitrary time hasn't
 * elapsed.
 *
 */
class DebounceNode : public rclcpp::Node {
public:
  DebounceNode(const std::string &name, std::chrono::milliseconds debounce_time, const rclcpp::NodeOptions &options) :
        Node(name, options), debounce_time_(debounce_time), last_set_time_(get_clock()->now()) {}

  void set(int new_value) {
    const auto now = get_clock()->now();
    if ((now - last_set_time_) >= debounce_time_) {
      value_ = new_value;
      last_set_time_ = now;
    }
  }

  int get() const { return value_; }

private:
  int value_{};
  std::chrono::milliseconds debounce_time_;
  rclcpp::Time last_set_time_;
};

}  // namespace test_composition