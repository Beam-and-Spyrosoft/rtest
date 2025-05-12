/**
 * @file      publisher.hpp
 * @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>

namespace test_composition
{

class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const rclcpp::NodeOptions & options);

  void publishCopy();
  void publishUniquePtr();
  void publishLoanedMsg();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace test_composition
