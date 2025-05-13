/**
 * @file      publisher.cpp
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

#include "test_composition/publisher.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace test_composition
{

Publisher::Publisher(const rclcpp::NodeOptions & options) : rclcpp::Node("test_publisher", options)
{
  publisher_ = create_publisher<std_msgs::msg::String>("test_topic", rclcpp::QoS{5UL});
  timer = create_timer(500ms, [this]() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->set__data("timer");
    publisher_->publish(std::move(msg));
  });
}

void Publisher::publishCopy()
{
  auto msg = std_msgs::msg::String{};
  msg.set__data("copy");
  publisher_->publish(msg);
}

void Publisher::publishUniquePtr()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->set__data("unique_ptr");
  publisher_->publish(std::move(msg));
}

void Publisher::publishLoanedMsg()
{
  auto msg = publisher_->borrow_loaned_message();
  msg.get().set__data("loaned_msg");
  publisher_->publish(std::move(msg));
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::Publisher);
