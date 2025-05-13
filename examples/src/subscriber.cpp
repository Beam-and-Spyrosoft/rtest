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
// @file      subscriber.cpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-26

#include "test_composition/subscriber.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace test_composition
{

Subscriber::Subscriber(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_subscriber", options)
{
  subscription = create_subscription<std_msgs::msg::String>(
    "test_topic", rclcpp::SensorDataQoS(), [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str());
      lastMsg_ = *msg;
    });
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::Subscriber);
