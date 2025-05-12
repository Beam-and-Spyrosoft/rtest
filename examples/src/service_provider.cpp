/**
 * @file      service_provider.cpp
 * @author    Mariusz Szczepanik (mua@spyro-soft.com)
 * @date      2025-05-28
 * @copyright Copyright (c) 2025 Spyrosoft Limited.
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

#include "test_composition/service_provider.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace test_composition
{

ServiceProvider::ServiceProvider(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_service_provider", options)
{
  service_ = create_service<std_srvs::srv::SetBool>(
    "test_service",
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
      handleServiceRequest(request, response);
    });
}

bool ServiceProvider::getState() const { return state_; }

void ServiceProvider::handleServiceRequest(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  RCLCPP_INFO(
    get_logger(), "Received service request with data: %s", request->data ? "true" : "false");

  if (!locked_) {
    RCLCPP_INFO(get_logger(), "Service is not locked, updating state");
    state_ = request->data;
    response->success = true;
    response->message = "State updated successfully";
  } else {
    RCLCPP_WARN(get_logger(), "Service is locked, cannot update state");
    response->success = false;
    response->message = "Service is locked, cannot update state";
  }
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ServiceProvider)