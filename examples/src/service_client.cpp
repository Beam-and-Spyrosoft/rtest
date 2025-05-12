/**
 * @file      service_client.cpp
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

#include "test_composition/service_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace test_composition
{

ServiceClient::ServiceClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_service_client", options)
{
  client_ = create_client<std_srvs::srv::SetBool>("test_service");
}

bool ServiceClient::setState(bool state)
{
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Service not available");
    last_call_success_ = false;
    last_response_message_ = "Service not available";
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  auto future_result = client_->async_send_request(request).share();

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Service call failed");
    last_call_success_ = false;
    last_response_message_ = "Service call failed";
    return false;
  }

  try {
    auto response = future_result.get();
    last_call_success_ = response->success;
    last_response_message_ = response->message;
    return response->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Service call error: %s", e.what());
    last_call_success_ = false;
    last_response_message_ = e.what();
    return false;
  }
}

bool ServiceClient::setStateWithCallback(bool state, ServiceClient::CallbackType callback)
{
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Service not available");
    last_call_success_ = false;
    last_response_message_ = "Service not available";
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  auto future_and_id = client_->async_send_request(request, callback);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future_and_id.future, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Service call failed");
    last_call_success_ = false;
    last_response_message_ = "Service call failed";
    return false;
  }

  try {
    auto response = future_and_id.future.get();
    last_call_success_ = response->success;
    last_response_message_ = response->message;
    return response->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Service call error: %s", e.what());
    last_call_success_ = false;
    last_response_message_ = e.what();
    return false;
  }
}

bool ServiceClient::setStateWithRequestCallback(
  bool state,
  ServiceClient::CallbackWithRequestType callback)
{
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Service not available");
    last_call_success_ = false;
    last_response_message_ = "Service not available";
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  auto future_and_id = client_->async_send_request(request, callback);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future_and_id.future, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Service call failed");
    last_call_success_ = false;
    last_response_message_ = "Service call failed";
    return false;
  }

  try {
    auto [req_res, response] = future_and_id.future.get();
    last_call_success_ = response->success;
    last_response_message_ = response->message;
    return response->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Service call error: %s", e.what());
    last_call_success_ = false;
    last_response_message_ = e.what();
    return false;
  }
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ServiceClient)