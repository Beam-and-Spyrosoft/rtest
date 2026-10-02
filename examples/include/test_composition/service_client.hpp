// Copyright 2025 Spyrosoft Limited.
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
// @file      service_client.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <functional>
#include <vector>

namespace test_composition
{

class ServiceClient : public rclcpp::Node
{
public:
  explicit ServiceClient(const rclcpp::NodeOptions & options);

  using CallbackType =
    std::function<void(std::shared_future<std::shared_ptr<std_srvs::srv::SetBool_Response>>)>;

  using CallbackWithRequestType =
    std::function<void(std::shared_future<std::pair<
                         std::shared_ptr<std_srvs::srv::SetBool_Request>,
                         std::shared_ptr<std_srvs::srv::SetBool_Response>>>)>;

  /**
   * @brief Call service to set state
   * @param state New state to set
   * @return true if service call succeeded, false otherwise
   */
  bool setState(bool state);

  /**
   * @brief Call service to set state with callback
   * @param state New state to set
   * @param callback Callback function called with future response
   * @return true if service call succeeded, false otherwise
   */
  bool setStateWithCallback(bool state, CallbackType callback);

  /**
   * @brief Call service to set state with request callback
   * @param state New state to set
   * @param callback Callback function called with future containing request and response
   * @return true if service call succeeded, false otherwise
   */
  bool setStateWithRequestCallback(bool state, CallbackWithRequestType callback);

  /**
   * @brief Call service to set state, blocking on the future instead of spinning the node
   *
   * On timeout the request is removed from the client, because
   * rclcpp::Client keeps every request without a response for the whole lifetime of the client.
   *
   * @param state New state to set
   * @param timeout Maximum time to wait for the response
   * @return true if service call succeeded, false otherwise
   */
  bool setStateWithTimeout(bool state, std::chrono::nanoseconds timeout);

  /**
   * @brief Give up on all requests that have not received a response yet
   * @return Number of requests that were dropped
   */
  size_t cancelPendingRequests();

  /**
   * @brief Give up on requests that have been waiting for a response for longer than max_age
   *
   * Typically called periodically from a timer, as recommended by rclcpp for requests sent with
   * a response callback.
   *
   * @param max_age Maximum time a request may wait for its response
   * @return Ids of the requests that were dropped
   */
  std::vector<int64_t> dropRequestsOlderThan(std::chrono::nanoseconds max_age);

  /**
   * @brief Get last service call success status
   * @return true if last call succeeded, false otherwise
   */
  bool getLastCallSuccess() const { return last_call_success_; }

  /**
   * @brief Get last received service response message
   * @return Response message string
   */
  std::string getLastResponseMessage() const { return last_response_message_; }

private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  bool last_call_success_{false};
  std::string last_response_message_;
};

}  // namespace test_composition