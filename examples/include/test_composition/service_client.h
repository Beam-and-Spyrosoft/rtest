/**
 * @file      service_client.h
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

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <functional>

namespace test_composition {

class ServiceClient : public rclcpp::Node {
public:
  explicit ServiceClient(const rclcpp::NodeOptions &options);

  using CallbackType = std::function<void(std::shared_future<std::shared_ptr<std_srvs::srv::SetBool_Response>>)>;

  using CallbackWithRequestType = std::function<void(std::shared_future<std::pair<
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