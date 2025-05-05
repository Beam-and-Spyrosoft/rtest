/**
 * @file      service_provider.h
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

#include <atomic>

namespace test_composition {

class ServiceProvider : public rclcpp::Node {
public:
  explicit ServiceProvider(const rclcpp::NodeOptions &options);

  /**
   * @brief Get current state value
   * @return Current boolean state
   */
  bool getState() const;

  void lockService() { locked_ = true; }

  void unlockService() { locked_ = false; }

private:
  /**
   * @brief Service callback handler
   */
  void handleServiceRequest(
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  bool state_{false};
  std::atomic_bool locked_{false};
};

}  // namespace test_composition