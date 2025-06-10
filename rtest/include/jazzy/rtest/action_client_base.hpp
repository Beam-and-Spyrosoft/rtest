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
// @file      action_client_base.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <memory>
#include <functional>
#include <chrono>
#include <future>
#include "rclcpp/waitable.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action
{

class ClientBase : public rclcpp::Waitable
{
public:
  /// Enum to identify entities belonging to the action client
  enum class EntityType : std::size_t
  {
    GoalClient,
    ResultClient,
    CancelClient,
    FeedbackSubscription,
    StatusSubscription,
  };

  ClientBase() = default;
  virtual ~ClientBase() = default;

  // Basic action client interface
  virtual bool action_server_is_ready() const { return true; }
  virtual rclcpp::Logger get_logger() { return rclcpp::get_logger("action_client_mock"); }
  virtual std::shared_future<std::shared_ptr<void>> async_send_goal(
    std::shared_ptr<void> goal,
    std::shared_ptr<void> options)
  {
    (void)goal;
    (void)options;
    std::promise<std::shared_ptr<void>> p;
    p.set_value(nullptr);
    return p.get_future().share();
  }
  virtual std::shared_future<std::shared_ptr<void>> async_get_result(
    std::shared_ptr<void> goal_handle)
  {
    (void)goal_handle;
    std::promise<std::shared_ptr<void>> p;
    p.set_value(nullptr);
    return p.get_future().share();
  }
  virtual std::shared_future<std::shared_ptr<void>> async_cancel_goal(
    std::shared_ptr<void> goal_handle)
  {
    (void)goal_handle;
    std::promise<std::shared_ptr<void>> p;
    p.set_value(nullptr);
    return p.get_future().share();
  }
  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override { (void)wait_set; }
  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    (void)wait_set;
    return false;
  }
  std::shared_ptr<void> take_data() override { return nullptr; }
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return nullptr;
  }
  void execute(const std::shared_ptr<void> & data) override { (void)data; }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override { (void)callback; }
  void clear_on_ready_callback() override {}

  // Helper for convenience
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    (void)timeout;
    return true;
  }
};

}  // namespace rclcpp_action