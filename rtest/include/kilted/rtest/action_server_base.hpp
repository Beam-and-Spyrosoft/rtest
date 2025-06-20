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
// @file      action_server_base.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <memory>
#include <functional>
#include "rclcpp/waitable.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action
{

enum class GoalResponse : int8_t
{
  REJECT = 1,
  ACCEPT_AND_EXECUTE = 2,
  ACCEPT_AND_DEFER = 3,
};

enum class CancelResponse : int8_t
{
  REJECT = 1,
  ACCEPT = 2,
};

class ServerBase : public rclcpp::Waitable
{
public:
  enum class EntityType : std::size_t
  {
    GoalService,
    ResultService,
    CancelService,
    Expired,
  };
  ServerBase() = default;
  virtual ~ServerBase() = default;

  virtual void publish_status() {}
  virtual void notify_goal_terminal_state() {}
  virtual void publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg)
  {
    (void)uuid;
    (void)result_msg;
  }
  virtual void publish_feedback(std::shared_ptr<void> feedback_msg) { (void)feedback_msg; }

  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }
  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const override { return {}; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }
  void configure_introspection(
    rclcpp::Clock::SharedPtr clock,
    const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t introspection_state)
  {
    (void)clock;
    (void)qos_service_event_pub;
    (void)introspection_state;
  }

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
};

}  // namespace rclcpp_action