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
// @file      action_server_mock.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <gmock/gmock.h>
#include <rtest/static_registry.hpp>
#include <rtest/action_server_base.hpp>
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"

#include "rcl_action/action_server.h"
#include "rcl_action/types.h"
#include "rclcpp/macros.hpp"

#include <array>

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__)      \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__)        \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__)

namespace rclcpp_action
{

template <typename ActionT>
class ServerGoalHandle
{
public:
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using Goal = typename ActionT::Goal;

  std::shared_ptr<const Goal> get_goal() const { return goal_; }
  bool is_canceling() const { return canceling_; }
  void set_canceling(bool canceling = true) { canceling_ = canceling; }
  bool is_active() const { return !canceling_; }

  void publish_feedback(std::shared_ptr<const Feedback> feedback) { (void)feedback; }
  void succeed(std::shared_ptr<Result> result) { (void)result; }
  void abort(std::shared_ptr<Result> result) { (void)result; }
  void canceled(std::shared_ptr<Result> result) { (void)result; }

private:
  std::shared_ptr<const Goal> goal_;
  bool canceling_ = false;
};

template <typename ActionT>
class Server : public ServerBase, public std::enable_shared_from_this<Server<ActionT>>
{
public:
  TEST_TOOLS_SMART_PTR_DEFINITIONS(Server<ActionT>)

  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using GoalResponse = rclcpp_action::GoalResponse;
  using CancelResponse = rclcpp_action::CancelResponse;

  using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const Goal>)>;
  using CancelCallback = std::function<CancelResponse(GoalHandleSharedPtr)>;
  using AcceptedCallback = std::function<void(GoalHandleSharedPtr)>;

  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rcl_action_server_options_t & options,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted)
  : node_base_(node_base),
    action_name_(name),
    handle_goal_(handle_goal),
    handle_cancel_(handle_cancel),
    handle_accepted_(handle_accepted)
  {
    (void)node_clock;
    (void)node_logging;
    (void)options;
    rtest::StaticMocksRegistry::instance().registerLazyInitServer(
      this, node_base_->get_fully_qualified_name(), action_name_, [this]() {
        this->post_init_setup();
      });
  }

  ~Server() { rtest::StaticMocksRegistry::instance().removeLazyInitServer(this); }

  void post_init_setup()
  {
    rtest::StaticMocksRegistry::instance().registerActionServer<ActionT>(
      node_base_->get_fully_qualified_name(), action_name_, this->shared_from_this());
  }

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::string action_name_;
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  AcceptedCallback handle_accepted_;
};

}  // namespace rclcpp_action

namespace rtest
{

template <typename ActionT>
class ActionServerMock : public MockBase
{
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using GoalResponse = rclcpp_action::GoalResponse;
  using CancelResponse = rclcpp_action::CancelResponse;

  explicit ActionServerMock(rclcpp_action::ServerBase * server) : server_(server) {}
  ~ActionServerMock() { StaticMocksRegistry::instance().detachMock(server_); }

  TEST_TOOLS_SMART_PTR_DEFINITIONS(ActionServerMock<ActionT>)

  // Core mocks
  MOCK_METHOD(
    GoalResponse,
    handle_goal,
    (const rclcpp_action::GoalUUID &, std::shared_ptr<const Goal>),
    ());

  MOCK_METHOD(CancelResponse, handle_cancel, (const GoalHandleSharedPtr &), ());

  MOCK_METHOD(void, handle_accepted, (const GoalHandleSharedPtr &), ());

  // Helper methods for testing
  void publish_feedback(
    const typename ActionT::Feedback & feedback,
    const GoalHandleSharedPtr & goal_handle)
  {
    if (goal_handle) {
      auto feedback_ptr = std::make_shared<const typename ActionT::Feedback>(feedback);
      goal_handle->publish_feedback(feedback_ptr);
    }
  }

  void succeed(const typename ActionT::Result & result, const GoalHandleSharedPtr & goal_handle)
  {
    if (goal_handle) {
      goal_handle->succeed(std::make_shared<typename ActionT::Result>(result));
    }
  }

  void abort(const typename ActionT::Result & result, const GoalHandleSharedPtr & goal_handle)
  {
    if (goal_handle) {
      goal_handle->abort(std::make_shared<typename ActionT::Result>(result));
    }
  }

  void canceled(const typename ActionT::Result & result, const GoalHandleSharedPtr & goal_handle)
  {
    if (goal_handle) {
      goal_handle->canceled(std::make_shared<typename ActionT::Result>(result));
    }
  }

private:
  rclcpp_action::ServerBase * server_{nullptr};
};

template <typename ActionT>
std::shared_ptr<ActionServerMock<ActionT>> findActionServer(
  const std::string & fullyQualifiedNodeName,
  const std::string & actionName)
{
  std::shared_ptr<ActionServerMock<ActionT>> server_mock{};
  auto server_base =
    StaticMocksRegistry::instance().getActionServer(fullyQualifiedNodeName, actionName).lock();

  if (server_base) {
    if (StaticMocksRegistry::instance().getMock(server_base.get()).lock()) {
      std::cerr << "WARNING: ActionServerMock already attached\n";
    } else {
      server_mock = std::make_shared<ActionServerMock<ActionT>>(server_base.get());
      StaticMocksRegistry::instance().attachMock(server_base.get(), server_mock);
    }
  }
  return server_mock;
}

template <typename ActionT, typename NodeT>
std::shared_ptr<ActionServerMock<ActionT>> findActionServer(
  const std::shared_ptr<NodeT> & node,
  const std::string & actionName)
{
  return findActionServer<ActionT>(node->get_fully_qualified_name(), actionName);
}

}  // namespace rtest