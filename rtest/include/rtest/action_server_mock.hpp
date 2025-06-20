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

  explicit ServerGoalHandle(std::shared_ptr<const Goal> goal) : goal_(goal) {}
  ServerGoalHandle() = default;
  virtual ~ServerGoalHandle() = default;

  virtual bool is_canceling() const { return canceling_; }
  virtual bool is_active() const { return !canceling_; }
  virtual bool is_executing() const { return executing_; }

  void set_canceling(bool canceling = true) { canceling_ = canceling; }
  void set_executing(bool executing = true) { executing_ = executing; }
  void set_goal(std::shared_ptr<const Goal> goal) { goal_ = goal; }
  void set_goal_id(const GoalUUID & uuid) { uuid_ = uuid; }

  virtual void publish_feedback(std::shared_ptr<Feedback> feedback) { (void)feedback; }
  virtual void succeed(typename ActionT::Result::SharedPtr result_msg) { (void)result_msg; }
  virtual void abort(typename ActionT::Result::SharedPtr result_msg) { (void)result_msg; }
  virtual void canceled(typename ActionT::Result::SharedPtr result_msg) { (void)result_msg; }
  virtual void execute() {}
  const GoalUUID & get_goal_id() const { return uuid_; }
  const std::shared_ptr<const typename ActionT::Goal> get_goal() const { return goal_; }

private:
  std::shared_ptr<const Goal> goal_{nullptr};
  GoalUUID uuid_{};
  bool canceling_ = false;
  bool executing_ = false;
};

template <typename ActionT>
class GoalHandleMock : public rclcpp_action::ServerGoalHandle<ActionT>
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;

  TEST_TOOLS_SMART_PTR_DEFINITIONS(GoalHandleMock<ActionT>)

  GoalHandleMock(std::shared_ptr<const Goal> goal) : rclcpp_action::ServerGoalHandle<ActionT>(goal)
  {
  }

  MOCK_METHOD(void, publish_feedback, (std::shared_ptr<Feedback>), (override));
  MOCK_METHOD(void, succeed, (typename ActionT::Result::SharedPtr), (override));
  MOCK_METHOD(void, abort, (typename ActionT::Result::SharedPtr), (override));
  MOCK_METHOD(void, canceled, (typename ActionT::Result::SharedPtr), (override));
  MOCK_METHOD(void, execute, (), (override));
  MOCK_METHOD(GoalUUID, get_goal_id, (), (const));
  MOCK_METHOD(std::shared_ptr<const typename ActionT::Goal>, get_goal, (), (const));

  MOCK_METHOD(bool, is_canceling, (), (override, const));
  MOCK_METHOD(bool, is_active, (), (override, const));
  MOCK_METHOD(bool, is_executing, (), (override, const));
};

template <typename ActionT>
class Server : public ServerBase, public std::enable_shared_from_this<Server<ActionT>>
{
public:
  TEST_TOOLS_SMART_PTR_DEFINITIONS(Server<ActionT>)

  using GoalCallback =
    std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  using AcceptedCallback = std::function<void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;

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

  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  AcceptedCallback handle_accepted_;

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::string action_name_;
};

}  // namespace rclcpp_action

namespace rtest
{
namespace experimental
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
  using GoalCallback = typename rclcpp_action::Server<ActionT>::GoalCallback;
  using CancelCallback = typename rclcpp_action::Server<ActionT>::CancelCallback;
  using AcceptedCallback = typename rclcpp_action::Server<ActionT>::AcceptedCallback;

  explicit ActionServerMock(rclcpp_action::ServerBase * server)
  {
    if (server == nullptr) {
      throw std::invalid_argument{"ActionServerMock error: server is null"};
    }
    server_ = dynamic_cast<rclcpp_action::Server<ActionT> *>(server);
    if (server_ == nullptr) {
      throw std::invalid_argument{
        "ActionServerMock error: server is type with invalid ActionT type"};
    }

    goal_callback = server_->handle_goal_;
    cancel_callback = server_->handle_cancel_;
    accepted_callback = server_->handle_accepted_;
  }
  ~ActionServerMock() { StaticMocksRegistry::instance().detachMock(server_); }

  TEST_TOOLS_SMART_PTR_DEFINITIONS(ActionServerMock<ActionT>)

  GoalCallback goal_callback;
  CancelCallback cancel_callback;
  AcceptedCallback accepted_callback;

private:
  rclcpp_action::Server<ActionT> * server_{nullptr};
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

template <typename ActionT>
std::shared_ptr<rclcpp_action::GoalHandleMock<ActionT>> createMockGoalHandle(
  std::shared_ptr<const typename ActionT::Goal> goal)
{
  return std::make_shared<rclcpp_action::GoalHandleMock<ActionT>>(goal);
}

template <typename ActionT, typename NodeT>
std::shared_ptr<ActionServerMock<ActionT>> findActionServer(
  const std::shared_ptr<NodeT> & node,
  const std::string & actionName)
{
  return findActionServer<ActionT>(node->get_fully_qualified_name(), actionName);
}
}  // namespace experimental
}  // namespace rtest