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
// @file      action_client_mock.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <gmock/gmock.h>
#include <rtest/static_registry.hpp>

#include "rtest/ros_versions.hpp"
#include "rosidl_typesupport_cpp/action_type_support.hpp"

// In the latest ROS2 distributions (such as Kilted), action ClientBase has been correctly separated from the action Client class.
#if RTEST_ROS_VERSION <= RTEST_ROS_JAZZY
#include <rtest/action_client_base.hpp>
#else
#include "rclcpp_action/client_base.hpp"
#endif

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include <future>

#include "rcl_action/action_client.h"
#include "rcl_action/types.h"

#include "rclcpp/macros.hpp"
#include "rclcpp_action/exceptions.hpp"

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__)      \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__)        \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__)

// Forward declarations
namespace rtest
{
namespace experimental
{
template <typename ActionT>
class ActionClientMock;
}  // namespace experimental
}  // namespace rtest

namespace rclcpp_action
{
/// The possible statuses that an action goal can finish with.
enum class ResultCode : int8_t
{
  UNKNOWN = action_msgs::msg::GoalStatus::STATUS_UNKNOWN,
  SUCCEEDED = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
  CANCELED = action_msgs::msg::GoalStatus::STATUS_CANCELED,
  ABORTED = action_msgs::msg::GoalStatus::STATUS_ABORTED
};

template <typename ActionT>
class ClientGoalHandle
{
public:
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using Goal = typename ActionT::Goal;
  using SharedPtr = std::shared_ptr<ClientGoalHandle>;
  using WeakPtr = std::weak_ptr<ClientGoalHandle>;

  struct WrappedResult
  {
    ResultCode code;
    std::shared_ptr<Result> result;
    GoalUUID goal_id;
  };

  using FeedbackCallback = std::function<void(SharedPtr, std::shared_ptr<const Feedback>)>;
  using ResultCallback = std::function<void(const WrappedResult &)>;

  FeedbackCallback feedback_callback;
  ResultCallback result_callback;

  const GoalUUID & get_goal_id() const { return goal_id_; }
  rclcpp::Time get_goal_stamp() const { return time_stamp_; }
  int8_t get_status();
  bool is_feedback_aware() { return feedback_callback != nullptr; }
  bool is_result_aware() { return is_result_aware_; }

  bool is_invalidated() const { return false; }
  std::shared_future<WrappedResult> async_get_result()
  {
    std::promise<WrappedResult> p;
    WrappedResult result;
    result.code = ResultCode::SUCCEEDED;
    result.result = std::make_shared<Result>();
    result.goal_id = goal_id_;
    p.set_value(result);
    return p.get_future().share();
  }
  bool set_result_awareness(bool aware)
  {
    bool previous = is_result_aware_;
    is_result_aware_ = aware;
    return previous;
  }
  void set_result(const WrappedResult & result) { (void)result; }
  void invalidate(const exceptions::UnawareGoalHandleError & ex) { (void)ex; }
  void set_status(int8_t status) { status_ = status; }
  void set_goal_id(const GoalUUID & goal_id) { goal_id_ = goal_id; }
  void set_goal_stamp(const rclcpp::Time & stamp) { time_stamp_ = stamp; }

private:
  GoalUUID goal_id_;
  rclcpp::Time time_stamp_;
  int8_t status_{0};
  bool is_result_aware_{false};
};

template <typename ActionT>
auto makeClientGoalHandle()
{
  return std::make_shared<rclcpp_action::ClientGoalHandle<ActionT>>();
}

template <typename ActionT>
auto makeClientGoalHandleFuture(
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = nullptr)
{
  std::promise<decltype(goal_handle)> promise{};
  promise.set_value(goal_handle);
  return promise.get_future().share();
}

template <typename ActionT>
class Client : public ClientBase, public std::enable_shared_from_this<Client<ActionT>>
{
public:
  TEST_TOOLS_SMART_PTR_DEFINITIONS(Client<ActionT>)
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void(typename GoalHandle::SharedPtr)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;
  using CancelCallback = std::function<void(typename CancelResponse::SharedPtr)>;

  struct SendGoalOptions
  {
    SendGoalOptions()
    : goal_response_callback(nullptr), feedback_callback(nullptr), result_callback(nullptr)
    {
    }
    GoalResponseCallback goal_response_callback;
    FeedbackCallback feedback_callback;
    ResultCallback result_callback;

    void call_feedback_callback(std::shared_ptr<const Feedback> feedback)
    {
      if (feedback_callback) {
        feedback_callback(nullptr, feedback);
      }
    }
  };

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rcl_action_client_options_t & options)
  :
#if RTEST_ROS_VERSION >= RTEST_ROS_KILTED
    ClientBase(
      node_base,
      node_graph,
      node_logging,
      action_name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
      options),
#endif
    node_base_(node_base),
    action_name_(action_name)
  {
    (void)node_graph;
    (void)node_logging;
    (void)options;
    rtest::StaticMocksRegistry::instance().registerLazyInitClient(
      this, node_base_->get_fully_qualified_name(), action_name_, [this]() {
        this->post_init_setup();
      });
  }

  ~Client() { rtest::StaticMocksRegistry::instance().removeLazyInitClient(this); }

  void post_init_setup()
  {
    rtest::StaticMocksRegistry::instance().registerActionClient<ActionT>(
      node_base_->get_fully_qualified_name(), action_name_, this->shared_from_this());
  }

  bool action_server_is_ready() const
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(const_cast<Client *>(this)).lock();
    if (mock) {
      return std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->action_server_is_ready();
    }
    return false;
  }

  std::shared_future<WrappedResult> async_get_result(
    typename GoalHandle::SharedPtr & goal_handle,
    ResultCallback result_callback = nullptr)
  {
    if (result_callback) {
      goal_handle->set_result_callback(result_callback);
    }
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->async_get_result(goal_handle, result_callback);
    }
    std::promise<WrappedResult> promise;
    WrappedResult result;
    result.code = ResultCode::SUCCEEDED;
    result.result = std::make_shared<Result>();
    promise.set_value(result);
    return promise.get_future().share();
  }

  std::shared_future<typename GoalHandle::SharedPtr> async_send_goal(
    const Goal & goal,
    const SendGoalOptions & options)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      auto f = std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
                 ->async_send_goal(goal, options);
      if (f.valid()) {
        return f;
      }
    }

    return makeClientGoalHandleFuture<ActionT>(nullptr);  // reject goal by default
  }

  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_all_goals(
    CancelCallback cancel_callback = nullptr)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->async_cancel_all_goals(cancel_callback);
    }
    std::promise<typename CancelResponse::SharedPtr> promise;
    CancelResponse result;
    result.code = ResultCode::SUCCEEDED;
    result.result = std::make_shared<Result>();
    promise.set_value(result);
    return promise.get_future().share();
  }

  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_goal(
    typename GoalHandle::SharedPtr goal_handle,
    CancelCallback cancel_callback = nullptr)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->async_cancel_goal(goal_handle, cancel_callback);
    }
    std::promise<typename CancelResponse::SharedPtr> promise;
    auto response = std::make_shared<CancelResponse>();
    promise.set_value(response);
    return promise.get_future().share();
  }

  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_goals_before(
    const rclcpp::Time & stamp,
    CancelCallback cancel_callback = nullptr)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->async_cancel_goals_before(stamp, cancel_callback);
    }
    std::promise<typename CancelResponse::SharedPtr> promise;
    auto response = std::make_shared<CancelResponse>();
    promise.set_value(response);
    return promise.get_future().share();
  }

  void stop_callbacks(typename GoalHandle::SharedPtr goal_handle)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->stop_callbacks(goal_handle);
      return;
    }
    if (goal_handle) {
      goal_handle->set_feedback_callback(FeedbackCallback());
      goal_handle->set_result_callback(ResultCallback());
    }
    std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.find(goal_handle->get_goal_id());
    if (it != goal_handles_.end()) {
      goal_handles_.erase(it);
    }
  }

  void stop_callbacks(const GoalUUID & goal_id)
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock)
        ->stop_callbacks(goal_id);
      return;
    }
    GoalHandleSharedPtr goal_handle;
    {
      std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
      auto it = goal_handles_.find(goal_id);
      if (it != goal_handles_.end()) {
        goal_handle = it->second.lock();
      }
    }
    if (goal_handle) {
      stop_callbacks(goal_handle);
    }
  }

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    auto mock = rtest::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      auto action_mock =
        std::static_pointer_cast<rtest::experimental::ActionClientMock<ActionT>>(mock);
      return action_mock->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
    }
    return true;
  }

#if RTEST_ROS_VERSION >= RTEST_ROS_KILTED
private:
  std::shared_ptr<void> create_goal_response() const override
  {
    using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
    return std::shared_ptr<void>(new GoalResponse());
  }

  std::shared_ptr<void> create_result_response() const override
  {
    using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
    return std::shared_ptr<void>(new GoalResultResponse());
  }

  std::shared_ptr<void> create_cancel_response() const override
  {
    return std::shared_ptr<void>(new CancelResponse());
  }

  std::shared_ptr<void> create_feedback_message() const override
  {
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    return std::shared_ptr<void>(new FeedbackMessage());
  }

  void handle_feedback_message(std::shared_ptr<void> message) override { (void)message; }

  std::shared_ptr<void> create_status_message() const override
  {
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  void handle_status_message(std::shared_ptr<void> message) override { (void)message; }
#endif

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::string action_name_;
  std::map<GoalUUID, typename GoalHandle::WeakPtr> goal_handles_;
  std::recursive_mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

namespace rtest
{
namespace experimental
{

template <typename ActionT>
class ActionClientMock : public MockBase
{
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using ResultCallback = typename GoalHandle::ResultCallback;
  using SendGoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;
  using CancelCallback = std::function<void(typename CancelResponse::SharedPtr)>;

  explicit ActionClientMock(rclcpp_action::ClientBase * client) : client_(client) {}
  ~ActionClientMock() { StaticMocksRegistry::instance().detachMock(client_); }
  TEST_TOOLS_SMART_PTR_DEFINITIONS(ActionClientMock<ActionT>)

  MOCK_METHOD(
    std::shared_future<GoalHandleSharedPtr>,
    async_send_goal,
    (const Goal &, const SendGoalOptions &),
    ());
  MOCK_METHOD(
    std::shared_future<WrappedResult>,
    async_get_result,
    (GoalHandleSharedPtr, ResultCallback),
    ());
  MOCK_METHOD(
    std::shared_future<typename CancelResponse::SharedPtr>,
    async_cancel_goal,
    (GoalHandleSharedPtr, CancelCallback),
    ());
  MOCK_METHOD(
    std::shared_future<typename CancelResponse::SharedPtr>,
    async_cancel_all_goals,
    (CancelCallback),
    ());
  MOCK_METHOD(bool, action_server_is_ready, (), ());
  MOCK_METHOD(void, stop_callbacks, (GoalHandleSharedPtr), ());
  MOCK_METHOD(void, stop_callbacks, (const rclcpp_action::GoalUUID &), ());
  MOCK_METHOD(
    std::shared_future<typename CancelResponse::SharedPtr>,
    async_cancel_goals_before,
    (const rclcpp::Time &, CancelCallback),
    ());

  MOCK_METHOD(bool, wait_for_action_server, (std::chrono::nanoseconds), ());

  // Template wrapper that converts all duration types to nanoseconds
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_action_server(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  // Convenience method to create ClientGoalHandle
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> makeClientGoalHandle()
  {
    return rclcpp_action::makeClientGoalHandle<ActionT>();
  }

  // Convenience method to create ClientGoalHandleFuture
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>
  makeClientGoalHandleFuture(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = nullptr)
  {
    return rclcpp_action::makeClientGoalHandleFuture<ActionT>(goal_handle);
  }

  void simulate_result(const GoalHandleSharedPtr & goal_handle, const WrappedResult & result)
  {
    if (goal_handle && goal_handle->result_callback) {
      goal_handle->result_callback(result);
    }
  }

private:
  rclcpp_action::ClientBase * client_{nullptr};
};

template <typename ActionT>
std::shared_ptr<ActionClientMock<ActionT>> findActionClient(
  const std::string & fullyQualifiedNodeName,
  const std::string & actionName)
{
  std::shared_ptr<ActionClientMock<ActionT>> client_mock{};
  auto client_base =
    StaticMocksRegistry::instance().getActionClient(fullyQualifiedNodeName, actionName).lock();
  if (client_base) {
    if (StaticMocksRegistry::instance().getMock(client_base.get()).lock()) {
      std::cerr << "WARNING: ActionClientMock already attached\n";
    } else {
      client_mock = std::make_shared<ActionClientMock<ActionT>>(client_base.get());
      StaticMocksRegistry::instance().attachMock(client_base.get(), client_mock);
    }
  }
  return client_mock;
}

template <typename ActionT, typename NodeT>
std::shared_ptr<ActionClientMock<ActionT>> findActionClient(
  const std::shared_ptr<NodeT> & node,
  const std::string & actionName)
{
  return findActionClient<ActionT>(node->get_fully_qualified_name(), actionName);
}

// Expose rclcpp_action::makeClientGoalHandle<ActionT> in the rtest namespace
template <typename ActionT>
std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> makeClientGoalHandle()
{
  return rclcpp_action::makeClientGoalHandle<ActionT>();
}

// Custom action for async_send_goal() return value
ACTION_P(ReturnGoalHandleFuture, goal_handle)
{
  std::promise<decltype(goal_handle)> promise{};
  promise.set_value(goal_handle);
  return promise.get_future().share();
}

}  // namespace experimental
}  // namespace rtest
