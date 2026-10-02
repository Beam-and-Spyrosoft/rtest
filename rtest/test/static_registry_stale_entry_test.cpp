// Copyright 2026 Spyrosoft Limited.
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
// @file      static_registry_stale_entry_test.cpp
// @date      2026-09-07
//
// @brief     Reproduces StaticMocksRegistry returning a previous test's entity
//            when a new node reuses the same fully-qualified name while the
//            previous node is still alive (GitHub issue #126).

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rtest/static_registry.hpp>
#include <rtest/test_clock.hpp>

#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace
{
using Fibonacci = example_interfaces::action::Fibonacci;

rclcpp::NodeOptions test_node_options()
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  options.enable_rosout(false);
  return options;
}

std::shared_ptr<rclcpp::Node> make_node(const std::string & name)
{
  return std::make_shared<rclcpp::Node>(name, test_node_options());
}

struct ActionCounts
{
  int goals = 0;
  int accepted = 0;
};

std::shared_ptr<rclcpp_action::Server<Fibonacci>> create_counting_action_server(
  const std::shared_ptr<rclcpp::Node> & node,
  ActionCounts & counts)
{
  return rclcpp_action::create_server<Fibonacci>(
    node,
    "fibonacci",
    [&counts](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      ++counts.goals;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> &) {
      return rclcpp_action::CancelResponse::REJECT;
    },
    [&counts](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> &) {
      ++counts.accepted;
    });
}
}  // namespace

// Keep two same-named nodes alive in one test to model overlapping lifetimes
// between test cases deterministically, without threads or timing assumptions.
class StaticRegistryStaleEntry : public ::testing::Test
{
protected:
  void SetUp() override { rtest::StaticMocksRegistry::instance().reset(); }

  void TearDown() override { rtest::StaticMocksRegistry::instance().reset(); }
};

TEST_F(StaticRegistryStaleEntry, FindPublisherReturnsNewEntityAfterPreviousDestroyed)
{
  auto previous = make_node("publisher_node");
  auto previous_pub = previous->create_publisher<std_msgs::msg::String>("topic", 10);
  ASSERT_TRUE((rtest::findPublisher<std_msgs::msg::String>(previous, "topic")));
  previous_pub.reset();
  previous.reset();

  auto current = make_node("publisher_node");
  auto current_pub = current->create_publisher<std_msgs::msg::String>("topic", 10);
  auto found_mock = rtest::findPublisher<std_msgs::msg::String>(current, "topic");
  ASSERT_TRUE(found_mock);

  std_msgs::msg::String msg;
  msg.data = "from_current";
  EXPECT_CALL(*found_mock, publish(msg)).Times(1);
  current_pub->publish(msg);
}

TEST_F(StaticRegistryStaleEntry, FindPublisherReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  auto previous = make_node("publisher_node");
  auto previous_pub = previous->create_publisher<std_msgs::msg::String>("topic", 10);
  ASSERT_TRUE(previous_pub);
  auto previous_mock = rtest::findPublisher<std_msgs::msg::String>(previous, "topic");
  ASSERT_TRUE(previous_mock);
  previous_mock.reset();

  auto current = make_node("publisher_node");
  auto current_pub = current->create_publisher<std_msgs::msg::String>("topic", 10);
  auto found_mock = rtest::findPublisher<std_msgs::msg::String>(current, "topic");
  ASSERT_TRUE(found_mock);

  std_msgs::msg::String msg;
  msg.data = "from_current";
  EXPECT_CALL(*found_mock, publish(msg)).Times(1);
  current_pub->publish(msg);

  auto previous_again = rtest::findPublisher<std_msgs::msg::String>(previous, "topic");
  ASSERT_TRUE(previous_again);
  EXPECT_CALL(*previous_again, publish(msg)).Times(1);
  previous_pub->publish(msg);
}

TEST_F(StaticRegistryStaleEntry, FindSubscriptionReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  int previous_msgs = 0;
  int current_msgs = 0;

  auto previous = make_node("subscription_node");
  auto previous_sub = previous->create_subscription<std_msgs::msg::String>(
    "topic", 10, [&previous_msgs](std_msgs::msg::String::UniquePtr) { ++previous_msgs; });
  ASSERT_TRUE(previous_sub);
  ASSERT_TRUE((rtest::findSubscription<std_msgs::msg::String>(previous, "topic")));

  auto current = make_node("subscription_node");
  auto current_sub = current->create_subscription<std_msgs::msg::String>(
    "topic", 10, [&current_msgs](std_msgs::msg::String::UniquePtr) { ++current_msgs; });
  ASSERT_TRUE(current_sub);

  auto found = rtest::findSubscription<std_msgs::msg::String>(current, "topic");
  ASSERT_TRUE(found);

  std_msgs::msg::String msg;
  msg.data = "ping";
  found->handle_message(msg);

  EXPECT_EQ(current_msgs, 1);
  EXPECT_EQ(previous_msgs, 0);

  auto previous_again = rtest::findSubscription<std_msgs::msg::String>(previous, "topic");
  ASSERT_TRUE(previous_again);
  previous_again->handle_message(msg);
  EXPECT_EQ(previous_msgs, 1);
  EXPECT_EQ(current_msgs, 1);
}

TEST_F(StaticRegistryStaleEntry, FindServiceReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  int previous_calls = 0;
  int current_calls = 0;

  auto previous = make_node("service_node");
  auto previous_service = previous->create_service<std_srvs::srv::SetBool>(
    "test_service",
    [&previous_calls](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> &,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      ++previous_calls;
      response->success = false;
    });
  ASSERT_TRUE(previous_service);
  auto previous_mock = rtest::findService<std_srvs::srv::SetBool>(previous, "test_service");
  ASSERT_TRUE(previous_mock);
  previous_mock.reset();

  auto current = make_node("service_node");
  auto current_service = current->create_service<std_srvs::srv::SetBool>(
    "test_service",
    [&current_calls](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> &,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      ++current_calls;
      response->success = true;
    });
  ASSERT_TRUE(current_service);

  auto found_mock = rtest::findService<std_srvs::srv::SetBool>(current, "test_service");
  ASSERT_TRUE(found_mock);

  auto header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  EXPECT_CALL(*found_mock, send_response(::testing::_, ::testing::_)).Times(1);
  found_mock->handle_request(header, request);

  EXPECT_EQ(current_calls, 1);
  EXPECT_EQ(previous_calls, 0);

  auto previous_again = rtest::findService<std_srvs::srv::SetBool>(previous, "test_service");
  ASSERT_TRUE(previous_again);
  EXPECT_CALL(*previous_again, send_response(::testing::_, ::testing::_)).Times(1);
  previous_again->handle_request(header, request);
  EXPECT_EQ(previous_calls, 1);
  EXPECT_EQ(current_calls, 1);
}

TEST_F(StaticRegistryStaleEntry, FindServiceClientReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  auto previous = make_node("service_client_node");
  auto previous_client = previous->create_client<std_srvs::srv::SetBool>("test_service");
  ASSERT_TRUE(previous_client);
  auto previous_mock = rtest::findServiceClient<std_srvs::srv::SetBool>(previous, "test_service");
  ASSERT_TRUE(previous_mock);
  previous_mock.reset();

  auto current = make_node("service_client_node");
  auto current_client = current->create_client<std_srvs::srv::SetBool>("test_service");
  auto found_mock = rtest::findServiceClient<std_srvs::srv::SetBool>(current, "test_service");
  ASSERT_TRUE(found_mock);

  EXPECT_CALL(*found_mock, service_is_ready()).WillOnce(::testing::Return(true));
  EXPECT_TRUE(current_client->service_is_ready());

  auto previous_again = rtest::findServiceClient<std_srvs::srv::SetBool>(previous, "test_service");
  ASSERT_TRUE(previous_again);
  EXPECT_CALL(*previous_again, service_is_ready()).WillOnce(::testing::Return(true));
  EXPECT_TRUE(previous_client->service_is_ready());
}

TEST_F(StaticRegistryStaleEntry, FindActionServerReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  ActionCounts previous_counts;
  ActionCounts current_counts;

  auto previous = make_node("action_server_node");
  auto previous_server = create_counting_action_server(previous, previous_counts);
  ASSERT_TRUE(previous_server);
  // Force registration of the previous server before creating the current one.
  auto previous_mock = rtest::experimental::findActionServer<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(previous_mock);
  previous_mock.reset();

  auto current = make_node("action_server_node");
  auto current_server = create_counting_action_server(current, current_counts);
  ASSERT_TRUE(current_server);

  auto found_mock = rtest::experimental::findActionServer<Fibonacci>(current, "fibonacci");
  ASSERT_TRUE(found_mock);
  ASSERT_TRUE(found_mock->goal_callback);

  auto goal = std::make_shared<Fibonacci::Goal>();
  const auto response = found_mock->goal_callback(rclcpp_action::GoalUUID{}, goal);

  EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  EXPECT_EQ(current_counts.goals, 1);
  EXPECT_EQ(previous_counts.goals, 0);

  auto previous_again = rtest::experimental::findActionServer<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(previous_again);
  previous_again->goal_callback(rclcpp_action::GoalUUID{}, goal);
  EXPECT_EQ(previous_counts.goals, 1);
  EXPECT_EQ(current_counts.goals, 1);
}

TEST_F(StaticRegistryStaleEntry, FindActionClientReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  auto previous = make_node("action_client_node");
  auto previous_client = rclcpp_action::create_client<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(previous_client);
  auto previous_mock = rtest::experimental::findActionClient<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(previous_mock);
  previous_mock.reset();

  auto current = make_node("action_client_node");
  auto current_client = rclcpp_action::create_client<Fibonacci>(current, "fibonacci");
  auto found_mock = rtest::experimental::findActionClient<Fibonacci>(current, "fibonacci");
  ASSERT_TRUE(found_mock);

  EXPECT_CALL(*found_mock, action_server_is_ready()).WillOnce(::testing::Return(true));
  EXPECT_TRUE(current_client->action_server_is_ready());

  auto previous_again = rtest::experimental::findActionClient<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(previous_again);
  EXPECT_CALL(*previous_again, action_server_is_ready()).WillOnce(::testing::Return(true));
  EXPECT_TRUE(previous_client->action_server_is_ready());
}

TEST_F(StaticRegistryStaleEntry, FindTimersReturnsCurrentEntityWhenPreviousNodeStillAlive)
{
  int previous_ticks = 0;
  int current_ticks = 0;

  auto previous = make_node("timer_node");
  auto previous_timer =
    previous->create_wall_timer(100ms, [&previous_ticks]() { ++previous_ticks; });
  ASSERT_TRUE(previous_timer);
  ASSERT_FALSE(rtest::findTimers(previous).empty());

  auto current = make_node("timer_node");
  auto current_timer = current->create_wall_timer(100ms, [&current_ticks]() { ++current_ticks; });
  ASSERT_TRUE(current_timer);

  auto timers = rtest::findTimers(current);
  ASSERT_EQ(timers.size(), 1u);
  timers.front()->execute_callback(nullptr);

  EXPECT_EQ(current_ticks, 1);
  EXPECT_EQ(previous_ticks, 0);

  auto previous_timers = rtest::findTimers(previous);
  ASSERT_EQ(previous_timers.size(), 1u);
  previous_timers.front()->execute_callback(nullptr);
  EXPECT_EQ(previous_ticks, 1);
  EXPECT_EQ(current_ticks, 1);
}

TEST_F(StaticRegistryStaleEntry, MissingEntitiesDoNotFallBackToAnotherSameNamedNode)
{
  auto previous = make_node("same_name");
  auto current = make_node("same_name");
  auto publisher = previous->create_publisher<std_msgs::msg::String>("topic", 10);
  auto subscription = previous->create_subscription<std_msgs::msg::String>(
    "topic", 10, [](std_msgs::msg::String::UniquePtr) {});
  auto service = previous->create_service<std_srvs::srv::SetBool>(
    "service",
    [](
      std::shared_ptr<std_srvs::srv::SetBool::Request>,
      std::shared_ptr<std_srvs::srv::SetBool::Response>) {});
  auto client = previous->create_client<std_srvs::srv::SetBool>("service");
  ActionCounts counts;
  auto server = create_counting_action_server(previous, counts);
  auto action_client = rclcpp_action::create_client<Fibonacci>(previous, "fibonacci");
  auto timer = previous->create_wall_timer(100ms, []() {});

  EXPECT_FALSE(rtest::findPublisher<std_msgs::msg::String>(current, "topic"));
  EXPECT_FALSE(rtest::findSubscription<std_msgs::msg::String>(current, "topic"));
  EXPECT_FALSE(rtest::findService<std_srvs::srv::SetBool>(current, "service"));
  EXPECT_FALSE(rtest::findServiceClient<std_srvs::srv::SetBool>(current, "service"));
  EXPECT_FALSE(rtest::experimental::findActionServer<Fibonacci>(current, "fibonacci"));
  EXPECT_FALSE(rtest::experimental::findActionClient<Fibonacci>(current, "fibonacci"));
  EXPECT_TRUE(rtest::findTimers(current).empty());
}

TEST_F(StaticRegistryStaleEntry, PendingActionServersRouteAcceptedCallbacksToTheirOwnNodes)
{
  ActionCounts previous_counts;
  ActionCounts current_counts;
  auto previous = make_node("action_node");
  auto current = make_node("action_node");
  auto previous_server = create_counting_action_server(previous, previous_counts);
  auto current_server = create_counting_action_server(current, current_counts);

  // Neither server has been looked up yet: both registrations are pending.
  auto found = rtest::experimental::findActionServer<Fibonacci>(current, "fibonacci");
  auto previous_found = rtest::experimental::findActionServer<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(found);
  ASSERT_TRUE(previous_found);
  found->accepted_callback(nullptr);
  EXPECT_EQ(current_counts.accepted, 1);
  EXPECT_EQ(previous_counts.accepted, 0);
  previous_found->accepted_callback(nullptr);
  EXPECT_EQ(current_counts.accepted, 1);
  EXPECT_EQ(previous_counts.accepted, 1);
}

TEST_F(StaticRegistryStaleEntry, PendingActionClientsRemainIndependent)
{
  auto previous = make_node("action_node");
  auto current = make_node("action_node");
  auto previous_client = rclcpp_action::create_client<Fibonacci>(previous, "fibonacci");
  auto current_client = rclcpp_action::create_client<Fibonacci>(current, "fibonacci");
  auto found = rtest::experimental::findActionClient<Fibonacci>(current, "fibonacci");
  auto previous_found = rtest::experimental::findActionClient<Fibonacci>(previous, "fibonacci");
  ASSERT_TRUE(found);
  ASSERT_TRUE(previous_found);
  EXPECT_CALL(*found, action_server_is_ready()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*previous_found, action_server_is_ready()).WillOnce(::testing::Return(false));
  EXPECT_TRUE(current_client->action_server_is_ready());
  EXPECT_FALSE(previous_client->action_server_is_ready());
}

TEST_F(StaticRegistryStaleEntry, DestroyedPendingActionsAreNotInitializedByLaterLookups)
{
  ActionCounts counts;
  auto previous = make_node("action_node");
  auto previous_server = create_counting_action_server(previous, counts);
  auto previous_client = rclcpp_action::create_client<Fibonacci>(previous, "fibonacci");
  previous_server.reset();
  previous_client.reset();
  previous.reset();

  auto current = make_node("action_node");
  EXPECT_FALSE(rtest::experimental::findActionServer<Fibonacci>(current, "fibonacci"));
  EXPECT_FALSE(rtest::experimental::findActionClient<Fibonacci>(current, "fibonacci"));
  auto server = create_counting_action_server(current, counts);
  auto client = rclcpp_action::create_client<Fibonacci>(current, "fibonacci");
  ASSERT_TRUE(rtest::experimental::findActionServer<Fibonacci>(current, "fibonacci"));
  ASSERT_TRUE(rtest::experimental::findActionClient<Fibonacci>(current, "fibonacci"));
}

TEST_F(StaticRegistryStaleEntry, TriggeringClockFindsAllAndOnlyItsNodesTimers)
{
  auto options = test_node_options();
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto previous = std::make_shared<rclcpp::Node>("timer_node", options);
  auto current = std::make_shared<rclcpp::Node>("timer_node", options);
  rtest::TriggeringTestClock clock(current);
  int previous_ticks = 0;
  int current_ticks = 0;
  auto previous_timer = rclcpp::create_timer(
    previous, previous->get_clock(), 100ms, [&previous_ticks]() { ++previous_ticks; });
  auto current_timer = rclcpp::create_timer(
    current, current->get_clock(), 100ms, [&current_ticks]() { ++current_ticks; });
  auto second_timer = rclcpp::create_timer(
    current, current->get_clock(), 100ms, [&current_ticks]() { ++current_ticks; });

  ASSERT_EQ(rtest::findTimers(current).size(), 2u);
  ASSERT_EQ(rtest::findTimers(previous).size(), 1u);
  clock.advance(100ms);
  EXPECT_EQ(current_ticks, 2);
  EXPECT_EQ(previous_ticks, 0);
}

TEST_F(StaticRegistryStaleEntry, NodeIdentityDoesNotKeepNodeAliveOrCollideOnAddressReuse)
{
  // Two ownership lifetimes deliberately use the same address. No ROS operations
  // are performed on this handle; this isolates the registry's identity contract.
  rcl_node_t storage{};
  auto first = std::shared_ptr<rcl_node_t>(&storage, [](rcl_node_t *) {});
  rtest::StaticMocksRegistry::NodeId first_id = first;
  auto node = make_node("publisher_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  auto & registry = rtest::StaticMocksRegistry::instance();
  registry.registerPublisher<std_msgs::msg::String>(first_id, "topic", publisher);
  first.reset();
  ASSERT_TRUE(first_id.expired());

  auto second = std::shared_ptr<rcl_node_t>(&storage, [](rcl_node_t *) {});
  EXPECT_TRUE(registry.getPublisher(second, "topic").expired());
}
