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
// @file      action_client_tests.cpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#include <gtest/gtest.h>
#include <test_composition/action_client.hpp>
#include <rtest/action_client_mock.hpp>

class ActionClientTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ActionClientTest, SendGoal)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  test_composition::ActionClient::MoveRobot::Goal expected_goal{};
  expected_goal.set__target_x(2.0f);
  expected_goal.set__target_y(3.0f);

  // Just check if the async_send_goal() was called exactly onecw with expected goal values
  // (by default client will be notified that goal was rejected)
  EXPECT_CALL(*client_mock, async_send_goal(expected_goal, ::testing::_)).Times(1);

  node->send_goal(2.0, 3.0);
}

TEST_F(ActionClientTest, ReceiveFeedback)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  // Initially no feedback should be received
  EXPECT_FALSE(node->has_received_feedback());

  // Store the SendGoalOptions that client may set up with callbacks
  rclcpp_action::Client<test_composition::ActionClient::MoveRobot>::SendGoalOptions goal_opts;

  auto goal_handle = client_mock->makeClientGoalHandle();

  // Expect that client will call async_send_goal and capture the goal options
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .WillOnce(::testing::DoAll(
      ::testing::SaveArg<1>(&goal_opts), rtest::experimental::ReturnGoalHandleFuture(goal_handle)));

  // Trigger the node to send goal
  node->send_goal(2.0, 3.0);

  // Verify if the node had set up the feedback callback correctly
  ASSERT_TRUE(goal_opts.feedback_callback != nullptr);

  // Create feedback with specific test values
  auto feedback = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Feedback>();
  feedback->current_x = 1.5f;
  feedback->current_y = 2.5f;
  feedback->distance_remaining = 3.0f;

  // Simulate feedback - this calls the node's feedback callback
  goal_opts.call_feedback_callback(feedback);

  // Now verify the node's state was updated correctly
  EXPECT_TRUE(node->has_received_feedback());
  EXPECT_EQ(node->get_last_feedback(), *feedback);
}

TEST_F(ActionClientTest, ReceiveResult)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  // Initially no result should be set (assuming default values)
  EXPECT_FALSE(node->get_last_result_success());
  EXPECT_TRUE(node->get_last_result_message().empty());

  // Store the SendGoalOptions that client may set up with callbacks
  rclcpp_action::Client<test_composition::ActionClient::MoveRobot>::SendGoalOptions goal_opts;

  auto goal_handle = client_mock->makeClientGoalHandle();

  // Mock async_send_goal to capture the real callbacks from the node
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce(::testing::DoAll(
      ::testing::SaveArg<1>(&goal_opts), rtest::experimental::ReturnGoalHandleFuture(goal_handle)));

  // Send goal through the real node - this sets up the real callbacks
  node->send_goal(2.0, 3.0);

  // Verify we captured the goal handle with real callbacks
  ASSERT_TRUE(goal_opts.result_callback != nullptr);

  // Create successful result with specific test values
  rclcpp_action::ClientGoalHandle<rtest_examples_interfaces::action::MoveRobot>::WrappedResult
    result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Result>();
  result.result->success = true;
  result.result->final_x = 2.5;
  result.result->final_y = 3.5;
  result.result->message = "Target reached successfully";

  // Simulate result - this calls the REAL node's result_callback
  goal_opts.result_callback(result);

  // Now verify the node's state was updated correctly through the real callback
  EXPECT_TRUE(node->get_last_result_success());
  EXPECT_EQ(node->get_last_result_message(), "Target reached successfully");

  // Verify that the node no longer has an active goal after receiving result
  EXPECT_FALSE(node->has_active_goal());
}

TEST_F(ActionClientTest, ReceiveCanceledResult)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  // Store the SendGoalOptions that client may set up with callbacks
  rclcpp_action::Client<test_composition::ActionClient::MoveRobot>::SendGoalOptions goal_opts;

  auto goal_handle = client_mock->makeClientGoalHandle();

  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce(::testing::DoAll(
      ::testing::SaveArg<1>(&goal_opts), rtest::experimental::ReturnGoalHandleFuture(goal_handle)));

  /// Send goal to register callbacks
  node->send_goal(2.0, 3.0);

  /// Create canceled result
  rclcpp_action::ClientGoalHandle<rtest_examples_interfaces::action::MoveRobot>::WrappedResult
    result;
  result.code = rclcpp_action::ResultCode::CANCELED;
  result.result = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Result>();
  result.result->success = false;

  // Use the stored goal handle with callback
  ASSERT_TRUE(goal_opts.result_callback);

  /// Manually trigger the result callback
  goal_opts.result_callback(result);

  /// Check that the client properly handled the canceled result
  EXPECT_FALSE(node->get_last_result_success());
  EXPECT_EQ(node->get_last_result_message(), "Goal canceled");
}

TEST_F(ActionClientTest, ServerNotReady)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(client_mock);

  /// Mock server not ready
  EXPECT_CALL(*client_mock, action_server_is_ready()).WillOnce(::testing::Return(false));

  EXPECT_FALSE(client_mock->action_server_is_ready());
}
