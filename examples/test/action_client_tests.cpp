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
    rtest::experimental::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  /// Check specific goal values inside lambda
  EXPECT_CALL(
    *client_mock,
    async_send_goal(
      ::testing::AllOf(
        ::testing::Field(&rtest_examples::action::MoveRobot::Goal::target_x, 2.0f),
        ::testing::Field(&rtest_examples::action::MoveRobot::Goal::target_y, 3.0f)),
      ::testing::_))
    .Times(1)
    .WillOnce(::testing::Invoke([](const auto & /*goal*/, const auto & /*options*/) {
      std::promise<
        std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>>
        promise;
      auto goal_handle =
        std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();
      promise.set_value(goal_handle);
      return promise.get_future().share();
    }));

  EXPECT_TRUE(client_mock->action_server_is_ready());
  node->send_goal(2.0, 3.0);
}

TEST_F(ActionClientTest, ReceiveFeedback)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  // Initially no feedback should be received
  EXPECT_FALSE(node->has_received_feedback());

  // Store the goal handle that will be created with real callbacks
  std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>
    stored_goal_handle;

  // Mock async_send_goal to capture the real callbacks from the node
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([&stored_goal_handle](const auto & goal, const auto & options) {
      (void)goal;
      // Create goal handle and set up the REAL callbacks from the node
      auto goal_handle =
        std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();
      goal_handle->feedback_callback = options.feedback_callback;
      goal_handle->result_callback = options.result_callback;
      stored_goal_handle = goal_handle;

      std::promise<
        std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>>
        promise;
      promise.set_value(goal_handle);
      return promise.get_future().share();
    });

  // Send goal through the real node - this sets up the real callbacks
  node->send_goal(2.0, 3.0);

  // Verify we captured the goal handle with real callbacks
  ASSERT_TRUE(stored_goal_handle);
  ASSERT_TRUE(stored_goal_handle->feedback_callback);

  // Create feedback with specific test values
  rtest_examples::action::MoveRobot::Feedback feedback;
  feedback.current_x = 1.5;
  feedback.current_y = 2.5;
  feedback.distance_remaining = 3.0;

  // Simulate feedback - this calls the REAL node's feedback_callback
  client_mock->simulate_feedback(stored_goal_handle, feedback);

  // Now verify the node's state was updated correctly
  EXPECT_TRUE(node->has_received_feedback());

  auto received_feedback = node->get_last_feedback();
  EXPECT_FLOAT_EQ(received_feedback.current_x, 1.5);
  EXPECT_FLOAT_EQ(received_feedback.current_y, 2.5);
  EXPECT_FLOAT_EQ(received_feedback.distance_remaining, 3.0);
}

TEST_F(ActionClientTest, ReceiveResult)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  // Initially no result should be set (assuming default values)
  EXPECT_FALSE(node->get_last_result_success());
  EXPECT_TRUE(node->get_last_result_message().empty());

  // Store the goal handle that will be created with real callbacks
  std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>
    stored_goal_handle;

  // Mock async_send_goal to capture the real callbacks from the node
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([&stored_goal_handle](const auto & goal, const auto & options) {
      (void)goal;
      // Create goal handle and set up the REAL callbacks from the node
      auto goal_handle =
        std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();
      // Only set callbacks that actually exist in ClientGoalHandle
      goal_handle->feedback_callback = options.feedback_callback;
      goal_handle->result_callback = options.result_callback;
      stored_goal_handle = goal_handle;

      std::promise<
        std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>>
        promise;
      promise.set_value(goal_handle);
      return promise.get_future().share();
    });

  // Send goal through the real node - this sets up the real callbacks
  node->send_goal(2.0, 3.0);

  // Verify we captured the goal handle with real callbacks
  ASSERT_TRUE(stored_goal_handle);
  ASSERT_TRUE(stored_goal_handle->result_callback);

  // Create successful result with specific test values
  rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = std::make_shared<rtest_examples::action::MoveRobot::Result>();
  result.result->success = true;
  result.result->final_x = 2.5;
  result.result->final_y = 3.5;
  result.result->message = "Target reached successfully";

  // Simulate result - this calls the REAL node's result_callback
  client_mock->simulate_result(stored_goal_handle, result);

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
    rtest::experimental::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  /// Store reference to goal handle to use later
  std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>
    stored_goal_handle;

  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([&stored_goal_handle](const auto & goal, const auto & options) {
      (void)goal;

      /// Create goal handle and set up callbacks
      auto goal_handle =
        std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();
      goal_handle->result_callback = options.result_callback;
      stored_goal_handle = goal_handle;

      std::promise<
        std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>>
        promise;
      promise.set_value(goal_handle);
      return promise.get_future().share();
    });

  /// Send goal to register callbacks
  node->send_goal(2.0, 3.0);

  /// Create canceled result
  rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::CANCELED;
  result.result = std::make_shared<rtest_examples::action::MoveRobot::Result>();
  result.result->success = false;

  // Use the stored goal handle with callback
  ASSERT_TRUE(stored_goal_handle);
  ASSERT_TRUE(stored_goal_handle->result_callback);

  /// Manually trigger the result callback
  stored_goal_handle->result_callback(result);

  /// Check that the client properly handled the canceled result
  EXPECT_FALSE(node->get_last_result_success());
  EXPECT_EQ(node->get_last_result_message(), "Goal canceled");
}

TEST_F(ActionClientTest, ServerNotReady)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock =
    rtest::experimental::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  /// Mock server not ready
  EXPECT_CALL(*client_mock, action_server_is_ready()).WillOnce(::testing::Return(false));

  EXPECT_FALSE(client_mock->action_server_is_ready());
}