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
  auto client_mock = rtest::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  EXPECT_CALL(*client_mock, action_server_is_ready()).WillRepeatedly(::testing::Return(true));

  /// Check specific goal values inside lambda
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([](const auto & goal, const auto & options) {
      /// Check if goal contains expected values
      EXPECT_FLOAT_EQ(goal.target_x, 2.0f);
      EXPECT_FLOAT_EQ(goal.target_y, 3.0f);

      (void)options;
      std::promise<
        std::shared_ptr<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>>
        promise;
      auto goal_handle =
        std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();
      promise.set_value(goal_handle);
      return promise.get_future().share();
    });

  EXPECT_TRUE(client_mock->action_server_is_ready());
  node->send_goal(2.0, 3.0);
}

TEST_F(ActionClientTest, ReceiveFeedback)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);
  /// Create mock goal handle
  auto goal_handle =
    std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();

  /// Simulate feedback
  rtest_examples::action::MoveRobot::Feedback feedback;
  feedback.current_x = 1.0;
  feedback.current_y = 2.0;
  feedback.distance_remaining = 1.5;

  EXPECT_NO_THROW(client_mock->simulate_feedback(goal_handle, feedback));
}

TEST_F(ActionClientTest, ReceiveResult)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);
  auto goal_handle =
    std::make_shared<rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>>();

  /// Create successful result
  rclcpp_action::ClientGoalHandle<rtest_examples::action::MoveRobot>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = std::make_shared<rtest_examples::action::MoveRobot::Result>();
  result.result->success = true;
  result.result->final_x = 2.0;
  result.result->final_y = 3.0;
  result.result->message = "Target reached";
  EXPECT_NO_THROW(client_mock->simulate_result(goal_handle, result));
}

TEST_F(ActionClientTest, ReceiveCanceledResult)
{
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
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
  auto client_mock = rtest::findActionClient<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(client_mock);

  /// Mock server not ready
  EXPECT_CALL(*client_mock, action_server_is_ready()).WillOnce(::testing::Return(false));

  EXPECT_FALSE(client_mock->action_server_is_ready());
}