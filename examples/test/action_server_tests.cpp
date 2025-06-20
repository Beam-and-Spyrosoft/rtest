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
// @file      action_server_tests.cpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#include <gtest/gtest.h>
#include <test_composition/action_server.hpp>
#include <rtest/action_server_mock.hpp>

class ActionServerTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ActionServerTest, CallbacksAreRegistered)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  // Verify all required callbacks are registered
  EXPECT_TRUE(server_mock->goal_callback);
  EXPECT_TRUE(server_mock->cancel_callback);
  EXPECT_TRUE(server_mock->accepted_callback);
}

TEST_F(ActionServerTest, GoalAcceptanceWithinRange)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  /// Test node's initial state
  EXPECT_FALSE(node->is_moving());
  auto [x, y] = node->get_current_position();
  EXPECT_FLOAT_EQ(x, 0.0);
  EXPECT_FLOAT_EQ(y, 0.0);

  rclcpp_action::GoalUUID uuid;

  /// Create a goal within acceptable range (distance < 10.0)
  auto goal = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Goal>();
  goal->target_x = 2.0;
  goal->target_y = 3.0;
  float expected_distance =
    std::sqrt(goal->target_x * goal->target_x + goal->target_y * goal->target_y);
  EXPECT_LT(expected_distance, 10.0);
  ASSERT_TRUE(server_mock->goal_callback);

  auto response = server_mock->goal_callback(uuid, goal);

  /// Verify the goal was accepted (distance = sqrt(4+9) ≈ 3.6 < 10.0 and not moving)
  EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(ActionServerTest, GoalRejectionWhenTooFar)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  rclcpp_action::GoalUUID uuid;

  /// Create a goal that should be rejected (distance > 10.0)
  auto goal = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Goal>();
  goal->target_x = 50.0;  /// distance = sqrt(2500+2500) = sqrt(5000) ≈ 70.7 > 10.0
  goal->target_y = 50.0;
  float expected_distance =
    std::sqrt(goal->target_x * goal->target_x + goal->target_y * goal->target_y);
  EXPECT_GT(expected_distance, 10.0);
  ASSERT_TRUE(server_mock->goal_callback);

  auto response = server_mock->goal_callback(uuid, goal);

  /// Verify the goal was rejected due to distance
  EXPECT_EQ(response, rclcpp_action::GoalResponse::REJECT);
}

TEST_F(ActionServerTest, FeedbackPublishingWithValidData)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  /// Test data: goal at (3.0, 4.0) - distance = 5.0
  auto goal = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Goal>();
  goal->target_x = 3.0;
  goal->target_y = 4.0;

  auto mock_goal_handle =
    rtest::experimental::createMockGoalHandle<rtest_examples_interfaces::action::MoveRobot>(goal);

  /// Verify initial position
  auto [initial_x, initial_y] = node->get_current_position();
  EXPECT_FLOAT_EQ(initial_x, 0.0);
  EXPECT_FLOAT_EQ(initial_y, 0.0);

  // Not doing direct argument comparison with:
  // EXPECT_CALL(*mock_goal_handle, publish_feedback(result)).Times(1);
  // Because it'd require direct float values comparison without precision for final_x and final_y
  std::shared_ptr<rtest_examples_interfaces::action::MoveRobot_Feedback> feedback;
  EXPECT_CALL(*mock_goal_handle, publish_feedback(::testing::_))
    .WillOnce(::testing::SaveArg<0>(&feedback));

  node->execute_single_step(goal, mock_goal_handle);

  /// Verify distance_remaining calculation is correct
  float expected_distance_remaining = std::sqrt(
    (goal->target_x - feedback->current_x) * (goal->target_x - feedback->current_x) +
    (goal->target_y - feedback->current_y) * (goal->target_y - feedback->current_y));
  EXPECT_NEAR(feedback->distance_remaining, expected_distance_remaining, 0.01);

  /// Verify we're moving away from origin (progress check)
  float distance_from_origin = std::sqrt(
    feedback->current_x * feedback->current_x + feedback->current_y * feedback->current_y);
  EXPECT_GT(distance_from_origin, 0.0);
}

TEST_F(ActionServerTest, GoalSuccessWhenTargetReached)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  // Create a goal very close to origin so it's immediately reachable
  auto goal = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Goal>();
  goal->target_x = 0.05;  // Very close to current position (0,0)
  goal->target_y = 0.05;

  auto mock_goal_handle =
    rtest::experimental::createMockGoalHandle<rtest_examples_interfaces::action::MoveRobot>(goal);

  // Expect that succeed will be called with correct result
  std::shared_ptr<rtest_examples_interfaces::action::MoveRobot::Result> captured_result;

  EXPECT_CALL(*mock_goal_handle, succeed(::testing::_))
    .Times(1)
    .WillOnce(::testing::SaveArg<0>(&captured_result));

  // Execute logic that should complete the goal
  bool completed = node->check_and_complete_goal(goal, mock_goal_handle);
  EXPECT_TRUE(completed);
  EXPECT_FALSE(node->is_moving());
  ASSERT_TRUE(captured_result);
  EXPECT_TRUE(captured_result->success);
  EXPECT_EQ(captured_result->message, "Target reached successfully");
}

TEST_F(ActionServerTest, GoalCanceledWhenCanceling)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples_interfaces::action::MoveRobot>(
      node, "move_robot");
  ASSERT_TRUE(server_mock);

  auto goal = std::make_shared<rtest_examples_interfaces::action::MoveRobot::Goal>();
  goal->target_x = 5.0;
  goal->target_y = 5.0;

  auto mock_goal_handle =
    rtest::experimental::createMockGoalHandle<rtest_examples_interfaces::action::MoveRobot>(goal);

  // Set up the mock to return true for is_canceling()
  EXPECT_CALL(*mock_goal_handle, is_canceling()).WillOnce(::testing::Return(true));

  // Expect that canceled will be called with correct result
  std::shared_ptr<rtest_examples_interfaces::action::MoveRobot::Result> captured_result;

  EXPECT_CALL(*mock_goal_handle, canceled(::testing::_))
    .Times(1)
    .WillOnce(::testing::SaveArg<0>(&captured_result));

  // Execute logic that should cancel the goal
  bool completed = node->check_and_complete_goal(goal, mock_goal_handle);

  EXPECT_TRUE(completed);
  EXPECT_FALSE(node->is_moving());
  ASSERT_TRUE(captured_result);
  EXPECT_FALSE(captured_result->success);
  EXPECT_EQ(captured_result->message, "Goal canceled");
}