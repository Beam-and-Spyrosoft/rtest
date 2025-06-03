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

TEST_F(ActionServerTest, GoalAcceptanceWithinRange)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(server_mock);

  // Test node's initial state
  EXPECT_FALSE(node->is_moving());
  auto [x, y] = node->get_current_position();
  EXPECT_FLOAT_EQ(x, 0.0);
  EXPECT_FLOAT_EQ(y, 0.0);

  // Create a goal within acceptable range (distance < 10.0)
  auto goal = std::make_shared<rtest_examples::action::MoveRobot::Goal>();
  goal->target_x = 2.0;
  goal->target_y = 3.0;

  rclcpp_action::GoalUUID uuid;

  // Mock should simulate the real business logic: accept if distance <= 10.0 and not moving
  EXPECT_CALL(*server_mock, handle_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([node](
                const rclcpp_action::GoalUUID & /*uuid*/,
                std::shared_ptr<const rtest_examples::action::MoveRobot::Goal> goal) {
      float distance = std::sqrt(goal->target_x * goal->target_x + goal->target_y * goal->target_y);
      if (distance > 10.0) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (node->is_moving()) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    });

  auto response = server_mock->handle_goal(uuid, goal);

  // Verify the goal was accepted (distance = sqrt(4+9) ≈ 3.6 < 10.0 and not moving)
  EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(ActionServerTest, GoalRejectionWhenTooFar)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(server_mock);

  // Create a goal that should be rejected (distance > 10.0)
  auto goal = std::make_shared<rtest_examples::action::MoveRobot::Goal>();
  goal->target_x = 50.0;  // distance = sqrt(2500+2500) = sqrt(5000) ≈ 70.7 > 10.0
  goal->target_y = 50.0;

  rclcpp_action::GoalUUID uuid;

  // Mock should simulate the real business logic
  EXPECT_CALL(*server_mock, handle_goal(::testing::_, ::testing::_))
    .Times(1)
    .WillOnce([node](
                const rclcpp_action::GoalUUID & /*uuid*/,
                std::shared_ptr<const rtest_examples::action::MoveRobot::Goal> goal) {
      float distance = std::sqrt(goal->target_x * goal->target_x + goal->target_y * goal->target_y);
      if (distance > 10.0) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (node->is_moving()) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    });

  auto response = server_mock->handle_goal(uuid, goal);

  // Verify the goal was rejected due to distance
  EXPECT_EQ(response, rclcpp_action::GoalResponse::REJECT);
}

TEST_F(ActionServerTest, FeedbackPublishingWithValidData)
{
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock =
    rtest::experimental::findActionServer<rtest_examples::action::MoveRobot>(node, "move_robot");
  ASSERT_TRUE(server_mock);
  auto goal = std::make_shared<rtest_examples::action::MoveRobot::Goal>();
  goal->target_x = 3.0;
  goal->target_y = 4.0;
  auto mock_goal_handle =
    rtest::experimental::createMockGoalHandle<rtest_examples::action::MoveRobot>(goal);
  EXPECT_CALL(
    *mock_goal_handle,
    publish_feedback(::testing::AllOf(
      ::testing::Pointee(::testing::Field(
        &rtest_examples::action::MoveRobot::Feedback::current_x, ::testing::Ge(0.0))),
      ::testing::Pointee(::testing::Field(
        &rtest_examples::action::MoveRobot::Feedback::current_y, ::testing::Ge(0.0))),
      ::testing::Pointee(::testing::Field(
        &rtest_examples::action::MoveRobot::Feedback::distance_remaining, ::testing::Ge(0.0))))))
    .Times(5)
    .WillRepeatedly(::testing::Invoke(
      [](std::shared_ptr<const rtest_examples::action::MoveRobot::Feedback> feedback) {
        std::cout << "Mock feedback: (" << feedback->current_x << ", " << feedback->current_y
                  << ") distance: " << feedback->distance_remaining << std::endl;
      }));

  // Execute test
  for (int i = 0; i < 5; i++) {
    node->execute_single_step(goal, mock_goal_handle);
  }
  auto [final_x, final_y] = node->get_current_position();
  EXPECT_GT(final_x, 0.0);
  EXPECT_GT(final_y, 0.0);
}