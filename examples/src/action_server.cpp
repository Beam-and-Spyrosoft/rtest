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
// @file      action_server.cpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#include "test_composition/action_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <thread>

using namespace std::chrono_literals;

namespace test_composition
{

ActionServer::ActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("move_robot_server", options)
{
  action_server_ = rclcpp_action::create_server<MoveRobot>(
    this,
    "move_robot",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveRobot::Goal> goal) {
      return handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleMoveRobot> & goal_handle) {
      return handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleMoveRobot> & goal_handle) {
      handle_accepted(goal_handle);
    });

  RCLCPP_INFO(get_logger(), "Move robot action server started");
}

rclcpp_action::GoalResponse ActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveRobot::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    get_logger(), "Received goal request to move to (%.2f, %.2f)", goal->target_x, goal->target_y);

  if (std::abs(goal->target_x) > 10.0 || std::abs(goal->target_y) > 10.0) {
    RCLCPP_WARN(get_logger(), "Goal rejected: target too far");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (is_moving_) {
    RCLCPP_WARN(get_logger(), "Goal rejected: already moving");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  current_goal_handle_ = goal_handle;
  is_moving_ = true;

  std::thread{[this, goal_handle]() { execute_move(goal_handle); }}.detach();
}

void ActionServer::execute_single_step(
  const std::shared_ptr<const MoveRobot::Goal> goal,
  const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  const float step_size = 0.01;

  // Calculate next position using real business logic
  auto [new_x, new_y] =
    calculate_next_position(current_x_, current_y_, goal->target_x, goal->target_y, step_size);

  current_x_ = new_x;
  current_y_ = new_y;

  // Create and publish feedback using real business logic
  auto feedback = std::make_shared<MoveRobot::Feedback>(
    calculate_feedback(current_x_, current_y_, goal->target_x, goal->target_y));
  goal_handle->publish_feedback(feedback);
}

void ActionServer::execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveRobot::Feedback>();
  auto result = std::make_shared<MoveRobot::Result>();

  const float speed = 1.0;
  const auto update_rate = 10ms;
  const float step_size = speed * 0.01;

  while (rclcpp::ok() && is_moving_) {
    // Check if goal is canceled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->final_x = current_x_;
      result->final_y = current_y_;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      is_moving_ = false;
      return;
    }

    // Calculate remaining distance
    *feedback = calculate_feedback(current_x_, current_y_, goal->target_x, goal->target_y);

    // Check if reached target
    if (is_goal_reached(current_x_, current_y_, goal->target_x, goal->target_y)) {
      result->success = true;
      result->final_x = current_x_;
      result->final_y = current_y_;
      result->message = "Target reached successfully";
      goal_handle->succeed(result);
      is_moving_ = false;
      return;
    }

    // Move towards target
    auto [new_x, new_y] =
      calculate_next_position(current_x_, current_y_, goal->target_x, goal->target_y, step_size);
    current_x_ = new_x;
    current_y_ = new_y;
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(update_rate);
  }
}

std::pair<float, float> ActionServer::calculate_next_position(
  float current_x,
  float current_y,
  float target_x,
  float target_y,
  float step_size) const
{
  float dx = target_x - current_x;
  float dy = target_y - current_y;
  float distance = std::sqrt(dx * dx + dy * dy);

  if (distance < 0.001) {  // Avoid division by zero
    return {current_x, current_y};
  }

  float step_x = (dx / distance) * step_size;
  float step_y = (dy / distance) * step_size;

  return {current_x + step_x, current_y + step_y};
}

ActionServer::MoveRobot::Feedback ActionServer::calculate_feedback(
  float current_x,
  float current_y,
  float target_x,
  float target_y) const
{
  MoveRobot::Feedback feedback;
  feedback.current_x = current_x;
  feedback.current_y = current_y;

  float dx = target_x - current_x;
  float dy = target_y - current_y;
  feedback.distance_remaining = std::sqrt(dx * dx + dy * dy);

  return feedback;
}

bool ActionServer::is_goal_reached(
  float current_x,
  float current_y,
  float target_x,
  float target_y,
  float tolerance) const
{
  float dx = target_x - current_x;
  float dy = target_y - current_y;
  float distance = std::sqrt(dx * dx + dy * dy);
  return distance < tolerance;
}

bool ActionServer::check_and_complete_goal(
  const std::shared_ptr<const MoveRobot::Goal> goal,
  const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  // Check if goal is canceled
  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<MoveRobot::Result>();
    result->success = false;
    result->final_x = current_x_;
    result->final_y = current_y_;
    result->message = "Goal canceled";
    goal_handle->canceled(result);
    is_moving_ = false;
    return true;
  }

  // Check if reached target
  if (is_goal_reached(current_x_, current_y_, goal->target_x, goal->target_y)) {
    auto result = std::make_shared<MoveRobot::Result>();
    result->success = true;
    result->final_x = current_x_;
    result->final_y = current_y_;
    result->message = "Target reached successfully";
    goal_handle->succeed(result);
    is_moving_ = false;
    return true;
  }

  return false;
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ActionServer)