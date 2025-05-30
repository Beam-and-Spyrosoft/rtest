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
    std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

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

  std::thread{std::bind(&ActionServer::execute_move, this, goal_handle)}.detach();
}

void ActionServer::execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveRobot::Feedback>();
  auto result = std::make_shared<MoveRobot::Result>();

  float dx = goal->target_x - current_x_;
  float dy = goal->target_y - current_y_;

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
    dx = goal->target_x - current_x_;
    dy = goal->target_y - current_y_;
    float remaining_distance = std::sqrt(dx * dx + dy * dy);

    // Check if reached target
    if (remaining_distance < 0.1) {
      result->success = true;
      result->final_x = current_x_;
      result->final_y = current_y_;
      result->message = "Target reached successfully";
      goal_handle->succeed(result);
      is_moving_ = false;
      return;
    }

    // Move towards target
    float step_x = (dx / remaining_distance) * step_size;
    float step_y = (dy / remaining_distance) * step_size;
    current_x_ += step_x;
    current_y_ += step_y;

    // Publish feedback
    feedback->current_x = current_x_;
    feedback->current_y = current_y_;
    feedback->distance_remaining = remaining_distance;
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(update_rate);
  }
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ActionServer)