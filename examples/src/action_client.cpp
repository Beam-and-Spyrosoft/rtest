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
// @file      action_client.cpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#include "test_composition/action_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace test_composition
{

ActionClient::ActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("move_robot_client", options)
{
  action_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");
  RCLCPP_INFO(get_logger(), "Move robot action client started");
}

void ActionClient::send_goal(float x, float y)
{
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return;
  }

  auto goal_msg = MoveRobot::Goal();
  goal_msg.target_x = x;
  goal_msg.target_y = y;

  RCLCPP_INFO(get_logger(), "Sending goal: move to (%.2f, %.2f)", x, y);

  rclcpp_action::Client<MoveRobot>::SendGoalOptions send_goal_options{};

  send_goal_options.goal_response_callback =
    [this](const GoalHandleMoveRobot::SharedPtr & goal_handle) {
      goal_response_callback(goal_handle);
    };
  send_goal_options.feedback_callback =
    [this](
      GoalHandleMoveRobot::SharedPtr, const std::shared_ptr<const MoveRobot::Feedback> feedback) {
      feedback_callback(feedback);
    };
  send_goal_options.result_callback = [this](const GoalHandleMoveRobot::WrappedResult & result) {
    result_callback(result);
  };

  auto f = action_client_->async_send_goal(goal_msg, send_goal_options);
  if (f.wait_for(1s) == std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "Failed to send goal within timeout");
    return;
  }
  current_goal_handle_ = f.get();

  if (current_goal_handle_) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Goal handle Id: " << rclcpp_action::to_string(current_goal_handle_->get_goal_id())
                         << " accepted by server at "
                         << current_goal_handle_->get_goal_stamp().seconds());
  } else {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  }
}

void ActionClient::cancel_current_goal()
{
  if (current_goal_handle_) {
    RCLCPP_INFO(get_logger(), "Canceling current goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
}

void ActionClient::goal_response_callback(const GoalHandleMoveRobot::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    current_goal_handle_ = nullptr;
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    current_goal_handle_ = goal_handle;
  }
}

void ActionClient::feedback_callback(const std::shared_ptr<const MoveRobot::Feedback> feedback)
{
  RCLCPP_INFO(
    get_logger(),
    "Feedback: current position (%.2f, %.2f), distance remaining: %.2f",
    feedback->current_x,
    feedback->current_y,
    feedback->distance_remaining);
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_ = *feedback;
    feedback_received_ = true;
  }
}

void ActionClient::result_callback(const GoalHandleMoveRobot::WrappedResult & result)
{
  current_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      last_result_success_ = result.result->success;
      last_result_message_ = result.result->message;
      RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      last_result_success_ = false;
      last_result_message_ = "Goal aborted";
      RCLCPP_ERROR(get_logger(), "Goal aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      last_result_success_ = false;
      last_result_message_ = "Goal canceled";
      RCLCPP_INFO(get_logger(), "Goal canceled");
      break;
    default:
      last_result_success_ = false;
      last_result_message_ = "Unknown result code";
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      break;
  }
}

bool ActionClient::has_received_feedback() const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return feedback_received_;
}

ActionClient::MoveRobot::Feedback ActionClient::get_last_feedback() const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return last_feedback_;
}

}  // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ActionClient)
