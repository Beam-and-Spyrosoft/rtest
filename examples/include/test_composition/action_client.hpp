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
// @file      action_client.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rtest_examples_interfaces/action/move_robot.hpp>

namespace test_composition
{

class ActionClient : public rclcpp::Node
{
public:
  using MoveRobot = rtest_examples_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

  explicit ActionClient(const rclcpp::NodeOptions & options);

  void send_goal(float x, float y);
  void cancel_current_goal();
  bool has_active_goal() const { return current_goal_handle_ != nullptr; }
  bool get_last_result_success() const { return last_result_success_; }
  std::string get_last_result_message() const { return last_result_message_; }
  bool has_received_feedback() const;
  MoveRobot::Feedback get_last_feedback() const;

private:
  void goal_response_callback(const GoalHandleMoveRobot::SharedPtr & goal_handle);
  void feedback_callback(const std::shared_ptr<const MoveRobot::Feedback> feedback);
  void result_callback(const GoalHandleMoveRobot::WrappedResult & result);
  rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
  GoalHandleMoveRobot::SharedPtr current_goal_handle_;
  bool last_result_success_{false};
  std::string last_result_message_;
  mutable std::mutex feedback_mutex_;
  MoveRobot::Feedback last_feedback_;
  bool feedback_received_{false};
};

}  // namespace test_composition