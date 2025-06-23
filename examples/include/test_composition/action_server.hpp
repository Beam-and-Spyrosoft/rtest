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
// @file      action_server.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rtest_examples_interfaces/action/move_robot.hpp>

namespace test_composition
{

class ActionServer : public rclcpp::Node
{
public:
  using MoveRobot = rtest_examples_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit ActionServer(const rclcpp::NodeOptions & options);

  bool is_moving() const { return is_moving_; }
  std::pair<float, float> get_current_position() const { return {current_x_, current_y_}; }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRobot::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  std::pair<float, float> calculate_next_position(
    float current_x,
    float current_y,
    float target_x,
    float target_y,
    float step_size) const;

  MoveRobot::Feedback calculate_feedback(
    float current_x,
    float current_y,
    float target_x,
    float target_y) const;

  bool is_goal_reached(
    float current_x,
    float current_y,
    float target_x,
    float target_y,
    float tolerance = 0.1) const;

  void execute_single_step(
    const std::shared_ptr<const MoveRobot::Goal> goal,
    const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  bool check_and_complete_goal(
    const std::shared_ptr<const MoveRobot::Goal> goal,
    const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

private:
  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle);
  void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_moving_{false};
  float current_x_{0.0};
  float current_y_{0.0};
  std::shared_ptr<GoalHandleMoveRobot> current_goal_handle_;
};

}  // namespace test_composition