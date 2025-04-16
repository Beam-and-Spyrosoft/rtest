/**
 * @file      create_timer_mock.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief     Mock header for timer creation functionality.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#define RCLCPP__CREATE_TIMER_HPP_  // disable default implementation

#include <gmock/gmock.h>
#include <test_tools_ros/static_registry.h>

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_clock_interface.hpp"
#include "rclcpp/node_interfaces/get_node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"

namespace rclcpp {
namespace detail {

/**
 * @copydoc  rclcpp::detail::safe_cast_to_period_in_ns(std::chrono::duration<DurationRepT, DurationT>)
 */
template <typename DurationRepT, typename DurationT>
std::chrono::nanoseconds safe_cast_to_period_in_ns(std::chrono::duration<DurationRepT, DurationT> period) {
  if (period < std::chrono::duration<DurationRepT, DurationT>::zero()) {
    throw std::invalid_argument{"timer period cannot be negative"};
  }

  constexpr auto maximum_safe_cast_ns =
      std::chrono::nanoseconds::max() - std::chrono::duration<DurationRepT, DurationT>(1);

  constexpr auto ns_max_as_double =
      std::chrono::duration_cast<std::chrono::duration<double, std::chrono::nanoseconds::period>>(maximum_safe_cast_ns);
  if (period > ns_max_as_double) {
    throw std::invalid_argument{"timer period must be less than std::chrono::nanoseconds::max()"};
  }

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  if (period_ns < std::chrono::nanoseconds::zero()) {
    throw std::runtime_error{"Casting timer period to nanoseconds resulted in integer overflow."};
  }

  return period_ns;
}
}  // namespace detail

/**
 * @copydoc  rclcpp::detail::create_timer()
 */
template <typename CallbackT>
typename rclcpp::TimerBase::SharedPtr create_timer(
    std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
    std::shared_ptr<node_interfaces::NodeTimersInterface> node_timers,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Duration period,
    CallbackT &&callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    bool autostart = true) {
  return create_timer(
      clock,
      period.to_chrono<std::chrono::nanoseconds>(),
      std::forward<CallbackT>(callback),
      group,
      node_base.get(),
      node_timers.get(),
      autostart);
}

/**
 * @copydoc  rclcpp::create_timer()
 */
template <typename NodeT, typename CallbackT>
typename rclcpp::TimerBase::SharedPtr create_timer(
    NodeT node,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Duration period,
    CallbackT &&callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    bool autostart = true) {
  return create_timer(
      clock,
      period.to_chrono<std::chrono::nanoseconds>(),
      std::forward<CallbackT>(callback),
      group,
      rclcpp::node_interfaces::get_node_base_interface(node).get(),
      rclcpp::node_interfaces::get_node_timers_interface(node).get(),
      autostart);
}

/**
 * @copydoc  rclcpp::create_timer()
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::GenericTimer<CallbackT>::SharedPtr create_timer(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group,
    node_interfaces::NodeBaseInterface *node_base,
    node_interfaces::NodeTimersInterface *node_timers,
    bool autostart = true) {
  if (clock == nullptr) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  if (node_base == nullptr) {
    throw std::invalid_argument{"input node_base cannot be null"};
  }
  if (node_timers == nullptr) {
    throw std::invalid_argument{"input node_timers cannot be null"};
  }

  const std::chrono::nanoseconds period_ns = detail::safe_cast_to_period_in_ns(period);

  // Add a new generic timer.
  auto timer = rclcpp::GenericTimer<CallbackT>::make_shared(
      std::move(clock), period_ns, std::move(callback), node_base->get_context(), autostart);
  node_timers->add_timer(timer, group);

  test_tools_ros::StaticMocksRegistry::instance().registerTimer(node_base->get_fully_qualified_name(), timer);

  return timer;
}

/**
 * @copydoc  rclcpp::create_timer()
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group,
    node_interfaces::NodeBaseInterface *node_base,
    node_interfaces::NodeTimersInterface *node_timers,
    bool autostart = true) {
  if (node_base == nullptr) {
    throw std::invalid_argument{"input node_base cannot be null"};
  }

  if (node_timers == nullptr) {
    throw std::invalid_argument{"input node_timers cannot be null"};
  }

  const std::chrono::nanoseconds period_ns = detail::safe_cast_to_period_in_ns(period);

  // Add a new wall timer.
  auto timer =
      rclcpp::WallTimer<CallbackT>::make_shared(period_ns, std::move(callback), node_base->get_context(), autostart);
  node_timers->add_timer(timer, group);

  test_tools_ros::StaticMocksRegistry::instance().registerTimer(node_base->get_fully_qualified_name(), timer);

  return timer;
}
}  // namespace rclcpp

namespace test_tools_ros {

/**
 * @brief Convenience function for getting a list of Timers created by thje given Node.
 *
 * @param fullyQualifiedNodeNamme Fully-qualified node name
 *
 * @return std::vector<std::shared_ptr<rclcpp::TimerBase>>
 */
static inline std::vector<std::shared_ptr<rclcpp::TimerBase>> findTimers(const std::string &fullyQualifiedNodeNamme) {
  std::vector<std::shared_ptr<rclcpp::TimerBase>> timers{};
  for (auto &weakPtr : StaticMocksRegistry::instance().getTimers(fullyQualifiedNodeNamme)) {
    if (auto timer = weakPtr.lock()) {
      timers.push_back(timer);
    }
  }
  return timers;
}

/**
 * @brief Convenience function for getting a list of Timers created by thje given Node.
 *
 * @param nodePtr shared_ptr to the Node
 *
 * @return std::vector<std::shared_ptr<rclcpp::TimerBase>>
 */
template <typename NodeT>
static inline std::vector<std::shared_ptr<rclcpp::TimerBase>> findTimers(const std::shared_ptr<NodeT> nodePtr) {
  return findTimers(nodePtr->get_fully_qualified_name());
}

}  // namespace test_tools_ros
