/**
 * @file      subscription_mock.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief     Mock header for ROS 2 Subscriber.
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

#include <gmock/gmock.h>
#include <test_tools_ros/static_registry.h>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/subscription.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/detail/resolve_intra_process_buffer_type.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp {

namespace node_interfaces {
class NodeTopicsInterface;
}  // namespace node_interfaces

/// Subscription implementation, templated on the type of message this subscription receives.
template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    /// MessageT::custom_type if MessageT is a TypeAdapter,
    /// otherwise just MessageT.
    typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
    /// MessageT::ros_message_type if MessageT is a TypeAdapter,
    /// otherwise just MessageT.
    typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
    typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageT, AllocatorT>>
class Subscription : public SubscriptionBase {
  friend class rclcpp::node_interfaces::NodeTopicsInterface;

public:
  // Redeclare these here to use outside of the class.
  using SubscribedType = SubscribedT;
  using ROSMessageType = ROSMessageT;
  using MessageMemoryStrategyType = MessageMemoryStrategyT;

  using SubscribedTypeAllocatorTraits = allocator::AllocRebind<SubscribedType, AllocatorT>;
  using SubscribedTypeAllocator = typename SubscribedTypeAllocatorTraits::allocator_type;
  using SubscribedTypeDeleter = allocator::Deleter<SubscribedTypeAllocator, SubscribedType>;

  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  using MessageAllocatorTraits [[deprecated("use ROSMessageTypeAllocatorTraits")]] = ROSMessageTypeAllocatorTraits;
  using MessageAllocator [[deprecated("use ROSMessageTypeAllocator")]] = ROSMessageTypeAllocator;
  using MessageDeleter [[deprecated("use ROSMessageTypeDeleter")]] = ROSMessageTypeDeleter;

  using ConstMessageSharedPtr [[deprecated]] = std::shared_ptr<const ROSMessageType>;
  using MessageUniquePtr [[deprecated("use std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> instead")]] =
      std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>;

private:
  using SubscriptionTopicStatisticsSharedPtr = std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  /**
   * @copydoc rclcpp::Subscription::Subscription()
   */
  Subscription(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const rosidl_message_type_support_t &type_support_handle,
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      AnySubscriptionCallback<MessageT, AllocatorT> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options,
      typename MessageMemoryStrategyT::SharedPtr message_memory_strategy,
      SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics = nullptr) :
        SubscriptionBase(
            node_base,
            type_support_handle,
            topic_name,
            options.to_rcl_subscription_options(qos),
            options.event_callbacks,
            options.use_default_callbacks,
            callback.is_serialized_message_callback() ? DeliveredMessageKind::SERIALIZED_MESSAGE
                                                      : DeliveredMessageKind::ROS_MESSAGE),  // NOLINT
        any_callback_(callback),
        options_(options),
        message_memory_strategy_(message_memory_strategy) {
    // Setup intra process publishing if requested.
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      using rclcpp::detail::resolve_intra_process_buffer_type;

      // Check if the QoS is compatible with intra-process.
      auto qos_profile = get_actual_qos();
      if (qos_profile.history() != rclcpp::HistoryPolicy::KeepLast) {
        throw std::invalid_argument("intraprocess communication allowed only with keep last history qos policy");
      }
      if (qos_profile.depth() == 0) {
        throw std::invalid_argument("intraprocess communication is not allowed with 0 depth qos policy");
      }
    }

    if (subscription_topic_statistics != nullptr) {
      this->subscription_topic_statistics_ = std::move(subscription_topic_statistics);
    }
  }

  /// Called after construction to continue setup that requires shared_from_this().
  void post_init_setup(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const rclcpp::QoS &qos,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options) {
    (void)node_base;
    (void)qos;
    (void)options;

    test_tools_ros::StaticMocksRegistry::instance().registerSubscription<MessageT>(
        node_base->get_fully_qualified_name(), get_topic_name(), weak_from_this());
  }

  /**
   * @copydoc rclcpp::Subscription::take(ROSMessageType &, rclcpp::MessageInfo &)
   */
  bool take(ROSMessageType &message_out, rclcpp::MessageInfo &message_info_out) {
    return this->take_type_erased(static_cast<void *>(&message_out), message_info_out);
  }

  /**
   * @copydoc rclcpp::Subscription::take(TakeT &, rclcpp::MessageInfo &)
   */
  template <typename TakeT>
  std::enable_if_t<!rosidl_generator_traits::is_message<TakeT>::value && std::is_same_v<TakeT, SubscribedType>, bool> take(
      TakeT &message_out,
      rclcpp::MessageInfo &message_info_out) {
    ROSMessageType local_message;
    bool taken = this->take_type_erased(static_cast<void *>(&local_message), message_info_out);
    if (taken) {
      rclcpp::TypeAdapter<MessageT>::convert_to_custom(local_message, message_out);
    }
    return taken;
  }

  std::shared_ptr<void> create_message() override {
    /* The default message memory strategy provides a dynamically allocated message on each call to
     * create_message, though alternative memory strategies that re-use a preallocated message may be
     * used (see rclcpp/strategies/message_pool_memory_strategy.hpp).
     */
    return message_memory_strategy_->borrow_message();
  }

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override {
    return message_memory_strategy_->borrow_serialized_message();
  }

  void handle_message(std::shared_ptr<MessageT> message) {
    auto voidMsg = std::static_pointer_cast<void>(message);
    this->handle_message(voidMsg, rclcpp::MessageInfo{});
  }

  void handle_message(std::shared_ptr<void> &message, const rclcpp::MessageInfo &message_info) override {
    auto typed_message = std::static_pointer_cast<ROSMessageType>(message);
    any_callback_.dispatch(typed_message, message_info);
  }

  void handle_serialized_message(
      const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
      const rclcpp::MessageInfo &message_info) override {
    std::chrono::time_point<std::chrono::system_clock> now;
    if (subscription_topic_statistics_) {
      // get current time before executing callback to
      // exclude callback duration from topic statistics result.
      now = std::chrono::system_clock::now();
    }

    any_callback_.dispatch(serialized_message, message_info);

    if (subscription_topic_statistics_) {
      const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      const auto time = rclcpp::Time(nanos.time_since_epoch().count());
      subscription_topic_statistics_->handle_message(message_info.get_rmw_message_info(), time);
    }
  }

  void handle_loaned_message(void *loaned_message, const rclcpp::MessageInfo &message_info) override {
    if (matches_any_intra_process_publishers(&message_info.get_rmw_message_info().publisher_gid)) {
      // In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.
      return;
    }

    auto typed_message = static_cast<ROSMessageType *>(loaned_message);
    // message is loaned, so we have to make sure that the deleter does not deallocate the message
    auto sptr = std::shared_ptr<ROSMessageType>(typed_message, [](ROSMessageType *msg) { (void)msg; });

    std::chrono::time_point<std::chrono::system_clock> now;
    if (subscription_topic_statistics_) {
      // get current time before executing callback to
      // exclude callback duration from topic statistics result.
      now = std::chrono::system_clock::now();
    }

    any_callback_.dispatch(sptr, message_info);

    if (subscription_topic_statistics_) {
      const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      const auto time = rclcpp::Time(nanos.time_since_epoch().count());
      subscription_topic_statistics_->handle_message(message_info.get_rmw_message_info(), time);
    }
  }

  /**
   * @copydoc rclcpp::Subscription::return_message(std::shared_ptr<void> &)
   */
  void return_message(std::shared_ptr<void> &message) override {
    auto typed_message = std::static_pointer_cast<ROSMessageType>(message);
    message_memory_strategy_->return_message(typed_message);
  }

  /**
   * @copydoc rclcpp::Subscription::return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &)
   */
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &message) override {
    message_memory_strategy_->return_serialized_message(message);
  }

  bool use_take_shared_method() const { return any_callback_.use_take_shared_method(); }

  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr get_shared_dynamic_message_type() override {
    throw rclcpp::exceptions::UnimplementedError("get_shared_dynamic_message_type is not implemented for Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override {
    throw rclcpp::exceptions::UnimplementedError("get_shared_dynamic_message is not implemented for Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr get_shared_dynamic_serialization_support()
      override {
    throw rclcpp::exceptions::UnimplementedError(
        "get_shared_dynamic_serialization_support is not implemented for Subscription");
  }

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override {
    throw rclcpp::exceptions::UnimplementedError("create_dynamic_message is not implemented for Subscription");
  }

  void return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &message) override {
    (void)message;
    throw rclcpp::exceptions::UnimplementedError("return_dynamic_message is not implemented for Subscription");
  }

  void handle_dynamic_message(
      const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &message,
      const rclcpp::MessageInfo &message_info) override {
    (void)message;
    (void)message_info;
    throw rclcpp::exceptions::UnimplementedError("handle_dynamic_message is not implemented for Subscription");
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)

  AnySubscriptionCallback<MessageT, AllocatorT> any_callback_;

  /// Copy of original options passed during construction.
  /**
   * It is important to save a copy of this so that the rmw payload which it
   * may contain is kept alive for the duration of the subscription.
   */
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options_;
  typename message_memory_strategy::MessageMemoryStrategy<ROSMessageType, AllocatorT>::SharedPtr
      message_memory_strategy_;

  /// Component which computes and publishes topic statistics for this subscriber
  SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics_{nullptr};
};

}  // namespace rclcpp

namespace test_tools_ros {

/**
 * @brief Convenience function for getting subscription object for given Node name and Topic name.
 *
 * @tparam MessageT Type of the ROS 2 message for the topic
 * @param fullyQualifiedNodeNamme Fully-qualified node name
 * @param topicName               Topic name
 *
 * @return std::shared_ptr<rclcpp::Subscription<MessageT>>
 */
template <typename MessageT>
std::shared_ptr<rclcpp::Subscription<MessageT>> findSubscription(
    const std::string &fullyQualifiedNodeNamme,
    std::string topicName) {
  if (topicName.empty()) {
    throw std::invalid_argument{"Topic name must not be empty"};
  }
  if (topicName.front() != '/') {
    topicName.insert(topicName.begin(), '/');
  }
  auto sub = StaticMocksRegistry::instance().getSubscription(fullyQualifiedNodeNamme, topicName).lock();
  return std::dynamic_pointer_cast<rclcpp::Subscription<MessageT>>(sub);
}

/**
 * @brief Convenience function for getting subscription object for given Node and Topic name.
 *
 * @tparam MessageT Type of the ROS 2 message for the topic
 * @tparam NodeT    Type of the ROS 2 Node
 * @param nodePtr    shared_ptr to the Node
 * @param topicName  Topic name
 *
 * @return std::shared_ptr<rclcpp::Subscription<MessageT>>
 */
template <typename MessageT, typename NodeT>
std::shared_ptr<rclcpp::Subscription<MessageT>> findSubscription(
    const std::shared_ptr<NodeT> nodePtr,
    const std::string &topicName) {
  return findSubscription<MessageT>(nodePtr->get_fully_qualified_name(), topicName);
}

}  // namespace test_tools_ros
