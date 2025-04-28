/**
 * @file      publisher_mock.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief     Mock header for ROS 2 Publisher.
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

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/publisher.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rosidl_runtime_cpp/traits.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/detail/resolve_intra_process_buffer_type.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/is_ros_compatible_type.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace test_tools_ros {

/**
 * @brief A wrapper object for rclcpp::Publisher that intercepts the publish() method invocation and implements a mock
 * for it.
 *
 * @tparam MessageT
 */
template <typename MessageT>
class PublisherMock : public MockBase {
public:
  PublisherMock(rclcpp::PublisherBase *pub) : pub_(pub) {}

  ~PublisherMock() { StaticMocksRegistry::instance().detachMock(pub_); }

  RCLCPP_SMART_PTR_DEFINITIONS(PublisherMock<MessageT>)

  /**
   * @brief This is the mock method that gets called eventually from all other overloads.
   */
  MOCK_METHOD(void, publish, (const MessageT &msg));

  /**
   * @brief Set the fake subscriptions count.
   *
   * @param count number of fake subscriptions
   */
  void set_subscription_count(size_t count) { subscriptions_count_ = count; }

  size_t subscriptions_count_{0UL};

  rclcpp::PublisherBase *pub_{nullptr};
};

}  // namespace test_tools_ros

namespace rclcpp {

template <typename MessageT, typename AllocatorT>
class LoanedMessage;

template <typename MessageT, typename AllocatorT = std::allocator<void>>
class Publisher : public PublisherBase {
public:
  /// MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, AllocatorT>;
  using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
  using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  using BufferSharedPtr = typename rclcpp::experimental::buffers::
      IntraProcessBuffer<ROSMessageType, ROSMessageTypeAllocator, ROSMessageTypeDeleter>::SharedPtr;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, AllocatorT>)

  /**
   * @copydoc rclcpp::Publisher::Publisher()
   */
  Publisher(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic,
      const rclcpp::QoS &qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options) :
        PublisherBase(
            node_base,
            topic,
            rclcpp::get_message_type_support_handle<MessageT>(),
            options.template to_rcl_publisher_options<MessageT>(qos),
            options.event_callbacks,
            options.use_default_callbacks),
        options_(options),
        published_type_allocator_(*options.get_allocator()),
        ros_message_type_allocator_(*options.get_allocator()) {
    allocator::set_allocator_for_deleter(&published_type_deleter_, &published_type_allocator_);
    allocator::set_allocator_for_deleter(&ros_message_type_deleter_, &ros_message_type_allocator_);
  }

  virtual void post_init_setup(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic,
      const rclcpp::QoS &qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options) {
    (void)node_base;
    (void)topic;
    (void)qos;
    (void)options;

    test_tools_ros::StaticMocksRegistry::instance().registerPublisher<MessageT>(
        node_base->get_fully_qualified_name(), get_topic_name(), weak_from_this());
  }

  virtual ~Publisher() {}

  /**
   * @brief Internal sink publish with activation control
   *
   * @param msg published message
   */
  void publish_(const MessageT &msg) {
    auto mock = test_tools_ros::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      (std::static_pointer_cast<test_tools_ros::PublisherMock<MessageT>>(mock))->publish(msg);
    }
  }

  /**
   * @brief Get the subscriptions count.
   *
   * @return number of real and fake subscriptions
   */
  size_t get_subscription_count() {
    auto mock = test_tools_ros::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return PublisherBase::get_subscription_count() +
             (std::static_pointer_cast<test_tools_ros::PublisherMock<MessageT>>(mock))->subscriptions_count_;
    }

    return PublisherBase::get_subscription_count();
  }

  /**
   * @copydoc rclcpp::Publisher::borrow_loaned_message()
   */
  rclcpp::LoanedMessage<ROSMessageType, AllocatorT> borrow_loaned_message() {
    return rclcpp::LoanedMessage<ROSMessageType, AllocatorT>(*this, std::allocator<void>{});
  }

  /**
   * @copydoc rclcpp::Publisher::publish(std::unique_ptr<MessageT>)
   */
  template <typename T>
  typename std::enable_if_t<rosidl_generator_traits::is_message<T>::value && std::is_same<T, ROSMessageType>::value> publish(
      std::unique_ptr<T, ROSMessageTypeDeleter> msg) {
    this->publish_(*msg);
  }

  /**
   * @copydoc rclcpp::Publisher::publish(const MessageT &)
   */
  template <typename T>
  typename std::enable_if_t<rosidl_generator_traits::is_message<T>::value && std::is_same<T, ROSMessageType>::value> publish(
      const T &msg) {
    this->publish_(msg);
  }

  /**
   * @copydoc rclcpp::Publisher::publish(std::unique_ptr<MessageT>)
   */
  template <typename T>
  typename std::enable_if_t<
      rclcpp::TypeAdapter<MessageT>::is_specialized::value && std::is_same<T, PublishedType>::value>
  publish(std::unique_ptr<T, PublishedTypeDeleter> msg) {
    this->publish_(*msg);
  }

  /**
   * @copydoc rclcpp::Publisher::publish(const MessageT &)
   */
  template <typename T>
  typename std::enable_if_t<
      rclcpp::TypeAdapter<MessageT>::is_specialized::value && std::is_same<T, PublishedType>::value>
  publish(const T &msg) {
    this->publish_(*msg);
  }

  /**
   * @copydoc rclcpp::Publisher::publish(rcl_serialized_message_t)
   */
  void publish(const rcl_serialized_message_t &serialized_msg) {
    (void)serialized_msg;
    throw std::runtime_error{"Publisher::publish(rcl_serialized_message_t) is not implemented"};
  }

  /**
   * @copydoc rclcpp::Publisher::publish(SerializedMessage)
   */
  void publish(const SerializedMessage &serialized_msg) {
    (void)serialized_msg;
    throw std::runtime_error{"Publisher::publish(SerializedMessage) is not implemented"};
  }

  /**
   * @copydoc rclcpp::Publisher::publish(rclcpp::LoanedMessage)
   */
  void publish(rclcpp::LoanedMessage<ROSMessageType, AllocatorT> &&loaned_msg) {
    if (!loaned_msg.is_valid()) {
      throw std::runtime_error("loaned message is not valid");
    }
    this->publish_(loaned_msg.get());
  }

  /**
   * @copydoc rclcpp::Publisher::duplicate_ros_message_as_unique_ptr()
   */
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> duplicate_ros_message_as_unique_ptr(
      const ROSMessageType &msg) {
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr, msg);
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> options_;

  PublishedTypeAllocator published_type_allocator_;
  PublishedTypeDeleter published_type_deleter_;
  ROSMessageTypeAllocator ros_message_type_allocator_;
  ROSMessageTypeDeleter ros_message_type_deleter_;

  BufferSharedPtr buffer_{nullptr};
};

}  // namespace rclcpp

namespace test_tools_ros {

/**
 * @brief Convenience function for getting publisher object for given Node name and Topic name.
 *
 * @tparam MessageT Type of the ROS 2 message for the topic
 * @param fullyQualifiedNodeNamme Fully-qualified node name
 * @param topicName               Topic name
 *
 * @return std::shared_ptr<rclcpp::Publisher<MessageT>>
 */
template <typename MessageT>
std::shared_ptr<PublisherMock<MessageT>> findPublisher(
    const std::string &fullyQualifiedNodeNamme,
    std::string topicName) {
  if (topicName.empty()) {
    throw std::invalid_argument{"Topic name must not be empty"};
  }
  if (topicName.front() != '/') {
    topicName.insert(topicName.begin(), '/');
  }
  std::shared_ptr<PublisherMock<MessageT>> pub_mock{};
  auto pub_base = StaticMocksRegistry::instance().getPublisher(fullyQualifiedNodeNamme, topicName).lock();
  auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<MessageT>>(pub_base);
  if (publisher) {
    if (StaticMocksRegistry::instance().getMock(publisher.get()).lock()) {
      std::cerr << "test_tools_ros::findPublisher() WARNING: PublisherMock already attached to the Publisher\n";
    } else {
      pub_mock = std::make_shared<PublisherMock<MessageT>>(publisher.get());
      StaticMocksRegistry::instance().attachMock(publisher.get(), pub_mock);
    }
  }
  return pub_mock;
}

/**
 * @brief Convenience function for getting publisher object for given Node and Topic name.
 *
 * @tparam MessageT Type of the ROS 2 message for the topic
 * @tparam NodeT    Type of the ROS 2 Node
 * @param nodePtr    shared_ptr to the Node
 * @param topicName  Topic name
 *
 * @return std::shared_ptr<rclcpp::Publisher<MessageT>>
 */
template <typename MessageT, typename NodeT>
std::shared_ptr<PublisherMock<MessageT>> findPublisher(
    const std::shared_ptr<NodeT> nodePtr,
    const std::string &topicName) {
  return findPublisher<MessageT>(nodePtr->get_fully_qualified_name(), topicName);
}

}  // namespace test_tools_ros
