/**
 * @file      static_registry.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief     Mock header for ROS 2 static registry.
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

#include <algorithm>
#include <map>
#include <vector>
#include <memory>
#include <iostream>
#include <typeinfo>

#include <boost/type_index.hpp>

#include <test_tools_ros/single_instance.h>

namespace rclcpp {
class PublisherBase;
class SubscriptionBase;
class TimerBase;
}  // namespace rclcpp

namespace test_tools_ros {

class MockBase {};

/**
 * Whenever the ROS 2 Node creates a Subscriber, Publisher or Timer
 * it is registered in this static registry, so the user can retrieve a handle
 * to the mock object in the test.
 */
class StaticMocksRegistry : SingleInstance<StaticMocksRegistry> {
public:
  using TopicNameT = std::string;
  using FullyQualifiedNodeNameT = std::string;
  using TopicToPublishersMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::PublisherBase>>;
  using TopicToSubscriptionsMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::SubscriptionBase>>;

  /**
   * @brief Get the static instance of the Mock Registry.
   *
   * @return StaticMocksRegistry&
   */
  static StaticMocksRegistry &instance() { return theRegistry_; }

  /**
   * @brief Register the newly created Publisher in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @param pub       Newly created Publisher object
   */
  template <typename MessageT>
  void registerPublisher(
      const FullyQualifiedNodeNameT &nodeName,
      const TopicNameT &topicName,
      std::weak_ptr<rclcpp::PublisherBase> pub) {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerPublisher<" << boost::typeindex::type_id<MessageT>().pretty_name()
                << ">(\"" << nodeName << "\", \"" << topicName << "\")\n";
    }
    registerEntity(publishersRegistry_[nodeName], topicName, pub);
  }

  /**
   * @brief Get list of all publishers created by the selected Node.
   *
   * @param nodeName Fully-qualified Node name
   * @return std::vector<std::weak_ptr<rclcpp::PublisherBase>>
   */
  std::vector<std::weak_ptr<rclcpp::PublisherBase>> getNodePublishers(const FullyQualifiedNodeNameT &nodeName) {
    std::vector<std::weak_ptr<rclcpp::PublisherBase>> publishers{};
    for (auto [topicName, publisher] : publishersRegistry_[nodeName]) {
      publishers.push_back(publisher);
    }
    return publishers;
  }

  /**
   * @brief Get a publisher created by a selected Node for a particular Topic.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::PublisherBase>
   */
  std::weak_ptr<rclcpp::PublisherBase> getPublisher(
      const FullyQualifiedNodeNameT &nodeName,
      const TopicNameT &topicName) {
    return findEntity(publishersRegistry_[nodeName], topicName);
  }

  /**
   * @brief Register the newly created Subscription in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @param sub       Newly created Subscription object
   */
  template <typename MessageT>
  void registerSubscription(
      const FullyQualifiedNodeNameT &nodeName,
      const TopicNameT &topicName,
      std::weak_ptr<rclcpp::SubscriptionBase> sub) {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerSubscription<" << boost::typeindex::type_id<MessageT>().pretty_name()
                << ">(\"" << nodeName << "\", \"" << topicName << "\")\n";
    }
    registerEntity(subscriptionsRegistry_[nodeName], topicName, sub);
  }

  /**
   * @brief Get list of all subscriptions created by the selected Node.
   *
   * @param nodeName
   * @return std::vector<std::weak_ptr<rclcpp::SubscriptionBase>>
   */
  std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> getNodeSubscriptions(const FullyQualifiedNodeNameT &nodeName) {
    std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> subscriptions{};
    for (auto [topicName, subscription] : subscriptionsRegistry_[nodeName]) {
      subscriptions.push_back(subscription);
    }
    return subscriptions;
  }

  /**
   * @brief Get a subscription created by a selected Node for a particular Topic.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::SubscriptionBase>
   */
  std::weak_ptr<rclcpp::SubscriptionBase> getSubscription(
      const FullyQualifiedNodeNameT &nodeName,
      const TopicNameT &topicName) {
    return findEntity(subscriptionsRegistry_[nodeName], topicName);
  }

  /**
   * @brief Register the newly created Timer in the regisrtry.
   * This function shall be used by the rclcpp::create_timer() function only.
   *
   * @param nodeName Fully-qualified Node name
   * @param timer    Newly created Timer object
   * @return true
   * @return false
   */
  bool registerTimer(const FullyQualifiedNodeNameT &nodeName, std::weak_ptr<rclcpp::TimerBase> timer) {
    timersRegistry_[nodeName].push_back(timer);
    return true;
  }

  /**
   * @brief Get list of all Timers created by the selected Node.
   *
   * @param nodeName Fully-qualified Node name
   * @return std::vector<std::weak_ptr<rclcpp::TimerBase>>
   */
  std::vector<std::weak_ptr<rclcpp::TimerBase>> getTimers(const FullyQualifiedNodeNameT &nodeName) {
    return findEntity(timersRegistry_, nodeName);
  }

  /**
   * @brief Enable additional verbose logs to trace registry events.
   *
   * @param on
   */
  void enableVerboseLogs(bool on) { verbose_ = on; }

  std::weak_ptr<MockBase> getMock(void *ptr) {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      return it->second;
    }
    return {};
  }

  void attachMock(void *ptr, std::weak_ptr<MockBase> mock) { mockRegistry_[ptr] = mock; }

  void detachMock(void *ptr) {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      mockRegistry_.erase(it);
    }
  }

private:
  StaticMocksRegistry() {}

  static StaticMocksRegistry theRegistry_;

  template <typename RegistryT, typename EntityT>
  void registerEntity(RegistryT &reg, const TopicNameT &topicName, EntityT e) {
    if (!reg[topicName].lock()) {
      reg[topicName] = e;
    }
  }

  template <typename RegistryT>
  typename RegistryT::value_type::second_type findEntity(RegistryT &reg, const TopicNameT &topicName) {
    auto it = reg.find(topicName);
    if (it != reg.end()) {
      return it->second;
    } else {
      return {};
    }
  }

  std::map<FullyQualifiedNodeNameT, TopicToPublishersMapT> publishersRegistry_;
  std::map<FullyQualifiedNodeNameT, TopicToSubscriptionsMapT> subscriptionsRegistry_;
  std::map<FullyQualifiedNodeNameT, std::vector<std::weak_ptr<rclcpp::TimerBase>>> timersRegistry_;

  std::map<void *, std::weak_ptr<MockBase>> mockRegistry_;

  bool verbose_{false};
};

void enableVerboseLogs(bool on);

}  // namespace test_tools_ros
