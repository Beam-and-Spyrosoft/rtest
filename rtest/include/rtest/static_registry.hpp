// Copyright 2024 Beam Limited.
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
// @file      static_registry.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-26
//
// @brief     Mock header for ROS 2 static registry.

#pragma once

#include <algorithm>
#include <map>
#include <vector>
#include <memory>
#include <iostream>
#include <typeinfo>
#include <functional>
#include <mutex>

#include <boost/type_index.hpp>
#include <rcl/node.h>

#include <rtest/single_instance.hpp>
#include <rtest/registry_cleaner.hpp>

#define TEST_TOOLS_MAKE_SHARED_DEFINITION(...)                             \
  template <typename... Args>                                              \
  static std::shared_ptr<__VA_ARGS__> make_shared(Args &&... args)         \
  {                                                                        \
    auto ptr = std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
    ptr->post_init_setup();                                                \
    return ptr;                                                            \
  }

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__)      \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__)        \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__)      \
  TEST_TOOLS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

namespace rclcpp
{
class PublisherBase;
class SubscriptionBase;
class TimerBase;
class ServiceBase;
class ClientBase;
}  // namespace rclcpp

namespace rclcpp_action
{
class ServerBase;
class ClientBase;
}  // namespace rclcpp_action

namespace rtest
{

class MockBase
{
};

/**
 * Whenever the ROS 2 Node creates a Subscriber, Publisher or Timer
 * it is registered in this static registry, so the user can retrieve a handle
 * to the mock object in the test.
 */
class StaticMocksRegistry : SingleInstance<StaticMocksRegistry>
{
public:
  struct LazyInitEntry
  {
    void * raw_ptr;
    std::string node_name;
    std::string action_name;
    std::function<void()> init_callback;
  };

public:
  using TopicNameT = std::string;
  // Weak ownership identifies a node without keeping it alive. owner_less compares
  // control blocks, so a later node reusing the same address cannot collide.
  using NodeId = std::weak_ptr<const rcl_node_t>;
  template <typename EntriesT>
  using NodeRegistry = std::map<NodeId, EntriesT, std::owner_less<NodeId>>;
  using TopicToPublishersMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::PublisherBase>>;
  using TopicToSubscriptionsMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::SubscriptionBase>>;
  using ServiceNameT = std::string;
  using ServiceToServicesMapT = std::map<ServiceNameT, std::weak_ptr<rclcpp::ServiceBase>>;
  using ServiceToClientsMapT = std::map<ServiceNameT, std::weak_ptr<rclcpp::ClientBase>>;
  using ActionNameT = std::string;
  using ActionToServersMapT = std::map<ActionNameT, std::weak_ptr<rclcpp_action::ServerBase>>;
  using ActionToClientsMapT = std::map<ActionNameT, std::weak_ptr<rclcpp_action::ClientBase>>;

  /**
   * @brief Get the static instance of the Mock Registry.
   *
   * @return StaticMocksRegistry&
   */
  static StaticMocksRegistry & instance() { return theRegistry_; }

  /**
   * @brief Register the newly created Publisher in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param node Node instance identity
   * @param topicName Topic name
   * @param pub       Newly created Publisher object
   */
  template <typename MessageT>
  void registerPublisher(
    const NodeId & node,
    const TopicNameT & topicName,
    std::weak_ptr<rclcpp::PublisherBase> pub)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerPublisher<"
                << boost::typeindex::type_id<MessageT>().pretty_name() << ">(\""
                << node.lock().get() << "\", \"" << topicName << "\")\n";
    }
    registerEntity(publishersRegistry_[node], topicName, pub);
  }

  /**
   * @brief Get list of all publishers created by the selected Node.
   *
   * @param node Node instance identity
   * @return std::vector<std::weak_ptr<rclcpp::PublisherBase>>
   */
  std::vector<std::weak_ptr<rclcpp::PublisherBase>> getNodePublishers(const NodeId & node)
  {
    std::vector<std::weak_ptr<rclcpp::PublisherBase>> publishers{};
    for (auto [topicName, publisher] : publishersRegistry_[node]) {
      publishers.push_back(publisher);
    }
    return publishers;
  }

  /**
   * @brief Get a publisher created by a selected Node for a particular Topic.
   *
   * @param node Node instance identity
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::PublisherBase>
   */
  std::weak_ptr<rclcpp::PublisherBase> getPublisher(
    const NodeId & node,
    const TopicNameT & topicName)
  {
    return findEntity(publishersRegistry_[node], topicName);
  }

  /**
   * @brief Register the newly created Subscription in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param node Node instance identity
   * @param topicName Topic name
   * @param sub       Newly created Subscription object
   */
  template <typename MessageT>
  void registerSubscription(
    const NodeId & node,
    const TopicNameT & topicName,
    std::weak_ptr<rclcpp::SubscriptionBase> sub)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerSubscription<"
                << boost::typeindex::type_id<MessageT>().pretty_name() << ">(\""
                << node.lock().get() << "\", \"" << topicName << "\")\n";
    }
    registerEntity(subscriptionsRegistry_[node], topicName, sub);
  }

  /**
   * @brief Get list of all subscriptions created by the selected Node.
   *
   * @param node
   * @return std::vector<std::weak_ptr<rclcpp::SubscriptionBase>>
   */
  std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> getNodeSubscriptions(const NodeId & node)
  {
    std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> subscriptions{};
    for (auto [topicName, subscription] : subscriptionsRegistry_[node]) {
      subscriptions.push_back(subscription);
    }
    return subscriptions;
  }

  /**
   * @brief Get a subscription created by a selected Node for a particular Topic.
   *
   * @param node Node instance identity
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::SubscriptionBase>
   */
  std::weak_ptr<rclcpp::SubscriptionBase> getSubscription(
    const NodeId & node,
    const TopicNameT & topicName)
  {
    return findEntity(subscriptionsRegistry_[node], topicName);
  }

  /**
   * @brief Register the newly created Timer in the regisrtry.
   * This function shall be used by the rclcpp::create_timer() function only.
   *
   * @param node Node instance identity
   * @param timer    Newly created Timer object
   * @return true
   * @return false
   */
  bool registerTimer(const NodeId & node, std::weak_ptr<rclcpp::TimerBase> timer)
  {
    timersRegistry_[node].push_back(timer);
    return true;
  }

  /**
   * @brief Get list of all Timers created by the selected Node.
   *
   * @param node Node instance identity
   * @return std::vector<std::weak_ptr<rclcpp::TimerBase>>
   */
  std::vector<std::weak_ptr<rclcpp::TimerBase>> getTimers(const NodeId & node)
  {
    return findEntity(timersRegistry_, node);
  }

  /**
   * @brief Enable additional verbose logs to trace registry events.
   *
   * @param on
   */
  void enableVerboseLogs(bool on) { verbose_ = on; }

  std::weak_ptr<MockBase> getMock(void * ptr)
  {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      return it->second;
    }
    return {};
  }

  void attachMock(void * ptr, std::weak_ptr<MockBase> mock) { mockRegistry_[ptr] = mock; }

  void detachMock(void * ptr)
  {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      mockRegistry_.erase(it);
    }
  }

  /**
   * @brief Register the newly created Service in the registry.
   */
  template <typename ServiceT>
  void registerService(
    const NodeId & node,
    const ServiceNameT & serviceName,
    std::weak_ptr<rclcpp::ServiceBase> service)
  {
    const char * namePtr = serviceName.c_str();
    if (!serviceName.empty() && serviceName[0] == '/') {
      namePtr++;
    }

    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerService<"
                << boost::typeindex::type_id<ServiceT>().pretty_name() << ">(\""
                << node.lock().get() << "\", \"" << serviceName << "\")\n";
    }
    registerEntity(servicesRegistry_[node], namePtr, service);
  }

  /**
   * @brief Get list of all services created by the selected Node.
   */
  std::vector<std::weak_ptr<rclcpp::ServiceBase>> getNodeServices(const NodeId & node)
  {
    std::vector<std::weak_ptr<rclcpp::ServiceBase>> services{};
    for (auto [serviceName, service] : servicesRegistry_[node]) {
      services.push_back(service);
    }
    return services;
  }

  /**
   * @brief Get a service created by a selected Node.
   */
  std::weak_ptr<rclcpp::ServiceBase> getService(
    const NodeId & node,
    const ServiceNameT & serviceName)
  {
    return findEntity(servicesRegistry_[node], serviceName);
  }

  template <typename ServiceT>
  void registerServiceClient(
    const NodeId & node,
    const ServiceNameT & serviceName,
    std::weak_ptr<rclcpp::ClientBase> client)
  {
    const char * namePtr = serviceName.c_str();
    if (!serviceName.empty() && serviceName[0] == '/') {
      namePtr++;
    }

    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerServiceClient<"
                << boost::typeindex::type_id<ServiceT>().pretty_name() << ">(\""
                << node.lock().get() << "\", \"" << serviceName << "\")\n";
    }
    registerEntity(serviceClientsRegistry_[node], namePtr, client);
  }

  std::vector<std::weak_ptr<rclcpp::ClientBase>> getNodeServiceClients(const NodeId & node)
  {
    std::vector<std::weak_ptr<rclcpp::ClientBase>> clients{};
    for (auto [serviceName, client] : serviceClientsRegistry_[node]) {
      clients.push_back(client);
    }
    return clients;
  }

  std::weak_ptr<rclcpp::ClientBase> getServiceClient(
    const NodeId & node,
    const ServiceNameT & serviceName)
  {
    return findEntity(serviceClientsRegistry_[node], serviceName);
  }

  template <typename ActionT>
  void registerActionServer(
    const NodeId & node,
    const ActionNameT & actionName,
    std::weak_ptr<rclcpp_action::ServerBase> server)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerActionServer<"
                << boost::typeindex::type_id<ActionT>().pretty_name() << ">(\"" << node.lock().get()
                << "\", \"" << actionName << "\")\n";
    }
    registerEntity(actionServersRegistry_[node], actionName, server);
  }

  template <typename ActionT>
  void registerActionClient(
    const NodeId & node,
    const ActionNameT & actionName,
    std::weak_ptr<rclcpp_action::ClientBase> client)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerActionClient<"
                << boost::typeindex::type_id<ActionT>().pretty_name() << ">(\"" << node.lock().get()
                << "\", \"" << actionName << "\")\n";
    }
    registerEntity(actionClientsRegistry_[node], actionName, client);
  }

  std::weak_ptr<rclcpp_action::ServerBase> getActionServer(
    const NodeId & node,
    const ActionNameT & actionName)
  {
    tryLazyInit(lazy_init_action_servers_);
    return findEntity(actionServersRegistry_[node], actionName);
  }

  void tryLazyInit(std::vector<LazyInitEntry> & lazyInitVector)
  {
    std::lock_guard<std::mutex> lock(lazy_init_mutex_);

    for (auto it = lazyInitVector.begin(); it != lazyInitVector.end();) {
      try {
        it->init_callback();
        it = lazyInitVector.erase(it);
      } catch (const std::exception & e) {
        ++it;
      }
    }
  }

  std::weak_ptr<rclcpp_action::ClientBase> getActionClient(
    const NodeId & node,
    const ActionNameT & actionName)
  {
    tryLazyInit(lazy_init_action_clients_);
    return findEntity(actionClientsRegistry_[node], actionName);
  }

  void registerLazyInitClient(
    void * raw_ptr,
    const std::string & node_name,
    const std::string & action_name,
    std::function<void()> callback)
  {
    std::lock_guard<std::mutex> lock(lazy_init_mutex_);
    lazy_init_action_clients_.push_back({raw_ptr, node_name, action_name, std::move(callback)});

    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerLazyInitClient - " << "Node: '" << node_name
                << "', Action: '" << action_name << "'" << std::endl;
    }
  }

  void removeLazyInitClient(void * raw_ptr)
  {
    std::lock_guard<std::mutex> lock(lazy_init_mutex_);
    lazy_init_action_clients_.erase(
      std::remove_if(
        lazy_init_action_clients_.begin(),
        lazy_init_action_clients_.end(),
        [raw_ptr](const LazyInitEntry & entry) { return entry.raw_ptr == raw_ptr; }),
      lazy_init_action_clients_.end());
  }

  void registerLazyInitServer(
    void * raw_ptr,
    const std::string & node_name,
    const std::string & action_name,
    std::function<void()> callback)
  {
    std::lock_guard<std::mutex> lock(lazy_init_mutex_);
    lazy_init_action_servers_.push_back({raw_ptr, node_name, action_name, std::move(callback)});

    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerLazyInitServer - " << "Node: '" << node_name
                << "', Action: '" << action_name << "'" << std::endl;
    }
  }

  void removeLazyInitServer(void * raw_ptr)
  {
    std::lock_guard<std::mutex> lock(lazy_init_mutex_);
    lazy_init_action_servers_.erase(
      std::remove_if(
        lazy_init_action_servers_.begin(),
        lazy_init_action_servers_.end(),
        [raw_ptr](const LazyInitEntry & entry) { return entry.raw_ptr == raw_ptr; }),
      lazy_init_action_servers_.end());
  }

  void reset()
  {
    publishersRegistry_.clear();
    subscriptionsRegistry_.clear();
    timersRegistry_.clear();
    servicesRegistry_.clear();
    serviceClientsRegistry_.clear();
    actionServersRegistry_.clear();
    actionClientsRegistry_.clear();
    mockRegistry_.clear();
    lazy_init_action_clients_.clear();
    lazy_init_action_servers_.clear();
  }

private:
  StaticMocksRegistry()
  {
    ::testing::UnitTest::GetInstance()->listeners().Append(new MockRegistryCleaner());
  }

  static StaticMocksRegistry theRegistry_;

  template <typename RegistryT, typename EntityT>
  void registerEntity(RegistryT & reg, const TopicNameT & topicName, EntityT e)
  {
    if (!reg[topicName].lock()) {
      reg[topicName] = e;
    }
  }

  template <typename RegistryT, typename KeyT>
  typename RegistryT::value_type::second_type findEntity(RegistryT & reg, const KeyT & topicName)
  {
    auto it = reg.find(topicName);
    if (it != reg.end()) {
      return it->second;
    } else {
      return {};
    }
  }

  NodeRegistry<TopicToPublishersMapT> publishersRegistry_;
  NodeRegistry<TopicToSubscriptionsMapT> subscriptionsRegistry_;
  NodeRegistry<std::vector<std::weak_ptr<rclcpp::TimerBase>>> timersRegistry_;
  NodeRegistry<ServiceToServicesMapT> servicesRegistry_;
  NodeRegistry<ServiceToClientsMapT> serviceClientsRegistry_;
  NodeRegistry<ActionToServersMapT> actionServersRegistry_;
  NodeRegistry<ActionToClientsMapT> actionClientsRegistry_;

  std::map<void *, std::weak_ptr<MockBase>> mockRegistry_;

  std::vector<LazyInitEntry> lazy_init_action_clients_;
  std::vector<LazyInitEntry> lazy_init_action_servers_;
  std::mutex lazy_init_mutex_;

  bool verbose_{false};
};

void enableVerboseLogs(bool on);

}  // namespace rtest
