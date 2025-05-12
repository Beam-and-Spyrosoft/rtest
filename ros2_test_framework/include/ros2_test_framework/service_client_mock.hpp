/**
 * @file      service_client_mock.hpp
 * @author    Mariusz Szczepanik (mua@spyro-soft.com)
 * @date      2025-05-28
 * @copyright Copyright (c) 2025 Spyrosoft Limited.
 *
 * @brief
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
#include <ros2_test_framework/static_registry.hpp>
#include <ros2_test_framework/client_base.hpp>

#include <memory>
#include <string>
#include <utility>
#include <future>
#include <type_traits>

#include "rcl/error_handling.h"
#include "rcl/client.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rclcpp/client.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"

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

namespace ros2_test_framework
{

template <typename ServiceT>
class ServiceClientMock : public MockBase
{
public:
  using Types = rclcpp::ClientTypes<ServiceT>;
  using FutureAndRequestId = typename Types::FutureResponseAndId;
  using SharedFutureAndRequestId = typename Types::SharedFutureAndRequestId;
  using SharedFutureWithRequestAndRequestId = typename Types::SharedFutureWithRequestAndRequestId;

  ServiceClientMock(rclcpp::ClientBase * client) : client_(client) {}
  ~ServiceClientMock() { StaticMocksRegistry::instance().detachMock(client_); }

  TEST_TOOLS_SMART_PTR_DEFINITIONS(ServiceClientMock<ServiceT>)

  MOCK_METHOD(FutureAndRequestId, async_send_request, (typename Types::SharedRequest), ());
  MOCK_METHOD(
    SharedFutureAndRequestId,
    async_send_request_with_callback,
    (typename Types::SharedRequest, typename Types::CallbackType),
    ());
  MOCK_METHOD(
    SharedFutureWithRequestAndRequestId,
    async_send_request_with_callback_and_request,
    (typename Types::SharedRequest, typename Types::CallbackWithRequestType),
    ());
  MOCK_METHOD(bool, service_is_ready, (), ());

private:
  rclcpp::ClientBase * client_{nullptr};
};

}  // namespace ros2_test_framework

namespace rclcpp
{

template <typename ServiceT>
class Client : public ClientBase, public std::enable_shared_from_this<Client<ServiceT>>
{
public:
  using Types = rclcpp::ClientTypes<ServiceT>;

  using Request = typename Types::Request;
  using Response = typename Types::Response;
  using SharedRequest = typename Types::SharedRequest;
  using SharedResponse = typename Types::SharedResponse;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;
  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

  using FutureAndRequestId = typename Types::FutureAndRequestId;
  using SharedFutureAndRequestId = typename Types::SharedFutureAndRequestId;
  using SharedFutureWithRequestAndRequestId = typename Types::SharedFutureWithRequestAndRequestId;

  using CallbackType = typename Types::CallbackType;
  using CallbackWithRequestType = typename Types::CallbackWithRequestType;

  TEST_TOOLS_SMART_PTR_DEFINITIONS(Client<ServiceT>)

  Client(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> & node_graph,
    const std::string & service_name,
    rcl_client_options_t & options)
  : ClientBase(node_base, node_graph), service_name_(service_name)
  {
    fully_qualified_name_ = node_base->get_fully_qualified_name();
  }

  virtual ~Client() = default;

  std::shared_ptr<void> create_response() override
  {
    return std::shared_ptr<void>(new typename ServiceT::Response());
  }

  std::shared_ptr<rmw_request_id_t> create_request_header() override
  {
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response) override
  {
    (void)request_header;
    (void)response;
    throw std::runtime_error("not implemented");
  }

  FutureAndRequestId async_send_request(SharedRequest request)
  {
    auto mock = ros2_test_framework::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<ros2_test_framework::ServiceClientMock<ServiceT>>(mock)
        ->async_send_request(request);
    }
    throw std::runtime_error("No mock attached");
  }

  SharedFutureAndRequestId async_send_request(SharedRequest request, CallbackType cb)
  {
    auto mock = ros2_test_framework::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<ros2_test_framework::ServiceClientMock<ServiceT>>(mock)
        ->async_send_request_with_callback(request, cb);
    }
    throw std::runtime_error("No mock attached");
  }

  SharedFutureWithRequestAndRequestId async_send_request(
    SharedRequest request,
    CallbackWithRequestType cb)
  {
    auto mock = ros2_test_framework::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<ros2_test_framework::ServiceClientMock<ServiceT>>(mock)
        ->async_send_request_with_callback_and_request(request, cb);
    }
    throw std::runtime_error("No mock attached");
  }

  bool service_is_ready()
  {
    auto mock = ros2_test_framework::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      return std::static_pointer_cast<ros2_test_framework::ServiceClientMock<ServiceT>>(mock)
        ->service_is_ready();
    }
    return false;
  }

  void post_init_setup()
  {
    ros2_test_framework::StaticMocksRegistry::instance().template registerServiceClient<ServiceT>(
      fully_qualified_name_, service_name_, this->template weak_from_this());
  }

private:
  RCLCPP_DISABLE_COPY(Client)

  std::shared_ptr<rcl_node_t> node_handle_;
  std::string fully_qualified_name_;
  std::string service_name_;
};

}  // namespace rclcpp

namespace ros2_test_framework
{

template <typename ServiceT>
std::shared_ptr<ServiceClientMock<ServiceT>> findServiceClient(
  const std::string & fullyQualifiedNodeName,
  const std::string & serviceName)
{
  std::shared_ptr<ServiceClientMock<ServiceT>> client_mock{};
  auto client_base =
    StaticMocksRegistry::instance().getServiceClient(fullyQualifiedNodeName, serviceName).lock();

  if (client_base) {
    if (StaticMocksRegistry::instance().getMock(client_base.get()).lock()) {
      std::cerr << "ros2_test_framework::findServiceClient() WARNING: ServiceClientMock already "
                   "attached to the Client\n";
    } else {
      client_mock = std::make_shared<ServiceClientMock<ServiceT>>(client_base.get());
      StaticMocksRegistry::instance().attachMock(client_base.get(), client_mock);
    }
  }
  return client_mock;
}

template <typename ServiceT, typename NodeT>
std::shared_ptr<ServiceClientMock<ServiceT>> findServiceClient(
  const std::shared_ptr<NodeT> nodePtr,
  const std::string & serviceName)
{
  const char * namePtr = serviceName.c_str();
  if (!serviceName.empty() && serviceName[0] == '/') {
    namePtr++;
  }
  return findServiceClient<ServiceT>(nodePtr->get_fully_qualified_name(), namePtr);
}

}  // namespace ros2_test_framework