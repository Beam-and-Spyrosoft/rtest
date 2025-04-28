/**
 * @file      service_mock.h
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
#include <test_tools_ros/static_registry.h>
#include <test_tools_ros/service_base.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/service.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/exceptions.hpp"

#define TEST_TOOLS_MAKE_SHARED_DEFINITION(...)                             \
  template <typename... Args>                                              \
  static std::shared_ptr<__VA_ARGS__> make_shared(Args &&...args) {        \
    auto ptr = std::make_shared<__VA_ARGS__>(std::forward<Args>(args)...); \
    ptr->post_init_setup();                                                \
    return ptr;                                                            \
  }

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__)      \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__)        \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__)      \
  TEST_TOOLS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

namespace test_tools_ros {

template <typename ServiceT>
class ServiceMock : public MockBase {
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

  ServiceMock(rclcpp::ServiceBase *service) : service_(service) {}
  ~ServiceMock() { StaticMocksRegistry::instance().detachMock(service_); }

  TEST_TOOLS_SMART_PTR_DEFINITIONS(ServiceMock<ServiceT>)

  MOCK_METHOD(void, handle, (const SharedRequest &, Response &), ());

private:
  rclcpp::ServiceBase *service_{nullptr};
};

}  // namespace test_tools_ros

namespace rclcpp {

template <typename ServiceT>
class Service : public ServiceBase, public std::enable_shared_from_this<Service<ServiceT>> {
public:
  using CallbackType = std::function<
      void(const std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>;

  TEST_TOOLS_SMART_PTR_DEFINITIONS(Service)

  Service(
      std::shared_ptr<rcl_node_t> node_handle,
      const std::string &service_name,
      AnyServiceCallback<ServiceT> callback,
      rcl_service_options_t &service_options) :
        ServiceBase(node_handle), service_name_(service_name), any_callback_(callback) {
    const char *name = rcl_node_get_name(node_handle.get());
    const char *namespace_ = rcl_node_get_namespace(node_handle.get());
    if (std::string(namespace_) == "/") {
      fully_qualified_name_ = "/" + std::string(name);
    } else {
      fully_qualified_name_ = std::string(namespace_) + "/" + std::string(name);
    }
  }

  /// Called after construction to continue setup that requires shared_from_this().
  void post_init_setup() {
    test_tools_ros::StaticMocksRegistry::instance().template registerService<ServiceT>(
        fully_qualified_name_, service_name_, this->template weak_from_this());
  }

  std::shared_ptr<void> create_request() override { return std::make_shared<typename ServiceT::Request>(); }

  std::shared_ptr<rmw_request_id_t> create_request_header() override { return std::make_shared<rmw_request_id_t>(); }

  void handle_request(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto response = std::make_shared<typename ServiceT::Response>();

    auto mock = test_tools_ros::StaticMocksRegistry::instance().getMock(this).lock();
    if (mock) {
      std::static_pointer_cast<test_tools_ros::ServiceMock<ServiceT>>(mock)->handle(typed_request, *response);
    } else {
      response = any_callback_.dispatch(this->shared_from_this(), request_header, typed_request);
    }

    if (response) {
      send_response(*request_header, *response);
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  AnyServiceCallback<ServiceT> any_callback_;

  void send_response(rmw_request_id_t &req_id, typename ServiceT::Response &response) {
    rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, &response);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
    }
  }

private:
  std::string fully_qualified_name_;
  std::string service_name_;
};

}  // namespace rclcpp

namespace test_tools_ros {

template <typename ServiceT>
std::shared_ptr<ServiceMock<ServiceT>> findService(
    const std::string &fullyQualifiedNodeName,
    const std::string &serviceName) {
  std::shared_ptr<ServiceMock<ServiceT>> service_mock{};
  auto service_base = StaticMocksRegistry::instance().getService(fullyQualifiedNodeName, serviceName).lock();

  if (service_base) {
    if (StaticMocksRegistry::instance().getMock(service_base.get()).lock()) {
      std::cerr << "test_tools_ros::findService() WARNING: ServiceMock already attached to the Service\n";
    } else {
      service_mock = std::make_shared<ServiceMock<ServiceT>>(service_base.get());
      StaticMocksRegistry::instance().attachMock(service_base.get(), service_mock);
    }
  }
  return service_mock;
}

template <typename ServiceT, typename NodeT>
std::shared_ptr<ServiceMock<ServiceT>> findService(
    const std::shared_ptr<NodeT> nodePtr,
    const std::string &serviceName) {
  const char *namePtr = serviceName.c_str();
  if (!serviceName.empty() && serviceName[0] == '/') {
    namePtr++;
  }
  return findService<ServiceT>(nodePtr->get_fully_qualified_name(), namePtr);
}

}  // namespace test_tools_ros