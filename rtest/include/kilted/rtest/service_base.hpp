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
// @file      service_base.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-06-04

// clang-format off
#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rcl/service.h"
#include "rcl/service_introspection.h"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

#include "tracetools/tracetools.h"

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
namespace rclcpp
{

class ServiceBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ServiceBase)

  RCLCPP_PUBLIC
  explicit ServiceBase(std::shared_ptr<rcl_node_t> node_handle);

  RCLCPP_PUBLIC
  virtual ~ServiceBase() = default;

  /// Return the name of the service.
  /** \return The name of the service. */
  RCLCPP_PUBLIC
  const char *
  get_service_name();

  /// Return the rcl_service_t service handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Service is destroyed.
   * The actual rcl service is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_service_t>
  get_service_handle();

  /// Return the rcl_service_t service handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Service is destroyed.
   * The actual rcl service is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_service_t>
  get_service_handle() const;

  /// Take the next request from the service as a type erased pointer.
  /**
   * This type erased version of \sa Service::take_request() is useful when
   * using the service in a type agnostic way with methods like
   * ServiceBase::create_request(), ServiceBase::create_request_header(), and
   * ServiceBase::handle_request().
   *
   * \param[out] request_out The type erased pointer to a service request object
   *   into which the middleware will copy the taken request.
   * \param[out] request_id_out The output id for the request which can be used
   *   to associate response with this request in the future.
   * \returns true if the request was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl calls fail.
   */
  RCLCPP_PUBLIC
  bool
  take_type_erased_request(void * request_out, rmw_request_id_t & request_id_out);

  virtual
  std::shared_ptr<void>
  create_request() = 0;

  virtual
  std::shared_ptr<rmw_request_id_t>
  create_request_header() = 0;

  virtual
  void
  handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request) = 0;

  /// Exchange the "in use by wait set" state for this service.
  /**
   * This is used to ensure this service is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool
  exchange_in_use_by_wait_set_state(bool in_use_state);

  /// Get the actual response publisher QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the service, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual response publisher qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_response_publisher_actual_qos() const;

  /// Get the actual request subscription QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the service, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual request subscription qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_request_subscription_actual_qos() const;

  /// Set a callback to be called when each new request is received.
  /**
   * The callback receives a size_t which is the number of requests received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if requests were received before any
   * callback was set.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the service
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_service_set_on_new_request_callback
   * \sa rcl_service_set_on_new_request_callback
   *
   * \param[in] callback functor to be called when a new request is received
   */
  void
  set_on_new_request_callback(std::function<void(size_t)> callback)
  {
    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_new_request_callback "
              "is not callable.");
    }

    auto new_callback =
      [callback, this](size_t number_of_requests) {
        try {
          callback(number_of_requests);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ServiceBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on new request' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ServiceBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on new request' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_request_callback(
      rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
      static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_request_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_request_callback(
      rclcpp::detail::cpp_callback_trampoline<
        decltype(on_new_request_callback_), const void *, size_t>,
      static_cast<const void *>(&on_new_request_callback_));
  }

  /// Unset the callback registered for new requests, if any.
  void
  clear_on_new_request_callback()
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (on_new_request_callback_) {
      set_on_new_request_callback(nullptr, nullptr);
      on_new_request_callback_ = nullptr;
    }
  }

protected:
  RCLCPP_DISABLE_COPY(ServiceBase)

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const;

  RCLCPP_PUBLIC
  void
  set_on_new_request_callback(rcl_event_callback_t callback, const void * user_data);

  std::shared_ptr<rcl_node_t> node_handle_;

  std::recursive_mutex callback_mutex_;
  // It is important to declare on_new_request_callback_ before
  // service_handle_, so on destruction the service is
  // destroyed first. Otherwise, the rmw service callback
  // would point briefly to a destroyed function.
  std::function<void(size_t)> on_new_request_callback_{nullptr};
  // Declare service_handle_ after callback
  std::shared_ptr<rcl_service_t> service_handle_;
  bool owns_rcl_handle_ = true;

  rclcpp::Logger node_logger_;

  std::atomic<bool> in_use_by_wait_set_{false};
};

}  // namespace rclcpp

// clang-format on