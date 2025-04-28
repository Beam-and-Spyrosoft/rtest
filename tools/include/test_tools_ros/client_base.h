/**
 * @file      client_base.h
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

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rcl/service_introspection.h"
#include "rcl/wait.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

namespace detail {
template <typename FutureT>
struct FutureAndRequestId {
  FutureT future;
  int64_t request_id;

  FutureAndRequestId(FutureT impl, int64_t req_id) : future(std::move(impl)), request_id(req_id) {}

  /// Allow implicit conversions to `std::future` by reference.
  operator FutureT &() { return this->future; }

  // delegate future like methods in the std::future impl_

  /// See std::future::get().
  auto get() { return this->future.get(); }
  /// See std::future::valid().
  bool valid() const noexcept { return this->future.valid(); }
  /// See std::future::wait().
  void wait() const { return this->future.wait(); }
  /// See std::future::wait_for().
  template <class Rep, class Period>
  std::future_status wait_for(const std::chrono::duration<Rep, Period> &timeout_duration) const {
    return this->future.wait_for(timeout_duration);
  }
  /// See std::future::wait_until().
  template <class Clock, class Duration>
  std::future_status wait_until(const std::chrono::time_point<Clock, Duration> &timeout_time) const {
    return this->future.wait_until(timeout_time);
  }

  // Rule of five, we could use the rule of zero here, but better be explicit as some of the
  // methods are deleted.

  /// Move constructor.
  FutureAndRequestId(FutureAndRequestId &&other) noexcept = default;
  /// Deleted copy constructor, each instance is a unique owner of the future.
  FutureAndRequestId(const FutureAndRequestId &other) = delete;
  /// Move assignment.
  FutureAndRequestId &operator=(FutureAndRequestId &&other) noexcept = default;
  /// Deleted copy assignment, each instance is a unique owner of the future.
  FutureAndRequestId &operator=(const FutureAndRequestId &other) = delete;
  /// Destructor.
  ~FutureAndRequestId() = default;
};

template <typename PendingRequestsT, typename AllocatorT = std::allocator<int64_t>>
size_t prune_requests_older_than_impl(
    PendingRequestsT &pending_requests,
    std::mutex &pending_requests_mutex,
    std::chrono::time_point<std::chrono::system_clock> time_point,
    std::vector<int64_t, AllocatorT> *pruned_requests = nullptr) {
  std::lock_guard guard(pending_requests_mutex);
  auto old_size = pending_requests.size();
  for (auto it = pending_requests.begin(), last = pending_requests.end(); it != last;) {
    if (it->second.first < time_point) {
      if (pruned_requests) {
        pruned_requests->push_back(it->first);
      }
      it = pending_requests.erase(it);
    } else {
      ++it;
    }
  }
  return old_size - pending_requests.size();
}
}  // namespace detail

namespace node_interfaces {
class NodeBaseInterface;
}  // namespace node_interfaces

class ClientBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase)

  RCLCPP_PUBLIC
  ClientBase(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph);

  RCLCPP_PUBLIC
  virtual ~ClientBase() = default;

  /// Take the next response for this client as a type erased pointer.
  /**
   * The type erased pointer allows for this method to be used in a type
   * agnostic way along with ClientBase::create_response(),
   * ClientBase::create_request_header(), and ClientBase::handle_response().
   * The typed version of this can be used if the Service type is known,
   * \sa Client::take_response().
   *
   * \param[out] response_out The type erased pointer to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  RCLCPP_PUBLIC
  bool take_type_erased_response(void *response_out, rmw_request_id_t &request_header_out);

  /// Return the name of the service.
  /** \return The name of the service. */
  RCLCPP_PUBLIC
  const char *get_service_name() const;

  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Client is destroyed.
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_client_t> get_client_handle();

  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * This handle remains valid after the Client is destroyed.
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_client_t> get_client_handle() const;

  /// Return if the service is ready.
  /**
   * \return `true` if the service is ready, `false` otherwise
   */
  RCLCPP_PUBLIC
  bool service_is_ready() const;

  /// Wait for a service to be ready.
  /**
   * \param timeout maximum time to wait
   * \return `true` if the service is ready and the timeout is not over, `false` otherwise
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    return wait_for_service_nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual std::shared_ptr<void> create_response() = 0;
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;
  virtual void handle_response(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

  /// Exchange the "in use by wait set" state for this client.
  /**
   * This is used to ensure this client is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

  /// Get the actual request publsher QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual request publsher qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_request_publisher_actual_qos() const;

  /// Get the actual response subscription QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual response subscription qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_response_subscription_actual_qos() const;

  /// Set a callback to be called when each new response is received.
  /**
   * The callback receives a size_t which is the number of responses received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if responses were received before any
   * callback was set.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the client
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_client_set_on_new_response_callback
   * \sa rcl_client_set_on_new_response_callback
   *
   * \param[in] callback functor to be called when a new response is received
   */
  void set_on_new_response_callback(std::function<void(size_t)> callback) {
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_response_callback "
          "is not callable.");
    }

    auto new_callback = [callback, this](size_t number_of_responses) {
      try {
        callback(number_of_responses);
      } catch (const std::exception &exception) {
        RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ClientBase@" << this << " caught " << rmw::impl::cpp::demangle(exception)
                                  << " exception in user-provided callback for the 'on new response' callback: "
                                  << exception.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ClientBase@" << this << " caught unhandled exception in user-provided callback "
                                  << "for the 'on new response' callback");
      }
    };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_response_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(on_new_response_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_response_callback_));
  }

  /// Unset the callback registered for new responses, if any.
  void clear_on_new_response_callback() {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (on_new_response_callback_) {
      set_on_new_response_callback(nullptr, nullptr);
      on_new_response_callback_ = nullptr;
    }
  }

protected:
  RCLCPP_DISABLE_COPY(ClientBase)

  RCLCPP_PUBLIC
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  rcl_node_t *get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *get_rcl_node_handle() const;

  RCLCPP_PUBLIC
  void set_on_new_response_callback(rcl_event_callback_t callback, const void *user_data);

  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  std::shared_ptr<rcl_node_t> node_handle_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Logger node_logger_;

  std::recursive_mutex callback_mutex_;
  // It is important to declare on_new_response_callback_ before
  // client_handle_, so on destruction the client is
  // destroyed first. Otherwise, the rmw client callback
  // would point briefly to a destroyed function.
  std::function<void(size_t)> on_new_response_callback_{nullptr};
  // Declare client_handle_ after callback
  std::shared_ptr<rcl_client_t> client_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};
};

template <typename ServiceT>
struct ClientTypes {
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

  using Future = std::future<SharedResponse>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using FutureResponseAndId = struct FutureAndRequestId : detail::FutureAndRequestId<Future> {
    using detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };

  using SharedFutureResponseAndId = struct SharedFutureAndRequestId : detail::FutureAndRequestId<SharedFuture> {
    using detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  using SharedFutureWithRequestAndId =
      struct SharedFutureWithRequestAndRequestId : detail::FutureAndRequestId<SharedFutureWithRequest> {
    using detail::FutureAndRequestId<SharedFutureWithRequest>::FutureAndRequestId;
  };

  using CallbackType = std::function<void(SharedFuture)>;
  using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;
};

}  // namespace rclcpp