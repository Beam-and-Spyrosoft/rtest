// Copyright 2026 Spyrosoft Limited.
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
// @file      service_client_pending_requests_test.cpp
// @date      2026-10-02
//
// @brief     Verifies that the mocked rclcpp::Client exposes remove_pending_request() and
//            prune_pending_requests() and forwards them to ServiceClientMock (GitHub issue #128).

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rtest/static_registry.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <vector>

namespace
{
using Empty = std_srvs::srv::Empty;
using Types = rclcpp::ClientTypes<Empty>;
using ::testing::Return;

// Distinct from std::allocator, to exercise the templated Client::prune_requests_older_than().
template <typename T>
struct CustomAllocator
{
  using value_type = T;

  CustomAllocator() = default;
  template <typename U>
  explicit CustomAllocator(const CustomAllocator<U> &)
  {
  }

  T * allocate(size_t n) { return std::allocator<T>{}.allocate(n); }
  void deallocate(T * p, size_t n) { std::allocator<T>{}.deallocate(p, n); }

  friend bool operator==(const CustomAllocator &, const CustomAllocator &) { return true; }
  friend bool operator!=(const CustomAllocator &, const CustomAllocator &) { return false; }
};

// Production-style blocking helper, taken verbatim from the rclcpp documentation pattern.
template <typename ServiceT>
typename ServiceT::Response::SharedPtr CallService(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  std::chrono::nanoseconds timeout)
{
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    return nullptr;
  }
  return future.get();
}

class ServiceClientPendingRequestsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);
    options.enable_rosout(false);
    node_ = std::make_shared<rclcpp::Node>("pending_requests_node", options);
    client_ = node_->create_client<Empty>("pending_requests_service");
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<Empty>::SharedPtr client_;
};

TEST_F(ServiceClientPendingRequestsTest, WhenRequestTimesOut_ThenHelperRemovesItByRequestId)
{
  auto client_mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(client_mock);

  // The promise outlives the call, so the future never becomes ready: a timed out request.
  std::promise<Types::SharedResponse> never_fulfilled;
  EXPECT_CALL(*client_mock, async_send_request(::testing::_)).WillOnce([&never_fulfilled](auto) {
    return Types::FutureResponseAndId(never_fulfilled.get_future(), 42);
  });
  EXPECT_CALL(*client_mock, remove_pending_request(42)).WillOnce(Return(true));

  auto response =
    CallService<Empty>(client_, std::make_shared<Empty::Request>(), std::chrono::milliseconds(1));
  EXPECT_FALSE(response);
}

TEST_F(ServiceClientPendingRequestsTest, WhenRequestCompletes_ThenNothingIsRemoved)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  EXPECT_CALL(*mock, async_send_request(::testing::_)).WillOnce([](auto) {
    std::promise<Types::SharedResponse> promise;
    promise.set_value(std::make_shared<Empty::Response>());
    return Types::FutureResponseAndId(promise.get_future(), 1);
  });
  EXPECT_CALL(*mock, remove_pending_request(::testing::_)).Times(0);

  auto response =
    CallService<Empty>(client_, std::make_shared<Empty::Request>(), std::chrono::milliseconds(1));
  EXPECT_TRUE(response);
}

TEST_F(ServiceClientPendingRequestsTest, AllOverloadsForwardTheRequestId)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  EXPECT_CALL(*mock, remove_pending_request(1)).WillOnce(Return(true));
  EXPECT_CALL(*mock, remove_pending_request(2)).WillOnce(Return(true));
  EXPECT_CALL(*mock, remove_pending_request(3)).WillOnce(Return(false));
  EXPECT_CALL(*mock, remove_pending_request(4)).WillOnce(Return(true));

  EXPECT_TRUE(client_->remove_pending_request(int64_t{1}));
  EXPECT_TRUE(client_->remove_pending_request(
    Types::FutureResponseAndId(std::future<Types::SharedResponse>{}, 2)));
  EXPECT_FALSE(
    client_->remove_pending_request(Types::SharedFutureResponseAndId(Types::SharedFuture{}, 3)));
  EXPECT_TRUE(client_->remove_pending_request(
    Types::SharedFutureWithRequestAndId(Types::SharedFutureWithRequest{}, 4)));
}

TEST_F(ServiceClientPendingRequestsTest, PrunePendingRequestsForwardsToMock)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  EXPECT_CALL(*mock, prune_pending_requests()).WillOnce(Return(3));

  EXPECT_EQ(client_->prune_pending_requests(), 3u);
}

TEST_F(ServiceClientPendingRequestsTest, PruneRequestsOlderThanForwardsTimePoint)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  const auto time_point = std::chrono::system_clock::now();
  EXPECT_CALL(*mock, prune_requests_older_than(time_point, nullptr)).WillOnce(Return(2));

  EXPECT_EQ(client_->prune_requests_older_than(time_point), 2u);
}

TEST_F(ServiceClientPendingRequestsTest, PruneRequestsOlderThanReportsPrunedIds)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  const auto time_point = std::chrono::system_clock::now();
  EXPECT_CALL(*mock, prune_requests_older_than(time_point, ::testing::NotNull()))
    .WillOnce(::testing::DoAll(::testing::SetArgPointee<1>(std::vector<int64_t>{5, 7}), Return(2)));

  std::vector<int64_t> pruned;
  EXPECT_EQ(client_->prune_requests_older_than(time_point, &pruned), 2u);
  EXPECT_THAT(pruned, ::testing::ElementsAre(5, 7));
}

TEST_F(ServiceClientPendingRequestsTest, PruneRequestsOlderThanAppendsIdsToCustomAllocatorVector)
{
  auto mock = rtest::findServiceClient<Empty>(node_, "pending_requests_service");
  ASSERT_TRUE(mock);

  const auto time_point = std::chrono::system_clock::now();
  EXPECT_CALL(*mock, prune_requests_older_than(time_point, ::testing::NotNull()))
    .WillOnce(::testing::DoAll(::testing::SetArgPointee<1>(std::vector<int64_t>{5, 7}), Return(2)));

  // Like rclcpp, pruned ids are appended to what the vector already holds
  std::vector<int64_t, CustomAllocator<int64_t>> pruned{1};
  EXPECT_EQ(client_->prune_requests_older_than(time_point, &pruned), 2u);
  EXPECT_THAT(pruned, ::testing::ElementsAre(1, 5, 7));
}

TEST_F(ServiceClientPendingRequestsTest, WithoutAttachedMock_ThenCleanupIsNoOp)
{
  EXPECT_FALSE(client_->remove_pending_request(int64_t{1}));
  EXPECT_EQ(client_->prune_pending_requests(), 0u);
  EXPECT_EQ(client_->prune_requests_older_than(std::chrono::system_clock::now()), 0u);
}

}  // namespace
