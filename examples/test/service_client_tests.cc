/**
 * @file      service_client_tests.cc
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

#include <gtest/gtest.h>
#include <test_composition/service_client.h>

class ServiceClientTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ServiceClientTest, WhenServiceNotAvailable_ThenSetStateFails) {
  auto node = std::make_shared<test_composition::ServiceClient>(opts);

  // Retrieve the client created by the Node
  auto client = ros2_test_framework::findServiceClient<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Client
  ASSERT_TRUE(client);

  // Set up expectation that service is not available
  EXPECT_CALL(*client, service_is_ready()).WillOnce(::testing::Return(false));

  EXPECT_CALL(*client, async_send_request(::testing::_)).Times(0);

  // Attempt to set state should fail
  EXPECT_FALSE(node->setState(true));
  EXPECT_FALSE(node->getLastCallSuccess());
  EXPECT_EQ(node->getLastResponseMessage(), "Service not available");
}

TEST_F(ServiceClientTest, WhenServiceCallSucceeds_ThenSetStateSucceeds) {
  // Create node
  auto node = std::make_shared<test_composition::ServiceClient>(opts);

  // Retrieve the client created by the Node
  auto client = ros2_test_framework::findServiceClient<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Client
  ASSERT_TRUE(client);

  // Create successful response
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  response->success = true;
  response->message = "State updated successfully";

  // Set up expectations
  EXPECT_CALL(*client, service_is_ready()).WillOnce(::testing::Return(true));

  EXPECT_CALL(*client, async_send_request(::testing::_))
      .WillOnce([response](std::shared_ptr<std_srvs::srv::SetBool::Request>) {
        // Create a new promise and future for each call
        std::promise<std::shared_ptr<std_srvs::srv::SetBool::Response>> promise;
        promise.set_value(response);

        return rclcpp::ClientTypes<std_srvs::srv::SetBool>::FutureResponseAndId(
            promise.get_future(),  // Move the future directly into constructor
            1UL                    // Request ID
        );
      });

  // Attempt to set state should succeed
  EXPECT_TRUE(node->setState(true));
  EXPECT_TRUE(node->getLastCallSuccess());
  EXPECT_EQ(node->getLastResponseMessage(), "State updated successfully");
}

TEST_F(ServiceClientTest, WhenServiceCallFails_ThenSetStateFails) {
  // Create node
  auto node = std::make_shared<test_composition::ServiceClient>(opts);

  // Retrieve the client created by the Node
  auto client = ros2_test_framework::findServiceClient<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Client
  ASSERT_TRUE(client);

  // Create failed response
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  response->success = false;
  response->message = "Failed to update state";

  // Set up expectations
  EXPECT_CALL(*client, service_is_ready()).WillOnce(::testing::Return(true));

  EXPECT_CALL(*client, async_send_request(::testing::_))
      .WillOnce([response](std::shared_ptr<std_srvs::srv::SetBool::Request>) {
        // Create a new promise and future for each call
        std::promise<std::shared_ptr<std_srvs::srv::SetBool::Response>> promise;
        promise.set_value(response);

        return rclcpp::ClientTypes<std_srvs::srv::SetBool>::FutureResponseAndId(
            promise.get_future(),  // Move the future directly into constructor
            1UL                    // Request ID
        );
      });

  // Attempt to set state should fail
  EXPECT_FALSE(node->setState(true));
  EXPECT_FALSE(node->getLastCallSuccess());
  EXPECT_EQ(node->getLastResponseMessage(), "Failed to update state");
}

TEST_F(ServiceClientTest, WhenServiceCallWithCallback_ThenSetStateSucceeds) {
  auto node = std::make_shared<test_composition::ServiceClient>(opts);
  auto client = ros2_test_framework::findServiceClient<std_srvs::srv::SetBool>(node, "/test_service");
  ASSERT_TRUE(client);

  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  response->success = true;
  response->message = "State updated with callback successfully";

  EXPECT_CALL(*client, service_is_ready()).WillOnce(::testing::Return(true));

  EXPECT_CALL(*client, async_send_request_with_callback(::testing::_, ::testing::_))
      .WillOnce([response](auto request, auto callback) {
        (void)request;
        std::promise<std::shared_ptr<std_srvs::srv::SetBool::Response>> promise;
        promise.set_value(response);
        auto shared_future = promise.get_future().share();
        callback(shared_future);
        return rclcpp::ClientTypes<std_srvs::srv::SetBool>::SharedFutureResponseAndId{shared_future, 1UL};
      });

  bool callback_called = false;
  EXPECT_TRUE(node->setStateWithCallback(true, [&callback_called, response](auto future) {
    callback_called = true;
    auto result = future.get();
    EXPECT_TRUE(result->success);
    EXPECT_EQ(result->message, "State updated with callback successfully");
  }));

  EXPECT_TRUE(callback_called);
  EXPECT_TRUE(node->getLastCallSuccess());
  EXPECT_EQ(node->getLastResponseMessage(), "State updated with callback successfully");
}

TEST_F(ServiceClientTest, WhenServiceCallWithRequestCallback_ThenSetStateSucceeds) {
  auto node = std::make_shared<test_composition::ServiceClient>(opts);
  auto client = ros2_test_framework::findServiceClient<std_srvs::srv::SetBool>(node, "/test_service");
  ASSERT_TRUE(client);

  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  response->success = true;
  response->message = "State updated with request callback successfully";

  EXPECT_CALL(*client, service_is_ready()).WillOnce(::testing::Return(true));

  EXPECT_CALL(*client, async_send_request_with_callback_and_request(::testing::_, ::testing::_))
      .WillOnce([response](auto request, auto callback) {
        auto pair = std::make_pair(request, response);
        std::promise<decltype(pair)> promise;
        promise.set_value(pair);
        auto shared_future = promise.get_future().share();
        callback(shared_future);
        return rclcpp::ClientTypes<std_srvs::srv::SetBool>::SharedFutureWithRequestAndRequestId{shared_future, 1UL};
      });

  bool callback_called = false;
  EXPECT_TRUE(node->setStateWithRequestCallback(true, [&callback_called](auto future) {
    callback_called = true;
    auto [request, response] = future.get();
    EXPECT_TRUE(request->data);
    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->message, "State updated with request callback successfully");
  }));

  EXPECT_TRUE(callback_called);
  EXPECT_TRUE(node->getLastCallSuccess());
  EXPECT_EQ(node->getLastResponseMessage(), "State updated with request callback successfully");
}