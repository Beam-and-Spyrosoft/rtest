/**
 * @file      service_provider_tests.cc
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

#include <test_composition/service_provider.h>

class ServiceProviderTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ServiceProviderTest, WhenServiceRequestReceived_ThenStateIsUpdated) {
  auto node = std::make_shared<test_composition::ServiceProvider>(opts);

  // Retrieve the service created by the Node
  auto service = test_tools_ros::findService<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Service
  ASSERT_TRUE(service);

  // Initial state should be false
  EXPECT_FALSE(node->getState());

  // Set up test request and response
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  std_srvs::srv::SetBool::Response response;

  // Set up expectation that service will handle request and set proper response
  EXPECT_CALL(*service, handle(::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(
          ::testing::Invoke(
              [](const std_srvs::srv::SetBool::Request::SharedPtr /*req*/, std_srvs::srv::SetBool::Response &resp) {
                resp.success = true;
                resp.message = "State updated successfully";
              }),
          ::testing::Return()));

  // Simulate service call
  service->handle(request, response);

  // Verify response
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.message, "State updated successfully");
}

TEST_F(ServiceProviderTest, WhenServiceRequestReceivedError_ThenStateIsUpdated) {
  auto node = std::make_shared<test_composition::ServiceProvider>(opts);

  // Retrieve the service created by the Node
  auto service = test_tools_ros::findService<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Service
  ASSERT_TRUE(service);

  // Initial state should be false
  EXPECT_FALSE(node->getState());

  // Set up test request and response
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  std_srvs::srv::SetBool::Response response;

  // Set up expectation that service will handle request and response error
  EXPECT_CALL(*service, handle(::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(
          ::testing::Invoke(
              [](const std_srvs::srv::SetBool::Request::SharedPtr /*req*/, std_srvs::srv::SetBool::Response &resp) {
                resp.success = false;
                resp.message = "Error: Invalid request";
              }),
          ::testing::Return()));

  // Simulate service call
  service->handle(request, response);

  // Verify response
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message, "Error: Invalid request");
}