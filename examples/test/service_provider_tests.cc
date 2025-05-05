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
  ASSERT_FALSE(node->getState());

  // Set up test request
  auto request_header = std::make_shared<rmw_request_id_t>();
  request_header->sequence_number = 123L;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Set up expected response
  std_srvs::srv::SetBool::Response expected_response;
  expected_response.success = true;
  expected_response.message = "State updated successfully";

  // Set up expectation that service will handle request and set proper response
  EXPECT_CALL(*service, send_response(*request_header, expected_response));

  // Simulate service call
  service->handle_request(request_header, request);

  // The state should be updated to true
  EXPECT_TRUE(node->getState());
}

TEST_F(ServiceProviderTest, WhenServiceIsLocked_ThenStateIsNotUpdated) {
  auto node = std::make_shared<test_composition::ServiceProvider>(opts);

  // Retrieve the service created by the Node
  auto service = test_tools_ros::findService<std_srvs::srv::SetBool>(node, "/test_service");

  // Check that the Node actually created the Service
  ASSERT_TRUE(service);

  // Initial state should be false
  ASSERT_FALSE(node->getState());

  node->lockService();

  // Set up test request
  auto request_header = std::make_shared<rmw_request_id_t>();
  request_header->sequence_number = 123L;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Set up expected response
  std_srvs::srv::SetBool::Response expected_response;
  expected_response.success = false;
  expected_response.message = "Service is locked, cannot update state";

  // Set up expectation that service will handle request and set proper response
  EXPECT_CALL(*service, send_response(*request_header, expected_response));

  // Simulate service call
  service->handle_request(request_header, request);

  // The state should not be updated because the service was locked
  EXPECT_FALSE(node->getState());

  // Unlock the service
  node->unlockService();

  // Set up expected response for unlocking
  expected_response.success = true;
  expected_response.message = "State updated successfully";

  // Set up expectation that service will handle request and set proper response
  EXPECT_CALL(*service, send_response(*request_header, expected_response));

  // Simulate service call again
  service->handle_request(request_header, request);

  // The state should be updated to true
  EXPECT_TRUE(node->getState());
}