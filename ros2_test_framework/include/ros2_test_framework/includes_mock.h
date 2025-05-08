/**
 * @file      includes_mock.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-21
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief    Mock header for ROS 2 tools.
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

// First, disable original implementations
#include <ros2_test_framework/mock_defines.h>

// Then include our mocks
#ifdef ROS2_TEST_FRAMEWORK_SERVICE_MOCK
#include <ros2_test_framework/service_base.h>
#include <ros2_test_framework/service_mock.h>
#endif

#ifdef ROS2_TEST_FRAMEWORK_CLIENT_MOCK
#include <ros2_test_framework/client_base.h>
#include <ros2_test_framework/service_client_mock.h>
#endif

#if defined(ROS2_TEST_FRAMEWORK_PUBLISHER_MOCK) && defined(ROS2_TEST_FRAMEWORK_SUBSCRIPTION_MOCK)
#include <ros2_test_framework/publisher_mock.h>
#include <ros2_test_framework/subscription_mock.h>
#elif defined(ROS2_TEST_FRAMEWORK_PUBLISHER_MOCK)
#include <ros2_test_framework/publisher_mock.h>
#elif defined(ROS2_TEST_FRAMEWORK_SUBSCRIPTION_MOCK)
#include <ros2_test_framework/subscription_mock.h>
#endif

#ifdef ROS2_TEST_FRAMEWORK_TIMER_MOCK
#include <ros2_test_framework/create_timer_mock.h>
#endif