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
// @file      includes_mock.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-21
//
// @brief     Mock header for ROS 2 tools.

#pragma once

// First, disable original implementations
#include <ros2_test_framework/mock_defines.hpp>

// Then include our mocks
#ifdef ROS2_TEST_FRAMEWORK_SERVICE_MOCK
#include <ros2_test_framework/service_base.hpp>
#include <ros2_test_framework/service_mock.hpp>
#endif

#ifdef ROS2_TEST_FRAMEWORK_CLIENT_MOCK
#include <ros2_test_framework/client_base.hpp>
#include <ros2_test_framework/service_client_mock.hpp>
#endif

#if defined(ROS2_TEST_FRAMEWORK_PUBLISHER_MOCK) && defined(ROS2_TEST_FRAMEWORK_SUBSCRIPTION_MOCK)
#include <ros2_test_framework/publisher_mock.hpp>
#include <ros2_test_framework/subscription_mock.hpp>
#elif defined(ROS2_TEST_FRAMEWORK_PUBLISHER_MOCK)
#include <ros2_test_framework/publisher_mock.hpp>
#elif defined(ROS2_TEST_FRAMEWORK_SUBSCRIPTION_MOCK)
#include <ros2_test_framework/subscription_mock.hpp>
#endif

#ifdef ROS2_TEST_FRAMEWORK_TIMER_MOCK
#include <ros2_test_framework/create_timer_mock.hpp>
#endif