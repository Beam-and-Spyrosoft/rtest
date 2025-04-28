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

#if defined(TEST_TOOLS_ROS_PUBLISHER_MOCK) && defined(TEST_TOOLS_ROS_SUBSCRIPTION_MOCK)
#include <test_tools_ros/publisher_mock.h>
#include <test_tools_ros/subscription_mock.h>
#elif defined(TEST_TOOLS_ROS_PUBLISHER_MOCK)
#include <test_tools_ros/publisher_mock.h>
#elif defined(TEST_TOOLS_ROS_SUBSCRIPTION_MOCK)
#include <test_tools_ros/subscription_mock.h>
#endif

#ifdef TEST_TOOLS_ROS_TIMER_MOCK
#include <test_tools_ros/create_timer_mock.h>
#endif