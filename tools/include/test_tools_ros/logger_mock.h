/**
 * @file      logger_mock.h
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-18
 * @copyright Copyright (c) 2024 Beam Limited.
 *  
 * @brief    Mock header for ROS 2 logging.
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

#include "test_tools_ros/single_instance.h"
#include <rcutils/logging.h>
#include <gmock/gmock.h>

namespace test_tools_ros {

// Fwd declaration
void log_handler(
    const rcutils_log_location_t *location,
    int severity,
    const char *name,
    rcutils_time_point_value_t timestamp,
    const char *format,
    va_list *args);

/**
 * @brief A mock for ROS 2 logging interface
 */
class LoggerMock : public test_tools_ros::SingleInstance<LoggerMock> {
public:
  LoggerMock() {
    instance_ = this;
    orig_log_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(log_handler);
  }

  ~LoggerMock() {
    instance_ = nullptr;
    rcutils_logging_set_output_handler(orig_log_handler_);
  }

  static LoggerMock *instance_;

  MOCK_METHOD(void, log, (RCUTILS_LOG_SEVERITY, const std::string));

private:
  rcutils_logging_output_handler_t orig_log_handler_{nullptr};
};

/**
 * @brief Disable the logs in the test.
 *
 */
class DisableLogs : public test_tools_ros::SingleInstance<DisableLogs> {
public:
  DisableLogs() {
    orig_log_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(nullptr);
  }

  ~DisableLogs() { rcutils_logging_set_output_handler(orig_log_handler_); }

private:
  rcutils_logging_output_handler_t orig_log_handler_{nullptr};
};

}  // namespace test_tools_ros
