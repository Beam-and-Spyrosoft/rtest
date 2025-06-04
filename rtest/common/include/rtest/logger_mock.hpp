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
// @file      logger_mock.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-18
//
// @brief     Mock header for ROS 2 logging.

#pragma once

#include "rtest/single_instance.hpp"
#include <rcutils/logging.h>
#include <gmock/gmock.h>

namespace rtest
{

/// Fwd declaration
void logHandler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args);

/**
 * @brief A mock for ROS 2 logging interface
 */
class LoggerMock : public rtest::SingleInstance<LoggerMock>
{
public:
  LoggerMock()
  {
    instance_ = this;
    orig_log_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(logHandler);
  }

  ~LoggerMock()
  {
    instance_ = nullptr;
    rcutils_logging_set_output_handler(orig_log_handler_);
  }

  static LoggerMock * instance_;

  MOCK_METHOD(void, log, (RCUTILS_LOG_SEVERITY, const std::string));

private:
  rcutils_logging_output_handler_t orig_log_handler_{nullptr};
};

/**
 * @brief Disable the logs in the test.
 *
 */
class DisableLogs : public rtest::SingleInstance<DisableLogs>
{
public:
  DisableLogs() : orig_log_handler_{rcutils_logging_get_output_handler()}
  {
    rcutils_logging_set_output_handler(nullptr);
  }

  DisableLogs(const DisableLogs &) = delete;
  DisableLogs(DisableLogs &&) = delete;
  DisableLogs & operator=(const DisableLogs &) = delete;
  DisableLogs & operator=(DisableLogs &&) = delete;

  ~DisableLogs() { rcutils_logging_set_output_handler(orig_log_handler_); }

private:
  rcutils_logging_output_handler_t orig_log_handler_{nullptr};
};

}  // namespace rtest
