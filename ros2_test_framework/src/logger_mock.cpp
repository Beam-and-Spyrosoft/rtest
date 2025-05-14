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
// @file      logger_mock.cpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-18
//
// @brief     Mock implementation for ROS 2 logging.

#include <ros2_test_framework/logger_mock.hpp>

namespace ros2_test_framework
{

LoggerMock * LoggerMock::instance_{nullptr};

void log_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{
  (void)location;
  (void)name;
  (void)timestamp;
  if (LoggerMock::instance_ != nullptr) {
    std::string msg;
    va_list args_copy;
    va_copy(args_copy, *args);
    size_t len = vsnprintf(nullptr, 0, format, args_copy);
    va_end(args_copy);
    if (len == 0UL) {
      return;
    }
    msg.resize(len + 1);
    vsnprintf(&msg[0], len + 1, format, *args);
    msg.resize(len);  // remove NUL
    LoggerMock::instance_->log(static_cast<RCUTILS_LOG_SEVERITY>(severity), msg);
  }
}

}  // namespace ros2_test_framework
