/**
 * @file      single_instance.h
 * @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief    Mock header for ROS 2 single instance class.
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

#include <atomic>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include <boost/type_index.hpp>

namespace ros2_test_framework {

template <typename T>
class SingleInstance {
public:
  SingleInstance() {
    if (instanceCreated_.test_and_set()) {
      throw std::runtime_error{
          std::string{"Attempt to create more than one instance of "} + boost::typeindex::type_id<T>().pretty_name()};
    }
  }

  ~SingleInstance() { instanceCreated_.clear(); }

private:
  static std::atomic_flag instanceCreated_;
};

template <typename T>
std::atomic_flag SingleInstance<T>::instanceCreated_ = ATOMIC_FLAG_INIT;

}  // namespace ros2_test_framework