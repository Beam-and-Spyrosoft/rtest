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
// @file      static_registry.cpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-26
//
// @brief    Mock implementation for ROS 2 static registry.

#include <rtest/static_registry.hpp>

namespace rtest
{

StaticMocksRegistry StaticMocksRegistry::theRegistry_;

void enableVerboseLogs(bool on) { StaticMocksRegistry::instance().enableVerboseLogs(on); }

}  // namespace rtest
