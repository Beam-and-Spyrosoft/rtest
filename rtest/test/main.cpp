// Copyright 2026 Spyrosoft Limited.
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
// @file      main.cpp
// @date      2026-09-07

#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  int result = EXIT_FAILURE;
  try {
    testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);

    result = RUN_ALL_TESTS();
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "Unknown exception\n";
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
