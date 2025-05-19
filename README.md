[![Static Code Analysis with clang-tidy](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-clang-tidy.yml/badge.svg)](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-clang-tidy.yml)

# RTEST

This repository provides a suite of tools and utilities tailored for testing and debugging ROS 2 (Robot Operating System) applications. It aims to simplify the development and testing workflows for ROS 2-based projects, particularly in scenarios involving unit and integration testing. 

The tools in this repository address challenges posed by ROS 2's inter-process communication, which can lead to inconsistent test results. By focusing on integration testing without revalidating the underlying RMW (ROS Middleware) implementations, this repository ensures a more streamlined and reliable testing process.

## Contributors
This repository and tooling was initally developed as a collaboration between [BEAM](https://beam.global/) and [Spyrosoft](https://spyro-soft.com/); and is maintained as a collaboration.

## Features

- Example of the testing framework being used

## Requirements

- rclcpp
- GoogleTest
- ament_cmake_ros

## Usage

1. Clone the repository:
    ```
    git clone https://github.com/yourusername/rtest.git
    ```
2. To build and run the test examples:
    ```
    colcon build && colcon test --packages-select rtest_examples --event-handlers console_cohesion+
    ```

## Adding Testing Support to Your Package

### 1. Add Dependencies

Add a dependency to `rtest` in your `package.xml` file:

```xml
<package format="3">
  ...
  <test_depend>rtest</test_depend>
</package>
```

### 2. Create Test Directory

Create a sub-folder `test` and add a `CMakeLists.txt` file there.

> **WARNING**: The Rtest uses C++ template code substitution at the source level. You must build the unit under test directly from sources. Linking with a static or dynamic library will not work.

Example `CMakeLists.txt`:

```cmake
find_package(ament_cmake_gmock REQUIRED)
find_package(rtest REQUIRED)

ament_add_gmock(${PROJECT_NAME}-test
  main.cpp
  unit_tests.cpp
  ${CMAKE_SOURCE_DIR}/src/ros2_node.cpp
)

target_link_libraries(${PROJECT_NAME}-test
  rtest::publisher_mock
  rtest::subscription_mock
  rtest::timer_mock
  rtest::service_mock
  rtest::service_client_mock
)

ament_target_dependencies(${PROJECT_NAME}-test
  rclcpp
)
```

### 3. Create Test Main File

The testing library uses Google Mocking framework and requires initialization:

```cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  // Run all tests
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
```

## Mockable ROS 2 Components

The framework allows mocking of the following ROS 2 components:

- `rclcpp::TimerBase`
- `rclcpp::Publisher`
- `rclcpp::Subscription`
- `rclcpp::Service`
- `rclcpp::Client`

## Testing Process

Testing ROS 2 components follows a two-step approach:

### Find Component Phase

Use the appropriate find* function to locate the component created by your node:

- `findTimers` for timers
- `findPublisher` for publishers
- `findSubscription` for subscriptions
- `findService` for service providers
- `findClient` for service clients

### Mocking Phase

Examples can be found in the `examples` folder.


## License

This project is licensed under the APACHE 2.0 License. See the `LICENSE` file for details.
