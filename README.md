![Rtest Logo](rtest/doc/logo_h_grey.png)

[![Licence](https://img.shields.io/github/license/Beam-and-Spyrosoft/rtest?style=for-the-badge)](./LICENSE)

[![ROS 2 Jazzy CI](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml/badge.svg?branch=main)](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml)

[![Static Code Analysis with clang-tidy](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-clang-tidy.yml/badge.svg)](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-clang-tidy.yml)


# Rtest

This repository provides a suite of tools and utilities tailored for testing and debugging ROS 2 (Robot Operating System) applications. It aims to simplify the development and testing workflows for ROS 2-based projects, particularly in scenarios involving unit and integration testing. 

The tools in this repository address challenges posed by ROS 2's inter-process communication, which can lead to inconsistent test results. By focusing on integration testing without revalidating the underlying RMW (ROS Middleware) implementations, this repository ensures a more streamlined and reliable testing process.

This framework enables writing reliable, fully repeatable unit tests (and more) for C++ ROS 2 implementations, eliminating the issue of so-called "flaky tests".

## Contributors
This repository and tooling was initally developed as a collaboration between [BEAM](https://beam.global/) and [Spyrosoft](https://spyro-soft.com/); and is maintained as a collaboration.

## Features

- Verifying whether the tested Node has created the necessary entities such as publishers, subscribers, timers, services clients, services providers, action servers or action clients.
- Setting expectations on mocked entities, enabling verification of events such as publishing expected messages on a selected topic, sending a request by a client or validating a service response, sending action goals or verifying action feedback and results.
- Direct message passing to subscribers, or direct request injection to services and responding to clients.
- Direct timer callbacks firing and simulated time control, resulting in immediate and precise time-dependent implementations testing.
- Single-threaded, controllable test execution.

> **LIMITATION**: The `rtest` framework has a significant limitation, in order to test the implementation against ROS 2, access to the source code of those components is required. They need to be directly included in the test build process.
For example: testing interactions with components coming from external dependencies, such as those installed from system repositories, is not possible. In such cases, the source code of those packages must be downloaded and included in the workspace source tree.

## Documentation

Complete documentation: [Rtest Documentation](https://rtest.readthedocs.io/en/latest/)

## Requirements

- rclcpp
- GoogleTest
- ament_cmake_ros

## Quick-Start

1. Clone the repository:
    ```sh
    $ git clone https://github.com/yourusername/rtest.git
    ```
2. Build and run the test examples:
    ```sh
    $ colcon build --cmake-args="-DRTEST_BUILD_EXAMPLES=On"
    $ colcon test --packages-select rtest_examples --event-handlers console_cohesion+
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
  rtest::action_server_mock
  rtest::action_client_mock
)

target_link_libraries(${PROJECT_NAME}-test
  rclcpp::rclcpp
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
- `rclcpp_action::Server` (experimental)
- `rclcpp_action::Client` (experimental)

## Testing Process

Testing ROS 2 components follows a two-step approach:

### Find Component Phase

Use the appropriate find* function to locate the component created by your node:

- `findTimers` for timers
- `findPublisher` for publishers
- `findSubscription` for subscriptions
- `findService` for service providers
- `findClient` for service clients
- `experimental::findActionClient` for action clients
- `experimental::findActionServer` for action servers

### Mocking Phase

Once the interesting mocked entities are found, user can set up call expectaions or interact with them directly.


### Examples

Examples are located in the `examples` folder.


## License

This project is licensed under the APACHE 2.0 License. See the `LICENSE` file for details.
