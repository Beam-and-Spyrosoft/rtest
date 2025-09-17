# Writing a Publisher test

```{contents} Contents
---
depth: 2
local: true
---
```

## Background
In this tutorial we will write a test suite that verifies a ROS 2 Node implementation that publishes messages.

The `rtest` framework provides white-box access to publishers via the `findPublisher` API. Combined with GoogleMock (`EXPECT_CALL`), this allows tests to assert exactly which messages are published, without spinning executors or relying on the ROS 2 middleware.

In this example, we will:
- Demonstrate a publisher node that publishes to '/test_topic' on a timer.
- Use the 'rtest' publisher interface to verify that publishing occurs as expected.
- Inspect the timer and simulate the timer callback to deterministically test behavior.

## Prerequisites

[Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#)

[Writing Basic Tests with C++ with GTest](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Cpp.html)

## Tasks

### 1 Create a package
Navigate to your ROS 2 workspace sources, e.g. `ros2_ws/src`, and run the package creation command:

```shell
$ ros2 pkg create example_app --dependencies rclcpp std_msgs
```

Navigate to `example_app`.

### 2 Write the Publisher node

Add the `Publisher` class definition in `include/example_app/publisher.hpp` with the following code:

```c++
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

And add a class implementation in `src/publisher.cpp`:

```c++
#include "example_app/publisher.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

Publisher::Publisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_publisher", options)
{
  publisher_ = create_publisher<std_msgs::msg::String>("test_topic", rclcpp::QoS{5UL});
  timer_ = create_wall_timer(500ms, [this]() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->set__data("timer");
    publisher_->publish(std::move(msg));
  });
}
```

Open the `CMakeLists.txt` and add the Publisher as a library:
- **Note:** `ament_target_dependencies` is [depricated](https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html#ament-target-dependencies-is-deprecated) so we use plain CMake `target_link_libraries`

```cmake
cmake_minimum_required(VERSION 3.8)
project(example_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add Publisher
add_library(publisher src/publisher.cpp)

target_include_directories(publisher PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(publisher
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
```

### 3 Examine the code
```c++
publisher_ = create_publisher<std_msgs::msg::String>("test_topic", rclcpp::QoS{5UL});
```
The Node creates a publisher with the msg type `std_msgs::msg::String` and topic name `test_topic`.

```c++
timer_ = create_wall_timer(500ms, [this]() {
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->set__data("timer");
  publisher_->publish(std::move(msg));
});
```
The timer callback is a lambda that publishes a message when triggered


### 4 Add unit tests

#### 4.1 Add dependency to `rtest`


Open the `package.xml` and add the `rtest` test dependency:

```xml
<package format="3">
  ...
  <test_depend>rtest</test_depend>
</package>
```

**NOTE** Currently `rtest` supports writing tests with GTest/GMock only. There's no need to add that dependencies explicitly.

#### 4.2 Implement a simple unit test

Create the `test` directory and add a C++ tests implementation file `test/publisher_test.cpp`

```c++
#include <gtest/gtest.h>
#include "example_app/publisher.hpp"

class PubSubTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(PubSubTest, PublisherTest)
{
  auto node = std::make_shared<Publisher>(opts);

  /// Retrieve the publisher created by the Node
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  /// Check that the Node actually created the Publisher with topic: "/test_topic"
  ASSERT_TRUE(publisher);

  /// Retrieve the timers created by the Node
  auto nodeTimers = rtest::findTimers(node);

  /// There should be just one timer
  ASSERT_EQ(nodeTimers.size(), 1UL);

  /// Set up expectation that the Node will publish a message when the timer callback is fired
  auto expectedMsg = std_msgs::msg::String{};
  expectedMsg.set__data("timer");
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);

  /// Fire the timer callback
  nodeTimers[0]->execute_callback(nullptr);
}
```

Or with triggering callback by time advancing:
```c++
TEST_F(PubSubTest, WhenTheTimeIsMovedByTimerPeriodCallbackShouldBeExecuted)
{
  // set use sim timer for mocked timers
  opts = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<test_composition::Publisher>(opts);
  auto triggering_test_clock = rtest::TriggeringTestClock{node};

  /// Retrieve the publisher created by the Node
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  // Check that the Node actually created the Publisher with topic: "/test_topic"
  ASSERT_TRUE(publisher);
  /// Set up expectation that the Node will publish a message when the timer callback is fired
  auto expectedMsg = std_msgs::msg::String{};
  expectedMsg.set__data("timer");

  // We do not expect the timer to trigger shortly before it reaches 500ms
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(0);
  triggering_test_clock.advance(std::chrono::milliseconds(499));

  // We expect the timer to trigger every 500ms
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
  triggering_test_clock.advance(std::chrono::milliseconds(1));

  // We do not expect the timer to trigger after one period expires but before the next begins
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(0);
  triggering_test_clock.advance(std::chrono::milliseconds(499));

  // We expect the timer to trigger every 500ms, so when the expiry time passes, the callback should fire
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
  triggering_test_clock.advance(std::chrono::milliseconds(50));
}
```

Create the main test runner in `test/main.cpp`:


```c++
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  // Initialize Google Test and ROS2
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  // Run all the tests
  int result = RUN_ALL_TESTS();

  // Shutdown ROS2
  rclcpp::shutdown();
  return result;
}
```

#### 4.3 Add tests to CMakeLists.txt

Create the `test/CMakeLists.txt` file:


```cmake
find_package(rtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}-test
    main.cpp
    ${CMAKE_SOURCE_DIR}/src/publisher.cpp
    publisher_test.cpp
)

target_include_directories(${PROJECT_NAME}-test PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}-test
  rtest::publisher_mock
  rtest::timer_mock
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
```

Update the root `CMakeLists.txt` with:

```cmake
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package() # Must be the last statement
```

### 4.4 Build and run the tests

Build the `example_app` package:

```shell
$ colcon build --packages-up-to example_app --cmake-args -DBUILD_TESTING=On
```

Run the tests:

```shell
$ colcon test --packages-select example_app --event-handlers console_cohesion+
```

## Key Concepts
- `rtest::findPublisher` locates a Publisher instance for testing.
- `rtest::findTimers` locates timers, and `execute_callback` triggers them deterministically. The callbacks can be triggered by the advancing the time as well.
- GoogleMock’s `EXPECT_CALL(...).Times(...)` allows verifying publish behavior precisely without a running ROS 2 system.
- **Note:** Other test frameworks (e.g., Catch2) are not currently supported.

Try It Yourself!
