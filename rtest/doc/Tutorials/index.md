# Tutorials

These step-by-step tutorials will help you get started with RTEST. They cover various testing scenarios and are designed to be followed in sequence.

## Tutorial Sequence

```{toctree}
---
maxdepth: 2
titlesonly: true
---
Subscriber-Test
Publisher-Test
Timer-Test
Clock-Test
Service-Test
```

## Quick Start Example

Here's a simple example of testing a ROS 2 publisher node:

```cpp
#include "rtest/rtest.hpp"
#include "my_package/publisher_node.hpp"

TEST(PublisherNodeTest, PublishesCorrectMessage) {
  // Create test environment
  auto test_env = rtest::TestEnvironment::create();
  
  // Create node under test
  auto node = std::make_shared<my_package::PublisherNode>();
  
  // Verify publisher was created
  auto publisher = test_env->get_publisher_handle("/test_topic");
  ASSERT_NE(publisher, nullptr);
  
  // Trigger publication and verify message
  node->publish_message();
  auto message = publisher->get_last_published_message();
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(message->data, "expected data");
}
```

## Examples Repository

All example code can be found in the [C++ minimal examples](https://github.com/Beam-and-Spyrosoft/rtest/tree/main/examples/test) directory.
