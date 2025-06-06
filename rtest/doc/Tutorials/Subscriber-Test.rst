Subscriber Testing
==================

This example demonstrates how to use ``rtest`` to test a simple ROS 2 subscriber component.
It walks through a component node that subscribes to a topic and shows how to inject messages
and verify received data.

Overview
========

``rtest`` allows white-box access to subscriptions, enabling isolated and deterministic tests
without requiring ROS 2 executors or spinning threads. This means all messages are delivered
immediately when the test code calls the injection API, with no latency.

The ``Subscriber`` node subscribes to the ``/test_topic`` topic using the default
``SensorDataQoS`` profile.

To test it, we use the ``rtest`` subscription framework to inject messages directly
into the subscriber without passing through the ROS middleware.

Upon receiving a message of type ``std_msgs/msg/String``, the node logs the message
content and stores it internally. We verify that the callback executed as expected
by inspecting the member variable ``lastMsg_``.

Implementation
==============

.. code-block:: cpp

   class Subscriber : public rclcpp::Node
   {
   public:
     Subscriber(const rclcpp::NodeOptions & options);
     std_msgs::msg::String getLastMsg() const { return lastMsg_; }

   private:
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
     std_msgs::msg::String lastMsg_;
   };

.. note::
   The full source is available in ``test_composition/subscriber.cpp``.

The node is registered as a component using ``RCLCPP_COMPONENTS_REGISTER_NODE``.

Testing the Subscriber
======================

The ``rtest`` library provides helper functions such as ``findSubscription`` to access
internal ROS 2 constructs like subscriptions. In this test, we validate the subscriber's
behavior when messages are injected directly.

.. code-block:: cpp

   TEST_F(PubSubTest, SubscriptionTest)
   {
     auto node = std::make_shared<test_composition::Subscriber>(opts);

     // Access the subscriber
     auto subscription = rtest::findSubscription<std_msgs::msg::String>(node, "/test_topic");
     ASSERT_TRUE(subscription);

     // Inject a message
     auto msg = std::make_shared<std_msgs::msg::String>();
     msg->set__data("someId");
     subscription->handle_message(msg);

     // Check internal state
     EXPECT_EQ(node->getLastMsg().data, "someId");

     // Send another message
     msg->set__data("otherId");
     subscription->handle_message(msg);

     // Ensure state updated
     EXPECT_EQ(node->getLastMsg().data, "otherId");
   }

Key Concepts
============

- ``rtest::findSubscription`` locates a ``SubscriptionBase`` instance for testing.
- ``handle_message`` is used to simulate message reception without a running ROS 2 system.
- Tests use standard GoogleTest (``gtest``) macros but can also work with other frameworks such as Catch2.

Try It Yourself
