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
// @file      pub_sub_tests.cpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-12-04
//
// @brief     Unit tests for the Publisher and Subscriber classes.

#include <gtest/gtest.h>

#include <test_composition/publisher.hpp>
#include <test_composition/subscriber.hpp>

class PubSubTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(PubSubTest, PublisherTest)
{
  auto node = std::make_shared<test_composition::Publisher>(opts);

  /// Retrieve the publisher created by the Node
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  // Check that the Node actually created the Publisher with topic: "/test_topic"
  ASSERT_TRUE(publisher);

  /// Retrieve the timers created by the Node
  auto nodeTimers = rtest::findTimers(node);

  // There should be just one timer
  ASSERT_EQ(nodeTimers.size(), 1UL);

  /// Set up expectation that the Node will publish a message when the timer callback is fired
  auto expectedMsg = std_msgs::msg::String{};
  expectedMsg.set__data("timer");
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);

  // Fire the timer callback
  nodeTimers[0]->execute_callback(nullptr);
}

TEST_F(PubSubTest, PublishIfSubscriuptionCountNonZeroTest)
{
  auto node = std::make_shared<test_composition::Publisher>(opts);
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  /// Set up expectation that the Node will not publish a message when the subscription count is 0
  EXPECT_CALL(*publisher, publish(::testing::_)).Times(0);
  node->publishIfSubscribersListening();

  /// Set subscription count to 1
  publisher->setSubscriptionCount(1UL);

  auto expectedMsg = std_msgs::msg::String{};
  expectedMsg.set__data("if_subscribers_listening");

  /// Set up expectation that the Node will publish a message when the subscription count is 1
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
  node->publishIfSubscribersListening();
}

TEST_F(PubSubTest, SubscriptionTest)
{
  auto node = std::make_shared<test_composition::Subscriber>(opts);

  /// Retrieve the subscription created by the Node
  auto subscription = rtest::findSubscription<std_msgs::msg::String>(node, "/test_topic");

  // Check that the Node actually created the Subscription with topic: "/test_topic"
  ASSERT_TRUE(subscription);

  /// Inject a message to the subscription
  subscription->handle_message("someId");

  // Test the side effects
  EXPECT_EQ(node->getLastMsg().data, "someId");

  // Send another msg
  subscription->handle_message("otherId");

  // Test the side effects
  EXPECT_EQ(node->getLastMsg().data, "otherId");
}

TEST_F(PubSubTest, PubSequenceTest)
{
  auto node = std::make_shared<test_composition::Publisher>(opts);
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  auto expectedMsg1 = std_msgs::msg::String{};
  expectedMsg1.set__data("copy");
  auto expectedMsg2 = std_msgs::msg::String{};
  expectedMsg2.set__data("unique_ptr");
  auto expectedMsg3 = std_msgs::msg::String{};
  expectedMsg3.set__data("loaned_msg");

  {
    ::testing::Sequence seq{};
    EXPECT_CALL(*publisher, publish(expectedMsg1)).InSequence(seq);
    EXPECT_CALL(*publisher, publish(expectedMsg2)).InSequence(seq);
    EXPECT_CALL(*publisher, publish(expectedMsg3)).InSequence(seq);

    // Any other sequence of publishing will fail
    node->publishCopy();
    node->publishUniquePtr();
    node->publishLoanedMsg();
  }
}

TEST_F(PubSubTest, IntraProcessCommTest)
{
  // Test if the framework also work if Intra-Process Communication is on
  opts.use_intra_process_comms(true);

  auto subNode = std::make_shared<test_composition::Subscriber>(opts);
  auto pubNode = std::make_shared<test_composition::Publisher>(opts);

  auto subscription = rtest::findSubscription<std_msgs::msg::String>(subNode, "/test_topic");
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(pubNode, "/test_topic");

  ASSERT_TRUE(subscription);
  ASSERT_TRUE(publisher);

  // Test the subscription
  {
    const std::string test_msg{"test_msg"};

    EXPECT_NE(subNode->getLastMsg().data, test_msg);

    subscription->handle_message(test_msg);
    EXPECT_EQ(subNode->getLastMsg().data, test_msg);
  }

  // Test the publisher
  {
    auto expectedMsg = std_msgs::msg::String{};
    expectedMsg.set__data("copy");

    EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
    pubNode->publishCopy();
  }
}
