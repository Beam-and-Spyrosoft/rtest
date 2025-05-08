/**
 * @file      pub_sub_tests.cc
 * @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
 * @date      2024-12-4
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief   Unit tests for the Publisher and Subscriber classes.
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

#include <gtest/gtest.h>

#include <test_composition/publisher.h>
#include <test_composition/subscriber.h>

class PubSubTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(PubSubTest, PublisherTest) {
  auto node = std::make_shared<test_composition::Publisher>(opts);

  // Retrieve the publisher created by the Node
  auto publisher = ros2_test_framework::findPublisher<std_msgs::msg::String>(node, "/test_topic");

  // Check that the Node actually created the Publisher with topic: "/test_topic"
  ASSERT_TRUE(publisher);

  // Retrieve the timers created by the Node
  auto nodeTimers = ros2_test_framework::findTimers(node);

  // There should be just one timer
  ASSERT_EQ(nodeTimers.size(), 1UL);

  // Set up expectation that the Node will publish a FalconState message when the timer callback is fired
  auto expectedMsg = std_msgs::msg::String{};
  expectedMsg.set__data("timer");
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);

  // Fire the timer callback
  nodeTimers[0]->execute_callback(nullptr);
}

TEST_F(PubSubTest, SubscriptionTest) {
  auto node = std::make_shared<test_composition::Subscriber>(opts);

  // Retrieve the subscription created by the Node
  auto subscription = ros2_test_framework::findSubscription<std_msgs::msg::String>(node, "/test_topic");

  // Check that the Node actually created the Subscription with topic: "/test_topic"
  ASSERT_TRUE(subscription);

  // Inject a message to the subscription
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->set__data("someId");
  subscription->handle_message(msg);
  auto lastMsg = *msg;

  // Test the side effects
  EXPECT_EQ(node->getLastMsg(), lastMsg);

  // Send another msg
  msg->set__data("otherId");
  subscription->handle_message(msg);

  // Test the side effects
  EXPECT_NE(node->getLastMsg(), lastMsg);
}

TEST_F(PubSubTest, PubSequenceTest) {
  auto node = std::make_shared<test_composition::Publisher>(opts);
  auto publisher = ros2_test_framework::findPublisher<std_msgs::msg::String>(node, "/test_topic");

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

TEST_F(PubSubTest, IntraProcessCommTest) {
  // Test if the framework also work if Intra-Process Communication is on
  opts.use_intra_process_comms(true);

  auto subNode = std::make_shared<test_composition::Subscriber>(opts);
  auto pubNode = std::make_shared<test_composition::Publisher>(opts);

  auto subscription = ros2_test_framework::findSubscription<std_msgs::msg::String>(subNode, "/test_topic");
  auto publisher = ros2_test_framework::findPublisher<std_msgs::msg::String>(pubNode, "/test_topic");

  ASSERT_TRUE(subscription);
  ASSERT_TRUE(publisher);

  // Test the subscription
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->set__data("test_msg");

    EXPECT_NE(subNode->getLastMsg(), *msg);

    subscription->handle_message(msg);
    EXPECT_EQ(subNode->getLastMsg(), *msg);
  }

  // Test the publisher
  {
    auto expectedMsg = std_msgs::msg::String{};
    expectedMsg.set__data("copy");

    EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
    pubNode->publishCopy();
  }
}