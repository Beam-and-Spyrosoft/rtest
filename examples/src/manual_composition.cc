/**
 * @file      manual_composition.cc
 * @author    Sławomir Cielepak (sie@spyro-soft.com)
 * @date      2024-11-26
 * @copyright Copyright (c) 2024 Beam Limited.
 *
 * @brief
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

#include <rclcpp/rclcpp.hpp>
#include <test_composition/publisher.h>
#include <test_composition/subscriber.h>
#include <test_composition/service_provider.h>
#include <test_composition/service_client.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts{};

  rclcpp::executors::SingleThreadedExecutor exec{};
  auto pub = std::make_shared<test_composition::Publisher>(opts);
  exec.add_node(pub);
  auto sub = std::make_shared<test_composition::Subscriber>(opts);
  exec.add_node(sub);
  auto service_provider = std::make_shared<test_composition::ServiceProvider>(opts);
  exec.add_node(service_provider);
  auto service_client = std::make_shared<test_composition::ServiceClient>(opts);
  exec.add_node(service_client);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}