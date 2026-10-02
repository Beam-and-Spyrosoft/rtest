find_package(ament_cmake_gmock REQUIRED)

#
# Use add_executable + GTest::gmock instead of `ament_add_gmock`.
#
# `ament_add_gmock` compiles gmock_vendor sources into a local libgmock.a whose API
# (MakeAndRegisterTestInfo(char const*,...)) is older than the installed headers
# (MakeAndRegisterTestInfo(std::string,...) from libgtest.so.1.17.0). Linking both
# versions creates duplicate GTest singletons; LLD does not preempt them the way
# GNU ld does, causing SIGFPE during static initialization.
add_executable(${PROJECT_NAME}-test
  main.cpp
  static_registry_stale_entry_test.cpp
)

target_link_libraries(${PROJECT_NAME}-test
  rtest::publisher_mock
  rtest::subscription_mock
  rtest::service_mock
  rtest::service_client_mock
  rtest::action_server_mock
  rtest::action_client_mock
  rtest::timer_mock
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${example_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  # GTest::gmock / GTest::gtest come in transitively through rtest_common. Do not
  # list them here: with static gtest (apt libgmock-dev) they would precede
  # librtest_common.so on the link line and give the executable its own copy of
  # gmock globals, which are then destroyed twice at exit (pthread_key_delete
  # failed with error 22).
)

ament_add_gmock_test(${PROJECT_NAME}-test)
