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
  pub_sub_tests.cpp
  service_provider_tests.cpp
  service_client_tests.cpp
  action_client_tests.cpp
  action_server_tests.cpp
  debounce_tests.cpp
  clock_tests.cpp
)

target_include_directories(${PROJECT_NAME}-test PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}-test
  Python3::Python
  ${PROJECT_NAME}::publisher_component_test_doubles
  ${PROJECT_NAME}::subscriber_component_test_doubles
  ${PROJECT_NAME}::service_provider_component_test_doubles
  ${PROJECT_NAME}::service_client_component_test_doubles
  ${PROJECT_NAME}::action_server_component_test_doubles
  ${PROJECT_NAME}::action_client_component_test_doubles
)

ament_add_gmock_test(${PROJECT_NAME}-test)
