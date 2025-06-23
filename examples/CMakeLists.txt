cmake_minimum_required(VERSION 3.10.2)
project(rtest_examples 
  LANGUAGES CXX
  VERSION 0.2.0
)

option(RTEST_BUILD_EXAMPLES "Build the rtest_examples" OFF)

if(RTEST_BUILD_EXAMPLES)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(rtest_examples_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
  find_package(Threads REQUIRED)

  # Publisher component
  add_library(publisher_component SHARED src/publisher.cpp)
  target_compile_definitions(publisher_component PRIVATE "COMPOSITION_BUILDING_DLL")

  target_include_directories(publisher_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(publisher_component PUBLIC
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(publisher_component "test_composition::Publisher")

  # Subscriber component
  add_library(subscriber_component SHARED src/subscriber.cpp)
  target_compile_definitions(subscriber_component PRIVATE "COMPOSITION_BUILDING_DLL")
  target_include_directories(subscriber_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(subscriber_component PUBLIC
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(subscriber_component "test_composition::Subscriber")

  # Service provider component
  add_library(service_provider_component SHARED src/service_provider.cpp)
  target_compile_definitions(service_provider_component PRIVATE "COMPOSITION_BUILDING_DLL")
  target_include_directories(service_provider_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(service_provider_component PUBLIC
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(service_provider_component "test_composition::ServiceProvider")

  # Service client component
  add_library(service_client_component SHARED src/service_client.cpp)
  target_compile_definitions(service_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
  target_include_directories(service_client_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(service_client_component PUBLIC
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(service_client_component "test_composition::ServiceClient")


  # Action server component
  add_library(action_server_component SHARED src/action_server.cpp)
  target_compile_definitions(action_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
  target_include_directories(action_server_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Link with generated action interfaces and threads
  target_link_libraries(action_server_component PUBLIC
    ${rtest_examples_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(action_server_component "test_composition::ActionServer")

  # Action client component
  add_library(action_client_component SHARED src/action_client.cpp)
  target_compile_definitions(action_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
  target_include_directories(action_client_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Link with generated action interfaces and threads
  target_link_libraries(action_client_component PUBLIC
    ${rtest_examples_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
  )

  rclcpp_components_register_nodes(action_client_component "test_composition::ActionClient")

  # Manual/static Node composition
  add_executable(manual_composition src/manual_composition.cpp)
  target_link_libraries(manual_composition
    publisher_component
    subscriber_component
    service_provider_component
    service_client_component
    action_server_component
    action_client_component
    ${cpp_typesupport_target}
  )
  target_link_libraries(manual_composition rclcpp::rclcpp)

  install(TARGETS
    publisher_component
    subscriber_component
    service_provider_component
    service_client_component
    action_server_component
    action_client_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS
    manual_composition
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(rtest REQUIRED)
  # Option to enable clang-tidy tests (default: disabled)
  option(ENABLE_CLANG_TIDY "Enable clang-tidy tests (can be time-consuming)" OFF)
  if(ENABLE_CLANG_TIDY)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)
    set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-tidy)
    ament_lint_auto_find_test_dependencies()
  else()
    find_package(ament_cmake_copyright REQUIRED)
    ament_copyright()

    test_tools_add_doubles(publisher_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        ${std_msgs_TARGETS}
    )
    test_tools_add_doubles(subscriber_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        ${std_msgs_TARGETS}
    )
    test_tools_add_doubles(service_provider_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
    )
    test_tools_add_doubles(service_client_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
    )
    test_tools_add_doubles(action_server_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_action::rclcpp_action
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${rtest_examples_interfaces_TARGETS}
    )
    test_tools_add_doubles(action_client_component
      AMENT_DEPENDENCIES
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_action::rclcpp_action
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${rtest_examples_interfaces_TARGETS}
    )
    add_subdirectory(test)
  endif()

  ament_package()
endif()
