cmake_minimum_required(VERSION 3.10.2)
project(rtest
  LANGUAGES CXX
  VERSION 0.2.0
)

# Detect ROS distribution
if(NOT DEFINED ROS_DISTRO_TARGET)
  if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO_TARGET $ENV{ROS_DISTRO})
  else()
    message(FATAL_ERROR "ROS_DISTRO environment variable not set. Please source ROS setup or set ROS_DISTRO_TARGET manually.")
  endif()
endif()
message(STATUS "Building Rtest for ROS distribution: ${ROS_DISTRO_TARGET}")

# Validate supported distributions
set(SUPPORTED_DISTROS "jazzy" "kilted" "rolling")
if(NOT ROS_DISTRO_TARGET IN_LIST SUPPORTED_DISTROS)
  message(FATAL_ERROR "Unsupported ROS distribution: ${ROS_DISTRO_TARGET}. Supported: ${SUPPORTED_DISTROS}")
endif()

if(ROS_DISTRO_TARGET STREQUAL "rolling")
  message(WARNING "ROS 2 Rolling distribution detected. This is experimental support, we don't guarantee that rolling will work correctly.")
  # Falling back to kilted distro
  set(ROS_DISTRO_TARGET "kilted")
endif()

# Define distribution-specific version number
if(ROS_DISTRO_TARGET STREQUAL "jazzy")
  set(ROS_VERSION_NUMBER 24)
elseif(ROS_DISTRO_TARGET STREQUAL "kilted")
  set(ROS_VERSION_NUMBER 25)
else()
  message(FATAL_ERROR "No ROS version number defined for distribution: ${ROS_DISTRO_TARGET}")
endif()
message(STATUS "Setting RTEST_ROS_VERSION=${ROS_VERSION_NUMBER} for ${ROS_DISTRO_TARGET}")

# Set distribution-specific include path (no src needed - all header-only)
set(DISTRO_INCLUDE_DIR "include/${ROS_DISTRO_TARGET}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gmock_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
find_package(rcl_action REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(action_msgs REQUIRED)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Rtest_common mocking library
add_library(rtest_common SHARED
  src/static_registry.cpp
  src/logger_mock.cpp
)
add_library(${PROJECT_NAME}::rtest_common ALIAS rtest_common)
set_property(TARGET rtest_common PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(rtest_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

if(WIN32)
MESSAGE ("WINDOWS")
  target_compile_options(rtest_common PUBLIC
    "/FI${CMAKE_CURRENT_SOURCE_DIR}/include/rtest/includes_mock.hpp"
  )
else()
  target_compile_options(rtest_common PUBLIC
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/include/rtest/includes_mock.hpp"
  )
endif()

target_link_libraries(rtest_common PUBLIC
  rcl_action::rcl_action
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

ament_export_dependencies(
  rclcpp rclcpp_action rcl_action gmock_vendor
)

# Publisher mock library
add_library(publisher_mock INTERFACE)
add_library(${PROJECT_NAME}::publisher_mock ALIAS publisher_mock)
target_include_directories(publisher_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(publisher_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

target_compile_definitions(publisher_mock INTERFACE RTEST_PUBLISHER_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

# Subscriber mock library
add_library(subscription_mock INTERFACE)
add_library(${PROJECT_NAME}::subscription_mock ALIAS subscription_mock)
target_include_directories(subscription_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(subscription_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

target_compile_definitions(subscription_mock INTERFACE RTEST_SUBSCRIPTION_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

# Service mock library
add_library(service_mock INTERFACE)
add_library(${PROJECT_NAME}::service_mock ALIAS service_mock)
target_include_directories(service_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(service_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

target_compile_definitions(service_mock INTERFACE RTEST_SERVICE_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

add_library(service_client_mock INTERFACE)
add_library(${PROJECT_NAME}::service_client_mock ALIAS service_client_mock)
target_include_directories(service_client_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(service_client_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

target_compile_definitions(service_client_mock INTERFACE RTEST_CLIENT_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

# Action mock library
add_library(action_server_mock INTERFACE)
add_library(${PROJECT_NAME}::action_server_mock ALIAS action_server_mock)
target_include_directories(action_server_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(action_server_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

#Add explicit ament dependency for action mock
target_link_libraries(action_server_mock INTERFACE
  rcl_action::rcl_action
  rclcpp_action::rclcpp_action
)

target_compile_definitions(action_server_mock INTERFACE RTEST_ACTION_SERVER_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

add_library(action_client_mock INTERFACE)
add_library(${PROJECT_NAME}::action_client_mock ALIAS action_client_mock)
target_include_directories(action_client_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(action_client_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

#Add explicit ament dependency for action mock
target_link_libraries(action_client_mock INTERFACE
  rcl_action::rcl_action
  rclcpp_action::rclcpp_action
)

target_compile_definitions(action_client_mock INTERFACE RTEST_ACTION_CLIENT_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

# Timer mock library
add_library(timer_mock INTERFACE)
add_library(${PROJECT_NAME}::timer_mock ALIAS timer_mock)
target_include_directories(timer_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${DISTRO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(timer_mock INTERFACE
  ${PROJECT_NAME}::rtest_common
)

target_compile_definitions(timer_mock INTERFACE RTEST_TIMER_MOCK
  RTEST_ROS_VERSION=${ROS_VERSION_NUMBER}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY ${DISTRO_INCLUDE_DIR}/
  DESTINATION include
)

install(DIRECTORY cmake/
        DESTINATION share/${PROJECT_NAME}/cmake
        FILES_MATCHING PATTERN "*.cmake"
                       PATTERN "*.cmake.in"
                       )

install(
  TARGETS
    rtest_common
    subscription_mock
    publisher_mock
    service_mock
    service_client_mock
    timer_mock
    action_server_mock
    action_client_mock
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rcl_action)

if(BUILD_TESTING)
  # Option to enable clang-tidy tests (default: disabled)
  option(ENABLE_CLANG_TIDY "Enable clang-tidy tests (can be time-consuming)" OFF)
  if(ENABLE_CLANG_TIDY)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)
    set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-tidy)
    ament_lint_auto_find_test_dependencies()
    # Exclude other distribution folders from linting/coverage
    foreach(DISTRO IN LISTS SUPPORTED_DISTROS)
      if(NOT DISTRO STREQUAL ROS_DISTRO_TARGET)
        list(APPEND AMENT_LINT_AUTO_EXCLUDE "distributions/${DISTRO}")
      endif()
    endforeach()
  else()
    find_package(ament_cmake_copyright REQUIRED)
    ament_copyright()
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/test_tools_add_doubles.cmake"
)
