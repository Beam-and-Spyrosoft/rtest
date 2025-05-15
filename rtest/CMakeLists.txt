cmake_minimum_required(VERSION 3.10.2)
project(rtest
  LANGUAGES CXX
  VERSION 0.1.0
)

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



set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


# Common mocking library
add_library(common SHARED
  src/static_registry.cpp
  src/logger_mock.cpp
)
add_library(${PROJECT_NAME}::common ALIAS common)
set_property(TARGET common PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(WIN32)
MESSAGE ("WINDOWS")
  target_compile_options(common PUBLIC
    "/FI${CMAKE_CURRENT_SOURCE_DIR}/include/rtest/includes_mock.hpp"
  )
else()
  target_compile_options(common PUBLIC
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/include/rtest/includes_mock.hpp"
  )
endif()


ament_target_dependencies(common
  PUBLIC rclcpp gmock_vendor
)

ament_export_dependencies(
  rclcpp gmock_vendor
)


# Publisher mock library
add_library(publisher_mock INTERFACE)
add_library(${PROJECT_NAME}::publisher_mock ALIAS publisher_mock)
target_include_directories(publisher_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(publisher_mock INTERFACE
  ${PROJECT_NAME}::common
)

target_compile_definitions(publisher_mock INTERFACE RTEST_PUBLISHER_MOCK)


# Subscriber mock library
add_library(subscription_mock INTERFACE)
add_library(${PROJECT_NAME}::subscription_mock ALIAS subscription_mock)
target_include_directories(subscription_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(subscription_mock INTERFACE
  ${PROJECT_NAME}::common
)

target_compile_definitions(subscription_mock INTERFACE RTEST_SUBSCRIPTION_MOCK)

# Service mock library
add_library(service_mock INTERFACE)
add_library(${PROJECT_NAME}::service_mock ALIAS service_mock)
target_include_directories(service_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(service_mock INTERFACE
  ${PROJECT_NAME}::common
)

target_compile_definitions(service_mock INTERFACE RTEST_SERVICE_MOCK)

add_library(service_client_mock INTERFACE)
add_library(${PROJECT_NAME}::service_client_mock ALIAS service_client_mock)
target_include_directories(service_client_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(service_client_mock INTERFACE
  ${PROJECT_NAME}::common
)

target_compile_definitions(service_client_mock INTERFACE RTEST_CLIENT_MOCK)

# Timer mock library
add_library(timer_mock INTERFACE)
add_library(${PROJECT_NAME}::timer_mock ALIAS timer_mock)
target_include_directories(timer_mock
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(timer_mock INTERFACE
  ${PROJECT_NAME}::common
)

target_compile_definitions(timer_mock INTERFACE RTEST_TIMER_MOCK)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY cmake/
        DESTINATION share/${PROJECT_NAME}/cmake
        FILES_MATCHING PATTERN "*.cmake"
                       PATTERN "*.cmake.in"
                       )

install(
  TARGETS
    common
    subscription_mock
    publisher_mock
    service_mock
    service_client_mock
    timer_mock
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/test_tools_add_doubles.cmake"
)
