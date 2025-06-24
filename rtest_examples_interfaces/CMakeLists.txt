cmake_minimum_required(VERSION 3.10.2)
project(rtest_examples_interfaces)

option(RTEST_BUILD_EXAMPLES "Build the rtest_examples" OFF)

if(RTEST_BUILD_EXAMPLES)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "action/MoveRobot.action"
    DEPENDENCIES action_msgs std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
endif()
