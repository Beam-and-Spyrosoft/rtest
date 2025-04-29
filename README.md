# Test Tools ROS

This repository provides a suite of tools and utilities tailored for testing and debugging ROS2 (Robot Operating System) applications. It aims to simplify the development and testing workflows for ROS2-based projects, particularly in scenarios involving integration testing. 

The tools in this repository address challenges posed by ROS2's inter-process communication, which can lead to inconsistent test results. By focusing on integration testing without revalidating the underlying RMW (ROS Middleware) implementations, this repository ensures a more streamlined and reliable testing process.

## Contributors
This repository and tooling was initally developed as a collaboration between [BEAM](https://beam.global/) and [Spyrosoft](https://spyro-soft.com/); and is maintained as a collaboration.

## Features

- Example of the testing framework being used

## Requirements

- ROS2

## Usage

1. Clone the repository:
    ```
    git clone https://github.com/yourusername/test_tools_ros.git
    ```
2. To build and run the test examples:
    ```
    colcon build && colcon test --packages-select test_tools_ros_examples --event-handlers console_cohesion+
    ```

## License

This project is licensed under the APACHE 2.0 License. See the `LICENSE` file for details.
