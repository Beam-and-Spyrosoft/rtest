# Installation


RTEST is distributed as an open source project and can be installed from source or downloaded as a pre-built package. The installation process is straightforward and can be done in a few simple steps.
To get started with RTEST, you can follow the instructions below to install it in your ROS 2 environment.
The installation process is designed to be simple and quick, allowing you to start writing and running tests in no time.


## Binary packages

Binaries are only created for the Tier 1 operating systems listed in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).

We provide ROS 2 binary packages for the following platforms:

- [Ubuntu 24.04 (Noble Numbat)](https://github.com/Beam-and-Spyrosoft/rtest/releases/)


## Install From Source


RTEST can be installed from source. We support building RTEST from source on the Tier 1 operating systems. The following steps outline how to install RTEST from source in your ROS 2 environment.

1. **Clone the RTEST repository**:

Open a terminal and navigate to your ROS 2 workspace (e.g., `~/ros2_ws/src`). Then, clone the RTEST repository using the following command:

```shell
$ git clone https://github.com/yourusername/rtest.git
```

2. **Build and run the test examples**:

```shell
$ colcon build && colcon test --packages-select rtest_examples --event-handlers console_cohesion+
```
