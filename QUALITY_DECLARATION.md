This document is a declaration of software quality for the `rtest` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rtest` Quality Declaration

The package `rtest` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#quality-practices) of the ROS 2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]
`rtest` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#versioning)

### Version Stability [1.ii]

`rtest` is not at a stable version yet, i.e. `< 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.md).

### Public API Declaration [1.iii]
All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

Additionally, there are CMake script files which are installed in `share` and therefore part of the public API.

### API Stability Policy [1.iv]

**[TODO]** - Declare API stability, once it's established.

~~`rtest` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.~~

### ABI Stability Policy [1.v]

**[TODO]** - Declare ABI stability, once it's established.

~~`rtest` This package maintains ABI stability for its public C++ libraries within a stable release series. This means that binaries compiled against the major version of this package should continue to work with different minor and/or patch versions without recompilation, provided the same C++ compiler and standard library versions are used as those with which the library was built.~~

### API and ABI Stability Within a Released ROS Distribution [1.vi]

**[TODO]** - Declare API and ABI Stability Within a Released ROS Distribution, once it's established.

~~`rtest` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.~~

## Change Control Process [2]

`rtest` follows the recommended Change control process guidelines defined in the [ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]
All changes will occur through a pull request.

### Contributor Origin [2.ii]
This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](./CONTRIBUTING.md)

### Peer Review Policy [2.iii]
All pull request will be peer-reviewed, check [ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull request must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rtest` has a documented feature list and it is hosted [RTEST Documentation](https://rtest.readthedocs.io/en/latest/).

### Public API Documentation [3.ii]

`rtest` has documentation of its public API and it is hosted [C++ API](https://rtest.readthedocs.io/en/latest/cpp_api_docs.html).

### License [3.iii]

The license for `rtest` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](./LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml)
can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rtest`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen
[here](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml).

### Package Quality Status [3.v]

This package's claimed quality level (Level 4) is declared in its `package.xml` file, making it visible on standard ROS package indexing platforms like [ROS Index](https://index.ros.org/).

In addition to the public declaration via `package.xml`:
* This package underwent an internal peer review process by the Beam-and-Spyrosoft development team, focusing on code quality, testing, and documentation against REP-2004 Level 4 criteria.
* This package was reviewed as part of its inclusion in the Beam's Autonomous Underwater Vehicle software stack, which assesses packages for stability and maintainability.

## Testing [4]

### Feature Testing [4.i]

[TODO] Implement unit tests for each framework feature and add coverage measurment.

~~Each feature in `rtest` has corresponding tests which simulate typical usage, and they are located in the [`test`](https://github.com/Beam-and-Spyrosoft/rtest/tree/main/test) directory.~~
~~New features are required to have tests before being added.~~
~~Currently nightly test results can be seen here:~~
~~* [linux-aarch64_release](<link_to_tests_results>)~~
~~* [linux-arm64_release](<link_to_tests_results>)~~
~~* [windows_release](<link_to_tests_results>)~~

### Public API Testing [4.ii]

~~Each part of the public API has tests, and new additions or changes to the public API require tests before being added.~~
~~The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.~~

### Coverage [4.iii]

`rtest` follows the recommendations for code coverage defined in the [ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#code-coverage), and targets to achieve line coveraget above 95%.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (>=95%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml).

### Performance [4.iv]

`rtest` doesn't have explicit performace tests. However it is assumed using the `rtest` significantly increases unit tests performace in comparison to unit tests that invole ROS 2 middleware and execution model.

### Linters and Static Analysis [4.v]

`rtest` uses and passes all the ROS 2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen [here](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-clang-tidy.yml)

## Dependencies [5]

Below are evaluations of each of `rtest`'s run-time and build-time dependencies that have been determined to influence the quality.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`rtest` has the following runtime ROS 2 dependencies:

#### `rclcpp`

`rclcpp` - The ROS 2 client library in C++.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/QUALITY_DECLARATION.md).

#### `ament_cmake_ros`

`ament_cmake_ros` - The ROS specific CMake bits in the ament buildsystem.

No quality level is declared for `ament_cmake_ros` package.

#### `gmock_vendor`

`gmock_vendor` - Googletest Mocking (gMock) Framework.

No quality level is declared for `gmock_vendor` package.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`rtest` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

[TODO] Add support for all tier 1 platforms and include them in CI build.

`rtest` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently build status can be seen [here](https://github.com/Beam-and-Spyrosoft/rtest/actions/workflows/ros2-pull-request.yml?query=branch%3Amain)

## Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
More details can be found in [`SECURITY.md`](./SECURITY.md) file

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the `rtest` package.
|Number|  Requirement| Current state |
|--|--|--|
|1| **Version policy** |---|
|1.i| Version Policy available | ✓ |
|1.ii| Stable version | ☓ |
|1.iii| Declared public API | ✓ |
|1.iv| API stability policy | ☓ |
|1.v| ABI stability policy  | ☓ |
|1.vi| API/ABI stable within ROS distribution | ☓ |
|2| **Change control process** |---|
|2.i| All changes occur on change request | ✓ |
|2.ii| Contributor origin (DCO, CLA, etc) | ✓ |
|2.iii| Peer review policy | ✓ |
|2.iv| CI policy for change requests | ✓ |
|2.v| Documentation policy for change requests | ✓ |
|3| **Documentation** | --- |
|3.i| Per feature documentation | ☓ |
|3.ii| Per public API item documentation | ☓ |
|3.iii| Declared License(s) | ✓ |
|3.iv| Copyright in source files | ✓ |
|3.v| Have a "quality declaration" document | ✓ |
|3.v.a| Quality declaration linked to README | ✓ |
|3.v.b| Centralized declaration available for peer review | ✓ |
|3.v.c| Public Declaration and Peer Review | ✓ |
|4| Testing | --- |
|4.i| Feature items tests | ☓ |
|4.ii| Public API tests | ☓ |
|4.iii.a| Coverage tracking | ✓ |
|4.iii.b| Enforced coverage policy | ✓ |
|4.iv.a| Performance tests (if applicable) | ☓ |
|4.iv.b| Performance tests policy | ☓ |
|4.v.a| Code style enforcement (linters) | ✓ |
|4.v.b| Use of static analysis tools | ✓ |
|5| Dependencies | --- |
|5.i| Must not have ROS lower level dependencies | ✓ |
|5.ii| Optional ROS lower level dependencies | ✓ |
|5.iii| Justifies quality use of non-ROS dependencies | ✓ |
|6| Platform support | --- |
|6.i| Support targets Tier1 ROS platforms | ☓ |
|7| Security | --- |
|7.i| Vulnerability Disclosure Policy | ✓ |
