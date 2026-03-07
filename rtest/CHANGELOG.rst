^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtest
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix missing includes (`#110 <https://github.com/Beam-and-Spyrosoft/rtest/issues/110>`_)
* Fix REP url locations (`#106 <https://github.com/Beam-and-Spyrosoft/rtest/issues/106>`_)
* Add wait_for_service to ServiceClientMock (`#101 <https://github.com/Beam-and-Spyrosoft/rtest/issues/101>`_)
* Add auto linking of GTest::gmock target (`#102 <https://github.com/Beam-and-Spyrosoft/rtest/issues/102>`_)
* Trigger timer callbacks when time is moved (`#98 <https://github.com/Beam-and-Spyrosoft/rtest/issues/98>`_)
  Co-authored-by: Sławek Cielepak <slawomir.cielepak@gmail.com>
* Fix install cmake path (`#99 <https://github.com/Beam-and-Spyrosoft/rtest/issues/99>`_)
* Typo fix: namme -> name  (`#90 <https://github.com/Beam-and-Spyrosoft/rtest/issues/90>`_)
* Add MockRegistryCleaner utility to reset StaticMocksRegistry (`#85 <https://github.com/Beam-and-Spyrosoft/rtest/issues/85>`_)
* Add build dependency to ros_environment (`#88 <https://github.com/Beam-and-Spyrosoft/rtest/issues/88>`_)
* `#79 <https://github.com/Beam-and-Spyrosoft/rtest/issues/79>`_ Docs/publisher tutorial (`#80 <https://github.com/Beam-and-Spyrosoft/rtest/issues/80>`_)
* Contributors: Kuo, Mei-Chun, Sławomir Cielepak, Tim Clephas, Wiktor Bajor, gmcross, rik-winters

0.2.0 (2025-06-23)
------------------
* Conditional build of rtest_examples (`#86 <https://github.com/Beam-and-Spyrosoft/rtest/issues/86>`_)
* Add actions support (experimental) (`#58 <https://github.com/Beam-and-Spyrosoft/rtest/issues/58>`_)
  Co-authored-by: Sławomir Cielepak <slawomir.cielepak@gmail.com>
* Add rtest limitation to docs (`#82 <https://github.com/Beam-and-Spyrosoft/rtest/issues/82>`_)
* Add new maintainer (`#78 <https://github.com/Beam-and-Spyrosoft/rtest/issues/78>`_)
* Merge jazzy -> main
* Add documentation and tutorials for RTEST framework (`#49 <https://github.com/Beam-and-Spyrosoft/rtest/issues/49>`_)
  Co-authored-by: MariuszSzczepanikSpyrosoft <118888269+MariuszSzczepanikSpyrosoft@users.noreply.github.com>
* Support rolling dist (`#71 <https://github.com/Beam-and-Spyrosoft/rtest/issues/71>`_)
* Fix boost dep (`#61 <https://github.com/Beam-and-Spyrosoft/rtest/issues/61>`_)
* Multi-distro support build system (`#66 <https://github.com/Beam-and-Spyrosoft/rtest/issues/66>`_)
  * The base idea of multi-distro separation
  * Upgrade CI verification
  * Upgrade CI verification
  * Fixed review findings and CI upload
  * Upgrade CI verification
  ---------
* Add convenience std_msgs handling (`#57 <https://github.com/Beam-and-Spyrosoft/rtest/issues/57>`_)
* Fix clang build, remove invalid template keyword usage (`#55 <https://github.com/Beam-and-Spyrosoft/rtest/issues/55>`_)
* Rename common lib (`#47 <https://github.com/Beam-and-Spyrosoft/rtest/issues/47>`_)
* Add project documentation (`#44 <https://github.com/Beam-and-Spyrosoft/rtest/issues/44>`_)
  * Add documentation
  * Move doc config files to package root
  * Fixes after merge
  * Move doc -> docs
  * Enable Python requirements in readthedocs.yaml
  * Update logo in docs
  * Package rename
  * Refactor documentation structure and update configuration for Sphinx build
  * Switch docs to Markdown format
  * Add license and static code analysis badges to README
  * Fix formatting in README by adding a newline for better readability
  * Fix unmerged files
  * Fixed code (`#45 <https://github.com/Beam-and-Spyrosoft/rtest/issues/45>`_)
  * Update requirements.txt to specify versions for exhale and linkify
  * Update requirements.txt to specify the correct package name for linkify
  * Refactor contributing documentation and update requirements for linkify-it-py version
  * Add RTEST logo and update documentation
  - Added RTEST logo to the Concepts index page.
  - Included the ros2_comm_flow.png image in the Concepts directory.
  - Updated Installation documentation to reflect the correct Ubuntu version (24.04 Noble Numbat).
  - Revised index.md to improve the structure and clarity of the Getting Started section, including links to Installation, Tutorials, Concepts, and C++ API documentation.
  - Removed the rosdoc2.yaml configuration file as it is no longer needed.
  * Update CHANGELOG
  * Update date in changelog
  * Update QUALITY_DECLARATION.md with links to documentation and linter results
  * Update CHANGELOG to correct service mock description
  ---------
  Co-authored-by: MariuszSzczepanikSpyrosoft <118888269+MariuszSzczepanikSpyrosoft@users.noreply.github.com>
* Add clang tidy support (`#39 <https://github.com/Beam-and-Spyrosoft/rtest/issues/39>`_)
  * Rename ros test framework to rtest
  * Add clang-tidy verification
  * Fix CI yaml
  * Fix CI yaml
  * Fixed findings
  * Run text only witout clang_tidy
  * Make clang-tidy analysis a nighly workflow (`#42 <https://github.com/Beam-and-Spyrosoft/rtest/issues/42>`_)
  * Use ament_clang_tidy for static analysis
  * Remove on: push for nighlt builds
  * Add on push to dev branch for testing
  * Remove badge generation
  * Use relative paths in workflow
  * Add failure exit
  * Rename the clang-tidy workflow
  * Add badge
  * Switch badge to main branch
  * Remove test trigger
  * Ignore cppcoreguidelines-avoid-magic-numbers check in .clang-tidy
  ---------
  Co-authored-by: Mariusz Szczepanik <mua@spyro-soft.com>
  * Fixed formating
  ---------
  Co-authored-by: Sławomir Cielepak <slawomir.cielepak@gmail.com>
* Rename ros test framework to rtest (`#37 <https://github.com/Beam-and-Spyrosoft/rtest/issues/37>`_)
* Contributors: Jakub Codogni, MariuszSzczepanikSpyrosoft, Sławomir Cielepak, Tim Clephas

### 0.1.0 (2025-05-19)
* Initial release of rtest package.
* Supports running tests in ROS 2 using the Google Test framework.
* Provides mocks for ROS 2 publishers and subscribers.
* Provides mocks for ROS 2 services and clients.
* Provides a mock for ROS 2 timers.
* Provides mock for ROS 2 logger.
* Provides clock control for testing time-dependent code.
* Includes example tests for common ROS 2 use cases.