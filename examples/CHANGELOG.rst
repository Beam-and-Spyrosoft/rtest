^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtest_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix clang-tidy issues (`#104 <https://github.com/Beam-and-Spyrosoft/rtest/issues/104>`_)
* Trigger timer callbacks when time is moved (`#98 <https://github.com/Beam-and-Spyrosoft/rtest/issues/98>`_)
  Co-authored-by: Sławek Cielepak <slawomir.cielepak@gmail.com>
* Contributors: Sławomir Cielepak, Wiktor Bajor

0.2.0 (2025-06-23)
------------------
* Conditional build of rtest_examples (`#86 <https://github.com/Beam-and-Spyrosoft/rtest/issues/86>`_)
* Fix GMock warning for Uninteresting mock function call (`#84 <https://github.com/Beam-and-Spyrosoft/rtest/issues/84>`_)
* Add actions support (experimental) (`#58 <https://github.com/Beam-and-Spyrosoft/rtest/issues/58>`_)
  Co-authored-by: Sławomir Cielepak <slawomir.cielepak@gmail.com>
* Merge jazzy -> main
* Add documentation and tutorials for RTEST framework (`#49 <https://github.com/Beam-and-Spyrosoft/rtest/issues/49>`_)
  Co-authored-by: MariuszSzczepanikSpyrosoft <118888269+MariuszSzczepanikSpyrosoft@users.noreply.github.com>
* Add convenience std_msgs handling (`#57 <https://github.com/Beam-and-Spyrosoft/rtest/issues/57>`_)
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
* CI code style check and update code (`#36 <https://github.com/Beam-and-Spyrosoft/rtest/issues/36>`_)
  * Fix comments in code
  * Unification of style
  * Add format cpp check
  * Created scripts to simplify code on CI
  * Recover code
  * Fix finding
  ---------
* Add automated linters for copyright checks (`#33 <https://github.com/Beam-and-Spyrosoft/rtest/issues/33>`_)
  * Add automated linters for copyright checks
  * Use ament copyfight and update header files
  * Restore file, author, date and description info in file headers
  ---------
  Co-authored-by: Sławomir Cielepak <slawomir.cielepak@gmail.com>
* Update ROS clang format rules (`#35 <https://github.com/Beam-and-Spyrosoft/rtest/issues/35>`_)
* ROS linter rules refactor (`#34 <https://github.com/Beam-and-Spyrosoft/rtest/issues/34>`_)
* Update maintainer email (`#22 <https://github.com/Beam-and-Spyrosoft/rtest/issues/22>`_)
* Add Quality Declaration (`#21 <https://github.com/Beam-and-Spyrosoft/rtest/issues/21>`_)
* Remove not needed dep (`#16 <https://github.com/Beam-and-Spyrosoft/rtest/issues/16>`_)
* Remove logger (`#15 <https://github.com/Beam-and-Spyrosoft/rtest/issues/15>`_)
* Support rclcpp services (`#9 <https://github.com/Beam-and-Spyrosoft/rtest/issues/9>`_)
  * Added clang format and fix findings
  * Support rclcpp services
  * Cmake fix
  * pixi: added std-srvs dependencies
  * Change the way of mocking services (`#12 <https://github.com/Beam-and-Spyrosoft/rtest/issues/12>`_)
  * Change the way of mocking services
  * Cleanup
  * Extend example service test
  * Add to README integration guide and how to run (`#10 <https://github.com/Beam-and-Spyrosoft/rtest/issues/10>`_)
  * Added build and how to run tests
  * Added integration guide to README
  * Update
  * Use updateRemoteUserUID instead of custom container user (`#11 <https://github.com/Beam-and-Spyrosoft/rtest/issues/11>`_)
  * Removed creation of dev container user with hardcoded UID/GID.
  * Used UID Update functionality of dev containers (will align UID/GID
  automatically).
  * Added clang-format extension and PTRACE cap for debugging.
  Co-authored-by: Sławomir Cielepak <sie@spyro-soft.com>
  * Turn of clang-format in service base rclcpp code
  ---------
  Co-authored-by: Guy Burroughes <gb24@beam.global>
  Co-authored-by: Sławomir Cielepak <slawomir.cielepak@gmail.com>
  Co-authored-by: Sławomir Cielepak <sie@spyro-soft.com>
* Added clang format and fix findings (`#8 <https://github.com/Beam-and-Spyrosoft/rtest/issues/8>`_)
  * Added clang format and fix findings
  * Fix file headers
* Remove "All rights reserved from copyright headers" (`#1 <https://github.com/Beam-and-Spyrosoft/rtest/issues/1>`_)
* Transfer
* Contributors: MariuszSzczepanikSpyrosoft, Sławomir Cielepak, guyVaarst
