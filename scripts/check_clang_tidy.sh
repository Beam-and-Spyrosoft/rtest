#!/bin/bash
# This script runs clang-tidy tests on the ROS 2 project
# Usage: ./rtest/scripts/check_clang_tidy.sh

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Check if required tools are installed
check_dependencies() {
  local missing_deps=0

  if ! command -v colcon &> /dev/null; then
    echo "ERROR: 'colcon' is not installed. Install it with: sudo apt-get install -y python3-colcon-common-extensions"
    missing_deps=1
  fi

  if ! command -v clang-tidy &> /dev/null; then
    echo "ERROR: 'clang-tidy' is not installed. Install it with: sudo apt-get install -y clang-tidy"
    missing_deps=1
  fi

  # Check if ROS Jazzy is installed
  if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS Jazzy not found."
    missing_deps=1
  fi

  if [ $missing_deps -ne 0 ]; then
    exit 1
  fi
}

# Execute dependency check
check_dependencies

echo "===== RUNNING CLANG-TIDY ANALYSIS ====="

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  . /opt/ros/jazzy/setup.bash
fi

# Step 1: Build the project with compile_commands.json and ENABLE_CLANG_TIDY=ON
echo -e "\n===== STEP 1: BUILDING PROJECT WITH CLANG-TIDY ENABLED ====="
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DENABLE_CLANG_TIDY=ON

if [ $? -ne 0 ]; then
  echo "❌ Build failed"
  exit 1
fi

echo "✅ Build completed successfully"

# Step 2: Run clang-tidy tests
echo -e "\n===== STEP 2: RUNNING CLANG-TIDY TESTS ====="

# Source the local setup after building
if [ -f "install/setup.bash" ]; then
  . install/setup.bash
fi

colcon test --event-handlers console_cohesion+ --packages-select rtest rtest_examples --ctest-args -R clang_tidy -V

# Step 3: Check test results
test_results=$(colcon test-result --verbose)
echo "$test_results"

if echo "$test_results" | grep -q "Failed"; then
  echo "❌ Clang-tidy tests had errors or failures"
  exit 1
fi

echo "✅ Clang-tidy tests completed successfully"

exit 0