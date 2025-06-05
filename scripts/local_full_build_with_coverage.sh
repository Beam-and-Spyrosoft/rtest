#!/bin/bash

# This script runs the coverage report generation and threshold check locally
# Usage: ./local_coverage_report.sh [lines_threshold] [functions_threshold]
# Default thresholds are 60% if not specified

# Set the default thresholds
LINES_THRESHOLD="${1:-60.0}"
FUNCTIONS_THRESHOLD="${2:-60.0}"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Check if required tools are installed
check_dependencies() {
  local missing_deps=0

  if ! command -v colcon &> /dev/null; then
    echo "ERROR: 'colcon' is not installed. Install it with: sudo apt-get install -y python3-colcon-common-extensions"
    missing_deps=1
  fi

  if ! command -v cmake &> /dev/null; then
    echo "ERROR: 'cmake' is not installed. Install it with: sudo apt-get install -y cmake"
    missing_deps=1
  fi

  if ! command -v lcov &> /dev/null; then
    echo "ERROR: 'lcov' is not installed. Install it with: sudo apt-get install -y lcov"
    missing_deps=1
  fi

  # Check if ROS is installed
  if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: ROS ${ROS_DISTRO} not found."
    missing_deps=1
  fi

  # Check if the generate_coverage.sh and check_coverage_thresholds.sh scripts exist
  if [ ! -f "${SCRIPT_DIR}/generate_coverage.sh" ]; then
    echo "ERROR: '${SCRIPT_DIR}/generate_coverage.sh' not found."
    missing_deps=1
  fi

  if [ ! -f "${SCRIPT_DIR}/check_coverage_thresholds.sh" ]; then
    echo "ERROR: '${SCRIPT_DIR}/check_coverage_thresholds.sh' not found."
    missing_deps=1
  fi

  if [ $missing_deps -ne 0 ]; then
    exit 1
  fi
}

# Execute dependency check
check_dependencies

echo "===== RUNNING LOCAL COVERAGE REPORT ====="
echo "Using thresholds: ${LINES_THRESHOLD}% (lines), ${FUNCTIONS_THRESHOLD}% (functions)"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Step 1: Build the project with coverage flags
echo -e "\n===== STEP 1: BUILDING PROJECT WITH COVERAGE FLAGS ====="
colcon build --event-handlers console_cohesion+  --cmake-args -DBUILD_TESTING=On -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage" -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage" -DCMAKE_EXE_LINKER_FLAGS="-lgcov --coverage"

if [ $? -ne 0 ]; then
  echo "❌ Build failed"
  exit 1
fi
echo "✅ Build completed successfully"

# Step 2: Run tests
echo -e "\n===== STEP 2: RUNNING TESTS ====="
# Source the local setup after building
if [ -f "install/setup.bash" ]; then
  . install/setup.bash
fi

colcon test --event-handlers console_cohesion+
colcon test-result

# Verify test results
test_results=$(colcon test-result --verbose | tee /dev/stderr)
if ! echo "$test_results" | grep -q "0 errors, 0 failures"; then
  echo "❌ Tests failed or had errors"
  exit 1
fi
echo "✅ Tests completed successfully"

# Step 3: Generate coverage report
echo -e "\n===== STEP 3: GENERATING COVERAGE REPORT ====="
"${SCRIPT_DIR}/generate_coverage.sh"
if [ $? -ne 0 ]; then
  echo "❌ Failed to generate coverage report"
  exit 1
fi

# Step 4: Check coverage thresholds
echo -e "\n===== STEP 4: CHECKING COVERAGE THRESHOLDS ====="
"${SCRIPT_DIR}/check_coverage_thresholds.sh" "$LINES_THRESHOLD" "$FUNCTIONS_THRESHOLD"
THRESHOLD_CHECK_RESULT=$?

# Display results
echo -e "\n===== RESULTS ====="
if [ -f "coverage_report_framework/index.html" ]; then
  REPORT_PATH="$(pwd)/coverage_report_framework/index.html"
  echo "📊 Coverage report generated: $REPORT_PATH"
  echo "   Open it in a browser with: xdg-open $REPORT_PATH"
else
  echo "⚠️ HTML coverage report not found."
fi

if [ $THRESHOLD_CHECK_RESULT -eq 0 ]; then
  echo "✅ Coverage thresholds check passed!"
else
  echo "❌ Coverage thresholds check failed!"
fi

# Exit with the threshold check result
exit $THRESHOLD_CHECK_RESULT