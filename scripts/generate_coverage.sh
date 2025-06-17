#!/bin/bash

# Check if required tools are installed
check_dependencies() {
  local missing_deps=0

  if ! command -v lcov &> /dev/null; then
    echo "ERROR: 'lcov' is not installed. Install it with: apt-get install -y lcov"
    missing_deps=1
  fi
  if ! command -v gcov &> /dev/null; then
    echo "ERROR: 'gcov' is not installed. Install it with: apt-get install -y gcc"
    missing_deps=1
  fi
  # Check if ROS Jazzy is installed
  if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: ROS ${ROS_DISTRO} not found"
    missing_deps=1
  fi

  if [ $missing_deps -ne 0 ]; then
    exit 1
  fi
}

# Execute dependency check
check_dependencies

# Source ROS environment
. /opt/ros/${ROS_DISTRO}/setup.bash

echo "===== DEBUGGING COVERAGE SETUP ====="
echo "Current directory: $(pwd)"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "LCOV version: $(lcov --version 2>/dev/null || echo 'Not found')"
echo "GCOV version: $(gcov --version | head -1 2>/dev/null || echo 'Not found')"

# Find .gcda and .gcno files for debugging
echo "Searching for coverage files..."
find . -name "*.gcda" -type f | head -10
find . -name "*.gcno" -type f | head -10

# Count coverage files
GCDA_COUNT=$(find . -name "*.gcda" -type f | wc -l)
GCNO_COUNT=$(find . -name "*.gcno" -type f | wc -l)
echo "Found ${GCDA_COUNT} .gcda files and ${GCNO_COUNT} .gcno files"

if [ $GCDA_COUNT -eq 0 ]; then
    echo "WARNING: No .gcda files found. Tests may not have run with coverage enabled."
fi

# Generate coverage report for framework library
echo "===== GENERATING COVERAGE REPORT ====="

# Generate initial coverage data with Ubuntu 22.04 compatible flags
echo "Capturing coverage data..."
lcov --capture --directory . --output-file all_coverage.info 2>/dev/null || true

# Show what we captured
if [ -f "all_coverage.info" ]; then
    echo "Coverage file size: $(du -h all_coverage.info)"
    echo "Coverage file line count: $(wc -l < all_coverage.info)"
else
    echo "ERROR: No coverage data file created"
    exit 1
fi

# Extract framework-related files with multiple patterns
echo "Extracting framework coverage..."
lcov --extract all_coverage.info \
     "*/rtest/*" \
     --ignore-errors source,empty,unused,mismatch \
     --output-file framework_tmp.info 2>/dev/null || true

lcov --remove framework_tmp.info "*/examples/*" "*/test/*" "*/tests/*" "*/test_composition/*" "*/rtest_examples_interfaces/*" \
    --ignore-errors source,empty,unused,mismatch -o framework_filtered.info 2>/dev/null || {
    echo "Remove operation failed, using unfiltered data..."
    cp framework_tmp.info framework_filtered.info
}

# Generate HTML report
genhtml framework_filtered.info \
        --output-directory coverage_report_framework \
        --ignore-errors source,empty \
        --legend \
        --title "ROS2 Framework Coverage" 2>/dev/null || true

if [ -s framework_filtered.info ]; then
  echo "Overall coverage rate:"
  COVERAGE_SUMMARY=$(lcov --summary framework_filtered.info 2>&1 | grep -E 'lines|functions')
  echo "$COVERAGE_SUMMARY"

  # Extract percentage numbers from lcov output
  FRAMEWORK_LINES=$(echo "$COVERAGE_SUMMARY" | grep 'lines' | sed -n 's/.*: \([0-9.]*\)%.*/\1/p' | head -1)
  FRAMEWORK_FUNCTIONS=$(echo "$COVERAGE_SUMMARY" | grep 'functions' | sed -n 's/.*: \([0-9.]*\)%.*/\1/p' | head -1)

  echo "Calculating coverage metrics..."
  echo "Framework library coverage: ${FRAMEWORK_LINES}% (lines), ${FRAMEWORK_FUNCTIONS}% (functions)"

  # Set environment variables if running in GitHub Actions
  if [ -n "$GITHUB_ENV" ]; then
    echo "FRAMEWORK_LINES_COVERAGE=${FRAMEWORK_LINES}" >> $GITHUB_ENV
    echo "FRAMEWORK_FUNCTIONS_COVERAGE=${FRAMEWORK_FUNCTIONS}" >> $GITHUB_ENV
    echo "LINES_COVERAGE=${FRAMEWORK_LINES}" >> $GITHUB_ENV
    echo "FUNCTIONS_COVERAGE=${FRAMEWORK_FUNCTIONS}" >> $GITHUB_ENV
  fi

  # Store the coverage values in a file for local use
  echo "FRAMEWORK_LINES_COVERAGE=${FRAMEWORK_LINES}" > .coverage_values
  echo "FRAMEWORK_FUNCTIONS_COVERAGE=${FRAMEWORK_FUNCTIONS}" >> .coverage_values
else
  echo "No coverage data available"
  FRAMEWORK_LINES="0.0"
  FRAMEWORK_FUNCTIONS="0.0"
  
  # Set environment variables if running in GitHub Actions
  if [ -n "$GITHUB_ENV" ]; then
    echo "FRAMEWORK_LINES_COVERAGE=0.0" >> $GITHUB_ENV
    echo "FRAMEWORK_FUNCTIONS_COVERAGE=0.0" >> $GITHUB_ENV
    echo "LINES_COVERAGE=0.0" >> $GITHUB_ENV
    echo "FUNCTIONS_COVERAGE=0.0" >> $GITHUB_ENV
  fi

  # Store the coverage values in a file for local use
  echo "FRAMEWORK_LINES_COVERAGE=0.0" > .coverage_values
  echo "FRAMEWORK_FUNCTIONS_COVERAGE=0.0" >> .coverage_values
fi

mkdir -p coverage_artifacts
[ -d "coverage_report_framework" ] && cp -r coverage_report_framework coverage_artifacts/ || \
  mkdir -p coverage_artifacts/coverage_report_framework

{
  echo "ROS2 TEST FRAMEWORK LIBRARY COVERAGE SUMMARY"
  echo "============================================"
  echo ""
  echo "Overall coverage:"
  echo "  Lines:     ${FRAMEWORK_LINES:-0.0}%"
  echo "  Functions: ${FRAMEWORK_FUNCTIONS:-0.0}%"
} > coverage_artifacts/framework_coverage_summary.txt

echo "✅ Coverage report generated successfully"
echo "FRAMEWORK_LINES=${FRAMEWORK_LINES}"
echo "FRAMEWORK_FUNCTIONS=${FRAMEWORK_FUNCTIONS}"