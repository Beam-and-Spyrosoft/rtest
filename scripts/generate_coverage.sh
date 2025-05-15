#!/bin/bash

# Check if required tools are installed
check_dependencies() {
  local missing_deps=0

  if ! command -v lcov &> /dev/null; then
    echo "ERROR: 'lcov' is not installed. Install it with: apt-get install -y lcov"
    missing_deps=1
  fi

  # Check if ROS Jazzy is installed
  if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS Jazzy not found"
    missing_deps=1
  fi

  if [ $missing_deps -ne 0 ]; then
    exit 1
  fi
}

# Execute dependency check
check_dependencies

# Source ROS environment
. /opt/ros/jazzy/setup.bash

# Generate coverage report for framework library
echo "===== GENERATING COVERAGE REPORT ====="

lcov --no-external --capture --directory . --output-file all_coverage.info --ignore-errors mismatch,source,unused 2>/dev/null || true
lcov --extract all_coverage.info "*/rtest/*" --ignore-errors source,empty,unused -o framework_tmp.info 2>/dev/null || true
lcov --remove framework_tmp.info "*/examples/*" "*/test/*" "*/tests/*" "*/test_composition/*" \
    --ignore-errors source,empty,unused -o framework_filtered.info 2>/dev/null || true

genhtml -o coverage_report_framework framework_filtered.info --ignore-errors source 2>/dev/null || true

if [ -s framework_filtered.info ]; then
  echo "Overall coverage rate:"
  COVERAGE_SUMMARY=$(lcov --summary framework_filtered.info 2>&1 | grep -E 'lines|functions')
  echo "$COVERAGE_SUMMARY"

  FRAMEWORK_LINES=$(echo "$COVERAGE_SUMMARY" | grep 'lines' | awk '{gsub(/%/,""); if ($2 > 100.0) print "100.0"; else print $2}' || echo "0.0")
  FRAMEWORK_FUNCTIONS=$(echo "$COVERAGE_SUMMARY" | grep 'functions' | awk '{gsub(/%/,""); if ($2 > 100.0) print "100.0"; else print $2}' || echo "0.0")

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