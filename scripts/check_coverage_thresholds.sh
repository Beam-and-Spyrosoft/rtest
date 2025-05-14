#!/bin/bash

# Set default thresholds if not provided (default to 60%)
LINES_THRESHOLD="${1:-60.0}"
FUNCTIONS_THRESHOLD="${2:-60.0}"

echo "===== CHECKING COVERAGE THRESHOLDS ====="
echo "Required thresholds: ${LINES_THRESHOLD}% lines, ${FUNCTIONS_THRESHOLD}% functions"

# Try to get coverage values from environment variables (CI) or from file (local)
if [ -n "${FRAMEWORK_LINES_COVERAGE}" ] && [ -n "${FRAMEWORK_FUNCTIONS_COVERAGE}" ]; then
  # Values from environment variables (CI)
  FRAMEWORK_LINES="${FRAMEWORK_LINES_COVERAGE}"
  FRAMEWORK_FUNCTIONS="${FRAMEWORK_FUNCTIONS_COVERAGE}"
elif [ -f ".coverage_values" ]; then
  # Values from file (local)
  source .coverage_values
  FRAMEWORK_LINES="${FRAMEWORK_LINES_COVERAGE}"
  FRAMEWORK_FUNCTIONS="${FRAMEWORK_FUNCTIONS_COVERAGE}"
else
  # Try to parse from the coverage summary file
  if [ -f "coverage_artifacts/framework_coverage_summary.txt" ]; then
    FRAMEWORK_LINES=$(grep "Lines:" coverage_artifacts/framework_coverage_summary.txt | awk '{gsub(/%/,""); print $2}')
    FRAMEWORK_FUNCTIONS=$(grep "Functions:" coverage_artifacts/framework_coverage_summary.txt | awk '{gsub(/%/,""); print $2}')
  else
    echo "WARNING: Could not find coverage data. Using default values of 0%."
    FRAMEWORK_LINES="0.0"
    FRAMEWORK_FUNCTIONS="0.0"
  fi
fi

echo "Framework library coverage: ${FRAMEWORK_LINES}% lines, ${FRAMEWORK_FUNCTIONS}% functions"

FRAMEWORK_LINES_FLOAT=$(printf "%.2f" "$FRAMEWORK_LINES")
FRAMEWORK_FUNCTIONS_FLOAT=$(printf "%.2f" "$FRAMEWORK_FUNCTIONS")
FAILED=0

if awk "BEGIN {exit !($FRAMEWORK_LINES_FLOAT < $LINES_THRESHOLD)}"; then
  echo "❌ Line coverage (${FRAMEWORK_LINES}%) is below the required threshold (${LINES_THRESHOLD}%)"
  FAILED=1
else
  echo "✅ Line coverage (${FRAMEWORK_LINES}%) meets the required threshold (${LINES_THRESHOLD}%)"
fi

if awk "BEGIN {exit !($FRAMEWORK_FUNCTIONS_FLOAT < $FUNCTIONS_THRESHOLD)}"; then
  echo "❌ Function coverage (${FRAMEWORK_FUNCTIONS}%) is below the required threshold (${FUNCTIONS_THRESHOLD}%)"
  FAILED=1
else
  echo "✅ Function coverage (${FRAMEWORK_FUNCTIONS}%) meets the required threshold (${FUNCTIONS_THRESHOLD}%)"
fi

if [ $FAILED -eq 1 ]; then
  echo "❌ Overall: Code coverage does not meet all required thresholds."
  exit 1
fi

echo "✅ Overall: Code coverage meets all required thresholds."
exit 0