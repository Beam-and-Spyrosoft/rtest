#!/bin/bash
# Fail if the coverage produced by generate_coverage.sh is below the thresholds.
#
# Usage: check_coverage_thresholds.sh <coverage_dir> [lines_threshold] [functions_threshold]
# Thresholds default to 60%.
set -euo pipefail

COVERAGE_DIR="${1:?usage: check_coverage_thresholds.sh <coverage_dir> [lines] [functions]}"
LINES_THRESHOLD="${2:-60.0}"
FUNCTIONS_THRESHOLD="${3:-60.0}"
SUMMARY="${COVERAGE_DIR}/summary.env"

echo "===== CHECKING COVERAGE THRESHOLDS ====="
echo "Required thresholds: ${LINES_THRESHOLD}% lines, ${FUNCTIONS_THRESHOLD}% functions"

if [ ! -f "${SUMMARY}" ]; then
  echo "❌ ${SUMMARY} not found. Run generate_coverage.sh first."
  exit 1
fi
# shellcheck source=/dev/null
source "${SUMMARY}"

FAILED=0

if awk "BEGIN {exit !(${LINES} < ${LINES_THRESHOLD})}"; then
  echo "❌ Line coverage (${LINES}%) is below the required threshold (${LINES_THRESHOLD}%)"
  FAILED=1
else
  echo "✅ Line coverage (${LINES}%) meets the required threshold (${LINES_THRESHOLD}%)"
fi

if awk "BEGIN {exit !(${FUNCTIONS} < ${FUNCTIONS_THRESHOLD})}"; then
  echo "❌ Function coverage (${FUNCTIONS}%) is below the required threshold (${FUNCTIONS_THRESHOLD}%)"
  FAILED=1
else
  echo "✅ Function coverage (${FUNCTIONS}%) meets the required threshold (${FUNCTIONS_THRESHOLD}%)"
fi

if [ ${FAILED} -eq 1 ]; then
  echo "❌ Overall: Code coverage does not meet all required thresholds."
  exit 1
fi

echo "✅ Overall: Code coverage meets all required thresholds."
