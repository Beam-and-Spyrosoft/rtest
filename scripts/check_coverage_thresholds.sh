#!/bin/bash
# Fail if the coverage produced by generate_coverage.sh is below the thresholds.
#
# Usage: check_coverage_thresholds.sh <coverage_dir> [lines] [functions] [branches]
# Line and function thresholds default to 70%. The branch threshold defaults to 0%: branch
# coverage is reported, but not enforced until a threshold is given.
set -euo pipefail

COVERAGE_DIR="${1:?usage: check_coverage_thresholds.sh <coverage_dir> [lines] [functions] [branches]}"
LINES_THRESHOLD="${2:-70.0}"
FUNCTIONS_THRESHOLD="${3:-70.0}"
BRANCHES_THRESHOLD="${4:-0.0}"
SUMMARY="${COVERAGE_DIR}/summary.env"

echo "===== CHECKING COVERAGE THRESHOLDS ====="
echo "Required thresholds: ${LINES_THRESHOLD}% lines, ${FUNCTIONS_THRESHOLD}% functions," \
  "${BRANCHES_THRESHOLD}% branches"

if [ ! -f "${SUMMARY}" ]; then
  echo "❌ ${SUMMARY} not found. Run generate_coverage.sh first."
  exit 1
fi
# shellcheck source=/dev/null
source "${SUMMARY}"

FAILED=0

# check <name> <coverage> <threshold>
check() {
  if awk "BEGIN {exit !($2 < $3)}"; then
    echo "❌ $1 coverage ($2%) is below the required threshold ($3%)"
    FAILED=1
  else
    echo "✅ $1 coverage ($2%) meets the required threshold ($3%)"
  fi
}

check "Line" "${LINES}" "${LINES_THRESHOLD}"
check "Function" "${FUNCTIONS}" "${FUNCTIONS_THRESHOLD}"
if awk "BEGIN {exit !(${BRANCHES_THRESHOLD} > 0)}"; then
  check "Branch" "${BRANCHES}" "${BRANCHES_THRESHOLD}"
else
  echo "ℹ️  Branch coverage (${BRANCHES}%) is reported, but not enforced"
fi

if [ ${FAILED} -eq 1 ]; then
  echo "❌ Overall: Code coverage does not meet all required thresholds."
  exit 1
fi

echo "✅ Overall: Code coverage meets all required thresholds."
