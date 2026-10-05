#!/bin/bash
# Generate an lcov coverage report for the rtest framework library from a
# gcc --coverage build (see the `coverage` pixi task).
#
# Usage: generate_coverage.sh <build_dir> <output_dir>
#
# Writes to <output_dir>:
#   coverage.info  - filtered lcov tracefile
#   html/          - genhtml report
#   summary.env    - LINES=<pct> FUNCTIONS=<pct>, read by check_coverage_thresholds.sh
#   summary.json   - {"distro", "lines", "functions"}, read by coverage_site.py
set -euo pipefail

BUILD_DIR="${1:?usage: generate_coverage.sh <build_dir> <output_dir>}"
OUTPUT_DIR="${2:?usage: generate_coverage.sh <build_dir> <output_dir>}"
WORKSPACE="$(pwd)"

if ! command -v lcov &> /dev/null; then
  echo "ERROR: 'lcov' not found. Run this script through 'pixi run coverage'."
  exit 1
fi

# The conda gcc toolchain ships a target-prefixed gcov next to $CC
# (e.g. x86_64-conda-linux-gnu-cc -> x86_64-conda-linux-gnu-gcov). It must match
# the compiler version, the system gcov cannot read gcc 16 .gcda files.
GCOV_TOOL="${GCOV:-}"
if [ -z "${GCOV_TOOL}" ] && [ -n "${CC:-}" ] && command -v "${CC%-cc}-gcov" &> /dev/null; then
  GCOV_TOOL="${CC%-cc}-gcov"
fi
GCOV_TOOL="${GCOV_TOOL:-gcov}"

# "inconsistent" is listed twice so lcov also suppresses its warnings: gtest's
# TEST() macros produce dozens of harmless "mismatched end line" reports.
LCOV_IGNORE="mismatch,source,unused,inconsistent,inconsistent,empty"

rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"

echo "===== GENERATING COVERAGE REPORT (${BUILD_DIR}, gcov: ${GCOV_TOOL}) ====="

lcov --capture \
  --directory "${BUILD_DIR}" \
  --base-directory "${WORKSPACE}" \
  --gcov-tool "${GCOV_TOOL}" \
  --output-file "${OUTPUT_DIR}/all.info" \
  --ignore-errors "${LCOV_IGNORE}"

# Keep only the framework sources. The pixi environment lives inside the
# workspace (.pixi/), so it is excluded explicitly together with tests,
# examples and generated code.
lcov --extract "${OUTPUT_DIR}/all.info" "${WORKSPACE}/rtest/*" \
  --ignore-errors "${LCOV_IGNORE}" \
  --output-file "${OUTPUT_DIR}/framework.info"
lcov --remove "${OUTPUT_DIR}/framework.info" \
  "*/test/*" "*/tests/*" "*/test_composition/*" \
  --ignore-errors "${LCOV_IGNORE}" \
  --output-file "${OUTPUT_DIR}/coverage.info"
rm -f "${OUTPUT_DIR}/all.info" "${OUTPUT_DIR}/framework.info"

genhtml "${OUTPUT_DIR}/coverage.info" \
  --output-directory "${OUTPUT_DIR}/html" \
  --prefix "${WORKSPACE}" \
  --title "rtest ${ROS_DISTRO:-}" \
  --ignore-errors "source,empty"

# Take the totals from `lcov --summary` so they match the HTML report
# (lcov merges template instantiations when counting functions).
SUMMARY_TEXT="$(lcov --summary "${OUTPUT_DIR}/coverage.info" --ignore-errors "${LCOV_IGNORE}" 2>&1)"
summary_pct() {
  echo "${SUMMARY_TEXT}" | awk -v key="$1" '$1 ~ "^" key "\\." { gsub(/%/, "", $2); print $2; found = 1 }
    END { if (!found) print "0.0" }'
}
LINES="$(summary_pct lines)"
FUNCTIONS="$(summary_pct functions)"

printf 'LINES=%s\nFUNCTIONS=%s\n' "${LINES}" "${FUNCTIONS}" > "${OUTPUT_DIR}/summary.env"
printf '{"distro": "%s", "lines": %s, "functions": %s}\n' \
  "${ROS_DISTRO:-unknown}" "${LINES}" "${FUNCTIONS}" > "${OUTPUT_DIR}/summary.json"

echo "Framework library coverage (${ROS_DISTRO:-unknown}): ${LINES}% lines, ${FUNCTIONS}% functions"
echo "HTML report: ${OUTPUT_DIR}/html/index.html"

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "### Coverage: ${ROS_DISTRO:-unknown}"
    echo ""
    echo "| Lines | Functions |"
    echo "|---|---|"
    echo "| ${LINES}% | ${FUNCTIONS}% |"
  } >> "${GITHUB_STEP_SUMMARY}"
fi
