#!/usr/bin/env bash

# get the script file path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

echo "===== BUILDING DOCUMENTATION ====="

# package directory
PACKAGE_DIR="${SCRIPT_DIR}/../ros2_test_framework"

rosdoc2 build --package-path "${PACKAGE_DIR}"