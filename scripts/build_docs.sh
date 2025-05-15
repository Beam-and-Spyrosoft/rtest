#!/usr/bin/env bash

# get the script file path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

echo "===== BUILDING DOCUMENTATION ====="

# package directory
PACKAGE_DIR="${SCRIPT_DIR}/../rtest"

rm -r cross_reference docs_build docs_output

rosdoc2 build --package-path "${PACKAGE_DIR}"

# Protect colcon from seeing a duplicated `rtest` package
touch docs_build/COLCON_IGNORE