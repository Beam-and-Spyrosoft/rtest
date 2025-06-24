#!/usr/bin/env bash

# get the script file path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

echo "===== BUILDING DOCUMENTATION ====="

# package directory
PACKAGE_DIR="${SCRIPT_DIR}/../rtest"

pushd ${PACKAGE_DIR} > /dev/null

rm -r doc/_build doc/api doc/html doc/xml

sphinx-build -b html doc doc/_build/html  

# Protect colcon from seeing a duplicated `rtest` package
# touch docs_build/COLCON_IGNORE

popd > /dev/null
echo "===== DOCUMENTATION BUILT ====="