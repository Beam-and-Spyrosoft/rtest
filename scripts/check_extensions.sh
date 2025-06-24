#!/bin/bash

# Find files with incorrect extensions
INCORRECT_EXTENSIONS=$(find . -type f \( -name "*.h" -o -name "*.cc" \) | grep -v "build/" | grep -v "install/" | sort)

if [ -n "$INCORRECT_EXTENSIONS" ]; then
  echo "ERROR: Found files with incorrect extensions:"
  echo "$INCORRECT_EXTENSIONS"
  echo ""
  echo "According to project standards:"
  echo " - Header files should use .hpp extension instead of .h"
  echo " - Implementation files should use .cpp extension instead of .cc"
  exit 1
fi
echo "✓ No files with incorrect extensions found."
exit 0
