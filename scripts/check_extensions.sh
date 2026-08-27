#!/bin/bash

# Project sources must use .hpp / .cpp. Flag other common C/C++ extensions.
INCORRECT_EXTENSIONS=$(find rtest examples -type f \( \
  -name '*.h' -o -name '*.hh' -o -name '*.hxx' -o -name '*.h++' -o -name '*.H' -o \
  -name '*.c' -o -name '*.cc' -o -name '*.cxx' -o -name '*.c++' -o -name '*.C' \
\) | sort)

if [ -n "$INCORRECT_EXTENSIONS" ]; then
  echo "ERROR: Found files with incorrect extensions:"
  echo "$INCORRECT_EXTENSIONS"
  echo ""
  echo "According to project standards:"
  echo " - Header files should use .hpp (not .h, .hh, .hxx, .h++, or .H)"
  echo " - Implementation files should use .cpp (not .c, .cc, .cxx, .c++, or .C)"
  exit 1
fi
echo "✓ No files with incorrect extensions found."
exit 0
