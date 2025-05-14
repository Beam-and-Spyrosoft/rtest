#!/bin/bash

# Check if required tools are installed
check_dependencies() {
  if ! command -v clang-format &> /dev/null; then
    echo "ERROR: 'clang-format' is not installed. Install it with: sudo apt-get install -y clang-format"
    exit 1
  fi
}

# Execute dependency check
check_dependencies

echo "===== CHECKING C++ CODE FORMATTING ====="

CPP_FILES=$(find . -type f \( -name "*.cpp" -o -name "*.hpp" \) | grep -v "build/" | grep -v "install/")

if [ -n "$CPP_FILES" ]; then
  FORMAT_ISSUES=0
  ISSUES_LIST=""
  TOTAL_FILES=0
  FORMATTED_FILES=0

  echo "Scanning all C++ files for formatting issues..."

  for file in $CPP_FILES; do
    TOTAL_FILES=`expr $TOTAL_FILES + 1`

    if ! clang-format -style=file --dry-run -Werror "$file" &>/dev/null; then
      FORMAT_ISSUES=`expr $FORMAT_ISSUES + 1`
      ISSUES_LIST="$ISSUES_LIST\n - $file"
      echo "❌ File needs formatting: $file"

      FORMATTED=$(clang-format -style=file "$file")
      DIFF=$(diff -u <(cat "$file") <(echo "$FORMATTED") | head -10)
      echo "$DIFF" | sed 's/^/ /'
      DIFF_LINES=$(echo "$DIFF" | wc -l)
      if [ $DIFF_LINES -gt 10 ]; then
        echo " ... (showing only first 10 lines of diff)"
      fi
      echo ""
    else
      FORMATTED_FILES=`expr $FORMATTED_FILES + 1`
    fi
  done

  echo -e "\n===== FORMATTING CHECK SUMMARY ====="
  echo "Total files scanned: $TOTAL_FILES"
  echo "Files with correct formatting: $FORMATTED_FILES"
  echo "Files needing formatting: $FORMAT_ISSUES"

  if [ $FORMAT_ISSUES -ne 0 ]; then
    echo -e "\nThe following files need formatting:$ISSUES_LIST"
    echo -e "\nRun the following command to fix all formatting issues:"
    echo "find . -name \"*.cpp\" -o -name \"*.hpp\" | grep -v \"build/\" | grep -v \"install/\" | xargs clang-format -style=file -i"
    exit 1
  else
    echo -e "\n✅ All C++ files are properly formatted."
  fi
else
  echo "No C++ files found to check."
fi