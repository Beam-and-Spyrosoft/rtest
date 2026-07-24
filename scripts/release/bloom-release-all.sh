#!/bin/bash
# Runs bloom-release sequentially for every ROS distro rtest is tracked on.
# Useful for catching up distros that lagged behind a release (see issue #122).
# Asks for confirmation before each distro so you can skip/abort individually --
# bloom-release itself pushes branches/tags and opens a rosdistro PR, which isn't
# something to run unattended.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTROS=(rolling jazzy kilted lyrical)

echo "===== BLOOM RELEASE: rtest / all distros ====="
echo "Tip: run 'pixi run -e release release-status' first to see which distros actually need it."
echo

for distro in "${DISTROS[@]}"; do
  echo
  read -r -p ">>> Release rtest for '${distro}'? [y/N] " answer
  case "${answer}" in
    [yY]*) "${SCRIPT_DIR}/bloom-release.sh" "${distro}" ;;
    *) echo "Skipping ${distro}." ;;
  esac
done
