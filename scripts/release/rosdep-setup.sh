#!/bin/bash
# One-time (per machine) rosdep bootstrap required by `bloom-generate rosdebian/rosrpm`,
# which `bloom-release` runs internally to produce build-farm-ready packaging branches.
# Safe to re-run: `rosdep update` is idempotent, and `rosdep init` is skipped if already done.
set -euo pipefail

SOURCES_LIST="/etc/ros/rosdep/sources.list.d/20-default.list"

echo "===== ROSDEP SETUP ====="

if [ -f "${SOURCES_LIST}" ]; then
  echo "✅ rosdep already initialized (${SOURCES_LIST} exists), skipping init."
else
  echo "rosdep has never been initialized on this machine."
  echo "This writes ${SOURCES_LIST} and needs sudo, once, regardless of how many ROS distros you release to."
  sudo "$(command -v rosdep)" init
fi

echo "Updating rosdep's dependency-name cache..."
rosdep update

echo "✅ rosdep is ready."
