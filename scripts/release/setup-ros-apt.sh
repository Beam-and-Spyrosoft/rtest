#!/bin/bash
# Add the official ROS 2 apt repository on an Ubuntu host by installing the
# ros2-apt-source package for this Ubuntu release (no Docker needed).
# Used by .github/workflows/release.yml. Needs sudo and the GitHub CLI (`gh`,
# authenticated through GH_TOKEN in CI) to look up the latest ros-apt-source release.
set -euo pipefail

# shellcheck source=/dev/null
CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
VERSION="$(gh api repos/ros-infrastructure/ros-apt-source/releases/latest --jq .tag_name)"
DEB_DIR="$(mktemp -d)"
DEB="${DEB_DIR}/ros2-apt-source.deb"

echo "Installing ros2-apt-source ${VERSION} for Ubuntu ${CODENAME}"
curl -fsSL -o "${DEB}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION}/ros2-apt-source_${VERSION}.${CODENAME}_all.deb"
sudo dpkg -i "${DEB}"
rm -rf "${DEB_DIR}"
sudo apt-get update
