#!/bin/bash
# Wraps `bloom-release` for the rtest package on a single ROS distro.
#
# rtest is already registered in rosdistro for every distro below, so bloom finds the
# release repo (ros2-gbp/rtest-release) and track automatically -- no --url needed.
# bloom-release clones it into its own temp directory regardless of cwd, so there's
# no workspace to manage here.
#
# This is interactive on purpose: bloom-release asks for confirmation before pushing
# branches/tags and before opening the rosdistro pull request. Pass extra bloom-release
# flags after the distro argument, e.g. `--pretend` for a dry run.
set -euo pipefail

usage() {
  echo "usage: $(basename "$0") <rolling|jazzy|kilted|lyrical> [extra bloom-release args...]" >&2
  exit 1
}

[ $# -ge 1 ] || usage
DISTRO="$1"
shift

case "${DISTRO}" in
  rolling | jazzy | kilted | lyrical) ;;
  *)
    echo "error: unknown distro '${DISTRO}' (rtest is tracked on rolling, jazzy, kilted, lyrical)" >&2
    exit 1
    ;;
esac

# bloom clones the release repo into a fresh /tmp directory every run, which has no git
# identity of its own. Carry this repo's local git identity (user.name/user.email) over
# via env vars -- git honors these regardless of cwd -- instead of requiring a global
# ~/.gitconfig, which some setups (like per-repo identities via includeIf) deliberately leave empty.
: "${GIT_AUTHOR_NAME:=$(git config user.name || true)}"
: "${GIT_AUTHOR_EMAIL:=$(git config user.email || true)}"
: "${GIT_COMMITTER_NAME:=${GIT_AUTHOR_NAME}}"
: "${GIT_COMMITTER_EMAIL:=${GIT_AUTHOR_EMAIL}}"
export GIT_AUTHOR_NAME GIT_AUTHOR_EMAIL GIT_COMMITTER_NAME GIT_COMMITTER_EMAIL

if [ -z "${GIT_AUTHOR_NAME}" ] || [ -z "${GIT_AUTHOR_EMAIL}" ]; then
  echo "error: no git identity found (checked GIT_AUTHOR_NAME/EMAIL and 'git config user.name/user.email' in $(pwd))." >&2
  echo "Set it for this repo with: git config user.name \"Your Name\" && git config user.email you@example.com" >&2
  exit 1
fi

# The release repo is registered in rosdistro over https, which needs a git credential
# helper bloom doesn't set up for you. Push over ssh instead -- same as this repo's own
# origin remote -- so releasing doesn't depend on an https credential helper being
# configured. Override with BLOOM_RELEASE_PUSH_URL, or an explicit
# --override-release-repository-push-url passed after `--` (it wins: later flags override
# earlier ones with the same name).
RELEASE_REPO_PUSH_URL="${BLOOM_RELEASE_PUSH_URL:-git@github.com:ros2-gbp/rtest-release.git}"

# bloom commits its generated packaging/tracks-config bookkeeping straight to the release
# repo (not something a human reviews or signs upstream), so a global `commit.gpgsign =
# true` breaks it: there's no secret key for a repo bloom itself owns. Scope the override to
# this process only, via git's env-based config mechanism -- it does not touch ~/.gitconfig,
# so signing still applies everywhere else (including real commits to this rtest repo).
export GIT_CONFIG_COUNT=2
export GIT_CONFIG_KEY_0=commit.gpgsign GIT_CONFIG_VALUE_0=false
export GIT_CONFIG_KEY_1=tag.gpgsign GIT_CONFIG_VALUE_1=false

echo "===== BLOOM RELEASE: rtest / ${DISTRO} ====="
echo "git identity: ${GIT_AUTHOR_NAME} <${GIT_AUTHOR_EMAIL}>"
echo "push url:     ${RELEASE_REPO_PUSH_URL}"
echo "command: bloom-release rtest --rosdistro ${DISTRO} --track ${DISTRO} --override-release-repository-push-url ${RELEASE_REPO_PUSH_URL} $*"
echo

bloom-release rtest --rosdistro "${DISTRO}" --track "${DISTRO}" \
  --override-release-repository-push-url "${RELEASE_REPO_PUSH_URL}" "$@"
