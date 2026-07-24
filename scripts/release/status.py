#!/usr/bin/env python3
"""Compare rtest's bloom-released version on each ROS distro against the local
package.xml version, so it's obvious which distro tracks are lagging (see issue #122,
where the rolling/jazzy/kilted tracks weren't re-released alongside lyrical)."""

import re
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_XML = REPO_ROOT / "rtest" / "package.xml"
PACKAGE_NAME = "rtest"
DISTROS = ["rolling", "jazzy", "kilted", "lyrical"]


def local_version() -> str:
    match = re.search(r"<version>([^<]+)</version>", PACKAGE_XML.read_text())
    if not match:
        raise SystemExit(f"error: could not find <version> in {PACKAGE_XML}")
    return match.group(1)


def released_version(distro: str) -> str | None:
    result = subprocess.run(
        [
            "gh",
            "api",
            f"repos/ros/rosdistro/contents/{distro}/distribution.yaml",
            "-H",
            "Accept: application/vnd.github.raw",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        print(f"warning: failed to fetch {distro}/distribution.yaml: {result.stderr.strip()}", file=sys.stderr)
        return None

    data = yaml.safe_load(result.stdout)
    repo = data.get("repositories", {}).get(PACKAGE_NAME)
    if not repo or "release" not in repo:
        return None
    # rosdistro release versions look like "0.2.3-1" (upstream version + debian increment).
    return repo["release"]["version"].rsplit("-", 1)[0]


def main() -> int:
    if shutil.which("gh") is None:
        print("error: the 'gh' CLI is required (and must be authenticated) for this check", file=sys.stderr)
        return 1

    local = local_version()
    print(f"local rtest/package.xml version: {local}\n")
    print(f"{'distro':<10} {'released':<12} status")

    behind = []
    for distro in DISTROS:
        released = released_version(distro)
        if released is None:
            print(f"{distro:<10} {'(none)':<12} not released on this distro")
            continue
        up_to_date = released == local
        status = "up to date" if up_to_date else "BEHIND"
        if not up_to_date:
            behind.append(distro)
        print(f"{distro:<10} {released:<12} {status}")

    print()
    if behind:
        print(f"Run: pixi run -e release bloom-release <distro> for: {', '.join(behind)}")
    else:
        print("All tracked distros match the local package.xml version.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
