#!/usr/bin/env python3
"""Merge per-package colcon compile_commands.json files into one database.

Colcon writes build/<base>/<package>/compile_commands.json when
CMAKE_EXPORT_COMPILE_COMMANDS is ON. Tools like clang-tidy/clangd expect a
single compile_commands.json (typically at the workspace root or build base).

Usage:
    merge_compile_commands.py <build-base>
    e.g. merge_compile_commands.py build/kilted-clang
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def merge_compile_commands(build_base: Path) -> Path:
    if not build_base.is_dir():
        raise SystemExit(f"error: build base does not exist: {build_base}")

    sources = sorted(build_base.glob("*/compile_commands.json"))
    if not sources:
        entries = sorted(p.name for p in build_base.iterdir())
        raise SystemExit(
            f"error: no per-package compile_commands.json under {build_base}\n"
            f"entries: {', '.join(entries) if entries else '(empty)'}"
        )

    merged: list[object] = []
    for path in sources:
        with path.open() as f:
            entries = json.load(f)
        if not isinstance(entries, list):
            raise SystemExit(f"error: expected a JSON array in {path}")
        merged.extend(entries)

    out = build_base / "compile_commands.json"
    out.write_text(json.dumps(merged, indent=2) + "\n")
    print(
        f"Merged {len(merged)} compile commands from {len(sources)} packages -> {out}"
    )

    link = Path.cwd() / "compile_commands.json"
    target = os.path.relpath(out, Path.cwd())
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(target)
    print(f"Linked {link} -> {target}")
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Merge per-package colcon compile_commands.json files."
    )
    parser.add_argument(
        "build_base",
        type=Path,
        help="Colcon build base directory (e.g. build/kilted-clang)",
    )
    args = parser.parse_args(argv)
    merge_compile_commands(args.build_base)
    return 0


if __name__ == "__main__":
    sys.exit(main())
