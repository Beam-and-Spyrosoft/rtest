#!/usr/bin/env python3
"""Assemble the GitHub Pages coverage site from per-distro coverage reports.

Input: a directory containing one sub-directory per ROS distro, each holding the
output of generate_coverage.sh (summary.json and html/).

Output (under <site_dir>/coverage/):
  index.html     - table of distros with line/function coverage and report links
  <distro>/      - the genhtml report for that distro
  badge.json     - shields.io endpoint badge showing the lowest line coverage

Usage: coverage_site.py <reports_dir> <site_dir> [--commit SHA] [--repo OWNER/NAME]
"""

import argparse
import datetime
import html
import json
import shutil
import sys
from pathlib import Path

DISTRO_ORDER = ["jazzy", "kilted", "lyrical", "rolling"]


def badge_color(pct: float) -> str:
    if pct < 70.0:
        return "red"
    if pct < 80.0:
        return "yellow"
    return "brightgreen"


def load_reports(reports_dir: Path) -> list[dict]:
    reports = []
    for summary_file in sorted(reports_dir.glob("*/summary.json")):
        summary = json.loads(summary_file.read_text())
        summary["html_dir"] = summary_file.parent / "html"
        reports.append(summary)
    order = {name: i for i, name in enumerate(DISTRO_ORDER)}
    reports.sort(key=lambda r: order.get(r["distro"], len(order)))
    return reports


def render_index(reports: list[dict], commit: str, repo: str) -> str:
    rows = "\n".join(
        f"""      <tr>
        <td><a href="{html.escape(r["distro"])}/index.html">{html.escape(r["distro"])}</a></td>
        <td class="num">{r["lines"]:.1f}%</td>
        <td class="num">{r["functions"]:.1f}%</td>
        <td class="num">{r["branches"]:.1f}%</td>
      </tr>"""
        for r in reports
    )
    generated = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    commit_html = ""
    if commit:
        short = html.escape(commit[:7])
        if repo:
            url = f"https://github.com/{html.escape(repo)}/commit/{html.escape(commit)}"
            commit_html = f' from commit <a href="{url}"><code>{short}</code></a>'
        else:
            commit_html = f" from commit <code>{short}</code>"
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>rtest coverage</title>
  <style>
    :root {{ --bg: #ffffff; --fg: #1f2328; --muted: #59636e; --line: #d1d9e0; --link: #0969da; }}
    @media (prefers-color-scheme: dark) {{
      :root {{ --bg: #0d1117; --fg: #e6edf3; --muted: #9198a1; --line: #3d444d; --link: #4493f8; }}
    }}
    body {{ margin: 0; background: var(--bg); color: var(--fg);
           font: 16px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }}
    main {{ max-width: 40rem; margin: 0 auto; padding: 2rem 1rem; }}
    h1 {{ font-size: 1.5rem; margin: 0 0 .25rem; }}
    p {{ color: var(--muted); margin: 0 0 1.5rem; }}
    a {{ color: var(--link); }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{ padding: .5rem .75rem; border-bottom: 1px solid var(--line); text-align: left; }}
    .num {{ text-align: right; font-variant-numeric: tabular-nums; }}
  </style>
</head>
<body>
  <main>
    <h1>rtest framework coverage</h1>
    <p>Line, function and branch coverage of the rtest library on <code>main</code>{commit_html}, generated {generated}.</p>
    <table>
      <thead><tr><th>ROS distro</th><th class="num">Lines</th><th class="num">Functions</th><th class="num">Branches</th></tr></thead>
      <tbody>
{rows}
      </tbody>
    </table>
  </main>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("reports_dir", type=Path)
    parser.add_argument("site_dir", type=Path)
    parser.add_argument("--commit", default="")
    parser.add_argument("--repo", default="")
    args = parser.parse_args()

    reports = load_reports(args.reports_dir)
    if not reports:
        print(f"No */summary.json found under {args.reports_dir}", file=sys.stderr)
        return 1

    out = args.site_dir / "coverage"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    for report in reports:
        shutil.copytree(report["html_dir"], out / report["distro"])

    (out / "index.html").write_text(render_index(reports, args.commit, args.repo))

    lowest = min(r["lines"] for r in reports)
    badge = {"schemaVersion": 1, "label": "coverage", "message": f"{lowest:.1f}%", "color": badge_color(lowest)}
    (out / "badge.json").write_text(json.dumps(badge) + "\n")

    # The site root has nothing else yet; send visitors to the coverage page.
    root_index = args.site_dir / "index.html"
    if not root_index.exists():
        root_index.write_text('<!doctype html><meta charset="utf-8"><title>rtest</title>'
                              '<meta http-equiv="refresh" content="0; url=coverage/">'
                              '<a href="coverage/">rtest coverage</a>\n')

    for r in reports:
        print(
            f"{r['distro']}: {r['lines']:.1f}% lines, {r['functions']:.1f}% functions, "
            f"{r['branches']:.1f}% branches"
        )
    print(f"badge: {badge['message']} ({badge['color']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
