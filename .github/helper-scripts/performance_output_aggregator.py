#!/usr/bin/env python3
"""Aggregate BRL-CAD performance lane outputs into Markdown and JSON.

Expected environment:
  RESULTS_ROOT              Root output directory. Defaults to ./perf-results.
  PERF_LANES                Space-separated lane names, e.g. "rtcmp_generic benchmark".

For each lane listed in PERF_LANES, the script reads:
  <LANE>_DESCRIPTION        Human-readable lane description.
  <LANE>_STATUS             PASS | FAIL | SKIP, or legacy numeric status.
  <LANE>_SUMMARY            Path to the lane summary CSV.
"""

from __future__ import annotations

import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class LaneResult:
    name: str
    description: str
    status: str
    status_raw: str
    summary_csv: str
    summary_csv_raw: str
    summary: list[list[str]]


# trim root from full path for just relative portion
def relative_path(path: str, root: Path) -> str:
    if not path:
        return ""

    raw = Path(path)
    absolute = raw if raw.is_absolute() else Path.cwd() / raw

    try:
        return str(absolute.resolve(strict=False).relative_to(root))
    except ValueError:
        return path


def read_csv_rows(path: str) -> list[list[str]]:
    if not path or not Path(path).is_file():
        return []

    with Path(path).open("r", encoding="utf-8", newline="") as csv_file:
        filtered = (
            line for line in csv_file
            if line.strip() and not line.lstrip().startswith("#")
        )
        return list(csv.reader(filtered))


# enforce a PASS means status + summary file exists
def normalize_status(status_raw: str, summary_csv: str) -> str:
    status = status_raw.strip().upper()
    summary_exists = bool(summary_csv) and Path(summary_csv).is_file()

    if status == "SKIP":
        return "SKIP"

    if not summary_exists:
        return "FAIL"

    return status


def collect_lanes(lanes: Iterable[str], root: Path) -> list[LaneResult]:
    results: list[LaneResult] = []

    for lane in lanes:
        upper = lane.replace("-", "_").upper()  # dash to under + caps
        description = os.environ.get(f"{upper}_DESCRIPTION", lane)
        status_raw = os.environ.get(f"{upper}_STATUS", "")
        summary_csv_raw = os.environ.get(f"{upper}_SUMMARY", "")
        summary_csv = relative_path(summary_csv_raw, root)
        status = normalize_status(status_raw, summary_csv_raw)
        summary = [] if status == "SKIP" else read_csv_rows(summary_csv_raw)

        results.append(
            LaneResult(
                name=lane,
                description=description,
                status=status,
                status_raw=status_raw,
                summary_csv=summary_csv,
                summary_csv_raw=summary_csv_raw,
                summary=summary,
            )
        )

    return results


def md_escape(value: object) -> str:
    return str(value).replace("|", "\\|").replace("\n", "<br>")


def csv_rows_to_md(rows: list[list[str]]) -> str:
    if not rows:
        return ""

    width = max(len(row) for row in rows)
    padded = [row + [""] * (width - len(row)) for row in rows]

    lines = [
        "| " + " | ".join(md_escape(cell) for cell in padded[0]) + " |",
        "| " + " | ".join("---" for _ in range(width)) + " |",
    ]

    for row in padded[1:]:
        lines.append("| " + " | ".join(md_escape(cell) for cell in row) + " |")

    return "\n".join(lines)


def write_summary_md(lanes: list[LaneResult], out_path: Path) -> None:
    lines = ["# BRL-CAD Performance Tracking", ""]

    for lane in lanes:
        lines.extend([
            f"## {lane.name} [ {lane.status} ]",
            lane.description,
            "",
        ])

        if lane.status == "SKIP":
            lines.extend(["_Skipped._", ""])
            continue

        if lane.summary_csv:
            lines.extend([f"`{lane.summary_csv}`", ""])

        if lane.summary:
            lines.extend([csv_rows_to_md(lane.summary), ""])
        else:
            missing = lane.summary_csv_raw or "<unset>"
            lines.extend([f"_Missing or empty summary file: `{missing}`_", ""])

    out_path.write_text("\n".join(lines), encoding="utf-8")


def write_run_json(lanes: list[LaneResult], out_path: Path) -> None:
    payload = {
        "lanes": [
            {
                "lane": lane.name,
                "description": lane.description,
                "status": lane.status,
                "summary_csv": lane.summary_csv,
                "summary": lane.summary,
            }
            for lane in lanes
        ]
    }

    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    perf_lanes = os.environ.get("PERF_LANES", "").split()
    if not perf_lanes:
        print("PERF_LANES is empty. Set it to a space-separated list of lane names.")
        return 1

    root = Path(os.environ.get("RESULTS_ROOT", str(Path.cwd() / "perf-results"))).resolve()
    out_dir = root / "summary"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_md = out_dir / "summary.md"
    run_json = out_dir / "run.json"

    print("Aggregating performance tracker results")

    lanes = collect_lanes(perf_lanes, root)
    write_summary_md(lanes, summary_md)
    write_run_json(lanes, run_json)

    print(f"Wrote {summary_md}")
    print(f"Wrote {run_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
