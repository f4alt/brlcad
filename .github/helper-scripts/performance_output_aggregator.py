#!/usr/bin/env python3
"""Aggregate BRL-CAD performance lane outputs into Markdown and JSON.

Expected environment:
  RESULTS_ROOT              Root output directory. Defaults to ./perf-results.
  PERF_LANES                Space-separated lane names, e.g. "rtcmp_generic benchmark".

For each lane listed in PERF_LANES, the script reads:
  <LANE>_DESCRIPTION        Human-readable lane description.
  <LANE>_STATUS             PASS | FAIL | SKIP, or legacy numeric status.
  <LANE>_SUMMARY            Path to the lane summary CSV.

Outputs:
  $RESULTS_ROOT/summary/summary.md
  $RESULTS_ROOT/summary/summary.json
"""

from __future__ import annotations

import csv
import json
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


SCHEMA_VERSION = 1


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

    if status in {"PASS", "FAIL"}:
        return status

    # Legacy shell/numeric statuses: 0 is success, anything else is failure.
    if status.isdigit():
        return "PASS" if status == "0" else "FAIL"

    return status or "FAIL"


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


def normalize_key(header: str, index: int) -> str:
    key = header.strip().lower()
    key = re.sub(r"[^a-z0-9]+", "_", key)
    key = key.strip("_")
    return key or f"column_{index + 1}"


def make_unique_columns(headers: list[str]) -> list[str]:
    columns: list[str] = []
    seen: dict[str, int] = {}

    for index, header in enumerate(headers):
        base = normalize_key(header, index)
        count = seen.get(base, 0)
        seen[base] = count + 1
        columns.append(base if count == 0 else f"{base}_{count + 1}")

    return columns


def is_numeric_column(key: str) -> bool:
    if key in {
        "vgr",
        "baseline_vgr",
        "compare_vgr",
        "bots",
        "bot_faces",
        "breps",
        "brlcad_prims",
        "num_comp_rays",
        "rays_per_sec",
        "perf1_rays_per_sec_wall",
        "perf2_rays_per_sec_wall",
        "rays_per_sec_ratio",
        "perf_delta_percent",
    }:
        return True

    return (
        key.endswith("_count")
        or key.endswith("_percent")
        or key.endswith("_ratio")
        or "rays_per_sec" in key
    )


def coerce_value(lane: str, key: str, value: str) -> Any:
    value = value.strip()
    if value == "":
        return None

    # Preserve identifiers and status-like values as strings.
    if key in {"build", "file", "component", "tag", "prim", "status", "compare_status", "comp_status_tol", "perf_status"}:
        return value
    if key.endswith("_status"):
        return value

    # Failure rows are emitted with -1 sentinels by the shell lanes. Convert
    # those to null so the dashboard shows a gap rather than graphing -1 as real
    # data. Keyed by column name so it covers both prims and benchmark failures.
    if value == "-1" and key in {
        "baseline_rays_per_sec",
        "compare_rays_per_sec",
        "delta_percent",
        "cv_percent",
        "baseline_vgr",
        "compare_vgr",
    }:
        return None

    if not is_numeric_column(key):
        return value

    try:
        if re.fullmatch(r"[-+]?\d+", value):
            return int(value)
        return float(value)
    except ValueError:
        return value


def csv_rows_to_objects(lane: str, rows: list[list[str]]) -> tuple[list[str], list[dict[str, Any]]]:
    if not rows:
        return [], []

    columns = make_unique_columns(rows[0])
    objects: list[dict[str, Any]] = []

    for raw_row in rows[1:]:
        padded = raw_row + [""] * (len(columns) - len(raw_row))
        item = {
            column: coerce_value(lane, column, padded[index])
            for index, column in enumerate(columns)
        }

        objects.append(item)

    return columns, objects


def workflow_url() -> str:
    server = os.environ.get("GITHUB_SERVER_URL", "")
    repo = os.environ.get("GITHUB_REPOSITORY", "")
    run_id = os.environ.get("GITHUB_RUN_ID", "")

    if server and repo and run_id:
        return f"{server}/{repo}/actions/runs/{run_id}"

    return ""


def make_run_metadata(now: datetime) -> dict[str, Any]:
    timestamp = os.environ.get("PERF_RUN_TIMESTAMP", now.strftime("%Y-%m-%dT%H:%M:%SZ"))
    timestamp_id = timestamp.replace(":", "")

    commit = os.environ.get("GITHUB_SHA", "")
    short_commit = commit[:12]
    fallback_id = f"{timestamp_id}-{short_commit}" if short_commit else timestamp_id

    return {
        "id": os.environ.get("PERF_RUN_ID", fallback_id),
        "timestamp": timestamp,
        "repository": os.environ.get("GITHUB_REPOSITORY", ""),
        "branch": os.environ.get("GITHUB_REF_NAME", ""),
        "ref": os.environ.get("GITHUB_REF", ""),
        "commit": commit,
        "short_commit": short_commit,
        "workflow": os.environ.get("GITHUB_WORKFLOW", ""),
        "workflow_run_id": os.environ.get("GITHUB_RUN_ID", ""),
        "workflow_run_number": os.environ.get("GITHUB_RUN_NUMBER", ""),
        "workflow_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT", ""),
        "workflow_url": workflow_url(),
        "runner": {
            "os": os.environ.get("RUNNER_OS", ""),
            "arch": os.environ.get("RUNNER_ARCH", ""),
            "name": os.environ.get("RUNNER_NAME", ""),
        },
        "baseline": os.environ.get("PERF_BASELINE", ""),
        "candidate": os.environ.get("PERF_CANDIDATE", ""),
        "rtcmp_ref": os.environ.get("PERF_RTCMP_REF", ""),
        "rtcmp_sha": os.environ.get("PERF_RTCMP_SHA", ""),
    }


def overall_status(lanes: list[LaneResult]) -> str:
    statuses = [lane.status for lane in lanes]

    if any(status == "FAIL" for status in statuses):
        return "FAIL"
    if any(status == "PASS" for status in statuses):
        return "PASS"
    if statuses and all(status == "SKIP" for status in statuses):
        return "SKIP"

    return "UNKNOWN"


def build_dashboard_payload(lanes: list[LaneResult]) -> dict[str, Any]:
    now = datetime.now(timezone.utc)
    lane_payload: dict[str, Any] = {}

    for lane in lanes:
        columns, rows = csv_rows_to_objects(lane.name, lane.summary)
        lane_payload[lane.name] = {
            "description": lane.description,
            "status": lane.status,
            "status_raw": lane.status_raw,
            "summary_csv": lane.summary_csv,
            "columns": columns,
            "rows": rows,
            "row_count": len(rows),
        }

    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "run": make_run_metadata(now),
        "overall_status": overall_status(lanes),
        "lane_order": [lane.name for lane in lanes],
        "lanes": lane_payload,
    }


def write_summary_json(lanes: list[LaneResult], out_path: Path) -> None:
    payload = build_dashboard_payload(lanes)
    out_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    perf_lanes = os.environ.get("PERF_LANES", "").split()
    if not perf_lanes:
        print("PERF_LANES is empty. Set it to a space-separated list of lane names.")
        return 1

    root = Path(os.environ.get("RESULTS_ROOT", str(Path.cwd() / "perf-results"))).resolve()
    out_dir = root / "summary"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_md = out_dir / "summary.md"
    summary_json = out_dir / "summary.json"

    print("Aggregating performance tracker results")

    lanes = collect_lanes(perf_lanes, root)
    write_summary_md(lanes, summary_md)
    write_summary_json(lanes, summary_json)

    print(f"Wrote {summary_md}")
    print(f"Wrote {summary_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
