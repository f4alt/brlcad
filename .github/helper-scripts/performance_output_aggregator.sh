#!/usr/bin/env bash

set -Eeuo pipefail

log() { printf '%s\n' "$*"; }
log_section() { printf '\n===> %s\n' "$*"; }
log_error() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

RESULTS_ROOT="${RESULTS_ROOT:-$PWD/perf-results}"
DASHBOARD_DIR="${DASHBOARD_DIR:-$RESULTS_ROOT/dashboard}"
PERF_LANES="${PERF_LANES:-}"

mkdir -p "$DASHBOARD_DIR"

upper_name() {
    printf '%s' "$1" | tr '[:lower:]-' '[:upper:]_'
}

get_env_default() {
    local name="$1"
    local default="$2"

    printf '%s' "${!name:-$default}"
}

relative_to_results_root() {
    local path="$1"

    case "$path" in
        "$RESULTS_ROOT"/*)
            printf '%s\n' "${path#"$RESULTS_ROOT"/}"
            ;;
        *)
            printf '%s\n' "$path"
            ;;
    esac
}

lane_description() {
    local lane="$1"
    local upper

    upper="$(upper_name "$lane")"
    get_env_default "${upper}_DESCRIPTION" "$lane"
}

lane_status() {
    local lane="$1"
    local upper

    upper="$(upper_name "$lane")"
    get_env_default "${upper}_STATUS" "ERROR"
}

lane_summary_csv() {
    local lane="$1"
    local upper

    upper="$(upper_name "$lane")"
    get_env_default "${upper}_SUMMARY" ""
}

lane_status_label() {
    local status="$(lane_status "$1")"
    local summary_csv="$2"

    if [[ "$status" != "SKIP" && (-z "$summary_csv" || ! -f "$summary_csv") ]]; then
        printf 'FAIL\n'
        return
    fi

    printf "$status\n"
}

csv_to_md_table() {
    local file="$1"

    if [[ -z "$file" || ! -f "$file" ]]; then
        printf '_Missing summary file: `%s`_\n' "$file"
        return
    fi

    awk -F',' '
      function trim_quotes(s) {
        gsub(/^"/, "", s)
        gsub(/"$/, "", s)
        return s
      }

      BEGIN {
        row = 0
      }

      /^#/ { next }
      NF == 0 { next }

      {
        row++

        printf "|"
        for (i = 1; i <= NF; i++) {
          cell = trim_quotes($i)
          gsub(/\|/, "\\|", cell)
          printf " %s |", cell
        }
        printf "\n"

        if (row == 1) {
          printf "|"
          for (i = 1; i <= NF; i++) {
            printf " --- |"
          }
          printf "\n"
        }
      }
    ' "$file"
}

write_summary_md_start() {
    cat > "$DASHBOARD_DIR/summary.md" <<EOF
# BRL-CAD Performance Tracker

EOF
}

append_summary_md_lane() {
    local lane="$1"
    local description="$2"
    local status="$3"
    local summary_csv="$4"
    local summary_path="$5"

    cat >> "$DASHBOARD_DIR/summary.md" <<EOF
## $lane [ $status ]
$description

\`$summary_path\`

EOF

    csv_to_md_table "$summary_csv" >> "$DASHBOARD_DIR/summary.md"

    cat >> "$DASHBOARD_DIR/summary.md" <<EOF

EOF
}

write_run_json() {
    local lane_data="$1"

    python3 - "$lane_data" "$DASHBOARD_DIR/run.json" <<'PY'
import csv
import json
import sys
from pathlib import Path

lane_data_path = Path(sys.argv[1])
out_path = Path(sys.argv[2])

lanes = []

with lane_data_path.open("r", encoding="utf-8") as f:
    for line in f:
        line = line.rstrip("\n")
        if not line:
            continue

        lane, lane_description, status, summary_path, summary_csv = line.split("|", 4)

        rows = []
        if summary_csv and Path(summary_csv).is_file():
            with Path(summary_csv).open("r", encoding="utf-8", newline="") as csv_file:
                reader = csv.reader(row for row in csv_file if row.strip() and not row.startswith("#"))
                rows = list(reader)

        lanes.append({
            "lane": lane,
            "description": lane_description,
            "status": status,
            "summary_csv": summary_path,
            "summary": rows,
        })

out_path.parent.mkdir(parents=True, exist_ok=True)
with out_path.open("w", encoding="utf-8") as f:
    json.dump({"lanes": lanes}, f, indent=2)
    f.write("\n")
PY
}

main() {
    log_section "Aggregating performance tracker results"

    if [[ -z "${PERF_LANES//[[:space:]]/}" ]]; then
        log_error "PERF_LANES is empty. Set it to a space-separated list of lane names."
    fi

    local lane_data="$DASHBOARD_DIR/.lane-data.txt"
    : > "$lane_data"

    write_summary_md_start

    local lane
    for lane in $PERF_LANES; do
        local description status_code status summary_csv summary_path

        description="$(lane_description "$lane")"
        summary_csv="$(lane_summary_csv "$lane")"
        status="$(lane_status_label "$lane" "$summary_csv")"
        summary_path="$(relative_to_results_root "$summary_csv")"

        append_summary_md_lane "$lane" "$description" "$status" "$summary_csv" "$summary_path"

        printf '%s|%s|%s|%s|%s\n' \
            "$lane" \
            "$description" \
            "$status" \
            "$summary_path" \
            "$summary_csv" >> "$lane_data"
    done

    write_run_json "$lane_data"

    rm -f "$lane_data"

    log "Wrote $DASHBOARD_DIR/summary.md"
    log "Wrote $DASHBOARD_DIR/run.json"
}

main "$@"