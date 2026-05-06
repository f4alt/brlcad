#!/usr/bin/env bash

set -Eeuo pipefail

log() { printf '%s\n' "$*"; }
log_section() { printf '\n===> %s\n' "$*"; }
log_error() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

RESULTS_ROOT="${RESULTS_ROOT:-$PWD/perf-results}"
DASHBOARD_DIR="$RESULTS_ROOT/dashboard"

# rtcmp_generic runs
RTCMP_GENERIC_SUMMARY="${RTCMP_GENERIC_SUMMARY:-$RESULTS_ROOT/summary.csv"
RTCMP_GENERIC_STATUS="${RTCMP_GENERIC_STATUS:-0}"

mkdir -p "$DASHBOARD_DIR"

count_csv_rows() {
    local file="$1"

    [ -f "$file" ] || {
        echo "0"
        return
    }

    awk '
      BEGIN { count = 0 }
      /^#/ { next }
      NF == 0 { next }
      NR == 1 { next }
      { count++ }
      END { print count }
    ' "$file"
}

count_compare_fails() {
    local file="$1"

    [ -f "$file" ] || {
        echo "0"
        return
    }

    awk -F',' '
      BEGIN { count = 0 }
      /^#/ { next }
      NF == 0 { next }
      NR == 1 { next }
      $9 == "FAIL" { count++ }
      END { print count }
    ' "$file"
}

count_perf_fails() {
    local file="$1"

    [ -f "$file" ] || {
        echo "0"
        return
    }

    # perf_status is expected to be the last column once tests.sh is updated.
    # If the column is not present yet, this safely returns 0.
    awk -F',' '
      BEGIN { count = 0; perf_col = 0 }
      NR == 1 {
        for (i = 1; i <= NF; i++) {
          if ($i == "perf_status") perf_col = i
        }
        next
      }
      /^#/ { next }
      NF == 0 { next }
      perf_col > 0 && $perf_col == "FAIL" { count++ }
      END { print count }
    ' "$file"
}

write_summary_md() {
    local total_cases="$1"
    local compare_fails="$2"
    local perf_fails="$3"
    local overall="$4"

    cat > "$DASHBOARD_DIR/summary.md" <<EOF
# BRL-CAD Performance Tracker

## Overall status

**$overall**

## rtcmp generic

| Metric | Value |
|---|---:|
| Total cases | $total_cases |
| Comparison failures | $compare_fails |
| Performance threshold failures | $perf_fails |
| Lane exit status | $GENERIC_STATUS |

## Artifacts

- \`rtcmp-generic/summary.csv\`
- \`dashboard/run.json\`
EOF
}

write_run_json() {
    local total_cases="$1"
    local compare_fails="$2"
    local perf_fails="$3"
    local overall="$4"

    local run_time
    run_time="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

    cat > "$DASHBOARD_DIR/run.json" <<EOF
{
  "schema_version": 1,
  "run_time": "$run_time",
  "overall_status": "$overall",
  "lanes": {
    "rtcmp_generic": {
      "status_code": $GENERIC_STATUS,
      "total_cases": $total_cases,
      "compare_failures": $compare_fails,
      "perf_failures": $perf_fails,
      "summary_csv": "rtcmp-generic/summary.csv"
    }
  }
}
EOF
}

main() {
    log_section "Aggregating performance tracker results"

    total_cases="$(count_csv_rows "$RTCMP_GENERIC_SUMMARY")"
    compare_fails="$(count_compare_fails "$RTCMP_GENERIC_SUMMARY")"
    perf_fails="$(count_perf_fails "$RTCMP_GENERIC_SUMMARY")"

    overall="PASS"
    if [ "$GENERIC_STATUS" -ne 0 ] || [ "$compare_fails" -ne 0 ] || [ "$perf_fails" -ne 0 ]; then
        overall="FAIL"
    fi

    write_summary_md "$total_cases" "$compare_fails" "$perf_fails" "$overall"
    write_run_json "$total_cases" "$compare_fails" "$perf_fails" "$overall"

    log "Wrote $DASHBOARD_DIR/summary.md"
    log "Wrote $DASHBOARD_DIR/run.json"
}

main "$@"