#!/usr/bin/env bash

set -Eeuo pipefail

log() { printf '%s\n' "$*" >&2; }
die() { log "ERROR: $*"; exit 1; }

# -E (errtrace) is set above; surface the failing line on any uncaught error.
trap 'log "benchmark_runner.sh: command failed (line $LINENO)"' ERR

need_exec() {
    local p="$1"
    [[ -x "$p" ]] || die "Missing or not executable: $p"
}

safe_tag() {
    echo "$1" | sed -e 's/[\/\\:*?"<>| ]\+/_/g'
}

abs_path() {
    local p="$1"

    if command -v realpath >/dev/null 2>&1; then
        realpath "$p"
    else
        case "$p" in
            /*) printf '%s\n' "$p" ;;
            *) printf '%s/%s\n' "$(pwd -P)" "$p" ;;
        esac
    fi
}

abs_dir_path() {
    local p="$1"
    mkdir -p "$p"
    (cd "$p" && pwd)
}

parse_build_name() {
    local logfile="$1"

    awk '
      /^BRL-CAD Release[[:space:]]/ {
        # Example:
        # BRL-CAD Release 7.42.2  The BRL-CAD Raytracer RT
        #
        # Capture only:
        # BRL-CAD Release 7.42.2
        if (match($0, /BRL-CAD Release[[:space:]]+[^[:space:]]+/)) {
          print substr($0, RSTART, RLENGTH)
          exit
        }
      }
    ' "$logfile"
}

parse_vgr() {
    local logfile="$1"

    awk '
      # Preferred final report line:
      # Benchmark results indicate an approximate VGR performance metric of 95436
      /Benchmark results indicate an approximate VGR performance metric of/ {
        for (i = NF; i >= 1; i--) {
          token = $i
          gsub(/[,;]/, "", token)
          if (token ~ /^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$/) {
            print token
            exit
          }
        }
      }
    ' "$logfile"
}

benchmark_args() {
    local -a args
    args=(run)

    if [[ -n "${BENCHMARK_TIMEFRAME:-}" ]]; then
        args+=("TIMEFRAME=$BENCHMARK_TIMEFRAME")
    fi

    if [[ -n "${BENCHMARK_MAXTIME:-}" ]]; then
        args+=("MAXTIME=$BENCHMARK_MAXTIME")
    fi

    if [[ -n "${BENCHMARK_AVERAGE:-}" ]]; then
        args+=("AVERAGE=$BENCHMARK_AVERAGE")
    fi

    printf '%s\n' "${args[@]}"
}

check_prereqs() {
    : "${BASELINE_BENCHMARK:?BASELINE_BENCHMARK must be set}"
    : "${COMPARE_BENCHMARK:?COMPARE_BENCHMARK must be set}"

    OUTDIR="${OUTDIR:-benchmark_out}"
    OUTDIR="$(abs_dir_path "$OUTDIR")"

    # Fail if compare VGR is more than this% below baseline
    PERF_FAIL_THRESHOLD_PCT="${PERF_FAIL_THRESHOLD_PCT:-20}"
}

run_one_benchmark() {
    local benchmark_exe="$1"
    local label="$2"   # "baseline" or "compare"

    benchmark_exe="$(abs_path "$benchmark_exe")"
    need_exec "$benchmark_exe"

    local tag build_outdir logfile build_name vgr rc=0
    tag="$(safe_tag "$label")"
    build_outdir="$OUTDIR/$tag"
    logfile="$build_outdir/benchmark.log"
    mkdir -p "$build_outdir"

    local -a args
    mapfile -t args < <(benchmark_args)

    log "Running $label benchmark: $benchmark_exe ${args[*]}"

    set +e
    (
        cd "$build_outdir" || exit 99
        "$benchmark_exe" "${args[@]}" >"$logfile" 2>&1
    )
    rc=$?
    set -e

    if [[ "$rc" -ne 0 ]]; then
        log "ERROR: $label benchmark failed with exit code $rc; see $logfile"
        echo "-1"
        return "$rc"
    fi

    build_name="$(parse_build_name "$logfile")"
    [[ -n "$build_name" ]] && log "  $label build: $build_name"

    vgr="$(parse_vgr "$logfile")"
    if [[ -z "$vgr" ]]; then
        log "ERROR: $label benchmark completed but VGR could not be parsed; see $logfile"
        echo "-1"
        return 2
    fi

    echo "$vgr"
}

main() {
    check_prereqs

    local summary_csv="$OUTDIR/summary.csv"
    local bvgr cvgr brc=0 crc=0

    bvgr="$(run_one_benchmark "$BASELINE_BENCHMARK" baseline)" || brc=$?
    cvgr="$(run_one_benchmark "$COMPARE_BENCHMARK"  compare)"  || crc=$?

    # atleast one benchmark failed
    if [[ "$brc" -ne 0 || "$crc" -ne 0 ]]; then
        {
            echo "metric,baseline_vgr,compare_vgr,delta_percent"
            echo "vgr,${bvgr:--1},${cvgr:--1},-1"
        } > "$summary_csv"
        log "Summary: $summary_csv"
        die "benchmark run failure (baseline rc=$brc, compare rc=$crc)"
    fi

    # got success and VGR from both benchmarks - compare
    local delta
    delta="$(awk -v b="$bvgr" -v c="$cvgr" 'BEGIN { printf "%.2f", (b > 0) ? (c - b) / b * 100 : 0 }')"

    {
        echo "metric,baseline_vgr,compare_vgr,delta_percent"
        echo "vgr,$bvgr,$cvgr,$delta"
    } > "$summary_csv"

    log "Summary: $summary_csv"
    log "VGR baseline=$bvgr compare=$cvgr delta=${delta}% (higher VGR = better)"

    # Regression iff compare VGR is more than THRESHOLD% below baseline.
    if awk -v d="$delta" -v t="$PERF_FAIL_THRESHOLD_PCT" 'BEGIN { exit !(d < -t) }'; then
        die "benchmark regression: compare VGR ${delta}% vs baseline (threshold -${PERF_FAIL_THRESHOLD_PCT}%)"
    fi
}

main "$@"