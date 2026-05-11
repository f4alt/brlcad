#!/usr/bin/env bash

set -Eeuo pipefail

log() { printf '%s\n' "$*" >&2; }
die() { log "ERROR: $*"; exit 1; }

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
    : "${BENCHMARKS:?BENCHMARKS must be set to a space-separated list of benchmark executables}"

    OUTDIR="${OUTDIR:-benchmark_out}"
    OUTDIR="$(abs_dir_path "$OUTDIR")"
}

run_one_benchmark() {
    local benchmark_exe="$1"

    benchmark_exe="$(abs_path "$benchmark_exe")"
    need_exec "$benchmark_exe"

    local default_name
    default_name="$(basename "$(dirname "$(dirname "$benchmark_exe")")")"

    local tag
    tag="$(safe_tag "$default_name")"

    local build_outdir="$OUTDIR/$tag"
    local logfile="$build_outdir/benchmark.log"

    local build_name="$default_name"
    local parsed_name=""
    local vgr="-1"
    local rc=0

    mkdir -p "$build_outdir"

    local -a args
    mapfile -t args < <(benchmark_args)

    log "Running benchmark: $benchmark_exe ${args[*]}"

    set +e
    (
        cd "$build_outdir" || exit 99
        "$benchmark_exe" "${args[@]}" >"$logfile" 2>&1
    )
    rc=$?
    set -e

    if [[ "$rc" -ne 0 ]]; then
        log "ERROR: benchmark failed with exit code $rc; see $logfile"
        printf '"%s",%s\n' "$build_name" "$vgr"
        return "$rc"
    fi

    parsed_name="$(parse_build_name "$logfile")"
    if [[ -n "$parsed_name" ]]; then
        build_name="$parsed_name"
    else
        log "WARN: unable to parse build name from $logfile; using '$build_name'"
    fi

    vgr="$(parse_vgr "$logfile")"
    if [[ -z "$vgr" ]]; then
        log "ERROR: benchmark completed but VGR could not be parsed; see $logfile"
        printf '"%s",%s\n' "$build_name" "-1"
        return 2
    fi

    printf '"%s",%s\n' "$build_name" "$vgr"
}

main() {
    check_prereqs

    local summary_csv="$OUTDIR/summary.csv"
    echo "build,vgr" > "$summary_csv"

    local rc=0
    local failures=0
    local fail_rc=0     # note: only first failing rc is captured
    local benchmark_exe
    for benchmark_exe in $BENCHMARKS; do
        [[ -n "${benchmark_exe//[[:space:]]/}" ]] || continue

        if run_one_benchmark "$benchmark_exe" >> "$summary_csv"; then
            rc=0
        else
            rc=$?
            failures=$((failures + 1))
            if [[ "$fail_rc" -eq 0 ]]; then
                fail_rc="$rc"
            fi
        fi
    done

    log "Summary: $summary_csv"

    if [[ "$failures" -ne 0 ]]; then
        # log instead of error so we can bubble rc
        log "ERROR: $failures benchmark runs(s) failed. First rc: $fail_rc"
        exit "$fail_rc"
    fi
}

main "$@"