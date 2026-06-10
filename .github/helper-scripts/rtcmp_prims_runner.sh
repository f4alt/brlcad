#!/usr/bin/env bash

set -Eeuo pipefail

log() { printf '%s\n' "$*" >&2; }
die() { log "ERROR: $*"; exit 1; }

need_exec() {
    local p="$1"
    [[ -x "$p" ]] || die "Missing or not executable: $p"
}

need_file() {
    local p="$1"
    [[ -f "$p" ]] || die "Missing file: $p"
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

safe_tag() {
    echo "$1" | sed -e 's/[\/\\:*?"<>| ]\+/_/g'
}

parse_perf_output() {
    local file="$1"

    tr -d '\r' <"$file" |
        awk '/^[[:space:]]*Rays\/sec \[wall\][[:space:]]*\(/ {print $NF; exit}'
}

# Read one number per line on stdin; print "median cv_percent"
# (cv = population stddev / mean * 100).
compute_stats() {
    awk '
      { v[n++] = $1; sum += $1 }
      END {
        if (n == 0) { print "0 0"; exit }
        for (i = 0; i < n; i++)
          for (j = i + 1; j < n; j++)
            if (v[j] < v[i]) { t = v[i]; v[i] = v[j]; v[j] = t }
        med  = (n % 2 == 1) ? v[(n - 1) / 2] : (v[n/2 - 1] + v[n/2]) / 2
        mean = sum / n
        ss = 0; for (i = 0; i < n; i++) { d = v[i] - mean; ss += d * d }
        sd = sqrt(ss / n)
        cv = (mean > 0) ? (sd / mean * 100) : 0
        printf "%.2f %.2f\n", med, cv
      }'
}

check_prereqs() {
    : "${RTCMP_PRIMS_G:?RTCMP_PRIMS_G must be set}"
    : "${MGED:?MGED must be set}"
    : "${RTCMP:?RTCMP must be set}"
    : "${RTCMP_BASELINE:?RTCMP_BASELINE must be set}"

    RTCMP_PRIMS_G="$(abs_path "$RTCMP_PRIMS_G")"
    MGED="$(abs_path "$MGED")"
    RTCMP="$(abs_path "$RTCMP")"
    RTCMP_BASELINE="$(abs_path "$RTCMP_BASELINE")"
    OUTDIR="${OUTDIR:-rtcmp_prims_out}"
    OUTDIR="$(abs_path "$OUTDIR")"

    need_file "$RTCMP_PRIMS_G"
    need_exec "$MGED"
    need_exec "$RTCMP"
    need_exec "$RTCMP_BASELINE"

    PERF_SECONDS="${PERF_SECONDS:-3}"
    # Discarded warmup runs + measured samples per primitive (median + cv%).
    WARMUP_RUNS="${WARMUP_RUNS:-1}"
    SAMPLES="${SAMPLES:-3}"
    # Thread pin + per-prim slowdown gate threshold (compare vs baseline).
    NUM_CPUS="${NUM_CPUS:-1}"
    PERF_FAIL_THRESHOLD_PCT="${PERF_FAIL_THRESHOLD_PCT:-50}"

    mkdir -p "$OUTDIR"
}

discover_prims() {
    # Exclude regions and combinations: primitives only.
    "$MGED" -c "$RTCMP_PRIMS_G" 'search -not ( -type r -or -type c )' 2>&1 |
        tr -d '\r' |
        awk 'NF && $0 !~ /^mged>/ && $0 !~ /^BRL-CAD/'
}

# Run one rtcmp binary against one prim: WARMUP_RUNS discarded + SAMPLES measured.
# Echoes "<median_rays_per_sec> <cv_percent>" on success; returns non-zero on failure.
time_rtcmp() {
    local bin="$1"
    local prim="$2"
    local label="$3"   # "baseline" or "compare" -- for log/artifact names
    local tag="$4"
    local logfile="$OUTDIR/${tag}.${label}.perf.txt"
    local rc=0 i rps
    local -a samples=()

    # Warmup runs are discarded to shed cold-start effects; the remaining runs
    # are measured and reduced to a median (+ cv%) for a stable per-prim number.
    for (( i = 1; i <= WARMUP_RUNS + SAMPLES; i++ )); do
        set +e
        (
            cd "$OUTDIR" || exit 99
            "$bin" -p -n "$NUM_CPUS" --perf-seconds "$PERF_SECONDS" "$RTCMP_PRIMS_G" "$prim" >"$logfile" 2>&1
        )
        rc=$?
        set -e

        if [[ "$rc" -ne 0 ]]; then
            log "    ERROR: $prim ($label) failed with exit code $rc on run $i; see $logfile"
            if [[ -f "$OUTDIR/bomb.log" ]]; then
                mv "$OUTDIR/bomb.log" "$OUTDIR/${tag}.${label}.bomb.log"
                log "    ERROR: bomb log saved to $OUTDIR/${tag}.${label}.bomb.log"
            fi
            return "$rc"
        fi

        # Skip warmup iterations; only measured runs contribute to the stats.
        [[ "$i" -le "$WARMUP_RUNS" ]] && continue

        rps="$(parse_perf_output "$logfile")"
        if [[ -z "$rps" ]]; then
            log "    ERROR: unable to parse rays/sec for $prim ($label, run $i); see $logfile"
            return 2
        fi
        samples+=("$rps")
    done

    printf '%s\n' "${samples[@]}" | compute_stats
}

# Time a prim against baseline + compare and emit one CSV row:
# "prim,baseline_rays_per_sec,compare_rays_per_sec,delta_percent,cv_percent,status"
# where status is PASS | REGRESS | FAIL
# Returns 1 on a run failure, 2 on a slowdown regression past the threshold.
run_prim() {
    local prim="$1"
    local tag="$2"
    local b c bmed cmed ccv delta status

    b="$(time_rtcmp "$RTCMP_BASELINE" "$prim" baseline "$tag")" || { echo "$prim,-1,-1,-1,-1,FAIL"; return 1; }
    c="$(time_rtcmp "$RTCMP"          "$prim" compare  "$tag")" || { echo "$prim,-1,-1,-1,-1,FAIL"; return 1; }

    bmed="${b%% *}"; cmed="${c%% *}"; ccv="${c#* }"
    delta="$(awk -v b="$bmed" -v c="$cmed" 'BEGIN { printf "%.2f", (b > 0) ? (c - b) / b * 100 : 0 }')"

    # Regression iff compare is more than THRESHOLD% slower than baseline
    # (rays/sec is higher-is-better, so a regression is a negative delta).
    if awk -v d="$delta" -v t="$PERF_FAIL_THRESHOLD_PCT" 'BEGIN { exit !(d < -t) }'; then
        status="REGRESS"
    else
        status="PASS"
    fi

    echo "$prim,$bmed,$cmed,$delta,$ccv,$status"

    [[ "$status" == "REGRESS" ]] && return 2
    return 0
}

main() {
    check_prereqs

    local raw_csv="$OUTDIR/prims_raw.csv"
    local sorted_csv="$OUTDIR/prims_summary.csv"

    log "Discovering primitives from: $RTCMP_PRIMS_G"

    # capture in a command substitution so a failing mged is actually detected (and doesnt swallow errors)
    local prims_raw=""
    prims_raw="$(discover_prims)" || die "mged failed enumerating primitives from $RTCMP_PRIMS_G"
    [[ -n "${prims_raw//[[:space:]]/}" ]] || die "No primitives found in $RTCMP_PRIMS_G"

    local -a prims
    mapfile -t prims <<< "$prims_raw"
    [[ "${#prims[@]}" -gt 0 ]] || die "No primitives found in $RTCMP_PRIMS_G"

    log "Found ${#prims[@]} primitives"
    log "Sampling: ${WARMUP_RUNS} warmup + ${SAMPLES} measured @ ${PERF_SECONDS}s, -n ${NUM_CPUS}, per prim x {baseline,compare} (median + cv%)"
    log "Regression gate: fail if any prim is >${PERF_FAIL_THRESHOLD_PCT}% slower than baseline"
    log "Running primitive performance tests..."

    echo "prim,baseline_rays_per_sec,compare_rays_per_sec,delta_percent,cv_percent,status" >"$raw_csv"

    local prim tag rc
    local total="${#prims[@]}"
    local idx=0
    local failures=0
    local regressions=0
    for prim in "${prims[@]}"; do
        [[ -n "${prim//[[:space:]]/}" ]] || continue
        idx=$((idx + 1))

        tag="$(safe_tag "$prim")"
        log "  [$idx/$total] $prim"

        rc=0
        run_prim "$prim" "$tag" >>"$raw_csv" || rc=$?
        case "$rc" in
            0) ;;
            2) regressions=$((regressions + 1))
               log "    REGRESSION: $prim >${PERF_FAIL_THRESHOLD_PCT}% slower than baseline" ;;
            *) failures=$((failures + 1))
               log "    FAILURE: $prim (rc=$rc)" ;;
        esac
    done

    {
        echo "prim,baseline_rays_per_sec,compare_rays_per_sec,delta_percent,cv_percent,status"
        tail -n +2 "$raw_csv" |
            awk -F',' 'NF >= 6 { print $0 }' |
            sort -t',' -k3,3nr
    } >"$sorted_csv"

    log "Done."
    log "Raw CSV    : $raw_csv"
    log "Sorted CSV : $sorted_csv"

    if [[ "$failures" -ne 0 || "$regressions" -ne 0 ]]; then
        log "ERROR: ${failures} run failure(s), ${regressions} regression(s) >${PERF_FAIL_THRESHOLD_PCT}% slower than baseline."
        exit 1
    fi
}

main "$@"