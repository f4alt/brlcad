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

    RTCMP_PRIMS_G="$(abs_path "$RTCMP_PRIMS_G")"
    MGED="$(abs_path "$MGED")"
    RTCMP="$(abs_path "$RTCMP")"
    OUTDIR="${OUTDIR:-rtcmp_prims_out}"
    OUTDIR="$(abs_path "$OUTDIR")"

    need_file "$RTCMP_PRIMS_G"
    need_exec "$MGED"
    need_exec "$RTCMP"

    PERF_SECONDS="${PERF_SECONDS:-3}"
    # Discarded warmup runs + measured samples per primitive (median + cv%).
    WARMUP_RUNS="${WARMUP_RUNS:-1}"
    SAMPLES="${SAMPLES:-3}"

    mkdir -p "$OUTDIR"
}

discover_prims() {
    # Exclude regions and combinations: primitives only.
    "$MGED" -c "$RTCMP_PRIMS_G" 'search -not ( -type r -or -type c )' 2>&1 |
        tr -d '\r' |
        awk 'NF && $0 !~ /^mged>/ && $0 !~ /^BRL-CAD/'
}

run_prim_perf() {
    local prim="$1"
    local tag="$2"
    local logfile="$OUTDIR/${tag}.perf.txt"
    local rc=0
    local i rps
    local -a samples=()

    # Warmup runs are discarded to shed cold-start effects; the remaining runs
    # are measured and reduced to a median (+ cv%) for a stable per-prim number.
    for (( i = 1; i <= WARMUP_RUNS + SAMPLES; i++ )); do
        set +e
        (
            cd "$OUTDIR" || exit 99
            "$RTCMP" -p --perf-seconds "$PERF_SECONDS" "$RTCMP_PRIMS_G" "$prim" >"$logfile" 2>&1
        )
        rc=$?
        set -e

        if [[ "$rc" -ne 0 ]]; then
            log "    ERROR: $prim failed with exit code $rc on run $i; see $logfile"

            if [[ -f "$OUTDIR/bomb.log" ]]; then
                mv "$OUTDIR/bomb.log" "$OUTDIR/${tag}.bomb.log"
                log "    ERROR: bomb log saved to $OUTDIR/${tag}.bomb.log"
            fi

            echo "$prim,-1,-1"
            return "$rc"
        fi

        # Skip warmup iterations; only measured runs contribute to the stats.
        [[ "$i" -le "$WARMUP_RUNS" ]] && continue

        rps="$(parse_perf_output "$logfile")"
        if [[ -z "$rps" ]]; then
            log "    ERROR: unable to parse rays/sec for $prim (run $i); see $logfile"
            echo "$prim,-1,-1"
            return 2
        fi
        samples+=("$rps")
    done

    local stats
    stats="$(printf '%s\n' "${samples[@]}" | compute_stats)"
    echo "$prim,${stats% *},${stats#* }"
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
    log "Sampling: ${WARMUP_RUNS} warmup + ${SAMPLES} measured @ ${PERF_SECONDS}s per prim (median + cv%)"
    log "Running primitive performance tests..."

    echo "prim,rays_per_sec,cv_percent" >"$raw_csv"

    local prim tag rc
    local total="${#prims[@]}"
    local idx=0
    local failures=0
    local fail_rc=0
    for prim in "${prims[@]}"; do
        [[ -n "${prim//[[:space:]]/}" ]] || continue
        idx=$((idx + 1))

        tag="$(safe_tag "$prim")"
        log "  [$idx/$total] $prim"

        if run_prim_perf "$prim" "$tag" >>"$raw_csv"; then
            rc=0
        else
            rc=$?
            failures=$((failures + 1))

            if [[ "$fail_rc" -eq 0 ]]; then
                fail_rc="$rc"
            fi
        fi
    done

    {
        echo "prim,rays_per_sec,cv_percent"
        tail -n +2 "$raw_csv" |
            awk -F',' 'NF >= 3 { print $0 }' |
            sort -t',' -k2,2nr
    } >"$sorted_csv"

    log "Done."
    log "Raw CSV    : $raw_csv"
    log "Sorted CSV : $sorted_csv"

    if [[ "$failures" -ne 0 ]]; then
        log "ERROR: $failures primitive performance run(s) failed. First rc: $fail_rc"
        exit "$fail_rc"
    fi
}

main "$@"