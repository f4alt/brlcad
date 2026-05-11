#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
    printf 'Usage: %s BASELINE_BRLCAD COMPARE_BRLCAD\n' "$(basename "$0")"
}

###
# Helpers
###
log() { printf '%s\n' "$*" >&2; }
log_section() { printf '\n===> %s\n' "$*" >&2; }
log_error() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

abs_path() {
    local path="$1"
    local dir=""
    local base=""

    if [ -d "$path" ]; then
        (cd "$path" && pwd)
    else
        dir="$(dirname "$path")"
        base="$(basename "$path")"
        printf '%s/%s\n' "$(cd "$dir" && pwd)" "$base"
    fi
}

check_args() {
    if [ "$#" -ne 2 ]; then
        usage
        exit 1
    fi

    BRL_BASELINE_INPUT="$1"
    BRL_COMPARE_INPUT="$2"
}

check_requirements() {
    log_section "Checking required tools"

    require_cmd() { command -v "$1" >/dev/null 2>&1 || log_error "Required command not found: $1"; }
    require_cmd cmake
    require_cmd git
    require_cmd find
    require_cmd tar
    require_cmd curl
    require_cmd ninja
}

setup_dirs() {
    log_section "Setting up dirs"

    # pull from env variables, if set
    WORK_DIR="${WORK_DIR:-$PWD/rtcmp-work}"
    RESULTS_DIR="${RESULTS_DIR:-$PWD/rtcmp-results}"
    REF="${REF:-main}"

    # scrub paths
    WORK_DIR="$(abs_path "$WORK_DIR")"
    RESULTS_DIR="$(abs_path "$RESULTS_DIR")"

    # setup path organization
    DOWNLOAD_DIR="$WORK_DIR/downloads"
    BRL_COMPARE_EXTRACT_DIR="$WORK_DIR/brlcad-compare"
    BRL_BASELINE_EXTRACT_DIR="$WORK_DIR/brlcad-baseline"
    RTCMP_SRC_DIR="$WORK_DIR/rtcmp"
    RTCMP_COMPARE_BUILD="$WORK_DIR/rtcmp-build-compare"
    RTCMP_BASELINE_BUILD="$WORK_DIR/rtcmp-build-baseline"

    # ensure work starts fresh
    rm -rf "$WORK_DIR"

    # ensure paths exist
    mkdir -p \
        "$WORK_DIR" \
        "$RESULTS_DIR" \
        "$DOWNLOAD_DIR" \
        "$BRL_COMPARE_EXTRACT_DIR" \
        "$BRL_BASELINE_EXTRACT_DIR" \
        "$RTCMP_SRC_DIR" \
        "$RTCMP_COMPARE_BUILD" \
        "$RTCMP_BASELINE_BUILD"
}

extract_tarball() {
    local package="$1"
    local dest="$2"

    [ -f "$package" ] || log_error "Package does not exist: $package"

    mkdir -p "$dest"
    case "$package" in
        *.tar.gz|*.tgz)
            log "Extracting $package"
            tar -xf "$package" --checkpoint=1000 --checkpoint-action=dot -C "$dest" >&2
            ;;
        *)
            log_error "Unsupported package type: $package"
            ;;
    esac
}

verify_brlcad() {
    local label="$1"
    local root="$2"
    local prefix=""
    local rt=""

    [[ -d "$root" ]] || log_error "Directory does not exist: $root"

    # see if we can find bin dir
    if [[ -x "$root/bin/rt" ]]; then
        # at the right level
        prefix="$root"
    else
        # try to see if we're nested somewhere
        rt="$(find "$root" -type f -path '*/bin/rt' | sort | head -n 1 || true)"
        [[ -n "$rt" ]] || log_error "Unable to locate BRL-CAD bin/ under: $root"
        prefix="$(dirname "$(dirname "$rt")")"
    fi

    # lazy check that this is a built brlcad
    [[ -x "$prefix/bin/rt" ]] || log_error "$label BRL-CAD rt is not executable: $prefix/bin/rt"
    [[ -x "$prefix/bin/mged" ]] || log_error "$label BRL-CAD mged is not executable: $prefix/bin/mged"

    # return
    echo "$prefix"
}

resolve_brlcad_input() {
    local label="$1"
    local input="$2"
    local out_dir="$3"
    local package=""
    local prefix=""

    log_section "Resolving $label BRL-CAD input"

    # should have one of: url to zip | local zip | local build
    if [[ "$input" == http* ]]; then
        local url_basename="${input##*/}"
        package="$DOWNLOAD_DIR/${label}-${url_basename}"
        curl -L --fail --retry 3 --output "$package" "$input"
        extract_tarball "$package" "$out_dir"
    elif [[ -f "$input" ]]; then
        package="$(abs_path "$input")"
        extract_tarball "$package" "$out_dir"
    elif [[ -d "$input" ]]; then
        out_dir="$(abs_path "$input")"
    else
        log_error "$label input is not a URL, tarball, or directory: $input"
    fi

    prefix="$(verify_brlcad "$label" "$out_dir")"

    echo "$prefix"
}

build_rtcmp() {
    local label="$1"
    local brl_root="$2"
    local build_dir="$3"
    local out_var="$4"
    local exe=""

    log_section "Building rtcmp against $label BRL-CAD"

    cmake -S "$RTCMP_SRC_DIR" -B "$build_dir" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBRLCAD_ROOT="$brl_root"

    cmake --build "$build_dir" --config Release --target rtcmp

    exe="$build_dir/rtcmp"
    [[ -x "$exe" ]] || log_error "$label-linked rtcmp was not built: $exe"

    printf -v "$out_var" '%s' "$exe"
}

checkout_rtcmp() {
    log_section "Checking out rtcmp ref: $REF"

    git clone --depth 1 --branch "$REF" \
        https://github.com/BRL-CAD/rtcmp.git \
        "$RTCMP_SRC_DIR"
}

log_test_env() {
    log "rtcmp test configuration:"
    log "  CMD1         : $BASELINE_RTCMP"
    log "  CMD2         : $COMPARE_RTCMP"
    log "  MGED         : $BASELINE_PREFIX/bin/mged"
    log "  OUTDIR       : $RESULTS_DIR"
    log "  MODEL_DIRS   : ${MODEL_DIRS:-<tests.sh default>}"
    log "  PERF_SECONDS : ${PERF_SECONDS:-<tests.sh default>}"
    log "  RAYS_PER_VIEW: ${RAYS_PER_VIEW:-<tests.sh default>}"
}

run_comparison() {
    # note: function assumes we have a COMPARE_RTCMP and BASELINE_RTCMP vars set for exe's
    #       as well as COMPARE_PREFIX to find a path to mged
    log_section "Running rtcmp comparison"

    # make sure we have rtcmp script
    local test_script="$RTCMP_SRC_DIR/tests.sh"
    test -f "$test_script" || log_error "rtcmp test script not found: $test_script"
    chmod +x "$test_script"
    test -x "$test_script" || log_error "rtcmp test script is not executable: $test_script"

    # make sure we have executables
    test -x "$COMPARE_RTCMP" || log_error "Compare rtcmp is not executable: $COMPARE_RTCMP"
    test -x "$BASELINE_RTCMP" || log_error "Baseline rtcmp is not executable: $BASELINE_RTCMP"
    test -x "$BASELINE_PREFIX/bin/mged" || log_error "Baseline MGED is not executable: $BASELINE_PREFIX/bin/mged"

    # if we don't have any model dirs requested, default to the baseline share/db/chess set for now
    MODEL_DIRS="${MODEL_DIRS:-$BASELINE_PREFIX/share/db/chess}"
    export MODEL_DIRS

    log_test_env

    CMD1="$BASELINE_RTCMP" \
    CMD2="$COMPARE_RTCMP" \
    MGED="$BASELINE_PREFIX/bin/mged" \
    OUTDIR="$RESULTS_DIR" \
    "$test_script"

    log_section "rtcmp comparison complete"
    log "Results directory: $RESULTS_DIR"
}

main() {
    check_args "$@"
    check_requirements

    setup_dirs

    BASELINE_PREFIX="$(resolve_brlcad_input "baseline" "$BRL_BASELINE_INPUT" "$BRL_BASELINE_EXTRACT_DIR")"
    COMPARE_PREFIX="$(resolve_brlcad_input "compare" "$BRL_COMPARE_INPUT" "$BRL_COMPARE_EXTRACT_DIR")"

    checkout_rtcmp
    build_rtcmp "baseline" "$BASELINE_PREFIX" "$RTCMP_BASELINE_BUILD" BASELINE_RTCMP
    build_rtcmp "compare" "$COMPARE_PREFIX" "$RTCMP_COMPARE_BUILD" COMPARE_RTCMP

    run_comparison
}

main "$@"
