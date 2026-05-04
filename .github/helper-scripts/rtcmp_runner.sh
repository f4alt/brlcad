#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
    printf 'Usage: %s COMPARE_PACKAGE RELEASE_PACKAGE_URL\n' "$(basename "$0")"
}

log() { printf '%s\n' "$*"; }
log_section() { printf '\n===> %s\n' "$*"; }
log_error() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

###
# Helpers
###
require_cmd() {
    command -v "$1" >/dev/null 2>&1 || log_error "Required command not found: $1"
}

abs_path() {
    local p="$1"

    if [ -d "$p" ]; then
        (cd "$p" && pwd)
    else
        local d
        local b
        d="$(dirname "$p")"
        b="$(basename "$p")"
        printf '%s/%s\n' "$(cd "$d" && pwd)" "$b"
    fi
}

extract_package() {
    local package="$1"
    local dest="$2"

    [ -f "$package" ] || log_error "Package does not exist: $package"

    mkdir -p "$dest"

    case "$package" in
        *.tar.gz|*.tgz)
            tar -xf "$package" --checkpoint=1000 --checkpoint-action=dot -C "$dest"
            ;;
        *)
            log_error "Unsupported package type: $package"
            ;;
    esac
}

find_brlcad_prefix() {
    local root="$1"
    local rt

    rt="$(find "$root" -type f -path '*/bin/rt' | sort | head -n 1 || true)"
    [ -n "$rt" ] || log_error "Unable to locate rt executable under: $root"

    dirname "$(dirname "$rt")"
}

check_args() {
    if [ "$#" -ne 2 ]; then
        usage
        exit 1
    fi

    COMPARE_PACKAGE="$(abs_path "$1")"
    RELEASE_URL="$2"

    [ -f "$COMPARE_PACKAGE" ] || log_error "Compare package does not exist: $COMPARE_PACKAGE"
}

check_requirements() {
    log_section "Checking required tools"

    require_cmd cmake
    require_cmd git
    require_cmd find
    require_cmd tar
    require_cmd curl
    require_cmd ninja
}

setup_paths() {
    log_section "Setting up paths"

    WORK_DIR="${RTCMP_WORK_DIR:-$PWD/rtcmp-work}"
    RESULTS_DIR="${RTCMP_RESULTS_DIR:-$PWD/rtcmp-results}"
    RTCMP_REF="${RTCMP_REF:-main}"

    WORK_DIR="$(abs_path "$WORK_DIR")"
    RESULTS_DIR="$(abs_path "$RESULTS_DIR")"

    COMPARE_EXTRACT_DIR="$WORK_DIR/brlcad-compare"
    RELEASE_EXTRACT_DIR="$WORK_DIR/brlcad-release"

    RELEASE_PACKAGE_DIR="$WORK_DIR/release-package"
    RELEASE_PACKAGE="$RELEASE_PACKAGE_DIR/$(basename "$RELEASE_URL")"

    RTCMP_SRC_DIR="$WORK_DIR/rtcmp"
    RTCMP_COMPARE_BUILD="$WORK_DIR/rtcmp-build-compare"
    RTCMP_RELEASE_BUILD="$WORK_DIR/rtcmp-build-release"
}

prepare_dirs() {
    log_section "Preparing work directories"

    rm -rf "$WORK_DIR"

    mkdir -p \
        "$WORK_DIR" \
        "$RESULTS_DIR" \
        "$RELEASE_PACKAGE_DIR"
}

download_release() {
    log_section "Resolving release package"

    if [[ "$RELEASE_URL" == http://* || "$RELEASE_URL" == https://* ]]; then
        log "Downloading release package: $RELEASE_URL"

        curl -L --fail --retry 3 --output "$RELEASE_PACKAGE" "$RELEASE_URL"

        [ -f "$RELEASE_PACKAGE" ] || log_error "Release package download failed: $RELEASE_URL"
    else
        log "Using local release package: $RELEASE_URL"

        RELEASE_PACKAGE="$(abs_path "$RELEASE_URL")"

        [ -f "$RELEASE_PACKAGE" ] || log_error "Release package does not exist: $RELEASE_PACKAGE"
    fi
}
download_release() {
    log_section "Downloading release package"

    # see if we have a url or a local copy
    if [[ "$RELEASE_URL" == http* ]]; then
    	curl -L --fail --retry 3 --output "$RELEASE_PACKAGE" "$RELEASE_URL"
    else
	RELEASE_PACKAGE="$(abs_path "$RELEASE_URL")"
    fi

    [ -f "$RELEASE_PACKAGE" ] || log_error "Release package download failed: $RELEASE_URL"
}

extract_brlcad_packages() {
    log_section "Extracting compare package"

    extract_package "$COMPARE_PACKAGE" "$COMPARE_EXTRACT_DIR"
    COMPARE_PREFIX="$(find_brlcad_prefix "$COMPARE_EXTRACT_DIR")"

    log "Compare BRL-CAD prefix: $COMPARE_PREFIX"

    log_section "Extracting release package"

    extract_package "$RELEASE_PACKAGE" "$RELEASE_EXTRACT_DIR"
    RELEASE_PREFIX="$(find_brlcad_prefix "$RELEASE_EXTRACT_DIR")"

    log "Release BRL-CAD prefix: $RELEASE_PREFIX"
}

verify_brlcad_installs() {
    log_section "Verifying BRL-CAD executables"

    test -x "$COMPARE_PREFIX/bin/rt" || log_error "Compare rt is not executable: $COMPARE_PREFIX/bin/rt"
    test -x "$RELEASE_PREFIX/bin/rt" || log_error "Release rt is not executable: $RELEASE_PREFIX/bin/rt"
    test -x "$COMPARE_PREFIX/bin/mged" || log_error "Compare mged is not executable: $COMPARE_PREFIX/bin/mged"

    "$COMPARE_PREFIX/bin/rt" -v || true
    "$RELEASE_PREFIX/bin/rt" -v || true
}

checkout_rtcmp() {
    log_section "Checking out rtcmp ref: $RTCMP_REF"

    git clone --depth 1 --branch "$RTCMP_REF" \
        https://github.com/BRL-CAD/rtcmp.git \
        "$RTCMP_SRC_DIR"
}

build_rtcmp_compare() {
    log_section "Building rtcmp against compare BRL-CAD"

    cmake -S "$RTCMP_SRC_DIR" -B "$RTCMP_COMPARE_BUILD" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBRLCAD_ROOT="$COMPARE_PREFIX"

    cmake --build "$RTCMP_COMPARE_BUILD" --config Release --target rtcmp

    COMPARE_RTCMP="$RTCMP_COMPARE_BUILD/rtcmp"
    test -x "$COMPARE_RTCMP" || log_error "Compare-linked rtcmp was not built: $COMPARE_RTCMP"
}

build_rtcmp_release() {
    log_section "Building rtcmp against release BRL-CAD"

    cmake -S "$RTCMP_SRC_DIR" -B "$RTCMP_RELEASE_BUILD" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBRLCAD_ROOT="$RELEASE_PREFIX"

    cmake --build "$RTCMP_RELEASE_BUILD" --config Release --target rtcmp

    RELEASE_RTCMP="$RTCMP_RELEASE_BUILD/rtcmp"
    test -x "$RELEASE_RTCMP" || log_error "Release-linked rtcmp was not built: $RELEASE_RTCMP"
}

log_test_env() {
    log "rtcmp test configuration:"
    log "  CMD1         : $COMPARE_RTCMP"
    log "  CMD2         : $RELEASE_RTCMP"
    log "  MGED         : $COMPARE_PREFIX/bin/mged"
    log "  OUTDIR       : $RESULTS_DIR"
    log "  MODEL_DIRS   : ${MODEL_DIRS:-<tests.sh default>}"
    log "  PERF_SECONDS : ${PERF_SECONDS:-<tests.sh default>}"
    log "  RAYS_PER_VIEW: ${RAYS_PER_VIEW:-<tests.sh default>}"
}

run_comparison() {
    log_section "Running rtcmp comparison"

    local test_script="$RTCMP_SRC_DIR/tests.sh"

    test -f "$test_script" || log_error "rtcmp test script not found: $test_script"
    chmod +x "$test_script"
    test -x "$test_script" || log_error "rtcmp test script is not executable: $test_script"

    test -x "$COMPARE_RTCMP" || log_error "Compare rtcmp is not executable: $COMPARE_RTCMP"
    test -x "$RELEASE_RTCMP" || log_error "Release rtcmp is not executable: $RELEASE_RTCMP"
    test -x "$COMPARE_PREFIX/bin/mged" || log_error "Compare MGED is not executable: $COMPARE_PREFIX/bin/mged"

    # if we don't have any model dirs requested, default to the release share/db/chess set for now
    MODEL_DIRS="${MODEL_DIRS:-$RELEASE_PREFIX/share/db/chess}"
    export MODEL_DIRS

    log_test_env

    CMD1="$RELEASE_RTCMP" \
    CMD2="$COMPARE_RTCMP" \
    MGED="$COMPARE_PREFIX/bin/mged" \
    OUTDIR="$RESULTS_DIR" \
    "$test_script"

    log_section "rtcmp comparison complete"
    log "Results directory: $RESULTS_DIR"
}

main() {
    check_args "$@"
    check_requirements
    setup_paths
    prepare_dirs

    download_release
    extract_brlcad_packages
    verify_brlcad_installs

    checkout_rtcmp
    build_rtcmp_compare
    build_rtcmp_release

    run_comparison
}

main "$@"
