#!/bin/sh
#
# Build librmw_cerulion.so for release inside a ROS 2 Jazzy container.
#
# rmw_cerulion's build.rs is ABI-safe only when bindgen runs against the real
# distro headers, so a deployed library is built inside `ros:jazzy` (Ubuntu
# 24.04, the platform Jazzy supports) rather than on the runner host. The
# container runs natively for the runner's own architecture: the release
# workflow runs this once on an x86_64 runner and once on an arm64 runner.
#
# Usage: build_rmw_jazzy.sh REPO_ROOT OUT_DIR
#
# Writes OUT_DIR/librmw_cerulion.so, OUT_DIR/librmw_cerulion.so.evidence.txt
# (file, ldd, NEEDED, symbol counts, sha256), OUT_DIR/cargo-build.log and
# OUT_DIR/build-env.txt, and prints the library path on success.
#
# Env:
#   CERULION_RMW_TOOLCHAIN   Rust toolchain for the build (default 1.93.0, the
#                            version the release workflow pins for the binaries)
#   CERULION_RMW_IMAGE       container image (default ros:jazzy)
set -eu

usage() {
    printf 'usage: %s REPO_ROOT OUT_DIR\n' "$0" >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 2 ] || {
    usage
    exit 2
}

repo_arg=$1
out_arg=$2
script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
toolchain=${CERULION_RMW_TOOLCHAIN:-1.93.0}
image=${CERULION_RMW_IMAGE:-ros:jazzy}
container_script="$script_dir/build_rmw_jazzy_container.sh"

command -v docker >/dev/null 2>&1 ||
    die "docker is required to build the rmw library inside the Jazzy image"
[ -f "$repo_arg/crates/rmw_cerulion/Cargo.toml" ] ||
    die "REPO_ROOT does not contain crates/rmw_cerulion: $repo_arg"
[ -f "$container_script" ] ||
    die "container script missing: $container_script"
repo=$(CDPATH='' cd "$repo_arg" && pwd)
mkdir -p "$out_arg"
out=$(CDPATH='' cd "$out_arg" && pwd)

# The checkout is mounted read-only: the build must not leave root-owned files
# in the tree the host later packages, and cargo builds into a target
# directory private to the container.
docker run --rm \
    -e CERULION_RMW_TOOLCHAIN="$toolchain" \
    -v "$repo":/work:ro \
    -v "$out":/out \
    -v "$container_script":/build_rmw.sh:ro \
    "$image" \
    bash /build_rmw.sh ||
    die "the container build failed"

[ -f "$out/librmw_cerulion.so" ] ||
    die "the container did not produce $out/librmw_cerulion.so"
printf '%s\n' "$out/librmw_cerulion.so"
