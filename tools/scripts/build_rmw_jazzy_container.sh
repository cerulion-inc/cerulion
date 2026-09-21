#!/usr/bin/env bash
# build_rmw_jazzy_container.sh: the container half of build_rmw_jazzy.sh.
#
# Runs as root INSIDE a `ros:jazzy` container with the repository mounted
# read-only at /work and an output directory at /out. It installs the build
# tools rmw_cerulion's build.rs needs (a C and C++ compiler for the std::string
# shim, clang and libclang for bindgen), the pinned Rust toolchain, sources the
# distro so build.rs bindgens against the REAL Jazzy headers, builds the
# release cdylib into a container-private target directory, and copies the
# library plus an evidence sidecar to /out.
#
# The bindgen path is asserted twice: sourcing setup.bash sets both ROS_DISTRO
# and AMENT_PREFIX_PATH, and build.rs refuses to fall back to the vendored
# bindings when a distro is named beside a header prefix; independently, the
# verbose cargo log is grepped for the build script's own
# `CERULION_RMW_BINDINGS_SOURCE=generated` directive, so a library built from
# the vendored snapshot can never reach /out.
#
# Env:
#   CERULION_RMW_TOOLCHAIN   the Rust toolchain to install (required; the
#                            workflow passes the version it pins elsewhere)
set -euo pipefail

toolchain=${CERULION_RMW_TOOLCHAIN:?CERULION_RMW_TOOLCHAIN must name the Rust toolchain}
repo=/work
out=/out
so_name=librmw_cerulion.so

die() {
    printf 'build_rmw_jazzy_container: %s\n' "$*" >&2
    exit 1
}

[ -f "$repo/crates/rmw_cerulion/Cargo.toml" ] ||
    die "repository is not mounted at $repo (crates/rmw_cerulion/Cargo.toml missing)"
[ -d "$out" ] || die "output directory $out is not mounted"

export DEBIAN_FRONTEND=noninteractive
apt-get update
# build-essential: the C++ std::string shim and the C parts of the transport
# tree; clang and libclang-dev: bindgen; binutils and file: the evidence
# sidecar. The rmw's dependency tree carries no crate that needs cmake.
apt-get install -y --no-install-recommends \
    build-essential clang libclang-dev pkg-config \
    curl ca-certificates binutils file
rm -rf /var/lib/apt/lists/*

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh
# rustfmt is NOT in the minimal profile, and rmw_cerulion's build.rs runs
# bindgen with formatting on, which shells out to `rustfmt` for the
# generated bindings: without the component the build script fails with
# "'rustfmt' is not installed for the toolchain" (seen on the first box
# proof). It is a component of the pinned toolchain, not a package.
sh /tmp/rustup-init.sh -y --profile minimal --no-modify-path \
    --default-toolchain "$toolchain" --component rustfmt
export PATH="$HOME/.cargo/bin:$PATH"
cargo --version
rustc --version

# Every tool the build script shells out to, checked by name BEFORE cargo
# runs, so a future toolchain or image change fails here with a named cause
# instead of deep inside a build-script log.
for tool in rustfmt clang g++ pkg-config; do
    command -v "$tool" >/dev/null 2>&1 ||
        die "$tool is not on PATH; rmw_cerulion's build script needs it (rustfmt for bindgen's output, clang for bindgen, g++ for the std::string shim)"
done
rustfmt --version || die "rustfmt is installed but does not run for toolchain $toolchain"
clang --version | head -n 1
g++ --version | head -n 1

# ROS setup scripts dereference unset variables; relax nounset across it.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[ "${ROS_DISTRO:-}" = jazzy ] ||
    die "expected ROS_DISTRO=jazzy after sourcing the distro, got '${ROS_DISTRO:-}'"
[ -n "${AMENT_PREFIX_PATH:-}" ] ||
    die "AMENT_PREFIX_PATH is empty after sourcing the distro; build.rs would not see the headers"
printf 'ROS_DISTRO=%s\nAMENT_PREFIX_PATH=%s\n' "$ROS_DISTRO" "$AMENT_PREFIX_PATH" \
    | tee "$out/build-env.txt"

# A container-private target directory: the checkout is mounted read-only and
# must stay byte-identical to what the host packages.
export CARGO_TARGET_DIR=/tmp/rmw-target
build_log="$out/cargo-build.log"
cargo build --release --locked -vv -p rmw_cerulion \
    --manifest-path "$repo/Cargo.toml" > "$build_log" 2>&1 || {
    tail -n 200 "$build_log" >&2
    die "cargo build failed; the full log is $build_log"
}
tail -n 5 "$build_log"

grep -q 'CERULION_RMW_BINDINGS_SOURCE=generated' "$build_log" ||
    die "the build script did not report generated bindings; refusing a library that may carry the vendored snapshot"
if grep -q 'CERULION_RMW_BINDINGS_SOURCE=vendored' "$build_log"; then
    die "the build script reported vendored bindings; a deployed library must be built against the distro headers"
fi

built="$CARGO_TARGET_DIR/release/$so_name"
[ -f "$built" ] || die "$built not found after the build"
cp "$built" "$out/$so_name"
chmod 0644 "$out/$so_name"

# Evidence sidecar: what the library is, what it links against, and which
# symbols it expects the hosting ROS process to provide.
{
    printf '== file ==\n'
    file "$out/$so_name"
    printf '\n== ldd ==\n'
    ldd "$out/$so_name"
    printf '\n== NEEDED ==\n'
    readelf -d "$out/$so_name" | grep NEEDED || true
    printf '\n== undefined rcutils symbols ==\n'
    nm -D --undefined-only "$out/$so_name" | grep -c ' rcutils_' || true
    printf '\n== exported rmw entry points ==\n'
    nm -D --defined-only "$out/$so_name" | grep -c ' T rmw_' || true
    printf '\n== sha256 ==\n'
    sha256sum "$out/$so_name"
} | tee "$out/$so_name.evidence.txt"

if ldd "$out/$so_name" | grep -q 'not found'; then
    die "ldd reports a missing dependency for $so_name"
fi
printf 'built %s\n' "$out/$so_name"
