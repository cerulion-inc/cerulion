#!/usr/bin/env bash
# build_release_linux_container.sh: build the Linux release binaries inside
# an `ubuntu:20.04` container.
#
# Runs as root INSIDE the container with the checkout mounted at /work. The
# release workflow builds the release archives with it, and CI's Debian
# smoke builds the debug programs it packages with it. The
# programs and the heap hook link against the container's glibc 2.31, so
# every symbol version they reference exists on Ubuntu 20.04; the release
# workflow's glibc check then confirms it. The build is native: the x86_64
# leg runs on an x86_64 runner and the aarch64 leg on an arm64 runner.
#
# It writes into /work/target exactly where a host build would
# (target/<TARGET>/<release|debug>/...) and hands the target directory back to the
# runner's user on exit, so the host steps that package the archive and the
# deb read it unchanged.
#
# Env:
#   TARGET                     the Rust target triple (required; must match
#                              the container's own architecture)
#   CERULION_RELEASE_TOOLCHAIN the Rust toolchain to install (required; the
#                              workflow passes the version it pins elsewhere)
#   CERULION_HOST_OWNER        uid:gid that owns target/ afterwards (required)
#   CERULION_BUILD_PROFILE     release (the default) or debug
#   CERULION_POSTHOG_KEY       passed through to the CLI build (optional)
set -euo pipefail

target=${TARGET:?TARGET must name the Rust target triple}
toolchain=${CERULION_RELEASE_TOOLCHAIN:?CERULION_RELEASE_TOOLCHAIN must name the Rust toolchain}
owner=${CERULION_HOST_OWNER:?CERULION_HOST_OWNER must be uid:gid}
profile=${CERULION_BUILD_PROFILE:-release}
repo=/work

die() {
    printf 'build_release_linux_container: %s\n' "$*" >&2
    exit 1
}

[ -f "$repo/Cargo.toml" ] || die "repository is not mounted at $repo"
case "$profile" in
    release) profile_flag=--release ;;
    debug) profile_flag= ;;
    *) die "CERULION_BUILD_PROFILE must be release or debug, got '$profile'" ;;
esac

case "$(uname -m)/$target" in
    x86_64/x86_64-unknown-linux-gnu | aarch64/aarch64-unknown-linux-gnu) ;;
    *) die "target $target does not match the container architecture $(uname -m); this build is native only" ;;
esac

# getconf, not `ldd --version | head`: head closes the pipe early, ldd takes
# SIGPIPE, and pipefail ends the script with status 141 and nothing printed.
glibc=$(getconf GNU_LIBC_VERSION)
printf 'container glibc: %s\n' "$glibc"
case "$glibc" in
    *" 2.31") ;;
    *) die "expected glibc 2.31 (Ubuntu 20.04) in the build container, got: $glibc" ;;
esac

hand_back() {
    if [ -d "$repo/target" ]; then
        chown -R "$owner" "$repo/target"
    fi
}
trap hand_back EXIT

export DEBIAN_FRONTEND=noninteractive
apt-get update
# build-essential: the C parts of the dependency tree (aws-lc); clang and
# libclang-dev: bindgen in the iceoryx2 platform layer; cmake: aws-lc's CMake
# builder when it selects one; curl and ca-certificates: the rustup download;
# git: available to cargo for the git dependencies.
apt-get install -y --no-install-recommends \
    build-essential pkg-config cmake clang libclang-dev \
    curl ca-certificates git
rm -rf /var/lib/apt/lists/*

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh
sh /tmp/rustup-init.sh -y --profile minimal --no-modify-path \
    --default-toolchain "$toolchain"
export PATH="$HOME/.cargo/bin:$PATH"
cargo --version
rustc --version

cd "$repo"
# shellcheck disable=SC2086 # an empty profile_flag must vanish
cargo build $profile_flag --locked \
    -p cerulion_cli -p cerulion_netd -p cerulion_connectd -p cerulion_remoted \
    --target "$target"
# The heap hook, an LD_PRELOAD malloc interposer the CLI auto-injects into
# every ros2 child when it sits beside the binaries.
# shellcheck disable=SC2086
cargo build $profile_flag --locked -p cerulion_heaphook --target "$target"

# The compiler that built the artifacts, which the archive ships as
# rustc-version.txt.
rustc -vV > "target/$target/$profile/rustc-version.txt"

for artifact in cerulion cerulion-netd cerulion-connectd cerulion-remoted libcerulion_heaphook.so rustc-version.txt; do
    [ -f "target/$target/$profile/$artifact" ] ||
        die "the build left no target/$target/$profile/$artifact"
done
