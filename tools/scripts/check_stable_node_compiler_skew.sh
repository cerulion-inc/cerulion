#!/usr/bin/env bash
# Build a node and its real host with different compilers, then assert delivery.
set -euo pipefail
repo_root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$repo_root"
node_toolchain=${CERULION_ABI_NODE_TOOLCHAIN:-1.95.0}
host_toolchain=${CERULION_ABI_HOST_TOOLCHAIN:-stable}
abi_target_root=${CERULION_ABI_TARGET_ROOT:?set a dedicated target root for the compiler-skew check}
node_target="$abi_target_root/node"
host_target="$abi_target_root/host"
node_compiler=$(rustup run "$node_toolchain" rustc -vV)
host_compiler=$(rustup run "$host_toolchain" rustc -vV)
printf '%s\n' "$node_compiler" "$host_compiler"
if [ "$node_compiler" = "$host_compiler" ]; then
    echo 'compiler-skew check requires two different actual compilers' >&2
    exit 1
fi
build_env=(env -u RUSTC -u RUSTC_WRAPPER -u RUSTC_WORKSPACE_WRAPPER -u RUSTUP_TOOLCHAIN
    CARGO_BUILD_JOBS=4 CARGO_INCREMENTAL=0
    CARGO_PROFILE_DEV_DEBUG=line-tables-only CARGO_PROFILE_TEST_DEBUG=line-tables-only)
"${build_env[@]}" CARGO_TARGET_DIR="$node_target" cargo "+$node_toolchain" build --locked -p test_node_stable_abi_cross_compiler
case $(uname -s) in
    Darwin) node_library="$node_target/debug/libtest_node_stable_abi_cross_compiler.dylib" ;;
    Linux) node_library="$node_target/debug/libtest_node_stable_abi_cross_compiler.so" ;;
    *) echo 'compiler-skew native plugin check requires Linux or macOS' >&2; exit 1 ;;
esac
if [ ! -f "$node_library" ]; then
    echo 'compiler-skew node artifact is missing' >&2
    exit 1
fi
"${build_env[@]}" CARGO_TARGET_DIR="$host_target" cargo "+$host_toolchain" build --locked -p cerulion_core --test stable_node_abi_cross_compiler_test
# Local runtime evidence uses the shared desk lock; hosted runners own their VM.
if [ "${CI:-}" = true ]; then
    "${build_env[@]}" CARGO_TARGET_DIR="$host_target" CERULION_TEST_NODE_ABI_DYLIB="$node_library" cargo "+$host_toolchain" test --locked -p cerulion_core --test stable_node_abi_cross_compiler_test different_compiler_delivers_oracle_frames_in_two_identical_runs -- --ignored --exact --test-threads=1
else
    case "$host_toolchain" in
        *[!a-zA-Z0-9._-]*) echo 'unsupported toolchain name for local locked command' >&2; exit 1 ;;
    esac
    "${build_env[@]}" CARGO_TARGET_DIR="$host_target" CERULION_TEST_NODE_ABI_DYLIB="$node_library" /private/tmp/cerulion-flock.py -w 7200 /private/tmp/cerulion-build.lock -c "cargo +$host_toolchain test --locked -p cerulion_core --test stable_node_abi_cross_compiler_test different_compiler_delivers_oracle_frames_in_two_identical_runs -- --ignored --exact --test-threads=1"
fi
