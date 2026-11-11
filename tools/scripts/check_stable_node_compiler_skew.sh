#!/usr/bin/env bash
# Build a node and its real host with different compilers, then assert delivery.
set -euo pipefail
repo_root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$repo_root"
node_toolchain=${CERULION_ABI_NODE_TOOLCHAIN:-1.95.0}
host_toolchain=${CERULION_ABI_HOST_TOOLCHAIN:-stable}
abi_target_root=${CERULION_ABI_TARGET_ROOT:?set a dedicated target root for the compiler-skew check}
node_target="$abi_target_root/node"
host_target=${CERULION_ABI_HOST_TARGET:-"$abi_target_root/host"}
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
# A standalone workspace freshly resolves its dependency graph. Reusing the
# root Cargo.lock would hide loader extension-trait version skew.
fresh_workspace="$abi_target_root/fresh-node"
abi_python=python3
if ! "$abi_python" -c 'import tomllib' 2>/dev/null; then
    abi_python=
    for candidate in python3.14 python3.13 python3.12 python3.11; do
        if command -v "$candidate" >/dev/null && "$candidate" -c 'import tomllib' 2>/dev/null; then
            abi_python=$candidate
            break
        fi
    done
    if [ -z "$abi_python" ]; then
        echo 'fresh standalone node generation requires Python 3.11 or newer' >&2
        exit 1
    fi
fi
"$abi_python" - "$repo_root" "$fresh_workspace" <<'FRESH_NODE'
import json
import pathlib
import re
import shutil
import sys
import tomllib
repo = pathlib.Path(sys.argv[1])
fresh = pathlib.Path(sys.argv[2])
fresh.mkdir(parents=True, exist_ok=True)
(fresh / "src").mkdir(exist_ok=True)
source = repo / "crates/test_fixtures/test_node_stable_abi_cross_compiler"
shutil.copyfile(source / "src/lib.rs", fresh / "src/lib.rs")
manifest = (source / "Cargo.toml").read_text()
for name in ["cerulion_core", "native_ros2_messages"]:
    manifest = manifest.replace(name + ".workspace = true", name + " = { path = " + json.dumps(str(repo / "crates" / name)) + " }")
manifest = manifest.replace("[lints]\nworkspace = true", "[workspace]")
root_text = (repo / "Cargo.toml").read_text()
root = tomllib.loads(root_text)
for section in re.finditer(r"(?ms)^\[workspace\.lints\.[^\]]+\]\n.*?(?=^\[|\Z)", root_text):
    manifest += "\n" + section.group()
manifest += "\n[lints]\nworkspace = true\n\n[patch.crates-io]\n"
for name, specification in root.get("patch", {}).get("crates-io", {}).items():
    specification = dict(specification)
    if "path" in specification:
        specification["path"] = str(repo / specification["path"])
    fields = ", ".join(key + " = " + json.dumps(value) for key, value in specification.items())
    manifest += json.dumps(name) + " = { " + fields + " }\n"
(fresh / "Cargo.toml").write_text(manifest)
# Only the isolated copy's lock is removed; the repository locks stay intact.
(fresh / "Cargo.lock").unlink(missing_ok=True)
FRESH_NODE
"${build_env[@]}" cargo "+$node_toolchain" generate-lockfile --manifest-path "$fresh_workspace/Cargo.toml"
"${build_env[@]}" CARGO_TARGET_DIR="$node_target" cargo "+$node_toolchain" build --locked --manifest-path "$fresh_workspace/Cargo.toml"

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
if [ "${CI:-}" = true ] || [ "${CERULION_ABI_DESK_LOCK_HELD:-}" = 1 ]; then
    "${build_env[@]}" CARGO_TARGET_DIR="$host_target" CERULION_TEST_NODE_ABI_DYLIB="$node_library" cargo "+$host_toolchain" test --locked -p cerulion_core --test stable_node_abi_cross_compiler_test different_compiler_delivers_oracle_frames_in_two_identical_runs -- --ignored --exact --test-threads=1
else
    case "$host_toolchain" in
        *[!a-zA-Z0-9._-]*) echo 'unsupported toolchain name for local locked command' >&2; exit 1 ;;
    esac
    "${build_env[@]}" CARGO_TARGET_DIR="$host_target" CERULION_TEST_NODE_ABI_DYLIB="$node_library" /private/tmp/cerulion-flock.py -w 7200 /private/tmp/cerulion-build.lock -c "cargo +$host_toolchain test --locked -p cerulion_core --test stable_node_abi_cross_compiler_test different_compiler_delivers_oracle_frames_in_two_identical_runs -- --ignored --exact --test-threads=1"
fi
