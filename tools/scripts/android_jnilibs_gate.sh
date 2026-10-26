#!/usr/bin/env bash
# Checks the native libraries an Android app ships before Gradle packages them.
#
#   android_jnilibs_gate.sh <jniLibs dir> <library name>
#   android_jnilibs_gate.sh --self-test [<jniLibs dir> <library name>]
#
# An app that embeds Cerulion loads exactly ONE native library: the app
# runtime library that links Cerulion and the app's graph nodes. For each ABI
# directory under <jniLibs dir> the gate requires:
#   - exactly one `.so`, and it is `lib<library name>.so`; a second library
#     (for example a node crate's own `.so`) would put a second runtime with
#     the same JNI exports into the app;
#   - an ELF machine type that matches the ABI directory (arm64-v8a is
#     AArch64, x86_64 is X86-64, armeabi-v7a is ARM, x86 is Intel 80386);
#   - every LOAD segment aligned to at least 16 KB (0x4000), so the library
#     loads on devices with 16 KB pages;
#   - `JNI_OnLoad` and `Java_com_cerulion_android_CerulionRuntime_nativeInit`
#     exported, and `cerulion_node_info` NOT exported: a node's C exports in
#     the app library mean a node crate was linked with its `cdylib` feature
#     on instead of `default-features = false`.
#
# `--self-test` links small libraries with the NDK clang and checks that the
# gate refuses each defect and accepts a good library. With a jniLibs
# directory and a library name it then also checks that real library.
#
# Tools come from the NDK: set ANDROID_NDK_HOME (or ANDROID_NDK_ROOT).
set -euo pipefail

MIN_ALIGN=$((0x4000))
REQUIRED_SYMBOLS=(JNI_OnLoad Java_com_cerulion_android_CerulionRuntime_nativeInit)
FORBIDDEN_SYMBOLS=(cerulion_node_info)

usage() {
  echo "usage: $0 <jniLibs dir> <library name>" >&2
  echo "       $0 --self-test [<jniLibs dir> <library name>]" >&2
  exit 2
}

ndk_bin() {
  local ndk="${ANDROID_NDK_HOME:-${ANDROID_NDK_ROOT:-}}"
  [[ -n "$ndk" ]] || { echo "FAIL jniLibs gate: set ANDROID_NDK_HOME to the NDK directory" >&2; exit 1; }
  local host
  case "$(uname -s)" in
    Darwin) host=darwin-x86_64 ;; # the NDK uses this tag on arm64 Macs too
    Linux) host=linux-x86_64 ;;
    *) echo "FAIL jniLibs gate: unsupported host $(uname -s)" >&2; exit 1 ;;
  esac
  local bin="$ndk/toolchains/llvm/prebuilt/$host/bin"
  [[ -x "$bin/llvm-readelf" && -x "$bin/llvm-nm" ]] ||
    { echo "FAIL jniLibs gate: no llvm-readelf/llvm-nm under $bin" >&2; exit 1; }
  printf '%s\n' "$bin"
}

# Prints every problem with one library, one per line; prints nothing when
# the library is good.
# The `Machine:` text llvm-readelf prints for the libraries of one ABI
# directory; empty for a directory name that is not an Android ABI.
abi_machine() {
  case "$1" in
    arm64-v8a) echo AArch64 ;;
    x86_64) echo "Advanced Micro Devices X86-64" ;;
    armeabi-v7a) echo ARM ;;
    x86) echo "Intel 80386" ;;
    *) echo "" ;;
  esac
}

library_problems() {
  local bin=$1 lib=$2 abi=$3
  local headers
  if ! headers="$("$bin/llvm-readelf" -hlW "$lib" 2>&1)"; then
    echo "$lib: llvm-readelf failed: $headers"
    return
  fi
  local want machine
  want="$(abi_machine "$abi")"
  machine="$(awk -F': *' '$1 ~ /^ *Machine$/ { print $2; exit }' <<<"$headers")"
  if [[ -z "$want" ]]; then
    echo "$lib: $abi is not an Android ABI directory (arm64-v8a, x86_64, armeabi-v7a, x86)"
  elif [[ "$machine" != "$want" ]]; then
    echo "$lib: ELF machine is ${machine:-unknown}, expected $want for $abi"
  fi
  local index=0 loads=0 align
  while read -r align; do
    loads=$((loads + 1))
    if [[ ! "$align" =~ ^0x[0-9a-fA-F]+$ ]] || (( align < MIN_ALIGN )); then
      echo "$lib: LOAD segment $index has Align $align, below 0x4000 (16 KB pages)"
    fi
    index=$((index + 1))
  done < <(awk '$1 == "LOAD" { print $NF }' <<<"$headers")
  (( loads > 0 )) || echo "$lib: no LOAD segment found"
  local symbols
  if ! symbols="$("$bin/llvm-nm" -D --defined-only "$lib" 2>&1)"; then
    echo "$lib: llvm-nm failed: $symbols"
    return
  fi
  local exported symbol
  exported="$(awk '{ print $NF }' <<<"$symbols")"
  for symbol in "${REQUIRED_SYMBOLS[@]}"; do
    grep -qxF "$symbol" <<<"$exported" || echo "$lib: does not export $symbol"
  done
  for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
    if grep -qxF "$symbol" <<<"$exported"; then
      echo "$lib: exports $symbol; link node crates with default-features = false"
    fi
  done
}

# Prints every problem with a jniLibs directory; prints nothing when it is
# good.
jnilibs_problems() {
  local bin=$1 dir=$2 name=$3
  [[ -d "$dir" ]] || { echo "$dir: not a directory"; return; }
  local abi_dirs=() abi_dir
  while IFS= read -r abi_dir; do
    abi_dirs+=("$abi_dir")
  done < <(find "$dir" -mindepth 1 -maxdepth 1 -type d | sort)
  (( ${#abi_dirs[@]} > 0 )) || { echo "$dir: no ABI directory"; return; }
  for abi_dir in "${abi_dirs[@]}"; do
    local libs=() lib
    while IFS= read -r lib; do
      libs+=("$lib")
    done < <(find "$abi_dir" -mindepth 1 -maxdepth 1 -name '*.so' | sort)
    if (( ${#libs[@]} != 1 )); then
      local names=()
      for lib in ${libs[@]+"${libs[@]}"}; do names+=("$(basename "$lib")"); done
      echo "$abi_dir: expected exactly one library lib$name.so, found ${#libs[@]}: ${names[*]:-none}"
      continue
    fi
    if [[ "$(basename "${libs[0]}")" != "lib$name.so" ]]; then
      echo "$abi_dir: expected lib$name.so, found $(basename "${libs[0]}")"
      continue
    fi
    library_problems "$bin" "${libs[0]}" "$(basename "$abi_dir")"
  done
}

check() {
  local bin=$1 dir=$2 name=$3 problems
  problems="$(jnilibs_problems "$bin" "$dir" "$name")"
  if [[ -n "$problems" ]]; then
    printf 'FAIL jniLibs gate: %s\n' "$problems" >&2
    return 1
  fi
  echo "PASS jniLibs gate: $dir holds one 16 KB aligned lib$name.so per ABI, built for that ABI, with the JNI exports and no node exports"
}

self_test() {
  local bin=$1
  local clang="$bin/aarch64-linux-android30-clang"
  [[ -x "$clang" ]] || { echo "FAIL jniLibs self-test: no NDK clang at $clang" >&2; exit 1; }
  local work
  work="$(mktemp -d "${TMPDIR:-/tmp}/jnilibs-gate.XXXXXX")"
  # shellcheck disable=SC2064 # expand $work now: it is local to this function
  trap "rm -rf '$work'" EXIT
  cat >"$work/good.c" <<'EOF'
int JNI_OnLoad(void *vm, void *reserved) { (void)vm; (void)reserved; return 0x00010006; }
int Java_com_cerulion_android_CerulionRuntime_nativeInit(void) { return 0; }
EOF
  cat >"$work/node.c" <<'EOF'
int cerulion_node_info(void) { return 0; }
EOF
  cat >"$work/no_onload.c" <<'EOF'
int Java_com_cerulion_android_CerulionRuntime_nativeInit(void) { return 0; }
EOF
  local clang_x86_64="$bin/x86_64-linux-android30-clang"
  [[ -x "$clang_x86_64" ]] || { echo "FAIL jniLibs self-test: no NDK clang at $clang_x86_64" >&2; exit 1; }
  link_with() { # <clang> <out> <page size> <sources...>
    local cc=$1 out=$2 page=$3
    shift 3
    mkdir -p "$(dirname "$out")"
    "$cc" -shared -fPIC -Wl,-z,max-page-size="$page" -o "$out" "$@"
  }
  link() { # <out> <page size> <sources...>; an arm64 library
    link_with "$clang" "$@"
  }
  local failures=0
  expect() { # <pass|fail> <case> <dir> <expected text in the failure>
    local want=$1 label=$2 dir=$3 needle=${4:-} out rc=0
    out="$(check "$bin" "$dir" app_runtime 2>&1)" || rc=$?
    if [[ "$want" == pass && $rc -eq 0 ]] ||
       [[ "$want" == fail && $rc -ne 0 && "$out" == *"$needle"* ]]; then
      echo "PASS jniLibs self-test $label"
    else
      echo "FAIL jniLibs self-test $label: rc=$rc output=$out" >&2
      failures=$((failures + 1))
    fi
  }
  link "$work/good/arm64-v8a/libapp_runtime.so" 16384 "$work/good.c"
  expect pass "16 KB aligned library" "$work/good"
  link "$work/page4k/arm64-v8a/libapp_runtime.so" 4096 "$work/good.c"
  expect fail "4 KB aligned library" "$work/page4k" "below 0x4000"
  link "$work/two/arm64-v8a/libapp_runtime.so" 16384 "$work/good.c"
  link "$work/two/arm64-v8a/libtimer.so" 16384 "$work/node.c"
  expect fail "second library" "$work/two" "found 2: libapp_runtime.so libtimer.so"
  link "$work/other/arm64-v8a/libcerulion_android.so" 16384 "$work/good.c"
  expect fail "wrong library name" "$work/other" "expected libapp_runtime.so"
  link "$work/node_exports/arm64-v8a/libapp_runtime.so" 16384 "$work/good.c" "$work/node.c"
  expect fail "node exports" "$work/node_exports" "exports cerulion_node_info"
  link "$work/no_onload/arm64-v8a/libapp_runtime.so" 16384 "$work/no_onload.c"
  expect fail "missing JNI_OnLoad" "$work/no_onload" "does not export JNI_OnLoad"
  link_with "$clang_x86_64" "$work/both/x86_64/libapp_runtime.so" 16384 "$work/good.c"
  link "$work/both/arm64-v8a/libapp_runtime.so" 16384 "$work/good.c"
  expect pass "arm64-v8a and x86_64 libraries" "$work/both"
  link "$work/wrong_abi/x86_64/libapp_runtime.so" 16384 "$work/good.c"
  expect fail "library built for another ABI" "$work/wrong_abi" "ELF machine is AArch64, expected Advanced Micro Devices X86-64 for x86_64"
  mkdir -p "$work/empty/arm64-v8a"
  expect fail "empty ABI directory" "$work/empty" "found 0"
  (( failures == 0 )) || { echo "FAIL jniLibs self-test: $failures case(s) failed" >&2; exit 1; }
}

case "${1:-}" in
  --self-test)
    (( $# == 1 || $# == 3 )) || usage
    BIN="$(ndk_bin)"
    self_test "$BIN"
    if (( $# == 3 )); then
      check "$BIN" "$2" "$3"
    fi
    ;;
  -*|"") usage ;;
  *)
    (( $# == 2 )) || usage
    BIN="$(ndk_bin)"
    check "$BIN" "$1" "$2"
    ;;
esac
