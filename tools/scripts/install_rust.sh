#!/bin/sh
# Makes Rust 1.95 or newer available for building nodes. Bundled inside the
# checksummed release archive and invoked before binary activation; the Debian
# package and the Homebrew formula run it behind `cerulion-install-rust`.
# An existing rustup default is never changed.
set -eu

# The workspace MSRV: `rust-version` in the root Cargo.toml.
# tools/scripts/test_install_rust.sh fails when the two differ.
rust_floor=1.95
floor_major=${rust_floor%%.*}
floor_minor=${rust_floor#*.}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 0 ] || die "usage: install_rust.sh"

cargo_home=${CARGO_HOME:-${HOME:?set HOME or CARGO_HOME to install Rust}/.cargo}
rustup_home=${RUSTUP_HOME:-${HOME:?set HOME or RUSTUP_HOME to install Rust}/.rustup}
# Absolute, so the probes below can run from / and still find them.
case $cargo_home in /*) ;; *) cargo_home=$(pwd)/$cargo_home ;; esac
case $rustup_home in /*) ;; *) rustup_home=$(pwd)/$rustup_home ;; esac
export CARGO_HOME="$cargo_home" RUSTUP_HOME="$rustup_home"

# Succeeds when release $1 is Rust $rust_floor or newer. The release must be
# MAJOR.MINOR.PATCH; a pre-release or build suffix (1.96.0-nightly,
# 1.95.0-beta.2) is ignored, as Cargo ignores it when it checks `rust-version`.
meets_floor() {
    floor_core=${1%%[-+]*}
    case "$floor_core" in
        '' | *[!0-9.]* | .* | *. | *..*) return 1 ;;
    esac
    floor_got_major=${floor_core%%.*}
    floor_rest=${floor_core#*.}
    floor_got_minor=${floor_rest%%.*}
    floor_got_patch=${floor_rest#*.}
    [ "$floor_rest" != "$floor_core" ] && [ "$floor_got_patch" != "$floor_rest" ] || return 1
    case "$floor_got_patch" in *.*) return 1 ;; esac
    [ "$floor_got_major" -gt "$floor_major" ] ||
        { [ "$floor_got_major" -eq "$floor_major" ] && [ "$floor_got_minor" -ge "$floor_minor" ]; }
}

# Runs a Rust command as a probe: from /, so no toolchain file in the current
# directory applies, and with rustup's automatic install of a missing
# toolchain turned off (RUSTUP_AUTO_INSTALL=0).
probe() {
    (cd / && RUSTUP_AUTO_INSTALL=0 "$@")
}

# Prints the one `release:` line of the `rustc -vV` the arguments run; fails
# when the command fails or reports no single release.
rustc_release() {
    rustc_release_output=$(probe "$@" -vV 2>/dev/null) || return 1
    rustc_release_value=$(printf '%s\n' "$rustc_release_output" | sed -n 's/^release: //p')
    [ -n "$rustc_release_value" ] || return 1
    case "$rustc_release_value" in *'
'*) return 1 ;; esac
    printf '%s\n' "$rustc_release_value"
}

# Prints $1 as an absolute path.
absolute() {
    case $1 in
        /*) printf '%s\n' "$1" ;;
        *) printf '%s/%s\n' "$(pwd)" "$1" ;;
    esac
}

# Prints the release when the Rust that will build nodes reports Rust
# $rust_floor or newer and has a cargo that runs. While Cargo's home is not on
# PATH and holds both programs, that is its Rust, since the installer puts the
# directory first on PATH; otherwise it is the one PATH finds.
ready_release() {
    case ":$PATH:" in
        *":$cargo_home/bin:"*) ready_home_first=0 ;;
        *) ready_home_first=1 ;;
    esac
    if [ "$ready_home_first" -eq 1 ] && [ -x "$cargo_home/bin/rustc" ] && [ -x "$cargo_home/bin/cargo" ]; then
        set -- "$cargo_home/bin/rustc" "$cargo_home/bin/cargo"
    elif ready_rustc=$(command -v rustc 2>/dev/null) && ready_cargo=$(command -v cargo 2>/dev/null); then
        set -- "$(absolute "$ready_rustc")" "$(absolute "$ready_cargo")"
    else
        return 1
    fi
    ready_value=$(rustc_release "$1") && meets_floor "$ready_value" &&
        probe "$2" --version >/dev/null 2>&1 || return 1
    printf '%s\n' "$ready_value"
}

# A usable Rust is left exactly as it is: no lock, no install, no file.
if release=$(ready_release); then
    printf 'Rust/Cargo %s is ready.\n' "$release"
    exit 0
fi

# Different Cerulion destinations can share either Rust home. Always lock the
# Rustup home first, then the Cargo home, before inspecting any shared state.
# Separate lock names avoid self-deadlock when both homes name one directory.
setup_file_identity() {
    for setup_stat in "$(command -v stat 2>/dev/null || :)" /usr/bin/stat /bin/stat; do
        [ -x "$setup_stat" ] || continue
        "$setup_stat" -c '%d:%i' "$1" 2>/dev/null && return 0
        "$setup_stat" -f '%d:%i' "$1" 2>/dev/null && return 0
    done
    return 1
}
setup_claim_matches() {
    [ -f "$2" ] && [ ! -L "$2" ] || return 1
    setup_claim_id=$(setup_file_identity "$1") || return 1
    setup_lock_id=$(setup_file_identity "$2") || return 1
    [ "$setup_claim_id" = "$setup_lock_id" ]
}
release_setup_lock() {
    [ -n "$1" ] || return 0
    if setup_claim_matches "$1" "$2"; then
        rm -f "$2" || return 1
    elif [ "$3" -eq 1 ]; then
        printf 'error: Rust setup lock changed before release: %s\n' "$2" >&2
        return 1
    fi
    rm -f "$1"
}
acquire_setup_lock() {
    setup_attempt=0
    setup_absent_failures=0
    while [ "$setup_attempt" -lt 60 ]; do
        if [ -L "$2" ] || { [ -e "$2" ] && [ ! -f "$2" ]; }; then
            die "Rust setup lock is not a regular file: $2"
        fi
        if setup_ln_error=$(ln "$1" "$2" 2>&1); then
            setup_claim_matches "$1" "$2" && return 0
            # ln can place a link inside a directory raced into the destination.
            if [ -d "$2" ] && setup_claim_matches "$1" "$2/${1##*/}"; then
                rm -f "$2/${1##*/}" || die "could not remove raced Rust setup lock link"
            fi
            die "Rust setup lock changed during acquisition: $2"
        fi
        if [ -e "$2" ]; then
            setup_absent_failures=0
        else
            setup_absent_failures=$((setup_absent_failures + 1))
            [ "$setup_absent_failures" -lt 3 ] ||
                die "could not acquire Rust setup lock $2: $setup_ln_error"
        fi
        setup_attempt=$((setup_attempt + 1))
        sleep 1
    done
    die "timed out waiting for Rust setup lock $2; check its owner PID before removing a stale lock"
}
rustup_claim=''
cargo_claim=''
temp=''
rustup_lock=$rustup_home/.cerulion-rustup-setup.lock
cargo_lock=$cargo_home/.cerulion-cargo-setup.lock
rustup_lock_held=0 cargo_lock_held=0
cleanup_setup() {
    setup_status=$?
    trap '' HUP INT TERM
    [ -z "$temp" ] || rm -rf "$temp" || setup_status=1
    release_setup_lock "$cargo_claim" "$cargo_lock" "$cargo_lock_held" || setup_status=1
    release_setup_lock "$rustup_claim" "$rustup_lock" "$rustup_lock_held" || setup_status=1
    exit "$setup_status"
}
trap cleanup_setup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -p "$rustup_home" "$cargo_home"
rustup_claim=$(mktemp "$rustup_home/.cerulion-setup-owner.XXXXXX") ||
    die "could not prepare Rustup setup lock"
printf '%s\n' "$$" > "$rustup_claim"
acquire_setup_lock "$rustup_claim" "$rustup_lock"
rustup_lock_held=1
cargo_claim=$(mktemp "$cargo_home/.cerulion-setup-owner.XXXXXX") ||
    die "could not prepare Cargo setup lock"
printf '%s\n' "$$" > "$cargo_claim"
acquire_setup_lock "$cargo_claim" "$cargo_lock"
cargo_lock_held=1

if [ -x "$cargo_home/bin/rustup" ]; then
    rustup=$cargo_home/bin/rustup
elif command -v rustup >/dev/null 2>&1; then
    rustup=$(command -v rustup)
else
    if [ -e "$rustup_home/settings.toml" ] || [ -L "$rustup_home/settings.toml" ]; then
        die "Rustup settings exist in $rustup_home but its executable is unavailable; restore rustup on PATH before retrying so your default compiler and profile remain unchanged"
    fi
    if command -v cargo >/dev/null 2>&1 || command -v rustc >/dev/null 2>&1 ||
        [ -e "$cargo_home/bin/cargo" ] || [ -L "$cargo_home/bin/cargo" ] ||
        [ -e "$cargo_home/bin/rustc" ] || [ -L "$cargo_home/bin/rustc" ] ||
        [ -e "$cargo_home/bin/rustup" ] || [ -L "$cargo_home/bin/rustup" ]; then
        die "Rust/Cargo is installed without rustup but is not a working Rust $rust_floor or newer with Cargo; upgrade or repair it with the tool that installed it, or install rustup explicitly to add current stable Rust without replacing your existing installation"
    fi
    temp=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-rust.XXXXXX") ||
        die "could not create Rust bootstrap directory"
    printf '%s\n' 'Installing current stable Rust/Cargo with rustup'
    # Plain --retry, which every curl accepts (--retry-all-errors needs 7.71).
    # --connect-timeout and --max-time bound one attempt, so a stalled
    # connection times out and is retried.
    curl --proto '=https' --tlsv1.2 -fsSL --retry 5 --retry-delay 5 \
        --connect-timeout 30 --max-time 120 https://sh.rustup.rs -o "$temp/rustup-init.sh" ||
        die "could not download the official rustup installer"
    sh "$temp/rustup-init.sh" -y --profile minimal --default-toolchain stable --no-modify-path ||
        die "Rust/Cargo installation failed; Cerulion binaries have not been replaced"
    rustup=$cargo_home/bin/rustup
    [ -x "$rustup" ] || die "rustup did not install into $cargo_home/bin"
fi
rustup_dir=$(CDPATH='' cd -- "$(dirname -- "$rustup")" && pwd) ||
    die "could not locate the selected rustup executable"
rustup="$rustup_dir/${rustup##*/}"

# An installed stable at or above the floor is reused; `rustup run` never
# installs a missing toolchain.
if stable=$(rustc_release "$rustup" run stable rustc) && meets_floor "$stable"; then
    :
else
    printf '%s\n' 'Adding current stable Rust/Cargo alongside existing toolchains'
    "$rustup" toolchain install stable --profile minimal --no-self-update ||
        die "could not install stable Rust/Cargo; Cerulion binaries have not been replaced"
    stable=$(rustc_release "$rustup" run stable rustc) ||
        die "rustup installed stable Rust, but its rustc reports no release; Cerulion binaries have not been replaced"
    meets_floor "$stable" ||
        die "stable Rust from rustup is $stable, older than Rust $rust_floor; Cerulion binaries have not been replaced"
fi
stable_cargo=$("$rustup" run stable cargo --version 2>/dev/null) ||
    die "stable Rust has no usable Cargo; Cerulion binaries have not been replaced"

# A PATH-installed rustup need not have proxies in a custom Cargo home. Add
# only absent paths; never replace an existing executable or dangling symlink.
mkdir -p "$cargo_home/bin"
for proxy in rustup cargo rustc; do
    proxy_path=$cargo_home/bin/$proxy
    if [ ! -e "$proxy_path" ] && [ ! -L "$proxy_path" ]; then
        ln -s "$rustup" "$proxy_path" || die "could not create Rust proxy: $proxy_path"
    fi
done
proxy_compiler=$(RUSTUP_TOOLCHAIN=stable "$cargo_home/bin/rustc" -vV) ||
    die "Cargo home's rustc proxy is unusable; existing files were preserved"
proxy_release=$(printf '%s\n' "$proxy_compiler" | sed -n 's/^release: //p')
[ "$proxy_release" = "$stable" ] ||
    die "Cargo home's rustc proxy does not select stable Rust; existing files were preserved"
proxy_cargo=$(RUSTUP_TOOLCHAIN=stable "$cargo_home/bin/cargo" --version) ||
    die "Cargo home's cargo proxy is unusable; existing files were preserved"
[ "$proxy_cargo" = "$stable_cargo" ] ||
    die "Cargo home's cargo proxy does not match stable Rust's Cargo; existing files were preserved"

# Only a missing default becomes stable; an existing one stays as it is. This
# comes after every refusal above, so a refused run leaves the default alone.
default_toolchain=
if default_output=$("$rustup" default 2>/dev/null); then
    default_toolchain=$(printf '%s\n' "$default_output" | awk 'NR == 1 { print $1 }')
fi
if [ -z "$default_toolchain" ]; then
    "$rustup" default stable >/dev/null ||
        die "could not make stable Rust the default toolchain"
    default_toolchain=stable
fi

if default_release=$(rustc_release "$rustup" run "$default_toolchain" rustc) &&
    meets_floor "$default_release"; then
    printf 'Rust/Cargo %s is ready.\n' "$default_release"
else
    # shellcheck disable=SC2016  # the backticks are literal text for the reader
    printf 'Rust %s or newer is now installed as `stable`, and your default toolchain was kept. Select stable with `rustup default stable`, or for one build with `RUSTUP_TOOLCHAIN=stable cerulion node build <node>`.\n' \
        "$rust_floor"
fi
printf 'Cargo executables: %s/bin\n' "$cargo_home"
