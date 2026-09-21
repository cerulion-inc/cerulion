#!/bin/sh
# Bundled inside the checksummed release archive; invoked before binary activation.
set -eu

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 1 ] || die "usage: install_rust.sh RUSTC_VERSION_FILE"
[ -f "$1" ] || die "release archive is missing its Rust compiler metadata"
metadata=$1
release=$(awk '/^release: / { count++; value=$2; if (NF != 2) invalid=1 } END { if (count != 1 || invalid) exit 1; print value }' "$metadata") ||
    die "release archive has invalid Rust release metadata"
commit=$(awk '/^commit-hash: / { count++; value=$2; if (NF != 2) invalid=1 } END { if (count != 1 || invalid) exit 1; print value }' "$metadata") ||
    die "release archive has invalid Rust compiler fingerprint"
printf '%s\n' "$release" | grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+$' ||
    die "release archive requires an unsupported Rust toolchain: $release"
printf '%s\n' "$commit" | grep -Eq '^[0-9a-f]{40}$' ||
    die "release archive has invalid Rust compiler fingerprint"

cargo_home=${CARGO_HOME:-${HOME:?set HOME or CARGO_HOME to install Rust}/.cargo}
rustup_home=${RUSTUP_HOME:-${HOME:?set HOME or RUSTUP_HOME to install Rust}/.rustup}
export CARGO_HOME="$cargo_home" RUSTUP_HOME="$rustup_home"

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
        die "Rust/Cargo is installed without rustup; install rustup explicitly to add Rust $release without replacing your existing installation"
    fi
    temp=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-rust.XXXXXX") ||
        die "could not create Rust bootstrap directory"
    printf 'Installing Rust/Cargo %s with rustup\n' "$release"
    curl --proto '=https' --tlsv1.2 -fsSL https://sh.rustup.rs -o "$temp/rustup-init.sh" ||
        die "could not download the official rustup installer"
    sh "$temp/rustup-init.sh" -y --profile minimal --default-toolchain "$release" --no-modify-path ||
        die "Rust/Cargo installation failed; Cerulion binaries have not been replaced"
    rustup=$cargo_home/bin/rustup
    [ -x "$rustup" ] || die "rustup did not install into $cargo_home/bin"
fi

cargo_version_matches() {
    case "$1" in
        "cargo $release"|"cargo $release ("*")") return 0 ;;
        *) return 1 ;;
    esac
}

compiler_matches() {
    compiler=$("$rustup" run "$release" rustc -vV 2>/dev/null) || return 1
    actual_release=$(printf '%s\n' "$compiler" | sed -n 's/^release: //p')
    actual_commit=$(printf '%s\n' "$compiler" | sed -n 's/^commit-hash: //p')
    [ "$actual_release" = "$release" ] && [ "$actual_commit" = "$commit" ] || return 1
    actual_cargo=$("$rustup" run "$release" cargo --version 2>/dev/null) || return 1
    cargo_version_matches "$actual_cargo"
}

if ! compiler_matches; then
    printf 'Adding Rust/Cargo %s alongside existing toolchains\n' "$release"
    "$rustup" toolchain install "$release" --profile minimal --no-self-update ||
        die "could not install Rust/Cargo $release; Cerulion binaries have not been replaced"
fi
compiler_matches ||
    die "installed Rust toolchain does not match the release fingerprint or Cargo version; Cerulion binaries have not been replaced"
# A PATH-installed rustup need not have proxies in a custom Cargo home. Add
# only absent paths; never replace an existing executable or dangling symlink.
rustup_dir=$(CDPATH='' cd -- "$(dirname -- "$rustup")" && pwd) ||
    die "could not locate the selected rustup executable"
rustup="$rustup_dir/${rustup##*/}"
mkdir -p "$cargo_home/bin"
for proxy in rustup cargo rustc; do
    proxy_path=$cargo_home/bin/$proxy
    if [ ! -e "$proxy_path" ] && [ ! -L "$proxy_path" ]; then
        ln -s "$rustup" "$proxy_path" || die "could not create Rust proxy: $proxy_path"
    fi
done
proxy_compiler=$(RUSTUP_TOOLCHAIN="$release" "$cargo_home/bin/rustc" -vV) ||
    die "Cargo home's rustc proxy is unusable; existing files were preserved"
proxy_release=$(printf '%s\n' "$proxy_compiler" | sed -n 's/^release: //p')
proxy_commit=$(printf '%s\n' "$proxy_compiler" | sed -n 's/^commit-hash: //p')
if [ "$proxy_release" != "$release" ] || [ "$proxy_commit" != "$commit" ]; then
    die "Cargo home's rustc proxy does not select the release compiler; existing files were preserved"
fi
proxy_cargo=$(RUSTUP_TOOLCHAIN="$release" "$cargo_home/bin/cargo" --version) ||
    die "Cargo home's cargo proxy is unusable; existing files were preserved"
[ "$proxy_cargo" = "$actual_cargo" ] ||
    die "Cargo home's cargo proxy does not match the release toolchain's Cargo; existing files were preserved"
printf 'Rust/Cargo %s is ready. Existing rustup defaults are preserved.\n' "$release"
printf 'Cargo executables: %s/bin\n' "$cargo_home"
