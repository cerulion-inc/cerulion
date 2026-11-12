#!/bin/sh
set -eu
original_path=$PATH
script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-rust-test.XXXXXX")
trap 'rm -rf "$work"' EXIT
# The installs below that succeed write the PATH setup under HOME and link
# into ~/.local/bin; give them a scratch home so a run on a workstation leaves
# the real one alone, and a system link directory that does not exist.
HOME=$work/home
CERULION_SYSTEM_BIN_DIR=$work/no-system-bin
export HOME CERULION_SYSTEM_BIN_DIR
mkdir -p "$HOME"

# The helper's floor is the workspace MSRV, `rust-version` in the root
# Cargo.toml. Each side must name it exactly once.
script_floor=$(sed -n 's/^rust_floor=\([0-9][0-9.]*\)$/\1/p' "$script_dir/install_rust.sh")
workspace_floor=$(sed -n 's/^rust-version = "\([0-9][0-9.]*\)"$/\1/p' "$script_dir/../../Cargo.toml")
if [ -z "$script_floor" ] || [ "$script_floor" != "$workspace_floor" ]; then
    printf 'error: install_rust.sh floor (%s) differs from the workspace rust-version (%s)\n' \
        "$script_floor" "$workspace_floor" >&2
    exit 1
fi

tools=$work/tools
mkdir -p "$tools"
for utility in awk grep sed mktemp rm chmod cp cat cmp dirname mkdir ln readlink sleep; do
    ln -s "$(command -v "$utility")" "$tools/$utility"
done
# A hand-written rustup. Toolchains are files under RUSTUP_HOME holding the
# release they report; the default lives in settings.toml, as rustup keeps it.
# Invoked as rustc or cargo it is a proxy: RUSTUP_TOOLCHAIN, else the default.
cat > "$work/rustup-fixture" <<'EOF'
#!/bin/sh
set -eu
settings=$RUSTUP_HOME/settings.toml
toolchains=$RUSTUP_HOME/fixture-toolchains
unexpected() {
    printf 'unexpected rustup command: %s\n' "$*" >&2
    exit 1
}
default_toolchain() {
    [ -f "$settings" ] || return 1
    fixture_default=$(sed -n 's/^default_toolchain = "\(.*\)"$/\1/p' "$settings")
    [ -n "$fixture_default" ] || return 1
    printf '%s\n' "$fixture_default"
}
case "${0##*/}" in
    rustc | cargo)
        proxy=${0##*/}
        if [ "$proxy" = rustc ]; then
            [ "$*" = -vV ] || unexpected "$@"
        else
            [ "$*" = --version ] || unexpected "$@"
        fi
        toolchain=${RUSTUP_TOOLCHAIN:-}
        if [ -z "$toolchain" ]; then
            toolchain=$(default_toolchain) || {
                printf '%s\n' 'error: rustup could not choose a version to run, and no default is configured' >&2
                exit 1
            }
        fi
        if [ "$proxy" = rustc ] && [ -n "${FIXTURE_PROXY_RELEASE:-}" ]; then
            printf 'release: %s\n' "$FIXTURE_PROXY_RELEASE"
            exit 0
        fi
        set -- run "$toolchain" "$proxy" "$@"
        ;;
esac
# A probe runs with automatic toolchain installs off, from /; it says so.
printf '%s%s\n' "$*" "${RUSTUP_AUTO_INSTALL:+ [auto-install=$RUSTUP_AUTO_INSTALL cwd=$(pwd)]}" >> "$CARGO_HOME/calls"
case "$1" in
    run)
        [ "$#" -eq 4 ] || unexpected "$@"
        [ -f "$toolchains/$2" ] || exit 1
        release=$(cat "$toolchains/$2")
        case "$3 $4" in
            'rustc -vV') printf 'rustc %s (fixture)\nbinary: rustc\nrelease: %s\n' "$release" "$release" ;;
            'cargo --version') printf 'cargo %s (fixture)\n' "$release" ;;
            *) unexpected "$@" ;;
        esac
        ;;
    toolchain)
        [ "$*" = 'toolchain install stable --profile minimal --no-self-update' ] || unexpected "$@"
        [ "${FIXTURE_FAIL_INSTALL:-0}" = 0 ] || exit 1
        [ -z "${FIXTURE_BARRIER:-}" ] || sh "$FIXTURE_BARRIER"
        mkdir -p "$toolchains"
        printf '%s\n' "${FIXTURE_STABLE_RELEASE:-1.98.0}" > "$toolchains/stable"
        ;;
    default)
        if [ "$#" -eq 1 ]; then
            fixture_default=$(default_toolchain) || {
                printf '%s\n' 'error: no default toolchain is configured' >&2
                exit 1
            }
            printf '%s (default)\n' "$fixture_default"
        else
            [ "$*" = 'default stable' ] || unexpected "$@"
            [ -f "$toolchains/stable" ] || exit 1
            fixture_settings=$( { grep -v '^default_toolchain = ' "$settings" 2>/dev/null || :; } )
            {
                [ -z "$fixture_settings" ] || printf '%s\n' "$fixture_settings"
                printf '%s\n' 'default_toolchain = "stable"'
            } > "$settings"
        fi
        ;;
    *) unexpected "$@" ;;
esac
EOF
# The fixture checks the exact HTTPS source and bootstrap arguments, then
# installs the hand-written rustup into the requested Cargo home with stable
# as its default toolchain, as the official installer does.
cat > "$tools/curl" <<'EOF'
#!/bin/sh
set -eu
if [ "$#" != 15 ] || [ "$1" != --proto ] || [ "$2" != '=https' ] ||
    [ "$3" != --tlsv1.2 ] || [ "$4" != -fsSL ] ||
    [ "$5" != --retry ] || [ "$6" != 5 ] || [ "$7" != --retry-delay ] || [ "$8" != 5 ] ||
    [ "$9" != --connect-timeout ] || [ "${10}" != 30 ] ||
    [ "${11}" != --max-time ] || [ "${12}" != 120 ] ||
    [ "${13}" != https://sh.rustup.rs ] || [ "${14}" != -o ]; then
    printf '%s\n' 'unexpected curl arguments' >&2
    exit 1
fi
[ "${FIXTURE_FAIL_DOWNLOAD:-0}" = 0 ] || exit 1
cat > "${15}" <<'INIT'
#!/bin/sh
set -eu
[ "$*" = '-y --profile minimal --default-toolchain stable --no-modify-path' ]
[ "${FIXTURE_FAIL_BOOTSTRAP:-0}" = 0 ] || exit 1
[ -z "${FIXTURE_BARRIER:-}" ] || sh "$FIXTURE_BARRIER"
mkdir -p "$CARGO_HOME/bin" "$RUSTUP_HOME/fixture-toolchains"
cp "$FIXTURE_RUSTUP" "$CARGO_HOME/bin/rustup"
chmod +x "$CARGO_HOME/bin/rustup"
printf '%s\n' "${FIXTURE_STABLE_RELEASE:-1.98.0}" > "$RUSTUP_HOME/fixture-toolchains/stable"
printf '%s\n' 'default_toolchain = "stable"' 'profile = "minimal"' > "$RUSTUP_HOME/settings.toml"
INIT
EOF
chmod +x "$tools/curl" "$work/rustup-fixture"
# Bootstrap invokes sh explicitly; other tools cannot leak Rust from host PATH.
ln -s /bin/sh "$tools/sh"
export PATH="$tools" FIXTURE_RUSTUP="$work/rustup-fixture"
export CARGO_HOME="$work/cargo 'home" RUSTUP_HOME="$work/rustup home"
mkdir -p "$CARGO_HOME" "$RUSTUP_HOME"

# Every individual argument is part of the oracle, including clauses that a
# short-circuit list under set -e would otherwise silently ignore.
for wrong_argument in 1 2 3 4 5 6 7 8 9 10 11 12 13 14; do
    set -- --proto '=https' --tlsv1.2 -fsSL --retry 5 --retry-delay 5 \
        --connect-timeout 30 --max-time 120 https://sh.rustup.rs -o "$work/bad-init"
    # Rebuild the list with argument number $wrong_argument replaced: each pass
    # appends one argument and shifts the original off the front.
    position=0
    for argument do
        position=$((position + 1))
        if [ "$position" = "$wrong_argument" ]; then
            set -- "$@" wrong
        else
            set -- "$@" "$argument"
        fi
        shift
    done
    if "$tools/curl" "$@" > "$work/wrong-curl.log" 2>&1; then
        printf 'error: curl oracle accepted incorrect argument %s\n' "$wrong_argument" >&2
        exit 1
    fi
    grep -Fq 'unexpected curl arguments' "$work/wrong-curl.log"
    [ ! -e "$work/bad-init" ]
done

# The helper takes no argument; the compiler metadata a 1.x helper read is
# refused rather than silently ignored.
printf '%s\n' 'release: 1.93.0' > "$work/old-metadata"
if sh "$script_dir/install_rust.sh" "$work/old-metadata" > "$work/usage.log" 2>&1; then
    printf '%s\n' 'error: an argument was accepted' >&2; exit 1
fi
grep -Fxq 'error: usage: install_rust.sh' "$work/usage.log"

# A Rust on PATH at or above the floor is ready as it is: the helper prints
# its release and creates nothing, not even the Rust homes or a lock. The
# releases are the floor itself, later ones (numerically, not as text), and
# pre-release or build suffixes, which Cargo also ignores.
ready_bin=$work/ready-bin
mkdir -p "$ready_bin"
printf '%s\n' '#!/bin/sh' 'printf "%s\n" "cargo (unmanaged)"' > "$ready_bin/cargo"
chmod +x "$ready_bin/cargo"
for release in 1.95.0 1.98.1 1.100.0 2.0.0 1.96.0-nightly 1.95.0-beta.2 1.95.0+distro; do
    printf '#!/bin/sh\nprintf "rustc %s\\nbinary: rustc\\nrelease: %s\\n" "%s" "%s"\n' \
        '%s' '%s' "$release" "$release" > "$ready_bin/rustc"
    chmod +x "$ready_bin/rustc"
    absent=$work/absent-homes
    PATH="$ready_bin:$tools" CARGO_HOME="$absent/cargo" RUSTUP_HOME="$absent/rustup" \
        sh "$script_dir/install_rust.sh" > "$work/ready.log" 2>&1 ||
        { cat "$work/ready.log" >&2; printf 'error: ready Rust %s was refused\n' "$release" >&2; exit 1; }
    grep -Fxq "Rust/Cargo $release is ready." "$work/ready.log"
    [ ! -e "$absent" ] || { printf 'error: ready Rust %s touched the Rust homes\n' "$release" >&2; exit 1; }
done
# Below the floor, or a release that cannot be read, is not ready. With no
# rustup to add stable beside it, the helper refuses and leaves it alone.
for release in 1.94.0 1.9.0 0.99.0 1.94.9-nightly 1.95 1.95.0.1 x.y.z none; do
    if [ "$release" = none ]; then
        printf '%s\n' '#!/bin/sh' 'printf "%s\n" "rustc 1.98.0" "binary: rustc"' > "$ready_bin/rustc"
    else
        printf '#!/bin/sh\nprintf "release: %s\\n" "%s"\n' '%s' "$release" > "$ready_bin/rustc"
    fi
    chmod +x "$ready_bin/rustc"
    old_homes=$work/old-homes
    if PATH="$ready_bin:$tools" CARGO_HOME="$old_homes/cargo" RUSTUP_HOME="$old_homes/rustup" \
        sh "$script_dir/install_rust.sh" > "$work/old.log" 2>&1; then
        printf 'error: Rust release %s was accepted\n' "$release" >&2; exit 1
    fi
    grep -Fq 'installed without rustup but is not a working Rust 1.95 or newer with Cargo' "$work/old.log"
    if grep -Fq 'is ready' "$work/old.log"; then
        printf 'error: Rust release %s was reported ready\n' "$release" >&2; exit 1
    fi
    [ ! -e "$old_homes/cargo/bin/rustup" ]
    rm -rf "$old_homes"
done
# A rustc with no cargo beside it is not ready either.
printf '#!/bin/sh\nprintf "release: 1.98.0\\n"\n' > "$ready_bin/rustc"
chmod +x "$ready_bin/rustc"
rm "$ready_bin/cargo"
if PATH="$ready_bin:$tools" CARGO_HOME="$work/no-cargo/cargo" RUSTUP_HOME="$work/no-cargo/rustup" \
    sh "$script_dir/install_rust.sh" > "$work/no-cargo.log" 2>&1; then
    printf '%s\n' 'error: rustc without cargo was accepted' >&2; exit 1
fi
grep -Fq 'installed without rustup' "$work/no-cargo.log"

# Orphaned Rustup state is not a fresh installation. Refuse before the official
# bootstrap can change a saved default or profile in an existing custom home.
printf '%s\n' 'default_toolchain = "stable"' 'profile = "complete"' > "$RUSTUP_HOME/settings.toml"
cp "$RUSTUP_HOME/settings.toml" "$work/settings-before"
if sh "$script_dir/install_rust.sh" > "$work/orphaned.log" 2>&1; then
    printf '%s\n' 'error: orphaned Rustup configuration was overwritten' >&2; exit 1
fi
grep -Fq 'default compiler and profile remain unchanged' "$work/orphaned.log"
[ "$(cat "$RUSTUP_HOME/settings.toml")" = "$(cat "$work/settings-before")" ]
[ ! -e "$CARGO_HOME/bin/rustup" ]
rm "$RUSTUP_HOME/settings.toml"

# No Rust and no rustup: the official installer runs with stable as the
# default toolchain, and that stable is used as it is.
(
    unset HOME
    sh "$script_dir/install_rust.sh"
) > "$work/fresh.log"
[ -x "$CARGO_HOME/bin/rustup" ]
grep -Fq 'Installing current stable Rust/Cargo with rustup' "$work/fresh.log"
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/fresh.log"
for proxy in rustup cargo rustc; do
    [ -L "$CARGO_HOME/bin/$proxy" ] || [ "$proxy" = rustup ]
done
if grep -Fq 'toolchain install' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: the stable rustup just installed was installed again' >&2; exit 1
fi

# Ready in Cargo's home: a rerun prints the release and changes nothing.
cp "$RUSTUP_HOME/settings.toml" "$work/settings-before"
: > "$CARGO_HOME/calls"
sh "$script_dir/install_rust.sh" > "$work/repeat.log"
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/repeat.log"
[ "$(cat "$CARGO_HOME/calls")" = "$(printf '%s\n' \
    'run stable rustc -vV [auto-install=0 cwd=/]' 'run stable cargo --version [auto-install=0 cwd=/]')" ]
# A relative Cargo home reads the same from /.
: > "$CARGO_HOME/calls"
(cd "$work" && CARGO_HOME="cargo 'home" sh "$script_dir/install_rust.sh") > "$work/relative.log"
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/relative.log"
if grep -Fq 'toolchain install' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: a relative Cargo home was not read' >&2; exit 1
fi
cmp "$work/settings-before" "$RUSTUP_HOME/settings.toml"
for leftover in "$CARGO_HOME"/.cerulion-* "$RUSTUP_HOME"/.cerulion-*; do
    [ ! -e "$leftover" ] || { printf 'error: a ready rerun left %s\n' "$leftover" >&2; exit 1; }
done

# Cargo's home decides when it holds both programs, since the installer puts
# it first on PATH: a ready Rust on PATH does not hide an older default there.
printf '#!/bin/sh\nprintf "release: 1.98.1\\n"\n' > "$ready_bin/rustc"
printf '#!/bin/sh\nprintf "cargo 1.98.1\\n"\n' > "$ready_bin/cargo"
chmod +x "$ready_bin/rustc" "$ready_bin/cargo"
mkdir -p "$RUSTUP_HOME/fixture-toolchains"
printf '%s\n' 1.80.0 > "$RUSTUP_HOME/fixture-toolchains/1.80.0"
printf '%s\n' 'default_toolchain = "1.80.0"' > "$RUSTUP_HOME/settings.toml"
PATH="$ready_bin:$tools" sh "$script_dir/install_rust.sh" > "$work/shadowed.log"
if grep -Fq 'Rust/Cargo 1.98.1 is ready.' "$work/shadowed.log"; then
    printf '%s\n' 'error: the Rust on PATH hid the older default in Cargo home' >&2; exit 1
fi
grep -Fq 'your default toolchain was kept' "$work/shadowed.log"
# Once Cargo's home is on PATH, PATH order decides.
PATH="$ready_bin:$CARGO_HOME/bin:$tools" sh "$script_dir/install_rust.sh" > "$work/path-order.log"
grep -Fxq 'Rust/Cargo 1.98.1 is ready.' "$work/path-order.log"
# A cargo that does not run is not a ready Rust.
printf '%s\n' '#!/bin/sh' 'exit 1' > "$ready_bin/cargo"
if PATH="$ready_bin:$tools" CARGO_HOME="$work/broken-cargo/cargo" RUSTUP_HOME="$work/broken-cargo/rustup" \
    sh "$script_dir/install_rust.sh" > "$work/broken-cargo.log" 2>&1; then
    printf '%s\n' 'error: a cargo that does not run was accepted' >&2; exit 1
fi
grep -Fq 'installed without rustup' "$work/broken-cargo.log"
# A rustc in Cargo's home without a cargo beside it does not count: a
# complete Rust on PATH answers instead, and without one the helper goes on to
# add the missing proxy.
printf '%s\n' 'default_toolchain = "stable"' > "$RUSTUP_HOME/settings.toml"
rm "$CARGO_HOME/bin/cargo"
printf '#!/bin/sh\nprintf "cargo 1.98.1\\n"\n' > "$ready_bin/cargo"
PATH="$ready_bin:$tools" sh "$script_dir/install_rust.sh" > "$work/rustc-only-path.log"
grep -Fxq 'Rust/Cargo 1.98.1 is ready.' "$work/rustc-only-path.log"
: > "$CARGO_HOME/calls"
sh "$script_dir/install_rust.sh" > "$work/rustc-only.log"
grep -Fxq 'default' "$CARGO_HOME/calls"
[ -L "$CARGO_HOME/bin/cargo" ]
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/rustc-only.log"

# An existing default older than the floor is kept. Stable is added beside it
# and the helper says how to select it, in one line.
mkdir -p "$RUSTUP_HOME/fixture-toolchains"
printf '%s\n' 1.80.0 > "$RUSTUP_HOME/fixture-toolchains/1.80.0"
rm "$RUSTUP_HOME/fixture-toolchains/stable"
printf '%s\n' 'default_toolchain = "1.80.0"' 'profile = "minimal"' > "$RUSTUP_HOME/settings.toml"
cp "$RUSTUP_HOME/settings.toml" "$work/settings-before"
: > "$CARGO_HOME/calls"
sh "$script_dir/install_rust.sh" > "$work/old-default.log"
grep -Fxq 'toolchain install stable --profile minimal --no-self-update' "$CARGO_HOME/calls"
if grep -Fq 'default stable' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: an existing default was replaced' >&2; exit 1
fi
cmp "$work/settings-before" "$RUSTUP_HOME/settings.toml"
# shellcheck disable=SC2016  # the assertion is the literal text of the helper
grep -Fxq 'Rust 1.95 or newer is now installed as `stable`, and your default toolchain was kept. Select stable with `rustup default stable`, or for one build with `RUSTUP_TOOLCHAIN=stable cerulion node build <node>`.' \
    "$work/old-default.log"
if grep -Fq 'is ready' "$work/old-default.log"; then
    printf '%s\n' 'error: an older default was reported ready' >&2; exit 1
fi
RUSTUP_TOOLCHAIN=stable "$CARGO_HOME/bin/rustc" -vV | grep -Fxq 'release: 1.98.0'

# No default at all: the stable already installed is reused, and becomes the
# default. The rest of the settings stay.
printf '%s\n' 'profile = "minimal"' > "$RUSTUP_HOME/settings.toml"
: > "$CARGO_HOME/calls"
sh "$script_dir/install_rust.sh" > "$work/no-default.log"
grep -Fxq 'default stable' "$CARGO_HOME/calls"
if grep -Fq 'toolchain install' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: an installed stable at the floor was installed again' >&2; exit 1
fi
[ "$(cat "$RUSTUP_HOME/settings.toml")" = "$(printf '%s\n' 'profile = "minimal"' 'default_toolchain = "stable"')" ]
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/no-default.log"

# An installed stable older than the floor is updated to current stable.
printf '%s\n' 1.80.0 > "$RUSTUP_HOME/fixture-toolchains/stable"
: > "$CARGO_HOME/calls"
sh "$script_dir/install_rust.sh" > "$work/stale-stable.log"
grep -Fxq 'toolchain install stable --profile minimal --no-self-update' "$CARGO_HOME/calls"
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/stale-stable.log"

rm "$RUSTUP_HOME/fixture-toolchains/stable"
if FIXTURE_FAIL_INSTALL=1 sh "$script_dir/install_rust.sh" > "$work/fail.log" 2>&1; then
    printf '%s\n' 'error: failed toolchain install was accepted' >&2; exit 1
fi
grep -Fq 'could not install stable Rust/Cargo' "$work/fail.log"
if FIXTURE_STABLE_RELEASE=1.94.0 sh "$script_dir/install_rust.sh" > "$work/old-stable.log" 2>&1; then
    printf '%s\n' 'error: a stable older than the floor was accepted' >&2; exit 1
fi
grep -Fq 'stable Rust from rustup is 1.94.0, older than Rust 1.95' "$work/old-stable.log"
rm -rf "$CARGO_HOME" "$RUSTUP_HOME"
mkdir -p "$CARGO_HOME" "$RUSTUP_HOME"
if FIXTURE_FAIL_DOWNLOAD=1 sh "$script_dir/install_rust.sh" > "$work/download.log" 2>&1; then
    printf '%s\n' 'error: failed rustup download was accepted' >&2; exit 1
fi
grep -Fq 'could not download the official rustup installer' "$work/download.log"
if FIXTURE_FAIL_BOOTSTRAP=1 sh "$script_dir/install_rust.sh" > "$work/init-fail.log" 2>&1; then
    printf '%s\n' 'error: failed rustup bootstrap was accepted' >&2; exit 1
fi
grep -Fq 'Rust/Cargo installation failed' "$work/init-fail.log"
printf '%s\n' '#!/bin/sh' > "$tools/cargo"
chmod +x "$tools/cargo"
if sh "$script_dir/install_rust.sh" > "$work/unmanaged.log" 2>&1; then
    printf '%s\n' 'error: unmanaged Cargo installation was replaced' >&2; exit 1
fi
grep -Fq 'installed without rustup' "$work/unmanaged.log"
rm "$tools/cargo"
mkdir -p "$CARGO_HOME/bin"
ln -s "$work/missing-unmanaged-cargo" "$CARGO_HOME/bin/cargo"
if sh "$script_dir/install_rust.sh" > "$work/unmanaged-link.log" 2>&1; then
    printf '%s\n' 'error: unmanaged dangling Cargo symlink was replaced' >&2; exit 1
fi
grep -Fq 'installed without rustup' "$work/unmanaged-link.log"
[ -L "$CARGO_HOME/bin/cargo" ]
[ ! -e "$CARGO_HOME/bin/cargo" ]
rm "$CARGO_HOME/bin/cargo"

# PATH-only rustup with an empty custom Cargo home must expose working proxies
# there, while retaining existing Rustup settings and the external executable.
cp "$work/rustup-fixture" "$tools/rustup"
printf '%s\n' 'default_toolchain = "stable"' > "$RUSTUP_HOME/settings.toml"
(
    unset HOME
    sh "$script_dir/install_rust.sh"
) > "$work/path-rustup.log"
for proxy in rustup cargo rustc; do
    [ -L "$CARGO_HOME/bin/$proxy" ]
done
RUSTUP_TOOLCHAIN=stable "$CARGO_HOME/bin/cargo" --version | grep -Fxq 'cargo 1.98.0 (fixture)'
RUSTUP_TOOLCHAIN=stable "$CARGO_HOME/bin/rustc" -vV | grep -Fxq 'release: 1.98.0'
[ "$(cat "$RUSTUP_HOME/settings.toml")" = 'default_toolchain = "stable"' ]
grep -Fxq 'Rust/Cargo 1.98.0 is ready.' "$work/path-rustup.log"
# Existing paths are preserved even when they prevent usable proxies, and a
# refused run sets no default. Without a default the rustc proxy cannot run,
# so the existing Rust is not ready and the proxies are checked.
printf '%s\n' 'profile = "minimal"' > "$RUSTUP_HOME/settings.toml"
rustup_proxy_target=$(readlink "$CARGO_HOME/bin/cargo")
rm "$CARGO_HOME/bin/cargo"
for existing_cargo in 'cargo 1.97.0 (unmanaged)' 'cargo 1.98.0 (different build)'; do
    printf '#!/bin/sh\nprintf "%%s\\n" "%s"\n' "$existing_cargo" > "$CARGO_HOME/bin/cargo"
    chmod +x "$CARGO_HOME/bin/cargo"
    cp "$CARGO_HOME/bin/cargo" "$work/unmanaged-cargo-before"
    if sh "$script_dir/install_rust.sh" > "$work/wrong-proxy.log" 2>&1; then
        printf '%s\n' 'error: wrong Cargo build in existing proxy was accepted' >&2; exit 1
    fi
    grep -Fq "cargo proxy does not match stable Rust's Cargo; existing files were preserved" "$work/wrong-proxy.log"
    [ "$(cat "$CARGO_HOME/bin/cargo")" = "$(cat "$work/unmanaged-cargo-before")" ]
    [ "$(cat "$RUSTUP_HOME/settings.toml")" = 'profile = "minimal"' ]
done
rm "$CARGO_HOME/bin/cargo"
ln -s "$rustup_proxy_target" "$CARGO_HOME/bin/cargo"
if FIXTURE_PROXY_RELEASE=1.97.0 sh "$script_dir/install_rust.sh" > "$work/wrong-rustc.log" 2>&1; then
    printf '%s\n' 'error: a rustc proxy that selects another release was accepted' >&2; exit 1
fi
grep -Fq "rustc proxy does not select stable Rust; existing files were preserved" "$work/wrong-rustc.log"
[ "$(cat "$RUSTUP_HOME/settings.toml")" = 'profile = "minimal"' ]
rm "$CARGO_HOME/bin/cargo"
ln -s "$work/nonexistent-cargo" "$CARGO_HOME/bin/cargo"
if sh "$script_dir/install_rust.sh" > "$work/broken-proxy.log" 2>&1; then
    printf '%s\n' 'error: dangling Cargo proxy was replaced or accepted' >&2; exit 1
fi
[ -L "$CARGO_HOME/bin/cargo" ]
[ ! -e "$CARGO_HOME/bin/cargo" ]
grep -Fq 'cargo proxy is unusable; existing files were preserved' "$work/broken-proxy.log"
[ "$(cat "$RUSTUP_HOME/settings.toml")" = 'profile = "minimal"' ]
rm -rf "$CARGO_HOME" "$RUSTUP_HOME"
mkdir -p "$CARGO_HOME" "$RUSTUP_HOME"
rm "$tools/rustup"

# Exercise the real installer transaction: a failed toolchain install cannot
# replace any of the existing sibling binaries, and success activates all three.
# The installers below run on a PATH of plain utilities, so a Rust the host has
# installed cannot answer for the fixture.
PATH=$original_path
export PATH
con_bin=$work/concurrent-tools
mkdir -p "$con_bin"
# GNU tar invokes gzip for -z; libarchive can decompress internally.
for utility in awk grep sed mktemp rm chmod cp cat dirname mkdir sleep sh uname tar gzip basename stat ps date wc tr ls mv cmp shasum sha256sum; do
    utility_path=$(command -v "$utility" 2>/dev/null || :)
    [ -z "$utility_path" ] || ln -s "$utility_path" "$con_bin/$utility"
done
FIXTURE_REAL_LN=$(command -v ln)
FIXTURE_REAL_CURL=$(command -v curl)
export FIXTURE_REAL_LN FIXTURE_REAL_CURL
export FIXTURE_BOOTSTRAP_CURL="$tools/curl"
cat > "$con_bin/ln" <<'EOF'
#!/bin/sh
if "$FIXTURE_REAL_LN" "$@"; then exit 0; else status=$?; fi
: > "$FIXTURE_CASE/wait.$FIXTURE_ROLE"
exit "$status"
EOF
cat > "$con_bin/curl" <<'EOF'
#!/bin/sh
case "$*" in
    *https://sh.rustup.rs*) exec "$FIXTURE_BOOTSTRAP_CURL" "$@" ;;
    *) exec "$FIXTURE_REAL_CURL" "$@" ;;
esac
EOF
chmod +x "$con_bin/ln" "$con_bin/curl"
# The ln stand-in records lock contention under the case directory.
FIXTURE_CASE=$work/transaction-case FIXTURE_ROLE=transaction
export FIXTURE_CASE FIXTURE_ROLE
mkdir -p "$FIXTURE_CASE"
case "$(uname -s):$(uname -m)" in
    Darwin:arm64) target=aarch64-apple-darwin ;;
    Darwin:x86_64) target=x86_64-apple-darwin ;;
    Linux:x86_64) target=x86_64-unknown-linux-gnu ;;
    Linux:aarch64) target=aarch64-unknown-linux-gnu ;;
    *) printf '%s\n' 'error: unsupported test host' >&2; exit 1 ;;
esac
stem=cerulion-0.2.0-$target
stage=$work/$stem
mkdir -p "$stage" "$work/dist/v0.2.0" "$work/installed" "$CARGO_HOME/bin"
cp "$work/rustup-fixture" "$CARGO_HOME/bin/rustup"
cp "$script_dir/install_rust.sh" "$stage/install_rust.sh"
for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
    printf '#!/bin/sh\nprintf "new %s\\n"\n' "$binary" > "$stage/$binary"
    chmod +x "$stage/$binary"
    printf 'old %s\n' "$binary" > "$work/installed/$binary"
done
pack_archive() {
    rm -f "$work/dist/v0.2.0/$stem.tar.gz"
    tar -czf "$work/dist/v0.2.0/$stem.tar.gz" -C "$work" "$stem"
    if command -v sha256sum >/dev/null 2>&1; then
        (cd "$work/dist/v0.2.0" && sha256sum "$stem.tar.gz" > "$stem.tar.gz.sha256")
    else
        (cd "$work/dist/v0.2.0" && shasum -a 256 "$stem.tar.gz" > "$stem.tar.gz.sha256")
    fi
}
pack_archive
if FIXTURE_FAIL_INSTALL=1 PATH="$con_bin" sh "$script_dir/install.sh" --version v0.2.0 \
    --base-url "file://$work/dist" --dir "$work/installed" > "$work/transaction-fail.log" 2>&1; then
    printf '%s\n' 'error: failed Rust setup activated the installer' >&2; exit 1
fi
grep -Fq 'Rust/Cargo setup failed' "$work/transaction-fail.log"
for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
    [ "$(cat "$work/installed/$binary")" = "old $binary" ]
done
(
    unset HOME
    CERULION_INSTALL_DIR="$work/installed" PATH="$con_bin" sh "$script_dir/install.sh" \
        --version v0.2.0 --base-url "file://$work/dist"
) > "$work/transaction-pass.log" 2>&1 ||
    { cat "$work/transaction-pass.log" >&2; exit 1; }
for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
    cmp "$stage/$binary" "$work/installed/$binary"
done
grep -Fq 'Rust/Cargo 1.98.0 is ready.' "$work/transaction-pass.log"
activation=$(sed -n 's/^Add tools to PATH: //p' "$work/transaction-pass.log")
[ -n "$activation" ]
# The size the user is told about before rustup starts fetching. Provisioning
# multiplies what a first install downloads, and the archive alone gives no
# hint of it. The whole sentence is pinned, not the number: the step runs on
# every install that carries the bootstrap, including one where a usable Rust
# is already there and nothing is fetched, so the claim is a conditional one.
grep -Fq 'where the Rust it needs is not already installed this downloads about 140 MB' \
    "$work/transaction-pass.log"
(
    PATH=$con_bin
    eval "$activation"
    [ "$(command -v cerulion)" = "$work/installed/cerulion" ]
    [ "$(command -v rustup)" = "$CARGO_HOME/bin/rustup" ]
    [ "$(command -v cargo)" = "$CARGO_HOME/bin/cargo" ]
    cargo --version | grep -Fxq 'cargo 1.98.0 (fixture)'
)

# A 1.x archive pairs its own helper with rustc-version.txt, which that helper
# takes as its one argument; the installer still passes it. This stand-in for
# the old helper records what it was given.
cp "$stage/install_rust.sh" "$work/install_rust.sh.current"
printf '%s\n' 'release: 1.93.0' > "$stage/rustc-version.txt"
cat > "$stage/install_rust.sh" <<'EOF'
#!/bin/sh
set -eu
[ "$#" -eq 1 ] || { printf '%s\n' 'usage: install_rust.sh RUSTC_VERSION_FILE' >&2; exit 1; }
[ "$(cat "$1")" = 'release: 1.93.0' ] || exit 1
printf '%s\n' 'legacy helper ran with its metadata'
EOF
pack_archive
PATH="$con_bin" sh "$script_dir/install.sh" --version v0.2.0 --base-url "file://$work/dist" \
    --dir "$work/installed" > "$work/legacy.log" 2>&1 ||
    { cat "$work/legacy.log" >&2; printf '%s\n' 'error: a 1.x archive no longer installs' >&2; exit 1; }
grep -Fxq 'legacy helper ran with its metadata' "$work/legacy.log"
# The metadata without a helper is an incomplete archive, never the path for
# archives that carry no Rust setup at all.
rm "$stage/install_rust.sh"
pack_archive
if PATH="$con_bin" sh "$script_dir/install.sh" --version v0.2.0 --base-url "file://$work/dist" \
    --dir "$work/installed" > "$work/partial.log" 2>&1; then
    printf '%s\n' 'error: compiler metadata without its helper was accepted' >&2; exit 1
fi
grep -Fq 'archive is missing part of its Rust/Cargo setup' "$work/partial.log"
rm "$stage/rustc-version.txt"
cp "$work/install_rust.sh.current" "$stage/install_rust.sh"
pack_archive

# Real competing installers pause immediately before the fixture mutates Rust.
# A failed real hardlink reports contention, so assertions need no guessed delay.
con_pids=
con_cleanup() {
    for con_pid in $con_pids; do kill -TERM "$con_pid" 2>/dev/null || :; done
    if [ -n "${con_case:-}" ]; then
        for con_pid_file in "$con_case"/child.*; do
            [ -f "$con_pid_file" ] || continue
            kill -TERM "$(cat "$con_pid_file")" 2>/dev/null || :
        done
    fi
    sleep 0.2
    for con_pid in $con_pids; do
        kill -KILL "$con_pid" 2>/dev/null || :
        wait "$con_pid" 2>/dev/null || :
    done
    rm -rf "$work"
}
trap con_cleanup EXIT
cat > "$work/concurrent-barrier" <<'EOF'
#!/bin/sh
set -eu
printf '%s\n' "$$" > "$FIXTURE_CASE/child.$FIXTURE_ROLE"
trap 'rm -f "$FIXTURE_CASE/child.$FIXTURE_ROLE"' EXIT
trap 'exit 143' TERM
printf '%s\n' "$FIXTURE_ROLE" >> "$FIXTURE_CASE/mutations"
: > "$FIXTURE_CASE/entered.$FIXTURE_ROLE"
attempt=0
while [ ! -f "$FIXTURE_CASE/release.$FIXTURE_ROLE" ]; do
    attempt=$((attempt + 1))
    [ "$attempt" -lt 400 ] || exit 81
    sleep 0.05
done
EOF
con_wait_file() {
    con_attempt=0
    while [ ! -f "$1" ]; do
        if [ "$1" = "$con_case/wait.b" ] && [ -f "$con_case/entered.b" ]; then
            printf "error: competing installer mutated Rust before the holder was released\n" >&2
            exit 1
        fi
        con_attempt=$((con_attempt + 1))
        if [ "$con_attempt" -ge 300 ]; then
            printf 'error: concurrent installer did not reach %s\n' "$1" >&2
            cat "$con_case"/*.log >&2
            exit 1
        fi
        sleep 0.05
    done
}
con_spawn() {
    mkdir -p "$2" "$3"
    PATH="$con_bin" CARGO_HOME="$2" RUSTUP_HOME="$3" \
        FIXTURE_CASE="$con_case" FIXTURE_ROLE="$1" FIXTURE_BARRIER="$work/concurrent-barrier" \
        sh "$script_dir/install.sh" --version v0.2.0 --base-url "file://$work/dist" \
        --dir "$4" > "$con_case/$1.log" 2>&1 &
    con_last=$!
    con_pids="$con_pids $con_last"
}
con_join() {
    con_attempt=0
    while kill -0 "$1" 2>/dev/null; do
        con_attempt=$((con_attempt + 1))
        [ "$con_attempt" -lt 400 ] || { printf 'error: concurrent child hung\n' >&2; exit 1; }
        sleep 0.05
    done
    if wait "$1"; then con_status=0; else con_status=$?; fi
    if [ "$2" = success ]; then
        [ "$con_status" -eq 0 ] || { cat "$con_case"/*.log >&2; exit 1; }
    else
        [ "$con_status" -ne 0 ]
    fi
}
for con_mode in same-destination fresh-shared split-cargo identical-homes cancel cancel-cargo; do
    con_case=$work/concurrent-$con_mode
    mkdir -p "$con_case/a" "$con_case/b" "$con_case/cargo" "$con_case/rustup"
    con_rustup_a=$con_case/rustup
    con_cargo_b=$con_case/cargo con_rustup_b=$con_case/rustup con_dest_b=$con_case/b
    if [ "$con_mode" = same-destination ]; then
        con_cargo_b=$con_case/cargo-b con_rustup_b=$con_case/rustup-b con_dest_b=$con_case/a
    elif [ "$con_mode" = identical-homes ]; then
        con_rustup_a=$con_case/cargo con_rustup_b=$con_case/cargo
    elif [ "$con_mode" = cancel-cargo ]; then
        con_rustup_b=$con_case/rustup-b
    elif [ "$con_mode" = split-cargo ]; then
        con_cargo_b=$con_case/cargo-b
        cp "$work/rustup-fixture" "$con_bin/rustup"
        printf 'default_toolchain = "stable"\nprofile = "complete"\n' > "$con_case/rustup/settings.toml"
        cp "$con_case/rustup/settings.toml" "$con_case/settings-before"
    fi
    for con_dest in "$con_case/a" "$con_case/b"; do
        for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
            printf 'old %s\n' "$binary" > "$con_dest/$binary"
        done
    done
    con_spawn a "$con_case/cargo" "$con_rustup_a" "$con_case/a"; con_a=$con_last
    con_wait_file "$con_case/entered.a"
    con_spawn b "$con_cargo_b" "$con_rustup_b" "$con_dest_b"; con_b=$con_last
    con_wait_file "$con_case/wait.b"
    [ ! -e "$con_case/entered.b" ]
    [ "$(cat "$con_case/mutations")" = a ]
    for con_dest in "$con_case/a" "$con_case/b"; do
        for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
            [ "$(cat "$con_dest/$binary")" = "old $binary" ]
        done
    done
    if [ "$con_mode" = cancel ] || [ "$con_mode" = cancel-cargo ]; then
        kill -TERM "$con_b"; con_join "$con_b" failure
        [ ! -e "$con_case/entered.b" ]
        [ -f "$con_case/rustup/.cerulion-rustup-setup.lock" ]
        [ -f "$con_case/cargo/.cerulion-cargo-setup.lock" ]
        if [ "$con_mode" = cancel-cargo ]; then
            [ ! -e "$con_rustup_b/.cerulion-rustup-setup.lock" ]
        fi
        kill -0 "$con_a"
        kill -TERM "$con_a"; con_join "$con_a" failure
        for con_dest in "$con_case/a" "$con_case/b"; do
            for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
                [ "$(cat "$con_dest/$binary")" = "old $binary" ]
            done
        done
    else
        : > "$con_case/release.a"; : > "$con_case/release.b"
        con_join "$con_a" success; con_join "$con_b" success
        for con_dest in "$con_case/a" "$con_dest_b"; do
            for binary in cerulion cerulion-netd cerulion-connectd cerulion-remoted; do
                cmp "$stage/$binary" "$con_dest/$binary"
            done
            [ "$(cat "$con_dest/.cerulion-provenance.json")" = \
                '{"method":"install.sh","version":"0.2.0"}' ]
        done
        if [ "$con_mode" = same-destination ]; then
            [ "$(cat "$con_case/mutations")" = "$(printf 'a\nb')" ]
        else
            [ "$(cat "$con_case/mutations")" = a ]
        fi
    fi
    for con_leftover in "$con_case"/*/.cerulion-* "$con_case"/child.*; do
        case "$con_leftover" in
            */.cerulion-provenance.json)
                # A finished install leaves its marker; a cancelled one must not.
                case "$con_mode" in cancel | cancel-cargo) ;; *) continue ;; esac
                ;;
        esac
        [ ! -e "$con_leftover" ] || { printf 'error: leaked setup state %s\n' "$con_leftover" >&2; exit 1; }
    done
    [ "$con_mode" != split-cargo ] || cmp "$con_case/settings-before" "$con_case/rustup/settings.toml"
    rm -f "$con_bin/rustup"
    con_pids=
done

# Refuse foreign locks without stealing or damaging them. Accelerate only the
# one-second lock retry, preserving a bounded real-process timeout check.
FIXTURE_REAL_SLEEP=$(command -v sleep)
export FIXTURE_REAL_SLEEP
rm "$con_bin/sleep"
cat > "$con_bin/sleep" <<'EOF'
#!/bin/sh
if [ "$*" = 1 ]; then exec "$FIXTURE_REAL_SLEEP" 0.01; fi
exec "$FIXTURE_REAL_SLEEP" "$@"
EOF
chmod +x "$con_bin/sleep"
for lock_home in rustup cargo; do
    for lock_kind in directory symlink stale; do
        con_case=$work/refuse-$lock_home-$lock_kind
        mkdir -p "$con_case/cargo" "$con_case/rustup"
        foreign_lock=$con_case/$lock_home/.cerulion-$lock_home-setup.lock
        case "$lock_kind" in
            directory) mkdir "$foreign_lock"; printf 'preserve\n' > "$foreign_lock/owner" ;;
            symlink) ln -s "$con_case/missing-owner" "$foreign_lock" ;;
            stale) printf '999999999\n' > "$foreign_lock" ;;
        esac
        if PATH="$con_bin" CARGO_HOME="$con_case/cargo" RUSTUP_HOME="$con_case/rustup" \
            FIXTURE_CASE="$con_case" FIXTURE_ROLE=refusal \
            sh "$script_dir/install_rust.sh" > "$con_case/refusal.log" 2>&1; then
            printf 'error: accepted foreign %s lock\n' "$lock_kind" >&2; exit 1
        fi
        case "$lock_kind" in
            directory) [ "$(cat "$foreign_lock/owner")" = preserve ] ;;
            symlink) [ "$(readlink "$foreign_lock")" = "$con_case/missing-owner" ] ;;
            stale) [ "$(cat "$foreign_lock")" = 999999999 ] ;;
        esac
        if [ "$lock_kind" = stale ]; then
            grep -Fq 'timed out waiting for Rust setup lock' "$con_case/refusal.log"
        else
            grep -Fq 'Rust setup lock is not a regular file' "$con_case/refusal.log"
        fi
        for con_leftover in "$con_case"/*/.cerulion-setup-owner.*; do
            [ ! -e "$con_leftover" ]
        done
        for con_leftover in "$con_case"/*/.cerulion-*-setup.lock; do
            [ "$con_leftover" = "$foreign_lock" ] || [ ! -e "$con_leftover" ]
        done
        [ ! -e "$con_case/cargo/bin/rustup" ]
    done
done
printf '%s\n' 'Rust/Cargo bootstrap tests passed'
