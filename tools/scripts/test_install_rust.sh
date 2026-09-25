#!/bin/sh
set -eu
original_path=$PATH
script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-rust-test.XXXXXX")
trap 'rm -rf "$work"' EXIT
tools=$work/tools
mkdir -p "$tools"
for utility in awk grep sed mktemp rm chmod cp cat dirname mkdir ln sleep; do
    ln -s "$(command -v "$utility")" "$tools/$utility"
done
cat > "$work/metadata" <<'EOF'
rustc 1.93.0 (111111111 2026-01-01)
commit-hash: 1111111111111111111111111111111111111111
release: 1.93.0
EOF
cat > "$work/rustup-fixture" <<'EOF'
#!/bin/sh
set -eu
case "${0##*/}" in
    rustc) [ "$*" = -vV ]; set -- run "${RUSTUP_TOOLCHAIN:?}" rustc -vV ;;
    cargo) [ "$*" = --version ]; set -- run "${RUSTUP_TOOLCHAIN:?}" cargo --version ;;
esac
printf '%s\n' "$*" >> "$CARGO_HOME/calls"
case "$*" in
    'run 1.93.0 rustc -vV')
        [ -f "${FIXTURE_STATE:-$CARGO_HOME}/installed" ] || exit 1
        cat "$FIXTURE_METADATA"
        ;;
    'run 1.93.0 cargo --version')
        [ -f "${FIXTURE_STATE:-$CARGO_HOME}/installed" ] || exit 1
        printf 'cargo %s (fixture)\n' "${FIXTURE_CARGO_RELEASE:-1.93.0}"
        ;;
    'toolchain install 1.93.0 --profile minimal --no-self-update')
        [ "${FIXTURE_FAIL_INSTALL:-0}" = 0 ] || exit 1
        [ -z "${FIXTURE_BARRIER:-}" ] || sh "$FIXTURE_BARRIER"
        : > "${FIXTURE_STATE:-$CARGO_HOME}/installed"
        ;;
    *) printf 'unexpected rustup command: %s\n' "$*" >&2; exit 1 ;;
esac
EOF
# The fixture checks the exact HTTPS source and bootstrap arguments, then
# installs the hand-written rustup oracle into the requested Cargo home.
cat > "$tools/curl" <<'EOF'
#!/bin/sh
set -eu
if [ "$#" != 7 ] || [ "$1" != --proto ] || [ "$2" != '=https' ] ||
    [ "$3" != --tlsv1.2 ] || [ "$4" != -fsSL ] ||
    [ "$5" != https://sh.rustup.rs ] || [ "$6" != -o ]; then
    printf '%s\n' 'unexpected curl arguments' >&2
    exit 1
fi
[ "${FIXTURE_FAIL_DOWNLOAD:-0}" = 0 ] || exit 1
cat > "$7" <<'INIT'
#!/bin/sh
set -eu
[ "$*" = '-y --profile minimal --default-toolchain 1.93.0 --no-modify-path' ]
[ "${FIXTURE_FAIL_BOOTSTRAP:-0}" = 0 ] || exit 1
[ -z "${FIXTURE_BARRIER:-}" ] || sh "$FIXTURE_BARRIER"
mkdir -p "$CARGO_HOME/bin"
cp "$FIXTURE_RUSTUP" "$CARGO_HOME/bin/rustup"
chmod +x "$CARGO_HOME/bin/rustup"
: > "${FIXTURE_STATE:-$CARGO_HOME}/installed"
INIT
EOF
chmod +x "$tools/curl" "$work/rustup-fixture"
# Bootstrap invokes sh explicitly; other tools cannot leak Rust from host PATH.
ln -s /bin/sh "$tools/sh"
export PATH="$tools" FIXTURE_RUSTUP="$work/rustup-fixture" FIXTURE_METADATA="$work/metadata"
export CARGO_HOME="$work/cargo 'home" RUSTUP_HOME="$work/rustup home"
mkdir -p "$CARGO_HOME" "$RUSTUP_HOME"

# Every individual argument is part of the oracle, including clauses that a
# short-circuit list under set -e would otherwise silently ignore.
for wrong_argument in 1 2 3 4 5 6; do
    set -- --proto '=https' --tlsv1.2 -fsSL https://sh.rustup.rs -o "$work/bad-init"
    case "$wrong_argument" in
        1) set -- wrong "$2" "$3" "$4" "$5" "$6" "$7" ;;
        2) set -- "$1" wrong "$3" "$4" "$5" "$6" "$7" ;;
        3) set -- "$1" "$2" wrong "$4" "$5" "$6" "$7" ;;
        4) set -- "$1" "$2" "$3" wrong "$5" "$6" "$7" ;;
        5) set -- "$1" "$2" "$3" "$4" wrong "$6" "$7" ;;
        6) set -- "$1" "$2" "$3" "$4" "$5" wrong "$7" ;;
    esac
    if "$tools/curl" "$@" > "$work/wrong-curl.log" 2>&1; then
        printf 'error: curl oracle accepted incorrect argument %s\n' "$wrong_argument" >&2
        exit 1
    fi
    grep -Fq 'unexpected curl arguments' "$work/wrong-curl.log"
    [ ! -e "$work/bad-init" ]
done

# Orphaned Rustup state is not a fresh installation. Refuse before the official
# bootstrap can change a saved default or profile in an existing custom home.
printf '%s\n' 'default_toolchain = "stable"' 'profile = "complete"' > "$RUSTUP_HOME/settings.toml"
cp "$RUSTUP_HOME/settings.toml" "$work/settings-before"
if sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/orphaned.log" 2>&1; then
    printf '%s\n' 'error: orphaned Rustup configuration was overwritten' >&2; exit 1
fi
grep -Fq 'default compiler and profile remain unchanged' "$work/orphaned.log"
[ "$(cat "$RUSTUP_HOME/settings.toml")" = "$(cat "$work/settings-before")" ]
[ ! -e "$CARGO_HOME/bin/rustup" ]
rm "$RUSTUP_HOME/settings.toml"
(
    unset HOME
    sh "$script_dir/install_rust.sh" "$work/metadata"
) > "$work/fresh.log"
[ -x "$CARGO_HOME/bin/rustup" ]
grep -Fq 'Rust/Cargo 1.93.0 is ready' "$work/fresh.log"
if grep -Fq 'toolchain install' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: matching compiler was reinstalled' >&2; exit 1
fi

# An existing matching compiler is reused; a missing compiler is added without
# default/override flags. The strict oracle refuses either destructive flag.
printf '%s\n' 'default_toolchain = "stable"' > "$RUSTUP_HOME/settings.toml"
sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/repeat.log"
if grep -Fq 'toolchain install' "$CARGO_HOME/calls"; then
    printf '%s\n' 'error: matching compiler was reinstalled' >&2; exit 1
fi
rm "$CARGO_HOME/installed"
sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/add.log"
grep -Fxq 'toolchain install 1.93.0 --profile minimal --no-self-update' "$CARGO_HOME/calls"
[ "$(cat "$RUSTUP_HOME/settings.toml")" = 'default_toolchain = "stable"' ]

rm "$CARGO_HOME/installed"
if FIXTURE_FAIL_INSTALL=1 sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/fail.log" 2>&1; then
    printf '%s\n' 'error: failed toolchain install was accepted' >&2; exit 1
fi
grep -Fq 'could not install Rust/Cargo 1.93.0' "$work/fail.log"
sed 's/1111111111111111111111111111111111111111/2222222222222222222222222222222222222222/' "$work/metadata" > "$work/wrong-commit"
if sh "$script_dir/install_rust.sh" "$work/wrong-commit" > "$work/mismatch.log" 2>&1; then
    printf '%s\n' 'error: mismatched compiler fingerprint was accepted' >&2; exit 1
fi
grep -Fq 'does not match the release fingerprint' "$work/mismatch.log"
if FIXTURE_CARGO_RELEASE=1.97.0 sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/cargo-mismatch.log" 2>&1; then
    printf '%s\n' 'error: wrong Cargo release in selected toolchain was accepted' >&2; exit 1
fi
grep -Fq 'does not match the release fingerprint' "$work/cargo-mismatch.log"
printf '%s\n' 'release: ../bad' > "$work/malformed"
if sh "$script_dir/install_rust.sh" "$work/malformed" > "$work/malformed.log" 2>&1; then
    printf '%s\n' 'error: malformed compiler metadata was accepted' >&2; exit 1
fi
rm -rf "$CARGO_HOME"
mkdir -p "$CARGO_HOME"
rm "$RUSTUP_HOME/settings.toml"
if FIXTURE_FAIL_DOWNLOAD=1 sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/download.log" 2>&1; then
    printf '%s\n' 'error: failed rustup download was accepted' >&2; exit 1
fi
grep -Fq 'could not download the official rustup installer' "$work/download.log"
if FIXTURE_FAIL_BOOTSTRAP=1 sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/init-fail.log" 2>&1; then
    printf '%s\n' 'error: failed rustup bootstrap was accepted' >&2; exit 1
fi
grep -Fq 'Rust/Cargo installation failed' "$work/init-fail.log"
printf '%s\n' '#!/bin/sh' > "$tools/cargo"
chmod +x "$tools/cargo"
if sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/unmanaged.log" 2>&1; then
    printf '%s\n' 'error: unmanaged Cargo installation was replaced' >&2; exit 1
fi
grep -Fq 'installed without rustup' "$work/unmanaged.log"
rm "$tools/cargo"
mkdir -p "$CARGO_HOME/bin"
ln -s "$work/missing-unmanaged-cargo" "$CARGO_HOME/bin/cargo"
if sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/unmanaged-link.log" 2>&1; then
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
    sh "$script_dir/install_rust.sh" "$work/metadata"
) > "$work/path-rustup.log"
for proxy in rustup cargo rustc; do
    [ -L "$CARGO_HOME/bin/$proxy" ]
done
RUSTUP_TOOLCHAIN=1.93.0 "$CARGO_HOME/bin/cargo" --version | grep -Fxq 'cargo 1.93.0 (fixture)'
RUSTUP_TOOLCHAIN=1.93.0 "$CARGO_HOME/bin/rustc" -vV | grep -Fxq 'release: 1.93.0'
[ "$(cat "$RUSTUP_HOME/settings.toml")" = 'default_toolchain = "stable"' ]
# Existing paths are preserved even when they prevent usable proxies.
rm "$CARGO_HOME/bin/cargo"
for existing_cargo in 'cargo 1.97.0 (unmanaged)' 'cargo 1.93.0 (different build)'; do
    printf '#!/bin/sh\nprintf "%%s\\n" "%s"\n' "$existing_cargo" > "$CARGO_HOME/bin/cargo"
    chmod +x "$CARGO_HOME/bin/cargo"
    cp "$CARGO_HOME/bin/cargo" "$work/unmanaged-cargo-before"
    if sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/wrong-proxy.log" 2>&1; then
        printf '%s\n' 'error: wrong Cargo build in existing proxy was accepted' >&2; exit 1
    fi
    grep -Fq "cargo proxy does not match the release toolchain's Cargo; existing files were preserved" "$work/wrong-proxy.log"
    [ "$(cat "$CARGO_HOME/bin/cargo")" = "$(cat "$work/unmanaged-cargo-before")" ]
done
rm "$CARGO_HOME/bin/cargo"
ln -s "$work/nonexistent-cargo" "$CARGO_HOME/bin/cargo"
if sh "$script_dir/install_rust.sh" "$work/metadata" > "$work/broken-proxy.log" 2>&1; then
    printf '%s\n' 'error: dangling Cargo proxy was replaced or accepted' >&2; exit 1
fi
[ -L "$CARGO_HOME/bin/cargo" ]
[ ! -e "$CARGO_HOME/bin/cargo" ]
grep -Fq 'cargo proxy is unusable; existing files were preserved' "$work/broken-proxy.log"
rm -rf "$CARGO_HOME"
mkdir -p "$CARGO_HOME"
rm "$tools/rustup"

# Exercise the real installer transaction: a failed toolchain install cannot
# replace any of the existing sibling binaries, and success activates all three.
PATH=$original_path
export PATH
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
cp "$work/metadata" "$stage/rustc-version.txt"
cp "$script_dir/install_rust.sh" "$stage/install_rust.sh"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '#!/bin/sh\nprintf "new %s\\n"\n' "$binary" > "$stage/$binary"
    chmod +x "$stage/$binary"
    printf 'old %s\n' "$binary" > "$work/installed/$binary"
done
tar -czf "$work/dist/v0.2.0/$stem.tar.gz" -C "$work" "$stem"
if command -v sha256sum >/dev/null 2>&1; then
    (cd "$work/dist/v0.2.0" && sha256sum "$stem.tar.gz" > "$stem.tar.gz.sha256")
else
    (cd "$work/dist/v0.2.0" && shasum -a 256 "$stem.tar.gz" > "$stem.tar.gz.sha256")
fi
if FIXTURE_FAIL_INSTALL=1 sh "$script_dir/install.sh" --version v0.2.0 \
    --base-url "file://$work/dist" --dir "$work/installed" > "$work/transaction-fail.log" 2>&1; then
    printf '%s\n' 'error: failed Rust setup activated the installer' >&2; exit 1
fi
grep -Fq 'Rust/Cargo setup failed' "$work/transaction-fail.log"
for binary in cerulion cerulion-netd cerulion-connectd; do
    [ "$(cat "$work/installed/$binary")" = "old $binary" ]
done
(
    unset HOME
    CERULION_INSTALL_DIR="$work/installed" sh "$script_dir/install.sh" \
        --version v0.2.0 --base-url "file://$work/dist"
) > "$work/transaction-pass.log" 2>&1 ||
    { cat "$work/transaction-pass.log" >&2; exit 1; }
for binary in cerulion cerulion-netd cerulion-connectd; do
    cmp "$stage/$binary" "$work/installed/$binary"
done
activation=$(sed -n 's/^Add tools to PATH: //p' "$work/transaction-pass.log")
[ -n "$activation" ]
# The size the user is told about before rustup starts fetching. Provisioning
# multiplies what a first install downloads, and the archive alone gives no
# hint of it. The whole sentence is pinned, not the number: the step runs on
# every install that carries the bootstrap, including one where the compiler is
# already there and nothing is fetched, so the claim is a conditional one.
grep -Fq 'where that compiler is not already installed this downloads about 140 MB' \
    "$work/transaction-pass.log"
(
    eval "$activation"
    [ "$(command -v cerulion)" = "$work/installed/cerulion" ]
    [ "$(command -v rustup)" = "$CARGO_HOME/bin/rustup" ]
    [ "$(command -v cargo)" = "$CARGO_HOME/bin/cargo" ]
    RUSTUP_TOOLCHAIN=1.93.0 cargo --version | grep -Fxq 'cargo 1.93.0 (fixture)'
)

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
chmod +x "$con_bin/ln" "$con_bin/curl"
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
    PATH="$con_bin" CARGO_HOME="$2" RUSTUP_HOME="$3" FIXTURE_STATE="$3" \
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
        for binary in cerulion cerulion-netd cerulion-connectd; do
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
        for binary in cerulion cerulion-netd cerulion-connectd; do
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
            for binary in cerulion cerulion-netd cerulion-connectd; do
                [ "$(cat "$con_dest/$binary")" = "old $binary" ]
            done
        done
    else
        : > "$con_case/release.a"; : > "$con_case/release.b"
        con_join "$con_a" success; con_join "$con_b" success
        for con_dest in "$con_case/a" "$con_dest_b"; do
            for binary in cerulion cerulion-netd cerulion-connectd; do
                cmp "$stage/$binary" "$con_dest/$binary"
            done
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
            sh "$script_dir/install_rust.sh" "$work/metadata" > "$con_case/refusal.log" 2>&1; then
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

# An incomplete new archive must not silently take the legacy-install path.
rm "$stage/rustc-version.txt"
tar -czf "$work/dist/v0.2.0/$stem.tar.gz" -C "$work" "$stem"
if command -v sha256sum >/dev/null 2>&1; then
    (cd "$work/dist/v0.2.0" && sha256sum "$stem.tar.gz" > "$stem.tar.gz.sha256")
else
    (cd "$work/dist/v0.2.0" && shasum -a 256 "$stem.tar.gz" > "$stem.tar.gz.sha256")
fi
if sh "$script_dir/install.sh" --version v0.2.0 --base-url "file://$work/dist" \
    --dir "$work/installed" > "$work/partial.log" 2>&1; then
    printf '%s\n' 'error: partial compiler metadata was accepted' >&2; exit 1
fi
grep -Fq 'archive is missing part of its Rust/Cargo setup' "$work/partial.log"
printf '%s\n' 'Rust/Cargo bootstrap tests passed'
