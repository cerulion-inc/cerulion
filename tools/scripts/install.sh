#!/bin/sh
#
# Install the prebuilt Cerulion command-line tools for a supported host.
#
# If activation fails, the installer restores the binaries it replaced. If a
# restore itself fails, the transaction directory is retained and named in the
# error so the backups remain available for manual recovery.
#
# Usage:
#   curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh
#
set -eu

REPOSITORY="cerulion-inc/cerulion"
DEFAULT_BASE_URL="https://github.com/${REPOSITORY}/releases/download"
# The release channel. GitHub never marks a prerelease "latest", so this
# redirect always resolves to the newest stable release, and that release
# carries a one line `stable.txt` asset naming its own tag. `--base-url`
# overrides this root as well, so a fixture directory holding a `stable.txt`
# beside the archives resolves its own version offline.
DEFAULT_STABLE_BASE_URL="https://github.com/${REPOSITORY}/releases/latest/download"

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage: install.sh [--version vX.Y.Z[-PRERELEASE]] [--dir DIRECTORY] [--base-url URL]
                  [--no-modify-path]
       install.sh --self-test

Install the Cerulion CLI and its sibling daemons from a release archive.

Options:
  --version TAG     Install this release (otherwise use the latest stable release)
  --dir DIRECTORY  Install into DIRECTORY instead of $CERULION_INSTALL_DIR or ~/.cerulion/bin
  --base-url URL    Override the release download root (for local/offline testing)
  --no-modify-path  Leave the shell profile files alone and print the PATH to set
  --self-test       Exercise platform and version parsing without network access

Setting CERULION_NO_MODIFY_PATH to any value has the same effect as
--no-modify-path.
EOF
}

platform_target() {
    platform_os=$1
    platform_arch=$2
    case "${platform_os}:${platform_arch}" in
        Linux:x86_64|Linux:amd64)
            printf '%s\n' "x86_64-unknown-linux-gnu"
            ;;
        Linux:aarch64|Linux:arm64)
            printf '%s\n' "aarch64-unknown-linux-gnu"
            ;;
        Darwin:x86_64|Darwin:amd64)
            printf '%s\n' "x86_64-apple-darwin"
            ;;
        Darwin:arm64|Darwin:aarch64)
            printf '%s\n' "aarch64-apple-darwin"
            ;;
        *)
            return 1
            ;;
    esac
}

is_wsl2() {
    wsl_kernel_os="${CERULION_WSL_KERNEL_OS:-$(uname -s)}"
    [ "$wsl_kernel_os" = Linux ] || return 1
    if [ -r "${CERULION_WSL_OSRELEASE_PATH:-/proc/sys/kernel/osrelease}" ] &&
        grep -qi 'microsoft' "${CERULION_WSL_OSRELEASE_PATH:-/proc/sys/kernel/osrelease}"; then
        return 0
    fi
    [ -r "${CERULION_WSL_PROC_VERSION_PATH:-/proc/version}" ] &&
        grep -qiE 'microsoft|wsl' "${CERULION_WSL_PROC_VERSION_PATH:-/proc/version}"
}

# Rust compiles through the system linker, so a host with no `cc` installs
# cleanly and then fails at the first `cerulion node build` with a linker
# error that names no remedy. rustup detects the same condition and says so
# in passing; this names the one command that fixes it. Nothing here installs
# anything: it reports, and the user decides.
c_toolchain_command() {
    printf '%s\n' "${CERULION_C_COMPILER:-cc}"
}

toolchain_kernel_os() {
    printf '%s\n' "${CERULION_TOOLCHAIN_KERNEL_OS:-$(uname -s)}"
}

xcode_select_command() {
    printf '%s\n' "${CERULION_XCODE_SELECT:-xcode-select}"
}

# macOS answers `command -v cc` on a machine that cannot compile anything:
# /usr/bin/cc is one of the shims the operating system ships, which asks xcrun
# for a real compiler and fails when the command line tools are absent. What is
# there or not is the developer directory, so that is what Darwin is asked.
have_c_toolchain() {
    if [ "$(toolchain_kernel_os)" = Darwin ]; then
        "$(xcode_select_command)" -p >/dev/null 2>&1
        return
    fi
    command -v "$(c_toolchain_command)" >/dev/null 2>&1
}

# Prints the exact command for hosts whose package manager is known, and
# nothing at all otherwise, so the caller can fall back to a general sentence
# rather than print a guess.
c_toolchain_remedy() {
    toolchain_os=$(toolchain_kernel_os)
    if [ "$toolchain_os" = Darwin ]; then
        printf '%s\n' 'xcode-select --install'
        return 0
    fi
    toolchain_os_release="${CERULION_OS_RELEASE_PATH:-/etc/os-release}"
    if [ -r "$toolchain_os_release" ] &&
        grep -Eq '^(ID|ID_LIKE)=.*(debian|ubuntu)' "$toolchain_os_release"; then
        printf '%s\n' 'sudo apt-get install -y build-essential git'
        return 0
    fi
    return 1
}

report_c_toolchain() {
    if have_c_toolchain; then
        return 0
    fi
    if [ "$(toolchain_kernel_os)" = Darwin ]; then
        printf '%s\n' \
            'NOTE: no C linker: the Xcode command line tools are not installed, so building nodes will fail.' >&2
    else
        printf 'NOTE: no C linker (%s) is on PATH, so building nodes will fail.\n' \
            "$(c_toolchain_command)" >&2
    fi
    if toolchain_remedy=$(c_toolchain_remedy); then
        printf 'NOTE: install one first: %s\n' "$toolchain_remedy" >&2
    else
        printf '%s\n' \
            'NOTE: install your system C compiler and linker before building nodes.' >&2
    fi
}

valid_version() {
    case $1 in
        *"
"*) return 1 ;;
    esac
    printf '%s' "$1" |
        grep -Eq '^v[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?$'
}

stable_channel_cleanup() {
    rm -f "${stable_body_path:-}" || :
    rm -f "${stable_status_path:-}" || :
}

stable_channel_abort() {
    if [ -n "${foreground_child_pid:-}" ]; then
        terminate_process_tree "$foreground_child_pid"
        wait "$foreground_child_pid" 2>/dev/null || :
        foreground_child_pid=""
    else
        reap_foreground_children
    fi
    stable_channel_cleanup
    exit 1
}

stable_channel_release() {
    stable_channel_cleanup
    stable_body_path=
    stable_status_path=
    trap - EXIT INT TERM HUP
}

resolve_stable_release() {
    stable_base_url=${1%/}
    stable_url=$stable_base_url/stable.txt
    stable_hint="base URL $stable_base_url; a --base-url without stable.txt requires explicit --version"
    stable_body_path=$(mktemp "${TMPDIR:-/tmp}/cerulion-stable.XXXXXX") ||
        die "could not create a temporary release channel file"
    stable_status_path="${stable_body_path}.status"
    trap stable_channel_cleanup EXIT
    trap stable_channel_abort INT TERM HUP
    stable_status=
    if ! run_foreground curl -sSL --retry 3 --retry-delay 1 \
        -o "$stable_body_path" -w '%{http_code}' "$stable_url" \
        > "$stable_status_path"; then
        stable_channel_release
        printf '%s\n' \
            "error: could not reach the Cerulion release channel to resolve the latest stable release ($stable_hint); specify --version vX.Y.Z[-PRERELEASE]" >&2
        return 1
    fi
    stable_status=$(cat "$stable_status_path")
    case "$stable_status" in
        # curl reports 000 when the transfer carried no HTTP response to
        # report. Every failed fetch already returned through the branch
        # above, so reaching here with 000 means a non-HTTP scheme delivered
        # the body: a `--base-url file://...` fixture directory. Over https a
        # successful fetch always carries a code, so this cannot loosen the
        # release channel itself, and the size, line and tag checks below
        # still apply to whatever the fixture served.
        200 | 000) ;;
        404)
            stable_channel_release
            printf '%s\n' \
                "error: the Cerulion release channel has no stable release published yet ($stable_hint); specify --version vX.Y.Z[-PRERELEASE]" >&2
            return 1
            ;;
        403)
            stable_channel_release
            printf '%s\n' \
                "error: the Cerulion release channel has no stable release published yet, or rejected/rate-limited this request (HTTP 403; $stable_hint); retry later or specify --version vX.Y.Z[-PRERELEASE]" >&2
            return 1
            ;;
        429)
            stable_channel_release
            printf 'error: the Cerulion release channel rate limit was reached (HTTP %s; %s); retry later or specify --version vX.Y.Z[-PRERELEASE]\n' \
                "$stable_status" "$stable_hint" >&2
            return 1
            ;;
        *)
            stable_channel_release
            printf 'error: the Cerulion release channel returned HTTP %s while resolving the latest stable release (%s); specify --version vX.Y.Z[-PRERELEASE]\n' \
                "$stable_status" "$stable_hint" >&2
            return 1
            ;;
    esac
    stable_bytes=$(wc -c < "$stable_body_path" | tr -d ' ')
    if [ "$stable_bytes" -gt 64 ]; then
        stable_channel_release
        printf '%s\n' \
            "error: the Cerulion release channel returned an oversized stable release body ($stable_hint); specify --version vX.Y.Z[-PRERELEASE]" >&2
        return 1
    fi
    stable_lines=$(awk 'END { print NR }' "$stable_body_path")
    if [ "$stable_lines" -ne 1 ]; then
        stable_channel_release
        printf '%s\n' \
            "error: the Cerulion release channel returned a malformed stable release tag ($stable_hint); specify --version vX.Y.Z[-PRERELEASE]" >&2
        return 1
    fi
    stable_tag=$(sed -n '1p' "$stable_body_path")
    stable_channel_release
    if ! valid_version "$stable_tag" || printf '%s' "$stable_tag" | grep -q -- '-'; then
        printf '%s\n' \
            "error: the Cerulion release channel returned no valid stable release tag ($stable_hint); specify --version vX.Y.Z[-PRERELEASE]" >&2
        return 1
    fi
    if [ "${stable_channel_quiet:-0}" -eq 0 ]; then
        printf '%s\n' "$stable_tag"
    fi
}

release_lock() {
    release_failed=0
    if [ "${lock_released:-0}" -eq 1 ] &&
        [ -n "${lock_path:-}" ] &&
        [ ! -e "$lock_path" ]; then
        return 0
    fi
    if [ -n "${lock_temp_path:-}" ]; then
        if ! rm -f "$lock_temp_path"; then
            printf 'error: could not remove temporary installer lock: %s\n' \
                "$lock_temp_path" >&2
            release_failed=1
        fi
    fi
    [ -n "${lock_path:-}" ] || return "$release_failed"
    if [ -n "${lock_claim_identity:-}" ] &&
        ! lock_claim_is_current; then
        printf 'error: installer lock changed before release: %s\n' "$lock_path" >&2
        return 1
    fi
    if [ -f "$lock_path" ]; then
        owner=$(cat "$lock_path" 2>/dev/null || true)
        if [ "$owner" = "$$" ] && ! rm -f "$lock_path"; then
            printf 'error: could not release installer lock: %s\n' "$lock_path" >&2
            release_failed=1
        elif [ "$owner" = "$$" ]; then
            lock_released=1
        fi
    fi
    return "$release_failed"
}

cleanup_staging() {
    cleanup_failed=0
    rm -rf "$1" || cleanup_failed=1
    rm -rf "$2" || cleanup_failed=1
    if [ "$cleanup_failed" -ne 0 ]; then
        printf 'error: could not remove installation staging files\n' >&2
    fi
    if ! release_lock; then
        cleanup_failed=1
    fi
    return "$cleanup_failed"
}

raced_activation_directory_cleanup() {
    raced_binary=$1
    raced_destination="$install_dir/$raced_binary"
    [ -d "$raced_destination" ] || return 1
    raced_nested="$raced_destination/$raced_binary"
    if [ -f "$raced_nested" ]; then
        raced_nested_digest=$(sha256_digest "$raced_nested" 2>/dev/null) ||
            return 1
        raced_archive_digest=$(sha256_digest "$archive_dir/$raced_binary" 2>/dev/null) ||
            return 1
        [ -n "$raced_nested_digest" ] || return 1
        [ -n "$raced_archive_digest" ] || return 1
        [ "$raced_nested_digest" = "$raced_archive_digest" ] || return 1
        rm -f "$raced_nested" || return 1
    fi
    if command -v rmdir >/dev/null 2>&1; then
        rmdir "$raced_destination"
    else
        rm -d "$raced_destination"
    fi
}

sha256_digest() {
    sha256_path=$1
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$sha256_path" | awk '{ print $1 }'
    elif command -v shasum >/dev/null 2>&1; then
        shasum -a 256 "$sha256_path" | awk '{ print $1 }'
    else
        return 1
    fi
}

file_identity() {
    file_identity_path=$1
    for file_identity_stat in \
        "$(command -v stat 2>/dev/null || :)" \
        /usr/bin/stat /bin/stat; do
        [ -x "$file_identity_stat" ] || continue
        if "$file_identity_stat" -c '%d:%i' "$file_identity_path" 2>/dev/null; then
            return 0
        fi
        if "$file_identity_stat" -f '%d:%i' "$file_identity_path" 2>/dev/null; then
            return 0
        fi
    done
    return 1
}

lock_claim_matches() {
    lock_source_path=$1
    lock_destination_path=$2
    [ -f "$lock_destination_path" ] || return 1
    [ "$(cat "$lock_destination_path" 2>/dev/null || :)" = "$$" ] || return 1
    lock_source_identity=$(file_identity "$lock_source_path") || return 1
    lock_destination_identity=$(file_identity "$lock_destination_path") || return 1
    [ "$lock_source_identity" = "$lock_destination_identity" ]
}

lock_claim_is_current() {
    [ -n "${lock_claim_identity:-}" ] || return 1
    lock_current_identity=$(file_identity "$lock_path") || return 1
    [ "$lock_current_identity" = "$lock_claim_identity" ] || return 1
    [ "$(cat "$lock_path" 2>/dev/null || :)" = "$$" ]
}

run_foreground() {
    "$@" &
    run_foreground_pid=$!
    if [ "${CERULION_SELF_TEST_SKIP_CHILD_PID:-0}" -eq 0 ]; then
        foreground_child_pid=$run_foreground_pid
    fi
    while kill -0 "$run_foreground_pid" 2>/dev/null; do
        sleep 0.1
    done
    if wait "$run_foreground_pid"; then
        foreground_status=0
    else
        foreground_status=$?
    fi
    foreground_child_pid=""
    return "$foreground_status"
}

terminate_process_tree() {
    process_tree_root=$1
    process_tree_descendants=$(
        # Every generation, captured before anything is signalled and listed
        # deepest first: the archive's Rust setup runs its own helpers, so the
        # tree under one foreground child is several levels deep, and killing a
        # parent first reparents its descendants where a later scan misses them.
        ps -eo pid=,ppid= 2>/dev/null |
            awk -v root="$process_tree_root" '
                { parent[$1] = $2 }
                END {
                    depth[root] = 0
                    found = 1
                    max_depth = 0
                    while (found) {
                        found = 0
                        for (pid in parent) {
                            if (!(pid in depth) && (parent[pid] in depth)) {
                                depth[pid] = depth[parent[pid]] + 1
                                if (depth[pid] > max_depth) max_depth = depth[pid]
                                found = 1
                            }
                        }
                    }
                    for (level = max_depth; level > 0; level--) {
                        for (pid in depth) {
                            if (depth[pid] == level) print pid
                        }
                    }
                }
            '
    )
    for process_tree_pid in $process_tree_descendants; do
        kill -TERM "$process_tree_pid" 2>/dev/null || :
    done
    kill -TERM "$process_tree_root" 2>/dev/null || :
}

reap_foreground_children() {
    reap_attempt=0
    while [ "$reap_attempt" -lt 10 ]; do
        reap_attempt=$((reap_attempt + 1))
        reap_child_pids=$(
            ps -eo pid=,ppid= 2>/dev/null |
                awk -v parent="$$" '$2 == parent { print $1 }'
        )
        [ -n "$reap_child_pids" ] || return 0
        for reap_child_pid in $reap_child_pids; do
            terminate_process_tree "$reap_child_pid"
            wait "$reap_child_pid" 2>/dev/null || :
        done
        sleep 0.1
    done
}

has_descendant() {
    descendant_root=$1
    ps -eo pid=,ppid=,stat= 2>/dev/null |
        awk -v root="$descendant_root" '
            {
                if ($3 !~ /^Z/) {
                    parent[$1] = $2
                }
            }
            END {
                found = 1
                seen[root] = 1
                while (found) {
                    found = 0
                    for (pid in parent) {
                        if (!(pid in seen) && (parent[pid] in seen)) {
                            seen[pid] = 1
                            found = 1
                        }
                    }
                }
                for (pid in seen) {
                    if (pid != root) {
                        print pid
                        exit
                    }
                }
            }
        ' |
        grep -q .
}

is_running_pid() {
    process_state=$(ps -p "$1" -o stat= 2>/dev/null || :)
    [ -n "$process_state" ] && [ "${process_state#Z}" = "$process_state" ]
}

# A child process cannot change the PATH of the shell that started it. What an
# installer can do is write the setup once and have every new shell read it:
# an `env` file that prepends the program directories, a fish twin of it, and
# one marked line in each profile the user's shell reads that sources the file.
PROFILE_MARKER='# added by the Cerulion installer'

sh_quote() {
    printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\\\''/g")"
}

fish_quote() {
    printf "'%s'" "$(printf '%s' "$1" | sed -e 's/\\/\\\\/g' -e "s/'/\\\\'/g")"
}

# shellcheck disable=SC2016
sh_path_stanza() {
    stanza_path=$(sh_quote "$1")
    printf 'case ":${PATH}:" in\n'
    printf '    *:%s:*) ;;\n' "$stanza_path"
    printf '    *) PATH=%s:"${PATH}" ;;\n' "$stanza_path"
    printf 'esac\n'
}

# shellcheck disable=SC2016
fish_path_stanza() {
    stanza_path=$(fish_quote "$1")
    printf 'if not contains %s $PATH\n' "$stanza_path"
    printf '    set -gx PATH %s $PATH\n' "$stanza_path"
    printf 'end\n'
}

# Written through a temporary file so a reader never sees a half-written env.
write_env_file() {
    env_path=$1
    env_bin=$2
    env_cargo=$3
    env_dir=${env_path%/*}
    mkdir -p "$env_dir" || return 1
    env_temp="$env_dir/.cerulion-env.$$"
    rm -f "$env_temp" || return 1
    {
        printf '%s\n' '#!/bin/sh'
        printf '%s\n' '# Put the Cerulion programs on the PATH of this shell.'
        printf '%s\n' '# Written by the Cerulion installer; the next install rewrites it.'
        sh_path_stanza "$env_bin"
        if [ -n "$env_cargo" ]; then
            sh_path_stanza "$env_cargo"
        fi
        printf '%s\n' 'export PATH'
    } > "$env_temp" || {
        rm -f "$env_temp"
        return 1
    }
    mv -f "$env_temp" "$env_path" || {
        rm -f "$env_temp"
        return 1
    }
}

write_env_fish_file() {
    fish_env_path=$1
    fish_env_bin=$2
    fish_env_cargo=$3
    fish_env_dir=${fish_env_path%/*}
    mkdir -p "$fish_env_dir" || return 1
    fish_env_temp="$fish_env_dir/.cerulion-env-fish.$$"
    rm -f "$fish_env_temp" || return 1
    {
        printf '%s\n' '# Put the Cerulion programs on the PATH of this shell.'
        printf '%s\n' '# Written by the Cerulion installer; the next install rewrites it.'
        fish_path_stanza "$fish_env_bin"
        if [ -n "$fish_env_cargo" ]; then
            fish_path_stanza "$fish_env_cargo"
        fi
    } > "$fish_env_temp" || {
        rm -f "$fish_env_temp"
        return 1
    }
    mv -f "$fish_env_temp" "$fish_env_path" || {
        rm -f "$fish_env_temp"
        return 1
    }
}

profile_source_line() {
    printf '. %s %s\n' "$(sh_quote "$1")" "$PROFILE_MARKER"
}

fish_source_line() {
    printf 'source %s %s\n' "$(fish_quote "$1")" "$PROFILE_MARKER"
}

# A profile that is a symlink pointing out of the home directory belongs to
# something else, a dotfile repository or another account, and is left alone.
# One link is resolved and the directory it lands in is canonicalized, so a
# chain of linked directories is covered too.
profile_path_is_safe() {
    safe_path=$1
    [ -L "$safe_path" ] || return 0
    [ -n "${HOME:-}" ] || return 1
    safe_home=$(CDPATH='' cd -- "$HOME" 2>/dev/null && pwd) || return 1
    safe_target=$(readlink "$safe_path" 2>/dev/null) || return 1
    case "$safe_target" in
        /*) ;;
        *) safe_target="${safe_path%/*}/$safe_target" ;;
    esac
    safe_target_dir=$(CDPATH='' cd -- "${safe_target%/*}" 2>/dev/null && pwd) || return 1
    case "$safe_target_dir" in
        "$safe_home") return 0 ;;
        "$safe_home"/*) return 0 ;;
        *) return 1 ;;
    esac
}

# 0 the line was added, 1 the edit failed, 2 nothing to do, 3 already present.
add_profile_line() {
    profile_path=$1
    profile_line=$2
    profile_create=$3
    if [ ! -e "$profile_path" ] && [ ! -L "$profile_path" ] &&
        [ "$profile_create" -ne 1 ]; then
        return 2
    fi
    if ! profile_path_is_safe "$profile_path"; then
        printf 'warning: leaving %s alone: it is a symlink out of your home directory\n' \
            "$profile_path" >&2
        return 2
    fi
    if [ -f "$profile_path" ] && grep -Fq "$PROFILE_MARKER" "$profile_path" 2>/dev/null; then
        return 3
    fi
    profile_dir=${profile_path%/*}
    mkdir -p "$profile_dir" || return 1
    # A last line without its newline would otherwise absorb the added one.
    if [ -s "$profile_path" ] &&
        [ -n "$(tail -c 1 "$profile_path" 2>/dev/null | tr -d '\n')" ]; then
        printf '\n' >> "$profile_path" || return 1
    fi
    printf '%s\n' "$profile_line" >> "$profile_path" || return 1
    return 0
}

apply_profile() {
    apply_status=0
    add_profile_line "$1" "$2" "$3" || apply_status=$?
    case "$apply_status" in
        0)
            printf 'PATH line added to %s\n' "$1"
            ;;
        3)
            printf 'PATH line already in %s\n' "$1"
            ;;
        2) ;;
        *)
            printf 'warning: could not add the PATH line to %s\n' "$1" >&2
            ;;
    esac
}

# Returns nonzero only when the env file itself could not be written, which is
# the one failure that leaves the user with nothing to source.
setup_path_for_shells() {
    setup_bin=$1
    setup_cargo=$2
    setup_home=${CERULION_HOME:-$HOME/.cerulion}
    setup_env="$setup_home/env"
    setup_env_fish="$setup_home/env.fish"
    if ! write_env_file "$setup_env" "$setup_bin" "$setup_cargo"; then
        printf 'warning: could not write %s\n' "$setup_env" >&2
        return 1
    fi
    printf 'PATH setup written to %s\n' "$setup_env"
    if write_env_fish_file "$setup_env_fish" "$setup_bin" "$setup_cargo"; then
        printf 'PATH setup written to %s\n' "$setup_env_fish"
    else
        printf 'warning: could not write %s\n' "$setup_env_fish" >&2
    fi
    setup_line=$(profile_source_line "$setup_env")
    apply_profile "$HOME/.profile" "$setup_line" 1
    apply_profile "$HOME/.bash_profile" "$setup_line" 0
    apply_profile "$HOME/.bash_login" "$setup_line" 0
    apply_profile "$HOME/.bashrc" "$setup_line" 0
    if [ -n "${ZDOTDIR:-}" ] || command -v zsh >/dev/null 2>&1; then
        apply_profile "${ZDOTDIR:-$HOME}/.zshenv" "$setup_line" 1
    fi
    setup_fish_dir="${XDG_CONFIG_HOME:-$HOME/.config}/fish"
    if command -v fish >/dev/null 2>&1 || [ -d "$setup_fish_dir" ]; then
        apply_profile "$setup_fish_dir/conf.d/cerulion.fish" \
            "$(fish_source_line "$setup_env_fish")" 1
    fi
    printf 'Open a new terminal, or run . %s, then run cerulion login\n' \
        "$(sh_quote "$setup_env")"
    return 0
}

# The install-provenance marker: one line of JSON beside the binaries naming
# this installer, which the CLI reports as its install method in usage
# telemetry. It is best effort; a failure warns and never fails the install.
write_install_marker() {
    marker_path="$install_dir/.cerulion-provenance.json"
    marker_tmp=$(mktemp "$install_dir/.cerulion-provenance.json.XXXXXX" 2>/dev/null || :)
    if [ -z "$marker_tmp" ]; then
        printf 'warning: could not write the install marker in %s\n' "$install_dir" >&2
        return 0
    fi
    if ! printf '{"method":"install.sh","version":"%s"}\n' "$version" > "$marker_tmp" ||
        ! chmod 0644 "$marker_tmp" ||
        ! mv -f "$marker_tmp" "$marker_path"; then
        rm -f "$marker_tmp"
        printf 'warning: could not write the install marker in %s\n' "$install_dir" >&2
    fi
}

self_test() {
    # The self-test installs into scratch directories many times over; none of
    # those installs may touch the profile files or the env file of the user
    # running it. The arms that exercise the PATH setup unset this again for
    # their own scratch homes.
    CERULION_NO_MODIFY_PATH=1
    export CERULION_NO_MODIFY_PATH
    [ "$(platform_target Linux x86_64)" = "x86_64-unknown-linux-gnu" ] ||
        die "self-test: Linux x86_64 mapping failed"
    [ "$(platform_target Linux aarch64)" = "aarch64-unknown-linux-gnu" ] ||
        die "self-test: Linux aarch64 mapping failed"
    [ "$(platform_target Darwin x86_64)" = "x86_64-apple-darwin" ] ||
        die "self-test: macOS x86_64 mapping failed"
    [ "$(platform_target Darwin arm64)" = "aarch64-apple-darwin" ] ||
        die "self-test: macOS arm64 mapping failed"
    if platform_target Windows x86_64 >/dev/null 2>&1; then
        die "self-test: unsupported platform was accepted"
    fi
    if [ "${base_url_explicit:-0}" -eq 0 ]; then
        [ "$stable_base_url" = "$DEFAULT_STABLE_BASE_URL" ] ||
            die "self-test: the default release channel root is $stable_base_url"
        [ "$stable_base_url" = "https://github.com/$REPOSITORY/releases/latest/download" ] ||
            die "self-test: the release channel root is not GitHub's latest stable redirect"
        [ "$stable_base_url" != "$base_url" ] ||
            die "self-test: the release channel root must not be the archive root"
    fi
    valid_version "v0.1.0" ||
        die "self-test: valid version was rejected"
    valid_version "v0.1.0-rc1" ||
        die "self-test: prerelease version was rejected"
    valid_version "v1.2.3-rc-1" ||
        die "self-test: hyphenated prerelease version was rejected"
    if valid_version "0.1.0"; then
        die "self-test: version without v prefix was accepted"
    fi
    if valid_version "v0.1.0-"; then
        die "self-test: malformed prerelease version was accepted"
    fi
    if valid_version "v0.1.0-rc..1"; then
        die "self-test: empty prerelease identifier was accepted"
    fi
    if valid_version "v1.2.3
../../x"; then
        die "self-test: embedded-newline version was accepted"
    fi
    (
        unset HOME CERULION_INSTALL_DIR
        [ "$(resolve_install_dir 1 /explicit/path)" = "/explicit/path" ]
    ) || die "self-test: explicit directory required HOME"
    channel_test_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-install-channel-test.XXXXXX") ||
        die "self-test: could not create release channel test directory"
    channel_test_tmpdir="$channel_test_dir/tmp"
    mkdir -p "$channel_test_tmpdir"
    channel_test_bin="$channel_test_dir/bin"
    mkdir -p "$channel_test_bin"
    channel_test_real_curl=$(command -v curl)
    cat > "$channel_test_bin/curl" <<'EOF'
#!/bin/sh
case "$*" in
    *stable.txt*)
        output=
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -o) output=$2; shift 2 ;;
                -w) shift 2 ;;
                *) shift ;;
            esac
        done
        body=
        status=200
        wire_oversized=0
        case "${CERULION_SELF_TEST_CHANNEL_MODE:-valid}" in
            valid)
                body='v0.1.0
'
                ;;
            valid-no-newline) body='v0.1.0' ;;
            valid-newline)
                body='v0.1.0
'
                ;;
            404)
                status=404
                ;;
            403)
                status=403
                ;;
            transport)
                exit 7
                ;;
            blocked)
                : > "${CERULION_SELF_TEST_CHANNEL_MARKER:?}"
                printf '%s\n' "$$" > "${CERULION_SELF_TEST_CHANNEL_PID:?}"
                while :; do
                    sleep 1
                done
                ;;
            blocked-nested)
                printf '%s\n' "$$" > "${CERULION_SELF_TEST_CHANNEL_PID:?}"
                "${CERULION_SELF_TEST_CHANNEL_DESCENDANT:?}" 1 &
                wait
                ;;
            malformed)
                body='not-a-version
'
                ;;
            oversized)
                body='v0.1.0-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
'
                ;;
            wire-oversized)
                wire_oversized=1
                ;;
            trailing-blank)
                body='v0.1.0

'
                ;;
            multiline)
                body='v0.1.0
ignored
'
                ;;
            prerelease)
                body='v0.1.0-rc1
'
                ;;
            *)
                exit 1
                ;;
        esac
        if [ "$wire_oversized" -eq 1 ]; then
            if [ -n "$output" ]; then
                printf 'v0.1.0-%057d\n' 0 > "$output"
            else
                printf 'v0.1.0-%057d\n' 0
            fi
        elif [ -n "$output" ]; then
            printf '%s' "$body" > "$output"
        else
            printf '%s' "$body"
        fi
        printf '%s\n' "$status"
        ;;
    *)
        exec "$CERULION_SELF_TEST_REAL_CURL" "$@"
        ;;
esac
EOF
    cat > "$channel_test_bin/blocked-descendant" <<'EOF'
#!/bin/sh
printf '%s\n' "$$" >> "${CERULION_SELF_TEST_CHANNEL_PID:?}"
if [ "$1" -gt 0 ]; then
    "$0" "$(($1 - 1))" &
else
    sleep 30 &
    printf '%s\n' "$!" >> "$CERULION_SELF_TEST_CHANNEL_PID"
    : > "${CERULION_SELF_TEST_CHANNEL_MARKER:?}"
fi
wait
EOF
    chmod +x "$channel_test_bin/curl" "$channel_test_bin/blocked-descendant"
    channel_test_previous_path=$PATH
    PATH="$channel_test_bin:$PATH"
    export PATH CERULION_SELF_TEST_REAL_CURL="$channel_test_real_curl"
    assert_channel_case() {
        channel_case=$1
        channel_expected_status=$2
        channel_expected_text=$3
        if channel_output=$(CERULION_SELF_TEST_CHANNEL_MODE="$channel_case" \
            TMPDIR="$channel_test_tmpdir" \
            resolve_stable_release https://example.test/dist 2>&1); then
            channel_status=0
        else
            channel_status=$?
        fi
        [ "$channel_status" -eq "$channel_expected_status" ] ||
            die "self-test: release channel case $channel_case returned $channel_status, expected $channel_expected_status"
        if [ "$channel_expected_status" -eq 0 ]; then
            [ "$channel_output" = "v0.1.0" ] ||
                die "self-test: valid release channel case returned $channel_output"
        else
            printf '%s\n' "$channel_output" |
                grep -Fq "$channel_expected_text" ||
                die "self-test: release channel case $channel_case had the wrong diagnostic"
            printf '%s\n' "$channel_output" |
                grep -Fq 'base URL https://example.test/dist; a --base-url without stable.txt requires explicit --version' ||
                die "self-test: release channel case $channel_case omitted the queried base URL guidance"
        fi
        if find "$channel_test_tmpdir" -name 'cerulion-stable.*' -type f \
            -print -quit | grep -q .; then
            die "self-test: release channel case $channel_case left a temporary body"
        fi
    }
    assert_channel_case valid 0 ''
    assert_channel_case valid-no-newline 0 ''
    assert_channel_case valid-newline 0 ''
    assert_channel_case 404 1 'no stable release published yet'
    assert_channel_case 403 1 'no stable release published yet, or rejected/rate-limited'
    assert_channel_case transport 1 'could not reach the Cerulion release channel'
    assert_channel_case malformed 1 'returned no valid stable release tag'
    assert_channel_case oversized 1 'returned an oversized stable release body'
    assert_channel_case wire-oversized 1 'returned an oversized stable release body'
    assert_channel_case trailing-blank 1 'returned a malformed stable release tag'
    assert_channel_case multiline 1 'returned a malformed stable release tag'
    assert_channel_case prerelease 1 'returned no valid stable release tag'
    run_blocked_channel_case() {
        blocked_channel_case=$1
        blocked_channel_skip_pid=$2
        blocked_channel_dir="$channel_test_dir/$blocked_channel_case"
        blocked_channel_tmpdir="$blocked_channel_dir/tmp"
        blocked_channel_install_dir="$blocked_channel_dir/install"
        blocked_channel_marker="$blocked_channel_dir/started"
        blocked_channel_pid="$blocked_channel_dir/curl.pid"
        mkdir -p "$blocked_channel_tmpdir" "$blocked_channel_install_dir"
        blocked_channel_output="$blocked_channel_dir/output"
        blocked_channel_error="$blocked_channel_dir/error"
        TMPDIR="$blocked_channel_tmpdir" \
        CERULION_SELF_TEST_CHANNEL_MODE="${3:-blocked}" \
        CERULION_SELF_TEST_CHANNEL_DESCENDANT="$channel_test_bin/blocked-descendant" \
        CERULION_SELF_TEST_CHANNEL_MARKER="$blocked_channel_marker" \
        CERULION_SELF_TEST_CHANNEL_PID="$blocked_channel_pid" \
        CERULION_SELF_TEST_SKIP_CHILD_PID="$blocked_channel_skip_pid" \
        PATH="$channel_test_bin:$channel_test_previous_path" \
            "$0" --dir "$blocked_channel_install_dir" \
            --base-url https://example.test/dist --no-modify-path \
            >"$blocked_channel_output" 2>"$blocked_channel_error" &
        blocked_channel_installer_pid=$!
        blocked_channel_attempt=0
        while [ ! -f "$blocked_channel_marker" ]; do
            [ "$blocked_channel_attempt" -lt 100 ] ||
                die "self-test: blocked release channel $blocked_channel_case did not start before timeout"
            blocked_channel_attempt=$((blocked_channel_attempt + 1))
            sleep 0.1
        done
        kill -TERM "$blocked_channel_installer_pid" 2>/dev/null || :
        if wait "$blocked_channel_installer_pid"; then
            blocked_channel_status=0
        else
            blocked_channel_status=$?
        fi
        [ "$blocked_channel_status" -ne 0 ] ||
            die "self-test: blocked release channel $blocked_channel_case ignored TERM"
        if find "$blocked_channel_tmpdir" -name 'cerulion-stable.*' -type f \
            -print -quit | grep -q .; then
            die "self-test: blocked release channel $blocked_channel_case left a temporary body"
        fi
        while IFS= read -r blocked_channel_curl_pid; do
            blocked_channel_attempt=0
            while kill -0 "$blocked_channel_curl_pid" 2>/dev/null; do
                if [ "$blocked_channel_attempt" -ge 100 ]; then
                    # Clean up failed regressions, including orphaned descendants.
                    while IFS= read -r blocked_channel_cleanup_pid; do
                        kill -TERM "$blocked_channel_cleanup_pid" 2>/dev/null || :
                    done < "$blocked_channel_pid"
                    die "self-test: blocked release channel $blocked_channel_case left curl or a descendant running"
                fi
                blocked_channel_attempt=$((blocked_channel_attempt + 1))
                sleep 0.1
            done
        done < "$blocked_channel_pid"
        rm -rf "$blocked_channel_dir"
    }
    run_blocked_channel_case blocked 0
    run_blocked_channel_case blocked-skip-pid 1
    run_blocked_channel_case blocked-nested 0 blocked-nested
    run_blocked_channel_case blocked-nested-skip-pid 1 blocked-nested
    PATH=$channel_test_previous_path
    export PATH
    rm -rf "$channel_test_dir"
    # Every arm above stubs curl, so none of them proves the reader's own curl
    # invocation works. A file:// fixture directory does, without a network:
    # `--base-url` points the reader at a directory holding a `stable.txt`.
    if command -v curl >/dev/null 2>&1; then
        channel_file_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-install-channel-file.XXXXXX") ||
            die "self-test: could not create release channel fixture directory"
        channel_file_root="$channel_file_dir/root"
        channel_file_tmpdir="$channel_file_dir/tmp"
        mkdir -p "$channel_file_root" "$channel_file_tmpdir"
        printf 'v0.1.0\n' > "$channel_file_root/stable.txt"
        channel_file_tag=$(TMPDIR="$channel_file_tmpdir" \
            resolve_stable_release "file://$channel_file_root") ||
            die "self-test: a file:// release channel fixture was refused"
        [ "$channel_file_tag" = "v0.1.0" ] ||
            die "self-test: a file:// release channel fixture returned $channel_file_tag"
        printf 'not-a-version\n' > "$channel_file_root/stable.txt"
        if TMPDIR="$channel_file_tmpdir" \
            resolve_stable_release "file://$channel_file_root" >/dev/null 2>&1; then
            die "self-test: a malformed file:// release channel body was accepted"
        fi
        printf 'v0.1.0-rc1\n' > "$channel_file_root/stable.txt"
        if TMPDIR="$channel_file_tmpdir" \
            resolve_stable_release "file://$channel_file_root" >/dev/null 2>&1; then
            die "self-test: a prerelease file:// release channel body was accepted"
        fi
        rm -f "$channel_file_root/stable.txt"
        if TMPDIR="$channel_file_tmpdir" \
            resolve_stable_release "file://$channel_file_root" >/dev/null 2>&1; then
            die "self-test: a missing file:// release channel body was accepted"
        fi
        # A spawned installer proves --base-url moves the channel root and not
        # only the archive root. It stays offline: the fixture answers with its
        # own tag, and the run then fails on the archive that is not there.
        channel_wiring_dir="$channel_file_dir/wiring"
        channel_wiring_root="$channel_wiring_dir/root"
        mkdir -p "$channel_wiring_root" "$channel_wiring_dir/install"
        printf 'v0.1.0\n' > "$channel_wiring_root/stable.txt"
        channel_wiring_error="$channel_wiring_dir/error"
        if TMPDIR="$channel_wiring_dir" "$0" \
            --base-url "file://$channel_wiring_root" \
            --dir "$channel_wiring_dir/install" --no-modify-path \
            >/dev/null 2>"$channel_wiring_error"; then
            die "self-test: an archive-less release channel fixture installed"
        fi
        grep -Fq "file://$channel_wiring_root/v0.1.0/" "$channel_wiring_error" ||
            die "self-test: --base-url did not move the release channel root"
        if find "$channel_file_tmpdir" -name 'cerulion-stable.*' -type f \
            -print -quit | grep -q .; then
            die "self-test: a file:// release channel arm left a temporary body"
        fi
        rm -rf "$channel_file_dir"
    else
        printf '%s\n' \
            'install.sh self-test: skipping the file:// release channel arms because curl is unavailable' >&2
    fi
    owner_test_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-install-owner-test.XXXXXX") ||
        die "self-test: could not create owner classification directory"
    owner_test_path="$owner_test_dir/lock"
    cleanup_owner_test() {
        rm -rf "$owner_test_dir"
    }
    trap cleanup_owner_test 0
    printf '%s\n' "$$" > "$owner_test_path"
    [ "$(classify_lock_owner "$owner_test_path")" = "live" ] ||
        die "self-test: live owner was not classified as live"
    (
        :
    ) &
    dead_owner=$!
    wait "$dead_owner"
    printf '%s\n' "$dead_owner" > "$owner_test_path"
    [ "$(classify_lock_owner "$owner_test_path")" = "dead" ] ||
        die "self-test: dead owner was not classified as dead"
    : > "$owner_test_path"
    [ "$(classify_lock_owner "$owner_test_path")" = "invalid" ] ||
        die "self-test: empty owner was not classified as invalid"
    printf '%s\n' 'not-a-pid' > "$owner_test_path"
    [ "$(classify_lock_owner "$owner_test_path")" = "invalid" ] ||
        die "self-test: malformed owner was not classified as invalid"
    printf '%s\n' "$$" > "$owner_test_path"
    second_claim_path="$owner_test_dir/second-claim"
    printf '%s\n' "$$" > "$second_claim_path"
    if ln "$second_claim_path" "$owner_test_path" 2>/dev/null; then
        die "self-test: second live-owner acquisition succeeded"
    fi
    [ "$(classify_lock_owner "$owner_test_path")" = "live" ] ||
        die "self-test: live lock was changed by a failed acquisition"
    nonregular_lock_path="$owner_test_dir/lock-directory"
    mkdir "$nonregular_lock_path"
    if lock_path_is_regular_or_absent "$nonregular_lock_path"; then
        die "self-test: non-regular lock path was accepted"
    fi
    install_dir="$owner_test_dir"
    lock_path="$owner_test_path"
    lock_temp_path="$owner_test_dir/lock.$$"
    printf '%s\n' "$$" > "$lock_path"
    [ "$(classify_lock_owner "$lock_path")" = "live" ] ||
        die "self-test: live lock was not classified as live"
    assert_lock_expiration() {
        expiration_state=$1
        expected_message=$2
        if expiration_output=$(die_stranded_lock "$expiration_state" 2>&1); then
            die "self-test: $expiration_state expiration unexpectedly succeeded"
        fi
        [ "$expiration_output" = "error: $expected_message" ] ||
            die "self-test: unexpected $expiration_state expiration message: $expiration_output"
    }
    assert_lock_expiration live \
        "another Cerulion installation is in progress: $lock_path (lock is held by a live PID)"
    assert_lock_expiration dead \
        "could not acquire installer lock at $lock_path: owner PID is no longer running; delete $lock_path if no installation is running"
    assert_lock_expiration invalid \
        "could not acquire installer lock at $lock_path: owner contents are unreadable; delete $lock_path if no installation is running"
    assert_lock_expiration unexpected \
        "could not acquire installer lock at $lock_path: unexpected owner state: unexpected"

    cleanup_fault_dir="$owner_test_dir/cleanup-fault"
    cleanup_fault_workdir="$cleanup_fault_dir/work"
    cleanup_fault_transaction="$cleanup_fault_dir/transaction"
    cleanup_fault_bin="$cleanup_fault_dir/bin"
    mkdir -p "$cleanup_fault_workdir" "$cleanup_fault_transaction" "$cleanup_fault_bin"
    printf '%s\n' "$$" > "$cleanup_fault_dir/lock"
    cat > "$cleanup_fault_bin/rm" <<'EOF'
#!/bin/sh
if [ "${1:-}" = "-rf" ] && [ "${2:-}" = "$CERULION_SELF_TEST_FAIL_PATH" ]; then
    if [ ! -e "$CERULION_SELF_TEST_FAIL_MARKER" ]; then
        : > "$CERULION_SELF_TEST_FAIL_MARKER"
        exit 1
    fi
fi
if [ "${1:-}" = "-f" ] && [ "${2:-}" = "$CERULION_SELF_TEST_FAIL_LOCK_PATH" ]; then
    if [ ! -e "$CERULION_SELF_TEST_FAIL_LOCK_MARKER" ]; then
        : > "$CERULION_SELF_TEST_FAIL_LOCK_MARKER"
        exit 1
    fi
fi
exec "$CERULION_SELF_TEST_REAL_RM" "$@"
EOF
    chmod +x "$cleanup_fault_bin/rm"
    cleanup_fault_real_rm=$(command -v rm)
    cleanup_fault_previous_path=$PATH
    export CERULION_SELF_TEST_FAIL_PATH="$cleanup_fault_transaction"
    export CERULION_SELF_TEST_FAIL_MARKER="$cleanup_fault_dir/fail-once"
    export CERULION_SELF_TEST_FAIL_LOCK_PATH=
    export CERULION_SELF_TEST_FAIL_LOCK_MARKER=
    export CERULION_SELF_TEST_REAL_RM="$cleanup_fault_real_rm"
    PATH="$cleanup_fault_bin:$PATH"
    export PATH
    workdir="$cleanup_fault_workdir"
    transaction_dir="$cleanup_fault_transaction"
    install_dir="$cleanup_fault_dir"
    lock_path="$cleanup_fault_dir/lock"
    lock_temp_path=
    if cleanup_staging "$cleanup_fault_transaction" "$cleanup_fault_workdir" 2>/dev/null; then
        die "self-test: cleanup failure unexpectedly succeeded"
    fi
    PATH=$cleanup_fault_previous_path
    export PATH
    [ ! -e "$cleanup_fault_dir/lock" ] ||
        die "self-test: cleanup failure stranded the install lock"

    export CERULION_SELF_TEST_FAIL_LOCK_PATH="$cleanup_fault_dir/lock"
    export CERULION_SELF_TEST_FAIL_LOCK_MARKER="$cleanup_fault_dir/fail-lock-once"
    PATH="$cleanup_fault_bin:$cleanup_fault_previous_path"
    export PATH
    printf '%s\n' "$$" > "$cleanup_fault_dir/lock"
    if cleanup_lock_output=$(cleanup_staging "$cleanup_fault_transaction" \
        "$cleanup_fault_workdir" 2>&1); then
        die "self-test: lock-release failure unexpectedly succeeded"
    fi
    printf '%s\n' "$cleanup_lock_output" |
        grep -Fq "could not release installer lock: $cleanup_fault_dir/lock" ||
        die "self-test: lock-release failure was not reported"
    [ -f "$cleanup_fault_dir/lock" ] ||
        die "self-test: lock-release fault unexpectedly removed the lock"
    "$cleanup_fault_real_rm" -f "$cleanup_fault_dir/lock"
    PATH=$cleanup_fault_previous_path
    export PATH
    rm -rf "$cleanup_fault_dir"

    signal_fixture_dir="$owner_test_dir/signal-fixture"
    signal_source_dir="$signal_fixture_dir/v0.1.0"
    signal_archive_stem="cerulion-0.1.0-$(platform_target "$(uname -s)" "$(uname -m)")"
    signal_archive_name="$signal_archive_stem.tar.gz"
    signal_archive_root="$signal_fixture_dir/$signal_archive_stem"
    signal_bin="$owner_test_dir/signal-bin"
    mkdir -p "$signal_source_dir" "$signal_archive_root" "$signal_bin"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "new-$binary" > "$signal_archive_root/$binary"
    done
    printf '%s\n' 'license' > "$signal_archive_root/LICENSE"
    tar -czf "$signal_source_dir/$signal_archive_name" \
        -C "$signal_fixture_dir" "$signal_archive_stem"
    if ! signal_checksum=$(sha256_digest "$signal_source_dir/$signal_archive_name"); then
        die "neither sha256sum nor shasum is available; self-test cannot run without a checksum tool"
    fi
    printf '%s\n' "$signal_checksum" > "$signal_source_dir/$signal_archive_name.sha256"
    signal_real_mv=$(command -v mv)
    signal_perl=$(command -v perl || true)
    cat > "$signal_bin/mv" <<'EOF'
#!/bin/sh
case "${3:-}" in
    */cerulion)
        printf '%s\n' "$$" > "$CERULION_SELF_TEST_SIGNAL_CHILD_PID"
        : > "$CERULION_SELF_TEST_SIGNAL_READY"
        while [ ! -e "$CERULION_SELF_TEST_SIGNAL_RELEASE" ]; do
            sleep 1 &
            signal_sleep_pid=$!
            printf '%s\n' "$signal_sleep_pid" >> "$CERULION_SELF_TEST_SIGNAL_CHILD_PID"
            wait "$signal_sleep_pid" || :
        done
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_MV" "$@"
EOF
    cat > "$signal_bin/launch" <<'EOF'
#!/bin/sh
trap - HUP INT TERM
if [ -n "${CERULION_SELF_TEST_PERL:-}" ]; then
    exec "$CERULION_SELF_TEST_PERL" -e \
        '$SIG{HUP} = "DEFAULT"; $SIG{INT} = "DEFAULT"; $SIG{TERM} = "DEFAULT"; exec @ARGV' "$@"
fi
exec "$@"
EOF
    chmod +x "$signal_bin/mv"
    chmod +x "$signal_bin/launch"
    signal_previous_path=$PATH
    for signal in HUP INT TERM; do
        signal_dir="$owner_test_dir/signal-$signal"
        signal_install_dir="$signal_dir/install"
        signal_ready="$signal_dir/ready"
        signal_release="$signal_dir/release"
        signal_child_pid="$signal_dir/child.pid"
        mkdir -p "$signal_dir" "$signal_install_dir"
        rm -f "$signal_child_pid"
        for binary in cerulion cerulion-netd cerulion-connectd; do
            printf '%s\n' "old-$binary" > "$signal_install_dir/$binary"
        done
        PATH="$signal_bin:$signal_previous_path"
        export PATH
        export CERULION_SELF_TEST_REAL_MV="$signal_real_mv"
        export CERULION_SELF_TEST_PERL="$signal_perl"
        export CERULION_SELF_TEST_SIGNAL_READY="$signal_ready"
        export CERULION_SELF_TEST_SIGNAL_RELEASE="$signal_release"
        export CERULION_SELF_TEST_SIGNAL_CHILD_PID="$signal_child_pid"
        if [ "$signal" = INT ] && [ -z "$signal_perl" ]; then
            printf '%s\n' \
                "install.sh self-test: skipping INT signal arm because perl is unavailable to reset inherited signal dispositions" >&2
            continue
        fi
        "$signal_bin/launch" "$0" --version v0.1.0 \
            --dir "$signal_install_dir" \
            --base-url "file://$signal_fixture_dir" \
            >"$signal_dir/output" 2>"$signal_dir/error" &
        signal_pid=$!
        signal_ready_seen=0
        signal_attempt=0
        while [ "$signal_attempt" -lt 100 ]; do
            if [ -e "$signal_ready" ]; then
                signal_ready_seen=1
                break
            fi
            if ! kill -0 "$signal_pid" 2>/dev/null; then
                break
            fi
            signal_attempt=$((signal_attempt + 1))
            sleep 1
        done
        [ "$signal_ready_seen" -eq 1 ] ||
            die "self-test: $signal install did not reach its in-progress state"
        kill "-$signal" "$signal_pid"
        sleep 1
        : > "$signal_release"
        if wait "$signal_pid"; then
            die "self-test: $signal install unexpectedly succeeded"
        else
            signal_status=$?
        fi
        case "$signal" in
        HUP) signal_expected_status=129 ;;
        INT) signal_expected_status=130 ;;
        TERM) signal_expected_status=143 ;;
        esac
        [ "$signal_status" -eq "$signal_expected_status" ] ||
            die "self-test: $signal install returned status $signal_status, expected $signal_expected_status"
        if has_descendant "$signal_pid"; then
            die "self-test: $signal install left a descendant process running"
        fi
        [ -s "$signal_child_pid" ] ||
            die "self-test: $signal install did not record its foreground child"
        while IFS= read -r signal_child; do
            if is_running_pid "$signal_child"; then
                die "self-test: $signal install left a descendant process running"
            fi
        done < "$signal_child_pid"
        [ ! -e "$signal_install_dir/.cerulion-install.lock" ] ||
            die "self-test: $signal install stranded its lock"
        for binary in cerulion cerulion-netd cerulion-connectd; do
            [ "$(cat "$signal_install_dir/$binary")" = "old-$binary" ] ||
                die "self-test: $signal install did not restore old $binary bytes"
        done
        for leftover_path in "$signal_install_dir"/.cerulion-install.* \
            "$signal_install_dir"/.cerulion-install-lock.*; do
            [ -e "$leftover_path" ] ||
                continue
            die "self-test: $signal install left temporary files"
        done
    done
    signal_dir="$owner_test_dir/signal-double"
    signal_install_dir="$signal_dir/install"
    signal_ready="$signal_dir/ready"
    signal_release="$signal_dir/release"
    signal_child_pid="$signal_dir/child.pid"
    mkdir -p "$signal_dir" "$signal_install_dir"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "old-$binary" > "$signal_install_dir/$binary"
    done
    PATH="$signal_bin:$signal_previous_path"
    export PATH
    export CERULION_SELF_TEST_SIGNAL_READY="$signal_ready"
    export CERULION_SELF_TEST_SIGNAL_RELEASE="$signal_release"
    export CERULION_SELF_TEST_SIGNAL_CHILD_PID="$signal_child_pid"
    "$signal_bin/launch" "$0" --version v0.1.0 \
        --dir "$signal_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$signal_dir/output" 2>"$signal_dir/error" &
    signal_pid=$!
    signal_ready_seen=0
    signal_attempt=0
    while [ "$signal_attempt" -lt 100 ]; do
        if [ -e "$signal_ready" ]; then
            signal_ready_seen=1
            break
        fi
        if ! kill -0 "$signal_pid" 2>/dev/null; then
            break
        fi
        signal_attempt=$((signal_attempt + 1))
        sleep 1
    done
    [ "$signal_ready_seen" -eq 1 ] ||
        die "self-test: double-signal install did not reach its in-progress state"
    kill -HUP "$signal_pid"
    kill -TERM "$signal_pid" 2>/dev/null || :
    sleep 1
    : > "$signal_release"
    wait "$signal_pid" || :
    if has_descendant "$signal_pid"; then
        die "self-test: double-signal install left a descendant process running"
    fi
    [ -s "$signal_child_pid" ] ||
        die "self-test: double-signal install did not record its foreground child"
    while IFS= read -r signal_child; do
        if is_running_pid "$signal_child"; then
            die "self-test: double-signal install left a descendant process running"
        fi
    done < "$signal_child_pid"
    [ ! -e "$signal_install_dir/.cerulion-install.lock" ] ||
        die "self-test: double-signal install stranded its lock"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ "$(cat "$signal_install_dir/$binary")" = "old-$binary" ] ||
            die "self-test: double-signal install did not restore old $binary bytes"
    done
    for leftover_path in "$signal_install_dir"/.cerulion-install.* \
        "$signal_install_dir"/.cerulion-install-lock.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: double-signal install left temporary files"
    done
    PATH=$signal_previous_path
    export PATH

    run_wsl_self_test_case() {
        wsl_case=$1
        wsl_case_osrelease=$2
        wsl_case_proc_version=$3
        wsl_case_install_dir="$owner_test_dir/wsl-$wsl_case-install"
        wsl_case_osrelease_path="$owner_test_dir/wsl-$wsl_case-osrelease"
        wsl_case_proc_version_path="$owner_test_dir/wsl-$wsl_case-proc-version"
        wsl_case_output="$owner_test_dir/wsl-$wsl_case-output"
        wsl_case_error="$owner_test_dir/wsl-$wsl_case-error"
        mkdir -p "$wsl_case_install_dir"
        printf '%s\n' "$wsl_case_osrelease" > "$wsl_case_osrelease_path"
        printf '%s\n' "$wsl_case_proc_version" > "$wsl_case_proc_version_path"
        if ! CERULION_WSL_OSRELEASE_PATH="$wsl_case_osrelease_path" \
            CERULION_WSL_PROC_VERSION_PATH="$wsl_case_proc_version_path" \
            CERULION_WSL_KERNEL_OS=Linux "$0" \
            --version v0.1.0 \
            --dir "$wsl_case_install_dir" \
            --base-url "file://$signal_fixture_dir" \
            >"$wsl_case_output" 2>"$wsl_case_error"; then
            die "self-test: WSL $wsl_case warning arm failed to install"
        fi
        grep -F 'warning: WSL2 is unverified and unsupported; proceeding with the Linux artifact' \
            "$wsl_case_error" >/dev/null ||
            die "self-test: WSL $wsl_case warning was not emitted"
        for binary in cerulion cerulion-netd cerulion-connectd; do
            [ "$(cat "$wsl_case_install_dir/$binary")" = "new-$binary" ] ||
                die "self-test: WSL $wsl_case warning arm did not install $binary"
        done
    }
    run_wsl_self_test_case osrelease \
        'Linux version 6.1.0-microsoft-standard-WSL2' \
        'Linux version 6.1.0-generic'
    run_wsl_self_test_case proc-version \
        'Linux version 6.1.0-generic' \
        'Linux version 4.4.0-Microsoft'

    # The C toolchain note, on the archive shape that does not provision Rust,
    # which is also the shape that says least on its own. The probe is named
    # rather than assumed so both arms are reachable on a host that has a
    # linker and on one that does not.
    run_c_toolchain_self_test_case() {
        toolchain_case=$1
        toolchain_case_compiler=$2
        toolchain_case_os=$3
        toolchain_case_os_release=$4
        toolchain_case_expected=$5
        toolchain_case_xcode_select=$6
        toolchain_case_install_dir="$owner_test_dir/cc-$toolchain_case-install"
        toolchain_case_os_release_path="$owner_test_dir/cc-$toolchain_case-os-release"
        toolchain_case_output="$owner_test_dir/cc-$toolchain_case-output"
        toolchain_case_error="$owner_test_dir/cc-$toolchain_case-error"
        mkdir -p "$toolchain_case_install_dir"
        printf '%s\n' "$toolchain_case_os_release" > "$toolchain_case_os_release_path"
        if ! CERULION_C_COMPILER="$toolchain_case_compiler" \
            CERULION_TOOLCHAIN_KERNEL_OS="$toolchain_case_os" \
            CERULION_XCODE_SELECT="$toolchain_case_xcode_select" \
            CERULION_OS_RELEASE_PATH="$toolchain_case_os_release_path" "$0" \
            --version v0.1.0 \
            --dir "$toolchain_case_install_dir" \
            --base-url "file://$signal_fixture_dir" \
            >"$toolchain_case_output" 2>"$toolchain_case_error"; then
            cat "$toolchain_case_error" >&2
            die "self-test: C toolchain $toolchain_case arm failed to install"
        fi
        for binary in cerulion cerulion-netd cerulion-connectd; do
            [ "$(cat "$toolchain_case_install_dir/$binary")" = "new-$binary" ] ||
                die "self-test: C toolchain $toolchain_case arm did not install $binary"
        done
        if [ -z "$toolchain_case_expected" ]; then
            if grep -Fq 'NOTE: no C linker' "$toolchain_case_error"; then
                cat "$toolchain_case_error" >&2
                die "self-test: C toolchain $toolchain_case arm warned with a linker present"
            fi
            return 0
        fi
        grep -Fq 'NOTE: no C linker' "$toolchain_case_error" ||
            die "self-test: C toolchain $toolchain_case arm did not report the missing linker"
        grep -Fq "$toolchain_case_expected" "$toolchain_case_error" || {
            cat "$toolchain_case_error" >&2
            die "self-test: C toolchain $toolchain_case arm did not name its remedy"
        }
    }
    # `sh` stands in for a linker that is present: away from Darwin the
    # installer only asks whether the named command resolves on PATH.
    run_c_toolchain_self_test_case present sh Linux \
        'ID=ubuntu' \
        '' \
        cerulion-self-test-absent-xcode-select
    run_c_toolchain_self_test_case debian cerulion-self-test-absent-linker Linux \
        'ID=ubuntu' \
        'sudo apt-get install -y build-essential git' \
        cerulion-self-test-absent-xcode-select
    # The two Darwin arms carry the production shape: a compiler command that
    # RESOLVES, because every Mac ships one, with the developer directory as
    # the only difference between them. An arm that named an unresolvable
    # compiler would pass whether Darwin read PATH or the developer directory.
    run_c_toolchain_self_test_case macos sh Darwin \
        '' \
        'xcode-select --install' \
        cerulion-self-test-absent-xcode-select
    run_c_toolchain_self_test_case macos-tools-present cerulion-self-test-absent-linker Darwin \
        '' \
        '' \
        true
    run_c_toolchain_self_test_case other cerulion-self-test-absent-linker Linux \
        'ID=alpine' \
        'install your system C compiler and linker before building nodes' \
        cerulion-self-test-absent-xcode-select

    upgrade_install_dir="$owner_test_dir/upgrade-install"
    upgrade_output="$owner_test_dir/upgrade-output"
    upgrade_error="$owner_test_dir/upgrade-error"
    mkdir -p "$upgrade_install_dir"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "old-$binary" > "$upgrade_install_dir/$binary"
    done
    if ! "$0" --version v0.1.0 \
        --dir "$upgrade_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$upgrade_output" 2>"$upgrade_error"; then
        die "self-test: existing installation upgrade failed"
    fi
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ "$(cat "$upgrade_install_dir/$binary")" = "new-$binary" ] ||
            die "self-test: upgrade did not install new $binary bytes"
    done
    for leftover_path in "$upgrade_install_dir"/.cerulion-install.* \
        "$upgrade_install_dir"/.cerulion-install-lock.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: successful upgrade left temporary files"
    done
    [ "$(cat "$upgrade_install_dir/.cerulion-provenance.json")" = \
        '{"method":"install.sh","version":"v0.1.0"}' ] ||
        die "self-test: the install marker was not written"
    for leftover_path in "$upgrade_install_dir"/.cerulion-provenance.json.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: the install marker left a temporary file"
    done

    # An archive that carries the ROS 2 Jazzy rmw and the heap hook installs
    # both beside the CLI and says so; the library-less upgrade archive above
    # is the control that older releases and macOS archives still install
    # without a word about either.
    rmw_fixture_dir="$owner_test_dir/rmw-fixture"
    rmw_source_dir="$rmw_fixture_dir/v0.1.0"
    rmw_archive_root="$rmw_fixture_dir/$signal_archive_stem"
    rmw_install_dir="$owner_test_dir/rmw-install"
    rmw_output="$owner_test_dir/rmw-output"
    rmw_error="$owner_test_dir/rmw-error"
    mkdir -p "$rmw_source_dir" "$rmw_archive_root" "$rmw_install_dir"
    for binary in cerulion cerulion-netd cerulion-connectd \
        librmw_cerulion.so libcerulion_heaphook.so; do
        printf '%s\n' "new-$binary" > "$rmw_archive_root/$binary"
    done
    printf '%s\n' 'license' > "$rmw_archive_root/LICENSE"
    tar -czf "$rmw_source_dir/$signal_archive_name" \
        -C "$rmw_fixture_dir" "$signal_archive_stem"
    if ! rmw_checksum=$(sha256_digest "$rmw_source_dir/$signal_archive_name"); then
        die "neither sha256sum nor shasum is available; self-test cannot run without a checksum tool"
    fi
    printf '%s\n' "$rmw_checksum" > "$rmw_source_dir/$signal_archive_name.sha256"
    if ! "$0" --version v0.1.0 \
        --dir "$rmw_install_dir" \
        --base-url "file://$rmw_fixture_dir" \
        >"$rmw_output" 2>"$rmw_error"; then
        cat "$rmw_error" >&2
        die "self-test: install of an archive carrying the rmw library failed"
    fi
    for binary in cerulion cerulion-netd cerulion-connectd \
        librmw_cerulion.so libcerulion_heaphook.so; do
        [ "$(cat "$rmw_install_dir/$binary")" = "new-$binary" ] ||
            die "self-test: rmw archive install did not place new $binary bytes"
    done
    for library in librmw_cerulion.so libcerulion_heaphook.so; do
        grep -Fq "$library" "$rmw_output" ||
            die "self-test: rmw archive install did not report $library"
        if grep -Fq "$library" "$upgrade_output"; then
            die "self-test: library-less archive install reported $library"
        fi
        [ ! -e "$upgrade_install_dir/$library" ] ||
            die "self-test: library-less archive install produced $library"
    done

    retry_install_dir="$owner_test_dir/retry-install"
    retry_bin="$owner_test_dir/retry-bin"
    retry_marker="$owner_test_dir/retry-lock-failed"
    mkdir -p "$retry_install_dir" "$retry_bin"
    retry_real_ln=$(command -v ln)
    cat > "$retry_bin/ln" <<'EOF'
#!/bin/sh
if [ ! -e "$CERULION_SELF_TEST_RETRY_MARKER" ]; then
    : > "$CERULION_SELF_TEST_RETRY_MARKER"
    "$CERULION_SELF_TEST_REAL_LN" "$@" || exit 1
    exit 1
fi
rm -f "$2"
exec "$CERULION_SELF_TEST_REAL_LN" "$@"
EOF
    chmod +x "$retry_bin/ln"
    export CERULION_SELF_TEST_REAL_LN="$retry_real_ln"
    export CERULION_SELF_TEST_RETRY_MARKER="$retry_marker"
    PATH="$retry_bin:$signal_previous_path"
    export PATH
    if ! "$0" --version v0.1.0 \
        --dir "$retry_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$owner_test_dir/retry-output" 2>"$owner_test_dir/retry-error"; then
        die "self-test: lock-claim retry failed"
    fi
    [ -e "$retry_marker" ] ||
        die "self-test: lock-claim retry did not exercise its failed attempt"
    [ ! -e "$retry_install_dir/.cerulion-install.lock" ] ||
        die "self-test: lock-claim retry stranded its lock"
    for leftover_path in "$retry_install_dir"/.cerulion-install.* \
        "$retry_install_dir"/.cerulion-install-lock.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: lock-claim retry left temporary files"
    done
    PATH=$signal_previous_path
    export PATH

    directory_lock_install_dir="$owner_test_dir/directory-lock-install"
    directory_lock_bin="$owner_test_dir/directory-lock-bin"
    directory_lock_marker="$owner_test_dir/directory-lock-created"
    mkdir -p "$directory_lock_install_dir" "$directory_lock_bin"
    directory_lock_real_ln=$(command -v ln)
    cat > "$directory_lock_bin/ln" <<'EOF'
#!/bin/sh
if [ ! -e "$CERULION_SELF_TEST_DIRECTORY_LOCK_MARKER" ]; then
    : > "$CERULION_SELF_TEST_DIRECTORY_LOCK_MARKER"
    mkdir "$2"
fi
exec "$CERULION_SELF_TEST_REAL_DIRECTORY_LOCK_LN" "$@"
EOF
    chmod +x "$directory_lock_bin/ln"
    export CERULION_SELF_TEST_REAL_DIRECTORY_LOCK_LN="$directory_lock_real_ln"
    export CERULION_SELF_TEST_DIRECTORY_LOCK_MARKER="$directory_lock_marker"
    PATH="$directory_lock_bin:$signal_previous_path"
    export PATH
    if "$0" --version v0.1.0 \
        --dir "$directory_lock_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$owner_test_dir/directory-lock-output" \
        2>"$owner_test_dir/directory-lock-error"; then
        die "self-test: directory lock claim unexpectedly succeeded"
    fi
    [ -e "$directory_lock_marker" ] ||
        die "self-test: directory lock claim did not exercise the race"
    [ -d "$directory_lock_install_dir/.cerulion-install.lock" ] ||
        die "self-test: directory lock race did not create its directory"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ ! -e "$directory_lock_install_dir/$binary" ] ||
            die "self-test: directory lock race installed $binary"
    done
    for leftover_path in "$directory_lock_install_dir"/.cerulion-install.* \
        "$directory_lock_install_dir"/.cerulion-install-lock.*; do
        [ "$leftover_path" = "$directory_lock_install_dir/.cerulion-install.lock" ] &&
            continue
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: directory lock race left temporary files"
    done
    for leftover_path in "$directory_lock_install_dir/.cerulion-install.lock"/*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: directory lock race left a stray link"
    done
    PATH=$signal_previous_path
    export PATH

    replacement_install_dir="$owner_test_dir/replacement-lock-install"
    replacement_bin="$owner_test_dir/replacement-lock-bin"
    replacement_marker="$owner_test_dir/replacement-lock-replaced"
    mkdir -p "$replacement_install_dir" "$replacement_bin"
    replacement_real_cp=$(command -v cp)
    cat > "$replacement_bin/cp" <<'EOF'
#!/bin/sh
case "${2:-}" in
    */staged/cerulion)
        if [ ! -e "$CERULION_SELF_TEST_REPLACEMENT_MARKER" ]; then
            : > "$CERULION_SELF_TEST_REPLACEMENT_MARKER"
            rm -f "$CERULION_SELF_TEST_REPLACEMENT_LOCK"
            printf '%s\n' 'replacement-owner' > "$CERULION_SELF_TEST_REPLACEMENT_LOCK"
        fi
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_REPLACEMENT_CP" "$@"
EOF
    chmod +x "$replacement_bin/cp"
    export CERULION_SELF_TEST_REAL_REPLACEMENT_CP="$replacement_real_cp"
    export CERULION_SELF_TEST_REPLACEMENT_MARKER="$replacement_marker"
    export CERULION_SELF_TEST_REPLACEMENT_LOCK="$replacement_install_dir/.cerulion-install.lock"
    PATH="$replacement_bin:$signal_previous_path"
    export PATH
    if "$0" --version v0.1.0 \
        --dir "$replacement_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$owner_test_dir/replacement-lock-output" \
        2>"$owner_test_dir/replacement-lock-error"; then
        die "self-test: replaced lock claim unexpectedly proceeded"
    fi
    [ -e "$replacement_marker" ] ||
        die "self-test: replaced lock claim did not exercise the race"
    [ "$(cat "$replacement_install_dir/.cerulion-install.lock")" = "replacement-owner" ] ||
        die "self-test: replaced lock claim was removed"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ ! -e "$replacement_install_dir/$binary" ] ||
            die "self-test: replaced lock claim installed $binary"
    done
    PATH=$signal_previous_path
    export PATH

    partial_install_dir="$owner_test_dir/partial-upgrade-install"
    partial_bin="$owner_test_dir/partial-upgrade-bin"
    partial_marker="$owner_test_dir/partial-upgrade-failed"
    partial_output="$owner_test_dir/partial-upgrade-output"
    partial_error="$owner_test_dir/partial-upgrade-error"
    mkdir -p "$partial_install_dir" "$partial_bin"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "old-$binary" > "$partial_install_dir/$binary"
    done
    partial_real_mv=$(command -v mv)
    cat > "$partial_bin/mv" <<'EOF'
#!/bin/sh
destination=
for argument do
    destination=$argument
done
case "$destination" in
    */cerulion-netd)
        if [ ! -e "$CERULION_SELF_TEST_PARTIAL_MARKER" ]; then
            : > "$CERULION_SELF_TEST_PARTIAL_MARKER"
            exit 1
        fi
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_MV" "$@"
EOF
    chmod +x "$partial_bin/mv"
    export CERULION_SELF_TEST_REAL_MV="$partial_real_mv"
    export CERULION_SELF_TEST_PARTIAL_MARKER="$partial_marker"
    PATH="$partial_bin:$signal_previous_path"
    export PATH
    if "$0" --version v0.1.0 \
        --dir "$partial_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$partial_output" 2>"$partial_error"; then
        die "self-test: partial activation failure unexpectedly succeeded"
    fi
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ "$(cat "$partial_install_dir/$binary")" = "old-$binary" ] ||
            die "self-test: rollback did not restore old $binary bytes"
    done
    [ ! -e "$partial_install_dir/.cerulion-install.lock" ] ||
        die "self-test: partial activation failure stranded its lock"
    for leftover_path in "$partial_install_dir"/.cerulion-install.* \
        "$partial_install_dir"/.cerulion-install-lock.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: failed upgrade left temporary files"
    done
    grep -F 'could not activate cerulion-netd' "$partial_error" >/dev/null ||
        die "self-test: activation failure did not report its binary"
    PATH=$signal_previous_path
    export PATH

    directory_install_dir="$owner_test_dir/activation-directory-install"
    directory_bin="$owner_test_dir/activation-directory-bin"
    directory_marker="$owner_test_dir/activation-directory-created"
    directory_output="$owner_test_dir/activation-directory-output"
    directory_error="$owner_test_dir/activation-directory-error"
    mkdir -p "$directory_install_dir" "$directory_bin"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "old-$binary" > "$directory_install_dir/$binary"
    done
    directory_real_mv=$(command -v mv)
    cat > "$directory_bin/mv" <<'EOF'
#!/bin/sh
destination=
for argument do
    destination=$argument
done
case "$destination" in
    */cerulion)
        if [ ! -e "$CERULION_SELF_TEST_DIRECTORY_MARKER" ]; then
            : > "$CERULION_SELF_TEST_DIRECTORY_MARKER"
            rm -f "$destination"
            mkdir "$destination"
        fi
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_DIRECTORY_MV" "$@"
EOF
    chmod +x "$directory_bin/mv"
    export CERULION_SELF_TEST_REAL_DIRECTORY_MV="$directory_real_mv"
    export CERULION_SELF_TEST_DIRECTORY_MARKER="$directory_marker"
    PATH="$directory_bin:$signal_previous_path"
    export PATH
    if "$0" --version v0.1.0 \
        --dir "$directory_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$directory_output" 2>"$directory_error"; then
        die "self-test: activation directory race unexpectedly succeeded"
    fi
    [ -e "$directory_marker" ] ||
        die "self-test: activation directory race did not exercise the race"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        [ "$(cat "$directory_install_dir/$binary")" = "old-$binary" ] ||
            die "self-test: activation directory race did not restore old $binary bytes"
        if [ -d "$directory_install_dir/$binary" ]; then
            die "self-test: activation directory race left a destination directory"
        fi
    done
    [ ! -e "$directory_install_dir/.cerulion-install.lock" ] ||
        die "self-test: activation directory race stranded its lock"
    for leftover_path in "$directory_install_dir"/.cerulion-install.* \
        "$directory_install_dir"/.cerulion-install-lock.*; do
        [ -e "$leftover_path" ] ||
            continue
        die "self-test: activation directory race left temporary files"
    done
    PATH=$signal_previous_path
    export PATH

    checksum_install_dir="$owner_test_dir/activation-checksum-install"
    checksum_bin="$owner_test_dir/activation-checksum-bin"
    checksum_marker="$owner_test_dir/activation-checksum-created"
    checksum_output="$owner_test_dir/activation-checksum-output"
    checksum_error="$owner_test_dir/activation-checksum-error"
    mkdir -p "$checksum_install_dir" "$checksum_bin"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' "old-$binary" > "$checksum_install_dir/$binary"
    done
    checksum_real_mv=$(command -v mv)
    checksum_real_sha256sum=$(command -v sha256sum || true)
    [ -n "$checksum_real_sha256sum" ] ||
        die "self-test: checksum-failure arm requires sha256sum"
    cat > "$checksum_bin/mv" <<'EOF'
#!/bin/sh
if [ "$1" = --version ]; then
    printf '%s\n' 'portable mv'
    exit 0
fi
destination=
for argument do
    destination=$argument
done
case "$destination" in
    */cerulion)
        if [ ! -e "$CERULION_SELF_TEST_CHECKSUM_MARKER" ]; then
            : > "$CERULION_SELF_TEST_CHECKSUM_MARKER"
            rm -f "$destination"
            mkdir "$destination"
        fi
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_CHECKSUM_MV" "$@"
EOF
    cat > "$checksum_bin/sha256sum" <<'EOF'
#!/bin/sh
case "$1" in
    */activation-checksum-install/cerulion/cerulion)
        exit 1
        ;;
esac
exec "$CERULION_SELF_TEST_REAL_SHA256SUM" "$@"
EOF
    chmod +x "$checksum_bin/mv" "$checksum_bin/sha256sum"
    export CERULION_SELF_TEST_REAL_CHECKSUM_MV="$checksum_real_mv"
    export CERULION_SELF_TEST_REAL_SHA256SUM="$checksum_real_sha256sum"
    export CERULION_SELF_TEST_CHECKSUM_MARKER="$checksum_marker"
    PATH="$checksum_bin:$signal_previous_path"
    export PATH
    if "$0" --version v0.1.0 \
        --dir "$checksum_install_dir" \
        --base-url "file://$signal_fixture_dir" \
        >"$checksum_output" 2>"$checksum_error"; then
        die "self-test: checksum-failure cleanup unexpectedly succeeded"
    fi
    [ -e "$checksum_marker" ] ||
        die "self-test: checksum-failure cleanup did not exercise the race"
    if [ ! -d "$checksum_install_dir/cerulion" ] ||
        [ ! -f "$checksum_install_dir/cerulion/cerulion" ]; then
        die "self-test: checksum-failure cleanup removed the unverified file"
    fi
    grep -F 'installation rollback did not complete' "$checksum_error" >/dev/null ||
        die "self-test: checksum-failure cleanup did not fail closed"
    [ ! -e "$checksum_install_dir/.cerulion-install.lock" ] ||
        die "self-test: checksum-failure cleanup stranded its lock"
    PATH=$signal_previous_path
    export PATH

    if command -v mkfifo >/dev/null 2>&1; then
        fifo_install_dir="$owner_test_dir/fifo-install"
        fifo_output="$owner_test_dir/fifo-output"
        fifo_error="$owner_test_dir/fifo-error"
        mkdir -p "$fifo_install_dir"
        mkfifo "$fifo_install_dir/cerulion" ||
            die "self-test: could not create FIFO destination"
        if "$0" --version v0.1.0 \
            --dir "$fifo_install_dir" \
            --base-url "file://$signal_fixture_dir" \
            >"$fifo_output" 2>"$fifo_error"; then
            die "self-test: FIFO destination was accepted"
        fi
        grep -F "installation destination is a non-regular, non-symlink file (fifo): $fifo_install_dir/cerulion" \
            "$fifo_error" >/dev/null ||
            die "self-test: FIFO destination error did not name its type and path"
        [ ! -e "$fifo_install_dir/.cerulion-install.lock" ] ||
            die "self-test: FIFO refusal stranded its lock"
        for leftover_path in "$fifo_install_dir"/.cerulion-install.* \
            "$fifo_install_dir"/.cerulion-install-lock.*; do
            [ -e "$leftover_path" ] ||
                continue
            die "self-test: FIFO refusal left temporary files"
        done
    else
        printf '%s\n' \
            "install.sh self-test: skipping FIFO destination arm because mkfifo is unavailable" >&2
    fi

    path_setup_root="$owner_test_dir/path-setup"
    mkdir -p "$path_setup_root"
    run_path_setup_install() {
        path_run_home=$1
        path_run_install=$2
        path_run_output=$3
        path_run_error=$4
        shift 4
        # env rather than a subshell assignment: the installer under test is a
        # separate process, and this keeps the harness from appearing to change
        # the variables the script itself reads.
        env -u CERULION_HOME -u CERULION_INSTALL_DIR -u CERULION_NO_MODIFY_PATH \
            HOME="$path_run_home" \
            ZDOTDIR="$path_run_home" \
            XDG_CONFIG_HOME="$path_run_home/.config" \
            "$0" --version v0.1.0 \
            --dir "$path_run_install" \
            --base-url "file://$signal_fixture_dir" \
            "$@" \
            >"$path_run_output" 2>"$path_run_error"
    }

    first_home="$path_setup_root/first-home"
    first_install="$path_setup_root/first-install"
    first_output="$path_setup_root/first-output"
    first_error="$path_setup_root/first-error"
    mkdir -p "$first_home/.config/fish" "$first_install"
    : > "$first_home/.bashrc"
    run_path_setup_install "$first_home" "$first_install" "$first_output" "$first_error" ||
        die "self-test: the PATH setup install failed"
    first_env="$first_home/.cerulion/env"
    first_env_fish="$first_home/.cerulion/env.fish"
    [ -f "$first_env" ] ||
        die "self-test: PATH setup wrote no env file"
    [ -f "$first_env_fish" ] ||
        die "self-test: PATH setup wrote no fish env file"
    grep -Fq "$first_install" "$first_env" ||
        die "self-test: the env file does not name the install directory"
    if grep -Fq '.cargo' "$first_env"; then
        die "self-test: the env file names Cargo for an archive that provisioned no Rust"
    fi
    grep -Fq 'set -gx PATH' "$first_env_fish" ||
        die "self-test: the fish env file does not set PATH"
    grep -Fq "$first_install" "$first_env_fish" ||
        die "self-test: the fish env file does not name the install directory"
    for path_profile in "$first_home/.profile" "$first_home/.zshenv" "$first_home/.bashrc"; do
        [ -f "$path_profile" ] ||
            die "self-test: PATH setup skipped $path_profile"
        grep -Fq "$PROFILE_MARKER" "$path_profile" ||
            die "self-test: $path_profile carries no marked line"
        grep -Fq "$first_env" "$path_profile" ||
            die "self-test: $path_profile does not source the env file"
    done
    for path_absent in "$first_home/.bash_profile" "$first_home/.bash_login"; do
        if [ -e "$path_absent" ]; then
            die "self-test: PATH setup created $path_absent"
        fi
    done
    first_fish_profile="$first_home/.config/fish/conf.d/cerulion.fish"
    [ -f "$first_fish_profile" ] ||
        die "self-test: PATH setup wrote no fish profile"
    grep -Fq "$first_env_fish" "$first_fish_profile" ||
        die "self-test: the fish profile does not source the fish env file"
    grep -Fq "PATH setup written to $first_env" "$first_output" ||
        die "self-test: PATH setup did not report the env file it wrote"
    grep -Fq "PATH line added to $first_home/.profile" "$first_output" ||
        die "self-test: PATH setup did not report the profile it edited"
    grep -Fq "Open a new terminal, or run . " "$first_output" ||
        die "self-test: PATH setup did not say how to use the new PATH"
    grep -Fq "cerulion login" "$first_output" ||
        die "self-test: PATH setup did not name the sign-in command"
    if grep -Fq 'Add tools to PATH' "$first_output"; then
        die "self-test: PATH setup still printed the manual export line"
    fi
    first_path_value=$(PATH=/usr/bin:/bin sh -c '. "$1"; . "$1"; printf "%s\n" "$PATH"' \
        sh "$first_env") ||
        die "self-test: the env file could not be sourced"
    first_occurrences=$(printf '%s\n' "$first_path_value" | tr ':' '\n' |
        grep -cxF "$first_install" || true)
    [ "$first_occurrences" = "1" ] ||
        die "self-test: sourcing the env file twice put the directory on PATH $first_occurrences times"
    if command -v fish >/dev/null 2>&1; then
        first_env_fish_quoted=$(fish_quote "$first_env_fish")
        first_fish_path=$(fish -c \
            "source $first_env_fish_quoted; source $first_env_fish_quoted; printf '%s\n' \$PATH") ||
            die "self-test: the fish env file could not be sourced"
        first_fish_occurrences=$(printf '%s\n' "$first_fish_path" |
            grep -cxF "$first_install" || true)
        [ "$first_fish_occurrences" = "1" ] ||
            die "self-test: sourcing the fish env file twice put the directory on PATH $first_fish_occurrences times"
    else
        printf '%s\n' \
            "install.sh self-test: skipping the fish sourcing arm because fish is unavailable" >&2
    fi

    second_output="$path_setup_root/second-output"
    second_error="$path_setup_root/second-error"
    for path_profile in .profile .zshenv .bashrc; do
        cp "$first_home/$path_profile" "$path_setup_root/before-$path_profile"
    done
    cp "$first_fish_profile" "$path_setup_root/before-fish"
    run_path_setup_install "$first_home" "$first_install" "$second_output" "$second_error" ||
        die "self-test: the second PATH setup install failed"
    for path_profile in .profile .zshenv .bashrc; do
        cmp -s "$path_setup_root/before-$path_profile" "$first_home/$path_profile" ||
            die "self-test: the second run changed $path_profile"
    done
    cmp -s "$path_setup_root/before-fish" "$first_fish_profile" ||
        die "self-test: the second run changed the fish profile"
    grep -Fq "PATH line already in $first_home/.profile" "$second_output" ||
        die "self-test: the second run did not report the line it found"

    nomod_home="$path_setup_root/nomod-home"
    nomod_install="$path_setup_root/nomod-install"
    mkdir -p "$nomod_home" "$nomod_install"
    run_path_setup_install "$nomod_home" "$nomod_install" \
        "$path_setup_root/nomod-output" "$path_setup_root/nomod-error" --no-modify-path ||
        die "self-test: the --no-modify-path install failed"
    if [ -e "$nomod_home/.cerulion/env" ] || [ -e "$nomod_home/.profile" ]; then
        die "self-test: --no-modify-path still changed the home directory"
    fi
    grep -Fq 'Add tools to PATH' "$path_setup_root/nomod-output" ||
        die "self-test: --no-modify-path printed no PATH to set"
    grep -Fq 'cerulion login' "$path_setup_root/nomod-output" ||
        die "self-test: --no-modify-path did not name the sign-in command"

    envvar_home="$path_setup_root/envvar-home"
    envvar_install="$path_setup_root/envvar-install"
    mkdir -p "$envvar_home" "$envvar_install"
    env -u CERULION_HOME -u CERULION_INSTALL_DIR \
        HOME="$envvar_home" \
        ZDOTDIR="$envvar_home" \
        XDG_CONFIG_HOME="$envvar_home/.config" \
        CERULION_NO_MODIFY_PATH=1 \
        "$0" --version v0.1.0 \
        --dir "$envvar_install" \
        --base-url "file://$signal_fixture_dir" \
        >"$path_setup_root/envvar-output" 2>"$path_setup_root/envvar-error" ||
        die "self-test: the CERULION_NO_MODIFY_PATH install failed"
    if [ -e "$envvar_home/.cerulion/env" ] || [ -e "$envvar_home/.profile" ]; then
        die "self-test: CERULION_NO_MODIFY_PATH still changed the home directory"
    fi
    grep -Fq 'Add tools to PATH' "$path_setup_root/envvar-output" ||
        die "self-test: CERULION_NO_MODIFY_PATH printed no PATH to set"

    link_home="$path_setup_root/link-home"
    link_install="$path_setup_root/link-install"
    link_outside="$path_setup_root/outside"
    mkdir -p "$link_home" "$link_install" "$link_outside"
    : > "$link_outside/profile"
    ln -s "$link_outside/profile" "$link_home/.profile"
    : > "$link_home/inside-bashrc"
    ln -s "$link_home/inside-bashrc" "$link_home/.bashrc"
    run_path_setup_install "$link_home" "$link_install" \
        "$path_setup_root/link-output" "$path_setup_root/link-error" ||
        die "self-test: the symlinked-profile install failed"
    if grep -Fq "$PROFILE_MARKER" "$link_outside/profile"; then
        die "self-test: a profile linked out of the home directory was edited"
    fi
    grep -Fq 'symlink out of your home directory' "$path_setup_root/link-error" ||
        die "self-test: the refusal to edit a linked profile was not reported"
    grep -Fq "$PROFILE_MARKER" "$link_home/inside-bashrc" ||
        die "self-test: a profile linked inside the home directory was skipped"
    if ! command -v fish >/dev/null 2>&1; then
        if [ -e "$link_home/.config/fish/conf.d/cerulion.fish" ]; then
            die "self-test: a fish profile was written on a host without fish"
        fi
    fi

    nohome_install="$path_setup_root/nohome-install"
    mkdir -p "$nohome_install"
    env -u HOME -u ZDOTDIR -u XDG_CONFIG_HOME -u CERULION_HOME \
        -u CERULION_INSTALL_DIR -u CERULION_NO_MODIFY_PATH \
        "$0" --version v0.1.0 \
        --dir "$nohome_install" \
        --base-url "file://$signal_fixture_dir" \
        >"$path_setup_root/nohome-output" 2>"$path_setup_root/nohome-error" ||
        die "self-test: the install without HOME failed"
    grep -Fq 'HOME is unset' "$path_setup_root/nohome-error" ||
        die "self-test: the install without HOME did not say why no profile changed"
    grep -Fq 'Add tools to PATH' "$path_setup_root/nohome-output" ||
        die "self-test: the install without HOME printed no PATH to set"

    cleanup_owner_test
    trap - 0
    printf '%s\n' "install.sh self-test passed"
}

resolve_install_dir() {
    requested_explicit=$1
    requested_dir=$2
    if [ "$requested_explicit" -eq 1 ]; then
        printf '%s\n' "$requested_dir"
    elif [ -n "${CERULION_INSTALL_DIR:-}" ]; then
        printf '%s\n' "$CERULION_INSTALL_DIR"
    elif [ -n "${HOME:-}" ]; then
        printf '%s\n' "$HOME/.cerulion/bin"
    else
        die "neither --dir nor CERULION_INSTALL_DIR was given and HOME is unset"
    fi
}

classify_lock_owner() {
    lock_path_to_classify=$1
    [ -f "$lock_path_to_classify" ] || {
        printf '%s\n' "invalid"
        return 0
    }
    owner=$(cat "$lock_path_to_classify" 2>/dev/null || true)
    case "$owner" in
        ''|0|*[!0-9]*)
            printf '%s\n' "invalid"
            ;;
        *)
            if kill -0 "$owner" 2>/dev/null; then
                printf '%s\n' "live"
            else
                printf '%s\n' "dead"
            fi
            ;;
    esac
}

lock_path_is_regular_or_absent() {
    lock_path_to_check=$1
    [ ! -e "$lock_path_to_check" ] || [ -f "$lock_path_to_check" ]
}

die_stranded_lock() {
    expiration_state=$1
    case "$expiration_state" in
    live)
        die "another Cerulion installation is in progress: $lock_path (lock is held by a live PID)"
        ;;
    dead)
        die "could not acquire installer lock at $lock_path: owner PID is no longer running; delete $lock_path if no installation is running"
        ;;
    invalid)
        die "could not acquire installer lock at $lock_path: owner contents are unreadable; delete $lock_path if no installation is running"
        ;;
    *)
        die "could not acquire installer lock at $lock_path: unexpected owner state: $expiration_state"
        ;;
    esac
}

download() {
    download_url=$1
    download_path=$2
    run_foreground curl -fL --retry 3 --retry-delay 1 --silent --show-error \
        "$download_url" -o "$download_path" ||
        die "download failed: $download_url"
}

verify_checksum() {
    checksum_path=$1
    archive_path=$2
    expected_checksum=$(awk 'NF { print $1; exit }' "$checksum_path")
    printf '%s' "$expected_checksum" | grep -Eq '^[[:xdigit:]]{64}$' ||
        die "invalid SHA-256 sidecar: $checksum_path"

    if ! actual_checksum=$(sha256_digest "$archive_path"); then
        die "neither sha256sum nor shasum is available; refusing an unverified download"
    fi
    [ "$actual_checksum" = "$expected_checksum" ] ||
        die "checksum mismatch for $(basename "$archive_path")"
}

version=""
install_dir=""
install_dir_explicit=0
base_url=$DEFAULT_BASE_URL
stable_base_url=$DEFAULT_STABLE_BASE_URL
base_url_explicit=0
self_test_requested=0
modify_path=1
if [ -n "${CERULION_NO_MODIFY_PATH:-}" ]; then
    modify_path=0
fi

while [ "$#" -gt 0 ]; do
    case "$1" in
        --version)
            [ "$#" -ge 2 ] || die "--version requires vX.Y.Z[-PRERELEASE]"
            version=$2
            shift 2
            ;;
        --dir)
            [ "$#" -ge 2 ] || die "--dir requires a directory"
            install_dir=$2
            install_dir_explicit=1
            shift 2
            ;;
        --base-url)
            [ "$#" -ge 2 ] || die "--base-url requires a URL"
            base_url=${2%/}
            # One override moves both roots: the channel file is served from
            # the same place as the archives, so a fixture directory holding a
            # `stable.txt` resolves its own version.
            stable_base_url=$base_url
            base_url_explicit=1
            shift 2
            ;;
        --no-modify-path)
            modify_path=0
            shift
            ;;
        --self-test)
            self_test_requested=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            die "unknown option: $1"
            ;;
    esac
done

if [ "$self_test_requested" -eq 1 ]; then
    self_test
    exit 0
fi

install_dir=$(resolve_install_dir "$install_dir_explicit" "$install_dir")

command -v curl >/dev/null 2>&1 ||
    die "curl is required to install Cerulion"
command -v tar >/dev/null 2>&1 ||
    die "tar is required to install Cerulion"

if [ -z "$version" ]; then
    stable_channel_quiet=1
    resolve_stable_release "$stable_base_url" ||
        exit 1
    version=$stable_tag
elif ! valid_version "$version"; then
    die "invalid version '$version'; expected vX.Y.Z[-PRERELEASE]"
fi

target=$(platform_target "$(uname -s)" "$(uname -m)") || {
    printf '%s\n' \
        "error: unsupported platform; build from source" >&2
    exit 1
}
if is_wsl2; then
    printf '%s\n' \
        "warning: WSL2 is unverified and unsupported; proceeding with the Linux artifact" >&2
fi

archive_name="cerulion-${version#v}-${target}.tar.gz"
archive_stem=${archive_name%.tar.gz}
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-install.XXXXXX") ||
    die "could not create a temporary directory"
foreground_child_pid=""
cleanup() {
    status=$?
    rm -rf "$workdir" || status=1
    release_lock || status=1
    exit "$status"
}
interrupt_install() {
    interrupt_status=$1
    trap '' HUP INT TERM
    if [ -n "${foreground_child_pid:-}" ]; then
        terminate_process_tree "$foreground_child_pid"
        wait "$foreground_child_pid" 2>/dev/null || :
        foreground_child_pid=""
    fi
    exit "$interrupt_status"
}
trap cleanup EXIT
trap 'interrupt_install 129' HUP
trap 'interrupt_install 130' INT
trap 'interrupt_install 143' TERM

download "${base_url}/${version}/${archive_name}" "$workdir/$archive_name"
download "${base_url}/${version}/${archive_name}.sha256" "$workdir/$archive_name.sha256"
verify_checksum "$workdir/$archive_name.sha256" "$workdir/$archive_name"

run_foreground tar -xzf "$workdir/$archive_name" -C "$workdir" ||
    die "could not extract $archive_name"
archive_dir="$workdir/$archive_stem"
archive_files="cerulion cerulion-netd cerulion-connectd"
for binary in $archive_files; do
    [ -f "$archive_dir/$binary" ] ||
        die "archive is missing $binary"
done
# The ROS 2 Jazzy rmw and the heap hook ship in the Linux archives and are
# installed beside the CLI, where `cerulion ros2 run` looks for both. Each is
# taken when the archive carries it rather than required, so releases that
# predate them and the macOS archives (ROS 2 Jazzy has no macOS binaries)
# still install.
installed_libraries=
for library in librmw_cerulion.so libcerulion_heaphook.so; do
    if [ -f "$archive_dir/$library" ]; then
        archive_files="$archive_files $library"
        installed_libraries="$installed_libraries $library"
    fi
done

# Older archives contain binaries only. New releases bundle the provisioning
# helper and the compiler metadata of the build inside the verified archive, so
# an explicit --version never selects today's release compiler by accident.
rust_setup=0
if [ -e "$archive_dir/install_rust.sh" ] || [ -e "$archive_dir/rustc-version.txt" ]; then
    if [ ! -f "$archive_dir/install_rust.sh" ] || [ ! -f "$archive_dir/rustc-version.txt" ]; then
        die "archive is missing part of its Rust/Cargo setup"
    fi
    rust_setup=1
else
    printf '%s\n' 'warning: this older release archive does not include automatic Rust/Cargo setup' >&2
fi

mkdir -p "$install_dir"
lock_path="$install_dir/.cerulion-install.lock"
lock_temp_path=
lock_claim_identity=
lock_released=0
owner_invalid_since=0
lock_attempt=1
lock_absent_attempts=0
# The hardlink is the only ownership operation: stale locks fail closed because
# release_lock covers normal exit and ordinary signals, while SIGKILL or machine
# death can leave one clearly named file for the user to delete.
while :; do
    lock_temp_path=$(mktemp "$install_dir/.cerulion-install-lock.XXXXXX") ||
        die "could not prepare ownership of install lock: $lock_path"
    printf '%s\n' "$$" > "$lock_temp_path" ||
        die "could not prepare ownership of install lock: $lock_path"
    if ! lock_path_is_regular_or_absent "$lock_path"; then
        die "could not create the install lock at $lock_path: lock path is not a regular file"
    fi
    lock_ln_error=$(ln "$lock_temp_path" "$lock_path" 2>&1) &&
    lock_claimed=1 || lock_claimed=0
    if [ "$lock_claimed" -eq 1 ]; then
        if lock_claim_matches "$lock_temp_path" "$lock_path"; then
            lock_claim_identity=$(file_identity "$lock_path") ||
                die "could not identify the acquired installer lock: $lock_path"
            rm -f "$lock_temp_path"
            break
        fi
        if [ -d "$lock_path" ]; then
            lock_stray_path="$lock_path/${lock_temp_path##*/}"
            if lock_claim_matches "$lock_temp_path" "$lock_stray_path"; then
                rm -f "$lock_stray_path" ||
                    die "could not clean up stray installer lock link: $lock_stray_path"
            fi
        fi
        rm -f "$lock_temp_path"
        die "could not create the install lock at $lock_path: lock claim changed during acquisition"
    fi
    rm -f "$lock_temp_path"
    if [ -e "$lock_path" ]; then
        lock_absent_attempts=0
    else
        lock_absent_attempts=$((lock_absent_attempts + 1))
        if [ "$lock_absent_attempts" -ge 3 ]; then
            lock_ln_error=${lock_ln_error:-unknown hardlink error}
            die "could not create the install lock at $lock_path; hardlinks may be unsupported there: $lock_ln_error"
        fi
    fi
    if [ "$lock_attempt" -ge 60 ]; then
        owner_state=$(classify_lock_owner "$lock_path")
        if [ "$owner_state" = invalid ]; then
            now=$(date +%s)
            if [ "$owner_invalid_since" -eq 0 ]; then
                owner_invalid_since=$now
            fi
            if [ "$((now - owner_invalid_since))" -lt 10 ]; then
                sleep 1
                continue
            fi
        fi
        die_stranded_lock "$owner_state"
    fi
    lock_attempt=$((lock_attempt + 1))
    owner_state=$(classify_lock_owner "$lock_path")
    case "$owner_state" in
    live|dead)
        owner_invalid_since=0
        ;;
    invalid)
        now=$(date +%s)
        if [ "$owner_invalid_since" -eq 0 ]; then
            owner_invalid_since=$now
        fi
        ;;
    esac
    sleep 1
done
cleanup_lock() {
    status=$?
    cleanup_failed=0
    rm -rf "$workdir" || cleanup_failed=1
    [ "$cleanup_failed" -eq 0 ] || status=1
    if ! release_lock; then
        status=1
    fi
    exit "$status"
}
trap cleanup_lock EXIT
# Provision only while owning the install lock, before staging any binaries.
if [ "$rust_setup" -eq 1 ]; then
    # rustup fetches its own installer and one minimal toolchain here, which is
    # several times the size of the Cerulion archive and the slow part of the
    # install on a metered or tethered link. Say so before it starts. The
    # helper this step runs fetches nothing when the exact compiler is already
    # installed, which a returning user usually has, so the size is stated as
    # the case it applies to rather than as a promise.
    printf '%s\n' \
        'Setting up Rust and Cargo for building nodes; where that compiler is not already installed this downloads about 140 MB'
    run_foreground sh "$archive_dir/install_rust.sh" "$archive_dir/rustc-version.txt" ||
        die "Rust/Cargo setup failed; Cerulion binaries have not been replaced"
fi
transaction_dir=""
replaced_binaries=""
activation_validated_binaries=""
destination_type() {
    # shellcheck disable=SC2012
    destination_mode=$(ls -ld "$1" 2>/dev/null | awk 'NR == 1 { print substr($1, 1, 1) }')
    case "$destination_mode" in
        p) printf '%s\n' fifo ;;
        s) printf '%s\n' socket ;;
        c) printf '%s\n' character-device ;;
        b) printf '%s\n' block-device ;;
        *) printf '%s\n' special-file ;;
    esac
}
cleanup_install() {
    trap '' HUP INT TERM
    status=$cleanup_status
    rollback_failed=0
    not_restored=""
    # Every destination write replaces a link or regular file; it never
    # follows a link or writes inside a real directory.
    if [ "$status" -ne 0 ] && [ -n "$transaction_dir" ] && [ -n "$replaced_binaries" ]; then
        for binary in $replaced_binaries; do
            if [ -f "$transaction_dir/backup/$binary" ] ||
                [ -L "$transaction_dir/backup/$binary" ]; then
                if [ -L "$install_dir/$binary" ]; then
                    rm -f "$install_dir/$binary" || {
                        rollback_failed=1
                        not_restored="$not_restored $binary"
                        continue
                    }
                elif [ -d "$install_dir/$binary" ]; then
                    if printf ' %s ' "$activation_validated_binaries" |
                        grep -qF " $binary "; then
                        if ! raced_activation_directory_cleanup "$binary"; then
                            printf 'error: installation destination is a directory: %s\n' \
                                "$install_dir/$binary" >&2
                            rollback_failed=1
                            not_restored="$not_restored $binary"
                            continue
                        fi
                    else
                        printf 'error: installation destination is a directory: %s\n' \
                            "$install_dir/$binary" >&2
                        rollback_failed=1
                        not_restored="$not_restored $binary"
                        continue
                    fi
                fi
                if ! mv -f "$transaction_dir/backup/$binary" "$install_dir/$binary"; then
                    rollback_failed=1
                    not_restored="$not_restored $binary"
                fi
            else
                if [ -L "$install_dir/$binary" ]; then
                    if ! rm -f "$install_dir/$binary"; then
                        rollback_failed=1
                        not_restored="$not_restored $binary"
                    fi
                elif [ -d "$install_dir/$binary" ]; then
                    if printf ' %s ' "$activation_validated_binaries" |
                        grep -qF " $binary "; then
                        if ! raced_activation_directory_cleanup "$binary"; then
                            printf 'error: installation destination is a directory: %s\n' \
                                "$install_dir/$binary" >&2
                            rollback_failed=1
                            not_restored="$not_restored $binary"
                        fi
                    else
                        printf 'error: installation destination is a directory: %s\n' \
                            "$install_dir/$binary" >&2
                        rollback_failed=1
                        not_restored="$not_restored $binary"
                    fi
                elif ! rm -f "$install_dir/$binary"; then
                    rollback_failed=1
                    not_restored="$not_restored $binary"
                fi
            fi
        done
    fi
    if [ "$rollback_failed" -ne 0 ]; then
        printf 'error: installation rollback did not complete; retained transaction directory: %s\n' \
            "$transaction_dir" >&2
        for binary in $not_restored; do
            printf 'error: binary was not restored: %s\n' "$binary" >&2
        done
        rm -rf "$workdir" || :
        release_lock
        exit 1
    fi
    if ! cleanup_staging "$transaction_dir" "$workdir"; then
        status=1
    fi
    exit "$status"
}
transaction_dir=$(mktemp -d "$install_dir/.cerulion-install.XXXXXX") ||
    die "could not create an installation staging directory"
cleanup_status=0
trap 'cleanup_status=$?; cleanup_install' EXIT
trap 'interrupt_install 129' HUP
trap 'interrupt_install 130' INT
trap 'interrupt_install 143' TERM
mkdir "$transaction_dir/staged" "$transaction_dir/backup"
for binary in $archive_files; do
    run_foreground cp "$archive_dir/$binary" "$transaction_dir/staged/$binary" ||
        die "could not stage $binary"
    chmod +x "$transaction_dir/staged/$binary"
    if [ ! -f "$transaction_dir/staged/$binary" ] ||
        [ ! -x "$transaction_dir/staged/$binary" ]; then
        die "staged binary is not executable: $binary"
    fi
    if [ -L "$install_dir/$binary" ]; then
        run_foreground cp -pP "$install_dir/$binary" "$transaction_dir/backup/$binary" ||
            die "could not back up installation destination: $install_dir/$binary"
    elif [ -d "$install_dir/$binary" ]; then
        die "installation destination is a directory: $install_dir/$binary"
    elif [ -f "$install_dir/$binary" ]; then
        run_foreground cp -pP "$install_dir/$binary" "$transaction_dir/backup/$binary" ||
            die "could not back up installation destination: $install_dir/$binary"
    elif [ -e "$install_dir/$binary" ]; then
        die "installation destination is a non-regular, non-symlink file ($(destination_type "$install_dir/$binary")): $install_dir/$binary"
    fi
done
# Recorded BEFORE the move: a signal caught in the gap after a completed move
# would otherwise leave that binary new and its siblings behind. Rolling back a
# move that never happened is a no-op.
for binary in $archive_files; do
    replaced_binaries="$replaced_binaries $binary"
    if [ -L "$install_dir/$binary" ]; then
        rm -f "$install_dir/$binary"
    # Keep regular files for mv -f: it atomically replaces the upgrade
    # destination. Check symlinks first because -f follows them.
    elif [ -f "$install_dir/$binary" ]; then
        :
    elif [ -d "$install_dir/$binary" ]; then
        die "installation destination is a directory: $install_dir/$binary"
    elif [ -e "$install_dir/$binary" ]; then
        die "installation destination is a non-regular, non-symlink file ($(destination_type "$install_dir/$binary")): $install_dir/$binary"
    fi
    lock_claim_is_current ||
        die "could not activate $binary: installer lock changed"
    activation_validated_binaries="$activation_validated_binaries $binary"
    staged_digest=$(sha256_digest "$transaction_dir/staged/$binary") ||
        die "could not checksum staged $binary"
    if mv --version 2>/dev/null | grep -qF 'GNU coreutils'; then
        run_foreground mv -T -f "$transaction_dir/staged/$binary" "$install_dir/$binary" || :
    else
        run_foreground mv -f "$transaction_dir/staged/$binary" "$install_dir/$binary" || :
    fi
    if [ "$foreground_status" -ne 0 ]; then
        die "could not activate $binary"
    fi
    if [ ! -f "$install_dir/$binary" ] ||
        [ -d "$install_dir/$binary" ] ||
        [ "$(sha256_digest "$install_dir/$binary" 2>/dev/null || :)" != "$staged_digest" ]; then
        die "could not verify activated $binary"
    fi
done

write_install_marker
printf 'Installed Cerulion %s for %s into %s\n' "$version" "$target" "$install_dir"
for library in $installed_libraries; do
    case "$library" in
        librmw_cerulion.so)
            printf 'The ROS 2 Jazzy rmw (%s) is installed beside the CLI for cerulion ros2 run and launch\n' \
                "$library"
            ;;
        libcerulion_heaphook.so)
            printf 'The Cerulion heap hook (%s) is installed beside the CLI; cerulion ros2 run and launch preload it into ROS 2 nodes\n' \
                "$library"
            ;;
    esac
done
cargo_bin_dir=""
if [ "$rust_setup" -eq 1 ]; then
    cargo_bin_dir="${CARGO_HOME:-$HOME/.cargo}/bin"
fi
# The profile files are touched only here, after every binary is activated and
# verified, so a run that fails earlier never leaves a line pointing at nothing.
# A failed edit is reported and the PATH is printed instead; it never fails the
# install, which has already succeeded by this point.
path_setup_done=0
if [ "$modify_path" -eq 1 ]; then
    if [ -n "${HOME:-}" ]; then
        if setup_path_for_shells "$install_dir" "$cargo_bin_dir"; then
            path_setup_done=1
        fi
    else
        printf '%s\n' \
            'NOTE: HOME is unset, so no shell profile was changed.' >&2
    fi
fi
if [ "$path_setup_done" -eq 0 ]; then
    path_prefix=$install_dir
    if [ -n "$cargo_bin_dir" ]; then
        path_prefix="$path_prefix:$cargo_bin_dir"
    fi
    quoted_path_prefix=$(printf '%s' "$path_prefix" | sed "s/'/'\\\\''/g")
    # Expand PATH in the user's shell, not while printing these instructions.
    # shellcheck disable=SC2016
    printf 'Add tools to PATH: export PATH=%s:"$PATH"\n' "'$quoted_path_prefix'"
    printf '%s\n' 'Then sign in once with cerulion login'
fi
# Both arms reach here: an archive that provisioned Rust, and an older
# binaries-only archive that did not. Either way a missing linker is only
# discovered at the first node build unless it is named now. Advice is never
# the reason an install reports failure, so its status is discarded.
report_c_toolchain || :
