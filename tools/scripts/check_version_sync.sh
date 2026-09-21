#!/usr/bin/env bash
# check_version_sync.sh — Guard that [workspace.package].version and every
# internal [workspace.dependencies] version string in the root Cargo.toml stay
# in lockstep. Run this before every crates.io publish so one workspace-version
# edit cannot leave a path dependency pinned to an older release.
#
# Usage: bash scripts/check_version_sync.sh
# Exit 0 = all versions match; Exit 1 = mismatch found (printed to stderr).
set -euo pipefail

CARGO_TOML="${1:-Cargo.toml}"

if [[ ! -f "$CARGO_TOML" ]]; then
    echo "error: $CARGO_TOML not found (run from workspace root)" >&2
    exit 1
fi

# Extract [workspace.package] version (the canonical value).
script_path=${BASH_SOURCE[0]}
script_dir=${script_path%/*}
if [[ "$script_dir" == "$script_path" ]]; then
    script_dir=.
elif [[ "$script_dir" == -* ]]; then
    script_dir="./$script_dir"
fi
script_dir="$(CDPATH='' cd "$script_dir" && pwd)"
if ! pkg_version=$("$script_dir/workspace_version.sh" "$CARGO_TOML"); then
    echo "error: could not parse [workspace.package].version from $CARGO_TOML" >&2
    exit 1
fi

if [[ -z "$pkg_version" ]]; then
    echo "error: could not parse [workspace.package].version from $CARGO_TOML" >&2
    exit 1
fi

# Extract every path dependency with an explicit version under the root
# [workspace.dependencies] table, including multiline inline tables and
# [workspace.dependencies.<name>] subtables.
dep_versions=$(
    awk '
        function fail_unclosed_inline() {
            printf "error: inline dependency table for %s never closed\n", \
                inline_name > "/dev/stderr"
            fatal = 1
            exit 1
        }

        function reset_inline() {
            inline_name = ""
            inline_path = 0
            inline_version = ""
        }

        function strip_toml_comment(line,   i, ch, quote, escaped) {
            for (i = 1; i <= length(line); i++) {
                ch = substr(line, i, 1)
                if (quote != "") {
                    if (escaped) {
                        escaped = 0
                    } else if (ch == "\\") {
                        escaped = 1
                    } else if (ch == quote) {
                        quote = ""
                    }
                } else if (ch == "\"" || ch == single_quote) {
                    quote = ch
                } else if (ch == "#") {
                    return substr(line, 1, i - 1)
                }
            }
            return line
        }

        function scan_value(line, key,   clean, rest, quote, escaped, i, ch,
                            assignment_pos) {
            clean = strip_toml_comment(line)
            assignment_pos = 1
            i = 1
            while (i <= length(clean)) {
                while (substr(clean, i, 1) ~ /[[:space:]]/)
                    i++
                if (i > length(clean))
                    return 0
                if (assignment_pos) {
                    rest = substr(clean, i)
                    if (match(rest,
                              "^" key "[[:space:]]*=[[:space:]]*")) {
                        rest = substr(rest, RSTART + RLENGTH)
                        quote = substr(rest, 1, 1)
                        if (quote != "\"" && quote != single_quote)
                            return 0
                        escaped = 0
                        for (i = 2; i <= length(rest); i++) {
                            ch = substr(rest, i, 1)
                            if (escaped) {
                                escaped = 0
                            } else if (ch == "\\") {
                                escaped = 1
                            } else if (ch == quote) {
                                scan_value_result = substr(rest, 2, i - 2)
                                return 1
                            }
                        }
                        return 0
                    }
                }
                ch = substr(clean, i, 1)
                if (ch == "\"" || ch == single_quote) {
                    quote = ch
                    escaped = 0
                    i++
                    while (i <= length(clean)) {
                        ch = substr(clean, i, 1)
                        if (escaped) {
                            escaped = 0
                        } else if (ch == "\\") {
                            escaped = 1
                        } else if (ch == quote) {
                            i++
                            break
                        }
                        i++
                    }
                    assignment_pos = 0
                } else if (ch == "{" || ch == ",") {
                    assignment_pos = 1
                    i++
                } else {
                    assignment_pos = 0
                    i++
                }
            }
            return 0
        }

        function has_closing_brace(line,   i, ch, quote, escaped) {
            for (i = 1; i <= length(line); i++) {
                ch = substr(line, i, 1)
                if (quote != "") {
                    if (escaped) {
                        escaped = 0
                    } else if (ch == "\\") {
                        escaped = 1
                    } else if (ch == quote) {
                        quote = ""
                    }
                } else if (ch == "\"" || ch == single_quote) {
                    quote = ch
                } else if (ch == "#") {
                    return 0
                } else if (ch == "}") {
                    return 1
                }
            }
            return 0
        }

        function scan_inline(line) {
            if (scan_value(line, "path"))
                inline_path = 1
            if (scan_value(line, "version"))
                inline_version = scan_value_result
        }

        function finish_inline() {
            if (inline_path && inline_version != "")
                print inline_name "\t" inline_version
            reset_inline()
        }

        function reset_subtable() {
            subtable_name = ""
            subtable_path = 0
            subtable_version = ""
        }

        function scan_subtable(line) {
            if (scan_value(line, "path"))
                subtable_path = 1
            if (scan_value(line, "version"))
                subtable_version = scan_value_result
        }

        function finish_subtable() {
            if (subtable_path && subtable_version != "")
                print subtable_name "\t" subtable_version
            reset_subtable()
        }

        function normalize_subtable_name(name,   first, last) {
            first = substr(name, 1, 1)
            last = substr(name, length(name), 1)
            if ((first == "\"" && last == "\"") ||
                (first == single_quote && last == single_quote))
                return substr(name, 2, length(name) - 2)
            if (name ~ /^[[:alnum:]_-]+$/)
                return name
            return ""
        }

        BEGIN {
            mode = ""
            fatal = 0
            single_quote = sprintf("%c", 39)
            reset_inline()
            reset_subtable()
        }

        /^[[:space:]]*\[[[:space:]]*workspace[[:space:]]*\.[[:space:]]*dependencies[[:space:]]*\][[:space:]]*(#.*)?$/ {
            if (mode == "inline")
                fail_unclosed_inline()
            if (mode == "subtable")
                finish_subtable()
            mode = "workspace_dependencies"
            next
        }

        /^[[:space:]]*\[[[:space:]]*workspace[[:space:]]*\.[[:space:]]*dependencies[[:space:]]*\./ {
            if (mode == "inline")
                fail_unclosed_inline()
            if (mode == "subtable")
                finish_subtable()
            subtable_name = $0
            sub(/^[[:space:]]*\[[[:space:]]*workspace[[:space:]]*\.[[:space:]]*dependencies[[:space:]]*\.[[:space:]]*/, "", subtable_name)
            sub(/[[:space:]]*\][[:space:]]*(#.*)?$/, "", subtable_name)
            subtable_name = normalize_subtable_name(subtable_name)
            if (!subtable_name) {
                mode = ""
                next
            }
            mode = "subtable"
            next
        }

        /^[[:space:]]*\[/ {
            if (mode == "inline")
                fail_unclosed_inline()
            if (mode == "subtable")
                finish_subtable()
            mode = ""
            next
        }

        mode == "inline" {
            scan_inline($0)
            if (has_closing_brace($0)) {
                finish_inline()
                mode = "workspace_dependencies"
            }
            next
        }

        mode == "workspace_dependencies" {
            if (!inline_name &&
                $0 ~ /^[[:space:]]*[[:alnum:]_-]+[[:space:]]*=[[:space:]]*\{/) {
                inline_name = $0
                sub(/^[[:space:]]*/, "", inline_name)
                sub(/[[:space:]]*=.*/, "", inline_name)
                mode = "inline"
                scan_inline($0)
                if (has_closing_brace($0)) {
                    finish_inline()
                    mode = "workspace_dependencies"
                }
                next
            }
            next
        }

        mode == "subtable" {
            scan_subtable($0)
            next
        }

        END {
            if (!fatal && mode == "inline")
                fail_unclosed_inline()
            if (!fatal && mode == "subtable")
                finish_subtable()
        }
    ' "$CARGO_TOML"
)

# Empty is an ERROR, not a pass: this guard exists because the internal
# cerulion_* version entries MUST be present (publishing breaks without
# them). If none parse, either the manifest format changed or the
# entries were deleted — both need a loud failure, and without this
# check the here-string below would feed one blank line into the loop
# and report a misleading '"" != version' mismatch instead.
if [[ -z "$dep_versions" ]]; then
    echo "error: no internal path dependency version entries found under [workspace.dependencies] in $CARGO_TOML" >&2
    echo "       (manifest format changed, or the internal dep entries were removed — both break publishing)" >&2
    exit 1
fi

mismatches=0
while IFS=$'\t' read -r dep_name dep_ver; do
    # Compare against a copy with any leading '=' exact-match prefix stripped
    # (e.g. "=0.0.1-alpha") so it matches [workspace.package].version, which
    # carries no operator. Keep the raw entry for the error message so the fix
    # hint doesn't silently drop the '=' pin.
    dep_ver_raw="$dep_ver"
    dep_ver="${dep_ver#=}"
    if [[ "$dep_ver" != "$pkg_version" ]]; then
        echo "error: [workspace.dependencies] entry $dep_name has version \"$dep_ver_raw\" but [workspace.package].version is \"$pkg_version\"" >&2
        mismatches=$((mismatches + 1))
    fi
done <<< "$dep_versions"

if [[ $mismatches -gt 0 ]]; then
    echo "Fix: update all version = \"...\" entries under [workspace.dependencies] to match \"$pkg_version\", keeping the leading \"=\" exact-pin (e.g. \"=$pkg_version\")." >&2
    exit 1
fi

echo "ok: all internal [workspace.dependencies] versions match [workspace.package].version ($pkg_version)"
