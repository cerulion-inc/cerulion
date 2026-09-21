#!/bin/sh

set -eu

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-version-test.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

run_case() {
    name=$1
    expected=$2
    manifest=$3
    case_dir="$workdir/$name"
    mkdir -p "$case_dir"
    printf '%s\n' "$manifest" > "$case_dir/Cargo.toml"
    actual=$(CDPATH='' cd "$case_dir" && "$script_dir/workspace_version.sh")
    [ "$actual" = "$expected" ] || {
        printf 'error: %s expected %s, got %s\n' "$name" "$expected" "$actual" >&2
        exit 1
    }
    printf '%s: %s\n' "$name" "$actual"
}

run_rejected_case() {
    name=$1
    manifest=$2
    case_dir="$workdir/$name"
    mkdir -p "$case_dir"
    printf '%s\n' "$manifest" > "$case_dir/Cargo.toml"
    if (CDPATH='' cd "$case_dir" && "$script_dir/workspace_version.sh") >/dev/null 2>&1; then
        printf 'error: %s incorrectly accepted\n' "$name" >&2
        exit 1
    fi
    printf '%s: rejected\n' "$name"
}

run_case quoted 0.1.0 '[workspace.package]
version = "0.1.0"'
run_case single_quoted 0.2.0 "[workspace.package]
version = '0.2.0'"
run_case single_quoted_backslash '0.2.1\tn' "[workspace.package]
version = '0.2.1\\tn'"
run_case no_space_equals 0.3.0 '[workspace.package]
version="0.3.0"'
run_case header_comment 0.4.0 '  [workspace.package]  # workspace metadata
version = "0.4.0"'
run_case header_leading_whitespace 0.5.0 '    [workspace.package]
version = "0.5.0"'
run_case indented_version 0.5.9 '[workspace.package]
    version = "0.5.9"'
run_case spaced_header 0.5.10 '[ workspace.package ]
version = "0.5.10"'
run_case tabbed_header 0.5.11 '	[	workspace.package	]
version = "0.5.11"'
run_case double_quoted_header 0.5.14 '["workspace"."package"]
version = "0.5.14"'
run_case single_quoted_header 0.5.15 "['workspace'.package]
version = '0.5.15'"
run_case mixed_quoted_header 0.5.16 '[ "workspace" . '\''package'\'' ]
version = "0.5.16"'
run_case spaced_quoted_header 0.5.17 "[ 'workspace' . \"package\" ]
version = '0.5.17'"
run_case quoted_key_with_outer_spaces 0.5.18 '[ "workspace" . "package" ]
version = "0.5.18"'
run_case basic_escapes '0.5.12	rc1' '[workspace.package]
version = "0.5.12\trc1"'
run_case basic_quote_backslash '0.5.13"rc\end' '[workspace.package]
version = "0.5.13\"rc\\end"'
run_case unicode_escape 0.5.19 '[workspace.package]
version = "0.5.\u0031\u0039"'
run_case unicode_multibyte "$(printf '\316\273')" '[workspace.package]
version = "\u03bb"'
run_case unicode_supplementary "$(printf '\360\237\230\200')" '[workspace.package]
version = "\U0001F600"'
unicode_bytes_case="$workdir/unicode-bytes"
mkdir -p "$unicode_bytes_case"
printf '%s\n' '[workspace.package]' 'version = "\u03bb"' \
    > "$unicode_bytes_case/Cargo.toml"
unicode_actual=$("$script_dir/workspace_version.sh" "$unicode_bytes_case/Cargo.toml")
unicode_bytes=$(printf '%s' "$unicode_actual" | od -An -tx1 | tr -d '[:space:]')
[ "$unicode_bytes" = "cebb" ] || {
    printf 'error: unicode_multibyte expected bytes cebb, got %s\n' "$unicode_bytes" >&2
    exit 1
}
printf '%s: exact bytes %s\n' 'unicode_multibyte' "$unicode_bytes"
if command -v gawk >/dev/null 2>&1; then
    unicode_gawk_actual=$(
        LC_ALL=C.UTF-8 CERULION_WORKSPACE_VERSION_AWK=gawk \
            "$script_dir/workspace_version.sh" "$unicode_bytes_case/Cargo.toml"
    )
    unicode_gawk_bytes=$(printf '%s' "$unicode_gawk_actual" |
        od -An -tx1 | tr -d '[:space:]')
    [ "$unicode_gawk_bytes" = "cebb" ] || {
        printf 'error: gawk unicode_multibyte expected bytes cebb, got %s\n' \
            "$unicode_gawk_bytes" >&2
        exit 1
    }
    printf '%s: exact bytes %s\n' 'unicode_multibyte_gawk' "$unicode_gawk_bytes"
else
    printf '%s\n' 'unicode_multibyte_gawk: unavailable'
fi
run_case multiline_description 0.5.1 '[workspace.package]
description = """
version = "9.9.9"
"""
version = "0.5.1"'
run_case comment_triple_quotes 0.5.2 '[workspace.package]
# This comment contains """ and must not open a string.
version = "0.5.2"'
run_case single_line_triple_quotes 0.5.3 '[workspace.package]
description = '\''contains """ inside one line'\''
version = "0.5.3"'
run_case single_quote_multiline_description 0.5.4 "[workspace.package]
description = '''
version = '9.9.9'
'''
version = '0.5.4'"
run_case escaped_multiline_quote 0.5.5 '[workspace.package]
description = """
escaped \""" quote
version = "9.9.9"
"""
version = "0.5.5"'
run_case literal_multiline_backslash 0.5.6 "[workspace.package]
description = '''
version = '9.9.9'
literal path uses a backslash: C:\\temp\\value
'''
version = '0.5.6'"
run_case preceding_basic_decoy 0.5.7 '[package]
description = """
[workspace.package]
version = "9.9.9"
"""
[workspace.package]
version = "0.5.7"'
run_case preceding_literal_decoy 0.5.8 "[package]
description = '''
[workspace.package]
version = '9.9.9'
'''
[workspace.package]
version = '0.5.8'"

sync_case="$workdir/sync"
mkdir -p "$sync_case"
cat > "$sync_case/Cargo.toml" <<'EOF'
[ workspace.package ] # package metadata
version = "0.5.0"

[ workspace.dependencies ] # internal pins
cerulion_core = { path = "cerulion_core", version = "=0.5.0" }

    [patch.crates-io]
unrelated = "0.0.1"
EOF
bash "$script_dir/check_version_sync.sh" "$sync_case/Cargo.toml" >/dev/null ||
    {
        printf '%s\n' 'error: spaced workspace tables failed version sync' >&2
        exit 1
    }
printf '%s\n' 'spaced_sync_tables: accepted'

indented_sync_case="$workdir/indented-sync"
mkdir -p "$indented_sync_case"
cat > "$indented_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = { path = "cerulion_core", version = "=0.5.0" }
    cerulion_macros = { path = "cerulion_macros", version = "=0.4.9" }
EOF
if bash "$script_dir/check_version_sync.sh" "$indented_sync_case/Cargo.toml" \
    >"$indented_sync_case/output" 2>&1; then
    printf '%s\n' 'error: indented mismatched dependency was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_macros' "$indented_sync_case/output" || {
    printf '%s\n' 'error: indented dependency diagnostic omitted its name' >&2
    cat "$indented_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'indented_sync_mismatch: rejected'

multiline_inline_sync_case="$workdir/multiline-inline-sync"
mkdir -p "$multiline_inline_sync_case"
cat > "$multiline_inline_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = {
    path = "cerulion_core",
    version = "=0.4.9",
}
EOF
if bash "$script_dir/check_version_sync.sh" "$multiline_inline_sync_case/Cargo.toml" \
    >"$multiline_inline_sync_case/output" 2>&1; then
    printf '%s\n' 'error: multiline inline dependency mismatch was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$multiline_inline_sync_case/output" || {
    printf '%s\n' 'error: multiline inline diagnostic omitted its name' >&2
    cat "$multiline_inline_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'multiline_inline_sync_mismatch: rejected'

single_quoted_sync_case="$workdir/single-quoted-sync"
mkdir -p "$single_quoted_sync_case"
cat > "$single_quoted_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = { path = "cerulion_core", version = '0.4.9' }
cerulion_macros = { path = "cerulion_macros", version = "=0.5.0" }
EOF
if bash "$script_dir/check_version_sync.sh" "$single_quoted_sync_case/Cargo.toml" \
    >"$single_quoted_sync_case/output" 2>&1; then
    printf '%s\n' 'error: single-quoted dependency mismatch was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$single_quoted_sync_case/output" || {
    printf '%s\n' 'error: single-quoted dependency diagnostic omitted its name' >&2
    cat "$single_quoted_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'single_quoted_sync_mismatch: rejected'

quoted_assignment_in_path_sync_case="$workdir/quoted-assignment-in-path-sync"
mkdir -p "$quoted_assignment_in_path_sync_case"
cat > "$quoted_assignment_in_path_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = { path = 'version = "0.0.0"', version = "=0.5.0" }
EOF
bash "$script_dir/check_version_sync.sh" \
    "$quoted_assignment_in_path_sync_case/Cargo.toml" >/dev/null || {
    printf '%s\n' 'error: assignment-looking path value caused a false mismatch' >&2
    exit 1
}
printf '%s\n' 'quoted_assignment_in_path_sync: accepted'

quoted_assignment_in_path_mismatch_case="$workdir/quoted-assignment-in-path-mismatch"
mkdir -p "$quoted_assignment_in_path_mismatch_case"
cat > "$quoted_assignment_in_path_mismatch_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = { path = 'version = "0.0.0"', version = "=0.4.9" }
EOF
if bash "$script_dir/check_version_sync.sh" \
    "$quoted_assignment_in_path_mismatch_case/Cargo.toml" \
    >"$quoted_assignment_in_path_mismatch_case/output" 2>&1; then
    printf '%s\n' 'error: stale version after assignment-looking path was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$quoted_assignment_in_path_mismatch_case/output" || {
    printf '%s\n' 'error: stale assignment-looking path diagnostic omitted its name' >&2
    cat "$quoted_assignment_in_path_mismatch_case/output" >&2
    exit 1
}
printf '%s\n' 'quoted_assignment_in_path_sync_mismatch: rejected'

comment_brace_sync_case="$workdir/comment-brace-sync"
mkdir -p "$comment_brace_sync_case"
cat > "$comment_brace_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = {
    path = "cerulion_core",
    # } is part of this comment, not the table terminator
    version = "=0.4.9",
}
cerulion_macros = { path = "cerulion_macros", version = "=0.5.0" }
EOF
if bash "$script_dir/check_version_sync.sh" "$comment_brace_sync_case/Cargo.toml" \
    >"$comment_brace_sync_case/output" 2>&1; then
    printf '%s\n' 'error: brace in comment caused stale dependency to be accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$comment_brace_sync_case/output" || {
    printf '%s\n' 'error: comment-brace diagnostic omitted its name' >&2
    cat "$comment_brace_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'comment_brace_sync_mismatch: rejected'

unterminated_header_case="$workdir/unterminated-header-sync"
mkdir -p "$unterminated_header_case"
cat > "$unterminated_header_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = {
    path = "cerulion_core",
    version = "=0.5.0",

[patch.crates-io]
unrelated = "0.0.1"
EOF
if bash "$script_dir/check_version_sync.sh" "$unterminated_header_case/Cargo.toml" \
    >"$unterminated_header_case/output" 2>&1; then
    printf '%s\n' 'error: unterminated inline table before header was accepted' >&2
    exit 1
fi
grep -Fq 'inline dependency table for cerulion_core never closed' \
    "$unterminated_header_case/output" || {
    printf '%s\n' 'error: unterminated header diagnostic was missing' >&2
    cat "$unterminated_header_case/output" >&2
    exit 1
}
printf '%s\n' 'unterminated_inline_before_header: rejected'

unterminated_eof_case="$workdir/unterminated-eof-sync"
mkdir -p "$unterminated_eof_case"
cat > "$unterminated_eof_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies]
cerulion_core = {
    path = "cerulion_core",
    version = "=0.5.0",
EOF
if bash "$script_dir/check_version_sync.sh" "$unterminated_eof_case/Cargo.toml" \
    >"$unterminated_eof_case/output" 2>&1; then
    printf '%s\n' 'error: unterminated inline table at EOF was accepted' >&2
    exit 1
fi
grep -Fq 'inline dependency table for cerulion_core never closed' \
    "$unterminated_eof_case/output" || {
    printf '%s\n' 'error: unterminated EOF diagnostic was missing' >&2
    cat "$unterminated_eof_case/output" >&2
    exit 1
}
printf '%s\n' 'unterminated_inline_at_eof: rejected'

subtable_sync_case="$workdir/subtable-sync"
mkdir -p "$subtable_sync_case"
cat > "$subtable_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies.cerulion_core]
path = "cerulion_core"
version = "=0.4.9"
EOF
if bash "$script_dir/check_version_sync.sh" "$subtable_sync_case/Cargo.toml" \
    >"$subtable_sync_case/output" 2>&1; then
    printf '%s\n' 'error: dependency subtable mismatch was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$subtable_sync_case/output" || {
    printf '%s\n' 'error: dependency subtable diagnostic omitted its name' >&2
    cat "$subtable_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'dependency_subtable_sync_mismatch: rejected'

quoted_subtable_sync_case="$workdir/quoted-subtable-sync"
mkdir -p "$quoted_subtable_sync_case"
cat > "$quoted_subtable_sync_case/Cargo.toml" <<'EOF'
[workspace.package]
version = "0.5.0"

[workspace.dependencies."cerulion_core"]
path = "cerulion_core"
version = "=0.4.9"

[workspace.dependencies.'cerulion_macros']
path = "cerulion_macros"
version = "=0.4.9"
EOF
if bash "$script_dir/check_version_sync.sh" "$quoted_subtable_sync_case/Cargo.toml" \
    >"$quoted_subtable_sync_case/output" 2>&1; then
    printf '%s\n' 'error: quoted dependency subtable mismatch was accepted' >&2
    exit 1
fi
grep -Fq 'cerulion_core' "$quoted_subtable_sync_case/output" || {
    printf '%s\n' 'error: quoted dependency subtable diagnostic omitted its name' >&2
    cat "$quoted_subtable_sync_case/output" >&2
    exit 1
}
grep -Fq 'cerulion_macros' "$quoted_subtable_sync_case/output" || {
    printf '%s\n' 'error: single-quoted dependency subtable diagnostic omitted its name' >&2
    cat "$quoted_subtable_sync_case/output" >&2
    exit 1
}
printf '%s\n' 'quoted_dependency_subtable_sync_mismatch: rejected'

missing_sync_case="$workdir/missing-sync"
mkdir -p "$missing_sync_case"
printf '%s\n' '[ workspace.package ]' 'version = "0.5.0"' \
    > "$missing_sync_case/Cargo.toml"
if bash "$script_dir/check_version_sync.sh" "$missing_sync_case/Cargo.toml" \
    >/dev/null 2>&1; then
    printf '%s\n' 'error: missing workspace dependencies table was accepted' >&2
    exit 1
fi
printf '%s\n' 'missing_sync_table: rejected'

explicit_case="$workdir/explicit"
mkdir -p "$explicit_case"
printf '%s\n' '[workspace.package]
version = "9.9.9"' > "$explicit_case/Cargo.toml"
explicit_manifest="$explicit_case/workspace.manifest"
printf '%s\n' '[workspace.package]
version = "0.6.0"' > "$explicit_manifest"
actual=$("$script_dir/workspace_version.sh" "$explicit_manifest")
[ "$actual" = "0.6.0" ] || {
    printf 'error: explicit_path expected 0.6.0, got %s\n' "$actual" >&2
    exit 1
}
printf '%s: %s\n' 'explicit_path' "$actual"

equals_manifest="$explicit_case/weird=name.toml"
printf '%s\n' '[workspace.package]
version = "0.7.0"' > "$equals_manifest"
actual=$("$script_dir/workspace_version.sh" "$equals_manifest")
[ "$actual" = "0.7.0" ] || {
    printf 'error: explicit_equals_path expected 0.7.0, got %s\n' "$actual" >&2
    exit 1
}
printf '%s: %s\n' 'explicit_equals_path' "$actual"

run_rejected_case true '[workspace.package]
version = true'
run_rejected_case integer '[workspace.package]
version = 1'
run_rejected_case multiline_version '[workspace.package]
version = """
0.8.0
"""'
run_case escaped_quote_after_content '0.8.2"tail' '[workspace.package]
version = "0.8.2\"tail"'
run_rejected_case basic_trailing_garbage '[workspace.package]
version = "0.1.0" garbage "'
run_rejected_case literal_interior_quote "[workspace.package]
version = '0.1.0'garbage'"
run_rejected_case header_inside_multiline '[package]
description = """
[ workspace.package ]
version = "9.9.9"
"""
version = "0.8.1"'
run_rejected_case quoted_key_with_interior_spaces '[" workspace "."package"]
version = "0.8.3"'
run_rejected_case unicode_surrogate '[workspace.package]
version = "\uD800"'
run_rejected_case unicode_out_of_range '[workspace.package]
version = "\U00110000"'
run_rejected_case unicode_truncated '[workspace.package]
version = "\u12"'

package_case="$workdir/package"
mkdir -p "$package_case"
printf '%s\n' '[package]
version = "9.9.9"' > "$package_case/Cargo.toml"
if (CDPATH='' cd "$package_case" && "$script_dir/workspace_version.sh") >/dev/null 2>&1; then
    printf '%s\n' 'error: [package] table incorrectly matched' >&2
    exit 1
fi
printf '%s\n' 'package_table: rejected'

run_rejected_case quoted_package_table '["package"]
version = "9.9.9"'

if "$script_dir/workspace_version.sh" "$workdir/missing.manifest" >/dev/null 2>&1; then
    printf '%s\n' 'error: missing manifest incorrectly accepted' >&2
    exit 1
fi
printf '%s\n' 'missing_manifest: rejected'
