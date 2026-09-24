#!/usr/bin/env bash
# The rmw distro lane's log harvester, sourced by gate.sh and by gate_selftest.sh so the same code
# is proven over a fixture log independently of whether any lane row carries failures.
#
# Every function reads a cargo test log that may carry terminal colour (cargo's status lines begin
# with an escape sequence when CARGO_TERM_COLOR=always, as the toolchain action sets it), so the log
# is stripped of ANSI escapes first; the lane also runs with colour off, belt and braces.

# strip_ansi <log>: the log with every "ESC[...m" colour sequence removed, on stdout.
strip_ansi() {
    local esc
    esc=$(printf '\033')
    sed -E "s/${esc}\[[0-9;]*m//g" "$1"
}

# qualified_failures <plain log>: the failing set as cargo itself reports it, one "<binary>::<test>"
# per line, sorted and unique. For each target, the names come from the FINAL `failures:` list (the
# one followed by that target's `test result:` line), qualified by the target binary read off the
# preceding `Running ... (path)` line (the lib's unit tests are the crate name; doc tests are
# "doctests"). This never depends on a `test <name> ... FAILED` line surviving the test's own stdout.
qualified_failures() {
    awk '
        /^ *Running / { if (match($0, /\([^)]*\)/)) { p = substr($0, RSTART + 1, RLENGTH - 2); sub(/.*\//, "", p); sub(/-[0-9a-f]+$/, "", p); bin = p } }
        /^ *Doc-tests / { bin = "doctests" }
        /^failures:$/ { collecting = 1; n = 0; next }
        collecting && /^    [^ ]+$/ { names[++n] = $1; next }
        collecting && /^$/ { next }
        collecting && /^test result: / { for (k = 1; k <= n; k++) print bin "::" names[k]; collecting = 0; n = 0; next }
        collecting { n = 0 }
    ' "$1" | sort -u
}

# suite_counts <plain log>: "<summaries> <tests run> <failed>" from the `test result:` lines.
suite_counts() {
    grep -E '^test result: ' "$1" | awk '{ s++; p += $4; f += $6; i += $8 } END { print s + 0, p + f + i, f + 0 }'
}

# crashed <plain log>: exit 0 when the log carries a crash or a compile failure of a test target.
crashed() {
    grep -qE "process didn't exit successfully|\(signal: |^error: could not compile|^error\[E[0-9]+\]" "$1"
}
