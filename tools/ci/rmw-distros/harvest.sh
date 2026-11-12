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
# per line, sorted and unique (a doc test's name carries spaces and is taken whole). For each target, the names come from the FINAL `failures:` list (the
# one followed by that target's `test result:` line), qualified by the target binary read off the
# preceding `Running ... (path)` line (the lib's unit tests are the crate name; doc tests are
# "doctests"). This never depends on a `test <name> ... FAILED` line surviving the test's own stdout.
qualified_failures() {
    awk '
        /^ *Running / { if (match($0, /\([^)]*\)/)) { p = substr($0, RSTART + 1, RLENGTH - 2); sub(/.*\//, "", p); sub(/-[0-9a-f]+$/, "", p); bin = p } }
        /^ *Doc-tests / { bin = "doctests" }
        /^failures:$/ { collecting = 1; n = 0; next }
        collecting && /^    [^ ]/ { names[++n] = substr($0, 5); next }
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
# Only cargo's own "could not compile" line marks a compile failure: a failing doc test prints the
# compiler's "error[E...]" lines inside its stdout dump, and that is a counted failure, not a crash.
crashed() {
    grep -qE "process didn't exit successfully|\(signal: |^error: could not compile" "$1"
}

# The positive control every rmw library defines: without it an empty, truncated or unreadable
# symbol table would satisfy a "none of these are present" check for free.
NM_CONTROL_SYMBOL="rmw_init"

# defined_symbols <nm output>: the defined symbol NAMES, one per line, sorted and unique.
# `nm -D --defined-only` prints "<address> <type> <name>" and has already dropped the undefined
# entries, so the name is the last field; a "@VERSION" suffix is stripped so a versioned library
# reads the same as an unversioned one.
defined_symbols() {
    awk '{ print $NF }' "$1" | sed -E 's/@.*$//; /^$/d' | sort -u
}

# symbol_audit <nm output> <absent symbols, space separated>: exit 0 when the library defines the
# control symbol and NONE of the named ones. An rmw whose headers do not declare a symbol must not
# export it: rcl resolves by name, so a defined symbol is a claim the distro cannot back. The
# control is what makes a zero finding mean something.
symbol_audit() {
    local nm_out="$1" absent_list="$2"
    local defined bad=0 s
    defined="$(defined_symbols "$nm_out")"
    if ! printf '%s\n' "$defined" | grep -qx -- "$NM_CONTROL_SYMBOL"; then
        echo "SYMBOL AUDIT FAIL: the control symbol $NM_CONTROL_SYMBOL is NOT defined, so an empty or unreadable symbol table cannot pass by default"
        return 1
    fi
    # shellcheck disable=SC2086  # the list is deliberately word-split into symbol names
    for s in $absent_list; do
        if printf '%s\n' "$defined" | grep -qx -- "$s"; then
            echo "SYMBOL AUDIT FAIL: $s is defined, but this distro's headers do not declare it"
            bad=1
        fi
    done
    return "$bad"
}

# refuse_row_pinned <errors> <marker>: exit 0 only when a `refuse` row pins BOTH inputs for real.
# Both of gate.sh's inert defaults pass vacuously otherwise: `grep -F ""` matches every line, so an
# empty marker accepts any failure at all, and an absent "due to N previous errors" line reads back
# as 0, so an errors=0 row accepts a build that printed no count. A row that stops pinning its
# refusal must fail the gate, not sail through it.
refuse_row_pinned() {
    local errors="$1" marker="$2"
    case "$errors" in
        '' | *[!0-9]*)
            echo "GATE FAIL: a refuse row must pin a decimal error count, got '$errors'"
            return 1
            ;;
        0)
            echo "GATE FAIL: a refuse row must pin a NONZERO error count; 0 is the inert default and a build printing no error count at all reads back as 0"
            return 1
            ;;
    esac
    if [ -z "$marker" ]; then
        echo "GATE FAIL: a refuse row must pin a NON-EMPTY marker; grep -F '' matches every line, so an empty marker accepts any failure"
        return 1
    fi
    return 0
}

# --- rclpy cross-process exchange predicates -------------------------------------
# Sourced by gate.sh's `build` case and PROVEN by gate_selftest.sh on crafted
# inputs, so the gate's rclpy red-path reasoning has teeth on the self-test step
# (which runs before any build, with no real exchange). The gate runs the real
# python3 / cargo / timeout(1); these judge the already-collected result. Each
# prints its NAMED reason on the failing branch.

# staged_so_present <so path> <distro>: exit 0 when the librmw_cerulion.so at
# <so path> exists (the caller passes the freshly built build output, which is
# then staged and digest-checked into the ament prefix the exchange loads from);
# else print the named reason and return 1. The
# exchange must run against a real, freshly built library; an absent .so is
# nothing to run against.
staged_so_present() {
    local so="$1" distro="$2"
    if [ ! -f "$so" ]; then
        echo "GATE FAIL: $distro rclpy exchange - required librmw_cerulion.so is missing: $so (nothing to run the exchange against)"
        return 1
    fi
    return 0
}

# rclpy_probe_ok <probe output> <distro>: exit 0 only when the python3 import
# probe printed exactly "ok". A missing python3/rclpy/std_msgs/geometry_msgs is a
# NAMED failure, never a silent skip; the caller runs the probe under the sourced
# ROS env and passes its captured output here.
rclpy_probe_ok() {
    local probe="$1" distro="$2"
    if [ "$probe" != "ok" ]; then
        echo "GATE FAIL: $distro rclpy exchange cannot run - python3/rclpy/std_msgs/geometry_msgs unavailable in ros:$distro-ros-base: $probe"
        return 1
    fi
    return 0
}

# rclpy_timed_out <rc> <distro> <what> <timeout secs> [suffix]: return 0 (TRUE)
# and print the NAMED reason when rc is a timeout(1) code - 124 (TERM at the
# deadline) or 137 (escalated to KILL); return 1 silently otherwise. Same
# polarity as `crashed`. The caller `&& exit 1`s on a true return. `what` names
# the leg (e.g. "rclpy exchange 'direction_a...'"); optional `suffix` appends a note.
rclpy_timed_out() {
    local rc="$1" distro="$2" what="$3" secs="$4" suffix="${5:-}"
    if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
        echo "GATE FAIL: $distro $what timed out after ${secs}s${suffix:+ $suffix}"
        return 0
    fi
    return 1
}

# serial_suite_runs [arch]: exit 0 (TRUE, run the in-process serial suite) unless the caller asked
# to skip it (RMW_GATE_SKIP_SERIAL_SUITE=1) AND the lane is not x86_64. The x86_64 lanes always run
# it BY CONSTRUCTION, so a skip variable leaked into a shared env can never empty their floors. The
# optional arch arg (default `uname -m`) lets gate_selftest drive both lanes without a real machine.
serial_suite_runs() {
    local arch="${1:-$(uname -m)}"
    [ "${RMW_GATE_SKIP_SERIAL_SUITE:-0}" != "1" ] || [ "$arch" = "x86_64" ]
}

# --- rclcpp talker/listener pairs ------------------------------------------------
# The rows that run the pairs: the distros whose C++ introspection gives a `bool` sequence no
# accessor at all, so the bridge reaches it through its compiled shim. Every rclcpp node publishes
# rcl_interfaces/ParameterEvent at startup, which carries ParameterValue.bool_array_value, so a
# node there starts at all only through that path. Pinned here, beside its predicate, so
# gate_selftest.sh proves the list still names both.
RCLCPP_PAIR_DISTROS="foxy galactic"

# rclcpp_pairs_run <distro>: exit 0 when <distro> is one of RCLCPP_PAIR_DISTROS.
rclcpp_pairs_run() {
    case " $RCLCPP_PAIR_DISTROS " in
        *" $1 "*) return 0 ;;
    esac
    return 1
}

# mono_ms: milliseconds on the monotonic boot clock (/proc/uptime, 10 ms resolution). The wall
# clock can step backwards under a VM's time sync, which would make a node that ran to its deadline
# read as having ended early.
mono_ms() {
    local up
    read -r up _ < /proc/uptime
    echo $((${up%.*} * 1000 + 10#${up#*.} * 10))
}

# node_status <rc> <ms ran> <deadline secs>: one word for how a node under
# `timeout --preserve-status -s INT <deadline>` ended, its run time read with mono_ms. "ok" when it
# ran to its deadline and then exited 0 (a clean shutdown on the SIGINT); "early" when it ended
# before the deadline, whatever its status; "rc=<n>" when it reached the deadline and exited <n> (a
# crash on the way down, or the KILL after a hang). The deadline counts as reached 50 ms early, for
# the clock's resolution.
node_status() {
    local rc="$1" ms="$2" deadline="$3"
    if [ $((ms + 50)) -lt $((deadline * 1000)) ]; then
        echo early
    elif [ "$rc" -eq 0 ]; then
        echo ok
    else
        echo "rc=$rc"
    fi
}

# rclcpp_pair_judge <talker log> <listener log> <talker status> <listener status> <min heard>
# <label>: exit 0 when a demo_nodes talker and listener pair ran over rmw_cerulion and the listener
# heard what the talker said; else print the NAMED reason and return 1. Each status is node_status's
# word (an rclcpp node that cannot publish its startup ParameterEvent aborts with "terminate called"
# long before its deadline). The talker's lines read `Publishing: 'Hello World: N'` (rclcpp) or
# `Publishing: "Hello World: N"` (rclpy) and the listener's `I heard: [Hello World: N]` (both).
# Passing needs: a positive decimal <min heard>; both statuses "ok"; no abort, Python traceback, ERROR
# line from rmw_cerulion or cerulion_core (the targets the rmw logs under) or dropped malformed
# frame in either log; no number heard twice; at least <min heard> numbers heard; no more than two lost of those
# the talker published from the first one heard on (the listener may join late, and the last one or
# two can still be in flight at the deadline); and every number heard one the talker published, so
# a wrong or stale payload reds.
RCLCPP_PAIR_RED='terminate called|Traceback \(most recent call last\)|ERROR (rmw_cerulion|cerulion_core)|malformed variable entry'
rclcpp_pair_judge() {
    local tlog="$1" llog="$2" tstatus="$3" lstatus="$4" min="$5" label="$6"
    local f published heard n np first stray raw
    case "$min" in
        '' | *[!0-9]*)
            echo "GATE FAIL: $label - the heard floor '$min' is not a positive decimal, so any run would pass it"
            return 1
            ;;
    esac
    if [ "$((10#$min))" -eq 0 ]; then
        echo "GATE FAIL: $label - the heard floor '$min' is not a positive decimal, so any run would pass it"
        return 1
    fi
    for f in "$tlog" "$llog"; do
        if grep -qE "$RCLCPP_PAIR_RED" "$f"; then
            echo "GATE FAIL: $label - $(basename "$f") shows a crash or an rmw error: $(grep -m1 -E "$RCLCPP_PAIR_RED" "$f")"
            return 1
        fi
    done
    if [ "$tstatus" != ok ] || [ "$lstatus" != ok ]; then
        echo "GATE FAIL: $label - a node did not run to its deadline and stop cleanly (talker $tstatus, listener $lstatus)"
        return 1
    fi
    published="$(grep -oE "Publishing: ['\"]Hello World: [0-9]+['\"]" "$tlog" | grep -oE '[0-9]+' | sort -un)"
    heard="$(grep -oE 'I heard: \[Hello World: [0-9]+\]' "$llog" | grep -oE '[0-9]+' | sort -un)"
    n=$(printf '%s\n' "$heard" | grep -c .)
    raw=$(grep -oE 'I heard: \[Hello World: [0-9]+\]' "$llog" | grep -c .)
    if [ "$raw" -ne "$n" ]; then
        echo "GATE FAIL: $label - the listener heard $raw messages but only $n distinct ones, so a message was delivered twice"
        return 1
    fi
    if [ "$n" -lt "$min" ]; then
        echo "GATE FAIL: $label - the listener heard $n distinct messages, floor $min (the talker published $(printf '%s\n' "$published" | grep -c .))"
        return 1
    fi
    first=$(printf '%s\n' "$heard" | head -n1)
    np=$(printf '%s\n' "$published" | awk -v f="$first" '$1 != "" && $1 >= f' | grep -c .)
    if [ "$n" -lt $((np - 2)) ]; then
        echo "GATE FAIL: $label - the listener heard $n of the $np messages published from $first on, more than two lost"
        return 1
    fi
    # The published set goes in through -v (space separated: not every awk takes a newline in a -v
    # value), not as a first file: an EMPTY first file would make awk read the heard numbers as the
    # published ones and pass every stray number.
    stray="$(printf '%s\n' "$heard" | awk -v pub="$(printf '%s\n' "$published" | tr '\n' ' ')" 'BEGIN { n = split(pub, a, " "); for (i = 1; i <= n; i++) p[a[i]] = 1 } $1 != "" && !($1 in p)')"
    if [ -n "$stray" ]; then
        echo "GATE FAIL: $label - the listener heard numbers the talker never published: $(printf '%s' "$stray" | tr '\n' ' ')"
        return 1
    fi
    echo "PAIR OK: $label - heard $n distinct messages, every one published ($(printf '%s\n' "$heard" | head -n1) to $(printf '%s\n' "$heard" | tail -n1))"
    return 0
}
