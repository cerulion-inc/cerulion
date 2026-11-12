#!/usr/bin/env bash
# Self-test of the lane harvester (harvest.sh) over a hand-built fixture log, so the harvester is
# proven independently of whether any lane row carries failures (a row with an empty pin never
# exercises it). The fixture carries every shape the real logs have shown: cargo status lines in
# terminal colour, the lib's unit tests, three test binaries, a doc-test target, a stdout dump with
# an indented line that looks like a name, a FAILED token on its own line after unterminated test
# stdout, a failing doc test whose name carries spaces, and the final per-binary failures lists.
# The symbol audit is proven over three hand-built `nm -D --defined-only` fixtures, so the gate's
# export check is exercised without a built library.
# Run: bash tools/ci/rmw-distros/gate_selftest.sh
set -u
here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=tools/ci/rmw-distros/harvest.sh
source "$here/harvest.sh"
fixture="$here/fixtures/cargo-test-coloured.log"
expected="$here/fixtures/cargo-test-coloured.expected"
plain="$(mktemp)"
strip_ansi "$fixture" > "$plain"
fail=0
# 1. Colour is really present in the fixture and really gone after stripping.
grep -q "$(printf '\033')\[" "$fixture" || { echo "SELFTEST FAIL: the fixture carries no colour, the strip is unproven"; fail=1; }
grep -q "$(printf '\033')\[" "$plain" && { echo "SELFTEST FAIL: colour survived strip_ansi"; fail=1; }
# 2. The raw (coloured) log defeats the Running rule; the plain log does not.
raw_bins=$(grep -cE '^ *Running ' "$fixture"); plain_bins=$(grep -cE '^ *Running ' "$plain")
if [ "$raw_bins" -ne 0 ] || [ "$plain_bins" -ne 3 ]; then echo "SELFTEST FAIL: Running lines raw=$raw_bins plain=$plain_bins (want 0 and 3)"; fail=1; fi
# 3. The harvested set is exactly the expected one, binary-qualified, the own-line FAILED and the spaced doc-test name included.
got="$(qualified_failures "$plain")"
if [ "$got" != "$(cat "$expected")" ]; then echo "SELFTEST FAIL: harvested set differs"; echo "-- expected:"; cat "$expected"; echo "-- got:"; echo "$got"; fail=1; fi
# 4. The counts come from the summaries: 4 targets, 11 tests run, 5 failed (one of them a doc test).
read -r summaries ran failed_total <<< "$(suite_counts "$plain")"
if [ "$summaries" -ne 4 ] || [ "$ran" -ne 11 ] || [ "$failed_total" -ne 5 ]; then echo "SELFTEST FAIL: counts summaries=$summaries ran=$ran failed=$failed_total (want 4 11 5)"; fail=1; fi
# 5. The failures-list count agrees with the summaries' count (the cross-check the gate makes).
[ "$(printf '%s\n' "$got" | grep -c .)" -eq "$failed_total" ] || { echo "SELFTEST FAIL: harvested $(printf '%s\n' "$got" | grep -c .) names against $failed_total summary failures"; fail=1; }
# 6. A clean fixture is not a crash; a compile failure and a signal death are.
crashed "$plain" && { echo "SELFTEST FAIL: the clean fixture reads as crashed"; fail=1; }
# shellcheck disable=SC2016  # the backticks are cargo's literal message text, not a command
printf 'error: could not compile `rmw_cerulion` (test "x") due to 3 previous errors\n' > "$plain.c"; crashed "$plain.c" || { echo "SELFTEST FAIL: a compile failure is not detected"; fail=1; }
printf "  process didn't exit successfully: (signal: 11, SIGSEGV: invalid memory reference)\n" > "$plain.s"; crashed "$plain.s" || { echo "SELFTEST FAIL: a signal death is not detected"; fail=1; }
# 7. The symbol audit, over three hand-built nm fixtures. Foxy's list is the subject: the clean
#    table passes, one leaked entry point fails, and a table without the control symbol fails even
#    though it contains none of the named symbols either.
absent="rmw_feature_supported rmw_qos_profile_check_compatible rmw_subscription_set_content_filter"
symbol_audit "$here/fixtures/nm-defined-clean.txt" "$absent" > /dev/null || { echo "SELFTEST FAIL: the clean symbol table does not pass the audit"; fail=1; }
symbol_audit "$here/fixtures/nm-defined-leaked-symbol.txt" "$absent" > /dev/null && { echo "SELFTEST FAIL: a header-absent symbol passed the audit"; fail=1; }
symbol_audit "$here/fixtures/nm-defined-no-control.txt" "$absent" > /dev/null && { echo "SELFTEST FAIL: a table without $NM_CONTROL_SYMBOL passed the audit"; fail=1; }
# An EMPTY table is the same class and is exactly what a silently-failing `nm` leaves behind.
: > "$plain.n"; symbol_audit "$plain.n" "$absent" > /dev/null && { echo "SELFTEST FAIL: an empty symbol table passed the audit"; fail=1; }
# A row with an EMPTY absent list (jazzy, lyrical, humble) still checks the control, so the audit
# is never a no-op on those rows.
symbol_audit "$here/fixtures/nm-defined-clean.txt" "" > /dev/null || { echo "SELFTEST FAIL: an empty absent list must still pass on a table with the control"; fail=1; }
symbol_audit "$here/fixtures/nm-defined-no-control.txt" "" > /dev/null && { echo "SELFTEST FAIL: an empty absent list must still FAIL without the control"; fail=1; }
# 8. The name reader: a "@@VERSION" suffix is stripped, and the address and type columns are never
#    mistaken for names.
names="$(defined_symbols "$here/fixtures/nm-defined-clean.txt")"
printf '%s\n' "$names" | grep -qx rmw_get_serialization_format || { echo "SELFTEST FAIL: a versioned symbol name is not stripped to its bare name"; fail=1; }
printf '%s\n' "$names" | grep -qE '^[0-9a-f]{8}|^[TDB]$' && { echo "SELFTEST FAIL: an address or type column was read as a symbol name"; fail=1; }
# 9. A `refuse` row must pin its refusal: the two inert defaults gate.sh carries for the next
#    distro that starts there must be REFUSED, because both pass vacuously (grep -F '' matches
#    every line, and an absent error count reads back as 0).
refuse_row_pinned 21 "cannot find type" > /dev/null || { echo "SELFTEST FAIL: a properly pinned refuse row was rejected"; fail=1; }
refuse_row_pinned 0 "cannot find type" > /dev/null && { echo "SELFTEST FAIL: a refuse row with errors=0 passed"; fail=1; }
refuse_row_pinned 21 "" > /dev/null && { echo "SELFTEST FAIL: a refuse row with an empty marker passed"; fail=1; }
refuse_row_pinned "" "cannot find type" > /dev/null && { echo "SELFTEST FAIL: a refuse row with no error count passed"; fail=1; }
refuse_row_pinned "many" "cannot find type" > /dev/null && { echo "SELFTEST FAIL: a refuse row with a non-decimal error count passed"; fail=1; }
# 10. The rclpy exchange red paths (harvest.sh predicates), driven on crafted
#     inputs so the gate's reasoning is proven on the self-test step (which runs
#     before any build - no real .so, no python invocation, no cargo/timeout):
#     (a) a missing .so, (b) a non-"ok" python import probe, and (c) a timeout
#     return code each red with a NAMED reason; each clean input passes.
no_so="$(mktemp -u)"   # a name mktemp guarantees is unused (nothing is created)
# (a) missing .so
staged_so_present "$no_so" selftest > /dev/null && { echo "SELFTEST FAIL: a missing .so passed staged_so_present"; fail=1; }
staged_so_present "$here/harvest.sh" selftest > /dev/null || { echo "SELFTEST FAIL: an existing file failed staged_so_present"; fail=1; }
case "$(staged_so_present "$no_so" selftest 2>&1)" in *"GATE FAIL"*"is missing"*) : ;; *) echo "SELFTEST FAIL: staged_so_present did not name the missing-.so reason"; fail=1 ;; esac
# (b) absent python/rclpy import
rclpy_probe_ok ok selftest > /dev/null || { echo "SELFTEST FAIL: an 'ok' probe failed rclpy_probe_ok"; fail=1; }
rclpy_probe_ok "ModuleNotFoundError: No module named 'rclpy'" selftest > /dev/null && { echo "SELFTEST FAIL: a non-ok probe passed rclpy_probe_ok"; fail=1; }
case "$(rclpy_probe_ok "ModuleNotFoundError: No module named 'rclpy'" selftest 2>&1)" in *"GATE FAIL"*"unavailable"*) : ;; *) echo "SELFTEST FAIL: rclpy_probe_ok did not name the import-absent reason"; fail=1 ;; esac
# (c) timeout return codes (124 TERM-at-deadline, 137 escalated to KILL); a normal rc is NOT a timeout
rclpy_timed_out 124 selftest "rclpy exchange" 600 > /dev/null || { echo "SELFTEST FAIL: rc 124 not read as a timeout"; fail=1; }
rclpy_timed_out 137 selftest "rclpy exchange" 600 > /dev/null || { echo "SELFTEST FAIL: rc 137 not read as a timeout"; fail=1; }
rclpy_timed_out 0   selftest "rclpy exchange" 600 > /dev/null && { echo "SELFTEST FAIL: rc 0 read as a timeout"; fail=1; }
rclpy_timed_out 101 selftest "rclpy exchange" 600 > /dev/null && { echo "SELFTEST FAIL: a normal non-zero rc (101) read as a timeout"; fail=1; }
case "$(rclpy_timed_out 124 selftest 'rclpy exchange' 600 2>&1)" in *"GATE FAIL"*"timed out"*) : ;; *) echo "SELFTEST FAIL: rclpy_timed_out did not name the timeout reason"; fail=1 ;; esac
# 11. The serial-suite skip decision (serial_suite_runs, harvest.sh): a skip request is honoured
#     ONLY off x86_64, so the x86_64 lanes run the suite no matter the variable (by construction).
( RMW_GATE_SKIP_SERIAL_SUITE=1; serial_suite_runs aarch64 ) && { echo "SELFTEST FAIL: skip=1 on aarch64 still ran the suite"; fail=1; }
( RMW_GATE_SKIP_SERIAL_SUITE=1; serial_suite_runs x86_64 ) || { echo "SELFTEST FAIL: skip=1 on x86_64 did not run the suite (the by-construction guard failed)"; fail=1; }
( RMW_GATE_SKIP_SERIAL_SUITE=0; serial_suite_runs aarch64 ) || { echo "SELFTEST FAIL: skip=0 on aarch64 did not run the suite"; fail=1; }
( unset RMW_GATE_SKIP_SERIAL_SUITE; serial_suite_runs aarch64 ) || { echo "SELFTEST FAIL: unset skip on aarch64 did not run the suite"; fail=1; }
# mono_ms reads a millisecond count off the monotonic clock that never goes backwards. It reads
# Linux's /proc/uptime, as the lanes do; a host without one (macOS) skips this check.
if [ -r /proc/uptime ]; then
    m1="$(mono_ms)"; m2="$(mono_ms)"
    case "$m1" in '' | *[!0-9]*) echo "SELFTEST FAIL: mono_ms printed '$m1'"; fail=1 ;; esac
    [ "$m2" -ge "$m1" ] || { echo "SELFTEST FAIL: mono_ms went backwards ($m1 then $m2)"; fail=1; }
fi
# 12. The rclcpp talker/listener pair judge (rclcpp_pair_judge, harvest.sh), over crafted logs in
#     both quoting styles: a clean pair passes; a C++ abort, a Python traceback, an rmw_cerulion
#     ERROR line, a node that ended before its deadline, too few messages heard, and a number heard
#     that was never published (with and without any published at all) each red, by name.
t="$plain.t"; l="$plain.l"
printf "[INFO] [talker]: Publishing: 'Hello World: 1'\n[INFO] [talker]: Publishing: 'Hello World: 2'\n[INFO] [talker]: Publishing: 'Hello World: 3'\n[INFO] [talker]: Publishing: 'Hello World: 4'\n" > "$t"
printf ' WARN rmw_cerulion::api::pubsub: requested QoS depth differs\n[INFO] [listener]: I heard: [Hello World: 2]\n[INFO] [listener]: I heard: [Hello World: 3]\n[INFO] [listener]: I heard: [Hello World: 4]\n' > "$l"
rclcpp_pair_judge "$t" "$l" ok ok 3 selftest > /dev/null || { echo "SELFTEST FAIL: a clean rclcpp pair did not pass"; fail=1; }
case "$(rclcpp_pair_judge "$t" "$l" ok ok 4 selftest)" in *"GATE FAIL"*"floor 4"*) : ;; *) echo "SELFTEST FAIL: three heard passed a floor of four, or the floor was not named"; fail=1 ;; esac
for bad_min in 0 00 "" abc; do
    case "$(rclcpp_pair_judge "$t" "$l" ok ok "$bad_min" selftest)" in *"GATE FAIL"*"not a positive decimal"*) : ;; *) echo "SELFTEST FAIL: a heard floor of '$bad_min' was accepted"; fail=1 ;; esac
done
: > "$l.none"
case "$(rclcpp_pair_judge "$t" "$l.none" ok ok 3 selftest)" in *"GATE FAIL"*"heard 0 distinct"*) : ;; *) echo "SELFTEST FAIL: a listener that heard nothing passed"; fail=1 ;; esac
printf "Publishing: 'Hello World: %s'\\n" 1 2 3 4 5 6 7 8 > "$t.many"
case "$(rclcpp_pair_judge "$t.many" "$l" ok ok 3 selftest)" in *"GATE FAIL"*"more than two lost"*) : ;; *) echo "SELFTEST FAIL: three heard of the seven published from the first heard on passed"; fail=1 ;; esac
# A listener that joined late (heard 6 to 8 of 1 to 8) lost nothing it could have heard.
printf 'I heard: [Hello World: %s]\n' 6 7 8 > "$l.late"
rclcpp_pair_judge "$t.many" "$l.late" ok ok 3 selftest > /dev/null || { echo "SELFTEST FAIL: a late-joining listener that heard every later message was refused"; fail=1; }
case "$(rclcpp_pair_judge "$t" "$l" early ok 3 selftest)" in *"GATE FAIL"*"talker early"*) : ;; *) echo "SELFTEST FAIL: a talker that ended early was not named"; fail=1 ;; esac
case "$(rclcpp_pair_judge "$t" "$l" ok rc=139 3 selftest)" in *"GATE FAIL"*"listener rc=139"*) : ;; *) echo "SELFTEST FAIL: a listener that crashed at its deadline was not named"; fail=1 ;; esac
# node_status: a clean stop at the deadline, an early end (even a clean one), and a crash or KILL
# at the deadline, with the deadline boundary itself counted as reached.
[ "$(node_status 0 15000 15)" = ok ] || { echo "SELFTEST FAIL: a clean stop at the deadline is not ok"; fail=1; }
[ "$(node_status 0 14950 15)" = ok ] || { echo "SELFTEST FAIL: a stop within the clock's resolution of the deadline is not ok"; fail=1; }
[ "$(node_status 0 14949 15)" = early ] || { echo "SELFTEST FAIL: a clean exit before the deadline is not early"; fail=1; }
[ "$(node_status 134 2000 15)" = early ] || { echo "SELFTEST FAIL: an abort at startup is not early"; fail=1; }
[ "$(node_status 139 15200 15)" = rc=139 ] || { echo "SELFTEST FAIL: a crash on the way down is not named by its status"; fail=1; }
[ "$(node_status 137 25000 15)" = rc=137 ] || { echo "SELFTEST FAIL: a KILL after a hang is not named by its status"; fail=1; }
printf 'I heard: [Hello World: 2]\nI heard: [Hello World: 3]\nI heard: [Hello World: 3]\nI heard: [Hello World: 4]\n' > "$l.dup"
case "$(rclcpp_pair_judge "$t" "$l.dup" ok ok 3 selftest)" in *"GATE FAIL"*"delivered twice"*) : ;; *) echo "SELFTEST FAIL: a message heard twice was not named"; fail=1 ;; esac
printf 'Publishing: "Hello World: 0"\nPublishing: "Hello World: 1"\nPublishing: "Hello World: 2"\n' > "$t.py"
printf 'I heard: [Hello World: 0]\nI heard: [Hello World: 1]\nI heard: [Hello World: 2]\n' > "$l.py"
rclcpp_pair_judge "$t.py" "$l.py" ok ok 3 selftest > /dev/null || { echo "SELFTEST FAIL: an rclpy talker's double-quoted lines were not read"; fail=1; }
printf 'I heard: [Hello World: 2]\nI heard: [Hello World: 3]\nI heard: [Hello World: 99]\n' > "$l.stray"
case "$(rclcpp_pair_judge "$t" "$l.stray" ok ok 3 selftest)" in *"GATE FAIL"*"never published: 99"*) : ;; *) echo "SELFTEST FAIL: a number heard but never published was not named"; fail=1 ;; esac
: > "$t.none"
case "$(rclcpp_pair_judge "$t.none" "$l" ok ok 3 selftest)" in *"GATE FAIL"*"never published"*) : ;; *) echo "SELFTEST FAIL: messages heard from a talker that published none passed"; fail=1 ;; esac
for marker in "terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'" \
    "Traceback (most recent call last):" \
    "2026-11-11T19:19:32Z ERROR rmw_cerulion::api::pubsub: frame sizing failed topic=/parameter_events" \
    "2026-11-11T19:19:32Z ERROR cerulion_core::transport::publisher: publish failed" \
    "2026-11-11T19:19:32Z  WARN rmw_cerulion::type_bridge: malformed variable entry in wire frame, dropping message"; do
    { cat "$t"; printf '%s\n' "$marker"; } > "$t.bad"
    case "$(rclcpp_pair_judge "$t.bad" "$l" ok ok 3 selftest)" in *"GATE FAIL"*"crash or an rmw error"*) : ;; *) echo "SELFTEST FAIL: a talker log carrying '$marker' was not named"; fail=1 ;; esac
    { cat "$l"; printf '%s\n' "$marker"; } > "$l.bad"
    case "$(rclcpp_pair_judge "$t" "$l.bad" ok ok 3 selftest)" in *"GATE FAIL"*"crash or an rmw error"*) : ;; *) echo "SELFTEST FAIL: a listener log carrying '$marker' was not named"; fail=1 ;; esac
done
# The pair rows: both distros whose C++ introspection leaves a bool sequence without accessors are
# on the list, and a distro off it is not.
for d in foxy galactic; do rclcpp_pairs_run "$d" || { echo "SELFTEST FAIL: $d is not on RCLCPP_PAIR_DISTROS"; fail=1; }; done
rclcpp_pairs_run jazzy && { echo "SELFTEST FAIL: jazzy is on RCLCPP_PAIR_DISTROS"; fail=1; }
rclcpp_pairs_run fox && { echo "SELFTEST FAIL: a prefix of a listed distro matched"; fail=1; }
rm -f "$plain" "$plain.c" "$plain.s" "$plain.n" "$t" "$l" "$t.py" "$l.py" "$l.stray" "$t.none" "$t.bad" "$l.bad" "$l.none" "$t.many" "$l.late" "$l.dup"
[ "$fail" -eq 0 ] && echo "SELFTEST PASS: harvester proven over the coloured fixture (3 binaries + doc tests, 5 qualified failures incl. one doc test), the symbol audit over three nm fixtures, the refuse-row pin guard, and the rclpy red paths (missing .so, absent import probe, timeout rc) each named, the serial-suite skip decision pinned on both arches, and the rclcpp pair judge's pass and every red path (bad floor, abort, traceback, rmw or core error, malformed frame, early end, crash at the deadline, duplicate, floor, loss, stray number) named, node_status pinned, and the pair rows pinned"
exit "$fail"
