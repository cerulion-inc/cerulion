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
rm -f "$plain" "$plain.c" "$plain.s" "$plain.n"
[ "$fail" -eq 0 ] && echo "SELFTEST PASS: harvester proven over the coloured fixture (3 binaries + doc tests, 5 qualified failures incl. one doc test), the symbol audit over three nm fixtures, the refuse-row pin guard, and the rclpy red paths (missing .so, absent import probe, timeout rc) each named, and the serial-suite skip decision pinned on both arches"
exit "$fail"
