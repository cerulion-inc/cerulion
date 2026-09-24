#!/usr/bin/env bash
# Self-test of the lane harvester (harvest.sh) over a hand-built fixture log, so the harvester is
# proven independently of whether any lane row carries failures (a row with an empty pin never
# exercises it). The fixture carries every shape the real logs have shown: cargo status lines in
# terminal colour, the lib's unit tests, three test binaries, a doc-test target, a stdout dump with
# an indented line that looks like a name, a FAILED token on its own line after unterminated test
# stdout, and the final per-binary failures lists. Run: bash tools/ci/rmw-distros/gate_selftest.sh
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
[ "$raw_bins" -eq 0 ] && [ "$plain_bins" -eq 3 ] || { echo "SELFTEST FAIL: Running lines raw=$raw_bins plain=$plain_bins (want 0 and 3)"; fail=1; }
# 3. The harvested set is exactly the expected one, binary-qualified, the own-line FAILED included.
got="$(qualified_failures "$plain")"
if [ "$got" != "$(cat "$expected")" ]; then echo "SELFTEST FAIL: harvested set differs"; echo "-- expected:"; cat "$expected"; echo "-- got:"; echo "$got"; fail=1; fi
# 4. The counts come from the summaries: 4 targets, 10 tests run (6 + 2 + 3... as the summaries say), 4 failed.
read -r summaries ran failed_total <<< "$(suite_counts "$plain")"
[ "$summaries" -eq 4 ] && [ "$ran" -eq 10 ] && [ "$failed_total" -eq 4 ] || { echo "SELFTEST FAIL: counts summaries=$summaries ran=$ran failed=$failed_total (want 4 10 4)"; fail=1; }
# 5. The failures-list count agrees with the summaries' count (the cross-check the gate makes).
[ "$(printf '%s\n' "$got" | grep -c .)" -eq "$failed_total" ] || { echo "SELFTEST FAIL: harvested $(printf '%s\n' "$got" | grep -c .) names against $failed_total summary failures"; fail=1; }
# 6. A clean fixture is not a crash; a compile failure and a signal death are.
crashed "$plain" && { echo "SELFTEST FAIL: the clean fixture reads as crashed"; fail=1; }
# shellcheck disable=SC2016  # the backticks are cargo's literal message text, not a command
printf 'error: could not compile `rmw_cerulion` (test "x") due to 3 previous errors\n' > "$plain.c"; crashed "$plain.c" || { echo "SELFTEST FAIL: a compile failure is not detected"; fail=1; }
printf "  process didn't exit successfully: (signal: 11, SIGSEGV: invalid memory reference)\n" > "$plain.s"; crashed "$plain.s" || { echo "SELFTEST FAIL: a signal death is not detected"; fail=1; }
rm -f "$plain" "$plain.c" "$plain.s"
[ "$fail" -eq 0 ] && echo "SELFTEST PASS: harvester proven over the coloured fixture (3 binaries + doc tests, 4 qualified failures)"
exit "$fail"
