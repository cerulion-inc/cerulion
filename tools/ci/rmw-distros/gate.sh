#!/usr/bin/env bash
# rmw distro lane gate: PASS means the tree's KNOWN state for this distro holds.
#
# Each distro has an expected state in the table below. `build` means rmw_cerulion must compile
# against the distro's real headers (generated bindings, never the vendored fallback) and its
# serial test suite must pass. `refuse` means the build is expected to stop at a KNOWN place, and
# the gate requires that exact marker in the build log: any other failure is a lane failure, and a
# distro that silently starts building fails the lane too, so the table can never lag the truth.
# The table is flipped deliberately, one PR per distro, as support lands.
set -u
distro="${1:?distro}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/$distro/setup.bash"
set -u
[ -n "${AMENT_PREFIX_PATH:-}" ] || { echo "FATAL: AMENT_PREFIX_PATH unset after sourcing /opt/ros/$distro"; exit 1; }
[ "${ROS_DISTRO:-}" = "$distro" ] || { echo "FATAL: ROS_DISTRO='${ROS_DISTRO:-}' is not '$distro'"; exit 1; }

# Expected-state table. Pinned from the 2026-09-23 pre-flight and the first lane run in these exact
# images (jazzy is the validated distro: it must build and pass its whole suite, the regression guard
# for every other row):
#   lyrical: builds from generated bindings and its whole suite is green (the C++ mirror carries the
#            Lyrical tail field under cfg(cerulion_has_is_rosidl_buffer)); first lane run at this
#            state: 31 targets, 441 passed, 0 failed;
#   humble:  builds from generated bindings and its whole suite is green (24-byte GID storage padded,
#            int8 request guids cast, the C++ mirror in its pre-Iron shape under
#            cfg(not(cerulion_has_is_key)));
#   foxy:    the compile stops at the post-Foxy surface (rmw_feature_t and 24 more missing symbols,
#            25 errors).
# A `build` row also pins floors the suite must clear before "green" means anything: at least
# min_targets target summaries and min_tests tests run (ok, failed or ignored), pinned PER ROW from
# that row's first lane run (jazzy and lyrical 2026-09-23: 31 targets, 476 tests run) with margin for
# targets that come and go; a lane that silently loses half its binaries lands below the floor.
case "$distro" in
    jazzy)   expect=build; min_targets=25; min_tests=400; known_failures="" ;;
    lyrical) expect=build; min_targets=25; min_tests=400; known_failures="" ;;
    humble)  expect=build; min_targets=25; min_tests=400; known_failures="" ;;
    foxy)    expect=refuse; errors=25; marker="cannot find type \`rmw_feature_t\` in module \`ffi\`" ;;
    *) echo "FATAL: no expected state for distro '$distro'"; exit 1 ;;
esac

# The failing set as cargo itself reports it: for each target, the names in the FINAL `failures:` list
# (the one followed by that target's `test result:` line), qualified by the target binary. This never
# depends on a `test <name> ... FAILED` line surviving the test's own stdout.
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

log="/tmp/rmw_build_${distro}.log"
echo "== rmw distro lane: $distro (expected: $expect) =="
cargo build -p rmw_cerulion --release 2>&1 | tee "$log"
rc=${PIPESTATUS[0]}
if grep -q -F "VENDORED" "$log"; then
    echo "GATE FAIL: build.rs took the VENDORED-bindings path inside a ROS container"
    exit 1
fi
case "$expect" in
    build)
        [ "$rc" -eq 0 ] || { echo "GATE FAIL: $distro is expected to BUILD against its real headers (rc=$rc)"; exit 1; }
        [ -f target/release/librmw_cerulion.so ] || { echo "GATE FAIL: no librmw_cerulion.so after a successful build"; exit 1; }
        ls target/release/build/rmw_cerulion-*/out/bindings.rs >/dev/null 2>&1 || { echo "GATE FAIL: no generated bindings.rs, the build did not run bindgen"; exit 1; }
        tlog="/tmp/rmw_test_${distro}.log"
        echo "== rmw serial suite on $distro (every target, no fail-fast) =="
        cargo test -p rmw_cerulion --release --no-fail-fast -- --test-threads=1 2>&1 | tee "$tlog"
        rc_test=${PIPESTATUS[0]}
        # "Green" must mean the suite RAN: a crash or a test target that does not compile fails here,
        # every target must report a summary, and the counts must clear the floors.
        if grep -qE "process didn't exit successfully|\(signal: |^error: could not compile|^error\[E[0-9]+\]" "$tlog"; then
            echo "GATE FAIL: $distro suite crashed or a test target did not compile (rc=$rc_test)"; exit 1
        fi
        summaries=$(grep -cE '^test result: ' "$tlog")
        [ "$summaries" -ge "$min_targets" ] || { echo "GATE FAIL: $distro suite reported $summaries target summaries, floor $min_targets"; exit 1; }
        ran=$(grep -E '^test result: ' "$tlog" | awk '{ p += $4; f += $6; i += $8 } END { print p + f + i }')
        [ "$ran" -ge "$min_tests" ] || { echo "GATE FAIL: $distro suite ran $ran tests, floor $min_tests"; exit 1; }
        failed_total=$(grep -E '^test result: ' "$tlog" | awk '{ f += $6 } END { print f + 0 }')
        failed="$(qualified_failures "$tlog")"
        expected="$(printf '%s\n' "${known_failures:-}" | sed '/^$/d' | sort -u)"
        expected_n=$(printf '%s\n' "$expected" | sed '/^$/d' | wc -l | tr -d ' ')
        if [ -z "$expected" ]; then
            [ "$rc_test" -eq 0 ] || { echo "GATE FAIL: $distro suite exited rc=$rc_test with an empty pinned set"; exit 1; }
        else
            [ "$rc_test" -ne 0 ] || { echo "GATE FAIL: $distro suite exited 0 but the table pins $expected_n failures; flip the row"; exit 1; }
        fi
        [ "$failed_total" -eq "$expected_n" ] || { echo "GATE FAIL: $distro summaries count $failed_total failures, the pinned set has $expected_n"; echo "-- got:"; echo "$failed"; exit 1; }
        if [ "$failed" = "$expected" ]; then
            if [ -z "$expected" ]; then echo "GATE PASS: $distro builds from generated bindings and its suite is green ($summaries targets, $ran tests run, rc 0)"
            else echo "GATE PASS (known failures only): $distro builds; $summaries targets, $ran tests run, the failing set is exactly the pinned one:"; echo "$expected"; fi
        else
            echo "GATE FAIL: $distro failing-test set differs from the pinned one."; echo "-- expected:"; echo "$expected"; echo "-- got:"; echo "$failed"
            echo "(a test that started passing or a new failure both land here; update the table in the PR that changes $distro)"; exit 1
        fi
        ;;
    refuse)
        [ "$rc" -ne 0 ] || { echo "GATE FAIL: $distro BUILT, but the table says it is refused today; flip its row in the PR that lands $distro support"; exit 1; }
        grep -q -F -- "$marker" "$log" || { echo "GATE FAIL: $distro failed WITHOUT the known marker; last lines:"; tail -n 40 "$log"; exit 1; }
        count=$(grep -oE 'due to [0-9]+ previous errors?' "$log" | grep -oE '[0-9]+' | tail -n 1)
        [ "${count:-0}" -eq "$errors" ] || { echo "GATE FAIL: $distro stopped with ${count:-0} errors, the table pins $errors; the refusal moved, update the table in the PR that changes $distro"; exit 1; }
        echo "GATE PASS (expected refusal): $distro stops at the known marker with exactly $errors errors"
        ;;
esac
