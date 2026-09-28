#!/usr/bin/env bash
# rmw distro lane gate: PASS means the tree's KNOWN state for this distro holds.
#
# Each distro has an expected state in the table below. `build` means rmw_cerulion must compile
# against the distro's real headers (generated bindings, never the vendored fallback), must export
# no symbol the distro's headers do not declare, and its serial test suite must pass. `refuse`
# means the build is expected to stop at a KNOWN place, and
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
#            Lyrical tail field under cfg(cerulion_has_is_rosidl_buffer)); first run at this state:
#            33 targets, 479 tests run, 0 failed;
#   humble:  builds from generated bindings and its whole suite is green (24-byte GID storage padded,
#            int8 request guids cast, the C++ mirror in its pre-Iron shape under
#            cfg(not(cerulion_has_is_key)));
#   galactic: builds from generated bindings and its whole suite is green (the same 96-byte
#             pre-Humble C++ mirror as foxy, and seven post-Galactic rmw entry points compiled
#             out whole, three fewer than foxy: Galactic declares the qos compatibility call and
#             the two network flow calls);
#   foxy:    builds from generated bindings and its whole suite is green (the C++ mirror in its
#            96-byte pre-Humble shape under cfg(not(cerulion_has_fetch_function)), and the ten
#            post-Foxy rmw entry points compiled out whole rather than stubbed).
# Every `build` row also names the entry points the distro's headers do NOT declare, in
# absent_symbols: the built library must define none of them. rcl resolves by name, so a defined
# symbol is a claim the distro cannot back, and a stub that answers UNSUPPORTED is worse than no
# symbol at all. The audit reads the library with `nm` and also requires the control symbol
# rmw_init, so an empty or unreadable symbol table can never pass it.
# A `build` row also pins floors the suite must clear before "green" means anything: at least
# min_targets target summaries and min_tests tests run (ok, failed or ignored), pinned PER ROW from
# that row's first lane run (jazzy and lyrical 2026-09-23: 31 targets, 476 tests run; 33 and 479
# with the two vendored-gate binaries) with margin for
# targets that come and go; a lane that silently loses half its binaries lands below the floor.
# The ten entry points Foxy's rmw headers do not declare: rmw_event_set_callback,
# rmw_subscription_set_on_new_message_callback, rmw_service_set_on_new_request_callback and
# rmw_client_set_on_new_response_callback (Humble), the two content filter calls (Humble),
# rmw_feature_supported (Humble), and rmw_qos_profile_check_compatible plus the two network
# flow calls (Galactic).
foxy_absent_symbols="rmw_event_set_callback
rmw_subscription_set_on_new_message_callback
rmw_service_set_on_new_request_callback
rmw_client_set_on_new_response_callback
rmw_subscription_set_content_filter
rmw_subscription_get_content_filter
rmw_publisher_get_network_flow_endpoints
rmw_subscription_get_network_flow_endpoints
rmw_qos_profile_check_compatible
rmw_feature_supported"
# Galactic's list is DERIVED from the same place the cfgs are: every guarded export sits behind
# #[cfg(cerulion_has_<cap>)], and CAPABILITY_MIN_ERA in crates/rmw_cerulion/src/era_check.rs
# gives each capability the era whose headers introduced it. A distro lacks exactly the exports
# whose capability is LATER than its own era, so Galactic's list is Foxy's without the three
# Galactic-era entry points (qos_compatibility, network_flow): the event_callback exports, the
# content_filter_options exports and rmw_feature_supported (features), all Humble. Seven.
# crates/rmw_cerulion/tests/rmw_absent_export_table_test.rs recomputes every row's list from the
# guarded exports and the era tables and fails on any difference.
galactic_absent_symbols="rmw_event_set_callback
rmw_subscription_set_on_new_message_callback
rmw_service_set_on_new_request_callback
rmw_client_set_on_new_response_callback
rmw_subscription_set_content_filter
rmw_subscription_get_content_filter
rmw_feature_supported"
# No row is `refuse` today; the arm stays for the next distro that starts there, so its two inputs
# carry inert defaults rather than being unset under `set -u`. The arm REFUSES those defaults
# (refuse_row_pinned): an errors=0 or empty-marker row would pass vacuously.
errors=0
marker=""
case "$distro" in
    jazzy)   expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="" ;;
    lyrical) expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="" ;;
    humble)  expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="" ;;
    galactic) expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="$galactic_absent_symbols" ;;
    foxy)    expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="$foxy_absent_symbols" ;;
    *) echo "FATAL: no expected state for distro '$distro'"; exit 1 ;;
esac

# shellcheck source=tools/ci/rmw-distros/harvest.sh
source "$(dirname "$0")/harvest.sh"

log="/tmp/rmw_build_${distro}.log"
echo "== rmw distro lane: $distro (expected: $expect) =="
cargo build --locked -p rmw_cerulion --release 2>&1 | tee "$log"
rc=${PIPESTATUS[0]}
# Everything below reads the build log with terminal colour stripped (see harvest.sh), the same
# way the suite log is read, so a change in cargo's colour setting can never hide a marker.
plain_log="${log}.plain"; strip_ansi "$log" > "$plain_log"
if grep -q -F "VENDORED" "$plain_log"; then
    echo "GATE FAIL: build.rs took the VENDORED-bindings path inside a ROS container"
    exit 1
fi
case "$expect" in
    build)
        [ "$rc" -eq 0 ] || { echo "GATE FAIL: $distro is expected to BUILD against its real headers (rc=$rc)"; exit 1; }
        [ -f target/release/librmw_cerulion.so ] || { echo "GATE FAIL: no librmw_cerulion.so after a successful build"; exit 1; }
        ls target/release/build/rmw_cerulion-*/out/bindings.rs >/dev/null 2>&1 || { echo "GATE FAIL: no generated bindings.rs, the build did not run bindgen"; exit 1; }
        # The export set must match what this distro's headers declare: none of absent_symbols
        # defined, and the control symbol present so a zero finding means something.
        nmlog="/tmp/rmw_symbols_${distro}.txt"
        nm -D --defined-only target/release/librmw_cerulion.so > "$nmlog" || { echo "GATE FAIL: nm could not read librmw_cerulion.so"; exit 1; }
        symbol_audit "$nmlog" "$absent_symbols" || { echo "GATE FAIL: $distro exports a symbol its headers do not declare"; exit 1; }
        absent_n=$(printf '%s\n' "$absent_symbols" | sed '/^$/d' | wc -l | tr -d ' ')
        echo "SYMBOL AUDIT PASS: $NM_CONTROL_SYMBOL defined and $absent_n header-absent symbol(s) undefined"
        tlog="/tmp/rmw_test_${distro}.log"
        echo "== rmw serial suite on $distro (every target, no fail-fast) =="
        cargo test --locked -p rmw_cerulion --release --no-fail-fast -- --test-threads=1 2>&1 | tee "$tlog"
        rc_test=${PIPESTATUS[0]}
        # Everything below reads the log with terminal colour stripped (see harvest.sh).
        plain="${tlog}.plain"; strip_ansi "$tlog" > "$plain"
        # "Green" must mean the suite RAN: a crash or a test target that does not compile fails here,
        # every target must report a summary, and the counts must clear the floors.
        if crashed "$plain"; then
            echo "GATE FAIL: $distro suite crashed or a test target did not compile (rc=$rc_test)"; exit 1
        fi
        read -r summaries ran failed_total <<< "$(suite_counts "$plain")"
        [ "$summaries" -ge "$min_targets" ] || { echo "GATE FAIL: $distro suite reported $summaries target summaries, floor $min_targets"; exit 1; }
        [ "$ran" -ge "$min_tests" ] || { echo "GATE FAIL: $distro suite ran $ran tests, floor $min_tests"; exit 1; }
        failed="$(qualified_failures "$plain")"
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
        # The row must pin its refusal for real before anything is compared against it.
        refuse_row_pinned "$errors" "$marker" || exit 1
        [ "$rc" -ne 0 ] || { echo "GATE FAIL: $distro BUILT, but the table says it is refused today; flip its row in the PR that lands $distro support"; exit 1; }
        grep -q -F -- "$marker" "$plain_log" || { echo "GATE FAIL: $distro failed WITHOUT the known marker; last lines:"; tail -n 40 "$plain_log"; exit 1; }
        count=$(grep -oE 'due to [0-9]+ previous errors?' "$plain_log" | grep -oE '[0-9]+' | tail -n 1)
        [ "${count:-0}" -eq "$errors" ] || { echo "GATE FAIL: $distro stopped with ${count:-0} errors, the table pins $errors; the refusal moved, update the table in the PR that changes $distro"; exit 1; }
        echo "GATE PASS (expected refusal): $distro stops at the known marker with exactly $errors errors"
        ;;
esac
