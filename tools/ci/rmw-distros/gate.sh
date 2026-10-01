#!/usr/bin/env bash
# rmw distro lane gate: PASS means the tree's KNOWN state for this distro holds.
#
# Each distro has an expected state in the table below. `build` means rmw_cerulion must compile
# against the distro's real headers (generated bindings, never the vendored fallback), must export
# no symbol the distro's headers do not declare, and (on the x86_64 lanes) its serial test suite
# must pass. After the build, a `build` row also runs a REAL rclpy cross-process exchange over
# rmw_cerulion in this distro's container (both directions, the three #[ignore]'d tests in
# rclpy_xproc_test.rs run with --ignored against the freshly staged .so), with a wrong-payload
# self-test that must red and a staged-vs-built digest check, so a green lane proves a real rclpy
# peer talks to native Cerulion on this distro. The emulated aarch64 leg sets
# RMW_GATE_SKIP_SERIAL_SUITE=1 to defer the in-process serial suite (the x86_64 lanes always run
# it); the build, symbol audit, staged digest and the exchange still run there. `refuse`
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
# A `build` row also pins floors the suite must clear before "green" means anything (on the lanes
# that run the suite; the emulated aarch64 leg defers it): at least
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
        # The in-process serial suite runs on every lane EXCEPT when a caller sets
        # RMW_GATE_SKIP_SERIAL_SUITE=1 AND the lane is not x86_64 (serial_suite_runs in harvest.sh,
        # self-tested in gate_selftest.sh). The emulated aarch64 leg sets it; the x86_64 lanes run the
        # suite unconditionally, so a skip variable leaked into a shared env cannot empty their floors.
        if serial_suite_runs; then
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
        else
            echo "== in-process serial suite SKIPPED on $distro (RMW_GATE_SKIP_SERIAL_SUITE=1); the gate still runs build + symbol audit + staged-library digest + rclpy exchanges + wrong-payload self-test =="
        fi
        # ===================================================================
        # rclpy CROSS-PROCESS EXCHANGE (item: distro-lane rclpy step).
        # A REAL python3 rclpy process talks to in-process native Cerulion over
        # rmw_cerulion in BOTH directions, inside this distro's
        # ros:<distro>-ros-base container. Reuses the three #[ignore]'d tests in
        # crates/rmw_cerulion/tests/rclpy_xproc_test.rs (run with --ignored).
        # Runs after the build and symbol audit; on the x86_64 lanes the serial
        # suite has also run green first, and on the emulated leg the freshly built
        # and staged .so is the known-good basis. The lane is never a required
        # context (see .github/workflows/rmw-distros.yml), so this section reds a
        # PR check without blocking anything.
        # -------------------------------------------------------------------
        RCLPY_TIMEOUT="${RCLPY_TIMEOUT:-600}"      # hard wall for one exchange run (s)
        EXCHANGES_EXPECTED=3                        # both directions: A (1) + B twist+string (2); == the 3 #[ignore]'d exchange tests run with --ignored (NOT the #[test] count, which includes the non-ignored seam unit arm)
        PREFIX="${RCLPY_PREFIX:-/tmp/rmw_prefix_${distro}}"
        so_built="target/release/librmw_cerulion.so"
        so_staged="$PREFIX/lib/librmw_cerulion.so"

        # (pre) The build output must exist (the build check above asserted it; re-assert with
        # a section-specific reason so a regression here names THIS step).
        staged_so_present "$so_built" "$distro" || exit 1

        # (Arm 2b) Stage the freshly built .so into an ament prefix and PROVE the
        # staged copy is byte-identical to the build output: the exchange must load
        # the .so THIS job just built, never a stale one left in $PREFIX. Combined
        # with $PREFIX being FIRST on AMENT_PREFIX_PATH below, this pins which
        # library rmw dlopens.
        mkdir -p "$PREFIX/lib"
        cp -f "$so_built" "$so_staged" || { echo "GATE FAIL: $distro rclpy exchange - could not stage $so_built -> $so_staged"; exit 1; }
        dig_built=$(sha256sum "$so_built"  | cut -d' ' -f1)
        dig_staged=$(sha256sum "$so_staged" | cut -d' ' -f1)
        [ "$dig_built" = "$dig_staged" ] || { echo "GATE FAIL: $distro rclpy exchange - staged .so digest ($dig_staged) != build-output digest ($dig_built); the exchange would not run against the freshly built library"; exit 1; }
        echo "STAGED .so DIGEST OK ($distro): $dig_built  ($so_staged == $so_built)"

        # (pre) python3 + rclpy + the two message packages must import under the
        # sourced ROS env (the sourced setup.bash sets PYTHONPATH). ros:<distro>-ros-base
        # ships all three (rclpy and common_interfaces, which provides std_msgs and
        # geometry_msgs, are in the ros-base package set, REP 2001). A missing one is
        # a NAMED failure, never a silent skip.
        py_probe=$(python3 -c 'import rclpy, std_msgs.msg, geometry_msgs.msg; print("ok")' 2>&1) || true
        rclpy_probe_ok "$py_probe" "$distro" || exit 1

        # (item 1) POSITIVE arm: run EACH ignored exchange test in its OWN cargo
        # invocation (a FRESH process), so the process-global native
        # TransportManager singleton is fresh for every direction and no live
        # singleton is ever swept out from under a running test. RMW_IMPLEMENTATION
        # is set and the staged prefix is FIRST on both search paths, under a hard
        # wall. Clean iox SHM BEFORE EACH invocation (the previous invocation and
        # the serial suite, when it ran, both leave services behind).
        # [N3] The serial suite compiles the exchange test target; when the suite is
        # deferred (emulated leg) nothing has, and the test-target build (it flips the
        # test-seams/test-helpers features on two in-tree crates) would otherwise fall
        # inside a per-exchange wall. Compile it ONCE here, OUTSIDE every wall, so each
        # timed invocation below is a build-cache hit that only RUNS its one test.
        echo "== pre-build the rclpy exchange test target on $distro (outside the per-exchange wall) =="
        cargo test --locked -p rmw_cerulion --release --test rclpy_xproc_test --no-run 2>&1 | tee "/tmp/rmw_rclpy_${distro}_build.log"
        [ "${PIPESTATUS[0]}" -eq 0 ] || { echo "GATE FAIL: $distro rclpy exchange test target did not compile"; exit 1; }
        # Each run logs to its OWN file and is judged off the colour-stripped copy
        # (no truncating pipe on cargo). One passing invocation per name IS the
        # identity proof (rule 37): its summary must read exactly `1 passed; 0 failed`
        # AND its own `test <name> ... ok` line must be present - so the earlier
        # separate identity-pin loop is SUBSUMED and removed.
        exchanges_ok=0
        for xt in \
            direction_a_rclpy_string_talker_to_native_subscriber \
            direction_b_native_twist_publisher_to_rclpy_listener \
            direction_b_native_string_publisher_to_rclpy_listener; do
            rm -rf /tmp/iceoryx2 /dev/shm/iox2_* 2>/dev/null || true
            xlog="/tmp/rmw_rclpy_${distro}_${xt}.log"
            echo "== rmw rclpy cross-process exchange on $distro: $xt (fresh process, --ignored) =="
            timeout --kill-after=30 "$RCLPY_TIMEOUT" \
                env RMW_IMPLEMENTATION=rmw_cerulion \
                    AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH" \
                    LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
                cargo test --locked -p rmw_cerulion --release --test rclpy_xproc_test -- \
                    --ignored --test-threads=1 --exact "$xt" 2>&1 | tee "$xlog"
            rc_rclpy=${PIPESTATUS[0]}
            # timeout returns 124 on TERM-at-deadline, 137 if it had to escalate to KILL.
            rclpy_timed_out "$rc_rclpy" "$distro" "rclpy exchange '$xt'" "$RCLPY_TIMEOUT" && exit 1
            plain_x="${xlog}.plain"; strip_ansi "$xlog" > "$plain_x"
            if crashed "$plain_x"; then
                echo "GATE FAIL: $distro rclpy exchange '$xt' crashed or its test target did not compile (rc=$rc_rclpy)"; exit 1
            fi
            # Prove it RAN (rule 37): --test selects exactly one binary and the
            # single name filter selects exactly one test, so there is exactly one
            # `test result:` line and it must read `1 passed; 0 failed`. A MISSING
            # summary means the binary never ran.
            summary_x="$(grep -E '^test result: ' "$plain_x" | tail -n 1)"
            [ -n "$summary_x" ] || { echo "GATE FAIL: $distro rclpy exchange '$xt' printed NO 'test result:' summary - the binary did not run (rc=$rc_rclpy); last lines:"; tail -n 40 "$plain_x"; exit 1; }
            passed_x=$(printf '%s\n' "$summary_x" | awk '{print $4}')   # "test result: ok. N passed; M failed; ..."
            failed_x=$(printf '%s\n' "$summary_x" | awk '{print $6}')
            # M5: REFUSE an unparseable passed OR failed count. A non-numeric failed
            # count is NEVER coerced to 0 - that would pass a torn/partial summary.
            case "$passed_x" in ''|*[!0-9]*) echo "GATE FAIL: $distro rclpy exchange '$xt' - unparseable passed count in: $summary_x"; exit 1 ;; esac
            case "$failed_x" in ''|*[!0-9]*) echo "GATE FAIL: $distro rclpy exchange '$xt' - unparseable failed count in: $summary_x"; exit 1 ;; esac
            [ "$rc_rclpy" -eq 0 ] || { echo "GATE FAIL: $distro rclpy exchange '$xt' exited rc=$rc_rclpy"; exit 1; }
            [ "$failed_x" -eq 0 ] || { echo "GATE FAIL: $distro rclpy exchange '$xt' summary reports $failed_x failed"; exit 1; }
            [ "$passed_x" -eq 1 ] || { echo "GATE FAIL: $distro rclpy exchange '$xt' summary reports $passed_x passed, expected exactly 1 (one name filter, one fresh process); a zero or other count is a non-run (rule 37): $summary_x"; exit 1; }
            # Identity BY CONSTRUCTION: the named test's own pass line must be
            # present, so a rename/drop reds here even if the count somehow held.
            grep -qE "^test ${xt} \.\.\. ok$" "$plain_x" || { echo "GATE FAIL: $distro rclpy exchange '$xt' summary passed but the named '^test ${xt} ... ok' line is absent; the pass is not this exchange (rule 37)"; echo "-- test lines seen:"; grep -E "^test " "$plain_x" | head -20; exit 1; }
            echo "EXCHANGE OK ($distro): $xt passed in its own fresh process (1 passed; 0 failed)"
            exchanges_ok=$((exchanges_ok + 1))
        done
        # Belt-and-suspenders: the loop `exit 1`s on any failure, so reaching here
        # means every named exchange passed; assert the count equals the pin so a
        # future edit that shortens the name list cannot quietly pass fewer.
        [ "$exchanges_ok" -eq "$EXCHANGES_EXPECTED" ] || { echo "GATE FAIL: $distro ran $exchanges_ok/$EXCHANGES_EXPECTED fresh-process rclpy exchanges (rule 37)"; exit 1; }
        echo "EXCHANGE IDENTITIES PINNED ($distro): direction_a (rclpy->native) and direction_b twist + string (native->rclpy) each passed by name in its own fresh process"
        echo "GATE PASS (rclpy exchange): $distro ran $exchanges_ok/$EXCHANGES_EXPECTED cross-process rclpy exchanges over rmw_cerulion, each in a FRESH process (both directions)"
        echo "GATE TABLE | distro=$distro | judged=rclpy_xproc | exchanges=$exchanges_ok/$EXCHANGES_EXPECTED | directions=A(rclpy_talker->native),B(native->rclpy:twist,string) | procs=fresh-per-test | staged_so=$dig_built | rc=0"

        # (Arm 2a) NEGATIVE self-test: inject a WRONG payload through the harness
        # seam and PROVE the exchange oracle rejects it - otherwise the positive
        # arm could pass vacuously (a toothless oracle, an inert exchange). The
        # oracle here is an INDEPENDENT LITERAL invented in THIS script
        # (GATE_WRONG_SENTINEL), never read from the harness oracle (b-1..b-5) nor
        # from any talker (Direction B has none). Two-sided:
        #   (a) the run MUST fail (the harness value oracle caught the mismatch), and
        #   (b) the injected literal MUST appear in the rclpy child's RECV output
        #       (the wrong payload really crossed the iceoryx2 boundary to a real
        #       rclpy process - the RED is a payload mismatch, not an unrelated crash).
        GATE_WRONG_SENTINEL="__cerulion_gate_selftest_wrong_payload__"
        rm -rf /tmp/iceoryx2 /dev/shm/iox2_* 2>/dev/null || true
        slog="/tmp/rmw_rclpy_selftest_${distro}.log"
        echo "== rmw rclpy exchange SELF-TEST on $distro (wrong payload MUST red) =="
        timeout --kill-after=30 "$RCLPY_TIMEOUT" \
            env RMW_IMPLEMENTATION=rmw_cerulion \
                AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH" \
                LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
                CERULION_RCLPY_XPROC_FORCE_WRONG_PAYLOAD="$GATE_WRONG_SENTINEL" \
            cargo test --locked -p rmw_cerulion --release --test rclpy_xproc_test -- \
                --ignored --test-threads=1 --nocapture \
                direction_b_native_string_publisher_to_rclpy_listener 2>&1 | tee "$slog"
        rc_self=${PIPESTATUS[0]}
        rclpy_timed_out "$rc_self" "$distro" "rclpy exchange SELF-TEST" "$RCLPY_TIMEOUT" "(expected a fast RED, not a hang)" && exit 1
        splain="${slog}.plain"; strip_ansi "$slog" > "$splain"
        # (a) the wrong-payload run MUST have failed.
        [ "$rc_self" -ne 0 ] || { echo "GATE FAIL: $distro rclpy exchange SELF-TEST(2a): the wrong-payload run PASSED; the exchange does NOT detect a payload mismatch"; exit 1; }
        scounts="$(suite_counts "$splain")"
        s_ran=$(printf '%s' "$scounts" | awk '{print $2}')
        s_fail=$(printf '%s' "$scounts" | awk '{print $3}')
        [ "${s_fail:-0}" -ge 1 ] || { echo "GATE FAIL: $distro rclpy exchange SELF-TEST(2a): no failed test in the wrong-payload run (ran=${s_ran:-0}); the RED is not a real test failure"; exit 1; }
        # (b) the gate's OWN literal must appear in the exchange's FAILURE output.
        # The harness prints the values it collected from the rclpy child in its
        # assertion panic ("got [...]"), so the injected wrong payload the child
        # received and the native side collected shows up in the cargo log. The
        # child's own RECV lines are drained programmatically by the harness and
        # never reach this log, so match the bare literal, not a "RECV " prefix.
        grep -q -F "$GATE_WRONG_SENTINEL" "$splain" || { echo "GATE FAIL: $distro rclpy exchange SELF-TEST(2a): the injected literal ($GATE_WRONG_SENTINEL) never appears in the exchange's failure output; the RED is not a proven payload mismatch"; echo "-- output tail:"; tail -n 40 "$splain"; exit 1; }
        echo "GATE SELF-TEST PASS (2a): $distro wrong payload ($GATE_WRONG_SENTINEL) crossed the exchange and the oracle REJECTED it ($s_fail failed; the literal is in the collected values)"
        rm -rf /tmp/iceoryx2 /dev/shm/iox2_* 2>/dev/null || true
        # ===================================================================
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
