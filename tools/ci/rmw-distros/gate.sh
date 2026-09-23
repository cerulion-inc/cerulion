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
set +u; source "/opt/ros/$distro/setup.bash"; set -u
[ -n "${AMENT_PREFIX_PATH:-}" ] || { echo "FATAL: AMENT_PREFIX_PATH unset after sourcing /opt/ros/$distro"; exit 1; }
[ "${ROS_DISTRO:-}" = "$distro" ] || { echo "FATAL: ROS_DISTRO='${ROS_DISTRO:-}' is not '$distro'"; exit 1; }

# Expected-state table. Pinned from the 2026-09-23 pre-flight in these exact images (jazzy is the
# validated distro: it must build and pass its whole suite, the regression guard for every other row):
#   lyrical: builds from generated bindings (43 s); the lib suite has exactly three known failures,
#            all the C++ introspection bridge refusing the Lyrical layout (no Lyrical-shaped mirror yet);
#   humble:  the compile stops at rmw's 24-byte GID storage against the 16-byte one the crate writes;
#   foxy:    the compile stops at the post-Foxy surface (rmw_feature_t and 24 more missing symbols).
case "$distro" in
    jazzy)   expect=build; known_failures="" ;;
    lyrical) expect=build
             known_failures="api::dispatch_tests::bridge_for_builds_cpp_anybridge_and_flattens_like_native
api::dispatch_tests::direct_leaf_resolves_without_a_dispatcher
api::dispatch_tests::resolves_cpp_arm_for_a_cpp_dispatcher" ;;
    humble)  expect=refuse; marker="expected an array with a size of 24, found one with a size of 16" ;;
    foxy)    expect=refuse; marker="cannot find type \`rmw_feature_t\` in module \`ffi\`" ;;
    *) echo "FATAL: no expected state for distro '$distro'"; exit 1 ;;
esac

log="/tmp/rmw_build_${distro}.log"
echo "== rmw distro lane: $distro (expected: $expect) =="
cargo build -p rmw_cerulion --release 2>&1 | tee "$log"
rc=${PIPESTATUS[0]}
if grep -q -i "vendored bindings" "$log"; then
    echo "GATE FAIL: build.rs took the VENDORED-bindings path inside a ROS container"
    exit 1
fi
case "$expect" in
    build)
        [ "$rc" -eq 0 ] || { echo "GATE FAIL: $distro is expected to BUILD against its real headers (rc=$rc)"; exit 1; }
        ls -l target/release/librmw_cerulion.so
        ls target/release/build/rmw_cerulion-*/out/bindings.rs >/dev/null || { echo "GATE FAIL: no generated bindings.rs, the build did not run bindgen"; exit 1; }
        echo "== rmw serial suite on $distro (every target, no fail-fast) =="
        cargo test -p rmw_cerulion --release --no-fail-fast -- --test-threads=1 2>&1 | tee "/tmp/rmw_test_${distro}.log"
        failed="$(grep -E '^test .* FAILED$' "/tmp/rmw_test_${distro}.log" | sed -E 's/^test (.*) \.\.\. FAILED$/\1/' | sort -u)"
        expected="$(printf '%s\n' "${known_failures:-}" | sed '/^$/d' | sort -u)"
        if [ "$failed" = "$expected" ]; then
            if [ -z "$expected" ]; then echo "GATE PASS: $distro builds from generated bindings and its suite is green"
            else echo "GATE PASS (known failures only): $distro builds; the failing set is exactly the pinned one:"; echo "$expected"; fi
        else
            echo "GATE FAIL: $distro failing-test set differs from the pinned one."; echo "-- expected:"; echo "$expected"; echo "-- got:"; echo "$failed"
            echo "(a test that started passing or a new failure both land here; update the table in the PR that changes $distro)"; exit 1
        fi
        ;;
    refuse)
        [ "$rc" -ne 0 ] || { echo "GATE FAIL: $distro BUILT, but the table says it is refused today; flip its row in the PR that lands $distro support"; exit 1; }
        grep -q -F -- "$marker" "$log" || { echo "GATE FAIL: $distro failed WITHOUT the known marker; last lines:"; tail -n 40 "$log"; exit 1; }
        echo "GATE PASS (expected refusal): $distro stops at the known marker"
        ;;
esac
