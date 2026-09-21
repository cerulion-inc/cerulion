#!/usr/bin/env bash
# entrypoint.sh — in-container orchestration for the MoveIt hero demo.
#
# Runs INSIDE the cerulion-moveit-hero image (ROS 2 Jazzy + MoveIt 2 + Panda
# config + Rust), with the repo bind-mounted at /work. It:
#   1. builds rmw_cerulion from /work and registers it on AMENT_PREFIX_PATH
#      (build.rs bindgens against the live distro headers — same wiring the
#      rmw bench uses);
#   2. launches the UNMODIFIED move_group + Panda control stack, headless, on
#      RMW_IMPLEMENTATION=rmw_cerulion;
#   3. sends a scripted OMPL plan via the MoveGroup action and asserts success;
#   4. sends 3 identical pilz PTP plans and asserts bit-identical output
#      (deterministic planning);
#   5. tears the launch down and writes a run log.
#
# Exit: 0 both checks passed; non-zero otherwise (the CI gate reads this).
#
# Env:
#   CER_DEMO_OUT    dir for the run log (default /work/examples/moveit_hero/out)
#   CER_DEMO_CAPTURE=1   also copy move_group's own stdout log into OUT (for
#                        hero-media capture; see README "capture hook")
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${CER_DEMO_REPO:-/work}"
OUT="${CER_DEMO_OUT:-$SCRIPT_DIR/out}"
mkdir -p "$OUT"
RUN_LOG="$OUT/run.log"
# scripted_plan.py writes ompl.json / pilz.json here (machine-readable CI gate).
export CER_DEMO_RESULT_DIR="$OUT"
rm -f "$OUT/ompl.json" "$OUT/pilz.json"

bold() { printf '\033[1m%s\033[0m\n' "$*" | tee -a "$RUN_LOG"; }
note() { printf '  %s\n' "$*" | tee -a "$RUN_LOG"; }

: > "$RUN_LOG"
bold "=== MoveIt hero demo — $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="

# --- ROS environment --------------------------------------------------------
# ROS setup scripts deref unset vars; disable nounset across the source.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# --- build + register rmw_cerulion from /work -------------------------------
# Same recipe as ensure_rmw_cerulion in the latency suite. build.rs takes its
# bindgen-against-real-headers path because AMENT_PREFIX_PATH is already set.
build_rmw_cerulion() {
    local prefix="$OUT/rmw_cerulion_prefix"
    local lib_dir="$prefix/lib"
    local so_name="librmw_cerulion.so"
    if [ ! -f "$REPO/crates/rmw_cerulion/Cargo.toml" ]; then
        echo "entrypoint: rmw_cerulion source not at $REPO/crates/rmw_cerulion" >&2
        echo "  (is the repo bind-mounted at /work? -v \"\$PWD\":/work)" >&2
        exit 2
    fi
    bold "=== building rmw_cerulion (release, against distro headers) ==="
    cargo build --release --manifest-path "$REPO/crates/rmw_cerulion/Cargo.toml" \
        2>&1 | tee -a "$RUN_LOG"
    local built="$REPO/target/release/$so_name"
    [ -f "$built" ] || { echo "entrypoint: $built not found after build" >&2; exit 2; }
    mkdir -p "$lib_dir"
    cp -f "$built" "$lib_dir/$so_name"
    export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    note "rmw_cerulion -> $lib_dir/$so_name"
}

# --- iceoryx2 state cleanup --------------------------------------------------
# rmw_cerulion is iceoryx2-backed; stale /tmp/iceoryx2 + /dev/shm segments from
# a prior run poison the notifier. Clean before we start (safe no-op if empty).
clean_iox2_state() {
    local root="${IOX2_STATE_DIR:-/tmp/iceoryx2}"
    rm -rf "$root/services" "$root/nodes" 2>/dev/null || true
    find "$root" -maxdepth 2 -name 'iox2_*' -type s -exec rm -f {} + 2>/dev/null || true
    find "$root" -maxdepth 2 -name '*.event' -exec rm -f {} + 2>/dev/null || true
    find /dev/shm -maxdepth 1 -name 'iox2*' -exec rm -f {} + 2>/dev/null || true
}

build_rmw_cerulion
clean_iox2_state

export RMW_IMPLEMENTATION=rmw_cerulion
# What we REQUEST. What actually loaded is reported by scripted_plan.py from
# rclpy, and that is what the CI gate asserts on — see its _active_rmw().
bold "=== RMW_IMPLEMENTATION requested: $RMW_IMPLEMENTATION ==="
ros2 doctor --report 2>/dev/null | grep -iA1 'middleware name' | tee -a "$RUN_LOG" || true

# --- launch move_group (headless) -------------------------------------------
MG_LOG="$OUT/move_group.log"
bold "=== launching unmodified move_group + Panda control stack (headless) ==="
ros2 launch "$SCRIPT_DIR/launch_move_group.py" >"$MG_LOG" 2>&1 &
LAUNCH_PID=$!
note "move_group launch pid=$LAUNCH_PID (log: $MG_LOG)"

# shellcheck disable=SC2329 # Called indirectly by the EXIT trap below.
cleanup() {
    bold "=== shutting down move_group (pid $LAUNCH_PID) ==="
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    if [ "${CER_DEMO_CAPTURE:-0}" = "1" ]; then
        note "capture: move_group log at $MG_LOG"
        # Hero-media capture slot: this is where a GIF/asciinema recorder
        # would wrap the plan run. The RViz visual is recorded by hand, separately;
        # this headless run produces the textual evidence + timing.
    fi
}
trap cleanup EXIT

# --- wait for move_group to finish standing up its action server ------------
# WHY: the MoveGroup action topics (move_action/_action/*) are TRANSIENT_LOCAL
# and are created by whichever side opens them first. If the plan client's
# ActionClient opens them before move_group's ActionServer does, the two sides
# disagree on the latched-history buffer size and move_group aborts on
# `DoesNotSupportRequestedMinHistorySize`. Gating the client on move_group's
# "You can start planning now!" readiness line lets move_group create those
# topics first, so the client only ever OPENs an already-correctly-sized
# service. (An iceoryx2 open-or-create ordering constraint, not a MoveIt one.)
bold "=== waiting for move_group to reach planning-ready ==="
READY=0
for _ in $(seq 1 120); do
    if grep -q "You can start planning now" "$MG_LOG" 2>/dev/null; then
        READY=1; note "move_group reported planning-ready"; break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        note "move_group launch exited before becoming ready"; break
    fi
    sleep 1
done
[ "$READY" = "1" ] || note "proceeding without the readiness marker (best effort)"

# --- scripted plans ---------------------------------------------------------
RC=0
bold "=== OMPL plan via MoveGroup action ==="
if python3 "$SCRIPT_DIR/scripted_plan.py" ompl 2>&1 | tee -a "$RUN_LOG"; then
    note "OMPL plan: PASS"
else
    note "OMPL plan: FAIL"
    RC=1
fi

bold "=== pilz determinism (3x identical PTP) ==="
if python3 "$SCRIPT_DIR/scripted_plan.py" pilz 2>&1 | tee -a "$RUN_LOG"; then
    note "pilz determinism: PASS"
else
    note "pilz determinism: FAIL"
    RC=1
fi

bold "=== DEMO $( [ $RC -eq 0 ] && echo PASS || echo FAIL ) (log: $RUN_LOG) ==="
exit "$RC"
