#!/usr/bin/env bash
# verify_shm.sh <rmw> <mode> — SHM-engagement smoke for one (rmw, mode).
#
# Smoke-publishes once under the given RMW + SHM mode and checks the
# RMW's SHM engagement marker. Exits non-zero if the marker is absent
# in shm/zc mode (i.e., SHM silently fell back to UDP / heap copies —
# a misleading bench label).
#
# In no_shm mode, the publish must succeed but no marker is required.
#
# QoS threading: the probe publishes under the
# cell's CER_BENCH_QOS (be1 | rel10, inherited from run_bench.sh;
# default be1), mapped onto `ros2 topic pub`'s --qos-* flags. SHM
# eligibility can be QoS-gated — CycloneDDS 0.10.5's documentation
# says RELIABLE is required for iceoryx exchange while its code gates
# only check that reliability is PRESENT, so a BEST_EFFORT lane
# engaging (or not engaging) SHM must be MEASURED per QoS lane, not
# assumed. A be1-ineligible SHM silently falling back to UDP is
# exactly the mislabeling this gate exists to catch.
#
# Markers per RMW (when SHM is engaged):
#   cyclonedds  → STRUCTURAL, two conditions BOTH required in the
#                 iox-roudi debug-log DELTA the probe caused (read via
#                 $IOX_ROUDI_LOG):
#                   1. "Registered new application" — proves the
#                      iceoryx plugin/bridge LOADED (on 0.10.5 the
#                      runtime name is "iceoryx_rt_<pid>_*"; on
#                      lyrical/CycloneDDS 11.x it is
#                      "CycloneDDS-iox_psmx-<16-hex>" — the grep
#                      deliberately omits the name so it matches both).
#                   2. a "Created new PublisherPort"/"Created new
#                      SubscriberPort" line referencing the DDS_CYCLONE
#                      service — proves an ENDPOINT actually qualified
#                      for iceoryx exchange during the probe window.
#                      Registration alone happens at participant init
#                      regardless of whether ANY endpoint's QoS/type
#                      passes the SHM gate, so it must not pass
#                      alone.
#                 The apt builds emit no grep-able iceoryx marker of
#                 their own.
#   fastdds     → STRUCTURAL /dev/shm DELTA while the probe is ALIVE
#                 (measured 2026-08-12 on box-x86, all three
#                 distro images, both directions). The apt Fast DDS
#                 builds (2.6.11 humble / 2.14.6 jazzy / 3.6.x
#                 lyrical) print NO SHM string to any stream at
#                 default verbosity — Info-level logging is compiled
#                 out of release builds, so the old
#                 'SharedMemTransport|SHM transport' grep NEVER
#                 matched (permanent false failure, the cyclonedds
#                 disease). Instead: the SHM transport is backed by
#                 named /dev/shm files that exist only while a
#                 participant holds them, so the probe HOLDS a 1 Hz
#                 publisher open and the /dev/shm delta vs a pre-probe
#                 snapshot is checked. mode=shm requires BOTH a data
#                 segment ≥ 1 MB (proves OUR ≥64 MiB tuned descriptor
#                 instantiated — measured 67-70 MB; a builtin-default
#                 SHM segment is ~549 KB, so a silently-unloaded
#                 profile cannot pass) AND a port file (an SHM locator
#                 listener). File prefix is fastrtps_ on Fast DDS 2.x
#                 (humble/jazzy) and fastdds_ on 3.x (lyrical) — both
#                 matched. mode=zc requires a fast_datasharing_*
#                 writer-history segment in the delta: measured on
#                 jazzy 2.14 AND lyrical 3.6, a data_sharing AUTOMATIC
#                 writer creates it at writer creation even with no
#                 subscriber (the earlier claim that a subscriber-less
#                 probe cannot structurally prove DataSharing was
#                 wrong). DELTA, not presence: 3.x does NOT unlink its
#                 segments on exit (measured on lyrical; run_bench.sh's
#                 clean_shm sweep predates the 3.x fastdds_ rename), so
#                 stale files from a prior run must never satisfy the
#                 gate. no_shm direction measured on all three distros:
#                 a UDP-only participant creates ZERO fast(rtps|dds)_*
#                 files.
#   zenoh       → probe-log markers at DEBUG level (measured
#                 2026-08-12 on humble 0.1.9 / jazzy 0.2.9 / lyrical
#                 0.10.5, both directions). BOTH required:
#                   1. "New transport opened ... shm: Some(
#                      TransportShmConfig" (zenoh_transport) — the
#                      probe↔router link NEGOTIATED SHM; the no_shm
#                      direction reads "shm: None" on the same line.
#                   2. "Created SHM segment" (zenoh_shm::posix_shm) —
#                      the probe process actually allocated SHM
#                      segments (watchdog + the mode="init" pool).
#                 AND the absence of two fallback signatures:
#                   - "Failed to insert value" / "Ignore the invalid
#                     configuration" — ZENOH_CONFIG_OVERRIDE pair
#                     REJECTED, session silently runs the shipped
#                     config (exactly what the unquoted mode=init
#                     shipped by the C3 rebase did — see the zenoh
#                     case arm below).
#                   - zenoh 1.8's "Error creating lazy ShmProvider" —
#                     the one RUST_LOG-visible trace of the
#                     silent-TCP-fallback failure mode (e.g. mlock
#                     over RLIMIT_MEMLOCK).
#                 The old markers (Watchdog Confirmator/Validator at
#                 zenoh_shm=info) never appeared: those lines are
#                 DEBUG/WARN-level and only exist once the SHM
#                 subsystem actually initializes, which the rejected
#                 mode=init override left lazy — the gate failed on
#                 every distro.
# (rmw=cerulion is rmw_cerulion, whose one data path is iceoryx2 shared
# memory; see the cerulion arm below.)
#
# Args:
#   rmw   one of: cyclonedds | fastdds | zenoh | cerulion
#   mode  one of: shm | no_shm | zc   (zc: fastdds only — the
#         DataSharing lane, configs/fastdds_zc.xml +
#         RMW_FASTRTPS_USE_QOS_FROM_XML=1)
#
# Env:
#   CER_BENCH_FASTDDS_PROFILE  absolute path of the Fast DDS profiles XML
#         to verify, overriding the derived configs/fastdds_<mode>.xml.
#         run_bench.sh sets it so the 16 MB cell's 256 MiB-segment profile
#         is verified with the SAME profile it measures with. Ignored
#         outside the fastdds non-zc arm; a missing path is refused (2).
#   IOX_ROUDI_LOG  iox-roudi's -l debug log — the cyclonedds oracle's only
#         evidence. run_bench.sh exports it; a standalone caller must.
#
# NOTE: leaves probe-participant SHM state behind by design — the
# caller (run_bench.sh) wipes it before the first measurement run.

set -uo pipefail

RMW="${1:-}"
MODE="${2:-}"

if [ -z "$RMW" ] || [ -z "$MODE" ]; then
    echo "usage: $0 <cyclonedds|fastdds|zenoh|cerulion> <shm|no_shm|zc>" >&2
    exit 2
fi
case "$MODE" in
    shm|no_shm|zc) ;;
    *)
        echo "verify_shm.sh: mode must be 'shm', 'no_shm' or 'zc', got '$MODE'" >&2
        exit 2
        ;;
esac
if [ "$MODE" = "zc" ] && [ "$RMW" != "fastdds" ]; then
    echo "verify_shm.sh: mode 'zc' (FastDDS DataSharing lane) exists for rmw=fastdds only, got rmw=$RMW" >&2
    exit 2
fi
if [ "$RMW" = "cerulion" ] && [ "$MODE" != "shm" ]; then
    # Mirrors run_bench.sh's own cross-check, and for the same reason:
    # iceoryx2 shared memory is rmw_cerulion's only data path, so there is
    # no second mode to verify. Refused rather than remapped onto `shm`.
    echo "verify_shm.sh: rmw=cerulion has one data path (iceoryx2 shared memory) - mode must be 'shm', got '$MODE'" >&2
    exit 2
fi

# The cell's QoS lane — the probe must publish
# under the SAME QoS the bench cell will use, because SHM eligibility
# can be QoS-gated (see header). Inherited from run_bench.sh's export.
CER_BENCH_QOS="${CER_BENCH_QOS:-be1}"
case "$CER_BENCH_QOS" in
    be1)
        QOS_ARGS=(--qos-reliability best_effort --qos-durability volatile
                  --qos-history keep_last --qos-depth 1)
        ;;
    rel10)
        QOS_ARGS=(--qos-reliability reliable --qos-durability volatile
                  --qos-history keep_last --qos-depth 10)
        ;;
    *)
        echo "verify_shm.sh: CER_BENCH_QOS must be 'be1' or 'rel10', got '$CER_BENCH_QOS'" >&2
        exit 2
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/configs"

# A pid is REUSABLE; an identity is not. `pid_identity` returns the
# process's absolute start time (`ps -o lstart=`, present on both BSD and
# procps), so a recorded (pid, start) pair still names the SAME process
# minutes later — a pid the kernel recycled reports a different start time
# and is refused. This is the identity the ownership flags claim to hold:
# without it, `ZENOHD_OWNED=1` plus a recycled pid means teardown kills a
# stranger, which is exactly what the flag exists to prevent.
#
# RESIDUAL, stated because a shell cannot close it: `ps` then `kill` is
# not atomic and there is no portable check-and-signal (pidfd is Linux
# only and unreachable from bash). The window shrinks from the whole run
# to the microseconds between two adjacent calls, and a victim must ALSO
# carry the right comm AND have started in the same second.
pid_identity() {
    [ -n "${1:-}" ] || return 0
    ps -p "$1" -o lstart= 2>/dev/null | sed 's/^ *//; s/ *$//'
}

# Signal a pid only while it is still the process we recorded: same
# start time, and still named `$2`. Any mismatch is a no-op, never a
# guess.
#
# RETURNS 0 only when it actually SIGNALLED; non-zero from each refusal
# arm. Callers that follow this with `wait` need that: waiting on a
# process we did NOT signal blocks until it exits on its own, which for
# the `timeout 20` probe is up to 20 s added to an EXIT path.
kill_owned_pid() {
    local pid="$1" want_id="$2" want_comm="$3" sig="${4:-KILL}"
    [ -n "$pid" ] && [ -n "$want_id" ] || return 1
    [ "$(pid_identity "$pid")" = "$want_id" ] || return 1
    # An EMPTY want_comm skips the name check — for a process whose comm is
    # not stable enough to gate on. `ros2 run` is a Python wrapper that may
    # report `ros2`, `python3` or the exec'd binary depending on the path it
    # took, so a comm check there false-negatives on a launcher that IS
    # ours. The start-time identity still pins it.
    if [ -n "$want_comm" ]; then
        [ "$(ps -p "$pid" -o comm= 2>/dev/null | awk '{print $1}')" = "$want_comm" ] ||
            return 1
    fi
    # NO `|| true`: this status IS the contract. `|| true` made the helper
    # report success when the kill FAILED — EPERM, or the target exiting
    # between the identity check and the signal — and a caller that then
    # `wait`s blocks until the target exits on its own (20 s for the probe,
    # never for RouDi). A kill to an unreaped ZOMBIE still succeeds, so the
    # ordinary already-dead case still returns 0 and the wait reaps at once;
    # only a genuine failure reports one.
    # THREE-WAY, because a failed `kill` has two causes the callers act
    # OPPOSITELY on, and returning 2 for both is wrong in each direction:
    #
    #   ESRCH — it EXITED between the identity check above and this signal.
    #           That race is normal (the identity check is not atomic with
    #           the kill), the daemon is gone, and the caller should clear
    #           its pid quietly. Reporting it as a failure warns "it is
    #           LEAKED" about a process that shut down by itself, and keeps
    #           ownership of nothing.
    #   EPERM — it is STILL OURS and refused the signal. Still running,
    #           still ours, and the caller must keep tracking it.
    #
    # `kill` in bash exposes no errno, so ask the question directly: after a
    # failed signal, does the pid still carry OUR identity? Gone means it
    # exited; still ours means the signal was refused.
    kill "-$sig" "$pid" 2>/dev/null && return 0
    [ "$(pid_identity "$pid")" = "$want_id" ] || return 1
    return 2
}

# The router, by identity. `comm` alone is not enough: the blanket
# `pkill -x rmw_zenohd` this replaces was name-scoped and so could never
# hit an unrelated process, but a raw pid recorded minutes ago can be
# recycled onto ANOTHER rmw_zenohd — the caller's — which passes a comm
# check and fails a start-time one.
kill_router_pid() {
    kill_owned_pid "$1" "${2:-}" rmw_zenohd KILL
}

source_ros() {
    # `set -u` after the source would OVERWRITE $?, so a setup script that
    # failed would return 0 and this gate would proceed under a partially
    # initialised overlay — and this script's verdict decides whether a
    # cell is recorded at all. Same rule as run_bench.sh::source_ros: assert
    # the OUTCOME (the overlay took) rather than a status this script does not control.
    set +u
    # shellcheck disable=SC1091,SC1090
    source "$1"
    local rc=$?
    set -u
    if [ "$rc" -ne 0 ]; then
        if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
            echo "verify_shm.sh: sourcing $1 failed (rc=$rc) and left" >&2
            echo "  AMENT_PREFIX_PATH unset — refusing to verify SHM" >&2
            echo "  engagement under a partially initialised ROS environment" >&2
            return "$rc"
        fi
        echo "verify_shm.sh: WARNING — sourcing $1 returned rc=$rc, but the" >&2
        echo "  overlay took (AMENT_PREFIX_PATH is set); continuing" >&2
    fi
    return 0
}

# In-container the Dockerfile exports ROS_SETUP_BASH; the fallback
# covers standalone `docker run ... bash` debugging.
source_ros "${ROS_SETUP_BASH:-/opt/ros/${ROS_DISTRO_NAME:-jazzy}/setup.bash}" || exit 2

CHECK_ROUDI_DELTA=0
CHECK_FASTDDS_DELTA=0
CHECK_ZENOH_LOG=0
CHECK_IOX2_DELTA=0
case "$RMW" in
    cyclonedds)
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file://$CONFIG_DIR/cyclonedds_${MODE}.xml"
        # NO log-marker grep: the apt CycloneDDS builds print no iceoryx
        # marker to any stream at any Tracing verbosity (measured on
        # jazzy 0.10.5, 2026-08-12; the May campaign found the same on
        # humble and downgraded its check to informational). Engagement
        # is verified STRUCTURALLY instead, against iox-roudi's debug
        # log delta — see the header for the two-condition pass rule
        # (registration proves plugin load; a DDS_CYCLONE
        # Publisher/SubscriberPort creation proves per-endpoint
        # engagement — registration alone must NOT pass). Runtime-name
        # note: 0.10.5 (humble/jazzy) registers as "iceoryx_rt_<pid>_*";
        # CycloneDDS 11.x (lyrical) loads the in-tree psmx_iox plugin —
        # the deprecated <SharedMemory> XML is converted by
        # convert_deprecated_sharedmemory() into the identical plugin
        # instance — and registers as "CycloneDDS-iox_psmx-<16-hex>".
        # The greps carry no runtime name, so both match. Requires
        # run_bench.sh's iox-roudi to run at `-l debug` with its log at
        # $IOX_ROUDI_LOG.
        CHECK_ROUDI_DELTA=1
        ;;
    fastdds)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        if [ "$MODE" = "zc" ]; then
            # DataSharing lane: the
            # rmw_fastrtps README recipe — data_sharing AUTOMATIC
            # default profiles + RMW_FASTRTPS_USE_QOS_FROM_XML=1
            # (without the env var rmw_fastrtps forces
            # data_sharing().off() and the XML QoS is ignored).
            export FASTRTPS_DEFAULT_PROFILES_FILE="$CONFIG_DIR/fastdds_zc.xml"
            export FASTDDS_DEFAULT_PROFILES_FILE="$CONFIG_DIR/fastdds_zc.xml"
            export RMW_FASTRTPS_USE_QOS_FROM_XML=1
        else
            # CER_BENCH_FASTDDS_PROFILE lets the caller verify the SAME
            # profile the measured cell will use. run_bench.sh switches to
            # configs/fastdds_shm_16mb.xml for the 16 MB payload, and a
            # one-time preflight against fastdds_shm.xml would leave that
            # profile's 256 MiB segment unverified — a profile-specific
            # failure would then reach a recorded, mislabeled cell.
            profile="${CER_BENCH_FASTDDS_PROFILE:-$CONFIG_DIR/fastdds_${MODE}.xml}"
            if [ ! -f "$profile" ]; then
                echo "verify_shm.sh: profile not found: $profile" >&2
                exit 2
            fi
            export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
            export FASTDDS_DEFAULT_PROFILES_FILE="$profile"
            echo "verify_shm.sh: fastdds profile = $(basename "$profile")"
            # Stock lane: a stray inherited value must not silently
            # turn it into the tuned one.
            unset RMW_FASTRTPS_USE_QOS_FROM_XML
        fi
        # NO log-marker grep: the apt Fast DDS builds print no SHM
        # string to any stream at default verbosity (Info logging is
        # compiled out of release builds — measured 2026-08-12 on
        # 2.6.11/2.14.6/3.6.x: probe logs carry zero SHM/shared
        # strings). Engagement is verified STRUCTURALLY against the
        # /dev/shm delta of a held-open probe — see the header and the
        # CHECK_FASTDDS_DELTA block below.
        CHECK_FASTDDS_DELTA=1
        ;;
    zenoh)
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp
        # The SHIPPED rmw_zenoh session config
        # (loaded when ZENOH_SESSION_CONFIG_URI is unset) + the
        # README-blessed key override — mirrors run_bench.sh exactly
        # (mode="init" rationale documented there: eager SHM init at
        # session open is what makes the probe's segment-allocation
        # marker exist, and it surfaces a memlock-class provider
        # failure HERE instead of as a mid-cell silent TCP fallback).
        unset ZENOH_SESSION_CONFIG_URI
        if [ "$MODE" = "shm" ]; then
            # json5 quoting on the mode value is LOAD-BEARING: the
            # override grammar parses each value as json5, so an
            # UNQUOTED init is REJECTED — zenohc logs ERROR "Failed to
            # insert value 'init' for key 'transport/shared_memory/
            # mode'", rmw_zenoh logs WARN "Ignore the invalid
            # configuration key-value pair", and the session silently
            # keeps the shipped mode (Lazy). Measured 2026-08-12 on
            # all three distro images: the unquoted string shipped by
            # the C3 rebase was rejected this way everywhere, which is
            # why the old gate never saw an SHM init line. enabled=true
            # needs no quotes (bare true IS valid json5). run_bench.sh
            # carries the SAME quoted string — C2: the probe must run
            # the exact config the cells run.
            export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/mode="init"'
        else
            export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'
        fi
        # Legacy env sanitation — these silently override the config's
        # SHM keys at rmw_zenoh 0.10.5.
        unset ZENOH_SHM_ALLOC_SIZE ZENOH_SHM_MESSAGE_SIZE_THRESHOLD
        # Both engagement markers are DEBUG-level (measured on all
        # three distros): the SHM link negotiation line comes from the
        # zenoh_transport crate and the segment-allocation lines from
        # zenoh_shm — an info-only filter shows NEITHER, which is half
        # of why the old gate failed. The leading bare `error`
        # directive keeps error-level lines from EVERY zenoh crate
        # visible (the lazy-ShmProvider fallback error is also logged
        # from zenoh_transport).
        export RUST_LOG='error,zenoh=debug,zenoh_shm=debug,zenoh_transport=debug'
        CHECK_ZENOH_LOG=1
        ;;
    cerulion)
        export RMW_IMPLEMENTATION=rmw_cerulion
        # No profiles file, no config override, no daemon: iceoryx2 takes
        # its sizing from the type's own declarations and runs no broker.
        # The staged ament prefix that makes the name resolvable is
        # exported by run_bench.sh's ensure_rmw_cerulion, which runs
        # BEFORE this gate; invoked standalone, put the prefix on
        # AMENT_PREFIX_PATH yourself or the probe fails to load the rmw.
        #
        # STRUCTURAL /dev/shm delta, same oracle shape as fastdds (no log
        # grep): a held-open probe publisher must create iox2_* segments
        # that were not there before it started. Delta, not presence: a
        # SIGKILL'd node from an earlier size leaves its segments behind,
        # and stale state must never satisfy the gate.
        #
        # SCOPE, because it is narrower than the fastdds gate's:
        # there, the question is whether the data plane engaged SHM or
        # silently fell back to UDP, and the size floor separates OUR
        # tuned descriptor from a builtin default. Here there is no
        # fallback path to detect: if rmw_cerulion moved a frame at all
        # it moved it through iceoryx2. What this gate actually catches is
        # the failure it CAN catch: an rmw that loaded but whose transport
        # never instantiated (a wrong-distro .so, an unwritable /dev/shm,
        # an ABI-skewed build that returns errors before creating a
        # publisher), which would otherwise show up as an empty cell with
        # no explanation. It carries no size floor because there is no
        # second, untuned configuration to tell apart.
        CHECK_IOX2_DELTA=1
        ;;
    *)
        echo "verify_shm.sh: unknown RMW: $RMW" >&2
        exit 2
        ;;
esac

# Topic-name + publish-flag notes (jazzy+ compatibility):
#   - `/_verify_shm` not `/__verify_shm`: ROS 2's topic naming rules
#     reject repeated underscores on jazzy+ (Humble's validator was
#     looser).
#   - `-w 0`: `ros2 topic pub --once` defaults to waiting for ≥1
#     matching subscription before publishing (ros2cli PR #642). This
#     smoke check has no subscriber, so without `-w 0` the publish
#     always times out — masquerading as a transport failure.
#   - QOS_ARGS: the probe publishes under the cell's CER_BENCH_QOS
#     so a QoS-gated SHM eligibility failure (e.g. a be1-ineligible
#     iceoryx exchange on CycloneDDS 0.10.5) is caught PER LANE instead
#     of hiding behind ros2cli's default (RELIABLE) QoS.
VERIFY_TOPIC="/_verify_shm"
PUB_COMMON_ARGS=("-w" "0" "${QOS_ARGS[@]}" "$VERIFY_TOPIC" "std_msgs/msg/Empty" "{}")
PUB_ARGS=("--once" "${PUB_COMMON_ARGS[@]}")

# rmw_zenoh requires a router (gossip-via-router discovery). The
# bench's run_bench.sh starts rmw_zenohd before the cells; verify_shm
# runs BEFORE that, so we need our own short-lived router here.
ZENOHD_PID=""
# The pid of the rmw_zenohd WE started (generally NOT ZENOHD_PID, which is
# the `ros2 run` Python wrapper). Recorded as soon as the router is
# IDENTIFIABLE — before readiness, so a router that never accepts is still
# torn down — see start_zenohd_if_needed and stop_zenohd.
ZENOHD_ROUTER_PID=""
# Start-time identities: a recorded pid outlives the process it named, and
# every teardown below is a claim about a PROCESS we started.
ZENOHD_ID=""
ZENOHD_ROUTER_ID=""
router_accepting() {
    # The shipped listen endpoint is tcp/[::]:7447; a successful
    # open-close of the port says SOMETHING is listening (sends no bytes —
    # zenoh drops the empty session attempt harmlessly).
    #
    # IDENTITY WARNING: this is readiness evidence for a router whose
    # identity is already established, never evidence that the listener IS
    # rmw_zenohd. Pair it with zenohd_live_pid.
    # `timeout 1` for the same reason its twin in run_bench.sh has one: a
    # DROP-filtered port makes the connect wait out the SYN timeout, and
    # inside the 60-iteration wait below that turns a documented "15 s"
    # ceiling into minutes.
    timeout 1 bash -c 'exec 3<>/dev/tcp/127.0.0.1/7447' 2>/dev/null
}
# PID of a LIVE (non-defunct) rmw_zenohd, or empty. `pgrep -x` alone also
# matches DEFUNCT processes — measured: a zombie router leaked by an
# unreaping container init made the old short-circuit skip the start, and
# the gate then ran against a corpse.
zenohd_live_pid() {
    ps -eo pid=,stat=,comm= |
        awk '$3 == "rmw_zenohd" && $2 !~ /Z/ { print $1; exit }'
}

start_zenohd_if_needed() {
    if [ "$RMW" != "zenoh" ]; then return; fi
    # Identity BEFORE the port: if an accepting 7447 returned straight
    # away, ANY listener — a stray dev server, a leftover Eclipse
    # zenohd, an SSH forward — would be accepted as "the router is up" and the
    # probe would then run router-less, failing as an SHM-engagement problem
    # rather than as the port collision it is.
    if [ -n "$(zenohd_live_pid)" ]; then
        : # a real router exists (ours or the caller's); wait for it below
    elif router_accepting; then
        echo "verify_shm.sh: TCP 127.0.0.1:7447 is held, but no live rmw_zenohd" >&2
        echo "  process exists — the listener is NOT a zenoh router, so this" >&2
        echo "  cell cannot be verified. Identify it (\`ss -ltnp sport = :7447\`" >&2
        echo "  or \`lsof -iTCP:7447 -sTCP:LISTEN\`) and stop it." >&2
        exit 2
    else
        if ! ros2 pkg executables rmw_zenoh_cpp 2>/dev/null | grep -q rmw_zenohd; then
            echo "verify_shm.sh: rmw_zenohd not available — skipping router start" >&2
            return
        fi
        ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null 2>&1 &
        ZENOHD_PID=$!
        ZENOHD_ID="$(pid_identity "$ZENOHD_PID")"
    fi
    # Wait until the router ACCEPTS, not a blind sleep: the router can
    # take >2 s to bind, and a `--once -w 0` probe exits before
    # zenoh's 1 s connect retry can land — measured 2026-08-12 as a
    # reproducible negotiated=0 false failure with the old `sleep 2`.
    # This also covers the run_bench.sh flow, where the router PROCESS
    # exists (the live-process short-circuit above) but may not be
    # accepting yet.
    for _ in $(seq 1 60); do
        # Record the router we started as soon as it is IDENTIFIABLE, not
        # when it becomes ACCEPTING. Only the branch above that SPAWNED one
        # can record (ZENOHD_PID non-empty): it proved no live rmw_zenohd
        # existed, so whatever is live now is ours. When we ADOPTED an
        # existing router this stays empty and teardown touches nothing —
        # run_bench.sh's router must survive this script.
        #
        # Recording only after readiness would be a bug: a router that starts and
        # never accepts (a bind refused, a broken overlay, a 15 s timeout)
        # would never be recorded, so stop_zenohd would reach only the wrapper and
        # its DIRECT children. `ros2 run` can fork the router deeper, and
        # this loop's whole purpose is the case where the router is slow —
        # exactly when it is most likely to be forked-but-not-accepting at
        # the moment we give up. The leak is not self-limiting: the next
        # cell finds the stray, calls it EXTERNAL, and permanently disarms
        # teardown for a process this script created.
        if [ -n "$ZENOHD_PID" ] && [ -z "$ZENOHD_ROUTER_PID" ]; then
            ZENOHD_ROUTER_PID="$(zenohd_live_pid)"
            ZENOHD_ROUTER_ID="$(pid_identity "$ZENOHD_ROUTER_PID")"
        fi
        if router_accepting; then
            return
        fi
        sleep 0.25
    done
    # Last look before giving up — the router may have forked in the final
    # slice, and this exit path leads straight to teardown.
    if [ -n "$ZENOHD_PID" ] && [ -z "$ZENOHD_ROUTER_PID" ]; then
        ZENOHD_ROUTER_PID="$(zenohd_live_pid)"
        ZENOHD_ROUTER_ID="$(pid_identity "$ZENOHD_ROUTER_PID")"
    fi
    echo "verify_shm.sh: WARNING — no zenoh router accepting on 127.0.0.1:7447 after 15s;" >&2
    echo "  the probe will run router-less and the SHM check will fail with diagnostics" >&2
}
stop_zenohd() {
    # Per attempt, never a global — same rule as the RouDi teardown flag.
    local launcher_refused=0
    # Only kill the router if WE started it — run_bench.sh's router must
    # survive this script. ZENOHD_PID is non-empty only when no external
    # router existed AT OUR START, which is not the same as "no external
    # router exists now": ours can die mid-script and a same-named one
    # appear, and a blanket `pkill -x rmw_zenohd` would then kill a router
    # this script never owned. So the sweep is scoped to OUR launcher's children —
    # which is the case it exists for, since `ros2 run` is a Python
    # wrapper that can orphan the actual binary one level down — and
    # anything still alive afterwards is by definition not ours.
    if [ -n "$ZENOHD_PID" ]; then
        # Identity-gated, same rule as the router below: the wrapper can
        # exit early and its pid is as recyclable as any other — and this
        # one is a `ros2` command, so a recycled pid is plausibly the
        # caller's unrelated ros2 process.
        if [ "$(pid_identity "$ZENOHD_PID")" = "$ZENOHD_ID" ]; then
            pkill -KILL -P "$ZENOHD_PID" -x rmw_zenohd 2>/dev/null || true
            # Through the same primitive as every other signal here, so the
            # launcher gets the same three-way answer. Waiting only
            # after a successful kill is right, but treating EVERY failure as
            # genuine is wrong: the identity check above is not atomic with
            # the signal, so the ordinary case of the launcher exiting in
            # that window would be reported as "could not signal" — and clearing
            # the pid regardless is the opposite of what the one
            # real failure needs.
            kill_owned_pid "$ZENOHD_PID" "$ZENOHD_ID" "" TERM
            case "$?" in
                0)  wait "$ZENOHD_PID" 2>/dev/null || true ;;
                2)  # Ours, still alive, and it refused the signal. Keeping
                    # the pid and identity is the point: cleared, an OWNED
                    # launcher becomes untracked and nothing can retry it.
                    launcher_refused=1
                    echo "warning: the ros2-run launcher (pid $ZENOHD_PID)" >&2
                    echo "  refused the signal and is STILL RUNNING; keeping" >&2
                    echo "  its pid so teardown can retry. The router it" >&2
                    echo "  forked is torn down by pid below." >&2
                    ;;
                *)  ;;  # 1 = it exited in the check-to-signal window: quiet
            esac
        fi
        if [ "$launcher_refused" != "1" ]; then
            ZENOHD_PID=""
            ZENOHD_ID=""
        fi
    fi
    # The router itself, BY PID. `-P` alone reaches only a child one level
    # down and only if it has already been forked, so on its own it leaks a
    # router WE started in two windows: this trap firing before the wrapper
    # forked it, and any deeper launcher wrapper. That leak is not
    # self-limiting — run_bench.sh's start_zenohd would then adopt the
    # stray as external and never tear it down. Naming the pid restores the
    # coverage the blanket `pkill -x` had, without its reach.
    if [ -n "$ZENOHD_ROUTER_PID" ]; then
        kill_router_pid "$ZENOHD_ROUTER_PID" "$ZENOHD_ROUTER_ID"
        # rc 2 = it IS our router and the signal FAILED. That leak is not
        # self-limiting: the next start_zenohd finds the stray, calls it
        # EXTERNAL, and permanently disarms teardown for a process this run
        # created. rc 1 (not ours / already gone) is the ordinary case and
        # stays quiet.
        if [ "$?" = "2" ]; then
            echo "warning: could not signal the rmw_zenohd this run started" >&2
            echo "  (pid $ZENOHD_ROUTER_PID) — it is LEAKED. A later run will" >&2
            echo "  adopt it as external and never tear it down; kill it by hand." >&2
            # KEEP the pid, the identity and the ownership flag. Clearing
            # them after the warning would be
            # narrating the leak and then causing it, because ownership is
            # what arms every retry. Held, the per-mode restart's next
            # stop_zenohd tries again; dropped, the stray is permanently
            # nobody's, which is exactly what the message above says goes
            # wrong.
        else
            ZENOHD_ROUTER_PID=""
            ZENOHD_ROUTER_ID=""
        fi
    fi
}
start_zenohd_if_needed

# In no_shm mode we just want a successful publish — no SHM marker.
if [ "$MODE" = "no_shm" ]; then
    if timeout 5 ros2 topic pub "${PUB_ARGS[@]}" >/dev/null 2>&1; then
        echo "verify_shm.sh: $RMW $MODE — published successfully (SHM not expected)"
        stop_zenohd
        exit 0
    else
        echo "verify_shm.sh: $RMW $MODE — publish failed" >&2
        stop_zenohd
        exit 1
    fi
fi

# SHM mode: capture stderr+stdout, then check the marker.
LOG=$(mktemp)
SHM_BEFORE=""
PROBE_PID=""
# Start-time identity for the probe — UNIFORMITY with the daemon ownership
# contract, not a fixed bug. Stated precisely because the daemons' rationale
# does NOT transfer: the probe is a `$!` child of this shell and every wait
# here is targeted, so an unreaped probe stays a zombie and the kernel
# cannot recycle its pid. The identity therefore guards nothing today; it
# means the file has ONE rule for signalling a recorded pid instead of two,
# and it stays correct if the probe ever gains a bare `wait` or a reaping
# subshell. `timeout` is the recorded process: the probe launches as
# `timeout 20 ros2 topic pub ...`, so $! is timeout's pid, not the ros2
# wrapper's.
PROBE_ID=""
cleanup_probe() {
    if [ -n "$PROBE_PID" ]; then
        # Identity-gated like every other signal in these two scripts. This
        # one runs from an EXIT trap, so the gap between the probe exiting
        # and the trap firing is unbounded — `timeout 20` can reap the probe
        # long before the script ends, and a raw SIGTERM to a recycled pid
        # would hit whatever took it.
        # Wait ONLY if we signalled. An unconditional wait after a refused
        # signal blocks until `timeout 20` fires on its own — up to 20 s
        # added to every EXIT path, which the raw `kill -TERM` this
        # replaced could not do.
        kill_owned_pid "$PROBE_PID" "$PROBE_ID" timeout TERM
        case "$?" in
            0)  wait "$PROBE_PID" 2>/dev/null || true ;;
            2)  # OURS and un-killable. Bounded here in a way the daemons
                # are not — the probe is `timeout 20 ros2 topic pub`, so it
                # self-terminates — but it holds a Fast DDS participant and
                # its /dev/shm segments meanwhile, and the caller's next
                # verify would read them in its delta. Say so rather than
                # exiting silently.
                echo "warning: could not signal this script's probe" >&2
                echo "  (pid $PROBE_PID) — it is LEFT RUNNING; it exits on" >&2
                echo "  its own within its 20s timeout, but a verify started" >&2
                echo "  before then may see its /dev/shm segments." >&2
                ;;
            *)  ;;
        esac
        PROBE_PID=""
        PROBE_ID=""
    fi
}
trap 'cleanup_probe; rm -f "$LOG" "$SHM_BEFORE"; stop_zenohd' EXIT

# Structural /dev/shm-delta oracle. Serves fastdds (mode shm AND mode
# zc) and cerulion (mode shm): ONE probe-and-poll body, because the
# question is identical in shape: did THIS probe create the segments its
# transport is supposed to create? Only the evidence classes and the pass
# rule differ, and each verdict below says which lane it speaks for.
#
# The evidence files exist only while a participant HOLDS them (Fast DDS
# 2.x unlinks them at graceful destruction), so `--once` (which
# publishes and exits) can never be checked: hold a 1 Hz publisher open
# and poll the /dev/shm DELTA against a pre-probe snapshot, then reap the
# probe. Delta, not presence: Fast DDS 3.x (lyrical) does NOT unlink its
# segments on process exit (measured), and neither does a SIGKILL'd
# iceoryx2 node, so a stale segment from a prior probe or bench node must
# never satisfy the gate.
if [ "$CHECK_FASTDDS_DELTA" = "1" ] || [ "$CHECK_IOX2_DELTA" = "1" ]; then
    SHM_BEFORE=$(mktemp)
    # Status checked: an empty baseline is not a neutral one. `comm -13`
    # against an empty file emits EVERY current entry, so a failed
    # snapshot turns stale segments from an earlier cell into this probe's
    # evidence — and the hygiene sweep below would then `rm -f` all of
    # them, including a live external RouDi's mempools.
    if ! ls -1 /dev/shm | sort > "$SHM_BEFORE"; then
        echo "verify_shm.sh: $RMW $MODE — cannot snapshot /dev/shm; refusing" >&2
        echo "  to verify (an empty baseline would read every stale segment as" >&2
        echo "  this probe's evidence)." >&2
        exit 2
    fi
    timeout 20 ros2 topic pub -r 1 "${PUB_COMMON_ARGS[@]}" >"$LOG" 2>&1 &
    PROBE_PID=$!
    PROBE_ID="$(pid_identity "$PROBE_PID")"
    if [ -z "$PROBE_ID" ]; then
        # `pid_identity` returns empty for TWO different facts — the
        # process is gone, and `ps` could not answer — and the poll below
        # would have to pick one meaning for both. Refuse instead of
        # guessing: an unreadable identity makes the liveness verdict wrong
        # in whichever direction it is not.
        echo "verify_shm.sh: cannot read the probe's start identity" >&2
        echo "  (ps -p $PROBE_PID returned nothing) — refusing to verify" >&2
        echo "  rather than poll a liveness check that cannot be trusted." >&2
        # Same rule as the teardown paths, with the failure SAID.
        # Waiting only after a successful signal is right; saying nothing
        # when the signal fails while clearing
        # PROBE_PID anyway is wrong — on a refusal the caller would be told cleanup had
        # completed while the `timeout 20` probe kept its Fast DDS
        # participant and its /dev/shm segments for the rest of its window,
        # which a verify started in that window reads in its own delta.
        # Three-way like every other signal here: gone, refused, signalled.
        kill_owned_pid "$PROBE_PID" "$PROBE_ID" "" TERM
        case "$?" in
            0)  wait "$PROBE_PID" 2>/dev/null || true ;;
            2)  echo "verify_shm.sh: the probe (pid $PROBE_PID) refused the" >&2
                echo "  signal and is STILL RUNNING. It exits on its own" >&2
                echo "  within its 20s timeout, but until then it holds a" >&2
                echo "  Fast DDS participant and its /dev/shm segments — a" >&2
                echo "  verify started before then will see them in its" >&2
                echo "  delta. Kill it by hand: kill $PROBE_PID" >&2
                ;;
            *)  ;;  # 1 = already gone: nothing to report
        esac
        PROBE_PID=""
        exit 2
    fi

    # Poll up to 12 s for the mode's evidence classes in the delta:
    #   segment  = fast(rtps|dds)_<hex> data segment ≥ 1 MB. The size
    #              floor proves OUR tuned descriptor (segment_size
    #              64 MiB in configs/fastdds_shm.xml + fastdds_zc.xml;
    #              measured 67,158,560 B on 2.6 / 70,255,136 B on
    #              2.14 + 3.6) instantiated — a builtin-default SHM
    #              segment is ~549,408 B on all three, so a silently
    #              unloaded profiles file cannot pass.
    #   port     = fast(rtps|dds)_port<N> — an SHM locator listener.
    #   ds       = fast_datasharing_* — a DataSharing writer-history
    #              segment (created at writer creation, no subscriber
    #              needed — measured on 2.14 and 3.6).
    SEG_EVIDENCE=""
    PORT_EVIDENCE=""
    DS_EVIDENCE=""
    IOX2_EVIDENCE=""
    DELTA_FILES=""
    PROBE_DIED=0
    PROBE_RC=0
    for _ in $(seq 1 24); do
        sleep 0.5
        # A probe that DIES is a broken lane, not a slow one — stop
        # polling immediately and report its exit code (measured: on
        # humble, RMW_FASTRTPS_USE_QOS_FROM_XML=1 — the zc recipe's
        # own env var — fails node creation with EITHER profiles XML
        # and ros2cli then segfaults at teardown, rc=139; without the
        # early exit each verify attempt burned the full poll window
        # against a corpse and misattributed the failure to missing
        # DataSharing evidence).
        # `kill -0` stays the LIVENESS fact; identity is only the recycle
        # discriminator, and only when it can be read. Making identity the
        # primary test inverted this poll in BOTH directions whenever
        # `pid_identity` came back empty: a live probe read as died (then
        # `wait` blocked to the end of its 20 s window and reported rc=124
        # as an abnormal death), and a reaped one read as alive (burning
        # the whole 12 s evidence window against a corpse). An empty answer
        # now means "cannot tell", which changes nothing.
        _probe_now_id="$(pid_identity "$PROBE_PID")"
        if ! kill -0 "$PROBE_PID" 2>/dev/null ||
           { [ -n "$_probe_now_id" ] &&
             [ "$_probe_now_id" != "$PROBE_ID" ]; }; then
            wait "$PROBE_PID" 2>/dev/null
            PROBE_RC=$?
            PROBE_PID=""
            PROBE_DIED=1
        fi
        DELTA_FILES=$(ls -1 /dev/shm 2>/dev/null | sort | comm -13 "$SHM_BEFORE" -)
        SEG_EVIDENCE=""; PORT_EVIDENCE=""; DS_EVIDENCE=""; IOX2_EVIDENCE=""
        while IFS= read -r f; do
            [ -z "$f" ] && continue
            case "$f" in
                iox2*)
                    # No size floor, unlike the fastdds segment class:
                    # iceoryx2 sizes its pool from the type, so there is
                    # no second untuned configuration a floor would tell
                    # apart. Any iox2 segment the probe created is proof
                    # the transport instantiated.
                    IOX2_EVIDENCE="$f"
                    ;;
                fast_datasharing_*)
                    DS_EVIDENCE="$f"
                    ;;
                fastrtps_port*|fastdds_port*)
                    PORT_EVIDENCE="$f"
                    ;;
                fastrtps_*|fastdds_*)
                    sz=$(stat -c%s "/dev/shm/$f" 2>/dev/null || echo 0)
                    if [ "$sz" -ge 1000000 ]; then
                        SEG_EVIDENCE="$f ($sz B)"
                    fi
                    ;;
            esac
        done <<< "$DELTA_FILES"
        if [ "$PROBE_DIED" = "1" ]; then break; fi
        if [ "$CHECK_IOX2_DELTA" = "1" ]; then
            if [ -n "$IOX2_EVIDENCE" ]; then break; fi
            continue
        fi
        if [ "$MODE" = "zc" ] && [ -n "$DS_EVIDENCE" ]; then break; fi
        if [ "$MODE" = "shm" ] && [ -n "$SEG_EVIDENCE" ] && [ -n "$PORT_EVIDENCE" ]; then break; fi
    done

    cleanup_probe
    # Probe hygiene: remove exactly the files the probe created. 2.x
    # already unlinked them at the graceful SIGTERM exit; 3.x leaves
    # them behind (measured on lyrical) and run_bench.sh's clean_shm
    # sweep predates the 3.x fastrtps_→fastdds_ rename, so without
    # this the probe's 64 MiB segment would linger into the
    # measurement runs.
    while IFS= read -r f; do
        [ -n "$f" ] && rm -f "/dev/shm/$f" 2>/dev/null
    done <<< "$DELTA_FILES"

    # A dead probe fails the gate REGARDLESS of whatever /dev/shm
    # evidence it left behind: a participant that crashed after
    # instantiating its transport (the measured humble-zc shape) is
    # not a runnable lane, and 2.x unlinks evidence at graceful death
    # anyway. rc 124 cannot appear here (timeout 20 outlives the 12 s
    # poll window), so any death is abnormal.
    if [ "$PROBE_DIED" = "1" ]; then
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — probe process DIED (rc=$PROBE_RC) during the evidence window" >&2
        echo "  (the publisher could not stay up under this lane's config — the cell cannot" >&2
        echo "   run. Known case: on humble, RMW_FASTRTPS_USE_QOS_FROM_XML=1 [the zc recipe]" >&2
        echo "   fails rmw node creation with either profiles XML and ros2cli segfaults at" >&2
        echo "   teardown, rc=139.)" >&2
        echo "/dev/shm delta at death:" >&2
        echo "$DELTA_FILES" | sed 's/^/  /' >&2
        echo "probe log:" >&2
        sed 's/^/  /' "$LOG" | head -15 >&2
        exit 1
    fi

    if [ "$CHECK_IOX2_DELTA" = "1" ]; then
        if [ -n "$IOX2_EVIDENCE" ]; then
            echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS - iceoryx2 transport confirmed (segment in the /dev/shm delta: $IOX2_EVIDENCE)"
            exit 0
        fi
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS - probe created NO iox2* segment" >&2
        echo "  (an rmw_cerulion publisher instantiates its iceoryx2 segment at creation, so" >&2
        echo "   an empty delta means the transport never came up: the .so may have loaded and" >&2
        echo "   then refused [wrong distro / ABI skew - check the probe log for an rmw_init or" >&2
        echo "   era-guard error], /dev/shm may be too small or unwritable, or the rmw was never" >&2
        echo "   loaded at all [is the staged prefix on AMENT_PREFIX_PATH?].)" >&2
        echo "/dev/shm delta:" >&2
        echo "$DELTA_FILES" | sed 's/^/  /' >&2
        echo "probe log:" >&2
        sed 's/^/  /' "$LOG" | head -20 >&2
        exit 1
    fi

    if [ "$MODE" = "zc" ]; then
        if [ -n "$DS_EVIDENCE" ]; then
            echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — DataSharing engagement confirmed (writer-history segment in the /dev/shm delta: $DS_EVIDENCE)"
            if [ -n "$SEG_EVIDENCE" ]; then
                echo "verify_shm.sh: $RMW $MODE — SHM transport also live (segment: $SEG_EVIDENCE)"
            fi
            exit 0
        fi
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — probe created NO fast_datasharing_* writer segment" >&2
        echo "  (a data_sharing AUTOMATIC writer creates its shared history segment at writer" >&2
        echo "   creation — measured on Fast DDS 2.14 and 3.6. Its absence means DataSharing" >&2
        echo "   did NOT engage: check RMW_FASTRTPS_USE_QOS_FROM_XML=1 and configs/fastdds_zc.xml," >&2
        echo "   or the QoS lane [qos=$CER_BENCH_QOS] disqualified data-sharing on this build.)" >&2
        echo "/dev/shm delta:" >&2
        echo "$DELTA_FILES" | sed 's/^/  /' >&2
        echo "probe log:" >&2
        sed 's/^/  /' "$LOG" | head -10 >&2
        exit 1
    fi

    if [ -n "$SEG_EVIDENCE" ] && [ -n "$PORT_EVIDENCE" ]; then
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — SHM engagement confirmed (tuned segment: $SEG_EVIDENCE; port file: $PORT_EVIDENCE)"
        exit 0
    fi
    echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — SHM transport evidence absent from the /dev/shm delta" >&2
    echo "  (needed BOTH a fast(rtps|dds)_* data segment ≥ 1 MB — proves the tuned SHM" >&2
    echo "   descriptor in the profiles XML instantiated — AND a fast(rtps|dds)_port* file" >&2
    echo "   [an SHM locator listener]. Missing:$([ -z "$SEG_EVIDENCE" ] && printf " segment")$([ -z "$PORT_EVIDENCE" ] && printf " port") — the probe's data" >&2
    echo "   plane would ride UDP under an 'shm' label. Was FASTDDS_DEFAULT_PROFILES_FILE" >&2
    echo "   honored? Is /dev/shm big enough for the 64 MiB segment [--shm-size]?)" >&2
    echo "/dev/shm delta:" >&2
    echo "$DELTA_FILES" | sed 's/^/  /' >&2
    echo "probe log:" >&2
    sed 's/^/  /' "$LOG" | head -10 >&2
    exit 1
fi

# Roudi-delta oracle (cyclonedds): snapshot roudi's log BEFORE the probe
# so only lines the probe itself caused count as evidence.
ROUDI_SNAPSHOT=0
if [ "$CHECK_ROUDI_DELTA" = "1" ]; then
    if [ -z "${IOX_ROUDI_LOG:-}" ] || [ ! -f "${IOX_ROUDI_LOG:-}" ]; then
        echo "verify_shm.sh: $RMW $MODE — IOX_ROUDI_LOG is not set or missing." >&2
        echo "  cyclonedds SHM engagement is verified via iox-roudi's debug log; run this" >&2
        echo "  through run_bench.sh (which starts iox-roudi -l debug and exports the path)," >&2
        echo "  or start iox-roudi -l debug yourself and export IOX_ROUDI_LOG." >&2
        exit 1
    fi
    # The status is checked, and this is not defensive padding: a failed
    # read leaves ROUDI_SNAPSHOT set-but-EMPTY, which destroys the `0`
    # initialiser above without tripping `set -u`; `$((SNAP + 1))` is then
    # 1, so `tail -n +1` hands the delta oracle the ENTIRE log. Every
    # earlier cell's registration and DDS_CYCLONE port lines satisfy both
    # pass conditions, and the gate prints "SHM engagement confirmed" for a
    # probe that engaged nothing. The `-f` test above does not cover it:
    # exists-but-unreadable is exactly the externally-owned-RouDi path this
    # script supports.
    if ! ROUDI_SNAPSHOT=$(wc -l < "$IOX_ROUDI_LOG"); then
        echo "verify_shm.sh: $RMW $MODE — cannot read $IOX_ROUDI_LOG to snapshot" >&2
        echo "  it; refusing to verify (an unread snapshot would make the delta" >&2
        echo "  oracle read the whole log as this probe's evidence)." >&2
        exit 2
    fi
fi

timeout 10 ros2 topic pub "${PUB_ARGS[@]}" >"$LOG" 2>&1
RC=$?
# Timeout (124) is acceptable — we've already captured the discovery
# log lines we need.
if [ "$RC" -ne 0 ] && [ "$RC" -ne 124 ]; then
    echo "verify_shm.sh: $RMW $MODE — ros2 topic pub failed (rc=$RC)" >&2
    cat "$LOG" >&2
    exit 1
fi

if [ "$CHECK_ROUDI_DELTA" = "1" ]; then
    # Give roudi a beat to flush the registration lines.
    sleep 0.5
    DELTA=$(tail -n +"$((ROUDI_SNAPSHOT + 1))" "$IOX_ROUDI_LOG")
    # Two-condition pass — BOTH required:
    #   1. registration        = the iceoryx plugin/bridge loaded
    #   2. a DDS_CYCLONE port  = an ENDPOINT actually qualified for
    #      iceoryx exchange under THIS QoS lane ($CER_BENCH_QOS)
    # Registration happens at participant init regardless of endpoint
    # QoS/type eligibility, so a registration-OR-port pass would let a
    # "shm"-labeled cell ride UDP whenever the lane's QoS disqualifies
    # the exchange (0.10.5 docs say RELIABLE is required while the code
    # gates only check reliability PRESENCE — code-vs-docs unresolved,
    # so it must be measured per lane, and either outcome here is a
    # valid finding).
    REGISTERED=0
    PORT_CREATED=0
    echo "$DELTA" | grep -q "Registered new application" && REGISTERED=1
    echo "$DELTA" | grep -q -E "Created new (Publisher|Subscriber)Port.*DDS_CYCLONE" && PORT_CREATED=1
    if [ "$REGISTERED" = "1" ] && [ "$PORT_CREATED" = "1" ]; then
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — SHM engagement confirmed (probe registered with iox-roudi AND created a DDS_CYCLONE port)"
        exit 0
    elif [ "$REGISTERED" = "1" ]; then
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — probe REGISTERED with iox-roudi but created NO DDS_CYCLONE Publisher/SubscriberPort" >&2
        echo "  (the iceoryx plugin loaded, but the probe endpoint did NOT qualify for iceoryx" >&2
        echo "   exchange — its data would ride UDP under an 'shm' label. Most likely the" >&2
        echo "   QoS lane disqualifies SHM on this CycloneDDS version [qos=$CER_BENCH_QOS]," >&2
        echo "   or roudi's port-creation line format drifted — inspect the delta below and" >&2
        echo "   the roudi log at \$IOX_ROUDI_LOG before relabeling or overriding.)" >&2
        echo "roudi log delta:" >&2
        echo "$DELTA" | sed 's/^/  /' | head -20 >&2
        echo "probe log:" >&2
        sed 's/^/  /' "$LOG" | head -10 >&2
        exit 1
    else
        echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — probe publish caused NO iox-roudi registration" >&2
        echo "  (a CycloneDDS participant with <SharedMemory> enabled registers with roudi;" >&2
        echo "   silence means SHM fell back to UDP — is iox-roudi running? did it start" >&2
        echo "   before the probe?)" >&2
        echo "roudi log delta:" >&2
        echo "$DELTA" | sed 's/^/  /' | head -10 >&2
        echo "probe log:" >&2
        sed 's/^/  /' "$LOG" | head -10 >&2
        exit 1
    fi
fi

# zenoh log oracle (rebuilt 2026-08-12 after the C3 rebase shipped a
# marker that never appears — see the header). Every check reads the
# PROBE's own log, so the evidence is attributed to the probe process
# even though the router shares /dev/shm.
if [ "$CHECK_ZENOH_LOG" != "1" ]; then
    echo "verify_shm.sh: internal error — rmw '$RMW' fell through every check discipline" >&2
    exit 2
fi

# Negative 1: the ZENOH_CONFIG_OVERRIDE pair was REJECTED. zenoh's
# override grammar parses each value as json5; a rejected pair logs one
# zenohc ERROR ("Failed to insert value ...") + one rmw_zenoh WARN
# ("Ignore the invalid configuration key-value pair") and the session
# silently runs the SHIPPED config — the exact silent-mislabel class
# this gate exists for (measured live: the unquoted mode=init shipped
# by the C3 rebase was rejected this way on all three distros, leaving
# mode: Lazy). Gated on its own so a future zenoh that changes the
# override grammar fails LOUDLY instead of measuring a default-config
# cell under a tuned label.
if grep -q -E "Failed to insert value|Ignore the invalid configuration" "$LOG"; then
    echo "verify_shm.sh: $RMW $MODE — ZENOH_CONFIG_OVERRIDE was (partly) REJECTED by the session" >&2
    echo "  (the probe log carries zenoh's config-insert error / rmw_zenoh's ignore warning;" >&2
    echo "   the session is running the shipped config, not the labeled one. Check the" >&2
    echo "   override's json5 quoting — string values need embedded quotes, e.g." >&2
    echo "   transport/shared_memory/mode=\"init\".)" >&2
    echo "log (config lines):" >&2
    grep -E "Failed to insert value|Ignore the invalid configuration" "$LOG" | sed 's/^/  /' | head -4 >&2
    exit 1
fi

# Negative 2: zenoh 1.8's transport-
# optimization provider logs ONE error on creation failure (e.g. mlock
# over RLIMIT_MEMLOCK) and then SILENTLY rides TCP — an "shm" row would
# measure loopback TCP with only this line as evidence. The positive
# markers below could even still match (init lines preceding the
# failure), so the fallback error must fail the gate on its own.
if grep -q "Error creating lazy ShmProvider" "$LOG"; then
    echo "verify_shm.sh: $RMW $MODE — zenoh SHM provider creation FAILED (silent-TCP-fallback signature)" >&2
    echo "  ('Error creating lazy ShmProvider' in the probe log — check RLIMIT_MEMLOCK:" >&2
    echo "   each process mlocks its own pool AND every mapped peer pool; docker needs" >&2
    echo "   --ulimit memlock=-1. Refusing the 'shm' label for a TCP data plane.)" >&2
    echo "log:" >&2
    sed 's/^/  /' "$LOG" >&2
    exit 1
fi

# Positive — BOTH required (measured both directions on humble 0.1.9 /
# jazzy 0.2.9 / lyrical 0.10.5, 2026-08-12):
#   negotiated = the probe↔router link NEGOTIATED SHM at transport
#                establishment (zenoh_transport, DEBUG). The no_shm
#                direction prints the same line with "shm: None", and
#                a disabled peer degrades the link to None even when
#                the probe has SHM on — so this is link-level
#                engagement evidence, not config echo.
#   allocated  = the probe process actually created SHM segments
#                (zenoh_shm::posix_shm, DEBUG; watchdog segment + the
#                mode="init" pool — 0 lines in the no_shm direction).
ZENOH_NEGOTIATED=0
ZENOH_ALLOCATED=0
grep -q "shm: Some(TransportShmConfig" "$LOG" && ZENOH_NEGOTIATED=1
grep -q "Created SHM segment" "$LOG" && ZENOH_ALLOCATED=1
if [ "$ZENOH_NEGOTIATED" = "1" ] && [ "$ZENOH_ALLOCATED" = "1" ]; then
    echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — SHM engagement confirmed (probe↔router link negotiated SHM AND the probe allocated SHM segments)"
    exit 0
fi
echo "verify_shm.sh: $RMW $MODE qos=$CER_BENCH_QOS — zenoh SHM evidence incomplete (negotiated=$ZENOH_NEGOTIATED allocated=$ZENOH_ALLOCATED)" >&2
if [ "$ZENOH_NEGOTIATED" = "0" ]; then
    echo "  no 'shm: Some(TransportShmConfig' on a transport-establishment line — the link" >&2
    echo "  to the router did NOT negotiate SHM (is the router running with the same" >&2
    echo "  SHM-enabled override? a disabled peer degrades the link to shm: None)." >&2
fi
if [ "$ZENOH_ALLOCATED" = "0" ]; then
    echo "  no 'Created SHM segment' line from zenoh_shm — the probe allocated no SHM" >&2
    echo "  segments (did the mode=\"init\" override apply? is RUST_LOG carrying" >&2
    echo "  zenoh_shm=debug?)." >&2
fi
echo "log:" >&2
sed 's/^/  /' "$LOG" | head -30 >&2
exit 1
