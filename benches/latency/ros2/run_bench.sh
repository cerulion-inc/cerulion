#!/usr/bin/env bash
# run_bench.sh — per-cell driver for the benches/latency ROS 2
# comparison harness. Runs INSIDE the docker image built
# from docker/Dockerfile; bench.py runs one container per cell.
#
# NOT A SUPPORTED ENTRY POINT. Running this
# script directly is a DIAGNOSTIC gesture, not a way to produce numbers.
# The supported entry point is `benches/latency/bench.py`, which owns the
# things this script cannot check for itself:
#   - the `CERULION` provenance + freshness refusal, so a campaign can
#     never be measured with a binary from another tree or an older one;
#   - the run manifest, the pacing variant and the payload schedule that
#     make the artifacts comparable;
#   - ONE CONTAINER PER CELL — never `--pid host` — which is what
#     confines the blanket bench-binary backstop this script
#     falls back on (it asserts that property before using it;
#     see `reap_bench_binaries`).
# Those policies live in ONE place on purpose: a second copy of them in
# shell would be a second thing to keep true, and the two would drift.
# Invoked directly, this script still refuses what it can see (its own env
# contract) and says so where it cannot — it does not silently substitute
# a weaker rule.
#
# Cell axes:
#   RMW       {cyclonedds, fastdds, zenoh, cerulion}
#             cerulion = rmw_cerulion, ROS 2 over the Cerulion
#             zero-copy transport. It was REMOVED from this suite once
#             and RE-ADMITTED for one reason:
#             the README's native round-trip chart wants an
#             rmw_cerulion series measured by the SAME harness as its
#             stock and composed ROS 2 lines, and a number produced by
#             any other harness cannot be drawn on that axis.
#             The .so is NOT in the image: it is built from the
#             bind-mounted repo at run time, against the live distro
#             headers (see ensure_rmw_cerulion).
#
#             READINESS RULE, lane specific (METHODOLOGY section 19).
#             Before the measured window opens,
#             the latency node waits for the chain to be up. On every
#             other RMW that wait is three conditions, and the third is
#             the ROS graph query count_subscribers on the ping topic.
#             rmw_cerulion answers graph queries from a registry holding
#             only the calling process's own endpoints, and this harness
#             runs ping, pong and latency as three processes, so that
#             query reads zero forever while the two matched endpoint
#             counts both read one, and no cerulion cell can ever start.
#             Under this RMW only, read from the environment once at
#             startup, the third condition is replaced by proof out of
#             data: keep the two matched conditions, then send bounded
#             probe kicks and start measuring once the first echo comes
#             back. Probe kicks and their echoes are warm up. They are
#             counted in the kicks_sent receipt because they were really
#             sent, and they never reach a sample, a bin file or a
#             percentile. Every ready line ends with a readiness field
#             reading matched+graph or matched+probe, so a row always
#             says which rule gated it.
#
#             ECHO RULE, all RMWs (METHODOLOGY section 20).
#             A pong loan branch that calls
#             borrow_loaned_message between taking the ping and
#             publishing the echo breaks it. A borrow is not a pointer handout on
#             every RMW: on rmw_cerulion it runs the typesupport init
#             over the loaned payload, which for this bench's fixed byte
#             array Pod message is a write the size of the payload, so
#             that write would sit inside the timed round trip and grow with
#             the message. The pong node borrows the NEXT reply right after
#             publishing the current one, and the first one before
#             signalling ready, so the only work between the take and the
#             publish is the eight byte stamp copy, the same rule the
#             ping side follows. The pong ready line carries
#             echo_rule, prefetched_loan or preallocated_copy, so a cell
#             log always says which discipline produced its numbers.
#   SHM mode  {shm, no_shm, zc}
#                               no_shm is jazzy-only
#                               (enforced by bench.py).
#                               zc is the
#                               FastDDS DataSharing true-zero-copy lane
#                               — fastdds ONLY, rejected loudly for
#                               every other RMW. It runs the
#                               vendor-recommended recipe from the
#                               rmw_fastrtps README (configs/
#                               fastdds_zc.xml + RMW_FASTRTPS_USE_QOS_
#                               FROM_XML=1); the plain `shm` lane stays
#                               the stock-defaults cell (rmw_fastrtps
#                               forces data_sharing().off() there —
#                               out-of-box ROS 2 behavior, labeled as
#                               such). zc is opt-in via SHM_MODE=zc
#                               (bench.py enumerates the cells); the
#                               manual WITH_SHM=1 sweep stays
#                               {shm, no_shm}.
#   recv      {rclcpp, loan}  — loan = rcl_take_loaned_message lane
#   qos       {be1, rel10, stock} — CER_BENCH_QOS, plumbed to the C++
#                               (stock = rmw_qos_profile_default,
#                               RELIABLE/VOLATILE/KEEP_LAST(10) — the
#                               lanes below only)
#   chrt      {off, on}       — SCHED_FIFO -f 80
#   payload   10 sizes, 64 B → 16 MB
#
# USAGE-PATTERN LANES (memo.md — two
# pseudo-rmw values riding the RMWS axis; bench.py enumerates them as
# their own cells, never crossed with the matrix axes):
#   RMWS=stock     the ZERO-CONFIG default (memo §3): what `ros2 run`
#                  gives you — rmw_fastrtps_cpp, NO profiles XML, NO
#                  transport env, DataSharing off, plain publish +
#                  typed-callback receive, qos=stock. Requires
#                  SHM_MODE=stock, RECV_PATH=rclcpp, CER_BENCH_QOS=
#                  stock. NO verify_shm gate — the lane claims no
#                  transport-engagement label; it gets a provenance
#                  note per cell instead (_logs/<cell>_provenance.txt).
#                  Cell: {distro}_stock_rclcpp_chrt{N}.
#   RMWS=composed  the FASTEST-COMMON pattern (memo §2): one process,
#                  three manually-composed nodes on one single-threaded
#                  executor (composed_rtt_node), rclcpp intra-process
#                  comms toggled by the mode. Requires SHM_MODE ∈
#                  {ipcon, ipcoff} (→ CER_BENCH_IPC=on/off),
#                  RECV_PATH=rclcpp, CER_BENCH_QOS=stock. Same
#                  provenance-note treatment (with ipc=on, the data
#                  path bypasses the rmw entirely — there is no
#                  transport label to verify).
#                  Cell: {distro}_composed_ipc{on,off}_rclcpp_chrt{N}.
#
# Knobs (env; names identical to the old trees where they existed):
#   CER_BENCH_RAW_DUMP_DIR   REQUIRED — .bin + per-size node logs land here
#   CER_BENCH_RAW_NAME       REQUIRED — cell name; binaries append _<size>.bin
#   CER_BENCH_MSG            pod (default) | image — the TYPE-CLASS axis
#                            (METHODOLOGY § "The type-class axis"):
#                            pod = fixed-size Pod<N> (the incumbent
#                            cells, loanable); image = sensor_msgs/msg/
#                            Image with its unbounded data array resized
#                            to the sweep point (non-plain — every RMW
#                            is structurally forced into full-serialize
#                            + delivery-memcpy; the class hypothesis).
#                            image is rclcpp-recv, shm-mode, be1,
#                            matrix-rmw ONLY (bench.py enumerates
#                            exactly shm × rclcpp × be1): image×loan
#                            and image×zc are exit-77 STRUCTURAL skips
#                            (unbounded types cannot loan; DataSharing
#                            requires plain bounded types);
#                            image×{stock,composed}, image×no_shm and
#                            image×rel10 are exit 2 (no inventory names
#                            such a cell — an unenumerated measurement
#                            is refused, never quietly minted).
#   CER_BENCH_QOS            be1 (default) | rel10 | stock (the stock and
#                            composed lanes only — cross-checked against
#                            the rmw value in the sweep)
#   CER_BENCH_PACING         quiescent (default) | fixed100 | backtoback
#   CER_BENCH_TARGET_RATE_HZ per-payload rate override (quiescent only)
#   TARGET_SAMPLES / WARMUP  sample-count overrides (all payloads).
#                            TARGET_SAMPLES is MEASURED samples — the
#                            post-warmup count every .bin must contain
#                            (G1 contract: CER_BENCH_TARGET_SAMPLES means
#                            measured in EVERY component). Empty = the
#                            per-payload schedule (quiescent, exported
#                            as total − warmup) or the binaries'
#                            10000/1000 defaults (backtoback)
#   CER_BENCH_ALLOW_UNVERIFIED_SHM=1
#                            escape hatch: a verify_shm failure records
#                            the cell anyway, with a loud stderr warning
#                            and a _logs/<cell>_<size>_SHM_UNVERIFIED
#                            marker per size (default: HARD failure,
#                            exit 13)
#
#   EXIT CODES: 0 ok · 2 setup/env refusal · 11,12 (see sites) · 13 one or
#   more sizes produced no/short .bin — under fixed100 this is the
#   CANNOT-SUSTAIN signal bench.py ladders on · 14 a size's DELIVERY
#   receipts could not describe its samples (never laddered: a lower rate
#   fixes nothing) · 77 structural skip.
#   RMWS                     space-separated RMW list. DEFAULT = the three
#                            stock RMWs {cyclonedds, fastdds, zenoh},
#                            UNCHANGED by the cerulion re-admission, so a
#                            default sweep measures exactly what it did
#                            before. The CLOSED set also accepts
#                            `cerulion` and the two lanes {stock,
#                            composed}, each of which bench.py pins one
#                            at a time. An unknown name, or a value that
#                            word-splits to nothing, is refused (exit 2)
#                            before any cell.
#   CER_RMW_REPO             cerulion cells ONLY: the bind-mounted Rust
#                            repo the cdylib is built from. Default
#                            /work. REQUIRED to exist (exit 2): there is
#                            no prebuilt fallback, by design: a stale .so
#                            would be measured under this run's label.
#   CER_RMW_PREFIX           cerulion cells ONLY: where the staged ament
#                            prefix is written. Default
#                            /tmp/rmw_cerulion_prefix (container-local;
#                            the raw dump dir is host-mounted evidence).
#   SIZES                    space-separated payload sizes (default all 10).
#                            CLOSED set — the ten pinned payloads; anything
#                            else, or a value that word-splits to nothing,
#                            is refused (exit 2). Decimal byte counts only
#                            — a non-digit token is refused, at most 10
#                            digits (bash arithmetic wraps at 64 bits —
#                            checked before any arithmetic), 1..16777216
#                            (the pinned ceiling), a zero-padded one is
#                            read as decimal with a loud note. The pinned
#                            set holds for BOTH classes (pod: Pod<N> is a
#                            fixed generated set, not per-size codegen;
#                            image: the in-container dispatch validates
#                            the sweep point) — refused here, before ROS
#                            is sourced, instead of by the node.
#   SHM_MODE / CHRT_MODE     pin to a single shm / chrt value (bench.py sets
#                            these with WITH_SHM=0 / WITH_CHRT=0).
#                            SHM_MODE ∈ {shm, no_shm, zc} for the three
#                            real RMWs, plus {stock} for RMWS=stock and
#                            {ipcon, ipcoff} for RMWS=composed; CHRT_MODE ∈
#                            {on, off}. Anything else is refused (exit 2)
#                            rather than silently read as off.
#                            SHM_MODE=zc is valid for RMWS=fastdds only.
#   IOX_ROUDI_LOG            iox-roudi's -l debug log. Set + exported for a
#                            RouDi this script starts; REQUIRED (exit 2)
#                            when one is already running externally — the
#                            cyclonedds SHM oracle reads that log and only
#                            RouDi's owner can capture it.
#   CER_BENCH_DUMP_VERSIONS=1
#                            dump the middleware package versions
#                            (dpkg) once per cell to
#                            _logs/<cell>_versions.txt
#                            (the per-cell receipt naming
#                            the exact debs each row ran against)
#   WITH_SHM / WITH_CHRT     sweep switches for manual in-container use
#   RECV_PATH                rclcpp (default) | loan
#   BENCH_CELL_TIMEOUT_S     per-(size) wall ceiling, default 300.
#                            bench.py exports a schedule-scaled value
#                            (2x nominal + 60 s, floored at 300) — the
#                            tail-resolved 16 MiB window is ~205 s
#                            nominal, so a flat 300 s would be tight;
#                            manual multi-size runs should export a
#                            larger value for the top sizes
#   BENCH_CHRT_PRIO          SCHED_FIFO priority, default 80
#
# Quiescent schedule per payload. The tuple is (rate_hz, TOTAL
# iterations, warmup); the exported CER_BENCH_TARGET_SAMPLES is
# MEASURED = total − warmup — the count every .bin must contain (the
# G1 contract). MUST stay in lockstep in FOUR places (G2):
#   native/src/lib.rs::quiescent_schedule
#   ros2/run_bench.sh          (this file, run_one)
#   bench.py::quiescent_schedule
#   workspace/run_workspace.sh::schedule_for
#
#   64–1024 B   @1000 Hz (60000, 5000 → 55000 measured)
#   4096–16384  @ 500 Hz (30000, 2500 → 27500)
#   65536 B     @ 200 Hz (12000, 1000 → 11000)
#   262144 B    @ 100 Hz  (6000,  500 →  5500)
#   1 MiB       @  60 Hz  (3600,  300 →  3300)
#   4 MiB       @  30 Hz  (2100,  100 →  2000)  tail-resolved
#   16 MiB      @  10 Hz  (2050,   50 →  2000)  tail-resolved
#
# fixed100 schedule (same four-place
# lockstep): every size @ 100 Hz (2100, 100 → 2000 measured). The
# 100→50→20→<sensor floor> fallback ladder is bench.py's — it re-runs
# one container per rung with the rung in CER_BENCH_TARGET_RATE_HZ;
# this driver never ladders on its own (an unsustainable manual
# in-container fixed100 run fails loudly, exit 13).
#
# Exit-code contract (bench.py maps these; do not renumber):
#    0  every requested size produced a full .bin, smoke checks clean
#    2  setup / env error (bad QoS, bad recv path, missing prereq, ...)
#   11  the class label is contradicted OR UNVERIFIABLE. Contradicted:
#       pod cells fail on "Msg::is_plain: 0" (Pod<N> not trivially
#       copyable — the loaned / CDR-memcpy assumption is broken, bad
#       build); image cells fail on the INVERSE, "Msg::is_plain: 1" (a
#       cell labeled 'variable/unbounded' ran a plain type).
#       Unverifiable (only when the run itself reported success): the
#       class's own marker is ABSENT, or the log cannot be read — an
#       unverified label is not a verified one. Every node logs the
#       marker from its constructor, so a log without it LOST it
#       (truncated, rotated, a dropped stderr redirect, a suppressed
#       INFO level) or came from binaries older than is_plain_check.
#       check_structural_loan_skip also exits 11 on an unreadable log.
#   12  /dev/cpu_dma_latency is PRESENT in the container but the DMA
#       lock failed — p99 would be polluted by C-state exits under a
#       label that claims otherwise (device absent = soft-warn only)
#   13  one or more sizes produced no / short .bin (bootstrap timeout,
#       transport failure, sample-count mismatch), OR verify_shm.sh
#       failed for the cell's (rmw, shm) pair — the SHM-engagement
#       label would be unverified (override: CER_BENCH_ALLOW_
#       UNVERIFIED_SHM=1, which records + marks the cell instead)
#   77  structural skip: the loan lane on an RMW whose
#       rmw_take_loaned_message is a NO-OP stub — rmw_zenoh (every
#       released distro). No retry helps.
#
# Side daemons:
#   iox-roudi   required for cyclonedds + shm (started before that
#               batch, killed after).
#   rmw_zenohd  required for rmw_zenoh discovery.
#
# Apples-to-apples preserved:
#   - cpu_dma_lock acquired by every node binary at the top of main().
#   - chrt -f 80 wraps the binary directly (not via `ros2 run`, which
#     is a Python wrapper that would mask the prio change). The old
#     host-side sudo/chrt_wrap.sh fallback is GONE: this script runs
#     as root in the container; if chrt fails there the cell FAILS
#     LOUDLY instead of silently measuring without RT priority.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- required env (fail-fast; the binaries also refuse) ---------------

if [ -z "${CER_BENCH_RAW_DUMP_DIR:-}" ] || [ -z "${CER_BENCH_RAW_NAME:-}" ]; then
    echo "run_bench.sh: CER_BENCH_RAW_DUMP_DIR and CER_BENCH_RAW_NAME must both be set" >&2
    echo "  (bench.py sets them; for manual runs: docker run -v \$host_raw:/raw -e CER_BENCH_RAW_DUMP_DIR=/raw -e CER_BENCH_RAW_NAME=<cell> ...)" >&2
    exit 2
fi
if [ ! -d "$CER_BENCH_RAW_DUMP_DIR" ] || [ ! -w "$CER_BENCH_RAW_DUMP_DIR" ]; then
    echo "run_bench.sh: CER_BENCH_RAW_DUMP_DIR=$CER_BENCH_RAW_DUMP_DIR is not a writable directory" >&2
    exit 2
fi

# Per-cell diagnostic artifacts — delivery accounting (F1.4) and
# SHM-unverified markers (F1.3) — land under _logs/ inside the raw
# dump dir, beside the .bins the host mounted in.
LOGS_DIR="$CER_BENCH_RAW_DUMP_DIR/_logs"
if ! mkdir -p "$LOGS_DIR"; then
    echo "run_bench.sh: cannot create $LOGS_DIR" >&2
    exit 2
fi

# Per-cell package-version receipt: when
# bench.py sets CER_BENCH_DUMP_VERSIONS=1, record the middleware debs
# this cell actually ran against — apt can lag rosdistro, and the
# documented version claims (fastdds 3.6.2, cyclonedds 11.0.1, rmw_zenoh
# 0.10.5, ...) must be provable per cell, not assumed from the image
# tag. Once per cell (this script IS one cell); loud on failure, never
# silently absent.
if [ "${CER_BENCH_DUMP_VERSIONS:-0}" = "1" ]; then
    VERSIONS_FILE="$LOGS_DIR/${CER_BENCH_RAW_NAME}_versions.txt"
    if ! dpkg -l 2>/dev/null | grep -E 'ros-|cyclonedds|fastdds|fastrtps|zenoh|iceoryx' > "$VERSIONS_FILE" || [ ! -s "$VERSIONS_FILE" ]; then
        echo "run_bench.sh: WARNING — CER_BENCH_DUMP_VERSIONS=1 but the dpkg version dump came back empty (non-dpkg image?)" >&2
        echo "dpkg version dump FAILED or empty at $(date -u +%Y-%m-%dT%H:%M:%SZ) — no version evidence for this cell" > "$VERSIONS_FILE"
    fi
fi

# --- axis validation (loud, never a silent fallback) ------------------

CER_BENCH_QOS="${CER_BENCH_QOS:-be1}"
case "$CER_BENCH_QOS" in
    be1|rel10) ;;
    stock) ;;  # rmw_qos_profile_default — the stock/composed lanes only
               # (cross-checked against the rmw value in the sweep below)
    *)
        echo "run_bench.sh: CER_BENCH_QOS must be 'be1', 'rel10' or 'stock', got '$CER_BENCH_QOS'" >&2
        exit 2
        ;;
esac
export CER_BENCH_QOS

# --- C-state posture (METHODOLOGY §11) --------------------------------
# The C++ nodes construct CpuDmaLock UNCONDITIONALLY: they cap whenever
# /dev/cpu_dma_latency is openable and soft-warn when it is not. Under
# bench.py the stock posture is made true by OMISSION — no `--device`, so
# the file does not exist in the container and the nodes run uncapped. A
# DIRECT `bash run_bench.sh` in a container that DOES have the device gets
# no such protection: the run is capped while every label says stock,
# which is the mislabeled-figure hazard the knob exists to remove. The
# lane cannot un-acquire the cap from the shell, so refuse rather than
# guess. bench.py cannot trip this — under CER_BENCH_DMA_LOCK=0 it never
# passes the device, so no measured behaviour changes on that path.
CER_BENCH_DMA_LOCK="${CER_BENCH_DMA_LOCK:-1}"
case "$CER_BENCH_DMA_LOCK" in
    0)
        if [ -e /dev/cpu_dma_latency ]; then
            echo "run_bench.sh: CER_BENCH_DMA_LOCK=0 (stock posture) but" >&2
            echo "  /dev/cpu_dma_latency EXISTS here, and the C++ nodes open it" >&2
            echo "  unconditionally — this run would be CAPPED while labelled" >&2
            echo "  stock. Run without the device (bench.py's stock posture omits" >&2
            echo "  '--device /dev/cpu_dma_latency'), or set CER_BENCH_DMA_LOCK=1" >&2
            echo "  and label the run tuned." >&2
            exit 2
        fi
        echo "run_bench.sh: stock mode — C-state exits unmanaged, tails not comparable to capped runs"
        ;;
    1)
        # Tuned posture ASKS for the cap; the nodes print their own loud
        # note when the device is absent, so a silently-uncapped "tuned"
        # run is already visible in the cell log rather than only here.
        ;;
    *)
        echo "run_bench.sh: CER_BENCH_DMA_LOCK must be 0 or 1, got '$CER_BENCH_DMA_LOCK'" >&2
        exit 2
        ;;
esac
export CER_BENCH_DMA_LOCK

CER_BENCH_PACING="${CER_BENCH_PACING:-quiescent}"
case "$CER_BENCH_PACING" in
    quiescent|fixed100|backtoback) ;;
    *)
        echo "run_bench.sh: CER_BENCH_PACING must be 'quiescent', 'fixed100' or 'backtoback', got '$CER_BENCH_PACING'" >&2
        exit 2
        ;;
esac
export CER_BENCH_PACING

# TYPE-CLASS axis (CER_BENCH_MSG — see the header + METHODOLOGY.md
# § "The type-class axis"). Validated here so a typo'd class dies
# before any node runs (a cell measured under a mislabeled class is
# worse than a failed cell); exported so the C++ dispatch
# (msg_class_dispatch.hpp) reads the same validated value.
CER_BENCH_MSG="${CER_BENCH_MSG:-pod}"
case "$CER_BENCH_MSG" in
    pod|image) ;;
    *)
        echo "run_bench.sh: CER_BENCH_MSG must be 'pod' or 'image', got '$CER_BENCH_MSG'" >&2
        exit 2
        ;;
esac
# The cell NAME is what compile_csv and plot.py classify on — the class
# is read off the `_image_` token in the stem by plot.py, never from
# this variable, and compile_csv keys its rows on the same stem —
# so a run whose name and class disagree publishes one class's numbers
# under the other's label. bench.py always agrees (Ros2Cell.name carries
# the token iff msg == image); a hand-run can disagree, and the README
# documents hand-runs. Refuse a name carrying the OTHER class's PINNED
# grammar: under `pod`, the `_image_` token; under `image`, any of the
# four pinned POD stems. An ad-hoc name carrying none of them claims no
# class and is left alone — which is deliberately weaker than the
# workspace runner's rule (there, a name opening with the pinned prefix
# must BE the pinned grammar), because ROS 2 cell names have five shapes
# and no single opening prefix to anchor on. `*_shm_*` covers `_no_shm_`
# too (the substring is there), which is why no separate arm spells it;
# `_zc_`, `_stock_` and `_composed_` are the other three.
case "$CER_BENCH_MSG:$CER_BENCH_RAW_NAME" in
    image:*_image_*) ;;
    pod:*_image_*)
        echo "run_bench.sh: CER_BENCH_MSG=pod but CER_BENCH_RAW_NAME='$CER_BENCH_RAW_NAME' carries the image class token" >&2
        echo "  — the cell name is what the CSV and the plots classify on, so this run would publish" >&2
        echo "  pod numbers under a variable-class label. Drop '_image' from the name, or set CER_BENCH_MSG=image." >&2
        exit 2
        ;;
    image:*_shm_*|image:*_zc_*|image:*_stock_*|image:*_composed_*)
        echo "run_bench.sh: CER_BENCH_MSG=image but CER_BENCH_RAW_NAME='$CER_BENCH_RAW_NAME' is a pinned POD cell name" >&2
        echo "  (no '_image' token) — the cell name is what the CSV and the plots classify on, so this run" >&2
        echo "  would publish image numbers under a pod-class label. Use the image grammar:" >&2
        echo "  {distro}_{rmw}_shm_image_rclcpp_be1_chrt{N}." >&2
        exit 2
        ;;
esac
export CER_BENCH_MSG

RECV_PATH="${RECV_PATH:-rclcpp}"
case "$RECV_PATH" in
    rclcpp|loan) ;;
    rcl_loan_recv|intra_naive|intra_forward)
        echo "run_bench.sh: RECV_PATH='$RECV_PATH' is a retired value." >&2
        echo "  Use RECV_PATH=loan for the rcl_take_loaned_message lane;" >&2
        echo "  the intra_* lanes were dropped (intra-process axis deferred)." >&2
        exit 2
        ;;
    *)
        echo "run_bench.sh: RECV_PATH must be 'rclcpp' or 'loan', got '$RECV_PATH'" >&2
        exit 2
        ;;
esac

# The subscription-side loan gate must match the
# lane's claim. rcl gates SUBSCRIPTION loans OFF by default (upstream's
# own safety decision, rclcpp#2335 / rcl#1110, Nov 2023, backported to
# Humble): with the env unset, sub->can_loan_messages() reads FALSE on
# every RMW even where rcl_take_loaned_message (which bypasses the
# gate) takes loans just fine — so the loan lane's published capability
# lines (`rcl_take_loaned_recv=` / the RMW_LOAN_RECV_UNSUPPORTED
# sentinel) contradicted what the lane actually measured on
# rmw_fastrtps. Export =0 on the `loan` lane ONLY, so the capability
# probe and the take path agree. The `rclcpp` lane stays env-UNTOUCHED:
# sub-loans-off is upstream's shipped decision and that cell is the
# stock-defaults measurement — a stray inherited value is scrubbed
# loudly rather than silently changing what the cell measures.
# Structural skips are unaffected: rmw_zenoh hardcodes
# can_loan_messages=false at the rmw layer, so its exit-77 sentinel
# still fires with the env set.
if [ "$RECV_PATH" = "loan" ]; then
    export ROS_DISABLE_LOANED_MESSAGES=0
elif [ -n "${ROS_DISABLE_LOANED_MESSAGES:-}" ]; then
    echo "run_bench.sh: WARNING — stray ROS_DISABLE_LOANED_MESSAGES='${ROS_DISABLE_LOANED_MESSAGES}' in the environment;" >&2
    echo "  unsetting it: the rclcpp lane is the stock-defaults cell (sub-side loans off per rclcpp#2335 / rcl#1110)" >&2
    unset ROS_DISABLE_LOANED_MESSAGES
fi

TARGET_SAMPLES="${TARGET_SAMPLES:-}"
WARMUP="${WARMUP:-}"
WITH_CHRT="${WITH_CHRT:-1}"
WITH_SHM="${WITH_SHM:-1}"
# Every value the sweep's dispatcher below implements. The DEFAULT sweep
# is the three real RMWs; `stock` and `composed` are lanes bench.py pins
# one at a time (they claim no transport label — see the provenance note
# at the verify_shm gate), so they are valid but not swept by default.
RMWS_VALID=(cyclonedds fastdds zenoh cerulion stock composed)
RMWS_DEFAULT=(cyclonedds fastdds zenoh)
if [ -z "${RMWS:-}" ]; then
    RMWS_LIST=("${RMWS_DEFAULT[@]}")
else
    # Whitespace-separated string; word splitting is intended.
    # `set -f`: an unquoted expansion also GLOBS, so `RMWS='fast*'` in a
    # directory holding a matching file would expand to a name nobody
    # typed. Word splitting is intended; pathname expansion is not.
    case $- in *f*) _globwas=off ;; *) _globwas=on ;; esac
    set -f
    # shellcheck disable=SC2128,SC2206
    RMWS_LIST=($RMWS)
    [ "$_globwas" = on ] && set +f
fi
# A whitespace-only RMWS word-splits to NOTHING: unrefused, the sweep runs zero
# RMWs, never sets ANY_SIZE_FAILED, and exits 0 with no measurement at all.
if [ "${#RMWS_LIST[@]}" -eq 0 ]; then
    echo "run_bench.sh: RMWS is empty after word splitting ('${RMWS:-}') — refusing a zero-RMW sweep" >&2
    echo "  valid: ${RMWS_VALID[*]}" >&2
    exit 2
fi
for _rmw in "${RMWS_LIST[@]}"; do
    _ok=0
    for _valid in "${RMWS_VALID[@]}"; do
        [ "$_rmw" = "$_valid" ] && _ok=1
    done
    if [ "$_ok" -ne 1 ]; then
        echo "run_bench.sh: unknown RMW '$_rmw' in RMWS — refusing before any cell runs" >&2
        echo "  valid: ${RMWS_VALID[*]}" >&2
        exit 2
    fi
done

# run_one treats every CHRT_MODE other than the literal `on` as off, so an
# unvalidated value records a NORMAL-priority run under whatever label was
# typed — a mislabeled cell, which is worse than a refusal.
if [ -n "${CHRT_MODE:-}" ]; then
    case "$CHRT_MODE" in
        on|off) ;;
        *)
            echo "run_bench.sh: CHRT_MODE must be 'on' or 'off', got '$CHRT_MODE'" >&2
            exit 2
            ;;
    esac
fi

SIZES_DEFAULT=(64 256 1024 4096 16384 65536 262144 1048576 4194304 16777216)
if [ -z "${SIZES:-}" ]; then
    SIZES=("${SIZES_DEFAULT[@]}")
else
    # SIZES env var is a whitespace-separated string; we want word
    # splitting here. Every token is validated as a DECIMAL integer and
    # canonicalized (same contract as run_workspace.sh): the per-size
    # schedule below is a string match that would route '0000064' to
    # the fallback tuple, the .bin names must carry the form bench.py
    # expects, and bash's $(( )) reads a leading zero as OCTAL. `10#`
    # pins base ten; a changed token is noted loudly.
    # Range contract, enforced BEFORE any arithmetic touches the token
    # (same as run_workspace.sh): bash integers are 64-bit and `10#`
    # WRAPS on overflow (18446744073709551680 reads as 64 — a 64-byte
    # cell under a 20-digit label), so the token LENGTH is bounded
    # first, then the surviving value is compared against the ceiling:
    # the pinned sweep's largest size, which the per-size schedule, the
    # iceoryx/FastDDS SHM segment profiles (16 MB needs the 256 MiB
    # profile) and the node prealloc are provisioned for.
    PAYLOAD_CEILING_BYTES=16777216
    MAX_SIZE_DIGITS=10
    # `set -f` for the same reason as the RMW list above: word splitting
    # is intended, pathname expansion is not (`SIZES='6*'` beside a file
    # named `64` would otherwise mint a size nobody typed).
    case $- in *f*) _globwas=off ;; *) _globwas=on ;; esac
    set -f
    # shellcheck disable=SC2128,SC2206
    SIZES_RAW=($SIZES)
    [ "$_globwas" = on ] && set +f
    SIZES=()
    for tok in "${SIZES_RAW[@]}"; do
        case "$tok" in
            ''|*[!0-9]*)
                echo "run_bench.sh: SIZES entry '$tok' is not a decimal integer (digits only — no 0x prefix, no exponent, no sign)" >&2
                exit 2
                ;;
        esac
        if [ "${#tok}" -gt "$MAX_SIZE_DIGITS" ]; then
            echo "run_bench.sh: SIZES entry '$tok' has too many digits (${#tok} > $MAX_SIZE_DIGITS) — bash arithmetic is 64-bit and wraps silently (18446744073709551680 would read as 64); the ceiling is $PAYLOAD_CEILING_BYTES bytes anyway" >&2
            exit 2
        fi
        size=$(( 10#$tok ))
        if [ "$size" -lt 1 ] || [ "$size" -gt "$PAYLOAD_CEILING_BYTES" ]; then
            echo "run_bench.sh: SIZES entry '$tok' (= $size) is outside 1..$PAYLOAD_CEILING_BYTES — the pinned sweep tops out there (schedule + SHM segment profiles are provisioned for it); a larger request is never a citable cell" >&2
            exit 2
        fi
        # Sweep-point realizability (the ROS 2 twin of run_workspace.sh's
        # pod multiple-of-4 rule, and STRICTER — and class-INDEPENDENT):
        # every ROS 2 cell takes exactly the ten pinned sizes. The pod
        # class is not generated per size — it is the FIXED set of
        # ros2_rtt_msgs Pod<N> types (Pod64 … Pod16777216, each exactly
        # N bytes: uint64 ts_ns + uint8[N-8]) dispatched at runtime by
        # pod_dispatch.hpp's switch over those ten; the image class has
        # ONE Image instantiation but msg_class_dispatch.hpp's
        # valid_sweep_size() rejects an off-sweep N just the same (the
        # matched-quantity rule pins image data to the sweep points, and
        # the SHM segment profiles + schedule are per pinned size). Both
        # in-container refusals fire only after ROS is sourced and the
        # daemons start — refuse HERE, in milliseconds, naming the set.
        case " ${SIZES_DEFAULT[*]} " in
            *" $size "*) ;;
            *)
                echo "run_bench.sh: SIZES entry '$tok' (= $size) is not a pinned sweep size" >&2
                echo "  — every ROS 2 cell takes exactly the pinned sizes (pod: Pod<N> is a FIXED set of generated types; image: the" >&2
                echo "  in-container dispatch validates the sweep point explicitly); valid: ${SIZES_DEFAULT[*]}" >&2
                exit 2
                ;;
        esac
        if [ "$size" != "$tok" ]; then
            echo "run_bench.sh: note — SIZES entry '$tok' read as decimal $size (leading zeros dropped; schedule lookup + .bin names use the canonical form)" >&2
        fi
        SIZES+=("$size")
    done
fi
# A whitespace-only SIZES splits to ZERO tokens: the per-token loop above
# never runs, so nothing is validated and the sweep would measure nothing
# and still exit 0. Membership in the pinned set is enforced per token
# during canonicalization above (one check, one message); this is the one
# condition that loop cannot see.
if [ "${#SIZES[@]}" -eq 0 ]; then
    echo "run_bench.sh: SIZES is set but splits to no tokens (empty or whitespace only)" >&2
    echo "  — refusing a zero-size sweep: it would measure nothing and still exit 0" >&2
    echo "  valid: ${SIZES_DEFAULT[*]}" >&2
    exit 2
fi

INSTALL_LIB="/bench/ros2_ws/install/ros2_rtt_bench/lib/ros2_rtt_bench"

if [ "$CER_BENCH_PACING" = "backtoback" ] && [ -n "${CER_BENCH_TARGET_RATE_HZ:-}" ]; then
    echo "run_bench.sh: note — CER_BENCH_TARGET_RATE_HZ is ignored under CER_BENCH_PACING=backtoback" >&2
fi

# Capture the CALLER's rate override ONCE. The old tree resolved the
# per-size schedule into CER_BENCH_TARGET_RATE_HZ with a
# self-referencing default (`${CER_BENCH_TARGET_RATE_HZ:-$rate}`), so
# in a multi-size invocation the FIRST size's schedule rate stuck for
# every later size (a 16 MB cell paced at the 64 B cell's 1000 Hz).
# Distinguishing "caller passed a rate" from "we resolved one for the
# previous size" fixes that while keeping the documented override
# precedence (an explicit caller rate applies to all payloads).
CALLER_RATE_HZ="${CER_BENCH_TARGET_RATE_HZ:-}"

# --- sweep axes --------------------------------------------------------
# CHRT/SHM mode resolution + the type-class gates are pure env parsing,
# so they run HERE — before ROS is sourced and before any daemon or node
# starts — and a mislabeled or unenumerated axis dies in milliseconds on
# any host (check_percentile_parity.py drives these gates through this
# script on a ROS-less host for exactly that reason).

CHRT_MODES=(off)
[ "$WITH_CHRT" = "1" ] && CHRT_MODES=(off on)
[ -n "${CHRT_MODE:-}" ] && CHRT_MODES=("$CHRT_MODE")

SHM_MODES=(no_shm)
[ "$WITH_SHM" = "1" ] && SHM_MODES=(shm no_shm)
# SHM_MODE pins a single mode (bench.py's per-cell path). `zc` — the
# FastDDS DataSharing lane — is reachable ONLY this way:
# the manual WITH_SHM sweep stays {shm, no_shm}. The lane modes —
# `stock` (RMWS=stock) and `ipcon`/`ipcoff` (RMWS=composed) — are also
# SHM_MODE-only values; the per-rmw sweep below cross-checks that a
# lane mode never rides a matrix rmw and vice versa.
if [ -n "${SHM_MODE:-}" ]; then
    case "$SHM_MODE" in
        shm|no_shm|zc|stock|ipcon|ipcoff) SHM_MODES=("$SHM_MODE") ;;
        *)
            echo "run_bench.sh: SHM_MODE must be one of 'shm', 'no_shm', 'zc', 'stock', 'ipcon', 'ipcoff', got '$SHM_MODE'" >&2
            exit 2
            ;;
    esac
fi

# TYPE-CLASS structural gates (CER_BENCH_MSG=image). These are the
# hypothesis the class exists to test, rendered as skip inventory:
# ROS 2's zero-copy lanes are structurally unavailable to unbounded
# types, so an image cell on one of those lanes is not a failed run —
# it is a lane that cannot exist. bench.py never enumerates these
# combinations; the gates catch manual in-container invocations.
if [ "$CER_BENCH_MSG" = "image" ]; then
    if [ "$RECV_PATH" = "loan" ]; then
        echo "SKIP (rc=77): image × loan is structurally unmeasurable — an" >&2
        echo "unbounded type is non-plain, so loan take never engages" >&2
        echo "(can_loan_messages gates on is_plain on every RMW that" >&2
        echo "implements loans). No retry helps; the image class's receive" >&2
        echo "cost lives in the rclcpp lane's delivery memcpy." >&2
        exit 77
    fi
    # RMWS_LIST, not the raw RMWS: the env var is optional (unset means
    # the default three), and this block was hoisted above ROS sourcing,
    # so `$RMWS` under `set -u` aborts the script with rc=1 before any
    # gate can fire. RMWS_LIST is always populated and already validated.
    case " ${RMWS_LIST[*]} " in
        *" stock "*|*" composed "*)
            echo "run_bench.sh: CER_BENCH_MSG=image cannot ride the usage lanes" >&2
            echo "  (RMWS=stock / RMWS=composed are pod-only by enumeration — a lane" >&2
            echo "  cell name carries no msg token, so an image lane cell would be" >&2
            echo "  an unlabeled axis)" >&2
            exit 2
            ;;
    esac
    # bench.py enumerates image cells on shm × rclcpp × be1 ONLY (the
    # README cell-name grammar: {distro}_{rmw}_shm_image_rclcpp_be1_
    # chrt{N}). no_shm and rel10 are MEASURABLE for an unbounded type —
    # nothing structural forbids them — but no inventory names such a
    # row and no plot expects it, so a run here would be an
    # unenumerated measurement under a name nothing can audit. Refused
    # as a setup error (exit 2, like the lanes), never quietly minted.
    # Widening the image class is an enumeration change first
    # (bench.py enumerate_ros2_cells + plot.py's _ROS2_IMG_RE + the
    # README grammar), not a relaxation of this gate.
    if [ "$CER_BENCH_QOS" != "be1" ]; then
        echo "run_bench.sh: CER_BENCH_MSG=image is enumerated for CER_BENCH_QOS=be1 only (got '$CER_BENCH_QOS')" >&2
        echo "  — an image × $CER_BENCH_QOS cell exists in no inventory (bench.py enumerate_ros2_cells: image = shm × rclcpp × be1);" >&2
        echo "  it would be an unenumerated measurement, so it is refused rather than minted under an unlisted name" >&2
        exit 2
    fi
    for m in "${SHM_MODES[@]}"; do
        case "$m" in
            zc)
                echo "SKIP (rc=77): image × zc is structurally unmeasurable — FastDDS" >&2
                echo "DataSharing requires plain, bounded types (an unbounded data" >&2
                echo "vector disqualifies the type at the DDS layer). The image class's" >&2
                echo "serialize + delivery-copy cost shows on the shm lane." >&2
                exit 77
                ;;
            no_shm)
                echo "run_bench.sh: CER_BENCH_MSG=image is enumerated for SHM_MODE=shm only (got shm modes [${SHM_MODES[*]}])" >&2
                echo "  — an image × no_shm cell exists in no inventory (bench.py enumerate_ros2_cells: image = shm × rclcpp × be1);" >&2
                echo "  it would be an unenumerated measurement, so it is refused rather than minted under an unlisted name." >&2
                echo "  Pin SHM_MODE=shm: the default WITH_SHM sweep {shm, no_shm} includes the refused mode." >&2
                exit 2
                ;;
        esac
    done
fi

# --- helpers ----------------------------------------------------------

source_ros() {
    # `set -u` after the source would OVERWRITE $? with its own status, so a
    # setup script that failed would return 0 and the driver would carry on
    # under a partially initialised overlay. Capture, restore, then report.
    set +u
    # shellcheck disable=SC1090,SC1091
    source "$1"
    local rc=$?
    set -u
    if [ "$rc" -ne 0 ]; then
        # `source` returns the status of the LAST command in the sourced
        # file, which this script does not control, so assert the OUTCOME rather than
        # the status alone: refuse only when the overlay demonstrably did
        # not take. Both /opt/ros/*/setup.bash and colcon's install/setup.bash
        # end in `unset` (status 0) today, so the warn arm should stay
        # theoretical — but a trailing non-zero must not abort a suite whose
        # environment is provably fine.
        if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
            echo "run_bench.sh: sourcing $1 failed (rc=$rc) and left" >&2
            echo "  AMENT_PREFIX_PATH unset — refusing to run under a" >&2
            echo "  partially initialised ROS environment" >&2
            return "$rc"
        fi
        echo "run_bench.sh: WARNING — sourcing $1 returned rc=$rc, but the" >&2
        echo "  overlay took (AMENT_PREFIX_PATH is set); continuing" >&2
    fi
    return 0
}

IOX_ROUDI_PID=""
ZENOHD_PID=""
# Start-time identities for the three pids this script signals. A pid is
# reusable and every ownership flag below is a claim about a PROCESS, so
# each recorded pid carries the identity that makes the claim checkable.
IOX_ROUDI_ID=""
ZENOHD_ID=""
ZENOHD_ROUTER_ID=""
# OWNERSHIP flags: 1 only when THIS invocation started the daemon. An
# externally started iox-roudi / rmw_zenohd is deliberately reused (see
# start_*), and killing it — or unlinking its shared-memory pools — would
# tear down a caller's router and break unrelated ROS 2 processes on this
# machine. Every teardown below is gated on these.
IOX_ROUDI_OWNED=0
ZENOHD_OWNED=0
# The pid of the rmw_zenohd WE started, captured after our readiness loop
# proved it up. `ros2 run` is a Python wrapper, so this is generally NOT
# $ZENOHD_PID (the wrapper) — and it is what lets every teardown name the
# router it may kill instead of sweeping every process called rmw_zenohd.
# A blanket sweep was only ever justified by a START-time invariant ("no
# external router existed when we started"), which says nothing about now:
# ours can die mid-run and a same-named one appear.
ZENOHD_ROUTER_PID=""

# All bench binaries by install path — used by cleanup + between-size
# reaping. No sudo: this script runs as root inside the container.
BENCH_BIN_PATTERN='ros2_rtt_bench/lib/ros2_rtt_bench/(ping_node|pong_node|latency_node|pong_node_rcl|latency_node_rcl|composed_rtt_node)'

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
# Return codes are THREE-WAY, because callers need to tell a benign
# refusal from a real failure:
#   0  we signalled it
#   1  not ours, or already gone — the ordinary teardown case
#   2  it IS ours and the signal FAILED — the only arm worth reporting
# BOTH RouDi callers in this file branch on it (the EXIT trap and
# stop_iox_roudi), and stop_iox_roudi additionally leans on that `wait` as
# the interlock before `rm -f /dev/shm/iceoryx_*`: reaching the unlink with
# a LIVE RouDi pulls its mempools out from under it. Do NOT restore
# `|| true` here — an unconditional 0 makes both `if`s always true and
# reinstates an unbounded wait on a daemon that never exits by itself.
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

# The same question for ANY pid this script recorded — a bench node or a
# daemon — asked as a liveness poll rather than a signal: is this pid
# still the process we started?
#
# Every poll in this file goes through here. A bare
# `kill -0` poll with a bare `kill -SIG` acting on it is the hazard
# `pid_identity` exists for. The bench nodes are a WORSE case than the
# daemons, not a better one: they are `&` children of THIS shell, so
# bash's SIGCHLD handler reaps them asynchronously, and the instant it
# does the kernel may hand the pid to somebody else. A signal ladder makes
# that window wide — the rung after a bounded grace is sent up to ~2 s
# after the last liveness poll, and when the grace ends BECAUSE the child
# exited, the next rung is aimed at a pid we have just watched disappear.
# The daemons are not better off for being long-lived: their pids are
# recorded at START and escalated at TEARDOWN, a whole cell later. This
# script runs as root in the container, so the stray signal is
# unrestricted either way.
#
# comm is not taken at all here, the same choice `ros2 run` gets above:
# the nodes are launched behind a `chrt`/`taskset` prefix, so their comm
# is the prefix tool on some paths and the binary on others. The start
# time still pins the identity, and the one caller that wants a comm gate
# as well (start_iox_roudi, whose daemon is unwrapped) adds it as its own
# conjunct rather than weakening this one.
node_still_ours() {
    [ -n "${1:-}" ] || return 1
    # An UNKNOWN identity must not silently switch the watchdog OFF. This
    # poll gates the cell timeout, so answering "not ours" for want of an
    # identity would end the wall-ceiling loop immediately and leave the
    # run blocking in `wait` on a hung node — trading a bounded timeout
    # for a hang, which is worse than the hazard this helper exists for.
    # A liveness poll has no side effect, so it falls back to the plain
    # existence test; every SIGNAL still goes through `kill_owned_pid`,
    # which refuses an empty want_id outright. The fallback can therefore
    # lengthen a reap (the graceful rungs become no-ops and the `pkill`
    # backstop does the work) and can never aim one at a stranger.
    if [ -z "${2:-}" ]; then
        kill -0 "$1" 2>/dev/null
        return
    fi
    [ "$(pid_identity "$1")" = "$2" ]
}

# The router, by identity. `comm` alone is not enough: the blanket
# `pkill -x rmw_zenohd` this replaces was name-scoped and so could never
# hit an unrelated process, but a raw pid recorded minutes ago can be
# recycled onto ANOTHER rmw_zenohd — the caller's — which passes a comm
# check and fails a start-time one.
kill_router_pid() {
    kill_owned_pid "$1" "${2:-}" rmw_zenohd KILL
}

# Did a reap rung actually END this pid? PURE arithmetic over the rung's
# return code, so the per-cell reap can hand `reap_bench_binaries` a
# verified answer instead of a claim.
#
#   $1 the rung's rc from `kill_owned_pid`: 0 signalled, 1 not ours or
#      already gone (the ordinary case), 2 ours and the signal FAILED
#   $2 the verdict so far (1 = every rung so far ended its pid)
#   $3/$4 the pid and its recorded identity
#
# rc 1 is ACCEPTED as reaped because it is overwhelmingly "already gone" —
# except in the one state this whole flag exists for: an empty identity,
# where `kill_owned_pid` returns 1 WITHOUT SIGNALLING, so rc 1 says
# nothing about whether the process ended.
#
# An empty identity therefore clears the verdict OUTRIGHT. Asking
# `kill -0 "$pid"` first and clearing it only if something
# is still alive there would be a PID-only liveness test in the middle
# of the identity-gated path, and a recycled pid answers it.
# The rule here is also the simpler one: with no identity nothing
# CAN be proven reaped, so nothing is claimed. The cost is one message
# being more cautious than it had to be on a cell whose child really did
# exit; the gain is that this function reads no pid it cannot name, and
# the only `kill -0` left in this file is `node_still_ours`'s documented
# fallback.
reap_verdict() {
    local rc="$1" sofar="$2" pid="${3:-}" want_id="${4:-}"
    [ "$sofar" = "1" ] || { printf '0\n'; return 0; }
    [ -n "$pid" ] || { printf '1\n'; return 0; }
    if [ "$rc" -gt 1 ] || [ -z "$want_id" ]; then
        printf '0\n'; return 0
    fi
    printf '1\n'
}

# ---------------------------------------------------------------------
# The blanket bench-binary backstop, and the one property that makes it
# safe.
#
# `pkill -KILL -f "$BENCH_BIN_PATTERN"` proves NO ownership: it signals
# every process whose command line matches the install-path regex, in
# whatever PID namespace this shell can see. Nothing about the pattern
# bounds that. What bounds it is a DEPLOYMENT property — bench.py runs
# ONE CONTAINER PER CELL, with `--network host` for the UDP lanes but
# never `--pid host` — so a separate run is a separate PID namespace and
# the only matching processes here are this cell's own.
#
# That property is not self-evident: run this
# harness on a host, or in a container given `--pid host`, and one cell's
# timeout reaps another's nodes mid-measurement, surfacing as an
# unexplained transport failure in the OTHER cell — the worst possible
# diagnostic. Direct invocation of
# this script is not a supported entry point, so the script ASSERTS the
# property rather than proving pid ownership for a pattern that cannot
# carry one.
#
# NOT DELETED, and not replaced by `pkill -P $$` either. Deleting it would
# convert a blunt reap into a hang in the degraded no-`ps` state, where
# `pid_identity` yields nothing and every `kill_owned_pid` rung refuses;
# and `-P $$` at a per-cell reap would kill this shell's OTHER children —
# iox-roudi and the zenohd launcher — mid-sweep. The captured node pids
# are reaped by identity first (see the ladders), and this is the last
# resort for anything they cannot name.
#
# PURE: the decision, given PID 1's argv (NUL-separated, rendered one
# argument per line) and this script's own file name. Split from the read
# so it can be driven with hand vectors on a host that has no /proc at
# all — check_percentile_parity.py does exactly that, and also drives the
# whole funnel with the read stubbed.
#
# THE QUESTION IS "AM I PID 1'S OWN TREE", and PID 1's command line is
# what answers it: docker/Dockerfile's CMD runs THIS script, so inside a
# per-cell container PID 1 is the shell that launched us and its argv
# names us. Under `--pid host`, on a bare host, or under `docker run …
# bash` (an unsupported entry point), PID 1 is somebody else's init or an
# interactive shell and does not.
#
# NOT /proc/1/sched. The obvious alternative is that file's parenthesised
# pid, on the theory that it is reported in the INITIAL namespace so a 1
# means "no namespace of my own". Two problems, and the second is fatal
# for a gate: the kernel prints it through the pid namespace of the
# READER'S /proc mount, so a container with its own /proc may legitimately
# print 1 and the backstop would decline in EVERY container — inert, plus
# a banner on every cell; and nothing in this repo can test which way a
# given kernel goes, so the claim would ship as reasoning. PID 1's argv is
# a contract this repository OWNS: the Dockerfile sets it, and
# check_percentile_parity.py pins the Dockerfile's CMD against the name
# looked for here, so a CMD change that would disable this gate fails the
# gate instead of silently switching it off.
backstop_confined_by() {
    local argv="${1:-}" self="${2:-}" line
    [ -n "$argv" ] && [ -n "$self" ] || return 1
    # One argument per line, so a path is matched whole rather than as a
    # substring of a longer word. The CMD spells the script inside a
    # `bash -lc` string, so the name is looked for anywhere in an
    # argument — but always with its leading `/`, so a log file called
    # `run_bench.sh.log` cannot answer for the script.
    while IFS= read -r line; do
        case "$line" in
            *"/$self"|*"/$self "*) return 0 ;;
            "$self"|"$self "*)     return 0 ;;
        esac
    done <<EOF_ARGV
$argv
EOF_ARGV
    return 1
}

# The read. PID 1's argv is NUL-separated; render it one argument per
# line. Empty when /proc is absent or unreadable (a non-Linux host, a
# hidepid mount), which the decision above treats as NOT established.
pid_one_cmdline() {
    tr '\0' '\n' < /proc/1/cmdline 2>/dev/null
}

# ONE funnel for the blanket sweep, so the gate cannot be copied without
# it. Both reap paths call this; nothing else in this file may spell the
# pattern sweep itself.
#
# $1 — 1 when the caller has just signalled every pid it recorded BY
# IDENTITY, so the refusal below may truthfully say this cell's own
# processes were reaped. Anything else (including absent) means that was
# NOT established, and the refusal says so instead of claiming it: with no
# `ps`, `pid_identity` yields nothing, `kill_owned_pid` refuses an empty
# identity before signalling, and every rung is a silent no-op — which is
# exactly the state in which somebody reads this message.
BACKSTOP_REFUSAL_REPORTED=0
reap_bench_binaries() {
    local identity_reaped="${1:-0}" argv prc
    argv=$(pid_one_cmdline)
    if backstop_confined_by "$argv" "$(basename -- "$0")"; then
        pkill -KILL -f "$BENCH_BIN_PATTERN" 2>/dev/null
        prc=$?
        # 0 = killed something, 1 = nothing matched (the ordinary case).
        # 2 = a usage/pattern error and 3 = fatal: the one reap this file
        # calls a backstop would then be INERT, which must not read as a
        # successful sweep.
        if [ "$prc" -gt 1 ]; then
            echo "run_bench.sh: the blanket bench-binary backstop could not" >&2
            echo "  RUN (pkill exit $prc) — its pattern is probably invalid." >&2
            echo "  Strays are NOT reaped: $BENCH_BIN_PATTERN" >&2
            return 1
        fi
        return 0
    fi
    # Once per run: this fires on every cell and every teardown, and the
    # operator needs the fact, not a hundred copies of it.
    if [ "$BACKSTOP_REFUSAL_REPORTED" = "0" ]; then
        BACKSTOP_REFUSAL_REPORTED=1
        echo "run_bench.sh: NOT running the blanket bench-binary backstop." >&2
        echo "  It kills by install-path pattern and proves no ownership;" >&2
        echo "  it is safe only because bench.py runs one container per" >&2
        echo "  cell (never --pid host), and that could not be established" >&2
        echo "  here." >&2
        # WHICH of the two reasons, because they call for opposite
        # actions. "PID 1 is not our entrypoint" means stop running the
        # harness this way. "PID 1's argv could not be read" means the
        # property is UNKNOWN, not refuted, and the likely cause is a host
        # with no /proc — which would decline the backstop on a desk run
        # that is otherwise fine. One line here is the difference between
        # diagnosing that in a grep and mistaking it for a reap that
        # quietly stopped working.
        if [ -z "$argv" ]; then
            echo "  PID 1 argv could not be READ (no /proc, or a hidepid" >&2
            echo "  mount) — the property is UNKNOWN, not refuted, and" >&2
            echo "  this refusal is the conservative answer." >&2
        else
            echo "  PID 1 is not this script: $(printf '%s' "$argv" | tr '\n' ' ')" >&2
            echo "  so this shell is not the init of its own process tree" >&2
            echo "  and a matching process may belong to something else." >&2
        fi
        if [ "$identity_reaped" = "1" ]; then
            echo "  This cell's own nodes WERE reaped by pid identity." >&2
        else
            echo "  This cell's own nodes were NOT reaped by identity" >&2
            echo "  either (no start-time identity was available, so every" >&2
            echo "  kill_owned_pid rung refused before signalling) —" >&2
            echo "  nothing ended them. Expect a hang in the following" >&2
            echo "  \`wait\`, and reap them by hand." >&2
        fi
        echo "  Strays, if any: pkill -KILL -f '$BENCH_BIN_PATTERN'" >&2
        echo "  (Running this script directly is NOT a supported entry" >&2
        echo "   point — use bench.py.)" >&2
    fi
    return 1
}

# One counter out of a DELIVERY receipt file. Three outcomes, kept apart
# because they mean different things: the role's line is ABSENT (empty),
# the line is there but the key will not parse ("?"), or the count.
receipt_count() {
    local file="$1" role="$2" key="$3" line grc val
    line=$(grep -m1 "DELIVERY role=$role " "$file" 2>/dev/null); grc=$?
    if [ "$grc" -gt 1 ]; then
        # grep could not READ the file. That is not "this role did not
        # report"; naming it as absence would assert a cause the code has
        # not established. "?" routes it to the unparseable arm, which says
        # only that the count could not be obtained.
        printf '?\n'
        return 0
    fi
    [ -n "$line" ] || return 0
    # WHOLE-TOKEN extraction, then a whole-VALUE validation. A prefix
    # regex (`$key=\([0-9][0-9]*\).*`) lets the trailing `.*` swallow
    # whatever follows the digits, so `published=12junk` parses as a clean
    # 12: a corrupted accounting file passes the coherence gate whenever its
    # truncated numbers happen to stay monotone, and rides out with a
    # published sample. (A leading `[[:space:]]` does already prevent a
    # SUFFIX key from answering — `retries_published=77` never matches key
    # `published` — so the trailing `.*` is the whole defect. Checked
    # rather than assumed: that expression, run against exactly that
    # line, produces nothing.)
    #
    # awk splits on whitespace, so `$i` IS one token; the key is compared
    # whole, and the value is everything after the FIRST `=`, never a
    # prefix of it. The `case` then requires the value to be entirely
    # digits, so any junk fails CLOSED to "?" instead of a plausible number.
    val=$(printf '%s\n' "$line" | awk -v k="$key" '
        {
            for (i = 1; i <= NF; i++) {
                n = index($i, "=")
                if (n > 0 && substr($i, 1, n - 1) == k) {
                    print substr($i, n + 1)
                    exit
                }
            }
        }')
    case "$val" in
        ''|*[!0-9]*) printf '?\n' ;;
        *)           printf '%s\n' "$val" ;;
    esac
}

cleanup() {
    rc=$?
    local launcher_refused=0
    # Its own per-attempt copy — see stop_iox_roudi. The trap is terminal,
    # but a latched global would have made this branch fire on a RouDi this
    # trap tore down cleanly.
    local roudi_teardown_failed=0
    # Ownership by PARENTHOOD, which needs no recorded identity: every
    # bench node and both daemons are `&` children of this shell. That is
    # why the funnel is told `1` here — unlike the per-cell reap, this
    # path really has ended its own processes before the sweep is asked.
    pkill -P $$ 2>/dev/null || true
    reap_bench_binaries 1
    # Both PIDs are set ONLY when this invocation started the daemon, so
    # killing them can never reach a caller's. The name-based sweep is
    # likewise gated on ownership (see stop_zenohd). Inlined rather than
    # calling stop_zenohd: this trap can fire before that function is
    # defined.
    if [ -n "$IOX_ROUDI_PID" ]; then
        # Identity-gated like the two zenoh pids — a recorded pid outlives
        # the process it named, and RouDi is unwrapped so `comm` is exact.
        # Wait only if we SIGNALLED: RouDi is a daemon that never exits on
        # its own, so an unconditional wait after a refused signal would
        # hang teardown forever if it were still our child (a `comm` the
        # gate did not recognise is the reachable way in).
        kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi TERM
        case "$?" in
            0)  # Signalled — and then BOUNDED, the same contract the
                # workspace sampler follows. The comment above says
                # RouDi "never exits on its own"; a plain `wait` after a
                # TERM it ignores or is wedged against therefore hangs
                # teardown for good, which is the failure the refused-signal
                # arm below avoids and this arm must avoid too.
                # BY IDENTITY, both the poll and the escalation. A bare
                # `kill -0` grace and a
                # bare `kill -KILL` on a pid recorded when RouDi STARTED
                # would send the one signal this file's own comment says must
                # never be sent blind, blind, at the end of every
                # teardown, and over a window of the whole cell rather
                # than a 2 s grace. The daemon is not an `&` child on
                # every path, and this script runs as root in the
                # container, so a recycled pid would take an unrestricted
                # SIGKILL. `node_still_ours` and `kill_owned_pid` are the
                # guard.
                _r_waited=0
                while [ "$_r_waited" -lt 100 ]; do
                    node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" || break
                    sleep 0.1
                    _r_waited=$((_r_waited + 1))
                done
                if node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID"; then
                    # The GUARD and the ACTOR must agree. `node_still_ours`
                    # falls back to a bare `kill -0` when the identity is
                    # empty, while `kill_owned_pid` REFUSES an empty
                    # identity before signalling — so announcing the
                    # SIGKILL before sending it printed "sending SIGKILL"
                    # on a path that sent nothing. Announce the OUTCOME.
                    # And no `|| true`: this helper's own header forbids
                    # it, because rc 2 ("it IS ours and the signal
                    # FAILED") must reach the arm below that KEEPS the
                    # /dev/shm pools — reaching `rm -f /dev/shm/iceoryx_*`
                    # under a live daemon is the corruption that arm
                    # exists to refuse.
                    kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi KILL
                    case "$?" in
                        0)  echo "warning: iox-roudi (pid $IOX_ROUDI_PID) did not" >&2
                            echo "  exit within 10s of SIGTERM; SIGKILL sent." >&2
                            ;;
                        2)  roudi_teardown_failed=1 ;;
                        *)  echo "warning: iox-roudi (pid $IOX_ROUDI_PID) is still" >&2
                            echo "  up after SIGTERM and could NOT be SIGKILLed by" >&2
                            echo "  identity (no start-time identity, or the pid is" >&2
                            echo "  no longer ours) — refusing to escalate blind." >&2
                            roudi_teardown_failed=1
                            ;;
                    esac
                fi
                # Reap only when teardown did not fail: `wait` on a daemon
                # this shell could not signal blocks for good, and RouDi
                # never exits by itself.
                if [ "$roudi_teardown_failed" != "1" ]; then
                    wait "$IOX_ROUDI_PID" 2>/dev/null || true
                fi
                ;;
            2)  # OURS, and the signal FAILED. An earlier fix gave the helper this
                # code and then let both callers read it like a benign
                # refusal — so a LIVE RouDi kept running while the caller
                # cleared its ownership and unlinked /dev/shm/iceoryx_*,
                # pulling the mempools out from under it. Keep ownership,
                # keep the pools, and say so; the run is over either way,
                # and a loud stray beats a silent corrupt one.
                roudi_teardown_failed=1
                echo "warning: could not signal the iox-roudi this run started" >&2
                echo "  (pid $IOX_ROUDI_PID) — it is LEFT RUNNING and its" >&2
                echo "  /dev/shm/iceoryx_* pools are LEFT IN PLACE (unlinking" >&2
                echo "  them under a live daemon corrupts it). Kill it by hand:" >&2
                echo "  kill $IOX_ROUDI_PID" >&2
                ;;
            *)  # 1 = not ours, or already gone. The ordinary case.
                ;;
        esac
        if [ "$roudi_teardown_failed" != "1" ]; then
            IOX_ROUDI_PID=""
            IOX_ROUDI_ID=""
        fi
    fi
    if [ -n "$ZENOHD_PID" ]; then
        # Identity-gated, same rule as the router: the launcher forks the
        # router and can exit early, and its pid is as recyclable as any
        # other — this one is a `ros2` Python wrapper, so a recycled pid is
        # very plausibly an unrelated ros2 command of the caller's.
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
    if [ "$ZENOHD_OWNED" = "1" ] && [ -n "$ZENOHD_ROUTER_PID" ]; then
        # By pid, never by name — see stop_zenohd.
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
            ZENOHD_OWNED=0
        fi
    fi
    exit "$rc"
}
trap cleanup EXIT INT TERM

start_iox_roudi() {
    # `kill -0` alone answers "a process with that pid exists", not "OUR
    # daemon is still alive": bash reaps a background child and the OS can
    # recycle its pid, after which a stale IOX_ROUDI_PID would claim
    # ownership of — and later kill — an unrelated process. So the
    # liveness half is the START-TIME identity. RouDi is started unwrapped
    # (`iox-roudi ... &`), so its comm IS iox-roudi, and that check is
    # KEPT as a second conjunct rather than dropped as redundant: where
    # `ps` is unavailable the identity is empty and `node_still_ours`
    # degrades to a bare existence test, and the comm conjunct is what
    # keeps the answer in that case exactly as conservative as it was.
    # (start_zenohd cannot do the same: `ros2 run` is a Python wrapper, so
    # the pid we hold may report python3 or ros2 rather than the router;
    # its exposure is noted there.)
    if node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" &&
       [ "$(ps -p "$IOX_ROUDI_PID" -o comm= 2>/dev/null | awk '{print $1}')" = "iox-roudi" ]; then
        return 0
    fi
    # Identity, not a port: RouDi has no TCP listener to confuse, but the
    # same rule applies — a DEFUNCT iox-roudi must not read as a live one
    # (`pgrep -x` matches zombies), or we would skip our own start and then
    # verify against a corpse.
    # RETIRE dead ownership before ANY branch below. The liveness check
    # above only returns for a PID that is still alive, so we reach here
    # having possibly owned a RouDi that has since DIED. Leaving
    # IOX_ROUDI_OWNED at 1 makes stop_iox_roudi unlink
    # `/dev/shm/iceoryx_*` — the live mempools of whatever daemon is up
    # NOW, quite possibly an external one; leaving IOX_ROUDI_PID set aims
    # a kill at a recycled pid; and leaving IOX_ROUDI_LOG pointing at the
    # dead run's log makes the cyclonedds delta oracle verify this cell
    # against a corpse's output.
    if [ "$IOX_ROUDI_OWNED" = "1" ] || [ -n "$IOX_ROUDI_PID" ]; then
        echo "  (the iox-roudi this run started is gone; dropping our" >&2
        echo "   ownership of it, and its log)" >&2
        IOX_ROUDI_PID=""
        IOX_ROUDI_OWNED=0
        unset IOX_ROUDI_LOG
    fi
    if ps -eo stat=,comm= |
           awk '$2 == "iox-roudi" && $1 !~ /Z/ { found = 1 } END { exit !found }'; then
        echo "  iox-roudi already running externally (NOT owned by this run:"
        echo "   it is left alive and its /dev/shm pools are never removed)"
        # verify_shm.sh's cyclonedds arm proves SHM engagement from RouDi's
        # debug log, which only exists if the EXTERNAL owner captured one.
        # Refuse now, with the recipe, rather than at the verify step. The
        # retirement above means the caller must have exported it for THIS
        # external daemon; a dead run's path cannot satisfy it.
        if [ -z "${IOX_ROUDI_LOG:-}" ] || [ ! -f "${IOX_ROUDI_LOG:-}" ]; then
            echo "run_bench.sh: an externally managed iox-roudi must publish its debug log" >&2
            echo "  for the SHM-engagement oracle: start it as \`iox-roudi -l debug > LOG 2>&1\`" >&2
            echo "  and export IOX_ROUDI_LOG=LOG before running this script (currently" >&2
            echo "  ${IOX_ROUDI_LOG:-<unset>}). Or stop it and let this script own one." >&2
            return 2
        fi
        export IOX_ROUDI_LOG
        return 0
    fi
    # Custom config with mempools sized for the 10-payload sweep up to
    # 16 MB. Without it RouDi's built-in defaults max out around 256 KB
    # and 1 MB+ Pod<N> publishes fail with "no chunks available".
    local roudi_args=()
    if [ -f "$SCRIPT_DIR/configs/iox_roudi_config.toml" ]; then
        roudi_args=(-c "$SCRIPT_DIR/configs/iox_roudi_config.toml")
    fi
    # `-l debug` + a captured log are LOAD-BEARING: verify_shm.sh's
    # cyclonedds arm proves SHM engagement by the probe's registration
    # lines appearing in this log (the apt CycloneDDS builds print no
    # iceoryx marker of their own — see verify_shm.sh).
    export IOX_ROUDI_LOG="${IOX_ROUDI_LOG:-/tmp/iox_roudi_bench.log}"
    : > "$IOX_ROUDI_LOG"
    iox-roudi "${roudi_args[@]}" -l debug > "$IOX_ROUDI_LOG" 2>&1 &
    IOX_ROUDI_PID=$!
    IOX_ROUDI_ID="$(pid_identity "$IOX_ROUDI_PID")"
    IOX_ROUDI_OWNED=1
    # Readiness, not a guess: RouDi announces its mempool segments in the
    # debug log once it is serving. Bounded; a dead daemon is reported as
    # dead rather than becoming a confusing "no chunks available" later.
    local waited=0
    while [ "$waited" -lt 100 ]; do
        if ! node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID"; then
            echo "run_bench.sh: iox-roudi exited during startup — see $IOX_ROUDI_LOG" >&2
            sed 's/^/  /' "$IOX_ROUDI_LOG" | tail -20 >&2
            return 2
        fi
        # Two spellings across the iceoryx versions this suite spans; the
        # needle is named on the timeout path below so a third is diagnosable.
        if grep -qiE "ready for clients|RouDi is ready" "$IOX_ROUDI_LOG" 2>/dev/null; then
            return 0
        fi
        sleep 0.1
        waited=$((waited + 1))
    done
    echo "run_bench.sh: iox-roudi did not report readiness within 10s (looked for" >&2
    echo "  /ready for clients|RouDi is ready/ in $IOX_ROUDI_LOG)" >&2
    sed 's/^/  /' "$IOX_ROUDI_LOG" | tail -20 >&2
    return 2
}

stop_iox_roudi() {
    # PER ATTEMPT, and a `local` rather than a global that every caller must
    # remember to reset. A script global would be set on a failed
    # kill and never cleared, so the FIRST failure would latch every later
    # teardown in the run: a freshly started RouDi that was signalled and
    # reaped cleanly would still take the failure path, print "keeping ownership
    # of the un-killable iox-roudi" about a daemon that was already gone,
    # return 1, and skip `rm -f /dev/shm/iceoryx_*` for every remaining
    # cell — leaving exactly the stale segments the branch exists to avoid.
    # The sweep loop calls this once per (rmw x shm-mode) plus an end-of-rmw
    # teardown, so "once per run" is not the right scope. Nothing here
    # needs to outlive one attempt: what persists is IOX_ROUDI_OWNED and
    # IOX_ROUDI_PID.
    local roudi_teardown_failed=0
    if [ "$IOX_ROUDI_OWNED" != "1" ]; then
        # An external RouDi is not ours to stop, and its pools are LIVE:
        # unlinking /dev/shm/iceoryx_* here would pull the segments out from
        # under the daemon and every other client attached to it.
        return 0
    fi
    if [ -n "$IOX_ROUDI_PID" ]; then
        # Identity-gated like the two zenoh pids — a recorded pid outlives
        # the process it named, and RouDi is unwrapped so `comm` is exact.
        # Wait only if we SIGNALLED: RouDi is a daemon that never exits on
        # its own, so an unconditional wait after a refused signal would
        # hang teardown forever if it were still our child (a `comm` the
        # gate did not recognise is the reachable way in).
        kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi TERM
        case "$?" in
            0)  # Signalled — and then BOUNDED, the same contract the
                # workspace sampler follows. The comment above says
                # RouDi "never exits on its own"; a plain `wait` after a
                # TERM it ignores or is wedged against therefore hangs
                # teardown for good, which is the failure the refused-signal
                # arm below avoids and this arm must avoid too.
                # BY IDENTITY, both the poll and the escalation. A bare
                # `kill -0` grace and a
                # bare `kill -KILL` on a pid recorded when RouDi STARTED
                # would send the one signal this file's own comment says must
                # never be sent blind, blind, at the end of every
                # teardown, and over a window of the whole cell rather
                # than a 2 s grace. The daemon is not an `&` child on
                # every path, and this script runs as root in the
                # container, so a recycled pid would take an unrestricted
                # SIGKILL. `node_still_ours` and `kill_owned_pid` are the
                # guard.
                _r_waited=0
                while [ "$_r_waited" -lt 100 ]; do
                    node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" || break
                    sleep 0.1
                    _r_waited=$((_r_waited + 1))
                done
                if node_still_ours "$IOX_ROUDI_PID" "$IOX_ROUDI_ID"; then
                    # The GUARD and the ACTOR must agree. `node_still_ours`
                    # falls back to a bare `kill -0` when the identity is
                    # empty, while `kill_owned_pid` REFUSES an empty
                    # identity before signalling — so announcing the
                    # SIGKILL before sending it printed "sending SIGKILL"
                    # on a path that sent nothing. Announce the OUTCOME.
                    # And no `|| true`: this helper's own header forbids
                    # it, because rc 2 ("it IS ours and the signal
                    # FAILED") must reach the arm below that KEEPS the
                    # /dev/shm pools — reaching `rm -f /dev/shm/iceoryx_*`
                    # under a live daemon is the corruption that arm
                    # exists to refuse.
                    kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi KILL
                    case "$?" in
                        0)  echo "warning: iox-roudi (pid $IOX_ROUDI_PID) did not" >&2
                            echo "  exit within 10s of SIGTERM; SIGKILL sent." >&2
                            ;;
                        2)  roudi_teardown_failed=1 ;;
                        *)  echo "warning: iox-roudi (pid $IOX_ROUDI_PID) is still" >&2
                            echo "  up after SIGTERM and could NOT be SIGKILLed by" >&2
                            echo "  identity (no start-time identity, or the pid is" >&2
                            echo "  no longer ours) — refusing to escalate blind." >&2
                            roudi_teardown_failed=1
                            ;;
                    esac
                fi
                # Reap only when teardown did not fail: `wait` on a daemon
                # this shell could not signal blocks for good, and RouDi
                # never exits by itself.
                if [ "$roudi_teardown_failed" != "1" ]; then
                    wait "$IOX_ROUDI_PID" 2>/dev/null || true
                fi
                ;;
            2)  # OURS, and the signal FAILED. An earlier fix gave the helper this
                # code and then let both callers read it like a benign
                # refusal — so a LIVE RouDi kept running while the caller
                # cleared its ownership and unlinked /dev/shm/iceoryx_*,
                # pulling the mempools out from under it. Keep ownership,
                # keep the pools, and say so; the run is over either way,
                # and a loud stray beats a silent corrupt one.
                roudi_teardown_failed=1
                echo "warning: could not signal the iox-roudi this run started" >&2
                echo "  (pid $IOX_ROUDI_PID) — it is LEFT RUNNING and its" >&2
                echo "  /dev/shm/iceoryx_* pools are LEFT IN PLACE (unlinking" >&2
                echo "  them under a live daemon corrupts it). Kill it by hand:" >&2
                echo "  kill $IOX_ROUDI_PID" >&2
                ;;
            *)  # 1 = not ours, or already gone. The ordinary case.
                ;;
        esac
        if [ "$roudi_teardown_failed" != "1" ]; then
            IOX_ROUDI_PID=""
            IOX_ROUDI_ID=""
        fi
    fi
    if [ "$roudi_teardown_failed" = "1" ]; then
        # Ownership is what arms this teardown, so dropping it after a
        # FAILED kill is how a stray daemon becomes permanently nobody's:
        # the next start_iox_roudi would find it, call it EXTERNAL, and
        # never try again. Keep it, and refuse the pool unlink below.
        echo "warning: keeping ownership of the un-killable iox-roudi and" >&2
        echo "  SKIPPING the /dev/shm/iceoryx_* cleanup for this run." >&2
        return 1
    fi
    IOX_ROUDI_OWNED=0
    # Our own log path must not later satisfy the external-RouDi contract.
    unset IOX_ROUDI_LOG
    # iox-roudi's mempool segments persist in /dev/shm even after the
    # daemon exits (POSIX SHM isn't reaped until shm_unlink). Hygiene
    # cleanup so the next RMW sees a clean tmpfs — only ever for a daemon
    # THIS invocation started.
    rm -f /dev/shm/iceoryx_* 2>/dev/null || true
}

# TCP reachability probe for the router port. bash's /dev/tcp redirection
# is available in every container image this suite builds; the `timeout`
# keeps a black-holed port from stalling the probe.
#
# IDENTITY WARNING: an open port says only that SOMETHING is listening. It
# is evidence of READINESS for a router we have already identified, never
# evidence that the listener IS rmw_zenohd — see zenohd_live_pid.
zenohd_port_open() {
    timeout 1 bash -c 'exec 3<>/dev/tcp/127.0.0.1/7447' 2>/dev/null
}

# PID of a LIVE (non-defunct) rmw_zenohd, or empty. Identity by what the
# process IS, never by a port being open: `pgrep -x` alone also matches
# DEFUNCT processes (our own router, reparented to a container init that
# may not reap, becomes exactly that between the per-mode restart's stop
# and start), and a port check alone matches ANY listener.
zenohd_live_pid() {
    ps -eo pid=,stat=,comm= |
        awk '$3 == "rmw_zenohd" && $2 !~ /Z/ { print $1; exit }'
}


start_zenohd() {
    # rmw_zenoh's discovery wants rmw_zenoh's own router (`rmw_zenohd`),
    # NOT the standalone Eclipse Zenoh router. Without it, publishers
    # and subscribers can't see each other: subscription_count stays 0
    # forever and the latency_node bootstrap times out at 15 s.
    # By START-TIME IDENTITY, not by liveness: a bare `kill -0` here read
    # a RECYCLED pid as "our router is still up", so this function
    # returned 0, started no router, and the cell ran with no discovery at
    # all — a bootstrap timeout charged to the transport, which is the
    # exact failure the port-squatter refusal below exists to prevent,
    # arriving through a different door. No comm check: `ros2 run` is a
    # Python wrapper, so the pid we captured may report python3/ros2
    # rather than rmw_zenohd and a comm gate would false-negative on a
    # router that IS ours (the exact check start_iox_roudi can make).
    # Where `ps` is unavailable the identity is empty and this degrades to
    # the old bare existence test — never to a false "not ours".
    if node_still_ours "$ZENOHD_PID" "$ZENOHD_ID"; then
        return 0
    fi
    # Identify the router by WHAT IT IS, never by port 7447 being open.
    # A port check answered "a router is already up" for ANY listener on
    # 7447 — a stray dev server, a leftover Eclipse zenohd, an SSH forward
    # — and the script would then skip starting rmw_zenohd and run the cell
    # with no discovery at all: the nodes' bootstrap times out and the cell
    # is charged to the transport rather than to the port squatter.
    # `pgrep -x` alone is not enough either: it matches DEFUNCT processes,
    # and our own router becomes exactly that between the per-mode restart's
    # stop and start (a zombie read as "external" skips the restart AND, via
    # the ownership flag, disarms teardown for a router we own).
    # RETIRE dead ownership before ANY branch below. We only reach this
    # line when the PID we started is not alive, so an ownership flag left
    # at 1 would let stop_zenohd `pkill -x rmw_zenohd` — killing a router
    # we no longer own, quite possibly the external one we are about to
    # adopt — and a stale PID would aim a kill at a recycled one. Hoisted
    # above the branches so the adopt, the port-squatter refusal and a
    # fresh start all leave the same clean state.
    # Retire ONLY when neither the launcher nor the router it forked is
    # alive. The early return above checks the wrapper pid, and `ros2 run`
    # can die while the rmw_zenohd it forked survives — retiring on the
    # wrapper alone would then adopt OUR OWN orphan as "external" and
    # permanently disarm teardown for a process this run created.
    if node_still_ours "$ZENOHD_ROUTER_PID" "$ZENOHD_ROUTER_ID"; then
        # Our router outlived its `ros2 run` launcher. RETURN here rather
        # than falling through: the identity check below would find this
        # very process, call it external, and permanently disarm teardown
        # for a router this run created.
        echo "  (our rmw_zenohd $ZENOHD_ROUTER_PID outlived its launcher;" >&2
        echo "   keeping ownership of it)" >&2
        # SAY it and MEAN it. This branch claimed to keep ownership without
        # asserting the flag, so after any teardown that cleared it the
        # claim was false and both the retry and the EXIT trap stayed
        # disarmed — a silent leak, since the message says the opposite.
        ZENOHD_OWNED=1
        # The launcher is provably dead (the early return above checked it),
        # so clear its pid: teardown otherwise aims `pkill -P` and `kill` at
        # a pid the kernel may have recycled onto an unrelated process.
        ZENOHD_PID=""
        return 0
    elif [ "$ZENOHD_OWNED" = "1" ] || [ -n "$ZENOHD_PID" ] ||
         [ -n "$ZENOHD_ROUTER_PID" ]; then
        echo "  (the rmw_zenohd this run started is gone; dropping our" >&2
        echo "   ownership of it)" >&2
        ZENOHD_PID=""
        ZENOHD_ID=""
        ZENOHD_ROUTER_PID=""
        ZENOHD_ROUTER_ID=""
        ZENOHD_OWNED=0
    fi
    if [ -n "$(zenohd_live_pid)" ]; then
        echo "  rmw_zenohd already running externally (NOT owned by this run:"
        echo "   it is left alive on teardown)"
        return 0
    fi
    # Nothing named rmw_zenohd is alive, so anything holding 7447 is NOT a
    # router we could use. Refuse with the recipe rather than starting a
    # router that cannot bind and reporting the failure as a transport
    # problem three minutes later.
    if zenohd_port_open; then
        echo "run_bench.sh: TCP 127.0.0.1:7447 is held, but no live rmw_zenohd" >&2
        echo "  process exists — the listener is NOT the router this lane needs." >&2
        echo "  Identify it (\`ss -ltnp sport = :7447\` or \`lsof -iTCP:7447 -sTCP:LISTEN\`)" >&2
        echo "  and stop it, or run this container with its own network namespace." >&2
        echo "  (Refusing rather than skipping the router start: a zenoh cell with" >&2
        echo "   no discovery times out and looks like a transport failure.)" >&2
        echo "  NOTE for no_shm cells: they run with --network host but NOT" >&2
        echo "  --pid host, so a HOST-side listener is visible here while its" >&2
        echo "  process is not — \`ss -ltnp\` will show a blank owner. The likely" >&2
        echo "  owner is this suite's own native zenoh leg; run" >&2
        echo "  \`pkill -x zenoh_shm_round_trip_pong\` ON THE HOST." >&2
        return 2
    fi
    if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null; then
        # return 2, NOT 0: returning success with no router started makes
        # every `start_zenohd || exit 2` call site pass and the zenoh cell
        # run discovery-less — a bootstrap timeout charged to the
        # transport, which is the exact failure the identity check above
        # exists to prevent, arriving through a different door. The rmw
        # presence is already hard-checked before we get here, so reaching
        # this branch means the two probes disagree; stderr, not stdout.
        echo "run_bench.sh: rmw_zenoh_cpp is not on the overlay, so no router" >&2
        echo "  can be started — refusing rather than running a zenoh cell" >&2
        echo "  with no discovery." >&2
        return 2
    fi
    ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null 2>&1 &
    ZENOHD_PID=$!
    ZENOHD_ID="$(pid_identity "$ZENOHD_PID")"
    ZENOHD_OWNED=1
    # Readiness, not a guess: without this wait, a router that failed to start (missing
    # overlay, a bind refused) is discovered only when the
    # measurement nodes' bootstrap times out — and verify_shm.sh may
    # meanwhile stand up and tear down a router of its own, leaving the cell
    # routerless. The port is legitimate evidence HERE and only here: we
    # have already established the identity of the process we started, so
    # an open port is its readiness, not somebody else's presence.
    local waited=0
    while [ "$waited" -lt 100 ]; do
        # Record the router as soon as it is IDENTIFIABLE, not when it is
        # READY. Whatever is live now is ours: the identity check above
        # proved no live rmw_zenohd existed before we spawned. Both exits
        # below are reached with a forked-but-not-yet-listening router as
        # the likeliest state, and an unrecorded router is UNKILLABLE —
        # teardown is gated on this variable, and the next start_zenohd
        # would find the stray, call it external, and permanently disarm
        # teardown for a process this run created.
        if [ -z "$ZENOHD_ROUTER_PID" ]; then
            ZENOHD_ROUTER_PID="$(zenohd_live_pid)"
            ZENOHD_ROUTER_ID="$(pid_identity "$ZENOHD_ROUTER_PID")"
        fi
        if ! node_still_ours "$ZENOHD_PID" "$ZENOHD_ID"; then
            echo "run_bench.sh: rmw_zenohd exited during startup" >&2
            return 2
        fi
        if [ -n "$ZENOHD_ROUTER_PID" ] && zenohd_port_open; then
            return 0
        fi
        sleep 0.1
        waited=$((waited + 1))
    done
    # Last look before giving up: `ros2 run` may have forked the router in
    # the final slice, and this exit leads to teardown.
    if [ -z "$ZENOHD_ROUTER_PID" ]; then
        ZENOHD_ROUTER_PID="$(zenohd_live_pid)"
        ZENOHD_ROUTER_ID="$(pid_identity "$ZENOHD_ROUTER_PID")"
    fi
    echo "run_bench.sh: rmw_zenohd did not accept connections on port 7447 within 10s" >&2
    return 2
}


stop_zenohd() {
    # Per attempt, like stop_iox_roudi's — never a global.
    local zenohd_teardown_failed=0
    local launcher_refused=0
    if [ "$ZENOHD_OWNED" != "1" ]; then
        # The router was already up when we arrived: it belongs to the
        # caller. `pkill -x rmw_zenohd` here would kill THEIR router (and
        # with it every unrelated ROS 2 process talking through it).
        return 0
    fi
    if [ -n "$ZENOHD_PID" ]; then
        # `ros2 run` is a Python wrapper that may exec into rmw_zenohd or
        # fork it: kill the router among OUR launcher's children first,
        # then the launcher itself.
        # Identity-gated, same rule as the router: the launcher forks the
        # router and can exit early, and its pid is as recyclable as any
        # other — this one is a `ros2` Python wrapper, so a recycled pid is
        # very plausibly an unrelated ros2 command of the caller's.
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
    # The router itself, BY PID. The blanket `pkill -KILL -x rmw_zenohd`
    # this replaces rested on a START-time invariant ("no external router
    # existed when we started") that says nothing about now: ours can die
    # mid-run and a same-named one appear, and stop_zenohd is reached
    # without an intervening start_zenohd on three paths (the per-mode
    # restart, the end-of-rmw teardown, and the EXIT trap), so the sweep
    # could kill a router this run never owned. Naming the pid also covers
    # a router forked deeper than one level, which is what the sweep
    # existed for.
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
            zenohd_teardown_failed=1
        else
            ZENOHD_ROUTER_PID=""
            ZENOHD_ROUTER_ID=""
        fi
    fi
    if [ "$zenohd_teardown_failed" = "1" ]; then
        # The router is STILL RUNNING, so two things below are wrong for it.
        #
        # The drain loop waits for the router to LEAVE — it never will, so
        # it would burn its full 5 s on every teardown from here on.
        #
        # And `ZENOHD_OWNED=0` is the bug this branch exists to prevent:
        # the EXIT trap's copy of it is guarded, but this one runs
        # unconditionally at the end of the function, outside the guard —
        # doing exactly what the comment above says not to do. Since
        # stop_zenohd early-returns on `ZENOHD_OWNED != 1` and the EXIT
        # trap's router teardown is gated on the same flag, one failed kill
        # disarmed BOTH the retry and the trap for that router, permanently.
        return 1
    fi
    # SIGKILL is ASYNCHRONOUS: until the task is scheduled out it is
    # neither gone nor defunct, so an immediate restart (the per-mode
    # stop/start below) could read our own dying router as an EXTERNAL one
    # — skipping the start AND, through the ownership flag, disarming
    # teardown for a router we own. Wait for it to actually leave.
    local gone=0
    while [ "$gone" -lt 50 ]; do
        [ -z "$(zenohd_live_pid)" ] && break
        sleep 0.1
        gone=$((gone + 1))
    done
    ZENOHD_OWNED=0
}

# Clean FastDDS + zenoh /dev/shm state between batches / sizes. iceoryx
# (v1, RouDi) cleanup is deliberately NOT here — clean_shm runs after
# `start_iox_roudi` for cyclonedds+SHM, and wiping /dev/shm/iceoryx_*
# while RouDi is live nukes the mempools the daemon just allocated.
# iceoryx cleanup lives in `stop_iox_roudi` instead.
clean_shm() {
    # BOTH Fast DDS prefixes: 2.x (humble/jazzy) names its segments
    # fastrtps_*, 3.x (lyrical) renamed them fastdds_* — and 3.x does
    # NOT unlink segments even on graceful process exit (measured
    # 2026-08-12: a cleanly SIGTERM'd publisher leaves its ~70 MB
    # segment + port files behind), so without the 3.x sweep a
    # multi-size in-container run accumulates ~212 MB per size toward
    # the 4g --shm-size ceiling.
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
    # DataSharing (zc lane) writer-history segments — stale ones from a
    # SIGKILL'd writer confuse the next reader (the Fast DDS release ships
    # `fastdds shm clean` for exactly this).
    rm -f /dev/shm/fast_datasharing_* /dev/shm/sem.fast_datasharing_* 2>/dev/null || true
    rm -f /dev/shm/*.zenoh /dev/shm/zenoh* /dev/shm/zenohshm* 2>/dev/null || true
    clean_iox2_state
}

# iceoryx2 (rmw_cerulion's transport) keeps process-global state in TWO
# places a SIGKILL'd node does not unlink: the service/node registry +
# notification sockets under /tmp/iceoryx2, and the /dev/shm/iox2_*
# segments. A stale registry from a previous size poisons the next one's
# notifier: iceoryx2 floods FailedToDeliverSignal and the round trips
# never complete.
#
# Called UNCONDITIONALLY from clean_shm, not only on cerulion cells, for
# one reason: the DDS and zenoh RMWs never create
# iceoryx2 state, so removing it is a no-op for them, and one
# unconditional sweep is less to keep true than a per-rmw scope. Like the
# rest of clean_shm it runs only between sizes/batches, when no bench
# process of ours is alive.
clean_iox2_state() {
    local root="${IOX2_STATE_DIR:-/tmp/iceoryx2}"
    rm -rf "$root/services" "$root/nodes" 2>/dev/null || true
    # `find ... -exec rm` rather than a glob: an empty match must be a
    # clean no-op, not a literal-glob rm (nullglob is off here).
    find "$root" -maxdepth 2 -name 'iox2_*' -type s -exec rm -f {} + 2>/dev/null || true
    find "$root" -maxdepth 2 -name '*.event' -exec rm -f {} + 2>/dev/null || true
    find /dev/shm -maxdepth 1 -name 'iox2*' -exec rm -f {} + 2>/dev/null || true
}

# Build librmw_cerulion.so from the bind-mounted Rust repo and stage it
# into a minimal ament prefix, so RMW_IMPLEMENTATION=rmw_cerulion can
# dlopen it.
#
# WHY NOT PREBUILT INTO THE IMAGE: the crate's build.rs runs bindgen
# against the LIVE distro headers it finds on AMENT_PREFIX_PATH, and the
# introspection struct layouts drift across distros. Baking one .so into
# the image would either pin a single distro or ship a vendored-bindings
# build that is compilation-correct and ABI-wrong for a deployed .so. It
# also makes freshness structural: the binary measured is the one built
# from the tree mounted at /work in this run, not whatever an older image
# happened to carry.
#
# COST: the FIRST cerulion container of a campaign pays a full release
# build. Every later one re-runs cargo against the same $repo/target and
# gets a freshness no-op in seconds. That is why the repo mount must be
# writable and CARGO_TARGET_DIR must point INSIDE it: a per-container
# scratch target dir would pay the full build once per cell.
ensure_rmw_cerulion() {
    local repo="${CER_RMW_REPO:-/work}"
    # Container-local, NOT under $CER_BENCH_RAW_DUMP_DIR: that directory is
    # bind-mounted from the host results tree, and a ~100 MB .so staged
    # there would be committed alongside the evidence it is not part of.
    # The build RECEIPT (a text file naming its sha256) goes to _logs
    # instead, which is what an auditor actually needs.
    local prefix="${CER_RMW_PREFIX:-/tmp/rmw_cerulion_prefix}"
    local lib_dir="$prefix/lib"
    local so_name="librmw_cerulion.so"

    if [ ! -f "$repo/crates/rmw_cerulion/Cargo.toml" ]; then
        echo "run_bench.sh: rmw_cerulion source not found at $repo/crates/rmw_cerulion" >&2
        echo "  cerulion cells build the cdylib from the bind-mounted repo; there is no" >&2
        echo "  prebuilt fallback (a stale .so would be measured under this run's label)." >&2
        echo "  Mount the repo: docker run -v \"\$PWD\":/work ... (bench.py does this for" >&2
        echo "  cerulion cells; set CER_RMW_REPO to override the path)." >&2
        exit 2
    fi
    if ! command -v cargo >/dev/null 2>&1; then
        echo "run_bench.sh: cargo not found in this image - cerulion cells cannot build" >&2
        echo "  librmw_cerulion.so. Rebuild the image (bench.py --build-image): the" >&2
        echo "  Dockerfile installs a Rust toolchain for exactly this." >&2
        exit 2
    fi

    echo "  building rmw_cerulion cdylib from $repo (bindgen against the live distro headers)"
    # AMENT_PREFIX_PATH is already exported by the sourced ROS setup, so
    # build.rs takes its bindgen-against-real-headers path.
    #
    # CARGO_TARGET_DIR inside the mounted repo: see the COST note above.
    # CARGO_HOME has to persist for the same reason and is a SEPARATE
    # problem: an up-to-date target dir does not save a rebuild if the
    # registry cache is gone, because cargo still needs the dependency
    # SOURCES on disk to build the unit graph, so a container-local
    # CARGO_HOME re-downloads the whole index and every .crate on every
    # cell. It cannot simply be /root/.cargo: that is where rustup put
    # the toolchain, and mounting over it would hide the `cargo` shim.
    # Under target/ instead, which every checkout already ignores.
    # RUSTUP_HOME is untouched, so the shim still resolves its toolchain.
    if ! env -u LD_PRELOAD CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$repo/target}" \
            CARGO_HOME="${CARGO_HOME:-$repo/target/.bench-cargo-home}" \
            cargo build --release --manifest-path "$repo/crates/rmw_cerulion/Cargo.toml"; then
        echo "run_bench.sh: cargo build of rmw_cerulion failed - refusing the cell" >&2
        exit 2
    fi

    local built_so="${CARGO_TARGET_DIR:-$repo/target}/release/$so_name"
    if [ ! -f "$built_so" ]; then
        echo "run_bench.sh: cargo reported success but $built_so is absent" >&2
        exit 2
    fi

    mkdir -p "$lib_dir"
    cp -f "$built_so" "$lib_dir/$so_name"
    # rmw_implementation dlopens the name `rmw_cerulion` -> librmw_cerulion.so
    # via the loader search path; the ament prefix is what makes the name
    # resolvable to ROS 2's rmw discovery.
    export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    echo "  rmw_cerulion -> $lib_dir/$so_name (on AMENT_PREFIX_PATH + LD_LIBRARY_PATH)"

    # Per-cell receipt: WHICH binary this cell measured. The .so is built
    # here rather than shipped, so without this the artifacts carry no
    # record of the tree it came from.
    local receipt="$LOGS_DIR/${CER_BENCH_RAW_NAME}_rmw_cerulion.txt"
    {
        echo "librmw_cerulion.so built by run_bench.sh at $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "repo:        $repo"
        echo "target dir:  ${CARGO_TARGET_DIR:-$repo/target}"
        echo "sha256:      $(sha256sum "$lib_dir/$so_name" 2>/dev/null | awk '{print $1}')"
        echo "size bytes:  $(stat -c %s "$lib_dir/$so_name" 2>/dev/null)"
        echo "cargo:       $(cargo --version 2>&1)"
        echo "rustc:       $(rustc --version 2>&1)"
        echo "ROS_DISTRO:  ${ROS_DISTRO_NAME:-unknown}"
        # A bind-mounted worktree usually carries no usable .git for the
        # container user: a linked worktree's .git FILE points at a
        # gitdir outside the mount. Report the failure text rather than
        # an empty sha, and do NOT count lines on a command that may have
        # failed: `git status --porcelain 2>&1 | wc -l` renders a fatal
        # error as "1 modified path(s)", which is a claim about the tree
        # that nothing measured. Ask, then say which answer you got.
        echo "git HEAD:    $(git -C "$repo" rev-parse HEAD 2>&1 | head -1)"
        if _dirty=$(git -C "$repo" status --porcelain 2>/dev/null); then
            echo "git status:  $(printf '%s' "$_dirty" | grep -c . || true) modified path(s)"
        else
            echo "git status:  unavailable (git could not read $repo)"
        fi
    } > "$receipt" 2>/dev/null || true
    echo "  build receipt -> $(basename "$receipt")"
}

# Run one (rmw, shm, chrt, size) measurement. Returns 0 on success,
# 1 on bench timeout / failure.
run_one() {
    local rmw=$1 shm=$2 chrt=$3 size=$4

    # Fresh DDS/zenoh SHM state per size — each size is a fresh
    # 3-process session (see clean_shm docs).
    clean_shm

    export CER_BENCH_PAYLOAD_SIZE="$size"
    if [ "$chrt" = "on" ]; then
        export CER_BENCH_CHRT=1
    else
        export CER_BENCH_CHRT=0
    fi

    # FastDDS 16 MB cells need the 256 MiB SHM segment; every other
    # size keeps the 64 MiB config (256 MiB as the global segment broke
    # size=64 on jazzy — see configs/fastdds_shm_16mb.xml). `shm` mode
    # ONLY: the zc (DataSharing) lane keeps fastdds_zc.xml at every
    # size — DataSharing bypasses the SHM transport segment entirely.
    if [ "$rmw" = "fastdds" ] && [ "$shm" = "shm" ]; then
        local fastdds_profile="$SCRIPT_DIR/configs/fastdds_shm.xml"
        if [ "$size" = "16777216" ]; then
            fastdds_profile="$SCRIPT_DIR/configs/fastdds_shm_16mb.xml"
            echo "    (fastdds 16MB cell: switching to 256MiB SHM segment profile)"
        fi
        export FASTRTPS_DEFAULT_PROFILES_FILE="$fastdds_profile"
        export FASTDDS_DEFAULT_PROFILES_FILE="$fastdds_profile"
    fi

    # Quiescent: per-payload rate schedule + sample counts derived from
    # a ~60 s measurement window, anchored to typical robotics sensor
    # rates (1 kHz control-loop payloads → 10 Hz full lidar scans).
    # The schedule tuple is (rate, TOTAL iterations, warmup); the
    # exported CER_BENCH_TARGET_SAMPLES is MEASURED = total − warmup
    # (G1 contract: CER_BENCH_TARGET_SAMPLES means measured samples in
    # EVERY component — the C++ nodes collect measured+warmup, drop
    # the warmup prefix, and dump exactly measured). Lockstep in FOUR
    # places — see the header.
    # Override precedence (highest wins):
    #   1. CER_BENCH_TARGET_RATE_HZ / TARGET_SAMPLES / WARMUP env vars
    #      — apply to all payloads in this invocation (TARGET_SAMPLES
    #      is measured; passed through verbatim).
    #   2. The per-payload schedule below.
    # fixed100: ONE uniform tuple at every
    # size — rate=100, total=2100, warmup=100 (measured 2000). The
    # FALLBACK LADDER (100→50→20→sensor floor) is driven by bench.py,
    # which re-invokes this container per rung with the rung in
    # CER_BENCH_TARGET_RATE_HZ (CALLER_RATE_HZ here) and writes the
    # achieved-rate sidecar host-side; a manual in-container fixed100
    # run that cannot sustain its rate simply FAILS loudly (exit 13) —
    # this driver never ladders on its own.
    # Backtoback: no rate; counts default to 10000 measured / 1000
    # warmup in the binaries unless TARGET_SAMPLES/WARMUP override.
    local rate=10 total=600 warmup=50
    case "$size" in
        64|256|1024)        rate=1000; total=60000; warmup=5000 ;;
        4096|16384)         rate=500;  total=30000; warmup=2500 ;;
        65536)              rate=200;  total=12000; warmup=1000 ;;
        262144)             rate=100;  total=6000;  warmup=500  ;;
        1048576)            rate=60;   total=3600;  warmup=300  ;;
        4194304)            rate=30;   total=2100;  warmup=100  ;;
        16777216)           rate=10;   total=2050;  warmup=50   ;;
    esac
    if [ "$CER_BENCH_PACING" = "fixed100" ]; then
        # Uniform fixed100 tuple — lockstep with native/src/lib.rs::
        # FIXED100_{RATE_HZ,TOTAL,WARMUP}, bench.py::fixed100_schedule,
        # workspace/run_workspace.sh::FIXED100_*.
        rate=100; total=2100; warmup=100
    fi
    if [ "$CER_BENCH_PACING" = "backtoback" ]; then
        unset CER_BENCH_TARGET_RATE_HZ
        export CER_BENCH_TARGET_SAMPLES="${TARGET_SAMPLES:-10000}"
        export CER_BENCH_WARMUP="${WARMUP:-1000}"
    else
        export CER_BENCH_TARGET_RATE_HZ="${CALLER_RATE_HZ:-$rate}"
        export CER_BENCH_TARGET_SAMPLES="${TARGET_SAMPLES:-$((total - warmup))}"
        export CER_BENCH_WARMUP="${WARMUP:-$warmup}"
    fi
    # The .bin sample-count gate below checks against this: MEASURED
    # samples, i.e. bytes/8 == measured (G1).
    EXPECTED_SAMPLES="$CER_BENCH_TARGET_SAMPLES"

    # Per-size node log (shared by the 3 binaries — append-opened so
    # concurrent writers interleave without truncating each other;
    # truncated once here so retries start clean). compile_csv.py
    # backfills the `loaned` column by grepping `loaned=` in this file.
    BIN_LOG="${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${size}_node.log"
    : > "$BIN_LOG"
    export BIN_LOG

    # chrt -f 80 (SCHED_FIFO at priority 80) for the chrt-on dimension.
    # Priority 80 dominates kthreads (ksoftirqd, kworker/n:H) — prio 50
    # was tried in the May campaign and REGRESSED zenoh-UDP cells
    # (discovery reordering/jitter at kthread-shared priority).
    #
    # This runs as root inside the container (--cap-add SYS_NICE
    # --ulimit rtprio=99); if chrt still fails, the cell FAILS rather
    # than silently measuring at normal priority under a chrt1 label.
    local chrt_prio="${BENCH_CHRT_PRIO:-80}"
    local prefix=()
    # EXPERIMENT FLAG, default off, never set by bench.py. Pins each of
    # the three nodes to its OWN core so the kernel cannot place a woken
    # node on the core its waker is still running on. This is a probe for
    # the wake placement finding (METHODOLOGY section 21), not a posture: the
    # native chart lines run unpinned, so turning this on by default
    # would break posture parity with them. Format: three comma free cpu
    # ids, "ping,pong,latency", e.g. CER_BENCH_PIN_CPUS=2,6,10
    local _pin_ping="" _pin_pong="" _pin_lat=""
    if [ -n "${CER_BENCH_PIN_CPUS:-}" ]; then
        IFS=',' read -r _pin_ping _pin_pong _pin_lat <<< "$CER_BENCH_PIN_CPUS"
        if [ -z "$_pin_ping" ] || [ -z "$_pin_pong" ] || [ -z "$_pin_lat" ]; then
            echo "run_bench.sh: CER_BENCH_PIN_CPUS needs three cpu ids 'ping,pong,latency', got '$CER_BENCH_PIN_CPUS'" >&2
            return 2
        fi
    fi
    if [ "$chrt" = "on" ]; then
        if chrt -f "$chrt_prio" /bin/true >/dev/null 2>&1; then
            prefix=(chrt -f "$chrt_prio")
        else
            echo "run_bench.sh: chrt -f $chrt_prio not permitted in this container" >&2
            echo "  (run with --cap-add SYS_NICE --ulimit rtprio=99; refusing to mislabel a non-RT run as chrt=on)" >&2
            return 2
        fi
    fi

    # Pick the binary set for this cell.
    #   rclcpp   — ping + pong + latency (typed-callback receive; the
    #              headline lane). The `stock` lane runs this same trio
    #              (its axis is the CONFIG, not the binaries).
    #   loan     — ping + pong_node_rcl + latency_node_rcl
    #              (rcl_take_loaned_message on receive)
    #   composed — ONE process carrying all three roles
    #              (composed_rtt_node; rmw=composed). latency_pid IS the
    #              whole cell; pong/ping pids stay empty so the reap
    #              logic below degrades to the single-process shape (the
    #              binary prints all three DELIVERY lines itself before
    #              exiting).
    local latency_pid="" pong_pid="" ping_pid=""
    # The identity is taken AT SPAWN, next to `$!`: it is only
    # meaningful while the pid is still ours, and after the first
    # reap it can never be recovered.
    local latency_id="" pong_id="" ping_id=""
    if [ "$rmw" = "composed" ]; then
        "${prefix[@]}" "$INSTALL_LIB/composed_rtt_node" >>"$BIN_LOG" 2>&1 &
        latency_pid=$!
        latency_id=$(pid_identity "$latency_pid")
    else
      case "$RECV_PATH" in
        rclcpp)
          # Start order: latency → pong → ping. latency_node's bootstrap
          # polls for kick subscribers before sending the first kick, so
          # absolute spawn timing isn't critical.
          ${_pin_lat:+taskset -c $_pin_lat} "${prefix[@]}" "$INSTALL_LIB/latency_node" >>"$BIN_LOG" 2>&1 &
          latency_pid=$!
          latency_id=$(pid_identity "$latency_pid")
          ${_pin_pong:+taskset -c $_pin_pong} "${prefix[@]}" "$INSTALL_LIB/pong_node" >>"$BIN_LOG" 2>&1 &
          pong_pid=$!
          pong_id=$(pid_identity "$pong_pid")
          ${_pin_ping:+taskset -c $_pin_ping} "${prefix[@]}" "$INSTALL_LIB/ping_node" >>"$BIN_LOG" 2>&1 &
          ping_pid=$!
          ping_id=$(pid_identity "$ping_pid")
          ;;
        loan)
          ${_pin_lat:+taskset -c $_pin_lat} "${prefix[@]}" "$INSTALL_LIB/latency_node_rcl" >>"$BIN_LOG" 2>&1 &
          latency_pid=$!
          latency_id=$(pid_identity "$latency_pid")
          ${_pin_pong:+taskset -c $_pin_pong} "${prefix[@]}" "$INSTALL_LIB/pong_node_rcl" >>"$BIN_LOG" 2>&1 &
          pong_pid=$!
          pong_id=$(pid_identity "$pong_pid")
          ${_pin_ping:+taskset -c $_pin_ping} "${prefix[@]}" "$INSTALL_LIB/ping_node" >>"$BIN_LOG" 2>&1 &
          ping_pid=$!
          ping_id=$(pid_identity "$ping_pid")
          ;;
      esac
    fi

    # Wall ceiling. 16 MB × warmup+measure round-trips through SHM
    # bandwidth + discovery legitimately exceeds a minute on some
    # RMWs; small payloads finish fast regardless.
    local timeout_s="${BENCH_CELL_TIMEOUT_S:-300}" elapsed=0 rc=0
    # The latency node's LAST rung's status, carried out of the timeout
    # branch to the reap verdict below. Defaults to 1 — `kill_owned_pid`'s
    # "not ours, or already gone" — which is the correct answer on the
    # ordinary path, where this loop ends because the node EXITED and no
    # rung is ever sent.
    local latency_kill_rc=1
    while node_still_ours "$latency_pid" "$latency_id"; do
        if [ "$elapsed" -ge "$timeout_s" ]; then
            # rc FIRST, so the timeout verdict is latched before any
            # signal can change the node's exit status (it must win over
            # the resulting signal status — see the propagation block
            # below).
            rc=1
            # SIGINT, grace, SIGTERM, grace, THEN SIGKILL.
            #
            # INT FIRST, and that ordering is MEASURED rather than
            # reasoned about — on the jazzy bench image, same node, same
            # shell shape:
            #
            #   kill -INT   -> rc 2,   DELIVERY receipt printed
            #   kill -TERM  -> rc 143, NO receipt (killed by the default
            #                  disposition, uncaught)
            #
            # Cause: every sink calls plain `rclcpp::init(argc, argv)` and
            # nothing passes InitOptions, so rclcpp installs a handler for
            # SIGINT ONLY. The plausible opposite reading — that
            # `SignalHandlerOptions::All` handles TERM, and that INT cannot
            # be relied on because a non-interactive shell starts `&`
            # children with SIGINT set to SIG_IGN — is wrong on both
            # halves: TERM is not handled at all, and INT is delivered
            # from exactly that shell shape (rclcpp overrides the
            # inherited disposition). TERM is KEPT as the second rung so
            # a future build that does install an All-handler still gets
            # a graceful stop, and KILL remains the backstop.
            #
            # The ladder covers the latency node as well as ping and
            # pong: it is the one role whose receipt a timed-out cell
            # most needs: a cell whose stamps are all unusable takes no
            # samples, never finalizes, and burns the whole ceiling to
            # arrive here — and its `unstamped=` / `nonpositive_rtt=`
            # counts are the only thing that says WHY. Under an
            # uncatchable SIGKILL rclcpp's spin never returns, so the
            # sink's post-spin receipt cannot run and the counters
            # die with the process. A caught signal lets rclcpp's handler end the
            # spin, the receipt prints, and the KILL below still reaps a
            # node that ignores it.
            kill_owned_pid "$latency_pid" "$latency_id" "" INT || true
            local lat_grace=0
            while [ "$lat_grace" -lt 20 ] &&
                    node_still_ours "$latency_pid" "$latency_id"; do
                sleep 0.1
                lat_grace=$((lat_grace + 1))
            done
            kill_owned_pid "$latency_pid" "$latency_id" "" TERM || true
            lat_grace=0
            while [ "$lat_grace" -lt 20 ] &&
                    node_still_ours "$latency_pid" "$latency_id"; do
                sleep 0.1
                lat_grace=$((lat_grace + 1))
            done
            # READ, not `|| true`. This is the LAST rung, so its status is
            # the only evidence of whether this node was ended at all, and
            # the reap verdict below is what decides whether the backstop's
            # refusal may CLAIM the cell was reaped by identity. Discarding
            # it and then handing the verdict a literal 1 meant an rc 2
            # ("it IS ours and the signal FAILED") published as a reap that
            # did not happen — the reap-accounting error this rung's status
            # exists to close, arriving through the one rung nothing read.
            kill_owned_pid "$latency_pid" "$latency_id" "" KILL
            latency_kill_rc=$?
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    # Reap children GRACEFULLY first: a signal rclcpp ACTS on ends
    # each node's spin → ping/pong print their DELIVERY lines into the
    # node log. A straight SIGKILL eats the delivery accounting.
    #
    # SIGINT, then SIGTERM, then SIGKILL — and the ORDER is measured. The
    # plausible opposite reading is that TERM belongs before INT
    # because `SignalHandlerOptions::All` handles TERM on humble+, while
    # INT cannot be relied on since POSIX makes a non-interactive
    # shell start `&` children with SIGINT set to SIG_IGN. Both halves
    # are false for these binaries. Measured on the jazzy bench image:
    #
    #   ping_node + kill -TERM  -> rc 143, NO `DELIVERY role=ping` line
    #   latency   + kill -INT   -> rc 2,   receipt printed
    #   latency   + kill -TERM  -> rc 143, no receipt
    #
    # because every sink calls plain `rclcpp::init(argc, argv)`, which
    # installs a SIGINT handler and nothing else — and INT IS delivered
    # from the `&`-child shell shape, rclcpp having overridden the
    # inherited SIG_IGN. So INT first is what lets ping and pong print their
    # timeout-path receipts too, not
    # only the latency sink. Bounded ~2 s grace per rung.
    kill_owned_pid "$pong_pid" "$pong_id" "" INT || true
    kill_owned_pid "$ping_pid" "$ping_id" "" INT || true
    local grace=0
    while [ "$grace" -lt 20 ]; do
        if ! node_still_ours "$pong_pid" "$pong_id" &&
                ! node_still_ours "$ping_pid" "$ping_id"; then
            break
        fi
        sleep 0.1
        grace=$((grace + 1))
    done
    kill_owned_pid "$pong_pid" "$pong_id" "" TERM || true
    kill_owned_pid "$ping_pid" "$ping_id" "" TERM || true
    grace=0
    while [ "$grace" -lt 20 ]; do
        if ! node_still_ours "$pong_pid" "$pong_id" &&
                ! node_still_ours "$ping_pid" "$ping_id"; then
            break
        fi
        sleep 0.1
        grace=$((grace + 1))
    done
    # The KILL rung, BY IDENTITY. Without a KILL rung of their own,
    # ping and pong would be left after TERM to the blanket pattern sweep
    # below, which is the one reap in this file that proves no ownership.
    # A node that ignores both graceful signals is ended by the pid
    # this shell captured at its spawn, so the sweep is a backstop for
    # strays rather than the mechanism that ends the cell — which is what
    # lets the sweep refuse itself where its confinement is unproven.
    #
    # The rc is READ, not discarded: `kill_owned_pid` returns 1 without
    # signalling when it has no identity to check, and that is the state
    # in which the sweep's refusal must NOT claim this cell was reaped.
    # rc 1 also covers "already gone", which is the ordinary case — so
    # a pid that is not ours any more counts as reaped, and only a pid we
    # still hold and could not signal counts against it.
    local identity_reaped=1
    kill_owned_pid "$pong_pid" "$pong_id" "" KILL
    identity_reaped=$(reap_verdict "$?" "$identity_reaped" "$pong_pid" "$pong_id")
    kill_owned_pid "$ping_pid" "$ping_id" "" KILL
    identity_reaped=$(reap_verdict "$?" "$identity_reaped" "$ping_pid" "$ping_id")
    # The latency node climbed its own INT/TERM/KILL ladder above, so it is
    # not signalled again here — its LAST rung's status is carried down in
    # `latency_kill_rc` instead, and is 1 ("not ours, or already gone") on
    # the ordinary path where the ladder never ran because the node exited.
    # A literal 1 here discarded the one case that matters: a KILL that
    # FAILED on a node still ours, which this then published as a reap.
    identity_reaped=$(reap_verdict "$latency_kill_rc" "$identity_reaped" \
                                   "$latency_pid" "$latency_id")
    reap_bench_binaries "$identity_reaped"
    wait "$latency_pid" 2>/dev/null
    local latency_rc=$?
    [ -n "$pong_pid" ] && wait "$pong_pid" 2>/dev/null
    [ -n "$ping_pid" ] && wait "$ping_pid" 2>/dev/null

    # Propagate the measurement node's OWN exit status. Discarding it made
    # a node-side setup refusal (strict sample-env parsing exits 2) surface
    # later as the generic missing-bin exit 13, pointing the reader at the
    # wrong problem. The timeout path keeps its own rc=1, which is set
    # before the kill and must win over the resulting signal status.
    #
    # A node status is REMAPPED to 3, never passed through as 2: 2 is
    # run_one's OWN setup-refusal code (chrt not permitted), which the
    # caller turns into a whole-sweep `exit 2`. The nodes use 2 for both a
    # genuine env refusal AND a bootstrap-discovery timeout, and a
    # transient timeout must fail its CELL, not abort every remaining size,
    # shm mode and RMW.
    if [ "$rc" -eq 0 ] && [ "$latency_rc" -ne 0 ]; then
        echo "run_bench.sh: latency node exited rc=$latency_rc — see $BIN_LOG" >&2
        rc=3
    fi
    return "$rc"
}

# --- per-size smoke checks (exit-11/12/13/77 contract) -----------------

# exit 11: the message's plainness contradicts its CLASS label.
#   pod   — "Msg::is_plain: 0" means Pod<N> is not trivially copyable:
#           the loaned / CDR-memcpy assumption is broken; every cell of
#           this build is invalid.
#   image — the INVERSE gate: "Msg::is_plain: 1" would mean the cell
#           labeled 'variable / unbounded' ran a plain type — the class
#           hypothesis (unbounded ⇒ no loan, full serialize + delivery
#           memcpy) would be tested against the wrong subject. Expected
#           output for image is is_plain: 0 (has_fixed_size=0).
# Both arms assert POSITIVELY: the expected marker must be PRESENT.
# `grep -q` returns 1 for no-match and 2 for cannot-read, and neither is
# an error here, so a "refuse the wrong marker" test alone treats a log
# that says NOTHING as a log that says the right thing. This gate is the
# only runtime check that the class label matches what the binaries
# actually ran, so absence of evidence is UNVERIFIED, not verified.
#
# Scope, because the obvious example does NOT apply:
# `is_plain_check` predates this axis, so a stale image whose binaries
# predate the type-class DISPATCH still logs the line — and on an image
# cell it logs `is_plain: 1`, which the negative arm above already
# refuses. What the positive arm covers is a log that lost the line: a
# binary older than `is_plain_check` itself, a truncated or rotated log,
# a redirect that dropped a node's stderr, a logging configuration that
# suppressed INFO. In each case the class claim is unbacked and the gate
# cannot tell which cause it is — which is the point.
check_is_plain() {
    local bin_log=$1
    local run_rc=$2
    local want bad
    if [ "$CER_BENCH_MSG" = "image" ]; then
        want='Msg::is_plain: 0'
        bad='Msg::is_plain: 1'
    else
        want='Msg::is_plain: 1'
        bad='Msg::is_plain: 0'
    fi
    if grep -q "$bad" "$bin_log"; then
        echo "run_bench.sh: SMOKE FAIL — '$bad' on a $CER_BENCH_MSG-class cell; see $bin_log" >&2
        echo "  (pod must be trivially copyable; an image cell labeled 'variable' must NOT be plain)" >&2
        exit 11
    fi
    # The marker-present assertion presumes the binaries CONSTRUCTED: every
    # node logs it from a constructor. `run_one` truncates the log before
    # it can refuse, so a chrt refusal (its rc=2) leaves an EMPTY log, and
    # an unconditional assertion here would exit 11 claiming the class is
    # unverified while the real reason — "--cap-add SYS_NICE" — sits one line above,
    # unread. The caller runs these gates before consulting run_rc ON
    # PURPOSE (a structural skip or a broken build is a more precise
    # verdict), so the negative arm above stays unconditional; only this
    # one waits for evidence the run happened. Nothing is lost: a stale
    # image runs to completion at rc=0 and still lacks the line.
    if [ "$run_rc" != "0" ]; then
        return 0
    fi
    if ! grep -q "$want" "$bin_log"; then
        echo "run_bench.sh: SMOKE FAIL — no '$want' line in $bin_log" >&2
        echo "  — the $CER_BENCH_MSG class label is UNVERIFIED, not verified: every node logs this line" >&2
        echo "  from its constructor, so a log without it lost it (truncated, rotated, a dropped stderr" >&2
        echo "  redirect, a suppressed INFO level) or came from binaries older than is_plain_check." >&2
        exit 11
    fi
}

# exit 12: the DMA-lock device is present in this container but the
# lock failed — p99 would silently include C-state exit latency. When
# the device is absent (e.g. host without /dev/cpu_dma_latency) the
# binaries soft-warn and we continue: that run is valid, just
# potentially noisier, and bench.py records the warning in the log.
check_dma_lock() {
    local bin_log=$1
    local run_rc=$2
    [ -e /dev/cpu_dma_latency ] || return 0
    grep -q 'cpu_dma_lock: .* failed' "$bin_log"
    local rc=$?
    if [ "$rc" -eq 0 ]; then
        echo "run_bench.sh: SMOKE FAIL — /dev/cpu_dma_latency present but the DMA lock failed; see $bin_log" >&2
        exit 12
    fi
    # rc 1 = the log says the lock held. rc >= 1 that is NOT 1 means grep
    # could not read the log, and a p99 polluted by C-state exits would
    # then ship under a label the METHODOLOGY says means the lock held.
    # DEFENCE IN DEPTH behind the call site's readability pre-check, and
    # run-gated for the same reason as the two gates above.
    if [ "$rc" -gt 1 ] && [ "$run_rc" = "0" ]; then
        echo "run_bench.sh: SMOKE FAIL — could not read $bin_log (grep exit $rc); the DMA-lock" >&2
        echo "  posture for this size is UNKNOWN, not verified" >&2
        exit 12
    fi
}

# exit 77: structural loan-recv skip. rmw_take_loaned_message is a
# NO-OP RMW_RET_UNSUPPORTED stub in rmw_zenoh (every released distro —
# ros2/rmw_zenoh#175 #893). No retry helps; re-verify on new versions.
# Other RMWs' sentinels fall through (cyclonedds reports can_loan=false
# conservatively even when the take works).
check_structural_loan_skip() {
    local bin_log=$1
    local run_rc=$2
    for stub_rmw in rmw_zenoh_cpp; do
        grep -q "RMW_LOAN_RECV_UNSUPPORTED rmw=$stub_rmw" "$bin_log"
        local rc=$?
        if [ "$rc" -eq 0 ]; then
            echo "run_bench.sh: SKIP — $stub_rmw does not implement rmw_take_loaned_message (structural stub; no retry helps)"
            exit 77
        fi
        # Same rule as the two gates above: an unreadable log is not a
        # "no sentinel found". A real structural skip would then run as a
        # normal cell and fail later on missing samples, blaming the wrong
        # thing.
        # DEFENCE IN DEPTH: the call site owns "the log cannot be read"
        # with a message that names all three unknowns, so this arm is
        # unreachable there. It stays because the gate is also driven
        # standalone (the parity harness extracts and runs it), and a
        # future caller that skipped the pre-check would otherwise read
        # an unreadable log as a clean sentinel scan. Run-gated for the
        # same reason as the pre-check.
        if [ "$rc" -gt 1 ] && [ "$run_rc" = "0" ]; then
            echo "run_bench.sh: SMOKE FAIL — could not read $bin_log (grep exit $rc); the structural" >&2
            echo "  loan-recv sentinel could not be checked" >&2
            exit 11
        fi
    done
}

# --- inline preflight for the ACTIVE rmw ------------------------------

# ROS_SETUP_BASH lets the same script drive any distro image; the
# Dockerfile exports it.
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/${ROS_DISTRO_NAME:-jazzy}/setup.bash}"
if [ ! -f "$ROS_SETUP_BASH" ]; then
    echo "run_bench.sh: ROS setup not found at $ROS_SETUP_BASH" >&2
    exit 2
fi
source_ros "$ROS_SETUP_BASH" || exit 2
export ROS_SETUP_BASH

if [ ! -f /bench/ros2_ws/install/setup.bash ]; then
    echo "run_bench.sh: /bench/ros2_ws/install/setup.bash missing — was the image built? (docker/Dockerfile colcon-builds the ws)" >&2
    exit 2
fi
BENCH_WS_SETUP="/bench/ros2_ws/install/setup.bash"
source_ros "$BENCH_WS_SETUP" || exit 2
export BENCH_WS_SETUP

if [ ! -e /dev/cpu_dma_latency ]; then
    echo "run_bench.sh: note — /dev/cpu_dma_latency not present (pass --device /dev/cpu_dma_latency); p99 may degrade" >&2
fi

# --- sweep -------------------------------------------------------------

START_TS=$(date +%s)
ANY_SIZE_FAILED=0
# Set when a size failed because its DELIVERY receipts could not describe
# its samples. Distinct from ANY_SIZE_FAILED because the two mean opposite
# things to the caller: rc 13 is bench.py's fixed100 CANNOT-SUSTAIN signal
# and drives a rate ladder, but a lower rate fixes nothing here — the cell
# would re-run just as incoherent, a lower rung would eventually "succeed",
# and the run would publish a fabricated rate-limitation claim about a
# stack that had no rate problem. That is strictly worse than the hole this
# gate closes, so the condition gets its OWN exit code rather than a log
# string for the caller to grep.
DELIVERY_INCOHERENT=0

echo "run_bench.sh: rmws=[${RMWS_LIST[*]}] shm=[${SHM_MODES[*]}] chrt=[${CHRT_MODES[*]}] recv=$RECV_PATH qos=$CER_BENCH_QOS msg=$CER_BENCH_MSG pacing=$CER_BENCH_PACING sizes=[${SIZES[*]}]"

for rmw in "${RMWS_LIST[@]}"; do
    echo
    echo "============================================================"
    echo "  RMW: $rmw"
    echo "============================================================"

    case "$rmw" in
        cyclonedds)
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
                echo "run_bench.sh: rmw_cyclonedds_cpp not installed in this image" >&2
                exit 2
            fi
            ;;
        fastdds)
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            if ! ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1; then
                echo "run_bench.sh: rmw_fastrtps_cpp not installed in this image" >&2
                exit 2
            fi
            ;;
        zenoh)
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
                echo "run_bench.sh: rmw_zenoh_cpp not installed in this image (no apt package for this distro?) — zenoh cells cannot run" >&2
                exit 2
            fi
            start_zenohd || exit 2
            ;;
        cerulion)
            export RMW_IMPLEMENTATION=rmw_cerulion
            # NO `ros2 pkg prefix` check, unlike every stock RMW above:
            # rmw_cerulion is not an apt package and has no ament index
            # entry to find. What makes the name resolvable is the prefix
            # ensure_rmw_cerulion stages, so the build IS the availability
            # check: it exits 2 with a specific reason (no source, no
            # cargo, build failed) rather than a generic "not installed".
            ensure_rmw_cerulion
            ;;
        stock)
            # STOCK lane (usage memo §3): the zero-config default. The
            # rmw is rmw_fastrtps_cpp BY DEFINITION — it is ROS 2's
            # default on humble/jazzy/lyrical (docs.ros.org middleware
            # vendors; the Lyrical discourse announcement) — with NO
            # profiles XML and NO transport env (scrubbed loudly in the
            # mode arm below).
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            if ! ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1; then
                echo "run_bench.sh: rmw_fastrtps_cpp not installed in this image" >&2
                exit 2
            fi
            ;;
        composed)
            # COMPOSED lane (usage memo §2): one process, manual
            # composition, intra-process comms per SHM_MODE
            # (ipcon/ipcoff). The underlying rmw is the stock default
            # (rmw_fastrtps_cpp, zero config): with ipc=off every hop
            # rides it; with ipc=on the data path bypasses it (the
            # rclcpp IntraProcessManager pointer-pass).
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            if ! ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1; then
                echo "run_bench.sh: rmw_fastrtps_cpp not installed in this image" >&2
                exit 2
            fi
            if [ ! -x "$INSTALL_LIB/composed_rtt_node" ]; then
                echo "run_bench.sh: $INSTALL_LIB/composed_rtt_node missing — the image predates the composed lane; rebuild it (bench.py --build-image)" >&2
                exit 2
            fi
            ;;
        *)
            echo "run_bench.sh: unknown RMW: $rmw (valid: cyclonedds fastdds zenoh + the lanes stock composed)" >&2
            exit 2
            ;;
    esac

    # Lane ↔ axis cross-checks + the per-rmw shm-mode set. A lane cell's
    # name carries no qos token and no transport label, so any
    # cross-labeled axis here would be a MISLABELED cell — hard error,
    # never a silent remap (the suite-wide rule).
    rmw_shm_modes=("${SHM_MODES[@]}")
    case "$rmw" in
        stock|composed)
            if [ "$RECV_PATH" != "rclcpp" ]; then
                echo "run_bench.sh: RMWS=$rmw requires RECV_PATH=rclcpp — the lane models plain typed-callback usage (memo §1: loan callers are benchmarks/vendor SDKs); got '$RECV_PATH'" >&2
                exit 2
            fi
            if [ "$CER_BENCH_QOS" != "stock" ]; then
                echo "run_bench.sh: RMWS=$rmw requires CER_BENCH_QOS=stock — the lane cell name carries no qos token, so a non-stock QoS would be an unlabeled axis (got '$CER_BENCH_QOS')" >&2
                exit 2
            fi
            if [ "$rmw" = "stock" ]; then
                if [ -n "${SHM_MODE:-}" ] && [ "$SHM_MODE" != "stock" ]; then
                    echo "run_bench.sh: RMWS=stock requires SHM_MODE=stock (got '$SHM_MODE')" >&2
                    exit 2
                fi
                rmw_shm_modes=(stock)
            else
                if [ -n "${SHM_MODE:-}" ]; then
                    case "$SHM_MODE" in
                        ipcon|ipcoff) rmw_shm_modes=("$SHM_MODE") ;;
                        *)
                            echo "run_bench.sh: RMWS=composed requires SHM_MODE=ipcon or ipcoff (got '$SHM_MODE')" >&2
                            exit 2
                            ;;
                    esac
                else
                    rmw_shm_modes=(ipcon ipcoff)
                fi
            fi
            ;;
        cerulion)
            if [ "$CER_BENCH_QOS" = "stock" ]; then
                echo "run_bench.sh: CER_BENCH_QOS=stock is the stock/composed lanes' label - matrix rmw '$rmw' cells carry a qos token and must run be1 or rel10" >&2
                exit 2
            fi
            # rmw_cerulion has exactly ONE data path: iceoryx2 shared
            # memory. There is no UDP/loopback fallback to turn off, so
            # `no_shm` names a configuration that does not exist, and
            # `zc` is Fast DDS DataSharing's lane name. Either would be a
            # cell whose label describes a transport nothing ran;
            # refused, never silently remapped onto `shm` (the suite-wide
            # rule for a mislabeled axis).
            for m in "${rmw_shm_modes[@]}"; do
                case "$m" in
                    shm) ;;
                    no_shm)
                        echo "run_bench.sh: shm mode 'no_shm' does not exist for rmw=cerulion - iceoryx2 shared memory is its only" >&2
                        echo "  data path, so there is nothing to turn off; a 'no_shm' cell would carry a label no run can honour" >&2
                        exit 2
                        ;;
                    *)
                        echo "run_bench.sh: shm mode '$m' is not valid for rmw=cerulion (only 'shm')" >&2
                        exit 2
                        ;;
                esac
            done
            ;;
        *)
            if [ "$CER_BENCH_QOS" = "stock" ]; then
                echo "run_bench.sh: CER_BENCH_QOS=stock is the stock/composed lanes' label — matrix rmw '$rmw' cells carry a qos token and must run be1 or rel10" >&2
                exit 2
            fi
            for m in "${rmw_shm_modes[@]}"; do
                case "$m" in
                    stock|ipcon|ipcoff)
                        echo "run_bench.sh: shm mode '$m' is a lane mode (RMWS=stock / RMWS=composed) — it cannot ride matrix rmw '$rmw'" >&2
                        exit 2
                        ;;
                esac
            done
            ;;
    esac

    for shm in "${rmw_shm_modes[@]}"; do
        echo
        echo "  ---- SHM mode: $shm ----"

        if [ "$shm" = "zc" ] && [ "$rmw" != "fastdds" ]; then
            echo "run_bench.sh: shm mode 'zc' is the FastDDS DataSharing lane — it exists for rmw=fastdds ONLY (got rmw=$rmw)" >&2
            echo "  (cyclonedds' iceoryx path IS its zero-copy mechanism and rides the 'shm' mode; rmw_zenoh implements no" >&2
            echo "   loaned-message/zero-copy API on any released distro — ros2/rmw_zenoh#175 #893)" >&2
            exit 2
        fi

        # Clean per-RMW SHM state before each (rmw, shm) pair — stale
        # segments from a prior batch hang the next discovery.
        clean_shm

        case "$rmw" in
            cyclonedds)
                export CYCLONEDDS_URI="file://$SCRIPT_DIR/configs/cyclonedds_${shm}.xml"
                if [ "$shm" = "shm" ]; then
                    if ! command -v iox-roudi >/dev/null 2>&1; then
                        echo "run_bench.sh: iox-roudi not found — cyclonedds+shm cannot engage SHM (was ros-\$distro-iceoryx-posh installable for this distro?)" >&2
                        exit 2
                    fi
                    start_iox_roudi || exit 2
                else
                    stop_iox_roudi
                fi
                ;;
            fastdds)
                # Both env spellings for cross-distro coverage (Fast
                # DDS 3.x renamed FASTRTPS_ -> FASTDDS_; 2.14 honors
                # only the old spelling, 3.x only the new one).
                if [ "$shm" = "zc" ]; then
                    # DataSharing (true zero-copy) lane —
                    # the rmw_fastrtps README's own
                    # recipe: publisher/subscriber default profiles
                    # carrying <data_sharing><kind>AUTOMATIC</kind>
                    # </data_sharing> + RMW_FASTRTPS_USE_QOS_FROM_XML=1
                    # (rmw_fastrtps forces data_sharing().off() on both
                    # writer and reader on jazzy AND lyrical unless
                    # that env var flips leave_middleware_default_qos).
                    # One profile for ALL sizes, 16 MB included:
                    # DataSharing bypasses the SHM transport segment,
                    # fragmentation, and the >=1MB async FlowController
                    # entirely (the writer history IS the shared
                    # segment), so the per-size segment switch below is
                    # deliberately not applied here.
                    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/configs/fastdds_zc.xml"
                    export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/configs/fastdds_zc.xml"
                    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
                else
                    # Stock-defaults lane: rmw_fastrtps applies
                    # data_sharing().off() — the out-of-box ROS 2
                    # behavior (see configs/fastdds_shm.xml header).
                    # The 16MB-size profile switch happens per size
                    # inside run_one; this sets the batch default.
                    # Unset the QoS-from-XML var explicitly so a stray
                    # inherited value cannot silently turn the stock
                    # lane into the tuned one.
                    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/configs/fastdds_${shm}.xml"
                    export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/configs/fastdds_${shm}.xml"
                    unset RMW_FASTRTPS_USE_QOS_FROM_XML
                fi
                ;;
            zenoh)
                # rmw_zenoh cells run the
                # SHIPPED session config (DEFAULT_RMW_ZENOH_SESSION_
                # CONFIG.json5 — loaded when ZENOH_SESSION_CONFIG_URI
                # is unset) with the README-blessed key override,
                # never a hand-rolled zenoh_{shm,
                # no_shm}.json5 file. A hand-rolled file deviates from the
                # shipped config in ways that are easy to miss: pool
                # 64 MiB (shipped ROS default 48 MiB), message_size_
                # threshold 0 (shipped 512 — the rmw_zenoh 0.10.5
                # README explicitly warns lowering it "could be
                # counter-productive for the latency of small
                # messages"), and — because a custom config file
                # REPLACES the shipped one, with unset keys falling to
                # zenoh built-ins — silently dropped ROS-tuned keys
                # (peer timestamping, open/accept timeouts). Here every
                # key is upstream's: pool 48 MiB, threshold 512 (small
                # payloads riding the network path under an SHM-enabled
                # session is upstream's shipped choice, disclosed in
                # METHODOLOGY), connect.endpoints tcp/localhost:7447
                # (the shipped default, which carries no 15 s
                # bootstrap delay).
                #
                # ONE named deviation rides the override beside the
                # enable key: mode="init" (shipped: lazy). Eager SHM-
                # subsystem init at session open is what makes
                # verify_shm.sh's structural check meaningful (the
                # zenoh_shm init/segment lines appear at open, and a
                # memlock-class provider failure surfaces AT VERIFY
                # TIME instead of as a mid-cell silent TCP fallback —
                # zenoh 1.8's LazyShmProvider logs one error then
                # quietly rides the network). Pro-zenoh if anything:
                # first-touch SHM setup cost moves out of the measured
                # window (warmup would absorb it regardless).
                #
                # The embedded quotes around init are LOAD-BEARING
                # (2026-08-12, measured on all three distro images):
                # the override grammar parses each value as json5, so
                # an UNQUOTED init is REJECTED — zenohc ERROR "Failed
                # to insert value 'init'", rmw_zenoh WARN "Ignore the
                # invalid configuration key-value pair" — and every
                # node silently ran mode: Lazy while the docs claimed
                # init. Bare true IS valid json5, so the enable key
                # needs no quotes. verify_shm.sh exports the SAME
                # string and hard-fails on the rejection signature, so
                # a regression here is caught at verify time.
                if [ "$shm" = "shm" ]; then
                    export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/mode="init"'
                else
                    # Shipped rmw_zenoh default is already
                    # enabled=false; the explicit override keeps the
                    # cell label self-documenting.
                    export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'
                fi
                unset ZENOH_SESSION_CONFIG_URI
                # Env sanitation: at rmw_zenoh 0.10.5 these legacy vars
                # silently OVERRIDE the config's SHM pool/threshold
                # keys. Scrub loudly rather than measure a tuned cell
                # under a shipped-config label.
                for legacy_var in ZENOH_SHM_ALLOC_SIZE ZENOH_SHM_MESSAGE_SIZE_THRESHOLD; do
                    if [ -n "${!legacy_var:-}" ]; then
                        echo "run_bench.sh: WARNING — stray $legacy_var in the environment overrides the zenoh SHM config; unsetting it" >&2
                        unset "$legacy_var"
                    fi
                done
                # Restart rmw_zenohd between zenoh's SHM-on/off modes:
                # SIGKILL'd bench nodes leave ghost subscriptions in the
                # router's discovery state and the next mode's bootstrap
                # hangs on a confused subscription_count. (The restarted
                # router inherits ZENOH_CONFIG_OVERRIDE — harmless: the
                # router is discovery-only for same-host peers, and SHM
                # on the router matters only for routed-out traffic.)
                stop_zenohd
                start_zenohd || exit 2
                ;;
            cerulion)
                # Deliberately empty of transport config, and said out
                # loud rather than left as a fall-through: every other
                # arm here points its RMW at a profile file or a config
                # override, and a silent gap would read as an omission.
                # rmw_cerulion takes none: iceoryx2 is configured by the
                # publisher/subscriber declarations in the type's own
                # schema, not by a file this harness could point at, and
                # it runs no discovery daemon (no RouDi, no router) that
                # would need starting or restarting between modes.
                :
                ;;
            stock|composed)
                # ZERO-CONFIG claim (usage memo §3): these lanes run the
                # middleware exactly as an unconfigured user gets it, so
                # ANY stray transport/profile env would silently turn
                # the lane into a tuned cell under a zero-config label.
                # Scrub loudly (same discipline as the
                # ROS_DISABLE_LOANED_MESSAGES scrub above; each
                # bench.py-launched container starts env-clean, so a hit
                # here means a manual invocation leaked config).
                for cfg_var in FASTRTPS_DEFAULT_PROFILES_FILE \
                               FASTDDS_DEFAULT_PROFILES_FILE \
                               RMW_FASTRTPS_USE_QOS_FROM_XML \
                               CYCLONEDDS_URI \
                               ZENOH_CONFIG_OVERRIDE \
                               ZENOH_SESSION_CONFIG_URI; do
                    if [ -n "${!cfg_var:-}" ]; then
                        echo "run_bench.sh: WARNING — stray $cfg_var='${!cfg_var}' contradicts the $rmw lane's zero-config claim; unsetting it" >&2
                        unset "$cfg_var"
                    fi
                done
                if [ "$rmw" = "composed" ]; then
                    case "$shm" in
                        ipcon)  export CER_BENCH_IPC=on ;;
                        ipcoff) export CER_BENCH_IPC=off ;;
                    esac
                fi
                ;;
        esac

        # SHM-engagement verification is a HARD gate (F1.3): a cell
        # recorded under a transport label verify_shm could not confirm
        # is a mislabeled result — worse than a failed cell. Nonzero
        # here rides bench.py's retry/documented-empty path.
        #
        # EXCEPT the stock/composed lanes: they claim NO transport-
        # engagement label (stock's whole point is "whatever the
        # defaults do"; composed+ipcon bypasses the rmw entirely), so
        # there is nothing for verify_shm to confirm. Each lane cell
        # gets a PROVENANCE NOTE instead, stating what the defaults are
        # and citing the usage memo — absence of a gate is documented,
        # never silent.
        #
        # Which Fast DDS profiles a GATED (rmw, shm) pair will actually
        # MEASURE with. run_one switches the 16 MB fastdds+shm cell to the
        # 256 MiB-segment profile, so verifying only fastdds_shm.xml would
        # leave the profile that cell really uses unverified — a
        # profile-specific failure could then reach a recorded, mislabeled
        # 16 MB result. Verify each profile the sweep will use, once.
        VERIFY_PROFILES=("")
        if [ "$rmw" = "fastdds" ] && [ "$shm" = "shm" ]; then
            WANTS_16MB=0
            WANTS_OTHER=0
            for _s in "${SIZES[@]}"; do
                if [ "$_s" = "16777216" ]; then WANTS_16MB=1; else WANTS_OTHER=1; fi
            done
            VERIFY_PROFILES=()
            [ "$WANTS_OTHER" = "1" ] && VERIFY_PROFILES+=("")
            [ "$WANTS_16MB" = "1" ] &&
                VERIFY_PROFILES+=("$SCRIPT_DIR/configs/fastdds_shm_16mb.xml")
        fi

        # Clear this sweep's per-size artifacts BEFORE ANYTHING ELSE —
        # including verify_shm, which can `exit 13`/`exit 2` and never
        # reach the size loop at all. Cleared later, a transport
        # this run REFUSED to label would leave the previous run's `.bin`
        # sitting in the raw dir, and compile_csv.py globs the dir, so
        # a rejected cell would publish the older measurement as its own.
        # A cell that does not produce data must leave none behind.
        # Under bench.py each payload gets a fresh container, so this
        # is redundant there — but README documents run_bench.sh as a
        # standalone per-cell driver, and on that path `prev_failed=1`
        # `continue`s past every later size, which then keeps the PRIOR
        # run's files. compile_csv.py globs `*.bin` as a row source and
        # backfills the `loaned` column from `_node.log`, so a stale set
        # publishes a previous run's samples and transport label as this
        # one's; `_delivery.txt` and the UNVERIFIED marker mislabel the
        # archived evidence the same way. The `.rate` in the list is
        # belt-and-braces: no ROS 2 node writes one today (only the
        # native bins and run_workspace.sh do), but compile_csv globs
        # `*.rate` per DIRECTORY as an independent row source, so a
        # leftover under this prefix would publish a rate verdict for a
        # size that did not run.
        # Same rule as run_workspace.sh and bench.py's own pre-spawn
        # clears: a size that does not run this time must leave no row.
        for _stale_size in "${SIZES[@]}"; do
            rm -f \
                "$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${_stale_size}.bin" \
                "$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${_stale_size}.rate" \
                "$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${_stale_size}_node.log" \
                "$LOGS_DIR/${CER_BENCH_RAW_NAME}_${_stale_size}_delivery.txt" \
                "$LOGS_DIR/${CER_BENCH_RAW_NAME}_${_stale_size}_SHM_UNVERIFIED"
        done

        SHM_UNVERIFIED=0
        if [ "$rmw" = "stock" ] || [ "$rmw" = "composed" ]; then
            provenance_file="$LOGS_DIR/${CER_BENCH_RAW_NAME}_provenance.txt"
            {
                if [ "$rmw" = "stock" ]; then
                    echo "lane=stock — the ZERO-CONFIG ROS 2 default (what \`ros2 run\` gives you)."
                    echo "transport label: stock (fastdds defaults: UDP+builtin SHM transport, datasharing off)"
                    echo "  - rmw: rmw_fastrtps_cpp (the ROS 2 default on humble/jazzy/lyrical)"
                    echo "  - transports: Fast DDS defaults — UDPv4 + builtin SHM transport (COPY-based,"
                    echo "    not zero-copy); DataSharing OFF at the rmw layer (rmw_fastrtps forces"
                    echo "    data_sharing().off() without RMW_FASTRTPS_USE_QOS_FROM_XML=1)"
                    echo "  - no profiles XML, no transport env (scrubbed loudly if inherited)"
                else
                    echo "lane=composed ($shm) — one process, three manually-composed nodes on one"
                    echo "single-threaded executor; rclcpp intra-process comms=${CER_BENCH_IPC}."
                    echo "  - underlying rmw: rmw_fastrtps_cpp at stock defaults (zero config);"
                    echo "    with ipc=on the pub/sub data path bypasses the rmw entirely"
                    echo "    (rclcpp IntraProcessManager unique_ptr pointer-pass)"
                fi
                echo "  - qos: stock = rmw_qos_profile_default (RELIABLE / VOLATILE / KEEP_LAST(10))"
                echo "  - recv: rclcpp typed callback; publish: plain publish (no loan call sites;"
                echo "    rcl's subscription-loan gate stays at its shipped default)"
                echo "  - NO verify_shm gate: this lane claims no SHM-engagement label — the row's"
                echo "    claim is 'the defaults, whatever they do', which needs no transport proof."
                echo "citations: memo.md §3 (stock) / §2 (composed) — rmw_fastrtps"
                echo "README defaults; Fast DDS transport + data-sharing docs; ros2/rmw qos_profiles.h;"
                echo "rclcpp node_options.hpp (use_intra_process_comms default false); nav2 PR #2750/#5804."
            } > "$provenance_file"
            echo "  lane provenance note -> $(basename "$provenance_file") (no SHM gate: the lane claims no transport label)"
        else
            VERIFY_RC=0
            for _profile in "${VERIFY_PROFILES[@]}"; do
                if [ -n "$_profile" ]; then
                    echo "  verify_shm: profile $(basename "$_profile")"
                fi
                CER_BENCH_FASTDDS_PROFILE="$_profile" \
                    "$SCRIPT_DIR/verify_shm.sh" "$rmw" "$shm" || VERIFY_RC=$?
                if [ "$VERIFY_RC" -ne 0 ]; then break; fi
            done
            if [ "$VERIFY_RC" -eq 2 ]; then
                # verify_shm distinguishes 1 ("SHM did not engage") from 2
                # ("the check could not run at all" — a squatted port, an
                # unreadable RouDi log, a missing profile). The escape
                # hatch below covers an UNPROVEN transport; it must not
                # convert an UNRUN check into a recorded cell whose marker
                # claims only "no SHM evidence" when the truth is "it was never
                # checked, and the router may not have been ours".
                echo "run_bench.sh: verify_shm could not RUN for $rmw/$shm (exit 2) —" >&2
                echo "  a setup problem, not an unproven transport; see its output above." >&2
                echo "  CER_BENCH_ALLOW_UNVERIFIED_SHM does not apply to this case." >&2
                exit 2
            fi
            if [ "$VERIFY_RC" -ne 0 ]; then
                if [ "${CER_BENCH_ALLOW_UNVERIFIED_SHM:-0}" = "1" ]; then
                    SHM_UNVERIFIED=1
                    echo "run_bench.sh: WARNING — verify_shm FAILED for $rmw/$shm but CER_BENCH_ALLOW_UNVERIFIED_SHM=1 is set:" >&2
                    echo "  recording this cell WITHOUT SHM-engagement evidence; every size gets an _SHM_UNVERIFIED marker under $LOGS_DIR" >&2
                else
                    echo "run_bench.sh: verify_shm FAILED for $rmw/$shm — refusing to record a cell whose transport label is unverified" >&2
                    echo "  (fix the transport — see the verify_shm.sh output above — or set CER_BENCH_ALLOW_UNVERIFIED_SHM=1 to record anyway with an explicit _SHM_UNVERIFIED marker per size)" >&2
                    exit 13
                fi
            fi
        fi
        # verify_shm's probe participant leaves SHM segments behind
        # that the first bench run can collide with. Wipe + settle.
        clean_shm
        sleep 1

        for chrt in "${CHRT_MODES[@]}"; do
            prev_failed=0
            for size in "${SIZES[@]}"; do
                if [ "$prev_failed" = "1" ]; then
                    printf "    %s %-6s chrt=%-3s qos=%-5s size=%-9s ... skip (prior size failed)\n" \
                        "$rmw" "$shm" "$chrt" "$CER_BENCH_QOS" "$size"
                    continue
                fi
                if [ "$SHM_UNVERIFIED" = "1" ]; then
                    # Escape-hatch marker (F1.3): every size recorded
                    # under a failed verify_shm carries an explicit
                    # UNVERIFIED marker so nothing downstream can read
                    # the row as a confirmed-SHM result.
                    marker="$LOGS_DIR/${CER_BENCH_RAW_NAME}_${size}_SHM_UNVERIFIED"
                    {
                        echo "verify_shm.sh FAILED for rmw=$rmw mode=$shm before this size ran."
                        echo "CER_BENCH_ALLOW_UNVERIFIED_SHM=1 was set: the row was recorded WITHOUT"
                        echo "SHM-engagement evidence — treat the '$shm' transport label as UNVERIFIED."
                    } > "$marker"
                    echo "run_bench.sh: SHM UNVERIFIED for size=$size — wrote $(basename "$marker")" >&2
                fi
                printf "    %s %-6s chrt=%-3s qos=%-5s size=%-9s ... " \
                    "$rmw" "$shm" "$chrt" "$CER_BENCH_QOS" "$size"
                run_rc=0
                run_one "$rmw" "$shm" "$chrt" "$size" || run_rc=$?

                # The per-size checks run BEFORE any abort: a structural
                # loan skip (exit 77) or a broken build (exit 11/12) is a
                # more precise verdict than a setup abort, and reading them
                # costs nothing. run_one's own setup refusal (rc=2, chrt not
                # permitted) is re-raised after them; a node-side failure
                # arrives as rc=3 and rides the normal per-size FAIL path.
                bin_log="${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${size}_node.log"
                # ONE owner for "the log cannot be read". The three gates
                # below each classify a grep status of >= 2, but they run
                # in a fixed order, so the FIRST would always own the
                # condition and an operator would be told the structural
                # loan-recv sentinel could not be checked — on a lane
                # where that sentinel is usually irrelevant. Say what is
                # actually unknown, once. Gated on the run having
                # succeeded, like the gates themselves: a cell that
                # failed earlier keeps its own verdict.
                if [ "$run_rc" = "0" ] && [ ! -r "$bin_log" ]; then
                    echo "run_bench.sh: SMOKE FAIL — cannot read $(basename "$bin_log") for a run that reported success;" >&2
                    echo "  the class label, the DMA-lock posture and the structural-skip sentinel are ALL unknown for this size" >&2
                    exit 11
                fi
                check_structural_loan_skip "$bin_log" "$run_rc"
                check_is_plain "$bin_log" "$run_rc"
                check_dma_lock "$bin_log" "$run_rc"
                if [ "$run_rc" = "2" ]; then
                    echo   # close the un-newlined progress line
                    exit 2  # setup error (chrt refused) — already logged
                fi

                # Delivery accounting (F1.4): collect every node's
                # DELIVERY line for this size (ping published= / pong
                # echoed= / latency received= kicks_sent= unstamped=
                # nonpositive_rtt=). REPORTED, never gated — be1 loss at
                # large payloads is a finding the analysis surfaces, not a
                # harness failure.
                #
                # unstamped/nonpositive_rtt are echoes that ARRIVED and
                # yielded no sample (see src/sample_gate.hpp). They are
                # not read by any gate below and do not weaken one: they
                # only make the `received >= samples` slack attributable,
                # where it otherwise has no explanation on the artifact.
                delivery_file="$LOGS_DIR/${CER_BENCH_RAW_NAME}_${size}_delivery.txt"
                # Distinguish "no lines" (rc 1) from "could not read the
                # log" (rc >= 2): folding the two records an
                # affirmatively wrong cause for the second case. The log is
                # ours, so grep's own error is the signal — no 2>/dev/null.
                grep -h "DELIVERY role=" "$bin_log" > "$delivery_file"
                delivery_grep_rc=$?
                if [ "$delivery_grep_rc" -gt 1 ]; then
                    echo "could not read $(basename "$bin_log") (grep exit $delivery_grep_rc) — the delivery accounting for this size is UNKNOWN, not absent" > "$delivery_file"
                elif [ "$delivery_grep_rc" -ne 0 ]; then
                    echo "no 'DELIVERY role=' lines in $(basename "$bin_log") — nodes killed before their exit prints?" > "$delivery_file"
                fi

                # COHERENCE gate. The receipts stay REPORTED-not-gated
                # for LOSS — be1 drops at large payloads are a finding the
                # analysis surfaces, and that is unchanged: every relation
                # below is an INEQUALITY in the direction loss actually
                # moves. What is gated is a receipt set that cannot
                # describe the `.bin` beside it.
                #
                # The chain is physical, so its counts are monotone
                # non-increasing:
                #
                #   kicks_sent >= published >= echoed >= received >= samples
                #
                # latency kicks; ping publishes at most once per kick it
                # got; pong echoes at most what ping published; latency
                # receives at most what pong echoed; and it writes one
                # sample per COMPLETED round trip, minus warmup.
                #
                # Snapshot times differ, and for three of the four links
                # that is harmless: the DOWNSTREAM count is snapshotted no
                # later than the upstream one, so it can only be smaller.
                # The exception is `kicks_sent >= published`, whose
                # snapshots are ADVERSELY ordered — latency prints
                # kicks_sent inside finalize() WHILE ping is still draining
                # in-flight kicks, and ping prints seconds later at
                # SIGTERM. That link holds for a different reason: kick
                # PRODUCTION stops at `done_` (the timer is cancelled and
                # every publish_kick() site is !done_-gated), so kicks_sent
                # is already final when it is printed. Anything that kicks
                # AFTER done_ — a bootstrap retry, a drain-the-tail step,
                # or moving the DELIVERY print ahead of the cancel — turns
                # this gate into a false positive that fails healthy runs.
                #
                # Gating only `== 0` and skipping an absent role is not enough:
                # with
                # published=1, echoed=1, received=1 beside a full .bin, or
                # with a role missing entirely, the row claims round trips
                # the receipt set does not establish. A missing set is
                # gated too (the argument against, that a flush race would
                # discard good data, does not hold):
                # the evidence is required,
                # the nodes print their line at their own exit with a
                # bounded grace ahead of the SIGTERM, and "no fake data"
                # means a row that cannot be corroborated is not a row. A
                # systematic flush problem surfaces as a loud failure
                # naming the missing role instead of silently retained
                # samples.
                delivery_contradiction=""
                _d_kicks=""; _d_ping=""; _d_pong=""; _d_recv=""
                if [ "$delivery_grep_rc" -gt 1 ]; then
                    # Not an absence claim: grep itself failed. The artifact
                    # written above says "UNKNOWN, not absent" for exactly
                    # this case, and a FAIL naming the wrong cause would
                    # contradict the file it points the operator at.
                    delivery_contradiction="the node log could not be READ (grep exit $delivery_grep_rc) — the receipts are UNKNOWN, not absent"
                elif [ "$delivery_grep_rc" != "0" ]; then
                    delivery_contradiction="no DELIVERY receipts at all"
                else
                    _d_kicks=$(receipt_count "$delivery_file" latency kicks_sent)
                    _d_ping=$(receipt_count "$delivery_file" ping published)
                    _d_pong=$(receipt_count "$delivery_file" pong echoed)
                    _d_recv=$(receipt_count "$delivery_file" latency received)
                    for _spec in "latency:kicks_sent:$_d_kicks" \
                                 "ping:published:$_d_ping" \
                                 "pong:echoed:$_d_pong" \
                                 "latency:received:$_d_recv"; do
                        _role="${_spec%%:*}"; _rest="${_spec#*:}"
                        _key="${_rest%%:*}"; _val="${_rest#*:}"
                        if [ -z "$_val" ]; then
                            delivery_contradiction="role=$_role receipt is MISSING (no ${_key}= reported)"
                            break
                        fi
                        if [ "$_val" = "?" ]; then
                            delivery_contradiction="role=$_role receipt carries no parseable ${_key}= count"
                            break
                        fi
                        if [ "$_val" -eq 0 ]; then
                            delivery_contradiction="role=$_role reports ${_key}=0"
                            break
                        fi
                    done
                    if [ -z "$delivery_contradiction" ]; then
                        if [ "$_d_ping" -gt "$_d_kicks" ]; then
                            delivery_contradiction="ping published=$_d_ping exceeds latency kicks_sent=$_d_kicks"
                        elif [ "$_d_pong" -gt "$_d_ping" ]; then
                            delivery_contradiction="pong echoed=$_d_pong exceeds ping published=$_d_ping"
                        elif [ "$_d_recv" -gt "$_d_pong" ]; then
                            delivery_contradiction="latency received=$_d_recv exceeds pong echoed=$_d_pong"
                        fi
                    fi
                fi

                bin_path="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${size}.bin"
                if [ "$run_rc" != "0" ]; then
                    prev_failed=1
                    ANY_SIZE_FAILED=1
                    if [ "$run_rc" = "1" ]; then
                        echo "FAIL (timeout after ${BENCH_CELL_TIMEOUT_S:-300}s)"
                    elif [ "$run_rc" = "3" ]; then
                        # run_one propagates the latency node's own
                        # failure (env refusal or bootstrap timeout) rather
                        # than blaming the watchdog for it.
                        echo "FAIL (latency node refused: setup or bootstrap — see $(basename "$bin_log"))"
                    else
                        echo "FAIL (run_one rc=$run_rc)"
                    fi
                elif [ ! -s "$bin_path" ]; then
                    prev_failed=1
                    ANY_SIZE_FAILED=1
                    echo "FAIL (no .bin written — latency_node bootstrap timeout?)"
                else
                    bytes=$(stat -c%s "$bin_path" 2>/dev/null || echo 0)
                    got_samples=$((bytes / 8))
                    if [ "$got_samples" != "$EXPECTED_SAMPLES" ]; then
                        prev_failed=1
                        ANY_SIZE_FAILED=1
                        echo "FAIL (.bin has $got_samples samples, expected $EXPECTED_SAMPLES)"
                    elif [ -z "$delivery_contradiction" ] &&
                         [ "$_d_recv" -lt "$got_samples" ]; then
                        # The last link, checkable only here: the node
                        # writes one sample per COMPLETED round trip, so it
                        # cannot have written more samples than it received
                        # echoes. (received also covers warmup, so the true
                        # relation is received >= samples + warmup; the
                        # weaker form is asserted because warmup is not on
                        # the receipt and a gate must not infer what it
                        # cannot read.)
                        prev_failed=1
                        ANY_SIZE_FAILED=1
                        rm -f "$bin_path"
                        DELIVERY_INCOHERENT=1
                        echo "FAIL (delivery receipts contradict the samples:" \
                             "latency received=$_d_recv is fewer than the" \
                             "$got_samples samples written — see" \
                             "$(basename "$delivery_file"))"
                    elif [ -n "$delivery_contradiction" ]; then
                        prev_failed=1
                        ANY_SIZE_FAILED=1
                        # REMOVE the sample file: compile_csv globs the
                        # raw dir, so a refused row is published anyway
                        # unless the evidence goes with the refusal. Note
                        # the ASYMMETRY with the wrong-sample-count branch
                        # above, which does not rm: that one is defused
                        # downstream, because compile_csv independently
                        # refuses a .bin whose sample count is wrong. It
                        # has no equivalent check for delivery receipts,
                        # which is why these two branches must rm.
                        rm -f "$bin_path"
                        DELIVERY_INCOHERENT=1
                        echo "FAIL (delivery receipts contradict the samples:" \
                             "$delivery_contradiction, yet $got_samples samples" \
                             "were written — see $(basename "$delivery_file"))"
                    else
                        echo "ok ($got_samples samples in $(basename "$bin_path"))"
                    fi
                fi
            done
        done
    done

    case "$rmw" in
        cyclonedds) stop_iox_roudi ;;
        zenoh)      stop_zenohd ;;
    esac
done

ELAPSED=$(( $(date +%s) - START_TS ))
echo
echo "============================================================"
echo "  Done in ${ELAPSED}s — raw .bins under $CER_BENCH_RAW_DUMP_DIR"
echo "============================================================"

# 14 BEFORE 13: an incoherent cell also set ANY_SIZE_FAILED, and the
# caller must not read it as a rate problem.
if [ "$DELIVERY_INCOHERENT" = "1" ]; then
    exit 14
fi
if [ "$ANY_SIZE_FAILED" = "1" ]; then
    exit 13
fi
exit 0
