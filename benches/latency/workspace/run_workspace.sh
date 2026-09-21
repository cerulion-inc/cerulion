#!/usr/bin/env bash
# run_workspace.sh — the PRIMARY benches/latency line: round-trip latency
# through a REAL `cerulion graph run` workspace (ping -> pong -> latency),
# one leg per invocation, sweeping the pinned 10-size payload schedule.
#
# A port + rewrite of
# benches/cerulion_round_trip_quiescent/graph_rtt_bench/run_bench.sh
# (all four hardenings kept: parity rebuild, watchdog, connection-flood
# guard, .bin data-flow + sample-count gates) with the mp_latency runner's
# leg structure + delivery accounting folded in.
#
# NOT A SUPPORTED ENTRY POINT. Running this
# script directly is a DIAGNOSTIC gesture, not a way to produce numbers.
# The supported entry point is `benches/latency/bench.py`, which owns the
# things this script cannot check for itself:
#   - the `CERULION` provenance + freshness refusal, so a campaign can
#     never be measured with a binary from another tree or an older one;
#   - the run manifest, the pacing variant and the payload schedule that
#     make the artifacts comparable;
#   - the raw output directory and raw prefix every artifact is
#     named by.
# Those policies live in ONE place on purpose: a second copy of them in
# shell would be a second thing to keep true, and the two would drift.
# Invoked directly, this script still refuses what it can see (its own env
# contract) and says so where it cannot — it does not silently substitute
# a weaker rule.
#
# LEGS (argv[1], required):
#   split    `cerulion graph run rtt_bench_split --release` — the declared
#            2-group `process_groups:` real-clock multi-process shape
#            (g1={ping} / g2={pong,latency}; the ping -> pong edge is a
#            REAL cross-process iceoryx2 hop under barrier lockstep). THE
#            HEADLINE ROW: it REPLACES the
#            flagless `default` leg, which currently reads ~25.9 µs p50 on
#            the bench machine because the derived process-per-node
#            partition parks on an un-ringable doorbell (the park-wake
#            bug). Split measured 4.92 µs, the
#            representative multi-process number until the park-wake fix lands and the
#            flagless default returns as the headline. Network posture
#            UNCHANGED: no env overrides; the gateway spawns
#            exactly as on every real-clock live run.
#   mono     `cerulion graph run rtt_bench --single-process --release`
#            — the single-process opt-in (one flag, nothing else; the
#            gateway still spawns — it is the product's default network
#            posture and not part of the measured SHM chain).
#   default  REFUSED LOUDLY (exit 2): retired while park-wake
#            inflation distorts it — see the case arm below for the full citation.
#
# PACING (CER_BENCH_PACING, default quiescent):
#   quiescent   per-payload (rate, TOTAL, warmup) from the pinned schedule
#               below; MEASURED = total − warmup is derived here and
#               exported as CER_BENCH_TARGET_SAMPLES (the suite-wide
#               contract: TARGET_SAMPLES always means MEASURED samples —
#               the .bin must hold exactly that many). The ping node's
#               compile-time
#               `#[cerulion_node(period_ms = N)]` attr is sed-rewritten +
#               the ping cdylib rebuilt per size (graph YAML carries no
#               policy override — the macro attr is the ONLY period knob).
#               PACING AUTHORITY: the period attr is only
#               the TICK SOURCE — the ping node gates each PUBLISH on a
#               WALL-clock slot grid from CER_BENCH_TARGET_RATE_HZ. On the
#               mono leg the live RealClock already fires periods at wall
#               intervals so the gate is a pass-through verifier; on the
#               SPLIT leg the multi-process handed-quantum lockstep makes
#               period_ms LOGICAL time (measured 2026-08-15: an ungated
#               split leg free-runs ~920 Hz wall while its labels say
#               100 Hz — every ungated split-leg rate label is false),
#               so the wall gate is what makes the rate label TRUE. Slots
#               the graph fails to reach are SKIPPED (never burst) and
#               counted (slots_skipped= on the ping's RTT_DELIVERY line).
#   fixed100    ONE uniform target rate,
#               100 Hz, at EVERY size — (2100 total, 100 warmup →
#               2000 measured) per size, real clock, same period_ms
#               rewrite mechanism (100 Hz → period_ms=10; every ladder
#               rung divides 1000 exactly, so no quantization skew).
#               A size that cannot SUSTAIN its rung — watchdog fired,
#               missing/short .bin, drop_oldest > 0 in the live-loop
#               delivery telemetry, or > 1% of the window's wall-grid
#               slots skipped (the ping's slots_skipped observable —
#               the native RateLimiter criterion) — steps DOWN the ladder
#               (ladder_for: 100→50→20→<sensor rate when below 20>) and
#               RE-RUNS the size at the next rung. The ACHIEVED rate is
#               recorded in ${RAW_NAME}_<size>.rate beside the .bin
#               (first line: integer Hz, or `did_not_sustain` when the
#               ladder is exhausted — then NO .bin exists and no latency
#               is minted). Hard failures (iceoryx2 connection flood,
#               unexpected exit codes, rebuild failures) still abort the
#               sweep — the ladder only absorbs the cannot-sustain shape.
#   backtoback  mono leg ONLY: `--time-source virtual` — the deterministic
#               UNCAPPED poll loop (the VirtualClock advances exactly 1 ms
#               per step regardless of wall time, so the period_ms=1 ping
#               fires every step at maximum step rate — a genuinely
#               saturating producer). Counts come from
#               CER_BENCH_TARGET_SAMPLES / CER_BENCH_WARMUP (10000/1000
#               default). split × backtoback is a STRUCTURAL SKIP (rc=77):
#               `--time-source virtual` on a `process_groups:` graph
#               dispatches to the supervisor with the time-source IGNORED
#               (real-clock mp has no uncapped mode; the period floor is
#               1 ms), so a "backtoback" split line would really be a
#               1 kHz-capped line masquerading as saturation — we refuse to
#               mint that number rather than mislabel it. Use mono for the
#               saturation variant.
#
# TIMESTAMP MECHANISM (verified against the node source — see
# nodes/ping_node/src/lib.rs): the nodes stamp `self.real_ns()`
# UNCONDITIONALLY — wall CLOCK_MONOTONIC, independent of the runtime's
# clock injection, valid across processes. They do NOT consult
# CER_BENCH_WALL_STAMP (unlike nodes built as record and replay assets); wall samples are
# collected on every leg with no env gate. We still export
# CER_BENCH_WALL_STAMP=1 below for env-contract parity across the suite —
# a documented no-op today, load-bearing if these nodes ever adopt the
# robotics-style deterministic-stamp gating.
#
# REQUIRED ENV (fail-fast, exit 2):
# A RECORDING OF THIS LEG IS EVIDENCE, NOT A REPLAY ORACLE. `graph run
# --record` here produces a replay-GRADE bag (every channel carries a
# schema name and an obtainable definition, so it decodes anywhere), but
# `cerulion bag play <bag> --resim all --verify` exits 6 on it BY DESIGN:
# the ping node reads real_ns() to stamp each frame and to decide, against
# a wall-clock slot grid, whether a tick publishes at all, so a
# re-execution gates different ticks and stamps different bytes. Measured
# both classes, same sitting: rtt_bench_pod and rtt_bench (the incumbent
# variable leg) both exit 6 with the same two diverging edges — this is a
# property of every bench leg, not of the type-class axis. See
# METHODOLOGY.md §17b.
#
#   CER_BENCH_RAW_DUMP_DIR   where <RAW_NAME>_<payloadbytes>.bin lands
#   CER_BENCH_RAW_NAME       the pinned raw_prefix for this leg×chrt cell,
#                            e.g. cerulion_workspace_mono_chrt0 (plot
#                            filenames couple to it)
# OPTIONAL ENV:
#   CER_BENCH_MSG             variable (default) | pod — the TYPE-CLASS
#                             axis (METHODOLOGY § "The type-class
#                             axis"): variable = the incumbent Image
#                             legs (unbounded data loaned per tick,
#                             byte-identical to the pre-axis runner);
#                             pod = the fixed-PodPayload twin chain
#                             (graphs rtt_bench_pod{,_split}; the pod
#                             crates are rebuilt per size with
#                             CER_BENCH_POD_BYTES baking the fixed
#                             array length, and their init verifies
#                             baked-vs-runtime size — loud Err on a
#                             stale artifact, never a mislabel)
#   CER_BENCH_PACING          quiescent (default) | backtoback
#   CER_BENCH_SMOKE_N         override (total, warmup) = (N, max(N/10,1))
#                             keeping the pacing — the smoke-gate knob
#   CER_BENCH_TARGET_SAMPLES  backtoback MEASURED count   (default 10000)
#                             (suite-wide contract: TARGET_SAMPLES means
#                             measured samples in every component; the
#                             quiescent leg ignores the ambient value —
#                             its measured count derives from the pinned
#                             schedule, matching native/src/lib.rs)
#   CER_BENCH_WARMUP          backtoback warmup count     (default 1000)
#   CER_BENCH_PAYLOAD_SIZES   space-separated size override (default: the
#                             pinned 10). Entries are DECIMAL byte counts:
#                             digits only (0x40 / 1e3 / a sign are
#                             refused, exit 2); at most 10 digits (bash
#                             arithmetic is 64-bit and wraps — checked
#                             BEFORE any arithmetic); 1..16777216 (the
#                             pinned sweep's ceiling — schedule + SHM
#                             provisioning end there); a zero-padded
#                             entry is read as decimal with a loud note
#                             (bash's $(( )) would read it as octal). Under
#                             CER_BENCH_MSG=pod every
#                             size must be a multiple of 4 >= 16 — the
#                             repr(C) PodPayloadShm rounds anything else
#                             up, so a misaligned override is REFUSED
#                             (exit 2) before any build, never rounded
#                             into a mislabeled row. Every entry, BOTH
#                             classes, must be one of the ten pinned
#                             sweep sizes: an off-sweep size takes
#                             schedule_for's *) fallback and would be
#                             measured under a schedule no other stack
#                             shares. A value that word-splits to nothing
#                             is refused too (an empty value means "no
#                             override" and runs the full sweep).
#   CER_BENCH_FORCE_RATE_HZ   quiescent-only DIAGNOSTIC: override the
#                             schedule rate for every size, counts/gates
#                             unchanged (how the 2026-08-13/14 flatness
#                             matrices were measured — METHODOLOGY §17).
#                             Recorded in NO artifact a plot can read, so
#                             a forced-rate run's figures carry the
#                             SCHEDULE's rate braces, not this value: a
#                             diagnostic figure, never a citable one.
#                             Labels nothing downstream; a first-class
#                             uniform-rate sweep is CER_BENCH_PACING=
#                             fixed100.
#   CER_BENCH_CHRT=1          wrap the run in `chrt -f 80` (SCHED_FIFO;
#                             direct or passwordless-sudo — hard error if
#                             neither works)
#   CER_BENCH_DMA_LOCK        1 (default) | 0. At 1 the runner exports
#                             CERULION_CPU_DMA_LOCK=1 so the graph
#                             process holds the /dev/cpu_dma_latency
#                             C-state cap for the run (the suite's normal
#                             posture). At 0 the runner UNSETS the var for
#                             the graph, an inherited value included
#                             (`env -u`) — STOCK MODE, what a flagless user
#                             on an untuned host gets: C-state exits are
#                             unmanaged, so wake latency includes deep-idle
#                             exits and the run's tails are NOT comparable
#                             to capped runs (announced loudly at start).
#                             Any other value is an env-contract violation
#                             (exit 2).
#   CERULION                  path to the cerulion binary (default: the
#                             repo's target/release/cerulion, rebuilt here)
#
# Exit codes: 0 ok · 1 hard failure (loud) · 2 env-contract violation
#             (incl. the retired `default` leg refusal) ·
#             77 structural skip (split × backtoback — see below).
#
# Deliberately NO `set -e` — every rc is checked bare so a failed step
# reports loudly instead of aborting mid-cleanup (mp_latency convention).

set -uo pipefail

# ── leg selection ───────────────────────────────────────────────────────────
LEG="${1:-}"
case "$LEG" in
    split|mono) ;;
    default)
        echo "run_workspace.sh: the 'default' leg is RETIRED from the matrix" >&2
        echo "The flagless \`graph run\` shape" >&2
        echo "measured ~25.9 µs p50 in the historical retirement investigation:" >&2
        echo "its derived process-per-node partition parked on an" >&2
        echo "un-ringable doorbell. That historical measurement explains the" >&2
        echo "leg retirement; it is not a current product latency claim. Running it" >&2
        echo "beside 'split' would double-count the mp shape. Use 'split'" >&2
        echo "(declared 2-group process_groups — the representative multi-process" >&2
        echo "row until a later change restores the flagless default as the" >&2
        echo "headline)." >&2
        exit 2
        ;;
    *)
        echo "usage: run_workspace.sh <split|mono>" >&2
        echo "  split = graph run rtt_bench_split (declared 2-group process_groups mp; THE headline row)" >&2
        echo "  mono  = graph run rtt_bench --single-process (one flag, nothing else)" >&2
        exit 2
        ;;
esac

# ── pacing mode ─────────────────────────────────────────────────────────────
PACING="${CER_BENCH_PACING:-quiescent}"
case "$PACING" in
    quiescent|fixed100|backtoback) ;;
    *)
        echo "run_workspace.sh: CER_BENCH_PACING='$PACING' is not a mode" >&2
        echo "  (quiescent | fixed100 | backtoback)" >&2
        exit 2
        ;;
esac

# ── type-class axis (CER_BENCH_MSG — METHODOLOGY § "The type-class axis") ───
# variable (default) = the INCUMBENT legs, byte-identical to the
#   pre-axis runner: sensor_msgs/Image with the unbounded `data` field
#   loaned per tick (graphs rtt_bench / rtt_bench_split; the pinned
#   cerulion_workspace_{split,mono}_chrt{N} prefixes carry NO class
#   token — they predate the axis).
# pod = the FIXED-POD twin: the PodPayload workspace schema whose wire
#   fixed size is exactly the sweep payload (pod_schema/pod_codegen.rs
#   bakes the array length per size via CER_BENCH_POD_BYTES; the pod
#   nodes' init verifies baked-vs-runtime size, so a stale build fails
#   the size loudly instead of mislabeling). Graphs rtt_bench_pod /
#   rtt_bench_pod_split; raw prefixes carry a `_pod` token
#   (cerulion_workspace_{split,mono}_pod_chrt{N} — bench.py mints them).
# Between the two classes the ONLY moving part is the message class, so
# their delta isolates what a variable field costs the loaned-slot
# write path.
MSG_CLASS="${CER_BENCH_MSG:-variable}"
case "$MSG_CLASS" in
    variable|pod) ;;
    *)
        echo "run_workspace.sh: CER_BENCH_MSG='$MSG_CLASS' is not a class" >&2
        echo "  (variable | pod)" >&2
        exit 2
        ;;
esac

# ── structural skips (LOUD, rc=77 — the suite's skip convention) ────────────
if [ "$LEG" = "split" ] && [ "$PACING" = "backtoback" ]; then
    echo "SKIP (rc=77): split × backtoback is structurally unmeasurable —" >&2
    echo "--time-source virtual on a process_groups graph dispatches to the" >&2
    echo "supervisor with the time-source IGNORED (real-clock mp has no uncapped" >&2
    echo "mode; the period floor is 1 ms). A 1 kHz-capped line labeled" >&2
    echo "'backtoback' would misrepresent saturation, so it is not minted." >&2
    exit 77
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# `cerulion graph run` walks up from cwd to find the workspace root — we must
# be inside workspace/ for it to resolve.
cd "$SCRIPT_DIR" || exit 1
# benches/latency/workspace -> repo root is three levels up.
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
# The ping crate whose period_ms attr is sed-rewritten per size — the
# type-class axis selects which chain runs (variable = the incumbent
# Image nodes; pod = the PodPayload twins).
if [ "$MSG_CLASS" = "pod" ]; then
    PING_PKG="pod_ping_node"
else
    PING_PKG="ping_node"
fi
PING_SRC="$SCRIPT_DIR/nodes/$PING_PKG/src/lib.rs"
PING_SRC_BAK="$PING_SRC.orig"

# ── env contract ────────────────────────────────────────────────────────────
if [ -z "${CER_BENCH_RAW_DUMP_DIR:-}" ] || [ -z "${CER_BENCH_RAW_NAME:-}" ]; then
    echo "run_workspace.sh: CER_BENCH_RAW_DUMP_DIR and CER_BENCH_RAW_NAME must be set." >&2
    echo "  The bench writes raw .bin only — the caller (bench.py) wires the dump" >&2
    echo "  location and the pinned raw_prefix (e.g. cerulion_workspace_${LEG}_chrt0)." >&2
    exit 2
fi
# The raw prefix is what compile_csv and plot.py classify on — the class
# is read off the `_pod` token in the stem by plot.py, never from
# CER_BENCH_MSG, and compile_csv keys its rows on the same stem — so
# a run whose name and class disagree publishes one class's numbers under
# the other's label. bench.py always agrees (workspace_raw_name carries
# the token iff msg == pod); a hand-run can disagree, and the README
# documents hand-runs. Two refusals, not one: a name carrying the OTHER
# class's `_pod` token, and — under the pod class — any name that opens
# with the pinned `cerulion_workspace_` prefix but is not the pinned pod
# grammar (plot.py classifies on the LEG token's `_pod` SUFFIX, so
# `cerulion_workspace_pod_split_chrt0` would satisfy a substring test
# here and still classify as VARIABLE there). An ad-hoc name that does
# not open with that prefix claims no class and is left alone.
# The pod accept-arm matches the PINNED grammar, not a bare `_pod`
# substring: plot.py classifies a workspace row by `_WS_RE` + the leg
# token ENDING in `_pod`, so `cerulion_workspace_pod_split_chrt0` would
# satisfy a substring test here and still classify as the VARIABLE class
# there — the leak this gate exists to close, through the gate itself.
case "$MSG_CLASS:$CER_BENCH_RAW_NAME" in
    pod:cerulion_workspace_*_pod_chrt[01]) ;;
    pod:cerulion_workspace_*)
        echo "run_workspace.sh: CER_BENCH_MSG=pod but CER_BENCH_RAW_NAME='$CER_BENCH_RAW_NAME' is not the pinned pod grammar" >&2
        echo "  — the raw prefix is what the CSV and the plots classify on (plot.py reads the LEG token's" >&2
        echo "  '_pod' suffix), so this run would publish pod numbers under a sensor_msgs/Image label." >&2
        echo "  Use: cerulion_workspace_<leg>_pod_chrt<0|1>." >&2
        exit 2
        ;;
    variable:*_pod_*)
        echo "run_workspace.sh: CER_BENCH_MSG=variable but CER_BENCH_RAW_NAME='$CER_BENCH_RAW_NAME' carries the pod class token" >&2
        echo "  — the raw prefix is what the CSV and the plots classify on, so this run would publish" >&2
        echo "  variable-class numbers under a pod label. Drop '_pod' from the name, or set CER_BENCH_MSG=pod." >&2
        exit 2
        ;;
esac

mkdir -p "$CER_BENCH_RAW_DUMP_DIR"
LOG_DIR="$CER_BENCH_RAW_DUMP_DIR/_logs"
mkdir -p "$LOG_DIR"

# ── optional C-state-cap opt-out (stock mode) ───────────────────────────────
# CER_BENCH_DMA_LOCK=0 UNSETS CERULION_CPU_DMA_LOCK for the run (env -u —
# an inherited value included; bench.py exports it on every leg), so the
# graph process never requests the /dev/cpu_dma_latency PM_QoS cap — the
# stock, untuned-host shape. Resolved ONCE here in the fail-fast env-contract
# section (a bad value must die BEFORE the rebuild, and the per-size loop
# below must never silently flip posture mid-sweep).
DMA_LOCK="${CER_BENCH_DMA_LOCK:-1}"
case "$DMA_LOCK" in
    1)
        DMA_ENV=(CERULION_CPU_DMA_LOCK=1)
        ;;
    0)
        # The CONTRACT (METHODOLOGY §11) says a contradictory
        # CERULION_CPU_DMA_LOCK export is REFUSED. bench.py enforces it,
        # and a direct run of this script must not silently
        # resolve the contradiction instead. Enforce it here too: asking
        # for the stock posture while also exporting the cap var is a
        # declaration this runner cannot honour both halves of, and
        # guessing which half the operator meant is exactly the
        # mislabeled-figure hazard the posture knob exists to remove.
        # bench.py cannot trip this: it exports the cap var only when
        # dma_lock_enabled(), so no measured behaviour changes.
        # `+set`, not `-n`: an exported-but-EMPTY CERULION_CPU_DMA_LOCK is
        # still a contradictory declaration, and both other entry points
        # refuse it (bench.py's `is not None`, the native bins' `if let
        # Ok`). METHODOLOGY says all three refuse; make that true.
        if [ "${CERULION_CPU_DMA_LOCK+set}" = "set" ]; then
            echo "run_workspace.sh: CER_BENCH_DMA_LOCK=0 (stock posture) but" >&2
            echo "  CERULION_CPU_DMA_LOCK is also exported (='$CERULION_CPU_DMA_LOCK')" >&2
            echo "  — contradictory C-state posture; unset one. A stock run must" >&2
            echo "  not inherit the cap var." >&2
            exit 2
        fi
        # `env -u` REMOVES an inherited CERULION_CPU_DMA_LOCK. It stays as
        # the mechanism that makes stock mode true BY CONSTRUCTION rather
        # than by the check above having run: an empty DMA_ENV only meant
        # "nothing is set here", so an ambient value survived into the graph
        # and the run was not the stock mode it claims.
        DMA_ENV=(-u CERULION_CPU_DMA_LOCK)
        echo "run_workspace.sh: stock mode — C-state exits unmanaged, tails not comparable to capped runs"
        echo "  (CER_BENCH_DMA_LOCK=0: CERULION_CPU_DMA_LOCK is UNSET for the graph, inherited value included;"
        echo "   no /dev/cpu_dma_latency cap is held)"
        ;;
    *)
        echo "run_workspace.sh: CER_BENCH_DMA_LOCK must be 0 or 1 (got '$DMA_LOCK')" >&2
        exit 2
        ;;
esac

# ── optional per-size usage sidecars (CER_BENCH_USAGE=1; default OFF) ───────
# Samples the run's WHOLE process tree (cerulion supervisor + workers +
# gateway, and the chrt/sudo wrapper when present) via ../usage_sampler.py
# descending from this shell, writing ${RAW_NAME}_<size>.usage.csv beside
# the .bin (CPU %/proc-stat at 5 Hz; RSS + PSS — PSS is the accurate number
# for SHM-heavy multi-process cells). Under fixed100 a re-run at a lower
# ladder rung overwrites the sidecar, so the surviving file describes the
# SAME rung the .bin came from (matching the .rate rule). Resolved once
# here, fail-fast on a bad value (same rule as CER_BENCH_DMA_LOCK).
USAGE_SAMPLER="$SCRIPT_DIR/../usage_sampler.py"
USAGE_ON="${CER_BENCH_USAGE:-0}"
case "$USAGE_ON" in
    0) ;;
    1)
        if [ "$(uname -s)" != "Linux" ]; then
            echo "run_workspace.sh: CER_BENCH_USAGE=1 but usage sampling is /proc-based (Linux-only) —" >&2
            echo "  no .usage.csv sidecars will be written (absent, never fabricated)" >&2
            USAGE_ON=0
        elif [ ! -f "$USAGE_SAMPLER" ]; then
            echo "run_workspace.sh: CER_BENCH_USAGE=1 but $USAGE_SAMPLER is missing" >&2
            exit 2
        else
            echo "run_workspace.sh: CER_BENCH_USAGE=1 — writing per-size .usage.csv sidecars (usage_sampler.py)"
        fi
        ;;
    *)
        echo "run_workspace.sh: CER_BENCH_USAGE must be 0 or 1 (got '$USAGE_ON')" >&2
        exit 2
        ;;
esac
SAMPLER_PID=""
stop_usage_sampler() {
    if [ -n "$SAMPLER_PID" ]; then
        # TERM, a BOUNDED grace, then KILL — bench.py's stop_usage_sampler
        # contract (terminate, wait 10 s, kill, reap), which this caller has
        # to match or the suite has two different cleanup guarantees for the
        # same sampler.
        #
        # A landed TERM does not guarantee the sampler exits: without the
        # bounded poll below, a bare `wait` after it can still block
        # forever if usage_sampler.py is wedged in a syscall or ignoring the
        # signal, and the sampler is a plain `$!` child with no deadline of
        # its own. A hung sidecar then hangs the whole leg.
        if kill -TERM "$SAMPLER_PID" 2>/dev/null; then
            _s_waited=0
            while [ "$_s_waited" -lt 100 ]; do
                kill -0 "$SAMPLER_PID" 2>/dev/null || break
                sleep 0.1
                _s_waited=$((_s_waited + 1))
            done
            if kill -0 "$SAMPLER_PID" 2>/dev/null; then
                echo "warning: the usage sampler (pid $SAMPLER_PID) did not" >&2
                echo "  exit within 10s of SIGTERM; sending SIGKILL. Its" >&2
                echo "  sidecar may be truncated." >&2
                kill -KILL "$SAMPLER_PID" 2>/dev/null || true
            fi
            # Reap either way: the poll above only observes, and an unreaped
            # child stays a zombie holding its pid.
            wait "$SAMPLER_PID" 2>/dev/null || true
        else
            echo "warning: could not signal the usage sampler (pid" >&2
            echo "  $SAMPLER_PID); not waiting on it. Its sidecar may be" >&2
            echo "  short." >&2
        fi
        SAMPLER_PID=""
    fi
}

# CER_BENCH_WALL_STAMP=1 on every workspace leg (env contract). For THESE
# nodes it is a documented NO-OP — they stamp wall real_ns() unconditionally
# (see the header + nodes/ping_node/src/lib.rs) — kept for contract parity
# and future-proofing against a robotics-style deterministic-stamp port.
export CER_BENCH_WALL_STAMP=1

# fd headroom before any iceoryx2 work (fd exhaustion presents as
# ServiceInCorruptedState — see PITFALLS.md). Warn loudly if the host
# refuses; on a Linux bench host this must succeed.
if ! ulimit -n 65536 2>/dev/null; then
    echo "WARNING: ulimit -n 65536 refused (hard limit too low?) — current" >&2
    echo "  limit: $(ulimit -n). Large sweeps may hit fd exhaustion, which" >&2
    echo "  iceoryx2 surfaces as ServiceInCorruptedState." >&2
fi

# ── payload sweep (pinned 10 sizes; env-overridable) ────────────────────────
DEFAULT_SIZES="64 256 1024 4096 16384 65536 262144 1048576 4194304 16777216"
# Word splitting, NOT `read -r -a <<<`: `read` without -d stops at the
# first NEWLINE, so `CER_BENCH_PAYLOAD_SIZES=$(cat sizes.txt)` silently
# dropped every token after line 1 — no note, no refusal, exit 0, and a
# sweep that measured one size while its caller asked for ten. The ROS 2
# runner has always word-split. `set -f` around the expansion because an
# unquoted one also GLOBS: `SIZES='6*'` in a directory holding a file
# named `64` would otherwise expand to a size nobody typed and pass the
# digit gate below. RESTORED, not forced on: `set +f` unconditionally
# ENABLES globbing, so a caller who ran this script under `bash -f`
# would silently get it back for everything after this line.
case $- in *f*) _globwas=off ;; *) _globwas=on ;; esac
set -f
# shellcheck disable=SC2206
SIZES_RAW=(${CER_BENCH_PAYLOAD_SIZES:-$DEFAULT_SIZES})
[ "$_globwas" = on ] && set +f
# Every size token is validated as a DECIMAL integer and canonicalized
# ONCE, for both classes, before anything reads it: bash's $(( )) reads
# a leading zero as OCTAL ('0000030' -> 24, and '0000064' -> 52, which
# the membership gate below would then refuse under a size the user
# never typed) and refuses 08/09 outright ("value too great for base");
# the schedule lookups (schedule_for / ladder_for) are
# string matches that would route '0000064' to the fallback tuple; and
# the .bin / .rate / log names + CER_BENCH_PAYLOAD_SIZE must carry the
# form bench.py expects. `10#` pins base ten; a changed token is noted
# loudly, never silently rewritten. Anything but digits (0x40, 1e3, a
# sign, an empty token) is refused — a size is a byte count.
# Range contract, enforced BEFORE any arithmetic touches the token:
# bash integers are 64-bit and `10#` WRAPS on overflow (measured:
# 18446744073709551680 reads as 64 — a 64-byte row minted under a
# 20-digit label; 2^63 reads negative), so the token LENGTH is bounded
# first (10 digits ⇒ at most 9,999,999,999, far inside 2^63), and only
# the value that survives is compared numerically against the suite's
# ceiling. The ceiling is the pinned sweep's largest size: the
# quiescent schedule, the iceoryx2 pool provisioning and the graphs'
# max-slice-len are laid out for it, and a larger request would run on
# the fallback schedule tuple under an unprovisioned buffer — never a
# citable row. A zero-byte payload is not a sweep point either.
PAYLOAD_CEILING_BYTES=16777216
MAX_SIZE_DIGITS=10
SIZES=()
for tok in "${SIZES_RAW[@]}"; do
    case "$tok" in
        ''|*[!0-9]*)
            echo "run_workspace.sh: CER_BENCH_PAYLOAD_SIZES entry '$tok' is not a decimal integer" >&2
            echo "  (digits only — no 0x prefix, no exponent, no sign; every entry names a byte count)" >&2
            exit 2
            ;;
    esac
    if [ "${#tok}" -gt "$MAX_SIZE_DIGITS" ]; then
        echo "run_workspace.sh: CER_BENCH_PAYLOAD_SIZES entry '$tok' has too many digits (${#tok} > $MAX_SIZE_DIGITS)" >&2
        echo "  — bash arithmetic is 64-bit and wraps silently on overflow (18446744073709551680 would read as 64," >&2
        echo "  minting a 64-byte row under that label); the ceiling is $PAYLOAD_CEILING_BYTES bytes anyway" >&2
        exit 2
    fi
    size=$(( 10#$tok ))
    if [ "$size" -lt 1 ] || [ "$size" -gt "$PAYLOAD_CEILING_BYTES" ]; then
        echo "run_workspace.sh: CER_BENCH_PAYLOAD_SIZES entry '$tok' (= $size) is outside 1..$PAYLOAD_CEILING_BYTES" >&2
        echo "  — the pinned sweep tops out at $PAYLOAD_CEILING_BYTES bytes (schedule + SHM provisioning are laid out" >&2
        echo "  for it); a larger request would run the fallback schedule under an unprovisioned buffer, never a" >&2
        echo "  citable row, and a zero-byte payload is not a sweep point" >&2
        exit 2
    fi
    if [ "$size" != "$tok" ]; then
        echo "run_workspace.sh: note — CER_BENCH_PAYLOAD_SIZES entry '$tok' read as decimal $size" >&2
        echo "  (leading zeros dropped; the schedule lookup, the .bin names and CER_BENCH_PAYLOAD_SIZE use the canonical form)" >&2
    fi
    SIZES+=("$size")
done

# A whitespace-only override splits to ZERO tokens, which would run
# nothing and still exit 0 — an apparently successful sweep with no
# measurement. Refuse that immediately after parsing.
if [ "${#SIZES[@]}" -eq 0 ]; then
    echo "run_workspace.sh: CER_BENCH_PAYLOAD_SIZES is set but splits to no tokens (empty or whitespace only)" >&2
    echo "  — refusing a zero-size sweep: it would measure nothing and still exit 0" >&2
    echo "  valid: $DEFAULT_SIZES" >&2
    exit 2
fi

# Pod-class size contract (METHODOLOGY §18, matched-quantity rule): the
# baked PodPayloadShm is #[repr(C)] with three u32 fields, so its wire
# size is a multiple of 4 — a 65-byte request bakes a 68-byte struct
# (pod_codegen.rs refuses it at build time; the nodes' init guard would
# fail the size at RUN time). Refuse a misaligned override HERE, before
# the release rebuilds, so a bad CER_BENCH_PAYLOAD_SIZES dies in
# milliseconds with the reason instead of minutes later in a cargo log.
# The refusal names the pinned sweep set as well as the alignment rule:
# every workspace row takes exactly those ten sizes (the gate below), so
# a message that named only "multiples of 4" would send the user to a
# second refusal.
if [ "$MSG_CLASS" = "pod" ]; then
    # $size is canonical decimal here (validated + 10#-normalized above),
    # so the modulo can never see an octal reading of a padded token.
    for size in "${SIZES[@]}"; do
        if [ "$size" -lt 13 ] || [ $((size % 4)) -ne 0 ]; then
            echo "run_workspace.sh: pod-class payload size $size is not realizable —" >&2
            echo "  PodPayloadShm is #[repr(C)] with u32 fields, so CER_BENCH_MSG=pod" >&2
            echo "  realizes multiples of 4 only (>= 16); a rounded size would be a" >&2
            echo "  mislabeled row (METHODOLOGY §18, matched-quantity rule), so it is" >&2
            echo "  refused rather than rounded. Every workspace row also takes exactly" >&2
            echo "  the pinned sweep sizes: $DEFAULT_SIZES" >&2
            exit 2
        fi
    done
fi

# Sweep-point contract, class-independent and on the canonicalized
# values: every workspace row takes exactly the ten pinned sizes. An
# unsupported size would silently take schedule_for's `*)` fallback and
# be measured under a schedule no other stack shares — realizable, but
# not comparable, so never a citable row. Same rule the ROS 2 runner and
# bench.py's ambient_payload_restriction enforce.
for _size in "${SIZES[@]}"; do
    _ok=0
    for _valid in $DEFAULT_SIZES; do
        [ "$_size" = "$_valid" ] && _ok=1
    done
    if [ "$_ok" -ne 1 ]; then
        echo "run_workspace.sh: payload size '$_size' is not in the suite's" >&2
        echo "  TEN PINNED SIZES, so no other stack can be compared against" >&2
        echo "  it and the row would answer a question nobody else answers." >&2
        echo "  valid: $DEFAULT_SIZES" >&2
        echo "" >&2
        echo "  This is the ONE inventory across all three stacks:" >&2
        echo "  every cell comparable, parity and plots" >&2
        echo "  keyed on one size list, and a size nobody can compare across" >&2
        echo "  stacks refused rather than measured. It is deliberately" >&2
        echo "  STRICTER than what this graph's variable payloads could" >&2
        echo "  carry on their own — the limit is the CAMPAIGN's, not the" >&2
        echo "  node's. See METHODOLOGY 'ten pinned sizes'." >&2
        exit 2
    fi
done
# Validate the quiescent flatness-diagnostic rate override the same way,
# and for the same reason as the sizes above: it flows into
# `PERIOD_MS=$(( 1000 / RATE_HZ ))` (a non-numeric or zero value is a bash
# arithmetic error naming neither the knob nor the fix) and into
# CER_BENCH_TARGET_RATE_HZ, where a value above 1 GHz floors the ping
# node's wall period to 0 ns. The node refuses that itself, but a knob
# should refuse its own bad input where the user typed it. 1 GHz is the
# same ceiling the ping node, the native RateLimiter and the three ROS 2
# nodes use.
#
# Unlike those ROS 2 checks this is NOT gated on the pacing mode, and the
# difference is deliberate: CER_BENCH_TARGET_RATE_HZ is EXPORTED by runners,
# so a stale inherited value must not refuse a saturation run that never
# reads it — whereas FORCE_RATE_HZ is only ever set by hand for the
# diagnostic, so there is no inherited-value case and a bad one is a typo
# worth refusing wherever the mode lands.
#
# The SHAPE check is `[1-9][0-9]{0,9}`, spelled as two `case` arms, and the
# spelling is load-bearing on both sides:
#
#   * a LEADING ZERO must be refused, not accepted. `$(( 1000 / 010 ))` is
#     125 — bash reads `010` as OCTAL 8 — while the ping node's `parse` and
#     every artifact label read it as decimal 10. Runner and node would pace
#     at different rates under one label, which is this knob's own failure
#     mode. (`008` is worse: bash refuses base-8 `8` outright, mid-sweep.)
#   * LENGTH must be bounded BEFORE any `[ -lt ]`/`[ -gt ]`. `test` returns
#     status 2 with "integer expression expected" on a value wider than
#     int64, and `if` reads that as FALSE — so a numeric range check alone
#     is silently BYPASSED by exactly the enormous values it exists to stop,
#     and `$(( 1000 / X ))` then wraps to 0 and clamps to a 1 kHz run
#     labeled with the typed value.
#
# 10 digits cannot exceed 9999999999, which `[ -gt ]` compares safely; the
# ceiling below then does the real bound.
if [ -n "${CER_BENCH_FORCE_RATE_HZ:-}" ]; then
    case "$CER_BENCH_FORCE_RATE_HZ" in
        [1-9]|[1-9][0-9]|[1-9][0-9][0-9]|[1-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9]|[1-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]) ;;
        *)
            echo "run_workspace.sh: CER_BENCH_FORCE_RATE_HZ='$CER_BENCH_FORCE_RATE_HZ' is not a" >&2
            echo "  positive decimal integer of at most 10 digits (no leading zeros: bash would" >&2
            echo "  read '010' as octal 8 while the node reads decimal 10 — one label, two rates)" >&2
            exit 2 ;;
    esac
    if [ "$CER_BENCH_FORCE_RATE_HZ" -gt 1000000000 ]; then
        echo "run_workspace.sh: CER_BENCH_FORCE_RATE_HZ='$CER_BENCH_FORCE_RATE_HZ' out of range" >&2
        echo "  valid: 1..1000000000 Hz (above 1 GHz the wall period 1e9/rate floors to 0 ns" >&2
        echo "  and the publish gate stops gating, while the run keeps the rate as its label)" >&2
        exit 2
    fi
    # Validated in every mode, CONSUMED in one. Under fixed100 the rate
    # comes from the ladder rung and under backtoback it is 0, so a VALID
    # value is silently discarded there — and a block that refuses bad input
    # in every mode reads as a knob that works in every mode. Say otherwise.
    if [ "$PACING" != "quiescent" ]; then
        echo "run_workspace.sh: note — CER_BENCH_FORCE_RATE_HZ is a" >&2
        echo "  QUIESCENT-ONLY diagnostic; under '$PACING' the rate comes from" >&2
        echo "  the variant, so this value is IGNORED (use --variant fixed100" >&2
        echo "  for a first-class uniform-rate sweep)." >&2
    fi
fi

# Quiescent schedule (rate_hz, TOTAL_iterations, warmup) per payload —
# MUST stay in lockstep in FOUR places: this function (schedule_for),
# ../native/src/lib.rs::quiescent_schedule, ../ros2/run_bench.sh's
# per-payload case statement, and ../bench.py::quiescent_schedule.
# The middle value is TOTAL iterations; MEASURED = total − warmup is
# derived in the sweep loop below and exported as CER_BENCH_TARGET_SAMPLES
# (the suite-wide contract), so all four stacks dump identical per-payload
# measured counts.
# One counter out of the RTT_DELIVERY lines for a role. Whole-token, the
# same discipline as the ROS 2 runner's receipt_count and for the same
# reason: a trailing-`.*` regex read `published=12junk` as a clean 12.
# Four outcomes, kept apart because they mean different things — the
# role's line is ABSENT (empty), the log could not be READ ("?"), the key
# is missing or the value is not all digits ("?"), or the count.
#
# The role can legitimately print MORE THAN ONCE: on the split leg each
# node runs in its own worker and every worker's stdout lands in this one
# log. Identical repeats are one answer; DIFFERING values for one role are
# a contradiction, not a value to pick from, so they fail closed to "?".
delivery_count() {
    local file="$1" role="$2" key="$3" vals grc
    vals=$(grep -h "RTT_DELIVERY role=$role " "$file" 2>/dev/null); grc=$?
    if [ "$grc" -gt 1 ]; then
        printf '?\n'
        return 0
    fi
    [ -n "$vals" ] || return 0
    printf '%s\n' "$vals" | awk -v k="$key" '
        {
            found = 0
            for (i = 1; i <= NF; i++) {
                n = index($i, "=")
                if (n > 0 && substr($i, 1, n - 1) == k) {
                    v = substr($i, n + 1)
                    found = 1
                    break
                }
            }
            if (!found || v !~ /^[0-9]+$/) { bad = 1; exit }
            if (seen && v != last) { bad = 1; exit }
            last = v
            seen = 1
        }
        END { print (bad || !seen) ? "?" : last }'
}

schedule_for() {
    # 4/16 MiB are the tail-resolved extended windows: measured n >= 2000
    # so a single-rep p99 cites at
    # every size (>= 20 tail exceedances).
    case "$1" in
        64|256|1024)  echo "1000 60000 5000" ;;
        4096|16384)   echo "500  30000 2500" ;;
        65536)        echo "200  12000 1000" ;;
        262144)       echo "100  6000  500"  ;;
        1048576)      echo "60   3600  300"  ;;
        4194304)      echo "30   2100  100"  ;;
        16777216)     echo "10   2050  50"   ;;
        *)            echo "10   600   50"   ;;
    esac
}

# fixed100 schedule: ONE uniform tuple at
# EVERY size — rate=100, total=2100, warmup=100 (measured 2000,
# tail-resolved). Lockstep with native/src/lib.rs::FIXED100_{RATE_HZ,
# TOTAL,WARMUP}, bench.py::fixed100_schedule, and ros2/run_bench.sh's
# fixed100 arm.
FIXED100_RATE_HZ=100
FIXED100_TOTAL=2100
FIXED100_WARMUP=100

# fixed100 fallback ladder for one size: the 100 Hz target, then 50,
# then 20, then the size's SENSOR rate (its schedule_for rate) when
# that sits BELOW the 20 Hz rung — strictly descending (today only
# 16 MiB @ 10 Hz gains a 4th rung). Counts stay the uniform FIXED100
# tuple at every rung, so measured = 2000 is rate-independent.
# Lockstep with native/src/lib.rs::fixed100_ladder +
# bench.py::fixed100_ladder.
ladder_for() {
    local sensor
    sensor=$(schedule_for "$1" | awk '{print $1}')
    if [ "$sensor" -lt 20 ]; then
        echo "100 50 20 $sensor"
    else
        echo "100 50 20"
    fi
}

# Backtoback counts (10000 MEASURED + 1000 warmup default).
# CER_BENCH_TARGET_SAMPLES is already MEASURED per the suite-wide contract,
# so it passes through with no derivation.
BB_TARGET="${CER_BENCH_TARGET_SAMPLES:-10000}"
BB_WARMUP="${CER_BENCH_WARMUP:-1000}"

# Smoke override: (total, warmup) := (N, max(N/10, 1)) keeping the pacing —
# mirrors native quiescent_schedule()'s CER_BENCH_SMOKE_N handling.
SMOKE_N="${CER_BENCH_SMOKE_N:-}"
if [ -n "$SMOKE_N" ]; then
    # Same shape check, same two reasons, as CER_BENCH_FORCE_RATE_HZ above:
    # `0100` would derive its counts from OCTAL 64 at `$(( SMOKE_N / 10 ))`
    # below, and a value wider than int64 would slip the `-le` comparison
    # (test returns 2, if reads false) that is the only other guard here.
    case "$SMOKE_N" in
        [1-9]|[1-9][0-9]|[1-9][0-9][0-9]|[1-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9]|[1-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]|\
        [1-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]) ;;
        *)
            echo "run_workspace.sh: CER_BENCH_SMOKE_N='$SMOKE_N' is not a positive decimal" >&2
            echo "  integer of at most 9 digits (no leading zeros — bash would read them octal)" >&2
            exit 2
            ;;
    esac
    if [ "$SMOKE_N" -le 2 ]; then
        echo "run_workspace.sh: CER_BENCH_SMOKE_N must be > 2 (n=$SMOKE_N leaves" >&2
        echo "  too few measurement samples to be a smoke signal)" >&2
        exit 2
    fi
    echo "CER_BENCH_SMOKE_N=$SMOKE_N: overriding (total, warmup) per size; pacing unchanged."
fi

# ── watchdog / flood-guard constants ────────────────────────────────────────
MAX_CONN_FAIL_LINES=16
IOX2_CONN_FAIL_NEEDLE="Unable to establish connection"

# `timeout` is GNU coreutils; macOS ships it only via brew (gtimeout). An
# unwatched run can hang forever on a stall — refuse to run without one.
if command -v timeout >/dev/null 2>&1; then
    TIMEOUT_BIN="timeout"
elif command -v gtimeout >/dev/null 2>&1; then
    TIMEOUT_BIN="gtimeout"
else
    echo "run_workspace.sh: neither 'timeout' nor 'gtimeout' found — the watchdog" >&2
    echo "  cannot be armed and an unwatched hang would stall the sweep forever." >&2
    echo "  Install coreutils (brew install coreutils on macOS)." >&2
    exit 1
fi

# ── iceoryx2 state cleanup (between sizes) ──────────────────────────────────
# The singleton SHM region carries over otherwise. DESTRUCTIVE to any other
# iceoryx2 tenant on the machine — dedicated bench hosts only (pattern from the old
# runners + ros2-bench's clean_iox2_state). Called only between runs, after the
# previous run's processes exited, so it never deletes a live run's state.
clean_iox2_state() {
    rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes 2>/dev/null || true
    # A prior chrt1 cell that ran via the passwordless-sudo chrt fallback
    # leaves these dirs ROOT-owned; the unprivileged rm above then silently
    # no-ops (|| true) and the NEXT cell dies at iceoryx2 node creation
    # ("failed to create iceoryx2 node: InternalError" — measured on box-x86,
    # 2026-08-13: tuned chrt1 sweep followed by a stock chrt0 sweep).
    # Escalate the SAME removal through sudo -n when residue survives and
    # passwordless sudo exists — the exact capability the residue-creating
    # chrt fallback already proved.
    if [ -e /tmp/iceoryx2/services ] || [ -e /tmp/iceoryx2/nodes ]; then
        sudo -n rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes 2>/dev/null || true
    fi
    if [ -e /tmp/iceoryx2/services ] || [ -e /tmp/iceoryx2/nodes ]; then
        echo "!!! clean_iox2_state: stale /tmp/iceoryx2/{services,nodes} survived" >&2
        echo "!!! removal (foreign-owned residue?) — the next cell may fail at" >&2
        echo "!!! iceoryx2 node creation. Remove them as their owner and re-run." >&2
    fi
    if [ -d /dev/shm ]; then
        # find, not a bare glob: an empty match is a clean no-op.
        find /dev/shm -maxdepth 1 -name 'iox2_*' -exec rm -f {} + 2>/dev/null || true
        if [ -n "$(find /dev/shm -maxdepth 1 -name 'iox2_*' -print -quit 2>/dev/null)" ]; then
            sudo -n find /dev/shm -maxdepth 1 -name 'iox2_*' -exec rm -f {} + 2>/dev/null || true
        fi
    fi
}

# ── hardening 1: rebuild-ALWAYS for binary<->cdylib iceoryx2 parity ─────────
# A stale binary voided a whole aarch64 result set once (mp_latency README);
# a binary<->cdylib iceoryx2 patch skew breaks the SHM event protocol ->
# connection flood -> ZERO data. Rebuild BOTH from THIS checkout, abort loud
# on failure (no `set -e` — guard explicitly).
# The login gate is on in every build, and this script runs the binary
# directly, so it does not pick up the workspace cargo configuration. This is
# how this repository's own runs pass the gate without an account.
export CERULION_LOGIN_GATE=off
CERULION="${CERULION:-$REPO_ROOT/target/release/cerulion}"
echo "Rebuilding cerulion binary + workspace cdylibs (release) for iceoryx2 parity..."
if ! ( cd "$REPO_ROOT" && cargo build -p cerulion_cli --release ); then
    echo "!!! pre-sweep cerulion_cli rebuild failed — aborting (would run stale binary)" >&2
    exit 1
fi
if ! ( cd "$SCRIPT_DIR" && cargo build --release ); then
    echo "!!! pre-sweep cdylib rebuild failed — aborting (would run stale cdylibs)" >&2
    exit 1
fi
# Freshest-wins guard (mp_latency convention): the CLI's cdylib resolution
# is freshest-wins, so a stale debug artifact must never shadow release.
rm -rf "$SCRIPT_DIR/target/debug"
if [ ! -x "$CERULION" ]; then
    echo "!!! cerulion binary not executable at $CERULION after rebuild" >&2
    exit 1
fi

# ── optional chrt wrap ──────────────────────────────────────────────────────
CHRT_PREFIX=()
if [ "${CER_BENCH_CHRT:-0}" = "1" ]; then
    if chrt -f 80 /bin/true >/dev/null 2>&1; then
        CHRT_PREFIX=(chrt -f 80)
    elif sudo -n chrt -f 80 /bin/true >/dev/null 2>&1; then
        CHRT_PREFIX=(sudo -nE chrt -f 80)
    else
        echo "run_workspace.sh: CER_BENCH_CHRT=1 set but chrt -f 80 not allowed for $(id -un)" >&2
        echo "  Add rtprio to /etc/security/limits.conf or configure passwordless sudo." >&2
        exit 1
    fi
fi

# ── leg -> graph + flags ────────────────────────────────────────────────────
# The row label IS the invocation. split = the declared `process_groups:`
# graph, zero flags (real-clock mp; the run spawns the gateway — no env
# overrides). mono = the same node chain in one process via exactly
# `--single-process`. The type-class axis picks the graph pair: the pod
# graphs are faithful wiring mirrors of the variable pair with only the
# node types + schema swapped (see the graphs' provenance headers).
GRAPH="rtt_bench"
[ "$MSG_CLASS" = "pod" ] && GRAPH="rtt_bench_pod"
LEG_FLAGS=(--single-process)
if [ "$LEG" = "split" ]; then
    GRAPH="${GRAPH}_split"
    LEG_FLAGS=()
fi
PACING_FLAGS=()
if [ "$PACING" = "backtoback" ]; then
    # Deterministic UNCAPPED poll loop (mono only — split×backtoback was
    # skipped above). The period_ms=1 ping fires every 1 ms-virtual step at
    # max wall step rate; the nodes' unconditional real_ns() stamps keep the
    # measured RTT real wall time.
    PACING_FLAGS=(--time-source virtual)
fi

# ── tracked-file rewrites: backup / self-healing restore ────────────────────
# ONE tracked file is rewritten per sweep size: the ping crate's period_ms
# attr. It gets the treatment that is the reason a runner may rewrite
# tracked source at all — the checkout is returned to its committed state
# on EVERY exit path.
#
# The pod class's registered `PodPayload.msg` is NOT a second one, although
# its array length describes a wire layout and the layout is per sweep size.
# THIS SCRIPT DOES NOT RECORD — the gate a sweep actually hits is `graph
# validate`'s fail-closed check that `schema:` names something the workspace
# can resolve, which the committed file satisfies at every size. Only a
# RECORDER that took a bag channel's fixed size from that file would need
# the file to track the build, and that resolution lives on the producing
# node itself (`OutputMeta::wire_fixed_size`, beside the wire hash the
# recorder takes from the node), so the registration is a NAME
# registration only — committed once at the first pinned size, read by
# `graph validate`, never written by a sweep.
# `check_percentile_parity.py::check_pod_schema_is_never_rewritten` is what
# holds that: it fails if this script grows a writer for it.
#
# Expect one schema-hash divergence warn per pod topic at
# every size but the committed one (the file and the baked type really do
# hash differently) — expected, harmless, read by no gate. See the
# `.msg` header for which warn is which.
#
# Recover a leftover backup FIRST: a prior hard kill (watchdog SIGKILL) can
# leave the file rewritten with the pristine copy in .orig. Restore before
# taking a fresh backup so a modified file is never clobbered INTO .orig.
if [ -f "$PING_SRC_BAK" ]; then
    mv -f "$PING_SRC_BAK" "$PING_SRC"
fi
cp -f "$PING_SRC" "$PING_SRC_BAK"
trap 'stop_usage_sampler;
      mv -f "$PING_SRC_BAK" "$PING_SRC" 2>/dev/null || true' EXIT INT TERM

# Portable file size (stat -c is GNU, stat -f is BSD — wc -c is both).
file_size() { wc -c <"$1" 2>/dev/null | tr -d '[:space:]'; }

# ── one (size, rate) run + gates ────────────────────────────────────────────
# The whole "announce → rewrite period → rebuild ping → run under the
# watchdog → gates → delivery accounting" body for ONE rate. Inputs
# (globals, set by the sweep loop): SIZE RATE_HZ MEASURED WARMUP PERIOD_MS
# EFFECTIVE_HZ WATCHDOG_SECS RUN_LOG_SUFFIX.
# Returns 0 = ok (bin present, exact count, and — fixed100 — zero
# drop_oldest). Returns 10 = NOT SUSTAINED, fixed100 ONLY (watchdog
# fired / no .bin / short .bin / drop_oldest > 0) — the caller steps down
# the fallback ladder. Every HARD failure (connection flood, unexpected
# exit codes, sed/rebuild failures, an over-count .bin) exits 1 directly
# in EVERY pacing mode: the ladder absorbs only the cannot-sustain shape.
run_size_once() {
    echo ""
    echo "========================================"
    if [ "$PACING" = "backtoback" ]; then
        echo "  payload=$SIZE  backtoback(virtual)  measured=$MEASURED  warmup=$WARMUP  watchdog=${WATCHDOG_SECS}s"
    else
        # effective= is the integer-period-quantized rate the ping node
        # actually ticks at (see the PERIOD_MS comment in the sweep loop).
        echo "  payload=$SIZE  pacing=$PACING rate=${RATE_HZ}Hz  effective=${EFFECTIVE_HZ}Hz (period_ms=${PERIOD_MS})  measured=$MEASURED  warmup=$WARMUP  watchdog=${WATCHDOG_SECS}s"
    fi
    echo "========================================"

    # Bake the per-payload period into the ping cdylib: rewrite the macro
    # attr FROM the pristine backup (rewrites never compound), rebuild ONLY
    # the ping crate — plus, on the pod class, the two other pod crates,
    # whose build scripts bake the PodPayload fixed-array length from
    # CER_BENCH_POD_BYTES (a fixed array is compile-time — the class
    # distinction itself; the nodes' init verifies baked-vs-runtime size
    # so a stale artifact fails the size loudly, never mislabels).
    # `[0-9][0-9]*` is POSIX BRE (the old tree's `\+` was GNU-only and
    # silently non-portable to BSD sed).
    sed -e "s/\(#\[cerulion_node(period_ms = \)[0-9][0-9]*/\1$PERIOD_MS/" \
        "$PING_SRC_BAK" > "$PING_SRC"
    # Fail LOUD if the rewrite did not land — a silently-unchanged period is
    # the dead-mechanism bug this guard exists to kill.
    if ! grep -q "period_ms = $PERIOD_MS)" "$PING_SRC"; then
        echo "" >&2
        echo "!!! payload=$SIZE: failed to rewrite period_ms=$PERIOD_MS into" >&2
        echo "!!! $PING_SRC — the ping macro attr did not match the sed. The" >&2
        echo "!!! per-size pacing would be silently wrong; aborting." >&2
        exit 1
    fi
    if [ "$MSG_CLASS" = "pod" ]; then
        # No registration rewrite is needed here: the committed
        # `PodPayload.msg` registers the NAME (which is what `graph
        # validate`'s fail-closed schema gate asks for), and the LAYOUT a
        # recording stamps into its bag channel comes from the node's own
        # `OutputMeta` — the type baked by the rebuild below. The two
        # cannot be a size out of step, because only one of them
        # describes the wire.
        if ! ( cd "$SCRIPT_DIR" && CER_BENCH_POD_BYTES="$SIZE" cargo build --release \
                   -p pod_ping_node -p pod_pong_node -p pod_latency_node ); then
            echo "" >&2
            echo "!!! payload=$SIZE: pod node rebuild (period_ms=$PERIOD_MS," >&2
            echo "!!! CER_BENCH_POD_BYTES=$SIZE) failed." >&2
            exit 1
        fi
    elif ! ( cd "$SCRIPT_DIR" && cargo build --release -p ping_node ); then
        echo "" >&2
        echo "!!! payload=$SIZE: ping_node rebuild for period_ms=$PERIOD_MS failed." >&2
        exit 1
    fi

    clean_iox2_state

    # Clear any STALE .bin from a prior sweep BEFORE this run so the
    # existence + sample-count gates below are load-bearing (the dump dir
    # persists across runs; a leftover dump would mask a 0-data run).
    BIN_PATH="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${SIZE}.bin"
    rm -f "$BIN_PATH"

    # Per-rung log suffix (fixed100): each ladder rung keeps its own log,
    # so a failed rung's evidence survives the re-run at the next rung.
    RUN_LOG="$LOG_DIR/${CER_BENCH_RAW_NAME}_${SIZE}${RUN_LOG_SUFFIX}.log"
    # The benchmarked command. `env` re-injects EVERY needed var AFTER any
    # chrt/sudo prefix so sudo's env-stripping cannot drop them.
    # NO CERULION_NETWORK override: the network
    # gateway is the product's default posture on BOTH legs — it is a
    # separate process that parks at zero demand and is not part of the
    # measured SHM chain; suppressing it would measure a shape no flagless
    # user gets. The env below is measurement INSTRUMENTATION only, never
    # run SHAPE. IOX2_LOG_LEVEL=error keeps the connection-flood needle at
    # a surfaced level for the guard below. DMA_ENV is the ONE deliberate
    # host-tuning toggle (CER_BENCH_DMA_LOCK, resolved once above): present
    # = the graph holds the C-state cap; absent = stock mode.
    # CER_BENCH_TARGET_SAMPLES = MEASURED (G1): the latency node collects
    # measured + warmup total, drains the warmup prefix, and dumps EXACTLY
    # measured samples — the equality gate below depends on this.
    RUN_CMD=(env
        # FIRST: `env` parses options only before the first name=value
        # operand, so the `-u` form above must precede every assignment.
        ${DMA_ENV[@]+"${DMA_ENV[@]}"}
        CER_BENCH_PAYLOAD_SIZE="$SIZE"
        CER_BENCH_TARGET_SAMPLES="$MEASURED"
        CER_BENCH_WARMUP="$WARMUP"
        CER_BENCH_TARGET_RATE_HZ="$RATE_HZ"
        CER_BENCH_LEG="$LEG"
        CER_BENCH_WALL_STAMP=1
        CER_BENCH_RAW_DUMP_DIR="$CER_BENCH_RAW_DUMP_DIR"
        CER_BENCH_RAW_NAME="$CER_BENCH_RAW_NAME"
        IOX2_LOG_LEVEL=error
        "$CERULION" graph run "$GRAPH" --release "${LEG_FLAGS[@]}" "${PACING_FLAGS[@]}")

    # Usage sidecar (CER_BENCH_USAGE=1): start the sampler right before the
    # run so the ONLY descendants of this shell in its window are the
    # timeout/chrt/cerulion tree (the ping rebuild above is excluded); it
    # rescans the tree every tick, so workers spawned mid-run are captured.
    # rm -f runs in EVERY mode: a prior usage run's sidecar must never sit
    # beside this run's fresh .bin claiming to describe it.
    USAGE_PATH="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${SIZE}.usage.csv"
    rm -f "$USAGE_PATH"
    if [ "$USAGE_ON" = "1" ]; then
        python3 "$USAGE_SAMPLER" --out "$USAGE_PATH" --parent-pid $$ --descend \
            --label "leg=$LEG msg=$MSG_CLASS size=$SIZE pacing=$PACING rate_hz=${RATE_HZ:-}" \
            2>"$LOG_DIR/${CER_BENCH_RAW_NAME}_${SIZE}${RUN_LOG_SUFFIX}.sampler.log" &
        SAMPLER_PID=$!
    fi

    # Hardening 2: watchdog, with a SIGKILL 10 s after SIGTERM in
    # case the child ignores TERM. When chrt runs via passwordless sudo the
    # child is ROOT, so an UNPRIVILEGED timeout's signal would be EPERM —
    # run timeout ITSELF under sudo (with -s KILL) in that case. Both
    # branches return 124 on kill, preserving the sentinel below. The full
    # run log (stdout+stderr) is kept in $RUN_LOG for the flood guard +
    # delivery accounting + post-hoc audit.
    if [ "${#CHRT_PREFIX[@]}" -gt 0 ] && [ "${CHRT_PREFIX[0]}" = "sudo" ]; then
        sudo -nE "$TIMEOUT_BIN" -s KILL --kill-after=10 "$WATCHDOG_SECS" \
            chrt -f 80 "${RUN_CMD[@]}" \
            > "$RUN_LOG" 2>&1
        RUN_STATUS=$?
    else
        "$TIMEOUT_BIN" --kill-after=10 "$WATCHDOG_SECS" "${CHRT_PREFIX[@]}" \
            "${RUN_CMD[@]}" \
            > "$RUN_LOG" 2>&1
        RUN_STATUS=$?
    fi

    stop_usage_sampler

    tail -3 "$RUN_LOG" || true

    # Count connection-failure lines. `grep -c` exits 0 (matches), 1 (no
    # match), or >=2 (a real error, e.g. unreadable file). Map ONLY exit 1
    # to zero; surface a genuine grep error loudly rather than passing it
    # off as "no flood".
    if CONN_FAIL_LINES=$(grep -c "$IOX2_CONN_FAIL_NEEDLE" "$RUN_LOG"); then
        :
    else
        GREP_STATUS=$?
        if [ "$GREP_STATUS" -gt 1 ]; then
            echo "" >&2
            echo "!!! payload=$SIZE: flood-guard grep errored (exit $GREP_STATUS)" >&2
            echo "!!! reading $RUN_LOG — cannot verify connection health. Aborting." >&2
            exit 1
        fi
        CONN_FAIL_LINES=0
    fi

    # Hardening 3: connection-flood guard — checked BEFORE the watchdog
    # verdict so a version-skew flood always reads as the skew it is
    # (and, under fixed100, can never masquerade as a cannot-sustain
    # rung and walk the ladder). Hard abort in EVERY pacing mode.
    if [ "$CONN_FAIL_LINES" -gt "$MAX_CONN_FAIL_LINES" ]; then
        echo "" >&2
        echo "!!! FLOOD GUARD: payload=$SIZE — $CONN_FAIL_LINES \"$IOX2_CONN_FAIL_NEEDLE\"" >&2
        echo "!!! lines in the run log (> $MAX_CONN_FAIL_LINES). This is the iceoryx2" >&2
        echo "!!! binary<->cdylib version-skew flood: the SHM event protocol is broken" >&2
        echo "!!! and no data flows. The rebuild step should prevent this; if it" >&2
        echo "!!! persists, workspace/Cargo.lock disagrees with the repo lockfile" >&2
        echo "!!! on the iceoryx2 family (must be =0.9.1 everywhere)." >&2
        exit 1
    fi

    # Hardening 2: `timeout` returns 124 when it kills. Under fixed100
    # a watchdog kill IS the cannot-sustain signal (the graph did not
    # reach its sample target within 2x the rung's nominal window) —
    # the caller steps down the ladder. Every other pacing mode keeps
    # the hard abort.
    if [ "$RUN_STATUS" -eq 124 ]; then
        if [ "$PACING" = "fixed100" ]; then
            echo "" >&2
            echo "!!! fixed100: payload=$SIZE watchdog (${WATCHDOG_SECS}s) fired at ${RATE_HZ} Hz —" >&2
            echo "!!! the graph did not reach its sample target at this rung (log: $RUN_LOG)." >&2
            return 10
        fi
        echo "" >&2
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" >&2
        echo "!!! WATCHDOG FIRED: payload=$SIZE exceeded ${WATCHDOG_SECS}s — killed." >&2
        echo "!!! The graph never reached its sample target (no data flowed /" >&2
        echo "!!! chain stalled). iceoryx2 \"$IOX2_CONN_FAIL_NEEDLE\" lines: $CONN_FAIL_LINES." >&2
        echo "!!! A large count is the binary<->cdylib iceoryx2 version skew;" >&2
        echo "!!! the rebuild step above is meant to keep them in lockstep." >&2
        echo "!!! Full log: $RUN_LOG" >&2
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" >&2
        exit 1
    fi

    # Any other non-zero exit is a hard failure — surface it loudly.
    if [ "$RUN_STATUS" -ne 0 ]; then
        echo "" >&2
        echo "!!! payload=$SIZE: 'cerulion graph run $GRAPH' exited $RUN_STATUS" >&2
        echo "!!! (not a watchdog kill) — a hard failure. Log tail:" >&2
        tail -30 "$RUN_LOG" >&2
        exit 1
    fi

    # Hardening 4a: data-flow gate — a clean exit that wrote no .bin means
    # samples never flowed (a stall/skew that let the process exit on its
    # own before hitting the sample target). BIN_PATH was computed + cleared
    # before the run, so this check is load-bearing. Under fixed100 this is
    # a cannot-sustain shape (the chain never delivered at this rung) —
    # step down; every other mode aborts.
    if [ ! -f "$BIN_PATH" ]; then
        echo "" >&2
        echo "!!! DATA-FLOW GATE: payload=$SIZE exited cleanly but wrote no" >&2
        echo "!!! $BIN_PATH — no samples flowed through the graph. Log tail:" >&2
        tail -30 "$RUN_LOG" >&2
        if [ "$PACING" = "fixed100" ]; then
            return 10
        fi
        exit 1
    fi

    # Hardening 4b: sample-count gate — the .bin holds raw u64 LE samples
    # (8 B each), and the latency node dumps EXACTLY MEASURED samples (it
    # caps accumulation at measured + warmup and drains the warmup prefix),
    # so this is an EQUALITY check on the measured count (G1). A shortfall
    # is a partial dump (garbage percentiles) — under fixed100 that is the
    # cannot-sustain shape (step down); elsewhere it aborts. An excess is a
    # node-semantics regression (samples collected past the window) and
    # aborts in EVERY mode.
    SAMPLES_IN_BIN=$(( $(file_size "$BIN_PATH") / 8 ))
    if [ "$SAMPLES_IN_BIN" -ne "$MEASURED" ]; then
        echo "" >&2
        echo "!!! DATA-FLOW GATE: payload=$SIZE wrote $SAMPLES_IN_BIN samples in" >&2
        echo "!!! $BIN_PATH (expected EXACTLY measured=$MEASURED). Under-count =" >&2
        echo "!!! partial dump; over-count = the node collected past the window." >&2
        if [ "$PACING" = "fixed100" ] && [ "$SAMPLES_IN_BIN" -lt "$MEASURED" ]; then
            echo "!!! fixed100: under-count = this rung was not sustained." >&2
            return 10
        fi
        echo "!!! Either way the percentiles would be wrong. Aborting." >&2
        exit 1
    fi

    # ── delivery-receipt COHERENCE gate ────────────────────────────────
    # The ROS 2 runner requires the same gate. Without it this leg only
    # PRINTS its receipts and proceeds, so a missing or contradictory
    # one still produces a CSV row. Same rule, same fail-closed shape.
    #
    # The chain here is ping -> pong -> latency, and it is monotone
    # non-increasing:
    #
    #   published >= forwarded >= received >= measured
    #
    # ping increments `published` once per tick as a PERIOD node; pong is
    # a data-trigger consumer of ping's output and increments `forwarded`
    # once per tick; latency is a data-trigger consumer of pong's and
    # increments `received_total` once per tick, then either records a
    # sample or counts a skip, capped at the window. So every downstream
    # counter only ever moves inside a callback the upstream drove.
    #
    # Every relation is an INEQUALITY in the direction loss moves, which
    # is what keeps this compatible with the leg's own design: the header
    # above says deficits are "explained by data-trigger latest-wins
    # coalescing, never silent loss", and coalescing only ever LOWERS a
    # downstream count. What is rejected is a receipt set that cannot
    # describe the .bin beside it — `forwarded` above `published`, a role
    # that never reported, a value that will not parse, or a `measured`
    # larger than the echoes that could have produced it.
    #
    # The backtoback leg is EXEMPT for a stated reason, not an oversight:
    # it runs `--time-source virtual` with no live loop, so the nodes
    # print no RTT_DELIVERY lines at all (the header above says so), and
    # requiring them there would refuse every backtoback row.
    if [ "$PACING" != "backtoback" ]; then
        _d_pub=$(delivery_count "$RUN_LOG" ping published)
        _d_fwd=$(delivery_count "$RUN_LOG" pong forwarded)
        _d_rcv=$(delivery_count "$RUN_LOG" latency received)
        DELIVERY_BAD=""
        for _spec in "ping:published:$_d_pub" "pong:forwarded:$_d_fwd" \
                     "latency:received:$_d_rcv"; do
            _role="${_spec%%:*}"; _rest="${_spec#*:}"
            _key="${_rest%%:*}"; _val="${_rest#*:}"
            if [ -z "$_val" ]; then
                DELIVERY_BAD="role=$_role receipt is MISSING (no ${_key}= reported)"
                break
            fi
            if [ "$_val" = "?" ]; then
                DELIVERY_BAD="role=$_role receipt carries no single parseable ${_key}= count"
                break
            fi
            if [ "$_val" -eq 0 ]; then
                DELIVERY_BAD="role=$_role reports ${_key}=0"
                break
            fi
        done
        if [ -z "$DELIVERY_BAD" ]; then
            if [ "$_d_fwd" -gt "$_d_pub" ]; then
                DELIVERY_BAD="pong forwarded=$_d_fwd exceeds ping published=$_d_pub"
            elif [ "$_d_rcv" -gt "$_d_fwd" ]; then
                DELIVERY_BAD="latency received=$_d_rcv exceeds pong forwarded=$_d_fwd"
            elif [ "$_d_rcv" -lt "$SAMPLES_IN_BIN" ]; then
                DELIVERY_BAD="latency received=$_d_rcv is fewer than the $SAMPLES_IN_BIN samples written"
            fi
        fi
        if [ -n "$DELIVERY_BAD" ]; then
            echo "" >&2
            echo "!!! DELIVERY GATE: payload=$SIZE — the RTT_DELIVERY receipts" >&2
            echo "!!! cannot describe the samples beside them: $DELIVERY_BAD." >&2
            echo "!!! Removing $BIN_PATH: compile_csv globs the raw dir, so a" >&2
            echo "!!! refused row is published anyway if its evidence stays." >&2
            # Same as the ROS 2 lane: the .bin goes with the refusal.
            rm -f "$BIN_PATH"
            exit 1
        fi
    fi

    # fixed100 sustain gate 3: drop_oldest evictions. The live-loop
    # delivery telemetry prints each data-trigger input's drop_oldest
    # counter at run_live exit; ANY eviction means a consumer could not
    # keep up at this rung — latest-wins coalescing would mint a
    # latency line that silently dropped frames at the target rate, so
    # the rung is NOT sustained (the quiescent/backtoback modes keep
    # today's report-never-gate posture).
    if [ "$PACING" = "fixed100" ]; then
        # Same rule as the flood guard above, and for a stronger reason:
        # this decides whether a rung is SUSTAINED. Reading the status off
        # the pipeline would conflate "no telemetry lines" (rc 1, a real
        # answer) with "could not read the log" (rc >= 2) — and `|| true`
        # would then turn a broken scan into DROPS=0, i.e. a false
        # sustained. Capture the first grep alone; only rc 1 means zero.
        TELEMETRY=$(grep 'live loop delivery telemetry' "$RUN_LOG")
        TELEMETRY_RC=$?
        if [ "$TELEMETRY_RC" -gt 1 ]; then
            echo "" >&2
            echo "!!! fixed100: payload=$SIZE — could not read $RUN_LOG" >&2
            echo "!!! (grep exit $TELEMETRY_RC) to check drop_oldest; refusing to" >&2
            echo "!!! call this rung sustained on an unread log." >&2
            exit 1
        fi
        DROPS=0
        if [ "$TELEMETRY_RC" -eq 0 ]; then
            # Whole-token, like the ROS 2 receipt parser: `grep -o
            # 'drop_oldest=[0-9][0-9]*'` matches a PREFIX of the value, so
            # `drop_oldest=12junk` reads as `drop_oldest=12`.
            #
            # That prefix match can look "conservative,
            # right by accident". MEASURED, it is not: the truncation is
            # conservative only for junk after a NONZERO digit. The whole
            # `0*` family fails OPEN, and that is the family the gate turns
            # on — `drop_oldest=0junk` truncates to `drop_oldest=0`, which
            # `grep -cv '=0$'` then EXCLUDES, so an unreadable token is
            # counted as zero drops and the rung passes. Same for
            # `drop_oldest=` (empty).
            #
            # A token whose value is not exactly `0` counts, junk included:
            # an unreadable token is not evidence of zero drops. One
            # direction is genuinely narrower than a `grep -o` —
            # `grep -o` also matches INSIDE a longer key, so a future
            # `input_drop_oldest=5` would count there and does not here.
            # No such field is emitted today (the runtime renders a bare
            # `drop_oldest=N` token), and answering for a key we were not
            # asked about is the same class of error in the other
            # direction.
            DROPS=$(printf '%s\n' "$TELEMETRY" | awk '{
                for (i = 1; i <= NF; i++) {
                    n = index($i, "=")
                    if (n > 0 && substr($i, 1, n - 1) == "drop_oldest" &&
                        substr($i, n + 1) != "0") c++
                }
            } END { print c + 0 }')
            # This gate had NO validator, so a dead parser (awk missing, a
            # broken pipe) printed nothing and read as ZERO DROPS —
            # SUSTAINED on a counter nobody managed to read, which is the
            # exact thing the `|| true` note above this block refuses for
            # the grep half. `print c + 0` always emits a digit, so an empty
            # or non-numeric result can only be a parse failure.
            #
            # NOT extended to "no drop_oldest token found ⇒ refuse": that
            # would need evidence that every fixed100 leg emits at least one
            # data-trigger binding line, and the runtime prints the field
            # only for `data_trigger_bindings`. Absence stays 0, as it was.
            case "$DROPS" in
                ''|*[!0-9]*)
                    echo "" >&2
                    echo "!!! fixed100: payload=$SIZE — the drop_oldest scan" >&2
                    echo "!!! produced '$DROPS', not a count; refusing to call" >&2
                    echo "!!! this rung sustained on an unread counter." >&2
                    exit 1
                    ;;
            esac
        fi
        if [ "$DROPS" -gt 0 ]; then
            echo "" >&2
            echo "!!! fixed100: payload=$SIZE at ${RATE_HZ} Hz — $DROPS input(s)" >&2
            echo "!!! report drop_oldest > 0 in the delivery telemetry; the" >&2
            echo "!!! consumer could not keep up at this rung. Not sustained." >&2
            return 10
        fi
    fi

    # fixed100 sustain gate 4: wall-grid slot skips. The ping
    # node paces publishes on a WALL slot grid (CER_BENCH_TARGET_RATE_HZ —
    # the period_ms attr is only the tick source; on the split leg it is
    # LOGICAL time under the handed-quantum lockstep, which is how
    # an ungated split leg free-runs at ~920 Hz while labeled 100 Hz) and
    # counts every slot the graph failed to reach in time. More than 1% of
    # the window's slots skipped = the graph cannot actually hold this rate
    # — the rung is NOT sustained, exactly the native bins' RateLimiter
    # slots_skipped criterion. Absence of the field parses as 0: the
    # runner rebuilds the ping cdylib from THIS tree every size, so the
    # field is only absent if the rewrite/rebuild gates above were
    # defeated.
    if [ "$PACING" = "fixed100" ]; then
        # Fourth member of the grep-status class, and the same verdict as
        # the drop_oldest gate above: an unreadable log must not read as
        # "0 slots skipped" and declare the rung SUSTAINED. `2>/dev/null`
        # is dropped deliberately — the log is ours, so grep's own error
        # IS the signal.
        SKIP_HITS=$(grep -h "RTT_DELIVERY role=ping" "$RUN_LOG")
        SKIP_RC=$?
        if [ "$SKIP_RC" -gt 1 ]; then
            echo "" >&2
            echo "!!! fixed100: payload=$SIZE — could not read $RUN_LOG (grep" >&2
            echo "!!! exit $SKIP_RC) to check slots_skipped; refusing to call this" >&2
            echo "!!! rung sustained on an unread log." >&2
            exit 1
        fi
        SKIPPED=0
        if [ "$SKIP_RC" -eq 0 ]; then
            # Whole-token for the same reason, and here it MATTERS in the
            # other direction: a prefix match truncates `12junk` to a
            # clean `12`, which then sails through the numeric `case`
            # below — the very validator written to catch a non-numeric
            # token never sees one. The last occurrence still wins.
            #
            # `seen` separates ABSENT from PRESENT-BUT-EMPTY, and that
            # distinction is the whole point: if both printed an empty
            # line, `${SKIPPED:-0}` below would turn it into a clean 0
            # BEFORE the validator ran. Then `slots_skipped=` — a real token
            # with an unreadable value — would read as "no slots skipped" and
            # the rung would pass. This field decides a SUSTAIN verdict, and
            # none of the three parsers in this sweep may
            # fail OPEN (`receipt_count` maps an empty value to "?",
            # `drop_oldest=` counts as a drop). An absent field still means
            # zero; a present one that cannot be read reaches the
            # refusal below like every other unreadable token in this file.
            SKIPPED=$(printf '%s\n' "$SKIP_HITS" | awk '{
                for (i = 1; i <= NF; i++) {
                    n = index($i, "=")
                    if (n > 0 && substr($i, 1, n - 1) == "slots_skipped") {
                        last = substr($i, n + 1)
                        seen = 1
                    }
                }
            } END { print (seen ? (last == "" ? "?" : last) : "0") }')
            # awk ALWAYS prints — "0" for an absent field — so an empty
            # read can only mean the parser itself failed (awk missing,
            # OOM, a broken pipe). That is not evidence of zero skips
            # either, so it takes the same refusal arm. A bare
            # `${SKIPPED:-0}` would map BOTH an empty parse and a dead
            # parser onto a clean 0 before the validator could see them.
            [ -n "$SKIPPED" ] || SKIPPED="?"
        fi
        # A non-numeric token would make the `-gt` test return 2 and, with
        # no `set -e`, silently take the false branch — the same false
        # SUSTAINED by a second door.
        case "$SKIPPED" in
            ''|*[!0-9]*)
                echo "" >&2
                echo "!!! fixed100: payload=$SIZE — slots_skipped parsed as" >&2
                echo "!!! '$SKIPPED', not a number — the token was truncated or" >&2
                echo "!!! the parser failed; refusing to judge the rung." >&2
                exit 1
                ;;
        esac
        SKIP_BUDGET=$(( (MEASURED + WARMUP) / 100 ))
        if [ "$SKIPPED" -gt "$SKIP_BUDGET" ]; then
            echo "" >&2
            echo "!!! fixed100: payload=$SIZE at ${RATE_HZ} Hz — the ping's wall" >&2
            echo "!!! grid skipped $SKIPPED slot(s) (> ${SKIP_BUDGET} = 1% of the window);" >&2
            echo "!!! the graph cannot hold this rate. Not sustained." >&2
            return 10
        fi
    fi

    # ── delivery accounting (per size; the mp_latency convention) ───────────
    # RTT_DELIVERY: printed by each node at shutdown (per WORKER on the
    # split leg — worker stdout is the supervisor's, so all lines reach
    # one log). "live loop delivery telemetry": the host's per-input fires +
    # drop_oldest counters, one line per data-trigger input + fires-only
    # producer lines, per process — printed at run_live exit, so the
    # backtoback (virtual poll) leg has none, which is expected and noted.
    # Full delivery == received ≈ forwarded ≈ published AND drop_oldest=0
    # (deficits explained by data-trigger latest-wins coalescing, never
    # silent loss).
    echo "-- delivery accounting (leg=$LEG payload=$SIZE) --"
    if [ "$PACING" != "backtoback" ]; then
        # Schedule vs EFFECTIVE rate side by side (integer period_ms
        # quantization — see the PERIOD_MS comment in the sweep loop): at
        # 60/30 Hz the ping actually ticks ~4%/~1% fast; visible here,
        # never hidden. (Under fixed100 every ladder rung divides 1000
        # exactly, so effective == rate there.)
        echo "   schedule_rate=${RATE_HZ}Hz effective_rate=${EFFECTIVE_HZ}Hz (period_ms=${PERIOD_MS})"
    fi
    # Status captured from the grep itself, not the `sort | grep .` tail:
    # an unreadable log would otherwise arrive here as an ordinary "no
    # lines" and be reported as missing accounting rather than as a log we
    # could not read.
    delivery_lines=$(grep -h "RTT_DELIVERY" "$RUN_LOG")
    delivery_rc=$?
    if [ "$delivery_rc" -gt 1 ]; then
        echo "!!! ERROR: could not read $RUN_LOG (grep exit $delivery_rc) —" >&2
        echo "!!! the delivery accounting below is unverified." >&2
    elif [ "$delivery_rc" -ne 0 ] || [ -z "$delivery_lines" ]; then
        echo "!!! WARNING: no RTT_DELIVERY lines in $RUN_LOG — node shutdown" >&2
        echo "!!! accounting is missing despite a passing .bin gate; inspect the log." >&2
    else
        printf '%s\n' "$delivery_lines" | sort -u
    fi
    # A node's published=/forwarded= counter is frames it wrote COMPLETELY
    # and handed to the output proxy; the commit happens when that proxy
    # drops, and a failed commit logs loudly from the host with an
    # `OutputProxy` prefix. Without this check such a frame is counted as
    # published and the resulting deficit downstream reads as data-trigger
    # coalescing — a transport failure wearing a benign label. Surface it.
    #
    # Match `OutputProxy` WITHOUT the colon: the dominant discard line
    # ("OutputProxy dropped without writing all declared variable fields")
    # has no colon there, and it is precisely the class these Image-output
    # nodes can hit. Then subtract the three prefix-sharing lines that are
    # NOT commit failures — "output recovered" (a regime ending), the
    # replay-suppression note, and a failed SentSample notification (the
    # data WAS delivered; only the wake was lost).
    # The FIRST grep's status is captured on its own, before any filter
    # runs. Reading it off the pipeline would conflate two answers: under
    # `pipefail` an unreadable log (grep rc 2) followed by `grep -v`
    # returning 1 on the resulting empty input yields pipeline status 1 —
    # indistinguishable from the ordinary "no matches", so a broken scan
    # would have been reported as a clean one.
    output_hits=$(grep -h "OutputProxy" "$RUN_LOG")
    grep_rc=$?
    if [ "$grep_rc" -gt 1 ]; then
        # grep itself failed (unreadable log / IO error) — the same rule the
        # flood guard above applies: a broken read is not "no failures".
        echo "!!! ERROR: could not scan $RUN_LOG for OutputProxy commit" >&2
        echo "!!! failures (grep rc=$grep_rc); refusing to claim the delivery" >&2
        echo "!!! accounting is clean." >&2
        exit 1
    fi
    # rc 1 is the normal no-match case: nothing to filter, nothing to report.
    output_fails=""
    if [ "$grep_rc" -eq 0 ]; then
        output_fails=$(printf '%s\n' "$output_hits" \
            | grep -v "output recovered" \
            | grep -v "SentSample notification failed" \
            | grep -v "replay suppressing") || true
    fi
    if [ -n "$output_fails" ]; then
        echo "!!! WARNING: the run log carries OutputProxy commit failures —" >&2
        echo "!!! the published=/forwarded= counters above OVERCOUNT by those" >&2
        echo "!!! frames; the deficit is dropped/discarded frames, NOT" >&2
        echo "!!! data-trigger coalescing:" >&2
        printf '%s\n' "$output_fails" | sort | uniq -c | sed 's/^/!!!   /' >&2
    fi
    telemetry_lines=$(grep -h "live loop delivery telemetry" "$RUN_LOG")
    telemetry_rc=$?
    if [ "$telemetry_rc" -gt 1 ]; then
        echo "!!! ERROR: could not read $RUN_LOG (grep exit $telemetry_rc) —" >&2
        echo "!!! the host-side drop_oldest counters are UNKNOWN, not absent." >&2
    elif [ "$telemetry_rc" -eq 0 ]; then
        printf '%s\n' "$telemetry_lines"
    else
        if [ "$PACING" = "backtoback" ]; then
            echo "   (no live-loop telemetry — the virtual poll path prints none; expected)"
        else
            echo "!!! WARNING: no 'live loop delivery telemetry' lines in $RUN_LOG —" >&2
            echo "!!! the host-side drop_oldest counters are missing; inspect the log." >&2
        fi
    fi
    echo "   samples_in_bin=$SAMPLES_IN_BIN measured=$MEASURED warmup=$WARMUP"
    return 0
}

echo ""
echo "leg=$LEG graph=$GRAPH msg=$MSG_CLASS pacing=$PACING raw_name=$CER_BENCH_RAW_NAME"
echo "sizes: ${SIZES[*]}"

for SIZE in "${SIZES[@]}"; do
    BIN_PATH="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${SIZE}.bin"
    RATE_PATH="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${SIZE}.rate"
    # Stale-sidecar guard (EVERY pacing mode): a leftover fixed100 .rate
    # from a prior run into this dump dir must never label THIS run's
    # rows with a rate it did not run at.
    rm -f "$RATE_PATH"
    RUN_LOG_SUFFIX=""

    if [ "$PACING" = "fixed100" ]; then
        # ── fixed100: uniform 100 Hz target + the fallback ladder ──────
        read -r -a RUNGS <<<"$(ladder_for "$SIZE")"
        ACHIEVED=""
        for RUNG in "${RUNGS[@]}"; do
            RATE_HZ="$RUNG"
            WARMUP="$FIXED100_WARMUP"
            MEASURED=$(( FIXED100_TOTAL - FIXED100_WARMUP ))
            if [ -n "$SMOKE_N" ]; then
                # Smoke override: (total, warmup) = (N, max(N/10, 1)) —
                # same rule as the native smoke_override(); pacing kept.
                WARMUP=$(( SMOKE_N / 10 ))
                [ "$WARMUP" -lt 1 ] && WARMUP=1
                MEASURED=$(( SMOKE_N - WARMUP ))
            fi
            # Every ladder rung (100/50/20/10) divides 1000 exactly — no
            # integer-period quantization skew on this variant.
            PERIOD_MS=$(( 1000 / RATE_HZ ))
            [ "$PERIOD_MS" -lt 1 ] && PERIOD_MS=1
            EFFECTIVE_HZ=$(awk -v p="$PERIOD_MS" 'BEGIN { printf "%.2f", 1000.0 / p }')
            NOMINAL_SECS=$(( (MEASURED + WARMUP + RATE_HZ - 1) / RATE_HZ ))
            WATCHDOG_SECS=$(( NOMINAL_SECS * 2 + 60 ))
            RUN_LOG_SUFFIX="_r${RUNG}"
            run_size_once
            RC=$?
            if [ "$RC" -eq 0 ]; then
                ACHIEVED="$RUNG"
                break
            fi
            echo "!!! fixed100: payload=$SIZE NOT SUSTAINED at ${RUNG} Hz — stepping down the ladder [${RUNGS[*]}] Hz" >&2
        done
        if [ -n "$ACHIEVED" ]; then
            # Achieved-rate sidecar (first line = integer Hz; readers
            # consume the first line only — compile_csv.py folds it into
            # the achieved_rate_hz column).
            {
                echo "$ACHIEVED"
                echo "# fixed100 achieved-rate sidecar: target=${FIXED100_RATE_HZ}Hz ladder=[${RUNGS[*]}]Hz"
            } > "$RATE_PATH"
            if [ "$ACHIEVED" != "$FIXED100_RATE_HZ" ]; then
                echo "!!! fixed100 FALLBACK: payload=$SIZE achieved ${ACHIEVED} Hz, not the ${FIXED100_RATE_HZ} Hz target —" >&2
                echo "!!! recorded in $(basename "$RATE_PATH"); plots annotate the point '@${ACHIEVED}Hz'." >&2
            fi
        else
            # Ladder exhausted: NO latency is minted (Principle #13) —
            # no .bin, a did_not_sustain sidecar, and the row renders
            # 'did not sustain' downstream.
            rm -f "$BIN_PATH"
            {
                echo "did_not_sustain"
                echo "# fixed100 ladder [${RUNGS[*]}]Hz exhausted: no rung sustained; no latency minted"
            } > "$RATE_PATH"
            echo "!!! fixed100 DID NOT SUSTAIN: payload=$SIZE ran at NO ladder rung [${RUNGS[*]}] Hz —" >&2
            echo "!!! no .bin, no latency minted; the row renders 'did not sustain' ($(basename "$RATE_PATH"))." >&2
        fi
        continue
    fi

    if [ "$PACING" = "quiescent" ]; then
        read -r RATE_HZ TOTAL WARMUP <<<"$(schedule_for "$SIZE")"
        # FLATNESS-BISECTION diagnostic knob (quiescent only): decouple
        # rate from size — override the schedule rate, keep everything
        # else. This is how the rate-vs-payload flatness
        # matrices of METHODOLOGY §17 were measured; for a
        # first-class uniform-rate sweep use --variant fixed100 instead.
        if [ -n "${CER_BENCH_FORCE_RATE_HZ:-}" ]; then
            RATE_HZ="${CER_BENCH_FORCE_RATE_HZ}"
        fi
        # MEASURED = total − warmup (G1: CER_BENCH_TARGET_SAMPLES means
        # MEASURED samples in every component; the schedule tuple carries
        # TOTAL, so the runner derives before exporting).
        MEASURED=$(( TOTAL - WARMUP ))
        # 1 kHz -> 1 ms, 500 Hz -> 2 ms, ..., 10 Hz -> 100 ms. INTEGER ms:
        # `period_ms` is the macro attr's unit, so non-divisor schedule
        # rates are QUANTIZED by the integer division — 60 Hz -> 16 ms
        # (62.50 Hz effective, ~4% fast), 30 Hz -> 33 ms (30.30 Hz
        # effective, ~1% fast); every other pinned rate divides 1000
        # exactly. The schedule is deliberately NOT changed to dodge this;
        # instead the EFFECTIVE rate (1000/period_ms) is printed beside the
        # schedule rate in the per-size header and the delivery-accounting
        # block so the skew is visible in every leg log.
        PERIOD_MS=$(( 1000 / RATE_HZ ))
        [ "$PERIOD_MS" -lt 1 ] && PERIOD_MS=1
        EFFECTIVE_HZ=$(awk -v p="$PERIOD_MS" 'BEGIN { printf "%.2f", 1000.0 / p }')
    else
        RATE_HZ=0
        EFFECTIVE_HZ=""
        # CER_BENCH_TARGET_SAMPLES is already MEASURED (suite contract).
        MEASURED="$BB_TARGET"
        WARMUP="$BB_WARMUP"
        # Uncapped virtual poll: period_ms=1 fires every step. Rewritten
        # explicitly (not assumed) so a stale per-size rewrite from a killed
        # quiescent run can never leak into a backtoback run.
        PERIOD_MS=1
    fi
    if [ -n "$SMOKE_N" ]; then
        # Smoke override is (total, warmup) = (N, max(N/10, 1)) — derive
        # MEASURED = total − warmup, mirroring native smoke_override()
        # (bench_plan dumps total − warmup under smoke too).
        WARMUP=$(( SMOKE_N / 10 ))
        [ "$WARMUP" -lt 1 ] && WARMUP=1
        MEASURED=$(( SMOKE_N - WARMUP ))
    fi

    # Watchdog: 2x the nominal sample-collection window (measured + warmup
    # = total iterations) + 60 s of graph build / bring-up / drain
    # headroom. Backtoback (virtual, uncapped) has no rate-derived
    # nominal — its window is seconds; 120 s is generous.
    if [ "$RATE_HZ" -gt 0 ]; then
        NOMINAL_SECS=$(( (MEASURED + WARMUP + RATE_HZ - 1) / RATE_HZ ))
        WATCHDOG_SECS=$(( NOMINAL_SECS * 2 + 60 ))
    else
        WATCHDOG_SECS=120
    fi

    run_size_once || exit 1
done

echo ""
echo "========================================"
echo "  Done — leg=$LEG raw .bin samples in $CER_BENCH_RAW_DUMP_DIR"
echo "  (percentiles are computed OFFLINE from the .bins — compile_csv.py)"
echo "========================================"
ls -la "$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_"*.bin 2>/dev/null || echo "  (no .bin files written)"
