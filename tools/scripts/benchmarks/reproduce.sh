#!/usr/bin/env bash
# tools/scripts/benchmarks/reproduce.sh — one-command benchmark reproduction
#
# Reproduces the Cerulion public latency suite (benches/latency/)
# from a fresh clone. The suite's own driver (bench.py) writes the full artifact
# convention natively: every run lands under
#   benches/latency/results/<machine_hash>-<YYYY-MM-DD>-<variant>/
# carrying raw .bin samples, per-cell CSVs, and the run manifest
# (`run.json` — git sha, machine hash + its plaintext input fields,
# governor/turbo state, per-cell timestamps, skip inventory, docker image
# IDs). Committing a run dir under docs/benchmarks/results/ is a separate,
# deliberate publishing act — see docs/benchmarks/README.md.
#
# Earlier revisions of this script drove the superseded
# benches/cerulion_round_trip{,_quiescent} trees (deleted; see git history)
# and hand-archived their CSVs — the new suite makes that half redundant.
#
# Usage:
#   ./tools/scripts/reproduce_benchmarks.sh [--variant quiescent|fixed100|backtoback] [--check-only]
#
# Flags:
#   --variant V     pacing variant (default: quiescent — the primary suite).
#                   quiescent = sensor-rate realism; fixed100 = uniform
#                   100 Hz at every payload with the fallback ladder (the
#                   payload-flatness variant); backtoback = saturation.
#                   The set MUST match bench.py's PACING_VARIANTS.
#   --check-only    run the prerequisite checks and exit (no benches)
#   -h, --help      this help
#
# Environment:
#   REPRODUCE_YES=1   non-interactive: auto-answer "yes" to the baseline-
#                     capture offer when this host has no expected-ranges
#                     entry for the SELECTED pacing variant
#
# Exit codes:
#   0  sweep completed (or --check-only passed)
#   2  bad CLI args or missing hard prerequisite
#   *  non-zero rc from the LAST failing sweep or post-processing step
#      (`bench.py full` with docker; otherwise native / workspace /
#      compile-csv / plots). LAST, not first: each failing step overwrites
#      SWEEP_RC, so with two failures the earlier code is lost — read the
#      per-step errors above, not just the exit code. Partial results remain
#      under benches/latency/results/; the rc tells CI the sweep was
#      incomplete, and the tail says INCOMPLETE rather than Done.
#
# See benches/latency/README.md for the cell matrix and per-phase runtime.

set -euo pipefail

# readlink -f: resolve through the scripts/reproduce_benchmarks.sh
# symlink — BASH_SOURCE[0] is the symlink path when invoked that way,
# and dirname alone would anchor SCRIPT_DIR one directory too high.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

BENCH_PY="$REPO_ROOT/benches/latency/bench.py"
RANGES_FILE="$REPO_ROOT/benches/latency/expected-ranges.yaml"

VARIANT="quiescent"
CHECK_ONLY=0

# Print the whole leading comment block: line 2 down to the first line that
# is not a comment. DERIVED, NOT A LINE NUMBER — `2,38p` was a magic number
# for a range whose end moves whenever anyone edits the header above it, and
# this change moved it: adding fixed100 to the --variant description pushed
# the cut into the middle of the exit-code table, so `--help` stopped
# mid-sentence and lost the pointer to benches/latency/README.md. The
# block's own end is already unambiguous and needs no maintenance. Same
# shape as scripts/ci_test_shard.sh::usage (which strips no prefix and
# writes to stderr, but bounds its block the same way and for the same
# reason).
usage() {
    awk 'NR == 1 { next } !/^#/ { exit } { sub(/^# ?/, ""); print }' \
        "${BASH_SOURCE[0]}"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --variant)
            case "${2:-}" in
                quiescent|fixed100|backtoback) VARIANT="$2"; shift 2 ;;
                *) echo "error: --variant must be quiescent, fixed100 or backtoback" >&2; exit 2 ;;
            esac
            ;;
        --check-only) CHECK_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown arg: $1" >&2; usage >&2; exit 2 ;;
    esac
done

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
note() { printf '  %s\n' "$*"; }
warn() { printf '  \033[33m! %s\033[0m\n' "$*"; }
fatal_hint() { printf '  \033[31mX %s\033[0m\n' "$*"; }

have() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------------------
# reproduce.sh exists to produce a FULL-fidelity sweep, so ambient
# sample-count overrides are definitionally unwanted here. Rather than
# letting them silently degrade the sweep artifact, declare intent: drop
# each override loudly for this run only. (CER_BENCH_SMOKE_N is smoke-only
# by the suite's own env contract; CER_BENCH_TARGET_SAMPLES/CER_BENCH_WARMUP
# would shrink the measured population under the schedule's minimums.)
# ---------------------------------------------------------------------------
# PRESENCE, not value. `-n` asks whether the variable is non-EMPTY, and the
# readers ask whether it is SET: bench.py's ambient_payload_restriction uses
# `os.environ.get(...)`, for which `raw = ""` is not None — it proceeds and
# then aborts on the empty restriction. So `CER_BENCH_PAYLOAD_SIZES= ./reproduce.sh`
# printed no warning, unset nothing, and the first bench.py call killed the
# whole sweep: exactly what this block exists to prevent. The same
# distinction is spelled out one directory over in
# benches/latency/workspace/run_workspace.sh, where an exported-but-EMPTY
# CERULION_CPU_DMA_LOCK is a contradictory declaration and `${VAR+set}` is
# what catches it.
for _override in CER_BENCH_SMOKE_N CER_BENCH_TARGET_SAMPLES CER_BENCH_WARMUP CER_BENCH_PAYLOAD_SIZES; do
    if [ "$(eval "printf '%s' \"\${$_override+set}\"")" = "set" ]; then
        printf '  \033[33m! %s is exported (value: %s) — unsetting it for this full-fidelity run\033[0m\n' \
            "$_override" "$(eval "printf '%s' \"\${$_override}\"")"
        printf '  (ambient overrides would silently degrade the sweep artifact)\n'
        unset "$_override"
    fi
done

bold "=== Prerequisite checks ==="

HARD_FAIL=0
DOCKER_OK=1
CHRT_OK=1

if [ "$(uname -s)" != "Linux" ]; then
    fatal_hint "this script requires a Linux host (found: $(uname -s))."
    note "On macOS / Windows, drive benches/latency/bench.py directly — see"
    note "benches/latency/README.md (chrt and docker-dependent cells are skipped there)."
    HARD_FAIL=1
fi

if [ "$(uname -m)" != "x86_64" ]; then
    warn "architecture is $(uname -m), not x86_64 — the sweep will run, but the"
    note "numbers are not comparable against x86_64 result sets."
fi

if have python3; then
    note "python3: $(python3 --version 2>&1)"
else
    fatal_hint "python3 not found. Install with:"
    note "    sudo apt install -y python3"
    HARD_FAIL=1
fi

if have cargo && have rustc; then
    note "rust: $(rustc --version 2>&1)"
else
    fatal_hint "cargo/rustc not found. Install with:"
    note "    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    HARD_FAIL=1
fi

if have docker && timeout 10 docker info >/dev/null 2>&1; then
    note "docker: daemon reachable"
else
    DOCKER_OK=0
    warn "docker unavailable (CLI missing or daemon unreachable) — ROS 2 cells will be SKIPPED."
    note "Install with:"
    note "    sudo apt install -y docker.io && sudo usermod -aG docker \$USER  # then re-login"
fi

if timeout 5 chrt -f 80 true 2>/dev/null || timeout 5 sudo -n chrt -f 80 true 2>/dev/null; then
    note "chrt: SCHED_FIFO prio 80 permitted"
else
    CHRT_OK=0
    warn "chrt -f 80 not permitted — chrt-on cells will be SKIPPED (bench.py warns"
    note "once per sweep step and records a *_chrt1 skip in the run's run.json)."
    note "Allow with: add 'your-user - rtprio 99' to /etc/security/limits.conf and re-login,"
    note "or configure passwordless sudo for chrt."
fi

if [ "$HARD_FAIL" -eq 1 ]; then
    echo ""
    echo "Missing hard prerequisites — install the tools above and re-run." >&2
    exit 2
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
    echo ""
    bold "Check-only mode: prerequisites OK (docker_ok=$DOCKER_OK chrt_ok=$CHRT_OK). No benches run."
    exit 0
fi

# ---------------------------------------------------------------------------
# Machine identity (canonical impl: scripts/benchmarks/lib/machine_hash.sh).
# ---------------------------------------------------------------------------
# shellcheck source=lib/machine_hash.sh
source "$SCRIPT_DIR/lib/machine_hash.sh"

MH="$(compute_live_machine_hash)"
GIT_SHA="$(git -C "$REPO_ROOT" rev-parse HEAD)"
bold ""
bold "=== Machine identity ==="
note "machine_hash: $MH"
note "git sha:      $GIT_SHA"

# ---------------------------------------------------------------------------
# Baseline offer: if this host has no expected-ranges entry FOR THIS VARIANT,
# offer to capture one via the smoke gate first so future smoke
# runs can gate.
# ---------------------------------------------------------------------------
# Baselines are keyed by (host, PACING VARIANT) — `hosts.<mh>.rtt_p50_ns.
# <variant>` — because the three variants have different latency
# distributions. A host-key grep therefore suppressed the capture offer for
# a host that has a quiescent baseline and is being asked to run fixed100,
# and `bench.py smoke` would later refuse that run with "no fixed100
# baseline for this host". Ask the question the gate asks.
#
# Through bench.py's own parser, not a second one: the check has to agree
# with the gate it is offering to satisfy, and re-deriving the schema in awk
# is how the two drift. Any failure to read, import or validate answers "no
# baseline", which is the safe direction — it can only over-offer, never
# suppress. Stated precisely, because "merely unnecessary" is true of only
# two of the three: with the file ABSENT or the entry missing the offered
# capture succeeds; with the file CORRUPT the capture fails too, since
# cmd_smoke re-validates even under --capture-baseline. It fails LOUDLY with
# the parse error, which is the point — the old `2>/dev/null` gave silence.
baseline_present() {
    python3 - "$RANGES_FILE" "$MH" "$VARIANT" <<'PYEOF'
import sys
from pathlib import Path
ranges, mh, variant = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
sys.path.insert(0, str(ranges.parent))
try:
    import bench
except Exception as e:                                   # noqa: BLE001
    print(f"  ! could not import benches/latency/bench.py ({e}) — treating "
          f"it as no baseline", file=sys.stderr)
    sys.exit(1)
try:
    data = bench.parse_expected_ranges(ranges.read_text(), str(ranges))
    # validate_expected_ranges too, and in this order: the GATE calls both
    # (bench.py, cmd_smoke), so a file that parses but fails validation
    # would otherwise suppress the offer here and then be refused there —
    # the exact shape this check exists to remove, surviving through the
    # validation door. The predicate MIRRORS the gate's — falsy, not
    # `is None` — so an entry whose variant map is present but empty is
    # "no baseline" in both places rather than one each way.
    bench.validate_expected_ranges(data, str(ranges))
    host = (data.get("hosts") or {}).get(mh) or {}
    ok = bool((host.get("rtt_p50_ns") or {}).get(variant))
except Exception as e:                                   # noqa: BLE001
    print(f"  ! could not use {ranges} ({e}) — treating it as no baseline",
          file=sys.stderr)
    ok = False
sys.exit(0 if ok else 1)
PYEOF
}

if ! baseline_present; then
    echo ""
    warn "no $VARIANT smoke-gate baseline for this host (machine_hash $MH) in"
    note "$(realpath --relative-to="$REPO_ROOT" "$RANGES_FILE")"
    CAPTURE=no
    if [ "${REPRODUCE_YES:-0}" = "1" ]; then
        CAPTURE=yes
        note "REPRODUCE_YES=1 — capturing a smoke baseline first."
    elif [ -t 0 ]; then
        # `|| REPLY=""`: read exits 1 on EOF (Ctrl+D) and set -e would
        # silently kill the script; EOF means "take the default" (No).
        read -r -p "  Run 'bench.py smoke --capture-baseline' first (~minutes)? [y/N] " REPLY || REPLY=""
        case "$REPLY" in [yY]*) CAPTURE=yes ;; esac
    else
        note "non-interactive and REPRODUCE_YES not set — skipping baseline capture."
    fi
    if [ "$CAPTURE" = "yes" ]; then
        # Best-effort: the capture is an optional convenience step and the
        # sweep is this script's purpose — a capture failure (exit 3 on
        # build/setup error) must not abort the reproduction under set -e.
        if python3 "$BENCH_PY" smoke --variant "$VARIANT" --capture-baseline; then
            note "baseline captured — review + commit $(realpath --relative-to="$REPO_ROOT" "$RANGES_FILE")"
        else
            warn "baseline capture exited non-zero — continuing with the full sweep"
        fi
    fi
fi

# ---------------------------------------------------------------------------
# The sweep. With docker: one `bench.py full` call. Without docker: run the
# phases bench.py full would run, minus ros2.
# ---------------------------------------------------------------------------
echo ""
bold "=== Running the sweep (variant=$VARIANT) ==="
SWEEP_RC=0
# The chrt axis is asked for in TWO different registers, and they are not
# the same question.
#
# MEASUREMENT asks `--chrt both` unconditionally. That is bench.py's
# designed graceful path — its own comment says "`--chrt both` always keeps
# chrt0, so its documented skip behaviour is untouched" — and asking is what
# produces the RECORD: on a host where `chrt -f 80` is refused, cmd_native /
# cmd_ros2 call `manifest.record_skip("*_chrt1", "chrt -f 80 unavailable on
# this host — chrt-on cells skipped")`, which lands in run.json's skip
# inventory. This file's own header advertises that inventory as part of the
# published artifact, and it is the only thing that lets a reader of a
# published run dir tell "host policy forbade it" from "nobody ran it".
# Passing `--chrt 0` here would be quieter and would delete that record:
# `record_skip` sits inside `if args.chrt in ("1", "both")`, so at `0` it is
# never reached. A citable run dir has to explain its own gaps.
#
# The workspace legs were hand-split into `--chrt 0` plus a CHRT_OK-gated
# `--chrt 1`, which is that same derivation by another spelling and carried
# the same cost: `cmd_workspace` records `cerulion_workspace_*_chrt1` under
# the identical `if args.chrt in ("1", "both")`, so on a chrt-less host the
# native gap was explained in run.json and the workspace gap was not — while
# the docker branch, where `cmd_full` forwards `both` to `cmd_workspace`,
# recorded both. One script, two branches, two different skip inventories
# for the same host. `--chrt both` collapses that: `cmd_workspace` handles it
# exactly as `cmd_native` does, and the shell probe above and bench.py's
# `find_chrt_prefix` run the same two commands under the same 5 s timeout, so
# they cannot disagree about the answer.
#
# RENDERING asks for what was MEASURED. `bench.py plots` defaults to
# `--chrt 0`, which on a chrt-permitted host silently drops every chrt1 cell
# the sweep just measured; and on a chrt-less host `both` would caveat every
# chrt1 line as absent on every figure of every run, which is caveat
# inflation — the reason is in run.json, where a reader can find it once.
# So this branch's plots step derives its chrt from the probe, and its
# measurement steps do not. Scoped deliberately: the docker `full` call
# forwards its `--chrt` straight into its OWN internal plots step, so on a
# chrt-less host that path does render the caveated figures — the tradeoff
# there is decided by one flag for measurement and rendering together, and
# the record is worth more than the caveat.
# `if`, not `[ ... ] && VAR=`: the && form returns 1 when the test fails, which
# is harmless here (statements follow) but trips `set -e` the moment anyone
# moves it to the end of a block or function. Position-independent instead.
PLOT_CHRT=0
if [ "$CHRT_OK" -eq 1 ]; then PLOT_CHRT=both; fi
if [ "$DOCKER_OK" -eq 1 ]; then
    python3 "$BENCH_PY" full --variant "$VARIANT" --chrt both --build-image \
        || SWEEP_RC=$?
else
    warn "docker unavailable — running native + workspace + post-processing only."
    python3 "$BENCH_PY" native --variant "$VARIANT" --chrt both || SWEEP_RC=$?
    python3 "$BENCH_PY" workspace --variant "$VARIANT" --chrt both || SWEEP_RC=$?
    CSV_RC=0
    python3 "$BENCH_PY" compile-csv --variant "$VARIANT" || CSV_RC=$?
    if [ "$CSV_RC" -ne 0 ]; then SWEEP_RC=$CSV_RC; fi
    # Post-processing must be handed exactly what this branch MEASURED,
    # on both axes, or the figures are not the run:
    #
    #  - chrt. `bench.py plots` defaults to --chrt 0. Where chrt is
    #    permitted this branch measures chrt1 cells too (native --chrt
    #    both, and the workspace chrt1 leg above), so plotting the default
    #    silently drops every one of them from the figures. Where chrt is
    #    NOT permitted, bench.py records a *_chrt1 skip and measures
    #    nothing at chrt1, so asking for it would fail on absent CSVs.
    #  - groups. `plots` defaults to --group all, which includes the three
    #    ros2-<distro> groups. This is the no-docker branch: no ROS 2 cell
    #    ran, so those CSVs cannot exist and a default `plots` fails on
    #    them. Name the two groups this branch actually produced.
    #  - strictness. `bench.py full` passes --skip-missing to its own plots
    #    step "because the SAME invocation printed its own '! skip'
    #    inventory above". This branch is that same situation — a different
    #    bench.py invocation, but the same script run, whose skip inventory
    #    the operator has already seen — and it has its own reachable skip
    #    that the docker path does not: `bench.py native`
    #    drops the zenoh-SHM cells and returns 0 when RLIMIT_MEMLOCK cannot
    #    be raised, which the prerequisite block above does not probe — so a
    #    sweep that succeeded would end on a plots refusal for CSVs the
    #    sweep had already announced it was not producing.
    #
    # ...and it does not run at all if the compile failed. `plots` reads the
    # CSVs `compile-csv` just wrote, so on a failed compile it renders
    # whatever partial set survived and — thanks to --skip-missing —
    # SUCCEEDS, writing PNGs into the run dir. The gated tail below would
    # still say INCOMPLETE, but the tail lives in the terminal and the
    # figures outlive it, indistinguishable from a clean run's. Skipping is
    # the correct outcome: no figure at all is a visible gap, a figure built
    # from a failed compile is not.
    if [ "$CSV_RC" -ne 0 ]; then
        warn "compile-csv failed (rc=$CSV_RC) — SKIPPING plots rather than rendering"
        note "figures from a partial CSV set. Fix the compile and re-run."
    elif python3 "$BENCH_PY" plots --variant "$VARIANT" --chrt "$PLOT_CHRT" \
            --group native --group workspace --skip-missing; then
        note "plots: groups native+workspace at --chrt $PLOT_CHRT — what this branch"
        note "measured. The ros2 groups need docker; re-run with docker for the full matrix."
    else
        SWEEP_RC=$?
    fi
fi
# The tail is a CLAIM about the run, so it is gated on the run. No sweep
# failure aborts before this point despite `set -e`: every step CAPTURES its
# status instead — `|| SWEEP_RC=$?` on the sweep steps, `|| CSV_RC=$?`
# (copied into SWEEP_RC) on compile-csv, and the `else` of the plots
# `if`/`elif`, which does not run at all when the compile failed —
# precisely so a failure reaches the exit code. Which also means
# nothing stops the summary printing after one. "=== Done ===" followed by where the
# artifacts are and how to PUBLISH them, printed after a failed compile-csv or
# plots, describes a run whose artifacts are incomplete as a publishable one.
echo ""
if [ "$SWEEP_RC" -eq 0 ]; then
    bold "=== Done ==="
    note "artifacts: benches/latency/results/${MH}-<date>-$VARIANT/ (raw .bins, CSVs, run.json)"
    note "publishing a run = committing its run dir under docs/benchmarks/results/ — see docs/benchmarks/README.md"
    note "before publishing, read run.json's skip inventory: exit 0 means no step"
    note "FAILED, not that every cell ran (a host limitation is a skip, not an error)."
else
    bold "=== INCOMPLETE (rc=$SWEEP_RC) ==="
    warn "a sweep or post-processing step failed — this run is NOT publishable as it stands."
    note "whatever was produced remains under benches/latency/results/${MH}-<date>-$VARIANT/;"
    note "read the step's own error above, fix it, and re-run. The rc is propagated at exit."
fi
exit "$SWEEP_RC"
