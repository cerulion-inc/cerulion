#!/usr/bin/env bash
# run_benchmarks.sh — Run Cerulion benchmark entry points
#
# Usage:
#   ./scripts/run_benchmarks.sh          # smoke + CI latency gate (the default;
#                                        # on a host with no captured smoke
#                                        # baseline the smoke gate is SKIPPED
#                                        # loudly and the latency gate still
#                                        # runs — a regression or a setup
#                                        # failure still stops it)
#   ./scripts/run_benchmarks.sh smoke    # benches/latency smoke gate (~minutes)
#   ./scripts/run_benchmarks.sh full     # benches/latency full sweep (hours; Linux + Docker)
#   ./scripts/run_benchmarks.sh latency  # CI latency threshold test
#
# The public latency suite is benches/latency/ — driven by
# benches/latency/bench.py. This wrapper exists for discoverability; the
# driver's own subcommands (native / workspace / ros2 / compile-csv /
# plots / smoke / full / list-cells) are the real interface. The old
# `fast` / `inprocess` arms drove trees that were later deleted
# (benches/shm_fast, the removed in-process transport) and are gone.
#
# One-command reproduction with prerequisite checks + baseline capture:
#   ./scripts/benchmarks/reproduce.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }

# bench.py smoke's "no baseline for this host+variant" status. A baseline is
# per (machine_hash, variant) and is captured ON the machine, so a fresh
# clone cannot have one: it is the normal first-run state, not a failure.
# It is its own code precisely so this wrapper can tell it apart from a
# build or setup failure without re-deriving the question.
SMOKE_RC_NO_BASELINE=4

# Returns bench.py's status instead of letting `set -e` take the script
# down, so the caller decides what each one means.
run_smoke() {
    bold "=== benches/latency smoke gate ==="
    echo "Curated low-n cell subset vs this host's expected-ranges baseline."
    echo ""
    local rc=0
    python3 "$REPO_ROOT/benches/latency/bench.py" smoke || rc=$?
    echo ""
    return "$rc"
}

run_full() {
    bold "=== benches/latency full sweep (hours; Linux + Docker) ==="
    echo ""
    python3 "$REPO_ROOT/benches/latency/bench.py" full --chrt both --build-image
    echo ""
}

run_latency() {
    bold "=== Latency Threshold Test (Release Mode) ==="
    echo "Verifies latency stays below regression threshold."
    echo ""
    cd "$REPO_ROOT"
    cargo test -p cerulion_core --test latency_threshold_test --release -- --test-threads=1 --nocapture 2>&1
    echo ""
}

case "${1:-all}" in
    smoke)
        # Asked for by name: "I cannot gate here" is still a non-zero answer
        # to "gate this host", so it is propagated verbatim.
        run_smoke
        ;;
    full)
        run_full
        ;;
    latency)
        run_latency
        ;;
    all)
        # The DEFAULT command, and the one a fresh clone runs first. The smoke
        # gate needs a machine-specific baseline that a fresh clone cannot
        # have, and under `set -e` its non-zero exit used to take the script
        # down before the latency gate — which needs no baseline at all — ever
        # ran. So the default gate could not run on a clean checkout.
        #
        # Exactly one status is tolerated, and only to SKIP: "no baseline
        # here". A real regression (2) and a build or setup failure (3) still
        # stop the run, and the banner never claims a gate that did not run.
        SMOKE_RC=0
        run_smoke || SMOKE_RC=$?
        if [ "$SMOKE_RC" -eq "$SMOKE_RC_NO_BASELINE" ]; then
            echo "" >&2
            echo "! smoke gate SKIPPED: no baseline was usable for this host and" >&2
            echo "  variant, so there was nothing to gate against. WHICH of the" >&2
            echo "  four shapes it was -- an absent file, the shipped 'hosts: {}'," >&2
            echo "  a file holding only OTHER hosts (so this machine's identity may" >&2
            echo "  have drifted), or this host's own entry with the variant" >&2
            echo "  missing -- was printed by the gate just above. They share an" >&2
            echo "  exit code but NOT a remedy, and this wrapper cannot tell them" >&2
            echo "  apart; read that line before capturing." >&2
            echo "  Baselines are per machine AND per pacing variant; this wrapper" >&2
            echo "  always gates the default (quiescent):" >&2
            echo "      python3 benches/latency/bench.py smoke --variant quiescent --capture-baseline" >&2
            echo "  Continuing to the latency gate, which needs no baseline." >&2
            echo "" >&2
        elif [ "$SMOKE_RC" -ne 0 ]; then
            exit "$SMOKE_RC"
        fi
        run_latency
        echo ""
        if [ "$SMOKE_RC" -eq 0 ]; then
            green "=== Smoke gate + latency gate passed ==="
        else
            green "=== Latency gate passed (smoke gate skipped — no baseline) ==="
        fi
        echo "For the full campaign: ./scripts/run_benchmarks.sh full"
        ;;
    *)
        echo "Usage: $0 {smoke|full|latency|all}"
        exit 1
        ;;
esac
