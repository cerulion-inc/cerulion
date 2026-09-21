#!/usr/bin/env bash
# scripts/benchmarks/lib/machine_hash.sh — compute a stable machine identifier
# from a baseline-report.md Summary card.
#
# The hash ties every results row (via the `machine_hash` CSV column) back to
# exactly ONE baseline-report.md. Every later benchmark cites this hash so a
# reader can always find the environmental context for any number in any CSV.
#
# The hash is a 16-char sha256 prefix over a canonicalized join of these
# fields (read from the baseline-report.md "## Summary card" section):
#
#   CPU model
#   CPU threads
#   Kernel version
#   Governor
#   PREEMPT_RT flag
#
# This is deliberately coarse. Minor changes (e.g. new microcode, a new
# apt package) are allowed to share a hash — they are already captured in
# run.json.tooling_versions. The hash is about hardware + kernel identity,
# not software state.
#
# shellcheck shell=bash

# ---------------------------------------------------------------------------
# compute_machine_hash <baseline_report_path>
#
# Parses the Summary card from a baseline report and prints a 16-char
# sha256 prefix to stdout. Exits non-zero with a message to stderr if the
# report is missing or any required field can't be found.
# ---------------------------------------------------------------------------
compute_machine_hash() {
    local report="$1"
    if [ -z "$report" ]; then
        echo "compute_machine_hash: usage: compute_machine_hash <baseline-report.md>" >&2
        return 2
    fi
    if [ ! -f "$report" ]; then
        echo "compute_machine_hash: file not found: $report" >&2
        return 1
    fi

    # Extract the Summary card section. It starts at "## Summary card" and
    # runs until the next "## " heading or end of file.
    local section
    section=$(awk '
        /^## Summary card[[:space:]]*$/ { capture=1; next }
        capture && /^## / { capture=0 }
        capture { print }
    ' "$report")

    if [ -z "$section" ]; then
        echo "compute_machine_hash: no \"## Summary card\" section in $report" >&2
        return 1
    fi

    # Each field is written by capture_baseline.sh as a line of the form:
    #   - CPU model: Intel(R) Core(TM) Ultra 9 285K
    #   - CPU threads: 24
    #   - Kernel: 6.8.0-50-generic
    #   - Governor: performance
    #   - PREEMPT_RT: no
    local cpu_model cpu_threads kernel governor preempt_rt
    cpu_model=$(_mh_field "$section" "CPU model")
    cpu_threads=$(_mh_field "$section" "CPU threads")
    kernel=$(_mh_field "$section" "Kernel")
    governor=$(_mh_field "$section" "Governor")
    preempt_rt=$(_mh_field "$section" "PREEMPT_RT")

    for name in cpu_model cpu_threads kernel governor preempt_rt; do
        local val="${!name}"
        if [ -z "$val" ]; then
            echo "compute_machine_hash: missing field '$name' in Summary card of $report" >&2
            return 1
        fi
    done

    # Canonical join with a null separator so fields can't collide if one
    # happens to contain commas/pipes.
    local joined
    joined=$(printf '%s\x1f%s\x1f%s\x1f%s\x1f%s' \
        "$cpu_model" "$cpu_threads" "$kernel" "$governor" "$preempt_rt")

    # 16-char sha256 prefix is plenty of collision resistance for a machine
    # roster that will stay under a few dozen hosts.
    printf '%s' "$joined" | sha256sum | cut -c1-16
}

# ---------------------------------------------------------------------------
# compute_live_machine_hash
#
# Compute the machine hash directly from the live system instead of a
# committed baseline-report.md. Derives the same five Summary-card fields
# the same way capture_baseline.sh does (lscpu Model name / CPU(s),
# uname -r, cpupower-then-sysfs governor, /sys/kernel/realtime), then
# applies the identical canonical join + sha256-16 as
# compute_machine_hash above. A host whose committed baseline report was
# captured on the same kernel/governor therefore hashes identically via
# either path.
#
# Consumers: scripts/benchmarks/reproduce.sh and
# benches/latency/bench.py::compute_machine_hash (via
# subprocess — this function is the single source of truth; do not
# reimplement the field derivation elsewhere).
#
# Prints a 16-char sha256 prefix to stdout. Missing probes degrade to
# "N/A" exactly as capture_baseline.sh writes them, so the hash is still
# stable on minimal hosts (e.g. containers without cpufreq).
# ---------------------------------------------------------------------------
compute_live_machine_hash() {
    local cpu_model="" cpu_threads="" kernel governor="" preempt_rt
    if command -v lscpu >/dev/null 2>&1; then
        # `|| var=""`: probe failures must degrade, not abort — callers
        # (reproduce.sh) run under `set -euo pipefail`, where a failing
        # pipeline inside a command-substitution assignment kills the shell.
        cpu_model=$(lscpu | awk -F: '/^Model name:/ { sub(/^ +/, "", $2); print $2; exit }') || cpu_model=""
        cpu_threads=$(lscpu | awk -F: '/^CPU\(s\):/ { gsub(/ /,"",$2); print $2; exit }') || cpu_threads=""
    fi
    [ -z "$cpu_model" ] && cpu_model="N/A"
    [ -z "$cpu_threads" ] && cpu_threads="N/A"

    kernel=$(uname -r) || kernel=""
    [ -z "$kernel" ] && kernel="N/A"

    if command -v cpupower >/dev/null 2>&1; then
        # cpupower exits non-zero on hosts without cpufreq (containers,
        # VMs) even when installed — guard so pipefail can't abort here.
        governor=$(cpupower frequency-info 2>/dev/null | awk -F\" '/"[a-z]+" governor/ { print $2; exit }') || governor=""
    fi
    if [ -z "$governor" ] && [ -r /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
        governor=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor) || governor=""
    fi
    [ -z "$governor" ] && governor="N/A"

    preempt_rt=no
    if [ -r /sys/kernel/realtime ] && [ "$(cat /sys/kernel/realtime 2>/dev/null)" = "1" ]; then
        preempt_rt=yes
    fi

    # Same canonical join as compute_machine_hash — null-ish separator so
    # fields can't collide.
    printf '%s\x1f%s\x1f%s\x1f%s\x1f%s' \
        "$cpu_model" "$cpu_threads" "$kernel" "$governor" "$preempt_rt" \
        | sha256sum | cut -c1-16
}

# Internal: extract "<value>" from a line like "- <label>: <value>".
# Prints empty string if not found. Trims leading/trailing whitespace.
_mh_field() {
    local section="$1"
    local label="$2"
    printf '%s\n' "$section" \
        | sed -n "s/^[[:space:]]*-[[:space:]]*${label}:[[:space:]]*//p" \
        | head -n 1 \
        | sed -e 's/[[:space:]]*$//'
}
