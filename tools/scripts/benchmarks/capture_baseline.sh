#!/usr/bin/env bash
# scripts/benchmarks/capture_baseline.sh — capture a machine baseline for
# Competitive Benchmark V1.
#
# Run ONCE on a fresh target machine BEFORE installing any middleware
# (ROS 2, iceoryx2 tooling, zenoh tooling, etc.). Produces a directory
# baseline-<timestamp>/ with baseline-report.md inside, whose "Summary
# card" section is parsable by scripts/benchmarks/lib/machine_hash.sh.
#
# Usage:
#   sudo ./scripts/benchmarks/capture_baseline.sh [--cyclictest-minutes N] [--dry-run]
#
# Flags:
#   --cyclictest-minutes N   run cyclictest for N minutes (default 10)
#   --dry-run                print what would be captured; run no heavy probes
#   -h, --help               this help
#
# Exit codes:
#   0  baseline captured (even if some Tier 2/3 probes N/A)
#   2  bad CLI args or missing dependency that we require (bash/coreutils)
#   3  user interrupt (SIGINT)
#
# Dependency install hint (print and proceed, not fatal):
#   sudo apt install -y rt-tests linux-tools-generic mbw numactl netperf \
#                       stress-ng dmidecode build-essential

set -euo pipefail

# -------- defaults --------------------------------------------------------
CYCLICTEST_MINUTES=10
DRY_RUN=0

usage() {
    # Derived, not a line number: the header block ends at the first
    # non-comment line. `2,22p` had already drifted past the dependency
    # install hint, so --help omitted it silently.
    awk 'NR == 1 { next } !/^#/ { exit } { sub(/^# ?/, ""); print }' "$0"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --cyclictest-minutes)
            if [ -z "${2:-}" ] || ! printf '%s' "$2" | grep -qE '^[1-9][0-9]*$'; then
                echo "error: --cyclictest-minutes requires a positive integer (>= 1)" >&2
                exit 2
            fi
            CYCLICTEST_MINUTES="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown arg: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

TS=$(date -u +%Y%m%dT%H%M%SZ)
OUT_DIR="baseline-${TS}"
REPORT=""
if [ "$DRY_RUN" -eq 0 ]; then
    mkdir -p "$OUT_DIR"
    REPORT="$OUT_DIR/baseline-report.md"
fi

# Trap Ctrl-C so we don't leave half-written reports.
trap 'echo "interrupted" >&2; exit 3' INT

# -------- tiny helpers ----------------------------------------------------
# Always emit valid markdown. Never leave a field blank — write "N/A" instead.

have() { command -v "$1" >/dev/null 2>&1; }

# write a line to the report (stdout in dry-run)
w() {
    if [ "$DRY_RUN" -eq 1 ]; then
        printf '%s\n' "$*"
    else
        printf '%s\n' "$*" >> "$REPORT"
    fi
}

section() { w ""; w "## $1"; w ""; }

kv() { w "- $1: ${2:-N/A}"; }

flag() { w "- ⚠️ $1"; }

# Run a command, capture COMBINED stdout+stderr, and write it into the
# report inside a fenced block. If the command exits non-zero OR produces
# empty output, write an explicit `N/A` note with the captured stderr so
# readers can see exactly why the section is not populated. Never
# silently swallow errors — that contradicts the anti-falsification
# guarantees in docs/benchmarks/README.md.
#
# Arguments:
#   $1  label                 short label printed before the block (context)
#   $2+ command (+ args)      the command to invoke
#
# Echoes the raw captured output to stdout as well, so callers that need
# to post-process (e.g. extract cyclictest Max) can do so without
# re-running the command.
probe_into_report() {
    local label="$1"
    shift
    local tmp_out tmp_rc
    tmp_out=$(mktemp)
    "$@" >"$tmp_out" 2>&1
    tmp_rc=$?
    if [ "$tmp_rc" -ne 0 ] || [ ! -s "$tmp_out" ]; then
        w ""
        w "**${label}: N/A** (exit=${tmp_rc}, captured output below)"
        w ""
        w '```'
        if [ -s "$tmp_out" ]; then
            cat "$tmp_out" >> "$REPORT"
        else
            printf '%s\n' "(no output)" >> "$REPORT"
        fi
        w '```'
    else
        w ""
        w "${label}:"
        w '```'
        cat "$tmp_out" >> "$REPORT"
        w '```'
    fi
    cat "$tmp_out"
    rm -f "$tmp_out"
    return "$tmp_rc"
}

# -------- dry-run preamble ------------------------------------------------
if [ "$DRY_RUN" -eq 1 ]; then
    cat <<EOF
DRY RUN — would capture the following (no heavy probes actually executed).

Tier 1 (always):
  - Hostname, date, kernel, uptime
  - /sys/kernel/realtime (PREEMPT_RT detection)
  - lscpu, /proc/cpuinfo (CPU model, threads, flags, invariant TSC)
  - cpupower frequency-info (governor, boost)
  - /proc/meminfo, numactl --hardware (memory + NUMA topology)
  - dmidecode (if present) — firmware / BIOS version
  - uname, /etc/os-release
  - ip link / lsblk (network + storage inventory)

Tier 2 (requires rt-tests package):
  - cyclictest -l (for ${CYCLICTEST_MINUTES} minutes on one core)
  - hwlatdetect (short 30s sample)

Tier 3 (requires mbw, netperf):
  - mbw memory bandwidth
  - netperf loopback TCP/UDP

Summary card (parsed by machine_hash.sh):
  - CPU model, CPU threads, Kernel, Governor, PREEMPT_RT flag

Install tools:
  sudo apt install -y rt-tests linux-tools-generic mbw numactl netperf \\
                      stress-ng dmidecode build-essential
EOF
    exit 0
fi

# -------- permissions check (informational; do not hard-fail) -------------
if [ "$(id -u)" -ne 0 ]; then
    echo "warning: not running as root; some probes (cyclictest, hwlatdetect, dmidecode) need sudo" >&2
fi

# -------- PREEMPT_RT guard (assumption A3) --------------------------------
PREEMPT_RT=no
if [ -r /sys/kernel/realtime ] && [ "$(cat /sys/kernel/realtime 2>/dev/null)" = "1" ]; then
    PREEMPT_RT=yes
    echo "WARNING: /sys/kernel/realtime == 1 — this is a PREEMPT_RT kernel." >&2
    echo "V1 results are stock-kernel only (assumption A3). Numbers produced" >&2
    echo "on this machine cannot be compared against the published V1 set." >&2
    echo "Proceeding with capture so the baseline is on record, but the measurement" >&2
    echo "run must STOP before committing if this flag is yes." >&2
fi

# -------- gather raw values we'll reuse -----------------------------------
HOSTNAME_STR=$(hostname)
KERNEL=$(uname -r)
OS_PRETTY="N/A"
if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    OS_PRETTY=$(. /etc/os-release && printf '%s' "${PRETTY_NAME:-N/A}")
fi
CPU_MODEL="N/A"
CPU_THREADS="N/A"
if have lscpu; then
    CPU_MODEL=$(lscpu | awk -F: '/^Model name:/ { sub(/^ +/, "", $2); print $2; exit }')
    CPU_THREADS=$(lscpu | awk -F: '/^CPU\(s\):/ { gsub(/ /,"",$2); print $2; exit }')
fi
[ -z "$CPU_MODEL" ] && CPU_MODEL="N/A"
[ -z "$CPU_THREADS" ] && CPU_THREADS="N/A"

# Invariant TSC flag
INV_TSC=no
if grep -q 'constant_tsc' /proc/cpuinfo 2>/dev/null && grep -q 'nonstop_tsc' /proc/cpuinfo 2>/dev/null; then
    INV_TSC=yes
fi

# Governor (pick the first policy; warn if they differ across CPUs)
GOVERNOR="N/A"
if have cpupower; then
    GOVERNOR=$(cpupower frequency-info 2>/dev/null | awk -F\" '/"[a-z]+" governor/ { print $2; exit }')
fi
if [ -z "$GOVERNOR" ] && [ -r /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    GOVERNOR=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
fi
[ -z "$GOVERNOR" ] && GOVERNOR="N/A"

# Mixed governor across CPUs?
MIXED_GOV=no
if ls /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1; then
    U=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort -u | wc -l)
    [ "$U" -gt 1 ] && MIXED_GOV=yes
fi

# NUMA node count
NUMA_NODES="N/A"
if have numactl; then
    NUMA_NODES=$(numactl --hardware 2>/dev/null | awk '/^available:/ { print $2; exit }')
fi
[ -z "$NUMA_NODES" ] && NUMA_NODES="N/A"

# -------- write header ----------------------------------------------------
w "# Baseline report — ${HOSTNAME_STR} — ${TS}"
w ""
w "Generated by \`scripts/benchmarks/capture_baseline.sh\`."
w ""
w "> This file is the environmental context for every CSV row that cites"
w "> the machine_hash derived from the Summary card below. Do not edit"
w "> after commit — if the machine changes, capture a new baseline."

# -------- Tier 1: always-on probes ---------------------------------------
section "Tier 1 — identity"
kv "Hostname" "$HOSTNAME_STR"
kv "Date (UTC)" "$TS"
kv "OS" "$OS_PRETTY"
kv "Kernel" "$KERNEL"
kv "Uptime" "$(uptime -p 2>/dev/null || echo N/A)"
kv "PREEMPT_RT" "$PREEMPT_RT"

section "Tier 1 — CPU"
kv "Model" "$CPU_MODEL"
kv "Threads" "$CPU_THREADS"
kv "Invariant TSC" "$INV_TSC"
kv "Governor (cpu0)" "$GOVERNOR"
kv "Mixed governors across CPUs" "$MIXED_GOV"
if have lscpu; then
    w ""
    w '```'
    lscpu >> "$REPORT" 2>/dev/null || true
    w '```'
fi
if [ "$INV_TSC" != "yes" ]; then
    flag "Invariant TSC not detected — latency measurements may drift. STOP and ask."
fi
if [ "$GOVERNOR" != "performance" ] && [ "$GOVERNOR" != "N/A" ]; then
    flag "CPU governor is \"$GOVERNOR\", not \"performance\". Expected on stock Ubuntu per A3 — document in PR."
fi
if [ "$MIXED_GOV" = "yes" ]; then
    flag "CPUs report different governors. Pinning strategy must account for this."
fi

section "Tier 1 — memory + NUMA"
kv "Total RAM" "$(awk '/^MemTotal:/ { print $2 " " $3 }' /proc/meminfo 2>/dev/null || echo N/A)"
kv "NUMA nodes" "$NUMA_NODES"
if have numactl; then
    w ""
    w '```'
    numactl --hardware >> "$REPORT" 2>/dev/null || true
    w '```'
fi
if [ "$NUMA_NODES" != "N/A" ] && [ "$NUMA_NODES" -gt 1 ] 2>/dev/null; then
    flag "Multi-NUMA system. Pinning strategy required before bench runs — STOP and ask."
fi

section "Tier 1 — firmware"
if have dmidecode; then
    w '```'
    (dmidecode -t bios 2>/dev/null || echo "N/A (need sudo)") >> "$REPORT"
    w '```'
else
    kv "dmidecode" "not installed"
fi

section "Tier 1 — network & storage inventory"
w '```'
(ip -br link 2>/dev/null || echo "ip not available") >> "$REPORT"
w '```'
w ""
w '```'
(lsblk -o NAME,SIZE,TYPE,MOUNTPOINT 2>/dev/null || echo "lsblk not available") >> "$REPORT"
w '```'

# -------- Tier 2: realtime latency probes ---------------------------------
section "Tier 2 — realtime latency"

if have cyclictest; then
    # -l N = N loops; we want approx CYCLICTEST_MINUTES of runtime at 1kHz
    # so loops = minutes * 60 * 1000
    LOOPS=$(( CYCLICTEST_MINUTES * 60 * 1000 ))
    CT_OUT=$(probe_into_report \
        "cyclictest, ${CYCLICTEST_MINUTES} minute(s), 1 thread, priority 99" \
        cyclictest -m -p99 -i1000 -l "$LOOPS" -t 1) || true

    # Extract Max for a flag (Max: <int> us appears on T: 0 line or summary)
    MAX_US=$(printf '%s\n' "$CT_OUT" | awk '
        /Max:/ { for (i=1;i<=NF;i++) if ($i ~ /Max:/) { print $(i+1); exit } }
    ' | tail -n 1 || true)
    if [ -n "$MAX_US" ] && [ "$MAX_US" -gt 1000 ] 2>/dev/null; then
        flag "cyclictest Max = ${MAX_US} us (>1000 us). Kill background load or document in PR."
    fi
else
    kv "cyclictest" "N/A (rt-tests package not installed — apt install rt-tests)"
fi

if have hwlatdetect; then
    probe_into_report "hwlatdetect, 30s sample" hwlatdetect --duration=30 >/dev/null || true
else
    kv "hwlatdetect" "N/A (rt-tests package not installed — apt install rt-tests)"
fi

# -------- Tier 3: bandwidth probes ----------------------------------------
section "Tier 3 — bandwidth"

if have mbw; then
    w ""
    w "mbw memory bandwidth, 256 MiB, MEMCPY+DUMB+MCBLOCK:"
    w '```'
    (mbw -q -n 3 256 2>/dev/null | tail -n 12) >> "$REPORT" || true
    w '```'
else
    kv "mbw" "not installed (apt install mbw)"
fi

if have netperf && have netserver; then
    w ""
    w "netperf loopback TCP_STREAM, 10s:"
    w '```'
    # netserver must be running; start a local one if not
    if ! pgrep -x netserver >/dev/null 2>&1; then
        (netserver -D >/dev/null 2>&1 &) || true
        sleep 1
    fi
    (netperf -H 127.0.0.1 -l 10 -t TCP_STREAM 2>/dev/null | tail -n 6) >> "$REPORT" || true
    w '```'
else
    kv "netperf" "not installed (apt install netperf)"
fi

# -------- Summary card (load-bearing: parsed by machine_hash.sh) ---------
section "Summary card"
w "- CPU model: $CPU_MODEL"
w "- CPU threads: $CPU_THREADS"
w "- Kernel: $KERNEL"
w "- Governor: $GOVERNOR"
w "- PREEMPT_RT: $PREEMPT_RT"

# -------- finalize --------------------------------------------------------
w ""
w "---"
w ""
w "Install hint (for missing tools above):"
w ""
w '```'
w "sudo apt install -y rt-tests linux-tools-generic mbw numactl netperf \\"
w "                    stress-ng dmidecode build-essential"
w '```'

echo "baseline captured: $REPORT"
