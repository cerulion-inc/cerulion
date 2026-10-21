#!/usr/bin/env bash
# S0 HIP campaign. Run from anywhere; it works in its own directory.
#
#   ./run.sh                          full campaign: probe, then REPS reps of floor + every mode at 8 sizes
#   SMOKE=1 ./run.sh                  one size (64 KiB) per mode, 200 iters, 1 rep; non-zero exit on any mismatch
#   DRY_CPU=1 ./run.sh                cpu rows only (no GPU needed); calibrates the timing loop
#   RESUME=results/<stamp> ./run.sh   continue an interrupted campaign (done units are skipped)
#   OUT_DIR=<dir> ./run.sh            write into <dir> instead of results/<UTC stamp>
#
# Env: REPS (default 3), MODES (space list, default all), SIZES (comma list or "all"),
#      ITERS (iteration override), UNIT_TIMEOUT (s, default 600; 180 in SMOKE), GPU_S0_SMI (default 1:
#      rocm-smi sclk after every row), GPU_S0_CLEANUP=1 (clean dead iceoryx2 nodes after a
#      failed unit; only on a machine the probe owns), GPU_S0_COMMIT (recorded in env.txt).
# Prints the results directory on the last line.
set -euo pipefail
cd "$(dirname "$0")"
B=target/release/gpu-probe-hip
[ -x "$B" ] || { echo "run.sh: $B missing; run cargo build --release first" >&2; exit 1; }

REPS=${REPS:-3}
SIZES=${SIZES:-all}
ITERS=${ITERS:-}
TAG=${GPU_S0_TAG:-}
if [ "${SMOKE:-0}" = 1 ]; then REPS=1; SIZES=65536; ITERS=${ITERS:-200}; TAG=smoke; UNIT_TIMEOUT=${UNIT_TIMEOUT:-180}; fi
if [ "${DRY_CPU:-0}" = 1 ]; then MODES=cpu; TAG=${TAG:-dry-cpu}; fi
ALL_MODES=$("$B" modes)
MODES=${MODES:-$ALL_MODES}

if [ -n "${RESUME:-}" ]; then
  OUT=$RESUME
  [ -d "$OUT" ] || { echo "run.sh: RESUME dir $OUT does not exist" >&2; exit 1; }
elif [ -n "${OUT_DIR:-}" ]; then
  OUT=$OUT_DIR
else
  OUT=results/$(date -u +%Y%m%dT%H%M%SZ)${TAG:+-$TAG}
fi
mkdir -p "$OUT"
RAW="$OUT/raw.log"
DONE="$OUT/done.list"
touch "$RAW" "$DONE"
export GPU_S0_KCACHE="$PWD/target/kcache"
export GPU_S0_SMI=${GPU_S0_SMI:-1}
[ "${DRY_CPU:-0}" = 1 ] && export GPU_S0_SMI=0

first_line() { head -n "${2:-1}" 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g'; }
if [ ! -s "$OUT/env.txt" ]; then
  {
    echo "commit=${GPU_S0_COMMIT:-unknown}"
    echo "host=$(hostname)"
    echo "date_utc=$(date -u +%FT%TZ)"
    echo "uname=$(uname -srm)"
    echo "os=$(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
    echo "cmdline=$(cat /proc/cmdline 2>/dev/null)"
    echo "cpu=$(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^ //') x$(nproc)"
    echo "mem=$(free -g 2>/dev/null | awk '/Mem:/{print $2" GiB"}')"
    echo "cpu_governor=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
    echo "rustc=$(rustc --version 2>/dev/null || ~/.cargo/bin/rustc --version 2>/dev/null)"
    echo "reps=$REPS sizes=$SIZES iters=${ITERS:-default} modes=$MODES"
    echo "tag=${TAG:-campaign}"
    echo "dev_shm=$(df -h /dev/shm 2>/dev/null | awk 'NR==2{print $2" total, "$4" free"}')"
    echo "iommu=$(ls /sys/class/iommu 2>/dev/null | tr '\n' ' ') groups=$(ls /sys/kernel/iommu_groups 2>/dev/null | wc -l)"
    if [ "${DRY_CPU:-0}" != 1 ]; then
      echo "rocm_version=$(cat /opt/rocm/.info/version 2>/dev/null || echo n/a)"
      echo "amdgpu_module=$(cat /sys/module/amdgpu/version 2>/dev/null || modinfo -F version amdgpu 2>/dev/null || echo n/a)"
      echo "hip_libs=$(ls /opt/rocm/lib/libamdhip64.so* /opt/rocm/lib/libhiprtc.so* 2>/dev/null | tr '\n' ' ')"
      echo "rocminfo_gpu=$(rocminfo 2>/dev/null | grep -E 'Marketing Name|^ *Name: *gfx' | sed 's/  */ /g' | tr '\n' ' ')"
      echo "rocm_smi_product=$(rocm-smi --showproductname 2>/dev/null | grep -E 'Card (series|model|SKU)|GFX Version' | sed 's/  */ /g' | tr '\n' ' ')"
      echo "rocm_smi_driver=$(rocm-smi --showdriverversion 2>/dev/null | grep -i version | sed 's/  */ /g' | tr '\n' ' ')"
      echo "partition_compute=$(rocm-smi --showcomputepartition 2>/dev/null | grep -i partition | sed 's/  */ /g' | tr '\n' ' ')"
      echo "partition_memory=$(rocm-smi --showmemorypartition 2>/dev/null | grep -i partition | sed 's/  */ /g' | tr '\n' ' ')"
      echo "amd_smi_partition=$(amd-smi static --partition 2>/dev/null | grep -iE 'compute|memory' | sed 's/  */ /g' | tr '\n' ' ')"
      echo "gpu_clocks_at_start=$(rocm-smi --showclocks 2>/dev/null | grep -iE 'sclk|mclk|fclk' | sed 's/  */ /g' | tr '\n' ' ')"
      echo "perf_level=$(rocm-smi --showperflevel 2>/dev/null | grep -i 'performance level' | sed 's/  */ /g' | tr '\n' ' ')"
      echo "gpu_count=$(rocm-smi --showid 2>/dev/null | grep -c 'GPU\[' || echo 0)"
    fi
  } > "$OUT/env.txt"
fi

# One unit: a probe-hip command whose RESULT lines go to raw.log tagged with the rep.
unit() {
  local rep=$1 name=$2; shift 2
  local key="$rep $name"
  if grep -qxF "$key" "$DONE"; then echo "  rep $rep $name: done earlier, skipped"; return 0; fi
  local tmp="$OUT/.unit.tmp" t0 t1 rc=0
  t0=$(date +%s.%N)
  timeout --kill-after=10 "${UNIT_TIMEOUT:-600}" "$@" > "$tmp" 2>>"$OUT/stderr.log" || rc=$?
  t1=$(date +%s.%N)
  grep '^RESULT {' "$tmp" | sed "s/^RESULT {/RESULT {\"rep\":$rep,/" >> "$RAW" || true
  if [ "$rc" -ne 0 ]; then
    echo "RESULT {\"rep\":$rep,\"align\":\"pad\",\"kind\":\"failed\",\"mode\":\"$name\",\"rc\":$rc,\"reason\":\"unit exited $rc\"}" >> "$RAW"
    if [ "${GPU_S0_CLEANUP:-0}" = 1 ]; then
      pkill -f "gpu-probe-hip reader" 2>/dev/null || true
      "$B" cleanup-dead-nodes | sed "s/^RESULT {/RESULT {\"rep\":$rep,/" >> "$RAW" || true
    fi
  fi
  local secs; secs=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.1f", b-a}')
  echo "WALL {\"rep\":$rep,\"unit\":\"$name\",\"secs\":$secs,\"rc\":$rc}" >> "$RAW"
  echo "$key" >> "$DONE"
  rm -f "$tmp"
  echo "  rep $rep $name rc=$rc ${secs}s"
}

echo "run.sh: $OUT (reps=$REPS sizes=$SIZES iters=${ITERS:-default})"
if [ "${DRY_CPU:-0}" != 1 ]; then
  unit 0 probe "$B" probe
fi
for rep in $(seq 1 "$REPS"); do
  if [ "${DRY_CPU:-0}" != 1 ]; then
    unit "$rep" floor "$B" floor "${ITERS:-default}" "$SIZES"
  fi
  for m in $MODES; do
    unit "$rep" "$m" "$B" writer "$m" "$SIZES" "${ITERS:-default}"
  done
done
python3 summarize.py "$OUT" > "$OUT/SUMMARY.md"
if [ "${SMOKE:-0}" = 1 ]; then
  python3 summarize.py --check "$OUT"
fi
echo "$OUT"
