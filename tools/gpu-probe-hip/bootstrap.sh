#!/usr/bin/env bash
# One command on a fresh ROCm Ubuntu GPU droplet, as root:
#
#   bash /root/cerulion-hip/tools/gpu-probe-hip/bootstrap.sh
#
# Stages (STAGES, default "setup build smoke campaign"):
#   setup     apt packages it lacks (build-essential pkg-config git rsync python3 curl),
#             rustup (minimal profile, stable, >= 1.95) into $HOME, ROCm checks, GPU print
#   build     cargo build --release, then the probe's unit tests and summarize tests
#   smoke     SMOKE=1 run.sh: probe + one size (64 KiB) per mode; 0 mismatches required
#   campaign  run.sh with REPS (default 3): results/<UTC stamp>/{env.txt,raw.log,SUMMARY.md}
#
# Idempotent: installed pieces are skipped, a build is incremental, and every run writes new
# results directories. RESUME=results/<stamp> continues an interrupted campaign.
# Logs to bootstrap-logs/bootstrap-<stamp>.log. The last line is
# "BOOTSTRAP DONE results=<dir>" or "BOOTSTRAP FAILED stage=<stage> ...".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE"
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p bootstrap-logs
LOG="$HERE/bootstrap-logs/bootstrap-$STAMP.log"
exec > >(tee -a "$LOG") 2>&1
STAGES=${STAGES:-setup build smoke campaign}
STAGE=init
T_START=$(date +%s)
trap 'echo "BOOTSTRAP FAILED stage=$STAGE line=$LINENO log=$LOG"' ERR

say() { echo "== [$(date -u +%T) +$(( $(date +%s) - T_START ))s] $*"; }
want() { case " $STAGES " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }
ROCM=${ROCM_PATH:-/opt/rocm}
export PATH="$HOME/.cargo/bin:$ROCM/bin:$PATH"
export LD_LIBRARY_PATH="$ROCM/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
RUST_MIN=1.95

say "bootstrap $STAMP stages=[$STAGES] host=$(hostname) user=$(id -un) log=$LOG"

ver_ge() { [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]; }

if want setup; then
  STAGE=setup
  say "setup: packages"
  need=()
  command -v cc >/dev/null || need+=(build-essential)
  command -v pkg-config >/dev/null || need+=(pkg-config)
  command -v git >/dev/null || need+=(git)
  command -v rsync >/dev/null || need+=(rsync)
  command -v python3 >/dev/null || need+=(python3)
  command -v curl >/dev/null || need+=(curl ca-certificates)
  if [ ${#need[@]} -gt 0 ]; then
    [ "$(id -u)" = 0 ] || { echo "missing ${need[*]} and not root; install them first"; false; }
    say "apt-get install ${need[*]}"
    DEBIAN_FRONTEND=noninteractive apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends "${need[@]}"
  else
    say "packages present"
  fi

  say "setup: rust"
  if command -v rustc >/dev/null && ver_ge "$(rustc --version | awk '{print $2}')" "$RUST_MIN"; then
    say "rustc present: $(rustc --version)"
  elif command -v rustup >/dev/null; then
    rustup toolchain install stable --profile minimal && rustup default stable
  else
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh
    sh /tmp/rustup-init.sh -y --profile minimal --default-toolchain stable --no-modify-path
    rm -f /tmp/rustup-init.sh
  fi
  ver_ge "$(rustc --version | awk '{print $2}')" "$RUST_MIN" || { echo "rustc $(rustc --version) is below $RUST_MIN"; false; }
  say "rust: $(rustc --version), $(cargo --version)"

  say "setup: ROCm checks ($ROCM)"
  [ -e /dev/kfd ] || { echo "no /dev/kfd: the amdgpu kernel driver is not loaded"; false; }
  ls -l /dev/kfd /dev/dri/renderD* 2>/dev/null | sed 's/^/  /' || true
  command -v rocminfo >/dev/null || { echo "rocminfo not found (looked in PATH and $ROCM/bin)"; false; }
  command -v rocm-smi >/dev/null || echo "WARNING: rocm-smi not found; GPU clock rows fall back to sysfs only"
  echo "  ROCm version: $(cat "$ROCM/.info/version" 2>/dev/null || echo unknown)"
  for lib in libamdhip64 libhiprtc libhsa-runtime64; do
    found=$(ls "$ROCM"/lib/$lib.so* 2>/dev/null | tr '\n' ' ' || true)
    echo "  $lib: ${found:-MISSING}"
    if [ -z "$found" ]; then
      case $lib in
        libhsa-runtime64) echo "  WARNING: no $lib; device-dmabuf-event will skip" ;;
        *) echo "$lib not found under $ROCM/lib"; false ;;
      esac
    fi
  done
  # cerulion_device tries libamdhip64.so.6, .so.5, .so; ROCm 7 ships .so.7. Without an
  # unversioned symlink, point a private shim dir at the real file (nothing under $ROCM changes).
  SHIM="$HERE/target/libshim"
  for lib in libamdhip64 libhiprtc; do
    if [ ! -e "$ROCM/lib/$lib.so" ]; then
      real=$(ls "$ROCM"/lib/$lib.so.[0-9]* 2>/dev/null | sort -V | tail -1 || true)
      [ -n "$real" ] || continue
      mkdir -p "$SHIM"
      ln -sf "$real" "$SHIM/$lib.so"
      echo "  shim: $SHIM/$lib.so -> $real"
    fi
  done
  echo "  ulimit -l: $(ulimit -l)   /dev/shm: $(df -h /dev/shm | awk 'NR==2{print $2" total, "$4" free"}')"
  echo "  kernel: $(uname -r)   cmdline: $(cat /proc/cmdline)"
  say "GPU:"
  rocminfo 2>/dev/null | grep -E '^ *(Marketing Name|Name: +gfx|Compute Unit|Chip ID)' | sed 's/  */ /g;s/^/  /' | head -12 || true
  rocm-smi --showproductname 2>/dev/null | grep -E 'GPU\[' | sed 's/^/  /' | head -8 || true
  rocm-smi --showcomputepartition --showmemorypartition 2>/dev/null | grep -E 'GPU\[' | sed 's/^/  /' | head -8 || true
fi
[ -d "$HERE/target/libshim" ] && export LD_LIBRARY_PATH="$HERE/target/libshim:$LD_LIBRARY_PATH"

if want build; then
  STAGE=build
  say "build: cargo build --release (jobs $(nproc))"
  t=$(date +%s)
  if ! CARGO_BUILD_JOBS=$(nproc) cargo build --release > bootstrap-logs/build-$STAMP.log 2>&1; then
    tail -40 bootstrap-logs/build-$STAMP.log; echo "cargo build failed"; false
  fi
  grep -E '^(warning|error)|Finished' bootstrap-logs/build-$STAMP.log || true
  say "build: $(( $(date +%s) - t ))s"
  say "build: unit tests"
  if ! CARGO_BUILD_JOBS=$(nproc) cargo test --release > bootstrap-logs/test-$STAMP.log 2>&1; then
    grep -E 'FAILED|panicked|error' bootstrap-logs/test-$STAMP.log | head -20; echo "unit tests failed"; false
  fi
  grep -E 'test result' bootstrap-logs/test-$STAMP.log
  python3 -I test_summarize.py
  say "build: probe"
  target/release/gpu-probe-hip probe | python3 -c '
import json, sys
for line in sys.stdin:
    if line.startswith("RESULT "):
        for k, v in json.loads(line[7:]).items():
            print(f"  {k} = {v}")
' || echo "  (probe exited non-zero; the smoke stage records it)"
fi

SMOKE_DIR=""
if want smoke; then
  STAGE=smoke
  SMOKE_DIR="results/$STAMP-smoke"
  say "smoke: $SMOKE_DIR"
  t=$(date +%s)
  if OUT_DIR="$SMOKE_DIR" SMOKE=1 GPU_S0_CLEANUP="${GPU_S0_CLEANUP:-1}" GPU_S0_COMMIT="${GPU_S0_COMMIT:-unknown}" ./run.sh; then
    say "smoke: PASS in $(( $(date +%s) - t ))s"
  else
    say "smoke: FAIL in $(( $(date +%s) - t ))s; see $SMOKE_DIR/SUMMARY.md and $SMOKE_DIR/stderr.log"
    if [ "${FORCE_CAMPAIGN:-0}" != 1 ]; then
      STAGE=smoke
      echo "BOOTSTRAP FAILED stage=smoke results=$HERE/$SMOKE_DIR (set FORCE_CAMPAIGN=1 to run the campaign anyway)"
      exit 1
    fi
  fi
fi

OUT=""
if want campaign; then
  STAGE=campaign
  OUT=${RESUME:-results/$STAMP}
  # A mode whose smoke unit hit its timeout (rc 124 or 137) would hang again in every rep:
  # leave it out of the campaign and say so (the smoke SUMMARY.md keeps its rows).
  CAMPAIGN_MODES=${MODES:-$(target/release/gpu-probe-hip modes)}
  if [ -n "$SMOKE_DIR" ] && [ -s "$SMOKE_DIR/raw.log" ]; then
    hung=$(python3 -I -c '
import json, sys
for line in open(sys.argv[1]):
    if line.startswith("WALL "):
        w = json.loads(line[5:])
        if w["rc"] in (124, 137):
            print(w["unit"])
' "$SMOKE_DIR/raw.log" | tr '\n' ' ')
    if [ -n "$hung" ]; then
      say "campaign: leaving out modes that timed out in smoke: $hung"
      for h in $hung; do CAMPAIGN_MODES=$(echo " $CAMPAIGN_MODES " | sed "s/ $h / /" | xargs); done
    fi
  fi
  say "campaign: $OUT reps=${REPS:-3} modes=[$CAMPAIGN_MODES]"
  t=$(date +%s)
  MODES="$CAMPAIGN_MODES" OUT_DIR="$OUT" REPS="${REPS:-3}" GPU_S0_CLEANUP="${GPU_S0_CLEANUP:-1}" GPU_S0_COMMIT="${GPU_S0_COMMIT:-unknown}" ./run.sh
  say "campaign: $(( $(date +%s) - t ))s"
  sed -n '1,60p' "$OUT/SUMMARY.md" | grep -v '^$' | head -60
fi

trap - ERR
say "total $(( $(date +%s) - T_START ))s"
echo "BOOTSTRAP DONE results=$HERE/${OUT:-$SMOKE_DIR}"
