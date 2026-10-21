#!/usr/bin/env bash
# Desk-side driver for the MI300X droplet. From this directory on the desk:
#
#   ./ship.sh <droplet-ip>            push, start bootstrap detached, watch, pull results
#   ./ship.sh <droplet-ip> push       rsync the worktree (no target/, .git, results) to /root/cerulion-hip/
#   ./ship.sh <droplet-ip> start      start bootstrap.sh detached on the droplet (survives ssh drops)
#   ./ship.sh <droplet-ip> watch      tail the droplet's bootstrap.out until DONE or FAILED
#   ./ship.sh <droplet-ip> pull       copy results/ and bootstrap logs back into ./results/
#   ./ship.sh <droplet-ip> check      ssh + ROCm presence check only (no changes on the droplet)
#
# Env passed to the droplet: REPS (default 3), STAGES, RESUME, FORCE_CAMPAIGN, GPU_S0_CLEANUP (default 1).
# Testing against another box: SHIP_USER (default root), SHIP_REMOTE (default /root/cerulion-hip),
# SHIP_WRAP (a command prefix for bootstrap, e.g. a lock), and GPU_S0_CLEANUP=0 on a shared box.
# WATCH_HOURS (default 4) bounds the watch loop; 10 consecutive ssh failures end it.
set -euo pipefail
IP=${1:?usage: ship.sh <droplet-ip> [all|push|start|watch|pull|check]}
ACTION=${2:-all}
HOST="${SHIP_USER:-root}@${IP}"
REMOTE=${SHIP_REMOTE:-/root/cerulion-hip}
HERE=$(cd "$(dirname "$0")" && pwd)
WT=$(cd "$HERE/../.." && pwd)
SSH=(ssh -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=30 -o ConnectTimeout=20 -o BatchMode=yes)
COMMIT=$(git -C "$WT" rev-parse --short=10 HEAD 2>/dev/null || echo unknown)
git -C "$WT" diff --quiet HEAD -- . 2>/dev/null || COMMIT="$COMMIT-dirty"

push() {
  echo "push: $WT -> ${HOST}:${REMOTE}/ (commit $COMMIT)"
  "${SSH[@]}" "$HOST" "mkdir -p ${REMOTE}"
  if "${SSH[@]}" "$HOST" "command -v rsync" >/dev/null; then
    rsync -az --delete -e "${SSH[*]}" \
      --exclude target/ --exclude .git --exclude 'tools/gpu-probe-hip/results/' --exclude 'tools/gpu-probe-hip/bootstrap-logs/' \
      "$WT/" "${HOST}:${REMOTE}/"
  else
    # No rsync on the droplet yet (bootstrap installs it): tar over ssh, without macOS AppleDouble files.
    COPYFILE_DISABLE=1 tar -C "$WT" --exclude ./target --exclude ./.git --exclude ./tools/gpu-probe-hip/target \
      --exclude ./tools/gpu-probe-hip/results --exclude ./tools/gpu-probe-hip/bootstrap-logs -czf - . \
      | "${SSH[@]}" "$HOST" "tar -C ${REMOTE} -xzf -"
    n=$("${SSH[@]}" "$HOST" "find ${REMOTE} -name '._*' | wc -l")
    [ "$n" = 0 ] || { echo "push: $n AppleDouble files arrived; aborting"; exit 1; }
  fi
  "${SSH[@]}" "$HOST" "ls ${REMOTE}/tools/gpu-probe-hip/bootstrap.sh ${REMOTE}/crates/cerulion_device/Cargo.toml" >/dev/null
  echo "push: done"
}

start() {
  echo "start: bootstrap on ${HOST} (REPS=${REPS:-3} STAGES=${STAGES:-all})"
  "${SSH[@]}" "$HOST" "cd ${REMOTE}/tools/gpu-probe-hip && nohup setsid ${SHIP_WRAP:-} env GPU_S0_COMMIT=${COMMIT} REPS=${REPS:-3} GPU_S0_CLEANUP=${GPU_S0_CLEANUP:-1} \
    ${STAGES:+STAGES='${STAGES}'} ${RESUME:+RESUME='${RESUME}'} ${FORCE_CAMPAIGN:+FORCE_CAMPAIGN=${FORCE_CAMPAIGN}} \
    bash bootstrap.sh > ${REMOTE}/bootstrap.out 2>&1 < /dev/null & echo started pid \$!"
}

watch() {
  local fails=0 deadline=$(( $(date +%s) + ${WATCH_HOURS:-4} * 3600 )) last=""
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if out=$("${SSH[@]}" "$HOST" "tail -n 4 ${REMOTE}/bootstrap.out" 2>/dev/null); then
      fails=0
      [ "$out" != "$last" ] && { echo "--- $(date +%T)"; echo "$out"; last=$out; }
      if grep -q "BOOTSTRAP DONE" <<<"$out"; then return 0; fi
      if grep -q "BOOTSTRAP FAILED" <<<"$out"; then return 1; fi
    else
      fails=$((fails + 1))
      echo "watch: ssh failed ($fails/10)"
      [ "$fails" -ge 10 ] && { echo "watch: giving up after 10 ssh failures"; return 2; }
    fi
    sleep 60
  done
  echo "watch: WATCH_HOURS reached"; return 3
}

pull() {
  mkdir -p "$HERE/results"
  rsync -az -e "${SSH[*]}" "${HOST}:${REMOTE}/tools/gpu-probe-hip/results/" "$HERE/results/"
  rsync -az -e "${SSH[*]}" "${HOST}:${REMOTE}/tools/gpu-probe-hip/bootstrap-logs/" "$HERE/results/bootstrap-logs-${IP}/"
  rsync -az -e "${SSH[*]}" "${HOST}:${REMOTE}/bootstrap.out" "$HERE/results/bootstrap-${IP}.out" || true
  echo "pull: $(ls -d "$HERE"/results/2* 2>/dev/null | tail -3 | tr '\n' ' ')"
}

check() {
  "${SSH[@]}" "$HOST" 'hostname; uname -r; ls -l /dev/kfd; cat /opt/rocm/.info/version 2>/dev/null; ls /opt/rocm/lib/libamdhip64.so* /opt/rocm/lib/libhiprtc.so* 2>/dev/null; command -v rocminfo rocm-smi rsync python3 cc || true'
}

case $ACTION in
  push) push ;;
  start) start ;;
  watch) watch ;;
  pull) pull ;;
  check) check ;;
  all) push; start; rc=0; watch || rc=$?; pull; exit $rc ;;
  *) echo "unknown action $ACTION"; exit 2 ;;
esac
