#!/bin/bash
# platform-air.sh - MAC LANE launcher for platform@ba5d75c5c.
# Adapted from the heroes-fix.sh / heroes3-a3.sh lineage for macOS:
#   * shasum -a 256 (no sha256sum)  * no setsid, no nohup (macOS nohup refuses
#   without a controlling terminal) - a backgrounded subshell + `trap '' HUP`
#   * no chrt / no governor: STOCK = untuned. `--chrt 0` means NO chrt wrapper.
#   * no /dev/shm: /tmp/iceoryx2* + .shm_state counts instead
#   * no systemd: no timer gate; launchctl count recorded instead
#   * the CI-idle window gate lives on the DESK (the state file is a desk file),
#     so this script only OBSERVES a HOLD/ABORT breadcrumb the desk drops, and
#     stamps every job's [start,end] UTC into jobs.tsv for desk-side validation.
# Re-runnable per (cell,rep): pass JOBS="mono:1 split:1" to redo just those.
set -uo pipefail
trap '' HUP

LANE=<BOXHOME>/lanes/platform-20260917T000530Z
TC=<BOXHOME>/.rustup/toolchains/stable-aarch64-apple-darwin/bin
export PATH="$LANE/bin:$TC:$PATH"       # bin/ carries the gtimeout + sha256sum shims
export CARGO_HOME="$LANE/.cargo"
export CARGO_TERM_COLOR=never
export CER_BENCH_DMA_LOCK=0             # published-row parity: no C-state cap ask
unset CERULION_FLASHBACK                # recorder stays ON (the visitor default)
unset CERULION_EXECUTION_MODE           # set PER CELL below, never globally
unset CERULION_CPU_DMA_LOCK
ulimit -n 65536 2>/dev/null || true

RD="$LANE/run-platform"
LOG="$LANE/platform.log"
JOBS_TSV="$LANE/jobs.tsv"
BENCH="$LANE/repo/benches/latency"
T0=$(date +%s)
log(){ printf '[%s] [+%05ds] %s\n' "$(date -u +%H:%M:%SZ)" "$(( $(date +%s) - T0 ))" "$*" >> "$LOG"; }

FINAL="ABORTED"; DONEJOBS=""
finish(){ local rc=$?
  { echo "stamp_utc_end: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "elapsed_s: $(( $(date +%s) - T0 ))"
    echo "exit_code: $rc"
    echo "final_state: $FINAL"
    echo "jobs_completed:$DONEJOBS"
    echo "load_at_end: $(uptime | sed 's/.*load averages: //')"
    echo "shm_at_end: iceoryx2_tmp_entries=$(ls /tmp/iceoryx2* 2>/dev/null | wc -l | tr -d ' ') shm_state=$(ls /tmp 2>/dev/null | grep -c shm_state)"
    echo "launchctl_at_end: $(launchctl list 2>/dev/null | wc -l | tr -d ' ')"
  } > "$LANE/DONE-PLATFORM"
  log "DONE-PLATFORM ($FINAL rc=$rc) jobs:$DONEJOBS"; }
trap finish EXIT

# ---- HEAD ASSERT: building/measuring the wrong tree is a silent NON-RESULT --
WANT=ba5d75c5c226e749c9f5e8846c55c21b8bfd1035
HEAD_SHA=$(cd "$LANE/repo" && git rev-parse HEAD)
if [ "$HEAD_SHA" != "$WANT" ]; then
  FINAL="NON-RESULT: HEAD $HEAD_SHA != requested $WANT"; log "!! $FINAL"; exit 12
fi
log "HEAD ASSERT OK: $HEAD_SHA"
BIN="$LANE/repo/target/release/cerulion"
[ -x "$BIN" ] || { FINAL="NON-RESULT: no cerulion binary at $BIN"; log "!! $FINAL"; exit 12; }
SF=$(/usr/bin/shasum -a 256 "$BIN" | cut -d' ' -f1)
log "cerulion binary sha16=${SF:0:16}"
log "load at start: $(uptime | sed 's/.*load averages: //')"
log "shm at start: iceoryx2_tmp=$(ls /tmp/iceoryx2* 2>/dev/null | wc -l | tr -d ' ') shm_state=$(ls /tmp 2>/dev/null | grep -c shm_state)"

# ---- ANSI strip: escapes sit BETWEEN key, '=' and value in on-disk logs ----
strip_ansi(){ sed -E 's/'$'\e''\[[0-9;]*m//g' "$1"; }

# ---- witness sweep over one rep's logs for one cell -------------------------
# MODE freerun|mono. Returns 1 = NON-RESULT, printing observed-vs-expected.
witness(){ local cell="$1" rep="$2" mode="$3" n=0 bad=0
  local d="$RD/rep$rep/raw/_logs"
  for f in "$d"/${cell}_*_r*.log; do
    [ -r "$f" ] || continue
    n=$((n+1))
    local T; T=$(strip_ansi "$f")
    local miss=""
    grep -qF 'flashback: holding a rolling window for this run' <<<"$T" || miss="$miss [MISSING recorder-holding-line]"
    grep -qE 'window="own_recorder"' <<<"$T" || miss="$miss [MISSING window=\"own_recorder\"]"
    if [ "$mode" = "freerun" ]; then
      grep -qE 'execution mode stamped into every worker plan .*execution_mode=FreeRun workers=2' <<<"$T" \
        || miss="$miss [MISSING stamped execution_mode=FreeRun workers=2 | OBSERVED: $(grep -oE 'execution_mode=[A-Za-z]+' <<<"$T" | sort -u | tr '\n' ',')]"
      grep -qF 'free-run deployment: no shared barrier is created' <<<"$T" \
        || miss="$miss [MISSING no-shared-barrier line]"
      local w; w=$(grep -cE 'worker build path resolved from the stamped execution mode .*execution_mode=FreeRun build_path=FreeRunLive' <<<"$T")
      [ "$w" -eq 2 ] || miss="$miss [build_path=FreeRunLive worker lines OBSERVED=$w EXPECTED=2 | OBSERVED build_path: $(grep -oE 'build_path=[A-Za-z]+' <<<"$T" | sort -u | tr '\n' ',')]"
      for bad_s in 'execution_mode=Lockstep' 'build_path=Lockstep' \
                   'deterministic-live (cross-process) gating quantum installed' \
                   'left cross-process barrier cohort' 'is INERT on this run' \
                   'CERULION_EXECUTION_MODE is set but not'; do
        grep -qF "$bad_s" <<<"$T" && miss="$miss [MUST-NOT PRESENT: $bad_s]"
      done
    else
      for bad_s in 'is INERT on this run' 'CERULION_EXECUTION_MODE is set but not'; do
        grep -qF "$bad_s" <<<"$T" && miss="$miss [MUST-NOT PRESENT: $bad_s]"
      done
    fi
    if [ -n "$miss" ]; then bad=$((bad+1)); log "  !! WITNESS FAIL $(basename "$f"):$miss"; fi
  done
  # macOS: the #912 placement line is #[cfg(target_os="linux")] in graph_cmd.rs
  # (BOTH arms - "pinned ... core=N nice=N" AND "floats"), so it cannot be
  # emitted here. RECORD the observation; do NOT gate on a line the binary
  # provably does not contain on this platform.
  local place; place=$(for f in "$d"/${cell}_*_r*.log; do [ -r "$f" ] && strip_ansi "$f" \
      | grep -oE 'window recorder (pinned|floats)[^"]*|core=[0-9]+ +nice=-?[0-9]+'; done | sort -u | tr '\n' ' ')
  log "  WITNESS[$cell rep$rep mode=$mode]: logs=$n failing=$bad ; #912 placement line OBSERVED=[${place:-<absent: cfg(linux)-only, EXPECTED absent on macOS>}]"
  [ "$n" -gt 0 ] || { log "  !! NON-RESULT: zero run logs found under $d"; return 1; }
  [ "$bad" -eq 0 ] || return 1
  return 0; }

# ---- desk-dropped breadcrumbs ----------------------------------------------
# HOLD  = CI went busy; pause at the next CELL boundary (bounded).
# ABORT = stop now.
wait_if_held(){ local waited=0
  while [ -f "$LANE/HOLD" ]; do
    [ "$waited" = "0" ] && log "  HOLD breadcrumb present (desk saw CI busy) - pausing at cell boundary"
    sleep 20; waited=$((waited+20))
    if [ "$waited" -ge 2700 ]; then log "  !! HOLD held 45 min - giving up this run"; return 1; fi
  done
  [ "$waited" -gt 0 ] && log "  HOLD cleared after ${waited}s - resuming"
  return 0; }

run_job(){ local cell="$1" rep="$2"
  [ -f "$LANE/ABORT" ] && { log "ABORT breadcrumb - stopping"; FINAL="ABORTED by desk"; exit 8; }
  wait_if_held || { FINAL="ABORTED: HOLD timeout"; exit 8; }
  local s e rc mode envs leg
  case "$cell" in
    cerulion_workspace_mono_chrt0)  mode=mono;    leg=mono;  envs="" ;;
    cerulion_workspace_split_chrt0) mode=freerun; leg=split; envs="CERULION_EXECUTION_MODE=free_run" ;;
    iox2_chrt0)                     mode=native;  leg="";    envs="" ;;
  esac
  s=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  log "JOB-START cell=$cell rep=$rep env='${envs:-<none>}' load=$(uptime | sed 's/.*load averages: //')"
  if [ "$mode" = "native" ]; then
    ( cd "$BENCH" && env CER_BENCH_DMA_LOCK=0 python3 bench.py native --variant fixed100 \
        --chrt 0 --rep "$rep" --bin iox2 --run-dir "$RD" ) >>"$LOG" 2>&1
    rc=$?
  else
    ( cd "$BENCH" && env CER_BENCH_DMA_LOCK=0 $envs python3 bench.py workspace --variant fixed100 \
        --chrt 0 --rep "$rep" --leg "$leg" --msg-class variable --run-dir "$RD" ) >>"$LOG" 2>&1
    rc=$?
  fi
  e=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  printf '%s\t%s\t%s\t%s\t%s\n' "$cell" "$rep" "$s" "$e" "$rc" >> "$JOBS_TSV"
  log "JOB-END   cell=$cell rep=$rep rc=$rc window=[$s .. $e]"
  if [ "$mode" != "native" ]; then
    if ! witness "$cell" "$rep" "$mode"; then
      log "  !! NON-RESULT: witness gate FAILED for $cell rep$rep"
      echo "$cell rep$rep" >> "$LANE/NON-RESULTS"
      [ "$rep" = "1" ] && { FINAL="NON-RESULT at $cell rep 1"; exit 7; }
    fi
  fi
  DONEJOBS="$DONEJOBS $cell:$rep"; }

CELLS_DEFAULT="cerulion_workspace_mono_chrt0 cerulion_workspace_split_chrt0 iox2_chrt0"
log "=== PLATFORM CAMPAIGN START (fixed100, k=5, STOCK/no-chrt, recorder ON) ==="
if [ -n "${JOBS:-}" ]; then
  log "targeted re-run: JOBS='$JOBS'"
  for j in $JOBS; do run_job "${j%%:*}" "${j##*:}"; done
else
  for rep in 1 2 3 4 5; do                       # INTERLEAVED BY REP
    log "REP-BOUNDARY $rep $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    for cell in $CELLS_DEFAULT; do run_job "$cell" "$rep"; done
  done
fi
( cd "$BENCH" && python3 bench.py compile-csv --variant fixed100 --run-dir "$RD" ) >>"$LOG" 2>&1
log "compile-csv rc=$?"
FINAL="COMPLETED"
