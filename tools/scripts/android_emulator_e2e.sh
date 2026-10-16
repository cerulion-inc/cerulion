#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"
cd "$ROOT"
ABI="${CERULION_E2E_ABI:-x86_64}"
OUT="${CERULION_E2E_OUT:-target/android-e2e}"
[[ "$OUT" = /* ]] || OUT="$ROOT/$OUT"
case "$ABI" in
  x86_64) TRIPLE=x86_64-linux-android ;;
  arm64-v8a) TRIPLE=aarch64-linux-android ;;
  *) echo "FAIL unsupported CERULION_E2E_ABI: $ABI" >&2; exit 1 ;;
esac

build() {
  cargo build -p cerulion_cli -p cerulion_netd
  cargo ndk -t "$ABI" --platform 30 build -p cerulion_cli -p cerulion_netd
  (cd "$ROOT/examples/basic_timer" && cargo ndk -t "$ABI" --platform 30 build)
  cargo ndk -t "$ABI" --platform 30 \
    -o "$ROOT/crates/cerulion_android/android/app/src/main/jniLibs" \
    build --release -p cerulion_android
  # shellcheck disable=SC2086
  gradle -p "$ROOT/crates/cerulion_android/android" assembleDebug ${GRADLE_ARGS:-}
}

if [[ "${1:-}" == build ]]; then
  build
  exit 0
fi
if [[ "${1:-}" != run ]]; then
  echo "usage: $0 build|run" >&2
  exit 2
fi

mkdir -p "$OUT"
exec > >(tee -a "$OUT/e2e.log") 2>&1
export CERULION_LOGIN_GATE=off

ADB=(adb)
if [[ -n "${ANDROID_SERIAL:-}" ]]; then
  ADB+=(-s "$ANDROID_SERIAL")
fi
DEV=/data/local/tmp/cer-e2e
APP=com.cerulion.sample
HOST_CER="$ROOT/target/debug/cerulion"
HOST_STATE="$(mktemp -d "${TMPDIR:-/tmp}/cer-e2e-host.XXXXXX")"
mkdir -p "$HOST_STATE/home"
mkdir -m 700 "$HOST_STATE/run"
HOST_ENV=(env)
while IFS= read -r name; do
  [[ "$name" == CERULION_LOGIN_GATE ]] || HOST_ENV+=(-u "$name")
done < <(compgen -e | grep '^CERULION_' || true)
HOST_ENV+=(HOME="$HOST_STATE/home" XDG_RUNTIME_DIR="$HOST_STATE/run" RUST_LOG=warn)
HOST_NETD_PID="$HOST_STATE/run/cerulion/netd.pid"
DEV_CER="$DEV/cerulion"
DEV_NETD="$DEV/cerulion-netd"
GRAPH_PID="$DEV/graph.pid"
NETD_PID="$DEV/netd.pid"
CHECKS=0
# Ownership flags: cleanup touches only what THIS invocation created, so a
# run that fails early never stops a foreign graph, app, or forward.
started_netd=0
started_graph=0
started_app=0
created_dev=0
added_forward=0
# The host netd can only be spawned by this run's own CLI invocations into
# this run's private HOST_STATE, so its pid file alone proves ownership;
# no separate flag is needed.
HOST_NETD_OWNED() { [[ -s "$HOST_NETD_PID" ]]; }

pass() {
  CHECKS=$((CHECKS + 1))
  echo "PASS $1: ${2:-ok}"
}
fail() {
  echo "FAIL $1: ${2:-unknown}" >&2
  exit 1
}
device() {
  "${ADB[@]}" shell "$@"
}
remote_alive() {
  device "test -s '$1' && kill -0 \$(cat '$1')" >/dev/null 2>&1
}
wait_remote_dead() {
  local pidfile=$1 deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    remote_alive "$pidfile" || return 0
    sleep 1
  done
  fail "remote process exit" "pid file $pidfile is still alive"
}
numeric_values() {
  sed -nE 's/.*data:[[:space:]]*(-?[0-9]+).*/\1/p'
}
tick_values() {
  sed -nE 's/.*tick[[:space:]]+([0-9]+).*/\1/p'
}
assert_increasing() {
  local label=$1 values=$2
  mapfile -t nums < <(printf '%s\n' "$values" | sed '/^[[:space:]]*$/d')
  (( ${#nums[@]} >= 3 )) || fail "$label decoded values" "only ${#nums[@]} values: ${nums[*]:-none}"
  local i
  for ((i = 1; i < ${#nums[@]}; i++)); do
    (( nums[i] > nums[i - 1] )) ||
      fail "$label decoded values" "not strictly increasing: ${nums[*]}"
  done
  pass "$label decoded values" "count=${#nums[@]} first=${nums[0]} last=${nums[-1]}"
}
# A `topic` invocation can spawn a host netd; if that child inherits the
# captured command's pipe it holds the pipe open after the CLI exits and the
# caller's capture hangs. Redirect to a plain file the child can keep without
# stalling us, then print it.
host_capture() {
  local out
  out=$(mktemp)
  "$@" >"$out" 2>&1 </dev/null
  local rc=$?
  cat "$out"
  rm -f "$out"
  return $rc
}
host_list() {
  host_capture timeout "${1:-10}" "${HOST_ENV[@]}" "$HOST_CER" topic list --connect tcp/127.0.0.1:7683
}
host_echo() {
  local topic=$1
  host_capture timeout "${2:-8}" "${HOST_ENV[@]}" CERULION_PEERS=127.0.0.1:7683 \
    CERULION_NETD_CONNECT=tcp/127.0.0.1:7683 "$HOST_CER" topic echo "$topic"
}
cleanup() {
  local rc=$?
  set +e
  (( started_graph == 1 )) &&
    device "test -s '$GRAPH_PID' && kill -INT \$(cat '$GRAPH_PID')" >/dev/null 2>&1
  (( started_netd == 1 )) &&
    device "test -s '$NETD_PID' && kill -TERM \$(cat '$NETD_PID')" >/dev/null 2>&1
  # Each PID file is touched only when THIS run started the matching
  # component: a run refused because $DEV pre-existed must never read or
  # kill PIDs belonging to whatever owns that directory.
  local -a pidfiles=()
  (( started_graph == 1 )) && pidfiles+=("$GRAPH_PID")
  (( started_netd == 1 )) && pidfiles+=("$NETD_PID")
  for pidfile in "${pidfiles[@]}"; do
    device "i=0; while test -s '$pidfile' && kill -0 \$(cat '$pidfile') 2>/dev/null && [ \$i -lt 5 ]; do sleep 1; i=\$((i+1)); done; if test -s '$pidfile' && kill -0 \$(cat '$pidfile') 2>/dev/null; then kill -KILL \$(cat '$pidfile') 2>/dev/null; sleep 1; fi" >/dev/null 2>&1
    device "test -s '$pidfile' && kill -0 \$(cat '$pidfile') 2>/dev/null" >/dev/null 2>&1 &&
      echo "cleanup: $(basename "$pidfile") process still running after SIGKILL" >&2
  done
  (( added_forward == 1 )) &&
    "${ADB[@]}" forward --remove tcp:7683 >/dev/null 2>&1
  HOST_NETD_OWNED && kill -TERM "$(cat "$HOST_NETD_PID")" >/dev/null 2>&1
  "${ADB[@]}" shell logcat -d > "$OUT/logcat.txt" 2>&1
  device "find '$DEV' -maxdepth 1 -type f -name '*.log' -exec sh -c 'for f; do printf \"%s\\n\" \"--- \$f ---\"; cat \"\$f\"; done' sh {} +" \
    > "$OUT/device-logs.txt" 2>&1
  (( started_app == 1 )) && device "am force-stop $APP" >/dev/null 2>&1
  (( created_dev == 1 )) && device "rm -rf '$DEV'" >/dev/null 2>&1
  rm -rf "$HOST_STATE"
  exit "$rc"
}
trap cleanup EXIT

if ! "${ADB[@]}" get-state >/dev/null 2>&1; then
  echo "FAIL no adb device is available"
  exit 1
fi
if [[ "$("${ADB[@]}" shell getprop sys.boot_completed | tr -d '\r')" != 1 ]]; then
  echo "FAIL adb device is not booted"
  exit 1
fi
device_serial="$("${ADB[@]}" get-serialno | tr -d '\r')"
pass "emulator boot" "serial=$device_serial"
host_local="$("${HOST_ENV[@]}" "$HOST_CER" topic list --local 2>&1)" ||
  fail "host local topics" "$host_local"
if grep -qE '/basic_timer/timer/count|/app/heartbeat' <<<"$host_local"; then
  fail "host local topics" "the host already publishes a test topic"
fi
pass "host local topics" "no local test publisher"

echo "android e2e: abi=$ABI triple=$TRIPLE device=${ANDROID_SERIAL:-default}"
# The busy-device check runs BEFORE any rm or force-stop: a device that is
# already serving another run must be refused untouched.
busy="$(device "if pidof cerulion cerulion-netd; then :; else rc=\$?; [ \$rc -eq 1 ] || exit \$rc; fi" 2>&1)" ||
  fail "dedicated device" "could not check for cerulion processes: $busy"
busy="${busy//$'\r'/}"
[[ -z "$busy" ]] ||
  fail "dedicated device" "cerulion processes are already running: pids=$busy"
busy="$(device "if pidof '$APP'; then :; else rc=\$?; [ \$rc -eq 1 ] || exit \$rc; fi" 2>&1)" ||
  fail "dedicated device" "could not check for the sample app: $busy"
busy="${busy//$'\r'/}"
[[ -z "$busy" ]] ||
  fail "dedicated device" "the sample app is already running: pid=$busy"
# Everything this run creates on the device lives under $DEV: the
# cerulion shm dir, and the iceoryx2 root via a project-local
# config/iceoryx2.toml that each device invocation sees from its cwd. The
# shared /data/local/tmp/iceoryx2 tree is never touched.
# The shm dir must be owned by the runner and not group/other-writable;
# the fork refuses a 777 directory (EACCES) at the first shm_open.
# Ownership BEFORE deletion: only a $DEV this run did not find reaches the
# rm below, so a refused run leaves a pre-existing directory's PID files
# untouched.
device "test -e '$DEV' -o -L '$DEV'" &&
  fail "device setup" "$DEV already exists on the device; this run refuses to remove a directory it did not create"
device "rm -f '$GRAPH_PID' '$NETD_PID'"
# Marked owned BEFORE the mkdir: the existence check above proves $DEV was
# absent, so anything under it at cleanup time is this run's partial setup.
created_dev=1
device "mkdir -p '$DEV' '$DEV/basic_timer/target/$TRIPLE/debug' '$DEV/config' '$DEV/basic_timer/config' && mkdir -m 700 '$DEV/shm'" 
device "printf '[global]\nroot-path = \"%s/iceoryx2\"\n' '$DEV' > '$DEV/config/iceoryx2.toml' && cp '$DEV/config/iceoryx2.toml' '$DEV/basic_timer/config/iceoryx2.toml'"
device "export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; mkdir -p \"\$HOME\" \"\$TMPDIR\""
"${ADB[@]}" push "$ROOT/target/$TRIPLE/debug/cerulion" "$DEV_CER" >/dev/null
"${ADB[@]}" push "$ROOT/target/$TRIPLE/debug/cerulion-netd" "$DEV_NETD" >/dev/null
"${ADB[@]}" push "$ROOT/examples/basic_timer/Cargo.toml" "$DEV/basic_timer/Cargo.toml" >/dev/null
"${ADB[@]}" push "$ROOT/examples/basic_timer/graphs" "$DEV/basic_timer/graphs" >/dev/null
"${ADB[@]}" push "$ROOT/examples/basic_timer/nodes" "$DEV/basic_timer/nodes" >/dev/null
mapfile -t node_libs < <(
  find "$ROOT/examples/basic_timer/target/$TRIPLE/debug" -maxdepth 1 -type f -name 'lib*.so' -print
)
((${#node_libs[@]} > 0)) ||
  fail "basic timer libraries" "no cdylibs found under examples/basic_timer/target/$TRIPLE/debug"
while IFS= read -r lib; do
  "${ADB[@]}" push "$lib" "$DEV/basic_timer/target/$TRIPLE/debug/" >/dev/null
done < <(printf '%s\n' "${node_libs[@]}")
device "chmod 755 '$DEV_CER' '$DEV_NETD'"
pass "device push" "node_libraries=${#node_libs[@]}"

device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open CERULION_NETD_LISTEN=tcp/0.0.0.0:7683; nohup '$DEV_NETD' > '$DEV/netd.log' 2>&1 & echo \$! > '$NETD_PID'"
started_netd=1
deadline=$((SECONDS + 20))
ready=0
while (( SECONDS < deadline )); do
  if device "netstat -ltn 2>/dev/null | grep -q ':7683'"; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "device netd" "tcp/7683 did not become ready"
period_ms="$(sed -nE 's/.*period_ms[[:space:]]*=[[:space:]]*([0-9]+).*/\1/p' \
  "$ROOT/examples/basic_timer/nodes/timer/src/lib.rs" | head -1)"
[[ "$period_ms" =~ ^[0-9]+$ && period_ms -gt 0 ]] ||
  fail "timer period" "could not read period_ms from the timer node"
graph_started_ms="$(date +%s%3N)"
device "cd '$DEV/basic_timer' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; nohup '$DEV_CER' graph run basic_timer --record='$DEV/record' > '$DEV/graph.log' 2>&1 & echo \$! > '$GRAPH_PID'"
started_graph=1
sleep 2
device_echo_rc=0
device_echo="$(device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; timeout 8 '$DEV_CER' topic echo /basic_timer/timer/count" 2>&1)" ||
  device_echo_rc=$?
printf 'rc=%s\n%s\n' "$device_echo_rc" "$device_echo" > "$OUT/device-echo.txt"
(( device_echo_rc == 0 || device_echo_rc == 124 )) ||
  fail "device echo" "rc=$device_echo_rc output=$device_echo"
device_values="$(printf '%s\n' "$device_echo" | tr -d '\r' | numeric_values)"
assert_increasing "device echo (rc=$device_echo_rc)" "$device_values"
# --no-rebind refuses to replace an existing forward: a port already bound by a
# foreign caller makes this run fail clearly instead of hijacking it, and
# cleanup removes only a forward THIS invocation added.
fwd_rc=0
fwd_out="$("${ADB[@]}" forward --no-rebind tcp:7683 tcp:7683 2>&1)" || fwd_rc=$?
(( fwd_rc == 0 )) ||
  fail "adb forward" "tcp:7683 is already forwarded (this run refuses to replace it): $fwd_out"
added_forward=1
list_rc=0
list="$(host_list)" || list_rc=$?
(( list_rc == 0 || list_rc == 124 )) ||
  fail "host topic list" "cerulion topic list exited rc=$list_rc: $list"
printf '%s\n' "$list"
grep -q '/basic_timer/timer/count' <<<"$list" ||
  fail "host topic list" "timer topic was absent"
pass "host topic list" "timer topic present"
host_echo_rc=0
host_echo_output="$(host_echo /basic_timer/timer/count)" || host_echo_rc=$?
printf 'rc=%s\n%s\n' "$host_echo_rc" "$host_echo_output" > "$OUT/host-echo.txt"
(( host_echo_rc == 0 || host_echo_rc == 124 )) ||
  fail "host echo" "rc=$host_echo_rc output=$host_echo_output"
host_values="$(printf '%s\n' "$host_echo_output" | numeric_values)"
assert_increasing "host echo (rc=$host_echo_rc)" "$host_values"
device "test -s '$GRAPH_PID' && kill -INT \$(cat '$GRAPH_PID')" >/dev/null 2>&1 || true
wait_remote_dead "$GRAPH_PID"
started_graph=0
graph_stopped_ms="$(date +%s%3N)"
bag_remote="$(device "find '$DEV/record' -type f -name '*.mcap' | head -1" | tr -d '\r')"
[[ -n "$bag_remote" ]] || fail "recording file" "no MCAP was created"
"${ADB[@]}" pull "$bag_remote" "$OUT/phase-a.mcap" >/dev/null
bag_info="$("${HOST_ENV[@]}" "$HOST_CER" bag info "$OUT/phase-a.mcap" 2>&1)" ||
  fail "phase A bag info" "$bag_info"
bag_row="$(awk -v t=/basic_timer/timer/count '$1 == t { print $2, $NF; exit }' <<<"$bag_info")"
read -r bag_frames bag_schema <<<"$bag_row"
expected=$(((graph_stopped_ms - graph_started_ms) / period_ms))
floor=$((expected / 2))
(( floor < 3 )) && floor=3
[[ "$bag_frames" =~ ^[0-9]+$ && bag_frames -ge floor ]] ||
  fail "phase A bag frames" "frames=${bag_frames:-none}, floor=$floor"
[[ "$bag_schema" == std_msgs/Int32 ]] ||
  fail "phase A bag schema" "schema=${bag_schema:-none}, expected=std_msgs/Int32"
pass "phase A bag" "frames=$bag_frames floor=$floor schema=$bag_schema"
device "test -s '$NETD_PID' && kill -TERM \$(cat '$NETD_PID')" >/dev/null 2>&1
wait_remote_dead "$NETD_PID"
started_netd=0
"${ADB[@]}" forward --remove tcp:7683 >/dev/null
added_forward=0

APK="$ROOT/crates/cerulion_android/android/app/build/outputs/apk/debug/app-debug.apk"
"${ADB[@]}" install -r "$APK" >/dev/null
device "run-as '$APP' rm -f files/stop"
"${ADB[@]}" logcat -c
fwd_rc=0
fwd_out="$("${ADB[@]}" forward --no-rebind tcp:7683 tcp:7683 2>&1)" || fwd_rc=$?
(( fwd_rc == 0 )) ||
  fail "adb forward" "tcp:7683 is already forwarded (this run refuses to replace it): $fwd_out"
added_forward=1
device "am start -n '$APP/.MainActivity'" >/dev/null
started_app=1
deadline=$((SECONDS + 30))
paired_list=
ready=0
while (( SECONDS < deadline )); do
  remaining=$((deadline - SECONDS))
  (( remaining > 0 )) || break
  paired_list="$(host_list "$remaining" || true)"
  if grep -q 'robot refused to serve its topic catalog (not authorized)' <<<"$paired_list"; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "paired catalog refusal" "$paired_list"
pass "paired catalog refusal" "not authorized"
# The announce token can lag the refused catalog reply by a discovery beat:
# bound-retry the echo while it reports the topic "not found", then require
# the expected refusal (a pass needs the refusal text, not any failure).
paired_rc=0
paired_echo=
deadline=$((SECONDS + 15))
while :; do
  paired_echo="$(host_echo /app/heartbeat)" || paired_rc=$?
  if grep -q "not found locally or on any discovered robot" <<<"$paired_echo" && (( SECONDS < deadline )); then
    sleep 1
    continue
  fi
  break
done
printf 'rc=%s\n%s\n' "$paired_rc" "$paired_echo" > "$OUT/paired-echo.txt"
paired_ticks="$(printf '%s\n' "$paired_echo" | tick_values)"
[[ -z "$paired_ticks" ]] ||
  fail "paired heartbeat" "rc=$paired_rc decoded_frames=$paired_ticks output=$paired_echo"
[[ "$paired_rc" != 0 && "$paired_rc" != 124 ]] ||
  fail "paired heartbeat" "rc=$paired_rc decoded_frames=0 output=$paired_echo"
grep -Fq 'refused to serve its topic catalog (not authorized)' <<<"$paired_echo" ||
  fail "paired heartbeat" "rc=$paired_rc decoded_frames=0 output=$paired_echo"
pass "paired heartbeat" "decoded_frames=0 rc=$paired_rc refused=not authorized"
device "am force-stop '$APP'"
device "am start -n '$APP/.MainActivity' --es lan_access open" >/dev/null
deadline=$((SECONDS + 30))
open_list=
ready=0
while (( SECONDS < deadline )); do
  remaining=$((deadline - SECONDS))
  (( remaining > 0 )) || break
  open_list="$(host_list "$remaining" || true)"
  if grep -q '/app/heartbeat' <<<"$open_list"; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "open catalog" "heartbeat topic was not listed"
pass "open catalog" "heartbeat topic present"
open_rc=0
open_echo="$(host_echo /app/heartbeat)" || open_rc=$?
printf 'rc=%s\n%s\n' "$open_rc" "$open_echo" > "$OUT/open-echo.txt"
(( open_rc == 0 || open_rc == 124 )) ||
  fail "open heartbeat" "rc=$open_rc output=$open_echo"
open_ticks="$(printf '%s\n' "$open_echo" | tick_values)"
assert_increasing "open heartbeat" "$open_ticks"
app_bag="$(device logcat -d -s CerulionSample:I 2>/dev/null | tr -d '\r' |
  grep '"lan_access":"open"' | grep -o '"record_path":"[^"]*\.mcap"' | tail -1 |
  sed 's/.*"\([^"]*\)"$/\1/')"
app_bag="${app_bag##*/}"
[[ "$app_bag" =~ ^[A-Za-z0-9._-]+\.mcap$ ]] ||
  fail "app bag path" "no record_path in the open run status: ${app_bag:-none}"
pass "app bag path" "file=$app_bag"
device "run-as '$APP' touch files/stop"
deadline=$((SECONDS + 20))
ready=0
while (( SECONDS < deadline )); do
  if device logcat -d 2>/dev/null | tr -d '\r' | grep 'stop=0' >/dev/null; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "app stop" "stop=0 was not logged"
pass "app stop" "stop=0"
mcap_magic='89 4d 43 41 50 30 0d 0a'
deadline=$((SECONDS + 20))
trailer=
ready=0
while (( SECONDS < deadline )); do
  trailer="$("${ADB[@]}" exec-out run-as "$APP" tail -c 8 "files/bags/$app_bag" 2>/dev/null |
    od -An -tx1 | tr -s ' \n' ' ' | sed 's/^ //; s/ $//' || true)"
  if [[ "$trailer" == "$mcap_magic" ]]; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "app bag finalized" "trailing MCAP magic absent: ${trailer:-none}"
pass "app bag finalized" "trailing MCAP magic present"
"${ADB[@]}" exec-out run-as "$APP" cat "files/bags/$app_bag" > "$OUT/app.mcap"
# The recording now lives under $OUT; remove it from the device.
device "run-as $APP rm -f 'files/bags/$app_bag'" 
app_info="$("${HOST_ENV[@]}" "$HOST_CER" bag info "$OUT/app.mcap" 2>&1)" ||
  fail "app bag info" "$app_info"
app_row="$(awk -v t=/app/heartbeat '$1 == t { print $2, $NF; exit }' <<<"$app_info")"
read -r app_frames app_schema <<<"$app_row"
minimum=$(printf '%s\n' "$open_ticks" | sed '/^[[:space:]]*$/d' | wc -l)
[[ "$app_frames" =~ ^[0-9]+$ && app_frames -ge minimum ]] ||
  fail "app bag frames" "frames=${app_frames:-none}, minimum=$minimum"
[[ "$app_schema" == std_msgs/String ]] ||
  fail "app bag schema" "schema=${app_schema:-none}, expected=std_msgs/String"
pass "app bag" "frames=$app_frames minimum=$minimum schema=$app_schema"
app_log="$(device logcat -d 2>/dev/null | tr -d '\r')"
grep -q 'start=0' <<<"$app_log" || fail "app start" "start=0 absent"
if grep 'avc: denied' <<<"$app_log" | grep 'scontext=u:r:untrusted_app' | grep -v 'runas_app' >/dev/null; then
  fail "app SELinux" "untrusted_app denial was logged"
fi
pass "app lifecycle and SELinux" "start=0, no app-context avc denial"
echo "android e2e: $CHECKS checks passed"
