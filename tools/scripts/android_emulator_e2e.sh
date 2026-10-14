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
DEV_CER="$DEV/cerulion"
DEV_NETD="$DEV/cerulion-netd"
GRAPH_PID="$DEV/graph.pid"
NETD_PID="$DEV/netd.pid"
CHECKS=0

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
host_list() {
  timeout "${1:-10}" "$HOST_CER" topic list --connect tcp/127.0.0.1:7683 2>&1 || true
}
host_echo() {
  local topic=$1
  CERULION_PEERS=127.0.0.1:7683 CERULION_NETD_CONNECT=tcp/127.0.0.1:7683 \
    timeout 8 "$HOST_CER" topic echo "$topic" 2>&1 || true
}
cleanup() {
  local rc=$?
  set +e
  device "test -s '$GRAPH_PID' && kill -INT \$(cat '$GRAPH_PID')" >/dev/null 2>&1
  device "test -s '$NETD_PID' && kill -TERM \$(cat '$NETD_PID')" >/dev/null 2>&1
  "${ADB[@]}" forward --remove-all >/dev/null 2>&1
  "${ADB[@]}" shell logcat -d > "$OUT/logcat.txt" 2>&1
  device "find '$DEV' -maxdepth 1 -type f -name '*.log' -exec sh -c 'for f; do printf \"--- %s ---\\n\" \"\$f\"; cat \"\$f\"; done' sh {} +" \
    > "$OUT/device-logs.txt" 2>&1
  device "am force-stop $APP" >/dev/null 2>&1
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

echo "android e2e: abi=$ABI triple=$TRIPLE device=${ANDROID_SERIAL:-default}"
device "rm -rf '$DEV' /data/local/tmp/iceoryx2 /data/local/tmp/cerulion-shm && mkdir -p '$DEV' '$DEV/basic_timer/target/$TRIPLE/debug'"
device "export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; mkdir -p \"\$HOME\" \"\$TMPDIR\""
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

device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open CERULION_NETD_LISTEN=tcp/0.0.0.0:7683; nohup '$DEV_NETD' > '$DEV/netd.log' 2>&1 & echo \$! > '$NETD_PID'"
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
device "cd '$DEV/basic_timer' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; nohup '$DEV_CER' graph run basic_timer --record='$DEV/record' > '$DEV/graph.log' 2>&1 & echo \$! > '$GRAPH_PID'"
sleep 2
device_echo="$(device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; timeout 8 '$DEV_CER' topic echo /basic_timer/timer/count" 2>&1 || true)"
device_values="$(printf '%s\n' "$device_echo" | tr -d '\r' | numeric_values)"
assert_increasing "device echo" "$device_values"
"${ADB[@]}" forward tcp:7683 tcp:7683 >/dev/null
list="$(host_list)"
printf '%s\n' "$list"
grep -q '/basic_timer/timer/count' <<<"$list" ||
  fail "host topic list" "timer topic was absent"
pass "host topic list" "timer topic present"
host_echo_output="$(host_echo /basic_timer/timer/count)"
host_values="$(printf '%s\n' "$host_echo_output" | numeric_values)"
assert_increasing "host echo" "$host_values"
device "test -s '$GRAPH_PID' && kill -INT \$(cat '$GRAPH_PID')" >/dev/null 2>&1 || true
wait_remote_dead "$GRAPH_PID"
graph_stopped_ms="$(date +%s%3N)"
bag_remote="$(device "find '$DEV/record' -type f -name '*.mcap' | head -1" | tr -d '\r')"
[[ -n "$bag_remote" ]] || fail "recording file" "no MCAP was created"
"${ADB[@]}" pull "$bag_remote" "$OUT/phase-a.mcap" >/dev/null
bag_info="$("$HOST_CER" bag info "$OUT/phase-a.mcap" 2>&1)" ||
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
"${ADB[@]}" forward --remove tcp:7683 >/dev/null

APK="$ROOT/crates/cerulion_android/android/app/build/outputs/apk/debug/app-debug.apk"
"${ADB[@]}" install -r "$APK" >/dev/null
"${ADB[@]}" logcat -c
"${ADB[@]}" forward tcp:7683 tcp:7683 >/dev/null
device "am start -n '$APP/.MainActivity'" >/dev/null
deadline=$((SECONDS + 30))
paired_list=
ready=0
while (( SECONDS < deadline )); do
  remaining=$((deadline - SECONDS))
  (( remaining > 0 )) || break
  paired_list="$(host_list "$remaining")"
  if grep -q 'robot refused to serve its topic catalog (not authorized)' <<<"$paired_list"; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "paired catalog refusal" "$paired_list"
pass "paired catalog refusal" "not authorized"
paired_echo="$(host_echo /app/heartbeat)"
paired_ticks="$(printf '%s\n' "$paired_echo" | tick_values)"
[[ -z "$paired_ticks" ]] || fail "paired heartbeat" "decoded frames were present: $paired_ticks"
pass "paired heartbeat" "decoded_frames=0"
device "am force-stop '$APP'"
device "am start -n '$APP/.MainActivity' --es lan_access open" >/dev/null
deadline=$((SECONDS + 30))
open_list=
ready=0
while (( SECONDS < deadline )); do
  remaining=$((deadline - SECONDS))
  (( remaining > 0 )) || break
  open_list="$(host_list "$remaining")"
  if grep -q '/app/heartbeat' <<<"$open_list"; then
    ready=1
    break
  fi
  sleep 1
done
(( ready == 1 )) || fail "open catalog" "heartbeat topic was not listed"
pass "open catalog" "heartbeat topic present"
open_echo="$(host_echo /app/heartbeat)"
open_ticks="$(printf '%s\n' "$open_echo" | tick_values)"
assert_increasing "open heartbeat" "$open_ticks"
max_tick=$(printf '%s\n' "$open_ticks" | tail -1)
first_tick=$(printf '%s\n' "$open_ticks" | head -1)
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
"${ADB[@]}" exec-out run-as "$APP" cat files/bags/app.mcap > "$OUT/app.mcap"
app_info="$("$HOST_CER" bag info "$OUT/app.mcap" 2>&1)" ||
  fail "app bag info" "$app_info"
app_row="$(awk -v t=/app/heartbeat '$1 == t { print $2, $NF; exit }' <<<"$app_info")"
read -r app_frames app_schema <<<"$app_row"
minimum=$((max_tick - first_tick))
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
