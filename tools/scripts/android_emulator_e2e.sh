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

RUNTIME_DIR="$ROOT/crates/cerulion_android/android/runtime"
RUNTIME_LIB=cerulion_sample_runtime
JNILIBS="$ROOT/crates/cerulion_android/android/app/src/main/jniLibs"

build() {
  cargo build -p cerulion_cli -p cerulion_netd
  # The direct data demand probe runs on the host, beside the host CLI.
  cargo build -p android_e2e_demand_probe
  cargo ndk -t "$ABI" --platform 30 build -p cerulion_cli -p cerulion_netd
  # The remote metadata probe runs on the device beside its netd. It is a
  # separate invocation: the example builds with cerulion_remoted's dev
  # dependencies, whose features must not reach the CLI and netd binaries.
  cargo ndk -t "$ABI" --platform 30 build -p cerulion_remoted --example android_e2e_remoted_probe
  (cd "$ROOT/examples/basic_timer" && cargo ndk -t "$ABI" --platform 30 build)
  # The app ships ONE native library: the sample runtime library, which
  # links cerulion_android and the graph's nodes. `cargo ndk -o` would also
  # copy each node crate's own library, so the build copies exactly the one
  # file, into a jniLibs directory emptied of anything an older build left.
  (cd "$RUNTIME_DIR" && cargo ndk -t "$ABI" --platform 30 build --release -p "$RUNTIME_LIB")
  # Cargo ran inside RUNTIME_DIR, so a relative CARGO_TARGET_DIR is relative
  # to that directory.
  local runtime_target="${CARGO_TARGET_DIR:-target}"
  [[ "$runtime_target" = /* ]] || runtime_target="$RUNTIME_DIR/$runtime_target"
  rm -rf "$JNILIBS"
  mkdir -p "$JNILIBS/$ABI"
  cp "$runtime_target/$TRIPLE/release/lib$RUNTIME_LIB.so" "$JNILIBS/$ABI/"
  bash "$SCRIPT_DIR/android_jnilibs_gate.sh" "$JNILIBS" "$RUNTIME_LIB"
  # shellcheck disable=SC2086
  gradle -p "$ROOT/crates/cerulion_android/android" assembleDebug ${GRADLE_ARGS:-}
}

# The run measures how long the graph and the app ran in milliseconds.
# `date +%s%3N` is GNU only: BSD and macOS date print a literal N. Bash 5
# keeps the time in $EPOCHREALTIME (whole seconds, the locale's radix
# character, six digits of microseconds), so no external tool is needed.
# The run already needs bash 4.3 or newer (`mapfile`, `${nums[-1]}`), and
# require_ms_clock refuses an older bash at start-up with the fix.
require_ms_clock() {
  if (( BASH_VERSINFO[0] < 5 )); then
    echo "FAIL millisecond clock: bash ${BASH_VERSION} has no \$EPOCHREALTIME; run this script with bash 5 or newer (on macOS: brew install bash, then run it with that bash)" >&2
    exit 1
  fi
}
# ms_from_epochrealtime VALUE: the milliseconds in one $EPOCHREALTIME value.
ms_from_epochrealtime() {
  if [[ ! "$1" =~ ^[0-9]+[^0-9][0-9]{6}$ ]]; then
    echo "FAIL millisecond clock: unexpected \$EPOCHREALTIME value '$1'" >&2
    exit 1
  fi
  local digits=${1//[!0-9]/}
  printf '%s\n' "${digits%???}"
}
now_ms() {
  ms_from_epochrealtime "$EPOCHREALTIME"
}
# self_test: the clock checks, with no device, cargo or adb.
self_test() {
  require_ms_clock
  local got want input first second wall
  while IFS='|' read -r input want; do
    got=$(ms_from_epochrealtime "$input")
    [[ "$got" == "$want" ]] ||
      { echo "FAIL ms_from_epochrealtime $input: got '$got', want '$want'" >&2; exit 1; }
  done <<'EOF'
1730000000.123456|1730000000123
1730000000,123456|1730000000123
1730000000.000999|1730000000000
1730000000.999999|1730000000999
EOF
  echo "PASS ms_from_epochrealtime: a dot and a comma radix give the same milliseconds"
  for input in 1730000000123N 1730000000.12345 1730000000 '' .123456; do
    if (ms_from_epochrealtime "$input") >/dev/null 2>&1; then
      echo "FAIL ms_from_epochrealtime refuses '$input': it printed a value" >&2
      exit 1
    fi
  done
  echo "PASS ms_from_epochrealtime: refuses a malformed value"
  first=$(now_ms)
  second=$(now_ms)
  wall=$(date +%s)
  [[ "$first" =~ ^[0-9]+$ && "$second" =~ ^[0-9]+$ ]] ||
    { echo "FAIL now_ms: not only digits: '$first' '$second'" >&2; exit 1; }
  (( second >= first )) ||
    { echo "FAIL now_ms: went back: $first then $second" >&2; exit 1; }
  (( first / 1000 - wall <= 2 && wall - first / 1000 <= 2 )) ||
    { echo "FAIL now_ms: $first ms is not within 2 s of date +%s ($wall s)" >&2; exit 1; }
  echo "PASS now_ms: digits only, $first then $second, within 2 s of date +%s"
}

if [[ "${1:-}" == build ]]; then
  build
  exit 0
fi
if [[ "${1:-}" == self-test ]]; then
  self_test
  exit 0
fi
if [[ "${1:-}" != run ]]; then
  echo "usage: $0 build|run|self-test" >&2
  exit 2
fi
require_ms_clock

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
DEV_REMOTED_PROBE="$DEV/android_e2e_remoted_probe"
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
PROBE="$ROOT/target/debug/android-e2e-demand-probe"
# The robot identity the sample app runs under, read from the app source the
# APK is built from. Every demand ack the probe counts must come from it.
APP_IDENTITY="$(sed -nE 's/.*CerulionRuntime\.init\(getFilesDir\(\)\.getAbsolutePath\(\), "([A-Za-z0-9_-]+)"\).*/\1/p' \
  "$ROOT/crates/cerulion_android/android/app/src/main/java/com/cerulion/sample/MainActivity.java" | head -1)"
[[ -n "$APP_IDENTITY" ]] ||
  fail "app identity" "could not read the robot name the sample app passes to CerulionRuntime.init"
PROBE_ENDPOINT=tcp/127.0.0.1:7683
# probe_refuses LABEL WANT ARG...: the probe refuses a bad argument, or an
# endpoint its session cannot open, before it demands anything, so this needs
# no device: exit 1, WANT in its output, and no report line.
probe_refuses() {
  local label=$1 want=$2 rc=0 out
  shift 2
  out="$(host_capture timeout 10 "${HOST_ENV[@]}" "$PROBE" "$@")" || rc=$?
  (( rc == 1 )) || fail "$label" "rc=$rc args=$* output=$out"
  grep -Fq -- "$want" <<<"$out" || fail "$label" "args=$* expected '$want' output=$out"
  if grep -q '^demand_probe ' <<<"$out"; then
    fail "$label" "args=$* printed a report: $out"
  fi
}
probe_argument_error() { probe_refuses "probe argument errors" "$@"; }
# run_probe LABEL EXPECT SCHEMA FILE: one direct data demand for the heartbeat
# over a 10 s window, judged by android_e2e_probe_verdict.sh. `topic echo`
# reads the catalog before it demands data; the probe demands without one.
run_probe() {
  local label=$1 expect=$2 schema=$3 file=$4 rc=0 out verdict
  out="$(host_capture timeout 30 "${HOST_ENV[@]}" "$PROBE" "$PROBE_ENDPOINT" /app/heartbeat \
    "$schema" 10)" || rc=$?
  printf 'rc=%s\n%s\n' "$rc" "$out" > "$OUT/$file"
  (( rc == 0 )) || fail "$label" "rc=$rc output=$out"
  verdict="$("$SCRIPT_DIR/android_e2e_probe_verdict.sh" "$expect" "$APP_IDENTITY" "$out")" ||
    fail "$label" "$verdict"
  pass "$label" "$verdict"
}
# run_remoted_probe LABEL FILE WANT ARG...: one run of the remote metadata
# probe on the device, with the device netd's environment and working
# directory (so it resolves the same control socket and shared-memory
# namespace). It serves the `cerulion-remoted` wire plane and sends a catalog
# and a schema request over loopback iroh; each answer is read from the netd
# serving snapshot through the deadline-bounded control connect. A pass needs
# exit code 0 and a fact line matching WANT.
run_remoted_probe() {
  local label=$1 file=$2 want=$3 rc=0 out fact
  shift 3
  out="$(device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open RUST_LOG=warn; timeout 150 '$DEV_REMOTED_PROBE' $*" 2>&1)" || rc=$?
  out="${out//$'\r'/}"
  printf 'rc=%s\n%s\n' "$rc" "$out" > "$OUT/$file"
  (( rc == 0 )) || fail "$label" "rc=$rc output=$out"
  fact="$(grep '^remoted_probe ' <<<"$out" || true)"
  [[ "$fact" =~ ^${want}$ ]] || fail "$label" "expected '$want', output=$out"
  pass "$label" "${fact#remoted_probe }"
}
# host_netd_pids: the pids of the live host processes named cerulion-netd. A
# zombie is dead: it holds no socket and answers nothing, so it is left out.
host_netd_pids() {
  ps -A -o pid= -o stat= -o comm= | awk '{
    pid = $1; stat = $2; $1 = ""; $2 = ""; sub(/^ +/, "")
    n = split($0, path, "/")
    if (path[n] == "cerulion-netd" && stat !~ /^Z/) print pid
  }'
}
# stop_host_netd: stop the host network daemon that `host_list` and
# `host_echo` spawned, then require that NO cerulion-netd is alive on the
# host. The probe runs with no other zenoh peer on the host: a live host netd
# would bridge the heartbeat into the shared memory the probe's subscriber
# reads (frames that never crossed the probe's ingress), and its own demand
# queryable can answer the probe's demand with `refused` for a topic it does
# not produce (an ack the paired verdict would count as the app's posture
# refusal). The check reads the process table, not the pid file: a netd whose
# pid file is missing or elsewhere still fails it. The run refused to start
# while a host netd was running, so any one alive now is this run's.
stop_host_netd() {
  local alive deadline
  HOST_NETD_OWNED && { kill -TERM "$(cat "$HOST_NETD_PID")" >/dev/null 2>&1 || true; }
  deadline=$((SECONDS + 20))
  while :; do
    alive="$(host_netd_pids | tr '\n' ' ')"
    [[ -n "${alive// /}" ]] || break
    (( SECONDS < deadline )) || fail "host netd stop" "a host netd is still alive: pids=${alive% }"
    sleep 1
  done
  rm -f "$HOST_NETD_PID"
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
# The demand probe must be the only zenoh peer on the host when it runs, and
# stop_host_netd can only prove that if no host netd ran before this run.
host_netd="$(host_netd_pids | tr '\n' ' ')"
[[ -z "${host_netd// /}" ]] ||
  fail "host netd free" "a cerulion-netd already runs on the host: pids=${host_netd% }"
pass "host netd free" "no host netd"
host_local="$("${HOST_ENV[@]}" "$HOST_CER" topic list --local 2>&1)" ||
  fail "host local topics" "$host_local"
if grep -qE '/basic_timer/timer/count|/app/heartbeat|/app/timer/count|/app/relay/echo' <<<"$host_local"; then
  fail "host local topics" "the host already publishes a test topic"
fi
pass "host local topics" "no local test publisher"
probe_argument_error 'usage: android-e2e-demand-probe'
probe_argument_error 'usage: android-e2e-demand-probe' "$PROBE_ENDPOINT" /app/heartbeat \
  std_msgs/String 10 extra
probe_argument_error "topic 'app/heartbeat' is not canonical" "$PROBE_ENDPOINT" app/heartbeat \
  std_msgs/String 10
probe_argument_error "schema 'std_msgs/Float64' is not one this probe knows" "$PROBE_ENDPOINT" \
  /app/heartbeat std_msgs/Float64 10
probe_argument_error "window 'ten' is not a whole number of seconds" "$PROBE_ENDPOINT" \
  /app/heartbeat std_msgs/String ten
probe_argument_error 'window must be at least one second' "$PROBE_ENDPOINT" /app/heartbeat \
  std_msgs/String 0
pass "probe argument errors" "cases=6 rc=1 no report"
# A connect endpoint zenoh cannot parse: the probe's session does not open.
probe_refuses "probe setup error" "the probe's session did not open" not-an-endpoint \
  /app/heartbeat std_msgs/String 1
pass "probe setup error" "rc=1 no report"

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
# Ownership is claimed atomically: a plain `mkdir` (no -p) fails when $DEV,
# or anything else at that path, already exists, so of two concurrent runs
# exactly one owns the directory. Only the winner sets created_dev, so a
# refused or losing run never removes a tree it did not create, and the
# fresh directory holds no PID files from anyone else.
claim="$(device "mkdir '$DEV'" 2>&1)" ||
  fail "device setup" "could not claim $DEV (it already exists or another run owns it); this run refuses to remove a directory it did not create: ${claim//$'\r'/}"
created_dev=1
device "mkdir -p '$DEV' '$DEV/basic_timer/target/$TRIPLE/debug' '$DEV/config' '$DEV/basic_timer/config' && mkdir -m 700 '$DEV/shm'" 
device "printf '[global]\nroot-path = \"%s/iceoryx2\"\n' '$DEV' > '$DEV/config/iceoryx2.toml' && cp '$DEV/config/iceoryx2.toml' '$DEV/basic_timer/config/iceoryx2.toml'"
device "export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open; mkdir -p \"\$HOME\" \"\$TMPDIR\""
# Serving needs a saved login even with CERULION_LOGIN_GATE=off. This is a
# fixture login: an expired session the serving gate accepts offline. The
# device netd points its account service at a closed loopback port, so no
# registration or refresh leaves the emulator.
device "mkdir -p '$DEV/home/.cerulion' && printf '%s' '{\"account_id\":\"00112233-4455-4677-8899-aabbccddeeff\",\"session_token\":\"e2e-expired-session\",\"refresh_token\":\"e2e-offline-refresh\",\"expires_at_ns\":0,\"logged_in_ever\":true}' > '$DEV/home/.cerulion/auth.json' && chmod 600 '$DEV/home/.cerulion/auth.json'"
"${ADB[@]}" push "$ROOT/target/$TRIPLE/debug/cerulion" "$DEV_CER" >/dev/null
"${ADB[@]}" push "$ROOT/target/$TRIPLE/debug/cerulion-netd" "$DEV_NETD" >/dev/null
"${ADB[@]}" push "$ROOT/target/$TRIPLE/debug/examples/android_e2e_remoted_probe" "$DEV_REMOTED_PROBE" >/dev/null
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
device "chmod 755 '$DEV_CER' '$DEV_NETD' '$DEV_REMOTED_PROBE'"
pass "device push" "node_libraries=${#node_libs[@]}"

device "cd '$DEV' && export HOME='$DEV/home' TMPDIR='$DEV/tmp' CERULION_SHM_DIR='$DEV/shm' CERULION_LOGIN_GATE=off CERULION_LAN_ACCESS=open CERULION_ACCOUNT_SERVICE=http://127.0.0.1:1 CERULION_NETD_LISTEN=tcp/0.0.0.0:7683; nohup '$DEV_NETD' > '$DEV/netd.log' 2>&1 & echo \$! > '$NETD_PID'"
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
graph_started_ms="$(now_ms)"
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
# Remote metadata through the robot's local netd while the graph serves the
# timer topic: the catalog names it std_msgs/Int32 from netd's snapshot (a
# topic binding or the wire-hash binding of its frames), the schema reply names
# a type for it too (a builtin, so no custom closure), and no socket file
# exists at the control path, so the reads came over the abstract namespace.
run_remoted_probe "remoted catalog and schema" remoted-probe-served.txt \
  'remoted_probe mode=served topic=/basic_timer/timer/count catalog_rows=[0-9]+ catalog_schema=std_msgs/Int32 schema_reply=builtin control_socket=abstract' \
  served /basic_timer/timer/count std_msgs/Int32
device "test -s '$GRAPH_PID' && kill -INT \$(cat '$GRAPH_PID')" >/dev/null 2>&1 || true
wait_remote_dead "$GRAPH_PID"
started_graph=0
graph_stopped_ms="$(now_ms)"
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
# With the device netd stopped, both reads fail and name the missing daemon.
run_remoted_probe "remoted reads without netd" remoted-probe-absent.txt \
  'remoted_probe mode=absent topic=/basic_timer/timer/count catalog=refused schema=refused' \
  absent /basic_timer/timer/count
"${ADB[@]}" forward --remove tcp:7683 >/dev/null
added_forward=0

APK="$ROOT/crates/cerulion_android/android/app/build/outputs/apk/debug/app-debug.apk"
# The APK carries exactly one native library, the runtime library that links
# Cerulion and the graph's nodes (the build step's gate checked its
# alignment and exports before Gradle packaged it).
mapfile -t apk_libs < <(unzip -Z1 "$APK" 'lib/*' 2>/dev/null | sed '/\/$/d')
[[ ${#apk_libs[@]} -eq 1 && "${apk_libs[0]}" == "lib/$ABI/lib$RUNTIME_LIB.so" ]] ||
  fail "app lib packaging" "expected only lib/$ABI/lib$RUNTIME_LIB.so in the APK, found: ${apk_libs[*]:-none}"
pass "app lib packaging" "${apk_libs[0]}"
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
grep -Fq 'refused to serve its topic catalog to LAN peers (not authorized)' <<<"$paired_echo" ||
  fail "paired heartbeat" "rc=$paired_rc decoded_frames=0 output=$paired_echo"
pass "paired heartbeat" "decoded_frames=0 rc=$paired_rc refused=not authorized"
# Paired direct data demand: a paired app must refuse every answered demand
# and send no frame. It runs twice, and both runs must give that verdict.
# With the host netd stopped, the app is the only peer left to answer.
stop_host_netd
run_probe "paired direct demand" paired std_msgs/String paired-demand-probe.txt
run_probe "paired direct demand, second run" paired std_msgs/String paired-demand-probe-2.txt

# The paired run ran the graph too. Keep its log, then kill the app: the
# open run below is a start after a kill, on top of whatever the killed
# process left in its files directory.
paired_pid="$(device "pidof '$APP'" | tr -d '\r')"
[[ "$paired_pid" =~ ^[0-9]+$ ]] || fail "app start after kill" "no pid for the paired run: ${paired_pid:-none}"
"${ADB[@]}" logcat -d > "$OUT/logcat-paired.txt" 2>&1
device "am force-stop '$APP'"
leftover="$(device "run-as '$APP' ls -1 files" | tr -d '\r')"
# The in-process transport makes no iceoryx2 tree; the runtime state a
# killed app leaves is the iceoryx2 config it wrote and its shm directory.
if ! grep -qx 'iceoryx2.toml' <<<"$leftover" || ! grep -qx 'cerulion-shm' <<<"$leftover"; then
  fail "app start after kill" "the killed run left no files/iceoryx2.toml and files/cerulion-shm: ${leftover//$'\n'/ }"
fi
"${ADB[@]}" logcat -c
open_started_ms="$(now_ms)"
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
printf '%s\n' "$open_list" > "$OUT/open-catalog.txt"
if ! grep -q '/app/timer/count' <<<"$open_list" || ! grep -q '/app/relay/echo' <<<"$open_list"; then
  fail "open node catalog" "the graph topics were not listed: $open_list"
fi
pass "open node catalog" "/app/timer/count and /app/relay/echo next to /app/heartbeat"
open_pid="$(device "pidof '$APP'" | tr -d '\r')"
[[ "$open_pid" =~ ^[0-9]+$ && "$open_pid" != "$paired_pid" ]] ||
  fail "app start after kill" "open run pid=${open_pid:-none}, paired run pid=$paired_pid"
open_rc=0
open_echo="$(host_echo /app/heartbeat)" || open_rc=$?
printf 'rc=%s\n%s\n' "$open_rc" "$open_echo" > "$OUT/open-echo.txt"
(( open_rc == 0 || open_rc == 124 )) ||
  fail "open heartbeat" "rc=$open_rc output=$open_echo"
open_ticks="$(printf '%s\n' "$open_echo" | tick_values)"
assert_increasing "open heartbeat" "$open_ticks"

# Graph node frames over the host echo. The timer counts up by its `step`
# param, 2 in the sample graph, so every value is even. The relay has a
# trigger input and an output, so its values prove frames entered a node.
assert_even() {
  local label=$1 values=$2 value
  for value in $values; do
    (( value % 2 == 0 )) || fail "$label" "odd value $value in: $(tr '\n' ' ' <<<"$values")"
  done
  pass "$label" "every value is even (timer step 2)"
}
node_echo() { # <topic> <evidence file> <label>
  local topic=$1 file=$2 label=$3 rc=0 output values
  output="$(host_echo "$topic")" || rc=$?
  printf 'rc=%s\n%s\n' "$rc" "$output" > "$OUT/$file"
  (( rc == 0 || rc == 124 )) || fail "$label" "rc=$rc output=$output"
  values="$(printf '%s\n' "$output" | numeric_values)"
  assert_increasing "$label (rc=$rc)" "$values"
  assert_even "$label even values" "$values"
}
node_echo /app/timer/count node-echo.txt "open timer echo"
node_echo /app/relay/echo relay-echo.txt "open relay echo"
status_lines() {
  device logcat -d -s CerulionSample:I 2>/dev/null | tr -d '\r' | grep '"lan_access":"open"'
}
open_status="$(status_lines | tail -1)"
if ! grep -q '"nodes":3' <<<"$open_status" ||
   ! grep -q '"linked_nodes":\["printer","relay","timer"\]' <<<"$open_status"; then
  fail "app linked nodes" "status=${open_status:-none}"
fi
pass "app linked nodes" "nodes=3 linked_nodes=printer,relay,timer pid=$open_pid (paired run pid=$paired_pid)"
# The runtime's own `tracing` lines reach logcat under the Cerulion tag: the
# open run listens on a fixed TCP port, so the runtime logs whether its mDNS
# beacon was raised, at info or warn.
runtime_line="$(device logcat -d --pid="$open_pid" -s Cerulion:V 2>/dev/null | tr -d '\r' |
  grep -m1 'cerulion_android::runtime: mDNS beacon' || true)"
[[ -n "$runtime_line" ]] ||
  fail "runtime line in logcat" "no Cerulion-tagged mDNS beacon line from pid $open_pid"
pass "runtime line in logcat" "$runtime_line"
pass "app start after kill" "start=0 in pid $open_pid after force-stop of pid $paired_pid, stale files tolerated"
# The open `host_list` and `host_echo` spawned a new host netd: stop it too.
stop_host_netd
# Open control: the same demand IS granted and delivered when the app is open.
# The app's refused ack does not say why it refused, so this is what ties the
# paired refusal to the posture and not to the topic, schema or allow-list.
run_probe "open direct demand" open std_msgs/String open-demand-probe.txt
# Edge: the topic is std_msgs/String, the probe expects std_msgs/Int32. The
# frames cross the network and the ingress drops every one on the schema.
run_probe "open demand, wrong schema" schema-mismatch std_msgs/Int32 \
  open-schema-mismatch-probe.txt
record_path_of() {
  grep -o '"record_path":"[^"]*\.mcap"' | sed 's/.*"\([^"]*\)"$/\1/'
}
app_bag="$(status_lines | record_path_of | tail -1)"
app_bag="${app_bag##*/}"
[[ "$app_bag" =~ ^[A-Za-z0-9._-]+\.mcap$ ]] ||
  fail "app bag path" "no record_path in the open run status: ${app_bag:-none}"
pass "app bag path" "file=$app_bag"
# Counts the open run's log lines that match: `logcat -c` does not always
# drop the killed paired run's lines, so the count is scoped to the open
# run's pid.
log_count() {
  device logcat -d --pid="$open_pid" -s CerulionSample:I 2>/dev/null | tr -d '\r' | grep -c "$1" || true
}
# Stops the app's runtime through files/stop and waits for the Nth stop=0.
stop_app() {
  local label=$1 want=$2 deadline
  device "run-as '$APP' touch files/stop"
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    (( $(log_count 'stop=0') >= want )) && return 0
    sleep 1
  done
  fail "$label" "stop=0 was not logged $want time(s)"
}
mcap_magic='89 4d 43 41 50 30 0d 0a'
# Waits for the closing MCAP magic, pulls the bag and removes it from the
# device.
pull_bag() {
  local label=$1 bag=$2 out=$3 deadline trailer='' ready=0
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    trailer="$("${ADB[@]}" exec-out run-as "$APP" tail -c 8 "files/bags/$bag" 2>/dev/null |
      od -An -tx1 | tr -s ' \n' ' ' | sed 's/^ //; s/ $//' || true)"
    if [[ "$trailer" == "$mcap_magic" ]]; then
      ready=1
      break
    fi
    sleep 1
  done
  (( ready == 1 )) || fail "$label" "trailing MCAP magic absent: ${trailer:-none}"
  pass "$label" "trailing MCAP magic present"
  "${ADB[@]}" exec-out run-as "$APP" cat "files/bags/$bag" > "$out"
  device "run-as $APP rm -f 'files/bags/$bag'"
}
# Prints "<frames> <schema>" for one topic of a pulled bag.
bag_row() {
  local info=$1 topic=$2
  awk -v t="$topic" '$1 == t { print $2, $NF; exit }' <<<"$info"
}
# Requires both graph topics in a pulled bag, as std_msgs/Int32 with at
# least half the frames the timer period allows in the run time (minimum 3).
assert_node_rows() {
  local label=$1 info=$2 run_ms=$3 floor topic frames schema
  floor=$((run_ms / period_ms / 2))
  (( floor < 3 )) && floor=3
  for topic in /app/timer/count /app/relay/echo; do
    read -r frames schema <<<"$(bag_row "$info" "$topic")"
    [[ "$frames" =~ ^[0-9]+$ && frames -ge floor ]] ||
      fail "$label" "$topic frames=${frames:-none}, floor=$floor"
    [[ "$schema" == std_msgs/Int32 ]] ||
      fail "$label" "$topic schema=${schema:-none}, expected=std_msgs/Int32"
  done
  pass "$label" "/app/timer/count and /app/relay/echo std_msgs/Int32, frames >= $floor"
}
stop_app "app stop" 1
open_stopped_ms="$(now_ms)"
pass "app stop" "stop=0"
pull_bag "app bag finalized" "$app_bag" "$OUT/app.mcap"
app_info="$("${HOST_ENV[@]}" "$HOST_CER" bag info "$OUT/app.mcap" 2>&1)" ||
  fail "app bag info" "$app_info"
printf '%s\n' "$app_info" > "$OUT/app-bag-info.txt"
read -r app_frames app_schema <<<"$(bag_row "$app_info" /app/heartbeat)"
minimum=$(printf '%s\n' "$open_ticks" | sed '/^[[:space:]]*$/d' | wc -l)
[[ "$app_frames" =~ ^[0-9]+$ && app_frames -ge minimum ]] ||
  fail "app bag frames" "frames=${app_frames:-none}, minimum=$minimum"
[[ "$app_schema" == std_msgs/String ]] ||
  fail "app bag schema" "schema=${app_schema:-none}, expected=std_msgs/String"
pass "app bag" "frames=$app_frames minimum=$minimum schema=$app_schema"
assert_node_rows "app bag node frames" "$app_info" $((open_stopped_ms - open_started_ms))

# Restart in the same process: a new intent reaches the stopped activity,
# which starts the runtime again with fresh nodes and a new recording.
restart_started_ms="$(now_ms)"
device "am start -n '$APP/.MainActivity' --es lan_access open" >/dev/null
deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  (( $(log_count 'start=0') >= 2 )) && break
  sleep 1
done
(( $(log_count 'start=0') == 2 )) ||
  fail "app restart same process" "expected start=0 twice, found $(log_count 'start=0')"
restart_pid="$(device "pidof '$APP'" | tr -d '\r')"
[[ "$restart_pid" == "$open_pid" ]] ||
  fail "app restart same process" "pid changed from $open_pid to ${restart_pid:-none}"
restart_bag=
deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  restart_bag="$(status_lines | record_path_of | tail -1)"
  restart_bag="${restart_bag##*/}"
  [[ -n "$restart_bag" && "$restart_bag" != "$app_bag" ]] && break
  sleep 1
done
[[ "$restart_bag" =~ ^[A-Za-z0-9._-]+\.mcap$ && "$restart_bag" != "$app_bag" ]] ||
  fail "app restart same process" "no new record_path after the restart: ${restart_bag:-none}"
# The host may need a discovery beat to see the restarted gateway's topics,
# so the echo is retried, bounded, until it decodes values.
restart_rc=0
restart_echo=
deadline=$((SECONDS + 30))
while :; do
  restart_rc=0
  restart_echo="$(host_echo /app/timer/count)" || restart_rc=$?
  restart_values="$(printf '%s\n' "$restart_echo" | numeric_values)"
  if [[ "$(sed '/^[[:space:]]*$/d' <<<"$restart_values" | wc -l)" -lt 3 ]] && (( SECONDS < deadline )); then
    sleep 1
    continue
  fi
  break
done
printf 'rc=%s\n%s\n' "$restart_rc" "$restart_echo" > "$OUT/restart-node-echo.txt"
(( restart_rc == 0 || restart_rc == 124 )) ||
  fail "app restart same process" "timer echo rc=$restart_rc output=$restart_echo"
assert_increasing "restart timer echo (rc=$restart_rc)" "$restart_values"
stop_app "app restart stop" 2
restart_stopped_ms="$(now_ms)"
pull_bag "app restart bag finalized" "$restart_bag" "$OUT/app-restart.mcap"
restart_info="$("${HOST_ENV[@]}" "$HOST_CER" bag info "$OUT/app-restart.mcap" 2>&1)" ||
  fail "app restart bag info" "$restart_info"
printf '%s\n' "$restart_info" > "$OUT/app-restart-bag-info.txt"
assert_node_rows "app restart bag node frames" "$restart_info" $((restart_stopped_ms - restart_started_ms))
pass "app restart same process" "start=0 and stop=0 twice in pid $open_pid, second bag $restart_bag"

"${ADB[@]}" logcat -d > "$OUT/logcat-open.txt" 2>&1
app_log="$(tr -d '\r' < "$OUT/logcat-open.txt")"
grep -q 'start=0' <<<"$app_log" || fail "app start" "start=0 absent"
denials="$(cat "$OUT/logcat-paired.txt" "$OUT/logcat-open.txt" | tr -d '\r' |
  grep 'avc: denied' | grep 'scontext=u:r:untrusted_app' | grep -v 'runas_app' || true)"
[[ -z "$denials" ]] || fail "app SELinux" "untrusted_app denial was logged: $denials"
pass "app lifecycle and SELinux" "start=0, no app-context avc denial in the paired, open and restart runs (no dlopen or execute denial)"
echo "android e2e: $CHECKS checks passed"
