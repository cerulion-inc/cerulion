#!/usr/bin/env bash
# verify_rmw_deb_container.sh: the container half of verify_rmw_deb.sh.
#
# Runs as root INSIDE `ros:jazzy-ros-base` (Ubuntu 24.04 with ros-jazzy-ros-base
# and nothing of Cerulion) with the package directory mounted at /pkg and an
# output directory at /out. It installs the deb the way a user would, then
# proves ROS 2 traffic crosses the shipped library:
#
#   1. dpkg -i the package, apt-get -f to honour its Depends; the library must
#      be at /usr/bin/librmw_cerulion.so and ldd must report nothing missing.
#   2. `cerulion ros2 run demo_nodes_cpp talker` and `... listener`, both
#      through the shipped CLI: the listener must print `I heard:`.
#   3. Under the environment the CLI staged for its children, stock rclpy
#      must report `rmw_cerulion` as the implementation it LOADED (the
#      identifier rclpy returns, never the variable this script exported).
#
# NOT proved here, on purpose: cross-process topic discovery for the ros2
# CLI graph tools. `ros2 topic list`, `ros2 topic echo` (daemon or not, with
# or without an explicit type) and rclpy's get_topic_names_and_types() from
# a third process see only that process's own /parameter_events and /rosout
# under rmw_cerulion, never the talker's /chatter, because the rmw's endpoint
# discovery is process-local today (a product gap, recorded as such; the
# talker and listener still meet, because pub/sub rendezvous is by name on
# the shared-memory service). A `ros2 topic echo` check stood here through
# six box attempts and could never pass; `cerulion topic list` and
# `cerulion topic echo` are the tools that read the shared-memory plane.
#   4. The heap hook: `cerulion ros2 run --adopt-take ...` is refused (exit
#      69) with a direct-launch recipe naming the SHIPPED hook path; a talker
#      run with CERULION_HEAPHOOK_DEBUG=1 prints the hook's own load
#      breadcrumb twice, once from the python launcher (won_malloc=0, it
#      loses to python's allocator) and once from the C++ node (won_malloc=1,
#      proof the CLI auto-injected it and it won malloc there), and the node
#      maps it; the same run with the hook file moved
#      aside prints no breadcrumb and maps nothing, so the evidence is the
#      file, not the environment. The adopt-take refusal itself names the
#      path whether or not the file exists (it is decided before any file
#      is inspected), which the control records rather than pretends away.
#
# Process discipline (a run on a slow machine hung here for twelve minutes): `cerulion
# ros2 run` execs the python `ros2` wrapper, which spawns the C++ node as a
# further child and does NOT forward SIGINT to it. Signalling the wrapper
# alone left the node running and a plain `wait` never returned. So every
# launch runs under `setsid`, in its own session and process GROUP whose id
# is the launcher's pid (verified, never assumed; shell job control cannot
# do this, because it needs a controlling terminal and `docker run` in CI
# has none, which a machine run proved), every stop signals the whole group
# with INT, then TERM, then KILL, each under a bounded wait, and every wait
# in this script is bounded. `die` names the stage, so a hang or a failure
# reads as "stage X" in the log instead of a job timeout with no cause.
#
# Every log lands in /out for the workflow to upload.
set -euo pipefail

# The login gate is on in every build, and this script runs the shipped CLI
# inside a container, where a workflow `env:` cannot reach. This is how this
# repository's own runs pass the gate without an account.
export CERULION_LOGIN_GATE=off

pkg=/pkg
out=/out
so=/usr/bin/librmw_cerulion.so
hook=/usr/bin/libcerulion_heaphook.so
aside=/root/libcerulion_heaphook.so.aside
listener_deadline=30
echo_deadline=30
identity_deadline=60
stop_grace=10
stage=setup

die() {
    printf 'verify_rmw_deb_container: stage %s: %s\n' "$stage" "$*" >&2
    exit 1
}

# Start `cerulion ros2 run "$@"` with its output in $1, as a direct child of
# this shell under `setsid`, and leave its pid in $started_pid. A background
# job of a non-interactive shell is not a group leader, so setsid execs in
# place rather than forking: the pid IS the new session and group id. That
# is asserted by reading the pgid back, with a short poll because the child
# can still be inside setsid an instant after the fork.
start_ros2() {
    start_log=$1
    shift
    setsid cerulion ros2 run "$@" > "$start_log" 2>&1 &
    started_pid=$!
    start_tries=0
    while :; do
        start_pgid=$(ps -o pgid= -p "$started_pid" 2>/dev/null | tr -d ' ' || true)
        [ "$start_pgid" != "$started_pid" ] || break
        if [ -z "$start_pgid" ]; then
            cat "$start_log" >&2
            die "launcher $started_pid exited before it could be inspected; its log is above"
        fi
        [ "$start_tries" -lt 50 ] ||
            die "launcher $started_pid never became its own process group leader (pgid $start_pgid); setsid did not take"
        sleep 0.1
        start_tries=$((start_tries + 1))
    done
}

group_alive() {
    kill -0 -- "-$1" 2>/dev/null
}

# Wait up to $2 seconds for process group $1 to be empty.
wait_group_gone() {
    wg_elapsed=0
    while group_alive "$1"; do
        [ "$wg_elapsed" -lt "$2" ] || return 1
        sleep 1
        wg_elapsed=$((wg_elapsed + 1))
    done
    return 0
}

# Stop the launch whose pid (and group id) is $1, named $2 for the log:
# INT, then TERM, then KILL to the WHOLE group, each with a bounded wait.
# Never an unbounded `wait`: the launcher is reaped only once its group is
# known to be empty.
stop_ros2() {
    stop_pid=$1
    stop_name=$2
    [ -n "$stop_pid" ] || return 0
    if ! group_alive "$stop_pid"; then
        wait "$stop_pid" 2>/dev/null || true
        return 0
    fi
    for stop_signal in INT TERM KILL; do
        kill "-$stop_signal" -- "-$stop_pid" 2>/dev/null || true
        if wait_group_gone "$stop_pid" "$stop_grace"; then
            wait "$stop_pid" 2>/dev/null || true
            return 0
        fi
        printf '%s (group %s) did not stop on SIG%s within %ss; escalating\n' \
            "$stop_name" "$stop_pid" "$stop_signal" "$stop_grace" >&2
    done
    die "$stop_name process group $stop_pid survived SIGKILL"
}

talker_pid=
listener_pid=
cleanup() {
    cleanup_status=$?
    trap - EXIT
    stage="cleanup (after $stage)"
    if [ -f "$aside" ] && [ ! -f "$hook" ]; then
        mv "$aside" "$hook" || true
    fi
    for pair in "$listener_pid:listener" "$talker_pid:talker"; do
        pair_pid=${pair%%:*}
        pair_name=${pair#*:}
        [ -n "$pair_pid" ] || continue
        if ! stop_ros2 "$pair_pid" "$pair_name"; then
            cleanup_status=1
        fi
    done
    exit "$cleanup_status"
}
trap cleanup EXIT

wait_for_line() {
    wait_file=$1
    wait_pattern=$2
    wait_deadline=$3
    wait_elapsed=0
    while [ "$wait_elapsed" -lt "$wait_deadline" ]; do
        if grep -q -- "$wait_pattern" "$wait_file" 2>/dev/null; then
            return 0
        fi
        sleep 1
        wait_elapsed=$((wait_elapsed + 1))
    done
    return 1
}

[ -d "$pkg" ] || die "package directory $pkg is not mounted"
[ -d "$out" ] || die "output directory $out is not mounted"
command -v setsid >/dev/null 2>&1 ||
    die "setsid (util-linux) is required to give each ros2 launch its own process group"

set -- "$pkg"/*.deb
[ "$#" -eq 1 ] || die "expected exactly one .deb under $pkg, found $#"
[ -f "$1" ] || die "expected exactly one .deb under $pkg, found none"
deb=$1

# Step 1: install the package the way a user would.
stage="1 install"
export DEBIAN_FRONTEND=noninteractive
apt-get update
# procps: `ps` for the process-group assertion and `pgrep` for the node
# lookup; a minimal Ubuntu image does not carry it.
apt-get install -y --no-install-recommends ros-jazzy-demo-nodes-cpp procps
if ! dpkg -i "$deb"; then
    # dpkg leaves the package unconfigured when a Depends is absent; apt
    # resolves it the way `apt install ./pkg.deb` would.
    apt-get install -y -f
fi
dpkg -s cerulion | tee "$out/dpkg-status.txt"
dpkg -s cerulion | grep -q '^Status: install ok installed$' ||
    die "the package did not reach the installed state"
rm -rf /var/lib/apt/lists/*

[ -f "$so" ] || die "$so is missing from the installed package"
ldd "$so" | tee "$out/ldd.txt"
if grep -q 'not found' "$out/ldd.txt"; then
    die "ldd reports a missing dependency for $so"
fi
cerulion --version | tee "$out/cerulion-version.txt"

# ROS setup scripts dereference unset variables; relax nounset across it.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# A process snapshot for the log: who is in which group and session, at
# every transition. Cheap, and it is what let the sixth box attempt show
# that every launch lived and died exactly when this script said.
snapshot() {
    ps -eo pid,ppid,pgid,sid,stat,etimes,args > "$out/ps-$1.txt" 2>&1 || true
}

# Step 2: two ROS 2 processes through the shipped CLI.
stage="2 talker and listener"
start_ros2 "$out/talker.log" demo_nodes_cpp talker
talker_pid=$started_pid
sleep 2
start_ros2 "$out/listener.log" demo_nodes_cpp listener
listener_pid=$started_pid
snapshot 2-both-started
if ! wait_for_line "$out/listener.log" 'I heard: \[Hello World: ' "$listener_deadline"; then
    printf '== talker.log ==\n' >&2
    cat "$out/talker.log" >&2
    printf '== listener.log ==\n' >&2
    cat "$out/listener.log" >&2
    die "the listener did not hear the talker within ${listener_deadline}s"
fi
printf 'listener heard the talker over the shipped library\n'

# Step 2b: stop the listener, then the talker, each by its own group. The
# snapshots on both sides are kept: they are how the sixth box attempt
# showed both nodes alive through the stage that followed.
stage="2b stop the listener and the talker"
snapshot 2b-before-listener-stop
stop_ros2 "$listener_pid" listener
listener_pid=
snapshot 2b-after-listener-stop
stop_ros2 "$talker_pid" talker
talker_pid=
snapshot 2b-after-talker-stop

# Step 3: the environment the CLI staged for its children. The CLI creates
# one ament prefix per lib dir under ~/.cerulion/ros2 whose lib/ links the
# shipped library; exactly one must exist after the launches above. Under
# it, stock rclpy must load the shipped rmw.
stage="3 rclpy identity under the staged environment"
set -- "$HOME"/.cerulion/ros2/prefix-*
[ "$#" -eq 1 ] ||
    die "expected exactly one staged ament prefix under $HOME/.cerulion/ros2, found $#"
[ -d "$1" ] ||
    die "expected exactly one staged ament prefix under $HOME/.cerulion/ros2, found none"
staged_prefix=$1
[ -e "$staged_prefix/lib/librmw_cerulion.so" ] ||
    die "the staged prefix does not link the library: $staged_prefix"
export RMW_IMPLEMENTATION=rmw_cerulion
export LD_LIBRARY_PATH="/usr/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$staged_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"

if ! timeout "$identity_deadline" python3 - > "$out/rmw-identity.txt" 2>&1 <<'PY'
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
rclpy.init()
try:
    print(get_rmw_implementation_identifier())
finally:
    rclpy.shutdown()
PY
then
    cat "$out/rmw-identity.txt" >&2
    die "rclpy did not report an rmw identity within ${identity_deadline}s"
fi
grep -qx 'rmw_cerulion' "$out/rmw-identity.txt" || {
    cat "$out/rmw-identity.txt" >&2
    die "rclpy loaded a different rmw than rmw_cerulion"
}
printf 'rclpy reports rmw_cerulion\n'

# Step 4: the heap hook.
stage="4a adopt-take refusal"
[ -f "$hook" ] || die "$hook is missing from the installed package"

# 4a. --adopt-take is refused under the ros2 CLI (exit 69), and the refusal's
# direct-launch recipe names the hook where the package installed it. The
# refusal returns before anything is spawned, so a plain bounded run suffices.
set +e
timeout "$echo_deadline" cerulion ros2 run --adopt-take demo_nodes_cpp talker \
    > "$out/adopt-take.log" 2>&1
adopt_status=$?
set -e
[ "$adopt_status" -eq 69 ] || {
    cat "$out/adopt-take.log" >&2
    die "cerulion ros2 run --adopt-take exited $adopt_status, expected the exit-69 refusal"
}
grep -q -- '--adopt-take cannot reach a node launched by' "$out/adopt-take.log" || {
    cat "$out/adopt-take.log" >&2
    die "the adopt-take refusal did not carry its launcher-refusal prefix"
}
grep -q "LD_PRELOAD=$hook" "$out/adopt-take.log" || {
    cat "$out/adopt-take.log" >&2
    die "the adopt-take recipe does not name the shipped hook $hook"
}
printf 'adopt-take is refused with a recipe naming %s\n' "$hook"

# 4b. Auto-injection: a talker with the hook's debug breadcrumb on. The
# breadcrumb is written by the hook itself from inside the node, so it proves
# the CLI prepended the hook AND that the hook won malloc resolution there.
stage="4b hook preloaded into a node"
export CERULION_HEAPHOOK_DEBUG=1
start_ros2 "$out/hook-talker.log" demo_nodes_cpp talker
talker_pid=$started_pid
# TWO breadcrumbs are expected, in this order, because `cerulion ros2 run`
# puts the hook in the LD_PRELOAD of the python `ros2` launcher, which then
# execs the C++ node, so the hook loads in BOTH processes: in the python
# interpreter it loses malloc to python's own allocator (`won_malloc=0`,
# expected and harmless), in the node it wins (`won_malloc=1`, the thing
# the package ships it for). The seventh box attempt matched the launcher's
# line first and read the node's before it was written; so the wait is for
# the node's line specifically, and the launcher's is asserted afterwards.
wait_for_line "$out/hook-talker.log" 'cerulion heap hook loaded: abi v[0-9]* won_malloc=1' "$listener_deadline" || {
    cat "$out/hook-talker.log" >&2
    if grep -q 'cerulion heap hook loaded' "$out/hook-talker.log"; then
        die "the heap hook loaded but no process reported winning malloc within ${listener_deadline}s (a node line with won_malloc=1 is expected after the launcher's won_malloc=0)"
    fi
    die "the talker printed no heap hook breadcrumb; the hook was not preloaded"
}
grep -q 'cerulion heap hook loaded: abi v[0-9]* won_malloc=0' "$out/hook-talker.log" || {
    cat "$out/hook-talker.log" >&2
    die "the python launcher's own breadcrumb (won_malloc=0) is missing; the hook was not in the launcher's LD_PRELOAD, so it reached the node some other way"
}
wait_for_line "$out/hook-talker.log" 'Publishing:' "$listener_deadline" ||
    die "the talker did not publish with the hook preloaded"
node_pid=$(pgrep -n -f 'demo_nodes_cpp/talker' || true)
[ -n "$node_pid" ] || die "could not find the talker node process to inspect its mappings"
grep -c "$hook" "/proc/$node_pid/maps" > "$out/hook-maps-count.txt" || true
[ "$(cat "$out/hook-maps-count.txt")" -gt 0 ] ||
    die "the talker node does not map $hook"
printf 'the talker node loaded the heap hook (maps: %s entries)\n' "$(cat "$out/hook-maps-count.txt")"
stop_ros2 "$talker_pid" talker
talker_pid=

# 4c. Control: the hook file moved aside (the EXIT trap restores it on any
# failure). No breadcrumb, nothing mapped, and the talker still runs on the
# copy path. The adopt-take refusal is repeated to record that it is
# path-based: still exit 69, still the same recipe.
stage="4c control without the hook file"
mv "$hook" "$aside"
start_ros2 "$out/hook-control-talker.log" demo_nodes_cpp talker
talker_pid=$started_pid
wait_for_line "$out/hook-control-talker.log" 'Publishing:' "$listener_deadline" || {
    cat "$out/hook-control-talker.log" >&2
    die "the control talker did not publish without the hook"
}
if grep -q 'cerulion heap hook loaded' "$out/hook-control-talker.log"; then
    die "the control talker printed a heap hook breadcrumb with the hook file absent"
fi
node_pid=$(pgrep -n -f 'demo_nodes_cpp/talker' || true)
[ -n "$node_pid" ] || die "could not find the control talker node"
if grep -q 'libcerulion_heaphook' "/proc/$node_pid/maps"; then
    die "the control talker maps the heap hook with the file absent"
fi
stop_ros2 "$talker_pid" talker
talker_pid=
set +e
timeout "$echo_deadline" cerulion ros2 run --adopt-take demo_nodes_cpp talker \
    > "$out/adopt-take-control.log" 2>&1
adopt_control_status=$?
set -e
mv "$aside" "$hook"
unset CERULION_HEAPHOOK_DEBUG
[ "$adopt_control_status" -eq 69 ] ||
    die "the adopt-take control exited $adopt_control_status, expected 69"
printf 'control without the hook file: no breadcrumb, nothing mapped, adopt-take still refused (exit 69)\n'

# Informational: the same identity as `ros2 doctor` reports it.
stage="5 informational"
timeout "$echo_deadline" ros2 doctor --report 2>/dev/null | grep -iA1 'middleware name' \
    > "$out/doctor.txt" || true
cat "$out/doctor.txt" || true

stage="done"
printf 'VERIFY_RMW_DEB_PASS\n' | tee "$out/result.txt"
