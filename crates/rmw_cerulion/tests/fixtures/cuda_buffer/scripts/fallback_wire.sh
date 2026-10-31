#!/usr/bin/env bash
# Inside the container: per rmw, (1) the wire form of a CUDA-backed Image to a
# CUDA-capable subscriber, captured from the publisher's socket sends; (2) the
# CPU fallback when the subscriber runs under another uid (real uid 1001);
# (3) the fallback under the upstream test overrides for uid and device.
# Writes $OUT/fallback/<rmw>/<case>/{sub.csv,pub.strace,wire.json,*.log}.
set -eo pipefail
OUT=${1:?out dir}
BIN=/opt/s2/ws/install/lib/cuda_buffer_s2_probe
LAT="$BIN/s2_lat --size 4096 --count 20 --warmup 5 --rate 10"
STRACE="strace -f -qq -xx -s 65536 -e signal=none -e trace=sendto,sendmsg,sendmmsg,write,writev"
SETUP="source /opt/ros/lyrical/setup.bash; source /opt/s2/ws/install/setup.bash"
chmod 0755 "$OUT" 2>/dev/null || true

run_case() {  # rmw case sub_prefix ; env ORDER=sub_first|pub_first, PUB_UMASK, SUB_ENV
  local rmw=$1 case=$2 prefix=$3 dir="$OUT/fallback/$1/$2" spid ppid prc src
  mkdir -p "$dir"; chmod 0777 "$dir"
  # Each case starts with no host endpoint registry left by an earlier one
  # (this is the container's private /dev/shm).
  rm -f /dev/shm/ros2_hem_* /dev/shm/sem.ros2_hem_*
  export RMW_IMPLEMENTATION=$rmw
  local sub_cmd="$prefix bash -c '$SETUP; RMW_IMPLEMENTATION=$rmw ${SUB_ENV:-} $LAT --role sub --accept cuda --out $dir/sub.csv'"
  local pub_cmd="umask ${PUB_UMASK:-022}; timeout -k 5 60 $STRACE -o $dir/pub.strace $LAT --role pub"
  set +e
  if [ "${ORDER:-sub_first}" = sub_first ]; then
    eval "$sub_cmd" > "$dir/sub.log" 2>&1 & spid=$!
    sleep 2
    bash -c "$pub_cmd" > "$dir/pub.log" 2>&1; prc=$?
    wait $spid; src=$?
  else
    bash -c "$pub_cmd" > "$dir/pub.log" 2>&1 & ppid=$!
    sleep 3
    eval "$sub_cmd" > "$dir/sub.log" 2>&1; src=$?
    wait $ppid; prc=$?
  fi
  set -e
  python3 /opt/s2/fixtures/scripts/wire_extract.py "$dir/pub.strace" "$dir/wire.json" > /dev/null
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$rmw" "$case" "$prc" "$src" \
    "$(tail -n +2 "$dir/sub.csv" 2>/dev/null | cut -d, -f3 | sort | uniq -c | tr -s ' ' | tr '\n' ';')" \
    "$(python3 -c "import json;d=json.load(open('$dir/wire.json'));print(d['marker_hits'],d['legacy_4096_hits'])")" \
    >> "$OUT/fallback/summary.tsv"
  gzip -f "$dir/pub.strace"
}

mkdir -p "$OUT/fallback"
printf 'rmw\tcase\tpub_rc\tsub_rc\tsub_backends\tmarker_hits legacy_hits\n' > "$OUT/fallback/summary.tsv"
for rmw in rmw_fastrtps_cpp rmw_zenoh_cpp; do
  if [ "$rmw" = rmw_zenoh_cpp ]; then
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUT/fallback/zenohd.log" 2>&1 & ZPID=$!
    sleep 3
  else
    # LOCALHOST discovery makes rmw_fastrtps add its own SHM transport, and
    # same-host samples then never touch a socket. SUBNET discovery with
    # UDPv4 only and a static loopback peer puts every sample on a socket.
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=127.0.0.1 \
      FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  fi
  SUB_ENV="" ORDER=sub_first run_case $rmw same_uid ""
  SUB_ENV="" ORDER=sub_first run_case $rmw real_uid_1001_sub_first "runuser -u s2other --"
  SUB_ENV="" ORDER=pub_first run_case $rmw real_uid_1001_pub_first "runuser -u s2other --"
  SUB_ENV="" ORDER=pub_first PUB_UMASK=000 run_case $rmw real_uid_1001_pub_first_umask000 \
    "runuser -u s2other --"
  SUB_ENV="CUDA_BUFFER_UID_OVERRIDE=99999" ORDER=sub_first run_case $rmw uid_override ""
  SUB_ENV="CUDA_BUFFER_DEVICE_ID_OVERRIDE=999" ORDER=sub_first run_case $rmw device_override ""
  unset FASTDDS_BUILTIN_TRANSPORTS ROS_STATIC_PEERS
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  [ -n "${ZPID:-}" ] && { kill $ZPID; wait $ZPID 2>/dev/null || true; ZPID=; }
done
cat "$OUT/fallback/summary.tsv"
