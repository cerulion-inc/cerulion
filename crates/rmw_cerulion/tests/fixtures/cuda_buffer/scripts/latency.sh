#!/usr/bin/env bash
# Inside the container: the cuda_buffer_backend latency baseline. One-way
# publish -> payload usable on the subscriber's stream (see s2_lat.cpp), per
# rmw x mode x size. Resumable: a cell whose CSV exists is skipped.
#
# modes:
#   inter       publisher and subscriber in two processes, accept "cuda"
#   intra       one process, rclcpp intra-process comms on, accept "cuda"
#   proc1_rmw   one process, intra-process comms off (through the rmw), "cuda"
#   inter_cpu   two processes, subscriber accepts "cpu" only (the CPU path)
set -eo pipefail
OUT=${1:?out dir}/latency${S2_STAMP:+_$S2_STAMP}
SIZES=${S2_SIZES:-"4096 65536 1048576 4194304 16777216 67108864"}
MODES=${S2_MODES:-"inter intra proc1_rmw inter_cpu"}
RMWS=${S2_RMWS:-"rmw_fastrtps_cpp rmw_zenoh_cpp"}
COUNT=${S2_COUNT:-1000}
WARMUP=${S2_WARMUP:-100}
RATE=${S2_RATE:-50}
# enqueued: the stamp precedes the producer's GPU write; ready: it follows it.
STAMP=${S2_STAMP:-enqueued}
LAT=/opt/s2/ws/install/lib/cuda_buffer_s2_probe/s2_lat
mkdir -p "$OUT"
{
  echo "count=$COUNT warmup=$WARMUP rate_hz=$RATE stamp=$STAMP"
  echo "sizes=$SIZES"
  echo "modes=$MODES"
  echo "rmws=$RMWS"
  date -u +%FT%TZ
} > "$OUT/params.txt"

for rmw in $RMWS; do
  export RMW_IMPLEMENTATION=$rmw
  ZPID=
  if [ "$rmw" = rmw_zenoh_cpp ]; then
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUT/zenohd.log" 2>&1 & ZPID=$!
    sleep 3
  fi
  for mode in $MODES; do
    for size in $SIZES; do
      cell="$OUT/${rmw}__${mode}__${size}"
      [ -s "$cell.csv" ] && continue
      common="--size $size --count $COUNT --warmup $WARMUP --rate $RATE --stamp $STAMP"
      set +e
      case $mode in
        inter|inter_cpu)
          acc=cuda; [ $mode = inter_cpu ] && acc=cpu
          $LAT --role sub --accept $acc $common --out "$cell.tmp" > "$cell.sub.log" 2>&1 & spid=$!
          sleep 1
          $LAT --role pub $common > "$cell.pub.log" 2>&1
          wait $spid; rc=$? ;;
        intra)
          $LAT --role both --intra --accept cuda $common --out "$cell.tmp" > "$cell.log" 2>&1; rc=$? ;;
        proc1_rmw)
          $LAT --role both --accept cuda $common --out "$cell.tmp" > "$cell.log" 2>&1; rc=$? ;;
      esac
      set -e
      echo "$rmw $mode $size rc=$rc" >> "$OUT/cells.log"
      [ -f "$cell.tmp" ] && mv "$cell.tmp" "$cell.csv"
    done
  done
  [ -n "$ZPID" ] && { kill $ZPID; wait $ZPID 2>/dev/null || true; }
done
python3 /opt/s2/fixtures/scripts/latency_summary.py "$OUT" "$COUNT" "$WARMUP"
