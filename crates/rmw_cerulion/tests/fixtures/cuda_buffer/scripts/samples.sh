#!/usr/bin/env bash
# Inside the container: run the upstream cuda_buffer_backend sample launch
# tests on rmw_fastrtps_cpp (as shipped) and on rmw_zenoh_cpp (the same files
# with the rmw name swapped, a router started first). Writes per-test xunit
# and logs under $OUT/samples/<rmw>/ and a summary TSV.
set -eo pipefail
OUT=${1:?out dir}
SRC=/opt/s2/src/rosidl_buffer_backends/cuda_buffer_backend/cuda_buffer_backend/test
mkdir -p "$OUT/samples/rmw_fastrtps_cpp" "$OUT/samples/rmw_zenoh_cpp"
SUMMARY="$OUT/samples/summary.tsv"
printf 'rmw\ttest\trc\ttests\tfailures\terrors\n' > "$SUMMARY"

run_one() {  # rmw file
  local rmw=$1 f=$2 name dir rc
  name=$(basename "$f" .py)
  dir="$OUT/samples/$rmw"
  set +e
  timeout -k 10 120 launch_test "$f" --junit-xml "$dir/$name.xunit.xml" > "$dir/$name.log" 2>&1
  rc=$?
  set -e
  python3 - "$rmw" "$name" "$rc" "$dir/$name.xunit.xml" >> "$SUMMARY" <<'PY'
import sys, xml.etree.ElementTree as ET
rmw, name, rc, path = sys.argv[1:]
t = f = e = -1
try:
    r = ET.parse(path).getroot()
    suites = [r] if r.tag == 'testsuite' else list(r)
    t = sum(int(s.get('tests', 0)) for s in suites)
    f = sum(int(s.get('failures', 0)) for s in suites)
    e = sum(int(s.get('errors', 0)) for s in suites)
except Exception:
    pass
print(f"{rmw}\t{name}\t{rc}\t{t}\t{f}\t{e}")
PY
}

for f in "$SRC"/test_*_fastrtps_launch.py; do
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp run_one rmw_fastrtps_cpp "$f"
done

ZDIR=$(mktemp -d)
for f in "$SRC"/test_*_fastrtps_launch.py; do
  sed 's/rmw_fastrtps_cpp/rmw_zenoh_cpp/g; s/FastRTPS/Zenoh/g' "$f" \
    > "$ZDIR/$(basename "$f" _fastrtps_launch.py)_zenoh_launch.py"
done
ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUT/samples/rmw_zenoh_cpp/zenohd.log" 2>&1 &
ZPID=$!
sleep 3
for f in "$ZDIR"/test_*_zenoh_launch.py; do
  RMW_IMPLEMENTATION=rmw_zenoh_cpp run_one rmw_zenoh_cpp "$f"
done
kill "$ZPID"; wait "$ZPID" 2>/dev/null || true
cat "$SUMMARY"
