#!/usr/bin/env bash
# ===========================================================================
# The perception replay-tolerance demo.
#
#   record once  ->  re-execute + verify (exit 0, byte/tolerance clean)
#                ->  swap in the perturbed detector cdylib
#                ->  re-execute + verify (exit 1, bbox_iou violation on the
#                    detector topic)
#                ->  restore + clean up
#
# Re-execution is `cerulion bag play <bag> --resim all --verify`. `--verify` is
# what carries the byte-comparison and the stable 0-6 exit contract the two
# legs below assert on; a bare `--resim all` re-executes and always exits 0.
#
# Everything goes through the `cerulion` verbs: `node build` for the node
# libraries, `graph run --record` for the recording, `bag play` for the
# verdict. The script uses the `cerulion` on PATH; set CERULION=/path/to/cerulion
# to drive a different binary (CI points it at the one it just built).
#
# Generated images exercise a deterministic intensity-band detector. A changed
# x-offset is caught as an IoU regression on a named topic/field. This tutorial
# demonstrates the regression workflow without requiring hardware or an ML model.
#
# Every expectation is checked with a loud failure. Bounded, deterministic,
# CI-cheap (perception_min = 2 nodes, tiny frames, ~3 s record window).
# ===========================================================================
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

CLI="${CERULION:-cerulion}"
RECORD_SECS="${RECORD_SECS:-3}"
REC_DIR="$HERE/recordings"
# `cerulion node build` honors CARGO_TARGET_DIR exactly as cargo does.
TARGET_DIR="${CARGO_TARGET_DIR:-$HERE/target}/release"

# Platform cdylib extension.
case "$(uname -s)" in
  Darwin) EXT="dylib"; LIBPRE="lib" ;;
  Linux)  EXT="so";    LIBPRE="lib" ;;
  *) echo "FATAL: unsupported OS $(uname -s)"; exit 1 ;;
esac
DETECTOR_LIB="$TARGET_DIR/${LIBPRE}detector.${EXT}"
PERTURBED_LIB="$TARGET_DIR/${LIBPRE}detector_perturbed.${EXT}"

fail() { echo "DEMO FAILED: $*" >&2; exit 1; }
say()  { printf '\n=== %s ===\n' "$*"; }

# ---------------------------------------------------------------------------
# 0. Preflight + build. The twin must differ from the golden detector in its
#    one constant and nowhere else, or the regression below proves nothing.
# ---------------------------------------------------------------------------
command -v "$CLI" >/dev/null 2>&1 \
  || fail "no '$CLI' on PATH: install Cerulion (see the top-level README), or set CERULION=/path/to/cerulion"
cmp -s nodes/detector/src/algorithm.rs nodes/detector_perturbed/src/algorithm.rs \
  || fail "nodes/detector_perturbed/src/algorithm.rs drifted from the detector's copy"
cmp -s nodes/detector/src/tests.rs nodes/detector_perturbed/src/tests.rs \
  || fail "nodes/detector_perturbed/src/tests.rs drifted from the detector's copy"
TWIN_DIFF="$(diff nodes/detector/src/lib.rs nodes/detector_perturbed/src/lib.rs | grep -c '^[<>]' || true)"
[ "$TWIN_DIFF" -eq 2 ] || fail "the perturbed twin must differ from the detector in exactly one line (found $TWIN_DIFF changed sides)"

for node in camera detector detector_perturbed; do
  say "cerulion node build $node --release"
  "$CLI" node build "$node" --release
done
[ -f "$DETECTOR_LIB" ] || fail "detector library not built: $DETECTOR_LIB"
[ -f "$PERTURBED_LIB" ] || fail "perturbed library not built: $PERTURBED_LIB"

say "cerulion graph validate perception_min"
"$CLI" graph validate perception_min

# SHM leak baseline (macOS keeps iceoryx2 state under /tmp; Linux under
# /dev/shm, so scan both). Graceful exits self-clean, so this should not grow; a
# shared machine may have unrelated entries, hence a loud WARN (not a hard
# fail) if it does.
# `|| true`: unmatched globs make ls exit nonzero (e.g. /dev/shm on macOS),
# which would kill the script via `set -e` + pipefail inside the substitution.
shm_count() { { ls /tmp/cer_* /dev/shm/cer_* /dev/shm/iox2_* 2>/dev/null || true; } | wc -l | tr -d ' '; }
SHM_BEFORE="$(shm_count)"

rm -rf "$REC_DIR"

# ---------------------------------------------------------------------------
# 1. Record perception_min for a bounded window (single-process = the
#    wall-faithful, byte-exact-replayable recording path).
# ---------------------------------------------------------------------------
say "record perception_min for ${RECORD_SECS}s"
"$CLI" graph run perception_min --release --single-process --record &
REC_PID=$!
sleep "$RECORD_SECS"
kill -INT "$REC_PID" 2>/dev/null || true
# Wait for the recorder to finalize the bag (bagd flush + index write).
wait "$REC_PID" 2>/dev/null || true

BAG="$(ls -t "$REC_DIR"/*.mcap 2>/dev/null | head -1 || true)"
[ -n "$BAG" ] || fail "no bag produced in $REC_DIR"
BAG_BYTES="$(wc -c < "$BAG" | tr -d ' ')"
echo "recorded bag: $BAG (${BAG_BYTES} bytes)"

# ---------------------------------------------------------------------------
# 2. Golden replay: the current (unperturbed) detector. Expect exit 0.
# ---------------------------------------------------------------------------
say "replay #1 (golden: unperturbed detector): expect PASS / exit 0"
set +e
GOLDEN_OUT="$("$CLI" bag play "$BAG" --resim all --verify --tolerance tolerance.yaml 2>&1)"
GOLDEN_CODE=$?
set -e
echo "$GOLDEN_OUT"
[ "$GOLDEN_CODE" -eq 0 ] || fail "golden replay exit $GOLDEN_CODE (expected 0)"
echo "$GOLDEN_OUT" | grep -q "replay PASS" || fail "golden replay missing 'replay PASS'"

# ---------------------------------------------------------------------------
# 3. Swap the perturbed detector cdylib over the resolved detector artifact,
#    then replay again. Expect exit 1 AND a bbox_iou violation on boxes.
# ---------------------------------------------------------------------------
say "swap in the perturbed detector cdylib (hyperparameter DETECTION_SHIFT 0.0 -> 8.0)"
cp "$DETECTOR_LIB" "${DETECTOR_LIB}.orig"
trap 'cp "${DETECTOR_LIB}.orig" "$DETECTOR_LIB" 2>/dev/null || true; rm -f "${DETECTOR_LIB}.orig"' EXIT
cp "$PERTURBED_LIB" "$DETECTOR_LIB"

say "replay #2 (perturbed detector): expect REGRESSION / exit 1"
set +e
FAIL_OUT="$("$CLI" bag play "$BAG" --resim all --verify --tolerance tolerance.yaml 2>&1)"
FAIL_CODE=$?
set -e
echo "$FAIL_OUT"
[ "$FAIL_CODE" -eq 1 ] || fail "perturbed replay exit $FAIL_CODE (expected 1)"
echo "$FAIL_OUT" | grep -q "bbox_iou" \
  || fail "perturbed replay verdict did not name the bbox_iou metric"
echo "$FAIL_OUT" | grep -q "field 'boxes'" \
  || fail "perturbed replay verdict did not name the boxes field"
echo "$FAIL_OUT" | grep -q "/percepmin/detector/detections" \
  || fail "perturbed replay verdict did not name the detector topic"

# ---------------------------------------------------------------------------
# 4. Restore + SHM leak check.
# ---------------------------------------------------------------------------
say "restore detector cdylib"
cp "${DETECTOR_LIB}.orig" "$DETECTOR_LIB"
rm -f "${DETECTOR_LIB}.orig"
trap - EXIT

SHM_AFTER="$(shm_count)"
if [ "$SHM_AFTER" -gt "$SHM_BEFORE" ]; then
  echo "WARN: iceoryx2 /tmp state grew ${SHM_BEFORE} -> ${SHM_AFTER} (possible leak, or concurrent unrelated runs on a shared machine)" >&2
else
  echo "SHM clean: /tmp cer_* count ${SHM_BEFORE} -> ${SHM_AFTER} (no net leak)"
fi

say "DEMO PASSED"
echo "golden replay: exit 0 (PASS)  |  perturbed replay: exit 1 (bbox_iou regression on /percepmin/detector/detections.boxes)"
