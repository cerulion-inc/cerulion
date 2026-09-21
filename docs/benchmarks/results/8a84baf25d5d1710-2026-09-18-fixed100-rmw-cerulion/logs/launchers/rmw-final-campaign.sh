#!/usr/bin/env bash
set -uo pipefail
LANE=<BOXHOME>/lanes/rmw-latency-lane-20260918
cd "$LANE"; SHA=$(git rev-parse HEAD); export DOCKER_BUILDKIT=1
IMG=$(docker images --no-trunc --format "{{.ID}}" latency_bench:jazzy)
MH=$(bash -c "source $LANE/tools/scripts/benchmarks/lib/machine_hash.sh && compute_live_machine_hash")
cd "$LANE/benches/latency"
RUN="$LANE/benches/latency/results/${MH}-2026-09-18-fixed100-g3-deferred"
mkdir -p "$RUN"
echo "=== run dir: $RUN"; echo "=== repo sha: $SHA"; echo "=== image: $IMG"; echo "=== mh: $MH"
echo "=== started: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
for k in 1 2 3 4 5; do
  echo "######## loan cell rep $k/5 (ten payloads) ########"
  CER_BENCH_DMA_LOCK=0 python3 bench.py ros2 --distro jazzy --chrt 0 \
      --variant fixed100 --run-dir "$RUN" --rep "$k" \
      --cells jazzy_cerulion_shm_loan_be1_chrt0 2>&1 | tail -2
  echo "---- rep $k rc=$? $(date -u +%H:%M:%SZ)"
done
echo "=== compile-csv STRICT ==="
CER_BENCH_DMA_LOCK=0 python3 bench.py compile-csv --variant fixed100 --run-dir "$RUN"
echo "---- compile rc=$?"
# Stock parity spot check in its OWN run dir: two sizes would make a
# STRICT compile of the campaign dir fail on a partial sweep.
SPOT="$LANE/benches/latency/results/${MH}-2026-09-18-fixed100-g3-deferred-stockspot"
mkdir -p "$SPOT"
echo "=== stock parity spot check (separate dir) ==="
CER_BENCH_DMA_LOCK=0 CER_BENCH_PAYLOAD_SIZES="4194304 16777216" \
  timeout 900 python3 bench.py ros2 --distro jazzy --chrt 0 --variant fixed100 \
    --run-dir "$SPOT" --cells jazzy_stock_rclcpp_chrt0 2>&1 | tail -3
echo "---- stock rc=$?"
CER_BENCH_DMA_LOCK=0 python3 bench.py compile-csv --variant fixed100 --run-dir "$SPOT" --allow-partial 2>&1 | tail -2
echo "=== FINAL CAMPAIGN FINISHED: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
