# Perception: turn a recording into a regression check

Record a native graph, re-execute its nodes against the recording, and compare
outputs using field-specific tolerances. This example uses generated images and
a small deterministic image operation, so the entire pass/fail loop runs without
camera hardware or an ML model.

The recording supplies a regression baseline. Matching that baseline checks for
behavior changes; it does not establish that the original behavior was correct.
Unlike ordinary message playback, this demo re-executes the current node code
and reports which output field exceeded its allowed tolerance.

## The graph

```
camera  --image_raw-->  detector  --detections-->  tracker
(30 Hz Image)           (DetectionArray)           (Vector3 track count)

imu  --reading-->
(100 Hz Imu)
```

- **camera**: valid 128x128 `mono8` frames with three moving 20x20 rectangles,
  generated directly in a shared-memory loan (`period_ms = 33`). Pure
  function of the tick counter (no `SystemTime`, no `rand`), so record and
  replay are byte-identical.
- **detector**: data-triggered; emits a `DetectionArray` (custom workspace
  schema) whose boxes bound three intensity bands in the borrowed pixels.
  It fills its output arrays directly in loaned storage. Scores are fixed
  tutorial values, not learned confidence estimates. `DETECTION_SHIFT` is
  0.0 in the golden build.
- **detector_perturbed**: the detector with `DETECTION_SHIFT = 8.0`, which
  shifts every box eight pixels and gives replay verification a known
  regression to detect. It is otherwise identical to `detector`, with the same
  ports and schema, so its library drops straight over the detector's artifact.
  The demo script verifies that before it builds.
- **tracker**: data-triggered on the detector output; publishes a track count.
  Included in the full `perception` graph; the smaller replay demo
  (`perception_min`) uses only camera and detector.
- **imu**: an independent periodic source (`period_ms = 10`) publishing a
  synthetic `sensor_msgs/Imu`. It is in the full `perception` graph to show two
  sources at different rates in one graph: the camera fires every 33 ms and
  the imu every 10 ms, each rate is the `period_ms` on that node's own macro,
  and the graph file says nothing about either.

The custom `DetectionArray` schema (`schemas/detections.yaml`) is three
**`float64[]`** primitive arrays: `boxes` (flattened `[x,y,w,h]` per
detection), `scores`, `class_ids`. Each node crate's `build.rs` reads that same
yaml to codegen the Rust type, so the wire `schema_hash` it stamps always
matches the hash the replayer resolves. (`float64[]` is deliberate: the replay
tolerance registry can only decode numeric scalars and `float64[]` arrays. A
`float32[]`/`uint32[]` array is publisher-opaque and a numeric metric on it is
refused at validation.)

## The tolerance spec (`tolerance.yaml`)

```yaml
topics:
  /percepmin/detector/detections:
    fields:
      boxes:     { kind: bbox_iou, min_iou: 0.5 }   # boxes must overlap >= 0.5 IoU
      scores:    { kind: max_abs, threshold: 0.05 } # confidences may drift 0.05
      class_ids: { kind: set_equal }                # predicted class set unchanged
```

Everything not named stays **byte-exact**.

## Run the demo

```bash
cd examples/perception
./run_replay_demo.sh
```

The script builds the nodes, records a run, verifies it, then repeats the
verification with the shifted detector:

1. Check the twin against the detector, then build the three node libraries the
   loop needs: `cerulion node build <type> --release` for `camera`, `detector`
   and `detector_perturbed`, then `cerulion graph validate perception_min`.
2. **Record** `perception_min` for ~3 s: `cerulion graph run perception_min --release --single-process --record`
   (single-process is the wall-faithful recording path; a split run records
   on its lockstep quantum instead).
3. **Replay #1 (golden):** `cerulion bag play <bag> --resim all --verify --tolerance tolerance.yaml`: **exit 0**, `replay PASS`.
4. **Swap** `libdetector_perturbed` over `libdetector` (the deliberate output regression).
5. **Replay #2 (perturbed):** same command: **exit 1**, a `bbox_iou` regression:

   ```
   FRAME-CONTENT DIVERGENCE: <bag>
     2822 tick(s) replayed; 1/2 topic(s) matched; 1 violation(s):
     - /percepmin/detector/detections [tolerance-exceeded]: field 'boxes' exceeded
       the bbox_iou tolerance on '/percepmin/detector/detections':
       worst value 0.42857142857142855 vs threshold 0.5 (worst at frame 0)
   ```

6. Restore the original library; check for leaked iceoryx2 SHM state.

A pure x-shift of 8 px on a 20x20 box gives IoU `240/560 = 0.4286`, below the
0.5 floor on every frame: a clean, deterministic regression.

The script drives the `cerulion` on your `PATH`. Set `CERULION=/path/to/cerulion`
to drive a different binary.

## Run it yourself, by hand

The same loop, one verb at a time:

```bash
cd examples/perception

cerulion node build camera --release
cerulion node build detector --release
cerulion graph validate perception_min

cerulion graph run perception_min --release --single-process --record   # Ctrl+C to stop
# The run prints `recording written to <path>` as its last line; use that path:
BAG="recordings/perception_min_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify --tolerance tolerance.yaml   # exit 0

# ...change DETECTION_SHIFT in nodes/detector/src/lib.rs, then:
cerulion node build detector --release
cerulion bag play "$BAG" --resim all --verify --tolerance tolerance.yaml   # exit 1 with the metric
```

The full four-node graph (camera -> detector -> tracker, plus the imu) needs the
other two libraries:

```bash
cerulion node build tracker --release
cerulion node build imu --release
cerulion graph validate perception
cerulion graph run perception --release --record
```

While it runs, `cerulion topic hz /percep/camera/image_raw` reads about 30 Hz
and `cerulion topic hz /percep/imu/reading` about 100 Hz.

`--verify` exit codes: `0` pass, `1` data/tolerance violation, `2` bag not
replay-grade, `3` node failure, `4` bad tolerance YAML, `5` internal error,
`6` trace divergence. `7` means the command needs you to sign in and is not a
replay verdict. Without `--verify` a `--resim all` re-executes and makes no
claim about matching the recording; it exits 0 when the re-execution itself
succeeds, and still reports setup and execution failures as errors.

## Layout

```
examples/perception/
  Cargo.toml            # own [workspace]; NOT a member of the repo-root workspace
  schemas/detections.yaml
  graphs/{perception,perception_min}.yaml
  tolerance.yaml
  run_replay_demo.sh
  nodes/{camera,detector,detector_perturbed,imu,tracker}/
```

This is a standalone cargo workspace (path deps back to the repo crates), so
`cargo build --workspace` at the repo root ignores it, the same mechanism
`examples/go2` uses.

## If a run crashes mid-demo

`run_replay_demo.sh` restores the swapped detector library on exit via a trap.
The exit trap restores the detector after ordinary exits. If SIGKILL interrupts
the swap, restore `target/release/libdetector.<ext>.orig` over
`target/release/libdetector.<ext>` before running the demo again (`<ext>` is
`dylib` on macOS or `so` on Linux). Rebuilding is not enough: a build that
finds the artifact up to date does not check its bytes.

## Test the message contract

The tests observe published images and detections, check handwritten pixel and
box expectations, reject malformed image metadata, and pin the 8-pixel IoU
regression. Run packages separately:

```bash
cargo test --locked -p camera --lib -- --test-threads=1
cargo test --locked -p detector --lib -- --test-threads=1
cargo test --locked -p detector_perturbed --lib -- --test-threads=1
```
