# camera_pipeline

A two-node Cerulion graph: a synthetic camera and a data-triggered detector.

```
sensor  --(sensor_msgs/Image)-->  detector  --(geometry_msgs/PoseStamped)-->
```

- **`sensor`**: periodic publisher (`period_ms = 33`) generating a valid 640x480 RGB image directly in its shared-memory loan. The synthetic shade cycles from dark to bright.
- **`detector`**: data-triggered consumer (`#[input(trigger)]`) that reads borrowed RGB pixels and publishes the centroid of pixels with mean intensity of 128 or more as a `PoseStamped`. A frame with no bright pixel publishes nothing: the tick leaves its output untouched.

The detector projects that centroid onto a **synthetic plane at z = 1 m**,
using focal lengths equal to image width/height and the image center as the
principal point. Its pose has identity orientation. This gives the tutorial
an observable output; a physical camera needs calibration and depth information.
The output retains the input header. Image bytes are never copied between the
nodes; the small header is copied into the new output message, and the pose is
written leaf by leaf (`self.detection.pose.position.x = x`) straight into the
loaned slot.

This example is a standalone workspace: run its commands from inside this
directory. See [how these workspaces work](../README.md#these-are-standalone-workspaces).

## Run it

```bash
cd examples/camera_pipeline

# Build both node libraries
cerulion node build sensor --release
cerulion node build detector --release

# Check the graph (exits nonzero on any failure)
cerulion graph validate camera_pipeline

# Run live with recording on (Ctrl+C to stop)
cerulion graph run camera_pipeline --release --record
```

The first node build in a workspace also compiles the Cerulion runtime, so it
takes a few minutes; later builds take seconds. On the first run Cerulion
proposes one process per node and asks to save that partition; see
[the first build and the partition prompt](../README.md#the-first-build-and-the-partition-prompt).

In a second terminal, watch the data flow:

```bash
cerulion topic list
cerulion topic hz /camera_pipeline/sensor/image
cerulion topic echo /camera_pipeline/detector/detection
```

## Verify the recording

Stop the run with **Ctrl+C**; Cerulion prints the recording's path. Re-execute
the current node code against it:

```bash
BAG="recordings/camera_pipeline_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

`replay PASS` and exit code 0 mean the rebuilt nodes reproduced the recorded
images and detections.

## Layout

```
camera_pipeline/
  Cargo.toml                    # [workspace] members = ["nodes/*"]
  .cargo/config.toml            # IOX2_LOG_LEVEL / RUST_LOG defaults
  graphs/camera_pipeline.yaml   # graph topology; trigger policies are declared on nodes
  nodes/
    sensor/src/lib.rs           # one node type, macro form
    detector/src/lib.rs         # one node type, macro form
```

## Verify the nodes

Each test uses an isolated transport and checks received message contents:

```bash
cargo test --locked -p sensor --lib -- --test-threads=1
cargo test --locked -p detector --lib -- --test-threads=1
```
