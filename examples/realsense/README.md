# realsense

**Needs hardware: an Intel RealSense depth camera (D400 series) on USB 3, and
the official Intel SDK, librealsense2, installed on the machine.** Without the
SDK the node compiles but has no driver; without the camera the SDK finds no
device. In both cases `cerulion graph run` refuses the launch and says why
(see "Without the SDK" and "Without a camera").

A camera is a *driver* node: no topic upstream fires it, so it declares
`#[cerulion_node(external)]`. librealsense2 offers no file descriptor to
watch, only a blocking wait for the next frameset, so the node hands the
runtime an `ExternalSource::Blocking` closure. The runtime drives that closure
on a helper thread; each call waits (bounded) for one synchronized color +
depth frameset, hands it to the node through a latest-wins slot, and rings the
node's doorbell. `tick` then copies the two planes into the loaned
shared-memory slots and publishes them.

```
realsense_camera  --(sensor_msgs/Image, rgb8)-->
realsense_camera  --(sensor_msgs/Image, 16UC1)-->  nearest_obstacle  --(sensor_msgs/Range)-->
```

- **`realsense_camera`** starts a pipeline for 640x480 color (RGB8) and depth
  (Z16) at 30 fps and publishes both as `sensor_msgs/Image`: `color` with the
  encoding `rgb8`, `depth` with `16UC1`, one little-endian `u16` per pixel in
  **millimetres**. At start it reads the camera's depth unit and refuses any
  other than the 0.001 m default, so the label is never wrong.
- **`nearest_obstacle`** is a data-triggered sink (`#[input(trigger)] depth`)
  that reads the depth counts in place, takes the smallest nonzero one in the
  central cone (the middle tenth of the image in each axis, 64x48 pixels) and
  publishes it as a `sensor_msgs/Range`, the message a single-beam distance
  sensor would send. Hold a hand in front of the camera and the range drops.
  A zero count is the SDK's marker for **no valid depth**, never a reading of
  "far away", so zeros are skipped rather than measured. A cone in which every
  count is zero therefore carries no distance at all: the node publishes
  `range = inf`, which a consumer must read as "the distance here is unknown"
  and handle as unavailable. It is not evidence that the path is clear.

Each node type is its own crate under `nodes/<type>/`, written in the macro
form. All wiring lives in `graphs/realsense.yaml`; the graph file carries
topology only. Each node's trigger policy is on its macro, in
`nodes/<type>/src/lib.rs`.

## Copies

Each frameset costs **one copy per plane**: the color pixels and the depth
counts are copied from the SDK's frame buffers into the shared-memory slots
the runtime loaned for the two images (`loan_data` then `copy_from_slice`).
librealsense2 owns its buffers and hands out pointers into them, so that copy
sits at the driver boundary and stays. Everything downstream is zero-copy:
`nearest_obstacle` reads the published depth counts in place from shared
memory and copies nothing but the small header it forwards.

The `Range` it publishes has a nominal field of view (the cone's share of an
87 degree D400 depth field of view) and nominal D435 range limits (0.28 m to
10 m). The camera node publishes no intrinsics, so these figures are
constants in `nodes/nearest_obstacle/src/lib.rs`, not measurements.

## Install the SDK

The node links librealsense2 through the `realsense-rust` bindings, behind a
cargo feature named `realsense` that is off by default. `cerulion node build
realsense_camera` probes for the SDK with `pkg-config realsense2` and turns
the feature on when it finds it; the manifest block
`[package.metadata.cerulion.optional-system-deps.realsense]` in
`nodes/realsense_camera/Cargo.toml` is what declares that. So install the SDK
first, then build.

macOS (Homebrew):

```bash
brew install librealsense
```

Ubuntu 22.04 or 24.04, from the official RealSense apt repository (the package
is not in Ubuntu's own archive):

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dev librealsense2-utils pkg-config
```

`librealsense2-utils` brings `rs-enumerate-devices` and `realsense-viewer`;
run the first one to confirm the camera is seen before building anything.

With ROS 2 Jazzy installed you can use its packaging of the same SDK instead,
`sudo apt-get install ros-jazzy-librealsense2`, but its `realsense2.pc` lives
under the ROS prefix and `source /opt/ros/jazzy/setup.bash` does not put it on
the pkg-config path. Export it before the build:

```bash
# `dpkg-architecture` comes from the `dpkg-dev` package.
export PKG_CONFIG_PATH="/opt/ros/jazzy/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/pkgconfig:$PKG_CONFIG_PATH"
```

The build prints what it found. With the SDK:

```
realsense_camera: system dependency `realsense` FOUND (realsense2 2.58.4).
  Building WITH `--features realsense` ...
```

Without it, the build still succeeds, says the feature is off, and prints the
install line for this platform (see "Without the SDK").

## Scaffolding it with the CLI

The workspace was scaffolded with these verbs. `node create` takes one `-o`, so
the second output is added with `node modify`:

```bash
cerulion workspace create realsense
cd realsense
cerulion node create realsense_camera --policy external -o sensor_msgs/Image color
cerulion node modify realsense_camera -o sensor_msgs/Image depth
cerulion node create nearest_obstacle -T sensor_msgs/Image depth -o sensor_msgs/Range obstacle
cerulion graph create realsense --prefix realsense
cerulion node stage realsense_camera -g realsense
cerulion node stage nearest_obstacle -g realsense -I depth realsense_camera/depth
```

The graph then sets `max_slice_len: 1 MiB` on both image outputs, above the
921600-byte RGB frame: a variable-length output's shared-memory budget has to
cover the largest frame it will publish.

This example is a standalone workspace: run its commands from inside this
directory. See [how these workspaces work](../README.md#these-are-standalone-workspaces).

## Run it

```bash
cd examples/realsense

# Build both node libraries (the first one probes for the SDK)
cerulion node build realsense_camera --release
cerulion node build nearest_obstacle --release

# Check the graph (exits nonzero on any failure)
cerulion graph validate realsense

# Run live with recording on (Ctrl+C to stop)
cerulion graph run realsense --release --record
```

The first node build in a workspace also compiles the Cerulion runtime, so it
takes a few minutes; later builds take seconds. On the first run Cerulion
proposes one process per node and asks to save that partition; see
[the first build and the partition prompt](../README.md#the-first-build-and-the-partition-prompt).

The run terminal logs the camera starting and, once a second, both nodes:

```
INFO realsense_camera: realsense camera streaming device="Intel RealSense D435 (serial ...)" width=640 height=480 fps=30
INFO realsense_camera: framesets published frames=30 overwritten=0
INFO nearest_obstacle: nearest obstacle in the central cone range_m=1.42 frames=30
```

`overwritten` counts framesets the helper thread replaced before `tick` took
them. A growing value means capture is outpacing the node.

In a second terminal, watch the data itself:

```bash
cerulion topic list
cerulion topic hz /realsense/realsense_camera/color
cerulion topic hz /realsense/realsense_camera/depth
cerulion topic echo /realsense/nearest_obstacle/obstacle
```

`topic hz` reports the camera's frame rate on each stream; `topic echo` prints
the `Range` per depth frame. Put a hand in front of the camera and watch
`range` fall.

## Without the SDK

`cerulion node build realsense_camera` builds the node without the feature
and says so:

```
realsense_camera: system dependency `realsense` NOT FOUND (missing pkg-config module(s): realsense2).
  Building WITHOUT `--features realsense`. The build will SUCCEED, but live RealSense capture (color rgb8 + depth 16UC1 through librealsense2) will be unavailable:
  this node has no camera driver: external_source() reports HostDriven and `cerulion graph run` refuses the graph at launch, naming this node
  To enable it, install the library and re-run this command:
      brew install librealsense
```

(On Linux the last line is the apt repository line from "Install the SDK".)
A `graph run` of that build is refused before a single tick; the log line
above the refusal names the fix:

```
ERROR realsense_camera cannot capture, so it reports HostDriven and the launch is refused. Connect a RealSense camera over USB 3 and relaunch reason="realsense_camera was built without the `realsense` feature, so it has no camera driver. Install librealsense2 (macOS: `brew install librealsense`; Ubuntu: Intel's apt repository or the `ros-<distro>-librealsense2` package, see README.md), then rebuild with `cerulion node build realsense_camera --release`, which enables the feature when pkg-config finds `realsense2`"
```

## Without a camera

With the SDK installed but no camera connected, the pipeline cannot start.
The node reports the SDK's own reason, plus how many devices the SDK
enumerated, and the launch is refused the same way:

```
ERROR realsense_camera cannot capture, so it reports HostDriven and the launch is refused. Connect a RealSense camera over USB 3 and relaunch reason="Config cannot be resolved by any active devices / stream combinations. (librealsense2 enumerated 0 RealSense device(s))"
```

That refusal is the whole story under `--single-process`. Under the default
multi-process layout (one process per node) the refusal happens inside the
camera's worker process: the supervisor logs the same error, reports
`worker process died`, and by its default `--peer-loss continue` policy keeps
the other node running with no data until you press **Ctrl+C**. To stop the
whole graph on that refusal, pass `--peer-loss fail`:

```bash
cerulion graph run realsense --release --peer-loss fail
```

## Verify the recording

Stop the run with **Ctrl+C**; Cerulion prints the recording's path. Re-execute
the current node code against it:

```bash
BAG="recordings/realsense_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

`replay PASS` and exit code 0 mean the rebuilt `nearest_obstacle` reproduced
the recorded ranges from the recorded depth frames byte for byte. The camera
node is a driver: replay feeds its recorded frames downstream and never opens
the SDK.

## Verify the nodes

The plane checks and the hand-over slot of the camera node, and the cone
arithmetic of the obstacle node, are tested without hardware and without the
SDK, on every platform:

```bash
cargo test --locked -p realsense_camera --lib
cargo test --locked -p nearest_obstacle --lib -- --test-threads=1
```

## Layout

```
realsense/
  Cargo.toml                        # [workspace] members = ["nodes/*"]
  .cargo/config.toml                # IOX2_LOG_LEVEL / RUST_LOG defaults
  graphs/realsense.yaml             # topology only: ids, types, inputs, outputs
  nodes/
    realsense_camera/Cargo.toml     # the `realsense` feature + its system-dep block
    realsense_camera/src/lib.rs     # the driver node, macro form
    realsense_camera/src/frame.rs   # stream shape, plane checks, hand-over slot (pure)
    realsense_camera/src/sdk.rs     # the librealsense2 face (feature on)
    realsense_camera/src/no_sdk.rs  # the driver-less face (feature off): refuses, loudly
    nearest_obstacle/src/lib.rs     # the data-triggered range node, macro form
```
