# v4l2_camera

**Needs hardware: a Linux machine (x86_64 or aarch64) with a V4L2 camera that
delivers 640x480 YUYV.** A plain USB webcam does. On macOS, on Linux without a
camera, or with a camera in another format, the node compiles but
`cerulion graph run` refuses the launch and says why (see "Without a camera").

A camera is a *driver* node: no topic upstream fires it, so it declares
`#[cerulion_node(external)]` and hands the runtime the device file descriptor
to watch. Every time the kernel has a frame, the runtime fires the node, and
`tick` dequeues that frame and publishes it.

```
v4l2_camera  --(sensor_msgs/Image, yuv422_yuy2)-->  brightness_meter  --(std_msgs/Float32)-->
```

- **`v4l2_camera`** opens `/dev/video0` (or `CERULION_V4L2_DEV`), checks the
  active format, maps the kernel's capture buffers, and returns
  `ExternalSource::Fd(fd)`. Each `tick` dequeues one frame and publishes a
  640x480 `sensor_msgs/Image`.
- **`brightness_meter`** is a data-triggered sink (`#[input(trigger)] image`)
  that reads the pixels in place and publishes the frame's mean luma (0 dark,
  255 bright) as a `std_msgs/Float32`. Cover the lens and the number drops.

Each node type is its own crate under `nodes/<type>/`, written in the macro
form. All wiring lives in `graphs/v4l2_camera.yaml`; the graph file carries
topology only. Each node's trigger policy is on its macro, in
`nodes/<type>/src/lib.rs`.

## Copies

The pixels are copied **once**: from the kernel's `mmap`'d capture buffer into
the shared-memory slot the runtime loaned for the frame (`loan_data` then
`copy_from_slice`). That copy sits at the driver boundary and stays unless the
buffer is imported as a DMABUF, which this example does not do. Everything
downstream is zero-copy: `brightness_meter` reads the published bytes in place
from shared memory and copies nothing.

## How this workspace was made

The workspace was scaffolded with these verbs. The node bodies, the relative
dependency paths in `Cargo.toml`, the `recordings/` line in `.gitignore`, the
`max_slice_len` on the image output (1 MiB, above the 614400-byte YUYV frame)
and the pinned `Cargo.lock` were then written by hand.

```bash
cerulion workspace create v4l2_camera
cd v4l2_camera
cerulion node create v4l2_camera --policy external -o sensor_msgs/Image image
cerulion node create brightness_meter -T sensor_msgs/Image image -o std_msgs/Float32 brightness
cerulion graph create v4l2_camera --prefix v4l2_camera
cerulion node stage v4l2_camera -g v4l2_camera
cerulion node stage brightness_meter -g v4l2_camera -I image v4l2_camera/image
```

This is a standalone workspace: it has its own `[workspace]` `Cargo.toml` and is
excluded from the repo's root workspace (see the root `Cargo.toml` `exclude`).
Inside this repository it depends on `cerulion_core` and `native_ros2_messages`
through the relative paths in `Cargo.toml`, so run it from inside this
directory. A workspace you create yourself gets the published crates.io
versions instead.

## Run it

```bash
cd examples/v4l2_camera

# Build both node libraries
cerulion node build v4l2_camera --release
cerulion node build brightness_meter --release

# Check the graph (exits nonzero on any failure)
cerulion graph validate v4l2_camera

# Run live with recording on (Ctrl+C to stop)
cerulion graph run v4l2_camera --release --record
```

The first node build also compiles the Cerulion runtime, so it takes a few
minutes; later builds take seconds. On the first run, Cerulion proposes one
process per node and asks `Apply this partition to the graph file? [y/N]`.
Press **Enter** to use that layout for this run only.

If the camera is not `/dev/video0`, name it before the run:

```bash
CERULION_V4L2_DEV=/dev/video2 cerulion graph run v4l2_camera --release --record
```

The camera must already deliver 640x480 YUYV. Check and set that once with
`v4l2-ctl` (package `v4l-utils`):

```bash
v4l2-ctl -d /dev/video0 --get-fmt-video
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=YUYV
```

The run terminal logs the camera starting and, once a second, the meter's
reading:

```
INFO v4l2_camera: v4l2 camera streaming device=/dev/video0 fd=... mapped=4
INFO brightness_meter: brightness measured mean_luma=118.3 frames=30
INFO brightness_meter: brightness measured mean_luma=117.9 frames=60
```

In a second terminal, watch the data itself:

```bash
cerulion topic list
cerulion topic hz /v4l2_camera/v4l2_camera/image
cerulion topic echo /v4l2_camera/brightness_meter/brightness
```

`topic hz` reports the camera's frame rate; `topic echo` prints the mean luma
per frame. Cover the lens and watch it fall.

## Without a camera

`external_source` cannot open a camera, so it reports `HostDriven` and the
runtime refuses the launch before a single tick, naming the node and the
reason. On macOS the same happens because V4L2 is a Linux API: the node still
compiles (`cargo check` passes on every platform, which is how CI covers this
workspace), it just has no capture path there. The log line above the refusal
tells you what to fix:

```
ERROR v4l2_camera cannot capture, so it reports HostDriven and the launch is refused. Needs Linux (x86_64 or aarch64) and a V4L2 camera delivering 640x480 YUYV; set CERULION_V4L2_DEV to the device path if it is not /dev/video0, then relaunch
```

That refusal is the whole story under `--single-process`. Under the default
multi-process layout (one process per node) the refusal happens inside the
camera's worker process: the supervisor logs the same error, reports
`worker process died`, and by its default `--peer-loss continue` policy keeps
the other node running with no data until you press **Ctrl+C**. To stop the
whole graph on that refusal, pass `--peer-loss fail`:

```bash
cerulion graph run v4l2_camera --release --peer-loss fail
```

## Verify the recording

Stop the run with **Ctrl+C**; Cerulion prints the recording's path. Re-execute
the current node code against it:

```bash
BAG="recordings/v4l2_camera_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

`replay PASS` and exit code 0 mean the rebuilt `brightness_meter` reproduced
the recorded readings from the recorded frames byte for byte. The camera node
is a driver: replay feeds its recorded frames downstream and never opens the
device.

## Verify the nodes

The format, payload and requeue rules of the camera node and the luma
arithmetic of the meter are tested without hardware, on every platform:

```bash
cargo test --locked -p v4l2_camera --lib
cargo test --locked -p brightness_meter --lib -- --test-threads=1
```

## Layout

```
v4l2_camera/
  Cargo.toml                       # [workspace] members = ["nodes/*"]
  .cargo/config.toml               # IOX2_LOG_LEVEL / RUST_LOG defaults
  graphs/v4l2_camera.yaml          # topology only: ids, types, inputs, outputs
  nodes/
    v4l2_camera/src/lib.rs         # the driver node, macro form
    v4l2_camera/src/frame.rs       # format, payload and requeue rules (pure)
    v4l2_camera/src/v4l2.rs        # the Linux ioctl/mmap face
    v4l2_camera/src/unsupported.rs # every other platform: refuses, loudly
    brightness_meter/src/lib.rs    # the data-triggered meter, macro form
```
