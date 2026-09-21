# go2

A Unitree Go2 quadruped as a Cerulion workspace: the robot's own DDS traffic
bridged onto the zero-copy wire, the front camera transcoded on the robot,
the transform tree broadcast, and a teleop stack (gamepad or keyboard, a
safety mux, and a sport driver that commands the robot) that you can watch
and record from your own machine.

```
                 companion computer (on the robot LAN)                         your desk
 Go2 firmware   +-------------------------------------------------+   +---------------------------+
 (CycloneDDS)   |  dds_bridge  ---> /go2/utlidar/cloud  (PointCloud2)|   | cerulion viz --robot <name>|
 /utlidar/cloud |              ---> /go2/odom            (Odometry)  |==>| cerulion topic list / hz  |
 /sportmodestate|              ---> /go2/camera/h264     (raw H.264) |   | cerulion bag record       |
 /frontvideo... |  camera_jpeg ---> /go2/camera/jpeg     (JPEG)      |   +---------------------------+
                |  go2_tf_source -> /tf, /tf_static     (TFMessage) |
                |                                                   |   +---------------------------+
 /api/sport/    |  joystick_teleop -> /go2/cmd_vel/joystick          |   | keyboard_teleop (optional)|
   request  <---|  keyboard_teleop -> /go2/cmd_vel/keyboard  (Twist) |<==| -> /go2/cmd_vel/keyboard  |
                |  teleop_mux      -> /go2/cmd_vel                   |   +---------------------------+
                |  sport_driver    -> unitree_api/Request over DDS   |
                +---------------------------------------------------+
```

The companion computer is the Go2 EDU's on-board Jetson, or any Linux
machine plugged into the robot's LAN. Nothing renders on the robot: it ships
raw frames, and `cerulion-vizd` on your desk decodes and draws them.

This is a standalone workspace: it has its own `[workspace]` `Cargo.toml`,
`Cargo.lock` and `deny.toml`, and it is excluded from the repository's root
workspace. It depends on `cerulion_core` / `native_ros2_messages` through the
relative paths in `Cargo.toml`, so run every command from inside this
directory. (Outside the repository, swap them for the published crates.io
versions.)

## What ships

| Piece | Kind | What it does |
|---|---|---|
| `nodes/dds_bridge` | `external` node | the generic, config-mapped DDS to Cerulion ingress bridge; the Go2 is its first config (`graphs/go2.bridge.yaml`) and it is the same node `cerulion ros2 attach` generates a graph for |
| `nodes/camera_jpeg` | data-triggered node | decodes one rendition of the bridged H.264 stream (NVDEC on a Jetson, software elsewhere) and publishes JPEG |
| `nodes/go2_tf_source` | periodic node | broadcasts the static sensor mounts on `/tf_static` and `odom` to `base` on `/tf` (an identity placeholder until the odometry projection is verified on a robot); "Where the transforms come from" below says what feeds it |
| `nodes/joystick_teleop` | `external` node | a Bluetooth gamepad to `geometry_msgs/Twist`, with a held-button deadman and a link watchdog |
| `nodes/keyboard_teleop` | `external` node | a terminal (WASD) to `geometry_msgs/Twist`, latched with an auto-zero |
| `nodes/teleop_mux` | periodic node | arbitrates the two command sources by freshness; a stale or silent source never keeps the robot moving |
| `nodes/sport_driver` | data-triggered node | turns the arbitrated command into `unitree_api/Request` on the robot's `/api/sport/request` (Move while moving, StopMove on every stop) |
| `lib/cerulion_go2_dds` | support library | one DDS participant per process, QoS helpers, and the Unitree message structs and CDR codecs the bridge and the driver share; see the hardware validation notes below for which of them are validated against a robot |
| `lib/unitree_go` | support library | the `unitree_go/Go2FrontVideoData` type, generated at build time from `schemas/unitree_go/msg/` through the same pipeline the bridge decodes with |
| `graphs/bridge.yaml` + `graphs/go2.bridge.yaml` | graph + bridge config | the sensor graph: bridge + camera, and the three DDS mappings it carries |
| `graphs/tf.yaml` | graph | the transform broadcaster on its own |
| `graphs/teleop.yaml` | graph | teleop with everything on the companion computer (keyboard over an `ssh -t` session) |
| `graphs/teleop_desk.yaml` + `graphs/teleop_remote.yaml` | graph pair | the keyboard on your desk, imported by the robot side over the network |
| `schemas/unitree_go/msg/Go2FrontVideoData.msg` | schema | what the firmware ACTUALLY publishes on `/frontvideostream` (the vendor's four-field definition does not match its own firmware; the file says how the layout was established) |
| `schemas/mux_state.yaml` | schema | a proposed arbitration-state output for the mux; the node publishes only the command, so nothing carries this schema on the wire |

## Where the transforms come from

A stock Go2 has no third-party autonomy stack publishing transforms. The
transform tree here is self-contained from the stock firmware, and no
autonomy stack is required for it: `go2_tf_source` broadcasts the static
sensor mounts on `/tf_static` and `odom` to `base` on `/tf`, and that
`odom` to `base` is an identity placeholder. The data a real one needs is
already bridged: the firmware's `/sportmodestate` carries the body position,
velocity and the IMU quaternion, and the bridge projects it onto `/go2/odom`
as `nav_msgs/Odometry`. The TF source does not consume it: the
`SportModeState` byte layout is not validated against samples from a robot
(the `odom` to `base` item under "Validate on your robot"), so compare
`/go2/odom` with your robot's motion before you drive a transform from it.

A SLAM stack such as the community `autonomy_stack_go2` is an optional
upgrade, not a prerequisite: it adds map-frame localization (it publishes
SLAM localization and a `map` frame). When one is running on the robot LAN,
`cerulion ros2 attach` discovers and bridges its topics like any other
("Bring it up", step 1).

## Defaults and overrides

Everything that varies between one Go2 and another is a documented default
with one override. The table lists the hardware and network defaults with
their overrides.

| Value | Default | Where it is read | Override |
|---|---|---|---|
| DDS domain id | `0` (the Go2 factory `ROS_DOMAIN_ID`) | `domain_id:` in `graphs/go2.bridge.yaml` (bridge); `GO2_DOMAIN_ID` (sport driver); `--domain` on `cerulion ros2 attach` | edit the config, export the variable, pass the flag |
| The companion's robot-LAN interface IP | none, with a loud warning: on a machine with more than one interface the robot's CycloneDDS drops fragmented discovery data and never finds you | `only_networks:` in `graphs/go2.bridge.yaml`, else the `GO2_IFACE` environment variable (both the bridge and the sport driver); `--iface` on `cerulion ros2 attach` | set `GO2_IFACE` to that address before every run, or write it into the config |
| Robot LAN addresses | Unitree's conventions: the LAN is `192.168.123.0/24`, the main control board is `.161`, the EDU's on-board Jetson is `.18`, and the vendor SDK asks an external PC to take `.99` | nothing in this workspace reads a robot address; discovery is multicast on the interface above | none needed; `GO2_IFACE` is the only address code consumes |
| DDS message identity format | the `humble` feature of `lib/cerulion_go2_dds` (the pre-Iron 24-byte GID the vendor images use, Foxy and Humble alike) | the crate's `[features]` | `--no-default-features --features jazzy` for an Iron-or-newer peer |
| Bridged DDS topics and types | `/utlidar/cloud` (`sensor_msgs/PointCloud2`), `/sportmodestate` (`unitree_go/SportModeState`), `/frontvideostream` (`unitree_go/Go2FrontVideoData`) | `mappings:` in `graphs/go2.bridge.yaml` | see what YOUR unit publishes with `cerulion ros2 attach --iface <ip> --dry-run`, then edit the mappings or let attach generate a full one (below) |
| Bridge config file | none: the bridge refuses to launch without one | `DDS_BRIDGE_CONFIG` | `export DDS_BRIDGE_CONFIG=graphs/go2.bridge.yaml` |
| Camera rendition | the 720-line rendition (the topic interleaves two, 360 and 720 lines high) | `CAMERA_TARGET_HEIGHT` | `export CAMERA_TARGET_HEIGHT=360` |
| The robot's announced name | the companion computer's hostname | `CERULION_ROBOT_IDENTITY`, read by the process that announces | `export CERULION_ROBOT_IDENTITY=go2` in the shell that runs a serving graph, as step 2 below does; without it the robot announces under the companion's hostname, and every `--robot` below has to use that name instead |
| Where the desk reaches the robot | nothing bound: `cerulion-netd` listens only where `CERULION_NETD_LISTEN` says, and advertises itself over mDNS only then | `CERULION_NETD_LISTEN` in the companion's service environment | `export CERULION_NETD_LISTEN=tcp/0.0.0.0:7683` before every serve |
| Teleop saturation limits | 0.6 m/s forward, 0.4 m/s sideways, 1 rad/s yaw (well under the sport API's maxima; first actuation happens on a stand) | constants in `nodes/joystick_teleop`, duplicated and pinned in lockstep in the keyboard and driver crates | edit the constants and rebuild |
| Companion file-descriptor limit | the image default is too low for a whole-robot bridge | the shell that runs a graph | `ulimit -n 65536` in that shell |

## Build

```bash
cd examples/go2

# Every node library (the first build also compiles the Cerulion runtime):
cerulion node build dds_bridge --release
cerulion node build go2_tf_source --release
cerulion node build joystick_teleop --release
cerulion node build keyboard_teleop --release
cerulion node build teleop_mux --release
cerulion node build sport_driver --release

# The camera needs GStreamer, a SYSTEM library: `node build` probes for it and
# enables capture when it is present, or builds a decoder-less library and
# prints the install command. A decoder-less camera refuses the graph
# launch by name instead of running blind.
cerulion node build camera_jpeg --release
```

On Linux the joystick node needs the `libudev` development package
(`sudo apt install libudev-dev`) because its gamepad backend links it. The
camera's GStreamer prerequisites are in `nodes/camera_jpeg/BRINGUP.md`.

Check every graph before touching a robot:

```bash
cerulion graph validate bridge
cerulion graph validate tf
cerulion graph validate teleop
cerulion graph validate teleop_desk
cerulion graph validate teleop_remote
```

## Bring it up

### 1. See what this unit publishes

Every Go2 differs a little: firmware versions, which vendor stack is
running, which topics exist. Ask the robot before trusting any mapping.
Replace `REPLACE_WITH_COMPANION_INTERFACE_IP` with the companion's own IP
address on the robot LAN, here and in every later block that sets it:

```bash
# On the companion computer:
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
cerulion ros2 attach --iface "$GO2_IFACE" --dry-run
```

The report lists every DDS topic the robot publishes, which types resolve
(the workspace's `.msg` store and the built-in ROS 2 corpus), and what a
restart under `rmw_cerulion` would buy. Nothing is written.

Two ways forward from here, and both run the same `dds_bridge` node:

- **The curated sensor graph** (`graphs/bridge.yaml` + `graphs/go2.bridge.yaml`,
  step 2): three mappings, explained line by line, re-validated in CI
  against captured robot bytes, and the graph that also hosts the camera
  transcode. Confirm its three topic names against the dry-run report.
- **Everything at once**: `cerulion ros2 attach --iface "$GO2_IFACE" --yes`
  writes `graphs/attach.yaml` and `graphs/attach.bridge.yaml` for every
  discovered topic (typed-port conflicts routed raw, missing schemas
  acquired) and runs it. Attach owns those two files and regenerates them
  on every run, so keep your own nodes in a separate graph that consumes
  its topics by absolute name. Raise the file-descriptor limit first: a
  whole robot is many topics.

### 2. Sensors: the bridge and the camera

```bash
# On the companion computer:
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
export CERULION_ROBOT_IDENTITY=go2
export CERULION_NETD_LISTEN=tcp/0.0.0.0:7683
export DDS_BRIDGE_CONFIG=graphs/go2.bridge.yaml
ulimit -n 65536
cerulion graph run bridge --release --single-process
```

`CERULION_ROBOT_IDENTITY` is what the robot announces itself as. This README
uses `go2`, so every `--robot go2` below matches; export it before the first
serving graph starts, because that run is what brings up the announcing
`cerulion-netd`. Skip it and the robot announces under the companion's
hostname, which you can read off `cerulion topic list` and pass instead.

`--single-process` is a recommendation: networked multi-process runs are
first-class, but this walkthrough uses one process, and the flag also skips
the partition prompt. The transform tree is a second run,
`cerulion graph run tf --release --single-process`, in another shell (both
runs converge on the machine's one `cerulion-netd`, so the identity exported
above already applies to it).

Check the rates on the companion:

```bash
cerulion topic hz /go2/camera/h264   # the raw route; both renditions ride it
cerulion topic hz /go2/camera/jpeg   # the decoded rendition
cerulion topic hz /go2/utlidar/cloud
```

A silent `/go2/camera/h264` is the bridge, not the camera: check
`DDS_BRIDGE_CONFIG`, the `/frontvideostream` mapping and `GO2_IFACE`.

### 3. Watch from your desk

```bash
# On your machine (remote discovery is automatic on one LAN):
cerulion topic list
cerulion viz --robot go2 /go2/utlidar/cloud /go2/camera/jpeg /tf /tf_static
```

`--robot` takes the robot's announced name, which is the
`CERULION_ROBOT_IDENTITY` exported in step 2, and needs at least one topic.
`cerulion topic list` prints the announced name under ROBOTS if you are
unsure. Off the LAN, name the listener:
`cerulion topic list --connect tcp/<companion address>:7683`, and the same
`--connect` on `cerulion viz`. The desk can decode the raw H.264 route
directly (`/go2/camera/h264`); the JPEG leg is the fallback for a desk that
cannot.

### 4. Teleop

Read `ROBOT-DAY.md` section 4 first: the robot goes on a stand, is stood up
with the vendor remote, and the deadman check is done before anything else.

```bash
# On the companion computer, in a real terminal (ssh -t gives you one):
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
export CERULION_ROBOT_IDENTITY=go2
cerulion graph run teleop --release --single-process
```

Hold the gamepad's right shoulder button to arm the stick, or use WASD in
that terminal. The mux prefers a fresh gamepad; a source that stops
publishing is dropped within its freshness window, and with both sources
stale the driver sends StopMove. The sport driver never issues a posture
command (StandUp, StandDown): standing up stays a deliberate step you take
with the remote.

The desk keyboard variant is the graph pair `teleop_desk` (your machine)
and `teleop_remote` (the companion); their headers say how the two find
each other.

## Record and replay

```bash
# On the companion computer, sensors only:
cerulion graph run tf --release --single-process --record
# Ctrl-C to finalize the bag, then re-execute the current node code against it:
cerulion bag play "recordings/tf_<timestamp>.mcap" --resim all --verify
```

Recording keeps the robot network-visible, so the desk keeps seeing the
topics while the bag is written. A bag holding `/go2/camera/jpeg` may report a
byte difference on that topic under `--verify`: the JPEG bytes come from a
GStreamer encoder that is not in the bag, and its output is not guaranteed to
reproduce across encoders or versions. Compare decoded content when you are
evaluating image changes, and use byte-exact replay for the deterministic node
outputs, which the crate's own tests cover
(`nodes/camera_jpeg/BRINGUP.md`). A graph
that declares `ingress:` (`teleop_remote`) cannot be recorded yet.

## Verify without a robot

The same set the `demos-go2` CI job runs on every pull request, on a machine
with no GStreamer and no robot (the tests that need a live DDS peer are
`#[ignore]`d and say what they need):

```bash
for pkg in cerulion_go2_dds unitree_go dds_bridge go2_tf_source \
           joystick_teleop keyboard_teleop sport_driver teleop_mux; do
  cargo test -p "$pkg" -- --test-threads=1
done
cargo test -p camera_jpeg --lib -- --test-threads=1
cargo test -p camera_jpeg --test node_publish_test -- --test-threads=1
```

Run one package at a time rather than `cargo test --workspace`: the
shared-memory test binaries are separate processes, which `--test-threads=1`
does not serialize.

The sport driver's policy and its trigger-to-request path are covered
there with a captured sink in place of DDS; the camera loopback rig needs
GStreamer (`nodes/camera_jpeg/BRINGUP.md`).

## Layout

```
go2/
|-- Cargo.toml                 # [workspace] members = ["nodes/*", "lib/*"]
|-- .cargo/config.toml         # IOX2_LOG_LEVEL / RUST_LOG defaults
|-- deny.toml                  # the workspace's own cargo-deny policy
|-- graphs/
|   |-- bridge.yaml            # bridge + camera (DDS_BRIDGE_CONFIG names the mapping file)
|   |-- go2.bridge.yaml        # the bridge's DDS mappings (domain, interface, topics)
|   |-- tf.yaml                # the transform broadcaster
|   |-- teleop.yaml            # gamepad + keyboard + mux + driver on the companion
|   |-- teleop_desk.yaml       # the keyboard on your desk
|   `-- teleop_remote.yaml     # the robot side of the desk-keyboard pair
|-- lib/
|   |-- cerulion_go2_dds/      # DDS participant, QoS, Unitree structs, CDR codecs
|   `-- unitree_go/            # generated Go2FrontVideoData type
|-- nodes/<type>/src/lib.rs    # one node type per crate, macro form
|-- schemas/
|   |-- unitree_go/msg/Go2FrontVideoData.msg
|   `-- mux_state.yaml
|-- ROBOT-DAY.md               # the robot-day runbook (prerequisites, network, safety ladder)
`-- README.md
```

Per-node bring-up notes: `nodes/camera_jpeg/BRINGUP.md`,
`nodes/joystick_teleop/BRINGUP.md`, `nodes/keyboard_teleop/DEMO.md`.
Platform references: `../../docs/networking.md` (how a robot is found and
served), `../../docs/schema_resolution.md` (how attach resolves types),
`../../docs/ros2_compatibility.md` (what runs where).

## Validate on your robot

These parts are experimental: they are covered by tests without hardware and
are not validated on a robot. Check each one on yours, with the robot on a
stand (`ROBOT-DAY.md` section 4).

- The sport driver: its api ids and JSON shape come from the vendor SDK and
  are not validated against robot firmware.
- The `odom` to `base` transform: `/go2/odom` is produced by the bridge, but
  the `SportModeState` layout is not validated against real bytes, so the
  TF source keeps its identity placeholder rather than fabricate motion
  ("Where the transforms come from").
- The static sensor mounts and the camera frame name are estimates: measure
  them on your unit.
- The camera's NVDEC pipeline link, latency legs and link byte rate are not
  measured on a robot.
- The desk-keyboard graph pair validates; it is not proven over a real link.

## A note on the two ROS 2 paths

`cerulion ros2 attach` and this workspace's bridge are one runtime: attach
discovers and generates, the bridge node carries the traffic. `rmw_cerulion`
is a different tool. It runs ROS 2 nodes you write on Cerulion's transport;
it does not speak DDS, so it can never talk to the Go2 firmware, and on a
Go2 the firmware side is always bridged. Where `rmw_cerulion` fits is a ROS 2
node of yours consuming the bridged topics; the attach report's MIGRATION
section says which discovered processes that would apply to.
