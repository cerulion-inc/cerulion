<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Go2 robot-day runbook

The checklist for taking a Unitree Go2 from a cold start to the full demo:
sensors on your desk, then teleop on a stand. It is written for ANY Go2 with
a companion computer on the robot's LAN (the EDU's on-board Jetson, or a
Linux machine you plug in); every value that varies between units is a
documented default with an override in `README.md` "Defaults and overrides",
and this file never repeats them. A step that can only be confirmed with
the robot powered and on the bench is marked **Check on your robot**.

Sourcing map (this runbook links rather than duplicates):

| Topic | Authoritative doc |
|---|---|
| The workspace, its defaults, the build and the run commands | `README.md` |
| Camera H.264 ingress and transcode | `nodes/camera_jpeg/BRINGUP.md` |
| Gamepad pairing, permissions and mapping | `nodes/joystick_teleop/BRINGUP.md` |
| Keyboard teleop and its Ctrl-C contract | `nodes/keyboard_teleop/DEMO.md` |
| Safety mux arbitration and staleness | `nodes/teleop_mux/src/lib.rs` and `arbitrate.rs` |
| Sport driver policy (Move, StopMove, keepalive, clamps) | `nodes/sport_driver/src/request.rs` |
| Where the transforms come from (stock firmware; where a SLAM stack fits) | `README.md` "Where the transforms come from" |
| How a robot is found and served on the network | `../../docs/networking.md` |
| How `cerulion ros2 attach` resolves message types | `../../docs/schema_resolution.md` |

## 0. Prerequisites

| # | Item | Where / how | State |
|---|---|---|---|
| 0.1 | Go2, charged, powered off | | check on arrival |
| 0.2 | Companion computer on the robot LAN | wired to the robot's switch; the vendor's LAN is `192.168.123.0/24`, the main board `.161`, the EDU's Jetson `.18`; note the companion's own interface IP, it is `GO2_IFACE` for every command below | **Check on your robot:** confirm with `ip addr` on the companion |
| 0.3 | Desk to companion path | one LAN, or a route to the companion; `cerulion-netd` on the companion listens on the well-known port once `CERULION_NETD_LISTEN=tcp/0.0.0.0:7683` is exported, and advertises itself over mDNS only then | confirm reachable (1.3) |
| 0.4 | Companion toolchain | Rust at the repository's floor (the root README states it), `cerulion` on the PATH, `libudev-dev` on Linux (the joystick node's gamepad backend) | build ahead of time |
| 0.5 | Companion: the workspace built | `README.md` "Build": one `cerulion node build <type> --release` per node, the camera last | build ahead of time |
| 0.6 | Companion: GStreamer for the camera | `nodes/camera_jpeg/BRINGUP.md` lists the header and plugin packages; `cerulion node build camera_jpeg --release` probes for them and says what is missing | verify with the loopback rig (0.8) |
| 0.7 | Gamepad paired to the companion (joystick teleop) | `nodes/joystick_teleop/BRINGUP.md` sections 1 and 2 | pair ahead of time |
| 0.8 | Smoke: the camera loopback rig, no robot | `cargo test -p camera_jpeg --features gstreamer --test loopback_e2e_test -- --nocapture`; the feature flag is REQUIRED, without it the binary fails naming the correct invocation rather than reporting a vacuous pass | run before robot day |
| 0.9 | File-descriptor limit on the companion | `ulimit -n 65536` in the shell that runs a graph; the image default is too low for a whole-robot bridge, and iceoryx2 "Corrupted" errors are usually this | set in the run shell |
| 0.10 | Desk: `cerulion` and Cerulion Studio | the robot's data renders in Studio: `cerulion viz` attaches the topics you name and Studio shows them | install Studio ahead of time (the root README links the downloads) |

The artifact-overwrite hazard, once: a plain `cargo build --release` in the
workspace rebuilds `camera_jpeg` WITHOUT capture and replaces the library
`cerulion node build camera_jpeg --release` produced. Re-run that `node
build` line after any workspace-wide rebuild, or the next `graph run`
refuses the graph naming the camera (which reads as a broken node rather
than a clobbered artifact).

## 1. Network bring-up

Reference: `../../docs/networking.md`. The robot is network-viewable by
default: a real-clock `graph run` with no `network:` block announces every
produced topic through the companion's one `cerulion-netd` and forwards a
topic only while a desk demands it.

The one required step, on every network, LAN included: give netd a listen
locator in the shell (or service environment) that will run the graph,
before you start it. Export the announced name in the same shell, so the
robot announces itself under the name this runbook uses:

```bash
# On the companion computer:
export CERULION_NETD_LISTEN=tcp/0.0.0.0:7683
export CERULION_ROBOT_IDENTITY=go2
```

netd takes its listen endpoints only from that variable and its default is
empty, so a netd started without it binds nothing and no desk can reach it
however close. With it set, the shared plane binds the port and advertises
the robot over mDNS under its announced name: `CERULION_ROBOT_IDENTITY` when
it is set, the companion's hostname otherwise. Every `--robot go2` below
expects the export above; if you skip it, read the announced name off
`cerulion topic list` under ROBOTS and pass that name instead.

1.1 Bring up the robot-LAN interface on the companion and confirm the
addresses. Replace `REPLACE_WITH_COMPANION_INTERFACE_IP` with the address
`ip addr` reports, here and in every later block that sets it:

```bash
ip addr                                # note the companion's robot-LAN interface IP  (check on your robot)
ping -c3 192.168.123.161               # the robot's main board (vendor convention)  (check on your robot)
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
```

1.2 Confirm the desk reaches the companion (its OTHER interface, the one on
your network), by hostname or address, with a plain `ping`.

1.3 Sanity-check the link before demanding anything (the companion graph
must be running, section 2):

```bash
# On the desk. Remote discovery is automatic; no flag turns it on:
cerulion topic list
# Off the LAN (another subnet, a VPN), name the companion's listener:
cerulion topic list --connect tcp/<companion address>:7683
# Expect a ROBOTS row for the companion and a REMOTE TOPICS section listing
# what it produces: /tf and /tf_static from the tf graph, /go2/utlidar/cloud,
# /go2/camera/h264 and /go2/camera/jpeg once the bridge graph runs.
```

The remote half is best-effort and loud: a bad locator errors naming the
`tcp/<host>:<port>` form, an unreachable `--connect` is dropped with a warn
rather than sinking the whole query, and a failed query is a note plus
exit 0 (the local list has already printed). Section 7 has the failures.

Do not add a `network:` block to any of the sensor graphs to "turn
networking on". A block TIGHTENS the run to Strict: a per-run gateway binds
the block's locators verbatim and ignores `CERULION_NETD_LISTEN`, and the
desk's discovery rungs that survive off the LAN all assume the well-known
port. A block is for a real egress allow-list or an `ingress:` import, which
is exactly what `graphs/teleop_remote.yaml` uses it for.

## 2. Sensors first, no actuation

Get the transform tree and the sensors rendering on the desk before
touching motion.

### 2a. See what this unit publishes

```bash
# On the companion:
cerulion ros2 attach --iface "$GO2_IFACE" --dry-run
```

Read the RESOLVABLE section against the three names in
`graphs/go2.bridge.yaml` (`/utlidar/cloud`, `/sportmodestate`,
`/frontvideostream`). A unit running a different vendor stack can publish a
different set; edit the mappings to match, or bridge everything with
`--yes` (`README.md` "Bring it up", step 1). **Check on your robot:** the report
from your unit is the fact; the names in the config are the vendor's.

### 2b. Transform tree

`go2_tf_source` publishes `/tf` (`odom` to `base`, an IDENTITY placeholder:
the odometry projection is not validated on a robot) and `/tf_static` (the
`base` to `lidar` and `base` to `camera` mounts, which are estimates:
**check on your robot**). The tree is self-contained from the stock firmware, no
autonomy stack required; `README.md` "Where the transforms come from" says
what data a real `odom` to `base` needs and where a SLAM stack fits.

```bash
# On the companion:
cerulion graph run tf --release --single-process
# On the desk, after 1.3 passes. `--robot` takes the announced name and
# needs at least one topic (there is no remote discover-all):
cerulion viz --robot go2 /tf /tf_static
```

Expected, in Studio: a Z-up frame tree with `odom`, `base`, `lidar` and
`camera`.

### 2c. Lidar and camera

```bash
# On the companion, in a second shell (both runs share the one netd):
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
export DDS_BRIDGE_CONFIG=graphs/go2.bridge.yaml
ulimit -n 65536
cerulion graph run bridge --release --single-process
```

Verify the rates on the companion once it runs (**check on your robot**):

```bash
cerulion topic hz /go2/utlidar/cloud   # the lidar cloud
cerulion topic hz /go2/camera/h264     # the raw route; both renditions ride it
cerulion topic hz /go2/camera/jpeg     # the decoded rendition
```

If `/go2/camera/h264` is silent the problem is the BRIDGE, not the camera:
`DDS_BRIDGE_CONFIG`, the `/frontvideostream` mapping, `GO2_IFACE`. Flowing
H.264 with no JPEG means the camera was built without capture (the graph
launch would have refused, section 0) or it is still waiting for a keyframe
(one warn says so).

Then on the desk:

```bash
cerulion viz --robot go2 /go2/utlidar/cloud /go2/camera/jpeg
```

The lidar producers write an empty PointCloud2 `fields` blob, so the
desk infers the XYZ / XYZI float layout from `point_step` (one warn, then
debug). Two renditions ride the camera topic; the node decodes the 720-line
one by default (`CAMERA_TARGET_HEIGHT=360` for the other).

## 3. Teleop dry run, robot powered OFF

Observe the safety mux before any actuation. `teleop_mux` (50 Hz)
arbitrates the gamepad against the keyboard and never lets a stale source
keep the robot moving:

| gamepad (fresh) | keyboard (fresh) | output |
|---|---|---|
| usable | any | the gamepad's command (a fresh centered stick is a real zero; it MUTES the keyboard) |
| stale, never, or garbage | usable | the keyboard's command |
| stale, never, or garbage | stale, never, or garbage | all zero (the safety zero) |

"Fresh" is a quarter of a second for the gamepad and three quarters for the
keyboard, measured on the frame's own wire stamp, exclusive at the
boundary; "usable" is fresh AND finite (the mux never forwards NaN or
infinities). Both sources publish an all-zero command at startup so the
mux's AND-gate opens at once.

Downstream, `sport_driver` turns the arbitrated stream into DDS requests:
`Move` on every nonzero frame, `StopMove` on the first zero after motion and
again every 200 ms while idle. Observe arbitration on `/go2/cmd_vel` and the
requests through the sport driver's counters and logs.

Dry run, with the robot OFF (no DDS peer, so the driver's writer has nobody
to talk to; every request is still counted and logged):

```bash
# On the companion, in a real terminal (ssh -t):
export GO2_IFACE="REPLACE_WITH_COMPANION_INTERFACE_IP"
cerulion graph run teleop --release --single-process
# On the desk:
cerulion viz --robot go2 /go2/cmd_vel /go2/cmd_vel/joystick /go2/cmd_vel/keyboard
```

Hold the right shoulder button and move the stick; release it and watch
`/go2/cmd_vel` drop to zero within the gamepad's window. Type WASD, stop
typing, and watch it drop within the keyboard's window. The contract is also
pinned without hardware by `nodes/teleop_mux/tests/mux_e2e_test.rs` and
`nodes/sport_driver/tests/driver_e2e_test.rs`.

## 4. On-stand actuation, the safe procedure

**Experimental:** the sport driver's api ids and JSON shape come from the
vendor SDK and are not validated against robot firmware. Your first live
check of them happens here, in this order.

Preconditions:

- The robot on a stand, feet clear of the ground.
- Stood up with the vendor remote or app (Damp, then StandUp): the driver
  never issues a posture command, by design.
- Speed limits low: the driver clamps to 0.6 m/s forward, 0.4 m/s sideways
  and 1 rad/s yaw, and the teleop sources saturate at the same values.
- The hardware e-stop within reach; it is the hard layer above everything
  here.

### 4.1 First motion

1. Start `graphs/teleop.yaml` as in section 3, robot ON, on the stand.
2. Hold the deadman and give the smallest forward nudge. The legs should
   move in place; the driver's log shows `Move` requests going out.
3. Release the deadman. Within the gamepad's window the mux zeros, the
   driver sends `StopMove`, and the legs stop.

If step 2 moves nothing: `GO2_IFACE` (the companion's robot-LAN interface),
the domain id, and whether the robot's sport service is up (the vendor app
reports it). If step 3 does not stop the legs: e-stop, and stop the demo.

### 4.2 Deadman verification, inline

What the software does is exactly this: the mux drops an input that has gone
stale and commands zero velocity, and the sport driver turns that into
`StopMove`. That is a command the robot is asked to obey, not a guarantee
that it stops: a wedged process, a dropped link or a fault below the driver
can leave the legs moving. The hardware stop is the layer above all of this
and stays within reach. Verify the software path three ways, robot secured
on the stand, before trusting teleop:

1. Release the deadman: one zero, then the stream goes quiet, the mux zeros
   within the gamepad's window.
2. Stop the keyboard node with Ctrl-C mid-motion: it publishes one zero,
   restores the terminal, then raises SIGINT one pump cycle later so the zero
   is processed first. That ordering is best effort; if the zero is lost, the
   keyboard input simply goes stale and the mux zeros the command when its
   keyboard window expires.
3. Drop the Bluetooth link (power the pad off): one zero and a loud warn,
   the mux zeros within the gamepad's window, and the node keeps scanning
   for the pad to come back (re-press the deadman to re-arm).

Per-source detail: `nodes/joystick_teleop/BRINGUP.md` section 3 (the
deadman, the 2 s no-event watchdog) and `nodes/keyboard_teleop/DEMO.md` (the
auto-zero, the Ctrl-C translation).

## 5. Teleop from the desk

`graphs/teleop_desk.yaml` runs the keyboard on your machine and
`graphs/teleop_remote.yaml` imports it on the companion through an
`ingress:` block; both headers say how the two find each other (mDNS on one
LAN, `CERULION_NETD_CONNECT` off it). Same preconditions as section 4.
**Experimental:** the graph pair validates; run it on a bench network before
you rely on it.

## 6. Where ROS 2 fits

The Go2 firmware is a bare CycloneDDS peer, so its traffic is always
bridged: `cerulion ros2 attach` discovers and generates, and this
workspace's `dds_bridge` node carries it. `rmw_cerulion` runs ROS 2 nodes
YOU write on Cerulion's transport; it does not speak DDS and cannot reach
the firmware, and on the vendor's Foxy image the rmw build is not available
yet (`../../docs/ros2_compatibility.md`). Where it fits is a ROS 2 node of
yours consuming the bridged topics; the attach report's MIGRATION section
lists which discovered processes that applies to. **Experimental:** a ROS 2
node running beside the bridge on a Go2 is not validated.

## 7. Record and replay

```bash
# On the companion, sensors and TF (the actuation graphs are not the ones to
# record: teleop_remote declares ingress, which is refused under --record):
cerulion graph run tf --release --single-process --record
# Ctrl-C to finalize, then re-execute the current node code against the bag:
cerulion bag play <bag> --resim all --verify
```

`--record` keeps the robot network-visible (the network never bleeds into
the bag; the bag is this machine's local run). A bag holding
`/go2/camera/jpeg` may report a byte difference on that topic: the JPEG bytes
come from a GStreamer encoder that is not in the bag, and its output is not
guaranteed to reproduce across encoders or versions. Compare decoded content
when you are evaluating image changes, and use byte-exact replay for the
deterministic node outputs (`nodes/camera_jpeg/BRINGUP.md` says which those
are).

## 8. Troubleshooting

| Symptom | Fix | Source |
|---|---|---|
| iceoryx2 "Corrupted", a node will not start | `ulimit -n 65536`; it is usually the file-descriptor limit, not corruption | 0.9 |
| Stale shared memory after a crash | `cerulion clean` between runs; start each demo from a clean state | `cerulion clean --help` |
| `topic list` shows no ROBOTS or REMOTE TOPICS | the companion graph is running? `CERULION_NETD_LISTEN=tcp/0.0.0.0:7683` exported on the companion? off the LAN, `--connect tcp/<companion address>:7683`? no `network:` block was added to a sensor graph? `GO2_IFACE` pinned to the robot-LAN interface (a multi-homed companion silently fails DDS discovery)? | 1, 2a |
| No camera video | `cerulion topic hz /go2/camera/h264` FIRST: silent means the BRIDGE (config, mapping, interface), not the camera; flowing but no JPEG means a capture-less build (the launch would have refused) or a wait for the first keyframe | 2c, `nodes/camera_jpeg/BRINGUP.md` |
| NVDEC pipeline will not link on a Jetson | `nvv4l2decoder ! nvjpegenc` may need an `nvvidconv` between them (not validated on a Jetson); use the software pipeline `avdec_h264 ! jpegenc` when its plugins are installed and the hardware pipeline is unavailable | `nodes/camera_jpeg/BRINGUP.md` |
| Gamepad not detected | Bluetooth pairing (`bluetoothctl`) and evdev enumeration; the user is not in the `input` group, or add the `uaccess` udev rule | `nodes/joystick_teleop/BRINGUP.md` |
| Terminal wedged after keyboard teleop | the restore is three-layered (shutdown, RAII guard, panic hook), so a clean or Ctrl-C exit restores it; if truly wedged, `reset` | `nodes/keyboard_teleop/DEMO.md` |
| The robot does not move on a Move request | `GO2_IFACE`, `GO2_DOMAIN_ID`, the sport service up (vendor app); the driver's `total_failures` counter in its warn line says whether the publish itself failed | 4.1 |
| The robot serves nothing after adding a `network:` block | remove it from the sensor graphs: a block TIGHTENS the run to Strict, which binds the block's locators verbatim and ignores `CERULION_NETD_LISTEN`; permissive (no block) is how a robot is served | 1, `../../docs/networking.md` |

## Validate on your robot

These parts are experimental: they are covered by tests without hardware and
are not validated on a robot. Check each one on yours.

1. The sport driver against live firmware (section 4).
2. The `odom` to `base` transform: `/go2/odom` is produced by the bridge, but
   the `SportModeState` layout is not validated against real robot bytes
   (the method is the one `schemas/unitree_go/msg/Go2FrontVideoData.msg`
   documents: capture raw samples, pin a fixture, then wire the input), so
   the TF source keeps its identity placeholder.
3. The static mounts and the camera `frame_id` (measure, or read the vendor
   URDF).
4. The camera's NVDEC link, latency legs and link byte rate
   (`nodes/camera_jpeg/BRINGUP.md`).
5. The gamepad's button-to-publish latency (`nodes/joystick_teleop/BRINGUP.md`
   section 4).
6. The desk-keyboard graph pair over a real link (section 5).
7. A `cerulion ros2 attach --dry-run` census of your unit against the three
   curated topic names (section 2a).
