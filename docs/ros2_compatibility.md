# ROS 2 compatibility

Cerulion has two ROS 2 surfaces, and they reach different distributions because
they are different pieces of software:

- **`rmw_cerulion`** runs unmodified ROS 2 nodes on Cerulion's transport. Its
  reach is set by the ROS 2 C API (rmw, rcutils, rosidl) it is compiled against.
- **`cerulion ros2 attach`** discovers a running stack's DDS topics and bridges
  them, with no rmw or rcl linkage at all. Its reach is set by DDS wire
  behaviour: the GID width and the type-description service.

```bash
# Run your stack on Cerulion's transport, one launch file or one executable at a time.
cerulion ros2 launch my_robot_bringup camera.launch.py
cerulion ros2 run my_robot_drivers imu_node

# Or bridge a stack that is already running, from this computer's interface on its LAN.
cerulion ros2 attach --iface <local-interface-IP>

# Preview which C++ publish sites can move to loaned messages, then apply it.
cerulion ros2 migrate --workspace ~/ros2_ws
cerulion ros2 migrate --workspace ~/ros2_ws --write
```

## Which distributions each surface runs on

**Supported** means it has been run and the evidence is in this repository.
**Untested** means the code carries an arm for the distribution that has never
executed here. **Not supported** means the tree refuses it, or it is known to
fail.

### `rmw_cerulion`

| Distribution | State | Why |
|---|---|---|
| Foxy, Galactic, Humble | Not supported | The pre-Iron rmw API: 104-byte init options and a 24-byte GID. A build against those headers fails at compile by design, and a Jazzy-built library loaded on a Foxy robot passed `rmw_init` and then faulted at the first typed operation. |
| Iron | Untested | Its GID width and init-options size match Jazzy, so the arm exists. The rclcpp bridge is refused as pre-Jazzy, so only the C typesupport path that rclpy uses would run. Never built or run here. |
| Jazzy | **Supported** | The only distribution the rmw has executed against, and the one the Linux release packages ship a library for (x86_64 and arm64). |
| Kilted | Untested | Its own era claim (160-byte init options) and otherwise the Jazzy era. A Jazzy-built library is refused under Kilted by name. Never run. |
| Rolling, Lyrical | Untested | The vendored snapshot is pinned to rolling and compiles and unit-tests in CI, but a deployed library has to be built against current rolling headers. The rclcpp bridge is refused post-Jazzy. Never run against a rolling node. |

### `cerulion ros2 attach`

Attach reaches two distributions the rmw does not, because it needs no ROS
headers at all.

| Distribution | State | Why |
|---|---|---|
| Foxy, Humble | **Supported**, through the local rungs | A pre-Iron distribution advertises no type hash and runs no `get_type_description` service, so the wire rung skips loudly and the ament harvest resolves the types instead. A Unitree Go2 on Foxy bridged 75 to 85 topics this way; a Humble peer produced seven `.msg` files byte-identical to `/opt/ros/humble`. |
| Galactic | Untested | The same pre-Iron path as Foxy. No run. |
| Iron | Untested | The wire rung needs the patched Iron releases (rclcpp 21.0.3, rclpy 4.1.3 or later); a pre-patch Iron advertises hashes with no service and falls back to the local rung. |
| Jazzy | **Supported** | Including the wire rung, whose live capture is from the Jazzy container. |
| Kilted, Rolling | Untested | Inside the wire rung's stated domain. No run. |

The DDS layer defaults to the 16-byte GID world (Iron and later). Its `humble`
feature selects the 24-byte GID and is an explicit opt-in that cannot decode
Iron-or-newer discovery. No CI job builds that feature.

## A deployed rmw library is per distribution

`rmw_cerulion` does not ship one binary for every distribution. Its C ABI
bindings are generated at build time from the ROS 2 headers installed on the
build machine, and the build reads exactly three environment variables:

| Variable | Effect |
|---|---|
| `CERULION_RMW_SYS_INCLUDE` | Explicit include directories. A variable naming a directory that does not exist fails the build. |
| `AMENT_PREFIX_PATH` | Every prefix whose `include/` carries the core ROS packages is used. |
| `ROS_DISTRO` | Cross-checked against what the headers say, never used as the source of layout. |

A build against real headers is the only supported deployment path, and the
release packages carry exactly that: the Linux archives, the Debian package and
the Homebrew formula on Linux install `librmw_cerulion.so` beside the `cerulion`
binary, built inside the `ros:jazzy` image for x86_64 and arm64. Every Debian
package is installed in a clean Jazzy container and exercised with a talker, a
listener and an rclpy identity check before it is published. The macOS packages
carry no rmw, because ROS 2 Jazzy publishes no macOS binaries; there
`cerulion ros2 run` and `cerulion ros2 launch` exit 69 saying so.

With no ROS installed, the crate falls back to committed bindings from pinned
rolling clones so it still compiles on a developer machine and in CI. That
fallback refuses to become a deployment: naming a distribution through
`ROS_DISTRO` while a header prefix is present fails the build rather than
silently using the snapshot.

The build bakes the distribution it compiled for into the library, and at load
time the rmw compares that claim against the runtime `ROS_DISTRO` and refuses a
mismatch inside `rmw_init_options_init`, before any typed operation. A runtime
with no `ROS_DISTRO` set passes. Distributions with identical layouts are
treated as one; Jazzy and Kilted are kept apart because their init-options sizes
differ.

The library exports 112 `rmw_*` functions and nothing else, mirroring
`rmw_iceoryx2` on rolling. The rmw C API carries no version constant, so what
changes between distributions is the set of symbols rcl looks up and the layout
of the structs they exchange. Symbols that first appeared in Galactic, Humble,
Iron and Jazzy are all exported unconditionally: omitting one makes rcutils log
a resolve error at every node start, whereas exporting a symbol an older
distribution never looks up is harmless. Several of them return
`RMW_RET_UNSUPPORTED`; see the limits below.

## What runs unmodified

- **rclpy and rclcpp nodes.** Pinned by a cross-process rclpy test in both
  directions, and by the three rclcpp nodes of the `rmw_cerulion` cell in the
  latency suite.
- **`ros2` CLI verbs through `cerulion ros2 run` and `cerulion ros2 launch`.**
  They set `RMW_IMPLEMENTATION=rmw_cerulion`, prepend the library directory and
  a minimal ament prefix, add the heap-hook preload and exec `ros2`. Unix only,
  and in practice Linux only, because the library ships in the Linux packages.
- **Launch files, and MoveIt 2 with zero source edits.** A scheduled workflow
  plans with OMPL and executes the trajectory through
  `joint_trajectory_controller` over `rmw_cerulion`; see
  [`examples/moveit_hero/`](../examples/moveit_hero/).
- **Parameters, services, guard conditions and `rmw_wait`**, and
  TRANSIENT_LOCAL late-joiner delivery with a raised subscriber ceiling. Actions
  have no rmw-level API; they run as the topics and services beneath them, which
  is what the MoveIt example exercises.
- **Zero-copy publish and take**, with the Jazzy opt-in
  `ROS_DISABLE_LOANED_MESSAGES=0` and the heap hook the `ros2` verbs preload
  (`libcerulion_heaphook.so`, shipped beside the rmw in the Linux packages).
  Without both, the rmw takes the copy path.

## Known limits

- **Endpoint discovery is process-local, so the ros2 CLI graph tools do not see
  Cerulion topics.** `ros2 topic list`, `ros2 topic echo`, `ros2 node list` and
  rclpy's `get_topic_names_and_types()` answer from the calling process's own
  registry, so in a container with a live talker they see only that process's
  `/parameter_events` and `/rosout`, never `/chatter`, while a second
  `cerulion ros2 run demo_nodes_cpp listener` receives it: pub/sub rendezvous is
  by name on the shared-memory service. Inspect Cerulion topics with
  `cerulion topic list` and `cerulion topic echo`, which read that plane
  directly. Cross-process endpoint discovery is not implemented.
- **A multi-node graph needs an `iceoryx2.toml`.** Every ROS 2 node publishes on
  `/rosout` and `/parameter_events`, and iceoryx2 caps publishers per service at
  2 by default, so a third node dies at startup unless the cap is raised. Raise
  the pub/sub and event caps together, because one notifier and one listener are
  armed per publisher and per subscriber:

  ```toml
  [defaults.publish-subscribe]
  max-publishers = 32
  max-subscribers = 32

  [defaults.event]
  max-notifiers = 64
  max-listeners = 64
  ```

  The MoveIt example bakes that file into `$HOME/.config/iceoryx2/iceoryx2.toml`.
- **SROS2 is not supported.** The rmw contains no handling of `ROS_SECURITY_*`.
  The security options rcl fills are carried in the init options and never
  consulted, so `ROS_SECURITY_ENABLE=true` does not secure a Cerulion transport,
  and the rmw neither honours nor explicitly refuses it.
- **These entry points return `RMW_RET_UNSUPPORTED`:** publisher and
  subscription events and their callbacks, the new-message, new-request and
  new-response callbacks, content filters, publisher and subscription
  allocations, network flow endpoints, dynamic messages,
  `rmw_serialization_support_init` and `rmw_get_serialized_message_size`. Loaned
  messages are refused for non-loanable types and when the heap hook is absent.
- **Platforms: 64-bit Linux and macOS only.** No Windows. The released rmw
  library is Linux only (x86_64 and arm64, built on Ubuntu 24.04); a macOS build
  against RoboStack headers has been run by hand, but no package ships one.
- **Two vendored message types hash-skew against a stock Jazzy robot**
  (`control_msgs/PidState` and `control_msgs/SteeringControllerStatus`).
  Resolving the schema from the robot through the attach wire rung is the
  remedy.
- **The rclcpp typesupport bridge is written to the Jazzy and rolling
  `MessageMember` layout.** On a pre-Jazzy distribution the C++ path is refused
  at registration; the C path that rclpy uses is unaffected.

## What was actually run

Every validated `rmw_cerulion` execution used ROS 2 Jazzy:

| Machine | What ran |
|---|---|
| `ros:jazzy` container, x86-64 Linux desktop | The rmw end-to-end test (rclpy talker and listener in both directions across processes), the fully loaned `rmw_cerulion` cell of the native latency harness, and the `cerulion ros2 migrate` prover matrix |
| Jazzy container, aarch64 Linux (Jetson class) | The `rmw_cerulion` ping-pong behind the wake policy default |
| Jazzy container, hosted amd64 CI runner | Unmodified MoveIt 2 (`move_group`, an OMPL plan, Pilz determinism), nightly, gated on the rmw identifier rclpy reports |
| Jazzy container on Apple Silicon and the hosted runner | Four regression lanes: an rclcpp action server, parameter services, a lifecycle node, and `/clock` with `use_sim_time` |
| Native macOS, RoboStack Jazzy, Apple Silicon | A six-axis arm driver, MoveIt 2 and RViz on `rmw_cerulion`, with the arm moving under it, against a library built by hand from the RoboStack headers |

The packaged measurements are the
[latency campaign](benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/);
the last two rows above, the distro guard, the Go2 attach and the Foxy fault
have no retained artifact in this repository.

The stock ROS 2 lanes in the benchmark suite name distributions other than
Jazzy. Those are stock nodes under stock RMWs, measured as the comparison side;
none of them ran `rmw_cerulion`.

## See also

- [`docs/schema_resolution.md`](schema_resolution.md): how attach resolves each
  discovered type, and what each rung needs from the robot.
- [`examples/moveit_hero/`](../examples/moveit_hero/): an unmodified MoveIt 2
  stack over the rmw.
