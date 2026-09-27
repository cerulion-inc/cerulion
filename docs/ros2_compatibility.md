# ROS 2 compatibility

This page says which ROS 2 distributions Cerulion's two ROS 2 surfaces run on, what has actually been tested, and what is known not to work. The two surfaces are different pieces of software with different reach:

- `rmw_cerulion`, the `RMW_IMPLEMENTATION` that runs unmodified ROS 2 nodes on Cerulion's transport. Its reach is set by the ROS 2 C API (rmw, rcutils, rosidl) it is compiled against.
- `cerulion ros2 attach`, which discovers a robot's DDS topics and bridges them without any rmw or rcl linkage. Its reach is set by DDS wire behaviour (GID width, the type-description service).

Source references below are paths in this repository (under `crates/`, `tools/` and `examples/`); the line numbers move with the tree. Anything marked unconfirmed has no retained artifact behind it and is stated only as far as the evidence goes.

## 1. How the rmw is built, and why a deployed library is per distro

`rmw_cerulion` does not ship one binary for every distro. Its C ABI bindings are generated at build time from the ROS 2 headers of the distro that is installed on the build machine (`crates/rmw_cerulion/build.rs` lines 6 to 17). The build reads exactly three environment variables (`build.rs` lines 236 to 238):

| variable | effect |
|---|---|
| `CERULION_RMW_SYS_INCLUDE` | explicit include directories; a variable that names no existing directory fails the build (`build.rs` lines 693 to 720) |
| `AMENT_PREFIX_PATH` | every prefix whose `include/` carries the core ROS packages is used; the rmw, rcutils, rosidl, introspection, dynamic typesupport, type description and service headers are added per prefix (`build.rs` lines 722 to 771) |
| `ROS_DISTRO` | cross-checked against what the headers say, never used as the source of layout (`build.rs` lines 306 to 357 and 600 to 620) |

A build with real headers is the only supported deployment path; the tree's own comment calls it "the only ABI-safe path for a deployed librmw_cerulion.so". The release packages carry exactly that build: the Linux archives, the Debian package and the Homebrew formula on Linux install `librmw_cerulion.so` beside the `cerulion` binary, built inside the `ros:jazzy` image for x86_64 and arm64 by the release workflow (`tools/scripts/build_rmw_jazzy.sh`, which refuses a library whose build script did not report generated bindings), and every Debian package is installed in a clean Jazzy container and exercised with a talker, a listener and an rclpy identity check before it is published (`tools/scripts/verify_rmw_deb.sh`); the check deliberately does not run `ros2 topic echo`, because the ros2 CLI graph tools do not see Cerulion topics (section 4). The macOS packages carry no rmw because ROS 2 Jazzy publishes no macOS binaries; on macOS `cerulion ros2 run` and `cerulion ros2 launch` exit 69 with a message that says so. With no ROS installed, the crate falls back to committed bindings generated from pinned rolling clones of ros2/rmw, ros2/rcutils and ros2/rosidl (`crates/rmw_cerulion/src/ffi/vendored_bindings.rs` lines 4 to 13). That fallback exists so the crate compiles on a developer machine and in CI. It refuses to become a deployment: naming a distro through `ROS_DISTRO` while a header prefix is present makes the build fail rather than silently use the snapshot (`build.rs` lines 325 to 335), and a vendored library only admits `rmw_init_options_init` under `ROS_DISTRO` values of `lyrical` or `rolling` (`crates/rmw_cerulion/src/era.rs` lines 650 to 658).

The build bakes the distro it was compiled for into the library (`CERULION_RMW_BUILT_FOR_DISTRO`, `build.rs` lines 212 to 233). At load time the rmw compares that claim with the runtime `ROS_DISTRO` and refuses a mismatch inside `rmw_init_options_init`, before any typed operation (`crates/rmw_cerulion/src/api/init.rs` lines 23 to 59; refusal text in `era.rs` lines 706 to 713). A runtime with no `ROS_DISTRO` set passes the check (`era.rs` lines 636 to 640). Distros with identical layouts are treated as one (`lyrical` and `rolling`); Jazzy and Kilted are kept apart because their `rmw_init_options_t` sizes differ (168 versus 160 bytes, `era.rs` lines 626 to 633).

### The exported entry points

The library exports 112 `rmw_*` functions and nothing else (31 in `api/misc.rs`, 28 in `api/pubsub.rs`, 19 in `api/guard_wait.rs`, 18 in `api/services.rs`, 8 in `api/init.rs`, 8 in `api/node.rs`). The inventory mirrors `rmw_iceoryx2` on rolling (`crates/rmw_cerulion/src/api/mod.rs` line 4). The rmw C API carries no version constant, so there is no single "rmw API version" to quote; what changes between distros is the set of symbols rcl looks up and the layout of the structs they exchange. The export set includes symbols that first appeared in Galactic (`rmw_qos_profile_check_compatible`, the network flow endpoint functions), Humble (`rmw_feature_supported`, content filter functions), Iron (`rmw_take_dynamic_message`, `rmw_serialization_support_init`) and Jazzy (`rmw_event_type_is_supported`), all exported unconditionally (`api/misc.rs` lines 482 to 518, 1254 to 1308; `api/pubsub.rs` lines 4090 to 4123; `api/guard_wait.rs` line 2415). Several of those return `RMW_RET_UNSUPPORTED`; the list is in section 4.

### What a distro with a different rmw API hits

Three failure classes, two of them now caught before they can crash a node:

1. **Struct layout drift.** The introspection `MessageMember` is 96 bytes on distros without `fetch_function`, 112 on Humble, Iron and Jazzy (Jazzy adds `is_key_` at offset 32 without changing the size), 120 on Lyrical and Rolling; the GID storage is 24 bytes before Iron and 16 from Iron on; `rmw_init_options_t` is 104 bytes before Iron, 168 on Iron and Jazzy, 160 from Kilted on (`crates/rmw_cerulion/src/ffi/era_pins.rs` lines 59 to 118, 177 to 196, 231 to 297; `docs/internals/rmw.md` lines 149 to 151 and 261 to 263). A library built for one layout and loaded into another passes `rmw_init` and then fails at the first typed operation: a Jazzy-shaped member stride walks over Foxy's smaller struct and faults in `strlen` under `rmw_create_publisher` (`era.rs` lines 5 to 11). The compile-time pins in `era_pins.rs` refuse a header set that contradicts the selected bindings, and the load-time distro guard above refuses a runtime distro that contradicts the baked claim.
2. **Missing symbols.** rcl resolves rmw entry points by name at load. A distro that looks up a symbol the library lacks does not fail the load; it logs a resolve error on every node start. That applies to `rmw_event_type_is_supported`, a Jazzy-era probe: omitting the symbol, rather than exporting it and returning false, makes rcutils log a resolve error at every node start (`api/guard_wait.rs` lines 2409 to 2413). The reverse direction, a library exporting symbols an older distro never looks up, is harmless.
3. **Layout inside rmw structs the library fills.** `rmw_subscription_t` gains fields across distros (rolling has `is_cft_supported`, Jazzy does not), so the rmw zeroes the struct and stores fields by name instead of naming every field (`api/pubsub.rs` lines 2374 to 2384). `rmw_publisher_options_t` is copied whole (`api/pubsub.rs` line 724). The first 40 bytes of `rmw_init_options_t` are the same on every distro and everything after them differs (`api/init.rs` lines 39 to 43).

Two further hard limits: the library is 64-bit only (`crates/rmw_cerulion/src/ffi/introspection_cpp.rs` lines 123 to 127), and the rclcpp typesupport bridge is written to the Jazzy and rolling `MessageMember` layout; on a pre-Jazzy distro the C++ path is refused at registration while the C path used by rclpy is unaffected (`introspection_cpp.rs` lines 10 to 13; `era.rs` lines 98 to 112 and 514 to 535; `docs/internals/rmw.md` lines 316 to 317).

## 2. What has been tested

Every artifact that shows ROS 2 running over `rmw_cerulion` is ROS 2 Jazzy.

| distro and OS | machine and arch | what ran | evidence |
|---|---|---|---|
| Jazzy, Ubuntu 24.04 container (`ros:jazzy`) | x86-64 Linux desktop, Ubuntu 22.04 host | the rmw e2e (rclpy talker and listener in both directions across processes), the fully loaned `rmw_cerulion` cell of the native latency harness, the `cerulion ros2 migrate` prover matrix | `tools/ros2_toolchain/Dockerfile` lines 1 to 35; `crates/rmw_cerulion/tests/rclpy_xproc_test.rs` lines 55 to 76; campaign package [ROS 2 over rmw_cerulion in the native harness](benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/) (all 50 cells in its `run.json` are `jazzy_cerulion_shm_loan_be1_chrt0`, run in the `latency_bench:jazzy` image), committed under `docs/benchmarks/results/`; `tools/ros2_migrate/run_matrix.sh` line 320 |
| Jazzy container | aarch64 Linux (Jetson class) | the `rmw_cerulion` ping-pong runs behind the wake policy default | `crates/rmw_cerulion/src/api/guard_wait.rs` lines 330 to 336; `docs/internals/rmw.md` lines 451 to 454 |
| Jazzy container | hosted amd64 CI runner | unmodified MoveIt 2 (`move_group`, OMPL plan, Pilz determinism) nightly, gated on the rmw identifier rclpy reports | `.github/workflows/moveit-hero.yml` lines 31 to 67; `examples/moveit_hero/scripted_plan.py` lines 122 to 153 |
| Jazzy container | arm64 Docker on an Apple Silicon workstation and the amd64 hosted runner | four regression lanes (an rclcpp action server, parameter services, a lifecycle node, `/clock` with `use_sim_time`) | no retained artifact in this repository, and the lanes are not in it |
| Jazzy, RoboStack conda environment, native macOS | Apple Silicon workstation | a six-axis industrial arm driver, MoveIt 2 and RViz on `rmw_cerulion`, with the arm moving under it; `rmw_cerulion` built against the RoboStack headers | no retained artifact in this repository |

Not evidence of the rmw: the `moveit_hero` demo requires Docker and has not been shown running on macOS; the stock and composed ROS 2 comparison lanes in `benches/latency/ros2` are Jazzy too but do not use `rmw_cerulion`; the one cell of that suite that does is the fully loaned cell in the table above (`benches/latency/ros2/run_bench.sh` lines 24 to 32).

`cerulion ros2 attach` and `rmw_cerulion` have separate compatibility requirements, and attach reaches two distros the rmw has no evidence for: a Unitree Go2 running ROS 2 Foxy over CycloneDDS, where attach bridged 75 to 85 topics through the ament and local rungs (`crates/cerulion_core/src/transport/liveness.rs` line 13), and Humble peers on an x86-64 desktop, where the ament harvest rung produced seven `.msg` files byte-identical to `/opt/ros/humble`. Neither run has a retained artifact in this repository, and the wire rung is skipped on both; see section 3.

## 3. Distribution matrix

States: **validated** means an artifact above shows it running; **expected, unvalidated** means the code carries an arm for it that has never executed in this tree (`crates/rmw_cerulion/src/ffi/era_pins.rs` lines 19 to 26 say so for every arm other than the vendored rolling snapshot and the Jazzy container); **blocked** means the tree refuses it or it is known to fail.

### `rmw_cerulion`

| distro | state | detail |
|---|---|---|
| Foxy, Galactic | blocked | pre-Iron rmw API (104-byte init options, 24-byte GID); a Jazzy-built library faulted on a Foxy robot (`era.rs` lines 5 to 11); a Foxy or Humble build fails at compile by design |
| Humble | blocked | same pre-Iron API and 24-byte GID; the compile failure above; the rclcpp bridge refuses pre-Jazzy layouts (`era.rs` lines 514 to 520); no Humble lane exists |
| Iron | expected, unvalidated | 16-byte GID and the 168-byte init options match Jazzy, and the `MessageMember` arm is the Humble and Iron 112-byte one (`era_pins.rs` lines 76 to 85); the rclcpp bridge is refused as pre-Jazzy, so only the C typesupport path (rclpy) would run; never built or run in this tree |
| Jazzy | validated | section 2; the only distro the rmw has executed against, and the one the release packages ship a library for (Linux x86_64 and arm64) |
| Kilted | expected, unvalidated | its own claim (160-byte init options, `crates/rmw_cerulion/src/era_check.rs` line 512) and otherwise the Jazzy era; a Jazzy-built library is refused under Kilted by name (`tests/rmw_era_guard_test.rs` lines 9 to 15); `docs/internals/rmw.md` line 259 keeps it outside the support matrix; never run |
| Rolling (Lyrical era) | expected, unvalidated | the vendored snapshot is pinned to rolling and compiles and unit-tests in CI, but a deployed library must be built against current rolling headers; an outdated rolling tree fails as a Jazzy-era contradiction (`docs/internals/rmw.md` lines 262 to 263); the rclcpp bridge admits the Lyrical/Rolling layout (the C++ mirror carries `is_rosidl_buffer_` under its capability cfg); never run against a rolling node; on Lyrical and Rolling an unbounded `uint8[]` member is a rosidl Buffer rather than a `std::vector`, and the rclcpp bridge fills it through the introspection accessors (no forged zero-copy take for those members); a vendored development build refuses the rclcpp path unless `ROS_DISTRO` names lyrical or rolling |

### `cerulion ros2 attach`

| distro | state | detail |
|---|---|---|
| Foxy, Galactic, Humble | validated on Foxy and Humble, via the local rungs only | no type hash and no `get_type_description` service, so the wire rung skips loudly and the ament harvest resolves the types (`docs/schema_resolution.md` lines 244 to 252 and 267 to 275; `crates/cerulion_dds/src/wire.rs` lines 69 to 70) |
| Iron | expected, unvalidated | the wire rung needs the patched Iron releases (rclcpp 21.0.3, rclpy 4.1.3 or later, as applicable); a pre-patch Iron .0 advertises hashes with no service and falls to the local rung (`docs/schema_resolution.md` lines 31 to 36 and 272 to 273) |
| Jazzy | validated | the wire rung's live capture is from the Jazzy container (`crates/cerulion_dds/src/wire.rs` line 116) |
| Kilted, Rolling | expected, unvalidated | inside the wire rung's stated domain (`docs/schema_resolution.md` lines 31 to 36); no run |

The DDS crate defaults to the 16-byte GID world (`crates/cerulion_dds/Cargo.toml` lines 44 to 84, `default = ["jazzy"]`). Its `humble` feature selects the 24-byte GID and is an explicit opt-in that cannot decode Iron-or-newer discovery (`crates/cerulion_dds/src/wire.rs` lines 807 to 817). No CI step builds that feature (the only `cerulion_dds` job runs default features, `.github/workflows/ci.yml` lines 3105 to 3106), so it is unvalidated; the Go2 example's own DDS wrapper crate is what carries a default `humble` build (`examples/go2/lib/cerulion_go2_dds/Cargo.toml` lines 55 to 57).

## 4. What runs unmodified over the rmw, and the known limits

Runs unmodified, with evidence:

- rclpy and rclcpp nodes: the cross-process rclpy test in both directions (`crates/rmw_cerulion/tests/rclpy_xproc_test.rs`), the three rclcpp nodes of the `rmw_cerulion` cell in the latency suite (`benches/latency/ros2`), `move_group` and `ros2_control` in the MoveIt hero.
- `ros2` CLI verbs through `cerulion ros2 run` and `cerulion ros2 launch`: pass-through wrappers that set `RMW_IMPLEMENTATION=rmw_cerulion`, prepend the library directory and a minimal ament prefix so rmw discovery can load the library, add the heap hook preload, and exec `ros2` (`crates/cerulion_cli_engine/src/ros2_cmd.rs` lines 2 to 34, 2373 to 2420, 2498 to 2534; user docs `docs/user-api.md` lines 163 to 164). Unix only (`ros2_cmd.rs` lines 1228 to 1233), and in practice Linux only: the library ships in the Linux packages, and a missing library is exit 69 with an install-route remedy on Linux or, on macOS, the statement that ROS 2 Jazzy has no macOS binaries and the verbs need a Linux machine. No distro gate of their own.
- Launch files: the MoveIt hero starts through `ros2 launch` under the rmw (`examples/moveit_hero/entrypoint.sh` lines 84 to 93).
- MoveIt 2 with zero source edits: the scheduled workflow above plans with OMPL and executes the trajectory through `joint_trajectory_controller` over `rmw_cerulion` (`examples/moveit_hero/`).
- Parameters (they ride ordinary rmw services, `docs/internals/rmw.md` lines 462 to 463), services (18 exports, e2e in `docs/internals/rmw.md` line 1007), guard conditions and `rmw_wait`, TRANSIENT_LOCAL late-joiner delivery with a raised subscriber ceiling (`docs/internals/rmw.md` lines 361 to 364 and 985 to 996). Actions have no rmw-level API; they run as the topics and services beneath them, and the MoveGroup action is what the MoveIt hero exercises.
- Zero-copy publish and take need the Jazzy opt-in `ROS_DISABLE_LOANED_MESSAGES=0` and the heap hook the `ros2` verbs preload (`libcerulion_heaphook.so`, shipped beside the rmw in the Linux packages) (`api/pubsub.rs` lines 2393 to 2394; `docs/internals/rmw.md` line 698); without them the rmw takes the copy path.

Known limits:

- **Endpoint discovery is process-local, so the ros2 CLI graph tools do not see Cerulion topics.** `rmw_get_publishers_info_by_topic` and `rmw_get_subscriptions_info_by_topic` answer from the process's own registry, and so does the graph query behind `ros2 topic list`, `ros2 topic echo` and `ros2 node list`. In a Jazzy container with a live `demo_nodes_cpp` talker, `ros2 topic list` (with and without the daemon), `ros2 topic echo --no-daemon` with an explicit type, and rclpy's `get_topic_names_and_types()` from a third process each see only that process's own `/parameter_events` and `/rosout`, never `/chatter`, while a second `cerulion ros2 run demo_nodes_cpp listener` receives it (pub/sub rendezvous is by name on the shared-memory service). Inspect Cerulion topics with `cerulion topic list` and `cerulion topic echo`, which read the shared-memory plane; cross-process endpoint discovery is not implemented (`docs/internals/rmw.md` lines 997 to 999 and 1050).
- **Multi-node graphs need an `iceoryx2.toml`.** Every ROS 2 node publishes on `/rosout` and `/parameter_events`, and iceoryx2's default caps publishers per service at 2, so a third node dies at startup unless the cap is raised; the MoveIt hero image installs a config raising publishers and subscribers to 32 and event ports to 64 (`examples/moveit_hero/README.md` lines 109 to 122; `examples/moveit_hero/iceoryx2.toml` lines 22 to 34; `api/pubsub.rs` lines 1061 to 1072; `docs/multi_process.md` lines 401 to 410).
- **SROS2 is not supported.** The rmw contains no handling of `ROS_SECURITY_*`; the security options rcl fills are carried in the init options and never consulted (`api/init.rs` lines 43 and 152). Setting `ROS_SECURITY_ENABLE=true` does not secure a Cerulion transport, and the rmw neither honours nor explicitly refuses it.
- **Unsupported entry points return `RMW_RET_UNSUPPORTED`:** publisher and subscription events and their callbacks (`api/guard_wait.rs` lines 2395 to 2485), the new-message, new-request and new-response callbacks (same range), content filters (`api/pubsub.rs` lines 4090 to 4103), publisher and subscription allocations (`api/pubsub.rs` lines 2106 to 2115 and 4106 to 4123), network flow endpoints (`api/misc.rs` lines 1292 to 1308), dynamic messages and `rmw_serialization_support_init` (`api/misc.rs` lines 482 to 518), `rmw_get_serialized_message_size` (`api/misc.rs` lines 209 to 216). Loaned messages are refused for non-loanable types and when the heap hook is absent (`api/pubsub.rs` lines 1326 to 1328 and 1462 to 1463).
- **Platforms:** 64-bit Linux and macOS only (`README.md` lines 173 to 175; `introspection_cpp.rs` lines 123 to 127). No Windows. The released rmw library is Linux only (x86_64 and arm64, built on Ubuntu 24.04); the macOS run in section 2 used a library built by hand against RoboStack headers, which no package ships.
- **Two vendored message types hash-skew against a stock Jazzy robot** (`control_msgs/PidState`, `control_msgs/SteeringControllerStatus`; `docs/internals/native-ros2-messages.md` lines 145 to 148); resolving the schema from the robot through the attach wire rung is the remedy.

## Claims without a retained artifact

Some runs cited above have no artifact in this repository: the four ROS 2 regression lanes, the distro guard, the Go2 attach, the macOS arm demonstration and the Foxy fault. Every row that rests on one says so.

## 5. Evidence behind the matrix

Every validated `rmw_cerulion` execution used ROS 2 Jazzy: a `ros:jazzy` container on an x86-64 Linux desktop and on an aarch64 Jetson-class machine, a hosted amd64 CI runner, arm64 Docker on an Apple Silicon workstation, and a native RoboStack Jazzy environment on that workstation. No other distro has run it. The one time an `rmw_cerulion` library was loaded under another distro, ROS 2 Foxy, it passed `rmw_init` and then failed at the first typed operation, and a rebuild against real Foxy headers stopped at bindgen. The non-Jazzy ROS hosts and containers that appear in benchmark records ran stock RMWs or the DDS bridge, never `rmw_cerulion`.

### What the tree carries for the other distros

- Header probes with gated includes, so bindgen stops aborting on Foxy and Humble headers; capability cfgs; per-era struct size pins; and a load-time refusal when the library and the running distro disagree. None of it changes the Jazzy path.
- A Foxy or Humble build still fails at compile, by design: the gates refuse the layouts those eras need, and this tree carries no Foxy or Humble container lane.

### Per-distro evidence

The states are the ones defined in section 3. **Expected, unvalidated** covers both a distro whose code arms exist but have never been compiled against its headers and one that compiles but has never run against a node.

| distro | rmw_cerulion | evidence | attach | evidence |
|---|---|---|---|---|
| Foxy | blocked | a Jazzy-built library loaded, passed `rmw_init` and faulted at the first typed operation; a rebuild against real Foxy headers fails at bindgen (`era.rs` lines 5 to 11); no Foxy build exists | validated, local rungs | a Unitree Go2 on Foxy: 75 to 85 topics bridged; the wire rung skips loudly, because a pre-Iron distro advertises no type hash |
| Galactic | blocked | pre-Iron, so it fails at compile for the same reason as Foxy and Humble; the size-pin arm shares Foxy's 96-byte `MessageMember` (`era_pins.rs`); never built or run | expected, unvalidated | the same pre-Iron path as Foxy; no run |
| Humble | blocked | the compile fails by design; the Foxy bindgen abort extends to Humble; the rclcpp bridge refuses pre-Jazzy layouts; no Humble build or run | validated, local rungs | Humble peers on an x86-64 Linux desktop: one attach produced seven `.msg` files byte-identical to `/opt/ros/humble` |
| Iron | expected, unvalidated | the 16-byte GID and 168-byte init options match Jazzy and the `MessageMember` arm is the Humble and Iron 112-byte one; never built or run | expected, unvalidated | inside the wire rung's domain only on the patched releases (rclcpp 21.0.3, rclpy 4.1.3 or later, as applicable); a pre-patch Iron .0 falls to the local rung; no run |
| Jazzy | validated | section 2, including the rmw end-to-end test, the loaned latency cell and the MoveIt workflow | validated | the wire rung's live capture from the Jazzy container (`crates/cerulion_dds/src/wire.rs` line 116) |
| Kilted | expected, unvalidated | its own claim token (160-byte init options); a Jazzy-built library is refused under Kilted by name (`tests/rmw_era_guard_test.rs`); `docs/internals/rmw.md` keeps it outside the support matrix; never built or run | expected, unvalidated | inside the wire rung's stated domain; no run |
| Lyrical (and Rolling, identical layout) | expected, unvalidated | the Lyrical and Rolling era arm exists (120-byte `MessageMember` with `is_rosidl_buffer_`, 160-byte init options), and the vendored bindings are Lyrical-era rolling and compile and unit-test in CI, but that path refuses deployment; a build against real Lyrical headers in the CI lane; the rclcpp C++ bridge admits Lyrical and Rolling at registration by design (`era.rs` lines 524 to 535) | expected, unvalidated | inside the wire rung's stated domain; no run |

### Stock ROS 2 comparison lanes, kept apart from the rmw

Benchmark records name distros other than Jazzy. Those lanes are stock ROS 2 nodes under stock RMWs, measured as the comparison side of a benchmark; none of them ran `rmw_cerulion`.

| distro | what was measured | artifact |
|---|---|---|
| Humble host, and humble, jazzy and kilted containers | an earlier round-trip campaign across `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp` and `rmw_zenoh_cpp`, with and without SHM | not published with this repository; it carries no Cerulion rows |
| humble, jazzy and lyrical containers | the public latency suite's stock ROS 2 lanes (cyclonedds, fastdds, zenoh; stock and composed) | `benches/latency/ros2/docker/Dockerfile` header lines 21 to 36 (the `rmw_cerulion` cell is its own cell, not one of these stock lanes); every published `run.json` says `ros_distro jazzy`, so only the Jazzy stock cells are packaged |
| Lyrical (message corpus only) | the distro pin for `control_msgs`, because 13 of its 39 messages do not exist in Jazzy: a manifest source line, not a run | `crates/native_ros2_messages/upstream_msg_manifest.txt` lines 27 and 60 |
