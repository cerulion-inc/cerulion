# ROS 2 Usage Patterns Memo: evidence for the bench's ROS 2 lanes

<!-- Evidence snapshot: 2026-08-13. Search counts below describe this snapshot. -->

**Method:** upstream PR/doc reads + GitHub-wide code search (counts describe
the 2026-08-13 snapshot, not the current code; searches are quoted so they can be re-run). Written to be quotable from
`benches/latency/README.md` / `METHODOLOGY.md`. This memo cites external sources only; where the
bench's own `METHODOLOGY.md` §15 has *measured* the same fact (e.g. rmw_fastrtps forcing
DataSharing off), the two agree.

---

## Headline verdicts

1. **Loan-take (`rcl_take_loaned_message` / loaned subscriptions) is a benchmark-and-vendor-demo
   feature, not a production pattern.** Upstream disabled it by default for safety
   (rclcpp#2335 / rcl#1110, backported to Humble & Iron); a GitHub-wide search finds **zero** uses
   in Autoware, Nav2, MoveIt, or any mainstream driver; the callers are benchmark tools, academic
   executor forks, and vendor SDKs. Autoware wanted zero-copy badly enough that it built a separate
   kernel-module IPC (**Agnocast**) rather than use the in-tree loan path.
2. **Fastest-common ROS 2 intra-host = composition (component containers), optionally +
   intra-process comms.** Nav2 has shipped composed bringup **by default since Humble** (PR #2750;
   their own numbers: CPU 44%→38%, memory 0.76%→0.23% vs separate processes). But rclcpp
   intra-process comms is **off by default** even inside a container
   (`use_intra_process_comms_ {false}`), and Nav2 only gained an *option* to enable it in the
   Kilted→Lyrical cycle (PR #5804, "currently disabled by default"). Anything faster
   (type adaptation/NITROS, Agnocast, vendor SHM stacks) is hardware- or vendor-specific, not
   mainstream ROS 2.
3. **Slowest-common = the zero-config default, and it is exactly what `ros2 run` gives you on
   Jazzy/Kilted/Lyrical:** one process per node, `rmw_fastrtps_cpp`, Fast DDS's default transports
   (UDPv4 **+ SHM transport**, copy-based, *not* zero-copy), DataSharing **OFF** at the rmw layer,
   synchronous publish, QoS = RELIABLE / VOLATILE / KEEP_LAST(10), loans disabled. A "stock" bench
   cell claiming "this is out-of-the-box ROS 2" is citable end-to-end (links in §3).

---

## §1 Loan-take usage in the wild

### The upstream default-off decision

- **[rcl#1110: "Disable loaned messages by default"](https://github.com/ros2/rcl/pull/1110)**
  (backported to Humble and Iron): flips `ROS_DISABLE_LOANED_MESSAGES` so the
  loan path is off unless the user exports `ROS_DISABLE_LOANED_MESSAGES=0`. Stated rationale:
  loaned messages are *"currently not safe to use; see the executor code in rclcpp for more
  information."*
- **[rclcpp#2335: "Disable the loaned messages inside the executor"](https://github.com/ros2/rclcpp/pull/2335)**
  (backported to Humble and Iron): the executor no longer takes loans for
  subscription callbacks by default. Root cause discussed in-thread: Fast DDS returned the *same*
  shared-pointer memory for successive subscriber callbacks, so a user who kept a reference
  ("rightly expect[s] that the object will persist") saw it silently reused; contributors called
  the automatic loaning path *"super dangerous"* outside a narrow ConstRef-callback case.

After those fixes, **every surveyed supported distro (Humble, Iron, Jazzy, Kilted, Lyrical) ships with the
subscription-side loan path disabled by default.** A bench lane that exercises
`rcl_take_loaned_message` must export `ROS_DISABLE_LOANED_MESSAGES=0`, i.e. it is by definition a
non-default configuration.

### Who actually calls it (GitHub-wide code-search snapshot)

- `rcl_take_loaned_message` excluding ros2/rcl + ros2/rclcpp → **83 hits**, essentially all
  (a) vendored copies of `rclcpp/src/rclcpp/executor.cpp`, (b) academic executor forks
  (rtenlab PiCAS, RTIS-UCF EDF, RTeX, ROSOMP, data-flow schedulers), (c)
  [Ekumen-OS/flatros2](https://github.com/Ekumen-OS/flatros2) (an experimental flatbuffers
  transport), and (d) **this repo's own bench** (`benches/.../pong_node_rcl.cpp`). No robotics
  stack, no driver.
- `borrow_loaned_message` (the publish-side API) excluding the ros2, Fast DDS vendor and iceoryx orgs → 564 hits,
  again dominated by vendored rclcpp headers; the real non-vendored users are
  [autowarefoundation/agnocast](https://github.com/autowarefoundation/agnocast) (see below),
  D-Robotics' forked rclcpp (`publisher_hbmem.hpp`, their RDK vendor stack), a mock library
  ([rtest](https://github.com/Beam-and-Spyrosoft/rtest)), and a wrapper
  ([cactus-rt](https://github.com/cactusdynamics/cactus-rt)).
- `loaned` in **org:moveit → 0 hits**. `borrow_loaned_message` in **org:ros-navigation +
  org:ros-planning → 0 hits**.
- Who sets `ROS_DISABLE_LOANED_MESSAGES=0` (i.e. re-enables the path):
  [ekxide/rmw_iceoryx2](https://github.com/ekxide/rmw_iceoryx2)'s **benchmark justfile and
  examples** (`benchmark/justfile` lines 62/75/88; even the rmw built *for* zero-copy has to tell
  users to flip the safety default), [mvukov/rules_ros2's zero-copy
  example](https://github.com/mvukov/rules_ros2/blob/main/examples/zero_copy/zero_copy.py),
  flatros2, a Kubernetes demo (fujitatomoya/ros_k8s), and D-Robotics' RDK docs/`hobot_shm`
  (vendor board stack). Benchmarks, examples, and vendor SDKs, not applications.

### The rmw support matrix

| rmw | Loan/zero-copy support | Default state | Gating conditions |
|---|---|---|---|
| `rmw_fastrtps_cpp` (the default rmw) | Via Fast DDS **DataSharing** | **OFF**: the [rmw_fastrtps README](https://github.com/ros2/rmw_fastrtps#readme) lists its defaults outright: "Data Sharing: `OFF`" | Vendor recipe = XML with `<data_sharing><kind>AUTOMATIC</kind></data_sharing>` + `RMW_FASTRTPS_USE_QOS_FROM_XML=1` ([README "Enable Zero Copy Data Sharing"](https://github.com/ros2/rmw_fastrtps#enable-zero-copy-data-sharing)). At the DDS layer, [data-sharing](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/datasharing.html) additionally requires a **bounded**, non-keyed type, preallocated memory mode, no security, so string/sequence-bearing ROS types are ineligible even when enabled. |
| `rmw_cyclonedds_cpp` | Via iceoryx (v1) plugin | **"Shared Memory is disabled by default"** ([shared_memory_support.md](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md)) | Needs `CYCLONEDDS_URI` XML + a running **RouDi daemon**; zero-copy only for **fixed-size** types; KEEP_LAST depth ≤ 16 or it silently falls back to the network stack; ≤ 8 concurrent loans per publisher; Linux only. |
| `rmw_zenoh_cpp` | **None** | `rmw_borrow_loaned_message` returns `RMW_RET_UNSUPPORTED` ([rmw_zenoh.cpp lines 563 to 572, rolling](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/src/rmw_zenoh.cpp)) | Zenoh SHM is a *separate*, implicit optimization and is also **off by default** (`transport/shared_memory/enabled: true` required: [README](https://github.com/ros2/rmw_zenoh#readme)). |

### The strongest single piece of evidence

**Autoware, the largest open production stack with a hard zero-copy requirement, did not adopt
loaned messages.** It built [Agnocast](https://github.com/autowarefoundation/agnocast), a separate
kernel-module-based zero-copy IPC with its own publisher/subscriber API that exists precisely
because the DDS loan path can't carry Autoware's **unbounded** message types
(see [agnocast docs/shared_memory.md](https://github.com/autowarefoundation/agnocast/blob/main/docs/shared_memory.md)
and the wrapper shipped in
[autoware_core](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_agnocast_wrapper/include/autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp)).

**Verdict: benchmark-only / vendor-niche.** The loan lane is worth publishing (it closes the "you
benched ROS 2 with zero-copy off" attack) but must be labeled as a configuration **no surveyed
production stack ships**, disabled by upstream default.

---

## §2 Fastest-common: composition (+ opt-in intra-process comms)

### Composition is mainstream, and default in Nav2

- **Nav2 made composed bringup the default in Humble**:
  [Galactic→Humble migration guide, "Dynamic Composition"](https://docs.nav2.org/migration/Galactic.html):
  *"[PR 2750] provides a optional bringup based on ROS2 dynamic composition … it's used by default,
  but can be disabled by using the launch argument `use_composition:=False`."* Their published
  measurement (i7-8700, psutil): **multi-process 44% CPU / 0.76% memory vs
  `component_container_isolated` 38% CPU / 0.23% memory**; *"consumes lower memory (saves ~70%),
  and lower cpu (saves ~13%) than normal multiple processes."*
- **Autoware runs on component containers**: 49 launch files in
  [autoware_universe](https://github.com/autowarefoundation/autoware_universe) compose nodes via
  `ComposableNode` (pointcloud preprocessor, ground segmentation, euclidean cluster, ADAPI, …).
- **Drivers/perception compose too**:
  [realsense-ros](https://github.com/realsenseai/realsense-ros) documents loading the camera node
  into a shared container with `-e use_intra_process_comms:=true` for "zero-copy" image/pointcloud
  delivery; [image_pipeline was ported to the composition model](https://github.com/ros-perception/image_pipeline/issues/403).

### But intra-process comms is opt-in, even when composed

- rclcpp default: [`node_options.hpp` line 456 (rolling)](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/node_options.hpp),
  `bool use_intra_process_comms_ {false};`. Composing nodes into one container does **not** by
  itself enable pointer-passing; without it, same-process pub/sub still round-trips the rmw.
- **Nav2 only gained the option in the Kilted→Lyrical cycle**:
  [Nav2 Kilted migration guide](https://docs.nav2.org/migration/Kilted.html): *"In PR 5804, an
  option to enable Intra-process Communication in Nav2 has been added … It is currently disabled
  by default."*
- Semantics when enabled ([design.ros2.org intra-process article](https://design.ros2.org/articles/intraprocess_communications.html)):
  publishing a `unique_ptr` reaches subscriptions with **0 copies only if none of them take
  ownership** (`shared_ptr` callbacks); *"If all the Subscriptions want ownership … a total of N-1
  copies of the message are required."* I.e. even the ceiling configuration has copy cliffs on
  fan-out, and it only covers same-process edges; cross-process edges drop back to the rmw.

### Is anything faster still "mainstream"?

Only in vendor silos: **type adaptation ([REP 2007](https://ros.org/reps/rep-2007.html)) + a GPU vendor's
ROS 2 type adaptation (NITROS)** (GPU-resident
pipelines on Jetson), **Agnocast** (Autoware), D-Robotics `hobot_shm` (RDK boards). Each is
hardware- or stack-specific; none is a portable ROS 2 configuration. The community-contributed
`EventsExecutor` reduces executor overhead but changes no data path. So **composition +
`use_intra_process_comms:=true` is the representative "fastest-common" ROS 2 lane.**

---

## §3 Slowest-common: the zero-config default (what `ros2 run` actually uses on Jazzy)

Every claim below is the out-of-the-box behavior with **zero configuration**:

| Axis | Stock behavior | Citation |
|---|---|---|
| Process model | One OS process per `ros2 run` node; no composition, no IPC | (definitionally; Nav2's default composition in §2 is the launch-file exception, not the `ros2 run` rule) |
| Default RMW | `rmw_fastrtps_cpp` on Jazzy, Kilted, **and Lyrical** | [Jazzy middleware vendors doc](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html); ["ROS 2 Lyrical Luth and 11 Years of Fast DDS as ROS 2 Default Middleware" (discourse)](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-and-11-years-of-fast-dds-as-ros-2-default-middleware/55062) |
| Intra-host transport | Fast DDS creates **UDPv4 + SHM transports by default**: *"SHM: … This transport is created by default on a new DomainParticipant if no specific transport configuration is given"* (same sentence for UDPv4). So stock intra-host traffic rides the SHM **transport**, which is copy-based: *"with Shared Memory Transport the data being transmitted must be copied from the DataWriter history to the transport and from the transport to the DataReader"* | [Fast DDS Transport Layer](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/transport.html); copy quote from [Data-sharing delivery](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/datasharing.html) |
| Zero-copy (DataSharing) | **OFF** at the rmw layer; rmw_fastrtps README defaults: "Data Sharing: `OFF`" (Fast DDS's own `AUTO` default never applies through ROS 2 without the XML + env recipe) | [rmw_fastrtps README](https://github.com/ros2/rmw_fastrtps#readme) |
| Publish mode | `SYNCHRONOUS_PUBLISH_MODE`, `PREALLOCATED_WITH_REALLOC` history (rmw overrides) | [rmw_fastrtps README](https://github.com/ros2/rmw_fastrtps#readme), [Fast DDS "Configuring Fast DDS in ROS 2"](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2_configure.html) |
| QoS | `rmw_qos_profile_default` = **KEEP_LAST(10), RELIABLE, VOLATILE** | [ros2/rmw `qos_profiles.h` lines 51 to 62](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h) |
| Subscription receive | Standard rclcpp callback with per-message deserialize/copy; loan dispatch disabled (§1) | rcl#1110 / rclcpp#2335 |
| If the user swaps to cyclonedds | UDP loopback intra-host; SHM needs a custom iceoryx build + XML + RouDi daemon | [rmw_cyclonedds shared_memory_support.md](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md) |
| If the user swaps to rmw_zenoh | A **router daemon is required** (*"Without the Zenoh router, nodes will not be able to discover each other"*), TCP links by default, SHM off by default, loans unsupported | [rmw_zenoh README](https://github.com/ros2/rmw_zenoh#readme) |

Two bench-fairness notes that fall out of this table:

- **The stock cell on jazzy IS the fastdds+SHM-transport cell.** Benching Fast DDS "with SHM on"
  is not a favor to ROS 2; it's literally the default. The `no_shm` (UDP) cell is the
  *pessimized* variant and should be labeled as such (the suite already does this).
- **Stock QoS is RELIABLE/KEEP_LAST(10)**; the suite's `be1` default (BEST_EFFORT/KL1) is
  *friendlier to ROS 2 than its own out-of-the-box QoS* (chosen for SHM/zero-copy path
  eligibility). Worth one sentence in the docs: our default lane hands ROS 2 a better-than-stock
  QoS; the `rel10` cells cover stock.

---

## §4 What other public benchmarks use as ROS 2 lanes

- **The ROS 2 `performance_test` harness**: the de-facto
  industry harness. Two lane families: (a) **native middleware plugins** (Fast DDS, Cyclone DDS +
  C++ binding, iceoryx and several DDS vendors) where *"there is no rclcpp or
  rmw layer overhead"*; (b) a **ROS 2 framework plugin** running real
  `rclcpp::publisher/subscriber` under three executor variants (single-threaded,
  static-single-threaded, WaitSet). Zero-copy is an explicit opt-in flag, `--zero-copy`
  (LoanedSamples), alongside `--shared-memory`; Cyclone's zero-copy lane requires RouDi. Per-lane
  transport tables (INTRA / SHMEM / LoanedSamples / UDP) are printed per plugin. Precedent: they
  publish default lanes AND opt-in zero-copy lanes, clearly labeled.
- **The `ros2-performance` harness**: latency /
  reliability / CPU / memory over synthetic node graphs; *"mostly meant for evaluating single
  process applications"*, i.e. its ROS 2 lane is the composed/single-process shape, and it
  measures pure communication overhead (nodes do no work).
- **The Fast DDS vendor's published results**: its "independent ROS 2 benchmark" news page (a
  university study) and its Fast DDS performance testing page
  (vs Cyclone/OpenDDS, latency + throughput, using their own perf harness). The official
  [OSRF TSC-RMW-Reports](https://osrf.github.io/TSC-RMW-Reports/humble/) (the Humble default-rmw
  selection) are the canonical "what lineup did the ROS TSC itself use" citation.

**Coverage:** the suite publishes stock defaults, the vendor-recipe SHM/zero-copy lane, the
loan lane and the composed lane, which together cover every configuration the published ROS 2
benchmark harnesses above report, each under its own label.

---

## §5 Comparison pairings

Cerulion shapes named per `benches/latency/README.md` (this memo covers pairings + labels only;
the bench code is documented there).

| ROS 2 lane | Exact ROS 2 config (citable) | Cerulion shape | What the pairing compares |
|---|---|---|---|
| **Stock** ("this is what `ros2 run` gives you") | Jazzy/Lyrical, `rmw_fastrtps_cpp`, default transports (SHM transport active), DataSharing OFF, sync publish, rclcpp callback recv, 3 processes. Stock QoS = RELIABLE/KL10 (`rel10` cells); `be1` cells are *friendlier than stock*. | `cerulion_workspace_split` (zero flags, zero env), the pairing the README chart draws | Default vs default. Both are the unconfigured multi-process experience: ROS 2 pays its stock serialize+SHM-copy path, Cerulion pays its full runtime (barrier lockstep + gateway spawn). Neither side is tuned. |
| **Tuned-SHM** (vendor's own best recipe) | fastdds `zc` lane: DataSharing XML (`<kind>AUTOMATIC</kind>`) + `RMW_FASTRTPS_USE_QOS_FROM_XML=1`, verbatim the [rmw_fastrtps README recipe](https://github.com/ros2/rmw_fastrtps#enable-zero-copy-data-sharing). (Cyclone's equivalent needs RouDi + custom build; reasonable to skip with the citation stating why.) | Still `cerulion_workspace_split` (zero flags) | Their documented best against our untuned default: ROS 2 runs its vendor-recommended zero-copy recipe while Cerulion stays flagless. The stock and zero-copy Fast DDS results are both published, each under its own label. |
| **Composed** (fastest-common) | `component_container` + `use_intra_process_comms:=true`, the Nav2-default-composition shape (Humble+, PR #2750) with the IPC opt-in Nav2 itself only offers as of Lyrical (PR #5804, off by default) | `mono` (`--single-process`) | Both collapse to one process. Label the asymmetry: ROS 2 intra-process bypasses the rmw entirely (framework pointer-pass with N-1-copy fan-out cliffs), so this row compares *framework* overhead, not transport. The suite measures this lane as `{distro}_composed_ipc{on,off}_rclcpp_chrt{N}`; `benches/latency/README.md` records it as the matched twin of `mono`, with the bypass asymmetry named. |
| **Loan** (benchmark-only) | `recv=loan` lane: `rcl_take_loaned_message` + `ROS_DISABLE_LOANED_MESSAGES=0` (+ fastdds DataSharing profile) | `cerulion_workspace_split`, with `iox2_chrt0` printed as the shared floor reference | Published with the §1 caption: **disabled by upstream default (rclcpp#2335/rcl#1110) and shipped by no surveyed production stack**; Autoware built Agnocast instead, MoveIt/Nav2 have zero call sites. It answers "what could ROS 2 do at its theoretical best," not "what ROS 2 users get." zenoh × loan is structurally SKIP (`RMW_RET_UNSUPPORTED`, cited above). |

Each row pairs a Cerulion shape with the ROS 2 configuration of the same process shape, and
every ROS 2 configuration in the table is published under its own label, so a stock result and a
tuned result are never quoted as one number. The upstream default and the adoption evidence
behind the loan lane are in §1 and the source index below.

---

## Source index

Loan decision & usage: [rclcpp#2335](https://github.com/ros2/rclcpp/pull/2335) ·
[rcl#1110](https://github.com/ros2/rcl/pull/1110) ·
[agnocast](https://github.com/autowarefoundation/agnocast) ·
[autoware_agnocast_wrapper](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_agnocast_wrapper/include/autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp) ·
[rmw_iceoryx2 benchmark justfile](https://github.com/ekxide/rmw_iceoryx2/blob/main/benchmark/justfile) ·
[rmw_zenoh rmw_zenoh.cpp](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/src/rmw_zenoh.cpp)
Composition & IPC: [Nav2 Galactic→Humble migration](https://docs.nav2.org/migration/Galactic.html) ·
[Nav2 Kilted migration](https://docs.nav2.org/migration/Kilted.html) ·
[nav2 PR #2750](https://github.com/ros-navigation/navigation2/pull/2750) ·
[nav2 PR #5804](https://github.com/ros-navigation/navigation2/pull/5804) ·
[design.ros2.org intra-process](https://design.ros2.org/articles/intraprocess_communications.html) ·
[rclcpp node_options.hpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/node_options.hpp) ·
[realsense-ros](https://github.com/realsenseai/realsense-ros) ·
[image_pipeline #403](https://github.com/ros-perception/image_pipeline/issues/403) ·
[REP 2007](https://ros.org/reps/rep-2007.html) ·
NITROS (GPU vendor type adaptation)
Stock defaults: [Jazzy middleware vendors](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html) ·
[Lyrical + 11 years of Fast DDS default (discourse)](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-and-11-years-of-fast-dds-as-ros-2-default-middleware/55062) ·
[Fast DDS transport layer](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/transport.html) ·
[Fast DDS data-sharing](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/datasharing.html) ·
[rmw_fastrtps README](https://github.com/ros2/rmw_fastrtps#readme) ·
[rmw qos_profiles.h](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h) ·
[rmw_cyclonedds shared_memory_support.md](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md) ·
[rmw_zenoh README](https://github.com/ros2/rmw_zenoh#readme)
Reference benches: the ROS 2 `performance_test` harness · the `ros2-performance` harness · the Fast DDS vendor's published benchmark pages ·
[OSRF TSC-RMW-Reports](https://osrf.github.io/TSC-RMW-Reports/humble/)
