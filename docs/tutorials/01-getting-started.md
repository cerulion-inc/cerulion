# Getting Started with Cerulion

Build a complete obstacle-avoidance safety controller from scratch with the
`cerulion` CLI: create a workspace, write three nodes, wire them in a graph file,
run it, record it, and verify a code change against the recording.

**What you'll build:**

```
lidar_sensor  ──scan──>  safety_controller  ──cmd_vel──>  drive_base
  (10 Hz)                 (data-triggered)                (data-triggered)
```

Three nodes forming a reactive pipeline:

| Node | Role | Input | Output |
|------|------|-------|--------|
| `lidar_sensor` | Publishes simulated laser scan readings at 10 Hz | none | `scan` (`sensor_msgs/LaserScan`) |
| `safety_controller` | Stops the robot when an obstacle is too close | `scan` | `cmd_vel` (`geometry_msgs/Vector3`) |
| `drive_base` | Receives velocity commands and logs them | `cmd_vel` | none |

**Prerequisites:**
- The `cerulion` CLI, installed as the [README](../../README.md#install) describes
  (installer script, Homebrew, apt, or `cargo install`):

  ```bash
  curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh
  ```

  The installer puts the programs on the PATH of every new shell, so open a new
  terminal before the steps below.

- A C linker. Cargo links every node through the system compiler, so a machine
  without one installs cleanly and then fails at the first build. On Ubuntu and
  Debian install it with `sudo apt-get install -y build-essential git`; on macOS
  with `xcode-select --install`. The installer above says so too when it finds
  no C linker on the machine.

- A Cerulion account. Every `cerulion` command runs under one, and signing in is a
  once-per-machine step that later commands read locally, so they keep working
  offline:

  ```bash
  cerulion login
  ```

  A robot or a headless box prints a short code to approve from a browser on any
  machine.

- The Rust compiler that built your `cerulion` binary. A node is a shared library
  loaded into the CLI's own process, so a node built by a different rustc release
  is refused at load, loudly, even when both meet the `1.93` minimum.

  The install script above provisions that compiler. On Homebrew and apt, run
  `cerulion-install-rust` once and put `${CARGO_HOME:-$HOME/.cargo}/bin` on your
  PATH. The workspace you create in Step 1 records the compiler in its own
  `rust-toolchain.toml` when it is installed, so the builds below select it with no
  environment variable. When your rustup default is a different compiler, name it:
  `RUSTUP_TOOLCHAIN=1.93.0 cerulion node build <node>`.

Every step below is a `cerulion` verb. You never run `cargo` yourself and you never
write a `main`: a node is a library the runtime loads, and the graph file is the
only place wiring lives.

---

## Step 1: Create a Workspace

A **workspace** is the top-level directory that holds your nodes, graphs, and
schemas. It contains a `Cargo.toml` with `[workspace]`, plus `graphs/`,
`nodes/`, and `schemas/` directories. Run this wherever you keep projects; it
does not need to be inside a Cerulion checkout.

```bash
cerulion workspace create safety_demo
cd safety_demo
```

**Expected output:**

```
Created workspace at /path/to/safety_demo
  dependencies: published crates from crates.io, pinned to =X.Y.Z (the installed CLI version)
```

The second line says where the node crates will get the framework from; its exact
wording depends on how the CLI was installed (see **Dependency selection** below).

> **Note:** One-shot CLI commands print only their output by default; operational
> `INFO` log lines are suppressed. Pass `-v` (or set `RUST_LOG=info`) to see the
> structured `tracing` breadcrumbs, e.g. `INFO workspace created ...`. Long-running
> commands like `cerulion graph run` keep their `INFO` lifecycle lines by default.

**What was generated:**

```
safety_demo/
  Cargo.toml              # [workspace] members = ["nodes/*"]
  .cargo/
    config.toml           # IOX2_LOG_LEVEL=error, RUST_LOG=warn
  .gitignore              # ignores .cerulion/ (the CLI's per-workspace state)
  graphs/                 # Empty: we add a graph in Step 4
  nodes/                  # Empty: we add nodes next
  schemas/                # Empty: we use built-in ROS 2 types
```

`recordings/` is not part of the scaffold. It appears the first time you run a
graph (Step 5), and it is where the recording lands.

> **How workspace discovery works:** When you run any `cerulion` command inside
> `safety_demo/` (or any subdirectory), the CLI walks upward until it finds a
> `Cargo.toml` containing `[workspace]` alongside a `graphs/` directory. This
> means you can run commands from `safety_demo/nodes/lidar_sensor/` and they'll
> still find the workspace root.

**Dependency selection.** The choice keys on where the `cerulion` binary lives,
not on your current directory. An installed CLI writes the published crates
pinned to its own version; a CLI built inside a `cerulion` source checkout
writes absolute paths into that checkout. To change it, edit `cerulion_core` and
`native_ros2_messages` under `[workspace.dependencies]` in the generated root
`Cargo.toml`; node manifests inherit those entries with `{ workspace = true }`.

---

## Step 2: Create the Node Types

Each node type is a separate Rust crate compiled as a **cdylib** (C dynamic
library). The graph runtime loads these at startup via `dlopen`.

### 2a. Create `lidar_sensor`

```bash
cerulion node create lidar_sensor --policy period_ms=100 -o sensor_msgs/LaserScan scan
```

This creates the node type **and** declares its output port in one command.
The `--policy period_ms=100` is required here: `lidar_sensor` has no inputs,
so there is no data arrival to trigger it, and a source-only node must declare a
periodic (or external) trigger policy up front. It also matches the
`period_ms = 100` the Step 3a code declares on the macro.

**Expected output:**

```
Created node type 'lidar_sensor'
```

### 2b. Create `safety_controller`

```bash
cerulion node create safety_controller \
  -T sensor_msgs/LaserScan scan \
  -o geometry_msgs/Vector3 cmd_vel
```

`-T` declares a TRIGGER input: the field becomes `#[input(trigger)]` and the
node's policy becomes `data_trigger=scan`. A plain `-i` declares the port
without a trigger. That is what you want for a second, non-triggering input, but
a node whose ONLY ports came from `-i` has no trigger at all, and the macro
refuses to build it (`no trigger policy: add #[input(trigger)] to a field, or
specify period_ms or external on the node`). Every node needs exactly one way
to fire.

**Expected output:**

```
Created node type 'safety_controller'
```

### 2c. Create `drive_base`

```bash
cerulion node create drive_base -T geometry_msgs/Vector3 cmd_vel
```

**Expected output:**

```
Created node type 'drive_base'
```

### Verify your nodes

```bash
cerulion node list
```

**Expected output:**

```
TYPE                 INPUTS     OUTPUTS    POLICY
drive_base           1          0          trigger:cmd_vel
lidar_sensor         0          1          period 100ms
safety_controller    1          1          trigger:scan
```

### Inspect a node's ports and policy

Use `cerulion node info` to see a node type's trigger policy and port layout:

```bash
cerulion node info lidar_sensor
```

**Expected output:**

```
Node type: lidar_sensor
Policy: period 100ms
Outputs:
  scan sensor_msgs/LaserScan
```

### Inspect a schema

`cerulion schema info` prints a message's wire layout as one recursive tree:
every non-primitive field expands inline beneath it, and each line carries its
fixed or variable class.

```bash
cerulion schema info sensor_msgs/LaserScan
```

**Expected output:**

```
schema: sensor_msgs/LaserScan
source: built-in (ROS 2)
wire fixed size: 28 bytes
hash: 0x64cc8631dc24946b
fields: 10
  header: std_msgs/Header (variable)
    stamp: builtin_interfaces/Time (fixed)
      sec: int32 (fixed)
      nanosec: uint32 (fixed)
    frame_id: string (variable)
  angle_min: float32 (fixed)
  angle_max: float32 (fixed)
  angle_increment: float32 (fixed)
  time_increment: float32 (fixed)
  scan_time: float32 (fixed)
  range_min: float32 (fixed)
  range_max: float32 (fixed)
  ranges: float32[] (variable)
  intensities: float32[] (variable)
```

It resolves workspace schemas (`schemas/*.yaml`, by file stem or schema name),
built-in ROS 2 types (`pkg/Type` or `pkg::Type`), and a robot's custom types
over the network; the `source:` line names which. A workspace schema that reuses
a built-in name wins, with a loud shadow warning on stderr. `cerulion schema
list` shows everything, grouped by package.

`LaserScan` is the type this tutorial uses because it carries variable-length
arrays, the shape real LIDAR pipelines have.

**Port metadata lives in the node's `src/lib.rs`, not in a sidecar file.** `-o`,
`-i` and `-T` write the port as a struct field with the matching attribute, and
`node info`, `node list` and `node stage` read it back by parsing that file. The
built node reports the same metadata to the runtime, because the macro bakes it
in at build time.

---

## Step 3: Write the Node Logic

The `node create` command generated a **template** `lib.rs` for each node: a
`#[cerulion_node]` struct with the ports you declared as fields, and a `tick()`
with a `TODO` in it. Now we replace each template's body with real logic. The
shape stays the same: one struct, one `tick()`. That macro form is the only way
to write a node.

### 3a. `lidar_sensor` -- the source node

Replace `nodes/lidar_sensor/src/lib.rs` with:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::LaserScan;

#[cerulion_node(period_ms = 100)]                     // 10 Hz publisher
#[derive(Default)]
struct LidarSensorNode {
    /// Output port: simulated scan readings published every tick.
    /// Bare `#[output]` is all you need: the macro resolves fixed vs
    /// variable fields at compile time, so `self.scan.<field> = ...`
    /// Just Works for every field.
    #[output]
    scan: LaserScan,

    /// Sequence counter -- drives the simulated scan pattern.
    /// (Non-port fields are ordinary user state.)
    seq: u32,
}

#[cerulion_node_impl]
impl LidarSensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seq += 1;

        // Direct fixed-field writes via Deref<Target = LaserScanFixedSection>.
        // Each assignment lands in the loaned SHM slot.
        self.scan.angle_min = -1.57;        // -90 degrees
        self.scan.angle_max = 1.57;         //  90 degrees
        self.scan.angle_increment = 0.01;   // ~314 rays
        self.scan.range_min = 0.05;
        self.scan.range_max = 10.0;

        // Simulate an obstacle that oscillates between 0.3m and 5.0m
        // in the center of the scan. Every ~63 ticks the obstacle
        // enters the danger zone (< 0.5m).
        let angle = (self.seq as f32) * 0.1;
        let center_distance = 2.65 + 2.35 * angle.sin(); // range: 0.3 .. 5.0

        // Variable-length field, written in place. `loan_ranges(n)` hands
        // back `&mut [f32]` of exactly `n` elements INSIDE the loaned
        // shared-memory frame, so the sweep is built where it will be
        // published: no per-tick `Vec`, and no copy on the way out.
        const NUM_RAYS: usize = 314;
        let ranges = self.scan.loan_ranges(NUM_RAYS)?;
        ranges.fill(5.0);
        // Place the obstacle in the center rays.
        ranges[150..164].fill(center_distance);
        // EVERY variable-length field of an output schema must be written
        // each tick: a frame with an unwritten variable field is discarded.
        // `intensities` stays empty for this simulated scan, and an empty
        // slice is a write that allocates nothing. The nested
        // `header` is a std_msgs/Header: write its fields by leaf assignment
        // like everything else: nested paths of any depth are rewritten the
        // same way. Header's one variable field is `frame_id`; writing it
        // satisfies the every-variable-field rule for `header` (the `stamp`
        // timestamp stays zeroed, because fixed leaves default).
        self.scan.intensities = &[][..];
        self.scan.header.frame_id = "laser";

        Ok(())
    }
}
```

**Key points:**
- `#[cerulion_node(period_ms = 100)]` declares the node type and its trigger
  policy: fire every 100 ms. The folder name (`nodes/lidar_sensor/`) is the node
  type identifier the graph YAML's `type:` field names.
- Ports are struct fields carrying `#[input]` / `#[output]`. The field's type
  (here `LaserScan`) selects the schema.
- **Write any output field by plain assignment**, at any depth:
  `self.scan.angle_min = -1.57`, `self.scan.header.frame_id = "laser"`. The
  macro rewrites every such assignment into a fallible shared-memory write, so a
  helper method that writes port fields must return `Result`.
- **A variable-length field can be written in place.** `self.scan.loan_ranges(n)?`
  returns `&mut [f32]` over `n` elements of the loaned frame; fill that slice and
  the frame is complete. From a device or a decoder,
  `self.scan.ranges.fill_from(producer)?` hands the producer the frame's own
  buffer. Assignment copies instead, which is right when you already hold the
  values. Every generated schema carries `loan_<field>`, `fill_from_<field>` and
  `set_<field>` for each variable-length field.
- **Every variable-length field must be written on every tick**, or the frame is
  discarded with an error log. `LaserScan` has three: `ranges`, `intensities`
  and the nested `header`, which is why this node writes all three even with no
  data for two of them. The rule reaches into nested schemas: `header`'s own
  variable field `frame_id` must be written too.
- `#[derive(Default)]` supplies the node's initial state, and
  `#[cerulion_node_impl]` enables the rewriter. Inside it, `tick` is
  `fn tick(&mut self) -> Result<(), NodeError>`.

### 3b. `safety_controller` -- the transform node

Replace `nodes/safety_controller/src/lib.rs` with:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

/// Distance threshold in meters. Below this, the robot stops.
const SAFE_DISTANCE: f32 = 0.5;

/// Cruising speed in m/s when no obstacle is detected.
const CRUISE_SPEED: f64 = 0.5;

#[cerulion_node]
#[derive(Default)]
struct SafetyControllerNode {
    /// Trigger input: fires `tick` when a new scan arrives on the
    /// graph-wired source topic. The `#[input(trigger)]` attribute
    /// declares this as the trigger; the `source:` for `scan` in the
    /// graph YAML (Step 4) decides which upstream topic feeds it.
    #[input(trigger)]
    scan: LaserScan,

    /// Output port: linear velocity command (one component of a full
    /// Twist). `Vector3` is a fixed-only schema so a bare `#[output]`
    /// is enough: fixed primitives `x/y/z: f64` are reachable via
    /// `Deref<Target = Vector3Shm>`.
    #[output]
    cmd_vel: Vector3,

    /// Closest valid range from the last scan (meters), or infinity when
    /// that scan carried no valid reading.
    closest_range: f32,
    /// Count of emergency stops triggered.
    stop_count: u32,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Direct read from the SHM-backed view: `self.scan.ranges()`
        // returns `&[f32]` over the loaned variable field, with no copy.
        // A beam is a measurement only when it is finite and strictly
        // positive: ROS LaserScan publishes NaN for an invalid beam, and a
        // zero or negative reading is not a distance. Everything else is
        // filtered out before the minimum is taken, so no invalid beam can
        // stand in for a real one.
        let closest_valid = self
            .scan
            .ranges()
            .iter()
            .copied()
            .filter(|r| r.is_finite() && *r > 0.0)
            .fold(f32::INFINITY, f32::min);
        self.closest_range = closest_valid;
        tracing::debug!(closest_valid, "closest valid reading");

        // Decide: cruise or stop. Cruising requires evidence -- one valid
        // reading at or beyond the safe distance. A scan that is empty, all
        // NaN, or all zeros leaves `closest_valid` at infinity and stops the
        // robot, because no valid reading is not the same as a clear path.
        let speed = if closest_valid.is_finite() && closest_valid >= SAFE_DISTANCE {
            CRUISE_SPEED
        } else {
            self.stop_count += 1;
            tracing::info!(
                // `inf` here means the scan carried no valid reading at all.
                range = self.closest_range,
                stop_count = self.stop_count,
                "emergency stop"
            );
            0.0 // Emergency stop
        };

        // Direct fixed-field writes via Deref<Target = Vector3Shm>:
        // x/y/z are `pub f64` fields on the SHM overlay, so each
        // assignment lands straight in the loaned SHM slot.
        self.cmd_vel.x = speed;
        // y / z stay zero -- drive straight

        Ok(())
    }
}
```

**Key points:**
- `#[input(trigger)] scan: LaserScan` makes `scan` both the data port and the
  trigger, so `tick()` fires when a scan arrives and never polls. The graph YAML
  supplies only the `source:` that feeds the port.
- `self.scan.ranges()` is the typed read accessor: a borrowed `&[f32]` directly
  over the loaned shared-memory payload. No deserialization, no copy, no
  allocation.
- **The controller cruises only on evidence.** A beam counts as a measurement
  only when it is finite and strictly positive, and a scan with no such beam
  (empty, all NaN, all zeros) stops the robot instead of reading as a clear
  path. Write every safety condition this way round: state what must be true to
  keep moving, rather than what must be true to stop.
- The output is a `Vector3` rather than a full `geometry_msgs/Twist` only to keep
  the example small. A `Twist` works the same way:
  `self.cmd_vel.linear.x = speed`.

### 3c. `drive_base` -- the sink node

Replace `nodes/drive_base/src/lib.rs` with:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
#[derive(Default)]
struct DriveBaseNode {
    /// Trigger input: fires `tick` when the safety controller publishes.
    #[input(trigger)]
    cmd_vel: Vector3,

    /// Total velocity commands received.
    msg_count: u64,
}

#[cerulion_node_impl]
impl DriveBaseNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.msg_count += 1;
        // Read the fixed fields into locals FIRST: the `#[cerulion_node_impl]`
        // rewriter lowers `self.cmd_vel.<field>` port accesses in ordinary
        // expressions, but cannot see inside another macro's body (like
        // `tracing::debug!`), so port reads must happen outside it.
        let linear_x = self.cmd_vel.x;
        let linear_y = self.cmd_vel.y;
        let linear_z = self.cmd_vel.z;
        tracing::debug!(
            linear_x,
            linear_y,
            linear_z,
            msg_count = self.msg_count,
            "drive_base received velocity command"
        );
        Ok(())
    }
}
```

**Key points:**
- A **sink node** has no `#[output]` fields: it consumes and never publishes.
  Fixed-field reads (`self.cmd_vel.x`) go through the shared-memory overlay at
  zero cost.
- **Hoist port reads out of macro invocations.** The rewriter lowers
  `self.<port>.<field>` in ordinary expressions but cannot see inside another
  macro's body, so `self.cmd_vel.x` written directly inside
  `tracing::debug!(...)` fails to compile (E0609). Read the port into a local
  first, as the code above does.
- A node logs through `tracing`, never `println!`. A `--release` build compiles
  `debug` and `trace` out, so to see these lines build this node and run the
  graph without `--release` and set `RUST_LOG=debug`. The `safety_controller`'s
  `emergency stop` line is `info`, so it always shows.

---

## Step 4: Create the Graph

A **graph** is a YAML file that defines which node instances run and how
they're wired together: topology only. (Trigger policy lives on each node
type's macro attribute, written in Step 3.) The file's name is the graph's name.

```bash
cerulion graph create obstacle_avoidance -n demo
```

**Expected output:**

```
Created graph 'obstacle_avoidance'
```

This creates `graphs/obstacle_avoidance.yaml` with an empty node list and the
prefix `demo`.

### Stage nodes into the graph

Staging adds a node instance to the graph and wires its ports:

```bash
# Source node: 10 Hz periodic trigger
cerulion node stage lidar_sensor -g obstacle_avoidance

# Transform node: fires when lidar_sensor publishes
cerulion node stage safety_controller -g obstacle_avoidance \
  -I scan lidar_sensor/scan

# Sink node: fires when safety_controller publishes
cerulion node stage drive_base -g obstacle_avoidance \
  -I cmd_vel safety_controller/cmd_vel
```

Each instance gets its node type's name as its id (`-i ID` picks another, which
is how one type is staged twice). `-I NAME SOURCE` wires the input `NAME` to an
upstream `<node_id>/<port>`.

**Expected output (one per command):**

```
Staged 'lidar_sensor' into graph 'obstacle_avoidance'
Staged 'safety_controller' into graph 'obstacle_avoidance'
Staged 'drive_base' into graph 'obstacle_avoidance'
```

Each is preceded by a `WARN overwriting an existing graph file` line that names a
`.bak` path. That is expected: the verb rewrites the graph file and keeps the
previous version beside it as `graphs/obstacle_avoidance.yaml.bak`.

### Read the graph YAML

Open `graphs/obstacle_avoidance.yaml`. The three `stage` commands wrote all of it,
and it is complete as it stands:

```yaml
prefix: demo
nodes:
  - id: lidar_sensor
    type: lidar_sensor
    outputs:
      - name: scan
        schema: sensor_msgs/LaserScan
  - id: safety_controller
    type: safety_controller
    inputs:
      - name: scan
        source: lidar_sensor/scan
    outputs:
      - name: cmd_vel
        schema: geometry_msgs/Vector3
  - id: drive_base
    type: drive_base
    inputs:
      - name: cmd_vel
        source: safety_controller/cmd_vel
```

It is a topology-only file: `id`, `type`, and the port wiring (`inputs.source`,
`outputs.schema`). **Trigger policy is declared on the macro side**, in the
`#[cerulion_node(...)]` attribute or the `#[input(trigger)]` marker on each node's
struct (written in Step 3); the graph YAML carries no `policy:` block. This file
is yours to hand-edit from here on: add comments, rewire a `source:`, or set a
per-output knob. `docs/user-api.md` ("Graph YAML reference") lists every key.

**No buffer sizes to set.** `LaserScan` carries variable-length arrays and its
shared-memory budget comes from the schema itself, so the scan publishes without
any `max_slice_len:` here. The key exists for the day you need to override it for
one output.

### Trigger policies

Every policy is declared on the macro side; the graph YAML carries topology only.

| Policy | Macro attribute | Behavior |
|--------|-----------------|----------|
| **Period** | `#[cerulion_node(period_ms = N)]` | Fire every N ms regardless of data |
| **Data** | `#[input(trigger)]` on a field | Fire when that input port has new data |
| **Sync (bounded)** | `#[cerulion_node(sync_window_ms = N)]` + `#[input(trigger)]` on two or more ports | Fire once per complete aligned set: one message from each trigger port, all within an N-ms window of each other. Each message joins at most one set, so a burst holding three complete sets fires three times. |
| **Sync (unbounded)** | `#[cerulion_node(unbounded_sync)]` + `#[input(trigger)]` on two or more ports | Fire once per complete set, as soon as every trigger port has an unconsumed message. No timing bound, so worst-case fire latency is the slowest publisher's inter-arrival interval, unbounded if it stops: not for control loops. |
| **Data + deadline watchdog** | `#[input(trigger, expect_within_ms = N)]` | Fire on data arrival, and record a miss if no fresh data lands within N ms. The timeout records the miss; it does not fire the node. |
| **External** | `#[cerulion_node(external)]` + an `external_source()` method | A driver node: it watches something outside Cerulion (a device file descriptor, a blocking SDK call) and fires itself when that signals. See "External nodes" in `docs/user-api.md`. |

The data trigger is the most common for reactive pipelines: each node fires
exactly when its upstream publishes, with minimal latency and no polling.

---

## Step 5: Build and Run

### Build all nodes

```bash
cerulion node build lidar_sensor --release
cerulion node build safety_controller --release
cerulion node build drive_base --release
```

**Expected output (per node):**

```
Building 'lidar_sensor'. The first build of a workspace also compiles the Cerulion runtime and can take a few minutes; nothing more is printed until the build finishes.
Built 'lidar_sensor'
```

The first build also compiles the Cerulion runtime, so it takes a few minutes on
a laptop; later builds take seconds.

`cerulion node build` runs your `cargo` on the node crate and produces
`target/release/lib<node_type>.dylib` on macOS or `.so` on Linux
(`target/debug/` without `--release`); `graph run` loads the libraries from
there. The runtime loads nodes as dynamic libraries, which is why each generated
crate is a `cdylib` with the `cdylib` feature on by default: that feature is what
makes the macro emit the entry points the runtime calls. Use `--release` for
anything you will measure or record; a debug build is far slower.

### Check the graph

```bash
cerulion graph validate obstacle_avoidance
```

Every check should read `[ok]`, ending in an `N/N checks passed.` line. Run
before the builds, the same command fails with three `cdylib ... not found`
lines that tell you to run `cerulion node build` first: the verb names the next
verb.

### Run the graph, with recording on

```bash
cerulion graph run obstacle_avoidance --release --record
```

This runs the graph **live**: an event-driven loop that wakes within microseconds
of each publish. `--record` also writes the run to a bag under `recordings/`,
which Step 7 uses.

**Multi-process by default:** the graph declares no `process_groups:` block, so
on Linux and macOS `graph run` first derives a **process-per-node partition**
(one OS process per node, for maximal fault isolation) and shows it for
confirmation:

```
graph partition: obstacle_avoidance
mode: process-per-node baseline (no cost artifact at .../graphs/obstacle_avoidance.costs.yaml ...)
nodes: 3  groups: 3
...
+process_groups:
+  grp_lidar_sensor: [lidar_sensor]
+  grp_safety_controller: [safety_controller]
+  grp_drive_base: [drive_base]
...
Apply this partition to the graph file? [y/N]
```

Press **Enter** to use that layout for this run and leave the file untouched, or
**y** to write the partition into `graphs/obstacle_avoidance.yaml` (a `.bak`
backup is kept), after which later runs skip the prompt. Prefer everything in one
process? Opt out with `--single-process` (no derivation, no prompt). Scripting
it? `--yes` writes without asking, and a run with no terminal attached never
writes at all.

Graph validation runs automatically before execution. If a check fails (a missing
source, an unbuilt node, a schema that does not resolve), you get the report and
the graph does not start.

**Expected output (after the prompt):** the supervisor spawns one worker process
per group, waits for each worker's READY, starts the recorder, then releases them
together. Representative lines (timestamps, pids and paths vary; long lines are
cut at `...`):

```
INFO graph validation complete graph=obstacle_avoidance
INFO spawned worker process group=grp_lidar_sensor pid=... nodes=1
INFO worker READY observed group=grp_lidar_sensor ready=...
INFO spawned worker process group=grp_safety_controller pid=... nodes=1
INFO worker READY observed group=grp_safety_controller ready=...
INFO spawned worker process group=grp_drive_base pid=... nodes=1
INFO worker READY observed group=grp_drive_base ready=...
INFO spawned bagd recorder graph=obstacle_avoidance pid=... bag=recordings/obstacle_avoidance_<timestamp>.mcap
INFO all workers spawned + READY ...
INFO safety_controller: emergency stop range=0.4970099925994873 stop_count=1
INFO safety_controller: emergency stop range=0.41373515129089355 stop_count=2
```

The graph is now running as three lockstepped processes. The `lidar_sensor`
publishes at 10 Hz, the `safety_controller` reacts to each reading (the
`emergency stop` lines are its own `tracing::info!`, printed each time the
simulated obstacle comes within half a meter), and the `drive_base` receives
velocity commands, all through **zero-copy shared memory**, with a crash in any
one group contained to its own process.

> **Warnings you will see on this first run, and what they mean.**
> - `WARN auto-partition declined: running MULTI-PROCESS with the derived process
>   groups IN-MEMORY` is the Enter you just pressed, said back to you.
> - `WARN absolute source is inside this graph's own prefix namespace but matches
>   no declared output` (twice per consuming worker) suggests a misspelled
>   `source:`. Nothing is misspelled: in a multi-process run each worker sees only
>   its own part of the graph, so its upstream looks external to it.
> - `WARN network egress OPEN BY DEFAULT`: with no `network:` block, the topics
>   are visible to other Cerulion machines on your LAN. `--network off` keeps a
>   run local-only. The first networked run also starts the shared `cerulion-netd`
>   daemon and says so; it exits by itself when idle.

Leave it running and open a second terminal.

---

## Step 6: Inspect Topics (in a second terminal)

While the graph is running, open another terminal, `cd` into `safety_demo`,
and use the topic introspection commands.

### List active topics

```bash
cerulion topic list
```

**Expected output:**

```
TOPIC
/demo/lidar_sensor/scan
/demo/safety_controller/cmd_vel
1 internal topic hidden (--all shows it)
remote: none discovered in 500 ms (a robot off the LAN needs --connect tcp/<host>:7683)
```

The hidden topic is `/bagd/status`, the recorder's own status channel (it is
there because of `--record`). `cerulion topic list --all` lists it, with an
`internal` marker after its path:

```
TOPIC
/bagd/status  internal
/demo/lidar_sensor/scan
/demo/safety_controller/cmd_vel
remote: none discovered in 500 ms (a robot off the LAN needs --connect tcp/<host>:7683)
```

The `remote:` line is the LAN discovery half of the verb; with no other
Cerulion machine around it reports none. `cerulion topic list --no-network`
skips that query and prints the local list alone, count line included.

> **Topic naming:** Topics use the canonical leading-`/` form
> `/{prefix}/{node_id}/{port_name}`. The prefix `demo` comes from the graph
> YAML. This namespacing prevents collisions when multiple graphs run
> simultaneously. Pass the leading-`/` form to the topic commands; a
> slashless name errors with a did-you-mean suggestion.

### Echo messages

Watch the velocity command the controller publishes:

```bash
cerulion topic echo /demo/safety_controller/cmd_vel
```

**Expected output (streaming):**

```
seq=152 ts=15300000000ns schema=0xd43ee5592039b9df size=56
  geometry_msgs/Vector3:
    x: 0.5
    y: 0.0
    z: 0.0
seq=153 ts=15400000000ns schema=0xd43ee5592039b9df size=56
  geometry_msgs/Vector3:
    x: 0.5
    y: 0.0
    z: 0.0
```

The first line of each message is its wire header:
- `seq`: the wire sequence number (monotonically increasing)
- `ts`: the wire timestamp in nanoseconds. In this multi-process run it is the
  graph's deterministic logical clock, which advances 100 ms per step, not a
  wall-clock reading
- `schema`: the layout-sensitive schema hash (see the wire-format footnote below)
- `size`: total wire size (header + payload)

Below it the message is decoded field by field. `x` switches between `0.5` and
`0.0` as the simulated obstacle crosses the half-meter threshold.

The scan is bigger, so cap how much of each array is printed:

```bash
cerulion topic echo /demo/lidar_sensor/scan --truncate-length 4
```

```
seq=119 ts=12000000000ns schema=0x64cc8631dc24946b size=1361
  sensor_msgs/LaserScan:
    angle_min: -1.57
    angle_max: 1.57
    angle_increment: 0.01
    time_increment: 0.0
    scan_time: 0.0
    range_min: 0.05
    range_max: 10.0
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: "laser"
    ranges: [5.0, 5.0, 5.0, 5.0, ...] (314 elements)
    intensities: []
```

Press **Ctrl+C** to stop echoing.

### Measure publish rate

```bash
cerulion topic hz /demo/lidar_sensor/scan
```

**Expected output (updated every second):**

```
average rate: 10.00 Hz, min: 0.1000s max: 0.1000s std: 0.0000s window: 10
average rate: 10.00 Hz, min: 0.1000s max: 0.1000s std: 0.0000s window: 10
```

This confirms the LIDAR sensor is publishing at the configured 10 Hz. (The
figures are exact because the rate is computed from the wire timestamps, which
this run stamps from its logical clock.)

### Get topic info

```bash
cerulion topic info /demo/lidar_sensor/scan
```

**Expected output:**

```
Topic: /demo/lidar_sensor/scan
Schema: sensor_msgs/LaserScan (0x64cc8631dc24946b)
Last sequence: 182
Last timestamp: 18300000000ns
```

---

## Step 7: Stop, Verify, Change, Verify Again

Let the graph run for at least five seconds, then go back to the first terminal
and press **Ctrl+C**. The workers drain, the recorder finalizes the bag, and the
run prints where it went:

```
INFO live loop delivery telemetry node_id=lidar_sensor fires=387
INFO live loop delivery telemetry node_id=safety_controller input=scan fires=387 drop_oldest=0
INFO live loop delivery telemetry node_id=drive_base input=cmd_vel fires=387 drop_oldest=0
...
recording written to recordings/obstacle_avoidance_<timestamp>.mcap
```

The telemetry lines are the run's own delivery account: how often each node
fired, and how many queued frames its inputs lost. `drop_oldest=0` on both
inputs means no frame was evicted from a full queue. To check what actually
crossed, read the recorded frames of the topic itself.

The bag is a standard MCAP file that holds the frames, the graph, the environment
and the scheduler's trace. `cerulion bag info` summarizes it:

```bash
BAG="recordings/obstacle_avoidance_<timestamp>.mcap"   # the path the run printed
cerulion bag info "$BAG"
```

```
bag: recordings/obstacle_avoidance_<timestamp>.mcap
state: finalized
frames: 774 across 2 topic(s); span 38.600s

TOPIC                                       FRAMES   MAX BYTES  SCHEMA
/demo/lidar_sensor/scan                        387        1361  sensor_msgs/LaserScan
/demo/safety_controller/cmd_vel                387          56  geometry_msgs/Vector3
...
```

### Verify the unchanged code

Re-execute the recording against the nodes you just built, and compare every
output frame with what was recorded:

```bash
cerulion bag play "$BAG" --resim all --verify
```

It ends with:

```
replay PASS: recordings/obstacle_avoidance_<timestamp>.mcap (387 tick(s) replayed, 2/2 topic(s) matched byte-for-byte and were credited)
```

and exits 0. (It may also print `WARNING: env divergence` lines for shell
variables such as `SHLVL` that differ between the two terminals. They are
advisory; the verdict is the `replay PASS` line and the exit code.)

### Change the controller, and verify again

Open `nodes/safety_controller/src/lib.rs` and change the cruising speed:

```rust
const CRUISE_SPEED: f64 = 0.25;   // was 0.5
```

Rebuild that one node and verify against the **same recording**:

```bash
cerulion node build safety_controller --release
cerulion bag play "$BAG" --resim all --verify
```

This time the verdict is a divergence, and the exit code is 1:

```
FRAME-CONTENT DIVERGENCE: recordings/obstacle_avoidance_<timestamp>.mcap
  387 tick(s) replayed; 1/2 topic(s) matched and were credited; 1 violation(s):
  - /demo/safety_controller/cmd_vel [byte-mismatch]: frame 0 differs at byte 38 on '/demo/safety_controller/cmd_vel' (recorded 56 B, replayed 56 B)
  Topics passed: 1/2
```

On the same sensor inputs, the changed controller commands a different speed. The
scan topic still matches, because the sensor did not change. That is the loop:
record once on the robot, then check every later change against the recording
before it reaches the robot again. In CI the nonzero exit is the gate.

---

## How it works underneath

Now that you have a working system, let's review what's happening under the
hood.

### Message flow

```
+--------------+     shared memory      +--------------------+     shared memory      +--------------+
| lidar_sensor | sensor_msgs/LaserScan-->| safety_controller  | geometry_msgs/Vector3 -->|  drive_base  |
|  (10 Hz)     |     zero-copy pub      |  (data-triggered)  |     zero-copy pub      | (data-trig.) |
+--------------+                        +--------------------+                        +--------------+
```

1. **lidar_sensor** ticks every 100ms (10 Hz periodic policy). Each tick loans
   a shared memory buffer from iceoryx2, writes the `LaserScan` message directly
   into it, and notifies subscribers.

2. **safety_controller** is woken by a **data trigger** event from
   `lidar_sensor/scan`. It reads the scan using the typed receive API (zero-copy
   for BOTH fixed fields and variable-length arrays: `ranges()` borrows straight
   out of the loaned SHM payload, which is where `lidar_sensor` wrote it), finds
   the closest valid reading across all rays, decides whether to stop, and
   publishes a `Vector3` velocity command.

3. **drive_base** is woken by a data trigger from `safety_controller/cmd_vel`
   and logs the received command.

### Zero-copy path

```
Publisher:     loan buffer  ->  write fields  ->  send (notify)
Subscriber:    wait/poll    ->  callback with &[u8] reference into shared memory
```

- **Fixed-size fields** (like `LaserScan.angle_min: f32`) are written directly
  into shared memory -- **0 copies**.
- **Variable-size fields** (like `LaserScan.ranges`) are written in place with
  `self.scan.loan_ranges(n)?` or `self.scan.ranges.fill_from(...)` -- **0
  copies**, which is what `lidar_sensor` does. Plain assignment
  (`self.scan.ranges = source`) costs one copy from the source into the frame
  (see "`fill_from`: zero-copy producer writes" in the user API).
- **Subscribers** get a `&[u8]` reference directly into shared memory -- **0 copies**
  on the receive side, regardless of message size.

### Wire format

Every message on the wire has a 32-byte `WireHeader`:

```
+----------------------------------+
|  schema_hash:         u64  (8B)  |  Layout-sensitive schema hash*
|  total_size:          u32  (4B)  |  Header + payload bytes
|  offset_table_offset: u32  (4B)  |  For variable-length fields
|  offset_table_count:  u32  (4B)  |  Number of variable fields
|  sequence:            u32  (4B)  |  Monotonic counter
|  timestamp_ns:        u64  (8B)  |  Publish time (nanoseconds)
+----------------------------------+
|  Payload (message fields)        |
+----------------------------------+
```

\* The schema hash is FNV-1a 64 over a length-prefixed stream of the
schema name, the fixed-section size, and every field's name + canonical
type string in declaration order. It catches own-field layout
changes (rename / reorder / retype / add / remove). It ALSO catches a layout
change inside a nested schema when that nested field resolved **fixed**: the
target's full schema hash is folded into the parent's (including a fixed
nested type inside a fixed array), so e.g. changing `Quaternion` bumps the hash
of every message embedding one. Only a **variable**-resolved nested field (one
whose target carries a string or a dynamic array, like `LaserScan.header`)
contributes name-only, and a layout change inside such a target is not caught.

---

## What's Next

- **Add history for late joiners:** Set `history_size: 5` on an output to
  buffer the last 5 messages and replay them to new subscribers.

- **Use sync triggers:** Fire `tick` only when all `#[input(trigger)]`
  ports receive a message within a time window. Declare `sync_window_ms`
  on the macro plus a `(trigger)` marker on every input that must
  participate (two or more of them); the graph YAML wires each input's
  `source:` as usual.
  ```rust
  #[cerulion_node(sync_window_ms = 50)]
  #[derive(Default)]
  struct FusionNode {
      #[input(trigger)] image: sensor_msgs::Image,
      #[input(trigger)] imu:   sensor_msgs::Imu,
      // ...
  }
  ```

- **Create custom schemas:** Define your own message types in `schemas/`:
  ```bash
  cerulion schema create obstacle_report
  ```
  Then edit `schemas/obstacle_report.yaml` to add fields.

- **Measure latency:** For latency measurement techniques, see [Measuring Latency](02-latency-measurement.md).

- **Keep the partition:** Answer **y** at the prompt (or run
  `cerulion graph partition obstacle_avoidance`) to write the `process_groups:`
  block into the graph file, so later runs start without asking.
  `cerulion graph profile obstacle_avoidance` measures the nodes first, which
  lets the partition fuse cheap nodes into one process.

- **Multiple instances:** The same node type can appear multiple times in a
  graph with different IDs. Both instances share the trigger policy
  declared on the type's macro attribute (here, `#[cerulion_node(period_ms = 100)]`
  from Step 3a). If two instances need different policies, make them two
  node types. Stage the second instance with its own id:
  `cerulion node stage lidar_sensor -g obstacle_avoidance -i rear_sensor`.
  ```yaml
  - id: front_sensor
    type: lidar_sensor
  - id: rear_sensor
    type: lidar_sensor
  ```

- **QoS deadlines:** Independent of trigger policy, every port + the node
  itself can declare a deadline. The runtime counts misses per port and emits
  a `tracing::warn!` on each violation, which is good for control-loop safety:
  ```rust
  #[cerulion_node(period_ms = 100, tick_within_ms = 50)]
  struct ControllerNode {
      #[input(expect_within_ms = 200)]            // subscriber QoS
      scan: LaserScan,                            // NOT `trigger`: a trigger
                                                  // input cannot combine with
                                                  // `period_ms` (compile error)
      #[output(promise_within_ms = 100)]          // publisher QoS
      cmd_vel: Vector3,
  }
  ```
  Each miss is a warn line in the run's log naming the node, the port and how
  late it was; a node can also react in code with an `#[on_event]` handler. See
  [QoS deadlines and miss counters](../user-api.md#qos-deadlines-and-miss-counters)
  in the user API.

- **Capture the moment:** every serving `graph run` holds a rolling window of
  the last 30 seconds in memory, recorded or not. A capture writes that window
  plus the 15 seconds after the trigger to a bag. The runtime captures on its own
  when it sees a fault, and you can ask for one by hand from another shell:

  ```bash
  cerulion flashback --note "controller stopped late"
  ```

  `cerulion bag info` reports whether the capture carries the scheduler trace
  that re-execution needs. See [docs/flashback.md](../flashback.md);
  [docs/bag.md](../bag.md) covers recording and playback in full.

- **Visualize it:** name the topics you want, and
  [Cerulion Studio](../../README.md#studio-for-your-computer) renders them from
  the same daemon. Nothing is added to the graph.

  ```bash
  cerulion viz /demo/lidar_sensor/scan /demo/safety_controller/cmd_vel
  ```

---

## Quick Reference

| Command | Description |
|---------|-------------|
| `cerulion workspace create <name>` | Create a new workspace |
| `cerulion node create <type> [--policy period_ms=N\|sync_window_ms=N\|data_trigger=NAME\|external] [-o SCHEMA NAME] [-T SCHEMA NAME] [-i SCHEMA NAME]` | Create node with ports (`-T` = trigger input, equivalent to `--policy data_trigger=NAME`; source-only nodes require `--policy`) |
| `cerulion node list` | List all node types |
| `cerulion node info <type>` | Show node details |
| `cerulion node build <type> [--release]` | Build node cdylib |
| `cerulion node stage <type> -g GRAPH [-i ID] [-I NAME SOURCE]...` | Add an instance to a graph and wire its inputs |
| `cerulion graph create <name> [-n prefix]` | Create a graph |
| `cerulion graph validate <name> [--release]` | Check the graph, its nodes, their libraries and schemas without running |
| `cerulion graph run <name> [--release] [--record] [--single-process] [--yes]` | Run the graph live (Ctrl+C to stop). `--release` loads release-built nodes; `--record` writes a bag under `recordings/`. An unpartitioned graph derives a multi-process partition on Linux and macOS (`--single-process` opts out; `--yes` persists it without the prompt) |
| `cerulion graph partition <name> [--dry-run] [--yes]` | Derive + write the `process_groups:` partition (cost-aware with a `graph profile` snapshot; process-per-node baseline without): preview + confirm, surgical rewrite, `.bak` backup |
| `cerulion graph list` | List all graphs |
| `cerulion topic list` | List active topics |
| `cerulion topic echo <topic> [--truncate-length N]` | Stream messages, decoded field by field |
| `cerulion topic hz <topic>` | Measure publish rate |
| `cerulion topic info <topic>` | Show the topic's schema and last message metadata |
| `cerulion bag info <bag>` | Summarize a recording |
| `cerulion bag play <bag> --resim all --verify` | Re-execute a recording against the current nodes and compare every output frame (exit 0 = match) |
| `cerulion schema create <name>` | Create a schema YAML |
| `cerulion schema info <name>` | Show schema fields (fixed vs variable), wire fixed size + hash, for workspace `schemas/*.yaml` or built-in ROS 2 types (`pkg/Type`); workspace wins on a name collision, with a loud shadow warning |
| `cerulion schema list` | List workspace schemas + all built-in ROS2 types grouped by package |

---

## Troubleshooting

**"cdylib not found" error:**
Build the node first with `cerulion node build <type> --release`. The runtime looks
for `lib<type>.dylib` (macOS) or `lib<type>.so` (Linux) under `target/release/`
and `target/debug/`, and `cerulion graph validate` says which one it found.

**`no trigger policy` build error:**
The node has no way to fire. Give one input `#[input(trigger)]`, or put
`period_ms = N` on `#[cerulion_node(...)]` (Step 2b explains how this happens
with `-i`).

**`graph run --record` refuses with `DoesNotSupportRequestedMinBufferSize`:**
The topic still exists from a run WITHOUT `--record` that you stopped seconds
ago, at the smaller buffer size such a run uses: the shared network daemon that
run started still holds it. The daemon exits by itself after about half a minute
of idleness and the topic goes with it (`cerulion topic list --no-network` shows
when it has), so wait, then start the recording again. `cerulion clean` does not
help here, because nothing is dead.

**"workspace not found" error:**
Make sure you're running commands from inside the workspace directory (or a
subdirectory). The CLI walks upward looking for `Cargo.toml` with `[workspace]`.

**No topics showing in `topic list`:**
Topics only exist while a graph is running. Start the graph first, then list
topics in a separate terminal.

**Build errors about `cerulion_core`:**
Look at the two entries under `[workspace.dependencies]` in the workspace's
root `Cargo.toml` (the `dependencies:` line printed at creation says which
form was written). With exact published pins (`= "=X.Y.Z"`), a resolution
failure means that version is not on crates.io: edit the pins to a published
version, or install a CLI built from a checkout. With absolute `path`
dependencies, a failure means the checkout moved or was deleted: point the
two entries at its new location. Nothing rewrites the manifest in place;
`cerulion workspace create` only writes it for a NEW workspace.
