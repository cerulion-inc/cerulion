# obstacle_avoidance

A two-node Cerulion graph mirroring the [Define a node](https://docs.cerulion.com/cerulion/guides/define-a-node) guide:

```
laser_scanner  --(sensor_msgs/LaserScan)-->  safety_controller  --(geometry_msgs/Vector3)-->
```

- **`laser_scanner`**: periodic source (`period_ms = 20`) publishing a `sensor_msgs/LaserScan`.
  A real driver would `fill_from` the hardware; this example synthesizes a
  180-beam forward sweep directly in the loaned range array, whose distance oscillates across the safety threshold so the
  controller visibly switches state.
- **`safety_controller`**: data-triggered loop (`#[input(trigger)] scan`, no
  `period_ms`) that fires once per published scan and emits a forward velocity:
  it stops (`x = 0.0`) when any beam is closer than 0.5 m, else cruises
  (`x = 0.3`).

The trigger is deliberate. The controller runs when a scan arrives, so every
command it emits is computed from a new measurement, and that is what keeps
this graph deterministic when
`cerulion graph run` splits the two nodes into one process per node (the
default): a `#[input(trigger)]` edge is a DAG edge, so the scanner
levelizes strictly above the controller and the cross-process level-boundary
barrier orders the publish before the read. Polling the same scan on a
`period_ms` timer leaves the edge out of the DAG: both nodes share one level,
which scan a tick pairs with becomes OS-scheduled, and the recording does not
replay. (Co-locating both nodes in one `process_groups:` group, or running
`--single-process`, also avoids that; see "Scope of the data guarantee" in
`docs/multi_process.md`.)

This example is a standalone workspace: run its commands from inside this
directory. See [how these workspaces work](../README.md#these-are-standalone-workspaces).

## Run it

The same sequence as the quickstart in the top-level `README.md`:

```bash
cd examples/obstacle_avoidance

# Build both node libraries
cerulion node build laser_scanner --release
cerulion node build safety_controller --release

# Check the graph (exits nonzero on any failure)
cerulion graph validate obstacle_avoidance

# Run live with recording on (Ctrl+C to stop)
cerulion graph run obstacle_avoidance --release --record
```

The first node build in a workspace also compiles the Cerulion runtime, so it
takes a few minutes; later builds take seconds. On the first run Cerulion
proposes one process per node and asks to save that partition; see
[the first build and the partition prompt](../README.md#the-first-build-and-the-partition-prompt).

In a second terminal, watch the velocity command flow:

```bash
cerulion topic list
cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

The `x` field switches between `0.3` and `0.0` as the synthetic scan crosses
the distance threshold.

## Verify the recording

Stop the run with **Ctrl+C**; Cerulion prints the recording's path. Re-execute
the current node code against it:

```bash
BAG="recordings/obstacle_avoidance_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

`replay PASS` and exit code 0 mean the rebuilt nodes reproduced the recorded
outputs. Change the clear-path velocity in `nodes/safety_controller/src/lib.rs`
from `0.3` to `0.25`, rebuild that node and run the same command again: the
verdict turns into a divergence with a nonzero exit code.

## Layout

```
obstacle_avoidance/
├── Cargo.toml                       # [workspace] members = ["nodes/*"]
├── .cargo/config.toml               # IOX2_LOG_LEVEL / RUST_LOG defaults
├── graphs/obstacle_avoidance.yaml   # graph topology; trigger policies are declared on nodes
└── nodes/
    ├── laser_scanner/src/lib.rs      # one node type, macro form
    └── safety_controller/src/lib.rs  # one node type, macro form
```

## Verify the source

```bash
cargo test --locked -p laser_scanner --lib -- --test-threads=1
```

This scheduling example uses generated scans. A physical robot additionally
needs sensor validity checks, stale-input handling and an actuator shutdown path.
