# basic_timer

The smallest Cerulion graph: two nodes, one topic.

```
timer  --(std_msgs/Int32)-->  printer
```

- **`timer`** is a periodic source (`period_ms = 100`) that publishes an
  incrementing counter on `count`.
- **`printer`** is a data-triggered sink (`#[input(trigger)] count`, no
  `period_ms`) that fires once per published count and logs the value with
  `tracing`.

Each node type is its own crate under `nodes/<type>/`, written in the macro
form. All wiring lives in `graphs/basic_timer.yaml`. The graph file carries
topology only: each node's trigger policy is on its macro, in
`nodes/<type>/src/lib.rs`.

## How this workspace was made

The workspace was scaffolded with these verbs. The node bodies and their
comments, the relative dependency paths in `Cargo.toml`, the `recordings/`
line in `.gitignore` and the pinned `Cargo.lock` were then written by hand.

```bash
cerulion workspace create basic_timer
cd basic_timer
cerulion node create timer --policy period_ms=100 -o std_msgs/Int32 count
cerulion node create printer -T std_msgs/Int32 count
cerulion graph create basic_timer --prefix basic_timer
cerulion node stage timer -g basic_timer
cerulion node stage printer -g basic_timer -I count timer/count
```

This is a standalone workspace: it has its own `[workspace]` `Cargo.toml` and is
excluded from the repo's root workspace (see the root `Cargo.toml` `exclude`).
Inside this repository it depends on `cerulion_core` and `native_ros2_messages`
through the relative paths in `Cargo.toml`, so run it from inside this
directory. A workspace you create yourself gets the published crates.io
versions instead.

## Run it

```bash
cd examples/basic_timer

# Build both node libraries
cerulion node build timer --release
cerulion node build printer --release

# Check the graph (exits nonzero on any failure)
cerulion graph validate basic_timer

# Run live with recording on (Ctrl+C to stop)
cerulion graph run basic_timer --release --record
```

The first node build also compiles the Cerulion runtime, so it takes a few
minutes; later builds take seconds. On the first run, Cerulion proposes one
process per node and asks `Apply this partition to the graph file? [y/N]`.
Press **Enter** to use that layout for this run only.

The run terminal shows the printer firing ten times a second:

```
INFO printer: count received value=1 received=1
INFO printer: count received value=2 received=2
```

In a second terminal, watch the topic itself:

```bash
cerulion topic list
cerulion topic echo /basic_timer/timer/count
cerulion topic hz /basic_timer/timer/count
```

## Verify the recording

Stop the run with **Ctrl+C**; Cerulion prints the recording's path. Re-execute
the current node code against it:

```bash
BAG="recordings/basic_timer_<timestamp>.mcap"
cerulion bag play "$BAG" --resim all --verify
```

`replay PASS` and exit code 0 mean the rebuilt nodes produced the recorded
`count` frames byte for byte. Change the increment in
`nodes/timer/src/lib.rs`, rebuild that node and run the same command again: the
verdict turns into a divergence with a nonzero exit code.

## Layout

```
basic_timer/
  Cargo.toml                 # [workspace] members = ["nodes/*"]
  .cargo/config.toml         # IOX2_LOG_LEVEL / RUST_LOG defaults
  graphs/basic_timer.yaml    # topology only: ids, types, inputs, outputs
  nodes/
    timer/src/lib.rs         # one node type, macro form
    printer/src/lib.rs       # one node type, macro form
```
