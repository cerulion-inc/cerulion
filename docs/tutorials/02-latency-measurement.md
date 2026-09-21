# Measuring Latency in Cerulion Pipelines

How to measure single-hop and end-to-end latency in multi-node Cerulion graphs.

---

## Two Kinds of Timestamps

| Timestamp | Set by | Scope |
|-----------|--------|-------|
| `WireHeader.timestamp_ns` | Transport layer (automatic each time a node publishes an output) | **Per-hop**: every hop stamps its own |
| Message payload timestamp (for example `std_msgs/Header.stamp`) | Your node code | **End-to-end**: preserved across hops |

> **Key insight:** `WireHeader.timestamp_ns` only measures the **last hop**,
> because each intermediate node stamps its own publish. To measure
> end-to-end latency, carry a source timestamp in the message payload.

---

## Single-Hop Latency

Compare `WireHeader.timestamp_ns` against a reading from the **same clock
domain**. In a macro node, read the stamp straight off the input:

```rust
// Requires a `--single-process` run: see the clock-domain warning below.
let publish_time = self.cmd_vel.wire_timestamp_ns();
let now = self.real_ns();
let latency_ns = now.saturating_sub(publish_time);
tracing::info!(latency_us = latency_ns / 1000, "single-hop latency");
```

`self.<input>.wire_timestamp_ns()` (and `wire_header()` for the whole header) is
built for exactly this job: it reads the stamp of the message this tick is
looking at. It is the whole API; a macro node has nothing lower to reach for.
`self.real_ns()` is the node's own read of the machine's monotonic clock, the
one accessor meant for measuring real durations (it is not replay-deterministic,
which is fine for a measurement you only log).

> **Clock domains do not mix.** Under `RealClock` the wire stamp is
> `real_ns()`: **monotonic time since boot**, not the Unix epoch. Subtracting it
> from `SystemTime::now()` yields a meaningless ~50-year number. Compare it only
> against `self.real_ns()`, on the same machine.
>
> **And only under `--single-process`.** The default `cerulion graph run`
> derives a multi-process partition whose workers run on a gating `VirtualClock`
> starting at 0 and advancing by a fixed logical quantum per step, so those wire
> stamps are deterministic LOGICAL values and are not wall-comparable at all.
> `--single-process` is what puts the run on `RealClock`; it is required here,
> not a convenience.

---

## End-to-End Latency

For a multi-hop pipeline like `lidar_sensor -> safety_controller -> drive_base`,
end-to-end measurement requires three steps:

**1. Source node embeds a timestamp in the message payload.**
The `lidar_sensor` writes `self.real_ns()` into a field (for example
`header.stamp`) each tick. This timestamp travels with the data.

**2. Intermediate nodes carry the timestamp through.**
The `safety_controller` copies the source timestamp from its input into its
output. If the output schema lacks a header, use a dedicated `u64` field.

**3. Sink node compares the carried timestamp to current time.**
The `drive_base` reads the forwarded source timestamp and subtracts it from
its own `self.real_ns()` to get end-to-end latency. Both reads are the machine's
monotonic clock, which every node reads directly, so a carried `real_ns` value
is comparable on one machine even when source and sink run in separate
processes. The `--single-process` requirement above applies to the wire-stamp
comparison, not to a carried one.

> **Measure in a measurement build, not in the node you ship.** A payload field
> stamped from `self.real_ns()` differs on every run, so a recording of that graph
> does not verify byte for byte under `cerulion bag play --resim all --verify`
> (see `docs/replay_determinism_footguns.md`). Logging a latency is harmless;
> writing a wall reading into an OUTPUT is what breaks replay.

> **Why not use `WireHeader.timestamp_ns`?** When `safety_controller` publishes
> its `Vector3`, the transport stamps `timestamp_ns` with that publish's own time.
> The original `lidar_sensor` publish time is lost. Only a user-carried payload
> timestamp survives across hops.

---

## Release Mode Reminder

Debug builds carry debug assertions and omit optimizations, so their latency is
not the latency you ship. Always measure in release mode:

```bash
cerulion node build lidar_sensor --release
cerulion node build safety_controller --release
cerulion node build drive_base --release
cerulion graph run obstacle_avoidance --release --single-process
```

`--single-process` is REQUIRED here, not a convenience. Without it,
`cerulion graph run` derives a multi-process partition by default for a graph
with no `process_groups:` block, and each worker runs on a gating `VirtualClock`
that starts at 0 and advances by a fixed logical quantum per step. Its wire
stamps are deterministic LOGICAL values, and subtracting a wall reading from one
measures nothing. `--single-process` is what puts the run on `RealClock`, whose
stamps are wall-comparable within the machine.

Expected local (iceoryx2) latency for fixed-size messages is **single-digit
microseconds per round trip** on a desktop and tens of microseconds on an embedded
board: the README's performance table and `docs/benchmarks/results/` carry the
measured figures per platform. The cost is dominated by wake + transport, not by
payload size, and that flatness is the property the release gates pin.
