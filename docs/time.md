# Clocks and time

A Cerulion graph runs on one clock. Every period trigger, every input and
output deadline (`expect_within_ms`, `promise_within_ms`), every output stamp
and every `self.now_ns()` read in a node comes from that clock, and the same
clock is what `cerulion graph run` chooses with `--time-source` or what the
graph YAML chooses with `sim_time:`. The one exception is `tick_within_ms`: it
is an execution budget, timed on the machine's monotonic clock around the
callback, so it measures real time under every clock, sim time included, and
keeps measuring while `/clock` is paused. This page names the clock domains, says
which one each run mode uses, and explains sim time.

## The three clock domains

| Domain | What drives it | How you get it |
| --- | --- | --- |
| Real | The machine's monotonic clock. Under `cerulion graph run` it is the run clock, which stands still while the run is paused with `cerulion graph pause` and continues on resume. | The default. `--time-source real`. |
| Virtual | The scheduler: the clock moves only when the graph steps. On `--time-source virtual` every step is exactly 1 ms. A recording and its replay use this domain too: the recorded step boundaries are what `cerulion bag play --resim` hands back, so the replay is byte-identical to the live run. | `--time-source virtual`; `--record`; `bag play --resim`. |
| Sim | A simulator or sim bridge: `rosgraph_msgs/Clock` samples published on `/clock`. The graph steps once per sample, so the graph advances only when `/clock` does. | `sim_time: true` in the graph YAML, or `--time-source external`. |

Sim time is a controlled clock in the virtual domain: the samples on `/clock`
decide how far each step advances it, and nothing else moves it. A sim-time
run is therefore a pure function of the `/clock` sample sequence: two runs
over the same samples fire the same nodes at the same stamps. It is not
recordable yet (`--record` is refused, see the limits below).

## What a node reads

| Read | Real | Virtual | Sim |
| --- | --- | --- | --- |
| `self.now_ns()` (the clock the scheduler fires on; use this) | monotonic ns | the stepped counter | the last accepted `/clock` sample |
| `self.virt_ns()` | `None` (warns once) | `Some(counter)` | `Some(sim time)` |
| `self.ext_ns()` | `None` (warns once) | `None` (warns once) | `None` (warns once; sim time is read with `now_ns()`) |
| `self.real_ns()` | monotonic ns | monotonic ns | monotonic ns |

Python nodes read the same clock through `ctx.now_ns`. Output stamps
(`timestamp_ns` on the wire) are minted from the same clock the node fires on,
so under sim time every stamp is a sim time.

## Sim time

Turn it on with one key in the graph YAML:

```yaml
sim_time: true
nodes:
  - id: controller
    type: controller
```

or, for a graph whose YAML does not say so, with `cerulion graph run <name>
--time-source external`. The two select the same path. A graph that says
`sim_time: true` and is run with `--time-source virtual` is refused: a graph
has one clock source. Drop the flag to run on sim time, or set
`sim_time: false` to run on the virtual clock.

What happens when the graph starts:

1. **Time is not valid until the first sample.** The graph waits for the first
   `rosgraph_msgs/Clock` on `/clock`. Nothing is built and nothing
   time-triggered fires before it. If no clock has arrived after two seconds
   the run prints one warning that names `/clock` and the key, and keeps
   waiting; a clock that arrives in time prints nothing.
2. **The first sample is the epoch.** The graph clock is placed at it before
   the nodes are built, so a period node's first deadline is one period after
   the epoch and no deadline window counts a phantom miss.
3. **One step per sample.** A sample ahead of the last accepted one steps the
   graph by the difference. A sample equal to it steps the graph by zero, so
   data triggers still drain while a paused simulator republishes the same
   time. A period node fires at most once per accepted sample, however many
   of its deadlines the jump between two samples passed: it fires once, when
   that sample is accepted, at the latest deadline the jump passed, and its
   next deadline is one period after that. The missed deadlines are
   not fired later. A 10 ms node whose clock jumps from 1.000 s to 1.100 s
   fires once, at 1.100 s. So a node with a period shorter than the gap
   between samples fires at the clock rate, not at its own. Publish `/clock`
   at least as fast as the fastest period node in the graph.
4. **Time never goes backwards.** A sample behind the last accepted one is
   refused with a warning and is never stored; the graph clock stands at the
   last accepted sample. A simulator that restarts its clock needs the graph
   restarted too. The warning is loud once and then counted, so a stream of
   backward samples cannot flood the log. A refused sample does not count as
   a clock: a stream of them still reaches the stalled-clock warning below.
5. **A clock that stops is reported.** When no sample has arrived for two
   seconds after the last accepted one, the run prints one warning per such
   gap that names `/clock`; the graph clock stands still, and nothing fires or
   drains until the next sample. The next sample prints that the clock is
   back.

A frame on `/clock` that is not a `rosgraph_msgs/Clock` (another type, a wire
header whose size does not match the frame, a short payload, a negative stamp)
is skipped with the same loud-once, counted warning. No field of a frame with a
bad header is read. A skipped frame does not count as a clock: a stream of them
still reaches the no-clock and stalled-clock warnings above.

### A clock publisher

Anything in another process that publishes `rosgraph_msgs/Clock` on `/clock`
drives the graph: a simulator's bridge, or a node of your own whose output
declares `schema: rosgraph_msgs/Clock` and `topic: /clock`, run in a separate
graph on the real clock. The type is part of the built-in message set; its one
field, `clock`, is a `builtin_interfaces/Time` (`sec`, `nanosec`).

The clock publisher cannot be a node of the sim-time graph itself: that graph
builds nothing until the first sample arrives, so a node inside it would never
be built and the run would wait forever. A `sim_time: true` graph with an
output on `/clock` is refused at load, as is one that lists `/clock` in
`multi_publisher_topics:` (two clock sources). `cerulion graph run
--time-source external` holds a graph to the same two rules.

### What sim time does not do yet

- **Multi-process.** A graph with `process_groups:` is refused under sim time:
  every worker runs on the real clock, and one `/clock` stream driving several
  processes is not supported. Run it with `--single-process`, or drop the
  switch. A graph without `process_groups:` runs in one process under sim
  time, so nothing else is needed.
- **External-policy nodes.** A node declared `#[cerulion_node(external)]`
  never fires under sim time: the graph steps once per `/clock` sample and
  nothing sweeps the node's descriptor or doorbell between samples. A
  sim-time graph with such a node is refused at launch, before it waits for a
  clock, naming the nodes. Set `sim_time: false` and drop `--time-source
  external` to run them on the live clock, or move them to a separate graph
  on the real clock.
- **Recording.** `--record` requires the live clock. A sim-time run is not
  recordable yet.
- **Networking.** A sim-time run opens no gateway, like every controlled-clock
  run. A graph whose `network:` block is enabled is told so at launch, with
  the egress and ingress topics that stay local for the run.
- **Profiling.** `cerulion graph profile` measures the real clock and reads no
  `/clock`; a `sim_time: true` graph is profiled on real time, and the profile
  says so.
- **Pause.** `cerulion graph pause` does not pause a sim-time run; the
  simulator owns the clock, so pause the simulator.
- **`bag play` as the clock.** Played-back data does not publish `/clock`.
