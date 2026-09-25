# Cost-Aware Auto-Partitioning: `graph profile` and the Cost Snapshot

Hand-writing `process_groups:` (see `docs/multi_process.md`) means guessing
which nodes are cheap enough to share a process and which edges are hot enough
to keep in-process. Auto-partitioning replaces the guess with a measured pipeline:

```
cerulion graph profile <name>          # 1. profile the LIVE graph once
        │
        ▼
graphs/<name>.costs.yaml               # 2. the cost snapshot (user-editable)
        │
        ▼
the auto-partitioner                   # 3. consumes the snapshot, emits
                                       #    process_groups: (validated
                                       #    spawner-consumable by construction)
```

The partition policy is **process-per-node baseline + validated greedy
fusion**: every node starts in its own process (maximum fault isolation), and
tightly-coupled low-latency chains are fused under a per-group compute budget,
minimizing cross-process topic edges. Fusion score:
`coupling(edge) = rate(edge) × (cross_ns − intra_ns)`, the per-second latency
saved by keeping the edge in one process.

## Step 1: profile once

```bash
# Defaults: per-node fire targets AUTO-DERIVED from a short warm-up;
# stops when every warm-up-active node meets its own target, or after 30 s.
cerulion graph profile perception

# Force ONE uniform target for every node (the override / escape hatch).
cerulion graph profile perception --fires 200

# A longer observation window (e.g. slow-starting nodes).
cerulion graph profile perception --duration 120

# Custom artifact location.
cerulion graph profile perception -o tuning/perception.costs.yaml
```

The run uses the SAME live loop as `graph run` (RealClock on the iceoryx2
WaitSet): the profiler measures **real** tick durations and **real** fire
rates; nothing is simulated or estimated. It stops when every targeted node
meets its fire target, at the duration cap, or on Ctrl+C, whichever comes
first (an early Ctrl+C still writes the artifact over whatever window was
observed). At the defaults there is **no rate a graph must sustain to
profile**: each node's target is scaled to its own observed rate, so a 1 Hz
telemetry ticker and a 1 kHz control loop both profile in one run with no
hand-tuned `--fires`/`--duration` recipe.

Recorded per node: the **p50 tick duration** (integer lower-median of observed
`duration_ns` samples). Recorded per triggering edge: the **fire rate in
millihertz** (`fires × 10¹² / window_ns`, integer-exact; every consumer of a
topic inherits its producer's rate, one frame per fire).

### How targets are derived (the default mode)

The run opens with a **warm-up** of `clamp(cap / 10, 1 s, 3 s)` (a 30 s cap
warms up for 3 s). At warm-up end, each node's observed fires are projected
over the full cap and halved (a ÷2 rate-droop tolerance: the run may sample
slower than the warm-up did), then clamped into **[20, 1000]** samples:

```
target = clamp(warmup_fires × cap / warmup × 1/2, 20, 1000)
```

That cap-horizon target is the run's **stop gate** (when has a full run
collected enough?). **Isolation at harvest is judged separately**, against the
same warm-up observation re-projected to the **actual observed window**, so a
mid-run Ctrl+C isolates only nodes that under-sampled what that window could
have delivered, never everything against a cap-length projection (on a full
run, window ≈ cap and the two coincide).

Warm-up fires count toward the totals (the warm-up is an observation phase of
the same continuous run, not a discarded prefix). A node that is **silent
through the warm-up derives no target**: it is excluded from the stop gate (it
cannot hold the run to the cap) and is isolated at harvest with a distinct
"silent through warm-up" marker. Note this includes nodes whose FIRST fire
simply lands after the warm-up (e.g. a period longer than 3 s); for those,
`--fires N` (the uniform override) is the escape hatch, paired with a
`--duration` long enough to collect N fires.

**Profile under representative load.** Goal-gated or driver-fed tails (a
planner that only fires when a goal arrives) are silent unless something feeds
them: run the driver graph alongside the profile so the inputs flow, or the
profiler will (correctly, loudly) isolate them.

### Under-sampled nodes are ISOLATED, not guessed

A node that falls short of **its own** fire target within the observed window
gets **no cost entry**: fabricating one from too few samples would poison the
fusion maths: a measurement nobody took is never invented. It is listed under `isolated:` instead, warned loudly
(one warn per node, with its observed fire count vs its target, or the
"silent through warm-up — no target derived" marker instead of a number), and
the auto-partitioner keeps it in **its own process group** (never fused).
A node that never fired at all and whose triggering inputs saw a zero-rate
topic over the window additionally gets the **starved-trigger hint** naming
those inputs: an upstream goal/driver-fed input may be silent; profile under
representative load (run the driver graph alongside). Isolation is a valid
outcome; the command still exits 0. To sample a slow node, raise
`--duration`; `--fires N` forces the uniform fixed-target mode.

## Step 2: the artifact (`graphs/<name>.costs.yaml`)

```yaml
version: 2              # artifact format version (v1 also readable; newer rejected)
graph: perception       # provenance
window_ns: 5000000000   # the observation window the rates were computed over
nodes:                  # node id -> p50 tick duration (ns); well-sampled only
  detector: 412000
  camera: 88000
edges:                  # observed trigger-edge rates (list: YAML map keys
- producer: camera      # must be scalars, so (producer, consumer) pairs
  consumer: detector    # cannot key a map)
  rate_mhz: 30000       # 30 Hz = 30_000 mHz
isolated:               # under-sampled nodes; NO cost, stay singleton
- diagnostics
hop:                    # hop-cost constants driving the fusion score
  intra_ns: 600         # in-process hop
  cross_ns: 6800        # cross-process boundary tax
derived_budget_ns: 125000  # the DEFAULT per-group budget, FROZEN at
                           # profile time = ceil(Σ node p50 / profile_cores)
profile_cores: 4           # the permitted core count it derived from
```

**The frozen budget.** `derived_budget_ns` is computed ONCE, at
profile time, from the PROFILING machine's permitted core count (Linux: the
`sched_getaffinity` set, cpuset/isolation-aware): the graph's total costed
compute divided evenly across the cores, ceil'd. Readers (`graph partition`,
the `graph run` default) consume the frozen value; they never re-derive it
from the reading machine's cores, so a reviewed partition cannot silently reshape
on a different machine. **Profile on the TARGET machine** (or re-profile there)
to re-derive the budget for its core count. An earlier (v1) artifact
carries neither field and partitions with unbounded fusion (a loud info
suggests re-profiling); an all-isolated profile writes neither field (no
compute to divide). A hand-edited `derived_budget_ns: 0` is never consumed
(a zero budget rejects every fusion): `graph partition` and
`--auto-partition` refuse it loudly; the flagless `graph run` default
degrades to the process-per-node baseline with a loud warn.

**The file is user-editable.** Two supported edits:

* **`hop:` overrides.** The profiler writes per-platform DEFAULT ESTIMATES
  (~1000 ns intra on Apple silicon, ~600 ns on x86_64 Linux,
  ~2100 ns on aarch64 Linux; cross ~6800 ns Linux / ~9000 ns macOS). Paste
  your own measured values over them and every later consumer uses yours. Deleting
  the block falls back to the *reading* machine's platform default.
* **Cost/rate touch-ups**: e.g. hand-raising a p50 you know spikes under a
  load the profiling window missed.

Hand-edits are validated loudly on read: unknown/typo'd keys are rejected at
parse (`deny_unknown_fields` at every level), as are a duplicate
`(producer, consumer)` edge, a node listed in both `nodes:` and `isolated:`,
and an unsupported `version`.

**The `graph:` line is PROVENANCE, and it is CHECKED.** An artifact
whose `graph:` names a different graph than the one being partitioned is
refused: it is not a label, it is what makes the file's costs attributable.
Two graphs can share node ids (`camera`, `detector`, `planner` are not unique
names), so a stale or copied `graphs/<name>.costs.yaml` otherwise fuses a
partition from *another graph's* measured p50s and edge rates, and the result
is credible: real group names, plausible bands, a preview indistinguishable
from a good one. The comparison is against the file STEM the CLI resolved
(the same key `graph profile` writes under and the default path is built from),
so an artifact is usable exactly where its own name says it belongs.

The posture matches every other present-but-unusable class above:

| Path | A foreign artifact |
|---|---|
| `graph partition --costs <PATH>` | **hard error** naming both graphs (an explicitly named file never falls back) |
| `graph partition` (default path) | **hard error**, as for a malformed or wrong-version artifact |
| `graph run` (zero-flag default) | **loud warn + process-per-node baseline** for that run; the graph file is untouched |
| `graph run --auto-partition` | **hard error** (you asked for cost-aware behavior) |

Remedy in every case: `cerulion graph profile <graph>` to harvest this graph's
own snapshot, or point `--costs` at an artifact whose `graph:` matches.

## Step 3: partition

Two consumers read the snapshot and derive `process_groups:` via
`auto_partition`, greedy descending-coupling fusion, gated by the per-group
compute budget AND the re-levelization spawner-consumability check (see
`docs/multi_process.md`, "Each group owns a contiguous band"), so the emitted
partition is accepted by `graph run`'s multi-process supervisor by
construction:

### Hard constraints: `block` edges are not a cost input

Costs and the budget describe what fusion is *worth*. A `block` edge is
different in kind: it is a **constraint the partition may not violate**, and it
is applied BEFORE any cost is scored.

`#[input(backpressure = block)]` defers the producer's tick while the
consumer's queue is full. The defer state is a mirror the producer's publisher
increments and the consumer's subscriber decrements. On a CO-LOCATED edge that
mirror is a process-local word inside ONE `GraphRuntime`, and a producer in
another OS process holds a different one (or none), so it cannot observe that
queue through it. An UNCREDITED split `block` edge therefore does not degrade
to something weaker: the consumer's worker sees a `block` input on a topic
whose producer is not in its subgraph and **refuses to build**.

The cross-process form of that mirror (a **shared-memory credit word** both
workers operate on, minted per edge by the supervisor) makes a split edge
lossless, and it is admitted at plan time for exactly the shape it can
describe: **one in-graph producer, no non-`block` consumers**. Such a split is
accepted; every other one still meets the refusal above.

The derivation co-locates the whole credited flow regardless: crossing a process
boundary costs a real
hop whether or not the edge is correct, so for every topic that has an in-graph
producer AND at least one `block` consumer, the derivation unions **its whole
flow (all of its producers and
ALL of its consumers, `block` or not) into one group**, on both the cost-fused
and the process-per-node baseline path. `cerulion graph partition` prints the
resulting constraints in the consent preview (every path, `--dry-run`
included), and `cerulion graph run` also logs them at `info`. Consequences
worth knowing:

* **It outranks the budget.** A seeded group is not a fusion candidate, so the
  per-group compute budget cannot veto it. Exceeding the budget is announced at
  `warn` (with `load_ns` reported as a floor; see below), never silently:
  `warn` because `graph partition` (the only verb carrying `--budget-ns`)
  defaults to `cerulion=warn`, so an `info!` there would not print at all.
* **It outranks `isolated:`.** `graph profile` isolates an under-sampled node
  and normally keeps it a singleton; a `block` consumer is by construction the
  slow node the producer is being deferred for, so it is the likeliest member of
  that set. Co-location wins, loudly. An isolated member carries no measured
  cost, so the reported `load_ns` is a FLOOR and the line names the unmeasured
  members rather than inventing a number for them.
* **It can grow the group further.** A seeded group can end up bridged by a
  foreign node, or owning a non-contiguous level band, shapes the
  cross-process barrier cannot represent. The derivation REPAIRS that by
  absorbing the offending node(s) and re-checking, announcing each absorption
  at `warn` (a node the operator never grouped is being written into a group),
  rather than refusing a graph that runs fine under `--single-process`. In the
  limit the group reaches the whole graph, which is the `--single-process` shape,
  but derived and announced instead of discovered from a dead worker.
* **A MIXED topic is co-located WHOLE, sibling included.** A topic with one
  `block` and one `drop_oldest` consumer degrades its `block` consumer to
  `drop_oldest` at runtime, but that degrade happens strictly LATER than the
  build-time check that refuses, and it is decided PER PROCESS from the
  worker's own topology, where a consumer the worker does not own has been
  filtered out. So the co-location covers the non-`block` siblings too: group
  only the producer and the `block` consumer, and that worker reads its local
  flow as all-`block`, INSTALLS the defer, and throttles the producer to the
  `block` consumer's drain rate (starving the sibling) while
  `--single-process` degrades and warns. The mixed-topic warn would fire in
  NEITHER process, so one graph would run two semantics in silence. A
  hand-written partition that splits a mixed topic's sibling is refused too,
  with its own diagnosis (that shape BUILDS; it just runs differently).
* **A `block` input with NO in-graph producer** is untouched here: it is
  refused at graph build with its own message, identically under
  `--single-process`, and there is nothing to co-locate it with.

A **hand-written** `process_groups:` that splits a `block` edge is ACCEPTED
when the edge can be credited (one in-graph producer, no non-`block`
consumers) and refused at plan time otherwise, by `graph run` before any
worker spawns and by `graph levels`. The refusal names the topic, the
producer, the consumer and its input, both groups, the fix, and **which bar it
hit**:

| Shape of the SPLIT topic | Verdict | Why |
|---|---|---|
| exactly one in-graph producer, every consumer `block` | **accepted** | the supervisor mints it a credit word |
| two or more in-graph producers | refused | the word counts ONE producer's outstanding frames; two writers would each spend the other's credit |
| one producer, but the topic also has a non-`block` consumer | refused | `block` is degraded to `drop_oldest` on a mixed topic, so there is no lossless defer left to credit |

The bars are checked in that order, so a topic hitting BOTH is reported as the
multi-producer one: it is the one an operator cannot fix by moving a node.
Note that the third row is about the SPLIT `block` edge on a mixed topic; a
mixed topic's non-`block` sibling being split from the flow is a *different*
violation with its own diagnosis (every worker builds, and one of them
silently installs a defer the monolith degrades).

It is a refusal rather than an automatic repair because the only degrade
available for those two shapes would be silently rewriting `block` to
`drop_oldest`, i.e. losing data the operator asked to keep.

One residual applies to a split edge exactly as to a co-located one: on a
`multi_publisher_topics:` topic the credit word counts IN-GRAPH publishes only, so an out-of-graph
writer can fill the consumer's queue without raising `outstanding`; the defer
then arrives late and the queue can evict. That is a property of `block` on a
listed topic, co-located or split, and there is no detector for it.

### The verb: `cerulion graph partition <NAME>`

```bash
cerulion graph partition perception            # preview + y/N confirm, then write
cerulion graph partition perception --dry-run  # preview only (wins over --yes)
cerulion graph partition perception --yes      # write without the confirm (scripts)
cerulion graph partition perception --costs my.costs.yaml --budget-ns 5000000
```

With a cost snapshot at `graphs/<NAME>.costs.yaml` (or an explicit `--costs`,
which MUST exist: a named-but-missing file is an error, never a silent
fallback) the partition is a cost-aware one, derived by greedy fusion of the
most tightly coupled nodes first; with no snapshot it
is the **process-per-node baseline** (maximal fault isolation, no fabricated
costs). `--budget-ns` caps each group's summed p50. **Default: the
artifact's frozen `derived_budget_ns`** (one core's fair share of the
graph's total compute, computed at profile time), so a cost-driven fusion stays
within that budget. A required co-location (a `block` topic's whole flow) is
seeded before the budget applies and can exceed it; the verb reports such a
group. An explicit `--budget-ns` always overrides the frozen value; an
earlier artifact (or an absent frozen value) falls back to unbounded
fusion with a loud info suggesting a re-profile. The `graph run` default
resolves the budget through the SAME point, so the verb and the run can
never diverge.

The write is **surgical**: only the `process_groups:` block is replaced (or
inserted before `nodes:`), every other byte (comments, formatting, key
order) is preserved, the prior file is backed up to `<file>.bak`, and a
stale `process_group_order:` block is removed (the emitted listing order IS
the rank order). The file is NEVER written without consent: a TTY run shows
the proposed bands + a block-scoped diff and asks y/N; a non-TTY run without
`--yes` refuses loudly. The verb uses **replace-scoped validation** (every
check EXCEPT the partition blocks it is about to replace), so it is also the
RECOVERY tool for a stale or broken `process_groups:` block and for a stale or
broken `level_assignments:` block.

### Cost-aware level refinement: `level_assignments:`

Big-graph e2e latency is level-lockstep wait: a level-`L+1` consumer waits
for EVERY level-`L` node, so a fast chain's latency is bounded by its gating
levels' compute: an expensive 30 Hz camera parked at level 0 inflates a
1 kHz proprioceptive chain that never reads it. With a cost snapshot, `graph
partition` first REFINES the levelization (`refine_levels`, two phases:
expensive low-rate nodes shift LATER within their topological slack, and,
because a DENSE pipeline has no slack, a chain-CASCADE pass that slides a
whole slow chain later uniformly, GROWING the level count as needed. Under
level-lockstep a chain's latency is the sum of the level makespans BEFORE
its sink, so a slow chain marches until it sits at the deepest faster
chain's sink level (leaving every faster chain's waiting span); a cascade
that would push a faster chain's node is refused, so the max-rate chain's
nodes never move; un-costed nodes stay put. A grown count is safe: the
assignment is frozen in the yaml, so every process derives the same barrier
generations from the same block) and bakes the result into the graph yaml as a
top-level `level_assignments:` block (node → level, every node covered),
emitted in the SAME consented rewrite as `process_groups:`, which is banded
over the REFINED levels, so the two blocks are always coherent.

Rules (all enforced + shown in the preview):

* **The block exists only when it changes something.** `refined == Kahn`
  (including every no-costs baseline run) OMITS the block and REMOVES a
  stale existing one: a `level_assignments:` block in a graph file always
  means "these levels differ from Kahn". A hand-written block that happens
  to equal Kahn is removed too (it changed nothing; the preview names the
  removal).
* **Refinement only takes effect PERSISTED.** The runtime, `graph levels`,
  the multi-process planner, and bag replays all consume the WRITTEN block
  through one seam (`resolve_levels`); a plain `graph run` (the mp-default
  preflight) never refines in-memory; in-memory refined levels would band
  groups over levels the yaml doesn't carry, breaking Replay=Live
  derivability. An already-persisted block IS respected (and preserved
  byte-identically) by the run preflight.
* **Hand-editable, loudly validated.** Unknown/missing nodes, non-contiguous
  levels, and trigger edges that stop being strictly level-increasing are
  build errors naming the offenders. `graph partition` REPLACES a
  stale/invalid block (recovery tool); `graph levels` renders which source
  applied (`levels source: level_assignments: block …` vs `derived`).
* **Workers run the same levels.** In a multi-process run each worker's
  sub-config carries the global assignment restricted to its group and
  compressed to the group's 0-based local band (exactly the barrier
  participant-map contract), so a worker can never re-derive levels that
  disagree with the supervisor's plan.

### The `graph run` default

`cerulion graph run` on an UNPARTITIONED graph (Unix, real
clock) derives this same partition **by default** and runs multi-process,
with the same snapshot-or-baseline rule and a consent ladder for persisting
the derivation into the file. See `docs/multi_process.md` ("When does a run
go multi-process?") for the full outcome table (`--yes` / TTY confirm /
decline / no-TTY floor / `--single-process` / `--auto-partition`).

## Knob reference

| Knob | Default | Meaning |
|---|---|---|
| `--duration SECS` | 30 | Observation-window cap; the run stops here even if some node is short of its target (that node is isolated). `0` rejected. |
| `--fires N` | *(omitted = auto-derive)* | Uniform per-node fire-target OVERRIDE: every node gates against the same `N` and the run stops early once all reach it. Omit to auto-derive each node's target from the warm-up (see "How targets are derived"). `0` rejected; auto-derive is selected by OMITTING the flag, not zeroing it. |
| `-o/--out PATH` | `graphs/<name>.costs.yaml` | Artifact location override. |
