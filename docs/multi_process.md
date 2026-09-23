# Multi-Process Graph Deployment: `process_groups`, the Supervisor, and Peer Loss

A "how it works and why" reference. A graph that declares
`process_groups:` in its YAML runs as N OS **processes** (one per group)
instead of one, with the SAME deterministic execution contract as the
single-process monolith: the merged cross-process fire trace is
byte-identical to the monolith's (the determinism firewall, Principle #7).
Multi-process buys you **fault isolation** (a crashed group takes down only
its own nodes) and OS-level resource separation, not latency: intra-process
fusion remains the fastest path.

Related: `docs/multi_publisher_topics.md` (single-writer topics across
groups), the cross-process barrier + deterministic-live clock,
recording/replay,
[`docs/bag.md`](bag.md) (the `coordination` / `trace_format` stamps a bag
declares), [`docs/read_log_forensics.md`](read_log_forensics.md) (reading a
recording's per-edge read log offline).

## TL;DR

- Declare `process_groups:` in the graph YAML, a partition of the graph's
  node ids into named groups. `cerulion graph run <name>` then becomes the
  **supervisor**: it plans the deployment, spawns one worker process per
  group, and joins them under a loud lifecycle contract.
- Workers run in **lockstep**: a shared-memory barrier gates every DAG-level
  boundary, and every worker advances the same deterministic-live logical
  clock quantum, so the merged cross-process fire TRACE is
  replay-deterministic, and so is the DATA on every edge the DAG orders. The
  one edge the DAG does NOT order, a same-level non-trigger cross-group pair,
  is ordered by an extra **mid-level** rendezvous on exactly the levels that
  carry one, **except where the consumer is `block`-involved**: the
  fused block group snapshots after that rendezvous, so those pairs are still
  OS-scheduled and still need one of the workarounds; see
  [Scope of the data guarantee](#scope-of-the-data-guarantee-dag-ordered-edges).
- A **`block` edge is split only when it can be credited**. On a co-located
  edge `#[input(backpressure = block)]`
  defers the producer through a PROCESS-LOCAL mirror, which a producer in
  another worker cannot see. A SPLIT edge instead carries that mirror in shared
  memory, a **cross-process credit word** the supervisor mints per edge, and
  is then lossless across the boundary. One is minted for exactly the shape it
  can describe: a topic with **exactly one in-graph producer** and **no
  non-`block` consumers**. Every other split `block` edge is still refused
  PRE-SPAWN (a multi-producer topic, because the word counts one producer's
  outstanding frames; a MIXED topic, because its `block` consumers degrade to
  `drop_oldest` and there is no lossless defer left to credit), and the refusal
  names which bar it hit. The derived partition still co-locates every `block`
  topic's WHOLE FLOW automatically (crossing a boundary costs a real hop
  whether or not it is correct): its producers, its `block` consumers, and (on
  a MIXED topic) the non-`block` siblings, without which the co-located worker
  reads its flow as all-`block` and installs a defer `--single-process`
  degrades (shown in `graph partition`'s consent preview). One residual is
  worth knowing: on a
  `multi_publisher_topics:`-listed topic the credit word counts IN-GRAPH
  publishes only, so an out-of-graph writer can fill the consumer's queue
  without raising `outstanding`; the defer then arrives late and the queue can
  evict. That is a property of `block` on a listed topic, co-located or split,
  and there is no detector for it. See
  [Hard constraints](auto_partitioning.md#hard-constraints-block-edges-are-not-a-cost-input).
- A crashed worker does NOT (by default) kill the deployment: the supervisor
  drops it from the barrier and the survivors continue **degraded, loudly**
  (`--peer-loss continue`, the default). CI/replay runs can opt into
  fail-loud (`--peer-loss fail`).
- Unix (Linux + macOS): the barrier is a portable POSIX
  `shm_open` `MAP_SHARED` primitive. The wake shape is per-OS (Linux: a
  process-shared futex; macOS 14.4 and later: a bounded boundary spin
  then an `os_sync_wait_on_address` kernel wake, with a chunked ~100 µs
  sleep-recheck as the fallback on older hosts, never a busy-spin). On non-Unix hosts the SAME
  graph runs single-process (monolith fallback) with a loud notice:
  identical results, no process isolation.

## When does a run go multi-process? (the auto-partition default)

`process_groups:` is not the only trigger: an
UNPARTITIONED graph goes multi-process **by default**:

- **Graph declares `process_groups:`** → respected exactly as written
  (the supervisor path below). No derivation, no prompt.
- **No `process_groups:` + Unix + the real (default) clock** →
  `graph run` DERIVES a partition: a cost-aware partition derived by greedy
  fusion when `graphs/<name>.costs.yaml` exists (from `cerulion graph profile`),
  else the **process-per-node baseline** (maximal fault isolation). It then runs the
  consent ladder for persisting the derivation into the YAML:

  | You ran | File | Run |
  |---|---|---|
  | `--yes` | Written (surgical splice, `.bak` backup) | multi-process, derived groups |
  | TTY, answered `y` | Written (same) | multi-process, derived groups |
  | TTY, answered `N` | **Untouched** | multi-process, derived groups held IN-MEMORY (decline ≠ abort; Ctrl-C is abort). Logged at `info`, not `warn`: you were asked and this is what you chose |
  | no TTY, no `--yes` | **Untouched** (the floor: never mutate without consent) | multi-process, derived groups IN-MEMORY + a loud notice naming `--yes` and `--single-process` |
  | `--single-process` | Untouched | the plain MONOLITH: no derivation, no prompt (the opt-out) |

  An in-memory derivation produces EXACTLY the deployment a written file
  would (same plan, ranks, subgraphs; pinned by test).
- **`--auto-partition`** → re-derive even over an existing `process_groups:`
  block: shows the diff vs your block; TTY `y` applies, `N` KEEPS your block
  (decline means "don't change what I wrote"); `--yes` applies; no TTY runs
  the re-derivation in-memory. Uses replace-scoped validation, so it also
  recovers a stale/broken block. Conflicts with `--single-process`.
- **`--time-source virtual`/`external` on an unpartitioned graph** → the
  monolith paths (the default derives on the REAL-clock live path only).
- **Non-Unix** → monolith fallback (below), derivation skipped entirely.

Persist once and forget: `cerulion graph partition <name>` (see
`docs/auto_partitioning.md`) writes the same derivation with a preview +
confirm, after which every run takes the declared-`process_groups:` path.

## Authoring `process_groups:` in graph YAML

> **Don't want to hand-partition?** `cerulion graph profile <name>` measures
> the live graph (per-node p50 tick durations + per-edge fire rates) into
> `graphs/<name>.costs.yaml`, the input of the cost-aware auto-partitioner;
> see `docs/auto_partitioning.md`.

```yaml
prefix: robot
nodes:
  - id: camera
    # ...
  - id: detector
    # ...
  - id: tracker
    # ...
  - id: logger
    # ...

# The partition: every node id in EXACTLY one group.
process_groups:
  sensors: [camera]
  perception: [detector, tracker]
  telemetry: [logger]

# OPTIONAL rank override (see "listed order is the contract" below).
# process_group_order: [perception, sensors, telemetry]
```

Validation (every violation fails loudly; the first three rules are checked at
graph load by `validate_process_groups`, the fourth at plan time by
`plan_deployment`):

| Rule | Enforced at | Why |
|---|---|---|
| Every node id appears in **exactly one** group | graph load | The groups are a partition: an orphan node would silently never run; a double-listed node would run twice |
| Every listed id **exists** in `nodes:` | graph load | Rejects dangling references |
| `process_group_order` (if present) is a **permutation** of the group names | graph load | Every group ranked exactly once |
| Each group owns a **contiguous band** of global DAG levels | plan time | The cross-process barrier supports contiguous splits only; an interleaved partition (a group owning levels {0, 2} while another owns {1}) is rejected |

**Listed order is the contract.** `process_groups` is an ordered map
(`IndexMap`): the **declaration order defines each group's rank** (the
cross-process trace-merge tiebreaker and barrier ordering) unless the
optional `process_group_order` list overrides it. Reordering the YAML
entries changes ranks; treat the listing order as meaningful, exactly like
node declaration order elsewhere in the graph file.

## The deployment model

`cerulion graph run <name>` on a `process_groups:` graph:

1. **Planning build**: the supervisor builds the FULL graph once
   (fail-fast on any validation error before a single process spawns) and
   extracts the global levelization + the graph's tightest timing.
2. **Plan**: one `WorkerPlan` per group: its subgraph, the topics it
   consumes from sibling groups (so the worker's own validation does not
   report a correct cross-group edge as a possible typo), rank, barrier
   participant-map, the shared **handed quantum** (the global tightest
   timing, identical for every worker, so all gating clocks advance in
   lockstep), and the deployment's shared iceoryx2 config: a
   snapshot of the supervisor's **resolved global config** (the **default**
   iceoryx2 namespace; prefix `iox2_` on a config-file-free machine),
   identical for every worker, so cross-group topics connect AND the data
   plane is visible to external tooling (see "SHM namespaces" below).
3. **Spawn**: one `cerulion graph run-worker` process per group (a hidden
   verb; users never invoke it), producer-owning groups first, each gated
   on a READY sentinel so a consumer group never opens a topic before its
   producer created it.
4. **GO gate**: no worker enters its live loop until EVERY worker is
   built + READY. This kills the startup first-sample race: all
   subscribers are connected before the first publish, so the run is
   deterministic **from step 0**.
5. **Lockstep execution**: every worker runs the deterministic-live loop;
   a shared SHM barrier (`MappedBarrier`) gates every global DAG-level
   boundary, and each worker's gating clock advances by the same handed
   quantum per step. The merged trace (by `(step, global_level, rank,
   seq)`) is byte-identical to a single-process run of the same graph.
6. **Join**: the supervisor monitors workers until shutdown (Ctrl-C, a
   clean worker exit, or the peer-loss machinery below). On a clean
   shutdown every worker gracefully leaves the barrier cohort and exits 0
   (no poison stalls).

> **Host tuning for the latency tail:** the millisecond-class MAX you may
> see on an untuned host is dominated by ambient kernel run-queue delay (the
> measured floor/p50/p99 stay in the microseconds). See
> [`docs/deployment_tuning.md`](deployment_tuning.md) for the Linux
> isolation ladder (`isolcpus`/`nohz_full`/`rcu_nocbs`/IRQ steering/SCHED_FIFO) that
> reduces it.

### Scope of the data guarantee: DAG-ordered edges

The barrier gates **level boundaries**, so what it orders is edges that CROSS
one. Read the guarantee in two halves, because they have different scopes:

- **The fire TRACE is byte-identical to the monolith's, always.** Which nodes
  fire, in what order, at what logical time: that is what the lockstep
  quantum plus the level-boundary barrier pin, for any graph and any
  partition.
- **The DATA is deterministic on every edge the DAG orders.** A
  `#[input(trigger)]` edge levelizes its consumer strictly below its producer,
  so the publish lands in an earlier global level than the read and the
  level-boundary barrier separates them. Cross-process is deterministic and
  the recording replays.

The rest of this section is the edge the DAG does NOT model, and how it is
ordered: the shape, the barrier that orders it, and the residual that barrier
carries. Jump to [the barrier](#the-conditional-mid-level-barrier) for the
behaviour on its own.

A plain non-trigger `#[input]` is
a latest-value read; the consumer is fired by something else (a `period_ms`
timer, another input), so the edge does not levelize and producer and consumer
can share ONE global level. In the monolith that is still deterministic: the
level executor snapshots every firing node's non-trigger inputs BEFORE any
node in the level ticks, so the consumer always reads the PRIOR step's frame.
Split across two groups there is no such ordering: the consumer's snapshot and
the producer's tick run concurrently, between the same two barrier
generations, so **which frame the tick pairs with is decided by OS
scheduling**. That pairing is not recorded and cannot be, and a monolith
replay re-derives it under the snapshot-then-tick rule, so a run that paired
the other way replays as a data violation.

**What that costs a recording.** Two runs of such a graph can record
different frames on the consumer's output, so the nondeterminism is in the
LIVE run rather than in replay. The fire TRACE stays identical throughout:
this shape yields a **frame-content divergence**, never a **fire-schedule**
one, so the trace guarantee above holds. The pairing is what has to be
ordered, which is what the rendezvous below does. Marking the edge
`#[input(trigger)]` or co-locating the two nodes removes the shape entirely.

#### The conditional MID-LEVEL barrier

The level is not one indivisible unit of work. `run_level` is split into a
**snapshot phase** (drain, decide, freeze every firing node's non-trigger
inputs) and a **tick phase**, and on the levels that need it the deployment
rendezvouses BETWEEN them:

```text
 unflagged level                 flagged level
 ───────────────                 ─────────────
 drain / decide / snapshot       drain / decide / snapshot
 fuse-block / tick               ══ barrier ══      <- every group has snapshotted
 ══ barrier ══                   fuse-block / tick
                                 ══ barrier ══
```

After the mid-level generation opens, every group's step-start snapshots are
provably taken, so a same-level cross-group producer's tick cannot race
a consumer's read: the monolith's guarantee, held across processes. The
diagram shows the one exception in its own shape: the block fuse sits AFTER the
rendezvous, so a `block`-involved consumer's snapshot is not covered by it (the
[residual](#the-conditional-mid-level-barrier) below).

**CONDITIONAL, so most graphs pay nothing.** The supervisor classifies these
pairs at plan time, warns about them, and stamps the classification into every
`WorkerPlan` as a per-global-level flag vector. Only a level
carrying a split same-level non-trigger edge takes the extra rendezvous, and the
run's law is `global_levels + flagged_levels` generations per step,
readable on each worker's build line (`mid_level_barriers=`,
`generations_per_step=`). A graph
with no such pair takes no extra rendezvous, and runs one generation per level.

Every participant crosses the extra rendezvous, owner of that level or not
(the same rule the end-of-level boundary already follows), and the flags are
computed ONCE by the supervisor and cloned to every worker. Two workers
disagreeing about one level would desynchronise the shared generation counter
for the rest of the run, so a worker never re-derives them and a flag vector
whose length disagrees with the participant map is refused at install time.

**Cost:** one extra rendezvous per flagged level per step, paid only by the
levels that carry the shape. `barrier_test.rs` carries a print-only A/B of the
two rendezvous tiers for measuring that cost on your own host.

**RESIDUAL: a `block`-involved consumer is still unordered.** The fused block
group (`evaluate_nodes_fused`) interleaves decide+snapshot+tick
per node in graph order, so its per-node snapshot cannot be hoisted into the
snapshot phase; the mid-level rendezvous therefore sits BEFORE the fuse. A node
that declares a `block` input, or publishes onto a `block` topic, takes its own
non-trigger snapshot after the rendezvous. (`block` inputs themselves are never
snapshotted, so this is exactly "a block-involved node's OTHER plain
non-trigger inputs".) The plan-time report names this residual per finding, and
for it the workarounds below still apply.

**The residual is reachable on the default path.** A graph
whose `block` edge crosses a group boundary builds, and the derived partition
creates a fused block group for every `block` topic in the graph, so a
`block`-involved consumer's other plain non-trigger inputs land in this
residual on a derived partition as well as on a hand-written one. The
workarounds below apply to it.

Any of these also orders the pair, and remains the better shape in some cases:

| Workaround | What it changes |
|---|---|
| Mark the edge `#[input(trigger)]` | It becomes a DAG edge, so the level-boundary barrier orders it and no extra rendezvous is needed. Usually the better control shape too; this is how `examples/obstacle_avoidance` is wired. |
| Co-locate both nodes in one `process_groups:` group | One worker, so the monolith's snapshot-then-tick order applies directly. |
| `cerulion graph run --single-process` | The plain monolith. |

**Where it is pinned.** The ORDERING guarantee is
`crates/cerulion_core/tests/mid_level_barrier_iox2_test.rs`: two `period_ms` nodes at
one level joined by a plain `#[input]`, driven under one barrier, with the
interleave fixed by the barrier's own `peers_waiting` signal so BOTH arms are
deterministic and the pair is one flag apart. Its control reproduces the
unordered pairing on demand. The PLUMBING (classify → stamp → serialise →
deserialise → install)
is `crates/cerulion_cli/tests/mp_split_pair_e2e_test.rs`, over the real binary, which
verifies that the extra generation does not desynchronise a real deployment
(record → `bag play --resim all --verify` → exit 0) and that two live runs
record identical frames. The ordering guarantee itself is verified by the
first test, not by that e2e.

### Networking: the gateway

A multi-process run is network-viewable exactly like a monolith. The
**workers carry NO zenoh session**: they publish into and read from shared
memory only. The deployment's produced topics reach the network through a
GATEWAY, which taps them on the deployment's iceoryx2 namespace.
One robot = one network peer (Principle #8).

- **Which gateway you get follows the posture.** A run with NO `network:`
  block is PERMISSIVE (every produced topic is announced + egressable), and on
  Unix the supervisor registers the deployment's egress plan with the
  machine's shared `cerulion-netd` daemon, forwarding the supervisor-minted
  worker namespace so netd's gateway taps the right shared memory. An explicit
  block is STRICT (the declared allow-list): verbatim locators and an
  allow-list cannot be represented on that shared session, so a Strict run is
  routed to a per-run gateway child (`cerulion graph run-gateway`, hidden)
  that honors the block. A permissive run falls back to the same child,
  loudly, when netd is unreachable or resolved a different namespace.
- A per-run child is spawned AFTER the GO gate (the topics exist)
  and only while the run is live; a Ctrl-C'd spawn phase skips it.
- The gateway is NOT a DAG worker: a per-run child is held in a separate
  guard, never in the worker set, so its death never trips the `--peer-loss`
  machinery (it degrades the run to local-only with a loud warn, exactly like
  the monolith arm). `--network off` / `CERULION_NETWORK=off` opt the whole
  run out.
- Recording composes: `--record` on a networked multi-process run keeps the
  egress up while recording (the record + declared-`ingress:` refusal
  is the only exception). See `docs/networking.md` for the full
  gateway model.

### SHM namespaces: the data plane is the DEFAULT namespace

The multi-process **data plane** (every topic the workers publish and
subscribe) lives on the **default iceoryx2 namespace**, the same
`iox2_`-prefixed namespace a single-process `graph run`, `topic
echo/hz/list/info`, and every external iceoryx2 process use. That is what
makes a default multi-process deployment interoperate:

- `cerulion topic echo/hz/list/info` see (and can tap) a live mp run's
  topics with zero configuration;
- **cross-graph absolute topics** work, e.g. two graphs both publishing
  `/tf` (a `multi_publisher_topics` accumulate-all topic) connect whether
  each graph runs single- or multi-process;
- **external absolute-source publishers** (a non-Cerulion process feeding
  a graph's `source: /ext/cam` input) attach exactly as they do against a
  single-process run.

Only the deployment's **infrastructure** stays run-scoped/isolated:

| Object | Name shape | Scope |
|---|---|---|
| Cross-process level barrier | `cerdep_{graph}_{nonce}` (POSIX SHM) | per run |
| Trace/recording rings | `cer_rec_*` / `cer_rg_*` (POSIX SHM) | per run |
| Doorbells | `/cer_db_<user>_*` | per `$USER` |
| Supervisor **planning** namespace | `cer_p_{hex}` (iceoryx2) | per run: the planning build attaches real single-writer publishers for every graph-owned topic; on the shared data-plane namespace it would collide with the workers' own |

Two consequences to know:

- **Concurrent runs of the SAME graph collide LOUDLY.** Two simultaneous
  `graph run` invocations of one graph target the same topic names on
  one namespace, and the second is refused by the single-writer publisher
  checks (the designed protection, the same guard that stops two graphs
  from silently double-driving `/cmd_vel`).
- **Stale SHM from a crashed run is visible.** A SIGKILLed run's leftover
  segments share the default namespace; standard sweep discipline
  applies: `cerulion clean` (dead-node-only cleanup; live siblings are
  untouched), iceoryx2's own stale-resource cleanup on next attach, or
  remove `/dev/shm/iox2_*` for a hard reset.

> **Config-file note:** iceoryx2 honors an operator-installed **global
> config file** (e.g. `/etc/iceoryx2/iceoryx2.toml` or a
> `~/.config`-level equivalent). The supervisor **snapshots its resolved
> global config at mint time** and ships that exact `Config` to every
> worker and to bagd (the same resolution `topic echo/hz/list/info` and
> the cleanup sweep use), so all components agree on one namespace
> whatever the file says (on a config-file-free machine: `iox2_`). The
> residual caveat is only that the snapshot is taken **once at launch**:
> a config file edited mid-run affects processes started afterwards, not
> the running deployment's workers.

### Barrier boundary spin (default-on)

At every level boundary each worker waits on the shared barrier. The
blocking tier is a process-shared futex on Linux and, on macOS 14.4 and
later, an `os_sync_wait_on_address` kernel wake; a chunked ~100 µs
sleep-recheck is the fallback on older macOS and under
`CERULION_BARRIER_OS_SYNC=0`. By default that wait first **spins for a
bounded budget: 20 µs on the kernel-wake tiers, 150 µs on the macOS
sleep-recheck fallback** (that fallback observes arrivals with up to
~100 µs skew, which a 20 µs spin would miss). The bounded spin rides
through the lockstep rendezvous instead of paying a kernel sleep/wake
round-trip at every boundary, then falls back to the blocking tier.

`CERULION_BARRIER_OS_SYNC=0` selects the sleep-recheck tier on macOS, and
with it the 150 µs spin default. `CERULION_BARRIER_SPIN_US` overrides the
budget on every tier; `=0` is the kill switch: it restores the legacy 50-iteration
pre-block spin + futex path (a hard-bounded read loop with no `Instant`
reads, then the unchanged kernel block), **not** a pure futex wait, for
power-sensitive deployments. Values above **100ms** are clamped to 100ms
with a loud warning (still far below the ~5s barrier
boundary timeout), so a fat-fingered budget can never turn the bounded
spin-then-block into a busy-spin for the whole boundary window. (Even at
the 100ms ceiling every cycle still reaches a real kernel block within
~110ms, but during a stalled-peer teardown window such a misconfigured
budget spends most of its wait spinning; the 20µs default spends almost none.) Determinism
firewall: the bounded spin-then-block changes only WHEN a waiter proceeds,
never the fire set/order.

## The per-edge credit word

A `block` edge is lossless because the producer reads the consumer's `outstanding` counter
before it fires. Split the two across process groups and that read is gone, so such
a split is accepted only for a **creditable** edge
(exactly one in-graph producer, no non-`block` consumers), which gets a shared word to
read instead.

**One word per EDGE, not per topic.** The identity is
`credit_edge_id(topic, consumer_node, consumer_input)`
(in `crates/cerulion_core/src/credit.rs`, like every credit symbol named here), so two
`block` consumers of the same topic get two words; the segment name is
`credit_shm_name(ns, id)`. The payload is a `CreditShared`: the same `outstanding` mirror
the co-located case keeps on the heap, placed in shared memory instead.

### Lifetime: the supervisor owns it, the workers borrow it

1. **The supervisor MINTS it**, before any worker spawns: `MappedCredit::create_owned` unlinks
   the name and `O_EXCL`-creates it (a unix arm and a fallback arm; the unix one delegates to
   `shm_map::create_exclusive` in `crates/cerulion_core/src/shm_map.rs`, which carries the
   `unlink` and the `O_CREAT | O_RDWR | O_EXCL` open), so a stale
   segment from a dead run can never be adopted.
2. **It is STAMPED into EVERY worker plan**, not only the ranks that touch the edge
   (`stamp_credit_edges` in `crates/cerulion_cli_engine/src/graph_cmd.rs`;
   pinned by `stamp_credit_edges_writes_the_same_list_into_every_worker`). That is deliberate:
   every worker gets the WHOLE list so no two can disagree about the edge set, and each then
   opens only its own halves: entries naming neither of a rank's roles are skipped at
   consumption time (`credit_role_for`, whose comment carries the rationale). A uniqueness check per group
   (`check_credit_edges_unique`) makes a plan naming one edge twice a refused build
   bug rather than a tolerated one.
3. **Each worker OPENS it, never creates it**: `MappedCredit::open_unowned` is `O_RDWR` with
   no create bit. The supervisor mints every segment before it spawns a worker, in the same
   function of `graph_cmd.rs`, so the ordering holds by construction. A worker whose exec
   startup outruns that mint finds the `/dev/shm` segment briefly absent (a `NotFound`) and
   waits on a bounded retry rather than creating it, because a silent create would hand the
   pair two different pages that each look fine.
4. **The supervisor holds the owners for the whole run** (`credit_owners` in `graph_cmd.rs`;
   the count is logged as `credit_words`). They are MOVED into `CreditDeathWatch::arm`,
   and the `Vec<ChildGuard>` is declared AFTER that, so by Rust's
   reverse-declaration drop order the child guards drop first and the segment outlives every
   worker that maps it.

An unarmed-but-full-size segment is REFUSED rather than handed over: without that check the
peer receives a live handle whose `outstanding`/`depth`/`epoch` are all zero, i.e. a producer
whose `is_full` holds unconditionally, forever, which is a silent wedge. The refusal is
pinned by
`a_full_size_but_unarmed_segment_is_refused_rather_than_wedging_its_producer`.

### The pre-spawn reconciliation refusal

The supervisor builds the topology TWICE before it spawns anything (once from the node
libraries as loaded, once from source metadata) and refuses if they disagree
(both builds are in `graph_cmd.rs`; the refusal itself is `reconcile_credit_edges`
in `crates/cerulion_cli_engine/src/multiprocess.rs`). Be precise about WHAT is compared: the derived credit-edge KEY SET, where a
key is `(topic, consumer_node, consumer_input)`, the edge's identity, with `depth` deliberately
EXCLUDED. So the refusal catches an edge that
appears, disappears or moves between the two views; it does not police a depth that changed. A
stale cdylib whose ports no longer match its source is caught before a credit word is minted for
an edge that is not there.

### Reading a credit-death warn

Under `--peer-loss continue` the survivors keep running when a worker dies. If the dead rank
was a credited edge's CONSUMER, the producer is now gating on a mirror nobody will drain, so
the supervisor says so. The head is a `warn!` (emitted by `CreditDeathWatch` in `graph_cmd.rs`) with these fields:

```
group  node_id  topic  consumer_input  dead_rank
deferred_producer_ranks  deferred_producer_groups  total_failures
```

Read it as: **`topic` + `consumer_input`** identify the edge; **`dead_rank` / `group`** say who
died; **`deferred_producer_groups`** names the groups whose producers are now stuck on it (real
group names, one per producer rank, not the batch; a rank whose group cannot be resolved
renders `<rank N>` rather than being dropped);
**`total_failures`** is a running, unconditional count that recovery never resets.

It is flood-latched on the shared `FailureRegimeLatch`: one loud head, `debug!` repeats,
and a `warn!` re-announcement at each decade of the running total. If a
producer this line reported as deferred-but-alive later dies, a RETRACTION `warn!` corrects the
record rather than the earlier line being amended, since it is already in the log. Under
free-run the departure line names its dead groups (`groups=`).

### The producer parks on the credit word

A producer deferred at the credit gate has nothing to wait ON: no publisher listener is
attached to a WaitSet anywhere, so its park has no event source of its own beyond
the recheck timer. **The credit word IS the wake channel.** The consumer's drain
(`CreditShared::record_drained`) bumps the word's `wake_seq` and, only when the `parked`
mask says somebody is waiting, pays a kernel wake; the producer's live loop carries a FIFTH
wake predicate beside listener / doorbell / external-fd / barrier, and blocks on that word
rather than sleeping out its slice.

The predicate is gated on **was this edge FULL at park entry**, which is the whole design.
An absolute "the edge has room" test reads true on every recheck of a producer that is not
blocked: an unbounded tight loop. A bare "the epoch moved" test wakes an unblocked producer
once per peer drain, so a 1 kHz consumer would wake it a thousand times a second for nothing.
Gating on was-full-at-entry means credit contributes **zero** wake sources unless this
producer is actually waiting on one.

It rides `MAPPED` edges only. On a co-located edge both ends are nodes of the same runtime,
so a loop that blocked waiting for that word would be waiting for a drain only it can
perform; the watch list simply never contains a local word, which is also what keeps every
single-process run byte-identical.

Attribution is a counter, not a wall: `wakes_credit` on the `run_live` park-telemetry line
(and `park_wakes_credit_count_for_test`) says the peer's drain ended the park. A fire that
arrives with that counter unmoved and `wakes_timeout` bumped came from the recheck cadence,
which is how the tests tell working wake plumbing from absent wake plumbing, since the
producer fires eventually either way.

A producer whose edge-local slot is past the 32-wide `parked` mask keeps the slice cadence:
correct, bounded, and short only the latency win, with one loud warn naming the degradation.
The supervisor's sweep clears a dead producer's stale bit, so a parked producer never
waits on a bit nobody will clear.

**A consumer that is alive but has STOPPED draining is also reported.** Occupancy cannot
tell that apart from a merely busy consumer (both pin the word full), but the wake epoch
can: a consumer draining at any rate at all moves it on every freeing drain. So a producer
that has been deferred across a thousand consecutive evaluations with **no credit motion at
all** emits a `warn!` naming the topic, the consumer node and its input, flood-latched on the
shared `FailureRegimeLatch` (one loud head, `debug!` repeats, a re-announcement at each
decade) and closed by an `info!` when credit moves again. The threshold is a COUNT, not a
duration, so the decision is derivable from the recording rather than from how loaded the machine
was; the cost is that detection latency scales with the producer's own rate: about a
second at 1 kHz, about a minute and a half at 10 Hz. That is acceptable because the condition
is permanent: a stalled consumer does not resume on its own, so the warn is
late, never wrong.

This is the second never-silent carrier for the wedge class, and it covers what the first
cannot. The supervisor's `CreditDeathWatch` above reports a consumer whose PROCESS DIED; this
reports one that is alive and idle. Between them both halves of a permanently-deferred
producer are named.

Pinned end to end over REAL processes by `crates/cerulion_cli/tests/credit_death_e2e_test.rs`: the
loud head and its fields, the ABSENCE of a repeat, the retraction, the free-run `groups=`. The
latch's DECADE re-announcement is pinned by an in-crate unit arm in
`graph_cmd.rs` instead, not by the e2e file. The word's own semantics: `credit_test.rs` and
`credit_block_iox2_test.rs`.

## Peer loss: `--peer-loss <continue|fail>`

What happens when a worker process **dies** (crash, OOM-kill, `kill -9`)
while the deployment is live:

| Policy | Behavior |
|---|---|
| `continue` (**default**) | The supervisor logs a loud error naming the lost group, drops the dead peer from the shared barrier (so the survivors do not stall at the next level boundary), and keeps the survivors running **degraded**. The run still exits 0, unless EVERY worker crashed (no survivors), which is an error. Near-simultaneous deaths are dropped as a batch (one disambiguation grace total). |
| `fail` | Any worker death stops the WHOLE deployment: the supervisor SIGKILLs every sibling and returns a loud error (non-zero exit). Deterministic: choose this for CI and replay-comparison runs. |

The flag wins outright; when absent, the hidden `CERULION_MP_PEER_LOSS`
env seam (a test seam, not a documented surface) is consulted, else the
default is `continue`.

**Replay caveat:** a degraded-continue run is faithful up to the
fault instant. A recorded run captures the
departure: the supervisor writes a Departure record naming the lost group's
rank into the bag (see "Recording a multi-process run" below), so the bag
documents WHERE the cohort degraded. The post-fault trace is still
live-only evidence (the crash instant itself is not re-executable); use
`--peer-loss fail` when you need a run that either completes
identically-replayable or stops.

## Recording a multi-process run (`--record`)

`--record` on a `process_groups:` graph records the
MULTI-PROCESS run into **one bag**. The supervisor stamps a per-worker
trace-ring tag into every worker plan (each worker creates its ring, header
rank = its group's declaration rank, manifest = its subgraph's node ids,
BEFORE signaling READY) and creates its own **departure ring**.
That stamping is NOT what `--record` buys: every multi-process run does it,
so an unrecorded run's Flashback capture is re-executable too (`--no-rings`
declines it). What `--record` adds is the CONTINUOUS bag: it spawns ONE `bagd`
that drains every worker ring plus the departure ring into a single `.mcap`.
bagd's taps-ready handshake completes BEFORE the GO sentinel releases the
workers, so step-0 publishes and step-0 trace records are in the bag.

**Rank provenance.** bagd stamps each ring's header rank into every trace
record's `reserved` field as it writes the bag (the ring-side 40-byte record
format is unchanged; rank lives in the ring header on-ring). Per-rank
manifests land as bag attachments: `__cerulion/trace_manifest_rank{N}.json`
(that group's subgraph node ids, config order). The departure ring's
manifest is `rank4294967295` (`u32::MAX`, the reserved sentinel a worker
rank can never take) with an empty `node_ids`.

**Quantum-timed, not wall-faithful, under the default barrier lockstep.**
An mp recording's workers advance their gating clocks by the SAME handed
global quantum in barrier lockstep; that lockstep is the whole point of the
split, and it is what makes the merged `(step, global_level, rank, seq)`
trace replay-deterministic. The single-process recording's wall-following
property (wall-faithful `fire_time_ns` jitter) deliberately does NOT carry
over: a wall-following worker clock would advance by that worker's private
jitter and desync it from its peers. Per-step boundary times are therefore
EQUAL across ranks. Want the wall-faithful recording? Run
`--single-process --record`, the single-process path.

**Under the `CERULION_EXECUTION_MODE=free_run` opt-in** the
paragraph above does not apply: there is no barrier and no handed gating
quantum, and each rank records its OWN wall-faithful timeline, a controlled
clock placed at a shared `real_ns()` epoch (read by every rank at its live
loop's clock anchor, after GO) that then follows the wall, so
per-step boundary times DIFFER across ranks and carry each rank's real
jitter. The bag's `coordination` stamp (`lockstep` / `free_run`) and the run
directory's `gating` label (`quantum` / `recorded_wall`) say which shape a
bag is. The default is lockstep.

**Departure records.** Under `--peer-loss continue`, each lost worker gets a
Departure record (`record_type` 2) on the supervisor's departure ring:
`node_idx` = the departed group's rank; `step`/`fire_time_ns` are
deliberately zero: the supervisor cannot know the workers' current step,
and fabricating one would be fake data (the contract needs the departure
PRESENT + IDENTIFIABLE, nothing more). Under `--peer-loss fail`, the
triggering death is recorded and the bag is finalized BEFORE the abort, so
the aborted run's bag stays diagnosable. Replay does not act on a Departure
record: it does not refuse to re-execute past a departure, so read the bag's
departure records yourself when interpreting a degraded run.

**Interleave.** The drain-order interleave ACROSS rings is explicitly NOT
part of the determinism contract; replay canonicalizes records by
`(step, global_level, rank, seq)`.

### The bag says which coordination it was recorded under

The two paragraphs above describe a **lockstep** recording. The contract is
stamped rather than assumed: `__cerulion/recorder.json` carries a
`coordination` key, and re-execution applies the contract the bag names.

| Stamp | Recorded under | What re-execution applies |
|---|---|---|
| absent | Any bag recorded before the stamp existed | **Lockstep, INFERRED**: the verdict says so out loud (`coordination: lockstep (inferred: no coordination stamp)`), because an absence is not a claim |
| `lockstep` | The barrier path, and every MONOLITH recording (a monolith is the degenerate one-rank lockstep timeline) | Lockstep: one authoritative clock, cross-rank boundary equality, one first boundary to anchor a mid-run resume |
| `free_run` | The free-run path: per-rank wall-faithful boundary streams sharing only the GO epoch. Reachable ONLY by opting in (`CERULION_EXECUTION_MODE=free_run` on a multi-process run); lockstep is the default | **Per-rank re-execution** (below). The bag also stamps a `trace_format` past 3, so an older binary refuses it instead of mis-applying the lockstep contract (a bag stamps 6 by default, or 5 when it is recorded with `CERULION_READ_LOG_FOLD=off`, see [`docs/bag.md`](bag.md)) |
| anything else | Nothing this binary knows: a newer recorder, or a hand-edited bag | **Refused, exit 2**, naming the value. Never inferred to lockstep: an unknown coordination is exactly the case where guessing is a silent mis-replay |

**Why an unknown value is refused rather than tolerated.** A format-3 reader
handed a free-run stream would apply cross-rank boundary equality and
single-anchor resume to a stream built to violate both, and every record in it
decodes cleanly, so the failure would be a confident wrong answer, not a parse
error. The greater-than gate on `trace_format` plus the closed `coordination`
vocabulary is what makes it loud.

Two properties invert with the mode, and they are the reason the stamp exists:

| | `lockstep` | `free_run` |
|---|---|---|
| Per-step boundary times | EQUAL across ranks (the handed quantum) | Per-rank and wall-faithful, sharing only the GO epoch |
| Mid-run resume | Anchors on the ONE first recorded boundary | **Refused loudly** if the recording begins mid-run: there is no single first boundary, k clocks cannot be placed off one value, and a stamp compared against it crosses per-rank clock domains. Per-rank anchors are not supported |

**Recording a `free_run` bag is opt-in.** The reader and the per-rank
executor handle such a bag with no switch; the recording arm is behind the execution-mode
variable: a multi-process `graph run --record` under
`CERULION_EXECUTION_MODE=free_run` stamps `free_run` and records each rank's
own wall-faithful timeline from the shared epoch. Without the variable every
`graph run --record` bag stamps `lockstep`.

## Replaying a multi-process recording

`cerulion bag play <bag> --resim all --verify` re-executes a multi-process
recording the way its `coordination` stamp says it was taken.

**A lockstep bag replays byte for byte**: the monolith-with-one-clock
model, rank-0-authoritative re-advance, cross-rank boundary equality, the k-way
trace merge. That covers a bag recorded before the `coordination` stamp
existed: an absent stamp is read as lockstep, and says so in the verdict, while
a free-run bag is the one that stamps a `trace_format` past 3, so an older
binary refuses such a bag instead of applying the lockstep contract to it.

**A free-run bag is re-executed PER RANK.** One runtime per rank, one at a time,
each driven to its own recorded boundary targets:

- **Sequential, one runtime at a time, over ONE transport.** The passes run in
  rank order; each stands up its own `GraphRuntime` on the process's single
  `TransportManager` and drops it before the next begins. That drop is
  load-bearing rather than tidy: a rank's graph-owned topics are provisioned
  single-writer, so the next pass can only take an injection publisher on a
  cross-rank topic once the producing rank's own publisher is gone. Because the
  passes are sequential there is no second `TransportManager` and therefore no
  Principle-#8 exemption, and no k-clock multiplexing scheduler.
- **Injection is re-keyed per CONSUMING rank.** `inject_up_to(target)` has no
  single "the target" once each rank has its own clock, so each pass opens its
  own injectors (for the external sources THAT rank reads, and for the topics
  it consumes from another rank) and drives them off ITS OWN boundary
  targets.


- **Fires come from the recorded trace.** Re-derivation is the verifier's job,
  never the fire driver's: a cross-process `block` producer's pre-fire gate
  WOULD read cross-rank occupancy at instants nothing records, so an
  independently re-derived schedule could legally differ from the recorded one
  and report a fire-schedule divergence on a candidate that changed nothing.
  (Such a bag CAN exist: plan time admits a creditable split,
  so `graph run --record` of one produces a bag carrying a cross-process
  `block` edge. Replay does not accept it: `replay_engine` builds no
  credited-edge set, so the consumer's rank sees a producer-less `block` topic
  and refuses at exit 5.) In replay that
  gate is demoted to a verifier ASSERT on the partial order the read log does
  record.
- **Cross-rank edges are served from the RECORDED frames**, injected through the
  real accounting drain (never a bypass; a bypass would silently lose
  `BackpressureEvent` dispatch) and steered by the consumer's own read log.
- **So a divergence LOCALIZES to the producing rank.** The verdict reads "rank A
  produced different bytes" rather than a cascade of consumer-side symptoms in
  every rank downstream of it. A cross-rank edge-read divergence is impossible
  by construction, because the input side of every cross-rank edge is the bag.

### What you see on a divergence

Divergences are named by descriptive phrase, never by bare exit code, and the
same three phrases appear in the verdict header, the `--report` JSON and the
verifier's own output:

| Phrase | What it means | Attribution |
|---|---|---|
| **fire-schedule divergence** | The SEQUENCE of node fires differs from the recording: a node fired when it shouldn't have, didn't when it should, or out of order | A NODE (and, on a multi-rank bag, its rank) |
| **frame-content divergence** | The node fired exactly as recorded and produced different BYTES | A NODE |
| **edge-read divergence** | One consumer's one input READ A DIFFERENT FRAME than recorded (`fusion.lidar read seq 47 where the recording says 46`) | An EDGE, the attribution neither other class can give |

They compose: one real bug commonly trips more than one (a stale read → a
different output → a downstream node firing differently), and precedence stays
root-cause-first. On a free-run bag the localisation above is what keeps that
chain SHORT: the bag feeds every cross-rank input, so the chain cannot cross a
rank boundary.

**A known limitation:** a candidate whose fire behaviour
differs ONLY through a cross-rank interleave does not surface as a
fire-schedule divergence; it surfaces as a frame-content or edge-read
divergence instead. Divergence does not PROPAGATE across ranks.

For reading a recording's per-edge read log WITHOUT re-executing it, see
[`docs/read_log_forensics.md`](read_log_forensics.md).

## Platform matrix

| Host | `process_groups:` graph |
|---|---|
| Linux | Multi-process (supervisor + workers); futex-woken barrier + CPU-park primitives |
| macOS | Multi-process (supervisor + workers). Same POSIX `shm_open` `MAP_SHARED` barrier; the wait is a bounded boundary spin then an `os_sync_wait_on_address` kernel wake on macOS 14.4 and later, or chunked ~100 µs sleep-rechecks below that and under `CERULION_BARRIER_OS_SYNC=0` (no futex/UMWAIT/WFE on this OS, and never a busy-spin). Linux-only tunings (C-state cap, CPU pinning) degrade gracefully. |
| other (non-Unix) | **Monolith fallback**: the graph runs single-process with a loud notice. Results are identical (determinism firewall); you lose only process isolation. |

`--single-process` forces the monolith path on ANY host (useful for
debugging a multi-process graph in one process, or for an
externally-clocked run; see below).

**macOS wake-latency tier:** any host without a CPU monitor-wait
primitive (macOS included, whether running the real multi-process split
or a `--single-process` monolith) gets a live-loop park via the
**recheck-nap tier**, on by default for live runs. Each ~100 µs nap is an
`os_sync_wait_on_address` timed wait on macOS 14.4 and later, and a plain
sleep-recheck below that or under `CERULION_PARK_OS_SYNC=0` (never a
busy-spin). Measured on macOS: a stable and lower median wake latency than the
plain blocking wait, whose median was unstable from run to run. Opt out with
`CERULION_MONITOR_WAIT=0` or `--no-monitor-wait`; see "Live-loop tuning" under
"Environment variables" in [`docs/user-api.md`](user-api.md).

## Flags

```bash
# Default: multi-process on Linux/macOS, peer-loss=continue, 100k-entry trace ring.
cerulion graph run perception_stack

# CI: any worker death stops the deployment (deterministic).
cerulion graph run perception_stack --peer-loss fail

# Debug the same graph in ONE process (works on macOS too).
cerulion graph run perception_stack --single-process

# A larger observability window (entries; 0 is rejected, see below).
cerulion graph run perception_stack --trace-limit 500000
```

| Flag | Notes |
|---|---|
| `--peer-loss <continue\|fail>` | Applies to every multi-process run, including one whose groups were derived automatically. A single-process run warns once that the option does not apply. Default `continue`. |
| `--single-process` | Forces the monolith even with `process_groups:` (info log names the override). On a graph WITHOUT `process_groups:` it opts out of the multi-process auto-partition default entirely: no derivation, no confirm, the plain monolith run (info log). |
| `--trace-limit <N>` | Caps the in-memory fire-trace ring (default 100 000 entries, stamped into every worker). The trace is a bounded observability window, NOT the replay record (recording is the bag's job). Unbounded growth costs ~2 GB/h at 1 kHz × 10 nodes, so `0` is rejected at parse; there is no unbounded escape hatch. |
| `--record[=DIR]` | Records the MULTI-PROCESS run into ONE bag: the run's per-rank trace rings + the supervisor's departure ring, which exist on every multi-process run, recording or not (see `--no-rings`), all drained by one `bagd`; rank-stamped provenance in every record's `reserved`. Quantum-timed under the default barrier lockstep (see "Recording a multi-process run" above); under the `CERULION_EXECUTION_MODE=free_run` opt-in each rank records its OWN wall-faithful timeline from a shared epoch; the bag's `coordination` stamp and the run directory's `gating` label say which. `--single-process --record` takes the single-process wall-faithful path instead. Unix-only; requires the live clock. |
| `--no-rings` | Declines this run's per-rank SCHEDULER-TRACE rings. They are provisioned by DEFAULT on every multi-process run so that a Flashback capture of ANY serving graph can be RE-EXECUTED (`cerulion bag play --resim`) rather than only a run somebody decided in advance to record; this is the opt-out for a memory-tight robot. Cost declined: ~40.06 MiB APPARENT per rank (`65_600 + 2^20 × 40` = 42,008,640 B) plus one 106,560 B departure ring; the segment is `ftruncate`d rather than written, so it costs a page at first and converges on the full figure only as the ring fills. It ALSO stops the Flashback window recorder being started for this run: with no trace rings nothing captured could be re-executed, so the run takes NO captures rather than frames-only ones (`CERULION_FLASHBACK=off` is the SEPARATE, orthogonal switch for the state plane + anchors). **Conflicts with `--record`** at parse time: `--record` requires the scheduler trace, because that is what a deterministic re-execution reads. On shapes that mint no ring anyway it is NOT a no-op: a `--single-process` or `--time-source external` run still has its window recorder stopped by it, so the run takes no captures. Each says at launch which it was. (`cerulion ros2 attach` and `node run` define no such flag at all.) NOT `--trace-limit`, which caps the IN-MEMORY fire-trace ring above; these are the SHARED-MEMORY rings a recorder drains. |
| `--time-source external` | **Rejected** for multi-process: every worker runs on the real clock (barrier-gated lockstep by default, or free-run (`CERULION_EXECUTION_MODE=free_run`) with each rank on its own wall-faithful clock) and an external time master driving N separate processes is unspecified either way. Drop the flag, or use `--single-process`. |
| `--time-source virtual` | Ignored with a loud warning: multi-process workers always run on the real clock (barrier lockstep or free-run). With `--record` it is rejected outright: recording requires the live clock (`--time-source real`, the default). |

## Exit-code contract

| Exit | Meaning |
|---|---|
| `0` | Clean shutdown (Ctrl-C or a clean worker exit → coordinated drain, every worker exits 0), OR a **degraded** `continue` run where at least one worker survived (the degraded summary is logged loudly). |
| non-zero | Fail-loud: a worker death under `--peer-loss fail`; ALL workers crashed under `continue`; a refused run (validation error, `--time-source external` × multi-process, a `HostDriven` external node on the live path); or an internal supervisor failure. |

Individual **workers** exit `2` when terminally poisoned by a barrier
boundary timeout (a peer stalled/crashed past the ~5s rendezvous deadline);
the supervisor's drain machinery tolerates and reports that; it never
surfaces as a supervisor success/failure on its own.
