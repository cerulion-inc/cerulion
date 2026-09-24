# cerulion_core internals: scheduler, triggers, graph runtime, determinism, barriers

Present-tense contracts for `cerulion_core`'s execution side. Read this before modifying
`src/scheduler/`, `src/graph/`, the live loop, the barrier/multi-process machinery, or
macro-runtime FFI behavior. Companion: `core-transport.md` (data plane),
`core-testing.md` (full test map). Code on `main` beats this document.

## Core principles (the numbered index)

Docs and code comments across the repo cite these by number ("Principle #7"). This table
is the resolver. The oracle column names the crispest enforcing test or gate where one
exists; `none` means the principle is enforced by the conventions in
`core-testing.md`, not by a gate.

| # | Principle | Rule | Enforcing oracle |
|---|---|---|---|
| 1 | Zero copy | Hot path = zero copy + shared memory | `check_hot_path_allocs.sh` (CI lint) + `zero_alloc_test` / `zero_copy_hot_path_test` / `flat_latency_test` |
| 2 | Data is truth | No callback encodes meaning; truth is in the data | none |
| 3 | Observable state | All state observable independently of execution | none (counter/accessor surfaces, e.g. `backpressure_counters_test`, pin instances) |
| 4 | Explicit timings | Clock triggers OR explicit pairing windows (`sync_window_ms`) | `sync_fire_iox2_test` (the pairing-window half) |
| 5 | Graph is source of truth | Execution order derivable from the graph file | `graph_test` |
| 6 | No data loss | Missing wakeups must not lose data | `external_live_fire_iox2_test` (wake-storm no-lost-tail arm) |
| 7 | Replay = Live | Replay MUST be identical to live execution | `polled_vs_live_iox2_test` + `replay_test` |
| 8 | Single sessions | ONE zenoh session, ONE iceoryx2 node per process | `context_transport_test` (every `NodeContext` shares the host `TransportManager`) |
| 9 | Use tokio | Async zenoh API on a tokio runtime | none |
| 10 | Loan and write directly | Write directly into the loaned buffer | `variable_schema_fixed_field_test` (direct access into the loaned slot) |
| 11 | No memory leaks | Proper lifecycle semantics | `miri` CI job (blocking; scoped to FFI-free tests) |
| 12 | Use logging levels | Structured logging via `tracing` | none |
| 13 | No fake data | NEVER fake/mock/simulated data in benchmarks, reports, or tests | the hand-written-oracle rule (`core-testing.md` §Adding a test here) |

## Execution model

- `GraphRuntime::step()` is the ONE seam both the polled path (`run_until_shutdown`) and
  the live path (`run_live` → `live_step`) call. Any change to `step()`, the executor,
  or drain ordering ripples into the entire live/WaitSet/reactor test family; run every
  binary that builds a `GraphRuntime` or drives `step()` (a curated subset misses
  regressions; reactor unit tests are the most fragile because they encode
  step-ordering premises).
- Everything the executor needs per DAG level (the block/non-block partition and the
  REST-walk verdict) lives in ONE `LevelPlan`, built by a single build-time pass over
  the levels and read by `level_idx`. Add a new per-level fact to that struct, never as
  a fourth parallel `Vec`: alignment is meant to be structural, not asserted.
- The level executor fires DAG levels in order. Levels narrower than
  `Scheduler::PARALLEL_FIRE_THRESHOLD` take a strictly-zero-alloc serial path; wide
  levels fire under rayon via `par_values_mut().for_each()` into reusable per-node
  trace fragments, merged by draining decisions in position order (no sort). Only
  rayon's intrinsic injector residual remains (~1 allocation per ~63 dispatches from a
  non-worker thread; a strict zero-alloc assertion must never run against a
  rayon-dispatching path; the serial-below-threshold routing is what makes 0/step
  possible). Pinned by `step_zero_alloc_test` (alloc count, not latency) and
  `rayon_fire_iox2_test` (parallel == serial == flat byte-identity; the decision-order merge
  is load-bearing: any other merge order fails it).
- Shared-topic serial walk: a level holding two or more non-serial-gated producers of
  ONE `multi_publisher_topics` topic never fires its REST under rayon. The build stamps
  that level `RestWalk::InsertionOrder` (its `LevelPlan::rest_walk`, static, one
  `Copy` read per level; an `info!` per (level, qualifying topic) names the topic, the
  producers, and whether the stamp changed the routing or the size gate had already routed
  that level serially) and
  `Scheduler::rest_driver` DECIDES the driver (the ONE exhaustive verdict, ranked
  size/pool, then armed replay pauses, then this constraint) and
  `tick_decided_parallel` walks the REST serially in insertion order, which IS declaration
  order (`add_node` runs in `config.nodes` order). CONSTRAINT, not a
  knob: the pair publishes into one shared FIFO from inside their ticks, a rayon
  interleave is a scheduling artifact the read log and the bag inherit but the
  decision-ordered fire trace cannot see, so replay could re-fire the pair into a
  different order on healthy code. WHEN, never WHAT: fire set, trace and data are
  byte-unchanged; the level forgoes within-level parallelism. Scope: the guarantee
  is per PROCESS: among a level's producers that share a process and reach the REST,
  publishes land in declaration order; a serial-gated producer (no-op-snapshot node with
  non-trigger inputs, or block-involved) publishes FIRST in PASS 1, so the level's order
  is fixed but not declaration order across that boundary; producers split across
  processes (the default process-per-node partition) have no cross-process order, and this
  rule creates none; which worker's connection is drained first is an attach-order race
  nothing pins. What holds is that a consumer never sees one writer's frames interleaved
  with another's (the drain is per publisher CONNECTION, measured), and that the recorder
  labels every frame of such a topic with its writer, so replay is per-writer. All-`block` / mixed
  shared topics are already fused in graph order by the block routing and never reach the
  REST, so they are not double-handled. `Scheduler::forced_serial_rest_walks` counts only
  the levels the constraint ALONE re-routed (a level narrow by size or by an armed replay
  seam is never credited). Pinned by `read_outcome_capture_iox2_test`'s `*` arms
  (the wide twin of the narrow producer-token arm, its thread-id routing pin, and the
  build-time verdict pins) and the scheduler's `rest_driver_verdict_*`
  + `tick_decided_parallel_insertion_order_*` unit arms.
- **`fire_count` records unconditionally** in `fire_node_into`: a fire is counted and
  traced whether the tick succeeded, returned an error, or panicked (panics bump panic
  counters; a node is disabled after the consecutive-panic cap). Only PUBLISHING is
  affected by tick outcome. Consequence for tests: a fire-count assertion proves a node
  was scheduled, not that it produced data; strong pins assert downstream DELIVERY
  against a hand oracle.
- On the macro path the publish contract is stronger than suppress-on-Err: the generated
  tick arms deferred publish per loan and only a fully-Ok tick enables Drop-publish, so
  EVERY early exit (sibling-loan failure, transport error in the input chain, user-body
  error, panic) publishes nothing. Non-macro paths (closures, raw FFI) keep plain
  Drop-publish: an explicit publish before an error stands.
- **Read paths**: `try_view` is the ONLY read served from the step-boundary frozen slot;
  `try_receive` bypasses it. Ticks needing accumulate-all semantics (drain-all
  consumers, e.g. a transform-aggregation topic) opt out of unified drain
  (`.with_unified_drain(false)`).
- Step-boundary snapshot/hold: a firing node's non-trigger (latest-value) inputs are
  snapshotted at the step boundary and HELD across steps: before first delivery the
  body waits (no fabricated default); a held frame replays its ORIGINAL wire timestamp
  (a held frame must not look fresh; `InputView::wire_timestamp_ns()` is the freshness
  arbitration surface for mux-style nodes); `block` inputs read live (excluded);
  `sample(N)` inputs replay the last accepted value. Backpressure accounting runs
  EXACTLY once whether an input drains at the snapshot or in-body (a re-drain double-counts and fails the pin).
  Same-step data flow: an earlier-LEVEL publish flows down the DAG within one step; the
  frozen prior-value rule applies to SAME-level producers only.
- **Graph YAML denies unknown fields.** A typo'd or stale key is a loud parse error, never
  a silently-defaulted one, the misleading-surface rule applied to config. Adding a field
  to a graph config type therefore means adding it to the round-trip and rejection oracles
  in `config_deny_unknown_fields_test`, or the gate that catches the typo class goes stale.
- Unified trigger drain: eligible data-trigger consumers drain trigger + body on one
  subscriber. `NodeEntry::unifies_trigger_drain` decouples drain eligibility from the
  rayon-eligibility flag (`performs_input_snapshot`). `CERULION_DRAIN_DISCIPLINE=separate`
  is a hidden A/B seam (exact match only; unrecognized values warn once and stay
  Unified; read once per build and threaded to both the count and build passes so
  provisioning/wiring drift is structurally impossible).

## Chain-fused execution (a future design)

Every node hop today runs through the shipped level executor and the one consumer read
path: the executor fires DAG levels in order, and a consumer reads a published frame
through the iceoryx2 queue receive in `src/transport/subscriber.rs` like every other
read. A bypass or raw-handle consumer read is not an accepted design
(`core-transport.md` §Transport model states the same rule): a second read path doubles
the read-path invariants and re-opens the use-after-reclaim SHM bug class.

Chain-fused synchronous execution is the intended path to nanosecond-class intra-process
latency. It is a design, not shipped code; any implementation is bound by these
constraints:

- Within a fused process group, a linear single-consumer trigger chain executes
  synchronously: the executor immediately invokes the consumer with a scheduler-bounded
  raw view of the just-committed slot. The per-hop TARGET is ~tens of ns (a target, not
  a measurement); chain end-to-end = the sum of node compute + one wake at the chain
  head.
- The producer's SHM publish is UNCHANGED: the loaned-slot write IS the publish, and
  every hop stays on SHM in the default namespace, so `topic echo`/`hz`, external
  attaches, and recorder (`bagd`) taps are untouched. This is the observability
  invariant; a fusion that skips the SHM commit has broken it.
- The raw view exists ONLY inside the chain-execution design, never as a general read
  path; implementing the lever as a bypass read re-creates exactly the read path the
  rule above excludes (doubled read-path invariants + the use-after-reclaim SHM bug
  class).
- Joins, fan-outs, and Sync keep the step + barrier structure; fusion applies to linear
  single-consumer chains only.

### The census (shipped; the executor ignores it)

`graph/chain.rs` decides WHICH edges qualify, and nothing more. It is pure analysis over
`GraphTopology` + `TriggerEdges` + `Levels` + the run's colocation: `census_chains` returns
the chains that qualify plus a `ChainBar` for every consumer edge that does not, so the
verdicts partition the graph's consumer edges and a reader can add them up. Read by
`cerulion graph chains` and written into every run's `run.json`, so the shape of real
graphs is measured rather than guessed. No execution path consumes it.

The rules and the reason each exists are in that module's own header. Two are worth
naming here because they are easy to get wrong:

- **The context rule.** A fused consumer may read no non-trigger input whose in-graph
  producer sits at a level at or after the CHAIN HEAD's level. The queued executor
  snapshots a node's non-trigger inputs at the start of its own level phase, when every
  earlier level has ticked; a fused consumer runs inside the HEAD's level phase, when only
  part of that level has. The head is the bound, not the consumer's own level. An edge the
  rule refuses CUTS its chain: the refused consumer heads a new one, judged against its
  own level.
- **One fusable edge per producer.** A topic-level fan-out is refused because the second
  consumer would wait for the first consumer's whole chain. A node publishing two
  single-consumer topics is the same wait one level up, and refusing it is also what makes
  "a node belongs to at most one chain" true rather than assumed: a consumer has at most
  one trigger edge, so once a producer has at most one outbound fused edge the fused edges
  form disjoint paths.

## Trigger policies

- Trigger policy is a NODE-TYPE concern (macro attribute); graph YAML carries topology
  only. `Period` fires on a schedule: `floor(elapsed/period)` fires, not one per step.
  `Data` fires per delivered trigger frame. `Sync`/`UnboundedSync` align ONLY
  `#[input(trigger)]`-marked inputs: a plain `#[input]` never gates the fire (a stale
  config stamp far outside the window must not block it); a starved trigger means the
  node never fires until it arrives, completing alignment on the retained other trigger.
  Sync wiring guards fail the build loudly for an unwired trigger port, warn for a
  degenerate single-trigger sync, and terminally reject a triggerless Sync policy
  (`sync_trigger_wiring_guard_test`).
- **Sync delivery is PER SET.** A Sync node fires once per COMPLETE ALIGNED SET, in set
  order, and each trigger message is consumed by at most one set, so a backlog holding k
  aligned sets yields k fires whose k-th tick reads the k-th set's members, not one fire on
  the freshest frames. The verdict engine is the PURE `scheduler/sync_match.rs`: choosing a
  set's members may need transport work (a non-consuming has-next probe, a pop into held
  storage to learn a stamp, a pop-past to skip a frame), so `next_sync_step` is pure over
  `(heads, next_info, window)` and DEMANDS those facts through verdicts the driver performs
  before re-running. Precedence, decided and inviolable in this order: (1) IN-ORDER per input
  (FIFO consumption, delivered stamps non-decreasing); (2) ARRIVED-SET PRESERVATION: a set
  complete and in-window among ARRIVED frames is never destroyed; (3) SPREAD (span
  minimisation, the quantity the window bounds) is best-effort WITHIN those, realised as
  single-step strict-improvement descent whose plateau stops the walk. Rule 2 is what the
  descent GATE exists for: descent is permitted only when at least one input has no second
  arrived frame, so in a k-set backlog sets 1..k-1 serve greedily and only the TAIL may be
  tightened. A failed probe is POSITION-AWARE and fail-closed: at the argmin, absence means
  "nothing to descend to" and the set fires; at the GATE, absence is the PASS witness, so a
  failed gate probe must resolve to PRESENT, or a probe failure would vouch for scarcity on
  an unverified input and destroy arrived complete sets. Death is evaluated BEFORE descent,
  so a provably unmatchable frame is classified UNMATCHABLE (something is wrong) rather than
  PASSED-OVER (the feature working).
- `External` nodes are ingress/drivers: `external_source()` returns
  `Fd` (pollable, attached non-owning), `Blocking` (helper thread + doorbell pipe), or
  `HostDriven` (polled seam only). `run_live` refuses any external node provably inert
  at launch with ONE aggregated error naming every offender with distinct reasons and
  the fix; the refusal is sticky (re-returned, never re-collected) and tears down
  sibling Blocking helpers eagerly. Raw-fd hazards are guarded on our side because
  iceoryx2's select maps EBADF to a process abort and its fd-set has no fd-value guard:
  invalid/duplicate fds are rejected at collect, a post-construction-stale fd is skipped
  at attach, POLLNVAL mid-run is a loud unbind, and the fd newtype is non-owning (never
  derive `Clone` on it; iceoryx2's `FileDescriptor::clone` dups into an OWNED fd).
  Wakes coalesce to one fire per step; an undrained fd level-triggers a refire; a
  poisoned Blocking helper's pipe EOF UNBINDS the doorbell loudly instead of
  busy-looping on the permanently-readable fd.
- QoS knobs are orthogonal to triggers: `expect_within_ms` (input watchdog: a
  non-trigger input resets on each delivered read, so it never false-fires while data
  flows), `promise_within_ms` (output), `tick_within_ms` (wall-clock tick budget,
  counter-only; tests assert liveness, never a deterministic value), `throttle_ms`
  (producer rate cap). Backpressure (`drop_oldest` / `sample(N)` / `block`) is a
  scheduling concern with no Cerulion-side data buffer; warns are once-per-regime
  edge-triggered while counters are UNCONDITIONAL (block is designed lossless
  backpressure: a consumer at threshold is a legitimate steady state, so per-frame
  warns are the flood class the latches kill). `drop_oldest` recovery is O(1) in
  overflow magnitude (drain-to-latest serves the freshest sample).
- `#[on_event]` handlers are type-routed per `(port, kind)`; multiple same-kind handlers
  co-firing dispatch in DECLARATION order (pinned with a reverse-alphabetical fixture so
  a sort-by-name or hash-order regression fails).

## Determinism (Replay = Live)

- The polled `step()` seam and the live WaitSet seam fire the same nodes in the same
  order with the same data (`polled_vs_live_iox2_test`, hand-oracle-anchored).
  `fire_time_ns` is the one legitimate difference on the default live path (wall-delta
  advancement).
- Deterministic-live: `GraphRuntime::build_live_deterministic*` runs the live loop on a
  Barrier gating clock advancing a fixed run-independent logical quantum (the graph's
  tightest timing, floored at 1 ms) per `live_step`; recorded `fire_time_ns` is
  replay-deterministic. Wall-clock health (liveliness sweeps, silence deadlines) rides a
  DEDICATED `RealClock` watch clock; the two never mix.
- Replay never touches external sources: `external_source()` is queried exactly once, at
  `run_live` entry, never under polled `step()` or replay (provably: a panicking
  fixture + a query counter at zero while the replay leg delivers the oracle sequence).
- Every wait/park/spin primitive is RECORD-ONLY: it changes WHEN `step()` runs, never
  WHAT fires. Any new wake path must preserve this firewall and be pinned park-ON ==
  park-OFF == hand oracle on the same build path.

## Live loop, wake, and power

Wait-terminology discipline (conflating these in discussion leads to conflating them in
code): **busy-spin** = perpetual poll, the failure mode; **spin-then-block** = bounded
imminence-gated poll then a blocking wait; **monitor-wait** = a hardware park instruction
idling the core watching a memory address (ring-3, no kernel block, no cache flush).

- The live runtime is event-driven, not ticked: `live_step` blocks on an iceoryx2
  WaitSet and wakes on data-trigger listeners or the next Period deadline; the 1 ms is
  only a floor on the WaitSet timeout (clamped to a bounded heartbeat), never a
  data-latency gate.
- iceoryx2 events cannot be monitor-waited (they are sockets; see the transport
  dossier), hence the SHM DOORBELL: a cache-line-aligned atomic per topic; the publisher
  rings AFTER the iceoryx2 send; the consumer monitor-waits on the line; iceoryx2 stays
  the data channel and correctness fallback. After a doorbell wake, drain the iceoryx2
  listener so the WaitSet doesn't double-report. The doorbell is a NO-OP STUB on macOS;
  never select it as a desk-side wake primitive.
- Platform park ladder (runtime-detected): x86 WAITPKG (UMWAIT) / AMD MWAITX where
  present; ARM WFE + event stream (WFE bypasses cpuidle entirely; it architecturally
  cannot hit the cache-flushing deep C-state); no-primitive targets (macOS included) run
  the park default-ON in a degraded chunked short-sleep-recheck: never a busy-spin,
  never a long single sleep (macOS timer-coalesces long sleeps; only a short final sleep
  wakes hot).
- The hardware park is OS-COOPERATIVE (it yields to a same-core peer): a UMWAIT/WFE-parked thread is RUNNING
  to the scheduler, so `monitor_wait_block` slices the hardware arm at the shared 20 µs
  `monitor_wait::PARK_RECHECK` (the REQUESTED slice; on aarch64 the effective slice is
  one event-stream period, next bullet; rmw's `park_block` uses the same constant) and ends
  EVERY park-loop iteration with `yield_now()` (`park_yields`, record-only); a
  co-located runnable peer gets the core within one slice instead of at CFS wakeup
  granularity; without the yield a same-core pinned ping-pong measures p50 6.997 ms
  (`rmw_cerulion`'s `rmw_wait_pingpong_discriminator_test`, the `#[ignore]`'d
  `same_core_pair_park_on_spin_off_stays_under_100us` arm). The degraded sleep and
  wake-word arms keep their measured ~100 µs pacing (a sleeping thread already released
  its core); `CERULION_MW_SINGLE_PARK=1` (measurement-only) removes the yield boundary.
  On aarch64 the slice is coarser: base WFE has no timeout operand, so a bounded wait
  rides the generic-timer event stream and the EFFECTIVE slice is one event-stream
  period, which is machine-specific (kernel target ~100 µs). A same-core ROUND-TRIP
  structurally costs two to three wake handoffs, so it takes a few periods (confirmed on
  aarch64 by the RTT and, independently, by the slice counter), far under the CFS wall on
  x86 and aarch64, with cross-core parks unaffected. The park offers no sub-100 µs ARM
  same-core wake: that needs the wake-word/futex arm or an event-stream divider control,
  both outside the park's scope.
- Deep-idle economics: the real-clock latency penalty on idle/low-rate graphs is the
  deep C-state exit (cache flush + frequency ramp), not the timer wake. The C-state/DMA
  lock (default-on for real-clock runs; `CERULION_CPU_DMA_LOCK=0` opts out,
  `CERULION_CPU_DMA_LOCK_US=N` sets an explicit cap) owns the large win; the scheduled
  spin (`CERULION_LIVE_SPIN_US`) catches only listener events within its budget: null
  on period-driven graphs, small on streaming ones. There is no cheap spin path to the
  deep-idle win. Wake-ahead timers buy nothing without heavy spin, which is why the
  shallow park, not deep-idle-plus-wake-ahead, is the shipped design.
- Stale-wake rule: any event a step does not consume must be drained where it lands:
  a unified binding's standalone listener is drained in `drain_level`'s Unified arm
  (events-then-samples), else one stale event per publish self-sustains a free-run under
  the multi-process split (`unified_stale_wake_park_test`; removing the drain fails it).
- The park WATCHES external fds on every tier (`park_poll_fd_ready`, record-only; the
  pre-step sweep still does the drain+fire); a barrier participant idles in
  `monitor_wait_block` regardless of park policy (opt-out honored via sleep-recheck);
  without that routing, park-off multi-process runs collapse to the timeout cadence.

## Barrier and multi-process lockstep

- `BarrierShared` (`src/barrier.rs`) is a lock-free count-down sense-reversing barrier;
  `MappedBarrier` maps the same atomics into a POSIX-SHM `MAP_SHARED` page for
  cross-process use. The step-start wake word (epoch + parked-rank bitmask, offsets
  const-assert-pinned) wakes parked peers on arrival; on hardware-park machines the
  parked bit is scoped to the kernel-block arm only, so arrivers never pay a wake
  syscall the CPU monitor cannot hear. Ranks beyond the bitmask width degrade with a
  warn. Known gap: the reset-before-publish memory ordering rests on reasoning plus
  an SB-litmus hammer test; there is no loom model.
- **The barrier is a WHEN-gate on level advance, never a change to the fire set, order,
  or data.** A split graph run as multiple contexts over one SHM root merges to a fire
  sequence byte-identical to the monolith and a hand oracle
  (`barrier_level_gate_iox2_test`; the real-subprocess cross-address-space replica is
  `barrier_level_gate_subprocess_iox2_test`, which merges the two processes' traces and
  asserts them against the same hand oracle. It is `#[ignore]`'d and runs on demand on
  any Unix with `-- --ignored`, over a real `MAP_SHARED` page and the host's own park
  shape (x86 WAITPKG, aarch64 WFE, or the macOS sleep-recheck); the CI gate is the
  in-process sibling).
- **The MID-LEVEL barrier covers the one edge the DAG does not model.** The level-boundary
  barrier orders edges that CROSS a boundary; a plain non-trigger `#[input]` whose consumer
  is fired by its own timer shares ONE global level with its producer, so a partition
  splitting that pair leaves the pairing to OS scheduling: which value the consumer
  snapshots depends on how the two processes interleave, so the live run is
  nondeterministic and its recording cannot be re-executed to the same result.
  `run_level` splits into a snapshot phase (drain + decide + freeze every firing node's
  non-trigger inputs) and a tick phase, called back-to-back so the MONOLITH is byte-unchanged
  BY CONSTRUCTION; on the levels that need it the live loop rendezvouses BETWEEN the halves,
  so after the mid-level generation opens every group's snapshots are provably taken.
  CONDITIONAL: flagged per global level, so a run's law is `global_levels + flagged_levels`
  generations per step (`GraphRuntime::generations_per_step()`) and a graph with no split
  pair pays NOTHING. EVERY participant crosses the extra rendezvous, owner of that level or
  not. The flags are computed ONCE by the SUPERVISOR from loaded cdylib metadata and cloned
  to every worker; a worker never re-derives them, because two workers disagreeing about
  one level desynchronise the shared generation counter for the rest of the run, and the
  install refuses a flag vector whose length disagrees with the map. RESIDUAL: the fused
  decide+snapshot+tick path interleaves per node, so its rendezvous sits BEFORE the fuse and
  a `block`-involved node's own plain non-trigger inputs stay unordered; the plan-time report
  names it per finding at `info`, carrying the producer, consumer, input, topic and
  global level, a stable `remedy=` token, and the fix sentence for that consumer's policy.
- Stalled peer: `barrier.wait` times out at `BARRIER_BOUNDARY_TIMEOUT`, `step()` sets a
  TERMINAL poison (`GraphRuntime::is_barrier_failed`) and early-returns with the logical
  clock frozen: a bounded escape, never a silent proceed.
- `build_live_deterministic_with_manager_and_barrier` validates with loud errors, never
  panics: handed quantum >= 1 ms AND <= the context's local tightest timing; the global
  level map must be a contiguous bijection; the clock contract must match. The k-way
  trace merge keys `(step, global_level, rank, seq)`; a duplicate rank is an always-on
  loud error (`trace_merge_test`). The producing owner provisions cross-context handoff
  topics above the consumer side's default depth.
- munmap-during-park safety is POSIX semantics: `shm_unlink` removes the NAME only, so a
  parked waiter survives a peer SIGKILL and the owner's unlink (`TimedOut`, never a
  fault), and a surviving pre-drop handle still wakes the park.

## Trace rings and the recording seam

- `src/shm_ring.rs` / `src/trace_ring.rs`: SPSC POSIX-SHM rings (real SHM on macOS AND
  Linux, no stub). The wait-free `push` does no alloc/lock/syscall/clock (own-binary
  counting-allocator pin). Plain consumer `open` REFUSES a lapped ring; the mid-run
  attach seam is `open_at_live` (sets `read_cursor := write_cursor`); any consumer
  attaching mid-run must use it (the default ring laps in minutes on a busy graph).
  Second producer mint returns `None` (SPSC guard); over-commit is a hard error in every
  build mode; owner drop unlinks but a prior consumer still drains.
- The scheduler → trace-ring hook records fires for the out-of-process recorder;
  recording-ON steady state stays zero-alloc (`trace_ring_hook_zero_alloc_test`).

## Clock model

- `real_ns()` does not advance while the machine is suspended, on either platform (Linux
  `CLOCK_MONOTONIC`, macOS `CLOCK_UPTIME_RAW`), matching `std::Instant` semantics.
  Determinism is unaffected: `real_ns` is the non-deterministic escape hatch; replay
  uses `VirtualClock`. `RealClock::virt_ns()` / `ext_ns()` return `None` with a one-time
  warn.

## cdylib runtime contracts (host side)

- `CERULION_ABI_VERSION` (`src/lib.rs`): the loader hard-rejects a cdylib whose
  exported ABI version disagrees (`abi_version_mismatch_test`). Bump it on ANY change to
  a generated FFI signature or error-code meaning; additive OPTIONAL symbols (the
  snapshot pair, the unified-drain export) need no bump; symbol PRESENCE is the
  capability, which also structurally scopes the feature to macro cdylibs.
- `RUSTC_FINGERPRINT` (`src/rustc_fingerprint.rs`, ABI v22): the loader ALSO
  hard-rejects a cdylib whose `cerulion_rustc_fingerprint()` disagrees with the host's
  (`rustc_fingerprint_mismatch_test`), right after the ABI-version check. This catches a
  class the ABI version and the `abi_layout` size/offset pin cannot: two rustc releases
  can lay out a struct identically and still encode a niche-holding `Option::None` with
  a different bit pattern (rustc 1.97.0 changed it), which corrupts
  `CerulionSubscriber.frozen: Option<FrozenSlot>` across the FFI and aborts the process.
- Capability semantics: a cdylib that holds its input snapshot reports
  `holds_input_snapshot() == true` while `performs_input_snapshot() == false` (holds
  across steps but fires SERIALLY, off the rayon path; any node with non-trigger
  inputs and no snapshot capability is likewise routed serial). `SnapshotState` is
  Pending → Active on a successful set and TERMINALLY Failed on a set failure; a failed
  set never masquerades as a healthy snapshot (the per-step snapshot FFI is never
  called again; `cdylib_snapshot_set_failure_test`).
- Host fail-safes: a failing drain FFI maps to the safe "nothing popped" result: no
  fabricated fire signal, one flood-latched error, the runtime survives. Corrupted info
  JSON refuses to LOAD (error carries length + prefix + label). A panicking
  `external_source()` runs under an inner `catch_unwind` BEFORE the registry guard
  drops, so it cannot poison the process-global NODES mutex. A cdylib that spawned a
  detached helper thread makes `dlclose` a use-after-unmap, so the `Library` handle is
  deliberately leaked with a breadcrumb.
- A cdylib statically links its OWN `cerulion_core` (and its own `tracing` dispatcher
  and its own iceoryx2 log-level static). The macro-generated init therefore installs a
  cdylib-local stderr tracing subscriber AND applies `IOX2_LOG_LEVEL` from the node's
  frozen env snapshot; a repo-walking structural guard requires every hand-written
  `cerulion_node_init` definition to do the same, so it covers every initializer in the
  tree rather than a hand-maintained list. `NodeContext::env_snapshot` isolates nodes
  from post-build env mutations. Producer reconciliation is logged from INSIDE the owning
  process at `NodeContext` Drop (host harvest marks first, so it logs at most once).
- Parity rule ("no inert shipping"): every macro-runtime semantic proven in-process must
  be proven again through a LOADED cdylib. The in-process path and the cdylib info-JSON
  emitter are separate code, and a trigger policy the emitter never writes leaves the host
  with no policy at all, so the node falls back to a plain data trigger and fires on any
  single input instead of its declared gate. `macro_cdylib_policy_round_trip_test` pins
  the macro to info-JSON to runtime policy round trip and `info_json_key_parity_test` pins
  the emitter/parser key set; the `cdylib_*` test family pins the behavior. Extend it when
  adding macro semantics.

## Key enforcing tests (execution side; full map in `core-testing.md`)

| Test file | Pins |
|---|---|
| `polled_vs_live_iox2_test.rs` | Replay=Live firewall: polled seam == live seam == hand oracle |
| `step_zero_alloc_test.rs` | zero allocations per step at steady state (incl. multi-node levels) |
| `rayon_fire_iox2_test.rs` | parallel == serial byte-identity; panic isolation; decision-order merge |
| `snapshot_wiring_iox2_test.rs` / `non_trigger_hold_iox2_test.rs` | frozen-prior-value rule; cross-step hold; held-stamp freshness |
| `sync_fire_iox2_test.rs` / `sync_trigger_wiring_guard_test.rs` | trigger-scoped Sync; loud starvation guards |
| `external_live_fire_iox2_test.rs` / `external_gating_replay_iox2_test.rs` | external fire seam; inert-at-launch refusal; replay never queries sources |
| `live_gating_clock_iox2_test.rs` | deterministic-live quantum advancement; watch-clock decoupling |
| `barrier_level_gate_iox2_test.rs` / `barrier_park_wake_iox2_test.rs` | barrier firewall + poison; barrier-arrival park wake |
| `unified_stale_wake_park_test.rs` | stale-event drain on the unified binding (free-run regression) |
| `abi_version_mismatch_test.rs` | loader rejects ABI skew |
