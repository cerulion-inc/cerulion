# Recording internals: bag format, recorder daemon, run binding

Contributor dossier for `cerulion_bag` (the MCAP format crate) and
`cerulion_bagd` (the recorder daemon), plus the run-directory and
recorder-attach seams they share with `cerulion_core` and the CLI. Everything
here is a present-tense contract; the enforcing test is named beside each one.
User-facing recording docs live in `docs/bag.md`.

## 1. Artifacts

- A recording bag is a **standard MCAP file** (`.mcap`) written by
  `cerulion_bag`'s hand-rolled writer. `graph run --record`, `bagd`, and
  `cerulion bag play` (with or without `--resim`) all read/write this format. The
  only JSONL artifact in the system is the separate legacy publish trace
  (`cerulion trace inspect`).
- Bags carry attachments under the reserved `__cerulion/` prefix:
  `recorder.json` (host identity: arch/os/version, used by replay's
  cross-host advisory), `record_health.json` (per-topic drain health),
  `record_coverage.json` (what was and was not recorded, and why), plus
  `graph.yaml` and `env.json` for replay.
- The bag's `graph.yaml` attachment is the **effective in-memory graph config
  the run executed**, rendered by `graph_cmd::render_effective_graph_yaml`,
  never a copy of the on-disk graph file. The same renderer writes the run
  directory's `graph.yaml`, so the two agree by construction: change one seam,
  check both. `prepare_recording_inputs` is the one seam both record paths
  (graph-run recording and standalone recording) share.

## 2. MCAP writer determinism rules (`cerulion_bag`)

The format crate is a byte-determinism contract plus a torn-tail crash model.

| Rule | Mechanism | Enforcing test |
|---|---|---|
| Zero wall-clock reads in the crate | Every timestamp comes from the caller; time-based policy injects its clock from the caller | `bag_determinism_test` (byte-identity); the crate imports no clock API |
| Same input twice ⇒ byte-identical files | Channel ids assigned by **sorted topic name**, schema ids by sorted schema identity, never registration order | `bag_determinism_test` |
| Chunk boundaries belong to the caller | `flush_chunk` closes a chunk; the writer self-closes only at the size threshold `DEFAULT_CHUNK_MAX_BYTES`, never on a clock it cannot have | `bag_determinism_test`, `bag_roundtrip_test` |
| Payloads are COPIED into the chunk arena | `write_message` copies immediately; the caller's buffer carries no lifetime obligation after return | `bag_roundtrip_test` |
| Torn tail is readable | A truncated bag (crash before finalize) recovers every complete chunk; a corrupted chunk body is detected, never silently returned | `bag_crash_recovery_test` |
| Reserved prefix | `__cerulion/` topics (scheduler trace, nondeterminism markers) are writer-internal; user topics under the prefix are rejected | `bag_validation_test` |
| Schema identity travels | Every schema entry carries `hash_recipe` (u32) = `HASH_RECIPE` in `cerulion_core::trace::bag`; an absent recipe in old artifacts means the original recipe | `bag_schema_catalog_test`, `bag_roundtrip_test` |
| Disk writes are `writev(2)` batches | IOV_MAX batching, partial-write resumption, EINTR retry, hard error on no progress | `bag_writer_syscall_test` |

The recorder copies the payload rather than borrowing it. A pointer-stashing
"zero-copy" write path would force the recorder to hold shared-memory borrows
until the disk write completes, and a topic created by a foreign publisher is
provisioned at a stock borrow budget that cannot afford that. Copying releases
the frame's owner at record time, which is what lets **one** write path serve
every producer regardless of provisioning; there is no `min_borrow` budget and
no inline write mode. The copy is confined to the recorder: the publish hot path
is unaffected, since a producer still writes directly into its loaned
shared-memory slot and the recorder reads a tap. Chunk-scoped writing
exists as a convenience (`write_chunk` / `ChunkScope`) with no lifetime
contract; its drop guard is pure error containment.

The channel set **freezes at bag creation**; there is no `add_channel`.
Anything learned after creation (a late schema resolution, a late-discovered
topic on an already-created bag) cannot extend the open bag; it can only be
reported in the coverage attachment.

Independent-oracle rule: round-trip and crash tests re-read every bag with the
upstream `mcap` crate, never only our own reader; the format claim is
"standard MCAP", so the oracle must be external.

## 3. Recorder drain and staging (`cerulion_bagd`)

- Taps are **listener-less `DataOnlySubscriber`s**; they add no event-service
  load to the producer and are structurally un-attachable to a wait set.
  A data-only tap has **no back-fill**: frames published before the tap
  attached are gone, which is why tests rendezvous on `topic_subscriber_count`
  (a condition poll) rather than a sleep.
- **bagd never evicts.** Both `room == 0` arms of the tap drain WAIT. The
  recording loss boundary is the topic's own provisioned SHM queue depth; no
  recorder-side policy discards a frame it has seen.
- **One write mode.** Payloads are copied into the MCAP chunk arena at drain
  (section 2), so the writer thread is affordable at any
  `subscriber_max_borrowed_samples >= 1`; the tap arming floor is 1.
- **Chunk timing lives in bagd.** A chunk closes at the size cap or at
  `CHUNK_TIME_FLOOR_MS`, whichever comes first; the clock is injected here so
  the format crate stays wall-clock-free.
- **Per-tap staging bound.** `RECORDING_TAP_STAGING_MAX_BYTES` caps what one
  tap may stage per pass; hitting it stops that tap's drain for the pass
  (never blocking other taps, never evicting) and increments
  `TopicHealth::staging_full_passes`.
- **Drive-loop pacing is backlog-aware** (drain harder when behind), pinned by
  the sustained-rate no-loss arm in `tests/e2e_test.rs`.
- **Writer-thread teardown is deterministic on every exit path**: the writer
  queue is drained and the thread joined at recorder drop, so a crash-path
  exit still bounds teardown.

## 4. Discovery and settle contracts

- **Enumeration runs OFF the drive loop, and must stay there.** `list_topics()`
  walks the iceoryx2 service directory at a cost that scales with the number of
  live services, so it cannot run inline on the recorder's drive loop: a pass
  spent walking the directory is a pass that drains no tap, and a fast topic
  overflows its own SHM queue inside that window with `staging_full_passes` and
  `dropped_unwritten` both at zero, because the writer is not the constraint. No
  drain-side change reaches that loss: the frames die in a window where the drain
  does not run at all. `discovery_scan::DiscoveryScanner` owns the walk on its
  own thread and publishes an immutable snapshot the drive loop takes in O(1)
  (`next_scan`). What the recorder DECIDES from that snapshot lives in
  `apply_discovery` → `plan_discovery`; `next_scan` TAKES the snapshot, so one
  enumeration yields exactly one applied scan. Never put a directory walk back
  on that loop.
- **Enumeration is driven by EVENTS, not a cadence.** iceoryx2 0.9.1 exposes no
  discovery event, but a service's static config is a FILE under
  `global.root_path` + `global.service.directory`, and `Service::list` is a walk
  of exactly that directory - so a producer registering is a file appearing, and
  the kernel reports it. `service_dir_watch` is that watch (`inotify` on Linux,
  `kqueue` on macOS, hand-written over `libc` because one flag from one
  directory does not justify a watcher engine). The worker blocks on it and
  walks only when something changed, so a settled machine runs **zero**
  enumerations rather than four a second forever. A burst is coalesced
  (`EVENT_COALESCE_WINDOW`), `SCAN_RATE_FLOOR` bounds a chatty directory, and a
  fixed tail of `CONFIRM_WALKS` confirmation walks spaced `EVENT_CONFIRM_DELAY`
  apart closes the gap between a static-config file appearing and its
  permissions being finalized, which is when `Service::list` starts reporting
  it. The watch is armed BEFORE the worker's baseline walk, so a producer that
  registers during that walk queues an event rather than falling between the
  two. The worker still wakes ten times a second
  to check its stop flag and stamp a heartbeat; those wakes do no work.
- **A background scanner can fail where an inline call could not, so every
  degradation is loud and observable.** Three tiers, each with its own report:
  the watch cannot be ARMED, or BREAKS mid-run, and the worker falls back to
  walking on `DISCOVERY_RESCAN_INTERVAL` with one `warn!` and
  `DiscoveryScanner::wake_source` flipping to `Poll`; or the thread cannot be
  SPAWNED and the scanner degrades to the inline walk on the drive loop
  (correct, just costly) with one `warn!`. A worker that spawned and then
  stopped is caught by the HEARTBEAT, never by scan silence - an event-driven
  worker is legitimately silent on a settled machine, so classifying that as a
  stall would warn on every healthy recording. The heartbeat verdict is
  classified by `classify_scanner_silence` and reported once per regime through
  `ScanSilenceLatch`. Nothing here may be silent: a scanner that has stopped
  working lets the recorder settle on a tap set that is already stale and report
  it as complete, which is the outcome the settle contract exists to prevent.
- **Defaults.** Live topic discovery is ON for `--topics-json` (the argv shape
  `graph run --record` generates) and for `--run`; OFF for `--topic`, `--all`,
  `--regex`, and `BagdConfig::new`. Rationale: a derived tap set (inferred from
  the graph, or declared by a run) admits topics it does not name, so live
  discovery may add them; an explicitly enumerated set is fixed, and stays
  exactly what the operator listed.
- **Levers.** `graph run --record` builds bagd's argv itself, so the bagd
  flags (`--no-live-discovery`, `--discovery-settle-ms`) are unreachable from
  that path; the env switches `CERULION_RECORD_DISCOVERY=off` and
  `CERULION_RECORD_DISCOVERY_SETTLE_MS` reach every path.
- **Settle window.** Bag creation is held until `DISCOVERY_SETTLE_MIN` has
  passed with NOTHING NEW DISCOVERED, measured from the later of the drive
  loop's start and the last discovered tap, capped at
  `DEFAULT_DISCOVERY_SETTLE_MS`. A walk the enumerator has been WOKEN for and
  not yet published also holds (`DiscoveryScanner::walk_pending`): quiet time is
  the absence of evidence and an event is evidence, so releasing on the boundary
  while that answer is in flight would freeze the channel set against a topic
  set already known to have moved. The cap still outranks it. It is a WALL, not a count of quiet
  enumerations: an event-driven enumeration produces no scans on a settled
  machine, so a counting rule could never be satisfied there and every plain
  recording would pay the whole cap - and the wall is the unit the guarantee was
  always about. Even a quiet graph pays the window, and an arm-time find does
  NOT start it (the recorder arms before the graph is released to step 0, so
  what it saw then says nothing about the run's producers).
  `--discovery-settle-ms 0` disables the hold entirely.
- **Schema wait.** Schema learning can hold creation past the settle cap; the
  two holds **max together, they do not add**. The realized hold is reported
  as `BagdSummary::channel_set_closed_after`; measure it from the recorder's
  own summary, never from the bag file's appearance on disk (an external
  anchor reads short under load, never long).
- **Per-verb flag spelling.** The schema-wait lever is `--schema-wait-ms` on
  `cerulion bag record` but `--schema-wait-timeout-ms` on `cerulion bagd`
  (and the background-resolver bound is `--schema-demand-timeout-ms` /
  `CERULION_RECORD_SCHEMA_DEMAND_MS`). A log or error emitted from code that
  cannot know which verb invoked it must name NO flag; naming the wrong
  verb's flag misleads the operator.
- **Co-tenancy hazard.** Both record paths run on the default `iox2_`
  shared-memory namespace, so discovery also taps other tenants' topics on
  the same machine. The only bounds are `DISCOVERY_MAX_TAPS` and the env
  kill-switch; there is no namespace isolation for recording.

## 5. Schema resolution (resolve-or-demand)

The wire carries a schema hash and no name; every network schema surface is
keyed by type NAME. So the recorder's resolver (`src/schema_resolve.rs`):

- resolves locally first (the built-in corpus plus workspace docs), then
  **demands by topic** over the network (the catalog is topic-keyed);
- **verifies, never trusts**: an externally-served doc is accepted only when
  its recomputed hash (current recipe, over doc + closure + corpus) equals the
  hash observed on the wire locally; a mismatch records `Unresolved`, never
  a wrong name (`verify_served_schema` is pure and oracle-tested);
- runs on a background resolver using the **non-waiting** netd verbs
  (`query_catalog_no_respawn` / `query_schema_no_respawn` over
  `NetdClient::connect_existing`), never the `_converged` first-contact loop
  and never a daemon spawn: a recorder must not inherit a desk-interactive
  wait, and a structural source-walk in `schema_resolve.rs` pins the verb
  choice;
- can bound only how many channels get a NAME; channels freeze at creation
  (section 2), so resolution never extends the bag.

## 6. Coverage and loss vocabulary

Three loss counters partition by **where the frame died**; they are disjoint
by definition and must stay so:

| Counter | Meaning | Producer |
|---|---|---|
| `frames_lost` | Never reached the recorder: the topic's SHM queue overflowed before the drain got there | Wire sequence-gap accounting in the tap drain (`record_health.json`, `TopicHealth`) |
| `prefix_lost` | Committed on the wire before the tap's first recorded frame: head truncation, proven by sequence arithmetic | The pure `prove_prefix_loss` predicate (`TappedTopic::prefix_lost`) |
| `dropped_unwritten` | Reached the recorder but never reached the bag: the writer thread died with frames handed off | The dead-writer hand-off, its ONLY producer |

- `dropped_unwritten` appears under the same key in `BagdSummary`,
  `record_health.json`, and the `/bagd/status` surface (a serde alias keeps
  files written under its former name readable; `RECORD_HEALTH_VERSION`
  tracks the rename).
- `prefix_lost` is stamped only when `prove_prefix_loss` holds, which requires
  `BagdConfig::armed_before_producers` (flag `--armed-before-producers`, set
  only by `graph run --record`'s generated argv; default OFF). Standalone
  `cerulion bag record` cannot know it was armed before the producers started,
  so it claims nothing about head loss. A stamped marker feeds
  `is_incomplete()`, emits a truncated-at-the-head WARN, and makes `bag info`
  withhold the COMPLETE verdict.
- `loss_counting_basis` (`prefix_proven` / `prefix_invisible`) says what the
  loss numbers beside it can SEE, and it is claimed **per tap**: the
  `--armed-before-producers` guarantee covers the DECLARED set, while live
  discovery attaches taps to producers that were already running, so those rows
  read `prefix_invisible` even on an armed run (`tap_loss_counting_basis`, the
  shared first half of `prove_prefix_loss`'s rule). The document-wide token on
  `RecordHealth` and `/bagd/status` is the **floor** over the rows; those
  surfaces are read with no row in hand, so one uncovered tap takes the whole
  document down and `bag info` prints the counting caveat. Per row it rides
  `TopicHealth::loss_counting_basis` (additive `Option`, no
  `RECORD_HEALTH_VERSION` bump; `None` = a document that predates the field, making no per-row
  claim).
- `record_coverage.json` reporting rules: `enumerated` records the OUTCOME of
  scanning, never configured intent; `is_incomplete()`, not `gap_count()`,
  escalates bagd's terminal summary line; a live producer that is NOT recorded
  is named with its reason.
- `RecordCoverage::mirrors_established: Option<bool>`: `Some(false)`
  escalates `is_incomplete()` and prints an UNVERIFIED coverage line; `None`
  means NO CLAIM (unknown, not failure). Additive field, no
  `RECORD_COVERAGE_VERSION` bump. It is fed by the checked mirror gather
  (`gather_mirror_provenance_checked`; an empty windowed listen is not
  evidence of absence), retried at arm time up to `MIRROR_GATHER_ATTEMPTS` ×
  `MIRROR_GATHER_WINDOW`; the retry cost is paid only on an incomplete gather.
- `all_channels_exact` is deprecated in place; readers consult `replay_grade`.
- Replay's side of the contract: a recorded topic outside the bag's embedded
  graph marked `source: discovered` lands in the unmodelled class and is
  skipped with exactly one warn, never a bag/graph-mismatch refusal. A
  malformed `record_coverage.json` is treated as absent plus a loud warn.

### Producer reconciliation lines

Every producer logs ONE compact `info!` line per topic at shutdown, marker
`producer reconciliation (per topic)`, carrying only structured counters:
`producer_next_sequence`, `producer_initial_sequence`,
`producer_committed_frames`, `frames_dropped_send_fail`,
`frames_dropped_overflow` (plus `node_id` on the node-side line). Two call
sites emit it, both in `cerulion_core`: the host harvest
(`GraphRuntime::log_publisher_reconciliation_stats`, reaching only
`ClosureNodeEntry` publishers) and the node-side teardown path
(`NodeContext::log_reconciliation_stats_at_teardown`, via `impl Drop for
NodeContext`, which is how a `#[cerulion_node]` macro node and a cdylib node
surface theirs; a cdylib's line arrives through its own stderr subscriber).
The `recon_logged` guard keeps each publisher to at most one line. A graph
with no host-reachable publishers logs a `debug!` note instead of a per-topic
line; the terms still print from the Drop path. A build declared
`BuildPurpose::PlanningOnly` (the multi-process supervisor's planning
build, torn down before any worker spawns) marks every context as
surfaced before `init()` moves it, so a runtime that was never meant to run
prints no line; the discriminator is the declared purpose, never the
counters, because a producer that never published is a real reconciliation
fact on a runtime that was meant to run. One grep of the marker collects the
producer side of the reconciliation from in-process and cdylib graphs alike.
What the counters mean, kept here rather than on every line:

- `producer_committed_frames` is `producer_next_sequence` MINUS
  `producer_initial_sequence`: frames THIS RUN committed. The two differ only
  on a restored replay, where the restore seeds the counter so the replayed
  stream continues the recording's numbering; reporting the raw counter there
  would mint the whole seed as phantom loss against bag-side terms that only
  ever saw this run's frames.
- The loss identity against the bag's `record_health.json`:
  `producer_committed_frames <= frames_recorded + frames_lost + headerless +
  dropped_unwritten + frames_dropped_send_fail + frames_dropped_overflow`.
  EXACT equality holds only when every producer-side drop was the topic's
  TAIL (highest committed sequence): a MID-STREAM producer-side drop is ALSO
  counted by bagd as a `frames_lost` gap (a surviving newer sequence reveals
  the hole), so it is double-attributed and the right-hand side over-counts by
  that overlap.
- The producer term derives from a u32 wire counter (wraps at 2^32, about
  49.7 days at 1 kHz); on a longer single-topic run the left-hand side wraps
  while the u64 bag-side terms do not.

Pinned by `publisher_recon_teardown_iox2_test.rs` (the marker, the fields,
the at-most-once guard, the frames-this-run rule on both call sites, and the
planning-only silence with its twin: an execution build that never stepped
still reports its zero count).

## 7. Run directory, run registry, and recorder-attach seams

- **Every `graph run`** (recorded or not) writes a run directory
  `~/.cerulion/runs/<sanitized-graph>-<run_id>/` (`run.json` plus
  `graph.yaml`/`env.json`/`recorder.json`; directory 0700, files 0600) and
  announces itself on the `/__cerulion/runs` registry topic. Run-dir
  bookkeeping is never fatal to the run: any failure is exactly one `warn!`;
  on Drop the run announces `Ending` and removes the directory.
- The run-registry record is one-per-run for its whole life: no
  register/unregister cycle, only `set_state` (`Live` → `Ending`)
  (`crates/cerulion_core/src/transport/run_registry.rs`).
- **`Ending` is a last word, not a durable state**; a fresh-subscriber poll
  structurally misses it. Consumers read it with the long-lived `RunWatcher`,
  never a one-shot gather. Lifetime verdicts: `RunEnded::{Graceful,Vanished}`,
  governed by `RUN_VANISH_GRACE` / `RUN_REPLACED_GRACE` /
  `RUN_ABSENCE_CONFIRMATIONS` / `RUN_OBSERVE_INTERVAL`; the first verdict is
  sticky.
- **Run selection** (`cerulion bag record --run`): the pure `select_run`
  resolves the target. A bare `--run` with exactly one live run attaches to
  it; two or more is a **loud refusal naming them**, never a pick; `--run=NAME`
  naming no live run is an error listing what is live. The flag uses clap
  `require_equals`: `--run=NAME` names a run, while a bare `--run` takes its
  default and every following word parses as a topic.
- **Mid-run ring attach**: trace-ring consumers attaching to a live run use
  `open_at_live` (sets `read_cursor := write_cursor`); plain `open` refuses a
  lapped ring, and the default trace ring laps in minutes on a busy graph.
  `drain_gated`'s `Ok(0)` is ambiguous between "idle ring" and "batch entirely
  before the head-step boundary"; a caller reporting coverage must also read
  `HeadStepGate::discarded()`, with `read_cursor` as the cross-check. A
  departure ring (header `rank == u32::MAX`, the multi-process departure
  sentinel) gets passthrough, never the head-step gate; that decision lives in
  `bagd::ring_head_gate`, deliberately not in the core ring layer.
- **Two recorders on one run is allowed** and mechanically safe: ring
  consumers keep local read cursors and the producer never reads consumer
  state. One bag = one run; one run may have several bags. A restarted run is
  a NEW run (new `run_id`, new ring generations); a bound recording ends
  with its run, never splices across a restart.
- **Termination semantics**: the signal path is final drain → attachments →
  success; a bound run ending (gracefully or by vanishing) finalizes the bag
  and is not a recorder failure; the recorder returns with the shutdown flag
  never set. Finalize failure is the failure path; a torn tail is the crash
  net, never a normal exit.
- **Every MULTI-PROCESS `graph run` provisions per-rank scheduler-trace rings,
  recording or not.** The supervisor's gate is `!no_rings`, never
  `record.is_some()`: trace rings are a property of the run, not of whether it
  records. The gate is a STRUCTURAL pin, not a convention: the stamp sits in a
  block whose governing condition is exactly the flag, so any `record` test
  placed around it becomes the innermost governor and fails
  `the_supervisor_stamps_trace_rings_without_asking_whether_it_records`. The
  departure ring rides the same decision, because a worker dies on a plain run
  too and the record that says so is what stops a capture reading as a clean
  roster. Where the rings exist, a Flashback capture and a `bag record --run`
  attach on the DEFAULT run shape both read them, so a capture on that shape
  carries a scheduler trace and not only frames.
- **"Provisions" is the INTENT; whether a ring exists is a separate fact the run
  reports.** Two things still stop one being created, and neither is a bug: a
  deliberate `graph run --no-rings`, and the `/dev/shm` FREE-SPACE gate
  (`gate_trace_ring` → `TraceRingGate::{Allowed, Refused}`). The gate is asked
  TWICE: once by the supervisor over the whole deployment, before any rank has
  reserved anything, and again by each worker for itself, because the
  supervisor's reading predates every allocation and a run whose earlier ranks
  used the space up must refuse the later ones rather than SIGBUS on first
  touch. A refusal is never fatal: the graph runs normally, the decision becomes
  `TracePlaneDecision::Unavailable`, and one loud warn says the captures will not
  be re-executable. Read the OUTCOME off the run's own report, and mind WHICH
  artifact you are reading; the two spell it differently, and there are two
  files called `run.json`:
  - the RUN DIRECTORY's `~/.cerulion/runs/<run>/run.json` carries
    **`trace_rings`** (`run_dir.rs`), whose grammar is `declared` / `declined:
    <reason>` / `unavailable: <reason>` / absent-is-unknown;
  - the BAG's `__cerulion/run.json` carries **`trace`**, with `state_rings`
    as its sibling (`bag_cmd.rs`).

  `docs/bag.md` holds the full state table. Do not infer "a trace exists" from
  the run shape alone.
- **The MONOLITH shapes mint NO ring, deliberately**: a PLAIN
  `--single-process` run, `ros2 attach`, `node run`, and any run that does not
  reach the supervisor. Their captures report no trace and are marked
  not-resimmable; routing them onto the recording clock's discipline is NOT
  implemented. The stated reason is a wall-driven gating clock, whose trace would
  carry step boundaries a resim cannot re-advance to, which is the reason for
  the REAL and `external` monolith shapes; a `virtual` monolith mints none
  because it is a monolith, not because `VirtualClock` is wall-driven.
- **`--time-source virtual` is NOT itself a no-ring condition**, and reading it
  as one is the easy mistake here. `resolve_deployment` rejects only
  `TimeSource::External` for multi-process, so a `process_groups:` graph under
  `virtual` on Unix still routes to the SUPERVISOR and mints its rings like any
  other multi-process run. What mints nothing is the `virtual` MONOLITH. The
  deployment decides, not the clock flag.
- **`--single-process --record` is the exception to the line above**, and the
  word "plain" there is load-bearing: a single-process RECORDING run creates a
  rank-0 ring for its own recorder and declares it. Its ring rides `--record`,
  NOT the `!no_rings` gate; that gate and the `/dev/shm` check above are the
  SUPERVISOR's, so neither statement about them transfers to this path. Both
  record paths declare what they create
  (`both_record_paths_declare_the_trace_rings_they_create`), but they do not
  declare the same VOCABULARY: the supervisor needs the general form because it
  can report `declined` or `unavailable`, whereas this path only ever has rings,
  so it declares them directly. Read the state, don't assume it.
- **`--no-rings` and `CERULION_FLASHBACK=off` are ORTHOGONAL**, and neither is a
  synonym for the other. The env kills the CAPTURE plane (arm word, state rings,
  anchors, window recorder) and leaves the trace rings up, so a later `bag
  record --run` still gets a scheduler trace. The flag kills the trace rings
  and, with them, the Flashback window recorder: with no rings nothing captured
  could be re-executed, so the run takes NO captures rather than frames-only
  ones, and "every capture is re-executable" keeps zero
  exceptions. `--no-rings` conflicts with `--record` at parse time.

  **`--no-rings` is an option of `graph run` ALONE.** `cerulion ros2 attach` and
  `cerulion node run` define no such flag, so passing it there is an
  unexpected-argument PARSE ERROR, not an inert flag; they reach a graph run
  internally and mint no ring on the monolith shapes, but there is no flag on
  them to say so. Within `graph run`, on the shapes that mint no ring anyway it
  is still NOT inert: a `--single-process` or `--time-source external` run has
  its window recorder stopped by it, so that run takes NO captures. Only two
  shapes are true no-ops: a `--time-source virtual` MONOLITH (it starts no
  recorder either way), and any NON-UNIX build, where the recorder, the run
  directory and the `flashback` verb are all `#[cfg(unix)]` and there is nothing
  for the flag to stop.
- **An absence or refusal message names the CAUSE, never another verb's flag.**
  A bag or a capture is read long after its command line scrolled
  away, so the shape is "this run declined scheduler-trace rings at launch", not
  "re-record with `--record`". `flashback_plane`'s `HANDOFF_TRACE_NONE_*`,
  `bag_cmd`'s `TRACE_*`/`STATE_RINGS_*` and `replay_cmd`'s
  `BagNoSchedulerTrace`/`BagNoStepBoundaries` all follow it, each pinned both
  positively (the cause text) and negatively (no `--record`/`--no-rings`).
- **`run.json` carries `state_ring_consumer`**, a tri-state STRING
  (`"standing"` / `"none: <reason>"` / absent-is-UNKNOWN) read by
  `run_dir::run_manifest_state_ring_consumer` into `StateRingConsumerReport`.
  `RUN_MANIFEST_VERSION` stays 1 (additive keys do not bump it). It is written
  by a THIRD manifest edit, after the window-recorder decision is taken: that
  decision is not knowable when the descriptor is written, and a best-effort
  spawn can fail. `run_dir::edit_run_manifest` is the ONE read-modify-rewrite
  shell both in-place manifest writers use; what it genuinely shares is the 0600
  re-assert, which `write_artifact`'s `OpenOptions` mode cannot supply on a
  rewrite.
- **`bag record --run` REFUSES state-ring discovery when the run reports a
  standing recorder.** Per-rank state rings are `OverrunPolicy::Backpressure`
  (one shared cursor slot), so a second consumer laps the first, and that first
  consumer is the run's own black box. The attach logs one loud line, leaves
  `state_ring_discovery_tag` unset, and the bag's `__cerulion/run.json` carries a
  `state_rings` verdict beside `trace`. Only a claim this build UNDERSTANDS
  declines: absent, unparseable and unrecognised all PROCEED and say so.
- User-facing doc for the whole capture plane (the verb, the window, what a
  capture carries, the two switches, the cost, and the known limitations) is
  `docs/flashback.md`.

## 8. Test map

`cerulion_bag` (parallel-safe; `cargo test -p cerulion_bag`):

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `bag_determinism_test.rs` | Same input twice ⇒ byte-identical files; channel ids from sorted topic names, not registration order | no | none |
| `bag_roundtrip_test.rs` | Write → read back via the upstream `mcap` crate AND our reader, against hand-built payload/attachment/trace oracles | no | none |
| `bag_crash_recovery_test.rs` | Torn tail recovers all complete chunks; corrupted chunk body detected, never returned | no | none |
| `bag_validation_test.rs` | Reserved-prefix / unknown-topic / duplicate-topic rejections; empty finalize still yields a valid bag; descriptor byte oracle | no | none |
| `bag_writer_syscall_test.rs` | `writev` batching (IOV_MAX), partial-write resume at adversarial boundaries, EINTR retry, no-progress hard error | no | none |
| `bag_channel_provisioning_test.rs` | Per-channel provisioning metadata round-trip through a real file, read by both readers | no | none |
| `bag_schema_catalog_test.rs` | The schema-provenance attachment's file-level contract | no | none |
| `advise_completeness_test.rs` | The completeness pre-scan's batched advise-behind memory bound | no | none |

`cerulion_bagd` (serial: `cargo test -p cerulion_bagd -- --test-threads=1`;
a process-global panic hook, not shared memory; each test builds an isolated
per-instance transport):

| Test file | What it pins | Serial? | Prereq fixtures |
|---|---|---|---|
| `e2e_test.rs` | In-process `run_bagd` end-to-end: hand-built frames/trace records, independent `mcap` re-read, backlog-aware drain no-loss, signal-path finalize | yes | none |
| `wire_gap_frame_e2e_test.rs` | rmw borrow-window gap-frame byte-fidelity: a page-aligned, gap-carrying frame records + reads back byte-identical (both readers), wire stamps verbatim | yes | none |
| `discovery_e2e_test.rs` | Coverage (an undeclared live producer is recorded) + disclosure (an unrecorded one is named with its reason); tap-attach rendezvous | yes | none |
| `discovery_event_e2e_test.rs` | Enumeration is event-driven: every walk has a directory change behind it while the timed engine's are uncaused, a real topic still wakes the worker, a burst is coalesced, and an unarmable watch degrades loudly to the timed walk | no | none |
| `prefix_loss_e2e_test.rs` | The head-loss marker: `prove_prefix_loss` + `armed_before_producers` gating, `is_incomplete()` escalation | yes | none |
| `run_lifetime_e2e_test.rs` | A bound recorder finalizes when its run ends without ever being signalled; the unbound control keeps that meaningful | yes | none |
| `schema_resolve_e2e_test.rs` | Channels named from the wire hash via the local corpus; wire-size derivation; explicit `Unresolved`; the non-waiting-verb source walk | yes | none |
| `firehose_bench.rs`, `go2_firehose_bench.rs` | `#[ignore]`d throughput measurements, not gates | n/a | none |

Related pins elsewhere:

| Test file | What it pins |
|---|---|
| `crates/cerulion_core/tests/run_lifetime_iox2_test.rs` | The `RunWatcher` verdict logic over real iceoryx2 (last word survives; a successor does not keep a binding alive) |
| `crates/cerulion_cli_engine/tests/bag_record_run_attach_test.rs` | `--run` mid-run attach over a REAL lapped live trace ring (the `open_at_live` path plain `open` refuses); the `*` arms are the attach half of the `state_ring_consumer` gate over hand-written manifests (refuse / proceed / could-not-tell), each asserting the bag's verdict AND whether the recorder really swept (read off `BagdSummary::state_coverage`) |
| `crates/cerulion_cli/tests/plain_run_resim_e2e_test.rs` | The RUN half of the `state_ring_consumer` gate over the real binary (`a_run_writes_its_window_recorder_decision_into_run_json`): without it a build that never writes the key leaves every attach on the UNKNOWN arm, which PROCEEDS, so the feature would ship inert with the attach suite green |
| `crates/cerulion_core/tests/notify_elision_iox2_test.rs` | A data-only recording tap keeps publisher notify-elision armed (a listener-full tap disarms it) |
| `crates/cerulion_cli/tests/mp_record_e2e_test.rs` | Multi-process `graph run --record` acceptance over the real binary: one finalized bag, per-rank manifests, departure sentinel |
