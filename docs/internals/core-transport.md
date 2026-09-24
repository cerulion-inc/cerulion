# cerulion_core internals: transport, liveness, latches, gateway, network

Present-tense contracts for `cerulion_core`'s transport plane. Read this before modifying
anything under `src/transport/`, `src/wire.rs`, `src/wake.rs`, or the network/gateway
code. Companion: `core-scheduler-graph.md` (execution side), `core-testing.md` (full test
map). Code on `main` beats this document; when they disagree, fix the document.

## Transport model

- ONE iceoryx2 node per process, owned by the `TransportManager` singleton
  (`get_or_init()`). Tests get isolation via per-test SHM roots
  (`init_for_test` / `build_for_test` / `generate_isolated_config()`), not via a second
  node.
- `AnyPublisher` / `AnySubscriber` (`src/graph/node.rs`) are single-variant enums
  (`Ipc(...)`), not type aliases. The one-arm match is the dispatch seam a second
  transport backend plugs into with zero call-site churn, and the dispatch has to keep
  three properties while growing through it: no `dyn Trait` vtable, no heap allocation
  for the dispatch itself, and no `Arc<dyn Trait>` carried across a cdylib boundary,
  where a vtable pointer minted in one linkage unit is not safe to use in another.
- **One consumer read path**: every read goes through the iceoryx2 queue receive in
  `src/transport/subscriber.rs`. A single path is what keeps drop_oldest
  accounting-once, `sample(N)` decimation, cross-step held borrows, Err-replay,
  zero-alloc, and polled==live each proven once and maintained once, and it is what
  keeps a frame's shared-memory lifetime governed in one place. A bypass or raw-handle
  consumer read over the queue plane is not accepted: a second read path doubles those
  invariants and re-opens the use-after-reclaim class. The queue receive is also what
  buys the always-on observability plane (taps, `topic echo/hz`, recorder), so it is not
  overhead to optimize away in isolation. The lever for lower intra-process latency is
  chain-fused synchronous execution (see `core-scheduler-graph.md`).
- There is NO network path below `AnyPublisher`. Publishers do not perform network
  fan-out: a graph process does no post-send network work (stated at the send site in
  `transport/publisher.rs`), and a publisher carries no network config of its own. The
  SEPARATE gateway process owns the whole network plane, tapping produced topics for
  egress via listener-less data-only subscribers (see §Gateway and network plane).
- Macro-generated node entries (`<Name>Entry`, wrapping the user struct + a context,
  the `NodeEntry` implementors that hold these ports): `new()` constructs via
  `<Struct>::default()`, so it REQUIRES the node struct to implement `Default`; the
  macro injects `#[derive(Default)]` when the user has not derived it, so the bound
  surfaces as a derive error on any FIELD type lacking `Default`; `with_state(inner)`
  constructs with caller-supplied initial state (the seam for pre-seeded node state in
  tests and hosts).

## Wire format and framing

- 32-byte little-endian `WireHeader` (`schema_hash`, `total_size`, offset-table
  offset/count, `sequence`, `timestamp_ns`). The canonical schema-hash recipe lives in
  exactly ONE place: `MessageSchema::schema_hash` (`src/codegen/schema.rs`). Never
  re-implement it. The recipe identifier is the `HASH_RECIPE` constant
  (`src/trace/bag.rs`); an artifact with an absent recipe field means the original
  recipe.
- Writer cursor contract (for anyone writing an independent frame reader): the
  variable-payload write cursor starts at `wire_fixed_size + 8·N` and only advances
  forward; a re-written variable field is appended at the tail, leaving interior dead
  bytes. `OffsetEntry.offset` is measured from `payload[0]` (immediately after the
  header), NOT from the start of the variable-data section.
- Alignment padding is initialized: primitive-array writers zero the bytes between the
  previous cursor and the aligned cursor before committing it, on the typed loan, slice
  set, first push and successful `fill_from` paths, and only after the capacity check
  (and any overflow spill) has succeeded; a failed or panicking producer commits no gap.
  Recycled slot bytes therefore never enter a published payload through an alignment
  gap, and byte-exact comparison sees a stable value there. Producers must still
  initialize every element they loan. Recordings made before this rule can carry
  nonzero gap bytes and may differ under byte-exact verification once their nodes are
  rebuilt; decoded field values are unchanged.
- Canonical element framing (arrays of nested messages, `string[]`): a self-describing
  encoding inside the offset table's variable payload: recursively-fixed elements ride
  back-to-back at the padded stride (no count); variable elements ride
  `u32 count` + per-element `u32 len` + headerless sub-frame; `string[]` rides
  `u32 count` + per-element `u32 len` + UTF-8. Both production ingress bridges (the DDS
  attach path's `CdrCodec::decode` and the rmw introspection bridge) build element bodies
  through the ONE shared `codegen::element_codec`; cross-validated by
  `crates/rmw_cerulion/tests/canonical_element_body_test.rs` (rmw encodes, the independent
  walker decodes, against hand-built byte oracles).
- `FrameWalker` decodes element framing with total, strict validation: non-canonical
  bytes degrade to `NestedArrayOpaque` (a loud text fallback), never a guessed element
  list. Its hostile-count budget (`count > remaining / 4` refuses) is output-equivalent;
  what it buys is WORK, so its pin (`frame_walker_count_budget_test`) uses an ALLOCATION
  oracle, not the verdict. Framing changes with an unchanged schema hash need a PAIRED
  producer/consumer rollout: nothing on the wire signals the framing vintage, and a hash
  skew (unlike a framing skew) is silent; no fallback renders.
- The codegen'd producer API for `Nested[]` / `string[]` fields is bytes-only
  (`<f>_bytes` / `loan_` / `set_` / `fill_from_` variants). Typed push accessors
  (`push_<field>(&T)`) are not supported.

## Publisher contracts

- **Commit-time sequence**: the wire `sequence` `fetch_add` lives in `OutputProxy::Drop`'s
  send paths; a discarded loan burns no sequence, so published streams are gap-free
  (pinned in `output_proxy_test`). Gap-free-at-commit holds on BOTH production publisher
  paths: `OutputProxy::Drop` AND the raw-frame ingress route (the DDS-attach bridge's
  `RawIngressRoute`), which owns its own commit counter; the rate estimate's
  sequence-delta numerator is valid only because both paths are gap-free. `publish_raw`
  itself writes the caller's header VERBATIM, so a caller-stamped sequence (the
  gateway/netd mirror re-injectors, the rmw serialized path) carries whatever the caller
  supplied; the rate estimate's floor fallback exists for exactly that case. General
  rule: any counter consumed at resource-ACQUISITION time leaks through every abort
  exit; consume at commit. The symptom of getting this wrong is phantom loss:
  sequence-gap detectors fire at the abort rate while completed-work counters match
  recorded artifacts exactly.
- **Self-drain**: the notifier delivers to EVERY listener on a topic's event service,
  including the publisher's own (each `CerulionPublisher` owns one for
  `SubscriberConnected`). `loan_proxy` and `publish_raw` both call
  `check_subscriber_events()`, which is GATED on the topic's live listener count: a
  change in that count arms a bounded run of drains, a drain that sees a transition
  disarms it, and a steady topic pays one relaxed load per publish. Under iceoryx2
  0.9.1 the drain was unconditional because an undrained listener filled its own
  AF_UNIX socket, after which every notify failed and was logged once per publish (a
  disk-filling flood on a many-topic robot); 0.10 removed that failure mode, and the
  remaining job of the call is late-joiner history. Oracle for a notify that does not
  reach every listener: `CerulionPublisher::notify_undelivered_count()`
  (log-level-independent); off-thread via `NodeHandle::notify_undelivered_count(output)`.
  Pinned by `notify_shortfall_iox2_test`, whose apparatus arm kills a consumer process
  to prove the condition is still reachable.
- **Notify elision**: the per-publish notify is skipped while the topic's live listener
  count equals the count the runtime has proven it owns; over- or under-count both fail
  safe to never-elide. A foreign listener attaching resumes notifies within one publish
  (the per-publish recheck is the self-heal); a bounded heartbeat notify is the
  wedge-proof backstop; kill switch `CERULION_NOTIFY_ELISION=off`.
  `CerulionPublisher::unannounced_publish` is the DEBT bit: set by an elided publish or
  an off-gate `publish_raw`, cleared by any real notify (a notify announces everything
  committed before it). The live-loop boundary resweep (`resweep_notify_elision`) fires
  only when debt is outstanding AND a foreign listener is present, and firing SPENDS the
  debt; the `live_n <= expected_n` early return deliberately does not consume it (that
  state IS the late-attach shape). After touching elision or the resweep, run
  `cargo test -p cerulion_core --test notify_elision_resweep_iox2_test --test
  notify_elision_iox2_test -- --test-threads=1`.
- **Late-joiner history** is iceoryx2-native (`update_connections` / `pump_history` on
  the transport); there is no Cerulion-side heap history buffer. Subscriber
  `buffer_size` must be >= the requested history.
- **Incomplete-output discard** (`OutputProxy` dropped with unwritten declared fields):
  loud first-of-regime `error!`, sustained repeats downgraded to `debug!`, recovery
  `info!` only when something was suppressed (`OutputDiscardLatch`). The unconditional
  `total_discards` / `CerulionPublisher::output_discard_count()` counter never resets on
  recovery and is readable off-thread via `NodeHandle::output_discard_count(output)`.

## Provisioning

- iceoryx2 0.9.1 pool formula (`service/static_config/publish_subscribe.rs`):
  `samples_per_segment = max_subscribers × (buffer + borrowed) + history + loaned`.
  Slots are refcount-shared across connections; the `× max_subscribers` term is a
  worst-case wait-free bound.
- Declared `#[input(depth = N)]` buffers are behaviorally REAL (a 16-buffer retains 16
  frames); the topic ceiling is topology-derived. An opener requiring more than the
  ceiling is rejected by iceoryx2 itself, wrapped with an actionable message
  (`topic_buffer_sizing_test`).
- Graph-owned topics provision `max_publishers = 1` (single-writer); a rogue second
  publisher fails at port creation naming the contract. Degraded opens (pre-existing
  service with looser caps) warn exactly once per (topic, kind). Cross-graph same-topic
  publishing is refused by the active-publisher pre-check even when a pre-existing
  service's port caps cannot enforce it (`cross_graph_collision_iox2_test`).
- `max_subscribers` provisioning is exact: in-graph consumers + trigger-drain terms +
  `INTROSPECTION_SUBSCRIBER_HEADROOM` spare tooling slots (the headroom includes the
  standing liveness observer's slot; changing either number requires revisiting the
  other). Event-service listener/notifier caps derive from the LIVE data service's
  subscriber + publisher terms.
- Wake-listener budget: a graph-owned topic's EVENT listener slots are its DATA
  subscriber slots + 1, so any tap that can exist can always hold its wake listener:
  the data cap binds first; the wake listener is never the binding refusal.
- Holding multi-sample bursts needs `subscriber_max_borrowed_samples >= 3`, so owned
  topics CREATE their services at the borrow floor (two-phase open-then-create in
  `open_topic_services`). The floor is CREATE-side only, never an owner open
  requirement; a foreign shallow-borrow service is rejected loudly at build with the
  borrow-floor hint (`non_trigger_hold_iox2_test`).

## Taps and introspection

- `TransportManager::create_subscriber_open_only`: structurally cannot create services
  (data `.open()` gates first; a missing topic errors leaving ZERO services; no phantom
  event services). `topic echo`/`hz`/`info` route through it.
- `DataOnlySubscriber` (`create_data_only_subscriber`): opens ONLY the data service (no
  listener, no notifier) and is un-attachable to a WaitSet BY TYPE (durable
  compile_fail doctest in `subscriber.rs`). Capture taps (recorder, gateway egress,
  liveness) are data-only so a producer's `number_of_listeners()` never grows and zero
  failed notifies is structural. Taps have NO history: frames published before
  tap-attach are gone (drop-to-live is correct for remote viewing; tests must
  attach-then-publish). Per-tap errors are per-topic NON-FATAL: forward the partial
  drain, drop the tap, re-attach fresh.
- `DataOnlySubscriber::has_samples()` MUST stay non-consuming: the gateway's
  one-budget-per-pass egress drain closes its liveness baseline on this emptiness query
  against a queue it is still forwarding from. A readiness probe must not consume a
  frame: a consuming probe answers the same boolean while moving the data plane, so
  which frames a pass forwards, and when, would depend on whether anything asked.
  Pinned by `data_only_tap_iox2_test`.
- `src/wake.rs`: `WakeSource` (listener-only handle on `{topic}/event`, minted only by
  `TransportManager::create_wake_listener`) + `WakeSet` (block on N sources, fired
  indices ascending), a thin face over the WaitSet reactor. Wake-loop discipline: a
  wake is a SIGNAL, never a count: drain BEFORE waiting; the timeout keeps the loop a
  strict superset of polling; pace on `last_wait_blocked()`; an EMPTY source slice does
  not block. A wait that returns instantly with no deliverable frame must be paced
  before the loop goes round again: a topic whose event service is rung faster than it
  publishes returns every wait at once, and an unpaced loop then spins without bound.
  Pinned by `wake_set_iox2_test`.

## Flood-suppression latches

- `transport/failure_regime_latch.rs` is the repo's ONE shared suppression machine:
  `FailureRegimeLatch` + level-agnostic
  `RegimeDecision::{Loud, Suppressed, StillFailing}` (sites disagree on whether the loud
  arm is `warn!` or `error!`). Contract: loud first-of-regime; `debug!` repeats carrying
  the running suppressed count; a LOUD re-announcement at each DECADE of the running
  total; recovery `info!` once per regime iff something was suppressed; an UNCONDITIONAL
  `total_failures` never reset by recovery. Pure: callers map decisions onto `tracing`.
- The decade ladder is clock-free (a clock would make emission sequence wall-dependent,
  against replay determinism) and log10-bounded (a week of kHz failures ≈ nine lines).
  It exists because rmw-side counters sit behind the standardized rmw C ABI; no
  accessor can exist there, so the log is a ROS user's only window. A re-announced
  failure is not counted as suppressed; recovery reports exactly what was missed; a
  recovery never buys a still-broken peer a fresh quota.
- A diagnostic latch never wedges the path it observes and never silently resets its
  regime on a poisoned mutex (`lock_regime_latch` arm).
- Three older latches predate the shared machine, each serving one site:
  `graph/drain_latch.rs` (pre-step drain warnings), `transport/output_discard_latch.rs`
  (incomplete-output discards) and `transport/notify_delivery_latch.rs` (notify-delivery
  shortfalls). They stay as they are. A new failure regime builds on
  `FailureRegimeLatch` rather than migrating one of the three or hand-writing a fourth.
- `transport/frame_drop_latch.rs` is the reporting layer for per-frame take-path drops,
  split into two independently-latched conditions: schema-hash mismatch (definition
  disagreement) and short envelope (framing disagreement), each on its own latch so
  neither regime swallows the other's loud head. `FrameDropSite` chooses the noun AND
  the structured field key: `topic=` for messages, `service=` for services; operators
  grep by key. Every consumer of the shared machine logs the total as
  `total_failures=` (one grep answers "how bad has this got"); WHICH condition is
  carried by message text + `kind=`.
- `transport/notify_delivery_latch.rs` adds `ListenerCountTiming`: the caller DECLARES
  when it read the listener count. Read BEFORE the notify → believed immediately (an
  attaching listener cannot inflate it). Read AFTER → must persist across two classified
  notifies before warning/counting (an attaching listener in the notify→read window
  fabricates a shortfall on a healthy graph); the armed suspicion forces the next
  classification. The seam takes a `NotifyListenerCount` enum, never `Option<usize>`:
  the timing is carried by the variant, never inferred from whether a count is present.
  The
  cost, stated in the module docs: on a quiescent unarmed producer the counter reads 0
  until the next publish; 0 means "nothing CONFIRMED", not "nothing seen".
- Test discipline for all latch pins: every predicate asserting a suppression line's
  presence or count matches the LEVEL TOKEN as well as the message; a text-only filter
  passes a variant where the suppressed arm emits at the loud level.

## Data-flow liveness (`transport/liveness.rs`)

- Why it exists: a DDS-attach bridge registers one publisher per discovered topic at
  build time, so publisher counts read the same for a streaming and a dead route;
  liveness must be measured from observed frames, not port counts. The headline test
  asserts both facts in one body (`topic_liveness_iox2_test`).
- `TopicLivenessObserver` holds long-lived, budget-bounded, listener-less
  `DataOnlySubscriber` taps feeding a shared `LivenessTable`; the gateway's catalog verb
  stamps `TopicLiveness { last_frame_age_ms, observed_for_ms, frames_observed }`,
  classified by the pure `LivenessState::{Streaming, Idle, NoData, Unknown}`. Taps must
  be long-lived: a tap attached at serve time drains zero already-published samples (the
  late-joiner pump rides the event service, which a data-only tap never opens); retained
  history reaches a fresh tap only when the publisher's next send (or another
  subscriber's connect) runs `update_connections()`. Kill switch:
  `CERULION_TOPIC_LIVENESS=off` (exact match, `LIVENESS_ENV`) disables the observer
  entirely; absent/empty observes silently; ANY other value leaves observation ON with a
  loud warn; a typo never silently disables (or silently keeps) the observer.
- **Dating rule (stamp advancement)**: publisher clock vs publisher clock, never the
  observer's. Each record keeps `max_stamp_ns`; a batch dates the topic only when its
  newest stamp EXCEEDS that maximum. The FIRST batch after any attach is a BASELINE
  (banked into `frames_observed`, never dated; it could be a retained-history flush);
  the baseline stays OPEN until a drain observes an empty queue
  (`DrainObservation::queue_emptied`, OBSERVED via `has_samples()`, never inferred from
  `drained < budget`, which is unreachable at budget 1). Ages are observer-clock
  arithmetic; the shipping deployment has publisher and observer on unrelated clocks.
- **Epoch reset**: `max_stamp_ns` is not monotone; a stamp regression is evidence of a
  new clock epoch (restarted worker whose virtual clock restarts at zero). The reset
  fires on `regressed && !baseline_open && (quiet_long_enough || sustained)`. The
  silence gate is `now − last_nonempty_drain >= REGRESSION_RESET_MIN_GAP_NS` (twice the
  sweep interval; zero-frame drains do not re-anchor it). `sustained` requires ALL of:
  every drain in the window saw exactly ONE publisher; at least
  `SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS` emptied drains; span >=
  `SUSTAINED_REGRESSION_MIN_SPAN_NS` (const-asserted strictly greater than the silence
  gate, so it can never pre-empt it); ascending stamps. Both guards exist because a
  re-flush chunk and a restarted run are stamp-for-stamp indistinguishable: adopting a
  re-flush prefix lowers the bar and a later chunk of the same backlog would date a dead
  route. A lull-free single-writer restart heals in ~1 s + one advancement on BOTH
  planes (pins: `a_lull_free_restart_dates_again_within_the_confirmation_window`,
  `gateway_iox2_test::a_lull_free_restart_resets_the_epoch_on_the_demand_plane`).
  Epoch reset does not cover `multi_publisher_topics` topics, or a topic whose drainer
  is saturated at restart time.
- Degradation: UNKNOWN (`None`) is never conflated with dead; a report requires a
  currently-active observation (un-attachable tap, past-budget, disabled observer, LOST
  tap all serve `None`, never a frozen verdict). `frames_observed > 0` forbids `NoData`
  (a produced topic is at worst `Idle`). Budget policy: incumbents are never preempted;
  slots ARE recycled (a hand-off or drain failure frees one for the canonically-first
  tapless topic).
- **Rate estimate** (`TopicLiveness::rate_estimate:
  Option<TopicRateEstimate { millihertz, is_floor }>`, additive on the wire; absent
  decodes to `None`, never a fabricated 0 Hz): the numerator is the publisher's commit
  SEQUENCE delta (frames drained are clipped by the shallow tap; a frames basis pins
  every fast topic at the derived `RATE_FLOOR_BASIS_CEILING_MHZ`); the denominator is
  the OBSERVER's clock (a count belongs to no clock domain; publisher stamps cannot
  serve; a worker's gating clock is logical, not wall). A stalled-sequence window
  (`publish_raw` writes the caller's header verbatim) falls back to counting frames and
  sets `is_floor`; render as `>= N Hz`, never a measurement. Any drain seeing other
  than exactly one publisher (`writers_seen`, UNKNOWN included) closes the window on the
  floor basis; the trust bit re-arms per window. Windows: minimum span
  `RATE_ESTIMATE_MIN_WINDOW_NS`, capped by `RATE_ESTIMATE_MAX_AGE_NS`; a
  silence-spanning window is DISCARDED (never a diluted average); a rate is served only
  while the topic classifies `Streaming`; a stopped stream drops its rate rather than
  decaying one. Window guards: a baseline burst (possible retained-history flush) and an
  epoch reset only RE-ANCHOR the window (they contribute nothing), so a dead route's
  flushed backlog never renders as a live stream; a sequence regression beyond
  `RATE_SEQ_RESET_TOLERANCE` discards its window (a restart or counter wrap), while
  in-tolerance backward jitter is absorbed by a saturating delta. Accepted cost: topics
  with period between the serve horizon and twice it read `Streaming` most of their
  period while carrying no rate. The desk side speaks the same field: vizd's own tap
  (`TopicStat::liveness`) carries its wire-sequence rate (`desk_rate_estimate`, never a
  floor) on `discover`/`list`/`status` rows alike, so a tapped LOCAL topic reports the
  number `status.hz` measures rather than UNKNOWN; untapped local topics stay UNKNOWN.
- Cost contract: zero publisher-hot-path cost; exactly ONE gateway subscriber port per
  topic: a demanded topic's liveness rides the EGRESS tap (`set_externally_observed` +
  `note_frames`), and `drive_once` releases the observer's tap BEFORE the egress attach
  so egress can never be starved. A dead egress tap ends
  the observation riding it (UNKNOWN, not frozen). Observer taps open at
  `LIVENESS_TAP_BUFFER_SIZE`, not the topic ceiling, so `frames_observed` is a true
  lower bound while ages stay exact.

## Gateway and network plane

- A graph build starts NOTHING network. The graph process is network-free; the gateway
  is a SEPARATE process driven from the pure `compute_gateway_plan` → `GatewayRuntime`.
  Plan derivation: explicit `network:` block ⇒ allow-list egress + announces + resolved
  ingress table; permissive ⇒ allow-all + all produced topics announced; an ingress
  topic nobody consumes refuses loudly naming the topic + fix
  (`network_graph_wiring_test`).
- `GatewayRuntime` owns one graph's network plane: announces produced topics, hears
  remote DEMAND tokens, forwards demanded topics' SHM frames to zenoh via listener-less
  taps (egress), re-injects declared ingress topics into local SHM. The zenoh payload is
  the Cerulion wire frame VERBATIM (no envelope); ONE network task serves all topics.
- Ingress validates `total_size` + `schema_hash` on every received frame BEFORE
  re-injecting; a mismatch is counted (`ingress_stats`), never delivered. Re-injection
  is byte-identical including header `sequence`/`timestamp_ns`.
- The egress/ingress loop is STRUCTURALLY impossible, not policed: a normal publisher is
  network-free and registers no flag; only the gateway registers a flag, only for an
  announced egress topic; an ingress publisher never taps out. Ingress+egress on one
  topic is refused loudly; ingress on a topic owned by a live single-writer producer is
  refused (the slot is taken).
- `register_ingress_topic` / `unregister_ingress_topic`: the session is lazy (first
  register opens it and starts the liveliness watch); the watch is latched-idempotent
  (refcounts never stack); teardown releases the mirror's local data service (freeing
  the single-writer slot), removes the topic from the `self_ingress` exclusion set, and
  the full register→unregister→register cycle succeeds. Unregistering a never-registered
  topic and double-unregister each error loudly.
- Mirror provenance (`transport/mirror_registry`): re-injectors register
  `(canonical topic, origin robot)`; the fold (`MirrorStreamRow`,
  `partition_local_topics`, `attribute_local_topic`, `merge_attribution`) lives here,
  in core, not the CLI, because multiple consumers need the same fold. Interactive
  surfaces use the unchecked windowed gather (transient staleness self-heals); any
  DURABLE artifact must use `gather_mirror_provenance_checked` →
  `MirrorGather { records, completeness }`; an empty windowed-listen answer is NOT
  evidence of absence. A writer with an empty record map still sends a bare PRESENCE
  frame so it can be heard; older readers count it malformed and ignore it (intended).
- Run registry (`transport/run_registry.rs`, `/__cerulion/runs`): exactly one record per
  run for its whole life: no register/unregister, only `set_state` (`Live` → `Ending`).
  `Ending` is a LAST WORD, not a durable state: a fresh-subscriber poll structurally
  misses it; consumers read it with the long-lived `RunWatcher`, never a one-shot
  gather.
- Cross-machine data-plane diagnosis: `crates/cerulion_core/tests/cross_machine_data_plane_harness.rs` is the
  standing two-machine A/B harness (env-driven robot/desk roles; ignored, never run in
  CI; run per its module docs on a live machine pair). It models the shipping asymmetric
  link (the robot LISTENS, the desk DIALS) and MEASURES whether frames cross. Reading:
  BOTH counters climbing (the robot-local `forwarded` and the desk-side `sub_received`)
  means the data plane is healthy end to end; desk `sub_received == 0` while the
  robot-LOCAL `forwarded` climbs localizes the break to the wire/route (or the
  re-inject), NOT the producer or the SHM tap.
- zenoh operational facts the plane is designed around: liveliness subscribers see only
  post-subscription declarations unless `.history(true)` is set (without it a restarted
  gateway under an already-declared demand never egresses, silently); link formation is
  order-sensitive: connect AFTER the listener accepts (tests gate on a bounded TCP
  probe first); liveliness tokens carry no payload and liveliness GETs deliver no
  attachments; metadata is KEY-ENCODED and the key space is the version boundary. The
  zenoh version pin is security-motivated: before any bump, check `zenoh-transport`'s
  `lz4_flex` requirement against the RUSTSEC-2026-0041 patched ranges (the requirement
  is `zenoh = "1"`, so bumps are lockfile-only).

## iceoryx2 0.9.1 platform facts (source-verified; pinned `=0.9.1` exactly)

- Events are AF_UNIX SOCK_DGRAM sockets on every target; every wake is a kernel
  crossing; there is no SHM word to monitor-wait on (hence Cerulion's own doorbell, see
  the scheduler dossier). On Linux, dgram queueing is charged to the SENDER (raising
  the receiver's SO_RCVBUF is a measured no-op); macOS/BSD charge receiver-side. Any
  "socket buffer" fix must state which side the target kernel charges.
- There is NO native receiver-side drop counter; eviction is silent in the sender's
  overflow queue. Drop accounting is inferred from wire-sequence gaps (Cerulion stamps
  `sequence` per publisher). Subscriber queues are per-(publisher,subscriber)
  connections (independent capacity/eviction, strict per-connection FIFO); eviction is
  an unreliable oracle for over-publish assertions; use a shared mirror instead.
- Liveliness is POLL-ONLY (`number_of_publishers()`); a crashed publisher's port lingers
  until dead-node cleanup. Auto dead-node cleanup is DISABLED on every config a
  `TransportManager` node is built from: the liveness probe's same-process fast path
  keys on a per-LINKAGE-UNIT id, so a dlopen'd cdylib's own iceoryx2 copy can judge the
  LIVE host dead (POSIX record locks never conflict within one PID) and reap its
  services. Crash recovery is one explicit `try_cleanup_dead_nodes` sweep at
  host-singleton init; the runtime liveliness sweep uses the explicit API
  (`dead_node_cleanup_config_test`). Any service lifecycle op from cdylib-resident code
  is a reap hazard unless the flags are off; only a real dlopen'd run can catch it.
- `AllocationStrategy::Static` pools are LAZY demand-paged tmpfs: resident RAM = pages
  written, not pool size; oversizing is latency-free; oversize generously. NOT free on
  Windows (eager commit) or under `mlockall` / `RLIMIT_AS`. Shmem-THP inflates residency
  up to 512×, hence the `MADV_NOHUGEPAGE` create-hook.
  `shm_guard_madvise_iox2_test` checks that the hook really applies: it creates an
  oversized iceoryx2 publisher (firing `shm_guard::advise_shm_pools_no_hugepage`), then
  reads `/proc/self/smaps` and asserts every shared `iox2_` pool mapping carries the
  `nh` VmFlag. It needs a real `/proc/self/smaps`, so it is Linux-only and `#[ignore]`d;
  run it by hand on a Linux host. `PowerOfTwo` breaks the zero-alloc gates and has a
  hard realloc-lifetime wall.
- macOS select() path: any fd NUMBER >= 1024 entering a WaitSet aborts the process (the
  fd number, not the count). Host-attached fds are guarded; iceoryx2's internal fds are
  not; keep macOS many-topic recordings modest until the upstream kqueue fix.
- A resident `*_node.global_mgmt.shm_state` segment from a DIFFERENT iceoryx2 version
  blocks node creation with an opaque `InternalError`/`VersionMismatch`; it lives
  outside the usual SHM root and survives prefix-scoped sweeps. Crashed runs also leave
  stale `*.event` sockets under `/tmp/iceoryx2/` (not `/dev/shm`); every publish then
  logs an undeliverable-notify warning per dead listener; sweep both locations.
- `generate_isolated_config()` mints a unique prefix baked into both service paths and
  the node-monitoring registry; a subprocess child must deserialize and reuse the
  parent's exact `Config`. `ipc_threadsafe::Service` is what makes ports `Send`
  (mutex-protected, small per-op tax).

## Key enforcing tests (transport plane; full map in `core-testing.md`)

| Test file | Pins |
|---|---|
| `output_proxy_test.rs` | proxy/view round-trips; commit-time sequence; discard-count observability |
| `notify_shortfall_iox2_test.rs` | undelivered-notify accounting; a killed consumer as the apparatus; latch lifecycle |
| `notify_elision_iox2_test.rs` + `notify_elision_resweep_iox2_test.rs` | elision self-heal gate; boundary resweep debt semantics |
| `data_only_tap_iox2_test.rs` | listener-less tap; non-consuming `has_samples()` |
| `topic_liveness_iox2_test.rs` | dating rule, baselines, epoch reset, rate estimate (observer plane) |
| `gateway_iox2_test.rs` | gateway egress/demand, liveness-over-egress, slot hand-off ordering |
| `topic_buffer_sizing_test.rs` | depth-is-real; subscriber/event-cap provisioning; single-writer |
| `failure_regime_latch_test.rs` | shared latch decisions, decade ladder, frame-drop reporting layer |
| `network_ingress_test.rs` / `network_ingress_e2e_test.rs` | byte-identical re-inject; loop exclusion; cross-session hop |
| `iceoryx2_version_lockstep_test.rs` | exact-pin + single-version workspace resolution |
