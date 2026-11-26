# Replay Determinism Footguns: the G-Category Catalog

A "why does my replay diverge" reference for re-execution (spelled
`cerulion bag play <bag> --resim all --verify`) and
the compile-time determinism lint. Cerulion's replay is **byte-exact**:
a recorded bag is the golden, your current workspace build is the candidate, and
every produced frame is diffed byte-for-byte against the recording
(a replay matches the live run). That guarantee holds **only if every node body
is a pure function of its inputs, its restored state and the framework clock.** A node that reaches
outside that boundary (the wall clock, an unseeded RNG, the filesystem, thread
scheduling) produces different bytes on the replay run than it did at record,
and the diff faithfully reports a `ByteMismatch` (exit 1) it cannot explain away.

This doc catalogs the footgun classes (the "G-category"), why each breaks
replay, and the sanctioned alternative. **Two of them are refused at compile
time** by the determinism lint inside `#[cerulion_node]`: clock reads (G1) and
raw thread spawns (G4). The DENY set is deliberately exactly the symbols that
have no legitimate in-tick use: `Instant::now`, `SystemTime::now`,
`thread::spawn`. Every other class in this catalog is your responsibility: the
table carries warn-class rows too, but the macro does not surface them, so
they produce NO diagnostic. The canonical banned-symbol table lives in
`crates/cerulion_macros/src/determinism.rs`. You opt out per node with
`#[cerulion_node(allow_non_deterministic)]` (blanket) or `uses_live_io` (the
IO-class subset), but opting out means owning the divergence.

Related: `docs/user-api.md` (the clock accessors `self.now_ns()` /
`self.real_ns()` / `self.virt_ns()`), `crates/cerulion_macros/src/determinism.rs`
(the enforced policy), the lint, and the replay engine.

## TL;DR

- Replay re-executes your **current** node builds and diffs their output against
  the recording byte-for-byte. Anything non-reproducible in a tick body → a
  spurious `ByteMismatch` (exit 1) that is NOT a real regression.
- The framework refuses the two unambiguous killers at **build time** (clock
  reads, raw thread spawns) and nothing else. Don't reach for
  `allow_non_deterministic` to silence those; reach for the sanctioned
  alternative below.
- A **schema change** since recording is a different failure: when it moves the
  output type's layout hash (its own fields, or any FIXED-resolved nested
  schema), the replay refuses up front with exit 2 (`SchemaDrift`), naming the
  drifted topics + both hashes. A layout change confined to a VARIABLE-resolved
  nested schema keeps the parent hash stable and still surfaces as an exit-1
  `ByteMismatch` (the pre-preflight path). Re-record, or check out the
  recording-era schemas.
- Cross-**arch/os** replay is allowed but **WARNS** (last-bit float skew); see
  G8 and the `recorder.json` identity check.
- A **multi-process** bag replays under the coordination contract it was recorded
  under, and a FREE-RUN one is re-executed **per rank**, which changes where a
  divergence is attributed; see
  [Free-run recordings](#free-run-recordings-and-per-rank-re-execution).

## The catalog

### G1: `SystemTime::now()` / `Instant::now()` (wall & monotonic clocks)

**Footgun.** Reading the system or monotonic clock in a tick embeds a timestamp
that advances between the record run and the replay run.

**Why replay diverges.** The recorded frame carries record-time's clock value;
the re-executed candidate reads replay-time's: different bytes, every frame.

**Sanctioned alternative.** Read time through the **framework clock**, which
replay drives deterministically off the recorded scheduler trace:
`self.now_ns()` (active-source ns: Real live / Virtual replay), or the raw
wire `timestamp_ns` the runtime already stamps on every publish. `self.real_ns()`
exists for benchmarks/real-time only and is explicitly **not** replay-safe.
*(The lint refuses `SystemTime::now`, `Instant::now`, and `tokio::time::Instant::now`
at compile time.)*

### G2: Unseeded RNG (`rand::thread_rng()`, `rand::random()`, `RandomState`)

**Footgun.** Drawing from a process-seeded RNG, `rand::thread_rng()`,
`rand::random()`, or hashing through a fresh `std::collections::hash_map::RandomState`
(seeded from the process RNG / ASLR), yields a per-process-random value.

**Why replay diverges.** The record process and the replay process seed
independently, so the "random" value differs run to run.

**Sanctioned alternative.** Seed a reproducible RNG from a value in the data
stream (a message field, a sequence number) or the framework clock, so the draw
is a pure function of recorded inputs. Cerulion does not provide a seeded RNG,
so construct one yourself from a fixed seed you own (for example
`rand::rngs::StdRng::seed_from_u64(seed)`), keep it on the node so the sequence
is a function of node state, and take the seed from configuration rather than
the environment. *(`rand::thread_rng` and `rand::random` are WARN-class rows in the lint table,
and the macro does not surface warn rows: an unseeded RNG in a tick compiles
clean with no diagnostic, so this one is on you. `RandomState`-based hashing is a
manual footgun too; hash with a fixed-seed hasher, e.g. a `BuildHasherDefault`,
instead.)*

### G3: `/dev/urandom` and other entropy sources

**Footgun.** Reading `/dev/urandom` (or `getrandom`, hardware RNG instructions)
directly bypasses even the RNG lint.

**Why replay diverges.** Fresh entropy per read, by definition.

**Sanctioned alternative.** Same as G2: derive any needed randomness from
recorded inputs or the deterministic clock. Entropy that MUST be captured
belongs in a recorded input frame (so replay re-injects the same bytes), never
minted inside a tick.

### G4: Thread spawning inside a tick (`std::thread::spawn`)

**Footgun.** Spawning a worker thread from a tick and folding its result back
into the output.

**Why replay diverges.** OS thread scheduling is non-deterministic: the thread's
interleaving, and thus what the tick observes when it collects the result,
varies run to run. (This also violates the single-node execution model.)

**Sanctioned alternative.** Keep tick bodies single-threaded and synchronous.
For genuine parallelism, split the work across graph **nodes** (the scheduler
parallelizes within a DAG level deterministically) rather than raw threads.
*(The lint refuses `std::thread::spawn` at compile time.)*

### G5: Shared atomics / statics for cross-node ordering

**Footgun.** Two nodes coordinating through a shared `static` atomic (a global
counter, a "who ran first" flag) instead of through the data plane.

**Why replay diverges.** The observed order of atomic operations depends on the
runtime's scheduling of the nodes, which, while deterministic under the
framework's DAG-level execution, is NOT part of the recorded data, so a
refactor that changes levelization silently changes what each node observes.
"Data is truth": meaning must live in messages, not in the timing
of a callback.

**Sanctioned alternative.** Communicate through **topics**. If node B needs to
know node A ran, A publishes a message B consumes; the ordering is then a graph
edge (recorded, replayable), not a race on a global.

### G6: `std::thread::sleep` / blocking waits in a tick

**Footgun.** Sleeping or blocking inside a tick to "pace" work or wait for
something.

**Why replay diverges.** A sleep couples the tick to wall time (which replay
does not reproduce) and, combined with any time-reading logic, leaks wall
jitter into the output. It also stalls the scheduler.

**Sanctioned alternative.** Use a `period_ms` trigger for rate control, or
`throttle_ms` for a producer rate cap; both are scheduling concerns the runtime
handles deterministically. A tick should compute and return, never wait.
*(`std::thread::sleep` is a WARN-class row and is not surfaced; a sleeping tick
compiles without complaint.)*

### G7: Allocator / `HashMap` iteration order

**Footgun.** Iterating a `std::collections::HashMap` (or `HashSet`) and writing
the results into an output in iteration order.

**Why replay diverges.** `HashMap` iteration order depends on the RandomState
seed (G2); it is deliberately randomized per process, so the SAME map yields a
DIFFERENT iteration order run to run, and thus different output bytes.

**Sanctioned alternative.** Use **`IndexMap`** (insertion-ordered, the crate the
graph layer already uses) when order matters, or **sort** the keys before
iterating. Never let hash iteration order reach the wire.

### G8: FMA / `-march` cross-host float skew

**Footgun.** Floating-point results that depend on fused-multiply-add
availability, `-march=native` autovectorization, or a different libm; the last
few ULPs of a transcendental or a fused expression differ across CPUs.

**Why replay diverges.** The bits differ even for the "same" computation on a
different microarchitecture; a byte-exact diff sees a mismatch.

**Sanctioned alternative.** Replay on the **same architecture** the bag was
recorded on (the common case). When you can't, a cross-host float difference can
explain the mismatch, so check the cause before you decide it is not a
regression; see G9. For hard cross-host determinism, avoid FMA-sensitive
expressions and pin the toolchain's float codegen; the `--tolerance`
mechanism can admit a bounded absolute or relative difference (`max_abs`,
`max_rel`) on selected fields so expected float skew does not fail the run.

### G9: The single-host assumption (cross-arch/os bags)

**Footgun.** Recording on one machine and replaying on another with a different
architecture or OS, then reading a byte mismatch as a code regression.

**Why replay diverges.** Beyond G8's float skew, differing pointer widths,
struct padding defaults, and endianness (the wire format is explicitly
little-endian, but a foreign writer may not be) can shift bytes.

**Behavior.** Every finalized bag carries a `__cerulion/recorder.json`
host-identity attachment (arch / os / version / recorded-at). `--verify`
**WARNS** on a cross-arch or cross-os bag ("float results may differ in last-bit
ULPs across arches — a byte-mismatch may be arch skew, not a regression") but
**never refuses**; replay is a tool, not a gatekeeper. An absent attachment is
silent back-compat; a malformed one warns. This is the sanctioned way to reason
about a cross-host mismatch: treat it as advisory, not authoritative.

## Not a determinism footgun: schema drift (exit 2)

Distinct from the above: if a node's **output schema layout changed** since the
bag was recorded (in the output type's own fields or any FIXED-resolved
nested schema), the recorded frames carry a different layout hash than the
current build produces. That is not node nondeterminism; it is a
workspace/recording mismatch, and re-execution refuses it up front with
**exit 2** (`SchemaDrift`), naming each drifted topic and both hashes, rather
than emitting an unexplained per-frame byte mismatch. (One blind spot: a layout
change confined to a VARIABLE-resolved nested schema contributes only its name
to the parent hash, so it evades the preflight and still surfaces as exit-1
byte mismatches.) Remediation: **re-record
the bag with the current workspace, or check out the recording-era schemas**
before replaying.

## Free-run recordings and per-rank re-execution

A multi-process recording carries the coordination contract it was taken under,
and the two contracts are re-executed differently. A **lockstep** bag (every bag
with no coordination stamp, every monolith bag, and every multi-process bag
recorded under the `CERULION_EXECUTION_MODE=lockstep` opt-out) is re-executed
on one authoritative clock: cross-rank boundary equality, one first
boundary to anchor a mid-run resume. A **free-run** bag is re-executed **per
rank**: one runtime per rank, one at a time, each driven to its OWN recorded
`STEP_BOUNDARY` targets, because a free-run run's ranks share only the GO epoch
and cross-rank equality does not hold by design.

The `--report` JSON's `coordination` block always says which contract the
replay applied (`{"mode": "free_run" | "lockstep", "inferred": true | false}`).
The terminal, on both surfaces (`--verify` and a bare `--resim`), prints a line
only when the contract was inferred, because the bag carries no stamp:

```text
coordination: lockstep (inferred: no coordination stamp)
```

**Availability.** The per-rank EXECUTOR described here is
available: a `coordination: free_run` bag handed to `cerulion bag play --resim` is
re-executed per rank rather than refused. The RECORDER is available too, and
free-run is the flagless default: a multi-process `graph run --record` produces
a `free_run` bag unless the run opted out with `CERULION_EXECUTION_MODE=lockstep`
(see [`docs/multi_process.md`](multi_process.md)), and a `lockstep` bag is
either that opt-out or a monolith recording (`--single-process --record` of a
graph that is one executor unit; one whose nodes split into several units
records a `free_run` bag, one rank per unit).

### Fires come from the trace; the verifier re-derives

Under per-rank re-execution the recorded trace DRIVES the fires; re-derivation
is the verifier's job, never the fire driver's. That is forced, not chosen: a
cross-process `block` producer's pre-fire gate would read cross-rank occupancy
at decision instants nothing records, and per-rank clocks are non-comparable
domains, so an independently re-derived defer/fire schedule can legally differ
from the recorded one under any legal interleave: spurious **fire-schedule
divergence** on exactly the block-paced graphs the feature exists for. In replay
the block gate is therefore BYPASSED as a fire decider and demoted to a verifier
ASSERT on the partial order the read log does record (credit conservation must
hold under SOME legal interleave). Loud on violation, never a gate.

**Scope.** A bag can carry a cross-process `block` edge: plan time accepts a
CREDITABLE split (exactly one in-graph producer, one word per split `block`
consumer) whose split `block` input sets no `block_timeout_ms`, the supervisor
mints the edge a cross-process credit word, and `graph run --record`
of that deployment writes one.

**Replay re-executes that bag.** The edge's producer and consumer run in
different passes, so the replay feeds the edge from the bag: the consumer's
pass builds its `block` input with no producer in its rank, no credit word and
no backpressure probe, and the producer's pass marks the producer as held by
the edge. For the split edge the verdict is exit 0, or 1 or 6 on a real
divergence, and the replay stands its producer down (as `block_gated`, or
`suppressible_fire` for a Sync producer), so that producer's own fire
schedule is not judged, even where a consumer in its own process also holds
it; every other hold in the bag gets the verdict a bag of the same graph in
one process gets, by the rules below.

- **The held producer's fire rules stand down.** The consumer's credit word
  in another process can defer it at any step, and the bag records neither
  the word nor its readings. Held Period, FIFO and `consume = latest`
  producers stand down as `block_gated`; a held Sync producer as
  `suppressible_fire`. A held Sync producer's owed fires go unjudged while
  the fires the bag records are still judged, and a held producer's
  `throttle_ms` is still judged. A held FIFO producer's trailing refills are
  still planned from its recorded pops and fires, so the replay serves the
  frame a hold froze.
- **Credit conservation** is judged on the consumer's recorded drains: the
  frames the bag records on the topic, less those the consumer drained, must
  stay within the input's depth, and a breach is a report-only
  `conservation` note. Where the bag does not bound that count, the edge
  stands down instead, with one `stand_down` note whose subject is the topic
  and whose code is the first cause that holds, in this order:
  `outside_writer_counted` (the graph lists the topic in
  `multi_publisher_topics`, so a writer outside the graph adds frames the
  credit word never counted), `frames_lost_at_record` (the record-health stamp
  does not show every frame of the topic recorded), `credit_unknown_at_anchor`
  (the producer's or the consumer's rank begins past step 0) and
  `consumer_stopped` (the consumer stopped with a stop that releases its slot).
  Where `--duration` stops the pass short, an edge judged otherwise stands
  down with a `stand_down` note that carries no code: the bound cuts the
  drains short and leaves the publish count whole.
- **The consumer's block events are not reproduced.** The bag does not record
  when the input's block events fired, so the replay delivers none. Each such
  input carries a `block_event_not_reproduced` note (its subject is the
  consumer) and the verdict prints one `NOTE:` line naming them: a node that
  reads `BackpressureEvent`s on that input can differ in its outputs or state
  for that reason alone.
- Where the bag's graph sets no `backpressure:` key on the input, `block`
  comes from the consumer's declaration in the build you replay, and the
  replay warns that the verdict matches the recorded run only if the recorded
  build declared it too.

What still surfaces structurally: rank-LOCAL decisions the verifier can
re-derive on its own (`Period` schedule and `throttle_ms` from the recorded
clock, `Sync` alignment, FIFO pop counts from the read log, and a
`consume = latest` node's one fire per step from the read log's drains) are
re-derived and positionally compared, so policy or declaration drift on those is still caught.
The producer of a split `block` edge is the exception: its Period, FIFO and
`consume = latest` rules and a Sync rule's owed fires stand down, as above,
and its `throttle_ms` is still judged.

**A node held by a `block` input in its own rank.** When a `block` input reads
a topic that a node in the same rank produces, that edge's pre-fire check can
hold the producer after its trigger drain popped a frame, and nothing records
the hold. The FIFO rule allows for it only at a step where the recording shows
the slot full: the frames the bag records on the topic, that step's included,
outnumber those the consumer's body reads drained before that step by at least
the input's depth.

- A frame whose stamp several steps share counts at the last of them, and a
  drain at an earlier one is owed against it there.
- A body read that finds the queue empty, or any body read of a latest-value
  input (`consume = latest`, or any input that is not a FIFO trigger), empties
  the slot; inside a shared stamp, such a read leaves none of that stamp's
  frames counted.
- A FIFO input drains only at its consumer's recorded reads.
- A fire the recording shows ended at an input the tick reads earlier drained
  nothing of the `block` input. A `#[cerulion_node]` tick reads its inputs in
  declaration order, `accumulate` ones aside, and ends at the first that serves
  nothing; an empty read of a context input that the tick reads frozen at the
  step boundary ends every fire of its step. The recording shows such an end
  only for a consumer whose metadata says it needs a publisher for every
  output, which that macro sets.

The slot is released, and holds nothing from then on:

- from the step after its consumer stops on a fatal error or a poisoned node
  entry;
- with no record, at a step where the recording shows the producer firing
  while the slot already counts its depth before that step's frames, since,
  while the slot holds, the check stops the producer at the depth before each
  fire;
- with no record, at a step where the slot counts past its depth, if the
  producer's metadata says it needs a publisher for every output.

The consecutive-panic breaker releases no slot. After a stop that releases
the slot, the two releases with no record do not apply. `#[cerulion_node]`
sets that metadata, a `--raw-ffi` node may set it by hand, and a Python node
cannot. Without it, a count past the depth is no release, since a `--raw-ffi`
node can send several frames in one fire. A step excused only by holds is a
divergence, and the finding says why, if every slot that excused it shows a
release with no record before its consumer next reads it.

The slot allows for no hold where:

- the bag records no frame of the topic, or a frame header it cannot parse;
- its record-health stamp is absent or malformed, has no entry for the topic,
  reports the topic lossy or written by several publishers (declared or
  observed), or reports any frame in the bag drained and never written. A
  Flashback capture carries no record-health stamp, so its slots allow for no
  hold;
- the bag has no `recorder.json`, or one stamped below `trace_format` 12;
- the producer's metadata says it needs a publisher for every output, and the
  recording shows it sending more frames at a stamp than it fired there. A
  worker can stop after its last recorded step published a frame and before
  that step's records reach the trace, so the frames of each worker process's
  last recorded step are compared only where the trace keeps a fire of that
  process at that step from a level after the producer's. Otherwise they are
  not compared, and neither are the frames of an earlier step that shares that
  step's clock. Frames stamped after a process's last recorded step are never
  compared. This holds for a recording of one worker process or several;
- the graph lists the topic in `multi_publisher_topics`;
- the recording does not pin the input's `block` policy and depth: the
  `node_inputs` entry of its `recorder.json` gives another policy or depth
  than the replayed build, or, on a bag recorded before `node_inputs`, its
  graph does not set both `backpressure: block` and `depth:` for the input;
- the input sets `block_timeout_ms`;
- the bag's fire records do not put the consumer on a later level than the
  producer (a node fired at more than one level, or never, orders no edge);
- the input is a Sync node's trigger input.

It also allows for no hold:

- from the step after a truncated read of the input, or a drain whose site
  the read log does not say;
- before any body read of the input is recorded, from the step after a
  consumer fire that the recording does not show ended before the input;
- on a resumed pass, until the consumer first reads its queue empty;
- on a latest-value input, from the step after a consumer fire that can hide
  a drain of it (no read of it recorded, and the recording does not show the
  fire ended before it), through the consumer's next read of it.

A step that neither the throttle nor the hold could defer owes every popped
frame a fire; the throttle's window comes from the recorded clock. Read through
one subscriber, a held node may leave one popped frame unfired at a step the
throttle or the hold could defer, or at a step that fired the scheduler's cap
of 64; the replay pops that frame after the step's last fire, as the live burst
did, so the next fire serves it. Read through two subscribers (a `block` or
`sample(N)` trigger input, or `CERULION_DRAIN_DISCIPLINE=separate`), it carries
every signal such a step left unfired, up to the same cap. On a bag that does
not record how a FIFO input was read, a `drop_oldest` trigger input is judged
without the hold, and a divergence on a short step there at which the bag
shows the slot full says so.

A `period_ms` node or a `Sync` node that such an input holds is judged on the
same count of the slot, and a step excused only by holds is a divergence on
the same terms. At a step where the recording shows the slot full, a
`period_ms` node may fire fewer of the fires its schedule owes there: none, or
the first of them at their instants. Its next deadline is then the first owed
instant it did not fire, a fire off that schedule is still a divergence, and
the node carries at most one note for it (a `hold_allowed_for` note of code
`period_block_held`). At such a step a `Sync` node may leave an aligned set
unfired. The node carries at most one note for it (a `hold_allowed_for` note
of code `sync_block_held`), and the node is still judged on every other step
and on the sets it fired. At any other step either node owes every fire. A
`hold_allowed_for` note is not a `stand_down`: the rule judged the node. A
neutral `--resim all` reaches the `Sync` rule for such a node. A `--verify`
replay of a free-run bag from trace format 8 refuses a per-set `Sync` node
before its first step (`read_identity_ungateable_stage`, exit 2), whichever
worker process its trigger topics come from, so it does not reach the rule
there. Where such an input holds a `Sync` node, the neutral `--resim all`
reports a read-log divergence at the node's first fire after the hold, and
exits 0.

A `consume = latest` input read through a second subscriber is also held to the
frames its trigger drain counted: a step that counted more frames since the
node's last fire than that fire's read took, plus two per writer still
delivering one, owed the node a fire, so a node that stops firing on frames its
queue still holds is a finding. The writers are the larger of the graph's
publisher cap and the writers the read log names on the input, because a topic
opened on a service that already existed with more publisher slots than the
graph asks for accepts publishers from outside the graph. Before the node's
first fire in the pass, one queue of the input more (its declared depth per
writer) is excused, and the `--report` JSON names it once as a judged note (a
`rederivation_notes` entry of kind `latest_before_first_fire`) rather than a
`stand_down`, because a node's init can read its inputs
before the recording starts its read log and a resumed pass starts after reads
its bag does not keep; a count past that queue is judged as one after a fire
is, on a resumed pass too. A resumed pass that holds no fire of the node and no
read its body made stands down by name once a count passes that queue (a
`stand_down` of code `latest_unfired_since_anchor`), because a node that
returned `NodeError::Fatal` before the pass's anchor records exactly
that, and so does a running node the scheduler never fired in the pass, and the
recording does not state which it was: the bag is below trace format 10 and the
node's checkpoint part carries no stopped flag, or the checkpoint holds no
complete part for the node (a node that saves no state, or a skipped or torn
part). From trace format 10 every complete checkpoint part of a stopped node
carries the stopped flag, and a resumed pass keeps such a node stopped and
judges no fire of it, and judges a node the checkpoint states as running in
full. A
from-start pass re-executes the stop, and a node that fires or reads in its
body anywhere in a resumed pass is still judged. A `--verify` replay of a
one-rank bag recorded mid-run below trace format 11 is refused before its first
step when a single-writer read in it continues a writer run that began before
the bag (`read_identity_absent`, exit 2), so on such a bag a neutral `--resim
all` is the replay that reaches this rule; from format 11 every read site names
its writer again after each checkpoint, so `--verify` feeds such a bag and
reaches this rule too. From `trace_format` 8 on a writer change on any edge stages a producer
annotation, so the counts a bag that begins mid-run records before the trigger
drain's first annotation are frames of the one writer whose run it began inside,
counted as one writer more. That rule stands down by name where a count needs an
excuse the bag cannot check: a topic no graph node produces leaves the frames in
delivery unbounded; a read log that does not name the writer of every frame
counted since the node's last fire cannot rule out a publisher beyond the cap,
until the node's next fire; and a node that produces a topic a `block` input in
its own rank reads can have a fire held by that edge's pre-fire check with no
record, which stands down the fire a pop owes on one subscriber too. A bag below
`trace_format` 8 names no writer on a topic with one graph producer, so there a
count past the cap stands down, from the start of a recording or resumed; a
count of more than one frame is named whole from `trace_format` 9, when its
batch frames fit the read stage. On a bag that does not record whether a
`drop_oldest` input was read through one subscriber or two, a step that popped
and fired nothing stands down rather than being judged as one.

For a node a resumed pass never runs and whose stop the recording does not
state, the other rules that owe a fire stand down the same way, at the first
step they would convict: the FIFO pop count (`fifo_unfired_since_anchor`), the
fire a pop owes a `consume = latest` input read through one subscriber
(`latest_unfired_since_anchor`), and the fire an aligned set owes a `Sync`
node (`sync_unfired_since_anchor`). Each of these stand-downs, the
two-subscriber one included, prints a warning on stderr on every run, naming
the node, the code, the bag's `trace_format` and the checkpoint step, and
changes no exit code; its `stand_down` note is in the `--report` JSON, which
only a `--verify` run writes. For a per-set `Sync` node a `--verify` replay
reaches this rule only on a bag below trace format 8: from format 8 a free-run
`--verify` is fed by read identity, which refuses a per-set `Sync` stage
before its first step (`read_identity_ungateable_stage`, exit 2), so on such
a bag a neutral `--resim all` is the replay that reaches it. A node that
exports no state shape, such as a Python node, has no checkpoint part on any
trace format, so the recording never states its stop: on a resumed pass where
it neither fires nor reads in its body, these rules stand it down by name from
trace format 10 too.

### Divergence LOCALIZES to the producing rank

Cross-rank edges are served from the **recorded frames**, injected through the
real accounting drain and steered by the consumer's own read log. So:

- **A cross-rank edge-read divergence is impossible by construction.** The input
  side of every cross-rank edge is the bag, not a co-replaying candidate.
- **A changed producer reads as "rank A produced different bytes"**, not as a
  cascade of consumer-side symptoms downstream of it. That is sharper blame, not
  weaker: the verdict names the rank that changed instead of everything that
  read from it.

**A named detection limit, and it is deliberate.** A candidate whose FIRE
behaviour differs only through a cross-rank interleave does not surface as a
fire-schedule divergence; it surfaces as a **frame-content divergence** or an
**edge-read divergence** instead. Divergence PROPAGATION (a changed producer's
different frames flowing downstream to its consumers, so their reads legitimately
differ) is not supported: cross-rank edges are always served from the bag.

### The ragged tail on a free-run bag

The ragged-shutdown-tail tolerance exists because the lockstep contract
drives every rank off ONE authoritative clock: a rank that shut down a few steps
early can be driven PAST its own recorded tail, re-fire there, and produce
"extra" output that is not a regression, so the comparator skips it and records
what it tolerated in `tolerated_ragged_tails`.

Per-rank re-execution does not create THAT situation. Each rank is driven to its
own recorded boundaries and stops, so no rank is over-driven because a PEER ran
longer, and the peer-tail half of the tolerance never engages. **This is by
design, not a coverage gap**: it compensates for an artefact of the single-clock
model, and the per-rank model does not produce the artefact.

**What can still open a window.** A rank whose recorded stream is COMPLETE (its
last recorded step banked fires) has an EMPTY window: the tolerated window's
upper bound is stated PER RANK (the last step that rank is actually driven to),
so `last_recorded == authoritative_last` and the window rule's "the rank's
recorded stream ended EARLY" clause is false at every step; its `tail_width` is
0, so an extra frame on that rank's topic is a VIOLATION, not a tolerated tail.
A rank whose LAST recorded step banked its boundary and NO fires is proven only
through the step before it, so that one final step IS inside the window, its
`tail_width` is 1, and the verdict reports it in `tolerated_ragged_tails` under
its own note wording. So on a free-run bag that list is empty when every rank
ended on a step that fired, and carries one note per rank that ended on a
fireless step. Under lockstep the bound is rank 0's last step broadcast
to every rank, one scalar for all of them, which is what lets the peer-tail half
engage there.

### G10: hand-crafting a bag: the wall epoch and `Period` phase

**Footgun.** Writing a free-run bag by hand (a crafted fixture, a trimmed
recording) with `STEP_BOUNDARY` targets that start at 0 or advance by a made-up
amount.

**Why replay diverges.** Under `--record`, every rank's controlled clock
INITIALIZES from `real_ns()` at live-loop entry (a machine-wide, boot-monotonic
value shared by every rank as the GO epoch) and then wall-follows. So a real
recording's first boundary target is a large NONZERO number, and the per-rank
re-advance accepts it. Two things follow for anyone writing one by hand:

- **A `Period` node's fire schedule is a function of that clock.** Boundary
  targets that do not advance by a plausible quantum produce a re-derived
  schedule that disagrees with the fires in your trace, and the verifier reports
  a fire-schedule divergence: a fixture bug rendered as a code verdict.
- **The epoch must be SHARED across ranks.** `sync_window_ms` is a spread over
  the trigger inputs' producer WIRE stamps, and `expect_within` is
  consumer-clock minus producer-stamp, so per-rank clocks each starting at 0 put
  those comparisons across unrelated clock domains: one skew direction saturates
  the watchdog inert, the other phantom-misses.

**Sanctioned alternative.** Craft from a REAL recording's boundary stream
(trim it, do not synthesise it), or derive targets from one epoch plus the
graph's own quantum for every rank. This is the one footgun in this catalog that
is about the BAG rather than about a node body.

## Rule of thumb

A tick body should be a **pure function of `(inputs, restored state, framework clock)`**. If it
reads anything else (the wall clock, entropy, the filesystem, the environment,
thread timing, hash order), replay cannot reproduce it. The lint is your
first line of defense; this catalog is the second. When you genuinely need
non-determinism (a driver/ingress node reading a live device), that node is an
`external` ingress whose data enters the graph as recorded input frames, so
replay re-injects the same bytes and stays byte-exact downstream.
