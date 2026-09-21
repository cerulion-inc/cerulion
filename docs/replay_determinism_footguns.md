# Replay Determinism Footguns: the G-Category Catalog

A "why does my replay diverge" reference for re-execution (spelled
`cerulion bag play <bag> --resim all --verify`) and
the compile-time determinism lint. Cerulion's replay is **byte-exact**:
a recorded bag is the golden, your current workspace build is the candidate, and
every produced frame is diffed byte-for-byte against the recording
(Principle #7: Replay = Live). That guarantee holds **only if every node body
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
"Data is truth" (Principle #2): meaning must live in messages, not in the timing
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
and the two contracts are re-executed differently. A **lockstep** bag (the default,
including every bag with no coordination stamp and every monolith bag) is re-executed
on one authoritative clock: cross-rank boundary equality, one first
boundary to anchor a mid-run resume. A **free-run** bag is re-executed **per
rank**: one runtime per rank, one at a time, each driven to its OWN recorded
`STEP_BOUNDARY` targets, because a free-run run's ranks share only the GO epoch
and cross-rank equality does not hold by design.

The verdict says which contract it applied, on both surfaces, always:

```text
coordination: free_run (per-rank boundary streams + read-log agreement; cross-rank lockstep equality not applicable)
coordination: lockstep
coordination: lockstep (inferred: no coordination stamp)
```

**Availability.** The per-rank EXECUTOR described here is
available: a `coordination: free_run` bag handed to `cerulion bag play --resim` is
re-executed per rank rather than refused. The RECORDER is available too, as an
opt-in: a multi-process `graph run --record` under
`CERULION_EXECUTION_MODE=free_run` writes a free-run bag (see
[`docs/multi_process.md`](multi_process.md)). Without that variable every bag a
robot produces is `lockstep`, which is the default.

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
CREDITABLE split (one in-graph producer, no non-`block` consumers), the
supervisor mints the edge a cross-process credit word, and `graph run --record`
of that deployment writes one.

**Replay REFUSES that bag, at exit 5.** `replay_engine` builds each rank's
runtime through constructors that pass no credited-edge set, so the consumer's
rank sees a `Block` consumer on a topic with no in-graph producer and
`GraphTopology::validate` refuses. The verdict is `ReplayError::Internal`
("failed to build the replay graph runtime for rank N"), NOT the
`not-replay-grade` exit-2 class, which is about a corrupt or non-replay-grade
BAG; this bag is fine and it is the replay build that cannot express the edge. The replay build does not thread the recorded credit edges (from the per-rank
plan/manifest), so do not
`--record` a deployment whose `process_groups:` splits a `block` edge if you
intend to replay it; co-locate that flow, or run `--single-process`, for any
run you plan to re-execute.

The demotion described above is unaffected and remains correct: any such bag
that does load meets a verifier rather than a spurious divergence, which is
what the demotion exists for.

What still surfaces structurally: rank-LOCAL decisions the verifier can
re-derive on its own (`Period` schedule and `throttle_ms` from the recorded
clock, `Sync` alignment, FIFO pop counts from the read log) are re-derived and
positionally compared, so policy or declaration drift on those is still caught.

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
