# Read-Log Forensics: which frame did that input serve, and what did it never see?

An **offline** reference for the per-edge read log. A bag recorded by
`cerulion graph run --record` carries, beside its frames and its fire trace, one
READ-OUTCOME record per input READ and, on the reads that take frames off the
queue, exactly one record per CONSUMED frame (trace-record kind 6,
`trace_format` 3 and later). From format 5 on, a bag carries a READ-SITE
ROLE on every record. It stamps **6**
exactly when its recorder had run-folding ENABLED, which is the default, so a
bag recorded with the defaults is a format-6 bag; `CERULION_READ_LOG_FOLD=off` turns
folding off and takes the stamp back to 5. A format-6 bag MAY fold a run of
identical reads into one counted record; see "The read-site role" and "Folded
runs" below.
This doc is how to read them **without running
re-execution**: given the bag and nothing else, you can answer

> which recorded frame did node `N`'s input `I` serve at step `S`, and what was
> lost on the way?

Why offline matters: `cerulion bag play <bag> --resim all --verify` answers a
DIFFERENT question ("does my current code still produce those bytes?") and needs
the workspace's cdylibs, a compatible arch, and a graph that will build. The read
log needs the bag and any MCAP reader. It is also the only surface that names an
**edge**: the fire trace names nodes, the frame stream names topics, and neither
can tell you which of two consumers of one topic missed a frame.

Related: [`docs/bag.md`](bag.md) (attachments, the `coordination` stamp and the
kind-6 annotation kinds), [`docs/multi_process.md`](multi_process.md) (per-rank
recordings), [`docs/replay_determinism_footguns.md`](replay_determinism_footguns.md)
(why a re-execution diverges).

## TL;DR: the three lemmas

| Question | Lemma | What it reads |
|---|---|---|
| Which step CONSUMED the frame popped at step `P`? | **a** fire in `[P, next_pop]`: one of them read this edge, and the ORDINAL join below says which | The same edge's next kind-6 record |
| …and for the LAST frame, which has no next pop? | `tail = the first FIRE of that consumer at a step ≥ P` | The consumer's own kind-1 fire records |
| What did this edge never see? | `loss = seq gap` between consecutive served ranges | `served_seq` + `popped` on consecutive records |

All three rest on ONE invariant, **one kind-6 record per CONSUMED frame**, and
that invariant rests on the boundary guard below. Read that section before
trusting an answer.

**On a `trace_format` 6 bag, expand folded runs FIRST.** A record is a
POSITION-count, not always one position: its aux word's high half says how many
consecutive byte-identical reads it stands for. Every lemma below is stated per
OCCURRENCE, so an unexpanded read of a folded bag under-counts every quantity
that counts records: the ordinal join, `Σ popped`, the fire-window walk. See
"Folded runs"; it is one line of arithmetic and it is not optional.

**Expand under a BOUND.** The count is a `u32` read off a 40-byte record, so a
corrupt or hand-edited one can state up to ~4.3e9 occurrences, and a reader that
materialises whatever it is told lets one record dictate its memory. A run
larger than the stage could ever have HELD is not a long run; it is a corrupt
record, and the correct reading is to refuse the edge rather than to expand it.
Two bounds are available offline: the stage's own rim, where the bag states one
(`read_log_capacities`), and failing that any reader-chosen ceiling. The replay
verifier uses its per-step budget and stands the affected edge down by
name; a hand reading should say "corrupt" at the same point rather than quietly
truncating, because a truncated expansion makes every count below too small
while still looking complete.

## The record

### Where it lives

| Artifact | Contents |
|---|---|
| `__cerulion/scheduler_trace` (an MCAP CHANNEL, schema `cerulion.SchedulerTrace`) | Every trace record, 40 bytes each, one per message |
| `__cerulion/trace_manifest_rank{N}.json` (attachments) | Per-rank `node_ids`, the per-node `inputs` table, the `publishers` table, the `read_log_capacities` staging-rim table, `read_log_capacity` |

A monolith recording has exactly one manifest, `rank0`. A multi-process
recording has one per worker rank plus the departure sentinel
`rank4294967295` (empty `node_ids`, no reads); see
[`docs/multi_process.md`](multi_process.md).

### The 40 bytes, as a kind-6 record reads them

The record layout is shared by every trace kind; kind 6 REINTERPRETS three of
the slots. All little-endian.

| Offset | Slot | Kind-6 meaning |
|---|---|---|
| 0..8 | `step` | The CONSUMER's logical step (unchanged meaning) |
| 8..16 | `fire_time_ns` | The **served wire sequence**, widened to `u64`. `u64::MAX` = no frame was served |
| 16..24 | `duration_ns` | Packed aux: low 32 bits = `popped`, high 32 bits = the FOLD RUN COUNT (`0` and `1` both mean one occurrence; see "Folded runs"). Through `trace_format` 5 the high half is structurally zero. **Exception:** on a `producer` annotation the whole 64 bits are the producer token, so it has no high half to read |
| 24..28 | `node_idx` | The CONSUMER node, indexing that rank's manifest `node_ids` |
| 28..32 | `global_level` | Packed `(input_idx:16 << 16) \| outcome_kind:16` |
| 32..36 | `record_type` | `6` for every record in this doc (`1` = fire, `2` = departure, `3` = step boundary) |
| 36..40 | `reserved` | The owning worker RANK, stamped by `bagd` at bag-write time |

### The outcome kinds

| Kind | Label | `served_seq` | `popped` (frames this drain took OFF THE QUEUE) |
|---|---|---|---|
| 1 | `served` | The frame served to the read | **1** under per-message FIFO; **N** under a drain-to-latest snapshot that popped N and served the NEWEST |
| 2 | `held` | The HELD frame's sequence, REPLAYED (no new arrival) | The drain's own count, 0 when nothing was taken |
| 3 | `none` | sentinel | The drain's own count, 0 when nothing was taken; nothing has EVER been delivered on this input |
| 4 | `drained_batch` | The NEWEST sequence in the batch | The batch size, counted as DELIVERED (a malformed frame is popped, skipped by the decoder, and NOT counted) |
| 5 | `decimated` | The last ACCEPTED sequence (or the sentinel) | > 0: popped and deliberately dropped by a `sample(N)` gate |
| 6 | `truncated` | sentinel | REINTERPRETED: records dropped IN THIS MERGE WINDOW. **A MARKED HOLE**, see below |
| 7 | `producer` | REPEATS the annotated read's sequence (a redundant join key) | unused (0); the payload is the 64-bit token in the aux slot |

**`popped` is a QUEUE count, not a served count**, and the difference is the
useful signal: a `served` record with `popped = 4` means the drain took four
frames off the queue and handed the tick the newest; the other three were
discarded by the read policy (`drop_oldest` drain-to-latest), visibly.

Kinds 6 and 7 are ANNOTATIONS, not reads. They ride inside record kind 6 rather
than taking new record kinds, which is what kept them from moving the format at
all; a reader that predates them decodes every field to the value the writer
meant and renders the kind as `unknown(6)` / `unknown(7)`. (They are defined
from format 3 on. A bag stamps 5 for the read-site role
and 6 for folded runs, each a WIRE change rather than a diagnostic, so
a bag recorded with the defaults carries these annotations under a 6, or under a 5 when it
was recorded with `CERULION_READ_LOG_FOLD=off`.)

### The read-site role (`trace_format` 5)

Every kind-6 record also carries the ROLE of the CALL SITE that performed the
read, in bits 14..16 of the same packed meta word the input index and outcome
kind ride in:

| wire | meaning |
|---|---|
| `0` | never written: every `trace_format` <= 4 bag. On a bag stamped 5 or later it means a record whose site nothing on the wire named (a hand-edited or foreign-written bag): it takes the pre-roles KIND arm, and its collisions report as `AmbiguousReadSite` |
| `1` | **drain**: a read that CONSUMES from the input's queue on the scheduler's behalf AND whose frame BECAME THE HEAD: the Separate/Sync trigger drain, the unified boundary drain, the Data burst refill, the per-set Sync matcher's `Advance`/`DiscardTie` refill, and its PROMOTION of an already-peeked frame. The enumeration is the definition: the step-boundary snapshot is scheduler-performed too and mints **body**, because it serves the body's latest-value read |
| `2` | **body**: NODE CODE read, inside its own tick |
| `3` | **peek**: a scheduler read that POPPED a frame to LOOK at its stamp. The frame is PARKED as the per-set Sync matcher's `next_head`; it is not the head, and the set the matcher fired may have been aligned on the frame ahead of it |

The role is the CALL SITE's, never the STAGE's: under the unified discipline the
boundary drain runs on the node's own body subscriber, so a drain-SITE record
legitimately lands in a body-role stage. The ONE exception is the overflow
MARKER (outcome kind 6), which is minted by the stage and therefore carries the
stage's role: diagnostic only, and no reader steers on it.

**The two-bit field is FULL.** Value `3` is `peek`, so `trace_format` 5
carries three roles. A `trace_format` <= 4 bag is unaffected; those bits were never
written there and every role on such a bag still reads unstamped. A FOURTH site
would need a wider subfield and a format bump.

**How a role RENDERS.** A believed role rides the kind as a parenthesised
suffix, in both places replay spells a read: the `--report` JSON's
`read_log_divergence.edges[].recorded` / `.replayed` `kind` field (the report
is an object whose per-edge records sit under `edges`), and the per-edge warn
on stderr:

```
drained_batch(drain)(seq 5, popped 1)
└───kind────┘└role ┘└─── details ───┘
```

Three segments, in that order: the kind label, the role suffix, the details.

The role goes on the LABEL, never inside the details, because the JSON has only
the `kind` field to carry it: a details-side role would exist on stderr and
nowhere else. A role of `0` renders NO suffix (`drained_batch(seq 5, popped 1)`),
so every `trace_format` <= 4 bag's report and warn lines are byte-identical to
what they were before roles existed.

**When replay COMPARES it.** The read-log verifier compares the role only when
BOTH of these hold, and reports nothing about it otherwise:

1. **the bag's `trace_format` is >= 5.** An archived bag's recorded role is `0`
   while the replay re-executes the real call sites and derives a real one, so
   an ungated compare would report a divergence on every read of every archived
   bag. Note the term is the recorder's `trace_format` stamp, NOT the kind-field
   width: they disagree on a bag with no `recorder.json` at all, where the bits
   still decode but the bag has claimed nothing.
2. **both sides NAME a site.** `0` is the ABSENCE of a claim, not a site, and it
   is a legal value on a bag stamped 5 or later, so `0` against `drain` reports
   nothing.

A role divergence therefore means what it says: both sides named a site and they
named DIFFERENT ones; the candidate read the frame somewhere the recording did
not. Like every read-log finding it is LOUD and REPORT-ONLY; the verdict and the
exit code are untouched.

**Reading it offline:** a role of `1` on a `Served`, `Held` or `NoFrame` record
is a `(kind, role)` pair no mint site can produce (those three are staged only
by the node body's own read paths), and replay reports it as a corrupt
recording. So is a `3` on anything but a `DrainedBatch` or its paired
`Producer` annotation; `peek` has exactly one mint. A run of `1`s and `3`s on
`DrainedBatch` records for ONE `(node, input)` in ONE step is NOT corruption;
it is one per-set Sync `align()`: a boundary drain, then the matcher's
stamp-peeks (`3`) and its refills/promotions (`1`). Within one `(node, input)`
the `1`s are in fire order, but the FIRE consumes the OLDEST head, and a step
can carry heads IN from the previous one (a frozen head plus a parked
`next_head`, per `verify_sync`'s own carry rule), so on a step with `c` carried
heads and `k` fires the fires consumed those `c` carried heads first and only
the FIRST `k - c` of THIS step's own head-naming records, not the first `k`.
The correct rule: the first `k` of (the heads carried in from the previous
step, THEN this step's own head-naming records, in that order) are the sets
fired; whatever remains is what the input carries forward. A hand reading must
ask whether SOME in-window tuple exists among that set (replay's arm B), not
whether the last record on this step alone reads as a fire. The `3`s are
frames the matcher looked at and parked.

### Folded runs (`trace_format` 6)

A `trace_format` 6 recorder collapses a **maximal run of CONSECUTIVE
BYTE-IDENTICAL** reads on one stage into ONE record carrying a COUNT, maximal
with one exception: the count SATURATES at `u32::MAX`, so an absurdly long run
continues into a second record rather than wrapping. Two adjacent records with
identical fields are therefore possible, and mean exactly what they say (the
occurrences sum). The count
rides the aux word's high 32 bits:

```
popped    = aux & 0xFFFF_FFFF
run_count = max(1, aux >> 32)      # 0 and 1 both mean ONE occurrence
```

`0 == 1` is what makes the section additive: every record written through
`trace_format` 5 has a structurally zero high half and decodes correctly with no
migration, and a `trace_format` 6 recorder writes the same zero for a read it
did NOT fold, so **a nonzero high half means a real fold**, and nothing else
does.

**What folds.** All of `(kind, served_seq, popped, token, role)` identical, on
the same stage, back to back.

**What BREAKS a run.** Each of these is a guarantee, not a caveat:

* any change in any of those fields (a `served_seq` that advances, a `popped`
  that differs, a different kind);
* a MERGE WINDOW boundary: a run never spans two steps, because the stage is
  drained at every level-end merge;
* a DROP at the stage rim. If the stream was `R, S, R` and `S` was refused at
  capacity, the second `R` does NOT fold into the first: a run count states
  ADJACENCY, and the hole is described by the window's `truncated` marker
  instead. **Run counts are adjacency-faithful.**
* a `producer` ANNOTATION is never a fold target in its own right, and a bare
  read never folds into an annotated pair's read half. A folded PAIR expands to
  `producer × 1` then `read × N`, and the count rides the READ half; the
  annotation's aux is the token and has no room for one.

**The arithmetic every lemma needs.** Sum `popped` and count positions over
OCCURRENCES, not records:

```
Σ popped over an edge = Σ_records (popped × run_count)
positions on an edge  = Σ_records run_count
```

**The `truncated` marker's arithmetic is unchanged, and that is the point.** The
marker reports RECORDS refused at the rim, and a fold consumes no slot, so a
run of 400 costs one record and refuses nothing extra. Read the marker as
"records the stage could not hold", never as "occurrences lost": those are
different quantities on a folded bag, and only the first is what the marker
counts.

Folding is a common-case saving, never a change to what a stage is SIZED for: a
live input's reads differ from one another, so it still costs one record per
read, and the stage's rim is derived for that worst case.

### Resolving the names

A record names nothing directly; both halves of the edge are indices into that
rank's manifest.

```jsonc
// __cerulion/trace_manifest_rank0.json
{
  "rank": 0,
  "node_ids": ["scanner", "fusion", "planner"],   // node_idx indexes THIS
  "inputs": {                                      // input_idx indexes the node's list
    "scanner": [],
    "fusion":  ["lidar", "camera"],
    "planner": ["fused"]
  },
  "publishers": { "0000…2f": ["scanner", "out"] }, // a kind-7 token resolves HERE
  // The per-input table is AUTHORITATIVE. Each row is
  // [input_idx, role, capacity], keyed on the (input_idx, role) PAIR; an input
  // under the Separate or legacy-Sync discipline carries TWO stages that share
  // an index, so the row list is not one-per-input and is not positional.
  // role 0 = body, 1 = drain.
  "read_log_capacities": { "fusion": [[0, 0, 22], [0, 1, 130], [1, 0, 4096]] },
  "read_log_capacity": 0 // a VERSION SENTINEL, not a capacity
}
```

So a record with `node_idx = 1`, `input_idx = 0`, `rank = 0` is the edge
`fusion.lidar`. **A record whose `input_idx` the manifest cannot resolve is not
guessable**; that is the same condition the replay-side verifier quarantines on,
and the correct offline answer is the same: report the edge as unresolvable and
stop, never fall back to a positional guess.

## The invariant the lemmas rest on: the boundary guard

> **The held head is never popped past unserved.** Under per-message FIFO
> (`EachFifo`) a boundary drain pops exactly ONE frame; if the consumer defers
> (a `block` gate, `throttle_ms`), that frame is HELD as the head and the next
> drain does **not** pop over it. **The re-offer records NOTHING**: one kind-6
> record per CONSUMED frame, stamped at the POP step.

Two consequences, and both are what make the join sound:

- **A pop is proof the previous head was served.** Nothing else in the bag says
  when a deferred frame was finally consumed (a defer emits no record at all),
  so the next pop is the only evidence, and it is conclusive only because the
  guard forbids popping over an unserved head. Lemma 1 IS that sentence,
  arithmetically.
- **`Σ popped` = frames taken off the queue**, over every popping record on the
  edge, weighting each record by its `run_count` on a `trace_format` 6 bag
  (see "Folded runs"; on formats <= 5 every weight is 1, so the identity reads
  the same). A per-re-offer record would break that identity (a re-offer pops
  nothing), which is one of the reasons re-offers record nothing.

**Where the invariant is enforced, and where it is pinned.** The guard lives in
the drain itself (`EachFifo` pops one frame and a deferred head is HELD, so the
next drain cannot pop over it), and the per-message FIFO tests exercise
that behaviour on the live path. The read log's own pin is
`crates/cerulion_core/tests/read_outcome_capture_iox2_test.rs`'s
`a_held_head_across_boundaries_records_one_row_per_consumed_frame`: a
`throttle_ms` consumer holds ONE FIFO head across several boundaries, and the
arm asserts the resulting kind-6 stream against a hand-built table of exactly the
shape above: one row per CONSUMED frame, `Σ popped` = consumptions plus the held
tail, pops and consumptions strictly ALTERNATING, and the hold genuinely spanning
two boundaries. Its control,
`without_a_throttle_no_head_is_held_and_pop_equals_consume`, runs the same
producer and window with the throttle removed, where every frame is popped and
consumed in its own step.

One place needs the interval form rather than the shorthand: on
the UNIFIED drain path the within-step refill pops the next frame in the
SAME step as the consumption, so `consumed_step = next_pop − 1` is true only
where the next pop was a BOUNDARY pop. The interval form below
(`consumed ∈ [pop, next_pop]`) is the one that holds on both, which is why it is
what this document states.

## Lemma 1: the frame popped at `P` is consumed by a fire in `[P, next_pop]`

Take one edge, `fusion.lidar`, and read its kind-6 records in step order. This is
a hand-built table of the shape a real one has:

| # | `step` | kind | `served_seq` | `popped` |
|---|---|---|---|---|
| a | 10 | `served` | 40 | 1 |
| b | 13 | `served` | 41 | 1 |
| c | 14 | `served` | 45 | 1 |
| d | 15 | `drained_batch` | 48 | 3 |
| e | 19 | `held` | 48 | 0 |

Record **a** pops sequence 40 at step 10. When was 40 actually consumed? The bag
does not say directly. But record **b** pops at step 13, and a pop cannot happen
over an unserved head, so 40 was consumed by SOME fire in the closed interval
`[10, 13]`:

```text
frame 40  popped step 10, next pop step 13  ⇒  consumed by a fire in [10, 13]
frame 41  popped step 13, next pop step 14  ⇒  consumed by a fire in [13, 14]
frame 45  popped step 14, next pop step 15  ⇒  consumed by a fire in [14, 15]
```

**"A fire", not "the unique fire".** What is
unique is the CONSUMPTION: exactly one fire in the interval read THIS EDGE. What
the bag records is a FIRE, and a fire record does not say what it read. Three
things put more than one fire in the closed window:

* the right endpoint is itself a pop, and on the Unified drain path that pop is
  routinely the same step as the NEXT frame's consuming fire, so the ordinary
  case already has TWO fires in `[P, next_pop]`, one per frame;
* a node with more than one trigger input fires for a reason that has nothing to
  do with this edge;
* a COLLAPSED read chain (the held-head warn, case 2) fires without
  reading the head it is holding.

The interval is still the lemma, because it is TRUE unconditionally within the
read-log contract and needs nothing but this edge's own records. Naming the
single fire needs the rule below.

### The ORDINAL join: the exact answer, and what it costs

Under per-message FIFO, pops and consumptions on ONE edge alternate strictly
1:1: a pop cannot happen over an unserved head, and a consumption cannot happen
without a served one. So the `i`-th pop recorded on an edge is consumed by that
consumer's `i`-th fire, and the answer is EXACT, no interval, no bound:

```text
fusion fires at steps: 13, 14, 15, 18, 22
pops on fusion.lidar:   10, 13, 14, 15
                        ⇒ pop #1 (step 10) consumed at fire #1 (step 13)
                        ⇒ pop #2 (step 13) consumed at fire #2 (step 14)
                        ⇒ pop #3 (step 14) consumed at fire #3 (step 15)
```

Its PRECONDITIONS are exactly the two things that break the alternation, and
both are readable from the bag:

1. **every fire of that consumer reads this edge.** A node with a second trigger,
   or one whose read chain collapsed, contributes a fire that consumed nothing
   here, and the shift is SILENT and PERMANENT: every later pop is joined to a
   fire one too late, with no marker anywhere saying so. Cross-check before
   trusting the join: `Σ popped` over the edge must equal the consumer's fire
   count plus the held tail. If it does not, the join is off by the difference
   and only the interval form is safe.
2. **no truncated marker intervenes.** A truncation is a hole in the pop
   sequence, so the ordinals on either side of it belong to different runs of the
   alternation. Restart the count at each truncated segment rather than joining
   across one.

**This is deliberately not fixed by recording a consumption time.** Re-offers
and serves record nothing, which is what makes `Σ
popped` mean "frames consumed" at all. A per-consumption record would change that
identity, and every lemma here reads it.

**Use the interval form.**
The closed form `consumed_step = next_pop − 1`, i.e. "the
last step before the next pop", is FALSE on the Unified drain
path. The within-step refill pops the NEXT frame in the SAME
step that consumed the previous one, so `next_pop` and the consuming step are
routinely equal and `next_pop − 1` names a step at which nothing was consumed.
Widening the answer to the interval `[P, next_pop]` is what makes it true on
both drain paths; narrowing it back to a single number needs the consumer's own
fire records, which is lemma 2.

**Read the interval's WIDTH as a DEFER.** `40` could not have been consumed
before step 10 and was consumed by step 13; `41` and `45` have width-1
intervals. An edge whose `next_pop − pop` is persistently > 1 is an edge whose
consumer is being gated, and the fire trace for `fusion` over steps 10..13 tells
you which step inside the interval actually took it.

**Scope.** Without backlog the interval's upper end is a weak
BOUND rather than a near-date: an empty queue means the next pop waits for the
next ARRIVAL, so `next_pop` may be far later than the actual consumption. The
bag cannot distinguish "deferred for three steps" from "consumed immediately,
then nothing arrived for three steps" from the read log alone; use lemma 2 (the
consumer's own fire records) whenever the edge is not visibly backlogged. A
`held` record (**e**) is the cheap tell: a `held` at step 19 says the input had
no new arrival there, i.e. the queue was empty, i.e. lemma 1 says almost nothing
about record **d**.

## Lemma 2: `tail = the first fire at a step ≥ pop`

Record **d** is the last pop on the edge. It has no next pop, so lemma 1 has
nothing to read. Date it against the CONSUMER'S OWN fire records instead
(trace-record kind 1, `node_idx` = `fusion`, in step order):

```text
fusion fires at steps: … 13, 14, 15, 18, 22
frame batch 46..48  popped step 15  ⇒  first fire at a step ≥ 15 is step 15
                                    ⇒  consumed at step 15
```

This is the general rule and lemma 1 is its cheap BOUND, so use it wherever
lemma 1's interval is too wide to act on: at the run's tail, on a quiet edge, or on any edge
whose `held` records show an empty queue. Two cautions:

- **A fire is not proof the input was read.** A node fires on ITS trigger; a
  plain non-trigger `#[input]` may replay a held value and produce no new record
  at all. The rule dates the pop against the node's next opportunity to consume
  it; that opportunity is what the fire record proves.
- **A discard-marked fire (`reserved` bit 31) still counts.** It fired
  and it read; it just committed none of its outputs.

## Lemma 3: `loss = seq gap`

A publisher's wire `sequence` is gap-free at commit, so any gap between
what one edge POPPED is exactly the frames that never reached it, evicted at its
queue before the drain got there.

Every popping record covers a CONTIGUOUS range `[served_seq − popped + 1,
served_seq]` (the queue is FIFO and commit sequences are gap-free), whether it
served all of them (`drained_batch`) or served the newest and discarded the rest
(a drain-to-latest `served`). So between consecutive records:

```text
loss = served_seq(next) − popped(next) − served_seq(prev)
```

Against the table above:

| Between | Arithmetic | Verdict |
|---|---|---|
| a → b | `41 − 1 − 40` | **0**: contiguous |
| b → c | `45 − 1 − 41` | **3**: sequences 42, 43, 44 never reached this input |
| c → d | `48 − 3 − 45` | **0**: the batch covers 46, 47, 48 contiguously |

**The gap and the discard are DIFFERENT losses, and this is how you separate
them.** A nonzero gap = frames evicted at the queue before this edge saw them
(the consumer could not keep up, or the depth is too shallow). A `popped > 1` on
a `served` record = frames the edge DID take and its own `drop_oldest` read
policy discarded on purpose. One is a capacity problem; the other is the policy
working as declared.

Five rules for applying it:

1. **Skip `held` and `none` records.** A `held` REPEATS a sequence already
   counted; folding it in reports a phantom negative gap.
2. **A `decimated` record is not loss.** Its `popped` frames really were taken
   off the queue and deliberately dropped by a `sample(N)` gate; `served_seq` is
   the last ACCEPTED sequence, so the arithmetic above already accounts for them.
   Loss and decimation are different verdicts and this is how you tell them apart.
3. **`popped` on a batch is the DELIVERED count.** A malformed frame is popped
   off the queue and skipped by the decoder, so it is not counted, which means
   the formula folds undecodable frames in with lost ones. If the number matters,
   cross-check `__cerulion/record_health.json`.
4. **A gap on a `multi_publisher_topics` edge means nothing on its own.**
   `sequence` is PER-PUBLISHER, so two writers interleave two unrelated counters.
   Attribute each read to its publisher through the kind-7 tokens first (below),
   then apply the arithmetic per publisher.
5. **Never span a `truncated` marker.** The records the marker stands for are
   gone, so the two records on either side of it are not consecutive and the
   subtraction reports a gap that was really a hole in the DIAGNOSTICS. Compute
   per segment, between markers.

Cross-check the verdict against the topic's own accounting: this is a
CONSUMER-side gap, so a real eviction should also show as `frames_lost` on the
recorder's side (`record_health.json`) if the recorder's own tap missed it, and
should NOT if only this consumer's queue overflowed. The two disagreeing is the
useful case: it localises the loss to this edge rather than to the wire.

## What the join CANNOT answer, and must say so

A forensic answer that guesses is worse than no answer. Each of these must be
reported as "no answer here", never inferred:

| Condition | What you see | What it means | What is true |
|---|---|---|---|
| **Unresolvable edge** | A `node_idx`/`input_idx` the manifest cannot resolve to a name | Same condition the replay verifier QUARANTINES on (a per-node quarantine when the input table cannot be resolved at all, a per-edge one for a single input) | The edge is unnameable. Report the raw indices; do not positionally guess |
| **No records at all for a node** | A node in `node_ids` with zero kind-6 records | The join answers NOTHING for it: the reads were never staged, or every one of its records was excluded | Say "no coverage"; absence of records is not absence of reads |
| **`truncated` marker** | Kind 6, `popped` = k | k records that WOULD have sat at this position were dropped at the staging rim in this merge window | A MARKED HOLE. The frames were consumed; only the diagnostics were lost. Lemma 1 must not span the marker, and lemma 3's arithmetic is invalid across it |
| **the staging rims a bag declares** | You are comparing two bags, or wondering which rim a replay armed | The AUTHORITATIVE key is `read_log_capacities`: per node, one `[input_idx, role, capacity]` row per STAGE, keyed on the `(input_idx, role)` pair. The scalar `read_log_capacity` is a VERSION SENTINEL, not a capacity. Three regimes, and what a replay does with each: **`read_log_capacities` present** ⇒ the replay ADOPTS each stated rim, so both sides truncate identically; **absent with `read_log_capacity` 64/160/320** ⇒ a bag from before the per-stage table, whose ONE global rim is adopted for every stage (such a bag is fully verifiable); **absent with `read_log_capacity: 18446744073709551615`** ⇒ a development build from before the table, which derived a rim per stage but had no table to write it in, so the rims cannot be recovered and the read log stands down. `0` is the table-era sentinel and means "the per-input keys are authoritative"; a binary that predates the table reads it, compares against its own linked `320`, and stands down loudly rather than claiming a match it cannot back. **Reading `18446744073709551615` by eye:** it is `u64::MAX`; it round-trips exactly through `serde_json`, but any reader that parses JSON numbers as doubles (`jq` ≤ 1.6, browsers, an MCAP viewer) displays `18446744073709552000`, a rendering artifact of the reader, not a different stamp | A bag whose table is present-but-empty, malformed, carries duplicate `(input_idx, role)` keys, or names only SOME of the run's stages is UNREADABLE: the verifier stands down loudly rather than fall back to a rim the bag never stated. An offline join over ONE bag is unaffected |
| **Unresolvable producer token** | A kind-7 token no `publishers` table maps | The ids are RUN-RANDOM, so a token only means something inside its own run | Report the token; never map it against another run's table |
| **A refused read log on a rank whose read-log-steered injection FELL BACK** | The read log stands down, and the report DECLINES the frame comparison on the topics that rank PRODUCES rather than failing them | Its cross-rank injection was timed from the read log; with it refused the injection falls back to the recorded-clock window, which can hand a frame to a fire early or late, so the rank's producers emit different bytes. That difference is the fallback, not the candidate | Those topics are not verifiable on this bag. The replay records each in `declined_frame_comparisons` (topic, rank(s), cause code, the outcome, and the withheld violations verbatim), unconditionally, a topic that matched included, takes its verdict from the topics no refused read log steered, and never reports a frame-content violation for this cause. The granularity is per TOPIC selected by RANK: every topic an affected rank produces, including one whose node consumed nothing injected. A refused read log on a rank with NOTHING injected declines nothing: there is no fallback to withhold, so no row is minted on its account. RESIDUAL, stated in full here: that is not the same as "every such topic is credited"; a `multi_publisher_topics` topic co-produced with a DECLINING rank is still declined, because the owner set is a SET and one declining owner is enough. For the whole-rank census decline that is the cause's own granularity; for a per-topic read-log-derived refusal it is a deliberate conservative superset. A produced topic whose owner rank is unknown is NOT declined. Re-record the bag to make them verifiable again |
| **A deferred frame at the very end** | A pop with no next pop and no later fire | The run ended before the frame was consumed | Consumed: unknown. It may never have been |

**Quarantine, precisely.** A quarantine is not a divergence and not a corruption;
it is the verifier declining to compare reads it cannot pair reliably, recorded
in the `--report` JSON with the node, the optional input, the step at which it
stopped comparing, and the reason. Steps BEFORE that step were compared normally.
The offline join inherits the same rule: it can answer for the prefix and must
decline for the rest.

**A refused read log never blames the candidate.** The read log is REPORT-ONLY:
no read-log condition produces a data violation. That holds even where the read
log STEERS something else, the cross-rank injection window, because the
fallback can misplace a frame by a fire, and comparing that against the
candidate's own output would invent a divergence the candidate did not cause.
So when the read log is refused for **a rank whose read-log-steered injection fell
back**, the frame comparison on the topics that rank PRODUCES is DECLINED: named,
with the cause and the divergences it withheld, in the report's
`declined_frame_comparisons` (NOT in `injection_stand_downs`, which carries the
separate fact that a topic's injection fell back), and the verdict comes from the
topics no refused read log steered.

The fallback is the whole premise, so it bounds the rule: a refused read log on
a rank with
**nothing injected declines nothing: there is no fallback to withhold**, and no
row is minted on its account. The engine says the same thing per rank, in the
whole-rank warn's computed `declines` field.

Two cause families reach this: the WHOLE-RANK census decline
(`verifier_declined_*`), and a PER-TOPIC read-log-derived injection refusal
(`truncated_read_log`, `unresolved_producer`, the shape a producer-token
COLLISION also arrives as). Both fall back to the same recorded-clock window,
so both decline what the affected rank produces, subject to the same bound
as every other statement of this rule: a rank with NOTHING injected has no
fallback, so neither family declines anything there. `no_read_log_coverage` is
deliberately NOT among them: it says the recording carries no read records for the
edge at all, so nothing was refused and nothing was lost; declining on it would
stop crediting every bag recorded before read-log annotation existed.

A DECLINED topic counts in `topics_checked` but not `topics_passed`, so it is
neither failed nor credited. That holds whether or not the comparison found a difference:
every topic an affected rank produces is listed, carrying `matched_under_fallback`
when it matched and the withheld divergences when it did not, because a match under
a fallback input stream is evidence the fallback reproduced the frames rather than
evidence the candidate is correct. The cost is deliberate and stated: a real regression on
exactly such a topic of exactly such a bag is reported as a decline rather than a
failure.

**Quarantine never touches the RECORD stream.** It is a REPLAY-side verdict:
the verifier declines to compare an edge it cannot pair reliably, and every
record it declined is still in the bag, byte for byte, for an offline join to
read. So a node with "no coverage" in a `--report` quarantine list is not a node
whose records are missing; check the trace before concluding anything about
what the recorder wrote. The row above is about the OTHER condition: a node with
genuinely zero kind-6 records in the bag.

## Where the mechanism is documented

| Question | Source of truth |
|---|---|
| Staging, capacity, the overflow marker | `crates/cerulion_core/src/read_outcome.rs` (module docs) |
| The 40-byte record + the kind-6 reinterpretations | `crates/cerulion_core/src/trace_ring.rs` (`RECORD_TYPE_READ_OUTCOME`) |
| The manifest's `inputs` / `publishers` / `read_log_capacities` / `read_log_capacity` | `cerulion_bagd` (the manifest attachment writer) |
| Verifier statuses, quarantines, divergence classes | `crates/cerulion_cli_engine/src/replay_engine.rs` + [`docs/bag.md`](bag.md) |
| Why re-offers record nothing | "The invariant the lemmas rest on" above, pinned by `crates/cerulion_core/tests/read_outcome_capture_iox2_test.rs` |
| The fold rule, the count word, and what breaks a run | `crates/cerulion_core/src/read_outcome.rs` (`records_fold`, `FoldAnchor`) + `crates/cerulion_core/src/trace_ring.rs` (`pack_read_outcome_aux_run`) |
