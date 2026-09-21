# Multi-publisher topics: delivery, ordering, and arbitration

A design rationale reference. Multi-publisher topics
(a topic with ≥2 publishers) are everywhere in robotics, but "multiple
publishers" is not one thing: the *consumer* semantics fan out into
four distinct categories, each with different delivery and safety needs.
This doc enumerates them, states what Cerulion needs per category, and
records what is supported, what is partial and what is not built.

Related: `docs/user-api.md`, "Backpressure" (the single-writer default, the
`multi_publisher_topics` opt-in, and the per-input backpressure policies).

## TL;DR

- Cerulion enforces **single-writer by default**. A 2nd publisher of a
  non-listed topic is **rejected at build**. Single-writer enforcement
  rejects uncoordinated publishers before the run starts: two uncoordinated
  writers racing for "latest" make the robot's behaviour nondeterministic.
- Multi-publisher is an **explicit opt-in** via `multi_publisher_topics`
  in the graph YAML. Opting in disables the single-writer guard for the
  listed topic and provisions the loose shared port cap.
- But opting in does *not* tell Cerulion *which* of the four
  multi-publisher patterns the user intends. Merge and append consumers need
  every retained message. Latest-value inputs and actuator commands need
  explicit ownership or arbitration. The delivery model and the safety
  stance differ per category, which is what this taxonomy sets out.

## The four categories (by CONSUMER semantics)

The dividing question is **what does the consumer do with N publishers'
messages?**, not how many publishers there are.

### A: Merge / dictionary (latest-per-KEY)

Each publisher broadcasts under a **different key**; the consumer
accumulates all of them and keeps the latest value *per key*. The topic
is a distributed dictionary; no message supersedes another unless it
carries the same key.

| Aspect | Detail |
|---|---|
| Examples | `/tf` (key = `(parent, child)` frame pair), `/diagnostics` (key = status name; the `diagnostic_aggregator` collects `DiagnosticArray` from every node), multi-controller `/joint_states` (key = joint name) |
| Consumer model | Merge by key into a keyed store; keep latest-per-key; usually time-indexed |
| Multi-pub is | **Intended** |
| Cerulion needs | ACCUMULATE-ALL delivery (consumer must see *every* message, not just the latest) + per-key merge in the consumer + a time-indexed buffer + query-by-time (e.g. tf `lookupTransform(target, source, T)`) |

The defining requirement is **accumulate-all**: if the consumer only
sees the latest message on the topic, it loses every key it didn't just
receive. A drain-to-latest read is *wrong* for category A.

### B: Append-only stream (full history)

Every message is a distinct event; nothing is superseded; order matters.
The consumer appends to an ordered log.

| Aspect | Detail |
|---|---|
| Examples | `/rosout` (logs from every node); Cerulion-specific observability streams: crash/panic events, network/liveliness events (publisher connect/disconnect), discovery events, audit/bag streams |
| Consumer model | Append in arrival order; optionally bound the retained history |
| Multi-pub is | **Intended** |
| Cerulion needs | ACCUMULATE-ALL delivery + ordered append in the consumer + a bounded-history option |

Like A, B is an accumulate-all category, but the consumer *appends*
rather than *merges by key*. There is no supersession at all.

### C: Latest-value inputs with multiple writers

A single "current value" topic with more than one *uncoordinated*
writer. The consumer wants "the latest value", but there is no coherent
"latest" when N writers are uncoordinated.

| Aspect | Detail |
|---|---|
| Examples | Any single-value topic accidentally driven by 2+ nodes: a setpoint, a mode flag, a status field |
| Consumer model | drain-to-latest, but "latest" is whichever writer's message happened to arrive last |
| Multi-pub is | **Footgun** |
| Cerulion needs | Single-writer-by-default puts this out of reach without an explicit opt-in. Listing the topic in `multi_publisher_topics` and then reading it drain-to-latest is accepted today with no build warning; that warning is not built (see the capability table below) |

The core problem: drain-to-latest over N uncoordinated writers yields a
**nondeterministic last-writer**. Two identical runs can disagree on
which message "wins". This is exactly the class single-writer-by-default
is designed to prevent, so the only way to reach category C is to
explicitly opt out of the guard, and that opt-out deserves a loud
warning.

### D: Arbitration / command (intended-but-must-coordinate)

Multiple sources command one actuator; exactly **one** should win, by
priority. Unlike C, multi-publisher is genuinely intended here, but the
"latest wins" semantics of C are wrong; "highest-priority active source
wins" is right.

| Aspect | Detail |
|---|---|
| Example | `/cmd_vel`: teleop + nav + safety all publish velocity commands to one robot base |
| Consumer model | Deterministic priority arbitration: highest-priority *active* publisher wins; lower-priority sources are ignored while a higher one is live |
| Multi-pub is | **Intended (with coordination)** |
| Cerulion needs | A first-class **deterministic priority-arbitrated input**: highest-priority active publisher wins, reproducible, replayable |

With N publishers on `/cmd_vel` and no arbitration, the base executes
whichever message arrives last: nondeterministic, and a safety hazard
(a stale teleop command can override a nav stop). The usual ROS 2 answer
is `twist_mux`, a *separate node* that subscribes to all sources and
republishes the winner by priority + timeout. Cerulion provides no
priority-arbitrated input policy today: use a priority-mux node of your
own for actuator commands, the same pattern.

### Summary table

| Cat | Examples | Consumer model | Intended / footgun | What Cerulion needs |
|---|---|---|---|---|
| **A** Merge / dictionary | `/tf`, `/diagnostics`, multi-controller `/joint_states` | latest-per-KEY (merge) | Intended | accumulate-all delivery + keyed merge + time-indexed buffer + query-by-time |
| **B** Append-only stream | `/rosout`, crash/liveliness/discovery/audit/bag streams | ordered append (full history) | Intended | accumulate-all delivery + ordered append + bounded-history option |
| **C** Latest-value multi-writer | accidental 2-writer setpoint / mode / status | drain-to-latest (nondeterministic winner) | **Footgun** | single-writer-by-default; **warn loudly** if opted in + read drain-to-latest |
| **D** Arbitration / command | `/cmd_vel` (teleop + nav + safety) | priority arbitration (highest active wins) | Intended (coordinated) | first-class deterministic priority-arbitrated input |

## Delivery and replay semantics

### What ROS2 `/tf` actually does

tf2 accumulates every transform into a time-indexed buffer (the
`tf2::BufferCore`). The default `/tf` QoS is **KEEP_LAST depth 100,
VOLATILE, RELIABLE**; `/tf_static` is **TRANSIENT_LOCAL** (latched for
late joiners). lookups (`lookupTransform`) interpolate against the
time-indexed buffer. This is a real, working category-A design. The rows
below name where its guarantees stop.

### Where the ROS2 defaults stop

| # | Failure | Consequence |
|---|---|---|
| 1 | **Nondeterministic delivery across publishers** | DDS arrival order across N publishers is not pinned, so two identical runs see transforms in different orders → the buffer state differs → `lookupTransform` can return different results run-to-run |
| 2 | **Silent queue overflow** | depth-100 KEEP_LAST: a burst of >100 transforms between consumer wakeups silently drops the oldest. Later a `lookupTransform` for a dropped time throws `ExtrapolationException`, with no record of *why* the data is missing |
| 3 | **Replay does not reproduce the buffer** | bag replay re-publishes into live DDS, which re-races arrival order and re-drops on overflow → the tf buffer state does not reproduce, so a replay-based CI check over tf-dependent behaviour is not reliable |
| 4 | **A second writer is accepted without a guard** | nothing rejects an additional publisher → an unarbitrated `/cmd_vel` (category D) and accidental category-C topics ship undetected |

### Cerulion's edge

**For A/B (accumulate-all categories):**

| ROS2 weakness | Cerulion answer |
|---|---|
| Nondeterministic cross-publisher order | **Within one process the scheduler preserves producer execution order**, so a single-process graph's accumulate-all sequence is reproducible. Writers split across processes have no cross-writer total order; a recording retains each frame's writer provenance, so replay reproduces what was recorded |
| Silent queue overflow | **No silent drops**, Principle #6 (no data loss): an eviction is counted per publisher stream and surfaced through `backpressure_drop_oldest_count` and the `#[on_event]` handler, so the consumer is not left guessing why a key vanished. Every edge of that accounting errs toward under-reporting rather than fabricating a loss |
| No replay fidelity | **Replay = Live** (Principle #7): a recorded `/diagnostics` or crash-event sequence re-executes from the recording instead of being re-published into a live transport, so the consumer sees the recorded order |

Replay is the practical consequence: the ordered sequence of
`/diagnostics`, `/rosout` or crash and liveliness events that preceded a
failure re-executes from the recording, which is what makes it usable as a
CI gate.

**For C/D (latest-value / command categories):**

| ROS2 weakness | Cerulion answer |
|---|---|
| Multi-writer footguns silently allowed | **Single-writer by default**: the accidental category-C topic is rejected at build, not discovered in the field |
| `/cmd_vel` raced nondeterministically, arbitrated by a separate `twist_mux` node | No priority-arbitrated input policy exists in Cerulion today; arbitrate in a node of your own (see the capability table below) |

### Ordering scope

Within one process the scheduler preserves producer execution order, so a
single-process graph's accumulate-all sequence is reproducible. Writers split
across processes have no cross-writer total order, and neither do two
separately run graphs publishing to the same topic (two graphs both writing
`/tf`): the multi-process lockstep contract
([`docs/multi_process.md`](multi_process.md)) hands one clock to the process
groups of ONE graph, and does not join two graphs. Cross-graph order is
correct by count but not reproducible frame for frame.

## What is available today

| Capability | Availability | How to use it, or what is missing |
|---|---|---|
| Single-writer enforcement | Available | A non-listed topic rejects a 2nd publisher at build (`graph/topology.rs::build`, `runtime.rs` active-publisher pre-check). The `multi_publisher_topics` opt-in (`graph/config.rs`) disables the guard for the listed topic and provisions the loose shared port cap |
| Latest-value reads (drain-to-latest) | Available | A plain `#[input]` (a non-trigger context input) reads the single newest sample, frozen at the step boundary. An `#[input(trigger)]` input's read is per-message FIFO (one fire per queued frame, in order); drain-to-latest is the latest-value-context semantic only |
| Backpressure policies | Available | `drop_oldest` / `sample(N)` / `block`. Zero-copy scheduling: no Cerulion-side data buffer |
| Accumulate-all delivery for categories A and B | Partly available | An `#[input(trigger)]` input observes every retained frame, one per fire, in order, and the node merges or appends across those fires. There is no surface that hands ONE tick the whole queued batch: `#[input(accumulate)]`, `#[input(history = N)]` and a buffered-subscriber API are not built. A plain `#[input]` is drain-to-latest, the wrong choice for A and B. Delivery across two graphs is covered by `multi_publisher_iox2_test::cross_graph_listed_topic_both_publish_and_flow`, which tallies both producers' markers rather than the latest |
| Build warning for a latest-value read of a listed topic | Not available | Listing a topic in `multi_publisher_topics` and reading it drain-to-latest is accepted with no warning. Check the read kind yourself when you opt in |
| Deterministic priority arbitration (category D) | Not available | No priority-arbitrated input exists. Arbitrate in a node of your own (the `twist_mux` pattern) |
| `block` backpressure on a multi-publisher topic | Not defined | The shared `outstanding` mirror is gated at the **declared depth**, while iceoryx2 queues are per-(publisher, subscriber) connection. The decide-phase over-publish behaviour and the meaning of `depth` for an N-publisher `block` topic are both unresolved: do not rely on it |
| Reproducible ordering across writers in different processes | Not available | Within one process the scheduler preserves producer execution order. Writers in different processes, and separately run graphs sharing a topic, have no cross-writer total order (see "Ordering scope" above) |

### Accumulate-all: what the two input kinds do

The two input kinds differ exactly at the merge/append boundary:

- A plain `#[input]` (a **latest-value context input**) is **drain-to-latest**.
  The tick reads the single newest sample, and everything queued behind
  it is discarded. Correct for a latest-value sensor; **wrong** for `/tf` (silently
  drops every frame except the last, so keys are lost).
- An `#[input(trigger)]` input is per-message FIFO (one fire per queued frame, each
  serving that frame in order). The read path itself discards nothing, so
  every frame the input's declared backpressure policy retains is observed
  (a `drop_oldest` eviction, a `sample(N)` decimation, and a corrupt-frame
  skip still apply, each counted), one at a time across fires rather than
  as one iterable batch. The node does its own merge/append across those fires.

Retain per-key history in node state across those fires. No batch API hands a
tick every queued frame at once; the framework's own tests reach a drain-all
read through internal transport API that node code does not have.
