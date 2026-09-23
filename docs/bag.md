# `cerulion bag`: bag as a data source

Record topics into a bag; play one back onto shared memory as if a robot were
publishing it.

```bash
# On the robot:
cerulion bag record -a -o session.mcap

# On your desk, after copying the file over:
cerulion bag info session.mcap      # what's in it
cerulion bag play session.mcap --loop
cerulion viz                        # …and the shell sees a live robot
```

The point of `play` is to be a **robot substitute**. When the robot is down, or
you are working on the desk-side shell (`topic list`/`echo`/`hz`, `cerulion
viz`, Studio) and don't want a robot in the loop at all, a bag playing onto
local shared memory is indistinguishable from a robot publishing.

---

## Playback vs re-simulation: two modes of one verb

`bag play` republishes the recorded frames; `bag play --resim all` re-executes
the recorded graph. Both modes are `bag play` and `--resim` is the switch: there
is no separate replay verb, and typing `replay` after `cerulion` is an
unrecognized subcommand.

| | `bag play <BAG>` | `bag play <BAG> --resim all` |
|---|---|---|
| What it does | Re-PUBLISHES the recorded frames | RE-EXECUTES the bagged graph's cdylibs |
| Produced topics | Replayed verbatim from the bag | RECOMPUTED by the nodes |
| Speed | Wall-paced from the bag's timeline | As fast as possible, no pacing knob |
| Purpose | DRIVE consumers with realistic data | RE-RUN your current code against real inputs |
| Needs the workspace? | No | Yes: it loads the cdylibs |

Re-simulation re-injects only external-source topics and recomputes everything
else, so re-simulating a `ros2 attach` bag on a desk with no DDS robot produces
**no output**: its `dds_bridge` node still needs its external DDS inputs. Use
plain playback to view an attached ROS 2 recording without the robot.
Re-simulation is also unpaced: its loop never reads the wall clock, which is
what makes it deterministic (Principle #7). Together those two properties mean
`--resim` cannot drive a viewer.

### `--resim` is NEUTRAL; `--verify` is the verdict

A bare `--resim all` re-executes and reports what happened. It makes **no
claim** about whether the result matches the recording (divergence is the
product, not a failure), so a completed re-execution always exits **0**.

The line between the two flag classes: a flag that shapes what EXECUTES
(`--duration`, `--strict-state`) works in both modes; a flag that shapes the
VERDICT (`--report`, `--tolerance`) needs `--verify`.

`--verify` adds the byte-comparison and its stable exit contract: 0 = identical,
1 = data violation, 2 = bag I/O or not-replay-grade, 3 = node cdylib load error
or a node that panicked, 4 = tolerance-YAML validation error, 5 = internal, 6 =
structural trace divergence.

One refusal worth knowing by name: a bag (or graph file) whose declared `name:`
iceoryx2 cannot represent (longer than 128 bytes, or carrying any non-ASCII
character; an accent or an emoji is refused even though it is valid UTF-8) is
a typed refusal, never a panic. The message names the identity, the constraint
it broke (with both numbers on the length arm), and the remedy: rename the
graph, or the bag, and re-run. On a resim this is the **exit-2**
not-replay-grade arm (the recording's own declaration is what cannot run); on
a single-process `graph run` or `graph profile` it is a loud startup refusal
(a multi-process run sanitizes its worker names and proceeds). The 128-byte
cap counts the prefix too: the identity's own budget is 112 bytes on a resim
(`cerulion_replay_{identity}`).

"Neutral" scopes exactly those two *comparison* outcomes (1 and 6). It does not
soften failure to re-execute: an unreadable bag, a cdylib that will not load, a
candidate that panicked and an internal error keep their loud codes in both
modes.

**A multi-process recording's shutdown tail is tolerated, and reported.**
A multi-process run's frame stream and its per-rank trace rings
are cut independently at shutdown, so a rank that outlives rank 0 can commit a
final step's frames past the last boundary the authoritative (rank 0) stream
carries: routinely one step at SIGINT, and unboundedly many when a survivor
outlives a dead rank 0 under `--peer-loss continue`. Those trailing frames are
IN the bag and readable; a resim excludes them from what it re-executes and
from the verify comparison (its authoritative clock ends at rank 0's last
boundary, so they could match nothing and be re-produced by nothing), and the
verdict reports the derived covered range plus a per-topic count rather than
excluding them silently. The tolerance is scoped to topics a PEER rank
produces: only a peer can legitimately out-run the authoritative stream, so a
rank-0-produced frame past rank 0's own last boundary is still refused (on a
healthy recording that shape cannot occur; seeing it means the bag was edited
or the recorder misbehaved). A frame that matches no boundary INSIDE the range
is likewise still refused as a corrupt recording (the tolerance is the peer
tail, never the body), and single-rank bags are unchanged (their frames and
trace are written by one thread from one batch, so a tail frame there really
is corruption).

**The mirror shape, a final step boundary with no fires behind it, is
tolerated too.** A worker banks each step's boundary record BEFORE
running that step's fires, so a worker that exits in between leaves a trailing
boundary the recording cannot explain: the resim runs that step (the boundary
stream is its clock) and re-fires the node, against a recording that shows
nothing there. Because a boundary for step *k+1* is itself proof that step *k*
completed, only the LAST recorded step is ever ambiguous, and only when it
banked no fires, so a resim tolerates re-fires (and the frames they produce)
in that ONE step, for that rank, and nowhere else. A step proven quiet by a
later boundary is still compared at full strength, as is a final step that
recorded SOME of its fires. Both cases are reported by name, and the note for
this one says the final recorded step banked its boundary with no fires behind
it, then names the two readings without choosing between them:

> either the step ran and fired nothing, or the worker shut down between the
> step boundary and the step's fires — the recording cannot tell which

The note reports both readings because the rule reads the recording, not the
intent. It also appears on a perfectly healthy bag: a rank whose slowest
node fires every other step ends its stream on a naturally quiet step, which is
indistinguishable from a cut worker, because only a LATER boundary can prove a
step completed and the final step has none. The cost is the same under either
reading, and the note states it: an over-fire in that one
step, for that one rank, is not caught.

---

## `cerulion bag play`

```
cerulion bag play <BAG> [-r|--rate N] [--loop] [--topics TOPIC]...
                        [-s|--start-offset S] [-u|--duration D]
cerulion bag play <BAG> --resim all [--verify] [-u|--duration D] [--strict-state]
                                    [--report PATH] [--tolerance PATH]
```

The two lines are the two modes, and their flags do **not** mix: every illegal
combination is refused by name with the reason, never silently ignored.

| Flag | Mode | Meaning |
|---|---|---|
| `-r`, `--rate N` | playback | Playback rate multiplier. `1.0` = the recorded pace (default), `2.0` = twice as fast, `0.5` = half speed. Must be finite and `> 0`. |
| `--loop` | playback | Restart at the beginning when the bag ends, until interrupted. |
| `--topics TOPIC` | playback | Play only these topics (repeatable). A name the bag does not carry is a loud error listing what it does. |
| `-s`, `--start-offset S` | playback | Skip the first `S` SECONDS of BAG TIME (fractional accepted). Per CHANNEL, measured from that channel's own first frame: wire stamps in different channels are different producers' clocks, so there is no one bag-wide `t0` to seek against. Re-applied on every `--loop` pass. Refused under `--resim` by name: re-executing from the middle needs a per-rank resume ANCHOR, which no recording carries. |
| `--resim <NODES\|all>` | n/a | RE-EXECUTE the bag's graph instead of republishing its frames. `all` re-executes every node; a node subset is not supported and is refused by name rather than widened to `all`. |
| `--verify` | resim | Byte-compare every re-executed frame against the recording and apply the 0 to 6 exit contract. Without it, the run claims nothing and exits 0. |
| `-u`, `--duration D` | both | Cover only the first `D` SECONDS of BAG TIME (fractional accepted). Playback stops republishing a channel once it has advanced that far through its OWN timeline, and it stays stopped for the rest of the pass, because the one way a later frame reads as back inside the window is a stamp that went BACKWARDS, which is a producer RESTART (the epoch reset) rather than a new window; a resim stops re-executing a rank once a recorded boundary reaches the run's bag-time ORIGIN plus `D`. The window is half-open on BOTH halves (ONE rule, `replay_rank::beyond_duration_bound`), so `--duration 0` covers nothing and a bound EXACTLY equal to a frame's (or a boundary's) own bag-time elapsed EXCLUDES it. (One rule for both halves matters: if playback excluded at `> D` while a resim excluded at `>= D`, `bag play --duration 0` would republish each channel's first frame while `bag play --resim all --duration 0` executed nothing.) Legal in both resim modes: it bounds the RUN, not the comparison. **The bound is in TIME, not in steps**: a per-rank resim has k step axes and no shared step number, while a bound in TIME has a shared ORIGIN: `replay_rank::run_epoch_ns`, the MINIMUM first-boundary target across the ranks. That is what makes `D` mean one thing for the whole run; it does NOT make the run rank-uniform. Under free-run each rank's clock starts at its own live-loop entry, so the ranks share an origin rather than an epoch value, `D` names one wall interval measured from that origin rather than each rank's own first `D` seconds, and a rank that entered later loses more of its OWN tail. Expect per-rank `ticks_replayed` under a bound to differ (see `rank_execution` in the report). `--max-ticks` is not accepted: it is clap's unknown-argument error (exit 2), never a silently accepted no-op. |
| `--strict-state` | resim | Refuse the re-execution unless EVERY executed node's state was restored. Inert on a bag that begins at step 0: nothing is restored there. Legal in both resim modes: it is a PRECONDITION on the run, not a claim about the recording, so a neutral resim honours it too (its refusal is the exit-2 not-replay-grade arm, which neutral mode never swallows). |
| `--report PATH` | resim + `--verify` | Persist the machine-readable verdict JSON. Needs `--verify`: its `passed` / `violations` fields ARE the verdict a bare `--resim` declines to make. Beside the verdict it carries the per-rank REPORT-ONLY arrays, none of which move the exit code: `rank_execution` (one entry per pass: steps executed, first/last step, and whether the `--duration` bound is what stopped it; the per-rank breakdown `ticks_replayed` sums away), `injection_anomalies` (a bag-fed topic whose injected stream differed from the recording's: a sequence the plan did not name, a schedule that over-named the stream, or recorded frames a steered schedule left unaccounted for), and `replay_input_shortfalls` (a node whose trace-driven burst asked for the next recorded frame and got nothing, so it fired on its held head). Read all three before attributing a byte mismatch to the candidate: each says the replay's INPUT differed first. |
| `--tolerance PATH` | resim + `--verify` | Relax the byte-comparison per field. Needs `--verify`: there is no comparison to relax without it. |

Why the playback flags are refused under `--resim`: `--rate` has no wall pace to
multiply (a resim runs on the recording's own gating clock), `--loop` would not
re-execute the same starting state a second time, a resim's topic set follows
the nodes that execute rather than a filter, and `--start-offset` would need a
per-rank resume ANCHOR (the graph state each worker held at that instant),
which is the same thing a mid-run free-run bag is refused for; neither
is supported.

`--duration` is the one bound that crosses the line, because it answers a
question about the RUN rather than about the comparison: "how much of this
recording do I want to act on".

### What it guarantees

- **Byte-verbatim republication.** Every frame is published exactly as recorded,
  wire `sequence` and `timestamp_ns` included. A player that re-stamped frames
  would look healthy to a subscriber while lying about every timestamp a
  consumer reads.
- **Deterministic order.** Two plays of one bag publish byte-identical frames in
  an identical order: the bag's recorded (file) order, which is fixed by
  immutable bytes on disk.
- **Nothing dropped or reordered under lag.** If the machine can't keep up the
  schedule SLIPS; the frame still publishes, in order, and the slippage is
  reported at exit.
- **No per-frame allocation.** The frame path is `mmap slice → publish_raw →
  loaned SHM slot`: one memcpy (the bytes must cross from a file mapping into
  the SHM segment) and zero heap allocations in the loop.

### Topics already owned by a live producer are refused BY NAME

A topic's publisher slot is single-writer. If something is already publishing a
topic in the bag, the player refuses **that topic**, prints why, and plays the
rest:

```
REFUSED (not played):
  /imu: 1 publisher(s) are already publishing this topic on this machine — a bag
        player must not compete with a live producer for the same topic. …
```

Silence here would be the worst outcome: a viewer showing one topic and no
explanation for the missing one.

### Timing

Recorded stamps come from the **producing process's clock**, and stamps from
different producers share no number line. So playback runs **one timeline per
CHANNEL**: each topic anchors on its own first frame and advances only by its
own deltas. No channel's stamp is ever compared to another's.

For a single-writer topic, one channel follows exactly one producer's clock: the
transport provisions graph topics with `max_publishers = 1` and refuses a
second, so one channel is one producing process. A `multi_publisher_topics`
channel carries several writers' clocks and uses the restart handling below.

- Every topic keeps **its own recorded rate**, whatever else is in the bag.
- All channels share one **run origin** (the wall instant the run's first frame
  was reached), and each carries its starting deficit IN FULL, so they never
  drift apart: inter-topic alignment stays accurate to the recorder's flush
  window.
- A channel within `CATCHUP_FREE_DELTAS` (16) of its own schedule publishes
  **immediately**. Ordinary write-batching deficits are bounded by the recorder's
  held budget (at most 7 frames), so they all fall inside this window and cost
  essentially no wall time. This matters more than it sounds: the player is
  single-threaded, so spacing every behind frame would be an *aggregate*
  throughput ceiling: beyond a handful of fast topics every channel falls
  permanently behind, and the playback wall grows with the topic count.
  With the free window in place the wall stays under a ceiling as topic count
  grows (the slow residual growth is the per-frame publish cost of a debug
  build, not the ceiling).

  Past that window, playback throughput depends on the workload: a bag that
  packs thousands of publishes into a few hundred milliseconds is bounded by
  raw publish throughput rather than by pacing.

  The free window has its own cost, in the other direction: the last ≤16 deltas
  of a deep backlog fall inside it and publish at loop speed rather than at the
  4x bound. So a handover's tail arrives in a short burst. That is bounded by
  construction (16 frames) and is the price of removing the aggregate ceiling.
- Only a channel behind by MORE than that window (a genuine handover backlog of
  hundreds of deltas) is rate-bounded, at **4x** its own recorded rate
  (`CATCHUP_FACTOR`), rather than dumping. A run reports how many channels
  fast-forwarded at some point: an observation about THAT RUN, never a claim
  about the recording.

  The 4x figure is a bound on reproducing a **recorded cadence**, not a universal
  publish-rate ceiling: it divides the channel's own inter-frame delta, so a
  channel whose consecutive stamps are identical has no spacing to divide and
  publishes as fast as the loop runs. That is faithful: the recording says those
  frames were simultaneous.
- A `multi_publisher_topics` channel genuinely mixes writers' clocks; its
  regressions re-anchor like a producer restart: bounded, and reported.

Backwards stamps *within* a channel are real producer restarts, and are reported
rather than hidden:

- A backwards step **re-anchors** that channel's schedule rather than
  accumulating phantom lag; the summary reports timeline restarts separately
  from machine lag.
- The run's first frame **anchors**: it is first by construction and can never
  be "late", so a healthy run reports zero slipped frames.
- A re-anchor resets the worst-slip accumulator, so the summary also reports a
  **wall overrun** (actual wall vs the recording's own duration), which nothing
  resets.
- A gap larger than `MAX_FRAME_GAP_NS` (5 s) plays as 5 s, so a recording pause
  doesn't stall playback for an hour.

**Slip without an overrun is STRUCTURAL, and the summary says so.** A recorder
writes each flush **grouped by topic** (not in global arrival order), so
publishing in recorded order legitimately reaches a later topic's frames past
their own target, bounded by the recorder's flush window. The run still
finishes on time. The discriminator is the wall overrun: slip **with** an
overrun is the machine failing to keep up and the summary advises lowering
`--rate`; slip **without** one is batch phase and the summary says no action is
needed.

A topic can therefore hold its recorded rate with **no wall overrun** while the
summary still reports slipped frames; those are the structural case above.

### Late joiners see only what follows

The player retains no late-joiner history. A viewer that attaches mid-run sees
what is published *after* it attaches; `--loop` gives it the next pass.

### A non-finalized bag is refused

Playback needs the summary footer a clean recorder shutdown writes. A bag from a
KILLED recorder has none, so `bag play` refuses it loudly and names the reason;
`bag info` still reports what it can read.

---

## `cerulion bag record`

```
cerulion bag record [TOPIC]... [-a] [-e PATTERN] [-x PATTERN]
                    [-o PATH] [--duration SECS] [--schema-wait-ms MS]
                    [--run[=RUN]]
```

This is a **front-end over `cerulion bagd`**, Cerulion's production recorder,
not a second recorder. bagd already owns the listener-less data-only taps, the
`writev` MCAP writer, the backlog-aware drive loop, per-topic loss
accounting and the finalize-on-shutdown contract. This verb
derives a topic set, translates the shutdown flag and `--duration`, and renders
the summary.

### Local payload capture

`bag record` taps **this machine's** shared memory: no frame of any topic is
pulled across the network. Recording a robot's topics means running it **on the
robot** and transferring the file. A named topic that is not live here is
refused by name, with that instruction, never a silent network fallback that
would pull every frame of every topic across the wire.

Schema resolution is the one part that may use the network: when the local
schema corpus does not name a recorded channel's type, the recorder may ask
network peers for it within its budget. `CERULION_RECORD_SCHEMA_DEMAND_MS=0`
disables those lookups (see "Environment variables" in
[`docs/user-api.md`](user-api.md)).

**Mirrors are not local.** A `cerulion-netd` mirror of a remote
robot's topic IS a local `{topic}/data` service, so a raw service scan reports
it as local, which would let `bag record -a` on a desk running
`cerulion viz --robot go2` record that robot's stream into a bag labelled a
local capture. Auto-selection therefore folds mirrors out (via the same
`mirror_registry::partition_local_topics` predicate `cerulion topic list` and
vizd use). Naming one **explicitly** still records it (you asked for it), with
a loud note naming the origin robot; the capture is then of the mirror *as this
desk received it*, network loss included.

**Un-tappable topics degrade, they do not abort.** A topic whose subscriber
slots are exhausted, or whose producer exited since the scan, is EXCLUDED by
name and the rest still record (`ros2 bag record -a` behaves the same way). An
**explicitly named** topic that cannot be tapped is a hard failure instead:
silently omitting what you asked for would be worse. This narrows the window
rather than closing it: bagd's own attach is still all-or-nothing, so a producer
exiting between the preflight and that attach can still fail the run (see Known
limits).

### `ros2 bag record` parity

| ros2 | cerulion | Notes |
|---|---|---|
| `ros2 bag record <topics>` | same | |
| `-a` / `--all` | `-a` / `--all` | Cerulion's internal channels (`__cerulion/*`, `/bagd/*`) and `cerulion-netd` MIRRORS of other robots' topics are never auto-selected. |
| `-e` / `--regex` | `-e` / `--regex` | |
| `-x` / `--exclude` | `-x` / `--exclude` | Narrows any selection, including an explicit list. |
| `-o` / `--output` | `-o` / `--out` | |
| `-b` / `--max-bag-size` | *(not mirrored)* | One recording is one file. Size-cap rotation is not supported: it would produce N sibling files with the coverage/health manifests in only the last one, so `bag info` on any earlier file would report a recorder that "never measured what else was live", false of a recording that measured it one file along. |
| `-d` / `--max-bag-duration` | *(not mirrored)* | Neither splitting flag is mirrored. `--duration` STOPS the recording; it does not split it. |
| `--storage` | *(not mirrored)* | Cerulion writes MCAP and nothing else. |
| `--compression-*` | *(not mirrored)* | Cerulion bags are always uncompressed; the reader refuses a compressed chunk rather than mis-indexing it. |
| `--max-cache-size`, `--polling-interval`, `--no-discovery`, `--snapshot-mode`, `--qos-profile-overrides-path` | *(not mirrored)* | No Cerulion equivalent. |

Exactly ONE topic source is allowed: positional names, `-a`, or `-e`.
Combining them is an error, because the merge an operator would assume
("union? intersection?") is not knowable. Narrow with `-x` instead.

### `--run`: record a RUN, not a set of topics

`--run` ATTACHES the recorder to a live `cerulion graph run`. `ros2 bag record`
has no equivalent: a ROS bag describes topics, and this describes a run.

```
cerulion bag record --run                 # the one live run
cerulion bag record --run=perception      # by graph NAME
cerulion bag record --run=0x8f…           # by run id, as printed in run.json
cerulion bag record --run /odom /tf       # that run's identity, YOUR topic list
```

**`--run=<RUN>` needs the `=`.** A bare `--run` means *the sole live run*, and
every word after it is a TOPIC. Without the `=` requirement the flag would
swallow the first positional, so `--run /odom` would look for a run called
`/odom`; the last form above would be unspellable.

What the bag gains: the run's **effective `graph.yaml`**, its **`env.json`**,
its **`recorder.json`** (all three copied verbatim out of the run directory) and
a new `__cerulion/run.json` carrying the run id, the attach instant and
`attached_mid_run: true`.

| Behaviour | Why |
|---|---|
| The topic set is what the run **DECLARES** | `--all` would sweep in a co-tenant graph, an external publisher or another bridge. |
| A declared topic with **no producer** costs that topic only | A graph legitimately holds outputs that have not fired yet; refusing the run over one would make the flag unusable. It is named on stdout and recorded in `record_coverage.json` as `declared_not_live`. |
| **Zero** live declared topics REFUSES | Nothing about the run would be captured, so a bag would be worse than the refusal. |
| **Live-service discovery is ON** | The declared set is inferred from the graph's outputs, which need not include every live producer. `CERULION_RECORD_DISCOVERY=off` opts out. |
| Several live runs and no id REFUSES | Recording the wrong run looks exactly like recording the right one. |
| **No** live run records standalone, loudly | An earlier `graph run` publishes no run record and is indistinguishable from no run. |
| Every channel is `attached_late` | The recording starts where it attached; nothing before it is recoverable. |

**Scheduler trace.** Every MULTI-PROCESS `graph run` creates its per-rank trace
rings, recording or not, and declares them in its `run.json` as it creates
them. That is what makes attaching to a live run worth doing: the recorder picks
the trace up from the attach point (the partial head step is discarded, so the
trace opens on a complete step), so a bag taken off a run nobody decided in
advance to record is still re-executable. Rings created only under `--record`
would make the black box a recording-only feature, so they are created regardless.

The other consumer of those rings is the always-on rolling window every serving
graph holds; see [`docs/flashback.md`](flashback.md) for `cerulion flashback`,
what a capture carries, and the two switches that turn the black box off.

**Node-state anchors.** The trace says WHAT ran; a re-execution also needs a
checkpoint to start FROM, and those per-rank state rings admit exactly ONE
reader: a second one laps the first. So a run records whether anything is
already draining them, in `run.json` under `state_ring_consumer`
(`standing` / `none: <reason>` / absent-is-unknown), and this verb reads it. When
the run reports a standing Flashback recorder the attach DECLINES the state
plane, warns, and writes a `state_rings` verdict beside `trace` in the bag's own
`__cerulion/run.json`, where `cerulion bag info` prints it, as a `state rings:`
line. (A bag that is not a mid-run attach has no such line, which is how you
tell the two apart.) Such a bag records frames and carries no
`state_coverage.json`, so `bag play --resim` will refuse it: anchors for that
run live in the standing recorder's captures. Whether it also carries a
scheduler TRACE is a separate fact the run states under `trace_rings`: a
multi-process run declares rings and the bag gets one; so does a
`--single-process --record` run, which creates a rank-0 ring for its own
recorder and declares it (`trace_rings: declared`, same as the multi-process
shape). The monolith shapes that mint none (a plain `--single-process` run,
`ros2 attach`, `node run`, an external time source) write no `trace_rings` key
at all, so the bag reports the absence as the legacy by-omission cause and gets
neither ring nor trace. Every UNKNOWN
state proceeds instead, exactly as a manifest without the key does, and says so.

They are not free. One ring per rank reserves ~40.06 MiB APPARENT
(`65_600 + 2^20 × 40` = 42,008,640 B), plus one 106,560 B supervisor departure
ring. The segment is `ftruncate`d rather than written, so it costs a page at
first and converges on the full figure only as the ring fills.
`graph run --no-rings` declines them for a robot that would rather spend the
memory elsewhere; such a run gets frames and attachments but **no trace**, and
`__cerulion/run.json`'s `trace` field says exactly that rather than leaving a
reader to conclude the recorder lost something.

The MONOLITH shapes (`--single-process`, `cerulion ros2 attach`, `node run`)
mint no ring either, and that is a decision rather than an omission: their gating
clock is WALL-driven, so a trace taken there would carry boundaries a resim
cannot re-advance to, i.e. a `resimmable: true` that is confidently false.
Routing them onto the recording clock's discipline is not implemented.

No `bag record` flag can add a trace to a run already under way: the rings are
the run's to create, and it has already started.

A `trace` string is never inferred from an empty ring list, because an empty
list is several different facts. The run states which one in its own manifest,
under `trace_rings`.

**All three states are written.** `declared` is what every multi-process run
writes. `declined` is `graph run --no-rings`: the run had the choice and took
it. `unavailable` is a run that WANTED rings and was refused them: the
`/dev/shm` free-space gate, or a departure-ring create failure on an otherwise
plain run. The reader still keeps `absent ⇒ unknown`, because absent stays
reachable three ways (an earlier binary, one of the monolith shapes above,
or a declaration that was never written), and only `declined` and `unavailable`
license a positive claim about WHY a bag carries no trace.

| `run.json`'s `trace_rings` | Means | `trace` says | Written by this build? |
|---|---|---|---|
| `declared` | Rings were created; `rings` names them | `from the attach point`, unless a declared RANK is missing (below) | **yes**: every multi-process `graph run`, recording or not |
| `declined: <reason>` | The run DECLINED rings at launch, by CHOICE | `none: this run DECLINED …`, carrying the run's reason | **yes**: `graph run --no-rings` |
| `unavailable: <reason>` | The run wanted rings and was REFUSED, by a resource gate or a run shape whose clock cannot produce a resimmable trace | `none: this run wanted … and could not have them` | **yes**: a `/dev/shm` free-space refusal, or a failed departure ring on a plain run |
| *absent* | The run said nothing: an earlier binary, a MONOLITH shape (`--single-process` / `ros2 attach` / `node run`), or a declaration that was never written | the legacy `none: the run's manifest declared no trace rings …` | **yes**, on those shapes |
| *anything else* | A state THIS BUILD does not know: a manifest written by a newer `cerulion` | `unknown: … a scheduler-trace state this build does not know (\`<token>\`) … upgrade this build to read it` | n/a: read, never written |

A present-but-unreadable value and an absent key look identical to a reader
that only asks "do I have a known state?", and they are facts about different
parties. Absent is about the **run**: it said
nothing, so the legacy cause is accurate. Unrecognised is about **this build**:
the run said something and this binary cannot read it, so the report says that
rather than reporting that the run declared no trace rings. The unreadable
token is quoted back (bounded, since it comes from a file another process
wrote) so the remedy is actionable.

Ring tags are stamped **before** the rings are created, so `rings` can name a
rank whose ring creation FAILED. That rank is recorded by number in
`declared_unavailable`, and `trace` then reads `partial: rank(s) N …`, never
`from the attach point`, which would claim a trace this bag does not carry for a
ring nothing could open. The supervisor builds that list from each worker's OWN
report, carried back through its READY sentinel, and never by inference: a rank
that made no statement is UNKNOWN and appears in NEITHER `declared_unavailable`
NOR the recorder's `--ring` set, because "this rank is unavailable" is as
positive a claim as "this rank works". Each of these strings names the CAUSE and
no flag: the reader is holding a bag, and no `bag record` flag can put a trace
into it.

`run.json` also carries `gating`, which records how the run's gating clock
advances (`quantum` / `recorded_wall` / `wall` / `polled`). It is there because
a resim judge cannot recover it from a bag: a boundary produced by a clock the
scheduler ADVANCES and one produced by a clock it merely READS are
byte-identical records with opposite meanings.

**`run.json`'s `trace` is the INTENT; `record_coverage.json` is the OUTCOME.**
Those two cases are the healthy ones. There is a third, and it is ordinary
rather than a corner: the run's directory is being **removed while you attach**.
A directory is deleted when the run ends, an `Ending` run stays deliberately
attachable (refusing it would make the verb race the run's own teardown), and a
ring's shared-memory name is unlinked the instant its owner drops. So a
recorder that meets an exiting run finds a manifest, or the rings it names, or
both, already gone. That costs the **trace**, never the frames: the frames are
the part that cannot be obtained later.

MCAP attachments are written when the bag is **created**, strictly before any
ring is opened, so such a bag's `run.json` was already stamped and cannot be
corrected in place. The correction is durable rather than a log line, and which
artifact carries it depends on what was lost:

| What was lost | `run.json`'s `trace` says | Where the outcome is |
|---|---|---|
| A declared ring's SHM segment | `from the attach point` (the intent) | `record_coverage.json`'s `rings_declared` + `rings_unavailable`, each ring with the transport's own error |
| The manifest itself | `unknown: … could not be read` | `artifacts_unreadable` in the same `run.json`, naming each file and why |
| The manifest, mid-rewrite | `unknown: … could not be parsed` | `run_manifest_unparsed` in the same `run.json`, carrying the bytes that were read |

The first is the split proper: `run.json` says what was intended, and
`cerulion bag info` prints the verdict:

```
trace: NONE — this recording declared 1 scheduler-trace ring(s) and could open 0.
```

`NONE` when nothing opened, `PARTIAL` when only some did. **When the two
artifacts disagree, `bag info` is right.**

The other two cannot use that channel: with no usable manifest there are no
*declared* rings, so nothing is ever recorded as unavailable and that line
never fires. `trace` therefore says **unknown** outright, because "the run
declared none" would be a claim about the run drawn from the recorder's own
failure to read it.

They stay distinct because their evidence lives in different keys, and each
`trace` string points at its own. A manifest that could not be READ leaves an
`artifacts_unreadable` entry; one that was read but would not PARSE does not
(the read succeeded) and is carried verbatim under `run_manifest_unparsed`
instead. The mid-rewrite case is ordinary rather than a corrupt-disk corner: a
run declaring its trace rings rewrites `run.json` in place, so a recorder that
attaches during that write reads an empty file. It is usually transient:
re-attaching gets the finished manifest.

A missing trace deliberately does **not** mark the coverage `INCOMPLETE`: that
verdict answers *which live producer is missing from this bag*, a different
question with a different remedy. It does withhold `coverage: COMPLETE`.

### Grade: observability, not replay

Each channel carries the **real wire `schema_hash`**, learned from that topic's
first frame: that is what makes the bag playable. The schema **name** does not
come off the wire at all, so the recorder resolves the hash against this
machine's schema corpus to get it; a hash nothing here can resolve
records `"unknown"` rather than a guessed name.

Consequences, stated plainly:

- `cerulion bag play` works fully (it validates against the hash).
- `cerulion topic echo` decodes the frames (it resolves names from the workspace
  schema store).
- **`--resim` compatibility is NOT claimed.** Re-execution needs the bagged
  graph, its env and the scheduler trace. Use `cerulion graph run --record` for
  a replay-grade bag.

A topic that is silent through the whole `--schema-wait-ms` window gets a
hash-0 placeholder, permanently for that bag: MCAP channels are immutable once
written.

### Loss is reported, never hidden

A data-only tap has no listener, so it is never sent late-joiner history: it
records only what is published after it attaches. The summary reports bagd's own
per-topic accounting, rendered from its **per-tap** health record so that:

- a **selected-but-silent** topic still gets a row (you can tell "never
  selected" from "selected but nothing arrived"; on a `-a` capture that is the
  difference between a working robot and a dead sensor);
- a topic whose loss detection was **disabled** shows `?` rather than `0`.
  bagd zeroes the counters as unattributable when a wire-sequence anomaly
  disables gap detection, and when a topic is declared multi-publisher it never
  arms detection at all. Rendering those as `0` would claim a verification that
  never happened, so each says why: *UNKNOWN, not zero* / *UNKNOWN by
  construction*.

The bag carries the full detail (including first/last sequence and the
reconciliation terms) in its `__cerulion/record_health.json` attachment, which
the summary always names.

---

## Coverage: what a bag contains, and what it does not

`record_health.json` answers **"of the topics I tapped, what did I lose?"**. It
cannot answer **"what existed, and did I tap it?"**. A bag can finalize with
`frames_lost = 0`, truthfully, while live producers stream outside the tapped
set entirely and contribute no loss to count.

So every finalized bag carries a second, separate attachment:
`__cerulion/record_coverage.json`.

### What it says

```jsonc
{
  "version": 1,
  "enumerated": true,            // live-service enumeration RAN and succeeded at least once
  "discovery_requested": true,   // ...and it was asked for (vs the caller opting out)
  "enumeration_failures": 0,     // scans that failed; >0 means the list below is incomplete
  "all_channels_exact": false,   // DEPRECATED, tap mode only (a discovered channel is attach-mode); read replay_grade
  "replay_grade": "partial",     // "full" | "partial" | "observability": channels with a name AND an obtainable definition
  "tapped": {
    "/imu":      { "source": "declared",   "frames_recorded": 400, "attached_late": false },
    "/lowstate": { "source": "discovered", "frames_recorded": 91,  "attached_late": true  }
  },
  "untapped": {
    "/camera/h264": { "reason": "appeared_after_bag_creation" },
    "/bagd/status": { "reason": "excluded_internal" }
  }
}
```

- **`source`**: `declared` (the caller named it) vs `discovered` (live-service
  enumeration found it). A discovered tap is attach-mode by construction, so it
  learns its schema from the first frame.
- **`attached_late`**: the tap attached *after* the recording started, so its
  topic is covered only from that instant. A data-only tap requests no
  late-joiner history, so there is no mechanism by which it could be otherwise;
  the marker exists so the row cannot be read as full coverage.
- **`untapped`**: every live producer that is NOT in the bag, with its reason:
  `excluded_internal`, `remote_mirror`, `budget_exhausted`,
  `appeared_after_bag_creation`, `attach_failed`. The first two are exclusions
  by RULE (the recorder's own status channel; another robot's `cerulion-netd`
  mirror) and are not counted as coverage gaps: counting them would train you
  to skim the number that matters.
- **`enumerated`**: the flag that stops an empty `untapped` list from being
  mistaken for a coverage guarantee nobody checked.
- **`replay_grade`**: `full` when every channel carries a schema name and a
  definition a reader can obtain, `partial` when some do, `observability` when
  none does. Absent on an older bag, which claims nothing either way. Read this
  rather than `all_channels_exact`, which reports only the tap
  mode and is `false` for any bag with a discovered channel, even
  a fully described one.

### How to read it

`cerulion bag info` prints it:

```
record coverage: 2 topic(s) tapped (1 declared, 1 discovered); live-service enumeration RAN.
TOPIC                                       FRAMES  TAPPED VIA
/imu                                           400  declared
/lowstate                                       91  discovered  [attached after the recording started — no back-fill]
coverage: INCOMPLETE — 1 live producer(s) existed that this bag does NOT contain:
  /camera/h264                             appeared_after_bag_creation
  appeared_after_bag_creation: MCAP channels are registered when the bag is created and are immutable afterwards, so a producer that appears later can be given no channel. Hold bag creation open longer by setting `CERULION_RECORD_DISCOVERY_SETTLE_MS=<ms>` in the recording's environment — that reaches every path, including `cerulion graph run --record`, which builds the recorder's command line itself. `cerulion bagd --discovery-settle-ms <ms>` is the same knob for a recorder you invoke by hand.
also untapped, by rule (not a coverage gap): /bagd/status [excluded_internal]
per-topic coverage detail (incl. each untapped topic's full reason) is in the bag's `__cerulion/record_coverage.json` attachment.
```

That block is the renderer's own output for the manifest above it, kept in step
by `bag_cmd::tests::the_bag_info_example_in_docs_bag_md_is_what_the_renderer_prints`.
Two rows carry more than their tag when they have more to say: a `remote_mirror`
names the origin robot, and an `attach_failed` carries the transport's own
message: the tag alone does not distinguish two robots' mirrors, nor slot
exhaustion from a service that vanished.

A bag with **no** manifest predates the manifest itself, and `bag info` says so
explicitly: an absence of information, never a clean bill of health. A bag that
is not finalized says something different again: its attachment index is
missing, so the manifest could not be *looked at* (it may well be in the file).

### `bag record` opts OUT of discovery

Discovery defaults ON for the `--topics-json` path (`graph run --record`), whose
tap set is INFERRED from the graph's declared outputs, which need not name
every live producer. It defaults OFF for `cerulion bag record`, which owns
its own selection policy: `--topic` is an explicit list an auto-add would
overrule, and `--all` / `--regex` already enumerate the live topic set
themselves (applying the same excluded-prefix list and the same netd-mirror
fold). Such a bag records `enumerated: false`, so a reader is told that its
empty `untapped` list is not a coverage claim.

### Turning discovery OFF

There are two switches, and which one you can reach depends on who launched the
recorder:

| Switch | Reaches |
|---|---|
| `CERULION_RECORD_DISCOVERY=off` | **Every** path, including `cerulion graph run --record` |
| `cerulion bagd --no-live-discovery` | Only a `cerulion bagd` you invoke **yourself** |

The flag is the more obvious of the two and is unreachable from the one path
where discovery is ON by default: `graph run --record` builds the recorder's
command line internally (`--out`, `--topics-json`, `--ring`, `--ready-file`,
`--attach`, and optionally `--iox2-config` / `--schema-catalog`, nothing else),
so there is nowhere to type it. That is why the environment switch exists; it
mirrors `CERULION_NETWORK=off` and `CERULION_TOPIC_LIVENESS=off`. Only the exact
value `off` disables; anything else is a **loud** warn and leaves the command
line's own decision standing, so a typo cannot quietly turn coverage off.

The same asymmetry applies to the discovery SETTLE window (how long bag creation
is held open for the live topic set to stop changing):
`CERULION_RECORD_DISCOVERY_SETTLE_MS=<ms>` reaches every path;
`cerulion bagd --discovery-settle-ms <ms>` is the same knob for a hand-run
recorder.

Turning discovery off does not make a bag quieter about what it holds: such a
recording carries `enumerated: false` / `discovery_requested: false`, which
`bag info` reports as *"this bag makes NO claim about what else was live"*,
never as clean coverage.

---

## The replay contract a bag declares

A replay-grade bag (`cerulion graph run --record`) declares, in
`__cerulion/recorder.json`, the two things re-execution must not guess: the
trace stream's FORMAT and the run's COORDINATION contract.

| Key | Answers |
|---|---|
| `trace_format` | "Can this binary decode my trace records at all?" A reader refuses a bag stamped ABOVE its own supported version, rather than decoding a stream it does not understand |
| `coordination` | "Which execution contract was I recorded under?": `lockstep` or `free_run` |

### The `coordination` contract, and what each reading does

| Stamp | Bag it came from | Behaviour |
|---|---|---|
| **absent** | Any bag recorded before the stamp existed | Treated as `lockstep`, and the verdict SAYS the inference out loud: `coordination: lockstep (inferred: no coordination stamp)`. An absence is not a claim, so it is never printed as though it were one |
| **`lockstep`** | The barrier multi-process path, and every MONOLITH recording (a monolith is the degenerate one-rank lockstep timeline, so it stamps the same word) | The original contract, unchanged: one authoritative clock, cross-rank boundary equality, single-anchor mid-run resume |
| **`free_run`** | The free-run multi-process path: per-rank wall-faithful boundary streams sharing only the GO epoch | Re-executed **per rank**; cross-rank boundary equality is not applicable and is not checked. Such a bag also stamps a `trace_format` past 3, so an older binary refuses it rather than silently applying the lockstep contract to a stream built to violate it. See [`docs/multi_process.md`](multi_process.md) |
| **anything else** | A newer recorder, or a hand-edited bag | **Exit 2**, naming the value. Deliberately NOT inferred to lockstep: an unknown coordination is precisely the case where a guess is a silent mis-replay |

**Because every NEW bag stamps the key explicitly, an ABSENT key means exactly
"recorded before the stamp existed".** That is what makes the inferred reading safe to print as a
fact rather than a hedge.

**Recording a `free_run` bag is an opt-in.** A multi-process
`graph run --record` under `CERULION_EXECUTION_MODE=free_run` stamps `free_run`
and records each rank's own wall-faithful timeline from the shared epoch; the
reader re-executes such a bag PER RANK. Without the variable every bag
`graph run --record` produces stamps `lockstep`, and that is the default.
The variable is an execution-mode switch rather than a tuning knob; see
[`docs/multi_process.md`](multi_process.md) for the contract. All four rows
describe current behaviour.


**A stamp never under-claims, which means every bag this build writes with a
`recorder.json` stamps 6 by default, or 5 when it was recorded with
`CERULION_READ_LOG_FOLD=off`.** (A bag with no recorder attachment carries no
stamp at all and reads as the "absent" row above, not as a 6; a
`cerulion bag record` without `--run` pushes none.) The rule: a
bag is stamped at whatever format a reader needs in order to decode everything
in it. Two encodings set the number. Every kind-6 record carries a
READ-SITE ROLE (`ReadSiteRole`, bits 14..16 of the record's meta word), and
reading those bits as roles is exactly what `trace_format` 5 means. The
recorder also FOLDS a run of consecutive identical reads into one counted
record, and reading that count is what `trace_format` 6 means. Folding is on by
default, so a bag this binary writes is a format-6 bag whether it is lockstep
or free-run; `CERULION_READ_LOG_FOLD=off` turns folding off, and a bag recorded
that way stamps 5, so a bag never claims an encoding it did not use. A bag
stamped 3 or 4 was recorded before the role bits existed: 4 when it is free-run
or its trace carries something a format-3 reader cannot decode, else 3. This
build writes neither.

Nothing REFUSES an older stamp. A format-5 bag replays with every record read
as one occurrence (the run count's half of the word is structurally zero
there). A format <= 4 bag replays with every
read-site rule on its pre-roles arm (the bits were never written, so
`read_site_role` decodes `Unstamped` and the KIND is the site vocabulary). What
the raised stamp costs is a bag this build writes being unreadable by a binary that
predates the format it stamps, which is the refusal the version gate
exists for.

The two-bit role subfield carries THREE sites: `drain` (1), `body` (2) and
`peek` (3), the last being a scheduler read that popped a frame to look at its
stamp and parked it as the per-set Sync matcher's `next_head`. Format 5
defines all three roles, so no bag on disk changes meaning. The field is full:
a fourth site needs a wider subfield and a format bump. See `docs/read_log_forensics.md` for the role
table and what each `(kind, role)` pairing means offline.

### The kind-6 annotations ride INSIDE the record kind

The read log's two annotation kinds, the OVERFLOW MARKER (outcome kind **6**)
and the PRODUCER TOKEN (outcome kind **7**), are *outcome* kinds in the low
half of a READ-OUTCOME record's packed field. They are **not record types of their own**,
and that encoding has two consequences:

- the record-type gate hard-refuses record kinds ≥ 7 **regardless of the
  stamp**, so a record type of their own would push every marker-bearing or
  token-bearing LOCKSTEP bag up a format. A format bump is for a WIRE change,
  which is what the role bits and run folding each are; an annotation a reader
  can pass through is not one;
- inside record kind 6, a reader that predates the annotations passes both
  through, decodes every field to the value the writer meant, and renders the
  kind as `unknown(6)` / `unknown(7)`. It reports rather than refuses.

Both annotations appear on LOCKSTEP bags too (the truncation residual and
multi-publisher edges exist under current recording), so they are carried
inside record kind 6 rather than as record types of their own. What each one
means, and how to join them offline, is
[`docs/read_log_forensics.md`](read_log_forensics.md).

---

## Schema resolution for viewers

**Resolution keys off the wire HASH, never the recorded name.** A viewer decodes
a frame by its `WireHeader.schema_hash`; a bag's schema NAME never reaches it.

### What a bag carries about its own types

A Cerulion MCAP schema record carries an 18-byte descriptor (recipe +
`schema_hash` + fixed size) plus the qualified schema name. That is per-CHANNEL
and cannot hold a schema CLOSURE: `unitree_go/LowState` needs
`unitree_go/BmsState` and friends, which are not channels of the bag at all. So
the definitions ride ONE bag-level attachment, `__cerulion/schemas.json`,
alongside `__cerulion/recorder.json` and `__cerulion/record_health.json`:

| Field | What it is |
|---|---|
| `docs` | The verbatim `.msg`/YAML text of every CUSTOM type the recording's frames use, closure-complete (each doc names the others it references) |
| `hashes` | `schema_hash` → qualified name, for every type the recorder could resolve, built-ins included |

**Built-in text is deliberately omitted.** Every Cerulion binary compiles
`native_ros2_messages` in, so shipping `sensor_msgs/Image`'s text would be bytes
nobody reads. Their `hashes` binding is written anyway: it costs tens of bytes and is
what turns an `unknown` channel name into `sensor_msgs/Image` in every standard
MCAP reader.

The recorder resolves against the workspace `schemas/` store, the workspace YAML
schemas, and the built-in corpus (the SAME corpus `topic echo` decodes against),
plus `DDS_BRIDGE_CONFIG`'s `msg_dirs` when set. What lands in the bag is the
CLOSURE of the hashes actually recorded, so a robot holding 85 vendor types and
recording four carries four plus what they reference.

**Both record verbs carry it.** `cerulion bag record` runs the recorder
in-process; `cerulion graph run --record` spawns it as a subprocess, so the CLI
writes the catalog to a scratch file and hands it over with `--schema-catalog`
(the same file-across-the-spawn shape as the graph, env and recorder-identity
attachments). Single-process and multi-process recordings take the same path, so
the file format promises the same thing whichever verb wrote it.

Two guards:

- If the catalog file is ever unreadable, the recorder REFUSES to start, naming
  it. (Only reachable by invoking `cerulion bagd` by hand; the CLI writes that
  file itself.)
- Before the spawn, `graph run --record` checks the topics it is ABOUT to record
  against the catalog and **warns, naming them**, when a hash resolves to nothing
  this machine knows. That is the reachable case (edit a `.msg` without
  rebuilding its node and the compiled-in hash stops matching the workspace text),
  and without the check the recording would look healthy until someone opened
  the bag on another machine.

Two consequences worth stating plainly:

- Recording from **inside a workspace** that holds the `.msg` files is what makes
  a bag self-describing. `cerulion bag record` in a bare directory still records
  perfectly; it just cannot name or define a type nothing on that machine knows.
- A bag whose recorder resolved **nothing at all** (no text and no bindings)
  grows no attachment and is byte-identical to a pre-attachment recording. A bag
  whose topics are merely all BUILT-IN is a different case: it still gets an
  attachment, carrying the hash→name bindings (and no text), which is what lets
  any reader name its channels.

### What `bag info` / the `bag play` banner report

Each channel resolves to exactly one of SEVEN readings (six markers plus the
unmarked case), and they are kept apart because they mean different things and
carry different remedies:

| Reading | Meaning | Renders? |
|---|---|---|
| (no marker) | The hash resolves against THIS machine's own corpus | yes |
| `[from the bag's own schema records]` | This machine never compiled it; the bag brought the definition | yes: `bag play` hands it to the viewer |
| `[defined by the bag, but the viewer cannot use that form]` | The definition is workspace-YAML. `topic echo` / `bag info` read it; `cerulion-vizd` parses only ROS `.msg` (the YAML parser lives in the CLI engine, which the daemon must not depend on) | decodes, does not render: express the type as a `.msg` and re-record |
| `[defined by the bag under a BUILT-IN name — the viewer refuses it]` | The bag carries a definition under a name that is ALSO a built-in; the viewer refuses to guess which one is meant | no: rename the type, or re-record against the built-in |
| `[named, but this build cannot decode it]` | The bag says what the type is and carries no text, and this build's copy hashes differently, a corpus skew | no |
| `[hash resolves to nothing here]` | Nothing local knows it and the bag says nothing about it | no: re-record so the bag carries the definitions, or point `DDS_BRIDGE_CONFIG` at a `msg_dirs` |
| `[no Cerulion descriptor — unplayable]` | The channel carries no Cerulion descriptor at all, so there is no hash to resolve and the route is refused outright. Deliberately NOT folded in with the row above: that row's remedy is about resolving a hash, and cannot help a channel that has none | no: the channel was not recorded through Cerulion |

A recorded name that DISAGREES with what the hash resolves to is also called out
(`[recorded as this name, but the hash is X here]`): the two ends are looking at
different definitions of one type name, which is exactly the shape that renders
nothing while looking fine.

### How a played bag reaches the viewer

`cerulion-vizd` builds its `FrameWalker` at daemon boot from
`native_ros2_messages::BUILTIN_MSGS` plus, if `DDS_BRIDGE_CONFIG` is set, that
config's `msg_dirs`. The other way to grow it at runtime, a REMOTE attach's
schema fetch, is unreachable for a played bag, whose frames land on LOCAL
shared memory. Without a third source a vendor-typed bag renders nothing, and
it does so quietly: vizd attaches `ok: true`, `cerulion viz` prints
`attached … → (resolving…) [(pending)]` and exits 0, and the only signal is one
`warn!` per unknown hash in `vizd.log`.

So vizd takes a `schemas` control request that side-loads definitions into that
walker, and `bag play` offers the bag's definitions over it while it plays:

```
cerulion bag play go2.mcap        # publishes, and keeps offering its definitions
cerulion viz /lowstate            # spawns vizd if needed; it learns the type and renders
```

The offer REPEATS (every 2 s) rather than firing once, because of the order the
two commands run in: `cerulion viz` can only attach to a topic that already
exists, so the player necessarily starts first, usually before any vizd is
running, since `cerulion viz` is what spawns one. Re-offering is free at the
daemon (seeding definitions it already holds rebuilds nothing). It is
CONNECT-ONLY: `bag play` never spawns a viewer, because it is also how frames are
fed to `topic echo` and to tests.

For a bag that carries no definitions (recorded by an older build, or outside a
workspace), the remedies are, in order: re-record from inside the workspace
holding those `.msg` files, or point `DDS_BRIDGE_CONFIG` at a config whose
`msg_dirs` includes that directory BEFORE vizd starts.

---

## Known limits

| Limit | Why | Remedy |
|---|---|---|
| Playback is LOCAL only: frames land in this machine's SHM | Serving a played bag to another machine composes from existing pieces (run the player on one machine, let a desk demand its topics) but is not wired here | Not supported; play the bag on the machine that reads it |
| Inter-topic alignment comes from file position, so it is accurate to the recorder's flush window (~100 ms), not better | Cross-channel stamp comparison is the forbidden operation: stamps from different producers share no number line | None: inherent to the design |
| A channel behind by more than 16 of its own deltas fast-forwards at up to 4x its recorded rate until it catches up | The deficit is carried in full so alignment is preserved; only the RATE is bounded, using the channel's own cadence. Routine write-batching deficits fall inside the free window and cost no wall time | None: inherent to the trade-off (the alternative is an unbounded backlog dump) |
| The 4x bound does not apply to a channel whose consecutive stamps are identical | There is no recorded cadence to divide; such frames publish as fast as the loop runs | None: faithful to the recording (it says those frames were simultaneous) |
| bagd's **declared** tap attach is all-or-nothing, so the preflight narrows but cannot close the exit-between-probe-and-attach window | The preflight probe is itself a TOCTOU: a producer can exit between it and bagd's own attach. (A DISCOVERED tap is deliberately not all-or-nothing: an inference is not a request, so its failed open is ledgered as `attach_failed` and retried, never fatal) | None: the declared attach fails as a whole, so start the recorder once every declared producer is stable |
| **With discovery ON** (`graph run --record`, or a hand-run `cerulion bagd` that was given `--topics-json`): a producer that appears AFTER the bag is created is NAMED, not recorded, and that is permanent | MCAP channels are registered when the bag is created and are immutable afterwards (`BagWriter::create` takes the complete topic list up front, and rotation reuses that set verbatim), so a topic discovered later can be given no channel. Discovery closes the window BEFORE creation (bagd enumerates the live service directory, re-enumerates whenever the kernel reports that directory changed, and HOLDS bag creation open while discovery is still finding things) and reports whatever falls outside it: the topic lands in `record_coverage.json` as `appeared_after_bag_creation`, `cerulion bag info` prints it, and the run's terminal line escalates to a WARN | Widen the pre-creation window: `CERULION_RECORD_DISCOVERY_SETTLE_MS=<ms>` on every path (including `graph run --record`, which builds bagd's argv itself), or `cerulion bagd --discovery-settle-ms <ms>` on one you launch by hand. **Not** `--schema-wait-timeout-ms`, which is a force-create deadline for schema learning, not a hold-open. A producer that appears after the bag is created cannot be added to that bag |
| **With discovery OFF** (which is `cerulion bag record` always, and any recorder run under `CERULION_RECORD_DISCOVERY=off`), a topic that appears mid-recording is neither recorded NOR named | `bag record` sets `discover_live = false` (it owns its own selection policy), and that flag short-circuits discovery entirely, so nothing is watching for the topic and there is nothing to write into `untapped` either. The bag records `enumerated: false`, which is the accurate form of "nobody looked", but it cannot name what it did not look for | Nothing widens a window that is not open. Re-run the capture with the producer already live, or use `graph run --record` (where discovery is on) if you need the recorder to pick up producers that register after it arms. `--discovery-settle-ms` is **inert** here: the hold is released immediately when discovery is off |
| A non-finalized bag cannot be played | The zero-copy frame walk needs the summary footer | None: only a finalized bag is playable |
| A type NEITHER this machine NOR the bag defines still renders nothing | Nothing anywhere knows it: the bag can only carry what its recording machine could resolve | Re-record from a workspace that holds the `.msg` files |
