# Flashback: the always-on black box

Something went wrong on the robot thirty seconds ago and nobody was recording.

That is the situation Flashback exists for. Every serving Cerulion graph on
Unix holds a **rolling window** of the recent past in memory: the last ~30 seconds of frames,
plus the scheduler's own record of what fired and when. When you ask for a
capture, that window is written out as an ordinary bag. Nothing had to be armed
in advance.

Flashback is a dashcam for your robot: the runtime is always holding the last
moments, and a fault or a manual trigger keeps them.

```bash
# On the robot, while the graph is running, in another shell:
cerulion flashback --note "arm jerked left on the pick"

# It prints the path it wrote and waits until the bag is on disk:
#   recordings/flashbacks/flashback_<graph>_<ms>_p<pid>_0000.mcap
```

Then, on your desk, after copying the file over:

```bash
cerulion bag info    <capture>.mcap    # what happened, and what the bag can prove
cerulion bag play    <capture>.mcap    # republish the frames as if the robot were live
cerulion bag play    <capture>.mcap --resim all   # RE-RUN your nodes on that input
```

That last command is the point of the whole feature, and it deserves its own
section.

---

## Re-execute a capture with its recorded context

A bag of frames tells you *what the robot published*. It does not tell you what
your code would do differently if you fixed it. To answer that, the bag has to
carry the **scheduler trace** as well as the frames: the record of which node
fired at which step, on which clock. With that, `cerulion bag play --resim` can
re-execute the graph's nodes against the recorded input on the recording's own
gating clock: the same execution, replayable on your laptop.

Multi-process runs, the default shape on Linux and macOS, provision the
scheduler-trace rings without requiring `--record`. Declining those rings with
`--no-rings` also stops the window recorder. Each capture reports whether its
retained trace and checkpoint boundary support resim in `anchor.resimmable`,
with a reason when they do not; `cerulion bag info` shows that verdict.

What makes that possible is that **every multi-process `cerulion graph run`
provisions the scheduler-trace rings**, whether or not you asked it to record.
Requiring `--record` in advance would be exactly backwards, because the runs
you most want to re-run are the ones nobody expected to go wrong.

Some run shapes mint no ring: a single-process run, `cerulion ros2 attach`,
`cerulion node run`, or a virtual or external time source. They still take
captures, and those captures are frames-only. Each capture records which of the
two it is; see [Limitations](#limitations).

---

## Taking a capture by hand

```
cerulion flashback [--note TEXT] [--pin] [--no-wait]
```

| Flag | What it does |
|---|---|
| `--note TEXT` | A note written into the capture, so a bag found three weeks later says what it was about. |
| `--pin` | Excludes this capture from retention eviction, so it cannot rotate away. |
| `--no-wait` | Returns as soon as the capture is accepted, instead of waiting for the bag to land. The accepted line already carries the path it will have. |

The verb publishes a request and, by default, waits, so you are handed a file
rather than a promise. It is Unix-only: the recorder that holds the rolling
window does not run elsewhere, and on other platforms the verb says so. The
printed path is the name the capture reserves; if something already holds that
name, an `_N` suffix is appended.

### Captures the runtime takes on its own

A capture does not need a human. The runtime requests one when it sees a fault,
and each trigger has its own switch, `CERULION_FLASHBACK_ON_<TRIGGER>`, set to
`on` or `off`:

| Trigger | Fires when | Default |
|---|---|---|
| `WORKER_DEATH` | A worker process exits without being asked to. | on |
| `PANIC_DISABLE` | A node is disabled after repeated panics, or its entry mutex is poisoned. The process is alive and one node has stopped. | on |
| `RUN_VANISHED` | The run this recorder was bound to disappeared without announcing anything: a hard crash. | on |
| `ESTOP` | A human engaged the e-stop over the ops plane. | on |
| `DECLARED` | Something declared an incident. | on |
| `STALL` | A topic stops producing while its process stays alive. | on |
| `RATE` | A topic's rate collapses inside the window. | on |
| `SILENT` | A route that has never produced at all. Off by default: on a robot where that is ordinary, it would capture on every boot. | off |

An unrecognized value keeps the row's default and prints what you typed; it
never guesses a direction. Automatic requests pass through the capture gate's
coalescing and rate limits, and a manual request stays available alongside them.

### What a capture covers

| | Default | Knob |
|---|---|---|
| How far BACK the window reaches | 30 s | `CERULION_FLASHBACK_WINDOW_MS` |
| How far FORWARD a capture runs after the trigger | 15 s | none |
| Where captures land | `<workspace>/recordings/flashbacks/` | `CERULION_FLASHBACK_DIR` |
| How much disk captures may hold, in total | 2 GiB | `CERULION_FLASHBACK_MAX_MB` |
| How many captures are kept | 20 | `CERULION_FLASHBACK_MAX_CAPTURES` |
| How many captures may be taken per hour | 20 | `CERULION_FLASHBACK_MAX_PER_HOUR` |

Retention is a dashcam contract: oldest captures rotate away when either cap is
reached, and `--pin` exempts one from that.

The 30 s back-window is not an arbitrary round number: it is the 15 s forward
window plus the 15 s anchor cadence, because a capture covering `[T−15s, T+15s]`
needs a checkpoint at or before its own start to be re-runnable from there.
Shortening it below that would produce captures anchored *after* the thing that
went wrong.

---

## What a capture says about itself

Open one with `cerulion bag info`. Beyond the frames, a capture carries:

| Attachment | What it holds |
|---|---|
| `__cerulion/flashback.json` | What the capture was ABOUT: `causes[]` (each with the `detail` your `--note` became), the window it claimed (`span_ms`) against the window it achieved (`achieved_span_ms`), a `handoff` object saying what this recorder was given, and `anchor.resimmable`, whether `bag play --resim` will accept this bag. |
| `__cerulion/state_coverage.json` | The node-state side: which nodes have an anchor, and which were skipped and why. Absent when the recorder had nothing to say about the state plane. |
| `graph.yaml`, `env.json` | The graph as executed and the environment it ran in. Bare names, not `__cerulion/`-prefixed: that namespace is reserved for manifests the recorder mints itself. |
| `__cerulion/recorder.json` | The host that wrote the bag, so a reader can warn about cross-architecture float skew. |
| `__cerulion/trace_manifest_rank<N>.json` | Which node ids the rank's trace records refer to. `bag play --resim` needs it to resolve a FIRE to a node. |

`anchor.resimmable` is the field to read first:

```bash
cerulion bag info recordings/flashbacks/<capture>.mcap
```

It is a plain `true`/`false`, always present. (The three-valued one is `cerulion flashback`'s own verdict LINE, which
can say UNKNOWN when the capture's outcome never reached it, and never a silent
"no".) When it is false, `anchor.resimmable_reason` says why in one sentence,
the same field the verb's not-resimmable line points at. For a missing TRACE
specifically, `handoff.trace` adds which absence it was
in its own words rather than leaving you to guess: for example, that the run's
trace rings held nothing yet because its nodes had not fired inside the window,
or that the capture's trace retention hit its byte ceiling
(`CERULION_FLASHBACK_TRACE_MAX_MB`, 64 MiB).

---

## Watching a live run from outside

A Flashback capture is bounded by the rolling window. If you want a *full*
recording of a run that is already going (no window, no bound), attach a
recorder to it:

```bash
cerulion bag record --run=<RUN>      # attach to a named live run
cerulion bag record --run=           # attach to the sole live run (keep the `=`)
```

The recording begins where it attaches. Frames and scheduler trace start at the
attach point, every channel is marked `attached_late`, and the bag's
`__cerulion/run.json` records `attached_mid_run`. Nothing before the attach is
recoverable, and nothing in the bag implies otherwise.

> **`--run` needs its `=`.** `--run=demo` names a run; a bare `--run` attaches
> to the sole live run and every following word stays a topic name. Written
> `--run demo`, the word `demo` is a *topic*, not a run.

The recorder copies the run's effective graph, environment and identity, and
attaches its available trace rings. A missing or unreadable artifact is reported
rather than treated as present.

A resim beginning mid-run also needs compatible checkpoint anchors. When the
run reports a standing Flashback recorder, the attach leaves its checkpoint
rings alone: those rings admit one reader, and a second would interfere with
the standing recorder. The bag records that choice under `state_rings` in
`__cerulion/run.json`. It remains useful for playback and inspection; for an
anchored incident replay, use the standing recorder's Flashback capture.

When the run reports no standing consumer, the attach discovers available state
rings. It does not arm checkpoints itself, so discovery alone does not promise
a usable anchor. `bag info` reports the captured state coverage and
`bag play --resim all --verify` checks the actual trace and resume boundary.
Mid-run free-run recordings require independent per-rank resume support, which
is not supported; recordings beginning at step zero do not need a resume
anchor.

---

## The run's own account: `run.json`

Every `graph run` writes a run directory under `~/.cerulion/runs/`, and its
`run.json` states what the run did about trace rings under the `trace_rings`
key:

| `trace_rings` | Means |
|---|---|
| `declared` | The run created rings; `rings` names them. A rank whose ring was declared but never created is listed separately under `declared_unavailable`, so "declared" is never read as "all present". |
| `declined: <reason>` | The run declined rings at launch, BY CHOICE. |
| `unavailable: <reason>` | The run wanted rings and could not have them, BY REFUSAL. |
| *(key absent)* | UNKNOWN. A build older than the key, a run shape that mints none, or a declaration that was never written. A reader must not turn this into a claim. |

A second key, `state_ring_consumer`, answers the other question: whether
anything is already draining the run's per-rank checkpoint rings:

| `state_ring_consumer` | Means |
|---|---|
| `standing` | A recorder this run started holds the run's capture-plane tag, so it is draining those rings. A mid-run `bag record --run` declines them. It says what the run REPORTED at launch; nothing un-declares it if that recorder later stops. |
| `none: <reason>` | Nothing this run started is draining them, in the run's own words: it declined its rings, the capture plane is switched off, its recorder could not be started, or its plane armed no ring. A mid-run attach sweeps them. |
| *(key absent)* | UNKNOWN. A build older than the key, a run shape that declares none, a run that has not yet reached the decision, or a declaration that was never written. An attach proceeds (declining on no evidence would cost it anchors for a consumer nobody observed) and says that is what it did. |

The two degraded values carry the run's own reason for the rings being
unavailable, because a bag or a capture is read long after the command line
that produced it has scrolled away. A later recording command cannot add
execution context the run never held.

---

## Turning it off

There are two switches, and they are **orthogonal**: they turn off different
things, and neither is a spelling of the other.

| | `CERULION_FLASHBACK=off` | `cerulion graph run --no-rings` |
|---|---|---|
| Scope | The whole capture plane, for that run | That run's scheduler-trace rings |
| Turns off | The rolling window recorder, the per-rank node-STATE rings, and the anchors a capture resumes from | The per-rank scheduler-trace rings (~40 MiB apparent each) and the supervisor's departure ring, **and, with them, the window recorder** |
| Still runs | The trace rings, so a later `cerulion bag record --run` still gets a scheduler trace, and `--record` is unaffected | Nothing of the capture plane's window: with no trace rings, nothing captured could be re-executed, so the run takes **no captures** rather than frames-only ones |
| You lose | Every capture and every mid-run anchor | Every capture of that run |
| Saved | The state rings and the window's own memory | ~40 MiB per rank, converging as the rings fill |
| Bad value | The plane stays ON and warns (loud over silent) | n/a |

`--no-rings` **conflicts with `--record`** at parse time, because a graph
recording carries the scheduler trace that makes it re-executable.

On the shapes that mint no ring anyway it is **not** a no-op, and the launch
line says which it is. On `--single-process`, `ros2 attach`, `node run` and
`--time-source external` it still stops the window recorder, so the run takes
no captures. Two shapes are true no-ops. A `--time-source virtual` MONOLITH
starts no recorder either way (though a `virtual` run of a `process_groups:`
graph still goes multi-process, and the supervisor's recorder gate has no clock
term, so there the flag bites like anywhere else). And on a NON-UNIX build the
recorder and the run directory are compiled out and `cerulion flashback` refuses
outright, so there is nothing for the flag to stop.

`--no-rings` is not `--trace-limit`. `--trace-limit` caps the **in-memory**
fire-trace deque that `cerulion graph` introspection reads. These are the
**shared-memory** rings a recorder drains.

Setting both switches gives you a graph run with no black box at all.

---

## What it costs

Per rank, a scheduler-trace ring is **~40.06 MiB apparent** (`65_600 + 2^20 × 40`
= 42,008,640 bytes), plus one 106,560-byte departure ring per deployment. The
segment is `ftruncate`d rather than written, so it costs a page at first and
converges on the full figure only as the ring fills; a graph whose ranks fire
slowly may never reach it.

The rolling frame window is separate, and sized from the memory this process may
actually use: that figure divided by 16, floored at 320 MiB and ceilinged at
8 GiB. `CERULION_FLASHBACK_WINDOW_MAX_MB` overrides it.

"May actually use" is the tighter of the machine's own total and the cgroup
ceiling binding the process, so a container sizes its window from its own share
rather than from the host it happens to run on. Every cgroup from the process's
own up to the mount point counts, and on cgroup v2 `memory.high` counts beside
`memory.max`, because the throttle an allocation meets first is the one that
decides. A desk whose own slice carries a `memory.high` is sized from it too.

Anything unreadable is left out rather than guessed at, so a misread can only
leave the window larger, never smaller than the 320 MiB floor. Two ceilings are
deliberately not read: a cgroup **above** the mount point, which binds but has
no readable path from inside a container, and swap, which would only ever raise
the figure. Set `CERULION_FLASHBACK_WINDOW_MAX_MB` when you want the size
decided rather than derived.

The apparent figures above are arithmetic from the ring geometry and are exact.
The rate at which resident memory converges on them is not measured on a
Jetson-class board.

---

## Limitations

**Single-process runs take captures that cannot be re-run.** A monolith shape
(`--single-process`, `cerulion ros2 attach`, `cerulion node run`, or
`--time-source external`) mints no trace ring, deliberately: its gating clock
is wall-driven, so a trace taken there would carry step boundaries a resim
cannot re-advance to, and the capture would claim to be re-runnable when it is
not. Such a capture says, in its own words, that it holds no trace.
(A `--time-source virtual` MONOLITH is different again: it starts
no window recorder, so it takes no captures rather than un-runnable ones. A
partitioned graph under `virtual` still routes to the supervisor and does take
them.) A `--single-process --record` run is the one monolith exception that DOES
create a rank-0 ring: its recorder is handed both the ring and the window.

**A capture taken on a machine with other live topics may be refused by
`--resim`.** A capture records what is LIVE, and it does not
carry a coverage manifest saying which of those topics belonged to the run's own
graph, so a co-tenant topic looks to replay like a bag that disagrees with its
own graph, and the resim exits 2 blaming corruption.

**A topic that first publishes mid-recording gets no channel.** A
bag's channel set is fixed when the bag is created, so a producer that appears
later is named in `record_coverage.json` as `appeared_after_bag_creation` and is
not recorded. This bites hardest on a cold ROS bridge, whose routes appear
seconds into the run.

---

## See also

- [`docs/bag.md`](bag.md): the bag format, `bag record` / `play` / `info`, and
  what a bag's coverage and health documents mean.
- [`docs/multi_process.md`](multi_process.md): `--no-rings` in the full
  `graph run` flag table, and why the multi-process shape is where rings live.
- [`docs/user-api.md`](user-api.md): the single ground-truth reference for the
  whole user-facing surface.
