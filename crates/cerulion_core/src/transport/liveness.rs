// SPDX-License-Identifier: AGPL-3.0-only
//! Per-topic **data-flow** liveness — did frames actually CROSS this
//! topic, as opposed to "does a publisher exist for it".
//!
//! # Why publisher-existence is not liveness
//!
//! The desk sidebar's per-row affordance was first built on
//! [`TransportManager::topic_publisher_count_checked`](super::TransportManager::topic_publisher_count_checked)
//! — a cheap `.open()` + `number_of_publishers()` read. On a graph whose
//! producers are created lazily that is a decent proxy, but on a
//! `cerulion ros2 attach` robot the bridge graph creates a Cerulion publisher for
//! EVERY discovered DDS topic at graph-BUILD time, whether or not that DDS route
//! ever carries a sample. The measured consequence (Go2, 75 topics): every topic
//! reports `producer_count: 1`, so the dead route `/uslam/cloud_map` is
//! indistinguishable from the genuinely-streaming `/utlidar/cloud_deskewed` and
//! the whole affordance is INERT on the exact robot class it is built for.
//!
//! The divergence is not ROS-specific: a native Cerulion node with a
//! [`#[output]`](cerulion_macros) port it never writes (lazy loan) also
//! has a publisher and no frames. So the signal has to be an OBSERVATION of
//! frames, not a count of ports.
//!
//! # Why a frame observation must be CONTINUOUS, not sampled at serve time
//!
//! The obvious cheap design — at catalog-serve time attach a listener-less
//! [`DataOnlySubscriber`], drain it, and read the newest sample's wire
//! `timestamp_ns` — CANNOT WORK on iceoryx2, and this is load-bearing enough to
//! state as measured fact rather than intuition. A freshly created subscriber
//! port receives only samples published AFTER its connection to the publisher is
//! ESTABLISHED, and nothing the tap itself does establishes that connection: a
//! tap opened on a topic that has already published four frames and then drained
//! immediately takes **zero** samples, even when the publisher retains a native
//! history and `pump_history` is driven (the late-joiner pump rides the event
//! service — it delivers only on a `SubscriberConnected` event, which a data-only
//! tap deliberately never sends, the invisibility property).
//!
//! So a true data-flow signal requires a tap that was ALREADY attached when
//! the frame was published. That is what [`TopicLivenessObserver`] is: a
//! long-lived, budget-bounded set of listener-less taps, swept on its own
//! throttled cadence off the gateway's existing drive thread, feeding a shared
//! [`LivenessTable`] the catalog serve reads.
//!
//! ## The retained-history caveat (measured, and defended against)
//!
//! The tap is invisible, but it is not unreachable. Once the publisher runs
//! `update_connections()` — which its own next `send()` does implicitly, and
//! which `deliver_history` does when ANY OTHER (listener-full) subscriber
//! connects — iceoryx2 establishes the pending connection and pushes the
//! publisher's RETAINED HISTORY into it. Measured against real iceoryx2 on a
//! `history_size = 4` topic with four frames published before the tap attached,
//! **with a tap deep enough to hold the whole flush** (the counts below are what
//! the publisher DELIVERS; the shipped observer tap is only
//! [`LIVENESS_TAP_BUFFER_SIZE`]-deep, so it retains the newest
//! `LIVENESS_TAP_BUFFER_SIZE` of them — the excess is reclaimed, exactly as for
//! any other burst deeper than the tap, and only the COUNT is clipped, never the
//! age):
//!
//! | Stimulus after the tap attached | Frames the publisher delivered |
//! |---|---|
//! | `pump_history()` with no other subscriber | **0** |
//! | one further `publish_raw` (no other subscriber, ever) | **5** (4 history + 1 new) |
//! | a listener-full subscriber connects, then `pump_history()` | **4** (the history) |
//!
//! Left alone that is a FALSE-LIVE hazard, and precisely on the row this feature
//! exists to dim: a dead-but-history-carrying route would deliver its stale
//! backlog the moment `bagd` / `topic echo` / vizd attached, and the observer
//! would stamp `last_frame_at_ns = now` and render it Streaming, then
//! [`LivenessState::Idle`] forever.
//!
//! ## The defence: STAMP ADVANCEMENT (publisher clock vs publisher clock)
//!
//! The defence is in [`LivenessRecord`], and its comparison is between two
//! publisher stamps — NEVER between a publisher stamp and the observer's clock.
//! Every drained frame carries the publisher's loan-time
//! [`WireHeader::timestamp_ns`](crate::wire::WireHeader), and the record keeps
//! the HIGHEST stamp it has ever seen on that topic (`max_stamp_ns`, private to
//! [`LivenessRecord`]). A batch DATES the topic iff its newest
//! stamp EXCEEDS that recorded maximum, because that proves the publisher
//! committed a frame BETWEEN the two drains — i.e. while the robot was already
//! watching. Three consequences, all load-bearing:
//!
//! * The FIRST batch after a tap attaches advances nothing — it ESTABLISHES the
//!   baseline. It may be the flushed backlog and nothing in it can be proven
//!   otherwise, so its frames are banked into
//!   [`TopicLiveness::frames_observed`] and the topic is NOT dated.
//! * A re-flush of the SAME backlog into a later connection (every hand-off and
//!   re-attach can trigger one) carries the same stamps, so it can never advance
//!   the baseline. The dead-but-history-carrying route stays undated however many
//!   times its history is flushed at us, and however that flush is CHUNKED — that
//!   is what the epoch reset below is guarded to preserve. An early chunk of a
//!   re-flush is a strict PREFIX of the backlog and therefore REGRESSES, and
//!   adopting its stamp would LOWER the bar for the chunks behind it, so a
//!   regression is allowed to lower the maximum only when it is provably NOT part
//!   of a burst: the baseline must be CLOSED **and** the burst reading must be
//!   excluded — by [`REGRESSION_RESET_MIN_GAP_NS`] of observer silence, or
//!   by a regression regime a burst cannot produce. Within a burst the
//!   maximum is monotone unconditionally; across bursts those two readings are
//!   what separate a restart from a chunk (see the restart sections). NOTE the
//!   chunk being described is a strict PREFIX: the backlog's own TAIL is stamped
//!   EQUAL to the carried maximum, not below it.
//! * A flush split across two drains cannot advance against its own first chunk,
//!   because the baseline stays OPEN until a drain leaves the subscriber queue
//!   EMPTY (the drainer reports that as
//!   [`DrainObservation::queue_emptied`]). Without that, the second chunk of one flush
//!   would out-rank the first and date a dead route.
//!
//! ### The one break in "one topic, one clock": a publisher RESTART
//!
//! The soundness premise above — successive stamps on one topic come from ONE
//! clock — holds for the LIFE of one publisher, and a publisher does not live
//! forever. The OBSERVER outlives it: on the shipping deployment it runs inside
//! the embedded gateway of the per-computer `cerulion-netd` daemon, which
//! survives any number of `cerulion graph run` invocations (the gateway's own
//! egress drain names "a producer worker restart re-creating its service" as its
//! canonical failure cause, so the codebase already expects exactly this). A
//! restarted worker's [`VirtualClock`](crate::clock::VirtualClock) starts at 0
//! again, so its first frames are stamped FAR BELOW the dead run's final stamp —
//! and a maximum that only ever RISES would then refuse to date the topic for as
//! long as the previous run lasted. Two hours of uptime followed by a restart
//! would render an actively-streaming 20 Hz topic `Idle` for another two hours:
//! the exact inversion this feature exists to prevent, and precisely the state an
//! operator restarts to escape.
//!
//! The single-writer contract that makes the maximum sound also makes the break
//! DETECTABLE. Within one publisher run the stamps on one topic are monotonically
//! non-decreasing (they are loan-time reads of ONE clock on ONE writer), so a
//! batch whose newest stamp lands BELOW the recorded maximum cannot have come
//! from the run that set it. That is structural evidence of a NEW clock epoch,
//! and `LivenessRecord::record_frames` (private) treats it as one: it ADOPTS the
//! regressed batch's stamp as the new maximum and RE-OPENS the baseline. The
//! regressed batch itself never dates the topic (it may be the new run's own
//! retained-history flush, which the baseline exists to absorb), and the next
//! batch that advances past it dates normally — so a restart the reset FIRES on
//! recovers within ONE advancement rather than never. WHEN it fires is the guards'
//! business (next two sections): on a continuously-streaming restarted topic the
//! SILENCE gate alone can be never, which is why there is a second, additive
//! reading that bounds it.
//!
//! ### Telling a restart apart from a re-flush chunk
//!
//! Stamps alone cannot do it. A restarted run's frames and a re-flushed backlog's
//! later chunks are both batches BELOW the recorded maximum, and both can ascend
//! among themselves. So the reset is gated on TWO facts, and a regression must
//! clear both; the second now accepts two readings rather than one
//! (either suffices; see the next section):
//!
//! 1. **The baseline is CLOSED.** A re-flush is triggered by connection
//!    establishment, so it arrives with the baseline OPEN
//!    (`begin_attached_observation` has just run). Its regressed early chunks are
//!    therefore BANKED without adopting.
//! 2. **The observer has been silent for [`REGRESSION_RESET_MIN_GAP_NS`]** — no
//!    drain has yielded a single frame for two sweep intervals. Guard 1 alone is
//!    NOT enough, because a burst's first chunk can leave the queue momentarily
//!    empty while `deliver_history` is still pushing (the mid-flush interleave in
//!    the residual list), which CLOSES the baseline mid-burst and hands the next
//!    chunk of that same backlog a closed one. Chunks of one burst are separated
//!    by at most one drain cadence; a restart is a process lifecycle event and
//!    shows up as silence orders of magnitude longer.
//!
//! A genuine restart under a continuously-attached tap — the shipping shape, since
//! the observer's own tap holds the iceoryx2 service open across the worker
//! swap — arrives with the baseline closed AND after the boot silence, and resets
//! through those two guards alone.
//!
//! ### The SECOND reset path, for the restart with no lull
//!
//! Guard 2 asks a question a whole class of topics can never answer YES to, and
//! it is exactly the class this feature exists to describe. Clearing it needs a
//! HOLE in the DRAINED stream, so a topic streaming faster than its plane's
//! boundary (roughly 5 Hz on the observer path, ~2.5 Hz on the demand path — the
//! band arithmetic is in [`REGRESSION_RESET_MIN_GAP_NS`]) re-anchors the gate on
//! every drain and NEVER clears it. A publisher that restarts without a
//! drain-visible pause — systemd bringing a crashed node back in ~100 ms, a
//! restart coinciding with a tap re-attach, a batch that spans the swap — then
//! leaves the topic pinned to the DEAD run's `last_frame_at_ns` for as long as it
//! keeps streaming: a row reading `Idle` with an age that grows a second every
//! second, which is an affirmatively WRONG freshness claim rather than an absence
//! of information. With guard 2 alone, low-rate topics heal and high-rate ones
//! cannot. That priority inversion is what the second reset path removes.
//!
//! Raising or lowering [`REGRESSION_RESET_MIN_GAP_NS`] cannot remove it (the anchor
//! moves with the threshold), so the remedy is an ADDITIONAL, independent way for a
//! banked regression to clear — one that asks about the regression regime itself
//! rather than about the silence in front of it. The silence gate is UNCHANGED
//! and still heals what it heals, faster than the second path can:
//!
//! ```text
//! regressed && !baseline_open && (quiet_long_enough || sustained)
//! ```
//!
//! `sustained` is read off a PENDING-EPOCH run: the consecutive frame-yielding
//! batches that have stayed strictly BELOW the recorded maximum while the
//! baseline was closed. Four conditions, each closing one way a retained-history
//! burst could otherwise fake a new epoch, and all four required:
//!
//! 1. **Every drain in the run saw exactly ONE publisher**
//!    ([`DrainObservation::writers_seen`] `== Some(1)`) — the same evidence
//!    the rate estimate's sequence basis is gated on, read on both planes and already
//!    carried. `None` — a drainer that did not ask — is treated like a
//!    multi-writer answer: absence of evidence is not evidence of a single writer.
//!    EVERY drain means every drain, the run's ANCHOR included: a batch failing
//!    this test contributes nothing at all — not the timestamp the span is measured
//!    from, not the ascent bar, not an emptied credit — and the next clean batch
//!    anchors a fresh run.
//!
//!    **State this one at its real strength, because the quantity is not the one
//!    the name suggests.** `writers_seen` is `DataOnlySubscriber::publisher_count`
//!    — how many LOCAL iceoryx2 publisher PORTS exist on the service at the
//!    instant of the drain. What the design wants is "these stamps come from one
//!    clock". On the shipping paths the two coincide, and that is worth writing
//!    down rather than assuming: a graph topic is provisioned `max_publishers = 1`
//!    (one port, one worker, one clock), and a `cerulion ros2 attach` route MINTS
//!    its own header from the BRIDGE's clock (`publish_cdr_body(.., now_ns())`),
//!    so a robot's `/tf` with a dozen upstream ROS writers still carries ONE clock
//!    and never regresses at all. Where they come apart:
//!
//!    * A topic with two LIVE local writers (`multi_publisher_topics`) reports
//!      `>= 2` on essentially every drain, so every batch RESTARTS the run and none
//!      ever lives long enough to confirm — excluded, as intended. But the gate is
//!      PER DRAIN, so during a window in which one of its writers is detached
//!      (restarting, not yet attached, a short-lived node) it reports `Some(1)` and
//!      IS admitted. Such a topic can therefore take this path in exactly the
//!      window its stamps are most likely to regress. The cost is bounded the way
//!      the observer already bounds reset churn on that class (`Idle` rather than
//!      `Streaming`, never a dim, never a `Streaming` unbacked by a dated frame),
//!      but "byte-unchanged" would be an over-claim and is not made.
//!    * A RE-INJECTED mirror (`publish_raw` writes the caller's header verbatim)
//!      is ONE local publisher carrying whatever clock the origin stamped. That
//!      shape is not reachable on the shipping desk today — the gateway observer
//!      tracks `bridge_manager().registered_topics()`, which is the EGRESS set,
//!      and a desk mirror is an INGRESS registration — but it is the shape to
//!      re-check if the observer is ever pointed at ingress topics.
//! 2. **At least [`SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS`] of the run's drains
//!    left the queue EMPTY** ([`DrainObservation::queue_emptied`]). On a
//!    BUDGET-LIMITED drainer this is a structural burst-killer: a prefix chunk
//!    leaves the queue non-empty BY CONSTRUCTION (the rest of the burst is still
//!    in it), so the only way a burst supplies an emptied sub-max drain is the
//!    documented mid-flush interleave, and it must do so twice.
//!
//!    **It is INERT on the observer plane, and that is stated rather than
//!    implied.** [`TopicLivenessObserver::sweep`]'s own drain loops to a SHORT
//!    READ, so every frame-yielding drain there reports `queue_emptied` and this
//!    condition degenerates to "the run has >= 2 drains". Condition 3 is the whole
//!    defence on that plane. The two conditions are therefore COMPLEMENTARY rather
//!    than redundant, and the joint argument is the one below.
//! 3. **The run has spanned [`SUSTAINED_REGRESSION_MIN_SPAN_NS`] of OBSERVER
//!    time.** The span is what makes the healing bound uniform across two planes
//!    whose drain cadences differ by three orders of magnitude — and on the
//!    observer plane it is the ONLY thing standing between a burst and a reset.
//!
//!    What makes that sufficient is a PHYSICAL bound, named here so a future edit
//!    cannot argue the span down without noticing what it carries: a burst is ONE
//!    `update_connections` push and `deliver_history` is synchronous, so it
//!    completes in microseconds-to-milliseconds. Producing an ascending, sub-max,
//!    queue-emptying drain on drain after drain for a WHOLE SECOND would require
//!    that push to stay in flight across the entire span. There is also an
//!    ANTI-CORRELATION: a drainer fast enough to be raced into repeated short
//!    reads mid-push is fast enough to finish the burst in milliseconds, so the
//!    same burst cannot also span the second.
//! 4. **The run's stamps ASCENDED** — this batch's stamp exceeds the run's
//!    first. A publisher whose stamps stand still is not producing, and a run
//!    that has not moved is not evidence of a live new epoch.
//!
//! A run is CLEARED by any frame-yielding batch that is not strictly below the
//! maximum (an advance, an EQUAL stamp, or an unstamped batch), by an OPEN
//! baseline (while the baseline is open we are provably absorbing a burst, so
//! nothing observed there is evidence), by a fresh attach, and by an epoch reset
//! of either kind.
//!
//! #### The equality boundary, and why a re-flush terminates its own run
//!
//! The condition that matters most is the one that is easy to get wrong. The
//! issue that filed this proposed "N consecutive batches stay BELOW the held
//! maximum", and a re-flush's chunks DO stay below it — until the last one. The
//! carried maximum `M` is the highest stamp ever seen, and a single-writer
//! publisher's retained history keeps its NEWEST frames, so the frame that SET
//! `M` is in the backlog and the backlog's newest stamp `B` EQUALS `M`. A
//! re-flush's tail chunk therefore carries `ts == M`, which the strict
//! comparisons make neither an advance nor a regression — so it CLEARS the run
//! rather than extending it. (The retention DEPTH is irrelevant to this: the tap
//! is `drop_oldest`, so whatever it holds ENDS at the newest frame pushed,
//! whether that is one frame or a thousand.)
//!
//! **SCOPE, because the argument is conditional and reads as absolute:** it holds
//! while `M` belongs to the run whose history is being flushed. Across an
//! un-reset epoch boundary it does not — a worker that restarts, publishes for
//! 300 ms and dies leaves `M` on the DEAD run's clock, so the new run's flush tail
//! is strictly BELOW `M` and regresses like any prefix. That shape is still
//! refused, by the OTHER two guards rather than by this one: the flush arrives
//! with the baseline OPEN (the re-attach ran `begin_attached_observation`), and
//! even if a mid-flush interleave closes it, one `update_connections` burst cannot
//! span [`SUSTAINED_REGRESSION_MIN_SPAN_NS`]. A re-flush can only ever contribute a strict PREFIX
//! of its backlog to a run, and that is why condition 2 bites: on a
//! budget-limited drainer a prefix chunk leaves the queue NON-empty by
//! construction (the rest of the burst is still in it). The one shape that
//! produces an emptied prefix chunk is the documented microsecond-scale
//! mid-flush interleave, and it can produce ONE — which is why the threshold is
//! TWO.
//!
//! #### Per-plane semantics
//!
//! The conditions are deliberately a COUNT plus a SPAN rather than "N sweeps",
//! because the two planes have no common cadence:
//!
//! * **OBSERVER plane** — the topic's own tap on the
//!   [`LIVENESS_SWEEP_INTERVAL_NS`] grid, drained TO EMPTY, so every
//!   frame-yielding drain reports `queue_emptied`. A restarted topic publishing
//!   at least once per sweep satisfies condition 2 on its second sweep and is
//!   then bound by condition 3.
//! * **DEMAND/egress plane** — the gateway's ~1 kHz drive loop, where
//!   `queue_emptied` comes from the non-consuming
//!   [`DataOnlySubscriber::has_samples`] query after a saturated read. A topic
//!   publishing slower than that loop empties its queue on essentially every
//!   frame-yielding pass, so condition 2 is met within two publish periods and
//!   condition 3 governs there too.
//!
//! So on BOTH planes a single-writer restarted topic that keeps streaming
//! recovers a dated reading within roughly
//! [`SUSTAINED_REGRESSION_MIN_SPAN_NS`] plus the one further advancement the
//! reset's re-opened baseline costs — bounded rather than unbounded, and
//! shortest on the fast topics the silence gate alone can never heal.
//!
//! #### What it does NOT fix
//!
//! Stated here rather than left to be discovered: the new path is inert wherever
//! its evidence cannot be gathered.
//!
//! * A topic with two LIVE local writers is excluded by condition 1 BY DESIGN; a
//!   transient single-writer window is the exception, stated in that condition.
//! * **A topic whose producer outruns its drainer so consistently that no drain
//!   ever empties the queue never satisfies condition 2 — and the tempting
//!   dismissal of that case is FALSE.** "Such a topic never closed its
//!   advancement baseline either, so it was already undatable" only holds when
//!   the saturation is present FROM ATTACH. `queue_emptied` is consulted in
//!   exactly two places — to CLOSE the baseline, and to count toward condition 2 —
//!   so once the baseline is closed, dating runs purely on stamp advancement and
//!   emptiness is irrelevant. A topic that attached while its robot was idle
//!   (baseline closes on the first short read), dated perfectly well for hours,
//!   and only LATER outran its drainer is not "already undatable" in any sense;
//!   it is fully datable right up to the moment its worker restarts, and from
//!   then on neither reset path can reach it. The accurate statement is the one
//!   without the excuse: the sustained path needs an emptying drain, so a topic
//!   whose drainer is saturated AT RESTART TIME cannot heal — whether or not it
//!   was datable before the saturation began.
//! * The reset still costs the confirmation window: for that window the row keeps
//!   serving the dead run's growing age, as it does under the silence gate
//!   alone, just not forever.
//!
//! ## Why the comparison must not involve the observer's clock
//!
//! This is not fastidiousness — an attach-instant comparison is BROKEN on the
//! shipping deployment. On the default multi-process `graph run` the
//! publisher lives in a `graph run-worker` process whose
//! [`VirtualClock`](crate::clock::VirtualClock) starts at 0 and advances by the
//! handed logical quantum, while the observer runs inside the separate
//! network GATEWAY process on a [`RealClock`](crate::clock::RealClock)
//! reading nanoseconds since BOOT. Those are two unrelated number lines, and
//! different workers each own an INDEPENDENT `VirtualClock`. "Is this frame
//! stamped after the instant my tap attached" therefore has no meaning: on a
//! robot that has been up a while every frame looks pre-attach (and the gate
//! degrades to a blanket discard), while on a worker stepping faster than wall
//! time every frame looks post-attach (and the gate is inert, re-opening the
//! false-live hazard). Stamp-vs-stamp is immune to all of it: the single-writer
//! contract means one publisher owns a graph topic, so successive stamps on one
//! topic come from ONE clock whatever kind it is.
//!
//! The observer's clock is still used — but only for observer-clock arithmetic:
//! `last_frame_at_ns` (when we DRAINED the advancing batch) and
//! `observed_for_ms` are both read and rendered on that one clock.
//!
//! ## What the undated-but-produced state means
//!
//! `frames_observed > 0 && last_frame_age_ms == None` means "this topic HAS
//! produced data and the robot cannot date any of it": the flushed backlog of a
//! dead route, a latched `/tf_static` that published exactly once, or a topic
//! whose only observed batch so far was its baseline. That is
//! [`LivenessState::Idle`] with no age — a produced topic is NEVER called dead.
//! [`LivenessState::NoData`] is reserved for `frames_observed == 0`: watched long
//! enough and nothing arrived at all, not even a flush.
//!
//! Be clear about what that payload does NOT carry: it is ONE-WAY (a topic that
//! has banked a frame can never report [`LivenessState::NoData`] again, however
//! long it is subsequently watched in silence) and it carries NO AGEING SIGNAL
//! (nothing records WHEN the undatable batch arrived, so a backlog flushed three
//! hours ago and one flushed 200 ms ago serve the identical
//! `{last_frame_age_ms: null, observed_for_ms: N, frames_observed: k}`). That is
//! deliberate — the robot cannot date those frames, and inventing an age from its
//! own clock is the very cross-clock subtraction the Clocks section forbids — but
//! it means a reader must treat the class as "has data, freshness unknown", never
//! as a staleness measurement.
//!
//! # The RATE, and why it is a different measurement
//!
//! The same observation also answers "how FAST" ([`TopicRateEstimate`]), which is
//! what lets the desk show a frequency for a topic NOBODY HAS CHECKED. Before it,
//! a rate existed only on the demand plane — checking a topic opened a mirror and
//! `cerulion-vizd` computed Hz from the frames crossing it — so an unchecked row
//! could say "live" and nothing more.
//!
//! It shares this module's clock discipline but NOT its numerator. Dating asks
//! whether the publisher committed anything while we watched, and answers it by
//! comparing publisher stamps to publisher stamps. The rate asks HOW MANY, and
//! answers it with the publisher's own commit SEQUENCE — a COUNT, which belongs
//! to no clock and may therefore be divided by an observer-clock duration without
//! comparing two clocks. The publisher's stamps could NOT serve as that
//! denominator: a `graph run` worker's gating clock advances by a fixed LOGICAL
//! quantum per step and its seconds are not wall seconds.
//!
//! Counting sequences rather than frames is what makes the number worth showing.
//! [`TopicLiveness::frames_observed`] is clipped by the tap depth, so a
//! frames-based rate would pin every topic above [`RATE_FLOOR_BASIS_CEILING_MHZ`]
//! (10 Hz at the shipped constants) at exactly that value — useless for the
//! 500 Hz control and sensor routes this exists to describe. Two frames a hundred
//! sequences apart say a hundred frames were committed, however many were
//! reclaimed. Where the sequence cannot be counted on, the window falls back to
//! that clipped count and LABELS itself a floor
//! ([`TopicRateEstimate::is_floor`]).
//!
//! The rate is served only for a topic that is [`LivenessState::Streaming`] with
//! a window closed inside [`RATE_ESTIMATE_MAX_AGE_NS`], so a stopped stream drops
//! its number rather than decaying one. That same horizon also bounds the window
//! itself: a window is a MINIMUM span with no maximum, and nothing advances it
//! while a topic is quiet, so a topic that PAUSES and resumes would otherwise
//! close one window covering the whole pause and serve its average (measured: a
//! 100 Hz topic silent for a minute reported 0.33 Hz on resuming). A window
//! longer than the horizon provably spans a stretch during which the topic was
//! not streaming, so it is DISCARDED rather than closed and the resuming drain
//! anchors a fresh one.
//!
//! Its residuals are separate from the dating ones below, and are these:
//!
//! * **A publisher that stamps successive frames identically gets no rate**, even
//!   when its sequence is advancing perfectly. Such a topic is never DATED (the
//!   residual list below says so), so it never classifies `Streaming`, and the
//!   gate withholds the number rather than render an `Idle` row carrying a
//!   confident frequency. The cost is one class of misbehaving publisher losing a
//!   rate it could technically have had.
//! * **A `multi_publisher_topics` topic gets a labelled FLOOR, never a
//!   measurement.** `sequence` is a PER-PUBLISHER commit counter and the drain
//!   reports the MAX across a batch, so with two writers that maximum hops
//!   between unrelated counters and its delta is not a frame count. The
//!   regression guard does NOT cover this: a FORWARD hop is arithmetically
//!   indistinguishable from a fast single writer whose frames the shallow tap
//!   clipped, and no plausibility bound separates them. Measured without
//!   the evidence gate, both served as confident `is_floor: false` values: a
//!   ~101 Hz two-writer `/tf` reported **1550 Hz**, and a long-uptime shape
//!   **25,000,005 Hz**. So the basis is gated on single-writer EVIDENCE
//!   ([`DrainObservation::writers_seen`], a dynamic-config read on the tap's own
//!   service handle): anything other than exactly one publisher — including an
//!   UNKNOWN count — closes the window on the FLOOR basis, a true lower bound
//!   from the frames actually drained. The bit is re-armed per window, so a
//!   topic recovers its exact basis as soon as a window runs clean.
//!
//!   The evidence is SAMPLED, once per drain, so state the guarantee at its real
//!   strength: a writer that both attaches and detaches BETWEEN two drains is
//!   never counted. That window is not reachable in a shipping shape, for a
//!   reason worth writing down rather than asserting. Such a writer's frames
//!   cannot raise the batch MAX at all unless its sequence is CALLER-SUPPLIED —
//!   `publish_raw`, i.e. the netd/gateway mirror and the rmw serialized paths,
//!   the only producers that do not stamp their own commit counter — and those
//!   arrive by a RE-ATTACH, which re-opens the advancement baseline and
//!   re-anchors the rate window anyway. So the residual is bounded at one wrong
//!   window, and no shipping configuration produces even that.
//! * **A topic slower than [`RATE_ESTIMATE_MAX_AGE_NS`] gets no rate at all**,
//!   because every window it could close is longer than the discard horizon.
//!   That is the price of refusing to average across a silence, and the cost is
//!   REAL rather than free — say so, because the obvious excuse is
//!   arithmetically backwards in exactly the band the cost lands in. The serve
//!   gate and the horizon are the SAME 5 s, so a topic of period `P` reads
//!   [`LivenessState::Streaming`] for `min(5s, P) / P` of its own period: 96 % at
//!   `P = 5.2 s`, 83 % at `P = 6 s`. Those topics look live nearly all the time
//!   and carry NO rate. Only above `P = 10 s` does "not Streaming for most of its
//!   period" become true (50 % and falling), so the genuinely uncomfortable band
//!   is `5 s < P < 10 s` — slow telemetry and heartbeats, a class a robot really
//!   has. Widening the horizon does not fix it: the horizon is what keeps a pause
//!   out of the average, so the two costs trade directly.
//! * **The denominator's endpoints are DRAIN instants, not publish instants**, so
//!   the number carries an error even though its count is exact: each endpoint is
//!   the instant we DRAINED the newest frame, which trails the instant the
//!   publisher committed it. What survives is the CHANGE in that lag across one
//!   window — consecutive windows share their boundary drain, so a steady lag
//!   cancels — and in the steady cadence that dominates it is near zero. No single
//!   percentage is quoted: the lag is bounded by the DRAIN cadence, which is the
//!   sweep interval on the observer plane but the publish period on the demand
//!   plane, and the window length varies between its minimum and the discard
//!   horizon, so the ratio has no one denominator.
//!   It is an ESTIMATE and named as one: comparing it digit-for-digit against
//!   `cerulion topic hz` compares two windows, and a disagreement in the last
//!   digit is not a bug in either.
//!
//! ## The residual, stated rather than hidden
//!
//! * **A mid-flush interleave can date a dead route once, on a topic's FIRST
//!   flush.** `queue_emptied` is observed from a drain that left the queue empty,
//!   so if the drain happens to empty it WHILE the publisher's `deliver_history`
//!   is still pushing (a microsecond-scale overlap against a 200 ms sweep
//!   cadence), the baseline closes early and the flush's tail — stamped ABOVE the
//!   chunk already drained — ADVANCES past it. Closing this needs a flush marker
//!   on the wire, which is a publisher-side change this feature does not justify.
//!
//!   The scope is exactly one flush per topic, because the interleave is only
//!   dangerous while the maximum is still being established. On every LATER flush
//!   of that same backlog (the re-flush every hand-off and re-attach triggers) the
//!   carried maximum already EQUALS the top of the backlog — the frame that SET it
//!   is in the retained history — so the tail chunk is EQUAL: it neither advances
//!   (the comparison is strict) nor regresses. The chunks BEFORE the tail are a
//!   strict PREFIX and do regress, and what keeps THEM from lowering the bar is
//!   the reset's guard set: a CLOSED baseline plus either
//!   [`REGRESSION_RESET_MIN_GAP_NS`] of silence or the sustained reading.
//!
//!   That equality is not a detail — it is the property the sustained path's
//!   safety rests on, because an equal batch CLEARS a pending-epoch run and a
//!   backlog therefore terminates its own. The tail does NOT regress: saying
//!   it "REGRESSES" is the opposite claim and would imply a re-flush
//!   EXTENDS a run.
//! * **One burst per attach is never dated.** A topic that publishes exactly
//!   once while observed reads `Idle` with no age rather than `Streaming` — the
//!   evidence for "it published just now" does not exist. Every attach and
//!   hand-off re-opens the baseline, so this cost is paid per attach.
//! * **A publisher that stamps successive frames identically never advances**
//!   (the comparison is strict), so it reads `Idle` with no age. Never a false
//!   live; a real logical clock advances between fires.
//! * **A batch that SPANS a publisher restart delays the epoch reset.** One drain
//!   can pick up the dead run's tail and the new run's head together; its NEWEST
//!   stamp then belongs to whichever epoch stamped higher, so it reads as an
//!   ordinary equal-or-advancing batch and triggers no reset. It also RE-ANCHORS
//!   the silence gate, so the first batch drawn purely from the new run — which
//!   does regress — arrives a drain cadence later and is banked as a re-flush
//!   chunk would be. The reset then lands on the first regression that clears the
//!   silence gate OR on the sustained regime the banked regressions
//!   build — which is the residual in the NEXT bullet, and is what bounds this one.
//!   Never a false live: a spanning batch dates the topic only when the OLD run's
//!   frames genuinely advanced past everything seen — which they genuinely did,
//!   while we were watching.
//! * **A restart the SILENCE GATE alone cannot judge costs a confirmation window
//!   — not an unbounded wait.** Two triggers produce the same
//!   shape. A restart that COINCIDES with a re-attach lands its first batch under
//!   an OPEN baseline, which banks it without adopting; and a restart the observer
//!   sees with NO silence at all (a supervisor restarting a crashed node in
//!   ~100 ms; a batch that SPANS the swap and re-anchors the gate) never puts
//!   [`REGRESSION_RESET_MIN_GAP_NS`] of quiet in front of any regression. Either
//!   way the silence gate then waits for the first [`REGRESSION_RESET_MIN_GAP_NS`]
//!   HOLE in the DRAINED stream, and for a topic in its plane's fast band (roughly
//!   5 Hz on the observer path, ~2.5 Hz on the demand path — the band arithmetic is
//!   in that constant's docs) that hole NEVER arrives while the topic streams.
//!
//!   Before the second reset path that was the whole story and the wait was UNBOUNDED. It is now
//!   bounded by the SUSTAINED path (see the second-reset-path section above): a single-writer
//!   topic whose drains empty their queue recovers within
//!   [`SUSTAINED_REGRESSION_MIN_SPAN_NS`] plus the one further advancement the
//!   reset's re-opened baseline costs. What remains is that window rather than
//!   forever, and it is shortest on the fast topics that could never heal at all
//!   before.
//!
//!   **The payload served during that window is still worse than "unknown":** the
//!   record keeps the PRE-restart `last_frame_at_ns` (nothing clears it), so the
//!   row reads [`LivenessState::Idle`] carrying the DEAD run's age, growing by a
//!   second every second. That is an affirmatively WRONG staleness claim — it
//!   understates freshness — not an absence of information. It is not the only
//!   residual here that asserts something FALSE: the mid-flush interleave and the
//!   slow-drained re-flush each DATE a dead route, which is the STRONGER false
//!   claim (they fabricate a `Streaming` reading, inside the recency window, on a
//!   route carrying nothing) — but each of those costs exactly ONE dated batch,
//!   where this one persists for the whole confirmation window. What it still
//!   cannot do is claim the topic is healthy or dead: it is never falsely
//!   [`LivenessState::Streaming`] (a `Streaming` reading inside
//!   [`LIVENESS_STREAMING_RECENCY_MS`] is backed by a genuinely dated frame the
//!   observer really saw) and never dimmed ([`LivenessState::NoData`] requires
//!   `frames_observed == 0`).
//!
//!   Two shapes are still NOT bounded by the new path, and they are named in its
//!   own section: a topic with two concurrently-attached LOCAL writers (excluded
//!   by the single-writer condition BY DESIGN — but note that condition is a
//!   PER-DRAIN local port count, so a transient single-writer window admits one),
//!   and a topic whose producer outruns its drainer so consistently that no drain
//!   ever empties the queue. The second is NOT excusable as "already undatable":
//!   a topic that closed its baseline while idle dates fine and only becomes
//!   unhealable when the saturation and the restart coincide.
//!
//!   It is also a corner rather than the shipping restart shape: the observer's own
//!   data-only tap holds the iceoryx2 service open across a worker swap, so a
//!   `graph run-worker` restart is normally observed on a CONTINUOUSLY-ATTACHED
//!   tap with a closed baseline, where the boot silence clears the gate outright
//!   and no confirmation window is paid at all.
//! * **A re-flush whose chunks are separated by MORE than
//!   [`REGRESSION_RESET_MIN_GAP_NS`] can still lower the bar.** The silence gate
//!   would then read the later chunk as a new epoch. It requires the publisher's
//!   `update_connections` backlog push to be drained in pieces two sweep intervals
//!   apart, which the burst shape argues against (the whole backlog is pushed at
//!   once, and both drainers take it at cadence), and it costs one dated batch on a
//!   dead route — the same bounded cost as the first-flush interleave above.
//! * **A `multi_publisher_topics` topic** (opt-in, e.g. `/tf`) mixes stamps from
//!   independent publishers, so the "maximum" is across clocks that need not
//!   agree. It can over-report freshness on such a topic; it cannot dim one. The
//!   epoch reset reads a stamp regression as a restart and two disagreeing
//!   publishers interleaving produce regressions routinely — but the silence gate
//!   makes that churn RARE rather than routine: a topic whose publishers are both
//!   live is drained at cadence, so its regressions arrive with no silence in front
//!   of them and are banked. Churn survives only where a lower-stamping publisher
//!   lands after a [`REGRESSION_RESET_MIN_GAP_NS`] lull.
//!
//!   The opt-in itself is invisible here (it is a graph-YAML fact and nothing in
//!   the [`GatewayPlan`](super::gateway::GatewayPlan) the gateway is handed carries
//!   it), but the LIVE publisher count is not: both drainers report
//!   [`DrainObservation::writers_seen`] on every drain (a dynamic-config read on
//!   the tap's own already-open service handle, no `.open()`), so the evidence is
//!   already paid for. The SUSTAINED path uses exactly it, but the count is
//!   a LIVE, PER-DRAIN, LOCAL-PORT reading, not the YAML opt-in, so the exclusion
//!   is only as good as "both writers are attached right now". While one of them
//!   is detached (restarting, not yet up, a short-lived node) the topic reads
//!   `Some(1)` and the sustained path DOES apply to it. The second-reset-path section states
//!   that scope in full; the bounded consequence is unchanged.
//!
//!   The SILENCE-gated path is deliberately NOT gated on the same evidence, and
//!   the reason is scope rather than cost: it predates the evidence, its behaviour
//!   on these topics is pinned by
//!   `interleaved_publisher_clocks_never_dim_the_row`, and narrowing it is a
//!   separate change with its own trade (a multi-publisher topic would then never
//!   reset an epoch at all, including across a genuine restart of one of its
//!   writers). So the residual churn is ACCEPTED, as it was. The bounded
//!   consequence either way is that such a topic may read `Idle` where a
//!   single-writer one would read `Streaming`; it can never be dimmed, because
//!   `frames_observed > 0` forbids [`LivenessState::NoData`] outright, and it can
//!   never be falsely `Streaming`, because that verdict still requires a batch
//!   that genuinely advanced past whatever epoch is current.
//! * **A queue that is never EMPTY at any drain never closes its baseline**, so
//!   such a topic stays `Idle` (produced, undatable) for as long as the saturation
//!   lasts. This is a genuine saturation residual rather than an arithmetic
//!   one: both drainers determine emptiness by OBSERVATION — the observer loops to
//!   a short read, and the gateway's one-budget-per-pass egress drain ASKS its port
//!   (the non-consuming [`DataOnlySubscriber::has_samples`]) whenever its read came
//!   back full — so a read that merely filled its budget never masquerades as
//!   "still backlogged". A topic whose
//!   producer outruns the gateway at every single pass is the remaining case, and
//!   it reads `Idle`, never dimmed.
//!
//! # Cost
//!
//! * **Zero** new work in the publisher's hot path — no counter, no atomic, no
//!   allocation, no clock read. Nothing in `loan_proxy`/`publish_raw` changes.
//! * **One** gateway subscriber port per observed topic — which is exactly the
//!   footprint a DEMANDED topic already pays today. The observer YIELDS its tap
//!   to the egress tap whenever a topic is being forwarded
//!   ([`TopicLivenessObserver::set_externally_observed`], via
//!   [`TopicLivenessObserver::release_own_tap`]) and reads liveness from the
//!   frames that drain ALREADY produces, so the peak gateway port count per topic
//!   stays at one. An UNDEMANDED topic goes from zero gateway ports to one; that
//!   is the full, and only, steady-state cost of this feature, and
//!   `CERULION_TOPIC_LIVENESS=off` turns it off entirely.
//! * That standing port is why
//!   [`INTROSPECTION_SUBSCRIBER_HEADROOM`](super::INTROSPECTION_SUBSCRIBER_HEADROOM)
//!   is 5 rather than 4: the observer is a PERMANENT fifth introspection
//!   consumer, and the four genuinely-spare slots (`bagd`, `topic echo`, `hz`,
//!   vizd) are provisioned on top of it, not shared with it.
//! * **RAM: bounded, and deliberately shallow.** An observer tap opens with a
//!   [`LIVENESS_TAP_BUFFER_SIZE`]-deep receive queue rather than the topic's full
//!   `subscriber_max_buffer_size` (which a `--record` topic can raise into the
//!   thousands). A subscriber's queue depth is how many SHM chunks it can PIN
//!   from the producer's pool between drains, so a deep observer tap on a fast
//!   topic could starve the producer's own loans across a 200 ms sweep. The
//!   shallow queue costs only precision in
//!   [`TopicLiveness::frames_observed`] — the newest frame is always retained
//!   (`drop_oldest`), so the AGE, which is what every threshold keys on, is
//!   exact.
//! * The tap budget ([`DEFAULT_LIVENESS_TAP_BUDGET`]) bounds the worst case on a
//!   very large robot; topics past the budget report `None` (UNKNOWN), never a
//!   fabricated "dead".
//!
//! # Degradation (`unknown` is never `live`, and never `dead`)
//!
//! Every step of the chain distinguishes "observed and saw nothing" from "did
//! not observe". The governing rule is that a report requires a CURRENTLY-ACTIVE
//! observation: a tap that attached and was then lost reverts to UNKNOWN rather
//! than freezing its last verdict, because nothing is watching to update it.
//!
//! | Situation | [`LivenessRecord::snapshot`] | Desk meaning |
//! |---|---|---|
//! | robot predates the liveness field / observer off | field absent ⇒ `None` | UNKNOWN — render as before the field existed |
//! | tracked but the tap never attached (slots exhausted, budget) | `None` | UNKNOWN |
//! | observed, then the tap was LOST (drain error, hand-off with no drainer) | `None` | UNKNOWN — never a stale verdict |
//! | observing < [`LIVENESS_NO_DATA_MIN_MS`], NOTHING seen at all | `Some`, age `None`, `frames_observed: 0` | [`LivenessState::Unknown`] — still settling |
//! | observing ≥ [`LIVENESS_NO_DATA_MIN_MS`], NOTHING seen at all | `Some`, age `None`, `frames_observed: 0` | [`LivenessState::NoData`] — the dead route |
//! | frames seen, none of them datable (a baseline burst / flushed backlog) | `Some`, age `None`, `frames_observed > 0` | [`LivenessState::Idle`] — produced, freshness unknown |
//! | frame within [`LIVENESS_STREAMING_RECENCY_MS`] | `Some(age)` | [`LivenessState::Streaming`] |
//! | frames seen, but not recently | `Some(age)` | [`LivenessState::Idle`] (carries the age) |
//!
//! # Clocks
//!
//! Two clocks are involved and they are kept strictly apart.
//!
//! * **The PUBLISHER's clock** appears only inside the advancement rule, and only
//!   ever compared against ITSELF (this topic's own previous maximum). Nothing
//!   else reads a wire stamp.
//! * **The OBSERVER's clock** produces every number that leaves this module:
//!   `last_frame_age_ms` is `now − (the instant we DRAINED the advancing batch)`
//!   and `observed_for_ms` is a sum of observer-clock intervals.
//!
//! So the report never subtracts one clock's reading from another's — not the
//! publisher's from the observer's (they are unrelated number lines on the
//! multi-process default, see above), and not the robot's from the desk's, which
//! is why the catalog serves an AGE the robot computed rather than a timestamp
//! the desk would have to interpret.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

use super::subscriber::{DataOnlySubscriber, OwnedInboundSample};
use super::TransportManager;
use crate::error::TransportError;

/// How often [`TopicLivenessObserver::sweep`] actually drains its taps.
///
/// The observer throttles ITSELF rather than making the caller schedule it: a sweep
/// is a control-plane duty, and 200 ms is far finer than the ~2 s sidebar refresh it
/// feeds. But the throttle is a FLOOR, not a cadence — `sweep` can only run when it
/// is CALLED, and its one production caller is `GatewayRuntime::drive_once`, so the
/// real grid is `max(this, the drive loop's pacing)`.
///
/// That matters at ZERO DEMAND, which is exactly the state this observer exists for
/// (it watches topics nobody has demanded — the ones whose liveness nothing else can
/// see). The drive loop parks such a gateway instead of spinning it, so the park is
/// what paces the observer there, and
/// `cerulion_core::transport::gateway::GATEWAY_ZERO_DEMAND_IDLE` is DERIVED from this
/// constant and const-asserted `<=` it for that reason: the two cannot drift apart,
/// and every band keyed on this value ([`REGRESSION_RESET_MIN_GAP_NS`],
/// [`SUSTAINED_REGRESSION_MIN_SPAN_NS`], [`RATE_ESTIMATE_MIN_WINDOW_NS`],
/// [`RATE_FLOOR_BASIS_CEILING_MHZ`]) keeps describing the grid a real gateway runs on.
/// While any demand is ON the loop paces at `GATEWAY_IDLE_POLL` (1 ms) and this
/// throttle binds outright.
pub const LIVENESS_SWEEP_INTERVAL_NS: u64 = 200_000_000;

/// How long the OBSERVER's clock must have been SILENT — no drain yielding a
/// single frame — before a stamp regression on a closed baseline may be read as a
/// publisher RESTART (see the module docs' restart section) rather than as a later
/// chunk of ONE re-flushed backlog.
///
/// The two look identical from stamps alone (both are batches below the recorded
/// maximum, both can ascend), so the discriminator is TIME, measured entirely on
/// the observer's own clock — never against a publisher stamp:
///
/// * A re-flush is a BURST. The publisher's `update_connections` pushes the whole
///   retained backlog into the fresh connection at once, and the drainer takes it
///   at whatever cadence it runs — one budget per gateway drive pass, or one
///   drain-to-empty per observer sweep. Its chunks are therefore separated by at
///   most one drain cadence.
/// * A restart is a PROCESS lifecycle event: the old worker dies, a new one boots,
///   builds its graph, opens its services and publishes. The observer sees SILENCE
///   across that whole span — orders of magnitude longer than a drain cadence.
///
/// Two sweep intervals is the threshold: comfortably above the cadence at which
/// chunks of one burst can possibly arrive (the observer's own sweep is the
/// SLOWEST drainer at [`LIVENESS_SWEEP_INTERVAL_NS`], and it drains to empty in
/// one pass anyway), and far below any real process restart.
///
/// That reasoning covers only the BURST side, and the other half has to be said
/// out loud because it is where the cost lands. The SAME sweep cadence drives the
/// frame-yielding drains that ANCHOR this gate, so what clearing it actually
/// requires is a HOLE in the DRAINED stream, aligned to the sweep grid: with drains
/// on an `S` = [`LIVENESS_SWEEP_INTERVAL_NS`] grid, a SINGLE sweep coming back with
/// ZERO frames already puts `2S` between the frame-yielding drains on either side of
/// it (drains at `t`, `t + S` empty, `t + 2S` ⇒ a gap of `2S`, which is exactly this
/// constant). The gate is therefore a function of the DRAIN cadence, not of the
/// publisher's rate — and the drain cadence depends on WHICH plane is observing the
/// topic. The three bands immediately below are the OBSERVER path: the topic's own
/// sweeping tap, draining on the `S` grid. The DEMAND/egress path has a different —
/// harsher — boundary, stated after them. On the observer path the publisher's period
/// `P` against `S` is the whole story, in THREE bands rather than two:
///
/// * **`P >= 2S`** — slower than the GATE itself (under ~2.5 Hz at these values).
///   Consecutive publishes are more than two sweeps apart, so EVERY pair of
///   frame-yielding drains is already at least `2S` apart and a banked regression
///   clears within a sweep or two. This is the only band where "it heals on its own"
///   is true.
/// * **`S < P < 2S`** — between the sweep rate and the gate rate (roughly 2.5 to
///   5 Hz). Most drains yield frames, and an empty one appears only where the
///   publish grid and the sweep grid beat against each other: once every
///   `S * P / (P − S)`. That beat DIVERGES as `P` approaches `S` from above —
///   `P = 250 ms` ⇒ ~1 s, `P = 210 ms` ⇒ ~4.2 s, `P = 201 ms` ⇒ ~40 s — so a topic
///   in this band stays un-reset for an arbitrarily long, beat-dependent stretch.
///   Neither "a sweep or two" nor never.
/// * **`P <= S`** — at or above the sweep rate (roughly 5 Hz and up: every control
///   and sensor topic on a real robot). EVERY drain yields frames, the gap is never
///   more than `S`, and the gate can NEVER clear while the topic keeps streaming.
///   Only a genuine `REGRESSION_RESET_MIN_GAP_NS` pause in the DRAINED stream does.
///
/// On the DEMAND/egress plane the arithmetic collapses to ONE band, and a worse one.
/// A topic a remote consumer is streaming is marked
/// [`set_externally_observed`](TopicLivenessObserver::set_externally_observed), so
/// [`sweep`](TopicLivenessObserver::sweep) SKIPS it entirely and every gate anchor
/// arrives instead through
/// [`note_frames`](TopicLivenessObserver::note_frames), called from the gateway's
/// egress drain on a drive loop that polls at ~1 kHz and reports only the passes that
/// actually drained frames. There is no `S` grid for the publish grid to beat
/// against: consecutive frame-yielding drains sit roughly `P` apart, so the gap
/// between anchors cannot reach `2S` unless the PUBLISHER itself pauses that long.
/// The never-clears boundary is therefore `P < 2S` (~2.5 Hz) rather than `P <= S`
/// (~5 Hz) — the whole middle band, which on the observer path clears once per beat,
/// clears NEVER while the topic is demanded. That is the plane a vizd-demanded topic
/// rides, so the widened band covers exactly the streams Studio renders.
///
/// So the gate is easiest to clear on the topics where a missed epoch reset costs
/// least, and hardest — unreachable, in each plane's fast band — on the ones where it
/// costs most. Raising or lowering this constant cannot fix that, because the anchor
/// moves with it. **The remedy is instead a SECOND, independent reset path**
/// that asks about the regression REGIME rather than the silence in front of it (see
/// [`SUSTAINED_REGRESSION_MIN_SPAN_NS`] and the module docs); this gate is unchanged
/// and still heals what it heals, faster than that path can.
pub const REGRESSION_RESET_MIN_GAP_NS: u64 = 2 * LIVENESS_SWEEP_INTERVAL_NS;

/// How long a PENDING-EPOCH run — consecutive frame-yielding batches that
/// have all stayed strictly BELOW the recorded maximum, on a closed baseline — must
/// persist on the OBSERVER's clock before it is read as a publisher RESTART that
/// arrived without a lull.
///
/// This is the SECOND reset path's time condition. Its companions are
/// [`SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS`] (the structural, burst-killing one),
/// single-writer evidence, and an ascending run; all four are required, and the
/// module docs' second-reset-path section explains what each closes.
///
/// FIVE sweep intervals, and the two bounds that pick it:
///
/// * **Above** the mid-flush interleave it must not be fooled by. That interleave is
///   a MICROSECOND-scale overlap between a drain and an in-flight `deliver_history`
///   push (module docs' residual list), so five sweeps clears it by five orders of
///   magnitude. It is also comfortably above the whole-burst drain time of any
///   realistic backlog on either plane, which is the belt to condition 2's braces.
/// * **Below** the ~2 s sidebar refresh and well below
///   [`LIVENESS_STREAMING_RECENCY_MS`], so a restarted topic's row recovers inside
///   one or two refreshes rather than after a visible stall.
///
/// It is expressed in sweep intervals rather than as a bare duration because the
/// OBSERVER plane can only ever gather evidence on that grid: a span shorter than a
/// couple of sweeps would be satisfied before the sweep grid could supply the
/// drains condition 2 wants, which would make the count the only binding condition
/// on that plane and hide a later loosening of it.
pub const SUSTAINED_REGRESSION_MIN_SPAN_NS: u64 = 5 * LIVENESS_SWEEP_INTERVAL_NS;

// The sustained path is the FALLBACK for regressions the silence gate cannot
// reach, so it must never be the QUICKER route on a regression the gate would
// have judged for itself. If a run could confirm in less observer time than the
// gate's own threshold, a regression preceded by nearly-enough silence would take
// the sustained path first and the gate's carefully-argued burst defence would be
// moot. Enforced at COMPILE time so neither constant can drift into that.
const _: () = assert!(
    SUSTAINED_REGRESSION_MIN_SPAN_NS > REGRESSION_RESET_MIN_GAP_NS,
    "the sustained-regression path must take STRICTLY longer to confirm \
     than the silence gate, or it would pre-empt the gate on regressions the gate \
     is there to judge"
);

/// How many of a pending-epoch run's drains must have left the subscriber
/// queue EMPTY ([`DrainObservation::queue_emptied`]) before the run may be read as a
/// restart.
///
/// **It binds on ONE plane, and that has to be said first, because the obvious
/// reading of this constant — "the structural, non-timing half of the defence" — is
/// only true on the DEMAND plane.**
///
/// * **DEMAND/egress plane (where it binds).** The drain takes one budget per drive
///   pass, so a prefix chunk leaves the queue non-empty BY CONSTRUCTION — the rest
///   of the burst is still in it. A flush is ONE `update_connections` push, so a
///   drain that empties the queue has taken everything the publisher had for us,
///   and a SECOND emptied drain still below the bar means it produced MORE after
///   that point, which a dead route cannot do. Here the condition really is
///   structural and really does refuse a burst without reference to time.
/// * **OBSERVER plane (where it is INERT).**
///   [`TopicLivenessObserver::sweep`]'s drain loops to a SHORT READ, so EVERY
///   frame-yielding drain reports `queue_emptied` and this condition degenerates to
///   "the run has at least two drains" — it contributes no protection at all.
///   [`SUSTAINED_REGRESSION_MIN_SPAN_NS`] is the WHOLE defence there, and what makes
///   it sufficient is the PHYSICAL bound written in that constant's docs: a
///   synchronous `deliver_history` completes in microseconds-to-milliseconds, so a
///   burst cannot supply an ascending, sub-max, queue-emptying drain on drain after
///   drain for a whole second.
///
/// So the two conditions are COMPLEMENTARY, not belt-and-braces on one argument:
/// each is the operative one on the plane where the other degenerates. A future
/// edit that weakens either must check which plane it is weakening.
///
/// TWO, not one, and not three:
///
/// * ONE is REACHABLE from a burst on the demand plane too. The mid-flush interleave
///   (module docs' residual list) has a drain empty the queue while
///   `deliver_history` is still pushing, so a prefix chunk can truthfully report
///   `queue_emptied` exactly once. A threshold of one would let that single
///   documented coincidence promote a re-flush.
/// * TWO requires that coincidence TWICE within one burst, on a drainer whose every
///   other prefix read leaves the queue non-empty by construction. Pinned by
///   `a_re_flush_with_two_emptied_prefix_drains_is_refused_by_the_span`, which drives
///   exactly that and shows the span refusing it.
/// * THREE buys nothing further and costs healing time on the DEMAND plane, whose
///   drains are the ones a slow topic supplies rarely.
pub const SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS: u32 = 2;

// One emptied drain is reachable from the single documented mid-flush interleave,
// so a threshold of one would let that coincidence promote a re-flush. Enforced at
// COMPILE time.
const _: () = assert!(
    SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS >= 2,
    "a single emptied prefix chunk is reachable from the documented \
     mid-flush interleave, so the sustained path needs at least two"
);

/// Upper bound on how many topics one observer will hold taps for. Past this,
/// additional topics stay untracked and report `None` (UNKNOWN) — never a
/// fabricated "no data". Chosen far above any realistic robot (a Go2
/// exposes 75 topics) while still bounding the port footprint of a
/// pathological graph.
pub const DEFAULT_LIVENESS_TAP_BUDGET: usize = 256;

/// A frame this recent means the topic is actively streaming.
pub const LIVENESS_STREAMING_RECENCY_MS: u64 = 5_000;

/// How long a topic must be observed having seen NO FRAME AT ALL before "nothing
/// is publishing" is a confident claim rather than a not-yet-settled one. Below
/// this the state is [`LivenessState::Unknown`], so a topic is never libeled as
/// dead before it has had a fair chance to publish.
///
/// It MUST be at least [`LIVENESS_STREAMING_RECENCY_MS`] (const-asserted below),
/// and this is not a style rule — it is a consistency one. A topic publishing
/// every 6 s is classified [`LivenessState::Idle`] once it HAS published, so a
/// threshold shorter than the recency window would libel that same topic
/// [`LivenessState::NoData`] during the wait for its FIRST frame: the two rules
/// would disagree about the same publisher depending only on when observation
/// started. 10 s doubles the recency window, so a topic slower than "streaming"
/// still gets a full extra window to prove itself before the sidebar dims it.
///
/// ONE observed frame is all a topic needs to escape this state permanently —
/// not one DATED frame. A frame that only establishes the advancement baseline
/// (the module docs' retained-history section) still banks into
/// [`TopicLiveness::frames_observed`], and a nonzero count classifies
/// [`LivenessState::Idle`], never `NoData`. So this threshold bounds the wait for
/// the first frame a topic delivers to us, never the wait for a second one.
pub const LIVENESS_NO_DATA_MIN_MS: u64 = 10_000;

// The consistency invariant above, enforced at COMPILE time so the two
// thresholds cannot drift into contradiction in a later edit.
const _: () = assert!(
    LIVENESS_NO_DATA_MIN_MS >= LIVENESS_STREAMING_RECENCY_MS,
    "LIVENESS_NO_DATA_MIN_MS must be >= LIVENESS_STREAMING_RECENCY_MS — \
     otherwise a topic slower than the recency window is reported `no_data` while \
     waiting for its first frame but `idle` for the identical gap afterwards"
);

/// Receive-queue depth of an observer tap — deliberately far below the topic's
/// `subscriber_max_buffer_size` ceiling (see the module docs' Cost section). A
/// subscriber's queue depth bounds the SHM chunks it PINS between drains, so a
/// ceiling-deep observer tap on a `--record` topic (ceiling in the thousands)
/// could starve the producer's own loans across a 200 ms sweep. Two, not one, so
/// the drain loop is genuinely exercised and a two-frame burst between sweeps is
/// still counted; the newest frame is always retained either way, which is what
/// every threshold keys on.
pub const LIVENESS_TAP_BUFFER_SIZE: usize = 2;

/// The MINIMUM span one rate-estimate window must cover before it is
/// closed and served. A window is closed by the FIRST frame-yielding drain at or
/// past this age, so it is a floor on the span, never a cap: a 0.2 Hz topic's
/// window simply closes at ~5 s, when its next frame arrives.
///
/// Ten observer sweeps. The count inside a window is EXACT (a wire-sequence
/// delta — see [`TopicRateEstimate`]); the error lives entirely in the
/// DENOMINATOR, because the window's endpoints are the instants we DRAINED the
/// newest frame rather than the instants the publisher committed it, and those
/// differ by up to one drain cadence. Consecutive windows SHARE an endpoint (the
/// closing drain anchors the next window), so in steady state that lag is the
/// same at both ends and cancels; the residual is the CHANGE in lag across one
/// window.
///
/// No single percentage is quoted for that residual, because it has no one
/// denominator: the lag is bounded by the DRAIN cadence, and the two planes drain
/// on different clocks — the sweep interval on the observer plane, but the
/// GATEWAY'S POLL CADENCE on the demand plane (`GATEWAY_IDLE_POLL`, 1 ms, and a
/// pass that forwarded re-loops immediately rather than sleeping). It is the poll
/// cadence rather than the publisher's period because the gateway is what decides
/// when a frame is drained, so on any topic slower than ~1 kHz the demand-plane
/// lag is bounded by that millisecond, not by how often the topic publishes. The
/// window's own length varies between this minimum and the discard horizon. What
/// is true on every plane is the shape: the error is a lag DIFFERENCE over the
/// window length, so it shrinks as the window grows and vanishes when the cadence
/// is steady, which in the steady state that dominates it is.
///
/// Ten is the trade: fewer sweeps makes the window shorter and the residual a
/// larger fraction of it, more makes the number lag a rate CHANGE by longer than
/// the ~2 s sidebar refresh that reads it.
pub const RATE_ESTIMATE_MIN_WINDOW_NS: u64 = 10 * LIVENESS_SWEEP_INTERVAL_NS;

// A window can only close between the minimum span and the staleness horizon
// (`record_rate_window` discards anything longer as spanning a silence), so the
// two must not cross — otherwise no window could EVER close and the feature
// would ship inert. Enforced at COMPILE time so a later edit to either constant
// cannot quietly do that.
const _: () = assert!(
    RATE_ESTIMATE_MIN_WINDOW_NS < RATE_ESTIMATE_MAX_AGE_NS,
    "a rate window must be able to reach its minimum span before the \
     staleness horizon discards it as spanning a silence. STRICTLY less: at \
     equality the only closable window is one landing on the boundary INSTANT, \
     which no real drain cadence hits, so the feature would ship inert"
);

/// How old the last CLOSED rate window may be and still be served.
///
/// Deliberately the SAME threshold the classifier uses for
/// [`LivenessState::Streaming`], so ONE number governs both halves of the rule
/// "a rate is served only for a topic that is streaming NOW" and the two can
/// never drift into disagreement. A stopped stream therefore stops carrying a
/// rate at the same moment its row stops reading `Streaming` — it never serves a
/// decaying average, and never a number the observer has stopped re-measuring.
pub const RATE_ESTIMATE_MAX_AGE_NS: u64 = LIVENESS_STREAMING_RECENCY_MS * 1_000_000;

/// Backward wire-sequence movement tolerated inside one rate window
/// before the window is discarded as a publisher EPOCH change.
///
/// A restarted publisher's sequence begins again at 0, and a `u32` sequence
/// eventually wraps; both land far below the window's anchor and neither can be
/// counted against it. A SMALL backward step is different — a benign reorder —
/// and is absorbed by the saturating delta the window close applies, rather than
/// thrown away. Mirrors
/// `cerulion-vizd`'s `SEQ_RESET_TOLERANCE`, which computes the same quantity on
/// the desk for an ATTACHED topic (see [`TopicRateEstimate`]).
pub const RATE_SEQ_RESET_TOLERANCE: u32 = 8;

/// The highest rate the FLOOR basis can report on the OBSERVER plane,
/// in millihertz — DERIVED from the two constants that bound it, never written
/// out as a number.
///
/// The floor basis counts FRAMES DRAINED (see [`TopicRateEstimate::is_floor`]),
/// and an observer tap holds [`LIVENESS_TAP_BUFFER_SIZE`] frames while draining
/// once per [`LIVENESS_SWEEP_INTERVAL_NS`], so a window of drains on the sweep
/// grid lands about here however fast the publisher runs.
///
/// It is the TYPICAL ceiling, not a hard bound, and the overshoot comes from the
/// INNER loop rather than the outer one. The observer's OUTER drain loop cannot
/// iterate twice here: the tap's queue is [`LIVENESS_TAP_BUFFER_SIZE`] deep while
/// the borrow budget it asks for is larger, so the first read is always short and
/// breaks the loop. What can exceed the ratio is
/// [`DataOnlySubscriber::drain_owned`]'s own `while pushed < max` loop, which
/// issues one `receive()` per iteration: a frame the publisher commits BETWEEN two
/// of those receives is taken in the same pass. The figure is what the constants
/// imply for a steady producer, and it is what makes a floor-basis estimate worth
/// labelling — not an invariant anything enforces.
/// A floor-basis estimate sitting at this value therefore means "at least this,
/// and the observer cannot see how much more" — which is precisely why the basis
/// is LABELLED rather than served as if it were a measurement.
///
/// It bounds the OBSERVER plane only. On the demand/egress plane the drain rides
/// the gateway's ~1 kHz drive loop over a ceiling-deep tap, so its floor basis is
/// bounded by that loop instead and sits orders of magnitude higher. Neither
/// bound applies to the EXACT basis, which counts the publisher's own committed
/// sequences and is unaffected by what the tap could hold.
pub const RATE_FLOOR_BASIS_CEILING_MHZ: u64 =
    (LIVENESS_TAP_BUFFER_SIZE as u64) * 1_000 * 1_000_000_000 / LIVENESS_SWEEP_INTERVAL_NS;

/// Safety cap on frames drained from ONE topic in ONE sweep. A topic's queue is
/// bounded by its `subscriber_max_buffer_size`, so a full drain terminates well
/// under this; the cap exists purely so a pathological/looping producer cannot
/// starve the rest of the sweep. Any remainder is drained next sweep.
const MAX_DRAIN_PER_SWEEP: u64 = 8_192;

/// The environment kill switch. `off` disables observation entirely (the gateway
/// attaches no liveness taps and every catalog entry serves `liveness: None`,
/// byte-identical to the wire before the field existed). Any other value is rejected LOUDLY
/// and observation stays ON — an internal diagnostic seam, matching
/// `CERULION_DRAIN_DISCIPLINE`'s exact-match, no-case-forgiveness rule.
pub const LIVENESS_ENV: &str = "CERULION_TOPIC_LIVENESS";

// ---------------------------------------------------------------------------
// Wire type + pure classification.
// ---------------------------------------------------------------------------

/// What the robot OBSERVED about one topic's data flow — the additive per-entry
/// payload of the `catalog` verb
/// ([`CatalogEntry::liveness`](super::cerulion_q::CatalogEntry::liveness)).
///
/// Every field is derived from the observer's own clock, so the numbers are
/// self-consistent on the serving robot and need no clock agreement with the
/// reader.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopicLiveness {
    /// Milliseconds since the robot last observed a frame it could DATE — one
    /// whose publisher stamp advanced past everything seen before it, proving the
    /// publisher committed it while the robot was already watching (see the
    /// module docs' advancement rule). `None` means no batch has ever been
    /// datable: either nothing arrived at all, or everything that arrived was a
    /// baseline burst that could equally have been the publisher's flushed
    /// backlog.
    ///
    /// `None` is therefore not on its own the reported `/uslam/cloud_map`
    /// signature — pair it with `frames_observed` (which [`Self::state`] does):
    /// `None` with zero frames is the dead route, `None` with frames is a topic
    /// that produced data of unknown freshness.
    pub last_frame_age_ms: Option<u64>,
    /// Milliseconds this topic has been under observation, accumulated across
    /// tap hand-offs (an egress tap taking over from the observer's own tap does
    /// not reset it). Only time a tap was genuinely attached counts.
    pub observed_for_ms: u64,
    /// Frames the robot DRAINED since observation began. Separates "one frame
    /// ever" from "a live stream" without inviting the reader to compute a rate.
    ///
    /// It is a floor, not an exact count, and it is deliberately NOT redundant
    /// with `last_frame_age_ms.is_some()`:
    ///
    /// * It UNDER-counts a fast topic — an observer tap keeps a
    ///   [`LIVENESS_TAP_BUFFER_SIZE`]-deep queue, so frames published between two
    ///   sweeps beyond that depth are reclaimed before they are drained (the
    ///   newest is always kept, so the AGE stays exact).
    /// * It can be nonzero while `last_frame_age_ms` is `None`, and that
    ///   combination means "this topic produced data the robot cannot date":
    ///   the publisher's RETAINED HISTORY flushed into a fresh tap (measured; see
    ///   the module docs), a latched topic that published exactly once, or simply
    ///   a topic whose only batch so far established the advancement baseline.
    ///   It is BANKED — the frames really were seen — but it is not evidence that
    ///   anything published while we watched, so it renders
    ///   [`LivenessState::Idle`] (produced, freshness unknown) and NEVER
    ///   [`LivenessState::NoData`].
    ///
    /// This field is therefore load-bearing for the classification, not
    /// decoration: it is the difference between "registered but dead" and "we
    /// have its data, just not its clock".
    pub frames_observed: u64,
    /// How fast this topic is PUBLISHING, as measured over a recent
    /// window — or `None` when the robot will not claim a rate.
    ///
    /// This is the field that lets the desk show a frequency for a topic NOBODY
    /// HAS CHECKED. Before it, a rate existed only on the demand plane: checking
    /// a topic opened a mirror and `cerulion-vizd` computed Hz from the frames
    /// crossing it, so an unchecked row could say "live" and nothing more. The
    /// observer already drains a frame or two per sweep from every topic it
    /// watches, and those frames carry the publisher's own commit counter, so the
    /// rate was derivable from observation the robot was ALREADY doing.
    ///
    /// `None` is a real answer and covers every case where a number would be a
    /// guess: nothing is observing this topic (the whole payload is absent then),
    /// the topic is not classified [`LivenessState::Streaming`] right now, or the
    /// last window closed longer ago than [`RATE_ESTIMATE_MAX_AGE_NS`]. A stopped
    /// stream therefore drops its rate rather than decaying one toward zero.
    ///
    /// It is an ESTIMATE and named as one. It is derived from the same quantity
    /// `cerulion topic hz` and the vizd status rate use — the publisher's wire
    /// sequence over a wall window — but measured over a different window from a
    /// different vantage, so the two agreeing to the digit is not something
    /// either side promises. A user comparing them and seeing 19.8 against 20.1
    /// has found the window, not a bug.
    ///
    /// ADDITIVE on the wire: absent when there is no estimate, so a robot with
    /// nothing to say serializes byte-identically to a robot predating the field, and
    /// that older robot's payload decodes to `None` here.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rate_estimate: Option<TopicRateEstimate>,
}

/// One topic's observed PUBLISH rate, plus the basis marker for how
/// it was counted.
///
/// # How the number is obtained
///
/// Every frame carries the publisher's
/// [`WireHeader::sequence`](crate::wire::WireHeader) — a per-topic counter the
/// publisher consumes at COMMIT, so a published stream's sequences are gap-free
/// and the `ros2 attach` bridge's raw-ingress route keeps the same
/// discipline. The rate is therefore
///
/// ```text
/// (newest sequence at the closing drain − newest sequence at the anchoring drain)
/// ─────────────────────────────────────────────────────────────────────────────
///          (observer clock at close − observer clock at anchor)
/// ```
///
/// The numerator is a COUNT — dimensionless, belonging to no clock — so dividing
/// it by an observer-clock duration does not compare two clocks, and the module's
/// clock rule is untouched. That matters more than it looks: the publisher's OWN
/// stamps could not be used as the denominator, because a `graph run` worker's
/// gating clock advances by a fixed LOGICAL quantum per step and its
/// seconds are not wall seconds.
///
/// The count is exact **whatever the observer tap could hold**. The tap keeps
/// only [`LIVENESS_TAP_BUFFER_SIZE`] frames, so a 500 Hz topic delivers two
/// frames per sweep and 498 are reclaimed — but the two that arrive carry
/// sequence numbers 100 apart, and 100 is how many the publisher committed. This
/// is the whole reason the estimate is useful rather than pinned at the tap's
/// own throughput.
///
/// `cerulion-vizd` computes exactly this quantity, the same way, for a topic that
/// IS attached (its `TopicStatus::hz`). This is that measurement moved to the one
/// vantage point that has every topic in view without anyone checking anything.
///
/// # When the count is a FLOOR instead
///
/// The sequence is only as good as the publisher's discipline with it.
/// `publish_raw` writes the caller's bytes VERBATIM, so a hand-rolled raw
/// publisher that never advances its sequence would make the exact basis report
/// 0 Hz on a topic that is plainly streaming — a confidently wrong number, which
/// is worse than no number. So a window whose sequence did NOT advance while
/// frames DID arrive falls back to counting the frames the observer actually
/// drained, and says so via [`Self::is_floor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopicRateEstimate {
    /// The rate in MILLIhertz (frames per 1000 seconds).
    ///
    /// Integral, not a float, for two reasons. The payload is JSON, and a float
    /// there is a formatting decision every producer and consumer would have to
    /// agree on before two robots' numbers could be compared or a test could
    /// carry a byte oracle. And [`TopicLiveness`] is `Eq`, which a float forbids.
    /// Millihertz resolves 0.001 Hz, three orders finer than anything a sidebar
    /// renders, and covers past 1.8e16 Hz.
    pub millihertz: u64,
    /// `true` when [`Self::millihertz`] is a FLOOR — the topic publishes AT LEAST
    /// this fast and the robot cannot say how much faster. Render it as `≥ N Hz`,
    /// never as a measurement.
    ///
    /// It is set when the window had to count FRAMES DRAINED rather than
    /// publisher sequences (see the type docs), and frames drained is bounded by
    /// what the tap could hold: on the observer plane that ceiling is
    /// [`RATE_FLOOR_BASIS_CEILING_MHZ`], so a floor estimate sitting there says
    /// "at least 10 Hz" about a topic that may be running at 500.
    ///
    /// `false` means the count was the publisher's own committed sequence delta,
    /// which no tap depth can clip.
    pub is_floor: bool,
}

impl TopicRateEstimate {
    /// The rate in Hz. Convenience for rendering — the wire carries
    /// [`Self::millihertz`].
    pub fn hz(&self) -> f64 {
        self.millihertz as f64 / 1_000.0
    }
}

/// The rendering-ready classification of a [`TopicLiveness`]. Derived by
/// [`TopicLiveness::state`] so the thresholds live in ONE oracle-tested place
/// rather than being re-implemented by each consumer (the Studio sidebar reads
/// the serialized string; see `DiscoveredEntry`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LivenessState {
    /// A DATABLE frame arrived within [`LIVENESS_STREAMING_RECENCY_MS`].
    Streaming,
    /// Frames HAVE been seen, but the robot cannot say they arrived recently.
    /// Attaching may show a stale last value.
    ///
    /// This covers TWO shapes, and a reader must handle both:
    ///
    /// * `last_frame_age_ms: Some(age)` — dated, and `age` says how stale.
    /// * `last_frame_age_ms: None` with `frames_observed > 0` — AGE UNKNOWN. The
    ///   topic produced data (a flushed backlog, a latched one-shot, or a
    ///   baseline burst) but nothing has yet proven a publish happened while the
    ///   robot watched. Render it as "has data, freshness unknown"; do NOT
    ///   render it as dead, and do not invent an age.
    Idle,
    /// Watched for at least [`LIVENESS_NO_DATA_MIN_MS`] having seen NO FRAME AT
    /// ALL (`frames_observed == 0`) — the registered-but-dead route. This is the
    /// only state the sidebar dims.
    ///
    /// A topic that delivered even one frame — including a dead route's flushed
    /// RETAINED HISTORY, which is banked but never datable (see
    /// [`TopicLiveness::frames_observed`]) — is [`Self::Idle`] or
    /// [`Self::Streaming`], never this. Dimming a row that demonstrably carries
    /// data would be the worse error, and the robot cannot tell "published an
    /// hour ago" from "published before we attached" without the publisher's
    /// clock, which it deliberately does not interpret.
    NoData,
    /// Not enough observation to say anything: either nothing is being observed
    /// (no liveness payload at all) or observation has seen NO frame at all and
    /// started too recently to distinguish a dead route from a slow one. NEVER
    /// conflate with [`Self::NoData`].
    ///
    /// Note this state requires `frames_observed == 0`: one delivered frame, even
    /// an undatable one, is already enough to say [`Self::Idle`].
    Unknown,
}

impl LivenessState {
    /// The state's WIRE spelling, for a `tracing` field, a log line, or any other
    /// place a `&'static str` is needed.
    ///
    /// Exists so prose and JSON say the SAME word. A log line that spelled a state
    /// differently from the wire would make an operator's grep disagree with the
    /// agent's payload about which verdict fired — the
    /// two-copies-of-one-vocabulary class, in the one place a `Serialize` impl
    /// cannot reach. Same shape and same reason as
    /// [`MonitorCondition::as_wire`](crate::monitor::MonitorCondition::as_wire),
    /// and pinned against the serde spelling by
    /// `the_liveness_state_wire_spelling_agrees_with_serde`.
    pub const fn as_wire(&self) -> &'static str {
        match self {
            LivenessState::Streaming => "streaming",
            LivenessState::Idle => "idle",
            LivenessState::NoData => "no_data",
            LivenessState::Unknown => "unknown",
        }
    }
}

impl TopicLiveness {
    /// Classify with the shipped thresholds.
    pub fn state(&self) -> LivenessState {
        self.state_with(LIVENESS_STREAMING_RECENCY_MS, LIVENESS_NO_DATA_MIN_MS)
    }

    /// The classification AS IT IS SERIALIZED — `None` for
    /// [`LivenessState::Unknown`].
    ///
    /// There is exactly ONE wire encoding of UNKNOWN, and it is ABSENCE. A
    /// serialized `"unknown"` alongside an absent field would be two spellings of
    /// the same thing, and every reader would have to handle both to be correct.
    /// Producers of the desk protocol classify through this, so the serialized
    /// set is closed to `streaming` / `idle` / `no_data`. (DEcoding stays
    /// tolerant: an `"unknown"` from some other producer still parses.)
    pub fn wire_state(&self) -> Option<LivenessState> {
        match self.state() {
            LivenessState::Unknown => None,
            state => Some(state),
        }
    }

    /// Classify with explicit thresholds — PURE, the oracle-tested core of
    /// [`Self::state`].
    ///
    /// Three asymmetries are DELIBERATE, and all fall on the conservative side
    /// (never claim more than was observed):
    ///
    /// * **A PRODUCED topic is never `NoData`.** `frames_observed > 0` with no
    ///   age means the topic delivered data the robot cannot date (a flushed
    ///   backlog, a latched one-shot, a baseline burst — see the module docs).
    ///   Dimming that row would assert "nothing ever published", which the
    ///   banked frames refute; reporting it `Streaming` would assert a freshness
    ///   nothing proved. `Idle` is the only correct class, and it is what the
    ///   sidebar renders undimmed and un-dotted.
    /// * **[`LivenessState::Idle`] is unbounded.** A topic that published once,
    ///   an hour ago, stays `Idle` — it never decays to `NoData` (which asserts
    ///   "nothing ever published", now false) nor back to `Unknown` (which
    ///   discards a fact we actually measured). The age rides along when there is
    ///   one, so a caller that wants an "hours stale" rendering has the number;
    ///   the CLASS stays the correct one.
    /// * **A backward clock reads age 0, i.e. `Streaming`.** `last_frame_age_ms`
    ///   is computed with a saturating subtraction, so if the observer's clock
    ///   steps backwards past the observation the age floors at 0 rather than
    ///   wrapping to ~584 years. The report is then "as fresh as possible",
    ///   which for a topic that demonstrably HAD a frame is the conservative
    ///   direction (it over-reports freshness for one step of the retreat rather
    ///   than fabricating a dead route). Cerulion's own clocks are monotonic;
    ///   this is the defence against a `VirtualClock` a host rewinds.
    pub fn state_with(&self, recency_ms: u64, no_data_min_ms: u64) -> LivenessState {
        match self.last_frame_age_ms {
            Some(age) if age <= recency_ms => LivenessState::Streaming,
            Some(_) => LivenessState::Idle,
            // Undatable but PRODUCED: frames really arrived, nothing proved when.
            // Never the dimmed row (see `LivenessState::NoData`), never a
            // fabricated freshness.
            None if self.frames_observed > 0 => LivenessState::Idle,
            // Nothing has EVER arrived: only a long-enough observation turns that
            // into the confident "no data" claim.
            None if self.observed_for_ms >= no_data_min_ms => LivenessState::NoData,
            None => LivenessState::Unknown,
        }
    }
}

/// What ONE drain saw — the evidence a drainer hands [`LivenessRecord`] (the
/// observer's own sweep does it directly; the gateway's egress tap goes through
/// [`TopicLivenessObserver::note_frames`]).
///
/// A named struct rather than three positional arguments because
/// [`Self::queue_emptied`] is a bare `bool` whose meaning is not recoverable at a
/// call site, and getting it wrong silently re-opens the split-flush hole the
/// module docs describe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrainObservation {
    /// Frames this drain actually took. Zero is a real observation of ABSENCE.
    pub frames: u64,
    /// The LARGEST publisher loan-time
    /// [`WireHeader::timestamp_ns`](crate::wire::WireHeader) across those frames,
    /// or `None` when none carried a parseable header. The drainer already holds
    /// the frames, so reading it is free.
    pub newest_stamp_ns: Option<u64>,
    /// The LARGEST publisher
    /// [`WireHeader::sequence`](crate::wire::WireHeader) across those frames, or
    /// `None` when none carried a parseable header — the numerator of
    /// [`TopicRateEstimate`].
    ///
    /// Read from the SAME header parse that yields [`Self::newest_stamp_ns`], so
    /// it costs a comparison and nothing else. Deliberately a separate field
    /// rather than something derived from [`Self::frames`]: the frame COUNT is
    /// what the observer's shallow tap clips, and the sequence is what it cannot.
    pub newest_sequence: Option<u32>,
    /// How many publishers were attached to the topic when this drain
    /// ran ([`DataOnlySubscriber::publisher_count`]), or `None` when the drainer
    /// did not ask.
    ///
    /// The single-writer EVIDENCE the sequence basis requires. `sequence` is a
    /// PER-PUBLISHER commit counter and [`Self::newest_sequence`] is the MAX
    /// across a batch, so on a topic with two writers that maximum hops between
    /// unrelated counters and its delta is not a frame count at all — measured on
    /// a two-writer `/tf` at ~101 Hz it reported **1550 Hz**, and on a
    /// long-uptime shape **25,000,005 Hz**, each served as a confident
    /// `is_floor: false` measurement. The backward-motion guard cannot catch it:
    /// a FORWARD hop across counters is indistinguishable from a fast single
    /// writer whose frames the tap clipped. So anything other than `Some(1)`
    /// forces the window to the labelled FLOOR (see
    /// [`TopicRateEstimate::is_floor`]) — a true lower bound instead of a
    /// confident wrong number.
    ///
    /// `None` is treated exactly like a multi-writer answer: the ABSENCE of
    /// evidence is not evidence of a single writer.
    pub writers_seen: Option<u32>,
    /// Whether this drain left the subscriber queue EMPTY — i.e. it came back
    /// with fewer frames than it asked for, and did not error.
    ///
    /// It is what ENDS the baseline burst (module docs): until a drain empties
    /// the queue, more of the SAME retained-history flush may still be sitting in
    /// it, and a later chunk of one flush must not be read as the publisher
    /// having produced something new.
    ///
    /// `false` is the safe direction ONLY in the sense that a wrong `true` can
    /// date a dead route. It is not free, and it is not merely a delay: the
    /// baseline closes exclusively on a `true`, so a drainer that never manages to
    /// report one leaves the baseline open FOREVER and its topic never dates at
    /// all. A drainer whose read can SATURATE — come back with exactly as many
    /// frames as it asked for — must therefore not infer emptiness from
    /// arithmetic: a full read proves nothing about the remainder, and with a
    /// budget of 1 "fewer frames than asked for" is unreachable at any site that
    /// already knows a frame arrived. It must OBSERVE emptiness instead, either by
    /// looping to a short read (what [`TopicLivenessObserver::sweep`]'s own drain
    /// does) or by ASKING the port after a saturated read — the non-consuming
    /// [`DataOnlySubscriber::has_samples`], which is what the gateway's
    /// one-budget-per-pass egress drain does. Non-consuming matters: draining one
    /// extra frame answers the same question but also changes what the caller
    /// forwards and when, and an emptiness query must not move the data plane.
    pub queue_emptied: bool,
}

/// The observation state the observer keeps for one topic. `snapshot` is PURE
/// (oracle-tested); everything that mutates it is driven by the sweep.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LivenessRecord {
    /// Observation time already banked from FINISHED observation intervals (a
    /// tap that was dropped, or handed to egress and handed back).
    observed_accum_ns: u64,
    /// Observer-clock ns at which the CURRENT observation interval began, or
    /// `None` when nothing is observing this topic right now.
    observing_since_ns: Option<u64>,
    /// Observer-clock ns at which the newest frame was drained. Never cleared
    /// when observation pauses, so a topic that streamed, went unobserved, and
    /// is observed again resumes with a growing age rather than forgetting it
    /// ever streamed. (While nothing is observing, [`Self::snapshot`] withholds
    /// the whole record — see its docs — so this is banked history, never a live
    /// claim.)
    last_frame_at_ns: Option<u64>,
    /// Frames drained since the first observation interval began.
    frames_observed: u64,
    /// The HIGHEST publisher loan-time
    /// [`WireHeader::timestamp_ns`](crate::wire::WireHeader) ever observed on
    /// this topic — the one and only thing a new batch is compared against.
    ///
    /// It is a PUBLISHER-clock value compared only against other publisher-clock
    /// values from the same single-writer topic, so the rule is sound whatever
    /// clock that publisher runs (module docs' "Why the comparison must not
    /// involve the observer's clock"). It PERSISTS across hand-offs, lost taps
    /// and re-attaches on purpose: that is what makes a re-flush of the same
    /// retained history unable to advance anything.
    ///
    /// It is NOT monotone, and that is the one deliberate exception: a batch
    /// stamped BELOW it cannot have come from the run that set it (one writer, one
    /// clock, non-decreasing within a run), so a regression is read as a NEW
    /// publisher epoch — a restart — and the maximum is RESET to the regressed
    /// batch's stamp rather than kept. Without that a restarted worker (whose
    /// `VirtualClock` begins again at 0) could never date its topics for the
    /// length of the previous run. See the module docs' restart section.
    ///
    /// The one exception has TWO guards, and a regression must clear BOTH before
    /// it is allowed to lower the maximum:
    ///
    /// 1. [`Self::baseline_open`] must be FALSE. Within one baseline burst the
    ///    maximum is monotone, because an early chunk of a RE-FLUSH is a strict
    ///    prefix of the same backlog and adopting it would lower the bar for the
    ///    chunks behind it.
    /// 2. The observer must have been SILENT for at least
    ///    [`REGRESSION_RESET_MIN_GAP_NS`] since the last drain that yielded frames
    ///    ([`Self::last_nonempty_drain_at_ns`]) **OR** the regression must be
    ///    SUSTAINED (see [`Self::regression_run_since_ns`]), because a
    ///    restart that arrives without a lull can never satisfy the silence half.
    ///    Guard 1 alone is not enough for either: a flush whose FIRST chunk happens
    ///    to empty the queue closes the baseline MID-BURST, and its next chunk then
    ///    arrives on a closed baseline.
    max_stamp_ns: Option<u64>,
    /// The publisher wire SEQUENCE this rate window is counting from,
    /// or `None` when the anchoring drain carried no parseable header.
    rate_anchor_seq: Option<u32>,
    /// Observer-clock ns at which the current rate window was anchored,
    /// or `None` when no window is open (nothing has been counted yet).
    rate_anchor_at_ns: Option<u64>,
    /// Frames DRAINED since the anchor — the fallback numerator, used
    /// only when the sequence did not advance (see [`TopicRateEstimate`]).
    rate_window_frames: u64,
    /// Whether EVERY drain folded into the open window saw exactly ONE
    /// publisher ([`DrainObservation::writers_seen`]).
    ///
    /// The sequence basis is only meaningful for a single writer, so a window
    /// that ever saw a different answer closes on the FLOOR basis instead. It is
    /// re-armed at each anchor, so a topic is judged on the writers present
    /// during the window it is reporting — not on a count from minutes ago.
    rate_window_trusted: bool,
    /// The last CLOSED window's answer. Held, never decayed: it is a
    /// measurement of a window that really happened, and it stops being served
    /// (rather than sliding toward zero) once it ages past
    /// [`RATE_ESTIMATE_MAX_AGE_NS`] or the topic stops classifying
    /// [`LivenessState::Streaming`].
    rate_estimate: Option<TopicRateEstimate>,
    /// Observer-clock ns at which [`Self::rate_estimate`] was computed —
    /// the staleness gate `snapshot` applies. Distinct from
    /// [`Self::rate_anchor_at_ns`], which tracks the window still being filled.
    rate_estimate_at_ns: Option<u64>,
    /// Observer-clock ns of the last drain that yielded FRAMES — the
    /// anchor for [`REGRESSION_RESET_MIN_GAP_NS`].
    ///
    /// Zero-frame drains deliberately do NOT touch it: silence is exactly what
    /// this measures, and a topic being polled every 200 ms while nothing arrives
    /// must accumulate that silence rather than reset it. It is observer-clock
    /// arithmetic compared only against the observer's clock — the module docs'
    /// clock rule is untouched.
    last_nonempty_drain_at_ns: Option<u64>,
    /// True from the instant a tap attaches until the first drain that
    /// both yielded frames and left the queue empty — the window during which
    /// what we are draining may still be the publisher's retained history.
    ///
    /// While it is open, batches RAISE [`Self::max_stamp_ns`] but never date the
    /// topic. It is what stops the second chunk of one flush from out-ranking the first
    /// (which would date a dead route), and it re-arms on every attach because
    /// every fresh connection can be flushed again.
    baseline_open: bool,
    /// Observer-clock ns at which the CURRENT pending-epoch run opened, or
    /// `None` when no run is open.
    ///
    /// A run is the consecutive frame-yielding batches that have all stayed
    /// STRICTLY BELOW [`Self::max_stamp_ns`] while the baseline was CLOSED, were
    /// all seen with exactly ONE local publisher, and never dropped below the run's
    /// own first stamp — the signature of a publisher that restarted onto a lower
    /// clock and kept streaming. It is the evidence behind the SECOND reset path,
    /// which exists because the silence gate is unsatisfiable for exactly the
    /// high-rate topics a missed reset hurts most (module docs' second-reset-path section).
    ///
    /// CLEARED by anything that refutes the reading outright: a batch at or above
    /// the maximum (an advance, an EQUAL stamp — which is what a re-flush's own
    /// tail carries — or an unstamped batch), an OPEN baseline (which is also what
    /// covers a fresh attach), and an epoch reset of either kind. RESTARTED at the
    /// batch in front of us when a premise stops holding — see
    /// [`Self::note_regression_run`], which records what a poisoned run
    /// measures.
    regression_run_since_ns: Option<u64>,
    /// The publisher stamp of the run's FIRST batch. The run must ASCEND
    /// past it before it counts as a live new epoch rather than a stalled one, and
    /// a batch stamped BELOW it RESTARTS the run rather than being measured against
    /// a bar that belongs to an epoch already dead.
    regression_run_first_stamp_ns: u64,
    /// How many of the run's drains left the subscriber queue EMPTY — the
    /// structural, burst-killing condition (see
    /// [`SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS`]).
    regression_run_emptied_drains: u32,
}

impl LivenessRecord {
    /// Total observation time as of `now_ns`, closed and open intervals summed.
    /// Saturating: a clock that steps backwards yields 0 for the open interval
    /// rather than a wrapped absurdity.
    fn observed_for_ns(&self, now_ns: u64) -> u64 {
        let open = self
            .observing_since_ns
            .map_or(0, |since| now_ns.saturating_sub(since));
        self.observed_accum_ns.saturating_add(open)
    }

    /// The wire view as of `now_ns`, or `None` (UNKNOWN) when NOTHING IS
    /// OBSERVING this topic right now.
    ///
    /// The rule is "is anything watching", NOT "has anything ever been watched"
    /// and NOT "has any time elapsed":
    ///
    /// * A tap that attached THIS instant on a topic with NO banked history
    ///   reports `Some` with `observed_for_ms: 0` and `frames_observed: 0`, which
    ///   [`TopicLiveness::state`] classifies [`LivenessState::Unknown`] — so it
    ///   renders like a genuine UNKNOWN while the record still states that
    ///   observation is underway. (A RE-attach reports whatever its banked
    ///   history earned, which is the point of keeping it.)
    /// * A topic observed EARLIER whose tap has since been LOST (a drain failure
    ///   dropped it; the gateway handed observation to an egress attach that then
    ///   failed; the budget reclaimed it) reports `None`. This matters: without
    ///   it the last verdict FREEZES and keeps being served with growing
    ///   confidence — a topic observed for 30 frameless seconds would report
    ///   [`LivenessState::NoData`] forever after the tap died, and one that had
    ///   streamed would report an ever-growing `Idle` age. Neither is something
    ///   the robot still knows. `None` says so.
    /// * The banked history (`observed_accum_ns`, `frames_observed`,
    ///   `last_frame_at_ns`, `max_stamp_ns`) is KEPT across the gap, so a
    ///   re-attach RESUMES the observation rather than restarting it. Keeping the
    ///   stamp maximum is what makes a backlog re-flushed into the NEXT connection
    ///   unable to advance anything.
    ///
    /// PURE.
    pub fn snapshot(&self, now_ns: u64) -> Option<TopicLiveness> {
        // Not currently observing ⇒ nothing to report. (`?` also covers the
        // never-observed record: its `observing_since_ns` is `None` too.)
        self.observing_since_ns?;
        let observed_for_ns = self.observed_for_ns(now_ns);
        let base = TopicLiveness {
            last_frame_age_ms: self
                .last_frame_at_ns
                .map(|at| now_ns.saturating_sub(at) / 1_000_000),
            observed_for_ms: observed_for_ns / 1_000_000,
            frames_observed: self.frames_observed,
            rate_estimate: None,
        };
        Some(TopicLiveness {
            rate_estimate: self.servable_rate_estimate(&base, now_ns),
            ..base
        })
    }

    /// The rate estimate this record may SERVE right now, or `None`.
    ///
    /// Two independent gates, both required, and they are not redundant:
    ///
    /// * The topic must classify [`LivenessState::Streaming`] — a rate is a claim
    ///   about what is happening NOW, and every other class is a claim that it
    ///   serving the last one for as long as the robot stays up.
    /// * The window that produced the number must have closed within
    ///   [`RATE_ESTIMATE_MAX_AGE_NS`] — because "the topic is streaming" and "we
    ///   have a current measurement of how fast" can genuinely come apart. A
    ///   topic whose sequence keeps regressing (two publishers interleaving on a
    ///   `multi_publisher_topics` topic) re-anchors its window on nearly every
    ///   drain and closes one rarely or never, while its frames still date it
    ///   `Streaming` on every sweep. Without this gate that topic would serve
    ///   whatever number it last managed, indefinitely.
    ///
    /// PURE.
    fn servable_rate_estimate(
        &self,
        base: &TopicLiveness,
        now_ns: u64,
    ) -> Option<TopicRateEstimate> {
        if base.state() != LivenessState::Streaming {
            return None;
        }
        let at = self.rate_estimate_at_ns?;
        if now_ns.saturating_sub(at) > RATE_ESTIMATE_MAX_AGE_NS {
            return None;
        }
        self.rate_estimate
    }

    /// Begin (or continue) an observation interval at `now_ns`. Idempotent — a
    /// second call while already observing does NOT restart the interval, so the
    /// per-pass "still observing" affirmation the gateway makes is free.
    fn start_observing(&mut self, now_ns: u64) {
        if self.observing_since_ns.is_none() {
            self.observing_since_ns = Some(now_ns);
        }
    }

    /// Begin an observation interval that a FRESHLY ATTACHED subscriber port
    /// backs — the observer's own tap, or the gateway's egress tap taking over.
    ///
    /// Identical to [`Self::start_observing`] except that it RE-OPENS the
    /// baseline: the burst this fresh connection is about to receive may be the
    /// publisher's retained history flushed into it (measured — see the module
    /// docs), so no batch in that burst may date the topic.
    ///
    /// [`Self::max_stamp_ns`] is deliberately NOT reset — carrying it across the
    /// attach is exactly what makes a re-flush of the same backlog a no-op.
    ///
    /// The pending-epoch run is discarded too, and NOT by a second line
    /// here. Re-opening the baseline is already sufficient: the run's own
    /// open-baseline filter ([`Self::note_regression_run`]) clears it at the first
    /// frame-yielding batch of the new connection, which is necessarily the first
    /// batch after an attach. An explicit clear here would be a redundant conjunct
    /// with no observable difference, so the one mechanism carries it. The cost —
    /// stated because it is a real one — is that a hand-off DURING a restart's
    /// regression regime restarts the confirmation window; the cost of the reverse
    /// would be letting a fresh flush's prefix chunks finish a confirmation the
    /// previous connection had started.
    fn begin_attached_observation(&mut self, now_ns: u64) {
        self.start_observing(now_ns);
        self.baseline_open = true;
    }

    /// End the current observation interval at `now_ns`, banking its duration.
    /// Idempotent.
    fn stop_observing(&mut self, now_ns: u64) {
        if let Some(since) = self.observing_since_ns.take() {
            self.observed_accum_ns = self
                .observed_accum_ns
                .saturating_add(now_ns.saturating_sub(since));
        }
    }

    /// Record one drain's worth of evidence, taken at observer-clock `now_ns`.
    ///
    /// A zero-frame drain is a genuine observation of ABSENCE: it banks nothing,
    /// does NOT touch `last_frame_at_ns` (that is what makes the age grow while a
    /// topic is silent), and does NOT close the baseline — a retained-history
    /// flush can arrive at ANY later time (it rides the publisher's next
    /// `update_connections`, not our drain schedule), so an empty queue is no
    /// evidence that the burst already happened.
    ///
    /// Otherwise the frames are banked and the batch is judged by ADVANCEMENT
    /// (module docs): its newest publisher stamp must EXCEED every stamp this
    /// record has ever seen, which proves the publisher committed a frame between
    /// the two drains. Until the baseline burst ends
    /// ([`DrainObservation::queue_emptied`]) no batch may date the topic at all;
    /// it only raises the maximum.
    ///
    /// The comparison is STRICT (`>`) and unstamped batches never advance — both
    /// conservative in the false-live direction, and both cost at most "this batch
    /// does not date the topic", never permanent deafness. (An unstamped batch
    /// that is the FIRST ever seen costs one batch MORE than that, because it
    /// leaves `max_stamp_ns` unset and the next stamped batch then has nothing to
    /// advance past; it is still bounded, and still not deafness.)
    ///
    /// A batch stamped BELOW the maximum is the one case that is not merely "no
    /// evidence": it MAY be EVIDENCE OF A NEW PUBLISHER EPOCH (module docs' restart
    /// section), because one writer's one clock cannot go backwards within a run.
    /// Such a batch resets the maximum to its own stamp and re-opens the baseline —
    /// it never dates the topic, and the next batch that advances past the new
    /// maximum does. But it may EQUALLY be a later chunk of one re-flushed backlog,
    /// which is stamp-for-stamp indistinguishable, so the reset requires a CLOSED
    /// baseline plus one of two independent readings of the regression: the
    /// observer has been silent for at least [`REGRESSION_RESET_MIN_GAP_NS`], OR
    /// the regression is SUSTAINED — a run of below-the-bar batches that
    /// a retained-history burst cannot produce ([`Self::note_regression_run`]).
    /// Otherwise the regression is BANKED without lowering the maximum. The trade
    /// is stated in the module docs' residual list.
    ///
    /// The same evidence also feeds the RATE window
    /// ([`Self::record_rate_window`]), which is deliberately a separate step over
    /// the same observation — the dating rule above answers "did the publisher
    /// commit anything while we watched", the rate answers "how many, how fast",
    /// and only the first may touch [`Self::last_frame_at_ns`].
    fn record_frames(&mut self, obs: DrainObservation, now_ns: u64) {
        if obs.frames == 0 {
            return;
        }
        // Captured BEFORE the dating rule can flip it: a drain that arrived
        // during a baseline burst is absorbing what may be the publisher's
        // retained history, whose frames are OLD however new they look. Counting
        // them toward a rate would report a burst on a route that has been dead
        // since the flush, so such a drain may only RE-ANCHOR the rate window.
        let baseline_at_entry = self.baseline_open;
        let epoch_reset = self.record_frames_dating(obs, now_ns);
        self.record_rate_window(obs, now_ns, baseline_at_entry || epoch_reset);
    }

    /// The DATING half of [`Self::record_frames`] — the advancement rule and its
    /// two guards, unchanged. Returns whether the EPOCH RESET arm fired, which
    /// the caller needs because a reset batch may be the new run's own retained
    /// history and must not be counted toward a rate either.
    fn record_frames_dating(&mut self, obs: DrainObservation, now_ns: u64) -> bool {
        // Observer-clock silence since the last drain that yielded frames, read
        // BEFORE this drain re-anchors it. `None` (nothing has ever arrived) is
        // unbounded silence — and cannot regress anyway, since there is no maximum
        // to regress from.
        let quiet_long_enough = self
            .last_nonempty_drain_at_ns
            .is_none_or(|last| now_ns.saturating_sub(last) >= REGRESSION_RESET_MIN_GAP_NS);
        self.last_nonempty_drain_at_ns = Some(now_ns);
        self.frames_observed = self.frames_observed.saturating_add(obs.frames);
        // Publisher stamp vs publisher stamp — the observer's clock never enters
        // this. A first-ever stamp has nothing to advance past OR regress from, so
        // it is a baseline, not evidence; an unstamped batch proves neither.
        let (advanced, regressed) = match (obs.newest_stamp_ns, self.max_stamp_ns) {
            (Some(ts), Some(seen)) => (ts > seen, ts < seen),
            (Some(_), None) | (None, _) => (false, false),
        };
        // Fold this batch into the pending-epoch run BEFORE the decision,
        // so the batch under judgement counts toward its own confirmation. Reads
        // `self.baseline_open` at its pre-batch value, exactly as guard 1 does.
        let sustained = self.note_regression_run(&obs, regressed, now_ns);
        // Whether the EPOCH RESET arm below fired — reported to
        // `record_frames` so the rate window re-anchors on the new epoch instead
        // of counting the dead run's stamps against the new run's.
        let mut epoch_reset = false;
        if regressed && !self.baseline_open && (quiet_long_enough || sustained) {
            // EPOCH RESET: the publisher restarted (its clock began again below
            // the dead run's high-water mark). ADOPT the new epoch's stamp — the
            // old maximum is meaningless against it and keeping it would refuse to
            // date this topic for the length of the previous run — and RE-OPEN the
            // baseline, because this batch may be the NEW run's own retained
            // history flushing into us. It therefore does not date the topic; the
            // next batch advancing past this stamp does.
            //
            // BOTH GUARDS ARE LOAD-BEARING, and they close DIFFERENT halves of the
            // same hole — a re-flush of one backlog into a fresh connection, whose
            // chunks are a strict PREFIX of it and therefore regress. Adopting one
            // LOWERS the maximum, after which a later chunk of that SAME backlog
            // out-ranks it and dates a route that has been dead since the first
            // flush — the exact hole the carried maximum exists to close, re-opened
            // on every hand-off and re-attach.
            //
            // * `!baseline_open` covers the chunks that arrive while the burst is
            //   still provably underway — and it is the ONLY guard standing when a
            //   re-flush's FIRST chunk lands after a real LULL, which a hand-off or
            //   a slot-recycled re-attach on an idle robot produces routinely:
            //   `quiet_long_enough` is then satisfied outright, and without this
            //   conjunct that prefix chunk would floor the bar so the next chunk of
            //   the SAME backlog dates a dead route. Pinned by
            //   `a_re_flush_after_a_lull_cannot_lower_the_bar_while_the_baseline_is_open`.
            // * `quiet_long_enough` covers the ones that do NOT: the burst's FIRST
            //   chunk can leave the queue momentarily empty (the mid-flush
            //   interleave residual — `deliver_history` is still pushing), which
            //   CLOSES the baseline mid-burst and hands the next chunk a closed
            //   one. A drain cadence apart is not silence, so that chunk is banked
            //   too, and the maximum the first flush established still out-ranks
            //   every chunk behind it.
            //
            //
            // The `|| sustained` disjunct sits on the SECOND conjunct,
            // because `quiet_long_enough` is UNSATISFIABLE for a topic that keeps
            // streaming across a lull-free restart — the drains that would have to
            // fall silent are the restarted run's own. `sustained` reads the
            // regression REGIME instead (see `note_regression_run`), and it is
            // deliberately NOT a third conjunct: the two are alternative readings of
            // the same question, and requiring both would leave the high-rate topics
            // exactly where the silence gate alone left them.
            //
            // A regression that fails EITHER remaining guard is BANKED without
            // adopting (the `else if` below keeps the higher `seen`).
            self.max_stamp_ns = obs.newest_stamp_ns;
            self.baseline_open = true;
            epoch_reset = true;
            // The reading has been acted on; the evidence behind it must not also
            // authorize the NEXT regression against the new epoch.
            self.clear_regression_run();
        } else if let Some(ts) = obs.newest_stamp_ns {
            self.max_stamp_ns = Some(self.max_stamp_ns.map_or(ts, |seen| seen.max(ts)));
        }
        if self.baseline_open {
            // Still absorbing the burst this attach (or this epoch reset) may have
            // been flushed. The maximum above has already been set, so nothing in
            // the rest of this burst can date the topic either.
            self.baseline_open = !obs.queue_emptied;
            return epoch_reset;
        }
        if advanced {
            self.last_frame_at_ns = Some(now_ns);
        }
        epoch_reset
    }

    /// Fold one frame-yielding drain into the PENDING-EPOCH run and report
    /// whether the run now amounts to a SUSTAINED regression — the second, silence-free
    /// reading of "the publisher restarted" (module docs' second-reset-path section).
    ///
    /// `regressed` is the caller's already-computed verdict for this batch against
    /// [`Self::max_stamp_ns`], so this method never re-derives it and the two cannot
    /// disagree.
    ///
    /// The run is CLEARED — not merely left unextended — by every observation that
    /// refutes the reading outright:
    ///
    /// * **Not regressed.** An ADVANCE means the publisher is back above the bar and
    ///   there is nothing to reset. An EQUAL stamp is the decisive one: it is exactly
    ///   what a re-flush's own TAIL carries (the frame that set the maximum is in the
    ///   retained history), so a backlog terminates its own run. An UNSTAMPED batch
    ///   proves nothing either way and is treated the same.
    /// * **An OPEN baseline.** While it is open we are provably absorbing a burst
    ///   whose frames may be retained history, so nothing observed there is evidence
    ///   of a new epoch. This is guard 1's reasoning applied to the evidence rather
    ///   than to the decision. It is also what covers a fresh ATTACH: the first
    ///   frame-yielding batch of a new connection necessarily lands here.
    /// * **A NON-SINGLE-WRITER reading** ([`DrainObservation::writers_seen`] other
    ///   than `Some(1)`, an UNKNOWN count included). Such a batch contributes
    ///   NOTHING — not a timestamp, not an ascent bar, not an emptied credit — and
    ///   the next clean batch anchors a fresh run.
    ///
    ///   A sticky `trusted &= …`, re-armed only at a run OPEN, would not do.
    ///   In THE scenario this path exists for, a run has no exit — every batch stays
    ///   below the dead run's high-water mark and the baseline stays closed — so one
    ///   bad reading would disable healing for the rest of the regime: MEASURED at **500
    ///   further flawless single-writer drains (100 s, 100× the confirmation span)
    ///   with the maximum still on the dead run's 2 h stamp**, silently
    ///   re-instating the exact defect this path removes. (The rate window's
    ///   `rate_window_trusted` is NOT a precedent: that bit re-arms at
    ///   every anchor and its windows close every 2 s, so one bad reading there
    ///   costs ONE window.) Making such a batch RESTART the run instead
    ///   would cure the permanence but leave the ANCHORING drain exempt from the
    ///   very premise condition 1 states, so a foreign writer could set the span
    ///   clock, supply the ascent bar from ITS stamp, and donate one of the two
    ///   emptied credits. Clearing instead is strictly stricter (the span starts one
    ///   drain later) and makes condition 1 TRUE AS WRITTEN rather than true with a
    ///   caveat — the run really is all-single-writer. The cost is one drain cadence
    ///   of extra recovery time in the rare case a dissent lands mid-regime:
    ///   ~200 ms on the observer plane against a 1 s span.
    ///
    /// And it RESTARTS — begins again AT THIS BATCH, rather than continuing — when
    /// **a stamp DROPS below the run's own first**. The ascent bar is frozen at the
    /// run's first stamp, so a SECOND restart inside an open run would leave it belonging to
    /// an epoch that is already dead — the same cross-epoch stamp comparison this
    /// module forbids everywhere else. MEASURED with a run that continues instead: a genuinely
    /// LIVE, single-writer, queue-emptying producer that crash-loops (each
    /// incarnation publishing below the run's first stamp) never heals across **200
    /// sweeps (40 s)**. A within-run stamp DROP is itself evidence of a further
    /// epoch, so it re-anchors the run rather than being no event at all. Restarting
    /// is strictly STRICTER than extending: the span, the emptied count and the
    /// ascent bar all begin again.
    ///
    /// A topic with two CONCURRENTLY-ATTACHED local writers is excluded by
    /// construction: it reports ≥ 2 on essentially every drain, so no run ever opens.
    /// (The gate is a per-drain LOCAL PORT count, so a window in which one writer is
    /// detached is admitted — the module docs' condition 1 states that scope.)
    ///
    /// There is deliberately NO `single_writer` conjunct in the verdict below: a
    /// batch that is not single-writer returned early, so every batch that reaches
    /// the verdict — and every batch folded into the run behind it — passed that
    /// test. Writing the conjunct as well would be a term with no observable
    /// difference.
    ///
    /// PURE. Returns `false` in every cleared and every restarted case, so a caller
    /// can use it as the whole of the sustained reading.
    fn note_regression_run(
        &mut self,
        obs: &DrainObservation,
        regressed: bool,
        now_ns: u64,
    ) -> bool {
        // Every premise that disqualifies a batch OUTRIGHT is tested here, before
        // anything can be recorded from it — which is what makes "every drain in the
        // run saw exactly one publisher" a fact about the run rather than about its
        // extensions only.
        let Some(stamp) = obs
            .newest_stamp_ns
            .filter(|_| regressed && !self.baseline_open && obs.writers_seen == Some(1))
        else {
            self.clear_regression_run();
            return false;
        };
        let since = match self.regression_run_since_ns {
            // EXTEND: a run is open and this batch has not dropped below its bar.
            Some(since) if stamp >= self.regression_run_first_stamp_ns => since,
            // RESTART AT THIS BATCH: no run was open, or the stamp dropped below the
            // run's own first. Everything the verdict reads begins again.
            _ => {
                self.regression_run_since_ns = Some(now_ns);
                self.regression_run_first_stamp_ns = stamp;
                self.regression_run_emptied_drains = 0;
                now_ns
            }
        };
        if obs.queue_emptied {
            self.regression_run_emptied_drains =
                self.regression_run_emptied_drains.saturating_add(1);
        }
        self.regression_run_emptied_drains >= SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS
            && now_ns.saturating_sub(since) >= SUSTAINED_REGRESSION_MIN_SPAN_NS
            // A run that has not ASCENDED is a publisher standing still, not a live
            // new epoch. The comparison is against the run's FIRST stamp, so the
            // ascent must have happened inside the run being judged.
            && stamp > self.regression_run_first_stamp_ns
    }

    /// Forget the pending-epoch run. Called wherever the evidence is
    /// refuted or spent — see [`Self::note_regression_run`] and the reset arm.
    ///
    /// Only `regression_run_since_ns` is behaviourally load-bearing (it is what
    /// makes the next run OPEN rather than EXTEND, and every other field is
    /// re-initialized at that open). The rest are cleared so that the derived
    /// `PartialEq` on this record cannot depend on spent history that no longer
    /// affects a single verdict.
    fn clear_regression_run(&mut self) {
        self.regression_run_since_ns = None;
        self.regression_run_first_stamp_ns = 0;
        self.regression_run_emptied_drains = 0;
    }

    /// Fold one frame-yielding drain into the rate window, closing +
    /// publishing the window when it has covered [`RATE_ESTIMATE_MIN_WINDOW_NS`].
    ///
    /// `anchor_only` is the caller's declaration that this drain's frames may be
    /// the publisher's RETAINED HISTORY rather than fresh production — a baseline
    /// burst, or the first batch of a new publisher epoch. Such a drain resets the
    /// window to start HERE and contributes nothing to any rate, which is what
    /// stops a dead route's flushed backlog from being rendered as a live stream.
    ///
    /// The anchoring drain's own frames are never counted, in either basis: they
    /// arrived BEFORE the window opened, and belong to whatever window preceded
    /// it. Consecutive windows share their boundary drain for exactly that
    /// reason, which is also what cancels the drain-lag bias out of the
    /// denominator (see [`RATE_ESTIMATE_MIN_WINDOW_NS`]).
    ///
    /// PURE.
    fn record_rate_window(&mut self, obs: DrainObservation, now_ns: u64, anchor_only: bool) {
        let Some(anchor_at) = self.rate_anchor_at_ns.filter(|_| !anchor_only) else {
            self.anchor_rate_window(&obs, now_ns);
            return;
        };
        // The sequence basis is meaningful only for a SINGLE writer, and
        // the evidence is per-drain. One multi-writer observation anywhere in the
        // window disqualifies its sequence delta for good — the maximum has
        // already hopped between counters by then.
        self.rate_window_trusted &= obs.writers_seen == Some(1);
        // A sequence far BELOW the anchor cannot be counted against it: the
        // publisher restarted (its counter begins again at 0) or the `u32`
        // wrapped. Either way the window's numerator is meaningless, so the
        // window is discarded and re-anchored on the new epoch rather than
        // reporting a wrapped or negative-looking rate. A SMALL backward step is
        // a benign reorder and is left to the saturating delta below.
        if let (Some(anchor_seq), Some(seq)) = (self.rate_anchor_seq, obs.newest_sequence) {
            if anchor_seq.saturating_sub(seq) > RATE_SEQ_RESET_TOLERANCE {
                self.anchor_rate_window(&obs, now_ns);
                return;
            }
        }
        let elapsed = now_ns.saturating_sub(anchor_at);
        // A window that has stayed open longer than the staleness horizon SPANS A
        // SILENCE, and its average is not a rate anybody wants. The window is a
        // MINIMUM span with no maximum, and nothing advances it while a topic is
        // quiet (a zero-frame drain returns before ever reaching here), so a topic
        // that pauses and resumes would otherwise close ONE window covering the
        // whole pause: measured, a 100 Hz topic silent for a minute then resuming
        // reports 0.33 Hz — a confidently wrong number, on a row the classifier
        // calls Streaming, for as long as it takes the next window to close.
        //
        // So such a window is DISCARDED rather than closed, and the resuming drain
        // anchors a fresh one. The horizon is the same
        // [`RATE_ESTIMATE_MAX_AGE_NS`] the serve gate uses, which is not a
        // coincidence: a window longer than it necessarily contains a stretch
        // during which the topic was not streaming at all, so the two rules draw
        // the same boundary. The cost is that a topic slower than that horizon
        // gets no rate, and that cost is real: the serve gate and the horizon are
        // the SAME 5 s, so a period-`P` topic still reads `Streaming` for
        // `min(5s, P) / P` of its period — 96 % at P = 5.2 s, 83 % at P = 6 s.
        // Such a topic looks live nearly all the time while carrying no rate; only
        // above P = 10 s is it genuinely not-Streaming most of the time. The
        // uncomfortable band is 5 s < P < 10 s, accepted rather than free
        // (widening the horizon just re-admits the pause into the average).
        if elapsed > RATE_ESTIMATE_MAX_AGE_NS {
            self.anchor_rate_window(&obs, now_ns);
            return;
        }
        self.rate_window_frames = self.rate_window_frames.saturating_add(obs.frames);
        if elapsed < RATE_ESTIMATE_MIN_WINDOW_NS {
            return;
        }
        self.rate_estimate = Some(self.close_rate_window(obs.newest_sequence, elapsed));
        self.rate_estimate_at_ns = Some(now_ns);
        // The closing drain opens the next window, so the two share an endpoint.
        self.anchor_rate_window(&obs, now_ns);
    }

    /// Start a fresh rate window at `now_ns`, counting from this drain's newest
    /// sequence and RE-ARMING the single-writer trust bit from its own evidence
    /// — a window is judged on the writers present while it runs.
    fn anchor_rate_window(&mut self, obs: &DrainObservation, now_ns: u64) {
        self.rate_anchor_seq = obs.newest_sequence;
        self.rate_anchor_at_ns = Some(now_ns);
        self.rate_window_frames = 0;
        self.rate_window_trusted = obs.writers_seen == Some(1);
    }

    /// Turn one closed window into a rate. PURE.
    ///
    /// The EXACT basis is the publisher's committed-sequence delta, which no tap
    /// depth can clip. It is used only when the sequence genuinely ADVANCED: a
    /// window in which frames arrived while the counter stood still is not
    /// evidence of a 0 Hz topic, it is evidence that this publisher's sequence
    /// cannot be counted on (`publish_raw` writes the caller's header verbatim),
    /// and the correct answer there is the FLOOR — the frames the observer itself
    /// drained, labelled as a floor.
    ///
    /// `u128` for the intermediate: a full `u32` sequence delta times the
    /// nanosecond-to-millihertz factor overflows `u64` by three orders.
    fn close_rate_window(&self, seq: Option<u32>, elapsed_ns: u64) -> TopicRateEstimate {
        // An UNTRUSTED window (any drain saw other than exactly one
        // publisher) has no usable sequence delta — its maximum hopped between
        // per-publisher counters — so it falls to the FLOOR basis, which counts
        // frames the observer really drained and says so.
        let advanced = match (self.rate_anchor_seq, seq) {
            (Some(anchor), Some(now)) if self.rate_window_trusted => now.saturating_sub(anchor),
            _ => 0,
        };
        let (count, is_floor) = if advanced > 0 {
            (u64::from(advanced), false)
        } else {
            (self.rate_window_frames, true)
        };
        let millihertz = (u128::from(count) * 1_000 * 1_000_000_000)
            .checked_div(u128::from(elapsed_ns))
            .unwrap_or(0)
            .min(u128::from(u64::MAX)) as u64;
        TopicRateEstimate {
            millihertz,
            is_floor,
        }
    }
}

/// The shared snapshot table: canonical topic → its observation record. The
/// observer (on the gateway drive thread) is the sole writer; the catalog serve
/// (on the zenoh callback thread) reads it. Cheap to clone (one `Arc`).
pub type LivenessTable = Arc<Mutex<HashMap<String, LivenessRecord>>>;

/// Read one topic's liveness out of a shared table as of `now_ns`. `None` for an
/// untracked topic or a record that knows nothing — UNKNOWN either way.
pub fn read_liveness(table: &LivenessTable, topic: &str, now_ns: u64) -> Option<TopicLiveness> {
    table
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .get(topic)
        .and_then(|r| r.snapshot(now_ns))
}

/// The [`LIVENESS_ENV`] decision. PURE (oracle-tested): `Some("off")` disables;
/// absent or empty leaves observation on silently; anything else leaves it on
/// and asks the caller to complain loudly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LivenessSwitch {
    /// Observe (the default).
    On,
    /// Explicitly disabled by `CERULION_TOPIC_LIVENESS=off`.
    Off,
    /// An unrecognized value — observation stays ON and the caller warns.
    UnrecognizedStaysOn,
}

/// Classify a raw [`LIVENESS_ENV`] value. Exact match, no trimming and no case
/// forgiveness (a typo must be LOUD, not silently honoured as something else).
pub fn liveness_switch(raw: Option<&str>) -> LivenessSwitch {
    match raw {
        None | Some("") => LivenessSwitch::On,
        Some("off") => LivenessSwitch::Off,
        Some(_) => LivenessSwitch::UnrecognizedStaysOn,
    }
}

/// Resolve [`LIVENESS_ENV`] from the process environment, warning once on an
/// unrecognized value.
pub fn liveness_enabled_from_env() -> bool {
    let raw = std::env::var(LIVENESS_ENV).ok();
    match liveness_switch(raw.as_deref()) {
        LivenessSwitch::On => true,
        LivenessSwitch::Off => {
            tracing::info!(
                env = LIVENESS_ENV,
                "topic data-flow liveness observation DISABLED — the catalog will serve \
                 no per-topic liveness and the desk sidebar degrades to \
                 publisher-presence rendering"
            );
            false
        }
        LivenessSwitch::UnrecognizedStaysOn => {
            tracing::warn!(
                env = LIVENESS_ENV,
                value = %raw.unwrap_or_default(),
                "unrecognized value for the topic-liveness switch (the only accepted \
                 value is exactly `off`) — liveness observation stays ENABLED"
            );
            true
        }
    }
}

// ---------------------------------------------------------------------------
// The observer.
// ---------------------------------------------------------------------------

/// Per-topic bookkeeping the observer keeps OUTSIDE the shared table (so the
/// catalog serve never locks over it).
#[derive(Debug, Default)]
struct TopicObservation {
    /// Cumulative tap-attach failures (Principle #3 — a topic that can never be
    /// observed must be visible, not silently UNKNOWN forever).
    attach_failures: u64,
    /// Once-per-regime latch for the attach-failure warn, re-armed by a success.
    attach_warned: bool,
    /// Cumulative drain failures.
    drain_failures: u64,
    /// Once-per-regime latch for the drain-failure warn.
    drain_warned: bool,
}

/// A long-lived, budget-bounded set of listener-less taps that turns "a
/// publisher exists" into "frames actually crossed". See the module docs for the
/// design rationale, the measured reason a serve-time peek cannot work, and the
/// degradation table.
///
/// Drive it with [`Self::sweep`] from an existing loop; it throttles itself to
/// [`LIVENESS_SWEEP_INTERVAL_NS`].
pub struct TopicLivenessObserver {
    manager: Arc<TransportManager>,
    /// The shared table the catalog serve reads.
    table: LivenessTable,
    /// Canonical topic → the observer's OWN tap. Absent for a topic that is
    /// externally observed (an egress tap is draining it), over budget, or whose
    /// attach has not succeeded yet.
    taps: HashMap<String, DataOnlySubscriber>,
    /// Canonical topics the observer has been asked to watch, in insertion
    /// order-independent form. A topic is tracked before it is tapped.
    tracked: HashSet<String>,
    /// `tracked` in CANONICAL (sorted) order — the order [`Self::sweep`] walks,
    /// and therefore the order in which a bounded tap budget is handed out
    /// WITHIN one sweep.
    ///
    /// Sorted because the budget makes the walk order OBSERVABLE: with more
    /// tracked topics than budget, whoever is visited first gets the taps and
    /// everyone else reports UNKNOWN. Iterating the `HashSet` made that
    /// assignment vary run to run (and even process to process, under
    /// `RandomState`), so the same robot could report a different subset of its
    /// topics on every restart. Sorted order makes it reproducible and
    /// explainable.
    ///
    /// It does NOT, on its own, make the observed SUBSET the canonically-first
    /// `budget` topics, and [`Self::ensure_tap`]'s operator warn does not claim
    /// it does. Taps are never PREEMPTED: nothing takes a tap away from an
    /// incumbent to give it to a lower-sorting topic, so once the budget is full
    /// a topic tracked LATER never displaces one, however early it sorts. The two
    /// coincide only when the whole tracked set is known before the first sweep —
    /// which is the boot case (the gateway `track_all`s its announced set) but NOT
    /// the `cerulion ros2 attach` case, where topics are registered as DDS
    /// discovery finds them and the first observed subset is therefore
    /// arrival-ordered. Re-ranking with eviction was considered and rejected: it
    /// would churn taps (each eviction loses that topic's observation and
    /// re-opens the advancement baseline on the topic that takes the slot) to
    /// reorder a set that is already accurately reported.
    ///
    /// Slots are nonetheless RECYCLED rather than owned for life:
    /// [`Self::release_own_tap`] (a remote demand's egress drain taking over),
    /// [`Self::set_externally_observed`] and a drain failure all free one, and
    /// because this walk is sorted the freed slot goes to the canonically-first
    /// tracked topic that has no tap — which may well be one refused earlier. The
    /// observed subset is therefore stable but not fixed, and the operator warn
    /// says so.
    ///
    /// Reused across sweeps and rebuilt only when `tracked` GROWS — which is the
    /// only way it can change, since [`Self::track`] is the sole mutator and
    /// never removes — so the steady-state sweep allocates nothing. This is the
    /// observer's ONE per-sweep-path allocation site.
    sweep_order: Vec<String>,
    /// Canonical topics whose frames are being drained by SOMEONE ELSE (the
    /// gateway's egress tap). The observer never attaches its own tap for these
    /// — one gateway port per topic, never two.
    externally_observed: HashSet<String>,
    /// Per-topic failure accounting.
    obs: HashMap<String, TopicObservation>,
    /// Reusable drain buffer — one allocation, reused every sweep.
    drain_buf: Vec<OwnedInboundSample>,
    /// Max concurrent taps.
    budget: usize,
    /// Observer-clock ns of the last executed sweep. `None` = never swept, kept
    /// distinct from a sweep AT time 0 (which a `VirtualClock` really can produce)
    /// so the throttle cannot be silently defeated at the clock's origin.
    last_sweep_ns: Option<u64>,
    /// Cumulative executed sweeps (Principle #3).
    sweeps: u64,
    /// Whether observation is enabled at all.
    enabled: bool,
    /// Once-only latch for the budget-exhausted warn.
    budget_warned: bool,
}

impl TopicLivenessObserver {
    /// Build an observer over `manager`, enabled per [`LIVENESS_ENV`].
    pub fn new(manager: Arc<TransportManager>) -> Self {
        Self::with_budget(
            manager,
            DEFAULT_LIVENESS_TAP_BUDGET,
            liveness_enabled_from_env(),
        )
    }

    /// Build an observer with an explicit tap budget and enable flag (the test
    /// seam and the construction point for a caller that resolves the switch
    /// itself).
    // hot-path-alloc-ok-fn: cold: the observer is CONSTRUCTED once, at gateway boot; these are
    // its empty tap/observation tables
    pub fn with_budget(manager: Arc<TransportManager>, budget: usize, enabled: bool) -> Self {
        Self {
            manager,
            // hot-path-alloc-ok: cold — one table per gateway, built at boot.
            table: Arc::new(Mutex::new(HashMap::new())),
            taps: HashMap::new(),
            tracked: HashSet::new(),
            // hot-path-alloc-ok: cold — the reusable canonical sweep order,
            // (re)built only when the tracked set grows, never per sweep.
            sweep_order: Vec::new(),
            externally_observed: HashSet::new(),
            obs: HashMap::new(),
            // hot-path-alloc-ok: cold — the reusable sweep drain buffer, allocated
            // once per observer at boot and reused by every sweep thereafter.
            drain_buf: Vec::new(),
            budget,
            last_sweep_ns: None,
            sweeps: 0,
            enabled,
            budget_warned: false,
        }
    }

    /// The shared table handle to hand the catalog serve. Cheap (one `Arc`).
    pub fn table(&self) -> LivenessTable {
        Arc::clone(&self.table)
    }

    /// Whether observation is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Begin watching `topic` (idempotent, allocation-free once tracked). The
    /// tap itself attaches on the next [`Self::sweep`], so a topic that has just
    /// been tracked correctly reports UNKNOWN rather than "no data".
    pub fn track(&mut self, topic: &str) {
        if !self.enabled || self.tracked.contains(topic) {
            return;
        }
        // hot-path-alloc-ok: cold — once per topic, at registration.
        self.tracked.insert(topic.to_string());
    }

    /// Track every topic in `topics` (the gateway's registered set).
    pub fn track_all<'a>(&mut self, topics: impl IntoIterator<Item = &'a str>) {
        for t in topics {
            self.track(t);
        }
    }

    /// Release the observer's OWN tap for `topic`, if it holds one, so a
    /// higher-priority attacher — the gateway's egress tap — gets first refusal
    /// on the subscriber slot it was occupying. Returns whether a tap was
    /// actually released.
    ///
    /// This is deliberately SEPARATE from [`Self::set_externally_observed`]: the
    /// slot must be freed BEFORE the egress attach is attempted (they compete for
    /// the same finite per-topic slots), but observation may only be declared
    /// external AFTER that attach SUCCEEDS — claiming a drainer that does not
    /// exist would bank observation time nothing is doing. Splitting them also
    /// makes the sustained-attach-failure regime free: the gateway retries every
    /// drive pass (~1 kHz) while the observer only re-attaches once per
    /// [`LIVENESS_SWEEP_INTERVAL_NS`], so on all but one of those passes this is
    /// a single hash lookup — no allocation, no table lock.
    pub fn release_own_tap(&mut self, topic: &str) -> bool {
        if !self.enabled || self.taps.remove(topic).is_none() {
            return false;
        }
        // Nothing of ours is watching until someone re-attaches: bank the
        // interval so `observed_for_ms` counts only genuinely-watched time (and,
        // per `snapshot`, the topic correctly reads UNKNOWN in the gap).
        let now = self.now_ns();
        self.with_existing_record(topic, |r| r.stop_observing(now));
        true
    }

    /// Declare whether `topic`'s frames are being drained by someone else (the
    /// gateway's egress tap). Allocation-free unless the answer CHANGED, so the
    /// gateway can affirm it every drive pass.
    ///
    /// Turning it ON begins an observation interval backed by the caller's
    /// freshly-attached port (so an idle demanded topic needs no per-pass
    /// [`Self::note_frames`] call to stay observed) and releases the observer's
    /// own tap, so a topic never costs two gateway ports. Turning it OFF banks
    /// the interval — the external drainer is gone and the observer's own tap
    /// only comes back on its next sweep.
    pub fn set_externally_observed(&mut self, topic: &str, external: bool) {
        if !self.enabled {
            return;
        }
        if external == self.externally_observed.contains(topic) {
            return;
        }
        let now = self.now_ns();
        if external {
            // hot-path-alloc-ok: cold — once per demand transition (the caller
            // declares this only after its own tap attach SUCCEEDED).
            self.externally_observed.insert(topic.to_string());
            self.taps.remove(topic);
            // The drainer's port is brand new, so its first burst may be the
            // publisher's retained history — re-open the same advancement
            // baseline the observer's own attach opens.
            self.with_record(topic, |r| r.begin_attached_observation(now));
        } else {
            self.externally_observed.remove(topic);
            // The external drainer stopped; the observer re-attaches its own tap
            // on the next sweep. Bank the interval so `observed_for_ms` counts
            // only time something was genuinely watching. `with_existing_record`:
            // a topic that was never observed must not gain an (all-default,
            // still-UNKNOWN) table entry just because demand ended.
            self.with_existing_record(topic, |r| r.stop_observing(now));
        }
    }

    /// Record a drain performed by an EXTERNAL drainer (the gateway's egress
    /// tap), so a demanded topic's liveness costs nothing beyond the drain it
    /// already performs. `obs.frames == 0` is a real observation of absence and
    /// keeps the interval open without refreshing the age.
    ///
    /// The drainer already holds the frames, so filling in
    /// [`DrainObservation::newest_stamp_ns`] is free — and it plus
    /// [`DrainObservation::queue_emptied`] are what let the advancement rule
    /// (module docs) work over the demand plane exactly as it does over the
    /// observer's own taps.
    ///
    /// A caller driving a hot loop may SKIP the call entirely when there were no
    /// frames: [`Self::set_externally_observed`] already opened the interval, an
    /// empty drain has no other effect (it deliberately does not close the
    /// baseline), and skipping it keeps an idle demanded topic off the table lock
    /// and the clock.
    pub fn note_frames(&mut self, topic: &str, obs: DrainObservation) {
        if !self.enabled || !self.externally_observed.contains(topic) {
            return;
        }
        let now = self.now_ns();
        self.with_record(topic, |r| {
            r.start_observing(now);
            r.record_frames(obs, now);
        });
    }

    /// Drain every tap the observer owns and update the table. Self-throttled to
    /// [`LIVENESS_SWEEP_INTERVAL_NS`]; returns the number of frames observed
    /// this sweep (0 when throttled out or disabled).
    ///
    /// Per-topic failures are NON-FATAL and never propagate: a transient attach
    /// or drain error on one topic must not stop the rest of the sweep, let
    /// alone the gateway's network plane. Each is counted and warned once per
    /// regime.
    pub fn sweep(&mut self) -> u64 {
        if !self.enabled {
            return 0;
        }
        let now = self.now_ns();
        // Never swept ⇒ run immediately, so a fresh gateway starts observing
        // without first waiting out an interval.
        if self
            .last_sweep_ns
            .is_some_and(|last| now.saturating_sub(last) < LIVENESS_SWEEP_INTERVAL_NS)
        {
            return 0;
        }
        self.last_sweep_ns = Some(now);
        self.sweeps += 1;

        // Walk the tracked set in CANONICAL order: the budget makes the order
        // observable (whoever is visited first gets the scarce taps), so it must
        // not be a hash-iteration accident. `mem::take` hands the reusable buffer
        // to the loop and leaves `self` free to mutate; it is put back below, so
        // no allocation happens unless the tracked set grew.
        let mut order = std::mem::take(&mut self.sweep_order);
        self.refresh_sweep_order(&mut order);
        let mut observed = 0u64;
        for topic in &order {
            if self.externally_observed.contains(topic) {
                continue;
            }
            if !self.taps.contains_key(topic) && !self.ensure_tap(topic, now) {
                continue;
            }
            observed += self.drain_topic(topic, now);
        }
        self.sweep_order = order;
        observed
    }

    /// Rebuild `order` from `tracked` — but ONLY when it is stale. `tracked` is
    /// add-only ([`Self::track`] is its sole mutator and never removes), so a
    /// length mismatch is an exact dirty-check and the steady-state sweep does no
    /// work here at all.
    fn refresh_sweep_order(&self, order: &mut Vec<String>) {
        if order.len() == self.tracked.len() {
            return;
        }
        order.clear();
        // hot-path-alloc-ok: cold — once per registration event (the tracked set
        // grew), not per sweep; this is the observer's ONE sweep-path alloc site.
        order.extend(self.tracked.iter().cloned());
        order.sort_unstable();
    }

    /// Attach the observer's own tap for `topic`. Returns whether a tap is now
    /// present. A failure is counted + warn-latched and retried next sweep; the
    /// record is NOT marked observing, so the topic correctly reports UNKNOWN.
    fn ensure_tap(&mut self, topic: &str, now_ns: u64) -> bool {
        if self.taps.len() >= self.budget {
            if !self.budget_warned {
                self.budget_warned = true;
                tracing::warn!(
                    budget = self.budget,
                    tracked = self.tracked.len(),
                    "this robot tracks more topics than the topic-liveness tap budget \
                     allows. An incumbent tap is never PREEMPTED to make room, so a topic \
                     registered after the budget filled is refused however early it sorts — on \
                     a robot whose topics appear over time (`cerulion ros2 attach`) the first \
                     observed set therefore follows REGISTRATION order, not alphabetical order. \
                     Slots ARE recycled, though: a tap released to a remote demand's egress \
                     drain, or dropped by a drain failure, frees its slot, and the next sweep \
                     hands it to the canonically-FIRST tracked topic that has none — so the \
                     observed subset can change over the life of the robot. Every topic without \
                     a tap reports UNKNOWN liveness — never a fabricated `no data` — so the desk \
                     renders those rows exactly as it does for a robot with no observer at all. \
                     Everything else is unaffected. To stop observing entirely set \
                     CERULION_TOPIC_LIVENESS=off; to observe a different subset, announce fewer \
                     topics (an unannounced topic is not tracked)"
                );
            }
            return false;
        }
        match self
            .manager
            .create_data_only_subscriber_with_buffer(topic, LIVENESS_TAP_BUFFER_SIZE)
        {
            Ok(tap) => {
                // hot-path-alloc-ok: cold — once per topic per attach regime.
                self.taps.insert(topic.to_string(), tap);
                let entry = self.observation_mut(topic);
                let rearm = entry.attach_warned;
                entry.attach_warned = false;
                if rearm {
                    tracing::info!(
                        topic = %topic,
                        "topic-liveness tap recovered — observation resumed"
                    );
                }
                // A brand-new port: its first burst may be the publisher's
                // retained history, so re-open the advancement baseline.
                self.with_record(topic, |r| r.begin_attached_observation(now_ns));
                true
            }
            Err(e) => {
                let entry = self.observation_mut(topic);
                entry.attach_failures += 1;
                let first = !entry.attach_warned;
                entry.attach_warned = true;
                if first {
                    tracing::warn!(
                        topic = %topic,
                        error = %e,
                        "could not attach a topic-liveness tap — this topic reports \
                         UNKNOWN liveness (never a fabricated `no data`) until it attaches; \
                         retrying each sweep (repeats are silent until it recovers)"
                    );
                }
                false
            }
        }
    }

    /// Drain `topic`'s tap to empty (bounded by [`MAX_DRAIN_PER_SWEEP`]) and
    /// record the result. Returns the frames observed.
    fn drain_topic(&mut self, topic: &str, now_ns: u64) -> u64 {
        let mut total = 0u64;
        // The newest publisher loan-time stamp seen across EVERY chunk of this
        // drain (the buffer is cleared per chunk, so it is accumulated here) —
        // what `record_frames` compares against this topic's recorded maximum to
        // tell a publish-since-last-drain from a flushed backlog.
        let mut newest_ts: Option<u64> = None;
        // The newest publisher COMMIT SEQUENCE across the same frames —
        // the rate estimate's numerator. Read from the same header parse.
        let mut newest_seq: Option<u32> = None;
        // Whether this drain left the queue EMPTY (see
        // `DrainObservation::queue_emptied`). Only the short-read exit proves it:
        // an error leaves an unknown remainder, and the `MAX_DRAIN_PER_SWEEP` cap
        // exits with the queue KNOWN to be non-empty.
        let mut queue_emptied = false;
        // How many publishers this topic has RIGHT NOW — the
        // single-writer evidence the rate's sequence basis needs. A dynamic-config
        // read on the tap's own already-open service handle, so it costs a load,
        // not an `.open()`. `None` when we hold no tap (nothing was drained
        // either, so the observation carries no sequence to trust).
        let mut writers_seen: Option<u32> = None;
        let mut failure: Option<TransportError> = None;
        if let Some(tap) = self.taps.get_mut(topic) {
            writers_seen = Some(tap.publisher_count());
            let budget = tap.max_borrowed_samples().max(1);
            loop {
                // Clearing first releases the previous chunk's SHM borrows, so a
                // full drain never trips `ExceedsMaxBorrows`.
                self.drain_buf.clear();
                let res = tap.drain_owned(budget, &mut self.drain_buf);
                // `drain_owned` KEEPS its partial fill on Err, so the buffer
                // length is the truth in both arms — those frames really were
                // observed and must be counted.
                let got = self.drain_buf.len() as u64;
                total += got;
                for owned in &self.drain_buf {
                    if let Some(header) = owned.wire_header() {
                        let ts = header.timestamp_ns;
                        newest_ts = Some(newest_ts.map_or(ts, |seen: u64| seen.max(ts)));
                        let seq = header.sequence;
                        newest_seq = Some(newest_seq.map_or(seq, |seen: u32| seen.max(seq)));
                    }
                }
                if let Err(e) = res {
                    failure = Some(e);
                    break;
                }
                if got < budget as u64 {
                    queue_emptied = true;
                    break;
                }
                if total >= MAX_DRAIN_PER_SWEEP {
                    break;
                }
            }
            self.drain_buf.clear();
        }
        let observation = DrainObservation {
            frames: total,
            newest_stamp_ns: newest_ts,
            newest_sequence: newest_seq,
            writers_seen,
            queue_emptied,
        };
        self.with_record(topic, |r| {
            r.start_observing(now_ns);
            r.record_frames(observation, now_ns);
        });
        if let Some(e) = failure {
            // Same reasoning as the gateway egress drain: the canonical cause is
            // a producer service torn down and re-created, which a stale tap can
            // never see. Drop it; the next sweep attaches a fresh one. Bank the
            // interval so no observation is claimed while unattached.
            self.taps.remove(topic);
            self.with_existing_record(topic, |r| r.stop_observing(now_ns));
            let entry = self.observation_mut(topic);
            entry.drain_failures += 1;
            let first = !entry.drain_warned;
            entry.drain_warned = true;
            if first {
                tracing::warn!(
                    topic = %topic,
                    error = %e,
                    "topic-liveness tap drain failed — dropping the tap and re-attaching \
                     next sweep (frames already drained this pass are counted; repeats are silent)"
                );
            }
        } else if total > 0 {
            self.observation_mut(topic).drain_warned = false;
        }
        total
    }

    fn now_ns(&self) -> u64 {
        self.manager.clock().now_ns()
    }

    fn observation_mut(&mut self, topic: &str) -> &mut TopicObservation {
        if !self.obs.contains_key(topic) {
            let key = topic.to_string(); // hot-path-alloc-ok: cold — once per topic, at its first accounting event.
            self.obs.insert(key, TopicObservation::default());
        }
        self.obs.get_mut(topic).expect("just inserted")
    }

    /// Mutate `topic`'s shared record, creating it if absent. Holds the table
    /// lock only for the closure.
    fn with_record(&self, topic: &str, f: impl FnOnce(&mut LivenessRecord)) {
        let mut table = self
            .table
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(rec) = table.get_mut(topic) {
            f(rec);
            return;
        }
        let mut rec = LivenessRecord::default();
        f(&mut rec);
        // hot-path-alloc-ok: cold — once per topic, at first observation.
        table.insert(topic.to_string(), rec);
    }

    /// Mutate `topic`'s shared record ONLY if it already exists — the
    /// end-of-observation counterpart to [`Self::with_record`]. Ending something
    /// that never began must not mint a record (which would replace a true
    /// "never observed" absence with an all-default entry).
    fn with_existing_record(&self, topic: &str, f: impl FnOnce(&mut LivenessRecord)) {
        if let Some(rec) = self
            .table
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get_mut(topic)
        {
            f(rec);
        }
    }

    // -- Principle #3 observables ------------------------------------------

    /// Topics the observer has been asked to watch.
    pub fn tracked_count(&self) -> usize {
        self.tracked.len()
    }

    /// Taps the observer currently owns (excludes externally-observed topics).
    pub fn active_tap_count(&self) -> usize {
        self.taps.len()
    }

    /// Cumulative executed (non-throttled) sweeps.
    pub fn sweep_count(&self) -> u64 {
        self.sweeps
    }

    /// Cumulative tap-attach failures for `topic`.
    pub fn attach_failure_count(&self, topic: &str) -> u64 {
        self.obs.get(topic).map_or(0, |o| o.attach_failures)
    }

    /// Cumulative tap-drain failures for `topic`.
    pub fn drain_failure_count(&self, topic: &str) -> u64 {
        self.obs.get(topic).map_or(0, |o| o.drain_failures)
    }

    /// This topic's liveness as of now — the same value the catalog serves.
    pub fn liveness(&self, topic: &str) -> Option<TopicLiveness> {
        read_liveness(&self.table, topic, self.now_ns())
    }

    /// Force the next [`Self::sweep`] to run regardless of the throttle. Test
    /// seam: production drives the observer from a loop that has been running
    /// far longer than one interval.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn force_next_sweep_for_test(&mut self) {
        self.last_sweep_ns = None;
    }

    /// Make `topic`'s next tap drain FAIL (test seam over
    /// [`DataOnlySubscriber::fault_inject_receive_after_for_test`]) — the
    /// production stimulus is a producer service torn down and re-created, which
    /// a stale tap can never see, and which is not deterministically stageable.
    /// Returns whether a tap was there to poison.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn fail_next_drain_for_test(&mut self, topic: &str) -> bool {
        match self.taps.get_mut(topic) {
            Some(tap) => {
                tap.fault_inject_receive_after_for_test(0);
                true
            }
            None => false,
        }
    }
}

impl std::fmt::Debug for TopicLivenessObserver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TopicLivenessObserver")
            .field("enabled", &self.enabled)
            .field("tracked", &self.tracked.len())
            .field("taps", &self.taps.len())
            .field("sweeps", &self.sweeps)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    /// The `&'static str` helper and the `Serialize` impl are two spellings of one
    /// vocabulary, and only serde's is enforced by the derive — so a hand-written
    /// arm renamed in one and not the other would ship a log line an operator's
    /// grep could not match against the JSON. Every variant, so adding one without
    /// its spelling fails here rather than at a call site.
    #[test]
    fn the_liveness_state_wire_spelling_agrees_with_serde() {
        for state in [
            LivenessState::Streaming,
            LivenessState::Idle,
            LivenessState::NoData,
            LivenessState::Unknown,
        ] {
            let serde_spelling = serde_json::to_string(&state).expect("state serializes");
            assert_eq!(
                serde_spelling,
                // hot-path-alloc-ok: test-only oracle — this builds the JSON
                // spelling (`"idle"`, quotes included) that serde must produce, and
                // it lives inside `#[cfg(test)] mod tests`, so it is not compiled
                // into any shipping binary and can never run on ANY path. Kept as a
                // `format!` deliberately: the quotes are the half of the assertion
                // that separates a correct encoding from a bare token, so writing
                // this into a reused buffer would obscure the oracle to satisfy a
                // lint that does not reach it.
                format!("\"{}\"", state.as_wire()),
                "as_wire disagrees with serde for {state:?}"
            );
        }
    }

    /// A drain that took `frames` frames whose newest publisher stamp is `stamp`
    /// AND left the queue empty — the ordinary shape (the observer's own drain
    /// loop always exits on a short read).
    fn drained(frames: u64, stamp: Option<u64>) -> DrainObservation {
        DrainObservation {
            frames,
            newest_stamp_ns: stamp,
            // The dating oracles below say nothing about rates, so they
            // carry no sequence — which is also the shape that exercises the
            // FLOOR fallback, keeping the dating arms independent of it.
            newest_sequence: None,
            // The dating oracles say nothing about writers either.
            writers_seen: None,
            queue_emptied: true,
        }
    }

    /// A drain that took `frames` frames, left the queue empty, and
    /// carried both halves of a real wire header — the shape a real publisher
    /// produces and the only one that can feed the EXACT rate basis.
    fn drained_seq(frames: u64, stamp: u64, seq: u32) -> DrainObservation {
        DrainObservation {
            frames,
            newest_stamp_ns: Some(stamp),
            newest_sequence: Some(seq),
            // The ordinary single-writer topic — graph-owned topics provision
            // `max_publishers = 1`, so this is what a real drain reads.
            writers_seen: Some(1),
            queue_emptied: true,
        }
    }

    /// A single-writer drain whose publisher count is whatever the
    /// caller says — the multi-publisher shapes.
    fn drained_seq_writers(frames: u64, stamp: u64, seq: u32, writers: u32) -> DrainObservation {
        DrainObservation {
            writers_seen: Some(writers),
            ..drained_seq(frames, stamp, seq)
        }
    }

    /// A drain that filled its budget, so MORE of the same burst may still be
    /// queued (the gateway's one-budget-per-pass egress drain).
    fn chunk(frames: u64, stamp: Option<u64>) -> DrainObservation {
        DrainObservation {
            frames,
            newest_stamp_ns: stamp,
            newest_sequence: None,
            writers_seen: None,
            queue_emptied: false,
        }
    }

    /// A drain that emptied the queue AND read exactly ONE publisher off
    /// the topic — the shape BOTH production drainers report on a graph-owned
    /// (single-writer) topic, and the only one the SUSTAINED reset path acts on.
    ///
    /// The dating oracles above deliberately carry `writers_seen: None`, which is
    /// why they are unaffected by that path; the sustained-path arms need this helper to
    /// reach it at all.
    fn drained_one_writer(frames: u64, stamp: u64) -> DrainObservation {
        DrainObservation {
            writers_seen: Some(1),
            ..drained(frames, Some(stamp))
        }
    }

    /// A single-writer drain that filled its budget and left frames behind
    /// — a re-flush's PREFIX chunk, which is what a budget-limited drainer reports
    /// for every chunk of a burst except its last.
    fn chunk_one_writer(frames: u64, stamp: u64) -> DrainObservation {
        DrainObservation {
            queue_emptied: false,
            ..drained_one_writer(frames, stamp)
        }
    }

    // -- LivenessRecord::snapshot (PURE) — hand oracles, never self-compares --

    #[test]
    fn never_observed_record_knows_nothing() {
        let r = LivenessRecord::default();
        assert_eq!(
            r.snapshot(10 * MS),
            None,
            "a tracked-but-never-attached topic must read UNKNOWN, not `no data`"
        );
    }

    /// The distinction `snapshot` draws is "is anything WATCHING", not "has time
    /// elapsed". A tap that attached this very instant reports `Some` (observation
    /// is underway) with zero elapsed — which still classifies UNKNOWN, so it
    /// renders identically to a never-observed topic while staying accurate inside.
    #[test]
    fn observation_that_just_started_reports_some_with_zero_elapsed() {
        let mut r = LivenessRecord::default();
        assert_eq!(r.snapshot(7_000 * MS), None, "before: knows nothing");
        r.start_observing(7_000 * MS);
        let snap = r.snapshot(7_000 * MS).expect("observation is underway");
        assert_eq!(
            snap,
            TopicLiveness {
                last_frame_age_ms: None,
                observed_for_ms: 0,
                frames_observed: 0,
                rate_estimate: None,
            }
        );
        assert_eq!(
            snap.state(),
            LivenessState::Unknown,
            "and it renders as UNKNOWN — never as a dead route"
        );
    }

    #[test]
    fn observed_with_no_frames_reports_the_watched_duration_and_no_age() {
        let mut r = LivenessRecord::default();
        r.start_observing(1_000 * MS);
        assert_eq!(
            r.snapshot(4_000 * MS),
            Some(TopicLiveness {
                last_frame_age_ms: None,
                observed_for_ms: 3_000,
                frames_observed: 0,
                rate_estimate: None,
            })
        );
    }

    /// An ADVANCING batch sets the age and the count. The baseline batch before
    /// it is what every attached observation starts with (see the module docs).
    #[test]
    fn an_advancing_batch_sets_the_age_and_the_count() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(1_000 * MS);
        r.record_frames(drained(1, Some(1_200 * MS)), 1_300 * MS); // baseline
        r.record_frames(drained(3, Some(2_500 * MS)), 2_500 * MS); // advances
        assert_eq!(
            r.snapshot(2_800 * MS),
            Some(TopicLiveness {
                last_frame_age_ms: Some(300),
                observed_for_ms: 1_800,
                frames_observed: 4,
                rate_estimate: None,
            })
        );
    }

    #[test]
    fn zero_count_is_an_observation_of_absence_and_never_refreshes_the_age() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(500 * MS)), 500 * MS); // baseline
        r.record_frames(drained(2, Some(1_000 * MS)), 1_000 * MS);
        r.record_frames(drained(0, None), 9_000 * MS);
        let snap = r.snapshot(9_000 * MS).expect("observed");
        assert_eq!(snap.last_frame_age_ms, Some(8_000), "age must keep growing");
        assert_eq!(snap.frames_observed, 3, "a zero drain adds no frames");
    }

    #[test]
    fn observation_time_accumulates_across_handoffs_and_excludes_the_gap() {
        let mut r = LivenessRecord::default();
        // Watched 1s, unobserved for 5s, watched 2s more.
        r.start_observing(0);
        r.stop_observing(1_000 * MS);
        r.start_observing(6_000 * MS);
        assert_eq!(
            r.snapshot(8_000 * MS).expect("observed").observed_for_ms,
            3_000,
            "the 5s unobserved gap must NOT count as observation"
        );
    }

    #[test]
    fn start_observing_is_idempotent_and_does_not_restart_the_interval() {
        let mut r = LivenessRecord::default();
        r.start_observing(1_000 * MS);
        r.start_observing(3_000 * MS);
        r.start_observing(4_000 * MS);
        assert_eq!(
            r.snapshot(5_000 * MS).expect("observed").observed_for_ms,
            4_000,
            "a re-affirmation must not reset the interval start"
        );
    }

    /// A second `stop_observing` must not bank the gap between the two stops —
    /// otherwise every idempotent "demand is still off" affirmation the gateway
    /// makes each drive pass would inflate `observed_for_ms` with time nothing
    /// was watching, and a never-observed topic would drift into `NoData`.
    ///
    /// Asserted through a RESUMED observation, since a stopped record reports
    /// `None` (the lost-observation rule).
    #[test]
    fn stop_observing_is_idempotent() {
        let mut r = LivenessRecord::default();
        r.start_observing(0);
        r.stop_observing(1_000 * MS);
        r.stop_observing(9_000 * MS);
        r.start_observing(9_000 * MS);
        assert_eq!(
            r.snapshot(9_000 * MS)
                .expect("observing again")
                .observed_for_ms,
            1_000,
            "only the one genuinely-watched second is banked"
        );
    }

    #[test]
    fn a_backward_clock_saturates_instead_of_wrapping() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(5_000 * MS);
        r.record_frames(drained(1, Some(4_900 * MS)), 5_000 * MS); // baseline
        r.record_frames(drained(1, Some(5_000 * MS)), 5_000 * MS); // advances
                                                                   // `now` BEFORE the interval start (a clock stepped backwards).
        let snap = r.snapshot(1_000 * MS).expect("frames observed");
        assert_eq!(snap.observed_for_ms, 0);
        assert_eq!(snap.last_frame_age_ms, Some(0));
    }

    /// THE LOST-OBSERVATION RULE: a record whose observation was LOST reports UNKNOWN,
    /// however much it saw before — and RESUMES with its history when something
    /// starts watching again.
    ///
    /// The alternative (keep serving the last verdict) is the failure this
    /// forbids: nothing is watching, so the verdict cannot be updated, yet it
    /// would be served with growing confidence — a topic that streamed would age
    /// into an ever-staler `Idle`, and a frameless one would harden into
    /// `NoData`, both attributed to a robot that stopped looking.
    #[test]
    fn a_lost_observation_reports_unknown_and_resumes_with_its_history() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        // The baseline burst: it may be the flushed backlog, so it is banked
        // without dating the topic.
        r.record_frames(drained(5, Some(0)), 100 * MS);
        // A genuine advance: sets the age.
        r.record_frames(drained(5, Some(500 * MS)), 500 * MS);
        assert!(r.snapshot(600 * MS).is_some(), "while observing: reports");

        // The tap is lost (drain failure / hand-off with no drainer).
        r.stop_observing(1_000 * MS);
        assert_eq!(
            r.snapshot(30_000 * MS),
            None,
            "nothing is watching ⇒ UNKNOWN, NOT a frozen 29.5 s-stale `idle`"
        );
        assert_eq!(
            r.snapshot(1_000 * MS),
            None,
            "and it is UNKNOWN immediately, not only once the value looks stale"
        );

        // Something re-attaches: the banked history comes back with it.
        r.begin_attached_observation(40_000 * MS);
        let snap = r.snapshot(40_500 * MS).expect("observation resumed");
        assert_eq!(
            snap.observed_for_ms,
            1_000 + 500,
            "banked (1 s) + the new open interval (0.5 s) — the gap is NOT counted"
        );
        assert_eq!(
            snap.last_frame_age_ms,
            Some(40_000),
            "the frame's age is measured from when it was SEEN, across the gap"
        );
        assert_eq!(snap.frames_observed, 10);
    }

    /// A record that never observed anything is UNKNOWN — the same `None` as one
    /// whose observation was lost, which is exactly right: both mean "the robot
    /// is not watching this topic".
    #[test]
    fn a_never_observed_record_and_a_lost_one_are_indistinguishable() {
        let never = LivenessRecord::default();
        let mut lost = LivenessRecord::default();
        lost.begin_attached_observation(0);
        lost.record_frames(drained(1, Some(10 * MS)), 10 * MS);
        lost.stop_observing(20 * MS);
        assert_eq!(never.snapshot(50 * MS), None);
        assert_eq!(lost.snapshot(50 * MS), None);
    }

    // -- The advancement rule (the retained-history defence) ------

    /// THE BASELINE. The first batch after a tap attaches may be the publisher's
    /// retained history (measured — see the module docs), and NOTHING in it can
    /// prove otherwise, so it is banked without dating the topic.
    ///
    /// The hand oracle is the reported hazard in miniature: a route dead for an
    /// hour whose backlog lands in our tap the moment some other subscriber
    /// connects. It must not read `Streaming` — and, per the produced-is-never-
    /// dead rule, it must not read `NoData` either.
    #[test]
    fn the_first_batch_after_an_attach_is_a_baseline_and_never_dates() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(50 * MS);
        // The publisher's retained backlog, delivered on the first drain.
        r.record_frames(drained(4, Some(10 * MS)), 100 * MS);
        let snap = r.snapshot(100 * MS).expect("observing");
        assert_eq!(
            snap.last_frame_age_ms, None,
            "a baseline burst must NOT present as a frame that just arrived"
        );
        assert_eq!(
            snap.frames_observed, 4,
            "the frames are still banked — they really were seen"
        );

        // Nothing more ever arrives. It is UNDATED forever, but it PRODUCED, so
        // it is Idle (freshness unknown) — never the dimmed dead row.
        let much_later = r
            .snapshot(LIVENESS_NO_DATA_MIN_MS * MS + 100 * MS)
            .expect("observing");
        assert_eq!(
            much_later.state(),
            LivenessState::Idle,
            "THE POINT: a dead route whose history was flushed into our tap is \
             neither fresh (nothing proved a publish) nor dimmed (it demonstrably \
             carries data)"
        );
        assert_ne!(much_later.state(), LivenessState::NoData);
        assert_ne!(much_later.state(), LivenessState::Streaming);

        // ANTI-TAUTOLOGY: a genuinely live topic's NEXT batch DOES advance.
        r.record_frames(drained(1, Some(200 * MS)), 200 * MS);
        let live = r.snapshot(250 * MS).expect("observing");
        assert_eq!(live.last_frame_age_ms, Some(50));
        assert_eq!(live.state(), LivenessState::Streaming);
        assert_eq!(live.frames_observed, 5);
    }

    /// THE HEADLINE ROBUSTNESS PROPERTY, and the reason the rule is stamp-vs-stamp
    /// rather than stamp-vs-attach-instant: a re-flush of the SAME backlog into a
    /// LATER connection carries the SAME stamps, so it can never advance anything.
    ///
    /// Every hand-off (a remote demand taking the tap) and every re-attach (a
    /// drain failure, a freed budget slot) creates a fresh connection that the
    /// publisher may flush its retained history into. A dead route must survive
    /// arbitrarily many of those without ever looking live.
    #[test]
    fn a_re_flush_of_the_same_backlog_can_never_advance() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        // The dead route's backlog: four frames, newest stamped 10 ms.
        r.record_frames(drained(4, Some(10 * MS)), 100 * MS);

        // Hand off and re-attach five times; each new connection is flushed the
        // SAME backlog.
        let mut now = 200 * MS;
        for _ in 0..5 {
            r.stop_observing(now);
            now += 100 * MS;
            r.begin_attached_observation(now);
            now += 10 * MS;
            r.record_frames(drained(4, Some(10 * MS)), now);
            assert_eq!(
                r.snapshot(now).expect("observing").last_frame_age_ms,
                None,
                "a re-flush of stamps we have already seen is not a publish"
            );
        }
        let snap = r.snapshot(now).expect("observing");
        assert_eq!(snap.frames_observed, 24, "banked in full: 4 + 5 * 4");
        assert_eq!(
            snap.state(),
            LivenessState::Idle,
            "produced, undatable, and STILL not dimmed after five hand-offs"
        );
    }

    /// The split-flush hole: a retained-history flush deeper than one drain's
    /// budget arrives in TWO batches, and the second chunk's newest stamp EXCEEDS the
    /// first's. Judged naively that is "advancement" and the dead route is dated.
    ///
    /// The baseline stays OPEN until a drain leaves the queue EMPTY, so both
    /// chunks are absorbed into the baseline and neither dates the topic. This is
    /// the gateway's egress shape exactly (one `drain_owned(budget)` per drive
    /// pass, no drain-to-empty loop).
    ///
    /// What breaks this test: close the baseline on the first nonempty batch regardless of
    /// `queue_emptied` and this fails with `last_frame_age_ms: Some(0)`.
    #[test]
    fn a_split_flush_cannot_advance_against_its_own_first_chunk() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(1_000 * MS);
        // First chunk: the drain filled its budget, so more may be queued.
        r.record_frames(chunk(3, Some(30 * MS)), 1_010 * MS);
        // Second chunk: the tail of the SAME flush — newer stamps, still history.
        r.record_frames(drained(1, Some(40 * MS)), 1_020 * MS);
        let snap = r.snapshot(1_020 * MS).expect("observing");
        assert_eq!(
            snap.last_frame_age_ms, None,
            "the tail of one flush must not out-rank its own head"
        );
        assert_eq!(snap.frames_observed, 4, "both chunks are banked");

        // And the burst is now over, so genuinely new traffic dates it.
        r.record_frames(drained(1, Some(1_100 * MS)), 1_100 * MS);
        assert_eq!(
            r.snapshot(1_100 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "once the queue emptied the baseline closed — this is a real publish"
        );
    }

    /// A baseline that NEVER completes (every drain fills its budget) never dates
    /// the topic. Accurate and bounded: the topic reads Idle with no age rather than
    /// a fabricated freshness, and a short read ENDS the burst — the short read
    /// does not itself date the topic (it is still the baseline's last batch), the
    /// first batch AFTER it does.
    #[test]
    fn a_baseline_that_never_completes_stays_undated_until_a_short_read_ends_it() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        let mut now = 0;
        for step in 1..=5u64 {
            now = step * 10 * MS;
            r.record_frames(chunk(3, Some(now)), now);
            assert_eq!(
                r.snapshot(now).expect("observing").last_frame_age_ms,
                None,
                "still absorbing the burst at step {step}"
            );
        }
        assert_eq!(
            r.snapshot(now).expect("observing").state(),
            LivenessState::Idle,
            "produced but undatable — never the dimmed row"
        );
        // One short read ends the burst; the next batch dates normally.
        now += 10 * MS;
        r.record_frames(drained(1, Some(now)), now);
        now += 10 * MS;
        r.record_frames(drained(1, Some(now)), now);
        assert_eq!(
            r.snapshot(now).expect("observing").last_frame_age_ms,
            Some(0)
        );
    }

    /// An unstamped batch (no parseable wire header on any frame) cannot advance
    /// anything — conservative in the false-live direction. Once a baseline stamp
    /// exists it costs only that batch: the next readable stamp dates the topic
    /// normally. (The FIRST-ever batch being unstamped costs one more — see
    /// `a_first_ever_unstamped_batch_costs_one_batch_more_than_a_later_one`.)
    #[test]
    fn an_unstamped_batch_never_advances_but_costs_only_that_batch() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(2, Some(10 * MS)), 100 * MS); // baseline
        r.record_frames(drained(2, None), 200 * MS);
        assert_eq!(
            r.snapshot(200 * MS).expect("observing").last_frame_age_ms,
            None,
            "unprovable ⇒ not dated"
        );
        r.record_frames(drained(1, Some(300 * MS)), 300 * MS);
        assert_eq!(
            r.snapshot(300 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "and a readable stamp right after it dates the topic — an unreadable \
             header costs one batch, never permanent deafness"
        );
        assert_eq!(r.snapshot(300 * MS).expect("observing").frames_observed, 5);
    }

    /// The edge the test above structurally cannot see: when the FIRST-ever
    /// batch is unstamped, `max_stamp_ns` stays unset, so the NEXT batch — stamped
    /// and genuinely new — has nothing to advance past and is itself only a
    /// baseline. Two batches, not one.
    ///
    /// Still bounded, still never deafness, and reachable in production only when
    /// a topic's very first drained frames carry no parseable header (a foreign
    /// external producer on an absolute `source:` topic emitting sub-header or
    /// corrupt frames).
    #[test]
    fn a_first_ever_unstamped_batch_costs_one_batch_more_than_a_later_one() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(2, None), 100 * MS); // first ever: unstamped
        r.record_frames(drained(1, Some(200 * MS)), 200 * MS); // sets the baseline stamp
        assert_eq!(
            r.snapshot(200 * MS).expect("observing").last_frame_age_ms,
            None,
            "the first STAMPED batch has nothing to advance past — it is the \
             baseline the unstamped one could not establish"
        );
        r.record_frames(drained(1, Some(300 * MS)), 300 * MS);
        assert_eq!(
            r.snapshot(300 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "and the batch after THAT dates — two batches lost, never permanent"
        );
    }

    /// The comparison is STRICT: a repeated stamp is not a publish. That is what
    /// makes a re-flush inert, and it is why a publisher whose clock does not
    /// advance between frames reads Idle rather than Streaming (a documented
    /// residual — never a false live).
    #[test]
    fn an_equal_stamp_does_not_advance() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(500 * MS)), 500 * MS); // baseline
        r.record_frames(drained(1, Some(500 * MS)), 600 * MS);
        assert_eq!(
            r.snapshot(600 * MS).expect("observing").last_frame_age_ms,
            None,
            "the same stamp again is not evidence of a new publish"
        );
        // One nanosecond more is unambiguously a new commit.
        r.record_frames(drained(1, Some(500 * MS + 1)), 700 * MS);
        assert_eq!(
            r.snapshot(700 * MS).expect("observing").last_frame_age_ms,
            Some(0)
        );
    }

    /// A stamp BELOW the recorded maximum never dates the topic — a stale frame
    /// (a re-flush's older tail, an out-of-order publisher, or the first frames of
    /// a RESTARTED publisher) cannot refresh an age that a newer frame set.
    ///
    /// It does, however, RESET the epoch (see
    /// `a_publisher_restart_resets_the_stamp_epoch_and_dating_resumes`): the
    /// regressed stamp becomes the new maximum, so the batch AFTER it dates
    /// against the new epoch instead of being measured against a maximum it can
    /// never reach. Both halves are asserted here.
    #[test]
    fn a_stamp_below_the_maximum_never_dates() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(100 * MS)), 100 * MS); // baseline
        r.record_frames(drained(1, Some(900 * MS)), 900 * MS); // advances
        r.record_frames(drained(1, Some(200 * MS)), 5_000 * MS); // regressed
        assert_eq!(
            r.snapshot(5_000 * MS).expect("observing").last_frame_age_ms,
            Some(4_100),
            "the age still dates from the ADVANCING batch, not the regressed one"
        );
        // The regression adopted 200 ms as the new maximum, so 300 ms — which is
        // BELOW the old 900 ms maximum — is now an advance and dates the topic.
        r.record_frames(drained(1, Some(300 * MS)), 6_000 * MS);
        assert_eq!(
            r.snapshot(6_000 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "the epoch reset makes the NEXT batch datable against the new maximum"
        );
    }

    /// The re-flush split, in the three chunks that defeat a baseline-only guard.
    /// A re-flushed backlog's early chunks are a strict prefix of it and
    /// therefore REGRESS against the maximum the first flush established. Adopting
    /// one LOWERS the bar, after which a later chunk of the SAME backlog out-ranks
    /// it and dates a route that has been dead since the first flush — the exact
    /// hole the carried maximum exists to close, re-opened on every hand-off and
    /// re-attach.
    ///
    /// TWO chunks are not enough to pin it, and that is the point of this shape.
    /// The `!baseline_open` guard alone defends only the chunks that arrive while
    /// the burst is provably underway: the module docs' mid-flush interleave has
    /// the first chunk empty the queue while `deliver_history` is still pushing, which
    /// CLOSES the baseline mid-burst, so the second chunk lands on a closed baseline and
    /// would LOWER the maximum from 400 ms to 300 ms. It does not itself
    /// date the topic — the reset re-opens the baseline for exactly that batch —
    /// which is why a two-chunk test passes while the bar is already on the floor.
    /// The tail chunk, the backlog's own 400 ms, then out-ranks the lowered bar and
    /// renders the dead route `Streaming`.
    ///
    /// The silence gate is what closes it: the second chunk arrives ONE DRAIN CADENCE after
    /// the first, nothing like the [`REGRESSION_RESET_MIN_GAP_NS`] of silence a
    /// process restart shows, so it is banked and the maximum stays at 400 ms.
    ///
    /// What breaks this test: drop `&& quiet_long_enough` ⇒ the second chunk
    /// adopts 300 ms and the tail's 400 ms out-ranks it ⇒ `max_stamp_ns:
    /// Some(300 ms)` and `Some(0)` / `Streaming` at the tail.
    ///
    /// This test does NOT catch reverting `&& !self.baseline_open`:
    /// EVERY chunk in this script arrives
    /// within [`REGRESSION_RESET_MIN_GAP_NS`] of the last frame-yielding drain, so
    /// the silence gate blocks all three resets on its own and guard 1 is
    /// unexercised here. Guard 1's own load-bearing shape — a re-flush whose FIRST
    /// chunk lands after a real lull, where the silence gate is satisfied outright
    /// and only the OPEN baseline stands — is pinned by
    /// `a_re_flush_after_a_lull_cannot_lower_the_bar_while_the_baseline_is_open`.
    #[test]
    fn a_re_flushed_backlog_chunk_cannot_lower_the_bar_and_date_a_dead_route() {
        // The route published a backlog topping out at 400 ms and DIED. Connection
        // 1 sees the whole thing as its baseline, so nothing is ever dated.
        const BACKLOG_NEWEST: u64 = 400 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(4, Some(BACKLOG_NEWEST)), 10 * MS);
        assert_eq!(
            r.snapshot(10 * MS).expect("observing").last_frame_age_ms,
            None,
            "the first flush is the baseline — a dead-but-history-carrying route"
        );

        // A hand-off (or a lost tap re-attaching) re-flushes the SAME backlog into
        // the new connection, and it arrives SPLIT across three drains a cadence
        // apart — the gateway's one-budget-per-pass shape.
        r.begin_attached_observation(100 * MS);
        // First chunk: a strict PREFIX of that backlog (stamps only up to 200 ms),
        // which regresses against the 400 ms maximum. The OPEN baseline banks it —
        // and it closes the baseline on its short read, mid-burst.
        r.record_frames(drained(2, Some(200 * MS)), 110 * MS);
        assert_eq!(
            r.snapshot(110 * MS).expect("observing").last_frame_age_ms,
            None,
            "the prefix chunk does not date the topic either way"
        );
        // Second chunk: more of that SAME backlog, on a now-CLOSED baseline. Only the
        // silence gate stands between it and the maximum, and 10 ms is a drain
        // cadence, not a restart.
        r.record_frames(drained(2, Some(300 * MS)), 120 * MS);
        assert_eq!(
            r.max_stamp_ns,
            Some(BACKLOG_NEWEST),
            "THE BAR ITSELF: a chunk of the re-flush must not lower the maximum, \
             whichever side of the baseline close it lands on"
        );
        assert_eq!(
            r.snapshot(120 * MS).expect("observing").last_frame_age_ms,
            None,
            "and it does not date the topic"
        );
        // Tail chunk: the backlog's own tail, stamped exactly where flush 1 topped
        // out. Against the PRESERVED maximum it is not an advance; against a
        // lowered one it would be, and the dead route would render Streaming.
        r.record_frames(drained(2, Some(BACKLOG_NEWEST)), 130 * MS);
        let l = r.snapshot(130 * MS).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "a re-flush of the SAME backlog can never advance the baseline, \
             however it is chunked: {l:?}"
        );
        assert_eq!(
            l.frames_observed, 10,
            "every re-flushed frame is still banked"
        );
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "produced, undatable — never the Streaming a lowered bar would fake"
        );
    }

    /// Guard 1's isolated oracle: a re-flush whose first chunk arrives
    /// after a LULL. This is the one shape in which `!self.baseline_open` is the
    /// ONLY thing standing between a backlog PREFIX and a dated dead route, and
    /// until this test existed nothing in the suite covered it — deleting
    /// `&& !self.baseline_open` left every other liveness test green (verified by
    /// running it: 44 lib + 27 `topic_liveness_iox2_test` + 7 `gateway_iox2_test`
    /// liveness arms all passed with guard 1 gone).
    ///
    /// The reason is structural. Every other re-flush script drains its chunks a
    /// drain cadence apart, so [`REGRESSION_RESET_MIN_GAP_NS`] of silence never
    /// accumulates and the SILENCE gate blocks each reset on its own, leaving
    /// guard 1 unexercised. But silence is a property of the OBSERVER's drain
    /// history, not of the flush: a hand-off onto an idle robot, or a tap
    /// re-attaching after the budget recycled its slot, re-opens the baseline long
    /// after the last frame-yielding drain. The backlog then flushes into that
    /// fresh connection with guard 3 ALREADY satisfied, and the open baseline is
    /// the whole defense.
    ///
    /// What breaks this test: drop `&& !self.baseline_open` ⇒ the first chunk
    /// adopts 200 ms (the `max_stamp_ns` assertion fails) and the second chunk's 300 ms then
    /// out-ranks the floored bar and dates the dead route ⇒ `Some(0)` /
    /// `Streaming`. The bar and the verdict EACH independently catch this failure
    /// (measured: with the bar assertion neutralized the verdict still fails with
    /// `last_frame_age_ms: Some(0)`), so weakening one leaves the other catching it
    /// — libtest simply reports whichever comes first.
    ///
    /// It is an ISOLATED oracle by
    /// construction: dropping `&& quiet_long_enough` leaves guard 1 banking the first chunk
    /// (the second then takes the wrong reset arm, but that arm re-opens the baseline
    /// for exactly that batch, so nothing dates inside a two-chunk script — the
    /// hole a third chunk opens is what
    /// `a_re_flushed_backlog_chunk_cannot_lower_the_bar_and_date_a_dead_route`
    /// pins), and neutralizing the `regressed` arm entirely keeps the maximum
    /// monotone at 400 ms so nothing advances either.
    #[test]
    fn a_re_flush_after_a_lull_cannot_lower_the_bar_while_the_baseline_is_open() {
        // The route published a backlog topping out at 400 ms and DIED. Connection
        // 1 sees the whole thing as its baseline, so nothing is ever dated.
        const BACKLOG_NEWEST: u64 = 400 * MS;
        const FIRST_FLUSH_AT: u64 = 10 * MS;
        // Then NOTHING is drained for well past the gate — the robot is idle and
        // this route is dead — and a hand-off attaches a fresh connection, into
        // which the SAME backlog is re-flushed.
        const REATTACH_AT: u64 = FIRST_FLUSH_AT + REGRESSION_RESET_MIN_GAP_NS + 500 * MS;
        const CHUNK_A_AT: u64 = REATTACH_AT + 10 * MS;
        const _: () = assert!(
            CHUNK_A_AT - FIRST_FLUSH_AT >= REGRESSION_RESET_MIN_GAP_NS,
            "precondition: the silence gate must be CLEARED in front of the first chunk, \
             so that guard 1 is the only guard left standing"
        );
        const CHUNK_B_AT: u64 = CHUNK_A_AT + 10 * MS;
        const _: () = assert!(
            CHUNK_B_AT - CHUNK_A_AT < REGRESSION_RESET_MIN_GAP_NS,
            "precondition: the second chunk follows a drain cadence behind the first, as the \
             chunks of one burst do"
        );

        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(4, Some(BACKLOG_NEWEST)), FIRST_FLUSH_AT);
        assert_eq!(
            r.snapshot(FIRST_FLUSH_AT)
                .expect("observing")
                .last_frame_age_ms,
            None,
            "the first flush is the baseline — a dead-but-history-carrying route"
        );

        r.begin_attached_observation(REATTACH_AT);
        // First chunk: a strict PREFIX of that same backlog (stamps only up to 200 ms),
        // regressing against the 400 ms maximum — but preceded by REAL silence, so
        // `quiet_long_enough` is TRUE and the open baseline is the only guard left.
        r.record_frames(drained(2, Some(200 * MS)), CHUNK_A_AT);
        assert_eq!(
            r.max_stamp_ns,
            Some(BACKLOG_NEWEST),
            "THE BAR ITSELF: an OPEN baseline must bank a lull-preceded prefix \
             chunk rather than adopt it, because the chunks behind it are stamped \
             HIGHER and would out-rank a floored bar"
        );
        assert_eq!(
            r.snapshot(CHUNK_A_AT).expect("observing").last_frame_age_ms,
            None,
            "and the prefix chunk does not date the topic either way"
        );
        // Second chunk: more of that SAME backlog. Against the PRESERVED 400 ms bar it
        // is not an advance; against a floored 200 ms one it is, and the dead route
        // would render Streaming.
        r.record_frames(drained(2, Some(300 * MS)), CHUNK_B_AT);
        let l = r.snapshot(CHUNK_B_AT).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "THE VERDICT: a later chunk of one re-flushed backlog must not date a \
             route that has been dead since the first flush: {l:?}"
        );
        assert_eq!(
            l.frames_observed, 8,
            "every re-flushed frame is still banked"
        );
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "produced, undatable — never the Streaming a floored bar would fake"
        );
    }

    /// The oracle that the reset arm is GUARDED AT ALL, so the guards cannot both
    /// be dropped silently: a regression drained under an OPEN baseline, within the
    /// silence window, leaves the maximum ALONE.
    ///
    /// The sequence is built to probe THE MAXIMUM rather than the verdict, because
    /// those are not the same thing — an epoch reset re-opens the baseline, so an
    /// adopted (wrong) maximum still produces `None` for the batch that adopted it,
    /// and a test that stops there cannot tell a preserved 900 ms bar from a
    /// lowered 100 ms one. So: the regression arrives as a `chunk` (the baseline
    /// stays OPEN), a 500 ms batch then closes the baseline while raising nothing
    /// (`max(900, 500) == 900`), and a 600 ms batch is the discriminator — below a
    /// preserved 900 ms maximum it must NOT date; above a wrongly-adopted 100 ms
    /// or 500 ms one it would.
    ///
    /// What breaks this test: drop BOTH guards — an unconditional
    /// `if regressed` — ⇒ the 100 ms chunk adopts and the first assertion below
    /// reads `max_stamp_ns: Some(100 ms)` where `Some(900 ms)` is required.
    ///
    /// Dropping either guard alone does not fail it (dropping
    /// `&& !self.baseline_open` alone leaves it green): the
    /// 100 ms chunk arrives 20 ms after the last frame-yielding drain, so guard 1
    /// and the silence gate BOTH block that reset and either one suffices. This
    /// test is therefore the "the arm is guarded at all" oracle; the guards' own
    /// isolated oracles are
    /// `a_re_flush_after_a_lull_cannot_lower_the_bar_while_the_baseline_is_open`
    /// (guard 1) and `the_silence_gate_is_a_threshold_pinned_on_both_sides`
    /// (guard 2).
    #[test]
    fn a_regression_under_an_open_baseline_banks_without_lowering_the_maximum() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(900 * MS)), 10 * MS); // baseline sets 900 ms
                                                              // A re-attach re-opens the baseline.
        r.begin_attached_observation(20 * MS);
        // Regressed under an OPEN baseline, and a CHUNK — so the baseline is still
        // open for the batch after it, isolating guard 1 from the silence gate.
        r.record_frames(chunk(1, Some(100 * MS)), 30 * MS);
        assert_eq!(
            r.max_stamp_ns,
            Some(900 * MS),
            "an open-baseline regression banks, it does not adopt"
        );
        // Still the same burst: raises nothing (900 stands), closes the baseline.
        r.record_frames(drained(1, Some(500 * MS)), 40 * MS);
        assert_eq!(
            r.max_stamp_ns,
            Some(900 * MS),
            "max(900, 500) is still 900 — the burst raised nothing"
        );
        // THE DISCRIMINATOR: 600 ms out-ranks both wrong answers (100 ms and
        // 500 ms) and loses to the right one.
        r.record_frames(drained(1, Some(600 * MS)), 50 * MS);
        assert_eq!(
            r.snapshot(50 * MS).expect("observing").last_frame_age_ms,
            None,
            "600 ms is below the PRESERVED 900 ms maximum, so it is not a publish"
        );
        // And one nanosecond past that preserved maximum is.
        r.record_frames(drained(1, Some(900 * MS + 1)), 60 * MS);
        assert_eq!(
            r.snapshot(60 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "one nanosecond past the PRESERVED maximum is a genuine publish"
        );
    }

    /// The SECOND guard's own oracle: the silence gate is a THRESHOLD, and both
    /// sides of it are pinned here so neither the constant nor the comparison can
    /// drift silently.
    ///
    /// Identical stamp scripts, identical baseline state, differing ONLY in the
    /// observer-clock gap in front of the regressed batch — one nanosecond under
    /// the threshold banks, exactly at it resets.
    #[test]
    fn the_silence_gate_is_a_threshold_pinned_on_both_sides() {
        /// Runs one regression `gap` ns after the last frame-yielding drain and
        /// reports the resulting maximum. A reset ADOPTS 100 ms; a bank keeps
        /// 900 ms.
        fn max_after_regression_with_gap(gap: u64) -> Option<u64> {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained(1, Some(900 * MS)), 10 * MS); // baseline, closes
            r.record_frames(drained(1, Some(100 * MS)), 10 * MS + gap);
            r.max_stamp_ns
        }
        assert_eq!(
            max_after_regression_with_gap(REGRESSION_RESET_MIN_GAP_NS - 1),
            Some(900 * MS),
            "one nanosecond short of the gate: a drain-cadence regression is a \
             re-flush chunk and must not lower the bar"
        );
        assert_eq!(
            max_after_regression_with_gap(REGRESSION_RESET_MIN_GAP_NS),
            Some(100 * MS),
            "exactly at the gate: `>=`, so the restart reading wins here"
        );
        // And the gate measures SILENCE, not elapsed time: drains that yield frames
        // re-anchor it, so a continuously-drained stream never accumulates a gap.
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(900 * MS)), 10 * MS);
        let cadence = REGRESSION_RESET_MIN_GAP_NS / 4;
        for i in 1..=8 {
            r.record_frames(drained(1, Some(100 * MS + i)), 10 * MS + i * cadence);
        }
        assert_eq!(
            r.max_stamp_ns,
            Some(900 * MS),
            "eight regressed batches spanning twice the gate but never SILENT for \
             it — every one is banked"
        );
        // Whereas the SAME elapsed time with nothing arriving in it clears the gate
        // on the first frame that follows. Zero-frame drains are silence and must
        // not re-anchor it.
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(900 * MS)), 10 * MS);
        r.record_frames(drained(0, None), 10 * MS + cadence);
        r.record_frames(drained(0, None), 10 * MS + 7 * cadence);
        r.record_frames(drained(1, Some(100 * MS)), 10 * MS + 8 * cadence);
        assert_eq!(
            r.max_stamp_ns,
            Some(100 * MS),
            "an empty drain is an observation of ABSENCE — it must not re-anchor \
             the silence the restart reading depends on"
        );
    }

    /// THE RESTART PIN (pure). A publisher runs for two hours, is restarted, and
    /// its replacement's `VirtualClock` begins again near zero — the shipping
    /// multi-process shape, under an observer (the `cerulion-netd` gateway) that
    /// outlives both runs.
    ///
    /// Were `max_stamp_ns` monotone, every batch of the NEW run would be
    /// compared against the DEAD run's high-water mark and `advanced` would be false
    /// for the whole of the next two hours: an actively-streaming topic would render
    /// `Idle` with no age, which is the exact inversion the observer exists to prevent.
    ///
    /// What breaks this test: delete the `regressed` arm from `record_frames` (restoring
    /// the monotone `seen.max(ts)`) and every post-restart assertion below fails
    /// with `last_frame_age_ms: None` / `Idle`.
    #[test]
    fn a_publisher_restart_resets_the_stamp_epoch_and_dating_resumes() {
        // Two hours of the first run, on its own clock.
        const RUN1_END: u64 = 2 * 60 * 60 * 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(RUN1_END - 100 * MS)), 10 * MS); // baseline
        r.record_frames(drained(1, Some(RUN1_END)), 20 * MS); // advances
        assert_eq!(
            r.snapshot(20 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "the first run dates normally"
        );

        // The worker is restarted. Its replacement stamps from ~0 on a FRESH
        // clock, far below everything the dead run ever produced. (Const-asserted
        // so a future edit cannot make the scenario non-probative.)
        const RUN2_START: u64 = 5 * MS;
        const _: () = assert!(
            RUN2_START < RUN1_END,
            "precondition: the restarted publisher's stamps must sit BELOW the \
             dead run's maximum, which is what a VirtualClock restarting at 0 does"
        );
        // The restart itself takes OBSERVER-clock time — teardown, boot, graph
        // build, service open — during which drains yield nothing. That silence is
        // the second guard on the reset (a re-flush's chunks arrive a drain cadence
        // apart, never after a lull), so it is modelled here rather than skipped.
        // Three seconds is a fast restart and still an order of magnitude past the
        // gate; const-asserted so shrinking the gate cannot silently defuse this.
        const RESTART_AT: u64 = 3_000 * MS;
        const _: () = assert!(
            RESTART_AT - 20 * MS >= REGRESSION_RESET_MIN_GAP_NS,
            "precondition: the restart must present as SILENCE longer than the \
             re-flush gate, which is what a process restart looks like"
        );
        r.record_frames(drained(2, Some(RUN2_START)), RESTART_AT);
        assert_eq!(
            r.snapshot(RESTART_AT).expect("observing").last_frame_age_ms,
            Some(2_980),
            "the regressed batch itself never dates — it may be the NEW run's own \
             retained-history flush, so it only re-opens the baseline"
        );

        // ONE advancement later the topic is dated again — the whole point.
        r.record_frames(drained(1, Some(RUN2_START + MS)), RESTART_AT + 10 * MS);
        let l = r.snapshot(RESTART_AT + 10 * MS).expect("observing");
        assert_eq!(
            l.last_frame_age_ms,
            Some(0),
            "WITH A MONOTONE MAXIMUM THIS IS `Some(2_990)` FOREVER (never refreshed): the restarted \
             publisher would have to climb past two hours of the dead run's \
             clock before a single frame could date the topic again"
        );
        assert_eq!(l.state(), LivenessState::Streaming);
        assert_eq!(l.frames_observed, 5, "every frame is still banked");

        // And it keeps dating as the new run climbs — the reset is not one-shot
        // luck, the record genuinely lives on the new epoch's number line now.
        r.record_frames(drained(1, Some(RUN2_START + 2 * MS)), RESTART_AT + 20 * MS);
        assert_eq!(
            r.snapshot(RESTART_AT + 20 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(0)
        );
    }

    /// The epoch reset RE-OPENS the baseline, so a restarted publisher's OWN
    /// retained-history flush cannot date it either — the restart fix does not
    /// re-open the false-live hole the baseline exists to close.
    ///
    /// The shape that matters: a worker restarts, publishes four frames into its
    /// retained history, and DIES again (a crash loop). That backlog flushes into
    /// the tap across two drains whose stamps ascend. Without the baseline re-open
    /// the second chunk would out-rank the first and the dead-again route would
    /// render `Streaming`.
    #[test]
    fn an_epoch_reset_re_opens_the_baseline_so_the_new_runs_flush_cannot_date_it() {
        const RUN1_END: u64 = 60 * 60 * 1_000 * MS;
        // The restart's observer-clock silence — see
        // `a_publisher_restart_resets_the_stamp_epoch_and_dating_resumes`.
        const RESTART_AT: u64 = 3_000 * MS;
        const _: () = assert!(RESTART_AT - 20 * MS >= REGRESSION_RESET_MIN_GAP_NS);
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(RUN1_END - MS)), 10 * MS); // baseline
        r.record_frames(drained(1, Some(RUN1_END)), 20 * MS); // advances
        assert_eq!(
            r.snapshot(20 * MS).expect("observing").last_frame_age_ms,
            Some(0)
        );

        // The restarted-then-dead run's backlog, split across two chunked drains
        // (the gateway's one-budget-per-pass shape) with ASCENDING stamps.
        r.record_frames(chunk(2, Some(10 * MS)), RESTART_AT); // regresses ⇒ reset + baseline
        r.record_frames(chunk(2, Some(20 * MS)), RESTART_AT + 10 * MS); // same flush
        assert_eq!(
            r.snapshot(RESTART_AT + 10 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(2_990),
            "the second chunk of the new run's flush must NOT out-rank the first — the age is \
             still the pre-restart arrival"
        );
        // A short read ends the burst; only what the publisher produces AFTER it
        // can date the topic.
        r.record_frames(drained(1, Some(30 * MS)), RESTART_AT + 20 * MS);
        assert_eq!(
            r.snapshot(RESTART_AT + 20 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(3_000),
            "the batch that CLOSES the baseline still does not date it"
        );
        r.record_frames(drained(1, Some(40 * MS)), RESTART_AT + 30 * MS);
        assert_eq!(
            r.snapshot(RESTART_AT + 30 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(0),
            "and the first genuinely-new frame after the burst does"
        );
    }

    /// THE DOCUMENTED RESIDUAL (module docs), pinned at its FULL cost rather than
    /// its convenient one. A drain that SPANS the restart — dead-run tail plus
    /// new-run head in one batch — cannot trigger the epoch reset, because the
    /// batch's NEWEST stamp still belongs to the dead run and therefore does not
    /// regress.
    ///
    /// That alone would cost one batch. The silence gate makes it cost more: the
    /// spanning batch RE-ANCHORS the silence, so run 2's next batch — the first
    /// that does regress — arrives a drain cadence later and is read as a re-flush
    /// chunk and BANKED. The reset therefore waits for the first regressed batch
    /// that is ALSO preceded by [`REGRESSION_RESET_MIN_GAP_NS`] of silence, i.e. the
    /// first lull in the restarted run's drained stream. That wait is NOT bounded: a
    /// restarted run streaming at or above the sweep rate (~5 Hz) never produces such
    /// a lull, so it stays un-reset for as long as it keeps streaming, and
    /// meanwhile the row serves the DEAD run's `last_frame_at_ns` — an age that grows
    /// a second every second. Never a false live and never dimmed, but affirmatively
    /// WRONG on freshness, and for far longer than "one batch". This test pins the
    /// HEALING case (a lull does arrive); the never-healing one is the module docs'
    /// residual.
    #[test]
    fn a_batch_spanning_a_restart_delays_the_reset_until_a_silent_regression() {
        const RUN1_END: u64 = 60 * 60 * 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(RUN1_END)), 10 * MS); // baseline at the top of run 1

        // The spanning batch: run 1's tail (RUN1_END) and run 2's head (5 ms),
        // drained together after the restart's silence. Its NEWEST stamp is run 1's,
        // so nothing regresses and nothing resets.
        const SPAN_AT: u64 = 3_000 * MS;
        const _: () = assert!(SPAN_AT - 10 * MS >= REGRESSION_RESET_MIN_GAP_NS);
        r.record_frames(drained(2, Some(RUN1_END)), SPAN_AT);
        assert_eq!(
            r.snapshot(SPAN_AT).expect("observing").last_frame_age_ms,
            None,
            "an equal newest stamp is not a publish and not a regression"
        );

        // The FIRST batch drawn purely from run 2 DOES regress — but it lands one
        // drain cadence behind the spanning batch, which re-anchored the silence.
        // Indistinguishable from a re-flush chunk at that spacing, so it is banked.
        r.record_frames(drained(1, Some(6 * MS)), SPAN_AT + 10 * MS);
        assert_eq!(
            r.max_stamp_ns,
            Some(RUN1_END),
            "the extra cost the silence gate adds: no reset yet, the dead run's \
             maximum still stands"
        );
        assert_eq!(
            r.snapshot(SPAN_AT + 10 * MS)
                .expect("observing")
                .last_frame_age_ms,
            None
        );

        // The reset lands on the first regression that IS preceded by silence — a
        // lull in the restarted run's stream (or, on a topic that never pauses, the
        // residual the module docs state).
        const LULL_AT: u64 = SPAN_AT + 10 * MS + REGRESSION_RESET_MIN_GAP_NS;
        r.record_frames(drained(1, Some(7 * MS)), LULL_AT);
        assert_eq!(
            r.max_stamp_ns,
            Some(7 * MS),
            "and there the epoch really does reset onto the new run's number line"
        );
        assert_eq!(
            r.snapshot(LULL_AT).expect("observing").last_frame_age_ms,
            None,
            "the resetting batch never dates the topic itself"
        );
        r.record_frames(drained(1, Some(8 * MS)), LULL_AT + 10 * MS);
        assert_eq!(
            r.snapshot(LULL_AT + 10 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(0),
            "the reset arrives late, never not at all"
        );
    }

    // ---------------------------------------------------------------------
    // The SUSTAINED-regression reset path.
    //
    // Every script below keeps consecutive frame-yielding drains STRICTLY closer
    // together than `REGRESSION_RESET_MIN_GAP_NS`, so the silence gate is never
    // satisfied and the only thing that can reset an epoch is the new path. That
    // is asserted structurally (const-asserted cadence) rather than trusted.
    // ---------------------------------------------------------------------

    /// The OBSERVER plane's drain grid — the cadence every script below
    /// runs at. Deliberately BELOW the silence gate, so no script can accidentally
    /// clear it (const-asserted at each use).
    const CADENCE: u64 = LIVENESS_SWEEP_INTERVAL_NS;
    const _: () = assert!(
        CADENCE < REGRESSION_RESET_MIN_GAP_NS,
        "sustained-path precondition: the scripts must drain FASTER than \
         the silence gate, or the gate could clear on its own and every oracle \
         below would be measuring the wrong path"
    );
    // The scripts put their confirming drain exactly on the span boundary, which
    // is only expressible on the cadence grid if the span is a whole number of
    // cadences.
    const _: () = assert!(
        SUSTAINED_REGRESSION_MIN_SPAN_NS.is_multiple_of(CADENCE),
        "sustained-path precondition: the scripts below place the confirming drain at \
         exactly `SUSTAINED_REGRESSION_MIN_SPAN_NS` after the run opened, which \
         requires the span to land on the cadence grid"
    );
    /// How many cadences of regime the span condition costs.
    const SPAN_CADENCES: u64 = SUSTAINED_REGRESSION_MIN_SPAN_NS / CADENCE;

    /// THE HEADLINE (pure): a publisher restarts WITHOUT a drain-visible
    /// lull and keeps streaming, and the topic recovers a dated reading.
    ///
    /// This is the shape the silence gate can never judge, and it is not exotic:
    /// systemd brings a crashed node back in ~100 ms, and any topic publishing at
    /// least once per drain re-anchors the gate on every drain. Every drain in the
    /// script below yields frames one CADENCE apart — const-asserted BELOW
    /// [`REGRESSION_RESET_MIN_GAP_NS`] — so `quiet_long_enough` is false at every
    /// single one of them and the reset can only come from the sustained path.
    ///
    /// WITHOUT THE SUSTAINED PATH THIS NEVER HEALS. The maximum stays on the dead run's
    /// high-water mark for as long as the restarted publisher keeps streaming, and
    /// the row serves the DEAD run's `last_frame_at_ns` growing a second every
    /// second — the affirmatively-wrong freshness claim the module docs describe.
    ///
    /// What breaks this test: drop `|| sustained` from the reset arm ⇒
    /// the `max_stamp_ns` assertion at the confirming drain reads the dead run's
    /// two-hour stamp, and the verdict one drain later is `None`/`Idle` instead of
    /// `Some(0)`/`Streaming`.
    #[test]
    fn a_lull_free_restart_resets_the_epoch_once_the_regression_is_sustained() {
        // Two hours of run 1, on its own clock — the restart shape.
        const RUN1_END: u64 = 2 * 60 * 60 * 1_000 * MS;
        const RUN2_BASE: u64 = 5 * MS;
        const _: () = assert!(
            RUN2_BASE < RUN1_END,
            "precondition: the replacement's clock begins BELOW the dead run's \
             maximum, which is what a VirtualClock restarting at 0 does"
        );
        // The last drain of run 1, and therefore the instant the row's age dates
        // from for the whole of the regime below.
        const RUN1_DATED_AT: u64 = 2 * CADENCE;
        const REGIME_START: u64 = RUN1_DATED_AT + CADENCE;
        const CONFIRM_AT: u64 = REGIME_START + SUSTAINED_REGRESSION_MIN_SPAN_NS;

        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(1, RUN1_END - 100 * MS), CADENCE); // baseline
        r.record_frames(drained_one_writer(1, RUN1_END), RUN1_DATED_AT); // advances
        assert_eq!(
            r.snapshot(RUN1_DATED_AT)
                .expect("observing")
                .last_frame_age_ms,
            Some(0),
            "precondition: run 1 dates normally"
        );

        // THE RESTART, with no lull whatsoever: the replacement's frames reach the
        // very next drain, and every drain after it.
        let mut at = REGIME_START;
        let mut stamp = RUN2_BASE;
        loop {
            r.record_frames(drained_one_writer(1, stamp), at);
            if at < CONFIRM_AT {
                assert_eq!(
                    r.max_stamp_ns,
                    Some(RUN1_END),
                    "the regime has not yet earned a reset, so the dead run's bar \
                     still stands at {at} ns"
                );
                assert_eq!(
                    r.snapshot(at).expect("observing").last_frame_age_ms,
                    Some((at - RUN1_DATED_AT) / MS),
                    "and meanwhile the row serves the DEAD run's age, growing — \
                     which is exactly the wrong claim the sustained path bounds"
                );
                at += CADENCE;
                stamp += MS;
                continue;
            }
            break;
        }
        assert_eq!(at, CONFIRM_AT, "the loop stops on the span boundary");
        assert_eq!(
            r.max_stamp_ns,
            Some(stamp),
            "THE RESET: with no silence available anywhere in this script, the \
             SUSTAINED regression is what moves the record onto the new epoch's \
             number line"
        );
        assert_eq!(
            r.snapshot(CONFIRM_AT).expect("observing").last_frame_age_ms,
            Some((CONFIRM_AT - RUN1_DATED_AT) / MS),
            "the resetting batch itself never dates — it may be the new run's own \
             retained-history flush"
        );

        // ONE advancement later the row is live again — the whole point.
        let dated_at = CONFIRM_AT + CADENCE;
        r.record_frames(drained_one_writer(1, stamp + MS), dated_at);
        let l = r.snapshot(dated_at).expect("observing");
        assert_eq!(
            l.last_frame_age_ms,
            Some(0),
            "WITHOUT THE SUSTAINED PATH THIS IS THE DEAD RUN'S AGE, FOREVER: {l:?}"
        );
        assert_eq!(l.state(), LivenessState::Streaming);
        assert_eq!(
            l.frames_observed,
            SPAN_CADENCES + 4,
            "two run-1 batches, the whole regime, and the batch that dated it"
        );

        // And it keeps dating as the new run climbs: the reset spent its evidence,
        // it did not leave a standing authorization.
        r.record_frames(drained_one_writer(1, stamp + 2 * MS), dated_at + CADENCE);
        assert_eq!(
            r.snapshot(dated_at + CADENCE)
                .expect("observing")
                .last_frame_age_ms,
            Some(0),
            "the record genuinely lives on the new epoch now"
        );
    }

    /// THE ADVERSARIAL TWIN: a re-flushed backlog drained in PREFIX chunks can run
    /// for as long as it likes and never sustain, because a prefix chunk leaves the
    /// queue NON-empty by construction — the rest of the burst is still in it.
    ///
    /// The script is the maximally hostile version of the module docs' mid-flush
    /// interleave: the first chunk empties the queue MID-BURST (closing the baseline, so
    /// guard 1 is out of the way), and the rest of the backlog then arrives on a
    /// closed baseline, one cadence apart, ascending, single-writer, for well past
    /// [`SUSTAINED_REGRESSION_MIN_SPAN_NS`]. Only the emptied-drains condition
    /// stands.
    ///
    /// What breaks this test: drop the
    /// `>= SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS` conjunct ⇒ the chunk on the
    /// span boundary confirms and LOWERS the bar, the emptied chunk after it closes
    /// the re-opened baseline on the lowered bar, and the backlog's own tail then
    /// out-ranks it and renders the dead route `Streaming`. Both the bar assertion
    /// and the verdict assertion fail.
    #[test]
    fn a_re_flushed_backlog_drained_in_prefix_chunks_never_sustains() {
        // The route published a backlog topping out at BACKLOG_NEWEST and DIED.
        const BACKLOG_NEWEST: u64 = 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(4, BACKLOG_NEWEST), 10 * MS);
        assert_eq!(
            r.snapshot(10 * MS).expect("observing").last_frame_age_ms,
            None,
            "the first flush is the baseline — a dead-but-history-carrying route"
        );

        // A hand-off re-flushes the SAME backlog into a fresh connection.
        r.begin_attached_observation(100 * MS);
        // First chunk: the mid-flush interleave — it EMPTIES the queue while
        // `deliver_history` is still pushing, which closes the baseline mid-burst
        // and hands every chunk behind it a closed one.
        r.record_frames(drained_one_writer(2, 100 * MS), 110 * MS);

        // The rest of the backlog, on a closed baseline, ascending, one cadence
        // apart, for TWICE the confirmation span.
        let mut at = 110 * MS + CADENCE;
        let mut stamp = 150 * MS;
        for _ in 0..(2 * SPAN_CADENCES) {
            r.record_frames(chunk_one_writer(2, stamp), at);
            assert_eq!(
                r.max_stamp_ns,
                Some(BACKLOG_NEWEST),
                "THE BAR: a prefix chunk never empties the queue, so however long \
                 the burst takes it can never sustain (at {at} ns)"
            );
            at += CADENCE;
            stamp += 50 * MS;
        }
        // A chunk that DOES empty the queue — one is all the interleave can supply,
        // and one is not enough.
        r.record_frames(drained_one_writer(2, stamp), at);
        assert_eq!(
            r.max_stamp_ns,
            Some(BACKLOG_NEWEST),
            "one emptied drain is exactly what the documented interleave produces, \
             and the threshold is deliberately TWO"
        );
        at += CADENCE;

        // The backlog's own TAIL, stamped exactly where the first flush topped out.
        r.record_frames(drained_one_writer(2, BACKLOG_NEWEST), at);
        let l = r.snapshot(at).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "THE VERDICT: a re-flush of the SAME backlog can never date a route \
             that has been dead since the first flush: {l:?}"
        );
        assert_eq!(
            l.state(),
            LivenessState::Idle,
            "produced, undatable — never the Streaming a lowered bar would fake"
        );
    }

    /// THE EQUALITY BOUNDARY, which is what makes the whole path safe: a re-flush's
    /// TAIL carries the maximum EXACTLY (a single-writer publisher's retained
    /// history keeps its NEWEST frames, so the frame that SET the maximum is in the
    /// backlog), and an equal stamp is neither an advance nor a regression — so a
    /// backlog TERMINATES ITS OWN RUN rather than extending it.
    ///
    /// Without that, evidence would accumulate across a burst and across the
    /// silence behind it, and the next prefix chunk of ANY later flush could finish
    /// a confirmation the previous one started.
    ///
    /// What breaks this test: make the not-regressed arm of
    /// `note_regression_run` a no-op instead of a clear ⇒ the run opened by the first chunk
    /// survives its own tail, the drain a full span later confirms against it, the
    /// bar drops to that chunk's stamp, and the drain after it ADVANCES and dates a
    /// dead route (`Some(0)` / `Streaming`).
    #[test]
    fn a_re_flush_tail_at_exactly_the_maximum_clears_its_own_run() {
        const BACKLOG_NEWEST: u64 = 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(4, BACKLOG_NEWEST), 10 * MS); // baseline

        // The first chunk of a re-flush, on a closed baseline, emptying the queue: this is
        // the run the tail must terminate.
        let a_at = 10 * MS + CADENCE;
        r.record_frames(drained_one_writer(2, 200 * MS), a_at);
        // The TAIL, stamped exactly at the maximum: equal, so it clears the run.
        r.record_frames(drained_one_writer(2, BACKLOG_NEWEST), a_at + CADENCE);
        assert_eq!(
            r.max_stamp_ns,
            Some(BACKLOG_NEWEST),
            "the tail neither advances nor regresses"
        );

        // More sub-max drains, one cadence apart so the SILENCE gate stays shut,
        // carried right up to a full span after the first chunk opened ITS run. If the tail
        // had not cleared that run, this stretch would finish its confirmation.
        let mut at = a_at + 2 * CADENCE;
        let mut stamp = 300 * MS;
        while at <= a_at + SUSTAINED_REGRESSION_MIN_SPAN_NS {
            r.record_frames(drained_one_writer(2, stamp), at);
            assert_eq!(
                r.max_stamp_ns,
                Some(BACKLOG_NEWEST),
                "THE BAR: the run these drains belong to opened AFTER the tail, so \
                 none of them is a span old (at {at} ns)"
            );
            at += CADENCE;
            stamp += 100 * MS;
        }
        // And the drain after that would ADVANCE past a lowered bar.
        r.record_frames(drained_one_writer(2, stamp), at);
        let l = r.snapshot(at).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "THE VERDICT: nothing here dates the dead route: {l:?}"
        );
        assert_eq!(l.state(), LivenessState::Idle);
    }

    /// The SINGLE-WRITER gate, pinned in all three directions plus the mixed case.
    ///
    /// A topic with two LIVE local writers (`multi_publisher_topics`, e.g. `/tf`)
    /// interleaves independent clocks, so its regressions are ROUTINE rather than
    /// evidence of a restart. It is excluded because it reports ≥ 2 on essentially
    /// every drain, so every batch RESTARTS the run and none ever lives long enough
    /// to confirm. `None` — a drainer that did not ask — is treated identically:
    /// absence of evidence is not evidence of a single writer.
    ///
    /// The MIXED arm is the one this file gets wrong if the premise is treated as
    /// a sticky poison rather than a restart. Note it can only
    /// show the run failing to confirm INSIDE this short regime; the arm that shows
    /// a transient dissent must not disable healing FOREVER is
    /// `a_transient_multi_writer_reading_costs_one_window_not_the_whole_run`.
    ///
    /// What breaks this test: drop `obs.writers_seen == Some(1)` from
    /// the run-EXTEND guard (so a non-single-writer batch neither restarts nor
    /// blocks) ⇒ the three non-single answers each adopt and fail.
    #[test]
    fn only_a_single_writer_topic_takes_the_sustained_path() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        /// Runs the lull-free regime with `writers` reported on every drain except
        /// the one at index `dissent`, which reports two writers.
        fn max_after_regime(writers: Option<u32>, dissent: Option<u64>) -> Option<u64> {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline
            for step in 0..=SPAN_CADENCES {
                let seen = if dissent == Some(step) {
                    Some(2)
                } else {
                    writers
                };
                let obs = DrainObservation {
                    writers_seen: seen,
                    ..drained_one_writer(1, 10 * MS + step * MS)
                };
                r.record_frames(obs, 2 * CADENCE + step * CADENCE);
            }
            r.max_stamp_ns
        }
        assert_eq!(
            max_after_regime(Some(1), None),
            Some(10 * MS + SPAN_CADENCES * MS),
            "THE CONTROL: a graph-owned single-writer topic — what both production \
             drainers report — takes the path and adopts the new epoch"
        );
        assert_eq!(
            max_after_regime(Some(2), None),
            Some(OLD_MAX),
            "a `multi_publisher_topics` topic is excluded: its regressions span \
             independent clocks and are no evidence of a restart"
        );
        assert_eq!(
            max_after_regime(None, None),
            Some(OLD_MAX),
            "a drain that did not ask reports NO evidence, which is not evidence \
             of a single writer"
        );
        assert_eq!(
            max_after_regime(Some(1), Some(SPAN_CADENCES / 2)),
            Some(OLD_MAX),
            "ONE multi-writer observation anywhere in the run disqualifies the \
             whole run — by then the maximum has already been compared across two \
             publishers' clocks"
        );
    }

    /// The SPAN condition is a threshold, pinned on both sides. Identical drain
    /// scripts, identical evidence, differing ONLY in where the judged drain lands
    /// relative to the run's start.
    #[test]
    fn the_sustained_span_is_a_threshold_pinned_on_both_sides() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        const RUN_START: u64 = 2 * CADENCE;
        const JUDGED_STAMP: u64 = 900 * MS;
        const _: () = assert!(
            JUDGED_STAMP < OLD_MAX,
            "precondition: the judged batch must still REGRESS"
        );
        /// Fills the regime at CADENCE and puts the judged drain exactly `span`
        /// after the run opened. Every gap stays under the silence gate.
        fn max_after_run_of_span(span: u64) -> Option<u64> {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline
            let mut at = RUN_START;
            let mut stamp = 10 * MS;
            while at + CADENCE < RUN_START + span {
                r.record_frames(drained_one_writer(1, stamp), at);
                at += CADENCE;
                stamp += MS;
            }
            r.record_frames(drained_one_writer(1, stamp), at);
            assert!(
                RUN_START + span - at < REGRESSION_RESET_MIN_GAP_NS,
                "the judged drain must land INSIDE the silence gate, or the gate \
                 would clear on its own and this would measure nothing"
            );
            r.record_frames(drained_one_writer(1, JUDGED_STAMP), RUN_START + span);
            r.max_stamp_ns
        }
        assert_eq!(
            max_after_run_of_span(SUSTAINED_REGRESSION_MIN_SPAN_NS - 1),
            Some(OLD_MAX),
            "one nanosecond short of the span: the regime has not earned a reset"
        );
        assert_eq!(
            max_after_run_of_span(SUSTAINED_REGRESSION_MIN_SPAN_NS),
            Some(JUDGED_STAMP),
            "exactly at the span: `>=`, so the restart reading wins here"
        );
    }

    /// The EMPTIED-DRAINS condition is a threshold too, and it is the STRUCTURAL
    /// one: a drain that empties the queue has caught up with everything the
    /// publisher had for us, so a SECOND emptied drain still below the bar means
    /// the publisher produced MORE after that — which a dead route cannot do.
    ///
    /// Both sides driven over the SAME span, so the span cannot be what separates
    /// them.
    #[test]
    fn the_sustained_emptied_drain_count_is_a_threshold_pinned_on_both_sides() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        const RUN_START: u64 = 2 * CADENCE;
        const FIRST_STAMP: u64 = 10 * MS;
        /// Runs a regime spanning twice the confirmation span in which exactly
        /// `emptied` of the drains left the queue empty (the earliest ones), and
        /// reports the maximum AT EACH STEP — so the caller can assert WHEN the
        /// reset landed, not merely that one eventually did.
        ///
        /// (Returning only the final maximum would be
        /// span-blind. Once the reset fires, the following chunks raise the
        /// maximum by ordinary advancement under the re-opened baseline, so the
        /// final value is identical whether the reset landed at step 0, 2, 5 or 9
        /// — MEASURED. Such an oracle pins "SOME reset happened", not that the
        /// span decides the outcome, and a change to the span alone does not
        /// fail it.)
        fn max_by_step(emptied: u32) -> Vec<Option<u64>> {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline
            let mut at = RUN_START;
            let mut stamp = FIRST_STAMP;
            let mut out = Vec::new(); // hot-path-alloc-ok: test-only collector.
            for step in 0..(2 * SPAN_CADENCES) {
                let obs = if (step as u32) < emptied {
                    drained_one_writer(1, stamp)
                } else {
                    chunk_one_writer(1, stamp)
                };
                r.record_frames(obs, at);
                out.push(r.max_stamp_ns);
                at += CADENCE;
                stamp += MS;
            }
            out
        }
        let short = max_by_step(SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS - 1);
        assert!(
            short.iter().all(|m| *m == Some(OLD_MAX)),
            "one emptied drain short: this is what a re-flush's single documented \
             mid-flush interleave supplies, and it must not be enough — at ANY \
             step of a regime twice the confirmation span: {short:?}"
        );

        // At the threshold the regime confirms, and it must confirm ON THE SPAN
        // BOUNDARY: the run opens at step 0, so step `SPAN_CADENCES` is the first
        // drain at or past `SUSTAINED_REGRESSION_MIN_SPAN_NS`.
        let met = max_by_step(SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS);
        let boundary = SPAN_CADENCES as usize;
        assert!(
            met[..boundary].iter().all(|m| *m == Some(OLD_MAX)),
            "the count was met on step 1, but every step BEFORE the span boundary \
             must still stand on the dead run's bar: {met:?}"
        );
        assert_eq!(
            met[boundary],
            Some(FIRST_STAMP + SPAN_CADENCES * MS),
            "and the reset lands EXACTLY on the span boundary, adopting that \
             drain's own stamp — the span is what binds once the count is met"
        );
    }

    /// The property the whole poison class turns on: a transient
    /// non-single-writer reading must cost ONE confirmation window, not the topic's
    /// whole regime.
    ///
    /// `only_the_single_writer_topic_takes_the_sustained_path`'s mixed arm cannot
    /// see this: it reads the outcome after a regime only `SPAN_CADENCES` long, so
    /// a restarted run is a span short either way and BOTH designs answer
    /// `Some(OLD_MAX)`. This arm runs the regime PAST the span after the dissent
    /// and requires the topic to heal.
    ///
    /// Reachable: a second publisher port is legal on an
    /// `PublisherProvisioning::External` absolute-source topic (iceoryx2's
    /// create-default 2 slots) and on `multi_publisher_topics` topics, so one
    /// transient attach anywhere inside a restart's regression regime is enough.
    ///
    /// MEASURED with a run that a dissent poisons for good: 500 further flawless single-writer drains
    /// (100 s — 100× the confirmation span) left `max_stamp_ns` on the dead run's
    /// 2 h stamp with no reset at all.
    ///
    /// It also pins where the healed run begins: a
    /// non-single-writer batch contributes NOTHING (no timestamp, no ascent bar, no
    /// emptied credit) and the run anchors at the first CLEAN drain AFTER it.
    /// Letting the dissenting batch ANCHOR the run would leave the
    /// anchoring drain exempt from the very premise condition 1 states — a foreign
    /// writer could start the span clock, supply the ascent bar from its own stamp,
    /// and donate one of the two required emptied credits. The exact-instant oracle
    /// below is what tells the two apart: anchoring on the dissent resets at
    /// `DISSENT_STEP + SPAN_CADENCES`, anchoring on the first clean drain at
    /// `DISSENT_STEP + 1 + SPAN_CADENCES` — one drain cadence later, MEASURED.
    ///
    /// What breaks this test: restore the sticky
    /// `regression_run_trusted &= single_writer` (re-armed only at a run open) ⇒
    /// this arm fails with the dead run's bar still standing (`reset_at: None`),
    /// while every other liveness test stays green. Drop `writers_seen == Some(1)`
    /// from the early-return filter so a foreign batch can anchor ⇒ this arm fails
    /// one cadence early.
    #[test]
    fn a_transient_multi_writer_reading_costs_one_window_not_the_whole_run() {
        const OLD_MAX: u64 = 2 * 60 * 60 * 1_000 * MS;
        const RUN_START: u64 = 2 * CADENCE;
        // A dissent EARLY in the regime, so the run it restarts still has room to
        // run a full span inside the script.
        const DISSENT_STEP: u64 = 1;
        for bad in [None, Some(0u32), Some(2u32)] {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline
            let mut reset_at: Option<u64> = None;
            // Run well past a span BEYOND the dissent, so a design that merely
            // delays cannot be mistaken for one that never heals.
            for step in 0..(3 * SPAN_CADENCES) {
                let at = RUN_START + step * CADENCE;
                let stamp = 10 * MS + step * MS;
                let obs = if step == DISSENT_STEP {
                    DrainObservation {
                        writers_seen: bad,
                        ..drained_one_writer(1, stamp)
                    }
                } else {
                    drained_one_writer(1, stamp)
                };
                r.record_frames(obs, at);
                if reset_at.is_none() && r.max_stamp_ns != Some(OLD_MAX) {
                    reset_at = Some(at);
                }
            }
            // The dissenting batch contributes NOTHING, so the run anchors at the
            // FIRST CLEAN drain after it and the reset lands exactly one span after
            // THAT — a delay of one window plus one cadence, not a permanent
            // disqualification.
            assert_eq!(
                reset_at,
                Some(RUN_START + (DISSENT_STEP + 1 + SPAN_CADENCES) * CADENCE),
                "a {bad:?} reading must cost exactly one confirmation window and \
                 anchor the healed run at the first CLEAN drain — IF A DISSENT POISONS THE RUN THIS IS \
                 `None` FOREVER, and with the dissent allowed to ANCHOR it is one \
                 cadence early"
            );
        }
    }

    /// A stamp that drops below the run's own first must re-anchor the run,
    /// because the ascent bar it would otherwise be measured against belongs to an
    /// epoch that is already dead.
    ///
    /// The shape is a crash-looping supervisor: the replacement is first observed
    /// at publisher-clock 300 ms (routine — the observer's grid is 200 ms and a
    /// worker's clock is at whatever it is when the sweep lands), then each
    /// incarnation publishes 50/100/150 ms and dies. Every batch regresses against
    /// the ancient maximum, so nothing ever clears the run, and no batch exceeds
    /// 300 ms — so with a frozen bar the topic NEVER heals.
    ///
    /// MEASURED with a frozen bar: 200 sweeps (40 s) of a genuinely live,
    /// single-writer, queue-emptying producer, `max_stamp_ns` still `Some(2 h)`.
    ///
    /// What breaks this test: weaken the run-EXTEND guard's
    /// `stamp >= self.regression_run_first_stamp_ns` to `true` ⇒ this arm fails
    /// with the dead run's bar standing.
    #[test]
    fn a_stamp_below_the_runs_own_first_re_anchors_the_run() {
        const OLD_MAX: u64 = 2 * 60 * 60 * 1_000 * MS;
        const FIRST_SEEN: u64 = 300 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline
        r.record_frames(drained_one_writer(1, FIRST_SEEN), 2 * CADENCE);

        let mut at = 2 * CADENCE;
        let mut reset_at: Option<u64> = None;
        for step in 0..(4 * SPAN_CADENCES) {
            at += CADENCE;
            // Below the run's own first stamp, cycling — a crash loop.
            let stamp = (50 + 50 * (step % 3)) * MS;
            assert!(stamp < FIRST_SEEN, "precondition: the loop never ascends");
            r.record_frames(drained_one_writer(1, stamp), at);
            if reset_at.is_none() && r.max_stamp_ns != Some(OLD_MAX) {
                reset_at = Some(at);
            }
        }
        assert!(
            reset_at.is_some(),
            "WITH A FROZEN BAR THIS IS `None` FOREVER: the ascent bar frozen at a DEAD \
             epoch's 300 ms stamp is unreachable by a live publisher whose \
             incarnations all stamp below it"
        );
    }

    /// The observer plane's re-flush shape, where condition 2 is
    /// STRUCTURALLY INERT and the SPAN is the sole defence.
    ///
    /// `a_re_flushed_backlog_drained_in_prefix_chunks_never_sustains` models the
    /// DEMAND plane only: its chunks carry `queue_emptied: false`, which is what a
    /// budget-limited drainer reports. The observer's own drain loops to a SHORT
    /// READ, so on that plane EVERY frame-yielding drain reports `queue_emptied:
    /// true` and the emptied-drain count degenerates to "there were >= 2 drains".
    /// Nothing pinned that asymmetry, and the module docs called condition 2
    /// "structural" without qualifying the plane.
    ///
    /// So this is the observer-plane twin, built from `drained_one_writer` (every
    /// chunk empties) and held INSIDE the span: the bar must stand at every step.
    /// What keeps the same shape safe in production once it exceeds the span is
    /// stated at `SUSTAINED_REGRESSION_MIN_EMPTIED_DRAINS` — a `deliver_history`
    /// push completes in microseconds-to-milliseconds, so a burst cannot supply an
    /// ascending sub-max chunk on drain after drain for a whole second.
    ///
    /// What breaks this test: neutralise the span condition ⇒ this arm
    /// fails on its second chunk (the count alone is met there).
    #[test]
    fn an_observer_plane_re_flush_is_refused_by_the_span_alone() {
        const BACKLOG_NEWEST: u64 = 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(4, BACKLOG_NEWEST), 10 * MS);
        // A hand-off, then the mid-flush interleave: the first chunk empties the queue
        // while `deliver_history` is still pushing, closing the baseline mid-burst.
        r.begin_attached_observation(100 * MS);
        r.record_frames(drained_one_writer(2, 100 * MS), 110 * MS);

        // The rest of the backlog on a CLOSED baseline, ascending, single-writer,
        // and every chunk EMPTYING the queue — the observer plane's real drain
        // shape. Held to one cadence short of the span.
        let run_start = 110 * MS + CADENCE;
        let mut at = run_start;
        let mut stamp = 105 * MS;
        while at < run_start + SUSTAINED_REGRESSION_MIN_SPAN_NS {
            r.record_frames(drained_one_writer(2, stamp), at);
            assert_eq!(
                r.max_stamp_ns,
                Some(BACKLOG_NEWEST),
                "THE BAR: on the observer plane the emptied-drain count is met on \
                 the run's SECOND chunk, so the SPAN is the only thing refusing \
                 this burst (at {at} ns)"
            );
            at += CADENCE;
            stamp += 5 * MS;
        }
        // The backlog's own TAIL, stamped exactly where the first flush topped out.
        r.record_frames(drained_one_writer(2, BACKLOG_NEWEST), at);
        let l = r.snapshot(at).expect("observing");
        assert_eq!(
            l.last_frame_age_ms, None,
            "THE VERDICT: the dead route is never dated: {l:?}"
        );
        assert_eq!(l.state(), LivenessState::Idle);
    }

    /// The demand-plane twin of the arm above — a re-flush that supplies
    /// TWO emptied prefix drains (two mid-flush interleaves in one burst) is still
    /// refused, by the span.
    ///
    /// The question is whether the composite's weaker half is
    /// load-bearing on prose alone: no arm anywhere drove `>= 2` emptied prefix
    /// drains. It does now, and the answer is that the span refuses it.
    #[test]
    fn a_re_flush_with_two_emptied_prefix_drains_is_refused_by_the_span() {
        const BACKLOG_NEWEST: u64 = 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(4, BACKLOG_NEWEST), 10 * MS);
        r.begin_attached_observation(100 * MS);
        // Interleave 1 closes the baseline mid-burst.
        r.record_frames(drained_one_writer(2, 100 * MS), 110 * MS);
        let run_start = 110 * MS + CADENCE;
        // The run opens here, and interleave 2 lands INSIDE it — so the emptied
        // count reaches the threshold while the span has barely begun.
        r.record_frames(drained_one_writer(2, 200 * MS), run_start);
        r.record_frames(drained_one_writer(2, 300 * MS), run_start + CADENCE);
        assert_eq!(
            r.max_stamp_ns,
            Some(BACKLOG_NEWEST),
            "TWO emptied prefix drains meet the count outright; only the span \
             stands, and it must"
        );
        // The rest of the burst as ordinary budget-limited chunks, then the tail.
        let mut at = run_start + 2 * CADENCE;
        let mut stamp = 400 * MS;
        while at < run_start + SUSTAINED_REGRESSION_MIN_SPAN_NS {
            r.record_frames(chunk_one_writer(2, stamp), at);
            assert_eq!(r.max_stamp_ns, Some(BACKLOG_NEWEST), "the bar holds");
            at += CADENCE;
            stamp += 50 * MS;
        }
        r.record_frames(drained_one_writer(2, BACKLOG_NEWEST), at);
        assert_eq!(
            r.snapshot(at).expect("observing").last_frame_age_ms,
            None,
            "and the backlog's own tail never dates the dead route"
        );
    }

    /// A tap-depth concern, refuted and pinned.
    ///
    /// The claim was that what keeps the observer plane safe is
    /// `LIVENESS_TAP_BUFFER_SIZE = 2` — that a shallow tap leaves the observer the
    /// backlog's SUFFIX rather than an ascending prefix — and that raising the
    /// depth would re-open the hole "outright". The depth is not what carries it,
    /// and the argument holds at EVERY depth: the tap is `drop_oldest`, so whatever
    /// it retains ENDS at the newest frame the publisher pushed, and on a
    /// single-writer topic that frame IS the one that set the maximum. A flush
    /// therefore arrives carrying `ts == M`, which is neither an advance nor a
    /// regression, so it CLEARS the run.
    ///
    /// Driven at three depths spanning the shipped one, each as a single
    /// drain-to-empty read (what `drain_topic` does): the run is cleared in every
    /// case and nothing dates.
    #[test]
    fn a_flush_taken_in_one_drain_carries_the_maximum_at_any_tap_depth() {
        const BACKLOG_NEWEST: u64 = 1_000 * MS;
        const T0: u64 = 10 * MS;
        for depth in [1u64, LIVENESS_TAP_BUFFER_SIZE as u64, 64] {
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(0);
            r.record_frames(drained_one_writer(4, BACKLOG_NEWEST), T0);

            // A run that is ONE cadence short of confirming. Every gap is a
            // cadence, so the silence gate is never in play.
            let mut stamp = 100 * MS;
            for k in 1..=SPAN_CADENCES {
                r.record_frames(drained_one_writer(1, stamp), T0 + k * CADENCE);
                assert_eq!(
                    r.max_stamp_ns,
                    Some(BACKLOG_NEWEST),
                    "depth {depth}: the run has not earned a reset yet"
                );
                stamp += 5 * MS;
            }

            // THE FLUSH, taken in ONE drain-to-empty read. However deep the tap,
            // `drop_oldest` retains the NEWEST `depth` frames, so the drain's
            // newest stamp is the backlog's top — which EQUALS the maximum, and an
            // equal stamp clears the run.
            let flush_at = T0 + (SPAN_CADENCES + 1) * CADENCE;
            r.record_frames(drained_one_writer(depth, BACKLOG_NEWEST), flush_at);
            assert_eq!(
                r.max_stamp_ns,
                Some(BACKLOG_NEWEST),
                "depth {depth}: the flush cannot lower the bar"
            );

            // The next sub-max drain would have confirmed the PRE-flush run (its
            // span is now past the threshold). It must instead open a fresh one.
            let after = flush_at + CADENCE;
            assert!(
                after - (T0 + CADENCE) >= SUSTAINED_REGRESSION_MIN_SPAN_NS,
                "precondition: the pre-flush run WOULD be old enough to confirm"
            );
            r.record_frames(drained_one_writer(1, stamp), after);
            assert_eq!(
                r.max_stamp_ns,
                Some(BACKLOG_NEWEST),
                "depth {depth}: the flush TERMINATED the run, so the drain after \
                 it does not inherit the span that run had accumulated"
            );
        }
    }

    /// A publisher whose stamps stand still is not producing, so a run that has not
    /// ASCENDED is not evidence of a live new epoch. Pinned on both sides in one
    /// body: the stalled regime never confirms however long it runs, and the very
    /// first batch that moves does.
    ///
    /// What breaks this test: drop the
    /// `stamp > regression_run_first_stamp_ns` conjunct ⇒ the stalled arm adopts.
    #[test]
    fn a_stalled_regression_never_sustains_but_one_that_moves_does() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        const STUCK: u64 = 10 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline

        let mut at = 2 * CADENCE;
        for _ in 0..(2 * SPAN_CADENCES) {
            r.record_frames(drained_one_writer(1, STUCK), at);
            assert_eq!(
                r.max_stamp_ns,
                Some(OLD_MAX),
                "a stalled publisher never sustains, however long it stalls \
                 (at {at} ns)"
            );
            at += CADENCE;
        }
        // The first batch that MOVES confirms the regime it has been building.
        r.record_frames(drained_one_writer(1, STUCK + 1), at);
        assert_eq!(
            r.max_stamp_ns,
            Some(STUCK + 1),
            "and one nanosecond of ascent is what the condition asks for"
        );
    }

    /// While the baseline is OPEN we are provably absorbing a burst that may be
    /// retained history, so nothing observed there is evidence of a new epoch —
    /// guard 1's reasoning applied to the EVIDENCE rather than to the decision.
    ///
    /// This is also what makes an explicit clear in `begin_attached_observation`
    /// redundant: the first frame-yielding batch after any attach lands on an open
    /// baseline, so a run cannot survive a hand-off. Both halves are asserted here.
    ///
    /// What breaks this test: drop the `!self.baseline_open` filter from
    /// `note_regression_run` ⇒ the chunks drained under the open baseline build a
    /// run spanning well past the confirmation window, and the first drain after
    /// the baseline closes confirms against it and lowers the bar.
    #[test]
    fn an_open_baseline_gathers_no_sustained_evidence() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline closes

        // A hand-off re-opens it, and the flush arrives as prefix chunks that keep
        // it open for well past the confirmation span.
        r.begin_attached_observation(2 * CADENCE);
        let mut at = 3 * CADENCE;
        let mut stamp = 10 * MS;
        for _ in 0..(2 * SPAN_CADENCES) {
            r.record_frames(chunk_one_writer(1, stamp), at);
            at += CADENCE;
            stamp += MS;
        }
        // The chunk that finally empties the queue CLOSES the baseline — and is
        // itself still under an open one, so it contributes nothing either.
        r.record_frames(drained_one_writer(1, stamp), at);
        at += CADENCE;
        stamp += MS;
        // The first TWO drains on the now-closed baseline. They open a fresh run
        // here, so neither can confirm: the span starts now.
        r.record_frames(drained_one_writer(1, stamp), at);
        assert_eq!(
            r.max_stamp_ns,
            Some(OLD_MAX),
            "THE BAR: the run opens at the first CLOSED-baseline batch, however \
             long the burst before it ran"
        );
        r.record_frames(drained_one_writer(1, stamp + MS), at + CADENCE);
        let l = r.snapshot(at + CADENCE).expect("observing");
        assert_eq!(
            r.max_stamp_ns,
            Some(OLD_MAX),
            "and its second drain is still a span short"
        );
        assert_eq!(
            l.last_frame_age_ms, None,
            "so nothing here dates the topic: {l:?}"
        );
    }

    /// A reset SPENDS the evidence behind it. Without that, the run that confirmed
    /// one reset would still be standing when the NEXT regression arrives — and
    /// that next regression is measured against the brand-new epoch, so a single
    /// reordered frame, or the restarted run's own retained-history tail, would
    /// confirm instantly on evidence gathered about a different epoch and drop the
    /// bar again.
    ///
    /// This arm exists because the first nine pass without this guard: deleting the
    /// reset arm's `clear_regression_run()` left all 76 liveness tests green, since
    /// every batch after a reset ADVANCES (and an advance clears the run on its own
    /// way through). Only a batch that regresses against the NEW maximum can see
    /// the difference.
    ///
    /// What breaks this test: delete `clear_regression_run()` from the
    /// reset arm ⇒ the jittered batch below confirms on the spent run and the bar
    /// drops to its stamp.
    #[test]
    fn a_sustained_reset_spends_its_evidence_rather_than_leaving_it_standing() {
        const OLD_MAX: u64 = 60 * 60 * 1_000 * MS;
        const REGIME_START: u64 = 2 * CADENCE;
        const CONFIRM_AT: u64 = REGIME_START + SUSTAINED_REGRESSION_MIN_SPAN_NS;
        const RUN2_BASE: u64 = 10 * MS;

        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained_one_writer(1, OLD_MAX), CADENCE); // baseline closes

        let mut at = REGIME_START;
        let mut stamp = RUN2_BASE;
        while at <= CONFIRM_AT {
            r.record_frames(drained_one_writer(1, stamp), at);
            at += CADENCE;
            stamp += MS;
        }
        let new_epoch = stamp - MS;
        assert_eq!(
            r.max_stamp_ns,
            Some(new_epoch),
            "precondition: the lull-free regime really did reset the epoch"
        );

        // A frame that REGRESSES against the NEW maximum: a reorder, or the
        // restarted run's own retained-history tail. It must open a FRESH run.
        let jitter = new_epoch - MS;
        assert!(
            jitter > RUN2_BASE,
            "precondition: the jittered stamp still ASCENDS past the SPENT run's \
             first stamp, so only the spent evidence could confirm it"
        );
        r.record_frames(drained_one_writer(1, jitter), at);
        assert_eq!(
            r.max_stamp_ns,
            Some(new_epoch),
            "a reset spends its evidence — the next regression starts a fresh run \
             rather than confirming on a run gathered about the previous epoch"
        );
    }

    /// INTERPLAY: a SUSTAINED reset must re-anchor the rate window exactly
    /// as a silence-gated one does, or the window still open from the DEAD run
    /// would close across the restart and serve a number counted over two epochs.
    ///
    /// The sequence guard cannot be what saves it — the restarted run's sequence is
    /// deliberately kept INSIDE [`RATE_SEQ_RESET_TOLERANCE`] of the anchor, exactly
    /// as `an_epoch_reset_batch_re_anchors_the_rate_window_and_contributes_nothing`
    /// does for the silence path — so only the reset's own `anchor_only` term can.
    ///
    /// What breaks this test: have the sustained arm set the maximum
    /// without reporting `epoch_reset` ⇒ dating still recovers, but the pre-restart
    /// window closes over its own span and the served rate is 4 Hz, not 5.
    #[test]
    fn a_sustained_reset_re_anchors_the_rate_window_and_contributes_nothing() {
        const RUN1_STAMP_A: u64 = 5_000 * MS;
        const RUN1_STAMP_B: u64 = 6_000 * MS;
        const ANCHOR_SEQ: u32 = 1_000;
        let mut r = attached_at(0);
        r.record_frames(drained_seq(2, RUN1_STAMP_A, ANCHOR_SEQ), CADENCE); // baseline + anchor
        r.record_frames(drained_seq(2, RUN1_STAMP_B, ANCHOR_SEQ + 2), 2 * CADENCE);

        // THE RESTART, lull-free: stamps regress hard onto a fresh clock while the
        // SEQUENCE lands where the rate's own reset tolerance cannot object.
        const REGIME_START: u64 = 3 * CADENCE;
        const CONFIRM_AT: u64 = REGIME_START + SUSTAINED_REGRESSION_MIN_SPAN_NS;
        assert!(
            ANCHOR_SEQ.saturating_sub(ANCHOR_SEQ) <= RATE_SEQ_RESET_TOLERANCE,
            "precondition: the restarted run's sequence is INSIDE the rate \
             tolerance, so the sequence guard cannot be what re-anchors this window"
        );
        let mut at = REGIME_START;
        let mut stamp = 10 * MS;
        let mut seq = ANCHOR_SEQ;
        while at <= CONFIRM_AT {
            r.record_frames(drained_seq(2, stamp, seq), at);
            at += CADENCE;
            stamp += MS;
            seq += 1;
        }
        assert_eq!(
            r.max_stamp_ns,
            Some(stamp - MS),
            "precondition: the sustained path really did reset the epoch here"
        );
        // The window the reset anchored runs from CONFIRM_AT; drive it to its
        // minimum span, one commit per cadence.
        let close_at = CONFIRM_AT + RATE_ESTIMATE_MIN_WINDOW_NS;
        while at <= close_at {
            r.record_frames(drained_seq(2, stamp, seq), at);
            at += CADENCE;
            stamp += MS;
            seq += 1;
        }
        let l = r.snapshot(close_at).expect("observing");
        assert_eq!(
            l.state(),
            LivenessState::Streaming,
            "precondition: the restarted topic dates again, so a rate is servable"
        );
        let commits = RATE_ESTIMATE_MIN_WINDOW_NS / CADENCE;
        assert_eq!(
            l.rate_estimate,
            Some(TopicRateEstimate {
                millihertz: commits * 1_000 * 1_000_000_000 / RATE_ESTIMATE_MIN_WINDOW_NS,
                is_floor: false,
            }),
            "the window must be counted from the RESET, not across it: {l:?}"
        );
    }

    /// THE OTHER DOCUMENTED RESIDUAL: a `multi_publisher_topics` topic (`/tf`)
    /// interleaves two disagreeing clocks, so regressions are ROUTINE. The bounded
    /// consequence is asserted here rather than asserted away: such a topic can
    /// read `Idle` where a single-writer one would read `Streaming`, and it is
    /// NEVER dimmed.
    ///
    /// The stimulus is a topic drained at cadence (10 ms here), so the silence gate
    /// BANKS every one of those regressions and the epoch never churns: the maximum
    /// stays on publisher A's number line, B's batches never date, and A's advance
    /// past A's own previous stamp. Churn survives only where a lower-stamping
    /// publisher lands after a [`REGRESSION_RESET_MIN_GAP_NS`] lull — rarer than
    /// pre-gate, and the verdict sequence is the same either way (see below).
    ///
    /// SCOPE: this is a RESIDUAL pin, not a mutation-discriminating one.
    /// Neither disabling the epoch reset nor letting it churn changes this oracle —
    /// the higher-stamping publisher's batches advance past a monotone maximum
    /// exactly as they advance past a reset one — which is precisely the point: the
    /// interleave costs freshness on the lower-stamping publisher's batches and
    /// nothing else. The reset itself is pinned by
    /// `a_publisher_restart_resets_the_stamp_epoch_and_dating_resumes`, and the
    /// gate by `the_silence_gate_is_a_threshold_pinned_on_both_sides`.
    #[test]
    fn interleaved_publisher_clocks_never_dim_the_row() {
        // Publisher A is an hour up its clock; publisher B started seconds ago.
        const A_BASE: u64 = 60 * 60 * 1_000 * MS;
        const B_BASE: u64 = 3 * 1_000 * MS;
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        let mut now = 0;
        let mut ages = Vec::new(); // hot-path-alloc-ok: test-only verdict collector.
        for step in 0..8u64 {
            now = (step + 1) * 10 * MS;
            // Strict alternation. Each B batch REGRESSES against A's maximum and
            // dates nothing — at this cadence the silence gate banks it, so the
            // maximum stays on A's number line — while each A batch advances past
            // A's own previous stamp and DOES date. That is the documented
            // over-report direction (it can over-report freshness on such a topic;
            // it can never dim one), and it is the same sequence the pre-gate
            // reset-churn produced.
            let stamp = if step % 2 == 0 {
                A_BASE + step * MS
            } else {
                B_BASE + step * MS
            };
            r.record_frames(drained(1, Some(stamp)), now);
            ages.push(r.snapshot(now).expect("observing").last_frame_age_ms);
        }
        // The HAND oracle: batch 0 is the baseline (undated); batch 1 is B's first
        // regression (banked, undated); every later A batch dates at drain time;
        // every B batch leaves the age growing by the 10 ms drain interval.
        assert_eq!(
            ages,
            // hot-path-alloc-ok: test-only hand oracle.
            vec![
                None,
                None,
                Some(0),
                Some(10),
                Some(0),
                Some(10),
                Some(0),
                Some(10)
            ],
            "the churn is over-reporting on the A batches, never a dim"
        );
        let l = r.snapshot(now).expect("observing");
        assert_eq!(
            l.frames_observed, 8,
            "every frame is banked — the churn costs freshness, never data"
        );
        assert_ne!(
            l.state(),
            LivenessState::NoData,
            "THE BOUND: `frames_observed > 0` forbids the dimmed row outright, \
             whatever the epoch does: {l:?}"
        );
        // Far past the settle threshold it is STILL Idle, never NoData.
        let settled = now + (LIVENESS_NO_DATA_MIN_MS + 1_000) * MS;
        assert_eq!(
            r.snapshot(settled).expect("observing").state(),
            LivenessState::Idle,
            "a churning /tf-class topic reads `Idle` (produced, freshness \
             unknown) — the accepted trade for the single-writer case"
        );
    }

    /// A zero-frame drain does NOT close the baseline. A retained-history flush
    /// rides the publisher's next `update_connections`, not our drain schedule, so
    /// an empty queue is no evidence that the burst already happened — closing on
    /// it would make the whole defence a race with the publisher's timing.
    #[test]
    fn an_empty_drain_does_not_close_the_baseline() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(0, None), 100 * MS);
        r.record_frames(drained(0, None), 200 * MS);
        // The flush finally arrives, long after several empty drains: it is still
        // the baseline.
        r.record_frames(drained(2, Some(0)), 300 * MS);
        assert_eq!(
            r.snapshot(300 * MS).expect("observing").last_frame_age_ms,
            None,
            "the first NONEMPTY batch is the baseline, whenever it lands"
        );
    }

    /// The baseline re-opens per ATTACH, so a backlog flushed into a SECOND
    /// connection is absorbed even though the record has been dating happily.
    /// That is what bounds the over-reporting when a tap is lost for a long time
    /// and the publisher produced (then died) during the gap.
    #[test]
    fn the_baseline_re_opens_on_every_attach() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(0)), 10 * MS); // baseline
        r.record_frames(drained(1, Some(20 * MS)), 20 * MS); // advances
        assert_eq!(
            r.snapshot(20 * MS).expect("observing").last_frame_age_ms,
            Some(0)
        );

        // The tap is lost for an hour; the publisher produced during the gap and
        // then died. The re-attach flushes those unseen frames at us.
        r.stop_observing(30 * MS);
        r.begin_attached_observation(3_600_000 * MS);
        r.record_frames(drained(3, Some(40 * MS)), 3_600_010 * MS);
        assert_eq!(
            r.snapshot(3_600_010 * MS)
                .expect("observing")
                .last_frame_age_ms,
            Some(3_599_990),
            "the age is still the PRE-gap arrival — an hour-old backlog flushed \
             into the new connection did not refresh it"
        );
        assert_eq!(
            r.snapshot(3_600_010 * MS)
                .expect("observing")
                .frames_observed,
            5
        );
    }

    /// `start_observing` (the per-pass affirmation) must NOT re-open the baseline
    /// — only a real attach does. Otherwise every affirmation would re-suspect the
    /// next batch and the age would never advance.
    #[test]
    fn a_plain_start_observing_does_not_re_open_the_baseline() {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(0);
        r.record_frames(drained(1, Some(0)), 10 * MS); // closes the baseline
        r.start_observing(20 * MS); // idempotent affirmation
        r.record_frames(drained(1, Some(5 * MS)), 30 * MS);
        assert_eq!(
            r.snapshot(30 * MS).expect("observing").last_frame_age_ms,
            Some(0),
            "an affirmation must not re-open the baseline"
        );
    }

    /// THE CROSS-CLOCK PROPERTY, pure: the verdict depends ONLY on the relation
    /// between successive publisher stamps — never on where those stamps sit
    /// relative to the observer's clock.
    ///
    /// This is the cross-clock hazard in oracle form. On the shipping
    /// multi-process deployment the publisher stamps a worker `VirtualClock` that
    /// starts at 0 while the observer reads the gateway's `RealClock` (ns since
    /// boot), so the two number lines are unrelated in BOTH directions. The three
    /// regimes below feed IDENTICAL stamp sequences at wildly different offsets
    /// from `now_ns` and must produce IDENTICAL verdicts.
    ///
    /// What breaks this test: reinstate any comparison of a wire stamp against the
    /// observer's clock (e.g. "date only frames stamped after the attach instant")
    /// and the far-below regime stops dating while the far-above regime dates its
    /// baseline — the two regimes diverge and this fails.
    #[test]
    fn the_advancement_rule_is_independent_of_the_observer_clock() {
        /// Runs the same three-batch script with publisher stamps based at
        /// `stamp_base`, while the OBSERVER clock runs from 1e12 ns (a plausible
        /// uptime) regardless.
        fn verdicts(stamp_base: u64) -> Vec<Option<u64>> {
            const OBS: u64 = 1_000_000 * MS; // ~16.6 min of observer uptime
            let mut r = LivenessRecord::default();
            r.begin_attached_observation(OBS);
            let mut out = Vec::new(); // hot-path-alloc-ok: test-only verdict collector.
                                      // (1) baseline burst, (2) advance, (3) no advance.
            r.record_frames(drained(2, Some(stamp_base)), OBS + 10 * MS);
            out.push(
                r.snapshot(OBS + 10 * MS)
                    .expect("observing")
                    .last_frame_age_ms,
            );
            r.record_frames(drained(1, Some(stamp_base + 5 * MS)), OBS + 20 * MS);
            out.push(
                r.snapshot(OBS + 20 * MS)
                    .expect("observing")
                    .last_frame_age_ms,
            );
            r.record_frames(drained(1, Some(stamp_base + 5 * MS)), OBS + 30 * MS);
            out.push(
                r.snapshot(OBS + 30 * MS)
                    .expect("observing")
                    .last_frame_age_ms,
            );
            out
        }
        // The HAND oracle: baseline undated, then dated at drain time, then the
        // repeated stamp leaves the age growing from that drain.
        let oracle = vec![None, Some(0), Some(10)]; // hot-path-alloc-ok: test-only oracle.
                                                    // A worker VirtualClock that started at 0 (stamps FAR BELOW the observer
                                                    // clock — a cross-clock comparison would degrade this regime to a blanket discard).
        assert_eq!(verdicts(0), oracle, "publisher clock near zero");
        // A worker that has stepped far past wall time (stamps FAR ABOVE — a cross-clock
        // comparison would make the gate inert and date the backlog).
        assert_eq!(
            verdicts(1_000_000_000 * MS),
            oracle,
            "publisher clock far ahead of the observer's"
        );
        // And a publisher that happens to share the observer's number line.
        assert_eq!(
            verdicts(1_000_000 * MS),
            oracle,
            "publisher clock coincident with the observer's"
        );
    }

    // -- LivenessState classification (PURE) --------------------------------

    fn live(age: Option<u64>, observed_for_ms: u64) -> TopicLiveness {
        TopicLiveness {
            last_frame_age_ms: age,
            observed_for_ms,
            frames_observed: u64::from(age.is_some()),
            rate_estimate: None,
        }
    }

    /// A record with NO datable frame, carrying `frames` banked ones. `frames > 0`
    /// is the produced-but-undated class (the flushed backlog, the latched
    /// one-shot, the baseline-only topic); `frames == 0` is its DISCRIMINATOR —
    /// the never-produced record that is allowed to settle into `NoData`. The
    /// helper is deliberately not named for either arm, because its whole job here
    /// is to build both from the same shape.
    fn undated(frames: u64, observed_for_ms: u64) -> TopicLiveness {
        TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms,
            frames_observed: frames,
            rate_estimate: None,
        }
    }

    /// The produced-is-never-dead rule: `frames_observed > 0`
    /// with no age is `Idle`, at EVERY observation length — including far past
    /// the no-data threshold, where the same record with zero frames is `NoData`.
    ///
    /// What breaks this test: drop the `frames_observed > 0` arm from `state_with` and
    /// every case here past the threshold flips to `NoData` — the dead-route
    /// rendering on rows that demonstrably carry data.
    #[test]
    fn a_produced_but_undated_topic_is_idle_never_no_data() {
        for observed_for_ms in [
            0,
            1,
            LIVENESS_STREAMING_RECENCY_MS,
            LIVENESS_NO_DATA_MIN_MS - 1,
            LIVENESS_NO_DATA_MIN_MS,
            100 * LIVENESS_NO_DATA_MIN_MS,
        ] {
            let l = undated(1, observed_for_ms);
            assert_eq!(
                l.state(),
                LivenessState::Idle,
                "observed {observed_for_ms} ms: a topic that delivered a frame is \
                 never dimmed, however long it has been watched"
            );
            // The DISCRIMINATOR: the identical record with zero frames is the
            // dead route once it is settled.
            let nothing = undated(0, observed_for_ms);
            let expected = if observed_for_ms >= LIVENESS_NO_DATA_MIN_MS {
                LivenessState::NoData
            } else {
                LivenessState::Unknown
            };
            assert_eq!(
                nothing.state(),
                expected,
                "observed {observed_for_ms} ms with NOTHING seen"
            );
        }
        // It also serializes as `idle`, not as an absent (UNKNOWN) key.
        assert_eq!(undated(3, 60_000).wire_state(), Some(LivenessState::Idle));
    }

    #[test]
    fn state_classification_matches_the_hand_table() {
        // THE oracle pair: a dead route and a streaming one, from the
        // same registered catalog, must classify DIFFERENTLY.
        assert_eq!(live(None, 30_000).state(), LivenessState::NoData);
        assert_eq!(live(Some(50), 30_000).state(), LivenessState::Streaming);
        // Not yet settled — never libel a topic as dead before it has had a fair
        // chance to publish.
        assert_eq!(live(None, 0).state(), LivenessState::Unknown);
        assert_eq!(
            live(None, LIVENESS_NO_DATA_MIN_MS - 1).state(),
            LivenessState::Unknown
        );
        assert_eq!(
            live(None, LIVENESS_NO_DATA_MIN_MS).state(),
            LivenessState::NoData,
            "the no-data threshold is inclusive"
        );
        // Seen, but not recently.
        assert_eq!(
            live(Some(LIVENESS_STREAMING_RECENCY_MS), 60_000).state(),
            LivenessState::Streaming,
            "the recency threshold is inclusive"
        );
        assert_eq!(
            live(Some(LIVENESS_STREAMING_RECENCY_MS + 1), 60_000).state(),
            LivenessState::Idle
        );
        // A frame ALWAYS beats the no-data rule — a topic that has streamed is
        // never `NoData`, however briefly it has been watched.
        assert_eq!(live(Some(10), 0).state(), LivenessState::Streaming);
    }

    /// The two thresholds must not contradict each other on the SAME
    /// publisher. A topic whose inter-frame gap is `g` classifies `Idle` once it
    /// has published; if `NO_DATA_MIN < RECENCY` there is a band of `g` where the
    /// identical publisher would be called `NoData` while waiting for its FIRST
    /// frame. The const-assert beside the constant makes that unrepresentable;
    /// this pins the BEHAVIOUR the assert protects.
    #[test]
    fn no_data_is_never_claimed_faster_than_idle_would_be() {
        // (The ordering itself is const-asserted beside the constant; clippy
        // rejects a constant-valued runtime `assert!`, so what is exercised here
        // is the BEHAVIOUR that ordering buys.)
        //
        // A gap slower than `Streaming` but plainly alive: 1 ms past the recency
        // window, which is inside the no-data threshold by construction.
        const GAP_MS: u64 = LIVENESS_STREAMING_RECENCY_MS + 1;
        assert_eq!(
            live(Some(GAP_MS), 60_000).state(),
            LivenessState::Idle,
            "once it HAS published, a just-past-recency gap is Idle"
        );
        assert_eq!(
            live(None, GAP_MS).state(),
            LivenessState::Unknown,
            "so the IDENTICAL gap BEFORE its first frame must not be `no_data` \
             — the same publisher would otherwise be described two ways depending \
             only on when we started watching"
        );
    }

    #[test]
    fn state_with_honours_explicit_thresholds() {
        let l = live(Some(1_000), 1_000);
        assert_eq!(l.state_with(2_000, 2_000), LivenessState::Streaming);
        assert_eq!(l.state_with(500, 500), LivenessState::Idle);
        let n = live(None, 1_000);
        assert_eq!(n.state_with(500, 500), LivenessState::NoData);
        assert_eq!(n.state_with(5_000, 5_000), LivenessState::Unknown);
    }

    /// The documented (deliberate) asymmetry: `Idle` never decays. A topic that
    /// published once, hours ago, keeps saying so rather than degrading to
    /// `NoData` (which would assert it NEVER published — now false) or back to
    /// `Unknown` (which would discard something measured).
    #[test]
    fn idle_is_unbounded_and_never_decays_to_no_data() {
        for age_ms in [
            LIVENESS_STREAMING_RECENCY_MS + 1,
            60_000,
            3_600_000,
            u64::MAX / 2,
        ] {
            assert_eq!(
                live(Some(age_ms), u64::MAX / 2).state(),
                LivenessState::Idle,
                "age {age_ms} ms: a topic that HAS published stays Idle forever"
            );
        }
    }

    #[test]
    fn state_serializes_as_snake_case() {
        assert_eq!(
            serde_json::to_string(&LivenessState::NoData).expect("serialize"),
            r#""no_data""#
        );
        assert_eq!(
            serde_json::to_string(&LivenessState::Streaming).expect("serialize"),
            r#""streaming""#
        );
    }

    /// `wire_state` is the ONE producer of a serialized classification, and
    /// it maps UNKNOWN to ABSENCE so the wire carries exactly three spellings.
    #[test]
    fn wire_state_maps_unknown_to_absence_and_is_otherwise_the_classification() {
        assert_eq!(
            live(Some(10), 60_000).wire_state(),
            Some(LivenessState::Streaming)
        );
        assert_eq!(
            live(Some(LIVENESS_STREAMING_RECENCY_MS + 1), 60_000).wire_state(),
            Some(LivenessState::Idle)
        );
        assert_eq!(
            live(None, LIVENESS_NO_DATA_MIN_MS).wire_state(),
            Some(LivenessState::NoData)
        );
        assert_eq!(
            live(None, 0).wire_state(),
            None,
            "UNKNOWN is ABSENCE on the wire, never the string `unknown`"
        );
    }

    // -- The env switch (PURE) ----------------------------------------------

    #[test]
    fn liveness_switch_is_exact_match_with_no_case_forgiveness() {
        assert_eq!(liveness_switch(None), LivenessSwitch::On);
        assert_eq!(liveness_switch(Some("")), LivenessSwitch::On);
        assert_eq!(liveness_switch(Some("off")), LivenessSwitch::Off);
        for bad in ["OFF", "Off", " off", "off ", "0", "false", "no", "on"] {
            assert_eq!(
                liveness_switch(Some(bad)),
                LivenessSwitch::UnrecognizedStaysOn,
                "`{bad}` must be loudly unrecognized, never silently honoured"
            );
        }
    }

    // -- The shared table ----------------------------------------------------

    #[test]
    fn read_liveness_is_none_for_an_untracked_topic() {
        let table: LivenessTable = Arc::new(Mutex::new(HashMap::new()));
        assert_eq!(read_liveness(&table, "/nope", 1_000 * MS), None);
    }

    #[test]
    fn read_liveness_projects_the_stored_record() {
        let table: LivenessTable = Arc::new(Mutex::new(HashMap::new()));
        let mut rec = LivenessRecord::default();
        rec.begin_attached_observation(0);
        rec.record_frames(drained(4, Some(500 * MS)), 500 * MS); // baseline
        rec.record_frames(drained(3, Some(1_000 * MS)), 1_000 * MS); // advances
        let key = "/a".to_string(); // hot-path-alloc-ok: test-only fixture key.
        table.lock().expect("lock").insert(key, rec);
        assert_eq!(
            read_liveness(&table, "/a", 4_000 * MS),
            Some(TopicLiveness {
                last_frame_age_ms: Some(3_000),
                observed_for_ms: 4_000,
                frames_observed: 7,
                rate_estimate: None,
            })
        );
    }

    // -- The env switch is LOUD on an unrecognized value ----------------------

    /// An unrecognized [`LIVENESS_ENV`] value leaves observation ON and
    /// says so LOUDLY. A typo (`CERULION_TOPIC_LIVENESS=OFF`) that silently kept
    /// observation on with no diagnostic is exactly the silent-inference failure
    /// the repo forbids — the operator would believe they disabled it.
    ///
    /// Pins the LEVEL and the CONTENT of each arm, not merely the boolean:
    /// `liveness_switch_is_exact_match_with_no_case_forgiveness` already covers
    /// the pure classification, so a `warn!` demoted to `debug!` (or deleted)
    /// would otherwise pass unnoticed. Cribs the `CERULION_DRAIN_DISCIPLINE` pin.
    ///
    /// One body, `#[serial]` + RAII-restored env: the three arms mutate ONE
    /// process-global variable, and `#[traced_test]`'s capture is per-test.
    #[test]
    #[serial_test::serial]
    #[tracing_test::traced_test]
    fn liveness_env_warns_loudly_on_an_unrecognized_value() {
        /// Restores (or removes) the variable on drop, panic or not.
        struct EnvGuard(Option<String>);
        impl Drop for EnvGuard {
            fn drop(&mut self) {
                match self.0.take() {
                    Some(v) => std::env::set_var(LIVENESS_ENV, v),
                    None => std::env::remove_var(LIVENESS_ENV),
                }
            }
        }
        let _guard = EnvGuard(std::env::var(LIVENESS_ENV).ok());

        // (a) a typo: observation stays ON, and the operator is TOLD.
        std::env::set_var(LIVENESS_ENV, "OFF");
        assert!(
            liveness_enabled_from_env(),
            "an unrecognized value must leave observation ON (fail-safe: keep the \
             affordance working rather than silently losing it)"
        );
        assert!(
            logs_contain("unrecognized value for the topic-liveness switch"),
            "and it must be LOUD — a silently-honoured typo is the failure mode"
        );
        assert!(
            logs_contain("liveness observation stays ENABLED"),
            "the message must state what actually happened, not just complain"
        );

        // (b) the ONE accepted value: disabled, with an info breadcrumb, and NO
        //     warn text beyond the one arm (a) already emitted.
        std::env::set_var(LIVENESS_ENV, "off");
        assert!(!liveness_enabled_from_env());
        assert!(logs_contain(
            "topic data-flow liveness observation DISABLED"
        ));

        // (c) absent: on, silently — the default must not nag.
        std::env::remove_var(LIVENESS_ENV);
        assert!(liveness_enabled_from_env());
    }

    // -- The rate estimate (PURE) — hand oracles, never self-compares --

    /// A record whose tap has just attached at `t0`, i.e. with its advancement
    /// baseline OPEN — the state every real observation starts from.
    fn attached_at(t0: u64) -> LivenessRecord {
        let mut r = LivenessRecord::default();
        r.begin_attached_observation(t0);
        r
    }

    /// THE headline. A topic publishing far faster than the observer's tap can
    /// hold is reported at its REAL rate, because the count is the publisher's
    /// own commit sequence and not the frames we managed to catch.
    ///
    /// The stimulus is exactly the shape a shallow tap produces on a fast topic:
    /// each drain yields only [`LIVENESS_TAP_BUFFER_SIZE`] frames while the
    /// sequence between drains moves by twenty. The two candidate answers are
    /// therefore FAR apart and neither is the other's rounding — 20 Hz if the
    /// sequence is counted, 2 Hz if the frames are — so this arm cannot pass on a
    /// frames-counting implementation.
    #[test]
    fn an_exact_window_counts_the_publishers_commits_not_the_frames_we_caught() {
        let mut r = attached_at(0);
        // The baseline drain: whatever it carries may be retained history, so it
        // only anchors. Sequence 0, one second in.
        r.record_frames(drained_seq(2, 100 * MS, 0), 1_000 * MS);
        // Two further drains, one second apart, twenty commits apart each.
        r.record_frames(drained_seq(2, 200 * MS, 20), 2_000 * MS);
        r.record_frames(drained_seq(2, 300 * MS, 40), 3_000 * MS);

        let snap = r.snapshot(3_000 * MS).expect("observing");
        let rate = snap.rate_estimate.expect("a window closed at 2 s");
        assert_eq!(
            rate,
            TopicRateEstimate {
                // 40 commits across the 2 s window.
                millihertz: 20_000,
                is_floor: false,
            },
            "the rate must be the publisher's 20 Hz, not the 2 Hz of frames the \
             tap happened to hold (4 frames over 2 s)"
        );
        assert_eq!(rate.hz(), 20.0);
        assert_eq!(
            snap.state(),
            LivenessState::Streaming,
            "and it is served on a Streaming row"
        );
    }

    /// The anchoring drain's own frames belong to the window BEFORE it — they
    /// arrived while nothing was counting. Counting them would inflate the first
    /// window of every observation.
    ///
    /// Pinned on the FLOOR basis, where frames are the numerator and the error
    /// would therefore be visible, and across TWO consecutive windows — because
    /// the two ways of getting this wrong fail in different windows. Counting the
    /// anchor's own frames inflates the FIRST window; failing to clear the count
    /// when a window closes inflates every window AFTER the first, and a
    /// one-window test cannot see it (measured: that omission passed a one-window
    /// version of this test, and every other arm in the file).
    #[test]
    fn the_anchoring_drains_own_frames_are_not_counted_into_the_window() {
        let mut r = attached_at(0);
        // A steady 2-frames-per-second drain whose sequence never moves, so both
        // closes fall to the floor basis.
        for (i, t) in [1_000u64, 2_000, 3_000, 4_000, 5_000].iter().enumerate() {
            r.record_frames(drained_seq(2, (i as u64 + 1) * 100 * MS, 7), t * MS);
        }

        let first_close = r
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("a window closed at 3 s");
        // The record has since closed a SECOND window at 5 s; re-read it there.
        let second_close = r
            .snapshot(5_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("and another at 5 s");
        assert_eq!(
            first_close.millihertz, 2_000,
            "4 frames across the first 2 s window is 2 Hz — counting the \
             anchor's own two frames as well would report 3 Hz"
        );
        assert_eq!(
            second_close.millihertz, 2_000,
            "and the SECOND window measures the same steady stream identically — \
             a close that does not clear the count would report 4 Hz here"
        );
    }

    /// The `Streaming` gate is load-bearing on its own, and this is the class
    /// that reaches it: a topic whose frames keep flowing (so its rate window
    /// closes on schedule) while its publisher stamps every frame IDENTICALLY, so
    /// the advancement rule can never date it.
    ///
    /// That combination is a documented liveness residual — such a topic reads
    /// [`LivenessState::Idle`] forever — and its rate window is perfectly fresh,
    /// so the staleness gate does not fire. Without the `Streaming` gate the
    /// sidebar would render an idle row carrying a confident frequency, which is
    /// the row contradicting itself.
    ///
    /// MEASURED: this is the only arm that fails when the gate is deleted; the
    /// stopped-stream arms pass on the staleness gate alone, because under the
    /// shipped constants both thresholds are `LIVENESS_STREAMING_RECENCY_MS` and
    /// a steady stream trips them at the same instant.
    #[test]
    fn a_topic_the_classifier_cannot_call_streaming_serves_no_rate() {
        let mut r = attached_at(0);
        // Sequences advance (the window will close) while the stamp NEVER does
        // (the advancement rule can never date it — strict `>`).
        for (i, t) in [1_000u64, 2_000, 3_000].iter().enumerate() {
            r.record_frames(drained_seq(2, 500 * MS, i as u32 * 20), t * MS);
        }
        assert_eq!(
            r.rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 20_000,
                is_floor: false,
            }),
            "precondition: a window really did close, and it is FRESH — so the \
             staleness gate cannot be what withholds it"
        );

        let snap = r.snapshot(3_000 * MS).expect("observing");
        assert_eq!(
            snap.last_frame_age_ms, None,
            "precondition: identical stamps never advance, so nothing dated it"
        );
        assert_eq!(
            snap.state(),
            LivenessState::Idle,
            "precondition: produced, freshness unknown — never Streaming"
        );
        assert_eq!(
            snap.rate_estimate, None,
            "so no rate is served: a row the robot will not call streaming must \
             not carry a frequency contradicting its own verdict"
        );
    }

    /// A publisher whose wire sequence never advances cannot be measured by it.
    /// `publish_raw` writes the caller's header VERBATIM, so this is reachable,
    /// and the wrong answer is not "no answer" but a confident **0 Hz** on a
    /// topic that is plainly streaming.
    ///
    /// The correct answer is the frames the observer itself drained, LABELLED as a
    /// floor.
    #[test]
    fn a_sequence_that_never_advances_falls_back_to_a_labelled_floor_never_zero() {
        let mut r = attached_at(0);
        for (i, t) in [1_000, 2_000, 3_000].iter().enumerate() {
            // Stamps advance (so the topic dates + streams); the sequence does not.
            r.record_frames(drained_seq(2, (100 + i as u64 * 100) * MS, 7), t * MS);
        }
        let rate = r
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert_eq!(
            rate,
            TopicRateEstimate {
                millihertz: 2_000,
                is_floor: true,
            },
            "a stalled counter must fall back to the frames actually drained and \
             SAY it is a floor — never report the 0 Hz the sequence delta implies"
        );
    }

    /// The FLOOR basis has a ceiling, and it is the one
    /// [`RATE_FLOOR_BASIS_CEILING_MHZ`] derives: an observer tap holds
    /// [`LIVENESS_TAP_BUFFER_SIZE`] frames and drains once per
    /// [`LIVENESS_SWEEP_INTERVAL_NS`], so a saturated window counts exactly that
    /// ratio however fast the publisher runs. This is the arm that shows WHY the
    /// floor is labelled rather than served as a measurement.
    ///
    /// Both bases are driven over the IDENTICAL drain schedule — a full tap on
    /// every sweep for one window — so the only difference between them is
    /// whether the sequence advanced. The exact basis reads the true 500 Hz; the
    /// floor basis saturates at the derived ceiling and says so.
    #[test]
    fn the_floor_basis_saturates_at_the_derived_ceiling_while_the_exact_basis_does_not() {
        // One window of drains on the sweep grid, each returning a FULL tap.
        let sweeps = RATE_ESTIMATE_MIN_WINDOW_NS / LIVENESS_SWEEP_INTERVAL_NS;
        let per_drain = LIVENESS_TAP_BUFFER_SIZE as u64;
        // 500 Hz against a 200 ms sweep = 100 commits between drains.
        let commits_per_drain = 100u32;

        let mut floor = attached_at(0);
        let mut exact = attached_at(0);
        for i in 0..=sweeps {
            let now = i * LIVENESS_SWEEP_INTERVAL_NS;
            let stamp = 1_000 * MS + now;
            floor.record_frames(drained_seq(per_drain, stamp, 7), now);
            exact.record_frames(
                drained_seq(per_drain, stamp, i as u32 * commits_per_drain),
                now,
            );
        }
        let at = sweeps * LIVENESS_SWEEP_INTERVAL_NS;

        let floor_rate = floor
            .snapshot(at)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert_eq!(
            floor_rate,
            TopicRateEstimate {
                millihertz: RATE_FLOOR_BASIS_CEILING_MHZ,
                is_floor: true,
            },
            "a floor-basis window of FULL taps must land exactly on the ceiling \
             the constants derive — that is what `is_floor` is warning about"
        );

        let exact_rate = exact
            .snapshot(at)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert_eq!(
            exact_rate,
            TopicRateEstimate {
                millihertz: 500_000,
                is_floor: false,
            },
            "the exact basis is bounded by nothing the tap does — the SAME \
             drain schedule reports the publisher's real 500 Hz"
        );
        assert!(
            exact_rate.millihertz > floor_rate.millihertz * 10,
            "and the two must be far apart, or this arm proves nothing"
        );
    }

    /// A stream that STOPS drops its rate — it does not decay one toward zero,
    /// and it does not keep serving the last number for as long as the robot is
    /// up.
    ///
    /// Both halves are asserted, because each fails a different mistake: the HELD
    /// value must stay exactly what was measured while the topic still streams
    /// (a `now`-ended sliding window would report a shrinking rate), and it must
    /// disappear once the classifier stops calling the topic `Streaming`.
    #[test]
    fn a_stopped_stream_holds_then_drops_its_rate_rather_than_decaying_it() {
        let mut r = attached_at(0);
        r.record_frames(drained_seq(2, 100 * MS, 0), 1_000 * MS);
        r.record_frames(drained_seq(2, 200 * MS, 20), 2_000 * MS);
        r.record_frames(drained_seq(2, 300 * MS, 40), 3_000 * MS);

        // One second of silence: still Streaming, still the SAME 20 Hz.
        let snap = r.snapshot(4_000 * MS).expect("observing");
        assert_eq!(snap.state(), LivenessState::Streaming);
        assert_eq!(
            snap.rate_estimate.expect("held").millihertz,
            20_000,
            "the held value is a measurement of a window that happened — it must \
             not shrink just because time passed"
        );

        // Past the recency window: the row is Idle and carries NO rate.
        let quiet = 3_000 * MS + (LIVENESS_STREAMING_RECENCY_MS + 1) * MS;
        let snap = r.snapshot(quiet).expect("observing");
        assert_eq!(snap.state(), LivenessState::Idle);
        assert_eq!(
            snap.rate_estimate, None,
            "a silent topic must serve NO rate — the liveness verdict is the \
             whole story there"
        );
    }

    /// The staleness gate is SEPARATE from the `Streaming` gate, and this drives
    /// it directly: a row that is genuinely `Streaming` — freshly dated, every
    /// sweep — while the last window it managed to CLOSE is nine seconds old must
    /// serve no rate at all.
    ///
    /// **Scope.** The stimulus is synthetic: stamps that advance (so the
    /// topic dates) while the wire sequence walks steadily BACKWARD (so every
    /// drain re-anchors the rate window and none of them can close it). Under the
    /// SHIPPED constants a real robot rarely separates the two gates, because
    /// [`RATE_ESTIMATE_MAX_AGE_NS`] IS the recency window and any sustained
    /// stream of frame-yielding drains closes a window inside it — so this is
    /// defence in depth, and it is exactly the kind of guard that goes quietly
    /// inert without an arm holding it. It stops being belt-and-braces the moment
    /// anyone raises `LIVENESS_STREAMING_RECENCY_MS`: the `Streaming` gate alone
    /// would then happily serve an hour-old measurement.
    #[test]
    fn a_streaming_topic_whose_window_went_stale_serves_no_rate() {
        let mut r = attached_at(0);
        // One window closes normally: 40 commits across 2 s = 20 Hz.
        r.record_frames(drained_seq(2, 100 * MS, 999_960), 1_000 * MS);
        r.record_frames(drained_seq(2, 200 * MS, 999_980), 2_000 * MS);
        r.record_frames(drained_seq(2, 300 * MS, 1_000_000), 3_000 * MS);
        assert_eq!(
            r.snapshot(3_000 * MS)
                .expect("observing")
                .rate_estimate
                .map(|e| e.millihertz),
            Some(20_000),
            "precondition: a window really did close, and this is its value"
        );

        // Then a long stretch of drains whose stamps ADVANCE (dating the topic on
        // every one) while their sequences fall further than the reset tolerance
        // each time — so every one re-anchors and none can close.
        let mut now = 3_000 * MS;
        for i in 1..=9u64 {
            now += 1_000 * MS;
            r.record_frames(
                drained_seq(2, (300 + i * 100) * MS, 1_000_000 - (i as u32) * 1_000),
                now,
            );
        }

        let snap = r.snapshot(now).expect("observing");
        assert_eq!(
            snap.last_frame_age_ms,
            Some(0),
            "precondition: the topic was dated by the very last drain"
        );
        assert_eq!(
            snap.state(),
            LivenessState::Streaming,
            "precondition: so the classifier calls it Streaming"
        );
        assert_eq!(
            snap.rate_estimate, None,
            "but the last window it managed to close is nine seconds old, so the \
             robot must stop claiming a rate rather than serve a stale one"
        );
    }

    /// A retained-history flush is OLD data however new it looks, so it may never
    /// produce a rate — otherwise a dead route whose backlog is flushed at us
    /// would render as a live stream at whatever rate it used to run.
    ///
    /// The internal window state is asserted as well as the served payload,
    /// because the served payload is `None` here for a SECOND reason (the topic
    /// is not `Streaming` — a baseline burst dates nothing), and an arm that
    /// only checked the payload would pass with the baseline guard deleted.
    #[test]
    fn a_baseline_burst_never_produces_a_rate() {
        let mut r = attached_at(0);
        // A long flush arriving in chunks, none of which empties the queue, with
        // sequences that climb exactly as a real backlog's do.
        for i in 0..10u32 {
            let now = (i as u64 + 1) * 1_000 * MS;
            r.record_frames(
                DrainObservation {
                    frames: 2,
                    newest_stamp_ns: Some((i as u64 + 1) * 100 * MS),
                    newest_sequence: Some(i * 20),
                    writers_seen: Some(1),
                    queue_emptied: false,
                },
                now,
            );
        }
        assert!(
            r.baseline_open,
            "precondition: the burst never ended, so the baseline is still open"
        );
        assert_eq!(
            r.rate_estimate, None,
            "no window may close while what we are draining might be the \
             publisher's flushed backlog"
        );
        assert_eq!(
            r.snapshot(10_000 * MS).expect("observing").rate_estimate,
            None
        );
    }

    /// A publisher RESTART resets its sequence to a low number. The window that
    /// spans the restart is meaningless and must be DISCARDED — not closed with a
    /// saturated (0 Hz) delta, and not with a wrapped one.
    ///
    /// Two assertions, in order: the spanning window produces nothing, and the
    /// window that follows measures the NEW run correctly.
    #[test]
    fn a_sequence_regression_discards_its_window_and_the_next_one_measures_the_new_run() {
        let mut r = attached_at(0);
        r.record_frames(drained_seq(2, 100 * MS, 1_000), 1_000 * MS);
        // Mid-window, the publisher restarts: sequence drops far below the anchor.
        r.record_frames(drained_seq(2, 200 * MS, 5), 2_000 * MS);
        // A drain that would have closed the spanning window had it survived.
        r.record_frames(drained_seq(2, 300 * MS, 10), 3_000 * MS);
        assert_eq!(
            r.rate_estimate, None,
            "the regressed drain re-anchored, so the window that spans the \
             restart never closes — a saturating delta there would have reported \
             a confident 0 Hz"
        );

        // The new run's own window closes normally: 30 commits over 2 s = 15 Hz.
        r.record_frames(drained_seq(2, 400 * MS, 35), 4_000 * MS);
        let rate = r
            .snapshot(4_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("the post-restart window closed");
        assert_eq!(
            rate,
            TopicRateEstimate {
                millihertz: 15_000,
                is_floor: false,
            },
            "and it counts only the new run's commits (5 → 35 across 2 s)"
        );
    }

    /// Backward sequence movement WITHIN the tolerance is a benign reorder, not
    /// an epoch change: the window survives it and still measures the topic.
    /// The complement of the arm above — together they pin the threshold on both
    /// sides, so widening or removing it is caught.
    #[test]
    fn in_tolerance_backward_jitter_does_not_discard_the_window() {
        let jitter = RATE_SEQ_RESET_TOLERANCE;
        let mut r = attached_at(0);
        r.record_frames(drained_seq(2, 100 * MS, 1_000), 1_000 * MS);
        // A drain whose newest sequence sits exactly `tolerance` BELOW the
        // anchor — admitted, so the window keeps filling.
        r.record_frames(drained_seq(2, 200 * MS, 1_000 - jitter), 2_000 * MS);
        // The closing drain is well ahead: 40 commits across the 2 s window.
        r.record_frames(drained_seq(2, 300 * MS, 1_040), 3_000 * MS);
        let rate = r
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("the window survived the jitter and closed");
        assert_eq!(
            rate,
            TopicRateEstimate {
                millihertz: 20_000,
                is_floor: false,
            },
            "an in-tolerance backward step must not throw the window away"
        );
    }

    /// A topic with no tap reports nothing at all — the whole payload is absent,
    /// so there is no row on which a rate could appear. The budget case
    /// (`ensure_tap` refused, nothing observing) and the lost-tap case land here
    /// identically.
    #[test]
    fn an_unobserved_topic_carries_no_rate_because_it_carries_no_payload() {
        let mut r = attached_at(0);
        r.record_frames(drained_seq(2, 100 * MS, 0), 1_000 * MS);
        r.record_frames(drained_seq(2, 200 * MS, 20), 2_000 * MS);
        r.record_frames(drained_seq(2, 300 * MS, 40), 3_000 * MS);
        assert!(
            r.snapshot(3_000 * MS)
                .expect("observing")
                .rate_estimate
                .is_some(),
            "precondition: while observed it does carry one"
        );

        r.stop_observing(3_000 * MS);
        assert_eq!(
            r.snapshot(3_000 * MS),
            None,
            "nothing is watching, so the robot withholds the whole observation — \
             a rate it has stopped re-measuring is exactly what must not survive"
        );
    }

    /// The window is a MINIMUM span, so a slow topic is measured correctly rather
    /// than never: its window simply closes on the first frame past the span.
    /// 0.5 Hz here — an order of magnitude below the sweep rate.
    #[test]
    fn a_topic_slower_than_the_window_still_gets_an_exact_rate() {
        let mut r = attached_at(0);
        r.record_frames(drained_seq(1, 100 * MS, 0), 1_000 * MS);
        // One commit every 2 s.
        r.record_frames(drained_seq(1, 2_100 * MS, 1), 3_000 * MS);
        r.record_frames(drained_seq(1, 4_100 * MS, 2), 5_000 * MS);
        let rate = r
            .snapshot(5_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("the window closed at the first frame past 2 s");
        assert_eq!(
            rate,
            TopicRateEstimate {
                millihertz: 500,
                is_floor: false,
            },
            "one commit across a 2 s window is 0.5 Hz — the window floor must not \
             round a slow topic to nothing"
        );
        assert_eq!(rate.hz(), 0.5);
    }

    /// A topic that PAUSES and resumes must not report the average over its own
    /// silence — the window that spans the pause is discarded, not closed.
    ///
    /// This is the one wrong-number class the min-window design leaves open on
    /// its own: the window has a minimum span and no maximum, and nothing
    /// advances it while a topic is quiet (a zero-frame drain returns before the
    /// rate accounting), so a pause is simply absorbed into whatever window was
    /// open. MEASURED without the window cap: a 100 Hz topic silent for a minute
    /// then resuming closed one 60-second window holding 20 commits and served
    /// **0.33 Hz** on a row the classifier called `Streaming` — a number wrong by
    /// three hundred times, exactly the class this feature promises never to
    /// produce.
    ///
    /// Both halves are asserted: nothing is served across the resume, and the
    /// first window that fits inside the horizon reports the truth.
    #[test]
    fn a_window_that_spans_a_silence_is_discarded_not_served_as_a_diluted_average() {
        let mut r = attached_at(0);
        // A 100 Hz topic: 20 commits per 200 ms drain.
        r.record_frames(drained_seq(2, 100 * MS, 0), 1_000 * MS);
        r.record_frames(drained_seq(2, 200 * MS, 200), 2_000 * MS);
        r.record_frames(drained_seq(2, 300 * MS, 400), 3_000 * MS);
        assert_eq!(
            r.rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 200_000,
                is_floor: false,
            }),
            "precondition: while streaming it measures its real 200 Hz"
        );

        // Sixty seconds of silence — no frame-yielding drain, so nothing touches
        // the open window — then the topic resumes at the same rate.
        let resume = 63_000 * MS;
        r.record_frames(drained_seq(2, 400 * MS, 420), resume);
        assert_eq!(
            r.snapshot(resume).expect("observing").rate_estimate,
            None,
            "the window spanning the pause must be DISCARDED — closing it would \
             serve 20 commits over 60 s = 0.33 Hz for a topic running at 200"
        );

        // The next window fits inside the horizon and reports the truth again.
        r.record_frames(drained_seq(2, 500 * MS, 620), resume + 1_000 * MS);
        r.record_frames(drained_seq(2, 600 * MS, 820), resume + 2_000 * MS);
        assert_eq!(
            r.snapshot(resume + 2_000 * MS)
                .expect("observing")
                .rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 200_000,
                is_floor: false,
            }),
            "and it recovers within one window rather than staying wrong"
        );
    }

    /// The discard horizon is a THRESHOLD, pinned on both sides at one
    /// millisecond, so widening or removing it is caught: a window closing
    /// exactly AT [`RATE_ESTIMATE_MAX_AGE_NS`] is served, one millisecond past it
    /// is discarded.
    #[test]
    fn the_window_discard_horizon_is_pinned_on_both_sides() {
        // Helper: anchor at 1 s, then close after `span`.
        let run = |span: u64| {
            let mut r = attached_at(0);
            r.record_frames(drained_seq(1, 100 * MS, 0), 1_000 * MS);
            r.record_frames(drained_seq(1, 200 * MS, 10), 1_000 * MS + span);
            r.rate_estimate
        };
        assert!(
            run(RATE_ESTIMATE_MAX_AGE_NS).is_some(),
            "a window closing exactly AT the horizon is still a measurement"
        );
        assert_eq!(
            run(RATE_ESTIMATE_MAX_AGE_NS + MS),
            None,
            "one millisecond past it, the window provably spans a stretch of \
             silence and must be discarded"
        );
    }

    /// A `multi_publisher_topics` topic must never serve a confident rate,
    /// whichever way its two writers interleave.
    ///
    /// `sequence` is a PER-PUBLISHER commit counter and the drain reports the MAX
    /// across a batch, so with two writers that maximum HOPS between unrelated
    /// counters. The backward-motion guard cannot help: a forward hop is
    /// arithmetically indistinguishable from a fast single writer whose frames the
    /// shallow tap clipped, and no plausibility bound separates them (a 500 Hz
    /// topic legitimately advances hundreds of sequences between drains).
    ///
    /// MEASURED without the evidence gate, both directions, both served as confident
    /// `is_floor: false` measurements: a fresh writer anchoring before the old
    /// high-numbered one appears reported **1550 Hz** for a ~101 Hz `/tf`, and the
    /// old-writer-dominates shape reported **25,000,005 Hz**. The shipped residual
    /// claimed such a topic "regresses often ⇒ withholding, not measuring" — false
    /// on both halves, since the maximum never regresses once both writers are
    /// present.
    ///
    /// Both shapes are driven here, and each must come back a labelled FLOOR.
    #[test]
    fn a_multi_writer_topic_never_serves_a_confident_rate_in_either_direction() {
        // SHAPE 1 — a fresh low-numbered writer anchors the window, then the
        // long-running high-numbered one appears: a huge FORWARD hop.
        let mut fresh_first = attached_at(0);
        fresh_first.record_frames(drained_seq_writers(2, 100 * MS, 5, 1), 1_000 * MS);
        fresh_first.record_frames(drained_seq_writers(2, 200 * MS, 10, 2), 2_000 * MS);
        fresh_first.record_frames(drained_seq_writers(2, 300 * MS, 50_000_015, 2), 3_000 * MS);
        let rate = fresh_first
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert!(
            rate.is_floor,
            "a forward hop ACROSS publisher counters must never be served as a \
             measurement — without the evidence gate this reads 25,000,005 Hz: {rate:?}"
        );
        assert_eq!(
            rate.millihertz, 2_000,
            "the correct answer is the frames the observer really drained (4 over \
             the 2 s window), labelled as a floor"
        );

        // SHAPE 2 — the high-numbered writer dominates every drain, so the
        // maximum climbs steadily and NOTHING ever regresses.
        let mut old_dominates = attached_at(0);
        old_dominates.record_frames(drained_seq_writers(2, 100 * MS, 900_000, 2), 1_000 * MS);
        old_dominates.record_frames(drained_seq_writers(2, 200 * MS, 901_550, 2), 2_000 * MS);
        old_dominates.record_frames(drained_seq_writers(2, 300 * MS, 903_100, 2), 3_000 * MS);
        let rate = old_dominates
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert!(
            rate.is_floor,
            "no regression ever fires here, so the discard guard cannot catch it — \
             without the evidence gate this reads 1550 Hz for a ~101 Hz topic: {rate:?}"
        );
        assert_eq!(rate.millihertz, 2_000);
    }

    /// The ANTI-TAUTOLOGY control for the trust gate: the identical drain script
    /// with ONE publisher still measures exactly. Without this the arm above
    /// would also pass an implementation that had simply stopped using the
    /// sequence basis at all.
    #[test]
    fn a_single_writer_topic_still_measures_exactly_under_the_trust_gate() {
        let mut r = attached_at(0);
        r.record_frames(drained_seq_writers(2, 100 * MS, 900_000, 1), 1_000 * MS);
        r.record_frames(drained_seq_writers(2, 200 * MS, 901_550, 1), 2_000 * MS);
        r.record_frames(drained_seq_writers(2, 300 * MS, 903_100, 1), 3_000 * MS);
        assert_eq!(
            r.snapshot(3_000 * MS).expect("observing").rate_estimate,
            Some(TopicRateEstimate {
                // 3100 commits across the 2 s window.
                millihertz: 1_550_000,
                is_floor: false,
            }),
            "one writer ⇒ the sequence delta IS a frame count, and the exact \
             basis must still be reached"
        );
    }

    /// An UNKNOWN writer count is not evidence of a single writer: a drainer that
    /// did not ask gets the floor, not the benefit of the doubt.
    #[test]
    fn an_unknown_writer_count_is_not_treated_as_a_single_writer() {
        let mut r = attached_at(0);
        let unknown = |frames, stamp, seq| DrainObservation {
            writers_seen: None,
            ..drained_seq(frames, stamp, seq)
        };
        r.record_frames(unknown(2, 100 * MS, 0), 1_000 * MS);
        r.record_frames(unknown(2, 200 * MS, 200), 2_000 * MS);
        r.record_frames(unknown(2, 300 * MS, 400), 3_000 * MS);
        let rate = r
            .snapshot(3_000 * MS)
            .expect("observing")
            .rate_estimate
            .expect("a window closed");
        assert!(
            rate.is_floor,
            "absence of evidence is not evidence of one writer: {rate:?}"
        );
    }

    /// The `epoch_reset` half of `anchor_only` is load-bearing ON ITS OWN, and
    /// this is the shape that separates it from the rate window's own guard.
    ///
    /// The two guards key on DIFFERENT quantities: the epoch reset is a STAMP
    /// regression (a restarted publisher's clock begins again), while the rate
    /// window's discard is a SEQUENCE regression beyond
    /// [`RATE_SEQ_RESET_TOLERANCE`]. They do not coincide, and a restart whose new
    /// sequence happens to land WITHIN that tolerance of the anchor slips past the
    /// rate guard entirely — so without the `epoch_reset` term the new run's
    /// retained-history flush would be COUNTED into the window that was already
    /// open, inflating it with frames that are not fresh production.
    ///
    /// The reset batch must therefore only RE-ANCHOR, contributing nothing.
    #[test]
    fn an_epoch_reset_batch_re_anchors_the_rate_window_and_contributes_nothing() {
        let mut r = attached_at(0);
        // A normal run: stamps and sequences both climbing, baseline closed.
        r.record_frames(drained_seq(2, 5_000 * MS, 1_000), 1_000 * MS);
        r.record_frames(drained_seq(2, 6_000 * MS, 1_002), 2_000 * MS);

        // Silence past the reset gate, then a RESTART: the stamp regresses hard
        // (a fresh clock) while the sequence lands only 2 BELOW the anchor —
        // inside RATE_SEQ_RESET_TOLERANCE, so the rate guard does NOT fire and
        // only `epoch_reset` can stop this batch being counted.
        let restart_at = 2_000 * MS + REGRESSION_RESET_MIN_GAP_NS + MS;
        assert!(
            1_002u32.saturating_sub(1_000) <= RATE_SEQ_RESET_TOLERANCE,
            "precondition: the restart's sequence is INSIDE the rate tolerance, \
             so the sequence guard cannot be what saves this window"
        );
        r.record_frames(drained_seq(2, 10 * MS, 1_000), restart_at);

        // Two more drains of the NEW run, two seconds apart.
        r.record_frames(drained_seq(2, 20 * MS, 1_020), restart_at + 1_000 * MS);
        r.record_frames(drained_seq(2, 30 * MS, 1_040), restart_at + 2_000 * MS);

        assert_eq!(
            r.snapshot(restart_at + 2_000 * MS)
                .expect("observing")
                .rate_estimate,
            Some(TopicRateEstimate {
                // Counted from the RESET batch (seq 1000) to the close (1040):
                // 40 commits across 2 s = 20 Hz. Without the `epoch_reset` term
                // the window still open from before the restart would close
                // instead, over a 4.4 s span carrying the flush.
                millihertz: 20_000,
                is_floor: false,
            }),
            "the epoch-reset batch may only RE-ANCHOR — its frames are the new \
             run's possible retained history, not fresh production"
        );
    }

    /// The trust bit is a property of the WINDOW, re-armed at each anchor — so a
    /// topic that briefly showed two publishers recovers its exact basis once the
    /// next window runs clean, rather than being floored for the rest of the run.
    #[test]
    fn the_trust_bit_re_arms_at_each_anchor() {
        let mut r = attached_at(0);
        // Window 1 sees a second writer mid-flight ⇒ floor.
        r.record_frames(drained_seq_writers(2, 100 * MS, 0, 1), 1_000 * MS);
        r.record_frames(drained_seq_writers(2, 200 * MS, 200, 2), 2_000 * MS);
        r.record_frames(drained_seq_writers(2, 300 * MS, 400, 1), 3_000 * MS);
        assert!(
            r.snapshot(3_000 * MS)
                .expect("observing")
                .rate_estimate
                .expect("closed")
                .is_floor,
            "the window that saw two writers is floored"
        );
        // Window 2 is clean throughout ⇒ exact again.
        r.record_frames(drained_seq_writers(2, 400 * MS, 600, 1), 4_000 * MS);
        r.record_frames(drained_seq_writers(2, 500 * MS, 800, 1), 5_000 * MS);
        assert_eq!(
            r.snapshot(5_000 * MS).expect("observing").rate_estimate,
            Some(TopicRateEstimate {
                // 400 commits (400 → 800) across the 2 s window.
                millihertz: 200_000,
                is_floor: false,
            }),
            "and the next clean window recovers the exact basis"
        );
    }

    /// Two runs of the same drain script produce byte-identical estimates
    /// (Principle #7): the whole computation is a function of the observations
    /// and the observer clock, with no wall-clock read and no iteration order in
    /// it.
    #[test]
    fn the_rate_estimate_is_deterministic_across_runs() {
        let script = |seed: u32| {
            let mut r = attached_at(0);
            for i in 0..6u32 {
                let now = (i as u64 + 1) * 700 * MS;
                r.record_frames(drained_seq(2, now / 2, seed + i * 13), now);
            }
            r.snapshot(4_200 * MS).expect("observing").rate_estimate
        };
        let a = script(0);
        let b = script(0);
        assert_eq!(a, b, "same script, same answer");
        assert_eq!(
            a,
            Some(TopicRateEstimate {
                // 13 commits per 700 ms drain; the window closes at 2.1 s having
                // covered three of them = 39 commits.
                millihertz: 18_571,
                is_floor: false,
            }),
            "and it equals the hand-computed value, so the pair above is not a \
             self-compare"
        );
    }

    /// The wire contract, both directions, against a LITERAL pre-rate-field
    /// document. Additive: a robot with no estimate is byte-identical to one that
    /// predates the field, and a robot that predates it decodes to `None` rather
    /// than to a fabricated zero.
    #[test]
    fn the_rate_estimate_is_additive_on_the_wire_in_both_directions() {
        let bare = TopicLiveness {
            last_frame_age_ms: Some(48),
            observed_for_ms: 30_000,
            frames_observed: 600,
            rate_estimate: None,
        };
        let json = serde_json::to_string(&bare).expect("serialize");
        assert_eq!(
            json, r#"{"last_frame_age_ms":48,"observed_for_ms":30000,"frames_observed":600}"#,
            "no estimate ⇒ no key: the pre-rate wire, byte for byte"
        );

        // An older robot's literal document decodes with the field ABSENT.
        let old = r#"{"last_frame_age_ms":48,"observed_for_ms":30000,"frames_observed":600}"#;
        let decoded: TopicLiveness = serde_json::from_str(old).expect("old wire decodes");
        assert_eq!(decoded, bare);
        assert_eq!(
            decoded.rate_estimate, None,
            "an old robot must read as UNKNOWN-rate, never as 0 Hz"
        );

        // And a carrying value round-trips.
        let carrying = TopicLiveness {
            rate_estimate: Some(TopicRateEstimate {
                millihertz: 20_000,
                is_floor: false,
            }),
            ..bare
        };
        let json = serde_json::to_string(&carrying).expect("serialize");
        assert!(
            json.contains(r#""rate_estimate":{"millihertz":20000,"is_floor":false}"#),
            "the nested object must serialize under its own key: {json}"
        );
        assert_eq!(
            serde_json::from_str::<TopicLiveness>(&json).expect("round-trip"),
            carrying
        );
    }

    /// The derived ceiling is what the constants say it is, on both sides of the
    /// derivation — a drift guard, so a future edit to the tap depth or the sweep
    /// interval cannot silently leave the documented ceiling behind.
    #[test]
    fn the_floor_basis_ceiling_is_derived_from_the_tap_depth_and_the_sweep_rate() {
        assert_eq!(
            RATE_FLOOR_BASIS_CEILING_MHZ,
            (LIVENESS_TAP_BUFFER_SIZE as u64) * 1_000 * 1_000_000_000 / LIVENESS_SWEEP_INTERVAL_NS
        );
        // At the shipped constants: 2 frames per 200 ms = 10 Hz.
        assert_eq!(RATE_FLOOR_BASIS_CEILING_MHZ, 10_000);
        assert_eq!(
            RATE_ESTIMATE_MAX_AGE_NS,
            LIVENESS_STREAMING_RECENCY_MS * 1_000_000,
            "the staleness gate must stay THE recency window — one threshold \
             governs both halves of `served only while streaming`"
        );
    }
}
