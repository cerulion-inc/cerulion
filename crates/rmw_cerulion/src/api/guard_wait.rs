// SPDX-License-Identifier: AGPL-3.0-only
//! Guard conditions, wait sets, and the (unsupported) event surface.
//!
//! # Wait strategy (event-driven)
//!
//! `rmw_wait` has THREE exits:
//!
//! 1. **Ready-return**: the non-consuming probe finds a ready entity →
//!    consuming pass → `RMW_RET_OK`.
//! 2. **Deadline**: the caller's timeout is up (rcl passes 0 whenever a
//!    timer is already ready, and `spin_some` polls with 0) → the
//!    race-safe final consuming pass →
//!    `RMW_RET_OK` or `RMW_RET_TIMEOUT`. A zero-timeout call returns
//!    after ONE probe — it never spins, never blocks.
//! 3. **The wait**: a `poll(2)` block on the attached entities' EXISTING
//!    iceoryx2 event-listener fds plus each guard condition's fd doorbell
//!    (a 100 µs sleep-poll measures ~153 µs per round under default timer
//!    slack — the dominant term of the rmw RTT floor, which this avoids).
//!
//! # How the block works (drain-before-wait — a wake is a SIGNAL)
//!
//! Per iteration: drain every wake residue (guard doorbells, listener
//! notification queues) → probe → if ready, consume + return → else block
//! on the fd snapshot with `min(remaining, 20ms, ladder rung)`. The 20 ms
//! cap keeps `pump_publisher_events` at its TRANSIENT_LOCAL late-joiner
//! cadence, and makes the loop a strict SUPERSET of a plain polling loop: a
//! missed, dropped or never-sent wake degrades to a 20 ms cadence, never a
//! stall. Drain-THEN-probe is race-free because every producer commits its
//! truth BEFORE its wake (publish commits the sample before the notify;
//! `GuardConditionState::trigger` stores the flag before the ring), so
//! anything whose signal the drain removed is already visible to the probe.
//!
//! # The adaptive block ladder (why the block is NOT a flat 20 ms)
//!
//! In measurement, a flat 20 ms block REGRESSED the stock posture
//! (64 B ping-pong p50 178 → 203 µs) even though the same primitive under
//! a C1 cap was 39.7 µs and under a C0 pin 13.9 µs: with no near timer the
//! governor takes a deep C-state during the block and both hops pay the
//! exit. So the block timeout is a LADDER on `FdWakeSet::wait` (Linux and Android:
//! `ppoll`, ns precision): first rung [`BLOCK_FIRST_RUNG_US`] (200 µs —
//! measured identical latency to 100 at half the idle wake rate; a near
//! timer keeps the core shallow the way the legacy sleep does by accident,
//! while the fd still wakes the block in µs); after
//! [`BLOCK_BACKOFF_AFTER`] consecutive EMPTY timeouts (50 ≈ 10 ms of
//! proven idleness) the rung doubles per iteration up to the 20 ms cap;
//! ANY wake or ready-return resets to the first rung. The ladder state
//! persists across calls on the wait set (a ping-pong executor returns
//! quickly and must come back to a shallow rung). Both numbers are
//! INTERNAL constants, not knobs — there is no env override for them,
//! and retuning the ladder is a code change (the shipped
//! shape has no off-switch). Counters: `block_rung_resets`,
//! `block_backoffs`. The spin stays in front, unchanged; the kill-switch
//! path never touches the ladder, and the ladder budget bounds the PARK
//! tier below exactly as it bounds the fd block.
//!
//! # The park tier (Linux): topic doorbells + the native monitor-wait
//!
//! On Linux the block has a second, shallower tier. Every rmw publisher
//! arms its topic's SHM doorbell at `rmw_create_publisher`
//! (`CerulionPublisher::enable_doorbell` — the SAME producer-side ring
//! the native graph build arms; `notify_sent_sample` already stores to it
//! after every send, so a publish rings it for free). When the event path
//! is on, `CERULION_MONITOR_WAIT` permits (shared resolution — see
//! Knobs), and at least one subscription topic's doorbell page is mapped
//! ([`WaitSetData::refresh_park_bells`] — the consumer-side open
//! `O_CREAT`s, so a subscription whose producer is not up yet CREATES the
//! page the producer later joins by name, and a genuinely failed open
//! (ENOSPC, EACCES) is retried every [`PARK_BELL_RETRY_CALLS`] calls), the
//! block PARKS
//! on the doorbells instead of sleeping in `ppoll`:
//! [`WaitSetData::park_block`] arms the native monitor-wait on the
//! primary doorbell line
//! ([`cerulion_core::monitor_wait::monitor_wait_until_addr`] — `UMWAIT`
//! on WAITPKG x86_64, `WFE` on aarch64, a bounded ~100 µs sleep-recheck
//! where the CPU has no primitive; never a busy-spin) and re-checks every
//! doorbell plus the fd snapshot ([`FdWakeSet::check`], a zero-timeout
//! poll) each recheck slice. A publish therefore wakes the waiter
//! hardware-instantly on the primary topic and within one slice on the
//! rest; guard/service/client fds wake within one slice. The ladder rung
//! still bounds every park, so the pump cadence and the idle backoff are
//! tier-independent. Off Linux no ring wakes this park
//! ([`RING_WAKES_THE_PARK`]): the park tier never engages and the fd block
//! serves everything — same observable behavior, µs-tier wakes via the
//! fd instead of ns-tier via the line. rmw publishers open the bell
//! UNOWNED and never unlink it (a ROS topic is provisioned at two
//! publishers — the `/rosout` shape — and the first to die must not pull
//! the page from under the survivor; one 64-byte page per topic may
//! outlive every publisher on the machine until the next creator). An OWNED
//! creator's lifecycle (the native graph build's bell, a stale-name
//! cleanup) can still replace the page under this wait set:
//! the first frame then arrives via the fd path (≤ one recheck slice,
//! never the timer) with the bell silent, and
//! [`WaitSetData::reconcile_stale_bells`] re-maps it on that exact
//! evidence — frame-without-ring, [`bell_is_stale`] — so the next publish
//! rings again. Counters: `park_blocks`, `park_wakes_doorbell`,
//! `park_bell_reopens`.
//!
//! **The park is BOUNDED and OS-cooperative.** Measured with
//! both ping-pong threads pinned to one CPU, an unbounded `UMWAIT` park
//! held its core against the publisher it was waiting for until CFS
//! wakeup granularity — a 7 ms RTT wall — because to the scheduler a
//! parked thread is RUNNING. So every [`PARK_RECHECK`] slice that times
//! out YIELDS the core (`park_yields`), and the horizon is platform-
//! informed ([`park_policy`], the table on [`ParkHorizon`]): NO park on
//! x86 — the fd/`ppoll` tier measured faster at 2 ms (27.7 vs 30.1 µs)
//! AND 10 ms (19.8 vs 58.8) even against a one-rung park; `FirstRungOnce`
//! on aarch64 `WFE`: one park of at most the first rung per call, then the
//! kernel fd block.
//! `CERULION_MONITOR_WAIT=1` forces the park on for x86 experiments
//! (bounded to one rung per call).
//!
//! # Fired-only drain and probe (why a wake costs O(fired), not O(set))
//!
//! A stock rclcpp executor's wait set is one subscription plus the six
//! parameter services plus two guards. Draining every listener and
//! probing every entity twice on every wake, as a naive loop does, was
//! ~19 non-blocking syscalls and ~21 SHM probes per hop, i.e. most of
//! the measured ping-pong RTT. Instead, the fd snapshot carries an
//! [`EntityRef`] table; `poll(2)`'s `revents` (`FdWakeSet::fired`) says
//! exactly which entities woke the block, and the loop drains and probes
//! only those (plus the subscriptions on a doorbell-named topic). The
//! call's ENTRY still runs one FULL SHM probe (`probes_entry`) — rclcpp
//! takes ONE message per ready subscription per spin, so a second queued
//! sample has no notification left to fire an fd for and only a full pass
//! finds it — preceded by one `poll(fds, 0)` so anything readable at entry
//! is drained first. A block that times out drains and probes NOTHING: a
//! frame committed since the entry pass has a readable fd (commit before
//! notify), and one racing the `revents` snapshot returns the next block
//! immediately. An iteration whose TRANSIENT_LOCAL pump actually ran
//! probes everything (history arrives without a wake). The single
//! consuming pass ([`consume_from_mask`]) nulls not-ready entries from the
//! mask the probe filled — no further pass. The kill-switch and the
//! degraded fallback keep the legacy drain-all/probe-all loop verbatim.
//!
//! # Why fd snapshots + `poll(2)`, not the iceoryx2 WaitSet
//!
//! * **Lock discipline**: attaching a `&Listener` would pin each entity's
//!   mutex for the whole block, stalling concurrent take/publish threads
//!   (MultiThreadedExecutor). An fd is a `Copy` snapshot: lock, read,
//!   UNLOCK, block ([`cerulion_core::transport::subscriber::CerulionSubscriber::event_listener_fd`]).
//! * **No FD_SETSIZE abort**: iceoryx2 0.9.1's WaitSet selects on macOS and
//!   aborts on any fd number >= 1024; `poll(2)` has no fd-number ceiling on
//!   any unix (a precedent already in the tree), so that hazard is structurally
//!   impossible here rather than guarded with a fallback.
//! * **No double-attach question**: nothing is registered with a listener;
//!   readability is level-triggered, so even two waiters watching one fd
//!   (which rcl's disjoint-entity-per-executor discipline should prevent)
//!   both wake, and the truth stays with the probe.
//!
//! No second wake source is minted per topic: every rmw publish already
//! notifies the subscription's own listener, so blocking on it costs the
//! publisher nothing new (a second `WakeSource` would cost an event-port
//! slot plus a `sendto` per publish).
//!
//! # Knobs (ONE surface, shared with the native live loop)
//!
//! * `CERULION_LIVE_SPIN_US` — the SAME knob the native live loop reads,
//!   through the SAME parse
//!   ([`cerulion_core::monitor_wait::classify_live_spin_us`]; one
//!   ceiling, [`cerulion_core::monitor_wait::SPIN_BUDGET_MAX_US`] =
//!   100 ms, one-time warn on clamp — the drift protection is the parse
//!   having exactly one definition). It sets the user-space busy-probe
//!   budget in front of the FIRST block of a call on the event path. The
//!   ONE asymmetry, stated plainly: `unset` means "the consumer's own
//!   default", and the two consumers derive different defaults — the
//!   native loop spins only while its graph says a wake is imminent,
//!   while the rmw wait has no graph to derive imminence from and
//!   defaults to PARK-FIRST (no spin). Measurements behind that default: spin
//!   OFF is best-or-equal in EVERY posture (stock 30 µs, C1 cap 19, C0
//!   pin 17 — 64 B ping-pong p50), while spin ON put roughly half the
//!   processes into a per-lifetime ~123 µs mode in the stock and C0
//!   postures (never under a C1 cap) — the same-core CFS co-location mode
//!   the discriminator binary pins. Per call the spin runs at most TWICE:
//!   once on entry, and at most one recovery spin after a wake whose
//!   probe found nothing (a per-call bool, consumed on first use, never
//!   re-armed — a stream of notify-without-readiness wakes cannot
//!   re-enter it); never after an empty timeout; and the ENTRY spin is
//!   armed only by a previous call that RETURNED READY — a call that
//!   delivered nothing disarms it, however its blocks woke, so neither an
//!   idle executor NOR a notification-without-readiness storm spins more
//!   than once per idle period, even at the 100 ms cap (the value that is
//!   safe on the native loop's imminence-gated spin). It yields between
//!   probes. `0`
//!   disables; malformed warns once and stays park-first. The
//!   `prctl(PR_SET_TIMERSLACK, 1)` is applied — see [`set_timer_slack_once`]
//!   — because the ladder's 200 µs `ppoll` rung benefits from it exactly
//!   as a sleep-poll would.
//! * `CERULION_MONITOR_WAIT` — the SAME flag the native live loop
//!   resolves, through the SAME classification
//!   ([`cerulion_core::monitor_wait::classify_env_flag`]), applied to the
//!   PLATFORM default ([`park_policy`] / [`resolve_park`]): unset ⇒ the
//!   default (one-rung park on aarch64 `WFE`, OFF on x86; measured, see
//!   [`ParkHorizon`]); `1` ⇒ force the park on (x86: the bounded one-rung
//!   shape); `0` ⇒ off (the fd/`ppoll` block serves every wake); any
//!   other non-empty value warns once and keeps the default.
//! * `CERULION_RMW_EVENT_WAIT=off` — the one rmw-only knob, the
//!   kill-switch: restores the legacy sleep-poll loop VERBATIM as exit 3 —
//!   probe, `sleep(100 µs)`, probe: NO fd block, NO park, NO spin
//!   (`CERULION_LIVE_SPIN_US` is ignored, and the once-only `info!` says
//!   so). Everything else identical.
//!
//! The ladder has no knobs — see above.
//!
//! # The degraded fallback (a drain that fails is not a wake)
//!
//! `drain_event_notifications` surfaces a listener drain failure instead of
//! swallowing it, because a listener whose `try_wait_one` errors while its
//! fd stays READABLE would make `poll(2)` fire on every block until the
//! deadline (forever on an infinite wait). On the first such error in a
//! call the wait set is marked DEGRADED for the rest of that call
//! (`WaitSetData::degraded`, reset at entry), the block becomes the legacy
//! `sleep(100 µs)` pacing, and the event is logged through a
//! `FailureRegimeLatch` (loud once per regime, counted repeats, a recovery
//! line) — never one line per iteration. The pure [`block_strategy`] is the
//! single decision both the kill-switch and the degrade route through.
//!
//! The deterministic host executor still never blocks here; this
//! path serves stock rclcpp/rclpy executors. Timeout math stays
//! wall-clock (`Instant`) — this path is not replay-deterministic.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use cerulion_core::error::TransportError;
use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use cerulion_core::wake::{FdWake, FdWakeSet};

use crate::ffi::{self, rmw_ret_t, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK, RMW_RET_TIMEOUT};
use crate::runtime::{self, ClientData, GuardConditionState, ServiceData, SubscriptionData};

/// Legacy sleep-poll cadence — exit 3 under `CERULION_RMW_EVENT_WAIT=off`
/// and while a wait set is DEGRADED (a listener drain failed this call).
const POLL_INTERVAL: Duration = Duration::from_micros(100);

/// Cap on any single kernel block, so a parked waiter keeps pumping
/// publisher events (TRANSIENT_LOCAL late-joiner delivery) at the pump's
/// own cadence. Derived from the pump throttle — one number, no drift.
const BLOCK_CAP: Duration = Duration::from_millis(runtime::PUMP_INTERVAL_MS);

/// The block cap in µs — the top rung of the adaptive block ladder.
const BLOCK_CAP_US: u64 = runtime::PUMP_INTERVAL_MS * 1_000;

/// The ladder's FIRST rung (µs). An INTERNAL constant with no env
/// override; the knob surface is
/// the shared live-wait set, and retuning the ladder is a code change.
const BLOCK_FIRST_RUNG_US: u64 = 200;

/// Consecutive EMPTY timeouts (no fd wake, nothing ready) before the rung
/// starts doubling — ~10 ms of proven idleness at the first rung.
/// Internal, like the rung.
const BLOCK_BACKOFF_AFTER: u64 = 50;

/// The block ladder's fixed shape.
///
/// # Why a ladder (the measurement that forced it)
///
/// A fixed 20 ms block REGRESSED the stock posture (no C-state cap): after
/// the spin the thread blocks with no near timer, the governor takes a deep
/// C-state, and BOTH ping-pong hops pay the exit (~100+ µs), so the reply
/// never lands inside the peer's spin — 64 B p50 178 µs → 203 µs,
/// while the same primitive under a C1 cap is 39.7 µs and under a C0 pin
/// 13.9 µs. A short first rung (200 µs; 100 measured identical at twice
/// the idle wake rate) keeps the core shallow the way the legacy sleep does
/// BY ACCIDENT (the governor bounds the C-state by the next timer) while
/// the fd still wakes the block in µs; after `backoff_after` consecutive
/// EMPTY timeouts the rung doubles per iteration up to the 20 ms pump cap,
/// so an idle wait set costs ~60 wakeups per 250 ms instead of ~1250. ANY
/// fd wake or ready-return resets to the first rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BlockLadder {
    /// First rung in µs (never 0 — the shipped shape has no off-switch).
    first_rung_us: u64,
    /// Consecutive empty timeouts before doubling begins (>= 1).
    backoff_after: u64,
}

/// The one shipped ladder.
const BLOCK_LADDER: BlockLadder = BlockLadder {
    first_rung_us: BLOCK_FIRST_RUNG_US,
    backoff_after: BLOCK_BACKOFF_AFTER,
};

impl BlockLadder {
    /// The rung every reset returns to.
    fn reset_rung_us(&self) -> u64 {
        self.first_rung_us
    }
}

/// Which entity a watched fd belongs to (index into the caller's array
/// of that class). Recorded beside every `FdWakeSet::watch`, so the fds
/// `poll(2)` reports readable map straight back to the entities to drain
/// and probe — the difference between O(fired) and O(entities) syscalls
/// per wake on a stock rclcpp executor's set (one subscription + the six
/// parameter services + two guards).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EntityRef {
    Sub(usize),
    Guard(usize),
    Service(usize),
    Client(usize),
}

/// How one kernel/park block ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BlockWake {
    /// A topic doorbell rang — the bell index names the topic.
    Bell(usize),
    /// A watched fd became readable (`FdWakeSet::fired` names which).
    Fd,
    /// The budget ran out with nothing.
    Timeout,
}

/// How one park ended ([`WaitSetData::park_until`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ParkEnd {
    wake: BlockWake,
    /// The park returned at its first check: a bell had already moved from the park's
    /// reference seq, so no wait ran.
    #[cfg_attr(not(any(test, feature = "period-park")), allow(dead_code))]
    at_entry: bool,
}

/// The seqs a park measures a ring against ([`WaitSetData::park_until`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParkRef {
    /// Each bell's seq read when the park starts (the stock park).
    Entry,
    /// Each bell's seq at the period learners' last note or sync (feature `period-park`). A
    /// ring that landed after that read, during the kernel lead included, ends the park at
    /// its first check, before a monitor wait is armed on a line that has already moved.
    #[cfg(feature = "period-park")]
    Noted,
}

/// What this loop iteration knows about why it is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Call entry: nothing is known — one `poll(fds, 0)` + a FULL SHM probe.
    Entry,
    /// A block WOKE (fd or bell): drain/probe only what fired.
    Woke,
    /// A block timed out with nothing: nothing new can be ready.
    Timeout,
}

/// How long the park tier may hold the core per call — a PLATFORM table
/// set by measurement, not opinion (x86 with WAITPKG + aarch64 with WFE,
/// 64 B rmw ping-pong, stock posture):
///
/// | platform | 2 ms park ON / OFF | 10 ms park ON / OFF |
/// |---|---|---|
/// | x86 (WAITPKG) | 30.1 / **27.7** | 58.8 / **19.8** |
/// | aarch64 (WFE), `ThroughRung` | **74.8** / 127.1 | **84.3** / 135.7 |
///
/// On x86 the fd/`ppoll` tier beats the park at BOTH periods — even a
/// one-rung park (the `FirstRungOnce` shape) lost 2.4 µs at
/// 2 ms and 39 µs at 10 ms. The aarch64 row was measured with the park
/// held through every block (`ThroughRung`). The x86 default is NO park;
/// the aarch64 default is `FirstRungOnce`: a call parks its first block
/// and then sleeps in the kernel fd block, so a wake later than the first
/// rung arrives through the fd block. Under `ThroughRung` an idle
/// subscriber on a Jetson Orin NX used one core (process CPU 1.00 s per
/// 1 s of idle wait). `CERULION_MONITOR_WAIT=1` forces the park on
/// anywhere it is real (the x86 experiment hatch, which then runs the
/// bounded one-rung shape), `0` forces it off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParkHorizon {
    /// No park tier: every block is the fd/`ppoll` kernel wait. The x86
    /// default (measured faster at 2 ms and 10 ms), Linux without a
    /// primitive (a sleep-recheck park is pointless), and off Linux, where no
    /// ring wakes this park (see `RING_WAKES_THE_PARK`, private to this module).
    NoPark,
    /// Park ONCE per call, for at most the ladder's first rung, then hand
    /// the rest of the idle to the fd/ppoll kernel block. The shape a
    /// FORCED park takes on x86, and the aarch64 `WFE` default.
    /// Measurement shows ANY busy waiter across an idle ≥ ~5 ms costs a
    /// flat ~+30 µs versus the kernel-sleeping waiter (a scheduler/topology
    /// effect around the peer's wake, not frequency), so a long park is
    /// never the right x86 wait.
    FirstRungOnce,
    /// Park on every block for the full ladder rung, yielding per slice.
    /// No platform default and no `CERULION_MONITOR_WAIT` value resolves
    /// to it; an idle wait under it keeps its thread running.
    ThroughRung,
}

/// The shipped PLATFORM default (compile-time: Linux aarch64 is always
/// `WFE`, parked for the call's first rung only; Linux x86_64 is `UMWAIT`
/// or nothing, both measured slower than the kernel wait; off Linux no
/// ring wakes this park).
pub fn park_policy() -> ParkHorizon {
    if cfg!(all(target_os = "linux", target_arch = "aarch64")) {
        ParkHorizon::FirstRungOnce
    } else {
        ParkHorizon::NoPark
    }
}

/// PURE: the horizon after the SHARED `CERULION_MONITOR_WAIT` flag is
/// applied to the platform default (`cerulion_core::monitor_wait::
/// classify_env_flag`, the same classification the native live loop
/// resolves): unset/auto ⇒ the platform default; `0` ⇒ no park; `1` ⇒
/// force the park on — the platform's own shape where it has one, else
/// the bounded one-rung shape on Linux (the x86 experiment hatch); off
/// Linux a forced park is still no park (nothing to ring).
pub fn resolve_park(
    flag: cerulion_core::monitor_wait::EnvFlag,
    default: ParkHorizon,
) -> ParkHorizon {
    use cerulion_core::monitor_wait::EnvFlag;
    match flag {
        EnvFlag::ForceOff => ParkHorizon::NoPark,
        EnvFlag::ForceOn => {
            if default != ParkHorizon::NoPark {
                default
            } else if cfg!(target_os = "linux") {
                ParkHorizon::FirstRungOnce
            } else {
                ParkHorizon::NoPark
            }
        }
        EnvFlag::Auto | EnvFlag::NearMiss => default,
    }
}

/// PURE: does THIS block of a call take the park tier, and what is the
/// call's park allowance afterwards? The whole per-block rule, split out
/// so it is pinnable by a hand oracle rather than only observable through
/// a timing test.
///
/// `allowance` enters a call `true` only when the wait set mapped at least
/// one topic doorbell; a wait set with none never parks, whatever the
/// horizon. [`ParkHorizon::FirstRungOnce`] then SPENDS the allowance on
/// the call's FIRST block by design (the aarch64 default and the x86
/// hatch's bounded shape), so
/// every LATER block of that same call takes the kernel fd block: on that
/// horizon a nonzero `fd_blocks` is the designed outcome of a call that
/// blocked more than once, never evidence that the park tier disengaged.
/// [`ParkHorizon::ThroughRung`] keeps the allowance, so every block of the
/// call parks and `fd_blocks` stays 0.
pub fn block_parks(horizon: ParkHorizon, allowance: bool) -> (bool, bool) {
    if !allowance {
        return (false, false);
    }
    match horizon {
        ParkHorizon::NoPark => (false, false),
        ParkHorizon::FirstRungOnce => (true, false),
        ParkHorizon::ThroughRung => (true, true),
    }
}

/// PURE: how a block spends a predicted window `(opens at, length)`. `Some((lead, park_end))`
/// means a kernel fd block of `lead`, then a doorbell park that ends at the instant
/// `park_end`: the window's close, or the block's end (`now + budget`) if that is sooner.
/// The end is an instant, so a lead that wakes late shortens the park and never moves its
/// end. `None` means the stock block. A window already closed, or opening at or beyond
/// `budget`, gives `None`.
pub fn predicted_block(
    now: Instant,
    window: Option<(Instant, Duration)>,
    budget: Duration,
) -> Option<(Duration, Instant)> {
    let (open, len) = window?;
    let close = open.checked_add(len)?;
    if close <= now {
        return None;
    }
    let lead = open.saturating_duration_since(now);
    if lead >= budget {
        return None;
    }
    let park_end = now.checked_add(budget).map_or(close, |end| close.min(end));
    Some((lead, park_end))
}

/// PURE (feature `period-park`): what a predicted block reports to its topic's learner.
/// `lead` is how the kernel lead ended and `park` how the park after it ended (`None`: no
/// park ran, an fd cut the lead). Only a ring of the predicted bell that the armed park saw
/// is a catch at ring time. An fd that cut the lead, or a predicted bell that had already
/// rung at the park's first check, is a late sighting of the message. Any other end (an fd,
/// another bell, the park's end) tells the learner nothing about the predicted message.
#[cfg(any(test, feature = "period-park"))]
fn window_end(
    lead: BlockWake,
    park: Option<ParkEnd>,
    bell: usize,
) -> Option<super::period_learner::WindowEnd> {
    use super::period_learner::WindowEnd;
    match park {
        None => (lead == BlockWake::Fd).then_some(WindowEnd::BeforePark),
        Some(ParkEnd {
            wake: BlockWake::Bell(b),
            at_entry,
        }) if b == bell => Some(if at_entry {
            WindowEnd::BeforePark
        } else {
            WindowEnd::Caught
        }),
        Some(_) => None,
    }
}

/// Most subscription topics one call predicts over (feature `period-park`): each wake
/// reads every mapped bell, and each regular topic adds one park window per period.
const PREDICT_MAX_TOPICS: usize = 16;

/// PURE: does a call predict windows (feature `period-park`)? Not under
/// `CERULION_MONITOR_WAIT=0` (`flag` is `ForceOff`), not without a CPU monitor-wait
/// primitive (`hw` false: the park would be a sleep-recheck loop), and not over more
/// than [`PREDICT_MAX_TOPICS`] subscription topics.
fn predicts_windows(flag: cerulion_core::monitor_wait::EnvFlag, hw: bool, topics: usize) -> bool {
    flag != cerulion_core::monitor_wait::EnvFlag::ForceOff && hw && topics <= PREDICT_MAX_TOPICS
}

/// The park horizon in µs under [`ParkHorizon::FirstRungOnce`] — the
/// ladder's first rung, so a call's one park is exactly as long as its
/// first kernel block would have been.
const PARK_HORIZON_US: u64 = BLOCK_FIRST_RUNG_US;

/// The park's recheck slice: every slice re-checks the bells and the fd
/// snapshot, and a slice that times out YIELDS the core. Short enough that
/// a co-located peer (same core) runs within it — the same-core ping-pong
/// stays well under 100 µs — while `UMWAIT`/`WFE` still wake instantly on
/// the ring inside a slice. The VALUE is the shared
/// `cerulion_core::monitor_wait::PARK_RECHECK` — the native live loop's
/// `monitor_wait_block` hardware arm slices (and yields) at the same
/// constant, so the two parks cannot drift apart.
const PARK_RECHECK: Duration = cerulion_core::monitor_wait::PARK_RECHECK;

/// PURE: the rung after one more EMPTY timeout. `consecutive_empty` already
/// counts this one. Returns `(rung_us, backed_off)` — `backed_off` only
/// when the rung actually GREW (the cap is a plateau, not a backoff).
fn next_rung_after_empty(rung_us: u64, consecutive_empty: u64, cfg: &BlockLadder) -> (u64, bool) {
    if consecutive_empty < cfg.backoff_after {
        return (rung_us, false);
    }
    let next = rung_us.saturating_mul(2).min(BLOCK_CAP_US);
    (next, next > rung_us)
}

/// How exit 3 of `rmw_wait` waits — decided ONCE per iteration by the pure
/// [`block_strategy`], so the kill-switch and the degraded fallback cannot
/// disagree about what "the legacy loop" means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BlockStrategy {
    /// Event-driven: spin-then-block on the entities' fds.
    Fd,
    /// The plain sleep-poll loop: probe, `sleep(POLL_INTERVAL)`, probe.
    /// NO spin — `CERULION_LIVE_SPIN_US` is ignored on this arm.
    Sleep,
}

/// PURE: the fd path needs BOTH the kill-switch on AND a healthy drain this
/// call. `CERULION_RMW_EVENT_WAIT=off` must restore the documented legacy
/// sleep-poll EXACTLY (no fd block, no spin), and a wait set whose listener
/// drain failed must stop blocking on fds it cannot drain (a level-readable
/// fd nobody drains re-fires every block, the stale-signal spin class): both
/// land on the same [`BlockStrategy::Sleep`] arm.
fn block_strategy(event_wait: bool, degraded: bool) -> BlockStrategy {
    if event_wait && !degraded {
        BlockStrategy::Fd
    } else {
        BlockStrategy::Sleep
    }
}

pub(crate) fn create_guard_condition_raw() -> *mut ffi::rmw_guard_condition_t {
    let state = Box::new(GuardConditionState::new());
    let gc = Box::new(ffi::rmw_guard_condition_t {
        implementation_identifier: ffi::implementation_identifier_ptr(),
        data: Box::into_raw(state) as *mut std::os::raw::c_void,
        context: std::ptr::null_mut(),
    });
    // Delivery log / replay: guards are named `gc#n` in creation order.
    crate::replay::on_entity(
        crate::replay::log::EntityKind::Guard,
        gc.data as usize,
        "",
        "",
    );
    Box::into_raw(gc)
}

/// # Safety
/// `gc` must come from `create_guard_condition_raw` and not be used
/// afterwards.
pub(crate) unsafe fn destroy_guard_condition_raw(gc: *mut ffi::rmw_guard_condition_t) {
    if gc.is_null() {
        return;
    }
    crate::replay::on_entity_destroyed((*gc).data as usize);
    drop(Box::from_raw((*gc).data as *mut GuardConditionState));
    drop(Box::from_raw(gc));
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_guard_condition(
    context: *mut ffi::rmw_context_t,
) -> *mut ffi::rmw_guard_condition_t {
    if context.is_null() || !ffi::is_our_identifier((*context).implementation_identifier) {
        return std::ptr::null_mut();
    }
    let gc = create_guard_condition_raw();
    (*gc).context = context;
    gc
}

/// # Safety
/// `guard_condition` must come from `rmw_create_guard_condition`.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_guard_condition(
    guard_condition: *mut ffi::rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    // Foreign-rmw guards must be rejected BEFORE Box::from_raw — a
    // wrong-implementation pointer here is heap corruption otherwise.
    if !ffi::is_our_identifier((*guard_condition).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    destroy_guard_condition_raw(guard_condition);
    RMW_RET_OK
}

/// # Safety
/// `guard_condition` must be valid. Callable from any thread.
#[no_mangle]
pub unsafe extern "C" fn rmw_trigger_guard_condition(
    guard_condition: *const ffi::rmw_guard_condition_t,
) -> rmw_ret_t {
    if guard_condition.is_null()
        || !ffi::is_our_identifier((*guard_condition).implementation_identifier)
    {
        return RMW_RET_INVALID_ARGUMENT;
    }
    let state = &*((*guard_condition).data as *const GuardConditionState);
    // Flag + doorbell ring — flag FIRST; see GuardConditionState.
    state.trigger();
    RMW_RET_OK
}

/// # Safety
/// Caller frees with `rmw_guard_condition_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_guard_condition_allocate() -> *mut ffi::rmw_guard_condition_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_guard_condition_t>()))
}

/// # Safety
/// `guard_condition` must come from `rmw_guard_condition_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_guard_condition_free(
    guard_condition: *mut ffi::rmw_guard_condition_t,
) {
    if !guard_condition.is_null() {
        drop(Box::from_raw(guard_condition));
    }
}

// =====================================================================
// Wait set
// =====================================================================

/// Per-wait-set wait state + diagnostics. Lives behind
/// `rmw_wait_set_t::data` — NEVER process-global: rclcpp processes run
/// CONCURRENT `rmw_wait` loops (GraphListener, TimeSource clock thread,
/// TransformListener), each on its own wait set over a DISJOINT entity
/// set (rcl re-populates wait sets per call; each entity belongs to one
/// executor).
///
/// The `&mut` in `rmw_wait` is sound because rcl uses one wait set from
/// one thread at a time (`rcl_wait` is documented not thread-safe on a
/// single wait set).
/// The counters are atomics (not plain `u64`) so a test can OBSERVE a
/// wait in progress from another thread — "publish once the waiter is
/// provably inside its spin" — without a data race; `rmw_wait` itself
/// touches everything here through a SHARED reference for the same
/// reason (the same observe-from-another-thread shape), which is why the non-atomic state sits
/// behind uncontended `Mutex`es.
#[derive(Default)]
pub struct WaitSetData {
    /// The poll(2) fd snapshot, rebuilt from the attached entities each
    /// call (buffer reused across calls and iterations). Never contended:
    /// one wait per set at a time is the rcl contract.
    fd_set: Mutex<FdWakeSet>,
    /// Delivery log / replay only: the index + 1 of the first thread that
    /// waited on this wait set (0 = none yet). A second thread waiting here
    /// is the rclcpp MultiThreadedExecutor mark (`replay::detect`). Never
    /// touched with recording and replay off.
    replay_owner: std::sync::atomic::AtomicU32,
    /// PER-CALL: a listener drain failed during this `rmw_wait`, so the fd
    /// path is untrustworthy for the rest of the call (an fd that cannot be
    /// drained may stay readable and would re-fire every block). Reset at
    /// entry — the next call retries the fd path, so a transient error
    /// heals by itself.
    degraded: AtomicBool,
    /// The PREVIOUS call on this wait set ended EMPTY (timeout, no
    /// fd/doorbell wake), so the next call's ENTRY spin is DISARMED. The
    /// per-call spin ceiling does not bound CUMULATIVE idle CPU: an idle
    /// executor re-enters `rmw_wait` in a loop, and at the shared knob's
    /// 100 ms cap every iteration would burn a full busy-probe before its
    /// first block — a hazard precisely because the knob is shared with
    /// the native live loop, whose spin is imminence-gated and safe at
    /// that value. PRODUCTIVE = the call RETURNED READY (something was
    /// delivered) — a woken-but-empty call does NOT re-arm, because a
    /// notification-without-readiness storm wakes blocks without
    /// delivering and would otherwise re-arm every iteration. During
    /// real traffic the entry spin runs; across an idle period it runs
    /// ONCE. `false` (the default) = armed — a fresh wait set may spin.
    spin_disarmed: AtomicBool,
    /// Flood-suppression for the drain-failure log: loud once per regime,
    /// counted repeats, a recovery line — never one line per iteration.
    drain_failures: Mutex<FailureRegimeLatch>,
    /// Diagnostics (Principle #3 — observable independent of logs):
    /// kernel blocks entered on the event path. Monotonic, never reset;
    /// tests read them back by casting the opaque data pointer.
    pub fd_blocks: AtomicU64,
    /// Blocks that returned because a watched fd became readable.
    pub fd_wakes: AtomicU64,
    /// Blocks that ran out their full slice.
    pub timeout_wakes: AtomicU64,
    /// Spin-front probes performed (one per busy-probe iteration).
    pub spin_probes: AtomicU64,
    /// Spin-front iterations that OBSERVED readiness (the wake was caught
    /// in user space, no kernel block paid).
    pub spin_wakes: AtomicU64,
    /// Calls that degraded to sleep pacing because a listener drain failed.
    pub degraded_waits: AtomicU64,
    /// Adaptive block ladder: the CURRENT rung in µs (`0` = never armed;
    /// re-derived from the config at each call). Persists ACROSS calls —
    /// a ping-pong executor returns from every call quickly, and the rung
    /// must still be the first one when it comes back.
    block_rung_us: AtomicU64,
    /// Consecutive EMPTY timeouts (no fd wake, nothing ready) since the
    /// last reset — the ladder's idleness evidence.
    consecutive_empty: AtomicU64,
    /// Ladder resets to the first rung (every fd wake or ready-return —
    /// counted whether or not the rung had moved).
    pub block_rung_resets: AtomicU64,
    /// Ladder backoffs: the rung actually doubled (reaching the cap is a
    /// plateau, not a backoff).
    pub block_backoffs: AtomicU64,
    /// Spin PHASES entered (one per bounded busy-probe run). The spin runs
    /// at most ONCE per call on entry — and only when the previous call
    /// was productive (see `spin_disarmed`) — plus at most one recovery
    /// spin after an fd wake whose probe found nothing; never after an
    /// EMPTY timeout. An idle EXECUTOR calling `rmw_wait` in a loop
    /// therefore records exactly 1 per idle period, however long.
    pub spin_phases: AtomicU64,
    /// PARK tier blocks entered (the doorbell-armed idle; Linux only).
    pub park_blocks: AtomicU64,
    /// Park blocks woken by a topic DOORBELL ring (the SHM line the
    /// publisher stores after every send — the instant data wake).
    pub park_wakes_doorbell: AtomicU64,
    /// Stale bells RE-MAPPED by the wait's stale-bell reconcile
    /// (a frame arrived without its bell ringing — the producer was
    /// destroyed and re-created on the same topic).
    pub park_bell_reopens: AtomicU64,
    /// Park recheck slices that timed out and YIELDED the core
    /// (`sched_yield`) — the OS-cooperative park: a co-located runnable
    /// peer gets the core within one slice instead of at CFS granularity.
    pub park_yields: AtomicU64,
    /// Listener / guard-doorbell DRAINS performed (each is ≥1 syscall),
    /// counted by BOTH drainers — the fired-only one and the legacy
    /// drain-all — so the per-hop oracle sees a drain-all loop as 9 per
    /// iteration. Fired-only draining keeps this O(fired) per wake.
    pub drain_calls: AtomicU64,
    /// SHM readiness probes (`has_pending_*`) run by a call's ENTRY pass —
    /// the one full pass per call the rmw contract needs (a second queued
    /// sample has no notification left to fire an fd for).
    pub probes_entry: AtomicU64,
    /// SHM readiness probes run AFTER a wake — fired entities plus the
    /// bell-named topic only; never the whole set.
    pub probes_wake: AtomicU64,
    /// The entity behind each watched fd, parallel to `fd_set`.
    fd_entities: Mutex<Vec<EntityRef>>,
    /// Reusable readiness mask for the single consuming pass (flat index
    /// over subs, guards, services, clients — see [`flat_index`]).
    ready_buf: Mutex<Vec<bool>>,
    /// The park tier's mapped topic doorbells (consumer side).
    park_bells: Mutex<ParkBells>,
    /// Period-learned park: per-topic arrival learners and the clock epoch they count from.
    #[cfg(feature = "period-park")]
    period: Mutex<PeriodState>,
    /// Parks entered from a predicted window: the kernel lead timed out (or there was none).
    #[cfg(feature = "period-park")]
    pub predicted_parks: AtomicU64,
    /// Predicted parks that returned on the predicted topic's own bell.
    #[cfg(feature = "period-park")]
    pub predicted_bell_wakes: AtomicU64,
    /// Predicted bell wakes found at the park's first check: the predicted bell had rung
    /// after the learners' last note or sync, during the kernel lead or before the park
    /// started.
    #[cfg(feature = "period-park")]
    pub predicted_rings_before_park: AtomicU64,
}

/// Arrival learners for the wait set's topics, timed in ns since `epoch` (the first arrival noted).
#[cfg(feature = "period-park")]
#[derive(Default)]
struct PeriodState {
    epoch: Option<Instant>,
    learners: super::period_learner::TopicLearners,
}

/// `at` in ns since `epoch`: 0 before it, `u64::MAX` past the u64 range.
#[cfg(feature = "period-park")]
fn ns_since(epoch: Instant, at: Instant) -> u64 {
    u64::try_from(at.saturating_duration_since(epoch).as_nanos()).unwrap_or(u64::MAX)
}

// A predicted park is never longer than the one-rung park.
#[cfg(feature = "period-park")]
const _: () = assert!(super::period_learner::MAX_PARK_NS <= PARK_HORIZON_US * 1_000);

#[cfg(feature = "period-park")]
impl WaitSetData {
    /// Match the learners to the mapped bells if the bell set changed since the last match.
    fn map_learners(&self) {
        let bells = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        let mut st = self.period.lock().unwrap_or_else(|e| e.into_inner());
        Self::map_learners_locked(&bells, &mut st);
    }

    fn map_learners_locked(bells: &ParkBells, st: &mut PeriodState) {
        if st.learners.is_mapped(bells.generation) {
            return;
        }
        // hot-path-alloc-ok: one Vec per bell set change, never per wake.
        let seqs: Vec<(&str, u64)> = bells
            .bells
            .iter()
            .map(|(t, b)| (t.as_str(), b.seq()))
            .collect();
        st.learners.remap(bells.generation, &seqs);
    }

    /// Mark every mapped bell's current seq as seen, with no arrival observed. The call's
    /// entry pass runs this before its full readiness probe, and the probe serves every
    /// message committed before this read (a publisher commits, then rings), so the next
    /// predicted park ([`ParkRef::Noted`]) reacts only to rings after it.
    fn sync_noted_bells(&self) {
        let bells = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        let mut st = self.period.lock().unwrap_or_else(|e| e.into_inner());
        Self::map_learners_locked(&bells, &mut st);
        st.learners.sync(bells.bells.iter().map(|(_, b)| b.seq()));
    }

    /// Record an arrival at `at` for every mapped bell whose seq moved since the last note,
    /// and apply the last predicted block's end to its bell's learner.
    fn note_rung_bells(&self, at: Instant) {
        let bells = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        let mut st = self.period.lock().unwrap_or_else(|e| e.into_inner());
        Self::map_learners_locked(&bells, &mut st);
        let now = ns_since(*st.epoch.get_or_insert(at), at);
        st.learners
            .note(now, bells.bells.iter().map(|(_, b)| b.seq()));
    }

    /// The window that opens first among the mapped bells' predictions still open at `at`,
    /// as (opens at, length, bell index).
    fn next_window(&self, at: Instant) -> Option<(Instant, Duration, usize)> {
        let bells = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        let st = self.period.lock().unwrap_or_else(|e| e.into_inner());
        if !st.learners.is_mapped(bells.generation) {
            return None;
        }
        let epoch = st.epoch?;
        let (bell, p) = st.learners.next(ns_since(epoch, at))?;
        let open = epoch.checked_add(Duration::from_nanos(p.open_ns()))?;
        Some((open, Duration::from_nanos(p.close_ns() - p.open_ns()), bell))
    }

    /// This block's predicted plan as (kernel lead, park end, bell index), or `None` for the
    /// stock block. The park end is an instant ([`predicted_block`]).
    fn predicted_plan(&self, budget: Duration) -> Option<(Duration, Instant, usize)> {
        let now = Instant::now();
        let (open, len, bell) = self.next_window(now)?;
        let (lead, park_end) = predicted_block(now, Some((open, len)), budget)?;
        Some((lead, park_end, bell))
    }

    /// Run a predicted plan: a kernel fd block of `lead`, then, if it timed out, a park armed
    /// on bell `bell` until the instant `park_end`, measuring a ring against the learners'
    /// last note or sync ([`ParkRef::Noted`]). A lead that wakes late leaves the park less time, never
    /// a later end. How it ended ([`window_end`]) is kept for the next
    /// [`Self::note_rung_bells`].
    fn predicted_wait(&self, (lead, park_end, bell): (Duration, Instant, usize)) -> BlockWake {
        let lead_wake = if lead.is_zero() {
            BlockWake::Timeout
        } else {
            self.fd_block(lead)
        };
        let park = if lead_wake == BlockWake::Timeout {
            self.predicted_parks.fetch_add(1, Ordering::Relaxed);
            let end = self.park_until(park_end, bell, ParkRef::Noted);
            if end.wake == BlockWake::Bell(bell) {
                self.predicted_bell_wakes.fetch_add(1, Ordering::Relaxed);
                if end.at_entry {
                    self.predicted_rings_before_park
                        .fetch_add(1, Ordering::Relaxed);
                }
            }
            Some(end)
        } else {
            None
        };
        let ended = window_end(lead_wake, park, bell).map(|end| (bell, end));
        self.period
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .learners
            .end_window(ended);
        park.map_or(lead_wake, |end| end.wake)
    }
}

/// The wait set's mapped topic doorbells — rebuilt when the subscription
/// topic set changes; a topic whose open FAILED (the consumer-side open
/// `O_CREAT`s, so "producer not up yet" is not a failure — only ENOSPC /
/// EACCES-class errors are) is retried on a throttle; a bell that goes
/// STALE (its producer re-created its page) is re-mapped by the reconcile
/// below.
#[derive(Default)]
struct ParkBells {
    /// The subscription topic set these bells were built for.
    topics: Vec<String>,
    /// Successfully mapped bells (consumer, unowned), by topic — the name
    /// is what a stale bell is re-opened under.
    bells: Vec<(String, cerulion_core::doorbell::Doorbell)>,
    /// Topics whose page could not be mapped yet.
    missing: Vec<String>,
    /// Calls since the last retry of `missing`.
    calls_since_retry: u32,
    /// Each bell's reference seq for the LAST park (parallel to `bells`): read
    /// at the park's entry, or for a predicted park the seq at the period
    /// learners' last note or sync ([`ParkRef`]). What "stayed silent" is
    /// measured against.
    last_snap: Vec<u64>,
    /// The last park returned via the FD path with every bell silent —
    /// a frame may have arrived without its bell ringing; reconcile.
    fd_wake_suspect: bool,
    /// Bumped on every rebuild or retry of `bells`, so the period learners re-match by position.
    #[cfg(feature = "period-park")]
    generation: u64,
}

/// PURE: is a park bell STALE? A frame reached the subscription (a sample
/// is pending) while the bell that should have rung for it is still at
/// the value the park snapshotted. The publisher rings BEFORE it notifies
/// the listener (both after the sample is committed), so by the time an
/// fd wake delivers a frame a HEALTHY bell has already advanced — a bell
/// still silent with a frame behind it maps a page nobody writes any
/// more: the producer was destroyed (the owner `shm_unlink`s the name on
/// drop) and re-created (a fresh inode under the same name).
fn bell_is_stale(sample_pending: bool, seq_now: u64, seq_at_park: u64) -> bool {
    sample_pending && seq_now == seq_at_park
}

/// Retry throttle for missing doorbell pages (calls between attempts).
const PARK_BELL_RETRY_CALLS: u32 = 256;

impl WaitSetData {
    /// Unconditional running total of listener-drain failures on this wait
    /// set (never reset by recovery — Principle #3).
    pub fn drain_failure_count(&self) -> u64 {
        lock_regime_latch(&self.drain_failures).total_failures()
    }

    /// Diagnostics (Principle #3): how many fds the last call's snapshot
    /// watches (dead, neutralized entries included).
    pub fn watched_fds(&self) -> usize {
        self.fd_set.lock().unwrap_or_else(|e| e.into_inner()).len()
    }

    /// Diagnostics: the ladder's current rung in µs (`0` = never armed).
    pub fn block_rung_us(&self) -> u64 {
        self.block_rung_us.load(Ordering::Relaxed)
    }

    /// Diagnostics: consecutive empty timeouts since the last reset.
    pub fn consecutive_empty(&self) -> u64 {
        self.consecutive_empty.load(Ordering::Relaxed)
    }

    /// Rebuild / retry the park tier's doorbell mappings for `topics`
    /// (the call's subscription topic set). A changed set rebuilds; a topic
    /// whose open failed is retried every [`PARK_BELL_RETRY_CALLS`] calls,
    /// so the steady state costs one Vec compare. (The open `O_CREAT`s, so
    /// a producer that is not up yet does NOT leave a topic missing — the
    /// consumer creates the page and the producer joins it by name.)
    fn refresh_park_bells(&self, topics: &[String]) {
        let mut pb = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        if pb.topics != topics {
            pb.topics = topics.to_vec();
            pb.bells.clear();
            pb.missing.clear();
            pb.calls_since_retry = 0;
            #[cfg(feature = "period-park")]
            {
                pb.generation = pb.generation.wrapping_add(1);
            }
            for t in topics {
                match cerulion_core::doorbell::Doorbell::open_unowned(doorbell_ns(), t) {
                    Ok(b) => pb.bells.push((t.clone(), b)),
                    Err(_) => pb.missing.push(t.clone()),
                }
            }
        } else if !pb.missing.is_empty() {
            pb.calls_since_retry += 1;
            if pb.calls_since_retry >= PARK_BELL_RETRY_CALLS {
                pb.calls_since_retry = 0;
                #[cfg(feature = "period-park")]
                {
                    pb.generation = pb.generation.wrapping_add(1);
                }
                let miss = std::mem::take(&mut pb.missing);
                for t in miss {
                    match cerulion_core::doorbell::Doorbell::open_unowned(doorbell_ns(), &t) {
                        Ok(b) => pb.bells.push((t, b)),
                        Err(_) => pb.missing.push(t),
                    }
                }
            }
        }
    }

    /// Diagnostics: how many topic doorbells the park tier has mapped.
    pub fn doorbell_topics(&self) -> usize {
        self.park_bells
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .bells
            .len()
    }

    /// After a park that returned via the FD path with every bell silent:
    /// for each bell, ask whether its subscription now holds a pending
    /// sample; if it does, the bell is stale ([`bell_is_stale`]) — re-map
    /// it (`open_unowned` of the same name, which now names the
    /// replacement producer's page) so the NEXT publish rings this wait
    /// set again. The rmw analog of the native `DoorbellRegistry::reopen`
    /// on a producer-reconnect `LivelinessEvent`; the rmw has no
    /// liveliness hook, but "a frame arrived and the bell did not ring"
    /// is the same evidence, read off the data path. Exact and cheap:
    /// nothing runs unless a park woke via the fd (a doorbell wake clears
    /// the suspect flag), and a bell is re-opened only when a frame
    /// provably arrived without it. Until then a stale bell costs ONE
    /// recheck slice (~100 µs) per wake — the park re-checks the fd
    /// snapshot every slice — never the 20 ms timer. (`open_unowned`
    /// `O_CREAT`s, so the re-open lands on the replacement producer's page
    /// if it is already there, else creates the page that producer will
    /// join by name; a genuinely failed re-open keeps the old mapping and
    /// is retried on the next stale detection.)
    ///
    /// # Safety
    /// Same contract as [`check_ready`] for `subscriptions`.
    unsafe fn reconcile_stale_bells(&self, subscriptions: *mut ffi::rmw_subscriptions_t) {
        let mut pb = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        if !std::mem::take(&mut pb.fd_wake_suspect) || subscriptions.is_null() {
            return;
        }
        let subs = &*subscriptions;
        for i in 0..pb.bells.len().min(pb.last_snap.len()) {
            let topic = pb.bells[i].0.clone();
            let seq_at_park = pb.last_snap[i];
            let seq_now = pb.bells[i].1.seq();
            let pending = (0..subs.subscriber_count).any(|j| {
                let handle = *subs.subscribers.add(j);
                if handle.is_null() {
                    return false;
                }
                let data = &*(handle as *const SubscriptionData);
                data.topic == topic
                    && runtime::lock_unpoisoned(&data.inner)
                        .is_some_and(|inner| inner.subscriber.has_pending_sample())
            });
            if bell_is_stale(pending, seq_now, seq_at_park) {
                if let Ok(fresh) =
                    cerulion_core::doorbell::Doorbell::open_unowned(doorbell_ns(), &topic)
                {
                    pb.bells[i].1 = fresh;
                    self.park_bell_reopens.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    /// One fd-tier block: the ppoll/poll wait on the fd snapshot; the fds
    /// that fired are left in `FdWakeSet::fired` for the caller's
    /// fired-only drain/probe. Counters attributed here.
    fn fd_block(&self, budget: Duration) -> BlockWake {
        self.fd_blocks.fetch_add(1, Ordering::Relaxed);
        let outcome = self
            .fd_set
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .wait(budget);
        match outcome {
            FdWake::Fired => {
                self.fd_wakes.fetch_add(1, Ordering::Relaxed);
                BlockWake::Fd
            }
            FdWake::TimedOut => {
                self.timeout_wakes.fetch_add(1, Ordering::Relaxed);
                BlockWake::Timeout
            }
        }
    }

    /// The topic behind park bell `idx` (for the bell-named probe).
    fn bell_topic(&self, idx: usize) -> Option<String> {
        self.park_bells
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .bells
            .get(idx)
            .map(|(t, _)| t.clone())
    }

    /// One PARK-tier block: idle on the topic doorbells' SHM lines —
    /// the hardware monitor-wait (`UMWAIT`/`WFE`) armed on the primary
    /// line (bell `primary`; an index past the mapped bells arms the first)
    /// when the CPU has one, a bounded sleep-recheck slice otherwise
    /// (never a busy-spin) — re-checking every doorbell and the fd
    /// snapshot each [`PARK_RECHECK`] slice, and YIELDING the core after
    /// every slice that timed out. The yield is what makes the park
    /// OS-cooperative: to the scheduler a parked thread is RUNNING, so
    /// without it a co-located peer (the publisher whose ring this park is
    /// waiting for) gets the core only at CFS wakeup granularity — measured
    /// 7 ms with both ping-pong threads pinned to one CPU. A
    /// doorbell ring wakes hardware-instantly on the primary and within
    /// one slice on the rest; listener/guard fds wake within one slice.
    fn park_block(&self, budget: Duration, primary: usize) -> BlockWake {
        let Some(deadline) = Instant::now().checked_add(budget) else {
            self.park_blocks.fetch_add(1, Ordering::Relaxed);
            return BlockWake::Timeout;
        };
        self.park_until(deadline, primary, ParkRef::Entry).wake
    }

    /// The park of [`Self::park_block`], ending at the instant `deadline` and
    /// measuring a ring against `reference`. Each pass checks the bells first,
    /// then the fds, then the deadline, so a bell that already differs from the
    /// reference returns at the first check ([`ParkEnd::at_entry`]) even when
    /// the deadline has passed.
    fn park_until(&self, deadline: Instant, primary: usize, reference: ParkRef) -> ParkEnd {
        self.park_blocks.fetch_add(1, Ordering::Relaxed);
        let mut bells = self.park_bells.lock().unwrap_or_else(|e| e.into_inner());
        let primary = if primary < bells.bells.len() {
            primary
        } else {
            0
        };
        // hot-path-alloc-ok: one small Vec per park entry (the idle path,
        // never the frame path).
        let snap: Vec<u64> = match reference {
            ParkRef::Entry => bells.bells.iter().map(|(_, b)| b.seq()).collect(),
            #[cfg(feature = "period-park")]
            ParkRef::Noted => {
                let st = self.period.lock().unwrap_or_else(|e| e.into_inner());
                let noted: Vec<u64> = if st.learners.is_mapped(bells.generation) {
                    st.learners.noted_seqs().collect()
                } else {
                    Vec::new()
                };
                if noted.len() == bells.bells.len() {
                    noted
                } else {
                    // Learners not matched to this bell set: the entry seqs.
                    bells.bells.iter().map(|(_, b)| b.seq()).collect()
                }
            }
        };
        bells.last_snap.clone_from(&snap);
        bells.fd_wake_suspect = false;
        let mut first_check = true;
        loop {
            if let Some(idx) = bells
                .bells
                .iter()
                .zip(snap.iter())
                .position(|((_, b), s)| b.seq() != *s)
            {
                self.park_wakes_doorbell.fetch_add(1, Ordering::Relaxed);
                return ParkEnd {
                    wake: BlockWake::Bell(idx),
                    at_entry: first_check,
                };
            }
            first_check = false;
            if self
                .fd_set
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .check()
                == FdWake::Fired
            {
                // Every bell was silent at the check above — if this fd
                // wake carries a frame for one of them, that bell is stale
                // (see `reconcile_stale_bells`, run by the caller).
                bells.fd_wake_suspect = true;
                self.fd_wakes.fetch_add(1, Ordering::Relaxed);
                return ParkEnd {
                    wake: BlockWake::Fd,
                    at_entry: false,
                };
            }
            let now = Instant::now();
            if now >= deadline {
                self.timeout_wakes.fetch_add(1, Ordering::Relaxed);
                return ParkEnd {
                    wake: BlockWake::Timeout,
                    at_entry: false,
                };
            }
            let slice_deadline = deadline.min(now + PARK_RECHECK);
            let performed = match bells.bells.get(primary) {
                Some((_, primary_bell)) => {
                    let exp_now = primary_bell.seq();
                    if snap.get(primary).is_some_and(|&s| exp_now != s) {
                        continue; // rang between the check above and here
                    }
                    // SAFETY: the mapping lives in `bells`, whose MutexGuard
                    // is held for the whole park, so the address stays
                    // mapped; the doorbell contract guarantees an aligned
                    // `AtomicU64`. `exp_now` is the just-read current value
                    // (the lost-wakeup `expected`).
                    // `monitor_wait_until_addr` returns
                    // an `AddrParkOutcome`; map it to the bool this loop
                    // needs ("skip the degraded sleep?"). rmw's own
                    // `park_yields` accounting is not gated on
                    // `.parked()`.
                    unsafe {
                        cerulion_core::monitor_wait::monitor_wait_until_addr(
                            primary_bell.addr() as *const u64,
                            exp_now,
                            slice_deadline,
                            PARK_RECHECK,
                        )
                    }
                    .performed()
                }
                None => false,
            };
            if !performed {
                // Degraded slice: a bounded sleep-recheck, never a busy-spin.
                std::thread::sleep(
                    slice_deadline
                        .saturating_duration_since(Instant::now())
                        .min(PARK_RECHECK),
                );
            }
            // The slice timed out (a ring would have returned above):
            // give the core to whoever is runnable before the next slice.
            std::thread::yield_now();
            self.park_yields.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// The ladder's current rung as a block budget, armed from `cfg` if
    /// this set has never blocked (or the config changed under it).
    fn ladder_rung(&self, cfg: &BlockLadder) -> Duration {
        let rung = self.block_rung_us.load(Ordering::Relaxed);
        let reset = cfg.reset_rung_us();
        let rung = if rung == 0 || rung < reset || rung > BLOCK_CAP_US {
            self.block_rung_us.store(reset, Ordering::Relaxed);
            reset
        } else {
            rung
        };
        Duration::from_micros(rung)
    }

    /// ANY fd wake or ready-return: back to the first rung.
    fn ladder_reset(&self, cfg: &BlockLadder) {
        self.consecutive_empty.store(0, Ordering::Relaxed);
        self.block_rung_us
            .store(cfg.reset_rung_us(), Ordering::Relaxed);
        self.block_rung_resets.fetch_add(1, Ordering::Relaxed);
    }

    /// One more EMPTY timeout: count it and maybe double the rung.
    fn ladder_empty(&self, cfg: &BlockLadder) {
        let empties = self.consecutive_empty.fetch_add(1, Ordering::Relaxed) + 1;
        let rung = self.block_rung_us.load(Ordering::Relaxed);
        let (next, backed_off) = next_rung_after_empty(rung, empties, cfg);
        if backed_off {
            self.block_rung_us.store(next, Ordering::Relaxed);
            self.block_backoffs.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_create_wait_set(
    context: *mut ffi::rmw_context_t,
    _max_conditions: usize,
) -> *mut ffi::rmw_wait_set_t {
    if context.is_null() || !ffi::is_our_identifier((*context).implementation_identifier) {
        return std::ptr::null_mut();
    }
    let ws = Box::new(ffi::rmw_wait_set_t {
        implementation_identifier: ffi::implementation_identifier_ptr(),
        data: Box::into_raw(Box::new(WaitSetData::default())) as *mut std::os::raw::c_void,
        guard_conditions: std::ptr::null_mut(),
    });
    Box::into_raw(ws)
}

/// # Safety
/// `wait_set` must come from `rmw_create_wait_set`.
#[no_mangle]
pub unsafe extern "C" fn rmw_destroy_wait_set(wait_set: *mut ffi::rmw_wait_set_t) -> rmw_ret_t {
    if wait_set.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if !ffi::is_our_identifier((*wait_set).implementation_identifier) {
        return ffi::RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    drop(Box::from_raw((*wait_set).data as *mut WaitSetData));
    drop(Box::from_raw(wait_set));
    RMW_RET_OK
}

/// # Safety
/// Caller frees with `rmw_wait_set_free`.
#[no_mangle]
pub unsafe extern "C" fn rmw_wait_set_allocate() -> *mut ffi::rmw_wait_set_t {
    Box::into_raw(Box::new(std::mem::zeroed::<ffi::rmw_wait_set_t>()))
}

/// # Safety
/// `wait_set` must come from `rmw_wait_set_allocate`.
#[no_mangle]
pub unsafe extern "C" fn rmw_wait_set_free(wait_set: *mut ffi::rmw_wait_set_t) {
    if !wait_set.is_null() {
        drop(Box::from_raw(wait_set));
    }
}

/// Readiness check across all attached entity arrays. Per the rmw
/// contract, NOT-ready entries are nulled out in place; ready entries
/// stay.
unsafe fn check_ready(
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
    consume_guards: bool,
) -> bool {
    let mut any = false;

    if !subscriptions.is_null() {
        let subs = &mut *subscriptions;
        for i in 0..subs.subscriber_count {
            let handle = *subs.subscribers.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const SubscriptionData);
            let ready = match runtime::lock_unpoisoned(&data.inner) {
                Some(inner) => inner.subscriber.has_pending_sample(),
                None => false, // a wedged entity is never ready
            };
            if ready {
                any = true;
            } else if consume_guards {
                *subs.subscribers.add(i) = std::ptr::null_mut();
            }
        }
    }

    if !guard_conditions.is_null() {
        let guards = &mut *guard_conditions;
        for i in 0..guards.guard_condition_count {
            let handle = *guards.guard_conditions.add(i);
            if handle.is_null() {
                continue;
            }
            // rcl attaches the guard's DATA pointer (GuardConditionState).
            let state = &*(handle as *const GuardConditionState);
            let ready = if consume_guards {
                let fired = state.triggered.swap(false, Ordering::AcqRel);
                if fired {
                    // A consumed trigger's doorbell ring must not
                    // survive to re-fire the next wait's block forever (the
                    // stale-signal spin class). A ring racing this consume
                    // (store→[swap+drain]→ring) can still leave residue —
                    // the loop-top drain in `rmw_wait` is the backstop.
                    if let Some(d) = &state.doorbell {
                        d.drain();
                    }
                }
                fired
            } else {
                state.triggered.load(Ordering::Acquire)
            };
            if ready {
                any = true;
            } else if consume_guards {
                *guards.guard_conditions.add(i) = std::ptr::null_mut();
            }
        }
    }

    if !services.is_null() {
        let svcs = &mut *services;
        for i in 0..svcs.service_count {
            let handle = *svcs.services.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ServiceData);
            let ready = match runtime::lock_unpoisoned(&data.server) {
                Some(server) => server.has_pending_request(),
                None => false, // wedged: never ready
            };
            if ready {
                any = true;
            } else if consume_guards {
                *svcs.services.add(i) = std::ptr::null_mut();
            }
        }
    }

    if !clients.is_null() {
        let cls = &mut *clients;
        for i in 0..cls.client_count {
            let handle = *cls.clients.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ClientData);
            let ready = match runtime::lock_unpoisoned(&data.client) {
                Some(client) => client.has_pending_response(),
                None => false, // wedged: never ready
            };
            if ready {
                any = true;
            } else if consume_guards {
                *cls.clients.add(i) = std::ptr::null_mut();
            }
        }
    }

    any
}

/// Is the event-driven block enabled? `CERULION_RMW_EVENT_WAIT=off` is the
/// event-wait kill-switch (loud `info!` once); an unrecognized value warns
/// once and keeps the default ON (exact match, no case forgiveness — the
/// CERULION_DRAIN_DISCIPLINE convention). Read per call: cheap at wait
/// frequency, and it lets a test/operator toggle take effect within one
/// process.
fn event_wait_enabled() -> bool {
    match std::env::var("CERULION_RMW_EVENT_WAIT") {
        Err(_) => true,
        Ok(v) if v.is_empty() || v == "on" => true,
        Ok(v) if v == "off" => {
            static OFF_NOTICE: std::sync::Once = std::sync::Once::new();
            OFF_NOTICE.call_once(|| {
                tracing::info!(
                    "CERULION_RMW_EVENT_WAIT=off — event-driven rmw_wait disabled; \
                     using the legacy 100µs sleep-poll verbatim: no fd block AND no \
                     spin (CERULION_LIVE_SPIN_US is ignored on this path)"
                );
            });
            false
        }
        Ok(v) => {
            static BAD_NOTICE: std::sync::Once = std::sync::Once::new();
            BAD_NOTICE.call_once(|| {
                tracing::warn!(
                    value = %v,
                    "unrecognized CERULION_RMW_EVENT_WAIT value (recognized: \
                     \"off\", \"on\"; exact match) — keeping the default \
                     event-driven wait ON"
                );
            });
            true
        }
    }
}

/// The rmw spin budget — the SHARED `CERULION_LIVE_SPIN_US` knob (one
/// surface with the native live loop; classification lives in
/// `cerulion_core::monitor_wait::classify_live_spin_us`, so the two
/// consumers cannot drift). The mapping has one asymmetry:
/// `Derived` (unset) means "the consumer's own default", and the rmw
/// wait's default is PARK-FIRST — no user-space spin — because unlike the
/// native loop it has no graph to derive wake-imminence from. `0`
/// disables; a ceiling arrives already clamped; malformed warns once and
/// takes the park-first default.
fn spin_budget() -> Duration {
    use cerulion_core::monitor_wait::LiveSpinSetting;
    let raw = std::env::var("CERULION_LIVE_SPIN_US").ok();
    match cerulion_core::monitor_wait::classify_live_spin_us(raw.as_deref()) {
        LiveSpinSetting::Derived | LiveSpinSetting::Disabled => Duration::ZERO,
        LiveSpinSetting::Ceiling { us, clamped } => {
            if clamped {
                static CLAMP_NOTICE: std::sync::Once = std::sync::Once::new();
                CLAMP_NOTICE.call_once(|| {
                    tracing::warn!(
                        value = %raw.as_deref().unwrap_or(""),
                        clamped_to_us = cerulion_core::monitor_wait::SPIN_BUDGET_MAX_US,
                        "CERULION_LIVE_SPIN_US above the 100ms ceiling — clamped for \
                         the rmw wait (the same one ceiling the native live loop \
                         applies)"
                    );
                });
            }
            Duration::from_micros(us)
        }
        LiveSpinSetting::Malformed => {
            static BAD_NOTICE: std::sync::Once = std::sync::Once::new();
            BAD_NOTICE.call_once(|| {
                tracing::warn!(
                    value = %raw.as_deref().unwrap_or(""),
                    "CERULION_LIVE_SPIN_US is not a non-negative µs integer — the \
                     rmw wait keeps its park-first default (no spin)"
                );
            });
            Duration::ZERO
        }
    }
}

/// The park horizon for THIS process: the SHARED `CERULION_MONITOR_WAIT`
/// flag ([`resolve_park`]) over the platform default ([`park_policy`]);
/// a near-miss value warns once and keeps the default. Returned with the
/// flag, which [`predicts_windows`] also reads.
fn park_horizon_from_env() -> (cerulion_core::monitor_wait::EnvFlag, ParkHorizon) {
    use cerulion_core::monitor_wait::EnvFlag;
    let raw = std::env::var("CERULION_MONITOR_WAIT").ok();
    let flag = cerulion_core::monitor_wait::classify_env_flag(raw.as_deref());
    if flag == EnvFlag::NearMiss {
        static NOTICE: std::sync::Once = std::sync::Once::new();
        NOTICE.call_once(|| {
            tracing::warn!(
                value = %raw.as_deref().unwrap_or(""),
                "ignoring unrecognized CERULION_MONITOR_WAIT value — only \"1\" \
                 (force the park on) and \"0\" (off) are honored; the rmw wait \
                 keeps its platform default (one-rung park on aarch64, off on x86)"
            );
        });
    }
    (flag, resolve_park(flag, park_policy()))
}

/// Does a ring WAKE this wait's park on this target? Only on Linux, where the
/// CPU monitor-wait primitive the park arms is watching the doorbell line the
/// ring stores into; on a Linux host whose CPU carries no such primitive this
/// wait takes [`ParkHorizon::NoPark`] anyway. Android maps the real ring counter
/// but has no CPU monitor-wait primitive for this park. Android, macOS and a
/// Linux host without the primitive map a real doorbell page with a real wake
/// word, but this wait has no kernel block on that word, so a park there would
/// trade the fd block's instant wake for a 100 µs recheck cadence and buy
/// nothing; the wake-word block is the native live loop's rung
/// (`cerulion_core::doorbell::wake_word_block_primitive_available`). Every
/// other target has neither.
const RING_WAKES_THE_PARK: bool = cfg!(target_os = "linux");

/// The doorbell namespace both sides agree on: the SAME
/// `default_namespace()` (`$USER`) on the arming side
/// (`rmw_create_publisher`) and the parking side (this wait).
fn doorbell_ns() -> &'static str {
    static NS: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    NS.get_or_init(cerulion_core::doorbell::default_namespace)
}

/// Tighten the calling thread's kernel timer slack to 1 ns.
///
/// Linux and Android stretch every non-RT timed block by the thread's
/// `timer_slack_ns` (default 50 µs): the legacy 100 µs `POLL_INTERVAL`
/// sleep really costs ~153 µs, and the event-driven ladder's 200 µs
/// `ppoll` first rung would pay the same +50 µs on every empty rung.
/// This DELIBERATELY changes sleep/timer precision for the WHOLE executor
/// thread (any thread that ever enters `rmw_wait`), not just the wait:
/// `prctl(PR_SET_TIMERSLACK)` is thread-scoped and no narrower scope
/// exists. Applied at most once per thread (the thread-local guard), so
/// steady-state waits pay one `Cell` read.
///
/// CRITICAL: the slack argument is 1, NEVER 0 — the kernel treats
/// `arg2 <= 0` as "restore the 50 µs default" (kernel/sys.c:
/// `current->timer_slack_ns = current->default_timer_slack_ns`), the
/// exact opposite of the intent.
#[cfg(any(target_os = "linux", target_os = "android"))]
fn set_timer_slack_once() {
    use std::cell::Cell;
    thread_local! {
        static SLACK_SET: Cell<bool> = const { Cell::new(false) };
    }
    SLACK_SET.with(|set| {
        if set.get() {
            return;
        }
        set.set(true);
        // SAFETY: prctl(PR_SET_TIMERSLACK) takes plain integer
        // arguments and touches no memory; it affects only the calling
        // thread's timer slack.
        let ret = unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, 1usize) };
        if ret != 0 {
            // Best-effort: the wait still works at default slack, just
            // ~50 µs slower per timed block. At most once per thread.
            tracing::debug!(
                error = %std::io::Error::last_os_error(),
                "PR_SET_TIMERSLACK failed; timed blocks keep the default timer slack"
            );
        }
    });
}

/// Timer slack is a Linux and Android kernel concept; no-op elsewhere.
#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn set_timer_slack_once() {}

/// How many entities the spin could actually OBSERVE: non-null handles
/// across the four arrays whose entity lock is HEALTHY (the same
/// `lock_unpoisoned` gate `check_ready` and `collect_wake_fds` apply — a
/// POISONED entity is permanently invisible to the probe and absent from
/// the fd snapshot, so a wait set holding only wedged entities is
/// observably EMPTY and must not buy a spin budget either; guard
/// conditions carry no lock and are always probe-able). ZERO means
/// nothing the wait could ever observe as ready: the spin must not run
/// (an entry spin at the 100 ms cap on an EMPTY wait set was measured at
/// ~480k probes and ~100 ms of thread CPU per 200 ms wait; the
/// wedged-entity variant burns the same), and the fd snapshot is empty
/// so the block degrades to the bounded sleep anyway.
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn count_attached_entities(
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
) -> usize {
    let mut n = 0usize;
    if !subscriptions.is_null() {
        let subs = &*subscriptions;
        n += (0..subs.subscriber_count)
            .filter(|&i| {
                let handle = *subs.subscribers.add(i);
                !handle.is_null()
                    && runtime::lock_unpoisoned(&(*(handle as *const SubscriptionData)).inner)
                        .is_some()
            })
            .count();
    }
    if !guard_conditions.is_null() {
        // Guard conditions carry no lock (an atomic flag + an owned
        // doorbell) — a non-null guard is always probe-able.
        let guards = &*guard_conditions;
        n += (0..guards.guard_condition_count)
            .filter(|&i| !(*guards.guard_conditions.add(i)).is_null())
            .count();
    }
    if !services.is_null() {
        let svcs = &*services;
        n += (0..svcs.service_count)
            .filter(|&i| {
                let handle = *svcs.services.add(i);
                !handle.is_null()
                    && runtime::lock_unpoisoned(&(*(handle as *const ServiceData)).server).is_some()
            })
            .count();
    }
    if !clients.is_null() {
        let cls = &*clients;
        n += (0..cls.client_count)
            .filter(|&i| {
                let handle = *cls.clients.add(i);
                !handle.is_null()
                    && runtime::lock_unpoisoned(&(*(handle as *const ClientData)).client).is_some()
            })
            .count();
    }
    n
}

/// Snapshot every attached entity's wake fd into `set` — the subscription
/// listeners, each guard's doorbell, and the service/client subscribers'
/// listeners (services and clients ride the SAME listener path: their
/// request/response sends `notify_sent_sample`, which lands on the
/// `request_subscriber()`/`reply_subscriber()` listener these fds name).
///
/// Lock discipline: each entity mutex is held only long enough to read an
/// integer — never across the block. A poisoned (wedged) entity
/// contributes no fd, matching `check_ready`'s "wedged is never ready"
/// (it cannot become ready, so it needs no wake).
///
/// # Safety
/// Same contract as [`check_ready`]: arrays valid for their counts;
/// handles are this implementation's entity data pointers.
unsafe fn collect_wake_fds(
    set: &mut FdWakeSet,
    entities: &mut Vec<EntityRef>,
    sub_topics: &mut Vec<String>,
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
) {
    if !subscriptions.is_null() {
        let subs = &*subscriptions;
        for i in 0..subs.subscriber_count {
            let handle = *subs.subscribers.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const SubscriptionData);
            sub_topics.push(data.topic.clone());
            if let Some(inner) = runtime::lock_unpoisoned(&data.inner) {
                entities.push(EntityRef::Sub(i));
                set.watch(inner.subscriber.event_listener_fd());
            }
        }
    }
    if !guard_conditions.is_null() {
        let guards = &*guard_conditions;
        for i in 0..guards.guard_condition_count {
            let handle = *guards.guard_conditions.add(i);
            if handle.is_null() {
                continue;
            }
            let state = &*(handle as *const GuardConditionState);
            if let Some(d) = &state.doorbell {
                entities.push(EntityRef::Guard(i));
                set.watch(d.poll_fd());
            }
        }
    }
    if !services.is_null() {
        let svcs = &*services;
        for i in 0..svcs.service_count {
            let handle = *svcs.services.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ServiceData);
            if let Some(server) = runtime::lock_unpoisoned(&data.server) {
                entities.push(EntityRef::Service(i));
                set.watch(server.request_subscriber().event_listener_fd());
            }
        }
    }
    if !clients.is_null() {
        let cls = &*clients;
        for i in 0..cls.client_count {
            let handle = *cls.clients.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ClientData);
            if let Some(client) = runtime::lock_unpoisoned(&data.client) {
                entities.push(EntityRef::Client(i));
                set.watch(client.reply_subscriber().event_listener_fd());
            }
        }
    }
}

/// The four entity arrays of one `rmw_wait` call, so the fired-only
/// helpers take one argument instead of four raw pointers each.
#[derive(Clone, Copy)]
struct Entities {
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
}

impl Entities {
    /// Array lengths (0 for a null array), in flat-index order.
    ///
    /// # Safety
    /// Same contract as [`check_ready`].
    unsafe fn counts(&self) -> (usize, usize, usize, usize) {
        let n = |p: *mut ffi::rmw_subscriptions_t| {
            if p.is_null() {
                0
            } else {
                (*p).subscriber_count
            }
        };
        let g = |p: *mut ffi::rmw_guard_conditions_t| {
            if p.is_null() {
                0
            } else {
                (*p).guard_condition_count
            }
        };
        let sv = |p: *mut ffi::rmw_services_t| if p.is_null() { 0 } else { (*p).service_count };
        let c = |p: *mut ffi::rmw_clients_t| if p.is_null() { 0 } else { (*p).client_count };
        (
            n(self.subscriptions),
            g(self.guard_conditions),
            sv(self.services),
            c(self.clients),
        )
    }
}

/// PURE: an entity's slot in the flat readiness mask (subs, then guards,
/// then services, then clients).
fn flat_index(e: EntityRef, ns: usize, ng: usize, nsv: usize) -> usize {
    match e {
        EntityRef::Sub(i) => i,
        EntityRef::Guard(i) => ns + i,
        EntityRef::Service(i) => ns + ng + i,
        EntityRef::Client(i) => ns + ng + nsv + i,
    }
}

/// Drain ONE entity's wake signal (its listener's notification queue, or
/// a guard's doorbell byte). One `drain_calls` tick per entity.
///
/// # Safety
/// Same contract as [`check_ready`]; `e` must index within its array.
unsafe fn drain_entity(
    ws: &WaitSetData,
    e: EntityRef,
    ents: Entities,
) -> Result<(), TransportError> {
    ws.drain_calls.fetch_add(1, Ordering::Relaxed);
    match e {
        EntityRef::Sub(i) => {
            let handle = *(*ents.subscriptions).subscribers.add(i);
            if handle.is_null() {
                return Ok(());
            }
            let data = &*(handle as *const SubscriptionData);
            if let Some(inner) = runtime::lock_unpoisoned(&data.inner) {
                inner.subscriber.drain_event_notifications()?;
            }
        }
        EntityRef::Guard(i) => {
            let handle = *(*ents.guard_conditions).guard_conditions.add(i);
            if handle.is_null() {
                return Ok(());
            }
            let state = &*(handle as *const GuardConditionState);
            if let Some(d) = &state.doorbell {
                d.drain();
            }
        }
        EntityRef::Service(i) => {
            let handle = *(*ents.services).services.add(i);
            if handle.is_null() {
                return Ok(());
            }
            let data = &*(handle as *const ServiceData);
            if let Some(server) = runtime::lock_unpoisoned(&data.server) {
                server.request_subscriber().drain_event_notifications()?;
            }
        }
        EntityRef::Client(i) => {
            let handle = *(*ents.clients).clients.add(i);
            if handle.is_null() {
                return Ok(());
            }
            let data = &*(handle as *const ClientData);
            if let Some(client) = runtime::lock_unpoisoned(&data.client) {
                client.reply_subscriber().drain_event_notifications()?;
            }
        }
    }
    Ok(())
}

/// Every subscription index carrying `topic` (a bell names a topic, and
/// two subscriptions may share one).
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn subs_on_topic(ents: Entities, topic: &str, out: &mut Vec<usize>) {
    if ents.subscriptions.is_null() {
        return;
    }
    let subs = &*ents.subscriptions;
    for i in 0..subs.subscriber_count {
        let handle = *subs.subscribers.add(i);
        if !handle.is_null() && (*(handle as *const SubscriptionData)).topic == topic {
            out.push(i);
        }
    }
}

/// Drain exactly the entities that woke this iteration: the fds
/// `poll(2)` reported readable (`fired`, indices into the fd snapshot)
/// plus every subscription on the bell-named `bell_topic` (a doorbell
/// wake has its listener notification pending too — drain it now rather
/// than let it re-fire the next block).
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn drain_fired_entities(
    ws: &WaitSetData,
    fired: &[usize],
    bell_topic: Option<&str>,
    ents: Entities,
    scratch: &mut Vec<usize>,
) -> Result<(), TransportError> {
    {
        let table = ws.fd_entities.lock().unwrap_or_else(|e| e.into_inner());
        for &fi in fired {
            if let Some(e) = table.get(fi) {
                drain_entity(ws, *e, ents)?;
            }
        }
    }
    if let Some(t) = bell_topic {
        scratch.clear();
        subs_on_topic(ents, t, scratch);
        let table = ws.fd_entities.lock().unwrap_or_else(|e| e.into_inner());
        for &i in scratch.iter() {
            // Skip a sub whose fd already fired (drained above).
            let already = fired
                .iter()
                .any(|&fi| table.get(fi) == Some(&EntityRef::Sub(i)));
            if !already {
                drain_entity(ws, EntityRef::Sub(i), ents)?;
            }
        }
    }
    Ok(())
}

/// SHM readiness of ONE entity (a wedged entity is never ready). Guards
/// are an atomic read and do not count as a probe.
///
/// # Safety
/// Same contract as [`check_ready`]; `e` must index within its array.
unsafe fn probe_entity(e: EntityRef, ents: Entities, probes: &AtomicU64) -> bool {
    match e {
        EntityRef::Sub(i) => {
            let handle = *(*ents.subscriptions).subscribers.add(i);
            if handle.is_null() {
                return false;
            }
            probes.fetch_add(1, Ordering::Relaxed);
            let data = &*(handle as *const SubscriptionData);
            runtime::lock_unpoisoned(&data.inner)
                .is_some_and(|inner| inner.subscriber.has_pending_sample())
        }
        EntityRef::Guard(i) => {
            let handle = *(*ents.guard_conditions).guard_conditions.add(i);
            if handle.is_null() {
                return false;
            }
            // Probe-and-CLEAR in ONE atomic op (closes the LOST_WAKE race): a
            // load here with a store(false) in the consuming pass leaves
            // a window where a trigger landing between the two is erased
            // by the blanket clear. The swap consumes exactly what this
            // probe reports (a swapped-true guard is always returned
            // ready), and a trigger landing AFTER it stays set for the
            // next call's entry probe — never consumed-and-lost.
            (*(handle as *const GuardConditionState))
                .triggered
                .swap(false, Ordering::AcqRel)
        }
        EntityRef::Service(i) => {
            let handle = *(*ents.services).services.add(i);
            if handle.is_null() {
                return false;
            }
            probes.fetch_add(1, Ordering::Relaxed);
            let data = &*(handle as *const ServiceData);
            runtime::lock_unpoisoned(&data.server)
                .is_some_and(|server| server.has_pending_request())
        }
        EntityRef::Client(i) => {
            let handle = *(*ents.clients).clients.add(i);
            if handle.is_null() {
                return false;
            }
            probes.fetch_add(1, Ordering::Relaxed);
            let data = &*(handle as *const ClientData);
            runtime::lock_unpoisoned(&data.client)
                .is_some_and(|client| client.has_pending_response())
        }
    }
}

/// What a probe pass covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProbeScope {
    /// Every attached entity (call entry; after a pump ran).
    All,
    /// Nothing (a timed-out block: nothing new can be ready).
    None,
    /// The fired fds' entities plus the bell-named subscriptions.
    Fired,
}

/// ONE readiness pass into the wait set's mask. Returns whether anything
/// is ready. The mask is what [`consume_from_mask`] nulls from, so no
/// second full pass ever runs.
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn probe_ready_into(
    ws: &WaitSetData,
    scope: ProbeScope,
    fired: &[usize],
    bell_topic: Option<&str>,
    entry: bool,
    ents: Entities,
    scratch: &mut Vec<usize>,
) -> bool {
    let probes = if entry {
        &ws.probes_entry
    } else {
        &ws.probes_wake
    };
    let (ns, ng, nsv, nc) = ents.counts();
    let mut ready = ws.ready_buf.lock().unwrap_or_else(|e| e.into_inner());
    ready.clear();
    // hot-path-alloc-ok: sized once per wait set; capacity is retained.
    ready.resize(ns + ng + nsv + nc, false);
    let mut any = false;
    let mut mark = |e: EntityRef, ready: &mut Vec<bool>| {
        let r = probe_entity(e, ents, probes);
        ready[flat_index(e, ns, ng, nsv)] = r;
        any |= r;
    };
    match scope {
        ProbeScope::None => {}
        ProbeScope::All => {
            for i in 0..ns {
                mark(EntityRef::Sub(i), &mut ready);
            }
            for i in 0..ng {
                mark(EntityRef::Guard(i), &mut ready);
            }
            for i in 0..nsv {
                mark(EntityRef::Service(i), &mut ready);
            }
            for i in 0..nc {
                mark(EntityRef::Client(i), &mut ready);
            }
        }
        ProbeScope::Fired => {
            {
                let table = ws.fd_entities.lock().unwrap_or_else(|e| e.into_inner());
                for &fi in fired {
                    if let Some(&e) = table.get(fi) {
                        mark(e, &mut ready);
                    }
                }
            }
            if let Some(t) = bell_topic {
                scratch.clear();
                subs_on_topic(ents, t, scratch);
                for &i in scratch.iter() {
                    let slot = flat_index(EntityRef::Sub(i), ns, ng, nsv);
                    if !ready[slot] {
                        mark(EntityRef::Sub(i), &mut ready);
                    }
                }
            }
        }
    }
    any
}

/// The single CONSUMING pass, from the mask: null every not-ready entry
/// (the rmw contract) and drain every ready guard's doorbell byte. The
/// guard FLAG is never touched here — it was consumed atomically AT the
/// probe (`swap(false)`), so a trigger landing in the probe→consume
/// window stays set and the next call's entry probe returns it; a
/// not-ready guard is nulled with its flag intact. Nothing is re-probed.
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn consume_from_mask(ws: &WaitSetData, ents: Entities) {
    let (ns, ng, nsv, _nc) = ents.counts();
    let ready = ws.ready_buf.lock().unwrap_or_else(|e| e.into_inner());
    let is_ready = |e: EntityRef| {
        ready
            .get(flat_index(e, ns, ng, nsv))
            .copied()
            .unwrap_or(false)
    };
    if !ents.subscriptions.is_null() {
        let subs = &mut *ents.subscriptions;
        for i in 0..subs.subscriber_count {
            if !is_ready(EntityRef::Sub(i)) {
                *subs.subscribers.add(i) = std::ptr::null_mut();
            }
        }
    }
    if !ents.guard_conditions.is_null() {
        let guards = &mut *ents.guard_conditions;
        for i in 0..guards.guard_condition_count {
            let handle = *guards.guard_conditions.add(i);
            if handle.is_null() {
                continue;
            }
            if is_ready(EntityRef::Guard(i)) {
                let state = &*(handle as *const GuardConditionState);
                // The FLAG was already consumed AT the probe (an atomic
                // swap — see `probe_entity`); only the doorbell byte is
                // drained here. Storing false here again would erase a
                // trigger that landed between probe and consume (the
                // LOST_WAKE race this ordering exists to close).
                if let Some(d) = &state.doorbell {
                    ws.drain_calls.fetch_add(1, Ordering::Relaxed);
                    d.drain();
                }
            } else {
                *guards.guard_conditions.add(i) = std::ptr::null_mut();
            }
        }
    }
    if !ents.services.is_null() {
        let svcs = &mut *ents.services;
        for i in 0..svcs.service_count {
            if !is_ready(EntityRef::Service(i)) {
                *svcs.services.add(i) = std::ptr::null_mut();
            }
        }
    }
    if !ents.clients.is_null() {
        let cls = &mut *ents.clients;
        for i in 0..cls.client_count {
            if !is_ready(EntityRef::Client(i)) {
                *cls.clients.add(i) = std::ptr::null_mut();
            }
        }
    }
}

/// Feed one drain outcome to the wait set's degrade + latch machinery
/// (shared by the legacy drain-all and the fired-only drain).
fn note_drain_outcome(ws: &WaitSetData, res: Result<(), TransportError>) {
    match res {
        Ok(()) => {
            let mut latch = lock_regime_latch(&ws.drain_failures);
            if let Some(suppressed) = latch.on_success() {
                tracing::info!(
                    suppressed_count = suppressed,
                    total_failures = latch.total_failures(),
                    "rmw_wait listener drain recovered — fd wakes resume on the \
                     next call"
                );
            }
        }
        Err(e) => {
            // A listener that cannot be drained may stay readable: blocking on
            // it would re-fire every block until the deadline (forever
            // on an infinite wait). Degrade THIS call to timeout pacing
            // — progress continues at the legacy cadence, never a spin.
            if !ws.degraded.swap(true, Ordering::Relaxed) {
                ws.degraded_waits.fetch_add(1, Ordering::Relaxed);
            }
            let mut latch = lock_regime_latch(&ws.drain_failures);
            match latch.on_failure() {
                RegimeDecision::Loud => tracing::warn!(
                    error = %e,
                    total_failures = latch.total_failures(),
                    "rmw_wait listener drain FAILED — this wait set is degraded \
                     to 100µs sleep pacing for the rest of the call (no fd \
                     block; a readable fd nobody can drain would spin)"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                    error = %e,
                    total_failures = total,
                    suppressed = suppressed,
                    "rmw_wait listener drain STILL failing — wait set degraded \
                     to sleep pacing"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    error = %e,
                    suppressed_count = suppressed,
                    "rmw_wait listener drain failed (suppressed repeat)"
                ),
            }
        }
    }
}

/// Drain-before-wait (a wake is a SIGNAL, never a count): clear every
/// residual wake — guard doorbell bytes and listener notification
/// queues — so a stale signal can never spin the block (the stale-signal
/// class: a level-readable fd nobody drains re-fires every wait).
///
/// Running this BEFORE the probe is race-free because every producer
/// commits its truth before its wake (sample commit before notify; flag
/// store before ring) — anything whose signal this drain removed is
/// already visible to the probe that follows. Notification-only: no data
/// sample and no trigger flag is consumed here.
///
/// # Errors
/// The FIRST listener whose notification queue could not be drained
/// (`try_wait_one` failed). Its fd may still be readable, so the caller
/// must stop blocking on fds for this call (`WaitSetData::degraded`) and
/// pace on the timeout instead — the drain error is surfaced from core
/// precisely so this decision can be made here rather than swallowed.
/// Guard doorbells drain infallibly (nonblocking `read` to empty).
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn drain_wake_signals(
    ws: &WaitSetData,
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
) -> Result<(), TransportError> {
    if !subscriptions.is_null() {
        let subs = &*subscriptions;
        for i in 0..subs.subscriber_count {
            let handle = *subs.subscribers.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const SubscriptionData);
            if let Some(inner) = runtime::lock_unpoisoned(&data.inner) {
                ws.drain_calls.fetch_add(1, Ordering::Relaxed);
                inner.subscriber.drain_event_notifications()?;
            }
        }
    }
    if !guard_conditions.is_null() {
        let guards = &*guard_conditions;
        for i in 0..guards.guard_condition_count {
            let handle = *guards.guard_conditions.add(i);
            if handle.is_null() {
                continue;
            }
            let state = &*(handle as *const GuardConditionState);
            if let Some(d) = &state.doorbell {
                ws.drain_calls.fetch_add(1, Ordering::Relaxed);
                d.drain();
            }
        }
    }
    if !services.is_null() {
        let svcs = &*services;
        for i in 0..svcs.service_count {
            let handle = *svcs.services.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ServiceData);
            if let Some(server) = runtime::lock_unpoisoned(&data.server) {
                ws.drain_calls.fetch_add(1, Ordering::Relaxed);
                server.request_subscriber().drain_event_notifications()?;
            }
        }
    }
    if !clients.is_null() {
        let cls = &*clients;
        for i in 0..cls.client_count {
            let handle = *cls.clients.add(i);
            if handle.is_null() {
                continue;
            }
            let data = &*(handle as *const ClientData);
            if let Some(client) = runtime::lock_unpoisoned(&data.client) {
                ws.drain_calls.fetch_add(1, Ordering::Relaxed);
                client.reply_subscriber().drain_event_notifications()?;
            }
        }
    }
    Ok(())
}

/// The spin-then-block front: busy-probe readiness for up to `budget`
/// (also bounded by the caller's `deadline`). Returns true iff readiness
/// was OBSERVED (the caller loops back to the probe head, which
/// re-arbitrates and consumes). Every probe is counted LIVE into
/// `probes` (the wait set's `spin_probes`) — an observer on another
/// thread must be able to see "the waiter is inside its spin" WHILE it
/// is, which is what the mid-wait publish/trigger pins synchronize on
/// (a count published only when the phase ends would fire the sync
/// after the once-per-call phase is already over). Record-only: the spin
/// changes WHEN we notice, never WHAT is ready.
///
/// # Safety
/// Same contract as [`check_ready`].
unsafe fn spin_for_ready(
    budget: Duration,
    deadline: Option<Instant>,
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
    probes: &AtomicU64,
) -> bool {
    let Some(spin_deadline) = Instant::now().checked_add(budget) else {
        return false; // unrepresentable budget: skip the spin, just block
    };
    loop {
        // Deadlines FIRST, probe second: a probe past the caller's
        // deadline could observe an entity that became ready after the
        // timeout and convert the call into a late OK. Stopping exactly
        // at the deadline loses nothing — a trigger racing the deadline
        // is served by exit 2's consuming pass, which is
        // the designed catcher for that window.
        let now = Instant::now();
        if now >= spin_deadline {
            return false;
        }
        if let Some(d) = deadline {
            if now >= d {
                return false; // the loop head owns the deadline exit
            }
        }
        probes.fetch_add(1, Ordering::Relaxed);
        if check_ready(subscriptions, guard_conditions, services, clients, false) {
            return true;
        }
        // YIELD between probes, never a pure `spin_loop`. The wake this
        // spin is waiting for is produced by a PEER THREAD — and when the
        // scheduler has co-located that peer on this core (CFS wake
        // affinity does exactly that to a ping-pong pair, per process
        // lifetime), a pure spin keeps the just-woken peer off the CPU for
        // the whole budget on BOTH hops: the measured stock-posture mode
        // where half the sessions sat at a tight 122 µs (= client spin +
        // server spin + ~22 µs of real path) while spin=0 sessions did
        // 30 µs. On an otherwise-idle core `sched_yield` returns in well
        // under a microsecond, so the imminent-wake catch is unchanged.
        std::thread::yield_now();
    }
}

/// # Safety
/// rmw ABI contract: arrays are valid for their counts; `wait_set`
/// comes from this implementation and is used by ONE thread at a time
/// (the rcl_wait contract — see [`WaitSetData`]).
#[no_mangle]
pub unsafe extern "C" fn rmw_wait(
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
    events: *mut ffi::rmw_events_t,
    wait_set: *mut ffi::rmw_wait_set_t,
    wait_timeout: *const ffi::rmw_time_t,
) -> rmw_ret_t {
    // Recording and replay off: one `OnceLock` load, then the live wait.
    if !crate::replay::active() {
        return rmw_wait_live(
            subscriptions,
            guard_conditions,
            services,
            clients,
            events,
            wait_set,
            wait_timeout,
        );
    }
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if wait_set.is_null()
            || !ffi::is_our_identifier((*wait_set).implementation_identifier)
            || (*wait_set).data.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        let ws = &*((*wait_set).data as *const WaitSetData);
        crate::replay::wait(
            crate::replay::WaitArrays {
                subscriptions,
                guard_conditions,
                services,
                clients,
            },
            events,
            &ws.replay_owner,
            wait_timeout,
            || {
                rmw_wait_live(
                    subscriptions,
                    guard_conditions,
                    services,
                    clients,
                    events,
                    wait_set,
                    wait_timeout,
                )
            },
        )
    })
}

/// The live `rmw_wait`: readiness from the transport, the caller's timeout.
///
/// # Safety
/// Same contract as [`rmw_wait`].
unsafe fn rmw_wait_live(
    subscriptions: *mut ffi::rmw_subscriptions_t,
    guard_conditions: *mut ffi::rmw_guard_conditions_t,
    services: *mut ffi::rmw_services_t,
    clients: *mut ffi::rmw_clients_t,
    events: *mut ffi::rmw_events_t,
    wait_set: *mut ffi::rmw_wait_set_t,
    wait_timeout: *const ffi::rmw_time_t,
) -> rmw_ret_t {
    ffi::ffi_guard(ffi::RMW_RET_ERROR, || unsafe {
        if wait_set.is_null()
            || !ffi::is_our_identifier((*wait_set).implementation_identifier)
            || (*wait_set).data.is_null()
        {
            return RMW_RET_INVALID_ARGUMENT;
        }
        // Events are not produced by this implementation; null them out so
        // rcl never sees a spurious ready event.
        if !events.is_null() {
            let evs = &mut *events;
            for i in 0..evs.event_count {
                *evs.events.add(i) = std::ptr::null_mut();
            }
        }

        let deadline = if wait_timeout.is_null() {
            None // infinite
        } else {
            // Checked time math: rcl passes {u64::MAX, …}
            // for "infinite", and `Instant + Duration` panics on overflow.
            // Saturate: anything unrepresentable IS infinite.
            let t = &*wait_timeout;
            Duration::from_secs(t.sec)
                .checked_add(Duration::from_nanos(t.nsec))
                .and_then(|d| Instant::now().checked_add(d))
        };

        // Sound: one wait set is used by one thread at a time (rcl_wait
        // contract); concurrent rmw_wait loops run on DISTINCT wait sets.
        // Shared (not `&mut`) so a concurrent diagnostic READ of the
        // atomic counters is not an aliasing violation.
        let ws = &*((*wait_set).data as *const WaitSetData);
        // 1 ns timer slack for this thread (Linux and Android; else no-op): every timed
        // block below — the ladder's ppoll rungs and the legacy sleep —
        // would otherwise pay the default 50 µs slack per wake.
        set_timer_slack_once();
        let event_wait = event_wait_enabled();
        // The spin knob is read only on the event path: the kill-switch
        // restores the legacy loop VERBATIM (probe + sleep, no spin).
        // The spin budget applies only when there is SOMETHING to observe:
        // an empty wait set can never become ready, so neither the entry
        // spin nor the post-wake recovery spin may run on it (a ZERO budget
        // gates both below).
        let attached = count_attached_entities(subscriptions, guard_conditions, services, clients);
        // …and only when the PREVIOUS call was productive (`spin_disarmed`):
        // an idle executor spins once per idle period, never per iteration.
        let spin = if event_wait && attached > 0 && !ws.spin_disarmed.load(Ordering::Relaxed) {
            spin_budget()
        } else {
            Duration::ZERO
        };
        // The ladder has no knobs — one shipped shape; the Sleep arm never
        // consults it, so the kill-switch path still touches no ladder state.
        let ladder = BLOCK_LADDER;
        // The park tier: the SHARED monitor-wait flag (auto = ON for the
        // event path), a ring that wakes this park (Linux only), and at least
        // one mapped bell (resolved after the fd/topic snapshot below).
        let (monitor_flag, park_policy) = park_horizon_from_env();
        let use_park_cfg = event_wait && RING_WAKES_THE_PARK && park_policy != ParkHorizon::NoPark;
        // The spin runs ONCE per call, on entry — data is most likely
        // imminent right after the executor finished a callback — and at
        // most ONE recovery spin after an fd wake whose probe found nothing
        // (a notify/commit race). NEVER after an empty timeout: a spin that
        // restarted every rung would burn spin/(spin+rung) of a core for
        // the whole idle life of the call (33 % at the defaults), and the
        // per-call ceiling would bound nothing.
        let mut spin_allowance: u8 = 1;
        // ONE post-wake recovery spin per call, consumed on first use and
        // NEVER re-armed: a readable listener/doorbell does not guarantee the
        // probe finds anything (a notify-without-readiness — a ring on an
        // untriggered guard, a coalesced event, a commit landing a hair
        // after the notify), and re-arming on every fd wake would let a stream
        // of such wakes re-enter the spin without bound (measured: 7 spin
        // phases for 6 rings in one wait). With this bool the spin
        // count per call is bounded at 2 regardless of how many fd wakes
        // find nothing.
        let mut recovery_spin_available = true;
        // Per-call: a drain failure this call degrades the rest of it.
        ws.degraded.store(false, Ordering::Relaxed);

        // Snapshot the wake fds ONCE per call — the entity arrays are
        // fixed for the call, and nothing below nulls an entry before the
        // final consuming pass. The entity table is recorded beside every
        // watched fd so a poll(2) report maps back to what to drain/probe.
        let ents = Entities {
            subscriptions,
            guard_conditions,
            services,
            clients,
        };
        let mut sub_topics: Vec<String> = Vec::new();
        {
            let mut fd_set = ws.fd_set.lock().unwrap_or_else(|e| e.into_inner());
            let mut table = ws.fd_entities.lock().unwrap_or_else(|e| e.into_inner());
            fd_set.clear();
            table.clear();
            if event_wait {
                collect_wake_fds(
                    &mut fd_set,
                    &mut table,
                    &mut sub_topics,
                    subscriptions,
                    guard_conditions,
                    services,
                    clients,
                );
            }
        }
        // Period-learned park (feature `period-park`): predicted windows need the bells
        // mapped too, so a call that predicts maps them even where the stock policy is NoPark.
        let predict = cfg!(feature = "period-park")
            && event_wait
            && RING_WAKES_THE_PARK
            && predicts_windows(
                monitor_flag,
                cerulion_core::monitor_wait::monitor_wait_available(),
                sub_topics.len(),
            );
        if use_park_cfg || predict {
            ws.refresh_park_bells(&sub_topics);
        }
        let parked = (use_park_cfg || predict) && ws.doorbell_topics() > 0;
        #[cfg(feature = "period-park")]
        if parked && predict {
            ws.map_learners();
        }
        // The park horizon: under `FirstRungOnce` the park is spent on the
        // call's FIRST block and the rest of the idle goes to the kernel
        // block; under `ThroughRung` every block parks.
        let mut park_allowance = parked;
        // Iteration state: what this iteration knows, and what fired.
        let mut phase = Phase::Entry;
        // hot-path-alloc-ok: two small per-call scratch Vecs (the wait
        // path, never the frame path); capacity is reused across iterations.
        let mut fired_buf: Vec<usize> = Vec::new();
        let mut scratch: Vec<usize> = Vec::new();
        let mut bell_topic: Option<String> = None;

        loop {
            // Pump THIS process's publisher events (throttled): late
            // joiners get TRANSIENT_LOCAL history even from publishers
            // that never publish again (robot_description-class topics).
            // An iteration whose pump actually RAN probes everything —
            // history delivery is data that arrives without this wait set
            // having seen a wake for it.
            let pumped = runtime::pump_publisher_events();
            let strategy = block_strategy(event_wait, ws.degraded.load(Ordering::Relaxed));
            // EXIT 1 — ready-return.
            let ready = match strategy {
                BlockStrategy::Sleep => {
                    // Kill-switch / degraded: the legacy loop VERBATIM —
                    // drain every entity, probe every entity, then the
                    // consuming pass.
                    note_drain_outcome(
                        ws,
                        drain_wake_signals(ws, subscriptions, guard_conditions, services, clients),
                    );
                    if check_ready(subscriptions, guard_conditions, services, clients, false) {
                        check_ready(subscriptions, guard_conditions, services, clients, true);
                        true
                    } else {
                        false
                    }
                }
                BlockStrategy::Fd => {
                    // Drain-before-probe, but only what FIRED. At entry
                    // nothing is known: one `poll(fds, 0)` says which fds
                    // are readable right now, and the probe is the one FULL
                    // pass the contract needs (a second queued sample has
                    // no notification left to fire an fd for). After a
                    // block: the fds it reported plus the bell-named topic.
                    // After a timed-out block: nothing can be ready — every
                    // producer commits before it notifies, so a frame
                    // committed since the entry pass has a readable fd, and
                    // one racing the revents snapshot returns the next
                    // block immediately (level-triggered).
                    fired_buf.clear();
                    match phase {
                        Phase::Entry => {
                            // Period-learned park: the full probe below serves every
                            // message whose ring came before this read.
                            #[cfg(feature = "period-park")]
                            if parked && predict {
                                ws.sync_noted_bells();
                            }
                            let mut fd_set = ws.fd_set.lock().unwrap_or_else(|e| e.into_inner());
                            let _ = fd_set.check();
                            fired_buf.extend_from_slice(fd_set.fired());
                        }
                        Phase::Woke => {
                            let fd_set = ws.fd_set.lock().unwrap_or_else(|e| e.into_inner());
                            fired_buf.extend_from_slice(fd_set.fired());
                        }
                        Phase::Timeout => {}
                    }
                    let scope = if pumped || phase == Phase::Entry {
                        ProbeScope::All
                    } else if phase == Phase::Woke {
                        ProbeScope::Fired
                    } else {
                        ProbeScope::None
                    };
                    note_drain_outcome(
                        ws,
                        drain_fired_entities(
                            ws,
                            &fired_buf,
                            bell_topic.as_deref(),
                            ents,
                            &mut scratch,
                        ),
                    );
                    if ws.degraded.load(Ordering::Relaxed) {
                        // The fired-only drain failed: the loop head
                        // re-derives the strategy (Sleep) and the legacy
                        // pass takes over without claiming readiness here.
                        continue;
                    }
                    let any = probe_ready_into(
                        ws,
                        scope,
                        &fired_buf,
                        bell_topic.as_deref(),
                        phase == Phase::Entry,
                        ents,
                        &mut scratch,
                    );
                    if any {
                        // Fault seam (`test-seams` builds only): one extra
                        // trigger lands HERE, between probe and consume —
                        // the LOST_WAKE window the atomic swap closes.
                        #[cfg(feature = "test-seams")]
                        crate::test_seams::maybe_trigger_guard_between_probe_and_consume();
                        consume_from_mask(ws, ents);
                    }
                    any
                }
            };
            if ready {
                // A ready-return is activity: the ladder goes back to its
                // first rung so the next block stays shallow. Gated on the
                // event path — the kill-switch loop is the legacy one
                // VERBATIM and must leave ladder state untouched (pinned:
                // `block_rung_us == 0` in the kill-switch test).
                if event_wait {
                    ws.ladder_reset(&ladder);
                    // A delivered call re-arms the next call's entry spin.
                    ws.spin_disarmed.store(false, Ordering::Relaxed);
                }
                return RMW_RET_OK;
            }
            // EXIT 2 — deadline (zero-timeout calls land here on their
            // FIRST iteration: one probe, no spin, no block).
            if let Some(d) = deadline {
                if Instant::now() >= d {
                    // The consuming pass can race a trigger arriving
                    // after the probe above — its result MUST be
                    // honored or the trigger is consumed and lost
                    // (a swallowed interrupt
                    // guard stalls the executor). A full consuming probe
                    // here is the rare path, kept verbatim.
                    if check_ready(subscriptions, guard_conditions, services, clients, true) {
                        // Same event-path gate as exit 1 (kill-switch purity).
                        if event_wait {
                            ws.ladder_reset(&ladder);
                            ws.spin_disarmed.store(false, Ordering::Relaxed);
                        }
                        return RMW_RET_OK;
                    }
                    if event_wait {
                        // A call that DELIVERED nothing disarms the next
                        // call's entry spin — regardless of how its blocks
                        // woke: a notification-without-readiness storm
                        // wakes blocks without delivering, and counting
                        // wakes as productivity would let it re-arm every
                        // iteration and defeat the idle bound.
                        ws.spin_disarmed.store(true, Ordering::Relaxed);
                    }
                    return RMW_RET_TIMEOUT;
                }
            }
            // EXIT 3 — the wait. One pure decision per iteration: the fd
            // path (spin first, then block) or the legacy sleep-poll.
            if strategy == BlockStrategy::Fd && spin_allowance > 0 && !spin.is_zero() {
                spin_allowance -= 1;
                ws.spin_phases.fetch_add(1, Ordering::Relaxed);
                let observed = spin_for_ready(
                    spin,
                    deadline,
                    subscriptions,
                    guard_conditions,
                    services,
                    clients,
                    &ws.spin_probes,
                );
                if observed {
                    ws.spin_wakes.fetch_add(1, Ordering::Relaxed);
                    // Observed ready in user space: the head must probe
                    // everything (the spin does not report what it saw).
                    phase = Phase::Entry;
                    bell_topic = None;
                    continue;
                }
            }
            // The block budget: the caller's remaining time, capped at the
            // pump interval, and on the fd path ALSO at the ladder's current
            // rung (first rung 200 µs: a near timer keeps the core shallow
            // so the fd wake lands in µs; doubling under proven idleness).
            let remaining = deadline
                .map(|d| d.saturating_duration_since(Instant::now()))
                .unwrap_or(BLOCK_CAP)
                .min(BLOCK_CAP);
            if remaining.is_zero() {
                continue; // deadline hit mid-spin — exit 2 handles it
            }
            match strategy {
                BlockStrategy::Fd => {
                    let budget = remaining.min(ws.ladder_rung(&ladder));
                    let (this_block_parks, next_allowance) =
                        block_parks(park_policy, park_allowance);
                    park_allowance = next_allowance;
                    // Period-learned park: a regular topic's window that opens inside
                    // this budget replaces the block (kernel lead, then a park on its bell).
                    #[cfg(feature = "period-park")]
                    let plan = if parked && predict {
                        ws.predicted_plan(budget)
                    } else {
                        None
                    };
                    #[cfg(feature = "period-park")]
                    let predicted = plan.map(|plan| ws.predicted_wait(plan));
                    #[cfg(not(feature = "period-park"))]
                    let predicted: Option<BlockWake> = None;
                    let outcome = if let Some(outcome) = predicted {
                        outcome
                    } else if this_block_parks {
                        let park_budget = match park_policy {
                            ParkHorizon::FirstRungOnce => {
                                budget.min(Duration::from_micros(PARK_HORIZON_US))
                            }
                            _ => budget,
                        };
                        ws.park_block(park_budget, 0)
                    } else {
                        ws.fd_block(budget)
                    };
                    #[cfg(feature = "period-park")]
                    let woke_at = Instant::now();
                    if this_block_parks || predicted.is_some() {
                        // A park that woke via the fd path with silent
                        // bells may be reporting a STALE bell (producer
                        // re-created) — re-map before the probe serves
                        // the frame; a doorbell wake makes this a no-op.
                        ws.reconcile_stale_bells(subscriptions);
                    }
                    bell_topic = None;
                    match outcome {
                        BlockWake::Bell(idx) => {
                            bell_topic = ws.bell_topic(idx);
                            phase = Phase::Woke;
                        }
                        BlockWake::Fd => phase = Phase::Woke,
                        BlockWake::Timeout => phase = Phase::Timeout,
                    }
                    #[cfg(feature = "period-park")]
                    if parked && predict && outcome != BlockWake::Timeout {
                        ws.note_rung_bells(woke_at);
                    }
                    if outcome != BlockWake::Timeout {
                        ws.ladder_reset(&ladder);
                        // A wake says data is imminent: if the probe finds
                        // nothing (the commit landed a hair after the
                        // notify), ONE more bounded spin is allowed — once
                        // per call, never re-armed (see the bool).
                        if recovery_spin_available {
                            recovery_spin_available = false;
                            spin_allowance = 1;
                        }
                    } else {
                        ws.ladder_empty(&ladder);
                    }
                }
                BlockStrategy::Sleep => {
                    // Kill-switch / degraded: the legacy sleep-poll cadence,
                    // verbatim (no ladder, no fd).
                    std::thread::sleep(remaining.min(POLL_INTERVAL));
                    phase = Phase::Entry;
                }
            }
        }
    })
}

// =====================================================================
// Events — not produced by a SHM transport; the init fns return
// UNSUPPORTED (rcl treats the event channel as absent, matching
// rmw_iceoryx2).
// =====================================================================

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_publisher_event_init(
    _rmw_event: *mut ffi::rmw_event_t,
    _publisher: *const ffi::rmw_publisher_t,
    _event_type: ffi::rmw_event_type_t,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_event_init(
    _rmw_event: *mut ffi::rmw_event_t,
    _subscription: *const ffi::rmw_subscription_t,
    _event_type: ffi::rmw_event_type_t,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// Jazzy+ capability probe: which QoS events this rmw can generate.
/// A SHM transport generates none (matching the UNSUPPORTED event-init
/// surface above) — rcl logs-and-degrades per event type. Without this
/// SYMBOL (not just a false return) every node spams rcutils
/// resolve errors at startup.
#[no_mangle]
pub extern "C" fn rmw_event_type_is_supported(_event_type: ffi::rmw_event_type_t) -> bool {
    false
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_take_event(
    _event_handle: *const ffi::rmw_event_t,
    _event_info: *mut std::os::raw::c_void,
    taken: *mut bool,
) -> rmw_ret_t {
    if !taken.is_null() {
        *taken = false;
    }
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_event_fini(rmw_event: *mut ffi::rmw_event_t) -> rmw_ret_t {
    if rmw_event.is_null() {
        return RMW_RET_INVALID_ARGUMENT;
    }
    std::ptr::write_bytes(rmw_event, 0, 1);
    RMW_RET_OK
}

// =====================================================================
// Listener callbacks (Humble and later)
// =====================================================================
// `rmw_event_callback_t` arrived with Humble, so these four exports do
// not exist in the rmw API of an older distro. They are compiled away
// WHOLE on a build whose headers lack the type, never stubbed: rcl looks
// symbols up by name, and a symbol an rmw is not supposed to have is a
// claim the header cannot back. The distro gate reads the built library
// with `nm` and fails if one of them is defined where the headers lack
// it.

/// # Safety
/// rmw ABI contract.
#[cfg(cerulion_has_event_callback)]
#[no_mangle]
pub unsafe extern "C" fn rmw_event_set_callback(
    _event: *mut ffi::rmw_event_t,
    _callback: ffi::rmw_event_callback_t,
    _user_data: *const std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[cfg(cerulion_has_event_callback)]
#[no_mangle]
pub unsafe extern "C" fn rmw_subscription_set_on_new_message_callback(
    _subscription: *mut ffi::rmw_subscription_t,
    _callback: ffi::rmw_event_callback_t,
    _user_data: *const std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[cfg(cerulion_has_event_callback)]
#[no_mangle]
pub unsafe extern "C" fn rmw_service_set_on_new_request_callback(
    _service: *mut ffi::rmw_service_t,
    _callback: ffi::rmw_event_callback_t,
    _user_data: *const std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

/// # Safety
/// rmw ABI contract.
#[cfg(cerulion_has_event_callback)]
#[no_mangle]
pub unsafe extern "C" fn rmw_client_set_on_new_response_callback(
    _client: *mut ffi::rmw_client_t,
    _callback: ffi::rmw_event_callback_t,
    _user_data: *const std::os::raw::c_void,
) -> rmw_ret_t {
    ffi::RMW_RET_UNSUPPORTED
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The one decision the kill-switch and the degraded fallback share:
    /// the fd path needs BOTH, and either alone lands on the legacy loop.
    #[test]
    fn block_strategy_takes_the_fd_path_only_when_enabled_and_healthy() {
        assert_eq!(block_strategy(true, false), BlockStrategy::Fd);
        assert_eq!(
            block_strategy(false, false),
            BlockStrategy::Sleep,
            "kill-switch off ⇒ legacy sleep-poll (no fd block, no spin)"
        );
        assert_eq!(
            block_strategy(true, true),
            BlockStrategy::Sleep,
            "a failed drain this call ⇒ sleep pacing, never an fd block"
        );
        assert_eq!(block_strategy(false, true), BlockStrategy::Sleep);
    }

    /// The stale-bell predicate, both sides: a frame behind a silent bell
    /// is stale; a frame behind an ADVANCED bell is the healthy race (the
    /// ring preceded the notify — the fd check simply ran between them);
    /// a silent bell with NO frame is someone else's fd wake (a guard, a
    /// service) and must not churn the mapping.
    #[test]
    fn a_bell_is_stale_only_when_a_frame_arrived_without_a_ring() {
        assert!(bell_is_stale(true, 7, 7), "frame + silent bell ⇒ stale");
        assert!(
            !bell_is_stale(true, 8, 7),
            "frame + advanced bell ⇒ healthy (the ring landed before the fd wake)"
        );
        assert!(
            !bell_is_stale(false, 7, 7),
            "no frame ⇒ a guard/service fd wake, never a re-map"
        );
        assert!(!bell_is_stale(false, 8, 7));
    }

    /// The rung schedule, hand-oracled: flat for the first N empties,
    /// then doubling per empty, and a plateau at the cap (no backoff
    /// counted). Driven on a NON-shipped `BlockLadder` shape (first rung
    /// 100 µs, threshold 3) so the schedule arithmetic is pinned apart
    /// from the shipped constants; the shipped shape itself is pinned at
    /// the end (the ladder has no knobs — a retune is a deliberate code
    /// change that must touch that assert).
    #[test]
    fn the_ladder_doubles_after_the_idleness_threshold_and_plateaus_at_the_cap() {
        let cfg = BlockLadder {
            first_rung_us: 100,
            backoff_after: 3,
        };
        assert_eq!(next_rung_after_empty(100, 1, &cfg), (100, false));
        assert_eq!(next_rung_after_empty(100, 2, &cfg), (100, false));
        assert_eq!(
            next_rung_after_empty(100, 3, &cfg),
            (200, true),
            "the N-th empty doubles"
        );
        assert_eq!(next_rung_after_empty(200, 4, &cfg), (400, true));
        assert_eq!(
            next_rung_after_empty(12_800, 10, &cfg),
            (BLOCK_CAP_US, true)
        );
        assert_eq!(
            next_rung_after_empty(BLOCK_CAP_US, 11, &cfg),
            (BLOCK_CAP_US, false),
            "the cap is a plateau — not a backoff"
        );
        // The shipped shape: internal constants, no knobs.
        assert_eq!(
            BLOCK_LADDER,
            BlockLadder {
                first_rung_us: 200,
                backoff_after: 50,
            }
        );
        assert_eq!(BLOCK_LADDER.reset_rung_us(), 200);
    }

    /// The per-block park rule, every input of it, against a HAND table
    /// written from the horizon definitions rather than from the function:
    /// no allowance never parks; `NoPark` never parks; `FirstRungOnce`
    /// parks the first block and SPENDS the allowance, so the second block
    /// of the same call takes the fd tier; `ThroughRung` parks every block
    /// and keeps the allowance. The second row of the `FirstRungOnce` pair
    /// is the one a timing test cannot state: it is why `fd_blocks` is
    /// nonzero on x86 for a call that blocked twice, and why an oracle must
    /// not read that as the park tier disengaging.
    #[test]
    fn a_calls_first_block_parks_and_only_through_rung_keeps_the_allowance() {
        use ParkHorizon::{FirstRungOnce, NoPark, ThroughRung};
        // (horizon, allowance in) -> (this block parks, allowance out)
        let cases: &[(ParkHorizon, bool, bool, bool)] = &[
            (NoPark, false, false, false),
            (NoPark, true, false, false),
            (FirstRungOnce, false, false, false),
            (FirstRungOnce, true, true, false),
            (ThroughRung, false, false, false),
            (ThroughRung, true, true, true),
        ];
        for (horizon, allowance, parks, after) in cases {
            assert_eq!(
                block_parks(*horizon, *allowance),
                (*parks, *after),
                "horizon={horizon:?} allowance={allowance}"
            );
        }
        // Walk a whole call the way `rmw_wait` does: three blocks, one
        // allowance. FirstRungOnce parks exactly the first; ThroughRung
        // parks all three.
        for (horizon, want) in [
            (FirstRungOnce, [true, false, false]),
            (ThroughRung, [true, true, true]),
            (NoPark, [false, false, false]),
        ] {
            let mut allowance = horizon != NoPark;
            let mut got = [false; 3];
            for slot in got.iter_mut() {
                let (parks, next) = block_parks(horizon, allowance);
                *slot = parks;
                allowance = next;
            }
            assert_eq!(got, want, "a three-block call under {horizon:?}");
        }
    }

    /// The platform policy table, pinned per target (the ARM-meaningful
    /// arm assertable on any desk): aarch64 Linux parks the call's first
    /// rung only (a park through every block holds a core while idle);
    /// everything else (x86 included),
    /// where the kernel wait measured faster at both periods — does not
    /// park by default. Plus the flag resolution: `0` always off; `1`
    /// forces the platform's own shape, or the bounded one-rung shape on
    /// a no-park Linux target, and nothing off Linux; auto/near-miss keep
    /// the default.
    #[test]
    fn park_policy_is_the_platform_table() {
        use cerulion_core::monitor_wait::EnvFlag;
        let expect = if cfg!(all(target_os = "linux", target_arch = "aarch64")) {
            ParkHorizon::FirstRungOnce
        } else {
            ParkHorizon::NoPark
        };
        assert_eq!(park_policy(), expect);
        for d in [
            ParkHorizon::NoPark,
            ParkHorizon::ThroughRung,
            ParkHorizon::FirstRungOnce,
        ] {
            assert_eq!(
                resolve_park(EnvFlag::ForceOff, d),
                ParkHorizon::NoPark,
                "0 ⇒ off"
            );
            assert_eq!(resolve_park(EnvFlag::Auto, d), d, "unset ⇒ the default");
            assert_eq!(
                resolve_park(EnvFlag::NearMiss, d),
                d,
                "near-miss ⇒ the default"
            );
        }
        assert_eq!(
            resolve_park(EnvFlag::ForceOn, ParkHorizon::ThroughRung),
            ParkHorizon::ThroughRung
        );
        assert_eq!(
            resolve_park(EnvFlag::ForceOn, ParkHorizon::NoPark),
            if cfg!(target_os = "linux") {
                ParkHorizon::FirstRungOnce
            } else {
                ParkHorizon::NoPark
            },
            "1 on a no-park default: the bounded one-rung hatch on Linux, nothing elsewhere"
        );
        assert_eq!(
            PARK_HORIZON_US, BLOCK_FIRST_RUNG_US,
            "the FirstRungOnce horizon IS the ladder's first rung"
        );
        assert!(
            PARK_RECHECK < Duration::from_micros(50),
            "a park slice must be short enough that a co-located peer runs \
             well inside 100 µs (two slices per same-core RTT)"
        );
    }

    #[test]
    fn a_predicted_window_maps_to_a_kernel_lead_then_a_park_to_an_absolute_end() {
        let now = Instant::now();
        let us = Duration::from_micros;
        let budget = us(1_000);
        assert_eq!(
            predicted_block(now, None, budget),
            None,
            "no window: stock block"
        );
        assert_eq!(
            predicted_block(now, Some((now + us(1_000), us(40))), budget),
            None,
            "opens at the budget"
        );
        assert_eq!(
            predicted_block(now, Some((now + us(2_000), us(40))), budget),
            None,
            "opens past the budget"
        );
        let past = now.checked_sub(us(100)).expect("clock past 100 us");
        assert_eq!(
            predicted_block(now, Some((past, us(100))), budget),
            None,
            "closed exactly now"
        );
        assert_eq!(
            predicted_block(now, Some((past, us(150))), budget),
            Some((us(0), now + us(50))),
            "open: park to the close"
        );
        assert_eq!(
            predicted_block(now, Some((now + us(300), us(40))), budget),
            Some((us(300), now + us(340))),
            "future: sleep, then park to the close"
        );
        assert_eq!(
            predicted_block(now, Some((now + us(900), us(400))), budget),
            Some((us(900), now + us(1_000))),
            "the park ends at the block's end when that comes first"
        );
        assert_eq!(
            predicted_block(now, Some((now + us(999), us(40))), budget),
            Some((us(999), now + us(1_000))),
            "an open just inside the budget keeps a park to the block's end"
        );
    }

    #[test]
    fn only_the_predicted_bells_own_ring_seen_by_the_armed_park_is_a_catch() {
        use super::super::period_learner::WindowEnd;
        let park = |wake, at_entry| Some(ParkEnd { wake, at_entry });
        assert_eq!(
            window_end(BlockWake::Fd, None, 1),
            Some(WindowEnd::BeforePark),
            "an fd cut the kernel lead"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Bell(1), false), 1),
            Some(WindowEnd::Caught),
            "the armed park saw the predicted bell ring"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Bell(1), true), 1),
            Some(WindowEnd::BeforePark),
            "the predicted bell had rung before the park's first check"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Fd, false), 1),
            None,
            "an fd end is never a catch"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Bell(0), false), 1),
            None,
            "another bell's ring is not the predicted message"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Bell(0), true), 1),
            None,
            "nor is another bell that rang before the park"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, park(BlockWake::Timeout, false), 1),
            None,
            "a park that ran to its end saw nothing"
        );
        assert_eq!(
            window_end(BlockWake::Timeout, None, 1),
            None,
            "no park and no lead wake"
        );
    }

    #[test]
    fn prediction_needs_the_flag_not_off_a_primitive_and_a_bounded_topic_set() {
        use cerulion_core::monitor_wait::EnvFlag;
        let n = PREDICT_MAX_TOPICS;
        assert!(predicts_windows(EnvFlag::Auto, true, 1));
        assert!(predicts_windows(EnvFlag::ForceOn, true, 1));
        assert!(
            predicts_windows(EnvFlag::NearMiss, true, 1),
            "a near miss keeps the default"
        );
        assert!(
            !predicts_windows(EnvFlag::ForceOff, true, 1),
            "CERULION_MONITOR_WAIT=0 turns prediction off"
        );
        assert!(
            !predicts_windows(EnvFlag::Auto, false, 1),
            "no monitor-wait primitive: no predicted park"
        );
        assert!(predicts_windows(EnvFlag::Auto, true, n));
        assert!(
            !predicts_windows(EnvFlag::Auto, true, n + 1),
            "past the topic bound: no prediction"
        );
    }

    /// The period-learned park over real doorbell pages: learners follow their topics across
    /// a bell set change, and the window names the bell of the topic that predicts it.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_predicted_window_names_the_bell_of_its_own_topic() {
        use cerulion_core::doorbell::Doorbell;
        let pid = std::process::id();
        let quiet = format!("/rmw_pp/{pid}/quiet");
        let beat = format!("/rmw_pp/{pid}/beat");
        let _quiet_page = Doorbell::open_owned(doorbell_ns(), &quiet).expect("quiet page");
        let beat_page = Doorbell::open_owned(doorbell_ns(), &beat).expect("beat page");
        let ws = WaitSetData::default();
        ws.refresh_park_bells(&[quiet.clone(), beat.clone()]);
        assert_eq!(ws.doorbell_topics(), 2);
        ws.map_learners();
        let base = Instant::now();
        let ms = Duration::from_millis;
        for k in 0..=10u64 {
            beat_page.ring();
            ws.note_rung_bells(base + ms(k));
        }
        let at = base + ms(10);
        let half = Duration::from_nanos(super::super::period_learner::MIN_WINDOW_NS);
        assert_eq!(
            ws.next_window(at),
            Some((base + ms(11) - half, 2 * half, 1)),
            "the beat topic's window, on bell 1"
        );
        ws.refresh_park_bells(&[beat.clone(), quiet.clone()]);
        assert_eq!(
            ws.next_window(at),
            None,
            "a rebuilt bell set predicts nothing until matched"
        );
        ws.map_learners();
        assert_eq!(
            ws.next_window(at).map(|w| w.2),
            Some(0),
            "the learner follows its topic to bell 0"
        );
        ws.refresh_park_bells(std::slice::from_ref(&quiet));
        ws.map_learners();
        assert_eq!(
            ws.next_window(at),
            None,
            "a learner outside the bell set never predicts"
        );
    }

    /// A wait set with one mapped bell whose learner noted a ring at `base + k ms` for k in
    /// 0..=10: a regular 1 ms topic whose next window on bell 0 is (base + 11 ms - 20 us,
    /// 40 us). Returns the wait set, the producer side of the bell, `base` and the bell's
    /// seq at the last note.
    #[cfg(feature = "period-park")]
    fn regular_one_ms_topic(
        tag: &str,
    ) -> (WaitSetData, cerulion_core::doorbell::Doorbell, Instant, u64) {
        let topic = format!("/rmw_pp/{}/{tag}", std::process::id());
        let page = cerulion_core::doorbell::Doorbell::open_owned(doorbell_ns(), &topic)
            .expect("beat page");
        let ws = WaitSetData::default();
        ws.refresh_park_bells(std::slice::from_ref(&topic));
        assert_eq!(ws.doorbell_topics(), 1);
        ws.map_learners();
        let base = Instant::now();
        for k in 0..=10u64 {
            page.ring();
            ws.note_rung_bells(base + Duration::from_millis(k));
        }
        let noted = page.seq();
        (ws, page, base, noted)
    }

    /// A readable fd in the wait set's fd snapshot, as a landed listener notify leaves it.
    #[cfg(feature = "period-park")]
    fn readable_fd(ws: &WaitSetData) -> std::os::unix::net::UnixStream {
        use std::io::Write as _;
        use std::os::fd::AsRawFd as _;
        let (rx, mut tx) = std::os::unix::net::UnixStream::pair().expect("socket pair");
        tx.write_all(&[1]).expect("one byte");
        ws.fd_set.lock().expect("fd set").watch(rx.as_raw_fd());
        // The byte stays readable while `rx` lives; the writer can go.
        rx
    }

    /// The predicted bell rings after the learners' last note and before the park, as a ring
    /// during the kernel lead does. The park measures against the noted seq, so its first
    /// check returns on the bell. Measured against its own entry seqs, it would arm the
    /// monitor wait on a line that had already moved and sleep to its end, 2 s here.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_ring_during_the_lead_ends_the_predicted_park_at_its_first_check() {
        let (ws, page, _, noted) = regular_one_ms_topic("lead_ring");
        page.ring();
        let end = Instant::now() + Duration::from_secs(2);
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, end, 0)),
            BlockWake::Bell(0)
        );
        assert!(Instant::now() < end, "returned before the park's end");
        assert_eq!(
            ws.park_yields.load(Ordering::Relaxed),
            0,
            "no park slice ran"
        );
        assert_eq!(ws.predicted_parks.load(Ordering::Relaxed), 1);
        assert_eq!(ws.predicted_bell_wakes.load(Ordering::Relaxed), 1);
        assert_eq!(ws.predicted_rings_before_park.load(Ordering::Relaxed), 1);
        let pb = ws.park_bells.lock().expect("bells");
        assert_eq!(pb.last_snap, [noted], "the reference is the learners' note");
        assert!(!pb.fd_wake_suspect);
    }

    /// A ring the call's entry pass synced (its message is served by the full probe) is not
    /// a ring for the next predicted park, which then runs to its end; the sync feeds the
    /// learner no arrival. Negative: a ring after the sync still ends the park at its first
    /// check.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_ring_synced_by_the_entry_pass_does_not_end_the_next_predicted_park() {
        let ms = Duration::from_millis;
        let us = Duration::from_micros;
        let (ws, page, base, noted) = regular_one_ms_topic("entry_sync");
        page.ring();
        ws.sync_noted_bells();
        let end = Instant::now() + ms(3);
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, end, 0)),
            BlockWake::Timeout
        );
        assert!(Instant::now() >= end, "the park ran to its end");
        assert_eq!(
            ws.park_bells.lock().expect("bells").last_snap,
            [noted + 1],
            "the reference is the synced seq"
        );
        assert_eq!(
            ws.next_window(base + ms(10)),
            Some((base + ms(11) - us(20), us(40), 0)),
            "the window of the last noted arrival at 10 ms"
        );
        page.ring();
        let end = Instant::now() + Duration::from_secs(2);
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, end, 0)),
            BlockWake::Bell(0)
        );
        assert_eq!(ws.predicted_rings_before_park.load(Ordering::Relaxed), 1);
    }

    /// The same ring with the listener notify already readable. The bell check comes before
    /// the fd check and sees the ring, so the park returns on the bell and leaves the
    /// stale-bell reconcile idle. Returned as an fd wake, the reconcile would find a pending
    /// sample with the bell at the park's entry seq and re-open a live bell.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_ring_before_the_park_with_its_notify_readable_returns_on_the_bell_not_the_fd() {
        let (ws, page, _, noted) = regular_one_ms_topic("ring_and_notify");
        let _notify = readable_fd(&ws);
        page.ring();
        let end = Instant::now() + Duration::from_secs(2);
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, end, 0)),
            BlockWake::Bell(0)
        );
        let pb = ws.park_bells.lock().expect("bells");
        assert!(
            !pb.fd_wake_suspect,
            "a bell wake does not ask for the reconcile"
        );
        assert_eq!(pb.last_snap, [noted]);
        assert!(
            !bell_is_stale(true, noted + 1, pb.last_snap[0]),
            "against the noted reference the rung bell is live"
        );
    }

    /// Negative for the two tests above: with no ring since the learners' note, the
    /// predicted park runs to its end, and a readable fd ends it as an fd wake that does arm
    /// the stale-bell reconcile.
    #[cfg(feature = "period-park")]
    #[test]
    fn with_no_ring_since_the_note_the_predicted_park_waits_for_an_fd_or_its_end() {
        let (ws, _page, _, noted) = regular_one_ms_topic("no_ring");
        let end = Instant::now() + Duration::from_millis(3);
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, end, 0)),
            BlockWake::Timeout
        );
        assert!(Instant::now() >= end, "a timeout never precedes the end");
        assert!(
            ws.park_yields.load(Ordering::Relaxed) > 0,
            "the park ran slices"
        );
        let _notify = readable_fd(&ws);
        let end = Instant::now() + Duration::from_secs(2);
        assert_eq!(ws.predicted_wait((Duration::ZERO, end, 0)), BlockWake::Fd);
        assert_eq!(ws.predicted_rings_before_park.load(Ordering::Relaxed), 0);
        assert_eq!(ws.predicted_bell_wakes.load(Ordering::Relaxed), 0);
        let pb = ws.park_bells.lock().expect("bells");
        assert!(
            pb.fd_wake_suspect,
            "a silent bell with an fd wake is suspect"
        );
        assert_eq!(pb.last_snap, [noted]);
    }

    /// The park ends at the plan's instant. A kernel lead that ends 4 ms after the park's
    /// end leaves the park its first check and no slice; a park whose length was measured
    /// from its own entry would run that length after the late lead.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_late_lead_leaves_the_predicted_park_its_first_check_and_no_later_end() {
        let (ws, page, _, _) = regular_one_ms_topic("late_lead");
        let start = Instant::now();
        let plan = (
            Duration::from_millis(5),
            start + Duration::from_millis(1),
            0,
        );
        assert_eq!(ws.predicted_wait(plan), BlockWake::Timeout);
        assert_eq!(
            ws.predicted_parks.load(Ordering::Relaxed),
            1,
            "the park was entered"
        );
        assert_eq!(
            ws.park_yields.load(Ordering::Relaxed),
            0,
            "and ran no slice past its end"
        );
        page.ring();
        let start = Instant::now();
        let plan = (
            Duration::from_millis(2),
            start + Duration::from_millis(1),
            0,
        );
        assert_eq!(
            ws.predicted_wait(plan),
            BlockWake::Bell(0),
            "a ring in a late lead is still served at the first check"
        );
        assert_eq!(ws.park_yields.load(Ordering::Relaxed), 0);
        assert_eq!(ws.predicted_rings_before_park.load(Ordering::Relaxed), 1);
    }

    /// What a predicted park tells the learner. Arrivals are noted on the hand clock
    /// base + k ms (1 ms period, 20 us half-width), so every window below is worked out by
    /// hand: a ring found at the park's first check widens the next window's early side by
    /// 20 us, an fd end leaves the widening, and a ring the armed park sees drops it.
    #[cfg(feature = "period-park")]
    #[test]
    fn a_ring_before_the_park_widens_the_next_window_and_only_a_ring_in_the_park_drops_it() {
        let ms = Duration::from_millis;
        let us = Duration::from_micros;
        let far = || Instant::now() + Duration::from_secs(2);
        let (ws, page, base, _) = regular_one_ms_topic("feedback");
        assert_eq!(
            ws.next_window(base + ms(10)),
            Some((base + ms(11) - us(20), us(40), 0))
        );
        page.ring();
        assert_eq!(
            ws.predicted_wait((Duration::ZERO, far(), 0)),
            BlockWake::Bell(0)
        );
        ws.note_rung_bells(base + ms(11));
        assert_eq!(
            ws.next_window(base + ms(11)),
            Some((base + ms(12) - us(40), us(60), 0)),
            "a ring before the park widens the early side by 20 us"
        );
        let notify = readable_fd(&ws);
        assert_eq!(ws.predicted_wait((Duration::ZERO, far(), 0)), BlockWake::Fd);
        page.ring();
        ws.note_rung_bells(base + ms(12));
        assert_eq!(
            ws.next_window(base + ms(12)),
            Some((base + ms(13) - us(40), us(60), 0)),
            "an fd end is not a catch: the widening stays"
        );
        ws.fd_set.lock().expect("fd set").clear();
        drop(notify);
        let slices_before = ws.park_yields.load(Ordering::Relaxed);
        std::thread::scope(|s| {
            s.spawn(|| {
                // Ring once the park has run a slice, so the ring lands inside the park.
                let give_up = Instant::now() + Duration::from_secs(2);
                while ws.park_yields.load(Ordering::Relaxed) == slices_before
                    && Instant::now() < give_up
                {
                    std::thread::yield_now();
                }
                page.ring();
            });
            assert_eq!(
                ws.predicted_wait((Duration::ZERO, far(), 0)),
                BlockWake::Bell(0)
            );
        });
        assert_eq!(ws.predicted_rings_before_park.load(Ordering::Relaxed), 1);
        ws.note_rung_bells(base + ms(13));
        assert_eq!(
            ws.next_window(base + ms(13)),
            Some((base + ms(14) - us(20), us(40), 0)),
            "a ring the armed park saw drops the widening"
        );
    }

    /// STRUCTURAL: the entry pass syncs the learners' seqs before its fd check and its full
    /// probe. A sync after the probe would mark a ring as seen whose message no probe looked
    /// for, and the next predicted park would wait past it. Walked over a comment-stripped
    /// view of `rmw_wait_live`.
    #[test]
    fn the_entry_pass_syncs_the_learners_before_its_full_probe() {
        let src = include_str!("guard_wait.rs");
        let start = src
            .find("unsafe fn rmw_wait_live(")
            .expect("rmw_wait_live present");
        let end = src[start..]
            .find("fn rmw_publisher_event_init(")
            .map(|e| start + e)
            .expect("rmw_publisher_event_init follows rmw_wait_live");
        let body = code_only(&src[start..end]);
        let arm = &body[body.find("Phase::Entry => {").expect("the entry arm")..];
        let sync = arm
            .find("ws.sync_noted_bells()")
            .expect("the entry arm syncs");
        let check = arm
            .find("fd_set.check()")
            .expect("the entry arm checks the fds");
        let probe = arm.find("probe_ready_into(").expect("the probe follows");
        assert!(
            sync < check && check < probe,
            "sync, then the fd check, then the probe"
        );
    }

    /// STRUCTURAL (off Linux the wait never takes the park tier, so the desk
    /// cannot measure it): the park loop's timed-out slice must reach
    /// `yield_now`, the OS-cooperative property the same-core Linux arm
    /// measures. Walked over a comment-stripped view of this file's
    /// `park_block` and `park_until`, so a yield mentioned only in prose
    /// cannot satisfy it.
    #[test]
    fn the_park_slice_yields_the_core() {
        let src = include_str!("guard_wait.rs");
        let start = src.find("fn park_block(&self").expect("park_block present");
        let end = src[start..]
            .find("fn ladder_rung(")
            .map(|e| start + e)
            .expect("ladder_rung follows park_block");
        let body = code_only(&src[start..end]);
        assert!(
            body.contains("std::thread::yield_now()"),
            "park_block must yield after a timed-out slice"
        );
        assert!(
            body.contains("park_yields.fetch_add"),
            "…and count it (park_yields)"
        );
    }

    /// Strip `//` line comments and (nesting) `/* */` block comments.
    fn code_only(src: &str) -> String {
        let mut out = String::with_capacity(src.len());
        let b = src.as_bytes();
        let mut i = 0;
        let mut depth = 0usize;
        while i < b.len() {
            if depth == 0 && b[i..].starts_with(b"//") {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
            if b[i..].starts_with(b"/*") {
                depth += 1;
                i += 2;
                continue;
            }
            if depth > 0 && b[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
                continue;
            }
            if depth == 0 {
                out.push(b[i] as char);
            }
            i += 1;
        }
        out
    }
}
