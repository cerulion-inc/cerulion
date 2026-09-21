// SPDX-License-Identifier: AGPL-3.0-only
//! The netd mirror HEALTH state machine — the pure decision layer for
//! the loud, self-healing observability watch over each shared mirror's stream.
//!
//! The failure mode: a desk `cerulion-netd` mirror keeps a robot's topic
//! streaming, but when the robot's `graph run` (and thus its gateway) restarts,
//! the netd's persistent zenoh session never re-affirmed demand across the fresh
//! link — every consumer went a SILENT BLACK HOLE until the netd happened to die.
//! Killing netd healed it (a fresh session re-dialed the current gateway).
//!
//! The self-heal ACTUATION is the demand-GET keepalive (wired for the
//! pure-ingress path by `TransportManager::ensure_ingress_demand_keepalive`): a
//! dialer→accepter query that re-crosses a reconnected link and re-affirms egress
//! automatically. THIS module is the OBSERVABILITY + prompt-belt half: a per-mirror
//! staleness state machine over the mirror's monotonic re-injected frame count, so
//! a degraded mirror is LOUD (never a silent black hole) and the belt re-affirms
//! demand PROMPTLY rather than waiting for the next background keepalive pass.
//!
//! Pure + oracle-tested here; the thread that drives it against real
//! `TransportManager::ingress_stats` frame counts + the `run_ingress_demand_get_pass`
//! belt lives in [`crate::mirror`] (where the manager is), and the loopback
//! restart e2e is `cerulion_netd/tests/mirror_restart_e2e_test.rs`.

use std::time::{Duration, Instant};

/// Production default: how often the health watch samples each mirror's frame
/// count. A coarse observability cadence, not a hot path.
pub const DEFAULT_HEALTH_POLL: Duration = Duration::from_secs(1);

/// Production default: how long a mirror may go with NO new frames before it is
/// declared DEGRADED (the robot's gateway is likely down / restarting).
///
/// WHY 5 s (generous): a control topic streaming at hundreds of Hz makes 5 s of
/// silence unambiguous. A genuinely idle / latched / low-rate topic (a
/// transient-local `/tf_static` that publishes once, an event topic like
/// `/cmd_vel` routinely quiet > 5 s) false-positives into DEGRADED. The
/// consequences are bounded, NOT a silent black hole:
///
/// - the loud `warn!` is EDGE-triggered (once per Healthy→Degraded regime, then
///   `debug!`), and for a topic that FLAPS (recovers then re-degrades within
///   [`HealthConfig::flap_window`]) the repeat warns downgrade to `debug!` so a
///   healthy-but-bursty topic does not emit recurring loud noise (finding C);
/// - the demand-GET re-affirm belt is EDGE-triggered with bounded backoff (a
///   [`BeltSchedule`], NOT one pass per poll — finding B): a permanently-idle
///   latched mirror fires a SMALL bounded burst of passes once, then goes quiet
///   and relies on the always-on background keepalive loop for sustained
///   re-affirmation. It is NOT "a singular idempotent GET"; it is a bounded burst.
///
/// A shorter `degrade_after` would reduce restart-detection latency at the cost of
/// more false positives; 5 s is the balance for the mixed sensor/event topic set.
pub const DEFAULT_DEGRADE_AFTER: Duration = Duration::from_secs(5);

/// Production default flap window: a topic that recovers then re-degrades within
/// this window of its last recovery is FLAPPING (a healthy low-rate / bursty
/// publisher whose inter-frame gap exceeds [`DEFAULT_DEGRADE_AFTER`]); its repeat
/// degrade/recover logs downgrade to `debug!` (first signal stays
/// loud, sustained flap goes quiet). `2 × DEFAULT_DEGRADE_AFTER`.
pub const DEFAULT_FLAP_WINDOW: Duration = Duration::from_secs(10);

/// Tunable timings for the health watch (production defaults; tests use short
/// timings so a restart e2e is fast).
#[derive(Debug, Clone, Copy)]
pub struct HealthConfig {
    /// Sample interval.
    pub poll: Duration,
    /// No-new-frames window before DEGRADED.
    pub degrade_after: Duration,
    /// A re-degrade within this window of the last recovery is a FLAP (its repeat
    /// warn is dampened to `debug!`).
    pub flap_window: Duration,
}

impl Default for HealthConfig {
    fn default() -> Self {
        Self {
            poll: DEFAULT_HEALTH_POLL,
            degrade_after: DEFAULT_DEGRADE_AFTER,
            flap_window: DEFAULT_FLAP_WINDOW,
        }
    }
}

/// A mirror's coarse health, derived PURELY from whether its re-injected frame
/// count has advanced recently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthState {
    /// Frames advanced within the degrade window — the robot is streaming.
    Healthy,
    /// No new frames for the degrade window — the robot's gateway is likely
    /// down / restarting; demand is being re-affirmed.
    Degraded,
}

/// The transition a fresh sample produced — the caller maps this to a log level
/// and (for the degraded arms) the demand-reaffirm belt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthTransition {
    /// Healthy and still streaming (or not yet stale) — nothing to do / say.
    None,
    /// Healthy → Degraded EDGE: frames stopped for the degrade window. The caller
    /// emits ONE loud `warn!` and re-affirms demand.
    Degraded,
    /// Still Degraded on a later sample — the caller re-affirms demand (the belt)
    /// and logs at `debug!` (no flood while a robot is down).
    StillDegraded,
    /// Degraded → Healthy EDGE: frames resumed. The caller emits ONE loud `info!`
    /// ("mirror recovered") — the self-heal was INVISIBLE to consumers.
    Recovered,
}

/// The pure per-mirror staleness state machine. Fed a monotonic frame count at
/// each poll; emits the [`HealthTransition`] the watch acts on. Deterministic +
/// oracle-tested — no clock, no transport (the caller supplies `now`).
#[derive(Debug, Clone)]
pub struct MirrorHealth {
    state: HealthState,
    /// The last frame count observed (the mirror's monotonic re-inject counter).
    last_frames: u64,
    /// The last time the count ADVANCED (the freshness anchor).
    last_progress_at: Instant,
    degrade_after: Duration,
    /// The last time this mirror RECOVERED (Degraded→Healthy). Used to classify a
    /// fresh degrade as a FLAP (finding C): a re-degrade close after a recovery is
    /// a healthy low-rate/bursty publisher, not a gone robot.
    last_recovered_at: Option<Instant>,
}

impl MirrorHealth {
    /// A fresh mirror observed at `now` with `frames` already re-injected (usually
    /// 0 at creation). Starts [`HealthState::Healthy`] with `now` as the freshness
    /// anchor, so a mirror that NEVER delivers its first frame still surfaces as
    /// DEGRADED after `degrade_after` (a mirror that never streams is also a
    /// problem worth the loud signal + a demand re-affirm).
    pub fn new(frames: u64, now: Instant, degrade_after: Duration) -> Self {
        Self {
            state: HealthState::Healthy,
            last_frames: frames,
            last_progress_at: now,
            degrade_after,
            last_recovered_at: None,
        }
    }

    /// Feed a fresh frame-count sample at `now`; return the transition to act on.
    ///
    /// - Count ADVANCED → refresh the anchor; if we were Degraded, that is the
    ///   RECOVERED edge (frames resumed), else `None`.
    /// - Count UNCHANGED (a monotonic counter never decreases; a spurious decrease
    ///   is treated as unchanged too) → if Healthy and stale past `degrade_after`,
    ///   that is the DEGRADED edge; if already Degraded, `StillDegraded`; else
    ///   `None`.
    pub fn observe(&mut self, frames: u64, now: Instant) -> HealthTransition {
        if frames > self.last_frames {
            self.last_frames = frames;
            self.last_progress_at = now;
            return match self.state {
                HealthState::Degraded => {
                    self.state = HealthState::Healthy;
                    self.last_recovered_at = Some(now);
                    HealthTransition::Recovered
                }
                HealthState::Healthy => HealthTransition::None,
            };
        }
        // No progress. `saturating_duration_since` guards a non-monotonic `now`.
        let stalled_for = now.saturating_duration_since(self.last_progress_at);
        match self.state {
            HealthState::Healthy => {
                if stalled_for >= self.degrade_after {
                    self.state = HealthState::Degraded;
                    HealthTransition::Degraded
                } else {
                    HealthTransition::None
                }
            }
            HealthState::Degraded => HealthTransition::StillDegraded,
        }
    }

    /// The current health (Principle #3 / tests).
    pub fn state(&self) -> HealthState {
        self.state
    }

    /// How long since frames last advanced, as of `now` (for the loud log's
    /// `stalled_ms` field). Saturating (guards a non-monotonic clock).
    pub fn stalled_for(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last_progress_at)
    }

    /// Whether a degrade at `now` is a FLAP — this mirror
    /// RECOVERED within `flap_window` before now (a healthy low-rate / bursty
    /// publisher whose gap merely exceeds `degrade_after`), so its loud warn should
    /// be dampened to `debug!`. `false` on the FIRST degrade (never recovered yet),
    /// keeping the first signal loud. Saturating (non-monotonic-clock safe).
    pub fn degrade_is_flap(&self, now: Instant, flap_window: Duration) -> bool {
        match self.last_recovered_at {
            Some(recovered) => now.saturating_duration_since(recovered) < flap_window,
            None => false,
        }
    }
}

/// The EDGE-TRIGGERED, bounded-backoff schedule for the
/// health watch's prompt demand-reaffirm belt. Replaces the level-triggered
/// "one pass per poll while any mirror is degraded" (which stormed a full
/// demand-GET pass every poll FOREVER for a by-design-quiet latched mirror).
///
/// The belt exists ONLY to make a restart's recovery PROMPT; the always-on 2 s
/// background keepalive loop is the SUSTAINED re-affirm (it refreshes the
/// producer's demand TTL on strict links). So the belt fires a SMALL bounded burst
/// on each fresh Degraded EDGE at growing intervals, then goes quiet — the repo's
/// once-per-regime pattern applied to WORK, not just logs.
#[derive(Debug, Clone)]
pub struct BeltSchedule {
    /// The next instant a belt pass is due, when armed.
    next_at: Instant,
    /// Passes fired in the current armed burst.
    attempts: u32,
    /// Armed = still within a burst (attempts < max). A fresh edge re-arms.
    armed: bool,
    config: BeltConfig,
}

/// Tunable bounds for [`BeltSchedule`].
#[derive(Debug, Clone, Copy)]
pub struct BeltConfig {
    /// Max belt passes per Degraded burst (then hand off to the background loop).
    pub max_attempts: u32,
    /// The first inter-pass gap (the edge pass itself fires immediately).
    pub initial_backoff: Duration,
    /// The backoff cap (doubling each attempt, clamped here).
    pub max_backoff: Duration,
}

impl Default for BeltConfig {
    fn default() -> Self {
        // Edge, then +1s, +2s, +4s → 4 passes over ~7s, bridging to the 2s
        // background keepalive; then quiet.
        Self {
            max_attempts: 4,
            initial_backoff: Duration::from_secs(1),
            max_backoff: Duration::from_secs(4),
        }
    }
}

impl BeltSchedule {
    /// A disarmed schedule (no belt due). `now` seeds `next_at` (never read while
    /// disarmed).
    pub fn new(now: Instant, config: BeltConfig) -> Self {
        Self {
            next_at: now,
            attempts: 0,
            armed: false,
            config,
        }
    }

    /// Call once per health poll. `new_edge` = at least one mirror crossed the
    /// Healthy→Degraded EDGE this poll (NOT merely still-degraded). Returns whether
    /// a belt pass should run NOW.
    ///
    /// A fresh edge RE-ARMS the burst (resets attempts, fires immediately) — a real
    /// new degrade (e.g. a second robot going down, or a restart) deserves a prompt
    /// re-affirm. Between edges, the armed burst fires at growing backoff intervals
    /// up to `max_attempts`, then disarms (the background loop sustains).
    pub fn poll(&mut self, new_edge: bool, now: Instant) -> bool {
        if new_edge {
            self.armed = true;
            self.attempts = 0;
            self.next_at = now;
        }
        if self.armed && self.attempts < self.config.max_attempts && now >= self.next_at {
            self.attempts += 1;
            // backoff = initial << (attempts-1), clamped to max_backoff.
            let shift = (self.attempts - 1).min(16);
            let backoff = self
                .config
                .initial_backoff
                .saturating_mul(1u32 << shift)
                .min(self.config.max_backoff);
            self.next_at = now + backoff;
            if self.attempts >= self.config.max_attempts {
                self.armed = false; // burst exhausted — hand off to the background loop.
            }
            return true;
        }
        false
    }

    /// Whether a burst is currently armed (tests / Principle #3).
    pub fn is_armed(&self) -> bool {
        self.armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEGRADE: Duration = Duration::from_secs(5);

    /// A base instant + offset helper so the oracle is deterministic (the machine
    /// only ever compares instants via `saturating_duration_since`).
    fn at(base: Instant, secs: u64) -> Instant {
        base + Duration::from_secs(secs)
    }

    #[test]
    fn steady_streaming_stays_healthy_and_silent() {
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        // Frames advance every poll → Healthy, no transitions.
        for (i, secs) in (1..=10u64).zip(1..) {
            assert_eq!(
                h.observe(i, at(t0, secs)),
                HealthTransition::None,
                "advancing frames never transitions"
            );
            assert_eq!(h.state(), HealthState::Healthy);
        }
    }

    #[test]
    fn silence_past_the_window_degrades_exactly_once_then_still_degraded() {
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(7, t0, DEGRADE);
        // Same count, still inside the window → no transition.
        assert_eq!(h.observe(7, at(t0, 4)), HealthTransition::None);
        assert_eq!(h.state(), HealthState::Healthy);
        // Crosses the 5 s window → the DEGRADED edge, exactly once.
        assert_eq!(h.observe(7, at(t0, 5)), HealthTransition::Degraded);
        assert_eq!(h.state(), HealthState::Degraded);
        // Further silence → StillDegraded (belt keeps re-affirming; log at debug).
        assert_eq!(h.observe(7, at(t0, 6)), HealthTransition::StillDegraded);
        assert_eq!(h.observe(7, at(t0, 30)), HealthTransition::StillDegraded);
        assert_eq!(h.state(), HealthState::Degraded);
    }

    #[test]
    fn resumed_frames_recover_exactly_once_then_healthy_silent() {
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        assert_eq!(h.observe(0, at(t0, 5)), HealthTransition::Degraded);
        // A single new frame (the restarted robot returns) → RECOVERED edge.
        assert_eq!(h.observe(1, at(t0, 6)), HealthTransition::Recovered);
        assert_eq!(h.state(), HealthState::Healthy);
        // Continued streaming stays silent-healthy.
        assert_eq!(h.observe(2, at(t0, 7)), HealthTransition::None);
    }

    #[test]
    fn full_degrade_recover_cycle_repeats() {
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        // Stream, then die, degrade, revive, recover — twice.
        assert_eq!(h.observe(5, at(t0, 1)), HealthTransition::None);
        assert_eq!(h.observe(5, at(t0, 7)), HealthTransition::Degraded);
        assert_eq!(h.observe(6, at(t0, 8)), HealthTransition::Recovered);
        assert_eq!(h.observe(6, at(t0, 14)), HealthTransition::Degraded);
        assert_eq!(h.observe(9, at(t0, 15)), HealthTransition::Recovered);
        assert_eq!(h.state(), HealthState::Healthy);
    }

    #[test]
    fn never_first_frame_still_degrades() {
        // A mirror whose initial demand never crossed (no first frame ever) is a
        // problem worth surfacing — it degrades on the same window.
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        assert_eq!(h.observe(0, at(t0, 4)), HealthTransition::None);
        assert_eq!(h.observe(0, at(t0, 5)), HealthTransition::Degraded);
    }

    #[test]
    fn stalled_for_reports_since_last_progress() {
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        h.observe(3, at(t0, 2)); // progress at +2s
        assert_eq!(h.stalled_for(at(t0, 2)), Duration::ZERO);
        assert_eq!(h.stalled_for(at(t0, 9)), Duration::from_secs(7));
    }

    #[test]
    fn non_monotonic_now_is_saturating_never_panics() {
        // A clock that jumps BACKWARD must not underflow; treated as zero elapsed.
        let t0 = Instant::now();
        let later = at(t0, 10);
        let mut h = MirrorHealth::new(0, later, DEGRADE);
        // Observe at an EARLIER instant than the anchor → saturating zero, no panic.
        assert_eq!(h.observe(0, t0), HealthTransition::None);
        assert_eq!(h.stalled_for(t0), Duration::ZERO);
    }

    const FLAP: Duration = Duration::from_secs(10);

    #[test]
    fn first_degrade_is_not_a_flap_but_a_close_re_degrade_is() {
        // The FIRST degrade is loud (never recovered); a re-degrade close
        // after a recovery is a FLAP (dampened); a re-degrade LONG after a recovery
        // is loud again (a genuinely-gone robot after a healthy run).
        let t0 = Instant::now();
        let mut h = MirrorHealth::new(0, t0, DEGRADE);
        assert_eq!(h.observe(0, at(t0, 5)), HealthTransition::Degraded);
        assert!(
            !h.degrade_is_flap(at(t0, 5), FLAP),
            "the FIRST degrade is never a flap (loud)"
        );
        // Recover at +6s.
        assert_eq!(h.observe(1, at(t0, 6)), HealthTransition::Recovered);
        // Re-degrade at +11s (5s after the +6s recovery) → within the 10s flap
        // window → a FLAP (dampened).
        assert_eq!(h.observe(1, at(t0, 11)), HealthTransition::Degraded);
        assert!(
            h.degrade_is_flap(at(t0, 11), FLAP),
            "a re-degrade 5s after recovery is a flap (dampened)"
        );
        // Recover again, then stream healthily, then degrade LONG after (> window).
        assert_eq!(h.observe(2, at(t0, 12)), HealthTransition::Recovered);
        assert_eq!(h.observe(3, at(t0, 20)), HealthTransition::None); // healthy stream
        assert_eq!(h.observe(3, at(t0, 30)), HealthTransition::Degraded);
        assert!(
            !h.degrade_is_flap(at(t0, 30), FLAP),
            "a degrade 18s after the last recovery is NOT a flap — loud again"
        );
    }

    /// A base BeltConfig for the oracle: edge, then +1s, +2s, +4s (cap 4s), max 4.
    fn belt_cfg() -> BeltConfig {
        BeltConfig::default()
    }

    #[test]
    fn belt_fires_edge_then_backoff_not_once_per_poll() {
        // N degraded polls → EXACTLY the edge-triggered + backoff pass
        // count (4), NOT N. Poll every 0.5s for 20s; the edge is only at t0.
        let t0 = Instant::now();
        let mut belt = BeltSchedule::new(t0, belt_cfg());
        let mut fires: Vec<u64> = Vec::new();
        // 40 polls at 0.5s spacing (0.0 .. 19.5s). Edge only on the first.
        for i in 0..40u64 {
            let now = t0 + Duration::from_millis(500 * i);
            let new_edge = i == 0;
            if belt.poll(new_edge, now) {
                fires.push(i); // record the poll index that fired
            }
        }
        // Fires at the edge (poll 0 = t0), then +1s (poll 2), +3s (poll 6), +7s
        // (poll 14) — exactly 4 passes, then disarmed and SILENT for the rest.
        assert_eq!(
            fires,
            vec![0, 2, 6, 14],
            "exactly 4 edge+backoff passes: {fires:?}"
        );
        assert!(
            !belt.is_armed(),
            "the burst is exhausted → disarmed (background loop sustains)"
        );
    }

    #[test]
    fn belt_permanently_idle_mirror_fires_a_bounded_burst_then_stops() {
        // The /tf_static case: one Degraded edge, never recovers → a bounded burst,
        // then FOREVER quiet (no per-poll storm).
        let t0 = Instant::now();
        let mut belt = BeltSchedule::new(t0, belt_cfg());
        let mut count = 0;
        for i in 0..1000u64 {
            let now = t0 + Duration::from_millis(500 * i);
            let new_edge = i == 0; // degraded once, never again
            if belt.poll(new_edge, now) {
                count += 1;
            }
        }
        assert_eq!(
            count, 4,
            "a permanently-idle mirror fires EXACTLY the bounded burst, never a per-poll storm"
        );
    }

    #[test]
    fn belt_a_fresh_edge_after_a_burst_re_arms() {
        // A second robot going down (a fresh edge) after the first burst exhausted
        // re-arms a prompt burst.
        let t0 = Instant::now();
        let mut belt = BeltSchedule::new(t0, belt_cfg());
        // Drive the first burst to exhaustion.
        for i in 0..40u64 {
            let now = t0 + Duration::from_millis(500 * i);
            belt.poll(i == 0, now);
        }
        assert!(!belt.is_armed());
        // A fresh edge at +30s → fires immediately + re-arms.
        assert!(
            belt.poll(true, at(t0, 30)),
            "a fresh degraded edge re-fires promptly"
        );
        assert!(belt.is_armed(), "and re-arms the burst");
    }

    #[test]
    fn belt_disarmed_never_fires_without_an_edge() {
        // A never-degraded process: the belt never fires (healthy steady state).
        let t0 = Instant::now();
        let mut belt = BeltSchedule::new(t0, belt_cfg());
        for i in 0..50u64 {
            assert!(
                !belt.poll(false, t0 + Duration::from_millis(300 * i)),
                "no edge ever ⇒ the belt never fires"
            );
        }
    }
}
