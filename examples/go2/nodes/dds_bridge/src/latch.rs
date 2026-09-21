// SPDX-License-Identifier: AGPL-3.0-only
//! A once-per-regime flood-suppression latch — the crate-local sibling of
//! `cerulion_viz::pointcloud::FieldsWarnLatch` and cerulion_core's
//! `OutputDiscardLatch` / `DrainWarnLatch`.
//!
//! Pure state machine (no logging, no clock): the CALLER maps [`FloodAction`]
//! and the recovery count to a log LEVEL, so one latch serves an `error!`-loud
//! site (per-port write failures) and a `warn!`-loud site alike. Contract: the first event of a regime is LOUD;
//! sustained repeats are DEBUG carrying a running suppressed count; a recovery
//! reports the total suppressed ONCE (only when it suppressed anything — a
//! lone-loud regime re-arms silently, so an every-other-fire flapper cannot
//! double the log volume) and re-arms, so the next event is loud again.
//!
//! The unconditional lifetime event COUNTER lives OUTSIDE the latch (an atomic
//! on the owning stats struct — Principle #3 queryability), so this latch
//! stays a minimal armed/suppressed state machine.

/// How to log one flood-latched event — the caller maps it to a level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloodAction {
    /// First event of a regime — the caller chooses the warning or error level.
    First,
    /// A sustained event — log at DEBUG carrying the running suppressed count.
    Suppressed { suppressed: u64 },
}

/// The flood-suppression state machine (see the module docs). Pure — no I/O.
///
/// CAPTURED. `suppressed` is the count a recovery line reports, and
/// `armed` decides whether the next event is loud — behaviour, even if the
/// behaviour is only a log level. Both are primitives, so the observability
/// rule's "carried, because it costs nothing" side applies.
#[derive(Debug, Clone, Copy, cerulion_core::state::CerulionState)]
pub struct FloodLatch {
    /// True when the next event should be LOUD (fresh, or re-armed by a
    /// recovery).
    armed: bool,
    /// Events DEBUG-downgraded (suppressed) in the CURRENT regime.
    suppressed: u64,
}

impl FloodLatch {
    /// A fresh, armed latch (the next event is loud).
    pub const fn new() -> Self {
        Self {
            armed: true,
            suppressed: 0,
        }
    }

    /// Record one event; returns how to log it.
    pub fn on_event(&mut self) -> FloodAction {
        if self.armed {
            self.armed = false;
            self.suppressed = 0;
            FloodAction::First
        } else {
            self.suppressed += 1;
            FloodAction::Suppressed {
                suppressed: self.suppressed,
            }
        }
    }

    /// Record a recovery (a successful, non-erroring event). Returns
    /// `Some(suppressed_count)` when a suppressed regime just healed (the
    /// caller logs ONE recovery `info!`), `None` otherwise (already armed, or
    /// a lone-loud regime with nothing suppressed — silent re-arm).
    pub fn on_recovered(&mut self) -> Option<u64> {
        if self.armed {
            return None;
        }
        self.armed = true;
        let suppressed = self.suppressed;
        self.suppressed = 0;
        (suppressed > 0).then_some(suppressed)
    }
}

impl Default for FloodLatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The canonical loud → debug(n) → recovery(total) → loud cycle, against
    /// hand-written expected actions (NOT a self-compare).
    #[test]
    fn canonical_cycle() {
        let mut l = FloodLatch::new();
        // Regime 1: first event is LOUD, the next two are DEBUG 1, 2.
        assert_eq!(l.on_event(), FloodAction::First);
        assert_eq!(l.on_event(), FloodAction::Suppressed { suppressed: 1 });
        assert_eq!(l.on_event(), FloodAction::Suppressed { suppressed: 2 });
        // Recovery reports the 2 suppressed, exactly once.
        assert_eq!(l.on_recovered(), Some(2));
        assert_eq!(l.on_recovered(), None, "already armed — no double recovery");
        // Regime 2: re-armed → LOUD again, suppressed counter reset.
        assert_eq!(l.on_event(), FloodAction::First);
        assert_eq!(l.on_event(), FloodAction::Suppressed { suppressed: 1 });
    }

    /// A lone-loud regime (zero suppressed) re-arms SILENTLY — no recovery
    /// value, so an every-other-fire flapper cannot double the log volume.
    #[test]
    fn lone_loud_regime_rearms_silently() {
        let mut l = FloodLatch::new();
        assert_eq!(l.on_event(), FloodAction::First);
        assert_eq!(l.on_recovered(), None, "nothing suppressed — silent re-arm");
        assert_eq!(l.on_event(), FloodAction::First);
        assert_eq!(l.on_recovered(), None);
    }

    /// A long sustained regime: exactly one loud, N-1 DEBUGs with exact
    /// running counts, then a recovery reporting all N-1.
    #[test]
    fn sustained_regime_suppresses_the_flood() {
        let mut l = FloodLatch::new();
        assert_eq!(l.on_event(), FloodAction::First);
        for i in 1..100u64 {
            assert_eq!(l.on_event(), FloodAction::Suppressed { suppressed: i });
        }
        assert_eq!(l.on_recovered(), Some(99));
    }

    /// A recovery on a fresh (never-fired) latch is a no-op — never a phantom
    /// recovery on an armed latch.
    #[test]
    fn recovery_on_fresh_latch_is_noop() {
        let mut l = FloodLatch::new();
        assert_eq!(l.on_recovered(), None);
        assert_eq!(
            l.on_event(),
            FloodAction::First,
            "still fresh after a no-op recovery"
        );
    }

    #[test]
    fn default_is_armed() {
        let mut l = FloodLatch::default();
        assert_eq!(l.on_event(), FloodAction::First);
    }
}
