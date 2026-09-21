// SPDX-License-Identifier: AGPL-3.0-only
//! The fault-injection seam for the automatic capture PRODUCERS.
//!
//! # Why this exists
//!
//! Every automatic Flashback producer — the node-death mint in
//! [`GraphRuntime`](crate::graph::GraphRuntime) and the process-fault mint in
//! the CLI supervisor — has the same three failure branches: this process has
//! no usable transport, the trigger channel will not open, the request will not
//! publish. All six log at `warn!`, not `debug!`, because a
//! robot binary enables `tracing/release_max_level_info` and a `debug!` there
//! does not exist at ANY `RUST_LOG`: a node or a worker died, no capture was
//! made for it, and the operator had no evidence of the second fact.
//!
//! That flip shipped with NO behavioural pin, which is the class
//! — an unpinned level is free to regress, and the regression is invisible
//! exactly where it costs the most. Pinning it needs the branches to be
//! REACHABLE, and none of them is: a test process always resolves a transport,
//! an isolated iceoryx2 namespace always opens, and a publish onto a live
//! service always succeeds. This module is what makes them reachable.
//!
//! # Thread-local, and STICKY
//!
//! THREAD-LOCAL, following [`crate::spill_fault_injection`]: each libtest
//! worker owns its own thread, so two tests arming different faults never see
//! each other and no test needs `#[serial]` for the switch's sake.
//!
//! STICKY rather than fire-once, which is the one place this departs from the
//! spill hook. The publish branch reports PER REQUEST — that per-request
//! reporting is itself a correction, since a batch used to
//! report only its last entry — so a fire-once fault would fail request 0,
//! silently succeed for 1..n, and pin the opposite of the contract. An RAII
//! [`FaultGuard`] disarms on drop, so a panicking test cannot leak the fault
//! into whatever runs next on its thread.
//!
//! # Always-on module, gated CALL SITES
//!
//! The module compiles in every build (like [`crate::spill_fault_injection`])
//! while every call site is `#[cfg(any(test, feature = …))]`. Two reasons, and
//! the second is the load-bearing one:
//!
//! * a shipping robot pays nothing — the branch it takes is byte-identical to
//!   the earlier one, because the check is not compiled in at all;
//! * one consumer is `cerulion_cli_engine`, whose `test-seams` feature is
//!   independent of this crate's `test-helpers`. A feature-gated MODULE would
//!   make the CLI's gated call site depend on a feature it does not control,
//!   which is the feature-unification hazard this repo has been bitten by
//!   before. An always-on module has no such coupling.
//!
//! Every item is `#[doc(hidden)]`: callable from a test crate that enables
//! neither feature, absent from the public API docs.

use std::cell::Cell;

use crate::error::TransportError;

/// Which producer failure branch the calling thread is currently forcing.
///
/// A closed enum rather than three booleans: the branches are mutually
/// exclusive by construction (a channel that will not open never reaches a
/// publish), and a test that armed two of them would be asserting on a shape
/// production cannot produce.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlashbackFault {
    /// Nothing injected — the production path runs unchanged.
    #[default]
    None,
    /// This process has no usable transport for the trigger channel.
    ///
    /// The runtime's `resolve_doorbell_transport` resolves to `None`, and the
    /// supervisor's `TransportManager::get_or_init()` fails.
    NoTransport,
    /// The transport resolves, but the trigger channel will not open.
    ChannelOpen,
    /// The channel opens, but every request published on it fails.
    Publish,
}

thread_local! {
    /// The fault this thread is forcing. Sticky until disarmed.
    static FAULT: Cell<FlashbackFault> = const { Cell::new(FlashbackFault::None) };
}

/// Arm `fault` on the CALLING thread until the returned guard drops.
///
/// The guard restores the PREVIOUS value rather than clearing, so nesting is
/// well defined and an inner scope cannot silently disarm an outer one.
#[doc(hidden)]
#[must_use = "the fault is disarmed as soon as the guard drops"]
pub fn arm(fault: FlashbackFault) -> FaultGuard {
    let previous = FAULT.with(Cell::get);
    FAULT.with(|cell| cell.set(fault));
    FaultGuard { previous }
}

/// Restores the fault a thread was forcing before [`arm`] was called.
///
/// RAII rather than an explicit `disarm()`, because a test that panics between
/// arm and disarm would otherwise leave the fault armed for every later test on
/// that libtest worker thread — a cross-test contamination whose symptom (a
/// warn appearing in an unrelated healthy-path arm) points nowhere near its
/// cause.
#[doc(hidden)]
#[derive(Debug)]
pub struct FaultGuard {
    previous: FlashbackFault,
}

impl Drop for FaultGuard {
    fn drop(&mut self) {
        let previous = self.previous;
        FAULT.with(|cell| cell.set(previous));
    }
}

/// The fault this thread is currently forcing.
#[doc(hidden)]
pub fn armed() -> FlashbackFault {
    FAULT.with(Cell::get)
}

/// `None` when [`FlashbackFault::NoTransport`] is armed, otherwise `resolved`.
///
/// Written as a FILTER over the production resolver's own answer rather than as
/// a `bool` the caller branches on, so the injected path and the real one reach
/// the identical `let … else` and log the identical line. A `bool` would let a
/// call site drift into a second, test-only refusal arm that production never
/// takes — which is the shape this whole module exists to avoid.
#[doc(hidden)]
pub fn filter_transport<T>(resolved: Option<T>) -> Option<T> {
    match armed() {
        FlashbackFault::NoTransport => None,
        _ => resolved,
    }
}

/// The error a transport lookup should return when
/// [`FlashbackFault::NoTransport`] is armed.
///
/// The `Result`-shaped twin of [`filter_transport`], for the supervisor, whose
/// transport lookup (`TransportManager::get_or_init`) is fallible rather than
/// optional.
#[doc(hidden)]
pub fn transport_error() -> Option<TransportError> {
    match armed() {
        FlashbackFault::NoTransport => Some(TransportError::Internal {
            reason: "fault injection: no transport for the flashback trigger".to_string(),
        }),
        _ => None,
    }
}

/// The error a channel open should return when [`FlashbackFault::ChannelOpen`]
/// is armed.
#[doc(hidden)]
pub fn channel_open_error() -> Option<TransportError> {
    match armed() {
        FlashbackFault::ChannelOpen => Some(TransportError::Internal {
            reason: "fault injection: the flashback trigger channel will not open".to_string(),
        }),
        _ => None,
    }
}

/// The error a request publish should return when [`FlashbackFault::Publish`]
/// is armed.
#[doc(hidden)]
pub fn publish_error() -> Option<TransportError> {
    match armed() {
        FlashbackFault::Publish => Some(TransportError::Internal {
            reason: "fault injection: the flashback trigger request will not publish".to_string(),
        }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Nothing armed is the default, and every accessor agrees with it.
    ///
    /// The anti-tautology arm for the whole module: without it, an accessor
    /// hardwired to its fault would pass every other test here.
    #[test]
    fn an_unarmed_thread_injects_nothing() {
        assert_eq!(armed(), FlashbackFault::None);
        assert_eq!(filter_transport(Some(7)), Some(7));
        assert!(transport_error().is_none());
        assert!(channel_open_error().is_none());
        assert!(publish_error().is_none());
    }

    /// Each fault reaches EXACTLY its own accessor.
    ///
    /// The cross-checks are the point: a `filter_transport` that keyed on
    /// "anything armed" would pass a test that only ever armed `NoTransport`,
    /// and would then make the channel-open and publish arms unreachable in
    /// production tests for reasons nothing would explain.
    #[test]
    fn each_fault_reaches_only_its_own_branch() {
        {
            let _g = arm(FlashbackFault::NoTransport);
            assert_eq!(filter_transport(Some(7)), None);
            assert!(transport_error().is_some());
            assert!(channel_open_error().is_none());
            assert!(publish_error().is_none());
        }
        {
            let _g = arm(FlashbackFault::ChannelOpen);
            assert_eq!(filter_transport(Some(7)), Some(7));
            assert!(transport_error().is_none());
            assert!(channel_open_error().is_some());
            assert!(publish_error().is_none());
        }
        {
            let _g = arm(FlashbackFault::Publish);
            assert_eq!(filter_transport(Some(7)), Some(7));
            assert!(transport_error().is_none());
            assert!(channel_open_error().is_none());
            assert!(publish_error().is_some());
        }
    }

    /// The fault is STICKY: it survives repeated reads.
    ///
    /// A fire-once hook would fail the FIRST request of a batch and silently
    /// succeed for the rest, which is the opposite of the per-request reporting
    /// contract the publish branch pins.
    #[test]
    fn an_armed_fault_survives_repeated_reads() {
        let _g = arm(FlashbackFault::Publish);
        for _ in 0..5 {
            assert!(publish_error().is_some());
        }
    }

    /// The guard restores what it displaced, and disarms on the way out.
    #[test]
    fn the_guard_restores_the_previous_fault_and_nests() {
        {
            let _outer = arm(FlashbackFault::NoTransport);
            {
                let _inner = arm(FlashbackFault::Publish);
                assert_eq!(armed(), FlashbackFault::Publish);
            }
            assert_eq!(
                armed(),
                FlashbackFault::NoTransport,
                "the inner guard must restore the outer fault, not clear it"
            );
        }
        assert_eq!(armed(), FlashbackFault::None);
    }

    /// A panic between arm and disarm still disarms.
    ///
    /// The contamination this guard exists to prevent: a leaked fault makes a
    /// LATER, unrelated test on the same libtest worker log a warn it never
    /// asked for.
    #[test]
    fn a_panicking_scope_still_disarms() {
        let caught = std::panic::catch_unwind(|| {
            let _g = arm(FlashbackFault::ChannelOpen);
            panic!("deliberate");
        });
        assert!(caught.is_err(), "the panic must propagate");
        assert_eq!(armed(), FlashbackFault::None);
    }
}
