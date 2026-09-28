// SPDX-License-Identifier: AGPL-3.0-only
//! The readiness sweep's two load-bearing assumptions, pinned against a live
//! iceoryx2 listener and against the source rather than against a comment.
//!
//! The live loop's two idle poll loops (the user-space spin and the park's
//! recheck) no longer ask each listener "is there anything" by DRAINING it.
//! They ask once for the whole set with a non-blocking `poll(2)` over the
//! listeners' doorbell file descriptors, and drain only the listeners that
//! answered. A census of the live loop on the round-trip bench graph counted
//! 969 recheck polls and 38 spin polls per step against 2 drains on the step
//! path, so those two loops are where essentially all of the drain cost was.
//!
//! That rewrite rests on exactly two things, and this file is both of them:
//!
//! 1. **The doorbell fd is a correct AND level-triggered readiness signal.** A
//!    notify must make it readable, and it must STAY readable across repeated
//!    polls until a drain takes the events. If it were edge triggered, a byte
//!    that arrived before a sweep began would be reported once and then lost,
//!    and the wake would be deferred to the park timeout.
//! 2. **The sweep reads file descriptors and nothing else.** It must not touch
//!    the barrier, the gating clock, or any lockstep state, because the loops
//!    it lives in are on the record-only side of the live loop.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};

/// The doorbell fd answers "are there undrained events", and keeps answering.
///
/// iceoryx2 0.10 sends the doorbell byte only on the IDLE to PENDING
/// transition and skips the send while the listener is already NOTIFIED;
/// nothing consumes the byte but a drain. The sweep depends on both halves of
/// that: readable when there is something, and NOT readable once drained.
///
/// The repeated-poll arm is the level-triggered half. It polls three times
/// without draining and requires every poll to say ready. An edge-triggered
/// multiplexer would pass the first and fail the rest, which is precisely the
/// failure that would strand a wake until the park timeout.
#[test]
fn the_doorbell_fd_is_a_level_triggered_readiness_signal() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "sweep_probe".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 4,
            network: None,
        },
        ix,
    )
    .expect("transport on an isolated config");

    let topic = "/sweep/readiness";
    let publisher = mgr
        .create_publisher_simple(topic, cerulion_core::wire::MaxSliceLen::const_new(64))
        .expect("publisher");
    let subscriber = mgr.create_subscriber(topic).expect("subscriber");
    let fd = subscriber.event_listener_fd();

    // Whatever the attach left behind, start from a drained listener.
    subscriber
        .drain_event_notifications()
        .expect("drain to a known state");
    assert!(
        !poll_ready(fd),
        "a drained listener must not report readable, or the sweep would drain \
         on every turn and buy nothing"
    );

    // A notify makes it readable.
    publisher.notify_sent_sample().expect("notify");
    assert!(
        poll_ready(fd),
        "a notify must make the doorbell readable, or the sweep would miss a \
         real wake and the loop would fall back to the park timeout"
    );

    // LEVEL TRIGGERED: it stays readable across repeated polls, because
    // nothing but a drain consumes the byte.
    for turn in 0..3 {
        assert!(
            poll_ready(fd),
            "poll turn {turn} found the doorbell NOT readable while events are \
             still undrained: the multiplexer is edge triggered, and a byte that \
             arrived before a sweep began would be reported once and then lost"
        );
    }

    // And the drain is what clears it. This is the half that makes the sweep
    // safe: the drain still does the clearing, the sweep only asks.
    subscriber
        .drain_event_notifications()
        .expect("drain the events");
    assert!(
        !poll_ready(fd),
        "after a drain the doorbell must be quiet again, or every later sweep \
         would report a stale ready and the spin would never settle"
    );

    // Repeats of the same id coalesce into one doorbell state, so several
    // notifies still leave exactly one thing to drain and one readable fd.
    for _ in 0..8 {
        publisher.notify_sent_sample().expect("notify");
    }
    assert!(poll_ready(fd), "a burst must leave the doorbell readable");
    subscriber.drain_event_notifications().expect("drain");
    assert!(
        !poll_ready(fd),
        "ONE drain must clear a burst, or the sweep would report ready with \
         nothing left to drain and the spin would burn its whole budget"
    );
}

/// A non-blocking readiness probe on one fd, the same shape the sweep runs
/// over the whole set.
fn poll_ready(fd: std::os::unix::io::RawFd) -> bool {
    let mut p = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    // SAFETY: one valid pollfd; timeout 0 makes it non-blocking.
    let rc = unsafe { libc::poll(&mut p, 1, 0) };
    rc > 0 && p.revents & (libc::POLLIN | libc::POLLERR | libc::POLLHUP) != 0
}

/// The sweep reads file descriptors and NOTHING else.
///
/// Both loops it serves are record-only: they change WHEN the live loop wakes,
/// never what fires. A sweep that consulted the barrier, the gating clock or
/// any lockstep state would put scheduling input into a path whose whole
/// contract is that it has none, and it would also tie the sweep to machinery
/// that is being removed.
///
/// A source walk rather than a runtime assertion, because the property is the
/// ABSENCE of a read: no test input can demonstrate that something was not
/// consulted, and the walk fails the day someone adds one.
#[test]
fn the_readiness_sweep_reads_only_file_descriptors() {
    let src = std::fs::read_to_string(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/graph/runtime.rs"),
    )
    .expect("the runtime source is readable");

    let start = src
        .find("pub(crate) struct ListenerSweep {")
        .expect("the sweep must still be called ListenerSweep");
    let end = src[start..]
        .find("fn park_poll_fd_ready")
        .map(|o| start + o)
        .expect("the sweep sits directly above the single-fd probe");
    let body = &src[start..end];
    assert!(
        body.len() > 400,
        "the walked region is implausibly short ({} bytes): the anchors moved and \
         this guard is reading the wrong code",
        body.len()
    );

    // Anything that would make the sweep a scheduling input rather than a
    // readiness probe. `wake_seq` and the quantum are the barrier's own words;
    // `watch_clock` is the deterministic timeline the record-only firewall
    // forbids this path from advancing.
    for banned in [
        "barrier",
        "wake_seq",
        "quantum",
        "lockstep",
        "watch_clock",
        "scheduler",
        "signal_data",
        "fire_node",
        "try_receive",
    ] {
        assert!(
            !body.contains(banned),
            "the readiness sweep references `{banned}`. It must read file \
             descriptors and nothing else: it runs inside two record-only poll \
             loops, and it must not depend on machinery that is being removed"
        );
    }

    // The positive half, so the guard cannot pass on a sweep that was gutted:
    // it really is one non-blocking poll over a set of descriptors.
    assert!(
        body.contains("libc::poll("),
        "the sweep must still be one `poll(2)` over the set"
    );
    assert!(
        body.contains("file_descriptor()"),
        "the sweep must still read its targets from the listeners' descriptors"
    );
}

/// The ready branch of BOTH sweeps still drains, and the step path still
/// drains every wake listener unconditionally.
///
/// This is the guard for the one mutant the behaviour cannot catch. Removing
/// the drain that follows a sweep changes nothing any test can see, and that
/// is measured, not assumed: `drain_level` drains every wake listener in the
/// level on every step whether or not its node fires — the `DrainSource::Unified`
/// arm and the Sync per-set arm each `try_wait` before they read anything — so
/// no listener in the live loop's source list can carry an event across a step
/// for an idle poll to find. With the drain after the park's sweep removed,
/// `unified_stale_wake_park_test` and seven other park and wake suites stay
/// green.
///
/// The drain stays anyway, for two reasons a walk can defend and a run cannot:
/// it keeps the rewrite a change of QUESTION rather than of behaviour, and it
/// is the only thing between a future wake source with no step-path drain and
/// a live loop that stops parking. So this walk pins the PAIR — the two idle
/// drains and the two step-path drains they lean on — and fails the day either
/// half is deleted, which is the day the other half starts mattering.
#[test]
fn every_sweep_still_drains_and_the_step_path_still_drains_unconditionally() {
    let src = std::fs::read_to_string(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/graph/runtime.rs"),
    )
    .expect("the runtime source is readable");

    // Both idle poll loops: every `sweep.poll_ready()` branch drains what the
    // sweep named. The window is generous because the branch carries the
    // reasoning above it; the anchor is the `poll_ready` call itself.
    let sweeps: Vec<usize> = src
        .match_indices("if sweep.poll_ready() {")
        .map(|(i, _)| i)
        .collect();
    assert_eq!(
        sweeps.len(),
        2,
        "expected exactly two readiness sweeps (the spin and the park recheck); \
         found {}. A third sweep needs its own drain and its own row here",
        sweeps.len()
    );
    for (n, start) in sweeps.iter().enumerate() {
        // The window ends at the loop's own closing brace column rather than a
        // character count: the branch carries a long reasoning block above the
        // drain, and a fixed budget would fail on a comment edit.
        let tail = &src[*start..];
        let end = tail
            .find("\n            }\n")
            .expect("each sweep branch closes at its own indentation");
        let window = &tail[..end];
        assert!(
            window.contains("listener.try_wait("),
            "readiness sweep {n} reports a listener ready and never drains it. \
             The sweep only changed how the loop ASKS: the drain is still what \
             clears the event, and without it a wake source with no step-path \
             drain would keep the live loop off the park forever"
        );
    }

    // The step path the claim above leans on: two unconditional drains, one per
    // wake-source family. Both carry the same marker line, which is what makes
    // them countable here.
    let step_drains = src
        .matches("// iceoryx2 0.10: one `try_wait` empties the queue.")
        .count();
    assert_eq!(
        step_drains, 2,
        "expected the step path to carry exactly two unconditional wake-listener \
         drains (the `DrainSource::Unified` arm and the Sync per-set arm); found \
         {step_drains}. If one is gone, the idle drains above are no longer \
         belt-and-braces and the mutant they guard becomes a live bug"
    );
}
