// SPDX-License-Identifier: AGPL-3.0-only
//! A publisher must DRAIN ITS OWN event listener on every path that
//! notifies — end-to-end over real iceoryx2.
//!
//! # The failure this pins
//!
//! iceoryx2's `Notifier::notify_with_custom_event_id` calls
//! `__internal_notify(value, skip_self_deliver = false)`
//! (`iceoryx2-0.9.1/src/port/notifier.rs:486`), so a notify is delivered to
//! EVERY listener registered on the topic's event service — including the
//! notifying publisher's OWN listener (each `CerulionPublisher` owns one, to
//! hear `SubscriberConnected`). Each delivery is an 8-byte datagram on an
//! `AF_UNIX SOCK_DGRAM` socket with a bounded kernel buffer.
//!
//! The typed loan path drains that socket on every `loan_proxy`
//! (`check_subscriber_events()`), so it never accumulates. `publish_raw` must do
//! the same, because it is armed to notify on every raw ingress publish
//! (`create_ingress_publisher` → `arm_publish_raw_notify`). Without the drain a
//! raw-ingress publisher fills its OWN socket after a few hundred publishes,
//! after which EVERY notify fails with
//! `NotifierNotifyError::FailedToDeliverSignal` and iceoryx2 logs a `warn!`
//! once per publish (`notifier.rs:525`). On an attached robot running
//! `cerulion graph run attach --single-process` — whose `dds_bridge` node opens
//! ~90 `RawIngressRoute`s, all `create_ingress_publisher` + `publish_raw` —
//! that is ~2500 lines/s ≈ 5 MB/s, enough to fill a root disk.
//!
//! # What the tests assert
//!
//! The oracle is `CerulionPublisher::notify_undelivered_count()` — the
//! unconditional counter, which is incremented exactly when a notify reached
//! fewer listeners than the topic's live listener count. It is log-level
//! independent, so these tests do not depend on iceoryx2's own (correctly
//! suppressed) warning.
//!
//! * [`undrained_self_listener_saturates_and_is_counted`] — the CONDITION is
//!   real and the detector fires (anti-tautology: proves the apparatus moves).
//! * [`ingress_publish_raw_never_saturates_its_own_listener`] — the guarantee.
//!   Deleting `self.check_subscriber_events();` from
//!   `publish_raw`'s notify branch makes exactly this test fail.
//! * [`draining_recovers_the_latch_and_rearms_the_loud_path`] — the latch's
//!   regime lifecycle over REAL transport: one loud head, sustained repeats
//!   downgraded, one recovery, re-armed.
//! * [`notify_delivery_log_arms_map_warn_then_debug_then_recovery`] — the
//!   `tracing` LEVEL mapping through the production call site (an inverted
//!   warn/debug mapping is invisible to the counter-only arms above).
//!
//! Per-test SHM roots + unique topics ⇒ parallel-safe (NO `#[serial]`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use cerulion_core::testing::{count_at_exclusively, debug_level_compiled_in, line_level};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use tracing_test::traced_test;

/// Notifies to issue when we WANT the socket to saturate. The bound must exceed
/// any plausible `AF_UNIX SOCK_DGRAM` capacity for 8-byte datagrams: Linux's
/// default 212992-byte buffer holds only a few hundred (each datagram is
/// charged its full `SKB_TRUESIZE`), macOS's default is far smaller. 20k
/// non-blocking `sendto`s run in tens of milliseconds.
const SATURATING_NOTIFIES: usize = 20_000;

/// Publishes to issue on the guarantee test. Same reasoning as
/// [`SATURATING_NOTIFIES`], sized well past saturation so a regression cannot
/// hide behind an under-filled socket. Kept lower because each iteration is a
/// full loan + copy + send + drain + notify.
const GUARANTEE_PUBLISHES: usize = 5_000;

fn unique_id() -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// A fresh per-test `TransportManager` (isolated SHM root, network=None) plus a
/// unique canonical topic.
fn setup(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{base}_{id}"),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test");
    (transport, format!("/{base}/{id}"))
}

/// Hand-build a minimal raw wire frame (32-byte little-endian header + payload).
fn make_wire_frame(schema_hash: u64, seq: u32, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// ANTI-TAUTOLOGY / apparatus pin: notifying WITHOUT ever draining the
/// publisher's own listener really does saturate its event socket, and the
/// undelivered-notify counter sees it.
///
/// This is a `publish_raw` with no self-drain, reproduced directly through the
/// public API (`notify_sent_sample` with no paired
/// `check_subscriber_events`). Without this arm, the self-drain test below could pass
/// vacuously on a platform where the socket never fills.
///
/// The hand oracle is the DIRECTION of two counts, not a platform-specific
/// capacity: healthy at the start (the first notifies are delivered) and
/// degraded by the end (`> 0` undelivered), on a publisher whose only listener
/// is its own.
#[test]
fn undrained_self_listener_saturates_and_is_counted() {
    let (transport, topic) = setup("saturate");
    let publisher = transport
        .create_publisher(&topic, MaxSliceLen::const_new(256), 0)
        .expect("publisher");

    // Precondition (hand oracle): a fresh publisher has delivered nothing and
    // is NOT degraded — so a nonzero count later cannot be pre-existing state.
    assert_eq!(
        publisher.notify_undelivered_count(),
        0,
        "a fresh publisher must have zero undelivered notifies"
    );

    // The very first notify MUST succeed: it proves the publisher's own
    // listener is a real, connected recipient (if it were not connected at all,
    // the saturation assert below would be meaningless).
    let first = publisher.notify_sent_sample().expect("first notify");
    assert!(
        first >= 1,
        "the publisher's OWN listener must be a live recipient of its own notify \
         (iceoryx2 self-delivers: skip_self_deliver = false); triggered {first}"
    );
    assert_eq!(
        publisher.notify_undelivered_count(),
        0,
        "a delivered notify must not count as undelivered"
    );

    for _ in 0..SATURATING_NOTIFIES {
        let _ = publisher.notify_sent_sample();
    }

    assert!(
        publisher.notify_undelivered_count() > 0,
        "after {SATURATING_NOTIFIES} notifies with the publisher's own listener never drained, \
         its AF_UNIX event socket must be full and notifies must start failing \
         (FailedToDeliverSignal) — got 0 undelivered, so either the platform's socket is \
         unexpectedly unbounded or the shortfall detector is not wired"
    );
}

/// The guarantee: a raw-ingress publisher — the exact
/// `create_ingress_publisher` + `publish_raw` shape the `ros2 attach`
/// `dds_bridge` node uses ~90 times — never saturates its own listener, because
/// `publish_raw`'s notify branch drains it first.
///
/// Deleting `self.check_subscriber_events();` from `publish_raw` in
/// `cerulion_core/src/transport/publisher.rs` makes this assertion fail with a
/// large nonzero count, while every other test in the suite stays green.
#[test]
fn ingress_publish_raw_never_saturates_its_own_listener() {
    let (transport, topic) = setup("ingress_drain");
    let mut ingress = transport
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(256))
        .expect("ingress publisher");

    let payload = [0xAAu8, 0xBB, 0xCC, 0xDD];
    for seq in 0..GUARANTEE_PUBLISHES {
        let frame = make_wire_frame(0x1234_5678_9ABC_DEF0, seq as u32, &payload);
        ingress.publish_raw(&frame).expect("publish_raw");
    }

    // EXACTLY 0 is a legitimate oracle here, not an over-tight one. The
    // accessor's docs name ONE accepted false positive — a listener
    // DEREGISTERING between an elision-ARMED publisher's count read and its
    // notify — and this environment structurally excludes it twice over: an
    // ingress publisher is never elision-armed (only the graph build calls
    // `arm_notify_elision`), so every classification here takes the self-read
    // `AfterNotify` path; and the topic's only listener is the publisher's own,
    // created before the loop and alive after it, so NOTHING attaches or
    // detaches inside the measured window. Any nonzero count is therefore a
    // real undelivered notify.
    assert_eq!(
        ingress.notify_undelivered_count(),
        0,
        "publish_raw must drain the publisher's OWN listener before notifying it \
         (iceoryx2 self-delivers), so its event socket can never fill across \
         {GUARANTEE_PUBLISHES} publishes — a nonzero count is the self-fill regression"
    );
}

/// The latch's full regime lifecycle over REAL transport, and the proof that
/// DRAINING is what heals it: saturate → the latch is degraded → drain the
/// publisher's own listener (`check_subscriber_events`, the same call
/// `publish_raw` and `loan_proxy` make) → notifies are delivered again and the
/// counter STOPS growing (it is never reset — Principle #3), and the loud path
/// is re-armed for a fresh regime.
#[test]
fn draining_recovers_the_latch_and_rearms_the_loud_path() {
    let (transport, topic) = setup("recover");
    let mut publisher = transport
        .create_publisher(&topic, MaxSliceLen::const_new(256), 0)
        .expect("publisher");

    for _ in 0..SATURATING_NOTIFIES {
        let _ = publisher.notify_sent_sample();
    }
    let degraded_total = publisher.notify_undelivered_count();
    assert!(
        degraded_total > 0,
        "precondition: the undrained regime must have opened"
    );

    // Drain our own listener — exactly what `publish_raw` (and
    // `loan_proxy`) do before notifying.
    publisher.check_subscriber_events();

    // The socket has room again, so this notify is delivered.
    let triggered = publisher.notify_sent_sample().expect("post-drain notify");
    assert!(
        triggered >= 1,
        "after draining, the publisher's own listener must be reachable again; triggered \
         {triggered}"
    );

    // Recovery does NOT reset the unconditional total (the Principle #3
    // queryability signal must survive a heal), and no NEW failure was counted.
    assert_eq!(
        publisher.notify_undelivered_count(),
        degraded_total,
        "a recovered notify must neither reset nor increment the undelivered total"
    );

    // Re-armed: keep draining and the count stays frozen — the healed state is
    // durable, not a one-shot.
    for _ in 0..200 {
        publisher.check_subscriber_events();
        publisher.notify_sent_sample().expect("drained notify");
    }
    assert_eq!(
        publisher.notify_undelivered_count(),
        degraded_total,
        "with the listener drained on every notify the count must never grow again"
    );
}

/// The LOG-ARM MAPPING through the PRODUCTION `notify_sent_sample` call site:
/// first undelivered notify of a regime ⇒ exactly one `warn!`, sustained
/// repeats ⇒ `debug!`, a delivered notify that closes the regime ⇒ exactly one
/// recovery `info!`.
///
/// The pure `NotifyDeliveryLatch` state machine is oracle-tested in
/// `transport/notify_delivery_latch.rs`; this pins that the publisher maps
/// each returned action onto the RIGHT `tracing` level. An inverted mapping
/// (warn↔debug, or recovery emitted per-notify) would ship green without
/// it — the counter-only assertions in the tests above cannot see log levels.
///
/// Hand oracle: 1 warn, ≥1 debug, 1 info, all carrying THIS test's unique
/// topic (so a sibling test's publisher can never satisfy the assertion).
#[test]
#[traced_test]
fn notify_delivery_log_arms_map_warn_then_debug_then_recovery() {
    let (transport, topic) = setup("logarms");
    let mut publisher = transport
        .create_publisher(&topic, MaxSliceLen::const_new(256), 0)
        .expect("publisher");

    // Saturate, stopping as soon as the regime is real so the captured log
    // volume stays small (the sustained arm needs only a couple of repeats).
    let mut opened = false;
    for _ in 0..SATURATING_NOTIFIES {
        let _ = publisher.notify_sent_sample();
        if publisher.notify_undelivered_count() > 0 {
            opened = true;
            break;
        }
    }
    assert!(
        opened,
        "precondition: the undrained regime must open within {SATURATING_NOTIFIES} notifies"
    );
    // Two more undelivered notifies ⇒ the sustained (downgraded) arm. The
    // counter is the level-free leg: it must count them whatever the log did.
    let at_open = publisher.notify_undelivered_count();
    for _ in 0..2 {
        let _ = publisher.notify_sent_sample();
    }
    assert_eq!(
        publisher.notify_undelivered_count(),
        at_open + 2,
        "the sustained arm must still COUNT every undelivered notify — the counter is \
         independent of log level and never reset by recovery"
    );

    // Drain + notify ⇒ the regime closes with one recovery report.
    publisher.check_subscriber_events();
    publisher.notify_sent_sample().expect("post-drain notify");

    let topic_for_logs = topic.clone();
    logs_assert(move |lines: &[&str]| {
        let mine: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains(&topic_for_logs))
            .collect();
        // Each count is level-token-matched AND paired with the level-free
        // total of its own conjunction (this topic + this marker), so neither a
        // line demoted to another level nor a duplicate of it at another level
        // reads as the one line.
        let warns =
            count_at_exclusively(lines, "WARN", &["reached fewer listeners", &topic_for_logs])?;
        // Level-free: a suppressed repeat must never be LOUD. This is the half
        // of the contract that survives `release_max_level_info`, where the
        // DEBUG count below reads 0 and its lower bound cannot bite. FIRST, so it
        // is this arm — which names the condition — that fires on a promoted
        // repeat, rather than the exclusive DEBUG count's generic "a copy at
        // another level".
        let loud_repeats = mine
            .iter()
            .filter(|l| {
                matches!(line_level(l), Some("WARN" | "INFO" | "ERROR"))
                    && l.contains("still undeliverable (suppressed)")
            })
            .count();
        if loud_repeats != 0 {
            return Err(format!(
                "a sustained repeat was emitted at a LOUD level on {topic_for_logs} \
                 ({loud_repeats} line(s)) — the downgrade to debug! is the flood suppression"
            ));
        }
        let debugs = count_at_exclusively(
            lines,
            "DEBUG",
            &["still undeliverable (suppressed)", &topic_for_logs],
        )?;
        let recoveries = count_at_exclusively(
            lines,
            "INFO",
            &["notify delivery recovered", &topic_for_logs],
        )?;
        if warns != 1 {
            return Err(format!(
                "expected EXACTLY one loud regime-opening warn! on {topic_for_logs}, got \
                 {warns} (a per-notify warn is the flood the latch exists to prevent; 0 means \
                 the loud head was downgraded)"
            ));
        }
        if debug_level_compiled_in() && debugs < 1 {
            return Err(format!(
                "expected the sustained repeats to be downgraded to debug! on \
                 {topic_for_logs}, got {debugs}"
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected EXACTLY one recovery info! when the regime closed on \
                 {topic_for_logs}, got {recoveries}"
            ));
        }
        Ok(())
    });
}
