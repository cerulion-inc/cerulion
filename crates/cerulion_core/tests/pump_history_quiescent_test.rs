// SPDX-License-Identifier: AGPL-3.0-only
//! Quiescent-publisher history delivery.
//!
//! Native iceoryx2 history is delivered to a late joiner by
//! `publisher.update_connections()`, which Cerulion drives from
//! `deliver_history` — but ONLY off the publish path (`loan_proxy` →
//! `check_subscriber_events`). So a publisher that filled its history queue
//! and then went QUIESCENT (never sends again) would leave a late joiner with
//! nothing, because the `SubscriberConnected` event is never drained.
//!
//! `CerulionPublisher::pump_history()` is a public, publish-free
//! driver the runtime calls on a cadence (in `live_step`, off the firing
//! path) so quiescent publishers still service late joiners. These tests pin
//! the publisher-level mechanism directly over an isolated `TestTransport`
//! (per-test SHM root → parallel-safe).
//!
//! Contrast with `history_test.rs`, whose tests pump delivery by doing "one
//! more `loan_proxy()`+drop" — here we do NO further publish and assert
//! delivery happens solely because of `pump_history()`.

use cerulion_core::wire::MaxSliceLen;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/pump_hist/{base}/{nanos}/{id}")
}

fn publish_vec3(publisher: &mut CerulionPublisher, x: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
    proxy.x = x;
    proxy.y = 0.0;
    proxy.z = 0.0;
}

fn drain_all_sequences(sub: &mut CerulionSubscriber) -> BTreeSet<u32> {
    let mut seqs = BTreeSet::new();
    for _ in 0..4 {
        let _ = sub
            .wait_for_message(Duration::from_millis(200), |msg| {
                seqs.insert(msg.header().sequence);
            })
            .expect("wait_for_message");
    }
    seqs
}

// ============================================================
// The core fix: a QUIESCENT publisher delivers history via pump_history()
// ============================================================

/// history_size = 3, late subscriber depth >= history. Publish three frames
/// (sequences 0,1,2), attach a late joiner, then — CRUCIALLY — publish NOTHING
/// more. Driving `pump_history()` alone must deliver all three retained
/// frames. This is the quiescent-publisher gap the chunk closes: without the
/// pump, the `SubscriberConnected` event is never drained and the late joiner
/// gets nothing.
#[test]
fn pump_history_delivers_retained_history_to_a_quiescent_publishers_late_joiner() {
    let topic = unique_topic("quiescent_full");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);

    // Fill the native history queue, then go quiescent.
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }

    // Late joiner attaches AFTER the publisher stopped publishing.
    let mut late = tt.subscriber(&topic);

    // NO further publish — the ONLY driver is the runtime-style pump.
    publisher.pump_history();

    let seqs = drain_all_sequences(&mut late);
    for expected in [0u32, 1, 2] {
        assert!(
            seqs.contains(&expected),
            "pump_history() on a quiescent publisher must deliver retained \
             native-history sequence {expected} to the late joiner; got {seqs:?}"
        );
    }
    // No frame was published after connect, so there is no sequence 3.
    assert!(
        !seqs.contains(&3),
        "no frame was published after connect — there must be no sequence 3; got {seqs:?}"
    );
}

/// A second `pump_history()` after the late joiner has already drained its
/// history is a harmless no-op: no `SubscriberConnected` event is pending, so
/// nothing is re-delivered (history is not replayed twice).
#[test]
fn pump_history_does_not_redeliver_after_the_join_was_serviced() {
    let topic = unique_topic("quiescent_no_redeliver");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }
    let mut late = tt.subscriber(&topic);
    publisher.pump_history();
    let first = drain_all_sequences(&mut late);
    assert!(
        first.contains(&0) && first.contains(&1) && first.contains(&2),
        "first pump must deliver the retained history; got {first:?}"
    );

    // A second pump with no NEW subscriber connect drains no pending event and
    // delivers nothing further.
    publisher.pump_history();
    let second = drain_all_sequences(&mut late);
    assert!(
        second.is_empty(),
        "a second pump_history() with no pending SubscriberConnected must \
         deliver nothing (history is not replayed); got {second:?}"
    );
}

/// `pump_history()` on a history-disabled (size 0) publisher is a harmless
/// no-op even with a late joiner present: nothing was retained, so nothing is
/// delivered, and the call neither panics nor errors.
#[test]
fn pump_history_is_harmless_noop_when_history_disabled() {
    let topic = unique_topic("quiescent_disabled");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }
    let mut late = tt.subscriber(&topic);

    // Must not panic; delivers nothing because history_size = 0 retained
    // nothing.
    publisher.pump_history();

    let seqs = drain_all_sequences(&mut late);
    assert!(
        seqs.is_empty(),
        "history-disabled publisher must replay nothing on pump_history(); got {seqs:?}"
    );
}

// ============================================================
// The gate that decides whether a pump drains at all
// ============================================================

/// A listener count change arms MORE THAN ONE drain.
///
/// The gate in `check_subscriber_events` is edge triggered on
/// `number_of_listeners()`, and it deliberately arms a BUDGET rather than a
/// single drain: the count rises when a subscriber's listener is created and
/// the `SubscriberConnected` notify lands after it, so one gated drain could
/// fall inside that window and see nothing. Nothing pinned the budget, so
/// shrinking it to one, which removes exactly that behaviour, was green.
///
/// A publisher with no subscriber yet is the clean stage: the first pass sees
/// the count move off its `usize::MAX` sentinel to one (the publisher's own
/// listener) and arms, and no transition is pending to spend the arming early,
/// so the budget is observable by counting the passes it survives.
#[test]
fn a_listener_count_change_arms_more_than_one_drain() {
    let topic = unique_topic("arming_budget");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);

    // One pass: the count moves off the sentinel, the budget is armed, and this
    // pass spends one of it. No subscriber exists, so no transition is drained.
    publisher.pump_history();
    let after_first = publisher.self_drains_armed_for_test();
    assert!(
        after_first > 1,
        "a listener count change must arm a BUDGET of drains, not one: after \
         the pass that observed the change {after_first} remained armed"
    );

    // Spend it down and count the passes. A budget of one would be exhausted by
    // the first pass above and this loop would run zero times.
    let mut passes = 1_u32;
    while publisher.self_drains_armed_for_test() > 0 {
        publisher.pump_history();
        passes += 1;
        assert!(
            passes < 10_000,
            "the arming budget is not being spent: {passes} passes and \
             {} still armed",
            publisher.self_drains_armed_for_test()
        );
    }
    assert!(
        passes > 1,
        "the budget must survive more than the pass that armed it; it was \
         exhausted after {passes}"
    );
}

/// A net zero listener swap on a quiescent publisher still delivers history.
///
/// This is the interleaving the call budget alone cannot cover, staged exactly:
///
/// 1. a subscriber attaches, is serviced, and the observed transition spends
///    the whole arming budget (asserted below, so the stage is real);
/// 2. that subscriber detaches and another attaches before the next pass, so
///    the listener COUNT is unchanged and the edge triggered gate never fires;
/// 3. the new subscriber's `SubscriberConnected` sits in the publisher's
///    listener with nothing left to drain it.
///
/// On a publisher that still sends, iceoryx2 rescues this itself: `send_sample`
/// calls `update_connections`, which delivers history to every newly connected
/// subscriber. On a QUIESCENT publisher there is no send, so without a time
/// bound the late joiner waits for the count to move again, which on a quiet
/// topic may be never. The idle deadline is what closes it, and this arm fails
/// the moment that deadline is removed from `pump_history`.
#[test]
fn a_net_zero_listener_swap_still_delivers_history_once_the_idle_deadline_passes() {
    let topic = unique_topic("net_zero_swap");

    let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);
    for x in [10.0_f64, 20.0, 30.0] {
        publish_vec3(&mut publisher, x);
    }

    // Stage 1: the first joiner is serviced, and that observed transition
    // zeroes the budget. Every later pass is the cheap load until the count
    // moves again.
    let first = tt.subscriber(&topic);
    publisher.pump_history();
    assert_eq!(
        publisher.self_drains_armed_for_test(),
        0,
        "servicing the first joiner must spend the arming, otherwise the swap \
         below is covered by leftover budget and this arm proves nothing"
    );

    // Stage 2: the swap. The count returns to what the gate last recorded, so
    // no arming fires, and stage 1 left nothing to spend.
    drop(first);
    let mut late = tt.subscriber(&topic);

    // Stage 3: the idle pass, run as if the deadline had elapsed rather than
    // sleeping for it. This is the ONLY thing that can drain the swap.
    publisher.pump_history_past_the_idle_deadline_for_test();

    let seqs = drain_all_sequences(&mut late);
    for expected in [0u32, 1, 2] {
        assert!(
            seqs.contains(&expected),
            "the idle deadline must deliver retained sequence {expected} to a \
             late joiner that arrived on a net zero listener swap; got {seqs:?}"
        );
    }
}

/// The idle deadline's arming is SPENDABLE at the cadence an idle loop runs.
///
/// The gate's promise is that a publisher with no listener transitions pays one
/// relaxed load per call, and the deadline has to re-arm without taking that
/// promise back. It arms a budget spent one per call, so an arming LARGER than
/// the calls that fit in one interval is never spendable: the drain the gate
/// exists to remove runs on every call for the life of the process, and every
/// arm in this file still passes, because they all assert that history IS
/// delivered.
///
/// A quarter second interval against a 20ms pump is about twelve calls, and
/// against a 100ms live step about two, so the count change's budget of 64
/// cannot be reused here. The deadline arms its own small one, and since the
/// slower cadence is the harder case, both are driven.
///
/// Deterministic, with no sleeping: `pump_history_at` takes the instant, so the
/// cadence is dialled rather than waited for. The count change's larger budget
/// is spent first (no subscriber ever attaches, so nothing zeroes it early),
/// then the arm walks past many deadlines and counts the passes that find the
/// arming spent against what the intervals allow.
#[test]
fn the_idle_deadline_arming_is_spendable_at_an_idle_loops_cadence() {
    // TWO cadences, because spendability gets HARDER as the cadence slows: the
    // arming is spendable only where more calls than it fall inside one deadline
    // interval. 20ms is the rmw pump's throttle, about twelve passes per quarter
    // second interval. 100ms is a slow live step, about two. A budget that only
    // cleared the first would leave a slow graph permanently armed, which is the
    // defect this arm exists for.
    for step_ms in [20_u64, 100] {
        let topic = unique_topic(&format!("idle_arming_{step_ms}"));
        let tt = cerulion_core::testing::TestTransport::with_buffer_size(8);
        let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 3);
        let step = Duration::from_millis(step_ms);
        let t0 = std::time::Instant::now();
        // 400 passes: eight seconds of dialled time at 20ms, forty at 100ms, so
        // both walk past many deadlines.
        let passes = 400_u32;

        let mut zeroed_at = None;
        let mut zero_passes = 0_u32;
        for i in 0..passes {
            publisher.pump_history_at(t0 + step * i);
            if publisher.self_drains_armed_for_test() == 0 {
                zero_passes += 1;
                if zeroed_at.is_none() {
                    zeroed_at = Some(i);
                }
            }
        }

        let first = zeroed_at.unwrap_or_else(|| {
            panic!(
                "at one pass every {step_ms}ms the arming was NEVER spent down to \
                 zero across {passes} passes: the deadline is re-arming more calls \
                 than fit in its own interval, so the listener drain this gate \
                 exists to remove runs on every pass forever"
            )
        });
        // DERIVED, and derived as a RANGE because the cadence moves it by one: the
        // count change arms 64 on the first pass, which spends one, so 63 passes
        // of spending remain. A deadline landing on the pass where one is left
        // takes the arming back to two and adds a pass, and where deadlines land
        // depends on the step. Measured: 63 at 20ms, 64 at 100ms.
        //
        // The range is what makes this fail if the count change's own budget is
        // shrunk, which a "greater than one" assertion would not.
        assert!(
            (63..=65).contains(&first),
            "at one pass every {step_ms}ms the arming first read zero on pass \
             {first}, outside 63 to 65: the count change's budget of 64, spent one \
             per pass from the pass that armed it, is what fixes that number"
        );
        // DERIVED: after those 64 are spent, each interval re-arms the small budget
        // and spends it in that interval's first calls, so the non-zero passes are
        // 63 plus about one per interval. Four passes of slack for where the
        // interval boundary falls.
        let intervals = (passes * step_ms as u32).div_ceil(250);
        let floor = passes - 63 - intervals - 4;
        assert!(
            zero_passes >= floor,
            "at one pass every {step_ms}ms only {zero_passes} of {passes} passes \
             found the arming spent, under the {floor} that {intervals} intervals \
             allow: the steady state of a publisher with no listener transitions \
             is the cheap load, not the drain"
        );
    }
}
