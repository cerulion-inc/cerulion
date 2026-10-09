// SPDX-License-Identifier: AGPL-3.0-only
//! WHEN the listener drain runs, counted.
//!
//! The drain changes no answer: it clears a wake queue nothing on these paths
//! waits on. So no behaviour test can see where it runs, and the rule it follows
//! has to be pinned by a COUNT instead. `listener_drain_count` is that count, a
//! test-visible tally bumped in the FALLIBLE CORE of the drain, the one function
//! every spelling reaches: a drain written as a direct core call moves it exactly
//! like one written through the infallible wrapper, so an arm here cannot be fooled
//! by the spelling a cost is put back in.
//!
//! # The rule these arms pin
//!
//! A read that REMOVED frames from the queue drains once. A read that removed none
//! drains nothing. That is a cost rule, not a safety one: on iceoryx2 0.10 a drain
//! of an empty listener is two `recvmsg` calls, two sequentially consistent atomic
//! operations and a walk of the shared-memory counting bitset, where 0.9.1 was one
//! `recvmsg`, and a read that finds nothing is the common case for a colocated edge
//! whose notify is elided at the source.
//!
//! What this file does NOT claim: that a listener left undrained costs a publisher
//! anything. On 0.10 it does not, which `notify_delivery_latch`'s module doc
//! records; that is why the rule can be keyed on cost alone.
//!
//! ```bash
//! cargo test -p cerulion_core --test listener_drain_count_test
//! ```

use std::time::{Duration, SystemTime};

use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::{listener_drain_count, reset_listener_drain_count};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::std_msgs::Int32;
use serial_test::serial;

fn unique_topic(arm: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    format!("/test/drain_count/{arm}/{nanos}")
}

fn slot_len() -> MaxSliceLen {
    MaxSliceLen::const_new((WireHeader::SIZE + 64) as u32)
}

fn publish(publisher: &mut CerulionPublisher, value: i32) {
    let mut proxy = publisher.loan_proxy::<Int32>().expect("a loan");
    proxy.data = value;
}

/// One publisher and one subscriber on a topic of this arm's own.
fn pair(
    arm: &str,
) -> (
    CerulionPublisher,
    cerulion_core::transport::subscriber::CerulionSubscriber,
) {
    let mgr = TransportManager::get_or_init().expect("a transport manager");
    let topic = unique_topic(arm);
    let publisher = mgr
        .create_publisher_simple(&topic, slot_len())
        .expect("a publisher");
    let subscriber = mgr.create_subscriber(&topic).expect("a subscriber");
    std::thread::sleep(Duration::from_millis(50));
    (publisher, subscriber)
}

/// A read that removed a frame drains ONCE.
#[test]
#[serial]
fn a_read_that_removed_frames_drains_once() {
    let (mut publisher, sub) = pair("removed");
    publish(&mut publisher, 1);
    std::thread::sleep(Duration::from_millis(20));

    reset_listener_drain_count();
    let found = sub
        .try_receive_one(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("a receive");
    let drains = listener_drain_count();

    assert!(found, "the frame just published must be found");
    assert_eq!(
        drains, 1,
        "a read that removed one frame must drain exactly once; {drains} means the drain \
         moved back to the head of the read, or ran twice around one frame"
    );
}

/// A read that removed nothing drains NOTHING.
#[test]
#[serial]
fn a_read_that_removed_nothing_does_not_drain() {
    let (_publisher, sub) = pair("empty");

    reset_listener_drain_count();
    let found = sub
        .try_receive_one(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("an empty receive");
    let drains = listener_drain_count();

    assert!(!found, "nothing was published, so no read may find a frame");
    assert_eq!(
        drains, 0,
        "a read that removed nothing must not drain at all; {drains} is the cost this \
         policy exists to stop paying, two recvmsg calls, two sequentially consistent \
         atomics and a bitset walk on every read that finds nothing"
    );
}

/// The batch path follows the same rule: frames removed drains once, empty drains none.
#[test]
#[serial]
fn the_batch_path_drains_on_removals_only() {
    let (mut publisher, sub) = pair("batch");
    publish(&mut publisher, 7);
    std::thread::sleep(Duration::from_millis(20));

    reset_listener_drain_count();
    let delivered = sub
        .try_receive(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("a batch receive");
    let after_frames = listener_drain_count();

    reset_listener_drain_count();
    let empty = sub
        .try_receive(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("an empty batch receive");
    let after_empty = listener_drain_count();

    assert_eq!(delivered, 1, "exactly the one published frame is delivered");
    assert_eq!(
        after_frames, 1,
        "a batch that removed one frame must drain exactly once; got {after_frames}"
    );
    assert_eq!(empty, 0, "the second batch finds nothing");
    assert_eq!(
        after_empty, 0,
        "a batch that removed nothing must not drain; got {after_empty}"
    );
}

/// The LOANED TAKE's three exits, which the call-site walk cannot cover.
///
/// `try_receive_one_owned` holds three policy call sites, one per exit, so the walk
/// keeps attributing the function while any one of them remains: deleting the drain
/// from its empty exit alone is invisible there. This arm covers the two exits whose
/// removals would otherwise go unreported, the delivered one and the empty one.
#[test]
#[serial]
fn the_loaned_take_drains_on_removals_only() {
    let (mut publisher, sub) = pair("owned");
    publish(&mut publisher, 3);
    std::thread::sleep(Duration::from_millis(20));

    reset_listener_drain_count();
    let taken = sub.try_receive_one_owned().expect("an owned receive");
    let after_frame = listener_drain_count();
    drop(taken);

    reset_listener_drain_count();
    let empty = sub.try_receive_one_owned().expect("an empty owned receive");
    let after_empty = listener_drain_count();

    assert_eq!(
        after_frame, 1,
        "a loaned take that removed one frame must drain exactly once; got {after_frame}"
    );
    assert!(
        empty.is_none(),
        "the second take finds nothing, which is the exit this arm is here for"
    );
    assert_eq!(
        after_empty, 0,
        "a loaned take that removed nothing must not drain; got {after_empty}"
    );
}
