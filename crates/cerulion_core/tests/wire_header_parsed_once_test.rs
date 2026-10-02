// SPDX-License-Identifier: AGPL-3.0-only
//! One frame, one header parse, inside the BODY of each of the four read entries
//! that serve a frame.
//!
//! # Why this is counted and not reasoned about
//!
//! A read path that parses the same thirty-two bytes two or three times per
//! frame gives exactly the same answers as one that parses them once. Every
//! value is identical, every test passes, and the only difference is time. So
//! the rule cannot be pinned by a result; it is pinned by a count.
//!
//! `cerulion_core::wire::header_parse_count` is that count: a test-visible tally
//! `WireHeader::read_from_buf` bumps. These arms reset it, publish ONE frame,
//! read that frame through one entry, and assert the tally is exactly one. A
//! re-parse put back anywhere on the path makes the arm read two or three.
//!
//! # The four entries, and why all four
//!
//! The paths differ in who parses and who needs the value afterwards, so one arm
//! cannot stand for the others:
//!
//! * `try_receive_one` parses in its own loop and hands the header to the served
//!   cursor.
//! * `try_receive` goes through the batch path, where the frame is validated and
//!   delivered by one helper and the read log's served sequence and the cursor
//!   both want the header that helper parsed.
//! * `try_view` parses inside the shared frame checks that build the typed view,
//!   and the cursor takes the header those checks returned.
//! * `try_receive_one_owned` is the ROS 2 middleware layer's loaned take: its loop
//!   parses, and the owned sample it returns takes that header for the frame length
//!   its caller sees rather than reading the same thirty two bytes again.
//!
//! # What this file does NOT claim
//!
//! It does not claim nothing else reads a header field. The read log's served
//! sequence on several other paths is a four-byte read of one field rather than
//! a parse of the whole header, and those are not counted here because
//! `read_from_buf` is not what they call.
//!
//! It also does not claim the typed view's own header accessor is counted in.
//! `InputView::wire_header` parses the frame again on demand, which is
//! `read_from_buf`, on a path these arms measure: a closure that calls it, or
//! calls `wire_timestamp_ns` over it, reads two rather than one. The frame checks'
//! header is not threaded into the view yet, and the closure in the third arm
//! below reads two plain fields instead, so what that arm pins is the read path
//! and not the accessor.
//!
//! One more accessor parses on demand, for the same reason, and one more read
//! entry can be made to; both are likewise out of these counts.
//! `OwnedInboundSample::wire_header` re-reads the thirty-two bytes its loop
//! already parsed, and the ROS 2 middleware layer's take calls it one statement
//! after the entry, so that path really costs two parses per frame and the fourth
//! arm reads one only because it never asks for the header. And
//! `spin_view_until_seq` parses ONCE in its own loop to read the sequence, and
//! building the view parses nothing, so a closure that then asks the view for the
//! header pays the second parse, exactly as in the `try_view` case above.
//! Threading the validated header into the owned sample and into the view is the
//! fix for both, and it touches construction sites these commits do not.
//!
//! ```bash
//! cargo test -p cerulion_core --test wire_header_parsed_once_test
//! ```

use std::time::{Duration, SystemTime};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::TransportManager;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::wire::{header_parse_count, reset_header_parse_count, MaxSliceLen, WireHeader};
use native_ros2_messages::sensor_msgs::Image;
use serial_test::serial;

/// The payload every frame carries. Small: this file counts parses, not bytes.
const PAYLOAD: usize = 64;

/// A slot big enough for one frame of `PAYLOAD` bytes.
fn slot_len() -> MaxSliceLen {
    MaxSliceLen::const_new(
        (WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * 3 + PAYLOAD + 64) as u32,
    )
}

fn unique_topic(arm: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    format!("test/parse_once/{arm}/{nanos}")
}

/// Publish one frame, loaning the payload and touching only its first byte.
fn publish(publisher: &mut CerulionPublisher) {
    let mut proxy = publisher.loan_proxy::<Image>().expect("a loan");
    proxy.height = 1;
    proxy.width = PAYLOAD as u32;
    proxy.is_bigendian = 0;
    proxy.step = 1;
    proxy.set_header_bytes(&[]).expect("header bytes");
    proxy.set_encoding("po").expect("an encoding");
    let dst = proxy.loan_data(PAYLOAD).expect("a payload loan");
    if let Some(first) = dst.first_mut() {
        *first = 0x80;
    }
}

/// One publisher and one subscriber on a topic of this arm's own.
fn pair(
    arm: &str,
) -> (
    CerulionPublisher,
    cerulion_core::transport::subscriber::CerulionSubscriber,
    Arc<AtomicU64>,
) {
    let mgr = TransportManager::get_or_init().expect("a transport manager");
    let topic = unique_topic(arm);
    let publisher = mgr
        .create_publisher_simple(&topic, slot_len())
        .expect("a publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("a subscriber");
    // The SERVED CURSOR, which graph wiring installs and a hand-built subscriber
    // does not. It is what puts the serve-point work on the measured path: without
    // it the cursor recorder returns before touching a header, so the parse this
    // commit removed never ran in the first place and these arms would read the
    // same count on either side of the change.
    let cursor = Arc::new(AtomicU64::new(0));
    subscriber.register_service_cursor_for_test(Arc::clone(&cursor));
    std::thread::sleep(Duration::from_millis(50));
    (publisher, subscriber, cursor)
}

/// The callback entry parses one header for one frame.
#[test]
#[serial]
fn try_receive_one_parses_the_header_once_per_frame() {
    let (mut publisher, sub, cursor) = pair("one");
    publish(&mut publisher);

    // Reset AFTER the publish: the publish writes a header, it does not read one,
    // but the loan path is not what this arm is about and the reset makes the
    // count this arm's own either way.
    reset_header_parse_count();
    let found = sub
        .try_receive_one(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("a receive");
    let parses = header_parse_count();

    assert!(found, "the frame just published must be found");
    assert_eq!(
        parses, 1,
        "one frame delivered through `try_receive_one` must cost exactly ONE header parse; \
         {parses} means the served cursor or a guard re-read bytes the loop had already parsed"
    );

    // The cursor ADVANCED, which is what proves the serve-point work ran at all:
    // an arm whose cursor stayed at zero would be reading a path where the recorder
    // returned before it reached a header, and its parse count would be the same on
    // either side of this change.
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The batch entry parses one header for one frame, and the read log's served
/// sequence and the cursor both take that one.
#[test]
#[serial]
fn try_receive_parses_the_header_once_per_frame() {
    let (mut publisher, sub, cursor) = pair("batch");
    publish(&mut publisher);

    reset_header_parse_count();
    let delivered = sub
        .try_receive(|sample| {
            std::hint::black_box(sample.payload().len());
        })
        .expect("a batch receive");
    let parses = header_parse_count();

    assert_eq!(delivered, 1, "exactly the one published frame is delivered");
    assert_eq!(
        parses, 1,
        "one frame through the batch path must cost exactly ONE header parse; {parses} means \
         the served sequence or the cursor re-read what the delivery guard had parsed"
    );

    // The cursor ADVANCED, which is what proves the serve-point work ran at all:
    // an arm whose cursor stayed at zero would be reading a path where the recorder
    // returned before it reached a header, and its parse count would be the same on
    // either side of this change.
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The typed-view entry parses one header for one frame, inside the shared frame
/// checks, and the cursor takes the one those checks returned.
#[test]
#[serial]
fn try_view_parses_the_header_once_per_frame() {
    let (mut publisher, mut sub, cursor) = pair("view");
    publish(&mut publisher);

    reset_header_parse_count();
    let seen = sub
        .try_view::<Image, _>(|view| (view.height, view.width))
        .expect("a view of the frame just published");
    let parses = header_parse_count();

    assert!(seen.is_some(), "the frame just published must be viewable");
    assert_eq!(
        parses, 1,
        "one frame through `try_view` must cost exactly ONE header parse; {parses} means the \
         cursor re-read what the shared frame checks had already parsed"
    );

    // The cursor ADVANCED, which is what proves the serve-point work ran at all:
    // an arm whose cursor stayed at zero would be reading a path where the recorder
    // returned before it reached a header, and its parse count would be the same on
    // either side of this change.
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The owned entry parses one header for one frame too.
///
/// It is the rmw loaned-take path, and it is the arm that reds on the second parse
/// this commit removed from it: the owned sample used to compute its own frame
/// length from a fresh `read_from_buf` over bytes the loop above had already
/// parsed and bounds-checked.
#[test]
#[serial]
fn try_receive_one_owned_parses_the_header_once_per_frame() {
    let (mut publisher, sub, cursor) = pair("owned");
    publish(&mut publisher);

    reset_header_parse_count();
    let taken = sub.try_receive_one_owned().expect("an owned receive");
    let parses = header_parse_count();

    assert!(taken.is_some(), "the frame just published must be taken");
    assert_eq!(
        parses, 1,
        "one frame taken through `try_receive_one_owned` must cost exactly ONE header parse; \
         {parses} means the owned sample re-read the thirty-two bytes the loop had parsed"
    );
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}
