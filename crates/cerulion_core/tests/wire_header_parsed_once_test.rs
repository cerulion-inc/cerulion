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
//! # The accessors, which are inside the count
//!
//! Three arms ASK for the header rather than reading two plain fields, which is
//! what makes the accessors part of the rule rather than a carve-out from it. The
//! typed view carries the header the frame checks parsed and the owned sample
//! carries the one its loop parsed, so `InputView::wire_header`,
//! `wire_timestamp_ns` over it, and `OwnedInboundSample::wire_header` each read a
//! copy and the tally stays at one. The third of those arms reads the spin entry,
//! which parses in its own loop to find the sequence it waits for and hands that
//! header to the view it builds.
//!
//! The ROS 2 middleware layer's take is the caller that asks: it calls the owned
//! sample's accessor one statement after the entry returns.
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
    // returns before it reaches a header, and such a path parses one header whatever
    // the entry does with it, so the count alone could not tell the two apart.
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
    // returns before it reaches a header, and such a path parses one header whatever
    // the entry does with it, so the count alone could not tell the two apart.
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
    // returns before it reaches a header, and such a path parses one header whatever
    // the entry does with it, so the count alone could not tell the two apart.
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The raw-view entry parses one header for one frame, inside the same shared
/// frame checks the typed view runs, and both the cursor and the view's own
/// `header()` take the one those checks returned.
#[test]
#[serial]
fn view_raw_parses_the_header_once_per_frame() {
    let (mut publisher, mut sub, cursor) = pair("raw");
    publish(&mut publisher);

    reset_header_parse_count();
    let view = sub
        .view_raw_expecting(Image::SCHEMA_HASH)
        .expect("a raw view of the frame just published");
    // Through the public `header()` accessor the doc comment names, so a
    // `header()` that re-parsed would land inside the count below.
    let schema_hash = view.as_ref().map(|view| view.header().schema_hash);
    let parses = header_parse_count();

    assert_eq!(
        schema_hash,
        Some(Image::SCHEMA_HASH),
        "the frame just published must be viewable, and its header must ride the view"
    );
    assert_eq!(
        parses, 1,
        "one frame through `view_raw_expecting` must cost exactly ONE header parse; {parses} \
         means the cursor or the view re-read what the shared frame checks had already parsed"
    );
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The owned entry parses one header for one frame too.
///
/// It is the rmw loaned-take path, and it is the arm that reds on a second parse at
/// the point the owned sample takes its frame length: that length comes from the
/// header the loop above already parsed and bounds-checked, so a fresh
/// `read_from_buf` over the same bytes costs a parse and answers nothing new.
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

/// The typed view's own header accessor is inside the count.
///
/// The closure asks the view for the whole header and for the publish stamp. Both
/// read the copy the frame checks handed the view at construction, so the frame
/// costs ONE parse however many times the closure asks.
#[test]
#[serial]
fn try_view_asking_the_view_for_its_header_parses_once() {
    let (mut publisher, mut sub, cursor) = pair("view_header");
    publish(&mut publisher);

    reset_header_parse_count();
    let seen = sub
        .try_view::<Image, _>(|view| (view.wire_header(), view.wire_timestamp_ns()))
        .expect("a view of the frame just published");
    let parses = header_parse_count();

    let (header, stamp) = seen.expect("the frame just published must be viewable");
    assert_eq!(
        parses, 1,
        "a view whose closure asks for the header must still cost ONE parse for the frame; \
         {parses} means the accessor parsed the bytes the frame checks had already parsed"
    );
    assert_eq!(
        header.schema_hash,
        Image::SCHEMA_HASH,
        "the header the accessor returned must be this frame's own, not a fabricated one"
    );
    assert!(
        stamp > 0 && stamp == header.timestamp_ns,
        "the stamp projection must read the same header the accessor returned; got {stamp} \
         against {}",
        header.timestamp_ns
    );
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The owned sample's header accessor is inside the count, which is the ROS 2
/// middleware layer's own shape: it takes a frame and asks for its header one
/// statement later.
#[test]
#[serial]
fn try_receive_one_owned_asking_for_the_header_parses_once() {
    let (mut publisher, sub, cursor) = pair("owned_header");
    publish(&mut publisher);

    reset_header_parse_count();
    let taken = sub.try_receive_one_owned().expect("an owned receive");
    let header = taken
        .as_ref()
        .expect("the frame just published must be taken")
        .wire_header();
    let parses = header_parse_count();

    assert_eq!(
        parses, 1,
        "a take whose caller asks for the header must cost ONE parse for the frame; {parses} \
         means the accessor re-read the thirty-two bytes the loop had parsed"
    );
    let header = header.expect("a frame a publisher wrote carries a parseable header");
    assert_eq!(
        header.schema_hash,
        Image::SCHEMA_HASH,
        "the header the accessor returned must be this frame's own, not a fabricated one"
    );
    assert_eq!(
        cursor.load(Ordering::Acquire),
        1,
        "the served cursor must have advanced past the one frame served, otherwise this arm \
         measured a path the cursor recorder never reached"
    );
}

/// The spin entry parses once in its own loop, and the view it builds carries that
/// header: a closure asking for it pays nothing further.
///
/// No cursor assertion here, because this entry records no read outcome at all. What
/// stands in for it is the header the closure read: a frame's own schema hash and a
/// nonzero publish stamp cannot come from a zeroed or fabricated header.
#[test]
#[serial]
fn spin_view_until_seq_asking_the_view_for_its_header_parses_once() {
    let (mut publisher, mut sub, _cursor) = pair("spin_header");
    publish(&mut publisher);

    reset_header_parse_count();
    let seen = sub
        .spin_view_until_seq::<Image, _>(0, Duration::from_secs(2), |view| view.wire_header())
        .expect("a spin read of the frame just published");
    let parses = header_parse_count();

    let header = seen.expect("the frame just published must be served within the timeout");
    assert_eq!(
        parses, 1,
        "a spin read whose closure asks for the header must cost ONE parse for the frame; \
         {parses} means the view parsed the bytes the spin loop had already parsed"
    );
    assert_eq!(
        header.schema_hash,
        Image::SCHEMA_HASH,
        "the header the accessor returned must be this frame's own, not a fabricated one"
    );
    assert!(
        header.timestamp_ns > 0,
        "the publisher stamps a publish time, so a zero here is a header nobody wrote"
    );
}
