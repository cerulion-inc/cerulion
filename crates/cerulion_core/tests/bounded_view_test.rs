// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle vectors for the scheduler-bounded read of an already-committed frame.
//!
//! Every frame here is built BY HAND, byte by byte, from constants chosen in
//! the test. Nothing is compared against a second read of the same bytes: a
//! serve is asserted against the literal values that were written, so a view
//! that read the wrong sixteen bytes, or the right bytes twice, fails.
//!
//! Pure: no transport, no shared memory, no iceoryx2, so parallel-safe and
//! needing no serialization. That is itself a property of the thing under
//! test, which borrows bytes a caller already holds and opens nothing.
//!
//! The TYPE-level half of the discipline (a frame cannot escape its scope,
//! cannot cross a thread, and a view cannot outlive its serve) is pinned by the
//! compile-fail doctests on `transport::bounded_view`, which CI runs through
//! `cargo test -p cerulion_core --doc`, and its rendering by the `#[ignore]`d
//! `tests/ui/type_error/bounded_view_escapes_its_scope.rs` fixture. A runtime
//! test cannot express "this does not compile", so it is not attempted here.
//!
//! ```bash
//! cargo test -p cerulion_core --test bounded_view_test
//! ```

use cerulion_core::error::TransportError;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::bounded_view::{with_committed_frame, BoundedFrame};
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;

/// The three payload values every frame in this file carries, chosen so each
/// is distinct, none is zero, and none could be confused with a length, an
/// offset or a byte count if the reader picked up the wrong window.
const X: f64 = -1.5;
const Y: f64 = 2.25;
const Z: f64 = 1e9 + 0.5;

/// The sequence and stamp the header carries, chosen the same way.
const SEQUENCE: u32 = 0x0BAD_F00D;
const TIMESTAMP_NS: u64 = 1_700_000_000_123_456_789;

/// Hand-build the wire frame a producer's commit leaves in the slot for a
/// `Vector3`: a 32-byte header followed by three little-endian `f64`s.
///
/// `Vector3` is FIXED-size (no variable fields), so there is no offset table
/// and the payload IS the fixed section. The header is written through the
/// shipped `WireHeader` writer rather than a second hand-rolled encoder, so
/// this fixture cannot drift from the format the product writes; the PAYLOAD
/// bytes are laid down here explicitly, which is the half the view must read.
fn commit_vector3_frame() -> Vec<u8> {
    let payload_len = std::mem::size_of::<f64>() * 3;
    let mut frame = vec![0u8; WireHeader::SIZE + payload_len];

    let mut header = WireHeader::new(Vector3::SCHEMA_HASH, SEQUENCE, TIMESTAMP_NS);
    header.total_size = (WireHeader::SIZE + payload_len) as u32;
    header.offset_table_count = 0;
    header.offset_table_offset = 0;
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);

    frame[WireHeader::SIZE..WireHeader::SIZE + 8].copy_from_slice(&X.to_le_bytes());
    frame[WireHeader::SIZE + 8..WireHeader::SIZE + 16].copy_from_slice(&Y.to_le_bytes());
    frame[WireHeader::SIZE + 16..WireHeader::SIZE + 24].copy_from_slice(&Z.to_le_bytes());

    frame
}

// ==========================================================================
// The serve reads the committed bytes, and reads them right
// ==========================================================================

/// A bounded serve reads the frame a producer committed, field for field,
/// against the values the test wrote.
///
/// The header is asserted too, and separately: a view that served the right
/// payload off a fabricated header would be a view that lies about which frame
/// it is showing, which is the half an observer relies on.
#[test]
fn a_bounded_serve_reads_the_committed_frame_against_a_hand_oracle() {
    let frame = commit_vector3_frame();

    let (x, y, z, header) = with_committed_frame("/t/producer/out", &frame, |committed| {
        assert_eq!(committed.topic(), "/t/producer/out");
        assert_eq!(committed.len(), WireHeader::SIZE + 24);
        assert!(!committed.is_empty());
        committed
            .serve_as::<Vector3, _>(|view| (view.x, view.y, view.z, view.wire_header()))
            .expect("a well-formed Vector3 frame must serve")
    });

    // The payload, against the literals written above. Never against a second
    // read of the same bytes.
    assert_eq!(x, X, "x must be the committed value");
    assert_eq!(y, Y, "y must be the committed value");
    assert_eq!(z, Z, "z must be the committed value");

    // The header, against the same literals the producer's commit stamped.
    assert_eq!(header.schema_hash, Vector3::SCHEMA_HASH);
    assert_eq!(header.sequence, SEQUENCE, "the sequence is the producer's");
    assert_eq!(header.timestamp_ns, TIMESTAMP_NS, "so is the stamp");
    assert_eq!(header.total_size, (WireHeader::SIZE + 24) as u32);
    assert_eq!(header.offset_table_count, 0, "Vector3 is fixed-size");
}

/// The frame's header is readable WITHOUT serving it, and agrees with the
/// header the serve reports.
///
/// Two surfaces, one set of checks: a caller that wants to look before it
/// decides must not get a different answer from the one a serve would give.
#[test]
fn the_header_reads_the_same_whether_or_not_the_frame_is_served() {
    let frame = commit_vector3_frame();

    let (peeked, served) = with_committed_frame("/t/producer/out", &frame, |committed| {
        let peeked = committed
            .wire_header::<Vector3>()
            .expect("a well-formed frame has a readable header");
        let served = committed
            .serve_as::<Vector3, _>(|view| view.wire_header())
            .expect("and serves");
        (peeked, served)
    });

    assert_eq!(peeked, served, "one frame, one header, two surfaces");
    assert_eq!(peeked.sequence, SEQUENCE);
}

/// Two serves of one frame read the same values, because a serve consumes
/// nothing.
///
/// The queued path has a serve-many rule with real teeth: a frozen trigger
/// head is CONSUMED by its serve, and a second serve of it would be a bug.
/// A bounded frame is a borrow of bytes the caller owns, so it has no slot to
/// consume and no accounting to run, and serving twice is simply reading twice.
/// This pins that difference rather than assuming it.
#[test]
fn serving_one_frame_twice_consumes_nothing() {
    let frame = commit_vector3_frame();

    with_committed_frame("/t/producer/out", &frame, |committed| {
        let first = committed
            .serve_as::<Vector3, _>(|view| view.x)
            .expect("first serve");
        let second = committed
            .serve_as::<Vector3, _>(|view| view.x)
            .expect("second serve");
        // Both against the LITERAL, not against each other: two reads that
        // agreed on the wrong value would pass a self-comparison.
        assert_eq!(first, X);
        assert_eq!(second, X);
    });
}

// ==========================================================================
// A frame that is not one is refused, and no view is ever handed out
// ==========================================================================

/// Every refusal class refuses, names itself, and runs no closure.
///
/// The "runs no closure" half is the runtime shape of "a stale view is
/// unobtainable": there is no path on which a caller holds a view over bytes
/// the checks rejected, because the closure that would receive it is never
/// called. A flag proves that rather than an argument.
#[test]
fn a_frame_that_is_not_one_is_refused_and_serves_no_view() {
    let good = commit_vector3_frame();

    // (label, frame bytes, what the refusal must be)
    let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();

    // Shorter than a header: there is nothing to read a header out of.
    cases.push(("undersized", good[..WireHeader::SIZE - 1].to_vec()));

    // A header whose schema hash is another schema's: the two ends disagree
    // about the message's fields, so the frame would be MISREAD.
    let mut wrong_schema = good.clone();
    let mut header = WireHeader::read_from_buf(&wrong_schema).expect("header");
    header.schema_hash = Vector3::SCHEMA_HASH ^ 0xFFFF_FFFF_FFFF_FFFF;
    header.write_to_buf(&mut wrong_schema[..WireHeader::SIZE]);
    cases.push(("schema mismatch", wrong_schema));

    // `total_size` past the frame: a reader over it would read bytes the
    // producer never wrote.
    let mut too_long = good.clone();
    let mut header = WireHeader::read_from_buf(&too_long).expect("header");
    header.total_size = (too_long.len() + 1) as u32;
    header.write_to_buf(&mut too_long[..WireHeader::SIZE]);
    cases.push(("total_size past the frame", too_long));

    // `total_size` below the header: the same, in the other direction.
    let mut too_short = good.clone();
    let mut header = WireHeader::read_from_buf(&too_short).expect("header");
    header.total_size = (WireHeader::SIZE - 1) as u32;
    header.write_to_buf(&mut too_short[..WireHeader::SIZE]);
    cases.push(("total_size below the header", too_short));

    // A header-only frame: well-formed as a header, but with no room for the
    // fields the schema names. The generated reader ASSERTS on a buffer this
    // short, and an assert in a reader is a panic in a node body, so this must
    // be a refusal and not a crash.
    let mut header_only = good.clone();
    let mut header = WireHeader::read_from_buf(&header_only).expect("header");
    header.total_size = WireHeader::SIZE as u32;
    header.write_to_buf(&mut header_only[..WireHeader::SIZE]);
    cases.push(("header-only frame", header_only));

    for (label, bytes) in cases {
        let mut closure_ran = false;
        let outcome = with_committed_frame("/t/producer/out", &bytes, |committed| {
            committed.serve_as::<Vector3, _>(|_view| {
                closure_ran = true;
            })
        });
        assert!(
            outcome.is_err(),
            "{label}: a frame that fails the checks must be refused"
        );
        assert!(
            !closure_ran,
            "{label}: no view may be handed out for a refused frame"
        );
        let err = outcome.expect_err("refused");
        match (&label, &err) {
            (&"schema mismatch", TransportError::SchemaMismatch { topic, .. }) => {
                assert_eq!(topic, "/t/producer/out", "the refusal names the topic");
            }
            (_, TransportError::Deserialization { topic, .. }) => {
                assert_eq!(topic, "/t/producer/out", "the refusal names the topic");
            }
            other => panic!("{label}: unexpected refusal {other:?}"),
        }
    }

    // ANTI-TAUTOLOGY: the unmodified frame serves, so the arms above are about
    // the specific damage rather than about a serve that always refuses.
    let served = with_committed_frame("/t/producer/out", &good, |committed| {
        committed.serve_as::<Vector3, _>(|view| view.x)
    });
    assert_eq!(served.expect("the good frame serves"), X);
}

/// The reader's window is the FRAME, not the slot, and a frame whose header
/// claims less than its schema needs is refused even when the slot has the
/// bytes to spare.
///
/// This is the arm that can tell the two apart. A slot is sized for the largest
/// frame a topic can carry, so trailing bytes from a previous, longer frame are
/// the normal case; but for a FIXED schema, serving `raw[32..]` and serving
/// `raw[32..total_size]` read the same three fields, so a tail alone proves
/// nothing. A header claiming a payload too short for the schema does prove it:
/// bounded by the frame the serve is refused, bounded by the slot it would read
/// the tail and answer.
#[test]
fn the_readers_window_is_the_frame_and_not_the_slot() {
    let good = commit_vector3_frame();

    // The same frame in a slot with room to spare, its header claiming a
    // payload of 8 bytes where `Vector3` needs 24. Bounded by `total_size`
    // this is refused; bounded by the slot it would serve the tail.
    let mut short_claim = good.clone();
    short_claim.extend(std::iter::repeat_n(0xAB, good.len()));
    let mut header = WireHeader::read_from_buf(&short_claim).expect("header");
    header.total_size = (WireHeader::SIZE + 8) as u32;
    header.write_to_buf(&mut short_claim[..WireHeader::SIZE]);

    let mut closure_ran = false;
    let outcome = with_committed_frame("/t/producer/out", &short_claim, |committed| {
        assert!(
            committed.len() > WireHeader::SIZE + 24,
            "the slot really does hold enough bytes to serve a Vector3"
        );
        committed.serve_as::<Vector3, _>(|_view| {
            closure_ran = true;
        })
    });
    assert!(
        outcome.is_err(),
        "a header claiming less than the schema needs must be refused"
    );
    assert!(!closure_ran, "and must serve no view");

    // ANTI-TAUTOLOGY: the same oversized slot with a header that MATCHES its
    // frame serves, and serves the committed values rather than the tail.
    let (x, y, z) = with_committed_frame(
        "/t/producer/out",
        &{
            let mut padded = good.clone();
            padded.extend(std::iter::repeat_n(0xAB, good.len()));
            padded
        },
        |committed| {
            committed
                .serve_as::<Vector3, _>(|view| (view.x, view.y, view.z))
                .expect("a matching header in an oversized slot serves")
        },
    );
    assert_eq!(x, X, "the payload is the committed one, not the tail");
    assert_eq!(y, Y);
    assert_eq!(z, Z);
}

/// A frame in an oversized slot reports its OWN length, not the slot's.
#[test]
fn a_frame_in_an_oversized_slot_reports_its_own_length() {
    let good = commit_vector3_frame();

    // The same frame in a slot twice its size, the tail filled with a byte
    // pattern that would be visible as nonsense if it were read as payload.
    let mut oversized_slot = good.clone();
    oversized_slot.extend(std::iter::repeat_n(0xAB, good.len()));

    let (x, y, z, total_size) = with_committed_frame("/t/producer/out", &oversized_slot, |c| {
        assert_eq!(
            c.len(),
            good.len() * 2,
            "the slot really is longer than the frame"
        );
        c.serve_as::<Vector3, _>(|view| (view.x, view.y, view.z, view.wire_header().total_size))
            .expect("a frame in an oversized slot still serves")
    });

    assert_eq!(x, X, "the payload is the committed one");
    assert_eq!(y, Y);
    assert_eq!(z, Z);
    assert_eq!(
        total_size,
        (WireHeader::SIZE + 24) as u32,
        "the frame's own length, not the slot's"
    );
}

// ==========================================================================
// The queued path is untouched
// ==========================================================================

/// The bounded path reaches none of the subscriber state the queued path keeps.
///
/// This is a STRUCTURAL pin, and it is what a test can check: the bounded serve
/// takes a topic and a byte slice and nothing else, so it has no subscriber to
/// touch, no frozen slot to take, no held sample to borrow and no accounting to
/// run. A signature that grew a subscriber argument would fail to compile here,
/// which is the point.
///
/// The queued path's own behaviour is pinned where it lives, by the receive and
/// serve-many suites; duplicating those assertions here against a path that
/// cannot reach them would be theatre.
/// A NAMED function, which is where the surface is spelled out: it takes a
/// frame and nothing else, and it is higher-ranked over the frame's lifetime
/// because a `fn` item always is. A serve that needed a subscriber, a slot or
/// any scheduler state could not be written this way.
fn reads_the_committed_x(committed: BoundedFrame<'_>) -> bool {
    committed
        .serve_as::<Vector3, _>(|view| view.x == X)
        .unwrap_or(false)
}

#[test]
fn the_bounded_serve_takes_only_bytes_and_a_topic() {
    let frame = commit_vector3_frame();
    // Passing a named `fn` rather than a closure is the pin: the signature
    // above is the whole surface, and it mentions no subscriber.
    assert!(with_committed_frame(
        "/t/producer/out",
        &frame,
        reads_the_committed_x
    ));
}

/// A frame the wire format would never place at that address is refused, not
/// read.
///
/// The generated reader reinterprets the payload bytes in place, so it ASSERTS
/// on a misaligned start. An assert in a reader is a panic in a node body, and
/// a node body must never panic through an accessor, so a misaligned frame is a
/// refusal with a reason.
///
/// A publisher's slot is always aligned, which is why this shape only arises
/// from a caller handing over a slice of its own; the check is what makes that
/// a typed error rather than a crash.
#[test]
fn a_misaligned_frame_is_refused_rather_than_read() {
    let good = commit_vector3_frame();

    // One byte of lead-in, then the frame, so the frame starts at an odd
    // address inside an allocation whose own start is aligned.
    let mut shifted = vec![0u8; 1];
    shifted.extend_from_slice(&good);
    let frame = &shifted[1..];
    assert_eq!(frame.len(), good.len(), "the same frame, one byte over");

    if frame.as_ptr().align_offset(8) == 0 {
        // The allocator happened to hand back a base that leaves the shifted
        // slice aligned anyway. Refusing to assert on an accident is better
        // than asserting on one.
        return;
    }

    let mut closure_ran = false;
    let outcome = with_committed_frame("/t/producer/out", frame, |committed| {
        committed.serve_as::<Vector3, _>(|_view| {
            closure_ran = true;
        })
    });
    assert!(outcome.is_err(), "a misaligned frame must be refused");
    assert!(!closure_ran, "and must serve no view");

    // ANTI-TAUTOLOGY: the SAME bytes at an aligned start serve, so the arm
    // above is about the address and not about the frame.
    let served = with_committed_frame("/t/producer/out", &good, |committed| {
        committed.serve_as::<Vector3, _>(|view| view.x)
    });
    assert_eq!(served.expect("the aligned frame serves"), X);
}
