// SPDX-License-Identifier: AGPL-3.0-only
//! The openh264 fetch WIRING pin: the decode path really does tell the fetcher when a
//! decoder was wanted and there was none.
//!
//! # Why this needs its own test binary
//!
//! [`cerulion_viz::openh264_fetch::consultations`] is a PROCESS-GLOBAL counter,
//! and several tests in `video_decode_test.rs` drive pools with no decoder — so
//! sharing a binary with them would leave this arm asserting a delta other tests
//! could satisfy on its behalf. If the
//! `note_decoder_needed()` call were deleted from `VideoDecoders::decode`, a sibling's
//! bumps would carry a `>=` assertion straight past it: a false pass, on the one
//! test whose whole job is to notice.
//!
//! A separate integration test file is a separate PROCESS, so the count here is
//! exact and nothing else can contribute to it. One test, deliberately.
//!
//! # What it does NOT claim
//!
//! It does not claim a fetch happened. This crate's tests build with
//! `decoder-from-source`, so `fetch_enabled()` is false and the gate answers
//! `Disabled` — no socket is opened, which is exactly what keeps CI hermetic.
//! The claim is narrower and is the one that would otherwise go unchecked:
//! **the call site exists and is reached.** Whether the fetch then SUCCEEDS is
//! the network-only `openh264_live_fetch_test.rs`, and whether the gate decides
//! correctly is `decide_fetch_kick`'s oracle vectors.

use cerulion_viz::openh264_fetch::{consultations, FetchGate};
use cerulion_viz::video::StreamKey;
use cerulion_viz::video_decode::{DecodeOutcome, VideoDecoders};

/// The rendition key is irrelevant to this pin — any one will do, since nothing
/// here decodes.
const KEY: StreamKey = StreamKey {
    width: 320,
    height: 240,
};

/// Deleting `crate::openh264_fetch::note_decoder_needed();`
/// from `VideoDecoders::decode`'s `Backend::Unavailable` arm fails this test with
/// a consultation count of 0.
#[test]
fn a_decode_with_no_decoder_tells_the_fetcher_one_was_needed() {
    // Nothing in this process has consulted the fetcher yet.
    assert_eq!(
        consultations(),
        0,
        "this binary holds ONE test, so the counter must start clean — if it does \
         not, another test was added here and the exact counts below are no longer \
         sound"
    );

    let mut pool = VideoDecoders::unavailable_pending_fetch_for_test("test: no blob cached");
    assert!(!pool.is_available());

    // Two access units, neither decodable on this pool. The bytes do not matter:
    // the whole-desk `Unavailable` arm returns before any decoder is touched.
    for _ in 0..2 {
        assert_eq!(
            pool.decode("/probe/wiring/h264", KEY, &[0u8, 0, 0, 1, 0x67], 0),
            DecodeOutcome::DecoderUnavailable
        );
    }

    // EXACTLY the two consultations our two units produced.
    assert_eq!(
        consultations(),
        2,
        "every access unit that finds no decoder must tell the fetcher — this is \
         the only observable that the decode path is wired to the fetcher at all"
    );

    // And in THIS build the fetcher declines, which is why no socket was opened.
    // Asserted so the hermetic-by-construction claim is checked rather than
    // assumed: if `decoder-from-source` ever stopped disabling the fetch, CI
    // would start reaching Cisco's CDN and this arm says so first.
    assert_eq!(
        cerulion_viz::openh264_fetch::note_decoder_needed(),
        FetchGate::Disabled,
        "a decoder-from-source build must never start a fetch"
    );
    assert_eq!(consultations(), 3, "the direct call counts too");
    assert!(
        !cerulion_viz::openh264_fetch::status().running,
        "a disabled fetcher must not have spawned anything"
    );
}
