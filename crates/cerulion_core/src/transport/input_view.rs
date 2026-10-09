// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-copy SHM-backed input view.
//!
//! `InputView<'sample, T: ShmMessage>` is the read-side counterpart to
//! `OutputProxy`. It wraps an inbound iceoryx2 sample together with a
//! `T::Reader<'sample>` (the SHM-backed `<Name>Shm` type) and lets
//! the user access fields directly off shared memory through `Deref`.
//!
//! A node author never constructs one. Inside a node's `tick`, an
//! `#[input] scan: LaserScan` field IS this view: fixed fields are read as
//! fields (`self.scan.range_min`), variable-length fields through accessor
//! methods (`self.scan.ranges()`), and `self.scan.wire_timestamp_ns()` is the
//! publish time of the frame being read (for a held latest-value input, the
//! held frame's original stamp, which is its freshness).
//!
//! A view built by the receive path holds the iceoryx2 sample for its entire
//! lifetime; once the view drops, the SHM slot is released back to the
//! publisher pool. A view built by [`super::bounded_view`] instead borrows
//! bytes its caller already holds and owns no sample, so it releases nothing:
//! the caller's own borrow is what keeps the slot alive. Which one a view is
//! shows in its `SampleHandle` variant and nowhere else,
//! which is the point, because a node reads both the same way.

use std::marker::PhantomData;

use crate::message::ShmMessage;
use crate::wire::WireHeader;

use super::shm_sample::SampleHandle;

/// Zero-copy SHM-backed receive handle.
///
/// `'sample` is the lifetime of whatever keeps the bytes alive: the held
/// iceoryx2 inbound sample on the receive path, or the caller's borrow of an
/// already-committed slot on the bounded path. Either way the bytes outlive the
/// view. `T: ShmMessage` is the schema being read.
///
/// # Send / Sync
///
/// `InputView` is intentionally `!Send` and `!Sync`, enforced
/// by the `_not_send: PhantomData<*const ()>` marker field. User code
/// holding a view inside a `tick(&mut self)` body cannot accidentally
/// smuggle the borrow across thread boundaries.
///
/// NOTE: the marker is the SOLE enforcer. Before the thread-safe service swap the view was
/// ALSO `!Send` because iceoryx2 inbound samples were `!Send` (`ipc::Service`,
/// Rc-backed); the swap to `ipc_threadsafe::Service` made samples `Send`,
/// so do NOT remove the `PhantomData` marker as "redundant".
//
// `sample` is held to keep the SHM slot alive; `reader` is the accessor
// that user code reaches via Deref.
pub struct InputView<'sample, T: ShmMessage + 'sample> {
    /// Held iceoryx2 inbound sample. Released on drop.
    sample: SampleHandle<'sample>,

    /// SHM-backed reader — the codegen-emitted `<Name><'sample>` type.
    /// Constructed from `&'sample [u8]` over `[32..total_size]` of the
    /// sample's payload (after the 32-byte WireHeader).
    reader: T::Reader<'sample>,

    /// `InputView` must be `!Send` and `!Sync`.
    _not_send: PhantomData<*const ()>,
}

impl<'sample, T: ShmMessage + 'sample> InputView<'sample, T> {
    /// Construct an `InputView` from an inbound sample and a reader over
    /// its payload.
    ///
    /// Crate-private, with exactly two callers: `CerulionSubscriber::try_view`
    /// on the receive path and [`super::bounded_view::BoundedFrame::serve_as`]
    /// on the scheduler-bounded one. A node reaches the view only as
    /// `self.<input>` inside `tick`, and cannot tell which caller built it.
    ///
    /// The caller is responsible for validating that the sample's
    /// WireHeader's `schema_hash` matches `T::SCHEMA_HASH`. Mismatch
    /// returns `TransportError::SchemaMismatch` from the `try_view`
    /// boundary.
    #[allow(dead_code)] // Constructor used by CerulionSubscriber::try_view.
    pub(crate) fn new(sample: SampleHandle<'sample>, reader: T::Reader<'sample>) -> Self {
        Self {
            sample,
            reader,
            _not_send: PhantomData,
        }
    }

    /// Access the held SHM bytes directly (excludes WireHeader).
    ///
    /// Most user code reaches the message via `Deref` to `T::Reader<'_>`;
    /// this escape hatch exists for tests and replay infrastructure that
    /// need the raw payload.
    #[allow(dead_code)] // Used by replay/codegen tests.
    pub(crate) fn sample_bytes(&self) -> &[u8] {
        self.sample.bytes()
    }

    /// The full [`WireHeader`] of the frame this view serves.
    ///
    /// Parses the 32-byte header off the front of the held SHM frame.
    /// `self.sample.bytes()` returns the ENTIRE frame INCLUDING the header —
    /// for a live sample AND for the held-replay path (which serves
    /// the subscriber-retained frame via `SampleHandle`'s `InboundRef`
    /// variant) — so for a held latest-value input the header reads the HELD
    /// frame's ORIGINAL values, unchanged across silent steps.
    ///
    /// # Infallibility (why the impossible arm does not panic)
    ///
    /// Every view is constructed behind `validate_wire_frame`, which parses
    /// the header and bounds the payload BEFORE the view exists: on the receive
    /// path through `build_inbound_view`, on the scheduler-bounded path through
    /// `BoundedFrame::serve_as`, and the held-replay path serves that same
    /// already-validated frame. So `read_from_buf` cannot return `None` in
    /// practice. On the by-construction-impossible short-frame arm we do NOT
    /// silently fabricate a header: a `debug_assert!` fires loudly in dev and
    /// a `tracing::error!` records it in prod, then a zeroed header is
    /// returned — a node body must never PANIC through this accessor.
    pub fn wire_header(&self) -> WireHeader {
        let bytes = self.sample.bytes();
        // Loud in dev on the by-construction-impossible short frame (the ONLY
        // input that makes `read_from_buf` return `None`). In release this is
        // compiled out and the `unwrap_or_else` below logs + returns a zeroed
        // header — a node body must never PANIC through this accessor.
        debug_assert!(
            bytes.len() >= WireHeader::SIZE,
            "InputView::wire_header: frame ({} bytes) shorter than \
             WireHeader::SIZE ({}) — the view was built from a header-validated \
             sample, so this is impossible",
            bytes.len(),
            WireHeader::SIZE,
        );
        WireHeader::read_from_buf(bytes).unwrap_or_else(|| {
            tracing::error!(
                frame_len = bytes.len(),
                wire_header_size = WireHeader::SIZE,
                "InputView::wire_header could not parse a WireHeader from a \
                 header-validated sample; returning a zeroed header — this is a bug"
            );
            // All fields zeroed (schema_hash / sequence / timestamp_ns = 0).
            WireHeader::new(0, 0, 0)
        })
    }

    /// The wire-header publish timestamp (`timestamp_ns`) of the frame this
    /// view serves — for a held latest-value input this is the HELD frame's
    /// original stamp, which is exactly its freshness (staleness
    /// arbitration compares this against the node's clock).
    ///
    /// Thin projection over [`Self::wire_header`] (one parse of the same 32
    /// bytes; no extra cost). On the impossible short-frame arm it returns
    /// `0` after the loud `wire_header` diagnostic — never panicking.
    pub fn wire_timestamp_ns(&self) -> u64 {
        self.wire_header().timestamp_ns
    }
}

impl<'sample, T: ShmMessage + 'sample> std::ops::Deref for InputView<'sample, T> {
    type Target = T::Reader<'sample>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.reader
    }
}

/// The wire-frame checks every view construction shares, and the payload range
/// they prove is in bounds.
///
/// ONE copy of the rule. Two call sites build an [`InputView`]: the receive
/// path, which borrows an iceoryx2 sample, and the scheduler-bounded path in
/// [`super::bounded_view`], which borrows bytes a producer has just committed.
/// Both must refuse the same frames for the same reasons, and a second copy of
/// four bounds checks is exactly how one of them ends up accepting a frame the
/// other rejects.
///
/// Returns the parsed header and the payload range `[WireHeader::SIZE,
/// total_size)` within `raw`, which the caller may slice without further
/// checks. Refuses, in this order:
///
/// * a frame shorter than the 32-byte header, which has no header to read;
/// * a header the reader cannot parse;
/// * a `schema_hash` that is not `T::SCHEMA_HASH`, which means the two ends
///   disagree about the message's fields and the frame would be MISREAD rather
///   than merely unfamiliar;
/// * a `total_size` below the header or past the frame. Checked explicitly
///   rather than clamped: a `.max(WireHeader::SIZE)` coercion of a malformed
///   size would silently serve a reader over bytes the producer never wrote.
pub(crate) fn validate_wire_frame<T: ShmMessage>(
    topic: &str,
    raw: &[u8],
) -> crate::error::TransportResult<(WireHeader, std::ops::Range<usize>)> {
    use crate::error::TransportError;

    let (header, payload) = validate_wire_frame_raw(topic, raw, Some(T::SCHEMA_HASH))?;
    let total_size = payload.end;
    // The reader `T::build_reader` returns REINTERPRETS the payload bytes, so
    // it ASSERTS on a buffer too small for the fixed section and on one whose
    // start is misaligned. An assert in a reader is a panic in a node body,
    // and a node body must never panic through an accessor, so both conditions
    // are refused HERE with a reason instead.
    //
    // A frame a publisher wrote always satisfies both, which is why the frames
    // in front of these checks are the malformed ones: a header claiming a
    // `total_size` with no room for the fields it names, and a slice a caller
    // handed over at an address the wire format does not put frames at.
    let required = crate::wire::frame_prefix_size(T::WIRE_FIXED_SIZE, T::VARIABLE_FIELD_COUNT)
        .ok_or_else(|| TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "schema fixed size {} with {} variable field(s) overflows a wire frame",
                T::WIRE_FIXED_SIZE,
                T::VARIABLE_FIELD_COUNT,
            ),
        })?;
    if total_size < required {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "wire header total_size {total_size} leaves no room for the schema: a {} \
                 byte fixed section plus {} offset table entry/entries needs {required}",
                T::WIRE_FIXED_SIZE,
                T::VARIABLE_FIELD_COUNT,
            ),
        });
    }
    // 8 is the wire format's own alignment: [`WireHeader`] is
    // `#[repr(C, align(8))]`, the header is exactly 32 bytes so a payload
    // starts 8-aligned whenever the frame does, and 8 bytes is the widest
    // primitive a generated fixed section holds.
    const WIRE_ALIGN: usize = 8;
    if raw.as_ptr().align_offset(WIRE_ALIGN) != 0 {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "frame is not {WIRE_ALIGN}-byte aligned, which the wire format requires \
                 and a zero-copy reader cannot work around"
            ),
        });
    }
    Ok((header, payload))
}

/// The untyped core of [`validate_wire_frame`]: the checks that need no
/// schema type, shared with the raw read path (`CerulionSubscriber::view_raw`)
/// so a frame the typed path refuses is refused by the raw path too, for the
/// same reason and with the same words.
///
/// Parses the 32-byte header exactly once and returns it with the payload
/// range `[WireHeader::SIZE, total_size)`; every later consumer of the header
/// on this read (the service cursor, the host that needs the schema hash)
/// takes this copy instead of re-reading the bytes. Refuses, in this order:
///
/// * a frame shorter than the header;
/// * a header the reader cannot parse;
/// * when `expected_schema_hash` is given, a `schema_hash` that differs
///   from it. A raw read that passes `None` accepts any schema and leaves the
///   hash to its caller;
/// * a `total_size` below the header or past the frame.
pub(crate) fn validate_wire_frame_raw(
    topic: &str,
    raw: &[u8],
    expected_schema_hash: Option<u64>,
) -> crate::error::TransportResult<(WireHeader, std::ops::Range<usize>)> {
    use crate::error::TransportError;

    if raw.len() < WireHeader::SIZE {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "undersized message: {} bytes, need at least {}",
                raw.len(),
                WireHeader::SIZE,
            ),
        });
    }
    let header = WireHeader::read_from_buf(raw).ok_or_else(|| TransportError::Deserialization {
        topic: topic.to_string(),
        reason: "failed to parse WireHeader from received message".to_string(),
    })?;
    if let Some(expected_hash) = expected_schema_hash {
        if header.schema_hash != expected_hash {
            return Err(TransportError::SchemaMismatch {
                topic: topic.to_string(),
                expected_hash,
                actual_hash: header.schema_hash,
            });
        }
    }
    let total_size = header.total_size as usize;
    if total_size < WireHeader::SIZE || total_size > raw.len() {
        return Err(TransportError::Deserialization {
            topic: topic.to_string(),
            reason: format!(
                "wire header total_size {} out of bounds (frame {} bytes, header {} bytes)",
                total_size,
                raw.len(),
                WireHeader::SIZE,
            ),
        });
    }
    Ok((header, WireHeader::SIZE..total_size))
}
