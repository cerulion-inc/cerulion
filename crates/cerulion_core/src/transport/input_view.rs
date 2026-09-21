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
//! The view holds the iceoryx2 sample for its entire lifetime; once the
//! view drops, the SHM slot is released back to the publisher pool.

use std::marker::PhantomData;

use crate::message::ShmMessage;
use crate::wire::WireHeader;

use super::shm_sample::SampleHandle;

/// Zero-copy SHM-backed receive handle.
///
/// `'sample` is the lifetime of the held iceoryx2 inbound sample; the
/// SHM slot stays alive as long as the view is alive. `T: ShmMessage`
/// is the schema being read.
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
    /// Crate-private: only `CerulionSubscriber::try_view` may call this. A
    /// node reaches the view only as `self.<input>` inside `tick`.
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
    /// A view is constructed ONLY by `CerulionSubscriber::try_view`'s
    /// `build_inbound_view`, which validates the header (and its 32-byte
    /// length) BEFORE the view exists; the held-replay path serves that same
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
