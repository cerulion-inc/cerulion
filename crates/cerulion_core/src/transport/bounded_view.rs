// SPDX-License-Identifier: AGPL-3.0-only
//! The scheduler-bounded read of a frame a producer has just committed.
//!
//! Inside one process a producer's frame is already in a shared-memory slot by
//! the time its tick returns: the commit ran, the wire header is finalised, the
//! sequence is stamped and the sample is sent. A consumer in the same process
//! can therefore be handed a read-only view of exactly those bytes instead of
//! ending the producer's level, crossing a level boundary and receiving the
//! frame again out of its own queue.
//!
//! This module is that view, and nothing more. It opens no transport, receives
//! nothing, drains nothing and changes no execution path: the frames it can
//! serve are frames a caller already holds.
//!
//! # What this is NOT
//!
//! Not a general-purpose consumer read. The queue plane remains the one way a
//! consumer reads a topic it is subscribed to, and a standalone second read
//! path over it was considered on its own merits and refused. What is here
//! exists to be called from one place, between two fires of one linear chain
//! inside one process, and the scope discipline below is what keeps it there.
//!
//! Not a change to the publish, either. Nothing in this module can skip,
//! defer or reorder a commit; it reads bytes a commit already produced. Every
//! observer of the topic sees the same frames whether or not this view is ever
//! constructed.
//!
//! # The scope discipline, enforced by the type
//!
//! The borrow may not outlive the slot, may not be stored, and may not cross a
//! thread. Three mechanisms, none of them a comment:
//!
//! 1. **It is a borrow.** [`BoundedFrame`] holds `&[u8]` into the slot, so the
//!    compiler refuses any use that outlives the bytes.
//! 2. **Its lifetime cannot be named.** [`with_committed_frame`] is the only
//!    constructor and it is higher-ranked over the frame's lifetime, which is
//!    INVARIANT. A caller inside the closure cannot write down a type that
//!    mentions that lifetime, so there is nowhere outside the closure to put
//!    the frame, and no coercion to a longer lifetime to reach for.
//! 3. **It is `!Send` and `!Sync`.** A raw-pointer marker field, the same
//!    mechanism [`super::input_view::InputView`] uses, so the borrow cannot be
//!    smuggled onto another thread.
//!
//! The compile-fail doctests on [`BoundedFrame`] and [`with_committed_frame`]
//! are the CI-gated half of that: they assert the refusals HAPPEN. The
//! RENDERING is pinned by the `#[ignore]`d
//! `tests/ui/type_error/bounded_view_escapes_its_scope.rs` fixture, which
//! asserts rustc's own lifetime diagnostics and is therefore toolchain-fragile,
//! exactly like the other fixtures in that directory.
//!
//! # What serves the consumer
//!
//! [`BoundedFrame::serve_as`] builds the SAME [`InputView`] the receive path
//! builds, over the same validated payload range, and hands it to a closure.
//! A node reads its trigger input through `Deref` on that view, so node code is
//! byte-identical whichever side built it and nothing in the macro layer has to
//! know which one did.
//!
//! The frame checks are shared rather than copied: both constructions go
//! through one `validate_wire_frame` in [`super::input_view`], so a frame one
//! path refuses cannot be a frame the other accepts.

use std::marker::PhantomData;

use crate::error::TransportResult;
use crate::message::ShmMessage;
use crate::wire::WireHeader;

use super::input_view::{validate_wire_frame, InputView};
use super::shm_sample::SampleHandle;

/// The marker that makes a [`BoundedFrame`] invariant in its lifetime and
/// neither `Send` nor `Sync`.
///
/// `fn(&'brand ()) -> &'brand ()` puts `'brand` in both argument and return
/// position, which is exactly what makes it INVARIANT: neither a longer nor a
/// shorter lifetime may be substituted for it, so a holder cannot widen the
/// frame's scope. That invariance is what rustc cites when it refuses to let a
/// frame escape, and the refusal is recorded verbatim in
/// `tests/ui/type_error/bounded_view_escapes_its_scope.stderr`.
///
/// `*const ()` is the `!Send`/`!Sync` half, the same mechanism
/// [`super::input_view::InputView`] uses and for the same reason: a borrow of a
/// shared-memory slot must not leave the thread that holds the slot.
type ScopeBrand<'brand> = PhantomData<(fn(&'brand ()) -> &'brand (), *const ())>;

/// A read-only frame borrowed for the duration of one bounded serve.
///
/// `'brand` is a lifetime the holder cannot name: [`with_committed_frame`]
/// quantifies over it and [`PhantomData`] makes it invariant, so a frame can be
/// used inside the scope it was handed to and stored nowhere that outlives it.
///
/// # A frame cannot outlive its scope
///
/// ```compile_fail
/// use cerulion_core::transport::bounded_view::{with_committed_frame, BoundedFrame};
/// let frame_bytes = vec![0u8; 64];
/// // ERROR: the type of `escaped` would have to name the frame's lifetime,
/// // which `with_committed_frame` quantifies over and does not lend out.
/// let mut escaped: Option<BoundedFrame<'_>> = None;
/// with_committed_frame("/t", &frame_bytes, |frame| {
///     escaped = Some(frame);
/// });
/// ```
///
/// # A frame cannot cross a thread
///
/// ```compile_fail
/// use cerulion_core::transport::bounded_view::with_committed_frame;
/// let frame_bytes = vec![0u8; 64];
/// with_committed_frame("/t", &frame_bytes, |frame| {
///     // ERROR: `BoundedFrame` is `!Send`; the borrow may not leave this thread.
///     std::thread::spawn(move || {
///         let _ = frame.topic();
///     });
/// });
/// ```
///
/// # The ANTI-TAUTOLOGY twin: using it inside the scope compiles
///
/// ```
/// use cerulion_core::transport::bounded_view::with_committed_frame;
/// let frame_bytes = vec![0u8; 64];
/// let topic = with_committed_frame("/t", &frame_bytes, |frame| frame.topic().to_string());
/// assert_eq!(topic, "/t");
/// ```
pub struct BoundedFrame<'brand> {
    /// The topic the frame was committed on, for the refusal messages.
    topic: &'brand str,
    /// The WHOLE committed frame, header included, exactly as the slot holds
    /// it. Header-inclusive so [`InputView::wire_header`] reads the producer's
    /// own bytes rather than anything this module reconstructed.
    raw: &'brand [u8],
    /// Invariant in `'brand`, so the lifetime cannot be widened, and
    /// `!Send`/`!Sync`, so the borrow cannot leave the thread. See
    /// [`ScopeBrand`] for why that exact shape.
    _brand: ScopeBrand<'brand>,
}

impl<'brand> BoundedFrame<'brand> {
    /// The topic this frame was committed on.
    pub fn topic(&self) -> &str {
        self.topic
    }

    /// The frame's byte length, header included.
    pub fn len(&self) -> usize {
        self.raw.len()
    }

    /// Whether the frame carries no bytes at all, which no committed frame
    /// does. Present because a bare `len` invites the clippy lint that asks
    /// for it, and a reader of the pair should see that the answer is always
    /// false for a real frame.
    pub fn is_empty(&self) -> bool {
        self.raw.is_empty()
    }

    /// The frame's wire header, or the reason the frame is not one.
    ///
    /// Reads the producer's own 32 bytes. Refuses for the same reasons a serve
    /// refuses, through the same shared checks, so a caller that wants the
    /// header before deciding anything gets the same verdict the serve would
    /// give.
    pub fn wire_header<T: ShmMessage>(&self) -> TransportResult<WireHeader> {
        validate_wire_frame::<T>(self.topic, self.raw).map(|(header, _)| header)
    }

    /// Serve this frame as `T` through the ordinary input view.
    ///
    /// Validates the frame with the checks the receive path uses, builds
    /// `T::Reader` over `[32, total_size)` of the producer's own bytes, and
    /// hands the closure an [`InputView`] indistinguishable from one the queue
    /// path built. A node reads its fields through `Deref` on that view, so its
    /// code does not change and cannot tell which side served it.
    ///
    /// The view borrows `self`, so it dies with the closure and cannot outlive
    /// the frame, which cannot outlive the slot.
    ///
    /// Allocation-free: the view is a borrow and the reader is a borrow, so a
    /// serve does not allocate. That is a property the step's zero-allocation
    /// probe can hold this to once a caller exists.
    ///
    /// # Errors
    ///
    /// The frame is refused, and the closure never runs, when it is shorter
    /// than a header, when the header cannot be parsed, when its `schema_hash`
    /// is not `T::SCHEMA_HASH`, or when its `total_size` falls outside the
    /// frame. A schema mismatch means the two ends disagree about the message's
    /// fields, so the frame would be MISREAD rather than merely unfamiliar,
    /// which is why it is an error and not a warning.
    pub fn serve_as<T: ShmMessage, R>(
        &self,
        f: impl FnOnce(InputView<'_, T>) -> R,
    ) -> TransportResult<R> {
        let (_header, payload) = validate_wire_frame::<T>(self.topic, self.raw)?;
        let reader = T::build_reader(&self.raw[payload]);
        let view = InputView::new(SampleHandle::ScheduledBytes { bytes: self.raw }, reader);
        Ok(f(view))
    }
}

/// Hand a committed frame to `f` as a [`BoundedFrame`], and take it back.
///
/// The ONLY way to obtain a `BoundedFrame`. `f` is higher-ranked over the
/// frame's lifetime, so a caller cannot name it, cannot store the frame past
/// the call, and cannot hold a stale one: when `with_committed_frame` returns,
/// every `BoundedFrame` it created is gone, and so is every view served from
/// one.
///
/// `raw` is the WHOLE frame the producer committed, header included, borrowed
/// from the slot it lives in. The caller is what keeps the slot alive for the
/// call, which is why this is the scheduler-bounded read and not a general one:
/// the bound is the caller's scope, and the type makes that the only scope
/// there is.
///
/// # A view cannot outlive the serve that produced it
///
/// ```compile_fail
/// use cerulion_core::transport::bounded_view::with_committed_frame;
/// use cerulion_core::transport::input_view::InputView;
/// use native_ros2_messages::geometry_msgs::Vector3;
/// let frame_bytes = vec![0u8; 64];
/// with_committed_frame("/t", &frame_bytes, |frame| {
///     let mut escaped: Option<InputView<'_, Vector3>> = None;
///     // ERROR: the view borrows `frame`, whose lifetime cannot be named.
///     let _ = frame.serve_as::<Vector3, _>(|view| {
///         escaped = Some(view);
///     });
/// });
/// ```
///
/// # The ANTI-TAUTOLOGY twin: the same call, the same imports, compiling
///
/// A `compile_fail` block passes when compilation fails for ANY reason, a
/// missing import included, so each one here is paired with a block that
/// compiles and RUNS through the same surface. This one also states what a
/// zeroed frame is: not a frame of this schema, refused before the closure.
///
/// ```
/// use cerulion_core::transport::bounded_view::with_committed_frame;
/// use native_ros2_messages::geometry_msgs::Vector3;
/// let frame_bytes = vec![0u8; 64];
/// let refused = with_committed_frame("/t", &frame_bytes, |frame| {
///     frame.serve_as::<Vector3, _>(|_view| ()).is_err()
/// });
/// assert!(refused, "a zeroed frame carries no matching schema hash");
/// ```
pub fn with_committed_frame<R>(
    topic: &str,
    raw: &[u8],
    f: impl for<'brand> FnOnce(BoundedFrame<'brand>) -> R,
) -> R {
    f(BoundedFrame {
        topic,
        raw,
        _brand: PhantomData,
    })
}
