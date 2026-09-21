// SPDX-License-Identifier: AGPL-3.0-only
//! Opaque iceoryx2 sample handle.
//!
//! `SampleHandle<'a>` wraps either an inbound `Sample` (held by `InputView`)
//! or an outbound `SendableSample` (held by `OutputProxy`). The lifetime
//! `'a` ties the borrowed bytes back to the iceoryx2 port that owns the
//! shared-memory slot.
//!
//! Construction is private to the transport module: only `try_view` and
//! `loan_proxy` (in `subscriber.rs` / `publisher.rs`) can produce one.
//! This is by design — user code never names this type, it only flows
//! through the proxy/view wrappers.
//!
//! # Why an enum
//!
//! Inbound and outbound iceoryx2 samples have different generic parameters
//! and method surfaces (`receive()` produces `Sample<S, [u8], ()>`,
//! `loan_slice_uninit()` produces `SampleMutUninit<S, [u8], ()>`). The
//! proxy / view types only need to know "I'm holding *some* iceoryx2 slot
//! that I will release on drop"; the enum captures that without leaking
//! the iceoryx2 type machinery into the public surface.
//!
//! # Lifetime story
//!
//! The contained iceoryx2 sample owns a refcount on the shared-memory slot.
//! Dropping `SampleHandle` releases the refcount. While the handle is
//! alive, the slot bytes are stable, which is what makes `&[u8]`
//! references derived from it valid for the handle's lifetime.

use std::marker::PhantomData;

use iceoryx2::sample::Sample;
use iceoryx2::sample_mut::SampleMut;

use super::output_discard_latch::DiscardLogLevel;
use super::publisher::CerulionPublisher;
use super::CerService;

/// Opaque wrapper around an iceoryx2 sample (inbound or outbound).
///
/// See module docs. `'a` is the lifetime of the iceoryx2 port that vended
/// the sample; the held shared-memory bytes are valid for `'a`.
///
/// The variants are `pub(crate)` because *constructing* one requires
/// access to the underlying iceoryx2 ports (held by `CerulionPublisher` /
/// `CerulionSubscriber`), which are themselves crate-private to construct.
pub(crate) enum SampleHandle<'a> {
    /// Inbound iceoryx2 sample held by `InputView` for the duration of the
    /// user's `try_view` callback.
    Inbound {
        sample: Sample<CerService, [u8], ()>,
        _phantom: PhantomData<&'a [u8]>,
    },
    /// A borrowed inbound sample, held by the SUBSCRIBER across
    /// steps (`held_sample`) and lent to `InputView` for the duration of a
    /// `try_view` callback. Unlike `Inbound`, this variant owns nothing —
    /// the `Sample` (and its SHM refcount) lives in the subscriber's
    /// `held_sample` field, which outlives the view. Used to serve a
    /// non-trigger latest-value input's HELD value without re-draining.
    InboundRef {
        sample: &'a Sample<CerService, [u8], ()>,
    },
    /// Outbound (initialized) iceoryx2 sample held by `OutputProxy` between
    /// loan and drop. On drop the proxy `take()`s the sample and calls
    /// `send()` on it. The `Option` wrapper enables that move out of
    /// `&mut self`.
    Outbound {
        sample: Option<SampleMut<CerService, [u8], ()>>,
        _phantom: PhantomData<&'a mut [u8]>,
    },
}

impl<'a> SampleHandle<'a> {
    /// Borrow the sample bytes immutably.
    ///
    /// Returns the entire loaned/received payload slice, including the
    /// 32-byte WireHeader at the front.
    #[inline]
    pub(crate) fn bytes(&self) -> &[u8] {
        match self {
            SampleHandle::Inbound { sample, .. } => sample.payload(),
            SampleHandle::InboundRef { sample } => sample.payload(),
            SampleHandle::Outbound { sample, .. } => sample
                .as_ref()
                .expect("Outbound SampleHandle accessed after take")
                .payload(),
        }
    }

    /// Borrow the sample bytes mutably.
    ///
    /// Only valid for outbound variants (loaned from a publisher); panics
    /// on the inbound variants. The proxy/view types never reach the
    /// wrong-direction branch by construction (`InputView` is built with
    /// inbound, `OutputProxy` with outbound).
    #[inline]
    pub(crate) fn bytes_mut(&mut self) -> &mut [u8] {
        match self {
            SampleHandle::Outbound { sample, .. } => sample
                .as_mut()
                .expect("Outbound SampleHandle accessed after take")
                .payload_mut(),
            SampleHandle::Inbound { .. } | SampleHandle::InboundRef { .. } => {
                panic!("SampleHandle::bytes_mut called on inbound variant — this is a bug")
            }
        }
    }

    /// Take the iceoryx2 outbound sample out of the handle, leaving `None`
    /// in place. Used by `OutputProxy::Drop` to call `send()` (which
    /// consumes the sample by value).
    ///
    /// Returns `None` for any non-iceoryx2-outbound variant or for an
    /// already-taken outbound handle.
    #[inline]
    pub(crate) fn take_outbound(&mut self) -> Option<SampleMut<CerService, [u8], ()>> {
        match self {
            SampleHandle::Outbound { sample, .. } => sample.take(),
            _ => None,
        }
    }
}

/// Borrow of the publisher backend that vended a `SampleHandle`.
///
/// `OutputProxy` holds one of these for the loan's lifetime so its `Drop`
/// implementation can call back into the right publisher to deliver the
/// finalized wire frame (iceoryx2 `send()` + notifier). There is no
/// dynamic dispatch on the per-tick hot path; the variant is matched once
/// at drop time.
pub(crate) enum ProxyPublisher<'loan> {
    /// iceoryx2 IPC publisher. Drop calls `send()` on the outbound sample
    /// and notifies subscribers via the publisher's `Notifier`.
    Iceoryx2(&'loan mut CerulionPublisher),
}

impl<'loan> ProxyPublisher<'loan> {
    /// Topic name of the underlying publisher.
    #[inline]
    pub(crate) fn topic(&self) -> &str {
        match self {
            ProxyPublisher::Iceoryx2(p) => p.topic(),
        }
    }

    /// Consume the next wire sequence number at COMMIT time —
    /// delegates to [`CerulionPublisher::commit_sequence`]. Called ONLY by
    /// `OutputProxy::Drop`'s two send paths (steady-state + overflow),
    /// which stamp the returned value into the header at `[20..24]` before
    /// handing the frame to iceoryx2.
    #[inline]
    pub(crate) fn commit_sequence(&self) -> u32 {
        match self {
            ProxyPublisher::Iceoryx2(p) => p.commit_sequence(),
        }
    }

    /// RECORD side: bump the underlying publisher's node discard signal,
    /// called by `OutputProxy::Drop` on the all-defer discard (a
    /// pre-commit release without `commit_sequence`). Delegates to
    /// [`CerulionPublisher::bump_discard_signal`] (no-op unless the runtime
    /// wired a signal).
    #[inline]
    pub(crate) fn bump_discard_signal(&self) {
        match self {
            ProxyPublisher::Iceoryx2(p) => p.bump_discard_signal(),
        }
    }

    /// REPLAY side: whether replay is suppressing this fire's publishes,
    /// read by `OutputProxy::Drop` FIRST (before `commit_sequence`) so a marked
    /// fire returns without publishing, the byte-identical mirror of the live
    /// discard. Delegates to [`CerulionPublisher::replay_suppress_active`].
    #[inline]
    pub(crate) fn replay_suppress_active(&self) -> bool {
        match self {
            ProxyPublisher::Iceoryx2(p) => p.replay_suppress_active(),
        }
    }

    /// Record an incomplete-output discard on the underlying port's
    /// flood-suppression latch and return the level to log it at (first of a
    /// regime → `Error`, repeats → `Debug` with a running suppressed count).
    /// Delegates to [`CerulionPublisher::record_output_discard`]. (The RECOVERY
    /// side — `record_output_complete` — is called on the destructured
    /// `&mut CerulionPublisher` directly at the two send-success sites in
    /// `OutputProxy::drop`, so it needs no `ProxyPublisher` delegation.)
    #[inline]
    pub(crate) fn record_output_discard(&mut self) -> DiscardLogLevel {
        match self {
            ProxyPublisher::Iceoryx2(p) => p.record_output_discard(),
        }
    }
}
