//! The capture sink — and the one type that turns the fast path's BYTE bound
//! into a TIME bound.
//!
//! [`BoundedSink`] writes into a caller-provided slice of a pre-allocated
//! arena under a capacity it never exceeds. Combined with
//! [`CerulionState::INLINE_SAFE`](super::CerulionState::INLINE_SAFE) — which
//! admits only framework-generated walks over lock-free, interior-mutability-
//! free types — the byte bound is the whole cost bound: the node thread runs
//! no user control flow, takes no lock and makes no syscall, so the only
//! unbounded dimension left is bytes, and this type bounds that.

use super::StateError;

/// The sink refused a write because it would pass its capacity.
///
/// Deliberately a distinct zero-sized type rather than a [`StateError`]
/// variant at the [`StateSink`] boundary: a sink knows only that it is full,
/// and converting at the encoder (`?` through [`From`]) keeps the sink trait
/// free of the decoder's vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SinkFull;

impl std::fmt::Display for SinkFull {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("state sink is full")
    }
}

impl std::error::Error for SinkFull {}

impl From<SinkFull> for StateError {
    fn from(_: SinkFull) -> Self {
        StateError::SinkFull
    }
}

/// Where a capture's bytes go.
///
/// Two implementations exist across the design: [`BoundedSink`] on the inline
/// carrier (bounded, refuses past its capacity) and, in the fork carrier, an
/// unbounded backpressured ring sink. Both feed the *same* encoder and emit
/// *identical bytes*, so which carrier a node took is invisible in the bag.
pub trait StateSink {
    /// Append `bytes`.
    ///
    /// A sink that cannot take the whole slice MUST write **nothing** and
    /// return [`SinkFull`]: a partially-applied write would leave a hole in
    /// the middle of an encoding that the decoder has no way to detect.
    fn write(&mut self, bytes: &[u8]) -> Result<(), SinkFull>;

    /// Bytes this sink can still accept, when it knows.
    ///
    /// `None` means "unbounded as far as the encoder need care" and is the
    /// default, so an unbounded ring sink implements nothing extra.
    ///
    /// This is a **hint for refusing early, never a substitute for
    /// [`Self::write`]'s own bound**. It exists because the canonical order
    /// for a hash-like container requires a sort index built *before*
    /// the first byte is written: without a hint, capturing a 30-million-entry
    /// `HashMap` into a 64 KiB sink would allocate a ~240 MB index on the node
    /// thread and only *then* discover it cannot fit — defeating the bounded-
    /// cost guarantee this module exists to provide. With it, an encoder that
    /// needs scratch can prove the container cannot fit and refuse before
    /// allocating anything.
    fn remaining_hint(&self) -> Option<usize> {
        None
    }

    /// Latch this sink as refused **without attempting a write**.
    ///
    /// An encoder that decides up front it cannot fit — the hash-like
    /// containers, which must build a sort index before their first byte and
    /// therefore consult [`Self::remaining_hint`] first — returns
    /// [`StateError::SinkFull`](super::StateError::SinkFull) having called
    /// `write` zero times. Without this call such a refusal would leave
    /// `refused` clear, so [`BoundedSink::consumed`] would report only the
    /// bytes written (usually **zero**) and the boundary walk would hand the
    /// same budget to the next node — an uncharged path through the invariant
    /// that at most one node per boundary pays for a refusal.
    ///
    /// Idempotent, and a no-op for unbounded sinks.
    fn refuse(&mut self) {}
}

/// The inline carrier's sink: a hard capacity over a borrowed arena slice.
///
/// The **first** write that would pass `cap` writes nothing, returns
/// [`SinkFull`], and **latches**: every later write is refused too. Latching
/// is not defensive tidiness — without it a small write following a refused
/// large one would land immediately after the last successful byte, producing
/// a byte stream that is missing a field in the middle and looks structurally
/// valid to a decoder.
///
/// # The shared budget, and charged abort
///
/// The boundary walk gives each node a sink over `&mut arena[used..]` with
/// `cap = BUDGET - used`, and adds [`Self::consumed`] to `used` afterwards —
/// on the success arm *and* on the abort arm. So the **total** inline
/// encoding at one step boundary is hard bounded at
/// [`CAPTURE_INLINE_BUDGET_BYTES`](super::CAPTURE_INLINE_BUDGET_BYTES) no
/// matter how many nodes overflow: a 40-node worker cannot spend 40 budgets.
///
/// [`Self::consumed`] charges an aborted sink its **entire granted capacity**,
/// not merely the bytes it managed to write. That is what makes the stated
/// consequence literally true — "the first overflow effectively pushes the
/// rest of that boundary's nodes into the fork set (they get a zero-byte sink
/// and refuse at their first write)" — and it bounds the boundary more tightly
/// than charging bytes-written would: at most **one** node per boundary can
/// pay for a refusal. Charging less would still be *bounded*, but would let
/// several nodes each burn most of a budget before the arena filled.
pub struct BoundedSink<'a> {
    buf: &'a mut [u8],
    used: usize,
    cap: usize,
    refused: bool,
}

impl<'a> BoundedSink<'a> {
    /// A sink over the whole of `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        let cap = buf.len();
        Self {
            buf,
            used: 0,
            cap,
            refused: false,
        }
    }

    /// A sink over `buf`, capped at `cap` bytes.
    ///
    /// `cap` is clamped to `buf.len()`: a capacity above the slice would
    /// promise memory the sink does not own, and there is no sound reading of
    /// it other than "the slice is the real bound". The boundary walk always
    /// passes `cap == buf.len()`, so the clamp is unreachable in production
    /// and exists so a caller bug degrades to a smaller sink rather than to a
    /// panic on the capture path.
    pub fn with_capacity(buf: &'a mut [u8], cap: usize) -> Self {
        let cap = cap.min(buf.len());
        Self {
            buf,
            used: 0,
            cap,
            refused: false,
        }
    }

    /// Bytes successfully written.
    pub fn len(&self) -> usize {
        self.used
    }

    /// Whether nothing has been written yet.
    pub fn is_empty(&self) -> bool {
        self.used == 0
    }

    /// The capacity this sink was granted (post-clamp).
    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// Whether a write has ever been refused.
    ///
    /// Observable independently of the encoder's return value (Principle #3):
    /// the carrier records the per-node carrier decision from this, and a
    /// refused sink's bytes are never published.
    pub fn refused(&self) -> bool {
        self.refused
    }

    /// The capacity to charge against the shared boundary budget.
    ///
    /// The bytes written, or — once a write has been **refused** — the sink's
    /// entire granted capacity. See the type-level docs for why the abort arm
    /// charges the whole capacity.
    pub fn consumed(&self) -> usize {
        if self.refused {
            self.cap
        } else {
            self.used
        }
    }

    /// The bytes written so far.
    ///
    /// Meaningful only when [`Self::refused`] is `false`; a refused sink's
    /// prefix is a partial encoding and the carrier discards it.
    pub fn written(&self) -> &[u8] {
        &self.buf[..self.used]
    }
}

impl StateSink for BoundedSink<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), SinkFull> {
        if self.refused {
            return Err(SinkFull);
        }
        match self.used.checked_add(bytes.len()) {
            // A write landing EXACTLY at the boundary is accepted; one byte
            // over is refused. `>` and not `>=` — the boundary is inclusive.
            Some(end) if end <= self.cap => {
                self.buf[self.used..end].copy_from_slice(bytes);
                self.used = end;
                Ok(())
            }
            _ => {
                self.refused = true;
                Err(SinkFull)
            }
        }
    }

    fn remaining_hint(&self) -> Option<usize> {
        Some(if self.refused {
            0
        } else {
            self.cap - self.used
        })
    }

    fn refuse(&mut self) {
        self.refused = true;
    }
}

/// A growable sink over a `Vec<u8>`.
///
/// The unbounded counterpart used by the fork child (which encodes into a
/// backpressured ring, never a bounded arena) and by tests that want the full
/// canonical bytes of a value. It reports no [`StateSink::remaining_hint`], so
/// encoders that need scratch take the ordinary path.
#[derive(Debug, Default)]
pub struct VecSink {
    buf: Vec<u8>,
}

impl VecSink {
    /// An empty sink.
    pub fn new() -> Self {
        Self::default()
    }

    /// The bytes written so far.
    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    /// Consume the sink and return its bytes.
    pub fn into_inner(self) -> Vec<u8> {
        self.buf
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    /// Whether nothing has been written yet.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

impl StateSink for VecSink {
    fn write(&mut self, bytes: &[u8]) -> Result<(), SinkFull> {
        self.buf.extend_from_slice(bytes);
        Ok(())
    }
}
