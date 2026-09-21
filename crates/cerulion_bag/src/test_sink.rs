// SPDX-License-Identifier: AGPL-3.0-only
//! A scripted in-memory [`WritevSink`] for deterministically exercising the
//! writer's `writev` batch / partial-write / EINTR / no-progress loop.
//!
//! Only compiled for this crate's own tests and downstream test consumers (the
//! `test-helpers` feature). Never part of the production build.

use std::collections::VecDeque;
use std::io::{self, IoSlice};

use crate::writer::{WritevOutcome, WritevSink};

/// A scripted response for one `writev_once` call.
#[derive(Debug, Clone, Copy)]
pub enum SinkAction {
    /// Accept the entire presented batch.
    Full,
    /// Accept exactly `n` bytes of the presented batch (clamped to the batch
    /// total) — models a mid-frame / mid-payload / iovec-boundary short write.
    Short(usize),
    /// Return `EINTR` (write nothing, caller retries).
    Interrupted,
    /// Return `0` bytes with the batch non-empty (no forward progress).
    Zero,
    /// Fail with this `errno`.
    Fail(i32),
}

/// A scripted response for one `write_bytes` call — the COLD path
/// (prelude, message indexes, attachments, and a mid-file channel
/// registration), which `SinkAction` cannot reach because it scripts
/// `writev_once` only.
///
/// It exists because a failed cold-path write is what the writer's `poisoned`
/// latch is FOR, and until now no test could produce one: `cerulion_bagd`
/// builds its writer over the real `FileSink`, and the fault-injection seam in
/// `flush_chunk` fires before a byte reaches the sink (so it deliberately does
/// not poison).
#[derive(Debug, Clone, Copy)]
pub enum BytesAction {
    /// Accept the whole buffer.
    Full,
    /// Accept the first `n` bytes (clamped), then fail with `errno` — the
    /// PARTIAL-progress shape the poison latch exists for: the sink's byte
    /// stream is now longer than the writer believes it wrote.
    ShortThenFail {
        /// How many bytes of the buffer to accept before failing.
        n: usize,
        /// The `errno` to fail with.
        errno: i32,
    },
    /// Fail with this `errno` having accepted nothing.
    Fail(i32),
}

/// One recorded call on the sink, in call order.
///
/// The two existing probes cannot tell the cold-path calls apart —
/// `writev_calls()` counts only chunk flushes, and `iov_captures()` never sees
/// a `write_bytes` at all — so "the registration cost exactly one extra write
/// and no extra chunk boundary" was unassertable. Sizes are carried because a
/// registration's write is identified by WHAT it wrote, not merely by having
/// happened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkCall {
    /// One `writev_once` call, with the number of iovecs presented.
    Writev(usize),
    /// One `write_bytes` call, with the buffer length REQUESTED (not the length
    /// accepted — a short-then-fail script accepts less).
    WriteBytes(usize),
}

/// A [`WritevSink`] that records the exact byte stream it accepts (honouring
/// scripted short writes) so a test can assert byte-for-byte identity with a
/// single-`writev` expectation.
///
/// It ALSO captures every presented iovec's base pointer + length, so a test
/// can prove the zero-copy contract STRUCTURALLY: pointer identity with the
/// caller's payload buffer means the bytes rode directly from the caller's
/// memory. A memcpy regression (payload copied into writer scratch, iovec
/// pointing at the copy) produces identical BYTES but a different POINTER —
/// byte-equality tests alone cannot catch it.
pub struct ScriptedSink {
    bytes: Vec<u8>,
    actions: VecDeque<SinkAction>,
    /// The `write_bytes` script. EMPTY (the default) means every
    /// cold-path write succeeds, so every earlier caller is unaffected.
    byte_actions: VecDeque<BytesAction>,
    iov_max: usize,
    writev_calls: usize,
    iov_captures: Vec<Vec<(usize, usize)>>,
    /// Every call on this sink, in order (see [`SinkCall`]).
    calls: Vec<SinkCall>,
}

impl ScriptedSink {
    /// A sink with `iov_max` per-call iovec cap and a script of `writev_once`
    /// responses. When the script is exhausted, subsequent calls behave as
    /// [`SinkAction::Full`].
    pub fn new(iov_max: usize, actions: impl IntoIterator<Item = SinkAction>) -> Self {
        Self {
            bytes: Vec::new(),
            actions: actions.into_iter().collect(),
            byte_actions: VecDeque::new(),
            iov_max: iov_max.max(1),
            writev_calls: 0,
            iov_captures: Vec::new(),
            calls: Vec::new(),
        }
    }

    /// A sink that accepts everything in a single writev per flush (huge
    /// `iov_max`, empty script) — the "single-call expectation" oracle.
    pub fn single_call() -> Self {
        Self::new(usize::MAX, std::iter::empty())
    }

    /// Script the COLD path. When the script is exhausted, subsequent
    /// `write_bytes` calls behave as [`BytesAction::Full`].
    ///
    /// A builder rather than a `new` parameter so every existing call site keeps
    /// compiling unchanged — the sink's default behaviour is what it always was.
    pub fn with_byte_script(mut self, actions: impl IntoIterator<Item = BytesAction>) -> Self {
        self.byte_actions = actions.into_iter().collect();
        self
    }

    /// Every call made on this sink, in call order.
    pub fn calls(&self) -> &[SinkCall] {
        &self.calls
    }

    /// The full accepted byte stream (the reconstructed file).
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// How many `writev_once` calls the writer made (EINTR retries counted).
    pub fn writev_calls(&self) -> usize {
        self.writev_calls
    }

    /// The `(base pointer, len)` of every iovec presented to each
    /// `writev_once` call, in call order — the structural zero-copy probe
    /// (see the type docs). Assertions on it are IDENTITY-based (compare
    /// against a live buffer's `as_ptr()`), never value-based, so tests stay
    /// deterministic across runs.
    pub fn iov_captures(&self) -> &[Vec<(usize, usize)>] {
        &self.iov_captures
    }

    /// Record the first `n` bytes across `iovs` (in order).
    fn record(&mut self, iovs: &[IoSlice<'_>], mut n: usize) {
        for s in iovs {
            if n == 0 {
                break;
            }
            let take = n.min(s.len());
            self.bytes.extend_from_slice(&s[..take]);
            n -= take;
        }
    }
}

impl WritevSink for ScriptedSink {
    fn writev_once(&mut self, iovs: &[IoSlice<'_>]) -> WritevOutcome {
        self.writev_calls += 1;
        self.calls.push(SinkCall::Writev(iovs.len()));
        self.iov_captures.push(
            iovs.iter()
                .map(|s| (s.as_ptr() as usize, s.len()))
                .collect(),
        );
        let batch_total: usize = iovs.iter().map(|s| s.len()).sum();
        match self.actions.pop_front().unwrap_or(SinkAction::Full) {
            SinkAction::Full => {
                self.record(iovs, batch_total);
                WritevOutcome::Wrote(batch_total)
            }
            SinkAction::Short(n) => {
                let n = n.min(batch_total);
                self.record(iovs, n);
                WritevOutcome::Wrote(n)
            }
            SinkAction::Interrupted => WritevOutcome::Interrupted,
            SinkAction::Zero => WritevOutcome::Wrote(0),
            SinkAction::Fail(errno) => WritevOutcome::Failed(errno),
        }
    }

    fn iov_max(&self) -> usize {
        self.iov_max
    }

    fn write_bytes(&mut self, buf: &[u8]) -> io::Result<()> {
        // Record the REQUESTED length, before the script decides how much of it
        // is accepted: the call log answers "what did the writer try to write",
        // which is what identifies a registration's single write.
        self.calls.push(SinkCall::WriteBytes(buf.len()));
        match self.byte_actions.pop_front().unwrap_or(BytesAction::Full) {
            BytesAction::Full => {
                self.bytes.extend_from_slice(buf);
                Ok(())
            }
            BytesAction::ShortThenFail { n, errno } => {
                // The PARTIAL shape: bytes really do land, then the call fails.
                // The sink's stream is now longer than the writer's `pos`, which
                // is exactly the divergence the poison latch refuses to build on.
                self.bytes.extend_from_slice(&buf[..n.min(buf.len())]);
                Err(io::Error::from_raw_os_error(errno))
            }
            BytesAction::Fail(errno) => Err(io::Error::from_raw_os_error(errno)),
        }
    }

    fn sync_all(&mut self) -> io::Result<()> {
        Ok(())
    }
}
