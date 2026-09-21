// SPDX-License-Identifier: MIT OR Apache-2.0
//! [`QuicOpsStream`] — a synchronous [`Read`] + [`Write`] adapter over a QUIC
//! bidirectional stream, channel-backed and cancel-safe BY CONSTRUCTION.
//!
//! # Why this exists
//!
//! The Cerulion ops daemon (`cerud`) exposes a **synchronous** transport seam:
//! its `OpsServer::serve_connection` speaks over an `impl Read + Write` and does
//! its OWN length-framing (a big-endian `u32` prefix + JSON) INSIDE the stream.
//! The remote-plane host (`cerulion_remoted`) accepts one `cerulion/ops/1`
//! QUIC connection, takes ONE bidirectional stream off it, and hands it to
//! `cerud` on a `spawn_blocking` task — **one bidi QUIC stream = one
//! request/response ops session**. `QuicOpsStream` is that bridge: it moves RAW
//! bytes between `cerud`'s synchronous world and the asynchronous QUIC stream.
//!
//! ```ignore
//! // On a tokio task, after accepting a `cerulion/ops/1` connection:
//! let (send, recv) = cerulion_link::accept_frame_stream(&connection).await?;
//! let ops_stream = cerulion_link::QuicOpsStream::new(send, recv);
//! tokio::task::spawn_blocking(move || {
//!     // `cerud` frames its own bytes inside the stream; this stream never touches framing.
//!     cerud::OpsServer::serve_connection(ops_stream, caller, authorizer)
//! });
//! ```
//!
//! # It does NOT frame — do not double-frame
//!
//! `cerud` frames its own bytes. This adapter is a transparent byte pipe: it
//! carries whatever `cerud` writes, verbatim. It deliberately does NOT use
//! [`write_frame`](crate::write_frame) / [`read_frame`](crate::read_frame)
//! (those belong to the WIRE plane, and layering them here would frame the
//! stream twice and corrupt it).
//!
//! # Cancel-safety by construction
//!
//! [`read_frame`](crate::read_frame) / [`write_frame`](crate::write_frame) are
//! NOT cancel-safe (two sequential awaits per call). This adapter sidesteps that
//! class of bug entirely: two DEDICATED tokio tasks own the [`SendStream`] and
//! [`RecvStream`] and drive every stream operation to completion. The
//! synchronous [`Read`] / [`Write`] halves only ever touch bounded in-memory
//! channels — there is no `await` a consumer could cancel mid-operation, so the
//! QUIC framing can never be left half-written or half-read.
//!
//! # Threading contract — Read/Write MUST run OFF the async runtime (they PANIC on it)
//!
//! [`Read`] and [`Write`] BLOCK the calling thread (they call
//! [`blocking_recv`](tokio::sync::mpsc::Receiver::blocking_recv) /
//! [`blocking_send`](tokio::sync::mpsc::Sender::blocking_send)). tokio PANICS
//! (`Cannot block the current thread from within a runtime`) if either is
//! called from an async runtime WORKER thread — i.e. from inside an `async fn`
//! or `block_on`. Use them ONLY off the runtime: on a
//! [`tokio::task::spawn_blocking`] task (blocking is allowed there) or a plain
//! dedicated thread. This is a hard contract, not a soft preference — there is
//! no programmatic guard (a `spawn_blocking` task legitimately holds a runtime
//! handle, so `Handle::try_current()` cannot tell correct usage from a worker
//! thread), so honoring it is the caller's responsibility.
//!
//! # No built-in timeout — a stalled peer wedges the blocking thread
//!
//! There is deliberately NO read/write timeout: a live-but-silent peer can wedge
//! the blocking session thread indefinitely (a `Read` parks in `blocking_recv`
//! with no bound). Bounding a session is the HOST's responsibility — the ops
//! host (`cerulion_remoted`) enforces a per-session deadline. To unblock a
//! parked reader out-of-band, drop the adapter from another task: the reader
//! task is aborted and `blocking_recv` returns `None` → the `Read` unblocks with
//! `Ok(0)`.
//!
//! # EOF, flush, and shutdown semantics
//!
//! - **Clean EOF vs. broken session.** When the remote writer cleanly finishes,
//!   the reader task closes the read channel and the synchronous [`Read::read`]
//!   returns `Ok(0)` — a normal end-of-stream, never a desync or a panic. When
//!   the recv stream instead RESETS / errors, the reader logs it and the next
//!   [`Read::read`] returns ONE [`io::Error`]
//!   ([`ConnectionReset`](std::io::ErrorKind::ConnectionReset), the underlying
//!   QUIC error chained as its `source`) before the subsequent `Ok(0)` — so a
//!   broken session is distinguishable from a graceful close.
//! - **Flush.** [`Write::flush`] is a no-op: each [`Write::write`] hands its
//!   bytes to the writer task, which drives them onto the QUIC stream
//!   immediately. There is no additional userspace buffer for `flush` to drain
//!   (the bounded channel is drained by the writer task, not by `flush`).
//! - **Drop / shutdown (never deadlocks).** Dropping the [`QuicOpsStream`] is
//!   non-blocking: it aborts the reader task (which may be parked on a read) and
//!   closes the write channel, which signals the writer task to drain any queued
//!   bytes, `finish()` the send stream (a clean EOF to the peer), and exit on its
//!   own. The writer is NEVER aborted, so a final response already written is not
//!   lost. Both tasks terminate without blocking the runtime; the caller keeps
//!   the QUIC connection alive long enough for the final bytes to transmit
//!   (standard QUIC teardown).

use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};

use iroh::endpoint::{RecvStream, SendStream};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use crate::error::BoxError;

/// A one-shot terminal error handed from a driver task to its synchronous half.
/// `None` = no error recorded (the clean path). The reader records a QUIC
/// recv-stream error here (surfaced once before EOF); the writer records a QUIC
/// send-stream error here (chained as the source of the next `Write`'s
/// `BrokenPipe`). Taken (set back to `None`) when surfaced, so it fires exactly
/// once.
type TerminalErrorSlot = Arc<Mutex<Option<io::Error>>>;

/// The concrete iroh stream error that terminated a driver task, wrapped so a
/// synchronous [`Read`] / [`Write`] can surface an [`io::Error`] whose
/// [`source`](std::error::Error::source) chains to the real QUIC cause.
///
/// `io::Error::new(kind, e)` alone would report only `e.source()` — std
/// delegates `io::Error::source` to the INNER error's own source, not to `e`
/// itself — so we interpose this wrapper to make `e` the reported cause: a
/// diagnosable chain instead of a cause-free static string.
#[derive(Debug)]
struct StreamCause {
    /// A short, direction-specific description (the wrapper's own `Display`).
    what: &'static str,
    /// The underlying iroh stream error, exposed via [`std::error::Error::source`].
    source: BoxError,
}

impl std::fmt::Display for StreamCause {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.what)
    }
}

impl std::error::Error for StreamCause {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.source.as_ref())
    }
}

/// Bounded capacity (in chunks) of each per-direction channel between the
/// synchronous halves and the QUIC driver tasks. Ops sessions are
/// request/response and bounded, so a small cap suffices; a full channel applies
/// natural back-pressure (the synchronous [`Write::write`] blocks until the
/// writer task drains) rather than buffering without bound.
const OPS_STREAM_QUEUE_CAP: usize = 64;

/// Size of the reader task's read buffer. Each QUIC read fills up to this many
/// bytes before the chunk is handed to the synchronous [`Read`] half.
const READ_CHUNK_LEN: usize = 64 * 1024;

/// Reader task: owns the [`RecvStream`], reads it to completion, and hands whole
/// chunks to the synchronous [`Read`] half over `tx`. Runs each `read` to
/// completion (never cancelled mid-read from the consumer side), so the QUIC
/// framing is never left mid-operation. Exits — closing `tx`, which surfaces to
/// the reader as EOF — on clean stream finish, on a stream error, or when the
/// synchronous half drops its receiver.
///
/// A clean peer finish (`Ok(None)`) and a stream error (`Err`) are kept
/// DISTINCT: a graceful finish records nothing (the sync `Read` sees `Ok(0)`),
/// while an error records a one-shot terminal [`io::Error`] into `read_err`
/// (and logs it) so the sync `Read` can surface a broken session as a
/// `ConnectionReset` ONCE before the subsequent EOF — distinguishing a reset
/// peer from a graceful close. (`cerud`'s own inner length-framing also detects
/// a truncated response; this is defense-in-depth + diagnosability, and the
/// clean-EOF-on-graceful-finish contract is unchanged.)
async fn drive_reader(
    mut recv: RecvStream,
    tx: mpsc::Sender<Vec<u8>>,
    read_err: TerminalErrorSlot,
) {
    let mut buf = vec![0u8; READ_CHUNK_LEN];
    loop {
        match recv.read(&mut buf).await {
            Ok(Some(n)) if n > 0 => {
                if tx.send(buf[..n].to_vec()).await.is_err() {
                    // The synchronous half dropped its receiver — nothing to
                    // deliver to. Stop.
                    break;
                }
            }
            // A non-empty read buffer never yields `Some(0)` from a live stream,
            // but forwarding a zero-length chunk would look like EOF to the
            // synchronous reader — defensively skip it and read again.
            Ok(Some(_)) => {}
            // The peer cleanly finished the send stream — a graceful EOF, no
            // error recorded. The sync `Read` observes the closed channel as
            // `Ok(0)`.
            Ok(None) => break,
            // The stream was reset / errored — record a one-shot terminal error
            // so the sync `Read` surfaces it once (after every already-delivered
            // chunk is consumed) before the subsequent clean EOF.
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "cerulion_link QuicOpsStream: QUIC recv stream errored; \
                     surfacing one ConnectionReset to the sync reader before EOF"
                );
                let io_err = io::Error::new(
                    io::ErrorKind::ConnectionReset,
                    StreamCause {
                        what: "QuicOpsStream: the QUIC recv stream errored",
                        source: Box::new(e),
                    },
                );
                if let Ok(mut slot) = read_err.lock() {
                    *slot = Some(io_err);
                }
                break;
            }
        }
    }
    // Dropping `tx` here closes the channel → the synchronous `Read` returns the
    // recorded terminal error once (if any), then `Ok(0)` (clean EOF).
}

/// Writer task: owns the [`SendStream`], pops whole chunks from the synchronous
/// [`Write`] half over `rx`, and writes each to completion. When the synchronous
/// half drops its sender (the [`QuicOpsStream`] was dropped), `rx.recv()` yields
/// `None`: the task has drained every queued byte, so it `finish()`es the send
/// stream (a clean EOF to the peer) and exits. On a write error the task logs
/// the error + how many chunks were still queued (never delivered), records a
/// one-shot terminal error into `write_err`, and stops; the now-closed `rx`
/// makes the synchronous [`Write::write`] observe a broken pipe on its next
/// call — and that `BrokenPipe` carries the underlying QUIC error as its source
/// (a cause-chained `io::Error`, not a static string).
async fn drive_writer(
    mut send: SendStream,
    mut rx: mpsc::Receiver<Vec<u8>>,
    write_err: TerminalErrorSlot,
) {
    while let Some(chunk) = rx.recv().await {
        if let Err(e) = send.write_all(&chunk).await {
            // Peer reset / connection gone. Log loudly with the discarded-queue
            // depth, then record the cause so the next synchronous `Write` can
            // chain it under a BrokenPipe. Dropping `rx` (on return) closes the
            // channel, which is how that `Write` observes the failure.
            let queued_chunks = rx.len();
            tracing::warn!(
                error = %e,
                queued_chunks,
                "cerulion_link QuicOpsStream: QUIC send stream write failed; \
                 discarding queued chunks and closing the write channel"
            );
            let io_err = io::Error::new(
                io::ErrorKind::BrokenPipe,
                StreamCause {
                    what: "QuicOpsStream: the QUIC writer task closed after a send error",
                    source: Box::new(e),
                },
            );
            if let Ok(mut slot) = write_err.lock() {
                *slot = Some(io_err);
            }
            return;
        }
    }
    // The synchronous half closed the write channel → drain complete; signal a
    // clean end-of-stream to the peer. `finish()` only errors if the stream was
    // already closed, which is harmless here.
    let _ = send.finish();
}

/// A synchronous [`Read`] + [`Write`] adapter over a QUIC bidirectional stream,
/// channel-backed and cancel-safe by construction.
///
/// Construct one with [`QuicOpsStream::new`] from a QUIC bidi stream's
/// `(SendStream, RecvStream)` pair (e.g. from
/// [`accept_frame_stream`](crate::accept_frame_stream) /
/// [`open_frame_stream`](crate::open_frame_stream)), then hand it to a
/// synchronous byte consumer. Its [`Read`] / [`Write`] BLOCK the calling
/// thread, so run it OFF the async runtime — on a
/// [`tokio::task::spawn_blocking`] task or a dedicated thread. Calling them from
/// an async runtime worker thread PANICS (see the module docs' threading
/// contract). Its purpose is to bridge an async QUIC stream to a SYNCHRONOUS
/// request/response consumer (the Cerulion ops daemon `cerud`): **one bidi QUIC
/// stream = one ops session**.
///
/// It moves RAW bytes and does NOT frame them — the consumer does its own
/// framing inside the stream, so this adapter deliberately avoids
/// [`write_frame`](crate::write_frame) / [`read_frame`](crate::read_frame)
/// (layering them here would frame the stream twice).
///
/// # Cancel-safety by construction
///
/// Two dedicated tokio tasks own the [`SendStream`] / [`RecvStream`] and drive
/// every stream operation to completion; the synchronous halves only touch
/// bounded in-memory channels, so there is no `await` a consumer could cancel
/// mid-operation and the QUIC framing can never be left half-read/-written.
///
/// # EOF, flush, and shutdown
///
/// - **EOF vs. broken session.** A clean remote finish makes [`Read::read`]
///   return `Ok(0)` (a clean end-of-stream, never a desync or a panic); a recv
///   stream reset/error instead surfaces ONE
///   [`ConnectionReset`](std::io::ErrorKind::ConnectionReset) `io::Error` (with
///   the QUIC cause chained as `source`) before the `Ok(0)`.
/// - **Flush.** [`Write::flush`] is a no-op: each [`Write::write`] hands its
///   bytes to the writer task, which drives them onto the stream immediately
///   (there is no extra userspace buffer to drain).
/// - **Drop.** Dropping the stream never blocks: it aborts the reader task and
///   closes the write channel, which signals the writer task to drain queued
///   bytes, `finish()` the send stream (clean EOF to the peer), and exit. The
///   writer is never aborted, so a final already-written response is not lost.
pub struct QuicOpsStream {
    /// Whole chunks handed over by the reader task.
    read_rx: mpsc::Receiver<Vec<u8>>,
    /// A chunk received but not yet fully consumed by [`Read::read`] (the
    /// caller's buffer was smaller than the chunk); served from `read_pos`.
    read_leftover: Vec<u8>,
    /// Cursor into `read_leftover` of the next unconsumed byte.
    read_pos: usize,
    /// One-shot terminal error recorded by the reader task on a QUIC recv-stream
    /// error, surfaced to the sync [`Read`] exactly once (as `ConnectionReset`)
    /// before EOF. Shared with `drive_reader`.
    read_err: TerminalErrorSlot,
    /// Feeds whole chunks to the writer task.
    write_tx: mpsc::Sender<Vec<u8>>,
    /// One-shot terminal error recorded by the writer task on a QUIC send-stream
    /// error, chained as the source of the sync [`Write`]'s `BrokenPipe`. Shared
    /// with `drive_writer`.
    write_err: TerminalErrorSlot,
    /// The reader task handle — aborted on [`Drop`] (it may be parked on a read).
    /// The writer task is intentionally NOT retained: closing `write_tx` on drop
    /// lets it drain + `finish()` + exit on its own (aborting it would drop a
    /// final unflushed response).
    reader: JoinHandle<()>,
}

impl QuicOpsStream {
    /// Wrap a QUIC bidirectional stream's `(send, recv)` halves in a synchronous
    /// [`Read`] + [`Write`] adapter.
    ///
    /// Spawns the two driver tasks, so this MUST be called from within a tokio
    /// runtime context (it uses [`tokio::spawn`]). The returned value is `Send`
    /// and can be moved onto a `spawn_blocking` task.
    pub fn new(send: SendStream, recv: RecvStream) -> Self {
        let (read_tx, read_rx) = mpsc::channel::<Vec<u8>>(OPS_STREAM_QUEUE_CAP);
        let (write_tx, write_rx) = mpsc::channel::<Vec<u8>>(OPS_STREAM_QUEUE_CAP);

        let read_err: TerminalErrorSlot = Arc::new(Mutex::new(None));
        let write_err: TerminalErrorSlot = Arc::new(Mutex::new(None));

        let reader = tokio::spawn(drive_reader(recv, read_tx, Arc::clone(&read_err)));
        // Detached on purpose: the writer self-terminates when `write_tx` closes
        // (drains, `finish()`es, exits), so its handle is not retained here. Its
        // `JoinHandle` is not `#[must_use]`, so dropping it here is a no-op for
        // the task (dropping a `JoinHandle` detaches; it does NOT cancel).
        tokio::spawn(drive_writer(send, write_rx, Arc::clone(&write_err)));

        Self {
            read_rx,
            read_leftover: Vec::new(),
            read_pos: 0,
            read_err,
            write_tx,
            write_err,
            reader,
        }
    }
}

impl Read for QuicOpsStream {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }
        // Serve any leftover from a prior chunk first; otherwise block for the
        // next whole chunk from the reader task.
        if self.read_pos >= self.read_leftover.len() {
            match self.read_rx.blocking_recv() {
                Some(chunk) => {
                    self.read_leftover = chunk;
                    self.read_pos = 0;
                }
                // Reader task ended. Every delivered chunk has been consumed
                // (a closed+empty channel), so if the reader recorded a terminal
                // stream error, surface it ONCE (leaving the slot empty), then a
                // clean EOF (`Ok(0)`) on the next call. A graceful peer finish
                // records no error → straight to `Ok(0)`.
                None => {
                    if let Some(err) = self.read_err.lock().ok().and_then(|mut slot| slot.take()) {
                        return Err(err);
                    }
                    return Ok(0);
                }
            }
        }
        let available = &self.read_leftover[self.read_pos..];
        let n = available.len().min(out.len());
        out[..n].copy_from_slice(&available[..n]);
        self.read_pos += n;
        Ok(n)
    }
}

impl Write for QuicOpsStream {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        if data.is_empty() {
            return Ok(0);
        }
        match self.write_tx.blocking_send(data.to_vec()) {
            Ok(()) => Ok(data.len()),
            // The writer task exited (peer reset / connection gone) → the byte
            // sink is gone; surface a broken pipe rather than losing bytes
            // silently. If the writer recorded the underlying QUIC error, return
            // that cause-chained `io::Error` (once — the diagnosable signal);
            // any later write finds an empty slot → a plain `BrokenPipe`.
            Err(_) => {
                if let Some(err) = self.write_err.lock().ok().and_then(|mut slot| slot.take()) {
                    return Err(err);
                }
                Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "QuicOpsStream: the QUIC writer task has closed",
                ))
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        // No-op: `write` hands each buffer to the writer task, which drives it to
        // the QUIC stream immediately. There is no additional userspace buffer
        // for `flush` to drain (see the module docs' "flush semantics").
        Ok(())
    }
}

impl Drop for QuicOpsStream {
    fn drop(&mut self) {
        // The reader may be parked in `recv.read().await` (peer silent) — abort
        // it so it does not linger holding the recv stream. This is safe: QUIC
        // reads are cancel-safe, and we are discarding the stream anyway.
        self.reader.abort();
        // The writer is NOT aborted. `write_tx` (a field) is dropped right after
        // this returns, closing the write channel; the detached writer task then
        // drains any queued bytes, `finish()`es the send stream (clean EOF to the
        // peer), and exits. Aborting it would drop a final unflushed response.
    }
}
