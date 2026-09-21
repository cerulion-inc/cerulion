// SPDX-License-Identifier: MIT OR Apache-2.0
//! Bidi-stream helpers with length-prefixed framing.
//!
//! Cerulion tunnels RAW wire frames over iroh streams — the payload is the
//! Cerulion [`WireHeader`]-framed frame VERBATIM (no envelope). A QUIC stream
//! is a byte stream, so we delimit frames with a 4-byte little-endian length
//! prefix: `[u32 len][len bytes]`. `read_frame` reads exactly that many bytes,
//! so a stream can carry many frames back-to-back.
//!
//! # Cancellation safety — [`read_frame`] and [`write_frame`] are NOT cancel-safe
//!
//! Each performs TWO sequential stream awaits (the length prefix, then the
//! payload), so dropping the future between them leaves the stream mid-frame
//! and PERMANENTLY desynced. **Never `select!` on these calls directly** — drive
//! framing in a dedicated per-connection task and `select!` on a channel
//! instead. See [`read_frame`]'s docs for the executable pattern.
//!
//! [`WireHeader`]: https://docs.rs/cerulion-wire

use iroh::endpoint::{Connection, RecvStream, SendStream};

use crate::error::LinkError;

/// Default cap for [`read_frame`]: 16 MiB — the largest payload the Cerulion
/// zero-copy moat is benchmarked against. Callers may pass their own bound.
pub const DEFAULT_MAX_FRAME_LEN: usize = 16 * 1024 * 1024;

/// Open a new outgoing bidirectional stream on `connection`.
pub async fn open_frame_stream(
    connection: &Connection,
) -> Result<(SendStream, RecvStream), LinkError> {
    connection
        .open_bi()
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))
}

/// Accept the next incoming bidirectional stream on `connection`.
pub async fn accept_frame_stream(
    connection: &Connection,
) -> Result<(SendStream, RecvStream), LinkError> {
    connection
        .accept_bi()
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))
}

/// Open a new outgoing UNIDIRECTIONAL stream on `connection` — this side sends,
/// the peer receives, and nothing flows back.
///
/// The one-way sibling of [`open_frame_stream`]: it yields only a
/// [`SendStream`]. This is the robot→desk data-stream primitive for the remote
/// wire plane — one demanded topic ⇒ one uni stream, so a slow topic's
/// per-stream QUIC flow control never stalls a fast one, and the stream's
/// lifetime IS the demand's lifetime (drop it to stop forwarding).
///
/// The frames themselves are written with the SAME [`write_frame`] used on bidi
/// streams — the length prefix is stream-shape-independent, so a [`SendStream`]
/// from here and one from [`open_frame_stream`] carry frames identically. Read
/// the peer's side with [`read_frame`] on the [`RecvStream`] from
/// [`accept_uni_frame_stream`].
pub async fn open_uni_frame_stream(connection: &Connection) -> Result<SendStream, LinkError> {
    connection
        .open_uni()
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))
}

/// Accept the next incoming UNIDIRECTIONAL stream on `connection` — the peer
/// sends, this side receives, and nothing flows back.
///
/// The one-way sibling of [`accept_frame_stream`]: it yields only a
/// [`RecvStream`]. The receiving half of the remote wire plane's per-topic
/// robot→desk data streams (see [`open_uni_frame_stream`]). Read frames off it
/// with [`read_frame`], exactly as on a bidi [`RecvStream`].
///
/// The same desync rules apply as on a bidi stream: a cancelled
/// [`read_frame`] OR a [`LinkError::FrameExceedsMaxLen`] leaves this stream
/// permanently mid-frame — drop it and accept the next uni stream, never read
/// the desynced one again.
pub async fn accept_uni_frame_stream(connection: &Connection) -> Result<RecvStream, LinkError> {
    connection
        .accept_uni()
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))
}

/// Write one length-prefixed frame: `[u32 LE len][frame bytes]`.
///
/// Returns [`LinkError::FrameExceedsWireLimit`] if `frame` does not fit in a
/// `u32` length prefix (the hard wire-format limit, ~4 GiB).
///
/// # Cancellation safety — NOT cancel-safe
///
/// This performs TWO sequential stream writes (the length prefix, then the
/// payload). If the returned future is dropped mid-call — e.g. a `tokio::select!`
/// branch fires — the peer is left mid-frame and the stream is permanently
/// desynced. **A cancelled call means the stream is unusable: abandon it, never
/// write it again.** Do NOT `select!` on `write_frame` directly; drive it from a
/// dedicated per-connection task that runs each call to completion (see
/// [`read_frame`] for the full pattern).
pub async fn write_frame(send: &mut SendStream, frame: &[u8]) -> Result<(), LinkError> {
    let len = u32::try_from(frame.len())
        .map_err(|_| LinkError::FrameExceedsWireLimit { len: frame.len() })?;
    send.write_all(&len.to_le_bytes())
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    send.write_all(frame)
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    Ok(())
}

/// Read one length-prefixed frame written by [`write_frame`].
///
/// The 4-byte length prefix is read first and validated against `max_len`
/// BEFORE any payload buffer is allocated — a hostile length can never make us
/// allocate more than `max_len`. Returns [`LinkError::FrameExceedsMaxLen`] if
/// the declared length exceeds `max_len`.
///
/// # Over-cap desync — the stream is unusable after [`LinkError::FrameExceedsMaxLen`]
///
/// The length prefix has been consumed but the payload has NOT, so — exactly
/// like a cancelled call (below) — the stream is left mid-frame and PERMANENTLY
/// desynced. **A `FrameExceedsMaxLen` means the stream is unusable: drop it,
/// never read it again.** Raising `max_len` (or a smaller peer frame) only
/// helps a fresh stream / next session, never this one.
///
/// # Cancellation safety — NOT cancel-safe
///
/// This performs TWO sequential stream reads (the length prefix, then the
/// payload). If the returned future is dropped between them — the classic
/// footgun of wrapping it in a `tokio::select!` for shutdown/timeout — the
/// length prefix has already been consumed but the payload has not, so the
/// stream is left mid-frame and its framing is PERMANENTLY desynced. **A
/// cancelled call means the stream is unusable: abandon it, never read it
/// again.**
///
/// Do NOT `select!` on `read_frame` directly. Drive framing in a DEDICATED
/// per-connection task that owns the stream and runs each call to completion,
/// hand whole frames off over a channel, and `select!` on the CHANNEL (which
/// IS cancel-safe) plus your shutdown signal:
///
/// ```no_run
/// use cerulion_link::{read_frame, LinkError, RecvStream, DEFAULT_MAX_FRAME_LEN};
/// use tokio::sync::mpsc;
///
/// /// Owns `recv`; every `read_frame` runs to completion (never cancelled
/// /// mid-frame). Delivers whole frames over a channel; any error ends the
/// /// task and closes the channel.
/// async fn reader_task(mut recv: RecvStream, frames: mpsc::Sender<Vec<u8>>) {
///     loop {
///         match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
///             Ok(frame) => {
///                 if frames.send(frame).await.is_err() {
///                     break; // consumer dropped the receiver
///                 }
///             }
///             Err(_e) => break, // stream ended/errored — stop, never reuse it
///         }
///     }
/// }
///
/// /// The consumer safely `select!`s on the CHANNEL (cancel-safe) plus a
/// /// shutdown signal — NEVER on `read_frame` itself.
/// async fn run(recv: RecvStream, mut shutdown: mpsc::Receiver<()>) {
///     let (tx, mut rx) = mpsc::channel::<Vec<u8>>(16);
///     let task = tokio::spawn(reader_task(recv, tx));
///     loop {
///         tokio::select! {
///             maybe_frame = rx.recv() => match maybe_frame {
///                 Some(_frame) => { /* dispatch the raw Cerulion wire frame */ }
///                 None => break, // reader task ended
///             },
///             _ = shutdown.recv() => break,
///         }
///     }
///     // Dropping/aborting the task drops the stream — safe now that we're done.
///     task.abort();
/// }
/// ```
pub async fn read_frame(recv: &mut RecvStream, max_len: usize) -> Result<Vec<u8>, LinkError> {
    let mut len_buf = [0u8; 4];
    recv.read_exact(&mut len_buf)
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    let len = u32::from_le_bytes(len_buf) as usize;
    if len > max_len {
        return Err(LinkError::FrameExceedsMaxLen { len, max: max_len });
    }
    let mut buf = vec![0u8; len];
    recv.read_exact(&mut buf)
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    Ok(buf)
}
