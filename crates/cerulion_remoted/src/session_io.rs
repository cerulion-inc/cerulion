// SPDX-License-Identifier: AGPL-3.0-only
//! Owned session framing tasks with bounded queues and writes.
//!
//! Each task owns its stream. Cancelling a task discards that stream; a partially
//! read or written frame must never be resumed by another task. Channel capacity
//! is chosen by the caller, and a full channel applies backpressure.

use std::io;
use std::time::Duration;

use cerulion_ops_proto::session::{
    decode_client_frame, encode_session, ClientFrame, ExitReason, ServerFrame,
    MAX_SESSION_FRAME_BYTES,
};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::sync::{mpsc, oneshot};

/// A validated client frame or a terminal framing failure.
#[derive(Debug, PartialEq, Eq)]
pub enum SessionIoEvent {
    Client(ClientFrame),
    Failed(ExitReason),
}

/// Read complete frames into the caller's bounded event queue.
///
/// E can be the driver's larger event enum, avoiding an intermediate task or
/// channel. The conversion must only move the event into that enum. EOF and read
/// errors report disconnection; invalid lengths or contents report protocol
/// failure. Either failure ends the task without reading another frame.
pub async fn read_client<R, E>(mut reader: R, events: mpsc::Sender<E>)
where
    R: AsyncRead + Unpin,
    E: From<SessionIoEvent>,
{
    loop {
        // Never select on framing: cancellation drops this entire stream.
        let length = match reader.read_u32().await {
            Ok(length) if length as usize <= MAX_SESSION_FRAME_BYTES => length as usize,
            Ok(_) => {
                let _ = events
                    .send(SessionIoEvent::Failed(ExitReason::ProtocolError).into())
                    .await;
                return;
            }
            Err(_) => {
                let _ = events
                    .send(SessionIoEvent::Failed(ExitReason::Disconnected).into())
                    .await;
                return;
            }
        };
        let mut bytes = vec![0; length];
        if reader.read_exact(&mut bytes).await.is_err() {
            let _ = events
                .send(SessionIoEvent::Failed(ExitReason::Disconnected).into())
                .await;
            return;
        }
        let frame = match decode_client_frame(&bytes) {
            Ok(frame) => frame,
            Err(_) => {
                let _ = events
                    .send(SessionIoEvent::Failed(ExitReason::ProtocolError).into())
                    .await;
                return;
            }
        };
        if events
            .send(SessionIoEvent::Client(frame).into())
            .await
            .is_err()
        {
            return;
        }
    }
}

/// Flush server frames in queue order, with one deadline per complete frame.
///
/// Serialization, write, flush or deadline failure reports disconnection and
/// drops done. Once all queued frames have been flushed, stream shutdown is
/// attempted within the same bound and done is sent. This completion describes
/// local writes, not receipt or validation by the peer.
pub async fn write_server<W, E>(
    mut writer: W,
    mut frames: mpsc::Receiver<ServerFrame>,
    events: mpsc::Sender<E>,
    timeout: Duration,
    done: oneshot::Sender<()>,
) where
    W: AsyncWrite + Unpin,
    E: From<SessionIoEvent>,
{
    while let Some(frame) = frames.recv().await {
        let result = tokio::time::timeout(timeout, async {
            let bytes = encode_session(&frame).map_err(io::Error::other)?;
            writer.write_u32(bytes.len() as u32).await?;
            writer.write_all(&bytes).await?;
            writer.flush().await
        })
        .await;
        if !matches!(result, Ok(Ok(()))) {
            let _ = events
                .send(SessionIoEvent::Failed(ExitReason::Disconnected).into())
                .await;
            return;
        }
    }
    let _ = tokio::time::timeout(timeout, writer.shutdown()).await;
    let _ = done.send(());
}

#[cfg(test)]
#[path = "session_io_test.rs"]
mod tests;
