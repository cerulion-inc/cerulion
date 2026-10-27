// SPDX-License-Identifier: AGPL-3.0-only
//! The desk side of the `cerulion/rcp/1` plane: a robot's `cerulion-rcpd` as a
//! local stdio MCP server.
//!
//! [`run_rcp`] dials the robot's rcp ALPN with the desk device key, opens ONE
//! bidi stream (one MCP session), and bridges it with [`bridge`]: each line on
//! stdin (a newline-delimited JSON-RPC message, the MCP stdio framing) is sent
//! as one length-prefixed frame, and each frame from the robot is written to
//! stdout as one line. Stdout carries nothing else. The session ends when the
//! robot closes the stream, or after stdin reaches EOF and the robot has
//! finished answering.
//!
//! The robot decides what the desk may do: it admits the connection only for a
//! paired key on its access list and checks every message by its MCP method.

use std::net::SocketAddr;
use std::time::Duration;

use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    EndpointAddr, EndpointConfig, EndpointId, RecvStream, RelayConfig, SendStream,
    DEFAULT_MAX_FRAME_LEN,
};
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt, BufReader};

use crate::error::ConnectError;

/// The longest stdin line (one desk message) forwarded, bytes.
pub const MAX_LINE: usize = 1024 * 1024;

/// How long the dial and the stream open may take.
pub const DIAL_TIMEOUT: Duration = Duration::from_secs(30);

/// The resolved `cerulion-connectd rcp` parameters.
#[derive(Debug, Clone)]
pub struct RcpConfig {
    /// The robot's endpoint id.
    pub robot_eid: EndpointId,
    /// Direct `ip:port` addresses (LAN direct dial).
    pub direct_addrs: Vec<SocketAddr>,
    /// The desk device key seed.
    pub desk_seed: [u8; 32],
    /// The relay posture.
    pub relay: RelayConfig,
}

/// Dial the robot and bridge `input`/`output` to one rcp session until it ends.
///
/// # Errors
///
/// [`ConnectError`] when the endpoint cannot bind, the dial or the stream open
/// fails or times out, or a read or write on either side fails.
pub async fn run_rcp<R, W>(config: RcpConfig, input: R, output: W) -> Result<(), ConnectError>
where
    R: AsyncRead + Unpin + Send + 'static,
    W: AsyncWrite + Unpin + Send + 'static,
{
    let endpoint = build_endpoint(
        EndpointConfig::new(config.desk_seed)
            .with_alpns(vec![alpn::RCP.to_vec()])
            .with_relay(config.relay.clone()),
    )
    .await?;
    let peer: EndpointAddr = if config.direct_addrs.is_empty() {
        EndpointAddr::new(config.robot_eid)
    } else {
        direct_addr(config.robot_eid, config.direct_addrs.iter().copied())
    };
    tracing::info!(desk_id = %endpoint.id(), robot = %config.robot_eid, "cerulion rcp: dialing the robot");
    let connection = tokio::time::timeout(DIAL_TIMEOUT, dial(&endpoint, peer, alpn::RCP))
        .await
        .map_err(|_| {
            ConnectError::Refused(format!("dialing the robot took over {DIAL_TIMEOUT:?}"))
        })??;
    let (send, recv) = tokio::time::timeout(DIAL_TIMEOUT, open_frame_stream(&connection))
        .await
        .map_err(|_| ConnectError::Refused("opening the rcp stream timed out".into()))??;
    let result = bridge(input, output, send, recv).await;
    connection.close(0u32.into(), b"done");
    endpoint.close().await;
    result
}

/// Bridge newline-delimited JSON-RPC on `input`/`output` to frames on
/// `send`/`recv`. Blank lines are skipped. A robot frame holding a raw newline
/// is re-serialized compactly so it stays one line.
///
/// # Errors
///
/// [`ConnectError::Protocol`] for a stdin line over [`MAX_LINE`] or a robot
/// frame that is not UTF-8 JSON; [`ConnectError::Io`] for a local read or
/// write failure.
pub async fn bridge<R, W>(
    input: R,
    mut output: W,
    mut send: SendStream,
    mut recv: RecvStream,
) -> Result<(), ConnectError>
where
    R: AsyncRead + Unpin + Send + 'static,
    W: AsyncWrite + Unpin + Send + 'static,
{
    let uplink = tokio::spawn(async move {
        let mut lines = BufReader::new(input);
        let mut line = Vec::new();
        loop {
            line.clear();
            let n = (&mut lines)
                .take(MAX_LINE as u64 + 1)
                .read_until(b'\n', &mut line)
                .await?;
            if n == 0 {
                break;
            }
            if line.len() > MAX_LINE && line.last() != Some(&b'\n') {
                return Err(ConnectError::Protocol(format!(
                    "a stdin message exceeds {MAX_LINE} bytes"
                )));
            }
            let message = trim_line(&line);
            if message.is_empty() {
                continue;
            }
            write_frame(&mut send, message).await?;
        }
        let _ = send.finish();
        Ok::<(), ConnectError>(())
    });
    let downlink = async {
        loop {
            let frame = match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
                Ok(f) => f,
                Err(e) => {
                    tracing::debug!(error = %e, "cerulion rcp: the robot closed the session");
                    break;
                }
            };
            let line = one_line(frame)?;
            output.write_all(&line).await?;
            output.write_all(b"\n").await?;
            output.flush().await?;
        }
        Ok::<(), ConnectError>(())
    };
    let down = downlink.await;
    // The robot closed: whatever stdin still holds has nowhere to go.
    uplink.abort();
    let up = match uplink.await {
        Ok(r) => r,
        Err(_) => Ok(()),
    };
    down.and(up)
}

/// Process stdin as an [`AsyncRead`] fed by a detached std thread. Unlike
/// `tokio::io::stdin`, a pending read does not hold the runtime open: the
/// process can exit while the MCP client still holds stdin open.
pub fn stdin_reader() -> ChannelReader {
    let (tx, rx) = tokio::sync::mpsc::channel::<Vec<u8>>(16);
    std::thread::spawn(move || {
        use std::io::Read;
        let mut stdin = std::io::stdin().lock();
        let mut buf = vec![0u8; 64 * 1024];
        loop {
            match stdin.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    if tx.blocking_send(buf[..n].to_vec()).is_err() {
                        break;
                    }
                }
            }
        }
    });
    ChannelReader {
        rx,
        pending: Vec::new(),
        at: 0,
    }
}

/// An [`AsyncRead`] over byte chunks from a channel; EOF when the sender is
/// dropped.
pub struct ChannelReader {
    rx: tokio::sync::mpsc::Receiver<Vec<u8>>,
    pending: Vec<u8>,
    at: usize,
}

impl AsyncRead for ChannelReader {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut tokio::io::ReadBuf<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        use std::task::Poll;
        if self.at >= self.pending.len() {
            match self.rx.poll_recv(cx) {
                Poll::Ready(Some(chunk)) => {
                    self.pending = chunk;
                    self.at = 0;
                }
                Poll::Ready(None) => return Poll::Ready(Ok(())),
                Poll::Pending => return Poll::Pending,
            }
        }
        let n = buf.remaining().min(self.pending.len() - self.at);
        let start = self.at;
        buf.put_slice(&self.pending[start..start + n]);
        self.at += n;
        Poll::Ready(Ok(()))
    }
}

fn trim_line(line: &[u8]) -> &[u8] {
    let mut end = line.len();
    while end > 0 && matches!(line[end - 1], b'\n' | b'\r' | b' ' | b'\t') {
        end -= 1;
    }
    let mut start = 0;
    while start < end && matches!(line[start], b' ' | b'\t') {
        start += 1;
    }
    &line[start..end]
}

/// `frame` as one line: verbatim when it holds no raw newline, else
/// re-serialized compactly.
///
/// # Errors
///
/// [`ConnectError::Protocol`] when a frame holding a newline is not JSON.
pub fn one_line(frame: Vec<u8>) -> Result<Vec<u8>, ConnectError> {
    if !frame.contains(&b'\n') && !frame.contains(&b'\r') {
        return Ok(frame);
    }
    let value: Value = serde_json::from_slice(&frame).map_err(|e| {
        ConnectError::Protocol(format!("the robot sent a frame that is not JSON: {e}"))
    })?;
    serde_json::to_vec(&value)
        .map_err(|e| ConnectError::Protocol(format!("cannot re-serialize a robot frame: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_line_keeps_compact_frames_and_flattens_pretty_ones() {
        assert_eq!(one_line(b"{\"a\":1}".to_vec()).unwrap(), b"{\"a\":1}");
        assert_eq!(
            one_line(b"{\n  \"a\": \"x\\ny\"\n}".to_vec()).unwrap(),
            b"{\"a\":\"x\\ny\"}"
        );
        assert!(one_line(b"not\njson".to_vec()).is_err());
    }

    #[tokio::test]
    async fn channel_reader_yields_chunks_then_eof() {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut r = ChannelReader {
            rx,
            pending: Vec::new(),
            at: 0,
        };
        tx.send(b"ab".to_vec()).await.unwrap();
        tx.send(b"cde".to_vec()).await.unwrap();
        drop(tx);
        let mut out = Vec::new();
        r.read_to_end(&mut out).await.unwrap();
        assert_eq!(out, b"abcde");
    }

    #[test]
    fn trim_line_strips_framing_whitespace_only() {
        assert_eq!(trim_line(b"  {\"a\":1}\r\n"), b"{\"a\":1}");
        assert_eq!(trim_line(b"\n"), b"");
        assert_eq!(trim_line(b"{\"a\":\" \"}\n"), b"{\"a\":\" \"}");
    }
}
