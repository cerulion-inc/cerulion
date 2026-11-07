// SPDX-License-Identifier: AGPL-3.0-only
//! The desk side of the `cerulion/rcp/1` plane: a robot's `cerulion-rcpd` as a
//! local stdio MCP server.
//!
//! [`run_rcp`] dials the robot's rcp ALPN with the desk device key, opens ONE
//! bidi stream (one MCP session), and bridges it with [`bridge`]: each line on
//! stdin (a newline-delimited JSON-RPC message, the MCP stdio framing) is sent
//! as one length-prefixed frame, and each frame from the robot is written to
//! stdout as one line. Stdout carries nothing else: a robot frame that is not a
//! JSON-RPC 2.0 message is never written. The session ends after stdin reaches
//! EOF and the robot has finished the stream at a frame boundary; that is the
//! one `Ok` end.
//!
//! Every other end is an error: the robot's refusal of the session (its
//! accept decision, [`ConnectError::Refused`] with the robot's reason), a
//! connection the robot closed with [`CLOSE_REVOKED`] or [`CLOSE_SUPERSEDED`]
//! ([`ConnectError::Refused`]), the robot ending the stream while stdin is
//! still open, or a stream that breaks: a reset, a lost connection, a frame
//! cut off by the end of the stream, or a robot frame over the 16 MiB cap
//! ([`ConnectError::Control`], its cause sanitized and bounded as peer text),
//! or any other frame that is not a JSON-RPC 2.0 message
//! ([`ConnectError::Protocol`]).
//!
//! The robot decides what the desk may do: it admits the connection only for a
//! paired key on its access list and checks every message by its MCP method.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame_or_end, write_frame,
    Connection, ConnectionError, EndpointAddr, EndpointConfig, EndpointId, LinkError, RecvStream,
    RelayConfig, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt, BufReader};

use crate::error::ConnectError;

/// The longest stdin line (one desk message) forwarded, bytes.
pub const MAX_LINE: usize = 1024 * 1024;

/// How long the dial and the stream open may take.
pub const DIAL_TIMEOUT: Duration = Duration::from_secs(30);

/// The QUIC application close code of an rcp connection whose key lost rcp
/// access (the robot's `cerulion_remoted::rcp::CLOSE_REVOKED`).
pub const CLOSE_REVOKED: u32 = 0x5243_0001;

/// The QUIC application close code of a key's oldest rcp connection, closed
/// for a newer one from the same key (the robot's
/// `cerulion_remoted::rcp::CLOSE_SUPERSEDED`).
pub const CLOSE_SUPERSEDED: u32 = 0x5243_0003;

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
/// fails or times out, a read or write on either side fails, or the session
/// ends any way but after stdin's EOF (see the module docs).
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
    let timed_out =
        |what: String| ConnectError::Io(std::io::Error::new(std::io::ErrorKind::TimedOut, what));
    let connection = tokio::time::timeout(DIAL_TIMEOUT, dial(&endpoint, peer, alpn::RCP))
        .await
        .map_err(|_| timed_out(format!("dialing the robot took over {DIAL_TIMEOUT:?}")))??;
    let (send, recv) = tokio::time::timeout(DIAL_TIMEOUT, open_frame_stream(&connection))
        .await
        .map_err(|_| timed_out("opening the rcp stream timed out".into()))??;
    let result = bridge(input, output, send, recv).await;
    let closed = robot_close(&connection);
    connection.close(0u32.into(), b"done");
    endpoint.close().await;
    match closed {
        Some(e) => Err(e),
        None => result,
    }
}

/// The error a robot-side application close of `connection` with
/// [`CLOSE_REVOKED`] or [`CLOSE_SUPERSEDED`] stands for; `None` for any other
/// state.
fn robot_close(connection: &Connection) -> Option<ConnectError> {
    let Some(ConnectionError::ApplicationClosed(close)) = connection.close_reason() else {
        return None;
    };
    let code = close.error_code.into_inner();
    if code == u64::from(CLOSE_REVOKED) {
        Some(ConnectError::Refused(
            "the robot revoked this desk's rcp access".into(),
        ))
    } else if code == u64::from(CLOSE_SUPERSEDED) {
        Some(ConnectError::Refused(
            "the robot closed this rcp connection for a newer one from this desk key".into(),
        ))
    } else {
        None
    }
}

/// Bridge newline-delimited JSON-RPC on `input`/`output` to frames on
/// `send`/`recv`. Blank lines are skipped. Each robot frame passes
/// [`session_line`] before it is written; one holding a raw newline is
/// re-serialized compactly so it stays one line.
///
/// # Errors
///
/// [`ConnectError::Protocol`] for a stdin line over [`MAX_LINE`] or a robot
/// frame that is not a JSON-RPC 2.0 message; [`ConnectError::Refused`] for
/// the robot's refusal; [`ConnectError::Control`] when the robot ends the
/// stream before `input` reaches EOF, or when the stream breaks (a reset, a
/// lost connection, a frame cut off, or a robot frame over the 16 MiB cap),
/// stdin ended or not; [`ConnectError::Io`] for a local read or write failure.
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
    let input_ended = Arc::new(AtomicBool::new(false));
    let ended = input_ended.clone();
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
        // Set before the finish, so the robot's end of the stream that
        // follows it reads as the normal end.
        ended.store(true, Ordering::SeqCst);
        let _ = send.finish();
        Ok::<(), ConnectError>(())
    });
    let downlink = async {
        loop {
            let frame = match read_frame_or_end(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
                Ok(Some(f)) => f,
                Ok(None) => {
                    tracing::debug!("cerulion rcp: the robot finished the session");
                    break;
                }
                Err(e) => {
                    // The chain ends in the robot's own close reason: peer
                    // text, sanitized and bounded here, where it enters.
                    return Err(ConnectError::Control(format!(
                        "the rcp stream from the robot broke: {}",
                        crate::sanitize_peer_text(&error_chain(&e))
                    )));
                }
            };
            let line = session_line(frame)?;
            output.write_all(&line).await?;
            output.write_all(b"\n").await?;
            output.flush().await?;
        }
        if input_ended.load(Ordering::SeqCst) {
            Ok(())
        } else {
            Err(ConnectError::Control(
                "the robot ended the rcp session".into(),
            ))
        }
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

/// `e` and each error under it, joined with `: `.
fn error_chain(e: &LinkError) -> String {
    let mut text = e.to_string();
    let mut source = std::error::Error::source(e);
    while let Some(cause) = source {
        text.push_str(": ");
        text.push_str(&cause.to_string());
        source = cause.source();
    }
    text
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

/// A robot `frame` as the stdout line [`one_line`] makes of it, when it is a
/// JSON-RPC 2.0 message (an object with `"jsonrpc": "2.0"`).
///
/// # Errors
///
/// [`ConnectError::Refused`] with the robot's reason for its refusal of the
/// session (an accept decision `{"decision": "refuse", "reason": ...}`);
/// [`ConnectError::Protocol`] for any other frame.
pub fn session_line(frame: Vec<u8>) -> Result<Vec<u8>, ConnectError> {
    let value: Value = serde_json::from_slice(&frame).map_err(|e| {
        ConnectError::Protocol(format!("the robot sent a frame that is not JSON: {e}"))
    })?;
    if value.get("jsonrpc").and_then(Value::as_str) == Some("2.0") {
        return one_line(frame);
    }
    match value.get("decision").and_then(Value::as_str) {
        Some("refuse") => Err(ConnectError::Refused(crate::sanitize_peer_text(
            value
                .get("reason")
                .and_then(Value::as_str)
                .unwrap_or("no reason given"),
        ))),
        Some(other) => Err(ConnectError::Protocol(format!(
            "the robot answered with the accept decision {:?}, not an rcp session",
            crate::sanitize_peer_text(other)
        ))),
        None => Err(ConnectError::Protocol(
            "the robot sent a frame that is not a JSON-RPC 2.0 message".into(),
        )),
    }
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
    fn the_close_codes_are_the_robots() {
        assert_eq!(CLOSE_REVOKED, 0x5243_0001);
        assert_eq!(CLOSE_SUPERSEDED, 0x5243_0003);
        assert_eq!(CLOSE_REVOKED, cerulion_remoted::rcp::CLOSE_REVOKED);
        assert_eq!(CLOSE_SUPERSEDED, cerulion_remoted::rcp::CLOSE_SUPERSEDED);
    }

    #[test]
    fn session_line_passes_json_rpc_and_nothing_else() {
        let msg = br#"{"jsonrpc":"2.0","id":1,"result":{}}"#;
        assert_eq!(session_line(msg.to_vec()).unwrap(), msg);
        match session_line(br#"{"decision":"refuse","reason":"rcp plane refused: x"}"#.to_vec()) {
            Err(ConnectError::Refused(reason)) => assert_eq!(reason, "rcp plane refused: x"),
            other => panic!("{other:?}"),
        }
        for bad in [
            &br#"{"decision":"rcp_admit"}"#[..],
            br#"{"jsonrpc":"1.0","id":1,"result":{}}"#,
            br#"[{"jsonrpc":"2.0","id":1,"result":{}}]"#,
            b"not json",
        ] {
            assert!(
                matches!(session_line(bad.to_vec()), Err(ConnectError::Protocol(_))),
                "{}",
                String::from_utf8_lossy(bad)
            );
        }
    }

    /// What the stand-in robot does after reading the desk's first frame.
    #[derive(Clone, Copy)]
    enum Robot {
        /// Answer with the refusal decision frame, as the robot does for a
        /// key it does not admit.
        Refuse,
        /// Close the connection with this application code.
        Close(u32),
        /// Answer the request, then end the stream once the desk ends its
        /// side.
        AnswerThenWait,
        /// Answer the request and end the stream at once.
        AnswerThenEnd,
        /// Answer nothing: once the desk ends its side, break the stream
        /// the way the [`Break`] says.
        BreakAfterInput(Break),
    }

    /// How [`Robot::BreakAfterInput`] breaks the stream.
    #[derive(Clone, Copy, Debug)]
    enum Break {
        /// Close the connection with application code 0, which names
        /// neither a revoke nor a newer connection.
        CloseConnection,
        /// Reset the robot's side of the stream.
        ResetStream,
        /// Send a length prefix and one byte of its frame, then finish.
        CutFrame,
    }

    /// Run [`run_rcp`] against a stand-in robot on loopback that plays
    /// `robot`, with one MCP request on stdin; stdin reaches EOF after it
    /// when `stdin_eof`. Returns the result and everything written to
    /// stdout.
    async fn run_against(robot: Robot, stdin_eof: bool) -> (Result<(), ConnectError>, Vec<u8>) {
        use cerulion_link::{accept_frame_stream, accept_one, read_frame};
        let step = Duration::from_secs(20);
        let robot_ep = build_endpoint(
            EndpointConfig::new([71; 32])
                .with_alpns(vec![alpn::RCP.to_vec()])
                .with_relay(RelayConfig::Disabled),
        )
        .await
        .expect("the stand-in binds");
        let direct_addrs = robot_ep
            .bound_sockets()
            .into_iter()
            .filter(SocketAddr::is_ipv4)
            .map(|a| SocketAddr::from((std::net::Ipv4Addr::LOCALHOST, a.port())))
            .collect();
        let config = RcpConfig {
            robot_eid: robot_ep.id(),
            direct_addrs,
            desk_seed: [72; 32],
            relay: RelayConfig::Disabled,
        };
        let served = tokio::spawn(async move {
            let accepted = accept_one(&robot_ep)
                .await
                .expect("accept")
                .expect("a connection");
            let conn = accepted.connection;
            let (mut send, mut recv) = accept_frame_stream(&conn).await.expect("a stream");
            let _ = read_frame(&mut recv, 1 << 20)
                .await
                .expect("the first frame");
            let answer = br#"{"jsonrpc":"2.0","id":1,"result":{}}"#;
            match robot {
                Robot::Refuse => {
                    write_frame(
                        &mut send,
                        br#"{"decision":"refuse","reason":"rcp plane refused: unpaired device key"}"#,
                    )
                    .await
                    .expect("write");
                    let _ = send.finish();
                }
                Robot::Close(code) => conn.close(code.into(), b"closed by the stand-in"),
                Robot::AnswerThenWait => {
                    write_frame(&mut send, answer).await.expect("write");
                    let _ = read_frame(&mut recv, 1 << 20).await;
                    let _ = send.finish();
                }
                Robot::AnswerThenEnd => {
                    write_frame(&mut send, answer).await.expect("write");
                    let _ = send.finish();
                }
                Robot::BreakAfterInput(how) => {
                    // The desk ends its side only after stdin's EOF, so the
                    // bridge has seen stdin end before the stream breaks.
                    let _ = read_frame(&mut recv, 1 << 20).await;
                    match how {
                        Break::CloseConnection => {
                            // The close reason is the robot's own text: it
                            // forges a log line and runs past the 512-char
                            // bound the desk applies to peer text.
                            let mut reason =
                                b"gone\r\ncerulion rcp: the robot finished the session".to_vec();
                            reason.resize(reason.len() + 600, b'x');
                            conn.close(0u32.into(), &reason)
                        }
                        Break::ResetStream => {
                            let _ = send.reset(0u32.into());
                        }
                        Break::CutFrame => {
                            send.write_all(&[64, 0, 0, 0, b'{']).await.expect("write");
                            let _ = send.finish();
                        }
                    }
                }
            }
            let _ = conn.closed().await;
            robot_ep.close().await;
        });
        let (mut stdin, input) = tokio::io::duplex(1 << 16);
        let (output, mut stdout) = tokio::io::duplex(1 << 16);
        stdin
            .write_all(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{}}\n")
            .await
            .unwrap();
        let stdin = if stdin_eof {
            drop(stdin);
            None
        } else {
            Some(stdin)
        };
        let result = tokio::time::timeout(step, run_rcp(config, input, output))
            .await
            .expect("run_rcp returns");
        drop(stdin);
        let mut written = Vec::new();
        stdout.read_to_end(&mut written).await.unwrap();
        let _ = tokio::time::timeout(step, served).await;
        (result, written)
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_refusal_is_an_error_and_never_reaches_stdout() {
        let (result, written) = run_against(Robot::Refuse, false).await;
        match result {
            Err(ConnectError::Refused(reason)) => {
                assert_eq!(reason, "rcp plane refused: unpaired device key")
            }
            other => panic!("expected the robot's refusal, got {other:?}"),
        }
        assert!(
            written.is_empty(),
            "stdout: {}",
            String::from_utf8_lossy(&written)
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_revoked_or_superseded_close_is_an_error() {
        for (code, want) in [
            (0x5243_0001, "the robot revoked this desk's rcp access"),
            (
                0x5243_0003,
                "the robot closed this rcp connection for a newer one from this desk key",
            ),
        ] {
            let (result, written) = run_against(Robot::Close(code), false).await;
            match result {
                Err(ConnectError::Refused(reason)) => assert_eq!(reason, want, "{code:#x}"),
                other => panic!("{code:#x}: expected an error, got {other:?}"),
            }
            assert!(written.is_empty(), "{code:#x}");
        }
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_session_ends_ok_only_after_stdin_ends() {
        let answer = b"{\"jsonrpc\":\"2.0\",\"id\":1,\"result\":{}}\n";
        let (result, written) = run_against(Robot::AnswerThenWait, true).await;
        assert!(result.is_ok(), "{result:?}");
        assert_eq!(written, answer);
        let (result, written) = run_against(Robot::AnswerThenEnd, false).await;
        match result {
            Err(ConnectError::Control(m)) => assert_eq!(m, "the robot ended the rcp session"),
            other => panic!("expected an error, got {other:?}"),
        }
        assert_eq!(written, answer, "frames before the end still reach stdout");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_stream_that_breaks_after_stdin_ends_is_an_error() {
        for how in [Break::CloseConnection, Break::ResetStream, Break::CutFrame] {
            let (result, written) = run_against(Robot::BreakAfterInput(how), true).await;
            match result {
                Err(ConnectError::Control(m)) => {
                    assert!(
                        m.starts_with("the rcp stream from the robot broke: "),
                        "{how:?}: {m}"
                    );
                    if matches!(how, Break::CloseConnection) {
                        assert!(!m.contains('\n') && !m.contains('\r'), "{m}");
                        assert!(
                            m.contains("gone\u{FFFD}\u{FFFD}cerulion rcp: the robot finished"),
                            "{m}"
                        );
                        assert!(m.ends_with("\u{2026}(truncated)"), "{m}");
                    }
                }
                other => panic!("{how:?}: expected an error, got {other:?}"),
            }
            assert!(written.is_empty(), "{how:?}");
        }
    }

    #[test]
    fn trim_line_strips_framing_whitespace_only() {
        assert_eq!(trim_line(b"  {\"a\":1}\r\n"), b"{\"a\":1}");
        assert_eq!(trim_line(b"\n"), b"");
        assert_eq!(trim_line(b"{\"a\":\" \"}\n"), b"{\"a\":\" \"}");
    }
}
