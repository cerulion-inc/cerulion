// SPDX-License-Identifier: MIT OR Apache-2.0
//! In-process loopback tests for the uni-stream frame helpers and the
//! [`QuicOpsStream`] synchronous adapter.
//!
//! Two endpoints in ONE tokio runtime connect over loopback with NO relay and
//! NO discovery (direct addresses only). Every await is bounded by a timeout so
//! CI can never hang. Oracles are LITERAL hand-built byte vectors (distinct
//! patterns, some with leading bytes that look like a length prefix so a
//! double-framing bug would corrupt the compare) — never a round-trip-only
//! self-compare (Principle #13).

use std::error::Error as _;
use std::future::Future;
use std::io::{Read, Write};
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::time::Duration;

use cerulion_link::{
    accept_frame_stream, accept_one, accept_uni_frame_stream, alpn, build_endpoint, dial,
    direct_addr, open_frame_stream, open_uni_frame_stream, read_frame, write_frame, Endpoint,
    EndpointAddr, EndpointConfig, EndpointId, LinkError, QuicOpsStream, RelayConfig,
};

/// Hard ceiling on any single network step — CI must never hang.
const STEP_TIMEOUT: Duration = Duration::from_secs(15);

/// Await `fut`, panicking with `what` if it does not finish within
/// [`STEP_TIMEOUT`].
async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("cerulion_link uni/ops-stream: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

/// Turn an endpoint's bound sockets into a dialable direct [`EndpointAddr`]:
/// iroh binds `0.0.0.0` / `[::]` (unspecified), which are not dialable, so the
/// unspecified addresses are rewritten to their loopback form (same port).
fn loopback_addr(id: EndpointId, bound: &[SocketAddr]) -> EndpointAddr {
    let dialable = bound.iter().map(|s| match s {
        SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
            SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
        }
        SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
            SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
        }
        other => *other,
    });
    direct_addr(id, dialable)
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("endpoint should bind")
}

/// Establish a connection: `client` dials `server` over `alpn`, `server` accepts.
/// Returns `(client_conn, server_conn)`; both must be kept alive by the caller
/// for the streams to stay usable.
async fn connect(
    client: &Endpoint,
    server: &Endpoint,
    server_addr: EndpointAddr,
    alpn: &[u8],
) -> (cerulion_link::Connection, cerulion_link::Connection) {
    let (client_conn, server_accepted) = tokio::join!(
        async {
            bounded("dial", dial(client, server_addr, alpn))
                .await
                .expect("dial ok")
        },
        async {
            bounded("accept_one", accept_one(server))
                .await
                .expect("accept_one ok")
                .expect("an incoming connection")
        },
    );
    assert_eq!(
        server_accepted.alpn.as_slice(),
        alpn,
        "server sees the negotiated ALPN"
    );
    (client_conn, server_accepted.connection)
}

/// A deterministic, hand-computable byte pattern of length `len` keyed by
/// `seed`. Pure integer math — never a round-trip capture — so a swapped or
/// corrupted direction fails the exact-byte compare. Distinct seeds give
/// distinct sequences.
fn patterned(len: usize, seed: u8) -> Vec<u8> {
    (0..len)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed))
        .collect()
}

/// Read EXACTLY `total` bytes from `ops` using a SMALL `chunk`-sized buffer, so
/// the adapter's leftover-cursor path (a QUIC read larger than the caller's
/// buffer) is exercised. Panics on an unexpected EOF.
fn read_exactly_small(ops: &mut QuicOpsStream, total: usize, chunk: usize) -> Vec<u8> {
    let mut got = Vec::with_capacity(total);
    let mut buf = vec![0u8; chunk];
    while got.len() < total {
        let want = chunk.min(total - got.len());
        let n = ops.read(&mut buf[..want]).expect("ops read");
        assert!(n > 0, "unexpected EOF after {} of {total} bytes", got.len());
        got.extend_from_slice(&buf[..n]);
    }
    got
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn loopback_uni_stream_roundtrips_hand_oracle_frames() {
    let server = disabled_endpoint([31u8; 32]).await;
    let client = disabled_endpoint([33u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    // Hand-oracle frames: an EMPTY frame, a small distinct pattern, and a LARGE
    // frame — all literal (not round-trip-only).
    let empty: Vec<u8> = Vec::new();
    let small: Vec<u8> = b"robot->desk uni \x00\x01\x02\x03\xDE\xAD\xBE\xEF".to_vec();
    let large: Vec<u8> = (0u8..=255).cycle().take(100 * 1024).collect();
    let oracle: [Vec<u8>; 3] = [empty.clone(), small.clone(), large.clone()];

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::WIRE).await;

    let send_frames = oracle.clone();
    let client_fut = async {
        // The ROBOT side opens the one-way stream and writes the frames.
        let mut send = bounded("open_uni", open_uni_frame_stream(&client_conn))
            .await
            .expect("open_uni ok");
        for frame in &send_frames {
            bounded("write_frame", write_frame(&mut send, frame))
                .await
                .expect("write_frame ok");
        }
        send.finish().expect("finish send");
    };

    let server_fut = async {
        // The DESK side accepts the one-way stream and reads the frames back.
        let mut recv = bounded("accept_uni", accept_uni_frame_stream(&server_conn))
            .await
            .expect("accept_uni ok");
        let mut got: Vec<Vec<u8>> = Vec::new();
        for _ in 0..3 {
            got.push(
                bounded("read_frame", read_frame(&mut recv, 1024 * 1024))
                    .await
                    .expect("read_frame ok"),
            );
        }
        got
    };

    let ((), got) = tokio::join!(client_fut, server_fut);

    assert_eq!(got.len(), 3, "three frames read back");
    assert_eq!(got[0], empty, "empty frame is byte-identical");
    assert_eq!(got[1], small, "small frame is byte-identical");
    assert_eq!(got[2], large, "large frame is byte-identical");
    assert_eq!(
        got.as_slice(),
        oracle.as_slice(),
        "all frames match the oracle"
    );

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn uni_stream_frame_length_cap_is_enforced_without_wedging() {
    let server = disabled_endpoint([35u8; 32]).await;
    let client = disabled_endpoint([37u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::WIRE).await;

    // A real 2 KiB frame; the reader caps at 1 KiB → it must ERROR loudly (with
    // the exact len/max), not wedge.
    let big_frame: Vec<u8> = vec![0xAB; 2048];
    let to_send = big_frame.clone();

    let client_fut = async {
        let mut send = bounded("open_uni", open_uni_frame_stream(&client_conn))
            .await
            .expect("open_uni ok");
        bounded("write_frame", write_frame(&mut send, &to_send))
            .await
            .expect("write_frame ok");
        send.finish().expect("finish send");
    };
    let server_fut = async {
        let mut recv = bounded("accept_uni", accept_uni_frame_stream(&server_conn))
            .await
            .expect("accept_uni ok");
        bounded("read_frame capped", read_frame(&mut recv, 1024)).await
    };

    let ((), result) = tokio::join!(client_fut, server_fut);
    match result {
        Err(LinkError::FrameExceedsMaxLen { len, max }) => {
            assert_eq!(len, 2048, "the reader saw the real declared length");
            assert_eq!(max, 1024, "the reader's configured cap");
        }
        other => panic!("expected FrameExceedsMaxLen on a uni stream, got {other:?}"),
    }

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quic_ops_stream_moves_bytes_both_directions_intact() {
    let server = disabled_endpoint([21u8; 32]).await;
    let client = disabled_endpoint([23u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    // Distinct literal oracles of DIFFERENT lengths. The leading 4 bytes look
    // like a u32 length prefix — if the adapter (wrongly) framed the bytes, they
    // would be consumed/shifted and the exact-byte compare would fail.
    let client_msg: Vec<u8> = {
        let mut v = vec![0xFF, 0xFF, 0xFF, 0xFF];
        v.extend_from_slice(b"cerud/ops/1::request::");
        v.extend((0u8..250).cycle().take(200));
        v
    };
    let server_msg: Vec<u8> = {
        let mut v = vec![0x08, 0x00, 0x00, 0x00];
        v.extend_from_slice(b"cerud/ops/1::response::");
        v.extend((0u8..100).map(|b| b ^ 0x5A));
        v
    };

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    // Client wraps its bidi stream and runs the SYNC request→response half on a
    // blocking task (the adapter's Read/Write block the calling thread).
    let (c_send, c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    let client_ops = QuicOpsStream::new(c_send, c_recv);
    let (req, expect_resp) = (client_msg.clone(), server_msg.clone());
    let client_task = tokio::task::spawn_blocking(move || {
        let mut ops = client_ops;
        ops.write_all(&req).expect("client write");
        ops.flush().expect("client flush");
        let mut resp = vec![0u8; expect_resp.len()];
        ops.read_exact(&mut resp).expect("client read_exact");
        resp
    });

    // Server accepts the (now data-bearing) stream, reads the request with a
    // SMALL buffer (exercises the leftover cursor), then writes the response.
    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    let server_ops = QuicOpsStream::new(s_send, s_recv);
    let (expect_req, resp) = (client_msg.clone(), server_msg.clone());
    let server_task = tokio::task::spawn_blocking(move || {
        let mut ops = server_ops;
        let got = read_exactly_small(&mut ops, expect_req.len(), 7);
        ops.write_all(&resp).expect("server write");
        ops.flush().expect("server flush");
        got
    });

    let client_resp = bounded("client task", client_task)
        .await
        .expect("client task join");
    let server_got = bounded("server task", server_task)
        .await
        .expect("server task join");

    // Literal-byte oracles — full duplex, byte-for-byte, no framing injected.
    assert_eq!(
        server_got, client_msg,
        "server received the client's exact request bytes"
    );
    assert_eq!(
        client_resp, server_msg,
        "client received the server's exact response bytes"
    );

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quic_ops_stream_dropped_writer_surfaces_clean_eof() {
    let server = disabled_endpoint([41u8; 32]).await;
    let client = disabled_endpoint([43u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let msg: Vec<u8> = b"final-request-before-drop::\x00\xFF\x10\x20".to_vec();

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    // Client writes ONE final message then DROPS the adapter. The drop must drain
    // the queued bytes and cleanly finish() the send stream (EOF), never abort
    // mid-write.
    let (c_send, c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    let client_ops = QuicOpsStream::new(c_send, c_recv);
    let to_send = msg.clone();
    let client_task = tokio::task::spawn_blocking(move || {
        let mut ops = client_ops;
        ops.write_all(&to_send).expect("client write");
        ops.flush().expect("client flush");
        // `ops` drops here → the writer drains `to_send`, finish()es, and exits.
    });

    // Server reads the final message, then the NEXT read must be a clean EOF
    // (Ok(0)) — a dropped remote writer is an end-of-stream, never a desync.
    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    let server_ops = QuicOpsStream::new(s_send, s_recv);
    let expect = msg.clone();
    let server_task = tokio::task::spawn_blocking(move || {
        let mut ops = server_ops;
        let mut buf = vec![0u8; expect.len()];
        ops.read_exact(&mut buf).expect("server read_exact");
        assert_eq!(buf, expect, "server received the final bytes before EOF");
        let mut extra = [0u8; 8];
        let n = ops.read(&mut extra).expect("server read after peer drop");
        assert_eq!(
            n, 0,
            "dropped remote writer surfaces as a clean EOF (Ok(0))"
        );
    });

    bounded("client task", client_task)
        .await
        .expect("client task join");
    bounded("server task", server_task)
        .await
        .expect("server task join");

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn dropping_quic_ops_stream_shuts_down_cleanly_without_hanging() {
    let server = disabled_endpoint([45u8; 32]).await;
    let client = disabled_endpoint([47u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    // Client opens a bidi stream and sends one frame so the server's accept
    // returns (QUIC yields an accepted stream only after data arrives). The
    // client keeps its RAW recv half to directly observe the server's EOF.
    let (mut c_send, mut c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    bounded(
        "write open frame",
        write_frame(&mut c_send, b"open-the-stream"),
    )
    .await
    .expect("write open frame ok");

    // Server wraps the stream and IMMEDIATELY drops it with nothing queued. The
    // drop must not block; the writer closes the send stream cleanly.
    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    let server_ops = QuicOpsStream::new(s_send, s_recv);
    drop(server_ops);

    // The client's recv half must observe a CLEAN EOF within the bounded timeout
    // — proving the dropped adapter shut its writer down (finish()) and nothing
    // hung the runtime.
    let mut buf = [0u8; 16];
    let eof = bounded("client recv EOF after server drop", c_recv.read(&mut buf)).await;
    assert!(
        matches!(eof, Ok(None)),
        "server-side drop must yield a clean EOF, got {eof:?}"
    );

    drop(c_send);
    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quic_ops_stream_multi_chunk_both_directions_byte_exact() {
    let server = disabled_endpoint([51u8; 32]).await;
    let client = disabled_endpoint([53u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    // Several hundred KiB EACH WAY — well beyond one 64 KiB reader chunk (and the
    // 64-item channel cap), so the adapter's chunk-replacement / leftover-cursor
    // reassembly path runs many times. DISTINCT patterns per direction (a hand
    // oracle), so a swapped-direction or reassembly bug fails the byte compare.
    const SIZE: usize = 384 * 1024;
    let c2s = patterned(SIZE, 0x11); // client -> server
    let s2c = patterned(SIZE, 0xA7); // server -> client
    assert_ne!(c2s, s2c, "the two directions carry distinct sequences");

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    // Both sides write their big payload (buffered to the writer task) then read
    // the peer's big payload with a SMALL buffer (drives the leftover cursor).
    let (c_send, c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    let client_ops = QuicOpsStream::new(c_send, c_recv);
    let (c_write, c_expect) = (c2s.clone(), s2c.clone());
    let client_task = tokio::task::spawn_blocking(move || {
        let mut ops = client_ops;
        ops.write_all(&c_write).expect("client write");
        ops.flush().expect("client flush");
        read_exactly_small(&mut ops, c_expect.len(), 1000)
    });

    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    let server_ops = QuicOpsStream::new(s_send, s_recv);
    let (s_write, s_expect) = (s2c.clone(), c2s.clone());
    let server_task = tokio::task::spawn_blocking(move || {
        let mut ops = server_ops;
        ops.write_all(&s_write).expect("server write");
        ops.flush().expect("server flush");
        read_exactly_small(&mut ops, s_expect.len(), 1000)
    });

    let client_got = bounded("client task", client_task)
        .await
        .expect("client task join");
    let server_got = bounded("server task", server_task)
        .await
        .expect("server task join");

    assert_eq!(
        server_got.len(),
        SIZE,
        "server read the full multi-chunk payload"
    );
    assert_eq!(
        client_got.len(),
        SIZE,
        "client read the full multi-chunk payload"
    );
    assert_eq!(
        server_got, c2s,
        "server received the client's exact patterned bytes across many chunks"
    );
    assert_eq!(
        client_got, s2c,
        "client received the server's exact patterned bytes across many chunks"
    );

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quic_ops_stream_write_after_peer_close_returns_sourced_broken_pipe() {
    let server = disabled_endpoint([55u8; 32]).await;
    let client = disabled_endpoint([57u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    let (c_send, c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    let client_ops = QuicOpsStream::new(c_send, c_recv);

    // Spawn the client's write loop FIRST: its first write opens the stream on
    // the wire (the server's accept_bi returns only once data arrives). The loop
    // keeps writing until a write ERRORS, returning that FIRST error — a live-
    // but-dead peer must never let writes succeed forever (bounded so CI can't
    // hang: <= 2000 * 1ms ~= 2s).
    let client_task = tokio::task::spawn_blocking(move || {
        let mut ops = client_ops;
        for _ in 0..2000 {
            match ops.write_all(b"payload-after-peer-close::\x00\x01\x02\xFF") {
                Ok(()) => {
                    let _ = ops.flush();
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => return Some(e),
            }
        }
        None
    });

    // Server accepts the (now data-bearing) stream, then DROPS BOTH halves.
    // Dropping the recv half STOP_SENDINGs the client's send direction, so the
    // client's writes start failing.
    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    drop(s_recv);
    drop(s_send);

    let err = bounded("client task", client_task)
        .await
        .expect("client task join")
        .expect("a write after the peer closed must eventually error, not succeed forever");
    assert_eq!(
        err.kind(),
        std::io::ErrorKind::BrokenPipe,
        "a dead peer surfaces a BrokenPipe-class write error, got {err:?}"
    );
    // Fix 5b: the FIRST post-close write chains the underlying QUIC error as its
    // source — a diagnosable cause chain, not a cause-free static string.
    assert!(
        err.source().is_some(),
        "the BrokenPipe must carry the underlying QUIC cause as its source, got {err:?}"
    );

    drop(client_conn);
    drop(server_conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn quic_ops_stream_peer_reset_surfaces_connection_reset_then_eof() {
    let server = disabled_endpoint([61u8; 32]).await;
    let client = disabled_endpoint([63u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let (client_conn, server_conn) = connect(&client, &server, server_addr, alpn::OPS).await;

    // Client writes some RAW bytes on its send stream, then abruptly RESETs it
    // (a stream error — NOT a clean `finish()`). Uses a raw `SendStream` so it
    // can reset; the server side wraps its accepted stream in the adapter.
    let (mut c_send, _c_recv) = bounded("open_bi", open_frame_stream(&client_conn))
        .await
        .expect("open_bi ok");
    bounded(
        "client raw write",
        c_send.write_all(b"raw-bytes-before-reset::\x00\x11\x22\x33"),
    )
    .await
    .expect("client raw write ok");
    // RESET_STREAM (an abrupt abort) — the server's reader must see this as a
    // stream ERROR, distinct from a graceful finish.
    c_send.reset(7u32.into()).expect("reset send stream");

    // Server wraps the accepted stream. Drain whatever bytes arrive; the stream
    // MUST terminate with an `Err` (the reset), NEVER a clean `Ok(0)` — that is
    // exactly the contract (a reset peer is distinguishable from a graceful close).
    let (s_send, s_recv) = bounded("accept_bi", accept_frame_stream(&server_conn))
        .await
        .expect("accept_bi ok");
    let server_ops = QuicOpsStream::new(s_send, s_recv);
    let server_task = tokio::task::spawn_blocking(move || {
        let mut ops = server_ops;
        let mut buf = [0u8; 64];
        // A reset never finishes cleanly, so this loop must break on `Err`; an
        // `Ok(0)` here would collapse the reset into an EOF.
        let terminal: Result<std::io::Error, &'static str> = loop {
            match ops.read(&mut buf) {
                Ok(0) => break Err("a peer RESET was surfaced as a clean EOF (Ok(0))"),
                Ok(_delivered) => continue,
                Err(e) => break Ok(e),
            }
        };
        let err = terminal.expect("terminal read");
        // The one-shot error is consumed; the NEXT read is a clean EOF.
        let mut after = [0u8; 8];
        let eof = ops
            .read(&mut after)
            .expect("Ok(0) EOF after the one-shot reset error");
        (err, eof)
    });

    let (err, eof) = bounded("server task", server_task)
        .await
        .expect("server task join");
    assert_eq!(
        err.kind(),
        std::io::ErrorKind::ConnectionReset,
        "a reset recv stream surfaces ConnectionReset, got {err:?}"
    );
    assert!(
        err.source().is_some(),
        "the ConnectionReset must chain the underlying QUIC cause as its source, got {err:?}"
    );
    assert_eq!(
        eof, 0,
        "the reset error fires exactly once, then a clean EOF"
    );

    drop(client_conn);
    drop(server_conn);
}
