// SPDX-License-Identifier: MIT OR Apache-2.0
//! In-process loopback tests for `cerulion_link`.
//!
//! Two endpoints in ONE tokio runtime connect over loopback with NO relay and
//! NO discovery (direct addresses only), round-trip hand-oracle frames through
//! the length-prefixed framing helpers, and cross-check that each side's
//! mutually-authenticated `remote_id()` equals the OTHER's device public key —
//! derived INDEPENDENTLY from the secret via `iroh::SecretKey` (an oracle, not
//! a self-compare). Every await is bounded by a timeout so CI can never hang.
//!
//! The endpoints and connections are held in the OUTER scope (borrowed into the
//! two concurrent halves, connections returned out) for the whole test, so
//! nothing is dropped mid-flight — no fragile graceful-close handshake is
//! needed to guarantee delivery.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::time::Duration;

use cerulion_link::{
    accept_frame_stream, accept_one, alpn, assert_peer_identity, build_endpoint, dial, direct_addr,
    open_frame_stream, read_frame, write_frame, EndpointConfig, RelayConfig,
};
use iroh::{Endpoint, EndpointAddr, EndpointId, SecretKey};

/// Hard ceiling on any single network step — CI must never hang.
const STEP_TIMEOUT: Duration = Duration::from_secs(15);

/// Await `fut`, panicking with `what` if it does not finish within
/// [`STEP_TIMEOUT`].
async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("cerulion_link loopback: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

/// The public key an `iroh::SecretKey::from_bytes(secret)` yields — the
/// independent identity oracle (does not go through `cerulion_link`).
fn public_key_of(secret: &[u8; 32]) -> [u8; 32] {
    *SecretKey::from_bytes(secret).public().as_bytes()
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

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn loopback_dial_by_key_roundtrips_frames_and_authenticates_both_directions() {
    // Deterministic device secrets → deterministic identities.
    let server_secret = [7u8; 32];
    let client_secret = [9u8; 32];

    // Independent identity oracle (via iroh::SecretKey, not cerulion_link).
    let server_pub = public_key_of(&server_secret);
    let client_pub = public_key_of(&client_secret);

    let server = disabled_endpoint(server_secret).await;
    let client = disabled_endpoint(client_secret).await;

    // The endpoint's own id must equal the independently-derived public key.
    assert_eq!(server.id().as_bytes(), &server_pub, "server id derivation");
    assert_eq!(client.id().as_bytes(), &client_pub, "client id derivation");

    // Direct dial target — no relay, no discovery.
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    // Hand-oracle frames (stand in for raw Cerulion wire frames).
    let ping: Vec<u8> = b"cerulion/wire/1::ping::\x00\x01\x02\x03frame".to_vec();
    let pong: Vec<u8> = b"cerulion/wire/1::pong::\xDE\xAD\xBE\xEFframe".to_vec();

    // Borrow the endpoints into the two halves so they stay owned (and alive)
    // in this outer scope for the whole test.
    let server_ref = &server;
    let client_ref = &client;

    let server_fut = {
        let ping = ping.clone();
        let pong = pong.clone();
        async move {
            let accepted = bounded("accept_one", accept_one(server_ref))
                .await
                .expect("accept_one ok")
                .expect("an incoming connection");

            // Negotiated plane + authenticated identity of the DIALER (client).
            assert_eq!(accepted.alpn.as_slice(), alpn::WIRE, "negotiated ALPN");
            assert_eq!(
                accepted.remote_id.as_bytes(),
                &client_pub,
                "server sees the client's authenticated key"
            );
            let confirmed = assert_peer_identity(&accepted.connection, &client_pub)
                .expect("client identity matches");
            assert_eq!(confirmed.as_bytes(), &client_pub);
            // A wrong expected key must be rejected loudly.
            assert!(
                assert_peer_identity(&accepted.connection, &server_pub).is_err(),
                "mismatched expected key must error"
            );

            let (mut send, mut recv) =
                bounded("accept_bi", accept_frame_stream(&accepted.connection))
                    .await
                    .expect("accept_bi ok");
            let got = bounded("read ping", read_frame(&mut recv, 4096))
                .await
                .expect("read ping ok");
            assert_eq!(got, ping, "server received the client's ping frame");

            bounded("write pong", write_frame(&mut send, &pong))
                .await
                .expect("write pong ok");
            send.finish().expect("finish send");

            // Return the connection so it stays alive until the test ends.
            (accepted.connection, accepted.remote_id)
        }
    };

    let client_fut = {
        let ping = ping.clone();
        let pong = pong.clone();
        async move {
            let conn = bounded("dial", dial(client_ref, server_addr, alpn::WIRE))
                .await
                .expect("dial ok");

            // The dialer sees the SERVER's authenticated key.
            let remote = conn.remote_id();
            assert_eq!(
                remote.as_bytes(),
                &server_pub,
                "client sees the server's authenticated key"
            );
            assert_peer_identity(&conn, &server_pub).expect("server identity matches");

            let (mut send, mut recv) = bounded("open_bi", open_frame_stream(&conn))
                .await
                .expect("open_bi ok");
            bounded("write ping", write_frame(&mut send, &ping))
                .await
                .expect("write ping ok");
            let got = bounded("read pong", read_frame(&mut recv, 4096))
                .await
                .expect("read pong ok");
            assert_eq!(got, pong, "client received the server's pong frame");

            (conn, remote)
        }
    };

    let ((_s_conn, server_saw), (_c_conn, client_saw)) = tokio::join!(server_fut, client_fut);
    assert_eq!(
        server_saw.as_bytes(),
        &client_pub,
        "server's returned remote_id == client key"
    );
    assert_eq!(
        client_saw.as_bytes(),
        &server_pub,
        "client's returned remote_id == server key"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn frame_too_large_is_rejected_before_allocation() {
    // A stream where the writer sends a hostile 4-byte length prefix
    // (u32::MAX) and no payload; the reader must refuse via the max-len guard
    // rather than trying to allocate ~4 GiB.
    let server = disabled_endpoint([11u8; 32]).await;
    let client = disabled_endpoint([13u8; 32]).await;
    let server_addr = loopback_addr(server.id(), &server.bound_sockets());

    let server_ref = &server;
    let client_ref = &client;

    let server_fut = async move {
        let accepted = bounded("accept_one", accept_one(server_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        let (_send, mut recv) = bounded("accept_bi", accept_frame_stream(&accepted.connection))
            .await
            .expect("accept_bi ok");
        // Cap the read at 1 KiB — the incoming length prefix claims u32::MAX.
        let result = bounded("read hostile", read_frame(&mut recv, 1024)).await;
        (accepted.connection, result)
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, server_addr, alpn::WIRE))
            .await
            .expect("dial ok");
        let (mut send, _recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi ok");
        // Raw hostile length prefix, no payload.
        bounded("write prefix", async {
            send.write_all(&u32::MAX.to_le_bytes()).await
        })
        .await
        .expect("write prefix ok");
        send.finish().expect("finish");
        conn
    };

    let ((_s_conn, read_result), _c_conn) = tokio::join!(server_fut, client_fut);
    match read_result {
        // The READ side surfaces the caller's cap, not the wire limit.
        Err(cerulion_link::LinkError::FrameExceedsMaxLen { len, max }) => {
            assert_eq!(len, u32::MAX as usize);
            assert_eq!(max, 1024);
        }
        other => panic!("expected FrameExceedsMaxLen, got {other:?}"),
    }
}

#[test]
fn alpn_constants_are_the_documented_wire_values() {
    // Hand oracle — the exact bytes on the wire.
    assert_eq!(alpn::WIRE, b"cerulion/wire/1");
    assert_eq!(alpn::OPS, b"cerulion/ops/1");
    assert_eq!(
        alpn::ALL,
        &[b"cerulion/wire/1".as_slice(), b"cerulion/ops/1"]
    );
}

#[test]
fn endpoint_config_defaults_to_n0_relays_and_both_planes() {
    let cfg = EndpointConfig::new([1u8; 32]);
    assert!(matches!(cfg.relay, RelayConfig::N0Default));
    assert_eq!(cfg.alpns, vec![alpn::WIRE.to_vec(), alpn::OPS.to_vec()]);
    assert_eq!(cfg.secret_key, [1u8; 32]);
}
