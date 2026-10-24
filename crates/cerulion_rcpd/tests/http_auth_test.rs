// SPDX-License-Identifier: AGPL-3.0-only
//! The streamable HTTP transport admits only the configured bearer token.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::http::{BearerToken, HttpServer};
use cerulion_rcpd::manifest::RobotIdentity;
use cerulion_rcpd::observer::GraphSource;
use cerulion_rcpd::RcpdError;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

const TOKEN: &str = "fixture-token-1862";
const INITIALIZE: &str = r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"probe","version":"0"}}}"#;

static COUNTER: AtomicU64 = AtomicU64::new(0);

async fn status_line(addr: std::net::SocketAddr, authorization: Option<&str>) -> String {
    let mut stream = TcpStream::connect(addr).await.expect("connect");
    let auth = authorization
        .map(|value| format!("Authorization: {value}\r\n"))
        .unwrap_or_default();
    let request = format!(
        "POST /mcp HTTP/1.1\r\nHost: {addr}\r\n{auth}Content-Type: application/json\r\n\
         Accept: application/json, text/event-stream\r\nContent-Length: {}\r\n\r\n{INITIALIZE}",
        INITIALIZE.len()
    );
    stream.write_all(request.as_bytes()).await.expect("send");
    let mut buf = Vec::new();
    let mut chunk = [0u8; 512];
    while !buf.windows(2).any(|w| w == b"\r\n") {
        let n = tokio::time::timeout(Duration::from_secs(10), stream.read(&mut chunk))
            .await
            .expect("response within 10 s")
            .expect("read");
        assert!(n > 0, "connection closed before a status line");
        buf.extend_from_slice(&chunk[..n]);
    }
    let text = String::from_utf8_lossy(&buf);
    text.lines().next().unwrap_or_default().to_string()
}

#[tokio::test(flavor = "multi_thread")]
async fn only_the_configured_bearer_token_reaches_the_mcp_layer() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!(
                "rcpd_http_{}_{}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager");
    let source = GraphSource::new(
        mgr,
        RobotIdentity {
            robot: "http_bot".into(),
            nodes: Vec::new(),
        },
    );
    let token = BearerToken::from_env_value(Some(TOKEN.into())).expect("token");
    let server = HttpServer::bind("127.0.0.1:0".parse().expect("addr"), token, source.feed())
        .await
        .expect("bind");
    let addr = server.local_addr().expect("local addr");
    let (stop_tx, stop_rx) = tokio::sync::oneshot::channel::<()>();
    let serving = tokio::spawn(server.serve(async {
        let _ = stop_rx.await;
    }));

    assert_eq!(status_line(addr, None).await, "HTTP/1.1 401 Unauthorized");
    assert_eq!(
        status_line(addr, Some("Bearer wrong-token-of-any-len")).await,
        "HTTP/1.1 401 Unauthorized"
    );
    assert_eq!(
        status_line(addr, Some(&format!("Basic {TOKEN}"))).await,
        "HTTP/1.1 401 Unauthorized"
    );
    assert_eq!(
        status_line(addr, Some(&format!("Bearer {TOKEN}x"))).await,
        "HTTP/1.1 401 Unauthorized"
    );
    assert_eq!(
        status_line(addr, Some(&format!("Bearer {TOKEN}"))).await,
        "HTTP/1.1 200 OK"
    );

    stop_tx.send(()).expect("stop");
    serving.await.expect("server task");
}

#[test]
fn a_missing_or_blank_token_is_refused() {
    for value in [None, Some(String::new()), Some("   ".to_string())] {
        let err = BearerToken::from_env_value(value.clone()).expect_err("refused");
        assert!(matches!(err, RcpdError::MissingToken), "{value:?}: {err:?}");
    }
}

#[test]
fn the_token_never_prints() {
    let token = BearerToken::from_env_value(Some(TOKEN.into())).expect("token");
    assert!(!format!("{token:?}").contains(TOKEN));
}
