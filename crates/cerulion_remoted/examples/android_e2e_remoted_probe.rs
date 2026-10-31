// SPDX-License-Identifier: AGPL-3.0-only
//! `android_e2e_remoted_probe`: the Android emulator e2e's remote metadata probe.
//!
//! It runs ON the device, beside the network daemon the e2e started. In one
//! process it serves the wire plane the way `cerulion-remoted` does
//! (`WirePlane::lazy` with the local schema provider on the default netd
//! control socket, accepts classified by a `PairingAuthorizer` through
//! `handle_accepted_with_wire`) and dials it from a paired desk endpoint over
//! loopback iroh with the relay disabled. The desk sends a `catalog` and a
//! `schema` request; the robot answers each by reading the netd serving
//! snapshot through the deadline-bounded control connect.
//!
//! Usage: `android_e2e_remoted_probe served <topic> <schema>` or
//! `android_e2e_remoted_probe absent <topic>`.
//!
//! `served`: netd is running and a graph serves `<topic>` as `<schema>`. The
//! catalog must list `<topic>` with schema name `<schema>`, and the schema
//! reply must name a type for the topic from netd's snapshot (a topic binding
//! or the wire-hash binding of its live frames): either the closure with
//! `<schema>` first, or, for a builtin type, the source's "no custom schema
//! closure" answer. A source that names no type for the topic gives a
//! different answer. Where control
//! sockets live in the abstract namespace (Android), no file may exist at the
//! socket path, so the reads can only have come over an abstract connect.
//!
//! `absent`: no netd is running. Both requests must fail with the provider's
//! "cannot reach the local network daemon" error.
//!
//! It prints ONE fact line and exits 0 when every check holds:
//!
//! ```text
//! remoted_probe mode=served topic=/basic_timer/timer/count catalog_rows=1 catalog_schema=std_msgs/Int32 schema_reply=builtin control_socket=abstract
//! ```
//!
//! Any failed check prints `FAIL <reason>` and exits 1 with no fact line.

use std::error::Error;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::process::ExitCode;
use std::sync::Arc;
use std::time::Duration;

use cerulion_link::{
    accept_one, alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame,
    write_frame, EndpointConfig, RecvStream, RelayConfig, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, PairingAuthorizer, RemotedClock, SharedTrust,
    WirePlane, WireRequest, WireResponse,
};
use ed25519_dalek::SigningKey;

/// The identity the probe's wire plane self-attributes in every reply.
const ROBOT: &str = "android-e2e-remoted-probe";
/// Fixed fixture identities: an owner account, a robot id, a chassis secret
/// and the clock the trust store is read at.
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);
const CHASSIS: &[u8] = b"android-e2e-remoted-probe-chassis-secret";
const T_NOW: u64 = 1_000_000_000_000;
/// The bound on every step (endpoint bind, dial, one request and its reply).
const STEP: Duration = Duration::from_secs(20);
/// The provider's error when no daemon answers on the control socket.
const NO_DAEMON: &str = "cannot reach the local network daemon";

type Result<T> = std::result::Result<T, Box<dyn Error>>;

enum Mode {
    Served { topic: String, schema: String },
    Absent { topic: String },
}

fn parse(args: &[String]) -> Result<Mode> {
    match args {
        [mode, topic, schema] if mode == "served" => Ok(Mode::Served {
            topic: topic.clone(),
            schema: schema.clone(),
        }),
        [mode, topic] if mode == "absent" => Ok(Mode::Absent {
            topic: topic.clone(),
        }),
        _ => {
            Err("usage: android_e2e_remoted_probe served <topic> <schema> | absent <topic>".into())
        }
    }
}

async fn bounded<F: std::future::Future>(what: &str, future: F) -> Result<F::Output> {
    tokio::time::timeout(STEP, future)
        .await
        .map_err(|_| format!("{what} did not finish within {STEP:?}").into())
}

/// An endpoint's bound sockets with unspecified addresses made dialable.
fn loopback(bound: &[SocketAddr]) -> Vec<SocketAddr> {
    bound
        .iter()
        .map(|socket| match socket {
            SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
            }
            SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
            }
            other => *other,
        })
        .collect()
}

/// One request on the control stream and its reply, each frame bounded.
async fn exchange(
    send: &mut SendStream,
    recv: &mut RecvStream,
    request: WireRequest,
) -> Result<WireResponse> {
    let bytes = serde_json::to_vec(&request)?;
    bounded("send request", write_frame(send, &bytes)).await??;
    let reply = bounded("receive reply", read_frame(recv, DEFAULT_MAX_FRAME_LEN)).await??;
    Ok(serde_json::from_slice(&reply)?)
}

async fn run(mode: Mode) -> Result<String> {
    let socket = cerulion_netd::default_socket_path();
    let robot_endpoint = bounded(
        "robot endpoint bind",
        build_endpoint(EndpointConfig::new([71; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await??;
    let desk_endpoint = bounded(
        "desk endpoint bind",
        build_endpoint(EndpointConfig::new([72; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await??;

    // An owner-claimed trust store with the desk's endpoint key bound to the
    // owner: an established pairing, checked by the real accept gate.
    let root = PublicKey(SigningKey::from_bytes(&[1; 32]).verifying_key().to_bytes());
    let mut store = TrustStore::provision(
        ROBOT_ID,
        PublicKey(*robot_endpoint.id().as_bytes()),
        RootSet::new(vec![root], 1)?,
        CHASSIS,
        0,
    )?;
    store.claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)?;
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(*desk_endpoint.id().as_bytes()), OWNER);
    let clock = RemotedClock::fixed(T_NOW);
    let shared = SharedTrust::new_with_clock(store, index, Vec::new(), clock);
    let authorizer = Arc::new(PairingAuthorizer::from_shared(shared));
    // The production wire plane's shape: a lazy transport manager and the
    // local schema provider on the default control socket.
    let plane = Arc::new(
        WirePlane::lazy(ROBOT)
            .with_local_schema_provider(socket.clone())
            .with_demand_authorizer(authorizer.clone()),
    );
    let serving = robot_endpoint.clone();
    let accept = tokio::spawn(async move {
        if let Ok(Some(accepted)) = accept_one(&serving).await {
            handle_accepted_with_wire(accepted, authorizer, None, Some(plane)).await;
        }
    });

    let address = direct_addr(
        robot_endpoint.id(),
        loopback(&robot_endpoint.bound_sockets()),
    );
    let connection = bounded("dial", dial(&desk_endpoint, address, alpn::WIRE)).await??;
    let (mut send, mut recv) =
        bounded("open control stream", open_frame_stream(&connection)).await??;
    let fact = match mode {
        Mode::Served { topic, schema } => {
            let catalog = exchange(&mut send, &mut recv, WireRequest::Catalog).await?;
            let WireResponse::Catalog(catalog) = catalog else {
                return Err(format!("catalog: expected a catalog, got {catalog:?}").into());
            };
            if let Some(error) = &catalog.error {
                return Err(format!("catalog: refused: {error}").into());
            }
            let rows: Vec<_> = catalog
                .entries
                .iter()
                .filter(|row| row.topic == topic)
                .collect();
            let [row] = rows.as_slice() else {
                return Err(format!(
                    "catalog: expected one row for {topic}, got {} in {:?}",
                    rows.len(),
                    catalog.entries
                )
                .into());
            };
            if row.schema_name.as_deref() != Some(schema.as_str()) {
                return Err(format!(
                    "catalog: {topic} schema name {:?}, expected {schema}",
                    row.schema_name
                )
                .into());
            }
            let reply = exchange(
                &mut send,
                &mut recv,
                WireRequest::Schema {
                    topic: topic.clone(),
                },
            )
            .await?;
            let WireResponse::Schema(reply) = reply else {
                return Err(format!("schema: expected a schema reply, got {reply:?}").into());
            };
            if reply.robot != ROBOT || reply.requested != topic {
                return Err(format!(
                    "schema: reply for robot {} topic {}, expected {ROBOT} {topic}",
                    reply.robot, reply.requested
                )
                .into());
            }
            let shape = match (reply.docs.first(), reply.error.as_deref()) {
                (Some(root), None) if root.qualified == schema => "closure",
                (None, Some(error)) if error.contains("no custom schema closure") => "builtin",
                _ => {
                    return Err(
                        format!("schema: {topic} did not resolve to {schema}: {reply:?}").into(),
                    );
                }
            };
            let control = if cfg!(target_os = "android") {
                if socket.exists() {
                    return Err(format!(
                        "control socket: a file exists at {} where the daemon binds an abstract name",
                        socket.display()
                    )
                    .into());
                }
                "abstract"
            } else {
                "pathname"
            };
            format!(
                "remoted_probe mode=served topic={topic} catalog_rows={} catalog_schema={schema} \
                 schema_reply={shape} control_socket={control}",
                catalog.entries.len()
            )
        }
        Mode::Absent { topic } => {
            for (label, wire_request) in [
                ("catalog", WireRequest::Catalog),
                (
                    "schema",
                    WireRequest::Schema {
                        topic: topic.clone(),
                    },
                ),
            ] {
                match exchange(&mut send, &mut recv, wire_request).await? {
                    WireResponse::Error { message, .. } if message.contains(NO_DAEMON) => {}
                    other => {
                        return Err(format!(
                            "{label}: expected an error containing '{NO_DAEMON}', got {other:?}"
                        )
                        .into());
                    }
                }
            }
            format!("remoted_probe mode=absent topic={topic} catalog=refused schema=refused")
        }
    };

    connection.close(0u32.into(), b"probe finished");
    bounded("robot endpoint close", robot_endpoint.close()).await?;
    bounded("desk endpoint close", desk_endpoint.close()).await?;
    bounded("robot session exit", accept).await??;
    Ok(fact)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let outcome = parse(&args).and_then(|mode| {
        tokio::runtime::Builder::new_multi_thread()
            .worker_threads(4)
            .enable_all()
            .build()?
            .block_on(run(mode))
    });
    match outcome {
        Ok(fact) => {
            println!("{fact}");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("FAIL {error}");
            ExitCode::from(1)
        }
    }
}
