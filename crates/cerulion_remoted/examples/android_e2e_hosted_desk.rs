// SPDX-License-Identifier: AGPL-3.0-only
//! `android_e2e_hosted_desk`: the Android emulator e2e's paired desk for the
//! remote plane an app hosts.
//!
//! It runs ON the device. `provision <dir>` writes the files an app's plane
//! reads from `<dir>/remoted/`: a device key from a fixed seed, a trust-store
//! MAC key, an owner-claimed trust store bound to that device key, and a
//! device index that pairs this desk's key with the owner. The e2e copies them
//! into the app's files directory.
//!
//! `read <endpoint id> <port> <topic> <schema>` dials the app's plane on
//! loopback with the relay disabled, as the paired desk. The endpoint id must
//! be the provisioned device key's public half. The catalog must list `<topic>`
//! as `<schema>`, and a demand for it must stream one whole wire frame whose
//! header names its size and the catalog's schema hash.
//!
//! It prints ONE fact line and exits 0 when every check holds:
//!
//! ```text
//! hosted_desk mode=read topic=/app/heartbeat catalog_schema=std_msgs/String frames=1 endpoint=provisioned
//! ```
//!
//! Any failed check prints `FAIL <reason>` and exits 1 with no fact line.

use std::error::Error;
use std::net::{Ipv4Addr, SocketAddr};
use std::path::Path;
use std::process::ExitCode;
use std::time::Duration;

use cerulion_core::wire::WireHeader;
use cerulion_link::{
    accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr, open_frame_stream,
    read_frame, write_frame, EndpointConfig, EndpointId, RecvStream, RelayConfig, SendStream,
    DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{DeviceAccountIndex, StreamPreamble, WireRequest, WireResponse};
use ed25519_dalek::SigningKey;

/// The app's device key seed: its public half is the plane's endpoint id.
const DEVICE_SEED: u8 = 91;
/// This desk's key seed; the device index pairs it with the owner.
const DESK_SEED: u8 = 92;
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);
const CHASSIS: &[u8] = b"android-e2e-hosted-desk-chassis-secret";
const MAC_KEY: &[u8] = b"android-e2e-hosted-desk-trust-store-mac-key";
const T_NOW: u64 = 1_000_000_000_000;
/// The bound on every step (endpoint bind, dial, one request, one frame).
const STEP: Duration = Duration::from_secs(20);

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn public(seed: u8) -> PublicKey {
    PublicKey(
        SigningKey::from_bytes(&[seed; 32])
            .verifying_key()
            .to_bytes(),
    )
}

fn provision(dir: &Path) -> Result<String> {
    let remoted = dir.join("remoted");
    std::fs::create_dir_all(&remoted)?;
    std::fs::write(remoted.join("device_key"), [DEVICE_SEED; 32])?;
    std::fs::write(remoted.join("trust_store.mac_key"), MAC_KEY)?;
    let mut store = TrustStore::provision(
        ROBOT_ID,
        public(DEVICE_SEED),
        RootSet::new(vec![public(1)], 1)?,
        CHASSIS,
        0,
    )?
    .with_path(remoted.join("trust_store"));
    store.claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)?;
    store.save(MAC_KEY)?;
    let mut index = DeviceAccountIndex::new().with_path(remoted.join("device_index.json"));
    index.bind(public(DESK_SEED), OWNER);
    index.save(MAC_KEY)?;
    Ok(format!(
        "hosted_desk mode=provision files=device_key,trust_store.mac_key,trust_store,device_index.json endpoint={}",
        hex(&public(DEVICE_SEED).0)
    ))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

async fn bounded<F: std::future::Future>(what: &str, future: F) -> Result<F::Output> {
    tokio::time::timeout(STEP, future)
        .await
        .map_err(|_| format!("{what} did not finish within {STEP:?}").into())
}

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

async fn read(endpoint: &str, port: u16, topic: &str, schema: &str) -> Result<String> {
    if endpoint != hex(&public(DEVICE_SEED).0) {
        return Err(format!("endpoint {endpoint} is not the provisioned device key").into());
    }
    let id: EndpointId = endpoint.parse()?;
    let desk = bounded(
        "desk endpoint bind",
        build_endpoint(EndpointConfig::new([DESK_SEED; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await??;
    let address = direct_addr(id, [SocketAddr::from((Ipv4Addr::LOCALHOST, port))]);
    let connection = bounded("dial", dial(&desk, address, alpn::WIRE)).await??;
    let (mut send, mut recv) =
        bounded("open control stream", open_frame_stream(&connection)).await??;

    let WireResponse::Catalog(catalog) =
        exchange(&mut send, &mut recv, WireRequest::Catalog).await?
    else {
        return Err("catalog: the reply is not a catalog".into());
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
            "catalog: expected one row for {topic}, got {:?}",
            catalog.entries
        )
        .into());
    };
    let catalog_hash = row.schema_hash;
    if row.schema_name.as_deref() != Some(schema) {
        return Err(format!(
            "catalog: {topic} schema name {:?}, expected {schema}",
            row.schema_name
        )
        .into());
    }

    let demand = WireRequest::Demand {
        topic: topic.to_string(),
    };
    match exchange(&mut send, &mut recv, demand).await? {
        WireResponse::DemandAccepted { topic: accepted } if accepted == topic => {}
        other => return Err(format!("demand: expected acceptance, got {other:?}").into()),
    }
    let mut stream = bounded("data stream", accept_uni_frame_stream(&connection)).await??;
    let preamble: StreamPreamble = serde_json::from_slice(
        &bounded("preamble", read_frame(&mut stream, DEFAULT_MAX_FRAME_LEN)).await??,
    )?;
    if preamble.topic != topic {
        return Err(format!("preamble: topic {}, expected {topic}", preamble.topic).into());
    }
    let frame = bounded(
        "first frame",
        read_frame(&mut stream, DEFAULT_MAX_FRAME_LEN),
    )
    .await??;
    // The frame must be one whole wire frame of the topic's schema: its header
    // names its own size and, when the catalog saw the topic's frames, the
    // same schema hash.
    let Some(header) = WireHeader::read_from_buf(&frame) else {
        return Err(format!("frame: {} bytes, shorter than a wire header", frame.len()).into());
    };
    if frame.len() <= WireHeader::SIZE || header.total_size as usize != frame.len() {
        return Err(format!(
            "frame: {} bytes, but its header says {} (a body must follow the header)",
            frame.len(),
            header.total_size
        )
        .into());
    }
    if header.schema_hash == 0 || catalog_hash.is_some_and(|hash| hash != header.schema_hash) {
        return Err(format!(
            "frame: schema hash {:#x}, the catalog row says {catalog_hash:?}",
            header.schema_hash
        )
        .into());
    }
    connection.close(0u32.into(), b"desk finished");
    bounded("desk endpoint close", desk.close()).await?;
    Ok(format!(
        "hosted_desk mode=read topic={topic} catalog_schema={schema} frames=1 endpoint=provisioned"
    ))
}

fn run(args: &[String]) -> Result<String> {
    match args {
        [mode, dir] if mode == "provision" => provision(Path::new(dir)),
        [mode, endpoint, port, topic, schema] if mode == "read" => {
            let port: u16 = port.parse()?;
            tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_all()
                .build()?
                .block_on(read(endpoint, port, topic, schema))
        }
        _ => Err("usage: android_e2e_hosted_desk provision <dir> | read <endpoint id> <port> <topic> <schema>".into()),
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
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
