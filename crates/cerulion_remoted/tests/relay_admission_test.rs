// SPDX-License-Identifier: AGPL-3.0-only
//! Real relay e-stop delivery while unpaired peers stall TLS handshakes.
//!
//! Genuine QUIC Initials captured from real dial endpoints reach the robot
//! directly over UDP or through a loopback iroh relay under unpaired relay keys.
//! The robot's server flights are withheld, so each admitted handshake stays
//! pending until its deadline; its Retry and refusal replies are forwarded. A
//! paired relay-only desk must still engage the e-stop.
mod common;

use std::net::{Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

use cerud::lease::EstopState;
use cerulion_link::{alpn, dial, Endpoint, EndpointAddr, EndpointId, RelayUrl};
use futures_util::{SinkExt, StreamExt};
use iroh_relay::client::{Client, ClientBuilder, ClientSink, ClientStream};
use iroh_relay::protos::relay::{ClientToRelayMsg, Datagrams, RelayToClientMsg};
use iroh_relay::server::{RelayConfig, Server, ServerConfig};
use tokio::net::UdpSocket;

const CEILING: Duration = Duration::from_secs(5);

/// QUIC v1 long-header packet types.
const INITIAL: u8 = 0;
const HANDSHAKE: u8 = 2;
const RETRY: u8 = 3;

/// A QUIC variable-length integer and its encoded length.
fn varint(bytes: &[u8]) -> Option<(usize, usize)> {
    let first = *bytes.first()?;
    let len = 1usize << (first >> 6);
    let encoded = bytes.get(..len)?;
    let value = encoded[1..]
        .iter()
        .fold(u64::from(first & 0x3f), |value, byte| {
            (value << 8) | u64::from(*byte)
        });
    Some((usize::try_from(value).ok()?, len))
}

/// The `(type, DCID, SCID)` of every QUIC v1 long-header packet coalesced in
/// one datagram, in order. Parsing stops at a short-header packet, a Retry, or a
/// packet it cannot parse.
fn long_headers(mut datagram: &[u8]) -> Vec<(u8, Vec<u8>, Vec<u8>)> {
    let mut packets = Vec::new();
    while datagram.len() > 6 && datagram[0] & 0x80 != 0 && datagram[1..5] == [0, 0, 0, 1] {
        let kind = (datagram[0] >> 4) & 0x3;
        let dcid_len = usize::from(datagram[5]);
        let Some(dcid) = datagram.get(6..6 + dcid_len) else {
            break;
        };
        let Some(&scid_len) = datagram.get(6 + dcid_len) else {
            break;
        };
        let mut at = 7 + dcid_len;
        let Some(scid) = datagram.get(at..at + usize::from(scid_len)) else {
            break;
        };
        at += usize::from(scid_len);
        packets.push((kind, dcid.to_vec(), scid.to_vec()));
        if kind == RETRY {
            break;
        }
        if kind == INITIAL {
            let Some((token_len, n)) = datagram.get(at..).and_then(varint) else {
                break;
            };
            at += n + token_len;
        }
        let Some((length, n)) = datagram.get(at..).and_then(varint) else {
            break;
        };
        let Some(rest) = datagram.get(at + n + length..) else {
            break;
        };
        datagram = rest;
    }
    packets
}

/// Whether `datagram` carries a packet of `kind` addressed to `scid`.
fn carries(datagram: &[u8], kind: u8, scid: &[u8]) -> bool {
    long_headers(datagram)
        .iter()
        .any(|(k, dcid, _)| *k == kind && dcid == scid)
}

async fn relay_endpoint(seed: u8, url: RelayUrl, ip: bool) -> Endpoint {
    let mut builder = iroh::Endpoint::builder(iroh::endpoint::presets::Empty)
        .secret_key(iroh::SecretKey::from([seed; 32]))
        .relay_mode(iroh::RelayMode::custom([url]))
        .alpns(alpn::ALL.iter().map(|alpn| alpn.to_vec()).collect())
        .crypto_provider(iroh_relay::tls::default_provider());
    if !ip {
        builder = builder.clear_ip_transports();
    }
    let endpoint = common::bounded("relay endpoint binds", builder.bind())
        .await
        .expect("relay endpoint binds");
    assert_eq!(endpoint.bound_sockets().is_empty(), !ip, "IP transports");
    common::bounded("relay online", endpoint.online()).await;
    endpoint
}

/// One relay client under an unpaired stranger key.
struct Stranger {
    sender: ClientSink,
    receiver: ClientStream,
}

impl Stranger {
    async fn connect(url: &RelayUrl, seed: u8, robot: &Endpoint) -> Self {
        let client: Client = common::bounded(
            "stranger relay client connects",
            ClientBuilder::new(
                url.clone(),
                iroh::SecretKey::from([seed; 32]),
                robot.dns_resolver().unwrap().clone(),
            )
            .tls_client_config(
                iroh_relay::tls::CaTlsConfig::default()
                    .client_config(iroh_relay::tls::default_provider())
                    .unwrap(),
            )
            .connect(),
        )
        .await
        .expect("an unpaired key authenticates only to the relay");
        let (receiver, sender) = client.split();
        Self { sender, receiver }
    }

    async fn send(&mut self, robot: EndpointId, datagram: &[u8]) {
        common::bounded(
            "relay send",
            self.sender.send(ClientToRelayMsg::Datagrams {
                dst_endpoint_id: robot,
                datagrams: Datagrams::from(datagram),
            }),
        )
        .await
        .unwrap();
    }

    /// The datagrams of the robot's next relay message to this key, answering
    /// relay pings.
    async fn next(&mut self, robot: EndpointId) -> Vec<Vec<u8>> {
        loop {
            match self
                .receiver
                .next()
                .await
                .expect("relay stays open")
                .expect("relay reply")
            {
                RelayToClientMsg::Datagrams {
                    remote_endpoint_id,
                    datagrams,
                } => {
                    assert_eq!(remote_endpoint_id, robot);
                    let segment = datagrams
                        .segment_size
                        .map_or(datagrams.contents.len(), |n| usize::from(n.get()));
                    return datagrams
                        .contents
                        .chunks(segment)
                        .map(<[u8]>::to_vec)
                        .collect();
                }
                RelayToClientMsg::Ping(data) => self
                    .sender
                    .send(ClientToRelayMsg::Pong(data))
                    .await
                    .unwrap(),
                _ => {}
            }
        }
    }

    /// Wait for a robot packet of `kind` addressed to `scid`.
    async fn expect(&mut self, robot: EndpointId, kind: u8, scid: &[u8]) -> Vec<u8> {
        common::bounded("robot relay reply", async {
            loop {
                for datagram in self.next(robot).await {
                    if carries(&datagram, kind, scid) {
                        return datagram;
                    }
                }
            }
        })
        .await
    }
}

struct StalledDial {
    endpoint: Endpoint,
    socket: Arc<UdpSocket>,
    client_addr: SocketAddr,
    initial: Vec<u8>,
    scid: Vec<u8>,
    /// The dial, until a refusal oracle consumes its result.
    task: Option<DialTask>,
}

type DialTask =
    tokio::task::JoinHandle<Result<cerulion_link::Connection, cerulion_link::LinkError>>;

impl StalledDial {
    /// Capture a genuine QUIC Initial from a real endpoint dialing `robot` at a
    /// socket this test owns. TLS stays incomplete while replies are withheld.
    async fn prepare(seed: u8, robot: EndpointId) -> Self {
        let endpoint = common::disabled_endpoint([seed; 32]).await;
        let socket = Arc::new(UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).await.unwrap());
        let addr = cerulion_link::direct_addr(robot, [socket.local_addr().unwrap()]);
        let client = endpoint.clone();
        let task = tokio::spawn(async move { dial(&client, addr, alpn::OPS).await });
        let (initial, client_addr) = Self::recv_initial(&socket, |_| true).await;
        let scid = long_headers(&initial)[0].2.clone();
        Self {
            endpoint,
            socket,
            client_addr,
            initial,
            scid,
            task: Some(task),
        }
    }

    async fn recv_initial(
        socket: &UdpSocket,
        accept: impl Fn(&[u8]) -> bool,
    ) -> (Vec<u8>, SocketAddr) {
        common::bounded("genuine QUIC Initial", async {
            let mut buf = vec![0; 65536];
            loop {
                let (len, addr) = socket.recv_from(&mut buf).await.unwrap();
                let datagram = &buf[..len];
                if long_headers(datagram)
                    .first()
                    .is_some_and(|p| p.0 == INITIAL)
                    && accept(datagram)
                {
                    return (datagram.to_vec(), addr);
                }
            }
        })
        .await
    }

    /// Hand the robot's Retry to the dialer and return its retried Initial,
    /// which carries the address-validation token.
    async fn answer_retry(&self, retry: &[u8]) -> Vec<u8> {
        let first_dcid = long_headers(&self.initial)[0].1.clone();
        self.socket.send_to(retry, self.client_addr).await.unwrap();
        Self::recv_initial(&self.socket, |datagram| {
            long_headers(datagram)[0].1 != first_dcid
        })
        .await
        .0
    }

    async fn close(self) {
        if let Some(task) = self.task {
            task.abort();
            let _ = task.await;
        }
        common::bounded("stalled dialer closes", self.endpoint.close()).await;
    }
}

/// Forward the robot's relay replies for `peer` to its dialer until the dial
/// ends, and require that the robot refused it.
async fn refused_by_relay(stranger: &mut Stranger, robot: EndpointId, peer: &mut StalledDial) {
    stranger.send(robot, &peer.initial).await;
    let mut task = peer.task.take().expect("the dial is still running");
    let refusal = tokio::time::timeout(CEILING, async {
        loop {
            tokio::select! {
                result = &mut task => break result.unwrap().expect_err("the robot refuses the dial"),
                datagrams = stranger.next(robot) => {
                    for datagram in datagrams.iter().filter(|d| carries(d, INITIAL, &peer.scid)) {
                        peer.socket.send_to(datagram, peer.client_addr).await.unwrap();
                    }
                }
            }
        }
    })
    .await
    .expect("the robot answers the dial with a refusal");
    assert_refused(&refusal);
}

/// Forward the robot's direct replies for `peer` to its dialer until the dial
/// ends, and require that the robot refused it.
async fn refused_by_direct(robot: SocketAddr, peer: &mut StalledDial) {
    peer.socket.send_to(&peer.initial, robot).await.unwrap();
    let mut task = peer.task.take().expect("the dial is still running");
    let refusal = tokio::time::timeout(CEILING, async {
        let mut buf = vec![0; 65536];
        loop {
            tokio::select! {
                result = &mut task => break result.unwrap().expect_err("the robot refuses the dial"),
                received = peer.socket.recv_from(&mut buf) => {
                    let (len, from) = received.unwrap();
                    if from == robot {
                        peer.socket.send_to(&buf[..len], peer.client_addr).await.unwrap();
                    }
                }
            }
        }
    })
    .await
    .expect("the robot answers the dial with a refusal");
    assert_refused(&refusal);
}

fn assert_refused(error: &cerulion_link::LinkError) {
    let text = format!("{error:?}");
    assert!(
        text.contains("refused to accept a new connection"),
        "{text}"
    );
}

/// Engage the e-stop from the paired desk over the relay and check the robot's
/// receipt and lease.
async fn engage_estop(desk: &Endpoint, robot: &common::Robot, robot_id: EndpointId, url: RelayUrl) {
    let addr = EndpointAddr::new(robot_id).with_relay_url(url);
    let result = tokio::time::timeout(
        CEILING,
        common::run_client(desk, addr, |client| {
            client.call("engage-estop", serde_json::json!({}))
        }),
    )
    .await
    .expect("paired relay e-stop delivered within five seconds")
    .expect("paired e-stop succeeds");
    assert_eq!(result["engaged"], true);
    let receipts = cerud::receipt::read_all(&robot.receipt_path).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].verb, "engage-estop");
    assert_eq!(receipts[0].outcome, cerud::receipt::ReceiptOutcome::Ok);
    cerud::receipt::verify_chain(&receipts).unwrap();
    assert_eq!(
        robot.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: hex::encode(desk.id().as_bytes()),
        }
    );
}

async fn loopback_relay() -> (Server, RelayUrl) {
    let mut config = ServerConfig::default();
    config.relay = Some(RelayConfig::new(SocketAddr::from((Ipv4Addr::LOCALHOST, 0))));
    let relay = common::bounded("loopback relay starts", Server::spawn(config))
        .await
        .expect("real loopback relay starts");
    let relay_addr = relay.http_addr().expect("HTTP relay listener");
    assert!(relay_addr.ip().is_loopback());
    let url = format!("http://{relay_addr}").parse().unwrap();
    (relay, url)
}

fn assert_unpaired(robot: &common::Robot, seed: u8) {
    assert!(matches!(
        robot
            .shared
            .snapshot_for_key(iroh::SecretKey::from([seed; 32]).public().as_bytes())
            .1,
        cerulion_remoted::trust::KeyAccess::Unpaired
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_unpaired_relay_staller_cannot_refuse_a_paired_estop() {
    let (relay, url) = loopback_relay().await;
    let robot_endpoint = relay_endpoint(40, url.clone(), false).await;
    let robot_id = robot_endpoint.id();
    let desk = relay_endpoint(70, url.clone(), false).await;
    let dir = tempfile::tempdir().unwrap();
    let robot = common::build_robot(
        dir.path(),
        *robot_id.as_bytes(),
        true,
        &[(*desk.id().as_bytes(), common::OWNER)],
    );
    // Prepare every dial first, so no handshake deadline runs while later
    // peers are prepared.
    let mut stalled = Vec::new();
    for seed in 100..110 {
        stalled.push(StalledDial::prepare(seed, robot_id).await);
    }
    assert_unpaired(&robot, 90);
    assert_unpaired(&robot, 91);
    let mut first = Stranger::connect(&url, 90, &robot_endpoint).await;
    let mut second = Stranger::connect(&url, 91, &robot_endpoint).await;

    common::with_serving(&robot, &robot_endpoint, async {
        // Eight stalled handshakes under one stranger key. The robot's first
        // server flight for each proves it was admitted.
        for peer in &stalled[..8] {
            first.send(robot_id, &peer.initial).await;
            first.expect(robot_id, HANDSHAKE, &peer.scid).await;
        }
        // A different stranger key on the same relay is refused: unpaired keys
        // share the URL bucket the first key filled.
        refused_by_relay(&mut second, robot_id, &mut stalled[8]).await;
        engage_estop(&desk, &robot, robot_id, url.clone()).await;
        // The bucket is still full after the desk got in, so the desk was not
        // admitted through a slot an expired stall released.
        refused_by_relay(&mut second, robot_id, &mut stalled[9]).await;
    })
    .await;
    futures_util::future::join_all(stalled.into_iter().map(StalledDial::close)).await;
    common::bounded("desk closes", desk.close()).await;
    common::bounded("robot closes", robot_endpoint.close()).await;
    common::bounded("loopback relay stops", relay.shutdown())
        .await
        .expect("loopback relay stops");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unpaired_peers_filling_the_shared_slots_cannot_take_the_paired_reserve() {
    let (relay, url) = loopback_relay().await;
    let robot_endpoint = relay_endpoint(41, url.clone(), true).await;
    let robot_id = robot_endpoint.id();
    let robot_udp = robot_endpoint
        .bound_sockets()
        .into_iter()
        .find(SocketAddr::is_ipv4)
        .map(|addr| SocketAddr::from((Ipv4Addr::LOCALHOST, addr.port())))
        .expect("an IPv4 transport");
    let desk = relay_endpoint(71, url.clone(), false).await;
    let dir = tempfile::tempdir().unwrap();
    let robot = common::build_robot(
        dir.path(),
        *robot_id.as_bytes(),
        true,
        &[(*desk.id().as_bytes(), common::OWNER)],
    );
    let mut direct = Vec::new();
    for seed in 100..118 {
        direct.push(StalledDial::prepare(seed, robot_id).await);
    }
    let mut relayed = Vec::new();
    for seed in 120..129 {
        relayed.push(StalledDial::prepare(seed, robot_id).await);
    }
    assert_unpaired(&robot, 92);
    let mut stranger = Stranger::connect(&url, 92, &robot_endpoint).await;

    common::with_serving(&robot, &robot_endpoint, async {
        // Sixteen unvalidated direct dials, admitted below half capacity. The
        // server flight each receives proves its admission; it is withheld from
        // the dialer, so each handshake stays pending.
        let mut buf = vec![0; 65536];
        for peer in &direct[..16] {
            peer.socket.send_to(&peer.initial, robot_udp).await.unwrap();
            common::bounded("direct admission", async {
                loop {
                    let (len, from) = peer.socket.recv_from(&mut buf).await.unwrap();
                    if from == robot_udp && carries(&buf[..len], HANDSHAKE, &peer.scid) {
                        break;
                    }
                }
            })
            .await;
        }
        // Eight stranger relay dials at 16 pending: each must answer a Retry,
        // then is admitted into the shared slots, which are then full at 24.
        for peer in &relayed[..8] {
            stranger.send(robot_id, &peer.initial).await;
            let retry = stranger.expect(robot_id, RETRY, &peer.scid).await;
            let validated = peer.answer_retry(&retry).await;
            stranger.send(robot_id, &validated).await;
            stranger.expect(robot_id, HANDSHAKE, &peer.scid).await;
        }
        // A seventeenth direct dial is refused at the shared cap.
        refused_by_direct(robot_udp, &mut direct[16]).await;
        // The paired desk answers its own Retry over the relay and is admitted
        // into the reserve.
        engage_estop(&desk, &robot, robot_id, url.clone()).await;
        // An unvalidated direct dial has no per-source bucket: below 24 pending
        // it would be retried or admitted, so this refusal proves the desk got
        // in while the 24 shared slots were full.
        refused_by_direct(robot_udp, &mut direct[17]).await;
        // Unpaired relay arrivals are still refused at the shared cap.
        refused_by_relay(&mut stranger, robot_id, &mut relayed[8]).await;
    })
    .await;
    futures_util::future::join_all(direct.into_iter().chain(relayed).map(StalledDial::close)).await;
    common::bounded("desk closes", desk.close()).await;
    common::bounded("robot closes", robot_endpoint.close()).await;
    common::bounded("loopback relay stops", relay.shutdown())
        .await
        .expect("loopback relay stops");
}
