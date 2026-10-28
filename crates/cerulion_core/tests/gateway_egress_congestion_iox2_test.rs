// SPDX-License-Identifier: AGPL-3.0-only
//! Gateway egress under a congested network link.
//!
//! A producer manager `P` and a gateway manager `G` share one SHM root (two
//! processes on one machine). `G` listens on 127.0.0.1; a receiver session
//! reaches it through an in-process TCP proxy that forwards `G`'s bytes at a
//! set rate, or not at all while paused. A slow or stalled proxy fills zenoh's
//! transmit queue on `G`, which is the condition under which zenoh's default
//! `Drop` congestion control discards a put at the sender and still returns
//! `Ok`.
//!
//! What the tests pin:
//!
//! - a reliable topic delivers every frame, in order, across a stall, with
//!   the gateway's drive pass never waiting on the link;
//! - a full reliable send queue refuses the new frame, counts it, and warns
//!   naming the topic;
//! - a best-effort topic on a stalled link loses frames without the drive pass
//!   waiting, and its counters account for every frame;
//! - an undeclared `send_queue` is as deep as the gateway's tap on the topic;
//! - when demand ends, or LAN access closes, queued frames are discarded,
//!   counted, and never sent;
//! - a stalled link does not hold up a topic on another link.
//!
//! Every stall stays shorter than zenoh's `wait_before_close` (5 s), after
//! which zenoh would close the stalled link; each test measures the stall
//! and fails by name when it outlived that bound.
//!
//! The receiver is a plain zenoh subscriber that records each frame's wire
//! sequence, so receive-side SHM queues cannot lose frames and blur the
//! sender-side count. Per-test SHM roots, probed ports and scouting-off
//! sessions make the binary parallel-safe.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::num::NonZeroUsize;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::testing::count_at_exclusively;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, StepTag, WireHeader};
use cerulion_core::{EgressQos, EgressReliability, EgressStats, EgressTopicStats};
use tracing_test::traced_test;
use zenoh::Wait;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

fn probe_ephemeral_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = listener.local_addr().expect("probe local_addr").port();
    drop(listener);
    port
}

/// The schema hash stamped on every test frame. The gateway forwards frames
/// verbatim and never checks it.
const TEST_SCHEMA_HASH: u64 = 0x5EED_CAFE_F00D_0001;

/// A frame published while the receiver's route is being established. Its
/// sequence is outside every test's range, and the receiver skips it.
const WARMUP_SEQUENCE: u32 = u32::MAX;

/// A wire frame of `len` bytes (header included) carrying `sequence`.
fn frame(sequence: u32, len: usize) -> Vec<u8> {
    assert!(len >= WireHeader::SIZE);
    let header = WireHeader {
        schema_hash: TEST_SCHEMA_HASH,
        total_size: len as u32,
        offset_table_offset: len as u32,
        offset_table_count: 0,
        sequence,
        timestamp_ns: u64::from(sequence) + 1,
        step_tag: StepTag::UNTAGGED,
    };
    let mut buf = vec![(sequence & 0xFF) as u8; len];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf
}

// ---------------------------------------------------------------------------
// Throttled TCP proxy
// ---------------------------------------------------------------------------

/// A TCP proxy between a receiver and `G`'s listen port. Bytes from the
/// receiver to `G` pass straight through; bytes from `G` to the receiver pass
/// at `rate` bytes per second, and not at all while `paused`.
struct ThrottledProxy {
    port: u16,
    paused: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl ThrottledProxy {
    fn start(upstream_port: u16, rate_bytes_per_s: u64) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind proxy");
        listener.set_nonblocking(true).expect("nonblocking proxy");
        let port = listener.local_addr().unwrap().port();
        let paused = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (accept_paused, accept_stop) = (Arc::clone(&paused), Arc::clone(&stop));
        let accept = std::thread::spawn(move || {
            let mut pumps = Vec::new();
            while !accept_stop.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((client, _)) => {
                        let Ok(upstream) = TcpStream::connect(("127.0.0.1", upstream_port)) else {
                            continue;
                        };
                        client.set_nonblocking(false).unwrap();
                        for s in [&client, &upstream] {
                            s.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
                            s.set_nodelay(true).unwrap();
                        }
                        let (c2, u2) = (client.try_clone().unwrap(), upstream.try_clone().unwrap());
                        let s1 = Arc::clone(&accept_stop);
                        pumps.push(std::thread::spawn(move || {
                            pump(client, upstream, None, None, &s1)
                        }));
                        let (s2, p2) = (Arc::clone(&accept_stop), Arc::clone(&accept_paused));
                        pumps.push(std::thread::spawn(move || {
                            pump(u2, c2, Some(rate_bytes_per_s), Some(p2), &s2)
                        }));
                    }
                    Err(e) if e.kind() == ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    Err(_) => break,
                }
            }
            for p in pumps {
                let _ = p.join();
            }
        });
        Self {
            port,
            paused,
            stop,
            threads: vec![accept],
        }
    }

    fn locator(&self) -> String {
        format!("tcp/127.0.0.1:{}", self.port)
    }
}

impl Drop for ThrottledProxy {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// Copy `from` → `to` until either side closes or `stop` is set, at most
/// `rate` bytes per second when set, holding every byte while `paused`.
fn pump(
    mut from: TcpStream,
    mut to: TcpStream,
    rate: Option<u64>,
    paused: Option<Arc<AtomicBool>>,
    stop: &AtomicBool,
) {
    let mut buf = vec![0u8; 16 * 1024];
    while !stop.load(Ordering::Relaxed) {
        if paused.as_ref().is_some_and(|p| p.load(Ordering::Relaxed)) {
            std::thread::sleep(Duration::from_millis(5));
            continue;
        }
        let n = match from.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => continue,
            Err(_) => break,
        };
        if to.write_all(&buf[..n]).is_err() {
            break;
        }
        if let Some(rate) = rate {
            std::thread::sleep(Duration::from_secs_f64(n as f64 / rate as f64));
        }
    }
    let _ = to.shutdown(std::net::Shutdown::Both);
}

// ---------------------------------------------------------------------------
// P + G + receivers
// ---------------------------------------------------------------------------

/// One egress topic's declared policy in a test.
struct TopicSpec {
    topic: String,
    reliability: EgressReliability,
    send_queue: Option<usize>,
    frame_len: usize,
}

struct Rig {
    producer: Arc<TransportManager>,
    gateway: GatewayRuntime,
    gateway_port: u16,
}

/// Build P (local only) and G (listening on a probed port) on one SHM root.
/// Each topic is announced, put at its declared reliability, sized to its
/// declared send queue, and its egress flag is switched on directly (a
/// remote demand's effect).
fn rig(tag: &str, topics: &[&TopicSpec]) -> Rig {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let producer = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("egc_p_{tag}_{id}"),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init producer P");
    let egress_qos: Vec<EgressQos> = topics
        .iter()
        .map(|t| EgressQos {
            topic: t.topic.clone(),
            reliability: t.reliability,
            send_queue: t.send_queue.and_then(NonZeroUsize::new),
        })
        .collect();
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: topics.iter().map(|t| t.topic.clone()).collect(),
        ingress: vec![],
    };
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("egc_g_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some("egctest".to_string()),
                    egress_qos: egress_qos.clone(),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init gateway G");
        match GatewayRuntime::new(g, plan.clone()) {
            Ok(gateway) => {
                for t in topics {
                    gateway
                        .manager()
                        .bridge_manager()
                        .enable_bridge(&t.topic)
                        .expect("enable egress flag");
                }
                return Rig {
                    producer,
                    gateway,
                    gateway_port: port,
                };
            }
            Err(e) => eprintln!("attempt {attempt}: G session failed (port {port}): {e}"),
        }
    }
    panic!("could not open G's listening session in 3 attempts");
}

/// A receiver session that records the wire sequence of every frame of the
/// topics it subscribes to, in arrival order, and notes whether a warmup
/// frame has arrived.
struct Receiver {
    // Field order: the subscribers drop before the session they belong to.
    _subscribers: Vec<zenoh::pubsub::Subscriber<()>>,
    _network: NetworkManager,
    received: Arc<Mutex<Vec<(String, u32)>>>,
    warmed: Arc<AtomicBool>,
}

impl Receiver {
    fn connect(locator: String, topics: &[&str]) -> Self {
        let network = NetworkManager::new(NetworkConfig {
            connect_endpoints: vec![locator],
            ..NetworkConfig::default()
        });
        let received = Arc::new(Mutex::new(Vec::new()));
        let warmed = Arc::new(AtomicBool::new(false));
        let session = network.session().expect("receiver session");
        let subscribers = topics
            .iter()
            .map(|topic| {
                let sink = Arc::clone(&received);
                let warm = Arc::clone(&warmed);
                let name = topic.to_string();
                session
                    .declare_subscriber(NetworkManager::data_key(topic))
                    .callback(move |sample| {
                        let bytes = sample.payload().to_bytes();
                        if let Some(h) = WireHeader::read_from_buf(&bytes) {
                            if h.sequence == WARMUP_SEQUENCE {
                                warm.store(true, Ordering::Relaxed);
                            } else {
                                sink.lock().unwrap().push((name.clone(), h.sequence));
                            }
                        }
                    })
                    .wait()
                    .expect("declare receiver subscriber")
            })
            .collect();
        Self {
            _subscribers: subscribers,
            _network: network,
            received,
            warmed,
        }
    }

    fn sequences(&self, topic: &str) -> Vec<u32> {
        self.received
            .lock()
            .unwrap()
            .iter()
            .filter(|(t, _)| t == topic)
            .map(|(_, s)| *s)
            .collect()
    }
}

/// Put warmup frames straight onto `G`'s session until `receiver` has heard
/// one on `topic`: zenoh sends a put only to sessions whose subscription it
/// has heard, so before this a put could reach nobody. Warmup frames never
/// reach a test's count.
fn wait_for_route(gateway: &GatewayRuntime, topic: &str, receiver: &Receiver) {
    let network = gateway.manager().network().expect("G has a network");
    let deadline = Instant::now() + Duration::from_secs(15);
    receiver.warmed.store(false, Ordering::Relaxed);
    while !receiver.warmed.load(Ordering::Relaxed) {
        assert!(
            Instant::now() < deadline,
            "receiver route for {topic} not established in 15 s"
        );
        network
            .publish_to_network(topic, frame(WARMUP_SEQUENCE, WireHeader::SIZE + 8))
            .expect("warmup put");
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Run `G`'s drive loop on its own thread, as `cerulion-netd` and `graph
/// run` do, until the returned guard drops.
struct DriveThread {
    exit: Arc<AtomicBool>,
    handle: Option<JoinHandle<GatewayRuntime>>,
}

impl DriveThread {
    fn start(mut gateway: GatewayRuntime) -> Self {
        let exit = Arc::new(AtomicBool::new(false));
        let thread_exit = Arc::clone(&exit);
        let handle = std::thread::spawn(move || {
            gateway.run(&thread_exit).expect("gateway drive loop");
            gateway
        });
        Self {
            exit,
            handle: Some(handle),
        }
    }

    fn stop(mut self) -> GatewayRuntime {
        self.exit.store(true, Ordering::Relaxed);
        self.handle.take().unwrap().join().expect("drive thread")
    }
}

impl Drop for DriveThread {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn stats_of(stats: &EgressStats, topic: &str) -> EgressTopicStats {
    stats
        .topic(topic)
        .unwrap_or_else(|| panic!("{topic} has no egress send queue yet"))
}

/// Poll `cond` every 20 ms until it holds or `secs` pass; returns whether it
/// held.
fn wait_until(secs: u64, mut cond: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        if cond() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    cond()
}

/// Wait until the gateway has a tap on `topic` (frames published before the
/// tap attaches are not seen by it).
fn wait_for_tap(stats: &EgressStats, topic: &str) {
    assert!(
        wait_until(10, || stats.topic(topic).is_some()),
        "{topic}'s egress tap did not attach"
    );
}

/// Publish `count` frames of `len` bytes on `publisher`, `gap` apart.
fn publish_burst(
    publisher: &mut cerulion_core::CerulionPublisher,
    first_sequence: u32,
    count: u32,
    len: usize,
    gap: Duration,
) {
    for seq in first_sequence..first_sequence + count {
        publisher
            .publish_raw(&frame(seq, len))
            .expect("publish_raw");
        std::thread::sleep(gap);
    }
}

/// The longest one gateway drive pass may take while a link is stalled. A
/// drive pass that waited on the stalled link would wait until it moved,
/// which is the rest of the stall.
const DRIVE_PASS_CEILING: Duration = Duration::from_secs(2);

/// A stall must end before zenoh's `wait_before_close` (5 s) closes the
/// stalled link.
const STALL_CEILING: Duration = Duration::from_secs(4);

/// Frames of 256 KiB offered while a link is stalled: 24 MiB, more than the
/// default loopback socket buffers and zenoh's transmit queue hold, so the
/// stall reaches the gateway's send queue. A host whose buffers hold more
/// fails the tests' `queued >= 1` precondition by name.
const STALL_FRAMES: u32 = 96;
const STALL_LEN: usize = 256 * 1024;

/// Publish `count` frames on the test thread with one gateway drive pass
/// after each (so the gateway's log lines are captured on this thread), and
/// return the slowest drive pass.
fn publish_and_drive(
    gateway: &mut GatewayRuntime,
    publisher: &mut cerulion_core::CerulionPublisher,
    first_sequence: u32,
    count: u32,
    len: usize,
) -> Duration {
    let mut slowest_pass = Duration::ZERO;
    for seq in first_sequence..first_sequence + count {
        publisher
            .publish_raw(&frame(seq, len))
            .expect("publish_raw");
        let started = Instant::now();
        gateway.drive_once().expect("drive pass");
        slowest_pass = slowest_pass.max(started.elapsed());
    }
    slowest_pass
}

/// Drive the gateway on the test thread for `for_how_long`.
fn drive_for(gateway: &mut GatewayRuntime, for_how_long: Duration) {
    let deadline = Instant::now() + for_how_long;
    while Instant::now() < deadline {
        gateway.drive_once().expect("drive pass");
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Every frame the gateway took off the topic's tap is in exactly one
/// counter.
fn accounted(s: &EgressTopicStats) -> u64 {
    s.queued + s.in_flight + s.forwarded + s.dropped + s.refused + s.failed + s.discarded
}

/// Received sequences rise strictly: no duplicate, no reordering.
fn strictly_rising(seqs: &[u32]) -> bool {
    seqs.windows(2).all(|w| w[0] < w[1])
}

/// One stall-test topic: P, G (driven on the test thread), a publisher, a
/// receiver behind a proxy, the route warmed and the egress tap attached.
struct StallRig {
    _producer: Arc<TransportManager>,
    gateway: GatewayRuntime,
    publisher: cerulion_core::CerulionPublisher,
    proxy: ThrottledProxy,
    receiver: Receiver,
    stats: Arc<EgressStats>,
    topic: String,
    paused_at: Option<Instant>,
}

impl StallRig {
    fn new(tag: &str, reliability: EgressReliability, send_queue: Option<usize>) -> Self {
        let spec = TopicSpec {
            topic: format!("/egc/{tag}/{}", unique_id()),
            reliability,
            send_queue,
            frame_len: STALL_LEN,
        };
        let Rig {
            producer,
            mut gateway,
            gateway_port,
        } = rig(tag, &[&spec]);
        let publisher = producer
            .create_publisher_simple(&spec.topic, MaxSliceLen::const_new(STALL_LEN as u32))
            .expect("P publisher");
        let proxy = ThrottledProxy::start(gateway_port, 64 * 1024 * 1024);
        let receiver = Receiver::connect(proxy.locator(), &[&spec.topic]);
        wait_for_route(&gateway, &spec.topic, &receiver);
        let stats = gateway.egress_stats();
        assert!(
            wait_until(10, || {
                gateway.drive_once().expect("attach pass");
                stats.topic(&spec.topic).is_some()
            }),
            "tap did not attach"
        );
        Self {
            _producer: producer,
            gateway,
            publisher,
            proxy,
            receiver,
            stats,
            topic: spec.topic,
            paused_at: None,
        }
    }

    fn pause(&mut self) {
        self.proxy.paused.store(true, Ordering::Relaxed);
        self.paused_at = Some(Instant::now());
    }

    fn resume(&mut self) {
        self.assert_stall_bound();
        self.paused_at = None;
        self.proxy.paused.store(false, Ordering::Relaxed);
    }

    /// Publish `count` frames from `first`, one drive pass after each.
    fn offer(&mut self, first: u32, count: u32) -> Duration {
        publish_and_drive(
            &mut self.gateway,
            &mut self.publisher,
            first,
            count,
            STALL_LEN,
        )
    }

    /// Publish [`STALL_FRAMES`] frames from 0, one drive pass after each, then
    /// assert the stall is still inside [`STALL_CEILING`]. Returns how many
    /// frames were offered and the slowest drive pass.
    fn offer_all(&mut self) -> (u32, Duration) {
        let slowest = self.offer(0, STALL_FRAMES);
        self.assert_stall_bound();
        (STALL_FRAMES, slowest)
    }

    /// Publish frames from 0, one drive pass after each, until the link is
    /// really stalled: at least `want_queued` frames wait in the send queue,
    /// one is inside the put, and `forwarded` does not move for 200 ms. A put
    /// that is merely slow (the sender copying a large frame) is not a stall,
    /// so this keeps offering until the socket buffers on both ends are full.
    /// Then asserts the stall is inside [`STALL_CEILING`]. Returns how many
    /// frames were offered.
    fn offer_until_stalled(&mut self, want_queued: u64) -> u32 {
        let mut offered = 0;
        while offered < STALL_FRAMES {
            self.offer(offered, 1);
            offered += 1;
            let s = self.stats();
            if s.queued >= want_queued && s.in_flight == 1 {
                let before = s.forwarded;
                drive_for(&mut self.gateway, Duration::from_millis(200));
                let t = self.stats();
                if t.forwarded == before && t.queued >= want_queued && t.in_flight == 1 {
                    break;
                }
            }
        }
        let s = self.stats();
        assert!(
            s.queued >= want_queued && s.in_flight == 1,
            "PRECONDITION: the link never stalled within {STALL_FRAMES} frames: {s:?}"
        );
        self.assert_stall_bound();
        offered
    }

    /// PRECONDITION: the stall has not outlived the bound under zenoh's
    /// `wait_before_close`, after which zenoh closes the stalled link and
    /// the counters no longer describe a stall.
    fn assert_stall_bound(&self) {
        let stalled_for = self.paused_at.expect("paused").elapsed();
        assert!(
            stalled_for < STALL_CEILING,
            "PRECONDITION: the stall lasted {stalled_for:?}, past the bound under zenoh's \
             wait_before_close"
        );
    }

    fn stats(&self) -> EgressTopicStats {
        stats_of(&self.stats, &self.topic)
    }

    fn got(&self) -> Vec<u32> {
        self.receiver.sequences(&self.topic)
    }

    /// Wait until the lane is idle and every forwarded frame has arrived.
    fn settle(&self) {
        assert!(
            wait_until(30, || {
                let s = self.stats();
                s.queued == 0 && s.in_flight == 0 && self.got().len() as u64 == s.forwarded
            }),
            "the lane did not settle: {:?}, {} received",
            self.stats(),
            self.got().len()
        );
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// A reliable topic whose link stalls loses no frame: its frames wait in the
/// send queue (declared deep enough for the whole burst) and in zenoh's
/// transmit queue, the drive pass never waits on the link, and when the link
/// moves every frame arrives exactly once, in order.
#[test]
fn a_reliable_topic_loses_no_frame_when_its_link_stalls() {
    let mut r = StallRig::new(
        "rel",
        EgressReliability::Reliable,
        Some(STALL_FRAMES as usize + 8),
    );
    r.pause();
    let offered = r.offer_until_stalled(8);
    let slowest_pass = r.offer(offered, 8);
    let offered = offered + 8;
    let stalled = r.stats();
    assert!(
        stalled.queued >= 1,
        "PRECONDITION: the stall must reach the send queue: {stalled:?}"
    );
    assert_eq!(stalled.refused, 0, "{stalled:?}");
    assert_eq!(accounted(&stalled), u64::from(offered), "{stalled:?}");
    assert!(
        slowest_pass < DRIVE_PASS_CEILING,
        "a drive pass waited {slowest_pass:?}"
    );
    r.resume();
    r.settle();
    assert_eq!(
        r.got(),
        (0..offered).collect::<Vec<_>>(),
        "every reliable frame arrives exactly once, in order"
    );
    let s = r.stats();
    assert_eq!(
        (s.forwarded, s.refused, s.dropped, s.failed, s.discarded),
        (u64::from(offered), 0, 0, 0, 0),
        "{s:?}"
    );
}

/// A reliable topic whose link stalls fills its send queue, and the next
/// frame is refused: counted in `refused` and logged at WARN naming the
/// topic, never silently lost. While the link is stalled exactly one frame
/// is inside the blocked put, and every other frame is in a counter. When
/// the link moves every accepted frame arrives in order, and the first frame
/// queued after the drain ends the refusal run with a recovery line.
#[traced_test]
#[test]
fn a_full_reliable_send_queue_refuses_loudly_and_counts_every_frame() {
    let mut r = StallRig::new("refuse", EgressReliability::Reliable, Some(2));
    r.pause();
    let offered = r.offer_until_stalled(2);
    // The link is stalled with the 2-frame queue full: the next frames are
    // refused.
    let slowest_pass = r.offer(offered, 4);
    let offered = offered + 4;
    drive_for(&mut r.gateway, Duration::from_millis(300));
    r.assert_stall_bound();
    let stalled = r.stats();
    assert!(
        stalled.refused >= 1,
        "a stalled link fills a 2-frame queue: {stalled:?}"
    );
    assert_eq!(
        (stalled.queued, stalled.in_flight),
        (2, 1),
        "the queue is full and one frame waits in the put: {stalled:?}"
    );
    assert_eq!(accounted(&stalled), u64::from(offered), "{stalled:?}");
    assert!(
        slowest_pass < DRIVE_PASS_CEILING,
        "a drive pass waited {slowest_pass:?}"
    );
    r.resume();
    r.settle();
    let s = r.stats();
    assert_eq!(s.failed + s.dropped + s.discarded, 0, "{s:?}");
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    let got = r.got();
    assert_eq!(
        got.len() as u64,
        s.forwarded,
        "every forwarded frame arrived"
    );
    assert!(
        strictly_rising(&got),
        "reliable frames arrive in order: {got:?}"
    );

    // Room again: the next frame is queued and the refusal run ends.
    r.offer(offered, 1);
    assert!(
        wait_until(10, || r.got().last() == Some(&offered)),
        "the frame after the stall did not arrive"
    );
    let topic = r.topic.clone();
    logs_assert(move |lines: &[&str]| {
        let loud = count_at_exclusively(
            lines,
            "WARN",
            &[
                topic.as_str(),
                "send queue is full: a reliable frame was refused",
            ],
        )?;
        let recovered = count_at_exclusively(
            lines,
            "INFO",
            &[topic.as_str(), "send queue has room again"],
        )?;
        // Each run of refusals opens with one loud line; a run that
        // suppressed repeats closes with one recovery line. The run that
        // ends after the drain held at least four refusals.
        if loud >= 1 && recovered >= 1 && recovered <= loud {
            Ok(())
        } else {
            Err(format!(
                "want at least one refusal and one recovery line, and no more \
                 recoveries than refusals: got {loud} and {recovered}"
            ))
        }
    });
}

/// A best-effort topic on a stalled link loses frames, as best effort may,
/// without the drive pass waiting; its counters account for every frame the
/// gateway took, and what arrives is in order.
#[test]
fn a_best_effort_topic_drops_on_a_stalled_link_without_waiting() {
    let mut r = StallRig::new("be", EgressReliability::BestEffort, Some(2));
    r.pause();
    let (offered, slowest_pass) = r.offer_all();
    drive_for(&mut r.gateway, Duration::from_millis(300));
    assert!(
        slowest_pass < DRIVE_PASS_CEILING,
        "a drive pass waited {slowest_pass:?}"
    );
    r.resume();
    // zenoh may discard a best-effort frame after its put returned `Ok`, so
    // only the lane can settle here, not the receive count.
    assert!(
        wait_until(30, || {
            let s = r.stats();
            s.queued == 0 && s.in_flight == 0
        }),
        "the lane did not drain: {:?}",
        r.stats()
    );
    drive_for(&mut r.gateway, Duration::from_millis(500));
    let s = r.stats();
    assert_eq!(s.reliability, EgressReliability::BestEffort);
    assert_eq!((s.refused, s.failed, s.discarded), (0, 0, 0), "{s:?}");
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    let got = r.got();
    assert!(
        (got.len() as u64) < u64::from(offered),
        "a stalled best-effort link must lose frames: got all {}",
        got.len()
    );
    assert!(
        got.len() as u64 <= s.forwarded,
        "{} arrived, {s:?}",
        got.len()
    );
    assert!(strictly_rising(&got), "{got:?}");
}

/// With no declared `send_queue`, a topic's send queue is as deep as the
/// gateway's tap on it: the topic service's default subscriber buffer, 16.
#[test]
fn an_undeclared_send_queue_is_as_deep_as_the_egress_tap() {
    let spec = TopicSpec {
        topic: format!("/egc/default/{}", unique_id()),
        reliability: EgressReliability::Reliable,
        send_queue: None,
        frame_len: WireHeader::SIZE + 8,
    };
    let Rig {
        producer,
        mut gateway,
        ..
    } = rig("default", &[&spec]);
    let _publisher = producer
        .create_publisher_simple(&spec.topic, MaxSliceLen::const_new(spec.frame_len as u32))
        .expect("P publisher");
    let stats = gateway.egress_stats();
    assert!(
        wait_until(10, || {
            gateway.drive_once().expect("attach pass");
            stats.topic(&spec.topic).is_some()
        }),
        "tap did not attach"
    );
    assert_eq!(stats_of(&stats, &spec.topic).send_queue, 16);
}

/// When demand for a topic ends, the frames in its send queue are
/// discarded, counted, and never sent; the frame already inside the put
/// finishes. A later demand starts from fresh frames.
#[test]
fn queued_frames_are_discarded_when_demand_ends() {
    let mut r = StallRig::new("demand", EgressReliability::Reliable, Some(4));
    r.pause();
    let offered = r.offer_until_stalled(4);
    r.offer(offered, 1);
    let offered = offered + 1;
    let before = r.stats();
    assert_eq!(
        (before.queued, before.in_flight),
        (4, 1),
        "PRECONDITION: a full queue behind a blocked put: {before:?}"
    );
    r.gateway
        .manager()
        .bridge_manager()
        .disable_bridge(&r.topic)
        .expect("disable");
    r.gateway.drive_once().expect("detach pass");
    let ended = r.stats();
    assert_eq!(ended.discarded, before.discarded + 4, "{ended:?}");
    assert_eq!(ended.queued, 0, "{ended:?}");
    r.resume();
    r.settle();
    let s = r.stats();
    assert_eq!(
        s.forwarded,
        before.forwarded + 1,
        "only the in-flight frame is sent: {s:?}"
    );
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    assert_eq!(
        r.got(),
        (0..s.forwarded as u32).collect::<Vec<_>>(),
        "no discarded frame arrives"
    );

    r.gateway
        .manager()
        .bridge_manager()
        .enable_bridge(&r.topic)
        .expect("re-enable");
    assert!(
        wait_until(10, || {
            r.gateway.drive_once().expect("re-attach pass");
            r.gateway.active_tap_topics().contains(&r.topic)
        }),
        "tap did not re-attach"
    );
    let fresh = 10_000;
    r.offer(fresh, 1);
    assert!(
        wait_until(10, || {
            r.gateway.drive_once().expect("drive");
            r.got().last() == Some(&fresh)
        }),
        "a frame after the new demand did not arrive"
    );
}

/// A tap drain error drops the tap and leaves the frames it queued before;
/// demand then ends while no tap is attached, and returns while a put waits
/// on the stalled link. The frames queued before demand ended are discarded,
/// not sent to the new demand.
#[test]
fn frames_queued_before_a_drain_error_are_discarded_when_demand_ends() {
    let mut r = StallRig::new("drainerr", EgressReliability::Reliable, Some(4));
    r.pause();
    let offered = r.offer_until_stalled(4);
    let before = r.stats();
    assert_eq!(
        (before.queued, before.in_flight),
        (4, 1),
        "PRECONDITION: a full queue behind a blocked put: {before:?}"
    );
    assert!(
        r.gateway.fail_next_egress_drain_for_test(&r.topic),
        "PRECONDITION: there was an egress tap to poison"
    );
    r.gateway
        .drive_once()
        .expect("drive survives a tap drain error");
    assert!(
        !r.gateway.active_tap_topics().contains(&r.topic),
        "PRECONDITION: the drain error dropped the tap"
    );
    assert_eq!(
        r.stats().queued,
        4,
        "PRECONDITION: the frames queued before the drain error are still queued"
    );
    let bridges = r.gateway.manager().bridge_manager_arc();
    bridges.disable_bridge(&r.topic).expect("disable");
    r.gateway.drive_once().expect("pass without demand");
    assert!(
        !bridges.is_enabled(&r.topic).expect("enabled?"),
        "PRECONDITION: demand stayed off through the pass"
    );
    let ended = r.stats();
    assert_eq!(
        (ended.queued, ended.discarded),
        (0, before.discarded + 4),
        "a pass without demand empties the queue although no tap is attached: {ended:?}"
    );
    bridges.enable_bridge(&r.topic).expect("re-enable");
    assert!(
        bridges.is_enabled(&r.topic).expect("enabled?"),
        "PRECONDITION: the topic is demanded again while the put waits"
    );
    r.resume();
    drive_for(&mut r.gateway, Duration::from_millis(300));
    r.settle();
    let s = r.stats();
    assert_eq!(
        s.forwarded,
        before.forwarded + 1,
        "only the put under way is sent: {s:?}"
    );
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    // Control: frames published under the new demand arrive.
    assert!(
        wait_until(10, || {
            r.gateway.drive_once().expect("re-attach pass");
            r.gateway.active_tap_topics().contains(&r.topic)
        }),
        "PRECONDITION: the tap re-attached under the new demand"
    );
    const AFTER: u32 = 2;
    r.offer(offered, AFTER);
    r.settle();
    let mut want: Vec<u32> = (0..s.forwarded as u32).collect();
    want.extend(offered..offered + AFTER);
    assert_eq!(
        r.got(),
        want,
        "no frame queued before demand ended arrives; frames after it returned do"
    );
}

/// Closing LAN access while a put waits on a stalled link: the close returns
/// only after that put has finished, and the frames still queued are
/// discarded, counted, and never sent, even though the link moves again.
#[test]
fn a_lan_close_discards_frames_queued_for_a_peer() {
    let mut r = StallRig::new("lanclose", EgressReliability::Reliable, Some(8));
    r.pause();
    let offered = r.offer_until_stalled(8);
    r.offer(offered, 1);
    let offered = offered + 1;
    let before = r.stats();
    assert_eq!(
        (before.queued, before.in_flight),
        (8, 1),
        "PRECONDITION: a full queue behind a blocked put: {before:?}"
    );
    let network = r.gateway.manager().network().expect("G has a network");
    let stats = Arc::clone(&r.stats);
    let topic = r.topic.clone();
    let forwarded_when_closed = std::thread::scope(|scope| {
        // The forwarded count the moment the close returns: the close must
        // not return before the put already under way has finished.
        let closer = scope.spawn(|| {
            network.set_lan_access(cerulion_core::LanAccess::Paired);
            stats_of(&stats, &topic).forwarded
        });
        std::thread::sleep(Duration::from_millis(300));
        let stalled_for = r.paused_at.expect("paused").elapsed();
        assert!(
            stalled_for < STALL_CEILING,
            "PRECONDITION: the stall lasted {stalled_for:?}, past the bound under zenoh's \
             wait_before_close"
        );
        r.proxy.paused.store(false, Ordering::Relaxed);
        closer.join().expect("closer")
    });
    assert!(
        wait_until(10, || {
            let s = r.stats();
            s.queued == 0 && s.in_flight == 0
        }),
        "the queue did not empty after the close: {:?}",
        r.stats()
    );
    drive_for(&mut r.gateway, Duration::from_millis(500));
    std::thread::sleep(Duration::from_millis(500));
    let s = r.stats();
    assert_eq!(
        s.forwarded,
        before.forwarded + 1,
        "only the in-flight frame is sent: {s:?}"
    );
    assert_eq!(
        forwarded_when_closed, s.forwarded,
        "no frame is forwarded after the close returns: {s:?}"
    );
    assert!(s.discarded >= 8, "{s:?}");
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    assert_eq!(
        r.got(),
        (0..s.forwarded as u32).collect::<Vec<_>>(),
        "no frame queued before the close arrives after it"
    );
}

/// A stalled link does not hold up a topic sent over another link: while the
/// slow topic's sender waits on its stalled link and its send queue refuses
/// frames, every frame of the fast topic reaches its receiver. (Topics sent
/// to the same peer share zenoh's transmit queue for that peer, so this
/// isolation is per link.)
#[test]
fn a_stalled_link_does_not_stall_a_topic_on_another_link() {
    const FAST_FRAMES: u32 = 50;
    let slow = TopicSpec {
        topic: format!("/egc/slow/{}", unique_id()),
        reliability: EgressReliability::Reliable,
        send_queue: Some(4),
        frame_len: STALL_LEN,
    };
    let fast = TopicSpec {
        topic: format!("/egc/fast/{}", unique_id()),
        reliability: EgressReliability::Reliable,
        send_queue: Some(4),
        frame_len: WireHeader::SIZE + 1024,
    };
    let Rig {
        producer,
        gateway,
        gateway_port,
    } = rig("slowfast", &[&slow, &fast]);
    let mut slow_pub = producer
        .create_publisher_simple(&slow.topic, MaxSliceLen::const_new(STALL_LEN as u32))
        .expect("slow publisher");
    let mut fast_pub = producer
        .create_publisher_simple(&fast.topic, MaxSliceLen::const_new(fast.frame_len as u32))
        .expect("fast publisher");
    let proxy = ThrottledProxy::start(gateway_port, 64 * 1024 * 1024);
    let slow_rx = Receiver::connect(proxy.locator(), &[&slow.topic]);
    let fast_rx = Receiver::connect(format!("tcp/127.0.0.1:{gateway_port}"), &[&fast.topic]);
    wait_for_route(&gateway, &slow.topic, &slow_rx);
    wait_for_route(&gateway, &fast.topic, &fast_rx);
    let stats = gateway.egress_stats();
    let drive = DriveThread::start(gateway);
    wait_for_tap(&stats, &slow.topic);
    wait_for_tap(&stats, &fast.topic);

    proxy.paused.store(true, Ordering::Relaxed);
    let paused_at = Instant::now();
    // Publish slow frames until its queue refuses one (the drive thread
    // queues them), so the stall stays short on a loaded host.
    let mut seq = 0;
    while seq < STALL_FRAMES && stats_of(&stats, &slow.topic).refused == 0 {
        publish_burst(
            &mut slow_pub,
            seq,
            1,
            slow.frame_len,
            Duration::from_millis(1),
        );
        seq += 1;
    }
    assert!(
        wait_until(3, || stats_of(&stats, &slow.topic).refused >= 1),
        "PRECONDITION: the slow topic's send queue must fill: {:?}",
        stats_of(&stats, &slow.topic)
    );
    publish_burst(
        &mut fast_pub,
        0,
        FAST_FRAMES,
        fast.frame_len,
        Duration::from_millis(1),
    );
    let fast_done = wait_until(3, || {
        fast_rx.sequences(&fast.topic).len() >= FAST_FRAMES as usize
    });
    let slow_during = stats_of(&stats, &slow.topic);
    let stalled_for = paused_at.elapsed();
    proxy.paused.store(false, Ordering::Relaxed);
    let _gateway = drive.stop();

    assert!(
        stalled_for < STALL_CEILING,
        "PRECONDITION: the stall lasted {stalled_for:?}, past the bound under zenoh's \
         wait_before_close"
    );
    assert!(
        fast_done,
        "the fast topic stalled behind the slow one: {} of {FAST_FRAMES} arrived",
        fast_rx.sequences(&fast.topic).len()
    );
    assert_eq!(
        fast_rx.sequences(&fast.topic),
        (0..FAST_FRAMES).collect::<Vec<_>>()
    );
    assert_eq!(
        slow_during.in_flight, 1,
        "the slow topic's sender was still waiting on its link: {slow_during:?}"
    );
}

/// Frames published while the gateway had not yet drained them, then a LAN
/// close and a reopen with the topic demanded again: none of them reaches the
/// peer. Frames published after the reopen do, in order.
#[test]
fn frames_published_before_a_lan_close_never_reach_a_peer_after_a_reopen() {
    const BEFORE: u32 = 3;
    const AFTER: u32 = 3;
    let mut r = StallRig::new("tapclose", EgressReliability::Reliable, Some(8));
    // The tap holds these; no drive pass takes them off it.
    publish_burst(&mut r.publisher, 0, BEFORE, STALL_LEN, Duration::ZERO);
    let network = r.gateway.manager().network().expect("G has a network");
    network.set_lan_access(cerulion_core::LanAccess::Paired);
    network.set_lan_access(cerulion_core::LanAccess::Open);
    let bridges = r.gateway.manager().bridge_manager_arc();
    bridges.enable_bridge(&r.topic).expect("enable");
    assert!(
        bridges.is_enabled(&r.topic).expect("enabled?"),
        "PRECONDITION: the topic is demanded again after the reopen"
    );
    drive_for(&mut r.gateway, Duration::from_millis(300));
    r.offer(BEFORE, AFTER);
    r.settle();
    assert_eq!(
        r.got(),
        (BEFORE..BEFORE + AFTER).collect::<Vec<_>>(),
        "only frames published after the reopen arrive"
    );
}

/// A close that waits on a put already under way, while access reopens and
/// the topic is demanded again before that put returns: the frames queued
/// behind the put are discarded, not sent after the reopen.
#[test]
fn a_reopen_during_a_lan_close_does_not_release_frames_queued_before_it() {
    let mut r = StallRig::new("reopen", EgressReliability::Reliable, Some(8));
    r.pause();
    let offered = r.offer_until_stalled(8);
    let before = r.stats();
    assert_eq!(
        (before.queued, before.in_flight),
        (8, 1),
        "PRECONDITION: a full queue behind a blocked put: {before:?}"
    );
    let network = r.gateway.manager().network().expect("G has a network");
    let bridges = r.gateway.manager().bridge_manager_arc();
    let topic = r.topic.clone();
    std::thread::scope(|scope| {
        let closer = scope.spawn(|| network.set_lan_access(cerulion_core::LanAccess::Paired));
        // The close has stored the flags; the put under way still holds the
        // egress pass, so the close cannot return before the unpause below.
        assert!(
            wait_until(5, || bridges.is_lan_closed().unwrap_or(false)),
            "PRECONDITION: the close started"
        );
        network.set_lan_access(cerulion_core::LanAccess::Open);
        bridges.enable_bridge(&topic).expect("enable");
        assert!(
            bridges.is_enabled(&topic).expect("enabled?"),
            "PRECONDITION: the topic is demanded again while the close waits"
        );
        r.proxy.paused.store(false, Ordering::Relaxed);
        closer.join().expect("closer");
    });
    drive_for(&mut r.gateway, Duration::from_millis(500));
    r.settle();
    let s = r.stats();
    assert_eq!(
        s.forwarded,
        before.forwarded + 1,
        "only the put under way is sent: {s:?}"
    );
    assert!(s.discarded >= 8, "{s:?}");
    assert_eq!(accounted(&s), u64::from(offered), "{s:?}");
    // Control: the lane still delivers frames published after the reopen.
    const AFTER: u32 = 2;
    r.offer(offered, AFTER);
    r.settle();
    let mut want: Vec<u32> = (0..s.forwarded as u32).collect();
    want.extend(offered..offered + AFTER);
    assert_eq!(
        r.got(),
        want,
        "no frame queued before the close arrives after it; frames after the reopen do"
    );
}
