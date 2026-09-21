// SPDX-License-Identifier: AGPL-3.0-only
//! C4 end-to-end: the recorder NAMES the channels it can, DERIVES their
//! wire fixed size, and says plainly which ones it could not name.
//!
//! Same shape as the discovery siblings: an ISOLATED per-test iceoryx2
//! transport (so the live enumeration sees exactly this test's topics),
//! hand-built wire frames, hand-written expected values.
//!
//! The oracles here are deliberately EXTERNAL where they can be. The headline
//! arm publishes frames stamped with `geometry_msgs/Twist::SCHEMA_HASH` — the
//! constant `build.rs` derived for the compiled type, i.e. exactly what a real
//! publisher puts on the wire — and requires the recorder to come back with the
//! name `geometry_msgs/Twist` and its `WIRE_FIXED_SIZE`. Nothing in the test
//! computes either value; both are read off the generated type.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_bag::BagReader;
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, RecordCoverage, ReplayGrade, SchemaSource,
    TapSpec, RECORD_COVERAGE_ATTACHMENT,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use native_ros2_messages::geometry_msgs::Twist;

use common::*;

/// A hash NOTHING on this machine can resolve — the R5 fixture.
const UNKNOWN_HASH: u64 = 0x0981_0981_0981_0981;

/// The hash a hand-built `--schema-catalog` binds — the rung's fixture.
///
/// It is the REAL `std_msgs/String` hash, not an invented number, because the
/// grade now RECOMPUTES each channel's identity from the closure the bag ships:
/// a catalog binding some arbitrary hash to a real type NAME is a claim the
/// recomputation refuses, so an invented value would make the channel named but
/// undescribable — which is the check doing its job, not a usable fixture.
fn catalog_only_hash() -> u64 {
    <native_ros2_messages::std_msgs::String as ShmMessage>::SCHEMA_HASH
}

fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

/// Fast cadences, status OFF, discovery ON. `schema_demand` stays at the library
/// default of ZERO unless a test raises it, so no test here touches the network
/// by accident.
fn cfg_for(out: std::path::PathBuf, taps: Vec<TapSpec>, ready: std::path::PathBuf) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(1500);
    cfg.discover_live = true;
    cfg
}

fn producer(
    mgr: &TransportManager,
    topic: &str,
) -> cerulion_core::transport::publisher::CerulionPublisher {
    publisher_with_provisioning(mgr, topic, 8, 16, 4096)
}

fn publish_n(pubr: &mut cerulion_core::transport::publisher::CerulionPublisher, hash: u64, n: u32) {
    for i in 0..n {
        let frame = build_frame(hash, i, 1_000 + i as u64, b"probe");
        pubr.publish_raw(&frame).expect("publish_raw");
        settle();
    }
}

fn read_coverage(out: &std::path::Path) -> RecordCoverage {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in EVERY finalized bag");
    serde_json::from_slice(&att.data).expect("record_coverage.json parses")
}

/// The bag's own channel descriptor for `topic` — name + the decoded
/// `wire_fixed_size`, read back off the FILE rather than from the summary.
fn descriptor_of(out: &std::path::Path, topic: &str) -> (String, u32, u64) {
    let reader = BagReader::open(out).expect("open bag");
    let chan = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .unwrap_or_else(|| panic!("no channel for {topic}"));
    let d = chan
        .descriptor
        .expect("a cerulion channel carries a descriptor");
    (chan.schema_name, d.wire_fixed_size, d.schema_hash)
}

fn finish(
    handle: JoinHandle<Result<BagdSummary, BagdError>>,
    shutdown: &Arc<AtomicBool>,
) -> BagdSummary {
    shutdown.store(true, Ordering::Relaxed);
    handle.join().expect("bagd thread").expect("clean finalize")
}

// ===========================================================================
// The HEADLINE: a discovered channel gets a FULL descriptor.
// ===========================================================================

/// A DISCOVERED (undeclared) live producer whose wire hash this machine's own
/// corpus knows lands with a full descriptor — name AND a non-zero
/// `wire_fixed_size` — and the manifest attributes it to the rung that named it.
///
/// This is the gap this feature exists to close. Before it the same channel
/// recorded `schema_name = "unknown"`, `wire_fixed_size = 0`, and rendered
/// nowhere; the catalog resolves the NAME for a hash it has bound, but leaves
/// the size at zero on every path (`attach_schema_from_header` hardcodes it).
///
/// The oracle is EXTERNAL: the frames carry `Twist::SCHEMA_HASH` — the constant
/// `build.rs` derived for the compiled type, which is what a real publisher
/// stamps into every header — and the expected name and size are read off the
/// generated type, not computed here.
#[test]
fn a_discovered_channel_this_machine_can_name_gets_a_full_descriptor() {
    let mgr = make_manager(16);
    let topic = unique_topic("full");
    let out = unique_out("full");
    let ready = unique_out("full_ready");

    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    // NO taps named: the topic reaches the bag purely through live discovery,
    // so its schema can only come from the ladder.
    let other = unique_topic("full_decl");
    let mut other_pub = producer(&mgr, &other);
    let cfg = cfg_for(out.clone(), vec![TapSpec::attach(&other)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, Twist::SCHEMA_HASH, 3);
    publish_n(&mut other_pub, Twist::SCHEMA_HASH, 1);
    std::thread::sleep(Duration::from_millis(400));
    let summary = finish(handle, &shutdown);

    let (name, size, hash) = descriptor_of(&out, &topic);
    assert_eq!(
        name, "geometry_msgs/Twist",
        "the ladder must NAME a channel whose wire hash this machine's corpus knows"
    );
    assert_eq!(
        size as usize,
        Twist::WIRE_FIXED_SIZE,
        "and DERIVE its wire_fixed_size — the field `attach_schema_from_header` leaves at 0"
    );
    assert_ne!(
        size, 0,
        "precondition: Twist is a fixed type with real size"
    );
    assert_eq!(
        hash,
        Twist::SCHEMA_HASH,
        "the wire hash is recorded verbatim"
    );

    // The manifest attributes it to the rung that answered.
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::LocalCorpus),
        "this machine's own corpus named it — no peer was involved"
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Full),
        "every channel is named, so the bag is replay-grade by descriptor"
    );
    assert_eq!(summary.record_coverage, coverage, "summary == attachment");
}

/// The ANTI-TAUTOLOGY control: a channel whose hash NOTHING can resolve must
/// stay `"unknown"`, be labelled `Unresolved`, and drag the grade down.
///
/// Without it, "the ladder names channels" is satisfied by an implementation
/// that stamps a name on everything.
#[test]
fn a_channel_no_rung_can_name_stays_unknown_and_says_so() {
    let mgr = make_manager(16);
    let topic = unique_topic("norung");
    let out = unique_out("norung");
    let ready = unique_out("norung_ready");

    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, UNKNOWN_HASH, 3);
    std::thread::sleep(Duration::from_millis(400));
    finish(handle, &shutdown);

    let (name, size, hash) = descriptor_of(&out, &topic);
    assert_eq!(
        name, "unknown",
        "a name nobody can supply must not be invented"
    );
    assert_eq!(size, 0, "and no size may be invented either");
    assert_eq!(
        hash, UNKNOWN_HASH,
        "the wire hash is still recorded — that is all we know"
    );

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::Unresolved)
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Observability),
        "no channel is named, so the bag renders nowhere and says so"
    );
    // The run did NOT ask for promotion (schema_demand defaults to zero), so the
    // grade alone must not escalate — nobody promised anything.
    assert!(!coverage.schema_demand_requested);
    assert!(
        !coverage.is_incomplete(),
        "a run that never asked for promotion did not fail at it"
    );
}

// ===========================================================================
// C4-T4 — `all_channels_exact` keeps its v1 meaning while `replay_grade`
//         reports promotion. BOTH in one body, or a redefinition passes.
// ===========================================================================

/// The two fields answer DIFFERENT questions and must not be collapsed.
///
/// The discriminating shape is an ATTACH-MODE tap whose schema the ladder then
/// RESOLVES:
///
/// * `all_channels_exact` measures TAP MODE — the channel was learned from the
///   wire, so it is `false`, exactly as it was in v1. A v1 reader keeps reading
///   its own field correctly.
/// * `replay_grade` measures DESCRIPTOR COMPLETENESS — the channel carries a
///   full name, so the bag is `Full`.
///
/// Asserting only one of them would pass a variant that redefined
/// `all_channels_exact` in terms of the new grade, which is the misleading-name
/// trap this repo rejects; asserting both in one body is what forbids it.
#[test]
fn all_channels_exact_keeps_its_tap_mode_meaning_while_replay_grade_reports_promotion() {
    let mgr = make_manager(16);
    let topic = unique_topic("grade");
    let out = unique_out("grade");
    let ready = unique_out("grade_ready");

    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    // ATTACH mode: the tap is named, but its schema is learned from the wire.
    let cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, Twist::SCHEMA_HASH, 3);
    std::thread::sleep(Duration::from_millis(400));
    finish(handle, &shutdown);

    let coverage = read_coverage(&out);

    // The v1 field, UNCHANGED: this channel is attach-mode, so not "exact".
    assert!(
        !coverage.all_channels_exact,
        "all_channels_exact measures TAP MODE and must keep saying so — an attach-mode \
         channel is not exact however completely it is named"
    );

    // The v2 field: the SAME channel is fully named, so the bag IS replay-grade.
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Full),
        "replay_grade measures DESCRIPTOR COMPLETENESS, which promotion changed"
    );
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::LocalCorpus)
    );
    let (name, size, _) = descriptor_of(&out, &topic);
    assert_eq!(name, "geometry_msgs/Twist");
    assert_eq!(size as usize, Twist::WIRE_FIXED_SIZE);
}

// ===========================================================================
// C4-T3 — the demand budget cannot extend bag creation.
// ===========================================================================

/// A netd that ACCEPTS the connection and then never speaks.
///
/// This is the adversarial shape, not an absent daemon: `connect_existing`
/// succeeds, and the client's own `ROUNDTRIP_TIMEOUT` (5 s) then blocks the
/// resolver thread inside its handshake. If `ensure_writer` shared any lock with
/// that round trip, bag creation would be pinned behind it.
struct SilentNetd {
    _dir: std::path::PathBuf,
    stop: Arc<AtomicBool>,
    accepts: Arc<std::sync::atomic::AtomicUsize>,
    handle: Option<JoinHandle<()>>,
}

impl SilentNetd {
    fn spawn(tag: &str) -> (Self, std::path::PathBuf) {
        let dir = unique_out(tag);
        std::fs::create_dir_all(&dir).expect("mk socket dir");
        let sock = dir.join("netd.sock");
        let listener =
            std::os::unix::net::UnixListener::bind(&sock).expect("bind the silent netd socket");
        listener
            .set_nonblocking(true)
            .expect("nonblocking so the thread can observe the stop flag");
        let stop = Arc::new(AtomicBool::new(false));
        let accepts = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let (s, a) = (Arc::clone(&stop), Arc::clone(&accepts));
        let handle = std::thread::spawn(move || {
            // Accept and HOLD — never write the Hello banner.
            let mut held = Vec::new();
            while !s.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        a.fetch_add(1, Ordering::SeqCst);
                        held.push(stream);
                    }
                    Err(_) => std::thread::sleep(Duration::from_millis(10)),
                }
            }
        });
        (
            Self {
                _dir: dir,
                stop,
                accepts,
                handle: Some(handle),
            },
            sock,
        )
    }

    /// How many connections this fake daemon has taken — the ANTI-VACUITY
    /// oracle. Zero means the resolver never reached it, and the arm below
    /// would then be proving nothing about a blocking round trip.
    fn accepts(&self) -> usize {
        self.accepts.load(Ordering::SeqCst)
    }
}

impl Drop for SilentNetd {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// RAII env guard — this arm mutates PROCESS env, hence `#[serial]` too.
struct EnvGuard(&'static str, Option<String>);
impl EnvGuard {
    fn set(key: &'static str, val: &std::path::Path) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        EnvGuard(key, prev)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.1 {
            Some(v) => std::env::set_var(self.0, v),
            None => std::env::remove_var(self.0),
        }
    }
}

/// The budget bounds the RESOLVER, never bag creation.
///
/// With a netd that accepts and then goes silent, the resolver's very first
/// round trip blocks for the client's full 5 s read deadline. Bag creation must
/// happen on its own schedule regardless — and it does, structurally, because
/// the resolver publishes by an `Arc` swap and `ensure_writer` only ever clones
/// a pointer out of that lock.
///
/// THE ORACLE IS THE BAG FILE APPEARING WHILE THE RUN IS STILL GOING, and that
/// choice is forced by what a blocked resolver read can and cannot move.
///
/// An assertion on the recorder's own `channel_set_closed_after` does NOT
/// catch it: the stamp is taken at the TOP of
/// `ensure_writer`, before the resolver's answers are read, so it cannot see a
/// read that blocks below it. Under a blocked read the wall moves —
/// the binary goes from 1.09 s to 5.08 s — but nothing fails.
///
/// The observable that actually carries the property is the bag FILE, which
/// `spawn_writer_thread` creates AFTER the answers are consulted. So the arm
/// waits for it BEFORE shutting down: under a healthy recorder it appears at the
/// `DISCOVERY_SETTLE_MIN` floor (~500 ms); under a resolver that holds its
/// publish lock across the round trip it cannot appear until the client's 5 s
/// read deadline expires.
///
/// MARGINS, stated because a ceiling needs them: healthy ~500 ms, ceiling 3 s
/// (6x headroom), blocked >= 5 s. `channel_set_closed_after` is still asserted
/// underneath — it remains a real bound on the settle hold — but it is not
/// what carries this arm.
#[test]
#[serial_test::serial]
fn an_unreachable_netd_cannot_delay_bag_creation() {
    let (netd, sock) = SilentNetd::spawn("silent");
    // `cerulion_netd::default_socket_path()` reads exactly this key; the
    // resolver thread inherits the process env.
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    let mgr = make_manager(16);
    let topic = unique_topic("budget");
    let out = unique_out("budget");
    let ready = unique_out("budget_ready");

    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    // ASK for promotion, with a budget far longer than the ceiling below — so a
    // budget that leaked into bag creation would be plainly visible.
    cfg.schema_demand = Duration::from_secs(30);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, Twist::SCHEMA_HASH, 2);

    // THE LOAD-BEARING ASSERTION, taken while the run is STILL GOING: the bag
    // file is created by `spawn_writer_thread`, which runs AFTER `ensure_writer`
    // consults the resolver — so its appearance is the observable that a
    // blocking read would move. A shutdown-time reading cannot serve: shutdown
    // force-creates, which reaches the same line by a different route.
    let bag_appeared = wait_for_file(&out, Duration::from_secs(3));

    std::thread::sleep(Duration::from_millis(300));
    let summary = finish(handle, &shutdown);

    // ANTI-VACUITY, asserted FIRST — and it really is first now: the resolver
    // must have CONNECTED to the silent daemon and be sitting in a blocked round
    // trip. If it never got there — a mis-named env key, a constructor that gave
    // up early — every assertion below is trivially satisfied and proves nothing
    // at all. This is the check that caught exactly that mistake while this test
    // was written, so it must not sit downstream of the claim it protects.
    assert!(
        netd.accepts() >= 1,
        "precondition: the schema resolver must have CONNECTED to the silent netd \
         (accepts = {}). Without a blocked round trip this arm cannot distinguish a \
         non-blocking design from a lucky one",
        netd.accepts()
    );
    drop(netd);

    assert!(
        bag_appeared,
        "bag creation must not wait on the resolver: with a 30 s demand budget and a netd \
         that never answers, the bag file did not appear within the ceiling"
    );

    let held = summary
        .channel_set_closed_after
        .expect("the recorder dates its own hold");
    assert!(
        held < Duration::from_secs(3),
        "bag creation must not wait on the resolver: the channel set closed after {held:?}, \
         which is past the ceiling — a 30 s demand budget and a netd that never answers must \
         not be able to move this number at all"
    );

    // …and the recording is otherwise NORMAL: the local rung still named the
    // channel, so an unreachable peer costs nothing beyond the peer's answer.
    let coverage = read_coverage(&out);
    assert!(
        coverage.schema_demand_requested,
        "precondition: this run really did ask for promotion"
    );
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::LocalCorpus),
        "the LOCAL rung needs no network and must still answer"
    );
    assert_eq!(coverage.replay_grade, Some(ReplayGrade::Full));
}

// ===========================================================================
// C4-T4 — the rungs that need NO network, and the provenance each one claims.
// ===========================================================================

/// R0: a channel the GRAPH declared is stamped `Declared`, keeps the graph's own
/// descriptor, and COUNTS.
///
/// `graph run --record` hands bagd `--topics-json`, i.e. EXACT-mode taps, so on
/// the flagship path every channel takes this rung — which makes it the one rung
/// whose absence is invisible to a suite built out of attach-mode taps. Skipping
/// declared channels in the ladder left `schema_sources` EMPTY on such a run, so
/// `replay_grade` came back `None` (no claim at all) while every other arm
/// stayed green.
#[test]
fn a_graph_declared_channel_is_stamped_declared_and_counts_toward_the_grade() {
    let mgr = make_manager(16);
    let topic = unique_topic("decl");
    let out = unique_out("decl");
    let ready = unique_out("decl_ready");

    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    // The EXACT-mode shape: the graph names the port, so the descriptor arrives
    // with the tap rather than off the wire.
    let declared = cerulion_bag::TopicSchema {
        topic: topic.clone(),
        schema_name: "geometry_msgs/Twist".to_string(),
        schema_hash: Twist::SCHEMA_HASH,
        wire_fixed_size: Twist::WIRE_FIXED_SIZE as u32,
    };
    let mut cfg = cfg_for(
        out.clone(),
        vec![TapSpec::exact(&topic, declared)],
        ready.clone(),
    );
    // Discovery OFF: this arm is about the DECLARED rung alone, and a co-tenant
    // topic would add a channel whose provenance is a different question.
    cfg.discover_live = false;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, Twist::SCHEMA_HASH, 3);
    std::thread::sleep(Duration::from_millis(300));
    finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::Declared),
        "the graph is the source of truth for its own ports, and the manifest must say so"
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Full),
        "a declared channel is describable, so an all-declared run is replay-grade — \
         `None` here means the ladder never looked at it"
    );
    let (name, size, _) = descriptor_of(&out, &topic);
    assert_eq!(
        name, "geometry_msgs/Twist",
        "and the graph's own name is kept, never re-derived"
    );
    assert_eq!(size as usize, Twist::WIRE_FIXED_SIZE);
}

/// R1 via the hash→name catalog: the rung claims provenance for the
/// channels it NAMED, and for no others.
///
/// Two channels, one hash each; the catalog binds exactly one. Stamping the
/// source outside the `if let` that does the naming makes this rung claim credit
/// for a channel it merely looked at — a manifest that reports `LocalCorpus` for
/// a channel recorded as `"unknown"`. No bagd test set `schema_catalog` at all
/// before this one, so that mutation survived the whole suite.
#[test]
fn the_catalog_rung_claims_provenance_only_for_the_channels_it_named() {
    let mgr = make_manager(16);
    let known = unique_topic("cat_known");
    let opaque = unique_topic("cat_opaque");
    let out = unique_out("cat");
    let ready = unique_out("cat_ready");

    // A hash→name binding for a type carrying NO text — exactly what
    // `build_record_schema_catalog` contributes for a built-in. Its NAME is a
    // real built-in, so the bag can still render it (readers compile it in).
    let catalog = cerulion_bag::BagSchemaCatalog::new(
        Vec::new(),
        vec![cerulion_core::SchemaHashName {
            schema_hash: catalog_only_hash(),
            qualified: "std_msgs/String".to_string(),
        }],
    );

    let mut known_pub = producer(&mgr, &known);
    let mut opaque_pub = producer(&mgr, &opaque);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(
        out.clone(),
        vec![TapSpec::attach(&known), TapSpec::attach(&opaque)],
        ready.clone(),
    );
    cfg.schema_catalog = Some(catalog);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut known_pub, catalog_only_hash(), 2);
    publish_n(&mut opaque_pub, UNKNOWN_HASH, 2);
    std::thread::sleep(Duration::from_millis(400));
    finish(handle, &shutdown);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&known].schema_source,
        Some(SchemaSource::LocalCorpus),
        "the catalog bound this hash, so this machine's own corpus is what named it"
    );
    assert_eq!(
        coverage.tapped[&opaque].schema_source,
        Some(SchemaSource::Unresolved),
        "nothing named this one — a rung must not claim a channel it did not name"
    );
    let (known_name, _, _) = descriptor_of(&out, &known);
    let (opaque_name, _, _) = descriptor_of(&out, &opaque);
    assert_eq!(known_name, "std_msgs/String");
    assert_eq!(
        opaque_name, "unknown",
        "the anti-tautology half: the unnamed channel really did stay unnamed"
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Partial),
        "one describable channel beside one that is not"
    );
}

// ===========================================================================
// C4-T5 — the DEMAND rungs, driven through a REAL netd control seam.
// ===========================================================================

/// The bag's own schema-doc closure, read back off the FILE.
fn bag_schema_docs(out: &std::path::Path) -> Vec<cerulion_core::SchemaDoc> {
    let reader = BagReader::open(out).expect("open bag");
    match reader
        .attachment(cerulion_bag::SCHEMA_DOCS_ATTACHMENT)
        .expect("read attachments")
    {
        Some(att) => {
            let cat: cerulion_bag::BagSchemaCatalog =
                cerulion_bag::BagSchemaCatalog::decode(&att.data).expect("schema catalog decodes");
            cat.docs
        }
        // No attachment at all is the correct empty answer: `write_schema_catalog`
        // writes nothing when the closure is empty.
        None => Vec::new(),
    }
}

/// A netd that SPEAKS: it serves a `Hello`, then answers `query_catalog` and
/// `query_schema` from a hand-written script.
///
/// This is the harness the first wave of this feature did not have, and its absence
/// is why a green suite proved nothing about the feature's headline: no test
/// anywhere implemented `SchemaOracle`, so the ENTIRE networked demand loop —
/// `NetdOracle`, the catalog fan-out, the schema query, the verification, the
/// provenance stamp, the doc carriage — was deletable with all 92 arms passing.
/// It speaks the real NDJSON control protocol over a real `UnixListener`, so it
/// drives the production `NetdOracle::connect` path rather than a seam built for
/// the test.
///
/// Deliberately NOT a positional script of canned answers: it decodes each
/// request and answers the one it was asked, because the resolver RE-ASKS and a
/// positional script would silently pair a retry with the previous answer.
struct ScriptedNetd {
    _dir: std::path::PathBuf,
    stop: Arc<AtomicBool>,
    catalog_calls: Arc<std::sync::atomic::AtomicUsize>,
    schema_calls: Arc<std::sync::atomic::AtomicUsize>,
    handle: Option<JoinHandle<()>>,
}

/// What one scripted robot serves.
#[derive(Clone)]
struct ServedType {
    robot: String,
    topic: String,
    qualified: String,
    text: String,
    /// Other qualified names this doc's text references — served verbatim on the
    /// `SchemaDoc`, which is how a real peer declares its closure.
    deps: Vec<String>,
}

impl ScriptedNetd {
    fn spawn(tag: &str, served: Vec<ServedType>) -> (Self, std::path::PathBuf) {
        use std::io::{Read, Write};

        let dir = unique_out(tag);
        std::fs::create_dir_all(&dir).expect("mk socket dir");
        let sock = dir.join("n.sock");
        // A UDS path over `sun_path` fails at BIND with an errno that reads like
        // a harness bug; say so up front instead.
        assert!(
            sock.as_os_str().len() < 100,
            "the scripted netd's socket path must fit sun_path; got {} bytes: {}",
            sock.as_os_str().len(),
            sock.display()
        );
        let listener =
            std::os::unix::net::UnixListener::bind(&sock).expect("bind the scripted netd socket");
        listener
            .set_nonblocking(true)
            .expect("nonblocking so the thread can observe the stop flag");
        let stop = Arc::new(AtomicBool::new(false));
        let catalog_calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let schema_calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let (s, cc, sc) = (
            Arc::clone(&stop),
            Arc::clone(&catalog_calls),
            Arc::clone(&schema_calls),
        );

        let handle = std::thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                let stream = match listener.accept() {
                    Ok((stream, _)) => stream,
                    Err(_) => {
                        std::thread::sleep(Duration::from_millis(5));
                        continue;
                    }
                };
                // A read timeout is what lets the serving loop observe `stop`
                // instead of parking forever on a client that has gone quiet.
                stream
                    .set_read_timeout(Some(Duration::from_millis(100)))
                    .ok();
                let Ok(mut w) = stream.try_clone() else {
                    continue;
                };
                let banner = cerulion_netd::protocol::Hello::new().to_json_line();
                if writeln!(w, "{banner}").is_err() {
                    continue;
                }
                let _ = w.flush();

                // Framing is done BY HAND over the raw stream rather than with
                // `BufReader::read_line`, and that is not a style choice: the
                // read timeout above (which is what lets this loop observe
                // `stop`) makes timeouts ROUTINE — the resolver's rounds are a
                // `RESOLVER_RETRY_INTERVAL` apart — and a timed-out `read_line`
                // discards whatever partial line it had accumulated. MEASURED:
                // that silently ate a request, the client then blocked on its
                // own 5 s deadline, and the connection desynchronised for the
                // rest of the run (`catalog=2, schema=0` on a test whose
                // recorder was behaving perfectly). Accumulating into a buffer
                // that survives a timeout is what makes the fake faithful.
                let mut stream = stream;
                let mut buf: Vec<u8> = Vec::new();
                let mut chunk = [0u8; 4096];
                loop {
                    if s.load(Ordering::Relaxed) {
                        break;
                    }
                    match stream.read(&mut chunk) {
                        Ok(0) => break,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                        Err(e)
                            if e.kind() == std::io::ErrorKind::WouldBlock
                                || e.kind() == std::io::ErrorKind::TimedOut =>
                        {
                            continue
                        }
                        Err(_) => break,
                    }
                    let Some(pos) = buf.iter().position(|b| *b == b'\n') else {
                        continue;
                    };
                    let line: Vec<u8> = buf.drain(..=pos).collect();
                    let Ok(v) = serde_json::from_slice::<serde_json::Value>(&line) else {
                        continue;
                    };
                    let id = v["id"].as_u64().unwrap_or(0);
                    let resp = match v["method"].as_str() {
                        Some("query_catalog") => {
                            cc.fetch_add(1, Ordering::SeqCst);
                            let catalogs = served
                                .iter()
                                .map(|t| cerulion_core::CatalogReply {
                                    version:
                                        cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION,
                                    robot: t.robot.clone(),
                                    entries: vec![cerulion_core::CatalogEntry {
                                        topic: t.topic.clone(),
                                        // Deliberately NONE: the peer's own
                                        // claimed hash is never used as
                                        // evidence, so the fake does not supply
                                        // one — the recorder must recompute.
                                        schema_hash: None,
                                        schema_name: Some(t.qualified.clone()),
                                        provenance: cerulion_core::CatalogProvenance::Runtime,
                                        producer_count: Some(1),
                                        liveness: None,
                                    }],
                                    error: None,
                                })
                                .collect();
                            cerulion_netd::protocol::Response::CatalogQuery(
                                cerulion_netd::protocol::CatalogQueryResponse {
                                    id,
                                    catalogs,
                                    discovery: Default::default(),
                                    plane_unsettled_ms: None,
                                },
                            )
                        }
                        Some("query_schema") => {
                            sc.fetch_add(1, Ordering::SeqCst);
                            let requested = v["requested"].as_str().unwrap_or_default().to_string();
                            // A real `query_schema` serves the requested type
                            // AND its CLOSURE — that is what makes the served
                            // doc usable, and it is also what puts a peer's
                            // version of a nested type on the wire beside the
                            // root that names it.
                            let doc_for = |t: &ServedType| cerulion_core::SchemaDoc {
                                qualified: t.qualified.clone(),
                                encoding: cerulion_core::SchemaEncoding::Msg,
                                text: t.text.clone(),
                                deps: t.deps.clone(),
                            };
                            let replies = served
                                .iter()
                                .filter(|t| t.qualified == requested)
                                .map(|t| {
                                    let mut docs = vec![doc_for(t)];
                                    for dep in &t.deps {
                                        if let Some(d) = served.iter().find(|o| &o.qualified == dep)
                                        {
                                            docs.push(doc_for(d));
                                        }
                                    }
                                    cerulion_core::SchemaReply::found(&t.robot, &t.qualified, docs)
                                })
                                .collect();
                            cerulion_netd::protocol::Response::SchemaQuery(
                                cerulion_netd::protocol::SchemaQueryResponse {
                                    id,
                                    replies,
                                    discovery: Default::default(),
                                    plane_unsettled_ms: None,
                                },
                            )
                        }
                        _ => continue,
                    };
                    if writeln!(w, "{}", resp.to_json_line()).is_err() {
                        break;
                    }
                    let _ = w.flush();
                }
            }
        });
        (
            Self {
                _dir: dir,
                stop,
                catalog_calls,
                schema_calls,
                handle: Some(handle),
            },
            sock,
        )
    }

    fn catalog_calls(&self) -> usize {
        self.catalog_calls.load(Ordering::SeqCst)
    }
    fn schema_calls(&self) -> usize {
        self.schema_calls.load(Ordering::SeqCst)
    }
}

impl Drop for ScriptedNetd {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// A netd that serves its `Hello` and then HANGS UP — the one shape that
/// reaches the first-request retry arm.
///
/// Its oracle is the ACCEPT COUNT: the plain `query_catalog` answers an
/// accepted-then-EOF first request by calling `reconnect()` →
/// `connect_or_spawn_at`, which dials the still-listening socket a SECOND time
/// (and, on a machine with the daemon binary reachable, SPAWNS one). The
/// no-respawn verb reports the error instead, so the recorder touches the socket
/// exactly ONCE.
struct HangUpNetd {
    _dir: std::path::PathBuf,
    stop: Arc<AtomicBool>,
    accepts: Arc<std::sync::atomic::AtomicUsize>,
    handle: Option<JoinHandle<()>>,
}

impl HangUpNetd {
    fn spawn(tag: &str) -> (Self, std::path::PathBuf) {
        use std::io::Write;
        let dir = unique_out(tag);
        std::fs::create_dir_all(&dir).expect("mk socket dir");
        let sock = dir.join("n.sock");
        assert!(sock.as_os_str().len() < 100, "sun_path");
        let listener =
            std::os::unix::net::UnixListener::bind(&sock).expect("bind the hang-up netd socket");
        listener.set_nonblocking(true).expect("nonblocking");
        let stop = Arc::new(AtomicBool::new(false));
        let accepts = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let (s, a) = (Arc::clone(&stop), Arc::clone(&accepts));
        let handle = std::thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((mut stream, _)) => {
                        a.fetch_add(1, Ordering::SeqCst);
                        let banner = cerulion_netd::protocol::Hello::new().to_json_line();
                        let _ = writeln!(stream, "{banner}");
                        let _ = stream.flush();
                        // …and drop it: the client's next read sees EOF.
                    }
                    Err(_) => std::thread::sleep(Duration::from_millis(5)),
                }
            }
        });
        (
            Self {
                _dir: dir,
                stop,
                accepts,
                handle: Some(handle),
            },
            sock,
        )
    }

    fn accepts(&self) -> usize {
        self.accepts.load(Ordering::SeqCst)
    }
}

impl Drop for HangUpNetd {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The recorder touches the daemon socket EXACTLY ONCE against a netd that
/// hangs up after its `Hello`.
///
/// `connect_existing` exists so a recorder never STARTS a network daemon on the
/// machine it is recording — but that contract covered the constructor only.
/// The plain `query_catalog` carries the first-request retry, whose
/// `reconnect()` runs the whole connect-or-SPAWN ladder, and `first_request` is
/// `next_id == 1`, always true of the recorder's first query. MEASURED against
/// this exact shape: a daemon started and the call blocked 10.06 s.
///
/// The oracle is the ACCEPT COUNT rather than a wall, because load cannot fake
/// a second dial. It is the BEHAVIOURAL twin of the structural adoption pin in
/// `schema_resolve.rs` — that one proves the call site names the right verb,
/// this one proves the socket is only ever touched once.
#[test]
#[serial_test::serial]
fn the_recorder_never_re_dials_a_daemon_that_hangs_up_after_its_hello() {
    let (netd, sock) = HangUpNetd::spawn("srhu");
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    let mgr = make_manager(16);
    let topic = unique_topic("hangup");
    let out = unique_out("hangup");
    let ready = unique_out("hangup_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    // Long enough that the retry loop runs several rounds against the dead
    // daemon — a re-dialling verb would take its second accept in the first one.
    cfg.schema_demand = Duration::from_secs(2);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, Twist::SCHEMA_HASH, 2);
    // ANTI-VACUITY: the resolver must really have reached the daemon, or the
    // count below is 0 for the wrong reason.
    assert!(
        await_condition(Duration::from_secs(8), || netd.accepts() >= 1),
        "precondition: the schema resolver must have CONNECTED to the hang-up netd"
    );
    // Let the retry loop spend its budget against the dead connection.
    std::thread::sleep(Duration::from_millis(2500));
    finish(handle, &shutdown);

    let accepts = netd.accepts();
    drop(netd);
    assert_eq!(
        accepts, 1,
        "the recorder must touch the daemon socket exactly ONCE. A second accept is a RE-DIAL \
         — the client's retry arm is the one that exists today, and its `reconnect()` runs the \
         connect-or-SPAWN ladder `connect_existing` exists to keep the recorder out of, but the \
         assertion is on the dial itself, so a future non-spawning reconnect is caught too"
    );

    // …and the recording is otherwise normal — the LOCAL rung still named it.
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::LocalCorpus)
    );
}

/// A type NO machine in this repo compiles in — the peer-only shape the demand
/// rung exists for.
const PEER_TEXT: &str = "float64 x\nfloat64 y\nfloat64 z\n";
const PEER_TYPE: &str = "peer_msgs/Widget";

/// The recipe-3 hash and fixed size a real publisher of [`PEER_TYPE`] would put
/// on the wire — derived HERE from the same text the fake serves, through the
/// repo's own parser, so the fixture cannot drift from what the recorder
/// recomputes without this test noticing.
fn peer_identity() -> (u64, u32) {
    let mut schemas =
        vec![
            cerulion_core::codegen::parse_rosmsg(PEER_TEXT, "Widget", Some("peer_msgs"))
                .expect("the peer fixture must parse"),
        ];
    let warnings = cerulion_core::codegen::resolve_fixed_nested(&mut schemas);
    assert!(
        warnings.is_empty(),
        "the peer fixture must resolve cleanly: {warnings:?}"
    );
    let s = &schemas[0];
    (s.schema_hash(), s.wire_fixed_size() as u32)
}

/// THE headline demand arm: a peer names a type this machine has never had, the
/// recorder VERIFIES it against the wire, and the bag comes back able to render
/// it — with the manifest saying who supplied it.
///
/// Three properties in one body, because they are one claim:
///
/// 1. PROVENANCE — `SchemaSource::Demanded { robot }`, not `LocalCorpus`. The
///    resolver folds served docs into the corpus the ladder's LOCAL rung
///    reads, so without care every successful demand is reported as something this
///    machine already had: the field answering the opposite of its own question,
///    100 % of the time, on the one path the feature exists for.
/// 2. VERIFICATION — the stamped `wire_fixed_size` is DERIVED from the served
///    text, and it is compared against an identity this test computes
///    independently through the repo's own parser rather than against itself.
/// 3. RENDERABILITY — the served TEXT lands in the bag's schema closure. A name
///    whose definition never reached the bag reads, on every machine but the
///    recorder, exactly like the `"unknown"` channel it replaced.
///
/// This is also the no-inert-shipping proof for the whole networked half: it
/// fails if the demand loop, the oracle, or the doc carriage is deleted.
#[test]
#[serial_test::serial]
fn a_peer_served_type_is_verified_named_attributed_and_carried_into_the_bag() {
    let topic = unique_topic("demand");
    let (peer_hash, peer_size) = peer_identity();
    let (netd, sock) = ScriptedNetd::spawn(
        "srdn",
        vec![ServedType {
            robot: "go2".to_string(),
            topic: topic.clone(),
            qualified: PEER_TYPE.to_string(),
            text: PEER_TEXT.to_string(),
            deps: Vec::new(),
        }],
    );
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    let mgr = make_manager(16);
    let out = unique_out("demand");
    let ready = unique_out("demand_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_demand = Duration::from_secs(10);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, peer_hash, 3);
    // A CONDITION, not a wall bet: wait until the daemon has actually served the
    // schema query, so a loaded runner delays this arm rather than failing it.
    assert!(
        await_condition(Duration::from_secs(8), || netd.schema_calls() >= 1),
        "the recorder never asked the daemon for a schema (catalog={}, schema={})",
        netd.catalog_calls(),
        netd.schema_calls()
    );
    // …then let the answer reach the channel set.
    std::thread::sleep(Duration::from_millis(600));
    finish(handle, &shutdown);

    // ANTI-VACUITY FIRST: the resolver really talked to the daemon. Without it
    // every assertion below could be satisfied by a run that never demanded.
    assert!(
        netd.catalog_calls() >= 1 && netd.schema_calls() >= 1,
        "precondition: the recorder must have asked the daemon (catalog={}, schema={})",
        netd.catalog_calls(),
        netd.schema_calls()
    );
    drop(netd);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::Demanded {
            robot: "go2".to_string()
        }),
        "a peer supplied this identity and the manifest must say so — reporting LocalCorpus \
         here is the exact false claim the field was added to prevent"
    );

    let (name, size, hash) = descriptor_of(&out, &topic);
    assert_eq!(name, PEER_TYPE, "the verified name is stamped");
    assert_eq!(
        size, peer_size,
        "the fixed size is DERIVED from the served text, never guessed"
    );
    assert_ne!(size, 0, "precondition: the fixture is a fixed-size type");
    assert_eq!(hash, peer_hash, "the wire hash is recorded verbatim");

    let docs = bag_schema_docs(&out);
    assert!(
        docs.iter()
            .any(|d| d.qualified == PEER_TYPE && d.text == PEER_TEXT),
        "the VERIFIED served text must ride into the bag's schema closure, VERBATIM — a name \
         whose definition never reached the bag renders nowhere but the recording machine, \
         which is precisely the state the demand rung exists to leave behind. docs = {:?}",
        docs.iter().map(|d| &d.qualified).collect::<Vec<_>>()
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Full),
        "named AND carrying its definition ⇒ replay-grade"
    );
}

/// THE CLOSURE THE BAG SHIPS MUST RECOMPUTE THE HASH ON ITS OWN CHANNELS.
///
/// This is the seam one level below the verification rung, and gating only the ROOT
/// left it open. `verify_served_schema` accepts a root by recomputing it over
/// the corpus — in which OUR definition of any duplicated name won the dedupe —
/// so a root can be accepted BECAUSE OF our `pkg/Bar` while the frontier below
/// it walked into `served_docs` and shipped the PEER's dropped `pkg/Bar`. A
/// reader then recomputes the root from the bag's own closure and gets a
/// DIFFERENT hash from the one stamped on the channel, while the manifest grades
/// it `Full`.
///
/// The scenario is built exactly that way: this machine defines `dep_msgs/Leaf`
/// (via `--schema-catalog`), the peer serves a DIVERGENT `dep_msgs/Leaf`
/// alongside the root that nests it, and the WIRE carries the identity our Leaf
/// produces — so the root verifies and is named. What the bag must not do is
/// carry the peer's Leaf.
///
/// The assertion is the RECOMPUTATION, not a doc inventory: it rebuilds the
/// reader's view (built-ins + the bag's shipped docs) and requires the root's
/// identity to equal the channel's recorded hash. That is the property a doc
/// list cannot state, and it is also the guard for the OTHER direction — a gate
/// that over-filtered and dropped a legitimate dep would fail it too.
#[test]
#[serial_test::serial]
fn the_closure_the_bag_ships_recomputes_the_hash_stamped_on_its_channel() {
    const LEAF: &str = "dep_msgs/Leaf";
    const ROOT: &str = "dep_msgs/Root";
    const OUR_LEAF_TEXT: &str = "float64 a\nfloat64 b\n";
    const PEER_LEAF_TEXT: &str = "float64 a\nfloat64 b\nfloat64 c\nfloat64 d\n";
    const ROOT_TEXT: &str = "dep_msgs/Leaf leaf\n";

    // The identity OUR definitions produce — what a publisher on this machine
    // would put on the wire, computed through the repo's own parser.
    let mut ours = vec![
        cerulion_core::codegen::parse_rosmsg(OUR_LEAF_TEXT, "Leaf", Some("dep_msgs"))
            .expect("leaf parses"),
        cerulion_core::codegen::parse_rosmsg(ROOT_TEXT, "Root", Some("dep_msgs"))
            .expect("root parses"),
    ];
    let warnings = cerulion_core::codegen::resolve_fixed_nested(&mut ours);
    assert!(warnings.is_empty(), "fixture must resolve: {warnings:?}");
    let root_hash = ours
        .iter()
        .find(|s| s.qualified_name() == ROOT)
        .expect("root indexed")
        .schema_hash();

    let topic = unique_topic("srdep");
    let (netd, sock) = ScriptedNetd::spawn(
        "srdp",
        vec![
            ServedType {
                robot: "go2".to_string(),
                topic: topic.clone(),
                qualified: ROOT.to_string(),
                text: ROOT_TEXT.to_string(),
                deps: vec![LEAF.to_string()],
            },
            // The peer's DIVERGENT leaf, reachable only through the root's deps.
            ServedType {
                robot: "go2".to_string(),
                topic: format!("{topic}/unused"),
                qualified: LEAF.to_string(),
                text: PEER_LEAF_TEXT.to_string(),
                deps: Vec::new(),
            },
        ],
    );
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    // THIS MACHINE's own definitions, handed in as `--schema-catalog`.
    let catalog = cerulion_bag::BagSchemaCatalog::new(
        vec![cerulion_core::SchemaDoc {
            qualified: LEAF.to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: OUR_LEAF_TEXT.to_string(),
            deps: Vec::new(),
        }],
        Vec::new(),
    );

    let mgr = make_manager(16);
    let out = unique_out("srdep");
    let ready = unique_out("srdep_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_demand = Duration::from_secs(10);
    cfg.schema_catalog = Some(catalog);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, root_hash, 3);
    assert!(
        await_condition(Duration::from_secs(8), || netd.schema_calls() >= 1),
        "the recorder never asked the daemon (catalog={}, schema={})",
        netd.catalog_calls(),
        netd.schema_calls()
    );
    std::thread::sleep(Duration::from_millis(600));
    finish(handle, &shutdown);
    drop(netd);

    let (name, _, hash) = descriptor_of(&out, &topic);
    assert_eq!(name, ROOT, "precondition: the root was named");
    assert_eq!(hash, root_hash, "precondition: the wire hash is recorded");

    // THE ASSERTION: rebuild the reader's corpus from the bag's own closure and
    // require it to reproduce the identity the channel claims.
    let docs = bag_schema_docs(&out);
    let mut reader: Vec<cerulion_core::codegen::MessageSchema> = Vec::new();
    for d in &docs {
        let (pkg, ty) = d.qualified.split_once('/').expect("qualified name");
        if let Ok(s) = cerulion_core::codegen::parse_rosmsg(&d.text, ty, Some(pkg)) {
            reader.push(s);
        }
    }
    let _ = cerulion_core::codegen::resolve_fixed_nested(&mut reader);
    let recomputed = reader
        .iter()
        .find(|s| s.qualified_name() == ROOT)
        .map(|s| s.schema_hash());
    assert_eq!(
        recomputed,
        Some(root_hash),
        "the bag's OWN closure must recompute the hash stamped on the channel — it shipped \
         {:?}, and the leaf text it carries is {:?}",
        docs.iter().map(|d| &d.qualified).collect::<Vec<_>>(),
        docs.iter()
            .find(|d| d.qualified == LEAF)
            .map(|d| d.text.as_str())
    );
    // …and concretely: OUR leaf, never the peer's.
    let leaf = docs
        .iter()
        .find(|d| d.qualified == LEAF)
        .expect("the closure must carry the leaf the root needs");
    assert_eq!(
        leaf.text, OUR_LEAF_TEXT,
        "a name this machine defines ships OUR definition, never the peer's dropped one"
    );
}

/// A peer's doc under a VENDORED name never reaches the attachment.
///
/// The other half of the deps hole, and it needs no `--schema-catalog` at all:
/// a built-in carries no local doc (the schema attachment omits their text, because every
/// reader compiles them), so a local-first lookup MISSES on a vendored name and
/// would fall through to whatever the peer served. A peer whose
/// `geometry_msgs/Vector3` has drifted then ships its text into the bag under a
/// name every reader already owns.
///
/// The self-consistency check cannot see this one: the reader's corpus puts
/// vendored types FIRST, so the dedupe keeps ours and the root still recomputes
/// — the bag is graded correctly while carrying a foreign definition under a
/// vendored name, which is a claim the recording has no business making. Hence
/// a separate arm, asserting the attachment's CONTENTS.
#[test]
#[serial_test::serial]
fn a_peers_definition_of_a_vendored_type_never_reaches_the_bag() {
    const ROOT: &str = "bi_msgs/Wrapper";
    const ROOT_TEXT: &str = "geometry_msgs/Vector3 v\n";
    // A DRIFTED Vector3 — four fields where the vendored type has three.
    const DRIFTED_VECTOR3: &str = "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";

    let mut ours =
        vec![
            cerulion_core::codegen::parse_rosmsg(ROOT_TEXT, "Wrapper", Some("bi_msgs"))
                .expect("root parses"),
        ];
    ours.extend(
        native_ros2_messages::BUILTIN_MSGS
            .iter()
            .filter(|(p, n, _)| *p == "geometry_msgs" && *n == "Vector3")
            .filter_map(|(p, n, t)| cerulion_core::codegen::parse_rosmsg(t, n, Some(p)).ok()),
    );
    let _ = cerulion_core::codegen::resolve_fixed_nested(&mut ours);
    let root_hash = ours
        .iter()
        .find(|s| s.qualified_name() == ROOT)
        .expect("root indexed")
        .schema_hash();

    let topic = unique_topic("srbi");
    let (netd, sock) = ScriptedNetd::spawn(
        "srbi",
        vec![
            ServedType {
                robot: "go2".to_string(),
                topic: topic.clone(),
                qualified: ROOT.to_string(),
                text: ROOT_TEXT.to_string(),
                deps: vec!["geometry_msgs/Vector3".to_string()],
            },
            ServedType {
                robot: "go2".to_string(),
                topic: format!("{topic}/unused"),
                qualified: "geometry_msgs/Vector3".to_string(),
                text: DRIFTED_VECTOR3.to_string(),
                deps: Vec::new(),
            },
        ],
    );
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    let mgr = make_manager(16);
    let out = unique_out("srbi");
    let ready = unique_out("srbi_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_demand = Duration::from_secs(10);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, root_hash, 3);
    assert!(
        await_condition(Duration::from_secs(8), || netd.schema_calls() >= 1),
        "the recorder never asked the daemon"
    );
    std::thread::sleep(Duration::from_millis(600));
    finish(handle, &shutdown);
    drop(netd);

    let docs = bag_schema_docs(&out);
    // PRECONDITION: the promotion really happened, so the walk really reached
    // the frontier this arm is about.
    let (name, _, _) = descriptor_of(&out, &topic);
    assert_eq!(name, ROOT, "precondition: the served root was named");
    assert!(
        docs.iter().any(|d| d.qualified == ROOT),
        "precondition: the root's own text travelled: {:?}",
        docs.iter().map(|d| &d.qualified).collect::<Vec<_>>()
    );
    // THE ASSERTION.
    assert!(
        !docs.iter().any(|d| d.qualified == "geometry_msgs/Vector3"),
        "a VENDORED name must carry no text in the bag — every reader compiles it, which is \
         why the schema attachment omits them, and shipping a peer's drifted copy under that name is a \
         definition the recording cannot stand behind. docs = {:?}",
        docs.iter().map(|d| &d.qualified).collect::<Vec<_>>()
    );
}

/// A workspace schema the recorder cannot RE-PARSE is still describable.
///
/// `schemas/<name>.yaml` is the documented first-class workspace format — what
/// `cerulion schema create` writes — and `build_schema_docs` ships those docs
/// with a TRUTHFUL recomputed hash. The recorder's own re-parse cannot read
/// them: the YAML parser lives in `cerulion_cli_engine`, which depends on this
/// crate. So the recomputation returns NOTHING for such a name, and reading that
/// absence as a NEGATIVE verdict made a correctly-named channel whose text is
/// right there in the attachment grade `Observability` — escalating
/// `is_incomplete()` and printing "every channel records a bare wire hash, so
/// the bag renders nothing on any machine", false on both halves.
///
/// A/B with ONLY the encoding tag flipped, both arms in one body, because the
/// claim is that the two are indistinguishable to the grade: the recorder
/// checked nothing about the YAML doc, and a grade must not assert what it did
/// not check. The MSG arm is the anti-tautology half — without it, "YAML grades
/// Full" is satisfied by a predicate that grades everything Full.
#[test]
fn a_workspace_schema_the_recorder_cannot_reparse_is_still_describable() {
    const NAME: &str = "yaml_msgs/Thing";
    const MSG_TEXT: &str = "float64 a\nfloat64 b\n";
    // What `cerulion schema create` writes — the recorder has no parser for it.
    const YAML_TEXT: &str =
        "name: Thing\npackage: yaml_msgs\nfields:\n  - a: float64\n  - b: float64\n";

    let mut parsed =
        vec![
            cerulion_core::codegen::parse_rosmsg(MSG_TEXT, "Thing", Some("yaml_msgs"))
                .expect("fixture parses"),
        ];
    let _ = cerulion_core::codegen::resolve_fixed_nested(&mut parsed);
    // ONE truthful hash, shared by both arms — so the ONLY difference between
    // them is the encoding tag on the doc.
    let hash = parsed[0].schema_hash();

    // A `.msg` the recorder CAN parse and which FAILS — the third arm, and the
    // one that keeps the YAML fallback SCOPED. "Not checked" earns the benefit
    // of the doubt; "checked, and the check failed" does not. Without this arm a
    // fallback widened to any-doc-at-all passes every other test in this file.
    const BROKEN_MSG_TEXT: &str = "float64[[ x\n";

    for (label, encoding, text, expect_full) in [
        ("msg", cerulion_core::SchemaEncoding::Msg, MSG_TEXT, true),
        ("yaml", cerulion_core::SchemaEncoding::Yaml, YAML_TEXT, true),
        (
            "broken-msg",
            cerulion_core::SchemaEncoding::Msg,
            BROKEN_MSG_TEXT,
            false,
        ),
    ] {
        let catalog = cerulion_bag::BagSchemaCatalog::new(
            vec![cerulion_core::SchemaDoc {
                qualified: NAME.to_string(),
                encoding,
                text: text.to_string(),
                deps: Vec::new(),
            }],
            vec![cerulion_core::SchemaHashName {
                schema_hash: hash,
                qualified: NAME.to_string(),
            }],
        );

        let mgr = make_manager(16);
        let topic = unique_topic(&format!("sry_{label}"));
        let out = unique_out(&format!("sry_{label}"));
        let ready = unique_out(&format!("sry_{label}_ready"));
        let mut pubr = producer(&mgr, &topic);
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
        cfg.schema_catalog = Some(catalog);
        // The budget is ON, as `graph run --record` has it — which is what makes
        // an Observability grade ESCALATE and print the false verdict.
        cfg.schema_demand = Duration::from_millis(250);
        let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
        assert!(
            wait_for_file(&ready, Duration::from_secs(10)),
            "[{label}] bagd ready"
        );

        publish_n(&mut pubr, hash, 3);
        std::thread::sleep(Duration::from_millis(400));
        finish(handle, &shutdown);

        let (name, _, _) = descriptor_of(&out, &topic);
        assert_eq!(name, NAME, "[{label}] precondition: the channel was named");
        let docs = bag_schema_docs(&out);
        assert!(
            docs.iter().any(|d| d.qualified == NAME),
            "[{label}] precondition: the bag carries the doc"
        );

        let coverage = read_coverage(&out);
        let expected = if expect_full {
            ReplayGrade::Full
        } else {
            ReplayGrade::Observability
        };
        assert_eq!(
            coverage.replay_grade,
            Some(expected),
            "[{label}] a channel named from a doc the bag CARRIES is describable when the \
             recorder could not CHECK it (the YAML arm — 'not checked' is not a negative \
             verdict), and NOT describable when the recorder checked and the check failed \
             (the broken-msg arm — that is evidence, and it is what keeps the fallback \
             scoped to the encodings this crate has no parser for)"
        );
        assert_eq!(
            coverage.is_incomplete(),
            !expect_full,
            "[{label}] the escalation follows the grade: a bag whose text is in the \
             attachment must not be reported as rendering nothing anywhere, and one whose \
             only doc does not parse must not be called replay-grade"
        );
    }
}

/// A closure that is COMPLETE but WRONG is not a describable channel.
///
/// This is the arm that separates "the bag carries a doc under that name" from
/// "the bag's closure reproduces this channel's identity" — and it is the only
/// shape where the two answers differ, which is why it is written rather than
/// assumed. With the deps gate correct, every closure the recorder builds is
/// also self-consistent, so a doc-PRESENCE predicate passes every other arm in
/// this file.
///
/// The reachable shape is a `--schema-catalog` whose BINDING its own doc
/// contradicts: the catalog says hash H is `pkg/Thing` and ships a doc for
/// `pkg/Thing`, but that doc does not hash to H — a stale workspace, a `.msg`
/// edited after the hash was recorded, a hand-built catalog. Its rung
/// takes the name from the binding, so the channel IS named and the doc IS
/// present; only recomputation can tell that a reader would resolve it to a
/// different type than the frames carry.
#[test]
fn a_closure_that_does_not_recompute_the_channels_hash_is_not_replay_grade() {
    const NAME: &str = "stale_msgs/Thing";
    const TEXT: &str = "float64 a\nfloat64 b\n";

    // A binding that LIES: this hash is not what TEXT recomputes to.
    let stale_hash = 0x0981_5747_0981_5747u64;
    let mut parsed = vec![
        cerulion_core::codegen::parse_rosmsg(TEXT, "Thing", Some("stale_msgs")).expect("parses"),
    ];
    let _ = cerulion_core::codegen::resolve_fixed_nested(&mut parsed);
    assert_ne!(
        parsed[0].schema_hash(),
        stale_hash,
        "precondition: the fixture's binding must genuinely disagree with its text"
    );

    let catalog = cerulion_bag::BagSchemaCatalog::new(
        vec![cerulion_core::SchemaDoc {
            qualified: NAME.to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: TEXT.to_string(),
            deps: Vec::new(),
        }],
        vec![cerulion_core::SchemaHashName {
            schema_hash: stale_hash,
            qualified: NAME.to_string(),
        }],
    );

    let mgr = make_manager(16);
    let topic = unique_topic("stale");
    let out = unique_out("stale");
    let ready = unique_out("stale_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_catalog = Some(catalog);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, stale_hash, 3);
    std::thread::sleep(Duration::from_millis(400));
    finish(handle, &shutdown);

    // PRECONDITIONS: the channel really IS named, and its doc really IS in the
    // bag — so a doc-presence predicate would call this describable.
    let (name, _, hash) = descriptor_of(&out, &topic);
    assert_eq!(name, NAME, "the catalog's binding named it");
    assert_eq!(hash, stale_hash, "the wire hash is recorded verbatim");
    let docs = bag_schema_docs(&out);
    assert!(
        docs.iter().any(|d| d.qualified == NAME),
        "precondition: the bag DOES carry a doc under that name — otherwise this arm is \
         indistinguishable from the ordinary missing-text case. docs = {:?}",
        docs.iter().map(|d| &d.qualified).collect::<Vec<_>>()
    );

    // THE ASSERTION: the grade follows the RECOMPUTATION, not the doc list.
    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Observability),
        "the bag's closure resolves this name to a DIFFERENT type than the frames carry, so \
         no reader can describe this channel — a doc under the right name is not the same \
         claim as a closure that reproduces the recorded identity"
    );
}

/// THE VERIFICATION RUNG at the production call site: a peer serving a doc that
/// describes a DIFFERENT type cannot rename the channel.
///
/// The pure `verify_served_schema` oracle pins the VERDICT; this arm pins that
/// the production seam OBEYS it. Without it the `Err` arm of `apply_schema_ladder`'s match
/// could stamp the peer's claim regardless and the whole suite would stay
/// green — the worst class of defect here, since a
/// channel that lies about its own type is worse than one that admits it does
/// not know.
///
/// The wire hash is the peer's own with ONE BIT flipped, so the served text is a
/// real, parseable definition of a real type. It simply is not this wire's.
#[test]
#[serial_test::serial]
fn a_peer_whose_doc_disagrees_with_the_wire_cannot_rename_the_channel() {
    let topic = unique_topic("liar");
    let (peer_hash, _) = peer_identity();
    let wire_hash = peer_hash ^ 0x1;
    let (netd, sock) = ScriptedNetd::spawn(
        "srln",
        vec![ServedType {
            robot: "go2".to_string(),
            topic: topic.clone(),
            qualified: PEER_TYPE.to_string(),
            text: PEER_TEXT.to_string(),
            deps: Vec::new(),
        }],
    );
    let _env = EnvGuard::set(cerulion_netd::SOCKET_ENV, &sock);

    let mgr = make_manager(16);
    let out = unique_out("liar");
    let ready = unique_out("liar_ready");
    let mut pubr = producer(&mgr, &topic);
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = cfg_for(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_demand = Duration::from_secs(10);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, Duration::from_secs(10)), "bagd ready");

    publish_n(&mut pubr, wire_hash, 3);
    assert!(
        await_condition(Duration::from_secs(8), || netd.schema_calls() >= 1),
        "the recorder never asked the daemon for a schema (catalog={}, schema={})",
        netd.catalog_calls(),
        netd.schema_calls()
    );
    std::thread::sleep(Duration::from_millis(600));
    finish(handle, &shutdown);

    assert!(
        netd.catalog_calls() >= 1 && netd.schema_calls() >= 1,
        "precondition: the recorder must have asked AND been answered, or the refusal below \
         is a refusal of nothing (catalog={}, schema={})",
        netd.catalog_calls(),
        netd.schema_calls()
    );
    drop(netd);

    let coverage = read_coverage(&out);
    assert_eq!(
        coverage.tapped[&topic].schema_source,
        Some(SchemaSource::Unresolved),
        "a served doc that does not hash to this topic's wire is REFUSED — the channel keeps \
         a name it can stand behind, because replay trusts the descriptor"
    );
    let (name, size, hash) = descriptor_of(&out, &topic);
    assert_eq!(
        name, "unknown",
        "the refused claim must not reach the descriptor"
    );
    assert_eq!(size, 0, "and nothing was derived from a doc we refused");
    assert_eq!(hash, wire_hash, "the wire hash is still recorded verbatim");

    // Nor does the refused doc's TEXT ride along: shipping a definition under a
    // name the bag never uses is the same unbacked claim in a second field.
    let docs = bag_schema_docs(&out);
    assert!(
        !docs.iter().any(|d| d.qualified == PEER_TYPE),
        "a REFUSED answer must contribute no definition either"
    );
    assert_eq!(
        coverage.replay_grade,
        Some(ReplayGrade::Observability),
        "nothing in this bag is describable"
    );
}
