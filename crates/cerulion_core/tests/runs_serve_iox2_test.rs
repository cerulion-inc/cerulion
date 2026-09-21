// SPDX-License-Identifier: AGPL-3.0-only
//! B2: the `runs` verb's SERVE side, over REAL iceoryx2.
//!
//! The verb answers *"which runs are live on this machine right now, and what is
//! each one's graph?"* by GATHERING `/__cerulion/runs` at serve time in whichever
//! process hosts the gateway, then reading each run's directory. This file drives
//! that decision through the production seam
//! ([`GatewayRuntime::runs_serve_for_test`], which delegates to the SAME
//! `runs_serve_decision` the zenoh callback runs — minus the reply send), against
//! REAL registry writers and REAL run directories.
//!
//! # What is real here, and what is not
//!
//! Real: the registry publisher (`RunHandle::publish_on_config` — production's
//! own writer), the gather, the run directory, the authorizer gate, the fold, and
//! the wire encode/decode. Not real: zenoh (the seam builds the reply rather than
//! sending it — the same shape as the `catalog`/`schema` serve seams, which keeps
//! every arm deterministic instead of timing-dependent).
//!
//! # The records and directories are CRAFTED, and where that line sits
//!
//! Each arm hand-builds its [`RunRecord`] and its two artifacts and pushes them
//! through the REAL writer and the REAL gather — a DI double in the sense
//! `vizd_e2e_test` uses for its spy `DemandPlane`, i.e. INPUT to a production
//! seam, not a fabricated measurement (Principle #13). It is crafted for a
//! structural reason rather than a convenient one: `start_run_descriptor` takes
//! no config and can only publish on the PROCESS-GLOBAL namespace, so using it
//! here would delete the isolation every arm's exact-value oracle rests on — and
//! would make the shapes below unreachable, since they are precisely the ones a
//! production writer cannot emit (a directory with no `graph.yaml`, a 256 KiB+
//! artifact, a non-UTF-8 blob, a FIFO, a registry record whose run has already
//! exited).
//!
//! What that GIVES UP is the file NAMES: an arm that writes `graph.yaml` through
//! the same constant B2 reads cannot see that constant move away from what a real
//! `cerulion graph run` writes. That claim is pinned where the production writer
//! is reachable — `cerulion_cli_engine/tests/runs_serve_production_test.rs`,
//! which drives `run_dir::start_run_descriptor` into this same serve and compares
//! against LITERAL filenames.
//!
//! # Absence is the whole subject
//!
//! Three of these arms exist to separate answers that all render as an EMPTY
//! list: an idle machine (`Settled` — a real absence claim), a gather that could
//! not hear a live writer (`Incomplete` — no claim at all), and a refusal
//! (`error` + a non-settled verdict). Getting any of them wrong renders a robot
//! that is running something as a robot that is running nothing, which is the one
//! answer this verb must never give.
//!
//! Per-test SHM roots (`generate_isolated_config`) + per-test temp directories ⇒
//! parallel-safe, no `#[serial]`.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::cerulion_q::{
    decode_runs_reply, encode_runs_reply, format_run_id, RunEntryState, RunsCompleteness, RunsReply,
};
use cerulion_core::transport::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::run_artifacts::{GRAPH_YAML_FILE, RUN_JSON_FILE};
use cerulion_core::transport::run_registry::{
    RunHandle, RunRecord, RunState, RunWatcher, RUN_REGISTRY_MAX_READERS, RUN_REGISTRY_MAX_WRITERS,
    RUN_REGISTRY_QUEUE_DEPTH, RUN_REGISTRY_SERVICE_NAME,
};
use cerulion_core::transport::{TransportConfig, TransportManager};

/// The robot identity every arm queries under. The reply SELF-ATTRIBUTES here.
const ROBOT: &str = "runsrv";

/// Liveness ceiling for the one arm that waits on the registry writer being
/// heard. Generous by orders of magnitude against the writer's 150 ms republish
/// interval — load can DELAY the answer, never invert it.
const HEARD_CEILING: Duration = Duration::from_secs(20);

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// The RUNS ROOT every crafted fixture in this file lives under, with
/// `CERULION_HOME` pointed at it.
///
/// B2 made the serve side REFUSE a `run_dir` outside the
/// canonical `~/.cerulion/runs` — a gathered record's path is attacker-shaped
/// input, and reading anywhere it points turns an authorized `runs` GET into an
/// arbitrary-file-read channel. Bare `tempfile::tempdir()` fixtures are exactly
/// that shape, so they are now (correctly) refused; putting them under a
/// redirected root is strictly MORE faithful, since it is where a real
/// `graph run` writes.
///
/// The `set_var` happens exactly once, inside `get_or_init`, and every arm calls
/// this before it has a fixture — so the single write completes before any read
/// can race it. The `TempDir` is held for the process lifetime (never dropped, so
/// never removed): one directory per test binary run, which the OS reaps.
fn runs_root() -> std::path::PathBuf {
    static ROOT: std::sync::OnceLock<tempfile::TempDir> = std::sync::OnceLock::new();
    let home = ROOT.get_or_init(|| {
        let dir = tempfile::tempdir().expect("runs root");
        std::env::set_var("CERULION_HOME", dir.path());
        std::fs::create_dir_all(dir.path().join("runs")).expect("runs dir");
        dir
    });
    home.path().join("runs")
}

/// A fresh, EXISTING run directory inside the canonical root.
fn run_dir_in_root(tag: &str) -> std::path::PathBuf {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let dir = runs_root().join(format!("{tag}-{}", SEQ.fetch_add(1, Ordering::Relaxed)));
    std::fs::create_dir_all(&dir).expect("run dir");
    dir
}

/// A gateway on its OWN isolated namespace. Returns the namespace too, because
/// every registry writer in a given arm must publish on THAT config — the whole
/// point of the verb is that the gather threads the host's own namespace.
fn gateway_on_isolated_namespace(tag: &str) -> (GatewayRuntime, iceoryx2::config::Config) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("runsrv_{tag}_{id}"),
            network: Some(NetworkConfig {
                robot_identity: Some(ROBOT.to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init gateway manager");
    // A minimal permissive plan: the `runs` source is built regardless of the
    // egress set, and this file never touches the egress plane.
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: Vec::new(),
        ingress: Vec::new(),
    };
    let gateway = GatewayRuntime::new(manager, plan).expect("gateway boot");
    (gateway, root)
}

/// Write a run directory holding both artifacts, and return its path.
fn write_run_dir(dir: &Path, graph_yaml: &str, run_json: &str) {
    std::fs::write(dir.join(GRAPH_YAML_FILE), graph_yaml).expect("write graph.yaml");
    std::fs::write(dir.join(RUN_JSON_FILE), run_json).expect("write run.json");
}

/// A run record pointing at `dir`, with hand-set identity fields so every oracle
/// names an exact value.
fn record_at(run_id: u128, graph_name: &str, started_at_ns: u64, dir: &Path) -> RunRecord {
    RunRecord {
        run_id,
        supervisor_pid: std::process::id(),
        run_started_at_ns: started_at_ns,
        state: RunState::Live,
        graph_name: graph_name.to_string(),
        run_dir: dir.to_string_lossy().into_owned(),
    }
}

/// Serve repeatedly until the gather has HEARD the run(s) — a CONDITION bounded
/// in seconds, never a wall in units of the registry's republish interval.
///
/// Serving is idempotent (a read-only gather + a read-only directory walk), so
/// retrying costs nothing but time. This is the ONLY timing-sensitive helper in
/// the file, and it is one-directional: load delays the answer, and a run that
/// was heard once stays heard.
fn serve_until_runs(gateway: &GatewayRuntime, want: usize) -> RunsReply {
    let deadline = Instant::now() + HEARD_CEILING;
    let mut last = gateway.runs_serve_for_test(ROBOT);
    while last.runs.len() < want && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        last = gateway.runs_serve_for_test(ROBOT);
    }
    assert_eq!(
        last.runs.len(),
        want,
        "the gather never heard {want} run(s) within {HEARD_CEILING:?} — completeness {:?}, \
         error {:?}",
        last.completeness,
        last.error
    );
    last
}

/// A publisher on the run registry's control service that NEVER sends — the
/// "crashed writer whose stale port lingers" shape the gather's own docs name.
///
/// It is what makes the `Incomplete` arm DETERMINISTIC rather than a race: the
/// service reports a live publisher, the gather can never hear from it, and no
/// window length changes that. Built by re-declaring the service with the SAME
/// public parameters the registry itself uses, so a drift in any of them fails
/// this open rather than silently opening a second, incompatible service.
struct SilentRegistryWriter {
    _node: iceoryx2::prelude::Node<iceoryx2::service::ipc_threadsafe::Service>,
    _publisher:
        iceoryx2::port::publisher::Publisher<iceoryx2::service::ipc_threadsafe::Service, [u8], ()>,
}

impl SilentRegistryWriter {
    fn attach(config: &iceoryx2::config::Config) -> Self {
        use iceoryx2::prelude::*;
        let name: NodeName = "silent-writer".try_into().expect("node name");
        let node = NodeBuilder::new()
            .name(&name)
            .config(config)
            .create::<iceoryx2::service::ipc_threadsafe::Service>()
            .expect("silent-writer node");
        let service_name: ServiceName = RUN_REGISTRY_SERVICE_NAME.try_into().expect("service name");
        let factory = node
            .service_builder(&service_name)
            .publish_subscribe::<[u8]>()
            .subscriber_max_buffer_size(RUN_REGISTRY_QUEUE_DEPTH)
            .max_subscribers(RUN_REGISTRY_MAX_READERS)
            .max_publishers(RUN_REGISTRY_MAX_WRITERS)
            .open_or_create()
            .expect("open the run registry control service");
        let publisher = factory
            .publisher_builder()
            .create()
            .expect("silent publisher");
        Self {
            _node: node,
            _publisher: publisher,
        }
    }
}

/// A deny-everything authorizer, asserting the subject the LAN query surface must
/// pass (crib `gateway_iox2_test::DenyAllLan`).
struct DenyAllLan;
impl DemandAuthorizer for DenyAllLan {
    fn authorize_demand(&self, subject: &DemandSubject, resource: &str) -> DemandDecision {
        assert!(
            matches!(subject, DemandSubject::Lan { locator: None }),
            "the LAN query surface must pass a Lan{{locator:None}} subject, got {subject:?}"
        );
        DemandDecision::deny(format!("test: no access to {resource}"))
    }
}

// ---------------------------------------------------------------------------
// The arms.
// ---------------------------------------------------------------------------

/// THE headline: a live run is served with its identity and BOTH documents
/// verbatim, against a hand oracle.
///
/// Every value is hand-chosen so the assertion names it rather than comparing the
/// serve against itself. The documents are deliberately awkward — a trailing
/// newline, an interior blank line, a multi-byte character, a `#` comment — since
/// "verbatim" is the property the whole desk-side re-parse rests on.
#[test]
fn a_live_run_is_served_with_its_identity_and_both_documents_verbatim() {
    let (gateway, ns) = gateway_on_isolated_namespace("headline");
    let dir = run_dir_in_root("a_live_run_is_served_w-dir");
    let yaml = "name: perception\nprefix: /percep\n\nnodes:\n  - id: camera  # \u{00b5}s\n";
    let json = "{\"run_id\":\"0x000000000000000000000000cafebabe\",\"graph\":\"perception\"}\n";
    write_run_dir(dir.as_path(), yaml, json);
    let started = 1_700_000_000_123_456_789_u64;
    let _writer = RunHandle::publish_on_config(
        &ns,
        record_at(0xcafe_babe, "perception", started, dir.as_path()),
    )
    .expect("publish the run record");

    let reply = serve_until_runs(&gateway, 1);

    assert_eq!(reply.robot, ROBOT, "the reply self-attributes");
    assert_eq!(reply.error, None, "an authorized serve is not a refusal");
    assert_eq!(
        reply.completeness,
        RunsCompleteness::Settled,
        "hearing the machine's only live writer settles the question"
    );
    let entry = &reply.runs[0];
    assert_eq!(
        entry.run_id,
        format_run_id(0xcafe_babe),
        "the wire carries the CANONICAL zero-padded text form"
    );
    assert_eq!(entry.graph_name, "perception");
    assert_eq!(entry.run_started_at_ns, started);
    assert_eq!(entry.state, RunEntryState::Live);
    assert_eq!(entry.graph_yaml, yaml, "graph.yaml must be VERBATIM");
    assert_eq!(entry.run_json, json, "run.json must be VERBATIM");
}

/// THE B1 CONTRACT: whatever this serve produces must survive the decode gate.
///
/// `decode_runs_reply` refuses states the serve side is documented as unable to
/// mint — a noncanonical `run_id`, an empty or oversized artifact, one identity
/// twice, a refusal paired with rows or a settled verdict. A served reply that
/// tripped any of them would be a robot the desk cannot read, so the round trip
/// is asserted BOTH directions (encode → decode → equal) on a REAL served answer
/// rather than on a hand-built one.
#[test]
fn a_served_reply_survives_the_wire_gate_it_must_pass_by_construction() {
    let (gateway, ns) = gateway_on_isolated_namespace("roundtrip");
    let dir = run_dir_in_root("a_served_reply_survive-dir");
    write_run_dir(dir.as_path(), "name: g\n", "{\"k\":1}\n");
    let _writer =
        RunHandle::publish_on_config(&ns, record_at(1, "g", 42, dir.as_path())).expect("publish");

    let served = serve_until_runs(&gateway, 1);

    served
        .validate()
        .expect("a served reply must satisfy every invariant the decode side enforces");
    let bytes = encode_runs_reply(&served);
    let decoded = decode_runs_reply(&bytes).expect("a served reply must decode cleanly");
    assert_eq!(decoded, served, "encode → decode is the identity");
}

/// An IDLE machine may claim absence — and that is the one empty answer that is a
/// claim. The registry's zero-publisher fast path makes it exact.
#[test]
fn an_idle_machine_answers_settled_empty_because_absence_is_a_claim_it_can_make() {
    let (gateway, _ns) = gateway_on_isolated_namespace("idle");

    let reply = gateway.runs_serve_for_test(ROBOT);

    assert!(reply.runs.is_empty());
    assert_eq!(
        reply.completeness,
        RunsCompleteness::Settled,
        "a machine with NO registry writer genuinely runs nothing"
    );
    assert_eq!(reply.error, None);
    assert!(
        reply.completeness.is_settled(),
        "the desk may read this empty list as 'nothing is running'"
    );
}

/// THE completeness PASSTHROUGH: a gather that could not hear a live writer must
/// NOT be served as an idle machine.
///
/// This is the arm the empty-list class turns on. A live-but-unheard writer and
/// an idle machine produce the SAME empty `runs` list, and only the verdict tells
/// them apart — so a serve that stamped `Settled` unconditionally, or that
/// re-derived the verdict from `runs.is_empty()`, would tell a desk with
/// confidence that a robot executing a graph is executing nothing.
///
/// Deterministic by construction rather than by timing: the silent writer holds a
/// live publisher port on the registry service and never sends, so no window
/// length can make the gather hear it (the "crashed writer whose stale port
/// lingers" shape `gather_from_node_checked` documents).
#[test]
fn a_gather_that_cannot_hear_a_live_writer_never_claims_the_machine_is_idle() {
    let (gateway, ns) = gateway_on_isolated_namespace("unheard");
    let _silent = SilentRegistryWriter::attach(&ns);

    let reply = gateway.runs_serve_for_test(ROBOT);

    assert!(
        reply.runs.is_empty(),
        "the silent writer contributes no record: {:?}",
        reply.runs
    );
    assert_eq!(reply.error, None, "an unheard writer is not a refusal");
    match reply.completeness {
        RunsCompleteness::Incomplete {
            live_writers,
            writers_heard,
        } => {
            assert!(
                live_writers >= 1,
                "the registry service reports the silent publisher live"
            );
            assert_eq!(writers_heard, 0, "it never answered");
        }
        RunsCompleteness::Settled => panic!(
            "an empty answer with a live-but-unheard writer must NOT be settled — that is a \
             confident 'this robot is running nothing' about a robot that may be running \
             something"
        ),
    }
    assert!(
        !reply.completeness.is_settled(),
        "the desk must render 'could not establish', never 'no runs'"
    );
}

/// A gather that could not RUN AT ALL is the THIRD answer, and it is neither a
/// refusal nor an absence claim.
///
/// A failed gather carries `not_established` and NO `error`, because the `error`
/// field means exactly "the robot refused you" — reporting a fault as a refusal
/// would point an operator at their own credentials. Reporting it as `Settled`
/// would be worse: a gateway that cannot reach its own machine's registry would
/// tell the desk the robot is idle.
///
/// Driven at a REAL production limit rather than by fault injection: the registry
/// service provisions `RUN_REGISTRY_MAX_READERS` subscriber slots, so holding all
/// of them makes the gather's own reader open FAIL. A live writer is present too,
/// since the zero-publisher fast path returns before a reader is ever opened —
/// which is also why this arm cannot be satisfied by an idle machine.
#[test]
fn a_gather_that_could_not_run_at_all_is_neither_a_refusal_nor_an_absence_claim() {
    let (gateway, ns) = gateway_on_isolated_namespace("gatherfail");
    let dir = run_dir_in_root("a_gather_that_could_no-dir");
    write_run_dir(dir.as_path(), "name: unreachable\n", "{\"k\":1}\n");
    let _writer =
        RunHandle::publish_on_config(&ns, record_at(0x77, "unreachable", 1, dir.as_path()))
            .expect("publish");
    // PRECONDITION: with a reader slot free, this machine's run IS servable — so
    // the empty answer below is the exhaustion's doing, not an empty machine's.
    let servable = serve_until_runs(&gateway, 1);
    assert_eq!(servable.runs[0].graph_name, "unreachable");

    // Exhaust every reader slot the registry service provisions.
    let _hogs: Vec<_> = (0..RUN_REGISTRY_MAX_READERS)
        .map(|i| {
            RunWatcher::open_on_config(&ns, 0x77).unwrap_or_else(|e| {
                panic!("reader slot {i} of {RUN_REGISTRY_MAX_READERS} must be claimable: {e}")
            })
        })
        .collect();

    let reply = gateway.runs_serve_for_test(ROBOT);

    assert!(
        reply.runs.is_empty(),
        "a gather that could not run establishes nothing: {:?}",
        reply.runs
    );
    assert_eq!(
        reply.error, None,
        "a FAULT is not a refusal — the `error` field means 'the robot refused you', and \
         reporting a fault there would point an operator at their own credentials"
    );
    // EXACTLY `not_established` — and that exact shape is what tells this arm
    // apart from the unheard-writer one. A gather that RAN with a live publisher
    // present reports `live_writers >= 1` (the precondition above proves one is
    // live), so `live_writers: 0` is reachable only when no gather happened at
    // all. Without this the two failure paths are indistinguishable and the arm
    // would pass on either.
    assert_eq!(
        reply.completeness,
        RunsCompleteness::not_established(),
        "a gateway that cannot reach its own machine's registry must NOT tell the desk the \
         robot is idle, and must report that it measured NOTHING"
    );
    assert!(!reply.completeness.is_settled());
    reply
        .validate()
        .expect("even the failure answer is a well-formed reply");
}

/// An UNAUTHORIZED runs GET is refused EXPLICITLY, and the refusal is sharp: it
/// is taken against a machine that IS running something, so "the denied party
/// learned nothing" cannot be satisfied by an empty machine.
///
/// The anti-tautology half runs in the SAME body on the SAME gateway: restoring
/// the default authorizer serves the run, so the deny was the authorizer's doing.
#[test]
fn an_unauthorized_runs_get_is_refused_explicitly_and_learns_nothing() {
    let (gateway, ns) = gateway_on_isolated_namespace("refusal");
    let dir = run_dir_in_root("an_unauthorized_runs_g-dir");
    write_run_dir(dir.as_path(), "name: secret\n", "{\"k\":1}\n");
    let _writer =
        RunHandle::publish_on_config(&ns, record_at(0x005e_c8e7, "secret", 9, dir.as_path()))
            .expect("publish");
    // PRECONDITION: the run really is servable, so the refusal below withholds
    // something rather than describing an empty machine.
    let allowed = serve_until_runs(&gateway, 1);
    assert_eq!(allowed.runs[0].graph_name, "secret");

    gateway.set_demand_authorizer(Arc::new(DenyAllLan));
    let denied = gateway.runs_serve_for_test(ROBOT);

    assert!(
        denied.error.is_some(),
        "a denied runs GET is an EXPLICIT refusal, never a silent empty reply"
    );
    assert!(
        denied.runs.is_empty(),
        "a denied runs GET serves NOTHING: {:?}",
        denied.runs
    );
    assert!(
        !denied.completeness.is_settled(),
        "a refusal establishes NOTHING — an empty list beside a settled verdict would tell \
         the denied desk the robot is idle"
    );
    denied
        .validate()
        .expect("even a refusal must satisfy the decode gate's exclusions");

    // ANTI-TAUTOLOGY: restore AllowAll → the run is served again.
    gateway.set_demand_authorizer(Arc::new(AllowAllAuthorizer));
    let restored = serve_until_runs(&gateway, 1);
    assert_eq!(restored.runs[0].graph_name, "secret");
    assert!(restored.error.is_none());
}

/// A run whose `graph.yaml` cannot be read is ABSENT from the answer — never an
/// entry carrying an empty document — and the skip costs ONLY that run.
///
/// The sibling run in the same body is what makes it a skip rather than a
/// failure: an error return would have cost the robot every other run it is
/// executing, and on the shipping path a run exiting mid-gather deletes its
/// directory, so this is an ordinary race rather than a corner.
#[test]
fn a_run_whose_directory_cannot_be_read_is_absent_rather_than_served_empty() {
    let (gateway, ns) = gateway_on_isolated_namespace("skip");
    let good = run_dir_in_root("a_run_whose_directory_-good");
    let bad = run_dir_in_root("a_run_whose_directory_-bad");
    write_run_dir(good.as_path(), "name: readable\n", "{\"k\":1}\n");
    // `bad` gets a run.json but NO graph.yaml — the run-directory-vanished shape,
    // narrowed to the one artifact whose absence the desk would render as a graph
    // with no nodes.
    std::fs::write(bad.as_path().join(RUN_JSON_FILE), "{\"k\":2}\n").expect("write");
    let _ok = RunHandle::publish_on_config(&ns, record_at(1, "readable", 10, good.as_path()))
        .expect("publish good");
    let _broken = RunHandle::publish_on_config(&ns, record_at(2, "broken", 20, bad.as_path()))
        .expect("publish bad");

    let reply = serve_until_runs(&gateway, 1);

    assert_eq!(
        reply.runs.len(),
        1,
        "exactly the readable run is served: {:?}",
        reply.runs.iter().map(|r| &r.graph_name).collect::<Vec<_>>()
    );
    let entry = &reply.runs[0];
    assert_eq!(entry.run_id, format_run_id(1));
    assert_eq!(entry.graph_name, "readable");
    assert!(
        !entry.graph_yaml.is_empty() && !entry.run_json.is_empty(),
        "a served entry NEVER carries an empty document"
    );
    assert!(
        reply.runs.iter().all(|r| r.run_id != format_run_id(2)),
        "the unreadable run must be ABSENT, not present-and-empty"
    );

    // ABSENCE ALONE IS NOT THE CONTRACT. A serve that silently
    // DROPPED the run — empty `undescribable`, still `Settled` — satisfies every
    // assertion above while telling the desk it has the whole picture; the desk
    // then renders one fewer run than the robot is executing and has no way to
    // know. So the withheld run must be REPORTED, with its reason, and the
    // verdict must be demoted. Waited on rather than read once: the two runs are
    // heard by the gather independently, so the readable one can arrive first.
    let deadline = Instant::now() + HEARD_CEILING;
    let mut reply = reply;
    while reply.undescribable.is_empty() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        reply = gateway.runs_serve_for_test(ROBOT);
    }
    assert_eq!(
        reply.undescribable.len(),
        1,
        "the withheld run must be REPORTED, not silently dropped — completeness {:?}",
        reply.completeness
    );
    let withheld = &reply.undescribable[0];
    assert_eq!(
        withheld.run_id,
        format_run_id(2),
        "and it must be the run that could not be read"
    );
    assert!(
        withheld.reason.contains(GRAPH_YAML_FILE),
        "with a reason naming the artifact: {}",
        withheld.reason
    );
    assert!(
        !reply.completeness.is_settled(),
        "a STRICT SUBSET is not the whole picture, so the verdict is demoted: {:?}",
        reply.completeness
    );
    // The readable run is STILL served alongside — the skip costs only itself.
    assert!(reply.runs.iter().any(|r| r.run_id == format_run_id(1)));
    reply
        .validate()
        .expect("the surviving answer is still a well-formed reply");
}

/// THE SHARP CASE of the withheld-run class, end to end: a machine whose ONLY
/// live run cannot be described must NOT answer as an idle machine.
///
/// The gather here genuinely SETTLES — one live writer, heard — so every input to
/// the verdict says "this answer is evidence". The run is then withheld by the
/// fold, and what reaches the wire is an EMPTY list. A settled verdict beside it
/// is a confident "this robot is running nothing" about a robot executing a
/// graph, which is the one answer this verb must never give.
///
/// Its CONTROL is `an_idle_machine_answers_settled_empty_…`: the two produce the
/// same empty list from the same settled gather, and only the verdict and the
/// withheld list tell them apart. Without that pairing "not settled" would be
/// satisfied by a serve that never settles at all.
#[test]
fn a_machine_whose_only_run_is_undescribable_never_answers_as_an_idle_machine() {
    let (gateway, ns) = gateway_on_isolated_namespace("undescribable");
    let dir = run_dir_in_root("a_machine_whose_only_r-dir");
    // A run directory with a `run.json` but NO `graph.yaml` — the run-exited-
    // mid-gather shape, narrowed to the artifact whose absence the desk would
    // otherwise render as a graph with no nodes.
    std::fs::write(dir.as_path().join(RUN_JSON_FILE), "{\"k\":1}\n").expect("write");
    let _writer = RunHandle::publish_on_config(&ns, record_at(0xd00d, "ghost", 7, dir.as_path()))
        .expect("publish");

    // PRECONDITION: wait until the gather has HEARD the writer, so the settled
    // verdict under test is a real one rather than a window that never opened.
    let deadline = Instant::now() + HEARD_CEILING;
    let mut reply = gateway.runs_serve_for_test(ROBOT);
    while reply.undescribable.is_empty() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(25));
        reply = gateway.runs_serve_for_test(ROBOT);
    }

    assert_eq!(
        reply.undescribable.len(),
        1,
        "the gather never heard the run within {HEARD_CEILING:?} — completeness {:?}",
        reply.completeness
    );
    assert!(
        reply.runs.is_empty(),
        "an undescribable run is NEVER served"
    );
    assert!(
        !reply.completeness.is_settled(),
        "an empty list whose only run was WITHHELD must not read as 'running nothing'"
    );
    assert_eq!(reply.error, None, "a withheld run is not a refusal");
    let withheld = &reply.undescribable[0];
    assert_eq!(
        withheld.run_id,
        format_run_id(0xd00d),
        "the withheld run is as identifiable as a served one"
    );
    assert!(
        withheld.reason.contains(GRAPH_YAML_FILE),
        "the reason names the artifact: {}",
        withheld.reason
    );
    assert!(
        !withheld
            .reason
            .contains(&dir.as_path().to_string_lossy().to_string()),
        "the robot's filesystem layout stays in its own log: {}",
        withheld.reason
    );
    reply
        .validate()
        .expect("a demoted reply must still satisfy the decode gate");
    let decoded =
        decode_runs_reply(&encode_runs_reply(&reply)).expect("and must survive the round trip");
    assert_eq!(decoded, reply);
}

/// TWO live runs are served as TWO entries — one per identity, in the promised
/// `(run_started_at_ns, run_id)` order.
///
/// The registry's gather is keyed by `run_id` through a `BTreeMap`, so one entry
/// per identity holds by construction rather than by a second dedup on the serve
/// side; this pins that the serve does not lose or duplicate a run on the way
/// through, and that the ORDER a desk renders is the one `build_runs_reply`
/// promises.
#[test]
fn two_concurrent_runs_are_served_one_entry_each_in_start_order() {
    let (gateway, ns) = gateway_on_isolated_namespace("two");
    let later = run_dir_in_root("two_concurrent_runs_ar-later");
    let earlier = run_dir_in_root("two_concurrent_runs_ar-earlier");
    write_run_dir(later.as_path(), "name: later\n", "{\"k\":1}\n");
    write_run_dir(earlier.as_path(), "name: earlier\n", "{\"k\":2}\n");
    // Published LATER-first, so a serve that echoed arrival order would fail.
    let _b = RunHandle::publish_on_config(&ns, record_at(0xbb, "later", 2_000, later.as_path()))
        .expect("publish later");
    let _a =
        RunHandle::publish_on_config(&ns, record_at(0xaa, "earlier", 1_000, earlier.as_path()))
            .expect("publish earlier");

    let reply = serve_until_runs(&gateway, 2);

    let names: Vec<&str> = reply.runs.iter().map(|r| r.graph_name.as_str()).collect();
    assert_eq!(
        names,
        vec!["earlier", "later"],
        "runs are served in start-time order"
    );
    assert_eq!(
        reply.runs[0].run_id,
        format_run_id(0xaa),
        "and each identity appears exactly once"
    );
    assert_eq!(reply.runs[1].run_id, format_run_id(0xbb));
    reply.validate().expect("two entries, two identities");
}

/// The gather runs on the GATEWAY HOST's own iceoryx2 namespace, never the
/// process-global one.
///
/// The registry is keyed by namespace, so a gather threading the wrong one
/// answers a confident empty about a machine that is running something — and the
/// two are INDISTINGUISHABLE from the reply alone, because a gather on the wrong
/// namespace and a gather on the right one with nothing published both return an
/// empty list. That is why the namespace itself is the observable this arm reads,
/// and why the behavioural half runs in the same body: this gateway serves a run
/// published on ITS namespace, which a global-config gather could not see.
#[test]
fn the_runs_gather_threads_the_gateway_hosts_own_namespace() {
    let (gateway, ns) = gateway_on_isolated_namespace("namespace");
    let dir = run_dir_in_root("the_runs_gather_thread-dir");
    write_run_dir(dir.as_path(), "name: isolated\n", "{\"k\":1}\n");
    let _writer = RunHandle::publish_on_config(&ns, record_at(0xf0, "isolated", 1, dir.as_path()))
        .expect("publish on the ISOLATED namespace");

    let gather_ns = gateway.runs_gather_namespace_for_test();
    let global = iceoryx2::config::Config::global_config();
    assert_eq!(
        gather_ns.global.prefix, ns.global.prefix,
        "the gather namespace is the one this gateway's manager was built on"
    );
    assert_ne!(
        gather_ns.global.prefix, global.global.prefix,
        "…and is NOT the process-global namespace — the arm that discriminates the two \
         (a host on a non-global namespace, i.e. the per-run gateway child \
         after a real mismatch)"
    );

    // The behavioural half: a run published on the isolated namespace IS served.
    let reply = serve_until_runs(&gateway, 1);
    assert_eq!(reply.runs[0].graph_name, "isolated");
}

/// A STRICT ingress-only gateway declares NO query surface — and `runs` is not
/// singled out, it goes with `demand`, `catalog` and `schema`.
///
/// Reported as a finding ("the runs verb is unreachable for ingress-only
/// gateways"), and the mechanism is real: `GatewayRuntime::new` starts the query
/// surface only for a non-empty egress set or an `AllowAll` posture. But the gate
/// wraps ONE `start_query_surface` call that declares ONE queryable serving ALL
/// FOUR verbs, so it is a PRE-EXISTING property of the surface, not
/// something this verb introduced — and `NetworkManager::has_query_surface`'s own
/// doc has recorded it as intended since then: *"an empty deny-all/ingress-only
/// gateway leaves it `false`"*.
///
/// So B2 MATCHES the surface rather than widening it. Widening would make a
/// strict-posture robot start answering `catalog` and `schema` GETs it has never
/// answered — a posture change (`Strict` is documented as verbatim locators, an
/// egress allow-list and declared ingress) that a serve-side chunk has no
/// business making unilaterally. The gap is narrow but real: a graph that
/// declares `ingress:` and no egress under an explicit `network:` block IS
/// running something and cannot be asked about it. It is B3's to decide, and this
/// arm exists so the decision is made against a checked fact.
///
/// The permissive CONTROL is in the same body: without it, "no surface" is
/// satisfied by a gateway that never starts one under any posture.
#[test]
fn a_strict_ingress_only_gateway_declares_no_query_surface_at_all() {
    let id = unique_id();
    let ingress_topic = format!("/runsrv/ingressonly/{id}");

    let strict = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("runsrv_strict_{id}"),
            network: Some(NetworkConfig {
                robot_identity: Some(ROBOT.to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init strict gateway manager");
    let strict_gateway = GatewayRuntime::new(
        strict,
        GatewayPlan {
            // Deny-all egress + one declared ingress = ingress-only under Strict.
            egress_policy: GatewayEgressPolicy::AllowList(Vec::new()),
            announce: Vec::new(),
            ingress: vec![cerulion_core::transport::gateway::GatewayIngressEntry {
                topic: ingress_topic,
                schema_hash: 0xABCD,
            }],
        },
    )
    .expect("strict ingress-only gateway boots");

    assert!(
        !strict_gateway.query_surface_active(),
        "a strict ingress-only gateway declares NO queryable — so demand, catalog, schema \
         AND runs are all unreachable on it, which is the existing query-surface shape rather than \
         anything the runs verb did"
    );

    // CONTROL: the permissive posture — every real-clock `graph run` — starts it.
    let (permissive_gateway, _ns) = gateway_on_isolated_namespace("permissive");
    assert!(
        permissive_gateway.query_surface_active(),
        "the permissive posture MUST declare the surface, or the assertion above is \
         satisfied by a gateway that never starts one"
    );
}

// ---------------------------------------------------------------------------
// The dispatch arm — STRUCTURAL, because nothing in B2 drives the real zenoh
// path.
// ---------------------------------------------------------------------------

/// Strip Rust comments and string/char literals from `src`, leaving the CODE.
///
/// Load-bearing rather than tidiness: `handle_query`'s neighbourhood explains the
/// verb in prose (and B1 left a `debug!` no-op there whose text NAMED the serve
/// side), so a raw-text search would be satisfied by a comment. Block comments
/// NEST in Rust, so the depth is tracked. Fails CLOSED — an unterminated block
/// swallows the tail, which can only make an assertion FAIL.
fn code_only(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth) = (0usize, 0usize);
    while i < b.len() {
        if depth > 0 {
            if b[i..].starts_with(b"/*") {
                depth += 1;
                i += 2;
            } else if b[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
            } else {
                i += 1;
            }
            continue;
        }
        if b[i..].starts_with(b"//") {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if b[i..].starts_with(b"/*") {
            depth = 1;
            i += 2;
        } else if b[i] == b'"' {
            i += 1;
            while i < b.len() && b[i] != b'"' {
                i += if b[i] == b'\\' { 2 } else { 1 };
            }
            i += 1;
        } else {
            out.push(b[i] as char);
            i += 1;
        }
    }
    out
}

/// The BODY of the match arm whose pattern is `pattern` — the arm alone, not the
/// match, and not "the rest of the function".
///
/// Scoping is the whole point: a `contains` over the enclosing function is
/// satisfied by an arm that is a bare `debug!` no-op beside a `handle_runs_verb`
/// mentioned anywhere else in the body, which is exactly the shape the dispatch
/// pin exists to refuse. Handles BOTH arm forms — a braced block (brace-matched)
/// and a bare expression (scanned to its terminating comma at nesting depth 0, so
/// commas inside the call's own argument list do not end it early).
fn match_arm_body(code: &str, pattern: &str) -> String {
    let at = code
        .find(pattern)
        .unwrap_or_else(|| panic!("the match arm `{pattern}` must exist in the searched source"));
    let arrow = at
        + code[at..]
            .find("=>")
            .unwrap_or_else(|| panic!("`{pattern}` must be a match arm (no `=>` found)"));
    let bytes = code.as_bytes();
    let mut i = arrow + 2;
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    if bytes[i] == b'{' {
        let (open, mut depth) = (i, 0usize);
        while i < bytes.len() {
            match bytes[i] {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return code[open..=i].to_string();
                    }
                }
                _ => {}
            }
            i += 1;
        }
        panic!("unbalanced braces in the `{pattern}` arm");
    }
    let (start, mut depth) = (i, 0usize);
    while i < bytes.len() {
        match bytes[i] {
            b'(' | b'[' | b'{' => depth += 1,
            b')' | b']' | b'}' => {
                // A closing delimiter at depth 0 ends the MATCH itself — an arm
                // may legitimately be the last one, with no trailing comma.
                if depth == 0 {
                    return code[start..i].to_string();
                }
                depth -= 1;
            }
            b',' if depth == 0 => return code[start..i].to_string(),
            _ => {}
        }
        i += 1;
    }
    panic!("the `{pattern}` arm never terminates");
}

/// Whether `expr` IS a call to `name` — not merely an expression that mentions
/// it.
///
/// `contains` cannot tell a CALL from a MENTION, and the difference is the whole
/// pin: an inert arm carrying `let _ = (runs, handle_runs_verb);` beside a
/// `debug!` no-op satisfies a containment check while serving nothing. (That
/// shape was RUN against the containment version of this guard and PASSED, which
/// is why this predicate exists.)
///
/// Everything before the `(` must be a plain path, and nothing but whitespace may
/// follow the matched `)` — so a block that happens to call it, or a call used as
/// one term of a larger expression, is refused. That is deliberate: the arm's
/// value must BE the delegation.
fn is_call_to(expr: &str, name: &str) -> bool {
    let expr = expr.trim();
    let Some(open) = expr.find('(') else {
        return false;
    };
    if expr[..open].trim() != name {
        return false;
    }
    let bytes = expr.as_bytes();
    let (mut i, mut depth) = (open, 0usize);
    while i < bytes.len() {
        match bytes[i] {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return expr[i + 1..].trim().is_empty();
                }
            }
            _ => {}
        }
        i += 1;
    }
    false
}

/// The body of `fn <name>`, brace-matched — never "everything after the
/// signature", which a later item would silently join.
fn fn_body(code: &str, name: &str) -> String {
    let at = code
        .find(&format!("fn {name}"))
        .unwrap_or_else(|| panic!("`fn {name}` must exist in the searched source"));
    let open = at + code[at..].find('{').expect("a function has a body");
    let bytes = code.as_bytes();
    let (mut i, mut depth) = (open, 0usize);
    while i < bytes.len() {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return code[open..=i].to_string();
                }
            }
            _ => {}
        }
        i += 1;
    }
    panic!("unbalanced braces in `fn {name}`");
}

/// The structural predicates answer HAND vectors, including the shapes that
/// defeated their earlier versions.
///
/// A predicate a guard asserts THROUGH is not covered by that guard: one that
/// answers `true` too easily makes its assertion vacuous without failing
/// anything, which is exactly how the containment version of the dispatch pin
/// shipped past an inert arm.
#[test]
fn the_structural_predicates_answer_their_hand_written_vectors() {
    // `is_call_to` — the call is the WHOLE expression.
    assert!(is_call_to(
        "handle_runs_verb(query, robot, runs, auth)",
        "handle_runs_verb"
    ));
    assert!(is_call_to("  handle_runs_verb(a)  ", "handle_runs_verb"));
    // …a MENTION is not a call (the shape that defeated `contains`).
    assert!(!is_call_to(
        "{ let _ = (runs, handle_runs_verb); tracing::debug!(\"ignored\"); }",
        "handle_runs_verb"
    ));
    // …nor is a call used as one term of a larger expression, or a different fn.
    assert!(!is_call_to(
        "if x { handle_runs_verb(a) } else { () }",
        "handle_runs_verb"
    ));
    assert!(!is_call_to("handle_runs_verb(a).ok()", "handle_runs_verb"));
    assert!(!is_call_to("handle_catalog_verb(a)", "handle_runs_verb"));
    assert!(!is_call_to("handle_runs_verb_extra(a)", "handle_runs_verb"));
    assert!(!is_call_to("handle_runs_verb", "handle_runs_verb"));
    // …and a nested paren does not end the call early.
    assert!(is_call_to(
        "handle_runs_verb(f(a, b), c)",
        "handle_runs_verb"
    ));

    // `match_arm_body` — both arm forms, scoped to ONE arm.
    let src = "match v {\n    A => one(x, y),\n    B => { two(z) }\n    C => three(),\n}";
    assert_eq!(match_arm_body(src, "A =>").trim(), "one(x, y)");
    assert_eq!(match_arm_body(src, "B =>").trim(), "{ two(z) }");
    // The LAST arm may carry no trailing comma — it ends at the match's brace.
    assert_eq!(match_arm_body(src, "C =>").trim(), "three()");
    // An arm's own argument commas must not end it early.
    let commas = "match v {\n    A => one(x, y, z),\n    B => two(),\n}";
    assert_eq!(match_arm_body(commas, "A =>").trim(), "one(x, y, z)");
}

/// The `runs` verb is really WIRED into the query surface's dispatch.
///
/// Every other arm in this file drives `runs_serve_for_test`, which delegates to
/// the same decision the callback runs — but BYPASSES `handle_query`. So the
/// dispatch arm itself is reachable by no behavioural assertion here, and B2
/// ships no consumer that drives the real zenoh path (that is B3's netd
/// `query_runs`). Reverting the arm to the `debug!` no-op B1 left there would
/// leave the whole verb INERT on the wire with all eleven arms green — the
/// no-inert-shipping class, closed the way this repo closes it when behaviour
/// cannot see the seam.
///
/// Its ANTI-TAUTOLOGY half is in the same body: the stripped view must still
/// contain the sibling verbs' dispatch, so a broken stripper cannot make the
/// assertion vacuous.
#[test]
fn the_runs_verb_is_wired_into_the_query_surfaces_dispatch() {
    let src = include_str!("../src/transport/network.rs");
    let code = code_only(src);

    // ANTI-TAUTOLOGY: the stripper still yields real code.
    assert!(
        code.contains("QueryVerb::Catalog") && code.contains("QueryVerb::Schema"),
        "the comment-stripped view must still hold the sibling verbs' dispatch — \
         otherwise every assertion below is vacuous"
    );

    let dispatch = fn_body(&code, "handle_query");
    assert!(
        dispatch.contains("QueryVerb::Runs"),
        "`handle_query` must route the runs verb"
    );
    // SCOPED TO THE ARM, not to the function. A `contains` over the whole
    // dispatch is satisfied by a `debug!` no-op arm beside any other mention of
    // the handler — which is precisely the inert shape this pin exists to refuse.
    let runs_arm = match_arm_body(&dispatch, "Some(QueryVerb::Runs)");
    assert!(
        is_call_to(&runs_arm, "handle_runs_verb"),
        "the `QueryVerb::Runs` ARM ITSELF must BE a call to the serve handler — a `debug!` \
         no-op there ships the whole verb inert on the wire while every behavioural arm in \
         this file stays green (they drive the seam, which bypasses this dispatch), and a \
         `contains` is satisfied by an inert arm that merely NAMES the handler. Arm: \
         {runs_arm}"
    );
    // ANTI-TAUTOLOGY for the extractor: a SIBLING arm must not carry the runs
    // handler. If `match_arm_body` silently returned the whole match, the
    // assertion above would hold no matter where the call sat.
    let catalog_arm = match_arm_body(&dispatch, "Some(QueryVerb::Catalog)");
    assert!(
        !catalog_arm.contains("handle_runs_verb"),
        "the extractor must scope to ONE arm — the catalog arm came back carrying the runs \
         handler, so the assertion above proves nothing. Arm: {catalog_arm}"
    );
    assert!(
        catalog_arm.contains("handle_catalog_verb"),
        "…and it must still return a REAL arm body rather than an empty string. Arm: \
         {catalog_arm}"
    );

    // …and the handler really serves: it consults the authorizer-gated decision
    // and replies on the verb's own concrete key.
    let handler = fn_body(&code, "handle_runs_verb");
    assert!(
        handler.contains("runs_serve_decision"),
        "the handler must run the SAME gated decision the test seam runs"
    );
    assert!(
        handler.contains("encode_runs_reply") && handler.contains("runs_selector"),
        "…and reply the encoded payload on `cerulion_q/{{robot}}/runs`"
    );
}

/// Two serves of one unchanged machine produce EQUAL replies (Principle #7).
///
/// The serve reads a `BTreeMap`-backed gather and a directory listing, so nothing
/// in it may leak an iteration accident or a wall-clock value onto the wire — a
/// desk caching per `run_id` compares these documents.
#[test]
fn two_serves_of_one_unchanged_machine_are_byte_identical() {
    let (gateway, ns) = gateway_on_isolated_namespace("determinism");
    let a = run_dir_in_root("two_serves_of_one_unch-a");
    let b = run_dir_in_root("two_serves_of_one_unch-b");
    write_run_dir(a.as_path(), "name: a\n", "{\"k\":1}\n");
    write_run_dir(b.as_path(), "name: b\n", "{\"k\":2}\n");
    let _wa = RunHandle::publish_on_config(&ns, record_at(0x1, "a", 100, a.as_path())).expect("a");
    let _wb = RunHandle::publish_on_config(&ns, record_at(0x2, "b", 200, b.as_path())).expect("b");

    let first = serve_until_runs(&gateway, 2);
    let second = gateway.runs_serve_for_test(ROBOT);

    assert_eq!(
        first, second,
        "the serve is a function of the machine state"
    );
    assert_eq!(
        encode_runs_reply(&first),
        encode_runs_reply(&second),
        "…and so are its wire bytes"
    );
}

/// A run that ends GRACEFULLY reaches the desk as `ending`, not `live`.
///
/// `Ending` is the registry's LAST WORD, and a desk rendering a
/// shutting-down run as healthy is the same class of error as rendering an
/// unheard machine as idle — a confident claim the evidence does not support.
#[test]
fn a_run_that_announced_its_ending_is_served_as_ending() {
    let (gateway, ns) = gateway_on_isolated_namespace("ending");
    let dir = run_dir_in_root("a_run_that_announced_i-dir");
    write_run_dir(dir.as_path(), "name: winding-down\n", "{\"k\":1}\n");
    let writer =
        RunHandle::publish_on_config(&ns, record_at(0x9, "winding-down", 5, dir.as_path()))
            .expect("publish");
    // PRECONDITION: it is served as LIVE first, so the flip below is observable
    // rather than a value that was always `ending`.
    let live = serve_until_runs(&gateway, 1);
    assert_eq!(live.runs[0].state, RunEntryState::Live);

    assert!(writer.set_ending(), "the state really flipped");

    let deadline = Instant::now() + HEARD_CEILING;
    let mut reply = gateway.runs_serve_for_test(ROBOT);
    while reply
        .runs
        .first()
        .is_none_or(|r| r.state != RunEntryState::Ending)
        && Instant::now() < deadline
    {
        std::thread::sleep(Duration::from_millis(25));
        reply = gateway.runs_serve_for_test(ROBOT);
    }
    assert_eq!(
        reply.runs.first().map(|r| r.state),
        Some(RunEntryState::Ending),
        "the registry's last word must reach the wire: {reply:?}"
    );
}
