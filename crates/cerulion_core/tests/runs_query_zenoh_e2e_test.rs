// SPDX-License-Identifier: AGPL-3.0-only
//! The `runs` verb over a REAL zenoh hop — a real serving gateway
//! answering a real desk-side GET, with a real run directory underneath.
//!
//! # Why this file exists, and why it is separate
//!
//! B2 pinned the SERVE DECISION through a sync seam and B3 pinned netd's client
//! against a scripted daemon. Neither drives the one path a robot actually takes:
//! a `RunSource` built at gateway boot, a `runs` GET arriving on the
//! `cerulion_q/{robot}/runs` selector over a TCP hop, a serve-time registry gather,
//! a run DIRECTORY read off the filesystem, and a reply decoded desk-side. That is
//! the composition B3 deferred to B4's lane, and it is where an inert wiring would
//! hide: every piece could be individually correct while the gateway never built
//! the source, or built it on the wrong namespace, and both halves of the existing
//! coverage would stay green.
//!
//! # It is `#[serial]` and env-guarded, and that is the whole reason for a new file
//!
//! Describing a run means reading `$CERULION_HOME/runs/…`, so this file MUTATES
//! PROCESS ENVIRONMENT. `run_dir_root()` is read fresh on every fold, so a
//! concurrent test in the same binary would see this file's `CERULION_HOME` — hence
//! `#[serial]` plus an RAII guard that restores the previous value on drop, panic
//! included.
//!
//! Everything else is hermetic: an isolated per-test SHM root, a scouting-OFF
//! session pair over a probed loopback port, and a run directory under a temp dir
//! this file creates and removes.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::transport::cerulion_q::{format_run_id, RunsCompleteness, RunsGatherOutcome};
use cerulion_core::transport::discovery::query_robot_runs;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};
use cerulion_core::transport::{TransportConfig, TransportManager};
use serial_test::serial;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

fn probe_ephemeral_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = listener.local_addr().expect("probe local_addr").port();
    drop(listener);
    port
}

/// RAII guard for `CERULION_HOME`, restoring whatever was there before.
///
/// Restoring rather than removing matters: a developer or CI job may legitimately
/// have one set, and a guard that unset it would leave the process in a state its
/// owner did not choose. `Drop` runs on a panicking unwind too, so a failing
/// assertion cannot leak the redirect into whatever runs next.
struct CerulionHomeGuard {
    previous: Option<std::ffi::OsString>,
}

impl CerulionHomeGuard {
    fn set(dir: &Path) -> Self {
        let previous = std::env::var_os("CERULION_HOME");
        // SAFETY: `#[serial]` gives this file exclusive use of the process
        // environment for the duration of the test.
        unsafe { std::env::set_var("CERULION_HOME", dir) };
        Self { previous }
    }
}

impl Drop for CerulionHomeGuard {
    fn drop(&mut self) {
        // SAFETY: as above — still inside the `#[serial]` scope.
        unsafe {
            match &self.previous {
                Some(v) => std::env::set_var("CERULION_HOME", v),
                None => std::env::remove_var("CERULION_HOME"),
            }
        }
    }
}

/// A temp `CERULION_HOME` whose `runs/` root is CANONICALISED.
///
/// The canonicalisation is load-bearing on macOS, where `/var` is a symlink to
/// `/private/var`: the serve side canonicalises the root once per fold and requires
/// every run directory to resolve INSIDE it, so a record naming the un-canonical
/// path would be refused as `OutsideRunRoot` — a containment refusal masquerading
/// as a test bug.
fn temp_cerulion_home(tag: &str) -> (PathBuf, PathBuf) {
    let dir = std::env::temp_dir().join(format!("{tag}_{}", unique_id()));
    std::fs::create_dir_all(dir.join("runs")).expect("mk run root");
    let canonical = std::fs::canonicalize(&dir).expect("canonicalize CERULION_HOME");
    (canonical.clone(), canonical.join("runs"))
}

/// Write one run's directory — the two artifacts the serve side reads — and return
/// its path.
fn write_run_dir(runs_root: &Path, run_id: u128, graph_yaml: &str, run_json: &str) -> PathBuf {
    let dir = runs_root.join(format!("perception-{}", format_run_id(run_id)));
    std::fs::create_dir_all(&dir).expect("mk run dir");
    std::fs::write(dir.join("graph.yaml"), graph_yaml).expect("write graph.yaml");
    std::fs::write(dir.join("run.json"), run_json).expect("write run.json");
    dir
}

/// Boot a gateway on an isolated SHM root, listening on a probed loopback port.
///
/// Returns the gateway's manager (so the test can publish a run record on the SAME
/// namespace the gateway's `RunSource` reads), the runtime, and the port.
fn boot_gateway(tag: &str, robot: &str) -> (Arc<TransportManager>, GatewayRuntime, u16) {
    let id = unique_id();
    let root = cerulion_core::testing::iceoryx_test_config();
    for attempt in 0..3 {
        let port = probe_ephemeral_port();
        let g = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("runs_gw_{tag}_{id}_{attempt}"),
                network: Some(NetworkConfig {
                    listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                    robot_identity: Some(robot.to_string()),
                    ..NetworkConfig::default()
                }),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("init gateway manager");
        let manager = Arc::clone(&g);
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![],
            ingress: vec![],
        };
        match GatewayRuntime::new(g, plan) {
            Ok(gateway) => return (manager, gateway, port),
            Err(e) => eprintln!("attempt {attempt}: gateway boot failed (port {port}): {e}"),
        }
    }
    panic!("could not boot a gateway in 3 attempts");
}

/// A desk-side session that reaches the gateway over a real loopback TCP hop, with
/// scouting OFF so the test cannot see anything else on the machine.
fn desk_session(port: u16) -> NetworkManager {
    NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        ..NetworkConfig::default()
    })
}

/// Poll the runs GET until it decodes a reply, or give up.
///
/// Bounded in SECONDS rather than in round trips: what is being waited on is the
/// zenoh session pair establishing, which is a wall a loaded runner can stretch but
/// not invert. A run that is genuinely absent produces a decoded reply with an
/// empty list, not a `NoReply`, so this loop cannot mask the arm's own subject.
fn await_runs_reply(session: &zenoh::Session, robot: &str) -> Option<cerulion_core::RunsReply> {
    let deadline = std::time::Instant::now() + Duration::from_secs(20);
    loop {
        match query_robot_runs(session, robot, Duration::from_millis(400)) {
            RunsGatherOutcome::Decoded(reply) => return Some(reply),
            RunsGatherOutcome::Ignored(reason) => {
                panic!(
                    "the robot answered UNUSABLY — a wire skew between two halves \
                        of one build: {reason}"
                )
            }
            RunsGatherOutcome::NoReply => {
                if std::time::Instant::now() >= deadline {
                    return None;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

/// **The composition B3 deferred**: a REAL desk GET reaches a REAL serving gateway,
/// which gathers its own machine's run registry, reads the run's DIRECTORY off
/// disk, and answers with the effective graph VERBATIM.
///
/// Every piece of this has its own test and none of them touch each other. What is
/// pinned here is that they are WIRED: the gateway really builds a `RunSource` at
/// boot, on the namespace its own manager is on, and the serve callback really
/// reaches the filesystem. An inert wiring — no source built, or one built on the
/// process-global namespace — leaves the B2 seam tests and the B3 client tests
/// entirely green while a robot answers every desk with nothing.
///
/// The oracle is the graph text, byte for byte: the whole design is that
/// the effective `graph.yaml` travels as TEXT and is re-parsed desk-side by the
/// same parser the robot used, so anything that re-serialized or normalized it in
/// transit would be the one failure the wire format exists to prevent.
#[test]
#[serial]
fn a_desk_get_reaches_a_serving_gateway_and_reads_the_runs_own_directory() {
    let robot = "rqgw";
    let (home, runs_root) = temp_cerulion_home("serve");
    let _home_guard = CerulionHomeGuard::set(&home);

    let run_id = 0x0000_0000_0000_0000_0000_0000_0000_0f01u128;
    // A graph whose text is DISTINCTIVE — comments, blank lines and key order that
    // a serde round trip would not reproduce.
    let graph_yaml = "# runs-query B4 acceptance\nname: perception\nprefix: /percep\n\nnodes: []\n";
    let run_json = format!(
        r#"{{"version":1,"run_id":"{}","graph_name":"perception"}}"#,
        format_run_id(run_id)
    );
    let run_dir = write_run_dir(&runs_root, run_id, graph_yaml, &run_json);

    let (manager, _gateway, port) = boot_gateway("serve", robot);
    // The run announces on the GATEWAY's own namespace — the one its `RunSource`
    // was built with.
    let _run = RunHandle::publish_on_config(
        &manager.iox_config(),
        RunRecord {
            run_id,
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "perception".to_string(),
            run_dir: run_dir.display().to_string(),
        },
    )
    .expect("the run announces on the gateway's namespace");

    let desk = desk_session(port);
    let session = desk.session().expect("desk session");
    let reply = await_runs_reply(session, robot)
        .expect("the gateway never answered the runs GET over the wire");

    assert_eq!(reply.robot, robot, "the serve self-attributes");
    assert_eq!(
        reply.runs.len(),
        1,
        "exactly the one live run, described from its own directory: {reply:#?}"
    );
    let served = &reply.runs[0];
    assert_eq!(served.run_id, format_run_id(run_id));
    assert_eq!(served.graph_name, "perception");
    assert_eq!(
        served.graph_yaml, graph_yaml,
        "the effective graph travels VERBATIM — comments, blank line and key order \
         intact. Anything that re-serialized it in transit would defeat the whole \
         reason the wire carries TEXT rather than a parsed topology."
    );
    assert_eq!(served.run_json, run_json);
    assert_eq!(
        reply.completeness,
        RunsCompleteness::Settled,
        "the registry heard from every live writer, so the list is complete"
    );
    assert!(
        reply.undescribable.is_empty(),
        "nothing was withheld: {reply:#?}"
    );
    assert_eq!(reply.error, None, "and nothing was refused");

    let _ = std::fs::remove_dir_all(&home);
}

/// A run whose directory the gateway CANNOT read is NAMED over the wire, and the
/// reply stops being an absence claim.
///
/// The sharp case this closes is a machine whose ONLY run is undescribable: it
/// serves an EMPTY list, which a settled verdict would license as "this robot is
/// running nothing" about a robot that is running something. The demotion is
/// `build_runs_reply`'s job and B2 pins it purely; what is pinned HERE is that the
/// pairing survives encode, the TCP hop and decode — `RunsReply::validate` refuses
/// the un-demoted pairing on the way in, so a serve that got this wrong would not
/// produce a wrong answer, it would produce NO answer, and only an end-to-end arm
/// can tell those apart.
#[test]
#[serial]
fn an_unreadable_run_directory_is_named_over_the_wire_and_demotes_the_verdict() {
    let robot = "rqbad";
    let (home, runs_root) = temp_cerulion_home("bad");
    let _home_guard = CerulionHomeGuard::set(&home);

    let run_id = 0x0000_0000_0000_0000_0000_0000_0000_0f02u128;
    // The directory EXISTS and is inside the root (so it passes containment), but
    // `graph.yaml` is absent — the ordinary shape of a run that exited mid-gather.
    let dir = runs_root.join("perception-gone");
    std::fs::create_dir_all(&dir).expect("mk run dir");
    std::fs::write(dir.join("run.json"), r#"{"version":1}"#).expect("write run.json");

    let (manager, _gateway, port) = boot_gateway("bad", robot);
    let _run = RunHandle::publish_on_config(
        &manager.iox_config(),
        RunRecord {
            run_id,
            supervisor_pid: std::process::id(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "perception".to_string(),
            run_dir: dir.display().to_string(),
        },
    )
    .expect("the run announces");

    let desk = desk_session(port);
    let session = desk.session().expect("desk session");
    let reply = await_runs_reply(session, robot).expect("the gateway answered");

    assert!(
        reply.runs.is_empty(),
        "a run whose graph could not be read is never served as an entry carrying an \
         empty document — that renders desk-side as a graph with no nodes: {reply:#?}"
    );
    assert_eq!(
        reply.undescribable.len(),
        1,
        "…it is NAMED instead, so its absence from the list is explained: {reply:#?}"
    );
    assert_eq!(reply.undescribable[0].run_id, format_run_id(run_id));
    assert!(
        !reply.undescribable[0].reason.is_empty(),
        "with a reason a person can act on"
    );
    assert!(
        !reply.undescribable[0]
            .reason
            .contains(&dir.display().to_string()),
        "…and the reason carries NO PATH: it is a filesystem location on another \
         machine, useless to the desk and a small disclosure of the robot's layout \
         to whoever asked: {:?}",
        reply.undescribable[0].reason
    );
    assert_ne!(
        reply.completeness,
        RunsCompleteness::Settled,
        "a machine whose only run is undescribable serves an EMPTY list, and a \
         settled verdict there is a confident 'nothing is running' about a machine \
         that is running something: {reply:#?}"
    );

    let _ = std::fs::remove_dir_all(&home);
}

/// A machine genuinely running nothing answers a SETTLED empty — the anti-tautology
/// control for both arms above.
///
/// Without it, "the gateway served the run" is satisfied by a serve that answers
/// with whatever it is given and never establishes anything, and "an unreadable
/// directory demotes the verdict" is satisfied by one that never settles at all.
#[test]
#[serial]
fn a_machine_running_nothing_answers_a_settled_empty_over_the_wire() {
    let robot = "rqnone";
    let (home, _runs_root) = temp_cerulion_home("none");
    let _home_guard = CerulionHomeGuard::set(&home);

    let (_manager, _gateway, port) = boot_gateway("none", robot);
    let desk = desk_session(port);
    let session = desk.session().expect("desk session");
    let reply = await_runs_reply(session, robot).expect("the gateway answered");

    assert!(reply.runs.is_empty());
    assert!(reply.undescribable.is_empty());
    assert_eq!(
        reply.completeness,
        RunsCompleteness::Settled,
        "no run writer is live, so the registry's zero-publisher fast path settles \
         the question outright — this is the one empty a desk may believe: {reply:#?}"
    );

    let _ = std::fs::remove_dir_all(&home);
}
