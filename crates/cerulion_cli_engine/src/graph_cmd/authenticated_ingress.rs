// SPDX-License-Identifier: AGPL-3.0-only
//! Canonical graph demands. Observer reads use their separate private channels.

use std::sync::{Condvar, Mutex};
use std::thread::JoinHandle;

use cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::{
    EgressReliability, GatewayIngressEntry, GatewayPlan, LanAccess, SchemaServing,
};
use cerulion_netd::account_access::RobotCatalog;
use cerulion_netd::protocol::ServingPlane;
use cerulion_netd::NetdClient;

use super::*;
use crate::account_robot_access::automatic::{self, RemoteCatalogs};

#[derive(Clone, Debug, PartialEq, Eq)]
struct Producer {
    topic: String,
    route: String,
    key: [u8; 32],
    schema_hash: u64,
}

/// A declaration selects exactly one live authorized writer, never a matching
/// hash among several writers. Unknown metadata cannot authorize a graph input.
fn select_producer(input: &GatewayIngressEntry, robots: &[RobotCatalog]) -> CliResult<Producer> {
    let mut candidates = Vec::new();
    for robot in robots {
        let Some(catalog) = &robot.catalog else {
            continue;
        };
        if catalog.error.is_some() {
            continue;
        }
        let entries: Vec<_> = catalog
            .entries
            .iter()
            .filter(|entry| entry.topic == input.topic && entry.producer_count != Some(0))
            .collect();
        if entries.is_empty() {
            continue;
        }
        let key = robot
            .endpoint_key
            .filter(|_| {
                catalog.version == CATALOG_WIRE_VERSION
                    && catalog.wire_header_matches()
                    && catalog.robot == robot.route
            })
            .ok_or_else(|| {
                CliError::Validation(format!(
                    "graph ingress '{}': producer identity or catalog version is unverified",
                    input.topic
                ))
            })?;
        if entries.len() != 1 || entries[0].producer_count != Some(1) {
            return Err(CliError::Validation(format!(
                "graph ingress '{}': requires exactly one verified remote writer",
                input.topic
            )));
        }
        candidates.push((robot, key, entries[0]));
    }
    if candidates.len() != 1 {
        return Err(CliError::Validation(format!("graph ingress '{}': {} authorized producers found; start exactly one declared egress producer", input.topic, candidates.len())));
    }
    let (robot, key, entry) = candidates[0];
    if entry.schema_hash != Some(input.schema_hash) {
        return Err(CliError::Validation(format!(
            "graph ingress '{}': producer schema does not match the consuming graph",
            input.topic
        )));
    }
    let policy = entry.egress_qos.as_ref().ok_or_else(|| {
        CliError::Validation(format!(
            "graph ingress '{}': producer queue policy is unknown; upgrade and restart its graph",
            input.topic
        ))
    })?;
    if policy.topic != input.topic || policy.reliability != EgressReliability::Reliable {
        return Err(CliError::Validation(format!("graph ingress '{}': authenticated wire ingress requires a declared reliable producer policy", input.topic)));
    }
    Ok(Producer {
        topic: input.topic.clone(),
        route: robot.route.clone(),
        key,
        schema_hash: input.schema_hash,
    })
}

#[derive(Clone)]
struct Registration {
    plan: GatewayPlan,
    schemas: SchemaServing,
    namespace: String,
    graph: String,
}

struct Session {
    // Closing this connection releases registration and every canonical demand.
    client: NetdClient,
    _catalogs: RemoteCatalogs,
    producers: Vec<Producer>,
}

fn open_session(
    registration: &Registration,
    expected: Option<&[Producer]>,
    cancelled: &impl Fn() -> bool,
) -> CliResult<Session> {
    check_cancelled(cancelled)?;
    let mut client = NetdClient::connect_or_spawn().map_err(network_error)?;
    if client.lan_access().map_err(network_error)? != Some(LanAccess::Paired) {
        return Err(CliError::Validation("authenticated graph networking requires a paired cerulion-netd; restart the daemon with paired access".into()));
    }
    check_cancelled(cancelled)?;
    let snapshot = client
        .query_serving_schema_until(Instant::now() + Duration::from_secs(5))
        .map_err(network_error)?;
    let expected_namespace =
        cerulion_core::ix_config_shm_identity_from_json(&registration.namespace)?;
    let actual = cerulion_core::ix_config_shm_identity_from_json(&snapshot.ix_config_json)?;
    if expected_namespace != actual {
        return Err(CliError::Validation("authenticated graph networking and cerulion-netd must use the same resolved shared-memory namespace; restart the daemon after changing its config".into()));
    }
    check_cancelled(cancelled)?;
    if !registration.plan.announce.is_empty() {
        cerulion_netd::serving_login::require_serving_authority()
            .map_err(|error| CliError::Validation(error.into()))?;
        let mut egress = registration.plan.clone();
        egress.ingress.clear(); // Only canonical authenticated demands admit ingress.
        let reply = client
            .register_egress(
                &egress,
                &registration.schemas,
                Some(&registration.namespace),
            )
            .map_err(network_error)?;
        if reply.lan_access != Some(LanAccess::Paired) {
            return Err(CliError::Validation(
                "graph egress registration did not confirm paired access".into(),
            ));
        }
    }
    let catalogs = if registration.plan.ingress.is_empty() {
        RemoteCatalogs::default()
    } else {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            check_cancelled(cancelled)?;
            let catalogs = automatic::discover()?;
            if catalogs.robots.is_empty()
                || catalogs.robots.iter().any(|robot| {
                    robot
                        .catalog
                        .as_ref()
                        .is_some_and(|catalog| catalog.error.is_none())
                })
                || Instant::now() >= deadline
            {
                break catalogs;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    };
    let producers = registration
        .plan
        .ingress
        .iter()
        .map(|input| select_producer(input, &catalogs.robots))
        .collect::<CliResult<Vec<_>>>()
        .map_err(|error| {
            if catalogs.diagnostics.is_empty() {
                error
            } else {
                CliError::Validation(format!(
                    "{error}; discovery: {}",
                    catalogs.diagnostics.join("; ")
                ))
            }
        })?;
    require_same_producers(expected, &producers)?;
    for producer in &producers {
        check_cancelled(cancelled)?;
        catalogs.sync_selected_epoch(&producer.route)?;
        let deadline = Instant::now() + Duration::from_secs(10);
        let reply = loop {
            check_cancelled(cancelled)?;
            match client.demand(&producer.route, &producer.topic, producer.schema_hash) {
                Ok(reply) => break reply,
                Err(cerulion_netd::ClientError::Netd { .. }) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(100))
                }
                Err(error) => return Err(network_error(error)),
            }
        };
        if reply.robot != producer.route || reply.topic != producer.topic || reply.refcount == 0 {
            return Err(CliError::Validation(
                "graph ingress demand returned an invalid canonical acknowledgment".into(),
            ));
        }
    }
    tracing::info!(graph = %registration.graph, ingress_topics = producers.len(), egress_topics = registration.plan.announce.len(), "authenticated graph network demands are held");
    Ok(Session {
        client,
        _catalogs: catalogs,
        producers,
    })
}

fn check_cancelled(cancelled: &impl Fn() -> bool) -> CliResult<()> {
    if cancelled() {
        return Err(CliError::Validation(
            "authenticated graph network setup was interrupted".into(),
        ));
    }
    Ok(())
}

fn require_same_producers(expected: Option<&[Producer]>, selected: &[Producer]) -> CliResult<()> {
    if expected.is_some_and(|expected| expected != selected) {
        return Err(CliError::GraphIngressSourceChanged);
    }
    Ok(())
}

fn network_error(error: cerulion_netd::ClientError) -> CliError {
    CliError::Validation(format!("authenticated graph networking: {error}"))
}

struct Control {
    stopped: Mutex<bool>,
    wake: Condvar,
    fatal: Mutex<Option<String>>,
}

pub(super) struct PairedGraphNetwork {
    control: Arc<Control>,
    worker: Option<JoinHandle<()>>,
}

impl PairedGraphNetwork {
    /// Called only after canonical services have their full topology capacity.
    pub(super) fn start(
        launch: &GatewayLaunch,
        namespace: Option<&str>,
        graph: &str,
        running: Arc<AtomicBool>,
    ) -> CliResult<Self> {
        let registration = Registration {
            plan: launch.plan.clone(),
            schemas: launch.schema_serving.clone(),
            graph: graph.into(),
            namespace: match namespace {
                Some(namespace) => namespace.into(),
                None => serde_json::to_string(iceoryx2::config::Config::global_config()).map_err(
                    |error| {
                        CliError::Validation(format!(
                            "graph network namespace cannot be encoded: {error}"
                        ))
                    },
                )?,
            },
        };
        // Unsatisfied declarations fail startup; no warning-only fallback.
        let session = open_session(&registration, None, &|| !running.load(Ordering::Acquire))?;
        let control = Arc::new(Control {
            stopped: Mutex::new(false),
            wake: Condvar::new(),
            fatal: Mutex::new(None),
        });
        let worker_control = control.clone();
        let worker = std::thread::Builder::new()
            .name("graph-network".into())
            .spawn(move || {
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    watch_session(session, &registration, &worker_control, &running)
                }));
                if outcome.is_err() {
                    fail(
                        &worker_control,
                        &running,
                        "authenticated graph network worker panicked".into(),
                    );
                }
            })?;
        Ok(Self {
            control,
            worker: Some(worker),
        })
    }

    pub(super) fn check(&self) -> CliResult<()> {
        let fatal =
            self.control.fatal.lock().map_err(|_| {
                CliError::Validation("graph network failure state is poisoned".into())
            })?;
        match fatal.as_ref() {
            Some(error) => Err(CliError::Validation(error.clone())),
            None => Ok(()),
        }
    }
}

impl Drop for PairedGraphNetwork {
    fn drop(&mut self) {
        if let Ok(mut stopped) = self.control.stopped.lock() {
            *stopped = true;
        }
        self.control.wake.notify_all();
        if let Some(worker) = self.worker.take() {
            let _joined = worker.join();
        }
    }
}

fn fail(control: &Control, running: &AtomicBool, error: String) {
    tracing::error!(error = %error, "authenticated graph networking stopped the run");
    if let Ok(mut fatal) = control.fatal.lock() {
        *fatal = Some(error);
    }
    running.store(false, Ordering::Release);
}

fn watch_session(
    mut session: Session,
    registration: &Registration,
    control: &Control,
    running: &AtomicBool,
) {
    let expected = session.producers.clone();
    let mut failures = FailureRegimeLatch::new();
    let mut probe_failures = FailureRegimeLatch::new();
    let mut registration_failures = FailureRegimeLatch::new();
    let mut missing = 0u8;
    loop {
        let Ok(stopped) = control.stopped.lock() else {
            fail(
                control,
                running,
                "graph network stop state is poisoned".into(),
            );
            return;
        };
        let Ok((stopped, _)) = control.wake.wait_timeout(stopped, Duration::from_secs(1)) else {
            fail(
                control,
                running,
                "graph network wait state is poisoned".into(),
            );
            return;
        };
        if *stopped || !running.load(Ordering::Acquire) {
            return;
        }
        drop(stopped);
        let holding_error = session.client.poll_pushed_losses().err();
        let lost = session.client.take_mirror_losses().iter().any(|loss| {
            expected
                .iter()
                .any(|producer| loss.robot == producer.route && loss.topic == producer.topic)
        });
        let status = session
            .client
            .probe_status_until(Instant::now() + Duration::from_millis(200));
        if let Ok(status) = &status {
            report_robot_registration_status(
                &registration.graph,
                status.robot_startup_error.as_deref(),
                &mut registration_failures,
            );
        }
        if holding_error.is_none() && !lost {
            if let Err(error) = &status {
                missing = 0;
                match probe_failures.on_failure() {
                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                        tracing::debug!(graph = %registration.graph, error = %error, total_failures = probe_failures.total_failures(), "netd busy during graph status probe")
                    }
                    RegimeDecision::Suppressed { suppressed } => {
                        tracing::debug!(graph = %registration.graph, suppressed, "graph status probe remains busy")
                    }
                }
                continue;
            }
            if let Some(suppressed) = probe_failures.on_success() {
                tracing::debug!(graph = %registration.graph, suppressed, "graph status probe recovered");
            }
        }

        if status
            .as_ref()
            .is_ok_and(|status| status.lan_access != Some(LanAccess::Paired))
        {
            fail(
                control,
                running,
                "graph network daemon no longer confirms paired access".into(),
            );
            return;
        }
        let ready = holding_error.is_none()
            && !lost
            && status.as_ref().is_ok_and(|status| {
                expected.iter().all(|producer| {
                    status.demands.iter().any(|row| {
                        row.robot == producer.route
                            && row.topic == producer.topic
                            && row.refcount > 0
                            && row
                                .local_topic
                                .as_deref()
                                .is_none_or(|topic| topic == producer.topic)
                            && row.plane == Some(ServingPlane::Iroh)
                    })
                })
            });
        if ready {
            missing = 0;
        } else if holding_error.is_none() && !lost {
            missing = missing.saturating_add(1);
            if missing < 2 {
                continue;
            }
        }
        let result = if ready {
            Ok(())
        } else {
            let reason = status.err().map_or_else(
                || {
                    let names = expected
                        .iter()
                        .map(|producer| {
                            format!("robot '{}' topic '{}'", producer.route, producer.topic)
                        })
                        .collect::<Vec<_>>()
                        .join(", ");
                    format!("canonical graph reader unavailable for {names}")
                },
                |error| error.to_string(),
            );
            // Loss events and missing status rows both require re-demand. Always
            // revalidate the exact source key, schema and policy before reconnecting.
            match open_session(registration, Some(&expected), &|| {
                !running.load(Ordering::Acquire)
                    || control.stopped.lock().map_or(true, |stopped| *stopped)
            }) {
                Ok(reconnected) => {
                    session = reconnected;
                    Ok(())
                }
                Err(CliError::GraphIngressSourceChanged) => {
                    fail(
                        control,
                        running,
                        CliError::GraphIngressSourceChanged.to_string(),
                    );
                    return;
                }
                Err(reconnect) => Err(format!(
                    "graph network connection ended: {reason}; reconnect: {reconnect}"
                )),
            }
        };
        match result {
            Ok(()) => {
                if let Some(suppressed) = failures.on_success() {
                    tracing::info!(graph = %registration.graph, suppressed, total_failures = failures.total_failures(), "authenticated graph network recovered");
                }
            }
            Err(error) => match failures.on_failure() {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                    tracing::warn!(graph = %registration.graph, error = %error, total_failures = failures.total_failures(), "authenticated graph network is reconnecting")
                }
                RegimeDecision::Suppressed { .. } => {
                    tracing::debug!(graph = %registration.graph, error = %error, total_failures = failures.total_failures(), "authenticated graph network remains unavailable")
                }
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{CatalogEntry, CatalogProvenance, CatalogReply};
    use cerulion_core::EgressQos;
    use cerulion_netd::account_access::{RobotPath, RobotPresence, RobotTrustSource};

    #[test]
    fn paired_graph_watch_reports_registration_failure_once_then_recovers_on_stderr() {
        let _environment = crate::test_env::env_lock();
        const CHILD: &str = "CERULION_PAIRED_REGISTRATION_WARN_CHILD";
        if std::env::var_os(CHILD).is_none() {
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["graph_cmd::authenticated_ingress::tests::paired_graph_watch_reports_registration_failure_once_then_recovers_on_stderr", "--exact", "--nocapture"])
                .env(CHILD, "1").output().unwrap();
            assert!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            let stderr = String::from_utf8(output.stderr).unwrap();
            assert_eq!(
                stderr
                    .matches("automatic robot registration failed; remote access is unavailable")
                    .count(),
                1,
                "{stderr}"
            );
            assert!(
                stderr
                    .contains("the robot state directory must be owner-only; run chmod 700 <dir>"),
                "{stderr}"
            );
            assert_eq!(
                stderr
                    .matches("automatic robot registration recovered")
                    .count(),
                1,
                "{stderr}"
            );
            return;
        }
        struct StatusPlane {
            error: Mutex<Option<String>>,
            probes: std::sync::atomic::AtomicUsize,
        }
        impl cerulion_netd::egress::EgressPlane for StatusPlane {
            fn register_egress(
                &self,
                _: cerulion_netd::egress::EgressId,
                _: &GatewayPlan,
                _: &SchemaServing,
                _: Option<&str>,
            ) -> Result<bool, cerulion_netd::egress::EgressError> {
                Ok(false)
            }
            fn release_egress(&self, _: cerulion_netd::egress::EgressId) {}
            fn lan_access(&self) -> Option<LanAccess> {
                Some(LanAccess::Paired)
            }
            fn robot_startup_error(&self) -> Option<String> {
                let error = self.error.lock().unwrap().clone();
                self.probes.fetch_add(1, Ordering::SeqCst);
                error
            }
        }
        struct UnusedMirror;
        impl cerulion_netd::mirror::MirrorPlane for UnusedMirror {
            fn ensure_mirror(
                &self,
                key: &cerulion_netd::registry::TopicKey,
                _: u64,
            ) -> Result<(), cerulion_netd::mirror::MirrorError> {
                Err(cerulion_netd::mirror::MirrorError::Register {
                    key: key.clone(),
                    source: Box::new(cerulion_core::TransportError::Internal {
                        reason: "this status-only test must never demand a mirror".into(),
                    }),
                })
            }
            fn release_mirror(
                &self,
                _: &cerulion_netd::registry::TopicKey,
            ) -> cerulion_netd::mirror::MirrorRelease {
                cerulion_netd::mirror::MirrorRelease::Retired
            }
        }
        let root = tempfile::tempdir().unwrap();
        let socket = root.path().join("netd.sock");
        let plane = Arc::new(StatusPlane {
            error: Mutex::new(Some(
                "the robot state directory must be owner-only; run chmod 700 <dir>".into(),
            )),
            probes: std::sync::atomic::AtomicUsize::new(0),
        });
        let mut daemon = cerulion_netd::daemon::start_with_egress(
            socket.clone(),
            Arc::new(UnusedMirror),
            plane.clone(),
            cerulion_netd::NetdConfig {
                idle_grace: Duration::from_secs(60),
                ..Default::default()
            },
        )
        .unwrap();
        let session = Session {
            client: NetdClient::connect_existing_at(socket).unwrap(),
            _catalogs: RemoteCatalogs::default(),
            producers: vec![],
        };
        let registration = Registration {
            plan: GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec![],
                ingress: vec![],
            },
            schemas: SchemaServing::default(),
            namespace: String::new(),
            graph: "paired-producing-graph".into(),
        };
        let control = Arc::new(Control {
            stopped: Mutex::new(false),
            wake: Condvar::new(),
            fatal: Mutex::new(None),
        });
        let running = Arc::new(AtomicBool::new(true));
        cerulion_core::init_logging(false, false);
        let worker_control = control.clone();
        let worker_running = running.clone();
        let watch = std::thread::spawn(move || {
            watch_session(session, &registration, &worker_control, &worker_running)
        });
        let wait_for_probes = |count| {
            let deadline = Instant::now() + Duration::from_secs(6);
            while plane.probes.load(Ordering::SeqCst) < count && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(10));
            }
            assert!(
                plane.probes.load(Ordering::SeqCst) >= count,
                "paired watcher stopped probing netd status"
            );
        };
        wait_for_probes(2);
        *plane.error.lock().unwrap() = None;
        wait_for_probes(4);
        *control.stopped.lock().unwrap() = true;
        control.wake.notify_all();
        watch.join().unwrap();
        assert!(control.fatal.lock().unwrap().is_none());
        assert!(running.load(Ordering::SeqCst));
        daemon.shutdown();
    }

    fn input() -> GatewayIngressEntry {
        GatewayIngressEntry {
            topic: "/camera/vector".into(),
            schema_hash: 7,
        }
    }
    fn robot(route: &str) -> RobotCatalog {
        RobotCatalog {
            route: route.into(),
            name: route.into(),
            endpoint_key: Some([3; 32]),
            trust: RobotTrustSource::Paired,
            presence: RobotPresence::Online,
            path: RobotPath::Direct,
            catalog: Some(CatalogReply {
                version: CATALOG_WIRE_VERSION,
                robot: route.into(),
                error: None,
                wire_header_version: cerulion_core::wire::WIRE_HEADER_VERSION,
                entries: vec![CatalogEntry {
                    topic: "/camera/vector".into(),
                    schema_hash: Some(7),
                    schema_name: Some("geometry_msgs/Vector3".into()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: Some(1),
                    liveness: None,
                    egress_qos: Some(EgressQos {
                        topic: "/camera/vector".into(),
                        reliability: EgressReliability::Reliable,
                        send_queue: None,
                    }),
                }],
            }),
        }
    }

    #[test]
    fn canonical_ingress_selects_the_same_exact_pinned_writer_in_two_runs() {
        let expected = Producer {
            topic: "/camera/vector".into(),
            route: "paired-camera".into(),
            key: [3; 32],
            schema_hash: 7,
        };
        for _ in 0..2 {
            assert_eq!(
                select_producer(&input(), &[robot("paired-camera")]).unwrap(),
                expected
            );
        }
    }

    #[test]
    fn canonical_ingress_reconnect_refuses_any_source_identity_change() {
        let original = select_producer(&input(), &[robot("paired-camera")]).unwrap();
        let expected = [original.clone()];
        require_same_producers(None, &expected).unwrap();
        require_same_producers(Some(&expected), &expected).unwrap();
        let changes: [fn(&mut Producer); 4] = [
            |p| p.key = [4; 32],
            |p| p.route = "another-paired-camera".into(),
            |p| p.topic = "/other/input".into(),
            |p| p.schema_hash = 8,
        ];
        for change in changes {
            let mut selected = original.clone();
            change(&mut selected);
            assert!(matches!(
                require_same_producers(Some(&expected), &[selected]),
                Err(CliError::GraphIngressSourceChanged)
            ));
        }
        assert!(matches!(
            require_same_producers(Some(&expected), &[]),
            Err(CliError::GraphIngressSourceChanged)
        ));
    }

    #[test]
    fn canonical_ingress_refuses_missing_foreign_and_ambiguous_producers() {
        assert!(select_producer(&input(), &[])
            .unwrap_err()
            .to_string()
            .contains("0 authorized producers"));
        let mut foreign = robot("foreign");
        foreign.catalog.as_mut().unwrap().error = Some("not paired".into());
        assert!(select_producer(&input(), &[foreign])
            .unwrap_err()
            .to_string()
            .contains("0 authorized producers"));
        let mut unavailable = robot("unreachable");
        unavailable.catalog = None;
        assert!(select_producer(&input(), &[unavailable]).is_err());
        for sources in [[robot("a"), robot("b")], [robot("b"), robot("a")]] {
            assert!(select_producer(&input(), &sources)
                .unwrap_err()
                .to_string()
                .contains("2 authorized producers"));
        }
    }

    #[test]
    fn canonical_ingress_refuses_every_unverified_schema_writer_and_queue_arm() {
        let mutate: [fn(&mut RobotCatalog); 11] = [
            |r| r.endpoint_key = None,
            |r| r.catalog.as_mut().unwrap().version += 1,
            |r| r.catalog.as_mut().unwrap().wire_header_version += 1,
            |r| r.catalog.as_mut().unwrap().robot = "different".into(),
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = None,
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(0),
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(2),
            |r| r.catalog.as_mut().unwrap().entries[0].schema_hash = Some(8),
            |r| r.catalog.as_mut().unwrap().entries[0].egress_qos = None,
            |r| {
                r.catalog.as_mut().unwrap().entries[0]
                    .egress_qos
                    .as_mut()
                    .unwrap()
                    .reliability = EgressReliability::BestEffort
            },
            |r| {
                r.catalog.as_mut().unwrap().entries[0]
                    .egress_qos
                    .as_mut()
                    .unwrap()
                    .topic = "/other".into()
            },
        ];
        for change in mutate {
            let mut source = robot("paired-camera");
            change(&mut source);
            assert!(select_producer(&input(), &[source]).is_err());
        }
        let mut duplicate = robot("paired-camera");
        let entries = &mut duplicate.catalog.as_mut().unwrap().entries;
        entries.push(entries[0].clone());
        assert!(select_producer(&input(), &[duplicate]).is_err());
    }
}
