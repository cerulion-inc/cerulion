// SPDX-License-Identifier: AGPL-3.0-only
//! Canonical graph demands. Observer reads use their separate private channels.

use std::collections::BTreeMap;
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

/// What one catalog snapshot decides for a declared input that no check has
/// refused. Of these verdicts, only a missing schema hash keeps discovery reading.
#[derive(Debug, PartialEq, Eq)]
enum Selection {
    Ready(Producer),
    SchemaUnknown,
}

/// What one catalog snapshot decides for every declared input together.
#[derive(Debug, PartialEq, Eq)]
enum Judged<'a> {
    Ready(Vec<Producer>),
    SchemaUnknown(&'a GatewayIngressEntry),
}

/// A declaration selects exactly one live authorized writer, never a matching
/// hash among several writers. Unknown metadata cannot authorize a graph input.
/// A refusal of any input is reported ahead of an input whose schema hash is
/// still unknown, so an unknown schema is named only when it is all that is left.
fn select_producers(
    inputs: &[GatewayIngressEntry],
    robots: &[RobotCatalog],
) -> CliResult<Vec<Producer>> {
    match judge_inputs(inputs, robots)? {
        Judged::Ready(producers) => Ok(producers),
        Judged::SchemaUnknown(input) => Err(CliError::Validation(format!(
            "graph ingress '{}': producer schema is unknown because the robot reported no schema hash for the topic within the discovery limit; check that the producer is publishing",
            input.topic
        ))),
    }
}

fn judge_inputs<'a>(
    inputs: &'a [GatewayIngressEntry],
    robots: &[RobotCatalog],
) -> CliResult<Judged<'a>> {
    let mut producers = Vec::with_capacity(inputs.len());
    let mut unknown = None;
    for input in inputs {
        match judge_producer(input, robots)? {
            Selection::Ready(producer) => producers.push(producer),
            Selection::SchemaUnknown => {
                unknown.get_or_insert(input);
            }
        }
    }
    Ok(match unknown {
        Some(input) => Judged::SchemaUnknown(input),
        None => Judged::Ready(producers),
    })
}

fn judge_producer(input: &GatewayIngressEntry, robots: &[RobotCatalog]) -> CliResult<Selection> {
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
    let policy = entry.egress_qos.as_ref().ok_or_else(|| {
        CliError::Validation(format!(
            "graph ingress '{}': producer queue policy is unknown; upgrade and restart its graph",
            input.topic
        ))
    })?;
    if policy.topic != input.topic || policy.reliability != EgressReliability::Reliable {
        return Err(CliError::Validation(format!("graph ingress '{}': authenticated wire ingress requires a declared reliable producer policy", input.topic)));
    }
    match entry.schema_hash {
        None => return Ok(Selection::SchemaUnknown),
        Some(hash) if hash != input.schema_hash => {
            return Err(CliError::Validation(format!(
                "graph ingress '{}': producer schema 0x{hash:016x} does not match the consuming graph's 0x{:016x}",
                input.topic, input.schema_hash
            )));
        }
        Some(_) => {}
    }
    Ok(Selection::Ready(Producer {
        topic: input.topic.clone(),
        route: robot.route.clone(),
        key,
        schema_hash: input.schema_hash,
    }))
}

fn any_answered(robots: &[RobotCatalog]) -> bool {
    robots.iter().any(|robot| {
        robot
            .catalog
            .as_ref()
            .is_some_and(|catalog| catalog.error.is_none())
    })
}

/// A robot fills a catalog entry's schema hash from a message the producer
/// publishes within a short per-catalog budget, so a catalog can list a live
/// single writer with no hash and a later catalog can name it. Discovery has
/// settled when no robot is known, or when a robot answered and the snapshot
/// either refuses some declared input or names every hash it needs.
fn discovery_settled(inputs: &[GatewayIngressEntry], robots: &[RobotCatalog]) -> bool {
    robots.is_empty()
        || (any_answered(robots)
            && !matches!(judge_inputs(inputs, robots), Ok(Judged::SchemaUnknown(_))))
}

/// Reads catalogs until discovery settles or `deadline` passes. At the deadline
/// a last read in which no robot returned a catalog is replaced by the latest
/// answered one, which says more about why the inputs are unresolved; a robot's
/// refusal in the last read is kept.
fn discover_until_settled(
    inputs: &[GatewayIngressEntry],
    graph: &str,
    deadline: Instant,
    cancelled: &impl Fn() -> bool,
    mut discover: impl FnMut() -> CliResult<RemoteCatalogs>,
) -> CliResult<RemoteCatalogs> {
    let mut answered_before = None;
    let mut waiting = false;
    loop {
        check_cancelled(cancelled)?;
        let catalogs = discover()?;
        if discovery_settled(inputs, &catalogs.robots) {
            return Ok(catalogs);
        }
        if Instant::now() >= deadline {
            let reached = catalogs.robots.iter().any(|robot| robot.catalog.is_some());
            return Ok(match answered_before {
                Some(earlier) if !reached => earlier,
                _ => catalogs,
            });
        }
        if !waiting {
            waiting = true;
            tracing::debug!(
                graph,
                ingress_topics = inputs.len(),
                "graph ingress discovery is waiting for a robot to answer or for a producer schema hash"
            );
        }
        if any_answered(&catalogs.robots) {
            answered_before = Some(catalogs);
        }
        std::thread::sleep(Duration::from_millis(100));
    }
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
    epoch_failures: &mut BTreeMap<String, FailureRegimeLatch>,
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
        discover_until_settled(
            &registration.plan.ingress,
            &registration.graph,
            Instant::now() + Duration::from_secs(10),
            cancelled,
            automatic::discover,
        )?
    };
    let producers =
        select_producers(&registration.plan.ingress, &catalogs.robots).map_err(|error| {
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
        report_epoch_refresh(
            &registration.graph,
            &producer.route,
            catalogs.sync_selected_epoch(&producer.route),
            epoch_failures,
        );
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

fn report_epoch_refresh(
    graph: &str,
    robot: &str,
    result: CliResult<()>,
    failures: &mut BTreeMap<String, FailureRegimeLatch>,
) {
    let latch = failures.entry(robot.to_owned()).or_default();
    match result {
        Ok(()) => {
            if let Some(suppressed) = latch.on_success() {
                tracing::info!(
                    graph,
                    robot,
                    suppressed,
                    total_failures = latch.total_failures(),
                    "graph ingress revocation refresh recovered"
                );
            }
        }
        Err(error) => match latch.on_failure() {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                tracing::warn!(graph, robot, error = %error, total_failures = latch.total_failures(), "graph ingress revocation refresh failed; continuing with saved authenticated trust");
            }
            RegimeDecision::Suppressed { .. } => {
                tracing::debug!(graph, robot, error = %error, total_failures = latch.total_failures(), "graph ingress revocation refresh remains unavailable; saved authenticated trust is retained");
            }
        },
    }
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
    #[cfg(test)]
    accepted_statuses: [std::sync::atomic::AtomicUsize; 2],
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
        let mut epoch_failures = BTreeMap::new();
        let session = open_session(
            &registration,
            None,
            &|| !running.load(Ordering::Acquire),
            &mut epoch_failures,
        )?;
        let control = Arc::new(Control {
            stopped: Mutex::new(false),
            wake: Condvar::new(),
            fatal: Mutex::new(None),
            #[cfg(test)]
            accepted_statuses: std::array::from_fn(|_| std::sync::atomic::AtomicUsize::new(0)),
        });
        let worker_control = control.clone();
        let worker = std::thread::Builder::new()
            .name("graph-network".into())
            .spawn(move || {
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    watch_session(
                        session,
                        &registration,
                        &worker_control,
                        &running,
                        epoch_failures,
                    )
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
    mut epoch_failures: BTreeMap<String, FailureRegimeLatch>,
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
            // Advance only after the consumer accepted and reported this reply.
            #[cfg(test)]
            control.accepted_statuses[usize::from(status.robot_startup_error.is_none())]
                .fetch_add(1, Ordering::Release);
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
            match open_session(
                registration,
                Some(&expected),
                &|| {
                    !running.load(Ordering::Acquire)
                        || control.stopped.lock().map_or(true, |stopped| *stopped)
                },
                &mut epoch_failures,
            ) {
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
    #[tracing_test::traced_test]
    fn epoch_refresh_failures_preserve_each_route_regime_until_recovery() {
        let mut failures = BTreeMap::new();
        for robot in ["first", "second"] {
            for _ in 0..3 {
                report_epoch_refresh(
                    "import",
                    robot,
                    Err(CliError::Validation("account service unavailable".into())),
                    &mut failures,
                );
            }
        }
        assert_eq!(failures["first"].total_failures(), 3);
        assert_eq!(failures["second"].total_failures(), 3);
        report_epoch_refresh("import", "first", Ok(()), &mut failures);
        report_epoch_refresh("import", "first", Ok(()), &mut failures);
        logs_assert(|lines: &[&str]| {
            let warnings = cerulion_core::testing::count_at(lines, "WARN", "graph ingress revocation refresh failed; continuing with saved authenticated trust");
            let recoveries = cerulion_core::testing::count_at(
                lines,
                "INFO",
                "graph ingress revocation refresh recovered",
            );
            if warnings == 2 && recoveries == 1 {
                Ok(())
            } else {
                Err(format!("expected one warning per route and one recovery, got {warnings} warnings and {recoveries} recoveries"))
            }
        });
    }

    #[test]
    fn paired_graph_watch_reports_registration_failure_once_then_recovers_on_stderr() {
        let _environment = crate::test_env::env_lock();
        const CHILD: &str = "CERULION_PAIRED_REGISTRATION_WARN_CHILD";
        if std::env::var_os(CHILD).is_none() {
            let output = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["graph_cmd::authenticated_ingress::tests::paired_graph_watch_reports_registration_failure_once_then_recovers_on_stderr", "--exact", "--nocapture"])
                .env(CHILD, "1").env("RUST_LOG", "info").output().unwrap();
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
            #[cfg(test)]
            accepted_statuses: std::array::from_fn(|_| std::sync::atomic::AtomicUsize::new(0)),
        });
        let running = Arc::new(AtomicBool::new(true));
        cerulion_core::init_logging(false, false);
        let worker_control = control.clone();
        let worker_running = running.clone();
        let watch = std::thread::spawn(move || {
            watch_session(
                session,
                &registration,
                &worker_control,
                &worker_running,
                BTreeMap::new(),
            )
        });
        let wait_for_statuses = |healthy: bool, count| {
            let observed = &control.accepted_statuses[usize::from(healthy)];
            let deadline = Instant::now() + Duration::from_secs(6);
            while observed.load(Ordering::Acquire) < count && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(10));
            }
            assert!(
                observed.load(Ordering::Acquire) >= count,
                "paired watcher stopped accepting netd status replies"
            );
        };
        wait_for_statuses(false, 2);
        *plane.error.lock().unwrap() = None;
        wait_for_statuses(true, 2);
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
    fn select_one(robots: &[RobotCatalog]) -> CliResult<Producer> {
        select_producers(&[input()], robots).map(|mut producers| producers.remove(0))
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
            assert_eq!(select_one(&[robot("paired-camera")]).unwrap(), expected);
        }
    }

    #[test]
    fn canonical_ingress_reconnect_refuses_any_source_identity_change() {
        let original = select_one(&[robot("paired-camera")]).unwrap();
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
        assert!(select_one(&[])
            .unwrap_err()
            .to_string()
            .contains("0 authorized producers"));
        let mut foreign = robot("foreign");
        foreign.catalog.as_mut().unwrap().error = Some("not paired".into());
        assert!(select_one(&[foreign])
            .unwrap_err()
            .to_string()
            .contains("0 authorized producers"));
        let mut unavailable = robot("unreachable");
        unavailable.catalog = None;
        assert!(select_one(&[unavailable]).is_err());
        for sources in [[robot("a"), robot("b")], [robot("b"), robot("a")]] {
            assert!(select_one(&sources)
                .unwrap_err()
                .to_string()
                .contains("2 authorized producers"));
        }
    }

    #[test]
    fn canonical_ingress_refuses_every_unverified_schema_writer_and_queue_arm() {
        let mutate: [fn(&mut RobotCatalog); 12] = [
            |r| r.endpoint_key = None,
            |r| r.catalog.as_mut().unwrap().version += 1,
            |r| r.catalog.as_mut().unwrap().wire_header_version += 1,
            |r| r.catalog.as_mut().unwrap().robot = "different".into(),
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = None,
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(0),
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(2),
            |r| r.catalog.as_mut().unwrap().entries[0].schema_hash = Some(8),
            |r| r.catalog.as_mut().unwrap().entries[0].schema_hash = None,
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
            assert!(select_one(&[source]).is_err());
        }
        let mut duplicate = robot("paired-camera");
        let entries = &mut duplicate.catalog.as_mut().unwrap().entries;
        entries.push(entries[0].clone());
        assert!(select_one(&[duplicate]).is_err());
    }

    fn with_schema(route: &str, schema_hash: Option<u64>) -> RobotCatalog {
        let mut source = robot(route);
        source.catalog.as_mut().unwrap().entries[0].schema_hash = schema_hash;
        source
    }

    #[test]
    fn canonical_ingress_names_an_unknown_producer_schema_apart_from_a_mismatch() {
        assert_eq!(
            select_one(&[with_schema("paired-camera", None)])
                .unwrap_err()
                .to_string(),
            "graph ingress '/camera/vector': producer schema is unknown because the robot \
             reported no schema hash for the topic within the discovery limit; check that the \
             producer is publishing"
        );
        assert_eq!(
            select_one(&[with_schema("paired-camera", Some(8))])
                .unwrap_err()
                .to_string(),
            "graph ingress '/camera/vector': producer schema 0x0000000000000008 does not match \
             the consuming graph's 0x0000000000000007"
        );
    }

    /// A second declared input whose single writer is best effort, which
    /// graph ingress refuses on any snapshot.
    fn arm_input() -> GatewayIngressEntry {
        GatewayIngressEntry {
            topic: "/arm/state".into(),
            schema_hash: 9,
        }
    }
    fn with_best_effort_arm(mut source: RobotCatalog) -> RobotCatalog {
        let entries = &mut source.catalog.as_mut().unwrap().entries;
        let mut arm = entries[0].clone();
        arm.topic = "/arm/state".into();
        arm.schema_hash = Some(9);
        arm.egress_qos = Some(EgressQos {
            topic: "/arm/state".into(),
            reliability: EgressReliability::BestEffort,
            send_queue: None,
        });
        entries.push(arm);
        source
    }

    #[test]
    fn a_refused_input_is_reported_ahead_of_an_unknown_schema_in_either_order() {
        let source = with_best_effort_arm(with_schema("paired-camera", None));
        for inputs in [[input(), arm_input()], [arm_input(), input()]] {
            assert_eq!(
                select_producers(&inputs, std::slice::from_ref(&source))
                    .unwrap_err()
                    .to_string(),
                "graph ingress '/arm/state': authenticated wire ingress requires a declared \
                 reliable producer policy"
            );
        }
    }

    #[test]
    fn discovery_settles_on_every_final_answer_and_waits_only_on_an_unknown_schema() {
        let inputs = [input()];
        let edit = |route: &str, change: fn(&mut RobotCatalog)| {
            let mut source = with_schema(route, None);
            change(&mut source);
            source
        };
        let refused = || {
            edit("refused", |r| {
                r.catalog.as_mut().unwrap().error = Some("not paired".into())
            })
        };
        let unreached = || edit("unreached", |r| r.catalog = None);
        // Each of these refuses the input on this snapshot, so discovery stops.
        let snapshot_refusals: [fn(&mut RobotCatalog); 7] = [
            |r| r.endpoint_key = None,
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = None,
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(2),
            |r| r.catalog.as_mut().unwrap().entries[0].egress_qos = None,
            |r| {
                r.catalog.as_mut().unwrap().entries[0]
                    .egress_qos
                    .as_mut()
                    .unwrap()
                    .reliability = EgressReliability::BestEffort
            },
            |r| r.catalog.as_mut().unwrap().entries[0].topic = "/camera/other".into(),
            |r| r.catalog.as_mut().unwrap().entries[0].producer_count = Some(0),
        ];
        let both = [input(), arm_input()];
        for _ in 0..2 {
            assert!(discovery_settled(&inputs, &[]));
            assert!(discovery_settled(&inputs, &[robot("paired-camera")]));
            assert!(discovery_settled(
                &[],
                &[with_schema("paired-camera", None)]
            ));
            assert!(!discovery_settled(
                &inputs,
                &[with_schema("paired-camera", None)]
            ));
            assert!(!discovery_settled(&inputs, &[unreached()]));
            assert!(!discovery_settled(&inputs, &[refused()]));
            assert!(!discovery_settled(
                &inputs,
                &[refused(), unreached(), with_schema("paired-camera", None)]
            ));
            // A second writer is refused at once, whichever of the two lacks a hash.
            assert!(discovery_settled(
                &inputs,
                &[robot("a"), with_schema("b", None)]
            ));
            assert!(discovery_settled(
                &inputs,
                &[with_schema("a", None), robot("b")]
            ));
            for change in snapshot_refusals {
                assert!(discovery_settled(&inputs, &[edit("paired-camera", change)]));
            }
            // One refused input ends discovery even while another lacks its hash.
            assert!(discovery_settled(
                &both,
                &[with_best_effort_arm(with_schema("paired-camera", None))]
            ));
            let mut hashed_arm = with_best_effort_arm(with_schema("paired-camera", None));
            hashed_arm.catalog.as_mut().unwrap().entries[1]
                .egress_qos
                .as_mut()
                .unwrap()
                .reliability = EgressReliability::Reliable;
            assert!(!discovery_settled(&both, &[hashed_arm]));
        }
    }

    fn snapshot(robots: Vec<RobotCatalog>) -> RemoteCatalogs {
        let mut catalogs = RemoteCatalogs::default();
        catalogs.robots = robots;
        catalogs
    }

    fn unreachable_robot() -> RobotCatalog {
        let mut source = robot("paired-camera");
        source.catalog = None;
        source
    }

    #[test]
    fn discovery_reads_catalogs_until_the_producer_schema_hash_arrives() {
        let mut sequence = vec![
            snapshot(vec![robot("paired-camera")]),
            snapshot(vec![with_schema("paired-camera", None)]),
            snapshot(vec![unreachable_robot()]),
        ];
        let mut reads = 0;
        let catalogs = discover_until_settled(
            &[input()],
            "ingress",
            Instant::now() + Duration::from_secs(60),
            &|| false,
            || {
                reads += 1;
                Ok(sequence
                    .pop()
                    .expect("discovery read past the settled catalog"))
            },
        )
        .unwrap();
        assert_eq!(reads, 3);
        assert_eq!(
            select_one(&catalogs.robots).unwrap(),
            Producer {
                topic: "/camera/vector".into(),
                route: "paired-camera".into(),
                key: [3; 32],
                schema_hash: 7,
            }
        );
    }

    #[test]
    fn discovery_past_its_deadline_keeps_the_snapshot_and_refuses_the_unknown_schema() {
        let mut reads = 0;
        let catalogs =
            discover_until_settled(&[input()], "ingress", Instant::now(), &|| false, || {
                reads += 1;
                assert_eq!(reads, 1, "discovery read again past its deadline");
                Ok(snapshot(vec![with_schema("paired-camera", None)]))
            })
            .unwrap();
        assert!(select_one(&catalogs.robots)
            .unwrap_err()
            .to_string()
            .contains("producer schema is unknown"));
    }

    #[test]
    fn discovery_at_its_deadline_keeps_the_latest_answer_over_an_unreached_robot() {
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut reads = 0;
        let catalogs = discover_until_settled(&[input()], "ingress", deadline, &|| false, || {
            reads += 1;
            match reads {
                1 => Ok(snapshot(vec![with_schema("paired-camera", None)])),
                2 => Ok(snapshot(vec![unreachable_robot()])),
                3 => {
                    while Instant::now() < deadline {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    Ok(snapshot(vec![unreachable_robot()]))
                }
                _ => panic!("discovery read again past its deadline"),
            }
        })
        .unwrap();
        assert_eq!(reads, 3);
        assert!(select_one(&catalogs.robots)
            .unwrap_err()
            .to_string()
            .contains("producer schema is unknown"));
        // A robot that refuses in the last read is reported, not hidden by an
        // earlier answer.
        let mut reads = 0;
        let deadline = Instant::now() + Duration::from_secs(2);
        let catalogs = discover_until_settled(&[input()], "ingress", deadline, &|| false, || {
            reads += 1;
            match reads {
                1 => Ok(snapshot(vec![with_schema("paired-camera", None)])),
                2 => {
                    while Instant::now() < deadline {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    let mut refused = robot("paired-camera");
                    refused.catalog.as_mut().unwrap().error = Some("not paired".into());
                    Ok(snapshot(vec![refused]))
                }
                _ => panic!("discovery read again past its deadline"),
            }
        })
        .unwrap();
        assert_eq!(reads, 2);
        assert_eq!(
            catalogs.robots[0]
                .catalog
                .as_ref()
                .unwrap()
                .error
                .as_deref(),
            Some("not paired")
        );
    }

    #[test]
    fn discovery_stops_reading_once_cancelled_and_passes_a_discovery_error_through() {
        let reads = std::cell::Cell::new(0);
        let Err(interrupted) = discover_until_settled(
            &[input()],
            "ingress",
            Instant::now() + Duration::from_secs(60),
            &|| reads.get() > 0,
            || {
                reads.set(reads.get() + 1);
                assert_eq!(reads.get(), 1, "discovery read again after cancellation");
                Ok(snapshot(vec![with_schema("paired-camera", None)]))
            },
        ) else {
            panic!("a cancelled discovery must stop");
        };
        assert_eq!(
            interrupted.to_string(),
            "authenticated graph network setup was interrupted"
        );
        let Err(failed) = discover_until_settled(
            &[input()],
            "ingress",
            Instant::now() + Duration::from_secs(60),
            &|| false,
            || Err(CliError::Validation("directory unavailable".into())),
        ) else {
            panic!("a failed discovery must pass its error through");
        };
        assert_eq!(failed.to_string(), "directory unavailable");
    }
}
