// SPDX-License-Identifier: AGPL-3.0-only
//! Connection-scoped ownership of the machine's authenticated remote server.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::{GatewayPlan, SchemaServing};

use crate::egress::{EgressError, EgressId, EgressPlane, GatewayEgressPlane};
use crate::mirror::MirrorPlane;
use crate::robot_supervisor::RobotSupervisor;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Starting,
    Serving,
    Reaping,
    Stopped,
}

/// The refusal while the daemon is stopping.
const SHUTTING_DOWN: &str = "network daemon is shutting down";

struct State {
    registrations: BTreeSet<EgressId>,
    /// Connection-scoped serving holds and their deadlines.
    holds: BTreeMap<EgressId, Instant>,
    phase: Phase,
    standing_requested: bool,
    stopped: bool,
}

impl State {
    /// Whether a server is wanted at `now`: standing serving, an egress
    /// registration, or a serving hold whose deadline is after `now`. The one
    /// rule for both the worker's decision and [`Self::may_reap`].
    fn wanted(&self, now: Instant) -> bool {
        self.standing_requested
            || !self.registrations.is_empty()
            || self.holds.values().any(|until| *until > now)
    }

    /// The worker's re-check once it holds the exclusive idle fence: the reap
    /// goes ahead only while the daemon is not stopping, nothing wants a server
    /// at `now` and no outbound reader remains.
    fn may_reap(&self, now: Instant, readers: bool) -> bool {
        !self.stopped && !self.wanted(now) && !readers
    }

    /// Hold serving for connection `id` until `until`. A connection has at most
    /// one hold: a later one replaces the earlier, deadline included, whether
    /// it is shorter or longer.
    fn hold(&mut self, id: EgressId, until: Instant) {
        self.holds.insert(id, until);
    }

    /// Remove every serving hold whose deadline is at or before `now`, and
    /// return the connections that held them.
    fn expire_holds(&mut self, now: Instant) -> Vec<EgressId> {
        let expired: Vec<EgressId> = self
            .holds
            .iter()
            .filter(|(_, until)| **until <= now)
            .map(|(id, _)| *id)
            .collect();
        for id in &expired {
            self.holds.remove(id);
        }
        expired
    }
}

#[derive(Debug, PartialEq, Eq)]
enum IdleAction {
    Stop,
    Start,
    Wait,
    Reap,
}

fn idle_action(stopped: bool, wanted: bool, has_server: bool, readers: bool) -> IdleAction {
    if stopped {
        IdleAction::Stop
    } else if wanted && !has_server {
        IdleAction::Start
    } else if has_server && !wanted && !readers {
        IdleAction::Reap
    } else {
        IdleAction::Wait
    }
}

/// First successful egress or serving hold wakes one lifecycle worker.
/// Registrations and holds never wait for endpoint transfer, bootstrap or
/// graceful child teardown. A hold lapses at its deadline or with its
/// connection. Idle teardown waits for outbound readers, admitted work, owed
/// delivery and the e-stop latch.
pub struct SupervisedEgressPlane {
    gateway: Arc<GatewayEgressPlane>,
    mirror: Arc<dyn MirrorPlane>,
    standing: bool,
    holds_endpoint: Arc<AtomicBool>,
    stop_grace: Duration,
    state: Arc<Mutex<State>>,
    wake: Arc<Condvar>,
    worker: Mutex<Option<JoinHandle<()>>>,
}

impl SupervisedEgressPlane {
    /// Construction opens no endpoint and starts no worker.
    pub fn new(
        gateway: Arc<GatewayEgressPlane>,
        mirror: Arc<dyn MirrorPlane>,
        standing: bool,
    ) -> Self {
        Self {
            gateway,
            mirror,
            standing,
            holds_endpoint: Arc::new(AtomicBool::new(false)),
            stop_grace: crate::robot_supervisor::resolve_epoch_sync_stop_grace(
                std::env::var(crate::robot_supervisor::EPOCH_SYNC_STOP_GRACE_ENV)
                    .ok()
                    .as_deref(),
            )
            .saturating_add(crate::robot_supervisor::remoted_stop_grace()),
            state: Arc::new(Mutex::new(State {
                registrations: BTreeSet::new(),
                holds: BTreeMap::new(),
                phase: Phase::Idle,
                standing_requested: false,
                stopped: false,
            })),
            wake: Arc::new(Condvar::new()),
            worker: Mutex::new(None),
        }
    }

    fn ensure_worker(&self) -> Result<(), EgressError> {
        let mut worker = self.worker.lock().unwrap_or_else(|e| e.into_inner());
        if self.state.lock().unwrap_or_else(|e| e.into_inner()).stopped {
            return Err(EgressError::Server {
                reason: SHUTTING_DOWN.into(),
            });
        }
        if worker.is_none() {
            let state = Arc::clone(&self.state);
            let wake = Arc::clone(&self.wake);
            let gateway = Arc::clone(&self.gateway);
            let mirror = Arc::clone(&self.mirror);
            let held = Arc::clone(&self.holds_endpoint);
            *worker = Some(
                std::thread::Builder::new()
                    .name("netd-idle-server".into())
                    .spawn(move || lifecycle_worker(state, wake, gateway, mirror, held))
                    .map_err(|error| EgressError::Server {
                        reason: error.to_string(),
                    })?,
            );
        }
        self.wake.notify_all();
        Ok(())
    }

    /// Explicit LISTEN serving starts after the daemon acquires its singleton.
    pub fn start_standing(&self) -> Result<(), EgressError> {
        if self.standing {
            self.state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .standing_requested = true;
            self.ensure_worker()?;
        }
        Ok(())
    }

    /// Stop accepting registrations; join graceful teardown outside bookkeeping.
    pub fn shutdown(&self) {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if state.stopped {
            while state.phase != Phase::Stopped {
                state = self.wake.wait(state).unwrap_or_else(|e| e.into_inner());
            }
            return;
        }
        state.stopped = true;
        state.registrations.clear();
        state.holds.clear();
        drop(state);
        self.wake.notify_all();
        let worker = self.worker.lock().unwrap_or_else(|e| e.into_inner()).take();
        if let Some(worker) = worker {
            if worker.join().is_err() {
                tracing::error!("remote serving lifecycle worker failed during shutdown");
            }
        }
        self.state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Stopped;
        self.wake.notify_all();
    }

    /// Extra graceful server/fetch budget, including a registration still starting.
    pub fn stop_grace(&self) -> Duration {
        self.stop_grace
    }
}

/// Supervisor start failures and idle-fence failures are separate conditions,
/// each with its own failure regime.
struct LifecycleLatches {
    start: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    fence: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
}

impl LifecycleLatches {
    fn new() -> Self {
        Self {
            start: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new(),
            fence: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new(),
        }
    }
    fn start_failed(&mut self) -> cerulion_core::transport::failure_regime_latch::RegimeDecision {
        self.start.on_failure()
    }
    fn started(&mut self) -> Option<u64> {
        self.start.on_success()
    }
    fn fence_failed(&mut self) -> cerulion_core::transport::failure_regime_latch::RegimeDecision {
        self.fence.on_failure()
    }
    fn fence_ok(&mut self) -> Option<u64> {
        self.fence.on_success()
    }
}

/// The state root a serving hold reports, from the resolved `root`: refused
/// when nothing resolves ([`EgressError::NoStateRoot`]), when it is not
/// absolute or holds a `.` or `..` component, the predicate the supervisor
/// applies ([`EgressError::StateRootNotAbsolute`]), or when it is not UTF-8
/// ([`EgressError::StateRootNotUtf8`]), in that order.
fn serving_root(root: Option<PathBuf>) -> Result<String, EgressError> {
    let root = root.ok_or(EgressError::NoStateRoot)?;
    if !crate::robot_supervisor::absolute_without_traversal(&root) {
        return Err(EgressError::StateRootNotAbsolute);
    }
    root.into_os_string()
        .into_string()
        .map_err(|_| EgressError::StateRootNotUtf8)
}

fn lifecycle_worker(
    state: Arc<Mutex<State>>,
    wake: Arc<Condvar>,
    gateway: Arc<GatewayEgressPlane>,
    mirror: Arc<dyn MirrorPlane>,
    held: Arc<AtomicBool>,
) {
    use cerulion_core::transport::failure_regime_latch::RegimeDecision;
    let mut supervisor: Option<RobotSupervisor> = None;
    let mut failures = LifecycleLatches::new();
    loop {
        // Lapsed holds leave the map here and are logged once; `wanted(now)`
        // already ignores them, so the decision below never waits on this.
        let expired = state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .expire_holds(Instant::now());
        for conn in expired {
            tracing::info!(conn, "serving hold reached its deadline and was released");
        }
        // The controller may be busy with WAN I/O. Never hold lifecycle state
        // while sampling it: registrations still get an immediate reply.
        let readers = mirror.holds_outbound_readers();
        let mut current = state.lock().unwrap_or_else(|e| e.into_inner());
        let wanted = current.wanted(Instant::now());
        match idle_action(
            current.stopped,
            wanted,
            current.phase == Phase::Serving,
            readers,
        ) {
            IdleAction::Stop => {
                current.phase = Phase::Reaping;
                drop(current);
                if let Some(mut server) = supervisor.take() {
                    server.shutdown();
                }
                state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Stopped;
                return;
            }
            IdleAction::Start => {
                current.phase = Phase::Starting;
                drop(current);
                match RobotSupervisor::start(
                    Arc::new(AtomicBool::new(true)),
                    Arc::clone(&gateway),
                    Arc::clone(&mirror),
                    Arc::clone(&held),
                ) {
                    Ok(server) => {
                        supervisor = Some(server);
                        state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Serving;
                        if let Some(suppressed) = failures.started() {
                            tracing::info!(suppressed, "remote serving supervisor start recovered");
                        }
                        continue;
                    }
                    Err(error) => {
                        state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Idle;
                        match failures.start_failed() {
                            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                                tracing::warn!(error = %error, total_failures = failures.start.total_failures(), "remote serving supervisor could not start; retrying")
                            }
                            RegimeDecision::Suppressed { .. } => {
                                tracing::debug!(error = %error, total_failures = failures.start.total_failures(), "remote serving supervisor start still unavailable")
                            }
                        }
                    }
                }
            }
            IdleAction::Reap => {
                drop(current);
                let fence = cerulion_discovery::robot_state::resolve()
                    .ok_or_else(|| std::io::Error::other("no robot state root is available"))
                    .and_then(|root| {
                        crate::serving_hold::IdleFence::try_acquire(
                            &crate::serving_hold::fence_path(&root),
                        )
                    });
                if fence.is_ok() {
                    if let Some(suppressed) = failures.fence_ok() {
                        tracing::info!(suppressed, "remote serving idle fence recovered");
                    }
                }
                match fence {
                    Ok(Some(fence)) => {
                        // Outbound Ensure also takes this native fence in shared
                        // mode. Once exclusive, no new reader can race the stop.
                        let readers = mirror.holds_outbound_readers();
                        let mut current = state.lock().unwrap_or_else(|e| e.into_inner());
                        if current.may_reap(Instant::now(), readers) {
                            current.phase = Phase::Reaping;
                            drop(current);
                            if let Some(mut server) = supervisor.take() {
                                server.shutdown();
                            }
                            state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Idle;
                            // Retain both the endpoint owner and exclusive fence
                            // until server, epoch worker and end_serving finish.
                            drop(fence);
                            continue;
                        }
                    }
                    Ok(None) => {}
                    Err(error) => match failures.fence_failed() {
                        RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                            tracing::warn!(error = %error, total_failures = failures.fence.total_failures(), "remote serving idle fence unavailable; retrying")
                        }
                        RegimeDecision::Suppressed { .. } => {
                            tracing::debug!(error = %error, total_failures = failures.fence.total_failures(), "remote serving idle fence remains unavailable")
                        }
                    },
                }
            }
            IdleAction::Wait => drop(current),
        }
        let current = state.lock().unwrap_or_else(|e| e.into_inner());
        let _wait = wake.wait_timeout(current, Duration::from_millis(100));
    }
}

impl EgressPlane for SupervisedEgressPlane {
    fn holds_serving_endpoint(&self) -> bool {
        self.holds_endpoint.load(Ordering::Acquire)
    }

    fn register_egress(
        &self,
        id: EgressId,
        plan: &GatewayPlan,
        serving: &SchemaServing,
        ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        if self.state.lock().unwrap_or_else(|e| e.into_inner()).stopped {
            return Err(EgressError::Server {
                reason: SHUTTING_DOWN.into(),
            });
        }
        let booted = self
            .gateway
            .register_egress(id, plan, serving, ix_config_json)?;
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if state.stopped {
            drop(state);
            self.gateway.release_egress(id);
            return Err(EgressError::Server {
                reason: SHUTTING_DOWN.into(),
            });
        }
        state.registrations.insert(id);
        drop(state);
        if let Err(error) = self.ensure_worker() {
            self.state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .registrations
                .remove(&id);
            self.gateway.release_egress(id);
            return Err(error);
        }
        Ok(booted)
    }

    fn release_egress(&self, id: EgressId) {
        self.gateway.release_egress(id);
        self.state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .registrations
            .remove(&id);
        self.wake.notify_all();
    }

    fn serving_state_root(&self) -> Result<String, EgressError> {
        serving_root(cerulion_discovery::robot_state::resolve())
    }

    /// If the lifecycle worker cannot start (a stopping daemon or a failed
    /// thread spawn), the hold is removed again and the start's error returned.
    fn hold_serving(&self, id: EgressId, until: Instant) -> Result<(), EgressError> {
        self.state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .hold(id, until);
        if let Err(error) = self.ensure_worker() {
            self.state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .holds
                .remove(&id);
            return Err(error);
        }
        Ok(())
    }

    fn release_serving(&self, id: EgressId) {
        let released = self
            .state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .holds
            .remove(&id)
            .is_some();
        if released {
            tracing::debug!(conn = id, "serving hold released");
            self.wake.notify_all();
        }
    }

    fn serving_schema_snapshot(&self) -> Result<crate::protocol::ServingSchemaSnapshot, String> {
        self.gateway.serving_schema_snapshot()
    }

    fn lan_access(&self) -> Option<cerulion_core::LanAccess> {
        self.gateway.lan_access()
    }

    fn egress_stats(&self) -> Option<Vec<cerulion_core::EgressTopicStats>> {
        self.gateway.egress_stats()
    }

    fn robot_startup_error(&self) -> Option<String> {
        self.gateway.robot_startup_error()
    }
}

impl Drop for SupervisedEgressPlane {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::testing::IsolatedRoot;
    use cerulion_core::{TransportConfig, TransportManager};

    #[test]
    fn lifecycle_conditions_rearm_independently_after_fence_recovery() {
        use cerulion_core::transport::failure_regime_latch::RegimeDecision;
        let mut latches = LifecycleLatches::new();
        assert_eq!(latches.fence_failed(), RegimeDecision::Loud);
        assert_eq!(
            latches.fence_failed(),
            RegimeDecision::Suppressed { suppressed: 1 }
        );
        assert_eq!(latches.start_failed(), RegimeDecision::Loud);
        assert_eq!(
            latches.fence_failed(),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(latches.fence_ok(), Some(2));
        assert_eq!(latches.fence_ok(), None);
        assert_eq!(latches.fence_failed(), RegimeDecision::Loud);
        assert_eq!(
            latches.start_failed(),
            RegimeDecision::Suppressed { suppressed: 1 }
        );
        assert_eq!(latches.started(), Some(1));
        assert_eq!(latches.start_failed(), RegimeDecision::Loud);
        assert_eq!(latches.fence_ok(), None);
        assert_eq!(latches.fence_failed(), RegimeDecision::Loud);
        // A start recovery leaves the open fence regime open.
        assert_eq!(
            latches.fence_failed(),
            RegimeDecision::Suppressed { suppressed: 1 }
        );
        assert_eq!(latches.started(), None);
        assert_eq!(
            latches.fence_failed(),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(latches.fence_ok(), Some(2));
        // Totals count every failure and never reset on recovery.
        assert_eq!(latches.start.total_failures(), 3);
        assert_eq!(latches.fence.total_failures(), 7);
    }

    #[test]
    fn a_supervised_gateway_reports_registration_failure_through_the_egress_trait() {
        let root = IsolatedRoot::mint("startup-status");
        let manager =
            TransportManager::init_for_test(TransportConfig::default(), root.config()).unwrap();
        let gateway = Arc::new(GatewayEgressPlane::new(Arc::clone(&manager)));
        let plane: Arc<dyn EgressPlane> = Arc::new(SupervisedEgressPlane::new(
            Arc::clone(&gateway),
            Arc::new(crate::mirror::GatewayMirrorPlane::new(manager)),
            false,
        ));
        let reason = "automatic robot registration: the robot state directory must be owner-only; run chmod 700 <dir>";
        gateway.set_robot_startup_error(Some(reason.into()));
        assert_eq!(plane.robot_startup_error().as_deref(), Some(reason));
        gateway.set_robot_startup_error(None);
        assert_eq!(plane.robot_startup_error(), None);
    }

    #[test]
    fn idle_decision_preserves_outbound_readers_and_starts_queued_registrations() {
        for (stopped, registered, server, expected) in [
            (false, false, false, IdleAction::Wait),
            (false, false, true, IdleAction::Reap),
            (false, true, false, IdleAction::Start),
            (false, true, true, IdleAction::Wait),
            (true, false, false, IdleAction::Stop),
            (true, false, true, IdleAction::Stop),
            (true, true, false, IdleAction::Stop),
            (true, true, true, IdleAction::Stop),
        ] {
            assert_eq!(idle_action(stopped, registered, server, false), expected);
            let held = if expected == IdleAction::Reap {
                IdleAction::Wait
            } else {
                expected
            };
            assert_eq!(idle_action(stopped, registered, server, true), held);
        }
    }

    fn state(holds: &[(EgressId, Instant)], registered: bool, standing_requested: bool) -> State {
        State {
            registrations: if registered {
                BTreeSet::from([9])
            } else {
                BTreeSet::new()
            },
            holds: holds.iter().copied().collect(),
            phase: Phase::Idle,
            standing_requested,
            stopped: false,
        }
    }

    /// A serving hold alone wants a server until its deadline, and only until
    /// it: the deadline itself no longer counts. Any one live hold among
    /// several keeps the server, whichever connection holds it. Each row is
    /// (holds, egress registered, standing, server running, outbound readers,
    /// expected action) at `now`.
    #[test]
    fn a_serving_hold_wants_a_server_until_its_deadline_and_then_lets_it_reap() {
        let now = Instant::now();
        let live_at = now + Duration::from_secs(60);
        let expired_at = now - Duration::from_millis(1);
        let live = [(4, live_at)];
        let expired = [(4, expired_at)];
        let at_deadline = [(4, now)];
        let expired_then_live = [(1, expired_at), (2, live_at)];
        let live_then_expired = [(1, live_at), (2, expired_at)];
        let expired_and_at_deadline = [(1, expired_at), (2, now)];
        type Row<'a> = (
            &'a [(EgressId, Instant)],
            bool,
            bool,
            bool,
            bool,
            IdleAction,
        );
        let rows: [Row; 18] = [
            (&live, false, false, false, false, IdleAction::Start),
            (&live, false, false, true, false, IdleAction::Wait),
            (&live, false, false, true, true, IdleAction::Wait),
            (&live, false, false, false, true, IdleAction::Start),
            (&expired, false, false, true, false, IdleAction::Reap),
            (&at_deadline, false, false, true, false, IdleAction::Reap),
            (&expired, false, false, false, false, IdleAction::Wait),
            (&expired, false, false, true, true, IdleAction::Wait),
            (&expired, true, false, true, false, IdleAction::Wait),
            (&expired, false, true, true, false, IdleAction::Wait),
            (&[], false, false, true, false, IdleAction::Reap),
            (&[], true, false, false, false, IdleAction::Start),
            (&[], false, true, false, false, IdleAction::Start),
            (
                &expired_then_live,
                false,
                false,
                true,
                false,
                IdleAction::Wait,
            ),
            (
                &live_then_expired,
                false,
                false,
                true,
                false,
                IdleAction::Wait,
            ),
            (
                &expired_then_live,
                false,
                false,
                false,
                false,
                IdleAction::Start,
            ),
            (
                &live_then_expired,
                false,
                false,
                false,
                false,
                IdleAction::Start,
            ),
            (
                &expired_and_at_deadline,
                false,
                false,
                true,
                false,
                IdleAction::Reap,
            ),
        ];
        for (holds, registered, standing, server, readers, expected) in rows {
            let current = state(holds, registered, standing);
            assert_eq!(
                idle_action(false, current.wanted(now), server, readers),
                expected,
                "holds={holds:?} registered={registered} standing={standing} \
                 server={server} readers={readers}"
            );
        }
    }

    #[test]
    fn expiring_holds_removes_exactly_those_at_or_past_their_deadline() {
        let now = Instant::now();
        let mut current = state(
            &[
                (1, now - Duration::from_secs(1)),
                (2, now + Duration::from_secs(1)),
                (3, now),
            ],
            false,
            false,
        );
        assert_eq!(current.expire_holds(now), vec![1, 3]);
        assert_eq!(
            current.holds,
            BTreeMap::from([(2, now + Duration::from_secs(1))])
        );
        assert!(current.wanted(now), "the live hold still wants a server");
        assert!(
            current.expire_holds(now).is_empty(),
            "nothing left to expire"
        );
    }

    /// A second hold on a connection replaces the first, deadline included,
    /// whether it shortens or extends the hold; other connections' holds stay.
    #[test]
    fn a_repeated_hold_on_one_connection_replaces_its_deadline_both_ways() {
        let now = Instant::now();
        let short = now + Duration::from_secs(1);
        let long = now + Duration::from_secs(60);
        let mut current = state(&[(8, short)], false, false);
        current.hold(5, short);
        current.hold(5, long);
        assert_eq!(current.holds, BTreeMap::from([(5, long), (8, short)]));
        current.hold(5, short);
        assert_eq!(current.holds, BTreeMap::from([(5, short), (8, short)]));
    }

    /// The re-check under the exclusive fence lets the reap go ahead only
    /// when nothing wants a server: a live hold, among others or alone, a
    /// registration, standing serving, a stopping daemon and outbound readers
    /// each stop it. Each row is (holds, registered, standing, stopped,
    /// readers, may reap) at `now`.
    #[test]
    fn the_fenced_reap_recheck_refuses_while_a_live_hold_or_other_work_remains() {
        let now = Instant::now();
        let live = [(4, now + Duration::from_secs(60))];
        let expired = [(4, now - Duration::from_millis(1))];
        let expired_then_live = [
            (1, now - Duration::from_millis(1)),
            (2, now + Duration::from_secs(60)),
        ];
        type Row<'a> = (&'a [(EgressId, Instant)], bool, bool, bool, bool, bool);
        let rows: [Row; 9] = [
            (&[], false, false, false, false, true),
            (&expired, false, false, false, false, true),
            (&[(4, now)], false, false, false, false, true),
            (&live, false, false, false, false, false),
            (&expired_then_live, false, false, false, false, false),
            (&expired, true, false, false, false, false),
            (&expired, false, true, false, false, false),
            (&expired, false, false, true, false, false),
            (&expired, false, false, false, true, false),
        ];
        for (holds, registered, standing, stopped, readers, expected) in rows {
            let mut current = state(holds, registered, standing);
            current.stopped = stopped;
            assert_eq!(
                current.may_reap(now, readers),
                expected,
                "holds={holds:?} registered={registered} standing={standing} \
                 stopped={stopped} readers={readers}"
            );
        }
    }

    fn idle_plane(prefix: &str) -> (IsolatedRoot, SupervisedEgressPlane) {
        let root = IsolatedRoot::mint(prefix);
        let manager =
            TransportManager::init_for_test(TransportConfig::default(), root.config()).unwrap();
        let plane = SupervisedEgressPlane::new(
            Arc::new(GatewayEgressPlane::new_without_mdns_for_test(Arc::clone(
                &manager,
            ))),
            Arc::new(crate::mirror::GatewayMirrorPlane::new(manager)),
            false,
        );
        (root, plane)
    }

    /// A state root the remote server cannot use is refused, each with its
    /// class's refusal text (missing, not absolute or with traversal, not
    /// UTF-8), and a usable one is reported verbatim.
    #[test]
    fn an_unusable_state_root_is_refused_with_its_class_text_and_a_usable_one_reported() {
        use std::os::unix::ffi::OsStrExt;
        let missing = "no robot state root is available, so cerulion-netd cannot serve remote \
                       access; set CERULION_STATE_ROOT, CERULION_HOME or HOME and restart \
                       cerulion-netd";
        let relative = "the robot state root must be an absolute path with no '.' or '..' \
                        components; set CERULION_STATE_ROOT to such a path and restart \
                        cerulion-netd";
        let not_utf8 = "the robot state root is not a UTF-8 path, so cerulion-netd cannot \
                        report it; set CERULION_STATE_ROOT to a UTF-8 path and restart \
                        cerulion-netd";
        let rows: [(Option<PathBuf>, &str); 6] = [
            (None, missing),
            (Some(PathBuf::from("robot-state")), relative),
            (Some(PathBuf::new()), relative),
            (Some(PathBuf::from("/srv/../robot-state")), relative),
            (Some(PathBuf::from("./robot-state")), relative),
            (
                Some(PathBuf::from(std::ffi::OsStr::from_bytes(
                    b"/srv/robot-\xff",
                ))),
                not_utf8,
            ),
        ];
        for (root, refusal) in rows {
            let shown = format!("{root:?}");
            let err = serving_root(root).expect_err("an unusable root is refused");
            assert_eq!(err.to_string(), refusal, "root={shown}");
            assert!(std::error::Error::source(&err).is_none());
        }
        assert_eq!(
            serving_root(Some(PathBuf::from("/srv/robot-state"))).unwrap(),
            "/srv/robot-state"
        );
    }

    /// A hold that reaches a stopping daemon is recorded, refused by the
    /// worker start, and rolled back, so the plane is left holding nothing.
    #[test]
    fn a_serving_hold_after_shutdown_is_rolled_back_and_starts_no_worker() {
        let (_root, plane) = idle_plane("h7");
        plane.shutdown();
        let err = plane
            .hold_serving(5, Instant::now() + Duration::from_secs(60))
            .expect_err("a stopping daemon holds nothing");
        assert_eq!(
            err.to_string(),
            "remote server startup failed: network daemon is shutting down"
        );
        assert!(plane.state.lock().unwrap().holds.is_empty());
        assert!(plane.worker.lock().unwrap().is_none());
    }

    /// A granted hold wakes the lifecycle worker, which drops the hold once
    /// its deadline has passed and starts no server for it.
    #[test]
    fn the_lifecycle_worker_drops_a_hold_whose_deadline_has_passed() {
        let (_root, plane) = idle_plane("h6");
        let lapsed = Instant::now()
            .checked_sub(Duration::from_millis(1))
            .expect("the monotonic clock is past one millisecond");
        plane
            .hold_serving(5, lapsed)
            .expect("a running daemon takes the hold");
        assert!(
            plane.worker.lock().unwrap().is_some(),
            "the hold woke the worker"
        );
        let bound = Instant::now() + Duration::from_secs(5);
        while !plane.state.lock().unwrap().holds.is_empty() {
            assert!(
                Instant::now() < bound,
                "the worker never dropped the lapsed hold"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(plane.state.lock().unwrap().phase, Phase::Idle);
        plane.shutdown();
    }

    #[test]
    fn release_drops_only_its_connections_hold_and_shutdown_drops_every_hold() {
        let (_root, plane) = idle_plane("h3");
        let until = Instant::now() + Duration::from_secs(60);
        plane.state.lock().unwrap().holds = BTreeMap::from([(1, until), (2, until)]);
        plane.release_serving(1);
        plane.release_serving(7);
        assert_eq!(
            plane.state.lock().unwrap().holds,
            BTreeMap::from([(2, until)])
        );
        plane.shutdown();
        assert!(plane.state.lock().unwrap().holds.is_empty());
    }

    #[test]
    fn shutdown_reserves_server_grace_before_pending_registration_claims_endpoint() {
        let root = IsolatedRoot::mint("r2");
        let manager =
            TransportManager::init_for_test(TransportConfig::default(), root.config()).unwrap();
        let plane = Arc::new(SupervisedEgressPlane::new(
            Arc::new(GatewayEgressPlane::new(Arc::clone(&manager))),
            Arc::new(crate::mirror::GatewayMirrorPlane::new(manager)),
            false,
        ));
        // Brief registration bookkeeping must not delay the cached shutdown budget,
        // including when endpoint ownership has not yet transferred.
        let registration = plane.state.lock().unwrap();
        assert!(!plane.holds_serving_endpoint());
        let (send, receive) = std::sync::mpsc::channel();
        let pending = Arc::clone(&plane);
        let reader = std::thread::spawn(move || send.send(pending.stop_grace()).unwrap());
        let budget = receive.recv_timeout(Duration::from_secs(1));
        drop(registration);
        reader.join().unwrap();
        assert!(budget.unwrap() >= crate::robot_supervisor::remoted_stop_grace());
    }
}
