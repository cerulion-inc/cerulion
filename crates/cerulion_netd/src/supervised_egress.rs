// SPDX-License-Identifier: AGPL-3.0-only
//! Connection-scoped ownership of the machine's authenticated remote server.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

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

struct State {
    registrations: BTreeSet<EgressId>,
    phase: Phase,
    standing_requested: bool,
    stopped: bool,
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

/// First successful egress wakes one lifecycle worker. Registrations never wait
/// for endpoint transfer, bootstrap or graceful child teardown. Idle teardown
/// waits for outbound readers, admitted work, owed delivery and the e-stop latch.
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
                reason: "network daemon is shutting down".into(),
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

fn lifecycle_worker(
    state: Arc<Mutex<State>>,
    wake: Arc<Condvar>,
    gateway: Arc<GatewayEgressPlane>,
    mirror: Arc<dyn MirrorPlane>,
    held: Arc<AtomicBool>,
) {
    use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
    let mut supervisor: Option<RobotSupervisor> = None;
    let mut failures = FailureRegimeLatch::new();
    loop {
        // The controller may be busy with WAN I/O. Never hold lifecycle state
        // while sampling it: registrations still get an immediate reply.
        let readers = mirror.holds_outbound_readers();
        let mut current = state.lock().unwrap_or_else(|e| e.into_inner());
        let wanted = current.standing_requested || !current.registrations.is_empty();
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
                        if let Some(suppressed) = failures.on_success() {
                            tracing::info!(suppressed, "remote serving lifecycle recovered");
                        }
                        continue;
                    }
                    Err(error) => {
                        state.lock().unwrap_or_else(|e| e.into_inner()).phase = Phase::Idle;
                        match failures.on_failure() {
                            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                                tracing::warn!(error = %error, total_failures = failures.total_failures(), "remote serving supervisor could not start; retrying")
                            }
                            RegimeDecision::Suppressed { .. } => {
                                tracing::debug!(error = %error, total_failures = failures.total_failures(), "remote serving supervisor start still unavailable")
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
                match fence {
                    Ok(Some(fence)) => {
                        // Outbound Ensure also takes this native fence in shared
                        // mode. Once exclusive, no new reader can race the stop.
                        let readers = mirror.holds_outbound_readers();
                        let mut current = state.lock().unwrap_or_else(|e| e.into_inner());
                        if !current.stopped
                            && current.registrations.is_empty()
                            && !current.standing_requested
                            && !readers
                        {
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
                    Err(error) => match failures.on_failure() {
                        RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                            tracing::warn!(error = %error, total_failures = failures.total_failures(), "remote serving idle fence unavailable; retrying")
                        }
                        RegimeDecision::Suppressed { .. } => {
                            tracing::debug!(error = %error, total_failures = failures.total_failures(), "remote serving idle fence remains unavailable")
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
                reason: "network daemon is shutting down".into(),
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
                reason: "network daemon is shutting down".into(),
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

    fn serving_schema_snapshot(&self) -> Result<crate::protocol::ServingSchemaSnapshot, String> {
        self.gateway.serving_schema_snapshot()
    }

    fn lan_access(&self) -> Option<cerulion_core::LanAccess> {
        self.gateway.lan_access()
    }

    fn egress_stats(&self) -> Option<Vec<cerulion_core::EgressTopicStats>> {
        self.gateway.egress_stats()
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
